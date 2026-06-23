# CAN Bus Failure Modes

This document describes the three failure modes implemented in `can_fault_monitor`,
their real-world causes, their signatures in `candump` output, the recovery strategy
employed by the `CANHealthMonitor` node, and what vcan0 *cannot* reproduce about each
mode on physical hardware.

Written before any code. Every detection threshold and recovery decision in the
implementation traces back to the analysis here.

---

## Failure Mode 1 — NODE_DROPOUT

### What causes it on real hardware

A CAN node goes silent when its microcontroller loses power, its firmware crashes and
halts, or its CAN transceiver enters thermal shutdown. In field robotics, NODE_DROPOUT
is the most common failure: a motor driver board loses its 24 V rail during a hard
impact, or a sensor node's firmware panics on a null pointer and the watchdog doesn't
fire in time.

NODE_DROPOUT is *silent* — the bus sees no error frames, no bus-off event, no
indication of trouble. The absent node simply stops transmitting, and every subscriber
waiting on its messages times out. This makes it the hardest fault to detect quickly
without an explicit heartbeat protocol.

### `candump` signature

Under normal operation, node 0x100 transmits at 50 Hz:
```
vcan0  100   [8]  01 02 03 04 05 06 07 08
vcan0  100   [8]  01 02 03 04 05 06 07 09
```

After NODE_DROPOUT is injected, traffic on ID 0x100 stops entirely:
```
vcan0  100   [8]  01 02 03 04 05 06 07 0B   ← last frame before dropout
                                             ← silence on 0x100
vcan0  200   [4]  AA BB CC DD               ← other nodes continue normally
```

### Recovery strategy

The `CANHealthMonitor` maintains a per-CAN-ID deadline timer. Each received frame
resets the timer. When the timer expires (default 100 ms = 5 missed frames at 50 Hz),
the monitor publishes `CanHealthStatus` with state `DEGRADED` and fault type
`NODE_DROPOUT`. Recovery uses exponential backoff (100 ms, 200 ms, 400 ms) to avoid
thundering herd on a flapping node.

Detection latency: `t_DEGRADED_publish − t_last_good_frame`

### What vcan0 cannot reproduce

On physical CAN, a node that goes bus-off (TEC > 255) also stops transmitting but
emits a bus-off sequence detectable in other nodes' error counters. A true power-loss
dropout produces neither error frames nor a bus-off sequence. vcan0 reproduces the
silence correctly but has no TEC/REC counters, no error confinement states, and no
bus-off sequences.

---

## Failure Mode 2 — FRAME_CORRUPTION

### What causes it on real hardware

CAN frames are corrupted by electromagnetic interference (EMI) from motor drive PWM
switching, ground loops between chassis and logic ground, or inadequate termination
resistance on a long bus stub. ISO 11898's bit-stuffing and CRC mechanism causes
receiving nodes to detect and discard corrupted frames. The transmitting node,
detecting no ACK, retransmits — incrementing its Transmit Error Counter (TEC).

In field robots, FRAME_CORRUPTION typically appears as bursty — correlated with motor
acceleration events — rather than random. A 120 Ω termination mismatch on a 4-meter
harness can cause 10–30% frame loss at 1 Mbit/s.

### `candump` signature

FRAME_CORRUPTION is simulated by sending frames with illegal DLC=9 (max per ISO 11898
is 8). The subscriber's DLC validation check catches and flags these frames:
```
vcan0  100   [8]  01 02 03 04 05 06 07 08   ← valid
vcan0  100   [9]  DE AD BE EF FF FF FF FF   ← injected: DLC=9 (invalid)
vcan0  100   [8]  01 02 03 04 05 06 07 09   ← valid (resumes)
```

### Recovery strategy

The `CANHealthMonitor` tracks a rolling window of valid-to-invalid frame ratio per CAN
ID over the last 500 ms. When the invalid frame rate exceeds 20% of received frames,
the monitor transitions to `DEGRADED`. Recovery is passive: waits for the ratio to
drop below 5% over a 1-second window before returning to `NOMINAL`.

Detection latency: `t_DEGRADED_publish − t_first_invalid_frame`

### What vcan0 cannot reproduce

On physical CAN, bit errors trigger hardware CRC checking and cause the transmitting
node's TEC to increment. If TEC reaches 128, the node enters "error passive" mode; at
255, it goes bus-off. vcan0 has no TEC/REC counters. Our DLC-based corruption
simulation is detectable at the application layer but produces no change in
protocol-layer error counters. On physical hardware, use `ip link set can0
berr-reporting on` and listen for `CAN_ERR_MASK` frames for hardware-layer detection.

---

## Failure Mode 3 — BUS_FLOOD

### What causes it on real hardware

A runaway node transmits at maximum rate, consuming bus bandwidth and starving
higher-priority (lower CAN ID) nodes of transmission slots. This happens when firmware
enters an infinite loop calling the CAN transmit function continuously, or when a CAN
controller's TX queue fills and the firmware mishandles the full-queue interrupt.

### `candump` signature

After BUS_FLOOD injected on ID 0x7FF:
```
vcan0  7FF   [8]  FF FF FF FF FF FF FF FF
vcan0  7FF   [8]  FF FF FF FF FF FF FF FF   ← flooding at ~2000 Hz
vcan0  100   [8]  ...                       ← still arriving but with jitter
```

### Recovery strategy

The `CANHealthMonitor` measures instantaneous frame rate per CAN ID over a 100 ms
sliding window. When any CAN ID exceeds 500 frames/sec, the monitor transitions to
`DEGRADED` and logs the offending CAN ID. Recovery action publishes the flooding ID on
`/can_health/flood_source` so a supervisor node can isolate the runaway node.

Detection latency: `t_DEGRADED_publish − t_first_frame_exceeding_threshold`

### What vcan0 cannot reproduce

vcan0 is a software loopback with no bandwidth limit. On physical CAN at 1 Mbit/s, a
standard 8-byte frame takes ~130 µs (including SOF, arbitration, control, data, CRC,
ACK, EOF, IFS) — the bus physically cannot exceed ~7,700 frames/sec. On vcan0,
flooding at 100,000 frames/sec is possible and imposes no timing constraint on other
frames. Our flood simulation uses 2,000 frames/sec to mimic the signature without
saturating the kernel scheduler.

---

## Hardware Delta Summary

| Property | vcan0 (this project) | Physical CAN |
|---|---|---|
| Bandwidth limit | None (memory-bus speed) | 1 Mbit/s (~7,700 frames/s for 8-byte) |
| TEC/REC counters | Not present | Per ISO 11898-1 §6.15 |
| Bus-off state machine | Not present | Triggered at TEC > 255 |
| Error frames | Not generated | Broadcast by all nodes on bit error |
| Bit stuffing / CRC | Bypassed | Enforced by CAN controller hardware |
| Frame timing jitter | ~0 µs | 50–500 µs depending on bus load |
| ACK mechanism | Loopback (always ACK'd) | Requires at least one physical receiver |

**Code portability:** swap `"vcan0"` for `"can0"` in `config/params.yaml`.
Enable berr-reporting: `ip link set can0 type can berr-reporting on`.
Detection logic is interface-agnostic.
