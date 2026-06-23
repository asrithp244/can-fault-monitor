# can_fault_monitor

**Fault injection and recovery latency benchmarking for CAN bus robot systems.**  
Methodology portable to physical hardware — swap one parameter, no code changes.

---

## The problem this solves

Field robots lose communication with actuators in unstructured environments. A motor
driver board loses its rail after a hard impact. A sensor node's firmware panics and
stops transmitting. A runaway controller floods the bus and starves safety-critical
nodes of their transmission slots. In all three cases, the *only* indication is
silence or noise on a shared CAN bus — and the robot needs to detect, classify, and
recover from the failure faster than the mechanical consequences become irreversible.

This project implements a complete fault-detection stack for a CAN bus robot system:
a publisher that simulates an actuator node, a fault injector that arms real failure
modes, and a health monitor that measures exactly how long detection and recovery take
for each fault class.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         vcan0 (Linux SocketCAN)                 │
│  ┌──────────────────┐    CAN frames    ┌──────────────────────┐ │
│  │  can_publisher   │ ───────────────► │  can_subscriber      │ │
│  │  (simulates MCU) │                  │  (DLC validation,    │ │
│  │  50 Hz heartbeat │ ◄── dropout cmd  │   seq counter check) │ │
│  └──────────────────┘   (ROS 2 topic)  └──────────┬───────────┘ │
│                                                   │             │
│  ┌──────────────────┐              /can_bus/frames │             │
│  │  fault_injector  │ ──corrupt/flood─►            ▼             │
│  │  /inject_fault   │       ┌──────────────────────┐            │
│  │  (ROS 2 service) │       │  can_health_monitor  │            │
│  │  /events pub     │──────►│  NODE_DROPOUT detect │            │
│  └──────────────────┘       │  CORRUPTION detect   │            │
│                             │  BUS_FLOOD detect    │            │
│                             │  → /can_health/status│            │
│                             │  → results/metrics.csv            │
│                             └──────────────────────┘            │
└─────────────────────────────────────────────────────────────────┘

Protocol boundary: CAN frames cross the SocketCAN layer at can_subscriber.
Above that layer, everything is typed ROS 2 messages. This mirrors the
real architecture in field robots: MCUs speak CAN; the host runs ROS 2.
```

---

## Fault injection methodology

### `NODE_DROPOUT`
Commands `can_publisher` to stop transmitting, reproducing the silence of a powered-off
microcontroller. Detected via per-CAN-ID heartbeat deadline timer (100 ms = 5 missed frames).

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "NODE_DROPOUT", target_can_id: 256, duration_sec: 2.0}'
```

### `FRAME_CORRUPTION`
Injects frames with DLC=9 (illegal per ISO 11898). Detected via 500 ms sliding window
invalid-frame ratio. Triggers DEGRADED when ratio > 20%.

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "FRAME_CORRUPTION", target_can_id: 256, duration_sec: 3.0}'
```

### `BUS_FLOOD`
Transmits 2,000 frames/sec on CAN ID 0x7FF (lowest priority). Detected via 100 ms
per-ID frame rate window. Triggers DEGRADED when any ID exceeds 500 frames/sec.

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "BUS_FLOOD", target_can_id: 2047, duration_sec: 2.0}'
```

---

## Results

| Fault type | Detection latency | Recovery latency |
|---|---|---|
| NODE_DROPOUT | ~105 ms | ~300 ms (3 backoff retries) |
| FRAME_CORRUPTION | ~30 ms | ~600 ms (passive window drain) |
| BUS_FLOOD | ~5 ms | ~120 ms |

Generate plots after running fault cycles:
```bash
python3 scripts/plot_recovery_latency.py
```

---

## Hardware Delta

| Property | vcan0 (this project) | Physical CAN |
|---|---|---|
| Bandwidth limit | None | 1 Mbit/s → ~7,700 frames/s |
| TEC/REC error counters | Not present | Per ISO 11898-1 §6.15 |
| Bus-off state machine | Not present | Triggered at TEC > 255 |
| Hardware error frames | Not generated | Broadcast on bit error |
| Frame timing jitter | ~0 µs | 50–500 µs at high load |

**To run on physical hardware:** change `interface: "can0"` in `config/params.yaml`. Zero code changes.

---

## Build & run

```bash
# 1. Set up virtual CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 2. Build
mkdir -p ~/ros2_ws/src
cp -r can_fault_monitor ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select can_fault_monitor
source install/setup.bash

# 3. Launch
ros2 launch can_fault_monitor can_demo.launch.py

# 4. Inject a fault (second terminal)
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "NODE_DROPOUT", target_can_id: 256, duration_sec: 2.0}'

# 5. Watch live
candump vcan0

# 6. Plot results
python3 scripts/plot_recovery_latency.py
```

---

## Project structure

```
can_fault_monitor/
├── docs/failure_modes.md          ← written before any code; all decisions trace here
├── include/can_fault_monitor/
│   ├── can_socket.hpp             ← RAII SocketCAN wrapper (raw syscalls)
│   └── fault_types.hpp            ← FaultType and BusState enums
├── src/
│   ├── can_socket.cpp             ← socket(PF_CAN), SIOCGIFINDEX, bind, O_NONBLOCK
│   ├── can_publisher_node.cpp     ← 50 Hz heartbeat; responds to NODE_DROPOUT
│   ├── can_subscriber_node.cpp    ← DLC validation; seq counter drop detection
│   ├── fault_injector_node.cpp    ← /inject_fault service; 3 fault modes
│   └── can_health_monitor_node.cpp← deadline timer, sliding windows, latency metrics
├── msg/                           ← custom typed messages (not std_msgs/String)
├── srv/InjectFault.srv
├── config/params.yaml             ← all thresholds with units and valid ranges
├── launch/can_demo.launch.py
└── scripts/plot_recovery_latency.py  ← generates results/ plots
```

---

## Why this design

- **Raw `socket(PF_CAN, SOCK_RAW, CAN_RAW)`** instead of a library — direct access to `can_frame.can_dlc` for FRAME_CORRUPTION detection. BCM sockets can't show protocol violations.
- **Non-blocking `O_NONBLOCK`** — subscriber runs inside ROS 2 single-threaded executor; blocking `read()` would stall the entire node graph.
- **Custom `CanFrame.msg`** instead of `std_msgs/String` — typed fields for CAN ID, DLC, and flags. Downstream nodes read structured data, not strings.
- **Heartbeat deadline at 5 missed frames** — tolerates scheduler jitter on non-RT kernel while detecting power-loss failures in under 120 ms total.
- **Exponential backoff on recovery (100/200/400 ms)** — prevents thundering-herd restart storms when a flapping node recovers briefly then drops again.
