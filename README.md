# can_fault_monitor

**Fault injection and recovery latency benchmarking for CAN bus robot systems.**  
Built with ROS 2 Humble (C++17) + Linux SocketCAN. Methodology portable to physical hardware — swap one parameter, zero code changes.

---

## Results

All three fault modes injected and measured on vcan0 (Ubuntu 22.04, VirtualBox):

| Fault type | Detection latency | Recovery latency | Notes |
|---|---|---|---|
| NODE_DROPOUT | **115 ms** | **1903 ms** | 5 missed frames at 50 Hz + exponential backoff |
| FRAME_CORRUPTION | **< 20 ms** | **~600 ms** | Payload signature detected via 500 ms sliding window |
| BUS_FLOOD | **< 20 ms** | **~120 ms** | 700 fps peak detected against 500 fps threshold |

### Bus Health Timeline

![Bus Health Timeline](results/bus_health_timeline.png)

- **Top**: frame rate spikes to ~700 fps during BUS_FLOOD injection, crossing the 500 fps detection threshold
- **Bottom**: invalid frame ratio spikes to 25% during FRAME_CORRUPTION, crossing the 20% threshold. Both return to 0 on recovery.

### Detection & Recovery Latency

![Recovery Latency](results/recovery_latency.png)

NODE_DROPOUT detection at 115 ms confirms the 5-missed-frame deadline timer (100 ms at 50 Hz + 15 ms eval jitter). Recovery at 1903 ms reflects the exponential backoff sequence (100 → 200 → 400 ms probes before node resumes).

---

## The problem this solves

Field robots lose communication with actuators in unstructured environments. A motor driver board loses its rail after a hard impact. A sensor node's firmware panics and stops transmitting. A runaway controller floods the bus and starves safety-critical nodes of their transmission slots. In all three cases, the only indication is silence or noise on a shared CAN bus — and the robot needs to detect, classify, and recover faster than the mechanical consequences become irreversible.

This project implements a complete fault-detection stack: a publisher simulating an actuator node, a fault injector arming real failure modes, and a health monitor measuring exactly how long detection and recovery take for each fault class.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         vcan0 (Linux SocketCAN)                 │
│  ┌──────────────────┐    CAN frames    ┌──────────────────────┐ │
│  │  can_publisher   │ ───────────────► │  can_subscriber      │ │
│  │  50 Hz heartbeat │ ◄── dropout cmd  │  DLC validation      │ │
│  │  seq counter     │   (ROS 2 topic)  │  seq counter check   │ │
│  └──────────────────┘                  └──────────┬───────────┘ │
│                                                   │             │
│  ┌──────────────────┐         /can_bus/frames     ▼             │
│  │  fault_injector  │ ──corrupt/flood──► ┌──────────────────┐  │
│  │  /inject_fault   │                    │ can_health_monitor│  │
│  │  (ROS 2 service) │──── /events ──────►│ NODE_DROPOUT     │  │
│  └──────────────────┘                    │ FRAME_CORRUPTION  │  │
│                                          │ BUS_FLOOD         │  │
│                                          │ → /can_health/    │  │
│                                          │   status          │  │
│                                          │ → results/        │  │
│                                          │   metrics.csv     │  │
│                                          └──────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Fault injection

### `NODE_DROPOUT`
Commands `can_publisher` to stop transmitting, reproducing the silence of a powered-off microcontroller. Detected via per-CAN-ID heartbeat deadline timer (100 ms = 5 missed frames at 50 Hz). Recovery uses exponential backoff (100/200/400 ms) to prevent thundering-herd restarts on a flapping node.

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "NODE_DROPOUT", target_can_id: 256, duration_sec: 2.0}'
```

### `FRAME_CORRUPTION`
Injects frames with a corruption signature (`0xDE AD BE EF`) detectable at the application layer. Detected via 500 ms sliding window — triggers DEGRADED when invalid frame ratio exceeds 20%. On physical CAN hardware, corruption arrives via hardware error frames (`CAN_ERR_FLAG`); the detection logic is identical.

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "FRAME_CORRUPTION", target_can_id: 256, duration_sec: 3.0}'
```

### `BUS_FLOOD`
Transmits 2,000 frames/sec on CAN ID 0x7FF (lowest priority). Detected via 100 ms per-ID frame rate window — triggers DEGRADED when any ID exceeds 500 fps.

```bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "BUS_FLOOD", target_can_id: 2047, duration_sec: 2.0}'
```

---

## Hardware Delta

| Property | vcan0 (this project) | Physical CAN |
|---|---|---|
| Bandwidth limit | None | 1 Mbit/s → ~7,700 frames/s max |
| TEC/REC error counters | Not present | Per ISO 11898-1 §6.15 |
| Bus-off state machine | Not present | Triggered at TEC > 255 |
| Hardware error frames | Not generated | Broadcast on bit error |
| Frame timing jitter | ~0 µs | 50–500 µs at high load |
| DLC enforcement | Kernel rejects DLC > 8 | Hardware CRC catches violations |

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

# 3. Launch all 4 nodes
ros2 launch can_fault_monitor can_demo.launch.py

# 4. Inject faults (second terminal)
source ~/ros2_ws/install/setup.bash
ros2 service call /fault_injector/inject_fault \
  can_fault_monitor/srv/InjectFault \
  '{fault_type: "NODE_DROPOUT", target_can_id: 256, duration_sec: 2.0}'

# 5. Monitor raw CAN frames
candump vcan0

# 6. Generate plots
python3 scripts/plot_recovery_latency.py
```

---

## Why this design

**Raw `socket(PF_CAN, SOCK_RAW, CAN_RAW)`** — direct access to `can_frame` structs including DLC and flags. BCM sockets filter on content; raw sockets see everything including protocol-layer anomalies.

**Non-blocking `O_NONBLOCK`** — subscriber polls inside a ROS 2 wall-timer callback. Blocking `read()` would stall the single-threaded executor and miss deadlines on other nodes.

**Custom `CanFrame.msg`** instead of `std_msgs/String` — typed fields for CAN ID, DLC, and error flags. Downstream nodes read structured data, not strings.

**Heartbeat deadline at 5 missed frames (100 ms)** — tolerates scheduler jitter on a non-RT kernel while detecting power-loss failures well under a 500 ms safety action timeout.

**Exponential backoff on recovery (100/200/400 ms, max 2 s)** — prevents thundering-herd restart storms when a flapping node recovers briefly then drops again. Measured recovery: 1903 ms median.

**Sliding window fault detection** — 500 ms for corruption (bursty by nature), 100 ms for flood (needs fast response). Windows sized to tolerate transient noise without false positives.

---

## Project structure

```
can_fault_monitor/
├── docs/failure_modes.md          ← written before code; all decisions traced here
├── include/can_fault_monitor/
│   ├── can_socket.hpp             ← RAII SocketCAN wrapper (raw syscalls)
│   └── fault_types.hpp            ← FaultType and BusState enums
├── src/
│   ├── can_socket.cpp             ← socket(PF_CAN), SIOCGIFINDEX, bind, O_NONBLOCK
│   ├── can_publisher_node.cpp     ← 50 Hz heartbeat; responds to NODE_DROPOUT command
│   ├── can_subscriber_node.cpp    ← corruption signature detection; seq counter gaps
│   ├── fault_injector_node.cpp    ← /inject_fault service; 3 fault modes
│   └── can_health_monitor_node.cpp← deadline timer, sliding windows, CSV metrics
├── msg/                           ← custom typed messages (not std_msgs/String)
├── srv/InjectFault.srv
├── config/params.yaml             ← all thresholds documented with units
├── launch/can_demo.launch.py
├── results/                       ← benchmark plots from actual run
│   ├── recovery_latency.png
│   └── bus_health_timeline.png
└── scripts/plot_recovery_latency.py
```
