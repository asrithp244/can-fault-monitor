builtin_interfaces/Time stamp
uint8   bus_state              # 0=NOMINAL 1=DEGRADED 2=RECOVERY
string  active_fault           # "NONE" | "NODE_DROPOUT" | "FRAME_CORRUPTION" | "BUS_FLOOD"
uint32  affected_can_id
float64 detection_latency_ms
float64 recovery_latency_ms
float64 bus_frame_rate_hz
float64 invalid_frame_ratio
uint32  total_faults_detected
