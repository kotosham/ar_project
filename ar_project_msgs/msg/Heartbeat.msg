# Published by every cross-link producer with deadline/liveliness QoS.
# Loss of heartbeat -> VLM mode degrades to FLAT.
uint8 OK=0
uint8 DEGRADED=1
uint8 DOWN=2

std_msgs/Header header
string node_name                # "slam" / "detector" / "planner_orchestrator" / ...
uint8 status
float32 cpu_load
float32 last_latency_ms         # measured latency of last response (for p99 timeouts)
uint32 mission_epoch
