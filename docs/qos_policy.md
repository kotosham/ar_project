# Cross-Link QoS and Heartbeat Policy - ROADMAP Phase 1.3

Every ROS exchange point between Pi and edge must use a named QoS profile from
[`fleet_comms/qos.py`](../fleet_comms/fleet_comms/qos.py). QoS is not configured
manually at call sites. This is the single source of truth. `search_coordinator`
on Pi and `planner_orchestrator` on edge both depend on `fleet_comms`.

## Profiles

| Profile | Reliability | Durability | Depth | Deadline | Liveliness (lease) | Purpose |
|---|---|---:|---:|---|---|---|
| `control_cmd` | RELIABLE | VOLATILE | 1 | 2.0 s | MANUAL_BY_TOPIC (3 s) | SeekObject goal, DetectTarget goal, PlanStep |
| `control_cmd_latched` | RELIABLE | TRANSIENT_LOCAL | 1 | - | AUTOMATIC | SeekObject result/status for operator reconnect |
| `liveliness_status(p)` | RELIABLE | VOLATILE | 1 | 1.5*p | MANUAL_BY_TOPIC (3*p) | heartbeat + periodic health |
| `correction_lowrate` | RELIABLE | VOLATILE | 1 | 1.0 s | AUTOMATIC (3 s) | MapOdomCorrection, about 1-2 Hz |
| `detection_stream` | BEST_EFFORT | VOLATILE | 1 | 1.5 s | - | offered side of periodic detection stream |
| `detection_stream_nodeadline` | BEST_EFFORT | VOLATILE | 1 | - | - | `/target_pixel` consumer; sporadic stream, app-level age gate |
| `media_besteffort` | BEST_EFFORT | VOLATILE | 1 | - | - | compressed frames/bursts only |

Deadline + liveliness make silent producers observable within seconds. Producer
and monitor using `liveliness_status` must use the same period so offered and
requested QoS remain compatible. `is_compatible()` encodes DDS Request-vs-Offered
rules and unit tests pin them down.

## Endpoint -> Profile Map

| Endpoint | Type | Producer | Profile | Status |
|---|---|---|---|---|
| `/seek_object` | action | Pi executive | `control_cmd` goal / `control_cmd_latched` result+status | planned / implemented by phase |
| `/detect_target` | action | edge | `control_cmd` goal / `media_besteffort` for annotated frame | planned / implemented by phase |
| `/map_odom_correction` | topic | edge SLAM | `correction_lowrate` | planned |
| `/heartbeat` | topic | every producer | `liveliness_status(0.5)` | current |
| `PlanStep` | topic | edge planner | `control_cmd` without fixed deadline | planned |
| `Candidate[]` | sub-message | edge | inherits from action result | planned |
| `/target_pixel` | topic | edge tracker | publisher BEST_EFFORT without deadline; consumer `detection_stream_nodeadline` | existing |
| `/target_prompt` | topic | Pi exec (`PromptBridge`) | `control_cmd_latched` | planned/current depending branch |
| GetObservation `result.view` | action payload | Pi | CompressedImage only; `media_besteffort` when relayed | planned |

## No Raw Depth / PointCloud2 over Wi-Fi

`media_besteffort` is for **compressed** media only. Raw depth and PointCloud2
must never cross the link.

Known architectural leak noted during Phase 1:

- `/tracker/aligned_depth_to_color/image_raw` was raw uncompressed aligned depth
  from `tracker_rgbd_bridge.py`. The real fix is to compress it or keep depth
  local to edge and transmit only derived points/candidates as part of the
  tracker-to-DetectTarget redesign.

Compressed media that may legitimately cross the link should use
`media_besteffort` if the node tolerates it. Heavy mono masks should be replaced
by `Candidate[]` plus a compressed annotated image.

Do not spend effort reassigning QoS to components removed by Phase 2.9:
`reliable_prompt_sender.py`, old prompt/ack/goal-locked latch soup, and reactive
edge-tracker `/cmd_vel`. `SeekObject` with `control_cmd_latched` replaces them.

## Heartbeat

`fleet_comms/heartbeat.py` provides:

- **Producers** (`HeartbeatPublisher`): edge SLAM (`slam`), detector
  (`detector`), `planner_orchestrator`, and Pi `search_coordinator` for symmetry.
  One `/heartbeat` topic is keyed by `node_name`; period is 0.5 s. Fields include
  synchronized stamp, status, CPU load, last latency, and mission epoch.
- **Monitor** (`HeartbeatMonitor` in `search_coordinator`): tracks health per
  `node_name` from status, QoS deadline/liveliness events, and stale-timeout
  fallback. In Phase 1.3 it logs and exposes `health_snapshot()`. FSM degradation
  wiring belongs to Phase 5.1.
