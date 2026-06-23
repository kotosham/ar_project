# Cross-link QoS & Heartbeat policy — ROADMAP Phase 1.3

Every Pi↔edge ROS endpoint takes its QoS from a named profile in
[`fleet_comms/qos.py`](../fleet_comms/fleet_comms/qos.py) — never hand-rolled.
This is the single source of truth; `fleet_comms` is depended on by
`search_coordinator` (Pi) and `planner_orchestrator` (edge).

## Profiles

| Profile | Reliability | Durability | Depth | Deadline | Liveliness (lease) | Use for |
|---|---|---|---|---|---|---|
| `control_cmd` | RELIABLE | VOLATILE | 1 | 2.0 s | MANUAL_BY_TOPIC (3 s) | SeekObject goal, DetectTarget goal, PlanStep |
| `control_cmd_latched` | RELIABLE | TRANSIENT_LOCAL | 1 | — | AUTOMATIC | SeekObject result/status (operator reconnect) |
| `liveliness_status(p)` | RELIABLE | VOLATILE | 1 | 1.5·p | MANUAL_BY_TOPIC (3·p) | Heartbeat + periodic health |
| `correction_lowrate` | RELIABLE | VOLATILE | 1 | 1.0 s | AUTOMATIC (3 s) | MapOdomCorrection (~1–2 Hz) |
| `detection_stream` | BEST_EFFORT | VOLATILE | 1 | 1.5 s | — | OFFER side of a *periodic* detection stream (publisher) |
| `detection_stream_nodeadline` | BEST_EFFORT | VOLATILE | 1 | — | — | /target_pixel **consumer** (sporadic stream; freshness via app age-gate) |
| `media_besteffort` | BEST_EFFORT | VOLATILE | 1 | — | — | **compressed** frames/bursts only |

Why deadline+liveliness: a silent producer must be observable within seconds.
`liveliness_status` producer and monitor MUST use the **same period** so the
offered/requested QoS stay compatible — [`is_compatible()`](../fleet_comms/fleet_comms/qos.py)
encodes the DDS Request-vs-Offered rules and the unit test locks them in.

## Cross-link endpoint → profile

| Endpoint | Kind | Producer | Profile | Status |
|---|---|---|---|---|
| `/seek_object` | action | Pi exec | `control_cmd` goal / `control_cmd_latched` result+status | planned (2.x) |
| `/detect_target` | action | edge | `control_cmd` goal / result `media_besteffort` for annotated frame | planned (3.2) |
| `/map_odom_correction` | topic | edge SLAM | `correction_lowrate` | planned (2.6) |
| `/heartbeat` | topic | every producer | `liveliness_status(0.5)` | **now** |
| `PlanStep` | topic | edge planner | `control_cmd` (no fixed deadline) | planned (4.x) |
| `Candidate[]` | sub-msg | edge | inherits carrying action result | planned (3.x) |
| `/target_pixel` | topic | edge tracker | publisher: no-deadline BEST_EFFORT (sporadic); **consumer: `detection_stream_nodeadline`** — requesting `detection_stream`'s 1.5 s deadline drops every sample (must-fix #2, locked by `is_compatible` test) | exists |
| `/target_prompt` | topic | Pi exec (`PromptBridge`) | `control_cmd_latched` — replaces `reliable_prompt_sender`; tracker sub should become TRANSIENT_LOCAL in 2.9 for late-join replay | planned (2.x) |
| GetObservation `result.view` | action payload | Pi | CompressedImage only; `media_besteffort` if relayed | planned (3.5) |

## No raw depth / PointCloud2 over Wi-Fi (ROADMAP 1.4 / 3.5)

`media_besteffort` is for **compressed** media only. Raw depth and PointCloud2
must never cross the link.

- **LEAK (open, architectural — Phase 3.x):** `/tracker/aligned_depth_to_color/image_raw`
  — RAW uncompressed aligned depth published by
  [`tracker_rgbd_bridge.py:56`](../ar_project/scripts/tracker_rgbd_bridge.py) at
  default RELIABLE depth=10. The one true raw-depth-over-Wi-Fi violation. Fix is
  to compress (`compressedDepth`) or keep depth edge-local and ship only the
  derived point/Candidate — part of the tracker→DetectTarget rework (Phase 3.2/3.5),
  not a QoS tweak.
- Compressed media that legitimately crosses but currently uses default RELIABLE
  depth=10 (→ `media_besteffort` *if the node survives*):
  `/tracker/color/image/compressed`, `/tracker/color/image_raw` (CompressedImage
  despite the name). Heavy mono8 mask `/target_mask` → replaced by
  Candidate[]+CompressedImage in Phase 3.2.
- No PointCloud2 publishers exist in source (Phase 1.4 moved the costmap to the
  local `/scan`).

## Do NOT re-profile these (deleted in Phase 2.9)

`reliable_prompt_sender.py` and the latch-soup handshake (`/target_prompt`,
`/target_prompt_ack`, `/target_goal_locked`), and the edge tracker's reactive
`/cmd_vel` path. Their QoS work would be thrown away — `SeekObject`
(`control_cmd_latched`) is the principled replacement.

## Heartbeat (`fleet_comms/heartbeat.py`)

- **Producers** (`HeartbeatPublisher`) — edge SLAM (`slam`), detector (`detector`),
  `planner_orchestrator`; the Pi `search_coordinator` emits one for symmetry. One
  `/heartbeat` topic, distinguished by `node_name`, **0.5 s** period fleet-wide.
  Fills `header.stamp` (chrony-synced, Phase 1.2), `status`, `cpu_load`,
  `last_latency_ms` (feeds the p99 circuit-breaker, Phase 4.4), `mission_epoch`
  (stale-epoch beats ignored, Phase 2.5).
- **Monitor** (`HeartbeatMonitor`, in `search_coordinator`) — tracks per-`node_name`
  health from the reported status, the QoS deadline-missed / liveliness-lost
  events, and a stale-timeout fallback. Phase 1.3 only logs / exposes
  `health_snapshot()`; the VLM→FLAT degradation FSM wiring is Phase 5.1.
