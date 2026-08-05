> **Verification corrections. These values override the body below.**
> - **Aligned depth is 640x480, not 424x240.** With `align_depth.enable=true`, the
>   native 424x240 depth profile is reprojected into the 640x480 color grid, and
>   `target_pixel_to_goal` uses color intrinsics. A raw frame is
>   640 * 480 * 2 = 614400 B.
> - **RVL gives about 3:1 lossless compression**, not 10-20x. Dense 640x480
>   16UC1 RVL is about **50-70 KB**; 10x+ only applies to sparse depth. Therefore
>   a depth keyframe is about 50-70 KB, a full L4 `DetectTarget` request is about
>   **75-115 KB**, DETECT peak is about 75-115 KB, and combined peak is about
>   80-130 KB. If bandwidth becomes critical, send native 424x240 and realign on
>   EDGE.
> - **`/scan` via `depthimage_to_laserscan` is a NEW component.** It is not
>   present in the repository yet. The current `local_costmap` obstacle source is
>   PointCloud2 `/camera/camera/depth/color/points_rgbd` locally. In `robust`, this
>   is replaced by local `/scan`.
> - **L6 `map->odom` uses `ar_project_msgs/MapOdomCorrection`** (TransformStamped
>   + covariance + seq + relocalized), not `PoseWithCovarianceStamped`.
> - **The tightest chrony window is EKF `transform_timeout` = 0.1 s**, tighter
>   than TF 0.2 s, depth-match 0.35 s, and pixel-age 1.5 s.
> - **The target is represented in the `map` frame**, so it depends on
>   `map_odom_relay` health. During long link loss, uncorrected odometry drift
>   makes the 3D target less accurate; limit exploration radius/time and enter
>   SAFE_STOP when the drift budget is exceeded.
> - JPEG q80 640x480 is about 25-45 KB, or roughly 20-37x smaller than raw, not
>   15-25x.

# Pi <-> PC Data Contracts

This document describes every target-architecture data channel that crosses
Wi-Fi between ROBOT (Raspberry Pi 5, no GPU) and EDGE/PC (GPU box). Transport is
`rmw_zenoh` with a single `zenohd` systemd router on EDGE; fallback is Fast DDS
LARGE_DATA + Discovery Server. Multicast is disabled, socket buffers are 12 MB,
and all hosts use chrony. The core principle is: **the reactive loop (EKF, Nav2,
/scan, control, safety) NEVER waits for Wi-Fi**. Only rare, small, low-rate, or
event-driven messages cross the link. Heavy streams such as PointCloud2 and raw
RGB/depth never cross Wi-Fi.

All identifiers (topics, actions, services, node names, parameters) are in
English. Size, bandwidth, and latency estimates target RealSense D435i profiles:
RGB `640x480x15`, depth `424x240x15` aligned to color (`16UC1`, millimeters), as
configured in `realsense_rgbd_pi.launch.py`.

## 1. Per-Link Summary

QoS abbreviations: R=Reliable, BE=Best-Effort, TL=Transient-Local, V=Volatile,
KL=Keep-Last(N), KA=Keep-All. Latency is estimated end-to-end over Wi-Fi,
including serialization and RTT, excluding inference/VLM time where noted.
Clocks are synchronized by chrony, with offset much smaller than 0.2 s.

| # | Channel | Direction | Interface | Message / key fields | Encoding | Size | Rate | QoS | Bandwidth | Link latency | Degradation behavior |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **L1** | `SeekObject` | operator -> Pi | action `/seek_object` | goal: instruction, mode `{flat|vlm}`, target_desc, mission_epoch; feedback: phase, progress, committed_subgoal; result: outcome, note | CDR text | goal ~200-600 B, feedback ~150 B, result ~200 B | event, feedback 1-2 Hz | R,V,KL(10), feedback deadline 2 s | <1 KB/s | 10-40 ms | Link loss does not cancel the mission. Pi executive continues the committed subgoal; operator sees link-lost feedback timeout. |
| **L2** | `plan-request` | Pi executive -> EDGE planner | action `/plan_request` | instruction, mission_epoch, history_digest, compact frontier list, detection summary, notes refs | CDR, compact list, no costmap | ~2-8 KB | commit-point, every 5-30 s | R,V,KL(1), deadline = measured VLM p99 | idle ~0 | 10-40 ms + VLM seconds | single-in-flight + UUID idempotency; timeout/circuit-breaker keeps executive in FLAT. |
| **L3** | `plan-decision` | EDGE -> Pi | action result/feedback | PlanDecision with mission_epoch, plan_id, SubtaskNode tree, note_to_self, confidence, stamp | CDR structured tool call; VLM returns indices, not coordinates | ~0.5-4 KB | response to L2 | R,V,KL(1) | peak 0.5-4 KB/cycle | 10-40 ms after VLM | stale epoch is dropped; adoption only at commit point; timeout continues current subtask then degrades to FLAT. |
| **L4** | `DetectTarget request` | Pi -> EDGE | action `/detect_target` or service | compressed RGB, compressed depth, CameraInfo optionally once, prompt, stamp, frame_id, req_id | RGB JPEG q80; depth RVL/PNG 16UC1; CDR wrapper | RGB ~25-45 KB, depth per correction above | event keyframe, 0.2-2 Hz while searching | R,V,KL(1), deadline 0.5 s | idle 0, peak ~75-115 KB/request | 30-90 ms + GPU inference | stale/lost request yields no pixel; executive continues exploration. |
| **L5** | `DetectTarget result` | EDGE -> Pi | action result | u, v, depth_m, score, class_label, frame_id, stamp, req_id, found | CDR numeric | ~120-250 B | response to L4 | R,V,KL(1), deadline 0.5 s | <1 KB/s | 10-30 ms | req_id mismatch or stale stamp is dropped. Approach never declares reached from a stale pixel. |
| **L6** | `map->odom correction` | EDGE SLAM -> Pi relay | topic `/map_odom_correction` | `ar_project_msgs/MapOdomCorrection`: TransformStamped + covariance + seq + relocalized | CDR, low-rate correction, not TF stream | ~350-500 B | 1-2 Hz | R,TL,KL(1), deadline 2 s | ~0.5-1 KB/s | 10-30 ms | relay holds last-good, gates jumps/covariance, drops stale stamps, rebroadcasts `map->odom` locally within transform tolerance. |
| **H1** | Pi heartbeat | Pi -> EDGE | topic | light Header/DiagnosticStatus | CDR minimal | ~60-120 B | 2 Hz | BE,V,KL(1), deadline 1 s | <0.5 KB/s | <10 ms | EDGE marks Pi offline; mission is not affected. |
| **H2** | EDGE heartbeat | EDGE -> Pi | topic | same as H1 | CDR minimal | ~60-120 B | 2 Hz | BE,V,KL(1), deadline 1 s | <0.5 KB/s | <10 ms | Pi marks edge unavailable, freezes SLAM corrections, and VLM mode degrades to FLAT. |
| **H3** | planner-ready heartbeat | EDGE -> Pi | topic `/hb/planner` | state `{ready|busy|circuit_open}`, p99_ms, seq | CDR minimal | ~80-150 B | 1 Hz | BE,V,KL(1), deadline 2 s | <0.5 KB/s | <10 ms | Executive stops sending new plan requests until recovered. |
| **L7** | notes digest | mostly local on EDGE, optional EDGE -> Pi digest | topic `/mission/notes_digest` | digest, mission_epoch, seq | CDR text | ~0.5-4 KB | rare, <=0.2 Hz | R,TL,KL(1) | <1 KB/s | 10-40 ms | If absent, executive uses only local mission state. |
| **L8** | aggregated diagnostics | Pi <-> EDGE | topic `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | CDR aggregated, not raw high-rate diagnostics | ~1-4 KB | 1 Hz | R,V,KL(5), deadline 5 s | ~2-4 KB/s | 20-60 ms | Only monitoring is affected. |

L4 should be an **action** rather than a service in production, because it is
preemptable, can carry feedback, and supports UUID idempotency on mission epoch
changes. A service is acceptable only for synchronous test stands.

## 2. Keyframe and Format Decisions

### 2.1 RGB Keyframe: JPEG q80, 640x480

- Format: `sensor_msgs/CompressedImage`, `format="jpeg"`, q80 quality. Source is
  the `640x480x15` color profile.
- Why: q80 is enough for open-vocabulary detection (YOLOE/GroundingDINO) and is
  typically 25-45 KB. q90+ increases bytes by 50-80% with little detection gain.
- Pi encoding: libjpeg-turbo on ARM, O(W*H), about 3-6 ms on Pi 5 for 640x480.
  This is event keyframe work, not streaming work.
- Edge decode: O(W*H), about 1-2 ms on CPU or below 1 ms with nvJPEG.
- No pre-send downscale: 640x480 keeps small target coverage while q80 already
  fits the bandwidth budget.

### 2.2 Depth Keyframe: RVL Preferred, PNG Fallback

- Format: depth `sensor_msgs/CompressedImage`, source `16UC1` millimeters.
- RVL is preferred for lossless 16UC1 depth. For aligned 640x480 dense depth,
  expect about 50-70 KB. Native 424x240 may be smaller if bandwidth forces
  realignment on EDGE.
- PNG is the fallback: lossless but slower and often larger.
- Raw depth and PointCloud2 never cross Wi-Fi.
- RGB/depth frames are synchronized and carry the same stamp; the edge matches
  them by stamp using the same tolerance family as `target_pixel_to_goal`.

### 2.3 Intrinsics Once per Session

`CameraInfo` is sent once at detection-session start or when the profile changes.
On the Pi, 3D reconstruction uses the same pinhole model:
`x=(u-cx)*d/fx`, `y=(v-cy)*d/fy`, `z=d`. The planner and detector never return
navigation coordinates, only image-space detections and optional depth.

### 2.4 CDR vs Compression, Event-Driven vs Stream

- Small control/status messages use CDR without compression.
- Only heavy L4 keyframes are compressed.
- Everything crossing the link is event-driven or low-rate. There are no
  continuous high-rate Wi-Fi streams.
- Anti-stale checks are mandatory: L5 carries `stamp` + `req_id`; Approach checks
  pixel age and detector-stream staleness at success time. L6 drops stale
  correction stamps and gates covariance/jumps.

## 3. Bandwidth and Latency Budget

| State | Channels | Estimate |
|---|---|---|
| Idle | H1+H2+H3, L6 correction, L8 diagnostics, optional L1 feedback | ~3-6 KB/s, about 30-50 kbit/s |
| One DETECT peak | L4 + L5 over idle | ~75-115 KB burst |
| One PLAN peak | L2 + L3 + optional L7 | ~3-16 KB burst |
| Combined peak | detection + replan | ~80-130 KB burst |

The reactive loop is local on the Pi:

```text
RealSense -> depthimage_to_laserscan -> /scan (local)
EKF odom->base_link @20Hz
map_odom_relay rebroadcasts map->odom locally
Nav2 local costmap -> DWB controller @8-10Hz
cmd_vel -> watchdog + Collision Monitor + CiA-402 quick-stop -> EPOS4 RT write()
```

The slowest link path is VLM planning in seconds, but it is never on the path
from sensor to `cmd_vel`.

## 4. Data That Stays Local on the Pi

These streams are produced and consumed only on the Pi and must not cross Wi-Fi:

| Data | Topic/interface | Rate | Why local |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | ~15 Hz | obstacle source for local costmap; generated locally from depth |
| Full TF tree | `/tf`, `/tf_static` | 20-50 Hz | high-rate and reactive-critical |
| `cmd_vel` | `/cmd_vel`, `/diff_cont/cmd_vel_unstamped` | 8-10+ Hz | safety-critical drive command |
| EKF | odometry topics, `odom->base_link` | 20 Hz | real-time pose estimate |
| control | ros2_control, EPOS4 CiA-402 SocketCAN | RT cycle | hard real-time and quick-stop path |
| RealSense raw streams | color/depth/IMU | camera rate | only compressed event keyframes are exported |

Also local: local costmap, frontier extraction, all skill action servers,
`target_pixel_to_goal`, twist_mux, Collision Monitor, and `cmd_vel` watchdog.

## 5. Budget Summary

- RGB 640x480 JPEG q80: about **25-45 KB**, encode on Pi 5 about 3-6 ms.
- Aligned 640x480 16UC1 RVL depth: about **50-70 KB**.
- `DetectTarget` request: about **75-115 KB** per keyframe; result about
  **120-250 B**.
- map->odom correction: about **350-500 B** at 1-2 Hz.
- Heartbeats: about **60-150 B** each at 1-2 Hz.
- plan-request: about **2-8 KB**; plan-decision: about **0.5-4 KB**; notes digest:
  about **0.5-4 KB**.
- Idle cross-link: about **3-6 KB/s**. DETECT peak: about **75-115 KB**. PLAN
  peak: about **3-16 KB**.
- Local reactive latency budget: roughly one controller period plus EKF, and
  independent from the link.

Relevant files used for these contracts:

- `ar_project/scripts/target_pixel_to_goal.py`
- `ar_project/launch/realsense_rgbd_pi.launch.py`
- `ar_project/config/ekf_gyro.yaml`
- `object_tracking/object_tracking/tracker_node.py`

Custom `.action/.srv/.msg` files were not present when this architecture note was
written. `SeekObject`, `PlanRequest`/`PlanDecision`, `DetectTarget`,
`DetectionResult`, and `FrontierCandidate` must be created on `robust` if not
already implemented.
