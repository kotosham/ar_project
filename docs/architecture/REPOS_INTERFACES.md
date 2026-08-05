# Repository Structure, Interfaces, and Estimates

This document records the target code layout for the `robust` branches of both
repositories (`ar_project` for Raspberry Pi / executive, `object_tracking` for
edge perception), ROS 2 interface inventory, complexity estimates, and the
FIX-FIRST implementation order. The architecture is considered accepted after
design and FMEA review; this document implements it rather than reopening it.
Target platform: ROS 2 Jazzy, with Gazebo testing inside WSL2 Ubuntu.

## 1. Package Layout

### 1.1 Split Principle

- **`ar_project` (Pi side, executive-on-Pi)**: real-time and reactive nodes live
  here: EKF, light Nav2, `map_odom_relay`, Search Coordinator, skill servers,
  `target_pixel_to_goal`, `depthimage_to_laserscan`, RealSense driver,
  `ros2_control` + `EmbodiedRobotSystem`, and safety. VLM never runs on this
  side.
- **`object_tracking` (edge/PC side)**: RTAB-Map SLAM, open-vocabulary detector
  (DINO+MobileSAM for target and fixed context vocabulary; YOLOE legacy/comparison
  only), Planner Orchestrator as a light HTTP client to an external
  OpenAI-compatible VLM API, semantic memory, and notes buffer. Edge GPU runs
  detector and SLAM only; VLM is behind an API.
- **Transport**: `rmw_zenoh` with one router on edge, Fast DDS fallback,
  multicast disabled, 12 MB socket buffers, chrony on all hosts, and QoS
  deadline/liveliness on cross-link topics. PointCloud2 and raw depth never cross
  Wi-Fi; `/scan` is generated locally on the Pi.

### 1.2 Interface-Only Packages

Interface packages avoid cyclic dependencies and let both hosts build message
definitions without heavy runtime dependencies:

- **`ar_project_msgs`**: Pi-side and cross-link interfaces used by the executive:
  skill actions, `MapOdomCorrection.msg`, heartbeat/status messages, and plan
  types consumed by the executive.
- **`object_tracking_msgs`**: perception/planning interfaces:
  `DetectTarget.action`, `SeekObject.action`, `PlanStep.msg`, `Notes.msg`, and
  Set-of-Mark candidate types.

Both packages are pure `rosidl` and lightweight.

### 1.3 Logic Packages

- **`search_coordinator`** (`ar_project`): executive FSM/BT, local frontier
  extraction, mission owner, skill action servers, `map_odom_relay`, frontier
  hysteresis, mission epoch, UUID idempotency, and default productive action.
- **`planner_orchestrator`** (`object_tracking`): async HTTP client to the
  external VLM API. It implements single-in-flight requests, UUID idempotency,
  p99 timeout, circuit breaker, structured tool-call output, streaming, notes
  summarization, and anytime/async replanning with commit-point adoption.

### 1.4 Reused / New / Deleted

**Reused:**

- `target_pixel_to_goal.py` math for pixel + aligned depth -> metric target.
- Segmentation backends in `object_tracking`.
- Nav2 configs, EKF configs, EPOS4/CAN hardware interface, RealSense launches,
  URDF/xacro, `ros2_control`, `twist_mux`, and `depthimage_to_laserscan`.

**New:**

- `ar_project_msgs`, `object_tracking_msgs`, `search_coordinator`,
  `planner_orchestrator`.
- `map_odom_relay`, local frontier extractor with hysteresis, skill action
  servers, `cmd_vel` watchdog, real CiA-402 quick-stop, RTAB-Map low-rate
  `MapOdomCorrection`, Set-of-Mark rendering, and notes/summary buffer.
- Gazebo-on-WSL bringup plus `rmw_zenoh`/`chrony` configuration.

**Deleted / replaced:**

- `reliable_prompt_sender.py` and its launch file. `SeekObject` action replaces
  timer-based prompt retry.
- Latched "soup" topics and logic: `/target_goal_locked`, `/target_prompt_ack`,
  `/target_prompt`, `goal_locked`, `lock_goal_on_publish`, and auto-success from
  `nav_status` inside old perception/goal code.
- Reactive `/cmd_vel` writes from `tracker_node.py`; search becomes an
  `ExploreFrontier` skill on the Pi. Edge never writes to the reactive path.

## 2. Interface Inventory

All actions are preemptable, carry feedback, and include UUID-style request IDs
for idempotency. Repeating the same `request_id` inside the current
`mission_epoch` attaches to the existing execution rather than executing twice.

### 2.1 High-Level Mission

`object_tracking_msgs/action/SeekObject.action`

```text
# Goal
string instruction
string request_id
uint32 mission_epoch
bool allow_vlm
---
# Result
uint8 outcome
geometry_msgs/PoseStamped final_pose
string summary
---
# Feedback
string state
string active_subtask
float32 progress
uint32 mission_epoch
```

### 2.2 Pi Skill Actions

`ar_project_msgs/action/ExploreFrontier.action`

```text
string request_id
uint32 mission_epoch
int32 frontier_id
float32 max_travel_m
---
uint8 outcome
geometry_msgs/PoseStamped reached_pose
---
float32 distance_remaining
int32 selected_frontier_id
float32 frontier_score
```

`ar_project_msgs/action/GoToPose.action`

```text
string request_id
uint32 mission_epoch
geometry_msgs/PoseStamped target_pose
float32 xy_tolerance
float32 yaw_tolerance
---
uint8 outcome
geometry_msgs/PoseStamped reached_pose
---
float32 distance_remaining
builtin_interfaces/Time stamp
```

`ar_project_msgs/action/ApproachDetection.action`

```text
string request_id
uint32 mission_epoch
string target_label
float32 approach_offset
float32 max_pixel_age_s
bool use_locked_target
geometry_msgs/PointStamped locked_target_point
---
uint8 outcome
geometry_msgs/PoseStamped reached_pose
geometry_msgs/PointStamped target_point
geometry_msgs/PoseStamped final_goal_pose
float32 final_distance_m
---
float32 distance_to_target
float32 detection_age_s
bool detection_fresh
```

`ar_project_msgs/action/GetObservation.action`

```text
string request_id
uint32 mission_epoch
bool with_setofmark
---
uint8 outcome
sensor_msgs/CompressedImage view
object_tracking_msgs/Candidate[] candidates
geometry_msgs/PoseStamped observed_from
---
string phase
```

`ar_project_msgs/action/Stop.action`

```text
string request_id
uint32 mission_epoch
uint8 mode
---
uint8 outcome
---
bool zero_velocity_confirmed
```

### 2.3 Perception

`object_tracking_msgs/action/DetectTarget.action`

```text
string request_id
uint32 mission_epoch
string query
bool render_setofmark
float32 conf_threshold
---
uint8 outcome
object_tracking_msgs/Candidate[] candidates
sensor_msgs/CompressedImage annotated
---
uint32 frames_processed
float32 best_confidence
```

`object_tracking_msgs/msg/Candidate.msg`

```text
uint32 mark_id
string label
float32 confidence
geometry_msgs/Point pixel
string source_frame_id
builtin_interfaces/Time stamp
sensor_msgs/RegionOfInterest bbox
```

### 2.4 Localization

`ar_project_msgs/msg/MapOdomCorrection.msg`

```text
std_msgs/Header header
geometry_msgs/TransformStamped map_to_odom
float64[36] covariance
float64 fitness
uint32 seq
bool relocalized
```

### 2.5 Planning

`object_tracking_msgs/msg/PlanStep.msg`

```text
uint8 skill
int32 frontier_id
uint32 approach_target_mark
string arg_label
string step_id
string rationale
```

`object_tracking_msgs/msg/Notes.msg`

```text
std_msgs/Header header
uint32 mission_epoch
string summary
string[] facts
uint32 token_estimate
```

### 2.6 Heartbeat / Health

`ar_project_msgs/msg/Heartbeat.msg`

```text
std_msgs/Header header
string node_name
uint8 status
float32 cpu_load
float32 last_latency_ms
uint32 mission_epoch
```

Optional test service:

`ar_project_msgs/srv/SetMode.srv`

```text
uint8 mode
---
bool accepted
uint8 active_mode
```

## 3. Complexity and Effort

Relative complexity: S ~1-2 days, M ~3-5 days, L ~6-10 days for one engineer,
including Gazebo tests. Days are rough estimates; risk is regression/rework
probability.

| Component | Package | Complexity | Days | Risk | Note |
|---|---|---:|---:|---|---|
| Real CiA-402 quick-stop in `write()` | ar_project | M | 4 | High | RT path, requires HIL |
| Per-cycle fault poll + CAN bus-off recovery | ar_project | M | 3 | High | Previously only sparse logging |
| `cmd_vel` watchdog + Collision Monitor | ar_project / search_coordinator | S | 2 | Medium | Standard Nav2/watchdog pieces |
| `use_sim_time` parameterization | ar_project | S | 1 | Low | Mechanical launch/yaml cleanup |
| rmw_zenoh + chrony + socket buffers + QoS | both | M | 4 | Medium | Cross-host networking |
| Gazebo-on-WSL bringup + worlds | ar_project | M | 3 | Medium | GPU/WSL quirks |
| Local `/scan` from depthimage_to_laserscan | ar_project | S | 1 | Low | Package setup |
| Interface packages | both | S | 2 | Low | Straightforward but verbose |
| Executive FSM/BT | search_coordinator | L | 9 | High | Mission ownership core |
| Frontier extractor + hysteresis | search_coordinator | M | 4 | Medium | Anti-oscillation critical |
| Skill action servers + idempotency/preemption | search_coordinator | L | 8 | High | Easy to get wrong |
| `map_odom_relay` | search_coordinator | M | 5 | High | TF and timing correctness |
| Light Nav2 profile | ar_project | M | 3 | Medium | Needs Pi profiling |
| `target_pixel_to_goal` adaptation | ar_project | S | 2 | Medium | Preserve geometry |
| RTAB-Map correction output | object_tracking | M | 5 | High | TF stream -> correction message |
| `DetectTarget` action + Set-of-Mark | object_tracking | M | 5 | Medium | Backends already exist |
| Detector stream staleness gate | object_tracking / search_coordinator | S | 2 | High | Direct FMEA fix |
| Planner Orchestrator | planner_orchestrator | L | 10 | High | Hardest edge component |
| Notes/summary buffer | planner_orchestrator | M | 4 | Medium | Context/cost control |
| Async replan + commit adoption | both | M | 5 | High | FLAT must keep moving |
| VLM->FLAT degradation | both | M | 3 | High | Must be seamless |
| Instruction change reset | search_coordinator | S | 2 | High | Prevent zombie goals |
| FMEA Gazebo tests | both | M | 5 | Medium | Failure injection infra |
| Remove prompt/latched soup | ar_project | S | 1 | Low | Cleanup and subscription check |
| Hardware bringup | ar_project | L | 8 | High | Full real robot integration |

Rough total: 110-120 person-days. Critical path: executive FSM, skill servers,
`map_odom_relay`, and Planner Orchestrator.
