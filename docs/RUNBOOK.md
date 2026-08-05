# RUNBOOK - Simulation and Hardware Stack Bringup

Short working instructions for the `robust` branch. Safety checklist before
autonomous runs: `HIL_BRINGUP_CHECKLIST.md`. Experiment logs:
`experiment_logs/vlm_missions/` and `experiment_logs/flat_missions/`.

## 0. What Runs Where

- **Raspberry Pi:** motors/CAN, RealSense, `/scan`, EKF, Nav2, `map_odom_relay`,
  `search_coordinator`.
- **Edge laptop:** camera relay `/camera_edge/*`, RTAB-Map SLAM, dashboard/logger,
  detector, VLM orchestrator.
- **FLAT:** mission is controlled by the Pi executive; VLM is not required.
- **VLM:** the target is sent to the executive, then the executive hands off to
  `planner_orchestrator`.
- **The detector is required in both modes:** FLAT also uses `detect_target_server`
  or the continuous tracker for DETECT/APPROACH.

## 1. Setup and Build

One-time machine setup:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Detector venv on edge:

```bash
python3 -m venv --system-site-packages ~/.venvs/ros-jazzy-ml
source ~/.venvs/ros-jazzy-ml/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install -r ~/ros2_ws/src/object_tracking/requirements.txt
```

VLM credentials, only for VLM mode:

```bash
cp ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env.example \
   ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
# Fill VLM_BASE_URL / VLM_API_KEY / VLM_MODEL.
```

## 2. Fast Hardware Bringup

Run this first in every terminal on **Pi**:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh
```

Run this first in every terminal on the **edge laptop**:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh
```

### Raspberry Pi

**Pi T1 - hardware, motors, watchdog, twist mux, `/scan`**

```bash
ros2 launch ar_project hardware_bringup.launch.py
```

Enable Collision Monitor if needed:

```bash
ros2 launch ar_project hardware_bringup.launch.py use_collision_monitor:=true
```

**Pi T2 - RealSense RGB-D + IMU**

```bash
ros2 launch ar_project realsense_rgbd_pi.launch.py \
  rgb_camera.color_profile:=640x480x6 \
  depth_module.depth_profile:=424x240x6
```

**Pi T3 - map->odom relay**

```bash
ros2 run search_coordinator map_odom_relay --ros-args \
  -p use_sim_time:=false
```

**Pi T4 - Nav2**

```bash
ros2 launch ar_project navigation_launch.py \
  use_sim_time:=false \
  odom_topic:=/odometry/filtered
```

**Pi T5 - executive / skill servers**

```bash
ros2 run search_coordinator coordinator_node --ros-args \
  -p use_sim_time:=false \
  -p approach_max_goal_step_m:=1.2 \
  -p approach_direct_clearance_m:=0.55 \
  -p approach_direct_if_goal_in_known_free_map:=true \
  -p approach_allow_unknown_bounded_goal:=true \
  -p approach_unknown_bounded_max_step_m:=0.6 \
  -p flat_target_pixel_max_age_s:=4.0 \
  -p flat_initial_scan_forward_wait_s:=4.0 \
  -p flat_initial_scan_settle_s:=2.0 \
  -p flat_initial_scan_view_detect_wait_s:=4.0
```

In FLAT, if the target is not found in the starting frame, the coordinator runs a
fixed non-semantic overview `forward -> right -> left`, then switches to
`ExploreFrontier`.

### Edge Laptop

**Edge T1 - camera relay + RTAB-Map + frontiers + dashboard/logger**

For FLAT experiments:

```bash
ros2 launch ar_project edge_bringup.launch.py \
  flat_log_run_id:=flat_scene_1 \
  start_vlm_logger:=false
```

For VLM experiments:

```bash
ros2 launch ar_project edge_bringup.launch.py \
  vlm_log_run_id:=vlm_scene_1 \
  start_flat_logger:=false
```

Quick frontier check:

```bash
ros2 node list | grep -E 'frontier_extractor|rtabmap'
ros2 topic info /frontiers -v
timeout 8 ros2 topic echo /frontiers --once
```

`/frontiers` in `topic list` is not proof by itself. The topic can appear only
because `search_coordinator` subscribes to it; there must be a publisher from
`/frontier_extractor`.

**Edge T2-FLAT - continuous detector (`/target_pixel`)**

FLAT expects a `/target_pixel` stream, so it uses the continuous tracker rather
than the VLM action detector.

```bash
ros2 launch object_tracking sam_node.launch.py \
  model_mode:=dino_mobilesam \
  tracking_mode:=continuous \
  use_compressed_input:=false \
  image_topic:=/camera_edge/color/image_raw \
  use_depth_input:=true \
  depth_topic:=/camera_edge/aligned_depth_to_color/image_raw
```

After sending a FLAT mission:

```bash
ros2 topic info /target_pixel -v
timeout 8 ros2 topic echo /target_pixel --once
```

`/target_pixel` should have `rgb_tracker_node` as publisher.

**Edge T2-VLM - action detector / Set-of-Mark**

```bash
/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server \
  --ros-args \
  -p image_topic:=/camera_edge/color/image_raw \
  -p depth_topic:=/camera_edge/aligned_depth_to_color/image_raw \
  -p target_conf_default:=0.60
```

Defaults are already encoded: `model_mode=dino`, `depth_point_strategy=nearest_mask`,
`use_compressed_input=false`.

**Edge T3 - VLM orchestrator, VLM only**

```bash
set -a
source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
set +a

/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node
```

**Edge T4 - RViz**

```bash
ros2 launch ar_project rviz_launch.py \
  use_sim_time:=false \
  config:=$(ros2 pkg prefix ar_project)/share/ar_project/config/rtabmap_rgbd.rviz
```

**Edge T5 - send mission**

```bash
# FLAT
ros2 run fleet_comms send_mission "chair" false

# VLM
ros2 run fleet_comms send_mission "chair" true
```

Dashboard: `http://localhost:8088`.

## 3. Pre-Mission Quick Check

Pi:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
timeout 8 ros2 topic hz /scan
timeout 8 ros2 topic hz /odometry/filtered
```

Edge:

```bash
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map_odom_correction
ros2 action list | grep detect
```

## 4. Fast Simulation Bringup

VLM simulation:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1
set -a; source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env; set +a

ros2 launch ar_project vlm_sim_bringup.launch.py \
  start_edge:=true \
  venv_python:=/home/user/.venvs/ros-jazzy-ml/bin/python \
  vlm_log_run_id:=sim_bus_001
```

Mission:

```bash
ros2 run fleet_comms send_mission "bus" true
```

FLAT simulation:

```bash
ros2 launch ar_project flat_sim_bringup.launch.py
ros2 run fleet_comms send_mission "bus" false
```

If FLAT has no frontiers, seed the map with a short rotation:

```bash
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}" &
sleep 5
kill %1
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: -0.6}}" &
sleep 5
kill %1
```

## 5. FLAT vs VLM

The operator command is the same:

```bash
ros2 run fleet_comms send_mission "<target>" <true|false>
```

- `false`: FLAT, `/seek_object allow_vlm=false`; the executive owns the mission.
  If the target is not visible, it performs a fixed scan and then
  `ExploreFrontier`.
- `true`: VLM, `/seek_object allow_vlm=true`; the executive publishes the
  instruction to internal `/vlm_mission`, and Qwen chooses actions from camera,
  map, context marks, and memory.
- `/vlm_mission` is a debug/internal topic; normal operators do not need it.
- VLM requires `Edge T3`; FLAT does not require `Edge T3` or `vlm.env`.

## 6. VLM Mode Behavior

At each step, the orchestrator sends Qwen:

- target/instruction;
- `visible_marks`: strict target candidates eligible for `DRIVE_TO_VISIBLE`;
- `context_marks`: office-object cues, not destinations;
- Set-of-Mark camera frame;
- top-down SLAM map `/map` with robot pose;
- notes/memory: corridor scans, previous actions, Nav2 failure reasons.

VLM actions:

- `TURN`
- `DRIVE_FORWARD`
- `DRIVE_TO_VISIBLE`
- `DETECT_ALL`, currently a refresh of the fixed DINO context vocabulary
- `DONE`

Current rule: if the target is not visible, the robot explores free corridors on
the map. Context objects help choose between corridors but are not attraction
points. If a target is confidently found, the orchestrator stores its map point
and continues approach even if the object is temporarily lost from the frame.

## 7. Important Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `target_conf_default` | `0.60` | Target DINO threshold; shared by VLM action detector and FLAT continuous tracker |
| `target_detect_conf` | `0.60` | Target threshold in orchestrator |
| `context_detect_conf` | `0.30` | DINO context-object threshold |
| `async_replan` | `false` | Discrete loop: drive -> stop -> observe -> think |
| `turn_settle_s` | `2.0` | Pause after TURN before image analysis |
| `min_effective_turn_rad` | `0.60` | Small TURNs are normalized because Nav2 may accept them without motion |
| `initial_scan_when_target_absent` | `true` | If no target, overview: forward -> right -> left |
| `flat_initial_scan_enabled` | `true` | FLAT baseline also scans, but without semantic VLM choice |
| `flat_initial_scan_forward_wait_s` | `4.0` | Wait for initial target detection before FLAT scan |
| `flat_initial_scan_settle_s` | `2.0` | Pause after FLAT scan TURN before detection |
| `flat_initial_scan_view_detect_wait_s` | `4.0` | Wait window for fresh target after each stable scan view |
| `continuous_inference_rate` | `0.5` | DINO/SAM rate in FLAT tracker, kept low to avoid starving RGB-D/RTAB-Map |
| `continuous_header_max_age` | `3.5` | Do not publish `/target_pixel` if RGB header stamp is already old |
| `approach_max_goal_step_m` | `1.2` | Bounded step toward distant/poorly mapped target |
| `approach_direct_clearance_m` | `0.55` | Known-free radius around direct standoff point |
| `approach_allow_unknown_bounded_goal` | `true` | Allow short cautious probe through unknown space |
| `locked_target_approach_max_attempts` | `8` | Keep a confirmed target after temporary visual loss |
| `vlm_timeout_s` | `30.0` | VLM response timeout |
| `send_map` | `true` | Send map as second image |

## 8. Monitoring and Logs

Dashboard:

```text
http://localhost:8088
```

CLI:

```bash
ros2 topic echo /robot_health
ros2 topic echo /vlm/activity
ros2 topic echo /mission/status
ros2 topic echo /planner/notes
ros2 topic echo /frontiers
ros2 node list
ros2 action list
```

Persistent mission logs are started automatically from `edge_bringup.launch.py`.
For meaningful file names, set `flat_log_run_id:=flat_scene_1` or
`vlm_log_run_id:=vlm_scene_1`.

```text
~/ros2_ws/experiment_logs/flat_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/flat_missions/<run_id>.csv
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.csv
```

Timing fields: VLM CSV stores `latency_ms` and `time_to_first_action_s`; FLAT CSV
stores `detector_runtime_mean_s`, `time_to_detect_s`, and `time_to_approach_s`.
For latency comparison between modes, use VLM `latency_ms` and FLAT
`detector_runtime_mean_s`. `time_to_detect_s` / `time_to_approach_s` include
turns, frame settling, and waiting for a depth point.

Manual loggers:

```bash
ros2 run fleet_comms flat_mission_logger --ros-args \
  -p output_dir:=~/ros2_ws/experiment_logs/flat_missions \
  -p run_id:=flat_scene_1

ros2 run fleet_comms vlm_mission_logger --ros-args \
  -p output_dir:=~/ros2_ws/experiment_logs/vlm_missions \
  -p run_id:=vlm_scene_1
```

## 9. Expected Degradation

If the VLM is unavailable or times out, the orchestrator opens the circuit
breaker and switches to DEGRADED/FLAT fallback. The mission should not stop
abruptly. If edge/Wi-Fi is lost, the Pi executive remains autonomous.

## 10. Common Issues

- **No `/camera_edge/*`:** check RealSense on Pi and `edge_bringup` on edge.
- **Detector fails because of `torch`:** run it through
  `/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server`.
- **Detector runs but depth is missing:** check
  `/camera_edge/aligned_depth_to_color/image_raw`.
- **Nav2 node not found:** restart `navigation_launch.py`, then check lifecycle
  nodes.
- **Robot does not move:** check `/cmd_vel_out`, `/cmd_vel_collision_safe`,
  `/diff_cont/cmd_vel`, and watchdog.
- **Collision Monitor blocks motion:** check `/scan` timestamp,
  `camera_link -> base_link` TF, and `/collision_monitor` lifecycle.
- **VLM sees target but `DRIVE_TO_VISIBLE` does not move:** inspect
  `search_coordinator` for `ABORTED` reasons: `clearance_occupied`,
  `clearance_unknown`, `outside_map`.
- **False target detections:** raise `target_conf_default`/`target_detect_conf`;
  do not tune context threshold first.
- **Map missing in VLM (`map=no`):** `/map` missing, `send_map=false`, or
  orchestrator not launched from the venv with cv2/numpy.
- **Gazebo SIGSEGV:** simulation only; before launch, run `export GZ_IP=127.0.0.1`.
