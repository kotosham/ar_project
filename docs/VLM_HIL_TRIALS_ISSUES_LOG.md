# VLM HIL Trials: Issue Log

Concise trial-and-error log for VLM/FLAT HIL testing.

Read each entry as: `symptom -> cause -> fix -> verification`.

## Current Behavior Contract

- Confident strict target found -> approach it, do not keep exploring.
- Target lost after confident localization -> `target_nav_lock` continues toward
  the last confirmed point.
- Goal inside known-free map -> direct approach.
- Goal outside known-free map -> bounded approach.
- Bounded approach impossible -> recovery and retry locked target.
- Target not found -> context objects are corridor cues, not destinations.
- Invisible-target search priority: free/unknown corridor, not driving toward
  furniture.
- VLM turns below about `0.60rad` are not useful scan actions.
- Persistent log file name equals `run_id`.

## Quick Checks

```bash
# transport
cat /etc/zenoh/zenoh_session_config.json5 | grep -E 'mode:|gossip'

# health / motion
ros2 control list_controllers
timeout 8 ros2 topic hz /joint_states
timeout 8 ros2 topic hz /odometry/filtered
timeout 8 ros2 topic hz /scan
ros2 topic info /cmd_vel_out -v
ros2 topic info /cmd_vel_collision_safe -v
ros2 topic info /diff_cont/cmd_vel -v

# camera / SLAM
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map
timeout 8 ros2 topic hz /map_odom_correction

# Nav2
ros2 node list | grep -E 'controller_server|planner_server|bt_navigator'
ros2 lifecycle get /controller_server
ros2 topic list | grep costmap
```

## 1. Transport / Infrastructure

### 1.1 Zenoh Peer/Gossip Hurt Pi Stability

- Symptom: `Unable to connect to any locator...`, `Unable to push non droppable
  network message`, teleop delay.
- Cause: ROS processes tried peer/direct connection paths instead of a single
  router.
- Fix: `mode: "client"`, `gossip: false`, single router endpoint,
  `transport_env.sh`.
- Check: `mode: "client"` and `gossip: { enabled: false }` on both machines.

### 1.2 Scary Zenoh/RMW Warnings Are Not Always Fatal

- Symptom: `Watchdog Confirmator`, `Watchdog Validator`, `Scouting delay`,
  unsupported QoS callbacks.
- Cause: `rmw_zenoh_cpp`/SHM/QoS behavior.
- Fix: treat as non-fatal when topics are alive, lifecycle is active, and health
  is OK.
- Check: `topic hz`, `ros2 control list_controllers`, dashboard health.

### 1.3 Logger Replay Duplicated Old Mission

- Symptom: after logger restart, a "new" run appeared without a real mission.
- Cause: `/vlm/activity` used `TRANSIENT_LOCAL`, so the logger received replayed
  history.
- Fix: persistent logger subscribes with `VOLATILE`.
- Check: no new `mission_start` appears without a new mission; `logger_rx_iso`
  is close to event `stamp`.

## 2. Camera / SLAM / Map

### 2.1 RealSense RGB/Depth Size Mismatch

- Symptom: RGB `640x480`, depth `424x240`; wrong or zero `distance_m`.
- Cause: RGB pixel used without conversion into depth coordinates.
- Fix: convert RGB -> depth using actual frame sizes.
- Check: depth `encoding=16UC1`, `frame_id=camera_color_optical_frame`, stable
  `topic hz`.

### 2.2 Camera/SLAM Overloaded Wi-Fi/Pi

- Symptom: RViz image lag, depth stream drops, RTAB-Map loses frames, teleop
  slows down.
- Cause: RGB-D + SLAM + RViz cloud are heavy for Wi-Fi/Pi.
- Fix: run SLAM on laptop, RealSense around `6 fps`, disable heavy RViz displays.
- Check: `topic hz` for color/depth/map/map_odom_correction.

### 2.3 RTAB-Map Correction Became Stale

- Symptom: dashboard shows stale `map->odom`; `/map_odom_correction age` grows.
- Cause: RTAB-Map should not publish `map` TF directly; relay owns correction.
- Fix: `map_odom_relay`, RTAB-Map `publish_tf_map:=false`, last-good correction.
- Check: `/mapGraph` and `/map_odom_correction` publish.

### 2.4 RViz Could Mislead about Local Costmap

- Symptom: local costmap/pink zone not visible in RViz.
- Cause: display config, not necessarily disabled costmap.
- Fix: check ROS graph/lifecycle/topic list, not only RViz.
- Check: `controller_server`, `planner_server`, `bt_navigator`, `*costmap*`
  topics.

## 3. Detector / Perception

### 3.1 YOLOE Was Noisy in Office Scenes

- Symptom: `chair` detected on cabinet panels/handles; broad `DETECT_ALL` added
  noise.
- Cause: open-vocabulary YOLOE was too noisy for low camera viewpoint and office
  furniture.
- Fix: main hardware/VLM mode switched to `model_mode:=dino`.
- Status: YOLOE remains only as legacy/comparison mode.

### 3.2 DINO Target Threshold Raised

- Symptom: false `office chair` around confidence `0.5`.
- Cause: weak single detections were accepted too easily.
- Fix: `target_conf_default` / `target_detect_conf` raised to `0.60`.
- Check: `/detect_target` with `conf_threshold: 0.60`.

### 3.3 Single-Frame False Target

- Symptom: one false detection became the target.
- Cause: single-frame detection was considered enough.
- Fix: `target_confirm` requires multiple observations.
- Log: `target_confirm[target]: raw -> confirmed`.

### 3.4 Context Noise Must Not Become a Target

- Symptom: `desk`, `drawer cabinet`, or weak context `office chair` became a
  destination.
- Cause: early logic mixed context and target candidates.
- Fix: context is not promoted into target candidates.
- Rule: `context_marks are not destinations`.

## 4. VLM Planning Logic

### 4.1 Context Objects Only Choose Corridors

- Symptom: robot drove to a cabinet/table to "search for a chair".
- Cause: prompt encouraged approaching office objects.
- Fix: context objects set direction relevance, not approach goals.
- Rule: if the target is absent, choose a free corridor; context helps choose
  between corridors.

### 4.2 Initial Scan Forward/Right/Left

- Symptom: without a target in the first frame, direction choice looked random.
- Cause: VLM had no comparable directional observations.
- Fix: structured scan: `forward -> right -> left -> choose corridor`.
- Log: `CORRIDOR_SCAN[forward/right/left]`.

### 4.3 Pause after Turn

- Symptom: DINO detected noise on blurred images after turns.
- Cause: observation began before robot/camera stabilized.
- Fix: `turn settle: waiting 2.00s before next observation`.
- Check: after `TURN`, a `turn settle` entry appears.

### 4.4 Micro-Turns Are Useless

- Symptom: `TURN +0.17rad` / `TURN +0.30rad`; Nav2 completed immediately.
- Cause: action fell inside tolerance and barely changed the frame.
- Fix: semantic scan turns normalize to about `0.60rad+`.
- Note: this applies to VLM scan actions, not internal controller corrections.

### 4.5 Edge Target Must Not Trigger New Initial Scan

- Symptom: target found at image edge, temporarily lost after recenter, then full
  scan started.
- Cause: `initial_scan` ran before recovery from recent target.
- Fix: `target_lock_recovery` placed before `initial_scan`.
- Test: `test_recent_edge_target_lock_recovery_preempts_initial_scan`.

### 4.6 Scene Exploration Must Be Active

- Symptom: robot spun in place and inspected the same local patch.
- Cause: logic over-weighted local furniture/context marks.
- Fix: after scan and one meaningful turn, prefer moving through free/unknown
  corridors.
- Check: after `CORRIDOR_SCAN`, expect `DRIVE_FORWARD`/safe-forward steps when
  the corridor is safe.

## 5. Approach / Nav2

### 5.1 Target Was Lost during Approach

- Symptom: `DRIVE_TO_VISIBLE`, then `0 target detection(s)`, then fallback/explore.
- Cause: while approaching, object leaves the frame, becomes occluded, or no
  longer fits in view.
- Fix: `target_lock` + `target_nav_lock`.
- Log: `target_nav_lock: target was already confidently localized`.

### 5.2 Goal Outside Known-Free Map

- Symptom: target visible, but Nav2 cannot plan.
- Cause: camera sees farther than the current SLAM map.
- Fix: bounded approach to the nearest safe point.
- Log: `bounded_step=... bounded_goal=known_free`.

### 5.3 No Safe Bounded Approach

- Symptom: `ABORTED (no safe bounded approach; final_goal=... last_bounded=...)`.
- Cause: final and bounded points are occupied/unknown/too close to obstacle.
- Fix: `target_approach_blocked` recovery, then retry locked target.
- Parameter: `target_approach_blocked_forward_m:=0.55`.

### 5.4 Safe-Forward Was Too Strict about Known-Free

- Symptom: VLM chooses `DRIVE_FORWARD`, but Pi reports
  `safe_forward ABORTED ... clearance_unknown`.
- Cause: frontier corridor is still unknown in the online SLAM map.
- Fix: short `clearance_unknown` motion is allowed for exploration.
- Parameters: `goto_safe_forward_allow_unknown:=true`,
  `goto_safe_forward_unknown_max_step_m:=0.60`.

### 5.5 Close Furniture in Swept Path

- Symptom: close furniture on the side, VLM drives forward, robot clips a table.
- Cause: image side does not guarantee object is outside footprint.
- Fix: close `center <0.8m` and `left/right <0.55m` block forward.
- Reaction: blocker left -> search right; blocker right -> search left.

### 5.6 Nav2 Spun near the Goal

- Symptom: robot is near object, but controller keeps adjusting.
- Cause: goal is close to occupied chair/table legs or incomplete online map.
- Fix: direct/bounded approach, safe bounded point, and original `approach_offset`.
- Note: `approach_offset` is measured from `base_link`; the front edge has a
  physical margin.

## 6. Collision Monitor

### 6.1 Timestamp/TF Problems

- Symptom: `Lookup would require extrapolation into the future`,
  `Robot to stop due to invalid source`.
- Cause: skew between `/scan`, camera, odom, and TF under load.
- Status: disabled by default during VLM debugging; re-enable after time/TF
  stabilization.
- Check: lifecycle active, fresh `/scan`, valid cmd_vel chain.

### 6.2 Collision Monitor Cuts Speed

- Symptom: `/cmd_vel_out` exists, but `/cmd_vel_collision_safe` is smaller.
- Cause: safety slowdown near obstacle.
- Conclusion: normal when sources are valid; bad only if invalid source spams
  stop.

## 7. Logs / Metrics

### 7.0 FLAT Initial Scan

- Problem: without scan, FLAT loses to VLM because it does not look around, not
  because of semantic reasoning.
- Fix: FLAT received fixed non-semantic overview `forward -> right -> left`
  before `ExploreFrontier` when the target is absent in the starting frame.
- Difference from VLM: FLAT does not choose corridors from context objects; scan
  only expands view/map.

### 7.1 Persistent Mission Logs

- `vlm_mission_logger`: `/vlm/activity` -> JSONL + CSV.
- `flat_mission_logger`: `/mission/status` -> JSONL + CSV.
- File name equals `run_id`.
- VLM timing: `latency_ms` for step planning and `time_to_first_action_s` from
  `mission_start` to first `step_start`.
- FLAT decision/perception timing: `detector_runtime_mean_s` from continuous
  tracker (`/experiment/cv_runtime`), averaged DINO+SAM time per frame.
- FLAT behavior timing: `time_to_detect_s` and `time_to_approach_s`; these
  include scan/turns and are not directly compared with VLM `latency_ms`.

Files:

```text
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.csv
~/ros2_ws/experiment_logs/flat_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/flat_missions/<run_id>.csv
```

### 7.2 FLAT Progress Rate

- `DONE -> 1.00`
- `FAILED after APPROACH/DETECT -> 0.66`
- `FAILED after SEARCH -> 0.33`
- `no useful progress -> 0.00`
- `success_auto=1` only for terminal `DONE`; `success_manual` remains for manual
  labeling.

### 7.3 VLM Good-Run Pattern

```text
1. initial_scan: forward/right/left
2. CORRIDOR_SCAN for directions
3. strict target found? -> DRIVE_TO_VISIBLE
4. no target? -> choose free/unknown corridor
5. safe-forward through corridor
6. target found -> target_lock + DRIVE_TO_VISIBLE
7. target lost -> target_nav_lock
8. approach blocked -> recovery + retry locked target
9. final approach reached -> auto_done
```

## Open Improvements

- Make `target_approach_blocked` recovery more directional.
- Evaluate corridors structurally from the map, not only from prompt + map image.
- Filter DINO context noise more strongly without losing useful office cues.
- Re-enable Collision Monitor by default after TF/time stabilization.
- Add compact Nav2/ApproachDetection summary next to VLM mission logs.
