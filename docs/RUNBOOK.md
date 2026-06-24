# RUNBOOK — how to build, bring up, and run (sim + hardware)

Detailed operating guide for the `robust` stack. For the hardware **safety** gates
see `HIL_BRINGUP_CHECKLIST.md`; for measured numbers see `FLAT_BASELINE.md`; for
build/deploy automation see `deploy/build/README.md`.

## 0. Architecture (who runs what)
- **Pi (robot):** executive `search_coordinator` (SeekObject FSM + 5 skill servers +
  `frontier_extractor`) · ros2_control HW interface (`embodied_robot_system`, CAN/EPOS4) ·
  RealSense · local `/scan` (depthimage_to_laserscan) · lightened Nav2 · `map_odom_relay`.
- **Edge (GPU box):** RTAB-Map RGB-D SLAM · `detect_target_server` (YOLOE, in the venv) ·
  `planner_orchestrator` (VLM). The VLM model itself is an external OpenAI-compatible API.
- **Two modes:** FLAT (zero-VLM, executive autonomous) and VLM (orchestrator drives the
  executive's skills, degrades back to FLAT on loss).

---

## 1. Prerequisites (once per machine)
- ROS 2 **Jazzy** + colcon + rosdep. `sudo rosdep init && rosdep update` once.
- Detector venv on the edge (torch needs it; the node's system shebang has no torch):
  ```bash
  python3 -m venv --system-site-packages ~/ot_venv
  source ~/ot_venv/bin/activate
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
  pip install -r object_tracking/requirements.txt
  ```
  YOLOE weights live in `object_tracking/object_tracking/model_weights/`
  (`yoloe-11s-seg.pt` + `mobileclip_blt.ts`).
- VLM credentials (VLM mode only): copy `object_tracking/planner_orchestrator/vlm.env.example`
  → `vlm.env`, fill `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`. Load before launching the
  orchestrator: `set -a; source vlm.env; set +a`.

## 2. Build
- **Sim (single box):** `colcon build` in your workspace, then `source install/setup.bash`.
- **Pi + edge (real robot):** from `ar_project/deploy/build/`: `make setup` (fill
  `deploy.env`), then `make all` — builds the edge set locally and rsync+remote-builds the
  Pi set. `make doctor` checks SSH/ROS first.

---

## 3. SIMULATION (the fully-tested path)

### 3a. FLAT mission (zero VLM) — one command
```bash
# T1: bring up the whole FLAT stack (sim -> SLAM -> Nav2 -> executive)
ros2 launch ar_project flat_sim_bringup.launch.py
#   default world = oscillation.world; override: world:=$(ros2 pkg prefix ar_project)/share/ar_project/worlds/flat_detect.world
```
Wait ~35 s until you see `search_coordinator up (Phase 2.2) ... epoch=0`.
```bash
# T2: bootstrap the map in a bounded world (gives SLAM unknown cells -> frontiers).
#     net-zero in-place rotation; robot ends where it started.
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}" &  sleep 5
kill %1; ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: -0.6}}" &  sleep 5; kill %1
ros2 topic echo /frontiers --once          # expect a non-empty list

# T2: start the FLAT mission (allow_vlm:=false)
ros2 action send_goal /seek_object object_tracking_msgs/action/SeekObject \
  "{instruction: 'find bus', request_id: 'm1', mission_epoch: 0, allow_vlm: false}" --feedback
```
The FSM runs SEARCH (drives to frontiers) → on a fresh `/target_pixel` → DETECT → APPROACH.

### 3b. Real detection (DETECT leg) — run the detector on the edge venv
Use the `flat_detect.world` (it has the bus.jpg billboard YOLOE reliably detects):
```bash
~/ot_venv/bin/python $(ros2 pkg prefix object_tracking)/lib/object_tracking/detect_target_server \
  --ros-args -p use_sim_time:=true
```
The executive's DETECT/APPROACH consumes `/target_pixel`. (Without the detector you can
inject a synthetic one for testing — see `~/inject_pixel.py` pattern in FLAT_BASELINE.)

### 3c. VLM mission
```bash
# detector running (3b) + executive up. In the orchestrator shell:
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a   # loads VLM_* (never printed)
ros2 run planner_orchestrator orchestrator_node --ros-args \
  -p use_sim_time:=true -p use_mock:=false -p replan_every_n:=3 -p max_steps:=40
# expect: "planner_orchestrator up ... client=OpenAICompatibleClient creds=env"
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: bus}"   # start the VLM mission
```
The orchestrator pulls real Set-of-Mark candidates from `detect_target_server`, the VLM picks
`DRIVE_TO_VISIBLE(mark_id)` / `GO_TO_FRONTIER`, and dispatches to the executive skills. Replans
overlap execution (4.6). For an offline run use `-p use_mock:=true` (no API key needed).

> RAM note: gz + RTAB-Map + Nav2 + YOLOE together need >4 GB. On a ≤4 GB host run the
> detector standalone (3b world) OR the nav stack, not all at once.

---

## 4. HARDWARE (follow HIL_BRINGUP_CHECKLIST.md §B safety FIRST — wheels off ground)

### 4a. Edge box
```bash
sudo systemctl start rmw-zenoh-router.service          # deploy/transport (transport)
sudo systemctl start chrony   # chrony-edge.conf master                (deploy/time_sync)
ros2 launch ar_project rtabmap_rgbd_launch.py use_sim_time:=false       # SLAM -> MapOdomCorrection
~/ot_venv/bin/python $(ros2 pkg prefix object_tracking)/lib/object_tracking/detect_target_server \
  --ros-args -p use_sim_time:=false -p use_compressed_input:=true       # detector
set -a; source vlm.env; set +a
ros2 run planner_orchestrator orchestrator_node --ros-args -p use_sim_time:=false  # VLM (optional)
```

### 4b. Pi (robot)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ar_project hardware_bringup.launch.py        # ros2_control + CAN/EPOS4 + twist_mux
                                                         #   + collision_monitor + cmd_vel watchdog
ros2 launch ar_project realsense_rgbd_pi.launch.py       # RealSense + local /scan + EKF
ros2 launch ar_project navigation_launch.py use_sim_time:=false odom_topic:=/odometry/filtered
ros2 run search_coordinator map_odom_relay --ros-args -p use_sim_time:=false
ros2 run search_coordinator frontier_extractor --ros-args -p use_sim_time:=false
ros2 run search_coordinator coordinator_node --ros-args -p use_sim_time:=false
```
Then trigger a mission exactly as in 3a (FLAT) / 3c (VLM) but with `use_sim_time:=false`.

---

## 5. Triggering missions
- **FLAT:** `/seek_object` action, `allow_vlm: false` — the executive owns the mission.
- **VLM:** publish target on `/vlm_mission` (std_msgs/String) — the orchestrator owns it and
  drives the executive's skills. (`allow_vlm: true` on a `/seek_object` goal is the
  executive-side flag for integrated mode.)
- **Change target mid-mission:** send a new `/seek_object` goal (or new `/vlm_mission`) — the
  epoch bumps, the old mission is PREEMPTED, in-flight skill goals are rejected as zombies.

## 6. Monitoring
- `ros2 topic echo /planner/notes` — VLM compact notes + token_estimate.
- `ros2 topic echo /frontiers` — frontier list + committed id.
- `ros2 action list` / `ros2 node list` — confirm servers up.
- Heartbeats: the executive logs `/heartbeat deadline missed` when a producer (edge) is silent.
- Logs: each node prints its phase; the orchestrator logs `step N: <ACTION> (rationale)`.

## 7. Degradation (FMEA 5.1) — expected behavior
If the VLM is lost (timeout/unreachable → circuit-breaker open), the orchestrator **latches
to the FLAT MockPlanner and the mission CONTINUES (DEGRADED)** — it does not stop. On a real
edge/Wi-Fi loss, the Pi executive keeps the FLAT autonomy. ApproachDetection never declares a
false `reached` on a stale pixel (returns STALE_DETECTION / LOST_TARGET).

## 8. Troubleshooting
- **Spawn loops "Requesting list of world names" / no `/odom`:** a stale `gz sim` server —
  `pkill -9 -f 'gz sim'; pkill -9 -f ruby` and relaunch.
- **Robot doesn't move in a bounded world:** no frontiers yet — run the rotation seed (3a, T2)
  so SLAM gets unknown cells. Confirm `/frontiers` non-empty.
- **`explore_frontier: nav drive terminal=no_server`:** Nav2 not active yet — it no longer
  blacklists; it waits (`explore_nav_ready_timeout_s`) and retries. Give Nav2 time to activate.
- **Detector: `ModuleNotFoundError: torch`:** you ran it with system python — use
  `~/ot_venv/bin/python <installed detect_target_server path>`.
- **DRIVE_TO_VISIBLE doesn't drive:** the candidate needs metric depth — ensure the depth topic
  is published (`/camera/camera/aligned_depth_to_color/image_raw`) and `use_depth:=true`.
- **VLM `client=MockVlmClient` unexpectedly:** no base_url resolved — `source vlm.env` (or pass
  `-p vlm_base_url:=`) and don't pass `use_mock:=true`.
