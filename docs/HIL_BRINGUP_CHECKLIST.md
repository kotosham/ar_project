# HIL Bringup Checklist - Connecting `robust` to the Real Robot

Everything in `robust` is first verified in Gazebo. This checklist covers the
items simulation cannot physically validate: CAN/EPOS4 motor safety, real
time-sync jitter, RealSense + Pi CPU budget, and field degradation. Execute the
sections in order. Do **not** run an autonomous mission until section B is green.

Roles:

- **Pi**: executive (`search_coordinator`), ros2_control hardware interface
  (`embodied_robot_system`), RealSense, local `/scan`, light Nav2, and
  `map_odom_relay`.
- **Edge**: RTAB-Map SLAM, `detect_target_server` (DINO+MobileSAM), and
  `planner_orchestrator`.

## A. Preconditions

- [ ] Robot on a stand with wheels off the ground for all of section B.
- [ ] Physical e-stop reachable and tested before power is applied.
- [ ] CAN wiring and termination checked; EPOS4 IDs match
  `config/epos4_diffdrive/bus.yml`; `ip link` shows `can0` up at the bus bitrate.
- [ ] `use_sim_time:=false` everywhere on hardware; confirm during execution, not
  only in YAML.

## B. Motor Safety (HIL). Wheels Off Ground.

1. [ ] **CAN/EPOS4 up**: start `hardware_bringup.launch.py`; confirm lely master
   reaches **Operation Enabled** on both drives and wheels respond to a tiny
   `/cmd_vel`.
2. [ ] **Quick-stop latch**: trigger `request_quick_stop()` while a wheel is
   rotating and confirm with CAN sniffing that the drive enters **Quick Stop
   Active**, not just target velocity zero.
3. [ ] **Fault reaction within one cycle**: induce an EPOS4 fault and confirm
   fault polling triggers coordinated quick-stop on both wheels within 100 ms.
4. [ ] **CAN bus-off**: physically disconnect or inject bus-off; confirm commands
   are zeroed safely and recovery behavior is controlled.
5. [ ] **Stop.action -> hardware**: send `Stop` with `QUICK_STOP_REQUEST`; confirm
   hardware quick-stop and `zero_velocity_confirmed`.
6. [ ] **cmd_vel watchdog**: stop publishing `/cmd_vel`; confirm zero/HOLD within
   0.5 s.
7. [ ] **Collision Monitor stop latency**: place an obstacle during slow driving;
   confirm slow-down/stop and measure stop latency below 200 ms.

Phase 0 EXIT closes here. Do not continue to autonomy before B1-B7 pass.

## C. Time Synchronization (HIL)

- [ ] Deploy `deploy/time_sync/chrony-edge.conf` on edge and `chrony-pi.conf` on Pi.
- [ ] Run `check_offset.sh` on Pi over real Wi-Fi and prove offset+RMS <= 0.02 s,
  well below the 0.2 s TF, 0.35 s depth-match, and 1.5 s pixel-age windows.

## D. Transport (HIL)

- [ ] Start the single `rmw-zenoh-router.service` on edge with multicast disabled
  and 12 MB buffers.
- [ ] Confirm inter-host Pi-edge pub/sub over Wi-Fi and measure jitter within
  budget. Keep Fast DDS LARGE_DATA + Discovery Server as fallback.

## E. Perception + Navigation on Pi

- [ ] `realsense_rgbd_pi.launch.py` up; depth -> `depthimage_to_laserscan` ->
  local `/scan` without raw depth/PointCloud2 over Wi-Fi. Confirm `/scan` about
  10-15 Hz.
- [ ] EKF fuses wheel odometry + RealSense IMU into `/odometry/filtered`.
- [ ] Light Nav2 (NavFn+DWB, 10 Hz) + executive fit the Pi 5 / 4 GB CPU budget.

## F. SLAM on Edge

- [ ] RTAB-Map offline mapping creates `.db`; online localization publishes
  `MapOdomCorrection` to Pi; `map_odom_relay` applies it.

## G. FLAT Mission on Hardware

- [ ] Run `seek_object` with `allow_vlm:=false`:
  SEARCH -> DETECT (`detect_target_server` on edge -> `/target_pixel`) -> APPROACH
  (Nav2). Compare end-to-end timing with `docs/FLAT_BASELINE.md`.

## H. VLM Mission on Hardware

- [ ] Run through `planner_orchestrator` on edge using VLM credentials from env:
  `VLM_BASE_URL`, `VLM_API_KEY`, `VLM_MODEL`. Mission start goes through
  `/seek_object allow_vlm:=true`, which internally publishes `/vlm_mission`.
- [ ] Confirm the observation/action cycle controls the robot:
  `DRIVE_TO_VISIBLE(mark_id)`, `DRIVE_FORWARD`, `TURN`, `DETECT_ALL`, `DONE`.
- [ ] Launch commands and parameters are in `RUNBOOK.md` sections 4a/4c. The
  target may be a direct label or a riddle-like semantic query; the query resolver
  normalizes it before detection.

## I. Field Degradation

- [ ] Cut Wi-Fi or terminate edge during a VLM mission. Confirm seamless
  VLM-to-FLAT degradation, mission continuation as FLAT, and no false `reached`
  from stale pixels.

## EXIT (Phase 6)

The robot completes both FLAT and VLM missions on real hardware. Every
safety/degradation scenario is reproduced in the field, and baseline metrics
match simulation within tolerance.

## Simulation Status Feeding This Checklist

Phase 0 simulation pieces (`use_sim_time`, `cmd_vel` watchdog, collision monitor)
are green. Phase 1 local `/scan`, single-host zenoh, QoS+Heartbeat are green.
Phase 2 FLAT executive is green. Phase 3 perception + Set-of-Mark with live
DINO+MobileSAM is green. Phase 4 VLM planner with live qwen3-vl async replan is
green. Phase 5 degradation tests are green. Remaining simulation gap:
inter-host jitter, which requires two hosts and is closed in sections C/D.
