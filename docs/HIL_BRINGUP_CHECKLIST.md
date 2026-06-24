# HIL bring-up checklist — connecting `robust` to the real robot

Everything in `robust` is validated in Gazebo. This is the procedure for the gates
that **simulation physically cannot cover** — CAN/EPOS4 motor safety, real time-sync
jitter, RealSense + Pi CPU budget, and field degradation. Do these **in order**; do
NOT run an autonomous mission before the safety section (B) is green.

Tiers: **Pi** runs the executive (`search_coordinator`) + ros2_control HW interface
(`embodied_robot_system`) + RealSense + local `/scan` + lightened Nav2 + `map_odom_relay`.
**Edge** (GPU box) runs RTAB-Map SLAM + `detect_target_server` (YOLOE) + `planner_orchestrator`.

---

## A. Prerequisites
- [ ] Robot on a **stand with wheels off the ground** for all of section B (motors will move).
- [ ] Physical e-stop within reach and tested before energizing.
- [ ] CAN wiring + termination verified; EPOS4 IDs match `config/epos4_diffdrive/bus.yml`
      (node ids, `.eds` = `maxon_motor_EPOS4_*`). `ip link` shows `can0` up at the bus bitrate.
- [ ] `use_sim_time:=False` everywhere on hardware (ROADMAP 0.1 ✅ — confirm at runtime,
      not just in yaml).

## B. Motor safety (HIL) — gates ROADMAP 0.2/0.3/0.4/0.7 + 5.5. **Wheels off ground.**
1. [ ] **CAN/EPOS4 up** (6.2): bring up `hardware_bringup.launch.py`, confirm the lely
       master reaches **Operation Enabled** on both drives (statusword `0x6041`), wheels
       commandable via a tiny `/cmd_vel`.
2. [ ] **Quick-stop latch (0.2)** — the sim-unverifiable one. Trigger `request_quick_stop()`
       (e.g. publish `/quick_stop_trigger`) **while a wheel is spinning** and confirm on a
       CAN sniffer that the drive enters **Quick Stop Active** (statusword), not merely
       `0x60FF`=0. If `handleWrite()` overwrites controlword back to `0x000F`, switch to the
       driver's `halt()`/CiA-402 quick-stop API (noted in `embodied_robot_system.cpp`).
3. [ ] **Per-cycle fault reaction (0.3)** — induce an EPOS4 fault (e.g. brief overcurrent /
       pull a feedback connector) mid-spin; confirm the fault-bit poll triggers a coordinated
       quick-stop on **both** wheels within ≤100 ms (decimation=5 @50 Hz).
4. [ ] **CAN bus-off (0.4 / 5.5)** — physically yank `can0` (or inject bus-off); confirm
       commands zero safely and a controlled NMT recovery (currently the latch is the reaction
       point — verify behavior and implement recovery if the link API allows).
5. [ ] **Stop.action → hardware (0.7)** — send `Stop` with `QUICK_STOP_REQUEST`; confirm it
       reaches the hardware quick-stop and the server reports `zero_velocity_confirmed` (via
       `/odometry/filtered`).
6. [ ] **cmd_vel watchdog (0.5)** — stop publishing `/cmd_vel`; confirm zero + HOLD < 0.5 s
       (sim ✅; reconfirm on hardware).
7. [ ] **Collision monitor stop-latency (0.6)** — obstacle in front during a slow drive;
       confirm slowdown→stop and measure stop-latency **< 200 ms** (ROADMAP Phase 0 EXIT).

→ **Phase 0 EXIT closes here.** Do not proceed to autonomy until B1–B7 pass.

## C. Time sync (HIL) — gates ROADMAP 1.2 / Phase 1 EXIT
- [ ] Deploy `deploy/time_sync/chrony-edge.conf` (edge master) + `chrony-pi.conf` (Pi).
- [ ] Run `check_offset.sh` on the Pi over real Wi-Fi; **prove offset+RMS ≤ 0.02 s** — i.e.
      ≪ the 0.2 s (TF) / 0.35 s (depth-match) / 1.5 s (pixel-age) windows.

## D. Transport (HIL) — gates ROADMAP 1.1 / Phase 1 EXIT
- [ ] Start the single `rmw-zenoh-router.service` on edge (multicast off, 12 MB buffers).
- [ ] Confirm cross-host pub/sub Pi↔edge over Wi-Fi; measure jitter within budget. Fallback
      Fast DDS LARGE_DATA + Discovery Server is wired if zenoh misbehaves.

## E. Perception + Nav on Pi — gates ROADMAP 6.3
- [ ] `realsense_rgbd_pi.launch.py` up; depth → `depthimage_to_laserscan` → local `/scan`
      (NO raw depth/PointCloud2 over Wi-Fi). Confirm `/scan` rate ~10–15 Hz.
- [ ] EKF (`ekf_*` / `imu_filter_madgwick`) fuses wheel odom + RealSense IMU → `/odometry/filtered`.
- [ ] Lightened Nav2 (NavFn+DWB, 10 Hz) + executive fit the **Pi 5/4 GB CPU budget** — profile
      `top`/`ros2 ... --use-sim-time false`; this is the real "Pi-class" profiling deferred from 2.7/2.10.

## F. SLAM (edge) — gates ROADMAP 6.4
- [ ] RTAB-Map offline mapping → `.db`; online localization publishes `MapOdomCorrection`
      to the Pi; `map_odom_relay` applies it (seq/jump/reloc/stale gates already unit-tested).

## G. FLAT mission on hardware — gates ROADMAP 6.5 (first autonomy)
- [ ] `seek_object` with `allow_vlm:=false`: SEARCH (frontiers) → DETECT (`detect_target_server`
      on edge → `/target_pixel`) → APPROACH (Nav2). Compare end-to-end timing to the sim
      baseline (`docs/FLAT_BASELINE.md`) within tolerance.

## H. VLM mission on hardware — gates ROADMAP 6.5
- [ ] Same with `allow_vlm:=true` + `planner_orchestrator` (VLM creds in env: `VLM_BASE_URL`/
      `VLM_API_KEY`/`VLM_MODEL`). Confirm the Set-of-Mark → `DRIVE_TO_VISIBLE(mark_id)` loop drives.

## I. Field degradation — gates ROADMAP 6.6 (+ live 5.1/5.4 on metal)
- [ ] Physically cut Wi-Fi / kill the edge **mid-VLM-mission**; confirm **seamless VLM→FLAT**
      (5.1) — mission continues as FLAT, result `DEGRADED_SUCCESS`, **no false `reached`** on a
      stale pixel (5.4). This is the hardware repeat of the sim-validated degradation.

## EXIT (Phase 6)
Robot runs both FLAT and VLM missions on real hardware; every safety/degradation scenario
reproduces in the field; baseline metrics match sim within tolerance.

---
### Sim status feeding this checklist (already green on `robust`)
Phase 0 (sim parts: `use_sim_time`, cmd_vel watchdog, collision monitor) ✅ · Phase 1
(`/scan` local, zenoh single-host, QoS+Heartbeat) ✅ · Phase 2 FLAT executive ✅ · Phase 3
perception + Set-of-Mark (live YOLOE) ✅ · Phase 4 VLM planner (live qwen3-vl, async replan) ✅ ·
Phase 5 FMEA (seamless VLM→FLAT live; 5.4/5.6/5.7 tests) ✅. Open sim item: cross-host jitter
(needs 2 hosts) — closes in C/D above.
