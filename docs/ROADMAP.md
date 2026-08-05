# Robust Architecture Roadmap (`robust` branches)

Implementation order is FIX-FIRST: safety and correctness first, then transport
and simulation, then the zero-VLM FLAT baseline, then perception as a service,
VLM mode, hardening, and hardware bringup. Each phase has an EXIT criterion.
FMEA-critical fixes are marked `[FMEA]`.

## Phase 0 - Safety and Correctness

- [x] Parameterize `use_sim_time` across YAML/launch. Hardware default is `false`,
  Gazebo launch sets `true`.
- [x] `[FMEA]` Implement real CiA-402 quick-stop on the RT `write()` path without
  blocking SDO.
- [x] `[FMEA]` Poll fault status regularly and trigger coordinated quick-stop.
- [ ] `[FMEA]` CAN bus-off detection and recovery.
- [x] `cmd_vel` watchdog before twist_mux.
- [x] Nav2 Collision Monitor on local `/scan`.
- [ ] Confirm `Stop.action` with `QUICK_STOP_REQUEST` reaches hardware quick-stop.
- [ ] **EXIT:** stop/degradation produces confirmed stop below 200 ms in Gazebo
  and HIL; artificial fault and bus-off trigger quick-stop.

## Phase 1 - Transport + Simulation Skeleton

- [x] Single `rmw_zenoh` router on edge, multicast off, 12 MB socket buffers, Fast
  DDS fallback documented.
- [x] chrony configs prepared; real offset proof requires Pi + edge.
- [x] Named QoS profiles and `/heartbeat` infrastructure in `fleet_comms`.
- [x] Local `/scan` through `depthimage_to_laserscan`; costmap no longer depends
  on raw depth/PointCloud2.
- [x] Gazebo-on-WSL bringup and test worlds.
- [x] Interface/logical package skeletons build.
- [ ] **EXIT:** cross-host zenoh jitter/offset measured within budget on Pi+edge.

## Phase 2 - ZERO-VLM FLAT Baseline

- [x] Define skill actions and `MapOdomCorrection`.
- [x] Implement executive FSM / SeekObject server.
- [x] `[FMEA]` Local frontier extractor from SLAM `/map` with hysteresis.
- [x] Implement skill servers: Explore, GoTo, Approach, GetObservation, Stop.
- [x] Mission epoch + UUID idempotency.
- [x] `map_odom_relay` with last-good, stale/jump/covariance gates.
- [x] Light Nav2 profile and local `/scan`.
- [x] Move approach geometry into `approach_geometry.py`.
- [x] Remove `reliable_prompt_sender` and old latched state soup.
- [x] End-to-end FLAT scenario and baseline record.
- [x] **EXIT:** FLAT can search, detect, and approach without VLM; no frontier
  oscillation; instruction reset works; baseline frozen.

## Phase 3 - Perception as a Service

- [x] `DetectTarget.action` and `Candidate.msg`.
- [x] Edge `DetectTarget` server.
- [x] Set-of-Mark rendering.
- [x] `[FMEA]` stale detection never produces approach success.
- [x] `GetObservation` returns compressed view + candidates.
- [x] **EXIT:** detector works as request service with Set-of-Mark; stale pixels
  cannot latch false reached.

## Phase 4 - VLM Mode

- [x] `SeekObject.action`, `PlanStep.msg`, `Notes.msg`.
- [x] Planner Orchestrator as external OpenAI-compatible VLM API client.
- [x] Structured/enum actions using real candidates; no invented coordinates.
- [x] Timeout and circuit breaker.
- [x] Notes/summary buffer with token budget.
- [x] Async replan with lead-time and commit-point adoption.
- [~] Hierarchical decomposition replaced by smaller atomic-action replanning by
  user decision.
- [x] **EXIT:** VLM periodically replans from history, stays off the reactive path,
  and does not idle between replans.

## Phase 5 - Degradation Hardening + FMEA Simulation Tests

- [x] `[FMEA]` Seamless VLM->FLAT degradation on VLM/edge/Wi-Fi loss.
- [x] stale TF / stale `MapOdomCorrection` tests.
- [x] localization jump gating tests.
- [x] edge loss during approach does not create false reached.
- [ ] EPOS4 bus-off/fault in full pipeline: HIL-only.
- [x] mid-mission instruction change abort/reset and no zombie UUIDs.
- [x] frontier oscillation under noise.
- [x] **EXIT:** simulation FMEA set is green except HIL-only CAN/EPOS4 items.

## Phase 6 - Hardware Bringup

- [ ] `use_sim_time=false`, real clocks, chrony on robot; offset proof over Wi-Fi.
- [ ] Bring up real CAN/EPOS4 and verify quick-stop, fault poll, bus-off recovery.
- [ ] RealSense + local `/scan` + EKF + light Nav2 on Pi 5/4 GB; confirm CPU budget.
- [ ] RTAB-Map offline mapping to `.db`, online localization to `MapOdomCorrection`.
- [ ] FLAT mission on hardware, then VLM mission through external VLM API.
- [ ] Field degradation run by physically cutting Wi-Fi/edge.
- [ ] **EXIT:** robot completes FLAT and VLM missions on hardware; safety and
  degradation scenarios reproduce in the field; baseline metrics match simulation
  within tolerance.

## Anchor Files

- `ar_project/ar_project/src/embodied_robot_system.cpp`
- `ar_project/ar_project/config/nav2_params.yaml`
- `ar_project/search_coordinator/search_coordinator/approach_geometry.py`
- `ar_project/search_coordinator/search_coordinator/seek_object_server.py`
- `object_tracking/object_tracking/object_tracking/tracker_node.py`
- `object_tracking/planner_orchestrator/planner_orchestrator/orchestrator_node.py`
