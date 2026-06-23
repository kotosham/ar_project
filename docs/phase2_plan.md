# Phase 2 — ZERO-VLM FLAT baseline: implementation plan

The Phase-2 gate. Derived from a planning workflow (architect + adversarial
critic). The critic returned **needs_revision**; the must-fixes below are folded
into this plan, so this doc — not the raw plan — is the source of truth.

## Architecture

All executive logic lives in one rclpy node, **`search_coordinator`** (ar_project),
on a `MultiThreadedExecutor` with distinct `ReentrantCallbackGroup`s (FSM tick,
each skill server, subscriptions) so the in-process loopback (the FSM is a client
of its own skill servers) cannot deadlock. **No callback may block** — drive loops
and "wait for zero velocity" are timer/async polled. **`map_odom_relay`** is a
separate node in the same package. The executive **never publishes `cmd_vel`** —
all motion flows Nav2 → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel` →
watchdog → twist_mux → collision_monitor (safety chain untouched).

- ENTRY: `SeekObject` action server (object_tracking_msgs) — the only mission
  entry point; owns mission_state, mission_epoch, the FSM.
- SKILL servers (all in search_coordinator, ar_project_msgs): `ExploreFrontier`,
  `GoToPose`, `ApproachDetection`, `GetObservation`, `Stop`. FSM drives them via
  loopback action clients (so each is testable with `ros2 action send_goal`).
- `GoToPose`/`ExploreFrontier`/`ApproachDetection` drive via Nav2
  `navigate_to_pose`. Reuse `fleet_comms` QoS + Heartbeat (Phase 1.3).

## FSM states (shared constant `executive_fsm.STATE`)

`IDLE, SEARCH, DETECT, APPROACH, STOP, DEGRADED, DONE, FAILED` (DRIVE folded into
skills; REPLAN reserved for Phase 4, never entered in FLAT). Published verbatim in
`SeekObject` feedback.state.

**Invariants:** (1) committed-subgoal — exactly one committed subgoal
`{skill,args,step_id-UUID,epoch}` while a SeekObject goal is active, adopted only
at a commit point; (2) default-productive-action — `_select_subgoal()` never
returns None while active; default = EXPLORE_FRONTIER, falling back to STOP(HOLD)
+ terminate only when frontiers are exhausted. Never idle-spin, never reactive cmd_vel.

Walk: IDLE →(goal)→ SEARCH →(fresh target pixel)→ DETECT →(3D goal computable)→
APPROACH →(reached & fresh)→ DONE. APPROACH STALE_DETECTION/LOST_TARGET → SEARCH
(never false-reached). ExploreFrontier NO_FRONTIER → FAILED. New instruction → STOP
→ reset (epoch++).

## Skill server contracts

- **ExploreFrontier** — resolve `frontier_id` (-1 ⇒ best after hysteresis; ≥0 ⇒
  stable id) → PoseStamped → Nav2. Feedback: distance_remaining, selected_frontier_id,
  frontier_score. NO_FRONTIER if list empty. Honors max_travel_m.
- **GoToPose** — forward target_pose + tolerances to Nav2; map result.
- **ApproachDetection** (FMEA) — subscribe `/target_pixel`; gate pixel age vs
  `max_pixel_age_s` (1.5). SUCCEEDED **only** when Nav2 reached the pose I set
  **AND** last pixel was fresh. Never reached while `detection_fresh==false`.
  STALE_DETECTION (pixels arrive but all stale) / LOST_TARGET (no pixel) abort
  instead of driving or false-success. No goal_locked latch.
- **GetObservation** — Phase-2 functional stub: capture one CompressedImage if a
  real source exists (else leave `view` empty — see must-fix #3), `candidates`
  empty (detector is Phase 3).
- **Stop** — idempotent. SOFT_STOP: cancel Nav2; stop comes from velocity_smoother
  input-timeout zeroing + watchdog (NOT a decel ramp — see must-fix #4).
  HOLD: cancel + latch. QUICK_STOP_REQUEST: also fire the external hardware
  quick-stop trigger (closes ROADMAP 0.7), honored regardless of epoch.
  `zero_velocity_confirmed` once `/odometry/filtered` speed < eps. Strongest
  idempotency: resend same request_id = no-op (cached terminal result).

## Mission-epoch + UUID idempotency (FMEA 2.5)

SeekObject is the epoch authority. New instruction → ABORT-AND-RESET: cancel all
in-flight skill + Nav2 goals, `mission_epoch++` (uint32 wrap-safe), clear committed
subgoal + frontier commit state + dedup tables, re-arm at SEARCH; old SeekObject
handle finalized PREEMPTED. Every dispatched skill goal stamped with current epoch
+ fresh step_id UUID. Servers epoch-gate on acceptance (reject mismatched =
zombie). Results filtered by tagging each pending handle with its dispatch epoch.
Dedup dict `{request_id: handle/result}` per server; repeated request_id in same
epoch = no-op.

## Frontier extractor (FMEA 2.3) — CORRECTED SOURCE

**Source decision (must-fix #1):** NOT `/local_costmap/costmap_raw` (rolling 3×3,
no `track_unknown_space` → no UNKNOWN cells → zero frontiers). Use the **SLAM
occupancy grid** (RTAB-Map `/map`, `nav_msgs/OccupancyGrid`, frame `map`,
-1=unknown / 0=free / 100=occupied) — the standard explore-lite source. **Must be
empirically verified** in sim before building: confirm `/map` is published with
unknown cells; if RTAB-Map isn't producing a grid in `launch_sim`, enable it.
ExploreFrontier goals are then in `map` frame (Nav2 global_frame=map). Frontier =
free cell with a 4-neighbour unknown; cluster (≥ `min_frontier_cells`); score =
`w_size*size − w_dist*dist − w_turn*heading_change`; stable ids by centroid
quantization; top-K in-memory + `/frontiers/markers` for RViz.

**Hysteresis:** don't switch off the committed frontier unless a competitor beats
it by `switch_margin` (15%) AND it has been committed ≥ `min_dwell_s` (4 s);
bypass only when the committed frontier vanishes (progress, not oscillation).
Params declared on the node. Needs a two-near-equal-frontier world (must-fix #10).

## FLAT detect path — CORRECTED (must-fix #2/#3)

`/target_pixel` flows only after a label reaches `/target_prompt`, and
`reliable_prompt_sender` is being deleted (2.9). So: the **FSM publishes
`SeekObject.instruction` → `/target_prompt`** (a small bridge inside the executive)
so the existing tracker tracks the requested object. **QoS fix:** the consumer
subscription for `/target_pixel` must NOT request a deadline the BEST_EFFORT/
no-deadline publisher can't offer (would be Request-vs-Offered incompatible →
silent zero samples). Either add a `detection_stream_nodeadline()` variant or
add a matching offered deadline on the tracker publisher; add an `is_compatible`
unit test for this exact pair.

## map_odom_relay (2.6)

`search_coordinator/map_odom_relay.py`. Subscribe `/map_odom_correction`
(`correction_lowrate` QoS); broadcast map→odom on /tf at ~10 Hz (< 0.2 s
transform_tolerance), holding last-good (identity until first correction). Gates:
stale-by-seq, stale-by-stamp (`max_correction_age_s` 1.0), covariance/fitness,
jump (`max_jump_m`/`max_jump_rad`; accept iff `relocalized==true`). **must-fix #7:**
parameterize RTAB-Map `publish_tf_map` (hardcoded `'true'` at
`rtabmap_rgbd_launch.py:278`) to a LaunchConfiguration, set `false` when the relay
runs (no duplicate broadcaster).

## Nav2 lightening (2.7)

`controller_frequency` 15→10; `expected_planner_frequency` 20→1; drop
`smoother_server`+`waypoint_follower` (params + lifecycle_nodes + Nodes); trim
behavior_plugins to spin/backup/wait; delete dead local static_layer block. Keep
NavFn(use_astar:false) + DWB, local costmap in odom, /scan obstacle source. Do NOT
hardcode use_sim_time (RewrittenYaml still rewrites). **must-fix #12:** before
deleting `map_server`, confirm what feeds `global_costmap` static_layer (RTAB-Map
`/map`?) so the global costmap still autostarts.

## Deletions (2.9)

`reliable_prompt_sender.py` + its launch + CMakeLists install entry; the
target_pixel_to_goal "soup" (goal_locked, lock_goal_on_publish, final_approach_freeze,
nav_status_callback, prompt_ack, goal_locked topic) keeping only the geometry
(moved to `approach_geometry.py` in 2.8); tracker_node reactive cmd_vel +
target_found/reached state machine. **must-fix #6:** also handle orphaned consumers
(`home_pose_manager.py`, `experiment_metrics_logger.py` subscribe `/goal_pose`,
`/target_prompt`) and the launch files referencing the gutted scripts.

## Corrected task order

1. **T2.8** geometry → `approach_geometry.py` (pure, pytest). *no deps*
2. **T2.6** map_odom_relay + RTAB-Map publish_tf_map param (mock-verify). *no deps*
3. **T2.7** Nav2 lightening (+ verify global_costmap source). *no deps*
4. **Frontier source spike** — empirically confirm `/map` (or chosen grid) emits
   unknown cells in sim; then **T2.3** frontier extractor + oscillation world.
5. **Prompt bridge + /target_pixel QoS fix** (+ is_compatible test).
6. **T2.4** skill servers — include `mission_state.py` epoch authority + getter
   here (resolves the T2.4↔T2.5 circularity; build with real epoch infra).
7. **T2.5** full mission-epoch behavior + HeartbeatMonitor epoch filtering
   (`set_mission_epoch` + epoch check in `_on_msg` — new work, must-fix #8).
8. **T2.2** executive FSM + SeekObject server.
9. **T2.9** deletions (after geometry rehomed + FSM live).
10. **T2.10** end-to-end FLAT scenario + Pi-profile measurement → freeze baseline.

## Key risks (from critic)

- Frontier source must be empirically proven before T2.3 (else SEARCH dead-ends).
- /target_pixel QoS + prompt bridge must land before any APPROACH can succeed.
- Loopback action servers: distinct callback groups, never block in a callback.
- Pi profiling under WSL is taskset/cpulimit emulation → relative gate only;
  re-measure on hardware in Phase 6.3.
