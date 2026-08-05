# Phase 2 - ZERO-VLM FLAT Baseline Implementation Plan

This document is the Phase 2 gate. It comes from the planning workflow
(architect + adversarial critic). The critic returned `needs_revision`; the
must-fix items below are already incorporated, so this document is the source of
truth rather than the original plan.

## Architecture

All executive logic lives in one rclpy node, **`search_coordinator`**
(`ar_project`), running on `MultiThreadedExecutor` with separate
`ReentrantCallbackGroup`s for the FSM tick, each skill server, and subscriptions.
This prevents in-process loopback action calls (FSM as a client of its own skill
servers) from deadlocking.

No callback may block. Motion loops and zero-velocity waits are polled through
timers/asynchronous checks. `map_odom_relay` is a separate node in the same
package. The executive never publishes `cmd_vel`; all motion goes through:

```text
Nav2 -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> watchdog
-> twist_mux -> collision_monitor -> downstream controller
```

- ENTRY: `SeekObject` action server from `object_tracking_msgs`; the only mission
  entry point and owner of mission state, mission epoch, and FSM.
- SKILLS: `ExploreFrontier`, `GoToPose`, `ApproachDetection`, `GetObservation`,
  and `Stop` from `ar_project_msgs`. The FSM controls them through loopback
  action clients, so each can be tested with `ros2 action send_goal`.
- Motion skills use Nav2 `navigate_to_pose` and reuse `fleet_comms` QoS plus
  Heartbeat from Phase 1.3.

## FSM States

Shared constant: `executive_fsm.STATE`

```text
IDLE, SEARCH, DETECT, APPROACH, STOP, DEGRADED, DONE, FAILED
```

`DRIVE` is folded into skills. `REPLAN` is reserved for Phase 4 and is never
reached in FLAT. The value is published verbatim in `SeekObject` feedback.

Invariants:

1. A committed subgoal is exactly one `{skill,args,step_id,epoch}` while a
   SeekObject goal is active. It is accepted only at a commit point.
2. `_select_subgoal()` never returns `None` while a mission is active. The default
   productive action is `EXPLORE_FRONTIER`, falling back to `STOP(HOLD)` and
   mission completion only when frontiers are exhausted.
3. Never spin idly and never emit reactive `cmd_vel`.

Nominal flow:

```text
IDLE -> SEARCH -> DETECT -> APPROACH -> DONE
```

`APPROACH` with `STALE_DETECTION`/`LOST_TARGET` returns to SEARCH and must never
produce false success. `ExploreFrontier` with `NO_FRONTIER` fails. A new
instruction triggers STOP, reset, and epoch increment.

## Skill Server Contracts

- **ExploreFrontier**: resolves `frontier_id` (`-1` means best local frontier),
  computes PoseStamped, and drives through Nav2. It reports distance remaining,
  selected frontier, and score. Empty list returns `NO_FRONTIER`.
- **GoToPose**: forwards target pose and tolerances to Nav2 and maps the result.
- **ApproachDetection**: subscribes to `/target_pixel`, checks pixel age against
  `max_pixel_age_s` (1.5 by default), and returns `SUCCEEDED` only when Nav2
  reached the requested pose and the last detection was fresh or safely handled
  by target-lock close-range logic. It never reports reached when
  `detection_fresh==false`.
- **GetObservation**: Phase 2 functional stub. It captures one CompressedImage
  when a real source is present; otherwise `view` is empty. Candidates are empty
  until Phase 3 detector integration.
- **Stop**: idempotent. `SOFT_STOP` cancels Nav2 and relies on velocity smoother
  input timeout + watchdog; `HOLD` cancels and latches; `QUICK_STOP_REQUEST`
  triggers hardware quick-stop independently from epoch. Repeating the same
  request ID is a no-op with cached terminal result.

## Mission Epoch + UUID Idempotency

`SeekObject` is the epoch authority. A new instruction performs ABORT-AND-RESET:
cancel all active skill and Nav2 goals, increment `mission_epoch`, clear the
committed subgoal, frontier commit state, and dedup tables, then re-enter SEARCH.
The old SeekObject handle finalizes as `PREEMPTED`.

Every skill goal carries the current epoch and a fresh UUID `step_id`. Servers
gate by epoch and reject zombie goals. Result handlers also filter by dispatch
epoch. Each server keeps `{request_id: handle/result}` deduplication.

## Frontier Extractor

Correct source: use the **SLAM occupancy grid** (`/map`, `nav_msgs/OccupancyGrid`,
frame `map`, -1 unknown / 0 free / 100 occupied), not rolling local costmap. The
local costmap can lack unknown cells and therefore produce zero frontiers.

Frontier definition: free cell with a 4-connected unknown neighbor. Clusters are
8-connected and filtered by `min_frontier_cells`. Score:

```text
w_size * size - w_dist * distance - w_turn * heading_change
```

Stable IDs are derived from quantized centroids. Top-K frontiers and RViz markers
are published.

Hysteresis:

- Do not switch from the committed frontier until a competitor beats it by
  `switch_margin` (15%) and the current choice has been held for at least
  `min_dwell_s` (4 s).
- Override only when the committed frontier disappears.

## FLAT Detect Path

`/target_pixel` flows only after the target prompt reaches the detector/tracker.
The executive publishes `SeekObject.instruction` to `/target_prompt` through a
small internal bridge.

QoS must be compatible: the `/target_pixel` consumer must not request a deadline
that a BEST_EFFORT publisher does not offer. Use `detection_stream_nodeadline()`
or add a matching offered deadline on the publisher. Keep a unit test around
`is_compatible()` for this pair.

## `map_odom_relay`

`search_coordinator/map_odom_relay.py` subscribes to `/map_odom_correction` with
`correction_lowrate` QoS and broadcasts `map->odom` into `/tf` at about 10 Hz,
within the 0.2 s transform tolerance. It holds the last valid correction, uses
identity before the first correction, and gates by:

- stale sequence
- stale stamp (`max_correction_age_s`)
- covariance/fitness
- jump (`max_jump_m`, `max_jump_rad`), accepted only when `relocalized==true`

RTAB-Map `publish_tf_map` must be parameterized and set to `false` when relay is
running to avoid duplicate broadcasters.

## Nav2 Lightening

- `controller_frequency`: 15 -> 10
- `expected_planner_frequency`: 20 -> 1
- remove `smoother_server` and `waypoint_follower` where not needed
- reduce behavior plugins to spin/backup/wait
- keep NavFn (`use_astar:false`) + DWB
- keep local costmap in `odom`
- use `/scan` as obstacle source
- do not hardcode `use_sim_time`

Before removing `map_server`, confirm what feeds `global_costmap` static layer.

## Deletions

Remove:

- `reliable_prompt_sender.py` and its launch/install entry
- old `target_pixel_to_goal` state soup (`goal_locked`, prompt ack, nav_status
  auto-success, etc.) after geometry is moved to `approach_geometry.py`
- reactive `cmd_vel` in tracker nodes
- dead launch references to deleted scripts

## Corrected Task Order

1. **T2.8** Geometry -> `approach_geometry.py` with pure pytest coverage.
2. **T2.6** `map_odom_relay` + RTAB-Map `publish_tf_map` parameter.
3. **T2.7** Nav2 lightening and global costmap source check.
4. **Frontier source spike**: empirically confirm `/map` unknown cells in sim.
5. **Prompt bridge + `/target_pixel` QoS fix**.
6. **T2.4** Skill servers with epoch authority support.
7. **T2.5** Full mission epoch behavior and HeartbeatMonitor epoch filtering.
8. **T2.2** Executive FSM + SeekObject server.
9. **T2.9** Deletions.
10. **T2.10** End-to-end FLAT scenario + Pi-class profiling baseline.

## Key Risks

- Frontier source must be empirically proven before T2.3.
- `/target_pixel` QoS and prompt bridge must work before any APPROACH can finish.
- Loopback action servers need separate callback groups and non-blocking
  callbacks.
- WSL Pi profiling is only a relative gate; remeasure on hardware in Phase 6.3.
