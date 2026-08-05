# Robust Architecture Overview (`robust` branches)

This directory is the single source of truth for the architecture connecting the
**robot (Raspberry Pi 5 / 4 GB)** and the **PC/edge host (GPU)**. The rewrite is
focused on robustness, near-real-time operation, graceful degradation during
network/node failures, and two operating modes: `flat` and `vlm`. The previous
implementation's models and logic are reused where useful (segmentation,
`target_pixel_to_goal`, Nav2, EKF, EPOS4/CAN, RealSense), but integrated into a
new architecture.

> The architecture was accepted after two design passes and FMEA review. These
> documents implement that decision rather than reopening it. Documentation
> comes first, then implementation according to [ROADMAP](../ROADMAP.md). Testing
> starts in Gazebo on WSL2 Ubuntu and then moves to hardware.

## Documents

| File | Topic |
|---|---|
| [DATA_CONTRACTS.md](DATA_CONTRACTS.md) | Contract for every Pi-PC channel: payload, format, QoS, size, bandwidth, latency, complexity, and what stays local. |
| [MODES.md](MODES.md) | `flat` and `vlm` modes: FSM, skill dictionary, replan timing, lead-time/commit-point, notes buffer, and degradation. |
| [REPOS_INTERFACES.md](REPOS_INTERFACES.md) | Package layout on `robust`, `.action`/`.msg`/`.srv` inventory, REUSED/NEW/DELETED status, complexity and time estimates. |
| [GAZEBO_WSL_TESTING.md](GAZEBO_WSL_TESTING.md) | Gazebo-on-WSL2 test plan: simulated sensors, mock EPOS4/CAN, Pi-PC split emulation, CUDA on WSL2, and FMEA-to-simulation matrix. |
| [../ROADMAP.md](../ROADMAP.md) | Step-by-step implementation checklist with `- [ ]` items. |

`DATA_CONTRACTS.md` and `GAZEBO_WSL_TESTING.md` both start with a verification
corrections block. Those corrections have priority over the body of the
document.

## Core Idea: 3-Layer Hierarchy, Executive on Pi

The split is based on **latency**, not on machine boundaries. The slow VLM is
never on the reactive path; the robot remains autonomous for navigation without
network access.

```text
  EXTERNAL VLM API    +-----------------------------------------------+
  OpenAI-compatible   | Qwen3-VL-30B-A3B as a separate service/cloud  |  ~0.05-0.3 Hz
  cloud/server        | vLLM/SGLang/provider, not hosted on edge/Pi   |  3-20 s
                      +-----------------------^-----------------------+
                                              | HTTP, async, non-reactive
                    +-------------------------+-----------------------+
  EDGE / PC (GPU)   | Planner Orchestrator: HTTP client, no GPU       | event-driven (vlm)
                    | single-in-flight, p99 timeout, breaker,         |
                    | notes/summary buffer (vlm only)                 |
                    | Perception: open-vocab detector (YOLOE/DINO+SAM)| on demand (GPU)
                    | SLAM (RTAB-Map) -> low-rate map->odom correction| 1-2 Hz
                    +---------------+---------------------------------+
                                    | Wi-Fi (rmw_zenoh, chrony, QoS)
                                    | small/event messages only; idle ~= 0
  ------------------+---------------+-------------------------------------------
                    v
  ROBOT (Pi 5)      +-----------------------------------------------+
                    | Executive: Search Coordinator (FSM/BT)        | 1-10 Hz
                    | owns mission, local frontiers, skill actions  | holds committed
                    | Explore/GoTo/Approach/GetObs/Stop             | subgoal/default
                    +-----------------------------------------------+
                    | Reactive: light Nav2, map_odom_relay, EKF,    | 10-50 Hz,
                    | target_pixel_to_goal, local /scan             | local
                    +-----------------------------------------------+
                    | Safety: cmd_vel watchdog, Collision Monitor,  | hard RT,
                    | real CiA-402 quick-stop -> EPOS4/CAN          | network-independent
                    +-----------------------------------------------+
```

## Invariants

1. **The VLM is outside the reactive loop.** EKF, Nav2, control, safety, and FLAT
   run locally and do not wait for Wi-Fi or the VLM. A new plan is adopted only
   at a safe **commit point**.
2. **TF is not streamed over Wi-Fi.** The full `map->odom->base_link` chain is
   assembled locally on the Pi. EKF provides `odom->base_link`; `map_odom_relay`
   applies a low-rate edge-SLAM correction using last-good state and
   jump/covariance gates, then rebroadcasts `map->odom` locally.
3. **No heavy streams over Wi-Fi.** PointCloud2 and raw depth are not sent.
   `/scan` is generated locally. The edge receives one compressed event
   keyframe.
4. **Perception is an event service.** Segmentation is one detection request
   with UUID idempotency and single-in-flight semantics, not a stream. The target
   coordinate is always computed by `target_pixel_to_goal`; neither the VLM nor
   the detector creates navigation coordinates.
5. **`flat` is both the baseline and permanent `vlm` fallback.** `vlm` is an
   extension layer. If VLM/edge/Wi-Fi is lost, the system degrades seamlessly to
   `flat`.
6. **Safety is a separate layer with two independent stop mechanisms**, invariant
   across all degradation modes.

## Two Modes

- **`flat`**: target description -> SEARCH with local frontiers and hysteresis ->
  DETECT through detector/pixel/3D goal -> DRIVE through Nav2. It has no VLM
  dependency.
- **`vlm`**: Planner Orchestrator decomposes the instruction into a tree or
  sequence of **FLAT-solvable subtasks**, periodically replanning from history.
  Replanning is slow, so lead-time is reserved: while FLAT executes the current
  subtask, the next plan is computed asynchronously and adopted at a commit
  point. The model keeps compact self-notes rather than storing frames, which
  keeps the token budget and latency bounded.

## FMEA Must-Fix Items

- Real CiA-402 quick-stop on the RT `write()` path. The previous code only
  logged faults; quick-stop was missing. It must avoid blocking 50 ms SDO calls,
  poll faults per cycle, and handle CAN bus-off.
- Approach must not declare "reached" from a stale pixel. Freshness is checked
  at the success latch (`nav_status`), not only at skill entry.
- Instruction changes require ABORT-and-reset plus mission epoch invalidation for
  all in-flight UUIDs.
- Establish a zero-VLM FLAT baseline on the Pi first; use it as a gate. Include
  frontier hysteresis, chrony offset below 0.1 s, and a lighter Nav2 profile.

## Target Stack

ROS 2 Jazzy, Gazebo Harmonic (gz-sim8), ros_gz, gz_ros2_control, rmw_zenoh,
chrony, Nav2 (NavFn + DWB), robot_localization (EKF), RTAB-Map, open-vocabulary
detector (YOLOE/DINO+SAM) on edge GPU, external OpenAI-compatible VLM API
(Qwen3-VL-30B-A3B; endpoint hosting through vLLM/SGLang/cloud is outside this
system), RealSense D435i, and Maxon EPOS4 (CiA-402/SocketCAN).
