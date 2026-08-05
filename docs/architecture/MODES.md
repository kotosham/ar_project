# Operating Modes: `flat` and `vlm`

## 1. One Execution Substrate, Two High-Level Controllers

The system has exactly one execution substrate: the reactive loop on Raspberry
Pi 5 (EKF -> light Nav2 with NavFn+DWB -> `ros2_control` `diff_cont` ->
`EmbodiedRobotSystem` for EPOS4/CiA-402 over SocketCAN) plus the safety layer and
idempotent skill servers:

- `ExploreFrontier`
- `GoToPose`
- `ApproachDetection`
- `GetObservation`
- `Stop`

This substrate is always owned by **Search Coordinator**, the executive FSM/BT
running on the Pi.

The modes differ only by the **source of subtasks**:

- **`flat`**: the executive is the only decision maker. It receives one object
  description and runs `search -> detect -> goal -> drive` with zero dependency
  on the VLM.
- **`vlm`**: Planner Orchestrator runs above the executive on the edge/PC as a
  light async HTTP client to an **external OpenAI-compatible VLM API**
  (Qwen3-VL-30B-A3B or similar). It does not drive the robot directly and does
  not output navigation coordinates. It decomposes the high-level instruction
  into FLAT-solvable subtasks and periodically replans from mission history.

Invariant: **the VLM is never on the reactive path**. FLAT and safety run in real
time independently from the planner. A new VLM plan is accepted only at a safe
**commit point**. Therefore `vlm` is an extension over `flat`, and `flat` is the
permanent degradation fallback for `vlm`.

```text
EDGE/PC
  Planner Orchestrator (VLM, async, seconds)
  notes/summary buffer, semantic memory
      |
      | subtask = FLAT skill, accepted only at commit point
      v
ROBOT / Pi
  Search Coordinator (executive FSM/BT)
  ExploreFrontier, GoToPose, ApproachDetection, GetObservation, Stop
  target_pixel_to_goal, Nav2, EKF, SAFETY
```

## 2. `flat` Mode

### 2.1 Purpose and Behavior

Input: one textual target description. The goal is to find and approach the
object described by the prompt.

1. **SEARCH**: local frontier exploration. Search Coordinator extracts frontiers
   locally from the costmap, selects one with hysteresis, and drives there using
   `ExploreFrontier`.
2. **DETECT**: the edge open-vocabulary detector compares the prompt with the
   current frame and publishes the target pixel (`/target_pixel`) and optional
   mask/candidates.
3. **pixel -> 3D goal**: `target_pixel_to_goal` computes the metric target from
   pixel + local aligned depth. The planner/detector never creates navigation
   coordinates.
4. **DRIVE**: Nav2 drives to the goal. The obstacle source is local `/scan` from
   `depthimage_to_laserscan`; raw depth and PointCloud2 never cross Wi-Fi.
5. **ARRIVE / LOST**: success when the approach goal is reached; if detection is
   lost/stale, return to SEARCH.

### 2.2 FSM

```text
IDLE
  prompt set
  v
SEARCH --frontiers exhausted/timeout--> FAILED
  detection valid
  v
DETECT/CONFIRM --pixel stale/lost--> SEARCH
  stable 3D goal
  v
APPROACH --Nav2 succeeded + detection fresh--> ARRIVED
APPROACH --Nav2 aborted or stream stale--> SEARCH

From any state: Stop / preempt / fault -> SAFE_STOP -> IDLE
```

FMEA-critical transition: during `APPROACH`, the executive must check detector
freshness at the success latch. It is forbidden to report `reached` from a stale
pixel. Close-range target loss after a confirmed final approach pose is handled
by the target lock/last-confirmed-goal logic rather than by stale-pixel success.

### 2.3 Frontier Hysteresis

To avoid oscillation between similar frontiers:

- **score margin**: switch to a new frontier only when
  `score(new) > score(current) + margin`.
- **min dwell**: after committing to a frontier, keep it for at least `min_dwell`
  unless it becomes unreachable or exhausted.

This is local Pi logic and applies in both modes.

### 2.4 Common Skill Dictionary

| Skill | Purpose |
|---|---|
| `ExploreFrontier` | Explore the selected local frontier. |
| `GoToPose` | Drive to a pose through Nav2. |
| `ApproachDetection` | Approach a detected target using pixel -> 3D geometry and freshness checks. |
| `GetObservation` | Capture an observation for VLM/detector at a safe point. |
| `Stop` | Safe stop / preempt current subtask. |

All skill servers are preemptable, feedback-carrying, and UUID-idempotent. Search
Coordinator is the only consumer of decisions and always holds a committed
subgoal plus a default productive action, so lack of new commands never causes
idle behavior.

## 3. `vlm` Mode

### 3.1 Orchestrator Loop

Planner Orchestrator implements an anytime/async loop:

1. **Decompose**: high-level instruction -> sequence/tree of subtasks where each
   subtask is one FLAT skill.
2. **Dispatch**: current subtask is sent to the FLAT executive as a normal
   idempotent skill goal. The robot executes reactively without waiting for VLM.
3. **Replan**: periodically and asynchronously compute a next plan from mission
   history and notes. The plan is adopted only at a commit point.

The VLM output is structured as enum/tool-call style decisions. It selects only
from real available options, such as `frontier_id`, `mark_id`, or an allowed
atomic action. It does not invent coordinates.

### 3.2 Notes / Summary Buffer

Frames are expensive to store and resend. Instead, the VLM keeps compact
self-notes, which act as mission memory. A typical notes object contains:

```json
{
  "mission_epoch": 7,
  "instruction": "operator instruction",
  "observations": ["desk visible in corridor, target not found"],
  "visited_rooms": ["hall", "kitchen"],
  "ruled_out": ["target not found in kitchen"],
  "candidate_locations": [{"place": "left corridor", "prior": 0.6, "reason": "office context"}],
  "open_subtasks": ["explore left corridor", "find target"],
  "last_result": {"subtask": "explore", "status": "done"}
}
```

The buffer is append-only after each subtask and observation. When it grows too
large, older observations are summarized into `ruled_out`, `visited_rooms`, and
merged candidate locations. The target budget is roughly 1.5-2k tokens for notes.

### 3.3 Replanning Timing

Replanning takes seconds, so it must overlap with current FLAT execution:

- `replan_interval`: nominal replanning interval.
- `T_lead`: start replanning before the expected end of the current subtask,
  with `T_lead` at least as large as measured VLM p99 latency.
- **single-in-flight**: only one VLM request may be active.
- **commit point**: a completed plan is held as pending and adopted only when the
  executive reaches a safe boundary.

If the next plan is not ready by the commit point, the executive continues the
default productive FLAT action. There is no reactive stall.

### 3.4 Mode Switching and Degradation

- **`vlm -> flat`**: on VLM timeout, edge loss, Wi-Fi loss, or circuit-breaker
  open, the orchestrator stops being a subtask source. The executive continues
  the current committed FLAT skill and then runs normal FLAT behavior.
- **`flat -> vlm`**: when VLM/edge is available and enabled, the orchestrator can
  take over at the next commit point.
- **Instruction change**: this is ABORT-and-reset, not soft replan. The executive
  increments `mission_epoch`, invalidates all in-flight UUIDs, preempts/stops the
  current subtask, archives or resets old notes, and starts the new instruction.

## 4. Safety and Time Windows

Both modes share the same safety layer: `cmd_vel` watchdog, Nav2 Collision
Monitor, real CiA-402 quick-stop (`0x6040`) on the RT `write()` path, per-cycle
fault polling, and CAN bus-off handling.

Clock synchronization must stay well inside these windows:

- TF `transform_tolerance`: 0.2 s
- depth-match tolerance: 0.35 s
- target pixel age: 1.5 s

VLM latency is not part of these windows because it is handled by lead-time and
commit-point adoption.

Relevant files:

- `ar_project/scripts/target_pixel_to_goal.py`
- `ar_project/src/embodied_robot_system.cpp`
- `object_tracking/object_tracking/yoloe_image_segmentation.py`
- `object_tracking/object_tracking/dino_mobilesam_image_segmentation.py`
- `object_tracking/object_tracking/clip_image_segmentation.py`
