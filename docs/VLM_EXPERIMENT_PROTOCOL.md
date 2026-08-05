# Wheeled-Robot Experiment Protocol

This is a concise HIL experiment protocol comparing two object-search modes on a
wheeled robot platform: `FLAT` and `VLM`.

Raw logs are attached separately:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_scene_*.csv
~/ros2_ws/experiment_logs/vlm_missions/vlm_scene_*.jsonl
~/ros2_ws/experiment_logs/flat_missions/flat_scene_*.csv
~/ros2_ws/experiment_logs/flat_missions/flat_scene_*.jsonl
```

Debug issues and fixes are tracked in:

```text
docs/VLM_HIL_TRIALS_ISSUES_LOG.md
```

## System Summary

The prototype is a differential-drive wheeled robot with an onboard Raspberry Pi
5 and an edge laptop. The Raspberry Pi runs motors/CAN, RealSense, `/scan`, EKF,
Nav2, `map_odom_relay`, and `search_coordinator`. The edge laptop receives
`/camera_edge/*`, runs RTAB-Map SLAM, the detector, dashboard/logger, and the VLM
orchestrator.

`FLAT` does not use Qwen. The Pi executive receives a target text, runs the
continuous detector `/target_pixel`, performs a fixed overview
`forward -> right -> left` when needed, then uses `ExploreFrontier` and Nav2. It
has no semantic corridor selection.

`VLM` uses Qwen as a reasoning module. The model receives the target text,
Set-of-Mark frame, SLAM map view, target/context candidates, notes/memory, and
previous action results. Qwen selects actions: approach a visible target, turn,
short forward motion, semantic corridor exploration, detection refresh, or
finish.

In scenes where the target is not initially visible, partial occlusion is allowed:
the target may be found after a viewpoint change or while moving through a
corridor. This matters for interpretation: the detector may see part of a chair,
but without a valid depth point the robot cannot navigate to it.

## Metrics

`Success` is binary per run. It is `1` only if the robot reaches the correct
ground-truth target object. For scenes with a specific intended object, reaching
any object of the same class is not enough.

`Progress` is a manual estimate of how far the run advanced through the task. It
captures not only the final FSM state, but also strategy quality: target
detection, motion toward the correct area, target lock, correct corridor choice,
and final approach to the intended object.

`D/P time (s)` is the characteristic detector/planning processing time:

```text
FLAT = mean detector update time from DINO+MobileSAM continuous tracker
VLM  = mean positive VLM planning latency from orchestrator step_result.latency_ms
```

`D/P time` is not full mission duration. Full `mission_time_s` is stored in raw
CSV/JSONL and includes turns, waits, Nav2, SLAM/map updates, and physical motion.

For FLAT, detector runtime does not necessarily increase with semantic scene
complexity. Empty frames and small false segments can be faster than stable masks
of large objects. Therefore D/P time reflects detector update cost, not search
quality.

## Scenes

| Scene | Setup | Target | Ground Truth |
| ---: | --- | --- | --- |
| 1 | Visible / reachable | drawer cabinet | Cabinet is directly in front of the robot and reachable on the map. |
| 2 | Visible / unmapped | office chair | Chair is visible in the starting frame, but the target is outside/on the edge of the current map. |
| 3 | Scan reveal | office chair | Chair is not visible forward, but appears after a simple turn. |
| 4 | Corridor search | office chair | Chair is not found by initial scan; the robot must explore the left corridor. |
| 5 | Semantic prompt | office chair / drawer cabinet | Target is given as a riddle, without the direct object name. |

## Experiment Notes

Scene 1 checks the basic visible approach. Both modes reliably reach the drawer
cabinet. This is a sanity check for detector, depth, and Nav2.

Scene 2 checks the case where the target is visible but the initial map does not
allow direct approach. VLM preserves the semantic target and moves toward the
initially visible chair. FLAT often loses it because of invalid depth and, after
overview, approaches a random other chair. Therefore FLAT `success=1` is assigned
only to the run that actually reached the originally visible chair; the others
receive partial progress for finding an object of class `office chair`, but
`success=0`.

Scene 3 checks simple scan-based search. VLM deliberately uses overview and
approaches the chair it finds. FLAT was also given fixed scan for fair comparison;
one run under-rotated, entered frontier exploration, and found another chair, so
progress is reduced while success remains positive for this scene.

Scene 4 checks semantic exploration. VLM chooses the corridor using map and
context objects, then searches for the chair in the chosen area. One VLM run
failed because a 2D detection lacked valid depth: the target was seen but did not
produce reliable `target_nav_lock` and was later lost. FLAT success happened only
through random frontier wandering rather than meaningful left-corridor choice.
FLAT runs 2, 3, and 4 are marked as negative success because the robot stopped at
a wall/furniture after false or single suspicious target detections.

Scene 5 checks semantic prompts. The target is described as a riddle instead of a
direct class name. VLM uses a resolver to convert the description into several
zero-shot detector phrases and then guides the robot to the found goal. All five
VLM runs are labeled `success=1`, `progress=1`. FLAT is weak here because it has
no resolver and sends the raw riddle text to the detector. In the first three
FLAT runs, FSM reached `DONE`, but this is not counted as success: `DONE` came
from detector noise/false target detections. In runs 4-5 the robot entered random
frontier wandering without finding the target. All five FLAT scene 5 runs are
manually labeled `success=0`, `progress=0.25`.

## Manual Progress Rules Used

Scene 2 FLAT:

```text
success  = [0, 0, 0, 1, 0]
progress = [0.50, 0.50, 0.50, 1.00, 0.50]
```

Scene 3 FLAT:

```text
success  = [1, 1, 1, 1, 1]
progress = [1.00, 1.00, 1.00, 0.50, 1.00]
```

Scene 4 FLAT:

```text
success  = [1, 0, 0, 0, 1]
progress = [0.75, 0.25, 0.25, 0.25, 0.75]
```

Scene 4 VLM:

```text
success  = [1, 0, 1, 1, 1]
progress = [1.00, 0.75, 1.00, 1.00, 1.00]
```

Scene 5 FLAT:

```text
success  = [0, 0, 0, 0, 0]
progress = [0.25, 0.25, 0.25, 0.25, 0.25]
```

Scene 5 VLM:

```text
success  = [1, 1, 1, 1, 1]
progress = [1.00, 1.00, 1.00, 1.00, 1.00]
```

## Final Metrics Table

| Scene | Setup | FLAT Success | FLAT Progress | FLAT D/P time (s) | VLM Success | VLM Progress | VLM D/P time (s) |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | Visible / reachable | 1.00 | 1.00 | 0.48 | 1.00 | 1.00 | 1.18 |
| 2 | Visible / unmapped | 0.20 | 0.60 | 0.38 | 1.00 | 1.00 | 1.14 |
| 3 | Scan reveal | 1.00 | 0.90 | 0.22 | 1.00 | 1.00 | 1.16 |
| 4 | Corridor search | 0.40 | 0.45 | 0.085 | 0.80 | 0.95 | 2.03 |
| 5 | Semantic prompt | 0.00 | 0.25 | 0.07 | 1.00 | 1.00 | 1.35 |

## Interpretation

FLAT is faster at the detector-update level, but in complex scenes its behavior
depends on geometry, random frontier exploration, and valid depth. If the target
is visible as a 2D candidate but depth is invalid, FLAT cannot reason about where
to move to reveal the map relative to that target.

VLM is slower per decision/planning step, but better preserves the semantic goal,
uses map and context objects to select a corridor, and avoids treating context
objects as approach targets. In scenes 2 and 4, higher D/P time is offset by
higher success/progress.
