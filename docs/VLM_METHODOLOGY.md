# VLM Methodology

This document briefly describes the physical prototype, system architecture, and
experimental procedure.

## Physical Prototype

Experiments were conducted on a differential-drive mobile robot with an onboard
Raspberry Pi 5. Low-level wheel-base control uses `ros2_control`: the
differential-drive controller receives velocity commands and sends them to Maxon
EPOS4 drives over CAN/CiA-402. Wheel odometry and inertial/gyroscopic data are
fused in an EKF to produce the local robot state estimate.

The main sensor is an Intel RealSense RGB-D camera. RGB images are used for
detection and VLM observations; depth is used to reconstruct metric 3D positions
of detected objects and to build a local laser scan (`depthimage_to_laserscan`)
for navigation and obstacle monitoring. The robot-side navigation loop runs in
ROS 2 Jazzy and includes Nav2, local costmap sources, a velocity-command
watchdog, and emergency stop mechanisms.

Heavy computation is moved to an external GPU edge computer. In the experiments,
the edge computer ran SLAM, open-vocabulary object detection, and the VLM
orchestrator. Communication between Raspberry Pi and the edge computer used ROS 2
`rmw_zenoh` over Wi-Fi. To reduce network load, one synchronized RGB-D stream was
sent across the link, decompressed on edge, and republished locally to SLAM, the
detector, and the VLM planner.

## Distributed System Architecture

The system uses a three-layer architecture split by latency requirements.

The first layer is the robot reactive loop. It runs on Raspberry Pi and includes
EKF, Nav2, motion controllers, local scan processing, and safety mechanisms. It
does not depend on the VLM and is never blocked by network requests.

The second layer is the `search_coordinator` executive, also on Raspberry Pi. It
provides atomic skills: turning/moving to a pose, moving forward, approaching a
detected object, collecting observations, and stopping. All high-level decisions,
including VLM decisions, are ultimately executed through these skills.

The third layer is the edge/VLM layer. The edge computer runs:

- RTAB-Map SLAM, which builds the map and publishes low-rate `map -> odom`
  correction;
- an open-vocabulary detector returning marked object candidates;
- `planner_orchestrator`, which calls an OpenAI-compatible VLM API and converts
  the scene into a sequence of discrete actions.

The VLM does not control motors directly and does not generate navigation
coordinates. It chooses an action from a limited dictionary and, when needed,
selects a visual candidate by `mark_id`. The executive computes the metric 3D
target from pixel, depth image, camera model, and current TF. This separates
semantic reasoning from metric navigation.

## Perception and Semantic Representation

At each planning step, the system builds a compact scene observation. The
detector receives an RGB frame and a textual target query. Strict target search
uses an open-vocabulary target detector; scene exploration additionally extracts
office context objects such as desks, drawer cabinets, cabinets, shelves,
monitors, and chairs. Detections are encoded as Set-of-Mark: each candidate gets
a `mark_id`, label, confidence, image position, and distance from depth when
available.

Candidates are separated into two types. Strict target detections directly match
the target and may be used for approach. Context detections are not navigation
targets; they are semantic cues for choosing exploration direction. For example,
if the target is an office chair, desks and office furniture increase the
relevance of a corridor but do not become goals to approach.

In scenes where the target is not fully visible initially, partial occlusion is
allowed: the object may be partly hidden by furniture, lie at the edge of the
field of view, or be visible only through fragments of characteristic shape. The
open-vocabulary detector can still return a useful candidate from partial visual
evidence. Such a candidate can justify a viewpoint change, short probe motion, or
later `target_nav_lock`, but final success still requires metric localization
from depth and an approach to the confirmed point.

The VLM also receives a compact map representation: free, occupied, and unknown
regions. The map is not decorative; it provides information about corridors and
frontier directions.

## VLM Planning Loop

A mission starts from a text request such as `office chair`, `drawer cabinet`, or
a more indirect semantic description. Before the first detector call, semantic
target resolution determines whether the request is already a concrete object
name, an object name with visual attributes, or a riddle/metaphor. Direct
requests are preserved, attributes may remain in the detector string, and
indirect descriptions are normalized into detector-friendly open-vocabulary
phrases. Logs preserve the raw query, canonical target, and detector query.

The orchestrator then runs an event loop:

```text
observe -> detect target/context -> build VLM prompt -> choose atomic action
-> execute skill on Pi -> log result -> update memory -> next observation
```

VLM inputs:

- current RGB frame with Set-of-Mark overlays;
- strict target candidates and context marks;
- compact map/free-space description;
- action history and mission notes.

VLM outputs one or more atomic actions from a limited dictionary:

```text
TURN +/- angle
DRIVE_FORWARD distance
DRIVE_TO_VISIBLE mark_id
DRIVE_TO_LOCKED_TARGET
DETECT_ALL / refresh context
DONE
```

Before execution, actions are consistency-checked. For example, `DONE` is not
accepted unless the goal has really been reached or a confirmed close-range
target lock has completed. If the target is at the image edge or lacks reliable
depth, the system may first center the view or move forward cautiously to obtain
a more stable observation.

## Behavior Modes

If the target is visible and has depth, the system selects `DRIVE_TO_VISIBLE`.
The executive builds a goal point with safe standoff and sends it to Nav2. If the
final point is outside known free map, the executive may use bounded approach:
move a limited step toward the target, reveal map, then retry.

If the target disappears after a confident detection, the system uses
`target_nav_lock`: the last confirmed metric point is stored and can be used to
continue approach. This handles cases where the object leaves the camera frame or
is occluded while the robot gets closer.

If the target is not visible in the starting view, the robot first performs a
structured overview: initial view, right turn of about 90 degrees, then a turn to
inspect the opposite side. If the target is found in any view, the system enters
target approach. If not, the VLM chooses an active exploration direction from the
map. Free corridors and frontiers have priority; context objects only help select
which corridor is most relevant.

If Nav2 or map checks cannot confirm a safe route to the target, the system enters
`target_approach_blocked`: it performs a short motion in a free direction to
reveal more map or change viewpoint, then retries approach to the stored target.

## Experimental Protocol

Experiments were HIL runs on the physical robot. Each scene had a ground-truth
description: whether the object was visible initially, where it was relative to
the robot, whether the target point was reachable in the initial map, and what
behavior counted as expected.

Each scene was repeated five times. Before repeats, the map and orchestrator were
restarted to reproduce initial conditions. The logger recorded mission events to
`jsonl` and aggregated step tables to `csv`: mission start/end, observations,
detections, VLM decisions, actions, skill results, durations, target-lock status,
and degraded/fallback flags.

Main metrics:

- `success_rate`: fraction of runs where the robot reached the correct target
  object;
- `progress_rate`: degree of progress through the semantic task decomposition;
- `mission_time_s`: time from `mission_start` to `mission_end`;
- number and types of actions;
- frequency of `target_nav_lock` and recovery behavior.

For simple scenes, `progress_rate` reaches 1.0 only when the robot finishes at
the correct object. For scenes where the target is initially hidden, the scale
includes intermediate stages: structured scan, correct corridor choice, progress
through the corridor, target detection, 3D localization or target lock, and final
approach.

## Scope

The methodology evaluates end-to-end system behavior, not the detector or Nav2 in
isolation. It focuses on the ability of the VLM planner to use vision, map, and
context objects to choose actions while staying outside the reactive control
loop. Debug issues, transport failures, costmap parameters, and behavior fixes
are documented separately in the trials/issues log.
