"""Pure decision logic for the skill servers (ROADMAP 2.4).

ROS-free so the branchy parts (frontier resolution, approach freshness gating,
Nav2 status mapping) are unit-testable without action infrastructure. The servers
in skills.py are thin glue around these.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Mirrors action_msgs/msg/GoalStatus.STATUS_SUCCEEDED without importing it here.
GOAL_STATUS_SUCCEEDED = 4


def resolve_frontier(frontiers: list, frontier_id: int,
                     max_travel_m: float,
                     exclude_ids: Optional[set] = None,
                     min_travel_m: float = 0.0) -> Tuple[Optional[object], str]:
    """Pick a frontier for ExploreFrontier.

    `frontiers` is best-first (each has .id, .score, .distance_m). `frontier_id`:
    -1 => best after hysteresis (the published order already encodes it); >=0 =>
    that stable id. `max_travel_m` > 0 filters out frontiers farther than the cap.
    `min_travel_m` > 0 filters out frontiers CLOSER than the floor: a frontier at
    the robot's own position (distance_m≈0) gets the smallest distance penalty so
    it scores highest, but driving to it is a no-op (Nav2 reports 'reached'
    instantly and the robot never moves). Dropping near-zero frontiers makes the
    explorer pick one far enough to actually travel and reveal new space.
    `exclude_ids` drops frontiers whose stable id is in the set — the explorer
    blacklists a frontier after Nav2 fails to reach it, so it tries the others
    and terminates cleanly once reachable space is exhausted instead of looping
    forever on an unreachable (e.g. behind-a-wall) frontier.

    Returns (frontier_or_None, reason) with reason in
    {'OK','EMPTY','EXCLUDED','NO_MATCH','TOO_FAR','TOO_NEAR'}.
    """
    if not frontiers:
        return None, 'EMPTY'
    candidates = frontiers
    if exclude_ids:
        candidates = [f for f in candidates if f.id not in exclude_ids]
        if not candidates:
            return None, 'EXCLUDED'
    if min_travel_m and min_travel_m > 0.0:
        candidates = [f for f in candidates if f.distance_m >= min_travel_m]
        if not candidates:
            return None, 'TOO_NEAR'
    if max_travel_m and max_travel_m > 0.0:
        candidates = [f for f in candidates if f.distance_m <= max_travel_m]
        if not candidates:
            return None, 'TOO_FAR'
    if frontier_id is not None and frontier_id >= 0:
        for f in candidates:
            if f.id == frontier_id:
                return f, 'OK'
        return None, 'NO_MATCH'
    return candidates[0], 'OK'


def explore_goal_xy(robot_xy: Tuple[float, float],
                    centroid_xy: Tuple[float, float],
                    min_drive_m: float,
                    standoff_m: float) -> Tuple[float, float, float]:
    """Project a NavigateToPose goal for frontier exploration.

    Aim from the robot toward the frontier centroid (a FREE cell on the
    free/unknown boundary), clamped so the goal is at least `min_drive_m` from
    the robot. The clamp avoids the degenerate cold-start where the centroid
    sits inside Nav2's xy_goal_tolerance: a goal AT the centroid is reported
    'reached' instantly, the robot never moves, the SLAM grid never grows and no
    new frontiers appear.

    `standoff_m` optionally pushes the goal that far PAST the centroid into the
    unknown side. Keep it small (0.0 by default): a goal placed deep in unknown
    space is uncosted, so the global planner aborts and Nav2 wastes seconds on
    recovery behaviours. With standoff 0 the goal stays on the free centroid and
    the robot still grows the map by sensing past the boundary once it arrives.

    Returns (goal_x, goal_y, yaw). Falls back to the centroid (yaw 0.0) when the
    robot is already on top of it (direction undefined).
    """
    rx, ry = robot_xy
    cx, cy = centroid_xy
    dx, dy = cx - rx, cy - ry
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return cx, cy, 0.0
    ux, uy = dx / dist, dy / dist
    goal_d = max(dist + standoff_m, min_drive_m)
    return rx + ux * goal_d, ry + uy * goal_d, math.atan2(uy, ux)


def approach_not_reached_outcome(have_pixel: bool) -> str:
    """Terminal classification for ApproachDetection when the pose was NOT reached
    fresh: no pixel ever => LOST_TARGET; pixels seen but stale => STALE_DETECTION.
    (FMEA: never declare SUCCEEDED while detection is not fresh.)"""
    return 'STALE_DETECTION' if have_pixel else 'LOST_TARGET'


def is_fresh(age_s: Optional[float], max_age_s: float) -> bool:
    """True if a detection of the given age is fresh enough to act on."""
    return age_s is not None and age_s <= max_age_s


def _distance_is_known(distance_m) -> bool:
    try:
        d = float(distance_m)
    except (TypeError, ValueError):
        return False
    return math.isfinite(d) and d > 0.0


def flat_approach_event(succeeded: bool,
                        bounded_step: bool,
                        final_distance_m,
                        final_threshold_m: float) -> str:
    """Map an ApproachDetection result to the FLAT FSM event.

    A bounded visual approach is only a short map-growing probe toward the target.
    Reaching that intermediate Nav2 pose is progress, not mission completion,
    unless it already leaves the robot within the final standoff threshold.
    """
    if not succeeded:
        return 'LOST'
    if not bounded_step:
        return 'REACHED'
    if not _distance_is_known(final_distance_m):
        return 'LOST'
    return 'REACHED' if float(final_distance_m) <= float(final_threshold_m) else 'LOST'


def nav_succeeded(status: int) -> bool:
    """Map a Nav2 action GoalStatus to a reached/ok boolean."""
    return status == GOAL_STATUS_SUCCEEDED


def should_blacklist_frontier(drive_terminal: str) -> bool:
    """Should a failed ExploreFrontier drive blacklist the frontier as unreachable?

    Only a GENUINE nav failure while Nav2 was actually driving ('rejected'/'failed')
    means the frontier is unreachable. 'no_server' (Nav2 not ready / dropped) is
    TRANSIENT -- blacklisting then would permanently exclude valid frontiers just
    because Nav2 was still activating. reached/canceled/zombie are handled before
    this and never blacklist."""
    return drive_terminal in ('rejected', 'failed')
