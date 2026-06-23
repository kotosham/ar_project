"""Pure decision logic for the skill servers (ROADMAP 2.4).

ROS-free so the branchy parts (frontier resolution, approach freshness gating,
Nav2 status mapping) are unit-testable without action infrastructure. The servers
in skills.py are thin glue around these.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

# Mirrors action_msgs/msg/GoalStatus.STATUS_SUCCEEDED without importing it here.
GOAL_STATUS_SUCCEEDED = 4


def resolve_frontier(frontiers: list, frontier_id: int,
                     max_travel_m: float) -> Tuple[Optional[object], str]:
    """Pick a frontier for ExploreFrontier.

    `frontiers` is best-first (each has .id, .score, .distance_m). `frontier_id`:
    -1 => best after hysteresis (the published order already encodes it); >=0 =>
    that stable id. `max_travel_m` > 0 filters out frontiers farther than the cap.

    Returns (frontier_or_None, reason) with reason in
    {'OK','EMPTY','NO_MATCH','TOO_FAR'}.
    """
    if not frontiers:
        return None, 'EMPTY'
    candidates = frontiers
    if max_travel_m and max_travel_m > 0.0:
        candidates = [f for f in frontiers if f.distance_m <= max_travel_m]
        if not candidates:
            return None, 'TOO_FAR'
    if frontier_id is not None and frontier_id >= 0:
        for f in candidates:
            if f.id == frontier_id:
                return f, 'OK'
        return None, 'NO_MATCH'
    return candidates[0], 'OK'


def approach_not_reached_outcome(have_pixel: bool) -> str:
    """Terminal classification for ApproachDetection when the pose was NOT reached
    fresh: no pixel ever => LOST_TARGET; pixels seen but stale => STALE_DETECTION.
    (FMEA: never declare SUCCEEDED while detection is not fresh.)"""
    return 'STALE_DETECTION' if have_pixel else 'LOST_TARGET'


def is_fresh(age_s: Optional[float], max_age_s: float) -> bool:
    """True if a detection of the given age is fresh enough to act on."""
    return age_s is not None and age_s <= max_age_s


def nav_succeeded(status: int) -> bool:
    """Map a Nav2 action GoalStatus to a reached/ok boolean."""
    return status == GOAL_STATUS_SUCCEEDED
