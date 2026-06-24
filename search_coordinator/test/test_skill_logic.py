"""Unit tests for pure skill decision logic (ROADMAP 2.4)."""
import math
from dataclasses import dataclass

from search_coordinator.skill_logic import (
    approach_not_reached_outcome,
    explore_goal_xy,
    is_fresh,
    nav_succeeded,
    resolve_frontier,
)


@dataclass
class F:
    id: int
    score: float
    distance_m: float


def test_resolve_empty():
    sel, reason = resolve_frontier([], -1, 0.0)
    assert sel is None and reason == 'EMPTY'


def test_resolve_best_when_id_negative():
    fs = [F(7, 30.0, 1.0), F(3, 20.0, 2.0)]   # best-first
    sel, reason = resolve_frontier(fs, -1, 0.0)
    assert reason == 'OK' and sel.id == 7


def test_resolve_specific_id():
    fs = [F(7, 30.0, 1.0), F(3, 20.0, 2.0)]
    sel, reason = resolve_frontier(fs, 3, 0.0)
    assert reason == 'OK' and sel.id == 3


def test_resolve_id_not_found():
    fs = [F(7, 30.0, 1.0)]
    sel, reason = resolve_frontier(fs, 99, 0.0)
    assert sel is None and reason == 'NO_MATCH'


def test_resolve_max_travel_filters_all():
    fs = [F(7, 30.0, 5.0), F(3, 20.0, 8.0)]
    sel, reason = resolve_frontier(fs, -1, max_travel_m=2.0)
    assert sel is None and reason == 'TOO_FAR'


def test_resolve_max_travel_keeps_near():
    fs = [F(7, 30.0, 5.0), F(3, 20.0, 1.5)]   # best is far, near one survives cap
    sel, reason = resolve_frontier(fs, -1, max_travel_m=2.0)
    assert reason == 'OK' and sel.id == 3


def test_approach_outcome():
    assert approach_not_reached_outcome(have_pixel=False) == 'LOST_TARGET'
    assert approach_not_reached_outcome(have_pixel=True) == 'STALE_DETECTION'


def test_is_fresh():
    assert is_fresh(1.0, 1.5) is True
    assert is_fresh(2.0, 1.5) is False
    assert is_fresh(None, 1.5) is False


def test_nav_succeeded():
    assert nav_succeeded(4) is True    # STATUS_SUCCEEDED
    assert nav_succeeded(6) is False   # STATUS_ABORTED


def test_explore_goal_coldstart_clamps_to_min_drive():
    # Frontier centroid 0.10 m away (inside Nav2 xy_goal_tolerance): a goal AT it
    # is reported 'reached' instantly. The projected goal must be >= min_drive_m.
    gx, gy, yaw = explore_goal_xy((0.0, 0.0), (0.10, 0.0),
                                  min_drive_m=0.5, standoff_m=0.4)
    d = math.hypot(gx, gy)
    assert d >= 0.5 - 1e-9
    # dist(0.1)+standoff(0.4)=0.5 == min_drive(0.5) -> 0.5 m straight ahead (+x).
    assert math.isclose(gx, 0.5, abs_tol=1e-6)
    assert math.isclose(gy, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)


def test_explore_goal_pushes_past_centroid():
    # Far frontier: goal sits standoff_m beyond the centroid along robot->centroid.
    gx, gy, yaw = explore_goal_xy((0.0, 0.0), (2.0, 0.0),
                                  min_drive_m=0.5, standoff_m=0.4)
    assert math.isclose(gx, 2.4, abs_tol=1e-6)   # 2.0 + 0.4 standoff
    assert math.isclose(gy, 0.0, abs_tol=1e-6)


def test_explore_goal_yaw_points_at_frontier():
    # Centroid up-and-right: yaw faces it, goal stays on that ray.
    gx, gy, yaw = explore_goal_xy((1.0, 1.0), (2.0, 2.0),
                                  min_drive_m=0.5, standoff_m=0.4)
    assert math.isclose(yaw, math.pi / 4, abs_tol=1e-6)
    assert math.isclose(gy - 1.0, gx - 1.0, abs_tol=1e-6)   # 45deg ray


def test_explore_goal_degenerate_robot_on_centroid():
    gx, gy, yaw = explore_goal_xy((1.5, -0.5), (1.5, -0.5),
                                  min_drive_m=0.5, standoff_m=0.4)
    assert (gx, gy, yaw) == (1.5, -0.5, 0.0)
