"""Unit tests for pure skill decision logic (ROADMAP 2.4)."""
from dataclasses import dataclass

from search_coordinator.skill_logic import (
    approach_not_reached_outcome,
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
