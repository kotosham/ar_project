"""Unit tests for the pure frontier-extraction logic (ROADMAP 2.3).

ROS-free: exercises detection, clustering, stable ids, scoring and the
anti-oscillation hysteresis on synthetic occupancy grids built exactly as a SLAM
node emits them (-1 unknown / 0 free / 100 occupied). The two-near-equal-frontier
case is the oscillation scenario (must-fix #10) proven at the logic level.
"""
import math

from search_coordinator.frontier_lib import (
    GridInfo,
    HysteresisParams,
    ScoreParams,
    cluster_frontiers,
    extract_frontiers,
    find_frontier_cells,
    has_unknown,
    score_frontier,
    should_switch,
    stable_frontier_id,
)


def _grid(rows):
    """rows: list of lists (top row first). Returns (flat_data, GridInfo)."""
    h = len(rows)
    w = len(rows[0])
    data = [v for row in rows for v in row]
    return data, GridInfo(width=w, height=h, resolution=1.0, origin_x=0.0, origin_y=0.0)


# --- has_unknown (fail-loud guard) -----------------------------------------

def test_has_unknown_true_false():
    assert has_unknown([0, 0, -1, 100]) is True
    assert has_unknown([0, 0, 0, 100]) is False
    assert has_unknown([]) is False


# --- frontier cell detection ------------------------------------------------

def test_find_frontier_cells_basic():
    # 3x3:  row0 all unknown, row1 = free,free,unknown, row2 all free
    data, info = _grid([
        [-1, -1, -1],
        [0, 0, -1],
        [0, 0, 0],
    ])
    cells = set(find_frontier_cells(data, info.width, info.height))
    # idx3(0,1): unknown above -> frontier; idx4(1,1): unknown above -> frontier;
    # idx8(2,2): unknown above (idx5) -> frontier; idx6/idx7 have no unknown nbr.
    assert cells == {3, 4, 8}


def test_find_frontier_cells_none_without_unknown():
    data, info = _grid([
        [0, 0, 0],
        [0, 100, 0],
        [0, 0, 0],
    ])
    assert find_frontier_cells(data, info.width, info.height) == []


# --- clustering -------------------------------------------------------------

def test_cluster_min_cells_filter():
    data, info = _grid([
        [-1, -1, -1],
        [0, 0, -1],
        [0, 0, 0],
    ])
    cells = find_frontier_cells(data, info.width, info.height)
    # {3,4,8} are 8-connected (4->8 diagonal) => one cluster of size 3
    assert len(cluster_frontiers(cells, info, min_cells=2)) == 1
    assert cluster_frontiers(cells, info, min_cells=4) == []


def test_cluster_two_separate():
    # two free/unknown boundaries separated by an occupied wall column
    data, info = _grid([
        [-1, 0, 100, 0, -1],
        [-1, 0, 100, 0, -1],
        [-1, 0, 100, 0, -1],
    ])
    cells = find_frontier_cells(data, info.width, info.height)
    clusters = cluster_frontiers(cells, info, min_cells=1)
    assert len(clusters) == 2
    # each free column (x=1 and x=3) borders unknown -> 3 cells each
    assert sorted(c.size for c in clusters) == [3, 3]


# --- stable ids -------------------------------------------------------------

def test_stable_id_deterministic_and_drift_stable():
    a = stable_frontier_id((4.2, 7.9), quant_m=1.0)
    b = stable_frontier_id((4.2, 7.9), quant_m=1.0)
    assert a == b
    # small drift staying inside the same 1.0 m quant cell -> same id
    assert stable_frontier_id((4.6, 7.1), quant_m=1.0) == a
    # crossing into a different quant cell -> different id
    assert stable_frontier_id((5.6, 7.9), quant_m=1.0) != a


def test_stable_id_nonnegative_never_minus_one():
    for x, y in [(-100.0, -100.0), (0.0, 0.0), (250.5, -300.2), (-12.0, 88.0)]:
        fid = stable_frontier_id((x, y), quant_m=0.5)
        assert fid >= 0
        assert fid != -1


# --- scoring ----------------------------------------------------------------

def test_score_prefers_big_and_near():
    p = ScoreParams(size_weight=1.0, distance_weight=2.0)
    near_big = score_frontier(20, 1.0, p)
    far_small = score_frontier(5, 10.0, p)
    assert near_big > far_small


# --- hysteresis (anti-oscillation) -----------------------------------------

def test_should_switch_nothing_committed():
    p = HysteresisParams()
    assert should_switch(False, 0.0, best_id=7, best_score=10.0,
                         committed_id=-1, dwell_s=0.0, params=p) is True


def test_should_switch_committed_vanished():
    p = HysteresisParams(min_dwell_s=3.0)
    assert should_switch(False, 0.0, best_id=7, best_score=10.0,
                         committed_id=3, dwell_s=100.0, params=p) is True


def test_should_not_switch_best_is_committed():
    p = HysteresisParams()
    assert should_switch(True, 9.0, best_id=3, best_score=9.0,
                         committed_id=3, dwell_s=100.0, params=p) is False


def test_should_not_switch_before_dwell():
    p = HysteresisParams(score_margin=5.0, min_dwell_s=3.0)
    # competitor beats by a lot, but we just committed -> hold
    assert should_switch(True, 10.0, best_id=7, best_score=100.0,
                         committed_id=3, dwell_s=0.5, params=p) is False


def test_should_not_switch_within_margin():
    p = HysteresisParams(score_margin=5.0, min_dwell_s=3.0)
    # dwell satisfied but competitor only marginally better -> hold (no oscillation)
    assert should_switch(True, 10.0, best_id=7, best_score=13.0,
                         committed_id=3, dwell_s=10.0, params=p) is False


def test_should_switch_beats_margin_after_dwell():
    p = HysteresisParams(score_margin=5.0, min_dwell_s=3.0)
    assert should_switch(True, 10.0, best_id=7, best_score=20.0,
                         committed_id=3, dwell_s=10.0, params=p) is True


# --- full pipeline + oscillation scenario -----------------------------------

def test_extract_frontiers_no_unknown_returns_guard_false():
    data, info = _grid([
        [0, 0, 0],
        [0, 100, 0],
        [0, 0, 0],
    ])
    clusters, has_unk = extract_frontiers(
        data, info, robot_xy=(0.0, 0.0), min_cells=1,
        score_params=ScoreParams(), id_quant_m=1.0)
    assert has_unk is False
    assert clusters == []


def test_extract_frontiers_two_near_equal_oscillation():
    # Symmetric map: two equal free columns (x=1, x=5) each bordering unknown,
    # separated by occupied walls, robot centred between them. Near-equal score.
    data, info = _grid([
        [-1, 0, 100, -1, 100, 0, -1],
        [-1, 0, 100, -1, 100, 0, -1],
        [-1, 0, 100, -1, 100, 0, -1],
    ])
    robot = info.cell_to_world(3, 1)  # centre column, middle row
    clusters, has_unk = extract_frontiers(
        data, info, robot_xy=robot, min_cells=1,
        score_params=ScoreParams(size_weight=1.0, distance_weight=2.0),
        id_quant_m=1.0)
    assert has_unk is True
    assert len(clusters) == 2
    left, right = sorted(clusters, key=lambda c: c.centroid_world[0])
    # symmetric -> equal size and (near) equal score
    assert left.size == right.size
    assert math.isclose(left.score, right.score, abs_tol=1e-6)
    # commit to the current best; the other is within margin -> must NOT switch
    best = clusters[0]
    other = clusters[1]
    hp = HysteresisParams(score_margin=5.0, min_dwell_s=3.0)
    assert should_switch(
        committed_present=True, committed_score=best.score,
        best_id=other.fid, best_score=other.score,
        committed_id=best.fid, dwell_s=100.0, params=hp) is False
