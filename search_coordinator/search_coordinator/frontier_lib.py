"""Pure frontier-extraction logic for the local explorer (ROADMAP 2.3).

ROS-free by design (math / collections / dataclasses only) so the frontier
detection, clustering, scoring and anti-oscillation hysteresis are unit-testable
without a running graph. The ROS node (`frontier_node.py`) feeds raw
OccupancyGrid data + params in and turns the result into FrontierArray / markers.

Occupancy-grid convention (`nav_msgs/OccupancyGrid`): -1 = unknown, 0 = free,
1..100 = occupancy probability (>= `OCC_THRESHOLD` treated as obstacle). A
frontier cell is a FREE cell with at least one 4-neighbour UNKNOWN cell — the
boundary between explored-free and not-yet-seen space.

Source contract (Phase 2.3 spike, must-fix #1): the input grid is the SLAM
occupancy grid (RTAB-Map `/map`), which carries -1 unknown cells. A rolling local
costmap without `track_unknown_space` has no unknown cells and yields zero
frontiers; `has_unknown()` lets the node fail loudly in that case.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

FREE = 0
UNKNOWN = -1
OCC_THRESHOLD = 65  # cells >= this are obstacles (typical costmap occupied threshold)


@dataclass(frozen=True)
class GridInfo:
    """Geometry of an OccupancyGrid (mirrors nav_msgs/MapMetaData we need)."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def cell_to_world(self, cx: float, cy: float) -> Tuple[float, float]:
        """Cell-centre (cx, cy in cell coords) -> (x, y) in the map frame."""
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)


@dataclass
class FrontierCluster:
    cells: List[int]                      # flat grid indices in this cluster
    centroid_cell: Tuple[float, float]    # (cx, cy) in cell coords
    centroid_world: Tuple[float, float]   # (x, y) in the map frame
    size: int
    fid: int = 0
    distance_m: float = 0.0
    score: float = 0.0


@dataclass(frozen=True)
class ScoreParams:
    size_weight: float = 1.0
    distance_weight: float = 2.0          # penalty per metre to the centroid


@dataclass(frozen=True)
class HysteresisParams:
    score_margin: float = 5.0             # competitor must beat committed by this much
    min_dwell_s: float = 3.0              # ...and committed must be held this long first


def has_unknown(data: Sequence[int]) -> bool:
    """True if the grid carries at least one UNKNOWN (-1) cell.

    Fail-loud guard: a grid with zero unknown cells can never yield a frontier.
    """
    for v in data:
        if v < 0:
            return True
    return False


def find_frontier_cells(data: Sequence[int], width: int, height: int) -> List[int]:
    """Flat indices of FREE cells with at least one 4-neighbour UNKNOWN cell."""
    out: List[int] = []
    for idx, v in enumerate(data):
        if v != FREE:
            continue
        x = idx % width
        y = idx // width
        if ((x > 0 and data[idx - 1] < 0) or
                (x < width - 1 and data[idx + 1] < 0) or
                (y > 0 and data[idx - width] < 0) or
                (y < height - 1 and data[idx + width] < 0)):
            out.append(idx)
    return out


def cluster_frontiers(frontier_cells: Sequence[int], info: GridInfo,
                      min_cells: int) -> List[FrontierCluster]:
    """Group adjacent frontier cells (8-connectivity) into clusters >= min_cells."""
    cellset = set(frontier_cells)
    visited = set()
    clusters: List[FrontierCluster] = []
    w, h = info.width, info.height
    for start in frontier_cells:
        if start in visited:
            continue
        comp: List[int] = []
        q = deque([start])
        visited.add(start)
        while q:
            idx = q.popleft()
            comp.append(idx)
            x = idx % w
            y = idx // w
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        nidx = ny * w + nx
                        if nidx in cellset and nidx not in visited:
                            visited.add(nidx)
                            q.append(nidx)
        if len(comp) >= min_cells:
            n = len(comp)
            ccx = sum((c % w) for c in comp) / n
            ccy = sum((c // w) for c in comp) / n
            clusters.append(FrontierCluster(
                cells=comp,
                centroid_cell=(ccx, ccy),
                centroid_world=info.cell_to_world(ccx, ccy),
                size=n))
    return clusters


def _zigzag(n: int) -> int:
    """Map signed int to non-negative int (0,-1,1,-2,2 -> 0,1,2,3,4)."""
    return 2 * n if n >= 0 else -2 * n - 1


def _cantor(a: int, b: int) -> int:
    """Bijective pairing of two non-negative ints into one."""
    return (a + b) * (a + b + 1) // 2 + b


def stable_frontier_id(centroid_world: Tuple[float, float], quant_m: float) -> int:
    """Deterministic, drift-stable id from the coarse-quantized centroid.

    Centroids that fall in the same `quant_m` cell across frames get the same id,
    so the executive can re-reference a frontier by id. Always >= 0 and well below
    2**31 for any sane map, so it never collides with the -1 "pick best" sentinel.
    """
    wx, wy = centroid_world
    qx = int(math.floor(wx / quant_m))
    qy = int(math.floor(wy / quant_m))
    return _cantor(_zigzag(qx), _zigzag(qy)) & 0x7FFFFFFF


def score_frontier(size: int, distance_m: float, params: ScoreParams) -> float:
    """Ranking score, higher = better: reward big frontiers, penalize distance."""
    return params.size_weight * size - params.distance_weight * distance_m


def should_switch(committed_present: bool, committed_score: float,
                  best_id: int, best_score: float, committed_id: int,
                  dwell_s: float, params: HysteresisParams) -> bool:
    """Anti-oscillation decision: switch the committed frontier?

    Stay on the committed frontier unless
      (a) nothing is committed yet, or
      (b) the committed frontier has vanished (switch immediately -> progress), or
      (c) a competitor beats it by > score_margin AND it has been held >= min_dwell_s.
    """
    if committed_id < 0:
        return True
    if not committed_present:
        return True
    if best_id == committed_id:
        return False
    if dwell_s < params.min_dwell_s:
        return False
    return best_score > committed_score + params.score_margin


def extract_frontiers(data: Sequence[int], info: GridInfo,
                      robot_xy: Tuple[float, float], min_cells: int,
                      score_params: ScoreParams,
                      id_quant_m: float) -> Tuple[List[FrontierCluster], bool]:
    """Full pipeline: cells -> clusters -> id/distance/score -> sorted best-first.

    Returns (clusters_sorted_best_first, source_has_unknown). When the grid has no
    unknown cells the second value is False and the list is empty (the node turns
    that into a loud log + an empty FrontierArray).
    """
    if not has_unknown(data):
        return [], False
    cells = find_frontier_cells(data, info.width, info.height)
    clusters = cluster_frontiers(cells, info, min_cells)
    rx, ry = robot_xy
    for c in clusters:
        c.fid = stable_frontier_id(c.centroid_world, id_quant_m)
        c.distance_m = math.hypot(c.centroid_world[0] - rx, c.centroid_world[1] - ry)
        c.score = score_frontier(c.size, c.distance_m, score_params)
    clusters.sort(key=lambda c: c.score, reverse=True)
    return clusters, True
