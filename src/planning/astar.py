"""Grid search planners: A*, Dijkstra and Weighted A* in one implementation.

All three are "best-first" searches that pop the node with the lowest
    f(n) = g(n) + w * h(n)
where g is the cost so far and h is the octile-distance heuristic.

    Dijkstra       w = 0   explores in all directions, always optimal
    A*             w = 1   heuristic focuses the search, still optimal
                           (octile distance never over-estimates)
    Weighted A*    w > 1   greedier and faster, path at most w x optimal

The search runs on an 8-connected grid and refuses to cut diagonally between
two blocked cells (the robot would clip the corner).
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from src.planning.costmap import Costmap

SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
HEURISTIC_WEIGHTS = {"dijkstra": 0.0, "astar": 1.0}


@dataclass
class PlanResult:
    success: bool
    cells: list = field(default_factory=list)   # [(row, col), ...] start -> goal
    cost: float = math.inf
    expanded: int = 0                            # nodes popped from the open list
    time_ms: float = 0.0
    message: str = ""


def _octile(r: int, c: int, gr: int, gc: int) -> float:
    dr, dc = abs(r - gr), abs(c - gc)
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def plan_path(costmap: Costmap, start: tuple, goal: tuple,
              algorithm: str = "astar", weight: float = 2.0,
              extra_blocked: np.ndarray | None = None) -> PlanResult:
    """Search a path from start to goal (both (row, col) cells).

    extra_blocked: optional mask of additional no-go cells, e.g. the area right
    behind a car-like robot that cannot turn on the spot.
    """
    t0 = time.perf_counter()
    w = HEURISTIC_WEIGHTS.get(algorithm, weight)
    rows, cols = costmap.cost.shape
    sr, sc = start
    gr, gc = goal

    lethal = costmap.lethal.copy()
    # The robot may be standing inside the inflation zone (e.g. an obstacle was
    # only just discovered next to it). Let it drive out of that zone, but
    # never through a cell that really contains an obstacle.
    k = costmap.inflation_cells
    r0, r1, c0, c1 = max(0, sr - k), min(rows, sr + k + 1), max(0, sc - k), min(cols, sc + k + 1)
    yy, xx = np.mgrid[r0:r1, c0:c1]
    disk = (yy - sr) ** 2 + (xx - sc) ** 2 <= k * k
    lethal[r0:r1, c0:c1] &= ~(disk & ~costmap.occupied[r0:r1, c0:c1])
    if extra_blocked is not None:
        lethal |= extra_blocked
        lethal[sr, sc] = False

    if costmap.occupied[sr, sc]:
        return PlanResult(False, message="start is inside an obstacle", time_ms=(time.perf_counter() - t0) * 1e3)
    if lethal[gr, gc]:
        return PlanResult(False, message="goal is blocked", time_ms=(time.perf_counter() - t0) * 1e3)

    # Flat Python lists are much faster than numpy scalar indexing in a tight loop.
    cost = costmap.cost.ravel().tolist()
    blocked = lethal.ravel().tolist()
    n = rows * cols
    g = [math.inf] * n
    parent = [-1] * n
    closed = bytearray(n)

    s_idx, g_idx = sr * cols + sc, gr * cols + gc
    g[s_idx] = 0.0
    open_heap = [(w * _octile(sr, sc, gr, gc), 0.0, s_idx)]
    expanded = 0

    while open_heap:
        _, g_cur, idx = heapq.heappop(open_heap)
        if closed[idx]:
            continue
        closed[idx] = 1
        expanded += 1
        if idx == g_idx:
            break
        r, c = divmod(idx, cols)
        for dr, dc, length in MOVES:
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= rows or nc >= cols:
                continue
            n_idx = nr * cols + nc
            if blocked[n_idx] or closed[n_idx]:
                continue
            if dr and dc and (blocked[r * cols + nc] or blocked[nr * cols + c]):
                continue  # no corner cutting
            new_g = g_cur + length * 0.5 * (cost[idx] + cost[n_idx])
            if new_g < g[n_idx]:
                g[n_idx] = new_g
                parent[n_idx] = idx
                heapq.heappush(open_heap, (new_g + w * _octile(nr, nc, gr, gc), new_g, n_idx))

    elapsed = (time.perf_counter() - t0) * 1e3
    if not closed[g_idx]:
        return PlanResult(False, expanded=expanded, time_ms=elapsed, message="no path exists")

    cells, idx = [], g_idx
    while idx != -1:
        cells.append(divmod(idx, cols))
        idx = parent[idx]
    cells.reverse()
    return PlanResult(True, cells, g[g_idx] * costmap.resolution, expanded, elapsed, "ok")
