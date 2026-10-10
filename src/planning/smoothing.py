"""Path post-processing: remove grid zig-zags and resample evenly.

A raw grid path only moves in 45-degree steps. A car cannot follow that
nicely, so we
  1. shortcut it (keep jumping ahead while the straight line stays safe), then
  2. resample it into evenly spaced waypoints for the controller.
"""
from __future__ import annotations

import numpy as np


def line_is_free(blocked: np.ndarray, a: tuple, b: tuple) -> bool:
    """Check every cell touched by the straight segment a -> b."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    rs = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    cs = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return not blocked[rs, cs].any()


def shortcut_path(cells: list, blocked: np.ndarray, max_jump: int = 40) -> list:
    """Greedy forward string-pulling. O(n) line checks."""
    if len(cells) <= 2:
        return list(cells)
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = i + 1
        while j + 1 < len(cells) and j + 1 - i <= max_jump and line_is_free(blocked, cells[i], cells[j + 1]):
            j += 1
        out.append(cells[j])
        i = j
    return out


def resample(points: np.ndarray, spacing: float) -> np.ndarray:
    """Return points spaced `spacing` metres apart along the polyline."""
    if len(points) < 2:
        return points.copy()
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    if arc[-1] < 1e-6:
        return points[:1].copy()
    samples = np.append(np.arange(0.0, arc[-1], spacing), arc[-1])
    return np.stack([np.interp(samples, arc, points[:, 0]), np.interp(samples, arc, points[:, 1])], axis=1)


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
