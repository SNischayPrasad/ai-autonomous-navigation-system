"""Unit tests for the costmap, A*/Dijkstra planner and path smoothing."""
import numpy as np
import pytest

from src.planning.astar import plan_path
from src.planning.costmap import build_costmap
from src.planning.smoothing import line_is_free, path_length, resample, shortcut_path

RES = 0.5


def make_costmap(occ):
    return build_costmap(occ, np.zeros_like(occ), RES, robot_radius=0.4, safety_margin=0.1,
                         soft_distance=1.0, soft_weight=2.0)


def empty_grid(rows=40, cols=60):
    return np.zeros((rows, cols), bool)


def test_straight_line_on_empty_map():
    cm = make_costmap(empty_grid())
    r = plan_path(cm, (20, 5), (20, 50))
    assert r.success
    assert r.cells[0] == (20, 5) and r.cells[-1] == (20, 50)
    assert all(row == 20 for row, _ in r.cells)          # never leaves the straight row


def test_path_goes_through_gap_in_wall():
    occ = empty_grid()
    occ[:, 30] = True
    occ[18:23, 30] = False                                # 5-cell door in the wall
    cm = make_costmap(occ)
    r = plan_path(cm, (5, 5), (35, 55))
    assert r.success
    assert any(c == 30 and 18 <= row <= 22 for row, c in r.cells)
    assert not any(cm.lethal[row, c] for row, c in r.cells[1:])


def test_no_path_when_fully_blocked():
    occ = empty_grid()
    occ[:, 30] = True
    r = plan_path(make_costmap(occ), (5, 5), (35, 55))
    assert not r.success
    assert r.message == "no path exists"


@pytest.mark.parametrize("algo", ["astar", "weighted_astar"])
def test_heuristic_search_expands_fewer_nodes_than_dijkstra(algo):
    cm = make_costmap(empty_grid())
    dijkstra = plan_path(cm, (20, 5), (20, 50), "dijkstra")
    other = plan_path(cm, (20, 5), (20, 50), algo)
    assert other.expanded < dijkstra.expanded


def test_astar_is_optimal_like_dijkstra():
    occ = empty_grid()
    occ[5:35, 25] = True
    cm = make_costmap(occ)
    a = plan_path(cm, (20, 5), (20, 50), "astar")
    d = plan_path(cm, (20, 5), (20, 50), "dijkstra")
    assert a.cost == pytest.approx(d.cost, rel=1e-6)


def test_shortcut_removes_zigzag_and_keeps_endpoints():
    blocked = np.zeros((20, 20), bool)
    cells = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (4, 5), (4, 6)]
    out = shortcut_path(cells, blocked)
    assert out[0] == cells[0] and out[-1] == cells[-1]
    assert len(out) < len(cells)
    assert line_is_free(blocked, out[0], out[-1])


def test_resample_spacing():
    pts = np.array([[0.0, 0.0], [10.0, 0.0]])
    rs = resample(pts, 0.5)
    assert len(rs) == 21
    assert path_length(rs) == pytest.approx(10.0)
