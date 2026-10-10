"""Scenario library: ready-made test environments.

Each scenario mirrors a real deployment of autonomous navigation:

* warehouse - shelf aisles, a forklift crossing the main aisle, and a pallet
              wall that blocks the planned route (the robot must discover it
              and re-plan through another aisle).
* city      - city blocks, parked cars, pedestrians on crosswalks and a road
              closed for construction.
* crosswalk - narrow street with pedestrians crossing; tests yielding / stopping.
* random    - procedurally generated obstacle field (seeded, always solvable)
              for stress-testing and benchmarking.
"""
from __future__ import annotations

import math

import numpy as np

from src.simulation.world import CircleObstacle, DynamicObstacle, RectObstacle, World

WORLD_W, WORLD_H = 80.0, 50.0


def _border(w: float = WORLD_W, h: float = WORLD_H, t: float = 0.5) -> list:
    return [RectObstacle(0, 0, w, t), RectObstacle(0, h - t, w, t),
            RectObstacle(0, 0, t, h), RectObstacle(w - t, 0, t, h)]


def warehouse() -> World:
    static = _border()
    for i in range(6):                     # six shelf columns, cross aisle at y = 21..29
        x = 12 + i * 10
        static.append(RectObstacle(x, 6, 3, 15))
        static.append(RectObstacle(x, 29, 3, 15))
    static.append(RectObstacle(46, 21, 5, 8))       # pallet stack blocking the cross aisle
    static += [RectObstacle(70, 30, 6, 3), CircleObstacle(72, 42, 1.2)]   # loading-dock clutter

    dynamic = [
        DynamicObstacle([(28.5, 8.0), (28.5, 42.0)], radius=1.0, speed=1.2, label="forklift"),
        DynamicObstacle([(58.5, 31.0), (58.5, 43.0)], radius=0.5, speed=1.0, label="worker"),
        DynamicObstacle([(66.5, 8.5), (78.5, 8.5)], radius=0.5, speed=0.9, label="worker"),
    ]
    return World("warehouse", WORLD_W, WORLD_H, static, dynamic,
                 start=(4.0, 25.0, 0.0), goal=(74.0, 12.0),
                 description="Warehouse AMR: shelf aisles, forklift traffic, blocked aisle")


def city() -> World:
    static = _border()
    blocks_x = [(8, 22), (30, 46), (54, 70)]
    blocks_y = [(8, 20), (28, 42)]
    for x0, x1 in blocks_x:
        for y0, y1 in blocks_y:
            mid = (x0 + x1) / 2
            static.append(RectObstacle(x0, y0, mid - x0 - 0.75, y1 - y0))      # two buildings
            static.append(RectObstacle(mid + 0.75, y0, x1 - mid - 0.75, y1 - y0))  # with an alley
    static += [  # parked cars along the kerbs
        RectObstacle(12, 20.6, 4.5, 1.8), RectObstacle(58, 26.6, 4.5, 1.8),
        RectObstacle(46.6, 10, 1.8, 4.5), RectObstacle(28.6, 33, 1.8, 4.5),
    ]
    static.append(RectObstacle(54.5, 21.0, 1.2, 6.5))  # construction barrier closes a road
    static += [CircleObstacle(55.5 + 1.2 * k, 21.5, 0.4) for k in range(3)]   # traffic cones

    dynamic = [
        DynamicObstacle([(23.0, 15.0), (29.0, 15.0)], radius=0.45, speed=0.8),
        DynamicObstacle([(47.0, 33.0), (53.0, 33.0)], radius=0.45, speed=0.7),
        DynamicObstacle([(38.0, 21.0), (38.0, 27.0)], radius=0.45, speed=0.9),
        DynamicObstacle([(71.0, 36.0), (78.5, 36.0)], radius=0.6, speed=2.0, label="cyclist"),
    ]
    return World("city", WORLD_W, WORLD_H, static, dynamic,
                 start=(4.0, 4.0, 0.0), goal=(75.0, 46.0),
                 description="Urban delivery robot: blocks, crosswalks, road closure")


def crosswalk() -> World:
    """Narrow street with three pedestrian crossings: the robot must yield."""
    static = _border()
    gaps = [(23.5, 27.5), (43.5, 47.5), (60.5, 64.5)]      # crosswalk openings between buildings
    edges = [0.5] + [v for g in gaps for v in g] + [79.5]
    for x0, x1 in zip(edges[::2], edges[1::2]):
        static.append(RectObstacle(x0, 0.5, x1 - x0, 20.5))   # north buildings, kerb at y = 21
        static.append(RectObstacle(x0, 29.0, x1 - x0, 20.5))  # south buildings, kerb at y = 29
    dynamic = [
        DynamicObstacle([(25.5, 17.0), (25.5, 33.0)], radius=0.45, speed=1.0),
        DynamicObstacle([(45.5, 33.0), (45.5, 17.0)], radius=0.45, speed=0.8),
        DynamicObstacle([(62.5, 17.0), (62.5, 33.0)], radius=0.6, speed=1.3, label="cyclist"),
    ]
    return World("crosswalk", WORLD_W, WORLD_H, static, dynamic,
                 start=(4.0, 25.0, 0.0), goal=(76.0, 25.0),
                 description="Narrow street, 3 crosswalks: yield to pedestrians")


def random_field(seed: int = 7, n_obstacles: int = 28) -> World:
    """Random obstacle field that is guaranteed to have a collision-free route."""
    from src.planning.astar import plan_path
    from src.planning.costmap import build_costmap

    rng = np.random.default_rng(seed)
    start, goal = (4.0, 25.0, 0.0), (76.0, 25.0)
    for _ in range(50):
        static = _border()
        while len(static) < n_obstacles + 4:
            x, y = rng.uniform(8, 72), rng.uniform(3, 47)
            if math.hypot(x - start[0], y - start[1]) < 6 or math.hypot(x - goal[0], y - goal[1]) < 6:
                continue
            if rng.random() < 0.5:
                static.append(CircleObstacle(x, y, rng.uniform(1.0, 2.5)))
            else:
                static.append(RectObstacle(x, y, rng.uniform(2, 6), rng.uniform(2, 6)))
        dynamic = []
        for _ in range(3):
            x = rng.uniform(20, 60)
            dynamic.append(DynamicObstacle([(x, rng.uniform(5, 15)), (x, rng.uniform(35, 45))],
                                           radius=0.5, speed=rng.uniform(0.6, 1.2)))
        world = World(f"random_seed{seed}", WORLD_W, WORLD_H, static, dynamic, start, goal,
                      description=f"Procedural obstacle field (seed {seed})")
        # Accept the map only if it is solvable with full knowledge of the obstacles.
        res = 0.5
        occ = world.static_grid(res)
        cm = build_costmap(occ, np.zeros_like(occ), res, 0.9, 0.4, 2.0, 5.0)
        plan = plan_path(cm, (int(start[1] / res), int(start[0] / res)), (int(goal[1] / res), int(goal[0] / res)))
        if plan.success:
            return world
    raise RuntimeError("could not generate a solvable random map")


SCENARIOS = {
    "warehouse": warehouse,
    "city": city,
    "crosswalk": crosswalk,
    "random": random_field,
}


def build_scenario(name: str, seed: int = 7) -> World:
    if name not in SCENARIOS:
        raise ValueError(f"unknown scenario '{name}', choose from {list(SCENARIOS)}")
    return SCENARIOS[name](seed) if name == "random" else SCENARIOS[name]()
