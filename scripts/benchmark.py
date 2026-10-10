"""Benchmark the navigation stack and compare path-planning algorithms.

Part 1 - planner micro-benchmark: every scenario map is fully known, plan
         start -> goal with A*, Dijkstra and Weighted A*; record runtime,
         expanded nodes and path length.
Part 2 - closed-loop benchmark: full autonomous runs (unknown map, moving
         obstacles) on all scenarios plus several random seeds.

Usage:
    python scripts/benchmark.py              # full benchmark
    python scripts/benchmark.py --quick      # fewer random seeds
Outputs (outputs/benchmark/):
    planner_comparison.csv / .png, closed_loop_results.csv, summary.md
"""
from __future__ import annotations

import argparse
import csv
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.config import Config  # noqa: E402
from src.planning.astar import plan_path  # noqa: E402
from src.planning.costmap import build_costmap  # noqa: E402
from src.planning.smoothing import path_length  # noqa: E402
from src.simulation.scenarios import build_scenario  # noqa: E402
from src.simulation.simulator import Simulation  # noqa: E402

PLANNERS = ["dijkstra", "astar", "weighted_astar"]
FIXED_SCENARIOS = ["warehouse", "city", "crosswalk"]


def planner_benchmark(out: Path, cfg: Config, seeds: list) -> list:
    rows = []
    worlds = [build_scenario(n) for n in FIXED_SCENARIOS] + [build_scenario("random", s) for s in seeds]
    res = cfg.planner.grid_resolution
    for world in worlds:
        occ = world.static_grid(res)
        cm = build_costmap(occ, np.zeros_like(occ), res, cfg.robot.radius, cfg.planner.safety_margin,
                           cfg.planner.soft_cost_distance, cfg.planner.soft_cost_weight)
        start = (int(world.start[1] / res), int(world.start[0] / res))
        goal = (int(world.goal[1] / res), int(world.goal[0] / res))
        for algo in PLANNERS:
            times = [plan_path(cm, start, goal, algo, cfg.planner.weighted_astar_w) for _ in range(3)]
            r = min(times, key=lambda p: p.time_ms)
            cells = np.array(r.cells, float)[:, ::-1] * res if r.success else np.zeros((0, 2))
            rows.append({"map": world.name, "planner": algo, "success": r.success,
                         "time_ms": round(r.time_ms, 2), "nodes_expanded": r.expanded,
                         "path_length_m": round(path_length(cells), 1), "path_cost": round(r.cost, 1)})
            print(f"  {world.name:<16}{algo:<16}{r.time_ms:8.2f} ms {r.expanded:8d} nodes "
                  f"{rows[-1]['path_length_m']:7.1f} m")
    _write_csv(out / "planner_comparison.csv", rows)

    # Bar chart: average expanded nodes and time per planner
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, key, label in ((axes[0], "nodes_expanded", "nodes expanded"), (axes[1], "time_ms", "planning time [ms]")):
        means = [np.mean([r[key] for r in rows if r["planner"] == p]) for p in PLANNERS]
        bars = ax.bar(["Dijkstra", "A*", "Weighted A*"], means, color=["#8c8c8c", "#2a7ab8", "#e08a1e"])
        ax.bar_label(bars, fmt="%.0f" if key == "nodes_expanded" else "%.1f")
        ax.set_title(f"Average {label} ({len(worlds)} maps)")
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "planner_comparison.png", dpi=120)
    plt.close(fig)
    return rows


def closed_loop_benchmark(out: Path, seeds: list) -> list:
    rows = []
    jobs = [(n, 7) for n in FIXED_SCENARIOS] + [("random", s) for s in seeds]
    for name, seed in jobs:
        for algo in PLANNERS:
            cfg = Config()
            cfg.planner.algorithm = algo
            cfg.sim.seed = seed
            r = Simulation(build_scenario(name, seed), cfg, output_dir=None, render=False, video=False).run()
            rows.append({"scenario": r.scenario, "planner": algo, "success": r.success,
                         "sim_time_s": r.sim_time_s, "distance_m": r.distance_driven_m,
                         "collisions": r.collisions, "min_clearance_m": r.min_clearance_m,
                         "obstacle_replans": r.navigator["replans_due_to_obstacles"],
                         "avg_plan_ms": r.navigator["avg_plan_time_ms"],
                         "emergency_stops": r.navigator["emergency_stops"],
                         "recoveries": r.navigator["recoveries"]})
            print(f"  {r.scenario:<16}{algo:<16}{'OK ' if r.success else 'FAIL'} {r.sim_time_s:6.1f} s "
                  f"{r.distance_driven_m:6.1f} m  collisions={r.collisions}  plan={r.navigator['avg_plan_time_ms']} ms")
    _write_csv(out / "closed_loop_results.csv", rows)
    return rows


def _write_csv(path: Path, rows: list) -> None:
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def write_summary(out: Path, planner_rows: list, loop_rows: list) -> None:
    lines = ["# Benchmark summary", "", "## Planner comparison (fully known maps)", "",
             "| Planner | Avg time (ms) | Avg nodes expanded | Avg path length (m) |", "|---|---|---|---|"]
    for p in PLANNERS:
        sel = [r for r in planner_rows if r["planner"] == p]
        lines.append(f"| {p} | {np.mean([r['time_ms'] for r in sel]):.1f} | "
                     f"{np.mean([r['nodes_expanded'] for r in sel]):.0f} | "
                     f"{np.mean([r['path_length_m'] for r in sel]):.1f} |")
    lines += ["", "## Closed-loop autonomous runs (unknown map + moving obstacles)", "",
              "| Planner | Runs | Success rate | Collisions | Avg time to goal (s) | Avg min clearance (m) |",
              "|---|---|---|---|---|---|"]
    for p in PLANNERS:
        sel = [r for r in loop_rows if r["planner"] == p]
        ok = [r for r in sel if r["success"]]
        lines.append(f"| {p} | {len(sel)} | {100 * len(ok) / len(sel):.0f} % | "
                     f"{sum(r['collisions'] for r in sel)} | "
                     f"{np.mean([r['sim_time_s'] for r in ok]) if ok else float('nan'):.1f} | "
                     f"{np.mean([r['min_clearance_m'] for r in sel]):.2f} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    seeds = [1, 2, 3] if args.quick else [1, 2, 3, 4, 5, 6, 7, 8]
    out = ROOT / "outputs" / "benchmark"
    out.mkdir(parents=True, exist_ok=True)
    print("Part 1: planner comparison")
    planner_rows = planner_benchmark(out, Config(), seeds)
    print("\nPart 2: closed-loop runs")
    loop_rows = closed_loop_benchmark(out, seeds)
    write_summary(out, planner_rows, loop_rows)
    print(f"\nSaved results to {out}")


if __name__ == "__main__":
    main()
