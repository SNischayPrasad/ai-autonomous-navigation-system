"""AI-Based Autonomous Navigation System - command-line entry point.

Examples
--------
    python main.py                                  # warehouse scenario, live window
    python main.py --scenario city --headless       # no window, still saves video + plots
    python main.py --scenario all --headless --gif  # run every scenario, also export GIFs
    python main.py --scenario random --seed 42 --planner dijkstra
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from src.config import Config
from src.simulation.scenarios import SCENARIOS, build_scenario
from src.simulation.simulator import Simulation

ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System (2D simulation)")
    p.add_argument("--scenario", default="warehouse", choices=[*SCENARIOS, "all"])
    p.add_argument("--planner", default="astar", choices=["astar", "dijkstra", "weighted_astar"])
    p.add_argument("--seed", type=int, default=7, help="seed for the random map and sensor noise")
    p.add_argument("--headless", action="store_true", help="do not open a window")
    p.add_argument("--no-video", action="store_true", help="skip writing run.mp4")
    p.add_argument("--gif", action="store_true", help="also export demo.gif")
    p.add_argument("--max-steps", type=int, default=None)
    p.add_argument("--output-dir", default=str(ROOT / "outputs"))
    return p.parse_args()


def setup_logging(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fmt = logging.Formatter("%(asctime)s | %(levelname)-7s | %(message)s", "%H:%M:%S")
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.handlers.clear()
    for handler in (logging.StreamHandler(sys.stdout), logging.FileHandler(out_dir / "run.log", mode="w")):
        handler.setFormatter(fmt)
        root.addHandler(handler)


def print_summary(r) -> None:
    nav = r.navigator
    print("\n" + "=" * 62)
    print(f" RESULT: {'SUCCESS' if r.success else 'NOT COMPLETED'}  ({r.scenario}, {r.planner})")
    print("=" * 62)
    rows = [
        ("Final state", r.final_state),
        ("Simulated time", f"{r.sim_time_s} s  ({r.steps} control steps)"),
        ("Distance driven", f"{r.distance_driven_m} m  (straight line {r.straight_line_m} m)"),
        ("Average speed", f"{r.avg_speed_mps} m/s"),
        ("Collisions", r.collisions),
        ("Min clearance", f"{r.min_clearance_m} m"),
        ("Plans / obstacle replans", f"{nav['plans_total']} / {nav['replans_due_to_obstacles']}"),
        ("Avg plan time", f"{nav['avg_plan_time_ms']} ms  ({nav['avg_nodes_expanded']} nodes)"),
        ("Emergency stops", nav["emergency_stops"]),
        ("Map explored", f"{nav['map_explored_pct']} %"),
        ("Outputs", r.output_dir),
    ]
    for k, v in rows:
        print(f"  {k:<26}{v}")
    print("=" * 62)


def main() -> int:
    args = parse_args()
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    all_ok = True
    for name in names:
        cfg = Config()
        cfg.planner.algorithm = args.planner
        cfg.sim.seed = args.seed
        if args.max_steps:
            cfg.sim.max_steps = args.max_steps
        out = Path(args.output_dir) / name
        setup_logging(out)
        world = build_scenario(name, args.seed)
        sim = Simulation(world, cfg, str(out), render=True, show=not args.headless,
                         video=not args.no_video, gif=args.gif)
        result = sim.run()
        print_summary(result)
        all_ok &= result.success
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
