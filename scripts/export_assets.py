"""Generate the README proof assets (GIFs, screenshots, plots) in one command.

Runs the demo scenarios with video + GIF recording, then copies the best
frames into assets/ with recruiter-friendly file names:

    python scripts/export_assets.py

Run scripts/benchmark.py first if you also want the planner comparison chart.
"""
from __future__ import annotations

import logging
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.config import Config  # noqa: E402
from src.simulation.scenarios import build_scenario  # noqa: E402
from src.simulation.simulator import Simulation  # noqa: E402

# (scenario, file in outputs/<scenario>/, name in assets/)
COPIES = [
    ("warehouse", "demo.gif", "gifs/warehouse_demo.gif"),
    ("crosswalk", "demo.gif", "gifs/crosswalk_demo.gif"),
    ("city", "demo.gif", "gifs/city_demo.gif"),
    ("warehouse", "snapshot_start.png", "images/01_simulation_environment.png"),
    ("crosswalk", "snapshot_detection.png", "images/02_obstacle_detection_tracking.png"),
    ("warehouse", "snapshot_replan.png", "images/03_dynamic_replanning.png"),
    ("warehouse", "trajectory.png", "images/04_path_planning_trajectory.png"),
    ("warehouse", "final_frame.png", "images/05_goal_reached.png"),
    ("city", "trajectory.png", "images/06_city_trajectory.png"),
    ("crosswalk", "telemetry.png", "images/07_telemetry.png"),
    ("random", "trajectory.png", "images/08_random_map_trajectory.png"),
]


def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    outputs = ROOT / "outputs"
    for name in ["warehouse", "city", "crosswalk", "random"]:
        print(f"running {name} ...")
        r = Simulation(build_scenario(name), Config(), str(outputs / name),
                       render=True, show=False, video=True, gif=name != "random").run()
        print(f"  {'SUCCESS' if r.success else r.final_state}: {r.distance_driven_m} m, {r.sim_time_s} s, "
              f"{r.collisions} collisions")

    assets = ROOT / "assets"
    for scenario, src_name, dst_name in COPIES:
        src = outputs / scenario / src_name
        if src.exists():
            dst = assets / dst_name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(src, dst)
            print(f"  copied {dst.relative_to(ROOT)} ({dst.stat().st_size // 1024} KB)")
        else:
            print(f"  skipped {src_name} for {scenario} (event did not occur in this run)")
    bench = outputs / "benchmark" / "planner_comparison.png"
    if bench.exists():
        shutil.copy(bench, assets / "images" / "09_planner_comparison.png")
        print("  copied assets/images/09_planner_comparison.png")


if __name__ == "__main__":
    main()
