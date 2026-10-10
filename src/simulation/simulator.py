"""Simulation runner: the main loop that connects the world, robot and autonomy stack.

    for every 0.1 s step:
        world moves (pedestrians, forklifts)
        sensors observe the world            -> LiDAR scan + camera image
        navigator decides                    -> acceleration + steering
        robot moves (bicycle model)
        referee checks collisions against the ground truth
        dashboard renders a frame (optional) -> window / video / GIF
"""
from __future__ import annotations

import csv
import json
import logging
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

from src.config import Config
from src.navigation.navigator import NavState, Navigator
from src.simulation.robot import Robot
from src.simulation.sensors import BEVCamera, Lidar
from src.simulation.world import World

log = logging.getLogger("navsys")


@dataclass
class RunResult:
    scenario: str
    planner: str
    success: bool
    final_state: str
    sim_time_s: float
    steps: int
    distance_driven_m: float
    straight_line_m: float
    path_efficiency: float
    avg_speed_mps: float
    collisions: int
    min_clearance_m: float
    wall_time_s: float
    navigator: dict = field(default_factory=dict)
    output_dir: str = ""


class Simulation:
    def __init__(self, world: World, cfg: Config, output_dir: str | None = None,
                 render: bool = True, show: bool = False, video: bool = True, gif: bool = False) -> None:
        self.world = world
        self.cfg = cfg
        self.out = Path(output_dir) if output_dir else None
        self.render = render or show or video or gif
        self.show = show
        self.video = video and self.out is not None
        self.gif = gif and self.out is not None
        if self.out:
            self.out.mkdir(parents=True, exist_ok=True)

    def run(self) -> RunResult:
        cfg, world = self.cfg, self.world
        rng = np.random.default_rng(cfg.sim.seed)
        x, y, theta = world.start
        robot = Robot(cfg.robot, x, y, theta)
        lidar, camera = Lidar(cfg.sensor, rng), BEVCamera(cfg.sensor, rng)
        nav = Navigator(cfg, world.width, world.height, world.goal)

        dashboard = writer = None
        gif_frames, snapshots = [], set()
        if self.render:
            from src.visualization.dashboard import Dashboard
            dashboard = Dashboard(world)
        if self.video:
            fps = int(round(2 / cfg.sim.dt))   # 2x real time
            writer = cv2.VideoWriter(str(self.out / "run.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1280, 720))

        telemetry, trajectory = [], []
        collisions, in_collision, min_clear = 0, False, math.inf
        t_wall = time.perf_counter()
        log.info("scenario=%s planner=%s start=%s goal=%s", world.name, cfg.planner.algorithm,
                 world.start[:2], world.goal)

        step, extra_frames = 0, 0
        for step in range(cfg.sim.max_steps):
            s = robot.state
            world.step(cfg.sim.dt, (s.x, s.y), cfg.robot.radius)
            scan = lidar.scan(world, s)
            frame = camera.capture(world, s)
            accel, steer, dbg = nav.update(s, scan, frame)
            robot.step(accel, steer, cfg.sim.dt)

            # Referee: compare against the ground truth, which the robot never sees.
            hit = world.check_collision(s.x, s.y, cfg.robot.radius)
            if hit and not in_collision:
                collisions += 1
                log.warning("COLLISION at t=%.1fs (%.1f, %.1f)", world.time, s.x, s.y)
            in_collision = hit
            clearance = world.clearance(s.x, s.y) - cfg.robot.radius
            min_clear = min(min_clear, clearance)
            trajectory.append((s.x, s.y))
            telemetry.append((round(world.time, 2), s.x, s.y, s.theta, s.v, math.degrees(s.steer),
                              dbg.state.value, clearance, dbg.obstacle_distance))
            for ev in dbg.events:
                log.info("t=%5.1fs  %s", world.time, ev)

            if dashboard is not None:
                metrics = {"plans": sum(nav.plan_reasons.values()),
                           "replans": nav.plan_reasons["path_blocked"] + nav.plan_reasons["stuck"],
                           "collisions": collisions}
                img = dashboard.draw(robot, dbg, trajectory, frame, world.time, metrics)
                self._snapshots(img, dbg, step, snapshots)
                if writer is not None:
                    writer.write(img)
                if self.gif and step % 4 == 0:
                    gif_frames.append(cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA))
                if self.show:
                    cv2.imshow("AI Autonomous Navigation System", img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        log.info("stopped by user")
                        break
                    if key == ord("p"):
                        cv2.waitKey(0)

            if dbg.state in (NavState.GOAL_REACHED, NavState.FAILED):
                extra_frames += 1              # keep recording ~1.5 s so the video shows the finish
                if extra_frames > 15 or dashboard is None:
                    break

        wall = time.perf_counter() - t_wall
        if writer is not None:
            writer.release()
        if self.show:
            cv2.destroyAllWindows()

        traj = np.array(trajectory)
        straight = float(np.hypot(world.goal[0] - world.start[0], world.goal[1] - world.start[1]))
        result = RunResult(
            scenario=world.name, planner=cfg.planner.algorithm,
            success=nav.state == NavState.GOAL_REACHED and collisions == 0,
            final_state=nav.state.value, sim_time_s=round(world.time, 1), steps=step + 1,
            distance_driven_m=round(robot.odometer, 1), straight_line_m=round(straight, 1),
            path_efficiency=round(straight / max(robot.odometer, 1e-6), 3),
            avg_speed_mps=round(robot.odometer / max(world.time, 1e-6), 2),
            collisions=collisions, min_clearance_m=round(min_clear, 2),
            wall_time_s=round(wall, 1), navigator=nav.summary(),
            output_dir=str(self.out) if self.out else "",
        )
        if self.out:
            self._save_outputs(result, traj, telemetry, nav, gif_frames, dashboard is not None)
        return result

    # ------------------------------------------------------------------ files
    def _snapshots(self, img, dbg, step, taken: set) -> None:
        if not self.out:
            return
        wanted = []
        if step == 5:
            wanted.append("snapshot_start")
        if any(t.speed > 0.3 and np.linalg.norm(t.center - dbg.scan.origin) < 9.0 for t in dbg.tracks):
            wanted.append("snapshot_detection")   # a tracked, moving object close to the robot
        if any(e.startswith("replan:path_blocked") for e in dbg.events):
            wanted.append("snapshot_replan")
        if any(e == "state:EMERGENCY_STOP" for e in dbg.events):
            wanted.append("snapshot_emergency_stop")
        for name in wanted:
            if name not in taken:
                cv2.imwrite(str(self.out / f"{name}.png"), img)
                taken.add(name)
        self._last_frame = img

    def _save_outputs(self, result, traj, telemetry, nav, gif_frames, rendered) -> None:
        from src.visualization.plots import plot_telemetry, plot_trajectory

        with open(self.out / "metrics.json", "w") as f:
            json.dump(asdict(result), f, indent=2)
        with open(self.out / "telemetry.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "theta", "speed", "steer_deg", "state", "clearance", "obstacle_on_path"])
            w.writerows(telemetry)
        if rendered and hasattr(self, "_last_frame"):
            cv2.imwrite(str(self.out / "final_frame.png"), self._last_frame)

        status = "SUCCESS" if result.success else result.final_state
        plot_trajectory(self.world, traj, nav.first_path, str(self.out / "trajectory.png"),
                        f"{self.world.name}: {status} | {result.distance_driven_m} m in {result.sim_time_s} s | "
                        f"{result.navigator['replans_due_to_obstacles']} obstacle replans | {result.collisions} collisions")
        tel = list(zip(*telemetry))
        plot_telemetry(tel[0], tel[4], tel[5], tel[7], tel[6], str(self.out / "telemetry.png"),
                       self.cfg.robot.max_speed)
        if gif_frames:
            from PIL import Image
            frames = [Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)).quantize(colors=128) for f in gif_frames]
            frames[0].save(self.out / "demo.gif", save_all=True, append_images=frames[1:],
                           duration=100, loop=0, optimize=True)
