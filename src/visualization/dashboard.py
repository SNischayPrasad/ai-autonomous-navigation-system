"""Live OpenCV dashboard (1280 x 720) - this is what the demo video shows.

    +--------------------------------------------+------------------+
    |  WORLD VIEW (ground truth)                 | BEV CAMERA       |
    |  - inflated costmap the robot believes     | + CV detections  |
    |  - LiDAR rays, planned path, trajectory    +------------------+
    |  - robot, goal, moving obstacles           | OCCUPANCY BELIEF |
    +--------------------------------------------+------------------+
    |  TELEMETRY: state, speed, steering, plans, |  SPEED PLOT      |
    +--------------------------------------------+------------------+
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from src.navigation.navigator import NavDebug, NavState
from src.perception.obstacle_detector import ObstacleDetector
from src.simulation.robot import Robot
from src.simulation.sensors import CameraFrame
from src.simulation.world import World

W, H = 1280, 720
MAP_W = 880
PANEL_X = MAP_W
BG = (24, 24, 28)
TEXT = (235, 235, 235)
DIM = (150, 150, 160)
CYAN = (255, 220, 0)
ORANGE = (0, 165, 255)
GREEN = (90, 230, 90)
RED = (60, 60, 255)
PURPLE = (160, 70, 140)

STATE_COLORS = {
    NavState.PLANNING: (0, 200, 255),
    NavState.FOLLOWING: GREEN,
    NavState.EMERGENCY_STOP: RED,
    NavState.RECOVERY: (255, 120, 255),
    NavState.GOAL_REACHED: (255, 255, 0),
    NavState.FAILED: RED,
}


def _text(img, s, org, scale=0.48, color=TEXT, thick=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class Dashboard:
    def __init__(self, world: World) -> None:
        self.world = world
        self.scale = MAP_W / world.width                 # pixels per metre
        self.map_h = int(round(world.height * self.scale))
        self.speed_history: list = []

    def w2p(self, pt) -> tuple:
        return int(round(pt[0] * self.scale)), int(round(pt[1] * self.scale))

    # ------------------------------------------------------------------ main
    def draw(self, robot: Robot, dbg: NavDebug, trajectory: list, frame: CameraFrame,
             sim_time: float, metrics: dict) -> np.ndarray:
        canvas = np.full((H, W, 3), BG, np.uint8)
        canvas[:self.map_h, :MAP_W] = self._world_view(robot, dbg, trajectory)
        self._side_panel(canvas, robot, dbg, frame)
        self._telemetry(canvas, robot, dbg, sim_time, metrics)
        return canvas

    def _world_view(self, robot: Robot, dbg: NavDebug, trajectory: list) -> np.ndarray:
        img = cv2.resize(self.world.render(), (MAP_W, self.map_h), interpolation=cv2.INTER_NEAREST)

        # What the robot BELIEVES: inflated costmap as a translucent purple overlay.
        lethal = cv2.resize(dbg.costmap.lethal.astype(np.uint8), (MAP_W, self.map_h),
                            interpolation=cv2.INTER_NEAREST).astype(bool)
        overlay = img.copy()
        overlay[lethal] = PURPLE
        img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)

        # LiDAR rays
        origin = self.w2p(dbg.scan.origin)
        rays = img.copy()
        for end, hit in zip(dbg.scan.end_points(), dbg.scan.hits):
            cv2.line(rays, origin, self.w2p(end), (80, 200, 80) if hit else (60, 110, 60), 1, cv2.LINE_AA)
        img = cv2.addWeighted(rays, 0.35, img, 0.65, 0)
        for p in dbg.scan.hit_points():
            cv2.circle(img, self.w2p(p), 2, (120, 255, 120), -1, cv2.LINE_AA)

        # Driven trajectory and current plan
        if len(trajectory) > 1:
            pts = np.array([self.w2p(p) for p in trajectory], np.int32)
            cv2.polylines(img, [pts], False, ORANGE, 2, cv2.LINE_AA)
        if dbg.path is not None and len(dbg.path) > 1:
            pts = np.array([self.w2p(p) for p in dbg.path], np.int32)
            cv2.polylines(img, [pts], False, CYAN, 2, cv2.LINE_AA)
        if dbg.lookahead_point is not None:
            cv2.circle(img, self.w2p(dbg.lookahead_point), 5, CYAN, 2, cv2.LINE_AA)

        # Tracked moving obstacles: ID + predicted motion over the next 2 s
        for t in dbg.tracks:
            c = self.w2p(t.center)
            cv2.circle(img, c, int((t.radius + 0.5) * self.scale), (0, 255, 255), 1, cv2.LINE_AA)
            if t.speed > 0.2:
                cv2.arrowedLine(img, c, self.w2p(t.predict(2.0)), (0, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)
            _text(img, f"ID{t.track_id} {t.speed:.1f}m/s", (c[0] + 10, c[1] - 10), 0.4, (0, 255, 255))

        # Start and goal
        sx, sy, _ = self.world.start
        cv2.circle(img, self.w2p((sx, sy)), 7, (200, 200, 200), 2, cv2.LINE_AA)
        g = self.w2p(self.world.goal)
        cv2.circle(img, g, 12, GREEN, 2, cv2.LINE_AA)
        cv2.circle(img, g, 4, GREEN, -1, cv2.LINE_AA)
        _text(img, "GOAL", (g[0] - 18, g[1] - 16), 0.45, GREEN)

        # Robot body + heading
        body = np.array([self.w2p(p) for p in robot.footprint()], np.int32)
        color = STATE_COLORS[dbg.state]
        cv2.fillPoly(img, [body], (245, 245, 245))
        cv2.polylines(img, [body], True, color, 2, cv2.LINE_AA)
        s = robot.state
        tip = (s.x + 1.8 * math.cos(s.theta), s.y + 1.8 * math.sin(s.theta))
        cv2.arrowedLine(img, self.w2p((s.x, s.y)), self.w2p(tip), color, 2, cv2.LINE_AA, tipLength=0.4)

        if dbg.state == NavState.EMERGENCY_STOP:
            cv2.rectangle(img, (2, 2), (MAP_W - 3, self.map_h - 3), RED, 4)
            _text(img, "EMERGENCY STOP - OBSTACLE IN PATH", (MAP_W // 2 - 200, 34), 0.7, RED, 2)
        elif dbg.state == NavState.GOAL_REACHED:
            _text(img, "GOAL REACHED", (MAP_W // 2 - 95, 34), 0.8, (255, 255, 0), 2)
        _text(img, f"{self.world.name.upper()}  |  {self.world.description}", (10, self.map_h - 10), 0.45, DIM)
        return img

    def _side_panel(self, canvas, robot: Robot, dbg: NavDebug, frame: CameraFrame) -> None:
        x0 = PANEL_X + 10
        _text(canvas, "BEV CAMERA + CV DETECTION", (x0, 22), 0.5, TEXT)
        cam = ObstacleDetector.annotate(frame, dbg.detections)
        cam = cv2.resize(cam, (220, 220), interpolation=cv2.INTER_LINEAR)
        canvas[30:250, PANEL_X + 90:PANEL_X + 310] = cam
        _text(canvas, f"static blobs: {len(dbg.detections.static):2d}   moving: {len(dbg.detections.dynamic)}",
              (x0, 270), 0.45, DIM)

        _text(canvas, "OCCUPANCY BELIEF + PLAN", (x0, 296), 0.5, TEXT)
        cm = dbg.costmap
        belief = np.full(cm.lethal.shape + (3,), (40, 40, 40), np.uint8)
        belief[cm.lethal] = PURPLE
        belief[cm.occupied] = (230, 230, 230)
        if dbg.dynamic_layer is not None:
            belief[dbg.dynamic_layer] = (60, 60, 255)
        bw, bh = 380, int(380 * cm.lethal.shape[0] / cm.lethal.shape[1])
        belief = cv2.resize(belief, (bw, bh), interpolation=cv2.INTER_NEAREST)
        sc = bw / self.world.width
        if dbg.path is not None and len(dbg.path) > 1:
            pts = (dbg.path * sc).astype(np.int32)
            cv2.polylines(belief, [pts], False, CYAN, 1, cv2.LINE_AA)
        cv2.circle(belief, (int(robot.state.x * sc), int(robot.state.y * sc)), 3, (0, 255, 0), -1)
        canvas[306:306 + bh, x0:x0 + bw] = belief

    def _telemetry(self, canvas, robot: Robot, dbg: NavDebug, sim_time: float, metrics: dict) -> None:
        y0 = self.map_h + 10
        cv2.rectangle(canvas, (0, self.map_h), (MAP_W, H), (32, 32, 38), -1)
        s = robot.state
        state_color = STATE_COLORS[dbg.state]
        _text(canvas, "STATE", (14, y0 + 22), 0.45, DIM)
        _text(canvas, dbg.state.value, (14, y0 + 50), 0.75, state_color, 2)

        plan = dbg.last_plan
        cols = [
            [("time", f"{sim_time:6.1f} s"), ("speed", f"{s.v:4.2f} m/s"),
             ("steering", f"{math.degrees(s.steer):+5.1f} deg")],
            [("dist to goal", f"{np.linalg.norm(np.subtract(self.world.goal, (s.x, s.y))):5.1f} m"),
             ("driven", f"{robot.odometer:5.1f} m"),
             ("obstacle on path", "none" if math.isinf(dbg.obstacle_distance) else f"{dbg.obstacle_distance:4.1f} m")],
            [("plans / replans", f"{metrics.get('plans', 0)} / {metrics.get('replans', 0)}"),
             ("last plan", f"{plan.time_ms:5.1f} ms, {plan.expanded} nodes" if plan else "-"),
             ("collisions", f"{metrics.get('collisions', 0)}")],
        ]
        for ci, col in enumerate(cols):
            x = 230 + ci * 215
            for ri, (k, v) in enumerate(col):
                _text(canvas, k, (x, y0 + 22 + ri * 48), 0.42, DIM)
                _text(canvas, v, (x, y0 + 42 + ri * 48), 0.52, TEXT)
        _text(canvas, "A* planning | OpenCV perception | LiDAR+camera fusion | pure pursuit",
              (14, H - 14), 0.42, DIM)

        # Speed sparkline in the bottom-right corner
        self.speed_history.append(s.v)
        hist = self.speed_history[-300:]
        gx, gy, gw, gh = PANEL_X + 10, self.map_h + 30, 380, H - self.map_h - 45
        _text(canvas, "SPEED (last 30 s)", (gx, self.map_h + 20), 0.45, DIM)
        cv2.rectangle(canvas, (gx, gy), (gx + gw, gy + gh), (60, 60, 66), 1)
        if len(hist) > 1:
            xs = np.linspace(gx, gx + gw, len(hist))
            ys = gy + gh - np.array(hist) / max(robot.cfg.max_speed, 1e-6) * gh
            cv2.polylines(canvas, [np.stack([xs, ys], 1).astype(np.int32)], False, GREEN, 1, cv2.LINE_AA)
