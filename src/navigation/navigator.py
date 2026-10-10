"""The navigator: decision making that ties perception, planning and control together.

Each control cycle (10 Hz) runs the classic sense -> think -> act loop:

    1. PERCEIVE  camera detections -> tracks with velocity; LiDAR + camera -> occupancy grid
    2. MAP       occupancy grid -> inflated costmap
    3. DECIDE    goal reached? need to (re)plan? is the path ahead blocked?
    4. PLAN      A* / Dijkstra on the costmap -> smoothed waypoint path
    5. SAFETY    check the swept corridor along the path against raw LiDAR hits
    6. ACT       pure pursuit -> steering + acceleration

Behaviour is a small finite-state machine:

    PLANNING --path found--> FOLLOWING --obstacle in corridor--> EMERGENCY_STOP
        ^                        |   ^                                |
        |                        |   +------- corridor clear ---------+
        +---- path blocked ------+
    EMERGENCY_STOP --static blocker, replan did not help--> RECOVERY (reverse) --> PLANNING
    FOLLOWING --within goal tolerance--> GOAL_REACHED
    too many failed plans --> FAILED
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from src.config import Config
from src.control.pure_pursuit import PurePursuitController
from src.perception.obstacle_detector import DetectionResult, ObstacleDetector
from src.perception.occupancy_grid import OccupancyGrid
from src.perception.tracker import MultiObjectTracker
from src.planning.astar import PlanResult, plan_path
from src.planning.costmap import Costmap, build_costmap
from src.planning.smoothing import resample, shortcut_path
from src.simulation.robot import RobotState
from src.simulation.sensors import CameraFrame, LidarScan


class NavState(str, Enum):
    PLANNING = "PLANNING"
    FOLLOWING = "FOLLOWING"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RECOVERY = "RECOVERY"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


@dataclass
class NavDebug:
    """Everything the dashboard needs to explain what the robot is thinking."""
    state: NavState
    detections: DetectionResult
    costmap: Costmap
    scan: LidarScan
    path: np.ndarray | None
    lookahead_point: np.ndarray | None = None
    speed_limit: float = 0.0
    target_speed: float = 0.0
    obstacle_distance: float = math.inf
    last_plan: PlanResult | None = None
    dynamic_layer: np.ndarray | None = None
    tracks: list = field(default_factory=list)
    events: list = field(default_factory=list)


class Navigator:
    def __init__(self, cfg: Config, world_width: float, world_height: float, goal: tuple) -> None:
        self.cfg = cfg
        self.goal = np.asarray(goal, float)
        self.grid = OccupancyGrid(world_width, world_height, cfg.planner.grid_resolution, cfg.perception)
        self.detector = ObstacleDetector(cfg.perception)
        self.tracker = MultiObjectTracker(cfg.sim.dt)
        self.controller = PurePursuitController(cfg.control, cfg.robot)

        self.state = NavState.PLANNING
        self.path: np.ndarray | None = None
        self.last_plan: PlanResult | None = None
        self.steps_since_plan = 0
        self.blocked_steps = 0
        self.plan_failures = 0
        self.force_replan = False
        self.stuck_replans = 0
        self.recovery_steps = 0
        self.recoveries = 0

        self.plan_reasons: Counter = Counter()
        self.plan_times_ms: list = []
        self.nodes_expanded: list = []
        self.emergency_stops = 0
        self.dynamic_detections = 0
        self.first_path: np.ndarray | None = None

    # ================================================================ main loop
    def update(self, rs: RobotState, scan: LidarScan, frame: CameraFrame):
        """One control cycle. Returns (accel, steer, NavDebug)."""
        cfg, events = self.cfg, []
        pos = rs.position

        # 1. PERCEIVE ---------------------------------------------------------
        det = self.detector.detect(frame)
        moving = det.dynamic
        self.dynamic_detections += len(moving)
        self.grid.update_from_camera(frame, det.static_mask)
        self.grid.update_from_lidar(scan, ignore_circles=[(d.center, d.radius + 0.6) for d in moving])
        tracks = self.tracker.update(moving)
        self.grid.set_dynamic(tracks, cfg.perception.dynamic_inflation, cfg.perception.prediction_horizon_s)

        # 2. MAP ---------------------------------------------------------------
        costmap = build_costmap(self.grid.static_occupied(), self.grid.dynamic, self.grid.resolution,
                                cfg.robot.radius, cfg.planner.safety_margin,
                                cfg.planner.soft_cost_distance, cfg.planner.soft_cost_weight)
        debug = NavDebug(self.state, det, costmap, scan, self.path, dynamic_layer=self.grid.dynamic, tracks=tracks, events=events)

        # 3. DECIDE ------------------------------------------------------------
        dist_goal = float(np.linalg.norm(self.goal - pos))
        if self.state in (NavState.GOAL_REACHED, NavState.FAILED):
            return self._brake(rs, debug)
        if dist_goal < cfg.control.goal_tolerance:
            self._set_state(NavState.GOAL_REACHED, events)
            return self._brake(rs, debug)

        if self.state == NavState.RECOVERY:
            return self._recover(rs, scan, debug, events)

        # 4. PLAN --------------------------------------------------------------
        self.steps_since_plan += 1
        reason = self._replan_reason(pos, costmap)
        if reason:
            self._plan(pos, rs.theta, costmap, reason, events)
        debug.path, debug.last_plan = self.path, self.last_plan
        if self.path is None:
            backoff_steps = int(cfg.control.dynamic_block_backoff_s / cfg.sim.dt)
            if self.plan_failures >= cfg.control.max_plan_failures:
                self._set_state(NavState.FAILED, events)
            elif self.plan_failures % backoff_steps == 0 and self.grid.dynamic.any():
                # Something moving blocks every route: give way by backing off a little.
                self.recovery_steps = 0
                self.recoveries += 1
                self._set_state(NavState.RECOVERY, events)
            else:
                self._set_state(NavState.PLANNING, events)
            return self._brake(rs, debug)

        # 5. SAFETY ------------------------------------------------------------
        obstacle_dist, blocker = self._corridor_obstacle_distance(rs, scan.hit_points())
        debug.obstacle_distance = obstacle_dist
        braking = max(rs.v, 0.0) ** 2 / (2 * cfg.robot.max_decel)
        stop_dist = cfg.control.stop_distance + braking
        if obstacle_dist < stop_dist:
            if self.state != NavState.EMERGENCY_STOP:
                self.emergency_stops += 1
            self._set_state(NavState.EMERGENCY_STOP, events)
            self.blocked_steps += 1
            if self.blocked_steps * cfg.sim.dt >= cfg.control.blocked_replan_s:
                self.blocked_steps = 0
                r, c = self.grid.world_to_cell(*blocker)
                moving_blocker = bool(self.grid.dynamic[r, c])
                if (self.stuck_replans >= 1 and not moving_blocker) or self.stuck_replans >= 3:
                    # Static blocker + replanning did not help: we are pinned against a wall.
                    # Moving blocker for ~6 s: a person is waiting for US (mutual yield
                    # deadlock). Either way, back up (recovery behaviour, as in ROS move_base).
                    self.stuck_replans, self.recovery_steps = 0, 0
                    self.recoveries += 1
                    self._set_state(NavState.RECOVERY, events)
                else:
                    self.force_replan = True
                    self.stuck_replans += 1
            return self._brake(rs, debug)
        self.blocked_steps = 0
        if rs.v > 0.5:
            self.stuck_replans = 0

        span = max(cfg.control.slow_down_distance - stop_dist, 1e-3)
        speed_limit = cfg.robot.max_speed * float(np.clip((obstacle_dist - stop_dist) / span, 0.2, 1.0))
        if any(np.linalg.norm(t.center - pos) < cfg.control.caution_radius for t in tracks):
            speed_limit = min(speed_limit, cfg.control.caution_speed)

        # 6. ACT ---------------------------------------------------------------
        cmd = self.controller.compute(rs, self.path, speed_limit, dist_goal)
        self._set_state(NavState.FOLLOWING, events)
        debug.state = self.state
        debug.lookahead_point = cmd.lookahead_point
        debug.speed_limit = speed_limit
        debug.target_speed = cmd.target_speed
        return cmd.accel, cmd.steer, debug

    # ================================================================ helpers
    def _set_state(self, new: NavState, events: list) -> None:
        if new != self.state:
            events.append(f"state:{new.value}")
        self.state = new

    def _brake(self, rs: RobotState, debug: NavDebug):
        """Bring the speed to exactly zero (forward or reverse) without overshooting."""
        debug.state = self.state
        return -rs.v / self.cfg.sim.dt, rs.steer, debug

    def _recover(self, rs: RobotState, scan: LidarScan, debug: NavDebug, events: list):
        """Reverse with counter-steer so the nose swings towards the path."""
        cfg = self.cfg
        self.recovery_steps += 1
        pos = rs.position
        heading = np.array([math.cos(rs.theta), math.sin(rs.theta)])
        hits = scan.hit_points()
        rear_blocked = False
        if len(hits):
            rel = hits - pos
            lon = rel @ heading
            lat = np.abs(rel @ np.array([-heading[1], heading[0]]))
            rear_blocked = bool(np.any((lon < 0) & (lon > -1.7) & (lat < 1.0)))
        if rear_blocked or self.recovery_steps * cfg.sim.dt >= cfg.control.recovery_time_s:
            self.force_replan = True
            self._set_state(NavState.PLANNING, events)
            return self._brake(rs, debug)

        if self.path is not None:
            nearest = int(np.argmin(np.linalg.norm(self.path - pos, axis=1)))
            target = self.path[min(len(self.path) - 1, nearest + 6)]
        else:
            target = self.goal
        alpha = math.atan2(target[1] - rs.y, target[0] - rs.x) - rs.theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        steer = -math.copysign(math.radians(cfg.robot.max_steer_deg), alpha)
        accel = cfg.control.speed_kp * (-0.8 - rs.v)
        debug.state = self.state
        return accel, steer, debug

    def _replan_reason(self, pos: np.ndarray, costmap: Costmap) -> str | None:
        if self.path is None:
            return "initial" if self.last_plan is None else "retry"
        if self.force_replan:
            self.force_replan = False
            return "stuck"
        # Is any upcoming waypoint now inside an inflated obstacle?
        d = np.linalg.norm(self.path - pos, axis=1)
        nearest = int(np.argmin(d))
        upcoming = self.path[nearest:]
        upcoming = upcoming[(d[nearest:] > 1.0) & (d[nearest:] < 20.0)]
        if len(upcoming):
            rows = np.clip((upcoming[:, 1] / costmap.resolution).astype(int), 0, costmap.lethal.shape[0] - 1)
            cols = np.clip((upcoming[:, 0] / costmap.resolution).astype(int), 0, costmap.lethal.shape[1] - 1)
            if costmap.lethal[rows, cols].any():
                return "path_blocked"
        if self.steps_since_plan * self.cfg.sim.dt >= self.cfg.planner.replan_interval_s:
            return "periodic"
        return None

    def _behind_mask(self, pos: np.ndarray, heading: float, costmap: Costmap) -> np.ndarray:
        """Half-disc of cells directly behind the robot."""
        res, (rows, cols) = costmap.resolution, costmap.lethal.shape
        k = int(2.0 / res)
        r0, c0 = int(pos[1] / res), int(pos[0] / res)
        mask = np.zeros((rows, cols), bool)
        yy, xx = np.mgrid[max(0, r0 - k):min(rows, r0 + k + 1), max(0, c0 - k):min(cols, c0 + k + 1)]
        dx, dy = (xx + 0.5) * res - pos[0], (yy + 0.5) * res - pos[1]
        behind = (dx * math.cos(heading) + dy * math.sin(heading) < -0.3) & (dx * dx + dy * dy <= 4.0)
        mask[yy[behind], xx[behind]] = True
        return mask

    def _plan(self, pos: np.ndarray, heading: float, costmap: Costmap, reason: str, events: list) -> None:
        pc = self.cfg.planner
        start = self.grid.world_to_cell(*pos)
        goal = self.grid.world_to_cell(*self.goal)
        # Prefer paths that leave in the direction the car is facing (it cannot turn
        # on the spot); fall back to an unconstrained search if that is impossible.
        result = plan_path(costmap, start, goal, pc.algorithm, pc.weighted_astar_w,
                           extra_blocked=self._behind_mask(pos, heading, costmap))
        if not result.success:
            result = plan_path(costmap, start, goal, pc.algorithm, pc.weighted_astar_w)
        self.last_plan = result
        self.steps_since_plan = 0
        self.plan_reasons[reason] += 1
        self.plan_times_ms.append(result.time_ms)
        self.nodes_expanded.append(result.expanded)
        if reason not in ("periodic", "initial"):
            events.append(f"replan:{reason}")

        if not result.success:
            self.plan_failures += 1
            self.path = None
            return
        self.plan_failures = 0
        # Shortcut only through clearly safe cells (keep a margin from the inflation zone).
        safe = costmap.lethal | (costmap.cost > 1.0 + 0.5 * pc.soft_cost_weight)
        cells = shortcut_path(result.cells, safe)
        pts = np.array([self.grid.cell_to_world(r, c) for r, c in cells])
        pts[0], pts[-1] = pos, self.goal
        new_path = resample(pts, pc.path_spacing)

        # Hysteresis: a routine (periodic) replan only replaces the current path if it
        # is clearly better. Without this the robot can flip-flop between two
        # near-equal routes and end up mid-turn facing a wall.
        if reason == "periodic" and self.path is not None:
            nearest = int(np.argmin(np.linalg.norm(self.path - pos, axis=1)))
            old_cost = self._path_cost(np.vstack([pos, self.path[nearest:]]), costmap)
            if self._path_cost(new_path, costmap) > self.cfg.control.replan_hysteresis * old_cost:
                return
        self.path = new_path
        if self.first_path is None:
            self.first_path = self.path.copy()

    @staticmethod
    def _path_cost(path: np.ndarray, costmap: Costmap) -> float:
        """Integrated traversal cost of a polyline; infinite if it crosses an obstacle."""
        if len(path) < 2:
            return 0.0
        seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
        rows = np.clip((path[1:, 1] / costmap.resolution).astype(int), 0, costmap.cost.shape[0] - 1)
        cols = np.clip((path[1:, 0] / costmap.resolution).astype(int), 0, costmap.cost.shape[1] - 1)
        arc = np.cumsum(seg)
        if costmap.lethal[rows, cols][arc > 1.0].any():
            return math.inf
        return float((seg * costmap.cost[rows, cols]).sum())

    def _corridor_obstacle_distance(self, rs: RobotState, hits: np.ndarray):
        """Distance along the path to the first LiDAR hit inside the robot's swept corridor.

        Returns (distance, hit_point). distance is inf when the corridor is clear.
        """
        if self.path is None or len(hits) == 0:
            return math.inf, None
        pos = rs.position
        heading = np.array([math.cos(rs.theta), math.sin(rs.theta)])
        hits = hits[(hits - pos) @ heading > 0.0]          # only obstacles in front
        if len(hits) == 0:
            return math.inf, None

        d = np.linalg.norm(self.path - pos, axis=1)
        nearest = int(np.argmin(d))
        pts = np.vstack([pos, self.path[nearest:]])
        arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        keep = arc <= self.cfg.control.slow_down_distance + 1.0
        pts, arc = pts[keep], arc[keep]

        dist = np.linalg.norm(hits[:, None, :] - pts[None, :, :], axis=2)   # (hits, path pts)
        inside = dist < self.cfg.control.corridor_half_width
        rows = inside.any(axis=1)
        if not rows.any():
            return math.inf, None
        first_arc = arc[np.argmax(inside[rows], axis=1)]
        k = int(np.argmin(first_arc))
        return float(first_arc[k]), hits[rows][k]

    # ================================================================ metrics
    def summary(self) -> dict:
        times = self.plan_times_ms or [0.0]
        return {
            "plans_total": int(sum(self.plan_reasons.values())),
            "plans_by_reason": dict(self.plan_reasons),
            "replans_due_to_obstacles": int(self.plan_reasons["path_blocked"] + self.plan_reasons["stuck"]),
            "avg_plan_time_ms": round(float(np.mean(times)), 2),
            "max_plan_time_ms": round(float(np.max(times)), 2),
            "avg_nodes_expanded": int(np.mean(self.nodes_expanded or [0])),
            "emergency_stops": self.emergency_stops,
            "recoveries": self.recoveries,
            "map_explored_pct": round(100 * self.grid.explored_fraction(), 1),
        }
