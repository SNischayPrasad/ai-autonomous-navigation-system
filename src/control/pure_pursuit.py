"""Pure Pursuit path tracking + speed control.

Pure Pursuit (Coulter, CMU 1992) is still one of the most widely used path
trackers in mobile robotics. It picks a "carrot" point on the path one
lookahead distance Ld ahead and steers along the circular arc that reaches it:

    delta = atan( 2 * L * sin(alpha) / Ld )

where alpha is the angle between the robot heading and the carrot. A longer
lookahead at higher speed gives smoother, more stable driving.

Speed is the minimum of several limits: max speed, the safety limit coming
from the navigator, a curvature limit (slow in turns) and a braking curve that
makes the robot arrive at the goal smoothly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.config import ControlConfig, RobotConfig
from src.simulation.robot import RobotState


@dataclass
class ControlCommand:
    accel: float
    steer: float
    target_speed: float
    lookahead_point: np.ndarray
    nearest_index: int


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuitController:
    def __init__(self, cfg: ControlConfig, robot: RobotConfig) -> None:
        self.cfg = cfg
        self.robot = robot
        self.max_steer = math.radians(robot.max_steer_deg)

    def compute(self, state: RobotState, path: np.ndarray, speed_limit: float,
                dist_to_goal: float) -> ControlCommand:
        pos = state.position
        d = np.linalg.norm(path - pos, axis=1)
        nearest = int(np.argmin(d))

        lookahead = float(np.clip(self.cfg.lookahead_base + self.cfg.lookahead_gain * state.v,
                                  self.cfg.lookahead_base, 6.0))
        ahead = np.nonzero(d[nearest:] >= lookahead)[0]
        target = path[nearest + ahead[0]] if len(ahead) else path[-1]

        alpha = wrap_angle(math.atan2(target[1] - state.y, target[0] - state.x) - state.theta)
        ld = max(lookahead, float(np.linalg.norm(target - pos)), 1e-3)
        steer = math.atan2(2.0 * self.robot.wheelbase * math.sin(alpha), ld)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))

        v_curve = self.robot.max_speed / (1.0 + self.cfg.curvature_speed_gain * abs(steer))
        v_arrive = math.sqrt(2.0 * 1.0 * max(dist_to_goal - 0.3, 0.0))   # comfortable 1 m/s^2 stop
        v_ahead = self.curve_speed_limit(path, nearest)
        target_speed = min(self.robot.max_speed, speed_limit, v_curve, v_arrive, v_ahead)
        if abs(alpha) > math.radians(60):
            target_speed = min(target_speed, 1.0)  # big heading error: turn slowly
        accel = self.cfg.speed_kp * (target_speed - state.v)
        return ControlCommand(accel, steer, target_speed, target, nearest)

    def curve_speed_limit(self, path: np.ndarray, start: int, horizon_m: float = 12.0,
                          decel: float = 2.0) -> float:
        """Look ahead along the path and slow down BEFORE a curve, not inside it.

        Curvature kappa is estimated from heading changes over a ~2 m window;
        the comfortable cornering speed is sqrt(a_lat / kappa). A curve that is
        s metres away still allows sqrt(v_curve^2 + 2 * decel * s) now.
        """
        seg = path[start:start + int(horizon_m / 0.5) + 2]
        if len(seg) < 4:
            return self.robot.max_speed
        d = np.diff(seg, axis=0)
        step = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
        heading = np.arctan2(d[:, 1], d[:, 0])
        dh = np.abs((np.diff(heading) + np.pi) % (2 * np.pi) - np.pi)
        window = 4
        # heading change over ~2 m (centred window, same length as dh)
        turn = np.convolve(dh, np.ones(window), mode="full")[window // 2:window // 2 + len(dh)]
        kappa = turn / (window * step[1:])
        v_curve = np.sqrt(self.cfg.max_lateral_accel / np.maximum(kappa, 1e-6))
        s = np.cumsum(step)[1:]
        allowed = np.sqrt(v_curve ** 2 + 2 * decel * s)
        return float(min(self.robot.max_speed, allowed.min()))
