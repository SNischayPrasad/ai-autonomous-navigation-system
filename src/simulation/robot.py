"""Virtual car-like robot using the kinematic bicycle model.

The bicycle model is the standard first-order model used in autonomous
driving research: the car is reduced to one front wheel (steerable) and one
rear wheel, which is accurate at low speeds without tyre slip.

    x'     = v * cos(theta)
    y'     = v * sin(theta)
    theta' = v / L * tan(delta)
    v'     = a
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.config import RobotConfig


@dataclass
class RobotState:
    x: float
    y: float
    theta: float          # heading [rad], 0 = facing +x
    v: float = 0.0        # forward speed [m/s]
    steer: float = 0.0    # current front-wheel angle [rad]

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y])


class Robot:
    def __init__(self, cfg: RobotConfig, x: float, y: float, theta: float) -> None:
        self.cfg = cfg
        self.state = RobotState(x, y, theta)
        self.max_steer = math.radians(cfg.max_steer_deg)
        self.odometer = 0.0   # total distance driven [m]

    def step(self, accel: float, steer: float, dt: float) -> RobotState:
        """Apply one control command (acceleration, steering) for `dt` seconds."""
        s = self.state
        accel = float(np.clip(accel, -self.cfg.max_decel, self.cfg.max_accel))
        s.steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        s.v = float(np.clip(s.v + accel * dt, -self.cfg.max_reverse_speed, self.cfg.max_speed))

        s.x += s.v * math.cos(s.theta) * dt
        s.y += s.v * math.sin(s.theta) * dt
        s.theta += s.v / self.cfg.wheelbase * math.tan(s.steer) * dt
        s.theta = (s.theta + math.pi) % (2 * math.pi) - math.pi
        self.odometer += abs(s.v) * dt
        return s

    def footprint(self) -> np.ndarray:
        """Four body corners in world coordinates (for drawing)."""
        s, half_l, half_w = self.state, self.cfg.length / 2, self.cfg.width / 2
        local = np.array([[half_l, half_w], [half_l, -half_w], [-half_l, -half_w], [-half_l, half_w]])
        c, si = math.cos(s.theta), math.sin(s.theta)
        rot = np.array([[c, -si], [si, c]])
        return local @ rot.T + np.array([s.x, s.y])
