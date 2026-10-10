"""Unit tests for the vehicle model and the pure pursuit controller."""
import math

import numpy as np
import pytest

from src.config import ControlConfig, RobotConfig
from src.control.pure_pursuit import PurePursuitController
from src.simulation.robot import Robot, RobotState


def test_robot_drives_straight():
    robot = Robot(RobotConfig(), 0.0, 0.0, 0.0)
    for _ in range(10):
        robot.step(accel=1.0, steer=0.0, dt=0.1)
    assert robot.state.y == pytest.approx(0.0)
    assert robot.state.x > 0.4
    assert robot.state.v == pytest.approx(1.0)


def test_robot_respects_limits():
    cfg = RobotConfig()
    robot = Robot(cfg, 0.0, 0.0, 0.0)
    robot.step(accel=100.0, steer=3.0, dt=0.1)
    assert robot.state.steer == pytest.approx(math.radians(cfg.max_steer_deg))
    assert robot.state.v == pytest.approx(cfg.max_accel * 0.1)


def test_controller_steers_towards_path():
    ctrl = PurePursuitController(ControlConfig(), RobotConfig())
    state = RobotState(0.0, 0.0, 0.0, v=1.0)
    path_left = np.array([[x, 3.0] for x in np.arange(0, 20, 0.5)])     # +y side
    path_ahead = np.array([[x, 0.0] for x in np.arange(0, 20, 0.5)])
    assert ctrl.compute(state, path_left, 3.0, 20.0).steer > 0.1
    assert ctrl.compute(state, path_ahead, 3.0, 20.0).steer == pytest.approx(0.0, abs=1e-6)


def test_controller_slows_before_sharp_corner():
    ctrl = PurePursuitController(ControlConfig(), RobotConfig())
    straight = np.array([[x, 0.0] for x in np.arange(0, 30, 0.5)])
    # 90-degree corner 2 m ahead: too close to brake comfortably from full speed
    corner = np.vstack([[[x, 0.0] for x in np.arange(0, 2, 0.5)], [[2.0, y] for y in np.arange(0, 20, 0.5)]])
    assert ctrl.curve_speed_limit(corner, 0) < ctrl.curve_speed_limit(straight, 0)
