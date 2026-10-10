"""Unit tests for the camera detector, the occupancy grid and the tracker."""
from types import SimpleNamespace

import numpy as np
import pytest

from src.config import PerceptionConfig, SensorConfig
from src.perception.obstacle_detector import ObstacleDetector
from src.perception.occupancy_grid import OccupancyGrid
from src.perception.tracker import MultiObjectTracker
from src.simulation.robot import RobotState
from src.simulation.sensors import BEVCamera, Lidar
from src.simulation.world import CircleObstacle, DynamicObstacle, RectObstacle, World


def small_world(dynamic=None):
    static = [RectObstacle(20, 5, 2, 10)]
    return World("test", 40, 20, static, dynamic or [], start=(5, 10, 0), goal=(35, 10))


def test_detector_finds_moving_object_at_correct_position():
    ped = DynamicObstacle([(12.0, 13.0)], radius=0.5, speed=0.0)
    world = small_world([ped])
    rng = np.random.default_rng(0)
    frame = BEVCamera(SensorConfig(), rng).capture(world, RobotState(10, 10, 0))
    result = ObstacleDetector(PerceptionConfig()).detect(frame)
    moving = result.dynamic
    assert len(moving) == 1
    assert np.linalg.norm(moving[0].center - np.array([12.0, 13.0])) < 0.3
    assert moving[0].radius == pytest.approx(0.5, abs=0.2)
    assert len(result.static) >= 1                       # the wall is also seen


def test_lidar_measures_distance_to_wall():
    world = small_world()
    scan = Lidar(SensorConfig(lidar_noise_std=0.0), np.random.default_rng(0)).scan(world, RobotState(10, 10, 0))
    forward = np.argmin(np.abs(scan.angles))              # beam pointing at +x
    assert scan.hits[forward]
    assert scan.ranges[forward] == pytest.approx(10.0, abs=0.2)


def test_occupancy_grid_learns_obstacles_and_free_space():
    world = small_world()
    cfg = PerceptionConfig()
    grid = OccupancyGrid(world.width, world.height, 0.25, cfg)
    lidar = Lidar(SensorConfig(lidar_noise_std=0.0), np.random.default_rng(0))
    for _ in range(3):
        grid.update_from_lidar(lidar.scan(world, RobotState(10, 10, 0)))
    occ = grid.static_occupied()
    assert occ[grid.world_to_cell(20.1, 10)]              # wall surface is mapped
    assert not occ[grid.world_to_cell(15, 10)]            # space in front of it is free
    assert grid.evidence[grid.world_to_cell(15, 10)] < 0


def test_lidar_hits_on_moving_objects_are_not_mapped():
    ped = DynamicObstacle([(14.0, 10.0)], radius=0.5, speed=0.0)
    world = small_world([ped])
    grid = OccupancyGrid(world.width, world.height, 0.25, PerceptionConfig())
    scan = Lidar(SensorConfig(lidar_noise_std=0.0), np.random.default_rng(0)).scan(world, RobotState(10, 10, 0))
    grid.update_from_lidar(scan, ignore_circles=[(np.array([14.0, 10.0]), 1.2)])
    assert not grid.static_occupied()[grid.world_to_cell(13.5, 10)]


def test_tracker_estimates_velocity_and_keeps_ids():
    tracker = MultiObjectTracker(dt=0.1, smoothing=1.0)
    for k in range(5):
        det = SimpleNamespace(center=np.array([10.0 + 0.1 * k, 5.0]), radius=0.5)
        tracks = tracker.update([det])
    assert len(tracks) == 1 and tracks[0].track_id == 1
    assert tracks[0].velocity == pytest.approx([1.0, 0.0], abs=1e-6)
    assert tracks[0].predict(2.0) == pytest.approx([12.4, 5.0])


def test_world_collision_check():
    world = World("t", 20, 20, [CircleObstacle(10, 10, 1.0)], [], (2, 2, 0), (18, 18))
    assert world.check_collision(10, 11.5, 0.6)
    assert not world.check_collision(10, 13.0, 0.6)
