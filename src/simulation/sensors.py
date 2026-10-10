"""Simulated sensors: a 2D LiDAR and a bird's-eye-view (BEV) camera.

Both sensors add noise, so the perception stack has to cope with imperfect
data exactly as it would on a real robot.

* LiDAR  - casts rays from the robot and returns the distance to the first
           obstacle along each ray (occlusions are handled naturally).
* Camera - returns a top-down colour image of the area around the robot.
           Self-driving stacks (Tesla, Waymo, many warehouse robots) fuse
           their camera views into exactly this kind of BEV representation.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from src.config import SensorConfig
from src.simulation.robot import RobotState
from src.simulation.world import COLOR_STATIC, World


@dataclass
class LidarScan:
    origin: np.ndarray       # (2,) sensor position in world frame
    angles: np.ndarray       # (N,) beam angles in the world frame [rad]
    ranges: np.ndarray       # (N,) measured distance [m]
    hits: np.ndarray         # (N,) True if the beam hit something before max range
    max_range: float

    def hit_points(self) -> np.ndarray:
        """(K, 2) world coordinates of beams that returned an obstacle."""
        r = self.ranges[self.hits]
        a = self.angles[self.hits]
        return self.origin + np.stack([np.cos(a) * r, np.sin(a) * r], axis=1)

    def end_points(self) -> np.ndarray:
        return self.origin + np.stack([np.cos(self.angles), np.sin(self.angles)], axis=1) * self.ranges[:, None]


@dataclass
class CameraFrame:
    image: np.ndarray        # BGR image, camera_px x camera_px
    origin: np.ndarray       # world (x, y) of the image's top-left corner
    m_per_px: float          # metres per pixel
    view_m: float            # side length of the covered square [m]

    def pixel_to_world(self, px: float, py: float) -> np.ndarray:
        return self.origin + np.array([px, py]) * self.m_per_px


class Lidar:
    def __init__(self, cfg: SensorConfig, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.rng = rng
        self.offsets = np.linspace(-np.pi, np.pi, cfg.lidar_beams, endpoint=False)

    def scan(self, world: World, state: RobotState) -> LidarScan:
        angles = state.theta + self.offsets
        step = world.resolution * 1.5
        dists = np.arange(step, self.cfg.lidar_range + step, step)
        xs = state.x + np.cos(angles)[:, None] * dists[None, :]
        ys = state.y + np.sin(angles)[:, None] * dists[None, :]
        occ = world.is_occupied(xs, ys)                      # (beams, samples)

        hits = occ.any(axis=1)
        first = np.argmax(occ, axis=1)
        ranges = np.where(hits, dists[first], self.cfg.lidar_range)
        noise = self.rng.normal(0.0, self.cfg.lidar_noise_std, size=ranges.shape)
        ranges = np.where(hits, np.clip(ranges + noise, 0.05, self.cfg.lidar_range), ranges)
        return LidarScan(state.position.copy(), angles, ranges, hits, self.cfg.lidar_range)


class BEVCamera:
    """North-up top-down camera centred on the robot."""

    def __init__(self, cfg: SensorConfig, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.rng = rng

    def capture(self, world: World, state: RobotState) -> CameraFrame:
        half = self.cfg.camera_view_m / 2
        origin = np.array([state.x - half, state.y - half])
        full = world.render()
        res = world.resolution

        # Crop the window around the robot; anything outside the map looks like a wall.
        n = int(round(self.cfg.camera_view_m / res))
        c0, r0 = int(round(origin[0] / res)), int(round(origin[1] / res))
        crop = np.empty((n, n, 3), np.uint8)
        crop[:] = COLOR_STATIC
        sc0, sr0 = max(c0, 0), max(r0, 0)
        sc1, sr1 = min(c0 + n, world.cols), min(r0 + n, world.rows)
        if sc1 > sc0 and sr1 > sr0:
            crop[sr0 - r0:sr1 - r0, sc0 - c0:sc1 - c0] = full[sr0:sr1, sc0:sc1]

        px = self.cfg.camera_px
        image = cv2.resize(crop, (px, px), interpolation=cv2.INTER_AREA)
        if self.cfg.camera_noise_std > 0:
            noise = self.rng.normal(0.0, self.cfg.camera_noise_std, image.shape)
            image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return CameraFrame(image, origin, self.cfg.camera_view_m / px, self.cfg.camera_view_m)
