"""Occupancy grid: the robot's own map of the world, built from sensor data.

The robot starts knowing NOTHING about the obstacles (every cell is assumed
free). Each sensor reading adds evidence:

    obstacle observed  -> evidence += hit_evidence
    free space seen    -> evidence -= miss_evidence
    evidence >= threshold  => cell is treated as an obstacle

This is a simplified, integer version of the log-odds occupancy mapping used
in SLAM systems. Moving objects are NOT written into this long-term map;
they live in a separate `dynamic` layer that is rebuilt every frame
(sensor fusion: the camera says "this blob is moving", so LiDAR hits on it are
ignored for mapping).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from src.config import PerceptionConfig
from src.simulation.sensors import CameraFrame, LidarScan


class OccupancyGrid:
    def __init__(self, width: float, height: float, resolution: float, cfg: PerceptionConfig) -> None:
        self.cfg = cfg
        self.resolution = resolution
        self.cols = int(math.ceil(width / resolution))
        self.rows = int(math.ceil(height / resolution))
        self.evidence = np.zeros((self.rows, self.cols), np.int16)
        self.observed = np.zeros((self.rows, self.cols), bool)
        self.dynamic = np.zeros((self.rows, self.cols), bool)

    # ---------------------------------------------------------------- helpers
    def world_to_cell(self, x: float, y: float) -> tuple:
        r = int(np.clip(y / self.resolution, 0, self.rows - 1))
        c = int(np.clip(x / self.resolution, 0, self.cols - 1))
        return r, c

    def cell_to_world(self, r: int, c: int) -> np.ndarray:
        return np.array([(c + 0.5) * self.resolution, (r + 0.5) * self.resolution])

    def _flat_indices(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        cols = np.floor(xs / self.resolution).astype(np.int64)
        rows = np.floor(ys / self.resolution).astype(np.int64)
        ok = (cols >= 0) & (rows >= 0) & (cols < self.cols) & (rows < self.rows)
        return np.unique(rows[ok] * self.cols + cols[ok])

    def _apply(self, free_idx: np.ndarray, hit_idx: np.ndarray, hit_weight: int) -> None:
        ev = self.evidence.reshape(-1)       # view, edits go into self.evidence
        seen = self.observed.reshape(-1)
        free_idx = np.setdiff1d(free_idx, hit_idx, assume_unique=True)
        ev[free_idx] -= self.cfg.miss_evidence
        ev[hit_idx] += hit_weight
        np.clip(self.evidence, -self.cfg.max_evidence, self.cfg.max_evidence, out=self.evidence)
        seen[free_idx] = True
        seen[hit_idx] = True

    # ---------------------------------------------------------------- updates
    def update_from_lidar(self, scan: LidarScan, ignore_circles=()) -> None:
        """Ray-trace free space along every beam and mark the hit cells."""
        step = self.resolution * 0.5
        t = np.arange(0.0, scan.max_range, step)
        free_len = np.where(scan.hits, scan.ranges - self.resolution, scan.ranges)
        valid = t[None, :] < free_len[:, None]
        xs = scan.origin[0] + np.cos(scan.angles)[:, None] * t[None, :]
        ys = scan.origin[1] + np.sin(scan.angles)[:, None] * t[None, :]
        free_idx = self._flat_indices(xs[valid], ys[valid])

        hits = scan.hit_points()
        if len(hits) and ignore_circles:
            keep = np.ones(len(hits), bool)
            for center, radius in ignore_circles:
                keep &= np.linalg.norm(hits - center, axis=1) > radius
            hits = hits[keep]
        hit_idx = self._flat_indices(hits[:, 0], hits[:, 1]) if len(hits) else np.array([], np.int64)
        self._apply(free_idx, hit_idx, self.cfg.hit_evidence)

    def update_from_camera(self, frame: CameraFrame, static_mask: np.ndarray) -> None:
        """Project the camera's static-obstacle mask into the grid."""
        res = self.resolution
        x0, y0 = frame.origin
        c0 = max(0, int(math.ceil(x0 / res)))
        r0 = max(0, int(math.ceil(y0 / res)))
        c1 = min(self.cols, int((x0 + frame.view_m) / res))
        r1 = min(self.rows, int((y0 + frame.view_m) / res))
        if c1 <= c0 or r1 <= r0:
            return

        # Max-pool the mask over one grid cell so thin obstacles are not missed.
        k = max(1, int(round(res / frame.m_per_px)))
        pooled = cv2.dilate(static_mask, np.ones((k, k), np.uint8))

        cc, rr = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        px = (((cc + 0.5) * res - x0) / frame.m_per_px).astype(np.int64)
        py = (((rr + 0.5) * res - y0) / frame.m_per_px).astype(np.int64)
        n = pooled.shape[0]
        ok = (px >= 0) & (py >= 0) & (px < n) & (py < n)
        occ = np.zeros_like(ok)
        occ[ok] = pooled[py[ok], px[ok]] > 0

        flat = (rr * self.cols + cc)
        hit_idx = flat[ok & occ]
        free_idx = flat[ok & ~occ]
        # The camera is trusted a little less than the LiDAR.
        self._apply(free_idx, hit_idx, max(1, self.cfg.hit_evidence - 1))

    def set_dynamic(self, objects, inflation: float, horizon_s: float = 0.0) -> None:
        """Rebuild the short-lived layer of moving obstacles.

        Each object blocks a disc around its current position and, if it has a
        velocity estimate, the whole "capsule" it will sweep in `horizon_s` seconds.
        """
        layer = np.zeros((self.rows, self.cols), np.uint8)
        for obj in objects:
            r, c = self.world_to_cell(*obj.center)
            radius = int(math.ceil((obj.radius + inflation) / self.resolution))
            cv2.circle(layer, (c, r), radius, 1, thickness=-1)
            velocity = getattr(obj, "velocity", None)
            if velocity is not None and horizon_s > 0 and np.linalg.norm(velocity) > 0.2:
                fr, fc = self.world_to_cell(*(obj.center + velocity * horizon_s))
                cv2.line(layer, (c, r), (fc, fr), 1, thickness=2 * radius)
                cv2.circle(layer, (fc, fr), radius, 1, thickness=-1)
        self.dynamic = layer.astype(bool)

    # ---------------------------------------------------------------- outputs
    def static_occupied(self) -> np.ndarray:
        return self.evidence >= self.cfg.occupied_threshold

    def explored_fraction(self) -> float:
        return float(self.observed.mean())
