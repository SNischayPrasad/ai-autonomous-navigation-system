"""Ground-truth 2D world: map size, static obstacles and moving obstacles.

Coordinate convention (same as an image): x grows to the right, y grows
DOWNWARD, units are metres. A world point (x, y) maps to raster cell
(row = y / resolution, col = x / resolution).

The navigation stack never reads this module directly - it only sees the
world through the simulated sensors (see sensors.py). That separation is what
makes the simulation honest.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# BGR colour palette shared by the world renderer and the BEV camera.
# The perception module recognises obstacles by these colours.
COLOR_GROUND = (58, 58, 62)      # dark asphalt / warehouse floor
COLOR_STATIC = (180, 140, 60)    # steel-blue: walls, shelves, buildings, parked cars
COLOR_DYNAMIC = (50, 50, 230)    # red: pedestrians, forklifts, cyclists


@dataclass
class RectObstacle:
    """Axis-aligned box. (x, y) is the top-left corner in metres."""
    x: float
    y: float
    w: float
    h: float

    def draw(self, img: np.ndarray, scale: float, value) -> None:
        p1 = (int(round(self.x * scale)), int(round(self.y * scale)))
        p2 = (int(round((self.x + self.w) * scale)) - 1, int(round((self.y + self.h) * scale)) - 1)
        cv2.rectangle(img, p1, p2, value, thickness=-1)


@dataclass
class CircleObstacle:
    cx: float
    cy: float
    r: float

    def draw(self, img: np.ndarray, scale: float, value) -> None:
        center = (int(round(self.cx * scale)), int(round(self.cy * scale)))
        cv2.circle(img, center, max(1, int(round(self.r * scale))), value, thickness=-1)


@dataclass
class DynamicObstacle:
    """A moving agent that walks back and forth through a list of waypoints.

    Like a real pedestrian it yields: it will not step closer to the robot
    once it is already within `yield_gap` of the robot's body.
    """
    waypoints: list
    radius: float
    speed: float
    label: str = "pedestrian"
    yield_gap: float = 0.5
    x: float = field(init=False)
    y: float = field(init=False)
    _target: int = field(init=False)

    def __post_init__(self) -> None:
        self.x, self.y = self.waypoints[0]
        self._target = 1 % len(self.waypoints)

    def step(self, dt: float, robot_xy=None, robot_radius: float = 0.0) -> None:
        if len(self.waypoints) < 2:
            return
        tx, ty = self.waypoints[self._target]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        move = self.speed * dt
        if dist <= move:
            nx, ny = tx, ty
            self._target = (self._target + 1) % len(self.waypoints)
        else:
            nx, ny = self.x + dx / dist * move, self.y + dy / dist * move

        if robot_xy is not None:
            keep_out = self.radius + robot_radius + self.yield_gap
            d_now = math.hypot(self.x - robot_xy[0], self.y - robot_xy[1])
            d_next = math.hypot(nx - robot_xy[0], ny - robot_xy[1])
            if d_next < keep_out and d_next < d_now:
                return  # wait for the robot instead of walking into it
        self.x, self.y = nx, ny

    def draw(self, img: np.ndarray, scale: float, value) -> None:
        center = (int(round(self.x * scale)), int(round(self.y * scale)))
        cv2.circle(img, center, max(1, int(round(self.radius * scale))), value, thickness=-1)


class World:
    """Holds the true state of the environment and answers geometric queries."""

    def __init__(self, name: str, width: float, height: float,
                 static_obstacles: list, dynamic_obstacles: list,
                 start: tuple, goal: tuple, resolution: float = 0.1,
                 description: str = "") -> None:
        self.name = name
        self.description = description
        self.width = width
        self.height = height
        self.resolution = resolution
        self.scale = 1.0 / resolution
        self.cols = int(round(width / resolution))
        self.rows = int(round(height / resolution))
        self.static_obstacles = static_obstacles
        self.dynamic_obstacles = dynamic_obstacles
        self.start = start            # (x, y, heading_rad)
        self.goal = goal              # (x, y)
        self.time = 0.0

        self.static_mask = np.zeros((self.rows, self.cols), np.uint8)
        for obs in static_obstacles:
            obs.draw(self.static_mask, self.scale, 1)
        self.static_mask = self.static_mask.astype(bool)

        self._static_image = np.empty((self.rows, self.cols, 3), np.uint8)
        self._static_image[:] = COLOR_GROUND
        self._static_image[self.static_mask] = COLOR_STATIC
        self._refresh_dynamic()

    # ------------------------------------------------------------------ update
    def _refresh_dynamic(self) -> None:
        dyn = np.zeros((self.rows, self.cols), np.uint8)
        for obs in self.dynamic_obstacles:
            obs.draw(dyn, self.scale, 1)
        self.dynamic_mask = dyn.astype(bool)
        self.occupancy = self.static_mask | self.dynamic_mask
        self._distance = None  # lazily recomputed clearance field

    def step(self, dt: float, robot_xy=None, robot_radius: float = 0.0) -> None:
        """Advance all moving obstacles by one time step."""
        for obs in self.dynamic_obstacles:
            obs.step(dt, robot_xy, robot_radius)
        self.time += dt
        self._refresh_dynamic()

    # ----------------------------------------------------------------- queries
    def is_occupied(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Vectorised occupancy lookup. Points outside the map count as occupied."""
        cols = np.floor(np.asarray(xs) / self.resolution).astype(np.int64)
        rows = np.floor(np.asarray(ys) / self.resolution).astype(np.int64)
        outside = (cols < 0) | (rows < 0) | (cols >= self.cols) | (rows >= self.rows)
        cols = np.clip(cols, 0, self.cols - 1)
        rows = np.clip(rows, 0, self.rows - 1)
        return self.occupancy[rows, cols] | outside

    def check_collision(self, x: float, y: float, radius: float) -> bool:
        """True if a disc of `radius` at (x, y) overlaps any obstacle."""
        res = self.resolution
        c0, c1 = int((x - radius) / res), int((x + radius) / res) + 1
        r0, r1 = int((y - radius) / res), int((y + radius) / res) + 1
        if c0 < 0 or r0 < 0 or c1 > self.cols or r1 > self.rows:
            return True
        yy, xx = np.mgrid[r0:r1, c0:c1]
        inside = ((xx + 0.5) * res - x) ** 2 + ((yy + 0.5) * res - y) ** 2 <= radius ** 2
        return bool(np.any(self.occupancy[r0:r1, c0:c1] & inside))

    def clearance(self, x: float, y: float) -> float:
        """Distance [m] from (x, y) to the nearest obstacle surface."""
        if self._distance is None:
            free = (~self.occupancy).astype(np.uint8)
            self._distance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.resolution
        c = int(np.clip(x / self.resolution, 0, self.cols - 1))
        r = int(np.clip(y / self.resolution, 0, self.rows - 1))
        return float(self._distance[r, c])

    def static_grid(self, resolution: float) -> np.ndarray:
        """Static obstacles downsampled to another resolution (any overlap = occupied)."""
        cols = int(math.ceil(self.width / resolution))
        rows = int(math.ceil(self.height / resolution))
        small = cv2.resize(self.static_mask.astype(np.float32), (cols, rows), interpolation=cv2.INTER_AREA)
        return small > 0.0

    # --------------------------------------------------------------- drawing
    def render(self) -> np.ndarray:
        """Top-down BGR image of the true world at the raster resolution."""
        img = self._static_image.copy()
        img[self.dynamic_mask] = COLOR_DYNAMIC
        return img
