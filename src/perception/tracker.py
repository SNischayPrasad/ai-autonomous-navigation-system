"""Multi-object tracking + motion prediction for moving obstacles.

A detector only says "there is a person at (x, y) right now". To avoid people
smoothly the robot also needs to know where they are GOING. This tracker:

  1. associates each new detection with an existing track (greedy nearest
     neighbour, gated by `max_match_dist`) - the same idea as SORT-style
     trackers used with YOLO in industry,
  2. estimates velocity with an exponential moving average to suppress noise,
  3. keeps a track alive for a few frames if a detection is missed,
  4. predicts future positions with a constant-velocity model, which the
     costmap uses to block the space a person is about to walk into.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Track:
    track_id: int
    center: np.ndarray
    radius: float
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    age: int = 1
    missed: int = 0

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))

    def predict(self, horizon_s: float) -> np.ndarray:
        return self.center + self.velocity * horizon_s


class MultiObjectTracker:
    def __init__(self, dt: float, max_match_dist: float = 1.5, smoothing: float = 0.3,
                 max_missed: int = 5) -> None:
        self.dt = dt
        self.max_match_dist = max_match_dist
        self.alpha = smoothing
        self.max_missed = max_missed
        self.tracks: list = []
        self._next_id = 1

    def update(self, detections: list) -> list:
        """Feed one frame of dynamic detections, get back the confirmed tracks."""
        # Candidate pairs sorted by distance -> greedy one-to-one assignment.
        pairs = sorted(
            (float(np.linalg.norm(t.center - d.center)), ti, di)
            for ti, t in enumerate(self.tracks) for di, d in enumerate(detections)
        )
        used_t, used_d = set(), set()
        for dist, ti, di in pairs:
            if dist > self.max_match_dist or ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t, d = self.tracks[ti], detections[di]
            measured_v = (d.center - t.center) / self.dt
            t.velocity = self.alpha * measured_v + (1 - self.alpha) * t.velocity
            t.center, t.radius = d.center.copy(), d.radius
            t.age += 1
            t.missed = 0

        for ti, t in enumerate(self.tracks):
            if ti not in used_t:              # not seen this frame: coast on the motion model
                t.missed += 1
                t.center = t.predict(self.dt)
        for di, d in enumerate(detections):
            if di not in used_d:
                self.tracks.append(Track(self._next_id, d.center.copy(), d.radius))
                self._next_id += 1

        self.tracks = [t for t in self.tracks if t.missed <= self.max_missed]
        return [t for t in self.tracks if t.missed == 0]
