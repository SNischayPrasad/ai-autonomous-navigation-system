"""Costmap: turns the occupancy grid into something a planner can search.

Two layers, the same idea as the ROS navigation stack's costmap_2d:

* lethal  - obstacles grown ("inflated") by robot radius + safety margin.
            Planning the robot's CENTRE point on this map guarantees the whole
            body stays clear, so the planner can treat the robot as a point.
* cost    - a soft gradient near obstacles, so the cheapest path runs down the
            middle of an aisle instead of scraping along a shelf.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Costmap:
    occupied: np.ndarray     # raw obstacles (static belief + dynamic layer)
    lethal: np.ndarray       # inflated obstacles - never enter
    cost: np.ndarray         # float traversal cost, >= 1.0
    resolution: float
    inflation_cells: int


def build_costmap(static_occ: np.ndarray, dynamic_occ: np.ndarray, resolution: float,
                  robot_radius: float, safety_margin: float,
                  soft_distance: float, soft_weight: float) -> Costmap:
    occupied = static_occ | dynamic_occ
    r = int(math.ceil((robot_radius + safety_margin) / resolution))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    lethal = cv2.dilate(occupied.astype(np.uint8), kernel) > 0

    # Distance (in metres) from every free cell to the nearest lethal cell.
    dist = cv2.distanceTransform((~lethal).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    proximity = np.clip(1.0 - dist / max(soft_distance, 1e-6), 0.0, 1.0)
    cost = (1.0 + soft_weight * proximity).astype(np.float32)
    return Costmap(occupied, lethal, cost, resolution, r)
