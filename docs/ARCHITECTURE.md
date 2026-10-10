# System Architecture

This document explains **how the autonomy stack is built**: what each module does, what data flows between them, and why each design decision was made.

---

## 1. Big picture: Sense → Think → Act

Every autonomous machine (a Tesla, a Waymo taxi, an Amazon warehouse robot, a DJI drone) runs the same loop many times per second:

| Stage | Question it answers | In this project |
|---|---|---|
| **Sense** | What do my sensors see? | 2D LiDAR + bird's-eye-view (BEV) camera |
| **Perceive** | What objects are around me and where are they going? | OpenCV detector, multi-object tracker, occupancy grid |
| **Plan** | How do I get to the goal safely? | Costmap + A* / Dijkstra / Weighted A* + path smoothing |
| **Decide** | Is it safe to keep going? Do I need a new plan? | Navigator state machine + safety corridor |
| **Act** | What steering and throttle do I apply? | Pure Pursuit + speed profile |

The loop runs at **10 Hz** (`dt = 0.1 s`).

---

## 2. Block diagram

```
                ┌───────────────────────────── SIMULATION (ground truth) ─────────────────────────────┐
                │  World: map 80x50 m, static obstacles, moving pedestrians/forklifts (waypoint agents) │
                │  Robot: kinematic bicycle model (x, y, θ, v, δ)                                       │
                └───────────────┬───────────────────────────────────────────────▲─────────────────────┘
                                │ true world state                              │ accel, steer
                    ┌───────────▼───────────┐                                   │
                    │        SENSORS        │                                   │
                    │  LiDAR: 120 rays,12 m │                                   │
                    │  BEV camera: 24x24 m  │                                   │
                    │  (both with noise)    │                                   │
                    └─────┬───────────┬─────┘                                   │
              LidarScan   │           │  CameraFrame (240x240 BGR)              │
                          │   ┌───────▼────────────────┐                        │
                          │   │  OBSTACLE DETECTOR     │  blur → HSV → threshold│
                          │   │  (OpenCV)              │  → morphology → contours
                          │   └───────┬───────┬────────┘                        │
                          │   static  │       │ dynamic detections              │
                          │   mask    │   ┌───▼──────────────┐                  │
                          │           │   │ MULTI-OBJECT     │ ID, velocity,    │
                          │           │   │ TRACKER          │ 2 s prediction   │
                          │           │   └───┬──────────────┘                  │
                    ┌─────▼───────────▼───────▼────┐                            │
                    │  OCCUPANCY GRID (0.25 m)     │  sensor fusion:            │
                    │  static evidence layer       │  LiDAR + camera evidence,  │
                    │  + dynamic (predicted) layer │  LiDAR hits on tracked     │
                    └─────────────┬────────────────┘  people are NOT mapped     │
                                  │                                             │
                    ┌─────────────▼────────────────┐                            │
                    │  COSTMAP                     │  inflate by robot radius,  │
                    │  lethal + soft cost gradient │  soft cost near obstacles  │
                    └─────────────┬────────────────┘                            │
                                  │                                             │
   ┌──────────────────────────────▼─────────────────────────────────┐           │
   │  NAVIGATOR  (finite-state machine)                              │           │
   │   • replan if: no path / path blocked / stuck / every 1.5 s     │           │
   │   • PLANNER: A* (or Dijkstra / Weighted A*) → shortcut → resample│           │
   │   • SAFETY: LiDAR hits inside the swept corridor of the path?   │           │
   │       → EMERGENCY_STOP, speed limit, caution near people        │           │
   │   • RECOVERY: reverse with counter-steer when pinned            │           │
   └──────────────────────────────┬─────────────────────────────────┘           │
                                  │ path + speed limit                          │
                    ┌─────────────▼────────────────┐                            │
                    │  CONTROLLER: Pure Pursuit     │───────────────────────────┘
                    │  + curvature speed profile    │
                    └──────────────────────────────┘
                                  │
                    ┌─────────────▼────────────────┐
                    │  VISUALIZATION & LOGGING      │  dashboard, MP4, GIF, PNG,
                    │                              │  metrics.json, telemetry.csv
                    └──────────────────────────────┘
```

---

## 3. Module by module

### 3.1 Simulation, `src/simulation/`
| File | Responsibility |
|---|---|
| `world.py` | Ground-truth map as a 0.1 m raster. Static obstacles (rectangles, circles) and `DynamicObstacle` agents that walk between waypoints and yield to the robot like real people do. Answers geometric queries: occupancy, collision, clearance. |
| `robot.py` | Kinematic **bicycle model**, the standard low-speed car model. Enforces speed, acceleration and steering limits. |
| `sensors.py` | **LiDAR:** vectorised ray casting with Gaussian range noise. **BEV camera:** top-down colour crop around the robot with pixel noise. |
| `scenarios.py` | Four test worlds: warehouse, city, crosswalk, random (seeded and guaranteed solvable). |
| `simulator.py` | Main loop. Also acts as the **referee**: it checks collisions against the ground truth, which the robot never sees. |

> **Key design rule:** the navigation code never touches `World`. It only receives `LidarScan` and `CameraFrame`. This is what makes the simulation an honest test of autonomy.

### 3.2 Perception, `src/perception/`
| File | Responsibility |
|---|---|
| `obstacle_detector.py` | Classical computer vision: Gaussian blur → HSV → colour thresholds → morphological open/close → contours → `Detection(label, center, radius, bbox, confidence)`. Same interface a YOLO detector would have. |
| `tracker.py` | Greedy nearest-neighbour data association plus an exponential-moving-average velocity estimate. Each moving object gets a persistent **track ID**, a **velocity**, and a **constant-velocity prediction**. |
| `occupancy_grid.py` | Integer evidence map, a simplified log-odds occupancy mapping. LiDAR traces free space along each beam and adds evidence at hits. The camera adds evidence from its static mask. Moving objects go into a separate **dynamic layer** that also covers the space they will occupy in the next 2 s. |

### 3.3 Planning, `src/planning/`
| File | Responsibility |
|---|---|
| `costmap.py` | Dilates obstacles by `robot_radius + safety_margin` (lethal), then adds a soft cost gradient using a distance transform, so paths stay in the middle of aisles. The same concept as ROS `costmap_2d`. |
| `astar.py` | One best-first search covering **Dijkstra (w=0)**, **A\* (w=1)** and **Weighted A\* (w>1)**. 8-connected, octile heuristic, no corner cutting. It can also block the half-disc behind the car, because a car cannot turn on the spot. |
| `smoothing.py` | Line-of-sight shortcutting ("string pulling") through safe cells, then resampling to 0.5 m waypoints. |

### 3.4 Navigation (decision making), `src/navigation/navigator.py`
```
PLANNING ──path found──▶ FOLLOWING ──obstacle in corridor──▶ EMERGENCY_STOP
    ▲                        │  ▲                                  │
    │                        │  └──────── corridor clear ──────────┘
    └──── path blocked ──────┘
EMERGENCY_STOP ──static blocker & replan didn't help──▶ RECOVERY (reverse) ──▶ PLANNING
PLANNING ──no path for 5 s and people nearby──▶ RECOVERY (give way)
FOLLOWING ──within 1.2 m of goal──▶ GOAL_REACHED
150 consecutive failed plans ──▶ FAILED
```

**Replanning triggers:**
1. `initial` / `retry`: there is no path.
2. `path_blocked`: an upcoming waypoint is now inside an inflated obstacle (a newly discovered wall or a person).
3. `stuck`: stopped for 2 s.
4. `periodic`: every 1.5 s. The new path is accepted only if it is at least 15 % cheaper (**hysteresis**, which prevents route flip-flopping).

**Safety layer, independent of the planner:** raw LiDAR hits are tested against a **1.05 m-wide corridor swept along the next ~8 m of the path**. If one lies closer than `stop_distance + v²/(2·a_max)`, the robot performs an emergency stop. Between the stop and slow-down distances, speed is scaled down linearly.

### 3.5 Control, `src/control/pure_pursuit.py`
* **Steering:** `δ = atan(2·L·sin α / Ld)`, with lookahead `Ld = 2.0 + 0.5·v` (clipped to 6 m).
* **Speed = min of:** max speed, the safety limit from the navigator, the current-steering limit, **look-ahead curvature limit** `√(a_lat/κ)` (propagated backwards with a braking curve so the robot slows *before* a corner), and a goal-arrival braking curve.

### 3.6 Visualization, `src/visualization/`
* `dashboard.py`: a 1280×720 live view showing ground truth, the belief costmap overlay, LiDAR rays, the plan, the trajectory, tracked objects with velocity arrows, the annotated camera image, the occupancy belief, telemetry and a speed plot.
* `plots.py`: trajectory and telemetry figures with Matplotlib.

---

## 4. Data flow per control step

| # | Producer → Consumer | Data |
|---|---|---|
| 1 | `World.step` | moves people/forklifts (they yield to the robot) |
| 2 | `Lidar.scan` → navigator | `LidarScan(origin, angles[120], ranges[120], hits[120])` |
| 3 | `BEVCamera.capture` → detector | `CameraFrame(image 240×240×3, origin, 0.1 m/px)` |
| 4 | detector → tracker, grid | `DetectionResult(detections, static_mask, dynamic_mask)` |
| 5 | tracker → grid, navigator | `Track(id, center, radius, velocity)` |
| 6 | grid → costmap | `static_occupied` (bool 200×320) + `dynamic` (bool 200×320) |
| 7 | costmap → planner | `Costmap(occupied, lethal, cost)` |
| 8 | planner → navigator | `PlanResult(cells, cost, expanded, time_ms)` → path (N×2, metres) |
| 9 | navigator → controller | path + `speed_limit` |
| 10 | controller → robot | `accel`, `steer` |
| 11 | simulator | collision check, telemetry row, dashboard frame |

---

## 5. Coordinate conventions
* World units are **metres**. `x` points right and `y` points **down** (image convention), so every raster is a plain NumPy array indexed `[row=y, col=x]`.
* Heading `θ = 0` faces +x. Angles are in radians internally and degrees only in the UI.

## 6. Why these choices (interview-ready)
| Decision | Reason |
|---|---|
| Custom 2D sim instead of CARLA/Gazebo | Runs on any laptop with no GPU, starts in seconds, and is fully deterministic and testable. Upgrading later is easy because the modules talk through small data classes. |
| Classical CV instead of YOLO | The colour-coded BEV makes classical CV accurate and very fast (about 1 ms on CPU). The `Detection` interface is detector-agnostic, so YOLO can be dropped in (see ROADMAP). |
| Occupancy grid + A* | This is the industry baseline (ROS nav stack). It is explainable, provably optimal on the grid, and easy to benchmark. |
| Pure pursuit | Robust, needs one tunable, and is widely used on real AMRs and AV prototypes. |
| Separate safety corridor | Defence in depth: even if planning is late or wrong, raw LiDAR can still stop the robot. |
