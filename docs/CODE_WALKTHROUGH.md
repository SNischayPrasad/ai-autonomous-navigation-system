# Code Walkthrough: File by File

Read the files in this order. Each one is short (50–300 lines) and heavily commented.

| Order | File | Lines of thought to follow |
|---|---|---|
| 1 | `src/config.py` | Every tunable parameter lives in one place. |
| 2 | `src/simulation/world.py` | How the true world is represented. |
| 3 | `src/simulation/robot.py` | How the car moves. |
| 4 | `src/simulation/sensors.py` | How the robot observes the world. |
| 5 | `src/perception/obstacle_detector.py` | Pixels → objects. |
| 6 | `src/perception/tracker.py` | Objects → tracks with velocity. |
| 7 | `src/perception/occupancy_grid.py` | Observations → map. |
| 8 | `src/planning/costmap.py` | Map → safe/unsafe/expensive. |
| 9 | `src/planning/astar.py` | Costmap → path. |
| 10 | `src/planning/smoothing.py` | Zig-zag path → smooth waypoints. |
| 11 | `src/control/pure_pursuit.py` | Waypoints → steering + speed. |
| 12 | `src/navigation/navigator.py` | The brain that connects everything. |
| 13 | `src/simulation/simulator.py` | The main loop + referee + output files. |
| 14 | `src/visualization/dashboard.py`, `plots.py` | What you see. |
| 15 | `main.py`, `scripts/*.py`, `tests/*.py` | How to run, benchmark and verify. |

---

### `main.py`: command-line entry point
Parses `--scenario`, `--planner`, `--seed`, `--headless`, `--gif`, `--no-video`, `--max-steps`. It builds the scenario, runs `Simulation`, and prints a summary table. It returns exit code 0 only if every run succeeded, which makes it usable in CI.

### `src/config.py`: configuration
Six dataclasses (`SimConfig`, `RobotConfig`, `SensorConfig`, `PerceptionConfig`, `PlannerConfig`, `ControlConfig`). **Why dataclasses?** They give type hints, defaults and autocomplete, and an experiment can override fields in code (`cfg.planner.algorithm = "dijkstra"`).

### `src/simulation/world.py`: ground truth
- `RectObstacle`, `CircleObstacle` draw themselves into a raster with `cv2.rectangle`/`cv2.circle`.
- `DynamicObstacle.step()` moves along waypoints and **yields**: if the next step brings it closer to the robot than `radius + robot_radius + 0.5`, it waits.
- `World.is_occupied(xs, ys)` is a vectorised lookup used by the LiDAR. Out of bounds counts as occupied.
- `World.check_collision()` tests a disc against the raster and is used by the referee.
- `World.clearance()` uses a distance transform to measure how close the robot came to obstacles.

### `src/simulation/robot.py`: bicycle model
`step(accel, steer, dt)` clamps the inputs to the physical limits, then integrates `x' = v cos θ`, `y' = v sin θ`, `θ' = v/L · tan δ`. Reverse is allowed down to −1 m/s (recovery only).

### `src/simulation/sensors.py`: LiDAR + camera
- **LiDAR:** all 120 rays × ~80 sample points are checked in **one NumPy call** (`world.is_occupied` on a 2D array). `argmax` finds the first hit per ray. Gaussian noise is then added.
- **Camera:** crops the rendered world around the robot (padding outside the map with wall colour), resizes it to 240×240 and adds pixel noise.

### `src/perception/obstacle_detector.py`: computer vision
```
GaussianBlur(5x5) → cvtColor(BGR→HSV) → inRange(static) / inRange(red, two hue bands)
→ morphologyEx(OPEN, CLOSE) → findContours → area filter → minEnclosingCircle → world coords
```
Red needs **two hue ranges** (0–10 and 170–179) because hue wraps around. `annotate()` draws the boxes shown on the dashboard.

### `src/perception/tracker.py`: multi-object tracking
1. Build all (track, detection) distance pairs and sort them.
2. Greedily match the closest pairs within 1.5 m (one-to-one).
3. Matched: `velocity = 0.3·measured + 0.7·previous` (an EMA filter).
4. Unmatched tracks coast with their velocity for up to 5 frames. Unmatched detections start new tracks.
5. `predict(t)` = `center + velocity·t` (constant-velocity model).

### `src/perception/occupancy_grid.py`: mapping and sensor fusion
- `update_from_lidar`: samples points every 0.125 m along each beam up to the hit, marks them **free** (−1), and marks hit cells **occupied** (+3). Hits that fall on tracked moving objects are skipped.
- `update_from_camera`: max-pools the camera's static mask to the grid resolution, then adds +2 or −1 evidence.
- Evidence is clipped to ±10, so the map can still change its mind (a door opens, a pallet is removed).
- `set_dynamic`: draws each tracked object **plus a capsule along its predicted path** (`cv2.line` with thickness).

### `src/planning/costmap.py`
`cv2.dilate` with an elliptical kernel = obstacle inflation. `cv2.distanceTransform` gives the distance from every cell to the nearest lethal cell, which becomes `cost = 1 + 5·max(0, 1 − d/2 m)`.

### `src/planning/astar.py`
- `HEURISTIC_WEIGHTS = {"dijkstra": 0, "astar": 1}`, and `weighted_astar` uses `w = 2`.
- **Octile distance** is the exact cost of an 8-connected move on an empty grid, so it never over-estimates and A* stays optimal.
- **Performance trick:** the NumPy arrays are converted to flat Python lists, because indexing NumPy scalars in a Python loop is about 10× slower.
- **Start-escape:** if the robot sits inside the inflation zone, cells around it are temporarily allowed (but never truly occupied cells).
- `extra_blocked`: the navigator passes the half-disc **behind** the car so the first choice of path leaves forwards.

### `src/planning/smoothing.py`
`shortcut_path` keeps extending a straight segment while `line_is_free` holds (it checks cells kept away from walls by half the soft-cost distance). `resample` interpolates by arc length.

### `src/control/pure_pursuit.py`
- Finds the nearest path point, then the first point at least `Ld` ahead.
- `steer = atan2(2·L·sin α, Ld)`.
- `curve_speed_limit()`: computes curvature from heading changes, takes the comfortable speed `√(1.5/κ)`, then `√(v² + 2·a·s)` so braking starts early enough.

### `src/navigation/navigator.py`: the brain
`update()` runs **perceive → map → decide → plan → safety → act** and returns `(accel, steer, NavDebug)`. `NavDebug` carries everything the dashboard needs. The important helpers are:
- `_replan_reason`: the four replanning triggers.
- `_plan`: heading-aware A* with fallback, smoothing, and **hysteresis** for periodic replans.
- `_corridor_obstacle_distance`: distance along the path to the first LiDAR hit within 1.05 m of the path.
- `_recover`: reverse at 0.8 m/s with opposite steering until 2.5 s pass or something is behind.
- `summary()`: the metrics reported at the end.

### `src/simulation/simulator.py`
The main loop. It also measures **collisions** and **minimum clearance** against the ground truth, writes `run.mp4`, `demo.gif`, snapshots, `metrics.json`, `telemetry.csv`, and calls the plotting functions.

### `src/visualization/dashboard.py` and `plots.py`
Pure drawing code: OpenCV for the live 1280×720 frame, Matplotlib (Agg backend, no window) for report figures.

### `scripts/benchmark.py`
Part 1 plans on fully known maps with all 3 algorithms (time, nodes, length). Part 2 runs the full autonomous loop on 11 maps × 3 planners. It writes CSVs, a bar chart and `summary.md`.

### `scripts/export_assets.py`
Runs the demo scenarios with recording and copies the best frames to `assets/` with clean names for the README.

### `tests/`
| File | What it proves |
|---|---|
| `test_planning.py` | Straight path, path through a door, no path when blocked, A* expands fewer nodes than Dijkstra, A* is optimal, smoothing, resampling |
| `test_perception.py` | Detector localises a pedestrian within 0.3 m, LiDAR range accuracy, mapping of walls/free space, moving objects not mapped, tracker velocity |
| `test_control.py` | Vehicle model straight-line motion and limits, controller steers towards the path, slows before corners |
| `test_integration.py` | Full stack reaches the goal with 0 collisions (warehouse, crosswalk) |
