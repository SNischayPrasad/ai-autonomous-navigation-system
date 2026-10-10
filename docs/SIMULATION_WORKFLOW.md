# Virtual Simulation Workflow

How the whole project runs **without any hardware**, step by step.

---

## 1. Which simulator, and why?

This project uses a **custom 2D simulator written in Python (NumPy + OpenCV)**, included in `src/simulation/`.

| Criterion | Custom 2D sim (chosen) | CARLA | Gazebo / ROS 2 | Webots |
|---|---|---|---|---|
| Install | `pip install` (1 min) | 20 GB download, GPU | Linux + ROS 2 setup | 500 MB installer |
| GPU | Not needed | Required | Recommended | Recommended |
| Windows support | Yes | Partial | Poor (use WSL) | Yes |
| Deterministic and testable with pytest | Yes | Hard | Hard | Medium |
| Speed | ~30 sim-seconds/second with rendering | Real-time | Real-time | Real-time |
| Learning value | You implement **every** algorithm yourself | Mostly sensor APIs | Mostly ROS plumbing | Mostly configuration |

The modules exchange plain data classes (`LidarScan`, `CameraFrame`, `Detection`, `Costmap`), so the same planner and controller can later be connected to CARLA or Gazebo (see [ROADMAP.md](ROADMAP.md)).

## 2. Install and launch
```bash
python -m venv .venv
.venv\Scripts\activate            # Windows   |   source .venv/bin/activate  (Mac/Linux)
pip install -r requirements.txt
python main.py                    # opens the live simulation window (warehouse)
```
Window keys: **q** quits, **p** pauses (press any key to continue).

## 3. How the virtual world is built

### 3.1 The environment / map
`src/simulation/scenarios.py` builds a `World` that is **80 m × 50 m**, stored as a 0.1 m raster (800 × 500 cells).
```python
static = _border()                                   # outer walls
static.append(RectObstacle(x=12, y=6, w=3, h=15))    # a shelf (metres)
static.append(CircleObstacle(cx=72, cy=42, r=1.2))   # a pillar
```

### 3.2 The virtual robot / car agent
`src/simulation/robot.py`: a car-like robot using the **kinematic bicycle model**.

| Parameter | Value |
|---|---|
| Wheelbase | 1.6 m |
| Body | 2.4 m × 1.4 m (collision radius 0.9 m) |
| Max speed | 3.0 m/s forward, 1.0 m/s reverse |
| Max steering | 35° → minimum turning radius ≈ 2.3 m |

### 3.3 Placing obstacles
- **Static:** `RectObstacle(x, y, w, h)` and `CircleObstacle(cx, cy, r)`.
- **Moving:** `DynamicObstacle(waypoints=[(x1, y1), (x2, y2)], radius=0.5, speed=1.0)`. The agent walks back and forth and *yields* if it would walk into the robot.

### 3.4 How the AI reads the environment
The navigation code **never sees the `World` object**. It only receives:
1. **LiDAR** (`Lidar.scan`): 120 rays over 360°, 12 m range, ±3 cm noise. This is implemented by marching along each ray in the raster (vectorised NumPy).
2. **BEV camera** (`BEVCamera.capture`): a 240×240 px top-down colour image of the 24 m × 24 m area around the robot, with pixel noise. Walls and shelves are steel-blue, and people and forklifts are red.

The robot starts with an **empty map** and must discover every obstacle itself.

### 3.5 How path planning is applied
1. Camera and LiDAR evidence goes into the **occupancy grid** (0.25 m cells).
2. Obstacles are **inflated** by 1.2 m (robot radius + margin), and a soft cost is added near walls.
3. **A\*** searches the 320×200 grid (~2–10 ms).
4. The path is **shortcut** and **resampled** every 0.5 m.
5. The robot **replans** when a newly discovered obstacle blocks the path, or every 1.5 s if a much better route appears.

### 3.6 How collision avoidance works (3 layers)
| Layer | Mechanism | Handles |
|---|---|---|
| 1. Planning | Inflated costmap + **predicted** pedestrian positions (2 s ahead) | Routing around obstacles and people's future paths |
| 2. Speed | Slow to 2 m/s near tracked people; slow before curves | Reaction time, comfort |
| 3. Safety | Raw LiDAR hits inside the path corridor → **emergency stop** | Anything the planner missed |
| + Recovery | Reverse with counter-steer if pinned; back off if a person blocks all routes | Deadlocks |

A **referee** in `simulator.py` checks the true robot footprint against the true world every step and counts collisions. The robot can't cheat.

### 3.7 How the output is visualised
The **dashboard** (1280×720) shows:
- **Left:** the ground-truth world, purple overlay = what the robot believes is dangerous (inflated costmap), green = LiDAR rays and hits, cyan = current plan, orange = driven trajectory, yellow = tracked people with ID, speed and predicted motion arrows.
- **Top right:** the BEV camera with computer-vision bounding boxes.
- **Middle right:** the robot's own occupancy map (white = mapped obstacles, red = moving objects, cyan = plan).
- **Bottom:** state (FOLLOWING / EMERGENCY_STOP / RECOVERY / GOAL_REACHED), speed, steering, distance to goal, plans/replans, last planning time, collisions, and a speed plot.

### 3.8 What a final run looks like
Warehouse example: the robot leaves the dock heading east along the cross aisle. At around x = 34 m its camera reveals a pallet wall blocking the aisle, and the plan instantly turns into a detour through the top corridor (`replan:path_blocked` appears in the log). It slows for the forklift crossing, turns down the far aisle, reaches the goal, and shows **GOAL REACHED** with 0 collisions.

---

## 4. Beginner step-by-step workflow

| Step | Action | Command / file |
|---|---|---|
| 1 | Set up the environment | see [SETUP_AND_RUN.md](SETUP_AND_RUN.md) |
| 2 | Run the tests to confirm everything works | `pytest -q` → `20 passed` |
| 3 | Watch the default demo live | `python main.py` |
| 4 | Run each scenario | `python main.py --scenario city` (also `crosswalk`, `random`) |
| 5 | Inspect the outputs | `outputs/<scenario>/` → `trajectory.png`, `telemetry.png`, `metrics.json`, `run.mp4` |
| 6 | Compare planners | `python main.py --scenario warehouse --planner dijkstra --headless` |
| 7 | Stress-test on random maps | `python main.py --scenario random --seed 42` |
| 8 | Run the benchmark | `python scripts/benchmark.py` → `outputs/benchmark/summary.md` |
| 9 | Generate README assets | `python scripts/export_assets.py` → `assets/` |
| 10 | Create your own scenario | copy `crosswalk()` in `scenarios.py`, rename it, add it to `SCENARIOS` |
| 11 | Tune parameters | edit `src/config.py` (e.g. `max_speed`, `safety_margin`) and re-run the benchmark |
| 12 | Publish | [GITHUB_GUIDE.md](GITHUB_GUIDE.md) |

### Make your own map (Step 10 in detail)
```python
# src/simulation/scenarios.py
def parking_lot() -> World:
    static = _border()
    for i in range(8):                                    # two rows of parked cars
        static.append(RectObstacle(10 + i * 7, 10, 2.0, 4.5))
        static.append(RectObstacle(10 + i * 7, 35, 2.0, 4.5))
    dynamic = [DynamicObstacle([(30, 20), (30, 30)], radius=0.45, speed=1.0)]
    return World("parking_lot", WORLD_W, WORLD_H, static, dynamic,
                 start=(4.0, 25.0, 0.0), goal=(76.0, 25.0),
                 description="Parking lot with a pedestrian")

SCENARIOS = {..., "parking_lot": parking_lot}
```
Then run `python main.py --scenario parking_lot`.

---

## 5. Proof to capture

### Screenshots
| Moment | Auto-generated file |
|---|---|
| Simulation starting | `outputs/warehouse/snapshot_start.png` |
| Camera detects and tracks a pedestrian | `outputs/crosswalk/snapshot_detection.png` |
| Robot discovers a blocked aisle and replans | `outputs/warehouse/snapshot_replan.png` |
| Emergency stop (if one occurs) | `outputs/<scenario>/snapshot_emergency_stop.png` |
| Goal reached | `outputs/<scenario>/final_frame.png` |
| Path planning result | `outputs/<scenario>/trajectory.png` |
| Speed/steering/clearance graphs | `outputs/<scenario>/telemetry.png` |
| Planner comparison | `outputs/benchmark/planner_comparison.png` |

Take these yourself:
- the terminal showing `pytest -q` → `20 passed`
- the terminal showing the `RESULT: SUCCESS` summary
- your GitHub repo page and the README preview

### Videos
- `outputs/<scenario>/run.mp4`: a full run at 2× speed (1280×720). Upload it to a **GitHub Release** or drag it into the README editor on GitHub (that creates a `user-attachments` link).
- `outputs/<scenario>/demo.gif`: the lightweight version for the README (`--gif` flag).
- Optional: a 60–90 s screen recording of `python main.py` with your voice explaining the dashboard (Windows: **Win + Alt + R** with Xbox Game Bar, or OBS Studio). This is great for LinkedIn.

### Files to commit to GitHub
- `assets/gifs/*.gif`, `assets/images/*.png` (generated by `scripts/export_assets.py`)
- `docs/benchmark_results.md` (copy of `outputs/benchmark/summary.md`)
- **Not** `outputs/` and **not** `*.mp4` (they're large and already in `.gitignore`)
