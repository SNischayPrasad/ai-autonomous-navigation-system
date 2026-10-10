# Project Guide: Understanding and Building the System

> Read this first. It explains *what* the project is, *why* it matters, *which* tech stack was chosen and *how* it was built phase by phase.

---

## A. What is an AI-Based Autonomous Navigation System?

### In simple language
Imagine a robot in a warehouse that must carry a box from the entrance to shelf 12. Nobody drives it. It has to:
1. **See** what is around it (walls, shelves, people, forklifts).
2. **Understand** what it sees: "this is a wall that will not move", "this is a person walking left".
3. **Remember** what it has seen by building its own map.
4. **Think** about the best route to the goal.
5. **Drive** along that route smoothly.
6. **React** instantly when something unexpected happens: stop for a person, find another route when an aisle is blocked.

A system that does all six of these by itself is an **autonomous navigation system**. "AI-based" means the robot makes these decisions itself, using perception (computer vision), search algorithms (A*), prediction (tracking) and decision logic (a state machine), instead of following a fixed, pre-programmed route.

### In technical language
An autonomous navigation system is a closed-loop **perception → mapping → planning → control** pipeline:
- **Perception** turns raw sensor data (camera pixels, LiDAR ranges) into a semantic, metric representation: detected objects, their classes and positions.
- **State estimation and tracking** associate detections over time and estimate the velocities of dynamic agents.
- **Mapping** fuses observations into an occupancy grid (a probabilistic map of free and occupied space).
- **Planning** searches the configuration space for a collision-free, low-cost path (graph search such as A*, or sampling such as RRT*), then smooths it.
- **Decision making** (behaviour layer) handles events: replanning, yielding, emergency stops and recovery.
- **Control** converts the geometric path into actuator commands (steering, throttle) while respecting vehicle kinematics.

### What problem does it solve?
- **Labour and cost:** warehouses, factories and hospitals need materials moved 24/7.
- **Safety:** humans get tired, and machines don't. Industrial vehicles and road traffic cause many injuries each year.
- **Scale:** one control centre can supervise hundreds of robots.
- **Access:** robots can go where it is unsafe for people (mines, disaster sites, chemical plants).

### Where is it used in the real world?
| Industry | Example systems | What navigation does there |
|---|---|---|
| **Warehouse automation** | Amazon Robotics, Locus Robotics, GreyOrange, Addverb | AMRs move shelves and totes, avoid workers, re-route around blocked aisles. This is exactly this project's warehouse scenario. |
| **Autonomous vehicles** | Waymo, Tesla FSD, Cruise, Baidu Apollo, Tata Elxsi and KPIT (Indian AV software) | Perception, prediction of pedestrians, path planning, control. |
| **Delivery robots** | Starship, Serve Robotics, Ottonomy | Sidewalk navigation and yielding at crosswalks (this project's crosswalk scenario). |
| **Drones** | DJI obstacle avoidance, Zipline, Skydio | 3D path planning and obstacle avoidance. |
| **Industrial safety** | Autonomous forklifts, AGVs in factories | Safety corridors, emergency stops, speed zones near people. |
| **Smart mobility** | Airport shuttles, campus pods, robotaxis | Route planning in mixed traffic. |
| **Agriculture / mining** | John Deere autonomous tractors, Komatsu haul trucks | Field coverage planning, obstacle detection. |
| **Healthcare / hospitality** | Hospital delivery robots, restaurant serving robots | Indoor navigation among people. |

### How companies use these exact building blocks
- **Occupancy grids and costmaps:** the ROS Navigation stack (used by most AMR start-ups) uses layered costmaps with inflation, just like `costmap.py`.
- **A\* / Dijkstra:** global planners in ROS (`NavFn`, `SmacPlanner`) and in game/robot engines.
- **Pure Pursuit:** used on the CMU NavLab, in Autoware (an open-source AV stack) and on many AGVs.
- **Tracking with constant-velocity prediction:** the baseline motion model in AV prediction modules.
- **Safety corridor / emergency stop:** required by safety standards such as ISO 3691-4 (driverless industrial trucks).
- **Recovery behaviours:** ROS `move_base` "back up / clear costmap" recoveries.

### The complete workflow
| Step | What happens | Implemented in |
|---|---|---|
| **Perception** | Sensors observe the scene | `simulation/sensors.py` |
| **Computer vision** | Image → HSV → colour segmentation → contours | `perception/obstacle_detector.py` |
| **Object detection** | Blobs become labelled detections with position/size/confidence | `perception/obstacle_detector.py` |
| **Lane/path detection** | Free space is detected from LiDAR rays and camera masks and stored in the occupancy grid. A soft cost keeps the robot centred in its "lane"/aisle | `perception/occupancy_grid.py`, `planning/costmap.py` |
| **Obstacle avoidance** | Inflated costmap, predicted pedestrian motion, safety corridor | `planning/costmap.py`, `perception/tracker.py`, `navigation/navigator.py` |
| **Decision making** | Finite-state machine: follow / stop / replan / recover | `navigation/navigator.py` |
| **Path planning** | A* (or Dijkstra/Weighted A*) + smoothing | `planning/astar.py`, `planning/smoothing.py` |
| **Control / navigation** | Pure pursuit steering + speed profile → bicycle model | `control/pure_pursuit.py`, `simulation/robot.py` |
| **Simulation / testing** | 4 scenarios, collision referee, metrics, 20 unit and integration tests, benchmark | `simulation/simulator.py`, `tests/`, `scripts/benchmark.py` |

---

## B. Tech stack options

| | **Option A: Easiest** | **Option B: Intermediate (CHOSEN)** | **Option C: Advanced** |
|---|---|---|---|
| **What** | Grid-world in Pygame/Matplotlib, robot jumps cell to cell, BFS/A* only | Continuous 2D simulator with a car model, simulated LiDAR + camera, OpenCV perception, tracking, occupancy mapping, A*, pure pursuit, safety layer, dashboard, tests, benchmark | CARLA (Unreal Engine) or ROS 2 + Gazebo/Webots, YOLOv8 on real camera images, SLAM, MPC control |
| **Tools** | Python, NumPy, Pygame | Python, NumPy, OpenCV, Matplotlib, pytest | Python/C++, CARLA or ROS 2, Gazebo, PyTorch, Ultralytics YOLO, Docker, Linux |
| **Difficulty** | ★☆☆☆☆ | ★★★☆☆ | ★★★★★ |
| **GPU needed?** | No | **No**, runs on any laptop | Yes. CARLA needs a strong GPU (6–8 GB VRAM recommended) |
| **Setup time** | 10 min | 10 min | 1–3 days (Linux, drivers, ROS) |
| **Outcome** | A toy demo. Recruiters have seen thousands of these | A complete, explainable autonomy stack with video proof, metrics, tests and benchmarks | Most realistic, but students often get stuck in setup and never finish |
| **Interview value** | Low | **High**: every module maps to a real industry concept you can explain | Very high *if finished* |

### C. Selected approach: Option B
Option B is the best choice for a student who wants **strong GitHub proof quickly**:
- It is **complete**: perception, tracking, mapping, planning, decision making, control, safety, simulation, testing and benchmarking.
- It needs **no GPU and no Linux**: it runs on Windows, macOS and Linux with `pip install`.
- It is **deterministic and measurable**, so you can quote real numbers (success rate, collisions, planning time).
- It is **modular**: each block can later be swapped for an advanced version (YOLO, ROS 2, CARLA, MPC) without rewriting the rest. That upgrade path is a great interview story.

---

## D. Implementation plan (how this project was built)

Each phase lists **what**, **why**, **expected output**, **common beginner mistakes**, and **how to verify**.

### Phase 1: Environment setup
- **What:** Install Python 3.10–3.13, create a virtual environment, install `requirements.txt`.
- **Why:** An isolated environment means reproducible results and no library conflicts.
- **Output:** `python -c "import cv2, numpy, matplotlib; print('ok')"` prints `ok`.
- **Mistakes:** installing into the global Python; forgetting to activate `.venv`; using Python 3.14 alpha builds.
- **Verify:** `pip list` shows numpy, opencv-python, matplotlib, pytest.

### Phase 2: Project structure
- **What:** Create `src/` packages (`simulation`, `perception`, `planning`, `navigation`, `control`, `visualization`), plus `tests/`, `scripts/`, `docs/`, `assets/`.
- **Why:** Separation of concerns. Recruiters judge code organisation within 30 seconds.
- **Output:** the folder tree in the README.
- **Mistakes:** one 1000-line `main.py`; missing `__init__.py` files (imports fail).
- **Verify:** `python -c "import src.config"` works from the project root.

### Phase 3: Simulation
- **What:** `world.py` (map + obstacles), `robot.py` (bicycle model), `sensors.py` (LiDAR + camera), `scenarios.py`.
- **Why:** You can't test autonomy without a world. The simulation replaces hardware.
- **Output:** `World.render()` returns a top-down image, and the robot moves when given commands.
- **Mistakes:** mixing up rows/cols and x/y; letting the planner read the ground-truth map ("cheating").
- **Verify:** `pytest tests/test_control.py::test_robot_drives_straight` and `tests/test_perception.py::test_lidar_measures_distance_to_wall`.

### Phase 4: Perception
- **What:** OpenCV detector, multi-object tracker, occupancy grid.
- **Why:** The robot must build its own understanding of the world from noisy sensors.
- **Output:** detections with positions within ±0.3 m, track IDs with velocities, an occupancy map that fills in as the robot drives.
- **Mistakes:** forgetting that OpenCV uses **BGR** and hue in **0–179**; not removing noise with morphology; mapping moving people as permanent walls.
- **Verify:** `pytest tests/test_perception.py`.

### Phase 5: Path planning
- **What:** Costmap inflation, A*/Dijkstra/Weighted A*, smoothing.
- **Why:** This finds a safe and short route.
- **Output:** a list of waypoints from start to goal that never touches inflated obstacles.
- **Mistakes:** forgetting to inflate obstacles (the path scrapes walls); an inadmissible heuristic; letting diagonal moves cut corners.
- **Verify:** `pytest tests/test_planning.py` (6 tests, including "A* cost equals Dijkstra cost").

### Phase 6: Navigation and control
- **What:** Navigator state machine, safety corridor, recovery, pure pursuit + speed profile.
- **Why:** Plans become motion, and events (blocked aisle, pedestrian) become decisions.
- **Output:** the robot reaches the goal in every scenario with zero collisions.
- **Mistakes:** braking with a negative acceleration that makes the car reverse; replanning every step (path flip-flop); ignoring that cars can't turn in place.
- **Verify:** `pytest tests/test_integration.py`.

### Phase 7: Visualization
- **What:** OpenCV dashboard, MP4/GIF recording, Matplotlib plots.
- **Why:** Recruiters spend about 10 seconds on a repo, and a GIF tells the story instantly.
- **Output:** `outputs/<scenario>/run.mp4`, `demo.gif`, `trajectory.png`, `telemetry.png`, snapshots.
- **Mistakes:** GIFs that are too large (keep them under 10 MB); unreadable text.
- **Verify:** open the files and check the dashboard shows state, speed and path.

### Phase 8: Testing and benchmarking
- **What:** 20 pytest tests and `scripts/benchmark.py`.
- **Why:** Numbers make claims believable ("100 % success over 11 maps, 0 collisions").
- **Output:** `outputs/benchmark/summary.md`, `planner_comparison.png`.
- **Verify:** `pytest -q` → `20 passed`.

### Phase 9: GitHub publishing
See [GITHUB_GUIDE.md](GITHUB_GUIDE.md).

### Phase 10: Resume and interview
See [CAREER_KIT.md](CAREER_KIT.md).
