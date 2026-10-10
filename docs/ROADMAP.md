# Roadmap: How to Upgrade This Project

The stack is modular on purpose, so each item below replaces **one** block without rewriting the others. Doing even one upgrade, and writing about it, makes the project clearly *yours*.

| # | Upgrade | Difficulty | What changes | Where to start |
|---|---|---|---|---|
| 1 | **YOLO object detection** | ★★☆ | Replace `ObstacleDetector.detect()` with a YOLOv8n model on rendered/real images; keep the `Detection` output format | `src/perception/obstacle_detector.py` |
| 2 | **Real-time camera input** | ★★☆ | Read a webcam with `cv2.VideoCapture(0)`, run YOLO, project detections to the ground plane with a homography (`cv2.getPerspectiveTransform`) | new `src/perception/camera_input.py` |
| 3 | **Lane detection** | ★★☆ | Canny edges + Hough lines (classic) or a segmentation net on road images; feed lane boundaries into the costmap as lethal/soft cost | new `src/perception/lane_detector.py` |
| 4 | **Kalman filter tracking** | ★★☆ | Replace the EMA velocity estimate with a constant-velocity Kalman filter + Hungarian assignment (`scipy.optimize.linear_sum_assignment`) | `src/perception/tracker.py` |
| 5 | **Hybrid A\* / RRT\*** | ★★★ | Plan in (x, y, θ) so paths respect the car's turning radius (as in Tesla/Waymo parking and the DARPA Urban Challenge) | `src/planning/` |
| 6 | **MPC controller** | ★★★ | Model Predictive Control with `cvxpy`/`casadi`: optimise steering over a horizon with constraints | `src/control/` |
| 7 | **SLAM** | ★★★ | Remove the perfect pose: add odometry noise and estimate the pose with scan matching (ICP) or a particle filter | new `src/localization/` |
| 8 | **Reinforcement learning** | ★★★ | Wrap the simulator as a Gymnasium env (obs = LiDAR, action = accel/steer, reward = progress − collisions); train PPO with Stable-Baselines3; compare against A* + pure pursuit | new `rl/` folder |
| 9 | **Multiple agents** | ★★☆ | Several robots share one world; add priority rules or reservation tables (fleet management, like Amazon Robotics) | `src/simulation/simulator.py` |
| 10 | **ROS 2 integration** | ★★★ | Publish `/scan`, `/map`, `/cmd_vel`, `/odom`; visualise in RViz; replace the planner with Nav2 to compare | new `ros2_ws/` |
| 11 | **CARLA integration** | ★★★★ | Use CARLA's camera + LiDAR sensors and vehicle; keep your planner/controller (needs a GPU) | new `carla_bridge/` |
| 12 | **Cloud dashboard** | ★★☆ | Stream telemetry to a Streamlit/FastAPI web app; view runs remotely | new `dashboard/` |
| 13 | **IoT integration** | ★★☆ | Send robot status over MQTT (`paho-mqtt`) to a fleet monitor | new `iot/` |
| 14 | **Autonomous warehouse robot** | ★★☆ | Add pick/drop tasks, a task queue, battery and charging station | `scenarios.py`, new `mission/` |
| 15 | **Drone navigation** | ★★★ | Extend the grid to 3D (voxels), A* in 3D, a quadrotor point-mass model | new `drone/` |
| 16 | **Real hardware** | ★★★ | Raspberry Pi + RPLiDAR A1 + an RC car or TurtleBot: same pipeline, real sensors | — |

## Suggested order for a student
1. **Kalman tracker (#4):** about 1 day, a big interview talking point.
2. **YOLO (#1):** shows deep-learning skills with a pretrained model; no training is required.
3. **Gymnasium + RL (#8):** compare a "learned" agent vs the "classical" planner in a table. This is excellent research-style content.
4. **ROS 2 (#10):** makes the project industry-standard and opens robotics job postings.

## How to document an upgrade
1. Create a branch: `git checkout -b feature/kalman-tracker`.
2. Add tests for the new module.
3. Re-run `scripts/benchmark.py` and paste the **before/after table** in the README.
4. Merge it with a pull request, even though you're solo, so GitHub shows a professional workflow.
5. Post a short LinkedIn update with the GIF and the metric improvement.
