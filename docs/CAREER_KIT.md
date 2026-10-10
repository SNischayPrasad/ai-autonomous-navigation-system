# Career Kit: Resume, LinkedIn and Interview Preparation

> All numbers below come from `scripts/benchmark.py` (see `docs/benchmark_results.md`). If you change the code, re-run the benchmark and update them. **Never quote a number you can't reproduce.**

---

## 1. Resume bullet points (pick 2–3)

**Project title line:**
**AI-Based Autonomous Navigation System** | Python, OpenCV, NumPy, A*, Pure Pursuit, pytest | [github.com/SNischayPrasad/ai-autonomous-navigation-system]

1. Built an end-to-end **autonomous navigation stack** in Python (perception → mapping → planning → control) for a car-like robot in a custom 2D simulator with simulated **LiDAR and camera**. It reached the goal in **33 of 33 benchmark runs with 0 collisions** across warehouse, city, crosswalk and random-obstacle maps.
2. Developed an **OpenCV obstacle-detection pipeline** (HSV segmentation, morphology, contours) with **multi-object tracking and 2 s motion prediction**, and fused LiDAR and camera evidence into an **occupancy grid**. Moving pedestrians are kept out of the static map and the robot avoids their predicted paths.
3. Implemented and benchmarked **A\*, Dijkstra and Weighted A\*** on an inflated costmap: A* expanded **56 % fewer nodes** than Dijkstra with identical path cost, and Weighted A* planned **9× faster**. Added dynamic replanning, a LiDAR safety corridor with emergency stop, recovery behaviours and a **pure-pursuit** controller with a curvature-aware speed profile. Covered by **20 pytest unit/integration tests**.

*Short version (one line):* Built a simulated self-driving robot in Python with OpenCV perception, object tracking, occupancy mapping, A* planning and pure-pursuit control. 100 % success over 33 benchmark runs with 0 collisions.

---

## 2. LinkedIn project descriptions

### Version 1: Announcement post (with the GIF)
> 🚗 I built an **AI-based autonomous navigation system** from scratch, without any robot hardware.
>
> The robot starts with **no map**. It sees the world through a simulated **LiDAR and camera**, detects and tracks moving people with **OpenCV**, builds its own **occupancy map**, plans routes with **A\***, and drives with a **pure-pursuit controller**, just like the navigation stacks in warehouse robots and self-driving prototypes.
>
> 📊 Results: 100 % success over 33 benchmark runs, **0 collisions**. A* explored 56 % fewer nodes than Dijkstra for the same optimal path.
>
> What I learned: why obstacles must be *inflated* before planning, why you need hysteresis to stop a robot from flip-flopping between routes, and how a robot and a pedestrian can deadlock by both being polite 😄
>
> 🔗 Code + demo: github.com/SNischayPrasad/ai-autonomous-navigation-system
> #Robotics #AutonomousVehicles #ComputerVision #Python #OpenCV #PathPlanning #AI

### Version 2: For the "Projects" section of your profile
> Autonomous navigation stack for a car-like robot in a custom Python simulator. Pipeline: BEV camera + 2D LiDAR → OpenCV detection → multi-object tracking with motion prediction → LiDAR/camera occupancy-grid fusion → inflated costmap → A*/Dijkstra/Weighted A* → path smoothing → pure-pursuit control, with a safety corridor, emergency stop and recovery behaviours. Four scenarios (warehouse, city, crosswalk, random), a benchmark suite and 20 automated tests. Skills: Python, NumPy, OpenCV, algorithms, robotics, testing.

### Version 3: Short (for a featured post or portfolio card)
> Self-driving robot simulator: it sees, maps, plans and drives on its own. OpenCV + A* + pure pursuit, 0 collisions across 33 benchmark runs. Built in Python with no GPU and no hardware.

---

## 3. Interview questions with strong sample answers

**Q1. Explain your project in two minutes.**
> I built a complete navigation stack for a car-like robot and tested it in a simulator I wrote. Each 0.1 s the robot reads a simulated LiDAR and a top-down camera. OpenCV detects static structures and moving people, a tracker gives each person an ID and a velocity, and both sensors update an occupancy grid. The robot starts with no map, so it discovers obstacles as it drives. I inflate the map by the robot's radius, plan with A*, smooth the path, and follow it with pure pursuit. A separate safety layer checks raw LiDAR hits along the path and can emergency-stop. A state machine handles replanning, yielding and recovery. I benchmarked it on 11 maps with three planners: 100 % success with A* and zero collisions.

**Q2. Why A\* instead of Dijkstra? Is A\* always optimal?**
> Both find the optimal path. A* adds a heuristic, an estimate of the remaining cost, so it explores towards the goal. I use the octile distance, which is the exact cost on an empty 8-connected grid and never over-estimates, so it's *admissible* and A* stays optimal. My benchmark shows A* expanding 56 % fewer nodes than Dijkstra with exactly the same path cost, and a unit test asserts the costs are equal. Weighted A* (heuristic × 2) is 9× faster but can return a path up to 2× the optimal cost. That's a speed/optimality trade-off.

**Q3. Why do you inflate obstacles?**
> The planner treats the robot as a point. If I grow every obstacle by the robot radius plus a safety margin, any path for the point is collision-free for the whole robot. This is configuration-space planning. I also add a soft cost gradient near obstacles so the cheapest path runs down the middle of an aisle instead of hugging the shelf.

**Q4. How does the robot handle moving obstacles?**
> At three levels. First, perception: detections are tracked over frames and get a velocity, and I block a capsule covering where each person will be in the next 2 seconds, so the plan avoids their future path. Second, speed: near tracked people the robot caps its speed at 2 m/s. Third, safety: if any raw LiDAR hit enters a corridor along the path closer than the stopping distance (base + v²/2a), it emergency-stops. Also, LiDAR hits on tracked people are not written into the long-term map, so people don't leave "ghost walls".

**Q5. What is pure pursuit and how did you tune it?**
> It picks a point on the path one lookahead distance ahead and computes the steering angle of the circular arc that hits it: δ = atan(2L·sin α / Ld). A short lookahead tracks tightly but oscillates. A long one is smooth but cuts corners. I made the lookahead grow with speed (2 m + 0.5 s·v). Speed is limited separately: I estimate the path curvature ahead and slow down *before* the curve using √(a_lat/κ) and a braking curve.

**Q6. What was the hardest bug?**
> Two. First, the robot drove into an aisle and stopped facing a shelf. The cause was a periodic replan that switched between two almost equal routes during a turn, and a car can't turn on the spot. I fixed it with replan hysteresis (a new route must be 15 % cheaper), heading-aware planning (I block the half-disc behind the car on the first try), and a reverse recovery behaviour. Second, a cyclist and the robot deadlocked: the cyclist yielded to the robot and the robot waited for the cyclist. Now, if a moving object blocks the path for about 6 s, the robot backs off to give way. Both were found by the benchmark, which is why I run it after every change.

**Q7. How is this different from a real self-driving car?**
> The architecture is the same: perception, prediction, mapping, planning, control and safety. What's simplified: 2D instead of 3D, perfect localisation (no SLAM), colour-coded objects instead of real images, and a kinematic model without tyre dynamics. Each of these is a clear upgrade path: YOLO for detection, a Kalman filter for tracking, a particle filter or scan matching for localisation, Hybrid A* for kinematically feasible paths, and MPC for control.

**Q8. How did you verify that the system works?**
> 20 pytest tests: unit tests for the planner (optimality, blocked map, a path through a door), perception (detection error < 0.3 m, LiDAR accuracy, map updates, tracker velocity), control and the vehicle model, plus integration tests that run the full stack and assert goal reached with zero collisions. A referee in the simulator checks collisions against the ground truth, which the robot never sees. The benchmark script runs 33 closed-loop runs and outputs success rate, collisions, clearance and planning time.

**Q9. Your planner runs in Python. Is it fast enough?**
> In the closed loop, A* averages roughly 10–25 ms on a 320×200 grid at 0.25 m resolution, well inside the 100 ms control period. I converted NumPy arrays to flat Python lists inside the search loop because NumPy scalar indexing is slow in loops. For production I'd move the search to C++ or use a library planner, or use Weighted A* or D* Lite for incremental replanning.

**Q10. If you had 3 more months, what would you add?**
> 1) Replace the colour detector with YOLOv8 on rendered images and compare accuracy. 2) A Kalman-filter tracker with Hungarian assignment. 3) Hybrid A* so paths respect the turning radius. 4) Wrap the simulator as a Gymnasium environment and compare an RL policy against my classical stack. 5) Port the planner to ROS 2 and test it in Gazebo against Nav2.

---

## 4. Explaining it to HR (non-technical, 30 seconds)
> "I built software that lets a robot drive itself, like the robots in Amazon warehouses or a self-driving car, but in a computer simulation, so I didn't need expensive hardware. The robot sees its surroundings, recognises people and obstacles, plans the best route, and stops or goes around when something gets in the way. I tested it in different environments, such as a warehouse, a city and a crosswalk, and it reached its destination every time without a single collision. It taught me computer vision, algorithms and how to build and test a large project end to end."

## 5. Explaining it to a technical interviewer (structure)
1. **Problem:** goal-directed navigation for a non-holonomic robot in a partially known, dynamic environment.
2. **Architecture:** sense → perceive (CV, tracking) → map (occupancy grid, sensor fusion) → plan (costmap + A*) → decide (FSM) → act (pure pursuit). Draw the block diagram.
3. **One deep dive** (choose what you know best): A* optimality, costmap inflation, the tracker, or the safety corridor.
4. **Evaluation:** benchmark numbers and tests.
5. **Limitations and next steps:** be honest (perfect localisation, 2D, colour-coded perception). This shows maturity.

## 6. Talking about industry relevance with confidence
- "The costmap with inflation and A* global planner is the same concept as the ROS 2 **Nav2** stack used by most AMR companies."
- "Pure pursuit is used in **Autoware**, the open-source autonomous-driving stack, and on many industrial AGVs."
- "The emergency-stop corridor mirrors the protective fields of **safety laser scanners** that standards like ISO 3691-4 require on driverless industrial trucks."
- "Constant-velocity prediction is the standard *baseline* in AV prediction benchmarks. Learned predictors are compared against it."
- "Warehouse automation (Amazon Robotics, GreyOrange, Addverb) and delivery robots are among the fastest-growing robotics markets, and they all need exactly this pipeline."
