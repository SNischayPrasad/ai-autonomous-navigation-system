# GitHub Guide: Publishing, Commit Strategy and Proof Assets

---

## 1. Repository identity (copy-paste)

| Field | Value |
|---|---|
| **Repository name** | `ai-autonomous-navigation-system` |
| **Description** | Autonomous robot navigation stack in a 2D simulator: OpenCV perception, multi-object tracking, LiDAR+camera occupancy mapping, A*/Dijkstra planning, pure pursuit control and safety layer. 4 scenarios, benchmarks, 20 tests. |
| **Website** | link to your demo video (YouTube unlisted / LinkedIn post) once you have one |
| **Topics** | `autonomous-navigation` `robotics` `path-planning` `a-star` `computer-vision` `opencv` `self-driving-car` `autonomous-vehicles` `obstacle-avoidance` `occupancy-grid` `pure-pursuit` `lidar` `simulation` `python` `motion-planning` `object-tracking` `amr` `warehouse-automation` `sensor-fusion` `numpy` |
| **License** | MIT (already in `LICENSE`, so update the name in it) |

---

## 2. One-time setup

```bash
git --version
git config --global user.name "Your Name"
git config --global user.email "your-github-email@example.com"
git config --global init.defaultBranch main
```
To log in to GitHub, install the GitHub CLI (https://cli.github.com) and run:
```bash
gh auth login
```

---

## 3. Create the repository and push

### Option A: GitHub website + git
1. Go to https://github.com/new.
2. Name it `ai-autonomous-navigation-system`, make it **Public**, and **don't** add a README, .gitignore or license (we already have them).
3. Click **Create repository**.
4. In the project root:
```bash
git init
git add .
git status
git commit -m "feat: initial autonomous navigation stack"
git branch -M main
git remote add origin https://github.com/<your-username>/ai-autonomous-navigation-system.git
git push -u origin main
```
Check `git status` before you commit. You must **not** see `.venv/` or `outputs/` in the list.

### Option B: everything from the terminal with GitHub CLI
```bash
git init
git add .
git commit -m "feat: initial autonomous navigation stack"
gh repo create ai-autonomous-navigation-system --public --source=. --remote=origin --push
```

### After pushing
1. On the repo page, click ⚙ next to **About**. Paste the description and topics.
2. Go to **Settings → General → Features** and enable **Discussions** (optional).
3. **Pin it:** open your profile → **Customize your pins** → select this repo.
4. **Release:** go to **Releases → Draft a new release**, tag `v1.0.0`, title "v1.0.0: Autonomous navigation stack", and **attach `outputs/warehouse/run.mp4`** (and the other MP4s). Videos belong here, not in git.

---

## 4. Uploading screenshots, GIFs and video

1. Generate the assets: `python scripts/export_assets.py` (run `python scripts/benchmark.py` first for the planner chart).
2. They are written to:
```
assets/
├── gifs/
│   ├── warehouse_demo.gif
│   ├── crosswalk_demo.gif
│   └── city_demo.gif
└── images/
    ├── 01_simulation_environment.png
    ├── 02_obstacle_detection_tracking.png
    ├── 03_dynamic_replanning.png
    ├── 04_path_planning_trajectory.png
    ├── 05_goal_reached.png
    ├── 06_city_trajectory.png
    ├── 07_telemetry.png
    ├── 08_random_map_trajectory.png
    ├── 09_planner_comparison.png
    ├── 10_tests_passing.png          ← take this screenshot yourself
    ├── 11_terminal_run_summary.png   ← take this screenshot yourself
    └── 12_github_repo.png            ← take this after publishing
```
3. Reference them in the README with **relative paths** (case-sensitive!):
```markdown
![Warehouse demo](assets/gifs/warehouse_demo.gif)
<img src="assets/images/03_dynamic_replanning.png" width="700">
```
4. **Embed the MP4 in the README:** on github.com, edit `README.md` with the pencil icon and drag `run.mp4` into the editor. GitHub uploads it and inserts a `https://github.com/user-attachments/assets/...` link that plays inline. Commit that edit and then run `git pull` locally.

Rules: keep each GIF under 10 MB and each image under 1 MB. Name files with lowercase letters, numbers and underscores.

---

## 5. Commit message style (Conventional Commits)

Format: `type(scope): short imperative summary`

| Type | Use for | Example |
|---|---|---|
| `feat` | new feature | `feat(planning): add A* planner with octile heuristic` |
| `fix` | bug fix | `fix(control): stop braking from reversing the car` |
| `test` | tests | `test(perception): verify detector localisation error < 0.3 m` |
| `docs` | documentation | `docs: add architecture diagram and data flow` |
| `refactor` | restructure, no behaviour change | `refactor(nav): extract corridor check into helper` |
| `perf` | speed-ups | `perf(astar): use flat lists instead of numpy indexing` |
| `chore` | tooling | `chore: add .gitignore and requirements` |

Good messages explain **what changed and why**. Bad: `update`, `final`, `asdf`, `changes`.

---

## 6. Step-by-step proof-building plan (commit strategy)

> **Be genuine.** The point of a commit history is to show *your* learning. Commit each stage **after you have read, run and understood that part of the code**, ideally after changing something yourself (a parameter, a new scenario, a test). Never backdate commits or fake history. Interviewers often ask "walk me through your commits", and real understanding is what gets you hired.

Here is a 7-stage plan (about a day or two each, at your pace). To publish progressively, start with an empty repo and add the folders stage by stage. Otherwise, publish everything at once and use the stages for your **own upgrades** (see ROADMAP).

| Stage | What to commit | Proof to save | Commit message | Notes for LinkedIn/resume |
|---|---|---|---|---|
| **1. Setup** | `README.md` (skeleton), `requirements.txt`, `.gitignore`, `LICENSE`, `src/config.py`, folder structure | terminal: venv active, `pip list` | `chore: project setup, dependencies and config` | "Started building an autonomous navigation stack from scratch in Python" |
| **2. Simulation** | `src/simulation/world.py`, `robot.py`, `sensors.py`, `scenarios.py` | screenshot of `World.render()` / first dashboard frame | `feat(sim): 2D world, bicycle-model robot, LiDAR and BEV camera` | Explain the bicycle model and simulated sensor noise |
| **3. Perception** | `src/perception/*`, `tests/test_perception.py` | annotated camera frame with boxes; occupancy map | `feat(perception): OpenCV obstacle detection, tracking and occupancy mapping` | "Detector localises objects within 0.3 m; tracker estimates velocity" |
| **4. Path planning** | `src/planning/*`, `tests/test_planning.py` | `trajectory.png`, planner comparison chart | `feat(planning): costmap inflation, A*/Dijkstra/Weighted A*, path smoothing` | Quote the nodes-expanded reduction of A* vs Dijkstra from `summary.md` |
| **5. Obstacle avoidance and control** | `src/control/*`, `src/navigation/*`, `tests/test_control.py` | `snapshot_replan.png`, emergency stop/yield screenshot | `feat(nav): state machine, safety corridor, recovery and pure pursuit control` | Explain the 3 safety layers |
| **6. Integration and benchmark** | `src/simulation/simulator.py`, `src/visualization/*`, `main.py`, `scripts/*`, `tests/test_integration.py` | `run.mp4`, `demo.gif`, `pytest` screenshot, `summary.md` | `feat: end-to-end simulation, dashboard, benchmark suite` | Quote the success rate and collision numbers from the benchmark |
| **7. Documentation and release** | `docs/*`, final `README.md`, `assets/*` | repo screenshot, README preview | `docs: architecture, guides, results and demo assets` | Publish the release v1.0.0 and the LinkedIn post with the GIF |

Exact commands for any stage:
```bash
git add <files of this stage>
git commit -m "<message from the table>"
git push
```

---

## 7. Proof assets checklist

| # | Asset | File name | How to get it |
|---|---|---|---|
| 1 | Terminal: tests passing | `assets/images/10_tests_passing.png` | `pytest -v`, screenshot (Win + Shift + S) |
| 2 | Terminal: successful run summary | `assets/images/11_terminal_run_summary.png` | `python main.py --headless`, screenshot the RESULT table |
| 3 | Simulation environment | `assets/images/01_simulation_environment.png` | auto (`export_assets.py`) |
| 4 | Obstacle detection + tracking | `assets/images/02_obstacle_detection_tracking.png` | auto |
| 5 | Path planning / replanning | `assets/images/03_dynamic_replanning.png`, `04_path_planning_trajectory.png` | auto |
| 6 | Navigation output (telemetry) | `assets/images/07_telemetry.png` | auto |
| 7 | Final successful run | `assets/images/05_goal_reached.png` | auto |
| 8 | Planner benchmark | `assets/images/09_planner_comparison.png` | auto (after `benchmark.py`) |
| 9 | Demo GIFs | `assets/gifs/*.gif` | auto |
| 10 | Demo video | GitHub Release attachment / README user-attachment | `outputs/*/run.mp4` |
| 11 | GitHub repo page | `assets/images/12_github_repo.png` | screenshot after publishing |
| 12 | README preview | for LinkedIn/portfolio (not needed in the repo) | screenshot of the rendered README |

---

## 8. How recruiters read a repo (structure your README for this)
1. **0–10 s:** title, one-line pitch, badges, **GIF**. They decide whether to keep reading.
2. **10–30 s:** "Key results" table with numbers.
3. **30–60 s:** architecture diagram, tech stack.
4. **Technical reviewers:** folder structure, code quality, tests, docs.

The provided README follows exactly this order.
