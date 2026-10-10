# Installation, Setup and Running

---

## 1. Requirements
| Item | Version |
|---|---|
| Python | **3.10 – 3.13** (tested on 3.13.11) |
| OS | Windows 10/11, macOS 12+, Ubuntu 20.04+ |
| GPU | **Not required** |
| RAM | 4 GB is enough |
| Disk | ~300 MB (virtual environment + outputs) |

Libraries (`requirements.txt`): `numpy`, `opencv-python`, `matplotlib`, `pillow`, `pytest`.

---

## 2. Setup on Windows (PowerShell)

```powershell
# 1. Check Python (install from python.org if missing; tick "Add to PATH")
python --version

# 2. Go to the project folder
cd path\to\ai-autonomous-navigation-system

# 3. Create a virtual environment
python -m venv .venv

# 4. Activate it (the prompt now starts with (.venv))
.venv\Scripts\Activate.ps1
#    If you see "running scripts is disabled", run this once and retry:
#    Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
#    In Command Prompt (cmd.exe) use: .venv\Scripts\activate.bat

# 5. Install the dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 3. Setup on macOS / Linux

```bash
python3 --version
cd path/to/ai-autonomous-navigation-system
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```
Ubuntu may need `sudo apt install python3-venv` first. On a server without a display, always add `--headless`.

## 4. Check that the installation worked

```bash
python -c "import numpy, cv2, matplotlib; print('numpy', numpy.__version__, '| opencv', cv2.__version__, '| matplotlib', matplotlib.__version__)"
pytest -q
```
Expected:
```
numpy 2.x | opencv 4.x/5.x | matplotlib 3.x
....................                                                     [100%]
20 passed in ~40s
```

## 5. Generating requirements.txt yourself
`requirements.txt` already exists and uses minimum versions, which is the most portable choice. To freeze the *exact* versions in your environment:
```bash
pip freeze > requirements-lock.txt
```
Commit `requirements.txt`. The lock file is optional.

---

## 6. Running the project

| Goal | Command |
|---|---|
| Live demo (window) | `python main.py` |
| Specific scenario | `python main.py --scenario city` (`warehouse`, `city`, `crosswalk`, `random`) |
| All scenarios, no window | `python main.py --scenario all --headless` |
| Also make GIFs | `python main.py --scenario warehouse --headless --gif` |
| Compare planners | `python main.py --planner dijkstra` / `--planner weighted_astar` |
| A new random map | `python main.py --scenario random --seed 42` |
| Faster (no MP4) | add `--no-video` |
| Unit + integration tests | `pytest -v` |
| Test only path planning | `pytest tests/test_planning.py -v` |
| Test only detection/tracking | `pytest tests/test_perception.py -v` |
| Full benchmark (~10 min) | `python scripts/benchmark.py` (or `--quick`) |
| Regenerate README assets | `python scripts/export_assets.py` |

Window controls: **q** quits, **p** pauses.

### What success looks like
- The dashboard state turns **GOAL REACHED** (yellow).
- The terminal prints `RESULT: SUCCESS` with `Collisions 0`.
- The exit code is `0` (`echo $?` / `echo %ERRORLEVEL%`).

### Output files (`outputs/<scenario>/`)
| File | Content |
|---|---|
| `run.mp4` | Full dashboard video at 2× speed, 1280×720 |
| `demo.gif` | Lightweight animation (with `--gif`) |
| `snapshot_start.png` | First frames of the run |
| `snapshot_detection.png` | First time a moving object is detected and tracked |
| `snapshot_replan.png` | First time the robot discovers a blocked path and replans |
| `snapshot_emergency_stop.png` | First emergency stop (only if one happened) |
| `final_frame.png` | Last frame (goal reached) |
| `trajectory.png` | Map + initial plan vs driven trajectory + moving-obstacle routes |
| `telemetry.png` | Speed, steering, clearance and state over time |
| `telemetry.csv` | Every control step: t, x, y, θ, speed, steering, state, clearance |
| `metrics.json` | Summary metrics for the run |
| `run.log` | Timestamped event log (state changes, replans, collisions) |

---

## 7. Sample terminal output

The log and result table from a real run of `python main.py --scenario warehouse --headless`
are recorded in [`sample_run_output.txt`](sample_run_output.txt).

## 8. Expected images
- **trajectory.png:** grey obstacles, a blue dashed *initial* plan (made while the map was still unknown, so it cuts straight through undiscovered shelves), and an orange *driven* path that bends around everything the robot discovered.
- **telemetry.png:** speed rising to 3 m/s on straights, dipping before corners and near people, and clearance always above 0.
- **planner_comparison.png:** bars showing Dijkstra expanding the most nodes, A* far fewer, and Weighted A* the fewest.
