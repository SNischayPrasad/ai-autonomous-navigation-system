# Troubleshooting

Each problem lists the **reason**, the **solution**, and a **prevention tip**.

---

## 1. Installation errors

### `python` is not recognized (Windows)
- **Reason:** Python isn't installed or isn't on PATH.
- **Solution:** Install Python 3.12 from python.org and tick **"Add python.exe to PATH"**. Alternatively, use the launcher: `py -3.12 -m venv .venv`.
- **Prevention:** After installing, open a *new* terminal and run `python --version`.

### `Activate.ps1 cannot be loaded because running scripts is disabled`
- **Reason:** The PowerShell execution policy blocks scripts.
- **Solution:** `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, then activate again. You can also use Command Prompt: `.venv\Scripts\activate.bat`.
- **Prevention:** Run this once per Windows account.

### `pip install opencv-python` fails / builds from source for a long time
- **Reason:** Your Python version is too new or too old for the available wheels, or pip is outdated.
- **Solution:** `python -m pip install --upgrade pip`, then install again. Use Python 3.10–3.13.
- **Prevention:** Stick to a mainstream Python version for project work.

### `ModuleNotFoundError: No module named 'cv2'` (or numpy)
- **Reason:** The packages were installed into a different Python than the one running the code (usually the venv isn't active).
- **Solution:** Activate the venv (your prompt shows `(.venv)`), then `pip install -r requirements.txt`. In VS Code, choose the interpreter: *Ctrl+Shift+P → Python: Select Interpreter → .venv*.
- **Prevention:** Always run `python -m pip ...` instead of a bare `pip`.

### `ModuleNotFoundError: No module named 'src'`
- **Reason:** The script is run from the wrong folder.
- **Solution:** `cd` into the project root (the folder that contains `main.py`) and run `python main.py`. For scripts: `python scripts/benchmark.py` from the root.
- **Prevention:** Keep your terminal in the project root.

## 2. Library conflicts

### `ImportError: numpy.core.multiarray failed to import` / OpenCV vs NumPy mismatch
- **Reason:** An old OpenCV wheel built against NumPy 1.x is being used with NumPy 2.x.
- **Solution:** `pip install --upgrade opencv-python numpy`.
- **Prevention:** Use a fresh venv per project.

### `opencv-python` and `opencv-contrib-python` / `opencv-python-headless` installed together
- **Reason:** Multiple OpenCV packages overwrite each other's `cv2`.
- **Solution:** `pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless`, then `pip install opencv-python`.
- **Prevention:** Install only one OpenCV flavour.

## 3. Simulator window not opening

### Nothing appears, or `cv2.error: ... The function is not implemented ... imshow`
- **Reason:** `opencv-python-headless` is installed (it has no GUI), or there's no display (SSH, WSL, Codespaces, Colab).
- **Solution:** Use `python main.py --headless`. The MP4, GIF and PNGs are still produced. For a window, install `opencv-python` (not headless).
- **Prevention:** On servers, always use `--headless`.

### Window freezes / "Not Responding"
- **Reason:** OpenCV only refreshes the window inside `cv2.waitKey`. A breakpoint or a long blocking call stops it.
- **Solution:** Let it run. Press **q** to quit and **p** to pause.

### `run.mp4` won't play
- **Reason:** The `mp4v` codec isn't supported by some players (for example, a browser preview).
- **Solution:** Open it with VLC, or convert it: `ffmpeg -i run.mp4 -vcodec libx264 run_h264.mp4`. The GIF always works.

## 4. "Model" / perception not working

### No moving objects are detected
- **Reason:** The HSV thresholds don't match the colours (for example, after you change the palette in `world.py`).
- **Solution:** Save a camera frame and check the HSV values: `cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[y, x]`. Update `STATIC_HSV` / `DYNAMIC_HSV` in `obstacle_detector.py`.
- **Prevention:** Remember that OpenCV hue runs from **0–179**, not 0–360, and images are **BGR**.

### YOLO model not loading (after upgrading to a neural detector)
- **Reason:** Wrong weights path, `ultralytics` not installed, or a CUDA/torch mismatch.
- **Solution:** `pip install ultralytics`, then `YOLO("yolov8n.pt")` (downloads automatically). Force CPU with `device="cpu"`.
- **Prevention:** Pin versions in `requirements.txt`.

## 5. Path planning failures

### Log says `no path exists` / state goes to `FAILED`
- **Reason:** The goal is enclosed, or the inflation (robot radius + margin) closes narrow gaps.
- **Solution:** Make sure gaps are wider than `2 × (robot.radius + planner.safety_margin)` = 2.4 m. Move the goal away from walls. Reduce `safety_margin` slightly.
- **Prevention:** When you design maps, run `python main.py --scenario <name> --headless` and check `trajectory.png`.

### `goal is blocked`
- **Reason:** The goal lies inside an obstacle's inflation zone.
- **Solution:** Move the goal at least 1.5 m from any obstacle.

### Planning is slow (> 100 ms)
- **Reason:** Dijkstra on a large map, or a very fine `grid_resolution`.
- **Solution:** Use `--planner astar` or `weighted_astar`, and keep the resolution at 0.25 m.

## 6. Collision / avoidance logic errors

### Robot collides with a pedestrian
- **Reason:** The prediction horizon or inflation is too small for fast agents, or `stop_distance` is too short.
- **Solution:** Increase `perception.prediction_horizon_s` or `dynamic_inflation`, or lower `control.caution_speed`.
- **Prevention:** Run `scripts/benchmark.py` after every tuning change and compare collisions.

### Robot stops forever in EMERGENCY_STOP
- **Reason:** A wall is inside the safety corridor (the corridor is too wide, or the path is too close to the wall).
- **Solution:** Keep `corridor_half_width` below `robot.radius + safety_margin`. Recovery (reversing) triggers automatically after two failed attempts.

### Robot drives in circles
- **Reason:** The target is behind the car (it can't turn on the spot), or the path keeps flip-flopping.
- **Solution:** This is already handled by heading-aware planning and replan hysteresis. If you change them, keep `replan_hysteresis < 1`.

## 7. Git errors

| Error | Reason | Solution |
|---|---|---|
| `fatal: not a git repository` | `git init` wasn't run, or you're in the wrong folder | `cd` to the project root, then `git init` |
| `Author identity unknown` | Git name/email not set | `git config --global user.name "Your Name"` and `git config --global user.email "you@example.com"` |
| `error: src refspec main does not match any` | Nothing committed yet | `git add . && git commit -m "..."` first |
| `failed to push some refs ... fetch first` | The remote has commits (for example, a README made on GitHub) | `git pull origin main --rebase`, then `git push` |
| `LF will be replaced by CRLF` warning | Windows line endings | Harmless. Optionally: `git config --global core.autocrlf true` |
| Accidentally committed `.venv/` | `.gitignore` was missing | `git rm -r --cached .venv && git commit -m "chore: untrack venv"` |

## 8. GitHub upload errors

| Error | Reason | Solution |
|---|---|---|
| `Authentication failed` / password rejected | GitHub no longer accepts account passwords | Use GitHub CLI (`gh auth login`) or a Personal Access Token as the password |
| `remote: error: File ... is 120 MB; this exceeds GitHub's file size limit` | Large video in history | Remove it: `git rm --cached file.mp4`, commit, and if needed rewrite history. Upload videos to **Releases** instead |
| GIF not showing in README | Wrong relative path or case-sensitive name | Paths are case-sensitive on GitHub: `assets/gifs/warehouse_demo.gif` |
| `remote origin already exists` | The remote was added twice | `git remote set-url origin https://github.com/<user>/<repo>.git` |
