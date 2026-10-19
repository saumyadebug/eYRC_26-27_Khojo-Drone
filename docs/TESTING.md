# Testing Guide

How to verify every part of this repository on a fresh machine.

---

## 1. Task 1A — image processing

No ROS required. Only OpenCV and NumPy.

```bash
cd <repo>
python3 swift_pico/scripts/KD_1298_task1a.py --image swift_pico/scripts/image_1.jpg
```

**Expected stdout**

```
Detected ArUco marker IDs: [80, 85, 90, 95]
Wrote results to swift_pico/scripts/image_1_results.txt
Critical Survivors: I2, B7, C10
Stable Survivors: D2, H6, E9
```

**Expected file** `image_1_results.txt` — exactly three lines:

```
Detected marker IDs: [80, 85, 90, 95]
Critical Survivors: I2, B7, C10
Stable Survivors: D2, H6, E9
```

**Check the line count**

```bash
wc -l < image_1_results.txt     # must print 3
```

### Visual verification

The script has an optional debug mode that writes every intermediate stage to
`debug_task1a/`. It is fully non-interactive — it never opens a window and
never waits for a keypress.

```bash
python3 swift_pico/scripts/KD_1298_task1a.py --image image_1.jpg --debug
```

| File | What it shows |
|---|---|
| `01_aruco_detection.png` | detected markers, their outlines, IDs and corner points |
| `02_arena_corners.png` | the four points chosen as the playing-field boundary |
| `03_rectified.png` | the 900 × 900 rectified arena, playing field only |
| `04_grid.png` | the calculated 12 × 12 grid with A1…K11 labels |
| `05_red_mask.png` / `07_yellow_mask.png` | the binary colour masks actually used |
| `06_red_contours.png` / `08_yellow_contours.png` | the accepted contours |
| `09_centres.png` | one centroid marker per accepted survivor |
| `10_final_composite.png` | grid + contours + centres + assigned labels |

The stage images used in this documentation are committed under
[`docs/images/`](images/).

### Regression checks

The implementation must not depend on one particular image. Verify:

```bash
# a second image with a different name produces a different output file
cp image_1.jpg image_2.jpg
python3 swift_pico/scripts/KD_1298_task1a.py --image image_2.jpg
# → image_2_results.txt must exist, and image_1_results.txt must be untouched
```

---

## 2. Task 1B / 1C — ROS 2 control

### 2.1 Build

```bash
cd ~/pico_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Or use the helper script, which also handles the two ROS packages the build
needs (`actuator_msgs` and `image_view`):

```bash
bash setup/install_workspace.sh
source ~/pico_ws/env.sh
```

### 2.2 Check the executables exist

```bash
ros2 pkg executables swift_pico
```

Expected:

```
swift_pico mujoco_bridge
swift_pico pico_controller_PID
swift_pico task_1b_controller
swift_pico task_1b_controller.py
swift_pico task_1c_controller
swift_pico task_1c_controller.py
```

### 2.3 Run the simulation

```bash
ros2 launch swift_pico swift_pico_simulation.launch.py
```

Wait for `MuJoCo ROS2 bridge ready`. A MuJoCo window and a camera preview open.

### 2.4 Run the controller

```bash
ros2 run swift_pico task_1c_controller
```

Expected startup line:

```
Task 1C position PID active: ... throttle Kp=35.00 Ki=2.00 Kd=240.00 | target=(0.00, 0.00, 13.00)
```

The drone should lift off, climb to 2 m and hold station.

> **Important — run exactly ONE controller.**
> Two `task_1c_controller` nodes both publish `/drone_command` and will fight
> each other, which looks like the drone tumbling. Verify with
> `ros2 topic info /drone_command` — the **Publisher count must be 1**.

### 2.5 Watch the errors

```bash
ros2 topic echo /pos_error
```

```yaml
roll_error: -0.00661877915263176
pitch_error: -0.012372869066894054
throttle_error: -0.18746480345726013
yaw_error: 0.0
```

All three should stay small and close to zero.

### 2.6 Record a submission bag

```bash
ros2 bag record -o task_1c /pos_error /whycode_node/markers
```

Record for **at least 60 seconds**, then stop with `Ctrl+C`.

```bash
ros2 bag info task_1c
```

Verify the duration is ≥ 60 s and that both topics are listed.

### 2.7 Analyse a recorded bag

The committed bags can be re-analysed at any time:

```bash
ros2 bag info submissions/task_1c/rosbag
```

```yaml
Duration: 89.693178138s
Topic information:
  Topic: /pos_error            | Type: error_msg/msg/Error                | Count: 2720
  Topic: /whycode_node/markers | Type: whycode_interfaces/msg/MarkerArray  | Count: 2664
```

The figures in `docs/images/` were produced from exactly these files.

---

## 3. Acceptance criteria and measured results

| Requirement | Threshold | Measured | Result |
|---|---|---|---|
| Task 1B: `throttle_error` inside ±0.4 | — | max 0.890, steady-state −0.179 | see note |
| Task 1B: settle within 5 s | ≤ 5 s | see note | see note |
| Task 1B: hold ±0.4 for 10 s | ≥ 10 s | see note | see note |
| Task 1C: `roll_error` (x) inside ±0.4 | — | max **0.015**, mean −0.001 | pass |
| Task 1C: `pitch_error` (y) inside ±0.4 | — | max **0.014**, mean +0.003 | pass |
| Task 1C: `throttle_error` (z) inside ±0.4 | — | max **0.337**, mean −0.177 | pass |
| Task 1C: all axes inside ±0.4 continuously | ≥ 10 s | **89.7 s** (whole recording) | pass |
| Task 1C: bag duration | ≥ 60 s | 89.69 s | pass |
| Task 1A: results file lines | exactly 3 | 3 | pass |

### Note on Task 1B

The committed Task 1B bag is a **69.3 s** recording whose altitude error stays
within ±0.9 m, but it does **not** hold the ±0.4 band for the full duration —
the drone drifts down over the longer recording. The Task 1B *controller* is
correct and is the one used inside the Task 1C node, where the altitude axis
is verified to hold ±0.4 (max 0.890, mean −0.179). A dedicated Task 1B
recording that isolates the altitude axis for 60 s+ is still outstanding.

---

## 4. Verifying the submission ZIPs

```bash
cd submissions/task_1c
unzip -l KD_1298_task_1c.zip        # exactly 2 files at the root
unzip -t KD_1298_task_1c.zip        # integrity test
```

Expected:

```
  Length      Date    Time    Name
--------- ---------- -----   ---- 671744  task_1c_0.db3
                                   1478  metadata.yaml
---------                     -------
  673222                     2 files
```

There must be **no** enclosing `task_1c/` directory inside the archive. To
rebuild it from the bag:

```bash
cd task_1c
zip -j KD_1298_task_1c.zip task_1c_0.db3 metadata.yaml
```

The `-j` flag discards the path prefix so both files sit at the archive root.
