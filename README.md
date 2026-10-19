# eYRC26-27 Khoj-o-Drone Theme

Stage 1 deliverables for the **Swift Pico** drone.

## Theme Developers

- Joel Siby
- Jones Rogan D
- Ajeet Jain

## Stage 1 — Task 1A

Arena survivor detection: ArUco-based arena localisation, 900×900 perspective
rectification, 12×12 grid reconstruction, HSV red/yellow segmentation, and
nearest-intersection labelling.

| | |
|---|---|
| Script | [`swift_pico/scripts/KD_1298_task1a.py`](swift_pico/scripts/KD_1298_task1a.py) |
| Submission | [`submissions/task_1a/KD_1298_task_1a.zip`](submissions/task_1a/) |
| Docs | [submissions/task_1a/README.md](submissions/task_1a/README.md) |

**Run:**

```bash
python3 swift_pico/scripts/KD_1298_task1a.py --image image_1.jpg
# → image_1_results.txt
```

## Stage 1 — Task 1B

Altitude (Z / throttle) PID for the Swift Pico. The non-obvious part is the
coordinate convention: the WhyCode camera looks down from z = 15, so the
reported `z` is *depth below camera*, and larger values mean the drone is
**lower**.

| | |
|---|---|
| Script | [`swift_pico/src/task_1b_controller.py`](swift_pico/src/task_1b_controller.py) |
| Submission | [`submissions/task_1b/KD_1298_task_1b.zip`](submissions/task_1b/) |
| Docs | [submissions/task_1b/README.md](submissions/task_1b/README.md) |

**Run:**

```bash
ros2 run swift_pico task_1b_controller
```

## Stage 1 — Task 1C

Position (X / pitch, Y / roll) PID, built on the Task 1B altitude loop. Since
`rc_pitch`/`rc_roll` are *attitude* commands, position control is a cascade:
position error → desired tilt → RC, with the vehicle closing the inner
attitude loop.

| | |
|---|---|
| Script | [`swift_pico/src/task_1c_controller.py`](swift_pico/src/task_1c_controller.py) |
| Submission | [`submissions/task_1c/KD_1298_task_1c.zip`](submissions/task_1c/) |
| Docs | [submissions/task_1c/README.md](submissions/task_1c/README.md) |

**Run:**

```bash
ros2 run swift_pico task_1c_controller
```

## Repository layout

```
swift_pico/
  scripts/      Task 1A image-processing script
  src/          ROS 2 nodes (Task 1B / 1C controllers, reference controllers)
  launch/       simulation launch files
  config/       MuJoCo bridge + FastDDS profile
submissions/
  task_1a/      Task 1A deliverable ZIP + docs
  task_1b/      Task 1B deliverable ZIP + ROS 2 bag + docs
  task_1c/      Task 1C deliverable ZIP + ROS 2 bag + docs
```

## Building and running the simulation

```bash
cd ~/pico_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch swift_pico swift_pico_simulation.launch.py   # terminal 1
ros2 run swift_pico task_1c_controller                    # terminal 2
ros2 topic echo /pos_error                               # terminal 3
```

Recording a submission bag:

```bash
ros2 bag record -o task_1c /pos_error /whycode_node/markers
```

## Engineering notes

Everything here was derived from the simulator and the repository itself rather
than assumed:

* **Plant model from real simulator parameters** — mass, `max_thrust` and
  `thrust_weight_offset` were used to compute the throttle loop's natural
  frequency and damping, which is how the Task 1B gains were chosen.
* **Sensor noise characterised, not guessed** — WhyCode depth noise implies
  ~2.9 m/s velocity noise; low-passing the measurement before differentiating
  is what turned a bang-bang limit cycle into a stable hover.
* **Control directions measured in simulation** — the pitch/roll axis map and
  signs were established with open-loop probes, and the cascade architecture
  came from discovering that RC channels command attitude, not translation.
* **No hard-coded results.** Every threshold, gain and label is either
  task-defined geometry or a value that was measured and then justified.
