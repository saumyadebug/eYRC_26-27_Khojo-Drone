# System Architecture

This document describes how the three Stage 1 tasks fit together, what runs
where, and how data flows through the system.

---

## 1. Big picture

```
                       ┌──────────────────────────────────────────────┐
                       │        Swift Pico simulation (MuJoCo)        │
                       │  drone dynamics · arena · overhead camera    │
                       └───────────────┬──────────────────────────────┘
                                       │  shared-memory image ring
                                       │  (30 Hz, 1280x1280, 'swift_pico_image_raw')
                                       ▼
                       ┌──────────────────────────────────────────────┐
                       │            whycode_node                      │
                       │  detects the WhyCode marker on the drone      │
                       │  → /whycode_node/markers  (pose)             │
                       └───────────────┬──────────────────────────────┘
                                       │  MarkerArray  (x, y, z, r, p, yaw)
                                       ▼
   ┌────────────────────────────────────────────────────────────────────┐
   │                        ROS 2 control graph                        │
   │                                                                    │
   │   ┌──────────────┐   /throttle_pid   ┌────────────────────────┐  │
   │   │  pid_tune    │──PIDTune──────────►│                        │  │
   │   │  (GUI tuner) │──PIDTune──────────►│   task_1c_controller   │  │
   │   │  Pitch/Roll/ │──PIDTune──────────►│   (Task 1B + 1C)      │  │
   │   │  Throttle    │                   │                        │  │
   │   └──────────────┘                   │  /pos_error (Error)    │──┼──► tuner graph
   │                                      │  /drone_command        │  │
   │                                      └───────────┬────────────┘  │
   └──────────────────────────────────────────────────┼───────────────┘
                                                      │ SwiftMsgs
                                                      │ (rc_roll, rc_pitch,
                                                      │  rc_throttle, rc_aux4)
                                                      ▼
                       ┌──────────────────────────────────────────────┐
                       │      rotors_swift_interface                   │
                       │  RC stick values → thrust.z                   │
                       │  thrust.z = (1 + (rc-1500)/500) · weight     │
                       └───────────────┬──────────────────────────────┘
                                       │ /rotors/command/... (thrust)
                                       ▼
                       ┌──────────────────────────────────────────────┐
                       │      roll_pitch_yawrate_thrust_controller    │
                       └───────────────┬──────────────────────────────┘
                                       ▼
                                  MuJoCo physics
```

---

## 2. The two feedback paths

The system has two independent sensing paths, and confusing them is the single
biggest source of bugs in this project.

### 2.1 Position feedback — the WhyCode marker

A WhyCode marker is mounted on the drone. The overhead camera (at z = 15,
looking straight down) tracks it and publishes a pose on
`/whycode_node/markers`.

```
markers[0].position.position.x   ->  world x            [m]
markers[0].position.position.y   ->  world y            [m]
markers[0].position.position.z   ->  DEPTH below camera [m]   <-- note!
```

**`z` is not height.** The camera is at 15 m, so:

```
whycode_z  =  15 (camera height)  −  world_height
```

A **larger** WhyCode `z` means the drone is **lower**. This is why the
throttle controller *subtracts* its output from neutral throttle, and why the
altitude setpoint of `13.0` corresponds to a hover **2 m** above the floor.

### 2.2 Attitude and thrust — the RC interface

`rc_pitch` and `rc_roll` are **attitude** commands, not thrusts. They are
converted to a desired tilt angle by the reference implementation
(`pico_controller_LQR.py::linear_map`):

```
rc = 1500 + (1000/π) · tilt        tilt clamped to ±π/12 (15°)
```

An open-loop probe with a large RC offset moved the drone only ~0.007 m: it
*tilted* rather than translated. Horizontal position control therefore has to
be a **cascade**:

```
position error  ──►  desired tilt (outer PID)  ──►  rc_pitch / rc_roll
                                                       │
                                            inner attitude loop (vehicle)
```

The throttle axis is different: `rc_throttle` maps linearly to vertical thrust.

---

## 3. Task-by-task architecture

### Task 1A — offline image processing (no ROS)

A standalone Python/OpenCV script. It is deliberately independent of the
simulation so it can be run on any arena photograph.

```
image_1.jpg
   │
   ├─ ArUco detection (DICT_4X4_250, IDs 80 / 85 / 90 / 95)
   │     └─ take each marker's INNER corner (nearest the arena centroid)
   │        → the four playing-field corners; markers are outside the field
   │
   ├─ cv2.getPerspectiveTransform  +  cv2.warpPerspective
   │     └─ 900 × 900 top-down rectified arena
   │
   ├─ grid from arena geometry: 12 cells over 900 px
   │     └─ 11 interior lines per axis → 121 intersections
   │
   ├─ HSV segmentation
   │     ├─ red    : two hue ranges (wraparound at 0/180)
   │     └─ yellow : one hue range
   │     └─ morphological open/close, then contour filtering
   │
   ├─ centroid per survivor via cv2.moments()
   │
   └─ nearest of the 121 intersections (Euclidean) → label A1…K11
         └─ image_1_results.txt  (exactly 3 lines)
```

### Task 1B — altitude PID (ROS 2 node)

```
/whycode_node/markers ──► filtered altitude ──► error = 13.0 − altitude
                                                        │
                              ┌─────────────────────────┼──────────┐
                              │                         │          │
                            P = Kp·e              I = Ki·∫e   D = Kd·(de/dt)
                              │                         │          │
                              └──────────► out = −(P+I+D) ◄───────┘
                                                  │
                                    rc_throttle = 1500 + out   (clamped 1000–2000)
```

### Task 1C — position PID (ROS 2 node)

Same node, three parallel PID loops:

| Axis | Loop | Output | Error field |
|---|---|---|---|
| x | outer position → tilt | `rc_pitch` | `roll_error` |
| y | outer position → tilt | `rc_roll` | `pitch_error` |
| z | altitude (Task 1B, unchanged) | `rc_throttle` | `throttle_error` |

---

## 4. Message and topic contract

| Direction | Topic | Type | Fields used |
|---|---|---|---|
| sim → ctrl | `/whycode_node/markers` | `whycode_interfaces/msg/MarkerArray` | `markers[0].position.position.{x,y,z}` |
| tuner → ctrl | `/throttle_pid` | `controller_msg/msg/PIDTune` | `kp`, `ki`, `kd` |
| tuner → ctrl | `/pitch_pid` | `controller_msg/msg/PIDTune` | `kp`, `ki`, `kd` |
| tuner → ctrl | `/roll_pid` | `controller_msg/msg/PIDTune` | `kp`, `ki`, `kd` |
| ctrl → tuner | `/pos_error` | `error_msg/msg/Error` | `roll_error`, `pitch_error`, `throttle_error`, `yaw_error` |
| ctrl → sim | `/drone_command` | `swift_msgs/msg/SwiftMsgs` | `rc_roll`, `rc_pitch`, `rc_throttle`, `rc_aux4` (arm) |
| ctrl → tuner | `/controller_heartbeat` | `std_msgs/msg/Header` | `frame_id` |

### The PID tuner's gain scaling

The tuner GUI shows **integers**, but the value published is:

```
published_gain = gui_integer × gui_multiplier
MULT_DEFAULTS = { 'Kp': 0.03, 'Ki': 0.008, 'Kd': 0.6 }
```

So a GUI `Kp` of 10 is a **real** gain of 0.30. Always report the multiplied
value, never the integer.

> The stock template controller applied a *second* factor on top of that
> (`Kp = alt.kp * 0.03`), which attenuated the effective gains by two extra
> orders of magnitude. These controllers apply the tuner value **once**.

---

## 5. Safety and robustness features

Both controllers implement the same defensive behaviour:

| Feature | Why |
|---|---|
| **Marker-loss failsafe** | If no marker for 0.5 s, freeze the integrator and command a descent, so the drone falls back into the camera's view instead of integrating stale data. Without this the drone climbed to 22 km in simulation. |
| **Depth envelope clamp** | The drone can never legitimately be nearer than ~0.5 m to the camera (depth ≤ 14.5 m). Out-of-envelope readings are rejected. |
| **Integral clamp** | The stored integral is bounded, so windup cannot grow without limit. |
| **Conditional integration** | If the command is saturated and the error would push it further into saturation, the integrator is unwound. |
| **Output saturation** | The final command is clamped to the valid RC range 1000–2000. |
| **Measurement low-pass** | Applied to altitude *and* to the horizontal axes before differentiating. |
| **Zero-area contour guard** | `cv2.moments()` with `m00 == 0` returns `None` instead of dividing by zero. |

---

## 6. Where each piece of code lives

```
swift_pico/
├── scripts/
│   └── KD_1298_task1a.py        Task 1A  (pure OpenCV, no ROS)
├── src/
│   ├── task_1b_controller.py    Task 1B  (altitude PID)
│   ├── task_1b_controller       shell wrapper so `ros2 run` finds it
│   ├── task_1c_controller.py    Task 1C  (position PID, keeps the 1B altitude loop)
│   ├── task_1c_controller       shell wrapper
│   ├── pico_controller_PID.py   upstream template (kept for reference)
│   └── pico_controller_LQR.py   reference attitude/thrust mapping
├── launch/
│   └── swift_pico_simulation.launch.py
└── CMakeLists.txt               installs both controllers as ROS 2 executables
```
