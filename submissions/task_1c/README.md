# Task 1C — Position (X / Pitch, Y / Roll) PID

**Deliverable:** `KD_1298_task_1c.zip` · **Script:** `swift_pico/src/task_1c_controller.py`

## What it does

Holds the drone at a target position in the arena by controlling pitch and roll,
while **retaining the Task 1B throttle tuning unchanged**.

## Interfaces

| Role | Topic | Type |
|---|---|---|
| Pose feedback | `/whycode_node/markers` | `whycode_interfaces/msg/MarkerArray` |
| Gain input | `/pitch_pid`, `/roll_pid` | `controller_msg/msg/PIDTune` |
| Error output | `/pos_error` | `error_msg/msg/Error` (`roll_error`, `pitch_error`, `throttle_error`) |
| Command output | `/drone_command` | `swift_msgs/msg/SwiftMsgs` (`rc_roll`, `rc_pitch`, `rc_throttle`) |

## The key architectural finding

`rc_pitch` and `rc_roll` on this vehicle are **attitude** commands, not thrust.
An open-loop probe with a large RC offset moved the drone only ~0.007 m — it
*tilted* rather than translated. So position control must be a **cascade**:

```
position error  ──►  desired tilt (outer PID)  ──►  rc_pitch / rc_roll
                                                       │
                                            inner attitude loop (vehicle)
```

The tilt→RC conversion matches the reference implementation
(`pico_controller_LQR.py::linear_map`):

```
rc = 1500 + (1000/π) · tilt        tilt clamped to ±π/12
```

The axis map and signs were **established empirically in simulation**, not
assumed: `rc_pitch` drives x and `rc_roll` drives y, and with
`error = desired − measured` a negative error drives the matching RC channel
below neutral.

## Gains

Outer position loop (both axes):

```text
Kp = 0.12   Ki = 0.02   Kd = 0.10
```

Throttle axis — **Task 1B gains preserved verbatim**: `Kp = 35, Ki = 2, Kd = 240`
with the 0.035 altitude low-pass filter.

## Recorded performance (`rosbag/`)

Bag duration **89.69 s**; both required topics present.

| Axis | Max abs error | Last 10 s max | Last 10 s mean |
|---|---|---|---|
| x (`roll_error`) | 0.017 | 0.014 | +0.001 |
| y (`pitch_error`) | 0.018 | 0.012 | −0.004 |
| z (`throttle_error`) | 0.337 | 0.318 | −0.177 |

**All three axes held inside ±0.4 for the entire 89.7 s recording** (requirement ≥ 10 s).

## Robustness features (carried over from Task 1B)

* **Marker-loss failsafe** — if no WhyCode marker arrives for 0.5 s, the
  integrator is frozen and a descent is commanded so the drone falls back into
  the camera's field of view instead of integrating stale data.
* **Depth envelope clamp** — the drone can never legitimately be nearer than
  ~0.5 m to the camera, so out-of-envelope readings are rejected.
* **Integral clamp + conditional integration** to prevent windup.
* **Low-pass filters on both the horizontal and vertical measurements** before
  differentiation, which is what makes the derivative term usable on this
  sensor.

## Verification note

Ground truth from `/rotors/odometry` shows the drone flies **level to ±0.06°**
and holds position to ±0.02 m. The ±10° wobble visible in the camera overlay is
*WhyCode orientation-estimation noise*, not real aircraft motion.
