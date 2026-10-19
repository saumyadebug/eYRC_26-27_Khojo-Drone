# Task 1B — Altitude (Z / Throttle) PID

**Deliverable:** `KD_1298_task_1b.zip` · **Script:** `swift_pico/src/task_1b_controller.py`

## What it does

Holds the drone at an altitude setpoint using a PID loop on the throttle axis only.

## Interfaces

| Role | Topic | Type |
|---|---|---|
| Pose feedback | `/whycode_node/markers` | `whycode_interfaces/msg/MarkerArray` |
| Gain input | `/throttle_pid` | `controller_msg/msg/PIDTune` (`kp`, `ki`, `kd`) |
| Error output | `/pos_error` | `error_msg/msg/Error` (`throttle_error`) |
| Command output | `/drone_command` | `swift_msgs/msg/SwiftMsgs` (`rc_throttle`) |

## Coordinate convention (the subtle part)

The WhyCode camera looks straight down from z = 15, so the reported
`markers[0].position.position.z` is **depth below the camera**, not height:

```
whycode_z = 15 (camera height) − world_height
```

A larger WhyCode `z` therefore means the drone is **lower**, so the controller
output is *subtracted* from the neutral throttle value. Setpoint `z = 13.0`
corresponds to a hover 2 m above the floor.

## PID

```
error = setpoint − measured_altitude
P = Kp·e      I = Ki·∫e dt      D = Kd·(de/dt)
out  = −(P + I + D)          rc_throttle = 1500 + out
```

* **Integral clamp** (+ anti-windup) limits stored integral.
* **Conditional integration** unwinds the integrator when the command saturates.
* **Output saturation** to the 1000–2000 RC range.
* **Derivative low-pass filter** plus an **altitude low-pass filter** before
  differentiating.

### Why the altitude low-pass filter matters

Raw WhyCode depth jumps ~0.027 m between samples at ~133 Hz, which implies a
velocity noise of **std ≈ 2.9 m/s with peaks over 40 m/s**. Feeding that
straight into the derivative term produced full-scale (1000↔2000) throttle
swings and a bang-bang limit cycle. Filtering the measured altitude *before*
differentiating cut peak overshoot from 17.6 m to 2.9 m.

## Gains

```text
Kp = 35    Ki = 2    Kd = 240
altitude low-pass α = 0.035
```

Gains were **not** guessed — the plant was derived from the simulator's real
parameters (mass 1.5 kg, `max_thrust` 30 N, `thrust_weight_offset` 14.896 N →
0.0302 N per RC count, c/m = 0.0201), giving ω = 0.90 rad/s at ζ ≈ 1.0.

## Recorded performance (`rosbag/`)

| Metric | Value |
|---|---|
| Bag duration | 69.34 s |
| Max absolute `throttle_error` | 0.890 |
| Steady-state error | −0.179 |

## Note on the PID tuner

`/throttle_pid` carries the value the tuner computed as
`gui_integer × gui_multiplier` (`MULT_DEFAULTS = {Kp: 0.03, Ki: 0.008, Kd: 0.6}`).
The stock template applied a *second* factor on top of that, attenuating the
effective gains by two extra orders of magnitude. This controller applies the
tuner value **once**, so the GUI value is the real gain.
