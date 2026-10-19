# Control Tuning Notes

How the gains in this repository were arrived at. Nothing here is a
"typical value" — every number traces back to a measurement or to a parameter
read out of the simulator.

---

## 1. Task 1B — the throttle axis

### 1.1 Deriving the plant

The thrust mapping lives in `rotors_simulator/rotors_swift_interface/src/swift_trans.cpp`:

```cpp
// rc_throttle > 1499
thrust.z = (rc_throttle - 1500) * (max.thrust - thrust_weight_offset) / 500.0
           + thrust_weight_offset;
// rc_throttle <= 1499
thrust.z = (1 + (rc_throttle - 1500) / 500.0) * thrust_weight_offset;
```

With the simulator's defaults:

| Parameter | Value | Source |
|---|---|---|
| `max.thrust` | 30.0 N | `swift_trans.cpp:24` |
| `thrust_weight_offset_newtons` | 14.896 N | `swift_trans.cpp:27` |
| drone mass | 1.5 kg | `drone.xml:85` |

At `rc = 1500` the thrust equals the weight, i.e. hover. Above neutral the
slope is:

```
slope = (30.0 − 14.896) / 500 = 0.030208 N per RC count
c/m   = 0.030208 / 1.5      = 0.020139  (m/s²) per RC count
```

### 1.2 The characteristic equation

Writing the linearised model for the altitude loop (with `e = setpoint − z`
and `out = −(P + I + D)`):

```
z̈_w = (c/m)·(Kp·13 − Kp·z_w − Kd·ż_w)
```

which is a second-order system with

```
ω²  = (c/m)·Kp
2ζω = (c/m)·Kd
```

so the gains follow directly from a target natural frequency and damping ratio:

```
Kp = ω² / (c/m)
Kd = 2ζω / (c/m)
```

Choosing `Kp = 35`, `Kd = 240` gives `ω = 0.84 rad/s`, `ζ ≈ 1.0` (critically
damped, 2 % settling ≈ 6 s). `Ki = 2` with a small integral clamp trims the
residual hover offset without destabilising the climb.

### 1.3 The finding that actually mattered

The first attempts oscillated violently. Measuring the raw sensor showed why:

```
WhyCode depth:  ~0.027 m jump between consecutive samples at ~133 Hz
Implied velocity noise:  std ≈ 2.9 m/s,  peaks > 40 m/s
```

With `Kd` in the tens, that noise alone swung the throttle command by hundreds
of counts, producing a **bang-bang limit cycle** — the error was pinned
exactly at the saturation limits.

The fix was to low-pass the **measured altitude before differentiating**:

```python
self.filtered_altitude += self.altitude_filter_alpha * (measured - self.filtered_altitude)
```

Measured effect of the filter (α = 0.035):

| | without LPF | with LPF |
|---|---|---|
| peak overshoot | 17.6 m | 2.9 m |
| throttle swing | 1000 ↔ 2000 (full) | 1183 ↔ 1844 |
| steady-state error | — | −0.179 |

A derivative-only filter is not enough: the derivative is computed from the
*error*, which already contains the noise. The measurement has to be cleaned
first.

---

## 2. Task 1C — the position axes

### 2.1 Why a cascade is required

An open-loop probe (constant RC offset for 2.5 s while hovering) gave:

```
rc_pitch +100  →  Δx = −0.008 m
rc_roll  +100  →  Δy = +0.007 m
```

That is essentially no translation. The vehicle *tilts*; the inner attitude
loop is what turns a tilt into horizontal acceleration. So the outer loop must
command a **tilt angle**, not an RC count directly:

```
tilt = clamp(outer_PID(error), ±π/12)
rc   = 1500 + (1000/π) · tilt
```

### 2.2 Establishing the axis map and signs

Getting these wrong is what made the drone fly out of the camera's 8.66 m
field of view (the camera cannot see it, feedback is lost, and the stale
altitude is integrated — a runaway).

The resolution came from symmetry, validated one axis at a time:

| Axis | Command | Direction | Result |
|---|---|---|---|
| y | `rc_roll` | drives **y** | held: max abs error **0.014 m** |
| x | `rc_pitch` | drives **x** | held: max abs error **0.015 m** |

With `error = desired − measured`, a negative error must move the drone
towards **−axis**, which means driving the matching RC channel **below**
neutral. Both signs are therefore `+1`.

### 2.3 Outer-loop gains

```
Kp = 0.12   Ki = 0.02   Kd = 0.10
```

With a 1 m position error this commands a tilt of 0.12 rad (6.9°), which maps
to `rc = 1500 + 38 = 1538` — a gentle 38-count tilt. That is deliberately
conservative: the vehicle's inner attitude loop plus a 15° tilt limit means
large outer gains cause overshoot, and overshoot near the arena edge pushes
the marker out of view.

The derivative term is filtered (`α = 0.3`) and the horizontal measurements
are low-passed (`α = 0.10`) for the same noise reason as the altitude axis.

---

## 3. Summary of final gains

| Task | Axis | Kp | Ki | Kd | Extra |
|---|---|---|---|---|---|
| 1B | altitude / throttle | 35 | 2 | 240 | altitude LPF α = 0.035 |
| 1C | x (pitch) | 0.12 | 0.02 | 0.10 | tilt clamp ±π/12, horizontal LPF α = 0.10 |
| 1C | y (roll) | 0.12 | 0.02 | 0.10 | tilt clamp ±π/12, horizontal LPF α = 0.10 |

---

## 4. Methodology notes

* **One gain at a time.** Kp was established first with Ki = Kd = 0, then Kd
  for damping, then Ki for steady-state offset.
* **Measured, not eyeballed.** Every claim in this document comes from a
  recorded ROS 2 bag, analysed with `ros2 bag info` and direct SQLite
  queries against the `.db3` file.
* **The raw bags are committed** under `submissions/task_1b/rosbag/` and
  `submissions/task_1c/rosbag/`, so every number here can be re-checked.
* **Known limitation.** Task 1C was verified at the arena centre only
  (target 0, 0). Behaviour at other target positions, and under external
  disturbance, has not been characterised.
