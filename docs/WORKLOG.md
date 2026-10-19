# Worklog

A chronological record of what was built, what went wrong, and how each
problem was diagnosed. This is the part of the project that is easiest to
under-document and the most useful to read.

---

## Task 1A — arena survivor detection

### Built
An OpenCV pipeline that finds the four ArUco markers (80, 85, 90, 95), rectifies
the playing field to 900 × 900, reconstructs the 12 × 12 grid, segments red and
yellow survivors in HSV, and labels each survivor with the nearest grid
intersection.

### Problems solved

**Which marker corners define the field?**
The markers sit *outside* the playing field, so using their outer corners
would have included the markers in the output. The fix: take the corner of
each marker nearest the centroid of all four marker centres — the inner corner
that touches the field.

**Is the derived grid really on the painted grid?**
Rather than assume, the white line positions were measured in the rectified
image and compared with the calculated ones:

```
calculated : [ 75, 150, 225, 300, 375, 450, 525, 600, 675, 750, 825]
painted    : [ 76, 150, 226, 300, 375, 450, 524, 600, 674, 748, 823]
```

Agreement within ~2 px across the whole field.

**Does it generalise?**
Tested on a second synthetic arena with different survivor positions, on a
rotated and perspective-warped arena, and on a zero-survivor arena. Each
produced correctly re-derived labels and correctly named output files. No
survivor count, coordinate or label is hard-coded.

---

## Task 1B — altitude PID

### Built
A ROS 2 node that holds the drone at an altitude setpoint on the throttle axis
only, with integral clamping, conditional integration, output saturation and a
filtered derivative.

### Problems solved

**The coordinate convention is inverted from what it looks like.**
The WhyCode camera looks straight down from z = 15, so `position.z` is the
*depth below the camera*. A larger value means the drone is **lower**, which
means the throttle output has to be **subtracted** from neutral. Getting this
backwards sends the drone the wrong way immediately.

**The gains were far too aggressive at first.**
An initial `Kp = 120` with a 2 m error commanded 240 counts — about 48 % excess
thrust — and the drone climbed past the camera and away to 22 km. Replaced with
gains derived from the simulator's real thrust parameters.

**Why did it oscillate even at moderate gains?**
Measuring the raw sensor explained it:

```
WhyCode depth jumps ~0.027 m between samples at ~133 Hz
→ implied velocity noise: std 2.9 m/s, peaks > 40 m/s
```

The derivative term amplified that noise into full-scale throttle swings. The
fix — low-pass the *measurement* before differentiating — took peak overshoot
from 17.6 m down to 2.9 m.

**A stale-data runaway.**
If the WhyCode marker leaves the field of view, `markers[]` arrives empty, the
altitude silently keeps its last value, and the controller keeps commanding
climb while blind. A marker-loss failsafe now freezes the integrator and
commands a descent so the drone falls back into view.

**The tuner's scaling is applied twice in the stock template.**
The GUI publishes `integer × multiplier`; the template then multiplies by
`0.03` again, so a maximum GUI value yields an effective `Kp` of just 4.5.
These controllers apply the tuner value once.

### Outstanding
The committed 1B bag records a run that does not hold ±0.4 for its full
duration (see `docs/TESTING.md`). The controller itself is correct — the same
altitude loop holds ±0.4 inside the Task 1C node. A dedicated 1B recording is
still to be produced.

---

## Task 1C — position PID

### Built
A ROS 2 node controlling x and y through pitch and roll while keeping the
Task 1B altitude gains untouched, and publishing `roll_error`, `pitch_error`
and `throttle_error`.

### Problems solved

**RC channels command attitude, not thrust.**
An open-loop probe with a 100-count offset moved the drone only 0.007 m. The
vehicle tilts, and its inner attitude loop does the translating. Horizontal
control therefore had to become a **cascade**: position error → desired tilt
→ RC, using the same `rc = 1500 + (1000/π)·tilt` mapping the reference LQR
controller uses.

**The axis map and signs.**
The first two attempts had pitch and roll swapped, which created positive
feedback: the drone accelerated away from the target, left the camera's
8.66 m field of view, lost its marker, and the stale-altitude runaway
compounded it. The mapping was then pinned down by holding one axis at a time
— `rc_roll` drives y and holds to 0.014 m, `rc_pitch` drives x and holds to
0.015 m.

### The trap that cost the most time
Several controller instances were left running at once during development.
Two nodes publishing `/drone_command` fight each other and the drone appears
to tumble, which looks exactly like a control bug. The check that settles it
immediately:

```bash
ros2 topic info /drone_command     # Publisher count must be 1
```

### Result
All three axes inside ±0.4 for the entire 89.7 s recording.

---

## Cross-cutting lessons

1. **Measure the plant, don't guess the gains.** The thrust slope came straight
   out of `swift_trans.cpp` and the mass out of `drone.xml`.
2. **Characterise the sensor noise.** It was the dominant error source and
   was invisible until the raw signal was measured.
3. **Verify control directions empirically.** A plausible sign convention is
   not a measurement.
4. **Keep the data.** The ROS 2 bags are committed so every number in the
   documentation can be re-checked rather than taken on trust.
5. **One controller at a time.** The most confusing bug in the whole project
   was process duplication, not control theory.
