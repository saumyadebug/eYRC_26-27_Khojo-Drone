# Task 1A — Arena Survivor Detection

**Deliverable:** `KD_1298_task_1a.zip` · **Script:** `swift_pico/scripts/KD_1298_task1a.py`

## What it does

Detects red (**Critical**) and yellow (**Stable**) survivors in an arena image and
reports the grid intersection each survivor occupies.

## Pipeline

```
input image
   │
   ├─ ArUco detection (4x4_250 dict, IDs 80/85/90/95)
   │     └─ inner marker corners → playing-field boundary
   │        (markers sit OUTSIDE the field, so inner corners are used)
   │
   ├─ cv2.getPerspectiveTransform + cv2.warpPerspective → 900×900 top-down arena
   │
   ├─ grid derived from arena geometry: 12 cells over 900 px
   │     └─ 11 interior lines per axis, 121 intersections
   │
   ├─ HSV segmentation (red needs 2 hue ranges for the 0/180 wraparound)
   │     └─ morphological open/close → contour filtering
   │
   ├─ centroid per survivor via cv2.moments()
   │
   └─ nearest intersection by Euclidean distance → label A1…K11
```

## Results file

`<image_stem>_results.txt`, exactly three lines:

```
Detected marker IDs: [80, 85, 90, 95]
Critical Survivors: I2, B7, C10
Stable Survivors: D2, H6, E9
```

## Design notes

* **No hard-coded answers.** Every label is computed from the image at run time;
  the only constants are task-defined geometry (marker IDs, 900×900, 12×12 grid,
  A–K / 1–11 labels) and HSV thresholds.
* **Grid alignment verified against the painted grid**, not assumed — measured
  painted-line positions in the rectified arena matched the calculated
  intersections to within ~2 px.
* **Generalises across images**: verified on a second arena with different
  survivor positions and on a rotated/warped arena, producing correctly
  re-derived labels and auto-derived output filenames.
