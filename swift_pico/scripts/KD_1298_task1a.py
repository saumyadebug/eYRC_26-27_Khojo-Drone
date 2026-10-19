#!/usr/bin/env python3
"""eYRC Khojo Drone Task 1A.

Detects red (Critical) and yellow (Stable) survivors inside an arena using
ArUco markers 80, 85, 90 and 95 for localisation, rectifies the playing field
to a true top-down 900x900 image, matches every survivor to the nearest of the
121 grid intersections (columns A-K left to right, rows 1-11 top to bottom),
and writes a three line results file named after the input image.

Usage:
    python3 KD_1298_task1a.py --image <path-to-arena-image>
    python3 KD_1298_task1a.py --image <path-to-arena-image> --debug

Only OpenCV (cv2) and NumPy are used.  Both modes are fully non-interactive:
no GUI windows, no keypress waits, no interactive prompts.  With --debug the
intermediate stages are written as PNG files into ./debug_task1a/ instead.

Every answer is derived from the input image at run time.  The only constants
are task-defined geometry (marker IDs, 900x900 arena, 12x12 grid, A-K / 1-11
labels) and the HSV / area segmentation thresholds.
"""

import argparse
import os
import sys

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Task-defined constants
# ---------------------------------------------------------------------------

REQUIRED_MARKER_IDS = [80, 85, 90, 95]

ARENA_SIZE = 900                    # rectified arena is exactly 900 x 900 px
GRID_CELLS = 12                     # the arena is a 12 x 12 grid of cells
GRID_INTERSECTIONS = GRID_CELLS - 1  # 11 interior lines in each direction

# Interior grid lines sit at i * ARENA_SIZE / GRID_CELLS, i.e. every 75 px.
GRID_STEP = ARENA_SIZE / float(GRID_CELLS)

# ---------------------------------------------------------------------------
# Segmentation thresholds (configurable, contain no survivor locations)
# ---------------------------------------------------------------------------

# HSV colour ranges (OpenCV hue spans 0-179).  Red wraps around the hue
# discontinuity at 0/180, so it needs two ranges.
RED_RANGES = [
    (np.array([0, 100, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
    (np.array([165, 100, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
]
YELLOW_RANGES = [
    (np.array([18, 90, 90], dtype=np.uint8), np.array([35, 255, 255], dtype=np.uint8)),
]

# Contour filtering, in pixels of the 900 x 900 rectified arena.
MIN_SURVIVOR_AREA = 150.0        # reject noise specks and line fragments
MAX_SURVIVOR_AREA = 0.08 * ARENA_SIZE * ARENA_SIZE  # reject large background tints
MIN_SOLIDITY = 0.55               # contour_area / convex_area; rejects thin slivers
MIN_SIDE_PX = 12.0                # smallest accepted bounding-box side

DEBUG_DIR_NAME = "debug_task1a"


# ---------------------------------------------------------------------------
# Argument handling / image loading
# ---------------------------------------------------------------------------

def parse_arguments(argv=None):
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Detect survivors in an eYRC Khojo arena image."
    )
    parser.add_argument(
        "--image",
        required=True,
        help="Path to the arena image to process.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Save intermediate stage images into ./{} (non-interactive).".format(
            DEBUG_DIR_NAME
        ),
    )
    return parser.parse_args(argv)


def load_image(path):
    """Load an image with OpenCV.  Exit non-zero if it cannot be read."""
    if not os.path.isfile(path):
        print("ERROR: image file does not exist: {}".format(path), file=sys.stderr)
        sys.exit(2)

    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        print("ERROR: could not load image: {}".format(path), file=sys.stderr)
        sys.exit(2)
    return image


# ---------------------------------------------------------------------------
# ArUco detection
# ---------------------------------------------------------------------------

def _get_aruco_dictionary():
    """Return the 4x4 (250) ArUco dictionary."""
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available in this OpenCV build")
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


def detect_aruco_markers(image):
    """Detect markers and return {id: corners as (4, 2) float32}.

    OpenCV 4.5.4 uses the legacy detectMarkers() API; newer OpenCV builds
    expose ArucoDetector instead, so both paths are handled.  The legacy path is
    always exercised on 4.5.4 because ArucoDetector is absent there.
    """
    dictionary = _get_aruco_dictionary()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    detected = {}
    if ids is None:
        return detected
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        detected[int(marker_id)] = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
    return detected


# ---------------------------------------------------------------------------
# Perspective rectification
# ---------------------------------------------------------------------------

def _order_corners(points):
    """Order four points as top-left, top-right, bottom-right, bottom-left.

    Uses the classic sum/difference heuristic, which is robust for arena images
    captured from above with mild perspective and little in-plane rotation.
    """
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = pts[:, 0] - pts[:, 1]

    ordered = np.array(
        [
            pts[np.argmin(sums)],    # top-left:     smallest x + y
            pts[np.argmax(diffs)],   # top-right:    largest  x - y
            pts[np.argmax(sums)],    # bottom-right: largest x + y
            pts[np.argmin(diffs)],   # bottom-left:  smallest  x - y
        ],
        dtype=np.float32,
    )
    return ordered


def get_arena_corners(detected, required_ids=REQUIRED_MARKER_IDS):
    """Return the four playing-field corners plus the detected required ids.

    The four markers sit *outside* the playing field, one per field corner, so
    the marker corner that faces the centre of the arena is the inner corner
    that touches the field.  For every marker that is the corner nearest to the
    centroid of all four marker centres.  Taking those four inner corners (and
    never the outer ones) is what keeps the markers themselves outside the
    rectified 900x900 field.
    """
    present = [mid for mid in required_ids if mid in detected]
    missing = [mid for mid in required_ids if mid not in detected]
    if missing:
        raise RuntimeError(
            "Missing required ArUco marker(s): {}".format(
                ", ".join(str(m) for m in missing)
            )
        )

    centres = np.array([detected[mid].mean(axis=0) for mid in present], dtype=np.float32)
    field_centroid = centres.mean(axis=0)

    inner_corners = []
    for mid in present:
        marker_corners = detected[mid]
        distances = np.linalg.norm(marker_corners - field_centroid, axis=1)
        inner_corners.append(marker_corners[int(np.argmin(distances))])

    return _order_corners(inner_corners), sorted(present)


def rectify_arena(image, src_corners):
    """Warp the playing field defined by src_corners to a 900x900 top-down view."""
    dst = np.array(
        [
            [0, 0],
            [ARENA_SIZE - 1, 0],
            [ARENA_SIZE - 1, ARENA_SIZE - 1],
            [0, ARENA_SIZE - 1],
        ],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(np.asarray(src_corners, dtype=np.float32), dst)
    return cv2.warpPerspective(image, matrix, (ARENA_SIZE, ARENA_SIZE))


# ---------------------------------------------------------------------------
# Grid
# ---------------------------------------------------------------------------

def intersection_label(column, row):
    """Convert a 1-based (column, row) intersection index to its A1-K11 label.

    column: 1..11 -> A..K (left to right)
    row:    1..11 -> 1..11 (top to bottom)
    """
    if not (1 <= column <= GRID_INTERSECTIONS and 1 <= row <= GRID_INTERSECTIONS):
        raise ValueError("intersection index out of range: ({}, {})".format(column, row))
    return "{}{}".format(chr(ord("A") + column - 1), row)


def build_grid(size=ARENA_SIZE, cells=GRID_CELLS):
    """Return (points, labels) for every interior grid intersection.

    Coordinates are computed from the rectified arena geometry: the 12 cells
    span ``size`` pixels, so interior line i is at i * size / 12.

    points: (121, 2) float32 array of (x, y) coordinates
    labels: 121 labels from 'A1' to 'K11', in matching order
    """
    step = size / float(cells)
    points = []
    labels = []
    for row in range(1, cells):
        for column in range(1, cells):
            points.append((column * step, row * step))
            labels.append(intersection_label(column, row))
    return np.array(points, dtype=np.float32), labels


# ---------------------------------------------------------------------------
# Survivor detection
# ---------------------------------------------------------------------------

def _segment_colour(hsv, ranges):
    """Build a cleaned binary mask from a list of (lower, upper) HSV ranges."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask |= cv2.inRange(hsv, lower, upper)

    # Open removes isolated noise, close seals small holes in the survivors.
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    return mask


def calculate_centroid(contour):
    """Return the (cx, cy) centroid of a contour, or None if degenerate.

    Image moments give a centroid that works for both triangles and circles.
    A zero-area contour returns None instead of dividing by zero.
    """
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])


def detect_survivors(rectified):
    """Detect red and yellow survivors in the rectified arena.

    Returns (results, masks) where ``results`` maps colour name to a list of
    dicts with the accepted contour, its area and its centroid, and ``masks``
    maps colour name to the binary mask that produced it.  The number of
    survivors is never assumed.
    """
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    masks = {
        "red": _segment_colour(hsv, RED_RANGES),
        "yellow": _segment_colour(hsv, YELLOW_RANGES),
    }

    results = {"red": [], "yellow": []}
    for name, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_SURVIVOR_AREA or area > MAX_SURVIVOR_AREA:
                continue

            # Geometric rejection of thin slivers such as grid-line fragments.
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area <= 0 or (area / hull_area) < MIN_SOLIDITY:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if min(w, h) < MIN_SIDE_PX:
                continue

            centre = calculate_centroid(contour)
            if centre is None:
                continue

            results[name].append({"contour": contour, "area": area, "centre": centre})

    return results, masks


# ---------------------------------------------------------------------------
# Matching
# ---------------------------------------------------------------------------

def nearest_intersection(centre, grid_points):
    """Return (index, distance) of the grid intersection nearest to centre.

    Selection is purely by Euclidean distance over all 121 intersections, so
    a centre anywhere in the arena (including near an edge) always resolves to
    one valid in-range intersection.
    """
    cx, cy = centre
    distances = np.hypot(grid_points[:, 0] - cx, grid_points[:, 1] - cy)
    index = int(np.argmin(distances))
    return index, float(distances[index])


def assign_nearest_intersection(centre, grid_points, grid_labels):
    """Return the A1-K11 label of the grid intersection nearest to centre."""
    index, _ = nearest_intersection(centre, grid_points)
    return grid_labels[index]


def _sort_labels(labels):
    """De-duplicate and sort labels by row then column, for deterministic output."""
    return sorted(set(labels), key=lambda lab: (int(lab[1:]), lab[0]))


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def results_path_for(image_path):
    """Derive the results filename from the input image path.

    The output is the input file's stem plus '_results.txt', written into the
    input file's own directory, so a bare relative path stays relative.
    """
    directory = os.path.dirname(image_path)
    stem = os.path.splitext(os.path.basename(image_path))[0]
    return os.path.join(directory, stem + "_results.txt")


def write_results(image_path, marker_ids, critical_labels, stable_labels):
    """Write the exactly three line results file, overwriting any existing file."""
    path = results_path_for(image_path)
    lines = [
        "Detected marker IDs: [{}]".format(", ".join(str(m) for m in marker_ids)),
        "Critical Survivors: {}".format(", ".join(critical_labels)),
        "Stable Survivors: {}".format(", ".join(stable_labels)),
    ]
    with open(path, "w") as handle:
        handle.write("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------------------
# Debug visualisation (writes PNG files only, never opens a window)
# ---------------------------------------------------------------------------

def _grid_lines(size=ARENA_SIZE, cells=GRID_CELLS):
    """Interior grid line pixel positions for the rectified arena."""
    step = size / float(cells)
    return [int(round(i * step)) for i in range(1, cells)]


def draw_grid(image, colour=(0, 255, 0), thickness=1, labels=False):
    """Draw the calculated 12x12 grid (11 interior lines per axis) on a copy."""
    debug = image.copy()
    size = image.shape[0]
    for pos in _grid_lines(size):
        cv2.line(debug, (pos, 0), (pos, size - 1), colour, thickness, cv2.LINE_AA)
        cv2.line(debug, (0, pos), (size - 1, pos), colour, thickness, cv2.LINE_AA)
    if labels:
        for row in range(1, GRID_CELLS):
            for column in range(1, GRID_CELLS):
                x = int(round(column * GRID_STEP))
                y = int(round(row * GRID_STEP))
                cv2.putText(debug, intersection_label(column, row), (x + 2, y - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1, cv2.LINE_AA)
    return debug


def measure_painted_lines(rectified):
    """Return detected painted white grid line positions along x and y.

    Used only to verify, in debug mode, that the calculated grid coincides
    with the lines actually painted on the arena.
    """
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 180], np.uint8), np.array([180, 90, 255], np.uint8))
    col_profile = white.sum(axis=0) / 255.0
    row_profile = white.sum(axis=1) / 255.0

    def peaks(profile, threshold):
        idx = np.where(profile > threshold)[0]
        groups = []
        if idx.size == 0:
            return groups
        start = prev = int(idx[0])
        for value in idx[1:]:
            value = int(value)
            if value - prev > 5:
                groups.append(int(round((start + prev) / 2.0)))
                start = value
            prev = value
        groups.append(int(round((start + prev) / 2.0)))
        return groups

    return peaks(col_profile, 300), peaks(row_profile, 300)


def save_debug_images(debug_dir, original, detected, arena_corners, rectified,
                      results, masks, grid_points, grid_labels, assignments):
    """Write every pipeline stage to PNG files.  No GUI, no keypress."""
    os.makedirs(debug_dir, exist_ok=True)
    saved = []

    def save(name, image):
        path = os.path.join(debug_dir, name)
        cv2.imwrite(path, image)
        saved.append(path)

    # 01 - ArUco detection: outlines, IDs and corner points.
    aruco_img = original.copy()
    for marker_id, corners in detected.items():
        pts = corners.reshape(1, 4, 2).astype(np.float32)
        cv2.aruco.drawDetectedMarkers(aruco_img, [pts])
        cv2.polylines(aruco_img, [corners.astype(np.int32)], True, (0, 255, 255), 2)
        for corner in corners.astype(np.int32):
            cv2.circle(aruco_img, tuple(corner), 4, (255, 0, 255), -1)
        centre = corners.mean(axis=0).astype(int)
        cv2.putText(aruco_img, str(marker_id), (int(centre[0]) - 18, int(centre[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    save("01_aruco_detection.png", aruco_img)

    # 02 - the four source points used as the playing-field boundary.
    corners_img = original.copy()
    poly = arena_corners.astype(np.int32)
    cv2.polylines(corners_img, [poly], True, (0, 0, 255), 3)
    names = ["TL", "TR", "BR", "BL"]
    for name, point in zip(names, poly):
        cv2.circle(corners_img, tuple(point), 9, (0, 255, 255), -1)
        cv2.putText(corners_img, name, (int(point[0]) + 10, int(point[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    save("02_arena_corners.png", corners_img)

    # 03 - rectified playing field.
    save("03_rectified.png", rectified)

    # 04 - calculated grid plus intersection labels.
    save("04_grid.png", draw_grid(rectified, labels=True))

    # 05 / 07 - raw colour masks.
    save("05_red_mask.png", masks["red"])
    save("07_yellow_mask.png", masks["yellow"])

    # 06 / 08 - accepted contours drawn on the rectified arena.
    # The outline is black so it stays visible on top of a saturated red or
    # yellow survivor fill.
    for name, filename in (
        ("red", "06_red_contours.png"),
        ("yellow", "08_yellow_contours.png"),
    ):
        canvas = rectified.copy()
        contours = [item["contour"] for item in results[name]]
        cv2.drawContours(canvas, contours, -1, (0, 0, 0), 3)
        cv2.drawContours(canvas, contours, -1, (255, 255, 255), 1)
        save(filename, canvas)

    # 09 - one centroid dot per accepted survivor.  A black ring keeps the
    # centre marker readable against any survivor colour.
    centres_img = draw_grid(rectified)
    for name, colour in (("red", (0, 0, 255)), ("yellow", (0, 165, 255))):
        for item in results[name]:
            cx, cy = int(round(item["centre"][0])), int(round(item["centre"][1]))
            cv2.drawMarker(centres_img, (cx, cy), (0, 0, 0), cv2.MARKER_CROSS, 22, 5)
            cv2.drawMarker(centres_img, (cx, cy), colour, cv2.MARKER_CROSS, 22, 2)
    save("09_centres.png", centres_img)

    # 10 - final composite: grid, all 121 intersections, contours, centres, labels.
    composite = draw_grid(rectified, labels=True)
    for point in grid_points.astype(np.int32):
        cv2.circle(composite, tuple(point), 2, (255, 255, 255), -1)
    for name, colour in (("red", (0, 0, 255)), ("yellow", (0, 165, 255))):
        for item in results[name]:
            cv2.drawContours(composite, [item["contour"]], -1, colour, 2)
    for name, colour in (("red", (0, 0, 255)), ("yellow", (0, 165, 255))):
        for centre, label in assignments[name]:
            cx, cy = int(round(centre[0])), int(round(centre[1]))
            cv2.drawMarker(composite, (cx, cy), colour, cv2.MARKER_CROSS, 20, 2)
            cv2.putText(composite, label, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA)
    save("10_final_composite.png", composite)

    return saved


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def main(argv=None):
    args = parse_arguments(argv)
    image = load_image(args.image)

    detected = detect_aruco_markers(image)
    print("Detected ArUco marker IDs: {}".format(sorted(detected.keys())))

    try:
        arena_corners, marker_ids = get_arena_corners(detected)
    except RuntimeError as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        print("Not writing a results file because localisation failed.", file=sys.stderr)
        return 1

    rectified = rectify_arena(image, arena_corners)
    if args.debug:
        print("Rectified image shape: {}".format(rectified.shape))

    grid_points, grid_labels = build_grid()
    results, masks = detect_survivors(rectified)

    assignments = {"red": [], "yellow": []}
    critical_labels = []
    stable_labels = []
    for name in ("red", "yellow"):
        for item in results[name]:
            centre = item["centre"]
            index, distance = nearest_intersection(centre, grid_points)
            label = grid_labels[index]
            assignments[name].append((centre, label))
            if name == "red":
                critical_labels.append(label)
            else:
                stable_labels.append(label)
            if args.debug:
                kind = "red" if name == "red" else "yellow"
                print("  {} centre=({:.1f}, {:.1f}) -> {} distance={:.2f}".format(
                    kind, centre[0], centre[1], label, distance))

    if args.debug:
        print("Accepted red contours: {}".format(len(results["red"])))
        print("Accepted yellow contours: {}".format(len(results["yellow"])))
        col_lines, row_lines = measure_painted_lines(rectified)
        expected = _grid_lines()
        print("Calculated interior grid lines: {}".format(expected))
        print("Painted vertical lines detected: {}".format(col_lines))
        print("Painted horizontal lines detected: {}".format(row_lines))

    critical_labels = _sort_labels(critical_labels)
    stable_labels = _sort_labels(stable_labels)

    path = write_results(args.image, marker_ids, critical_labels, stable_labels)
    print("Wrote results to {}".format(path))
    print("Critical Survivors: {}".format(", ".join(critical_labels)))
    print("Stable Survivors: {}".format(", ".join(stable_labels)))

    if args.debug:
        debug_dir = os.path.join(os.getcwd(), DEBUG_DIR_NAME)
        saved = save_debug_images(debug_dir, image, detected, arena_corners, rectified,
                                  results, masks, grid_points, grid_labels, assignments)
        print("Debug images written to {}:".format(debug_dir))
        for item in saved:
            print("  {}".format(item))

    return 0


if __name__ == "__main__":
    sys.exit(main())