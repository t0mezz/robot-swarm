# Camera calibration

Three separate tools, run in this order when you set the camera up:

| Tool | What it fixes | Writes |
|------|---------------|--------|
| `make_charuco.py` + `build/intrinsics` | **The lens**: focal length, principal point, distortion | `tools/vision/camera_intrinsics.yml` |
| `build/homography` | **The arena**: undistorted pixels to world millimetres | `tools/vision/aruco_homography.yml` |
| `build/calibrate` | **The detector**: ArUco thresholds for this lighting (section below) | `aruco_tracker_config_optimised.json` |

Each one needs the camera to itself — close any other vision tool first.

## 1. Lens calibration (`intrinsics`)

The Basler C125-0618-5M is a plain 6 mm rectilinear lens, so this fits the
standard model (`k1 k2 [k3] p1 p2`) with `cv::calibrateCamera`. It is **not** the
fisheye model (`FisheyeUndistortPreprocessor` in `aruco_tracker.h` is for fisheye
lenses and is unused here).

**Print the board:**

```sh
pip install opencv-contrib-python numpy pillow
python3 make_charuco.py                      # -> charuco_board.pdf / .png / .json
python3 make_charuco.py --cam-height-mm 1500 # check it is still detectable from 1.5 m
```

A3, 10 x 7 squares of 36 mm, `DICT_4X4_50` — the dictionary the tracker itself
detects, whose few large cells survive distance and blur. The script checks that
OpenCV finds all inner corners on a render shrunk to the camera's pixel density and
prints the marker cell size in pixels (aim for >= 6). Print at **100 % / actual
size**, mount it on something rigid and flat (aluminium composite, glass; not foam
board), matt finish so the lights do not glare. Check the 100 mm scale bar with a
ruler, then **measure the squares with calipers** over several squares and pass the
result as `--square-mm`: the calibration is only as accurate as the board.

**Capture and fit:**

```sh
cd tools && make        # builds build/intrinsics and build/homography
./build/intrinsics --square-mm 36.02 --cam-height-mm 1200
```

Keys: `SPACE` take a view, `a` auto-capture (waits for a steady board in a new
place), `u` undo, `ENTER` calibrate, `q` quit. Aim for 15-25 views. A coverage grid
is drawn over the live image: red cells have no board corners yet. Distortion is
largest at the edges, so bring the board into **every corner of the frame** and tilt
it (up to ~45 degrees). Frames are saved to `./intrinsics_frames` so
`--use-cache` re-runs the fit without the camera; `--images <dir>` calibrates from
any set of photos.

It prints per-view error, every parameter with its standard deviation, the
coverage, how far the correction moves a pixel (in px, and in mm at the arena with
`--cam-height-mm`) and a verdict. Views that do not fit the model (bent board,
motion blur) are dropped automatically (`--no-prune` keeps them). `k3` and
`aspect` are fixed by default (`--k3`, `--free-aspect` free them), because a free
k3 overfits a 40 degree lens unless the corners are very well covered.

`./build/intrinsics --verify` shows the live image with a straight-line grid; `u`
toggles raw / undistorted — a straight edge near the border must stay straight.

**What the tracker does with it:** `ArucoTracker::open()` loads
`camera_intrinsics.yml` and undistorts the *tracked points* (marker centre and
heading tip), not the frames — a full-frame remap costs more than the ArUco
detection at 115 fps. It is ignored (with a message) if the resolution or the
sensor ROI offset (`offset_x/offset_y`) differ from when it was made;
`"use_intrinsics": 0` in `aruco_tracker_config.json` turns it off.

## 2. Homography (`homography`)

```sh
./build/homography --arena 800 600 --plane-mm 35        # click reference points
./build/homography --board --arena 800 600 --plane-mm 35 # lay the ChArUco board down
```

Fits pixel -> world mm in **undistorted** pixel space and stamps
`undistorted: 1` into `aruco_homography.yml`; the tracker rejects a homography made
with the other setting, so redo this after (re)calibrating the lens. It replaces the
4-corner `c` / `--calibrate` flow in the demos (which still works, but gives you a
4-point fit with no error estimate).

*Arena mode:* mark the arena (tape at the corners, the four edge midpoints and the
centre) and click them in order: TL TR BR BL, top / right / bottom / left mid,
centre. A magnifier follows the cursor, arrow keys nudge the last point by 0.25 px,
`r` snaps clicks to the nearest image corner. With 9 points it reports each point's
residual and a **leave-one-out error** (fit without the point, predict it) — the
honest accuracy. With only 4 it tells you the error is zero by construction.

*Board mode:* lay the board flat and upright in the arena, `SPACE`, type where its
top-left outer corner sits in the arena frame (mm; origin at the arena's top-left,
x right, y down), and repeat in several places. Every inner corner (54 by default)
is a sub-pixel reference point, and each placement is held out in turn. Take the
robots out first: their markers share ids with the board's.

Afterwards the 100 mm world grid is drawn back onto the live image — it should lie
on the floor features; `s` saves, `b` goes back, `q` quits.

**Calibrate at the height of the robot markers.** A homography is exact for one
plane. Markers a few cm above the floor, seen at distance r from the optical axis,
land `h·r/H_cam` off a floor calibration: centimetres at the arena edge. Put the
board / tape at marker height and pass `--plane-mm` so it is recorded.

Existing ring/circle fixtures (`car_following_ring.yml`, `circle_demo.yml`) are in
world millimetres of the *old* frame: re-set them if the origin or arena changed.

## 3. ArUco detector tuning (`calibrate`)

Tunes `aruco_tracker_config.json` for a specific lighting environment using
CMA-ES (Covariance Matrix Adaptation Evolution Strategy).  Run once when you
set up in a new room, change the lighting, or notice detection quality degrading.

## Quick start

```sh
cd tools/vision/calibration
make

# Tune for still markers (interactive capture, then offline optimisation)
../../build/calibrate

# Re-run the optimiser on the frames saved by the last run (no camera needed)
../../build/calibrate --use-cache

# Score the current config without optimising
../../build/calibrate --eval
```

## How it works

**Phase 1 — Capture (interactive):**
A live preview window opens.  Press SPACE to capture a frame; press ENTER or Q
when you have enough.  At least 5 frames are recommended — more pictures from
varied positions give the optimiser a broader dataset.  Captured frames are
saved as PNGs to `--cache-dir` so later runs can skip this phase with
`--use-cache`.

**Phase 2 — Optimisation (offline):**
CMA-ES runs for up to 150 generations.  Each generation evaluates ~11 candidate
configs by replaying the detector against the captured frames — the camera is
never touched again.  At ~1 ms per detection, one generation takes ~660 ms and
a full run completes in roughly 2 minutes.

**Output:**
The winning config is written to `aruco_tracker_config_optimised.json` (or
`--output`) and a diff of changed parameters is printed.

## All flags

| Flag | Default | Description |
|---|---|---|
| `--eval` | | Evaluate the current config only (no optimisation) |
| `--use-cache` | | Skip capture and reuse frames saved by the last run |
| `--cache-dir <path>` | `/tmp/calib_frame_cache` | Directory for the frame cache |
| `--serial <sn>` | from config | Basler camera serial number |
| `--ip <addr>` | from config | Basler camera IP address |
| `--ids <n>` | auto-detect | Treat IDs 0…n as the expected marker set (e.g. `--ids 2` → markers 0, 1, 2) |
| `--iters <n>` | 150 | CMA-ES generations |
| `--sigma <f>` | 0.30 | Initial step size in [0,1] space |
| `--config <path>` | `../aruco_tracker_config.json` | Base config to start from |
| `--output <path>` | `../aruco_tracker_config_optimised.json` | Where to write the result |

## File structure

```
tools/vision/calibration/
├── calib_main.cpp        CLI entry point — ties optimizer + objective together
├── Makefile
└── README.md

lib/Calibration/
├── cmaes.h               IOptimizer interface + full CMA-ES implementation
├── param_space.h         Search space: bounds, encode/decode, makeDetector, writeConfig
├── objective.h           IObjective interface + detectFrame/preprocessGray helpers
└── objective_static.h    StaticObjective  — still-scene scoring
```

## Scoring

### Static objective
```
score = 0.7 · detection_rate + 0.3 · corner_stability
```
- **detection_rate** — fraction of (expected ID × frame) pairs where the marker
  was successfully found
- **corner_stability** — 1 − (mean per-corner std-dev / 5% of marker perimeter);
  rewards configs where detected corner positions are consistent across frames

## Parameters being optimised

| Parameter | Range | Notes |
|---|---|---|
| `thresh_c` | 3 – 15 | Adaptive threshold constant |
| `win_max` | 11 – 31 (odd) | Max adaptive threshold window |
| `win_step` | 1 – 4 | Window size step |
| `min_perim_rate` | 0.01 – 0.06 | Minimum marker perimeter fraction |
| `poly_approx` | 0.02 – 0.10 | Polygon approximation accuracy |
| `error_corr` | 0.50 – 0.95 | Hamming error correction rate |
| `min_otsu_stddev` | 2 – 12 | Minimum Otsu std dev |
| `clahe_clip` | 0.5 – 5.0 | CLAHE contrast clip limit |
| `kf_proc_vel` | 0.001 – 0.10 (log) | Kalman velocity process noise |
| `kf_meas` | 1 – 20 | Kalman measurement noise (px²) |
| `roi_pad` | 1.0 – 2.0 | ROI bbox padding multiplier |

Non-optimised fields (camera settings, dictionary, ROI state machine thresholds)
are preserved from the base config.

---

## Planned extensions

### Motion objective (`objective_motion.h`)
A `MotionObjective : IObjective` scored on frames of robots driving at
operating speed, e.g.:
```
score = 0.5 · detection_rate + 0.3 · streak_ratio + 0.2 · smoothness
```
- **detection_rate** — same as static
- **streak_ratio** — longest consecutive detection run / total frames; penalises
  flickering configs that break the Kalman tracker's prediction window
- **smoothness** — 1 − velocity_std / mean_speed; a config that drops and
  re-acquires a marker produces large apparent velocity jumps — this catches that

Refinements worth adding beyond the basic scoring:
- **Occlusion tolerance** — exclude frames where a robot is legitimately out of
  frame from the detection rate denominator (use a convex-hull visibility estimate)
- **Per-robot speed weighting** — robots at the edge of the frame move through
  more distortion; weight their detection rate lower in the score

### Lens correction inside the detector objective
`objective_static.h` scores detection on the raw (distorted) frames, which is what
the live tracker sees too — the lens model is applied to points afterwards, so the
two stay consistent. If detection near the frame edge ever becomes the limit,
a frame-level stage could be added here, at the cost described in
`camera_intrinsics.h`.

### GP with ARD kernel (`gp_ard.h`)
Drop-in replacement for CMA-ES implementing `IOptimizer`.  Gaussian Process
Bayesian optimisation with an Automatic Relevance Determination (ARD) kernel
learns which parameters actually matter for the current environment and allocates
more search budget there.

GP-BO is more sample-efficient than CMA-ES when evaluations are expensive.
In this setup evaluations are cheap (2 min total), so CMA-ES is the better
default.  GP-ARD becomes useful if the objective is extended to include live
robot runs (expensive) or fisheye re-calibration per candidate (very expensive).

The ask/tell interface in `IOptimizer` is already GP-compatible:
`ask()` returns a batch (GP can return batch size 1 for sequential BO),
`tell()` updates the posterior.  The swap in `calib_main.cpp` is one line:

```cpp
// Current
CMAES opt(kNParams, args.sigma0);

// Future
GPARD opt(kNParams);  // gp_ard.h
```

### Combined static + motion objective
Run both captures and combine scores:
```
combined = α · static_score + (1−α) · motion_score
```
with α tunable via `--alpha 0.5`.  This produces a config that performs well
under both conditions rather than specialising for one.

### Parameter range narrowing
After a first broad sweep, print a suggested narrowed search range based on
the CMA-ES final covariance — parameters with low variance at convergence
have been well-determined and their range can be halved for the next run.
