# ArUco Tracker Calibration

Tunes `aruco_tracker_config.json` for a specific lighting environment using
CMA-ES (Covariance Matrix Adaptation Evolution Strategy).  Run once when you
set up in a new room, change the lighting, or notice detection quality degrading.

## Quick start

```sh
cd tools/calibration
make

# Static — tune for still markers (good baseline, run first)
./calibrate

# Motion — tune for moving robots
./calibrate --motion
```

## How it works

**Phase 1 — Capture (once):**
Frames are captured from the live camera and stored in memory.
- Static: 60 frames (~2 s), markers held still
- Motion: 120 frames (~4 s), robots driving at operating speed

**Phase 2 — Optimisation (offline):**
CMA-ES runs for up to 150 generations.  Each generation evaluates ~11 candidate
configs by replaying the detector against the cached frames — the camera is never
touched again.  At ~1 ms per detection, one generation takes ~660 ms and a full
run completes in roughly 2 minutes.

**Output:**
The winning config is written back to `aruco_tracker_config.json` and a diff of
changed parameters is printed.

## All flags

| Flag | Default | Description |
|---|---|---|
| `--static` | ✓ | Optimise for still markers |
| `--motion` | | Optimise for moving robots |
| `--camera <idx>` | 0 | Camera index |
| `--frames <n>` | 60 / 120 | Frames to capture (static / motion) |
| `--iters <n>` | 150 | CMA-ES generations |
| `--sigma <f>` | 0.30 | Initial step size in [0,1] space |
| `--config <path>` | `../aruco_tracker_config.json` | Base config to start from |
| `--output <path>` | same as `--config` | Where to write the result |

## File structure

```
calibration/
├── calib_main.cpp        CLI entry point — ties optimizer + objective together
├── cmaes.h               IOptimizer interface + full CMA-ES implementation
├── param_space.h         Search space: bounds, encode/decode, makeDetector, writeConfig
├── objective.h           IObjective interface + detectFrame/preprocessGray helpers
├── objective_static.h    StaticObjective  — still-scene scoring
├── objective_motion.h    MotionObjective  — moving-robot scoring
└── Makefile
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

### Motion objective
```
score = 0.5 · detection_rate + 0.3 · streak_ratio + 0.2 · smoothness
```
- **detection_rate** — same as static
- **streak_ratio** — longest consecutive detection run / total frames; penalises
  flickering configs that break the Kalman tracker's prediction window
- **smoothness** — 1 − velocity_std / mean_speed; a config that drops and
  re-acquires a marker produces large apparent velocity jumps — this catches that

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

### Motion objective improvements
The current motion scoring assumes all robots move continuously.
Two refinements worth adding:
- **Occlusion tolerance** — exclude frames where a robot is legitimately out of
  frame from the detection rate denominator (use a convex-hull visibility estimate)
- **Per-robot speed weighting** — robots at the edge of the frame move through
  more distortion; weight their detection rate lower in the score

### Fisheye calibration (`objective_fisheye.h`)
A `FisheyeObjective : IObjective` that drives a checkerboard capture session,
calls `cv::fisheye::calibrate`, writes `fisheye_calib.yaml`, and then runs the
ArUco optimisation with `FisheyeUndistortPreprocessor` active.  This would make
the full "new room" setup a single `./calibrate --fisheye` command.

The `FisheyeUndistortPreprocessor` already exists in `aruco_tracker.h` and slots
directly into the preprocessor pipeline — the objective just needs to load the
resulting YAML and prepend the stage before scoring.

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
