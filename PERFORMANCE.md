# Performance Tracking

Tracks measured performance (loop_fps, det_fps, per-section timings, etc.) of
the PC-side vision/control tools (`tools/build/*`), and what changes were made
to improve it. Goal: keep a before/after record per tool so improvements can
be visualized over time, instead of relying on memory/anecdote.

Naming note (2026-07): the tracker's detection-thread rate was labeled
`cam_fps` in the HUD and in the historical entries below; it has been renamed
`det_fps` because it measures frames *processed* by the detection thread, not
the camera's acquisition rate (the two diverge whenever detection can't keep
up — the camera keeps grabbing and frames are skipped). Old entries keep the
old name. The config key `cam_fps` (requested AcquisitionFrameRate) is
unchanged.

Until the generalized DEBUG-util exists (see `TODO.md` → "Tooling / Tests"),
numbers come from ad-hoc `[perf]` instrumentation added directly to each demo
(throwaway code, removed/replaced once the shared util lands).

Test machine: Intel i9-14900K (32 threads), Wayland session (OpenCV/Qt window
runs via XWayland), cpufreq governor as noted per entry.

## How to log a new entry

1. Run the binary from `tools/build/` (its relative config path
   `../vision/aruco_tracker_config.json` only resolves from there):
   ```
   cd tools/build && ./circle_demo
   ```
2. Let it run ~10s after "Robot ... registered" — `[perf]` lines print once
   per second and are quite stable; eyeballing 5-10 lines is enough, no need
   to formally average.
3. Add a new dated subsection under the relevant tool with:
   - what changed since the last entry (one line)
   - environment notes if relevant (governor, robot count, resolution —
     anything that could explain a delta besides the change itself)
   - the same total/control/draw/imshow/waitKey/other breakdown table
   - a one-line delta vs. the previous entry

## circle_demo

### 2026-06-12 — Baseline (revised: split by tracking state)
- cam_fps: ~115 (Basler `AcquisitionFrameRate` cap, live-measured in detection thread)
- governor: `powersave`, EPP: `balance_power` (fans ~700/4500 RPM, cores ~27-31°C — idle EC fan curve)
- 1 robot. This run cleanly captured two regimes back to back: ROI tracking
  active, then the robot was removed and tracking fell back to full-frame
  search. Per-frame breakdown (avg over ~1s, from `[perf]` log):

  | section | tracking (ROI active, 53 samples) | lost / full-frame search (12 samples) |
  |---|---|---|
  | loop_fps | ~73 (58-94) | ~33.5 (31-36) |
  | total | ~13.8 ms | ~30.0 ms |
  | control | ~0.01 ms | ~0.01 ms |
  | draw | ~3.76 ms | ~3.53 ms |
  | imshow | ~1.03 ms | ~1.79 ms |
  | waitKey(1) | ~3.14 ms | ~3.08 ms |
  | other (inside `tracker.update()`) | ~5.85 ms | ~21.57 ms |

- **Key finding**: losing ROI and falling back to full-frame ArUco search at
  1492x1248 makes `other` ~4x more expensive (~5.9ms -> ~21.6ms), which alone
  roughly doubles `total` and halves `loop_fps` (~73 -> ~33.5). This dwarfs
  every governor/EPP effect measured below (those are ~10-50% effects). The
  gap between `loop_fps` (~73, tracking) and `cam_fps` (115) is dominated by
  `other` (~5.9ms) + `waitKey` (~3.1ms) + `draw` (~3.8ms) + `imshow` (~1ms).
- `waitKey(1)` here (~3.1ms) and `draw`/`imshow` are essentially unchanged
  between the two regimes — the ROI-vs-full-frame cost is isolated to
  `tracker.update()`. Likely candidate: ROI restricts the search area enough
  that most of the ~30 detection threads finish almost immediately, whereas
  full-frame search gives every thread a full tile to scan.
- Removed unused per-robot selection/speed-override system (0-9, `.`,
  selection-gated +/-) — `cv::waitKey(1)` is still needed for the remaining
  controls (`t [ ] s c q +/-`) and window/mouse event pumping, so this did
  not change the timing breakdown above; it's a code-cleanup entry, not a
  perf change.

### 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config)
- Change: `tools/vision/aruco_tracker_config.json` — `half_res_sweep` was
  explicitly `false` (code default is `true`); flipped back to `true`. This
  makes the global/`GLOBAL`-state sweep run `detectMarkers` on a 746x624
  image instead of the full 1492x1248, with results scaled back up.
- Result (1 robot, governor `powersave`/EPP `balance_power`, same as above):

  | section | tracking (21 samples) | lost / GLOBAL sweep (16 samples) |
  |---|---|---|
  | loop_fps | ~74.1 | ~78.3 |
  | total | ~13.6 ms | ~12.85 ms |
  | other | ~5.53 ms | ~4.56 ms |

- **GLOBAL-state `other` dropped ~5x (~21.6ms -> ~4.56ms)** and `total`
  dropped from ~30ms to ~12.85ms (loop_fps ~33.5 -> ~78.3) — losing tracking
  is no longer a meaningfully different cost regime at all. Re-acquisition
  after eviction still worked (`Robot 0 registered` fired again).
- Remaining gap to `cam_fps` (115) is now ~35-40fps (~35%) **regardless of
  tracking state** — `other` (~4.5-5.5ms) + `draw` (~3.6ms) + `waitKey`
  (~3.1ms) + `imshow` (~1-1.8ms) are now all comparable-sized contributors,
  no single dominant bucket left like the old GLOBAL-sweep case.
- This single config flip was a much bigger win than anything in the
  governor/EPP/fan experiments below — done, no further action needed here
  unless half-res sweep turns out to hurt re-acquisition range/reliability in
  practice (worth keeping an eye on with robots farther from the camera).

### Governor / EPP / fan experiments (see below)
The entries below vary `cpufreq` governor and EPP; all were measured with 0-1
robots and inconsistent tracking state, so their `other`/`total` numbers are
**not directly comparable** to the revised baseline above (different regime
mix). Treat them as governor/EPP-only signal: `waitKey` swings ~3-9ms and
`draw`/`other` swing ~50% run-to-run from governor/EPP/scheduling noise alone,
i.e. noticeably smaller than the ~4x ROI-vs-full-frame effect above.

**-> All resolved below**: the governor/EPP/throttling/fan-curve story in this
whole subsection turned out to be a red herring. The real cause was a
`sleep_for(30ms)` poll in `circle_demo`'s main loop, fixed in the final entry
below — `loop_fps` now matches `cam_fps` (~115) under any governor/EPP.

### 2026-06-12 — governor `powersave` → `performance`
- Change: `sudo cpupower frequency-set -g performance`.
- Caveat: this run was captured headless via the agent's shell (1 robot,
  no homography loaded, camera at 1492x1248 vs the usual 1920x1080), so
  `other`/`total`/`loop_fps` aren't directly comparable to the baseline row —
  please re-run from your normal session and replace this entry with those
  numbers.

  | section | avg time |
  |---|---|
  | total | ~26.0 ms |
  | control | ~0.01 ms |
  | draw | ~0.9 ms |
  | imshow | ~0.3 ms |
  | waitKey(1) | ~2.4 ms |
  | other | ~22.8 ms |

- Signal that *did* carry over: `waitKey(1)` dropped from ~6.4ms → ~2.4ms,
  consistent with the powersave-oversleep hypothesis. `draw` also dropped
  (~2.4ms → ~0.9ms), but that's likely the smaller frame size, not the
  governor.
- Open question: `other` (inside `tracker.update()`) is much larger here
  (~22.8ms vs baseline ~5.1ms) — needs the real-session numbers to know if
  that's environment noise or something governor=performance made worse.
  -> Resolved below: this was real thermal throttling.

### 2026-06-12 — back to `powersave` (after the `performance` test)
- Change: `sudo cpupower frequency-set -g powersave` (reverting the previous entry).
- Measured in the normal session (1 robot, 1492x1248@115).

  | section | avg time |
  |---|---|
  | total | ~15.6 ms |
  | control | ~0.01 ms |
  | draw | ~3.4 ms |
  | imshow | ~0.9 ms |
  | waitKey(1) | ~3.0 ms |
  | other | ~8.2 ms |

- vs. the very first baseline (also `powersave`, before any governor change):
  `waitKey` improved a lot (~6.4ms -> ~3.0ms), but `draw` (~2.4ms -> ~3.4ms)
  and `other` (~5.1ms -> ~8.2ms) got slightly worse and noticeably noisier
  (loop_fps swings 53-80 here vs. a tight 62-75 originally). Net total is
  roughly a wash (~14.6ms -> ~15.6ms).

#### Why this isn't a clean revert: governor vs. EPP, and real throttling
- `cpupower frequency-set -g performance` also set each core's "Energy
  Performance Preference" (EPP) to `performance`. Switching the governor back
  to `powersave` does **not** revert EPP — confirmed via
  `/sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference`, which
  is now `balance_performance` on every core (not the pre-change default).
  EPP and `scaling_governor` are independent `intel_pstate` knobs.
  `balance_performance` EPP ramps frequency more aggressively on load, which
  is likely both why `waitKey` stayed improved *and* why `draw`/`other` got
  noisier.
- The `performance`-governor run above also tripped real thermal throttling:
  `/sys/devices/system/cpu/cpu{8,9}/thermal_throttle/core_throttle_count` = 37,
  `cpu{10,11}` = 27, a few others = 3, rest = 0 (package back to 32°C now).
  That explains that run's `other` ~22.8ms: the ~30-thread detection pool
  pinned at max clock pushed the package into its thermal/power limit and
  starved the detection thread.
- No reboot needed for any of this — both governor and EPP are live sysfs
  knobs. To get back to the exact pre-experiment state for a clean A/B, reset
  EPP explicitly:
  ```
  echo balance_power | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
  ```
  then re-measure.

### 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default)
- Change: none — EPP had reverted to `balance_power` on its own (or via
  reboot) since the previous entry; `sensors` (now installed) confirms
  governor=`powersave`, EPP=`balance_power`, fans ~695-706/4500 RPM
  (~15%, `pwm_enable`=manual/EC-driven), cores ~27-31°C at idle.

  | section | avg time (steady-state, 23/31 samples) |
  |---|---|
  | total | ~15.4 ms |
  | control | ~0.01 ms |
  | draw | ~3.65 ms |
  | imshow | ~0.77 ms |
  | waitKey(1) | ~9.1 ms |
  | other | ~1.9 ms |

- Plus an ~8s transient mid-run (8/31 samples): loop_fps dropped 65->~32,
  `other` spiked to ~9-22ms (avg ~18.9ms), then fully recovered. Throttle
  counters were unchanged before/after (no new thermal events), so this was
  some other one-off scheduling/background blip, not thermal.
- Caveat: same governor+EPP as the very first baseline entry, but `waitKey`
  (~6.4ms -> ~9.1ms) and `draw` (~2.4ms -> ~3.65ms) are both higher here,
  while `other` (excl. the transient) is lower (~5.1ms -> ~1.9ms). i.e.
  per-section timings vary by ~50% run-to-run even with identical
  governor/EPP — single-run governor/EPP comparisons on this machine are
  noisy. `total` (~14.6 vs ~15.4ms) is the more stable headline number.

### Next experiment — BIOS fan-curve fix
- Root cause for the `performance`-governor throttling (previous entries):
  this is a Dell Precision 3680, fans are EC-controlled (`dell_smm`, currently
  ~15% / pwm_enable=manual-by-EC) and barely respond to load; RAPL limits are
  high (PL1=200W, PL2=253W). Sustained `performance` governor load can hit
  Tjmax before the EC ramps fans, causing PROCHOT throttling.
- Plan: go into BIOS, switch the Thermal Management / System Profile to a more
  aggressive fan curve (e.g. "Performance"), then re-test `performance`
  governor and compare `core_throttle_count` deltas + `other`/`total` against
  the entries above. If throttling stops, `performance` governor should net a
  real improvement instead of the previous wash.
  -> Done below: throttling did stop, but `performance` governor is still a
  net loss — throttling wasn't the actual cause of the high `other`.
- `other` (~2-20ms depending on run) remains the single biggest and noisiest
  lever overall — still needs instrumentation *inside* `tracker.update()` to
  know what it's actually waiting on (frame acquisition vs. detection vs.
  lock contention).

### 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change
- Change: BIOS Thermal Management / System Profile set to a more aggressive
  fan curve, then `sudo cpupower frequency-set -g performance` (governor=
  `performance`; EPP auto-follows to `performance`). Used `--log-perf` on
  `circle_demo` (now gated behind this flag), 1 robot, steady-state (14
  samples, no transients).
- Fan/thermal state, before and throughout the run: all 4 `dell_smm`
  `pwm_enable` channels = `1` (manual) — static duty cycle now 64%
  (~1190 RPM), vs. ~15% (~700 RPM) before the BIOS change. Cores stayed at
  32-34°C the whole time; `core_throttle_count` = 0 on every core, both
  before and after (zero throttling events this run).

  | section | avg time |
  |---|---|
  | loop_fps | ~39 (38-40) |
  | total | ~25.8 ms |
  | control | ~0.01 ms |
  | draw | ~0.96 ms |
  | imshow | ~0.19 ms |
  | waitKey(1) | ~2.35 ms |
  | other | ~22.3 ms |

- **Key finding**: this reproduces the *first* `performance`-governor entry
  above almost exactly (`other`~22.8ms, `total`~26.0ms), which was blamed on
  thermal throttling (`core_throttle_count` 37/27 back then). This run hit
  **zero** throttling events yet shows the same ~22ms `other` / ~26ms
  `total` / loop_fps ~39 — so throttling was likely not the real cause.
  `performance` governor/EPP itself appears to roughly **4x** `other` vs.
  `powersave`/`balance_power` (~1.9-5.5ms) and roughly **halve** `loop_fps`
  (~39 vs ~74-78), independent of thermal state.
- **Not compute-bound either**: `sensors` taken *while* `circle_demo` was
  running showed core temps at only 37-40°C (package 40°C), vs. `high`=80°C
  / `crit`=100°C — barely above the 27-32°C idle baseline. If the ~30-thread
  detection pool were actually spending the extra ~17-20ms computing, cores
  would run noticeably hotter. So the slowdown is most likely the detection
  thread **waiting** (lock/condvar/frame-acquisition) longer under
  `performance`/EPP=`performance` than under `powersave`/`balance_power`,
  not extra CPU work. Reinforces the existing TODO: instrument *inside*
  `tracker.update()` to see what it's actually blocked on.
- **Fan curve still doesn't respond to load**: `pwm_enable=1` means
  `dell_smm` statically holds the duty cycle (now 64% from the BIOS change)
  and overrides the EC's automatic curve — the BIOS change raised the static
  floor but doesn't make the fans reactive to temperature. To get a reactive
  curve, try `echo 2 | sudo tee /sys/class/hwmon/hwmon6/pwm{1,2,3,4}_enable`
  (automatic mode, if the EC/driver combo supports it) or
  `sudo modprobe -r dell_smm_hwmon` to let the EC's own curve run
  unobstructed (reload with `modprobe dell_smm_hwmon` to revert). Not yet
  tested — also note cores never exceeded 34°C here, so the curve may not
  have needed to ramp regardless.
- **Recommendation**: revert to `powersave`/`balance_power` for circle_demo —
  `performance` governor is a clear net loss on this workload (loop_fps ~39
  vs ~74-78 baseline), with or without throttling:
  ```
  sudo cpupower frequency-set -g powersave
  echo balance_power | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
  ```
  -> Superseded by the next entry: this "recommendation" was based on a
  polling artifact, not a real governor effect.

### 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop
- `circle_demo`'s main loop (`tools/vision/circle_demo.cpp`) was:
  ```cpp
  while (g_running) {
      if (!tracker.update()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          continue;
      }
      ...
  }
  ```
  `tracker.update()` is non-blocking (just checks/clears a `fresh` flag under
  a mutex, `aruco_tracker.h:232`). `accTotal` (-> `total`/`other`) measures
  time between consecutive *successful* `update()` calls, so every
  `sleep_for(30ms)` retry while waiting for the detection thread lands
  entirely in `other` — a 30ms poll granularity against an ~8.7ms
  (115fps) camera period and a similarly fast detection thread.
- Added a temporary `stalls` counter to `[perf]` (count of
  `update()==false` -> 30ms-sleep cycles per second) to test this. Result
  matched almost exactly in both governor states:
  - `performance`/EPP=`performance`: `stalls`=29/s, loop_fps=39.
    29 x 30ms = 870ms; `other` x loop_fps = 22.3ms x 39 = 870ms. ✓
  - `powersave`/EPP=`balance_performance` (leftover from earlier tests):
    `stalls`=15-23/s, loop_fps=50-72 — fewer stalls, proportionally higher
    loop_fps, same relationship.
  - i.e. **every governor/EPP/throttling/fan-curve effect in the entries
    above was just this poll landing on a different phase relative to the
    detection thread** — not real CPU/thermal/fan differences.
- **Fix**: `sleep_for(milliseconds(30))` -> `sleep_for(milliseconds(1))`.
  Removed the temporary `stalls` counter afterwards (throwaway debug code,
  per the workflow note at the top of this file).
- Result — same robot/lighting, back-to-back, both governors, steady-state
  (10 samples each):

  | section | `performance` / EPP=`performance` | `powersave` / EPP=`balance_power` |
  |---|---|---|
  | loop_fps | ~115-117 | ~115-116 |
  | total | ~8.7 ms | ~8.7 ms |
  | control | ~0.00 ms | ~0.01 ms |
  | draw | ~0.75-0.85 ms | ~2.4-3.0 ms |
  | imshow | ~0.15-0.16 ms | ~0.27-0.33 ms |
  | waitKey(1) | ~2.33-2.39 ms | ~2.84-2.97 ms |
  | other | ~5.2-5.5 ms | ~2.4-3.1 ms |

- **`loop_fps` now matches `cam_fps` (~115) almost exactly, under both
  governors** — `total`≈8.7ms ≈ 1000/115, i.e. the loop is now
  camera-bound (limited by the Basler `AcquisitionFrameRate` cap), which is
  the best achievable without raising that cap. This closes the
  loop_fps-vs-cam_fps gap that was the entire point of
  `TODO.md` -> "Performance: loop_fps vs cam_fps (circle_demo)".
- governor/EPP no longer materially affects `loop_fps` (115-117 vs
  115-116). `draw`/`other` still trade off between the two (draw higher
  under powersave, other higher under performance, total ~constant), but
  this is now slack inside the camera-bound budget and doesn't matter.
- **Recommendation**: no governor/EPP change needed for circle_demo going
  forward — both hit ~115fps. `powersave`/`balance_power` remains a
  reasonable default for idle power/heat; if the system is still in
  `performance`/EPP=`performance` from testing, revert with:
  ```
  sudo cpupower frequency-set -g powersave
  echo balance_power | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
  ```
- The BIOS fan-curve / `pwm_enable`/RAPL investigation in the previous entry
  is fine as-is and doesn't need further action — this workload never got
  hot enough to need a reactive fan curve, and now runs the same regardless.
- Remaining open item: `other` (~2.5-5.5ms) is a small second-order cost,
  likely real `tracker.update()`/frame-handling overhead — not worth
  chasing further now that the loop is camera-bound.


## vision_controller / wingman / shape_demo / drag_drop_demo
- Not yet instrumented. Add the same `[perf]` breakdown (total/control/draw/
  imshow/waitKey/other) before optimizing, then log baseline here.
