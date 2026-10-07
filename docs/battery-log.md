# Battery logger — architecture

Estimating how long a robot's battery will last, from the overhead camera alone.

## Research question

The long-term goal is to predict a robot's remaining runtime without trusting
its own telemetry. The first step is to show that the camera can see the
battery at all: **does a robot's real, camera-measured speed at a fixed motor
command track its battery voltage?**

The problem splits into two parts, and they are not equally hard:

| Step | Question | Expected difficulty |
|------|----------|---------------------|
| 1. Vision → voltage | Does measured speed follow the voltage under load? | Feasible: see *Why it works* |
| 2. Voltage → runtime | How much time is left at a given voltage? | Hard for NiMH, which holds a flat plateau and then drops at a sharp knee |

`battery_log` answers step 1. It also records whole drain curves (voltage and
speed from full to empty, with the total runtime), which are the ground truth
step 2 needs.

## Why it works: open-loop control

The robots run open-loop: `ODOMETRY_ENABLED = False` in
`src/robots/uart_controller.py`. A motor command therefore sets a fixed PWM
duty. The effective motor voltage is `duty × V_battery`, and a DC motor's speed
is roughly proportional to that voltage, so **speed at a fixed command falls
as the battery drains.**

With the encoder PID enabled, the controller would cancel the voltage drop.
Speed would stay flat until the motors saturate, and the signal would vanish.
Flipping that flag invalidates every comparison with earlier runs.

## Data flow

```
 Basler camera ──► ArucoTracker ──► pose (mm, via homography) ─┐
   (owned by battery_log;        also published on              │
    poses go out on /tmp/vision_hub.sock for the dashboard)     ▼
                                                   ┌────────────────────────┐
 robot RP2040                                      │  tools/vision/         │
   mean of ~200 ADC reads, sent every 2 s         │  battery_log.cpp       │
   └► MSG_METRICS ─► receiver ESP32                │                        │
        └► MSG_TELEMETRY (battery mV, uint16,     │  orbit controller ─────┼──► SwarmClient
             BAT_VALID flag) ─► dongle ─► swarm_hub│  (motor units)         │      └► swarm_hub ─► robot
             ─► SwarmClient::robotState(id) ──────►│                        │
                                                   │  lib/BatteryLog/       │
                                                   │   BlRowAccumulator     │
                                                   │   BlSchedule, BlStop   │
                                                   └───────────┬────────────┘
                                                               ▼
                                    battery_log_results/battery_log_r<id>_c<cmd>_<ts>.csv
                                    battery_log_results/battery_log_r<id>_c<cmd>_<ts>.png
                                                               │
                                                               ▼
                                    tools/analysis/battery_log_plot.py  (runs compared)
                                    tools/analysis/battery_log_view.py  (one run, interactive)
```

No new wire-protocol messages. The battery voltage already rides in
`MSG_TELEMETRY` (as uint16 millivolts), so the "keep it at three" rule for the protocol is untouched.

## Components

| File | Role | Depends on |
|------|------|-----------|
| `tools/vision/battery_log.cpp` | Vision, control, I/O, CSV and plot drawing | ArucoTracker, SwarmClient, DemoHud, OpenCV, pylon |
| `lib/BatteryLog/battery_log.h` | Camera-free bookkeeping: rows, stop criteria, rest schedule, rolling median | the standard library only |
| `tests/test_battery_log.cpp` | Unit tests for the header, using synthetic orbits of known geometry | the header |
| `tools/analysis/battery_log_plot.py` | Fits across runs; a figure if matplotlib is installed | stdlib (matplotlib optional) |
| `tools/analysis/battery_log_view.py` | Zoomable viewer for one run's CSV | matplotlib, numpy (fetched by `uv run`) |

This is the same split as `lib/CarFollowing/`: the tool does only vision,
control and I/O, and everything that can be tested without hardware lives in
a pure header.

### `battery_log.h`

- **`BlSchedule`**: the orbit and rest duty cycle. It handles Orbit → Rest
  after `restEveryS` and Rest → Seek after `restForS`. Seek → Orbit depends on
  where the robot is, so the tool decides that transition.
- **`BlRowAccumulator`**: turns per-frame poses into one log row. It measures
  speed along the ring (see *Speed measurement*) and also computes the mean
  distance off the ring, the mean |turn| and the frame and pair counts.
- **`BlStop`**: decides when a run is over. The reasons are low voltage, the
  low-battery flag, a stall, a seek timeout and a lost robot. The first reason
  wins and is never overwritten.
- **`blRollingMedian`**: a causal (trailing-window) median for the smoothed
  trace on the plot.

## Lifecycle

```
 setup ──cue + hub + robot visible──► seek ──within 20 mm of ring──► orbit
   ▲                                   ▲  ▲                           │  │
   │                                   │  └──── robot lost > 1 s ─────┘  │
   │                                   │  └──── hub disconnected ────────┤
   │                                   └── rest over ── rest ◄── every   │
   │                                                          --rest-every
   └ (motors held at 0)                                                  │
                         any phase ──stop criterion / --max-time / s,q──► done
```

- **setup**: motors held at zero. The run is cued by `<enter>` on stdin,
  space in `--debug`, or `--start`. A cue is latched until the hub is
  connected and the robot is in view. The ring fixture and the homography are
  checked at launch, and the tool refuses to start without them.
- **seek**: radial-only drive onto the ring, the same as `circle_speed_test`.
  Only time in which the robot could actually be driven (visible, hub
  connected) counts toward the 30 s seek timeout.
- **orbit**: `--cmd` is fed straight into the heading controller as the
  tangential speed, in motor units, exactly as circle_demo feeds `vTan`.
- **rest**: motors at zero, so the log captures the resting voltage. A rest
  always ends in a reseek, so a robot that drifted while stopped is back on
  the ring before its speed counts again.
- **done**: the final row is closed, the CSV gets an `# ended:` footer, the
  PNG is written one last time, and a zero motor command is sent three times.
  The final PNG is then opened in the desktop's image viewer (`xdg-open`,
  `open` on macOS). It is skipped with `--no-open` or when no display is set,
  e.g. over ssh.

**Every row belongs to exactly one phase.** A phase change closes the current
row early, which is why the CSV carries a `dt_s` column. Ctrl-C leaves the loop
without passing through `done`, so the open row is closed after the loop.

### A hub dropout is not a stall

When the hub drops, the robot's own watchdog stops it. Ten seconds later the
stall check would end a run that took hours to reach that point. So during
orbit, a disconnect is treated like a lost robot: the tool drops back to seek,
the seek timer is frozen while the hub is down, and orbit resumes once the
robot is back on the ring.

## Heading controller

This is circle_demo's orbit controller, **ported from `car_following.cpp`**
rather than from `circle_speed_test.cpp`, because only car_following's copy
has circle_demo's 0.5 s yaw low-pass (`YAW_TAU_S`). The rest is the same: the
one-control-period D-term window, a single `MAX_TURN` over feedforward and
feedback together, the `MAX_TURN_RATE` slew limit, and a velocity field in
motor units. The "port, not a variant" rule in `CLAUDE.md` applies. This is
now the fourth copy of that law (circle_demo, car_following, circle_speed_test
and battery_log); extracting it into a shared library is an open cleanup item.

## Speed measurement

Each frame's displacement is **projected onto the ring tangent** at the
midpoint of the step. The projections are summed over the row and divided by
the time those steps covered:

```
speed = Σ (Δp · t̂_mid) / Σ Δt          t̂ = dirSign · (−r̂_y, r̂_x)
```

- **Not `Σ |Δp|`.** Position noise always adds to a magnitude, so `hypot` of
  per-frame deltas reads fast. A 2 mm zig-zag at 100 fps adds hundreds of mm/s,
  and the error depends on the frame rate. A signed projection averages the
  noise out, and radial wobble from the controller drops out the same way.
- **It is signed.** A robot orbiting against `--dir` reads negative instead of
  looking fine.
- **A dropout breaks the chain.** A frame without a pose ends the current run
  of consecutive poses, and the gap's time is not counted, so a dropout
  neither inflates nor deflates the speed.
- **The last pose is carried across rows**, so the step that straddles a row
  boundary lands in the next row and isn't lost.

A homography is mandatory, because speed has to be in millimetres. A constant
scale error in the calibration cancels out once speed is normalised per run.

## Battery reading

`SwarmClient::robotState(id)` gives the battery as `batteryMv`, in
millivolts. A reading is **valid** only if all of these hold:

- `SC_STATUS_BAT_VALID` is set (a value of 0 without it means "never measured",
  not an empty battery);
- the value is non-zero;
- the telemetry is fresher than `TELEMETRY_STALE_S` (3 s).

Two properties of the reading shape the rest of the design:

- **Averaged.** The robot adds one ADC read per 10 ms PID tick to a sum and
  sends the mean every 2 s, so ADC noise and motor-PWM ripple are smoothed and
  the value keeps the ADC's full 1 mV resolution on the wire.
- **Repeated samples.** The robot samples every 2 s (`BAT_INTERVAL_MS`), and
  telemetry repeats the last sample, so the PC can't tell a new reading from a
  repeated one. That is why the low-voltage stop is debounced by **time**
  below the threshold (`lowHoldS`, 6 s ≈ 3 samples) and not by counting
  readings.

**Loaded and resting voltage.** The reading taken during orbit is lower than
the resting one by the cell's internal resistance times the motor current.
Speed follows the *loaded* voltage, so the fits use orbit rows only. The rest
windows record the resting voltage, and the difference between the two is a
second signal worth studying on its own.

`STATUS_LOW_BATTERY` is logged and would stop the run, but nothing in the
firmware sets it today.

## Stop criteria (`BlStopConfig`)

| Reason | Condition | Default |
|--------|-----------|---------|
| Low voltage | valid reading ≤ `stopMv` held for `lowHoldS` | 4000 mV (1.0 V/cell), 6 s |
| Low-battery flag | `STATUS_LOW_BATTERY` set | immediately |
| Stall | orbit rows < `stallFrac` × baseline for `stallS` without a break | 20 %, 10 s |
| Seek timeout | seek lasts longer than `seekTimeoutS` while the robot could be driven | 300 s (the seek itself is floored at 18 motor units outside the arrival band, `blSeekRadial`, so it cannot stall just short of it) |
| Robot lost | not seen for `lostS` (outside setup) | 60 s |
| Max time | `--max-time` reached | off |
| User | `s` / `q` / Ctrl-C | |

The stall baseline is the time-weighted mean orbit speed over the first
`baselineS` (60 s) of **accumulated orbit time**, not wall time, so rests and
reseeks don't eat into it. Rest and seek rows reset the stall timer.

## Outputs

### CSV

One row per second (shorter at a phase change), flushed after every row so a
crash costs one row, not the run. The `#` header lines record the robot,
command, direction, ring (centre, radius and the file it came from), rest
schedule, stop threshold, and the open-loop assumption. An `# ended:` footer
gives the duration, the reason and the stall baseline.

```
t_s,wall_iso,robot,phase,dt_s,cmd,motor_l,motor_r,speed_mms,radial_err_mm,
abs_turn,n_frames,visible,bat_mv,bat_valid,low_bat_flag,loop_hz
```

Missing values (no pose, no valid battery) are empty fields, not zeros.
`radial_err_mm` and `abs_turn` are there so drift in orbit quality can be told
apart from slowdown caused by the battery. `loop_hz` is the tool's own main-loop
iterations per second over the row (not the camera rate, which is `n_frames`);
a drop shows the PC side starving, as opposed to the robot slowing down. The
same figure is on the once-a-second status line (`loop N/s`).

`--log` additionally appends every console line to
`/tmp/battery_log_r<ID>_<time>.log` (flushed per line).

### Plot (in the tool)

The plot is drawn with OpenCV only, in the same hand-drawn style as
`circle_speed_test`'s `renderPlot()`. It is rewritten every 60 s and at exit,
and shown live in `--debug`.

1. **Measured speed vs time**: raw orbit rows, a 30 s rolling median, and
   rest windows shaded. The speed axis is sized from the median's peak, so a
   single bad detection can't rescale it.
2. **Battery mV vs time**: loaded (orange) and resting (blue) points, with a
   dashed line at `--stop-mv`.
3. **Speed vs loaded voltage**: orbit rows shaded light (early) to dark
   (late). This is the correlation under test.

Time is shown in minutes, with at least 5 on the axis. `putText` uses Hershey
fonts, which are ASCII-only, so labels must avoid non-ASCII characters.

### Cross-run analysis

For each run, `battery_log_plot.py` fits speed = a·mV + b on the orbit rows
with a valid reading. It reports the slope (mm/s per V), R², the starting speed
v0 (median over the first `--baseline-min` minutes), the run length and the
end reason. It then fits all runs together on **speed / v0**, because robots
differ in friction, gearing and wheel wear far more than in how their speed
responds to voltage.

### Interactive viewer

`battery_log_view.py` opens one window per run with the in-tool plot's panels
plus the sag per rest and the orbit quality (radial error, mean |turn|). The
time panels share one axis, every y-axis rescales to the data in view, and the
speed-vs-voltage panel and its fit follow the time window. Zoom with the
toolbar or the scroll wheel; `--range FROM TO` (minutes) opens zoomed in.

The smoothed speed is a trimmed mean over a **whole number of laps** (at least
30 s), not a fixed 30 s median. The per-row speed varies around the lap with
the robot's position on the ring, so a window of a fractional lap count leaves
a ripple as large as the battery effect. The bimodal raw speeds come from the
same lap pattern.

## Running an experiment

```bash
cd tools && make build/battery_log
# smoke test: 2 minutes, short rest cycle
./build/battery_log --robot 3 --cmd 60 --max-time 120 --rest-every 30 --rest-for 10 --debug
# real drain: fully charged cells, run until 4.0 V
./build/battery_log --robot 3 --cmd 60 --start
python3 analysis/battery_log_plot.py battery_log_results/*.csv
analysis/battery_log_view.py                 # newest run, zoomable (needs uv)
```

Prerequisites: a saved ring (`car_following_ring.yml`, or `circle_demo.yml` as
a fallback) and a homography, both set up via `car_following` or
`circle_demo`. Repeat for 2–3 robots so the cross-robot normalisation has
something to work with.

## Open questions and next steps

- **Voltage → runtime.** With NiMH's flat plateau, remaining time is poorly
  determined by voltage in the middle of the discharge. Candidate features
  besides the plain voltage: the rate at which the speed is falling, the sag
  between resting and loaded voltage (internal resistance rises near empty),
  and cumulative distance driven.
- **Live recalibration.** `car_following` converts to motor units through a
  fixed `--robot-max-speed`, which goes stale as batteries drain. A fitted
  speed-vs-voltage model could correct each robot live.
- **Command dependence.** Is the slope proportional to the command
  (`speed ∝ duty × V`), or is it offset by friction and a PWM deadband?
  Answering that means runs at several `--cmd` values.
- **Controller extraction.** Four copies of the orbit law is a maintenance
  risk; a shared `OrbitController` header would remove it.
