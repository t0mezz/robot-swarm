# Robot Swarm

ESP-NOW based swarm control system for up to 32 Pololu 3pi+ 2040 robots. A controller PC sends motor commands over USB serial to a dongle ESP32, which broadcasts them via ESP-NOW to all robot ESP32s. Each robot ESP32 forwards commands to the RP2040 over UART. End-to-end latency is ~4 ms from keypress to motor response.

A Basler ace2 GigE camera running pylon 8.1.0 provides overhead ArUco marker tracking for vision-based swarm control modes.

---

## Architecture

```
Controller-PC  →  USB-Serial  →  Dongle-ESP32  →  ESP-NOW Broadcast  →  N× Robot-ESP32  →  UART  →  RP2040
                  921600 Baud                        ~1 ms latency                          921600 Baud

Basler ace2 GigE  →  pylon 8.1.0  →  OpenCV ArUco  →  vision_controller / wingman / circle_demo / shape_demo
                                                     │
                       (optional) vision_hub owns the camera instead, and shares
                       poses (Unix socket) + frames (shared memory) + an MJPEG stream
```

---

## Project Structure

```
robot-swarm/
├── platformio.ini              # Firmware build config
├── src/
│   ├── dongle/main.cpp         # Dongle ESP32 firmware
│   └── receiver/main.cpp       # Robot ESP32 firmware
├── lib/
│   ├── SwarmProtocol/          # Shared headers (protocol.h, hardware.h, debug_protocol.h)
│   ├── ArucoTracker/           # Camera abstraction + ArUco tracker (aruco_tracker.h, basler_pylon_source.h);
│   │                           #   pose_hub.h / frame_shm.h / mjpeg_server.h: OpenCV-free pose, frame and video sharing
│   ├── SwarmClient/            # High-level swarm socket client (SwarmClient.h)
│   ├── Calibration/            # CMA-ES detector tuning (cmaes.h, param_space.h, objective*.h)
│   └── swarm/                  # PC-side host tools (swarm_hub, swarm_terminal, swarm_controller, latency_plot)
├── tools/
│   ├── Makefile                # Builds all PC tools (macOS arm64 + Ubuntu x86_64)
│   ├── game.cpp                # SFML game pad controller
│   ├── dashboard-ink/          # Ink/React terminal dashboard (Node; own package.json, no build step)
│   └── vision/
│       ├── vision_hub.cpp         # Opt-in camera owner: shares poses, frames and an MJPEG stream
│       ├── vision_controller.cpp  # Main vision-based swarm controller
│       ├── wingman.cpp            # V-formation follower controller
│       ├── circle_demo.cpp        # Circle orbit formation
│       ├── car_following.cpp      # Sugiyama ring experiment (headless; optional NetLogo page bridge)
│       ├── shape_demo.cpp         # Freehand path drawing controller
│       ├── marker_eval.cpp        # Camera + detection benchmarking tool
│       ├── frame_inspector.cpp    # Record N seconds, step through frames, inspect detections
│       ├── aruco_tracker_config.json
│       └── calibration/           # CMA-ES detector calibrator, lens (intrinsics) + homography tools, ChArUco generator
└── docs/
    └── architecture.md
```

---

## Prerequisites

### Firmware (all platforms)

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)

---

### PC Tools — macOS (Apple Silicon)

**1. Command Line Tools**
```bash
xcode-select --install
```

**2. Homebrew dependencies**
```bash
brew install opencv pkg-config sfml
```

> `game.cpp` targets the SFML 3 API (Homebrew's unversioned `sfml` formula). The
> older pinned `sfml@2` formula will not compile it.

**3. Basler pylon 8.1.0**

Download the macOS Universal Binary from [baslerweb.com](https://www.baslerweb.com/en/software/pylon/) and install. The framework will land at `/Library/Frameworks/pylon.framework`.

**4. WASD keyboard input**

Vision tools (`vision_controller`, `wingman`) and `swarm_controller` use WASD for direct leader control. On macOS this uses CoreGraphics; grant **Accessibility** permission when prompted (System Preferences → Privacy & Security → Accessibility).

---

### PC Tools — Ubuntu 22.04 / 24.04 (x86\_64)

**1. System packages**
```bash
sudo apt update
sudo apt install g++ make pkg-config \
                 libopencv-dev \
                 libsfml-dev
```

> Ubuntu 22.04/24.04's `libsfml-dev` provides SFML 3, which is the API `game.cpp` targets.

**2. Add yourself to the `dialout` and `input` groups**
```bash
sudo usermod -aG dialout,input $USER
# Log out and back in, or run: newgrp dialout && newgrp input
```

> * `dialout` is required for USB serial access to the robots/dongle.
> * `input` is required for multi-key WASD input in `vision_controller`, `wingman`,
>   and `swarm_controller`. They poll `/dev/input/eventN` directly via evdev
>   (`tools/swarm/evdev_keys.h`) rather than the X11 `XQueryKeymap`, because
>   `XQueryKeymap` only reflects keys delivered to an X11 surface — under a
>   Wayland session (the Ubuntu default) it silently reports nothing pressed,
>   since XWayland never receives input meant for native Wayland clients like
>   the terminal these tools run in. evdev reads the kernel input layer
>   directly, so it works under X11, XWayland, and Wayland alike. Without
>   `input` group membership these tools print a warning and run with WASD
>   disabled — `/dev/input/eventN` is `root:input 0660` by default.

**3. Basler pylon 8.1.0**

Download the Ubuntu amd64 `.deb` from [baslerweb.com](https://www.baslerweb.com/en/software/pylon/) and install:
```bash
sudo apt install ./pylon_8.1.0.*_amd64.deb
```

Pylon installs to `/opt/pylon/`. Add its `bin/` to your PATH so the Makefile's `pylon-config` queries work:
```bash
echo 'export PATH=/opt/pylon/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

> `pylon-config --libs` only emits a link-time `-L/opt/pylon/lib`, and `/opt/pylon/lib`
> is not registered with `ldconfig`. Without an rpath the built binaries fail at
> runtime with `error while loading shared libraries: libpylonbase.so.10: cannot
> open shared object file`. The Makefile adds `-Wl,-rpath,/opt/pylon/lib` to
> `PYLON_LIBS` on Linux to fix this.

**4. GigE camera NIC setup** (if using a Basler camera)
```bash
# Set NIC MTU to 9000 for jumbo frames (replace eth1 with your camera NIC)
sudo ip link set eth1 mtu 9000
# Set a static IP on the same subnet as the camera (default 169.254.x.x)
sudo ip addr add 169.254.1.1/16 dev eth1
```

**5. WASD keyboard input**

WASD reads `/dev/input/eventN` directly via evdev — see the `input` group note in
step 2 above. No graphical session or `DISPLAY` is required; this also means it
works the same over SSH as it does locally.

---

## Quick Start

### 1. Flash the dongle

```bash
pio run -e dongle -t upload
```

### 2. Flash each robot

Robot ID is baked in at flash time (default: 0):

```bash
ROBOT_ID=0 pio run -e receiver -t upload
ROBOT_ID=3 pio run -e receiver -t upload
```

### 3. Build PC tools

```bash
cd tools
make
```

### 4. Find the dongle serial port

**macOS:**
```bash
ls /dev/tty.usbmodem*
```

**Ubuntu:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

### 5. Run the hub

```bash
# macOS
./build/swarm_hub /dev/tty.usbmodem*

# Ubuntu
./build/swarm_hub /dev/ttyACM0
```

Then launch any controller (see below). Vision tools auto-launch the hub if a dongle is detected.

---

## PC Tools

### `swarm_hub`
Serial ↔ Unix socket bridge. All other tools connect to it via `/tmp/swarm_hub.sock`. Launched automatically by vision tools if a USB dongle is detected.

```bash
./build/swarm_hub /dev/tty.usbmodem*    # macOS
./build/swarm_hub /dev/ttyACM0          # Ubuntu
./build/swarm_hub --daemon /dev/ttyACM0
```

---

### `swarm_terminal`
Terminal UI showing all registered robots with RSSI, latency, battery, and motor state.

```bash
./build/swarm_terminal
```

---

### `swarm_controller`
Interactive keyboard controller and test suite. Drive individual robots or run automated test sequences. WASD drives robots (requires `input` group membership on Ubuntu — see Prerequisites).

```bash
./build/swarm_controller
```

---

### `latency_plot`
Live ASCII latency plot for a specific robot. Shows round-trip ping time in µs.

```bash
./build/latency_plot <robot_id>
```

---

### `swarm_telemetry_json`
Headless telemetry producer: connects to `swarm_hub`, subscribes to the vision hub for poses, and writes one JSON object per tick to stdout. Sends no motor commands. Exists so UIs that aren't C++ can consume the swarm without reimplementing the wire protocol — it's what `dashboard-ink` runs underneath.

It does **not** open the camera by default. The Basler allows one application at a time, so any tool that owns it — a demo, or [`vision_hub`](#vision_hub-opt-in-camera-owner) — publishes poses on `/tmp/vision_hub.sock` (see `lib/ArucoTracker/pose_hub.h`) and this subscribes — which is what lets a dashboard run alongside `circle_demo` or `vision_controller`. `--camera` opens the device directly for standalone use, and locks those demos out while it runs.

```bash
./build/swarm_telemetry_json [--interval MS] [--no-vision] [--camera]
./build/swarm_telemetry_json --no-vision | jq .        # inspect the stream
```

---

### `dashboard-ink` (Ink/React terminal dashboard)
Rework of `swarm_dashboard` as a Node/Ink TUI: one row per robot (32 fit on a screen), severity-coloured latency and battery, L/R drive in a single half-block meter, a square arena minimap with heading arrows, and a focus panel for the selected robot. Keyboard: `↑↓` select, `f` follow the worst robot, `s` cycle sort, `p` pause, `q` quit.

Needs Node ≥ 20. It is a separate toolchain from the Makefile — but has no build step of its own, only `npm install`.

```bash
cd tools && make build/swarm_telemetry_json   # the data producer it spawns
cd dashboard-ink && npm install

npm start                  # live; subscribes to the vision hub for poses
npm run demo               # synthetic swarm — no dongle, hub or camera needed
npm test                   # glyph + layout unit tests (node:test)
node src/cli.js --camera   # own the camera instead (locks vision demos out)
```

Poses come from whichever tool owns the camera, so this can run alongside a vision demo. The status bar tags the source: `cam 116fps·hub` (subscribed) vs `cam 116fps·own` (this process holds the device).

The C++ `swarm_dashboard` is unchanged and still works; the two can be compared side by side.

---

### `vision_hub` (opt-in camera owner)
The Basler admits **one** application, so normally whichever vision tool you start owns the camera and a second one fails with `0xE1018006`. `vision_hub` is an optional daemon that owns the camera *instead of a demo*, and shares what it sees three ways:

| What | How | Who uses it |
|---|---|---|
| **Poses** (where the robots are) | Unix socket `/tmp/vision_hub.sock` | every tool; `swarm_telemetry_json` subscribes to it directly |
| **Frames** (the picture, with the tracker's overlay) | POSIX shared memory `/swarm_vision_frames` | tools that draw on or click in the image: `circle_demo`, `wingman`, `shape_demo`, `vision_controller`, `drag_drop_demo`, `max_speed_test`, `circle_speed_test`, and `car_following` / `battery_log` with `--debug` |
| **Video** | MJPEG on `127.0.0.1:8081` | a browser, over an SSH tunnel |

```bash
./build/vision_hub                 # foreground (Ctrl-C stops it)
./build/vision_hub --daemon        # detach; PID in /tmp/vision_hub.pid, log in /tmp/vision_hub.log
./build/vision_hub --stop          # stop the daemon

./build/vision_hub [--stream-port N] [--stream-fps F] [--stream-width W] [--stream-quality Q]
                   [--no-stream] [--shm-fps F] [--no-shm] [--robots N]
                   [--serial SN] [--ip IP] [--homography FILE]
```

Nothing starts it for you. **With no hub running, every tool opens the camera itself exactly as before** — no extra hop, no change in latency.

#### Which tools attach, and when
Tools try the normal path first: they `open()` the camera if it is free, and only when it is held do they fall back to attaching to its poses (and frames, if they draw). So the hub is what *opts tools in*:

```
hub stopped :  ./build/circle_demo        → owns the camera, as always
hub running :  ./build/circle_demo        → "the camera is held by another process — attached to its
                                             published poses and shared frames instead."
```

Two things always own the camera themselves, never attach:
- **Calibration runs** (`--calibrate`, or the calibration keys): a homography fitted in an attached tool would not match the poses the hub publishes, and `setHomography()` refuses while attached. Stop the hub, calibrate, then restart it.
- **Evaluation tools** (`marker_eval`, `measurement_test`, `frame_inspector`, the `intrinsics`/`homography`/`calibrate` tools): they measure the camera and detection themselves.

A tool that needs frames will **not** attach to a plain camera owner (for example another demo, or `swarm_telemetry_json --camera`), since those share poses but no frames; it says so and exits. Headless tools (`car_following`, `battery_log` without `--debug`) only need poses and attach to anything.

#### Watching the camera from another machine
```bash
ssh -L 8081:localhost:8081 user@robot-pc       # on your laptop
# then open  http://localhost:8081/             (also /stream, and /snapshot.jpg for one frame)
```
The stream is the tracker's own frame (marker outlines, ids, headings), raw sensor pixels, downscaled to 960 px wide at JPEG quality 70 by default. It binds to loopback only; the SSH tunnel is the way in. A viewer that cannot keep up simply misses frames — there is no backlog, so a slow link never adds latency. Overlays a demo draws for itself are *not* in the stream: the demo draws on its own copy.

#### Cost
Everything is demand-driven, so a hub nobody is using costs almost nothing: no viewer means no encoding, no attached tool asking for frames means no copying. Measured on the lab machine (2048×2048 BGR, 12.6 MB per frame):

| | |
|---|---|
| Detection with the hub streaming | unchanged (~116 fps with a marker in view) |
| MJPEG stream | ~28 fps, ~900 kB/s, ~10 ms to encode — on its own thread |
| Shared-memory segment | 3 × 12.6 MB = 37.7 MB |
| Copy into shared memory | ~5–7 ms per frame, on a worker thread, only while a tool is asking, at most `--shm-fps` (default 30) |
| Attached tool, `debugFrame()` | ~0 ms in the control thread: a prefetch thread keeps the newest frame copied out (~4 ms, ~8 ms at p90) |
| Attaching to a held camera | ~0.8 s slower to start (the failed `open()` attempt) |
| Hub restart under an attached tool | frames freeze on the last good one, then resume ~2 s after the new hub is up; no tool restart needed |

#### Things to know
- **The shared frame is up to one hub frame (~33 ms) older than the poses.** An owner's frame is exact; attached tools see the newest copy the hub made. Fine for overlays and clicking, not for anything that needs pixel-exact alignment with a pose.
- **Detection settings are the hub's.** Exposure, ROI, dictionary, `--robots`/`--count` and `debug_overlay` on an attached tool do nothing; set them on the hub (`aruco_tracker_config.json`, or the flags above).
- **Restart the hub after recalibrating.** Attached tools read `aruco_homography.yml` for their own pixel↔world drawing; the hub reads it only at start. If the hub has no usable homography its poses are pixels, and attached tools run in pixels too (they refuse their local file rather than treat pixels as mm).
- **Hub down means attached tools stall**, the same as a stalled camera: `update()` reports nothing fresh, they reconnect on their own when it comes back, and the robots' on-board watchdog stops the motors meanwhile.
- **The HUD's `Radio RTT` column is not control latency.** It is the dongle's ESP-NOW ping round trip to that robot (dongle → robot ESP32 → back, timed on the dongle), sampled once per robot per ping interval — so it excludes the PC/USB legs, the UART hop to the RP2040 and the motors, and can be a few seconds old.
- **Shared memory** lives in `/dev/shm`; the hub removes it on a clean exit. `--no-shm` shares no frames (poses-only tools still attach).

---

### `vision_controller`
Main vision-based controller. Overhead camera tracks ArUco markers; click to set movement goals. WASD drives the leader robot (requires `input` group membership on Ubuntu — see Prerequisites).

```bash
./build/vision_controller [--serial SN] [--ip IP] [--calibrate]
```

| Key | Action |
|-----|--------|
| Left-click | Set goal for all robots |
| Right-click | Set goal for selected robot |
| `0`–`9` | Select robot |
| `s` | Stop all, unlock leader |
| `c` | Re-run homography calibration |
| `+` / `-` | Speed ±10% |
| `WASD` | Drive leader directly |
| `q` / Esc | Quit |

---

### `wingman`
V-formation controller. Leader is driven with WASD; all other robots autonomously hold positions in a V behind it.

```bash
./build/wingman [--serial SN] [--ip IP] [--spacing MM] [--dist MM] [--speed PCT]
```

| Key | Action |
|-----|--------|
| `WASD` | Drive leader |
| `+` / `-` | Formation spacing ±25 mm |
| `s` | Stop all |
| `c` | Re-calibrate |
| `q` / Esc | Quit |

> **macOS:** `wingman` uses a CGEventTap and requires Accessibility permission.  
> **Ubuntu:** Polls `/dev/input/eventN` directly via evdev — requires `input` group
> membership (see Prerequisites above). Works under X11, XWayland, and Wayland alike.

---

### `circle_demo`
Robots orbit a point you click. Speed, radius, and orbit direction are adjustable live.

```bash
./build/circle_demo [--serial SN] [--ip IP] [--radius MM] [--min-gap MM]
              [--orbit-speed DEG_S] [--speed PCT]
```

| Key | Action |
|-----|--------|
| Left-click | Set orbit centre |
| `0`–`9` | Select robot (again to deselect) |
| `+` / `-` | Radius ±25 mm (or speed ±10% when robot selected) |
| `[` / `]` | Orbit speed ±5 °/s |
| `t` | Toggle orbit tracking |
| `s` | Stop all |
| `q` / Esc | Quit |

---

### `car_following`
Runs the [Sugiyama et al. (2007)](https://iopscience.iop.org/article/10.1088/1367-2630/10/3/033001/meta)
ring-road experiment on real robots: each robot follows the one ahead using one
of seven car-following models, and — at a tight enough time gap — the ring
spontaneously develops the phantom traffic jam the experiment is famous for.

Unlike the other vision tools this is **headless by default** (one status line
per second, no window); `--debug` opens the usual view and HUD.

```bash
./build/car_following [--model NAME] [--speed-max M/S] [--car-size M] [--time-gap S]
                      [--reaction-time S] [--sigma A] [--sim-length M] [--radius MM]
                      [--centre X Y] [--ring-file PATH] [--fit] [--dir cw|ccw]
                      [--time-scale K] [--robot-max-speed MM_S] [--start]
                      [--buffer-b B] [--buffer-id ID]
                      [--bridge] [--port N] [--debug] [--count N]
```

Models: `Reuschel` `Pipes` `OVM` `CF-OVM` `FVDM` `ATG` `IDM` (default `FVDM`, the
NetLogo page's default). Parameter defaults match that page's sliders.

**Setup, cue, run.** The tool comes up in *setup*: camera, hub and ring are live
and every motor is held at zero, so the robots can be placed and the ring dialled
in without anything driving off. A run starts on a cue and continues until it is
stopped; stopping returns the models to rest, so the next run begins from
standstill the way the experiment's own setup does.

| Cue | Start | Stop |
|---|---|---|
| NetLogo page (`--bridge`) | press **Move** | press **Move** again, or **Setup** |
| `--debug` window | `space` | `space` or `s` |
| headless terminal | `<enter>`, `go` | `s` or `stop` (`q` quits) |
| launch flag | `--start` | — |

A cue is latched rather than obeyed on the spot: the run begins on the first
frame where the hub is connected, the ring has a radius, the roster has settled
and at least one robot is in view, and it says once what it is still waiting for.

`--count N` pins the number of vehicles the virtual ring is sized for. Without
it the count is the *settled* roster (a change has to hold for a second), so a
dropped detection cannot rescale the model mid-run; a robot that blinks out also
keeps its place on the ring for a second, so its follower keeps braking for it.

**The ring** is a saved fixture of the arena, kept in `/tmp/car_following_ring.yml`
in the same format `circle_demo` uses for its circle — so if you have already
calibrated a circle there, `/tmp/circle_demo.yml` is read as a fallback and no
setup is needed. Set it with `--centre X Y` / `--radius MM`, or interactively in
`--debug`; every change is written straight back, so the ring one run ends with
is the ring the next one starts on.

| Key / action | Effect (in `--debug`) |
|---|---|
| `space` | Run / stop |
| `s` | Stop, back to setup |
| Left-click | Move the ring centre there |
| `+` / `-` | Radius ±25 mm |
| `f` | Fit the ring to the robots that are visible |
| `,` / `.` | Time scale ÷ / × 1.25 |
| `q` / Esc | Quit |

`--fit` does that same fit once at startup: it takes the centroid of the visible
robots and their mean distance from it, waiting up to 5 s for at least three to
be detected. That is only a ring if the robots are already standing on one — it
used to be the automatic startup behaviour, and a scatter that was slightly off,
or a robot not yet detected, produced an off-centre ring the controller then
fought for the whole run (and which silently rescaled the model, since the
simulated-metres-per-mm factor divides by the radius). Hence: opt-in, and saved
like any other edit.

**Scale.** The models are written in the paper's units (a 230 m ring, 5 m cars,
15 m/s) and the arena is under a metre across, so positions and speeds are
converted through one factor. What that factor preserves is *density* — the wave
depends on metres per vehicle, not on the ring's absolute size — so by default
the physical ring maps to `N × (230/22)` simulated metres for however many robots
are on it, and four robots see the spacing 22 cars see in the paper.
`--sim-length` pins the virtual ring length instead.

**`--time-scale` — start here.** That factor maps *space* only. Time was mapped
1:1, so a lap took as long on a 300 mm ring as it does on the paper's 230 m one,
and the models asked for motor commands the robot cannot deliver:

| Robots | Ring 250 mm | 300 mm | 350 mm |
|---|---|---|---|
| 3 | 114 | 137 | 159 |
| 4 | 85 | 102 | 120 |
| 5 | 68 | 82 | 96 |
| 6 | 57 | 68 | 80 |

(steady-state motor command at `K=1`, FVDM defaults, `--robot-max-speed 300`;
100 is full throttle, so the top rows are clamped.)

`--time-scale K` supplies the missing half: K real seconds become one simulated
second. The model integrates a dt that is K times smaller and the commanded
speed is scaled back down by K — so the loop stays self-consistent and the
trajectories and the wave are unchanged, the whole experiment just runs K times
slower in wall clock. Divide the table above by K. `--time-scale 4` is a sane
starting point; `,` and `.` adjust it live in `--debug`. The heading controller
is deliberately untouched by it and keeps running in real time.

Above roughly `K=10` a model tick's travel shrinks into the tracker's own
position noise (at `K=10`, four robots on a 300 mm ring move ~3 mm per 100 ms
tick) — hence the `--time-scale` ceiling of 50. That shows up as jitter in the
*reported* speed only: what couples the models to the robots is the gap, which
is a whole vehicle spacing and far above the noise floor.

**Where each speed comes from.** Each vehicle's speed is the model's own state,
integrated by `cfStep()`; vision supplies positions, and therefore gaps, which
is what couples the models to each other and to the real ring. The status line
and HUD show the model speed next to the one measured from the ring, so a robot
falling behind what it was asked for is visible. Feeding the measurement *back*
into the model is what the first version did, and it is why nothing moved: for
the second-order models `cfStep()` returns `speed + a·dt`, so the command was
never more than one Euler step above what the robot had already managed, and a
robot's speed lags its command. From standstill that step is millimetres per
second — below the tool's own floor for commanding a motor, so the wheels never
turned and the measurement stayed at zero with them.

`--robot-max-speed` is the robot's physical speed (mm/s) at motor command 100 and
is what converts simulated m/s into motor units — measure it once for your
robots if the motion looks uniformly too fast or too slow.

**Cooperative buffering.** `--buffer-id ID` nominates one robot as the
*buffering vehicle*, after
[Korbmacher & Tordeux](https://vzu.uni-wuppertal.de/fileadmin/site/vzu/Stop-and-Go_Mitigation_via_Cooperative_Buffering.html):
it keeps `B` times the nominal time gap (`--buffer-b B`), while the other `N-1`
keep `(N-B)/(N-1)` of theirs. The mean time gap — and so the density — is
unchanged, which is what makes it a strategy rather than a disguised reduction
in traffic; the one enlarged gap gives that robot room to decelerate gently, and
the wave dies at it instead of being reflected back. `B = 1` is the
non-cooperative baseline.

This is a change to the *desired spacing*, so it is not IDM-specific: every
model here that has one takes it — `Reuschel`, `OVM`, `CF-OVM` and `FVDM`
through `V(s)`, `ATG` through `T0`, `IDM` through `s*`. **`Pipes` is the
exception**: its `dv/dt = Δv/τ` never reads the gap, so it has no desired
spacing to enlarge and buffering does nothing under it (the tool says so at
startup rather than appearing to act).

More buffering is not always better, though — the compensation is simultaneously
tightening the other `N-1`. Under `IDM`, the model the strategy was published
for, damping is monotone; under `FVDM` and `CF-OVM` a large `B` destabilises the
followers before the buffer can absorb anything, so sweep `B` rather than
assuming. Two guards apply on hardware that the reference page does not need:
`B` is capped where the followers reach half the nominal gap (the page can
afford a fixed slider max of 10 because it always has 20 vehicles; three robots
cannot), and buffering is suspended whenever the buffering robot is not
currently on the ring, since otherwise every remaining robot would tighten up
with nothing holding the space open.

**Live bridge.** `--bridge` serves the vendored NetLogo page from
`tools/car-following-models/` at `http://127.0.0.1:8770/` (loopback only) with a
small script appended that reports the model chooser, the slider values and the
run state back as they change. The robots then follow whatever the page is set
to and run when the page runs — its **Move** button is the cue and **Setup**
returns them to rest — so the simulation and the real ring run the same dynamics
side by side. The vendored HTML on disk is never modified: the script is
injected at serve time, and the bridge carries UI parameters only, never `MSG_*`
frames.

That script also injects a small **Cooperative buffering** panel of its own
(bottom right) with fields for `B` and the buffering robot's id, which ride out
on the same POST as the NetLogo widgets. They are not NetLogo widgets because
the vendored page is a fixed artefact, and because a robot id has no meaning
inside the simulation.

---

### `shape_demo`
Draw shapes on the camera view; robots slowly trace the path.

```bash
./build/shape_demo [--serial SN] [--ip IP] [--speed PCT]
```

| Key / Action | Effect |
|---|---|
| `l` / `r` / `o` / `f` | Switch to line / rectangle / circle / freehand |
| Left-drag | Draw shape |
| Right-click | Undo last shape |
| `x` | Clear all |
| `s` | Save path |
| `d` | Toggle Draw / Track mode |
| `q` / Esc | Quit |

---

### `marker_eval`
Camera and detection benchmark. Always opens the camera itself, so stop a running `vision_hub` first. Shows per-marker detection rate, live FPS, resolution, and **Detect time** (the detection thread's smoothed processing time per frame — not photon-to-pose: exposure, readout, the GigE transfer and the wait for `update()` are not in it). Run this first to verify the camera and ArUco config are working correctly.

```bash
./build/marker_eval [--config JSON] [--serial SN] [--ip IP]
              [--expected 0,1,2] [--mirror]
```

| Key | Action |
|-----|--------|
| `r` | Reset evaluation counters |
| `q` / Esc | Print summary and quit |

---

### `frame_inspector`
Records a short burst of frames at the camera's current fps, then lets you step through them one at a time to inspect motion blur, exposure, or detection quality. Frames live only in memory and are dropped when the program exits — nothing is written to disk.

```bash
./build/frame_inspector [--config JSON] [--serial SN] [--ip IP] [--seconds N] [--mirror]
```

| Key | Action |
|-----|--------|
| `Left` / `Right` | Step one frame backward / forward |
| `Home` / `End` | Jump to first / last frame |
| `d` | Run the ArUco detector once over every recorded frame, then toggle the overlay |
| `q` / Esc | Quit |

---

### `intrinsics` and `homography`

Lens calibration from a printed ChArUco board (`make_charuco.py`, same `DICT_4X4_50`
the tracker detects) and a many-point pixel-to-mm homography with a held-out error
estimate. Run them once per camera setup, lens first; see
`tools/vision/calibration/README.md`.

```bash
cd tools/vision/calibration && python3 make_charuco.py   # print charuco_board.pdf at 100 %
cd ../.. && make
./build/intrinsics --square-mm <measured> --cam-height-mm 1200
./build/homography --arena 800 600 --plane-mm <marker height>
```

### `calibrate`
CMA-ES optimiser that tunes `aruco_tracker_config.json` for the current lighting. Run once when setting up in a new room or after changing lighting conditions.

```bash
cd vision/calibration

../../build/calibrate             # static: markers held still
../../build/calibrate --motion    # motion: robots driving at operating speed
```

See `vision/calibration/README.md` for full flag reference and scoring details.

---

## Wire Protocol

All frames: `[0xAA][0x55][type][len][payload…][CRC-8]`

| Type | Code | Direction | Payload |
|------|------|-----------|---------|
| SWARM | 0x10 | PC → Broadcast | `[id\|L\|R] × 32 = 96 bytes` |
| ANNOUNCE | 0x20 | Robot → Broadcast | `[id, MAC×6]` |
| ANNOUNCE_ACK | 0x21 | Dongle → Broadcast | `[id]` |
| PING | 0x22 | PC → Robot | `[target_id, timestamp×4]` |
| PONG | 0x23 | Robot → Dongle | `[id, echo_timestamp×4]` |
| TELEMETRY | 0x30 | Robot → Dongle | `[id, mV×2, flags, mL, mR, uptime×2]` |
| SPEED | 0x01 | ESP32 → RP2040 | `[left, right]` |

---

## Camera Setup (Basler ace2 GigE)

1. Connect camera to a dedicated GigE port — use a 5GbE NIC for full bandwidth.
2. Set the NIC to a static IP on the same subnet as the camera (default: `169.254.x.x`).
3. Set MTU to 9000 (jumbo frames) for maximum throughput.
4. Pass `--serial SN` or `--ip IP` to any vision tool, or set `baslerSerial` / `baslerIp` in `aruco_tracker_config.json`.
5. Run `./build/marker_eval` to verify FPS and detection before using a controller.
