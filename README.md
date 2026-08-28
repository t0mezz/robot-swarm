# Robot Swarm

ESP-NOW based swarm control system for up to 32 Pololu 3pi+ 2040 robots. A controller PC sends motor commands over USB serial to a dongle ESP32, which broadcasts them via ESP-NOW to all robot ESP32s. Each robot ESP32 forwards commands to the RP2040 over UART. End-to-end latency is ~4 ms from keypress to motor response.

A Basler ace2 GigE camera running pylon 8.1.0 provides overhead ArUco marker tracking for vision-based swarm control modes.

---

## Architecture

```
Controller-PC  →  USB-Serial  →  Dongle-ESP32  →  ESP-NOW Broadcast  →  N× Robot-ESP32  →  UART  →  RP2040
                  921600 Baud                        ~1 ms latency                          921600 Baud

Basler ace2 GigE  →  pylon 8.1.0  →  OpenCV ArUco  →  vision_controller / wingman / circle_demo / shape_demo
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
│   ├── ArucoTracker/           # Camera abstraction + ArUco tracker (aruco_tracker.h, basler_pylon_source.h)
│   ├── SwarmClient/            # High-level swarm socket client (SwarmClient.h)
│   ├── Calibration/            # CMA-ES detector tuning (cmaes.h, param_space.h, objective*.h)
│   └── swarm/                  # PC-side host tools (swarm_hub, swarm_terminal, swarm_controller, latency_plot)
├── tools/
│   ├── Makefile                # Builds all PC tools (macOS arm64 + Ubuntu x86_64)
│   ├── game.cpp                # SFML game pad controller
│   ├── dashboard-ink/          # Ink/React terminal dashboard (Node; own package.json, no build step)
│   └── vision/
│       ├── vision_controller.cpp  # Main vision-based swarm controller
│       ├── wingman.cpp            # V-formation follower controller
│       ├── circle_demo.cpp        # Circle orbit formation
│       ├── car_following.cpp      # Sugiyama ring experiment (headless; optional NetLogo page bridge)
│       ├── shape_demo.cpp         # Freehand path drawing controller
│       ├── marker_eval.cpp        # Camera + detection benchmarking tool
│       ├── frame_inspector.cpp    # Record N seconds, step through frames, inspect detections
│       ├── aruco_tracker_config.json
│       └── calibration/           # CMA-ES detector calibrator
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

It does **not** open the camera by default. The Basler allows one application at a time, so any tool that owns it publishes poses on `/tmp/vision_hub.sock` (see `lib/ArucoTracker/pose_hub.h`) and this subscribes — which is what lets a dashboard run alongside `circle_demo` or `vision_controller`. `--camera` opens the device directly for standalone use, and locks those demos out while it runs.

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

**Live bridge.** `--bridge` serves the vendored NetLogo page from
`tools/car-following-models/` at `http://127.0.0.1:8770/` (loopback only) with a
small script appended that reports the model chooser, the slider values and the
run state back as they change. The robots then follow whatever the page is set
to and run when the page runs — its **Move** button is the cue and **Setup**
returns them to rest — so the simulation and the real ring run the same dynamics
side by side. The vendored HTML on disk is never modified: the script is
injected at serve time, and the bridge carries UI parameters only, never `MSG_*`
frames.

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
Camera and detection benchmark. Shows per-marker detection rate, live FPS, resolution, and pipeline latency. Run this first to verify the camera and ArUco config are working correctly.

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
| TELEMETRY | 0x30 | Robot → Dongle | `[id, rssi, bat, flags, mL, mR, uptime×2]` |
| SPEED | 0x01 | ESP32 → RP2040 | `[left, right]` |

---

## Camera Setup (Basler ace2 GigE)

1. Connect camera to a dedicated GigE port — use a 5GbE NIC for full bandwidth.
2. Set the NIC to a static IP on the same subnet as the camera (default: `169.254.x.x`).
3. Set MTU to 9000 (jumbo frames) for maximum throughput.
4. Pass `--serial SN` or `--ip IP` to any vision tool, or set `baslerSerial` / `baslerIp` in `aruco_tracker_config.json`.
5. Run `./build/marker_eval` to verify FPS and detection before using a controller.
