# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-NOW based swarm control system for up to 32 Pololu 3pi+ 2040 robots. A controller PC sends motor commands over USB serial to a dongle ESP32, which broadcasts them via ESP-NOW to all robot ESP32s; each robot ESP32 forwards commands to its onboard RP2040 (running MicroPython) over UART. A Basler ace2 GigE camera with OpenCV ArUco tracking provides overhead vision for the vision-based control modes. End-to-end latency from keypress to motor response is ~4ms — see `docs/architecture.md` for the full per-segment latency budget and `PERFORMANCE.md` for measured PC-tool performance history.

The codebase has four independent toolchains that don't share a build system: PlatformIO firmware (C++), a plain Makefile for PC tools (C++), MicroPython deployed directly to the robot (no build step at all), and npm for the Ink terminal dashboard (`tools/dashboard-ink/`, Node — dependencies only, also no build step).

## Commands

### Firmware (PlatformIO — `src/dongle`, `src/receiver`)

```bash
pio run -e dongle                        # build dongle firmware
pio run -e receiver                      # build receiver (robot) firmware
pio run -e dongle -t upload              # flash dongle
ROBOT_ID=3 pio run -e receiver -t upload # flash a robot with a specific ID (default 0)
```

`ROBOT_ID` is consumed by `extra_script.py`, which regenerates `lib/SwarmProtocol/robot_id_cfg.h` (gitignored) before every build — PlatformIO/SCons recompiles automatically when that header changes.

### PC tools (`tools/`, plain Makefile)

```bash
cd tools
make              # builds all tools + the CMA-ES calibrator into tools/build/
make clean        # removes tools/build/
make build/vision_controller         # build a single tool (targets are build/<name> from within tools/)
```

Requires OpenCV, SFML 3 (not `sfml@2`), and Basler pylon 8.1.0 on the host — see README.md "Prerequisites" for the per-OS setup (Homebrew vs apt, `dialout`/`input` group membership on Linux, pylon rpath quirk).

### Robot firmware (`src/robots/`, MicroPython)

No build step. Deploy `robot_uart.py`, `screen_manager.py`, and `uart_controller.py` directly to the Pololu 3pi+ 2040 over MicroPython (`uart_controller.py` is the entry point / `main.py`).

### Ink dashboard (`tools/dashboard-ink/`, npm)

```bash
cd tools && make build/swarm_telemetry_json   # the C++ producer it spawns
cd dashboard-ink && npm install               # no build step; htm, not JSX
npm start          # live
npm run demo       # synthetic swarm — no dongle, hub or camera needed
npm test           # node:test unit tests for the glyph + layout functions
```

Requires Node >= 20. Uses `htm` tagged templates instead of JSX specifically so there is no transpile step — see the comment in `src/html.js` before adding a bundler.

### Tests (`tests/`, plain Makefile)

```bash
cd tests
make test         # builds and runs tests/build/test_protocol
make clean        # removes tests/build/
```

Covers the lens model's round trips and file-compatibility rules (`test_camera_intrinsics.cpp`), the CRC-8 framing pure functions (`crc8`, `buildFrame`, `validateFrame`, `frameSize` in `lib/SwarmProtocol/protocol.h`), the pose-hub publish/subscribe round trip (`test_pose_hub.cpp` — real sockets, no camera), the MJPEG server's routing, viewer counting and slow-client behaviour (`test_mjpeg_server.cpp`), the shared-memory frame ring's round trip, demand heartbeat, replacement and torn-frame freedom under a flat-out writer (`test_frame_shm.cpp`), the car-following models (`test_car_following.cpp`), the ring bookkeeping + run-state machine behind them (`test_car_following_ring.cpp`), and `battery_log`'s speed/stop bookkeeping (`test_battery_log.cpp`) with a small assert-based harness — no test framework dependency. There's no CI configured yet. Formation-math unit tests are still pending extraction of that logic into testable pure functions (see `TODO.md` under "Tooling / Tests"); `lib/CarFollowing/car_following.h` is the model for how that extraction should look.

## Architecture

### Wire protocol has three independent implementations

The frame format `[0xAA][0x55][type][len][payload...][CRC-8 (poly 0x07)]` and the message type table are defined three times, once per language runtime, and must be kept in sync **by hand**:

- `lib/SwarmProtocol/protocol.h` — canonical C++ definition, used by `src/dongle` and `src/receiver` firmware
- `lib/SwarmClient/SwarmClient.h` — PC-side header-only client; mirrors the same constants under an `SC_` prefix
- `src/robots/robot_uart.py` — MicroPython parser on the RP2040 (`UARTProtocol` class)

When changing a message type or payload layout, all three need updating; there's no shared codegen.

Keep it at three. Non-C++ UIs consume the swarm through `swarm_telemetry_json` (`tools/swarm/swarm_telemetry_json.cpp`), which links `SwarmClient` and `ArucoTracker` and emits one NDJSON snapshot per tick on stdout — that's how `tools/dashboard-ink/` gets its data. Parsing frames in a fourth language would add another hand-synced copy of this table, and vision data (pylon + OpenCV) isn't reachable from outside C++ anyway.

### `swarm_hub` is the only process that owns the serial port

PC tools never open the dongle's serial device directly. `swarm_hub` (`tools/swarm/swarm_hub.cpp`) bridges the USB-serial connection to a Unix socket at `/tmp/swarm_hub.sock`, and every other PC tool connects to that socket. Vision tools auto-launch `swarm_hub` as a daemon if a USB dongle is detected and the hub isn't already running.

For new PC tools, use `lib/SwarmClient/SwarmClient.h` rather than talking to the socket directly — it auto-connects (and will auto-launch `swarm_hub` via `fork`/`exec` if needed), builds outgoing `MSG_SWARM` frames from a `setSpeed()`/`flush()` call pair, and parses incoming `MSG_ANNOUNCE`/`MSG_TELEMETRY`/`MSG_PONG` frames into per-robot `RobotState`. Calling `poll()` regularly is required even if you don't care about telemetry — it's what reads and parses incoming frames (including the `MSG_PONG`s that populate `latencyUs`). The round-robin pinging that produces those pongs is driven centrally by `swarm_hub` (it snoops announce/telemetry/pong frames to learn live robot IDs and emits one `MSG_PING` per interval), **not** per-client: this keeps exactly one ping in flight no matter how many tools are connected, since the dongle's latency tracker (`src/dongle/main.cpp`, single `pingTracker`) only holds one outstanding ping at a time — multiple independent pingers would clobber it and corrupt RTT.

### `ArucoTracker` is the only process that owns the camera

The Basler admits exactly one application — a second `open()` gets `0xE1018006` ("device is controlled by another application") — so the camera cannot be multiplexed the way `swarm_hub` multiplexes the dongle. Instead, whichever tool opens it publishes tracked poses on `/tmp/vision_hub.sock` automatically (`lib/ArucoTracker/pose_hub.h`, wired into `ArucoTracker::open()`/`update()`); pose-only tools subscribe rather than opening the camera.

The rule this buys: **only tools that need pixels own the camera.** Every vision demo calls `debugFrame()`/`cv::imshow`, so they own it and publish. `swarm_telemetry_json` needs poses only, so it subscribes by default and a demo can always start alongside it. `--camera` makes it own the device instead, for standalone use — at the cost of locking demos out.

`vision_hub` (`tools/vision/vision_hub.cpp`) is the **opt-in** way to be that owner without a demo: start it by hand and it holds the camera, publishes poses, and streams the tracker's frame as MJPEG on `127.0.0.1:8081` (`ssh -L 8081:localhost:8081 <host>`, then `/`, `/stream` or `/snapshot.jpg`). Nothing auto-launches it, so by default every tool still opens the camera itself with no extra hop. It is the *only* place the stream is served from — keep the tracker free of HTTP and encoding. Resize, JPEG encode and all socket I/O run on a second thread; the main thread only calls `update()` and, while `MjpegServer::wantsFrames()`, hands over a `cv::Mat` header (no pixel copy — the tracker never writes into a frame it has handed out). Nobody watching means no encoding. Measured on the real camera: detection stays at ~116 fps while streaming ~28 fps at ~900 kB/s. A second hub refuses to start (it probes `vision_hub.sock` first, because `open()` ignores that refusal).

**Subscriber mode** (`ArucoTracker::attach()` / `openOrAttach()`) is how a tool runs alongside a hub — or alongside any demo, since they all publish on the same socket. `openOrAttach()` takes **the normal path first**: it `open()`s the camera if it is free, and only when that fails (something holds it) falls back to attaching, so a tool never gives up the direct, no-extra-hop path just because a hub could exist, and starting a hub is what opts tools into attaching. Attached, it opens no camera and starts no detection threads; `update()` fills `robots()` from the publisher's snapshots, and a vanished publisher is retried once a second while `update()` reports nothing fresh, exactly what a stalled camera looks like. `openOrAttach(needFrames)`: headless tools (`car_following`, `battery_log`) need poses only; tools that draw on the frame pass `needFrames = true` (all the demos, and the two above under `--debug`) and, unless they are calibrating, use it too — calibration calls `open()` directly because a homography fitted in an attached tool would not match the poses the hub publishes (`setHomography()` refuses while subscribed). Three things about it are easy to get wrong:
- **Coordinate space comes from the publisher, not the local homography file.** `PoseHubHeader::flags` carries `POSE_HUB_FLAG_WORLD`; `loadHomography()` in subscriber mode refuses (returns false) when the publisher's poses are pixels, since treating those as millimetres is off by an order of magnitude and nothing complains.
- **Frame size and lens model are the publisher's.** `attach()` waits (up to 1.5 s) for the first snapshot to learn the size, then loads intrinsics, so the homography's resolution and `undistorted` checks run against the same pixel space.
- **Detection settings are the hub's.** `--count`/`robotCount`, exposure, ROI and dictionary on the attached tool's `ArucoConfig` do nothing; `latencyMs()` is 0 (not measured across the process boundary).

**Shared frames** (`lib/ArucoTracker/frame_shm.h`) are what let the tools that draw on the frame attach too. The hub copies its debug frame (the tracker's overlay, 8-bit BGR, 12.6 MB at 2048²) into a POSIX shared-memory ring of three slots (`/swarm_vision_frames`), guarded by a per-slot seqlock so a reader never sees half a frame and neither side ever waits for the other. It is **demand-driven**: a reader stamps a heartbeat in the header, and the hub's worker thread copies (at most `--shm-fps`, default 30) only while that stamp is under a second old, so with no reader the cost is one atomic load and a crashed reader leaks nothing. The reader side lives in `ArucoTracker`: a small prefetch thread keeps the newest frame copied out (~4 ms, ~8 ms at p90 here) so `debugFrame()` in the tool's control thread is a pointer hand-over; it copies only while `debugFrame()` is being called, and re-opens the segment if frame ids stop advancing (a restarted hub makes a new one — the writer replaces, never truncates, since truncating a mapped file SIGBUSes its readers). Things to keep in mind: the shared frame is up to one hub frame interval (~33 ms) older than the poses where an owner's is exact; an attached tool's `ArucoConfig` detection settings (and `debug_overlay`) are the hub's, not its own; `marker_eval`, `measurement_test` and `frame_inspector` still call `open()` directly because they evaluate the camera and detection themselves; and **restart the hub after recalibrating** — attached tools load the homography file for their own pixel↔world drawing, the hub only reads it at start.

`lib/ArucoTracker/mjpeg_server.h` is the server (loopback only, non-blocking, takes finished JPEG bytes). A client that can't keep up misses frames rather than queueing them, so a slow link never adds latency. `/snapshot.jpg` waits for the next fresh frame and counts as demand.

`pose_hub.h` is deliberately free of OpenCV and pylon includes, so subscribers link neither, and so is `frame_shm.h`. Frames do not go through the pose socket (2048² at ~116 fps is ~470 MB/s — that is what the shared-memory ring is for, and it is demand-driven so the hub copies at most `--shm-fps` of them and only while a tool asks).

### Lens and homography calibration: points are undistorted, frames never are

`lib/ArucoTracker/camera_intrinsics.h` holds the lens model (K, D) that `tools/vision/calibration/intrinsics.cpp` fits from a printed ChArUco board (`make_charuco.py`, `DICT_4X4_50`). The camera is a plain 6 mm rectilinear Basler lens, so this is the standard `calibrateCamera` model — **not** the fisheye one (`FisheyeUndistortPreprocessor` is unused and nothing writes its YAML).

- **Undistort points, not frames.** `ArucoTracker` undistorts the marker centre and heading tip before the homography; a full-frame remap costs more than the whole ArUco detection at 115 fps. `RobotPose::px/py` and `debugFrame()` stay in the raw, distorted image, so anything that draws or clicks on it converts at the boundary.
- **Pixel space has two flavours, and the homography records which.** `aruco_homography.yml` carries `undistorted: 0/1`; `loadHomography()` refuses one fitted in the other space, as it already does for a different resolution. `camera_intrinsics.yml` is likewise rejected if the resolution or the `offset_x/offset_y` sensor ROI differ (K's principal point moves with the ROI).
- **Tool-local `pixelToWorld`/`worldToPixel` helpers** (every demo has its own copy) call `arucoUndistortPixel()` / `arucoDistortPixel()` around the homography, and their own `findHomography(cs.pixPts, …)` calibrate flows go through `arucoUndistortPixels()`. Both are the identity without a loaded lens, so the old behaviour is unchanged. New tools doing click-to-world must do the same.
- **`mirror_input`:** the tracker flips frames before detection, but K and D describe the unflipped sensor. `CameraIntrinsics::mirror` un-flips points before the lens model and flips back after.
- **`tools/vision/calibration/homography.cpp`** replaces the 4-corner click flow for serious calibration: more points (arena marks, or ChArUco placements), sub-pixel, and a leave-one-out / leave-one-placement-out error. A 4-point fit has zero residual by construction; only held-out error says anything. It should be run at the plane of the robot markers (`--plane-mm`): a floor calibration is off by `h·r/H_cam` for markers at height `h`.

### Robot registration and addressing

Each robot's ID is baked into firmware at flash time (`ROBOT_ID` env var, defaults to 0), not configured at runtime. On boot a robot broadcasts `MSG_ANNOUNCE` every 500ms until the dongle ACKs it; `swarm_hub`/`SwarmClient` track per-ID state (MAC, battery, motor state, last-seen) keyed off that ID. A `MSG_SWARM` frame is variable-length and only carries entries for currently-known robots (`SwarmClient::flush()` skips unknown IDs), not a fixed 32-slot array.

### Safety watchdogs live on the robot, not the PC

`WATCHDOG_TIMEOUT_MS` (`lib/SwarmProtocol/hardware.h`) stops motors if a robot's RP2040 hasn't received a fresh `MSG_SPEED` recently, independent of whether the PC/dongle/ESP-NOW link is still alive. Re-announce/expiry timing (`ANNOUNCE_TIMEOUT_MS`, `ROBOT_EXPIRY_MS`, `REANNOUNCE_INTERVAL_MS`) is also centralized there — check this header before touching timing-sensitive robot behavior.

### Vision pipeline is a separate concern layered on top of the swarm link

`lib/ArucoTracker/` wraps the Basler camera (via pylon) and OpenCV ArUco detection behind `aruco_tracker.h`. Vision-based controllers (`tools/vision/vision_controller.cpp`, `wingman.cpp`, `circle_demo.cpp`, `shape_demo.cpp`) consume tracked poses and then drive robots through the same `SwarmClient` as any other PC tool — vision and swarm control aren't otherwise coupled. Per `TODO.md`, this separation is still informal (most demos build ArUco poses and protocol frames inline rather than through a clean `PoseStream`-style interface); `drag_drop_demo.cpp` is cited there as the tool that already does this more cleanly via `SwarmClient`.

### `car_following` is headless-first, and its models are a separate pure library

`tools/vision/car_following.cpp` runs the Sugiyama ring experiment on real robots. Two things about it differ from the other vision tools deliberately:

- **Headless by default.** No `namedWindow`/`imshow`/`waitKey` unless `--debug` is passed; the default is a status line per second, like `swarm_telemetry_json`. This is the direction `TODO.md` "Webserver / Headless" wants the vision tools to move, so prefer this shape for new ones.
- **The physics lives in `lib/CarFollowing/car_following.h`** and the ring bookkeeping in `lib/CarFollowing/ring.h`, both free of OpenCV, pylon and SwarmClient and unit-tested (`tests/test_car_following.cpp`, `tests/test_car_following_ring.cpp`). The tool itself only does vision, control and I/O. This is exactly the extraction `TODO.md` asks for under "Tooling / Tests" for the formation math — use it as the template.

`ring.h` holds everything that used to be inline loop state and was the part that misbehaved on hardware: ring order, gaps, the roster, the scale factor, each vehicle's model speed, and the run-state machine. Three rules there are load-bearing and easy to undo by accident:

- **The model owns each vehicle's speed.** `cfStep()` returns `speed + a·dt` for the second-order models, so re-seeding `speed` from the vision measurement caps the command at one Euler step above whatever the robot has already achieved — and a robot's speed lags its command. From standstill that is millimetres per second, under the tool's own floor for commanding a motor, so the ring never moved at all. Vision closes the loop through the **gaps** instead, which is both the paper's coupling and a signal measured over a whole vehicle spacing rather than one tick's travel. The measured speed is reported, never fed back.
- **A robot that blinks out keeps its place on the ring** (`CfRingConfig::holdS`) even though its motors are cut sooner. Drop it from the ordering and its follower inherits the gap to the vehicle *beyond* it — roughly double — and accelerates into a robot that is still physically there.
- **The vehicle count is debounced** (`CfRingConfig::settleS`) or pinned with `--count`. `simPerMm` divides by it, so taking the live per-frame count means one dropped detection rescales every gap and every speed the models see.

The heading controller is `circle_demo.cpp`'s orbit controller (yaw feedforward + PD, `MAX_TURN_RATE` slew limit); only the source of the tangential speed setpoint differs — per robot from the model, rather than one global orbit rate. Fixes to that control law should land in both. Treat it as a **port, not a variant**: its inputs and its constants are circle_demo's, including the 0.5 s yaw low-pass (`YAW_TAU_S`), the one-control-period D-term window taken on top of that filtered signal, and the single `MAX_TURN` cap covering feedforward and feedback together. Two plausible-looking departures have each been tried on hardware and reverted, and both failed in the same direction — worse the faster the ring ran:

- **Dropping the yaw low-pass.** The argument for it is real (on a circle the true heading rotates at `v/R`, so a `tau` lag becomes a standing P-term error of about `tau·v/R`, ~9° at 100 mm/s on a 300 mm ring) but it buys a *bias* at the price of *oscillation*: unfiltered, a degree of ArUco yaw noise differenced over the ~10 ms D-term window reads as 100 °/s, which `K_YAW_D` turns into ~15 turn units against a `MAX_TURN` of 20. A fixed radius offset is the better failure. Widening the D-term window instead of filtering the heading does cut that noise, but it is not a substitute — it adds lag to the damping term specifically.
- **Splitting `MAX_TURN` into a per-half budget** so the PD keeps its full authority at every speed. The shared cap looks like it starves the feedback (the feedforward is `K_FF_YAW·vTan/R`, so it consumes a share growing linearly with speed, leaving the PD ±12 at circle_demo's default vTan of 40 and ±5 at the 80 this tool commands). But that cap is also a **gain limit that tightens as the loop speeds up**, and the loop needs it: its bandwidth already rises with speed, since a heading error converts to lateral offset at a rate proportional to `v`. Handing the PD more authority exactly where the margin is thinnest oscillates harder. The same applies to "protecting" turn authority by scaling `forward` down instead of letting `forward ± turn` clip — that couples the turn back into the speed and adds its own modulation.

Two invariants that controller depends on, both of which have already been broken once:

- **Its velocity field is in motor units, not mm/s.** `circle_demo` never converts — it feeds `vTan` straight to the motors as `forward` — so `K_FF_YAW` and `K_RAD` are tuned against that scale. `car_following` therefore converts the model's m/s all the way down to motor units (through `--robot-max-speed`) *before* the heading law sees them. Expressing the field in mm/s instead scales the yaw feedforward up and the radial pull down by `robotMaxMms / MOTOR_MAX`; since the feedforward is proportional to speed, the robots then hold a radius that depends on how fast they are going and separate into lanes, the fast ones orbiting inside the slow ones.
- **The model's speed is a state, not a per-tick measurement.** `cfStep` returns `in.speed + a·dt`, so passing it the vision-measured speed each tick closes a unity-gain loop around vision: the drive command becomes the robot's own last-100ms speed handed back nearly unchanged (noise included), and any measurement bias — on a ring, `ring.radius` divided by the robot's actual orbit radius — drives it away to `speedMax`, so the model stops running at all. Vision corrects the state through `cfSyncAlpha()` instead. The algebra is in `lib/CarFollowing/car_following.h` and both behaviours are pinned in `tests/test_car_following.cpp`.

The ring itself is a **saved fixture**, not something inferred per run: it lives in `/tmp/car_following_ring.yml` in the same key layout `circle_demo.cpp` writes its circle to (`cx`/`cy`/`centre_set`/`radius`), falls back to reading `/tmp/circle_demo.yml`, and is re-saved on every change — a flag, a click, a radius key. Fitting it to the robots' current positions is available but opt-in (`--fit`, or `f` in `--debug`), because it was the automatic startup behaviour and was the main reason runs misbehaved: the centroid-and-mean-radius of the detected poses is only a ring if the robots already sit on one, and a wrong radius doesn't just steer badly, it rescales the model (`simPerMm` divides by it). Prefer this shape — a persisted, explicitly-set arena geometry — over inferring geometry from wherever the robots happen to be.

Models are written in the paper's units (230m ring, 5m cars) and the arena is under a metre, so one scale factor maps between them. It is chosen to preserve **density** (metres per vehicle), not absolute size, because that is what determines whether the stop-and-go wave forms — hence the default of `N × (230/22)` simulated metres for N robots. N there is
the robot *population* — those seen within the last second, or `--count` — not
this frame's detections: the road length divides into every gap and every
measured speed, so deriving it from a frame that missed one marker rescales the
whole experiment for as long as the dropout lasts.

That factor maps **space only**, and time was originally 1:1 — which made the tool unusable on hardware, since a lap then takes as long on a 300mm ring as on the paper's 230m one (three robots at FVDM defaults ask for motor command 137, past full throttle). `--time-scale K` is the other half of the mapping: K real seconds per simulated second. It must be applied in all three places or the loop stops being self-consistent — measured speed divides by `simDt = modelDt/K`, `cfStep` integrates `simDt`, and the commanded `vTan` divides by K on the way out. The heading controller (yaw rate, PD, slew limit) stays in **real** time throughout and must not be scaled; only the model's clock is dilated. Scaling just the output instead would leave the model measuring robots that move slower than it commanded, and it would fight that.

The tool has an explicit lifecycle rather than driving from the moment the camera opens: it comes up in **setup** with every motor held at zero, and a run starts on a cue — the page's "Move" button under `--bridge`, `space` in `--debug`, a line on stdin when headless, or `--start`. A cue is *latched*, not obeyed on the spot: `CfRunState::update()` starts the run on the first frame where the hub is connected, the ring has a radius, the roster has settled and a robot is in view. A stop returns the models to rest, so the next run begins from standstill the way the experiment's own setup does. Keep new cue sources going through `CfRunState` rather than poking a phase flag.

The second cue, **align**, drives the robots onto an *initial condition* rather than a car-following model, and which one is `CfLayout` (`--init-layout`, or the page's injected "Initial position" selector): `Uniform`, the paper's own evenly spaced start, or `Jam` — everyone queued behind one leader (the buffering robot, else the lowest id on the ring) with the road ahead of it empty, so a run starts mid-wave. Both are the same mechanism: `CfRing::computeAlignTargets()` builds one angular offset per vehicle and `applySlotOffsets()` rotates that pattern onto the ring where it costs the least travel. Two things there are deliberate. The jam is driven as **two** maneuvers — spread evenly, *then* close up — because the queue is built out of the ring order the vehicles already sit in, and compressing a scattered ring straight into a queue is the case where two of them have to swap places around the ring to reach their slots; the tool sequences that with `alignStage`, and `CfRunState` only ever sees the end of the second stage. And the jam's spacing is the model's own bumper-to-bumper distance (one `carSize`) mapped back through the same density scale the models see, floored at `JAM_MIN_SPACING_MM` for the chassis and capped at the even-spread slot width, so it is as tight relative to the arena ring as 22 five-metre cars are on the paper's 230m one.

**Cooperative buffering** (Korbmacher & Tordeux's companion NetLogo page) is implemented as a transformation of `CfParams`, not as a branch inside any model: `cfBufferedParams()` gives the buffering vehicle `B * timeGap` and the other N-1 `timeGap * (N-B)/(N-1)`, holding the mean — and therefore the density — constant. Because it acts on the desired spacing, it applies to every model that has one; `Pipes` is the sole exception (`dv/dt = Δv/τ` never reads the gap), which `cfModelHasDesiredGap()` reports so callers can say so rather than silently no-op. Prefer this shape for further strategies from that family: a pure per-vehicle parameter transformation in `car_following.h`, applied by `CfRing::step()` (which also caps B against the live vehicle count and suspends buffering while the buffering robot is off the ring), with the tool deciding only *which* robot it applies to.

The `--bridge` page's two buffering fields and its "Initial position" selector are injected by `tools/vision/car_following_bridge.js`, not added to the vendored HTML — the same rule as the reporting script. `applyParams()` picks them up as `buffer-b`/`buffer-id`/`init-layout` alongside the scraped NetLogo widget values; this is still UI parameters over the HTTP bridge, never `MSG_*` frames, so the "keep it at three" rule is unaffected.

The page's third graph — the camera-measured trajectories — is injected the same way, and is deliberately drawn to the *same* conventions as the two NetLogo plots beside it rather than to whatever its own data spans: an absolute `0..TRAJ_WINDOW_S` (250 simulated seconds) time axis, one black pen for every vehicle, and the plot's own size, anchored to the right edge of the "Simulation" widget so the three sit in a row. The tool publishes that axis extent as `window` in `/trajectories` and clears `trajBuf` (carrying the overshoot, so sweeps stay a whole window apart) when the run's model clock fills it — the plots' own `clear-plot` every 2500 ticks. Earlier this was a *rolling* window with per-id colours and a data-fitted axis, which is what made it unreadable next to the others: the axis slid under the trace, so the same wave never sat at the same height twice and nothing lined up across the three graphs.

`lib/CarFollowing/http_bridge.h` is a ~150-line loopback HTTP server used by `--bridge` to serve the vendored NetLogo page and receive its slider/chooser values and run state back as `name=value` lines. It is **not** a fourth implementation of the swarm wire protocol — it carries UI parameters only, never `MSG_*` frames, so the "keep it at three" rule above is unaffected. Plain HTTP rather than WebSockets because the traffic is one small POST per parameter change over loopback; the handshake and framing a WebSocket needs would be larger than the whole file. The vendored HTML is never modified on disk — the reporting script is injected at serve time.

### `battery_log` measures speed against voltage, and depends on open-loop robots

`tools/vision/battery_log.cpp` is the first tool for the battery-runtime research: can remaining charge be estimated from the camera alone? It orbits one robot (`--robot`) on the saved ring at a fixed motor command until the battery is flat. It writes one CSV row per second (vision speed, battery mV, phase) and a PNG plot every minute into `battery_log_results/`. `tools/analysis/battery_log_plot.py` fits speed against voltage across runs. The full design (data flow, lifecycle, measurement, CSV format, open questions) is in `docs/battery-log.md`. Its shape copies `car_following`: headless-first, the ring as a read-only fixture (no fitting, and it refuses to run without a ring or a homography), and the camera-free bookkeeping in `lib/BatteryLog/battery_log.h`, tested by `tests/test_battery_log.cpp`. Its heading controller is another port of circle_demo's orbit law, taken from `car_following` with the yaw low-pass, so the "port, not a variant" rule above applies to it too.

Three things it relies on:

- **The measurement only exists because the robots run open-loop** (`ODOMETRY_ENABLED = False`). A command is then a fixed PWM duty and the speed follows the battery. With the encoder PID on, the speed holds until the motors saturate and the log is flat. Flipping that flag invalidates every comparison with earlier runs.
- **Speed is the displacement projected onto the ring tangent**, not the `hypot` of frame deltas. Position noise always adds to a magnitude, so the `hypot` version reads fast by an amount that depends on the frame rate. Rows that span a dropout don't count the gap's time.
- **Rests are part of the data, not downtime.** Every `--rest-every` seconds the motors are held at zero for `--rest-for` seconds. The resting voltage minus the loaded voltage is the cell's sag under the motor current, a second signal next to the speed. A rest ends in a reseek, and each row belongs to exactly one phase (hence the `dt_s` column). The battery is only sampled every 2 s (`BAT_INTERVAL_MS`) and telemetry repeats that sample, so low-voltage stops are debounced by time below `--stop-mv`, not by counting readings. `STATUS_LOW_BATTERY` is logged and obeyed, but nothing in the firmware sets it yet.

### MicroPython robot firmware: feature-flag + isolated-module pattern

`src/robots/uart_controller.py` is the main loop (UART receive → PID speed control → motor output → display), and it deliberately keeps optional features out of the core file: each one is gated by a module-level `..._ENABLED` flag and implemented in its own module, with the main loop calling at most one hook per loop iteration. Example: `ENGINE_SOUND_ENABLED` guards both the import of `engine_sound.EngineSound` and the single `engine.update(actual_l, actual_r, dt)` call inside `run_pid()`. Follow this pattern for new robot-local features (sound, additional sensors, etc.) rather than growing `uart_controller.py` or `robot_uart.py` directly — and keep such features computed from data the robot already has rather than extending the shared wire protocol, since that protocol is also implemented independently in the C++ firmware and PC client (see above).

`ODOMETRY_ENABLED` in the same file is a similar toggle but for swapping the *entire* control strategy (closed-loop PID on encoder counts/s vs. open-loop target-to-power mapping) rather than adding a feature — both code paths are kept intact so it can be flipped without re-deriving the open-loop math.

### Repo layout

```
src/dongle, src/receiver   — PlatformIO firmware (C++), built/flashed independently per platformio.ini env
src/robots/                — MicroPython firmware for the RP2040, deployed without a build step
lib/SwarmProtocol/         — wire protocol shared by both firmware targets (canonical C++ definition)
lib/SwarmClient/           — header-only PC client library; new PC tools should build on this
lib/ArucoTracker/          — camera + ArUco tracking abstraction (Basler pylon + OpenCV); camera_intrinsics.h is the lens model (OpenCV only); pose_hub.h / mjpeg_server.h are the OpenCV-free pose and video servers
lib/CarFollowing/          — car-following models, ring bookkeeping + run state, localhost HTTP bridge (no OpenCV/pylon/SwarmClient)
lib/BatteryLog/            — battery_log's row/stop/schedule bookkeeping (no OpenCV/pylon/SwarmClient)
tools/                     — Makefile + PC tool entry points (game.cpp, vision/*.cpp, swarm/*.cpp); binaries land in tools/build/
docs/architecture.md       — protocol/timing design doc (German)
docs/battery-log.md        — battery_log research tool: design, measurement, outputs
```

Build artifacts (`.pio/`, `tools/build/`, `lib/SwarmProtocol/robot_id_cfg.h`) are gitignored and regenerated by the commands above — don't hand-edit them.
