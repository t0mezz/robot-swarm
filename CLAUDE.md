# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-NOW based swarm control system for up to 32 Pololu 3pi+ 2040 robots. A controller PC sends motor commands over USB serial to a dongle ESP32, which broadcasts them via ESP-NOW to all robot ESP32s; each robot ESP32 forwards commands to its onboard RP2040 (running MicroPython) over UART. A Basler ace2 GigE camera with OpenCV ArUco tracking provides overhead vision for the vision-based control modes. End-to-end latency from keypress to motor response is ~4ms — see `docs/architecture.md` for the full per-segment latency budget and `PERFORMANCE.md` for measured PC-tool performance history.

The codebase has three independent toolchains that don't share a build system: PlatformIO firmware (C++), a plain Makefile for PC tools (C++), and MicroPython deployed directly to the robot (no build step at all).

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
make tools/build/vision_controller   # build a single tool
```

Requires OpenCV, SFML 3 (not `sfml@2`), and Basler pylon 8.1.0 on the host — see README.md "Prerequisites" for the per-OS setup (Homebrew vs apt, `dialout`/`input` group membership on Linux, pylon rpath quirk).

### Robot firmware (`src/robots/`, MicroPython)

No build step. Deploy `robot_uart.py`, `screen_manager.py`, and `uart_controller.py` directly to the Pololu 3pi+ 2040 over MicroPython (`uart_controller.py` is the entry point / `main.py`).

### Tests (`tests/`, plain Makefile)

```bash
cd tests
make test         # builds and runs tests/build/test_protocol
make clean        # removes tests/build/
```

Currently covers the CRC-8 framing pure functions (`crc8`, `buildFrame`, `validateFrame`, `frameSize` in `lib/SwarmProtocol/protocol.h`) with a small assert-based harness — no test framework dependency. There's no CI configured yet. Formation-math unit tests are still pending extraction of that logic into testable pure functions (see `TODO.md` under "Tooling / Tests").

## Architecture

### Wire protocol has three independent implementations

The frame format `[0xAA][0x55][type][len][payload...][CRC-8 (poly 0x07)]` and the message type table are defined three times, once per language runtime, and must be kept in sync **by hand**:

- `lib/SwarmProtocol/protocol.h` — canonical C++ definition, used by `src/dongle` and `src/receiver` firmware
- `lib/SwarmClient/SwarmClient.h` — PC-side header-only client; mirrors the same constants under an `SC_` prefix
- `src/robots/robot_uart.py` — MicroPython parser on the RP2040 (`UARTProtocol` class)

When changing a message type or payload layout, all three need updating; there's no shared codegen.

### `swarm_hub` is the only process that owns the serial port

PC tools never open the dongle's serial device directly. `swarm_hub` (`tools/swarm/swarm_hub.cpp`) bridges the USB-serial connection to a Unix socket at `/tmp/swarm_hub.sock`, and every other PC tool connects to that socket. Vision tools auto-launch `swarm_hub` as a daemon if a USB dongle is detected and the hub isn't already running.

For new PC tools, use `lib/SwarmClient/SwarmClient.h` rather than talking to the socket directly — it auto-connects (and will auto-launch `swarm_hub` via `fork`/`exec` if needed), builds outgoing `MSG_SWARM` frames from a `setSpeed()`/`flush()` call pair, and parses incoming `MSG_ANNOUNCE`/`MSG_TELEMETRY`/`MSG_PONG` frames into per-robot `RobotState`. Calling `poll()` regularly is required even if you don't care about telemetry — it's what reads and parses incoming frames (including the `MSG_PONG`s that populate `latencyUs`). The round-robin pinging that produces those pongs is driven centrally by `swarm_hub` (it snoops announce/telemetry/pong frames to learn live robot IDs and emits one `MSG_PING` per interval), **not** per-client: this keeps exactly one ping in flight no matter how many tools are connected, since the dongle's latency tracker (`src/dongle/main.cpp`, single `pingTracker`) only holds one outstanding ping at a time — multiple independent pingers would clobber it and corrupt RTT.

### Robot registration and addressing

Each robot's ID is baked into firmware at flash time (`ROBOT_ID` env var, defaults to 0), not configured at runtime. On boot a robot broadcasts `MSG_ANNOUNCE` every 500ms until the dongle ACKs it; `swarm_hub`/`SwarmClient` track per-ID state (MAC, battery, motor state, last-seen) keyed off that ID. A `MSG_SWARM` frame is variable-length and only carries entries for currently-known robots (`SwarmClient::flush()` skips unknown IDs), not a fixed 32-slot array.

### Safety watchdogs live on the robot, not the PC

`WATCHDOG_TIMEOUT_MS` (`lib/SwarmProtocol/hardware.h`) stops motors if a robot's RP2040 hasn't received a fresh `MSG_SPEED` recently, independent of whether the PC/dongle/ESP-NOW link is still alive. Re-announce/expiry timing (`ANNOUNCE_TIMEOUT_MS`, `ROBOT_EXPIRY_MS`, `REANNOUNCE_INTERVAL_MS`) is also centralized there — check this header before touching timing-sensitive robot behavior.

### Vision pipeline is a separate concern layered on top of the swarm link

`lib/ArucoTracker/` wraps the Basler camera (via pylon) and OpenCV ArUco detection behind `aruco_tracker.h`. Vision-based controllers (`tools/vision/vision_controller.cpp`, `wingman.cpp`, `circle_demo.cpp`, `shape_demo.cpp`) consume tracked poses and then drive robots through the same `SwarmClient` as any other PC tool — vision and swarm control aren't otherwise coupled. Per `TODO.md`, this separation is still informal (most demos build ArUco poses and protocol frames inline rather than through a clean `PoseStream`-style interface); `drag_drop_demo.cpp` is cited there as the tool that already does this more cleanly via `SwarmClient`.

### MicroPython robot firmware: feature-flag + isolated-module pattern

`src/robots/uart_controller.py` is the main loop (UART receive → PID speed control → motor output → display), and it deliberately keeps optional features out of the core file: each one is gated by a module-level `..._ENABLED` flag and implemented in its own module, with the main loop calling at most one hook per loop iteration. Example: `ENGINE_SOUND_ENABLED` guards both the import of `engine_sound.EngineSound` and the single `engine.update(actual_l, actual_r, dt)` call inside `run_pid()`. Follow this pattern for new robot-local features (sound, additional sensors, etc.) rather than growing `uart_controller.py` or `robot_uart.py` directly — and keep such features computed from data the robot already has rather than extending the shared wire protocol, since that protocol is also implemented independently in the C++ firmware and PC client (see above).

`ODOMETRY_ENABLED` in the same file is a similar toggle but for swapping the *entire* control strategy (closed-loop PID on encoder counts/s vs. open-loop target-to-power mapping) rather than adding a feature — both code paths are kept intact so it can be flipped without re-deriving the open-loop math.

### Repo layout

```
src/dongle, src/receiver   — PlatformIO firmware (C++), built/flashed independently per platformio.ini env
src/robots/                — MicroPython firmware for the RP2040, deployed without a build step
lib/SwarmProtocol/         — wire protocol shared by both firmware targets (canonical C++ definition)
lib/SwarmClient/           — header-only PC client library; new PC tools should build on this
lib/ArucoTracker/          — camera + ArUco tracking abstraction (Basler pylon + OpenCV)
tools/                     — Makefile + PC tool entry points (game.cpp, vision/*.cpp, swarm/*.cpp); binaries land in tools/build/
docs/architecture.md       — protocol/timing design doc (German)
```

Build artifacts (`.pio/`, `tools/build/`, `lib/SwarmProtocol/robot_id_cfg.h`) are gitignored and regenerated by the commands above — don't hand-edit them.
