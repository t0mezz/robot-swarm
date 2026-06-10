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
│   └── vision/
│       ├── vision_controller.cpp  # Main vision-based swarm controller
│       ├── wingman.cpp            # V-formation follower controller
│       ├── circle_demo.cpp        # Circle orbit formation
│       ├── shape_demo.cpp         # Freehand path drawing controller
│       ├── marker_eval.cpp        # Camera + detection benchmarking tool
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
>   (`lib/swarm/evdev_keys.h`) rather than the X11 `XQueryKeymap`, because
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
