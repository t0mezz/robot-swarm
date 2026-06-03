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
│   ├── Makefile                # Builds all PC tools (macOS arm64 + Linux x86_64)
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

### Firmware
- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)

### PC tools (macOS)
- Xcode Command Line Tools
- Homebrew: `brew install opencv pkg-config sfml@2`
- [Basler pylon 8.1.0](https://www.baslerweb.com/en/software/pylon/) — install the macOS Universal Binary package

### PC tools (Linux x86_64)
- `apt install libopencv-dev pkg-config`
- [Basler pylon](https://www.baslerweb.com/en/software/pylon/) — install to `/opt/pylon/`

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

### 4. Run

Start the hub (bridges USB serial to a Unix socket):

```bash
./swarm_hub /dev/tty.usbmodem*
```

Then launch any controller (see below).

---

## PC Tools

### `swarm_hub`
Serial ↔ Unix socket bridge. All other tools connect to it via `/tmp/swarm_hub.sock`. Launched automatically by vision tools if a USB dongle is detected.

```bash
./swarm_hub /dev/tty.usbmodem*
./swarm_hub --daemon /dev/tty.usbmodem*
```

---

### `swarm_terminal`
Terminal UI showing all registered robots with RSSI, latency, battery, and motor state.

```bash
./swarm_terminal
```

---

### `swarm_controller`
Interactive keyboard controller and test suite. Drive individual robots or run automated test sequences.

```bash
./swarm_controller
```

---

### `latency_plot`
Live ASCII latency plot for a specific robot. Shows round-trip ping time in µs.

```bash
./latency_plot <robot_id>
```

---

### `vision_controller`
Main vision-based controller. Overhead camera tracks ArUco markers; click to set movement goals. WASD drives the leader robot (macOS).

```bash
./vision_controller [--serial SN] [--ip IP] [--calibrate]
```

| Key | Action |
|-----|--------|
| Left-click | Set goal for all robots |
| Right-click | Set goal for selected robot |
| `0`–`9` | Select robot |
| `s` | Stop all, unlock leader |
| `c` | Re-run homography calibration |
| `+` / `-` | Speed ±10% |
| `WASD` | Drive leader directly (macOS) |
| `q` / Esc | Quit |

---

### `wingman`
V-formation controller. Leader is driven with WASD; all other robots autonomously hold positions in a V behind it.

```bash
./wingman [--serial SN] [--ip IP] [--spacing MM] [--dist MM] [--speed PCT]
```

| Key | Action |
|-----|--------|
| `WASD` | Drive leader |
| `+` / `-` | Formation spacing ±25 mm |
| `s` | Stop all |
| `c` | Re-calibrate |
| `q` / Esc | Quit |

---

### `circle_demo`
Robots orbit a point you click. Speed, radius, and orbit direction are adjustable live.

```bash
./circle_demo [--serial SN] [--ip IP] [--radius MM] [--min-gap MM]
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
./shape_demo [--serial SN] [--ip IP] [--speed PCT]
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
./marker_eval [--config JSON] [--serial SN] [--ip IP]
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

./calibrate             # static: markers held still
./calibrate --motion    # motion: robots driving at operating speed
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
5. Run `./marker_eval` to verify FPS and detection before using a controller.
