# Graph Report - robot-swarm  (2026-09-22)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1724 nodes · 3289 edges · 87 communities (80 shown, 7 thin omitted)
- Extraction: 88% EXTRACTED · 12% INFERRED · 0% AMBIGUOUS · INFERRED: 382 edges (avg confidence: 0.81)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `23254f13`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- CfRing
- swarm_telemetry_json.cpp
- receiver/main.cpp
- main
- Vehicle
- circle_demo.cpp
- test_car_following.cpp
- ArucoConfig
- shape_demo.cpp
- swarm_controller.cpp
- ArucoTracker
- Robot Swarm Pipeline — Architekturplan
- CMAES
- SwarmClient
- Screen
- uart_controller.py
- BaslerPylonSource
- drag_drop_demo.cpp
- cli.js
- RingBuffer
- DemoHud
- swarm_hub.cpp
- objective.h
- Roster.js
- string
- vision_controller.cpp
- flash.py
- BarGraph
- ================== FIXED ===================
- Architecture
- UARTProtocol
- swarm_hub_simulation.cpp
- wingman.cpp
- aruco_demo.py
- circle_demo
- swarm_dashboard.cpp
- vector
- dashboard-ink/package.json
- cstdio
- ScreenManager
- aruco_tracker.h
- calib_main.cpp
- HttpBridge
- CMA-ES ArUco Detector Calibration
- Wire Protocol: Three Independent Implementations
- debug_protocol.h
- circle_demo sleep_for(30ms) Poll Root Cause
- `swarm_hub`
- MSG_TELEMETRY Packet Format
- RobotState
- Pololu 3pi+ 2040 Swarm Control
- .detectionLoop
- PC Tools
- html
- fromFile
- generate_markers_pdf.py
- MicroPython Feature-Flag + Isolated-Module Pattern
- Safety Watchdogs Live on the Robot
- CRC Error Handling (Silent Drop)
- Latency Budget (~4ms End-to-End)
- ODOMETRY_ENABLED Control-Strategy Toggle
- drawTelHud
- generate_marker_stl.py
- Robot Swarm
- RobotPose
- param_space.h
- app.js
- IPreprocessor
- Fisheye Calibration (planned)
- Args
- ICameraSource
- IOptimizer
- algorithm
- ArUco Tracker Calibration
- EvdevKeyboard
- Quick Start
- Planned extensions
- PoseHubHeader
- main.py
- DetectionResult
- _draw_line
- car_following_bridge.js
- ClientConn
- ClientConn
- SlotOffset
- hardware.h

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 75 edges
2. `ArucoConfig` - 57 edges
3. `SwarmClient` - 47 edges
4. `CfRing` - 45 edges
5. `main()` - 44 edges
6. `Vehicle` - 37 edges
7. `CMAES` - 36 edges
8. `main()` - 31 edges
9. `DemoHud` - 30 edges
10. `main()` - 29 edges

## Surprising Connections (you probably didn't know these)
- `Recommendations for the rework` --references--> `UARTProtocol`  [INFERRED]
  ROBOT_FIRMWARE_OVERVIEW.md → src/robots/robot_uart.py
- `Recommendations for the rework` --references--> `BarGraph`  [INFERRED]
  ROBOT_FIRMWARE_OVERVIEW.md → src/robots/screen_manager.py
- `RP2040 — `src/robots/`` --references--> `BarGraph`  [INFERRED]
  ROBOT_FIRMWARE_OVERVIEW.md → src/robots/screen_manager.py
- `Recommendations for the rework` --references--> `Gauge`  [INFERRED]
  ROBOT_FIRMWARE_OVERVIEW.md → src/robots/screen_manager.py
- `RP2040 — `src/robots/`` --references--> `Gauge`  [INFERRED]
  ROBOT_FIRMWARE_OVERVIEW.md → src/robots/screen_manager.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **circle_demo Performance Root-Cause Investigation** — performance_circle_demo_sleep_poll_rootcause, performance_governor_epp_red_herring, performance_half_res_sweep, performance_loop_fps_vs_cam_fps [EXTRACTED 0.90]
- **Vision-Based Swarm Controllers** — readme_vision_controller, readme_wingman, readme_circle_demo, readme_shape_demo [EXTRACTED 0.90]
- **Wire Protocol Three-Language Implementation** — claudemd_wire_protocol_three_implementations, readme_wire_protocol_frame_format, src_robots_readme_uartprotocol, docs_architecture_crc_error_handling [INFERRED 0.85]

## Communities (87 total, 7 thin omitted)

### Community 0 - "CfRing"
Cohesion: 0.05
Nodes (74): CfPhase, CfRunEvent, cfNormAngleDeg(), CfRing, cars_, cfg_, order_, pendingCount_ (+66 more)

### Community 1 - "swarm_telemetry_json.cpp"
Cohesion: 0.07
Nodes (38): vector, HubPose, id, px, py, x, y, yaw (+30 more)

### Community 2 - "receiver/main.cpp"
Cohesion: 0.05
Nodes (52): adafruit_neopixel, debug_protocol, esp_now, esp_wifi, hardware, protocol, anyRobotActive(), enqueueSend() (+44 more)

### Community 3 - "main"
Cohesion: 0.06
Nodes (47): car_following, cctype, fstream, http_bridge, poll, ring, sstream, applyParams() (+39 more)

### Community 4 - "Vehicle"
Cohesion: 0.07
Nodes (46): Color, Font, graphics, optional, RenderWindow, alpha(), pair, string (+38 more)

### Community 5 - "circle_demo.cpp"
Cohesion: 0.07
Nodes (40): assignNearestSlots(), CalibState, done, pixPts, CircleState, centre, centreSet, minGapMm (+32 more)

### Community 6 - "test_car_following.cpp"
Cohesion: 0.12
Nodes (41): cfAcceleration(), cfBufferedParams(), cfBufferedTimeGap(), CfInput, gap, predGap, predSpeed, speed (+33 more)

### Community 7 - "ArucoConfig"
Cohesion: 0.05
Nodes (41): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+33 more)

### Community 8 - "shape_demo.cpp"
Cohesion: 0.09
Nodes (38): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+30 more)

### Community 9 - "swarm_controller.cpp"
Cohesion: 0.09
Nodes (33): coregraphics, termios, advanceTest(), applyTestStep(), KeyHandle, time_point, vector, drawUI() (+25 more)

### Community 10 - "ArucoTracker"
Cohesion: 0.06
Nodes (28): ArucoTracker, captureRunning_, captureThread_, cfg_, clahe_, debug_, detectionRunning_, detectionThread_ (+20 more)

### Community 11 - "Robot Swarm Pipeline — Architekturplan"
Cohesion: 0.06
Nodes (34): 1. Roboter-Registrierung, 2. Steuerkanal (PC → Roboter), 3. Telemetrie-Rückkanal (Roboter → PC), 4. Protokoll-Übersicht, 5. Fehlerbehandlung, 6. Controller-PC Software, 7. Hardware-Checkliste pro Roboter, 8. Implementierungsreihenfolge (+26 more)

### Community 12 - "CMAES"
Cohesion: 0.07
Nodes (25): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+17 more)

### Community 13 - "SwarmClient"
Cohesion: 0.10
Nodes (13): mutex, string, vector, SwarmClient, m_debugLog, m_fd, m_robots, m_rxBuf (+5 more)

### Community 14 - "Screen"
Cohesion: 0.07
Nodes (15): Repräsentiert einen einzelnen Debug-Screen. Verwendung: screen = Screen("MY…, Fügt eine neue Nachricht zum Log hinzu., Löscht alle Log-Einträge., Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück., Entfernt eine einzelne Primitive per Handle. (+7 more)

### Community 15 - "uart_controller.py"
Cohesion: 0.08
Nodes (24): framebuf, machine, math, pololu_3pi_2040_robot_battery, build_packet(), _crc8(), CRC-8 via Lookup-Table (Polynom 0x07)., Verpackt Nutzdaten in einen vollstaendigen Frame. Frame:… (+16 more)

### Community 16 - "BaslerPylonSource"
Cohesion: 0.09
Nodes (24): CBaslerUniversalInstantCamera, CImageFormatConverter, cstdlib, BaslerPylonSource, camera_, converter_, height_, pylonRuntime_ (+16 more)

### Community 17 - "drag_drop_demo.cpp"
Cohesion: 0.10
Nodes (25): Point, Scalar, AvoidState, arc, arcDx, arcDy, minDist, buildAvoidance() (+17 more)

### Community 18 - "cli.js"
Cohesion: 0.09
Nodes (14): ref_node_child_process, ref_node_events, ref_node_fs, ref_node_path, ref_node_url, app, argv, DEFAULT_PRODUCER (+6 more)

### Community 19 - "RingBuffer"
Cohesion: 0.09
Nodes (17): array, cstddef, unordered_map, N, swarmclient, Buffer, unordered_map, vector (+9 more)

### Community 20 - "DemoHud"
Cohesion: 0.11
Nodes (21): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+13 more)

### Community 21 - "swarm_hub.cpp"
Cohesion: 0.17
Nodes (23): errno, ioss, stat, any_client_active(), broadcast_to_clients(), build_ping(), client_accept(), client_close() (+15 more)

### Community 22 - "objective.h"
Cohesion: 0.10
Nodes (21): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+13 more)

### Community 23 - "Roster.js"
Cohesion: 0.18
Nodes (22): ref_node_assert, ref_node_test, LatencyGraph(), fmtLost(), Header(), Roster(), rosterLayout(), Row() (+14 more)

### Community 24 - "string"
Cohesion: 0.13
Nodes (19): cerrno, cstdint, cstring, dirent, dyld, fcntl, glob, in (+11 more)

### Community 25 - "vision_controller.cpp"
Cohesion: 0.11
Nodes (21): evdev_keys, CalibState, done, pixPts, clampf(), KeyHandle, Point2f, vector (+13 more)

### Community 26 - "flash.py"
Cohesion: 0.15
Nodes (23): hashlib, pathlib, serial_tools_list_ports, shutil, subprocess, connected_robot_ports(), deploy_cmd(), deploy_problems() (+15 more)

### Community 27 - "BarGraph"
Cohesion: 0.10
Nodes (15): Detailed breakdown, ESP32-S3 Receiver — `src/receiver/main.cpp`, Graph, Legend, Physical Robot Firmware — Used vs. Unused Feature Map, Recommendations for the rework, RP2040 — `src/robots/`, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG). Der ESP32 stellt die… (+7 more)

### Community 28 - "================== FIXED ==================="
Cohesion: 0.12
Nodes (16): Bug: Latency Erledigt (2026-06-18), Erledigt (2026-06-10), Erledigt (2026-06-12), Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06), ================== FIXED ===================, Genera aruco tracker issue: Erledigt (2026-07-08), Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12), questions to answer: (+8 more)

### Community 29 - "Architecture"
Cohesion: 0.12
Nodes (14): Architecture, Commands, Firmware (PlatformIO — `src/dongle`, `src/receiver`), MicroPython robot firmware: feature-flag + isolated-module pattern, PC tools (`tools/`, plain Makefile), Project Overview, Repo layout, Robot firmware (`src/robots/`, MicroPython) (+6 more)

### Community 30 - "UARTProtocol"
Cohesion: 0.11
Nodes (12): _crc8_buf(), Packet, Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED). :param left: Geschwindigkeit…, Sendet die Batteriespannung an den ESP32 (MSG_METRICS). :param battery_byte:…, Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig. (+4 more)

### Community 31 - "swarm_hub_simulation.cpp"
Cohesion: 0.18
Nodes (22): broadcast(), buildFrame(), client_accept(), client_close(), crc8(), findRobot(), frameSize(), hub_server_create() (+14 more)

### Community 32 - "wingman.cpp"
Cohesion: 0.14
Nodes (20): CGEventRef, CGEventTapProxy, CGEventType, applyLeaderMotors(), CalibState, done, pts, clampf() (+12 more)

### Community 33 - "aruco_demo.py"
Cohesion: 0.10
Nodes (16): concurrent_futures, cv2, numpy, os, re, threading, CaptureThread, _detect_with_roi() (+8 more)

### Community 34 - "circle_demo"
Cohesion: 0.13
Nodes (14): 2026-06-12 — back to `powersave` (after the `performance` test), 2026-06-12 — Baseline (revised: split by tracking state), 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config), 2026-06-12 — governor `powersave` → `performance`, 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change, 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default), 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop, circle_demo (+6 more)

### Community 35 - "swarm_dashboard.cpp"
Cohesion: 0.17
Nodes (21): appendf(), bipolarMeter(), brailleGraph(), computeLayout(), Buffer, pair, string, time_point (+13 more)

### Community 36 - "vector"
Cohesion: 0.11
Nodes (18): KalmanFilter, Point2f, time_point, vector, MarkerState, bboxSize, center, failCount (+10 more)

### Community 37 - "dashboard-ink/package.json"
Cohesion: 0.10
Nodes (20): htm, ink, react, bin, swarm-dashboard-ink, dependencies, htm, ink (+12 more)

### Community 38 - "cstdio"
Cohesion: 0.14
Nodes (12): chrono, csignal, cstdarg, cstdio, deque, iostream, drawPlot(), main() (+4 more)

### Community 39 - "ScreenManager"
Cohesion: 0.12
Nodes (11): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display. Knopfdruck…, :param display: robot.Display() Instanz :param button: Button-Instanz mit…, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+3 more)

### Community 40 - "aruco_tracker.h"
Cohesion: 0.12
Nodes (16): aruco, atomic, basleruniversalinstantcamera, condition_variable, function, functional, future, ThreadPool (+8 more)

### Community 41 - "calib_main.cpp"
Cohesion: 0.16
Nodes (17): cmaes, filesystem, highgui, imgproc, bestX_, memory, objective, objective_static (+9 more)

### Community 42 - "HttpBridge"
Cohesion: 0.19
Nodes (11): Conn, answered, fd, rx, tx, string, vector, HttpBridge (+3 more)

### Community 43 - "CMA-ES ArUco Detector Calibration"
Cohesion: 0.32
Nodes (8): Vision Pipeline as Separate Concern, `marker_eval`, CMA-ES ArUco Detector Calibration, GP-ARD Bayesian Optimizer (planned), IOptimizer ask/tell Interface, Motion Objective Scoring, Scoring, Static objective

### Community 44 - "Wire Protocol: Three Independent Implementations"
Cohesion: 0.29
Nodes (7): Wire Protocol: Three Independent Implementations, Wire Protocol Frame Format, flash.py Batch Flashing, ScreenManager Display Manager, uart_controller.py Main Loop, UARTProtocol Library (robot_uart.py), Remote Robot Shutdown (scrapped)

### Community 45 - "debug_protocol.h"
Cohesion: 0.22
Nodes (16): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+8 more)

### Community 46 - "circle_demo sleep_for(30ms) Poll Root Cause"
Cohesion: 0.33
Nodes (6): circle_demo sleep_for(30ms) Poll Root Cause, Governor/EPP/Throttling Red Herring, half_res_sweep Config Optimization, loop_fps vs cam_fps Gap, `circle_demo`, Generalized DEBUG/HUD Util (planned)

### Community 47 - "`swarm_hub`"
Cohesion: 0.40
Nodes (5): Centralized Round-Robin Pinging in swarm_hub, swarm_hub Owns the Serial Port, Pylon rpath Link Fix, `swarm_hub`, Per-Client Ping Clobbering Bug (fixed)

### Community 48 - "MSG_TELEMETRY Packet Format"
Cohesion: 0.40
Nodes (5): Robot Registration and Addressing, MSG_TELEMETRY Packet Format, Robot Registration via Announce, TDMA Telemetry Return Channel, MSG_METRICS Battery Channel (0x03)

### Community 49 - "RobotState"
Cohesion: 0.11
Nodes (18): DebugEntry, at, fieldId, robotId, text, time_point, RobotState, battery (+10 more)

### Community 50 - "Pololu 3pi+ 2040 Swarm Control"
Cohesion: 0.18
Nodes (10): Batch flashing, Data Pipeline, Debug Display Types, Dependencies, Display, Files, Message Types, Packet Format (+2 more)

### Community 51 - ".detectionLoop"
Cohesion: 0.14
Nodes (7): F, FisheyeUndistortPreprocessor, map1_, map2_, Mat, Size, submit()

### Community 52 - "PC Tools"
Cohesion: 0.20
Nodes (11): `calibrate`, evdev WASD Keyboard Input, `frame_inspector`, `latency_plot`, PC Tools, `shape_demo`, `swarm_controller`, `swarm_terminal` (+3 more)

### Community 53 - "html"
Cohesion: 0.28
Nodes (12): Arena(), BoxRow(), Field(), flagNames(), Focus(), FocusStrip(), KeyHints(), Log() (+4 more)

### Community 54 - "fromFile"
Cohesion: 0.15
Nodes (11): aruco_tracker, demohud, defaultConfigPath, fromFile, time_point, LoopFps, count_, last_ (+3 more)

### Community 55 - "generate_markers_pdf.py"
Cohesion: 0.20
Nodes (13): io, reportlab_lib_pagesizes, reportlab_lib_units, reportlab_lib_utils, reportlab_pdfgen, generate_pdf(), main(), _marker_image() (+5 more)

### Community 56 - "MicroPython Feature-Flag + Isolated-Module Pattern"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

### Community 61 - "drawTelHud"
Cohesion: 0.35
Nodes (8): string, Mode, main(), Mat, drawTelHud(), Mat, drawPanel(), drawTelHud()

### Community 62 - "generate_marker_stl.py"
Cohesion: 0.22
Nodes (12): argparse, struct, box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Generate a thin STL model of an ArUco 4x4 marker — black cells only. Usage:… (+4 more)

### Community 63 - "Robot Swarm"
Cohesion: 0.20
Nodes (9): Architecture, Camera Setup (Basler ace2 GigE), Firmware (all platforms), PC Tools — macOS (Apple Silicon), PC Tools — Ubuntu 22.04 / 24.04 (x86\_64), Prerequisites, Project Structure, Robot Swarm (+1 more)

### Community 64 - "RobotPose"
Cohesion: 0.24
Nodes (13): RobotPose, id, px, py, x, y, yaw, assignByLateral() (+5 more)

### Community 65 - "param_space.h"
Cohesion: 0.22
Nodes (12): decode(), encode(), fromNorm(), string, vector, ParamSpec, hi, key (+4 more)

### Community 66 - "app.js"
Cohesion: 0.28
Nodes (11): App(), clamp(), computeLayout(), snapshot(), sortRobots(), SORTS, useTerminalSize(), chips() (+3 more)

### Community 67 - "IPreprocessor"
Cohesion: 0.20
Nodes (7): CLAHE, CLAHEPreprocessor, clahe_, IPreprocessor, process, Ptr, unique_ptr

### Community 69 - "Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 70 - "ICameraSource"
Cohesion: 0.20
Nodes (4): ICameraSource, open, read, size

### Community 71 - "IOptimizer"
Cohesion: 0.20
Nodes (8): IOptimizer, ask, bestFit, converged, generation, lambda, setMean, tell

### Community 72 - "algorithm"
Cohesion: 0.25
Nodes (7): algorithm, cmath, core, limits, map, numeric, random

### Community 73 - "ArUco Tracker Calibration"
Cohesion: 0.29
Nodes (6): All flags, ArUco Tracker Calibration, File structure, How it works, Parameters being optimised, Quick start

### Community 74 - "EvdevKeyboard"
Cohesion: 0.25
Nodes (5): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector

### Community 75 - "Quick Start"
Cohesion: 0.33
Nodes (6): 1. Flash the dongle, 2. Flash each robot, 3. Build PC tools, 4. Find the dongle serial port, 5. Run the hub, Quick Start

### Community 76 - "Planned extensions"
Cohesion: 0.33
Nodes (6): Combined static + motion objective, Fisheye calibration (`objective_fisheye.h`), GP with ARD kernel (`gp_ard.h`), Motion objective (`objective_motion.h`), Parameter range narrowing, Planned extensions

### Community 77 - "PoseHubHeader"
Cohesion: 0.25
Nodes (8): PoseHubHeader, count, detFps, frameH, frameW, magic, seq, version

### Community 78 - "main.py"
Cohesion: 0.29
Nodes (6): pololu_3pi_2040_robot, pololu_3pi_2040_robot_display, pololu_3pi_2040_robot_extras_splash_loader, pololu_3pi_2040_robot_motors, pololu_3pi_2040_robot_rgb_leds, sys

### Community 79 - "DetectionResult"
Cohesion: 0.33
Nodes (6): DetectionResult, debug, fps, fresh, latencyMs, robots

### Community 80 - "_draw_line"
Cohesion: 0.33
Nodes (3): _draw_line(), Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-…, Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest.

### Community 81 - "car_following_bridge.js"
Cohesion: 0.53
Nodes (4): buildPanel(), field(), store(), stored()

### Community 82 - "ClientConn"
Cohesion: 0.40
Nodes (5): ClientConn, active, fd, rxBuf, rxLen

### Community 83 - "ClientConn"
Cohesion: 0.40
Nodes (5): ClientConn, active, fd, rxBuf, rxLen

### Community 84 - "SlotOffset"
Cohesion: 0.67
Nodes (3): SlotOffset, dx, dy

## Knowledge Gaps
- **538 isolated node(s):** `open`, `id`, `x`, `y`, `yaw` (+533 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 817 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **7 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoTracker` connect `ArucoTracker` to `RobotPose`, `swarm_telemetry_json.cpp`, `wingman.cpp`, `IPreprocessor`, `vector`, `circle_demo.cpp`, `ICameraSource`, `ArucoConfig`, `aruco_tracker.h`, `shape_demo.cpp`, `DetectionResult`, `drag_drop_demo.cpp`, `.detectionLoop`, `RingBuffer`, `string`, `vision_controller.cpp`?**
  _High betweenness centrality (0.076) - this node is a cross-community bridge._
- **Why does `ArucoConfig` connect `ArucoConfig` to `param_space.h`, `aruco_tracker.h`, `calib_main.cpp`, `ArucoTracker`, `CMAES`, `BaslerPylonSource`, `fromFile`, `objective.h`, `string`?**
  _High betweenness centrality (0.041) - this node is a cross-community bridge._
- **Why does `main()` connect `main` to `CfRing`, `swarm_telemetry_json.cpp`, `test_car_following.cpp`, `HttpBridge`, `SwarmClient`, `DemoHud`, `fromFile`, `drawTelHud`?**
  _High betweenness centrality (0.039) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 12 inferred relationships involving `CfRing` (e.g. with `test_a_dropout_keeps_its_place_on_the_ring()` and `test_a_lagging_robot_still_gets_the_ring_moving()`) actually correct?**
  _`CfRing` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 28 inferred relationships involving `main()` (e.g. with `cfModelFromName()` and `cfModelHasDesiredGap()`) actually correct?**
  _`main()` has 28 INFERRED edges - model-reasoned connections that need verification._
- **What connects `open`, `id`, `x` to the rest of the system?**
  _538 weakly-connected nodes found - possible documentation gaps or missing edges._