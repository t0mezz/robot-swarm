# Graph Report - robot-swarm  (2026-09-24)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1852 nodes · 3598 edges · 103 communities (96 shown, 7 thin omitted)
- Extraction: 88% EXTRACTED · 12% INFERRED · 0% AMBIGUOUS · INFERRED: 429 edges (avg confidence: 0.81)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `7033aad4`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- CfRing
- swarm_telemetry_json.cpp
- receiver/main.cpp
- car_following.cpp
- Vehicle
- circle_speed_test.cpp
- test_car_following.cpp
- circle_demo.cpp
- ArucoConfig
- vision_controller.cpp
- shape_demo.cpp
- Robot Swarm Pipeline — Architekturplan
- ArucoTracker
- CMAES
- SwarmClient
- uart_controller.py
- max_speed_test.cpp
- Screen
- cli.js
- swarmclient
- BaslerPylonSource
- swarm_hub.cpp
- objective.h
- Roster.js
- cstring
- wingman.cpp
- flash.py
- DemoHud
- ================== FIXED ===================
- Architecture
- swarm_controller.cpp
- drag_drop_demo.cpp
- swarm_hub_simulation.cpp
- aruco_demo.py
- circle_demo
- swarm_dashboard.cpp
- vector
- HttpBridge
- cstdio
- screen_manager.py
- string
- calib_main.cpp
- dashboard-ink/package.json
- CMA-ES ArUco Detector Calibration
- Wire Protocol: Three Independent Implementations
- UARTProtocol
- circle_demo sleep_for(30ms) Poll Root Cause
- `swarm_hub`
- MSG_TELEMETRY Packet Format
- ScreenManager
- Pololu 3pi+ 2040 Swarm Control
- main
- PC Tools
- debug_protocol.h
- main
- generate_markers_pdf.py
- MicroPython Feature-Flag + Isolated-Module Pattern
- Safety Watchdogs Live on the Robot
- CRC Error Handling (Silent Drop)
- Latency Budget (~4ms End-to-End)
- ODOMETRY_ENABLED Control-Strategy Toggle
- .detectionLoop
- generate_marker_stl.py
- Robot Swarm
- html
- measurement_test.cpp
- car_following_bridge.js
- thread
- Fisheye Calibration (planned)
- RobotPose
- param_space.h
- app.js
- algorithm
- ArUco Tracker Calibration
- RobotState
- Quick Start
- Planned extensions
- CaptureThread
- main.py
- IPreprocessor
- RingBuffer
- msc-flash.py
- Args
- Ring
- ICameraSource
- hardware.h
- .drawText
- IOptimizer
- Buffering
- Servo
- EvdevKeyboard
- PoseHubHeader
- applyParams
- TestState
- RateEstimator
- DetectionResult
- DebugEntry
- PI
- Snapshot
- ClientConn
- ClientConn
- RobotRowLayout

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 83 edges
2. `ArucoConfig` - 57 edges
3. `CfRing` - 55 edges
4. `main()` - 49 edges
5. `SwarmClient` - 47 edges
6. `main()` - 38 edges
7. `Vehicle` - 37 edges
8. `CMAES` - 36 edges
9. `DemoHud` - 30 edges
10. `main()` - 29 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `buildArucoDetector()`  [INFERRED]
  tools/vision/frame_inspector.cpp → lib/ArucoTracker/aruco_tracker.h
- `main()` --calls--> `bestX_`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/cmaes.h
- `main()` --calls--> `fromNorm()`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/param_space.h
- `main()` --calls--> `encode()`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/param_space.h
- `runOptimisation()` --calls--> `encode()`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/param_space.h

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **circle_demo Performance Root-Cause Investigation** — performance_circle_demo_sleep_poll_rootcause, performance_governor_epp_red_herring, performance_half_res_sweep, performance_loop_fps_vs_cam_fps [EXTRACTED 0.90]
- **Vision-Based Swarm Controllers** — readme_vision_controller, readme_wingman, readme_circle_demo, readme_shape_demo [EXTRACTED 0.90]
- **Wire Protocol Three-Language Implementation** — claudemd_wire_protocol_three_implementations, readme_wire_protocol_frame_format, src_robots_readme_uartprotocol, docs_architecture_crc_error_handling [INFERRED 0.85]

## Communities (103 total, 7 thin omitted)

### Community 0 - "CfRing"
Cohesion: 0.05
Nodes (85): CfPhase, CfRunEvent, cfNormAngleDeg(), CfRing, cars_, cfg_, jamSpacingDeg_, leaderId_ (+77 more)

### Community 1 - "swarm_telemetry_json.cpp"
Cohesion: 0.07
Nodes (38): vector, HubPose, id, px, py, x, y, yaw (+30 more)

### Community 2 - "receiver/main.cpp"
Cohesion: 0.05
Nodes (52): adafruit_neopixel, debug_protocol, esp_now, esp_wifi, hardware, protocol, anyRobotActive(), enqueueSend() (+44 more)

### Community 3 - "car_following.cpp"
Cohesion: 0.16
Nodes (12): car_following, cctype, http_bridge, poll, ring, sstream, clampf(), string (+4 more)

### Community 4 - "Vehicle"
Cohesion: 0.07
Nodes (47): Color, Font, graphics, optional, RenderWindow, alpha(), deque, pair (+39 more)

### Community 5 - "circle_speed_test.cpp"
Cohesion: 0.05
Nodes (45): ctime, fstream, map, CalibState, done, pixPts, CircleState, centre (+37 more)

### Community 6 - "test_car_following.cpp"
Cohesion: 0.12
Nodes (41): cfAcceleration(), cfBufferedParams(), cfBufferedTimeGap(), CfInput, gap, predGap, predSpeed, speed (+33 more)

### Community 7 - "circle_demo.cpp"
Cohesion: 0.06
Nodes (39): assignNearestSlots(), CalibState, done, pixPts, CircleState, centre, centreSet, minGapMm (+31 more)

### Community 8 - "ArucoConfig"
Cohesion: 0.05
Nodes (41): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+33 more)

### Community 9 - "vision_controller.cpp"
Cohesion: 0.09
Nodes (22): atomic, coregraphics, evdev_keys, CalibState, done, pixPts, clampf(), KeyHandle (+14 more)

### Community 10 - "shape_demo.cpp"
Cohesion: 0.09
Nodes (38): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+30 more)

### Community 11 - "Robot Swarm Pipeline — Architekturplan"
Cohesion: 0.06
Nodes (34): 1. Roboter-Registrierung, 2. Steuerkanal (PC → Roboter), 3. Telemetrie-Rückkanal (Roboter → PC), 4. Protokoll-Übersicht, 5. Fehlerbehandlung, 6. Controller-PC Software, 7. Hardware-Checkliste pro Roboter, 8. Implementierungsreihenfolge (+26 more)

### Community 12 - "ArucoTracker"
Cohesion: 0.06
Nodes (32): ArucoTracker, autoCountPeak_, autoCountStart_, autoCountStarted_, captureRunning_, captureThread_, cfg_, clahe_ (+24 more)

### Community 13 - "CMAES"
Cohesion: 0.07
Nodes (25): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+17 more)

### Community 14 - "SwarmClient"
Cohesion: 0.10
Nodes (13): mutex, string, vector, SwarmClient, m_debugLog, m_fd, m_robots, m_rxBuf (+5 more)

### Community 15 - "uart_controller.py"
Cohesion: 0.08
Nodes (22): framebuf, machine, pololu_3pi_2040_robot_battery, build_packet(), _crc8(), _crc8_buf(), Packet, CRC-8 via Lookup-Table (Polynom 0x07). (+14 more)

### Community 16 - "max_speed_test.cpp"
Cohesion: 0.09
Nodes (19): cstdlib, CalibState, done, pixPts, clampf(), Point2f, vector, RunResult (+11 more)

### Community 17 - "Screen"
Cohesion: 0.07
Nodes (17): _draw_circle(), Repräsentiert einen einzelnen Debug-Screen. Verwendung: screen = Screen("MY…, Fügt eine neue Nachricht zum Log hinzu., Löscht alle Log-Einträge., Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück. (+9 more)

### Community 18 - "cli.js"
Cohesion: 0.09
Nodes (14): ref_node_child_process, ref_node_events, ref_node_fs, ref_node_path, ref_node_url, app, argv, DEFAULT_PRODUCER (+6 more)

### Community 19 - "swarmclient"
Cohesion: 0.13
Nodes (12): array, cstddef, unordered_map, swarmclient, Buffer, unordered_map, vector, PerRobot (+4 more)

### Community 20 - "BaslerPylonSource"
Cohesion: 0.09
Nodes (23): CBaslerUniversalInstantCamera, CImageFormatConverter, BaslerPylonSource, camera_, converter_, height_, pylonRuntime_, width_ (+15 more)

### Community 21 - "swarm_hub.cpp"
Cohesion: 0.16
Nodes (24): errno, ioss, stat, termios, any_client_active(), broadcast_to_clients(), build_ping(), client_accept() (+16 more)

### Community 22 - "objective.h"
Cohesion: 0.10
Nodes (21): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+13 more)

### Community 23 - "Roster.js"
Cohesion: 0.18
Nodes (22): ref_node_assert, ref_node_test, LatencyGraph(), fmtLost(), Header(), Roster(), rosterLayout(), Row() (+14 more)

### Community 24 - "cstring"
Cohesion: 0.16
Nodes (17): cerrno, cstdint, cstring, dirent, dyld, fcntl, glob, in (+9 more)

### Community 25 - "wingman.cpp"
Cohesion: 0.12
Nodes (23): CGEventRef, CGEventTapProxy, CGEventType, applyLeaderMotors(), CalibState, done, pts, clampf() (+15 more)

### Community 26 - "flash.py"
Cohesion: 0.26
Nodes (12): hashlib, pathlib, serial_tools_list_ports, connected_robot_ports(), deploy_cmd(), deploy_problems(), eject_micropython_volume(), flash() (+4 more)

### Community 27 - "DemoHud"
Cohesion: 0.11
Nodes (21): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+13 more)

### Community 28 - "================== FIXED ==================="
Cohesion: 0.12
Nodes (16): Bug: Latency Erledigt (2026-06-18), Erledigt (2026-06-10), Erledigt (2026-06-12), Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06), ================== FIXED ===================, Genera aruco tracker issue: Erledigt (2026-07-08), Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12), questions to answer: (+8 more)

### Community 29 - "Architecture"
Cohesion: 0.12
Nodes (14): Architecture, Commands, Firmware (PlatformIO — `src/dongle`, `src/receiver`), MicroPython robot firmware: feature-flag + isolated-module pattern, PC tools (`tools/`, plain Makefile), Project Overview, Repo layout, Robot firmware (`src/robots/`, MicroPython) (+6 more)

### Community 30 - "swarm_controller.cpp"
Cohesion: 0.14
Nodes (24): advanceTest(), applyTestStep(), KeyHandle, vector, drawUI(), handleInput(), keyDown(), main() (+16 more)

### Community 31 - "drag_drop_demo.cpp"
Cohesion: 0.11
Nodes (23): AvoidState, arc, arcDx, arcDy, minDist, buildAvoidance(), CalibState, done (+15 more)

### Community 32 - "swarm_hub_simulation.cpp"
Cohesion: 0.18
Nodes (22): broadcast(), buildFrame(), client_accept(), client_close(), crc8(), findRobot(), frameSize(), hub_server_create() (+14 more)

### Community 33 - "aruco_demo.py"
Cohesion: 0.17
Nodes (11): concurrent_futures, cv2, numpy, os, re, subprocess, threading, time (+3 more)

### Community 34 - "circle_demo"
Cohesion: 0.13
Nodes (14): 2026-06-12 — back to `powersave` (after the `performance` test), 2026-06-12 — Baseline (revised: split by tracking state), 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config), 2026-06-12 — governor `powersave` → `performance`, 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change, 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default), 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop, circle_demo (+6 more)

### Community 35 - "swarm_dashboard.cpp"
Cohesion: 0.29
Nodes (13): appendf(), bipolarMeter(), brailleGraph(), computeLayout(), Buffer, string, drawUI(), hline() (+5 more)

### Community 36 - "vector"
Cohesion: 0.11
Nodes (18): KalmanFilter, Point2f, time_point, vector, MarkerState, bboxSize, center, failCount (+10 more)

### Community 37 - "HttpBridge"
Cohesion: 0.16
Nodes (12): Conn, answered, fd, rx, tx, string, vector, HttpBridge (+4 more)

### Community 38 - "cstdio"
Cohesion: 0.24
Nodes (7): basleruniversalinstantcamera, chrono, cstdarg, cstdio, demohud, opencv, pylonincludes

### Community 39 - "screen_manager.py"
Cohesion: 0.10
Nodes (11): math, BarGraph, _draw_line(), Gauge, LineGraph, Balkendiagramm für einen einzelnen Wert. Verwendung: bar =…, Halbkreis-Gauge für einen einzelnen Wert. Verwendung: gauge =…, Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-… (+3 more)

### Community 40 - "string"
Cohesion: 0.13
Nodes (15): aruco, condition_variable, function, functional, future, defaultConfigPath, arucoVisionDataPath(), mutex (+7 more)

### Community 41 - "calib_main.cpp"
Cohesion: 0.16
Nodes (17): cmaes, filesystem, highgui, imgproc, bestX_, memory, objective, objective_static (+9 more)

### Community 42 - "dashboard-ink/package.json"
Cohesion: 0.10
Nodes (20): htm, ink, react, bin, swarm-dashboard-ink, dependencies, htm, ink (+12 more)

### Community 43 - "CMA-ES ArUco Detector Calibration"
Cohesion: 0.32
Nodes (8): Vision Pipeline as Separate Concern, `marker_eval`, CMA-ES ArUco Detector Calibration, GP-ARD Bayesian Optimizer (planned), IOptimizer ask/tell Interface, Motion Objective Scoring, Scoring, Static objective

### Community 44 - "Wire Protocol: Three Independent Implementations"
Cohesion: 0.29
Nodes (7): Wire Protocol: Three Independent Implementations, Wire Protocol Frame Format, flash.py Batch Flashing, ScreenManager Display Manager, uart_controller.py Main Loop, UARTProtocol Library (robot_uart.py), Remote Robot Shutdown (scrapped)

### Community 45 - "UARTProtocol"
Cohesion: 0.15
Nodes (9): Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED). :param left: Geschwindigkeit…, Sendet die Batteriespannung an den ESP32 (MSG_METRICS). :param battery_byte:…, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG). Der ESP32 stellt die…, Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., Verwaltet das Senden und Empfangen von strukturierten UART-Paketen. MSG_SPEED…, UARTProtocol (+1 more)

### Community 46 - "circle_demo sleep_for(30ms) Poll Root Cause"
Cohesion: 0.33
Nodes (6): circle_demo sleep_for(30ms) Poll Root Cause, Governor/EPP/Throttling Red Herring, half_res_sweep Config Optimization, loop_fps vs cam_fps Gap, `circle_demo`, Generalized DEBUG/HUD Util (planned)

### Community 47 - "`swarm_hub`"
Cohesion: 0.40
Nodes (5): Centralized Round-Robin Pinging in swarm_hub, swarm_hub Owns the Serial Port, Pylon rpath Link Fix, `swarm_hub`, Per-Client Ping Clobbering Bug (fixed)

### Community 48 - "MSG_TELEMETRY Packet Format"
Cohesion: 0.40
Nodes (5): Robot Registration and Addressing, MSG_TELEMETRY Packet Format, Robot Registration via Announce, TDMA Telemetry Return Channel, MSG_METRICS Battery Channel (0x03)

### Community 49 - "ScreenManager"
Cohesion: 0.12
Nodes (11): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display. Knopfdruck…, :param display: robot.Display() Instanz :param button: Button-Instanz mit…, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+3 more)

### Community 50 - "Pololu 3pi+ 2040 Swarm Control"
Cohesion: 0.18
Nodes (10): Batch flashing, Data Pipeline, Debug Display Types, Dependencies, Display, Files, Message Types, Packet Format (+2 more)

### Community 51 - "main"
Cohesion: 0.37
Nodes (11): fromFile, string, Mode, main(), main(), main(), Mat, drawTelHud() (+3 more)

### Community 52 - "PC Tools"
Cohesion: 0.20
Nodes (11): `calibrate`, evdev WASD Keyboard Input, `frame_inspector`, `latency_plot`, PC Tools, `shape_demo`, `swarm_controller`, `swarm_terminal` (+3 more)

### Community 53 - "debug_protocol.h"
Cohesion: 0.22
Nodes (16): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+8 more)

### Community 54 - "main"
Cohesion: 0.16
Nodes (11): aruco_tracker, time_point, LoopFps, count_, last_, main(), Mat, Scalar (+3 more)

### Community 55 - "generate_markers_pdf.py"
Cohesion: 0.18
Nodes (14): io, reportlab_lib_pagesizes, reportlab_lib_units, reportlab_lib_utils, reportlab_pdfgen, sys, generate_pdf(), main() (+6 more)

### Community 56 - "MicroPython Feature-Flag + Isolated-Module Pattern"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

### Community 61 - ".detectionLoop"
Cohesion: 0.14
Nodes (7): F, FisheyeUndistortPreprocessor, map1_, map2_, Mat, Size, submit()

### Community 62 - "generate_marker_stl.py"
Cohesion: 0.24
Nodes (11): argparse, box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Generate a thin STL model of an ArUco 4x4 marker — black cells only. Usage:…, Return a 6×6 list-of-lists (1 = black, 0 = white) via OpenCV. (+3 more)

### Community 63 - "Robot Swarm"
Cohesion: 0.20
Nodes (9): Architecture, Camera Setup (Basler ace2 GigE), Firmware (all platforms), PC Tools — macOS (Apple Silicon), PC Tools — Ubuntu 22.04 / 24.04 (x86\_64), Prerequisites, Project Structure, Robot Swarm (+1 more)

### Community 64 - "html"
Cohesion: 0.28
Nodes (12): Arena(), BoxRow(), Field(), flagNames(), Focus(), FocusStrip(), KeyHints(), Log() (+4 more)

### Community 65 - "measurement_test.cpp"
Cohesion: 0.16
Nodes (10): csignal, CalibState, done, pixPts, Point2f, vector, pixelToWorld(), Segment (+2 more)

### Community 66 - "car_following_bridge.js"
Cohesion: 0.23
Nodes (9): anchorTrajPanel(), buildPanel(), choice(), drawTrajectories(), field(), heading(), simulationPlot(), store() (+1 more)

### Community 67 - "thread"
Cohesion: 0.22
Nodes (9): deque, iostream, thread, drawPlot(), main(), onPong(), drawUI(), main() (+1 more)

### Community 69 - "RobotPose"
Cohesion: 0.24
Nodes (13): RobotPose, id, px, py, x, y, yaw, assignByLateral() (+5 more)

### Community 70 - "param_space.h"
Cohesion: 0.22
Nodes (12): decode(), encode(), fromNorm(), string, vector, ParamSpec, hi, key (+4 more)

### Community 71 - "app.js"
Cohesion: 0.28
Nodes (11): App(), clamp(), computeLayout(), snapshot(), sortRobots(), SORTS, useTerminalSize(), chips() (+3 more)

### Community 72 - "algorithm"
Cohesion: 0.29
Nodes (6): algorithm, cmath, core, limits, numeric, random

### Community 73 - "ArUco Tracker Calibration"
Cohesion: 0.29
Nodes (6): All flags, ArUco Tracker Calibration, File structure, How it works, Parameters being optimised, Quick start

### Community 74 - "RobotState"
Cohesion: 0.17
Nodes (12): RobotState, battery, flags, hasTelemetry, known, lastPongAt, lastSeen, latencyUs (+4 more)

### Community 75 - "Quick Start"
Cohesion: 0.33
Nodes (6): 1. Flash the dongle, 2. Flash each robot, 3. Build PC tools, 4. Find the dongle serial port, 5. Run the hub, Quick Start

### Community 76 - "Planned extensions"
Cohesion: 0.33
Nodes (6): Combined static + motion objective, Fisheye calibration (`objective_fisheye.h`), GP with ARD kernel (`gp_ard.h`), Motion objective (`objective_motion.h`), Parameter range narrowing, Planned extensions

### Community 77 - "CaptureThread"
Cohesion: 0.17
Nodes (7): CaptureThread, _detect_with_roi(), ArucoDetector, ndarray, Continuously grabs frames via AVFoundation; always exposes the latest one., Detect markers using a full-frame sweep as the authoritative source,…, VideoCapture

### Community 78 - "main.py"
Cohesion: 0.33
Nodes (5): pololu_3pi_2040_robot, pololu_3pi_2040_robot_display, pololu_3pi_2040_robot_extras_splash_loader, pololu_3pi_2040_robot_motors, pololu_3pi_2040_robot_rgb_leds

### Community 79 - "IPreprocessor"
Cohesion: 0.20
Nodes (7): CLAHE, CLAHEPreprocessor, clahe_, IPreprocessor, process, Ptr, unique_ptr

### Community 80 - "RingBuffer"
Cohesion: 0.25
Nodes (5): N, RingBuffer, buf_, count_, head_

### Community 81 - "msc-flash.py"
Cohesion: 0.27
Nodes (10): shutil, onboard_verify_problems(), Reconnects fresh (soft-reset, so nothing else is running) and verifies on-…, copy_via_msc(), flash(), main(), mounted_volume(), Returns the mounted MicroPython volume's path, or None. (+2 more)

### Community 82 - "Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 83 - "Ring"
Cohesion: 0.18
Nodes (11): Point, Point2f, unordered_map, fitRing(), pixelToWorld(), Ring, centre, centreSet (+3 more)

### Community 84 - "ICameraSource"
Cohesion: 0.20
Nodes (4): ICameraSource, open, read, size

### Community 86 - ".drawText"
Cohesion: 0.29
Nodes (9): Point, Scalar, runCalibration(), Mat, Size, drawGrid(), main(), runCalibration() (+1 more)

### Community 87 - "IOptimizer"
Cohesion: 0.20
Nodes (8): IOptimizer, ask, bestFit, converged, generation, lambda, setMean, tell

### Community 88 - "Buffering"
Cohesion: 0.20
Nodes (9): Buffering, b, id, deque, trajectoriesJson(), TrajSample, id, s (+1 more)

### Community 89 - "Servo"
Cohesion: 0.20
Nodes (10): Servo, alignIntegral, everSeen, firstSeen, lastSeen, prevTurn, registered, yaw (+2 more)

### Community 90 - "EvdevKeyboard"
Cohesion: 0.25
Nodes (5): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector

### Community 91 - "PoseHubHeader"
Cohesion: 0.25
Nodes (8): PoseHubHeader, count, detFps, frameH, frameW, magic, seq, version

### Community 92 - "applyParams"
Cohesion: 0.29
Nodes (8): applyParams(), CfLayout, CfModel, layoutFromName(), layoutName(), PageState, run, setupNo

### Community 93 - "TestState"
Cohesion: 0.29
Nodes (7): time_point, TestState, running, status, stepIdx, stepStart, suiteIdx

### Community 94 - "RateEstimator"
Cohesion: 0.33
Nodes (7): time_point, RateEstimator, baseAngle, baseTime, init, rate, updateRate()

### Community 95 - "DetectionResult"
Cohesion: 0.33
Nodes (6): DetectionResult, debug, fps, fresh, latencyMs, robots

### Community 96 - "DebugEntry"
Cohesion: 0.33
Nodes (6): DebugEntry, at, fieldId, robotId, text, time_point

### Community 97 - "PI"
Cohesion: 0.33
Nodes (3): PI, Discrete PI controller with anti-windup integral clamp. Target and measurement…, Returns clamped motor power for this timestep.

### Community 98 - "Snapshot"
Cohesion: 0.40
Nodes (5): pair, time_point, vector, Snapshot, robots

### Community 99 - "ClientConn"
Cohesion: 0.40
Nodes (5): ClientConn, active, fd, rxBuf, rxLen

### Community 100 - "ClientConn"
Cohesion: 0.40
Nodes (5): ClientConn, active, fd, rxBuf, rxLen

### Community 101 - "RobotRowLayout"
Cohesion: 0.67
Nodes (3): RobotRowLayout, meterWidth, sparkWidth

## Knowledge Gaps
- **583 isolated node(s):** `open`, `id`, `x`, `y`, `yaw` (+578 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 874 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **7 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoTracker` connect `ArucoTracker` to `swarm_telemetry_json.cpp`, `thread`, `vector`, `RobotPose`, `circle_speed_test.cpp`, `circle_demo.cpp`, `string`, `vision_controller.cpp`, `ArucoConfig`, `shape_demo.cpp`, `IPreprocessor`, `swarmclient`, `ICameraSource`, `.drawText`, `wingman.cpp`, `drag_drop_demo.cpp`, `.detectionLoop`, `DetectionResult`?**
  _High betweenness centrality (0.083) - this node is a cross-community bridge._
- **Why does `ArucoConfig` connect `ArucoConfig` to `param_space.h`, `string`, `calib_main.cpp`, `ArucoTracker`, `CMAES`, `main`, `BaslerPylonSource`, `objective.h`?**
  _High betweenness centrality (0.046) - this node is a cross-community bridge._
- **Why does `main()` connect `main` to `CfRing`, `car_following.cpp`, `HttpBridge`, `test_car_following.cpp`, `SwarmClient`, `main`, `Ring`, `Buffering`, `applyParams`, `RateEstimator`?**
  _High betweenness centrality (0.042) - this node is a cross-community bridge._
- **Are the 13 inferred relationships involving `CfRing` (e.g. with `test_a_dropout_keeps_its_place_on_the_ring()` and `test_a_lagging_robot_still_gets_the_ring_moving()`) actually correct?**
  _`CfRing` has 13 INFERRED edges - model-reasoned connections that need verification._
- **Are the 30 inferred relationships involving `main()` (e.g. with `cfModelFromName()` and `cfModelHasDesiredGap()`) actually correct?**
  _`main()` has 30 INFERRED edges - model-reasoned connections that need verification._
- **Are the 6 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 6 INFERRED edges - model-reasoned connections that need verification._
- **What connects `open`, `id`, `x` to the rest of the system?**
  _583 weakly-connected nodes found - possible documentation gaps or missing edges._