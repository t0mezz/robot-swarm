# Graph Report - robot-swarm  (2026-08-28)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1596 nodes · 2947 edges · 80 communities (75 shown, 5 thin omitted)
- Extraction: 88% EXTRACTED · 12% INFERRED · 0% AMBIGUOUS · INFERRED: 366 edges (avg confidence: 0.81)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `39598611`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- CfRing
- app.js
- main
- swarm_hub.cpp
- Vehicle
- car_following.cpp
- circle_demo.cpp
- receiver/main.cpp
- shape_demo.cpp
- ArucoConfig
- SwarmClient
- Robot Swarm Pipeline — Architekturplan
- wingman.cpp
- car_following.h
- ArucoTracker
- CMAES
- swarm_controller.cpp
- Screen
- dongle/main.cpp
- vector
- DemoHud
- objective.h
- drag_drop_demo.cpp
- vision_controller.cpp
- package.json
- UARTProtocol
- main
- HttpBridge
- ================== FIXED ===================
- Architecture
- ScreenManager
- flash.py
- BaslerPylonSource
- unordered_map
- circle_demo
- screen_manager.py
- swarm_dashboard.cpp
- CaptureThread
- aruco_tracker.h
- DetectionResult
- param_space.h
- main
- IPreprocessor
- CMA-ES ArUco Detector Calibration
- Wire Protocol: Three Independent Implementations
- main
- circle_demo sleep_for(30ms) Poll Root Cause
- `swarm_hub`
- MSG_TELEMETRY Packet Format
- drawPanel
- Pololu 3pi+ 2040 Swarm Control
- RobotState
- PC Tools
- robot_uart.py
- uart_controller.py
- RingBuffer
- MicroPython Feature-Flag + Isolated-Module Pattern
- Safety Watchdogs Live on the Robot
- CRC Error Handling (Silent Drop)
- Latency Budget (~4ms End-to-End)
- ODOMETRY_ENABLED Control-Strategy Toggle
- Args
- string
- Robot Swarm
- ICameraSource
- IOptimizer
- generate_marker_stl.py
- generate_markers_pdf.py
- Fisheye Calibration (planned)
- LineGraph
- DebugEntry
- PI
- car_following_bridge.js
- ArUco Tracker Calibration
- Snapshot
- Quick Start
- Planned extensions
- RobotRowLayout

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 75 edges
2. `ArucoConfig` - 55 edges
3. `SwarmClient` - 47 edges
4. `CfRing` - 45 edges
5. `main()` - 44 edges
6. `Vehicle` - 37 edges
7. `CMAES` - 36 edges
8. `main()` - 31 edges
9. `DemoHud` - 30 edges
10. `main()` - 30 edges

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

## Communities (80 total, 5 thin omitted)

### Community 0 - "CfRing"
Cohesion: 0.05
Nodes (74): CfPhase, CfRunEvent, cfNormAngleDeg(), CfRing, cars_, cfg_, order_, pendingCount_ (+66 more)

### Community 1 - "app.js"
Cohesion: 0.07
Nodes (52): App(), clamp(), computeLayout(), snapshot(), sortRobots(), SORTS, useTerminalSize(), app (+44 more)

### Community 2 - "main"
Cohesion: 0.06
Nodes (46): vector, HubPose, id, px, py, x, y, yaw (+38 more)

### Community 3 - "swarm_hub.cpp"
Cohesion: 0.06
Nodes (57): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector, any_client_active(), broadcast_to_clients(), build_ping() (+49 more)

### Community 4 - "Vehicle"
Cohesion: 0.06
Nodes (48): Color, deque, Font, RenderWindow, alpha(), pair, string, draw_boost() (+40 more)

### Community 5 - "car_following.cpp"
Cohesion: 0.06
Nodes (39): applyParams(), Buffering, b, id, clampf(), CfModel, Point, Point2f (+31 more)

### Community 6 - "circle_demo.cpp"
Cohesion: 0.06
Nodes (39): assignNearestSlots(), CalibState, done, pixPts, CircleState, centre, centreSet, minGapMm (+31 more)

### Community 7 - "receiver/main.cpp"
Cohesion: 0.08
Nodes (32): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+24 more)

### Community 8 - "shape_demo.cpp"
Cohesion: 0.09
Nodes (38): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+30 more)

### Community 9 - "ArucoConfig"
Cohesion: 0.05
Nodes (39): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+31 more)

### Community 10 - "SwarmClient"
Cohesion: 0.09
Nodes (16): mutex, string, vector, SwarmClient, m_debugLog, m_fd, m_robots, m_rxBuf (+8 more)

### Community 11 - "Robot Swarm Pipeline — Architekturplan"
Cohesion: 0.06
Nodes (34): 1. Roboter-Registrierung, 2. Steuerkanal (PC → Roboter), 3. Telemetrie-Rückkanal (Roboter → PC), 4. Protokoll-Übersicht, 5. Fehlerbehandlung, 6. Controller-PC Software, 7. Hardware-Checkliste pro Roboter, 8. Implementierungsreihenfolge (+26 more)

### Community 12 - "wingman.cpp"
Cohesion: 0.08
Nodes (35): CGEventRef, CGEventTapProxy, CGEventType, RobotPose, id, px, py, x (+27 more)

### Community 13 - "car_following.h"
Cohesion: 0.14
Nodes (35): cfAcceleration(), cfBufferedParams(), cfBufferedTimeGap(), CfInput, gap, predGap, predSpeed, speed (+27 more)

### Community 14 - "ArucoTracker"
Cohesion: 0.06
Nodes (28): ArucoTracker, captureRunning_, captureThread_, cfg_, clahe_, debug_, detectionRunning_, detectionThread_ (+20 more)

### Community 15 - "CMAES"
Cohesion: 0.07
Nodes (25): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+17 more)

### Community 16 - "swarm_controller.cpp"
Cohesion: 0.10
Nodes (31): advanceTest(), applyTestStep(), KeyHandle, time_point, vector, drawUI(), handleInput(), keyDown() (+23 more)

### Community 17 - "Screen"
Cohesion: 0.07
Nodes (15): Repräsentiert einen einzelnen Debug-Screen. Verwendung: screen = Screen("MY…, Fügt eine neue Nachricht zum Log hinzu., Löscht alle Log-Einträge., Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück., Entfernt eine einzelne Primitive per Handle. (+7 more)

### Community 18 - "dongle/main.cpp"
Cohesion: 0.10
Nodes (29): anyRobotActive(), enqueueSend(), ensurePeer(), ledOff(), ledOn(), loop(), onReceive(), PingTracker (+21 more)

### Community 19 - "vector"
Cohesion: 0.10
Nodes (19): KalmanFilter, Point2f, Scalar, time_point, vector, MarkerState, bboxSize, center (+11 more)

### Community 20 - "DemoHud"
Cohesion: 0.11
Nodes (21): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+13 more)

### Community 21 - "objective.h"
Cohesion: 0.11
Nodes (20): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+12 more)

### Community 22 - "drag_drop_demo.cpp"
Cohesion: 0.11
Nodes (22): AvoidState, arc, arcDx, arcDy, minDist, buildAvoidance(), CalibState, done (+14 more)

### Community 23 - "vision_controller.cpp"
Cohesion: 0.09
Nodes (20): atomic, CalibState, done, pixPts, clampf(), KeyHandle, Point2f, vector (+12 more)

### Community 24 - "package.json"
Cohesion: 0.10
Nodes (20): htm, ink, react, bin, swarm-dashboard-ink, dependencies, htm, ink (+12 more)

### Community 25 - "UARTProtocol"
Cohesion: 0.15
Nodes (9): Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED). :param left: Geschwindigkeit…, Sendet die Batteriespannung an den ESP32 (MSG_METRICS). :param battery_byte:…, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG). Der ESP32 stellt die…, Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., Verwaltet das Senden und Empfangen von strukturierten UART-Paketen. MSG_SPEED…, UARTProtocol (+1 more)

### Community 26 - "main"
Cohesion: 0.35
Nodes (11): fromFile, string, Mode, main(), main(), main(), Mat, drawTelHud() (+3 more)

### Community 27 - "HttpBridge"
Cohesion: 0.17
Nodes (11): Conn, answered, fd, rx, tx, string, vector, HttpBridge (+3 more)

### Community 28 - "================== FIXED ==================="
Cohesion: 0.12
Nodes (16): Bug: Latency Erledigt (2026-06-18), Erledigt (2026-06-10), Erledigt (2026-06-12), Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06), ================== FIXED ===================, Genera aruco tracker issue: Erledigt (2026-07-08), Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12), questions to answer: (+8 more)

### Community 29 - "Architecture"
Cohesion: 0.12
Nodes (14): Architecture, Commands, Firmware (PlatformIO — `src/dongle`, `src/receiver`), MicroPython robot firmware: feature-flag + isolated-module pattern, PC tools (`tools/`, plain Makefile), Project Overview, Repo layout, Robot firmware (`src/robots/`, MicroPython) (+6 more)

### Community 30 - "ScreenManager"
Cohesion: 0.12
Nodes (11): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display. Knopfdruck…, :param display: robot.Display() Instanz :param button: Button-Instanz mit…, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+3 more)

### Community 31 - "flash.py"
Cohesion: 0.20
Nodes (18): connected_robot_ports(), deploy_cmd(), deploy_problems(), eject_micropython_volume(), flash(), main(), onboard_verify_problems(), Returns human-readable problems with a deploy attempt; empty = verified OK. (+10 more)

### Community 32 - "BaslerPylonSource"
Cohesion: 0.12
Nodes (11): CBaslerUniversalInstantCamera, CImageFormatConverter, BaslerPylonSource, camera_, converter_, height_, pylonRuntime_, width_ (+3 more)

### Community 33 - "unordered_map"
Cohesion: 0.15
Nodes (10): array, unordered_map, Buffer, unordered_map, vector, PerRobot, buffers, TelemetryHistory (+2 more)

### Community 34 - "circle_demo"
Cohesion: 0.13
Nodes (14): 2026-06-12 — back to `powersave` (after the `performance` test), 2026-06-12 — Baseline (revised: split by tracking state), 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config), 2026-06-12 — governor `powersave` → `performance`, 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change, 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default), 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop, circle_demo (+6 more)

### Community 35 - "screen_manager.py"
Cohesion: 0.14
Nodes (8): BarGraph, _draw_circle(), _draw_line(), Gauge, Balkendiagramm für einen einzelnen Wert. Verwendung: bar =…, Halbkreis-Gauge für einen einzelnen Wert. Verwendung: gauge =…, Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-…, Bresenham-Kreis direkt auf das Display zeichnen.

### Community 36 - "swarm_dashboard.cpp"
Cohesion: 0.30
Nodes (13): appendf(), bipolarMeter(), brailleGraph(), computeLayout(), Buffer, string, vector, drawUI() (+5 more)

### Community 37 - "CaptureThread"
Cohesion: 0.17
Nodes (10): CaptureThread, _detect_with_roi(), _find_gopro_index(), main(), ArucoDetector, ndarray, Find the AVFoundation index for the connected GoPro camera. Two-step lookup: 1.…, Continuously grabs frames via AVFoundation; always exposes the latest one. (+2 more)

### Community 38 - "aruco_tracker.h"
Cohesion: 0.18
Nodes (12): condition_variable, F, function, mutex, submit(), ThreadPool, cv_, mtx_ (+4 more)

### Community 39 - "DetectionResult"
Cohesion: 0.17
Nodes (10): DetectionResult, debug, fps, fresh, latencyMs, robots, FisheyeUndistortPreprocessor, map1_ (+2 more)

### Community 40 - "param_space.h"
Cohesion: 0.22
Nodes (12): decode(), encode(), fromNorm(), string, vector, ParamSpec, hi, key (+4 more)

### Community 41 - "main"
Cohesion: 0.23
Nodes (12): Mat, Point2f, vector, drawDetections(), FrameDetections, corners, ids, isEnd() (+4 more)

### Community 42 - "IPreprocessor"
Cohesion: 0.18
Nodes (7): CLAHE, CLAHEPreprocessor, clahe_, IPreprocessor, process, Ptr, unique_ptr

### Community 43 - "CMA-ES ArUco Detector Calibration"
Cohesion: 0.32
Nodes (8): Vision Pipeline as Separate Concern, `marker_eval`, CMA-ES ArUco Detector Calibration, GP-ARD Bayesian Optimizer (planned), IOptimizer ask/tell Interface, Motion Objective Scoring, Scoring, Static objective

### Community 44 - "Wire Protocol: Three Independent Implementations"
Cohesion: 0.29
Nodes (7): Wire Protocol: Three Independent Implementations, Wire Protocol Frame Format, flash.py Batch Flashing, ScreenManager Display Manager, uart_controller.py Main Loop, UARTProtocol Library (robot_uart.py), Remote Robot Shutdown (scrapped)

### Community 45 - "main"
Cohesion: 0.30
Nodes (9): bestX_, captureInteractive(), Mat, string, vector, loadFrameCache(), main(), parseArgs() (+1 more)

### Community 46 - "circle_demo sleep_for(30ms) Poll Root Cause"
Cohesion: 0.33
Nodes (6): circle_demo sleep_for(30ms) Poll Root Cause, Governor/EPP/Throttling Red Herring, half_res_sweep Config Optimization, loop_fps vs cam_fps Gap, `circle_demo`, Generalized DEBUG/HUD Util (planned)

### Community 47 - "`swarm_hub`"
Cohesion: 0.40
Nodes (5): Centralized Round-Robin Pinging in swarm_hub, swarm_hub Owns the Serial Port, Pylon rpath Link Fix, `swarm_hub`, Per-Client Ping Clobbering Bug (fixed)

### Community 48 - "MSG_TELEMETRY Packet Format"
Cohesion: 0.40
Nodes (5): Robot Registration and Addressing, MSG_TELEMETRY Packet Format, Robot Registration via Announce, TDMA Telemetry Return Channel, MSG_METRICS Battery Channel (0x03)

### Community 49 - "drawPanel"
Cohesion: 0.21
Nodes (9): time_point, LoopFps, count_, last_, Mat, Scalar, drawPanel(), main() (+1 more)

### Community 50 - "Pololu 3pi+ 2040 Swarm Control"
Cohesion: 0.18
Nodes (10): Batch flashing, Data Pipeline, Debug Display Types, Dependencies, Display, Files, Message Types, Packet Format (+2 more)

### Community 51 - "RobotState"
Cohesion: 0.17
Nodes (12): RobotState, battery, flags, hasTelemetry, known, lastPongAt, lastSeen, latencyUs (+4 more)

### Community 52 - "PC Tools"
Cohesion: 0.20
Nodes (11): `calibrate`, evdev WASD Keyboard Input, `frame_inspector`, `latency_plot`, PC Tools, `shape_demo`, `swarm_controller`, `swarm_terminal` (+3 more)

### Community 53 - "robot_uart.py"
Cohesion: 0.18
Nodes (8): build_packet(), _crc8(), _crc8_buf(), Packet, CRC-8 via Lookup-Table (Polynom 0x07)., CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig., Verpackt Nutzdaten in einen vollstaendigen Frame. Frame:…, Repraesentiert ein empfangenes Paket.

### Community 54 - "uart_controller.py"
Cohesion: 0.18
Nodes (10): _battery_byte(), _draw_scaled_text(), on_packet(), process_pending(), Render text at (x, y) with pixel-doubled scaling using a temp framebuf., Measure wheel speeds and run PID; called at PID_INTERVAL_MS cadence., Store the latest speed packet; applied on next loop iteration., Convert the pending signed bytes to target wheel speeds (counts/s). (+2 more)

### Community 55 - "RingBuffer"
Cohesion: 0.25
Nodes (5): N, RingBuffer, buf_, count_, head_

### Community 56 - "MicroPython Feature-Flag + Isolated-Module Pattern"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

### Community 61 - "Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 62 - "string"
Cohesion: 0.20
Nodes (5): defaultConfigPath, Point, Size, string, runCalibration()

### Community 63 - "Robot Swarm"
Cohesion: 0.20
Nodes (9): Architecture, Camera Setup (Basler ace2 GigE), Firmware (all platforms), PC Tools — macOS (Apple Silicon), PC Tools — Ubuntu 22.04 / 24.04 (x86\_64), Prerequisites, Project Structure, Robot Swarm (+1 more)

### Community 64 - "ICameraSource"
Cohesion: 0.20
Nodes (4): ICameraSource, open, read, size

### Community 65 - "IOptimizer"
Cohesion: 0.20
Nodes (8): IOptimizer, ask, bestFit, converged, generation, lambda, setMean, tell

### Community 66 - "generate_marker_stl.py"
Cohesion: 0.31
Nodes (9): box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Return a 6×6 list-of-lists (1 = black, 0 = white) via OpenCV., 12 outward-facing triangles for an axis-aligned box., _tri() (+1 more)

### Community 67 - "generate_markers_pdf.py"
Cohesion: 0.36
Nodes (8): generate_pdf(), main(), _marker_image(), _ndarray_to_png_bytes(), ndarray, generate_markers.py — print ArUco or AprilTag marker sheets as a PDF. Usage: #…, Return (cv2_dict_id, max_n, label_prefix) from CLI arguments., _resolve_dict()

### Community 69 - "LineGraph"
Cohesion: 0.29
Nodes (4): LineGraph, Scrollender Liniengraph für kontinuierliche Werte. Verwendung: graph =…, Fügt einen neuen Messwert hinzu., Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest.

### Community 70 - "DebugEntry"
Cohesion: 0.33
Nodes (6): DebugEntry, at, fieldId, robotId, text, time_point

### Community 71 - "PI"
Cohesion: 0.33
Nodes (3): PI, Discrete PI controller with anti-windup integral clamp. Target and measurement…, Returns clamped motor power for this timestep.

### Community 72 - "car_following_bridge.js"
Cohesion: 0.53
Nodes (4): buildPanel(), field(), store(), stored()

### Community 73 - "ArUco Tracker Calibration"
Cohesion: 0.29
Nodes (6): All flags, ArUco Tracker Calibration, File structure, How it works, Parameters being optimised, Quick start

### Community 74 - "Snapshot"
Cohesion: 0.50
Nodes (4): pair, time_point, Snapshot, robots

### Community 75 - "Quick Start"
Cohesion: 0.33
Nodes (6): 1. Flash the dongle, 2. Flash each robot, 3. Build PC tools, 4. Find the dongle serial port, 5. Run the hub, Quick Start

### Community 76 - "Planned extensions"
Cohesion: 0.33
Nodes (6): Combined static + motion objective, Fisheye calibration (`objective_fisheye.h`), GP with ARD kernel (`gp_ard.h`), Motion objective (`objective_motion.h`), Parameter range narrowing, Planned extensions

### Community 77 - "RobotRowLayout"
Cohesion: 0.67
Nodes (3): RobotRowLayout, meterWidth, sparkWidth

## Knowledge Gaps
- **534 isolated node(s):** `open`, `id`, `x`, `y`, `yaw` (+529 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **5 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoTracker` connect `ArucoTracker` to `ICameraSource`, `unordered_map`, `main`, `aruco_tracker.h`, `DetectionResult`, `circle_demo.cpp`, `ArucoConfig`, `IPreprocessor`, `shape_demo.cpp`, `wingman.cpp`, `vector`, `vision_controller.cpp`, `string`?**
  _High betweenness centrality (0.106) - this node is a cross-community bridge._
- **Why does `main()` connect `main` to `CfRing`, `car_following.cpp`, `SwarmClient`, `car_following.h`, `drawPanel`, `HttpBridge`?**
  _High betweenness centrality (0.045) - this node is a cross-community bridge._
- **Why does `ArucoConfig` connect `ArucoConfig` to `BaslerPylonSource`, `aruco_tracker.h`, `param_space.h`, `main`, `ArucoTracker`, `CMAES`, `objective.h`, `main`, `string`?**
  _High betweenness centrality (0.042) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 12 inferred relationships involving `CfRing` (e.g. with `test_a_dropout_keeps_its_place_on_the_ring()` and `test_a_lagging_robot_still_gets_the_ring_moving()`) actually correct?**
  _`CfRing` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 28 inferred relationships involving `main()` (e.g. with `cfModelFromName()` and `cfModelHasDesiredGap()`) actually correct?**
  _`main()` has 28 INFERRED edges - model-reasoned connections that need verification._
- **What connects `open`, `id`, `x` to the rest of the system?**
  _534 weakly-connected nodes found - possible documentation gaps or missing edges._