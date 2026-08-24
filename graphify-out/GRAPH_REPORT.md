# Graph Report - robot-swarm  (2026-08-24)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1381 nodes · 2374 edges · 79 communities (73 shown, 6 thin omitted)
- Extraction: 92% EXTRACTED · 8% INFERRED · 0% AMBIGUOUS · INFERRED: 184 edges (avg confidence: 0.81)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `8c6ca610`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- app.js
- swarm_hub.cpp
- swarm_dashboard.cpp
- main
- Vehicle
- shape_demo.cpp
- receiver/main.cpp
- wingman.cpp
- ArucoConfig
- ArucoTracker
- swarm_controller.cpp
- Robot Swarm Pipeline — Architekturplan
- Screen
- main
- dongle/main.cpp
- DemoHud
- SwarmClient
- vision_controller.cpp
- CMAES
- aruco_tracker.h
- package.json
- UARTProtocol
- drag_drop_demo.cpp
- .drawText
- ScreenManager
- flash.py
- BaslerPylonSource
- screen_manager.py
- ================== FIXED ===================
- Architecture
- CaptureThread
- vector
- MarkerState
- DemoHud.h
- circle_demo
- param_space.h
- main
- main
- drawTelHud
- .connect
- robot_uart.py
- uart_controller.py
- IPreprocessor
- CMA-ES ArUco Detector Calibration
- Wire Protocol: Three Independent Implementations
- StaticObjective
- circle_demo sleep_for(30ms) Poll Root Cause
- `swarm_hub`
- MSG_TELEMETRY Packet Format
- Args
- Pololu 3pi+ 2040 Swarm Control
- IOptimizer
- PC Tools
- objective.h
- generate_marker_stl.py
- ControlScore
- MicroPython Feature-Flag + Isolated-Module Pattern
- Safety Watchdogs Live on the Robot
- CRC Error Handling (Silent Drop)
- Latency Budget (~4ms End-to-End)
- ODOMETRY_ENABLED Control-Strategy Toggle
- ICameraSource
- runOptimisation
- Robot Swarm
- generate_markers_pdf.py
- PoseHubHeader
- LineGraph
- swarm_terminal.cpp
- Fisheye Calibration (planned)
- latency_plot.cpp
- PI
- AvoidState
- IObjective
- ArUco Tracker Calibration
- CalibState
- Quick Start
- Planned extensions

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 75 edges
2. `ArucoConfig` - 55 edges
3. `SwarmClient` - 46 edges
4. `Vehicle` - 37 edges
5. `CMAES` - 36 edges
6. `DemoHud` - 30 edges
7. `main()` - 30 edges
8. `Screen` - 25 edges
9. `main()` - 24 edges
10. `html` - 22 edges

## Surprising Connections (you probably didn't know these)
- `main()` --references--> `SwarmClient`  [INFERRED]
  tools/swarm/latency_plot.cpp → lib/SwarmClient/SwarmClient.h
- `main()` --references--> `SwarmClient`  [INFERRED]
  tools/swarm/swarm_telemetry_json.cpp → lib/SwarmClient/SwarmClient.h
- `main()` --references--> `SwarmClient`  [INFERRED]
  tools/vision/circle_demo.cpp → lib/SwarmClient/SwarmClient.h
- `main()` --references--> `SwarmClient`  [INFERRED]
  tools/vision/shape_demo.cpp → lib/SwarmClient/SwarmClient.h
- `drawUI()` --calls--> `scBatteryVolts()`  [INFERRED]
  tools/swarm/swarm_dashboard.cpp → lib/SwarmClient/SwarmClient.h

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **circle_demo Performance Root-Cause Investigation** — performance_circle_demo_sleep_poll_rootcause, performance_governor_epp_red_herring, performance_half_res_sweep, performance_loop_fps_vs_cam_fps [EXTRACTED 0.90]
- **Vision-Based Swarm Controllers** — readme_vision_controller, readme_wingman, readme_circle_demo, readme_shape_demo [EXTRACTED 0.90]
- **Wire Protocol Three-Language Implementation** — claudemd_wire_protocol_three_implementations, readme_wire_protocol_frame_format, src_robots_readme_uartprotocol, docs_architecture_crc_error_handling [INFERRED 0.85]

## Communities (79 total, 6 thin omitted)

### Community 0 - "app.js"
Cohesion: 0.07
Nodes (52): App(), clamp(), computeLayout(), snapshot(), sortRobots(), SORTS, useTerminalSize(), app (+44 more)

### Community 1 - "swarm_hub.cpp"
Cohesion: 0.06
Nodes (57): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector, any_client_active(), broadcast_to_clients(), build_ping() (+49 more)

### Community 2 - "swarm_dashboard.cpp"
Cohesion: 0.05
Nodes (47): array, unordered_map, RobotState, battery, flags, hasTelemetry, known, lastPongAt (+39 more)

### Community 3 - "main"
Cohesion: 0.08
Nodes (37): vector, HubPose, id, px, py, x, y, yaw (+29 more)

### Community 4 - "Vehicle"
Cohesion: 0.07
Nodes (44): Color, Font, RenderWindow, alpha(), pair, string, draw_boost(), draw_drift_badge() (+36 more)

### Community 5 - "shape_demo.cpp"
Cohesion: 0.09
Nodes (37): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+29 more)

### Community 6 - "receiver/main.cpp"
Cohesion: 0.09
Nodes (32): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+24 more)

### Community 7 - "wingman.cpp"
Cohesion: 0.09
Nodes (36): CGEventRef, CGEventTapProxy, CGEventType, RobotPose, id, px, py, x (+28 more)

### Community 8 - "ArucoConfig"
Cohesion: 0.05
Nodes (39): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+31 more)

### Community 9 - "ArucoTracker"
Cohesion: 0.06
Nodes (28): ArucoTracker, captureRunning_, captureThread_, cfg_, clahe_, debug_, detectionRunning_, detectionThread_ (+20 more)

### Community 10 - "swarm_controller.cpp"
Cohesion: 0.10
Nodes (31): advanceTest(), applyTestStep(), KeyHandle, time_point, vector, drawUI(), handleInput(), keyDown() (+23 more)

### Community 11 - "Robot Swarm Pipeline — Architekturplan"
Cohesion: 0.06
Nodes (34): 1. Roboter-Registrierung, 2. Steuerkanal (PC → Roboter), 3. Telemetrie-Rückkanal (Roboter → PC), 4. Protokoll-Übersicht, 5. Fehlerbehandlung, 6. Controller-PC Software, 7. Hardware-Checkliste pro Roboter, 8. Implementierungsreihenfolge (+26 more)

### Community 12 - "Screen"
Cohesion: 0.07
Nodes (15): Repräsentiert einen einzelnen Debug-Screen. Verwendung: screen = Screen("MY…, Fügt eine neue Nachricht zum Log hinzu., Löscht alle Log-Einträge., Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück., Entfernt eine einzelne Primitive per Handle. (+7 more)

### Community 13 - "main"
Cohesion: 0.11
Nodes (26): assignNearestSlots(), CircleState, centre, centreSet, minGapMm, orbitSpeed, radius, tracking (+18 more)

### Community 14 - "dongle/main.cpp"
Cohesion: 0.09
Nodes (29): anyRobotActive(), enqueueSend(), ensurePeer(), ledOff(), ledOn(), loop(), onReceive(), PingTracker (+21 more)

### Community 15 - "DemoHud"
Cohesion: 0.11
Nodes (21): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+13 more)

### Community 16 - "SwarmClient"
Cohesion: 0.12
Nodes (12): mutex, vector, SwarmClient, m_debugLog, m_fd, m_robots, m_rxBuf, m_rxLen (+4 more)

### Community 17 - "vision_controller.cpp"
Cohesion: 0.11
Nodes (21): atomic, CalibState, done, pixPts, clampf(), KeyHandle, Point2f, vector (+13 more)

### Community 18 - "CMAES"
Cohesion: 0.08
Nodes (23): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+15 more)

### Community 19 - "aruco_tracker.h"
Cohesion: 0.13
Nodes (14): condition_variable, function, defaultConfigPath, fromFile, mutex, string, ThreadPool, cv_ (+6 more)

### Community 20 - "package.json"
Cohesion: 0.10
Nodes (20): htm, ink, react, bin, swarm-dashboard-ink, dependencies, htm, ink (+12 more)

### Community 21 - "UARTProtocol"
Cohesion: 0.15
Nodes (9): Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED). :param left: Geschwindigkeit…, Sendet die Batteriespannung an den ESP32 (MSG_METRICS). :param battery_byte:…, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG). Der ESP32 stellt die…, Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., Verwaltet das Senden und Empfangen von strukturierten UART-Paketen. MSG_SPEED…, UARTProtocol (+1 more)

### Community 22 - "drag_drop_demo.cpp"
Cohesion: 0.14
Nodes (18): buildAvoidance(), CalibState, done, pixPts, clampf(), Point2f, unordered_map, vector (+10 more)

### Community 23 - ".drawText"
Cohesion: 0.11
Nodes (13): DetectionResult, debug, fps, fresh, latencyMs, robots, FisheyeUndistortPreprocessor, map1_ (+5 more)

### Community 24 - "ScreenManager"
Cohesion: 0.12
Nodes (11): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display. Knopfdruck…, :param display: robot.Display() Instanz :param button: Button-Instanz mit…, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+3 more)

### Community 25 - "flash.py"
Cohesion: 0.20
Nodes (18): connected_robot_ports(), deploy_cmd(), deploy_problems(), eject_micropython_volume(), flash(), main(), onboard_verify_problems(), Returns human-readable problems with a deploy attempt; empty = verified OK. (+10 more)

### Community 26 - "BaslerPylonSource"
Cohesion: 0.12
Nodes (11): CBaslerUniversalInstantCamera, CImageFormatConverter, BaslerPylonSource, camera_, converter_, height_, pylonRuntime_, width_ (+3 more)

### Community 27 - "screen_manager.py"
Cohesion: 0.14
Nodes (8): BarGraph, _draw_circle(), _draw_line(), Gauge, Balkendiagramm für einen einzelnen Wert. Verwendung: bar =…, Halbkreis-Gauge für einen einzelnen Wert. Verwendung: gauge =…, Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-…, Bresenham-Kreis direkt auf das Display zeichnen.

### Community 28 - "================== FIXED ==================="
Cohesion: 0.12
Nodes (16): Bug: Latency Erledigt (2026-06-18), Erledigt (2026-06-10), Erledigt (2026-06-12), Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06), ================== FIXED ===================, Genera aruco tracker issue: Erledigt (2026-07-08), Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12), questions to answer: (+8 more)

### Community 29 - "Architecture"
Cohesion: 0.12
Nodes (14): Architecture, Commands, Firmware (PlatformIO — `src/dongle`, `src/receiver`), MicroPython robot firmware: feature-flag + isolated-module pattern, PC tools (`tools/`, plain Makefile), Project Overview, Repo layout, Robot firmware (`src/robots/`, MicroPython) (+6 more)

### Community 30 - "CaptureThread"
Cohesion: 0.17
Nodes (10): CaptureThread, _detect_with_roi(), _find_gopro_index(), main(), ArucoDetector, ndarray, Find the AVFoundation index for the connected GoPro camera. Two-step lookup: 1.…, Continuously grabs frames via AVFoundation; always exposes the latest one. (+2 more)

### Community 31 - "vector"
Cohesion: 0.22
Nodes (7): F, KalmanFilter, Point2f, Scalar, vector, submit(), Rect

### Community 32 - "MarkerState"
Cohesion: 0.14
Nodes (14): time_point, MarkerState, bboxSize, center, failCount, globalFrames, kf, kfInit (+6 more)

### Community 33 - "DemoHud.h"
Cohesion: 0.18
Nodes (9): time_point, LoopFps, count_, last_, Mat, Scalar, drawPanel(), main() (+1 more)

### Community 34 - "circle_demo"
Cohesion: 0.13
Nodes (14): 2026-06-12 — back to `powersave` (after the `performance` test), 2026-06-12 — Baseline (revised: split by tracking state), 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config), 2026-06-12 — governor `powersave` → `performance`, 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change, 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default), 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop, circle_demo (+6 more)

### Community 35 - "param_space.h"
Cohesion: 0.22
Nodes (12): decode(), encode(), fromNorm(), string, vector, ParamSpec, hi, key (+4 more)

### Community 36 - "main"
Cohesion: 0.23
Nodes (12): Mat, Point2f, vector, drawDetections(), FrameDetections, corners, ids, isEnd() (+4 more)

### Community 37 - "main"
Cohesion: 0.30
Nodes (9): bestX_, captureInteractive(), Mat, string, vector, loadFrameCache(), main(), parseArgs() (+1 more)

### Community 38 - "drawTelHud"
Cohesion: 0.42
Nodes (6): string, Mode, main(), Mat, drawTelHud(), drawTelHud()

### Community 39 - ".connect"
Cohesion: 0.18
Nodes (7): DebugEntry, at, fieldId, robotId, text, string, time_point

### Community 40 - "robot_uart.py"
Cohesion: 0.18
Nodes (8): build_packet(), _crc8(), _crc8_buf(), Packet, CRC-8 via Lookup-Table (Polynom 0x07)., CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig., Verpackt Nutzdaten in einen vollstaendigen Frame. Frame:…, Repraesentiert ein empfangenes Paket.

### Community 41 - "uart_controller.py"
Cohesion: 0.18
Nodes (10): _battery_byte(), _draw_scaled_text(), on_packet(), process_pending(), Render text at (x, y) with pixel-doubled scaling using a temp framebuf., Measure wheel speeds and run PID; called at PID_INTERVAL_MS cadence., Store the latest speed packet; applied on next loop iteration., Convert the pending signed bytes to target wheel speeds (counts/s). (+2 more)

### Community 42 - "IPreprocessor"
Cohesion: 0.20
Nodes (7): CLAHE, CLAHEPreprocessor, clahe_, IPreprocessor, process, Ptr, unique_ptr

### Community 43 - "CMA-ES ArUco Detector Calibration"
Cohesion: 0.32
Nodes (8): Vision Pipeline as Separate Concern, `marker_eval`, CMA-ES ArUco Detector Calibration, GP-ARD Bayesian Optimizer (planned), IOptimizer ask/tell Interface, Motion Objective Scoring, Scoring, Static objective

### Community 44 - "Wire Protocol: Three Independent Implementations"
Cohesion: 0.29
Nodes (7): Wire Protocol: Three Independent Implementations, Wire Protocol Frame Format, flash.py Batch Flashing, ScreenManager Display Manager, uart_controller.py Main Loop, UARTProtocol Library (robot_uart.py), Remote Robot Shutdown (scrapped)

### Community 45 - "StaticObjective"
Cohesion: 0.24
Nodes (8): Mat, string, vector, StaticObjective, detectedIds_, frames_, ArucoDetector, makeDetector()

### Community 46 - "circle_demo sleep_for(30ms) Poll Root Cause"
Cohesion: 0.33
Nodes (6): circle_demo sleep_for(30ms) Poll Root Cause, Governor/EPP/Throttling Red Herring, half_res_sweep Config Optimization, loop_fps vs cam_fps Gap, `circle_demo`, Generalized DEBUG/HUD Util (planned)

### Community 47 - "`swarm_hub`"
Cohesion: 0.40
Nodes (5): Centralized Round-Robin Pinging in swarm_hub, swarm_hub Owns the Serial Port, Pylon rpath Link Fix, `swarm_hub`, Per-Client Ping Clobbering Bug (fixed)

### Community 48 - "MSG_TELEMETRY Packet Format"
Cohesion: 0.40
Nodes (5): Robot Registration and Addressing, MSG_TELEMETRY Packet Format, Robot Registration via Announce, TDMA Telemetry Return Channel, MSG_METRICS Battery Channel (0x03)

### Community 49 - "Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 50 - "Pololu 3pi+ 2040 Swarm Control"
Cohesion: 0.18
Nodes (10): Batch flashing, Data Pipeline, Debug Display Types, Dependencies, Display, Files, Message Types, Packet Format (+2 more)

### Community 51 - "IOptimizer"
Cohesion: 0.20
Nodes (8): IOptimizer, ask, bestFit, converged, generation, lambda, setMean, tell

### Community 52 - "PC Tools"
Cohesion: 0.20
Nodes (11): `calibrate`, evdev WASD Keyboard Input, `frame_inspector`, `latency_plot`, PC Tools, `shape_demo`, `swarm_controller`, `swarm_terminal` (+3 more)

### Community 53 - "objective.h"
Cohesion: 0.24
Nodes (9): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+1 more)

### Community 54 - "generate_marker_stl.py"
Cohesion: 0.31
Nodes (9): box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Return a 6×6 list-of-lists (1 = black, 0 = white) via OpenCV., 12 outward-facing triangles for an axis-aligned box., _tri() (+1 more)

### Community 55 - "ControlScore"
Cohesion: 0.20
Nodes (10): ControlScore, biasEma, init, jerkVar, oscVar, posBiasEma, posVar, prevTurn (+2 more)

### Community 56 - "MicroPython Feature-Flag + Isolated-Module Pattern"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

### Community 61 - "ICameraSource"
Cohesion: 0.22
Nodes (4): ICameraSource, open, read, size

### Community 63 - "Robot Swarm"
Cohesion: 0.20
Nodes (9): Architecture, Camera Setup (Basler ace2 GigE), Firmware (all platforms), PC Tools — macOS (Apple Silicon), PC Tools — Ubuntu 22.04 / 24.04 (x86\_64), Prerequisites, Project Structure, Robot Swarm (+1 more)

### Community 64 - "generate_markers_pdf.py"
Cohesion: 0.36
Nodes (8): generate_pdf(), main(), _marker_image(), _ndarray_to_png_bytes(), ndarray, generate_markers.py — print ArUco or AprilTag marker sheets as a PDF. Usage: #…, Return (cv2_dict_id, max_n, label_prefix) from CLI arguments., _resolve_dict()

### Community 65 - "PoseHubHeader"
Cohesion: 0.25
Nodes (8): PoseHubHeader, count, detFps, frameH, frameW, magic, seq, version

### Community 66 - "LineGraph"
Cohesion: 0.29
Nodes (4): LineGraph, Scrollender Liniengraph für kontinuierliche Werte. Verwendung: graph =…, Fügt einen neuen Messwert hinzu., Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest.

### Community 67 - "swarm_terminal.cpp"
Cohesion: 0.38
Nodes (3): drawUI(), main(), statusParts()

### Community 69 - "latency_plot.cpp"
Cohesion: 0.47
Nodes (4): deque, drawPlot(), main(), onPong()

### Community 70 - "PI"
Cohesion: 0.33
Nodes (3): PI, Discrete PI controller with anti-windup integral clamp. Target and measurement…, Returns clamped motor power for this timestep.

### Community 71 - "AvoidState"
Cohesion: 0.40
Nodes (5): AvoidState, arc, arcDx, arcDy, minDist

### Community 72 - "IObjective"
Cohesion: 0.50
Nodes (3): IObjective, evaluate, name

### Community 73 - "ArUco Tracker Calibration"
Cohesion: 0.29
Nodes (6): All flags, ArUco Tracker Calibration, File structure, How it works, Parameters being optimised, Quick start

### Community 74 - "CalibState"
Cohesion: 0.50
Nodes (4): CalibState, done, pixPts, vector

### Community 75 - "Quick Start"
Cohesion: 0.33
Nodes (6): 1. Flash the dongle, 2. Flash each robot, 3. Build PC tools, 4. Find the dongle serial port, 5. Run the hub, Quick Start

### Community 76 - "Planned extensions"
Cohesion: 0.33
Nodes (6): Combined static + motion objective, Fisheye calibration (`objective_fisheye.h`), GP with ARD kernel (`gp_ard.h`), Motion objective (`objective_motion.h`), Parameter range narrowing, Planned extensions

## Knowledge Gaps
- **473 isolated node(s):** `termios`, `SORTS`, `app`, `argv`, `ARROWS` (+468 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **6 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoConfig` connect `ArucoConfig` to `param_space.h`, `main`, `ArucoTracker`, `StaticObjective`, `aruco_tracker.h`, `objective.h`, `BaslerPylonSource`, `runOptimisation`?**
  _High betweenness centrality (0.072) - this node is a cross-community bridge._
- **Why does `ArucoTracker` connect `ArucoTracker` to `MarkerState`, `swarm_dashboard.cpp`, `main`, `wingman.cpp`, `ArucoConfig`, `IPreprocessor`, `main`, `vision_controller.cpp`, `aruco_tracker.h`, `drag_drop_demo.cpp`, `.drawText`, `ICameraSource`, `vector`?**
  _High betweenness centrality (0.066) - this node is a cross-community bridge._
- **Why does `SwarmClient` connect `SwarmClient` to `swarm_dashboard.cpp`, `main`, `swarm_terminal.cpp`, `shape_demo.cpp`, `latency_plot.cpp`, `.connect`, `drawTelHud`, `main`, `aruco_tracker.h`, `drag_drop_demo.cpp`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **Are the 5 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 5 INFERRED edges - model-reasoned connections that need verification._
- **What connects `termios`, `SORTS`, `app` to the rest of the system?**
  _473 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `app.js` be split into smaller, more focused modules?**
  _Cohesion score 0.07026307026307026 - nodes in this community are weakly interconnected._
- **Should `swarm_hub.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.05879692446856626 - nodes in this community are weakly interconnected._