# Graph Report - robot-swarm  (2026-08-15)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1318 nodes · 2228 edges · 85 communities (78 shown, 7 thin omitted)
- Extraction: 92% EXTRACTED · 8% INFERRED · 0% AMBIGUOUS · INFERRED: 175 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `51254a10`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- lib/swarm/swarm_dashboard.cpp
- lib/swarm/latency_plot.cpp
- Vehicle
- receiver/main.cpp
- circle_demo.cpp
- shape_demo.cpp
- lib/swarm/swarm_hub.cpp
- ArucoConfig
- tools/swarm/swarm_hub.cpp
- lib/swarm/swarm_controller.cpp
- wingman.cpp
- Robot Swarm Pipeline — Architekturplan
- ArucoTracker
- tools/swarm/swarm_controller.cpp
- Screen
- RingBuffer
- dongle/main.cpp
- drag_drop_demo.cpp
- tools/swarm/swarm_hub_simulation.cpp
- uart_controller.py
- SwarmClient
- CMAES
- vision_controller.cpp
- DemoHud
- UARTProtocol
- ScreenManager
- tools/flash.py
- FisheyeUndistortPreprocessor
- ================== FIXED ===================
- Architecture
- main
- tools/swarm/swarm_dashboard.cpp
- BaslerPylonSource
- main
- circle_demo
- screen_manager.py
- main
- main
- vector
- lib/swarm/swarm_terminal.cpp
- EngineSound
- robot_uart.py
- calib_main.cpp
- CMA-ES ArUco Detector Calibration
- Wire Protocol: Three Independent Implementations
- StaticObjective
- circle_demo sleep_for(30ms) Poll Root Cause
- `swarm_hub`
- MSG_TELEMETRY Packet Format
- robots/flash.py
- Pololu 3pi+ 2040 Swarm Control
- Args
- PC Tools
- string
- thread
- runOptimisation
- MicroPython Feature-Flag + Isolated-Module Pattern
- Safety Watchdogs Live on the Robot
- CRC Error Handling (Silent Drop)
- Latency Budget (~4ms End-to-End)
- ODOMETRY_ENABLED Control-Strategy Toggle
- IOptimizer
- objective.h
- Robot Swarm
- generate_marker_stl.py
- TelemetryHistory
- RingBuffer
- generate_markers_pdf.py
- Fisheye Calibration (planned)
- SwarmClient.h
- DemoHud.h
- LineGraph
- TelemetryHistory
- ArUco Tracker Calibration
- .connect
- Quick Start
- Planned extensions
- tools/swarm/latency_plot.cpp
- Snapshot
- IObjective
- RobotRowLayout
- SlotOffset

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 70 edges
2. `ArucoConfig` - 55 edges
3. `SwarmClient` - 51 edges
4. `Vehicle` - 37 edges
5. `CMAES` - 36 edges
6. `main()` - 30 edges
7. `DemoHud` - 29 edges
8. `Screen` - 27 edges
9. `main()` - 24 edges
10. `main()` - 19 edges

## Surprising Connections (you probably didn't know these)
- `drawShapeWorld()` --calls--> `px`  [INFERRED]
  tools/vision/shape_demo.cpp → lib/ArucoTracker/aruco_tracker.h
- `main()` --calls--> `buildArucoDetector()`  [INFERRED]
  tools/vision/frame_inspector.cpp → lib/ArucoTracker/aruco_tracker.h
- `main()` --calls--> `bestX_`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/cmaes.h
- `runOptimisation()` --calls--> `encode()`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/param_space.h
- `runOptimisation()` --calls--> `decode()`  [INFERRED]
  tools/vision/calibration/calib_main.cpp → lib/Calibration/param_space.h

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **circle_demo Performance Root-Cause Investigation** — performance_circle_demo_sleep_poll_rootcause, performance_governor_epp_red_herring, performance_half_res_sweep, performance_loop_fps_vs_cam_fps [EXTRACTED 0.90]
- **Vision-Based Swarm Controllers** — readme_vision_controller, readme_wingman, readme_circle_demo, readme_shape_demo [EXTRACTED 0.90]
- **Wire Protocol Three-Language Implementation** — claudemd_wire_protocol_three_implementations, readme_wire_protocol_frame_format, src_robots_readme_uartprotocol, docs_architecture_crc_error_handling [INFERRED 0.85]

## Communities (85 total, 7 thin omitted)

### Community 0 - "lib/swarm/swarm_dashboard.cpp"
Cohesion: 0.23
Nodes (11): bipolarMeter(), computeLayout(), drawUI(), main(), peakMeter(), RobotRowLayout, meterWidth, sparkWidth (+3 more)

### Community 1 - "lib/swarm/latency_plot.cpp"
Cohesion: 0.47
Nodes (4): deque, drawPlot(), main(), onPong()

### Community 2 - "Vehicle"
Cohesion: 0.07
Nodes (44): Color, Font, RenderWindow, alpha(), pair, string, draw_boost(), draw_drift_badge() (+36 more)

### Community 3 - "receiver/main.cpp"
Cohesion: 0.08
Nodes (33): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+25 more)

### Community 4 - "circle_demo.cpp"
Cohesion: 0.06
Nodes (39): assignNearestSlots(), CalibState, done, pixPts, CircleState, centre, centreSet, minGapMm (+31 more)

### Community 5 - "shape_demo.cpp"
Cohesion: 0.09
Nodes (38): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+30 more)

### Community 6 - "lib/swarm/swarm_hub.cpp"
Cohesion: 0.06
Nodes (55): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, any_client_active(), broadcast_to_clients(), build_ping(), client_accept() (+47 more)

### Community 7 - "ArucoConfig"
Cohesion: 0.05
Nodes (39): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+31 more)

### Community 8 - "tools/swarm/swarm_hub.cpp"
Cohesion: 0.10
Nodes (30): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector, any_client_active(), broadcast_to_clients(), build_ping() (+22 more)

### Community 9 - "lib/swarm/swarm_controller.cpp"
Cohesion: 0.11
Nodes (28): advanceTest(), applyTestStep(), drawUI(), handleInput(), keyDown(), main(), normalMode(), pollWASD() (+20 more)

### Community 10 - "wingman.cpp"
Cohesion: 0.10
Nodes (33): CGEventRef, CGEventTapProxy, CGEventType, RobotPose, id, px, py, x (+25 more)

### Community 11 - "Robot Swarm Pipeline — Architekturplan"
Cohesion: 0.06
Nodes (34): 1. Roboter-Registrierung, 2. Steuerkanal (PC → Roboter), 3. Telemetrie-Rückkanal (Roboter → PC), 4. Protokoll-Übersicht, 5. Fehlerbehandlung, 6. Controller-PC Software, 7. Hardware-Checkliste pro Roboter, 8. Implementierungsreihenfolge (+26 more)

### Community 12 - "ArucoTracker"
Cohesion: 0.06
Nodes (27): DetectionResult, ArucoTracker, captureRunning_, captureThread_, cfg_, clahe_, debug_, detectionRunning_ (+19 more)

### Community 13 - "tools/swarm/swarm_controller.cpp"
Cohesion: 0.10
Nodes (31): advanceTest(), applyTestStep(), KeyHandle, time_point, vector, drawUI(), handleInput(), keyDown() (+23 more)

### Community 14 - "Screen"
Cohesion: 0.07
Nodes (15): Repräsentiert einen einzelnen Debug-Screen. Verwendung: screen = Screen("MY…, Fügt eine neue Nachricht zum Log hinzu., Löscht alle Log-Einträge., Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück., Entfernt eine einzelne Primitive per Handle. (+7 more)

### Community 15 - "RingBuffer"
Cohesion: 0.25
Nodes (5): RingBuffer, buf_, count_, head_, N

### Community 16 - "dongle/main.cpp"
Cohesion: 0.10
Nodes (29): anyRobotActive(), enqueueSend(), ensurePeer(), ledOff(), ledOn(), loop(), onReceive(), PingTracker (+21 more)

### Community 17 - "drag_drop_demo.cpp"
Cohesion: 0.10
Nodes (25): fromFile, Point, AvoidState, arc, arcDx, arcDy, minDist, buildAvoidance() (+17 more)

### Community 18 - "tools/swarm/swarm_hub_simulation.cpp"
Cohesion: 0.14
Nodes (27): broadcast(), buildFrame(), client_accept(), client_close(), ClientConn, active, fd, rxBuf (+19 more)

### Community 19 - "uart_controller.py"
Cohesion: 0.09
Nodes (20): _battery_byte(), _draw_scaled_text(), on_packet(), PI, process_pending(), Render text at (x, y) with pixel-doubled scaling using a temp framebuf., Render text at (x, y) with pixel-doubled scaling using a temp framebuf., Measure wheel speeds and run PID; called at PID_INTERVAL_MS cadence. (+12 more)

### Community 20 - "SwarmClient"
Cohesion: 0.11
Nodes (12): DebugEntry, mutex, RobotState, SwarmClient, m_debugLog, m_fd, m_robots, m_rxBuf (+4 more)

### Community 21 - "CMAES"
Cohesion: 0.08
Nodes (23): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+15 more)

### Community 22 - "vision_controller.cpp"
Cohesion: 0.12
Nodes (20): CalibState, done, pixPts, clampf(), KeyHandle, Point2f, vector, keyDown() (+12 more)

### Community 23 - "DemoHud"
Cohesion: 0.11
Nodes (18): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+10 more)

### Community 24 - "UARTProtocol"
Cohesion: 0.15
Nodes (9): Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED). :param left: Geschwindigkeit…, Sendet die Batteriespannung an den ESP32 (MSG_METRICS). :param battery_byte:…, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG). Der ESP32 stellt die…, Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., Verwaltet das Senden und Empfangen von strukturierten UART-Paketen. MSG_SPEED…, UARTProtocol (+1 more)

### Community 25 - "ScreenManager"
Cohesion: 0.12
Nodes (11): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display. Knopfdruck…, :param display: robot.Display() Instanz :param button: Button-Instanz mit…, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+3 more)

### Community 26 - "tools/flash.py"
Cohesion: 0.19
Nodes (18): connected_robot_ports(), deploy_cmd(), deploy_problems(), eject_micropython_volume(), flash(), main(), onboard_verify_problems(), Returns human-readable problems with a deploy attempt; empty = verified OK. (+10 more)

### Community 27 - "FisheyeUndistortPreprocessor"
Cohesion: 0.12
Nodes (11): CLAHE, CLAHEPreprocessor, clahe_, FisheyeUndistortPreprocessor, map1_, map2_, Mat, IPreprocessor (+3 more)

### Community 28 - "================== FIXED ==================="
Cohesion: 0.12
Nodes (16): Bug: Latency Erledigt (2026-06-18), Erledigt (2026-06-10), Erledigt (2026-06-12), Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06), ================== FIXED ===================, Genera aruco tracker issue: Erledigt (2026-07-08), Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12), questions to answer: (+8 more)

### Community 29 - "Architecture"
Cohesion: 0.12
Nodes (14): Architecture, Commands, Firmware (PlatformIO — `src/dongle`, `src/receiver`), MicroPython robot firmware: feature-flag + isolated-module pattern, PC tools (`tools/`, plain Makefile), Project Overview, Repo layout, Robot firmware (`src/robots/`, MicroPython) (+6 more)

### Community 30 - "main"
Cohesion: 0.35
Nodes (8): string, vector, Mode, main(), main(), Mat, drawTelHud(), drawTelHud()

### Community 31 - "tools/swarm/swarm_dashboard.cpp"
Cohesion: 0.29
Nodes (13): Buffer, appendf(), bipolarMeter(), brailleGraph(), DebugEntry, string, vector, drawUI() (+5 more)

### Community 32 - "BaslerPylonSource"
Cohesion: 0.13
Nodes (10): CBaslerUniversalInstantCamera, CImageFormatConverter, BaslerPylonSource, camera_, converter_, height_, pylonRuntime_, width_ (+2 more)

### Community 33 - "main"
Cohesion: 0.21
Nodes (14): bestX_, decode(), encode(), fromNorm(), string, vector, ParamSpec, hi (+6 more)

### Community 34 - "circle_demo"
Cohesion: 0.13
Nodes (14): 2026-06-12 — back to `powersave` (after the `performance` test), 2026-06-12 — Baseline (revised: split by tracking state), 2026-06-12 — fix: `half_res_sweep: true` (was `false` in config), 2026-06-12 — governor `powersave` → `performance`, 2026-06-12 — `performance` governor + EPP after BIOS fan-curve change, 2026-06-12 — re-baseline (`powersave` / EPP `balance_power`, confirmed default), 2026-06-12 — ROOT CAUSE FOUND & FIXED: `sleep_for(30ms)` poll in main loop, circle_demo (+6 more)

### Community 35 - "screen_manager.py"
Cohesion: 0.14
Nodes (8): BarGraph, _draw_circle(), _draw_line(), Gauge, Balkendiagramm für einen einzelnen Wert. Verwendung: bar =…, Halbkreis-Gauge für einen einzelnen Wert. Verwendung: gauge =…, Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-…, Bresenham-Kreis direkt auf das Display zeichnen.

### Community 36 - "main"
Cohesion: 0.23
Nodes (10): CaptureThread, _detect_with_roi(), _find_gopro_index(), main(), ArucoDetector, ndarray, Find the AVFoundation index for the connected GoPro camera. Two-step lookup: 1.…, Continuously grabs frames via AVFoundation; always exposes the latest one. (+2 more)

### Community 37 - "main"
Cohesion: 0.23
Nodes (12): Mat, Point2f, vector, drawDetections(), FrameDetections, corners, ids, isEnd() (+4 more)

### Community 38 - "vector"
Cohesion: 0.24
Nodes (5): KalmanFilter, Point2f, Scalar, vector, MarkerState

### Community 39 - "lib/swarm/swarm_terminal.cpp"
Cohesion: 0.23
Nodes (6): drawUI(), main(), statusParts(), drawUI(), main(), statusParts()

### Community 40 - "EngineSound"
Cohesion: 0.25
Nodes (4): EngineSound, Idle hum that rises in pitch/volume with wheel speed, with a turn-wobble     lay, Recompute and apply the tone. dt in seconds, same dt as the caller's own loop., Silence the buzzer (idle hum off).

### Community 41 - "robot_uart.py"
Cohesion: 0.18
Nodes (8): build_packet(), _crc8(), _crc8_buf(), Packet, CRC-8 via Lookup-Table (Polynom 0x07)., CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig., Verpackt Nutzdaten in einen vollstaendigen Frame. Frame:…, Repraesentiert ein empfangenes Paket.

### Community 42 - "calib_main.cpp"
Cohesion: 0.27
Nodes (8): Mat, captureInteractive(), Mat, string, vector, loadFrameCache(), parseArgs(), saveFrameCache()

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

### Community 49 - "robots/flash.py"
Cohesion: 0.70
Nodes (4): connected_robot_ports(), eject_micropython_volume(), flash(), main()

### Community 50 - "Pololu 3pi+ 2040 Swarm Control"
Cohesion: 0.18
Nodes (10): Batch flashing, Data Pipeline, Debug Display Types, Dependencies, Display, Files, Message Types, Packet Format (+2 more)

### Community 51 - "Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 52 - "PC Tools"
Cohesion: 0.20
Nodes (11): `calibrate`, evdev WASD Keyboard Input, `frame_inspector`, `latency_plot`, PC Tools, `shape_demo`, `swarm_controller`, `swarm_terminal` (+3 more)

### Community 53 - "string"
Cohesion: 0.22
Nodes (5): atomic, condition_variable, defaultConfigPath, Size, string

### Community 54 - "thread"
Cohesion: 0.20
Nodes (5): ICameraSource, open, read, size, thread

### Community 56 - "MicroPython Feature-Flag + Isolated-Module Pattern"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

### Community 61 - "IOptimizer"
Cohesion: 0.20
Nodes (8): IOptimizer, ask, bestFit, converged, generation, lambda, setMean, tell

### Community 62 - "objective.h"
Cohesion: 0.24
Nodes (9): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+1 more)

### Community 63 - "Robot Swarm"
Cohesion: 0.20
Nodes (9): Architecture, Camera Setup (Basler ace2 GigE), Firmware (all platforms), PC Tools — macOS (Apple Silicon), PC Tools — Ubuntu 22.04 / 24.04 (x86\_64), Prerequisites, Project Structure, Robot Swarm (+1 more)

### Community 64 - "generate_marker_stl.py"
Cohesion: 0.31
Nodes (9): box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Return a 6×6 list-of-lists (1 = black, 0 = white) via OpenCV., 12 outward-facing triangles for an axis-aligned box., _tri() (+1 more)

### Community 65 - "TelemetryHistory"
Cohesion: 0.20
Nodes (7): main(), RobotState, unordered_map, vector, TelemetryHistory, kWindow, perRobot_

### Community 66 - "RingBuffer"
Cohesion: 0.29
Nodes (4): RingBuffer, buf_, count_, head_

### Community 67 - "generate_markers_pdf.py"
Cohesion: 0.36
Nodes (8): generate_pdf(), main(), _marker_image(), _ndarray_to_png_bytes(), ndarray, generate_markers.py — print ArUco or AprilTag marker sheets as a PDF. Usage: #…, Return (cv2_dict_id, max_n, label_prefix) from CLI arguments., _resolve_dict()

### Community 69 - "SwarmClient.h"
Cohesion: 0.36
Nodes (4): array, mutex, unordered_map, scBatteryVolts()

### Community 70 - "DemoHud.h"
Cohesion: 0.32
Nodes (5): Mat, Scalar, drawPanel(), main(), tempColor()

### Community 71 - "LineGraph"
Cohesion: 0.29
Nodes (4): LineGraph, Scrollender Liniengraph für kontinuierliche Werte. Verwendung: graph =…, Fügt einen neuen Messwert hinzu., Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest.

### Community 72 - "TelemetryHistory"
Cohesion: 0.29
Nodes (4): TelemetryHistory, kWindow, perRobot_, PerRobot

### Community 73 - "ArUco Tracker Calibration"
Cohesion: 0.29
Nodes (6): All flags, ArUco Tracker Calibration, File structure, How it works, Parameters being optimised, Quick start

### Community 75 - "Quick Start"
Cohesion: 0.33
Nodes (6): 1. Flash the dongle, 2. Flash each robot, 3. Build PC tools, 4. Find the dongle serial port, 5. Run the hub, Quick Start

### Community 76 - "Planned extensions"
Cohesion: 0.33
Nodes (6): Combined static + motion objective, Fisheye calibration (`objective_fisheye.h`), GP with ARD kernel (`gp_ard.h`), Motion objective (`objective_motion.h`), Parameter range narrowing, Planned extensions

### Community 77 - "tools/swarm/latency_plot.cpp"
Cohesion: 0.60
Nodes (3): drawPlot(), main(), onPong()

### Community 78 - "Snapshot"
Cohesion: 0.40
Nodes (5): pair, RobotState, time_point, Snapshot, robots

### Community 79 - "IObjective"
Cohesion: 0.50
Nodes (3): IObjective, evaluate, name

### Community 80 - "RobotRowLayout"
Cohesion: 0.50
Nodes (4): computeLayout(), RobotRowLayout, meterWidth, sparkWidth

### Community 81 - "SlotOffset"
Cohesion: 0.67
Nodes (3): SlotOffset, dx, dy

## Knowledge Gaps
- **423 isolated node(s):** `open`, `id`, `x`, `y`, `yaw` (+418 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **7 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoTracker` connect `ArucoTracker` to `circle_demo.cpp`, `SwarmClient.h`, `vector`, `ArucoConfig`, `shape_demo.cpp`, `wingman.cpp`, `drag_drop_demo.cpp`, `string`, `thread`, `vision_controller.cpp`, `FisheyeUndistortPreprocessor`?**
  _High betweenness centrality (0.070) - this node is a cross-community bridge._
- **Why does `Vehicle` connect `Vehicle` to `lib/swarm/latency_plot.cpp`?**
  _High betweenness centrality (0.050) - this node is a cross-community bridge._
- **Why does `ArucoConfig` connect `ArucoConfig` to `BaslerPylonSource`, `main`, `calib_main.cpp`, `ArucoTracker`, `StaticObjective`, `drag_drop_demo.cpp`, `string`, `runOptimisation`, `objective.h`?**
  _High betweenness centrality (0.048) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 6 INFERRED edges - model-reasoned connections that need verification._
- **What connects `open`, `id`, `x` to the rest of the system?**
  _423 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Vehicle` be split into smaller, more focused modules?**
  _Cohesion score 0.07446808510638298 - nodes in this community are weakly interconnected._
- **Should `receiver/main.cpp` be split into smaller, more focused modules?**
  _Cohesion score 0.080338266384778 - nodes in this community are weakly interconnected._