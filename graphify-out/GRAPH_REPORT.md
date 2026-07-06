# Graph Report - .  (2026-07-03)

## Corpus Check
- 52 files · ~67,530 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1011 nodes · 1729 edges · 69 communities (55 shown, 14 thin omitted)
- Extraction: 91% EXTRACTED · 9% INFERRED · 0% AMBIGUOUS · INFERRED: 148 edges (avg confidence: 0.78)
- Token cost: 62,698 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Terminal Dashboard UI|Terminal Dashboard UI]]
- [[_COMMUNITY_Latency Plot & Game HUD|Latency Plot & Game HUD]]
- [[_COMMUNITY_CMA-ES Optimizer Core|CMA-ES Optimizer Core]]
- [[_COMMUNITY_Wire Protocol & CRC Framing|Wire Protocol & CRC Framing]]
- [[_COMMUNITY_Shape Demo Controller|Shape Demo Controller]]
- [[_COMMUNITY_Wingman V-Formation & macOS Input|Wingman V-Formation & macOS Input]]
- [[_COMMUNITY_swarm_hub & evdev Keyboard|swarm_hub & evdev Keyboard]]
- [[_COMMUNITY_ArUco Configuration|ArUco Configuration]]
- [[_COMMUNITY_Circle Demo Controller|Circle Demo Controller]]
- [[_COMMUNITY_Swarm Controller Tool|Swarm Controller Tool]]
- [[_COMMUNITY_Receiver Firmware & Ping Tracker|Receiver Firmware & Ping Tracker]]
- [[_COMMUNITY_ArUco Tracker Core|ArUco Tracker Core]]
- [[_COMMUNITY_Swarm Hub Simulation|Swarm Hub Simulation]]
- [[_COMMUNITY_Basler Pylon Camera Source|Basler Pylon Camera Source]]
- [[_COMMUNITY_Drag & Drop Demo|Drag & Drop Demo]]
- [[_COMMUNITY_Telemetry Ring Buffer|Telemetry Ring Buffer]]
- [[_COMMUNITY_Demo HUD Rendering|Demo HUD Rendering]]
- [[_COMMUNITY_Vision Controller Tool|Vision Controller Tool]]
- [[_COMMUNITY_ScreenManager (Robot Display)|ScreenManager (Robot Display)]]
- [[_COMMUNITY_Robot UART Controller & PID|Robot UART Controller & PID]]
- [[_COMMUNITY_Robot Debug Screen|Robot Debug Screen]]
- [[_COMMUNITY_Parameter Space Encoding|Parameter Space Encoding]]
- [[_COMMUNITY_GoPro ArUco Demo|GoPro ArUco Demo]]
- [[_COMMUNITY_Calibration Objective Interface|Calibration Objective Interface]]
- [[_COMMUNITY_Telemetry HUD|Telemetry HUD]]
- [[_COMMUNITY_UART Protocol & Heartbeat|UART Protocol & Heartbeat]]
- [[_COMMUNITY_Kalman Filter Tracking|Kalman Filter Tracking]]
- [[_COMMUNITY_Robot UART Packet & CRC|Robot UART Packet & CRC]]
- [[_COMMUNITY_CLAHE Preprocessor|CLAHE Preprocessor]]
- [[_COMMUNITY_Fisheye Undistort Preprocessor|Fisheye Undistort Preprocessor]]
- [[_COMMUNITY_Static Objective Scoring|Static Objective Scoring]]
- [[_COMMUNITY_Calibration CLI Args|Calibration CLI Args]]
- [[_COMMUNITY_Tracker Homography & SwarmClient|Tracker Homography & SwarmClient]]
- [[_COMMUNITY_Robot Display Gauge|Robot Display Gauge]]
- [[_COMMUNITY_Marker STL Generator|Marker STL Generator]]
- [[_COMMUNITY_Demo HUD Preview & Marker Eval|Demo HUD Preview & Marker Eval]]
- [[_COMMUNITY_Camera Source Interface|Camera Source Interface]]
- [[_COMMUNITY_CMA-ES Calibration Main|CMA-ES Calibration Main]]
- [[_COMMUNITY_Robot Screen Primitives|Robot Screen Primitives]]
- [[_COMMUNITY_Marker PDF Generator|Marker PDF Generator]]
- [[_COMMUNITY_Engine Sound Feature|Engine Sound Feature]]
- [[_COMMUNITY_Robot UART Send Methods|Robot UART Send Methods]]
- [[_COMMUNITY_Robot LineGraph Widget|Robot LineGraph Widget]]
- [[_COMMUNITY_Vision & Calibration Rationale|Vision & Calibration Rationale]]
- [[_COMMUNITY_Wire Protocol & Firmware Rationale|Wire Protocol & Firmware Rationale]]
- [[_COMMUNITY_Rate Estimator|Rate Estimator]]
- [[_COMMUNITY_circle_demo Performance Investigation|circle_demo Performance Investigation]]
- [[_COMMUNITY_swarm_hub Serial-Ownership Rationale|swarm_hub Serial-Ownership Rationale]]
- [[_COMMUNITY_Robot Registration & Telemetry Rationale|Robot Registration & Telemetry Rationale]]
- [[_COMMUNITY_Robot Flashing Tool|Robot Flashing Tool]]
- [[_COMMUNITY_Robot BarGraph Widget|Robot BarGraph Widget]]
- [[_COMMUNITY_ArucoTracker Build|ArucoTracker Build]]
- [[_COMMUNITY_Input & Vision Tools Rationale|Input & Vision Tools Rationale]]
- [[_COMMUNITY_Robot Screen Clear|Robot Screen Clear]]
- [[_COMMUNITY_Robot Screen Render|Robot Screen Render]]
- [[_COMMUNITY_Target Struct|Target Struct]]
- [[_COMMUNITY_MicroPython Feature-Flag Rationale|MicroPython Feature-Flag Rationale]]
- [[_COMMUNITY_Robot Safety Watchdog Rationale|Robot Safety Watchdog Rationale]]
- [[_COMMUNITY_CRC Error Handling & Tests|CRC Error Handling & Tests]]
- [[_COMMUNITY_Latency Budget & MSG_SWARM|Latency Budget & MSG_SWARM]]
- [[_COMMUNITY_ODOMETRY Toggle Rationale|ODOMETRY Toggle Rationale]]
- [[_COMMUNITY_Dongle Watchdog Broadcast|Dongle Watchdog Broadcast]]
- [[_COMMUNITY_frame_inspector Tool|frame_inspector Tool]]
- [[_COMMUNITY_latency_plot Tool Node|latency_plot Tool Node]]
- [[_COMMUNITY_shape_demo Path Drawing|shape_demo Path Drawing]]
- [[_COMMUNITY_swarm_terminal UI Node|swarm_terminal UI Node]]
- [[_COMMUNITY_Fisheye Calibration (planned)|Fisheye Calibration (planned)]]

## God Nodes (most connected - your core abstractions)
1. `ArucoTracker` - 69 edges
2. `ArucoConfig` - 53 edges
3. `SwarmClient` - 46 edges
4. `Vehicle` - 37 edges
5. `CMAES` - 36 edges
6. `DemoHud` - 29 edges
7. `main()` - 29 edges
8. `Screen` - 25 edges
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
- **Vision-Based Swarm Controllers** — readme_vision_controller, readme_wingman, readme_circle_demo, readme_shape_demo [EXTRACTED 0.90]
- **Wire Protocol Three-Language Implementation** — claudemd_wire_protocol_three_implementations, readme_wire_protocol_frame_format, src_robots_readme_uartprotocol, docs_architecture_crc_error_handling [INFERRED 0.85]
- **circle_demo Performance Root-Cause Investigation** — performance_circle_demo_sleep_poll_rootcause, performance_governor_epp_red_herring, performance_half_res_sweep, performance_loop_fps_vs_cam_fps [EXTRACTED 0.90]

## Communities (69 total, 14 thin omitted)

### Community 0 - "Terminal Dashboard UI"
Cohesion: 0.06
Nodes (29): Buffer, DebugEntry, bipolarMeter(), computeLayout(), string, drawUI(), main(), peakMeter() (+21 more)

### Community 1 - "Latency Plot & Game HUD"
Cohesion: 0.06
Nodes (48): Color, deque, Font, drawPlot(), main(), onPong(), pair, RenderWindow (+40 more)

### Community 2 - "CMA-ES Optimizer Core"
Cohesion: 0.06
Nodes (33): CMAES, B_, C_, c1_, cc_, chin_, cmu_, cs_ (+25 more)

### Community 3 - "Wire Protocol & CRC Framing"
Cohesion: 0.09
Nodes (32): registerAllFields(), registerField(), sendInt8(), sendPacket(), sendString(), updateAll(), buildFrame(), crc8() (+24 more)

### Community 4 - "Shape Demo Controller"
Cohesion: 0.09
Nodes (38): Tool, buildWaypoints(), CalibState, done, pixPts, clampf(), Mat, Point2f (+30 more)

### Community 5 - "Wingman V-Formation & macOS Input"
Cohesion: 0.09
Nodes (36): CGEventRef, CGEventTapProxy, CGEventType, RobotPose, id, px, py, x (+28 more)

### Community 6 - "swarm_hub & evdev Keyboard"
Cohesion: 0.10
Nodes (30): EvdevKeyboard, fds_, kBitsPerLong, kKeyLongs, vector, any_client_active(), broadcast_to_clients(), build_ping() (+22 more)

### Community 7 - "ArUco Configuration"
Cohesion: 0.05
Nodes (38): ArucoConfig, baslerIp, baslerSerial, cellMargin, claheClip, claheTile, cornerMaxIter, cornerWin (+30 more)

### Community 8 - "Circle Demo Controller"
Cohesion: 0.08
Nodes (32): CalibState, done, pixPts, CircleState, centre, centreSet, minGapMm, orbitSpeed (+24 more)

### Community 9 - "Swarm Controller Tool"
Cohesion: 0.10
Nodes (31): advanceTest(), applyTestStep(), KeyHandle, time_point, vector, drawUI(), handleInput(), keyDown() (+23 more)

### Community 10 - "Receiver Firmware & Ping Tracker"
Cohesion: 0.09
Nodes (28): anyRobotActive(), enqueueSend(), ensurePeer(), ledOff(), ledOn(), loop(), PingTracker, pending (+20 more)

### Community 11 - "ArUco Tracker Core"
Cohesion: 0.07
Nodes (24): DetectionResult, ArucoTracker, captureRunning_, captureThread_, cfg_, clahe_, debug_, detectionRunning_ (+16 more)

### Community 12 - "Swarm Hub Simulation"
Cohesion: 0.14
Nodes (27): broadcast(), buildFrame(), client_accept(), client_close(), ClientConn, active, fd, rxBuf (+19 more)

### Community 13 - "Basler Pylon Camera Source"
Cohesion: 0.09
Nodes (21): CBaslerUniversalInstantCamera, CImageFormatConverter, BaslerPylonSource, camera_, converter_, height_, width_, Mat (+13 more)

### Community 14 - "Drag & Drop Demo"
Cohesion: 0.10
Nodes (24): Point, AvoidState, arc, arcDx, arcDy, minDist, buildAvoidance(), CalibState (+16 more)

### Community 15 - "Telemetry Ring Buffer"
Cohesion: 0.10
Nodes (14): array, unordered_map, RobotState, unordered_map, vector, RingBuffer, buf_, count_ (+6 more)

### Community 16 - "Demo HUD Rendering"
Cohesion: 0.12
Nodes (18): DemoHud, COL_BAD, COL_GAP, COL_OK, COL_TEXT, COL_WARN, FONT, FONT_SCALE (+10 more)

### Community 17 - "Vision Controller Tool"
Cohesion: 0.15
Nodes (16): CalibState, done, pixPts, clampf(), KeyHandle, Point2f, vector, keyDown() (+8 more)

### Community 18 - "ScreenManager (Robot Display)"
Cohesion: 0.12
Nodes (10): Verwaltet mehrere Screens und zeichnet den aktiven auf das Display.     Knopfdru, :param display: robot.Display() Instanz         :param button:  Button-Instanz m, Registriert einen Screen. Erster registrierter Screen ist aktiv., Setzt den aktiven Screen direkt per Index., Schaltet zum nächsten Screen weiter (wraps around)., Gibt den aktuell aktiven Screen zurück., Markiert das Display als neu zu zeichnen (z.B. nach screen.log())., Muss regelmäßig in der Hauptschleife aufgerufen werden. (+2 more)

### Community 19 - "Robot UART Controller & PID"
Cohesion: 0.13
Nodes (13): _battery_byte(), _draw_scaled_text(), on_packet(), PI, process_pending(), Render text at (x, y) with pixel-doubled scaling using a temp framebuf., Measure wheel speeds and run PID; called at PID_INTERVAL_MS cadence., Store the latest speed packet; applied on next loop iteration. (+5 more)

### Community 20 - "Robot Debug Screen"
Cohesion: 0.12
Nodes (9): Repräsentiert einen einzelnen Debug-Screen.      Verwendung:         screen = Sc, Fügt eine neue Nachricht zum Log hinzu., Entfernt eine einzelne Primitive per Handle., Registriert eine Metrik an fester Position (überlappen immer).          :param n, Entfernt eine Metrik., Setzt ein einzelnes Diagramm (ersetzt alle vorherigen).         height: Pixel-Ho, Fuegt ein weiteres Diagramm unterhalb der vorherigen hinzu.         height: Pixe, Entfernt alle Diagramme, kehrt zu Vollbild-Log zurueck. (+1 more)

### Community 21 - "Parameter Space Encoding"
Cohesion: 0.21
Nodes (14): bestX_, decode(), encode(), fromNorm(), string, vector, ParamSpec, hi (+6 more)

### Community 22 - "GoPro ArUco Demo"
Cohesion: 0.23
Nodes (10): CaptureThread, _detect_with_roi(), _find_gopro_index(), main(), ArucoDetector, ndarray, Find the AVFoundation index for the connected GoPro camera.      Two-step lookup, Continuously grabs frames via AVFoundation; always exposes the latest one. (+2 more)

### Community 23 - "Calibration Objective Interface"
Cohesion: 0.16
Nodes (12): detectFrame(), DetResult, corners, ids, ArucoDetector, Mat, Point2f, vector (+4 more)

### Community 24 - "Telemetry HUD"
Cohesion: 0.38
Nodes (7): string, vector, Mode, main(), Mat, drawTelHud(), drawTelHud()

### Community 25 - "UART Protocol & Heartbeat"
Cohesion: 0.24
Nodes (5): Muss regelmaessig in der Hauptschleife aufgerufen werden., Ueberprueft den Heartbeat-Timeout und sendet ggf. einen Ping., Verwaltet das Senden und Empfangen von strukturierten UART-Paketen.      MSG_SPE, UARTProtocol, UART

### Community 26 - "Kalman Filter Tracking"
Cohesion: 0.24
Nodes (5): KalmanFilter, Point2f, Scalar, vector, MarkerState

### Community 27 - "Robot UART Packet & CRC"
Cohesion: 0.18
Nodes (8): build_packet(), _crc8(), _crc8_buf(), Packet, CRC-8 via Lookup-Table (Polynom 0x07)., CRC-8 direkt auf bytearray mit Offset – keine Kopie noetig., Verpackt Nutzdaten in einen vollstaendigen Frame.     Frame: [0xAA][0x55][type][, Repraesentiert ein empfangenes Paket.

### Community 28 - "CLAHE Preprocessor"
Cohesion: 0.20
Nodes (7): CLAHE, CLAHEPreprocessor, clahe_, IPreprocessor, process, Ptr, unique_ptr

### Community 29 - "Fisheye Undistort Preprocessor"
Cohesion: 0.20
Nodes (5): FisheyeUndistortPreprocessor, map1_, map2_, Mat, Size

### Community 30 - "Static Objective Scoring"
Cohesion: 0.24
Nodes (8): Mat, string, vector, StaticObjective, detectedIds_, frames_, ArucoDetector, makeDetector()

### Community 31 - "Calibration CLI Args"
Cohesion: 0.18
Nodes (11): Args, baslerIp, baslerSerial, cacheDir, config, eval, idsMax, maxIter (+3 more)

### Community 32 - "Tracker Homography & SwarmClient"
Cohesion: 0.24
Nodes (6): atomic, condition_variable, mutex, string, scBatteryVolts(), thread

### Community 33 - "Robot Display Gauge"
Cohesion: 0.22
Nodes (6): _draw_circle(), _draw_line(), Gauge, Halbkreis-Gauge für einen einzelnen Wert.      Verwendung:         gauge = Gauge, Linie auf das Display zeichnen – nutzt framebuf C-Implementierung statt Python-B, Bresenham-Kreis direkt auf das Display zeichnen.

### Community 34 - "Marker STL Generator"
Cohesion: 0.31
Nodes (9): box_triangles(), get_marker_grid(), main(), print_marker(), Write a list of (normal, v1, v2, v3) tuples as a binary STL file., Return a 6×6 list-of-lists (1 = black, 0 = white) via OpenCV., 12 outward-facing triangles for an axis-aligned box., _tri() (+1 more)

### Community 35 - "Demo HUD Preview & Marker Eval"
Cohesion: 0.28
Nodes (6): fromFile, Mat, Scalar, drawPanel(), main(), tempColor()

### Community 36 - "Camera Source Interface"
Cohesion: 0.22
Nodes (4): ICameraSource, open, read, size

### Community 37 - "CMA-ES Calibration Main"
Cohesion: 0.36
Nodes (7): captureInteractive(), Mat, string, vector, loadFrameCache(), parseArgs(), saveFrameCache()

### Community 38 - "Robot Screen Primitives"
Cohesion: 0.22
Nodes (4): Persistente Linie. Gibt Handle zurück., Persistenter Kreis. Gibt Handle zurück., Persistentes Rechteck. Gibt Handle zurück., Persistentes Pixel. Gibt Handle zurück.

### Community 39 - "Marker PDF Generator"
Cohesion: 0.36
Nodes (8): generate_pdf(), main(), _marker_image(), _ndarray_to_png_bytes(), ndarray, generate_markers.py — print ArUco or AprilTag marker sheets as a PDF.  Usage:, Return (cv2_dict_id, max_n, label_prefix) from CLI arguments., _resolve_dict()

### Community 40 - "Engine Sound Feature"
Cohesion: 0.25
Nodes (4): EngineSound, Idle hum that rises in pitch/volume with wheel speed, with a turn-wobble     lay, Recompute and apply the tone. dt in seconds, same dt as the caller's own loop., Silence the buzzer (idle hum off).

### Community 41 - "Robot UART Send Methods"
Cohesion: 0.25
Nodes (4): Sendet ein Paket ueber UART., Sendet ein kombiniertes SPEED Paket (MSG_SPEED).          :param left:  Geschwin, Sendet die Batteriespannung an den ESP32 (MSG_METRICS).          :param battery_, Sendet eine Debug-Log-Zeile an den PC (MSG_DEBUG).          Der ESP32 stellt die

### Community 42 - "Robot LineGraph Widget"
Cohesion: 0.29
Nodes (4): LineGraph, Scrollender Liniengraph für kontinuierliche Werte.      Verwendung:         grap, Fügt einen neuen Messwert hinzu., Gibt (min, max) zurück – dynamisch bei auto_scale, sonst fest.

### Community 43 - "Vision & Calibration Rationale"
Cohesion: 0.38
Nodes (7): Vision Pipeline as Separate Concern, marker_eval Detection Benchmark, CMA-ES ArUco Detector Calibration, GP-ARD Bayesian Optimizer (planned), IOptimizer ask/tell Interface, Motion Objective Scoring, Static Objective Scoring

### Community 44 - "Wire Protocol & Firmware Rationale"
Cohesion: 0.29
Nodes (7): Wire Protocol: Three Independent Implementations, Wire Protocol Frame Format, flash.py Batch Flashing, ScreenManager Display Manager, uart_controller.py Main Loop, UARTProtocol Library (robot_uart.py), Remote Robot Shutdown (scrapped)

### Community 45 - "Rate Estimator"
Cohesion: 0.33
Nodes (7): time_point, RateEstimator, baseAngle, baseTime, init, rate, updateRate()

### Community 46 - "circle_demo Performance Investigation"
Cohesion: 0.33
Nodes (6): circle_demo sleep_for(30ms) Poll Root Cause, Governor/EPP/Throttling Red Herring, half_res_sweep Config Optimization, loop_fps vs cam_fps Gap, circle_demo Orbit Controller, Generalized DEBUG/HUD Util (planned)

### Community 47 - "swarm_hub Serial-Ownership Rationale"
Cohesion: 0.40
Nodes (5): Centralized Round-Robin Pinging in swarm_hub, swarm_hub Owns the Serial Port, Pylon rpath Link Fix, swarm_hub Serial-Socket Bridge, Per-Client Ping Clobbering Bug (fixed)

### Community 48 - "Robot Registration & Telemetry Rationale"
Cohesion: 0.40
Nodes (5): Robot Registration and Addressing, MSG_TELEMETRY Packet Format, Robot Registration via Announce, TDMA Telemetry Return Channel, MSG_METRICS Battery Channel (0x03)

### Community 49 - "Robot Flashing Tool"
Cohesion: 0.70
Nodes (4): connected_robot_ports(), eject_micropython_volume(), flash(), main()

### Community 52 - "Input & Vision Tools Rationale"
Cohesion: 0.50
Nodes (4): evdev WASD Keyboard Input, vision_controller Tool, wingman V-Formation Controller, Unified PID Unit (planned)

### Community 55 - "Target Struct"
Cohesion: 0.50
Nodes (4): Target, set, x, y

### Community 56 - "MicroPython Feature-Flag Rationale"
Cohesion: 0.67
Nodes (3): MicroPython Feature-Flag + Isolated-Module Pattern, Buzzer Sound (silent-by-design protocol), MSG_DEBUG Robot-to-PC Channel (0x02)

## Knowledge Gaps
- **291 isolated node(s):** `open`, `id`, `x`, `y`, `yaw` (+286 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **14 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ArucoConfig` connect `ArUco Configuration` to `Tracker Homography & SwarmClient`, `CMA-ES Optimizer Core`, `Demo HUD Preview & Marker Eval`, `CMA-ES Calibration Main`, `ArUco Tracker Core`, `Basler Pylon Camera Source`, `ArucoTracker Build`, `Parameter Space Encoding`, `Calibration Objective Interface`, `Static Objective Scoring`?**
  _High betweenness centrality (0.108) - this node is a cross-community bridge._
- **Why does `ArucoTracker` connect `ArUco Tracker Core` to `Tracker Homography & SwarmClient`, `Camera Source Interface`, `Wingman V-Formation & macOS Input`, `Shape Demo Controller`, `ArUco Configuration`, `Circle Demo Controller`, `Drag & Drop Demo`, `Telemetry Ring Buffer`, `Vision Controller Tool`, `ArucoTracker Build`, `Kalman Filter Tracking`, `CLAHE Preprocessor`, `Fisheye Undistort Preprocessor`?**
  _High betweenness centrality (0.087) - this node is a cross-community bridge._
- **Why does `SwarmClient` connect `Terminal Dashboard UI` to `Tracker Homography & SwarmClient`, `Latency Plot & Game HUD`, `Shape Demo Controller`, `swarm_hub & evdev Keyboard`, `Circle Demo Controller`, `Drag & Drop Demo`, `Telemetry HUD`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `SwarmClient` (e.g. with `main()` and `main()`) actually correct?**
  _`SwarmClient` has 4 INFERRED edges - model-reasoned connections that need verification._
- **What connects `open`, `id`, `x` to the rest of the system?**
  _365 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Terminal Dashboard UI` be split into smaller, more focused modules?**
  _Cohesion score 0.06219426974143955 - nodes in this community are weakly interconnected._
- **Should `Latency Plot & Game HUD` be split into smaller, more focused modules?**
  _Cohesion score 0.0649895178197065 - nodes in this community are weakly interconnected._