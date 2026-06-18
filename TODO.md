# TODO

## Fix bugs in robots logic and fix odometry jitter.

## Bug: Latency
- [x] Latency only got calculated when swarm_controller was the client running,
  since the round-robin ping lived in its main loop instead of SwarmClient.
  Fixed: moved the 200ms round-robin ping into SwarmClient::poll(), so any
  program using SwarmClient (swarm_terminal, latency_plot, drag_drop_demo,
  shape_demo, ...) now gets live latency independent of which client is active.
  Note: vision_controller/wingman/circle_demo still never call poll(), so they
  still won't see latency (separate, pre-existing issue — they don't process
  any incoming telemetry at all, not just pongs).
- Investigated "second client (swarm_terminal) doubles latency": could not
  reproduce in a controlled A/B test (single idle robot, original pre-fix
  binaries, 15s windows) — avg/min/max were statistically identical with and
  without swarm_terminal attached. Natural RTT jitter alone spans ~2x
  (~2.5ms min vs ~5.5ms occasional spikes), which plausibly explains a casual
  "it doubled" observation. Re-open with specifics (robots moving? multiple
  robots known? which exact second client?) if it recurs.

## Grundlagen-Refactor (Basis für die folgenden Punkte)
- Codebase aufräumen
    - Socket/Protokoll-Boilerplate zentralisieren (aktuell in swarm_controller, swarm_terminal,
      latency_plot, game.cpp, circle_demo dupliziert) -> konsequent SwarmClient nutzen
    - Tuning-Konstanten (K_DIST, K_ANGLE, K_YAW_D etc.) in gemeinsame Config auslagern
      (analog aruco_tracker_config.json)
    - Große Demo-Dateien (circle_demo, wingman, shape_demo, je 800-1000 Zeilen) entflechten:
      Controller-Logik / Socket-Code / Rendering trennen

## Vision/Control Interface
- Vision und Control Layer isolieren, sauberes Vision-Interface (PoseStream) erstellen
    - Im swarm repo bleiben (kein Repo-Split, Codebase noch zu klein/jung dafür)
    - PlatformIO-Firmware ist bereits sauber getrennt - hier nur PC-seitige Vision/Control-
      Kopplung lösen
    - Vision-Demos sollen Posen über das Interface beziehen statt direkt ArucoTracker +
      Protokoll-Frames selbst zu bauen (drag_drop_demo macht das schon besser via SwarmClient)

## Formationsfunktionen zusammenfassen
- Funktionen wie follow_circle, follow_shape (kein Userinput nötig) als Swarm-/
  Formationsfunktionen-Bibliothek zusammenfassen (Kontroll-Mathematik ist bereits
  rendering-agnostisch, leicht extrahierbar)
- Übrige Funktionen mit Userinput (drag_drop, WASD-Controller) separat halten
    - Hier Latenz über remote testen (braucht Input-Stream Remote-Client -> PC)

## Webserver / Headless
- Webserver-Fähigkeit hinzufügen, komplett headless
    - swarm_hub/SwarmClient sind bereits headless - Kernarbeit ist Trennung von
      Tracking-Berechnung und Anzeige (cv::imshow) in den Vision-Demos
    - Telemetrie-Dashboard: Live-Posen/Latenz/Batterie über WebSocket an Browser-UI
      (kombiniert mit Remote-Latency-Test oben, auch ohne Vision-Pipeline auf dem
      Client nutzbar)
    - Bestehendes Telemetrie-Parsing (RobotStatus, parsePacket für MSG_ANNOUNCE/
      MSG_TELEMETRY/MSG_PONG) steckt aktuell in lib/swarm/swarm_terminal.cpp -> in
      gemeinsames Modul extrahieren, von Dashboard + swarm_terminal nutzen lassen,
      danach die alte Implementierung in swarm_terminal löschen

## Tooling / Tests
- Record/Replay: Vision-Posen + gesendete Commands loggen und offline replayen
  (hilft beim Tunen der Formationscontroller ohne Hardware, macht Latenz-Tests
  reproduzierbar)
- Unit-Tests für reine Funktionen: CRC-8 Framing (SwarmProtocol), Formation-Mathematik
  nach Extraktion
- Generalisiertes DEBUG-Util: gemeinsame Overlay/HUD-Komponente für cam_fps,
  loop_fps, Latenz, Batterie-Prozent etc. über alle PC-Tools (circle_demo,
  shape_demo, vision_controller, wingman, ...), statt jedes Tool sein eigenes
  Ad-hoc-HUD baut. Erster Schritt schon gemacht: die beiden FPS-Werte heißen
  jetzt überall "cam_fps" (Capture/Detection-Thread, aruco_tracker.h-Overlay)
  und "loop_fps" (jeweilige Main/Render-Loop des Tools).
- Default-Fenster-Framework: gemeinsame Basis für Fenstererstellung über alle
  PC-Tools (aktuell dupliziert jedes Tool sein eigenes namedWindow/
  resizeWindow/setMouseCallback/imshow-Boilerplate, siehe der einzelne
  resizeWindow-Fix vom 2026-06-12). Soll generalisiertes Drawing (Overlays,
  Marker, HUD), ein Debug-Menü unter dem DEBUG-Util oben sowie generalisierte
  Window-Properties/Designs (Default-Größe passend zur Kamera-Auflösung,
  Theme/Layout) bereitstellen.

## Performance: loop_fps vs cam_fps (circle_demo)
~~Gelöst (2026-06-12)~~ - loop_fps liegt jetzt bei ~115-117, praktisch
identisch mit cam_fps (115). Details siehe "Erledigt (2026-06-12)" unten und
PERFORMANCE.md.

## Hardware
- Bessere Aruco-Code Halterung bauen

## Später
- Windows Kompatibilität (neuer branch)
    - Aktuell kein #ifdef _WIN32 vorhanden - betrifft AF_UNIX Sockets, evdev-Tastatur,
      CoreGraphics, /dev/tty* Serial-Globs, /tmp/swarm_hub.sock
    - Niedrige Priorität - nach den Refactors oben (Grundlagen + Interface) deutlich
      einfacher umzusetzen

## Erledigt (2026-06-12)
- loop_fps-vs-cam_fps-Lücke (circle_demo) geschlossen: Ursache war ein
  `sleep_for(30ms)`-Poll in der Main-Loop (`if (!tracker.update()) {
  sleep_for(30ms); continue; }`), der bei jedem "noch kein frisches Ergebnis"
  voll in "other" landete - bei ~115fps-Kamera (8.7ms/Frame) ein massiver
  Overhead. Fix: 30ms -> 1ms. loop_fps jetzt ~115-117 (≈ cam_fps), unter
  jedem cpufreq-Governor/EPP. Die komplette Governor/EPP/Throttling/
  Fan-Curve-Untersuchung in PERFORMANCE.md war dadurch ein Red Herring -
  Details und Vorher/Nachher-Zahlen dort.

## Erledigt (2026-06-10)
- Build-Artefakte aus tools/ in tools/build/ verschoben (ein gemeinsames
  Verzeichnis für alle PC-Tool-Binaries inkl. calibrate)
    - tools/Makefile und tools/vision/calibration/Makefile bauen jetzt nach
      tools/build/ (BUILD_DIR-Variable, mkdir-Rule als order-only-Prerequisite)
    - .gitignore: 11 einzelne Binary-Einträge durch tools/build/ ersetzt
    - README.md und tools/vision/calibration/README.md auf neue Pfade angepasst
    - verwaiste, nicht ignorierte Binary tools/drag_drop_demo entfernt
- Makefile-Target für drag_drop_demo ergänzt (tools/vision/drag_drop_demo.cpp,
  analog zu shape_demo: ArucoTracker + SwarmClient)
