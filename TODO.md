# TODO

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

## Hardware
- Bessere Aruco-Code Halterung bauen

## Später
- Windows Kompatibilität (neuer branch)
    - Aktuell kein #ifdef _WIN32 vorhanden - betrifft AF_UNIX Sockets, evdev-Tastatur,
      CoreGraphics, /dev/tty* Serial-Globs, /tmp/swarm_hub.sock
    - Niedrige Priorität - nach den Refactors oben (Grundlagen + Interface) deutlich
      einfacher umzusetzen

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
