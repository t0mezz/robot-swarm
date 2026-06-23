# TODO

## ⚠️ Reflash needed (2026-06-19)
- Reflash the **robot RP2040** (`src/robots/` MicroPython: `robot_uart.py`) and the
  **receiver ESP32** (`pio run -e receiver -t upload`) — added the `MSG_DEBUG`
  robot→PC debug-log channel (`UARTProtocol.send_debug()` → receiver relays with
  `robot_id` → `swarm_terminal` DEBUG LOG pane). The **dongle is unchanged** (it
  already forwards unknown frame types 1:1 to the PC).

## Think about if its a good idea to:
- Create a unified, well tuned PID unit taking in referenced of robot poses and desired poses (maybe in vector space)
- Every demo is dependent of a good PID controller
- we dont want unnessarary computational or memory costs though (references, vecor space conversion)

## Fix bugs in robots logic and fix odometry jitter.

## Tuning-Konstanten (K_DIST, K_ANGLE, K_YAW_D etc.) in gemeinsame Config auslagern (analog aruco_tracker_config.json)

## Idea: remote robot shutdown command — SCRAPPED (2026-06-19)

**Scrapped: not firmware-feasible without a hardware mod on every robot.**
The power latch's control input is *not* wired to any RP2040/ESP32 GPIO on the
3pi+ 2040 — there is no onboard trace to drive it from software. Triggering it
would require manually soldering a wire from a free GPIO to the latch on each
of the (up to 32) robots. Not worth it; parked. Research kept below in case the
trade-off changes (e.g. a one-off hardware revision).

Goal was: power a robot off remotely (end of session, low battery, retrieving a
robot that wandered out of reach) instead of walking over to press the physical
power button.

**Research findings:**
- The 3pi+ 2040 control board's power button is *not* a plain mechanical
  switch — it's Pololu's solid-state pushbutton-power-switch latching
  circuit. Driving the latch's control input releases it and cuts the board's
  own power. So a true power-off is electrically possible *if* you can reach
  that input — but see the scrap reason above: no onboard GPIO connection
  exists. Source: Pololu 3pi+ 2040 User's Guide §6.7 "Power".
- The vendored `pololu_3pi_2040_robot` MicroPython library exposes no
  `power.py`/shutdown module (modules: `battery.py`, `motors.py`,
  `buttons.py`, `buzzer.py`, `display.py`, `encoders.py`, `imu.py`,
  `ir_sensors.py`, `rgb_leds.py`, `yellow_led.py`) — consistent with there
  being no software-reachable latch line.
- CONFIRMED: the onboard ESP32-S3 receiver has no battery of its own — it's
  powered from the 3pi+ board's regulated rail — so cutting RP2040/board power
  also cuts the receiver. Shutdown would have been "whole robot off" (matches
  the physical button, irreversible remotely), which was the intended behavior.

**Sketched design (only relevant if the hardware mod ever gets done):**
- New protocol message targeted at a single robot ID (not broadcast — an
  accidental swarm-wide shutdown would require physically visiting every
  robot to recover). Either reuse `MSG_CONFIG` (0x40, PC→Robot) or add a new
  `MSG_SHUTDOWN`; needs adding in all three protocol implementations (see
  CLAUDE.md "Wire protocol" note). Path: PC tool → swarm_hub → dongle →
  ESP-NOW unicast → robot ESP32 → new UART message type → RP2040.
- RP2040 graceful sequence: `motors.set_speeds(0, 0)` → brief "shutting down"
  message on the OLED → short delay → drive the (hand-wired) latch GPIO to
  release power.
- PC-side: require an explicit per-robot confirmation step, since the action
  is irreversible without physical access.

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
- ~~Unit-Tests für reine Funktionen: CRC-8 Framing (SwarmProtocol)~~ erledigt,
  siehe unten. Formation-Mathematik nach Extraktion noch offen.
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


## Bug: Latency Erledigt (2026-06-18)
- [x] Latency only got calculated when swarm_controller was the client running,
  since the round-robin ping lived in its main loop instead of SwarmClient.
  Fixed: moved the 200ms round-robin ping into SwarmClient::poll(), so any
  program using SwarmClient (swarm_terminal, latency_plot, drag_drop_demo,
  shape_demo, ...) now gets live latency independent of which client is active.
  Note: vision_controller/wingman/circle_demo previously never called poll(),
  so they didn't see latency (separate, pre-existing issue — they didn't
  process any incoming telemetry at all, not just pongs). Fixed (2026-06-23):
  each tool's main loop now calls `g_swarm.poll()` / `swarm.poll()` once per
  iteration (under `g_swarmMutex` in vision_controller/wingman, which are
  multi-threaded; circle_demo is single-threaded so no lock needed).
- Investigated "second client (swarm_terminal) doubles latency": could not
  reproduce in a controlled A/B test (single idle robot, original pre-fix
  binaries, 15s windows) — avg/min/max were statistically identical with and
  without swarm_terminal attached. Natural RTT jitter alone spans ~2x
  (~2.5ms min vs ~5.5ms occasional spikes), which plausibly explains a casual
  "it doubled" observation. Re-open with specifics (robots moving? multiple
  robots known? which exact second client?) if it recurs.

  ## Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12)
~~Gelöst (2026-06-12)~~ - loop_fps liegt jetzt bei ~115-117, praktisch
identisch mit cam_fps (115). Details siehe "Erledigt (2026-06-12)" unten und
PERFORMANCE.md.

## Tests: CRC-8 Framing Unit Tests Erledigt (2026-06-19)
- [x] Neues `tests/`-Verzeichnis (eigenes Makefile, kein Framework, einfache
  Asserts): `tests/test_protocol.cpp` deckt die reinen Funktionen aus
  `lib/SwarmProtocol/protocol.h` ab (`crc8`, `buildFrame`, `validateFrame`,
  `frameSize`) — bekannte CRC-8-Testvektoren (u.a. der Standard-Check-Wert
  0xF4 für "123456789", unabhängig per Python gegengerechnet), Build→Validate-
  Round-Trip für mehrere Payload-Längen, sowie Ablehnung von kaputten Frames
  (CRC-Bit-Flip, Payload-Bit-Flip, falsches Magic-Byte, abgeschnittener Buffer).
  `cd tests && make test` baut und führt sie aus. Formation-Mathematik bleibt
  offen (siehe "Tooling / Tests" oben — braucht erst die Extraktion in reine
  Funktionen).