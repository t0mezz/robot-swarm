# TODO

(NEXT UP) ## Clean up Swarmhub code and terminal output, make it run as deamon on default.

(NEXT UP) ## Clean up unnessary comments

## Think about if its a good idea to: (is this already done?????)
- Create a unified, well tuned PID unit taking in references of the robot poses and desired poses (maybe in vector space)
- Every demo is dependent of a good PID controller
- we dont want unnessarary computational or memory costs though (references, vecor space conversion)

## questions to answer:
- is the kalmann filter implemented in aruco_tracker or redundant in every demo?

## Webserver / Headless
- Webserver-Fähigkeit hinzufügen, komplett headless
    - swarm_hub/SwarmClient sind bereits headless - Kernarbeit ist Trennung von
      Tracking-Berechnung und Anzeige (cv::imshow) in den Vision-Demos
    - Telemetrie-Dashboard: Live-Posen/Latenz/Batterie über WebSocket an Browser-UI
      (kombiniert mit Remote-Latency-Test oben, auch ohne Vision-Pipeline auf dem
      Client nutzbar)
    - Bestehendes Telemetrie-Parsing (RobotStatus, parsePacket für MSG_ANNOUNCE/
      MSG_TELEMETRY/MSG_PONG) steckt aktuell in tools/swarm/swarm_terminal.cpp -> in
      gemeinsames Modul extrahieren, von Dashboard + swarm_terminal nutzen lassen,
      danach die alte Implementierung in swarm_terminal löschen

## Tooling / Tests
- Record/Replay: Vision-Posen + gesendete Commands loggen und offline replayen
  (hilft beim Tunen der Formationscontroller ohne Hardware, macht Latenz-Tests
  reproduzierbar)
- ~~Unit-Tests für reine Funktionen: CRC-8 Framing (SwarmProtocol)~~ erledigt,
  siehe unten. Formation-Mathematik nach Extraktion noch offen.
- Generalisiertes DEBUG-Util: gemeinsame Overlay/HUD-Komponente für det_fps,
  loop_fps, Latenz, Batterie-Prozent etc. über alle PC-Tools (circle_demo,
  shape_demo, vision_controller, wingman, ...), statt jedes Tool sein eigenes
  Ad-hoc-HUD baut. Erster Schritt schon gemacht: die beiden FPS-Werte heißen
  jetzt überall "det_fps" (Detection-Thread, aruco_tracker.h-Overlay; hieß
  bis 2026-07 "cam_fps", war aber nie die Kamera-Rate — der Detection-Thread
  kann Frames überspringen, siehe GrabStrategy_LatestImageOnly) und
  "loop_fps" (jeweilige Main/Render-Loop des Tools). Der Config-Key
  `cam_fps` (angeforderte AcquisitionFrameRate) heißt weiterhin so.
- Default-Fenster-Framework: gemeinsame Basis für Fenstererstellung über alle
  PC-Tools (aktuell dupliziert jedes Tool sein eigenes namedWindow/
  resizeWindow/setMouseCallback/imshow-Boilerplate, siehe der einzelne
  resizeWindow-Fix vom 2026-06-12). Soll generalisiertes Drawing (Overlays,
  Marker, HUD), ein Debug-Menü unter dem DEBUG-Util oben sowie generalisierte
  Window-Properties/Designs (Default-Größe passend zur Kamera-Auflösung,
  Theme/Layout) bereitstellen.
- Optional/explorativ — Fixkosten des Detection-Threads senken (~8.7ms pro
  Frame bei 2048x2048, gemessen 2026-07-08 via `circle_demo --log-perf`):
  Die Kamera ist mono, aber `BaslerPylonSource::read()`
  (`lib/ArucoTracker/basler_pylon_source.h`) konvertiert jedes Frame nach
  BGR8 (12.6MB) + `.clone()`, und `detectionLoop()` konvertiert es direkt
  wieder per `cvtColor` nach Grau. Stattdessen Mono8 direkt aus dem
  Konverter ziehen (1/3 der Kopie, beide Konvertierungen entfallen) und das
  farbige Debug-Bild nur bei aktivem `debug_overlay` per gray->BGR bauen.
  Nur relevant, wenn wir näher ans Kameralimit (115fps) wollen — seit
  `half_res_sweep=1` (2026-07-08) läuft die Pipeline bereits bei Kamerarate;
  vorher/nachher in PERFORMANCE.md dokumentieren.

## Später
- Windows Kompatibilität (neuer branch)
    - Aktuell kein #ifdef _WIN32 vorhanden - betrifft AF_UNIX Sockets, evdev-Tastatur,
      CoreGraphics, /dev/tty* Serial-Globs, /tmp/swarm_hub.sock
    - Niedrige Priorität - nach den Refactors oben (Grundlagen + Interface) deutlich
      einfacher umzusetzen

# ================== FIXED ===================
## Genera aruco tracker issue: Erledigt (2026-07-08)
~~When a set of markers are registered for some time, the tracker is not
picking up now markers. resolved by restarting the demo (maybe due to lazy
search????)~~ Root cause: the full-frame global sweep in
`ArucoTracker::detectionLoop()` (`lib/ArucoTracker/aruco_tracker.h`) — the
only path that can discover a marker ID with no prior ROI state — only ran
when `markerStates_` was empty or some already-known marker had fully lost
tracking (`RoiState::GLOBAL`). Once every currently-known marker settled into
`LOCAL` ROI tracking, the global sweep stopped running entirely, so a
new/late-appearing robot was never picked up until an existing marker lost
tracking (or the demo was restarted, which cleared `markerStates_`). Fixed by
adding a `robot_count` config option (`aruco_tracker_config.json`,
`ArucoConfig::robotCount`, default 0 = uncapped): the global sweep now runs
every frame until that many distinct marker IDs have been discovered, then
drops back to the old on-demand behavior to save full-frame detection cost.

## Fix swarm dashboard flickering on ubuntu. Erledigt (2026-07-06)
~~Fix swarm dashboard (maybe other tools aswell) flickering on ubuntu, working
fine on macos tahoe~~ Root cause: `drawUI()` started with a full-screen erase
(`\033[H\033[J`) followed by dozens of separately flushed writes (stdout is
line-buffered on a tty); GNOME Terminal/VTE repaints between flushes and kept
catching the just-erased blank screen. macOS Terminal coalesces reads more, so
it never showed. Fix: each frame is built into one string and emitted as a
single `write()`, wrapped in DEC synchronized-update (`\033[?2026h/l`), with
per-line `\033[K` + trailing `\033[J` instead of a leading full clear, and the
cursor hidden while drawing. Check `swarm_terminal.cpp`/`latency_plot.cpp` for
the same pattern if they're kept (swarm_terminal is slated for removal below).
In the same pass the dashboard moved to snapshot-then-render: all robot states
are copied at one instant every `UPDATE_INTERVAL_MS` (250ms) and history gets
exactly one sample per robot per tick (uniform 15s sparkline window), so
values update in lockstep instead of whenever a frame arrives.

## Update Swarm dashboard: Erledigt (2026-06-24)
  - ~~robot motor level meter still does not show the value of the motors
    (f.e. -15,15 still dark grey level meter)~~ Fixed: `bipolarMeter()` in
    `swarm_dashboard.cpp` truncated `(int)(frac*mid)` toward zero, so small
    commands (and the divider eating the first positive cell) floored to 0 lit
    cells. Now rounds to nearest cell, guarantees ≥1 cell for any nonzero value,
    and lights symmetrically outward from the center divider.
  - ~~sometimes all robots are lost at the same time for seconds, especially
    when other software using swarm_hub is running.~~ Root cause: each
    `SwarmClient` ran its own 200ms round-robin pinger, so N connected tools =
    N independent ping streams. The dongle (`src/dongle/main.cpp`) tracks only
    *one* outstanding ping (single `pingTracker`); overlapping pings clobbered
    it (corrupt RTT) and the extra non-coalescible serial/ESP-NOW traffic could
    stall telemetry relay link-wide → all robots appear LOST together for a few
    seconds until it drained. Fix: pinging moved into `swarm_hub` (snoops
    announce/telemetry/pong for live robot IDs, emits one MSG_PING per interval
    for all clients). Per-client `autoPing()` removed from `SwarmClient`.

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

## Performance: loop_fps vs cam_fps (circle_demo) Erledigt (2026-06-12)
~~Gelöst (2026-06-12)~~ - loop_fps liegt jetzt bei ~115-117, praktisch
identisch mit cam_fps (115). Details siehe "Erledigt (2026-06-12)" unten und
PERFORMANCE.md.

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

  ## Verify is /lib/swarm/* is correctly placed there. Erledigt (2026-07-06)
Moved to `tools/swarm/` — the files there (`swarm_hub.cpp`, `swarm_terminal.cpp`,
etc.) are standalone PC-tool entry points, not reusable libraries linked into
firmware or multiple targets, so they belong under `tools/` alongside
`vision/*.cpp` and `game.cpp` rather than in `lib/` next to `SwarmClient`/
`SwarmProtocol`/`ArucoTracker`. Updated `tools/Makefile` (`SWARMDIR` var),
`CLAUDE.md`, and `README.md` accordingly.
