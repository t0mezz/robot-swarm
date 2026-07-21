# TODO

## IMPROVE FORMATE
- every robot should move to position after each other — still open. Today all
  robots depart at once and the avoidance engine untangles whatever conflicts
  remain. Sequencing departures (or grouping them by slot distance) would cut
  the number of conflicts instead of resolving them.
- ~~hard code home formation for indefinit robots~~ addressed by
  `tools/vision/formation.cpp` (2026-07-21), though *not* by hard-coding:
  formations are named slot lists placed in the camera view and stored in
  `~/.config/robot-swarm/formations.yml`, and the robot count is free — the
  assignment is solved at run time, so surplus robots park and surplus slots
  stay empty. `./formation --add home` creates the home formation.

## Formation tool (2026-07-21) — NEEDS FLOOR VALIDATION
`tools/vision/formation.cpp`. Reuses drag_drop_demo's control loop, gains and
the stateful `AvoidanceEngine` verbatim; only the goal source differs (assigned
slots instead of the mouse). Modes: `--list`, `--add NAME`, `--run NAME`,
`--delete NAME`, `--frontend`, `--hold`; bare invocation is a CLI picker that
then runs headless.

- Robot→slot assignment is a Hungarian/JV solve (O(n³), rectangular) on **plain
  Euclidean distance, not squared**. This is load-bearing: a sum-of-distances
  optimum is provably non-crossing (triangle inequality), a squared one is not —
  the swap that removes a crossing changes squared cost by `2(w-y)(z-x)`, which
  is positive for half of all geometries. Measured over 20k random equal-count
  layouts: squared left a crossing in **15.9%** of formations vs **0.000%** for
  linear, buying only ~8% shorter worst-case travel (1089mm vs 1174mm mean). A
  crossing costs far more than 8% because it latches the avoidance engine, drops
  the dodger to `dodgeSpeedFrac` and can escalate to an emergency stop. Do not
  "optimise" this back to squared distance.
- The solver is checked against exhaustive permutation search (3000 random
  layouts: optimal cost, exactly min(n,m) assignments, no duplicate slots, zero
  crossings). That harness is currently ad-hoc and lives outside the repo — it
  is the obvious first candidate for the "Formation-Mathematik" unit tests noted
  under "Tooling / Tests", since `assignSlots` is already a pure function.
- **Untested on hardware.** Only the camera-less paths (library round-trip,
  --list/--delete, error handling, CLI menu) have been exercised. First floor
  run should be `--frontend --speed 20`; the frontend draws the assignment
  arrows specifically so a crossing is visible before anything moves.
- Slots are world mm, so a formation is only meaningful against the homography
  it was placed under. Both `--add` and the run modes refuse to proceed without
  one rather than silently mixing pixel and world coordinates. Re-calibrating
  the arena invalidates every stored formation — no versioning guards this yet.
- Reassignment is debounced (`REASSIGN_DEBOUNCE_S`) on a change in the visible
  robot set, not run per frame, so a blinking marker cannot reshuffle the swarm
  mid-drive. If markers drop out often on the floor, this is the knob.

## Avoidance reworked to dodge-based (2026-07-21) — NEEDS FLOOR VALIDATION
The braking strategy is gone. `lib/SwarmControl/avoidance.h` is now an
`AvoidanceEngine` (stateful — it latches dodges) instead of a pure function:
one robot of a conflicting pair is nominated as dodger and makes a committed
detour, the other is `priority` and is never slowed. No proximity speed ramp.

**All numbers below come from a closed-loop sim, not hardware.** The sim assumed
3.0 mm/s per motor unit and 3 deg/s per unit of turn — both GUESSES. The
structural findings are parameter-independent and trustworthy; the specific
constants are not.

- The trigger distance is bounded below by kinematics:
  `dangerMm > (2 x MAX_SPEED x mmPerUnit) x (90 / turnRateDegPerSec)`. At
  MAX_SPEED 60 that is ~340mm, which is why DANGER_MM went 200 -> 500 and
  SAFE_MM 400 -> 1000. Below that floor the dodge cannot complete at any blend.
  **Measure mm-per-motor-unit and deg/s-per-unit on the floor and redo this
  arithmetic** — if the real values differ, so does the required distance.
- **DANGER_MM=500 may be too large for the arena.** In an 800x600 arena nearly
  every pair is permanently in conflict. If that shows up, lower MAX_SPEED
  rather than DANGER_MM — they are two ends of the same inequality.
- Sim results at danger 500 / safe 1000 / blend 0.95 / emergency 160: head-on
  swap, perpendicular cross, overtake and mover-vs-parked all clear (minSep
  108-128mm, > the 98mm contact distance) and all reach their goals. Confirm
  each of those four on hardware.
- **Speed raised 1.5x (2026-07-21): MAX_SPEED 60 -> 90, MAX_TURN 16 -> 24.**
  These must move TOGETHER. computeGoto emits forward +/- turn into a +/-100
  motor clamp, so raising speed alone saturates the outer wheel and the robot
  turns lazily — and a lazy turn needs a longer dodge runway. Scaling both keeps
  turn rate proportional to speed and leaves the avoidance geometry unchanged,
  so DANGER_MM stays 500. emergencyMm scaled 160 -> 240 to match. All four
  encounter types still clear in sim (minSep 110-189mm). Watch for wheel-slip
  and ArUco motion blur on the floor — neither is modelled in the sim.
- The `swarm_dashboard`/HUD now shows DODGE vs HOLD per robot — quickest way to
  see whether the nomination is sane while watching the floor.

## Avoidance thresholds vs. body size (lib/SwarmControl/avoidance.h)
`ROBOT_DIAMETER_MM` (98.0) exists in `avoidance.h` and is the unit every distance
there is implicitly measured in — two robots touch at ~1.0 D centre-to-centre.
It is taken from the published 3pi+ chassis size, NOT measured.

Superseded in part by the dodge rework above: the old `stopMm`/crawl band is
gone, so the "hard stop radius sits inside the footprint" finding no longer
applies. What remains open is the calibration underneath it:
- Measure the chassis with calipers, confirm `ROBOT_DIAMETER_MM`.
- Confirm the ArUco scale: `minDist` is a marker-*centre* distance, so it only
  equals a hull distance if the marker is centred and the mm scale is right.
- Measure mm-per-motor-unit and deg/s-per-turn-unit. The dodge runway
  inequality, DANGER_MM, and emergencyMm all derive from these, and all three
  are currently sized off sim guesses (3.0 and 3.0).
- Then express dangerMm / safeMm / emergencyMm as multiples of D.

Fix inconsistancies in SWARM_DASHBOARD and ROBOT communication: ROOT CAUSE FOUND
(2026-07-21) — fix in src/receiver/main.cpp, pending on-hardware verification.
- Symptom: specific robots appear LOST on the dashboard for ~30s at a time (others
  fine); circle_demo still drives them (motor path = broadcast SWARM, unaffected);
  restarting the robot clears it.
- Root cause: the receiver learned the dongle's MAC from the *first ESP-NOW frame
  it heard, of any type* (`onReceive` captured the raw source MAC). MSG_ANNOUNCE is
  broadcast, so a booting robot could latch a *neighbour robot's* MAC as "the
  dongle". Telemetry + pong are unicast to that MAC (`Transport::sendToDongle`), so
  they went to the wrong robot and never reached the PC — while broadcast announces
  still got ACK'd, keeping the robot ACTIVE. `lastDongleSeen` stayed fresh off
  broadcast SWARM/pings, so the 10s ANNOUNCE_TIMEOUT never fired and the wrong MAC
  was never corrected without a restart. The only frame reaching the PC was the 30s
  re-announce → dashboard blinked the robot back every REANNOUNCE_INTERVAL_MS.
- Fix: learn/verify the dongle MAC only from *validated dongle-authored* frames
  (SWARM/ACK/PING/PONG) in `processIncoming()`, and re-register when the source MAC
  differs, so a robot self-heals a wrong dongleMAC within one dongle frame instead
  of needing a power cycle. `onReceive` no longer captures the MAC at all.
- TODO: flash robots + a neighbour, confirm all stay green on swarm_dashboard under
  circle_demo, and that a deliberately mis-latched robot recovers without restart.

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
## Avoidance rework: holes found by simulation (2026-07-21)
Design bugs caught by running the algorithm closed-loop rather than reasoning
about it. Each was invisible in unit tests — they need robots actually moving.

- **Nomination by ID before testing who is closing.** In an overtake this picks
  the robot in front, which is driving away and correctly declines to dodge — so
  nobody dodges and the robot behind, which is never slowed, rear-ends it.
  Sim: minSep 0.0mm, i.e. straight through. Fixed by requiring a dodger to be
  moving AND closing; ID priority is now only the tiebreak when both qualify.
- **Latching the arc as a world-frame vector.** The bearing between two robots
  rotates as they converge, so a frozen detour goes stale. Pinned perpendicular
  crossings at ~55mm separation *regardless of trigger distance or blend* — the
  parameter-invariance is what identified it as structural. Fixed by latching
  the side (a sign) and recomputing the perpendicular from the live bearing.
  Head-on hid this bug because that bearing barely moves.
- **Emergency-stopping both robots deadlocks permanently.** Neither can move, so
  the distance never changes and the stop never lifts: a crossing locked solid
  at 110mm and stayed there — strictly worse than the braking it replaced.
  Stopping only the dodger is the opposite failure (the priority robot drives
  through it, 6mm). Fixed: stop the CHARGING robot, let the dodger maneuver —
  it is the only party with a plan, so it is the only possible escape.
- **Stale nomination when the dodger parks.** A latched dodge kept pointing at
  its dodger after that robot reached its goal and stopped, so the priority
  robot — never slowed — drove into a robot that would never move. Recovery came
  only via the 4s timeout, by which point the pair had closed to 56mm (contact),
  versus 108mm for the same geometry with neither robot parking. Fixed by
  dropping the latch as soon as `!isMoving(dodger)`, which re-nominates on the
  next frame (~5mm of travel at 50Hz). The MIRROR case was already correct: when
  the *priority* robot parks the latch stays valid, since the dodger is still
  the one moving and still the one that has to get around — 143mm, no change.
- **Emergency-stopping a fleeing robot.** In an overtake the priority robot is
  in front and moving away; halting it deletes the separation it was creating
  (128mm -> 55mm). Fixed by stopping a robot only if it is actually closing.

## Avoidance defects (lib/SwarmControl/avoidance.h) Erledigt (2026-07-21)
Three of the four defects found by probing the extracted code are fixed, with
regression tests in `tests/test_goto_controller.cpp`. Still synthetic-only —
none were observed on hardware, and none are retunes, so behavior on the floor
should be unchanged except where it was previously broken. The fourth
(thresholds vs. body size) is still open above, since it *is* a retune.

- ~~Crossing paths get no avoidance at all.~~ The moving/moving branch required
  BOTH robots to be closing (`faceFace`), so a robot driving straight at a
  neighbour got nothing as soon as that neighbour moved across the line of
  centers. Sharper than first recorded: the *identical* geometry with a parked
  neighbour did arc, so a moving obstacle was treated as **less** dangerous than
  a stationary one. Fixed by judging each robot on its own approach —
  `loClosing`/`hiClosing` evaluated separately; mutual closing still yields by ID
  (higher ID arcs), so the face-to-face rule is unchanged.

- ~~Stall band just above the danger line.~~ The proximity ramp now runs to zero
  at the stop radius and is floored at `dodgeSpeedFrac` for a robot that is
  arcing, making the curve monotonic and continuous across `stopR` instead of
  collapsing to ~0 just outside it and jumping back to 9.0 below. A dodging
  robot always keeps at least the crawl, so the two-robots-in-the-band deadlock
  can't form. Covered by a monotonicity sweep over 20-240mm.

- ~~`applyAvoidance()` leaves `maxSpd` at the base value when `hardStop`.~~ Now
  zeroed, so a caller that reads the cap without checking `hardStop` first still
  gets a safe number.

- Drive-by: `AVOID_PARAMS` in `drag_drop_demo.cpp` used positional aggregate
  init, which silently mis-assigns every field when a member is added to
  `AvoidParams` (adding `stopMm` would have put 0.25 in it and 0.15 in
  `faceDot`). Converted to designated initializers.


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
