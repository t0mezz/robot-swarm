// app.js — layout budgeting, selection state, keyboard.
//
// Everything here is driven by explicit row/column arithmetic rather than
// flex alone. A terminal has no scrollbars and no overflow: a panel that
// asks for one row more than exists pushes the bottom of the screen off the
// screen, and the frame after it renders against a shifted origin. So the
// available rows are divided up once, per frame, and each panel is told
// exactly how tall it is.

import { useEffect, useRef, useState } from 'react';
import { Box, Text, useApp, useInput, useStdin, useStdout } from 'ink';
import { html } from './html.js';
import { C } from './theme.js';
import { HISTORY } from './telemetry.js';
import { StatusBar } from './components/StatusBar.js';
import { Roster } from './components/Roster.js';
import { LatencyGraph } from './components/LatencyGraph.js';
import { Arena } from './components/Arena.js';
import { Focus, FocusStrip } from './components/Focus.js';
import { Log, KeyHints } from './components/Log.js';

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

// Deep enough to survive `p`: the source mutates its history arrays in
// place, so a shallow copy would keep "paused" graphs animating.
function snapshot(s) {
  return {
    ...s,
    robots: s.robots.map((r) => ({ ...r, hist: [...r.hist] })),
    avgLat: [...s.avgLat],
    log: [...s.log],
    vision: { ...s.vision, robots: [...(s.vision.robots ?? [])] }
  };
}

function useTerminalSize() {
  const { stdout } = useStdout();
  const [size, setSize] = useState({
    cols: stdout.columns || 100,
    rows: stdout.rows || 30
  });
  useEffect(() => {
    const onResize = () => setSize({ cols: stdout.columns || 100, rows: stdout.rows || 30 });
    stdout.on('resize', onResize);
    return () => stdout.off('resize', onResize);
  }, [stdout]);
  return size;
}

// Divides the terminal into panels. Pulled out of the component so the
// breakpoints can be reasoned about (and tested) on their own.
//
// Two shapes, not one shape that shrinks:
//
//   WIDE (>=118 cols)   roster + swarm graph | arena + focus
//   STACKED (<118)      roster
//                       focus strip
//                       arena | swarm graph
//
// The arena reflows rather than disappearing. Dropping it below a width
// threshold was the original behaviour and it was wrong: at 100x30 the
// dashboard showed no vision at all while nine rows of empty graph sat
// underneath the roster.
export function computeLayout(cols, rows, robotCount) {
  // ── Vertical: the log yields rows to the body, then disappears ──
  let logH = clamp(Math.floor(rows * 0.18), 3, 8);
  let showLog = true;
  const chrome = () => 2 + 1 + (showLog ? 1 + logH : 0);  // status+rule, hints, rule+log
  const MIN_BODY = 6;

  let bodyH = rows - chrome();
  if (bodyH < MIN_BODY) {
    logH = Math.max(3, logH - (MIN_BODY - bodyH));
    bodyH = rows - chrome();
  }
  if (bodyH < MIN_BODY) {
    showLog = false;
    logH = 0;
    bodyH = rows - chrome();
  }
  bodyH = Math.max(0, bodyH);

  // ── Horizontal ──
  // The right column yields width before it restructures: at 132 columns a
  // 62-wide arena would squeeze the roster's trend column down to nothing,
  // and a latency trend you can't read is worse than a smaller arena.
  const rightW = cols >= 150 ? 62 : cols >= 118 ? 52 : 0;
  const stacked = rightW === 0;
  const gap = stacked ? 0 : 2;
  const leftW = cols - rightW - gap;

  const rosterH = Math.min(robotCount + 1, Math.max(3, Math.floor(bodyH * 0.55)));

  const L = {
    cols, rows, stacked, leftW, rightW, gap, bodyH, logH, showLog, rosterH,
    showRight: false, showFocus: false, focusH: 0, arenaH: 0, arenaW: 0,
    graphH: 0, graphW: 0, showGraph: false, focusStrip: false
  };

  if (!stacked) {
    L.showRight = bodyH >= 12;
    L.showFocus = L.showRight && bodyH >= 20;
    L.focusH = L.showFocus ? clamp(Math.floor(bodyH * 0.45), 11, 14) : 0;
    L.arenaH = L.showRight ? Math.max(5, bodyH - L.focusH - (L.showFocus ? 1 : 0)) : 0;
    L.arenaW = rightW;
    // Capped, not stretched to fill: past ~12 rows a braille trace gains no
    // readable detail, and a 28-row graph beside a 6-row roster reads as if
    // the graph were the point of the screen.
    L.graphH = Math.min(12, bodyH - rosterH - 1);
    L.graphW = leftW;
    L.showGraph = L.graphH >= 7;
    if (!L.showGraph) L.graphH = 0;
    return L;
  }

  // Stacked: the roster is always followed by a spacer row (which doubles as
  // the "N more" indicator), then the focus strip, then the lower band.
  // Forgetting that spacer here overruns the body by one row, and Ink clips
  // the overflow out of the middle of the frame — it cost a robot row and
  // half the arena legend.
  const SPACER = 1;
  L.focusStrip = bodyH - rosterH - SPACER >= 2;
  const band = bodyH - rosterH - SPACER - (L.focusStrip ? 1 : 0);

  // An arena narrower than a 5x5 grid says nothing a coordinate readout
  // doesn't say better, so below that the band goes entirely to the graph.
  const MIN_ARENA_H = 8;
  if (band >= MIN_ARENA_H) {
    L.arenaH = Math.min(band, 16);
    // Square grid: pick the width that exactly fits the height, so the arena
    // box has no dead padding inside it.
    L.arenaW = Math.min(leftW, 2 * (L.arenaH - 3) + 4);
    L.showRight = true;
  }
  L.graphW = leftW - L.arenaW - (L.arenaW ? 2 : 0);
  L.graphH = band;
  L.showGraph = band >= 4 && L.graphW >= 28;
  if (!L.showGraph) {
    L.graphH = 0;
    L.graphW = 0;
    // Nothing to share the band with — let the arena use the full width it
    // can square off against.
    if (L.arenaW) L.arenaW = Math.min(leftW, 2 * (L.arenaH - 3) + 4);
  }
  return L;
}

function sortRobots(robots, sortBy) {
  const out = [...robots];
  if (sortBy === 'latency') {
    // Lost robots are the most urgent thing on screen, so they sort above the
    // merely-slow ones rather than falling to the bottom on a latency of 0.
    out.sort((a, b) => (b.lost - a.lost) || (b.latencyUs - a.latencyUs));
  } else if (sortBy === 'battery') {
    out.sort((a, b) => (a.batteryV ?? 99) - (b.batteryV ?? 99));
  } else {
    out.sort((a, b) => a.id - b.id);
  }
  return out;
}

const SORTS = ['id', 'latency', 'battery'];

export function App({ source, onExit }) {
  const { exit } = useApp();
  const { isRawModeSupported } = useStdin();
  const { cols, rows } = useTerminalSize();
  const [state, setState] = useState(() => snapshot(source.state));
  const [selectedId, setSelectedId] = useState(null);
  const [sortBy, setSortBy] = useState('id');
  const [follow, setFollow] = useState(false);
  const [paused, setPaused] = useState(false);
  const pausedRef = useRef(false);

  useEffect(() => {
    const onUpdate = (s) => { if (!pausedRef.current) setState(snapshot(s)); };
    source.on('update', onUpdate);
    source.start();
    return () => source.off('update', onUpdate);
  }, [source]);

  const sorted = sortRobots(state.robots, sortBy);

  // Follow mode re-picks the robot most worth looking at every frame: a dead
  // link first, then the slowest. It is the "something is wrong, show me
  // which one" key.
  const followTarget = follow && sorted.length
    ? [...sorted].sort((a, b) => (b.lost - a.lost) || (b.latencyUs - a.latencyUs))[0].id
    : null;
  const effectiveId = followTarget ?? selectedId;
  const selected = sorted.find((r) => r.id === effectiveId) ?? sorted[0] ?? null;

  useInput((input, key) => {
    if (input === 'q' || key.escape || (key.ctrl && input === 'c')) {
      onExit?.();
      exit();
      return;
    }
    if (input === 'p') {
      pausedRef.current = !pausedRef.current;
      setPaused(pausedRef.current);
      return;
    }
    if (input === 's') {
      setSortBy((s) => SORTS[(SORTS.indexOf(s) + 1) % SORTS.length]);
      return;
    }
    if (input === 'f') { setFollow((f) => !f); return; }
    if ((key.upArrow || key.downArrow || input === 'k' || input === 'j') && sorted.length) {
      setFollow(false);
      const up = key.upArrow || input === 'k';
      const idx = sorted.findIndex((r) => r.id === (selected?.id ?? sorted[0].id));
      const next = clamp(idx + (up ? -1 : 1), 0, sorted.length - 1);
      setSelectedId(sorted[next].id);
    }
  // Keyboard needs raw mode, which a piped stdin doesn't have. Without this
  // guard Ink throws on mount, so `node src/cli.js | head` — a reasonable
  // thing to try — dumps a stack trace instead of frames.
  }, { isActive: isRawModeSupported });

  // Ink terminates every frame with a newline. A frame exactly as tall as
  // the terminal therefore scrolls it by one row on each render, which walks
  // the status bar off the top of the screen. Give the newline a row to land
  // in instead.
  const frameH = rows - 1;
  const L = computeLayout(cols, frameH, sorted.length);
  const windowSec = Math.round((HISTORY * state.intervalMs) / 1000);

  // Roster taller than its budget scrolls around the selection rather than
  // being cut off at whatever id happens to sort last.
  const rosterCap = Math.max(1, L.rosterH - 1);
  let start = 0;
  if (sorted.length > rosterCap && selected) {
    const idx = sorted.findIndex((r) => r.id === selected.id);
    start = clamp(idx - Math.floor(rosterCap / 2), 0, sorted.length - rosterCap);
  }
  const visible = sorted.slice(start, start + rosterCap);
  const hidden = sorted.length - visible.length;

  const banner = state.producer === 'failed' ? state.producerError : null;

  const linked = state.robots.filter((r) => !r.lost).length;
  const graphSubtitle = `mean of ${linked} linked robot${linked === 1 ? '' : 's'}`;

  return html`
    <${Box} flexDirection="column" width=${cols} height=${frameH}>
      <${StatusBar} state=${state} width=${cols} paused=${paused} />
      <${Text} color=${C.rule}>${'─'.repeat(cols)}<//>

      <${Box} height=${L.bodyH} flexDirection="row">
        <${Box} width=${L.leftW} flexDirection="column">
          ${banner
            ? html`
                <${Box} flexDirection="column" marginBottom=${1}>
                  ${banner.split('\n').map((line, i) => html`
                    <${Text} key=${i} color=${i === 0 ? C.red : C.dim} wrap="truncate">
                      ${i === 0 ? ' producer: ' : '   '}${line}
                    <//>
                  `)}
                <//>
              `
            : null}

          <${Roster} robots=${visible} width=${L.leftW}
                     selectedId=${selected?.id ?? null} sortBy=${sortBy} />
          ${hidden > 0
            ? html`<${Text} color=${C.dim}>   … ${hidden} more (↑↓ to scroll)<//>`
            : html`<${Text}> <//>`}

          ${L.focusStrip
            ? html`<${FocusStrip} robot=${selected} width=${L.leftW} />`
            : null}

          ${L.stacked
            ? html`
                <${Box} flexGrow=${1} flexDirection="row" alignItems="flex-end">
                  ${L.showRight
                    ? html`
                        <${Arena} vision=${state.vision} robots=${state.robots}
                                  selectedId=${selected?.id ?? null}
                                  width=${L.arenaW} height=${L.arenaH} />
                        ${L.showGraph ? html`<${Box} width=${2}><${Text}> <//><//>` : null}
                      `
                    : null}
                  ${L.showGraph
                    ? html`
                        <${LatencyGraph} series=${state.avgLat} width=${L.graphW}
                                         height=${L.graphH} windowSec=${windowSec}
                                         subtitle=${graphSubtitle} />
                      `
                    : null}
                <//>
              `
            : L.showGraph
              ? html`
                  <${Box} flexGrow=${1} flexDirection="column" justifyContent="flex-end">
                    <${LatencyGraph} series=${state.avgLat} width=${L.graphW}
                                     height=${L.graphH} windowSec=${windowSec}
                                     subtitle=${graphSubtitle} />
                  <//>
                `
              : null}
        <//>

        ${!L.stacked && L.showRight
          ? html`
              <${Box} width=${L.gap}><${Text}> <//><//>
              <${Box} width=${L.rightW} flexDirection="column">
                <${Arena} vision=${state.vision} robots=${state.robots}
                          selectedId=${selected?.id ?? null}
                          width=${L.arenaW} height=${L.arenaH} />
                ${L.showFocus
                  ? html`<${Focus} robot=${selected} width=${L.rightW} height=${L.focusH} />`
                  : null}
              <//>
            `
          : null}
      <//>

      ${L.showLog
        ? html`
            <${Text} color=${C.rule}>${'─'.repeat(cols)}<//>
            <${Log} entries=${state.log} width=${cols} height=${L.logH} now=${Date.now()} />
          `
        : null}
      <${KeyHints} width=${cols} sortBy=${sortBy} follow=${follow} paused=${paused} />
    <//>
  `;
}
