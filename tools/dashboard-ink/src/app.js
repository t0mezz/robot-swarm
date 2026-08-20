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
import { Focus } from './components/Focus.js';
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
export function computeLayout(cols, rows, robotCount) {
  // The right column yields width before it disappears: at 132 columns a
  // 62-wide arena would squeeze the roster's trend column down to nothing,
  // and a latency trend you can't read is worse than a smaller arena.
  const rightW = cols >= 150 ? 62 : cols >= 118 ? 52 : 0;
  const gap = rightW ? 2 : 0;
  const leftW = cols - rightW - gap;

  const logH = clamp(Math.floor(rows * 0.18), 3, 8);
  // status + rule + body + rule + log + hints
  const bodyH = rows - 1 - 1 - 1 - logH - 1;

  const showRight = rightW > 0 && bodyH >= 12;
  const showFocus = showRight && bodyH >= 20;
  const focusH = showFocus ? clamp(Math.floor(bodyH * 0.45), 11, 14) : 0;
  const arenaH = showRight ? Math.max(5, bodyH - focusH - (showFocus ? 1 : 0)) : 0;

  // The roster gets what it needs up to half the body; the swarm graph takes
  // the rest. Below ~8 rows a braille graph carries no more information than
  // the per-robot sparklines already do, so it yields entirely.
  const rosterWanted = robotCount + 1;
  const rosterH = Math.min(rosterWanted, Math.max(3, Math.floor(bodyH * 0.55)));
  // Capped, not stretched to fill: past ~12 rows a braille trace gains no
  // readable detail, and a 28-row graph next to a 6-row roster reads as if
  // the graph were the point of the screen. Spare rows stay empty.
  const graphH = Math.min(12, bodyH - rosterH - 1);
  const showGraph = graphH >= 7;

  return {
    cols, rows, leftW, rightW, gap, bodyH, logH,
    showRight, showFocus, focusH, arenaH,
    rosterH, graphH: showGraph ? graphH : 0, showGraph
  };
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

          ${L.showGraph
            ? html`
                <${Box} flexGrow=${1} flexDirection="column" justifyContent="flex-end">
                  <${LatencyGraph} series=${state.avgLat} width=${L.leftW}
                                   height=${L.graphH} windowSec=${windowSec}
                                   subtitle=${(() => {
                                     const n = state.robots.filter((r) => !r.lost).length;
                                     return `mean of ${n} linked robot${n === 1 ? '' : 's'}`;
                                   })()} />
                <//>
              `
            : null}
        <//>

        ${L.showRight
          ? html`
              <${Box} width=${L.gap}><${Text}> <//><//>
              <${Box} width=${L.rightW} flexDirection="column">
                <${Arena} vision=${state.vision} robots=${state.robots}
                          selectedId=${selected?.id ?? null}
                          width=${L.rightW} height=${L.arenaH} />
                ${L.showFocus
                  ? html`<${Focus} robot=${selected} width=${L.rightW} height=${L.focusH} />`
                  : null}
              <//>
            `
          : null}
      <//>

      <${Text} color=${C.rule}>${'─'.repeat(cols)}<//>
      <${Log} entries=${state.log} width=${cols} height=${L.logH} now=${Date.now()} />
      <${KeyHints} width=${cols} sortBy=${sortBy} follow=${follow} paused=${paused} />
    <//>
  `;
}
