// glyphs.js — pure functions that turn numbers into terminal glyphs.
//
// No React, no state, no I/O: everything here is a value-in/string-out
// transform, which is what makes the layout testable without a terminal
// (see test/glyphs.test.js).

import { C } from './theme.js';

// ── Braille line graphs ──────────────────────────────────────────
// A braille cell is a 2x4 dot matrix, so one terminal cell carries 8
// independently addressable pixels — 8x the vertical resolution of the
// block-character sparkline. Same algorithm as brailleGraph() in
// tools/swarm/swarm_dashboard.cpp, kept identical so both dashboards draw
// the same trace from the same samples.

const BIT = [[0x01, 0x08], [0x02, 0x10], [0x04, 0x20], [0x40, 0x80]];

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

// Renders `series` as `rows` lines of `cells` braille characters, scaled to
// [0, maxVal], newest sample at the right edge. Consecutive samples are
// joined vertically so the trace reads as a continuous line rather than
// disconnected dots.
export function brailleGraph(series, cells, rows, maxVal) {
  if (!(maxVal > 0)) maxVal = 1;
  if (cells < 1 || rows < 1) return [];
  const dotW = cells * 2, dotH = rows * 4;
  const dots = Array.from({ length: dotH }, () => new Uint8Array(dotW));

  const n = Math.min(series.length, dotW);
  const x0 = dotW - n;
  let prevY = -1;
  for (let i = 0; i < n; i++) {
    const v = series[series.length - n + i];
    const frac = clamp(v / maxVal, 0, 1);
    const y = Math.round((1 - frac) * (dotH - 1));
    const from = prevY < 0 ? y : prevY;
    for (let yy = Math.min(from, y); yy <= Math.max(from, y); yy++) dots[yy][x0 + i] = 1;
    prevY = y;
  }

  const out = [];
  for (let r = 0; r < rows; r++) {
    let line = '';
    for (let c = 0; c < cells; c++) {
      let mask = 0;
      for (let dy = 0; dy < 4; dy++)
        for (let dx = 0; dx < 2; dx++)
          if (dots[r * 4 + dy][c * 2 + dx]) mask |= BIT[dy][dx];
      line += String.fromCharCode(0x2800 + mask);
    }
    out.push(line);
  }
  return out;
}

// One-row sparkline, drawn with block characters rather than braille.
//
// Braille wins on multi-row graphs (8 dots per cell), but a ONE-row braille
// cell only has 4 vertical positions, which flattens a latency trace into a
// dashed line. Block characters give 8 levels in the same single row, so the
// inline trend column reads as an actual shape.
const BLOCKS = [' ', '▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];

export function blockSpark(series, cells, maxVal) {
  if (!(maxVal > 0)) maxVal = 1;
  if (cells < 1) return '';
  const n = Math.min(series.length, cells);
  let out = ' '.repeat(cells - n);
  for (let i = series.length - n; i < series.length; i++) {
    const frac = clamp(series[i] / maxVal, 0, 1);
    out += BLOCKS[clamp(Math.round(frac * 8), 0, 8)];
  }
  return out;
}

// ── Bars ─────────────────────────────────────────────────────────

const EIGHTHS = ['', '▏', '▎', '▍', '▌', '▋', '▊', '▉'];

// Horizontal bar with sub-cell precision: the last cell is drawn as a
// partial block, so a 9-cell battery meter resolves ~72 steps instead of 9
// and small changes are actually visible.
export function smoothBar(frac, width) {
  frac = clamp(frac, 0, 1);
  const eighths = Math.round(frac * width * 8);
  const full = Math.floor(eighths / 8);
  const rem = eighths % 8;
  const bar = '█'.repeat(full) + EIGHTHS[rem];
  return { bar, track: '░'.repeat(Math.max(0, width - full - (rem ? 1 : 0))) };
}

// Bipolar L/R drive in ONE row. Upper half-block is the left motor, lower
// half-block the right; a cell where both are lit and both run the same
// direction becomes a full block. This is the layout's density trick — the
// C++ dashboard spends two full-width meters and ~50 columns on what fits
// here in 21, which is most of what makes 32 robots fit on a screen.
//
// Returns cells as {ch, color, bg} so the caller can merge runs.
export function dualDrive(l, r, width, maxV = 127) {
  const mid = Math.floor(width / 2);
  const mag = (v) => {
    const m = Math.round((Math.abs(v) / maxV) * mid);
    return v !== 0 && m === 0 ? 1 : m;  // any nonzero command lights ≥1 cell
  };
  const ml = mag(l), mr = mag(r);
  const cl = l >= 0 ? C.cyan : C.magenta;
  const cr = r >= 0 ? C.cyan : C.magenta;

  const cells = [];
  for (let i = 0; i < width; i++) {
    if (i === mid) { cells.push({ ch: '┃', color: C.rule }); continue; }
    const rel = i - mid;
    const onL = l >= 0 ? rel >= 1 && rel <= ml : rel <= -1 && rel >= -ml;
    const onR = r >= 0 ? rel >= 1 && rel <= mr : rel <= -1 && rel >= -mr;
    if (onL && onR) {
      if (cl === cr) cells.push({ ch: '█', color: cl });
      else cells.push({ ch: '▀', color: cl, bg: cr });
    } else if (onL) cells.push({ ch: '▀', color: cl });
    else if (onR) cells.push({ ch: '▄', color: cr });
    else cells.push({ ch: '·', color: C.rule });
  }
  return cells;
}

// Merges adjacent cells that share styling into one run, so a 21-cell drive
// meter renders as ~4 Text nodes instead of 21. At 32 robots x 4Hz that is
// the difference between a few hundred nodes per frame and a few thousand.
export function mergeRuns(cells) {
  const out = [];
  for (const c of cells) {
    const last = out[out.length - 1];
    if (last && last.color === c.color && last.bg === c.bg) last.text += c.ch;
    else out.push({ text: c.ch, color: c.color, bg: c.bg });
  }
  return out;
}

// ── Formatting ───────────────────────────────────────────────────

export function fmtMs(us) {
  if (us == null || us === 0) return '—';
  return `${(us / 1000).toFixed(1)}ms`;
}

export function fmtUptime(sec) {
  if (sec == null) return '—';
  if (sec >= 3600) return `${Math.floor(sec / 3600)}h${String(Math.floor((sec % 3600) / 60)).padStart(2, '0')}m`;
  if (sec >= 60) return `${Math.floor(sec / 60)}m${String(sec % 60).padStart(2, '0')}s`;
  return `${sec}s`;
}

// Heading arrow for a yaw in degrees CCW from world +X, snapped to 8 points.
const ARROWS = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘'];
export function headingArrow(yaw) {
  if (yaw == null) return '•';
  const idx = ((Math.round(yaw / 45) % 8) + 8) % 8;
  return ARROWS[idx];
}

// Pads/truncates to an exact column count. Every panel is laid out on a
// character grid, so a string that is one column too long silently wraps and
// shifts every row below it.
export function fit(text, width) {
  if (text.length > width) return text.slice(0, width);
  return text + ' '.repeat(width - text.length);
}
