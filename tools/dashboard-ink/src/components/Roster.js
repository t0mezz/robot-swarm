// Roster.js — one line per robot.
//
// The C++ dashboard spends three rows per robot (meters, a stats line, a
// blank), so six robots already overflow a 48-row terminal and the code has
// a "shrink the bars once there are more than 8" branch to cope. Here every
// robot is exactly one row and nothing shrinks: 32 robots fit in 32 rows.
//
// What made that fit: the two full-width bipolar motor meters collapse into
// one half-block meter (see dualDrive), and the per-robot min/avg/max line
// moves into the focus panel, where it is read on demand rather than printed
// 32 times.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C, latencyColor, batteryColor } from '../theme.js';
import { blockSpark, smoothBar, dualDrive, mergeRuns, fmtMs, fmtUptime, fit } from '../glyphs.js';

const BAT_BAR = 9;
const DRIVE = 21;
const BATTERY_FULL_V = 6.5;   // meter full-scale; matches swarm_dashboard.cpp

// Never wider than the 8-column uptime slot it replaces — "lost 7.4s" is 9
// and pushes the row past the panel edge, which wraps and shifts the grid.
function fmtLost(ms) {
  const s = Math.round(ms / 1000);
  return s < 60 ? `lost ${s}s` : `lost ${Math.min(99, Math.round(s / 60))}m`;
}

// Column widths, in the order Row renders them. Kept as named constants
// because the header and the rows have to agree exactly: a header that is
// one column wider than its rows makes every value look subtly misfiled.
const W_GUTTER = 3;       // " ▸ "
const W_ID = 4;           // "R12 "
const W_LAT = 9;          // 8 right-aligned + 1 space
const W_GAP = 2;
const W_BATTERY = 6 + BAT_BAR;   // "5.12V " + meter
const W_UPTIME = 8;
const MIN_TREND = 6;

const BASE = W_GUTTER + W_ID + W_LAT + W_GAP + W_BATTERY;
const WITH_DRIVE = BASE + W_GAP + DRIVE;
const WITH_UPTIME = WITH_DRIVE + W_GAP + W_UPTIME;

// Resolved against the actual panel width, so the roster degrades by dropping
// whole columns rather than letting rows wrap — a wrapped row shifts every
// row beneath it and the grid stops lining up. `trend` absorbs all remaining
// slack, so a row is always exactly `width` columns.
export function rosterLayout(width) {
  const showUptime = width >= WITH_UPTIME + MIN_TREND;
  const showDrive = showUptime || width >= WITH_DRIVE + MIN_TREND;
  const fixed = showUptime ? WITH_UPTIME : showDrive ? WITH_DRIVE : BASE;
  return { showDrive, showUptime, trend: Math.max(MIN_TREND, width - fixed) };
}

function Header({ width, layout, sortBy }) {
  const label = (text, active) => html`
    <${Text} color=${active ? C.cyanBright : C.dim}>${text}<//>
  `;
  // The active sort column is highlighted, so `s` shows what it did without
  // needing a separate indicator.
  return html`
    <${Box} width=${width}>
      <${Text} color=${C.dim}>${' '.repeat(W_GUTTER)}${fit('ID', W_ID)}<//>
      ${label(fit('LATENCY'.padStart(8), W_LAT), sortBy === 'latency')}
      <${Text} color=${C.dim}>${fit('TREND·15s', layout.trend)}${' '.repeat(W_GAP)}<//>
      ${label(fit('BATTERY', W_BATTERY), sortBy === 'battery')}
      ${layout.showDrive
        ? html`<${Text} color=${C.dim}>${' '.repeat(W_GAP)}${fit('DRIVE  L▀ R▄', DRIVE)}<//>`
        : null}
      ${layout.showUptime
        ? html`<${Text} color=${C.dim}>${' '.repeat(W_GAP)}${fit('UPTIME', W_UPTIME)}<//>`
        : null}
    <//>
  `;
}

function Row({ robot, width, layout, selected, maxLat }) {
  const r = robot;
  const latCol = r.lost ? C.dim : latencyColor(r.latencyUs);
  const batCol = batteryColor(r.batteryV);
  const idCol = r.lost ? C.red : selected ? C.white : C.yellow;

  const bat = r.batteryV == null
    ? null
    : smoothBar(r.batteryV / BATTERY_FULL_V, BAT_BAR);

  return html`
    <${Box} width=${width}>
      <${Text} color=${C.cyanBright} bold>${selected ? ' ▸ ' : '   '}<//>
      <${Text} color=${idCol} bold>${`R${r.id}`.padEnd(4)}<//>

      <${Text} color=${latCol}>${(r.lost ? '—' : fmtMs(r.latencyUs)).padStart(8)}<//>
      <${Text}>${' '}<//>
      <${Text} color=${latCol}>${
        r.lost ? '░'.repeat(layout.trend) : blockSpark(r.hist, layout.trend, maxLat)
      }<//>
      <${Text}>${'  '}<//>

      ${bat
        ? html`
            <${Text} color=${batCol}>${r.batteryV.toFixed(2)}V <//>
            <${Text} color=${batCol}>${bat.bar}<//>
            <${Text} color=${C.rule}>${bat.track}<//>
          `
        : html`
            <${Text} color=${C.dim}>${'  —   '}<//>
            <${Text} color=${C.rule}>${'░'.repeat(BAT_BAR)}<//>
          `}

      ${layout.showDrive
        ? html`
            <${Text}>${'  '}<//>
            ${mergeRuns(dualDrive(r.motorL, r.motorR, DRIVE)).map((run, i) => html`
              <${Text} key=${i} color=${run.color} backgroundColor=${run.bg}>${run.text}<//>
            `)}
          `
        : null}

      ${layout.showUptime
        ? html`
            <${Text}>${'  '}<//>
            ${r.lost
              ? html`<${Text} color=${C.red}>${fmtLost(r.ageMs).padEnd(8)}<//>`
              : html`<${Text} color=${C.dim}>${fmtUptime(r.uptime).padEnd(8)}<//>`}
          `
        : null}
    <//>
  `;
}

export function Roster({ robots, width, selectedId, sortBy }) {
  const layout = rosterLayout(width);
  // One shared full-scale across every trend line, so a tall trace means a
  // slow robot rather than an aggressively autoscaled one. Floored at 5ms
  // (LATENCY_MAX_US in the C++ dashboard) so an all-healthy swarm doesn't
  // magnify sub-millisecond noise into alarming spikes.
  const maxLat = Math.max(5000, ...robots.flatMap((r) => (r.lost ? [] : r.hist))) * 1.15;

  return html`
    <${Box} flexDirection="column" width=${width}>
      <${Header} width=${width} layout=${layout} sortBy=${sortBy} />
      ${robots.length === 0
        ? html`<${Text} color=${C.dim}>   waiting for robots to announce…<//>`
        : robots.map((r) => html`
            <${Row} key=${r.id} robot=${r} width=${width} layout=${layout}
                    selected=${r.id === selectedId} maxLat=${maxLat} />
          `)}
    <//>
  `;
}
