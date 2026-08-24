// StatusBar.js — one row carrying every "is the rig healthy" fact.
//
// Replaces the C++ dashboard's three-row centred title banner, which spent
// 6% of a 48-row terminal on a string that never changes. Link, camera,
// robot count, swarm latency and the active warnings all fit in the row that
// title used to sit in.
//
// The row budgets itself. Ink shrinks overflowing Text nodes rather than
// clipping them, so a status bar wider than the terminal comes out with
// letters missing from the middle of words ("swar / telemetry"). Chips are
// therefore measured and dropped whole, lowest priority first.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C } from '../theme.js';
import { fmtMs } from '../glyphs.js';

const GAP = '   ';

// A chip is a list of [text, color, bold] triples plus a priority: lower
// numbers survive longer. Priority is about triage, not layout — a warning
// outranks the frame rate because it is the reason you looked at the screen.
function chips({ state, paused }) {
  const robots = state.robots ?? [];
  const live = robots.filter((r) => !r.lost);
  const lost = robots.length - live.length;
  const lowBatt = robots.filter((r) => r.batteryV != null && r.batteryV < 4.6).length;

  const avg = state.avgLat.length ? state.avgLat[state.avgLat.length - 1] : 0;
  const peak = state.avgLat.length ? Math.max(...state.avgLat) : 0;

  const warnings = [];
  if (lost) warnings.push(`${lost} lost`);
  if (lowBatt) warnings.push(`${lowBatt} low batt`);

  const out = [
    { priority: 0, segs: [[' swarm', C.white, true], [' / ', C.rule], ['telemetry', C.dim]] },
    { priority: 2, segs: [['●', state.hub ? C.green : C.red], [' hub', C.fg],
                          [state.hub ? ' linked' : ' offline', C.dim]] },
    // The source tag is not decoration: "no vision" almost always means
    // something else holds the camera, and seeing `hub` vs `own` is what tells
    // you whether this process is the one locking demos out.
    { priority: 4, segs: [['●', state.vision.ok ? C.green : C.red], [' cam', C.fg],
                          [state.vision.ok ? ` ${state.vision.fps.toFixed(0)}fps` : ' none', C.dim],
                          [visionTag(state.vision), C.rule]] },
    { priority: 3, segs: [['●', C.cyan], [` ${live.length}`, C.white, true],
                          [`/${robots.length} robots`, C.dim]] },
    { priority: 5, segs: [['avg ', C.dim], [fmtMs(avg), C.fg],
                          ['  peak ', C.dim], [fmtMs(peak), C.fg]] }
  ];
  if (warnings.length)
    out.push({ priority: 1, segs: [[`⚠ ${warnings.join(' · ')}`, C.orange]] });
  if (paused)
    out.push({ priority: 0, segs: [[' PAUSED ', C.bg, true, C.yellow]] });
  return out;
}

// 'hub' = poses arrive from whichever tool owns the camera; 'own' = this
// process opened it (and demos will fail to start while it runs).
function visionTag(vision) {
  if (!vision.ok) return '';
  if (vision.source === 'hub') return '·hub';
  if (vision.source === 'camera') return '·own';
  return '';
}

const chipWidth = (chip) => chip.segs.reduce((n, [t]) => n + t.length, 0);

export function StatusBar({ state, width, paused }) {
  const all = chips({ state, paused });

  // Greedy by priority, then rendered back in declaration order so the row's
  // reading order never shuffles as chips come and go.
  const order = [...all.keys()].sort((a, b) => all[a].priority - all[b].priority);
  const keep = new Set();
  let used = 0;
  for (const i of order) {
    const cost = chipWidth(all[i]) + (keep.size ? GAP.length : 0);
    if (used + cost > width) continue;
    used += cost;
    keep.add(i);
  }

  const visible = all.filter((_, i) => keep.has(i));

  return html`
    <${Box} width=${width} flexDirection="row">
      ${visible.map((chip, ci) => html`
        <${Box} key=${ci}>
          ${ci > 0 ? html`<${Text}>${GAP}<//>` : null}
          ${chip.segs.map(([text, color, bold, bg], si) => html`
            <${Text} key=${si} color=${color} bold=${!!bold} backgroundColor=${bg}>${text}<//>
          `)}
        <//>
      `)}
    <//>
  `;
}
