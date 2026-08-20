// Arena.js — overhead minimap of the tracked robots.
//
// Two changes from the C++ version's minimap. It shows heading, not just
// presence: each robot is an arrow snapped to 8 compass points from its
// ArUco yaw, so "facing the wrong way" is visible without reading degrees
// out of a table. And it is sized from the panel instead of being a fixed
// 14x14 grid inside a fixed 40-column box, which is what left the old one a
// mostly-empty dark rectangle.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C } from '../theme.js';
import { headingArrow } from '../glyphs.js';

// Cells are 2 terminal columns wide and 1 row tall: a monospace cell is
// roughly twice as tall as it is wide, so a 2-wide cell reads as square and
// the arena keeps the camera's 1:1 aspect.
const CELL_W = 2;

export function Arena({ vision, robots, selectedId, width, height }) {
  // The camera frame is square (2048x2048), so the grid has to be square
  // too — a 29x18 grid would stretch the arena horizontally and put robots
  // where they aren't. Fit a square to the smaller dimension and pad the
  // leftover columns, rather than filling the panel and lying about position.
  const maxW = Math.max(4, Math.floor((width - 4) / CELL_W));
  const maxH = Math.max(3, height - 3);
  const n = Math.min(maxW, maxH);
  const gridW = n, gridH = n;
  const padLeft = Math.floor((maxW - n) / 2) * CELL_W;

  const byId = new Map(robots.map((r) => [r.id, r]));
  const grid = Array.from({ length: gridH }, () => new Array(gridW).fill(null));

  const fw = vision.w || 2048;
  const fh = vision.h || 2048;
  for (const p of vision.robots ?? []) {
    // Prefer the raw pixel centroid: it exists whether or not a homography
    // is loaded, whereas x/y switch between mm and pixels depending on
    // calibration and would silently rescale the map.
    const sx = p.px || p.x;
    const sy = p.py || p.y;
    const col = Math.min(gridW - 1, Math.max(0, Math.floor((sx / fw) * gridW)));
    const row = Math.min(gridH - 1, Math.max(0, Math.floor((sy / fh) * gridH)));
    grid[row][col] = p;
  }

  const title = ' ARENA ';
  const right = vision.ok ? ` ${fw}² ` : ' no camera ';
  const fill = Math.max(0, width - 2 - 2 - title.length - right.length);

  return html`
    <${Box} flexDirection="column" width=${width}>
      <${Box}>
        <${Text} color=${C.rule}>╭─<//>
        <${Text} color=${C.white} bold>${title}<//>
        <${Text} color=${C.rule}>${'─'.repeat(fill)}<//>
        <${Text} color=${C.dim}>${right}<//>
        <${Text} color=${C.rule}>╮<//>
      <//>
      ${grid.map((gridRow, y) => html`
        <${Box} key=${y}>
          <${Text} color=${C.rule}>│ <//>
          <${Text}>${' '.repeat(padLeft)}<//>
          ${gridRow.map((p, x) => {
            if (!p) return html`<${Text} key=${x} color=${C.field}> ·<//>`;
            const r = byId.get(p.id);
            const col = r?.lost ? C.red
              : r?.batteryV != null && r.batteryV < 4.6 ? C.yellow
              : p.id === selectedId ? C.white : C.cyanBright;
            return html`
              <${Text} key=${x} color=${col} bold=${p.id === selectedId}>
                ${headingArrow(p.yaw)}${String(p.id % 10)}
              <//>
            `;
          })}
          <${Text} color=${C.rule}>${' '.repeat(Math.max(0, width - 4 - padLeft - gridW * CELL_W))} │<//>
        <//>
      `)}
      <${Box}>
        <${Text} color=${C.rule}>╰${'─'.repeat(Math.max(0, width - 2))}╯<//>
      <//>
      <${Box}>
        <${Text} color=${C.cyanBright}> ↑→<//>
        <${Text} color=${C.dim}> heading   <//>
        <${Text} color=${C.yellow}>●<//>
        <${Text} color=${C.dim}> low batt   ${vision.robots?.length ?? 0} tracked<//>
      <//>
    <//>
  `;
}
