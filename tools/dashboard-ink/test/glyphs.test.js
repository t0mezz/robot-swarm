// Tests for the pure rendering functions and the layout budget.
//
// Uses node:test, so there is no test-framework dependency — the same choice
// tests/test_protocol.cpp makes for the C++ side.
//
//   npm test        (from tools/dashboard-ink)

import test from 'node:test';
import assert from 'node:assert/strict';

import {
  brailleGraph, blockSpark, smoothBar, dualDrive, mergeRuns,
  fmtMs, fmtUptime, headingArrow, fit
} from '../src/glyphs.js';
import { computeLayout } from '../src/app.js';
import { rosterLayout } from '../src/components/Roster.js';
import { latencyColor, batteryColor, C } from '../src/theme.js';

test('brailleGraph returns exactly the requested grid', () => {
  const g = brailleGraph([1, 2, 3, 4, 5], 10, 4, 5);
  assert.equal(g.length, 4);
  for (const line of g) assert.equal([...line].length, 10);
  for (const line of g)
    for (const ch of line)
      assert.ok(ch.codePointAt(0) >= 0x2800 && ch.codePointAt(0) <= 0x28ff,
        `${JSON.stringify(ch)} is not a braille cell`);
});

test('brailleGraph puts the newest sample at the right edge', () => {
  // A series that is flat then spikes should light the rightmost column at
  // the top row and leave the leftmost column blank at that row.
  const series = [...new Array(20).fill(0), 100];
  const g = brailleGraph(series, 12, 4, 100);
  assert.equal(g[0][0], '⠀', 'left edge of the top row should be empty');
  assert.notEqual(g[0].at(-1), '⠀', 'right edge of the top row should be lit');
});

test('brailleGraph joins consecutive samples into a continuous line', () => {
  // A single step from bottom to top must not leave a gap in the middle rows:
  // every row should carry at least one lit cell.
  const g = brailleGraph([0, 100], 2, 4, 100);
  for (const [i, line] of g.entries())
    assert.notEqual(line, '⠀'.repeat(2), `row ${i} is empty — the trace is broken`);
});

test('brailleGraph tolerates degenerate inputs', () => {
  assert.deepEqual(brailleGraph([], 0, 4, 100), []);
  assert.deepEqual(brailleGraph([], 10, 0, 100), []);
  assert.equal(brailleGraph([1], 4, 2, 0).length, 2);   // maxVal 0 must not divide by zero
});

test('blockSpark resolves 8 levels and right-aligns a short series', () => {
  assert.equal(blockSpark([0, 25, 50, 75, 100], 5, 100), ' ▂▄▆█');
  assert.equal(blockSpark([100], 4, 100), '   █');
  assert.equal(blockSpark([50, 50], 2, 100), '▄▄');
});

test('smoothBar is sub-cell accurate and always fills its width', () => {
  const half = smoothBar(0.5, 8);
  assert.equal(half.bar.length + half.track.length, 8);
  assert.equal(half.bar, '████');

  // 1/16th of an 8-cell bar is half a cell: a partial glyph, not a full one.
  const sliver = smoothBar(1 / 16, 8);
  assert.equal(sliver.bar, '▌');
  assert.equal(sliver.bar.length + sliver.track.length, 8);

  const full = smoothBar(1, 8);
  assert.equal(full.bar, '████████');
  assert.equal(full.track, '');

  // Out-of-range values clamp rather than overrun the column.
  assert.equal(smoothBar(3, 6).bar.length, 6);
  assert.equal(smoothBar(-1, 6).bar, '');
});

test('dualDrive encodes left in the upper half and right in the lower', () => {
  const cells = dualDrive(127, 0, 11);
  assert.equal(cells.length, 11);
  assert.equal(cells[5].ch, '┃', 'centre column is the divider');
  // Full forward on the left motor only: upper half-blocks to the right.
  assert.equal(cells[6].ch, '▀');
  assert.equal(cells[6].color, C.cyan);

  const right = dualDrive(0, 127, 11);
  assert.equal(right[6].ch, '▄');

  // Both motors same direction and magnitude merge into a full block.
  assert.equal(dualDrive(127, 127, 11)[6].ch, '█');

  // Reverse grows to the LEFT of the divider and switches colour.
  const rev = dualDrive(-127, -127, 11);
  assert.equal(rev[4].ch, '█');
  assert.equal(rev[4].color, C.magenta);
  assert.equal(rev[6].ch, '·', 'nothing lit on the forward side');
});

test('dualDrive lights a cell for any nonzero command', () => {
  // The C++ meter floored small values to zero cells, making a crawling
  // robot look stopped. One cell minimum is the fix.
  const cells = dualDrive(1, 0, 21);
  assert.ok(cells.some((c) => c.ch === '▀'), 'a command of 1 must be visible');
  assert.ok(dualDrive(0, 0, 21).every((c) => c.ch === '·' || c.ch === '┃'),
    'a stopped robot must show no lit cells');
});

test('mergeRuns collapses adjacent cells with identical styling', () => {
  // A stopped robot is one uniform run: the divider and the off-cells differ
  // in glyph but share a colour, and merging is by style, not by character.
  const idle = mergeRuns(dualDrive(0, 0, 21));
  assert.equal(idle.length, 1);
  assert.equal(idle[0].text, `${'·'.repeat(10)}┃${'·'.repeat(10)}`);

  // A driving robot splits into styled runs, but never loses a column.
  const moving = mergeRuns(dualDrive(90, -40, 21));
  assert.ok(moving.length > 1, 'differing motor directions must not merge');
  assert.equal(moving.map((r) => r.text).join('').length, 21);
});

test('formatting helpers', () => {
  assert.equal(fmtMs(3420), '3.4ms');
  assert.equal(fmtMs(0), '—');
  assert.equal(fmtMs(null), '—');
  assert.equal(fmtUptime(45), '45s');
  assert.equal(fmtUptime(1277), '21m17s');
  assert.equal(fmtUptime(7325), '2h02m');
  assert.equal(fit('abcdef', 3), 'abc');
  assert.equal(fit('ab', 4), 'ab  ');
});

test('headingArrow snaps yaw to 8 compass points and wraps', () => {
  assert.equal(headingArrow(0), '→');
  assert.equal(headingArrow(90), '↑');
  assert.equal(headingArrow(180), '←');
  assert.equal(headingArrow(-90), '↓');
  assert.equal(headingArrow(360), '→', 'wraps past a full turn');
  assert.equal(headingArrow(-180), '←', 'negative half turn is the same as +180');
  assert.equal(headingArrow(null), '•');
});

test('severity bands match the documented ~4ms design latency', () => {
  assert.equal(latencyColor(3400), C.green);
  assert.equal(latencyColor(6000), C.yellow);
  assert.equal(latencyColor(12000), C.orange);
  assert.equal(batteryColor(5.2), C.green);
  assert.equal(batteryColor(4.7), C.yellow);
  assert.equal(batteryColor(4.2), C.red);
  assert.equal(batteryColor(null), C.dim);
});

test('computeLayout never asks for more rows than the terminal has', () => {
  // The invariant the whole TUI rests on: a panel budget that overruns the
  // terminal scrolls the frame and shifts every subsequent redraw.
  for (const rows of [24, 30, 40, 48, 60, 80]) {
    for (const cols of [80, 100, 112, 132, 160, 200]) {
      for (const n of [0, 1, 6, 32]) {
        const L = computeLayout(cols, rows, n);
        const chrome = 1 + 1 + 1 + L.logH + 1;   // status, rule, rule, log, hints
        assert.equal(chrome + L.bodyH, rows, `body budget wrong at ${cols}x${rows}/${n}`);

        const leftUsed = L.rosterH + 1 + L.graphH;
        assert.ok(leftUsed <= L.bodyH, `left column overruns at ${cols}x${rows}/${n}`);

        const rightUsed = L.showRight ? L.arenaH + L.focusH + (L.showFocus ? 1 : 0) : 0;
        assert.ok(rightUsed <= L.bodyH, `right column overruns at ${cols}x${rows}/${n}`);

        assert.ok(L.leftW + L.rightW + L.gap === cols, `columns must tile the width`);
        assert.ok(L.leftW > 0, 'left column must never vanish');
      }
    }
  }
});

test('a roster row is always exactly as wide as its panel', () => {
  // The grid invariant: header and rows must sum to the panel width for every
  // width the layout can hand them, or the row wraps and shifts the rows below.
  const BAT_BAR = 9, DRIVE = 21;
  for (let width = 40; width <= 200; width++) {
    const L = rosterLayout(width);
    const used = 3 + 4 + 9 + L.trend + 2 + (6 + BAT_BAR)
      + (L.showDrive ? 2 + DRIVE : 0)
      + (L.showUptime ? 2 + 8 : 0);
    assert.equal(used, width, `row is ${used} columns in a ${width}-column panel`);
    assert.ok(L.trend >= 6, `trend collapsed at width ${width}`);
  }
});

test('roster columns drop in priority order as width shrinks', () => {
  assert.deepEqual(
    ['showDrive', 'showUptime'].map((k) => rosterLayout(160)[k]), [true, true]);
  const narrow = rosterLayout(70);
  assert.equal(narrow.showUptime, false, 'uptime goes first');
  assert.equal(narrow.showDrive, true, 'drive outlives uptime');
  assert.equal(rosterLayout(50).showDrive, false);
});

test('computeLayout drops the right column on narrow terminals', () => {
  assert.equal(computeLayout(90, 40, 6).showRight, false);
  assert.equal(computeLayout(90, 40, 6).leftW, 90);
  assert.equal(computeLayout(160, 48, 6).showRight, true);
});

test('computeLayout caps the swarm graph instead of stretching it', () => {
  const tall = computeLayout(160, 80, 2);
  assert.ok(tall.graphH <= 12, 'graph must not grow without limit');
});
