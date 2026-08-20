// theme.js — the dashboard's colour vocabulary.
//
// Hex, not ANSI names: the C++ dashboard is limited to the 16 basic colours,
// which is why its latency sparkline is one flat cyan no matter how bad the
// link is. Truecolour lets severity carry meaning instead. Terminals without
// truecolour degrade to their nearest palette entry via Ink/chalk.

export const C = {
  bg: '#0b0e14',
  rule: '#1f2430',   // borders, off-cells, graph axes
  field: '#1a1f2b',  // arena background dots
  dim: '#565b66',    // labels, secondary text
  fg: '#bfbdb6',     // body text
  white: '#e6e1cf',  // headings, selected robot
  cyan: '#39bae6',   // primary accent; forward motor
  cyanBright: '#73d0ff',
  yellow: '#ffcc66', // robot ids, warnings
  green: '#aad94c',  // healthy
  orange: '#ff8f40', // degraded
  red: '#f07178',    // lost / critical
  magenta: '#d2a6ff' // reverse motor
};

// Severity bands. The thresholds are the interesting part of the design:
// end-to-end keypress→motor latency is ~4ms by design (docs/architecture.md),
// so 4.5ms is "as expected", 8ms is "something is retrying", above that the
// link is degraded.
export function latencyColor(us) {
  if (!us) return C.dim;
  if (us < 4500) return C.green;
  if (us < 8000) return C.yellow;
  return C.orange;
}

// A fresh 4xAAA pack is ~6.4V; the 3pi+ browns out well before it reaches
// zero, so the meaningful band is narrow and the warning has to be early.
export function batteryColor(v) {
  if (v == null) return C.dim;
  if (v >= 5.0) return C.green;
  if (v >= 4.6) return C.yellow;
  return C.red;
}
