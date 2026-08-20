#!/usr/bin/env node
// cli.js — entry point.
//
//   node src/cli.js                 live: spawns tools/build/swarm_telemetry_json
//   node src/cli.js --demo          synthetic swarm, no hardware needed
//   node src/cli.js --no-vision     skip the camera (hub telemetry only)
//   node src/cli.js --producer PATH point at a producer binary elsewhere
//
// Sends no motor commands — like swarm_dashboard it is a pure observer and
// is safe to run alongside a demo that is driving the robots.

import { render } from 'ink';
import { html } from './html.js';
import { App } from './app.js';
import { TelemetrySource, DemoSource, DEFAULT_PRODUCER } from './telemetry.js';

const argv = process.argv.slice(2);
const has = (flag) => argv.includes(flag);
const valueOf = (flag, fallback) => {
  const i = argv.indexOf(flag);
  return i >= 0 && i + 1 < argv.length ? argv[i + 1] : fallback;
};

if (has('--help') || has('-h')) {
  process.stdout.write(`swarm-dashboard-ink

  --demo             synthetic swarm; no dongle, hub or camera required
  --no-vision        don't open the camera (hub telemetry only)
  --producer PATH    producer binary (default: ${DEFAULT_PRODUCER})
  --robots N         demo mode only: how many robots to synthesise (default 6)
  -h, --help         this message
`);
  process.exit(0);
}

const source = has('--demo')
  ? new DemoSource({ robots: Number(valueOf('--robots', 6)) })
  : new TelemetrySource({
      producerPath: valueOf('--producer', DEFAULT_PRODUCER),
      noVision: has('--no-vision')
    });

// Alternate screen buffer: the dashboard gets the whole terminal and the
// user's scrollback is handed back untouched on exit. Ink has no notion of
// this, so it is bracketed by hand around render().
const ALT_ON = '\x1b[?1049h\x1b[H';
const ALT_OFF = '\x1b[?1049l';
const CURSOR_HIDE = '\x1b[?25l';
const CURSOR_SHOW = '\x1b[?25h';

let restored = false;
function restore() {
  if (restored) return;
  restored = true;
  source.stop();
  process.stdout.write(CURSOR_SHOW + ALT_OFF);
}

if (process.stdout.isTTY) process.stdout.write(ALT_ON + CURSOR_HIDE);

const app = render(html`<${App} source=${source} onExit=${restore} />`, {
  exitOnCtrlC: false  // handled in App so the terminal is restored first
});

// SIGTERM/SIGHUP as well as a clean exit: leaving the terminal in the
// alternate buffer with a hidden cursor makes the shell look broken.
for (const sig of ['SIGINT', 'SIGTERM', 'SIGHUP']) {
  process.on(sig, () => { restore(); app.unmount(); });
}
process.on('exit', restore);

await app.waitUntilExit();
restore();
