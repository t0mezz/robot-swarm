// telemetry.js — owns the C++ producer process and the rolling state the UI
// renders from.
//
// The dashboard never speaks the swarm wire protocol itself. It spawns
// tools/build/swarm_telemetry_json, which is the only thing here that links
// SwarmClient and ArucoTracker, and reads newline-delimited JSON from its
// stdout. That keeps the protocol's canonical C++ definition the single
// source of truth — it is already mirrored by hand in three places
// (lib/SwarmProtocol/protocol.h, lib/SwarmClient/SwarmClient.h,
// src/robots/robot_uart.py) and a fourth, in JavaScript, would be one more
// thing to forget to update.

import { spawn } from 'node:child_process';
import { EventEmitter } from 'node:events';
import { existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));

// tools/dashboard-ink/src -> tools/build. The Makefile puts every PC tool in
// that one directory, so this holds regardless of the caller's cwd.
export const DEFAULT_PRODUCER = resolve(HERE, '../../build/swarm_telemetry_json');

export const HISTORY = 60;        // samples kept per metric (60 x tick = 15s)
export const LOST_AFTER_MS = 5000;
const LOG_MAX = 200;
const RESPAWN_DELAY_MS = 2000;

function emptyState() {
  return {
    producer: 'starting',   // starting | running | failed | stopped
    producerError: null,
    hub: false,
    vision: { ok: false, source: 'none', fps: 0, w: 0, h: 0, robots: [] },
    robots: [],
    avgLat: [],
    log: [],
    notes: [],              // producer stderr, surfaced instead of swallowed
    intervalMs: 250,
    ticks: 0,
    lastTickAt: 0
  };
}

export class TelemetrySource extends EventEmitter {
  constructor({ producerPath = DEFAULT_PRODUCER, noVision = false, ownCamera = false } = {}) {
    super();
    this.producerPath = producerPath;
    this.noVision = noVision;
    // Off by default on purpose: the producer subscribes to whichever tool
    // owns the camera rather than opening it, so a dashboard left running
    // never stops a vision demo from starting.
    this.ownCamera = ownCamera;
    this.state = emptyState();
    this.hist = new Map();      // robot id -> latency samples
    this.proc = null;
    this.stopped = false;
    this.respawnTimer = null;
  }

  start() {
    if (!existsSync(this.producerPath)) {
      this.state.producer = 'failed';
      this.state.producerError =
        `not found: ${this.producerPath}\nBuild it with:  cd tools && make build/swarm_telemetry_json`;
      this.emit('update', this.state);
      return;
    }
    this.spawnProducer();
  }

  spawnProducer() {
    const args = [];
    if (this.noVision) args.push('--no-vision');
    if (this.ownCamera) args.push('--camera');
    this.proc = spawn(this.producerPath, args, { stdio: ['ignore', 'pipe', 'pipe'] });

    let buf = '';
    this.proc.stdout.setEncoding('utf8');
    this.proc.stdout.on('data', (chunk) => {
      buf += chunk;
      // The producer writes each object with a single write(), but a pipe can
      // still split one mid-line, so only complete lines are parsed and the
      // remainder is carried into the next chunk.
      let nl;
      while ((nl = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, nl);
        buf = buf.slice(nl + 1);
        if (line.trim()) this.handleLine(line);
      }
      if (buf.length > 1 << 20) buf = '';  // desynced beyond recovery; resync on the next newline
    });

    this.proc.stderr.setEncoding('utf8');
    this.proc.stderr.on('data', (chunk) => {
      for (const line of chunk.split('\n')) {
        if (!line.trim()) continue;
        this.state.notes.push({ at: Date.now(), text: line.trim() });
        if (this.state.notes.length > 20) this.state.notes.shift();
      }
      this.emit('update', this.state);
    });

    this.proc.on('error', (err) => {
      this.state.producer = 'failed';
      this.state.producerError = err.message;
      this.emit('update', this.state);
    });

    this.proc.on('exit', (code, signal) => {
      this.proc = null;
      if (this.stopped) return;
      this.state.producer = 'failed';
      this.state.producerError = `producer exited (${signal ?? `code ${code}`}) — restarting`;
      this.state.hub = false;
      this.emit('update', this.state);
      // Keep the UI alive across a producer crash: the swarm outlives any one
      // observer, and a dashboard that dies with its data source is useless
      // exactly when something has gone wrong.
      this.respawnTimer = setTimeout(() => this.spawnProducer(), RESPAWN_DELAY_MS);
    });

    this.state.producer = 'running';
    this.state.producerError = null;
    this.emit('update', this.state);
  }

  handleLine(line) {
    let msg;
    try {
      msg = JSON.parse(line);
    } catch {
      return;  // partial or corrupted line; the next one resyncs
    }
    if (msg.type === 'hello') {
      this.state.intervalMs = msg.intervalMs ?? 250;
      return;
    }
    if (msg.type !== 'tick') return;
    this.applyTick(msg);
    this.emit('update', this.state);
  }

  applyTick(msg) {
    const s = this.state;
    s.hub = !!msg.hub;
    s.vision = msg.vision?.ok
      ? { ok: true, source: msg.vision.source ?? 'camera',
          fps: msg.vision.fps ?? 0, w: msg.vision.w ?? 0, h: msg.vision.h ?? 0,
          robots: msg.vision.robots ?? [] }
      : { ok: false, source: msg.vision?.source ?? 'none', fps: 0, w: 0, h: 0, robots: [] };

    const seen = new Set();
    s.robots = (msg.robots ?? []).map((r) => {
      seen.add(r.id);
      let hist = this.hist.get(r.id);
      if (!hist) {
        // A robot that announces mid-session starts with a flat window rather
        // than an empty one, so its trace shares the same time base as
        // everyone else's instead of growing in from the right edge.
        hist = new Array(HISTORY).fill(r.latencyUs || 0);
        this.hist.set(r.id, hist);
      }
      hist.push(r.latencyUs || 0);
      if (hist.length > HISTORY) hist.shift();
      const pose = s.vision.robots.find((p) => p.id === r.id) ?? null;
      return { ...r, hist, pose, lost: r.ageMs > LOST_AFTER_MS };
    });
    for (const id of [...this.hist.keys()]) if (!seen.has(id)) this.hist.delete(id);

    // Swarm mean over robots that still have a link. Including lost robots
    // would drag the mean toward whatever they last reported and mask a real
    // change in the robots that are actually talking.
    const live = s.robots.filter((r) => !r.lost);
    if (live.length) {
      s.avgLat.push(live.reduce((a, r) => a + (r.latencyUs || 0), 0) / live.length);
      if (s.avgLat.length > HISTORY) s.avgLat.shift();
    }

    const now = Date.now();
    for (const e of msg.log ?? []) {
      s.log.push({ id: e.id, field: e.field, text: e.text, at: now - (e.ageMs ?? 0) });
    }
    if (s.log.length > LOG_MAX) s.log.splice(0, s.log.length - LOG_MAX);

    s.ticks++;
    s.lastTickAt = now;
  }

  stop() {
    this.stopped = true;
    if (this.respawnTimer) clearTimeout(this.respawnTimer);
    if (this.proc) this.proc.kill('SIGTERM');
  }
}

// ── Demo source ──────────────────────────────────────────────────
// Synthetic swarm, no hardware, no hub. Same contract as TelemetrySource so
// the UI cannot tell them apart — this is how the layout gets worked on
// without occupying the robots (and how a reviewer sees a full 6-robot
// screen when only one robot is powered on). Mirrors the precedent set by
// tools/vision/demo_hud_preview.cpp.

export class DemoSource extends EventEmitter {
  constructor({ robots = 6, intervalMs = 250 } = {}) {
    super();
    this.state = emptyState();
    this.state.producer = 'running';
    this.state.intervalMs = intervalMs;
    this.intervalMs = intervalMs;
    this.timer = null;
    this.t = 0;
    this.ids = [0, 3, 7, 8, 12, 19, 21, 25].slice(0, robots);
    this.hist = new Map(this.ids.map((id) => [id, new Array(HISTORY).fill(3500 + id * 120)]));
    this.phase = new Map(this.ids.map((id) => [id, id * 0.7]));
  }

  start() {
    this.timer = setInterval(() => this.tick(), this.intervalMs);
    this.tick();
  }

  tick() {
    const s = this.state;
    this.t += this.intervalMs / 1000;
    s.hub = true;

    const visionRobots = [];
    s.robots = this.ids.map((id, i) => {
      const ph = this.phase.get(id) + this.t * (0.25 + i * 0.05);
      // id 19 is deliberately a dead link and id 7 a flat battery, so the
      // states the layout has to handle are always on screen in demo mode.
      const lost = id === 19 && this.t > 6;
      const base = 3200 + i * 900 + Math.sin(ph * 0.6) * 700;
      const latencyUs = lost ? 0 : Math.max(400, Math.round(base + Math.sin(ph * 7) * 350));

      const hist = this.hist.get(id);
      hist.push(latencyUs);
      if (hist.length > HISTORY) hist.shift();

      const batteryV = id === 7 ? 4.28 + Math.sin(this.t * 0.2) * 0.03
                                : 5.35 - (i * 0.09) - Math.min(0.4, this.t * 0.002);
      const drive = Math.sin(ph * 0.8);
      const turn = Math.sin(ph * 0.35) * 0.6;
      const motorL = lost ? 0 : Math.round(Math.max(-127, Math.min(127, (drive + turn) * 110)));
      const motorR = lost ? 0 : Math.round(Math.max(-127, Math.min(127, (drive - turn) * 110)));

      if (!lost) {
        visionRobots.push({
          id,
          x: 1024 + Math.cos(ph * 0.4 + i) * (300 + i * 90),
          y: 1024 + Math.sin(ph * 0.4 + i) * (300 + i * 90),
          yaw: ((ph * 40 + i * 45) % 360) - 180,
          px: 0, py: 0
        });
      }

      return {
        id, latencyUs, motorL, motorR,
        uptime: Math.round(300 + i * 411 + this.t),
        ageMs: lost ? 7400 : 40,
        flags: 0x10, hasTelemetry: true,
        mac: `A4:CF:12:9B:${String(id * 7 % 256).padStart(2, '0')}:E8`,
        batteryV: id === 19 ? null : Number(batteryV.toFixed(2)),
        hist,
        pose: visionRobots.find((p) => p.id === id) ?? null,
        lost
      };
    });

    s.vision = { ok: true, source: 'demo', fps: 116 + Math.sin(this.t) * 3, w: 2048, h: 2048, robots: visionRobots };

    const live = s.robots.filter((r) => !r.lost);
    if (live.length) {
      s.avgLat.push(live.reduce((a, r) => a + r.latencyUs, 0) / live.length);
      if (s.avgLat.length > HISTORY) s.avgLat.shift();
    }

    if (s.ticks % 12 === 0) {
      const msgs = ['pid_reset l=0 r=0', 'odom drift 12mm', 'announce ack rssi=-54',
                    'turn_cmd yaw=-40', 'bat_low', 'link retry 1/3'];
      s.log.push({
        id: this.ids[s.ticks / 12 % this.ids.length],
        field: 1,
        text: msgs[(s.ticks / 12) % msgs.length],
        at: Date.now()
      });
      if (s.log.length > LOG_MAX) s.log.shift();
    }

    s.ticks++;
    s.lastTickAt = Date.now();
    this.emit('update', s);
  }

  stop() { if (this.timer) clearInterval(this.timer); }
}
