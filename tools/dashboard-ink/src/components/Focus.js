// Focus.js — everything about the one selected robot.
//
// This panel is what lets the roster be one line per robot: min/avg/max,
// jitter, pose, MAC and flags used to be printed for every robot on every
// frame, so they cost 32 rows to say something you only ever want about one
// robot at a time. Here they cost 6 rows total and follow the selection.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C, latencyColor, batteryColor } from '../theme.js';
import { brailleGraph, fmtUptime, headingArrow } from '../glyphs.js';

const SC_STATUS_LOW_BATTERY = 0x04;
const SC_STATUS_ANNOUNCING = 0x08;
const SC_STATUS_BAT_VALID = 0x10;

function flagNames(flags) {
  const out = [];
  if (flags & SC_STATUS_BAT_VALID) out.push('BAT_VALID');
  if (flags & SC_STATUS_LOW_BATTERY) out.push('LOW_BATTERY');
  if (flags & SC_STATUS_ANNOUNCING) out.push('ANNOUNCING');
  return out.length ? out.join(' · ') : 'none';
}

// One bordered row. justifyContent="space-between" pushes the closing border
// to the panel's right edge, so the box closes correctly without every row
// having to measure the visible width of its own coloured, variable-length
// contents — which is exactly the hand-counting the C++ dashboard needs
// visibleLen()/padVisibleRight() for.
function BoxRow({ width, children }) {
  return html`
    <${Box} width=${width} justifyContent="space-between">
      <${Box}>
        <${Text} color=${C.rule}>│ <//>
        ${children}
      <//>
      <${Text} color=${C.rule}>│<//>
    <//>
  `;
}

function Field({ width, label, children }) {
  return html`
    <${BoxRow} width=${width}>
      <${Text} color=${C.dim}>${label.padEnd(6)}  <//>
      ${children}
    <//>
  `;
}

export function Focus({ robot, width, height }) {
  const inner = width - 2;

  if (!robot) {
    return html`
      <${Box} flexDirection="column" width=${width}>
        <${Box}>
          <${Text} color=${C.rule}>╭─ <//>
          <${Text} color=${C.dim}>FOCUS<//>
          <${Text} color=${C.rule}> ${'─'.repeat(Math.max(0, width - 11))}╮<//>
        <//>
        <${BoxRow} width=${width}>
          <${Text} color=${C.dim}>no robot selected<//>
        <//>
        <${Box}>
          <${Text} color=${C.rule}>╰${'─'.repeat(Math.max(0, width - 2))}╯<//>
        <//>
      <//>
    `;
  }

  const live = robot.hist.filter((v) => v > 0);
  const min = live.length ? Math.min(...live) : 0;
  const max = live.length ? Math.max(...live) : 0;
  const avg = live.length ? live.reduce((a, b) => a + b, 0) / live.length : 0;
  // Mean absolute deviation rather than a standard deviation: latency here is
  // spiky and bimodal (a retransmit doubles it), and MAD describes "how far
  // off is a typical sample" without one 12ms retry dominating the number.
  const jitter = live.length ? live.reduce((a, b) => a + Math.abs(b - avg), 0) / live.length : 0;

  const graphRows = Math.max(2, height - 9);
  const graphCells = Math.max(6, inner - 8);
  const scale = Math.max(5000, max) * 1.2;
  const graph = brailleGraph(robot.hist, graphCells, graphRows, scale);
  const col = latencyColor(robot.latencyUs);

  const title = ` R${robot.id} `;
  const hint = ' ↑↓ select ';
  const fill = Math.max(0, width - 4 - title.length - hint.length - 7);

  return html`
    <${Box} flexDirection="column" width=${width}>
      <${Box}>
        <${Text} color=${C.rule}>╭─<//>
        <${Text} color=${C.white} bold>${title}<//>
        <${Text} color=${C.dim}>· FOCUS<//>
        <${Text} color=${C.rule}> ${'─'.repeat(fill)}<//>
        <${Text} color=${C.dim}>${hint}<//>
        <${Text} color=${C.rule}>╮<//>
      <//>

      ${graph.map((line, i) => html`
        <${BoxRow} key=${i} width=${width}>
          <${Text} color=${C.dim}>${
            (i === 0 ? (scale / 1000).toFixed(1) : i === graph.length - 1 ? '0.0' : '').padStart(4)
          } <//>
          <${Text} color=${robot.lost ? C.rule : col}>${line}<//>
        <//>
      `)}

      <${Field} width=${width} label="">
        <${Text} color=${C.dim}>min <//>
        <${Text} color=${C.fg}>${(min / 1000).toFixed(1)}<//>
        <${Text} color=${C.dim}>  avg <//>
        <${Text} color=${C.fg}>${(avg / 1000).toFixed(1)}<//>
        <${Text} color=${C.dim}>  max <//>
        <${Text} color=${C.fg}>${(max / 1000).toFixed(1)} ms<//>
        <${Text} color=${C.dim}>   jitter <//>
        <${Text} color=${C.fg}>±${(jitter / 1000).toFixed(1)}<//>
      <//>

      <${Box}>
        <${Text} color=${C.rule}>├${'─'.repeat(Math.max(0, width - 2))}┤<//>
      <//>

      <${Field} width=${width} label="pose">
        ${robot.pose
          ? html`
              <${Text} color=${C.fg}>x ${robot.pose.x.toFixed(0).padStart(5)}  y ${robot.pose.y.toFixed(0).padStart(5)}  yaw ${robot.pose.yaw.toFixed(0).padStart(4)}° <//>
              <${Text} color=${C.cyanBright}>${headingArrow(robot.pose.yaw)}<//>
            `
          : html`<${Text} color=${C.dim}>no vision fix<//>`}
      <//>

      <${Field} width=${width} label="drive">
        <${Text} color=${robot.motorL >= 0 ? C.cyan : C.magenta}>L ${String(robot.motorL).padStart(5)}<//>
        <${Text} color=${robot.motorR >= 0 ? C.cyan : C.magenta}>   R ${String(robot.motorR).padStart(5)}<//>
        <${Text} color=${C.dim}>   battery <//>
        <${Text} color=${batteryColor(robot.batteryV)}>${
          robot.batteryV == null ? '—' : `${robot.batteryV.toFixed(2)}V`
        }<//>
      <//>

      <${Field} width=${width} label="link">
        <${Text} color=${C.dim}>${robot.mac}   up ${fmtUptime(robot.uptime)}${robot.lost ? '' : `   ${robot.ageMs}ms ago`}<//>
      <//>

      <${Field} width=${width} label="flags">
        <${Text} color=${robot.flags & SC_STATUS_LOW_BATTERY ? C.orange : C.green}>${flagNames(robot.flags)}<//>
      <//>

      <${Box}>
        <${Text} color=${C.rule}>╰${'─'.repeat(Math.max(0, width - 2))}╯<//>
      <//>
    <//>
  `;
}
