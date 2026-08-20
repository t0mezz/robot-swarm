// Log.js — the robot→PC debug channel (MSG_DEBUG), docked to the bottom.
//
// Same entries the C++ dashboard prints, but in a fixed-height strip at a
// fixed place. There it appeared between the robot rows and the vision
// panel and only when non-empty, so the whole lower half of the screen
// jumped the first time a robot logged anything.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C } from '../theme.js';

export function Log({ entries, width, height, now }) {
  const rows = Math.max(1, height - 1);
  const shown = entries.slice(-rows);
  const blanks = rows - shown.length;

  return html`
    <${Box} flexDirection="column" width=${width}>
      <${Box}>
        <${Text} color=${C.white} bold> LOG<//>
        <${Text} color=${C.dim}>   ${entries.length ? `last ${shown.length} of ${entries.length}` : 'no robot debug output yet'}<//>
      <//>
      ${shown.map((e, i) => {
        const age = (now - e.at) / 1000;
        return html`
          <${Box} key=${`${e.at}-${i}`}>
            <${Text} color=${C.dim}>${`-${age.toFixed(1)}s`.padStart(8)} <//>
            <${Text} color=${C.cyan}>${`R${e.id}`.padEnd(4)}<//>
            <${Text} color=${C.fg} wrap="truncate">${e.text}<//>
          <//>
        `;
      })}
      ${Array.from({ length: blanks }, (_, i) => html`<${Box} key=${`b${i}`}><${Text}> <//><//>`)}
    <//>
  `;
}

export function KeyHints({ width, sortBy, follow, paused }) {
  const key = (k, label, active) => html`
    <${Box}>
      <${Text} color=${active ? C.cyanBright : C.white} bold>${k}<//>
      <${Text} color=${active ? C.cyanBright : C.dim}> ${label}  <//>
    <//>
  `;
  return html`
    <${Box} width=${width}>
      <${Text}> <//>
      ${key('↑↓', 'select')}
      ${key('f', follow ? 'following worst' : 'follow', follow)}
      ${key('s', `sort:${sortBy}`, sortBy !== 'id')}
      ${key('p', paused ? 'resume' : 'pause', paused)}
      ${key('q', 'quit')}
    <//>
  `;
}
