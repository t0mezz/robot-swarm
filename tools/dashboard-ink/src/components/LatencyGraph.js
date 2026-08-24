// LatencyGraph.js — swarm-wide latency over the history window.
//
// Same braille trace the C++ dashboard draws, but given the left column's
// full width instead of being padded into whatever space was left over after
// the vision box, and with a labelled time axis so "how far back does this
// go" doesn't have to be inferred from the tick interval.

import { Box, Text } from 'ink';
import { html } from '../html.js';
import { C } from '../theme.js';
import { brailleGraph } from '../glyphs.js';

const LABEL_W = 8;   // "  5.9ms │ "

export function LatencyGraph({ series, width, height, windowSec, title = 'SWARM LATENCY', subtitle, color = C.cyan }) {
  const rows = Math.max(2, height - 3);      // title, axis rule, time labels
  const cells = Math.max(4, width - LABEL_W - 2);
  // Floored at 5ms so a healthy swarm shows a flat low trace rather than a
  // dramatic-looking one produced by autoscaling to sub-millisecond noise.
  const maxVal = Math.max(5000, ...series) * 1.2;
  const graph = brailleGraph(series, cells, rows, maxVal);

  return html`
    <${Box} flexDirection="column" width=${width}>
      <${Box}>
        <${Text} color=${C.white} bold> ${title}<//>
        ${subtitle ? html`<${Text} color=${C.dim}>   ${subtitle}<//>` : null}
      <//>
      ${graph.map((line, i) => {
        const label = i === 0 ? `${(maxVal / 1000).toFixed(1)}ms`
          : i === graph.length - 1 ? '0.0ms' : '';
        return html`
          <${Box} key=${i}>
            <${Text} color=${C.dim}>${label.padStart(LABEL_W - 2)}<//>
            <${Text} color=${C.rule}> │ <//>
            <${Text} color=${color}>${line}<//>
          <//>
        `;
      })}
      <${Box}>
        <${Text} color=${C.rule}>${' '.repeat(LABEL_W - 1)}└${'─'.repeat(cells + 1)}<//>
      <//>
      <${Box}>
        <${Text} color=${C.dim}>${' '.repeat(LABEL_W + 1)}-${windowSec}s<//>
        <${Text} color=${C.dim}>${'now'.padStart(Math.max(1, cells - 4))}<//>
      <//>
    <//>
  `;
}
