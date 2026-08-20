// html.js — JSX-like templating without a build step.
//
// htm gives tagged-template syntax that compiles to React.createElement at
// runtime, so this tool runs with a bare `node src/cli.js`. That is a
// deliberate match for the rest of the repo: the PC tools use a plain
// Makefile and the robot firmware has no build step at all, so adding a
// bundler/transpiler here would have been the only watch-mode-and-artifacts
// toolchain in the tree. Swapping to real JSX later is mechanical — add tsx
// or esbuild and change the tag to nothing.
//
// Usage differs from JSX in one way: components are interpolated rather than
// named, i.e. html`<${Box} flexDirection="column">…<//>`.

import htm from 'htm';
import { createElement } from 'react';

export const html = htm.bind(createElement);
