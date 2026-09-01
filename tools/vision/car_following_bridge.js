// car_following_bridge.js — appended to the NetLogo page when car_following
// serves it with --bridge. Reports the live model, slider values and run state
// back to the tool, so the robots run whatever the page is running and start
// when the page starts.
//
// The vendored HTML on disk is never modified; this is injected at serve time
// (see HttpBridge in lib/CarFollowing/http_bridge.h).
//
// It polls the DOM instead of listening for change events: NetLogo Web also
// updates widgets programmatically (and re-renders them on tab switches), and
// a poll catches every one of those paths without knowing any of them. At a
// few dozen elements every 200ms the cost is irrelevant.
//
// It also injects two controls of its own — the cooperative-buffering
// parameter B and the id of the robot that plays the buffering vehicle. They
// are not NetLogo widgets: the vendored page is a fixed artefact and is never
// edited on disk, and a robot id has no meaning inside the simulation anyway.
// They ride out on the same POST as everything else, as buffer-b / buffer-id.
//
// And a third graph: the page's own "Simulation" and "Real trajectories"
// plots are NetLogo widgets (the live model, and a hardcoded historical
// dataset from the 2007 paper) — neither is what the camera actually saw. A
// canvas panel here polls GET /trajectories, which the tool republishes once
// per model tick, and draws it the same shape (space vs. time) as those two:
// the same absolute 0..250s time axis that clears and restarts when it fills,
// the same single black pen, and the same size, anchored to the right edge of
// the "Simulation" plot so the three read as one row.

(function () {
  var last = "";
  var setupClicks = 0;

  // The page's one-shot buttons (here: "Setup") render as <button>, so a click
  // leaves no state in the DOM to poll for. Count them instead, in the capture
  // phase so a NetLogo handler that stops propagation cannot hide it, and
  // report a monotone counter — the tool reacts to the counter *changing*,
  // which keeps every POST idempotent like the rest of the snapshot.
  document.addEventListener("click", function (ev) {
    var el = ev.target && ev.target.closest
             ? ev.target.closest(".netlogo-button")
             : null;
    if (!el || el.classList.contains("netlogo-forever-button")) return;
    var label = el.querySelector(".netlogo-label");
    if (label && label.textContent.trim().toLowerCase() === "setup") setupClicks++;
  }, true);

  // Survives a reload of the page; the tool holds its own copy either way.
  function stored(key, fallback) {
    try { var v = localStorage.getItem(key); return v === null ? fallback : v; }
    catch (e) { return fallback; }        // storage disabled — defaults are fine
  }
  function store(key, value) {
    try { localStorage.setItem(key, value); } catch (e) { /* not worth failing over */ }
  }

  var panel = null;

  // One labelled number input, appended to the panel.
  function field(labelText, key, attrs, hint) {
    var wrap = document.createElement("label");
    wrap.style.cssText = "display:block;margin:6px 0;font:12px sans-serif";
    wrap.appendChild(document.createTextNode(labelText));

    var input = document.createElement("input");
    input.type = "number";
    for (var a in attrs) input.setAttribute(a, attrs[a]);
    input.value = stored(key, attrs.value);
    input.style.cssText = "width:70px;margin-left:8px";
    input.addEventListener("input", function () { store(key, input.value); });
    wrap.appendChild(input);

    var note = document.createElement("div");
    note.textContent = hint;
    note.style.cssText = "color:#555;font:10px sans-serif;margin-top:2px";
    wrap.appendChild(note);

    panel.appendChild(wrap);
    return input;
  }

  var bInput, idInput;

  function buildPanel() {
    panel = document.createElement("div");
    panel.style.cssText =
      "position:fixed;right:12px;bottom:12px;z-index:9999;background:#fff;" +
      "border:1px solid #999;border-radius:4px;padding:8px 10px;" +
      "box-shadow:0 1px 6px rgba(0,0,0,.25)";

    var title = document.createElement("div");
    title.textContent = "Cooperative buffering";
    title.style.cssText = "font:bold 12px sans-serif;margin-bottom:4px";
    panel.appendChild(title);

    // B = 1 is the non-cooperative baseline, so it is the default. The tool
    // caps B against the live robot count (see cfMaxBuffering) rather than at
    // a fixed maximum, so this input is deliberately not bounded above.
    bInput  = field("B", "cf-buffer-b", { min: "1", step: "0.5", value: "1" },
                    "buffering vehicle keeps B x the time gap");
    idInput = field("Robot id", "cf-buffer-id", { min: "-1", step: "1", value: "-1" },
                    "which robot buffers; -1 = none");
    document.body.appendChild(panel);
  }

  // Widget names come from the rendered labels, so this stays in step with
  // the page rather than with a hard-coded list of parameters. The names are
  // the NetLogo slider titles ("Speed-max", "Time-gap", ...), lowercased.
  //
  // NetLogo's own "model speed" control carries the class netlogo-speed-slider,
  // a different class token, so .netlogo-slider does not pick it up.
  function snapshot() {
    var out = [];
    document.querySelectorAll(".netlogo-slider").forEach(function (w) {
      var label = w.querySelector(".netlogo-label");
      var input = w.querySelector("input[type=range]");
      if (label && input)
        out.push(label.textContent.trim().toLowerCase() + "=" + input.value);
    });
    // Each <option value> is the choice string itself ("IDM", "CF-OVM", ...),
    // which is what the tool parses.
    var sel = document.querySelector(".netlogo-chooser select");
    if (sel && sel.selectedIndex >= 0)
      out.push("model=" + (sel.value ||
                           sel.options[sel.selectedIndex].textContent).trim());

    // The run cue. A forever button ("Move") renders as a <label> carrying a
    // checkbox whose checked state *is* the running state — NetLogo also
    // marks the label .netlogo-active, and either is read, so a template
    // change on one of them does not silently strand the robots.
    var running = false;
    document.querySelectorAll(".netlogo-forever-button").forEach(function (b) {
      var box = b.querySelector("input[type=checkbox]");
      if ((box && box.checked) || b.classList.contains("netlogo-active")) running = true;
    });
    out.push("run=" + (running ? 1 : 0));
    out.push("setup=" + setupClicks);

    // Our own two fields, on the same POST. The tool reads them per key, so
    // they can ride along from the first snapshot like everything else.
    out.push("buffer-b=" + (bInput.value || "1"));
    out.push("buffer-id=" + (idInput.value || "-1"));
    return out.join("\n");
  }

  buildPanel();

  setInterval(function () {
    var s = snapshot();
    if (s === "" || s === last) return;   // interface not built yet, or unchanged
    last = s;
    // Clearing `last` on failure means the next tick retries, so the tool
    // resyncs by itself after a restart.
    fetch("/params", { method: "POST", body: s }).catch(function () { last = ""; });
  }, 200);

  // ── Camera trajectories ─────────────────────────────────────────────────
  // The panel is anchored to the page's own "Simulation" plot rather than to
  // the viewport, so the three space-time graphs sit in a row. NetLogo Web
  // positions its widgets from the model's own coordinates and re-lays them
  // out on resize and on tab switches, so the anchor is re-measured on the
  // same poll that redraws the trace — the same reason the parameter snapshot
  // above polls the DOM instead of listening for events.
  var TRAJ_W = 330, TRAJ_H = 442;   // the page's own plot size, until measured
  var TRAJ_GAP = 36;                // and the gap it leaves between them
  var trajWrap, trajCanvas, trajCtx;

  function buildTrajPanel() {
    trajWrap = document.createElement("div");
    // content-box + a 1px black border is exactly how .netlogo-plot is
    // styled, so the canvas inside lines up with the plots beside it.
    trajWrap.style.cssText =
      "position:absolute;left:-9999px;top:0;z-index:9998;background:#fff;" +
      "box-sizing:content-box;border:1px solid #000";

    trajCanvas = document.createElement("canvas");
    trajCanvas.width = TRAJ_W;
    trajCanvas.height = TRAJ_H;
    trajCanvas.style.cssText = "display:block";
    trajWrap.appendChild(trajCanvas);

    document.body.appendChild(trajWrap);
    trajCtx = trajCanvas.getContext("2d");
  }

  // The "Simulation" plot, by its rendered title, with the rightmost plot as
  // the fallback — the row is built left to right, so the panel belongs after
  // the last of them either way.
  function simulationPlot() {
    var plots = document.querySelectorAll(".netlogo-plot");
    var right = null;
    for (var i = 0; i < plots.length; i++) {
      if (/simulation/i.test(plots[i].textContent || "")) return plots[i];
      if (!right ||
          plots[i].getBoundingClientRect().right > right.getBoundingClientRect().right)
        right = plots[i];
    }
    return right;
  }

  // Moves and sizes the panel to sit against the right edge of that plot.
  // Returns false while the interface has not been laid out yet, which is the
  // state the page is in for the first few hundred milliseconds.
  function anchorTrajPanel() {
    var sim = simulationPlot();
    if (!sim) return false;
    var r = sim.getBoundingClientRect();
    if (r.width < 2 || r.height < 2) return false;

    // Take the page's own plot-to-plot spacing where there is one to measure,
    // so the row keeps its rhythm if the page is ever zoomed.
    var gap = TRAJ_GAP;
    var plots = document.querySelectorAll(".netlogo-plot");
    for (var i = 0; i < plots.length; i++) {
      var o = plots[i].getBoundingClientRect();
      if (plots[i] !== sim && o.right <= r.left + 1 && r.left - o.right < gap)
        gap = r.left - o.right;
    }

    trajWrap.style.left = (r.right + window.pageXOffset + gap) + "px";
    trajWrap.style.top  = (r.top + window.pageYOffset) + "px";

    // clientWidth/Height is the plot's content box — the canvas matches that,
    // and our own 1px border stands in for the plot's.
    var w = sim.clientWidth || TRAJ_W, h = sim.clientHeight || TRAJ_H;
    if (trajCanvas.width !== w || trajCanvas.height !== h) {
      trajCanvas.width = w;
      trajCanvas.height = h;
    }
    return true;
  }

  // Margins around the plot area: axis lines, end-of-axis tick labels and
  // "Space [m]" / "Time [s]" titles, plus room for the panel's own title at
  // the top — drawn inside the canvas, as Highcharts draws the plots' titles.
  var PAD_L = 40, PAD_R = 10, PAD_T = 20, PAD_B = 30;

  function drawTrajectories(data) {
    var ctx = trajCtx;
    var W = trajCanvas.width, H = trajCanvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#fff";
    ctx.fillRect(0, 0, W, H);

    var plotW = W - PAD_L - PAD_R;
    var plotH = H - PAD_T - PAD_B;

    var road = data.road > 0 ? data.road : 1;
    // The time axis is absolute: 0 to the window the tool publishes, whatever
    // the samples currently span. The tool clears its buffer and restarts its
    // clock when that fills, so the trace sweeps the plot and starts over
    // rather than the axis sliding under it.
    var span = data.window > 0 ? data.window : 250;
    var pts = data.pts || [];

    function x(s) { return PAD_L + (s / road) * plotW; }
    function y(t) { return PAD_T + (1 - t / span) * plotH; }

    ctx.fillStyle = "#000";
    ctx.font = "bold 11px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Camera trajectories", W / 2, 13);

    // Axes.
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(PAD_L, PAD_T);
    ctx.lineTo(PAD_L, PAD_T + plotH);
    ctx.lineTo(PAD_L + plotW, PAD_T + plotH);
    ctx.stroke();

    ctx.fillStyle = "#333";
    ctx.font = "10px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Space [m]", PAD_L + plotW / 2, H - 4);
    ctx.save();
    ctx.translate(10, PAD_T + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText("Time [s]", 0, 0);
    ctx.restore();

    // End-of-axis values, so the fixed extents are readable rather than
    // implied — the whole point of an absolute axis.
    ctx.textAlign = "left";
    ctx.fillText("0", PAD_L, PAD_T + plotH + 12);
    ctx.textAlign = "right";
    ctx.fillText(road.toFixed(road < 10 ? 1 : 0), PAD_L + plotW, PAD_T + plotH + 12);
    ctx.fillText("0", PAD_L - 4, PAD_T + plotH);
    ctx.fillText(span.toFixed(0), PAD_L - 4, PAD_T + 8);

    if (!pts.length) {
      ctx.textAlign = "center";
      ctx.fillText("waiting for a run\u2026", PAD_L + plotW / 2, PAD_T + plotH / 2);
      return;
    }

    // Bucket by id — pts already arrive time-ordered per id, since the tool
    // appends one sample per visible vehicle each model tick.
    var byId = {};
    for (var j = 0; j < pts.length; j++) {
      var id = pts[j][0];
      (byId[id] = byId[id] || []).push({ t: pts[j][1], s: pts[j][2] });
    }

    // A rough "one tick" duration, taken as the smallest positive gap
    // between consecutive samples anywhere in the buffer. A within-id gap
    // much larger than this means the vehicle dropped out and reappeared,
    // not that it kept moving — break the line there rather than drawing a
    // spurious diagonal across the whole plot. The same threshold catches a
    // ring wrap-around in space.
    var tickDt = Infinity;
    for (var id2 in byId) {
      var arr = byId[id2];
      for (var k = 1; k < arr.length; k++) {
        var d = arr[k].t - arr[k - 1].t;
        if (d > 0 && d < tickDt) tickDt = d;
      }
    }
    if (!isFinite(tickDt) || tickDt <= 0) tickDt = 0.1;

    // One black pen for every vehicle, as both NetLogo plots use: the wave is
    // the thing being read off these graphs, and it reads across the three of
    // them only if they are drawn the same way.
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 1.5;
    for (var id3 in byId) {
      var pts2 = byId[id3];
      ctx.beginPath();
      var started = false;
      for (var m = 0; m < pts2.length; m++) {
        var p = pts2[m];
        if (m > 0) {
          var prev = pts2[m - 1];
          var dropout = (p.t - prev.t) > tickDt * 4;
          var wrapped = Math.abs(p.s - prev.s) > road / 2;
          if (dropout || wrapped) started = false;
        }
        if (!started) { ctx.moveTo(x(p.s), y(p.t)); started = true; }
        else ctx.lineTo(x(p.s), y(p.t));
      }
      ctx.stroke();
    }
  }

  buildTrajPanel();

  setInterval(function () {
    if (!anchorTrajPanel()) return;   // interface not laid out yet
    fetch("/trajectories").then(function (r) { return r.json(); })
      .then(drawTrajectories).catch(function () { /* transient — next poll retries */ });
  }, 500);
})();
