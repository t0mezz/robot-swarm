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
    return out.join("\n");
  }

  setInterval(function () {
    var s = snapshot();
    if (s === "" || s === last) return;   // interface not built yet, or unchanged
    last = s;
    // Clearing `last` on failure means the next tick retries, so the tool
    // resyncs by itself after a restart.
    fetch("/params", { method: "POST", body: s }).catch(function () { last = ""; });
  }, 200);
})();
