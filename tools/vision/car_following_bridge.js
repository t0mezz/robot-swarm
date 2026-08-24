// car_following_bridge.js — appended to the NetLogo page when car_following
// serves it with --bridge. Reports the live model + slider values back to the
// tool so the robots run whatever the page is running.
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
