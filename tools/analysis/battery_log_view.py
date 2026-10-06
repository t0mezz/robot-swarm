#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Interactive viewer for battery_log runs: zoom into one run's data.

    tools/analysis/battery_log_view.py                        # newest run
    tools/analysis/battery_log_view.py run.csv [run2.csv ...] # one window each
    tools/analysis/battery_log_view.py run.csv --save view.png
    tools/analysis/battery_log_view.py run.csv --range 10 15  # open zoomed in (minutes)

Reads a CSV written by tools/vision/battery_log.cpp. Unlike the in-tool PNG
(fixed axes, redrawn every minute), every axis here is scaled from the data
in view, so zooming into a few minutes shows that stretch in full detail.

Left column, sharing one time axis (zoom or pan one, they all follow):
  speed      orbit rows + a centred trimmed mean over whole laps, rests shaded
  battery    loaded (orbit) and resting readings, --stop-mv
  sag        per rest: resting minus loaded voltage
  orbit      radial error and mean |turn|, to tell drift from slowdown
Right column, following the time window on the left:
  speed vs loaded voltage, with a linear fit and per-reading medians
  sag vs resting voltage

The y-axes rescale to the 1st..99th percentile of what is in view, so a
single bad detection never flattens a panel. Points outside the time window
stay in the right-hand panels as grey context.

Controls: the toolbar's zoom (box) and pan, plus the scroll wheel (time panels
zoom in time only, the right-hand panels in both axes). Home or `h` resets.

Run with `uv run` (the shebang does this), which fetches matplotlib and numpy
into a cached environment, or with any python that already has both.
"""

import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

SMOOTH_WINDOW_S = 30.0   # at least this long, rounded up to whole laps; centred, since offline we have the future
SMOOTH_TRIM = 0.2        # fraction cut from each end before averaging
SAG_LOADED_S = 30.0      # orbit rows before a rest that define its loaded voltage


# ── Loading ──────────────────────────────────────────────────────────────────

def read_run(path):
    meta, lines = {}, []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                key, _, val = line[1:].strip().partition(":")
                meta[key.strip()] = val.strip()
            elif line.strip():
                lines.append(line)
    rows = list(csv.DictReader(lines))

    def col(name):
        out = np.full(len(rows), np.nan)
        for i, r in enumerate(rows):
            v = r.get(name, "")
            if v not in ("", None):
                out[i] = float(v)
        return out

    run = {
        "path": path, "meta": meta,
        "t": col("t_s"), "dt": col("dt_s"),
        "phase": np.array([r["phase"] for r in rows]),
        "speed": col("speed_mms"), "radial": col("radial_err_mm"), "turn": col("abs_turn"),
        "mv": col("bat_mv"),
    }
    valid = np.array([r.get("bat_valid") == "1" for r in rows])
    run["mv"][~valid] = np.nan
    run["v"] = run["mv"] / 1000.0
    run["tm"] = run["t"] / 60.0
    return run


def newest_csv():
    here = os.path.dirname(os.path.abspath(__file__))
    found = []
    for d in ("battery_log_results", os.path.join(here, "..", "build", "battery_log_results"),
              os.path.join(here, "..", "battery_log_results")):
        found += glob.glob(os.path.join(d, "battery_log_*.csv"))
    return max(found, key=os.path.getmtime) if found else None


# ── Derived series ───────────────────────────────────────────────────────────

def trimmed_mean(x, trim):
    x = np.sort(x)
    k = int(len(x) * trim)
    return x[k:len(x) - k].mean() if len(x) > 2 * k else np.median(x)


def rolling_trimmed_mean(t, v, window, trim):
    """Centred rolling mean with the extremes cut off. Not a median: the
    per-row speeds are bimodal (they vary around the lap), and a median
    flips between the two modes from row to row."""
    out = np.full_like(v, np.nan)
    ok = np.isfinite(v)
    tt, vv = t[ok], v[ok]
    lo = np.searchsorted(tt, tt - window / 2)
    hi = np.searchsorted(tt, tt + window / 2, side="right")
    out[ok] = [trimmed_mean(vv[a:b], trim) for a, b in zip(lo, hi)]
    return out


def lap_window(run, speed_orbit):
    """SMOOTH_WINDOW_S rounded up to a whole number of laps, and the lap count.

    The per-row speed is periodic in the lap: it depends on where on the ring
    the robot is (camera calibration, ring centre), not only on the battery.
    A window of 2.3 laps leaves that period in the trace as a ripple as large
    as the effect being measured; a whole number of laps cancels it."""
    m = re.search(r"radius=([\d.]+)", run["meta"].get("ring", ""))
    v = np.nanmedian(speed_orbit) if np.isfinite(speed_orbit).any() else math.nan
    if not m or not v > 0:
        return SMOOTH_WINDOW_S, None
    lap = 2 * math.pi * float(m.group(1)) / v
    laps = max(1, math.ceil(SMOOTH_WINDOW_S / lap - 0.05))
    return laps * lap, laps


def segments(mask):
    """(start, end) index pairs of the runs of True in mask, end exclusive."""
    edges = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def rest_sag(run):
    """One point per rest: (time min, resting V, sag mV).

    Resting is the rest's last valid reading, the most recovered one (the
    battery is sampled every 2 s and the first reading of a rest may still
    be from under load). Loaded is the median orbit reading in the
    SAG_LOADED_S before the rest began."""
    t, mv, phase = run["t"], run["mv"], run["phase"]
    orbit = (phase == "orbit") & np.isfinite(mv)
    out = []
    for a, b in segments(phase == "rest"):
        rest_mv = mv[a:b][np.isfinite(mv[a:b])]
        t0 = t[a] - run["dt"][a]
        before = orbit & (t <= t0) & (t > t0 - SAG_LOADED_S)
        if rest_mv.size == 0 or not before.any():
            continue
        out.append(((t0 + t[b - 1]) / 2 / 60.0, rest_mv[-1] / 1000.0,
                    rest_mv[-1] - np.median(mv[before])))
    return np.array(out).reshape(-1, 3)


def robust_lim(vals, min_span, pct=(1, 99), pad=0.12):
    v = np.asarray(vals)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return None
    lo, hi = np.percentile(v, pct)
    mid, span = (lo + hi) / 2, max(hi - lo, min_span)
    return mid - span / 2 - pad * span, mid + span / 2 + pad * span


def linfit(x, y):
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 3 or np.ptp(x) == 0:
        return None
    a, b = np.polyfit(x, y, 1)
    r = np.corrcoef(x, y)[0, 1]
    return a, b, r * r


# ── Figure ───────────────────────────────────────────────────────────────────

class Highlight:
    """A scatter that shows every point grey and the ones inside the time
    window in colour, rescaling its axes to the coloured ones."""

    def __init__(self, ax, tm, x, y, cmap, norm, min_span, size=8):
        ok = np.isfinite(x) & np.isfinite(y)
        self.ax, self.tm, self.x, self.y, self.min_span = ax, tm[ok], x[ok], y[ok], min_span
        ax.scatter(self.x, self.y, s=size * 0.75, color="0.85", lw=0, zorder=1)
        self.fg = ax.scatter(self.x, self.y, c=self.tm, s=size, cmap=cmap, norm=norm, lw=0, zorder=2)
        self.sel = np.ones(self.x.size, bool)

    def window(self, t0, t1):
        self.sel = (self.tm >= t0) & (self.tm <= t1)
        self.fg.set_offsets(np.column_stack((self.x[self.sel], self.y[self.sel])))
        self.fg.set_array(self.tm[self.sel])
        for lim, setter, span in ((robust_lim(self.x[self.sel], self.min_span[0]), self.ax.set_xlim, 0),
                                  (robust_lim(self.y[self.sel], self.min_span[1]), self.ax.set_ylim, 1)):
            if lim:
                setter(lim)


def build(run, interactive, span=None):
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    tm, phase, meta = run["tm"], run["phase"], run["meta"]
    orbit = phase == "orbit"
    speed_orbit = np.where(orbit, run["speed"], np.nan)
    win_s, laps = lap_window(run, speed_orbit)
    med = rolling_trimmed_mean(run["t"], speed_orbit, win_s, SMOOTH_TRIM)
    med[~orbit] = np.nan
    sag = rest_sag(run)
    stop_v = float(meta["stop_mv"]) / 1000.0 if meta.get("stop_mv") else None
    tmax = np.nanmax(tm) if tm.size else 1.0

    fig = plt.figure(figsize=(16, 10), layout="constrained")
    gs = fig.add_gridspec(4, 2, width_ratios=(1.9, 1), height_ratios=(3, 2, 1.2, 1.2))
    axS = fig.add_subplot(gs[0, 0])
    axV = fig.add_subplot(gs[1, 0], sharex=axS)
    axG = fig.add_subplot(gs[2, 0], sharex=axS)
    axQ = fig.add_subplot(gs[3, 0], sharex=axS)
    axT = axQ.twinx()
    axC = fig.add_subplot(gs[0:2, 1])
    axR = fig.add_subplot(gs[2:4, 1])
    time_axes = (axS, axV, axG, axQ, axT)

    ended = meta.get("ended", "no end line: still running or killed")
    fig.suptitle(f"{os.path.basename(run['path'])}   robot {meta.get('robot', '?')}, "
                 f"cmd {meta.get('cmd', '?')}, {meta.get('dir', '?')}   ended {ended}", fontsize=10)

    for ax in (axS, axV, axG, axQ):
        for a, b in segments(phase == "rest"):
            ax.axvspan(tm[a] - run["dt"][a] / 60.0, tm[b - 1], color="0.92", lw=0, zorder=0)
        ax.grid(alpha=0.3)
    for ax in (axS, axV, axG):
        ax.tick_params(labelbottom=False)

    axS.plot(tm, speed_orbit, ".", ms=2.5, color="tab:orange", alpha=0.5, label="orbit row")
    axS.plot(tm, med, "-", lw=1.6, color="tab:blue", label=f"trimmed mean over {laps} laps ({win_s:.0f} s)" if laps
             else f"{win_s:.0f} s trimmed mean")
    axS.set_ylabel("speed (mm/s)")
    axS.legend(loc="upper right", fontsize=8, markerscale=3)
    axS.set_title("measured speed; grey = rest", fontsize=9, loc="left")

    v = run["v"]
    seek = (phase != "orbit") & (phase != "rest")
    axV.plot(tm[orbit], v[orbit], ".", ms=3, color="tab:orange", label="under load")
    axV.plot(tm[phase == "rest"], v[phase == "rest"], "o", ms=4, color="tab:blue", label="resting",
             zorder=3)
    axV.plot(tm[seek], v[seek], ".", ms=3, color="0.6", label="seek")
    if stop_v:
        axV.axhline(stop_v, color="tab:red", ls="--", lw=1, label=f"--stop-mv {stop_v:.2f} V")
    axV.set_ylabel("battery (V)")
    axV.legend(loc="upper right", fontsize=8, markerscale=3, ncol=4)

    axG.plot(sag[:, 0], sag[:, 2], "o-", ms=4, lw=1, color="tab:purple")
    axG.set_ylabel("sag (mV)")
    axG.set_title(f"resting minus loaded (median of the {SAG_LOADED_S:.0f} s before the rest)",
                  fontsize=9, loc="left")

    radial = np.where(orbit, run["radial"], np.nan)   # NaN, not dropped: no lines across rests
    turn = np.where(orbit, run["turn"], np.nan)
    axQ.plot(tm, radial, "-", lw=0.8, color="tab:green")
    axQ.set_ylabel("radial err (mm)", color="tab:green")
    axT.plot(tm, turn, "-", lw=0.8, color="tab:gray", alpha=0.8)
    axT.set_ylabel("mean |turn|", color="tab:gray")
    axQ.set_xlabel("time (min)")

    norm = Normalize(0.0, tmax)
    loaded = orbit & np.isfinite(v)
    hlC = Highlight(axC, tm[loaded], v[loaded], run["speed"][loaded], "viridis", norm, (0.1, 20.0))
    fitC, = axC.plot([], [], "k-", lw=1.5, zorder=3)
    lvlC, = axC.plot([], [], "D", ms=5, mfc="white", mec="k", zorder=4, label="median per reading")
    axC.set(xlabel="battery under load (V)", ylabel="speed (mm/s)")
    axC.set_title("speed vs loaded voltage", fontsize=9, loc="left")
    axC.grid(alpha=0.3)
    fig.colorbar(hlC.fg, ax=axC, label="time (min)")

    hlR = Highlight(axR, sag[:, 0], sag[:, 1], sag[:, 2], "viridis", norm, (0.1, 40.0), size=40)
    axR.set(xlabel="resting battery (V)", ylabel="sag (mV)")
    axR.set_title("sag vs resting voltage, one point per rest", fontsize=9, loc="left")
    axR.grid(alpha=0.3)

    # (axis, x, y, min span) for every panel that rescales y to the time window.
    scaled = [(axS, tm, speed_orbit, 20.0), (axV, tm, v, 0.1), (axG, sag[:, 0], sag[:, 2], 40.0),
              (axQ, tm, radial, 10.0), (axT, tm, turn, 1.0)]
    busy = [False]

    def on_window(_ax=None):
        if busy[0]:
            return
        busy[0] = True
        t0, t1 = axS.get_xlim()
        for ax, x, y, span in scaled:
            lim = robust_lim(y[(x >= t0) & (x <= t1)], span)
            if lim:
                ax.set_ylim(lim)
        hlC.window(t0, t1)
        hlR.window(t0, t1)

        x, y = hlC.x[hlC.sel], hlC.y[hlC.sel]
        fit = linfit(x, y)
        if fit:
            a, b, r2 = fit
            xs = np.array([x.min(), x.max()])
            fitC.set_data(xs, a * xs + b)
            fitC.set_label(f"fit: {a:.0f} mm/s per V, R² {r2:.2f}, n={x.size}")
        else:
            fitC.set_data([], [])
            fitC.set_label("fit: too few points")
        levels = [(u, np.median(y[x == u])) for u in np.unique(x) if np.count_nonzero(x == u) >= 5]
        lvlC.set_data([l[0] for l in levels], [l[1] for l in levels])
        axC.legend(loc="lower right", fontsize=8)
        busy[0] = False
        fig.canvas.draw_idle()

    for ax in time_axes:
        ax.callbacks.connect("xlim_changed", on_window)

    def on_scroll(ev):
        ax = ev.inaxes
        if ax is None or ev.xdata is None:
            return
        f = 0.8 if ev.button == "up" else 1.25
        x0, x1 = ax.get_xlim()
        ax.set_xlim(ev.xdata - (ev.xdata - x0) * f, ev.xdata + (x1 - ev.xdata) * f)
        if ax not in time_axes:
            y0, y1 = ax.get_ylim()
            ax.set_ylim(ev.ydata - (ev.ydata - y0) * f, ev.ydata + (y1 - ev.ydata) * f)
        fig.canvas.draw_idle()

    if interactive:
        fig.canvas.mpl_connect("scroll_event", on_scroll)

    axS.set_xlim(span if span else (0.0, tmax * 1.01))
    on_window()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="*", help="battery_log CSV(s); default: the newest run")
    ap.add_argument("--save", metavar="PNG", help="write the figure instead of opening a window "
                    "(with several CSVs, the run's name is appended)")
    ap.add_argument("--range", nargs=2, type=float, metavar=("FROM", "TO"),
                    help="open zoomed to this time window, in minutes")
    args = ap.parse_args()

    paths = args.csv or [p for p in [newest_csv()] if p]
    if not paths:
        sys.exit("no CSV given and none found in battery_log_results/")

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for p in paths:
        run = read_run(p)
        if run["t"].size == 0:
            print(f"{p}: no rows", file=sys.stderr)
            continue
        fig = build(run, interactive=not args.save, span=args.range)
        if args.save:
            out = args.save if len(paths) == 1 else \
                f"{os.path.splitext(args.save)[0]}_{os.path.splitext(os.path.basename(p))[0]}.png"
            fig.savefig(out, dpi=150)
            print(f"wrote {out}")
    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
