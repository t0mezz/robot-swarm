#!/usr/bin/env python3
"""Compare battery_log runs: does vision-measured speed track battery voltage?

    python3 tools/analysis/battery_log_plot.py battery_log_results/*.csv
    python3 tools/analysis/battery_log_plot.py run1.csv run2.csv --out fit.png

Reads the CSVs written by tools/vision/battery_log.cpp and, per run, fits
orbit speed against the battery voltage measured under load (the same rows,
so the same load). It prints slope, intercept, R^2, the run's length and why
it ended. It then pools every run after normalising speed to that run's own
start (the median over its first --baseline-min minutes of orbit), because
robots differ in friction and gearing far more than they differ in how
their speed falls with voltage.

The in-tool PNG already shows one run as it happens. This script is for
comparing runs after the fact, which that plot can't do.

Only the standard library is needed for the numbers. The figure needs
matplotlib and is skipped with a note if it isn't installed.
"""

import argparse
import csv
import math
import os
import statistics
import sys


def read_run(path):
    meta, lines = {}, []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                key, _, val = line[1:].strip().partition(":")
                meta[key.strip()] = val.strip()
            elif line.strip():
                lines.append(line)
    rows = []
    for r in csv.DictReader(lines):
        def num(k):
            v = r.get(k, "")
            return float(v) if v not in ("", None) else math.nan
        rows.append({
            "t": num("t_s"), "phase": r["phase"], "speed": num("speed_mms"),
            "mv": num("bat_mv"), "valid": r.get("bat_valid") == "1",
        })
    return {"path": path, "meta": meta, "rows": rows}


def linfit(xs, ys):
    """Least squares y = a*x + b; returns (a, b, r2) or None."""
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    b = my - a * mx
    syy = sum((y - my) ** 2 for y in ys)
    r2 = (sxy * sxy) / (sxx * syy) if syy > 0 else math.nan
    return a, b, r2


def orbit_points(run):
    return [(r["t"], r["mv"], r["speed"]) for r in run["rows"]
            if r["phase"] == "orbit" and r["valid"] and not math.isnan(r["speed"])
            and not math.isnan(r["mv"])]


def hms(s):
    s = int(s)
    return f"{s // 3600}:{(s // 60) % 60:02d}:{s % 60:02d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--baseline-min", type=float, default=5.0,
                    help="minutes of orbit that define a run's starting speed (default 5)")
    ap.add_argument("--out", default="battery_log_analysis.png")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    runs = [read_run(p) for p in args.csv]
    pooled_x, pooled_y = [], []

    print(f"{'run':<44} {'robot':>5} {'cmd':>4} {'rows':>6} {'length':>8} "
          f"{'mm/s per V':>11} {'R^2':>6} {'v0 mm/s':>8}  ended")
    for run in runs:
        pts = orbit_points(run)
        run["pts"] = pts
        name = os.path.basename(run["path"])
        length = run["rows"][-1]["t"] if run["rows"] else 0.0
        ended = run["meta"].get("ended", "(no end line: still running or killed)")
        if not pts:
            print(f"{name:<44} no orbit rows with a valid battery reading")
            continue
        t0 = pts[0][0]
        base = [s for t, _, s in pts if t - t0 <= args.baseline_min * 60.0]
        v0 = statistics.median(base)
        run["v0"] = v0
        fit = linfit([mv for _, mv, _ in pts], [s for _, _, s in pts])
        run["fit"] = fit
        for _, mv, s in pts:
            pooled_x.append(mv)
            pooled_y.append(s / v0)
        slope = f"{fit[0] * 1000:11.1f}" if fit else f"{'-':>11}"
        r2 = f"{fit[2]:6.3f}" if fit else f"{'-':>6}"
        print(f"{name:<44} {run['meta'].get('robot', '?'):>5} {run['meta'].get('cmd', '?'):>4} "
              f"{len(pts):>6} {hms(length):>8} {slope} {r2} {v0:8.1f}  {ended}")

    pooled = linfit(pooled_x, pooled_y)
    if pooled and len(runs) > 1:
        a, b, r2 = pooled
        print(f"\npooled, normalised to each run's v0: speed/v0 = {a * 1000:.3f} * V {'-' if b < 0 else '+'} {abs(b):.3f}"
              f"   R^2 = {r2:.3f}   ({len(pooled_x)} rows, {len(runs)} runs)")

    if args.no_plot:
        return
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed: numbers only; `pip install matplotlib` for the figure)",
              file=sys.stderr)
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    for run in runs:
        pts = run.get("pts")
        if not pts:
            continue
        label = f"r{run['meta'].get('robot', '?')} c{run['meta'].get('cmd', '?')}"
        mv = [p[1] / 1000.0 for p in pts]
        sp = [p[2] for p in pts]
        sc = ax1.scatter(mv, sp, s=3, alpha=0.4, label=label)
        if run.get("fit"):
            a, b, _ = run["fit"]
            xs = [min(p[1] for p in pts), max(p[1] for p in pts)]
            ax1.plot([x / 1000.0 for x in xs], [a * x + b for x in xs], color=sc.get_facecolor()[0][:3], lw=2)
        ax2.scatter(mv, [s / run["v0"] for s in sp], s=3, alpha=0.4, label=label)
    if pooled:
        a, b, _ = pooled
        xs = [min(pooled_x), max(pooled_x)]
        ax2.plot([x / 1000.0 for x in xs], [a * x + b for x in xs], "k--", lw=1.5, label="pooled fit")
    ax1.set(xlabel="battery under load (V)", ylabel="measured speed (mm/s)",
            title="speed vs voltage, per run")
    ax2.set(xlabel="battery under load (V)", ylabel="speed / run's starting speed",
            title="normalised, all runs")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend(markerscale=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
