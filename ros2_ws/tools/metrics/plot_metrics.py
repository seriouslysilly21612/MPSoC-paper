#!/usr/bin/env python3
"""Turn the pipeline metrics CSVs into readable dashboards + a text summary.

Reads what a recording run produced under a base dir (default evidence/metrics):
    performance/{detector,pick_logic,target_3d,base}.csv (+ merged.csv if present)
    resource/run.csv
    yield/{detector,pick_logic,target_3d,base}.csv
and writes three PNGs to <base>/plots/ plus a plain-English summary to stdout.

Runs on PC or Kria. Only extra dependency: matplotlib. If you recorded on the
board and want to plot on your PC, just scp the evidence/metrics dir over and point
--base at it:
    scp -r ubuntu@<board>:~/ros2_ws/evidence/metrics /tmp/metrics
    python3 plot_metrics.py --base /tmp/metrics

Design: Okabe-Ito colorblind-safe categorical colors assigned per-entity in a
fixed order (never cycled); single-axis panels only (temp and power are separate
subplots, not a dual axis); a legend on every multi-series panel.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")  # headless: save PNGs, no display needed
import matplotlib.pyplot as plt

# ---- Okabe-Ito colorblind-safe palette (fixed order, not cycled) ----
OKABE_ITO = ["#0072B2", "#E69F00", "#009E73", "#D55E00",
             "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
# CPU/mem panels: color follows the PROCESS (same color across panels), not rank.
PROC_ORDER = ["camera", "detector", "worker", "pick_logic",
              "target_3d", "target_base", "static_tf"]
PROC_COLOR = {p: OKABE_ITO[i] for i, p in enumerate(PROC_ORDER)}
HUE = "#0072B2"      # single-series hue
ACCENT = "#D55E00"   # percentile / reference lines

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "axes.edgecolor": "#888888", "axes.linewidth": 0.8,
    "axes.grid": True, "grid.color": "#E6E6E6", "grid.linewidth": 0.8,
    "axes.axisbelow": True, "font.size": 9, "axes.titlesize": 10,
    "axes.titleweight": "bold", "legend.fontsize": 7.5, "legend.frameon": False,
})


# ------------------------- CSV / stats helpers -------------------------

def load_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def to_float(v):
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def fcol(rows, name):
    """Parsed non-null floats of a column (order preserved)."""
    return [x for x in (to_float(r.get(name)) for r in rows) if x is not None]


def pct(sorted_vals, q):
    n = len(sorted_vals)
    if n == 0:
        return None
    if n == 1:
        return sorted_vals[0]
    i = (n - 1) * q / 100.0
    lo, hi = int(i), min(int(i) + 1, n - 1)
    return sorted_vals[lo] * (1 - (i - lo)) + sorted_vals[hi] * (i - lo)


def stats(vals):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return None
    n = len(v)
    mean = sum(v) / n
    std = (sum((x - mean) ** 2 for x in v) / n) ** 0.5 if n > 1 else 0.0
    return {"n": n, "mean": mean, "std": std, "min": v[0], "max": v[-1],
            "p50": pct(v, 50), "p95": pct(v, 95), "p99": pct(v, 99)}


def f(x, nd=1):
    return "n/a" if x is None else f"{x:.{nd}f}"


# ------------------------- performance -------------------------

# E2E decomposition (mean of each ≈ mean true_e2e); label -> merged column
STAGES = [
    ("capture→detect", "age_in_ms"),
    ("detector proc", "processing_ms"),
    ("→pick (DDS)", "handoff_det_to_pick_ms"),
    ("pick_logic", "pick_logic_compute_ms"),
    ("→3d (DDS)", "handoff_pick_to_3d_ms"),
    ("3d reproj", "target3d_compute_ms"),
    ("→base (DDS)", "handoff_3d_to_base_ms"),
    ("base TF", "base_compute_ms"),
]


def load_perf(perf_dir):
    """Prefer merged.csv (join_perf output); else join the 4 node CSVs on the
    capture stamp and compute handoffs, so the tool works either way."""
    merged = os.path.join(perf_dir, "merged.csv")
    if os.path.exists(merged):
        return load_rows(merged)
    sources = {k: load_rows(os.path.join(perf_dir, f"{k}.csv"))
               for k in ("detector", "pick_logic", "target_3d", "base")}
    keyed = {k: {(r.get("capture_sec"), r.get("capture_nanosec")): r
                 for r in rows if r.get("capture_sec")}
             for k, rows in sources.items()}
    all_keys = set().union(*[set(d) for d in keyed.values()]) if any(keyed.values()) else set()
    rows = []
    for key in sorted(all_keys, key=lambda k: (int(k[0]), int(k[1]))):
        row = {"capture_sec": key[0], "capture_nanosec": key[1]}
        for name, d in keyed.items():
            r = d.get(key)
            if r:
                for kk, vv in r.items():
                    if kk not in ("capture_sec", "capture_nanosec"):
                        row[kk if kk not in row else f"{name}_{kk}"] = vv
        row["handoff_det_to_pick_ms"] = _diff(row.get("pick_logic_in_age_ms"), row.get("frame_age_ms"))
        row["handoff_pick_to_3d_ms"] = _diff(row.get("target3d_in_age_ms"), row.get("pick_logic_out_age_ms"))
        row["handoff_3d_to_base_ms"] = _diff(row.get("base_in_age_ms"), row.get("target3d_out_age_ms"))
        rows.append(row)
    return rows


def _diff(a, b):
    fa, fb = to_float(a), to_float(b)
    return None if fa is None or fb is None else fa - fb


def plot_performance(perf_dir, out_dir, summary):
    rows = load_perf(perf_dir)
    if not rows:
        summary.append("performance: CSV 없음 — 건너뜀")
        return
    e2e = fcol(rows, "true_e2e_ms")
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Performance — capture → /pick_target_base", fontsize=13, fontweight="bold")

    # (a) E2E stage breakdown — stacked horizontal bar of mean ms per stage
    a = ax[0, 0]
    means, labels, colors = [], [], []
    for i, (lab, colname) in enumerate(STAGES):
        s = stats(fcol(rows, colname))
        means.append(s["mean"] if s else 0.0)
        labels.append(lab)
        colors.append(OKABE_ITO[i % len(OKABE_ITO)])
    left = 0.0
    for m, lab, c in zip(means, labels, colors):
        a.barh(0, m, left=left, color=c, edgecolor="white", linewidth=1.2, label=f"{lab} {m:.1f}")
        if m > sum(means) * 0.04:
            a.text(left + m / 2, 0, f"{m:.0f}", ha="center", va="center", fontsize=7, color="white")
        left += m
    a.set_title(f"E2E stage breakdown (mean, sum={sum(means):.0f}ms)")
    a.set_xlabel("ms"); a.set_yticks([]); a.set_ylim(-0.5, 0.5)
    a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=4, fontsize=7)

    # (b) true_e2e histogram + percentile lines
    b = ax[0, 1]
    if e2e:
        s = stats(e2e)
        b.hist(e2e, bins=40, color=HUE, alpha=0.85, edgecolor="white", linewidth=0.4)
        for q, lab in [("p50", "p50"), ("p95", "p95"), ("p99", "p99")]:
            b.axvline(s[q], color=ACCENT, linestyle="--", linewidth=1.4)
            b.text(s[q], b.get_ylim()[1] * 0.95, f" {lab}={s[q]:.0f}", color=ACCENT, fontsize=7, rotation=90, va="top")
    b.set_title("true_e2e distribution (tail matters)"); b.set_xlabel("ms"); b.set_ylabel("frames")

    # (c) true_e2e over the run (jitter/drift)
    c = ax[1, 0]

    def cap_t(r):
        s = to_float(r.get("capture_sec"))
        return None if s is None else s + (to_float(r.get("capture_nanosec")) or 0) / 1e9

    valid = [(cap_t(r), to_float(r.get("true_e2e_ms"))) for r in rows]
    valid = [(ci, v) for ci, v in valid if ci is not None and v is not None]
    if valid:
        t0 = min(ci for ci, _ in valid)
        xs = [ci - t0 for ci, _ in valid]
        ys = [v for _, v in valid]
        c.plot(xs, ys, color=HUE, linewidth=1.0)
        c.axhline(66.6, color=ACCENT, linestyle=":", linewidth=1.2)
        c.text(0, 66.6, " 15Hz budget 66.6ms", color=ACCENT, fontsize=7, va="bottom")
    c.set_title("true_e2e over time (drift / spikes)"); c.set_xlabel("elapsed s"); c.set_ylabel("ms")

    # (d) cadence jitter — detect_period vs output_period histograms
    d = ax[1, 1]
    dp, op = fcol(rows, "detect_period_ms"), fcol(rows, "output_period_ms")
    plotted = False
    if dp:
        d.hist(dp, bins=40, color=OKABE_ITO[1], alpha=0.6, label="detect_period", edgecolor="white", linewidth=0.3)
        plotted = True
    if op:
        d.hist(op, bins=40, color=OKABE_ITO[2], alpha=0.6, label="output_period", edgecolor="white", linewidth=0.3)
        plotted = True
    d.axvline(66.6, color=ACCENT, linestyle=":", linewidth=1.2)
    d.set_title("publish interval jitter (15Hz = 66.6ms)"); d.set_xlabel("ms"); d.set_ylabel("count")
    if plotted:
        d.legend(loc="upper right")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(out_dir, "performance.png")
    fig.savefig(out, dpi=130); plt.close(fig)

    # text summary
    s = stats(e2e)
    if s:
        budget = 66.6
        over = sum(1 for x in e2e if x > budget) / len(e2e) * 100
        summary.append("── PERFORMANCE ──")
        summary.append(f"  true_e2e (ms): p50={f(s['p50'])}  p95={f(s['p95'])}  p99={f(s['p99'])}  max={f(s['max'])}  (n={s['n']})")
        summary.append(f"  15Hz budget(66.6ms) 초과 프레임: {over:.1f}%")
        worst = max(((stats(fcol(rows, cn)) or {'mean': 0})['mean'], lab) for lab, cn in STAGES)
        summary.append(f"  최대 병목 단계(mean): {worst[1]} ≈ {worst[0]:.1f}ms")
        dps = stats(dp)
        if dps:
            summary.append(f"  detect_period: mean={f(dps['mean'])}ms → ~{1000/dps['mean']:.1f}Hz, jitter(std)={f(dps['std'])}ms")
    summary.append(f"  → {out}")


# ------------------------- resource -------------------------

def _cols_with_prefix(rows, prefix):
    keys = []
    for r in rows[:1]:
        keys = [k for k in r.keys() if k.startswith(prefix)]
    return keys


def _cols_matching(rows, *needles):
    keys = []
    for r in rows[:1]:
        for k in r.keys():
            if all(n in k for n in needles):
                keys.append(k)
    return keys


def plot_resource(resource_csv, out_dir, summary):
    rows = load_rows(resource_csv)
    if not rows:
        summary.append("resource: CSV 없음 — 건너뜀")
        return
    tw = fcol(rows, "wall_time_s")
    t0 = tw[0] if tw else 0
    t = [(to_float(r.get("wall_time_s")) or t0) - t0 for r in rows]

    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle("Resource — per-process CPU / memory / thermal", fontsize=13, fontweight="bold")

    # (a) CPU cores per process — stacked area
    a = ax[0, 0]
    series = [(p, [to_float(r.get(f"cpu_cores_{p}")) or 0.0 for r in rows]) for p in PROC_ORDER
              if f"cpu_cores_{p}" in (rows[0] if rows else {})]
    if series:
        a.stackplot(t, *[s for _, s in series], labels=[p for p, _ in series],
                    colors=[PROC_COLOR[p] for p, _ in series], edgecolor="white", linewidth=0.3)
        a.legend(loc="upper left", ncol=2)
    a.set_title("CPU per process (stacked, cores)"); a.set_xlabel("elapsed s"); a.set_ylabel("cores")

    # (b) system CPU per core
    b = ax[0, 1]
    core_cols = sorted(_cols_matching(rows, "sys_cpu_pct_cpu"))
    for i, cc in enumerate(core_cols):
        vals = [(to_float(r.get(cc)) or 0.0) * 100 for r in rows]
        b.plot(t, vals, color=OKABE_ITO[i % len(OKABE_ITO)], linewidth=1.2, label=cc.replace("sys_cpu_pct_", ""))
    if core_cols:
        b.legend(loc="upper left", ncol=2, frameon=True, framealpha=0.85,
                 facecolor="white", edgecolor="#cccccc")
    b.set_title("system CPU per core (%)"); b.set_xlabel("elapsed s"); b.set_ylabel("%"); b.set_ylim(0, 100)

    # (c) RSS per process
    c = ax[0, 2]
    any_rss = False
    for p in PROC_ORDER:
        col = f"rss_kb_{p}"
        if rows and col in rows[0]:
            vals = [(to_float(r.get(col)) or None) for r in rows]
            xs = [ti for ti, v in zip(t, vals) if v is not None]
            ys = [v / 1024 for v in vals if v is not None]
            if ys:
                c.plot(xs, ys, color=PROC_COLOR[p], linewidth=1.2, label=p)
                any_rss = True
    if any_rss:
        c.legend(loc="best", ncol=2, frameon=True, framealpha=0.85,
                 facecolor="white", edgecolor="#cccccc")
    c.set_title("memory RSS per process (MB)"); c.set_xlabel("elapsed s"); c.set_ylabel("MB")

    # (d) temperature (own subplot — NOT a dual axis with power)
    d = ax[1, 0]
    temp_cols = sorted(_cols_matching(rows, "temp", "input"))
    for i, tc in enumerate(temp_cols):
        vals = [(to_float(r.get(tc)) or None) for r in rows]
        xs = [ti for ti, v in zip(t, vals) if v is not None]
        ys = [v / 1000 for v in vals if v is not None]  # milli-°C → °C
        if ys:
            d.plot(xs, ys, color=OKABE_ITO[i % len(OKABE_ITO)], linewidth=1.1, label=tc)
    if temp_cols:
        d.legend(loc="upper left", fontsize=6)
    d.set_title("SoC temperature (°C)"); d.set_xlabel("elapsed s"); d.set_ylabel("°C")

    # (e) power (own subplot)
    e = ax[1, 1]
    pow_cols = sorted(_cols_matching(rows, "power", "input"))
    for i, pc in enumerate(pow_cols):
        vals = [(to_float(r.get(pc)) or None) for r in rows]
        xs = [ti for ti, v in zip(t, vals) if v is not None]
        ys = [v / 1e6 for v in vals if v is not None]  # microW → W
        if ys:
            e.plot(xs, ys, color=OKABE_ITO[i % len(OKABE_ITO)], linewidth=1.1, label=pc)
    if pow_cols:
        e.legend(loc="upper left", fontsize=6)
    e.set_title("board power (W)"); e.set_xlabel("elapsed s"); e.set_ylabel("W")

    # (f) available memory + worker restarts
    g = ax[1, 2]
    memav = [(to_float(r.get("mem_available_kb")) or None) for r in rows]
    if any(v is not None for v in memav):
        xs = [ti for ti, v in zip(t, memav) if v is not None]
        ys = [v / 1024 for v in memav if v is not None]
        g.plot(xs, ys, color=HUE, linewidth=1.3, label="MemAvailable")
        g.legend(loc="upper left")
    g.set_title("MemAvailable (MB) — downward = possible leak"); g.set_xlabel("elapsed s"); g.set_ylabel("MB")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(out_dir, "resource.png")
    fig.savefig(out, dpi=130); plt.close(fig)

    # text summary
    summary.append("── RESOURCE ──")
    if series:
        totals = [sum(vals[i] for _, vals in series) for i in range(len(rows))]
        summary.append(f"  total CPU: mean={f(sum(totals)/len(totals),2)} cores, peak={f(max(totals),2)} cores")
        for p, vals in series:
            summary.append(f"    {p:<12} mean={f(sum(vals)/len(vals),2)} peak={f(max(vals),2)} cores")
    for p in PROC_ORDER:
        col = f"rss_kb_{p}"
        rs = stats(fcol(rows, col))
        if rs:
            summary.append(f"    RSS {p:<12} peak={f(rs['max']/1024,0)} MB")
    ts = [stats([(to_float(r.get(tc)) or None) for r in rows]) for tc in temp_cols]
    tmax = max((s["max"] for s in ts if s), default=None)
    if tmax is not None:
        summary.append(f"  peak SoC temp: {f(tmax/1000)}°C")
    wr = fcol(rows, "worker_restart_count")
    if wr:
        summary.append(f"  worker restarts during run: {int(wr[-1])}")
    summary.append(f"  → {out}")


# ------------------------- yield -------------------------

def plot_yield(yield_dir, out_dir, summary):
    files = [("detector", "detector.csv"), ("pick_logic", "pick_logic.csv"),
             ("target_3d", "target_3d.csv"), ("base", "base.csv")]
    data = {name: (load_rows(os.path.join(yield_dir, fn)) or [{}])[0] for name, fn in files}
    if not any(data.values()):
        summary.append("yield: CSV 없음 — 건너뜀")
        return

    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle("Yield — where/why pick targets are dropped", fontsize=13, fontweight="bold")
    summary.append("── YIELD ──")

    def bars(axis, row, prefix, title, rate_key=None):
        items = [(k[len(prefix):], to_float(v)) for k, v in row.items()
                 if k.startswith(prefix) and to_float(v) not in (None, 0)]
        items.sort(key=lambda x: x[1], reverse=True)
        if items:
            labels = [k for k, _ in items]
            vals = [v for _, v in items]
            axis.barh(range(len(vals)), vals, color=ACCENT, edgecolor="white", linewidth=1.0)
            axis.set_yticks(range(len(vals))); axis.set_yticklabels(labels, fontsize=7)
            axis.invert_yaxis()
            for i, v in enumerate(vals):
                axis.text(v, i, f" {int(v)}", va="center", fontsize=7)
        else:
            axis.text(0.5, 0.5, "(none)", ha="center", transform=axis.transAxes, color="#888")
        rate = to_float(row.get(rate_key)) if rate_key else None
        axis.set_title(f"{title}" + (f"  |  {rate_key}={rate:.2f}" if rate is not None else ""))
        axis.set_xlabel("count")

    # detector funnel (own style)
    a = ax[0, 0]
    d = data.get("detector", {})
    funnel = [("arrived", "frames_arrived"), ("processed", "frames_processed"),
              ("empty_out", "frames_empty_out"), ("overwritten", "frames_overwritten_dropped"),
              ("gate_skipped", "frames_gate_skipped")]
    fl = [(lab, to_float(d.get(col))) for lab, col in funnel if to_float(d.get(col)) is not None]
    if fl:
        a.barh(range(len(fl)), [v for _, v in fl], color=[PROC_COLOR["detector"]] * len(fl),
               edgecolor="white", linewidth=1.0)
        a.set_yticks(range(len(fl))); a.set_yticklabels([k for k, _ in fl], fontsize=8); a.invert_yaxis()
        for i, (_, v) in enumerate(fl):
            a.text(v, i, f" {int(v)}", va="center", fontsize=7)
    a.set_title("detector frame funnel"); a.set_xlabel("frames")

    bars(ax[0, 1], data.get("pick_logic", {}), "reject_", "pick_logic reject reasons", "target_valid_rate")
    bars(ax[1, 0], data.get("target_3d", {}), "fail_", "3d depth failure causes", "depth_valid_rate")
    bars(ax[1, 1], data.get("base", {}), "reject_", "base reject reasons", "final_target_valid_rate")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(out_dir, "yield.png")
    fig.savefig(out, dpi=130); plt.close(fig)

    # text summary
    for name, row in data.items():
        if not row:
            continue
        for rk in ("target_valid_rate", "depth_valid_rate", "final_target_valid_rate"):
            if rk in row:
                summary.append(f"  {name}: {rk}={f(to_float(row.get(rk)),3)}")
    d = data.get("detector", {})
    for k in ("worker_restart_count", "worker_timeout_count", "worker_slow_count"):
        if k in d:
            summary.append(f"  detector.{k}={d.get(k)}")
    summary.append(f"  → {out}")


# ------------------------- main -------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", default="evidence/metrics",
                   help="base dir holding performance/ resource/ yield/ (default evidence/metrics)")
    p.add_argument("--perf-dir"); p.add_argument("--resource"); p.add_argument("--yield-dir")
    p.add_argument("--out-dir")
    args = p.parse_args()

    perf_dir = args.perf_dir or os.path.join(args.base, "performance")
    resource = args.resource or os.path.join(args.base, "resource", "run.csv")
    yield_dir = args.yield_dir or os.path.join(args.base, "yield")
    out_dir = args.out_dir or os.path.join(args.base, "plots")
    os.makedirs(out_dir, exist_ok=True)

    summary = []
    plot_performance(perf_dir, out_dir, summary)
    plot_resource(resource, out_dir, summary)
    plot_yield(yield_dir, out_dir, summary)

    print("\n".join(summary))
    print(f"\nPNG 3개 저장 위치: {out_dir}")


if __name__ == "__main__":
    main()
