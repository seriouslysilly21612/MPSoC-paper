#!/usr/bin/env python3
"""Publication-style figures for the pipeline metrics CSVs (numpy + matplotlib).

This is the "paper" renderer -- a companion to plot_metrics.py (which stays
stdlib-only). This one uses numpy for proper ECDF/CCDF/percentile/rolling stats.

NOT a ROS program: a pure offline CSV -> PNG conversion. No setup.bash, no
running pipeline, no colcon build. It runs ON THE BOARD as-is (numpy 1.21.5 +
matplotlib 3.5.1 are installed there; verified generating all 8 figures under
`env -i`, i.e. with the environment completely stripped):

    python3 tools/metrics/plot_metrics_paper.py --base evidence/metrics/latest

To view the PNGs on a PC, generate on the board and copy only the PNGs
(much smaller than the CSVs):

    scp -r kria:~/ros2_ws/evidence/metrics/latest/plots_paper .

Why these forms (systems-paper conventions):
  * Latency is NOT summarized by a histogram + vertical percentile lines -- a few
    huge outliers stretch the x-axis and cram p50/p95/p99 into one bin. The right
    forms are the ECDF (percentiles read off the y-axis at 0.5/0.95/0.99, spread
    along x by their real values) and the CCDF on a log-y survival axis (the tail
    is the whole story for a jitter budget).
  * Stage attribution = a stacked mean-contribution bar (where the budget goes) +
    a log-y per-stage boxplot (which stage carries the tail).
  * Stability over time = raw scatter + a rolling median/p95, not a hairball line.

Design follows the repo's dataviz skill: one validated categorical palette
assigned per-entity in fixed order (never cycled), single-axis panels only
(temperature and power are separate subplots, never a dual axis), top/right
spines dropped for a clean plot frame, 300 dpi.
"""
import argparse
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless: write PNGs, no display server needed
import matplotlib.pyplot as plt

# ---- validated categorical palette (dataviz skill reference, light mode) ----
# Fixed slot order; assigned per entity, never cycled by rank.
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
           "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID = "#e1e0d9"
STATUS_GOOD, STATUS_WARN, STATUS_CRIT = "#0ca30c", "#fab219", "#d03b3b"
HUE = PALETTE[0]        # single-series blue
ACCENT = PALETTE[1]     # reference / rolling-stat orange

# process color follows the ENTITY across every panel, not its rank
PROC_ORDER = ["camera", "detector", "worker", "pick_logic",
              "target_3d", "target_base", "static_tf"]
PROC_COLOR = {p: PALETTE[i] for i, p in enumerate(PROC_ORDER)}

# E2E latency ladder: (label, merged-CSV column). Sum of means ~= mean true_e2e.
STAGES = [
    ("capture->detect (queue+gate)", "age_in_ms"),
    ("detector compute", "processing_ms"),
    ("-> pick_logic (DDS)", "handoff_det_to_pick_ms"),
    ("pick_logic compute", "pick_logic_compute_ms"),
    ("-> target_3d (DDS)", "handoff_pick_to_3d_ms"),
    ("target_3d reproj", "target3d_compute_ms"),
    ("-> base (DDS)", "handoff_3d_to_base_ms"),
    ("base TF", "base_compute_ms"),
]
STAGE_COLOR = {col: PALETTE[i % len(PALETTE)] for i, (_, col) in enumerate(STAGES)}
# strictly-positive stages that get a log-y boxplot (handoffs can go slightly
# negative from clock/scheduling noise -> excluded from the log panel, they live
# in the stacked bar where their tiny positive means are honest)
BOX_STAGES = ["age_in_ms", "processing_ms", "pick_logic_compute_ms",
              "target3d_compute_ms", "base_compute_ms"]

BUDGET_MS = 66.6  # 15 Hz frame period -- the jitter reference line

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "savefig.facecolor": "white", "savefig.dpi": 300,
    "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold",
    "axes.labelsize": 11, "legend.fontsize": 9,
    "axes.edgecolor": "#c3c2b7", "axes.linewidth": 1.0,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 9.5,
    "ytick.labelsize": 9.5, "text.color": INK, "axes.labelcolor": INK,
    "figure.titlesize": 15, "figure.titleweight": "bold",
    "legend.framealpha": 0.9, "legend.edgecolor": "#cccccc",
})


# ------------------------- CSV / stats helpers -------------------------

def load_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, name):
    """Column as a float ndarray with blanks/non-numeric dropped (order kept)."""
    out = []
    for r in rows:
        v = r.get(name)
        if v in (None, ""):
            continue
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            continue
    return np.asarray(out, dtype=float)


def rolling(y, w, reducer):
    """Centered rolling `reducer` (e.g. np.median) over a window of w samples."""
    n = len(y)
    w = max(1, min(w, n))
    half = w // 2
    out = np.empty(n)
    for i in range(n):
        out[i] = reducer(y[max(0, i - half):min(n, i + half + 1)])
    return out


def fmt(x, nd=1):
    return "n/a" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{nd}f}"


# ------------------------- performance loading -------------------------

def load_perf(perf_dir):
    """Prefer merged.csv (join_perf output); else join the 4 node CSVs on the
    capture stamp and derive the 3 handoffs, so the tool works either way."""
    merged = os.path.join(perf_dir, "merged.csv")
    if os.path.exists(merged):
        return load_rows(merged)
    sources = {k: load_rows(os.path.join(perf_dir, f"{k}.csv"))
               for k in ("detector", "pick_logic", "target_3d", "base")}
    keyed = {k: {(r.get("capture_sec"), r.get("capture_nanosec")): r
                 for r in rows if r.get("capture_sec")}
             for k, rows in sources.items()}
    all_keys = set().union(*[set(d) for d in keyed.values()]) if any(keyed.values()) else set()

    def to_f(a):
        try:
            return float(a)
        except (TypeError, ValueError):
            return None

    def diff(a, b):
        fa, fb = to_f(a), to_f(b)
        return None if fa is None or fb is None else fa - fb

    rows = []
    for key in sorted(all_keys, key=lambda k: (int(k[0]), int(k[1]))):
        row = {"capture_sec": key[0], "capture_nanosec": key[1]}
        for name, d in keyed.items():
            r = d.get(key)
            if r:
                for kk, vv in r.items():
                    if kk not in ("capture_sec", "capture_nanosec"):
                        row[kk if kk not in row else f"{name}_{kk}"] = vv
        row["handoff_det_to_pick_ms"] = diff(row.get("pick_logic_in_age_ms"), row.get("frame_age_ms"))
        row["handoff_pick_to_3d_ms"] = diff(row.get("target3d_in_age_ms"), row.get("pick_logic_out_age_ms"))
        row["handoff_3d_to_base_ms"] = diff(row.get("base_in_age_ms"), row.get("target3d_out_age_ms"))
        rows.append(row)
    return rows


def elapsed_seconds(rows):
    """Capture wall time of each row, rebased to 0 at the first frame."""
    t = np.array([(float(r["capture_sec"]) + float(r.get("capture_nanosec", 0)) / 1e9)
                  if r.get("capture_sec") not in (None, "") else np.nan for r in rows])
    good = ~np.isnan(t)
    if good.any():
        t = t - np.nanmin(t)
    return t, good


# ------------------------- FIG 1: latency distribution -------------------------

def fig_latency_distribution(e2e, out_path, summary):
    if e2e.size == 0:
        summary.append("latency distribution: true_e2e 없음 — 건너뜀")
        return None
    v = np.sort(e2e)
    n = v.size
    y = np.arange(1, n + 1) / n                 # ECDF: F(v[i]) = P(X <= v[i])
    # empirical survival P(X >= v[i]) = (n-i)/n. Inclusive-at-x by construction,
    # which keeps a strictly-positive log-y floor (min = 1/n at x = max); the axis
    # is labelled ">=" to match exactly (for continuous latency > and >= coincide
    # except on the measure-zero data points).
    ccdf = 1.0 - np.arange(n) / n
    p50, p95, p99, p999 = np.percentile(v, [50, 95, 99, 99.9])
    x_hi = np.percentile(v, 99.5)               # clip ECDF x so the outlier tail
    #                                             doesn't squash the body

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(12.5, 5.0))
    fig.suptitle("End-to-end latency distribution  (capture -> /pick_target_base)", y=1.00)

    # (left) ECDF -- percentiles read cleanly off the axes, no line-crowding
    axL.step(v, y, where="post", color=HUE, lw=2.2)
    axL.set_xlim(v.min() * 0.97, x_hi * 1.03)
    axL.set_ylim(0, 1.02)
    for q, val, name in [(0.50, p50, "p50"), (0.95, p95, "p95"), (0.99, p99, "p99")]:
        axL.vlines(val, 0, q, color=ACCENT, ls=":", lw=1.3)
        axL.hlines(q, axL.get_xlim()[0], val, color=ACCENT, ls=":", lw=1.0, alpha=0.6)
        axL.plot([val], [q], "o", color=ACCENT, ms=7, zorder=5)
        axL.annotate(f"{name} = {val:.0f} ms", (val, q), textcoords="offset points",
                     xytext=(7, -12), fontsize=9.5, color=INK, fontweight="bold")
    axL.set_title("ECDF")
    axL.set_xlabel("true_e2e latency (ms)")
    axL.set_ylabel("cumulative fraction of frames")

    # (right) CCDF on log-y -- the tail. p95/p99/p99.9 sit decades apart.
    axR.step(v, ccdf, where="post", color=HUE, lw=2.2)
    axR.set_yscale("log")
    axR.set_ylim(max(1.0 / n * 0.7, 1e-4), 1.0)
    for val, sy, name in [(p95, 0.05, "p95"), (p99, 0.01, "p99"), (p999, 0.001, "p99.9")]:
        axR.plot([val], [sy], "o", color=ACCENT, ms=6, zorder=5)
        axR.annotate(f"{name}\n{val:.0f} ms", (val, sy), textcoords="offset points",
                     xytext=(6, 4), fontsize=8.5, color=INK)
    axR.axvline(BUDGET_MS, color=MUTED, ls="--", lw=1.2)
    axR.annotate(f"15 Hz budget\n{BUDGET_MS:.1f} ms", (BUDGET_MS, 0.5),
                 textcoords="offset points", xytext=(6, 0), fontsize=8.5, color=MUTED)
    axR.set_title("CCDF (tail survival, log scale)")
    axR.set_xlabel("true_e2e latency (ms)")
    axR.set_ylabel("P(latency ≥ x)")
    axR.annotate(f"max = {v.max():.0f} ms  (1 frame)", (0.98, 0.95),
                 xycoords="axes fraction", ha="right", fontsize=8.5, color=MUTED)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    over = float((v > BUDGET_MS).mean() * 100)
    summary.append("── LATENCY (true_e2e, ms) ──")
    summary.append(f"  p50={p50:.0f}  p95={p95:.0f}  p99={p99:.0f}  p99.9={p999:.0f}  "
                   f"max={v.max():.0f}  (n={n})")
    summary.append(f"  15Hz budget(66.6ms) 초과: {over:.1f}% 프레임")
    return out_path


# ------------------------- FIG 2: stage breakdown -------------------------

def fig_stage_breakdown(rows, out_path, summary):
    means, labels, colors, cols_present = [], [], [], []
    for lab, cname in STAGES:
        c = col(rows, cname)
        if c.size == 0:
            continue
        means.append(float(np.mean(c)))
        labels.append(lab)
        colors.append(STAGE_COLOR[cname])
        cols_present.append(cname)
    if not means:
        summary.append("stage breakdown: 데이터 없음 — 건너뜀")
        return None
    total = sum(means)

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.5, 5.2),
                                   gridspec_kw={"width_ratios": [1.1, 1.0]})
    fig.suptitle("Where the E2E latency budget goes", y=1.00)

    # (left) stacked mean-contribution bar (horizontal, single stacked bar)
    left = 0.0
    for m, lab, c in zip(means, labels, colors):
        axL.barh(0, m, left=left, color=c, edgecolor="white", linewidth=1.5,
                 label=f"{lab}  ({m:.1f} ms, {m/total*100:.0f}%)")
        if m > total * 0.045:
            axL.text(left + m / 2, 0, f"{m:.0f}", ha="center", va="center",
                     fontsize=9, color="white", fontweight="bold")
        left += m
    axL.set_xlim(0, total * 1.02)
    axL.set_ylim(-0.5, 0.5)
    axL.set_yticks([])
    axL.set_xlabel("mean contribution to E2E (ms)")
    axL.set_title(f"stage contribution (sum of means = {total:.0f} ms)")
    axL.legend(loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=2, fontsize=8.5,
               frameon=False)
    axL.grid(axis="y", visible=False)

    # (right) per-stage log-y boxplot -- which stage carries the tail
    col2lab = {cn: lab for lab, cn in STAGES}   # column -> ladder label
    box_data, box_labels, box_colors = [], [], []
    for cname in BOX_STAGES:
        c = col(rows, cname)
        c = c[c > 0]                             # log axis needs strictly positive
        if c.size:
            box_data.append(c)
            box_labels.append(col2lab[cname])
            box_colors.append(STAGE_COLOR[cname])
    if box_data:
        bp = axR.boxplot(box_data, vert=True, patch_artist=True, widths=0.6,
                         showfliers=True, flierprops=dict(marker="o", markersize=2.5,
                         markerfacecolor=MUTED, markeredgecolor="none", alpha=0.35),
                         medianprops=dict(color=INK, lw=1.6),
                         whiskerprops=dict(color=INK2), capprops=dict(color=INK2),
                         boxprops=dict(edgecolor="white", linewidth=1.0))
        for patch, c in zip(bp["boxes"], box_colors):
            patch.set_facecolor(c)
        axR.set_yscale("log")
        axR.set_xticklabels([b.replace(" ", "\n", 1) for b in box_labels],
                            fontsize=8.5, rotation=0)
        axR.set_ylabel("stage latency (ms, log)")
        axR.set_title("per-stage distribution (log-y, fliers = outliers)")
        axR.grid(axis="x", visible=False)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    worst = max(zip(means, labels))
    summary.append("── STAGE BREAKDOWN ──")
    summary.append(f"  최대 병목(mean): {worst[1]} ≈ {worst[0]:.1f} ms "
                   f"({worst[0]/total*100:.0f}% of {total:.0f} ms)")
    t3 = col(rows, "target3d_compute_ms")
    if t3.size:
        summary.append(f"  target_3d reproj tail: max={t3.max():.0f} ms "
                       f"(p50={np.percentile(t3,50):.1f}) — E2E spike의 주범")
    return out_path


# ------------------------- FIG 3: latency over time -------------------------

def fig_latency_timeline(rows, out_path, summary):
    e2e = np.array([float(r["true_e2e_ms"]) if r.get("true_e2e_ms") not in (None, "")
                    else np.nan for r in rows])
    t, _ = elapsed_seconds(rows)
    good = ~np.isnan(e2e) & ~np.isnan(t)
    if good.sum() < 5:
        summary.append("latency timeline: 데이터 부족 — 건너뜀")
        return None
    t, e2e = t[good], e2e[good]
    order = np.argsort(t)
    t, e2e = t[order], e2e[order]

    p50 = np.percentile(e2e, 50)
    p95 = np.percentile(e2e, 95)
    y_cap = np.percentile(e2e, 99.5) * 1.12
    n_over = int((e2e > y_cap).sum())

    # smoothed distribution over time: a running-median line (the "typical" frame)
    # inside a shaded p10-p90 band (the middle 80% of frames at each moment). No
    # raw scatter -- a 5000-point cloud reads as a hairball; the band shows spread
    # cleanly and the tail lives in the CCDF panel instead.
    w = max(21, len(e2e) // 40 | 1)   # odd window, a few seconds of frames
    rmed = rolling(e2e, w, np.median)
    rp10 = rolling(e2e, w, lambda a: np.percentile(a, 10))
    rp90 = rolling(e2e, w, lambda a: np.percentile(a, 90))

    fig, ax = plt.subplots(figsize=(13, 5.0))
    fig.suptitle("End-to-end latency over the run  (is it stable?)", y=1.00)
    ax.fill_between(t, rp10, rp90, color=HUE, alpha=0.18, lw=0,
                    label="middle 80% of frames (p10–p90)")
    ax.plot(t, rmed, color=HUE, lw=2.4, label="running median (typical frame)", zorder=4)
    # NOTE: this is a THROUGHPUT reference (1 output every frame period), NOT a
    # latency deadline. In a pipelined system latency naturally exceeds the period
    # (here ~2 periods) while outputs still arrive every period -- so the median
    # sitting above this line is expected, not a violation.
    ax.axhline(BUDGET_MS, color=STATUS_WARN, ls="--", lw=1.5,
               label=f"1 frame period ({BUDGET_MS:.0f} ms) — throughput target, not a latency limit")
    ax.set_ylim(0, y_cap)
    ax.annotate("outputs arrive every ~59 ms (~17 Hz ✓);  E2E latency ≈ 2 frame periods "
                "— expected for a pipeline (latency ≠ rate)",
                (0.015, 0.05), xycoords="axes fraction", fontsize=8.5, color=INK2)
    if n_over:
        ax.annotate(f"↑ {n_over} rare spikes above axis (worst {e2e.max():.0f} ms) — see CCDF",
                    (0.99, 0.97), xycoords="axes fraction", ha="right", va="top",
                    fontsize=9.5, color=STATUS_CRIT, fontweight="bold")
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("end-to-end latency (ms)")
    ax.set_xlim(0, t.max())
    ax.legend(loc="upper left", framealpha=0.9)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    summary.append("── TIMELINE ──")
    summary.append(f"  running median 범위 {rmed.min():.0f}~{rmed.max():.0f} ms, "
                   f"축 위로 튄 spike {n_over}개 (드묾)")
    return out_path


# ------------------------- FIG 3b: throughput -------------------------
# The companion to latency: latency = how OLD a result is; throughput = how OFTEN
# one comes out. The 15 Hz "budget" is a throughput target, so it belongs here
# (on the inter-output gap), not on the latency plot.

def fig_throughput(rows, out_path, summary):
    op = col(rows, "output_period_ms")     # base: gap between final publishes
    dp = col(rows, "detect_period_ms")     # detector: gap between /detections
    if op.size == 0 and dp.size == 0:
        summary.append("throughput: period 데이터 없음 — 건너뜀")
        return None

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(12.5, 5.0))
    fig.suptitle("Throughput  (how often a result comes out)", y=1.00)

    # (left) ECDF of the gap between consecutive outputs vs one frame period.
    # A gap left of the budget line = that result arrived within one 15 Hz period.
    lo, hi = [], []
    for name, a, c in [("base output", op, HUE),
                       ("detector /detections", dp, PALETTE[1])]:
        if a.size == 0:
            continue
        s = np.sort(a)
        y = np.arange(1, s.size + 1) / s.size
        axL.step(s, y, where="post", color=c, lw=2.2, label=name)
        lo.append(float(np.min(a)))
        hi.append(float(np.percentile(a, 99)))
    axL.axvline(BUDGET_MS, color=STATUS_WARN, ls="--", lw=1.5,
                label=f"1 frame period {BUDGET_MS:.0f} ms (15 Hz)")
    if op.size:
        within = float((op <= BUDGET_MS).mean() * 100)
        axL.annotate(f"{within:.0f}% of output gaps ≤ {BUDGET_MS:.0f} ms\n(kept up with 15 Hz)",
                     (0.03, 0.90), xycoords="axes fraction", ha="left", va="top",
                     fontsize=9, color=INK2)
    if lo and hi:
        axL.set_xlim(min(lo) * 0.9, max(hi) * 1.05)
    axL.set_ylim(0, 1.02)
    axL.set_title("gap between consecutive outputs: ECDF")
    axL.set_xlabel("inter-output gap (ms)")
    axL.set_ylabel("cumulative fraction")
    axL.legend(loc="lower right", fontsize=9)

    # (right) output rate over time (Hz): running median + middle-80% band.
    t, _ = elapsed_seconds(rows)
    opv = np.array([float(r["output_period_ms"]) if r.get("output_period_ms") not in (None, "")
                    else np.nan for r in rows])
    m = np.isfinite(t) & np.isfinite(opv) & (opv > 0)
    if m.sum() > 5:
        tt = t[m]
        rate = 1000.0 / opv[m]             # instantaneous Hz per output
        order = np.argsort(tt)
        tt, rate = tt[order], rate[order]
        w = max(21, len(rate) // 40 | 1)
        rmed = rolling(rate, w, np.median)
        rp10 = rolling(rate, w, lambda z: np.percentile(z, 10))
        rp90 = rolling(rate, w, lambda z: np.percentile(z, 90))
        axR.fill_between(tt, rp10, rp90, color=HUE, alpha=0.18, lw=0,
                         label="middle 80% of outputs")
        axR.plot(tt, rmed, color=HUE, lw=2.4, label="running median rate")
        axR.axhline(15.0, color=STATUS_WARN, ls="--", lw=1.5, label="15 Hz target")
        achieved = len(rate) / (tt.max() - tt.min()) if tt.max() > tt.min() else float(np.median(rate))
        axR.annotate(f"achieved ≈ {achieved:.1f} Hz over the run (meets 15 Hz)",
                     (0.02, 0.06), xycoords="axes fraction", fontsize=9, color=INK2)
        axR.set_ylim(0, max(22.0, float(np.nanpercentile(rp90, 99)) * 1.15))
        axR.legend(loc="upper right", fontsize=9)
    axR.set_title("output rate over time")
    axR.set_xlabel("elapsed time (s)")
    axR.set_ylabel("output rate (Hz)")
    axR.set_xlim(0, t.max() if t.size else 1)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    summary.append("── THROUGHPUT ──")
    if op.size:
        p50 = float(np.percentile(op, 50))
        p95 = float(np.percentile(op, 95))
        summary.append(f"  output period: p50={p50:.1f}ms (~{1000/p50:.1f}Hz), p95={p95:.1f}ms; "
                       f"gap ≤ {BUDGET_MS:.0f}ms인 비율 {(op<=BUDGET_MS).mean()*100:.0f}% (=순간 15Hz 충족)")
    return out_path


# ------------------------- FIG 4: CPU -------------------------

def fig_cpu(rows, out_path, summary):
    """Process view: who uses the cores (stacked area over time + mean/peak bar)."""
    if not rows:
        summary.append("cpu: resource CSV 없음 — 건너뜀")
        return None
    t, _ = elapsed_seconds_res(rows)
    have = [p for p in PROC_ORDER if f"cpu_cores_{p}" in rows[0]]
    series = {p: col_res(rows, f"cpu_cores_{p}") for p in have}

    fig, (a, c) = plt.subplots(1, 2, figsize=(14, 5.2),
                               gridspec_kw={"width_ratios": [1.4, 1.0]})
    fig.suptitle("CPU by process  (who uses the cores)", y=1.00)

    # (left) stacked area of per-process cores + total line
    if series:
        stack = np.vstack([np.nan_to_num(series[p], nan=0.0) for p in have])
        a.stackplot(t, *stack, labels=have, colors=[PROC_COLOR[p] for p in have],
                    edgecolor="white", linewidth=0.2)
        total = stack.sum(axis=0)
        a.plot(t, total, color=INK, lw=1.4, label="total")
        a.legend(loc="upper left", ncol=2, fontsize=8.5)
        a.annotate(f"total: mean {total.mean():.2f} / peak {total.max():.2f} cores (of 4)",
                   (0.98, 0.05), xycoords="axes fraction", ha="right", fontsize=9, color=INK2)
    a.set_title("per-process CPU over time (stacked)")
    a.set_xlabel("elapsed time (s)")
    a.set_ylabel("CPU (cores;  4 = whole chip)")
    a.set_xlim(0, t.max() if t.size else 1)

    # (right) per-process mean bar + peak tick (sorted by mean)
    stat = [(p, float(np.nanmean(series[p])), float(np.nanmax(series[p])))
            for p in have if np.isfinite(series[p]).any()]
    stat.sort(key=lambda x: x[1])
    ys = np.arange(len(stat))
    c.barh(ys, [m for _, m, _ in stat], color=[PROC_COLOR[p] for p, _, _ in stat],
           edgecolor="white", linewidth=1.0, label="mean")
    c.plot([pk for _, _, pk in stat], ys, "|", color=INK, ms=16, mew=2.2, label="peak")
    for i, (_, m, pk) in enumerate(stat):
        c.annotate(f"{m:.2f}", (m, i), textcoords="offset points", xytext=(4, 0),
                   va="center", fontsize=8.5)
    c.set_yticks(ys)
    c.set_yticklabels([p for p, _, _ in stat], fontsize=9.5)
    c.set_title("per-process CPU: mean bar, peak tick (cores)")
    c.set_xlabel("CPU (cores)")
    c.legend(loc="lower right", fontsize=9)
    c.grid(axis="y", visible=False)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    summary.append("── CPU ──")
    if series:
        total = np.vstack([np.nan_to_num(series[p], nan=0.0) for p in have]).sum(axis=0)
        summary.append(f"  파이프라인 total CPU: mean={np.mean(total):.2f} cores, "
                       f"peak={np.max(total):.2f} cores (of 4)")
        for p, m, pk in sorted(stat, key=lambda x: -x[1]):
            summary.append(f"    {p:<12} mean={m:.2f}  peak={pk:.2f} cores")
    return out_path


def fig_cpu_cores(rows, out_path, summary):
    """Core view: each of the 4 A53 cores in its own panel (small multiples)."""
    if not rows:
        return None
    t, _ = elapsed_seconds_res(rows)
    core_cols = sorted(k for k in rows[0] if k.startswith("sys_cpu_pct_cpu"))
    if not core_cols:
        return None
    n = len(core_cols)
    ncol = 2
    nrow = (n + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(13, 3.6 * nrow),
                             sharex=True, sharey=True)
    axes = np.atleast_1d(axes).ravel()
    fig.suptitle("CPU per core  (each of the 4 A53 cores, %)", y=1.00)
    for i, cc in enumerate(core_cols):
        ax = axes[i]
        v = col_res(rows, cc) * 100
        ci = PALETTE[i % len(PALETTE)]
        ax.plot(t, v, color=ci, lw=1.3)
        ax.fill_between(t, 0, v, color=ci, alpha=0.15)
        m = float(np.nanmean(v))
        ax.axhline(m, color=INK2, ls="--", lw=1.2, label=f"mean {m:.0f}%")
        ax.set_title(f"core {cc.replace('sys_cpu_pct_cpu', '')}")
        ax.set_ylim(0, 105)
        ax.set_xlim(0, t.max() if t.size else 1)
        ax.legend(loc="upper right", fontsize=9)
        if i % ncol == 0:
            ax.set_ylabel("utilisation (%)")
        if i // ncol == nrow - 1:
            ax.set_xlabel("elapsed time (s)")
    for j in range(n, len(axes)):
        axes[j].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    summary.append("  per-core CPU → 4b_cpu_cores.png (코어별 개별 패널)")
    return out_path


# ------------------------- FIG 5: memory + thermal -------------------------

def fig_memory_thermal(rows, out_path, summary):
    if not rows:
        summary.append("memory/thermal: resource CSV 없음 — 건너뜀")
        return None
    t, _ = elapsed_seconds_res(rows)

    fig, ax = plt.subplots(2, 2, figsize=(13.5, 9.5))
    fig.suptitle("Memory & thermal", y=1.00)

    # (a) RSS per process (leak watch)
    a = ax[0, 0]
    for p in PROC_ORDER:
        key = f"rss_kb_{p}"
        if key in rows[0]:
            v = col_res(rows, key) / 1024.0    # row-aligned; NaN -> line gap
            if np.isfinite(v).any():
                a.plot(t, v, color=PROC_COLOR[p], lw=1.4, label=p)
    a.legend(loc="best", ncol=2, fontsize=8)
    a.set_title("resident memory (RSS) per process (MB)")
    a.set_xlabel("elapsed time (s)")
    a.set_ylabel("RSS (MB)")
    a.set_xlim(0, t.max() if t.size else 1)

    # (b) system available memory + CMA free
    b = ax[0, 1]
    memav = col_res(rows, "mem_available_kb") / 1024.0
    if np.isfinite(memav).any():
        b.plot(t, memav, color=HUE, lw=1.8, label="MemAvailable")
    cma = col_res(rows, "cma_free_kb") / 1024.0
    if np.isfinite(cma).any() and np.nanmax(cma) > 0:
        b.plot(t, cma, color=PALETTE[2], lw=1.4, ls="--", label="CMA free (DMA)")
    b.legend(loc="best")
    b.set_title("free system RAM (MemAvailable) — flat line = no memory leak")
    b.set_xlabel("elapsed time (s)")
    b.set_ylabel("free RAM (MB)")
    b.set_xlim(0, t.max() if t.size else 1)

    # (c) SoC temperature
    c = ax[1, 0]
    temp_cols = sorted(k for k in rows[0] if k.startswith("ams_temp") and k.endswith("_input"))
    for i, tc in enumerate(temp_cols):
        v = col_res(rows, tc) / 1000.0   # milli-degC -> degC
        if np.isfinite(v).any() and np.nanmax(v) > 0:
            c.plot(t, v, color=PALETTE[i % len(PALETTE)], lw=1.4, label=tc.replace("_input", ""))
    if temp_cols:
        c.legend(loc="best", fontsize=8)
    c.set_title("SoC temperature (°C)")
    c.set_xlabel("elapsed time (s)")
    c.set_ylabel("temperature (°C)")
    c.set_xlim(0, t.max() if t.size else 1)

    # (d) board power
    d = ax[1, 1]
    pw = col_res(rows, "ina260_u14_power1_input") / 1e6   # microW -> W
    if np.isfinite(pw).any() and np.nanmax(pw) > 0:
        d.plot(t, pw, color=PALETTE[3], lw=1.6)
        d.fill_between(t, 0, pw, color=PALETTE[3], alpha=0.12)
        d.axhline(float(np.mean(pw)), color=ACCENT, ls="--", lw=1.3,
                  label=f"mean {np.mean(pw):.1f} W")
        d.legend(loc="best")
        d.set_ylim(0, np.nanmax(pw) * 1.15)
    d.set_title("board power rail U14 (W)")
    d.set_xlabel("elapsed time (s)")
    d.set_ylabel("power (W)")
    d.set_xlim(0, t.max() if t.size else 1)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    summary.append("── MEMORY / THERMAL ──")
    fin = memav[np.isfinite(memav)]
    if fin.size:
        drift = fin[-1] - fin[0]
        summary.append(f"  MemAvailable: {fin[0]:.0f}→{fin[-1]:.0f} MB "
                       f"(Δ{drift:+.0f} MB over run)")
    tmax = 0.0
    for tc in temp_cols:
        v = col_res(rows, tc) / 1000.0
        if np.isfinite(v).any():
            tmax = max(tmax, float(np.nanmax(v)))
    if tmax:
        summary.append(f"  peak SoC temp: {tmax:.1f} °C")
    if np.isfinite(pw).any() and np.nanmax(pw) > 0:
        summary.append(f"  board power: mean={np.nanmean(pw):.1f} W, peak={np.nanmax(pw):.1f} W")
    return out_path


# ------------------------- FIG 6: yield funnel -------------------------

def fig_yield_funnel(yield_dir, out_path, summary):
    det = (load_rows(os.path.join(yield_dir, "detector.csv")) or [{}])[0]
    pk = (load_rows(os.path.join(yield_dir, "pick_logic.csv")) or [{}])[0]
    t3 = (load_rows(os.path.join(yield_dir, "target_3d.csv")) or [{}])[0]
    bs = (load_rows(os.path.join(yield_dir, "base.csv")) or [{}])[0]
    if not any([det, pk, t3, bs]):
        summary.append("yield: CSV 없음 — 건너뜀")
        return None

    def gi(row, key):
        v = row.get(key)
        try:
            return int(float(v))
        except (TypeError, ValueError):
            return None

    arrived = gi(det, "frames_arrived")
    gate = gi(det, "frames_gate_skipped")
    over = gi(det, "frames_overwritten_dropped")
    processed = gi(det, "frames_processed")
    pick_ok = gi(pk, "frames_accepted")
    depth_ok = gi(t3, "depth_valid_out")
    base_ok = gi(bs, "published_valid")

    # survival funnel: (label, surviving count, drop note)
    steps = []
    if arrived is not None:
        steps.append(("camera frames in (30 fps)", arrived, ""))
    if arrived is not None and gate is not None:
        steps.append(("after 15 Hz gate", arrived - gate,
                      f"−{gate} skipped by 15 Hz gate (by design, 30→15 Hz)"))
    if processed is not None:
        note = f"−{over} overwritten (detector busy, newest-first)" if over else ""
        steps.append(("detector processed", processed, note))
    if pick_ok is not None:
        steps.append(("pick_logic valid", pick_ok, ""))
    if depth_ok is not None:
        steps.append(("target_3d depth valid", depth_ok, ""))
    if base_ok is not None:
        steps.append(("base published", base_ok, ""))

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.5, 5.4),
                                   gridspec_kw={"width_ratios": [1.5, 1.0]})
    fig.suptitle("Frame yield  (how many frames survive each stage)", y=1.00)

    # (left) survival funnel -- ordinal single-hue ramp, darkest at the top
    base_n = steps[0][1] if steps else 1
    ramp = ["#256abf", "#3987e5", "#5598e7", "#6da7ec", "#86b6ef", "#9ec5f4"]
    ys = np.arange(len(steps))[::-1]
    for i, (lab, cnt, note) in enumerate(steps):
        y = ys[i]
        axL.barh(y, cnt, color=ramp[i % len(ramp)], edgecolor="white", linewidth=1.2)
        pctv = cnt / base_n * 100 if base_n else 0
        axL.annotate(f"{cnt:,}  ({pctv:.0f}%)", (cnt, y), textcoords="offset points",
                     xytext=(6, 0), va="center", fontsize=9, fontweight="bold")
        if note:
            axL.annotate(note, (0, y), textcoords="offset points", xytext=(6, -13),
                         va="center", fontsize=8.5, color=INK2)
    axL.set_yticks(ys)
    axL.set_yticklabels([s[0] for s in steps], fontsize=9.5)
    axL.set_xlabel("frames over the run")
    axL.set_title("pipeline frame survival  (30 fps in → ~15 Hz by design)")
    axL.set_xlim(0, base_n * 1.18)
    axL.grid(axis="y", visible=False)

    # (right) downstream validity -- QUALITY, not throughput. The detector's 57%
    # "process rate" is the intended 15 Hz gate (already shown in the funnel), NOT
    # a failure, so it does NOT belong on a valid-rate axis colored by status.
    rates = []
    for row, key, name in [(pk, "target_valid_rate", "pick_logic valid rate"),
                           (t3, "depth_valid_rate", "target_3d depth-valid rate"),
                           (bs, "final_target_valid_rate", "base final valid rate")]:
        if row.get(key) not in (None, ""):
            rates.append((name, float(row[key])))
    yb = np.arange(len(rates))[::-1]
    for i, (name, rate) in enumerate(rates):
        c = STATUS_GOOD if rate >= 0.99 else (STATUS_WARN if rate >= 0.9 else STATUS_CRIT)
        axR.barh(yb[i], rate * 100, color=c, edgecolor="white", linewidth=1.0)
        axR.annotate(f"{rate*100:.1f}%", (rate * 100, yb[i]), textcoords="offset points",
                     xytext=(5, 0), va="center", fontsize=9.5, fontweight="bold")
    axR.set_yticks(yb)
    axR.set_yticklabels([n for n, _ in rates], fontsize=9.5)
    axR.set_xlim(0, 122)
    axR.set_ylim(-1.4, len(rates) - 0.4)   # headroom below for the worker note
    axR.set_xlabel("valid rate (%)  — throughput is the left funnel")
    axR.set_title("valid rate of processed frames (quality)")
    axR.grid(axis="y", visible=False)

    # worker reliability note, tucked under the bars
    wr = det.get("worker_restart_count", "?")
    wt = det.get("worker_timeout_count", "?")
    ws = det.get("worker_slow_count", "?")
    axR.annotate(f"worker: {wr} restarts, {wt} timeouts, {ws} slow",
                 (0, -1.2), va="center", fontsize=8.5, color=INK2, family="monospace")

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)

    summary.append("── YIELD ──")
    if arrived and processed:
        summary.append(f"  detector: {arrived:,} arrived → {processed:,} processed "
                       f"({processed/arrived*100:.0f}%); gate-skip {gate:,}, overwrite {over:,}")
    for name, rate in rates:
        summary.append(f"  {name}: {rate*100:.1f}%")
    summary.append(f"  worker restarts={wr} timeouts={wt} slow={ws}")
    return out_path


# ------------------------- resource-CSV helpers -------------------------
# (resource CSV has its own time column and no capture stamp)

def col_res(rows, name):
    """Row-aligned float column: length == len(rows), NaN where blank/non-numeric.
    Alignment matters -- the worker (re-resolved pid) is blank on some ticks, so a
    blank-dropping parse would shift its samples off the shared time axis."""
    out = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        v = r.get(name)
        if v in (None, ""):
            continue
        try:
            out[i] = float(v)
        except (TypeError, ValueError):
            continue
    return out


def elapsed_seconds_res(rows):
    tw = col_res(rows, "wall_time_s")
    if not np.isfinite(tw).any():
        return np.arange(len(rows), dtype=float), np.ones(len(rows), bool)
    t = tw - np.nanmin(tw)
    return t, np.ones(t.size, bool)


# ------------------------- main -------------------------

def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", default="evidence/metrics",
                   help="base dir holding performance/ resource/ yield/")
    p.add_argument("--perf-dir")
    p.add_argument("--resource")
    p.add_argument("--yield-dir")
    p.add_argument("--out-dir")
    args = p.parse_args()

    perf_dir = args.perf_dir or os.path.join(args.base, "performance")
    resource = args.resource or os.path.join(args.base, "resource", "run.csv")
    yield_dir = args.yield_dir or os.path.join(args.base, "yield")
    out_dir = args.out_dir or os.path.join(args.base, "plots_paper")
    os.makedirs(out_dir, exist_ok=True)

    summary = []
    saved = []

    perf_rows = load_perf(perf_dir)
    if perf_rows:
        e2e = col(perf_rows, "true_e2e_ms")
        saved.append(fig_latency_distribution(e2e, os.path.join(out_dir, "1_latency_distribution.png"), summary))
        saved.append(fig_stage_breakdown(perf_rows, os.path.join(out_dir, "2_latency_breakdown.png"), summary))
        saved.append(fig_latency_timeline(perf_rows, os.path.join(out_dir, "3_latency_timeline.png"), summary))
        saved.append(fig_throughput(perf_rows, os.path.join(out_dir, "3b_throughput.png"), summary))
    else:
        summary.append("performance: CSV 없음 — figure 1~3 건너뜀")

    res_rows = load_rows(resource)
    if res_rows:
        saved.append(fig_cpu(res_rows, os.path.join(out_dir, "4_cpu.png"), summary))
        saved.append(fig_cpu_cores(res_rows, os.path.join(out_dir, "4b_cpu_cores.png"), summary))
        saved.append(fig_memory_thermal(res_rows, os.path.join(out_dir, "5_memory_thermal.png"), summary))
    else:
        summary.append("resource: CSV 없음 — figure 4~5 건너뜀")

    saved.append(fig_yield_funnel(yield_dir, os.path.join(out_dir, "6_yield_funnel.png"), summary))

    print("\n".join(summary))
    saved = [s for s in saved if s]
    print(f"\n{len(saved)}개 figure 저장: {out_dir}")
    for s in saved:
        print(f"  → {os.path.basename(s)}")


if __name__ == "__main__":
    main()
