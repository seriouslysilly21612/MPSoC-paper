#!/usr/bin/env python3
"""Offline join of the per-node performance CSVs into one E2E table + stats.

Each pipeline node writes its own performance CSV (it is a separate process, so
no node sees all stages). This joins them on the propagated capture stamp
(capture_sec, capture_nanosec) -- unique, monotonic, byte-exact -- and derives
the 3 inter-node handoffs as pure differences of paired age columns (zero
runtime cost; the real work already happened online). It then reports p50/p95/
p99/max/mean/std for latency columns -- for a jitter-sensitive baseline the mean
('E2E ~81ms') is the wrong summary; the tail is what matters.

    python3 tools/metrics/join_perf.py \
        --detector  evidence/metrics/latest/performance/detector.csv \
        --pick-logic evidence/metrics/latest/performance/pick_logic.csv \
        --target-3d evidence/metrics/latest/performance/target_3d.csv \
        --base      evidence/metrics/latest/performance/base.csv \
        --out-merged evidence/metrics/latest/performance/merged.csv \
        --out-summary evidence/metrics/latest/performance/summary.csv

Every CSV arg is optional; whatever is given is joined.
"""
import argparse
import csv
import os


def to_float(v):
    if v is None or v == "":
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def diff(a, b):
    fa, fb = to_float(a), to_float(b)
    if fa is None or fb is None:
        return None
    return round(fa - fb, 3)


def percentile(sorted_data, q):
    n = len(sorted_data)
    if n == 0:
        return None
    if n == 1:
        return sorted_data[0]
    idx = (n - 1) * q / 100.0
    lo = int(idx)
    hi = min(lo + 1, n - 1)
    frac = idx - lo
    return sorted_data[lo] * (1 - frac) + sorted_data[hi] * frac


def stats(values):
    vals = [v for v in (to_float(x) for x in values) if v is not None]
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    mean = sum(s) / n
    var = sum((x - mean) ** 2 for x in s) / n if n > 1 else 0.0
    return {
        "n": n,
        "mean": round(mean, 3),
        "std": round(var ** 0.5, 3),
        "p50": round(percentile(s, 50), 3),
        "p95": round(percentile(s, 95), 3),
        "p99": round(percentile(s, 99), 3),
        "min": round(s[0], 3),
        "max": round(s[-1], 3),
    }


def load_keyed(path):
    rows = {}
    if not path or not os.path.exists(path):
        return rows
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            sec = r.get("capture_sec")
            nsec = r.get("capture_nanosec")
            if sec in (None, "") or nsec in (None, ""):
                continue
            rows[(sec, nsec)] = r
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--detector")
    p.add_argument("--pick-logic")
    p.add_argument("--target-3d")
    p.add_argument("--base")
    p.add_argument("--out-merged")
    p.add_argument("--out-summary")
    args = p.parse_args()

    sources = {
        "detector": load_keyed(args.detector),
        "pick_logic": load_keyed(args.pick_logic),
        "target_3d": load_keyed(args.target_3d),
        "base": load_keyed(args.base),
    }

    all_keys = set()
    for d in sources.values():
        all_keys |= set(d.keys())
    if not all_keys:
        print("No rows loaded -- check the CSV paths (need capture_sec/nanosec).")
        return

    def key_sort(k):
        try:
            return (int(k[0]), int(k[1]))
        except (TypeError, ValueError):
            return (0, 0)

    merged = []
    for key in sorted(all_keys, key=key_sort):
        row = {"capture_sec": key[0], "capture_nanosec": key[1]}
        for name, src in sources.items():
            r = src.get(key)
            if not r:
                continue
            for k, v in r.items():
                if k in ("capture_sec", "capture_nanosec"):
                    continue
                # namespace on collision (e.g. target_valid appears in both
                # pick_logic and base) so a later stage never silently clobbers
                # an earlier stage's column
                row[f"{name}_{k}" if k in row else k] = v
        # Inter-node handoffs = downstream in_age - upstream out_age. Both are
        # ages vs the SAME capture stamp, so the subtraction is valid offline.
        # Detector's publish anchor = frame_age_ms (capture -> end of process).
        row["handoff_det_to_pick_ms"] = diff(
            row.get("pick_logic_in_age_ms"), row.get("frame_age_ms"))
        row["handoff_pick_to_3d_ms"] = diff(
            row.get("target3d_in_age_ms"), row.get("pick_logic_out_age_ms"))
        row["handoff_3d_to_base_ms"] = diff(
            row.get("base_in_age_ms"), row.get("target3d_out_age_ms"))
        merged.append(row)

    # column set = union, capture keys first
    fieldnames = ["capture_sec", "capture_nanosec"]
    for r in merged:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)

    if args.out_merged:
        d = os.path.dirname(args.out_merged)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(args.out_merged, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(merged)
        print(f"Wrote {len(merged)} merged rows to {args.out_merged}")

    # summarize the headline latency columns present in the data
    headline = [
        "true_e2e_ms", "frame_age_ms", "age_in_ms",
        "handoff_det_to_pick_ms", "handoff_pick_to_3d_ms", "handoff_3d_to_base_ms",
        "pick_logic_compute_ms", "target3d_compute_ms", "base_compute_ms",
        "processing_ms", "dpu_ms", "detect_period_ms", "output_period_ms",
        "depth_vs_color_skew_ms",
    ]
    summary_rows = []
    print("\n=== latency summary (ms) — tail matters, not mean ===")
    print(f"{'metric':<26}{'n':>6}{'p50':>9}{'p95':>9}{'p99':>9}{'max':>9}"
          f"{'mean':>9}{'std':>8}")
    for col in headline:
        if not any(col in r for r in merged):
            continue
        st = stats([r.get(col) for r in merged])
        if st is None:
            continue
        summary_rows.append({"metric": col, **st})
        print(f"{col:<26}{st['n']:>6}{st['p50']:>9}{st['p95']:>9}{st['p99']:>9}"
              f"{st['max']:>9}{st['mean']:>9}{st['std']:>8}")

    if args.out_summary and summary_rows:
        d = os.path.dirname(args.out_summary)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(args.out_summary, "w", newline="") as f:
            w = csv.DictWriter(
                f, fieldnames=["metric", "n", "mean", "std", "p50", "p95",
                               "p99", "min", "max"])
            w.writeheader()
            w.writerows(summary_rows)
        print(f"\nWrote summary to {args.out_summary}")


if __name__ == "__main__":
    main()
