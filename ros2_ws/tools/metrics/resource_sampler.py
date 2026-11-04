#!/usr/bin/env python3
"""Standalone resource sampler for the KV260 pick & place pipeline.

Only an EXTERNAL observer can see every pipeline OS process -- including the
detector's VART worker SUBPROCESS, the realsense node, and static_transform_
publisher -- that no single ROS node can enumerate. Run it alongside the launch:

    python3 tools/metrics/resource_sampler.py \
        --output evidence/metrics/resource/run1.csv --duration 185 --interval 1.0

Writes ONE CSV at the end (duration elapsed or Ctrl-C). Per-process CPU is in
A53 cores (0-4); memory in kB. Cheap: ~25 small /proc reads per 1 Hz tick
(< 0.1 core). NOTE the worker can restart mid-run (pid changes) -- pids are
re-resolved every tick and worker_restart_count counts pid changes, otherwise
the worker's numbers would silently drop. See evidence/metrics/README.md.
"""
import argparse
import csv
import glob
import os
import signal
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")  # ticks/sec, usually 100

# label -> substring matched against /proc/<pid>/cmdline. The worker and the
# detector are DISTINCT processes (worker = vitis_ai_worker_yolo.py subprocess).
PROC_PATTERNS = [
    ("camera", "realsense2_camera"),
    ("detector", "vitis_ai_detector_node"),
    ("worker", "vitis_ai_worker_yolo"),
    ("pick_logic", "pick_logic"),
    ("target_3d", "pick_target_3d_node"),
    ("target_base", "pick_target_base_node"),
    ("static_tf", "static_transform_publisher"),
]


def read_file(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def find_pids():
    """label -> pid (first cmdline match). Missing processes are just absent."""
    me = os.getpid()
    found = {}
    for cmdline_path in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            pid = int(cmdline_path.split("/")[2])
        except (ValueError, IndexError):
            continue
        if pid == me:
            continue
        raw = read_file(cmdline_path)
        if not raw:
            continue
        cmd = raw.replace("\x00", " ")
        for label, needle in PROC_PATTERNS:
            if label not in found and needle in cmd:
                found[label] = pid
    return found


def parse_stat(pid):
    """(cpu_ticks, majflt, last_cpu) from /proc/<pid>/stat, or None.

    comm (field 2) can contain spaces/parens, so split AFTER the last ')':
    the first token then is field 3, and field F maps to index F-3.
    """
    content = read_file(f"/proc/{pid}/stat")
    if not content:
        return None
    rpar = content.rfind(")")
    if rpar < 0:
        return None
    fields = content[rpar + 2:].split()
    try:
        majflt = int(fields[9])     # field 12
        utime = int(fields[11])     # field 14
        stime = int(fields[12])     # field 15
        last_cpu = int(fields[36])  # field 39 (processor)
    except (IndexError, ValueError):
        return None
    return (utime + stime, majflt, last_cpu)


def parse_status(pid):
    """VmRSS, VmSwap (kB), involuntary ctxt-switch count from status."""
    content = read_file(f"/proc/{pid}/status")
    out = {}
    if not content:
        return out
    for line in content.splitlines():
        if line.startswith("VmRSS:"):
            out["rss_kb"] = int(line.split()[1])
        elif line.startswith("VmSwap:"):
            out["swap_kb"] = int(line.split()[1])
        elif line.startswith("nonvoluntary_ctxt_switches:"):
            out["nonvol_ctxt"] = int(line.split()[1])
    return out


def parse_pss(pid):
    """Proportional set size (kB): shared .so pages divided among sharers.
    One kernel-aggregated read (heaviest per-proc read -> sampled less often)."""
    content = read_file(f"/proc/{pid}/smaps_rollup")
    if not content:
        return None
    for line in content.splitlines():
        if line.startswith("Pss:"):
            return int(line.split()[1])
    return None


def parse_proc_stat_cpu():
    """{'cpu'|'cpu0'..: (busy_ticks, total_ticks)} from /proc/stat."""
    content = read_file("/proc/stat")
    out = {}
    if not content:
        return out
    for line in content.splitlines():
        if not line.startswith("cpu"):
            continue
        parts = line.split()
        vals = [int(x) for x in parts[1:]]
        # user nice system idle iowait irq softirq steal guest guest_nice
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)  # idle + iowait
        total = sum(vals)
        out[parts[0]] = (total - idle, total)
    return out


def parse_meminfo():
    content = read_file("/proc/meminfo")
    out = {}
    if not content:
        return out
    wanted = {
        "MemAvailable": "mem_available_kb",
        "SwapFree": "swap_free_kb",
        "CmaFree": "cma_free_kb",   # CMA pool the DPU/PL draw from
    }
    for line in content.splitlines():
        key = line.split(":")[0]
        if key in wanted:
            out[wanted[key]] = int(line.split()[1])
    return out


def discover_hwmon():
    """[(column, path)] for temp/power/curr/voltage/fan _input + pwm sysfs nodes.
    KV260 has no cpufreq/DVFS, so these are the only throttle-precursor signals.
    Values are raw sysfs units (temp=milli-degC, power=microW) -- see README."""
    cols = []
    prefixes = ("temp", "power", "curr", "in", "fan")
    for hw in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        name = (read_file(os.path.join(hw, "name")) or "hwmon").strip()
        try:
            entries = sorted(os.listdir(hw))
        except OSError:
            continue
        for f in entries:
            is_input = f.endswith("_input") and any(f.startswith(p) for p in prefixes)
            is_pwm = f.startswith("pwm") and f[3:].isdigit()
            if is_input or is_pwm:
                cols.append((f"{name}_{f}", os.path.join(hw, f)))
    return cols


def write_csv(path, rows):
    if not rows:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    fieldnames = []
    for r in rows:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True, help="output CSV path (absolute)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to sample; 0 = until Ctrl-C")
    p.add_argument("--interval", type=float, default=1.0, help="sample period (s)")
    p.add_argument("--pss-every", type=int, default=5,
                   help="sample PSS (smaps_rollup) every Nth tick (heavier read)")
    return p.parse_args()


def main():
    args = parse_args()
    hwmon_cols = discover_hwmon()

    stop = {"flag": False}

    def _stop(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    pids = find_pids()
    print(f"[resource_sampler] tracking pids: {pids}", file=sys.stderr)
    print(f"[resource_sampler] hwmon sensors: {len(hwmon_cols)}", file=sys.stderr)
    worker_pid = pids.get("worker")
    worker_restart_count = 0

    prev_cpu = {}   # label -> (cpu_ticks, majflt)
    for label, pid in pids.items():
        st = parse_stat(pid)
        if st is not None:
            prev_cpu[label] = (st[0], st[1])
    prev_sys = parse_proc_stat_cpu()

    rows = []
    start = time.monotonic()
    prev_wall = start
    tick = 0

    while not stop["flag"]:
        deadline = start + (tick + 1) * args.interval
        while not stop["flag"] and time.monotonic() < deadline:
            time.sleep(0.05)
        if stop["flag"]:
            break
        tick += 1

        now = time.monotonic()
        dt = now - prev_wall
        prev_wall = now

        pids = find_pids()
        wp = pids.get("worker")
        if wp is not None and worker_pid is not None and wp != worker_pid:
            worker_restart_count += 1
        if wp is not None:
            worker_pid = wp

        row = {"wall_time_s": round(time.time(), 3), "tick": tick}

        for label, _needle in PROC_PATTERNS:
            pid = pids.get(label)
            if pid is None:
                row[f"cpu_cores_{label}"] = None
                row[f"rss_kb_{label}"] = None
                prev_cpu.pop(label, None)
                continue
            st = parse_stat(pid)
            if st is not None:
                ticks, majflt, last_cpu = st
                prev = prev_cpu.get(label)
                if prev is not None and dt > 0 and ticks >= prev[0]:
                    row[f"cpu_cores_{label}"] = round(
                        (ticks - prev[0]) / CLK_TCK / dt, 4)
                    row[f"majflt_{label}"] = max(0, majflt - prev[1])
                else:
                    row[f"cpu_cores_{label}"] = None
                    row[f"majflt_{label}"] = None
                row[f"last_cpu_{label}"] = last_cpu
                prev_cpu[label] = (ticks, majflt)
            status = parse_status(pid)
            row[f"rss_kb_{label}"] = status.get("rss_kb")
            row[f"swap_kb_{label}"] = status.get("swap_kb")
            row[f"nonvol_ctxt_{label}"] = status.get("nonvol_ctxt")
            if args.pss_every > 0 and tick % args.pss_every == 0:
                row[f"pss_kb_{label}"] = parse_pss(pid)

        sys_cpu = parse_proc_stat_cpu()
        for key, (busy, total) in sys_cpu.items():
            prev = prev_sys.get(key)
            if prev is not None and (total - prev[1]) > 0:
                col = "sys_cpu_pct_total" if key == "cpu" else f"sys_cpu_pct_{key}"
                row[col] = round((busy - prev[0]) / (total - prev[1]), 4)
        prev_sys = sys_cpu

        row.update(parse_meminfo())

        for col, path in hwmon_cols:
            v = read_file(path)
            if v is not None:
                v = v.strip()
                row[col] = int(v) if v.lstrip("-").isdigit() else None

        row["worker_restart_count"] = worker_restart_count
        rows.append(row)

        if args.duration > 0 and (now - start) >= args.duration:
            break

    write_csv(args.output, rows)
    print(f"[resource_sampler] wrote {len(rows)} rows to {args.output}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
