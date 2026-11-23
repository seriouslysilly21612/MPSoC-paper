#!/usr/bin/env python3
"""mode_energy.py - the energy of one inference of each mode (energy_j of mode_profile.yaml), from runs of
tools/run_timing.py: the board power with the inference against the power of the same pipeline without it.

  energy of an inference = (mean power of the mode's run - mean power of the base run) / inferences a second

The inferences are the images that the rule ran, OUT/decisions.csv (method A3 or FULL, one model a run: --model
of run_timing), over the time from the first to the last of them; the power is OUT/temp_power.csv of the run
(board_W, the INA260 once a second) over the same time (its unix_s), less --tail-s at each end. The base run is
the pipeline without inference: method M1 with detect_period_s:=1000 (the period of M1's gate; the rule of FULL
does not use it); its power from --skip-s after its start (the start of the pipeline) to --tail-s before its end.
What the difference holds: the DPU and the CPU work of an inference (the pre- and post-processing, the result
into the mailbox); not the camera, ROS or the control task, which run in both. It needs the same scene, load
and arch in all the runs. With OUT/apm.csv in the runs (tools/host/ddr_apm.py during the run) the DDR bytes of
an inference too, the same way (ddr_bytes): the bytes a second of the ports S1-S5 above the base, per inference.

Without --base, the base of a mode's run is its own time before its first inference: from its first decision + 5 s
(the pipeline is up and rejects every image for want of a status) to 2 s before its first inference. On the board
(2026-11-22, I-136) that agreed with a run of its own within 0.05 W and 5 MB/s, and it leaves out the differences
between runs.

Usage:
  python3 tools/mode_energy.py [--base DIR] MODE:DIR [MODE:DIR ...] [--skip-s 60] [--tail-s 10]
  e.g. 0:~/timing_logs/en_416 1:~/timing_logs/en_320
Prints per mode the two powers, the rate and energy_j, and the lines to put into mode_profile.yaml.
Standard library only.
"""
import argparse
import csv
import os
import sys


def mean_power(run_dir, skip_s, tail_s, span=None):
    """(mean board power [W], samples) of a run: between skip_s from its start and tail_s before its end, or
    within span (unix_s, unix_s) less tail_s at each end"""
    path = os.path.join(run_dir, 'temp_power.csv')
    try:
        with open(path, newline='') as f:
            rows = [(float(r['t_s']), float(r['board_W']), float(r['unix_s']) if span else 0.0)
                    for r in csv.DictReader(f)]
    except (OSError, KeyError, ValueError) as ex:
        raise ValueError('%s: %s' % (path, ex)) from None
    rows = [(t, w, u) for t, w, u in rows if w == w]
    if not rows:
        raise ValueError('%s: no power samples' % path)
    if span:
        lo, hi = span[0] + tail_s, span[1] - tail_s
        inside = [w for _, w, u in rows if lo <= u <= hi]
    else:
        lo, hi = skip_s, rows[-1][0] - tail_s
        inside = [w for t, w, _ in rows if lo <= t <= hi]
    if len(inside) < 10:
        raise ValueError('%s: %d samples between %.0f and %.0f, fewer than 10' % (path, len(inside), lo, hi))
    return sum(inside) / len(inside), len(inside)


def inference_rate(run_dir):
    """(inferences a second, inferences, (first, last) [unix s]) from the decisions of the run: the images run"""
    path = os.path.join(run_dir, 'decisions.csv')
    try:
        with open(path, newline='') as f:
            times = sorted(float(r['release_s']) for r in csv.DictReader(f) if r['action'] == 'run')
    except (OSError, KeyError, ValueError) as ex:
        raise ValueError('%s: %s' % (path, ex)) from None
    if len(times) < 2 or times[-1] <= times[0]:
        raise ValueError('%s: %d images run, too few for a rate' % (path, len(times)))
    return (len(times) - 1) / (times[-1] - times[0]), len(times), (times[0], times[-1])


def ddr_rate(run_dir, skip_s, tail_s, span=None):
    """the DDR bytes a second of a run (tools/host/ddr_apm.py, all its ports): the samples that end between skip_s
    from the first and tail_s before the last, or within span (unix_s, unix_s) less tail_s at each end, as the power;
    None without OUT/apm.csv"""
    path = os.path.join(run_dir, 'apm.csv')
    if not os.path.exists(path):
        return None
    try:
        with open(path, newline='') as f:
            rows = [(float(r['unix_s']), float(r['dt_s']), sum(float(v) for k, v in r.items() if k.endswith('_B')))
                    for r in csv.DictReader(f)]
    except (OSError, KeyError, ValueError) as ex:
        raise ValueError('%s: %s' % (path, ex)) from None
    if not rows:
        raise ValueError('%s: no samples' % path)
    if span:
        lo, hi = span[0] + tail_s, span[1] - tail_s
    else:
        lo, hi = rows[0][0] + skip_s, rows[-1][0] - tail_s
    inside = [(dt, b) for u, dt, b in rows if lo <= u - dt and u <= hi]
    seconds = sum(dt for dt, _ in inside)
    if seconds < 10.0:
        raise ValueError('%s: %.1f s of samples between %.0f and %.0f, less than 10' % (path, seconds, lo, hi))
    return sum(b for _, b in inside) / seconds


def before_first(run_dir):
    """(from, to) [unix s] of a run with no inference yet: its first decision + 5 s to 2 s before its first one run"""
    path = os.path.join(run_dir, 'decisions.csv')
    try:
        with open(path, newline='') as f:
            rows = [(float(r['release_s']), r['action']) for r in csv.DictReader(f)]
    except (OSError, KeyError, ValueError) as ex:
        raise ValueError('%s: %s' % (path, ex)) from None
    runs = [t for t, act in rows if act == 'run']
    if not rows or not runs:
        raise ValueError('%s: no decisions, or no image run' % path)
    lo, hi = min(t for t, _ in rows) + 5.0, min(runs) - 2.0
    if hi - lo < 10.0:
        raise ValueError('%s: %.0f s before the first inference, less than 10' % (path, hi - lo))
    return lo, hi


def energy(base_w, mode_w, rate):
    """[J] of one inference"""
    return (mode_w - base_w) / rate


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', help='a run without inference (FULL with a table that passes no mode); default: the '
                                   'time of each run before its first inference')
    ap.add_argument('modes', nargs='+', metavar='MODE:DIR')
    ap.add_argument('--skip-s', type=float, default=60.0)
    ap.add_argument('--tail-s', type=float, default=10.0)
    a = ap.parse_args()
    if not a.skip_s >= 0 or not a.tail_s >= 0:
        sys.exit('mode_energy: --skip-s and --tail-s not negative')
    try:
        if a.base:
            base_w, base_n = mean_power(os.path.expanduser(a.base), a.skip_s, a.tail_s)
            base_ddr = ddr_rate(os.path.expanduser(a.base), a.skip_s, a.tail_s)
            print('base %s: %.3f W (%d samples)%s' % (a.base, base_w, base_n, '' if base_ddr is None else
                                                      ', DDR %.1f MB/s' % (base_ddr / 1e6)))
        lines = []
        for item in a.modes:
            mode, sep, run_dir = item.partition(':')
            if not sep or not mode.isdigit():
                sys.exit('mode_energy: %r is not MODE:DIR' % item)
            run_dir = os.path.expanduser(run_dir)
            rate, k, span = inference_rate(run_dir)
            if not a.base:
                pre = before_first(run_dir)
                base_w, base_n = mean_power(run_dir, 0.0, 0.0, pre)
                base_ddr = ddr_rate(run_dir, 0.0, 0.0, pre)
                print('mode %s, before its first inference: %.3f W (%d samples)%s' % (
                    mode, base_w, base_n, '' if base_ddr is None else ', DDR %.1f MB/s' % (base_ddr / 1e6)))
            w, n = mean_power(run_dir, a.skip_s, a.tail_s, span)
            e = energy(base_w, w, rate)
            print('mode %s %s: %.3f W (%d samples), %d inferences, %.2f a second: %+.3f W, energy_j %.4f'
                  % (mode, run_dir, w, n, k, rate, w - base_w, e))
            if e <= 0:
                print('  not above the base: the difference is within the noise of the power, or the runs differ')
            lines.append('  %s: energy_j: %.4f' % (mode, e))
            ddr = ddr_rate(run_dir, a.skip_s, a.tail_s, span)
            if ddr is not None and base_ddr is not None:
                per = (ddr - base_ddr) / rate
                print('  DDR %.1f MB/s: %+.1f MB/s, ddr_bytes %.4g' % (ddr / 1e6, (ddr - base_ddr) / 1e6, per))
                lines.append('  %s: ddr_bytes: %.1f' % (mode, per))       # a plain float: YAML 1.1 reads 4e+07 as text
    except ValueError as ex:
        sys.exit('mode_energy: %s' % ex)
    print('mode_profile.yaml, modes:\n' + '\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
