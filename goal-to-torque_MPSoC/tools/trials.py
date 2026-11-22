#!/usr/bin/env python3
"""trials.py - the trials of the main experiment (D-25, paper Sec. VII-A, eq. (9)): one run of the camera
application is one trial, judged without the camera.

A trial succeeds when the tool stays within TOL of the hover point of its object (the object plus the goal
offset of the run: APP_HOVER_XY_M, APP_HOVER_M) for HOLD seconds without a break, and that hold ends within
LIMIT seconds of the start of TRACK. The tool is the one of the high-level log (tool_x..z: the joint angles of
the period, forward kinematics), the object the place of the run: "object" in the .json of the log (the camera
of the simulation writes it), or --object for runs on one place. Its time is from the start of TRACK to the end
of the hold, or LIMIT when it fails; HOME before it is the same for every method and is left out.

Per method:
  trials, successes, the success rate with its Wilson 95 % interval (the trial is the unit, D-25)
  Y = successes / the sum of the trial times [1/min] (eq. (9): every method on the same time bound)
  the times of the successful trials (median, p90), the largest final distance of a failed one
  with --power (tools/host/power_log.py on the board, or resource/run.csv of the camera pipeline with
  metrics:=true): the mean power over the trials and the energy per success
A run that did not end as 'completed' (stopped, a fault) counts as a failed trial and is named.

Usage:
  python3 tools/trials.py LOG_OR_DIR [...] [--tol-mm MM | --cfg CFG] [--hold-s S] [--limit-s S]
                          [--object X Y Z] [--power CSV] [--out CSV]
  TOL default: [CONTRACT] DELTA_MM of --cfg (the task tolerance delta), else 10 mm. HOLD default 0.5 s. LIMIT
  default: the length of the run (APP_DURATION_S). All first values (D-25: set after the errors are measured).
  --out: one row per trial. Standard library only.
  The start of a trial in wall time is the stamp of its log name (whole seconds, the local time of the machine
  that wrote it), against the power samples: run this on that machine, or with its TZ (TZ=UTC python3 ...).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402

STAMP = re.compile(r'_r(\d+)_(\d{8})_(\d{6})\.csv$')
Z95 = 1.959964
ORDER = ['M1', 'M2', 'A2', 'A3', 'FULL']


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def wilson(k, n, z=Z95):
    """the Wilson score interval of k successes in n trials: (low, high)"""
    if n == 0:
        return float('nan'), float('nan')
    p = k / n
    d = 1.0 + z * z / n
    mid = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, mid - half), min(1.0, mid + half)


def start_unix(path):
    """the wall time of the start of the log [s since the epoch, local time of this machine], None without"""
    m = STAMP.search(os.path.basename(path))
    if not m:
        return None
    return time.mktime(time.strptime(m.group(2) + m.group(3), '%Y%m%d%H%M%S'))


def judge(times, dists, tol, hold, limit):
    """(success, the trial time): the first hold of `hold` s within tol that ends by `limit`, else (False, limit).
    times ascending [s from the start of TRACK], dists [m] of the same rows"""
    since = None                                     # the start of the current stay within tol
    for t, d in zip(times, dists):
        if t > limit + 1e-9:
            break
        if d <= tol:
            if since is None:
                since = t
            if t - since >= hold - 1e-9:
                return True, t
        else:
            since = None
    return False, limit


def trial(path, tol, hold, limit=None, obj=None):
    """the trial of one high-level log of the application"""
    with open(os.path.splitext(path)[0] + '.json') as f:
        meta = json.load(f)
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    out = {'log': os.path.basename(path), 'method': meta.get('method') or 'M1', 'run': meta.get('run'),
           'result': meta.get('result'), 'faults': meta.get('timing_faults', 0), 'start_unix': start_unix(path)}
    place = meta.get('object') or obj
    length = len(rows) * float(meta.get('period') or 0.001)
    out['limit'] = limit if limit is not None else length
    if place is None:
        out.update(success=False, time=out['limit'], why='no place of the object (the .json has none: --object)')
        return out
    off = meta.get('goal_offset') or [0.0, 0.0, 0.0]
    hover = [p + o for p, o in zip(place, off)]
    out['object'] = list(place)
    times = [num(r['t']) for r in rows]
    dists = [math.dist((num(r['tool_x']), num(r['tool_y']), num(r['tool_z'])), hover) for r in rows]
    ok, t = judge(times, dists, tol, hold, out['limit'])
    if out['limit'] > length + 1e-6:                 # a run shorter than the limit cannot show a failure in time
        out['why'] = 'the run (%.1f s) is shorter than the limit' % length
    if out['result'] != 'completed':
        ok, t = False, out['limit']
        out['why'] = 'the run ended %s' % out['result']
    out.update(success=ok, time=t, final_mm=dists[-1] * 1e3 if dists else float('nan'))
    return out


class Power:
    """the power samples of tools/host/power_log.py (unix_s, power_w), or of the resource monitor of the camera
    pipeline (resource/run.csv of metrics:=true: wall_time_s, ina260_u14_power1_input [uW])"""

    def __init__(self, path):
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        if rows and 'power_w' in rows[0]:
            self.t = [num(r['unix_s']) for r in rows]
            self.w = [num(r['power_w']) for r in rows]
        elif rows and 'ina260_u14_power1_input' in rows[0]:
            self.t = [num(r['wall_time_s']) for r in rows]
            self.w = [num(r['ina260_u14_power1_input']) * 1e-6 for r in rows]
        else:
            sys.exit('trials: %s has neither unix_s,power_w nor wall_time_s,ina260_u14_power1_input' % path)

    def energy(self, t0, t1):
        """(the energy between t0 and t1 [J], the samples in it); the mean of the samples times the span"""
        inside = [w for t, w in zip(self.t, self.w) if t0 <= t <= t1 and w == w]
        return (sum(inside) / len(inside) * (t1 - t0), len(inside)) if inside else (float('nan'), 0)


def quantile(vals, q):
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * len(s)))] if s else float('nan')


def summary(trials, power=None):
    """{method: the numbers of its trials}"""
    out = {}
    for m in sorted({t['method'] for t in trials}, key=lambda m: (ORDER.index(m) if m in ORDER else 99, m)):
        ts = [t for t in trials if t['method'] == m]
        k = sum(1 for t in ts if t['success'])
        total = sum(t['time'] for t in ts)
        lo, hi = wilson(k, len(ts))
        ok_times = [t['time'] for t in ts if t['success']]
        s = {'trials': len(ts), 'successes': k, 'rate': k / len(ts) if ts else float('nan'), 'low': lo,
             'high': hi, 'time_s': total, 'Y_per_min': 60.0 * k / total if total > 0 else float('nan'),
             'median_s': quantile(ok_times, 0.5), 'p90_s': quantile(ok_times, 0.9),
             'worst_fail_mm': max([t.get('final_mm', float('nan')) for t in ts if not t['success']] or
                                  [float('nan')]),
             'not_completed': [t['log'] for t in ts if t['result'] != 'completed']}
        if power is not None:
            # the mean power over the trials that have samples, for the energy of all the trial time: a trial
            # without samples (the log began late) does not count as 0 W
            e = covered = 0.0
            n = m_trials = 0
            for t in ts:
                if t['start_unix'] is None:
                    continue
                de, dn = power.energy(t['start_unix'], t['start_unix'] + t['time'])
                if dn:
                    e, covered, n, m_trials = e + de, covered + t['time'], n + dn, m_trials + 1
            s['power_w'] = e / covered if covered > 0 else float('nan')
            s['energy_per_success_j'] = s['power_w'] * total / k if k and covered > 0 else float('nan')
            s['power_samples'], s['power_trials'] = n, m_trials
        out[m] = s
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--tol-mm', type=float)
    ap.add_argument('--cfg', help='TOL from its [CONTRACT] DELTA_MM')
    ap.add_argument('--hold-s', type=float, default=0.5)
    ap.add_argument('--limit-s', type=float)
    ap.add_argument('--object', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--power', help='CSV of tools/host/power_log.py, or resource/run.csv of the camera pipeline')
    ap.add_argument('--out', help='CSV, one row per trial')
    a = ap.parse_args()
    tol_mm = a.tol_mm
    if tol_mm is None and a.cfg:
        tol_mm = num(jp.cfg_get(a.cfg, 'CONTRACT', 'DELTA_MM'))
    if tol_mm is None or not tol_mm > 0:
        tol_mm = 10.0
    if not a.hold_s >= 0 or (a.limit_s is not None and not a.limit_s > 0):
        sys.exit('trials: --hold-s not negative, --limit-s above 0')
    paths = []
    for p in a.logs:
        paths += sorted(glob.glob(os.path.join(p, '**', 'hl_app_*.csv'), recursive=True)) if os.path.isdir(p) else [p]
    if not paths:
        sys.exit('trials: no hl_app_*.csv in %s' % ', '.join(a.logs))
    trials = [trial(p, tol_mm * 1e-3, a.hold_s, a.limit_s, a.object) for p in paths]
    power = Power(a.power) if a.power else None

    print('trials: %d runs; success: the tool within %.1f mm of the hover point for %.2f s, by the limit '
          '(%s)' % (len(trials), tol_mm, a.hold_s, '%.1f s' % a.limit_s if a.limit_s else 'the length of a run'))
    for t in trials:
        if t.get('why'):
            print('  %s: %s' % (t['log'], t['why']))
    print('%-6s %6s %9s %8s %15s %9s %9s %9s %9s'
          % ('method', 'trials', 'successes', 'rate', 'Wilson 95 %', 'Y [/min]', 'median s', 'p90 s', 'fail mm'))
    table = summary(trials, power)
    for m, s in table.items():
        print('%-6s %6d %9d %7.1f%% %6.1f - %5.1f%% %9.2f %9.2f %9.2f %9.1f' % (
            m, s['trials'], s['successes'], 100 * s['rate'], 100 * s['low'], 100 * s['high'], s['Y_per_min'],
            s['median_s'], s['p90_s'], s['worst_fail_mm']))
        if power is not None:
            print('       power over the trials %.2f W (%d samples, %d of %d trials), energy per success %.1f J'
                  % (s['power_w'], s['power_samples'], s['power_trials'], s['trials'], s['energy_per_success_j']))
        if s['not_completed']:
            print('       not completed (failed trials): %s' % ', '.join(s['not_completed']))
    if a.out:
        keys = ['method', 'run', 'log', 'success', 'time', 'limit', 'final_mm', 'result', 'faults', 'object']
        with open(a.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            for t in trials:
                w.writerow(dict(t, object=' '.join('%.4f' % v for v in t.get('object') or [])))
        print('trials: -> %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
