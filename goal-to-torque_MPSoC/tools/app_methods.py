#!/usr/bin/env python3
"""app_methods.py - runs of the camera application side by side, one line per run with its method (--method of the
controller: M1, M2, A2, A3, FULL).

Reads the high-level logs of the application (hl_app_*.csv with their .json, in the cfg JOINT_TEST_LOG_DIR) and,
for a method with the mailbox, the log of the camera results of the same run (obs_<method>_*_rNN_*.csv with its
.json). Prints for every run
  used        the camera results that the control task used, of those it took out of the mailbox (M1: none, the
              bridge writes the goal)
  goals       the goals that the high-level controller accepted (a goal counts when it is more than delta from
              the accepted one)
  first       the time of the first goal [s]: how long the arm waited for the camera
  rest        the time at which the arm came to rest after its last goal [s]
and, when the place of the object is known (--object, or --cfg with SIM_CAM_OBJECT_M: the camera of the
simulation), against that place
  object      the error of the object of the results used: largest, mean [mm]
  goal        the largest error of the goal that the high-level controller read [mm]
  tool        the distance of the tool to the hover point above the object at the end of the run [mm]
Then, for a method with the mailbox: the verdicts of the results, their age when they were used, and for a
method without the contract (M2, A3) the error of each object against the way of the tool between the capture and
the joint angles that the control task used for it. (The tag sits 85 mm from the tool point: the two ways differ
where the tool turns.)

--check TOL_MM: exit 1 unless, in every run with the contract (A2, FULL), every result used has its object
within TOL_MM of the object and the tool ends within TOL_MM of the hover point. Needs the place of the object.

Usage:
  python3 tools/app_methods.py [LOG_OR_DIR ...] [--object X Y Z | --cfg FILE] [--check TOL_MM]
  (default indy7/jt_logs). Standard library only.
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402

DEFAULT_DIR = os.path.join(HERE, '..', 'indy7', 'jt_logs')
STAMP = re.compile(r'_r(\d+)_(\d{8})_(\d{6})\.csv$')
VERDICTS = ['accepted', 'superseded', 'expired', 'state_unrecoverable', 'seq_fail', 'error_exceeded', 'evicted']
HL_AT_REST, HL_HAS_GOAL = 32, 64        # flags of the high-level log (TrajectoryRT.h)
# why the camera side did not write a result (ObsWriter.h), why the workspace check refused one (GoalFilter.h)
NOT_WRITTEN = [('invalid', 'invalid'), ('not_finite', 'not finite'), ('age', 'older than the cap or from the future'),
               ('full', 'with no free slot'), ('expired', 'past their deadline')]
OUTSIDE = [('invalid', 'invalid'), ('not_finite', 'not finite'), ('below_floor', 'below the floor'),
           ('outside_box', 'outside the box or the radius')]


def goal_times(rows):
    """(the time of the first goal, the time the arm came to rest after its last accepted goal) [s]; None: never"""
    first = next((num(r['t']) for r in rows if r['goal_x'] != ''), None)
    last_goal, accepted = None, None
    for r in rows:
        goal = (r['acc_x'], r['acc_y'], r['acc_z'])
        if int(num(r['flags'])) & HL_HAS_GOAL and goal != accepted:
            last_goal, accepted = num(r['t']), goal
    if last_goal is None:
        return first, None
    return first, next((num(r['t']) for r in rows if num(r['t']) > last_goal and int(num(r['flags'])) & HL_AT_REST),
                       None)


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    with open(os.path.splitext(path)[0] + '.json') as f:
        meta = json.load(f)
    return meta, rows


def seconds(path):
    """the time of day in the name of a log [s], None without one"""
    m = STAMP.search(os.path.basename(path))
    return (m.group(2), int(m.group(3)[:2]) * 3600 + int(m.group(3)[2:4]) * 60 + int(m.group(3)[4:])) if m else None


def obs_log(hl_path, meta):
    """the log of the camera results of the same run: same method, platform and run, written within 5 s"""
    platform = 'sim' if meta.get('platform') == 'simulation' else 'robot'
    pat = os.path.join(os.path.dirname(hl_path), 'obs_%s_%s_r%02d_*.csv' % (meta.get('method', ''), platform,
                                                                              int(meta.get('run', 0))))
    here = seconds(hl_path)
    best = None
    for p in glob.glob(pat):
        s = seconds(p)
        if here and s and s[0] == here[0] and abs(s[1] - here[1]) <= 5 and (best is None or
                                                                             abs(s[1] - here[1]) < best[0]):
            best = (abs(s[1] - here[1]), p)
    return best[1] if best else None


def xyz(row, name):
    return (num(row[name + '_x']), num(row[name + '_y']), num(row[name + '_z']))


def quantile(vals, q):
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * len(s)))]


def analyze(hl_path, obj):
    meta, rows = load(hl_path)
    out = {'log': os.path.basename(hl_path), 'method': meta.get('method', 'M1'), 'run': meta.get('run'),
           'goals': meta.get('accepted'), 'result': meta.get('result'), 'faults': meta.get('timing_faults', 0)}
    out['first'], out['rest'] = goal_times(rows)
    hover = (meta.get('goal_offset') or [0.0, 0.0, 0.0])[2]
    tool = [xyz(r, 'tool') for r in rows]
    if obj and rows:
        point = (obj[0], obj[1], obj[2] + hover)
        errs = [math.dist(xyz(r, 'goal'), point) for r in rows if r['goal_x'] != '']
        out['goal_err'] = max(errs) if errs else float('nan')
        out['tool_err'] = math.dist(tool[-1], point)
    op = obs_log(hl_path, meta)
    if op is None:
        return out
    ometa, orows = load(op)
    out['obs'] = os.path.basename(op)
    out['taken'] = ometa.get('taken')
    out['verdicts'] = [(v, ometa.get(v, 0)) for v in VERDICTS if ometa.get(v, 0)]
    out['camera'] = ometa.get('camera_side', {})
    out['workspace'] = ometa.get('workspace', {})
    out['policy'] = ometa.get('policy')
    used = [r for r in orows if r['verdict'] == 'accepted']
    out['used'] = len(used)
    out['age'] = [num(r['t']) - num(r['t_e']) for r in used]
    if not obj:
        return out
    out['obj_err'] = [math.dist(xyz(r, 'obj'), obj) for r in used]
    # The way of the tool between the capture and the joint angles used: a row of the high-level log at time t has
    # the tool at the joint angles that the period of t received, which are state_age periods old.
    period = float(ometa.get('period', 0.001))
    back = int(ometa.get('state_age_periods', 0))

    def tool_at(t_angles):                     # the tool when the arm had the angles of time t_angles
        i = int(round(t_angles / period)) + back
        return tool[min(max(i, 0), len(tool) - 1)] if tool else None

    ratio = []
    for r, e in zip(used, out['obj_err']):
        if ometa.get('policy') != 'max_age' or not tool:
            break
        way = math.dist(tool_at(num(r['t']) - back * period), tool_at(num(r['t_e'])))
        if way > 0.001:
            ratio.append(e / way)
    out['ratio'] = ratio
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*', default=[DEFAULT_DIR])
    ap.add_argument('--object', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='the place of the object [m]')
    ap.add_argument('--cfg', help='read the place of the object from SIM_CAM_OBJECT_M of this cfg')
    ap.add_argument('--check', type=float, metavar='TOL_MM')
    a = ap.parse_args()
    obj = tuple(a.object) if a.object else None
    if obj is None and a.cfg:
        obj = tuple(float(x) for x in (jp.cfg_get(a.cfg, 'SYSTEM', 'SIM_CAM_OBJECT_M') or '').split()[:3])
        if len(obj) != 3:
            sys.exit('app_methods: no SIM_CAM_OBJECT_M in %s' % a.cfg)
    if a.check is not None and obj is None:
        sys.exit('app_methods: --check needs the place of the object (--object or --cfg)')
    paths = []
    for p in a.logs:
        paths += sorted(glob.glob(os.path.join(p, 'hl_app_*.csv'))) if os.path.isdir(p) else [p]
    if not paths:
        sys.exit('app_methods: no hl_app_*.csv in %s' % ', '.join(a.logs))
    runs = [analyze(p, obj) for p in paths]

    mm = lambda v: '%8.3f' % (v * 1e3) if v is not None and v == v else '       -'
    sec = lambda v: '%.3f' % v if v is not None else 'never'
    print('%-6s %-34s %9s %5s %7s %7s %s' % ('method', 'log', 'used', 'goals', 'first', 'rest',
                                             'object max     mean     goal     tool [mm]' if obj else ''))
    failed = 0
    for r in runs:
        used = '%d/%d' % (r['used'], r['taken']) if 'used' in r else '-'
        line = '%-6s %-34s %9s %5s %7s %7s' % (r['method'], r['log'][:34], used, r['goals'], sec(r['first']),
                                              sec(r['rest']))
        if obj:
            oe = r.get('obj_err')
            line += ' %s %s %s %s' % (mm(max(oe)) if oe else mm(None), mm(sum(oe) / len(oe)) if oe else mm(None),
                                      mm(r.get('goal_err')), mm(r.get('tool_err')))
        print(line + ('' if r['result'] == 'completed' else '   (%s)' % r['result']) +
              ('   TIMING FAULTS' if r['faults'] else ''))
        if a.check is not None and r['method'] in ('A2', 'FULL'):
            oe = r.get('obj_err') or []
            ok = bool(oe) and max(oe) * 1e3 <= a.check and r.get('tool_err', 1.0) * 1e3 <= a.check
            failed += 0 if ok else 1
            print('       %s: %d results used, the largest error of an object %.3f mm, the tool ends %.3f mm from the '
                  'hover point (allowed %.3f mm each)' % ('PASS' if ok else 'FAIL', len(oe),
                                                         max(oe) * 1e3 if oe else float('nan'),
                                                         r.get('tool_err', float('nan')) * 1e3, a.check))
    for r in runs:
        if 'obs' not in r:
            continue
        cam = r['camera']
        print('\n%s %s (%s)' % (r['method'], r['obs'], r['policy']))
        some = lambda counts, names: ', '.join('%d %s' % (counts[k], what) for k, what in names if counts.get(k))
        print('  verdicts: %s' % (', '.join('%d %s' % (n, v) for v, n in r['verdicts']) or 'none'))
        print('  camera side: %s written; not written: %s' % (cam.get('written', '?'), some(cam, NOT_WRITTEN) or 'none'))
        if some(r['workspace'], OUTSIDE):
            print('  workspace check of the results used: %s accepted; refused: %s' % (
                r['workspace'].get('accepted', '?'), some(r['workspace'], OUTSIDE)))
        if r['age']:
            print('  age of a result when it was used: mean %.1f ms, largest %.1f ms' % (
                sum(r['age']) / len(r['age']) * 1e3, max(r['age']) * 1e3))
        if r.get('ratio'):
            q = r['ratio']
            print('  error of the object against the way of the tool since the capture (%d results with a way above '
                  '1 mm): median %.2f, from %.2f to %.2f for 8 of 10' % (len(q), quantile(q, 0.5), quantile(q, 0.1),
                                                                         quantile(q, 0.9)))
    if a.check is not None:
        n = sum(1 for r in runs if r['method'] in ('A2', 'FULL'))
        print('\n%s: %d of %d runs with the contract fail the check' % ('PASS' if failed == 0 and n else 'FAIL',
                                                                        failed, n))
        return 0 if failed == 0 and n else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
