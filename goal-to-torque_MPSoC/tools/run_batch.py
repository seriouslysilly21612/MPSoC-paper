#!/usr/bin/env python3
"""run_batch.py - the unattended batch of the measurement protocol (D-11): tools/run_timing.py for every
condition and repetition, with nobody at the board.

  python3 tools/run_batch.py --out DIR [--quick] [--bus FILE] [--arch P0|S1] [--dry-run]

  (full)     the 7 conditions (idle, stress, perception, pressure 25, 50, 75, 100; perception is pressure 0) x 30 min x 3; each
             repetition is one round of the 7, so that a slow drift (the temperature of the board) falls on every
             condition alike; about 11.8 h, about 2.3 GB of timing logs
  --quick    idle, stress, perception, pressure 100 for 10 min each, then pressure 100 twice more; about 1.4 h
  --bus FILE the bus of every run (default indy7/bench/L7N3_bus.cfg, the L7N bench)
  --arch S1  every run with the EtherCAT task on the R5 (run_timing.py --arch S1)
  --dry-run  the runs and their commands; nothing is run
  --from N   start at run N in an OUT of an earlier batch; the directory of a run done again is kept as
             <nn>_<condition>.before-<time>

Before it: the IgH master running (sudo systemctl start ethercat; with --arch S1 instead the R5 running, its bus
in OP, and the UIO device of its record there: rpu_work/README_S1.md), NTP off (sudo timedatectl set-ntp false),
none of the controller, stress-ng and the camera pipeline running, and a terminal that outlives the login (tmux). Every run gets OUT/<nn>_<condition>/
(the files of run_timing.py, its output in run.txt), the high-level test not logged (--no-hl-log). A run that
fails or hangs is recorded and the batch goes on.
  OUT/batch.csv          one line per run: n, repetition, condition, pressure, start, end, exit code (and why:
                         stopped, or the criteria that failed), the memrate rate reached [MB/s], the highest PS
                         temperature [C], what was left running and killed
  OUT/timing_tables.csv  the timing_table.csv of every run (its row of Table 6) after the columns of the run
Standard library only.
"""
import argparse
import csv
import os
import re
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_timing as rt  # noqa: E402

CONDS = [('idle', None), ('stress', None), ('perception', None)] + [('pressure', p) for p in (25, 50, 75, 100)]   # 0 % is perception
QUICK = [('idle', None), ('stress', None), ('perception', None)] + [('pressure', 100)] * 3
# what a run may leave behind; killed before the next run (every run is -b: the drives of the bus stay off)
LEFTOVERS = ('indy7_ctrl', '^stress-ng', 'pick_place_vitis_ai.launch.py') + rt.PERCEPTION
LOG_BYTES = 60 * 1000                                     # the timing log per second (1 kHz, 60 B a period)


def plan(quick):
    """the runs: (n, repetition, condition, pressure, the goal stream [s])"""
    runs, seen = [], {}
    for n, cp in enumerate(QUICK if quick else CONDS * 3, 1):
        seen[cp] = seen.get(cp, 0) + 1
        runs.append((n, seen[cp], cp[0], cp[1], 600.0 if quick else 1800.0))
    return runs


def name(n, cond, pressure):
    return '%02d_%s' % (n, cond if pressure is None else 'pressure%d' % pressure)


def command(run, out, bus, arch):
    n, _, cond, pressure, dur = run
    return ([sys.executable, os.path.join(HERE, 'run_timing.py'), '--condition', cond] +
            (['--pressure', str(pressure)] if pressure is not None else []) +
            ['--duration', '%g' % dur, '--out', os.path.join(out, name(n, cond, pressure)), '--no-hl-log',
             '--bus', bus] + (['--arch', arch] if arch != 'P0' else []))


def estimate(run):
    """[s] of a run: the goal stream, and before it the camera pipeline (about 60 s), the 120 s warm-up of a load,
    the start, settle and HOME of the controller (about 60 s)"""
    _, _, cond, _, dur = run
    return dur + 60 + (120 if cond != 'idle' else 0) + (60 if cond in ('perception', 'pressure') else 0) + 10


def why(d, code):
    """the exit code of a run with its reason: where run_timing.py stopped (a fault of the run), or the criteria of
    timing_summary.py that failed (a result: the run measured to its end)"""
    if code == 0:
        return '0'
    try:
        m = re.search(r'^run_timing: stopped: (.*)$', open(os.path.join(d, 'run.txt')).read(), re.M)
        if m:
            return '%s (stopped: %s)' % (code, m.group(1).strip())
        fails = re.findall(r'^  (\S.*?)\s+FAIL\s+(.*)$', open(os.path.join(d, 'timing_summary.txt')).read(), re.M)
        if fails:
            return '%s (criteria FAIL: %s)' % (code, '; '.join('%s, %s' % (a, b.strip()) for a, b in fails))
    except OSError:
        pass
    return str(code)


def running(pattern):
    return subprocess.run(['pgrep', '-f', pattern], capture_output=True).returncode == 0


def preflight(out, bus, runs, arch):
    errs = []
    if arch == 'S1':
        errs += rt.r5_missing()
    elif not rt.master_running():        # the master itself: it may be started without systemd (ethercatctl)
        errs.append('the IgH master is not running (sudo systemctl start ethercat)')
    if rt.ntp_active():
        errs.append('NTP is active (sudo timedatectl set-ntp false; after the batch: sudo timedatectl set-ntp true)')
    errs += ['%s runs already' % p.lstrip('^') for p in LEFTOVERS if running(p)]
    if not os.path.isfile(bus):
        errs.append('no bus file %s' % bus)
    need = sum(r[4] for r in runs) * LOG_BYTES * 1.3 + 0.2e9
    free = shutil.disk_usage(out).free
    if free < need:
        errs.append('%.1f GB free in %s, %.1f GB needed' % (free / 1e9, out, need / 1e9))
    if errs:
        sys.exit('run_batch: ' + '; '.join(errs))
    if not (os.environ.get('TMUX') or os.environ.get('STY')):
        print('run_batch: WARNING: not inside tmux or screen: the batch ends when the login does')


def tables(out, runs):
    """the timing_table.csv of every run in one file"""
    head, rows = None, []
    for n, rep, cond, pressure, _ in runs:
        try:
            r = list(csv.reader(open(os.path.join(out, name(n, cond, pressure), 'timing_table.csv'))))
        except OSError:
            continue
        if len(r) >= 2:
            head = head or ['run', 'repetition', 'condition', 'pressure'] + r[0]
            rows.append([n, rep, cond, '' if pressure is None else pressure] + r[1])
    if head:
        with open(os.path.join(out, 'timing_tables.csv'), 'w', newline='') as f:
            csv.writer(f).writerows([head] + rows)
    return len(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='the directory of the batch')
    ap.add_argument('--quick', action='store_true', help='the short protocol, about 1.4 h')
    ap.add_argument('--bus', default=os.path.join(rt.APP, 'bench', 'L7N3_bus.cfg'), help='the bus of every run')
    ap.add_argument('--arch', default='P0', choices=('P0', 'S1'), help='where the EtherCAT task runs (S1: on the R5)')
    ap.add_argument('--dry-run', action='store_true', help='the runs and their commands only')
    ap.add_argument('--from', dest='first', type=int, default=1, help='start at run N (an earlier batch in OUT)')
    a = ap.parse_args()
    out, bus = os.path.abspath(os.path.expanduser(a.out)), os.path.abspath(a.bus)
    runs = plan(a.quick)
    todo = [r for r in runs if r[0] >= a.first]
    total = sum(estimate(r) for r in todo)
    print('run_batch: %d runs, about %.1f h, into %s' % (len(todo), total / 3600, out))
    if a.dry_run:
        for r in todo:
            print('  %-16s %s' % (name(r[0], r[2], r[3]), ' '.join(command(r, out, bus, a.arch))))
        return 0
    os.makedirs(out, exist_ok=True)
    preflight(out, bus, todo, a.arch)
    path = os.path.join(out, 'batch.csv')
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['run', 'repetition', 'condition', 'pressure', 'start', 'end', 'exit', 'memrate_MBps',
                        'ps_temp_max_C', 'killed_after'])
        for r in todo:
            n, rep, cond, pressure, dur = r
            d = os.path.join(out, name(n, cond, pressure))
            if os.path.exists(d):
                os.rename(d, d + '.before-' + time.strftime('%H%M%S'))
            os.makedirs(d, exist_ok=True)
            start = time.strftime('%Y-%m-%d %H:%M:%S')
            print('run_batch: %d/%d %s (repetition %d) from %s' % (n, len(runs), name(n, cond, pressure), rep,
                                                                     start), flush=True)
            with open(os.path.join(d, 'run.txt'), 'w') as log:
                p = subprocess.Popen(command(r, out, bus, a.arch), stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
                try:
                    code = p.wait(timeout=estimate(r) + 900)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGINT)
                    try:
                        code = 'timeout %d' % p.wait(timeout=60)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGKILL)
                        code = 'timeout, killed'
            killed = [x.lstrip('^') for x in LEFTOVERS if running(x)]
            for x in LEFTOVERS:
                subprocess.run(['pkill', '-KILL', '-f', x], capture_output=True)
            rate, tmax = rt.run_extras(d)
            code = why(d, code)
            w.writerow([n, rep, cond, '' if pressure is None else pressure, start,
                        time.strftime('%Y-%m-%d %H:%M:%S'), code, '' if rate is None else '%.0f' % rate,
                        '' if tmax is None else '%.1f' % tmax, ' '.join(killed)])
            f.flush()
            print('run_batch: %s exit %s%s' % (name(n, cond, pressure), code,
                                               '; killed after it: ' + ' '.join(killed) if killed else ''), flush=True)
            time.sleep(10)
    print('run_batch: done; %d timing tables in %s' % (tables(out, runs), os.path.join(out, 'timing_tables.csv')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
