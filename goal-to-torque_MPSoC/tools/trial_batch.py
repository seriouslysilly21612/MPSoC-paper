#!/usr/bin/env python3
"""trial_batch.py - the trials of the main experiment in the simulation (-s, D-25): every method runs the same
places in the same order, then tools/trials.py judges them.

A round visits every place once, in an order drawn from --seed and the round; the rounds follow each other, and
every method gets the same sequence (the methods differ in the rule only, not in the targets). The camera of the
simulation (SIM_CAM=1) puts the object at the next place before every run of the application (sim/SimCamera.h),
so one run is one trial of APP_DURATION_S = --limit-s. Per method: tools/hl_run_sim.py --tests hl-app with
--runs-all = places x rounds, its logs in OUT/<method>/. With --power the board's power is sampled for the whole
batch (tools/host/power_log.py, OUT/power.csv). At the end: tools/trials.py over all of it -> OUT/trials.csv
(one row per trial) and OUT/summary.txt (the table).

The cfg of the batch is a copy of --cfg next to it (indy7/trial_batch.cfg, the relative paths of the cfg hold),
written again for every method; its last state is kept as OUT/trial_batch.cfg. A method folder of OUT that holds
logs already is refused (trials.py would judge the old ones with the new): give a new --out for a batch again.
"place k" below is the k-th place of --places; the application's "place k of n" is the k-th run of the sequence.

Usage (in the repository; the PC or the board):
  python3 tools/trial_batch.py --out DIR [--methods M2,A2,A3,FULL] [--rounds 2] [--places "X Y Z; X Y Z; ..."]
                               [--limit-s 15] [--hold-s 0.5] [--seed 1] [--arch S2] [--direct] [--cpu N]
                               [--sim-noise-mm MM] [--sim-latency-ms MS] [--power] [--cfg CFG]
  --cpu N: the two tasks on CPU N (a PC with fewer than 4 CPUs; the cfg has CPU=3). Standard library only.
"""
import argparse
import glob
import os
import random
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402

REPO = os.path.normpath(os.path.join(HERE, '..'))
CFG_MAIN = os.path.join(REPO, 'indy7', 'INDY7.cfg')
METHODS = ('M1', 'M2', 'A2', 'A3', 'FULL')
# first places in the box of the application (APP_GOAL_BOX_M) on the table (z 0.10): goal = place + (-0.05, 0,
# 0.20) within the box and the radius. The robot's places come from probing them (D-25)
PLACES = [(0.55, 0.15, 0.10), (0.50, -0.10, 0.10), (0.65, 0.30, 0.10), (0.70, 0.00, 0.10), (0.45, 0.30, 0.10)]
MAX_RUNS = 32                                  # the places of the camera of the simulation (CSimCamera::PLACES)


def parse_places(text):
    """'x y z; x y z' -> [(x, y, z), ...]"""
    out = []
    for part in text.split(';'):
        if part.strip():
            v = [float(x) for x in part.split()]
            if len(v) != 3:
                raise ValueError('%r: three numbers a place' % part.strip())
            out.append(tuple(v))
    if not out:
        raise ValueError('no place')
    return out


def sequence(places, rounds, seed):
    """the places of every run: each round a permutation of its own (seed, round), the same for every method"""
    seq = []
    for r in range(rounds):
        order = list(range(len(places)))
        random.Random(seed * 1000 + r).shuffle(order)
        seq += [places[i] for i in order]
    return seq


def cfg_items(seq, limit_s, log_dir, cpu=None, noise_mm=None, latency_ms=None):
    items = [('SIM_CAM', '1'), ('APP_DURATION_S', '%g' % limit_s),
             ('SIM_CAM_OBJECT_M', '  '.join('%.4f %.4f %.4f' % p for p in seq)), ('JOINT_TEST_LOG_DIR', log_dir)]
    if cpu is not None:
        items += [('TASK0.CPU', str(cpu)), ('TASK1.CPU', str(cpu))]
    if noise_mm is not None:
        items.append(('SIM_CAM_NOISE_MM', '%g' % noise_mm))
    if latency_ms is not None:
        items.append(('SIM_CAM_LATENCY_MS', '%g' % latency_ms))
    return items


def write_cfg(src, dst, items):
    """dst = src with items set; raises on a key that src does not have"""
    lines, _, errors = cfgset.plan(src, items)
    if errors:
        raise ValueError('; '.join(errors))
    with open(dst, 'w', encoding='utf-8', errors='surrogateescape', newline='') as f:
        f.writelines(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--methods', default='M2,A2,A3,FULL')
    ap.add_argument('--rounds', type=int, default=2)
    ap.add_argument('--places', help='"X Y Z; X Y Z; ..." [m, base frame]; default: five first places')
    ap.add_argument('--limit-s', type=float, default=15.0, help='the limit of a trial: APP_DURATION_S')
    ap.add_argument('--hold-s', type=float, default=0.5)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--arch')
    ap.add_argument('--direct', action='store_true', help='bin/indy7_ctrl without run.sh (no ROS 2 needed)')
    ap.add_argument('--cpu', type=int)
    ap.add_argument('--sim-noise-mm', type=float)
    ap.add_argument('--sim-latency-ms', type=float)
    ap.add_argument('--power', action='store_true', help='sample the INA260 of the board into OUT/power.csv')
    ap.add_argument('--cfg', default=CFG_MAIN)
    a = ap.parse_args()
    methods = [m.strip() for m in a.methods.split(',') if m.strip()]
    bad = [m for m in methods if m not in METHODS]
    if bad or not methods:
        sys.exit('trial_batch: methods from %s (got %s)' % (', '.join(METHODS), a.methods))
    try:
        places = parse_places(a.places) if a.places else PLACES
    except ValueError as ex:
        sys.exit('trial_batch: --places: %s' % ex)
    if a.rounds < 1 or len(places) * a.rounds > MAX_RUNS:
        sys.exit('trial_batch: places x rounds from 1 to %d (%d x %d)' % (MAX_RUNS, len(places), a.rounds))
    if not a.limit_s > 0 or not a.hold_s >= 0 or a.hold_s >= a.limit_s:
        sys.exit('trial_batch: --limit-s above 0, --hold-s from 0 to below the limit')
    seq = sequence(places, a.rounds, a.seed)
    out = os.path.abspath(a.out)
    used = [m for m in methods if glob.glob(os.path.join(out, m, '**', 'hl_app_*.csv'), recursive=True)]
    if used:
        sys.exit('trial_batch: %s already hold logs in %s: give a new --out' % (', '.join(used), out))
    os.makedirs(out, exist_ok=True)
    src = os.path.abspath(a.cfg)
    batch_cfg = os.path.join(os.path.dirname(src), 'trial_batch.cfg')
    print('trial_batch: %d methods x %d trials (%d places x %d rounds, seed %d), limit %.1f s -> %s' % (
        len(methods), len(seq), len(places), a.rounds, a.seed, a.limit_s, out))
    for i, p in enumerate(seq):
        print('  trial %2d: place %d (%.3f, %.3f, %.3f)' % (i + 1, places.index(p) + 1, *p))
    power = None
    if a.power:
        power = subprocess.Popen([sys.executable, '-B', os.path.join(HERE, 'host', 'power_log.py'),
                                  '--out', os.path.join(out, 'power.csv')])
        time.sleep(1.0)                    # no sensor: it ends at once, before any trial
        if power.poll() is not None:
            sys.exit('trial_batch: power_log ended (%d) before the trials: see its message' % power.returncode)
    failed = []
    try:
        for m in methods:
            log_dir = os.path.join(out, m)
            os.makedirs(log_dir, exist_ok=True)
            try:
                write_cfg(src, batch_cfg, cfg_items(seq, a.limit_s, log_dir, a.cpu, a.sim_noise_mm,
                                                    a.sim_latency_ms))
            except ValueError as ex:
                sys.exit('trial_batch: the cfg: %s' % ex)
            shutil.copy(batch_cfg, os.path.join(out, 'trial_batch.cfg'))
            cmd = [sys.executable, '-B', os.path.join(HERE, 'hl_run_sim.py'), '--cfg', batch_cfg, '--tests',
                   'hl-app', '--runs-all', str(len(seq)), '--method', m]
            cmd += ['--arch', a.arch] if a.arch else []
            cmd += ['--direct'] if a.direct else []
            print('\ntrial_batch: %s' % ' '.join(cmd), flush=True)
            if subprocess.call(cmd) != 0:
                failed.append(m)
                print('trial_batch: %s did not end well (its console log is in %s)' % (m, log_dir))
    finally:
        if power is not None:
            power.send_signal(signal.SIGTERM)
            power.wait(10)
    cmd = [sys.executable, '-B', os.path.join(HERE, 'trials.py')] + [os.path.join(out, m) for m in methods]
    cmd += ['--cfg', batch_cfg, '--hold-s', str(a.hold_s), '--limit-s', str(a.limit_s), '--planned', str(len(seq)),
            '--out', os.path.join(out, 'trials.csv')]
    if power is not None:
        cmd += ['--power', os.path.join(out, 'power.csv')]
    print('\ntrial_batch: %s' % ' '.join(cmd), flush=True)
    res = subprocess.run(cmd, capture_output=True, text=True)
    sys.stdout.write(res.stdout)
    sys.stderr.write(res.stderr)
    with open(os.path.join(out, 'summary.txt'), 'w') as f:
        f.write(res.stdout)
    if failed:
        print('trial_batch: FAILED runs of %s' % ', '.join(failed))
    return 1 if failed or res.returncode else 0


if __name__ == '__main__':
    sys.exit(main())
