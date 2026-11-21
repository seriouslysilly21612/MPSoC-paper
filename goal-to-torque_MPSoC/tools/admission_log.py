#!/usr/bin/env python3
"""admission_log.py - what the admission rule of the camera side decided in a run, and what became of it.

Reads the decision log of the detector (methods A3 and FULL; launch argument decision_log:=FILE of the camera
pipeline): one row per image, with the decision and every candidate that was looked at. Prints
  decisions   images run, skipped and rejected, the rejections by their reason
  checks      of the rejected images with candidates: how many failed the time check, the error check and the
              response check
  prediction  the age at which the rule expected the result of a run image to be used first
With the log of the camera results of the control application of the same run (obs_<method>_*.csv with its
.json), the images that were run are joined to the results by their capture time, and it prints
  arrival     how many of the run images reached the control task, and their verdicts there
  realized    the period that took the result against the first use that the rule predicted: the rule holds its
              promise when the result is there no later than predicted

Usage:
  python3 tools/admission_log.py DECISIONS.csv [OBS.csv]
Standard library only.
"""
import argparse
import bisect
import csv
import json
import os
import sys

JOIN_TOLERANCE_S = 0.001        # two images are 33 ms apart; the two logs convert the same stamp separately
CHECKS = [('in_time', 'time'), ('within_tolerance', 'error'), ('response_ok', 'response')]


def read_rows(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def quantile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def run_candidate(row):
    """the columns of the mode that a run decision chose: (first use after the release [s], age at first use [s])"""
    prefix = 'm%s_' % row['mode']
    return float(row[prefix + 'first_use_ms']) * 1e-3, float(row[prefix + 'age_ms']) * 1e-3


def failed_checks(row):
    """the checks that no mode candidate of a rejected image passed: {'time': bool, 'error': bool, 'response': bool}"""
    failed = {}
    for column, name in CHECKS:
        values = [row['m%d_%s' % (mode, column)] for mode in (0, 1) if row.get('m%d_%s' % (mode, column), '') != '']
        failed[name] = bool(values) and all(v == '0' for v in values)
    return failed


def results_by_capture(obs_path):
    """the results of the control application's log: [(capture on the clock of the tasks [s], row)], sorted"""
    with open(os.path.splitext(obs_path)[0] + '.json') as f:
        meta = json.load(f)
    # with S2 the ticks are the R5's counter, not the clock of the decision log (I-119): its start on that clock
    if 't_start_monotonic_ns' in meta:
        start_s = meta['t_start_monotonic_ns'] * 1e-9
    else:
        start_s = meta['t_start_ticks'] / meta['ticks_per_s']
    return start_s, sorted((start_s + float(row['t_e']), row) for row in read_rows(obs_path))


def join(decisions, results):
    """for every run decision the result of the same image, or None: [(decision row, result row or None)]"""
    captures = [capture for capture, _ in results]
    joined = []
    for row in decisions:
        if row['action'] != 'run':
            continue
        capture = float(row['capture_task_s'])
        after = bisect.bisect_left(captures, capture)          # the nearest capture is this one or the one before
        nearest = min((i for i in (after - 1, after) if 0 <= i < len(captures)),
                      key=lambda i: abs(captures[i] - capture), default=None)
        near = nearest is not None and abs(captures[nearest] - capture) <= JOIN_TOLERANCE_S
        joined.append((row, results[nearest][1] if near else None))
    return joined


def report(decisions, obs_path=None, out=sys.stdout):
    def line(text=''):
        print(text, file=out)

    actions = [row['action'] for row in decisions]
    line('%d decisions: %d run, %d skipped, %d rejected' % (len(actions), actions.count('run'), actions.count('skip'),
                                                            actions.count('reject')))
    rejected = [row for row in decisions if row['action'] == 'reject']
    reasons = sorted(set(row['reason'] for row in rejected))
    if reasons:
        line('  rejected: ' + ', '.join('%d %s' % (sum(1 for row in rejected if row['reason'] == reason), reason)
                                        for reason in reasons))
    with_candidates = [row for row in rejected if row['reason'] == 'no_candidate_passes']
    if with_candidates:
        failed = [failed_checks(row) for row in with_candidates]
        line('  of the %d with candidates, no mode passed: %s' % (
            len(with_candidates), ', '.join('the %s check in %d' % (name, sum(1 for f in failed if f[name]))
                                            for _, name in CHECKS)))
    ran = [row for row in decisions if row['action'] == 'run']
    if ran:
        ages = [run_candidate(row)[1] for row in ran]
        line('  predicted age of a run image at its first use: median %.1f ms, largest %.1f ms' % (
            quantile(ages, 0.5) * 1e3, max(ages) * 1e3))
    if obs_path is None or not ran:
        return
    start_s, results = results_by_capture(obs_path)
    joined = join(decisions, results)
    arrived = [(row, result) for row, result in joined if result is not None]
    line('\n%s' % os.path.basename(obs_path))
    line('  %d of the %d run images reached the control task' % (len(arrived), len(joined)))
    verdicts = sorted(set(result['verdict'] for _, result in arrived))
    if verdicts:
        line('  their verdicts: ' + ', '.join('%d %s' % (sum(1 for _, r in arrived if r['verdict'] == v), v)
                                              for v in verdicts))
    late = []                   # realized first use minus the predicted one [s]
    for row, result in arrived:
        release_task_s = float(row['release_s']) + (float(row['capture_task_s']) - float(row['capture_s']))
        predicted_s = release_task_s + run_candidate(row)[0]
        late.append(start_s + float(result['t_in']) - predicted_s)
    if late:
        line('  the period that took the result against the predicted first use: median %+.1f ms, 9 of 10 below '
             '%+.1f ms, largest %+.1f ms; later than predicted: %d of %d' % (
                 quantile(late, 0.5) * 1e3, quantile(late, 0.9) * 1e3, max(late) * 1e3,
                 sum(1 for x in late if x > 0), len(late)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('decisions', help='decision log of the detector (CSV)')
    ap.add_argument('obs', nargs='?', help='obs_<method>_*.csv of the control application, same run')
    a = ap.parse_args()
    decisions = read_rows(a.decisions)
    if not decisions:
        sys.exit('admission_log: %s has no decisions' % a.decisions)
    report(decisions, a.obs)
    return 0


if __name__ == '__main__':
    sys.exit(main())
