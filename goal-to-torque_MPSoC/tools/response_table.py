#!/usr/bin/env python3
"""response_table.py - the table of check 3 of the admission rule (eq. (8), D-24) from measured runs: the longest
response of the control task (S2: t_w + t_e of the R5 with the control computation) in each run of a mode and load.

  python3 -B tools/response_table.py --out response.csv MODE:LEVEL:RUN_DIR [...]

MODE is the mode number of the mailbox (0 = 416, 1 = 320) that the run's detector ran alone (M1 with that model);
LEVEL names the load (e.g. pressure800); RUN_DIR has the timing_summary.txt of tools/run_timing.py. The detector's
launch takes the file as response_table:= (admission.load_response_table: the worst row of a mode counts); its
mode_profile.yaml has the margin. Standard library only.
"""
import argparse
import csv
import os
import re
import sys

LARGEST = re.compile(r'^largest t_w \+ t_e of the R5 with the control computation: ([0-9.]+) us', re.M)


def largest_response_us(run_dir):
    """the largest t_w + t_e with the control computation of a run of --arch S2 [us]"""
    path = os.path.join(run_dir, 'timing_summary.txt')
    with open(path) as f:
        m = LARGEST.search(f.read())
    if m is None:
        raise ValueError('%s: no "largest t_w + t_e of the R5 with the control computation" (not --arch S2?)' % path)
    return float(m.group(1))


def parse_item(item):
    """MODE:LEVEL:RUN_DIR -> (mode, level, run_dir)"""
    parts = item.split(':', 2)
    if len(parts) != 3 or not parts[0].isdigit() or not re.fullmatch(r'[A-Za-z0-9_.-]+', parts[1]):
        raise ValueError('%r: MODE:LEVEL:RUN_DIR (mode a number, level a word)' % item)
    return int(parts[0]), parts[1], parts[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='the CSV: mode,level,response_us,run')
    ap.add_argument('runs', nargs='+', metavar='MODE:LEVEL:RUN_DIR')
    a = ap.parse_args()
    try:
        rows = [(mode, level, largest_response_us(d), d) for mode, level, d in map(parse_item, a.runs)]
    except (OSError, ValueError) as e:
        sys.exit('response_table: %s' % e)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['mode', 'level', 'response_us', 'run'])
        for mode, level, us, d in rows:
            w.writerow([mode, level, '%.1f' % us, os.path.abspath(d)])
    for mode, level, us, d in rows:
        print('mode %d, %s: %.1f us (%s)' % (mode, level, us, d))
    return 0


if __name__ == '__main__':
    sys.exit(main())
