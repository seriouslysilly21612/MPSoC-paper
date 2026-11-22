#!/usr/bin/env python3
"""power_log.py - the power of the KV260 SOM (D-26: the sensor on the board), sampled into a CSV for
tools/trials.py --power.

Reads the INA260 of the SOM's 5 V rail through hwmon (name ina260_u14, as for the measurements of 2026-11-21):
power1_input [uW], or in1_input [mV] x curr1_input [mA] where the driver has no power1_input. One row per sample:
unix_s (time.time(), the clock of the names of the logs), power_w. Until --duration or Ctrl-C (SIGTERM too);
prints the mean at the end.

Usage (on the board, no root needed for hwmon):
  python3 tools/host/power_log.py --out ~/power_trials.csv [--period 1.0] [--duration S] [--name ina260_u14]
  --hwmon-root DIR: another root than /sys/class/hwmon (the tests). Standard library only.
"""
import argparse
import glob
import os
import signal
import sys
import time


def find_sensor(root, name):
    """the hwmon directory whose name is `name`, else the only one whose name starts with ina260; None"""
    found = []
    for d in sorted(glob.glob(os.path.join(root, 'hwmon*'))):
        try:
            with open(os.path.join(d, 'name')) as f:
                found.append((f.read().strip(), d))
        except OSError:
            continue
    for n, d in found:
        if n == name:
            return d
    ina = [d for n, d in found if n.startswith('ina260')]
    return ina[0] if len(ina) == 1 else None


def read_int(path):
    with open(path) as f:
        return int(f.read().strip())


def read_power_w(d):
    """the power now [W]"""
    p = os.path.join(d, 'power1_input')
    if os.path.exists(p):
        return read_int(p) * 1e-6
    return read_int(os.path.join(d, 'in1_input')) * read_int(os.path.join(d, 'curr1_input')) * 1e-6


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--period', type=float, default=1.0, help='[s], 0.05 .. 60')
    ap.add_argument('--duration', type=float, help='[s]; default: until Ctrl-C')
    ap.add_argument('--name', default='ina260_u14')
    ap.add_argument('--hwmon-root', default='/sys/class/hwmon')
    a = ap.parse_args()
    if not 0.05 <= a.period <= 60.0 or (a.duration is not None and not a.duration > 0):
        sys.exit('power_log: --period 0.05 .. 60 s, --duration above 0')
    d = find_sensor(a.hwmon_root, a.name)
    if d is None:
        sys.exit('power_log: no hwmon named %s (nor one ina260*) under %s' % (a.name, a.hwmon_root))
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    n, total = 0, 0.0
    t_end = time.monotonic() + a.duration if a.duration else None
    t_next = time.monotonic()
    print('power_log: %s (%s) every %.2f s -> %s' % (d, os.path.basename(d), a.period, a.out), flush=True)
    with open(a.out, 'w') as f:
        f.write('unix_s,power_w\n')
        while not stop and (t_end is None or time.monotonic() < t_end):
            try:
                w = read_power_w(d)
            except (OSError, ValueError) as ex:
                print('power_log: a sample failed (%s); goes on' % ex, file=sys.stderr)
            else:
                f.write('%.3f,%.4f\n' % (time.time(), w))
                f.flush()
                n, total = n + 1, total + w
            t_next += a.period
            time.sleep(max(0.0, t_next - time.monotonic()))
    print('power_log: %d samples, mean %.3f W' % (n, total / n if n else float('nan')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
