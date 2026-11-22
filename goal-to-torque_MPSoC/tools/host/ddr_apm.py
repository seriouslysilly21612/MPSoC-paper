#!/usr/bin/env python3
"""ddr_apm.py - the traffic into the DDR controller through its ports, from the hardened AXI performance monitor
of the PS (APM_DDR at 0xFD0B0000, UG1087; the monitor of the "Analyzing and Modeling Memory Traffic" tutorial of
Vitis). For ddr_bytes of mode_profile.yaml (tools/mode_energy.py), and for what the DPU and the APU put on the DDR.

Ten counters, the read and the write bytes of the ports S1 to S5 (S0, the RPU and the LPD, is left out): S1 and
S2 the APU through the CCI, S3 to S5 the HP ports of the PL (the DPU), as UG1085 assigns them. Every --period
the ten counters are read and their increase (32 bits, wrapping) goes into one row of --out: unix_s, dt_s, then
s1_rd_B, s1_wr_B ... s5_rd_B, s5_wr_B. At the end the mean MB/s of each port, and the monitor's registers back
to what they were.

The metric numbers 2 (write bytes) and 3 (read bytes) are those of the AXI Performance Monitor (PG037), not
checked on this board yet: check them with a known load first (memrate at a set rate: S1-S2 should carry about
that rate). --probe only reads the registers.

One recording at a time (a lock file): a second one would clear the counters of the first, and the first, when it
ends, would put back the registers as they were before it, stopping the second (2026-11-22, I-136).

Usage (on the board, root for /dev/mem):
  sudo python3 tools/host/ddr_apm.py --probe
  sudo python3 tools/host/ddr_apm.py --out OUT/apm.csv [--period 0.25] [--duration S]     (Ctrl-C or SIGTERM ends)
Standard library only.
"""
import argparse
import fcntl
import mmap
import os
import signal
import sys
import time

BASE = 0xFD0B0000            # APM_DDR
PAGE = 4096
CR = 0x300                   # bit 0 MET_CNT_EN, bit 1 MET_CNT_RST
MSR = (0x44, 0x48, 0x4C)     # metric selectors: counters 0-3, 4-7, 8-9 (SEL 4:0, SLOT 7:5 of each byte)
MCR0, MCR_STEP = 0x100, 0x10
WRITE_BYTES, READ_BYTES = 2, 3
PORTS = (1, 2, 3, 4, 5)
COUNTERS = [(p, m) for p in PORTS for m in (READ_BYTES, WRITE_BYTES)]          # counter i: (port, metric)
NAMES = ['s%d_%s_B' % (p, 'rd' if m == READ_BYTES else 'wr') for p, m in COUNTERS]
LOCK = '/run/lock/ddr_apm.lock' if os.path.isdir('/run/lock') else '/tmp/ddr_apm.lock'


def take_lock(path):
    """an open file holding the lock of one recording, or None while another one holds it"""
    f = open(path, 'a')
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    return f


class Regs:
    """32-bit registers of one page of /dev/mem"""

    def __init__(self, base, write):
        fd = os.open('/dev/mem', (os.O_RDWR if write else os.O_RDONLY) | os.O_SYNC)
        prot = mmap.PROT_READ | (mmap.PROT_WRITE if write else 0)
        self.mm = mmap.mmap(fd, PAGE, mmap.MAP_SHARED, prot, offset=base)
        self.w = memoryview(self.mm).cast('I')

    def read(self, off):
        return int(self.w[off // 4])

    def write(self, off, value):
        self.w[off // 4] = value & 0xFFFFFFFF


def selectors(counters):
    """the three MSR words that put counter i on (port, metric) of counters[i]"""
    words = [0, 0, 0]
    for i, (port, metric) in enumerate(counters):
        words[i // 4] |= ((port & 0x7) << 5 | (metric & 0x1F)) << (8 * (i % 4))
    return words


def increase(new, old):
    """the increase of a 32-bit counter that may have wrapped once"""
    return (new - old) & 0xFFFFFFFF


def counters(regs):
    return [regs.read(MCR0 + MCR_STEP * i) for i in range(len(COUNTERS))]


def probe(regs):
    lines = ['APM_DDR 0x%08X: CR 0x%08X, MSR 0x%08X 0x%08X 0x%08X' % (
        BASE, regs.read(CR), *[regs.read(m) for m in MSR])]
    a = counters(regs)
    time.sleep(1.0)
    b = counters(regs)
    lines.append('the counters over 1 s (as they are set, enabled or not): ' + ', '.join(
        '%d' % increase(y, x) for x, y in zip(a, b)))
    return lines


def record(regs, out, period, duration, stop):
    """set the counters, sample them into out until duration or stop; the registers back as they were"""
    saved = [regs.read(CR)] + [regs.read(m) for m in MSR]
    try:
        for m, word in zip(MSR, selectors(COUNTERS)):
            regs.write(m, word)
        regs.write(CR, 0x2)                       # clear the counters
        regs.write(CR, 0x1)                       # count
        total = [0] * len(COUNTERS)
        seconds = 0.0
        t_end = time.monotonic() + duration if duration else None
        with open(out, 'w') as f:
            f.write('unix_s,dt_s,' + ','.join(NAMES) + '\n')
            last, t_last = counters(regs), time.monotonic()
            while not stop and (t_end is None or time.monotonic() < t_end):
                time.sleep(period)
                now, t_now = counters(regs), time.monotonic()
                d = [increase(y, x) for x, y in zip(last, now)]
                dt = t_now - t_last
                f.write('%.3f,%.4f,%s\n' % (time.time(), dt, ','.join(str(x) for x in d)))
                f.flush()
                total = [a + b for a, b in zip(total, d)]
                seconds += dt
                last, t_last = now, t_now
        return total, seconds
    finally:
        regs.write(CR, saved[0])
        for m, word in zip(MSR, saved[1:]):
            regs.write(m, word)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--probe', action='store_true', help='read the registers only, nothing written')
    ap.add_argument('--out')
    ap.add_argument('--period', type=float, default=0.25,
                    help='[s], 0.05 .. 1 (a 32-bit counter wraps in 2 s at 2 GB/s)')
    ap.add_argument('--duration', type=float, help='[s]; default: until Ctrl-C')
    a = ap.parse_args()
    if not a.probe and not a.out:
        sys.exit('ddr_apm: --probe or --out')
    if not 0.05 <= a.period <= 1.0 or (a.duration is not None and not a.duration > 0):
        sys.exit('ddr_apm: --period 0.05 .. 1 s, --duration above 0')
    lock = None if a.probe else take_lock(LOCK)
    if not a.probe and lock is None:
        sys.exit('ddr_apm: another recording runs (%s): wait for its end' % LOCK)
    try:
        regs = Regs(BASE, write=not a.probe)
    except OSError as ex:
        sys.exit('ddr_apm: /dev/mem at 0x%08X: %s (root?)' % (BASE, ex))
    if a.probe:
        print('\n'.join(probe(regs)))
        return 0
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    print('ddr_apm: read and write bytes of the DDR ports S1-S5 every %.2f s -> %s' % (a.period, a.out), flush=True)
    total, seconds = record(regs, a.out, a.period, a.duration, stop)
    if seconds > 0:
        print('ddr_apm: %.1f s, MB/s ' % seconds + ', '.join(
            '%s %.1f' % (n[:-2], t / seconds / 1e6) for n, t in zip(NAMES, total)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
