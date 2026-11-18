#!/usr/bin/env python3
"""timing_summary.py - the host timing of a run with -t DIR (paper, Sec. III-C, Table 6, Fig. 3).

  python3 tools/timing_summary.py DIR/timing_<stamp>.bin [--window track|op|all] [--skip 100]
                                  [--tb-us T] [--hourly] [--out DIR]

Reads the timing log of the controller (indy7/src/TimingLog.h, format g2t-timing-1): a text header of 1024
bytes, then one record of 60 bytes per period. In the window (default: from the first to the last period
in TRACK, the high-level test that ran during the measurement, without its first --skip periods) it gives,
in us, the mean, the 99.9th percentile and the maximum (percentiles from 0.1 us bins, the upper edge of the
bin; the maximum exact) of:
  send   Delta T_send = (t_w + t_s)_k - (t_w + t_s)_(k-1), the deviation of the interval between two sends
         from T_s (signed, as in Table 6), and its magnitude |Delta T_send|
  EtherCAT task  t_w (planned release -> start), t_s (start -> return of the send) and its phases, t_w + t_s
         (eq. (3): t_w + t_s <= t_b), t_e, t_w + t_e
  Trajectory and Dynamics task  t_w, t_e, t_w + t_e; the record of period k carries their cycle of period
         k - 1, the last one complete when the EtherCAT task wrote it
  chain  R = START_DELAY + t_w + t_e of the Dynamics task, from the release to the torque of the period; with -b (the
         bus of the robot beside the simulated arm, mode=hybrid in the header) the EtherCAT task steps the
         simulated arm after the send, on the same core, before the two other tasks start: their t_w and
         the chain are given without that step (R_adj = R_raw - C_sim, C_sim the step of the period of
         their cycle; for a Dynamics task with a START_DELAY only the part of C_sim that the work before
         it ran past its release)
and the faults in the window: late releases (t_w > T_s), overruns (t_w + t_e > T_s), missed send deadlines
(t_w + t_s > t_b), incomplete working counters, periods without OP, periods in OP without the torque of the
period before (its Dynamics cycle not complete when the EtherCAT task read the Torque slot: the record has no
chain for it, so the chain criterion cannot see it); with -b also that no drive of the bus was ever enabled. It writes timing_summary.txt, timing_table.csv (the row of Table 6) and timing_tw.hist
(t_w of the EtherCAT task in 1 us bins, the format of the cyclictest histograms) into --out (default: the
directory of the log). Standard library only.

A log of --arch S1 (arch=S1 in the header): the EtherCAT exchange runs on the R5, and the records are those of
the task that takes its State slot here, one per cycle, with the period number of the R5. That task does not
send, so the send rows and the send deadline are those of the R5, not of this log. Instead:
  cycles without a new State     the cycle found no State of a new period (the R5 late, or the task early)
  States that no cycle took      periods of the R5 between two States taken: the arm moved for a period that
                                 the reference did not
  chain  R = WAKE_DELAY + START_DELAY + t_w + t_e of the Dynamics task: from the release of the R5, which the
         tasks follow WAKE_DELAY behind, to the torque in its slot; R < T_s means before the next release of
         the R5. The phase error of the following is not in the records: the console summary of the run has
         it ([R5] following), and the torques that the R5 did not have in time, exactly ([R5] Torque slot).
"""
import argparse
import csv
import os
import struct
import sys

HEADER_BYTES = 1024
REC = struct.Struct('<IiiiiiiIHHiiiiii')
FORMAT = 'g2t-timing-1'
FLAGS = dict(OP=1, WKC=2, LATE=4, MISSED=8, OVERRUN=16, LAW=32, HL=64, TRACK=128, STOP=256, SIM=512,
             BUS=1024, SERVO=2048, BUSSERVO=4096, TEST=8192, R5=16384, NOSTATE=32768)
BIN_NS = 100            # bins of the percentiles
MAX_NS = 10_000_000     # the last bin: 10 ms (beyond: overflow, the maximum stays exact)


class Dist:
    """a distribution [ns]: 0.1 us bins up to 10 ms, the exact maximum, minimum and mean; off [ns] shifts the
    bins for a signed quantity (the bins then hold -off .. 10 ms - off)"""

    def __init__(self, off=0):
        self.off = off
        self.bins = [0] * (MAX_NS // BIN_NS + 1)
        self.n = 0
        self.sum = 0
        self.sumsq = 0.0
        self.max = None
        self.min = None
        self.over = 0

    def add(self, v):
        self.n += 1
        self.sum += v
        self.sumsq += float(v) * float(v)
        self.max = v if self.max is None or v > self.max else self.max
        self.min = v if self.min is None or v < self.min else self.min
        b = (v + self.off) // BIN_NS
        if b >= len(self.bins):
            self.over += 1
            b = len(self.bins) - 1
        if b < 0:
            self.over += 1
            b = 0
        self.bins[b] += 1

    def pct(self, p):
        """the upper edge of the bin of the p-quantile [ns] (the maximum, if it is in the last bin)"""
        if self.n == 0:
            return None
        target = p * self.n
        cum = 0
        for i, c in enumerate(self.bins):
            cum += c
            if cum >= target - 1e-9:
                if i == len(self.bins) - 1:
                    return self.max
                return min((i + 1) * BIN_NS - self.off, self.max)
        return self.max

    def mean(self):
        return self.sum / self.n if self.n else None

    def std(self):
        if self.n == 0:
            return None
        m = self.sum / self.n
        return max(self.sumsq / self.n - m * m, 0.0) ** 0.5


def read_log(path):
    with open(path, 'rb') as fp:
        raw = fp.read(HEADER_BYTES)
        data = fp.read()
    if len(raw) < HEADER_BYTES:
        raise SystemExit('not a timing log: the header is shorter than %d bytes' % HEADER_BYTES)
    head = {}
    for line in raw.decode('utf-8', 'replace').splitlines():
        if '=' in line:
            k, _, v = line.strip().partition('=')
            head[k.strip()] = v.strip()
    if head.get('format') != FORMAT:
        raise SystemExit('not a timing log of this controller (format=%s, expected %s)' % (head.get('format'), FORMAT))
    if int(head.get('record_bytes', '0')) != REC.size:
        raise SystemExit('record size %s, %s has %d' % (head.get('record_bytes'), FORMAT, REC.size))
    n = len(data) // REC.size
    if len(data) % REC.size:
        print('note: %d trailing bytes ignored (the controller ended while writing)' % (len(data) % REC.size))
    return head, n, data[:n * REC.size]


def us(v, fmt='%.1f'):
    return '-' if v is None else fmt % (v / 1000.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--window', choices=('track', 'op', 'all'), default='track',
                    help='track: the periods in TRACK (default); op: every period in OP before the stop; all')
    ap.add_argument('--skip', type=int, default=100, help='first periods of the window left out (default 100)')
    ap.add_argument('--tb-us', type=float, default=None,
                    help='send deadline t_b [us] (default: deadline_ns of the log, SEND_DEADLINE of the cfg)')
    ap.add_argument('--hourly', action='store_true', help='the maxima and the faults per hour of the window')
    ap.add_argument('--out', default=None, help='directory of the outputs (default: that of the log)')
    a = ap.parse_args(argv)

    head, n, data = read_log(a.log)
    if n == 0:
        raise SystemExit('no records')
    period = int(head.get('period_ns', '1000000'))
    tb_ns = a.tb_us * 1000.0 if a.tb_us is not None else float(head.get('deadline_ns', period) or period)
    hybrid = head.get('mode') == 'hybrid'
    # --arch S1: the records of the task that takes the State slot of the R5, released wake_delay after it (a
    # line that only a log of S1 has)
    s1 = head.get('arch') == 'S1'
    wake_delay = int(head.get('wake_delay_ns', '0') or 0)
    # START_DELAY of the Dynamics task (the third periodic task; logs before the KV260 port have none): its
    # t_w counts from its own release, the chain from the release of the period
    delays = [int(x) for x in head.get('start_delay_ns', '').split(',') if x.strip()]
    dyn_delay = delays[2] if len(delays) > 2 else 0

    # pass 1: the window [first, last] in release periods
    first = last = before = None
    for r in REC.iter_unpack(data):
        per, fl = r[0], r[8]
        if s1 and before is not None and per < before:
            raise SystemExit('the periods of the R5 go back (%d after %d): the R5 started again during the run; a '
                             'summary is of one run of it' % (per, before))
        before = per
        if a.window == 'track':
            ok = bool(fl & FLAGS['TRACK'])
        elif a.window == 'op':
            ok = bool(fl & FLAGS['OP']) and not (fl & FLAGS['STOP'])
        else:
            ok = not (fl & FLAGS['STOP'])
        if ok:
            first = per if first is None else first
            last = per
    if first is None:
        raise SystemExit('no period in the window "%s" (try --window op)' % a.window)

    names = ['Delta T_send (signed)', '|Delta T_send|', 'EtherCAT t_w', 'EtherCAT t_s', '  receive', '  process',
             '  send', 'EtherCAT t_w + t_s', 'EtherCAT t_e', 'EtherCAT t_w + t_e', 'simulator step',
             'dynamics (RBDL calls)', 'high-level controller', 'Trajectory t_w', 'Trajectory t_e',
             'Trajectory t_w + t_e', 'Dynamics t_w', 'Dynamics t_e', 'chain R' + ('_adj' if hybrid else '')]
    d = {k: Dist() for k in names}
    d['Delta T_send (signed)'] = Dist(off=MAX_NS // 2)
    CHAIN = names[-1]
    cnt = dict(periods=0, late=0, overrun=0, missed=0, wkc=0, notop=0, busservo=0, chain_n=0,
               over_chain=0, nosim=0, notorque=0, nostate=0, states_missed=0)
    worst_ws = worst_chain = None
    prev = None        # (period, t_w + t_s) of the last period of the window
    prev_state = None  # S1: the period of the R5 of the last State taken in the window
    prev_sim = None    # (period, simulator step, EtherCAT t_w + t_e) of the last record
    seen = present = 0
    tw_hist = {}
    per_hour = 3600 * 10 ** 9 // period
    hours = {}
    for (per, w, s, e, dyn, hl, sim, sync, fl, _res, rcv, snd, tjw, tje, dyw, dye) in REC.iter_unpack(data):
        sim_before = prev_sim[1] if (prev_sim is not None and prev_sim[0] == per - 1) else None
        ecat_end_before = prev_sim[2] if (prev_sim is not None and prev_sim[0] == per - 1) else None
        prev_sim = (per, sim, w + e)
        if per < first or per > last:
            continue
        present += 1
        seen += 1
        nostate = s1 and bool(fl & FLAGS['NOSTATE'])
        states_missed = per - prev_state - 1 if (s1 and not nostate and prev_state is not None) else 0
        if s1 and not nostate:
            prev_state = per
        if seen <= a.skip:
            prev = (per, w + s)
            continue
        cnt['nostate'] += nostate
        cnt['states_missed'] += states_missed
        if not s1 and prev is not None and per == prev[0] + 1:
            dv = (w + s) - prev[1]
            d['Delta T_send (signed)'].add(dv)
            d['|Delta T_send|'].add(abs(dv))
        prev = (per, w + s)
        cnt['periods'] += 1
        hh = None
        if a.hourly:
            hh = hours.setdefault((per - first) // per_hour, [0, 0, 0, 0, 0, 0, 0, 0])
            hh[0] += 1
        for key, flag, idx in (('late', 'LATE', 4), ('overrun', 'OVERRUN', 5), ('missed', 'MISSED', 6),
                               ('wkc', 'WKC', 7)):
            if fl & FLAGS[flag]:
                cnt[key] += 1
                if hh is not None:
                    hh[idx] += 1
        if nostate:
            pass                    # counted above: a cycle without a State has no OP and no torque to tell
        elif not (fl & FLAGS['OP']):
            cnt['notop'] += 1
        elif dyw < 0 or dye < 0:    # the EtherCAT task found no torque of the period before (TaskEtherCAT.cpp)
            cnt['notorque'] += 1
        if fl & FLAGS['BUSSERVO']:
            cnt['busservo'] += 1
        d['EtherCAT t_w'].add(w)
        if not s1:                  # the task of S1 does not send
            d['EtherCAT t_s'].add(s)
            if rcv >= 0 and snd >= 0:
                d['  receive'].add(rcv)
                d['  send'].add(snd)
                d['  process'].add(s - rcv - snd)
            d['EtherCAT t_w + t_s'].add(w + s)
            if worst_ws is None or w + s > worst_ws[0]:
                worst_ws = (w + s, per)
        d['EtherCAT t_e'].add(e)
        # with -b the step of the simulated arm after the send is not part of the response of the task
        resp = w + e - (sim if (hybrid and sim >= 0) else 0)
        d['EtherCAT t_w + t_e'].add(resp)
        if sim >= 0:
            d['simulator step'].add(sim)
        if dyn >= 0:
            d['dynamics (RBDL calls)'].add(dyn)
        if hl >= 0:
            d['high-level controller'].add(hl)
        chain = None
        if tjw >= 0 or dyw >= 0:
            if hybrid and (sim_before is None or sim_before < 0):
                cnt['nosim'] += 1
            else:
                corr = sim_before if hybrid else 0
                if tjw >= 0 and tje >= 0:
                    d['Trajectory t_w'].add(tjw - corr)
                    d['Trajectory t_e'].add(tje)
                    d['Trajectory t_w + t_e'].add(tjw - corr + tje)
                if dyw >= 0 and dye >= 0:
                    # released dyn_delay after the others, the Dynamics task waited for the step only as far
                    # as the work before it (EtherCAT with the step, then Trajectory) ran past its release
                    before = max(ecat_end_before or 0, tjw + tje if (tjw >= 0 and tje >= 0) else 0)
                    corr_dyn = min(corr, max(0, before - dyn_delay)) if dyn_delay > 0 else corr
                    chain = wake_delay + dyn_delay + dyw - corr_dyn + dye
                    d['Dynamics t_w'].add(dyw - corr_dyn)
                    d['Dynamics t_e'].add(dye)
                    d[CHAIN].add(chain)
                    cnt['chain_n'] += 1
                    if chain >= period:
                        cnt['over_chain'] += 1
                    if worst_chain is None or chain > worst_chain[0]:
                        worst_chain = (chain, per - 1)
        if hh is not None:
            hh[1] = max(hh[1], 0 if s1 else w + s)
            hh[2] = max(hh[2], resp)
            hh[3] = max(hh[3], chain or 0)
        b = w // 1000
        tw_hist[b] = tw_hist.get(b, 0) + 1
    gaps = (last - first + 1) - present

    out = a.out or os.path.dirname(os.path.abspath(a.log))
    os.makedirs(out, exist_ok=True)
    L = []
    L.append('%s: mode %s, %s' % (os.path.basename(a.log), head.get('mode', '?'), head.get('created', '')))
    if s1:
        L.append('--arch S1: the cycles of the task that takes the State slot of the R5, released %.0f us after the '
                 'R5; the send and its deadline are those of the R5, not in this log' % (wake_delay / 1e3))
        L.append('window: %s, periods of the R5 %d..%d (%.1f s), the first %d cycles left out; %d cycles measured, %d '
                 'with a new State; T_s %.3f ms' % (a.window, first, last, (last - first + 1) * period * 1e-9, a.skip,
                                                     cnt['periods'], cnt['periods'] - cnt['nostate'], period / 1e6))
    else:
        L.append('window: %s, release periods %d..%d (%.1f s), the first %d left out; %d periods measured; T_s %.3f '
                 'ms, t_b %.1f us after the release; sync mode %s'
                 % (a.window, first, last, (last - first + 1) * period * 1e-9, a.skip, cnt['periods'], period / 1e6,
                    tb_ns / 1000.0, head.get('sync_mode', '-')))
    if head.get('tasks'):
        L.append('tasks (name:priority:cpu): %s' % head['tasks'])
    if any(delays):
        L.append('start delays [us]: %s (the chain includes that of the Dynamics task)'
                 % ','.join('%.0f' % (d / 1e3) for d in delays))
    if s1:
        L.append('the chain counts from the release of the R5: the %.0f us that the tasks follow behind it are in it, '
                 'the phase error of the following is not' % (wake_delay / 1e3))
    if head.get('call_timing') == '1':
        L.append('NOTE: call timing was on (G2T_CALL_TIMING): the times include the clock reads around the library '
                 'calls')
    if hybrid:
        L.append('-b: the step of the simulated arm (after the send) is left out of t_w + t_e of the EtherCAT task, of '
                 't_w of the Trajectory and the Dynamics task and of the chain (R_adj = R_raw - C_sim); %d periods '
                 'without that step in the record before left out of those' % cnt['nosim'])
    L.append('')
    L.append('%-28s %9s %9s %9s %9s %9s %9s' % ('[us]', 'mean', 'std', '99.9 %', '99.999 %', 'max', 'samples'))
    for k in names:
        x = d[k]
        if x.n == 0:
            continue
        L.append('%-28s %9s %9s %9s %9s %9s %9d' % (k, us(x.mean()), us(x.std()), us(x.pct(0.999)), us(x.pct(0.99999)),
                                                   us(x.max), x.n))
    cols = [('Send Delta T_send', d['Delta T_send (signed)']), ('EtherCAT t_w', d['EtherCAT t_w']),
            ('EtherCAT t_w + t_s', d['EtherCAT t_w + t_s']), ('Trajectory t_e', d['Trajectory t_e']),
            ('Dynamics t_e', d['Dynamics t_e']), ('Chain ' + CHAIN.split()[-1], d[CHAIN])]
    L.append('')
    L.append('the row of Table 6 (mean / 99.9 %% / max [us]; the send column signed%s):' %
             (', the chain without the simulator step' if hybrid else ''))
    row = []
    for c, x in cols:
        cell = '-' if x.n == 0 else '%s / %s / %s' % (us(x.mean()), us(x.pct(0.999)), us(x.max))
        row.append(cell)
        L.append('  %-20s %s' % (c, cell))
    L.append('')
    if s1:
        L.append('faults in the window: %d late releases, %d overruns, %d cycles without a new State, %d States of the '
                 'R5 that no cycle took, %d incomplete working counters, %d periods without OP'
                 % (cnt['late'], cnt['overrun'], cnt['nostate'], cnt['states_missed'], cnt['wkc'], cnt['notop']))
    else:
        L.append('faults in the window: %d late releases, %d overruns, %d missed send deadlines, %d incomplete '
                 'working counters, %d periods without OP, %d periods missing from the log'
                 % (cnt['late'], cnt['overrun'], cnt['missed'], cnt['wkc'], cnt['notop'], gaps))
    if cnt['chain_n']:
        L.append('periods in OP without the torque of the period before: %d (their chain is not in the log)%s'
                 % (cnt['notorque'], '; with -b the simulator step ran before the control task on the same core'
                    if hybrid and cnt['notorque'] else ''))
    if worst_ws:
        L.append('largest t_w + t_s: %.1f us (release period %d)' % (worst_ws[0] / 1000.0, worst_ws[1]))
    if worst_chain:
        L.append('largest chain: %.1f us (the cycle of release period %d)' % (worst_chain[0] / 1000.0, worst_chain[1]))
    if a.hourly and hours:
        L.append('')
        L.append('%6s %10s %10s %10s %10s %6s %8s %7s %5s' % ('hour', 'periods', 't_w+t_s', 't_w+t_e', 'chain', 'late',
                                                             'overrun', 'missed', 'wkc'))
        for h in sorted(hours):
            c = hours[h]
            L.append('%6d %10d %10s %10.1f %10.1f %6d %8d %7d %5d' % (h + 1, c[0], '-' if s1 else '%.1f' % (c[1] / 1e3),
                                                                     c[2] / 1e3, c[3] / 1e3, c[4], c[5], c[6], c[7]))
    if s1:
        crit = [('a new State of the R5 in every cycle', cnt['nostate'] == 0, '%d without' % cnt['nostate']),
                ('no State of the R5 that no cycle took', cnt['states_missed'] == 0, str(cnt['states_missed']))]
    else:
        crit = [('t_w + t_s <= t_b in every period', cnt['missed'] == 0, '%d missed' % cnt['missed'])]
    crit += [('no late release', cnt['late'] == 0, str(cnt['late'])),
             ('no overrun', cnt['overrun'] == 0, str(cnt['overrun'])),
             ('no incomplete working counter', cnt['wkc'] == 0, str(cnt['wkc']))]
    if cnt['chain_n']:
        crit.append(('chain < T_s in every period', cnt['over_chain'] == 0, '%d at or above' % cnt['over_chain']))
        crit.append(('a fresh torque in every period', cnt['notorque'] == 0, '%d without' % cnt['notorque']))
    if hybrid:
        crit.append(('-b: no drive of the bus enabled', cnt['busservo'] == 0, '%d periods' % cnt['busservo']))
    L.append('')
    L.append('criteria:')
    for name, ok, note in crit:
        L.append('  %-40s %-5s %s' % (name, 'PASS' if ok else 'FAIL', note))
    allok = all(ok for _, ok, _ in crit)
    text = '\n'.join(L)
    print(text)
    with open(os.path.join(out, 'timing_summary.txt'), 'w') as f:
        f.write(text + '\n')
    with open(os.path.join(out, 'timing_table.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['log'] + [c for c, _ in cols])
        w.writerow([os.path.basename(a.log)] + row)
    with open(os.path.join(out, 'timing_tw.hist'), 'w') as f:
        f.write('# Histogram of t_w of the EtherCAT task, 1 us bins (%s)\n' % os.path.basename(a.log))
        top = max(tw_hist) if tw_hist else 0
        for b in range(0, max(top + 1, 100)):
            f.write('%06d %06d\n' % (b, tw_hist.get(b, 0)))
        f.write('# Total: %09d\n' % d['EtherCAT t_w'].n)
        f.write('# Max Latencies: %05d\n' % ((d['EtherCAT t_w'].max or 0) // 1000))
    print('written: %s/timing_summary.txt, timing_table.csv, timing_tw.hist' % out)
    return 0 if allok else 1


if __name__ == '__main__':
    sys.exit(main())
