"""timing_summary: the counts and the criteria of a timing log, with the EtherCAT task here and on the R5.

Guards what a log of --arch S1 is judged by: a State that no cycle took, a cycle without a new State, a chain
that counts from the release of the R5, a torque the R5 did not have at its release, and the send of the R5 from
its own log (--r5). A slip there would pass a run whose torque came after that release, or whose send was late.
And what a run with --shadow says about the control task on the R5 (D-14): a slip there would move the control
task to the R5 on a time that was not that of its control computation.
    python3 -m pytest tools/test_timing_summary.py
"""
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import timing_summary as ts  # noqa: E402

F = ts.FLAGS
T_NS = 1000000


def record(period, flags, t_w=50000, t_s=30000, t_e=40000, dyn_w=100000, dyn_e=20000, sim=-1, flags2=0):
    """a record of the EtherCAT task: its own times, and the Dynamics task's cycle of the period before"""
    return ts.REC.pack(period, t_w, t_s, t_e, 5000, 3000, sim, 0xFFFFFFFF, flags, flags2, 1000 if t_s >= 0 else -1,
                       20000 if t_s >= 0 else -1, 60000, 10000, dyn_w, dyn_e)


def write_log(path, records, extra=''):
    head = ('format=%s\nrecord_bytes=%d\nmode=simulation\nperiod_ns=%d\ndeadline_ns=400000\n%s'
            % (ts.FORMAT, ts.REC.size, T_NS, extra)).encode()
    with open(path, 'wb') as f:
        f.write(head + b' ' * (ts.HEADER_BYTES - len(head) - 1) + b'\n')
        f.write(b''.join(records))


def summary(tmp_path, records, extra='', args=()):
    log = os.path.join(str(tmp_path), 'timing_test.bin')
    write_log(log, records, extra)
    passed = ts.main([log, '--window', 'all', '--skip', '0'] + list(args)) == 0
    return passed, open(os.path.join(str(tmp_path), 'timing_summary.txt')).read()


def test_here_a_clean_run_passes_and_a_period_missing_from_the_log_is_counted(tmp_path):
    on = F['OP'] | F['TRACK']
    passed, text = summary(tmp_path, [record(k, on) for k in (10, 11, 12)])
    assert passed and '0 periods missing from the log' in text and 't_w + t_s <= t_b in every period' in text
    passed, text = summary(tmp_path, [record(k, on) for k in (10, 11, 13)])
    assert '1 periods missing from the log' in text


def test_here_a_missed_send_deadline_fails(tmp_path):
    on = F['OP'] | F['TRACK']
    passed, text = summary(tmp_path, [record(10, on), record(11, on | F['MISSED']), record(12, on)])
    assert not passed and '1 missed send deadlines' in text


S1 = 'arch=S1\nwake_delay_ns=150000\n'


def s1_record(period, flags=0, **times):
    return record(period, F['R5'] | flags, t_s=-1, **times)


def test_s1_a_clean_run_passes_without_the_rows_of_a_send(tmp_path):
    on = F['OP'] | F['TRACK']
    passed, text = summary(tmp_path, [s1_record(k, on) for k in (84, 85, 86)], S1)
    assert passed
    assert 'Delta T_send (signed)' not in text and 'EtherCAT t_s' not in text and 'send deadlines' not in text
    assert '3 cycles measured, 3 with a new State' in text
    assert '0 cycles without a new State, 0 States of the R5 that no cycle took' in text


def test_s1_a_cycle_without_a_state_and_states_that_no_cycle_took_fail(tmp_path):
    on = F['OP'] | F['TRACK']
    # period 85 taken, then a cycle that found nothing new, then period 88: 86 and 87 went by untaken
    records = [s1_record(84, on), s1_record(85, on), s1_record(85, F['NOSTATE']), s1_record(88, on)]
    passed, text = summary(tmp_path, records, S1)
    assert not passed
    assert '4 cycles measured, 3 with a new State' in text
    assert '1 cycles without a new State, 2 States of the R5 that no cycle took' in text
    # the cycle without a State is not one without OP, nor one without a torque
    assert '0 periods without OP' in text and 'torque of the period before: 0' in text
    # either of the two alone fails the run
    passed, text = summary(tmp_path, [s1_record(84, on), s1_record(85, on), s1_record(88, on)], S1)
    assert not passed and '0 cycles without a new State, 2 States of the R5 that no cycle took' in text
    passed, text = summary(tmp_path, [s1_record(84, on), s1_record(84, F['NOSTATE']), s1_record(85, on)], S1)
    assert not passed and '1 cycles without a new State, 0 States of the R5 that no cycle took' in text


def test_s1_the_periods_before_the_first_state_taken_are_not_missed_states(tmp_path):
    on = F['OP'] | F['TRACK']
    # a cycle before the first State (its record has period 0), then the R5 at its period 84
    passed, text = summary(tmp_path, [s1_record(0, F['NOSTATE']), s1_record(84, on), s1_record(85, on)], S1)
    assert '1 cycles without a new State, 0 States of the R5 that no cycle took' in text


def test_s1_a_log_in_which_the_r5_started_again_is_refused(tmp_path):
    on = F['OP'] | F['TRACK']
    with pytest.raises(SystemExit, match='started again'):
        summary(tmp_path, [s1_record(84, on), s1_record(85, on), s1_record(1, on), s1_record(2, on)], S1)


def test_s1_the_chain_counts_from_the_release_of_the_r5(tmp_path):
    on = F['OP'] | F['TRACK']
    # the torque 849 us after the release of the tasks: 999 us after that of the R5, just before its next release
    passed, text = summary(tmp_path, [s1_record(k, on, dyn_w=100000, dyn_e=749000) for k in (84, 85)], S1)
    assert passed and '0 at or above' in text
    # 850 us: at the next release of the R5, which has taken the Torque slot by then
    passed, text = summary(tmp_path, [s1_record(84, on, dyn_w=100000, dyn_e=749000),
                                      s1_record(85, on, dyn_w=100000, dyn_e=750000)], S1)
    assert not passed and '1 at or above' in text
    # the same times with the EtherCAT task here are 150 us earlier in the period: no fault
    passed, text = summary(tmp_path, [record(k, on, dyn_w=100000, dyn_e=750000) for k in (84, 85)])
    assert passed


S1_MARKS = S1 + 'flag2_bits=TORQUE_LATE:1\n'
LATE = ts.FLAGS2['TORQUE_LATE']


def test_s1_a_torque_the_r5_did_not_have_fails_and_a_log_without_the_mark_has_no_such_criterion(tmp_path):
    on = F['OP'] | F['TRACK']
    records = [s1_record(84, on), s1_record(85, on, flags2=LATE), s1_record(86, on)]
    passed, text = summary(tmp_path, records, S1_MARKS)
    assert not passed and 'torques that the R5 did not have at its release: 1' in text
    assert 'the torque at the R5 by its next release FAIL  1 late' in text
    passed, text = summary(tmp_path, [s1_record(k, on) for k in (84, 85, 86)], S1_MARKS)
    assert passed and 'the torque at the R5 by its next release PASS  0 late' in text
    # a log of before the mark: the field was 0 whatever happened, so nothing is said about it
    passed, text = summary(tmp_path, [s1_record(k, on) for k in (84, 85, 86)], S1)
    assert passed and 'the torque at the R5' not in text and 'did not have at its release' not in text


def test_s1_with_b_a_torque_late_only_by_the_simulator_step_is_counted_apart(tmp_path):
    on = F['OP'] | F['TRACK']
    hybrid = S1_MARKS + 'mode=hybrid\n'
    # the step of period 84 took 700 us before the control task: its torque 1050 us after the release of the R5,
    # 350 us without the step
    by_step = [s1_record(84, on, sim=700000), s1_record(85, on, sim=80000, dyn_w=800000, dyn_e=100000, flags2=LATE),
               s1_record(86, on, sim=80000)]
    passed, text = summary(tmp_path, by_step, hybrid)
    assert passed and 'at its release: 1 (1 of them only by the simulator step' in text
    assert 'PASS  0 late, 1 more by the simulator step only' in text
    # late with a chain that ended before the release even with the step: not the step (the tasks ran off the R5)
    other = [s1_record(84, on, sim=80000), s1_record(85, on, sim=80000, flags2=LATE), s1_record(86, on, sim=80000)]
    passed, text = summary(tmp_path, other, hybrid)
    assert not passed and 'at its release: 1 (0 of them only' in text and 'FAIL  1 late' in text
    # late and past the release without the step as well
    long = [s1_record(84, on, sim=80000), s1_record(85, on, sim=80000, dyn_w=900000, dyn_e=100000, flags2=LATE)]
    passed, text = summary(tmp_path, long, hybrid)
    assert not passed and 'FAIL  1 late' in text and '1 at or above' in text
    # without -b there is no step to take out: the same times are a fault
    passed, text = summary(tmp_path, [s1_record(84, on), s1_record(85, on, dyn_w=800000, dyn_e=100000, flags2=LATE)],
                           S1_MARKS)
    assert not passed and 'FAIL  1 late' in text


def r5_record(period, t_w=3000, t_s=63000, flags=0, t_e=None, traj=-1, dyn=-1):
    """a record of the R5's own log (cycle_record.c): BUS and OP, nothing of the other tasks; with --shadow the
    cycle goes on after the send (t_e) with the two parts of the control computation (traj, dyn; -1: not run)"""
    return ts.REC.pack(period, t_w, t_s, t_s if t_e is None else t_e, dyn, traj, -1, 0xFFFFFFFF,
                       F['BUS'] | F['OP'] | flags, 0, 20000, 16000, -1, -1, -1, -1)


def r5_log(tmp_path, records, extra='mode=r5\n'):
    path = os.path.join(str(tmp_path), 'r5', 'timing_r5_test.bin')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    write_log(path, records, extra)
    return path


def test_s1_the_send_columns_come_from_the_log_of_the_r5_over_the_periods_of_the_window(tmp_path):
    on = F['OP'] | F['TRACK']
    apu = [s1_record(k, on) for k in range(84, 90)]
    # the R5 ran before and after the window; in it, period 86 sent 10 us later than the others
    r5 = r5_log(tmp_path, [r5_record(k, t_s=73000 if k == 86 else 63000) for k in range(80, 95)])
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert passed
    assert 'R5 Delta T_send (signed)           0.0' in text and 'R5 |Delta T_send|' in text
    assert 'Send Delta T_send    0.0 / 10.0 / 10.0' in text         # +10 us into period 86, -10 us out of it
    assert 'EtherCAT t_w         3.0 / 3.0 / 3.0' in text           # of the R5, not the 50 us of the task here
    assert 'EtherCAT t_w + t_s   67.7 / 76.0 / 76.0' in text        # 5 x 66 us and 76 us
    assert 'largest t_w + t_s of the R5: 76.0 us (its period 86)' in text
    assert '0 missed send deadlines, 0 incomplete working counters, 0 periods without OP, 0 periods missing' in text
    # the same periods read as a log by themselves (83 left out as the first) give the same three columns
    alone = r5_log(tmp_path / 'alone', [r5_record(k, t_s=73000 if k == 86 else 63000) for k in range(83, 90)])
    assert ts.main([alone, '--window', 'all', '--skip', '1']) == 0
    by_itself = open(os.path.join(os.path.dirname(alone), 'timing_summary.txt')).read()
    for column in ('Send Delta T_send    0.0 / 10.0 / 10.0', 'EtherCAT t_w         3.0 / 3.0 / 3.0',
                   'EtherCAT t_w + t_s   67.7 / 76.0 / 76.0'):
        assert column in by_itself and column in text
    # --skip leaves the same first periods out on both sides
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5, '--skip', '3'])
    assert 'EtherCAT t_w + t_s   66.0 / 66.0 / 66.0' in text        # periods 87..89


def test_s1_a_fault_of_the_r5_or_a_period_missing_from_its_log_fails(tmp_path):
    on = F['OP'] | F['TRACK']
    apu = [s1_record(k, on) for k in range(84, 90)]
    for flag, crit in (('MISSED', 'R5: t_w + t_s <= t_b in every period     FAIL  1 missed'),
                       ('LATE', 'R5: no late release, no overrun          FAIL  1 late, 0 overruns'),
                       ('OVERRUN', 'R5: no late release, no overrun          FAIL  0 late, 1 overruns'),
                       ('WKC', 'R5: no incomplete working counter        FAIL  1')):
        r5 = r5_log(tmp_path, [r5_record(k, flags=F[flag] if k == 87 else 0) for k in range(80, 95)])
        passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
        assert not passed and crit in text, flag
    # a fault outside the window is not one of this run
    r5 = r5_log(tmp_path, [r5_record(k, flags=F['MISSED'] if k in (83, 90) else 0) for k in range(80, 95)])
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert passed
    # a period of the R5 without OP (its working counter incomplete, or its bus not up) is told
    r5 = r5_log(tmp_path, [r5_record(k) if k != 87 else ts.REC.pack(87, 3000, 63000, 63000, -1, -1, -1, 0xFFFFFFFF,
                                                                    F['BUS'], 0, 20000, 16000, -1, -1, -1, -1)
                           for k in range(80, 95)])
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert '0 incomplete working counters, 1 periods without OP, 0 periods missing from its log' in text
    # the reader of the ring lost period 87, or started after the window began, or recorded another time
    for periods, missing in (([k for k in range(80, 95) if k != 87], 1), (range(86, 95), 2), (range(200, 210), 6)):
        passed, text = summary(tmp_path, apu, S1, ['--r5', r5_log(tmp_path, [r5_record(k) for k in periods])])
        assert not passed and 'R5: every period of the window logged    FAIL  %d missing' % missing in text
    # no interval between two sends across the period that is not there: 84, 85, 86 and 89 have one
    r5 = r5_log(tmp_path, [r5_record(k, t_s=63000 + 1000 * k) for k in range(83, 90) if k != 87])
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert re.search(r'R5 \|Delta T_send\| +1\.0 +0\.0 +1\.0 +1\.0 +1\.0 +4\n', text)


def test_the_log_of_the_r5_is_taken_only_with_a_log_of_s1_and_only_if_it_is_one(tmp_path):
    on = F['OP'] | F['TRACK']
    r5 = r5_log(tmp_path, [r5_record(k) for k in range(80, 95)])
    with pytest.raises(SystemExit, match='--arch S1'):
        summary(tmp_path, [record(k, on) for k in (84, 85)], '', ['--r5', r5])
    apu_as_r5 = r5_log(tmp_path, [s1_record(k, on) for k in range(80, 95)], S1)
    with pytest.raises(SystemExit, match='not a log of r5rec.py'):
        summary(tmp_path, [s1_record(k, on) for k in (84, 85)], S1, ['--r5', apu_as_r5])
    with pytest.raises(SystemExit, match='started again'):
        summary(tmp_path, [s1_record(k, on) for k in (84, 85)], S1,
                ['--r5', r5_log(tmp_path, [r5_record(k) for k in (84, 85, 1, 2)])])


OPTIMIZED = 'mode=r5\noptimized=1\n'


def test_shadow_the_control_computation_of_the_r5_is_judged_by_its_own_cycles(tmp_path):
    on = F['OP'] | F['TRACK']
    apu = [s1_record(k, on) for k in range(84, 90)]
    cycles = {85: dict(t_e=185000, traj=20000, dyn=100000),         # both parts
              86: dict(t_e=155000, dyn=90000),                      # the law only
              87: dict(t_e=425000, traj=250000, dyn=110000),        # the largest with the control computation
              88: dict(t_e=950000),                                 # longer, but without it: not what is judged
              92: dict(t_e=990000, traj=600000, dyn=300000)}        # after the window
    r5 = r5_log(tmp_path, [r5_record(k, **cycles.get(k, {})) for k in range(80, 95)], OPTIMIZED)
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert passed                                                   # the verdict is no criterion of the run
    assert re.search(r'R5 Trajectory part +135\.0 +115\.0 +250\.0 +250\.0 +250\.0 +2\n', text)
    assert re.search(r'R5 Dynamics part +100\.0 +8\.2 +110\.0 +110\.0 +110\.0 +3\n', text)
    assert re.search(r'R5 control \(both parts\) +190\.0 +\S+ +360\.0 +360\.0 +360\.0 +3\n', text)
    assert re.search(r'R5 t_w \+ t_e with control +258\.0 +\S+ +428\.0 +428\.0 +428\.0 +3\n', text)
    assert ('largest t_w + t_e of the R5 with the control computation: 428.0 us (its period 87: t_w 3.0, t_s 63.0, '
            'the Trajectory part 250.0, the Dynamics part 110.0)') in text
    assert 'the control computation on the R5 (--shadow), in 3 of the 6 periods of the window' in text
    assert '  largest t_w + t_e 428.0 us, at most 500 us: the control task fits on the R5 (S2)' in text
    # the first periods left out on both sides: 87 is not in the window any more
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5, '--skip', '4'])
    assert 'control computation' not in text and 'R5 Dynamics part' not in text
    # a cycle with the law only is told as that
    r5 = r5_log(tmp_path, [r5_record(k, **cycles.get(k, {})) for k in range(80, 95) if k != 87], OPTIMIZED)
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert ('188.0 us (its period 85: t_w 3.0, t_s 63.0, the Trajectory part 20.0, the Dynamics part 100.0)') in text
    r5 = r5_log(tmp_path, [r5_record(k, **cycles.get(k, {})) for k in range(80, 95) if k not in (85, 87)], OPTIMIZED)
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert '158.0 us (its period 86: t_w 3.0, t_s 63.0, the Trajectory part -, the Dynamics part 90.0)' in text


@pytest.mark.parametrize('worst_ns, verdict', [
    (500000, 'at most 500 us: the control task fits on the R5 (S2)'),
    (500001, 'above 500 us, at most 800 us: S2 with a small margin'),
    (800000, 'above 500 us, at most 800 us: S2 with a small margin'),
    (800001, 'above 800 us: trajectory generation (IK and the replanning) leaves the 1 ms task')])
def test_shadow_the_two_lines_of_the_verdict(tmp_path, worst_ns, verdict):
    on = F['OP'] | F['TRACK']
    apu = [s1_record(k, on) for k in range(84, 90)]
    # t_w counts: the cycle started 40 us after its release
    r5 = r5_log(tmp_path, [r5_record(k, t_w=40000, t_e=worst_ns - 40000 if k == 86 else 150000, traj=10000, dyn=70000)
                           for k in range(80, 95)], OPTIMIZED)
    passed, text = summary(tmp_path, apu, S1, ['--r5', r5])
    assert passed and '  largest t_w + t_e %.1f us, %s' % (worst_ns / 1000.0, verdict) in text
    assert sum(text.count(word) for word in ('at most 500 us:', 'at most 800 us:', 'above 800 us:')) == 1


def test_shadow_no_verdict_from_a_firmware_that_is_not_an_optimized_build(tmp_path):
    on = F['OP'] | F['TRACK']
    apu = [s1_record(k, on) for k in range(84, 90)]
    records = [r5_record(k, t_e=150000, traj=10000, dyn=70000) for k in range(80, 95)]
    for extra, told in (('mode=r5\noptimized=0\n', 'optimized=0'), ('mode=r5\n', 'optimized=?')):
        passed, text = summary(tmp_path, apu, S1, ['--r5', r5_log(tmp_path, records, extra)])
        assert '  no verdict: the firmware is not an optimized build (%s in its log)' % told in text
        assert 'at most' not in text and 'R5 control (both parts)' in text         # the times are still given


def test_s1_a_run_without_shadow_has_nothing_of_the_control_computation_on_the_r5(tmp_path):
    on = F['OP'] | F['TRACK']
    r5 = r5_log(tmp_path, [r5_record(k) for k in range(80, 95)], OPTIMIZED)
    passed, text = summary(tmp_path, [s1_record(k, on) for k in range(84, 90)], S1, ['--r5', r5])
    assert passed and 'control computation' not in text and 'verdict' not in text
    assert not any(row in text for row in ts.R5_CTRL_ROWS)
