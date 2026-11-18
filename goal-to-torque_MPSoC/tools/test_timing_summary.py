"""timing_summary: the counts and the criteria of a timing log, with the EtherCAT task here and on the R5.

Guards what a log of --arch S1 is judged by: a State that no cycle took, a cycle without a new State, and a chain
that counts from the release of the R5. A slip there would pass a run whose torque came after that release.
    python3 -m pytest tools/test_timing_summary.py
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import timing_summary as ts  # noqa: E402

F = ts.FLAGS
T_NS = 1000000


def record(period, flags, t_w=50000, t_s=30000, t_e=40000, dyn_w=100000, dyn_e=20000):
    """a record of the EtherCAT task: its own times, and the Dynamics task's cycle of the period before"""
    return ts.REC.pack(period, t_w, t_s, t_e, 5000, 3000, -1, 0xFFFFFFFF, flags, 0, 1000 if t_s >= 0 else -1,
                       20000 if t_s >= 0 else -1, 60000, 10000, dyn_w, dyn_e)


def write_log(path, records, extra=''):
    head = ('format=%s\nrecord_bytes=%d\nmode=simulation\nperiod_ns=%d\ndeadline_ns=400000\n%s'
            % (ts.FORMAT, ts.REC.size, T_NS, extra)).encode()
    with open(path, 'wb') as f:
        f.write(head + b' ' * (ts.HEADER_BYTES - len(head) - 1) + b'\n')
        f.write(b''.join(records))


def summary(tmp_path, records, extra=''):
    log = os.path.join(str(tmp_path), 'timing_test.bin')
    write_log(log, records, extra)
    passed = ts.main([log, '--window', 'all', '--skip', '0']) == 0
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
