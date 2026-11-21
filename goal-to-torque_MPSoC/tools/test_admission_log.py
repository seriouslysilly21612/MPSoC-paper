"""admission_log: the decision log of the camera side, joined to the log of the control application.

Guards the join by the capture time and the sign of "later than predicted": a slip there would report a rule that
keeps its promise when it does not. No ROS needed:
    python3 -m pytest tools/test_admission_log.py
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import admission_log  # noqa: E402

TASK_MINUS_SYSTEM = -1000.0     # the clock of the tasks started 1000 s after the system clock's zero
RUN_START = 500.0               # start of the control application's log on the clock of the tasks


def decision(action, capture_task_s, mode=0, reason='', first_use_ms=181.0, age_ms=231.0, checks='111'):
    """a row of the decision log: an image captured at capture_task_s and released 50 ms later"""
    row = {'action': action, 'mode': str(mode if action == 'run' else -1), 'reason': reason,
           'capture_task_s': '%.6f' % capture_task_s, 'capture_s': '%.6f' % (capture_task_s - TASK_MINUS_SYSTEM),
           'release_s': '%.6f' % (capture_task_s - TASK_MINUS_SYSTEM + 0.050)}
    for m in (0, 1):
        candidate = m == mode and reason in ('', 'no_candidate_passes')
        row.update({'m%d_first_use_ms' % m: '%.3f' % first_use_ms if candidate else '',
                    'm%d_age_ms' % m: '%.3f' % age_ms if candidate else '',
                    'm%d_in_time' % m: checks[0] if candidate else '',
                    'm%d_within_tolerance' % m: checks[1] if candidate else '',
                    'm%d_response_ok' % m: checks[2] if candidate else ''})
    return row


def result(capture_task_s, taken_after_capture_s, verdict='accepted'):
    """a row of the control application's log, times from the start of its run"""
    return {'t_e': '%.6f' % (capture_task_s - RUN_START), 'verdict': verdict,
            't_in': '%.6f' % (capture_task_s - RUN_START + taken_after_capture_s)}


def write_obs(tmp_path, rows):
    path = tmp_path / 'obs_FULL_sim_r01_20261118_000000.csv'
    with open(path, 'w') as f:
        f.write('t_e,verdict,t_in\n' + ''.join('%s,%s,%s\n' % (r['t_e'], r['verdict'], r['t_in']) for r in rows))
    with open(str(path)[:-4] + '.json', 'w') as f:
        json.dump({'t_start_ticks': int(RUN_START * 1e9), 'ticks_per_s': 1e9}, f)
    return str(path)


def test_join_takes_the_result_of_the_same_capture_only():
    results = [(600.000, {'id': 'a'}), (600.033, {'id': 'b'}), (600.100, {'id': 'c'})]
    decisions = [decision('run', 600.0004), decision('skip', 600.033), decision('run', 600.0325),
                 decision('run', 600.0668), decision('reject', 600.100, reason='no_status'), decision('run', 600.1012)]
    joined = admission_log.join(decisions, results)
    assert [row['capture_task_s'] for row, _ in joined] == ['600.000400', '600.032500', '600.066800', '600.101200']
    assert [r and r['id'] for _, r in joined] == ['a', 'b', None, None]     # 0.4, 0.5 ms; none; 1.2 ms off
    assert admission_log.join(decisions, []) == [(row, None) for row, _ in joined]


def test_failed_checks_are_those_that_no_mode_passed():
    assert admission_log.failed_checks(decision('reject', 1.0, reason='no_candidate_passes', checks='011')) == {
        'time': True, 'error': False, 'response': False}
    both = decision('reject', 1.0, reason='no_candidate_passes', checks='010')
    both.update(m1_in_time='1', m1_within_tolerance='0', m1_response_ok='0')    # the other mode was in time
    assert admission_log.failed_checks(both) == {'time': False, 'error': False, 'response': True}
    assert admission_log.failed_checks(decision('reject', 1.0, reason='no_status')) == {
        'time': False, 'error': False, 'response': False}


def test_report_counts_and_compares_the_arrival_with_the_prediction(tmp_path):
    decisions = [
        decision('run', 600.000),                                   # arrives 20 ms before the predicted first use
        decision('run', 600.033),                                   # arrives 9 ms after it
        decision('run', 600.066),                                   # never reaches the control task
        decision('skip', 600.100),
        decision('reject', 600.133, reason='no_candidate_passes', checks='011'),
        decision('reject', 600.166, reason='status_stale'),
    ]
    # predicted first use: release (capture + 50 ms) + 181 ms = capture + 231 ms
    obs = write_obs(tmp_path, [result(600.000, 0.211), result(600.033, 0.240, 'expired'), result(600.500, 0.200)])
    out = io.StringIO()
    admission_log.report(decisions, obs, out)
    text = out.getvalue()
    assert '6 decisions: 3 run, 1 skipped, 2 rejected' in text
    assert 'rejected: 1 no_candidate_passes, 1 status_stale' in text
    assert 'of the 1 with candidates, no mode passed: the time check in 1, the error check in 0' in text
    assert 'predicted age of a run image at its first use: median 231.0 ms, largest 231.0 ms' in text
    assert '2 of the 3 run images reached the control task' in text
    assert 'their verdicts: 1 accepted, 1 expired' in text
    assert 'median +9.0 ms' in text and 'largest +9.0 ms' in text and 'later than predicted: 1 of 2' in text


def test_report_without_the_log_of_the_control_application():
    out = io.StringIO()
    admission_log.report([decision('reject', 600.0, reason='no_status')], None, out)
    assert out.getvalue() == '1 decisions: 0 run, 0 skipped, 1 rejected\n  rejected: 1 no_status\n'


def test_an_s2_log_starts_on_the_clock_of_the_decisions(tmp_path):
    # --arch S2 (I-119): the ticks of the log are the R5's counter; t_start_monotonic_ns is the same start on the
    # clock of the decision log, and the join goes by it
    path = write_obs(tmp_path, [result(600.0, 0.15)])
    with open(path[:-4] + '.json', 'w') as f:
        json.dump({'t_start_ticks': 123456789012, 'ticks_per_s': 99999000.0,
                   't_start_monotonic_ns': int(RUN_START * 1e9)}, f)
    start_s, results = admission_log.results_by_capture(path)
    assert abs(start_s - RUN_START) < 1e-6
    assert abs(results[0][0] - 600.0) < 1e-6
