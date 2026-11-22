"""pytest: tools/trials.py, tools/trial_batch.py and tools/host/power_log.py on stand-in logs."""
import csv
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402
import trial_batch as B  # noqa: E402
import trials as T  # noqa: E402

CFG = os.path.join(HERE, '..', 'indy7', 'INDY7.cfg')
HOVER_OFF = [-0.05, 0.0, 0.20]


PLACE = (0.55, 0.15, 0.10)


def hl_log(d, name, dists_mm, obj=PLACE, result='completed', method='FULL', period=0.1):
    """a high-level log whose tool is dists_mm[i] from the hover point of PLACE (along x) at t = i * period;
    obj: the place that its .json names (None: none)"""
    d.mkdir(parents=True, exist_ok=True)
    path = d / name
    hover = [o + h for o, h in zip(PLACE, HOVER_OFF)]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'tool_x', 'tool_y', 'tool_z'])
        for i, mm in enumerate(dists_mm):
            w.writerow(['%.3f' % (i * period), hover[0] + mm * 1e-3, hover[1], hover[2]])
    meta = {'method': method, 'run': 1, 'result': result, 'period': period, 'goal_offset': HOVER_OFF,
            'object': list(obj) if obj is not None else None, 'timing_faults': 0}
    (d / name.replace('.csv', '.json')).write_text(json.dumps(meta))
    return str(path)


def test_wilson_interval_has_the_known_values():
    assert [round(v, 3) for v in T.wilson(16, 20)] == [0.584, 0.919]
    assert [round(v, 3) for v in T.wilson(0, 10)] == [0.0, 0.278]
    assert [round(v, 3) for v in T.wilson(10, 10)] == [0.722, 1.0]


def test_a_success_needs_the_hold_without_a_break_and_by_the_limit():
    t = [0.1 * i for i in range(30)]
    near = [20.0] * 5 + [5.0] * 3 + [20.0] + [5.0] * 10 + [20.0] * 11       # a break at 0.8 s
    assert T.judge(t, [x * 1e-3 for x in near], 0.010, 0.5, 3.0) == (True, pytest.approx(1.4))
    assert T.judge(t, [x * 1e-3 for x in near], 0.010, 0.5, 1.2) == (False, 1.2)    # the hold would end after
    assert T.judge(t, [x * 1e-3 for x in near], 0.010, 2.0, 3.0) == (False, 3.0)    # never 2 s in a row


def test_a_trial_is_judged_against_the_place_in_its_log(tmp_path):
    p = hl_log(tmp_path, 'hl_app_sim_r01_20261122_150000.csv', [30.0] * 10 + [2.0] * 20)
    tr = T.trial(p, 0.010, 0.5)
    assert tr['success'] and tr['time'] == pytest.approx(1.5) and tr['limit'] == pytest.approx(3.0)
    assert tr['object'] == [0.55, 0.15, 0.10] and tr['final_mm'] == pytest.approx(2.0)
    # the same way of the tool, with another place in its log: a failure, the whole limit
    q = hl_log(tmp_path, 'hl_app_sim_r02_20261122_150010.csv', [30.0] * 10 + [2.0] * 20, obj=(0.60, 0.15, 0.10))
    tr = T.trial(q, 0.010, 0.5)
    assert not tr['success'] and tr['time'] == pytest.approx(3.0)


def test_a_run_that_did_not_complete_or_has_no_place_fails(tmp_path):
    p = hl_log(tmp_path, 'hl_app_sim_r01_20261122_150000.csv', [2.0] * 30, result='aborted')
    tr = T.trial(p, 0.010, 0.5)
    assert not tr['success'] and 'aborted' in tr['why']
    q = hl_log(tmp_path / 'b', 'hl_app_robot_r01_20261122_150000.csv', [2.0] * 30, obj=None)
    assert not T.trial(q, 0.010, 0.5)['success']
    assert T.trial(q, 0.010, 0.5, obj=(0.55, 0.15, 0.10))['success']        # --object for one place


def test_the_summary_has_the_throughput_and_the_energy_per_success(tmp_path):
    ok = hl_log(tmp_path / 'm', 'hl_app_sim_r01_20261122_150000.csv', [2.0] * 30)               # 0.4 s
    bad = hl_log(tmp_path / 'm', 'hl_app_sim_r02_20261122_150010.csv', [30.0] * 30)             # 3.0 s
    trials = [T.trial(ok, 0.010, 0.4), T.trial(bad, 0.010, 0.4)]
    power = tmp_path / 'power.csv'
    t0 = T.start_unix(ok)
    rows = ['unix_s,power_w'] + ['%.1f,%.1f' % (t0 + 0.1 * i, 6.0 if i < 100 else 9.0) for i in range(200)]
    power.write_text('\n'.join(rows) + '\n')
    s = T.summary(trials, T.Power(str(power)))['FULL']
    assert (s['trials'], s['successes']) == (2, 1)
    assert s['Y_per_min'] == pytest.approx(60.0 / 3.4)
    assert s['power_w'] == pytest.approx((6.0 * 0.4 + 9.0 * 3.0) / 3.4)     # time-weighted over the trials
    assert s['energy_per_success_j'] == pytest.approx(6.0 * 0.4 + 9.0 * 3.0)  # all the energy, per success
    # samples for the first trial only: the mean power of what was sampled, scaled to all the trial time
    power.write_text('\n'.join(rows[:20]) + '\n')
    s = T.summary(trials, T.Power(str(power)))['FULL']
    assert s['power_w'] == pytest.approx(6.0) and s['energy_per_success_j'] == pytest.approx(6.0 * 3.4)
    assert s['power_trials'] == 1


def test_the_power_of_the_resource_monitor_is_read_too(tmp_path):
    p = tmp_path / 'run.csv'
    p.write_text('wall_time_s,tick,ina260_u14_power1_input\n100.0,1,8000000\n101.0,2,9000000\n')
    pw = T.Power(str(p))
    assert pw.t == [100.0, 101.0] and pw.w == [8.0, 9.0]
    e, n = pw.energy(99.5, 101.5)
    assert n == 2 and e == pytest.approx(8.5 * 2.0)


def test_the_batch_gives_every_method_the_same_sequence_of_places():
    places = B.PLACES
    a, b = B.sequence(places, 2, 1), B.sequence(places, 2, 1)
    assert a == b and len(a) == 10
    assert sorted(a[:5]) == sorted(places) and sorted(a[5:]) == sorted(places)     # each round every place once
    assert a[:5] != a[5:] or B.sequence(places, 2, 2) != a                         # the order changes
    with pytest.raises(ValueError):
        B.parse_places('0.5 0.1')
    assert B.parse_places('0.5 0.1 0.1; 0.6 0 0.1') == [(0.5, 0.1, 0.1), (0.6, 0.0, 0.1)]


def test_the_batch_cfg_has_the_places_and_the_limit(tmp_path):
    dst = tmp_path / 'trial_batch.cfg'
    seq = B.sequence(B.PLACES[:2], 1, 1)
    B.write_cfg(CFG, str(dst), B.cfg_items(seq, 12.0, '/tmp/x', cpu=1))
    v = cfgset.values(str(dst), ['SIM_CAM', 'APP_DURATION_S', 'SIM_CAM_OBJECT_M', 'JOINT_TEST_LOG_DIR',
                                 'TASK0.CPU', 'TASK1.CPU'])
    assert v['SIM_CAM'] == '1' and v['APP_DURATION_S'] == '12' and v['JOINT_TEST_LOG_DIR'] == '/tmp/x'
    assert [float(x) for x in v['SIM_CAM_OBJECT_M'].split()] == [c for p in seq for c in p]
    assert v['TASK0.CPU'] == v['TASK1.CPU'] == '1'
    with pytest.raises(ValueError):
        B.write_cfg(CFG, str(dst), [('NO_SUCH_KEY', '1')])


@pytest.mark.parametrize('files, watts', [({'power1_input': '7500000'}, 7.5),
                                          ({'in1_input': '5000', 'curr1_input': '1600'}, 8.0)])
def test_power_log_samples_the_ina260(tmp_path, files, watts):
    for name, sensor in (('hwmon0', 'other'), ('hwmon3', 'ina260_u14')):
        d = tmp_path / name
        d.mkdir()
        (d / 'name').write_text(sensor + '\n')
        if sensor.startswith('ina260'):
            for f, v in files.items():
                (d / f).write_text(v + '\n')
    out = tmp_path / 'p.csv'
    subprocess.run([sys.executable, '-B', os.path.join(HERE, 'host', 'power_log.py'), '--out', str(out),
                    '--period', '0.05', '--duration', '0.3', '--hwmon-root', str(tmp_path)], check=True,
                   capture_output=True)
    rows = list(csv.DictReader(open(out)))
    assert len(rows) >= 3 and all(float(r['power_w']) == pytest.approx(watts) for r in rows)
