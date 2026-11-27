"""frame_gate: what the detector does with an image, by the method of the run.

Guards the wiring between the node and the rule: the files a method needs,
the mode of the loaded model, which methods look at the status of the control
task, and the row that every decision leaves in the log. The rule itself is
tested in test_admission.py. Pure stdlib + PyYAML, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_frame_gate.py
"""
import csv
import json
import time
from dataclasses import replace

import pytest

from vitis_ai_detector_pkg import admission
from vitis_ai_detector_pkg.contract import read_contract
from vitis_ai_detector_pkg.frame_gate import FrameGate, GateError, LOG_COLUMNS, mode_of_model

CFG = """\
[CONTRACT]
DELTA_MM=10
AGE_CAP_MS=250
E0_MM=1
G=1
EPS_V_MPS=0
A_MAX_MPS2=0
EPS_CAL_MM=2
EPS_M_MM=2 3
"""

PROFILE = """\
frame_period_ms: 33.0
acquisition_ms: 50.0
status_max_age_ms: 100.0
reference_speed_mps: 0.2
response_margin_us: 0.0
weights: {error: 1.0, energy: 0.0, ddr: 0.0}
modes:
  0: {name: "416", release_to_publication_ms: 100.0, energy_j: 0.0, ddr_bytes: 0.0}
  1: {name: "320", release_to_publication_ms: 80.0, energy_j: 0.0, ddr_bytes: 0.0}
"""

NOW = 1000.0


@pytest.fixture
def files(tmp_path):
    """Return the paths a gate needs: the contract, the profile and a 320 model."""
    cfg = tmp_path / 'INDY7.cfg'
    cfg.write_text(CFG, encoding='utf-8')
    profile = tmp_path / 'mode_profile.yaml'
    profile.write_text(PROFILE, encoding='utf-8')
    model_dir = tmp_path / 'model'
    model_dir.mkdir()
    (model_dir / 'decode_meta.json').write_text(json.dumps({'input_size': 320}), encoding='utf-8')
    return str(cfg), str(profile), str(model_dir / 'net.xmodel')


def status_of(gate, **changes):
    """Return a fresh status of a control task that runs the method of the gate."""
    plain = admission.Status(
        stamp_s=NOW - 0.001, tolerance_m=0.010, control_period_s=0.001,
        contract_crc=gate.contract.crc(), method=gate.method, result_in_use=False,
        result_capture_s=0.0, result_valid_until_s=0.0, result_mode=1)
    return replace(plain, **changes)


def test_the_mode_of_a_model_is_that_of_its_input_size(tmp_path):
    meta = tmp_path / 'decode_meta.json'
    model = str(tmp_path / 'net.xmodel')
    for input_size, mode in ((416, 0), (320, 1)):
        meta.write_text(json.dumps({'input_size': input_size}), encoding='utf-8')
        assert mode_of_model(model) == mode
    for unusable in ({'input_size': 512}, {'input_size': True}, {'names': []}, [416]):
        meta.write_text(json.dumps(unusable), encoding='utf-8')
        with pytest.raises(GateError):
            mode_of_model(model)
    meta.write_text('{', encoding='utf-8')
    with pytest.raises(GateError):
        mode_of_model(model)
    with pytest.raises(GateError):
        mode_of_model(str(tmp_path / 'elsewhere' / 'net.xmodel'))


def test_the_gate_reads_what_its_method_needs(files):
    cfg, profile, model = files
    gate = FrameGate('FULL', cfg, profile, model)
    assert gate.contract == read_contract(cfg)
    assert gate.modes == (1,)                       # the one model that is loaded
    assert gate.response_table is None              # no table: check 3 is off
    assert gate.uses_status and FrameGate('A3', cfg, profile, model).uses_status
    assert not FrameGate('M2', cfg, profile, model).uses_status
    assert not FrameGate('A2', cfg, profile, model).uses_status


def test_a_gate_that_cannot_be_set_up_is_refused(files, tmp_path):
    cfg, profile, model = files
    for method in ('M1', 'full', ''):
        with pytest.raises(GateError, match='no gate'):
            FrameGate(method, cfg, profile, model)
    with pytest.raises(GateError, match='cannot read'):
        FrameGate('FULL', str(tmp_path / 'none.cfg'), profile, model)
    with pytest.raises(GateError, match='cannot read'):
        FrameGate('FULL', cfg, str(tmp_path / 'none.yaml'), model)
    with pytest.raises(GateError, match='cannot read'):
        FrameGate('FULL', cfg, profile, model, str(tmp_path / 'none.csv'))
    with pytest.raises(GateError, match='input size'):
        FrameGate('FULL', cfg, profile, str(tmp_path / 'net.xmodel'))
    only_416 = tmp_path / 'only_416.yaml'
    only_416.write_text(PROFILE.split('  1:')[0], encoding='utf-8')
    with pytest.raises(GateError, match='no entry for mode 1'):
        FrameGate('FULL', cfg, str(only_416), model)


def test_the_response_table_is_read_when_one_is_given(files, tmp_path):
    cfg, profile, model = files
    table = tmp_path / 'response.csv'
    table.write_text('mode,level,response_us\n1,0,2000\n', encoding='utf-8')
    gate = FrameGate('FULL', cfg, profile, model, str(table))
    gate.on_status(status_of(gate))
    decision = gate.decide(NOW - 0.050, NOW)        # 2 ms measured, the period is 1 ms
    assert decision.action == admission.REJECT and not decision.candidates[0].response_ok


def test_the_score_methods_run_every_image_and_keep_no_log(files):
    cfg, profile, model = files
    gate = FrameGate('M2', cfg, profile, model)
    decision = gate.decide(NOW - 0.050, NOW)        # no status: these methods do not look at it
    assert (decision.action, decision.mode) == (admission.RUN, 1)
    assert gate.rows == [] and sum(gate.counts.values()) == 0


def test_the_rule_methods_decide_from_the_newest_status(files):
    cfg, profile, model = files
    gate = FrameGate('FULL', cfg, profile, model)
    assert gate.decide(NOW - 0.050, NOW).reason == 'no_status'
    gate.on_status(status_of(gate))
    assert gate.decide(NOW - 0.050, NOW).action == admission.RUN
    gate.on_status(status_of(gate, method='A3'))    # the control application runs another method
    assert gate.decide(NOW - 0.050, NOW).reason == 'method_mismatch'
    capture = NOW - 0.050
    gate.on_status(status_of(gate, result_in_use=True, result_capture_s=capture - 0.030,
                             result_valid_until_s=capture + 0.220))
    assert gate.decide(capture, NOW).action == admission.SKIP
    assert gate.counts == {admission.RUN: 1, admission.SKIP: 1, admission.REJECT: 2}


def test_every_decision_leaves_a_row_in_the_log(files, tmp_path):
    cfg, profile, model = files
    gate = FrameGate('FULL', cfg, profile, model)
    gate.decide(NOW - 0.050, NOW)                                   # no status
    gate.on_status(status_of(gate))
    gate.decide(NOW - 0.050, NOW)                                   # run
    capture = NOW - 0.050
    gate.on_status(status_of(gate, result_in_use=True, result_capture_s=capture - 0.030,
                             result_valid_until_s=capture + 0.220))
    gate.decide(capture, NOW)                                       # skip
    log = tmp_path / 'decisions.csv'
    assert gate.save_log(str(log)) == 3
    with open(log, newline='', encoding='utf-8') as log_file:
        reader = csv.DictReader(log_file)
        assert tuple(reader.fieldnames) == LOG_COLUMNS
        rejected, ran, skipped = list(reader)

    assert (rejected['action'], rejected['mode'], rejected['reason']) == (
        'reject', '-1', 'no_status')
    assert rejected['status_age_ms'] == '' and rejected['m1_age_ms'] == ''

    assert (ran['action'], ran['mode'], ran['reason']) == ('run', '1', '')
    assert (ran['capture_s'], ran['release_s']) == ('999.950000', '1000.000000')
    assert (ran['status_age_ms'], ran['tolerance_mm']) == ('1.000', '10.000')
    # the 320 model: used 80 ms + one period after the release, 131 ms after the capture
    assert (ran['m1_first_use_ms'], ran['m1_age_ms'], ran['m1_bound_mm']) == (
        '81.000', '131.000', '6.0000')
    assert (ran['m1_in_time'], ran['m1_within_tolerance'], ran['m1_response_ok']) == (
        '1', '1', '1')
    assert float(ran['m1_cost']) == pytest.approx(0.006 ** 2, rel=1e-5)
    assert ran['m0_age_ms'] == '' and ran['skip_age_ms'] == ''      # not candidates
    # the capture on the clock of the control tasks, for the join with their log
    offset = time.clock_gettime(time.CLOCK_MONOTONIC) - time.time()
    assert abs(float(ran['capture_task_s']) - (999.95 + offset)) < 0.5

    assert (skipped['action'], skipped['mode']) == ('skip', '-1')
    # the next image could replace the result 33 + 50 + 80 + 1 ms after this capture
    assert (skipped['skip_age_ms'], skipped['skip_bound_mm']) == ('194.000', '6.0000')
    assert (skipped['skip_in_time'], skipped['skip_within_tolerance']) == ('1', '1')
    assert skipped['m1_age_ms'] == '131.000'        # the run candidate is logged too


def two_models(tmp_path, sizes=(416, 320)):
    paths = []
    for k, size in enumerate(sizes):
        d = tmp_path / f'model{k}'
        d.mkdir()
        (d / 'decode_meta.json').write_text(json.dumps({'input_size': size}), encoding='utf-8')
        paths.append(str(d / 'net.xmodel'))
    return paths


def test_two_models_two_modes(files, tmp_path):
    cfg, profile, _ = files
    m416, m320 = two_models(tmp_path)
    gate = FrameGate('FULL', cfg, profile, m416, '', m320)
    assert gate.modes == (0, 1)
    gate.on_status(status_of(gate))
    # with both, the rule can take the faster one (the profile: 80 ms against 100 ms; the cost of the error is
    # the bound at that age): the mode it chose is one of the two and is in the log
    decision = gate.decide(NOW - 0.002, NOW)
    assert decision.action == admission.RUN and decision.mode in (0, 1)
    assert gate.rows[-1]['mode'] == decision.mode
    assert {'m0_age_ms', 'm1_age_ms'} <= {k for k, v in gate.rows[-1].items() if v != ''}
    # M2 and A2 choose by the score, among both
    assert FrameGate('M2', cfg, profile, m416, '', m320).decide(NOW, NOW).mode in (0, 1)


def test_two_models_of_one_mode_are_refused(files, tmp_path):
    cfg, profile, _ = files
    a, b = two_models(tmp_path, sizes=(320, 320))
    with pytest.raises(GateError):
        FrameGate('FULL', cfg, profile, a, '', b)


# ---- dynamic L_m (2026-11-27) ------------------------------------------------

def test_the_gate_measures_the_latency_of_its_runs_from_the_status(files):
    cfg, profile, model = files
    gate = FrameGate('FULL', cfg, profile, model)
    gate.on_status(status_of(gate))
    capture_s = NOW - 0.040
    assert gate.decide(capture_s, NOW).action == admission.RUN
    # the status first names that result 70 ms later: 70 - 1 (one period) = 69 ms
    named = status_of(gate, stamp_s=NOW + 0.070, result_n=1, result_in_use=True,
                      result_capture_s=capture_s + 0.0005, result_mode=1)
    gate.on_status(named)
    gate.on_status(replace(named, stamp_s=NOW + 0.090))      # the same result again: no sample
    assert list(gate.latency.samples[1]) == pytest.approx([0.069])
    assert gate.latencies()[1] == (pytest.approx(0.080), 1)   # 1 < 20 samples: the profile
    # a result of no run of this gate is counted, not taken
    gate.on_status(replace(named, result_n=2, result_capture_s=NOW - 5.0))
    assert gate.unmatched_results == 1 and gate.latency.taken[1] == 1


def test_with_dynamic_latency_the_rule_uses_what_was_measured(files, tmp_path):
    cfg, profile, model = files
    fixed = FrameGate('FULL', cfg, profile, model)
    dynamic = FrameGate('FULL', cfg, profile, model, dynamic_latency=True)
    for gate in (fixed, dynamic):
        for n in range(1, 21):                    # 20 results of 320, each 50 ms
            release_s = NOW - 10.0 + 0.1 * n
            gate._pending[release_s - 0.030] = (release_s, 1)
            gate.on_status(status_of(gate, stamp_s=release_s + 0.051, result_n=n,
                                     result_capture_s=release_s - 0.030))
        gate.on_status(status_of(gate))
    # an image 180 ms old: 180 + 80 (profile) + 1 > 250 rejected; 180 + 50 + 1 in time
    assert fixed.decide(NOW - 0.180, NOW).action == admission.REJECT
    assert dynamic.decide(NOW - 0.180, NOW).action == admission.RUN
    path = tmp_path / 'decisions.csv'
    dynamic.save_log(str(path))
    row = list(csv.DictReader(open(path, encoding='utf-8')))[-1]
    assert (row['m1_lm_ms'], row['m1_lm_samples'], row['lm_used']) == (
        '50.000', '20', 'dynamic')
    fixed.save_log(str(path))
    row = list(csv.DictReader(open(path, encoding='utf-8')))[-1]
    assert (row['m1_lm_ms'], row['lm_used']) == ('50.000', 'profile')   # measured, not used


def test_dynamic_latency_goes_with_the_rule_only(files):
    cfg, profile, model = files
    for method in ('M2', 'A2'):
        with pytest.raises(GateError, match='dynamic_latency'):
            FrameGate(method, cfg, profile, model, dynamic_latency=True)
    assert FrameGate('A3', cfg, profile, model, dynamic_latency=True).dynamic_latency


def test_a_run_whose_result_never_came_is_forgotten(files):
    cfg, profile, model = files
    gate = FrameGate('FULL', cfg, profile, model)
    gate.on_status(status_of(gate))
    gate.decide(NOW - 0.040, NOW)
    assert len(gate._pending) == 1
    gate.on_status(status_of(gate, stamp_s=NOW + 2.5))
    assert gate._pending == {}
