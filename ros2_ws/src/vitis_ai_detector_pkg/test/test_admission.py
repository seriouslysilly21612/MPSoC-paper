"""admission: the rule that decides whether an image gets an inference (eq. (8), Alg. 2).

Guards each line of the rule on its own: a regression here does not crash, it
makes the detector run inferences whose results the control task then throws
away, or skip images it needed. All times are seconds; NOW is the release of
the decision. Pure stdlib + PyYAML, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_admission.py
"""
import os
from dataclasses import replace

import pytest

from vitis_ai_detector_pkg.admission import (
    Candidate, choose_by_score, decide, load_profile, load_response_table, ModeProfile, Profile,
    ProfileError, REJECT, RUN, SKIP, Status)
from vitis_ai_detector_pkg.contract import Contract

NOW = 1000.0
MODE_416, MODE_320 = 0, 1

# [CONTRACT] of INDY7.cfg: a target at rest, so the bound does not grow with the age
CONTRACT = Contract(tolerance_m=0.010, age_cap_s=0.250, e0_m=0.001, gain=1.0,
                    target_speed_mps=0.0, target_accel_mps2=0.0, calibration_error_m=0.002,
                    mode_error_m=(0.002, 0.003))
BOUND_416, BOUND_320 = 0.005, 0.006      # e0 + gain (mode error + calibration error)
MOVING = replace(CONTRACT, target_speed_mps=0.020)

PROFILE = Profile(
    frame_period_s=0.033, acquisition_s=0.050, status_max_age_s=0.100, reference_speed_mps=0.2,
    weight_error=1.0, weight_energy=0.0, weight_ddr=0.0, response_margin_s=0.0,
    modes={MODE_416: ModeProfile('416', 0.100, 2.0, 0.0),
           MODE_320: ModeProfile('320', 0.080, 1.0, 0.0)})
BOTH = (MODE_416, MODE_320)


def status(**changes):
    """Return the status of a control task that has no camera result in use."""
    plain = Status(stamp_s=NOW - 0.001, tolerance_m=0.010, control_period_s=0.001,
                   contract_crc=CONTRACT.crc(), method='FULL', result_in_use=False,
                   result_capture_s=0.0, result_valid_until_s=0.0, result_mode=MODE_416)
    return replace(plain, **changes)


def in_use(capture_s, mode=MODE_416):
    """Return the status of a control task that uses a result captured at capture_s."""
    return status(result_in_use=True, result_capture_s=capture_s,
                  result_valid_until_s=capture_s + CONTRACT.age_cap_s, result_mode=mode)


def run(capture_s=NOW - 0.050, state=None, contract=CONTRACT, profile=PROFILE, modes=BOTH,
        response_table=None):
    """Decide at NOW, with a control task that runs with the same contract."""
    state = replace(status() if state is None else state, contract_crc=contract.crc())
    return decide(capture_s, NOW, state, contract, profile, modes, 'FULL', response_table)


def close(a, b):
    return abs(a - b) < 1e-9


# ---- nothing to decide with ------------------------------------------------

@pytest.mark.parametrize('state, reason', [
    (None, 'no_status'),
    (status(stamp_s=NOW - 0.1001), 'status_stale'),
    (status(contract_crc=CONTRACT.crc() ^ 1), 'contract_mismatch'),
    (status(method='A2'), 'method_mismatch'),
    (status(tolerance_m=0.0), 'bad_status'),
    (status(tolerance_m=float('nan')), 'bad_status'),
    (status(control_period_s=0.0), 'bad_status'),
])
def test_without_a_status_to_trust_the_image_is_rejected(state, reason):
    decision = decide(NOW - 0.050, NOW, state, CONTRACT, PROFILE, BOTH, 'FULL')
    assert (decision.action, decision.mode, decision.reason) == (REJECT, -1, reason)
    assert decision.candidates == ()


def test_a_status_at_its_oldest_is_still_trusted():
    assert run(state=status(stamp_s=NOW - 0.0999)).action == RUN


# ---- a mode that can run -----------------------------------------------------

def test_the_numbers_of_a_candidate():
    decision = run(modes=(MODE_416,))
    assert (decision.action, decision.mode, decision.reason) == (RUN, MODE_416, '')
    (candidate,) = decision.candidates
    assert candidate.action == RUN and candidate.mode == MODE_416
    assert close(candidate.first_use_s, NOW + 0.100 + 0.001)   # latency, then one period
    assert close(candidate.age_s, 0.050 + 0.100 + 0.001)       # since the capture
    assert close(candidate.error_bound_m, BOUND_416)
    assert candidate.in_time and candidate.within_tolerance and candidate.response_ok
    assert close(candidate.cost, BOUND_416 ** 2)


def test_of_two_modes_the_cheaper_one_runs():
    assert run().mode == MODE_416                    # error only: the smaller bound
    energy_counts = replace(PROFILE, weight_energy=2e-5)
    assert run(profile=energy_counts).mode == MODE_320   # 36e-6 + 2e-5 < 25e-6 + 4e-5


# ---- check 1: time ---------------------------------------------------------------

def test_a_result_that_would_arrive_after_its_deadline_is_not_started():
    # 416 would be used 0.101 s from now, 320 after 0.081 s; the age cap is 0.250 s
    fits_416 = run(capture_s=NOW - 0.1489)
    assert fits_416.mode == MODE_416
    only_320 = run(capture_s=NOW - 0.1491)
    assert only_320.mode == MODE_320
    assert [c.in_time for c in only_320.candidates] == [False, True]
    neither = run(capture_s=NOW - 0.1691)
    assert (neither.action, neither.reason) == (REJECT, 'no_candidate_passes')
    assert [c.in_time for c in neither.candidates] == [False, False]


# ---- check 2: error --------------------------------------------------------------

def test_a_result_whose_bound_is_above_the_tolerance_is_not_started():
    assert run(state=status(tolerance_m=0.0055)).mode == MODE_416
    assert [c.within_tolerance for c in run(state=status(tolerance_m=0.0055)).candidates] == [
        True, False]
    assert run(state=status(tolerance_m=0.0049)).action == REJECT


def test_the_bound_is_taken_at_the_age_of_the_first_use():
    # a target that may move at 20 mm/s: 20 mm/s x 0.151 s = 3.02 mm on top of 5 mm
    fresh = run(contract=MOVING, modes=(MODE_416,))
    assert close(fresh.candidates[0].error_bound_m, BOUND_416 + 0.020 * 0.151)
    assert fresh.action == RUN
    # the same image 0.1 s later: 5.02 mm on top, above the tolerance of 10 mm
    late = decide(NOW - 0.050, NOW + 0.1,
                  status(stamp_s=NOW + 0.099, contract_crc=MOVING.crc()), MOVING, PROFILE,
                  (MODE_416,), 'FULL')
    assert close(late.candidates[0].error_bound_m, BOUND_416 + 0.020 * 0.251)
    assert late.action == REJECT


# ---- check 3: the response of the control task -------------------------------------

def test_a_mode_whose_measured_response_is_too_long_is_not_started():
    table = {MODE_416: 0.00096, MODE_320: 0.00090}     # the period is 1 ms
    assert run(response_table=table).mode == MODE_416
    with_margin = replace(PROFILE, response_margin_s=0.00005)
    assert run(profile=with_margin, response_table=table).mode == MODE_320
    assert run(profile=replace(PROFILE, response_margin_s=0.00011),
               response_table=table).action == REJECT


def test_a_mode_without_a_measured_response_is_not_started():
    decision = run(response_table={MODE_320: 0.0005})
    assert decision.mode == MODE_320
    assert [c.response_ok for c in decision.candidates] == [False, True]


# ---- skip ------------------------------------------------------------------------

# the next image could replace the result in use 0.033 + 0.050 + 0.080 + 0.001 s after
# the capture of this one
NEXT_USE = 0.164


def test_skip_wins_while_the_result_in_use_carries_to_the_next_image():
    capture = NOW - 0.050
    decision = run(capture_s=capture, state=in_use(capture - 0.030))
    assert (decision.action, decision.mode, decision.reason) == (SKIP, -1, '')
    skip = decision.candidates[0]
    assert skip.action == SKIP and skip.mode == MODE_416
    assert close(skip.first_use_s, capture + NEXT_USE)
    assert close(skip.age_s, NEXT_USE + 0.030)
    assert close(skip.cost, BOUND_416 ** 2)            # no dearer than to run 416 again


def test_no_skip_without_a_result_in_use():
    decision = run()
    assert decision.action == RUN
    assert all(candidate.action == RUN for candidate in decision.candidates)


def test_no_skip_when_the_result_in_use_expires_before_the_next_image():
    capture = NOW - 0.050
    just_enough = in_use(capture + NEXT_USE - 0.250 + 0.0001)
    assert run(capture_s=capture, state=just_enough).action == SKIP
    too_old = in_use(capture + NEXT_USE - 0.250 - 0.0001)
    decision = run(capture_s=capture, state=too_old)
    assert decision.action == RUN
    assert decision.candidates[0].action == SKIP and not decision.candidates[0].in_time


def test_no_skip_when_the_bound_of_the_result_in_use_grows_above_the_tolerance():
    capture = NOW - 0.050
    # a target that may move at 20 mm/s. At the next use the result in use is
    # 0.194 s old: 5 + 3.88 mm, within the tolerance of 10 mm
    young = run(capture_s=capture, state=in_use(capture - 0.030), contract=MOVING)
    assert young.candidates[0].action == SKIP and young.candidates[0].feasible
    # 0.264 s old: 5 + 5.28 mm
    aged = in_use(capture - 0.100)
    aged = replace(aged, result_valid_until_s=NOW + 1.0)      # only the bound speaks against it
    decision = run(capture_s=capture, state=aged, contract=MOVING)
    assert decision.action == RUN
    assert decision.candidates[0].in_time and not decision.candidates[0].within_tolerance


def test_a_newer_image_beats_the_skip_when_the_target_may_move():
    capture = NOW - 0.050
    # skip: the result in use at 0.194 s, 8.88 mm; run 416: a new one at 0.151 s, 8.02 mm
    state = in_use(capture - 0.030)
    assert run(capture_s=capture, state=state, contract=MOVING).mode == MODE_416
    energy_counts = replace(PROFILE, weight_energy=1e-5)     # 78.9e-6 < 64.3e-6 + 2e-5
    assert run(capture_s=capture, state=state, contract=MOVING,
               profile=energy_counts).action == SKIP


def test_a_better_measurement_beats_the_skip_and_a_dear_inference_does_not():
    capture = NOW - 0.050
    coarse_in_use = in_use(capture - 0.030, mode=MODE_320)
    assert run(capture_s=capture, state=coarse_in_use).mode == MODE_416    # 25e-6 < 36e-6
    energy_counts = replace(PROFILE, weight_energy=1e-5)
    assert run(capture_s=capture, state=coarse_in_use,
               profile=energy_counts).action == SKIP                       # 36e-6 < 45e-6


def test_skip_does_not_need_a_mode_that_passes():
    capture = NOW - 0.050
    decision = run(capture_s=capture, state=in_use(capture - 0.030),
                   response_table={MODE_416: 0.002, MODE_320: 0.002})
    assert decision.action == SKIP
    assert [c.feasible for c in decision.candidates] == [True, False, False]


def test_feasible_needs_all_three_checks():
    passing = Candidate(RUN, MODE_416, 0.0, 0.0, 0.0, True, True, True, 0.0)
    assert passing.feasible
    for check in ('in_time', 'within_tolerance', 'response_ok'):
        assert not replace(passing, **{check: False}).feasible


# ---- the score of M2 and A2 ---------------------------------------------------------

def test_score_is_the_error_plus_the_way_of_the_tool_during_the_latency():
    # 416: 2 mm + 0.2 m/s x 0.100 s = 22 mm; 320: 3 mm + 0.2 m/s x 0.080 s = 19 mm
    assert choose_by_score(CONTRACT, PROFILE, BOTH, {}) == MODE_320
    # 320 measured slower than its profile: 3 mm + 0.2 m/s x 0.120 s = 27 mm
    assert choose_by_score(CONTRACT, PROFILE, BOTH, {MODE_320: 0.120}) == MODE_416
    # a slow tool: 416: 2 mm + 1.0 mm; 320: 3 mm + 0.8 mm
    slow_tool = replace(PROFILE, reference_speed_mps=0.01)
    assert choose_by_score(CONTRACT, slow_tool, BOTH, {}) == MODE_416
    assert choose_by_score(CONTRACT, PROFILE, (MODE_416,), {}) == MODE_416


# ---- the files -------------------------------------------------------------------

PROFILE_YAML = """\
frame_period_ms: 33.0
acquisition_ms: 50.0
status_max_age_ms: 100.0
reference_speed_mps: 0.2
response_margin_us: 50.0
weights: {error: 1.0, energy: 0.5, ddr: 0.25}
modes:
  0: {name: "416", release_to_publication_ms: 100.0, energy_j: 2.0, ddr_bytes: 3.0}
  1: {name: "320", release_to_publication_ms: 80.0, energy_j: 1.0, ddr_bytes: 0.0}
"""


def _write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text, encoding='utf-8')
    return str(path)


def test_the_profile_file_in_seconds(tmp_path):
    profile = load_profile(_write(tmp_path, 'p.yaml', PROFILE_YAML))
    assert profile == Profile(
        frame_period_s=0.033, acquisition_s=0.050, status_max_age_s=0.100,
        reference_speed_mps=0.2, weight_error=1.0, weight_energy=0.5, weight_ddr=0.25,
        response_margin_s=1e-6 * 50.0,
        modes={MODE_416: ModeProfile('416', 0.100, 2.0, 3.0),
               MODE_320: ModeProfile('320', 0.080, 1.0, 0.0)})


@pytest.mark.parametrize('broken, what', [
    (PROFILE_YAML.replace('frame_period_ms: 33.0', 'frame_period_ms: 0'), 'frame_period_ms'),
    (PROFILE_YAML.replace('acquisition_ms: 50.0\n', ''), 'acquisition_ms'),
    (PROFILE_YAML.replace('energy: 0.5', 'energy: -1'), 'weights.energy'),
    (PROFILE_YAML.replace('error: 1.0', 'error: .nan'), 'weights.error'),
    (PROFILE_YAML.replace('release_to_publication_ms: 80.0', 'release_to_publication_ms: fast'),
     'modes.1.release_to_publication_ms'),
    (PROFILE_YAML.replace('status_max_age_ms: 100.0', 'status_max_age_ms: true'),
     'status_max_age_ms'),
    (PROFILE_YAML.replace('  1: {', '  2: {'), 'mode 2'),
    (PROFILE_YAML.replace('  1: {', '  yes: {'), 'mode True'),
    (PROFILE_YAML.split('modes:')[0] + 'modes: {}\n', 'no modes'),
    (PROFILE_YAML.replace('weights: {error: 1.0, energy: 0.5, ddr: 0.25}\n', ''), 'no weights'),
    ('modes: [', 'cannot read'),
])
def test_a_profile_that_cannot_be_used_is_refused(tmp_path, broken, what):
    with pytest.raises(ProfileError, match=what):
        load_profile(_write(tmp_path, 'p.yaml', broken))


def test_a_missing_profile_is_refused(tmp_path):
    with pytest.raises(ProfileError, match='cannot read'):
        load_profile(str(tmp_path / 'none.yaml'))


def test_the_profile_that_ships_with_the_pipeline():
    here = os.path.dirname(os.path.abspath(__file__))
    shipped = os.path.join(here, '..', '..', 'system_bringup_pkg', 'config', 'mode_profile.yaml')
    profile = load_profile(shipped)
    assert sorted(profile.modes) == [MODE_416, MODE_320]


def test_the_response_table_keeps_the_worst_row_of_a_mode(tmp_path):
    table = load_response_table(_write(
        tmp_path, 't.csv', 'mode,level,response_us\n0,0,300\n0,100,640\n0,50,410\n1,100,520\n'))
    assert set(table) == {MODE_416, MODE_320}
    assert close(table[MODE_416], 640e-6) and close(table[MODE_320], 520e-6)


@pytest.mark.parametrize('broken', [
    'mode,level,response_us\n',                 # no rows
    'mode,level\n0,0\n',                        # no response
    'mode,level,response_us\n0,0,-5\n',         # not a response
    'mode,level,response_us\nfast,0,300\n',     # not a mode
])
def test_a_response_table_that_cannot_be_used_is_refused(tmp_path, broken):
    with pytest.raises(ProfileError):
        load_response_table(_write(tmp_path, 't.csv', broken))
    with pytest.raises(ProfileError, match='cannot read'):
        load_response_table(str(tmp_path / 'none.csv'))
