"""contract: the constants of [CONTRACT] and eq. (7) on the camera side.

Guards the one thing that makes the two sides one contract: the camera side
must read the same constants and compute the same error bound and the same
check value as the control application. The five bounds and the two check
values below are those of the test of the control application
(goal-to-torque_MPSoC/tools/obs_status_check.cpp, parts 4 and 5): change one
side and its own test fails until the other side follows.
Pure stdlib, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_contract.py
"""
import os

import pytest

from vitis_ai_detector_pkg.contract import Contract, ContractError, read_contract

CFG = """\
; a comment line
[SYSTEM]
DELTA_MM=99            ; another section: not ours

[CONTRACT]
; the constants of INDY7.cfg
DELTA_MM=10            ; tolerance
AGE_CAP_MS=250         # a comment after a hash
e0_mm = 1
G=1# a comment right after the number
EPS_V_MPS=0
A_MAX_MPS2=0
EPS_CAL_MM=2
EPS_M_MM=2 3
EPS_M_MM=7 8           ; given twice: the first one counts
TAG_OFFSET_M=-0.085 0 0

[RT_TASKS]
G=5

[CONTRACT]
DELTA_MM=55            ; a second section of the name: ignored
"""

OTHER = Contract(tolerance_m=12.5 * 1e-3, age_cap_s=180 * 1e-3, e0_m=0.7 * 1e-3, gain=1.5,
                 target_speed_mps=0.05, target_accel_mps2=0.3, calibration_error_m=1.2 * 1e-3,
                 mode_error_m=(2.5 * 1e-3, 4 * 1e-3))


def _write(tmp_path, text):
    path = tmp_path / 'test.cfg'
    path.write_text(text, encoding='utf-8')
    return str(path)


@pytest.fixture
def contract(tmp_path):
    return read_contract(_write(tmp_path, CFG))


def test_reads_the_section_as_the_control_application_does(contract):
    assert contract == Contract(
        tolerance_m=10 * 1e-3, age_cap_s=250 * 1e-3, e0_m=1 * 1e-3, gain=1.0,
        target_speed_mps=0.0, target_accel_mps2=0.0, calibration_error_m=2 * 1e-3,
        mode_error_m=(2 * 1e-3, 3 * 1e-3))


def test_reads_other_constants_and_a_section_name_in_any_case(tmp_path):
    text = ('[Contract]\nDELTA_MM=12.5\nAGE_CAP_MS=180\nE0_MM=0.7\nG=1.5\nEPS_V_MPS=0.05\n'
            'A_MAX_MPS2=0.3\nEPS_CAL_MM=1.2\nEPS_M_MM=2.5 4\n')
    assert read_contract(_write(tmp_path, text)) == OTHER


def test_check_value_is_that_of_the_control_application(contract):
    assert contract.crc() == 0x2E7B0347
    assert OTHER.crc() == 0x21A36B83


def test_every_constant_changes_the_check_value(contract):
    for field in ('tolerance_m', 'age_cap_s', 'e0_m', 'gain', 'target_speed_mps',
                  'target_accel_mps2', 'calibration_error_m'):
        changed = Contract(**{**contract.__dict__, field: getattr(contract, field) + 1e-6})
        assert changed.crc() != contract.crc(), field
    for mode in (0, 1):
        errors = list(contract.mode_error_m)
        errors[mode] += 1e-6
        changed = Contract(**{**contract.__dict__, 'mode_error_m': tuple(errors)})
        assert changed.crc() != contract.crc(), mode


@pytest.mark.parametrize('use_other, mode, age_s, bound', [
    (False, 0, 0.18, 0.0050000000949949027),
    (False, 1, 0.25, 0.0060000000260770322),
    (True, 0, 0.2, 0.030249999916180968),
    (True, 1, 0.0, 0.0085000002849847066),
    (True, 1, 0.137, 0.022998025284984711),
])
def test_error_bound_is_that_of_the_control_task(contract, use_other, mode, age_s, bound):
    side = OTHER if use_other else contract
    assert abs(side.error_bound(mode, age_s) - bound) < 1e-15


@pytest.mark.parametrize('broken, what', [
    (CFG.replace('G=1# a comment right after the number\n', ''), 'G is not given'),
    # only the first section of the name counts, as in the control application
    (CFG.replace('G=1# a comment right after the number\n', '') + 'G=1\n', 'G is not given'),
    (CFG.replace('EPS_M_MM=2 3', 'EPS_M_MM=2'), 'needs 2 number'),
    (CFG.replace('E0_MM=1', 'E0_MM=-1').replace('e0_mm = 1', 'E0_MM=-1'), 'not negative'),
    (CFG.replace('EPS_CAL_MM=2', 'EPS_CAL_MM=nan'), 'finite'),
    (CFG.replace('EPS_V_MPS=0', 'EPS_V_MPS=fast'), 'not a number'),
    (CFG.replace('DELTA_MM=10 ', 'DELTA_MM=0 '), 'above 0'),
    (CFG.replace('AGE_CAP_MS=250', 'AGE_CAP_MS=0'), 'above 0'),
    (CFG.replace('[CONTRACT]', '[CONTRACTS]'), 'no section'),
])
def test_a_section_that_cannot_be_used_is_refused(tmp_path, broken, what):
    with pytest.raises(ContractError, match=what):
        read_contract(_write(tmp_path, broken))


def test_a_missing_file_is_refused(tmp_path):
    with pytest.raises(ContractError, match='cannot read'):
        read_contract(str(tmp_path / 'none.cfg'))


def test_the_configuration_of_the_control_application():
    """INDY7.cfg itself, where this test can find it: in the repository, or on the board."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', '..', '..', '..', 'goal-to-torque_MPSoC', 'indy7', 'INDY7.cfg'),
        os.path.expanduser('~/goal-to-torque_MPSoC/indy7/INDY7.cfg'),
    ]
    found = [path for path in candidates if os.path.isfile(path)]
    if not found:
        pytest.skip('INDY7.cfg is not here')
    contract = read_contract(found[0])
    assert contract.age_cap_s < 0.253   # below the record of the joint angles (obs_receiver.h)
    assert contract.error_bound(0, 0.0) <= contract.tolerance_m
