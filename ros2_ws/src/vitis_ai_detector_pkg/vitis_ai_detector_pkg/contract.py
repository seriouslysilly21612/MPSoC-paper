"""contract.py: the contract constants of the control application, on the camera side.

The control task and the camera side must compute with the same constants
(paper, eq. (2), (7)). They stand in one place, the section [CONTRACT] of the
configuration of the control application (INDY7.cfg), and this module reads
them from that file:

  read_contract(path)   the constants, as the control application reads them
  Contract.error_bound  eq. (7): the bound of the task error that a result of
                        a mode causes at an age (obs_error_bound of
                        obs_receiver.h)
  Contract.crc          the check value that the control task writes into its
                        status (ObsContractCrc of ObsContract.h); the two
                        differ as soon as one constant differs

Pure stdlib, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_contract.py
"""
import math
import struct
import zlib
from dataclasses import dataclass

CONTRACT_LAYOUT = 1   # OBS_CONTRACT_LAYOUT of ObsContract.h
SECTION = 'CONTRACT'


class ContractError(Exception):
    """The section [CONTRACT] cannot be used."""


def _as_float32(value):
    """Round as the mailbox does: it carries the error of a mode as a 32-bit float."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


@dataclass(frozen=True)
class Contract:
    """The constants that both sides compute with, in SI units."""

    tolerance_m: float          # DELTA_MM: tolerance of the task error
    age_cap_s: float            # AGE_CAP_MS: no result is used later than this after its capture
    e0_m: float                 # E0_MM: the alignment error with an exact goal
    gain: float                 # G: gain of the measurement error on the task error
    target_speed_mps: float     # EPS_V_MPS: bound of the speed of the target
    target_accel_mps2: float    # A_MAX_MPS2: bound of the acceleration of the target
    calibration_error_m: float  # EPS_CAL_MM: error of the calibration and the clocks
    mode_error_m: tuple         # EPS_M_MM: measurement error of the modes 0 (416) and 1 (320)

    def error_bound(self, mode, age_s):
        """Eq. (7): the bound of the task error [m] of a result of this mode at this age [s]."""
        sigma = _as_float32(self.mode_error_m[mode])
        return self.e0_m + self.gain * (
            sigma + self.target_speed_mps * age_s
            + 0.5 * self.target_accel_mps2 * age_s * age_s + self.calibration_error_m)

    def crc(self):
        """Return the check value of the constants, as the control task computes it."""
        constants = (self.tolerance_m, self.age_cap_s, self.e0_m, self.gain,
                     self.target_speed_mps, self.target_accel_mps2, self.calibration_error_m,
                     self.mode_error_m[0], self.mode_error_m[1])
        packed = struct.pack('<I', CONTRACT_LAYOUT) + b''.join(
            struct.pack('<d', constant) for constant in constants)
        return zlib.crc32(packed)


def _read_section(path, section):
    """Return {KEY: value text} of one section, by the rules of the control application.

    A comment starts at ';' or '#'. A key that is given twice keeps its first
    value. Of two sections with the same name the first one counts.
    """
    entries = {}
    in_section = False
    seen = False
    with open(path, encoding='utf-8') as cfg:
        for raw in cfg:
            line = raw
            for mark in ';#':
                line = line.split(mark, 1)[0]
            line = line.strip()
            if not line:
                continue
            if line.startswith('['):
                name = line[1:].split(']', 1)[0].strip().upper()
                in_section = (name == section) and not seen
                seen = seen or in_section
                continue
            if in_section and '=' in line:
                key, value = line.split('=', 1)
                entries.setdefault(key.strip().upper(), value.strip())
    if not seen:
        raise ContractError(f'{path} has no section [{section}]')
    return entries


def _numbers(entries, key, count, scale, path):
    """Return `count` numbers of a key, each times `scale`; they must be finite and not negative."""
    if key not in entries or not entries[key]:
        raise ContractError(f'{path}: [{SECTION}] {key} is not given')
    words = entries[key].split()
    try:
        values = [float(word) * scale for word in words[:count]]
    except ValueError:
        raise ContractError(f'{path}: [{SECTION}] {key}={entries[key]} is not a number') from None
    if len(values) < count:
        raise ContractError(f'{path}: [{SECTION}] {key} needs {count} number(s)')
    for value in values:
        if not math.isfinite(value) or value < 0.0:
            raise ContractError(f'{path}: [{SECTION}] {key} must be finite and not negative')
    return values


def read_contract(path):
    """Read [CONTRACT] of the configuration of the control application.

    Every constant must be given: the control application has defaults for a
    missing one, and a second copy of them here could silently differ.
    """
    try:
        entries = _read_section(path, SECTION)
    except OSError as error:
        raise ContractError(f'cannot read {path}: {error}') from None

    def one(key, scale=1.0):
        return _numbers(entries, key, 1, scale, path)[0]

    contract = Contract(
        tolerance_m=one('DELTA_MM', 1e-3),
        age_cap_s=one('AGE_CAP_MS', 1e-3),
        e0_m=one('E0_MM', 1e-3),
        gain=one('G'),
        target_speed_mps=one('EPS_V_MPS'),
        target_accel_mps2=one('A_MAX_MPS2'),
        calibration_error_m=one('EPS_CAL_MM', 1e-3),
        mode_error_m=tuple(_numbers(entries, 'EPS_M_MM', 2, 1e-3, path)))
    if not contract.tolerance_m > 0.0 or not contract.age_cap_s > 0.0:
        raise ContractError(f'{path}: [{SECTION}] DELTA_MM and AGE_CAP_MS must be above 0')
    return contract
