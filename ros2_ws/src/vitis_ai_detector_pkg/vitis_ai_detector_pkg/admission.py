"""admission.py: is this image worth an inference? (paper, eq. (8), Alg. 2; D-24, D-28).

Before the detector starts an inference on an image, the camera side asks
whether its result can still be of use when it reaches the control task. The
control task answers every question with what it wrote into its status; it is
never asked to wait, and it checks every result again when it arrives.

  decide()           the admission rule (methods A3 and FULL). For every mode
                     that can run now:
                       1. time       the result reaches the control task
                                     before its deadline (capture + the age cap)
                       2. error      its error bound at that age (eq. (7)) is
                                     within the tolerance of the control task
                       3. response   the control task keeps its deadline while
                                     the inference runs (a table of measured
                                     responses; without a table the check is off)
                     SKIP is a candidate while the result in use stays good
                     until the next image could replace it. The cheapest
                     candidate wins; with none, the image is rejected.
  choose_by_score()  the mode of the methods M2 and A2: measurement error plus
                     the way of the tool during the latency. No look at the
                     control task, no skip.
  DynamicLatency     L_m of every mode from the latencies its results had in
                     this run, in place of the fixed value of the profile
                     (launch dynamic_latency:=true)

Times are seconds on one clock (the node uses that of the image stamps).
Pure stdlib + PyYAML, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_admission.py
"""
import collections
import csv
import math
from dataclasses import dataclass, field

import yaml

RUN = 'run'
SKIP = 'skip'
REJECT = 'reject'

MODE_OF_INPUT_SIZE = {416: 0, 320: 1}   # the mode numbers of the mailbox (obs_mailbox.h)


class ProfileError(Exception):
    """The mode profile or the response table cannot be used."""


@dataclass(frozen=True)
class Status:
    """What the control task wrote in one period (obs_status.h), times in seconds."""

    stamp_s: float                 # release of the period that wrote it
    tolerance_m: float             # delta of that period
    control_period_s: float        # period of the control task = deadline of its response
    contract_crc: int              # check value of the constants of the control task
    method: str                    # method of the control application: 'M1' .. 'FULL'
    result_in_use: bool            # that period had a camera result to use
    result_capture_s: float        # the result used last: its capture
    result_valid_until_s: float    # its deadline
    result_mode: int               # its mode
    result_n: int = 0              # number of that result (0: none so far); a new one = new number


@dataclass(frozen=True)
class ModeProfile:
    """What is known of one inference mode before it runs."""

    name: str                           # '416', '320'
    release_to_publication_s: float     # start of the inference -> result in the mailbox
    energy_j: float                     # energy of one inference
    ddr_bytes: float                    # DDR traffic of one inference


@dataclass(frozen=True)
class Profile:
    """The numbers of the camera side that the rules compute with (mode_profile.yaml)."""

    frame_period_s: float          # time between two images of the camera
    acquisition_s: float           # capture -> the detector has the image
    status_max_age_s: float        # an older status is not trusted
    reference_speed_mps: float     # tool speed that turns latency into error (choose_by_score)
    weight_error: float            # w_e of the cost [1/m^2]
    weight_energy: float           # w_p [1/J]
    weight_ddr: float              # w_b [1/byte]
    response_margin_s: float       # margin M on a measured response
    modes: dict = field(default_factory=dict)   # mode number -> ModeProfile
    latency_window: int = 100        # dynamic L_m: the newest samples of a mode it looks at
    latency_quantile: float = 0.9    # dynamic L_m: this quantile of them
    latency_min_samples: int = 20    # dynamic L_m: fewer samples -> the value of the profile
    latency_clamp: tuple = (0.5, 1.5)   # dynamic L_m: within these multiples of the profile


@dataclass(frozen=True)
class Candidate:
    """One thing the detector could do with the image, and what speaks for or against it."""

    action: str                    # RUN or SKIP
    mode: int                      # RUN: the mode to run; SKIP: the mode of the result in use
    first_use_s: float             # RUN: first period that can use the result (t_k*);
    #                                SKIP: when the next image could replace the result in use
    age_s: float                   # age of the result at first_use_s
    error_bound_m: float           # eq. (7) at that age
    in_time: bool                  # check 1
    within_tolerance: bool         # check 2
    response_ok: bool              # check 3 (True for SKIP: nothing runs)
    cost: float                    # J

    @property
    def feasible(self):
        return self.in_time and self.within_tolerance and self.response_ok


@dataclass(frozen=True)
class Decision:
    """What to do with one image, and every candidate that was looked at."""

    action: str                    # RUN, SKIP or REJECT
    mode: int                      # RUN: the mode to run; else -1
    reason: str                    # REJECT: why; else ''
    candidates: tuple = ()


def _positive(value, name, path, allow_zero=False):
    ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    if not ok or value < 0 or (value == 0 and not allow_zero):
        limit = 'not negative' if allow_zero else 'above 0'
        raise ProfileError(f'{path}: {name} must be a number {limit}')
    return float(value)


def load_profile(path):
    """Read mode_profile.yaml. Times in the file are ms, in the Profile s."""
    try:
        with open(path, encoding='utf-8') as profile_file:
            raw = yaml.safe_load(profile_file)
    except (OSError, yaml.YAMLError) as error:
        raise ProfileError(f'cannot read {path}: {error}') from None
    if not isinstance(raw, dict) or not isinstance(raw.get('modes'), dict) or not raw['modes']:
        raise ProfileError(f'{path}: no modes')
    weights = raw.get('weights')
    if not isinstance(weights, dict):
        raise ProfileError(f'{path}: no weights')
    modes = {}
    for number, entry in raw['modes'].items():
        known = not isinstance(number, bool) and number in MODE_OF_INPUT_SIZE.values()
        if not known or not isinstance(entry, dict):
            raise ProfileError(f'{path}: mode {number!r} is not one of the mailbox (0, 1)')
        where = f'modes.{number}.'
        modes[number] = ModeProfile(
            name=str(entry.get('name', number)),
            release_to_publication_s=1e-3 * _positive(
                entry.get('release_to_publication_ms'), where + 'release_to_publication_ms', path),
            energy_j=_positive(entry.get('energy_j'), where + 'energy_j', path, allow_zero=True),
            ddr_bytes=_positive(entry.get('ddr_bytes'), where + 'ddr_bytes', path,
                                allow_zero=True))
    dynamic = raw.get('dynamic_latency', {})
    if not isinstance(dynamic, dict):
        raise ProfileError(f'{path}: dynamic_latency must be a mapping')
    window = dynamic.get('window', 100)
    min_samples = dynamic.get('min_samples', 20)
    for name, value in (('window', window), ('min_samples', min_samples)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ProfileError(f'{path}: dynamic_latency.{name} must be a whole number above 0')
    if min_samples > window:
        raise ProfileError(f'{path}: dynamic_latency.min_samples is above its window')
    quantile = _positive(dynamic.get('quantile', 0.9), 'dynamic_latency.quantile', path)
    if quantile > 1.0:
        raise ProfileError(f'{path}: dynamic_latency.quantile must be at most 1')
    clamp = dynamic.get('clamp', [0.5, 1.5])
    if not isinstance(clamp, (list, tuple)) or len(clamp) != 2:
        raise ProfileError(f'{path}: dynamic_latency.clamp must be [low, high]')
    low = _positive(clamp[0], 'dynamic_latency.clamp[0]', path)
    high = _positive(clamp[1], 'dynamic_latency.clamp[1]', path)
    if low > high:
        raise ProfileError(f'{path}: dynamic_latency.clamp: low above high')
    return Profile(
        frame_period_s=1e-3 * _positive(raw.get('frame_period_ms'), 'frame_period_ms', path),
        acquisition_s=1e-3 * _positive(raw.get('acquisition_ms'), 'acquisition_ms', path),
        status_max_age_s=1e-3 * _positive(raw.get('status_max_age_ms'), 'status_max_age_ms', path),
        reference_speed_mps=_positive(raw.get('reference_speed_mps'), 'reference_speed_mps', path,
                                      allow_zero=True),
        weight_error=_positive(weights.get('error'), 'weights.error', path, allow_zero=True),
        weight_energy=_positive(weights.get('energy'), 'weights.energy', path, allow_zero=True),
        weight_ddr=_positive(weights.get('ddr'), 'weights.ddr', path, allow_zero=True),
        response_margin_s=1e-6 * _positive(raw.get('response_margin_us'), 'response_margin_us',
                                           path, allow_zero=True),
        modes=modes,
        latency_window=window, latency_quantile=quantile, latency_min_samples=min_samples,
        latency_clamp=(low, high))


class DynamicLatency:
    """L_m of every mode from the latencies its results had in this run (2026-11-26).

    A sample is taken when the status of the control task first names a new
    result: the time of that status minus the release of the inference minus
    one control period, i.e. release -> the result reached the control task,
    as release_to_publication_s of the profile. The status comes every 20 ms
    from the bridge, so a sample is up to about 20 ms late (on the safe side).

    value(mode): the quantile of the newest window samples of the mode, held
    within the clamp around the value of the profile; the value of the profile
    while the mode has fewer than min_samples. The rule only spends work with
    it: a result it lets through too late is refused by the control task.
    """

    def __init__(self, profile):
        self.profile = profile
        self.samples = {mode: collections.deque(maxlen=profile.latency_window)
                        for mode in profile.modes}
        self.taken = dict.fromkeys(profile.modes, 0)

    def add(self, mode, latency_s):
        if mode in self.samples and math.isfinite(latency_s) and latency_s > 0.0:
            self.samples[mode].append(latency_s)
            self.taken[mode] += 1

    def value(self, mode):
        fixed_s = self.profile.modes[mode].release_to_publication_s
        samples = self.samples[mode]
        if len(samples) < self.profile.latency_min_samples:
            return fixed_s
        ordered = sorted(samples)
        rank = max(1, math.ceil(self.profile.latency_quantile * len(ordered)))   # nearest rank
        low, high = self.profile.latency_clamp
        return min(max(ordered[rank - 1], low * fixed_s), high * fixed_s)

    def values(self):
        """{mode: L_m [s]} for every mode of the profile."""
        return {mode: self.value(mode) for mode in self.profile.modes}


def latency_of(profile, mode, latency_s=None):
    """L_m [s] of a mode: from latency_s ({mode: s}) when it has the mode, else the profile."""
    if latency_s and mode in latency_s:
        return latency_s[mode]
    return profile.modes[mode].release_to_publication_s


def load_response_table(path):
    """Read the measured responses of the control task: {mode: the longest response [s]}.

    The file has a row per mode and load level: mode,level,response_us. Which
    level the board is in is not known here, so the worst row of a mode counts.
    """
    table = {}
    try:
        with open(path, newline='', encoding='utf-8') as table_file:
            for row in csv.DictReader(table_file):
                mode = int(row['mode'])
                response_s = 1e-6 * _positive(float(row['response_us']), 'response_us', path)
                table[mode] = max(table.get(mode, 0.0), response_s)
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise ProfileError(f'cannot read {path}: {error!r}') from None
    if not table:
        raise ProfileError(f'{path}: no rows')
    return table


def _run_candidate(mode, capture_s, now_s, status, contract, profile, response_table,
                   latency_s=None):
    mode_profile = profile.modes[mode]
    arrival_s = now_s + latency_of(profile, mode, latency_s)
    first_use_s = arrival_s + status.control_period_s      # one period later, to be safe
    age_s = first_use_s - capture_s
    error_bound_m = contract.error_bound(mode, age_s)
    if response_table is None:
        response_ok = True
    else:                                                  # a mode without a measured response: no
        response_ok = (mode in response_table and
                       response_table[mode] + profile.response_margin_s <= status.control_period_s)
    return Candidate(
        action=RUN, mode=mode, first_use_s=first_use_s, age_s=age_s, error_bound_m=error_bound_m,
        in_time=first_use_s <= capture_s + contract.age_cap_s,
        within_tolerance=error_bound_m <= status.tolerance_m,
        response_ok=response_ok,
        cost=(profile.weight_error * error_bound_m ** 2
              + profile.weight_energy * mode_profile.energy_j
              + profile.weight_ddr * mode_profile.ddr_bytes))


def _skip_candidate(modes, capture_s, status, contract, profile, latency_s=None):
    """Return SKIP as a candidate, or None while no result is in use.

    Skipping this image means waiting for the next one. The result in use has
    to carry the control task until the result of that image could be used.
    """
    if not status.result_in_use:
        return None
    fastest_s = min(latency_of(profile, mode, latency_s) for mode in modes)
    next_use_s = (capture_s + profile.frame_period_s + profile.acquisition_s + fastest_s
                  + status.control_period_s)
    age_s = next_use_s - status.result_capture_s
    error_bound_m = contract.error_bound(status.result_mode, age_s)
    return Candidate(
        action=SKIP, mode=status.result_mode, first_use_s=next_use_s, age_s=age_s,
        error_bound_m=error_bound_m,
        in_time=next_use_s <= status.result_valid_until_s,
        within_tolerance=error_bound_m <= status.tolerance_m,
        response_ok=True,
        cost=profile.weight_error * error_bound_m ** 2)


def decide(capture_s, now_s, status, contract, profile, modes, method, response_table=None,
           latency_s=None):
    """Decide what to do with the image captured at capture_s, now (Alg. 2).

    status: the newest Status of the control task, or None. modes: the mode
    numbers that can run now, each with an entry in the profile. method: the
    method this side runs with. latency_s: {mode: L_m [s]} in place of the
    profile's (DynamicLatency); None: the profile.
    """
    if status is None:
        return Decision(REJECT, -1, 'no_status')
    if now_s - status.stamp_s > profile.status_max_age_s:
        return Decision(REJECT, -1, 'status_stale')
    if status.contract_crc != contract.crc():
        return Decision(REJECT, -1, 'contract_mismatch')
    if status.method != method:
        return Decision(REJECT, -1, 'method_mismatch')
    if not status.tolerance_m > 0.0 or not status.control_period_s > 0.0:
        return Decision(REJECT, -1, 'bad_status')

    candidates = []
    skip = _skip_candidate(modes, capture_s, status, contract, profile, latency_s)
    if skip is not None:
        candidates.append(skip)       # first: with equal cost, no inference wins
    for mode in modes:
        candidates.append(
            _run_candidate(mode, capture_s, now_s, status, contract, profile, response_table,
                           latency_s))
    feasible = [candidate for candidate in candidates if candidate.feasible]
    if not feasible:
        return Decision(REJECT, -1, 'no_candidate_passes', tuple(candidates))
    best = min(feasible, key=lambda candidate: candidate.cost)
    return Decision(best.action, best.mode if best.action == RUN else -1, '', tuple(candidates))


def choose_by_score(contract, profile, modes, measured_latency_s):
    """Return the mode of the methods M2 and A2 (D-28).

    The score of a mode is its measurement error plus the way of the tool at
    the reference speed during its latency; the smallest wins. The latency is
    the one measured while the mode ran (measured_latency_s: {mode: s}), and
    that of the profile for a mode that has not run.
    """
    def score(mode):
        latency_s = measured_latency_s.get(mode, profile.modes[mode].release_to_publication_s)
        return contract.mode_error_m[mode] + profile.reference_speed_mps * latency_s
    return min(modes, key=score)
