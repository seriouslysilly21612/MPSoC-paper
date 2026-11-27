"""frame_gate.py: what the detector does with an image, by the method of the run.

The detector has one place where an image is taken or dropped before the
inference (the worker loop of vitis_ai_detector_node.py). The method of the
run (paper, Table III; --method of the control application) decides what
stands there:

  M1        the fixed 30 ms gate of the node, as before (no FrameGate)
  M2, A2    every image the worker can take; the mode by its score
  A3, FULL  the admission rule (admission.decide), from the newest status of
            the control task; every decision is kept for the log

One FrameGate per run. on_status() is called by the thread that receives the
status, decide() by the worker loop.

A3 and FULL also measure the latency of every mode in the run: a run's
release is kept until the status first names its result (DynamicLatency).
With dynamic_latency the rule computes with those latencies; without, it
computes with the profile and the log still shows what they were. Pure stdlib, no ROS needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_frame_gate.py
"""
import csv
import json
import os
import threading
import time

from vitis_ai_detector_pkg import admission
from vitis_ai_detector_pkg.contract import ContractError, read_contract

METHODS_WITH_SCORE = ('M2', 'A2')
METHODS_WITH_RULE = ('A3', 'FULL')
METHODS = ('M1',) + METHODS_WITH_SCORE + METHODS_WITH_RULE

LOG_COLUMNS = (
    'capture_s', 'capture_task_s', 'release_s', 'action', 'mode', 'reason', 'status_age_ms',
    'tolerance_mm',
    'skip_age_ms', 'skip_bound_mm', 'skip_in_time', 'skip_within_tolerance', 'skip_cost',
    'm0_first_use_ms', 'm0_age_ms', 'm0_bound_mm', 'm0_in_time', 'm0_within_tolerance',
    'm0_response_ok', 'm0_cost',
    'm1_first_use_ms', 'm1_age_ms', 'm1_bound_mm', 'm1_in_time', 'm1_within_tolerance',
    'm1_response_ok', 'm1_cost',
    'm0_lm_ms', 'm0_lm_samples', 'm1_lm_ms', 'm1_lm_samples', 'lm_used')

# a run's release is forgotten when no status named its result by then
PENDING_KEEP_S = 2.0
# a status names the image of a run within this (images are 33 ms apart)
CAPTURE_MATCH_S = 0.005


class GateError(Exception):
    """The gate of this method cannot be set up."""


def mode_of_model(model_path):
    """Return the mode number of an xmodel, from the input size in its decode_meta.json."""
    meta_path = os.path.join(os.path.dirname(model_path), 'decode_meta.json')
    try:
        with open(meta_path, encoding='utf-8') as meta_file:
            input_size = json.load(meta_file)['input_size']
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise GateError(f'cannot read the input size of the model from {meta_path}: {error!r}')
    if isinstance(input_size, bool) or input_size not in admission.MODE_OF_INPUT_SIZE:
        raise GateError(f'{meta_path}: input size {input_size!r} is no mode of the mailbox '
                        f'({sorted(admission.MODE_OF_INPUT_SIZE)})')
    return admission.MODE_OF_INPUT_SIZE[input_size]


def task_clock_minus_system_clock():
    """Return the offset [s] between the clock of the control tasks and the system clock.

    The control application stamps its log on CLOCK_MONOTONIC, the images
    carry the system clock. The two are read one after the other, as the
    bridge of the control application does.
    """
    return time.clock_gettime(time.CLOCK_MONOTONIC) - time.time()


class FrameGate:
    """The gate of one run with a method other than M1."""

    def __init__(self, method, contract_cfg_path, profile_path, model_path,
                 response_table_path='', model_path_2='', dynamic_latency=False):
        if method not in METHODS_WITH_SCORE + METHODS_WITH_RULE:
            raise GateError(f'method {method!r} has no gate of this kind ({METHODS})')
        self.method = method
        try:
            self.contract = read_contract(contract_cfg_path)
            self.profile = admission.load_profile(profile_path)
            self.response_table = (
                admission.load_response_table(response_table_path) if response_table_path
                else None)
        except (ContractError, admission.ProfileError) as error:
            raise GateError(str(error)) from None
        # the modes of the models the worker loads: those the rule can choose among (D-23)
        modes = []
        for path in [model_path] + ([model_path_2] if model_path_2 else []):
            mode = mode_of_model(path)
            if mode not in self.profile.modes:
                raise GateError(f'{profile_path} has no entry for mode {mode}, the mode of {path}')
            if mode in modes:
                raise GateError(f'{model_path} and {model_path_2} are of the same mode {mode}')
            modes.append(mode)
        self.modes = tuple(modes)
        self.uses_status = method in METHODS_WITH_RULE
        self.dynamic_latency = bool(dynamic_latency)
        if self.dynamic_latency and not self.uses_status:
            raise GateError(f'dynamic_latency goes with {METHODS_WITH_RULE}: {method} reads no '
                            f'status')
        self.latency = admission.DynamicLatency(self.profile)
        self._pending = {}             # capture_s of a run -> (release_s, mode)
        self._last_result_n = None
        self._lock = threading.Lock()  # _pending and latency: two threads
        self._status = None
        self.rows = []
        self.counts = {admission.RUN: 0, admission.SKIP: 0, admission.REJECT: 0}
        self.unmatched_results = 0     # results the status named with no run of this gate known

    def on_status(self, status):
        """Take the newest admission.Status of the control task."""
        self._status = status
        if not self.uses_status:
            return
        with self._lock:
            new_result = (status.result_n > 0 and status.result_n != self._last_result_n)
            self._last_result_n = status.result_n
            if new_result:
                self._take_sample(status)
            for capture_s in [capture_s for capture_s, (release_s, _) in self._pending.items()
                              if status.stamp_s - release_s > PENDING_KEEP_S]:
                del self._pending[capture_s]

    def _take_sample(self, status):
        """A sample of L_m from the status that first names a result."""
        match = None
        for capture_s in self._pending:
            if abs(capture_s - status.result_capture_s) <= CAPTURE_MATCH_S:
                match = capture_s
                break
        if match is None:
            self.unmatched_results += 1
            return
        release_s, mode = self._pending.pop(match)
        self.latency.add(mode, status.stamp_s - release_s - status.control_period_s)

    def latencies(self):
        """{mode: (L_m [s] it would compute with, samples taken)}."""
        with self._lock:
            return {mode: (self.latency.value(mode), self.latency.taken[mode])
                    for mode in self.modes}

    def decide(self, capture_s, now_s):
        """Return the admission.Decision for the image captured at capture_s."""
        if not self.uses_status:
            # no mode measures its latency to the mailbox yet: the score takes that of the profile
            mode = admission.choose_by_score(self.contract, self.profile, self.modes, {})
            return admission.Decision(admission.RUN, mode, '')
        status = self._status           # one read: the other thread may replace it
        latencies = self.latencies()
        latency_s = ({mode: value for mode, (value, _) in latencies.items()}
                     if self.dynamic_latency else None)
        decision = admission.decide(capture_s, now_s, status, self.contract, self.profile,
                                    self.modes, self.method, self.response_table, latency_s)
        if decision.action == admission.RUN:
            with self._lock:
                self._pending[capture_s] = (now_s, decision.mode)
        self.counts[decision.action] += 1
        row = self._log_row(capture_s, now_s, status, decision)
        for mode, (value, taken) in latencies.items():
            row[f'm{mode}_lm_ms'] = f'{1e3 * value:.3f}'
            row[f'm{mode}_lm_samples'] = taken
        row['lm_used'] = 'dynamic' if self.dynamic_latency else 'profile'
        self.rows.append(row)
        return decision

    @staticmethod
    def _log_row(capture_s, now_s, status, decision):
        row = dict.fromkeys(LOG_COLUMNS, '')
        row.update(
            capture_s=f'{capture_s:.6f}',
            capture_task_s=f'{capture_s + task_clock_minus_system_clock():.6f}',
            release_s=f'{now_s:.6f}', action=decision.action, mode=decision.mode,
            reason=decision.reason)
        if status is not None:
            row.update(status_age_ms=f'{1e3 * (now_s - status.stamp_s):.3f}',
                       tolerance_mm=f'{1e3 * status.tolerance_m:.3f}')
        for candidate in decision.candidates:
            run = candidate.action == admission.RUN
            prefix = f'm{candidate.mode}_' if run else 'skip_'
            row.update({
                prefix + 'age_ms': f'{1e3 * candidate.age_s:.3f}',
                prefix + 'bound_mm': f'{1e3 * candidate.error_bound_m:.4f}',
                prefix + 'in_time': int(candidate.in_time),
                prefix + 'within_tolerance': int(candidate.within_tolerance),
                prefix + 'cost': f'{candidate.cost:.6g}'})
            if run:
                row.update({
                    prefix + 'first_use_ms': f'{1e3 * (candidate.first_use_s - now_s):.3f}',
                    prefix + 'response_ok': int(candidate.response_ok)})
        return row

    def save_log(self, path):
        """Write every decision of the run to a CSV file; return the number of rows."""
        with open(path, 'w', newline='', encoding='utf-8') as log_file:
            writer = csv.DictWriter(log_file, fieldnames=LOG_COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)
        return len(self.rows)
