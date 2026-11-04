#!/usr/bin/env python3
"""Synthetic (camera-free) verification of pick_logic v2.

Publishes crafted DetectionArray sequences on /detections, runs the real
pick_logic_node (subprocess, same params file as the launch), and asserts the
PickTarget stream on /pick_target.

Scenarios (v3, 2026-09-04: operator-named class, no scoring / AUTO):
  S1 no selection   — desired_class '' with apple+banana visible: every
                      target invalid (no_class_selected).
  S2 desired_class  — live `ros2 param set desired_class banana`:
                      valid targets are banana only.
  S3 desired absent — desired_class=orange while only apple/banana visible:
                      stream stays invalid.
  S4 max-confidence — desired_class=apple, two apples: the higher-confidence
                      one is adopted (checked by center_x).
  S5 sticky         — the two apples alternate as "more confident" every
                      frame: the published instance must not change.
  S6 occluded move  — the tracked apple hidden 2 frames (< lost_timeout),
                      reappears alone 200 px away: valid again on the FIRST
                      frame (single instance = unambiguous).

Run:  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
      python3 ~/ros2_ws/tools/vision/test_pick_logic_v2.py
"""
import os
import signal as _signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from my_interfaces.msg import Detection, DetectionArray, PickTarget

PARAMS = {
    'min_confidence': 0.3,
    'allowed_classes': ['apple', 'orange', 'banana', 'tennis_ball', 'mustard_bottle'],
    'image_width': 848, 'image_height': 480,
    'stability_min_frames': 3, 'lost_timeout_frames': 5,
    'stability_match_px': 60.0,
    'publish_invalid_targets': True,
    'log_period_sec': 999.0,          # keep the log quiet
}


def det(cls, conf, cx, cy, w=80.0, h=80.0):
    d = Detection()
    d.class_id = 0
    d.class_name = cls
    d.confidence = float(conf)
    d.center_x, d.center_y = float(cx), float(cy)
    d.width, d.height = float(w), float(h)
    return d


class Harness(Node):
    def __init__(self):
        super().__init__('pick_logic_v2_test_harness')
        self.pub = self.create_publisher(DetectionArray, '/detections', 10)
        self.sub = self.create_subscription(PickTarget, '/pick_target',
                                            self._on_target, 10)
        self.results = []

    def _on_target(self, msg: PickTarget):
        self.results.append((msg.target_valid, msg.class_name,
                             round(msg.center_x)))

    def send(self, dets, n_frames, dt=0.02):
        """Publish n_frames DetectionArrays and collect responses."""
        self.results.clear()
        for _ in range(n_frames):
            arr = DetectionArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.detections = dets
            self.pub.publish(arr)
            t_end = time.time() + dt
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.005)
        # drain
        t_end = time.time() + 0.2
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.005)
        return list(self.results)


def set_param(name, value):
    # The freshly-respawned daemon needs a moment to discover the node; retry
    # instead of failing the whole suite on its first empty graph snapshot.
    last = None
    for _ in range(5):
        r = subprocess.run(
            ['ros2', 'param', 'set', '/pick_logic_node', name, value],
            capture_output=True, text=True)
        if r.returncode == 0:
            return
        last = r
        time.sleep(1.0)
    raise RuntimeError(f'param set {name}={value!r} failed: '
                       f'{(last.stderr or last.stdout).strip()}')


def summarize(results):
    valid = [c for v, c, _ in results if v]
    invalid = len([1 for v, _, _ in results if not v])
    return valid, invalid


def valid_cx(results):
    return [cx for v, _, cx in results if v]


def _kill_stale_nodes():
    """A half-dead leftover node poisons the ros2 daemon's graph cache and
    steals/blackholes traffic — refuse to test next to one."""
    out = subprocess.run(['pgrep', '-f', 'install/pick_logic_pkg'],
                         capture_output=True, text=True).stdout.split()
    killed = False
    for pid in out:
        # -f matches any process whose CMDLINE merely CONTAINS the pattern
        # (e.g. a shell that echoed it) — only kill actual python nodes.
        try:
            with open(f'/proc/{pid}/comm') as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm not in ('python3', 'pick_logic'):
            continue
        print(f'preflight: killing stale pick_logic pid {pid} ({comm})')
        subprocess.run(['kill', '-9', pid], capture_output=True)
        killed = True
    if killed:
        time.sleep(1.0)
    # SIGKILLed nodes never unregister -> the ros2 daemon's cached graph keeps
    # routing `ros2 param set` to dead nodes. Always start from a fresh daemon
    # (it respawns on first use).
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    time.sleep(0.5)


def main():
    _kill_stale_nodes()
    rclpy.init()
    node = Harness()

    param_args = []
    for k, v in PARAMS.items():
        param_args += ['-p', f'{k}:={v}']
    # start_new_session so teardown can kill the WHOLE group — terminating just
    # the `ros2 run` wrapper orphans the actual node (observed: duplicate nodes
    # doubling every /pick_target message on reruns).
    proc = subprocess.Popen(
        ['ros2', 'run', 'pick_logic_pkg', 'pick_logic', '--ros-args'] + param_args,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)

    # Functional handshake: keep sending empty frames until the node actually
    # ANSWERS (graph-count checks can be satisfied by the daemon's stale cache).
    t_end = time.time() + 15.0
    ready = False
    while time.time() < t_end and not ready:
        arr = DetectionArray()
        arr.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(arr)
        t_spin = time.time() + 0.15
        while time.time() < t_spin:
            rclpy.spin_once(node, timeout_sec=0.02)
        ready = len(node.results) > 0
    if not ready:
        print('FATAL: pick_logic node never answered')
        os.killpg(os.getpgid(proc.pid), _signal.SIGKILL)
        return 1
    node.results.clear()

    apple = det('apple', 0.95, 424, 240)          # center, strong
    banana = det('banana', 0.60, 700, 120)        # off-center, weak

    passed, failed = [], []

    def check(name, cond, detail):
        (passed if cond else failed).append(name)
        print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")

    try:
        # S1: nothing selected -> everything invalid
        r = node.send([apple, banana], 6)
        valid, inv = summarize(r)
        check('S1 no-selection', len(valid) == 0 and inv >= 5,
              f'valid={len(valid)}, invalid={inv}')

        # S2: live desired_class = banana
        set_param('desired_class', 'banana')
        r = node.send([apple, banana], 8)
        valid, inv = summarize(r)
        check('S2 desired_class', len(valid) > 0 and set(valid) == {'banana'}
              and inv >= PARAMS['stability_min_frames'] - 1,
              f'valid={len(valid)}x{set(valid) or "{}"}, warmup_invalid={inv}')

        # S3: desired class not visible -> all invalid
        set_param('desired_class', 'orange')
        r = node.send([apple, banana], 6)
        valid, inv = summarize(r)
        check('S3 desired-absent', len(valid) == 0 and inv >= 5,
              f'valid={len(valid)}, invalid={inv}')

        # S4: two apples -> the more confident one is adopted
        set_param('desired_class', 'apple')
        apple_a = det('apple', 0.70, 300, 240)
        apple_b = det('apple', 0.90, 500, 240)          # 200 px apart > match_px
        r = node.send([apple_a, apple_b], 6)
        cxs = valid_cx(r)
        check('S4 max-confidence', len(cxs) > 0 and set(cxs) == {500},
              f'valid_cx={cxs}')

        # S5: sticky — alternate which apple is "more confident" each frame
        seq = []
        for i in range(12):
            a = det('apple', 0.80 + (0.10 if i % 2 == 0 else 0.0), 300, 240)
            b = det('apple', 0.80 + (0.10 if i % 2 == 1 else 0.0), 500, 240)
            seq += valid_cx(node.send([a, b], 1))
        check('S5 sticky', len(seq) > 0 and set(seq) == {500},
              f'valid_cx_seq={seq}')

        # S6: single instance re-acquired at once after an occluded move
        node.send([], 2)                                      # hidden < lost_timeout
        r = node.send([det('apple', 0.90, 700, 240)], 3)      # +200 px > match_px
        valid, inv = summarize(r)
        check('S6 occluded-move', len(r) == 3 and inv == 0
              and set(valid_cx(r)) == {700},
              f'valid_cx={valid_cx(r)}, invalid={inv}')

    finally:
        # rclpy's idle spin swallows SIGTERM (EINTR re-wait, python handler
        # never runs with no traffic to wake the executor) — the wrapper dies
        # but the node survives. Always finish the group with SIGKILL.
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pgid = None
        if pgid is not None:
            try:
                os.killpg(pgid, _signal.SIGTERM)
                proc.wait(timeout=2)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass
            try:
                os.killpg(pgid, _signal.SIGKILL)
            except ProcessLookupError:
                pass
        node.destroy_node()
        rclpy.shutdown()

    print(f'\n==> {len(passed)} PASS / {len(failed)} FAIL'
          + (f'  (failed: {failed})' if failed else ''))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
