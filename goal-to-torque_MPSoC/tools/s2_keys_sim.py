#!/usr/bin/env python3
"""s2_keys_sim.py - the keys of the operator with --arch S2, in the simulation.

With --arch S2 the controller is on the R5 (in the simulation: the simulated R5), and what a key does there goes
as a command: tools/hl_run_sim.py --arch S2 runs the tests of the high-level controller; this script presses the
other keys and waits for what each must print. Six sessions of the controller (-s --arch S2):

  keys     u and s are refused (they need the law here); h is refused with the controller off; r, g, k (twice),
           h; m a joint move to its target; t a task move, a second goal, x; y 6 HOME to its end; m and e in
           the motion; a start with the servo off is refused; h; j; r off, a start is refused; r; q.
           The controller says the mode of each motion; the arm is at the target of the joint move and at q_r
           after HOME (its angles from the view stream, VIEW_PORT); at the end 16 commands were written, none
           was refused by the R5 or dropped, and no report was missed.
  dropout  SIM_FAULT_WHAT=dropout: n in a joint move: the R5 switches the drives off and ends it; h enables again.
  estop    SIM_FAULT_WHAT=estop: n cuts the power: the R5 switches the drives off; h is refused while a drive
           reports Switch on disabled; n, h: on again.
  timeout  SIM_FAULT_WHAT=estop, SIM_ESTOP_REPORTS=ready, SERVO_ON_TIMEOUT_MS=600: with the power cut the drives say
           Ready to switch on and do not enable: h is taken, and the R5 withdraws it after its time; with the power
           back h enables. One servo on withdrawn, no dropout.
  fault    SIM_FAULT_WHAT=state: n in a joint move gives the R5 an angle that is no number: its controller fault
           ends the motion and switches the drives off; h is refused until r, r; then h enables again.
  law      SIM_FAULT_WHAT=torque: n is refused (the torque of the law is on the R5).

Before them: --arch S2 with a method other than M1 does not start.

Usage (in the repository, the controller built):
  python3 tools/s2_keys_sim.py [--cfg FILE] [--direct]
Exit code 0 when every line came. The script drives nothing but the simulation.
"""
import argparse
import math
import os
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402
import jt_predict as jp  # noqa: E402
from jt_run_sim import APP, Console, FAIL_START, SIM_START, SIM_READY, CTRL_ON, GRAV_COMP  # noqa: E402

# the time is that from the answer of the R5 to its first report with the drives on: a few periods
SERVO_ON = r'\[ARM\] servo on: every joint in Operation enabled \d ms after the R5 took the command'
PROMPT = r'\[MOVE\] > '


def start(cfg, direct, log_dir, name):
    cmd = (['./bin/indy7_ctrl', '-f', cfg, '-s'] if direct else ['./run.sh', '-s', '-f', cfg]) + ['--arch', 'S2']
    log = os.path.join(log_dir, 'run_s2keys_%s_%s.txt' % (name, time.strftime('%Y%m%d_%H%M%S')))
    print('s2_keys_sim: %s: %s; console -> %s' % (name, ' '.join(cmd), log))
    return Console(cmd, APP, dict(os.environ, MALLOC_ARENA_MAX='2'), log)


def stop(c):
    """a controller that a session left running (a line did not come) must not stay: it keeps its CPU and its port"""
    if c.proc.poll() is None:
        try:
            c.send('q')
        except OSError:
            pass
    c.close(timeout=10)


def posture(cfg):
    """the joint angles of the simulated arm now [deg], from the view stream of the controller"""
    port = int(jp.cfg_get(cfg, 'SYSTEM', 'VIEW_PORT', '0') or 0)
    with socket.create_connection(('127.0.0.1', port), timeout=3) as s:
        s.settimeout(3)
        for line in s.makefile():
            if not line.startswith('#'):
                return [math.degrees(float(x)) for x in line.split()[1:]]
    raise RuntimeError('the view stream (VIEW_PORT %d) gave no angles' % port)


def at(cfg, what, want):
    q = posture(cfg)
    if len(q) != len(want) or max(abs(a - b) for a, b in zip(q, want)) > 0.5:
        raise RuntimeError('%s: the arm is at [%s] deg, not at [%s]' % (what, ' '.join('%.1f' % x for x in q),
                                                                     ' '.join('%.1f' % x for x in want)))


def key(c, k, pattern, timeout=5, fail=None):
    c.send(k)
    return c.expect(pattern, timeout, fail=fail)


def on(c):
    key(c, 'r', CTRL_ON)
    key(c, 'g', GRAV_COMP)
    key(c, 'h', SERVO_ON)


def end(c):
    """q: the summary; the commands refused by the R5, dropped here, and the reports missed"""
    c.send('q')
    m1 = c.expect(r'\[S2\] reports of the R5\'s control task: (\d+) taken, (\d+) of a period with a state of the arm; '
                  r'(\d+) of such a period not taken', 15)
    m2 = c.expect(r'\[S2\] commands: (\d+) written, (\d+) refused by the R5, (\d+) dropped here', 5)
    return int(m1.group(3)), int(m2.group(1)), int(m2.group(2)), int(m2.group(3))


def keys(c, cfg):
    key(c, 'u', r'\[JT\] not with --arch S2')
    key(c, 's', r'\[TCP Pose\] not with --arch S2')
    key(c, 'h', r"refused - press 'r'")
    key(c, 'r', CTRL_ON)
    key(c, 'g', GRAV_COMP)
    first = key(c, 'k', r'grav-comp hold (ON|OFF)').group(1)
    second = key(c, 'k', r'grav-comp hold (ON|OFF)').group(1)
    if first == second:
        raise RuntimeError('k twice left the hold %s both times' % first)
    key(c, 'h', SERVO_ON)
    # a joint move, to its end
    key(c, 'm', PROMPT)
    key(c, '10 -10 -80 5 -80 10\n', r'\[MOVE\] joint move to')
    c.expect(r'\(CRobotIndy7\) mode: HOME', 5)
    c.expect_all([r'\[MOVE\] at the target', r'\(CRobotIndy7\) mode: Gravity Compensation'], 30,
                 fail=r'\[MOVE\] (stopped|ended|refused)[^\n]*|refused the start[^\n]*')
    at(cfg, 'after the joint move', [10, -10, -80, 5, -80, 10])
    # a task move, a second goal, the stop
    key(c, 't', PROMPT)
    key(c, '450 -100 450\n', r'\[MOVE\] task move to', fail=r'\[MOVE\] refused[^\n]*')
    c.expect(r'\(CRobotIndy7\) mode: TRACK', 5)
    time.sleep(3.0)
    key(c, 't', PROMPT)
    key(c, '500 100 400\n', r'\[MOVE\] new goal', fail=r'\[MOVE\] refused[^\n]*')
    time.sleep(3.0)
    key(c, 'x', r'\[MOVE\] stopped', fail=r'\[MOVE\] ended[^\n]*')
    time.sleep(0.5)
    # HOME to its end; HOME again with the e-stop in it
    key(c, 'y', r'\[HL\] high-level tests')
    key(c, '6', r'\[HL\] HOME: HOME to q_r', fail=r'\[HL\] refused[^\n]*')
    c.expect(r'\[HL\] at q_r', 30, fail=r'\[HL\] HOME stopped|refused the start[^\n]*')
    time.sleep(0.5)
    at(cfg, 'after HOME', [math.degrees(float(x)) for x in jp.cfg_get(cfg, 'SYSTEM', 'HL_READY_Q').split()])
    key(c, 'm', PROMPT)
    key(c, '20 0 -70 0 -70 0\n', r'\[MOVE\] joint move to')
    time.sleep(0.4)
    key(c, 'e', r'\[MOVE\] stopped', fail=r'\[MOVE\] at the target')
    time.sleep(0.5)
    key(c, 'y', r'\[HL\] high-level tests')
    key(c, '6', r'\[HL\] refused - servo OFF')
    key(c, 'h', SERVO_ON)
    key(c, 'j', r'SERVO OFF')
    time.sleep(0.3)
    key(c, 'r', r'\(CRobotIndy7\) controller DISABLED')
    time.sleep(0.3)
    key(c, 'y', r'\[HL\] high-level tests')
    key(c, '6', r'\[HL\] refused - controller not enabled')
    key(c, 'r', CTRL_ON)
    time.sleep(0.3)
    missed, written, refused, dropped = end(c)
    if missed or refused or dropped or written != 16:
        raise RuntimeError('at the end: %d commands written, %d refused by the R5, %d dropped; %d reports missed'
                           % (written, refused, dropped, missed))
    return written


def dropout(c, cfg):
    on(c)
    key(c, 'm', PROMPT)
    key(c, '20 0 -70 0 -70 0\n', r'\[MOVE\] joint move to')
    time.sleep(0.4)
    c.send('n')
    c.expect_all([r'\[FAULT\] injected: J\d leaves Operation enabled', r'DRIVES OFF by the R5', r'\[MOVE\] stopped'], 5)
    time.sleep(0.3)
    key(c, 'h', SERVO_ON)
    key(c, 'j', r'SERVO OFF')
    c.send('q')
    c.expect(r'\[RT\] drive dropouts: 1 ', 15)


def estop(c, cfg):
    on(c)
    c.send('n')
    c.expect_all([r'\[FAULT\] E-stop PRESSED', r'DRIVES OFF by the R5'], 5)
    time.sleep(0.3)
    key(c, 'h', r'\[ARM\] refused - J1 \(0x0040\)[^\n]*in Switch on disabled')
    key(c, 'n', r'\[FAULT\] E-stop RELEASED')
    time.sleep(0.3)
    key(c, 'h', SERVO_ON)
    key(c, 'j', r'SERVO OFF')
    c.send('q')
    c.expect(r'\[RT\] drive dropouts: 1 ', 15)


def timeout(c, cfg):
    key(c, 'r', CTRL_ON)
    key(c, 'g', GRAV_COMP)
    key(c, 'n', r'\[FAULT\] E-stop PRESSED')
    time.sleep(0.3)
    key(c, 'h', r'>>> SERVO ON')
    # 600 ms: the time limit that went with the command, not the 2000 periods that the R5 takes without one
    c.expect(r'SERVO ON NOT COMPLETE after 600 ms \(period \d+ of the R5\)', 1.5, fail=SERVO_ON + r'|DRIVES OFF by the R5')
    key(c, 'n', r'\[FAULT\] E-stop RELEASED')
    time.sleep(0.5)
    key(c, 'h', SERVO_ON, fail=r'\[ARM\] (the R5 )?refused[^\n]*')
    key(c, 'j', r'SERVO OFF')
    c.send('q')
    c.expect(r'\[RT\] drive dropouts: 0', 15)
    c.expect(r'\[RT\] servo on: 1 completed, [^\n]*, 1 withdrawn', 5)


def fault(c, cfg):
    on(c)
    key(c, 'm', PROMPT)
    key(c, '20 0 -70 0 -70 0\n', r'\[MOVE\] joint move to')
    time.sleep(0.4)
    c.send('n')
    c.expect_all([r'\[FAULT\] injected: a non-finite measured angle of J3', r'CONTROLLER FAULT on the R5',
                  r'\[MOVE\] stopped'], 5)
    time.sleep(0.3)
    key(c, 'h', r'\[ARM\] refused - controller fault latched')
    key(c, 'r', r'\(CRobotIndy7\) controller DISABLED')
    time.sleep(0.3)
    key(c, 'r', CTRL_ON)
    key(c, 'g', GRAV_COMP)
    time.sleep(0.3)
    key(c, 'h', SERVO_ON)
    key(c, 'j', r'SERVO OFF')
    c.send('q')
    c.expect(r'\[RT\] controller faults: 1 ', 15)
    c.expect(r'\[RT\] drive dropouts: 0', 5)


def law(c, cfg):
    key(c, 'r', CTRL_ON)
    key(c, 'n', r'\[FAULT\] not with --arch S2')
    c.send('q')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh')
    a = ap.parse_args()
    cfg = os.path.abspath(a.cfg)
    log_dir = jp.cfg_get(cfg, 'SYSTEM', 'JOINT_TEST_LOG_DIR', 'jt_logs') or 'jt_logs'
    if not os.path.isabs(log_dir):
        log_dir = os.path.join(os.path.dirname(cfg), log_dir)
    os.makedirs(log_dir, exist_ok=True)
    # the sessions with a fault test need their cfg: copies beside the cfg (its paths are relative to it)
    variants = {'dropout': ['SIM_FAULT_JOINT=3', 'SIM_FAULT_WHAT=dropout'],
                'estop': ['SIM_FAULT_JOINT=1', 'SIM_FAULT_WHAT=estop', 'SIM_ESTOP_REPORTS=sod'],
                'timeout': ['SIM_FAULT_JOINT=1', 'SIM_FAULT_WHAT=estop', 'SIM_ESTOP_REPORTS=ready',
                            'SERVO_ON_TIMEOUT_MS=600'],
                'fault': ['SIM_FAULT_JOINT=3', 'SIM_FAULT_WHAT=state'],
                'law': ['SIM_FAULT_JOINT=3', 'SIM_FAULT_WHAT=torque']}
    made, failed = [], 0
    r = subprocess.run(['./bin/indy7_ctrl', '-f', cfg, '-s', '--arch', 'S2', '--method', 'A2'], cwd=APP,
                       capture_output=True, text=True, timeout=30)
    if r.returncode != 1 or 'with --method M1 only' not in r.stderr:
        failed += 1
        print('s2_keys_sim: --arch S2 --method A2: FAILED: exit code %d, not refused' % r.returncode)
    try:
        for name, fn in (('keys', keys), ('dropout', dropout), ('estop', estop), ('timeout', timeout), ('fault', fault),
                         ('law', law)):
            use = cfg
            if name in variants:
                use = os.path.join(os.path.dirname(cfg), '.s2keys_%s.cfg' % name)
                shutil.copyfile(cfg, use)
                made += [use, use + '.changes.log']
                ok, out = cfgset.apply_files([use], variants[name])
                if not ok:
                    raise SystemExit('s2_keys_sim: %s' % '; '.join(out))
            c = start(use, a.direct, log_dir, name)
            try:
                c.expect(SIM_START, 30, fail=FAIL_START)
                c.expect(SIM_READY, 30, fail=FAIL_START)
                c.expect(r'\[R5\] the control task of the R5 reports', 10)
                time.sleep(0.5)
                out = fn(c, use)
                print('s2_keys_sim: %s: every line came%s' % (name, '' if out is None else ' (%d commands)' % out))
            except (RuntimeError, OSError) as ex:
                failed += 1
                print('s2_keys_sim: %s: FAILED: %s' % (name, ex))
            finally:
                stop(c)
    finally:
        for f in made:
            if os.path.exists(f):
                os.remove(f)
    print('s2_keys_sim: %s' % ('PASS' if failed == 0 else 'FAIL: %d session(s)' % failed))
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
