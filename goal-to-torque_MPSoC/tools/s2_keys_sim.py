#!/usr/bin/env python3
"""s2_keys_sim.py - the keys of the operator with --arch S2, in the simulation.

With --arch S2 the controller is on the R5 (in the simulation: the simulated R5), and what a key does there goes
as a command: tools/hl_run_sim.py --arch S2 runs the tests of the high-level controller; this script presses the
other keys and waits for what each must print. Six sessions of the controller (-s --arch S2):

  keys     u and s are refused (they need the law here); h is refused with the controller off; r, g, k (twice),
           h; m a joint move to its target; t a task move, a second goal, x; y 6 HOME to its end; m and e in
           the motion; a start with the servo off is refused; h; j; r off, a start is refused; r; q.
           The controller says the mode of each motion; the arm is at the target of the joint move and at q_r
           after HOME (its angles from the view stream, VIEW_PORT); at the end 17 commands were written (the
           last two those of q: servo off, controller off), none was refused by the R5 or dropped, and no report
           was missed.
  dropout  SIM_FAULT_WHAT=dropout: n in a joint move: the R5 switches the drives off and ends it; h enables again.
  estop    SIM_FAULT_WHAT=estop: n cuts the power: the R5 switches the drives off; h is refused while a drive
           reports Switch on disabled; n, h: on again.
  timeout  SIM_FAULT_WHAT=estop, SIM_ESTOP_REPORTS=ready, SERVO_ON_TIMEOUT_MS=600: with the power cut the drives say
           Ready to switch on and do not enable: h is taken, and the R5 withdraws it after its time; with the power
           back h enables. One servo on withdrawn, no dropout.
  fault    SIM_FAULT_WHAT=state: n in a joint move gives the R5 an angle that is no number: its controller fault
           ends the motion and switches the drives off; h is refused until r, r; then h enables again.
  law      SIM_FAULT_WHAT=torque: n is refused (the torque of the law is on the R5). Then g, h, and q with the
           servo on: the run still leaves the R5 with the servo not asked and its controller off.

Every session ends with q, and its summary must say that the run leaves the R5 as a new one has it: controller
off, the servo not asked, no motion. The first session also writes its timing log (-t): that of --arch S2, with
the time that S2 follows the R5 behind, and tools/timing_summary.py finds the TRACK of the task move in it, with a
report of the R5 in most of its cycles.

Before them: --arch S2 with a method other than M1 does not start; the fault injection is refused with -b and
--arch S1, and not with -b and --arch S2 (there the controller cannot open the slots of an R5 on a PC, and ends
for that); and the timing log of a run with --arch S1 has the time that S1 follows the R5 behind, not that of S2.

Usage (in the repository, the controller built):
  python3 tools/s2_keys_sim.py [--cfg FILE] [--direct] [--bench BUS]
Exit code 0 when every line came. The script drives nothing but the simulation.

  --bench BUS  on the board, the R5 itself with a firmware that has the control task (rpu_work/r5_freertos,
               CONTROL_TASK), its bus that of BUS (indy7 -b -B BUS, e.g. indy7/bench/L7N3_bus.cfg): the same six
               sessions, the arm still the one simulated here, the drives of the bus off. What the keys ask is
               then decided by the firmware, and every session finds what the one before left on the R5: its
               controller off (a run that ends with q switches it off). A seventh session then:
  restart      the controller is killed in a joint move with the servo on. The R5 goes on; the next run finds its
               controller on, the servo asked and the motion, and is told the stop of the drives when the R5
               finds the drives of the new simulated arm off; h enables again, and q leaves the R5 as a new one.
"""
import argparse
import glob
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402
import jt_predict as jp  # noqa: E402
from jt_run_sim import APP, Console, FAIL_START, SIM_START, SIM_READY, CTRL_ON, GRAV_COMP  # noqa: E402
from run_timing import B_READY  # noqa: E402
import timing_summary as ts  # noqa: E402

# the time is that from the answer of the R5 to its first report with the drives on: a few periods (--bench: a
# period of the R5 without a state of the arm now and then, so up to 99)
SERVO_ON = r'\[ARM\] servo on: every joint in Operation enabled \d ms after the R5 took the command'
PROMPT = r'\[MOVE\] > '
# what the control task of the R5 has when a session begins
FIRST = (r'\[R5\] the control task of the R5 reports \(its period \d+\): (controller (?:on|off), the servo '
         r'(?:asked on|not asked), (?:a motion runs there|no motion))')
NEW = 'controller off, the servo not asked, no motion'
LEFT = r'\[S2\] the R5 as this run leaves it \(its period \d+\): ([^\n]*)'


def start(cfg, direct, log_dir, name, bus, timing=None):
    arm = ['-b', '-B', bus] if bus else ['-s']
    cmd = (['./bin/indy7_ctrl', '-f', cfg] + arm if direct else ['./run.sh'] + arm + ['-f', cfg]) + ['--arch', 'S2']
    if timing:
        cmd += ['-t', timing]
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


def left(c, timeout=15):
    """the line of the summary that says what this run leaves on the R5 (anywhere in the console so far: the
    sessions wait for lines behind it)"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with c.lock:
            m = re.search(LEFT, c.text)
        if m:
            return m.group(1).strip()
        time.sleep(0.05)
    raise RuntimeError('no line of what the run leaves on the R5 within %.0f s' % timeout)


def ready(c, bus):
    """the controller is up -> what the control task of the R5 has"""
    if bus:
        c.expect_all([B_READY, r'\[R5\] OP: '], 120, fail=FAIL_START)
    else:
        c.expect(SIM_START, 30, fail=FAIL_START)
        c.expect(SIM_READY, 30, fail=FAIL_START)
    return c.expect(FIRST, 10).group(1)


def timing_log(cfg, out):
    """the timing log of a session that had a TRACK, as the summary of the timing runs reads it"""
    logs = glob.glob(os.path.join(out, 'timing_*.bin'))
    if len(logs) != 1:
        raise RuntimeError('%d timing logs of the session' % len(logs))
    head = ts.read_log(logs[0])[0]
    behind = int(round(float(jp.cfg_get(cfg, 'R5_LINK', 'WAKE_DELAY_S2_US', '640') or 640) * 1000))
    if head.get('arch') != 'S2' or head.get('wake_delay_ns') != str(behind):
        raise RuntimeError('the timing log has arch=%s, wake_delay_ns=%s; S2 follows the R5 %d ns behind'
                           % (head.get('arch'), head.get('wake_delay_ns'), behind))
    r = subprocess.run([sys.executable, '-B', os.path.join(HERE, 'timing_summary.py'), logs[0], '--out', out],
                       capture_output=True, text=True)
    cycles = re.search(r'(\d+) cycles measured, (\d+) with a new State', r.stdout)
    none = re.search(r'cycles in OP without a new report of the control task of the R5: (\d+)', r.stdout)
    if r.returncode not in (0, 1) or '--arch S2: ' not in r.stdout or not cycles or not none:
        raise RuntimeError('timing_summary.py on the timing log: exit code %d, %s'
                           % (r.returncode, (r.stderr or r.stdout).strip().splitlines()[-1:]))
    # the TRACK of the task move, about 6 s; a report in most cycles (none in any: the summary does not find them)
    if int(cycles.group(1)) < 3000 or 2 * int(none.group(1)) > int(cycles.group(1)):
        raise RuntimeError('the timing log: %s cycles in TRACK, %s of them without a report of the R5'
                           % (cycles.group(1), none.group(1)))
    return int(cycles.group(1)), int(none.group(1))


def s1_timing_head(cfg, direct, log_dir):
    """a run with --arch S1 (-s) that ends at once: its timing log has the time that S1 follows the R5 behind"""
    out = tempfile.mkdtemp(prefix='s2keys_timing_')
    cmd = (['./bin/indy7_ctrl', '-f', cfg, '-s'] if direct else ['./run.sh', '-s', '-f', cfg]) + ['--arch', 'S1', '-t', out]
    c = Console(cmd, APP, dict(os.environ, MALLOC_ARENA_MAX='2'),
                os.path.join(log_dir, 'run_s2keys_s1_%s.txt' % time.strftime('%Y%m%d_%H%M%S')))
    try:
        c.expect(SIM_READY, 30, fail=FAIL_START)
        time.sleep(0.5)
        stop(c)
        logs = glob.glob(os.path.join(out, 'timing_*.bin'))
        head = ts.read_log(logs[0])[0] if len(logs) == 1 else {}
        behind = int(round(float(jp.cfg_get(cfg, 'R5_LINK', 'WAKE_DELAY_US', '150') or 150) * 1000))
        if head.get('arch') != 'S1' or head.get('wake_delay_ns') != str(behind):
            raise RuntimeError('the timing log has arch=%s, wake_delay_ns=%s; S1 follows the R5 %d ns behind'
                               % (head.get('arch'), head.get('wake_delay_ns'), behind))
    finally:
        stop(c)
        shutil.rmtree(out, ignore_errors=True)


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
    if missed or refused or dropped or written != 17:
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
    # one fault of one period: told once, not with every report after it
    m = c.expect(r'\[RT\] controller faults: 1 of the law on the R5 \(first at its period (\d+), last at its period (\d+)', 15)
    if m.group(1) != m.group(2):
        raise RuntimeError('one controller fault, told from period %s to period %s of the R5' % (m.group(1), m.group(2)))
    c.expect(r'\[RT\] drive dropouts: 0', 5)


def law(c, cfg):
    key(c, 'r', CTRL_ON)
    key(c, 'n', r'\[FAULT\] not with --arch S2')
    key(c, 'g', GRAV_COMP)
    key(c, 'h', SERVO_ON)
    c.send('q')         # with the servo on: the application asks it off, then the controller


def restart(cfg, direct, log_dir, bus):
    """--bench: the R5 outlives the controller"""
    c = start(cfg, direct, log_dir, 'restart-killed', bus)
    try:
        has = ready(c, bus)
        if has != NEW:
            raise RuntimeError('the R5 has: %s; the sessions begin with: %s' % (has, NEW))
        time.sleep(0.5)
        on(c)
        key(c, 'm', PROMPT)
        key(c, '20 0 -70 0 -70 0\n', r'\[MOVE\] joint move to')
        time.sleep(0.4)
        os.kill(c.proc.pid, signal.SIGKILL)     # run.sh has become the controller (exec)
    finally:
        stop(c)
    time.sleep(1.0)
    c = start(cfg, direct, log_dir, 'restart', bus)
    try:
        has, want = ready(c, bus), 'controller on, the servo asked on, a motion runs there'
        if has != want:
            raise RuntimeError('after the kill the R5 has: %s; expected: %s' % (has, want))
        # the arm of this run is a new one, its drives off: the R5 takes that for a stop of the drives
        c.expect(r'DRIVES OFF by the R5', 5)
        time.sleep(0.5)
        key(c, 'h', SERVO_ON, fail=r'\[ARM\] (the R5 )?refused[^\n]*')
        key(c, 'j', r'SERVO OFF')
        c.send('q')
        c.expect(r'\[RT\] drive dropouts: 1 ', 15)
        if left(c) != NEW:
            raise RuntimeError('the run leaves the R5 with: %s' % left(c))
    finally:
        stop(c)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh')
    ap.add_argument('--bench', metavar='BUS', help='on the board: the R5 itself, its bus that of BUS (-b -B BUS)')
    a = ap.parse_args()
    cfg = os.path.abspath(a.cfg)
    bus = os.path.abspath(a.bench) if a.bench else None
    if bus:
        global SERVO_ON
        SERVO_ON = SERVO_ON.replace(r'\d ms', r'\d{1,2} ms')
    if int(jp.cfg_get(cfg, 'SYSTEM', 'VIEW_PORT', '0') or 0) == 0:
        sys.exit('s2_keys_sim: VIEW_PORT=0 in %s: the sessions read the posture of the arm from the view stream' % cfg)
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
        if not bus:
            # the fault injection beside a bus (-b): only where the arm of the R5's control task is the simulated
            # one (S2). On a PC neither run gets further than the slots of the R5, which are not there
            use = os.path.join(os.path.dirname(cfg), '.s2keys_rule.cfg')
            shutil.copyfile(cfg, use)
            made += [use, use + '.changes.log']
            ok, out = cfgset.apply_files([use], variants['fault'])
            if not ok:
                raise SystemExit('s2_keys_sim: %s' % '; '.join(out))
            for arch, refused in (('S1', True), ('S2', False)):
                r = subprocess.run(['./bin/indy7_ctrl', '-f', use, '-b', '--arch', arch], cwd=APP,
                                   capture_output=True, text=True, timeout=30)
                text = r.stdout + r.stderr
                if r.returncode == 0 or ('is refused outside the simulation' in text) != refused or \
                   ("[FAULT] fault injection on 'n'" in text) == refused:
                    failed += 1
                    print('s2_keys_sim: the fault injection with -b --arch %s: FAILED: %s'
                          % (arch, 'not refused' if refused else 'refused, or the controller started'))
            try:
                s1_timing_head(cfg, a.direct, log_dir)
            except (RuntimeError, OSError) as ex:
                failed += 1
                print('s2_keys_sim: the timing log with --arch S1: FAILED: %s' % ex)
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
            timing = tempfile.mkdtemp(prefix='s2keys_timing_') if name == 'keys' else None
            c = start(use, a.direct, log_dir, name, bus, timing)
            try:
                # as a new R5 has it: a session before that ended with q left its controller off
                has = ready(c, bus)
                if has != NEW:
                    raise RuntimeError('the R5 has: %s; the sessions begin with: %s' % (has, NEW))
                time.sleep(0.5)
                out = fn(c, use)
                if left(c) != NEW:
                    raise RuntimeError('the run leaves the R5 with: %s' % left(c))
                print('s2_keys_sim: %s: every line came%s' % (name, '' if out is None else ' (%d commands)' % out))
                if timing:
                    stop(c)         # the log is whole when the controller has ended
                    print('s2_keys_sim: %s: the timing log: %d cycles in TRACK, %d without a report of the R5'
                          % ((name,) + timing_log(use, timing)))
            except (RuntimeError, OSError) as ex:
                failed += 1
                print('s2_keys_sim: %s: FAILED: %s' % (name, ex))
            finally:
                stop(c)
                if timing:
                    shutil.rmtree(timing, ignore_errors=True)
        if bus:
            try:
                restart(cfg, a.direct, log_dir, bus)
                print('s2_keys_sim: restart: every line came')
            except (RuntimeError, OSError) as ex:
                failed += 1
                print('s2_keys_sim: restart: FAILED: %s' % ex)
    finally:
        for f in made:
            if os.path.exists(f):
                os.remove(f)
    print('s2_keys_sim: %s' % ('PASS' if failed == 0 else 'FAIL: %d session(s)' % failed))
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
