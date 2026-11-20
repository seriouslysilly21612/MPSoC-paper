#!/usr/bin/env python3
"""run_timing.py - one condition of the host-timing test (paper, Sec. III-C, Table 6, Fig. 3).

The bus of the robot in OP with its drives off, beside the simulated arm (-b), the reference-check goal
stream (high-level test 5: newly sampled reachable goals, unreachable goals, repeated goals, goal-free
intervals, 1-mm perturbations every 67 periods) for --duration s (with --desired-class the camera application
in its place), the timing of every period (-t), and the summary of tools/timing_summary.py (the row of Table 6).

  python3 tools/run_timing.py --condition idle|stress|perception|pressure [--pressure 0|25|50|75|100]
                              [--duration 600] [--cfg indy7/INDY7.cfg] [--bus FILE] [--out DIR] [--settle 30]
                              [--direct] [--no-hl-log] [--arch P0|S1|S2] [--shadow]
                              [--method M1|M2|A2|A3|FULL] [--desired-class CLASS]

  --bus FILE  another bus than that of the cfg (indy7 -B FILE, e.g. indy7/bench/L7N3_bus.cfg, the L7N
              bench); the arm and everything else stay those of the cfg; FILE is copied into OUT
  --arch S1   the EtherCAT task on the R5 (rpu_work/README_S1.md): the R5 runs the bus and must have it in OP, the
              controller takes its State slot and follows its period; no IgH master. The record of every cycle
              of the R5 is taken during the run (rpu_work/r5_freertos/r5rec.py, into OUT/r5, its output in
              OUT/r5rec.txt), and the summary has the quantities of S1 with the send of the R5 from that log
              (tools/timing_summary.py --r5)
  --shadow    with --arch S1: the control task tells the R5 every period what it did (indy7 --shadow), and a
              firmware with the second controller (SHADOW_CONTROL) runs the control computation after its send;
              the summary then has its times on the R5 and what they say about the control task there (D-14)
  --arch S2   the control task on the R5 as well (a firmware built with CONTROL_TASK; rpu_work/r5_freertos/
              README.md): the keys below go to it as commands, and with -b the arm simulated here steps on its
              answers. The record of the R5 is taken as with S1; the summary has the row of Table 6 from it, with
              the two parts of its control task and its t_w + t_e in place of a chain here (tools/
              timing_summary.py). With --shadow a controller here repeats every period of the R5 from its report
              (the console: [S2] the control of the R5 a second time here)
  --method M  --method of the controller and method:= of the camera pipeline (default M1). With another method
              the results of the camera go through the mailbox to the control task (with S2 the R5's); they come
              only for an object of --desired-class
  --desired-class CLASS
              the camera application in place of the goal stream (perception, pressure): 'p', the digit of CLASS
              in the object menu of the bridge (CLASS must be in view), HOME to q_r, then TRACK following it for
              --duration s, 'x'. The bridge clears desired_class:= of the launch when a session starts, so the
              menu is the only way to set it. Without it the goal stream runs, and with a method other than M1
              the mailbox stays empty. The logs of the application (obs_*, hl_app_*) go to OUT/hl_logs, the
              decisions of the detector (A3, FULL) to OUT/decisions.csv
  --no-hl-log HL_LOG=0 in the copy of the cfg: the high-level test is not logged (the 30-min runs of
              tools/run_batch.py; the log takes about 0.7 kB per period)

  idle        nothing else runs on the host
  stress      stress-ng --taskset OTHERS --cpu 3 --vm 1 --vm-bytes 10% (KV260; the paper, Table 6: --cpu 14 --vm 2
              --vm-bytes 35% on its 16 threads); OTHERS = the CPUs besides the real-time core (default 0-2)
  perception  the camera pipeline runs (ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py, started
              here unless it runs already, its output in OUT/vision.txt, stopped at the end if started here;
              the detector processes every frame it can: its gate, 30 ms, is shorter than its processing,
              about 43 ms), and the controller runs its bridge (a build with ROS 2, APP_BRIDGE=1)
  pressure    perception + memory pressure (D-11): stress-ng --taskset OTHERS --memrate 3, its read
              and write rates bounded so that its time-averaged rate is about --pressure % of its unbounded
              rate beside the camera pipeline (MEMRATE_CAP; 0: no memrate, i.e. perception; 100: unbounded);
              the rates it reached in OUT/memrate.txt. Without the stress load: beside it the workers got about
              a third of the CPUs and reached about 2 GB/s whatever the bound, so 50, 75 and 100 % were the
              same (11-15)

In order:
  1. checks, warnings only: the host as in Table 3 (PREEMPT_RT, the core isolated, kernel.timer_migration 0,
     NTP not synchronizing the clock), the bus (the IgH master running), the controller built
  2. a copy of the cfg in OUT: HL_TEST_REF_S = the duration, VIEW_PORT = 0, JOINT_TEST_LOG_DIR = OUT/hl_logs,
     its relative paths made absolute
  3. every second until the end, the PS temperatures and the board power: OUT/temp_power.csv
  4. the load of the condition, then 120 s before the controller
  5. the controller through indy7/run.sh (the limits, EtherCAT-OP at SCHED_FIFO 40) with -b -t OUT; --settle s
     in OP; then the keys r, g, k (the hold off), h (servo on of the simulated arm; the drives of the bus stay
     off), y 5 (HOME to q_r, then TRACK with the stream); its end (--desired-class: p and the digit of the
     class, HOME, TRACK; x after --duration s); j, q
  6. the load stopped; tools/timing_summary.py on the timing log: OUT/timing_summary.txt, timing_table.csv
The console of the controller is saved as OUT/console.txt. The high-level log of the run takes about 0.7 kB
per period (OUT/hl_logs), the timing log 60 bytes per period. Standard library only.
"""
import argparse
import glob
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402
from jt_run_sim import APP, Console, FAIL_START, CTRL_ON, GRAV_COMP  # noqa: E402

PATH_KEYS = ('URDF_PATH', 'SIM_MJCF_PATH', 'JOINT_TEST_LOG_DIR', 'CALIB_DIR', 'HL_TEST_NOISE_FILE')
STRESS_ARGS = ['--cpu', '3', '--vm', '1', '--vm-bytes', '10%', '--timeout', '0', '--quiet']   # KV260: 3 cores, 3.8 GB
PERCEPTION = ('realsense2_camera_node', 'vitis_ai_detector_node', 'pick_target_base_node')
VISION = ('source /opt/ros/humble/setup.bash && source "$HOME/ros2_ws/install/setup.bash" && '
          'exec ros2 launch system_bringup_pkg pick_place_vitis_ai.launch.py')            # as run.sh sources them
# the line of vitis_ai_detector_node once its worker has loaded the model; the loads start only after it: under
# them the worker missed the node's 30 s startup limit (11-15, run 04 of the quick batch)
VISION_READY = 'Vitis-AI worker ready'
# The memory pressure, read and write bounds [MB/s] of each of the three memrate workers. A worker runs the 17
# methods of a round over its buffer in turn, so its time-averaged rate is the harmonic mean of their rates.
# Beside the camera pipeline the three reached 2.87 GB/s unbounded (2026-11-15, the quick batch, runs 04-06); by
# the rates of the methods there these bounds give about 26, 52, 76 % of it. (Alone, nothing else running, the
# three reached 6.0 GB/s; the bounds from that, 500, 1000, 1620, gave 52, 83, 93 % beside the pipeline.)
MEMRATE_CAP = {25: 250, 50: 500, 75: 800}
MEMRATE_ARGS = ['--memrate', '3', '--timeout', '0', '--metrics']
AMS, INA = '/sys/bus/iio/devices/iio:device0', '/sys/bus/iio/devices/iio:device1'   # as tools/host/kv260_rt_setup.sh
B_READY = r'\(CRobotIndy7\) \S+: the bus of the robot \(drives off\) beside the simulated arm \(-b\)'
# --arch S2: what the control task of the R5 has when this run takes its first report
FIRST_REPORT = r'\[R5\] the control task of the R5 reports \(its period \d+\): controller (on|off)'
R5REC = os.path.normpath(os.path.join(HERE, '..', '..', 'rpu_work', 'r5_freertos', 'r5rec.py'))


def warn(text):
    print('run_timing: WARNING: %s' % text)


def cpu_list(text):
    """'domain,managed_irq,6-7' or '0-5,8-15' -> the set of CPU numbers"""
    cpus = set()
    for part in text.split(','):
        if re.fullmatch(r'\d+-\d+', part):
            lo, hi = part.split('-')
            cpus.update(range(int(lo), int(hi) + 1))
        elif part.isdigit():
            cpus.add(int(part))
    return cpus


def ntp_active():
    try:
        return subprocess.run(['timedatectl', 'show', '-p', 'NTP', '--value'], capture_output=True, text=True,
                              timeout=5).stdout.strip() == 'yes'
    except (OSError, subprocess.SubprocessError):
        return False


def master_running():
    """the thread of the IgH master: EtherCAT-IDLE with no application, EtherCAT-OP with one"""
    return subprocess.run(['pgrep', '-x', 'EtherCAT-(IDLE|OP)'], capture_output=True).returncode == 0


def r5_missing(sys_root='/sys', root=None):
    """--arch S1, S2: what of the R5 side is not there (rpu_work/README_S1.md); root: this user is root (r5rec.py then
    reads the record of the R5 through /dev/mem)"""
    missing = []
    try:
        if open(os.path.join(sys_root, 'class/remoteproc/remoteproc0/state')).read().strip() != 'running':
            missing.append('the R5 is not running (remoteproc0; its bus must be in OP)')
    except OSError:
        missing.append('no remoteproc0: is this the device tree of the R5?')
    names = []
    for path in glob.glob(os.path.join(sys_root, 'class/uio/uio*/name')):
        try:
            names.append(open(path).read().strip())
        except OSError:
            pass
    if 'ocm-link' not in names:
        missing.append('no UIO device ocm-link: the slots of the R5 (the device tree of S1)')
    if 'r5-record' not in names and not (os.geteuid() == 0 if root is None else root):
        missing.append('no UIO device r5-record: the record of the cycles of the R5 (rpu_work/r5_record_uio.dtso)')
    if not os.path.isfile(R5REC):
        missing.append('no %s, which takes that record' % R5REC)
    return missing


def start_r5rec(out):
    """--arch S1, S2: r5rec.py takes the record of every cycle of the R5 from now on, into OUT/r5 (not OUT itself, where
    the timing log of the controller is found by its name)"""
    p = subprocess.Popen([sys.executable, R5REC, '--out', os.path.join(out, 'r5')],
                         stdout=open(os.path.join(out, 'r5rec.txt'), 'w'), stderr=subprocess.STDOUT)
    time.sleep(1.0)
    if p.poll() is not None:
        raise RuntimeError('r5rec.py ended at once: %s' % open(os.path.join(out, 'r5rec.txt')).read().strip())
    return p


def stop_r5rec(p, out):
    """ends r5rec.py as Ctrl-C does -> the log it wrote, or None"""
    if p.poll() is None:
        p.send_signal(signal.SIGTERM)
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()
    for line in open(os.path.join(out, 'r5rec.txt')).read().splitlines():
        print('run_timing: ' + line)
    logs = sorted(glob.glob(os.path.join(out, 'r5', 'timing_r5_*.bin')))
    return logs[-1] if logs else None


def checks(a, rt_cpu):
    try:
        rt = open('/sys/kernel/realtime').read().strip() == '1'
    except OSError:
        rt = False
    if not rt:
        warn('not a PREEMPT_RT kernel (Table 3)')
    m = re.search(r'\bisolcpus=(\S+)', open('/proc/cmdline').read())
    if not m or rt_cpu not in cpu_list(m.group(1)):
        warn('CPU %d of the real-time tasks is not in isolcpus= of the kernel command line (Table 3)' % rt_cpu)
    try:
        if open('/proc/sys/kernel/timer_migration').read().strip() != '0':
            warn('kernel.timer_migration is not 0 (Table 3: sudo sysctl kernel.timer_migration=0)')
    except OSError:
        pass
    if ntp_active():
        warn('NTP is active (Table 3: stopped during the measurements; sudo timedatectl set-ntp false)')
    if a.arch != 'P0':
        for text in r5_missing():
            warn(text)
    elif not master_running():
        warn('no EtherCAT-IDLE or EtherCAT-OP thread: is the IgH master running (ethercat.service)?')
    if not os.path.exists(os.path.join(APP, 'bin', 'indy7_ctrl')):
        sys.exit('run_timing: indy7/bin/indy7_ctrl is missing: make')


def cfg_copy(src, dst, out, duration, hl_log=True):
    """the cfg of the run: the test values set, the relative paths of the source made absolute"""
    base = os.path.dirname(os.path.abspath(src))
    sets = {'HL_TEST_REF_S': '%g' % duration, 'VIEW_PORT': '0', 'JOINT_TEST_LOG_DIR': os.path.join(out, 'hl_logs')}
    if not hl_log:
        sets['HL_LOG'] = '0'
    lines, done = [], set()
    for line in open(src).read().split('\n'):
        m = re.match(r'^([A-Za-z0-9_]+)(\s*=\s*)(.*)$', line)
        if m and m.group(1) in sets and m.group(1) not in done:
            line = '%s=%s' % (m.group(1), sets[m.group(1)])
            done.add(m.group(1))
        elif m and m.group(1) in PATH_KEYS:
            v = m.group(3).split(';', 1)[0].strip()
            if v and not os.path.isabs(v):
                line = '%s=%s' % (m.group(1), os.path.normpath(os.path.join(base, v)))
        lines.append(line)
    missing = [k for k in sets if k not in done]
    if missing:
        sys.exit('run_timing: %s lacks %s' % (src, ', '.join(missing)))
    with open(dst, 'w') as f:
        f.write('\n'.join(lines))


def vision_missing():
    """the processes of the camera pipeline that do not run"""
    return [p for p in PERCEPTION if subprocess.run(['pgrep', '-f', p], capture_output=True).returncode]


def start_vision(out, method='M1'):
    """the camera pipeline in a process group of its own; None if it runs already (it is then left running)"""
    if not vision_missing():
        print('run_timing: the camera pipeline runs already; it is left running (its method and class are its own)')
        return None
    log = os.path.join(out, 'vision.txt')
    # decision_log: why the detector ran, skipped or rejected each image (A3, FULL; the others write none)
    launch = VISION + ' method:=%s decision_log:=%s' % (shlex.quote(method), shlex.quote(os.path.join(out, 'decisions.csv')))
    p = subprocess.Popen(['bash', '-c', launch], stdout=open(log, 'w'), stderr=subprocess.STDOUT,
                         start_new_session=True, env=dict(os.environ, PYTHONUNBUFFERED='1'))
    for _ in range(150):
        time.sleep(1.0)
        if not vision_missing() and VISION_READY in open(log, errors='replace').read():
            print('run_timing: the camera pipeline started, its detector ready (vision.txt)')
            return p
        if p.poll() is not None:
            break
    missing = vision_missing()
    stop_vision(p)
    raise RuntimeError('the camera pipeline did not start (%s; see vision.txt)'
                       % ('no ' + ', '.join(missing) if missing else 'no "%s"' % VISION_READY))


def stop_vision(p):
    """SIGINT to the launch and its nodes, then SIGTERM, then SIGKILL, until none of them runs"""
    if p is None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        for _ in range(20):
            time.sleep(0.5)
            if p.poll() is not None and len(vision_missing()) == len(PERCEPTION):
                return


def run_app(c, desired_class, duration):
    """the camera application following desired_class for duration s: 'p', its digit in the object menu, HOME, TRACK,
    then 'x'; a RuntimeError if the class is not in view or the application does not start or ends before"""
    c.send('p')
    m = c.expect(r'\[bridge\] classes in view[^\n]*\n((?:[^\n]*\n)*?)  0 cancel[^\n]*', 10)
    sel = re.search(r'^  (\d) %s \(' % re.escape(desired_class), m.group(1), re.M)
    if sel is None:
        c.send('0')
        raise RuntimeError('%s is not in view of the camera (the menu: %s)'
                           % (desired_class, ', '.join(re.findall(r'^  \d (\S+)', m.group(1), re.M)) or 'empty'))
    c.send(sel.group(1))
    c.expect(r'\[HL\] camera application: HOME', 10, fail=r'\[(HL|bridge)\] (refused|no entry)[^\n]*')
    c.expect(r'\[HL\] TRACK: camera application', 120, fail=r'\[HL\] (HOME stopped|TRACK refused|refused)[^\n]*')
    print('run_timing: the camera application follows %s for %.0f s' % (desired_class, duration))
    # nothing ends the TRACK of the application but 'x' (or a fault): one that ends before is a fault of the run
    ended = re.compile(r'\[HL\] (camera application stopped|saved [^\n]*\(app, run)[^\n]*')
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        time.sleep(1.0)
        with c.lock:
            e = ended.search(c.text, c.pos)
        if e is not None:
            raise RuntimeError('the camera application ended after %.0f s: %s'
                               % (duration - (t_end - time.monotonic()), e.group(0)))
        if not c.alive:
            raise RuntimeError('the controller has exited (see the console log)')
    c.send('x')
    print('run_timing: %s' % c.expect(r'\[HL\] camera application stopped[^\n]*', 10).group(0).strip())


def memrate_rate(path):
    """the time-averaged rate [MB/s] that the memrate workers reached (stress-ng --metrics; a worker runs every
    method once over the same buffer per round: the harmonic mean of its method rates), summed over the workers"""
    per = {}
    try:
        for pid, rate in re.findall(r'\[(\d+)\] stress-ng-memrate:\s+\w+:\s+([0-9.]+) MB/sec', open(path).read()):
            per.setdefault(pid, []).append(float(rate))
    except OSError:
        return None
    rates = [len(r) / sum(1.0 / x for x in r) for r in per.values() if min(r) > 0]
    return sum(rates) if rates else None


def read_value(path):
    try:
        return float(open(path).read())
    except (OSError, ValueError):
        return float('nan')


def read_sensors():
    """the PS temperatures, LPD and FPD [C], (raw + offset) * scale / 1000 (AMD: Zynq UltraScale+ MPSoC AMS), and the
    board power [W] (INA260). Not the PL temperature: AMD warns that reading it while the PL is being initialized
    can lock the system."""
    v = [(read_value(b + '_raw') + read_value(b + '_offset')) * read_value(b + '_scale') / 1000.0
         for b in (os.path.join(AMS, 'in_temp0_ps_temp'), os.path.join(AMS, 'in_temp1_remote_temp'))]
    v.append(read_value(os.path.join(INA, 'in_power2_raw')) * read_value(os.path.join(INA, 'in_power2_scale')) / 1000.0)
    return v


def sample(path, stop):
    """every second until stop is set: t [s] and read_sensors() into path"""
    if all(x != x for x in read_sensors()):
        warn('no temperature log: none of %s, %s reads' % (AMS, INA))
        return
    t0 = time.monotonic()
    with open(path, 'w') as f:
        f.write('t_s,ps_lpd_C,ps_fpd_C,board_W\n')
        while True:
            f.write('%.0f,%.1f,%.1f,%.2f\n' % tuple([time.monotonic() - t0] + read_sensors()))
            f.flush()
            if stop.wait(1.0):
                return


def run_extras(out):
    """the memrate rate reached [MB/s] and the highest PS (LPD) temperature [C] of a run; None if not logged"""
    tmax = None
    try:
        v = [float(line.split(',')[1]) for line in open(os.path.join(out, 'temp_power.csv')).read().split('\n')[1:]
             if line]
        v = [x for x in v if x == x]
        tmax = max(v) if v else None
    except (OSError, ValueError, IndexError):
        pass
    return memrate_rate(os.path.join(out, 'memrate.txt')), tmax


def main():
    sys.stdout.reconfigure(line_buffering=True)     # in order with the output of the programs it runs (run.txt)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--condition', required=True, choices=('idle', 'stress', 'perception', 'pressure'))
    ap.add_argument('--pressure', type=int, choices=(0, 25, 50, 75, 100),
                    help='[%%] of the largest memory rate, with --condition pressure')
    ap.add_argument('--duration', type=float, default=600.0, help='[s] of the goal stream (Table 6: 600)')
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--out', help='results (default ./timing_<condition>_<date>)')
    ap.add_argument('--settle', type=float, default=30.0, help='[s] in OP before the keys')
    ap.add_argument('--others', default=os.environ.get('OTHERS', '0-2'), help='CPUs of the stress-ng load')
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh (no limit check)')
    ap.add_argument('--bus', help='-B FILE of the controller: the slaves of the bus from FILE (the L7N bench)')
    ap.add_argument('--no-hl-log', action='store_true', help='HL_LOG=0: the high-level test not logged')
    ap.add_argument('--arch', default='P0', choices=('P0', 'S1', 'S2'),
                    help='where the EtherCAT task runs (S1: on the R5; S2: the control task there too)')
    ap.add_argument('--method', default='M1', choices=('M1', 'M2', 'A2', 'A3', 'FULL'),
                    help='--method of the controller and method:= of the camera pipeline')
    ap.add_argument('--desired-class', default='', help='the camera application following CLASS, not the stream')
    ap.add_argument('--shadow', action='store_true',
                    help='the control computation a second time: with --arch S1 on the R5, with S2 here')
    a = ap.parse_args()
    if (a.condition == 'pressure') != (a.pressure is not None):
        ap.error('--pressure goes with --condition pressure, and only with it')
    if a.shadow and a.arch == 'P0':
        ap.error('--shadow goes with --arch S1 or S2')
    if a.desired_class and a.condition not in ('perception', 'pressure'):
        ap.error('--desired-class goes with the camera pipeline: --condition perception or pressure')

    src = os.path.abspath(a.cfg)
    rt_cpu = int(float(jp.cfg_get(src, 'TASK0', 'CPU', '3') or 3))
    out = os.path.abspath(a.out or 'timing_%s_%s' % (a.condition, time.strftime('%Y%m%d_%H%M')))
    os.makedirs(os.path.join(out, 'hl_logs'), exist_ok=True)
    checks(a, rt_cpu)
    cfg = os.path.join(out, 'INDY7.cfg')
    cfg_copy(src, cfg, out, a.duration, hl_log=not a.no_hl_log)

    stop = threading.Event()
    sampler = threading.Thread(target=sample, args=(os.path.join(out, 'temp_power.csv'), stop), daemon=True)
    sampler.start()
    vision = load = mem = c = r5rec = r5_log = None
    code = 0
    try:
        if a.condition in ('perception', 'pressure'):
            vision = start_vision(out, a.method)
        elif len(vision_missing()) < len(PERCEPTION):
            warn('the camera pipeline runs: this is not the condition %s' % a.condition)
        if a.condition == 'stress':
            load = subprocess.Popen(['stress-ng', '--taskset', a.others] + STRESS_ARGS)
            print('run_timing: stress-ng --taskset %s %s' % (a.others, ' '.join(STRESS_ARGS)))
        if a.condition == 'pressure' and a.pressure:
            cap = MEMRATE_CAP.get(a.pressure)
            margs = MEMRATE_ARGS + (['--memrate-rd-mbs', str(cap), '--memrate-wr-mbs', str(cap)] if cap else [])
            mem = subprocess.Popen(['stress-ng', '--taskset', a.others] + margs,
                                   stdout=open(os.path.join(out, 'memrate.txt'), 'w'), stderr=subprocess.STDOUT)
            print('run_timing: stress-ng --taskset %s %s' % (a.others, ' '.join(margs)))
        if a.condition != 'idle':
            print('run_timing: 120 s warm-up')
            time.sleep(120)
            if any(p is not None and p.poll() is not None for p in (load, mem)):
                raise RuntimeError('stress-ng did not start')
            if a.condition in ('perception', 'pressure') and vision_missing():
                raise RuntimeError('the camera pipeline stopped (no %s)' % ', '.join(vision_missing()))

        cmd = (['./bin/indy7_ctrl'] if a.direct else ['./run.sh']) + ['-f', cfg, '-b', '-t', out]
        if a.bus:
            bus = os.path.join(out, os.path.basename(a.bus))
            shutil.copyfile(os.path.abspath(a.bus), bus)        # the bus of the run, kept beside its log
            cmd += ['-B', bus]
        if a.arch != 'P0':
            cmd += ['--arch', a.arch]
        if a.shadow:
            cmd += ['--shadow']
        if a.method != 'M1':
            cmd += ['--method', a.method]
        if a.arch != 'P0':
            r5rec = start_r5rec(out)
        print('run_timing: %s (in %s)' % (' '.join(cmd), APP))
        c = Console(cmd, APP, dict(os.environ, MALLOC_ARENA_MAX='2'), os.path.join(out, 'console.txt'))
        # in any order: CRobotIndy7::Init() starts the bridge (InitBridge) before it prints B_READY, and the bus
        # reaches OP later still
        ready, fail = [B_READY, r'\[R5\] OP: ' if a.arch != 'P0' else r'\[BUS\] OP: \d+ slaves'], FAIL_START
        if a.condition in ('perception', 'pressure'):
            ready.append(r'\[bridge\] up')
            fail += r'|\[bridge\] cannot start|built without ROS 2'
        c.expect_all(ready, 120, fail=fail)
        print('run_timing: the bus in OP; %.0f s settle' % a.settle)
        time.sleep(a.settle)
        if a.arch == 'S2':
            # the controller of the R5 outlives a run that did not end with q (killed): off first, as the keys below
            # expect it ('r' switches it over)
            with c.lock:
                first = re.search(FIRST_REPORT, c.text)
            if first is None:
                raise RuntimeError('no report of the control task of the R5')
            if first.group(1) == 'on':
                print('run_timing: the controller of the R5 was left on by a run before: off first')
                c.send('r')
                c.expect(r'\(CRobotIndy7\) controller DISABLED', 10)
        c.send('r')
        c.expect(CTRL_ON, 10)
        c.send('g')
        c.expect(GRAV_COMP, 10)
        for _ in range(2):
            c.send('k')
            if c.expect(r'grav-comp hold (ON|OFF)', 10).group(1) == 'OFF':
                break
        c.send('h')
        c.expect(r'\[ARM\] servo on: every joint in Operation enabled', 10,
                 fail=r'\[ARM\] (refused|the R5 refused the servo on)[^\n]*|SERVO ON NOT COMPLETE[^\n]*')
        if a.desired_class:
            run_app(c, a.desired_class, a.duration)
        else:
            c.send('y')
            c.expect(r'\[HL\] high-level tests', 10)
            c.send('5')
            c.expect(r'\[HL\] reference check: HOME to q_r', 10, fail=r'\[HL\] refused[^\n]*')
            c.expect(r'\[HL\] TRACK: reference check', 120, fail=r'\[HL\] (HOME stopped|TRACK refused)[^\n]*')
            print('run_timing: the goal stream runs for %.0f s' % a.duration)
            m = c.expect(r'\[HL\] reference check (done|ended|stopped)[^\n]*', a.duration + 300)
            print('run_timing: %s' % m.group(0).strip())
            if 'coming to rest' in m.group(0):      # the arm stops at the accepted goal before the servo off
                c.expect(r'\[HL\] (at rest at the accepted goal|not at rest after 20 s|stopped before the arm was at rest)', 30)
        time.sleep(1.0)
        c.send('j')
        c.expect(r'SERVO OFF', 10)
        time.sleep(0.5)
        c.send('q')
    except (RuntimeError, KeyboardInterrupt, OSError) as ex:
        print('run_timing: stopped: %s' % ex)
        code = 1
        if c is not None:
            try:
                c.send('j')
                time.sleep(0.3)
                c.send('q')
            except OSError:
                pass
    if c is not None:
        c.close()
    if r5rec is not None:
        r5_log = stop_r5rec(r5rec, out)
    if mem is not None:
        mem.send_signal(signal.SIGINT)          # stress-ng then reports the rates reached (memrate.txt)
        try:
            mem.wait(timeout=30)
        except subprocess.TimeoutExpired:
            mem.kill()
    if load is not None:
        load.terminate()
        try:
            load.wait(timeout=10)
        except subprocess.TimeoutExpired:
            load.kill()
    if load is not None or mem is not None:
        subprocess.run(['pkill', '-f', '^stress-ng'], capture_output=True)
    stop_vision(vision)
    stop.set()
    sampler.join()
    rate, tmax = run_extras(out)
    if rate is not None:
        print('run_timing: memrate reached %.0f MB/s (time-averaged, the 3 workers)' % rate)
    if tmax is not None:
        print('run_timing: the PS temperature (LPD) at most %.1f C (temp_power.csv)' % tmax)
    if c is None:
        return 1
    with c.lock:
        for line in c.text.splitlines():
            if line.startswith(('[RT]', '[BUS]', '[SIM]', '[R5]', '[SHADOW]', '[S2]', '[OBS]', '[bridge] ')):
                print('run_timing: ' + line)
    logs = sorted(glob.glob(os.path.join(out, 'timing_*.bin')))
    if not logs:
        print('run_timing: no timing log in %s' % out)
        return 1
    if a.arch != 'P0' and r5_log is None:
        print('run_timing: no log of the R5 (r5rec.txt): the summary is without its send')
        code = 1
    r = subprocess.run([sys.executable, os.path.join(HERE, 'timing_summary.py'), logs[-1], '--out', out] +
                       (['--r5', r5_log] if r5_log else []))
    return code or r.returncode


if __name__ == '__main__':
    sys.exit(main())
