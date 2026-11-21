"""pytest: the architecture (--arch) of tools/run_batch.py and tools/run_timing.py, and --shadow of the latter.

  python3 -m pytest tools/test_run_batch.py
"""
import os
import sys
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_batch as rb  # noqa: E402
import run_timing as rt  # noqa: E402

# in place of run_timing.py: leaves its arguments in the directory of the run
FAKE_RUN = ("import os, sys\n"
            "out = sys.argv[sys.argv.index('--out') + 1]\n"
            "os.makedirs(out, exist_ok=True)\n"
            "open(os.path.join(out, 'argv.txt'), 'w').write(' '.join(sys.argv[1:]))\n")


@pytest.fixture(autouse=True)
def no_pl_qos_wrapper(tmp_path, monkeypatch):
    # on a board with tools/host/install_pl_qos.sh done, the runs here must not call the real wrapper
    monkeypatch.setattr(rt, 'PL_QOS', str(tmp_path / 'no-pl-qos-wrapper'))


def fake_sys(root, state, uio_names):
    os.makedirs(os.path.join(root, 'class/remoteproc/remoteproc0'))
    if state is not None:
        open(os.path.join(root, 'class/remoteproc/remoteproc0/state'), 'w').write(state + '\n')
    for n, name in enumerate(uio_names):
        os.makedirs(os.path.join(root, 'class/uio/uio%d' % n))
        open(os.path.join(root, 'class/uio/uio%d/name' % n), 'w').write(name + '\n')
    return root


def test_r5_side_there(tmp_path):
    assert rt.r5_missing(fake_sys(str(tmp_path), 'running', ['axi-pmon', 'ocm-link', 'r5-record']), root=False) == []


def test_the_record_of_the_r5_needs_its_uio_device_unless_root(tmp_path):
    sys_root = fake_sys(str(tmp_path), 'running', ['axi-pmon', 'ocm-link'])
    as_user = rt.r5_missing(sys_root, root=False)
    assert len(as_user) == 1 and 'r5-record' in as_user[0]
    assert rt.r5_missing(sys_root, root=True) == []          # /dev/mem


def test_the_run_takes_the_record_of_the_r5_and_hands_its_log_to_the_summary(tmp_path, monkeypatch, capsys):
    # in place of r5rec.py: writes a log into --out and, as r5rec.py, ends at SIGTERM with a line about it
    fake = tmp_path / 'r5rec.py'
    fake.write_text("import os, signal, sys, time\n"
                    "out = sys.argv[sys.argv.index('--out') + 1]\n"
                    "os.makedirs(out, exist_ok=True)\n"
                    "open(os.path.join(out, 'timing_r5_1.bin'), 'w').write('x')\n"
                    "def end(*_):\n"
                    "    print('r5rec: 5 cycles written', flush=True)\n"
                    "    sys.exit(0)\n"
                    "signal.signal(signal.SIGTERM, end)\n"
                    "time.sleep(60)\n")
    monkeypatch.setattr(rt, 'R5REC', str(fake))
    out = str(tmp_path)
    p = rt.start_r5rec(out)
    assert p.poll() is None
    assert rt.stop_r5rec(p, out) == os.path.join(out, 'r5', 'timing_r5_1.bin')
    assert 'run_timing: r5rec: 5 cycles written' in capsys.readouterr().out
    # one that cannot read the ring ends at once: the run stops there, with what it said
    fake.write_text("import sys\nsys.exit('r5rec: /dev/mem: Permission denied (sudo, or the UIO device)')\n")
    try:
        rt.start_r5rec(out)
        stopped = None
    except RuntimeError as ex:
        stopped = str(ex)
    assert stopped is not None and 'Permission denied' in stopped


def test_r5_not_running_or_no_slots(tmp_path):
    stopped = rt.r5_missing(fake_sys(str(tmp_path / 'a'), 'offline', ['ocm-link', 'r5-record']))
    assert len(stopped) == 1 and 'not running' in stopped[0]
    no_node = rt.r5_missing(fake_sys(str(tmp_path / 'b'), 'running', ['axi-pmon', 'r5-record']))
    assert len(no_node) == 1 and 'ocm-link' in no_node[0]
    neither = rt.r5_missing(fake_sys(str(tmp_path / 'c'), None, []), root=True)
    assert len(neither) == 2 and 'remoteproc0' in neither[0]


def run_one(tmp_path, monkeypatch, arch_args):
    tools = tmp_path / 'tools'
    tools.mkdir()
    (tools / 'run_timing.py').write_text(FAKE_RUN)
    bus = tmp_path / 'bus.cfg'
    bus.write_text('')
    monkeypatch.setattr(rb, 'HERE', str(tools))
    monkeypatch.setattr(rb, 'QUICK', [('idle', None)])
    monkeypatch.setattr(rb, 'LEFTOVERS', ())
    monkeypatch.setattr(rb.rt, 'r5_missing', lambda: [])
    monkeypatch.setattr(rb.rt, 'master_running', lambda: True)
    monkeypatch.setattr(rb.rt, 'ntp_active', lambda: False)
    monkeypatch.setattr(rb.time, 'sleep', lambda s: None)
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--quick', '--bus', str(bus)]
                        + arch_args)
    assert rb.main() == 0
    return (tmp_path / 'out' / '01_idle' / 'argv.txt').read_text().split()


def test_arch_reaches_the_run(tmp_path, monkeypatch):
    argv = run_one(tmp_path, monkeypatch, ['--arch', 'S1'])
    assert argv[argv.index('--arch') + 1] == 'S1'


def test_s2_reaches_the_run_with_the_stream(tmp_path, monkeypatch):
    argv = run_one(tmp_path, monkeypatch, ['--arch', 'S2'])
    assert argv[argv.index('--arch') + 1] == 'S2'
    assert '--method' not in argv and '--desired-class' not in argv      # M1 and the goal stream, as P0 and S1


def test_p0_command_as_before(tmp_path, monkeypatch):
    assert '--arch' not in run_one(tmp_path, monkeypatch, [])


@pytest.mark.parametrize('arch', ['S1', 'S2'])
def test_r5_batch_needs_the_r5(tmp_path, monkeypatch, capsys, arch):
    monkeypatch.setattr(rb.rt, 'r5_missing', lambda: ['the R5 is not running'])
    monkeypatch.setattr(rb.rt, 'master_running', lambda: True)
    monkeypatch.setattr(rb.rt, 'ntp_active', lambda: False)
    monkeypatch.setattr(rb, 'LEFTOVERS', ())
    bus = tmp_path / 'bus.cfg'
    bus.write_text('')
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--quick', '--bus', str(bus),
                                      '--arch', arch])
    try:
        rb.main()
        stopped = None
    except SystemExit as ex:
        stopped = str(ex)
    assert stopped is not None and 'the R5 is not running' in stopped


class FakeConsole:
    """in place of the controller: keeps its command and what the run waits for first, has printed its start, and
    is gone at that wait; records: the directories for which the record of the R5 was started"""
    made = []
    records = []
    first = None    # with it the controller gets through its start: the line of its first report of the R5 ...
    waits = 0       # ... and then answers this many waits

    def __init__(self, cmd, cwd, env, log_path):
        self.cmd, self.lock, self.keys, self.waited = cmd, threading.Lock(), [], []
        self.text = ('[R5] OP: the bus of the R5\n[SHADOW] the control task tells the R5 every period\n'
                     '[S2] reports of the control task of the R5\nanother line\n')
        FakeConsole.made.append(self)

    def expect_all(self, patterns, timeout, fail=None):
        self.patterns = patterns
        if FakeConsole.first is None:
            raise RuntimeError('the controller has exited (see the console log)')
        self.text += FakeConsole.first

    def expect(self, pattern, timeout, fail=None):
        self.waited.append(pattern)
        if len(self.waited) > FakeConsole.waits:
            raise RuntimeError('the controller has exited (see the console log)')

    def send(self, key):
        self.keys.append(key)

    def close(self):
        pass


def timing_run(tmp_path, monkeypatch, args, first=None, waits=0):
    FakeConsole.made, FakeConsole.records = [], []
    monkeypatch.setattr(FakeConsole, 'first', first)
    monkeypatch.setattr(FakeConsole, 'waits', waits)
    monkeypatch.setattr(rt, 'Console', FakeConsole)
    monkeypatch.setattr(rt, 'checks', lambda a, cpu: None)
    monkeypatch.setattr(rt, 'cfg_copy', lambda *a, **k: None)
    monkeypatch.setattr(rt, 'sample', lambda path, stop: None)
    monkeypatch.setattr(rt, 'start_r5rec', lambda out: FakeConsole.records.append(out))
    monkeypatch.setattr(rt, 'vision_missing', lambda: list(rt.PERCEPTION))
    monkeypatch.setattr(rt, 'run_extras', lambda out: (None, None))
    monkeypatch.setattr(rt.time, 'sleep', lambda s: None)
    monkeypatch.setattr(sys, 'argv', ['run_timing.py', '--condition', 'idle', '--out', str(tmp_path / 'out')] + args)
    return rt.main()


def test_shadow_reaches_the_controller_and_its_lines_the_output_of_the_run(tmp_path, monkeypatch, capsys):
    assert timing_run(tmp_path, monkeypatch, ['--arch', 'S1', '--shadow']) == 1      # the fake wrote no timing log
    cmd = FakeConsole.made[0].cmd
    assert cmd[cmd.index('--arch'):] == ['--arch', 'S1', '--shadow']
    printed = capsys.readouterr().out
    assert 'run_timing: [SHADOW] the control task tells the R5 every period' in printed
    assert 'another line' not in printed
    timing_run(tmp_path, monkeypatch, ['--arch', 'S1'])
    assert '--shadow' not in FakeConsole.made[0].cmd


def test_shadow_goes_with_the_ethercat_task_on_the_r5_only(tmp_path, monkeypatch, capsys):
    with pytest.raises(SystemExit):
        timing_run(tmp_path, monkeypatch, ['--shadow'])
    assert '--shadow goes with --arch S1' in capsys.readouterr().err
    assert FakeConsole.made == [] and not (tmp_path / 'out').exists()


def test_s2_the_run_takes_the_record_of_the_r5_and_waits_for_its_bus(tmp_path, monkeypatch, capsys):
    assert timing_run(tmp_path, monkeypatch, ['--arch', 'S2', '--shadow']) == 1      # the fake wrote no timing log
    run = FakeConsole.made[0]
    assert run.cmd[run.cmd.index('--arch'):] == ['--arch', 'S2', '--shadow']
    assert FakeConsole.records == [str(tmp_path / 'out')] and run.patterns[1] == r'\[R5\] OP: '
    assert 'run_timing: [S2] reports of the control task of the R5' in capsys.readouterr().out
    # the EtherCAT task here: no record of an R5, and the bus is that of the master
    timing_run(tmp_path, monkeypatch, [])
    assert FakeConsole.records == [] and 'BUS' in FakeConsole.made[0].patterns[1]
    assert '--arch' not in FakeConsole.made[0].cmd


REPORTS = '[R5] the control task of the R5 reports (its period 5120): controller %s, the servo not asked, no motion\n'


def test_s2_a_controller_that_a_killed_run_left_on_is_switched_off_before_the_keys(tmp_path, monkeypatch, capsys):
    # the R5 as a run that ended with q leaves it: the keys start with r, which switches its controller on
    timing_run(tmp_path, monkeypatch, ['--arch', 'S2'], first=REPORTS % 'off', waits=1)
    run = FakeConsole.made[0]
    assert run.keys[:2] == ['r', 'g'] and run.waited[0] == rt.CTRL_ON
    assert 'left on' not in capsys.readouterr().out
    # left on: one r more, which switches it off, and the wait for that
    timing_run(tmp_path, monkeypatch, ['--arch', 'S2'], first=REPORTS % 'on', waits=2)
    run = FakeConsole.made[0]
    assert run.keys[:3] == ['r', 'r', 'g'] and 'DISABLED' in run.waited[0] and run.waited[1] == rt.CTRL_ON
    assert 'run_timing: the controller of the R5 was left on by a run before: off first' in capsys.readouterr().out
    # no report: not the firmware with the control task, or it does not answer
    assert timing_run(tmp_path, monkeypatch, ['--arch', 'S2'], first='another line\n', waits=9) == 1
    assert FakeConsole.made[0].waited == []
    assert 'run_timing: stopped: no report of the control task of the R5' in capsys.readouterr().out
    # with S1 nothing reports: the keys as before
    timing_run(tmp_path, monkeypatch, ['--arch', 'S1'], first='another line\n', waits=1)
    assert FakeConsole.made[0].keys[:2] == ['r', 'g'] and FakeConsole.made[0].waited[0] == rt.CTRL_ON


def test_vision_args_are_launch_arguments_quoted_for_bash():
    assert rt.vision_args("model_path_2:=~/m/x.xmodel 'tag_id:=3'") == ["'model_path_2:=~/m/x.xmodel'", 'tag_id:=3']
    assert rt.vision_args('a:=$(reboot)') == ["'a:=$(reboot)'"]      # quoted: bash runs nothing
    for bad in ('foo', ':=1', '$(x):=1', 'a b:=1'):
        with pytest.raises(ValueError):
            rt.vision_args(bad)


def test_model_args_are_the_files_of_the_models_quoted_for_bash(tmp_path):
    m416, m320 = tmp_path / 'a 416.xmodel', tmp_path / 'b.xmodel'
    m416.write_text('')
    m320.write_text('')
    assert rt.model_args(str(m416)) == ["'model_path:=%s'" % m416]
    assert rt.model_args(str(m416), str(m320))[1] == 'model_path_2:=%s' % m320
    assert rt.model_args('launch') == [] and rt.model_args('launch', str(m320)) == ['model_path_2:=%s' % m320]
    for bad in ((str(tmp_path / 'none.xmodel'),), (str(m416), str(tmp_path))):
        with pytest.raises(ValueError):
            rt.model_args(*bad)


def batch_argv(tmp_path, monkeypatch, conds, args):
    """the arguments of run_timing.py of each run of a batch of conds, with run_one's stand-ins"""
    tools = tmp_path / 'tools'
    tools.mkdir()
    (tools / 'run_timing.py').write_text(FAKE_RUN)
    bus = tmp_path / 'bus.cfg'
    bus.write_text('')
    monkeypatch.setattr(rb, 'HERE', str(tools))
    monkeypatch.setattr(rb, 'QUICK', conds)
    monkeypatch.setattr(rb, 'LEFTOVERS', ())
    monkeypatch.setattr(rb.rt, 'r5_missing', lambda: [])
    monkeypatch.setattr(rb.rt, 'master_running', lambda: True)
    monkeypatch.setattr(rb.rt, 'ntp_active', lambda: False)
    monkeypatch.setattr(rb.time, 'sleep', lambda s: None)
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--quick', '--bus', str(bus)]
                        + args)
    assert rb.main() == 0
    return [(tmp_path / 'out' / rb.name(n, c, p) / 'argv.txt').read_text().split()
            for n, (c, p) in enumerate(conds, 1)]


def test_the_runs_with_the_camera_get_the_model(tmp_path, monkeypatch):
    m = tmp_path / 'v5n_416.xmodel'
    m.write_text('')
    idle, perception, pressure = batch_argv(tmp_path, monkeypatch, [('idle', None), ('perception', None),
                                                                    ('pressure', 100)], ['--model', str(m)])
    assert '--model' not in idle
    for argv in (perception, pressure):
        assert argv[argv.index('--model') + 1] == str(m) and '--model-2' not in argv


def test_a_batch_with_the_camera_stops_before_its_runs_without_the_model(tmp_path, monkeypatch, capsys):
    with pytest.raises(SystemExit) as ex:
        batch_argv(tmp_path, monkeypatch, [('perception', None)], ['--model', str(tmp_path / 'none.xmodel')])
    assert 'no such model file' in str(ex.value)


def test_a_batch_without_the_camera_does_not_need_the_model(tmp_path, monkeypatch):
    (idle,) = batch_argv(tmp_path, monkeypatch, [('idle', None)], ['--model', str(tmp_path / 'none.xmodel')])
    assert '--model' not in idle


def test_the_runs_with_the_camera_get_the_vision_args(tmp_path, monkeypatch):
    idle, pressure = batch_argv(tmp_path, monkeypatch, [('idle', None), ('pressure', 100)],
                                ['--model', 'launch', '--vision-args', 'tool_tag:=true tag_id:=3'])
    assert '--vision-args' not in idle
    i = pressure.index('--vision-args')
    assert pressure[i + 1:i + 3] == ['tool_tag:=true', 'tag_id:=3']


def test_a_batch_with_the_camera_stops_before_its_runs_with_a_word_that_is_no_launch_argument(tmp_path, monkeypatch):
    with pytest.raises(SystemExit) as ex:
        batch_argv(tmp_path, monkeypatch, [('pressure', 100)], ['--model', 'launch', '--vision-args', 'tool_tag'])
    assert 'no name:=value argument' in str(ex.value)


def test_a_batch_with_pl_qos_needs_the_wrapper(tmp_path, monkeypatch):
    with pytest.raises(SystemExit) as ex:
        batch_argv(tmp_path, monkeypatch, [('idle', None)], ['--pl-qos', '9'])
    assert 'install_pl_qos.sh' in str(ex.value)


def test_a_batch_with_pl_qos_restores_after_every_run(tmp_path, monkeypatch):
    # a run killed before its own restore must not leave the ports raised for the next (I-116)
    wrapper = tmp_path / 'pl-qos'
    wrapper.write_text('')
    fake = FakePlQos()
    monkeypatch.setattr(rt, 'PL_QOS', str(wrapper))
    monkeypatch.setattr(rt, 'pl_qos', fake)
    idle, stress = batch_argv(tmp_path, monkeypatch, [('idle', None), ('stress', None)], ['--pl-qos', '9'])
    assert idle[idle.index('--pl-qos') + 1] == '9'
    assert fake.calls == [('show',), ('restore',), ('restore',)]
    assert 'restore (run_batch' in (tmp_path / 'out' / '01_idle' / 'pl_qos.txt').read_text()


class FakePlQos:
    """in place of sudo -n pl-qos: records the calls; show answers SHOW, set answers SET"""
    def __init__(self, show=(0, 'HP1 ... QoS static 0'), set_=(0, 'after HP1 ... QoS static 9')):
        self.calls, self.show, self.set_ = [], show, set_

    def __call__(self, *args):
        self.calls.append(args)
        return {'show': self.show, 'set': self.set_}.get(args[0], (0, 'restored'))


def qos_run(tmp_path, monkeypatch, args, fake):
    wrapper = tmp_path / 'pl-qos'
    wrapper.write_text('')
    monkeypatch.setattr(rt, 'PL_QOS', str(wrapper))
    monkeypatch.setattr(rt, 'pl_qos', fake)
    return timing_run(tmp_path, monkeypatch, args)


def test_pl_qos_is_set_before_the_run_and_restored_once_after(tmp_path, monkeypatch, capsys):
    fake = FakePlQos()
    assert qos_run(tmp_path, monkeypatch, ['--pl-qos', '9'], fake) == 1      # the fake controller exits at once
    assert fake.calls == [('show',), ('set', '9'), ('restore',)]
    out = capsys.readouterr().out
    assert 'QoS of HPC0-1 and HP0-3 at 9' in out and 'back to their values before' in out
    record = (tmp_path / 'out' / 'pl_qos.txt').read_text()
    assert 'pl-qos show' in record and 'pl-qos set 9' in record and 'pl-qos restore' in record


def test_without_pl_qos_the_run_records_the_ports_and_sets_nothing(tmp_path, monkeypatch):
    fake = FakePlQos()
    qos_run(tmp_path, monkeypatch, [], fake)
    assert fake.calls == [('show',)] and 'QoS static 0' in (tmp_path / 'out' / 'pl_qos.txt').read_text()


def test_ports_left_raised_by_a_run_before_stop_the_run(tmp_path, monkeypatch):
    fake = FakePlQos(show=(0, 'HP1 ... QoS static 9\npl-qos: raised since 2026-11-21 18:00:00 and not restored'))
    with pytest.raises(SystemExit) as ex:
        qos_run(tmp_path, monkeypatch, [], fake)
    assert 'left raised' in str(ex.value) and FakeConsole.made == [] and fake.calls == [('show',)]


def test_pl_qos_refused_or_failing(tmp_path, monkeypatch, capsys):
    with pytest.raises(SystemExit) as ex:                          # no password-less sudo for it
        qos_run(tmp_path, monkeypatch, ['--pl-qos', '9'], FakePlQos(show=(1, 'sudo: a password is required')))
    assert 'a password is required' in str(ex.value) and FakeConsole.made == []
    fake = FakePlQos(set_=(1, 'hp_qos: 1 value(s) did not read back as written'))
    assert qos_run(tmp_path, monkeypatch, ['--pl-qos', '9'], fake) == 1
    # a set that fails may have saved and written some ports: the run restores what was saved (I-116)
    assert fake.calls == [('show',), ('set', '9'), ('restore',)]
    assert 'stopped: pl-qos set 9 failed' in capsys.readouterr().out and FakeConsole.made == []
    with pytest.raises(SystemExit):
        qos_run(tmp_path, monkeypatch, ['--pl-qos', '16'], FakePlQos())


def test_the_memory_pressure_by_its_percent_or_by_its_bound_per_worker():
    assert rt.memrate_args(100) == rt.MEMRATE_ARGS
    assert rt.memrate_args(75)[-4:] == ['--memrate-rd-mbs', '800', '--memrate-wr-mbs', '800']
    assert rt.memrate_args(None, 1200)[-4:] == ['--memrate-rd-mbs', '1200', '--memrate-wr-mbs', '1200']


@pytest.mark.parametrize('args', [['--memrate-mbs', '800'],                               # not with idle
                                  ['--condition', 'pressure', '--pressure', '75', '--memrate-mbs', '800'],
                                  ['--condition', 'pressure', '--memrate-mbs', '0']])
def test_memrate_mbs_goes_with_the_pressure_alone_and_in_its_range(tmp_path, monkeypatch, capsys, args):
    with pytest.raises(SystemExit):
        timing_run(tmp_path, monkeypatch, args)
    assert '--memrate-mbs' in capsys.readouterr().err


def test_a_quick_batch_with_memrate_mbs_bounds_its_pressure_runs(tmp_path, monkeypatch):
    conds = [('idle', None), ('pressure', 100)]
    tools = tmp_path / 'tools'
    tools.mkdir()
    (tools / 'run_timing.py').write_text(FAKE_RUN)
    bus = tmp_path / 'bus.cfg'
    bus.write_text('')
    monkeypatch.setattr(rb, 'HERE', str(tools))
    monkeypatch.setattr(rb, 'QUICK', conds)
    monkeypatch.setattr(rb, 'LEFTOVERS', ())
    monkeypatch.setattr(rb.rt, 'r5_missing', lambda: [])
    monkeypatch.setattr(rb.rt, 'master_running', lambda: True)
    monkeypatch.setattr(rb.rt, 'ntp_active', lambda: False)
    monkeypatch.setattr(rb.time, 'sleep', lambda s: None)
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--quick', '--bus', str(bus),
                                      '--model', 'launch', '--memrate-mbs', '800'])
    assert rb.main() == 0
    idle = (tmp_path / 'out' / '01_idle' / 'argv.txt').read_text().split()
    pressure = (tmp_path / 'out' / '02_pressure800MBps' / 'argv.txt').read_text().split()
    assert '--memrate-mbs' not in idle and '--pressure' not in idle
    assert pressure[pressure.index('--memrate-mbs') + 1] == '800' and '--pressure' not in pressure
    rows = (tmp_path / 'out' / 'batch.csv').read_text().splitlines()
    assert rows[2].split(',')[3] == '800 MB/s'


def test_memrate_mbs_goes_with_a_quick_batch(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--memrate-mbs', '800'])
    with pytest.raises(SystemExit):
        rb.main()
