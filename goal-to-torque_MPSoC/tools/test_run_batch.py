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


def test_p0_command_as_before(tmp_path, monkeypatch):
    assert '--arch' not in run_one(tmp_path, monkeypatch, [])


def test_s1_batch_needs_the_r5(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(rb.rt, 'r5_missing', lambda: ['the R5 is not running'])
    monkeypatch.setattr(rb.rt, 'master_running', lambda: True)
    monkeypatch.setattr(rb.rt, 'ntp_active', lambda: False)
    monkeypatch.setattr(rb, 'LEFTOVERS', ())
    bus = tmp_path / 'bus.cfg'
    bus.write_text('')
    monkeypatch.setattr(sys, 'argv', ['run_batch.py', '--out', str(tmp_path / 'out'), '--quick', '--bus', str(bus),
                                      '--arch', 'S1'])
    try:
        rb.main()
        stopped = None
    except SystemExit as ex:
        stopped = str(ex)
    assert stopped is not None and 'the R5 is not running' in stopped


class FakeConsole:
    """in place of the controller: keeps its command, has printed its start, and is gone at the first wait"""
    made = []

    def __init__(self, cmd, cwd, env, log_path):
        self.cmd, self.lock = cmd, threading.Lock()
        self.text = ('[R5] OP: the bus of the R5\n[SHADOW] the control task tells the R5 every period\n'
                     'another line\n')
        FakeConsole.made.append(self)

    def expect_all(self, patterns, timeout, fail=None):
        raise RuntimeError('the controller has exited (see the console log)')

    def send(self, key):
        pass

    def close(self):
        pass


def timing_run(tmp_path, monkeypatch, args):
    FakeConsole.made = []
    monkeypatch.setattr(rt, 'Console', FakeConsole)
    monkeypatch.setattr(rt, 'checks', lambda a, cpu: None)
    monkeypatch.setattr(rt, 'cfg_copy', lambda *a, **k: None)
    monkeypatch.setattr(rt, 'sample', lambda path, stop: None)
    monkeypatch.setattr(rt, 'start_r5rec', lambda out: None)
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
