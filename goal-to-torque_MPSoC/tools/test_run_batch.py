"""pytest: the architecture (--arch) of tools/run_batch.py and tools/run_timing.py.

  python3 -m pytest tools/test_run_batch.py
"""
import os
import sys

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
    assert rt.r5_missing(fake_sys(str(tmp_path), 'running', ['axi-pmon', 'ocm-link'])) == []


def test_r5_not_running_or_no_slots(tmp_path):
    stopped = rt.r5_missing(fake_sys(str(tmp_path / 'a'), 'offline', ['ocm-link']))
    assert len(stopped) == 1 and 'not running' in stopped[0]
    no_node = rt.r5_missing(fake_sys(str(tmp_path / 'b'), 'running', ['axi-pmon']))
    assert len(no_node) == 1 and 'ocm-link' in no_node[0]
    neither = rt.r5_missing(fake_sys(str(tmp_path / 'c'), None, []))
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
