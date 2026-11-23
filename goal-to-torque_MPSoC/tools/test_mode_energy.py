"""pytest: tools/mode_energy.py on stand-in runs of run_timing."""
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mode_energy as E  # noqa: E402


def run_dir(d, watts, runs_per_s=0.0, seconds=200):
    """a run from unix 1000 - 50: power `watts` every second (10 W in the first 30 s, the start), decisions at
    runs_per_s from unix 1000 to its end"""
    d.mkdir(parents=True)
    rows = ['t_s,ps_lpd_C,ps_fpd_C,board_W,unix_s']
    rows += ['%d,40.0,40.0,%.2f,%.3f' % (t, 10.0 if t < 30 else watts, 950.0 + t) for t in range(seconds + 50)]
    (d / 'temp_power.csv').write_text('\n'.join(rows) + '\n')
    lines = ['capture_s,release_s,action']
    if runs_per_s > 0:
        n = int(seconds * runs_per_s)
        lines += ['%.3f,%.3f,%s' % (1000 + i / runs_per_s, 1000.05 + i / runs_per_s, 'run') for i in range(n)]
        lines += ['1000.0,1000.0,reject']
    (d / 'decisions.csv').write_text('\n'.join(lines) + '\n')
    return str(d)


def test_the_energy_is_the_power_above_the_base_per_inference(tmp_path):
    base = run_dir(tmp_path / 'base', 7.0)
    m416 = run_dir(tmp_path / 'm416', 7.6, runs_per_s=12.0)
    assert E.mean_power(base, 60, 10) == (pytest.approx(7.0), 180)
    rate, k, span = E.inference_rate(m416)
    assert k == 2400 and rate == pytest.approx(12.0, rel=1e-4)       # stamps in ms
    assert span[0] == pytest.approx(1000.05) and E.mean_power(m416, 0, 5, span)[1] == 189
    assert E.energy(7.0, 7.6, 12.0) == pytest.approx(0.05)
    out = subprocess.run([sys.executable, '-B', os.path.join(HERE, 'mode_energy.py'), '--base', base, '0:' + m416],
                         capture_output=True, text=True, check=True).stdout
    assert 'energy_j 0.0500' in out and '0: energy_j: 0.0500' in out


def test_without_a_base_run_the_time_before_the_first_inference_is_the_base(tmp_path):
    m416 = run_dir(tmp_path / 'm416', 7.6, runs_per_s=12.0)
    path = os.path.join(m416, 'decisions.csv')
    with open(path, 'a') as f:                       # rejected images from unix 960 on: the pipeline is up
        f.write(''.join('%.3f,%.3f,reject\n' % (960 + i / 30, 960.05 + i / 30) for i in range(1200)))
    rows = ['t_s,ps_lpd_C,ps_fpd_C,board_W,unix_s']
    rows += ['%d,40.0,40.0,%.2f,%.3f' % (t, 7.0 if 950 + t < 1000 else 7.6, 950.0 + t) for t in range(250)]
    (tmp_path / 'm416' / 'temp_power.csv').write_text('\n'.join(rows) + '\n')
    assert E.before_first(m416) == (pytest.approx(965.05), pytest.approx(998.05))
    out = subprocess.run([sys.executable, '-B', os.path.join(HERE, 'mode_energy.py'), '0:' + m416],
                         capture_output=True, text=True, check=True).stdout
    assert 'before its first inference: 7.000 W' in out and 'energy_j 0.0500' in out


def test_runs_that_cannot_give_a_number_are_refused(tmp_path):
    base = run_dir(tmp_path / 'base', 7.0)
    with pytest.raises(ValueError, match='too few'):
        E.inference_rate(base)                                   # no image run
    with pytest.raises(ValueError, match='fewer than 10'):
        E.mean_power(run_dir(tmp_path / 'short', 7.0, seconds=15), 60, 10)
    with pytest.raises(ValueError):
        E.mean_power(str(tmp_path / 'none'), 60, 10)
    res = subprocess.run([sys.executable, '-B', os.path.join(HERE, 'mode_energy.py'), '--base', base, 'x:' + base],
                         capture_output=True, text=True)
    assert res.returncode != 0 and 'Traceback' not in res.stderr


def apm_log(d, mb_per_s, seconds=250, before_mb_per_s=None):
    """samples every 0.25 s from unix 950 (as run_dir): mb_per_s from unix 1000 on, before_mb_per_s before it"""
    rows = ['unix_s,dt_s,s1_rd_B,s1_wr_B']
    for i in range(seconds * 4):
        u = 950.0 + 0.25 * (i + 1)
        r = mb_per_s if u > 1000.0 or before_mb_per_s is None else before_mb_per_s
        rows.append('%.3f,0.25,%d,%d' % (u, r * 1e6 * 0.25 / 2, r * 1e6 * 0.25 / 2))
    (d / 'apm.csv').write_text('\n'.join(rows) + '\n')


def test_the_ddr_bytes_of_an_inference_with_the_monitor_logs(tmp_path):
    base = run_dir(tmp_path / 'base', 7.0)
    m416 = run_dir(tmp_path / 'm416', 7.6, runs_per_s=12.0)
    assert E.ddr_rate(base, 60, 10) is None
    apm_log(tmp_path / 'base', 400)
    apm_log(tmp_path / 'm416', 520, before_mb_per_s=400)            # before the first decision: no inference
    assert E.ddr_rate(base, 60, 10) == pytest.approx(400e6)
    span = E.inference_rate(m416)[2]
    assert E.ddr_rate(m416, 60, 10, span) == pytest.approx(520e6)     # the time of the inferences only
    out = subprocess.run([sys.executable, '-B', os.path.join(HERE, 'mode_energy.py'), '--base', base, '0:' + m416],
                         capture_output=True, text=True, check=True).stdout
    value = out.split('0: ddr_bytes: ')[1].split()[0]                       # a plain float for YAML
    assert 'e' not in value and float(value) == pytest.approx(1e7, rel=1e-4)  # 120 MB/s over 12 a second


def test_the_selectors_and_the_wrap_of_the_ddr_monitor():
    sys.path.insert(0, os.path.join(HERE, 'host'))
    import ddr_apm as A
    words = A.selectors(A.COUNTERS)
    # counter 0: S1 read bytes (slot 1, metric 3) -> 0x23; counter 1: S1 write bytes -> 0x22
    assert words[0] & 0xFFFF == 0x2223 and (words[2] >> 8) & 0xFF == (5 << 5 | A.WRITE_BYTES)
    assert len(A.NAMES) == 10 and A.NAMES[0] == 's1_rd_B' and A.NAMES[-1] == 's5_wr_B'
    assert A.increase(5, 0xFFFFFFFE) == 7 and A.increase(10, 3) == 7


def test_the_ddr_monitor_puts_its_registers_back(tmp_path):
    sys.path.insert(0, os.path.join(HERE, 'host'))
    import ddr_apm as A

    class Fake:
        def __init__(self):
            self.r = {A.CR: 0x0, A.MSR[0]: 0x11, A.MSR[1]: 0x22, A.MSR[2]: 0x33}
            self.reads = 0

        def read(self, off):
            if A.MCR0 <= off < A.MCR0 + A.MCR_STEP * len(A.COUNTERS):
                self.reads += 1
                return (self.reads * 1000) & 0xFFFFFFFF
            return self.r.get(off, 0)

        def write(self, off, v):
            self.r[off] = v

    regs = Fake()
    total, seconds = A.record(regs, str(tmp_path / 'apm.csv'), 0.05, 0.2, [])
    assert regs.r == {A.CR: 0x0, A.MSR[0]: 0x11, A.MSR[1]: 0x22, A.MSR[2]: 0x33}
    assert seconds > 0 and all(t > 0 for t in total)
    assert open(tmp_path / 'apm.csv').readline().strip().endswith('s5_wr_B')


def test_the_ddr_monitor_records_one_at_a_time(tmp_path):
    sys.path.insert(0, os.path.join(HERE, 'host'))
    import ddr_apm as A
    first = A.take_lock(str(tmp_path / 'l'))
    assert first is not None and A.take_lock(str(tmp_path / 'l')) is None
    first.close()
    assert A.take_lock(str(tmp_path / 'l')) is not None


def test_the_ddr_monitor_without_dev_mem_leaves_no_lock_file(tmp_path, monkeypatch, capsys):
    sys.path.insert(0, os.path.join(HERE, 'host'))
    import ddr_apm as A
    lock = tmp_path / 'l'
    monkeypatch.setattr(A, 'LOCK', str(lock))
    monkeypatch.setattr(sys, 'argv', ['ddr_apm', '--out', str(tmp_path / 'apm.csv'), '--duration', '1'])

    def no_mem(base, write):
        raise PermissionError(13, 'Permission denied')
    monkeypatch.setattr(A, 'Regs', no_mem)
    with pytest.raises(SystemExit) as ex:
        A.main()
    assert '/dev/mem' in str(ex.value) and not lock.exists()


def test_the_ddr_monitor_names_a_lock_file_it_cannot_open(tmp_path, monkeypatch):
    sys.path.insert(0, os.path.join(HERE, 'host'))
    import ddr_apm as A
    monkeypatch.setattr(A, 'LOCK', str(tmp_path / 'no_dir' / 'l'))
    monkeypatch.setattr(sys, 'argv', ['ddr_apm', '--out', str(tmp_path / 'apm.csv'), '--duration', '1'])
    monkeypatch.setattr(A, 'Regs', lambda base, write: object())
    with pytest.raises(SystemExit) as ex:
        A.main()
    assert 'the lock file' in str(ex.value)
