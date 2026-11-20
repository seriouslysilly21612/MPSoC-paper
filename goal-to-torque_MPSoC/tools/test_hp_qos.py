"""pytest: tools/host/hp_qos.py on stand-in registers (no /dev/mem)."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'host'))
import hp_qos as Q  # noqa: E402

RESET = {Q.RDCTRL: 0x000003B0, Q.RDISSUE: 0xF, Q.RDQOS: 0x7, Q.WRCTRL: 0x000003B0, Q.WRISSUE: 0xF,
         Q.WRQOS: 0x0}


class FakeMem:
    def __init__(self):
        self.pages = {}
        for name in Q.PORTS:
            w = memoryview(bytearray(Q.PAGE)).cast('I')
            for off, v in RESET.items():
                w[off // 4] = v
            w[0x100 // 4] = 0xDEADBEEF          # a register the tool must not touch
            self.pages[name] = w

    def port(self, name):
        return Q.Port(self.pages[name])


def test_read_as_it_is(capsys):
    mem = FakeMem()
    v = mem.port('HP1').read()
    assert v == {'rd_fabric': False, 'rd_issue': 16, 'rd_qos': 7, 'wr_fabric': False, 'wr_issue': 16,
                 'wr_qos': 0}
    assert Q.main([], mem) == 0
    out = capsys.readouterr().out
    assert out.count('\n') == len(Q.PORTS) and 'HP1  0xFD390000' in out and 'HPC1 0xFD370000' in out


def test_set_keeps_the_other_bits_and_saves(tmp_path, capsys):
    mem = FakeMem()
    save = str(tmp_path / 'before.json')
    assert Q.main(['set', '--port', 'hp1,HPC1', '--rd-issue', '4', '--wr-issue', '2', '--rd-qos', '0',
                   '--static', '--save', save], mem) == 0
    for name in ('HP1', 'HPC1'):
        w = mem.pages[name]
        assert mem.port(name).read() == {'rd_fabric': False, 'rd_issue': 4, 'rd_qos': 0, 'wr_fabric': False,
                                         'wr_issue': 2, 'wr_qos': 0}
        assert w[Q.RDCTRL // 4] == 0x3B0 and w[0x100 // 4] == 0xDEADBEEF
    assert mem.port('HP0').read()['rd_issue'] == 16                  # another port: as it was
    saved = json.load(open(save))
    assert set(saved) == {'HP1', 'HPC1'} and saved['HP1']['rd_issue'] == 16
    # a second set keeps the first saved values; restore brings them back
    assert Q.main(['set', '--port', 'HP1', '--rd-issue', '1', '--save', save], mem) == 0
    assert json.load(open(save))['HP1']['rd_issue'] == 16
    assert Q.main(['restore', save], mem) == 0
    assert mem.port('HP1').read()['rd_issue'] == 16 and mem.port('HPC1').read()['wr_issue'] == 16


def test_fabric_and_its_note(tmp_path, capsys):
    mem = FakeMem()
    save = str(tmp_path / 'b.json')
    Q.main(['set', '--port', 'HP1', '--fabric', '--save', save], mem)
    assert mem.pages['HP1'][Q.RDCTRL // 4] == 0x3B0 | Q.FABRIC_QOS_EN
    capsys.readouterr()
    Q.main(['set', '--port', 'HP1', '--wr-qos', '3', '--save', save], mem)
    assert 'the PL drives the QoS' in capsys.readouterr().out


@pytest.mark.parametrize('args', [['--rd-issue', '0'], ['--wr-issue', '17'], ['--rd-qos', '16'],
                                  ['--wr-qos', '-1'], []])
def test_refused(tmp_path, args):
    mem = FakeMem()
    with pytest.raises(SystemExit):
        Q.main(['set', '--port', 'HP1', '--save', str(tmp_path / 'x.json')] + args, mem)
    assert mem.port('HP1').read()['rd_issue'] == 16 and not os.path.exists(tmp_path / 'x.json')


def test_bad_port_and_bad_file(tmp_path):
    mem = FakeMem()
    with pytest.raises(SystemExit):
        Q.main(['set', '--port', 'HP9', '--rd-issue', '2', '--save', str(tmp_path / 'y.json')], mem)
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps({'HP1': {'rd_issue': 40}}))
    with pytest.raises(SystemExit):
        Q.main(['restore', str(bad)], mem)
    assert mem.port('HP1').read()['rd_issue'] == 16


def test_a_second_port_is_added_to_the_saved_file(tmp_path):
    mem = FakeMem()
    save = str(tmp_path / 'before.json')
    assert Q.main(['set', '--port', 'HP1', '--rd-issue', '2', '--save', save], mem) == 0
    assert Q.main(['set', '--port', 'HPC1,HP1', '--rd-issue', '1', '--rd-qos', '0', '--save', save], mem) == 0
    saved = json.load(open(save))
    assert set(saved) == {'HP1', 'HPC1'} and saved['HP1']['rd_issue'] == 16 and saved['HPC1']['rd_qos'] == 7
    assert Q.main(['restore', save], mem) == 0
    for name in ('HP1', 'HPC1'):
        assert mem.port(name).read() == {'rd_fabric': False, 'rd_issue': 16, 'rd_qos': 7, 'wr_fabric': False,
                                         'wr_issue': 16, 'wr_qos': 0}


def test_a_broken_saved_file_stops_before_any_write(tmp_path):
    mem = FakeMem()
    for text in ('{not json', json.dumps({'HP1': {'rd_issue': 40}}), json.dumps(['HP1'])):
        bad = tmp_path / 'bad.json'
        bad.write_text(text)
        with pytest.raises(SystemExit):
            Q.main(['set', '--port', 'HP1', '--rd-issue', '2', '--save', str(bad)], mem)
        assert mem.port('HP1').read()['rd_issue'] == 16 and bad.read_text() == text


def test_the_default_file_is_in_the_home_of_the_sudo_user(monkeypatch):
    monkeypatch.setenv('SUDO_USER', 'ubuntu')
    monkeypatch.setattr(Q.pwd, 'getpwnam', lambda user: type('P', (), {'pw_dir': '/home/' + user})())
    assert Q.default_save() == '/home/ubuntu/hp_qos_before.json'
    monkeypatch.delenv('SUDO_USER')
    assert Q.default_save() == os.path.expanduser('~/hp_qos_before.json')
