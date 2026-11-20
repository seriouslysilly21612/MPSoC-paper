#!/usr/bin/env python3
"""hp_qos.py - the QoS of the PL ports into the PS (the AFIFM of each HP and HPC port; plan C3-4, D-24).

The DPU reaches the DDR through HP1 and HPC1 (the smartcam platform). Each port has, per channel, a static
QoS value that tags its requests at the DDR controller (0-3 best effort, 4-11 video, 12-15 low latency for
reads; that value counts only while FABRIC_QOS_EN is 0, else the PL drives it) and the number of commands
it may have outstanding (1-16). Both may be changed at run time (Xilinx Wiki, Traffic Shaping of HP Ports
on Zynq UltraScale+). AMD UG1087, AFIFM: RDCTRL 0x00 (bit 2 FABRIC_QOS_EN), RDISSUE 0x04 (count - 1),
RDQoS 0x08, WRCTRL 0x14, WRISSUE 0x18, WRQoS 0x1C.

  sudo python3 tools/host/hp_qos.py                       every port, as it is (reads only)
  sudo python3 tools/host/hp_qos.py set --port HP1,HPC1 [--rd-issue N] [--wr-issue N]
                                    [--rd-qos V] [--wr-qos V] [--static | --fabric] [--save FILE]
  sudo python3 tools/host/hp_qos.py restore FILE          the values a set saved

set reads every register it changes, keeps its other bits, writes, reads back and prints both; it saves the
values before it (default hp_qos_before.json in the home of the user who ran sudo). A port already in that
file keeps its first values there, the ones to go back to; a port not in it yet is added. Only these six
registers of these seven ports are touched. Standard library only; root (/dev/mem).
"""
import argparse
import json
import mmap
import os
import pwd
import sys

# UG1087: AFIFM0..5 are HPC0, HPC1, HP0..HP3 (FPD), AFIFM6 the LPD port
PORTS = {'HPC0': 0xFD360000, 'HPC1': 0xFD370000, 'HP0': 0xFD380000, 'HP1': 0xFD390000,
         'HP2': 0xFD3A0000, 'HP3': 0xFD3B0000, 'LPD': 0xFF9B0000}
RDCTRL, RDISSUE, RDQOS, WRCTRL, WRISSUE, WRQOS = 0x00, 0x04, 0x08, 0x14, 0x18, 0x1C
FABRIC_QOS_EN = 1 << 2
PAGE = 4096


class Port:
    """the six registers of one port in a word view w (memoryview 'I' of its page, or a stand-in)"""

    def __init__(self, w):
        self.w = w

    def get(self, off):
        return int(self.w[off // 4])

    def put(self, off, value):
        self.w[off // 4] = value & 0xFFFFFFFF

    def read(self):
        return {'rd_fabric': bool(self.get(RDCTRL) & FABRIC_QOS_EN), 'rd_issue': (self.get(RDISSUE) & 0xF) + 1,
                'rd_qos': self.get(RDQOS) & 0xF,
                'wr_fabric': bool(self.get(WRCTRL) & FABRIC_QOS_EN), 'wr_issue': (self.get(WRISSUE) & 0xF) + 1,
                'wr_qos': self.get(WRQOS) & 0xF}

    def write(self, values):
        """values: some of the keys of read(); the other bits of each register stay"""
        for key, (off, mask) in (('rd_issue', (RDISSUE, 0xF)), ('wr_issue', (WRISSUE, 0xF)),
                                 ('rd_qos', (RDQOS, 0xF)), ('wr_qos', (WRQOS, 0xF))):
            if key in values:
                v = values[key] - 1 if key.endswith('issue') else values[key]
                self.put(off, (self.get(off) & ~mask) | v)
        for key, off in (('rd_fabric', RDCTRL), ('wr_fabric', WRCTRL)):
            if key in values:
                r = self.get(off)
                self.put(off, (r | FABRIC_QOS_EN) if values[key] else (r & ~FABRIC_QOS_EN))


def check(values):
    """values for a write: issue 1-16, QoS 0-15, fabric a bool; a list of what is wrong"""
    bad = []
    for key, v in values.items():
        if key.endswith('issue') and not (isinstance(v, int) and 1 <= v <= 16):
            bad.append(f'{key} {v!r}: 1-16 commands')
        elif key.endswith('qos') and not (isinstance(v, int) and 0 <= v <= 15):
            bad.append(f'{key} {v!r}: 0-15')
        elif key.endswith('fabric') and not isinstance(v, bool):
            bad.append(f'{key} {v!r}: true or false')
        elif not key.endswith(('issue', 'qos', 'fabric')):
            bad.append(f'{key}: no such field')
    return bad


def default_save():
    """hp_qos_before.json in the home of the user who ran sudo (sudo sets HOME to /root)"""
    user = os.environ.get('SUDO_USER')
    if user:
        try:
            return os.path.join(pwd.getpwnam(user).pw_dir, 'hp_qos_before.json')
        except KeyError:
            pass
    return os.path.expanduser('~/hp_qos_before.json')


def read_saved(path):
    """{port: values} of a file a set wrote; SystemExit if it is not one"""
    try:
        with open(path) as f:
            saved = json.load(f)
    except (OSError, ValueError) as e:
        raise SystemExit(f'hp_qos: {path}: {e}')
    if not isinstance(saved, dict) or any(name not in PORTS or not isinstance(values, dict) or check(values)
                                          for name, values in saved.items()):
        raise SystemExit(f'hp_qos: {path}: not a file of saved ports')
    return saved


def show(name, v):
    def ch(p):
        src = 'from the PL' if v[f'{p}_fabric'] else f'static {v[f"{p}_qos"]}'
        return f'{v[f"{p}_issue"]:2d} outstanding, QoS {src}'
    return f'{name:4s} 0x{PORTS[name]:08X}  read: {ch("rd")};  write: {ch("wr")}'


class DevMem:
    """the page of each port, mapped from /dev/mem"""

    def __init__(self):
        try:
            self.fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        except OSError:
            raise SystemExit('hp_qos: cannot open /dev/mem: run with sudo')
        self.maps = {}

    def port(self, name):
        if name not in self.maps:
            try:
                mm = mmap.mmap(self.fd, PAGE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                               offset=PORTS[name])
            except OSError as e:
                raise SystemExit(f'hp_qos: mmap of {name} 0x{PORTS[name]:08X} failed: {e}')
            self.maps[name] = mm
        return Port(memoryview(self.maps[name]).cast('I'))


def parse_ports(text):
    names = [p.strip().upper() for p in text.split(',') if p.strip()]
    bad = [p for p in names if p not in PORTS]
    if bad or not names:
        raise SystemExit(f'hp_qos: ports {bad or text!r}: of {", ".join(PORTS)}')
    return names


def main(argv=None, mem=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd')
    s = sub.add_parser('set')
    s.add_argument('--port', required=True)
    s.add_argument('--rd-issue', type=int)
    s.add_argument('--wr-issue', type=int)
    s.add_argument('--rd-qos', type=int)
    s.add_argument('--wr-qos', type=int)
    g = s.add_mutually_exclusive_group()
    g.add_argument('--static', action='store_true', help='FABRIC_QOS_EN 0: the static values count')
    g.add_argument('--fabric', action='store_true', help='FABRIC_QOS_EN 1: the PL drives the QoS')
    s.add_argument('--save', default=None, help='the file of the values before it (default: '
                   'hp_qos_before.json in the home of the user who ran sudo)')
    r = sub.add_parser('restore')
    r.add_argument('file')
    a = ap.parse_args(argv)
    mem = mem or DevMem()

    if a.cmd is None:
        for name in PORTS:
            print(show(name, mem.port(name).read()))
        return 0

    if a.cmd == 'restore':
        saved = read_saved(a.file)
        for name, values in saved.items():
            mem.port(name).write(values)
            print('restored ' + show(name, mem.port(name).read()))
        return 0

    values = {k: v for k, v in (('rd_issue', a.rd_issue), ('wr_issue', a.wr_issue),
                                ('rd_qos', a.rd_qos), ('wr_qos', a.wr_qos)) if v is not None}
    if a.static or a.fabric:
        values['rd_fabric'] = values['wr_fabric'] = bool(a.fabric)
    bad = check(values)
    if bad or not values:
        raise SystemExit('hp_qos: ' + ('; '.join(bad) if bad else 'nothing to set'))
    names = parse_ports(a.port)
    save = a.save or default_save()
    before = {name: mem.port(name).read() for name in names}
    # each port keeps the values it had before its first set: a second set on another port adds that port
    saved = read_saved(save) if os.path.exists(save) else {}
    new = [name for name in names if name not in saved]
    if new:
        saved.update({name: before[name] for name in new})
        with open(save, 'w') as f:
            json.dump(saved, f, indent=1)
        print(f'saved the values before it of {", ".join(new)}: {save} (hp_qos.py restore {save})')
    if len(new) < len(names):
        print(f'{save}: the first values of {", ".join(n for n in names if n not in new)} kept')
    failed = 0
    for name in names:
        port = mem.port(name)
        port.write(values)
        after = port.read()
        print('before ' + show(name, before[name]))
        print('after  ' + show(name, after))
        failed += sum(after[k] != v for k, v in values.items())
        for ch in ('rd', 'wr'):
            if f'{ch}_qos' in values and after[f'{ch}_fabric']:
                print(f'  note: {name} {ch}: the PL drives the QoS (FABRIC_QOS_EN 1): the static value '
                      'counts only with --static')
    if failed:
        print(f'hp_qos: {failed} value(s) did not read back as written')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
