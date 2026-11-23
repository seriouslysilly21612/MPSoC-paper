#!/usr/bin/env python3
"""ddr_qos.py - how the DDR controller of the PS treats the AXI QoS of its six ports (C3-4, I-116): reads only.

tools/host/hp_qos.py sets the static QoS that the PL ports (HPC0-1, HP0-3) put on their requests; what that value
does is decided in the DDR controller, port by port. For the reads (PCFGQOS0_n) two levels split the 16 values
into three regions, each mapped to a traffic class (LPR low priority, VPR variable priority, HPR high priority);
for the writes (PCFGWQOS0_n) one level splits them into two (NPW normal, VPW variable priority). The port
arbiter also takes the value itself as the priority of the port (UG1087). This prints, for every port, whether
it is enabled and the class of every QoS value, with the raw registers, and the PORT_TYPE register of the
DDR QoS controller. Nothing is written.

  sudo python3 -I -B tools/host/ddr_qos.py

The ports (UG1085, from memory: check against the board's design): 0 RPU, 1 and 2 the CCI (APU, HPC0-1),
3 HP0 and DisplayPort, 4 HP1 and HP2, 5 HP3 and the FPD DMA. Registers: UG1087, DDRC at 0xFD070000 (PCCFG 0x400,
then per port n at 0xB0 * n: PCFGR 0x404, PCFGW 0x408, PCTRL 0x490, PCFGQOS0 0x494, PCFGQOS1 0x498, PCFGWQOS0
0x49C, PCFGWQOS1 0x4A0), DDR_QOS_CTRL at 0xFD090000 (PORT_TYPE 0x0). Standard library only; root (/dev/mem).
"""
import mmap
import os
import sys

DDRC, DDR_QOS_CTRL = 0xFD070000, 0xFD090000
PAGE = 4096
PCCFG = 0x400
PORT_REGS = {'PCFGR': 0x404, 'PCFGW': 0x408, 'PCTRL': 0x490, 'PCFGQOS0': 0x494, 'PCFGQOS1': 0x498,
             'PCFGWQOS0': 0x49C, 'PCFGWQOS1': 0x4A0}
PORT_STEP = 0xB0
PORTS = ('RPU', 'CCI (APU, HPC0-1)', 'CCI (APU, HPC0-1)', 'HP0, DisplayPort', 'HP1, HP2', 'HP3, FPD DMA')
READ_CLASS = {0: 'LPR', 1: 'VPR', 2: 'HPR'}
WRITE_CLASS = {0: 'NPW', 1: 'VPW'}


def field(v, lo, width):
    return (v >> lo) & ((1 << width) - 1)


def read_class(qos0, q):
    """the read class of AxQoS q under PCFGQOS0: region 0 up to level1, region 1 up to level2, region 2 above.
    level2 <= level1 means a port with one read queue: two regions, region 1 above level1 (I-119)"""
    level1, level2 = field(qos0, 0, 4), field(qos0, 8, 4)
    if level2 <= level1:
        region = 0 if q <= level1 else 1
    else:
        region = 0 if q <= level1 else 1 if q <= level2 else 2
    c = field(qos0, 16 + 4 * region, 2)
    return READ_CLASS.get(c, '?%d' % c)


def write_class(wqos0, q):
    """the write class of AwQoS q under PCFGWQOS0: region 0 up to its level, region 1 above"""
    region = 0 if q <= field(wqos0, 0, 4) else 1
    c = field(wqos0, 16 + 4 * region, 2)
    return WRITE_CLASS.get(c, '?%d' % c)


def class_map(cls):
    """the 16 AxQoS values grouped by class: cls(q) -> name; '0-3 LPR, 4-15 VPR'"""
    runs, start = [], 0
    for q in range(1, 17):
        if q == 16 or cls(q) != cls(start):
            runs.append('%s %s' % ('%d' % start if q - 1 == start else '%d-%d' % (start, q - 1), cls(start)))
            start = q
    return ', '.join(runs)


def report(ddrc, qosc):
    """the lines for the word reader of each page: ddrc(off), qosc(off) -> the 32-bit value at that offset"""
    lines = ['PCCFG 0x%08X; PORT_TYPE (DDR QoS controller) 0x%08X: port 3 %d, port 4 %d, port 5 %d '
             '(0 best effort, 1 low latency, 2 video)'
             % (ddrc(PCCFG), qosc(0), field(qosc(0), 10, 2), field(qosc(0), 12, 2), field(qosc(0), 14, 2))]
    for n, who in enumerate(PORTS):
        r = {name: ddrc(off + PORT_STEP * n) for name, off in PORT_REGS.items()}
        q0, w0 = r['PCFGQOS0'], r['PCFGWQOS0']
        lines.append('port %d (%s): enabled %d; read %s (levels %d %d); write %s (level %d)'
                     % (n, who, r['PCTRL'] & 1, class_map(lambda q: read_class(q0, q)), field(q0, 0, 4),
                        field(q0, 8, 4), class_map(lambda q: write_class(w0, q)), field(w0, 0, 4)))
        lines.append('  ' + ' '.join('%s 0x%08X' % (name, v) for name, v in r.items()))
    return lines


def main():
    try:
        fd = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
    except OSError:
        sys.exit('ddr_qos: cannot open /dev/mem: run with sudo')
    words = []
    for base in (DDRC, DDR_QOS_CTRL):
        try:
            mm = mmap.mmap(fd, PAGE, mmap.MAP_SHARED, mmap.PROT_READ, offset=base)
        except OSError as e:
            sys.exit('ddr_qos: mmap of 0x%08X failed: %s' % (base, e))
        w = memoryview(mm).cast('I')
        words.append(lambda off, w=w: int(w[off // 4]))
    print('\n'.join(report(*words)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
