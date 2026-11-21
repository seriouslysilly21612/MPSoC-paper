"""pytest: tools/host/ddr_qos.py on stand-in registers (no /dev/mem)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'host'))
import ddr_qos as D  # noqa: E402


def test_the_reset_value_maps_15_to_high_priority_and_the_rest_to_low():
    reset = 0x02000E00                  # UG1087 PCFGQOS0_2: level1 0, level2 14, region 2 HPR
    assert D.read_class(reset, 0) == 'LPR' and D.read_class(reset, 14) == 'LPR'
    assert D.read_class(reset, 15) == 'HPR'


def test_three_regions_and_two_write_regions():
    qos0 = (2 << 24) | (1 << 20) | (0 << 16) | (11 << 8) | 3        # 0-3 LPR, 4-11 VPR, 12-15 HPR
    assert [D.read_class(qos0, q) for q in (0, 3, 4, 11, 12, 15)] == ['LPR', 'LPR', 'VPR', 'VPR', 'HPR', 'HPR']
    wqos0 = (1 << 20) | 7                                            # 0-7 NPW, 8-15 VPW
    assert [D.write_class(wqos0, q) for q in (0, 7, 8, 15)] == ['NPW', 'NPW', 'VPW', 'VPW']
    assert D.write_class(0, 15) == 'NPW'                             # reset: one class


def test_report_reads_every_port_at_its_offsets():
    regs = {}
    for n in range(6):
        regs[D.PORT_REGS['PCTRL'] + D.PORT_STEP * n] = 1
        regs[D.PORT_REGS['PCFGQOS0'] + D.PORT_STEP * n] = 0x02000E00
    regs[0x6A4] = (2 << 24) | (1 << 20) | (11 << 8) | 3             # port 3, the address of UG1087
    lines = D.report(lambda off: regs.get(off, 0), lambda off: (2 << 10) if off == 0 else 0)
    assert len(lines) == 1 + 2 * 6
    assert 'port 3 2' in lines[0]
    assert lines[7].startswith('port 3 (HP0, DisplayPort): enabled 1; read 0-3 LPR, 4-11 VPR, 12-15 HPR (levels 3 11)')
    assert 'PCFGQOS0 0x02100B03' in lines[8]


def test_a_port_with_one_read_queue_has_two_regions():
    # the board (I-119): HP ports 0x00100003 (0-3 LPR, 4-15 VPR), RPU 0x0020000B (0-11 LPR, 12-15 HPR)
    assert D.class_map(lambda q: D.read_class(0x00100003, q)) == '0-3 LPR, 4-15 VPR'
    assert D.class_map(lambda q: D.read_class(0x0020000B, q)) == '0-11 LPR, 12-15 HPR'
    assert D.class_map(lambda q: D.write_class(0x00100003, q)) == '0-3 NPW, 4-15 VPW'
    assert D.class_map(lambda q: D.read_class(0x02000B03, q)) == '0-11 LPR, 12-15 HPR'   # CCI: three regions
    assert D.class_map(lambda q: D.read_class(0x02000E00, q)) == '0-14 LPR, 15 HPR'
