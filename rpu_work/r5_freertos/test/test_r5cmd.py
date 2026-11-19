"""Host test of r5cmd.py's slot code against a simulated R5 (a bytearray as the OCM). python3 test/test_r5cmd.py"""
import os, struct, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import r5cmd as c

def mk():
    buf = bytearray(c.OCM_SIZE)
    return Link(buf)

def Link(buf):
    return c.Link(memoryview(buf).cast("I"))

def r5_start(l):
    l.w[c.HEAD_R5 // 4 + 2] += 1; l.w[c.HEAD_R5 // 4 + 1] = c.IPC_LAYOUT; l.w[c.HEAD_R5 // 4] = c.IPC_MAGIC
    l.w[c.OFF_STATE // 4] |= 1

def r5_publish(l, flags, cmd_done):
    vals = [0, 7, 4, flags, 0, cmd_done, 0, 0, 3] + [0x1237] * 8 + [0.5] * 8 + [0.0] * 8 + [0.1] * 8 + [5] * 8 + [6] * 8 + [1, 2, 3, 4]
    l.slot_write(c.OFF_STATE, l.w[c.HEAD_R5 // 4 + 2], struct.pack(c.STATE_FMT, *vals))

l = mk()
assert l.read_state() is None                      # R5 not up
r5_start(l)
assert l.read_state() is None                      # up but slot still odd
r5_publish(l, c.ST_OP, 0)
s = l.read_state()
assert s and s["flags"] == c.ST_OP and s["period"] == 7 and s["q"][3] == 0.5 and s["tau_act"][7] == 5 and s["wkc"] == 4
assert s["stale"] == 3 and s["status"][0] == 0x1237 and s["tau_cmd"][0] == 6 and s["late"] == 1

# torn read: the "R5" writes between the reader's two gen loads -> None
l.w[c.OFF_STATE // 4] |= 1
assert l.read_state() is None
r5_publish(l, c.ST_OP, 0)
# epoch of an older R5 run is refused
l.w[c.OFF_STATE // 4 + 1] = 99
assert l.read_state() is None
r5_publish(l, c.ST_OP, 0)

# APU side: start leaves slots odd, first write makes them even, gen never restarts, epoch never 0
l.side_start()
e1 = l.epoch
assert l.w[c.OFF_COMMAND // 4] & 1 and l.w[c.OFF_TORQUE // 4] & 1
l.send_command(c.CMD_SERVO_ON, 1)
g = l.w[c.OFF_COMMAND // 4]
assert g % 2 == 0 and l.w[c.OFF_COMMAND // 4 + 1] == e1
assert (l.w[c.OFF_COMMAND // 4 + 2], l.w[c.OFF_COMMAND // 4 + 3]) == (1, c.CMD_SERVO_ON)
assert not any(l.w[c.OFF_COMMAND // 4 + 4 + i] for i in range(28))       # the numbers of a start: none in a servo command
l.side_start()
assert l.epoch == e1 + 1 and l.w[c.OFF_COMMAND // 4] == g | 1
l.epoch = 0xFFFFFFFF; l.w[c.HEAD_APU // 4 + 2] = 0xFFFFFFFF; l.side_start(); assert l.epoch == 1   # wrap skips 0

# torque payload lands where ipc_torque_t has it
l.send_torque([0.05, 0, 0, -0.05], 9)
i = c.OFF_TORQUE // 4 + 2
raw = b"".join(struct.pack("<I", l.w[i + k]) for k in range(18))
per, fl, *t = struct.unpack(c.TORQUE_FMT, raw)
assert per == 9 and fl == 3 and t[0] == 0.05 and t[3] == -0.05 and t[4] == 0.0

# clamp and parse
assert c.parse_tau("1") == [c.MAX_TAU_NM] * 3 and c.parse_tau("-1,0,0.05")[0] == -c.MAX_TAU_NM and c.parse_tau(None) == [0.0] * 3
try:
    c.parse_tau("1,2"); assert False
except SystemExit:
    pass
# gen wrap: 0 is never published
l.w[c.OFF_TORQUE // 4] = 0xFFFFFFFE; l.send_torque([0.0]); assert l.w[c.OFF_TORQUE // 4] == 2
# sine shape: starts at 0, peak at a quarter period, constant when freq is 0
assert c.shape_tau([0.01, 0, 0, 0], 0, 3.0) == [0.01, 0, 0, 0]
assert c.shape_tau([0.01, -0.02, 0, 0], 2.0, 0.0) == [0.0, 0.0, 0.0, 0.0]
q = c.shape_tau([0.01, -0.02, 0, 0], 2.0, 0.125)
assert abs(q[0] - 0.01) < 1e-12 and abs(q[1] + 0.02) < 1e-12
# the numbers of the layout are those of the firmware's header (a firmware with another layout number refuses the tool)
import re
hdr = open(os.path.join(os.path.dirname(__file__), "..", "src", "ipc", "ipc_layout.h")).read()
num = lambda name: int(re.search(r"#define\s+%s\s+(0x[0-9A-Fa-f]+|\d+)u" % name, hdr).group(1), 0)
assert (num("IPC_LAYOUT"), num("IPC_OFF_STATE"), num("IPC_OFF_TORQUE"), num("IPC_OFF_COMMAND")) == \
       (c.IPC_LAYOUT, c.OFF_STATE, c.OFF_TORQUE, c.OFF_COMMAND)
print("test_r5cmd OK")
