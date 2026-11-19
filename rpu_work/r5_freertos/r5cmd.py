#!/usr/bin/env python3
"""r5cmd.py -- run the L7N bench from Linux: servo on/off and a small test torque through the OCM link.

The R5 firmware (ocm_link.c) shares the first 16 KB of the OCM (0xFFFC0000) with the APU: a State slot (R5 -> APU),
a Torque slot and a Command slot (APU -> R5), the single-writer slots of ipc_slot.h.  The APU application has no
R5 code yet, so this script is the APU side for the bench.  Needs root (/dev/mem).  Start the R5 first
(r5log.py -f in another terminal shows its console), wait for "OP" in its log, then:

  sudo python3 r5cmd.py status                 State slot once: flags, status words, q, torque, counters
  sudo python3 r5cmd.py watch                  State slot 5x per second, until Ctrl-C
  sudo python3 r5cmd.py run                    servo on, hold zero torque, show State; Ctrl-C = servo off
  sudo python3 r5cmd.py run --tau 0.05,0,0     the same with a constant test torque [N m] per axis
  sudo python3 r5cmd.py off                    send servo off (one command, no loop)

Why `run` is a loop: the R5 counts a Torque slot that did not change as "stale" and switches the servo off after
TORQUE_STALE_LIMIT (10) such periods (1 kHz), also when the torque is 0.  So the Torque slot is rewritten every
millisecond for as long as the servo is on.  If the loop stalls for 10 ms the R5 turns the servo off by itself (the
safe direction); `run` then says so and ends.  --tau is clamped to MAX_TAU_NM (the drive's own limit is 10 % of the
rated torque = 0.1337 N m, MAX_TORQUE_PERMILLE).

Words only: /dev/mem maps the OCM as Device memory; every access here is one aligned 32-bit load or store (a
memoryview cast to 'I'), never a slice copy (r5log.py rev2: memcpy faults there).
"""
import argparse
import gc
import math
import mmap
import os
import signal
import struct
import sys
import time

OCM_ADDR, OCM_SIZE = 0xFFFC0000, 0x4000
IPC_MAGIC, IPC_LAYOUT = 0x31435049, 5
OFF_HEAD, OFF_STATE, OFF_TORQUE, OFF_COMMAND = 0x000, 0x040, 0x1C0, 0x800
HEAD_R5, HEAD_APU = 0x00, 0x20                  # ipc_side_t: magic, layout, epoch (+5 reserved)
CMD_SERVO_ON, CMD_SERVO_OFF, CMD_STOP = 1, 2, 3
TORQUE_VALID, TORQUE_ENABLED = 1, 2
ST_OP, ST_WKC_ERR, ST_SERVO_ON, ST_STOP, ST_CONTROL = 1, 2, 4, 8, 16
AXES = 3                                        # L7N bench (bench/L7N3_bus.cfg)
MAX_TAU_NM = 0.13                               # a hair under the drive-side 10 % limit (0.1337 N m)
PERIOD_S = 0.001
OFF_WAIT_S = 2.0

STATE_FMT = "<QIIIIIIII8H8d8d8d8i8i4I"          # ipc_state_t, 328 bytes
STATE_BYTES = struct.calcsize(STATE_FMT)
TORQUE_FMT = "<II8d"                            # ipc_torque_t, 72 bytes
COMMAND_FMT = "<IIII13d"                        # ipc_command_t, 120 bytes: seq, cmd, then what a servo command leaves 0
assert STATE_BYTES == 328 and struct.calcsize(TORQUE_FMT) == 72 and struct.calcsize(COMMAND_FMT) == 120


class Link:
    """The shared region as 32-bit words.  `w` is a list-like of uint32 (memoryview 'I', or one over a bytearray in the test)."""

    def __init__(self, words):
        self.w = words
        self.epoch = 0

    # ---- slots (ipc_slot.h) ----
    def slot_write(self, off, epoch, payload):
        """No barriers here (shortcut): Python runs the stores in program order and the mapping is Device memory."""
        i = off // 4
        gen = self.w[i] | 1
        self.w[i] = gen
        self.w[i + 1] = epoch
        for k in range(len(payload) // 4):
            self.w[i + 2 + k] = struct.unpack_from("<I", payload, 4 * k)[0]
        gen = (gen + 1) & 0xFFFFFFFF
        self.w[i] = gen or 2

    def slot_read(self, off, writer_off, nbytes):
        """One attempt, like IPC_READ.  -> (payload bytes, (epoch, gen)) or None."""
        wi = writer_off // 4
        if self.w[wi] != IPC_MAGIC or self.w[wi + 1] != IPC_LAYOUT:
            return None
        i = off // 4
        gen = self.w[i]
        if gen & 1:
            return None
        epoch = self.w[i + 1]
        buf = b"".join(struct.pack("<I", self.w[i + 2 + k]) for k in range(nbytes // 4))
        if self.w[i] != gen or epoch != self.w[wi + 2]:
            return None
        return buf, (epoch, gen)

    # ---- the APU side ----
    def side_start(self):
        """ipc_side_start for the APU half: slots Torque and Command stay 'being written' until the first write."""
        h = HEAD_APU // 4
        self.w[h] = 0
        for off in (OFF_TORQUE, OFF_COMMAND):
            self.w[off // 4] |= 1
        e = (self.w[h + 2] + 1) & 0xFFFFFFFF
        self.epoch = e or 1
        self.w[h + 2] = self.epoch
        self.w[h + 1] = IPC_LAYOUT
        self.w[h] = IPC_MAGIC

    def send_command(self, cmd, seq):
        self.slot_write(OFF_COMMAND, self.epoch, struct.pack(COMMAND_FMT, seq, cmd, 0, 0, *([0.0] * 13)))

    def send_torque(self, tau, period=0):
        t = list(tau) + [0.0] * (8 - len(tau))
        self.slot_write(OFF_TORQUE, self.epoch, struct.pack(TORQUE_FMT, period, TORQUE_VALID | TORQUE_ENABLED, *t))

    def read_state(self):
        """-> dict or None (R5 not up / being written / torn: try again)."""
        r = self.slot_read(OFF_STATE, HEAD_R5, STATE_BYTES)
        if r is None:
            return None
        v = struct.unpack(STATE_FMT, r[0])
        return {"t": v[0], "period": v[1], "dof": v[2], "flags": v[3], "not_ready": v[4], "cmd_done": v[5],
                "cfg_crc": v[6], "torque_period": v[7], "stale": v[8], "status": v[9:17], "q": v[17:25],
                "qd": v[25:33], "tau": v[33:41], "tau_act": v[41:49], "tau_cmd": v[49:57],
                "late": v[57], "overrun": v[58], "missed": v[59], "wkc": v[60]}


def fmt_state(s):
    fl = "".join(n if s["flags"] & b else "-" for n, b in (("OP ", ST_OP), ("WKC_ERR ", ST_WKC_ERR),
                                                           ("SERVO_ON ", ST_SERVO_ON), ("STOP ", ST_STOP),
                                                           ("CONTROL ", ST_CONTROL)))
    n = min(AXES, s["dof"])
    return ("period %u  flags %s  cmd_done %u  stale %u  late/overrun/missed/wkc %u/%u/%u/%u\n"
            "  status  %s\n  q [rad] %s\n  tau_act [N m] %s   tau_cmd_raw %s   tau_act_raw %s"
            % (s["period"], fl, s["cmd_done"], s["stale"], s["late"], s["overrun"], s["missed"], s["wkc"],
               " ".join("%04x" % x for x in s["status"][:n]), " ".join("%+.4f" % x for x in s["q"][:n]),
               " ".join("%+.4f" % x for x in s["tau"][:n]), list(s["tau_cmd"][:n]), list(s["tau_act"][:n])))


def parse_tau(text):
    if not text:
        return [0.0] * AXES
    v = [float(x) for x in text.split(",")]
    if len(v) == 1:
        v = v * AXES                            # one number = every axis
    if len(v) != AXES:
        raise SystemExit("--tau needs 1 or %d numbers, got %d" % (AXES, len(v)))
    return [max(-MAX_TAU_NM, min(MAX_TAU_NM, x)) for x in v]


def shape_tau(amp, freq_hz, t):
    """Torque of the moment t [s] after the start: a constant (freq_hz 0) or amp * sin(2 pi f t), so it begins at 0."""
    if not freq_hz:
        return amp
    k = math.sin(2.0 * math.pi * freq_hz * t)
    return [a * k for a in amp]


def open_link():
    try:
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    except PermissionError:
        raise SystemExit("cannot open /dev/mem: run with sudo")
    try:
        mm = mmap.mmap(fd, OCM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=OCM_ADDR)
    except (OSError, ValueError) as e:
        raise SystemExit("mmap of OCM 0x%X failed: %s  (CONFIG_STRICT_DEVMEM? see r5log.py)" % (OCM_ADDR, e))
    link = Link(memoryview(mm).cast("I"))
    link._keep = mm
    return link


def wait_state(link, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = link.read_state()
        if s:
            return s
        time.sleep(0.005)
    return None


def need_r5(link):
    s = wait_state(link)
    if not s:
        raise SystemExit("no State from the R5 (R5 not running, or the firmware has no OCM link yet): start it, check r5log.py")
    time.sleep(0.05)
    s2 = wait_state(link)
    if not s2 or s2["period"] == s["period"]:
        raise SystemExit("the State slot does not advance: the R5 task is not cycling")
    return s2


def last_seq(link):
    """The sequence number in the Command slot now, so the next command differs from it."""
    return link.w[OFF_COMMAND // 4 + 2]


def cmd_status(link, args):
    s = wait_state(link)
    if not s:
        print("no State (R5 down, or the slot is being written)")
        return 1
    print(fmt_state(s))
    return 0


def cmd_watch(link, args):
    try:
        while True:
            s = link.read_state()
            if s:
                print(fmt_state(s) + "\n")
            time.sleep(0.2)
    except KeyboardInterrupt:
        return 0


def send_once(link, cmd):
    s = need_r5(link)
    link.side_start()
    seq = (last_seq(link) + 1) & 0xFFFFFFFF
    link.send_command(cmd, seq)
    t0 = time.time()
    while time.time() - t0 < OFF_WAIT_S:
        s = link.read_state()
        if s and s["cmd_done"] == seq:
            print("R5 took command %d (seq %d)" % (cmd, seq))
            return 0
        time.sleep(0.01)
    print("the R5 did not report seq %d within %.0f s" % (seq, OFF_WAIT_S))
    return 1


def cmd_off(link, args):
    return send_once(link, CMD_SERVO_OFF)


def cmd_run(link, args):
    tau = parse_tau(args.tau)
    s = need_r5(link)
    if not s["flags"] & ST_OP:
        raise SystemExit("the R5 is not in OP yet (flags %d): wait for OP in r5log.py" % s["flags"])
    if s["flags"] & ST_CONTROL:
        raise SystemExit("this firmware has the control task (CONTROL_TASK): it takes no torque of the APU, and a servo on "
                         "only with its controller on. `run` is for the firmware without it")
    print("torque [N m] %s (clamped to +-%.2f)%s" % (tau, MAX_TAU_NM,
          " as the amplitude of a sine of %.3g Hz" % args.sine if args.sine else ""))
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50))
        print("[SCHED_FIFO 50 on]")
    except (OSError, AttributeError):
        print("[no SCHED_FIFO: the 1 kHz loop may jitter; the R5 stops the servo after 10 late periods]")
    try:
        os.sched_setaffinity(0, {os.cpu_count() - 1})     # one core, so the loop is not moved about
    except (OSError, AttributeError):
        pass
    gc.disable()                                      # a collection in the middle of the loop is a 10 ms hole
    print("[auto servo off: after %s s, or when a speed exceeds %.1f rad/s]" % (args.seconds or "no limit", args.max_qd))
    stop = []
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    link.side_start()
    link.send_torque([0.0] * AXES)                    # a fresh torque slot before the servo is asked for
    seq = (last_seq(link) + 1) & 0xFFFFFFFF
    link.send_command(CMD_SERVO_ON, seq)
    nxt = last = t_start = time.perf_counter()
    next_print = 0.0
    gap_max = 0.0                                     # longest time between two Torque slot writes [s]
    n_over3 = n_over9 = 0                             # gaps above 3 ms / 9 ms (10 ms = the R5's stale limit)
    n = 0
    seen_on = False
    ended = None
    try:
        while not stop:
            n += 1
            link.send_torque(shape_tau(tau, args.sine, time.perf_counter() - t_start), n)
            now = time.perf_counter()
            gap = now - last
            last = now
            if n > 1:
                gap_max = max(gap_max, gap)
                n_over3 += gap > 0.003
                n_over9 += gap > 0.009
            s = link.read_state()
            if s:
                seen_on = seen_on or bool(s["flags"] & ST_SERVO_ON)
                if max(abs(x) for x in s["qd"][:AXES]) > args.max_qd:
                    ended = "speed limit %.1f rad/s exceeded: servo off" % args.max_qd
                    break
                if seen_on and not s["flags"] & ST_SERVO_ON:
                    ended = "the drives left OPERATION_ENABLED (R5 stopped the servo: stale torque, fault, or WKC error)"
                    break
                if now >= next_print:
                    print(fmt_state(s) + "\n  loop: %d writes, longest gap %.1f ms, >3 ms %d, >9 ms %d\n"
                          % (n, gap_max * 1e3, n_over3, n_over9))
                    next_print = now + 1.0
            if args.seconds and now - t_start > args.seconds:
                break
            nxt += PERIOD_S
            while time.perf_counter() < nxt - 0.0002:
                time.sleep(0.0001)
            while time.perf_counter() < nxt:
                pass
            if time.perf_counter() - nxt > 0.005:     # the loop itself stalled
                nxt = time.perf_counter()
    except KeyboardInterrupt:
        pass
    print("[r5cmd: %d writes, longest gap %.1f ms, >3 ms %d, >9 ms %d]" % (n, gap_max * 1e3, n_over3, n_over9))
    if ended:
        print("[r5cmd: %s]" % ended)
    # servo off: zero torque, keep the slot fresh while the R5 takes the command
    link.send_torque([0.0] * AXES)
    link.send_command(CMD_SERVO_OFF, (seq + 1) & 0xFFFFFFFF)
    t0 = time.time()
    while time.time() - t0 < OFF_WAIT_S:
        link.send_torque([0.0] * AXES, n)
        n += 1
        s = link.read_state()
        if s and s["cmd_done"] == (seq + 1) & 0xFFFFFFFF:
            print("[r5cmd: servo off taken by the R5]")
            return 0 if not ended else 1
        time.sleep(PERIOD_S)
    print("[r5cmd: no confirmation of servo off within %.0f s: E-stop / check r5log.py]" % OFF_WAIT_S)
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run the L7N bench through the OCM link (needs root).")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("status")
    sp.add_parser("watch")
    sp.add_parser("off")
    r = sp.add_parser("run")
    r.add_argument("--seconds", type=float, default=0, help="servo off after this many seconds (default: until Ctrl-C)")
    r.add_argument("--max-qd", type=float, default=10.0, help="servo off when any |speed| [rad/s] exceeds this (default 10)")
    r.add_argument("--sine", type=float, default=0.0, metavar="HZ",
                   help="--tau is the amplitude of tau*sin(2 pi HZ t) instead of a constant")
    r.add_argument("--tau", help="test torque [N m]: one number for all axes or %d comma-separated (default 0)" % AXES)
    args = ap.parse_args(argv)
    link = open_link()
    return {"status": cmd_status, "watch": cmd_watch, "off": cmd_off, "run": cmd_run}[args.cmd](link, args)


if __name__ == "__main__":
    sys.exit(main())
