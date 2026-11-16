#!/usr/bin/env python3
"""r5log.py -- read the Cortex-R5 console ring buffer from Linux.

The R5 firmware (r5log.c) sends everything that goes through xil_printf()/print()/printf()
into a 16 KB ring at physical address 0x363FC000 (last 16 KB of the 4 MB R5 DDR carveout).
This script reads that ring through /dev/mem, so it needs root.

  sudo python3 r5log.py            print what is in the ring now, then exit
  sudo python3 r5log.py -f         keep printing new output (like tail -f); Ctrl-C = stop the run (below)
  sudo python3 r5log.py --header   show the ring header (magic / head / boot_id) and exit

The ring is NOT cleared when the R5 stops, so after an R5 hang the last lines are still here.
Read it BEFORE you start the R5 again (a new run restarts the ring).

Ring layout (little endian): u32 magic 'R5LG' | u32 version=1 | u32 data_bytes | u32 head |
u32 boot_id | 3 x u32 reserved | data[data_bytes]; byte number k of the output stream lives at
data[k % data_bytes]; head = number of bytes written since the R5 (re)started.

Ctrl-C in -f (rev3, 2026-11-17) stops the run cleanly, as the APU application does on Ctrl-C.  r5log.py writes a
stop command into the page below the ring (0x363FB000, the same address as ecat_task.c); the R5 holds the position
100 ms, switches every drive off, takes the drives to PRE-OP while the frames still flow, and stops sending.  When
the R5 reports it stopped, r5log.py prints its last lines and stops the R5 (remoteproc0).  In PRE-OP no process
data is expected, so no drive's SM watchdog expires (AL-67 on the L7N).  Stopping the R5 in OP instead cuts the
frames there.  If no stop is reported within STOP_WAIT_S, r5log.py leaves the R5 alone and says what to do; a
second Ctrl-C stops waiting.  After Ctrl-C the output goes to stderr: Ctrl-C has also ended a "| tee".  Ending the
viewer alone: kill it (SIGTERM).  Firmware without the command (no 'R5ST' in the page): Ctrl-C ends the viewer only.

Why every read here is a single-byte load (rev2): /dev/mem maps the R5 carveout (a "no-map" reserved region)
as Device memory, where every access must be naturally aligned.  mm[a:b] / bytes(mm[a:b]) run the C library's
memcpy, which finishes an odd-sized copy with an unaligned 8/16-byte load -> alignment fault -> the kernel
kills the process with SIGBUS ("Bus error").  rev1 did exactly that: it had only been tried on a regular
file, where nothing can fault.  Byte loads cannot be misaligned, so rd() is the only way the mapping is read.
"""
import argparse
import faulthandler
import mmap
import os
import struct
import sys
import time

MAGIC = b"R5LG"
HDR = 32
DEF_ADDR = 0x363FC000
DEF_SIZE = 0x4000

# The stop command (rev3).  Linux writes only the page's first 32-byte line, the R5 only the second (ecat_task.c),
# so the write-back of one side's cache line can never undo the other side's byte.  Bytes only: Device memory.
CTL_ADDR = 0x363FB000
CTL_SIZE = 0x1000
CTL_CMD = 0x00                  # u8, Linux -> R5
CMD_STOP = 0x53                 # 'S'
CTL_MAGIC = 0x20                # R5 -> Linux: b"R5ST", the state byte (ecat_status as int8), 3 bytes, u32 boot_id
ST_MAGIC = b"R5ST"
ST_STOPPED, ST_INIT_FAILED = 3, 0xFF
RPROC_STATE = "/sys/class/remoteproc/remoteproc0/state"
STOP_WAIT_S = 15                # bring-up (OP_TIMEOUT_MS 10 s) + hold, servo off and PRE-OP (2.6 s at most)
MANUAL = ("E-stop (or drive power off) first, then:  echo stop | sudo tee %s" % RPROC_STATE)


class RingError(Exception):
    pass


def rd(mm, off, n):
    """n bytes at `off`, read with single-byte loads only (see the module docstring)."""
    return bytes(mm[i] for i in range(off, off + n))


def parse_hdr(mm):
    """-> (magic, version, data_bytes, head, boot_id)"""
    return struct.unpack("<4sIIII", rd(mm, 0, 20))


def check_hdr(mm, size):
    """Validate the header, return (data_bytes, head, boot_id)."""
    magic, version, dsize, head, boot = parse_hdr(mm)
    if magic != MAGIC:
        raise RingError("no R5 log header yet (magic=%r): R5 not started, or it runs firmware without r5log.c"
                        % (magic,))
    if version != 1:
        raise RingError("unknown ring version %d" % version)
    if dsize == 0 or dsize > size - HDR:
        raise RingError("corrupt header: data_bytes=%d does not fit in %d bytes" % (dsize, size))
    return dsize, head, boot


def stream_bytes(mm, dsize, pos, head):
    """Bytes number pos .. head-1 of the output stream (needs head - pos <= dsize)."""
    n = head - pos
    if n <= 0:
        return b""
    if n > dsize:
        raise ValueError("asked for %d bytes but the ring holds %d" % (n, dsize))
    s = pos % dsize
    first = min(n, dsize - s)
    out = rd(mm, HDR + s, first)
    if n > first:                                   # wrapped around the end of the data area
        out += rd(mm, HDR, n - first)
    return out


def to_text(b):
    # xil_printf sends "\r\n"; a chunk boundary may split it, so simply drop every '\r'
    return b.decode("utf-8", "replace").replace("\r", "")


def dump(mm, size, out):
    dsize, head, _boot = check_hdr(mm, size)
    start = max(0, head - dsize)
    if start:
        out.write("[r5log: %d older bytes were overwritten, showing the last %d]\n" % (start, dsize))
    out.write(to_text(stream_bytes(mm, dsize, start, head)))


def rproc_state():
    try:
        with open(RPROC_STATE) as f:
            return f.read().strip()
    except OSError as e:
        return "unknown (%s)" % e


def stop_rproc(out):
    try:
        with open(RPROC_STATE, "w") as f:
            f.write("stop")
        out.write("[r5log: R5 stopped (remoteproc0 %s)]\n" % rproc_state())
    except OSError as e:
        out.write("[r5log: could not stop the R5 (%s):  echo stop | sudo tee %s]\n" % (e, RPROC_STATE))


def ctl_state(ctl):
    """-> (magic, state byte, boot_id) of the R5's line of the control page."""
    b = rd(ctl, CTL_MAGIC, 12)
    return b[:4], b[4], struct.unpack("<I", b[8:12])[0]


def request_stop(ctl, ring_boot, out):
    """The first Ctrl-C of -f.  -> deadline for the R5's 'stopped', or None (nothing to wait for)."""
    rs = rproc_state()
    if rs != "running":
        out.write("\n[r5log: R5 is %s: nothing to stop]\n" % rs)
        return None
    magic, st, boot = ctl_state(ctl)
    if magic != ST_MAGIC or boot != ring_boot:      # a page an earlier run left: this firmware has none
        out.write("\n[r5log: this R5 firmware takes no stop command. %s]\n" % MANUAL)
        return None
    if st in (ST_STOPPED, ST_INIT_FAILED):          # no frames from the R5 any more, no drive enabled by it
        out.write("\n[r5log: the R5 is parked (%s)]\n" % ("stopped" if st == ST_STOPPED else "init failed"))
        stop_rproc(out)
        return None
    ctl[CTL_CMD] = CMD_STOP
    out.write("\n[r5log: Ctrl-C: stop requested (hold, servo off, drives to PRE-OP), then the R5 is stopped."
              " Ctrl-C again = stop waiting]\n")
    out.flush()
    return time.time() + STOP_WAIT_S


def follow(mm, size, out, interval=0.05, max_seconds=None, ctl=None):
    t0 = time.time()
    pos = None
    last_boot = None
    waiting = False
    deadline = None                 # set by the first Ctrl-C
    parked = False                  # the R5 reported it stopped: one more read, then stop it
    while True:
        try:
            try:
                dsize, head, boot = check_hdr(mm, size)
            except RingError as e:
                if not waiting:
                    out.write("[r5log: waiting for the R5 ... (%s)]\n" % e)
                    out.flush()
                    waiting = True
                dsize = None
            if dsize is not None:
                waiting = False
                if pos is None:                                   # first look
                    pos = max(0, head - dsize)
                    if pos:
                        out.write("[r5log: %d older bytes were overwritten]\n" % pos)
                    last_boot = boot
                elif boot != last_boot or head < pos:             # R5 restarted
                    out.write("\n[r5log: ---- R5 restarted (boot_id %d) ----]\n" % boot)
                    pos = max(0, head - dsize)
                    last_boot = boot
                elif head - pos > dsize:                          # we were too slow
                    lost = head - pos - dsize
                    out.write("\n[r5log: %d bytes lost (reader too slow)]\n" % lost)
                    pos = head - dsize
                if head > pos:
                    out.write(to_text(stream_bytes(mm, dsize, pos, head)))
                    out.flush()
                    pos = head
            if deadline is not None:
                if parked:                                        # its last lines are printed above
                    stop_rproc(out)
                    return 0
                parked = ctl_state(ctl)[1] in (ST_STOPPED, ST_INIT_FAILED)
                if not parked and time.time() >= deadline:
                    out.write("[r5log: the R5 did not report 'stopped' within %d s; it is left running. %s]\n"
                              % (STOP_WAIT_S, MANUAL))
                    return 1
            if max_seconds is not None and time.time() - t0 >= max_seconds:
                return 0
            time.sleep(interval)
        except KeyboardInterrupt:
            if ctl is None:
                return 0
            out = sys.stderr
            if deadline is not None:
                out.write("\n[r5log: stopped waiting; the R5 is left as it is. %s]\n" % MANUAL)
                return 1
            deadline = request_stop(ctl, last_boot, out)
            if deadline is None:
                return 0


def open_ring(args):
    if args.file:
        fd = os.open(args.file, os.O_RDONLY)
        offset = 0
    else:
        if args.addr % mmap.ALLOCATIONGRANULARITY:
            raise SystemExit("--addr must be a multiple of %d" % mmap.ALLOCATIONGRANULARITY)
        try:
            fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
        except PermissionError:
            raise SystemExit("cannot open /dev/mem: run with sudo")
        offset = args.addr
    try:
        return mmap.mmap(fd, args.size, mmap.MAP_SHARED, mmap.PROT_READ, offset=offset)
    except (OSError, ValueError) as e:
        raise SystemExit(
            "mmap of 0x%X (+0x%X) failed: %s\n"
            "  - /dev/mem may be restricted (CONFIG_STRICT_DEVMEM): check  grep -i reserved /proc/iomem\n"
            "  - or the address is wrong for this build (r5log.c R5LOG_ADDR)" % (offset, args.size, e))


def open_ctl(args):
    """The control page, read/write, for -f on the board; None for --file, or when it cannot be mapped."""
    if args.file:
        return None
    try:
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        return mmap.mmap(fd, CTL_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=CTL_ADDR)
    except OSError as e:
        sys.stderr.write("[r5log: no stop command (%s): Ctrl-C ends the viewer only]\n" % e)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="Read the Cortex-R5 console ring buffer (needs root).")
    ap.add_argument("-f", "--follow", action="store_true", help="keep printing new output")
    ap.add_argument("--header", action="store_true", help="print the ring header and exit")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=DEF_ADDR,
                    help="physical address of the ring (default 0x363FC000)")
    ap.add_argument("--size", type=lambda s: int(s, 0), default=DEF_SIZE,
                    help="ring size including the 32-byte header (default 0x4000)")
    ap.add_argument("--file", help="read a ring dump file instead of /dev/mem (for tests)")
    ap.add_argument("--max-seconds", type=float, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    try:
        faulthandler.enable()               # a fatal signal (SIGBUS ...) then prints the Python line, not just "Bus error"
    except (OSError, ValueError, RuntimeError):
        pass

    mm = open_ring(args)
    out = sys.stdout
    try:
        if args.header:
            magic, version, dsize, head, boot = parse_hdr(mm)
            out.write("magic=%r version=%d data_bytes=%d head=%d boot_id=%d\n"
                      % (magic, version, dsize, head, boot))
            return 0
        if args.follow:
            return follow(mm, args.size, out, max_seconds=args.max_seconds, ctl=open_ctl(args))
        else:
            dump(mm, args.size, out)
        out.flush()
    except RingError as e:
        sys.stderr.write("r5log: %s\n" % e)
        return 2
    except KeyboardInterrupt:
        return 0
    except BrokenPipeError:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
