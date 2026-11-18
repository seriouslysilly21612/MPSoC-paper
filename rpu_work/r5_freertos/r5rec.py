#!/usr/bin/env python3
"""r5rec.py -- the record of every cycle of the R5's EtherCAT task, from its ring into a timing log on Linux.

The R5 firmware (cycle_record.c) writes one record of 60 bytes per 1 kHz cycle into a ring at 0x36300000, in the DDR
that Linux reserves for the R5: when the cycle started after its release (t_w), when its send returned (t_s), the
receive and the send, and whether it was late, missed the send deadline or overran.  The record is that of the timing
log of the APU controller, so the file this script writes is read by the same tool:

  sudo python3 r5rec.py --out DIR [--seconds N]   the records from now on into DIR/timing_r5_<stamp>.bin, until
                                                  Ctrl-C (or N seconds); the counts into DIR/timing_r5_<stamp>.txt
  sudo python3 r5rec.py --head                    the head of the ring, and exit

  python3 goal-to-torque_MPSoC/tools/timing_summary.py DIR/timing_r5_<stamp>.bin --window op

Needs root (/dev/mem).  The R5 never waits for this script: it overwrites the oldest record, and the ring holds 16 s.
Records that were overwritten before they were read are counted as lost (none, unless this script is held up for
seconds).  A file is of one run of the R5: when the R5 starts again, the script ends.

Words only: /dev/mem maps the R5's memory as Device memory, where a slice copy faults (r5log.py rev2).  Every access
here is one aligned 32-bit load through a memoryview cast to 'I', as in r5cmd.py.
"""
import argparse
import mmap
import os
import platform
import signal
import struct
import sys
import time

RING_ADDR, RING_BYTES = 0x36300000, 0x000F0000          # RECORD_RING_ADDR, RECORD_RING_BYTES of cycle_record.h
MAGIC, VERSION = 0x43523552, 1                          # "R5RC"
HEAD_WORDS, RECORD_WORDS = 16, 15                       # 64 and 60 bytes
W_MAGIC, W_VERSION, W_RECORD_BYTES, W_CAPACITY, W_COUNT, W_BOOT, W_PERIOD, W_DEADLINE, W_HZ, W_CRC, W_BUILD = range(11)
BUILD_GEM3_OCM, BUILD_OPTIMIZED, BUILD_DC = 1, 2, 4
HEADER_BYTES = 1024                                     # of a timing log (indy7/src/TimingLog.h)
POLL_S = 0.05
# the two lines every timing log starts with (Indy7Ctrl.cpp); the R5 sets OP, WKC, LATE, MISSED, OVERRUN, STOP, BUS, SERVO
FORMAT_LINES = ("format=g2t-timing-1\nrecord_bytes=60\n"
                "fields=period,w_ns,s_ns,e_ns,dyn_ns,hl_ns,sim_ns,sync_ns,flags,reserved,recv_ns,send_ns,"
                "traj_w_ns,traj_e_ns,dyn_w_ns,dyn_e_ns\n"
                "flag_bits=OP:1,WKC:2,LATE:4,MISSED:8,OVERRUN:16,LAW:32,HL:64,TRACK:128,STOP:256,"
                "SIM:512,BUS:1024,SERVO:2048,BUSSERVO:4096,TEST:8192,R5:16384,NOSTATE:32768\n")


class RingError(Exception):
    pass


def ring_head(w):
    """the head of the ring as a dict; RingError when there is no ring of this layout"""
    if w[W_MAGIC] != MAGIC:
        raise RingError("no record ring at 0x%08x (magic 0x%08x): the R5 is not started, or its firmware has none"
                        % (RING_ADDR, w[W_MAGIC]))
    h = dict(version=w[W_VERSION], record_bytes=w[W_RECORD_BYTES], capacity=w[W_CAPACITY], count=w[W_COUNT],
             boot=w[W_BOOT], period_ns=w[W_PERIOD], deadline_ns=w[W_DEADLINE], counter_hz=w[W_HZ], cfg_crc=w[W_CRC],
             build=w[W_BUILD])
    if h["version"] != VERSION or h["record_bytes"] != 4 * RECORD_WORDS:
        raise RingError("a record ring of another layout (version %d, records of %d bytes)"
                        % (h["version"], h["record_bytes"]))
    if h["capacity"] == 0 or HEAD_WORDS + h["capacity"] * RECORD_WORDS > len(w):
        raise RingError("the head of the ring is damaged: %d records do not fit in %d bytes"
                        % (h["capacity"], 4 * len(w)))
    return h


def take(w, pos, boot):
    """The records number pos and up that the ring holds now.  -> (records as bytes, the number of the next one,
    records lost), or None when the R5 started again (another boot_id, or its count went back)."""
    h = ring_head(w)
    count, cap = h["count"], h["capacity"]
    lost = 0
    if count - pos > cap:                               # overwritten before this read
        lost = count - pos - cap
        pos = count - cap
    recs = []
    for k in range(pos, count):
        base = HEAD_WORDS + (k % cap) * RECORD_WORDS
        recs.append([w[base + i] for i in range(RECORD_WORDS)])
    # The R5 went on while this read: it may be writing record number `after` now, into the slot of record
    # after - cap.  That one and those before it are not to be trusted.
    after = w[W_COUNT]
    if w[W_BOOT] != boot or count < pos or after < count:          # the R5 started again, before or during this read
        return None
    torn = min(max(0, after - cap + 1 - pos), len(recs))
    return b"".join(struct.pack("<15I", *r) for r in recs[torn:]), count, lost + torn


def file_header(h, first):
    """the text head of the timing log: the keys of the APU's log that tools/timing_summary.py reads, and what the
    ring says about the build of the firmware"""
    b = h["build"]
    text = FORMAT_LINES + (
        "mode=r5\nperiod_ns=%d\ndeadline_ns=%d\n"
        "source=the EtherCAT task of the R5 firmware, record ring 0x%08x, boot %d, from record %d\n"
        "counter_hz=%d\ncfg_crc=0x%08X\ntarget=%s\ngem3_buffers=%s\noptimized=%d\nsync_mode=%s\n"
        "host=%s\nkernel=%s\ncreated=%s\n"
        % (h["period_ns"], h["deadline_ns"], RING_ADDR, h["boot"], first, h["counter_hz"], h["cfg_crc"],
           {0: "l7n", 1: "indy7"}.get((b >> 8) & 0xFF, "?"), "ocm" if b & BUILD_GEM3_OCM else "ddr",
           1 if b & BUILD_OPTIMIZED else 0, "DC" if b & BUILD_DC else "FREE_RUN", platform.node(),
           platform.release(), time.strftime("%Y-%m-%d %H:%M:%S %Z")))
    return (text.ljust(HEADER_BYTES - 1) + "\n").encode()


def follow(w, out_dir, seconds, sleep=time.sleep):
    """records from now on into a new file of out_dir, until Ctrl-C, `seconds`, or the R5 starts again.
    -> (path, records written, records lost, why it ended early or None)"""
    h = ring_head(w)
    pos, boot = h["count"], h["boot"]
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, "timing_r5_" + time.strftime("%Y%m%d_%H%M%S"))
    written = lost = 0
    why = None
    end = None if seconds is None else time.monotonic() + seconds
    with open(stem + ".bin", "wb") as f:
        f.write(file_header(h, pos))
        print("r5rec: the cycles of the R5 from record %d on -> %s.bin (Ctrl-C ends)" % (pos, stem), flush=True)
        try:
            while end is None or time.monotonic() < end:
                sleep(POLL_S)
                got = take(w, pos, boot)
                if got is None:
                    why = "the R5 started again"
                    break
                data, pos, missed = got
                f.write(data)
                written += len(data) // (4 * RECORD_WORDS)
                lost += missed
        except KeyboardInterrupt:
            pass
        except RingError as e:                          # the head went away: the R5 is starting again
            why = str(e)
    with open(stem + ".txt", "w") as f:
        f.write("file=%s.bin\nrecords=%d\ndropped=%d\nrecord_bytes=%d\nheader_bytes=%d\n"
                % (stem, written, lost, 4 * RECORD_WORDS, HEADER_BYTES))
        if why:
            f.write("ended_early=%s\n" % why)
    return stem + ".bin", written, lost, why


def on_sigterm(signum, frame):
    raise KeyboardInterrupt                             # a tool that ends this script: as Ctrl-C


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", help="directory of the timing log")
    ap.add_argument("--seconds", type=float, help="end after this many seconds (default: at Ctrl-C)")
    ap.add_argument("--head", action="store_true", help="print the head of the ring and exit")
    a = ap.parse_args(argv)
    if not a.head and not a.out:
        ap.error("--out DIR or --head")
    try:
        fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    except OSError as e:
        sys.exit("r5rec: /dev/mem: %s (sudo?)" % e)
    mm = mmap.mmap(fd, RING_BYTES, mmap.MAP_SHARED, mmap.PROT_READ, offset=RING_ADDR)
    w = memoryview(mm).cast("I")
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        if a.head:
            h = ring_head(w)
            print("record ring 0x%08x: %d records of %d bytes; %d written; boot %d; period %d ns, send deadline %d ns; "
                  "counter %d ticks/s; check value 0x%08X; build 0x%X"
                  % (RING_ADDR, h["capacity"], h["record_bytes"], h["count"], h["boot"], h["period_ns"],
                     h["deadline_ns"], h["counter_hz"], h["cfg_crc"], h["build"]))
            return 0
        path, written, lost, why = follow(w, a.out, a.seconds)
    except RingError as e:
        sys.exit("r5rec: %s" % e)
    print("r5rec: %d cycles written to %s%s%s" % (written, path, "; ended early: " + why if why else "",
                                                 "; %d RECORDS LOST: the reader did not keep up" % lost if lost else ""))
    return 0 if lost == 0 and why is None else 1


if __name__ == "__main__":
    sys.exit(main())
