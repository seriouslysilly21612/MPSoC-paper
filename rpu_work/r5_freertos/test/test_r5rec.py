"""Host test of r5rec.py against a simulated record ring (a bytearray as the R5's memory), and against the ring that
the firmware's own code writes (test_cycle_record.c, when gcc is there).  python3 test/test_r5rec.py"""
import contextlib, io, os, shutil, struct, subprocess, sys, tempfile, time
sys.dont_write_bytecode = True                          # no __pycache__ beside the scripts
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "goal-to-torque_MPSoC", "tools"))
import r5rec as r
import timing_summary as ts

CAP = 16
REC = struct.Struct("<IiiiiiiIHHiiiiii")                # the record of TimingLog.h
assert REC.size == 60 == ts.REC.size and REC.format == ts.REC.format


def ring(cap=CAP, boot=3):
    buf = bytearray(64 + cap * 60)
    w = memoryview(buf).cast("I")
    w[r.W_VERSION], w[r.W_RECORD_BYTES], w[r.W_CAPACITY], w[r.W_BOOT] = 1, 60, cap, boot
    w[r.W_PERIOD], w[r.W_DEADLINE], w[r.W_HZ], w[r.W_CRC], w[r.W_BUILD] = 1000000, 400000, 99999000, 0x07AE943D, 5
    w[r.W_MAGIC] = r.MAGIC
    return w


def record(k, w_ns=4000, s_ns=60000, flags=0x401):     # BUS | OP
    return REC.pack(k, w_ns, s_ns, s_ns, -1, -1, -1, 0xFFFFFFFF, flags, 0, 15000, 12000, -1, -1, -1, -1)


def push(w, n=1, **kw):                                 # what cycle_record_push() does
    for _ in range(n):
        k = w[r.W_COUNT]
        base = r.HEAD_WORDS + (k % w[r.W_CAPACITY]) * r.RECORD_WORDS
        for i, v in enumerate(struct.unpack("<15I", record(k, **kw))):
            w[base + i] = v
        w[r.W_COUNT] = k + 1


def periods(data):
    return [x[0] for x in REC.iter_unpack(data)]


def refused(w):
    try:
        r.ring_head(w)
    except r.RingError:
        return True
    return False


# the head: no ring, another layout, a capacity that does not fit
w = ring(); w[r.W_MAGIC] = 0xA5A5A5A5; assert refused(w)
w = ring(); w[r.W_VERSION] = 2; assert refused(w)
w = ring(); w[r.W_RECORD_BYTES] = 64; assert refused(w)
w = ring(); w[r.W_CAPACITY] = 0; assert refused(w)
w = ring(); w[r.W_CAPACITY] = CAP + 1; assert refused(w)
w = ring(); h = r.ring_head(w)
assert h["capacity"] == CAP and h["count"] == 0 and h["boot"] == 3 and h["deadline_ns"] == 400000

# records in order, a second read takes only the new ones, and the ring wraps
assert r.take(w, 0, 3) == (b"", 0, 0)
push(w, 5)
data, pos, lost = r.take(w, 0, 3)
assert periods(data) == [0, 1, 2, 3, 4] and pos == 5 and lost == 0 and data[:60] == record(0)
push(w, 14)                                             # 19 written: slots 0..2 hold records 16..18
data, pos, lost = r.take(w, pos, 3)
assert periods(data) == list(range(5, 19)) and pos == 19 and lost == 0

# a reader that fell behind: what was overwritten is lost, and so is the record whose slot the R5 may be writing
push(w, 40)                                             # 59 written, the ring holds 43..58
data, pos, lost = r.take(w, 19, 3)
assert periods(data) == list(range(44, 59)) and pos == 59 and lost == 25
push(w, CAP)                                            # exactly one ring behind: record 59 is in the slot of record 75
data, pos, lost = r.take(w, 59, 3)
assert periods(data) == list(range(60, 75)) and pos == 75 and lost == 1


class Racing:
    """a ring whose R5 writes `more` records while the reader copies: the count is higher from its second read on"""
    def __init__(self, w, more):
        self.w, self.more, self.reads = w, more, 0
    def __len__(self):
        return len(self.w)
    def __getitem__(self, i):
        if i == r.W_COUNT:
            self.reads += 1
            return self.w[i] + (self.more if self.reads > 1 else 0)
        return self.w[i]


w = ring(); push(w, 10)
data, pos, lost = r.take(Racing(w, 6), 0, 3)           # 16 written by the end: the slot of record 0 is being written
assert periods(data) == list(range(1, 10)) and pos == 10 and lost == 1
data, pos, lost = r.take(Racing(w, 9), 0, 3)           # 19 by the end: records 0..3 are not to be trusted
assert periods(data) == list(range(4, 10)) and pos == 10 and lost == 4
data, pos, lost = r.take(Racing(w, 100), 0, 3)         # all of them overwritten meanwhile
assert data == b"" and pos == 10 and lost == 10
data, pos, lost = r.take(Racing(w, 5), 0, 3)           # 15 by the end: nothing of 0..9 is touched
assert periods(data) == list(range(0, 10)) and lost == 0

# the R5 started again: another boot_id, a count that went back, or one that goes back during the read
w = ring(); push(w, 8)
assert r.take(w, 3, 4) is None
assert r.take(w, 9, 3) is None
assert r.take(Racing(w, -6), 0, 3) is None

# follow(): a file that tools/timing_summary.py reads; records before the start are not in it
tmp = tempfile.mkdtemp()
try:
    # the device: the UIO node "r5-record" whatever its number, at offset 0; without it /dev/mem at the ring
    sysfs = os.path.join(tmp, "sys")
    for number, name in ((0, "axi-pmon"), (4, "ocm-link")):
        os.makedirs(os.path.join(sysfs, "class/uio/uio%d" % number))
        open(os.path.join(sysfs, "class/uio/uio%d/name" % number), "w").write(name + "\n")
    assert r.ring_device(sysfs, "/dev") == ("/dev/mem", r.RING_ADDR)
    os.makedirs(os.path.join(sysfs, "class/uio/uio5"))
    open(os.path.join(sysfs, "class/uio/uio5/name"), "w").write("r5-record\n")
    assert r.ring_device(sysfs, "/dev") == ("/dev/uio5", 0)

    w = ring(cap=64); push(w, 7)
    def r5_runs(seconds):                               # 20 cycles per poll, every 9th past the send deadline
        for _ in range(20):
            k = w[r.W_COUNT]
            push(w, 1, s_ns=420000 if k % 9 == 0 else 60000, flags=0x409 if k % 9 == 0 else 0x401)
        time.sleep(0.002)
    with contextlib.redirect_stdout(io.StringIO()):
        path, written, lost, why = r.follow(w, tmp, 0.1, sleep=r5_runs)
    assert why is None and lost == 0 and written > 0 and written % 20 == 0
    head, n, data = ts.read_log(path)
    assert n == written and os.path.getsize(path) == 1024 + 60 * n
    assert head["mode"] == "r5" and head["deadline_ns"] == "400000" and head["period_ns"] == "1000000"
    assert head["gem3_buffers"] == "ocm" and head["optimized"] == "0" and head["sync_mode"] == "DC" and head["target"] == "l7n"
    assert head["cfg_crc"] == "0x07AE943D" and head["counter_hz"] == "99999000"
    assert periods(data) == list(range(7, 7 + n))
    txt = dict(l.strip().split("=", 1) for l in open(path[:-4] + ".txt"))
    assert txt["records"] == str(n) and txt["dropped"] == "0" and "ended_early" not in txt
    missed = sum(1 for k in range(7 + 10, 7 + n) if k % 9 == 0)      # the summary leaves its first 10 out here
    with contextlib.redirect_stdout(io.StringIO()):
        rc = ts.main([path, "--window", "op", "--skip", "10", "--out", tmp])
    text = open(os.path.join(tmp, "timing_summary.txt")).read()
    assert rc == 1 and ("%d missed send deadlines" % missed) in text and ("FAIL  %d missed" % missed) in text
    assert "EtherCAT t_w + t_s" in text and "Delta T_send" in text and "mode r5" in text

    # the R5 starts again during the read: the file ends there
    w = ring(cap=64)
    def r5_restarts(seconds):
        push(w, 5)
        if w[r.W_COUNT] >= 15:
            w[r.W_BOOT] = 4; w[r.W_COUNT] = 0
    with contextlib.redirect_stdout(io.StringIO()):
        path, written, lost, why = r.follow(w, tmp, None, sleep=r5_restarts)
    assert why == "the R5 started again" and written == 10
    assert "ended_early=the R5 started again" in open(path[:-4] + ".txt").read()

    # the ring as the firmware's own code writes it (cycle_record.c, 16 slots, 37 records pushed, boot 7)
    if shutil.which("gcc"):
        exe, img = os.path.join(tmp, "t_rec"), os.path.join(tmp, "ring.bin")
        root = os.path.join(HERE, "..")
        subprocess.check_call(["gcc", "-std=gnu11", "-DHOST_TEST", "-DRECORD_RING_BYTES=1024", "-Itest/stub", "-Isrc",
                               "-Isrc/oshw/freertos", "test/test_cycle_record.c", "src/cycle_record.c", "-lm", "-o", exe],
                              cwd=root)
        subprocess.check_call([exe, img], stdout=subprocess.DEVNULL)
        w = memoryview(bytearray(open(img, "rb").read())).cast("I")
        h = r.ring_head(w)
        assert h["capacity"] == 16 and h["count"] == 37 and h["boot"] == 7 and h["counter_hz"] == 99999000
        data, pos, lost = r.take(w, 0, 7)
        assert periods(data) == list(range(22, 37)) and pos == 37 and lost == 22
        for x in REC.iter_unpack(data):
            k = x[0]
            assert x[1] == 4000 + k and x[2] == 60000 + k and x[8] == (0x401 | (8 if k % 7 == 3 else 0)) and x[4] == -1
        print("the ring of cycle_record.c read")
    else:
        print("NOT CHECKED: the ring of cycle_record.c (no gcc)")
finally:
    shutil.rmtree(tmp)
print("test_r5rec OK")
