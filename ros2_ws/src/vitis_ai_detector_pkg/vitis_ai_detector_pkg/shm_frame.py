"""Frame handoff detector node -> worker over a /dev/shm mmap (2026-08-05).

Why: the stdin pipe moved every 848x480x3 frame (1.22 MB) through tobytes() +
a 64 KB kernel pipe + read_exact() reassembly — 3-4 full copies and dozens of
context switches per frame, measured as ipc_overhead_ms ~9-12 ms (~20% of the
detect budget, docs/vision/throughput.md §4 방법 0). A file in /dev/shm mapped
by both sides reduces that to ONE memcpy: the node writes the frame into the
mapping, the worker wraps it in a read-only numpy view and preprocesses from it
in place. The JSON request/response control channel stays on the pipe (tiny).

No locking needed: the protocol is strictly serial. The node writes the buffer
only BEFORE sending a request; the worker reads it only BETWEEN request and
response; the node never touches it while waiting.

Growth keeps the inode: the writer ftruncates the SAME file and remaps, and the
reader remaps when the requested size exceeds its mapping. Never unlink the
file while a worker may still attach (a recreated file is a NEW inode and the
worker would silently read stale frames) — the node unlinks only at shutdown.
"""

import mmap
import os

import numpy as np


class ShmFrameWriter:
    """Node side: owns the file, grows it in place when the frame grows."""

    def __init__(self, path):
        self.path = path
        self._mm = None
        self._np = None
        self._size = 0

    def write(self, arr):
        """One memcpy of arr (uint8, C-contiguous) into the mapping."""
        n = arr.nbytes
        if self._mm is None or self._size < n:
            self._unmap()
            fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o600)
            try:
                os.ftruncate(fd, n)
                self._mm = mmap.mmap(fd, n)
            finally:
                os.close(fd)
            self._np = np.frombuffer(self._mm, dtype=np.uint8)
            self._size = n
        self._np[:n] = arr.reshape(-1)
        return n

    def _unmap(self):
        if self._mm is not None:
            self._np = None      # release the buffer export before close()
            self._mm.close()
            self._mm = None
            self._size = 0

    def unlink(self):
        """Shutdown only — see the inode warning in the module docstring."""
        self._unmap()
        try:
            os.unlink(self.path)
        except OSError:
            pass


class ShmFrameReader:
    """Worker side: attaches read-only, remaps on path change or growth."""

    def __init__(self):
        self.path = None
        self._mm = None
        self._np = None
        self._size = 0

    def view(self, path, nbytes):
        """Flat read-only uint8 view of the first nbytes of the mapping."""
        if self._mm is None or path != self.path or self._size < nbytes:
            if self._mm is not None:
                self._np = None
                self._mm.close()
                self._mm = None
            fd = os.open(path, os.O_RDONLY)
            try:
                size = os.fstat(fd).st_size
                if size < nbytes:
                    raise RuntimeError(
                        f"shm file {path} smaller than frame: {size} < {nbytes}")
                self._mm = mmap.mmap(fd, size, prot=mmap.PROT_READ)
            finally:
                os.close(fd)
            self.path = path
            self._size = size
            self._np = np.frombuffer(self._mm, dtype=np.uint8)
        return self._np[:nbytes]
