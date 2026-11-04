"""shm_frame: the /dev/shm frame handoff (node writer -> worker reader).

Guards the only novel logic — growth-in-place keeping the inode, and remap on
the reader side. A regression here shows up on the robot as the worker silently
detecting on a STALE frame, which no runtime error would ever surface.
Pure stdlib+numpy, no ROS/DPU needed:
    python3 -m pytest src/vitis_ai_detector_pkg/test/test_shm_frame.py
"""
import os

import numpy as np
import pytest

from vitis_ai_detector_pkg.shm_frame import ShmFrameReader, ShmFrameWriter


def _frame(h, w, seed):
    return (np.arange(h * w * 3, dtype=np.uint32) + seed).astype(
        np.uint8).reshape(h, w, 3)


@pytest.fixture
def pair(tmp_path):
    w = ShmFrameWriter(str(tmp_path / "frame.shm"))
    yield w, ShmFrameReader()
    w.unlink()


def test_roundtrip_and_readonly(pair):
    w, r = pair
    a = _frame(480, 848, seed=7)
    w.write(a)
    v = r.view(w.path, a.nbytes).reshape(a.shape)
    assert np.array_equal(v, a)
    assert not v.flags.writeable        # worker must not be able to corrupt it


def test_rewrite_without_remap_is_seen(pair):
    # The serial protocol rewrites the same buffer every frame; the reader's
    # CACHED mapping must observe the new content (same file, same mapping).
    w, r = pair
    w.write(_frame(480, 848, seed=1))
    r.view(w.path, 480 * 848 * 3)
    b = _frame(480, 848, seed=99)
    w.write(b)
    assert np.array_equal(r.view(w.path, b.nbytes).reshape(b.shape), b)


def test_growth_keeps_inode_and_reader_remaps(pair):
    w, r = pair
    w.write(_frame(240, 424, seed=3))
    ino = os.stat(w.path).st_ino
    r.view(w.path, 240 * 424 * 3)
    big = _frame(480, 848, seed=5)      # larger frame -> writer grows in place
    w.write(big)
    assert os.stat(w.path).st_ino == ino    # SAME inode — worker stays attached
    assert np.array_equal(r.view(w.path, big.nbytes).reshape(big.shape), big)


def test_shrink_reuses_mapping(pair):
    # Smaller frame after a big one: no ftruncate, view is just a prefix.
    w, r = pair
    w.write(_frame(480, 848, seed=5))
    small = _frame(240, 424, seed=8)
    w.write(small)
    assert np.array_equal(
        r.view(w.path, small.nbytes).reshape(small.shape), small)


def test_reader_rejects_undersized_file(pair):
    w, r = pair
    w.write(_frame(240, 424, seed=3))
    with pytest.raises(RuntimeError, match="smaller than frame"):
        r.view(w.path, 480 * 848 * 3)
