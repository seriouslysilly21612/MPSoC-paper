# The tool tag in rendered images of a known pose (target_3d_pkg/tool_tag.py). No ROS.
import os
import sys
import threading
import time

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from target_3d_pkg.tool_tag import (  # noqa: E402
    TagResults, TagSearch, ToolTagFinder, gray_of_image, marker_points)

W, H = 848, 480
K = np.array(
    [[606.0, 0.0, 424.0], [0.0, 606.0, 240.0], [0.0, 0.0, 1.0]]
)  # about the D435i color at 848x480


def marker_image(tag_id, px=240):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, tag_id, px)
    return cv2.aruco.drawMarker(d, tag_id, px)


def rot(rx, ry, rz):
    """the tag facing the camera (its z towards it: a half turn about x), turned by (rx, ry, rz)"""
    face = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]
    return cv2.Rodrigues(np.array([rx, ry, rz], dtype=np.float64))[0] @ face


def render(tags, k=K):
    """a white image with each (tag_id, size_m, R, t) drawn at its pose"""
    img = np.full((H, W), 255, np.uint8)
    for tag_id, size_m, r, t in tags:
        m = marker_image(tag_id)
        n = m.shape[0]
        p3 = (r @ marker_points(size_m).T).T + t
        uv = (k @ p3.T).T
        uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float32)
        # TL, TR, BR, BL of the black square: its edges lie half a pixel outside the centres of its
        # pixels
        src = np.array([[0, 0], [n, 0], [n, n], [0, n]], dtype=np.float32) - 0.5
        warp = cv2.warpPerspective(
            m,
            cv2.getPerspectiveTransform(src, uv),
            (W, H),
            flags=cv2.INTER_LINEAR,
            borderValue=255,
        )
        mask = cv2.warpPerspective(
            np.full_like(m, 255),
            cv2.getPerspectiveTransform(src, uv),
            (W, H),
            flags=cv2.INTER_NEAREST,
            borderValue=0,
        )
        img[mask > 0] = warp[mask > 0]
    return cv2.GaussianBlur(img, (3, 3), 0.6)


def finder(tag_id=0, size_m=0.065, scale=1.0):
    f = ToolTagFinder(tag_id, size_m, whole_image_scale=scale)
    f.set_camera(K.flatten(), [0.0] * 5)
    return f


@pytest.mark.parametrize(
    't, r',
    [
        ((0.05, -0.03, 0.80), (0.2, -0.3, 0.1)),
        ((-0.15, 0.10, 1.00), (-0.4, 0.2, 2.0)),
        ((0.20, 0.05, 0.60), (0.0, 0.5, -1.0)),
    ],
)
@pytest.mark.parametrize('scale', [1.0, 0.5])
def test_the_centre_of_a_rendered_tag(t, r, scale):
    t = np.array(t)
    res = finder(scale=scale).find(render([(0, 0.065, rot(*r), t)]))
    assert res.found, res.reason
    assert np.abs(res.p_cam[:2] - t[:2]).max() < 2e-3  # across the view: within 2 mm at 0.6-1.0 m
    # in depth, the weak axis of one tag: a few tenths of a pixel on a 40-65 px square are 1 % of
    # the distance
    assert abs(res.p_cam[2] - t[2]) < 12e-3
    assert res.reproj_px < 1.0


def test_the_size_scales_the_distance():
    # the same image read with a wrong size: the centre is off by that ratio (why the side must be
    # measured)
    t = np.array([0.0, 0.0, 0.8])
    img = render([(0, 0.065, rot(0.1, 0.1, 0.0), t)])
    res = finder(size_m=0.080).find(img)
    assert res.found and abs(res.p_cam[2] / t[2] - 0.080 / 0.065) < 0.01


def test_other_ids_and_no_tag():
    assert finder().find(np.full((H, W), 255, np.uint8)).reason == 'not_found'
    res = finder(tag_id=0).find(render([(5, 0.065, rot(0, 0, 0), np.array([0.0, 0.0, 0.8]))]))
    assert not res.found and res.reason == 'other_id'
    f = finder(tag_id=0)
    f.find(render([(5, 0.065, rot(0, 0, 0), np.array([0.0, 0.0, 0.8]))]))
    assert f.seen_ids == (5,)


def test_the_tool_tag_among_others():
    t0 = np.array([0.1, 0.0, 0.8])
    img = render(
        [(0, 0.065, rot(0, 0.2, 0), t0), (7, 0.065, rot(0, 0, 0), np.array([-0.15, 0.0, 0.8]))]
    )
    res = finder(tag_id=0).find(img)
    assert res.found and np.linalg.norm(res.p_cam - t0) < 12e-3


def test_two_tags_with_its_id():
    img = render(
        [
            (0, 0.065, rot(0, 0, 0), np.array([0.12, 0.0, 0.8])),
            (0, 0.065, rot(0, 0, 0), np.array([-0.12, 0.0, 0.8])),
        ]
    )
    assert finder().find(img).reason == 'several'


def test_the_window_then_the_whole_image():
    f = finder()
    t1 = np.array([0.0, 0.0, 0.8])
    a = f.find(render([(0, 0.065, rot(0.1, 0, 0), t1)]), t_s=0.0)
    assert a.found and not a.window
    t2 = t1 + np.array([0.01, 0.005, 0.0])  # a small move: found in the window
    b = f.find(render([(0, 0.065, rot(0.1, 0, 0), t2)]), t_s=0.033)
    assert b.found and b.window and np.linalg.norm(b.p_cam - t2) < 12e-3
    t3 = np.array([-0.25, 0.10, 0.9])  # far away: the window misses it, the whole image does not
    c = f.find(render([(0, 0.065, rot(0.1, 0, 0), t3)]), t_s=0.066)
    assert c.found and not c.window and np.linalg.norm(c.p_cam - t3) < 12e-3
    d = f.find(render([(0, 0.065, rot(0.1, 0, 0), t3)]), t_s=5.0)  # the window is too old
    assert d.found and not d.window


def test_the_whole_image_goes_to_the_plain_detector_only():
    # AprilTag refinement of a whole image takes about 20 times as long: it sees windows only
    f = finder()
    seen = {'fine': [], 'coarse': []}

    def recording(name):
        inner = getattr(f, name)
        return lambda gray: (seen[name].append(gray.shape), inner(gray))[1]

    f.fine, f.coarse = recording('fine'), recording('coarse')
    f.in_window = [f.fine, f.coarse]
    t = np.array([0.05, -0.03, 0.80])
    res = f.find(render([(0, 0.065, rot(0.2, -0.3, 0.1), t)]), t_s=0.0)
    assert res.found and not res.window and abs(res.p_cam[2] - t[2]) < 12e-3
    assert seen['coarse'] == [(H, W)]
    assert len(seen['fine']) == 1 and seen['fine'][0][0] < H / 2 and seen['fine'][0][1] < W / 2
    f.find(np.full((H, W), 255, np.uint8), t_s=5.0)  # no tag: the plain detector alone
    assert seen['coarse'][-1] == (H, W) and len(seen['fine']) == 1


def test_after_a_whole_image_without_the_tag_the_next_one_waits():
    f = finder()
    calls = []
    inner = f.coarse
    f.coarse = lambda gray: (calls.append(gray.shape), inner(gray))[1]
    f.in_window = [f.fine, f.coarse]
    blank = np.full((H, W), 255, np.uint8)
    tag = render([(0, 0.065, rot(0, 0, 0), np.array([0.0, 0.0, 0.8]))])
    assert f.find(blank, t_s=10.00).reason == 'not_found' and len(calls) == 1
    res = f.find(tag, t_s=10.05)  # the tag is back, but the whole image is not searched yet
    assert not res.found and res.reason == 'not_searched' and len(calls) == 1
    assert f.find(tag, t_s=10.25).found  # 0.2 s after the miss: searched, found
    assert f.find(blank, t_s=20.0).reason == 'not_found'
    assert f.find(blank, t_s=19.9).reason == 'not_found'  # a time before the miss: searched
    assert f.find(blank).reason == 'not_found'  # without a time: no wait
    assert finder().find(blank).reason == 'not_found'


@pytest.mark.parametrize('z', [0.55, 0.80, 1.10])
def test_the_whole_image_at_half_size_then_the_corners_at_full_size(z):
    # the search of I-134: the copy is a quarter of the pixels; the corners come from the full
    # image, so the centre is as good as with the whole image at full size
    f = finder(scale=0.5)
    seen = []
    inner = f.coarse
    f.coarse = lambda gray: (seen.append(gray.shape), inner(gray))[1]
    f.in_window = [f.fine, f.coarse]
    t = np.array([0.08, -0.04, z])
    img = render([(0, 0.065, rot(0.2, -0.3, 0.1), t)])
    res = f.find(img, t_s=0.0)
    full = finder().find(img)
    assert res.found and full.found and not res.window
    assert seen[0] == (H // 2, W // 2) and seen[1][0] < H / 2      # the copy, then a window
    assert np.linalg.norm(res.p_cam - full.p_cam) < 1e-3
    assert f.find(np.full((H, W), 255, np.uint8), t_s=5.0).reason == 'not_found'


def test_no_camera_info_and_bad_size():
    assert ToolTagFinder(0, 0.065).find(np.zeros((H, W), np.uint8)).reason == 'no_camera_info'
    with pytest.raises(ValueError):
        ToolTagFinder(0, 0.0)
    with pytest.raises(ValueError):
        ToolTagFinder(0, 0.065, refine='bogus')
    for bad in (0.2, 1.5):
        with pytest.raises(ValueError):
            ToolTagFinder(0, 0.065, whole_image_scale=bad)


def test_gray_of_image():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[..., 0] = 200  # red
    padded = np.zeros((2, 12), np.uint8)  # step 12 > 9: padded rows
    padded[:, :9] = rgb.reshape(2, 9)
    g_rgb = gray_of_image(padded.tobytes(), 2, 3, 12, 'rgb8')
    g_bgr = gray_of_image(padded.tobytes(), 2, 3, 12, 'bgr8')
    assert g_rgb.shape == (2, 3) and g_rgb[0, 0] == cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)[0, 0]
    assert g_bgr[0, 0] != g_rgb[0, 0]  # the channel order matters
    assert gray_of_image(b'\x01\x02\x03\x04', 2, 2, 2, 'mono8').tolist() == [[1, 2], [3, 4]]
    assert gray_of_image(b'\x00' * 8, 2, 2, 4, '16UC1') is None
    assert gray_of_image(b'\x00' * 3, 2, 2, 2, 'mono8') is None  # short data


# ----- the search beside the inference (I-122) -----


def test_the_search_thread_finds_the_tag_of_each_image_it_is_offered():
    got, done = [], threading.Event()

    def on_done(key, res, ms):
        got.append((key, res.found, ms))
        if len(got) == 2:
            done.set()

    s = TagSearch(finder(), on_done)
    img = render([(0, 0.065, rot(0.1, 0.0, 0.0), np.array([0.0, 0.0, 0.8]))])
    s.offer('a', 1_000_000_000, img)
    deadline = time.monotonic() + 5.0
    while not got and time.monotonic() < deadline:    # the first done before the second comes
        time.sleep(0.01)
    s.offer('b', 1_033_000_000, np.repeat(img[:, :, None], 3, axis=2))   # BGR too
    assert done.wait(5.0)
    s.stop()
    assert [(k, f) for k, f, _ in got] == [('a', True), ('b', True)]
    assert all(ms > 0 for _, _, ms in got)


def test_an_image_offered_while_one_waits_replaces_it_and_the_replaced_one_is_told_busy():
    gate, got = threading.Event(), []

    class Slow:
        def find(self, gray, t_s):
            gate.wait(5.0)
            return finder().find(gray, t_s)

    s = TagSearch(Slow(), lambda key, res, ms: got.append((key, res.reason)))
    img = render([])
    s.offer('first', 1, img)          # the thread takes it and waits in find
    time.sleep(0.1)
    s.offer('second', 2, img)         # waits ...
    s.offer('third', 3, img)          # ... and is replaced: every image gets one result
    assert ('second', 'busy') in got
    gate.set()
    deadline = time.monotonic() + 5.0
    while len(got) < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    s.stop()
    assert sorted(k for k, _ in got) == ['first', 'second', 'third']
    assert dict(got)['first'] == 'not_found'
    assert dict(got)['third'] in ('not_found', 'not_searched')


def test_a_search_that_raises_gives_search_error_and_the_thread_goes_on(capsys):
    # I-127: before, any error but cv2.error ended the thread, and every later capture went
    # without its result
    class Raises:
        def __init__(self):
            self.n = 0

        def find(self, gray, t_s):
            self.n += 1
            if self.n == 2:
                raise ValueError('a bad image')
            return finder().find(gray, t_s)

    got = []
    s = TagSearch(Raises(), lambda key, res, ms: got.append((key, res.reason)))
    img = render([])
    for k in range(4):
        s.offer(k, k, img)
        deadline = time.monotonic() + 5.0
        while len(got) <= k and time.monotonic() < deadline:   # one at a time: none is busy
            time.sleep(0.01)
    s.stop()
    assert [k for k, _ in got] == [0, 1, 2, 3] and got[1][1] == 'search_error'
    assert capsys.readouterr().err.count('ValueError: a bad image') == 1   # one traceback


def test_the_result_of_a_capture_is_waited_for_and_found_by_its_stamp():
    r = TagResults(keep_s=1.0)
    threading.Timer(0.05, lambda: r.put(5_000_000_300, 'tag')).start()   # 0.3 us off: the same
    t0 = time.monotonic()
    assert r.wait(5_000_000_000, 1.0) == 'tag' and time.monotonic() - t0 < 0.5
    assert r.wait(5_033_000_000, 0.0) is None          # the next image: none, at once
    t0 = time.monotonic()
    assert r.wait(5_033_000_000, 0.05) is None and time.monotonic() - t0 >= 0.045


def test_old_results_are_dropped():
    r = TagResults(keep_s=0.5)
    r.put(1_000_000_000, 'old')
    r.put(1_400_000_000, 'mid')
    r.put(1_600_000_000, 'new')        # 0.6 s after the first
    assert r.wait(1_000_000_000, 0.0) is None and r.wait(1_400_000_000, 0.0) == 'mid'
