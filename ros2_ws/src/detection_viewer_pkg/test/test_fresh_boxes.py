"""fresh_boxes: which boxes may be drawn on a given frame (smooth mode).

The failure this guards is silent and misleading: a stalled detector leaving its
last boxes frozen on live 30 fps video, which reads as "still detecting". The
function itself is pure, but importing the node module needs the ROS env:
    source /opt/ros/humble/setup.bash && source install/setup.bash
    PYTHONPATH="$PWD/src/detection_viewer_pkg:$PYTHONPATH" \
        python3 -m pytest src/detection_viewer_pkg/test/test_fresh_boxes.py
"""
from types import SimpleNamespace

from detection_viewer_pkg.detection_viewer_node import fresh_boxes

MAX_AGE = 0.5


def _dets(sec, nanosec=0):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=sec, nanosec=nanosec)))


def test_none_before_any_detection_arrives():
    assert fresh_boxes((100, 0), None, MAX_AGE) == (None, None)


def test_normal_lag_is_drawn_with_its_age():
    # The detection for a frame ~107 ms back — the steady state of smooth mode.
    dets, age = fresh_boxes((100, 107_000_000), _dets(100), MAX_AGE)
    assert dets is not None
    assert abs(age - 0.107) < 1e-6


def test_stalled_detector_hides_boxes():
    # Detector died 2 s ago: must go blank, not freeze the last boxes.
    assert fresh_boxes((102, 0), _dets(100), MAX_AGE) == (None, None)


def test_boundary_just_inside_and_just_outside():
    assert fresh_boxes((100, 499_000_000), _dets(100), MAX_AGE)[0] is not None
    assert fresh_boxes((100, 501_000_000), _dets(100), MAX_AGE)[0] is None


def test_detection_newer_than_frame_is_still_drawn():
    # Negative age (reordered arrival) is not staleness — draw it.
    dets, age = fresh_boxes((100, 0), _dets(100, 30_000_000), MAX_AGE)
    assert dets is not None and age < 0
