#!/usr/bin/env python3
"""Self-check for raon_operator_gui — no camera, no board, no display.

Drives RosLink's callbacks with synthetic messages and asserts the join, the
drawing and the GUI slots behave. Runs headless via QT_QPA_PLATFORM=offscreen,
so it is safe to run on the board while the robot is live (it opens no ROS
subscriptions of consequence and never touches the app).

    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    QT_QPA_PLATFORM=offscreen python3 tools/gui/operator_console_selfcheck.py

Covers the three join paths that field experience says matter:
  hit   — image arrived first (the common case)
  late  — detection arrived first (the two-sided join's reason to exist)
  stale — an out-of-order pair, which must be skipped, not drawn backwards
"""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import cv2                                     # noqa: E402
import numpy as np                             # noqa: E402
import rclpy                                   # noqa: E402
from PyQt5.QtCore import Qt                    # noqa: E402
from PyQt5.QtWidgets import QApplication       # noqa: E402
from sensor_msgs.msg import CompressedImage    # noqa: E402

from my_interfaces.msg import Detection, DetectionArray, PickTarget3D  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src", "raon_operator_gui"))
from raon_operator_gui.main_window import MainWindow   # noqa: E402
from raon_operator_gui.ros_link import RosLink         # noqa: E402

W, H = 848, 480


def mk_img(sec):
    frame = np.full((H, W, 3), 60, np.uint8)
    m = CompressedImage()
    m.header.stamp.sec, m.header.stamp.nanosec = sec, 5
    m.format = "jpeg"
    m.data = cv2.imencode(".jpg", frame)[1].tobytes()
    return m


def mk_det(sec, name="apple"):
    d = Detection()
    d.class_name, d.confidence = name, 0.87
    d.center_x, d.center_y, d.width, d.height = 424.0, 240.0, 100.0, 80.0
    a = DetectionArray()
    a.header.stamp.sec, a.header.stamp.nanosec = sec, 5
    a.detections = [d]
    return a


def main():
    app = QApplication([])
    rclpy.init()
    node = RosLink("/selfcheck/image", "/selfcheck/detections", "/selfcheck/target")
    win = MainWindow(node.signals, {"image": "-", "detections": "-", "pick_target": "-"})

    drawn = []
    node.signals.frame_ready.connect(drawn.append, Qt.DirectConnection)

    node.on_image(mk_img(10));  node.on_detections(mk_det(10))                    # hit
    node.on_detections(mk_det(11, "mustard_bottle")); node.on_image(mk_img(11))   # late
    node.on_image(mk_img(9));   node.on_detections(mk_det(9))                     # stale

    # hit == 2 is CORRECT: the out-of-order detection also found its buffered
    # image (hit), and was only then skipped by the monotonic guard (stale).
    assert (node.n_hit, node.n_late, node.n_stale, node.n_drop) == (2, 1, 1, 0), \
        f"join counters wrong: hit={node.n_hit} late={node.n_late} " \
        f"stale={node.n_stale} drop={node.n_drop}"
    assert len(drawn) == 2, f"expected 2 rendered frames, got {len(drawn)}"
    assert drawn[0].shape == (H, W, 3), f"frame reshaped: {drawn[0].shape}"
    assert drawn[0].std() > 1.0, "frame is flat — nothing was drawn on it"
    print(f"join     OK  hit={node.n_hit} late={node.n_late} "
          f"stale={node.n_stale} drop={node.n_drop} rendered={len(drawn)}")

    # BGR must survive the numpy -> QImage -> QPixmap trip unswapped.
    red = np.zeros((H, W, 3), np.uint8)
    red[:, :, 2] = 255                       # pure red in BGR
    win.video.set_frame(red)
    c = win.video._pixmap.toImage().pixelColor(10, 10)
    assert (c.red(), c.green(), c.blue()) == (255, 0, 0), \
        f"channel swap! BGR red rendered as {(c.red(), c.green(), c.blue())}"
    print("color    OK  BGR(0,0,255) -> RGB(255,0,0), no swap (Format_BGR888)")

    win.on_frame(drawn[0])
    win.on_detections(mk_det(10))
    assert win.tbl.rowCount() == 1, "detection table not populated"

    t = PickTarget3D()
    t.target_valid = t.depth_valid = True
    t.class_name, t.x, t.y, t.z = "apple", 0.841, -0.102, 0.179
    win.on_pick_target(t)
    assert win.lbl_tgt_valid.text() == "LOCKED", "pick target not shown as locked"
    win._check_link()
    assert win.lbl_link.text() == "LIVE", "watchdog says dead right after a frame"
    print(f"widgets  OK  rows={win.tbl.rowCount()} target={win.lbl_tgt_xyz.text()} "
          f"link={win.lbl_link.text()}")

    # And it must go dead on its own when frames stop.
    win._last_frame_wall -= 10.0
    win._check_link()
    assert win.lbl_link.text() == "NO SIGNAL", "watchdog never trips"
    print("watchdog OK  goes NO SIGNAL after the timeout")

    node.destroy_node()
    rclpy.shutdown()
    del win, app
    print("\nSELF-CHECK PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
