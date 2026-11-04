#!/usr/bin/env python3
"""Desktop-side bbox overlay viewer for the KV260 perception pipeline.

Runs on the DESKTOP, not the board. Subscribes to the board's compressed color
stream and /detections, draws the boxes, and shows the result. The board only
compresses; all drawing happens here.

Two render modes — the video and the boxes arrive at DIFFERENT rates (color
30 fps, detections ~15 Hz), and you cannot have both smooth video and
frame-exact boxes, because a frame's own boxes only exist ~107 ms after it was
captured:

  smooth (default)  render every color frame -> 30 fps, near-live video, with
                    the newest boxes available. Boxes therefore lag the picture
                    by ~107 ms (invisible on a static pick scene; on a fast
                    moving object the box trails it).
  --sync            render only frames whose EXACT detection arrived -> 15 fps,
                    ~107 ms behind live, boxes pixel-exact on their own frame.
                    This is the mode that verified bbox<->frame alignment
                    (2026-07-16); keep using it for that check.

Either way the board sends the same 30 fps JPEG stream and pays the same cost —
smooth mode simply stops throwing half the decoded frames away.

Why not draw on the board: it cost ~44 ms/frame on top of a 37.6 ms detect and
busted the 66.6 ms budget for 15 Hz, so the board-side path was removed
(2026-09-03). Drawing here costs the board nothing.

Wire contract (verified against the board 2026-07-16, see
docs/vision/desktop_viewer_plan.md):
  - Detection bbox coords are pixels in the ORIGINAL color frame, center+size.
    The worker already undoes the 416x416 letterbox, so do NOT rescale here.
  - DetectionArray.header is a byte copy of the source Image header, so an
    exact (sec, nanosec) join is valid.
  - /detections is RELIABLE; the compressed image inherits SYSTEM_DEFAULT
    (= RELIABLE), and a BEST_EFFORT subscriber is compatible with it.
"""

import argparse
import csv
import os
import sys
import time
from collections import OrderedDict
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from my_interfaces.msg import DetectionArray

# Keyed by class_name, not class_id: the string ships in every Detection, and
# keying on the id would silently mis-label if decode_meta.json's order ever
# changed.
CLASS_COLORS = {
    "apple": (60, 60, 220),
    "orange": (0, 150, 255),
    "banana": (60, 220, 220),
    "tennis_ball": (60, 220, 60),
    "mustard_bottle": (200, 140, 60),
}
UNKNOWN_COLOR = (200, 200, 200)


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


def stamp_s(key):
    return key[0] + key[1] * 1e-9


def fresh_boxes(frame_key, dets, max_age_s):
    """Which boxes to draw on the frame at frame_key -> (dets_or_None, age_s).

    (None, None) means nothing is fresh enough to draw: either no detection has
    arrived yet, or the newest one is older than max_age_s relative to this
    frame — a stalled detector must go visibly blank, not leave its last boxes
    frozen on a live picture. Pure function so it is testable without ROS.
    """
    if dets is None:
        return None, None
    age = stamp_s(frame_key) - stamp_s(stamp_key(dets.header))
    if age > max_age_s:
        return None, None
    return dets, age


class DetectionViewer(Node):
    def __init__(self, image_topic, detections_topic, buffer_len, window,
                 fps_csv="", fps_dir="~/pp_ws/viewer_ws/fps_log",
                 sync=False, max_box_age_s=0.5):
        super().__init__("detection_viewer_node")
        self.window = window
        self.buffer_len = buffer_len
        # smooth (default): every color frame is rendered as it lands, carrying
        # the newest boxes we have. sync: only exact (image, detection) pairs.
        self.smooth = not sync
        # Boxes older than this relative to the frame being shown are dropped
        # rather than drawn. Without it, a stalled detector (or a crashed board)
        # would leave the last boxes frozen on screen looking live.
        self.max_box_age_s = max_box_age_s
        self.last_dets = None      # newest DetectionArray (smooth mode)
        self.n_drawn = 0           # frames rendered (smooth mode)
        self.n_nobox = 0           # rendered with boxes suppressed as too old
        # The join is two-sided because the two streams race. A detection is
        # published ~37 ms after its frame was captured and then crosses the
        # network; its compressed twin is encoded and crosses the network on an
        # independent path. Either can arrive first.
        #   frames:  stamp -> decoded BGR frame   (covers "image arrived first")
        #   pending: stamp -> DetectionArray      (covers "detection arrived first")
        # Both are bounded FIFOs; whichever side completes a pair renders it.
        self.frames = OrderedDict()
        self.pending = OrderedDict()
        # A detection whose image has not shown up within this many detections
        # is never going to be useful — the frame is already stale on screen.
        self.pending_len = 10
        # Guards against rendering backwards: a pathologically late image could
        # otherwise draw frame N after N+1 is already up, which reads as a
        # stutter. Skipping is honest — we count it rather than show it.
        self.last_rendered = None

        # Displayed FPS = the rate render() actually pushes frames to the window
        # (one imshow per completed pair), measured on the wall clock, not the
        # ROS clock. EMA-smoothed so the HUD number does not jitter frame to
        # frame; alpha 0.2 -> ~5-frame time constant (~0.3 s at 15 Hz).
        self.last_render_t = None
        self.first_render_t = None
        self.fps_ema = None
        # Per-frame FPS log, always on. Rows are buffered in RAM (a plain
        # list.append on the render path — no disk I/O) and flushed to CSV once
        # at shutdown, so the logging never touches the hot path. The file is
        # auto-named per run under fps_dir; --fps-csv overrides with an exact path.
        self.fps_csv_path = self._resolve_fps_path(fps_csv, fps_dir)
        self.fps_samples = []  # rows of (elapsed_s, inst_fps, ema_fps, n_det)

        # BEST_EFFORT on the image: the publisher is RELIABLE, which is
        # compatible, and asking for RELIABLE over the network would invite
        # retransmissions and latency for data we are about to overwrite anyway.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Must match the publisher's RELIABLE — a BEST_EFFORT subscriber would
        # also match, but detections are small and we want every one of them.
        det_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(CompressedImage, image_topic, self.on_image, image_qos)
        self.create_subscription(DetectionArray, detections_topic, self.on_detections, det_qos)

        self.n_img = 0
        self.n_det = 0
        self.n_hit = 0     # image was already buffered when the detection landed
        self.n_late = 0    # detection waited; its image landed after (the two-sided win)
        self.n_drop = 0    # detection aged out of pending — its image never came
        self.n_stale = 0   # pair completed but out of order; skipped to keep time moving forward
        self.create_timer(5.0, self.report)

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"image:      {image_topic}")
        self.get_logger().info(f"detections: {detections_topic}")
        self.get_logger().info(
            "mode: smooth — every color frame is drawn (~30 fps), boxes are the "
            f"newest available (they lag ~100 ms; hidden past {self.max_box_age_s:.1f}s). "
            "Use --sync for 15 fps frame-exact boxes."
            if self.smooth else
            "mode: sync — only exact (image, detection) pairs are drawn (~15 fps).")
        self.get_logger().info("waiting for both topics... (q or ESC in the window to quit)")

    def on_image(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # always BGR, whatever was encoded
        if frame is None:
            self.get_logger().warn("imdecode failed", throttle_duration_sec=5.0)
            return
        key = stamp_key(msg.header)
        self.n_img += 1

        if self.smooth:
            # Draw immediately — never wait for this frame's own detection,
            # which is still ~107 ms away and would cost both the frame rate
            # and the liveness.
            if self.last_rendered is not None and key <= self.last_rendered:
                self.n_stale += 1   # out-of-order/duplicate arrival
                return
            self.last_rendered = key
            dets, age = fresh_boxes(key, self.last_dets, self.max_box_age_s)
            if dets is None and self.last_dets is not None:
                self.n_nobox += 1
            self.n_drawn += 1
            self.render(frame, dets, age)
            return

        self.frames[key] = frame
        while len(self.frames) > self.buffer_len:
            self.frames.popitem(last=False)

        # The other half of the join: a detection may already be waiting on this
        # exact frame.
        waiting = self.pending.pop(key, None)
        if waiting is not None:
            self.n_late += 1
            self.try_render(key, frame, waiting)

    def on_detections(self, msg):
        self.n_det += 1
        if self.smooth:
            self.last_dets = msg
            return
        key = stamp_key(msg.header)
        frame = self.frames.get(key)
        if frame is None:
            # Not a miss yet — the image may still be in flight. Hold the
            # detection and let on_image complete the pair. Only aging out of
            # this queue counts as a real drop.
            self.pending[key] = msg
            while len(self.pending) > self.pending_len:
                self.pending.popitem(last=False)
                self.n_drop += 1
            return
        self.n_hit += 1
        self.try_render(key, frame, msg)

    def try_render(self, key, frame, msg):
        if self.last_rendered is not None and key < self.last_rendered:
            self.n_stale += 1
            return
        self.last_rendered = key
        self.render(frame.copy(), msg)

    def render(self, frame, msg, box_age=None):
        # msg None = we have no boxes fresh enough to honestly draw.
        dets = msg.detections if msg is not None else []
        for det in dets:
            # Coords are already in this frame's pixel space — no scaling.
            xmin = int(round(det.center_x - det.width / 2.0))
            ymin = int(round(det.center_y - det.height / 2.0))
            xmax = int(round(det.center_x + det.width / 2.0))
            ymax = int(round(det.center_y + det.height / 2.0))
            color = CLASS_COLORS.get(det.class_name, UNKNOWN_COLOR)
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ytop = max(0, ymin - th - 4)
            cv2.rectangle(frame, (xmin, ytop), (xmin + tw + 4, ytop + th + 4), color, -1)
            cv2.putText(frame, label, (xmin + 2, ytop + th),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        hud = f"{len(dets)} det  {frame.shape[1]}x{frame.shape[0]}"
        if box_age is not None:
            # How far the boxes lag the picture — the honest cost of smooth mode.
            hud += f"  box +{box_age * 1000:.0f}ms"
        elif msg is None and self.smooth:
            hud += "  NO FRESH BOXES"
        cv2.putText(frame, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)

        # Update the displayed-FPS estimate once per shown frame, then draw it
        # top-right. First frame has no interval yet, so it stays blank.
        now = time.monotonic()
        if self.last_render_t is not None:
            dt = now - self.last_render_t
            if dt > 0.0:
                inst = 1.0 / dt
                self.fps_ema = inst if self.fps_ema is None else 0.2 * inst + 0.8 * self.fps_ema
                if self.fps_csv_path:
                    self.fps_samples.append(
                        (now - self.first_render_t, inst, self.fps_ema, len(dets)))
        else:
            self.first_render_t = now
        self.last_render_t = now
        if self.fps_ema is not None:
            fps_text = f"{self.fps_ema:.1f} FPS"
            (fw, _), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            org = (frame.shape[1] - fw - 8, 24)
            # Black outline under a bright label so it reads on any background.
            cv2.putText(frame, fps_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, fps_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow(self.window, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()

    def report(self):
        if self.n_img == 0 and self.n_det == 0:
            self.get_logger().warn(
                "no images AND no detections — check discovery: "
                "ros2 topic list --no-daemon --spin-time 8")
            return
        if self.n_img == 0:
            self.get_logger().warn("detections but NO images — is the compressed plugin "
                                   "installed and the camera restarted?")
        elif self.n_det == 0:
            self.get_logger().warn("images but NO detections — is my_interfaces built from "
                                   "the SAME commit as the board?")
        if self.smooth:
            self.get_logger().info(
                f"img={self.n_img} det={self.n_det} drawn={self.n_drawn} "
                f"(smooth: every frame) reorder_skip={self.n_stale} "
                f"stale_boxes_hidden={self.n_nobox}")
            self.n_img = self.n_det = self.n_drawn = 0
            self.n_stale = self.n_nobox = 0
            return
        drawn = self.n_hit + self.n_late
        total = drawn + self.n_drop + self.n_stale
        rate = (100.0 * drawn / total) if total else 0.0
        self.get_logger().info(
            f"img={self.n_img} det={self.n_det} drawn={drawn} "
            f"(hit={self.n_hit} late={self.n_late}) "
            f"drop={self.n_drop} stale={self.n_stale} pending={len(self.pending)} "
            f"({rate:.0f}% joined)")
        self.n_img = self.n_det = self.n_hit = self.n_late = 0
        self.n_drop = self.n_stale = 0

    def _resolve_fps_path(self, fps_csv, fps_dir):
        """Pick the CSV path — auto-named per run under fps_dir, unless --fps-csv
        gives an exact file — and ensure its directory exists. Returns "" (which
        disables logging) if the directory cannot be created, so a bad path never
        crashes the viewer."""
        if fps_csv:
            path = os.path.expanduser(fps_csv)
            directory = os.path.dirname(path) or "."
        else:
            directory = os.path.expanduser(fps_dir)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(directory, f"fps_{stamp}.csv")
        try:
            os.makedirs(directory, exist_ok=True)
        except OSError as e:
            self.get_logger().error(
                f"cannot create FPS log dir {directory}: {e} — FPS logging disabled")
            return ""
        self.get_logger().info(f"FPS log -> {path}")
        return path

    def dump_fps_csv(self):
        """Flush the buffered FPS samples to CSV. Called once on shutdown, so
        the whole run's data hits the disk in a single write — nothing during
        rendering. A SIGKILL (kill -9) skips this; q/ESC and Ctrl-C do not.

        Uses print(), not self.get_logger(): by the time this runs on Ctrl-C the
        rclpy context is already invalid (the SIGINT handler tore it down first),
        so a ROS log here still prints to the console but then fails to publish to
        /rosout with a 'publisher's context is invalid' warning. print() avoids
        the /rosout hop entirely, so the summary shows cleanly with no warning."""
        if not self.fps_csv_path or not self.fps_samples:
            return
        try:
            with open(self.fps_csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["elapsed_s", "inst_fps", "ema_fps", "n_det"])
                for elapsed, inst, ema, ndet in self.fps_samples:
                    w.writerow([f"{elapsed:.4f}", f"{inst:.3f}", f"{ema:.3f}", ndet])
        except OSError as e:
            print(f"[detection_viewer] could not write FPS CSV {self.fps_csv_path}: {e}",
                  file=sys.stderr)
            return
        dur = self.fps_samples[-1][0]
        avg = len(self.fps_samples) / dur if dur > 0 else 0.0
        print(f"[detection_viewer] wrote {len(self.fps_samples)} FPS rows to "
              f"{self.fps_csv_path} ({dur:.1f}s, avg {avg:.1f} FPS)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/camera/camera/color/image_raw/compressed")
    ap.add_argument("--detections-topic", default="/detections")
    ap.add_argument("--buffer", type=int, default=30)
    ap.add_argument("--window", default="KV260 detections")
    ap.add_argument("--fps-dir", default="~/pp_ws/viewer_ws/fps_log",
                    help="directory for the auto-named per-run FPS CSV (logging is always on)")
    ap.add_argument("--fps-csv", default="",
                    help="exact CSV path; overrides --fps-dir auto-naming")
    ap.add_argument("--sync", action="store_true",
                    help="render only exact (image, detection) pairs: 15 fps and "
                         "~107 ms behind live, but boxes are exact on their own "
                         "frame. Use for bbox<->frame alignment checks. Default "
                         "is smooth 30 fps with the newest boxes.")
    ap.add_argument("--max-box-age", type=float, default=0.5,
                    help="smooth mode: hide boxes older than this many seconds "
                         "relative to the displayed frame (default 0.5)")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = DetectionViewer(args.image_topic, args.detections_topic, args.buffer,
                           args.window, fps_csv=args.fps_csv, fps_dir=args.fps_dir,
                           sync=args.sync, max_box_age_s=args.max_box_age)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dump_fps_csv()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
