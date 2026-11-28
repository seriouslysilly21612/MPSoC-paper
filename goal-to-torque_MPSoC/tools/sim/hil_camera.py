#!/usr/bin/env python3
"""Step 2 of the desktop simulation (2026-11-28): the camera of the board, rendered on a desktop
(hardware in the loop).

The desktop renders the bench of render_check.py (Indy7, the 36h11 tag on the tool, a YCB apple on
the table) from the colour camera of the board's RealSense and publishes what the RealSense node of
pick_place_vitis_ai.launch.py would, so the board runs its pipeline unchanged with camera:=false:

  /camera/camera/color/image_raw          rgb8 848x480, 30 Hz
  /camera/camera/depth/image_rect_raw     16UC1 [mm], rendered from the same camera (aligned)
  /camera/camera/color|depth/camera_info  the pinhole of the render (f 600.33 px: the mean of the
                                          board's fx 601.06 / fy 599.60; MuJoCo has square pixels)
  /camera/camera/extrinsics/depth_to_color  identity (the depth is rendered in the colour camera)
  /tf_static  camera_link -> camera_color_frame (the board's RealSense, 2026-11-28) -> optical,
              camera_link -> camera_depth_frame -> optical (the same pose as colour)

Every image is stamped with the BOARD clock (the clock of release_s and of the status): the offset
of the two clocks comes from pings answered by clock_echo.py on the board (the median of the 20 %
with the shortest round trip in the last 60 s). The arm is drawn in the pose the board publishes on
/joint_states (the simulated Indy7 of the bench, 20 Hz), or parked (--arm park).

The ground truth of every published image goes to OUT/truth.csv: its stamp (board clock), the apple
centre in base_link, its pixel and depth, the tool, the clock offset and the age of the arm pose.

  board:    python3 clock_echo.py
  desktop:  source /opt/ros/humble/setup.bash && python3 hil_camera.py --out DIR
                [--apple-obj textured.obj] [--speed 0.10 --amp 0.15] [--arm board|park]
                [--duration S] [--save-every 30]

Python 3.10 of Ubuntu 22.04 with ROS 2 Humble; numpy < 2 (the ROS messages are built against
numpy 1): python3 -m venv --system-site-packages ~/hilenv && ~/hilenv/bin/pip install "numpy<2"
mujoco "opencv-contrib-python<4.11". Rendering uses EGL (MUJOCO_GL=egl) unless MUJOCO_GL is set.
"""
import argparse
import array
import csv
import math
import os
import sys
import threading
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

import cv2  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from realsense2_camera_msgs.msg import Extrinsics  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import render_check as rc  # noqa: E402

W, H = rc.W, rc.H
F = 0.5 * (601.064208984375 + 599.600341796875)
CX, CY = W / 2 - 0.5, H / 2 - 0.5         # MuJoCo: the principal point at the image centre
# camera_link -> camera_color_frame of the board's RealSense (/tf_static, 2026-11-28)
LINK_TO_COLOR_T = np.array([-5.0905517127830535e-05, 0.015085823833942413, 0.00038072120514698327])
LINK_TO_COLOR_Q = np.array([0.007408179808408021, -0.0008408292196691036, 0.0020767285022884607,
                            0.9999700784683228])
OPTICAL_Q = np.array([-0.5, 0.5, -0.5, 0.5])   # frame -> optical frame (z forward, x right, y down)
APPLE_C = np.array([0.753, -0.032, rc.TABLE_Z + rc.APPLE_R])   # the bench apple of I-112
PREFIX = '/camera/camera'


def colour_optical_pose():
    """base_link -> camera_color_optical_frame: rotation and position"""
    r_link = rc.quat_to_mat(rc.CAM_Q_XYZW)
    r_col = r_link @ rc.quat_to_mat(LINK_TO_COLOR_Q)
    p_col = rc.CAM_T + r_link @ LINK_TO_COLOR_T
    return r_col @ rc.quat_to_mat(OPTICAL_Q), p_col


class Clock:
    """the offset board - desktop from the pings with the shortest round trip"""

    def __init__(self):
        self.samples = []           # (t_desktop, rtt, offset)
        self.lock = threading.Lock()

    def add(self, t1, t2, t3):
        with self.lock:
            self.samples.append((t3, t3 - t1, t2 - 0.5 * (t1 + t3)))
            self.samples = [s for s in self.samples if t3 - s[0] < 60.0]

    def estimate(self):
        """(offset, rtt of the samples used, number of samples) or None before 10 samples"""
        with self.lock:
            if len(self.samples) < 10:
                return None
            best = sorted(self.samples, key=lambda s: s[1])[:max(5, len(self.samples) // 5)]
        return float(np.median([s[2] for s in best])), float(np.median([s[1] for s in best])), len(best)


class HilNode(Node):
    def __init__(self, clock):
        super().__init__('hil_camera')
        self.clock_est = clock
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.color_pub = self.create_publisher(Image, PREFIX + '/color/image_raw', image_qos)
        self.depth_pub = self.create_publisher(Image, PREFIX + '/depth/image_rect_raw', image_qos)
        self.color_info_pub = self.create_publisher(CameraInfo, PREFIX + '/color/camera_info', reliable)
        self.depth_info_pub = self.create_publisher(CameraInfo, PREFIX + '/depth/camera_info', reliable)
        self.extr_pub = self.create_publisher(Extrinsics, PREFIX + '/extrinsics/depth_to_color', latched)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.ping_pub = self.create_publisher(Float64MultiArray, '/hil/clock_ping', reliable)
        self.create_subscription(Float64MultiArray, '/hil/clock_pong', self.pong, reliable)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.seq = 0
        self.q = None                 # (joint angles, desktop time received)
        self.lock = threading.Lock()
        self.create_timer(0.1, self.ping)
        self.publish_static()

    def ping(self):
        self.seq += 1
        msg = Float64MultiArray()
        msg.data = [float(self.seq), time.time()]
        self.ping_pub.publish(msg)

    def pong(self, msg):
        t3 = time.time()
        _, t1, t2 = msg.data
        self.clock_est.add(t1, t2, t3)

    def joints(self, msg):
        q = dict(zip(msg.name, msg.position))
        if all(f'joint{i}' in q for i in range(6)):
            with self.lock:
                self.q = (np.array([q[f'joint{i}'] for i in range(6)]), time.time())

    def arm(self):
        with self.lock:
            return self.q

    def publish_static(self):
        now = self.get_clock().now().to_msg()      # static: the stamp does not matter
        tfs = []
        for parent, child, t, q in (
                ('camera_link', 'camera_color_frame', LINK_TO_COLOR_T, LINK_TO_COLOR_Q),
                ('camera_color_frame', 'camera_color_optical_frame', np.zeros(3), OPTICAL_Q),
                ('camera_link', 'camera_depth_frame', LINK_TO_COLOR_T, LINK_TO_COLOR_Q),
                ('camera_depth_frame', 'camera_depth_optical_frame', np.zeros(3), OPTICAL_Q)):
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now, parent, child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t)
            (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
             tf.transform.rotation.w) = map(float, q)
            tfs.append(tf)
        self.tf_pub.publish(TFMessage(transforms=tfs))
        e = Extrinsics()
        e.rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        e.translation = [0.0, 0.0, 0.0]
        self.extr_pub.publish(e)

    def info(self, stamp, frame):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, frame
        m.height, m.width, m.distortion_model = H, W, 'plumb_bob'
        m.d = [0.0] * 5
        m.k = [F, 0.0, CX, 0.0, F, CY, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [F, 0.0, CX, 0.0, 0.0, F, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def publish(self, stamp_s, rgb, depth_mm):
        stamp = Time(nanoseconds=int(round(stamp_s * 1e9))).to_msg()
        for img, enc, step, frame, pub in ((rgb, 'rgb8', 3 * W, 'camera_color_optical_frame', self.color_pub),
                                           (depth_mm, '16UC1', 2 * W, 'camera_depth_optical_frame', self.depth_pub)):
            m = Image()
            m.header.stamp, m.header.frame_id = stamp, frame
            m.height, m.width, m.encoding, m.is_bigendian, m.step = H, W, enc, 0, step
            # an array.array is taken as it is; bytes or a list would be checked element by element
            # by the message (about 90 ms for the two images)
            m.data = array.array('B', img.tobytes())
            pub.publish(m)
        self.color_info_pub.publish(self.info(stamp, 'camera_color_optical_frame'))
        self.depth_info_pub.publish(self.info(stamp, 'camera_depth_optical_frame'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--apple-obj', default='')
    ap.add_argument('--speed', type=float, default=0.0, help='peak speed of the apple along y [m/s]; 0 = still')
    ap.add_argument('--amp', type=float, default=0.15, help='amplitude of the motion along y [m]')
    ap.add_argument('--arm', choices=('board', 'park'), default='board')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--duration', type=float, default=0.0, help='s; 0 = until Ctrl-C')
    ap.add_argument('--save-every', type=int, default=0, help='write every Nth image to OUT/frames')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.save_every:
        os.makedirs(os.path.join(a.out, 'frames'), exist_ok=True)

    r_opt, p_opt = colour_optical_pose()
    fovy = math.degrees(2 * math.atan(H / 2 / F))
    m = rc.build(a.out, a.apple_obj, camera=(p_opt, r_opt[:, 0], -r_opt[:, 1], fovy))
    d = mujoco.MjData(m)
    colour = mujoco.Renderer(m, H, W)
    depth_r = mujoco.Renderer(m, H, W)
    depth_r.enable_depth_rendering()
    tcp, apple_mocap = m.body('tcp').id, m.body('apple').mocapid[0]

    rclpy.init()
    clock = Clock()
    node = HilNode(clock)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print('hil_camera: waiting for clock_echo.py on the board ...', flush=True)
    while clock.estimate() is None:
        time.sleep(0.2)
    off, rtt, n = clock.estimate()
    print(f'hil_camera: board - desktop {off * 1e3:+.2f} ms (round trip {rtt * 1e3:.2f} ms, {n} pings)', flush=True)

    f_motion = a.speed / (2 * math.pi * a.amp) if a.speed > 0 else 0.0
    fields = ['frame', 'stamp_board_s', 't_desktop_s', 'offset_ms', 'rtt_ms', 'apple_x', 'apple_y', 'apple_z',
              'apple_vy', 'apple_u', 'apple_v', 'apple_depth_m', 'tcp_x', 'tcp_y', 'tcp_z', 'arm_age_ms',
              'render_ms', 'publish_ms']
    truth = open(os.path.join(a.out, 'truth.csv'), 'w', newline='')
    wr = csv.DictWriter(truth, fieldnames=fields)
    wr.writeheader()
    t0 = time.time()
    period, nxt, k, last_print, render_sum = 1.0 / a.fps, time.perf_counter(), 0, time.time(), 0.0
    try:
        while a.duration <= 0 or time.time() - t0 < a.duration:
            nxt += period
            t_desk = time.time()
            off, rtt, _ = clock.estimate()
            stamp = t_desk + off
            ts = t_desk - t0
            phase = 2 * math.pi * f_motion * ts
            apple = APPLE_C + np.array([0.0, a.amp * math.sin(phase) if f_motion else 0.0, 0.0])
            vy = a.amp * 2 * math.pi * f_motion * math.cos(phase) if f_motion else 0.0
            d.mocap_pos[apple_mocap] = apple
            arm = node.arm() if a.arm == 'board' else None
            d.qpos[:6] = arm[0] if arm is not None else rc.Q0
            mujoco.mj_forward(m, d)
            t_r = time.perf_counter()
            colour.update_scene(d, camera='d435')
            rgb = colour.render()
            depth_r.update_scene(d, camera='d435')
            z = depth_r.render()
            depth_mm = np.where((z > 0.1) & (z < 10.0), np.round(z * 1000.0), 0).astype(np.uint16)
            t_p = time.perf_counter()
            node.publish(stamp, rgb, depth_mm)
            t_e = time.perf_counter()
            pc = r_opt.T @ (apple - p_opt)
            u, v = CX + F * pc[0] / pc[2], CY + F * pc[1] / pc[2]
            tool = d.xpos[tcp]
            wr.writerow(dict(frame=k, stamp_board_s=f'{stamp:.6f}', t_desktop_s=f'{t_desk:.6f}',
                             offset_ms=f'{off * 1e3:.3f}', rtt_ms=f'{rtt * 1e3:.3f}',
                             apple_x=f'{apple[0]:.5f}', apple_y=f'{apple[1]:.5f}', apple_z=f'{apple[2]:.5f}',
                             apple_vy=f'{vy:.5f}', apple_u=f'{u:.2f}', apple_v=f'{v:.2f}',
                             apple_depth_m=f'{pc[2]:.4f}', tcp_x=f'{tool[0]:.5f}', tcp_y=f'{tool[1]:.5f}',
                             tcp_z=f'{tool[2]:.5f}',
                             arm_age_ms=f'{(t_desk - arm[1]) * 1e3:.1f}' if arm is not None else '',
                             render_ms=f'{(t_p - t_r) * 1e3:.2f}', publish_ms=f'{(t_e - t_p) * 1e3:.2f}'))
            if a.save_every and k % a.save_every == 0:
                img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.circle(img, (int(round(u)), int(round(v))), 4, (255, 0, 0), -1)
                cv2.imwrite(os.path.join(a.out, 'frames', f'frame_{k:05d}.png'), img)
            render_sum += t_p - t_r
            k += 1
            if time.time() - last_print > 5.0:
                print(f'hil_camera: {k} images, {k / (time.time() - t0):.1f} /s, render {render_sum / k * 1e3:.1f} ms, '
                      f'offset {off * 1e3:+.2f} ms (rtt {rtt * 1e3:.2f}), arm '
                      f'{"board" if arm is not None else "parked"}', flush=True)
                last_print = time.time()
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()    # late: no burst to catch up
    except KeyboardInterrupt:
        pass
    truth.close()
    print(f'hil_camera: {k} images in {time.time() - t0:.1f} s -> {os.path.join(a.out, "truth.csv")}', flush=True)
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
