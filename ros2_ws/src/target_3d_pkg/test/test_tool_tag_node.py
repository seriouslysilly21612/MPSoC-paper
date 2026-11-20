# pick_target_3d_node with tool_tag:=true on a rendered color image and a flat depth image: the tag
# of the
# capture's own image, in the frame of the object, both ways of its distance, the CSV row, the
# yield counts.
# Runs without ROS: rclpy and the message packages are stand-ins here (they are not on the PC; on
# the board
# the real ones are imported instead).
import importlib
import os
import sys
import types

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)


class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Stamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(int(ns), 1_000_000_000)


def stub_ros():
    """stand-ins for rclpy and the messages the node imports, unless the real ones are there"""
    try:
        import rclpy  # noqa: F401

        return False
    except ImportError:
        pass

    class Node:
        def __init__(self, name):
            self._params = dict(PARAMS)
            self.published = []

        def declare_parameter(self, name, default):
            self._params.setdefault(name, default)

        def get_parameter(self, name):
            return Msg(value=self._params[name])

        def create_subscription(self, *a, **k):
            return object()

        def create_publisher(self, *a, **k):
            return Msg(publish=self.published.append)

        def create_timer(self, *a, **k):
            return object()

        def destroy_subscription(self, s):
            pass

        def destroy_timer(self, t):
            pass

        def get_logger(self):
            return Msg(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

        def get_clock(self):
            return Msg(
                now=lambda: Msg(nanoseconds=2_000_000_000, to_msg=lambda: Stamp(2_000_000_000))
            )

    mods = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.qos': types.ModuleType('rclpy.qos'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'realsense2_camera_msgs': types.ModuleType('realsense2_camera_msgs'),
        'realsense2_camera_msgs.msg': types.ModuleType('realsense2_camera_msgs.msg'),
        'my_interfaces': types.ModuleType('my_interfaces'),
        'my_interfaces.msg': types.ModuleType('my_interfaces.msg'),
    }
    mods['rclpy.node'].Node = Node
    q = mods['rclpy.qos']
    q.QoSProfile = lambda **k: k
    q.DurabilityPolicy = q.HistoryPolicy = q.ReliabilityPolicy = Msg(
        VOLATILE=0, KEEP_LAST=0, BEST_EFFORT=0, RELIABLE=0, TRANSIENT_LOCAL=0
    )
    mods['sensor_msgs.msg'].Image = mods['sensor_msgs.msg'].CameraInfo = Msg
    mods['realsense2_camera_msgs.msg'].Extrinsics = Msg
    mods['my_interfaces.msg'].PickTarget = Msg
    mods['my_interfaces.msg'].PickTarget3D = lambda: Msg(header=Msg(stamp=None, frame_id=''))
    for name in (
        'rclpy.duration',
        'geometry_msgs',
        'geometry_msgs.msg',
        'tf2_ros',
        'tf2_geometry_msgs',
    ):
        mods[name] = types.ModuleType(name)
    mods['rclpy.duration'].Duration = Msg
    mods['geometry_msgs.msg'].PointStamped = Msg
    mods['tf2_ros'].Buffer = Msg
    mods['tf2_ros'].TransformException = Exception
    mods['tf2_ros'].TransformListener = lambda *a: None
    mods['tf2_geometry_msgs'].do_transform_point = None
    sys.modules.update(mods)
    return True


PARAMS = {}
STUBBED = stub_ros()
if not STUBBED:
    pytest.skip('the real rclpy is here: this test is for the PC', allow_module_level=True)
import test_tool_tag as render_lib  # noqa: E402

K = render_lib.K
T_NS = 1_700_000_000_123_456_789


def make_node(**params):
    PARAMS.clear()
    PARAMS.update(params)
    import target_3d_pkg.pick_target_3d_node as mod

    mod = importlib.reload(mod)
    node = mod.PickTarget3DNode()
    info = Msg(k=list(K.flatten()), d=[0.0] * 5)
    node.depth_info_callback(
        Msg(k=list(K.flatten()), d=[0.0] * 5, header=Msg(frame_id='depth_optical'))
    )
    node.color_info_callback(info)
    node.extrinsics_callback(Msg(rotation=list(np.eye(3).flatten()), translation=[0.0, 0.0, 0.0]))
    return node


def color_msg(img_gray, ns):
    rgb = np.repeat(img_gray[:, :, None], 3, axis=2)
    h, w = img_gray.shape
    return Msg(
        header=Msg(stamp=Stamp(ns)),
        data=rgb.tobytes(),
        height=h,
        width=w,
        step=3 * w,
        encoding='rgb8',
    )


def depth_msg(z_m, ns, h=render_lib.H, w=render_lib.W):
    d = np.full((h, w), int(round(z_m * 1000)), dtype='<u2')
    return Msg(
        header=Msg(stamp=Stamp(ns), frame_id='depth_optical'),
        data=d.tobytes(),
        height=h,
        width=w,
        step=2 * w,
        encoding='16UC1',
        is_bigendian=0,
    )


def target(ns, u=200.0, v=300.0, mode=1):
    return Msg(
        capture_stamp=Stamp(ns),
        target_valid=True,
        class_id=1,
        class_name='banana',
        confidence=0.9,
        center_x=u,
        center_y=v,
        width=30.0,
        height=30.0,
        mode=mode,
    )


def run(node, tag_t, plane_z, color_ns=T_NS, target_ns=T_NS):
    node.depth_callback(depth_msg(plane_z, T_NS))
    node.color_callback(
        color_msg(
            render_lib.render([(0, 0.065, render_lib.rot(0.05, 0.05, 0.0), tag_t)]), color_ns
        )
    )
    node.pick_target_callback(target(target_ns))
    return node.published[-1]


def test_the_tag_from_the_depth_image():
    node = make_node(tool_tag=True)
    t = np.array([0.05, -0.02, 0.80])
    out = run(node, t, plane_z=0.80)
    assert out.depth_valid and out.tag_valid and out.mode == 1
    # the depth pixel nearest the tag centre: within a pixel (about 1.3 mm) across, the plane in
    # depth
    assert np.abs(np.array([out.tag_x, out.tag_y]) - t[:2]).max() < 2.5e-3
    assert abs(out.tag_z - 0.80) < 1e-6
    assert node.tag_found == 1


def test_the_tag_from_its_corners():
    node = make_node(tool_tag=True, tag_position='pnp')
    t = np.array([-0.10, 0.04, 0.70])
    out = run(node, t, plane_z=0.50)  # a wrong plane: pnp does not read it
    assert out.tag_valid
    assert (
        np.abs(np.array([out.tag_x, out.tag_y]) - t[:2]).max() < 2e-3
        and abs(out.tag_z - t[2]) < 12e-3
    )


def test_not_another_capture():
    node = make_node(tool_tag=True)
    out = run(
        node, np.array([0.0, 0.0, 0.8]), 0.8, color_ns=T_NS + 33_000_000
    )  # the next image only
    assert out.depth_valid and not out.tag_valid
    assert node.tag_tally == {'no_color_image': 1}


def test_no_tag_in_view():
    node = make_node(tool_tag=True)
    node.depth_callback(depth_msg(0.8, T_NS))
    node.color_callback(color_msg(np.full((render_lib.H, render_lib.W), 255, np.uint8), T_NS))
    node.pick_target_callback(target(T_NS))
    assert not node.published[-1].tag_valid and node.tag_tally == {'not_found': 1}


def test_another_id_in_view_is_told_once():
    node = make_node(tool_tag=True)
    img = render_lib.render([(7, 0.065, render_lib.rot(0, 0, 0), np.array([0.0, 0.0, 0.8]))])
    # 0, 0.1 and 0.3 s: the whole image is searched again only 0.2 s after one without the tag
    for ns in (T_NS, T_NS + 100_000_000, T_NS + 300_000_000):
        node.depth_callback(depth_msg(0.8, ns))
        node.color_callback(color_msg(img, ns))
        node.pick_target_callback(target(ns))
    assert node.tag_tally == {'other_id': 2, 'not_searched': 1} and node.tag_ids_told == {(7,)}


def test_the_tag_goes_from_the_color_to_the_depth_frame():
    # the color camera 15 mm beside the depth camera (p_color = p_depth + t): the tag found in the
    # color image is published in the frame of the object, the depth one
    t_dc = np.array([0.015, 0.0, 0.0])
    p_color = np.array([0.05, -0.02, 0.80])
    for position in ('depth', 'pnp'):
        node = make_node(tool_tag=True, tag_position=position)
        node.extrinsics_callback(
            Msg(rotation=list(np.eye(3).flatten(order='F')), translation=list(t_dc))
        )
        out = run(node, p_color, plane_z=0.80)
        got = np.array([out.tag_x, out.tag_y, out.tag_z])
        # the other direction would put it 30 mm away
        assert out.tag_valid and np.abs(got[:2] - (p_color - t_dc)[:2]).max() < 2.5e-3, position


def test_off_by_default():
    node = make_node()
    out = run(node, np.array([0.0, 0.0, 0.8]), 0.8)
    assert (
        out.depth_valid and not out.tag_valid and node.color_sub is None
    )  # no subscription, no search


def test_bad_tag_position():
    with pytest.raises(RuntimeError):
        make_node(tool_tag=True, tag_position='both')


def test_the_csv_rows_have_the_tag_columns(tmp_path):
    node = make_node(tool_tag=True, metrics_csv_path=str(tmp_path / 'p.csv'))
    node.pick_target_callback(target(T_NS))  # no depth yet: a row without a tag search
    run(node, np.array([0.02, 0.01, 0.75]), 0.75)
    first, second = node.metrics.rows
    assert (
        set(first) >= {'tag_valid', 'tag_depth_z', 'tag_pnp_z', 'mode'}
        and first['tag_valid'] is None
    )
    assert second['tag_valid'] == 1 and abs(second['tag_depth_z'] - 0.75) < 1e-6
    assert abs(second['tag_pnp_z'] - 0.75) < 12e-3 and second['tag_reason'] == ''


def test_the_base_node_moves_the_tag_with_the_object():
    PARAMS.clear()
    import target_3d_pkg.pick_target_base_node as mod

    mod = importlib.reload(mod)
    node = mod.PickTargetBaseNode()
    r = render_lib.rot(0.3, -0.2, 1.0)
    t = np.array([0.8, 0.1, 0.9])
    node._tf_cache['depth_optical'] = (r, t)
    m = Msg(
        capture_stamp=Stamp(T_NS),
        target_valid=True,
        depth_valid=True,
        class_id=1,
        class_name='banana',
        confidence=0.9,
        center_x=1.0,
        center_y=2.0,
        width=3.0,
        height=4.0,
        header=Msg(frame_id='depth_optical'),
        x=0.1,
        y=0.0,
        z=0.7,
        mode=1,
        tag_valid=True,
        tag_x=-0.05,
        tag_y=0.02,
        tag_z=0.65,
    )
    node.target_callback(m)
    out = node.published[-1]
    assert (
        out.target_valid and out.tag_valid and out.mode == 1 and out.header.frame_id == 'base_link'
    )
    np.testing.assert_allclose(
        [out.tag_x, out.tag_y, out.tag_z], r @ np.array([-0.05, 0.02, 0.65]) + t, atol=1e-9
    )
    np.testing.assert_allclose([out.x, out.y, out.z], r @ np.array([0.1, 0.0, 0.7]) + t, atol=1e-9)
    m.tag_valid = False
    node.target_callback(m)
    assert not node.published[-1].tag_valid and node.published[-1].target_valid
    m.z = 0.05  # too close: the target is refused, the tag with it
    m.tag_valid = True
    node.target_callback(m)
    assert not node.published[-1].target_valid and not node.published[-1].tag_valid
