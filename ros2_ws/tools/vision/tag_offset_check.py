#!/usr/bin/env python3
"""tag_offset_check.py -- the object minus the tool tag on /pick_target_base, against a measured
offset.

The desk check of the tool tag (plan A-9, D-22) before the robot: the tag and an object lie on the
table at an offset measured with a ruler, the pipeline runs with tool_tag:=true and
desired_class:=<the object>, and this prints what the camera side would put in the mailbox with
APP_RESULT_FORMAT=tag (the object minus the tag, in base_link) and how far it is from the measured
offset. Its distance in the plane of the table and its length are what a ruler checks; the
direction depends on how the ruler lies in base_link.

    source install/setup.bash
    python3 tools/vision/tag_offset_check.py --seconds 30 --offset-mm 250 [--out samples.csv]

Also: how many targets had their tag (tag_valid), the inference modes of the targets, the spread of
each."""

import argparse
import collections
import csv
import math
import statistics
import sys
import time


def summary(samples, offset_mm=None):
    """samples: (target_valid, tag_valid, mode, object xyz, tag xyz) per message -> the numbers"""
    valid = [s for s in samples if s[0]]
    pairs = [s for s in valid if s[1]]
    out = {
        'messages': len(samples),
        'valid': len(valid),
        'with_tag': len(pairs),
        'modes': dict(collections.Counter(s[2] for s in valid)),
    }
    if not pairs:
        return out
    d = [[1e3 * (o - t) for o, t in zip(s[3], s[4])] for s in pairs]
    for k, axis in enumerate('xyz'):
        col = [v[k] for v in d]
        out[f'd{axis}_mean'] = statistics.fmean(col)
        out[f'd{axis}_std'] = statistics.pstdev(col)
    lengths = [math.sqrt(sum(c * c for c in v)) for v in d]
    planar = [math.hypot(v[0], v[1]) for v in d]
    out['length_mean'], out['length_std'] = statistics.fmean(lengths), statistics.pstdev(lengths)
    out['planar_mean'], out['planar_std'] = statistics.fmean(planar), statistics.pstdev(planar)
    for name, idx in (('object', 3), ('tag', 4)):
        for k, axis in enumerate('xyz'):
            out[f'{name}_{axis}_mean'] = statistics.fmean(1e3 * s[idx][k] for s in pairs)
    if offset_mm is not None:
        out['length_error'] = out['length_mean'] - offset_mm
        out['planar_error'] = out['planar_mean'] - offset_mm
    return out


def report(out, offset_mm=None):
    lines = [
        f"{out['messages']} messages, {out['valid']} valid targets, "
        f"{out['with_tag']} of them with the "
        f"tag in their image; modes of the valid ones {out['modes']} (0 = 416, 1 = 320)"
    ]
    if not out['with_tag']:
        lines.append(
            'no target with the tag: is the tag in view, tool_tag:=true, its id and size right?'
        )
        return '\n'.join(lines)
    lines.append(
        'object - tag in base_link [mm]: '
        + ', '.join(f"{a} {out[f'd{a}_mean']:+.1f} (sd {out[f'd{a}_std']:.1f})" for a in 'xyz')
    )
    lines.append(
        f"length {out['length_mean']:.1f} mm (sd {out['length_std']:.1f}), "
        "in the plane of the table "
        f"{out['planar_mean']:.1f} mm (sd {out['planar_std']:.1f})"
    )
    lines.append(
        'object at '
        + ' '.join(f"{out[f'object_{a}_mean']:.1f}" for a in 'xyz')
        + ' mm, tag at '
        + ' '.join(f"{out[f'tag_{a}_mean']:.1f}" for a in 'xyz')
        + ' mm (base_link)'
    )
    if offset_mm is not None:
        lines.append(
            f'against the measured {offset_mm:.1f} mm: length {out["length_error"]:+.1f} mm, '
            f'in the plane {out["planar_error"]:+.1f} mm'
        )
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--topic', default='/pick_target_base')
    p.add_argument(
        '--offset-mm',
        type=float,
        default=None,
        help='the distance between the object and the tag centre measured with a ruler',
    )
    p.add_argument('--out', default='', help='CSV of every message')
    args = p.parse_args()

    import rclpy
    from rclpy.node import Node
    from my_interfaces.msg import PickTarget3D

    rclpy.init()
    node = Node('tag_offset_check')
    samples = []

    def on_msg(m):
        samples.append(
            (
                bool(m.target_valid and m.depth_valid),
                bool(m.tag_valid),
                int(m.mode),
                (m.x, m.y, m.z),
                (m.tag_x, m.tag_y, m.tag_z),
            )
        )

    node.create_subscription(PickTarget3D, args.topic, on_msg, 10)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if args.out:
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(
                ['target_valid', 'tag_valid', 'mode', 'x', 'y', 'z', 'tag_x', 'tag_y', 'tag_z']
            )
            for s in samples:
                w.writerow([int(s[0]), int(s[1]), s[2], *s[3], *s[4]])
    print(report(summary(samples, args.offset_mm), args.offset_mm))
    return 0 if any(s[0] and s[1] for s in samples) else 1


if __name__ == '__main__':
    sys.exit(main())
