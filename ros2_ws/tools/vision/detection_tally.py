#!/usr/bin/env python3
"""detection_tally.py -- per-class detection rate and confidence on /detections.

For a static scene (objects placed once and left alone) while the pipeline runs:
how often each class appears in the published DetectionArray messages (the
detector publishes empty arrays too, so every processed frame counts) and its
confidence, the best one per message. Two runs of the same scene with two
models compare their detection; the timing comes from the metrics CSVs.

    source install/setup.bash
    python3 tools/vision/detection_tally.py --seconds 120 [--out tally.csv]
"""
import argparse
import collections
import csv
import statistics
import sys
import time


def tally(messages):
    """messages: one list of (class_name, confidence) per DetectionArray."""
    best = collections.defaultdict(list)
    for dets in messages:
        per_msg = {}
        for name, c in dets:
            per_msg[name] = max(per_msg.get(name, 0.0), c)
        for name, c in per_msg.items():
            best[name].append(c)
    rows = []
    for name in sorted(best, key=lambda n: -len(best[n])):
        cs = best[name]
        rows.append({'class': name, 'frames': len(cs),
                     'rate': len(cs) / len(messages),
                     'conf_median': statistics.median(cs), 'conf_min': min(cs)})
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seconds', type=float, default=120.0)
    p.add_argument('--topic', default='/detections')
    p.add_argument('--out', default='', help='CSV of the table below')
    args = p.parse_args()

    import rclpy
    from rclpy.node import Node
    from my_interfaces.msg import DetectionArray

    rclpy.init()
    node = Node('detection_tally')
    messages, stamps = [], []

    def on_msg(msg):
        stamps.append(time.monotonic())
        messages.append([(d.class_name, d.confidence) for d in msg.detections])

    node.create_subscription(DetectionArray, args.topic, on_msg, 10)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

    if not messages:
        sys.exit(f'no message on {args.topic} in {args.seconds:.0f} s')
    span = stamps[-1] - stamps[0]
    rate = (len(messages) - 1) / span if span > 0 else float('nan')
    print(f'{len(messages)} messages on {args.topic} in {span:.1f} s ({rate:.1f} Hz)')
    rows = tally(messages)
    print(f'{"class":<16}{"frames":>8}{"rate":>8}{"conf med":>10}{"conf min":>10}')
    for r in rows:
        print(f'{r["class"]:<16}{r["frames"]:>8}{r["rate"]:>8.3f}'
              f'{r["conf_median"]:>10.3f}{r["conf_min"]:>10.3f}')
    if not rows:
        print('(no detection in any message)')
    if args.out:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['class', 'frames', 'rate', 'conf_median', 'conf_min'])
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
