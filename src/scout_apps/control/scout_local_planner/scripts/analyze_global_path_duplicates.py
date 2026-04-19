#!/usr/bin/env python3
"""Locate duplicate / near-duplicate segments inside /scout/global_path from a rosbag."""

import argparse
import math

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--topic", default="/scout/global_path", help="Path topic to inspect")
    parser.add_argument("--short-threshold", type=float, default=1e-3,
                        help="Segment length below this is treated as suspicious [m]")
    parser.add_argument("--topk", type=int, default=20,
                        help="Maximum suspicious segments to print per message")
    return parser.parse_args()


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    args = parse_args()
    found = 0

    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=[args.topic]):
            t = stamp.to_sec() - start
            pts = [
                (float(p.pose.position.x), float(p.pose.position.y))
                for p in msg.poses
                if math.isfinite(float(p.pose.position.x)) and math.isfinite(float(p.pose.position.y))
            ]
            if len(pts) < 2:
                continue

            suspicious = []
            segs = []
            for i in range(len(pts) - 1):
                seg = dist(pts[i], pts[i + 1])
                segs.append(seg)
                if seg < args.short_threshold:
                    suspicious.append((i, seg, pts[i], pts[i + 1]))

            if not suspicious:
                continue

            found += 1
            total_len = sum(segs)
            print(f"t={t:.2f}s poses={len(pts)} total_len={total_len:.4f}m suspicious={len(suspicious)}")
            for i, seg, p0, p1 in suspicious[:args.topk]:
                print(
                    f"  idx={i}->{i+1} seg={seg:.6e} "
                    f"p0=({p0[0]:.6f},{p0[1]:.6f}) p1=({p1[0]:.6f},{p1[1]:.6f})"
                )
            if len(suspicious) > args.topk:
                print(f"  ... {len(suspicious) - args.topk} more suspicious segments")

    if found == 0:
        print("No suspicious short segments found.")


if __name__ == "__main__":
    main()
