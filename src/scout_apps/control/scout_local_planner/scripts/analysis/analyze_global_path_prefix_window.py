#!/usr/bin/env python3
"""Inspect the prefix window of /scout/global_path from a rosbag.

This script focuses on the first K points, matching the current failure mode:
closest_idx=0, start=0, end=K-1.
"""

import argparse
import math

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--topic", default="/scout/global_path", help="Path topic to inspect")
    parser.add_argument("--window-size", type=int, default=89, help="Prefix point count to inspect")
    parser.add_argument("--message-index", type=int, default=-1,
                        help="Which matching path message to inspect, -1 means last")
    parser.add_argument("--topk", type=int, default=15, help="How many worst segments/turns to print")
    return parser.parse_args()


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def sanitize_tail_duplicates(points, eps=1e-6):
    pts = list(points)
    while len(pts) >= 2 and dist(pts[-2], pts[-1]) < eps:
        pts.pop()
    return pts


def main():
    args = parse_args()
    messages = []
    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=[args.topic]):
            pts = [
                (float(p.pose.position.x), float(p.pose.position.y))
                for p in msg.poses
                if math.isfinite(float(p.pose.position.x)) and math.isfinite(float(p.pose.position.y))
            ]
            pts = sanitize_tail_duplicates(pts)
            messages.append((stamp.to_sec() - start, pts))

    if not messages:
        raise SystemExit("No matching path messages found.")

    idx = args.message_index if args.message_index >= 0 else len(messages) - 1
    idx = max(0, min(idx, len(messages) - 1))
    t, pts = messages[idx]

    if len(pts) < 3:
        raise SystemExit(f"Selected path too short after tail sanitize: {len(pts)} points")

    window = pts[: min(args.window_size, len(pts))]
    segs = []
    headings = []
    for i in range(len(window) - 1):
        seg = dist(window[i], window[i + 1])
        segs.append((i, seg, window[i], window[i + 1]))
        headings.append((i, heading(window[i], window[i + 1])))

    turns = []
    for i in range(len(headings) - 1):
        dtheta = wrap(headings[i + 1][1] - headings[i][1])
        ds = max(1e-9, 0.5 * (segs[i][1] + segs[i + 1][1]))
        turns.append((i + 1, abs(dtheta), abs(dtheta) / ds, dtheta))

    print(f"bag: {args.bag}")
    print(f"message_index: {idx}")
    print(f"t: {t:.2f}s")
    print(f"window_points: {len(window)} / total_points: {len(pts)}")
    print(f"window_length: {sum(seg for _, seg, _, _ in segs):.4f}m")

    print("\nShortest Segments")
    for i, seg, p0, p1 in sorted(segs, key=lambda x: x[1])[: args.topk]:
        print(
            f"- idx={i}->{i+1} seg={seg:.6f} "
            f"p0=({p0[0]:.6f},{p0[1]:.6f}) p1=({p1[0]:.6f},{p1[1]:.6f})"
        )

    print("\nLargest Heading Changes")
    for idx_turn, abs_dtheta, turn_rate, signed_dtheta in sorted(turns, key=lambda x: x[2], reverse=True)[: args.topk]:
        print(
            f"- around idx={idx_turn} dtheta={signed_dtheta:.4f}rad "
            f"|dtheta|={abs_dtheta:.4f} turn_rate={turn_rate:.4f}rad/m"
        )

    print("\nPrefix Coordinates")
    preview = min(20, len(window))
    for i in range(preview):
        print(f"- idx={i} p=({window[i][0]:.6f},{window[i][1]:.6f})")
    if len(window) > preview:
        print(f"- ... {len(window) - preview} more prefix points")


if __name__ == "__main__":
    main()
