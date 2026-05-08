#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-check /slosh/height and /camera/color/image_raw timestamps from a bag.

For RA-L visual GT (D5) the alignment between modal-predicted height and the
visual height series must be within 50 ms; otherwise gamma_model is unreliable.
This script reads a bag, finds the nearest /image_raw stamp for each
/slosh/height stamp, and prints how many fall outside the tolerance.
"""

import argparse
import csv
import os
import sys

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="bag file containing /slosh/height and /camera/color/image_raw")
    parser.add_argument("--height-topic", default="/slosh/height")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--tol-ms", type=float, default=50.0)
    parser.add_argument("--csv", default="")
    return parser.parse_args()


def collect_stamps(bag_path, topic):
    stamps = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            if stamp is not None:
                stamps.append(stamp.to_sec())
            else:
                stamps.append(t.to_sec())
    return sorted(stamps)


def nearest(sorted_list, value):
    if not sorted_list:
        return None
    lo = 0
    hi = len(sorted_list) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_list[mid] < value:
            lo = mid + 1
        else:
            hi = mid
    candidates = [sorted_list[lo]]
    if lo > 0:
        candidates.append(sorted_list[lo - 1])
    return min(candidates, key=lambda x: abs(x - value))


def main():
    args = parse_args()
    if not os.path.isfile(args.bag):
        sys.stderr.write(f"bag not found: {args.bag}\n")
        sys.exit(2)
    height_stamps = collect_stamps(args.bag, args.height_topic)
    image_stamps = collect_stamps(args.bag, args.image_topic)
    if not height_stamps or not image_stamps:
        sys.stderr.write(
            f"missing topics; height={len(height_stamps)} image={len(image_stamps)}\n"
        )
        sys.exit(1)
    deltas = []
    for ts in height_stamps:
        match = nearest(image_stamps, ts)
        if match is not None:
            deltas.append((ts, abs(ts - match) * 1000.0))
    n = len(deltas)
    n_bad = sum(1 for _, d in deltas if d > args.tol_ms)
    avg = sum(d for _, d in deltas) / n if n else 0.0
    p95 = sorted(d for _, d in deltas)[int(0.95 * (n - 1))] if n else 0.0
    print(
        f"bag={os.path.basename(args.bag)} pairs={n} avg_delta={avg:.1f}ms "
        f"p95={p95:.1f}ms over_tol({args.tol_ms:.0f}ms)={n_bad}/{n}"
    )
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["height_stamp_s", "delta_ms", "over_tol"])
            for ts, d in deltas:
                writer.writerow([f"{ts:.6f}", f"{d:.3f}", int(d > args.tol_ms)])
        print(f"csv: {args.csv}")
    sys.exit(0 if n_bad == 0 else 1)


if __name__ == "__main__":
    main()
