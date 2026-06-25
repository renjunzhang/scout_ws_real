#!/usr/bin/env python3
"""Plot fixed-path reference XY against actual robot XY from a rosbag."""

import argparse
import bisect
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca = math.cos(ayaw)
    sa = math.sin(ayaw)
    return (ax + ca * bx - sa * by,
            ay + sa * bx + ca * by,
            ayaw + byaw)


def transform_tuple(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    return (t.x, t.y, yaw_from_quat(r))


def nearest_xy_at_or_before(series, stamp, times=None):
    if not series:
        return None
    if times is None:
        times = [item[0] for item in series]
    idx = bisect.bisect_right(times, stamp) - 1
    if idx < 0:
        return None
    return series[idx][1]


def nearest_distance(point, polyline):
    if not polyline:
        return float("nan")
    px, py = point
    best = float("inf")
    for x, y in polyline:
        d = math.hypot(px - x, py - y)
        if d < best:
            best = d
    return best


def rms(values):
    vals = [v for v in values if math.isfinite(v)]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def read_bag(bag_path, path_topic, odom_topic, target_frame, base_frames):
    reference = []
    actual_from_odom = []
    direct_base_tf = []
    parent_to_base = {}
    target_to_parent = {}

    topics = [path_topic, odom_topic, "/tf"]
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            stamp = t.to_sec()
            if topic == path_topic:
                pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                if pts:
                    reference = pts
                    if msg.header.frame_id:
                        target_frame = msg.header.frame_id
            elif topic == odom_topic:
                p = msg.pose.pose.position
                actual_from_odom.append((stamp, (p.x, p.y)))
            elif topic == "/tf":
                for tr in msg.transforms:
                    parent = tr.header.frame_id.lstrip("/")
                    child = tr.child_frame_id.lstrip("/")
                    tr_stamp = tr.header.stamp.to_sec() or stamp
                    value = transform_tuple(tr)
                    if parent == target_frame and child in base_frames:
                        direct_base_tf.append((tr_stamp, (value[0], value[1])))
                    elif child in base_frames:
                        parent_to_base.setdefault(parent, []).append((tr_stamp, value))
                    elif parent == target_frame:
                        target_to_parent.setdefault(child, []).append((tr_stamp, value))

    actual_from_tf = list(direct_base_tf)
    for parent, base_series in parent_to_base.items():
        anchor_series = target_to_parent.get(parent)
        if not anchor_series:
            continue
        anchor_series.sort(key=lambda item: item[0])
        anchor_times = [item[0] for item in anchor_series]
        for stamp, parent_base in base_series:
            target_parent = nearest_xy_at_or_before(anchor_series, stamp, anchor_times)
            if target_parent is None:
                continue
            x, y, _ = compose(target_parent, parent_base)
            actual_from_tf.append((stamp, (x, y)))

    actual_from_tf.sort(key=lambda item: item[0])
    actual_source = "tf"
    actual = [xy for _, xy in actual_from_tf]
    if not actual:
        actual_source = "odom"
        actual = [xy for _, xy in actual_from_odom]
    return reference, actual, actual_source, target_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--path-topic", default="/scout/global_path_fixed")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--target-frame", default="map")
    parser.add_argument("--base-frame", action="append", default=["base_link", "base_footprint"])
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    reference, actual, actual_source, frame = read_bag(
        args.bag, args.path_topic, args.odom_topic, args.target_frame, set(args.base_frame)
    )
    if not reference:
        raise SystemExit(f"no reference path found on {args.path_topic}")
    if not actual:
        raise SystemExit("no actual trajectory found from /tf or odom")

    errors = [nearest_distance(p, reference) for p in actual]
    final_error = math.hypot(actual[-1][0] - reference[-1][0], actual[-1][1] - reference[-1][1])
    tracking_rms = rms(errors)
    tracking_max = max((e for e in errors if math.isfinite(e)), default=float("nan"))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["bag", "path_points", "actual_points", "actual_source", "frame", "tracking_rms_m", "tracking_max_m", "final_error_m"])
            writer.writerow([str(args.bag), len(reference), len(actual), actual_source, frame, tracking_rms, tracking_max, final_error])

    ref_x, ref_y = zip(*reference)
    act_x, act_y = zip(*actual)
    plt.figure(figsize=(8, 7))
    plt.plot(ref_x, ref_y, "--", linewidth=2.0, label="reference path")
    plt.plot(act_x, act_y, "-", linewidth=2.0, label=f"actual trajectory ({actual_source})")
    plt.scatter([ref_x[0]], [ref_y[0]], marker="o", s=50, label="reference start")
    plt.scatter([ref_x[-1]], [ref_y[-1]], marker="*", s=90, label="reference goal")
    plt.scatter([act_x[-1]], [act_y[-1]], marker="x", s=70, label="actual final")
    title = args.title or args.bag.stem
    plt.title(f"{title}\nRMS={tracking_rms:.3f} m, max={tracking_max:.3f} m, final={final_error:.3f} m")
    plt.xlabel(f"x in {frame} [m]")
    plt.ylabel(f"y in {frame} [m]")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.output, dpi=160)
    print(f"wrote {args.output}")
    if args.csv:
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
