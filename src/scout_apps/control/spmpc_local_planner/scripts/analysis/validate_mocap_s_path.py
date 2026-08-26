#!/usr/bin/env python3
"""Validate and summarize a frozen compact S path JSON.

The accepted JSON schema is the one used by fixed_global_path_runner.py and
template_fixed_path_generator.py: a frame_id plus a list of poses.  This tool
does not publish ROS messages and is safe to run on a development machine.
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def load_points(path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    poses = payload.get("poses")
    if not isinstance(poses, list):
        raise ValueError("path JSON must contain a poses list")
    points = []
    nonfinite = 0
    for index, pose in enumerate(poses):
        if not isinstance(pose, dict):
            raise ValueError("pose {} is not an object".format(index))
        try:
            x = float(pose["x"])
            y = float(pose["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("pose {} has invalid x/y: {}".format(index, exc))
        if not math.isfinite(x) or not math.isfinite(y):
            nonfinite += 1
        points.append((x, y))
    return payload, points, nonfinite


def path_metrics(points, min_segment_length):
    segments = []
    headings = []
    for first, second in zip(points[:-1], points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= min_segment_length:
            continue
        segments.append(length)
        headings.append(math.atan2(dy, dx))

    turns = [wrap_angle(second - first) for first, second in zip(headings[:-1], headings[1:])]
    positive_turn = sum(value for value in turns if value > 0.0)
    negative_turn = -sum(value for value in turns if value < 0.0)
    sign_changes = 0
    significant_signs = []
    for value in turns:
        if abs(value) < 1e-4:
            continue
        sign = 1 if value > 0.0 else -1
        if significant_signs and significant_signs[-1] != sign:
            sign_changes += 1
        significant_signs.append(sign)

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return {
        "pose_count": len(points),
        "usable_segment_count": len(segments),
        "length_m": sum(segments),
        "segment_median_m": percentile(segments, 0.5),
        "segment_p95_m": percentile(segments, 0.95),
        "positive_turn_rad": positive_turn,
        "negative_turn_rad": negative_turn,
        "turn_sign_changes": sign_changes,
        "bbox": {
            "min_x": min(xs) if xs else None,
            "max_x": max(xs) if xs else None,
            "min_y": min(ys) if ys else None,
            "max_y": max(ys) if ys else None,
            "span_x_m": max(xs) - min(xs) if xs else 0.0,
            "span_y_m": max(ys) - min(ys) if ys else 0.0,
        },
        "start": {"x": points[0][0], "y": points[0][1]} if points else None,
        "goal": {"x": points[-1][0], "y": points[-1][1]} if points else None,
    }


def validate(args):
    path = Path(args.path).expanduser().resolve()
    failures = []
    payload = {}
    points = []
    nonfinite = 0
    if not path.is_file():
        failures.append("path file is missing")
    else:
        try:
            payload, points, nonfinite = load_points(path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            failures.append(str(exc))

    finite_points = [
        point for point in points if math.isfinite(point[0]) and math.isfinite(point[1])
    ]
    metrics = path_metrics(finite_points, args.min_segment_length_m) if finite_points else {
        "pose_count": 0,
        "usable_segment_count": 0,
        "length_m": 0.0,
        "positive_turn_rad": 0.0,
        "negative_turn_rad": 0.0,
        "turn_sign_changes": 0,
        "bbox": {"span_x_m": 0.0, "span_y_m": 0.0},
    }
    digest = sha256_file(path) if path.is_file() else ""

    if nonfinite:
        failures.append("path contains {} non-finite poses".format(nonfinite))
    if metrics["pose_count"] < args.min_poses:
        failures.append("pose_count {} < {}".format(metrics["pose_count"], args.min_poses))
    if metrics["length_m"] < args.min_length_m:
        failures.append("length {:.3f}m < {:.3f}m".format(metrics["length_m"], args.min_length_m))
    if metrics["positive_turn_rad"] < args.min_each_turn_rad:
        failures.append(
            "positive turn {:.3f}rad < {:.3f}rad".format(
                metrics["positive_turn_rad"], args.min_each_turn_rad
            )
        )
    if metrics["negative_turn_rad"] < args.min_each_turn_rad:
        failures.append(
            "negative turn {:.3f}rad < {:.3f}rad".format(
                metrics["negative_turn_rad"], args.min_each_turn_rad
            )
        )
    if metrics["turn_sign_changes"] < args.min_turn_sign_changes:
        failures.append(
            "turn sign changes {} < {}".format(
                metrics["turn_sign_changes"], args.min_turn_sign_changes
            )
        )
    for axis in ("x", "y"):
        maximum = getattr(args, "max_span_{}_m".format(axis))
        actual = metrics["bbox"]["span_{}_m".format(axis)]
        if maximum > 0.0 and actual > maximum + 1e-9:
            failures.append("span_{} {:.3f}m > {:.3f}m".format(axis, actual, maximum))
    if args.expected_sha256 and digest.lower() != args.expected_sha256.lower():
        failures.append(
            "SHA-256 mismatch: expected={}, actual={}".format(args.expected_sha256, digest)
        )

    report = {
        "schema": "spmpc_mocap_s_path_validation_v1",
        "path": str(path),
        "sha256": digest,
        "frame_id": str(payload.get("frame_id", "")),
        "metrics": metrics,
        "pass": not failures,
        "failures": failures,
    }
    if args.report:
        report_path = Path(args.report).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["pass"] else 2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--report")
    parser.add_argument("--expected-sha256", default="")
    parser.add_argument("--min-poses", type=int, default=10)
    parser.add_argument("--min-length-m", type=float, default=0.50)
    parser.add_argument("--min-segment-length-m", type=float, default=1e-5)
    parser.add_argument("--min-each-turn-rad", type=float, default=0.10)
    parser.add_argument("--min-turn-sign-changes", type=int, default=1)
    parser.add_argument("--max-span-x-m", type=float, default=0.0)
    parser.add_argument("--max-span-y-m", type=float, default=0.0)
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(validate(parse_args()))
