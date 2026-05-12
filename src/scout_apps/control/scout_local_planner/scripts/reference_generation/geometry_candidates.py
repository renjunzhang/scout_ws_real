#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate offline anti-slosh path geometry candidates from fixed path JSON files."
    )
    parser.add_argument("inputs", nargs="+", help="fixed path JSON files")
    parser.add_argument("--output-dir", default="/data/a/fixed_paths/candidates")
    parser.add_argument("--ds", type=float, default=0.10, help="candidate resampling spacing in meters")
    parser.add_argument("--smooth-iters", type=int, default=8)
    parser.add_argument("--radius-iters", type=int, default=18)
    parser.add_argument("--smooth-gain", type=float, default=0.35)
    parser.add_argument("--radius-gain", type=float, default=0.45)
    parser.add_argument("--smooth-max-drift", type=float, default=0.25)
    parser.add_argument("--radius-max-drift", type=float, default=0.60)
    parser.add_argument("--summary-csv", default="")
    parser.add_argument("--dry-run", action="store_true", help="print metrics without writing candidate JSON")
    return parser.parse_args()


def load_path(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    poses = data.get("poses", [])
    points = [(float(p["x"]), float(p["y"])) for p in poses]
    if len(points) < 3:
        raise RuntimeError(f"{path}: path needs at least 3 poses")
    endpoint_quats = (
        {key: float(poses[0][key]) for key in ("qx", "qy", "qz", "qw")},
        {key: float(poses[-1][key]) for key in ("qx", "qy", "qz", "qw")},
    )
    return data.get("frame_id", "map"), points, endpoint_quats


def path_id_from_file(path):
    return os.path.splitext(os.path.basename(path))[0]


def cumulative_s(points):
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + dist(points[i - 1], points[i]))
    return s


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def interpolate_path(points, s_values, target_s):
    if target_s <= 0.0:
        return points[0]
    if target_s >= s_values[-1]:
        return points[-1]
    lo = 0
    hi = len(s_values) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if s_values[mid] < target_s:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    s0 = s_values[i - 1]
    s1 = s_values[i]
    if s1 <= s0:
        return points[i]
    r = (target_s - s0) / (s1 - s0)
    x = points[i - 1][0] * (1.0 - r) + points[i][0] * r
    y = points[i - 1][1] * (1.0 - r) + points[i][1] * r
    return (x, y)


def resample_path(points, ds):
    s_values = cumulative_s(points)
    total = s_values[-1]
    n = max(2, int(math.ceil(total / ds)) + 1)
    out = []
    for i in range(n):
        target = min(total, i * ds)
        out.append(interpolate_path(points, s_values, target))
    if dist(out[-1], points[-1]) > 1e-6:
        out.append(points[-1])
    return out


def clamp_to_origin(candidate, origin, max_drift):
    dx = candidate[0] - origin[0]
    dy = candidate[1] - origin[1]
    d = math.hypot(dx, dy)
    if d <= max_drift or d <= 1e-12:
        return candidate
    scale = max_drift / d
    return (origin[0] + dx * scale, origin[1] + dy * scale)


def smooth_path(points, iters, gain, max_drift):
    if len(points) < 3 or iters <= 0:
        return list(points)
    origin = list(points)
    current = list(points)
    for _ in range(iters):
        nxt = list(current)
        for i in range(1, len(current) - 1):
            avg = (
                0.5 * (current[i - 1][0] + current[i + 1][0]),
                0.5 * (current[i - 1][1] + current[i + 1][1]),
            )
            candidate = (
                current[i][0] + gain * (avg[0] - current[i][0]),
                current[i][1] + gain * (avg[1] - current[i][1]),
            )
            nxt[i] = clamp_to_origin(candidate, origin[i], max_drift)
        current = nxt
    return current


def curvature_series(points):
    kappa = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ax, ay = points[i - 1]
        bx, by = points[i]
        cx, cy = points[i + 1]
        ab = dist(points[i - 1], points[i])
        bc = dist(points[i], points[i + 1])
        ac = dist(points[i - 1], points[i + 1])
        denom = ab * bc * ac
        if denom <= 1e-9:
            continue
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        kappa[i] = 2.0 * cross / denom
    return kappa


def dkappa_series(points, kappa):
    s = cumulative_s(points)
    dkappa = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        ds = max(1e-6, s[i + 1] - s[i - 1])
        dkappa[i] = (kappa[i + 1] - kappa[i - 1]) / ds
    return dkappa


def percentile(values, pct):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    pos = (len(values) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    r = pos - lo
    return values[lo] * (1.0 - r) + values[hi] * r


def path_metrics(points, origin=None):
    s = cumulative_s(points)
    kappa = curvature_series(points)
    dkappa = dkappa_series(points, kappa)
    k_abs = [abs(v) for v in kappa]
    dk_abs = [abs(v) for v in dkappa]
    segs = [dist(points[i - 1], points[i]) for i in range(1, len(points))]
    max_drift = 0.0
    if origin is not None and len(origin) == len(points):
        max_drift = max(dist(a, b) for a, b in zip(origin, points))
    return {
        "n": len(points),
        "length_m": s[-1],
        "min_seg_m": min(segs) if segs else float("nan"),
        "kappa_p95": percentile(k_abs, 95.0),
        "kappa_max": max(k_abs) if k_abs else float("nan"),
        "dkappa_p95": percentile(dk_abs, 95.0),
        "dkappa_max": max(dk_abs) if dk_abs else float("nan"),
        "max_drift_m": max_drift,
    }


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return {
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(half),
        "qw": math.cos(half),
    }


def path_to_json(frame_id, points, endpoint_quats, source_path, candidate):
    poses = []
    for i, point in enumerate(points):
        if i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - point[1], points[i + 1][0] - point[0])
        else:
            yaw = math.atan2(point[1] - points[i - 1][1], point[0] - points[i - 1][0])
        quat = yaw_to_quat(yaw)
        if i == 0:
            quat = endpoint_quats[0]
        elif i + 1 == len(points):
            quat = endpoint_quats[1]
        poses.append({
            "x": point[0],
            "y": point[1],
            "z": 0.0,
            **quat,
        })
    return {
        "frame_id": frame_id,
        "candidate": candidate,
        "source_path": source_path,
        "poses": poses,
    }


def fmt(value):
    return "nan" if not math.isfinite(value) else f"{value:.4g}"


def metric_row(path_id, candidate, points, origin):
    metrics = path_metrics(points, origin)
    row = {"path_id": path_id, "candidate": candidate}
    row.update(metrics)
    return row


def write_json(path, payload):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=True, indent=2)
        handle.write("\n")


def write_csv(path, rows):
    if not path or not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def generate_for_path(path, args):
    frame_id, raw_points, endpoint_quats = load_path(path)
    path_id = path_id_from_file(path)
    base = resample_path(raw_points, max(0.02, args.ds))
    candidates = [
        ("original_resampled", base),
        ("smooth", smooth_path(base, args.smooth_iters, args.smooth_gain, args.smooth_max_drift)),
        ("radius", smooth_path(base, args.radius_iters, args.radius_gain, args.radius_max_drift)),
    ]
    rows = []
    for name, points in candidates:
        row = metric_row(path_id, name, points, base)
        rows.append(row)
        print(
            f"{path_id} {name}: n={row['n']} len={fmt(row['length_m'])} "
            f"k_p95={fmt(row['kappa_p95'])} k_max={fmt(row['kappa_max'])} "
            f"dk_p95={fmt(row['dkappa_p95'])} dk_max={fmt(row['dkappa_max'])} "
            f"drift={fmt(row['max_drift_m'])}"
        )
        if not args.dry_run and name != "original_resampled":
            out_path = os.path.join(args.output_dir, f"{path_id}_{name}.json")
            payload = path_to_json(frame_id, points, endpoint_quats, path, name)
            write_json(out_path, payload)
            row["output_path"] = out_path
        else:
            row["output_path"] = ""
    return rows


def main():
    args = parse_args()
    rows = []
    for path in args.inputs:
        rows.extend(generate_for_path(path, args))
    write_csv(args.summary_csv, rows)
    if args.summary_csv:
        print(f"summary_csv: {args.summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
