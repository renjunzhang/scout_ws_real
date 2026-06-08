#!/usr/bin/env python3
"""Analyze fixed-path geometry feasibility for diff-drive tracking.

The script is intentionally offline: it reads a fixed-path JSON, computes geometric
curvature, then reports the speed/turn-rate implications for a nominal v_ref.
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from path_profile_utils import cumulative_s, load_path_points


def finite(values):
    return [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]


def percentile(values, pct):
    vals = sorted(finite(values))
    if not vals:
        return float("nan")
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    ratio = pos - lo
    return vals[lo] * (1.0 - ratio) + vals[hi] * ratio


def dedupe_points(points, min_ds):
    out = []
    for point in points:
        if not out:
            out.append(point)
            continue
        if math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) >= min_ds:
            out.append(point)
    if len(out) < 2:
        raise RuntimeError("path has fewer than 2 usable points after duplicate filtering")
    return out


def signed_triangle_curvature(p0, p1, p2):
    x0, y0 = p0[0], p0[1]
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    a = math.hypot(x1 - x0, y1 - y0)
    b = math.hypot(x2 - x1, y2 - y1)
    c = math.hypot(x2 - x0, y2 - y0)
    denom = a * b * c
    if denom <= 1e-12:
        return 0.0
    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    return 2.0 * cross / denom


def curvature_profile(points):
    n = len(points)
    if n < 3:
        return [0.0] * n
    kappa = [0.0] * n
    for i in range(1, n - 1):
        kappa[i] = signed_triangle_curvature(points[i - 1], points[i], points[i + 1])
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa


def segment_lengths(points):
    return [
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    ]


def max_abs(values):
    vals = finite(values)
    return max((abs(v) for v in vals), default=float("nan"))


def prefix_summary(s_values, kappa, v_ref, windows):
    rows = []
    for window in windows:
        vals = [abs(k) for s, k in zip(s_values, kappa) if s <= window]
        if not vals:
            vals = [abs(kappa[0])]
        kmax = max(vals)
        rows.append(
            {
                "window_m": window,
                "kappa_abs_max_1_m": kmax,
                "omega_req_abs_max_rad_s": abs(v_ref) * kmax,
                "a_lat_req_abs_max_m_s2": v_ref * v_ref * kmax,
            }
        )
    return rows


def band_summary(points, kappa, bands, total_length):
    seg_lens = segment_lengths(points)
    rows = []
    for band in bands:
        length = 0.0
        max_contig = 0.0
        current = 0.0
        for i, seg_len in enumerate(seg_lens):
            active = max(abs(kappa[i]), abs(kappa[i + 1])) >= band
            if active:
                length += seg_len
                current += seg_len
            else:
                max_contig = max(max_contig, current)
                current = 0.0
        max_contig = max(max_contig, current)
        rows.append(
            {
                "kappa_band_1_m": band,
                "length_m": length,
                "fraction": length / total_length if total_length > 1e-9 else float("nan"),
                "max_contiguous_m": max_contig,
            }
        )
    return rows


def analyze_points(points, v_ref, bands, prefix_windows, omega_max, alpha_max):
    s_values = cumulative_s(points)
    total_length = s_values[-1]
    kappa = curvature_profile(points)
    abs_kappa = [abs(k) for k in kappa]
    kappa_max = max(abs_kappa) if abs_kappa else float("nan")
    min_radius = float("inf") if kappa_max <= 1e-12 else 1.0 / kappa_max
    omega_req = [v_ref * k for k in kappa]
    a_lat_req = [v_ref * v_ref * k for k in kappa]
    summary = {
        "points": len(points),
        "length_m": total_length,
        "v_ref_m_s": v_ref,
        "omega_max_rad_s": omega_max,
        "alpha_max_rad_s2_recorded": alpha_max,
        "kappa_abs_max_1_m": kappa_max,
        "kappa_abs_p95_1_m": percentile(abs_kappa, 95.0),
        "kappa_abs_p99_1_m": percentile(abs_kappa, 99.0),
        "min_radius_m": min_radius,
        "omega_req_abs_max_rad_s": max_abs(omega_req),
        "a_lat_req_abs_max_m_s2": max_abs(a_lat_req),
        "omega_limit_exceeded": bool(omega_max > 0.0 and max_abs(omega_req) > omega_max),
        "prefix_windows": prefix_summary(s_values, kappa, v_ref, prefix_windows),
        "curvature_bands": band_summary(points, kappa, bands, total_length),
    }
    samples = [
        {
            "s_m": s,
            "x": p[0],
            "y": p[1],
            "yaw": p[2],
            "kappa_1_m": k,
            "omega_req_rad_s": om,
            "a_lat_req_m_s2": al,
        }
        for s, p, k, om, al in zip(s_values, points, kappa, omega_req, a_lat_req)
    ]
    return summary, samples


def write_json(path, summary):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path, samples):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "s_m",
        "x",
        "y",
        "yaw",
        "kappa_1_m",
        "omega_req_rad_s",
        "a_lat_req_m_s2",
    ]
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)


def print_summary(path_file, summary):
    print(f"[path_feasibility] file={path_file}")
    print(f"points: {summary['points']}")
    print(f"length_m: {summary['length_m']:.3f}")
    print(f"v_ref_m_s: {summary['v_ref_m_s']:.3f}")
    print(f"kappa_abs_max_1_m: {summary['kappa_abs_max_1_m']:.3f}")
    print(f"kappa_abs_p95_1_m: {summary['kappa_abs_p95_1_m']:.3f}")
    print(f"kappa_abs_p99_1_m: {summary['kappa_abs_p99_1_m']:.3f}")
    radius = summary["min_radius_m"]
    radius_text = "inf" if math.isinf(radius) else f"{radius:.3f}"
    print(f"min_radius_m: {radius_text}")
    print(f"omega_req_abs_max_rad_s: {summary['omega_req_abs_max_rad_s']:.3f}")
    print(f"a_lat_req_abs_max_m_s2: {summary['a_lat_req_abs_max_m_s2']:.3f}")
    print(f"omega_max_rad_s: {summary['omega_max_rad_s']:.3f}")
    print(f"omega_limit_exceeded: {str(summary['omega_limit_exceeded']).lower()}")
    print("prefix_windows:")
    for row in summary["prefix_windows"]:
        print(
            "  first {window_m:.2f}m: max|kappa|={kappa_abs_max_1_m:.3f} "
            "max|omega_req|={omega_req_abs_max_rad_s:.3f} "
            "max|a_lat_req|={a_lat_req_abs_max_m_s2:.3f}".format(**row)
        )
    print("curvature_bands:")
    for row in summary["curvature_bands"]:
        print(
            "  |kappa|>={kappa_band_1_m:.2f}: length={length_m:.3f}m "
            "fraction={fraction:.3f} max_contiguous={max_contiguous_m:.3f}m".format(**row)
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path-file", required=True, help="Fixed-path JSON file")
    parser.add_argument("--v-ref", type=float, default=0.25, help="Nominal speed used for feasibility estimates")
    parser.add_argument("--omega-max", type=float, default=1.2, help="Omega limit for omega_req flagging; <=0 disables flag")
    parser.add_argument("--alpha-max", type=float, default=1.2, help="Recorded alpha limit for context only")
    parser.add_argument("--band", type=float, action="append", default=None, help="Curvature band threshold; repeatable")
    parser.add_argument("--prefix-window", type=float, action="append", default=None, help="Prefix window length in meters; repeatable")
    parser.add_argument("--min-ds", type=float, default=1e-4, help="Minimum point spacing kept after duplicate filtering")
    parser.add_argument("--json-out", default="", help="Optional JSON summary output path")
    parser.add_argument("--csv-out", default="", help="Optional per-point CSV output path")
    parser.add_argument("--fail-on-omega-limit", action="store_true", help="Exit nonzero if omega_req exceeds omega_max")
    return parser.parse_args()


def main():
    args = parse_args()
    bands = args.band if args.band is not None else [0.5, 1.0, 2.0, 3.0]
    prefix_windows = args.prefix_window if args.prefix_window is not None else [0.5, 1.0, 2.0]
    points = dedupe_points(load_path_points(args.path_file), max(0.0, args.min_ds))
    summary, samples = analyze_points(
        points,
        args.v_ref,
        sorted(bands),
        sorted(prefix_windows),
        args.omega_max,
        args.alpha_max,
    )
    print_summary(args.path_file, summary)
    if args.json_out:
        write_json(args.json_out, summary)
    if args.csv_out:
        write_csv(args.csv_out, samples)
    if args.fail_on_omega_limit and summary["omega_limit_exceeded"]:
        raise SystemExit(3)


if __name__ == "__main__":
    main()
