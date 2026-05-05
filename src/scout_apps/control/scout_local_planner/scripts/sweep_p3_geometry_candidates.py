#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Generate a small sweep of smoothed P3 geometry candidates."""

import argparse
import csv
import os

from generate_anti_slosh_path_candidates import (
    load_path,
    metric_row,
    path_id_from_file,
    path_to_json,
    resample_path,
    smooth_path,
    write_json,
)


def parse_float_list(text):
    return [float(item) for item in text.split(",") if item.strip()]


def parse_int_list(text):
    return [int(item) for item in text.split(",") if item.strip()]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="source fixed path JSON")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--summary-csv", required=True)
    parser.add_argument("--ds", type=float, default=0.10)
    parser.add_argument("--iters", default="18,28,40,56")
    parser.add_argument("--gains", default="0.35,0.45,0.55")
    parser.add_argument("--max-drifts", default="0.08,0.12,0.18,0.25,0.35")
    parser.add_argument("--top-k", type=int, default=12)
    return parser.parse_args()


def candidate_score(row):
    # Prefer low curvature/curvature-rate without excessive geometric drift or path shortening.
    return (
        row["kappa_p95"],
        row["dkappa_p95"],
        row["kappa_max"],
        row["dkappa_max"],
        row["max_drift_m"],
    )


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    frame_id, raw_points, endpoint_quats = load_path(args.input)
    path_id = path_id_from_file(args.input)
    base = resample_path(raw_points, max(0.02, args.ds))
    rows = []
    candidates = []

    for iters in parse_int_list(args.iters):
        for gain in parse_float_list(args.gains):
            for max_drift in parse_float_list(args.max_drifts):
                name = f"geo_i{iters}_g{int(round(gain * 100)):02d}_d{int(round(max_drift * 100)):02d}"
                points = smooth_path(base, iters, gain, max_drift)
                row = metric_row(path_id, name, points, base)
                row["iters"] = iters
                row["gain"] = gain
                row["max_drift_limit_m"] = max_drift
                row["output_path"] = os.path.abspath(os.path.join(args.output_dir, f"{path_id}_{name}.json"))
                rows.append(row)
                candidates.append((row, points))

    rows.sort(key=candidate_score)
    selected_names = {row["candidate"] for row in rows[: max(1, args.top_k)]}
    for row, points in candidates:
        if row["candidate"] not in selected_names:
            continue
        payload = path_to_json(frame_id, points, endpoint_quats, args.input, row["candidate"])
        write_json(row["output_path"], payload)

    write_csv(args.summary_csv, rows)
    print(f"generated_top_k={len(selected_names)} total={len(rows)}")
    for row in rows[: max(1, args.top_k)]:
        print(
            f"{row['candidate']}: len={row['length_m']:.3f} "
            f"k95={row['kappa_p95']:.3f} kmax={row['kappa_max']:.3f} "
            f"dk95={row['dkappa_p95']:.3f} drift={row['max_drift_m']:.3f} "
            f"path={row['output_path']}"
        )
    print(f"summary_csv: {args.summary_csv}")


if __name__ == "__main__":
    raise SystemExit(main())
