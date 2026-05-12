#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""GeoRef 路径平滑与几何指标库 — 入口 + re-export shim。

库实现已迁至 reference_generation/geometry_candidates.py；
本文件保留命令行入口和 parse_args/main，保持旧 rosrun/import 路径兼容。
"""

import reference_generation.geometry_candidates as _geom

# re-export 所有公开符号，保持旧 import 路径兼容
from reference_generation.geometry_candidates import (  # noqa: F401
    clamp_to_origin,
    cumulative_s,
    curvature_series,
    dist,
    dkappa_series,
    fmt,
    generate_for_path,
    interpolate_path,
    load_path,
    metric_row,
    path_id_from_file,
    path_metrics,
    path_to_json,
    percentile,
    resample_path,
    smooth_path,
    write_csv,
    write_json,
    yaw_to_quat,
)


def parse_args():
    return _geom.parse_args()


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
