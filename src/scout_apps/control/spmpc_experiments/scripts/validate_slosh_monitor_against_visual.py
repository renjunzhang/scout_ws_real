#!/usr/bin/env python3
"""Validate model/monitor slosh height against visual liquid-height CSV.

第一版只做曲线对齐和指标统计，不重写 RGB 液面识别。视觉 CSV 可由已有
extract_visual_height.py 或人工后处理生成。
"""

import argparse
import csv
import math
from pathlib import Path

import rosbag


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def mean(values):
    vals = [v for v in values if finite(v)]
    return sum(vals) / len(vals) if vals else float("nan")


def read_model_height_from_bag(bag_path, topic):
    series = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        if topic not in available:
            raise SystemExit(f"topic not found in bag: {topic}")
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            value = safe_float(getattr(msg, "data", float("nan")))
            if topic == "/slosh/height":
                value *= 1000.0  # /slosh/height is m
            series.append((stamp.to_sec(), value))
    return series


def choose_column(fieldnames, candidates):
    lowered = {name.lower(): name for name in fieldnames}
    for cand in candidates:
        if cand.lower() in lowered:
            return lowered[cand.lower()]
    for name in fieldnames:
        low = name.lower()
        if any(cand.lower() in low for cand in candidates):
            return name
    return None


def read_visual_csv(path, unit):
    with Path(path).open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise SystemExit(f"empty visual csv: {path}")
        t_col = choose_column(reader.fieldnames, ["t", "time", "time_s", "stamp", "stamp_sec", "sec"])
        h_col = choose_column(
            reader.fieldnames,
            ["height_mm", "liquid_height_mm", "h_mm", "max_lcr_mm", "max_height_mm", "height", "liquid_height"],
        )
        if t_col is None or h_col is None:
            raise SystemExit(f"cannot infer time/height columns from {reader.fieldnames}")
        out = []
        for row in reader:
            t = safe_float(row.get(t_col))
            h = safe_float(row.get(h_col))
            if unit == "m":
                h *= 1000.0
            if finite(t) and finite(h):
                out.append((t, h))
    return out


def normalize_time(series):
    if not series:
        return []
    t0 = series[0][0]
    return [(t - t0, v) for t, v in series]


def interp(series, t):
    if not series or t < series[0][0] or t > series[-1][0]:
        return float("nan")
    lo = 0
    hi = len(series) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid - 1
    if lo == 0:
        return series[0][1]
    if lo >= len(series):
        return series[-1][1]
    t0, v0 = series[lo - 1]
    t1, v1 = series[lo]
    if t1 <= t0:
        return v1
    r = (t - t0) / (t1 - t0)
    return v0 * (1.0 - r) + v1 * r


def align_series(model, visual, lag=0.0):
    rows = []
    for t, model_h in model:
        visual_h = interp(visual, t + lag)
        if finite(model_h) and finite(visual_h):
            rows.append((t, model_h, visual_h, model_h - visual_h))
    return rows


def rmse(errors):
    vals = [e for e in errors if finite(e)]
    return math.sqrt(sum(e * e for e in vals) / len(vals)) if vals else float("nan")


def corrcoef(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    if len(pairs) < 2:
        return float("nan")
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    vx = sum((x - mx) ** 2 for x, _ in pairs)
    vy = sum((y - my) ** 2 for _, y in pairs)
    if vx <= 1e-12 or vy <= 1e-12:
        return float("nan")
    cov = sum((x - mx) * (y - my) for x, y in pairs)
    return cov / math.sqrt(vx * vy)


def best_lag(model, visual, max_lag_s, step_s):
    best = (0.0, -float("inf"), [])
    n = int(round(max_lag_s / step_s))
    for i in range(-n, n + 1):
        lag = i * step_s
        rows = align_series(model, visual, lag)
        c = corrcoef([r[1] for r in rows], [r[2] for r in rows])
        score = c if finite(c) else -float("inf")
        if score > best[1]:
            best = (lag, score, rows)
    return best


def write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--visual-csv", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--model-topic", default="/slosh/height", choices=("/slosh/height", "/spmpc/slosh_height"))
    parser.add_argument("--visual-unit", choices=("mm", "m"), default="mm")
    parser.add_argument("--absolute-time", action="store_true", help="Do not normalize both series to t=0")
    parser.add_argument("--max-lag-s", type=float, default=2.0)
    parser.add_argument("--lag-step-s", type=float, default=0.05)
    args = parser.parse_args()

    model = read_model_height_from_bag(Path(args.bag), args.model_topic)
    visual = read_visual_csv(args.visual_csv, args.visual_unit)
    if not args.absolute_time:
        model = normalize_time(model)
        visual = normalize_time(visual)
    model = sorted(model)
    visual = sorted(visual)

    lag, corr, rows = best_lag(model, visual, args.max_lag_s, args.lag_step_s)
    errors = [r[3] for r in rows]
    model_vals = [r[1] for r in rows]
    visual_vals = [r[2] for r in rows]

    metrics = {
        "bag": str(Path(args.bag).resolve()),
        "visual_csv": str(Path(args.visual_csv).resolve()),
        "model_topic": args.model_topic,
        "sample_count": len(rows),
        "valid_overlap_s": (rows[-1][0] - rows[0][0]) if len(rows) >= 2 else float("nan"),
        "time_lag_s": lag,
        "corrcoef": corr,
        "height_model_peak_mm": max(model_vals) if model_vals else float("nan"),
        "height_visual_peak_mm": max(visual_vals) if visual_vals else float("nan"),
        "peak_error_mm": (max(model_vals) - max(visual_vals)) if model_vals and visual_vals else float("nan"),
        "rmse_mm": rmse(errors),
        "mae_mm": mean([abs(e) for e in errors]),
        "bias_mm": mean(errors),
    }

    out_dir = Path(args.out_dir).expanduser().resolve()
    write_csv(out_dir / "slosh_validation_metrics.csv", [metrics], list(metrics.keys()))
    write_csv(
        out_dir / "slosh_validation_timeseries.csv",
        [
            {"t_model_s": t, "model_height_mm": mh, "visual_height_mm": vh, "error_mm": err, "lag_s": lag}
            for t, mh, vh, err in rows
        ],
        ["t_model_s", "model_height_mm", "visual_height_mm", "error_mm", "lag_s"],
    )
    print(f"[done] wrote {out_dir / 'slosh_validation_metrics.csv'}")


if __name__ == "__main__":
    main()
