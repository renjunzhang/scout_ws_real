#!/usr/bin/env python3
"""Inspect Q5_test1 outlier segments using slosh inputs and state traces."""

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DEFAULT_VERIFY_ROOT = Path("/data/a/realsense_validation_v2/verify/0325_rezero_bias")
DEFAULT_BAG_ROOT = Path("/data/a/slosh_bags/0325")
DEFAULT_LABEL = "Q5_test1"

BAG_PATTERNS = {
    "Q5_test1": ("qq5", "test1"),
}


@dataclass
class Segment:
    rank: int
    start_frame_index: int
    end_frame_index: int
    start_time_s: float
    end_time_s: float
    num_flagged_rows: int
    raw_error_abs_max_mm: float
    raw_error_mean_mm: float
    raw_error_median_mm: float
    dominant_peak_source_v2: str


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-root", default=str(DEFAULT_VERIFY_ROOT))
    parser.add_argument("--bag-root", default=str(DEFAULT_BAG_ROOT))
    parser.add_argument("--label", default=DEFAULT_LABEL)
    parser.add_argument("--liquid-csv", default="")
    parser.add_argument("--segment-csv", default="")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--center-column",
        default="height_center_rel_mm_bias_corrected_v2",
        help="RealSense main liquid level column.",
    )
    parser.add_argument(
        "--window-pad-s",
        type=float,
        default=1.0,
        help="Extra context on both sides of each outlier segment.",
    )
    parser.add_argument(
        "--max-segments",
        type=int,
        default=6,
        help="Maximum number of top-ranked segments to export.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def mean_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.mean(clean))


def median_or_nan(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(statistics.median(clean))


def max_abs_or_nan(values: Iterable[float]) -> float:
    clean = [abs(float(v)) for v in values if math.isfinite(float(v))]
    if not clean:
        return math.nan
    return float(max(clean))


def find_bag_path(label: str, bag_root: Path) -> Path:
    tokens = BAG_PATTERNS.get(label)
    if tokens is None:
        raise RuntimeError(f"no bag pattern for {label}")
    matches = []
    for bag_path in sorted(bag_root.glob("*.bag")):
        bag_name = bag_path.name.lower()
        if all(token in bag_name for token in tokens):
            matches.append(bag_path)
    if not matches:
        raise RuntimeError(f"no bag matched {label} under {bag_root}")
    if len(matches) > 1:
        raise RuntimeError(f"multiple bags matched {label}: {matches}")
    return matches[0]


def default_liquid_csv(verify_root: Path, label: str) -> Path:
    return verify_root / label / "liquid_height_v2.csv"


def default_segment_csv(verify_root: Path, label: str) -> Path:
    return verify_root / "q5_phase_outlier_analysis" / f"{label}_outlier_segments.csv"


def load_liquid_rows(path: Path, center_column: str) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            rows.append(
                {
                    "frame_index": int(raw["frame_index"]),
                    "stamp": float(raw["stamp"]),
                    "relative_time_s": float(raw["relative_time_s"]),
                    "accept_for_peak_report_v2": int(raw.get("accept_for_peak_report_v2", "0") or "0"),
                    "center_mm": finite_float(raw.get(center_column)),
                    "slosh_mm": math.nan,
                }
            )
    return rows


def load_segments(path: Path, max_segments: int) -> List[Segment]:
    segments: List[Segment] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            segments.append(
                Segment(
                    rank=int(raw["rank"]),
                    start_frame_index=int(raw["start_frame_index"]),
                    end_frame_index=int(raw["end_frame_index"]),
                    start_time_s=float(raw["start_time_s"]),
                    end_time_s=float(raw["end_time_s"]),
                    num_flagged_rows=int(raw["num_flagged_rows"]),
                    raw_error_abs_max_mm=float(raw["raw_error_abs_max_mm"]),
                    raw_error_mean_mm=float(raw["raw_error_mean_mm"]),
                    raw_error_median_mm=float(raw["raw_error_median_mm"]),
                    dominant_peak_source_v2=raw.get("dominant_peak_source_v2", ""),
                )
            )
    segments.sort(key=lambda item: item.rank)
    return segments[:max_segments]


def load_scalar_topic(
    bag_path: Path,
    topic_name: str,
    scale: float = 1.0,
) -> Tuple[List[float], List[float]]:
    times: List[float] = []
    values: List[float] = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            times.append(t.to_sec())
            values.append(float(msg.data) * scale)
    return times, values


def load_state_topic(bag_path: Path) -> Tuple[List[float], List[np.ndarray]]:
    times: List[float] = []
    values: List[np.ndarray] = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, t in bag.read_messages(topics=["/slosh/state"]):
            data = list(msg.data)
            if len(data) < 4:
                continue
            times.append(t.to_sec())
            values.append(np.array(data[:4], dtype=np.float64))
    return times, values


def interpolate_scalar(series_times: Sequence[float], series_values: Sequence[float], ts: float) -> float:
    if not series_times:
        return math.nan
    idx = bisect.bisect_left(series_times, ts)
    if idx <= 0:
        return float(series_values[0])
    if idx >= len(series_times):
        return float(series_values[-1])
    t0 = float(series_times[idx - 1])
    t1 = float(series_times[idx])
    v0 = float(series_values[idx - 1])
    v1 = float(series_values[idx])
    if t1 <= t0:
        return v1
    ratio = (ts - t0) / (t1 - t0)
    return v0 + ratio * (v1 - v0)


def nearest_state_index(state_times: Sequence[float], ts: float) -> int:
    idx = bisect.bisect_left(state_times, ts)
    if idx <= 0:
        return 0
    if idx >= len(state_times):
        return len(state_times) - 1
    if abs(state_times[idx] - ts) <= abs(ts - state_times[idx - 1]):
        return idx
    return idx - 1


def attach_slosh_to_rows(
    rows: List[Dict[str, float]],
    slosh_times: Sequence[float],
    slosh_values: Sequence[float],
):
    if not slosh_times:
        return
    series_t0 = float(slosh_times[0])
    series_t1 = float(slosh_times[-1])
    for row in rows:
        stamp = float(row["stamp"])
        if stamp < series_t0 or stamp > series_t1:
            row["slosh_mm"] = math.nan
        else:
            row["slosh_mm"] = interpolate_scalar(slosh_times, slosh_values, stamp)


def in_window(xs: Sequence[float], ys: Sequence[float], t0: float, t1: float) -> Tuple[List[float], List[float]]:
    out_x: List[float] = []
    out_y: List[float] = []
    for x, y in zip(xs, ys):
        if t0 <= x <= t1:
            out_x.append(float(x))
            out_y.append(float(y))
    return out_x, out_y


def in_window_state(
    xs: Sequence[float], ys: Sequence[np.ndarray], t0: float, t1: float
) -> Tuple[List[float], List[np.ndarray]]:
    out_x: List[float] = []
    out_y: List[np.ndarray] = []
    for x, y in zip(xs, ys):
        if t0 <= x <= t1:
            out_x.append(float(x))
            out_y.append(np.array(y, dtype=np.float64))
    return out_x, out_y


def plot_segment(
    out_path: Path,
    segment: Segment,
    rows_window: Sequence[Dict[str, float]],
    ax_window: Tuple[Sequence[float], Sequence[float]],
    ay_window: Tuple[Sequence[float], Sequence[float]],
    omega_window: Tuple[Sequence[float], Sequence[float]],
    alpha_window: Tuple[Sequence[float], Sequence[float]],
    state_window: Tuple[Sequence[float], Sequence[np.ndarray]],
):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(6, 1, figsize=(13, 14), sharex=True, constrained_layout=True)

    times = [float(row["relative_time_s"]) for row in rows_window]
    center = [float(row["center_mm"]) for row in rows_window]
    slosh = [float(row["slosh_mm"]) for row in rows_window]
    error = [
        (float(row["center_mm"]) - float(row["slosh_mm"]))
        if math.isfinite(float(row["center_mm"])) and math.isfinite(float(row["slosh_mm"]))
        else math.nan
        for row in rows_window
    ]

    axes[0].plot(times, center, color="#d62728", linewidth=1.6, label="RealSense center")
    axes[0].plot(times, slosh, color="#1f77b4", linewidth=1.3, label="/slosh/height")
    axes[0].set_ylabel("height [mm]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, error, color="#9467bd", linewidth=1.4, label="center - /slosh/height")
    axes[1].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[1].set_ylabel("error [mm]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(ax_window[0], ax_window[1], color="#ff7f0e", linewidth=1.3, label="ax_est")
    axes[2].plot(ay_window[0], ay_window[1], color="#2ca02c", linewidth=1.3, label="ay_est")
    axes[2].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[2].set_ylabel("acc [m/s²]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(omega_window[0], omega_window[1], color="#8c564b", linewidth=1.3, label="omega_est_used")
    axes[3].plot(alpha_window[0], alpha_window[1], color="#17becf", linewidth=1.3, label="alpha_est")
    axes[3].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[3].set_ylabel("rot")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)

    state_times = list(state_window[0])
    states = list(state_window[1])
    eta_x = [float(state[0]) for state in states]
    eta_x_dot = [float(state[1]) for state in states]
    eta_y = [float(state[2]) for state in states]
    eta_y_dot = [float(state[3]) for state in states]

    axes[4].plot(state_times, eta_x, color="#d62728", linewidth=1.2, label="eta_x")
    axes[4].plot(state_times, eta_y, color="#1f77b4", linewidth=1.2, label="eta_y")
    axes[4].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[4].set_ylabel("eta [m]")
    axes[4].legend(loc="upper right")
    axes[4].grid(True, alpha=0.25)

    axes[5].plot(state_times, eta_x_dot, color="#ff7f0e", linewidth=1.2, label="eta_x_dot")
    axes[5].plot(state_times, eta_y_dot, color="#2ca02c", linewidth=1.2, label="eta_y_dot")
    axes[5].axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    axes[5].set_ylabel("eta_dot [m/s]")
    axes[5].set_xlabel("relative time [s]")
    axes[5].legend(loc="upper right")
    axes[5].grid(True, alpha=0.25)

    for axis in axes:
        axis.axvspan(segment.start_time_s, segment.end_time_s, color="#cccccc", alpha=0.22)

    axes[0].set_title(
        f"Q5_test1 segment #{segment.rank}: frames {segment.start_frame_index}-{segment.end_frame_index}, "
        f"abs_max={segment.raw_error_abs_max_mm:.3f} mm, peak_source={segment.dominant_peak_source_v2}"
    )

    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def summarize_segment(
    segment: Segment,
    rows_window: Sequence[Dict[str, float]],
    ax_window: Tuple[Sequence[float], Sequence[float]],
    ay_window: Tuple[Sequence[float], Sequence[float]],
    omega_window: Tuple[Sequence[float], Sequence[float]],
    alpha_window: Tuple[Sequence[float], Sequence[float]],
    state_window: Tuple[Sequence[float], Sequence[np.ndarray]],
) -> Dict[str, object]:
    times = [float(row["relative_time_s"]) for row in rows_window]
    center = [float(row["center_mm"]) for row in rows_window]
    slosh = [float(row["slosh_mm"]) for row in rows_window]
    errors = [
        c - s
        for c, s in zip(center, slosh)
        if math.isfinite(c) and math.isfinite(s)
    ]

    states = list(state_window[1])
    eta_x = [float(state[0]) for state in states]
    eta_x_dot = [float(state[1]) for state in states]
    eta_y = [float(state[2]) for state in states]
    eta_y_dot = [float(state[3]) for state in states]
    eta_modal = [math.hypot(float(state[0]), float(state[2])) for state in states]

    return {
        "rank": segment.rank,
        "start_frame_index": segment.start_frame_index,
        "end_frame_index": segment.end_frame_index,
        "start_time_s": f"{segment.start_time_s:.6f}",
        "end_time_s": f"{segment.end_time_s:.6f}",
        "window_start_s": f"{(min(times) if times else math.nan):.6f}",
        "window_end_s": f"{(max(times) if times else math.nan):.6f}",
        "num_rows_window": len(rows_window),
        "num_flagged_rows": segment.num_flagged_rows,
        "mean_error_mm": f"{mean_or_nan(errors):.6f}",
        "median_error_mm": f"{median_or_nan(errors):.6f}",
        "max_abs_error_mm": f"{max_abs_or_nan(errors):.6f}",
        "center_range_mm": f"{(max(center) - min(center)):.6f}" if center else "",
        "slosh_range_mm": f"{(max(slosh) - min(slosh)):.6f}" if slosh else "",
        "max_abs_ax_est_mps2": f"{max_abs_or_nan(ax_window[1]):.6f}",
        "max_abs_ay_est_mps2": f"{max_abs_or_nan(ay_window[1]):.6f}",
        "max_abs_omega_est_radps": f"{max_abs_or_nan(omega_window[1]):.6f}",
        "max_abs_alpha_est_radps2": f"{max_abs_or_nan(alpha_window[1]):.6f}",
        "max_abs_eta_x_m": f"{max_abs_or_nan(eta_x):.6f}",
        "max_abs_eta_y_m": f"{max_abs_or_nan(eta_y):.6f}",
        "max_abs_eta_modal_m": f"{max_abs_or_nan(eta_modal):.6f}",
        "max_abs_eta_x_dot_mps": f"{max_abs_or_nan(eta_x_dot):.6f}",
        "max_abs_eta_y_dot_mps": f"{max_abs_or_nan(eta_y_dot):.6f}",
        "dominant_peak_source_v2": segment.dominant_peak_source_v2,
    }


def build_readme(path: Path, bag_path: Path, summary_rows: Sequence[Dict[str, object]]):
    lines = [
        "# Q5_test1 分段输入/状态分析",
        "",
        "本目录围绕 `Q5_test1` 的高误差坏点段，逐段导出：",
        "- `RealSense center` vs `/slosh/height`",
        "- `/slosh/ax_est`、`/slosh/ay_est`",
        "- `/slosh/omega_est_used`、`/slosh/alpha_est`",
        "- `/slosh/state = [eta_x, eta_x_dot, eta_y, eta_y_dot]`",
        "",
        f"bag: `{bag_path}`",
        "",
        "用途：",
        "- 判断 `/slosh/height` 偏低时对应的输入激励和状态量级",
        "- 区分问题更像输入侧、状态滞后，还是视觉 `center` 局部偏高",
        "",
        "输出：",
        "- `segment_input_state_summary.csv`: 每段窗口的输入/状态峰值摘要",
        "- `segments/*.png`: 每段的 6 行时序图",
        "",
        "当前段摘要：",
    ]
    for row in summary_rows:
        lines.append(
            f"- `#{row['rank']}` frames {row['start_frame_index']}-{row['end_frame_index']}: "
            f"max|error|={row['max_abs_error_mm']} mm, "
            f"max|ay|={row['max_abs_ay_est_mps2']} m/s², "
            f"max|omega|={row['max_abs_omega_est_radps']} rad/s, "
            f"max|eta_modal|={row['max_abs_eta_modal_m']} m"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    verify_root = Path(args.verify_root).expanduser().resolve()
    bag_root = Path(args.bag_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    ensure_dir(out_dir)
    ensure_dir(out_dir / "segments")

    bag_path = find_bag_path(args.label, bag_root)
    liquid_csv = Path(args.liquid_csv).expanduser().resolve() if args.liquid_csv else default_liquid_csv(verify_root, args.label)
    segment_csv = Path(args.segment_csv).expanduser().resolve() if args.segment_csv else default_segment_csv(verify_root, args.label)

    if not liquid_csv.is_file():
        raise SystemExit(f"[ERROR] liquid csv not found: {liquid_csv}")
    if not segment_csv.is_file():
        raise SystemExit(f"[ERROR] segment csv not found: {segment_csv}")

    liquid_rows = load_liquid_rows(liquid_csv, args.center_column)
    segments = load_segments(segment_csv, args.max_segments)
    if not segments:
        raise SystemExit(f"[ERROR] no segments loaded from: {segment_csv}")

    if not liquid_rows:
        raise SystemExit(f"[ERROR] no liquid rows loaded from: {liquid_csv}")
    t0_abs = float(liquid_rows[0]["stamp"])

    slosh_times_abs, slosh_values = load_scalar_topic(bag_path, "/slosh/height", scale=1000.0)
    if not slosh_times_abs:
        raise SystemExit(f"[ERROR] /slosh/height missing in: {bag_path}")
    attach_slosh_to_rows(liquid_rows, slosh_times_abs, slosh_values)

    ax_times_abs, ax_values = load_scalar_topic(bag_path, "/slosh/ax_est", scale=1.0)
    ay_times_abs, ay_values = load_scalar_topic(bag_path, "/slosh/ay_est", scale=1.0)
    omega_times_abs, omega_values = load_scalar_topic(bag_path, "/slosh/omega_est_used", scale=1.0)
    alpha_times_abs, alpha_values = load_scalar_topic(bag_path, "/slosh/alpha_est", scale=1.0)
    state_times_abs, state_values = load_state_topic(bag_path)

    ax_times = [ts - t0_abs for ts in ax_times_abs]
    ay_times = [ts - t0_abs for ts in ay_times_abs]
    omega_times = [ts - t0_abs for ts in omega_times_abs]
    alpha_times = [ts - t0_abs for ts in alpha_times_abs]
    state_times = [ts - t0_abs for ts in state_times_abs]

    summary_rows: List[Dict[str, object]] = []
    for segment in segments:
        window_start = segment.start_time_s - args.window_pad_s
        window_end = segment.end_time_s + args.window_pad_s

        rows_window = [
            row
            for row in liquid_rows
            if int(row["accept_for_peak_report_v2"]) == 1
            and window_start <= float(row["relative_time_s"]) <= window_end
            and math.isfinite(float(row["center_mm"]))
        ]

        ax_window = in_window(ax_times, ax_values, window_start, window_end)
        ay_window = in_window(ay_times, ay_values, window_start, window_end)
        omega_window = in_window(omega_times, omega_values, window_start, window_end)
        alpha_window = in_window(alpha_times, alpha_values, window_start, window_end)
        state_window = in_window_state(state_times, state_values, window_start, window_end)

        png_path = out_dir / "segments" / (
            f"segment_{segment.rank:02d}_frames_{segment.start_frame_index}_{segment.end_frame_index}_inputs.png"
        )
        plot_segment(
            png_path,
            segment,
            rows_window,
            ax_window,
            ay_window,
            omega_window,
            alpha_window,
            state_window,
        )

        summary_rows.append(
            summarize_segment(
                segment,
                rows_window,
                ax_window,
                ay_window,
                omega_window,
                alpha_window,
                state_window,
            )
        )

    summary_csv = out_dir / "segment_input_state_summary.csv"
    with summary_csv.open("w", encoding="utf-8", newline="") as handle:
        fieldnames = list(summary_rows[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    summary_json = out_dir / "segment_input_state_summary.json"
    with summary_json.open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "bag": str(bag_path),
                "liquid_csv": str(liquid_csv),
                "segment_csv": str(segment_csv),
                "center_column": args.center_column,
                "window_pad_s": args.window_pad_s,
                "segments": summary_rows,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    build_readme(out_dir / "README.md", bag_path, summary_rows)

    print(f"[OK] out dir: {out_dir}")
    print(f"[OK] summary csv: {summary_csv}")
    print(f"[OK] summary json: {summary_json}")
    print(f"[OK] readme: {out_dir / 'README.md'}")
    print(f"[OK] segment plots: {out_dir / 'segments'}")


if __name__ == "__main__":
    main()
