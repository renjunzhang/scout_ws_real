#!/usr/bin/env python3
"""Export a sequence dataset for slosh-height regression from debug_session.csv."""

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


FEATURE_NAMES = (
    "v_mps",
    "omega_radps",
    "ax_cmd_mps2",
    "ay_model_mps2",
    "imu_ax_mps2",
    "imu_ay_mps2",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Export a fixed-length dynamics-only dataset from a v2 debug session. "
            "Each sample uses a past motion window as input and a liquid-height column as target."
        )
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument(
        "--debug-dir",
        default="",
        help="Debug session directory containing debug_session.csv.",
    )
    source_group.add_argument(
        "--session-csv",
        default="",
        help="Direct path to debug_session.csv.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <debug_dir>/dynamics_dataset or <session_csv_parent>/dynamics_dataset.",
    )
    parser.add_argument(
        "--target-column",
        default="human_height_mm",
        help="Primary target column. Examples: human_height_mm, center_rel_mm_v2, slosh_height_mm.",
    )
    parser.add_argument(
        "--fallback-target-column",
        default="",
        help="Optional fallback target column used when the primary target is empty for a row.",
    )
    parser.add_argument(
        "--history-frames",
        type=int,
        default=30,
        help="Number of history frames included in each sample window. Default: 30.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Stride between frames inside each sample window. Default: 1.",
    )
    parser.add_argument(
        "--max-window-gap-sec",
        type=float,
        default=0.20,
        help="Reject a sample if adjacent frames inside its window exceed this time gap. Default: 0.20 s.",
    )
    parser.add_argument(
        "--show-first",
        type=int,
        default=5,
        help="Print the first N exported samples as a smoke summary. Default: 5.",
    )
    return parser.parse_args()


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    text = str(raw_value).strip()
    if text == "":
        return math.nan
    try:
        value = float(text)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def session_csv_from_args(args) -> Path:
    if args.session_csv:
        return Path(args.session_csv).expanduser().resolve()
    return (Path(args.debug_dir).expanduser().resolve() / "debug_session.csv").resolve()


def output_dir_from_args(args, session_csv_path: Path) -> Path:
    if args.out_dir:
        return Path(args.out_dir).expanduser().resolve()
    return (session_csv_path.parent / "dynamics_dataset").resolve()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def read_session_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"session csv has no rows: {path}")
    return rows


def nearest_finite_index(values: np.ndarray, start_idx: int, direction: int) -> Optional[int]:
    idx = int(start_idx)
    n = int(values.shape[0])
    while 0 <= idx < n:
        if math.isfinite(float(values[idx])):
            return idx
        idx += int(direction)
    return None


def finite_difference(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    derivatives = np.full(values.shape, np.nan, dtype=np.float64)
    n = int(values.shape[0])
    for idx in range(n):
        if not math.isfinite(float(times[idx])):
            continue
        left_idx = nearest_finite_index(values, idx - 1, -1)
        right_idx = nearest_finite_index(values, idx + 1, +1)
        if left_idx is not None and right_idx is not None:
            dt = float(times[right_idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[left_idx]) / dt
                continue
        if right_idx is not None and math.isfinite(float(values[idx])):
            dt = float(times[right_idx] - times[idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[right_idx] - values[idx]) / dt
                continue
        if left_idx is not None and math.isfinite(float(values[idx])):
            dt = float(times[idx] - times[left_idx])
            if dt > 1e-9:
                derivatives[idx] = float(values[idx] - values[left_idx]) / dt
                continue
    return derivatives


def resolve_target_value(
    row: Dict[str, str],
    target_column: str,
    fallback_target_column: str,
) -> Tuple[float, str]:
    primary = finite_float(row.get(target_column))
    if math.isfinite(primary):
        return primary, target_column
    if fallback_target_column:
        fallback = finite_float(row.get(fallback_target_column))
        if math.isfinite(fallback):
            return fallback, fallback_target_column
    return math.nan, ""


def build_feature_table(rows: Sequence[Dict[str, str]]) -> Dict[str, np.ndarray]:
    relative_time_s = np.asarray([finite_float(row.get("relative_time_s")) for row in rows], dtype=np.float64)
    stamp = np.asarray([finite_float(row.get("stamp")) for row in rows], dtype=np.float64)
    frame_index = np.asarray([int(finite_float(row.get("frame_index"))) for row in rows], dtype=np.int32)

    v_mps = np.asarray([finite_float(row.get("cmd_vel_linear_x")) for row in rows], dtype=np.float64)
    omega_radps = np.asarray([finite_float(row.get("cmd_vel_angular_z")) for row in rows], dtype=np.float64)
    imu_ax_mps2 = np.asarray([finite_float(row.get("imu_ax")) for row in rows], dtype=np.float64)
    imu_ay_mps2 = np.asarray([finite_float(row.get("imu_ay")) for row in rows], dtype=np.float64)

    ax_cmd_mps2 = finite_difference(v_mps, relative_time_s)
    ay_model_mps2 = np.full(v_mps.shape, np.nan, dtype=np.float64)
    finite_mask = np.isfinite(v_mps) & np.isfinite(omega_radps)
    ay_model_mps2[finite_mask] = v_mps[finite_mask] * omega_radps[finite_mask]

    return {
        "frame_index": frame_index,
        "stamp": stamp,
        "relative_time_s": relative_time_s,
        "v_mps": v_mps,
        "omega_radps": omega_radps,
        "ax_cmd_mps2": ax_cmd_mps2,
        "ay_model_mps2": ay_model_mps2,
        "imu_ax_mps2": imu_ax_mps2,
        "imu_ay_mps2": imu_ay_mps2,
    }


def export_dataset(args) -> int:
    session_csv_path = session_csv_from_args(args)
    out_dir = output_dir_from_args(args, session_csv_path)
    ensure_dir(out_dir)

    rows = read_session_rows(session_csv_path)
    fields = set(rows[0].keys())
    if args.target_column not in fields:
        raise RuntimeError(f"missing target column in session csv: {args.target_column}")
    if args.fallback_target_column and args.fallback_target_column not in fields:
        raise RuntimeError(f"missing fallback target column in session csv: {args.fallback_target_column}")

    feature_table = build_feature_table(rows)
    frame_index = feature_table["frame_index"]
    stamp = feature_table["stamp"]
    relative_time_s = feature_table["relative_time_s"]

    feature_matrix = np.stack([feature_table[name] for name in FEATURE_NAMES], axis=1)

    history_frames = max(1, int(args.history_frames))
    stride = max(1, int(args.stride))
    required_history = (history_frames - 1) * stride
    max_window_gap_sec = max(0.0, float(args.max_window_gap_sec))

    samples_x: List[np.ndarray] = []
    samples_y: List[float] = []
    samples_time_offsets: List[np.ndarray] = []
    samples_frame_index: List[int] = []
    samples_stamp: List[float] = []
    samples_relative_time: List[float] = []
    samples_target_source: List[str] = []
    manifest_rows: List[Dict[str, object]] = []

    skip_counts = Counter()
    target_source_counts = Counter()

    for end_idx, row in enumerate(rows):
        target_value, target_source = resolve_target_value(
            row,
            args.target_column,
            args.fallback_target_column,
        )
        if not math.isfinite(target_value):
            skip_counts["missing_target"] += 1
            continue
        if end_idx < required_history:
            skip_counts["insufficient_history"] += 1
            continue

        window_indices = list(range(end_idx - required_history, end_idx + 1, stride))
        if len(window_indices) != history_frames:
            skip_counts["window_length_mismatch"] += 1
            continue

        window_times = relative_time_s[window_indices]
        if not np.all(np.isfinite(window_times)):
            skip_counts["nonfinite_window_time"] += 1
            continue
        if history_frames >= 2 and max_window_gap_sec > 0.0:
            if np.any(np.diff(window_times) > max_window_gap_sec):
                skip_counts["window_gap_too_large"] += 1
                continue

        window_features = feature_matrix[window_indices, :]
        if not np.all(np.isfinite(window_features)):
            skip_counts["nonfinite_window_feature"] += 1
            continue

        target_idx = int(window_indices[-1])
        current_values = window_features[-1, :]
        time_offsets = window_times - float(window_times[-1])

        samples_x.append(window_features.astype(np.float32))
        samples_y.append(float(target_value))
        samples_time_offsets.append(time_offsets.astype(np.float32))
        samples_frame_index.append(int(frame_index[target_idx]))
        samples_stamp.append(float(stamp[target_idx]))
        samples_relative_time.append(float(relative_time_s[target_idx]))
        samples_target_source.append(str(target_source))
        target_source_counts[str(target_source)] += 1

        manifest_rows.append(
            {
                "sample_index": len(samples_y) - 1,
                "frame_index": int(frame_index[target_idx]),
                "stamp": f"{float(stamp[target_idx]):.9f}",
                "relative_time_s": f"{float(relative_time_s[target_idx]):.9f}",
                "target_height_mm": f"{float(target_value):.6f}",
                "target_source": str(target_source),
                "window_start_frame_index": int(frame_index[int(window_indices[0])]),
                "window_end_frame_index": int(frame_index[target_idx]),
                "v_mps": f"{float(current_values[0]):.6f}",
                "omega_radps": f"{float(current_values[1]):.6f}",
                "ax_cmd_mps2": f"{float(current_values[2]):.6f}",
                "ay_model_mps2": f"{float(current_values[3]):.6f}",
                "imu_ax_mps2": f"{float(current_values[4]):.6f}",
                "imu_ay_mps2": f"{float(current_values[5]):.6f}",
            }
        )

    if not samples_y:
        raise RuntimeError(
            "no training samples were exported; try labeling more frames or use "
            "--target-column center_rel_mm_v2 for a smoke test"
        )

    x_array = np.asarray(samples_x, dtype=np.float32)
    y_array = np.asarray(samples_y, dtype=np.float32)
    time_offsets_array = np.asarray(samples_time_offsets, dtype=np.float32)
    frame_index_array = np.asarray(samples_frame_index, dtype=np.int32)
    stamp_array = np.asarray(samples_stamp, dtype=np.float64)
    relative_time_array = np.asarray(samples_relative_time, dtype=np.float64)
    target_source_array = np.asarray(samples_target_source, dtype=object)

    npz_path = out_dir / "slosh_dynamics_dataset.npz"
    np.savez_compressed(
        str(npz_path),
        X=x_array,
        y_mm=y_array,
        time_offsets_s=time_offsets_array,
        feature_names=np.asarray(FEATURE_NAMES, dtype=object),
        frame_index=frame_index_array,
        stamp=stamp_array,
        relative_time_s=relative_time_array,
        target_source=target_source_array,
        target_column=np.asarray([args.target_column], dtype=object),
        fallback_target_column=np.asarray([args.fallback_target_column], dtype=object),
        history_frames=np.asarray([history_frames], dtype=np.int32),
        stride=np.asarray([stride], dtype=np.int32),
    )

    summary_csv_path = out_dir / "slosh_dynamics_samples.csv"
    with summary_csv_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "sample_index",
            "frame_index",
            "stamp",
            "relative_time_s",
            "target_height_mm",
            "target_source",
            "window_start_frame_index",
            "window_end_frame_index",
            "v_mps",
            "omega_radps",
            "ax_cmd_mps2",
            "ay_model_mps2",
            "imu_ax_mps2",
            "imu_ay_mps2",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in manifest_rows:
            writer.writerow(row)

    metadata = {
        "session_csv": str(session_csv_path),
        "out_dir": str(out_dir),
        "num_rows": len(rows),
        "num_samples": int(y_array.shape[0]),
        "history_frames": int(history_frames),
        "stride": int(stride),
        "max_window_gap_sec": float(max_window_gap_sec),
        "target_column": str(args.target_column),
        "fallback_target_column": str(args.fallback_target_column),
        "feature_names": list(FEATURE_NAMES),
        "target_source_counts": dict(target_source_counts),
        "skip_counts": dict(skip_counts),
        "npz_path": str(npz_path),
        "samples_csv_path": str(summary_csv_path),
        "target_stats_mm": {
            "min": float(np.min(y_array)),
            "median": float(np.median(y_array)),
            "max": float(np.max(y_array)),
            "mean": float(np.mean(y_array)),
        },
    }
    metadata_path = out_dir / "slosh_dynamics_metadata.json"
    with metadata_path.open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=False, indent=2)

    print(f"[OK] session csv: {session_csv_path}")
    print(f"[OK] target column: {args.target_column}")
    if args.fallback_target_column:
        print(f"[OK] fallback target column: {args.fallback_target_column}")
    print(f"[OK] exported samples: {y_array.shape[0]}")
    print(f"[OK] history frames: {history_frames}")
    print(f"[OK] stride: {stride}")
    print(f"[OK] npz: {npz_path}")
    print(f"[OK] samples csv: {summary_csv_path}")
    print(f"[OK] metadata json: {metadata_path}")
    print(f"[OK] target stats [mm]: min={np.min(y_array):.3f} median={np.median(y_array):.3f} max={np.max(y_array):.3f}")
    if skip_counts:
        print(f"[OK] skip counts: {dict(skip_counts)}")
    if target_source_counts:
        print(f"[OK] target source counts: {dict(target_source_counts)}")

    show_first = max(0, int(args.show_first))
    for row in manifest_rows[:show_first]:
        print(
            "[OK] sample[{sample_index}] frame={frame_index} target={target_height_mm}mm "
            "src={target_source} v={v_mps} omega={omega_radps} ax={ax_cmd_mps2} ay={ay_model_mps2}".format(
                **row
            )
        )

    return 0


def main():
    try:
        args = parse_args()
        return export_dataset(args)
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by Ctrl+C")
        return 130


if __name__ == "__main__":
    sys.exit(main())
