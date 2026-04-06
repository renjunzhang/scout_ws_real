#!/usr/bin/env python3
"""Plot val/test curves for the visual human-label SL experiment."""

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BASE_SERIES_SPECS = [
    ("target_human_peak_mm", "human peak", "#111111", 2.2, 1.0),
    ("pred_visual", "SL visual", "#d1495b", 1.9, 0.95),
    ("baseline_slosh_height_mm", "bag /slosh/height", "#457b9d", 1.5, 0.95),
    ("pred_center_rel_mm_v2_affine", "center affine", "#2a9d8f", 1.3, 0.85),
    ("pred_peak_rel_mm_v2_affine", "peak affine", "#8d5a97", 1.3, 0.85),
    ("pred_train_mean_baseline", "train-mean", "#9a9a9a", 1.1, 0.8),
]
REPLAY_SERIES_SPEC = ("replay_imu_height_mm", "imu replay /slosh/height", "#ff7f0e", 1.7, 0.95)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot val/test time-series curves from SL_visual_human_predictions.csv."
    )
    parser.add_argument("--predictions-csv", required=True, help="Path to SL_visual_human_predictions.csv.")
    parser.add_argument(
        "--summary-json",
        default="",
        help="Optional SL_visual_human_summary.json used for title annotations.",
    )
    parser.add_argument(
        "--replay-root",
        default="",
        help=(
            "Optional replay root containing <bag_id>/slosh_recomputed_aligned.csv. "
            "If provided, the selected replay column is merged into the plots."
        ),
    )
    parser.add_argument(
        "--replay-column",
        default="recomputed_slosh_height_mm",
        help="Replay csv column to overlay. Default: recomputed_slosh_height_mm.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <predictions_parent>/curves.",
    )
    parser.add_argument("--dpi", type=int, default=160, help="Figure DPI. Default: 160.")
    return parser.parse_args()


def finite_float(raw_value) -> Optional[float]:
    if raw_value is None:
        return None
    text = str(raw_value).strip()
    if text == "":
        return None
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_optional_summary(path: Path) -> Dict:
    if not path.exists():
        return {}
    import json

    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def sort_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, str]]:
    return sorted(
        rows,
        key=lambda row: (
            str(row.get("split", "")),
            str(row.get("session_name", "")),
            float(finite_float(row.get("relative_time_s")) or 0.0),
            int(finite_float(row.get("frame_index")) or 0),
        ),
    )


def group_rows(rows: Sequence[Dict[str, str]]) -> List[Tuple[str, str, List[Dict[str, str]]]]:
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = defaultdict(list)
    for row in rows:
        key = (str(row.get("split", "")), str(row.get("session_name", "")))
        grouped[key].append(dict(row))
    ordered = []
    for split_name, session_name in sorted(grouped.keys()):
        ordered.append((split_name, session_name, sort_rows(grouped[(split_name, session_name)])))
    return ordered


def interpolate_series(query_times: Sequence[float], base_times: Sequence[float], base_values: Sequence[Optional[float]]) -> List[Optional[float]]:
    pairs = [(float(t), float(v)) for t, v in zip(base_times, base_values) if v is not None]
    if len(pairs) < 2:
        return [None for _ in query_times]
    times = [p[0] for p in pairs]
    values = [p[1] for p in pairs]
    out: List[Optional[float]] = []
    idx = 0
    for qt in query_times:
        q = float(qt)
        if q < times[0] or q > times[-1]:
            out.append(None)
            continue
        while idx + 1 < len(times) and times[idx + 1] < q:
            idx += 1
        if idx + 1 >= len(times):
            out.append(values[idx])
            continue
        t0, t1 = times[idx], times[idx + 1]
        v0, v1 = values[idx], values[idx + 1]
        if abs(t1 - t0) < 1e-9:
            out.append(v0)
        else:
            alpha = (q - t0) / (t1 - t0)
            out.append(v0 + alpha * (v1 - v0))
    return out


def merge_replay_rows(rows: Sequence[Dict[str, str]], replay_root: Path, replay_column: str) -> Tuple[List[Dict[str, str]], Dict[str, int]]:
    merged = [dict(row) for row in rows]
    stats = {"bags_with_replay": 0, "rows_with_replay": 0}
    by_bag: Dict[str, List[int]] = defaultdict(list)
    for idx, row in enumerate(merged):
        by_bag[str(row.get("bag_id", ""))].append(idx)
    for bag_id, indices in sorted(by_bag.items()):
        replay_csv = replay_root / bag_id / "slosh_recomputed_aligned.csv"
        if not replay_csv.exists():
            continue
        replay_rows = read_csv_rows(replay_csv)
        replay_times = [float(finite_float(row.get("relative_time_s")) or 0.0) for row in replay_rows]
        replay_values = [finite_float(row.get(replay_column)) for row in replay_rows]
        query_times = [float(finite_float(merged[idx].get("relative_time_s")) or 0.0) for idx in indices]
        interp = interpolate_series(query_times, replay_times, replay_values)
        stats["bags_with_replay"] += 1
        for row_idx, value in zip(indices, interp):
            merged[row_idx]["replay_imu_height_mm"] = "" if value is None else f"{float(value):.9f}"
            if value is not None:
                stats["rows_with_replay"] += 1
    return merged, stats


def format_panel_title(split_name: str, session_name: str, rows: Sequence[Dict[str, str]], summary: Dict) -> str:
    title = f"{split_name}: {session_name}"
    if not summary:
        return title
    visual_metrics = summary.get("visual_metrics", {})
    metrics = visual_metrics.get(split_name, {})
    mae = metrics.get("mae")
    corr = metrics.get("corr")
    if mae is None:
        return title
    if corr is None:
        return f"{title} | visual MAE={float(mae):.4f}"
    return f"{title} | visual MAE={float(mae):.4f}, corr={float(corr):.3f}"


def active_series_specs(rows: Sequence[Dict[str, str]]) -> List[Tuple[str, str, str, float, float]]:
    specs = list(BASE_SERIES_SPECS)
    if any(finite_float(row.get(REPLAY_SERIES_SPEC[0])) is not None for row in rows):
        specs.append(REPLAY_SERIES_SPEC)
    return specs


def plot_panel(ax, rows: Sequence[Dict[str, str]], title: str):
    times = [float(finite_float(row.get("relative_time_s")) or 0.0) for row in rows]
    for column, label, color, linewidth, alpha in active_series_specs(rows):
        values = [finite_float(row.get(column)) for row in rows]
        if not any(value is not None for value in values):
            continue
        ax.plot(
            times,
            [float("nan") if value is None else float(value) for value in values],
            label=label,
            color=color,
            linewidth=linewidth,
            alpha=alpha,
        )
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("relative_time_s")
    ax.set_ylabel("height (mm)")
    ax.grid(True, alpha=0.25)


def save_combined_figure(groups: Sequence[Tuple[str, str, List[Dict[str, str]]]], summary: Dict, out_path: Path, dpi: int):
    num_panels = len(groups)
    fig, axes = plt.subplots(num_panels, 1, figsize=(14, 4 * num_panels), squeeze=False, sharex=False)
    axes_flat = axes[:, 0]
    for ax, (split_name, session_name, rows) in zip(axes_flat, groups):
        plot_panel(ax, rows, format_panel_title(split_name, session_name, rows, summary))
    handles, labels = axes_flat[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=min(6, len(labels)), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=int(dpi), bbox_inches="tight")
    plt.close(fig)


def save_split_figures(groups: Sequence[Tuple[str, str, List[Dict[str, str]]]], summary: Dict, out_dir: Path, dpi: int):
    by_split: Dict[str, List[Tuple[str, str, List[Dict[str, str]]]]] = defaultdict(list)
    for group in groups:
        by_split[group[0]].append(group)
    for split_name, split_groups in sorted(by_split.items()):
        fig, axes = plt.subplots(len(split_groups), 1, figsize=(14, 4 * len(split_groups)), squeeze=False, sharex=False)
        axes_flat = axes[:, 0]
        for ax, (_, session_name, rows) in zip(axes_flat, split_groups):
            plot_panel(ax, rows, format_panel_title(split_name, session_name, rows, summary))
        handles, labels = axes_flat[0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels, loc="upper center", ncol=min(6, len(labels)), frameon=False)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(out_dir / f"SL_visual_human_curves_{split_name}.png", dpi=int(dpi), bbox_inches="tight")
        plt.close(fig)


def main() -> int:
    try:
        args = parse_args()
        predictions_csv = Path(args.predictions_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else (predictions_csv.parent / "curves").resolve()
        ensure_dir(out_dir)

        rows = read_csv_rows(predictions_csv)
        summary = load_optional_summary(Path(args.summary_json).expanduser().resolve()) if args.summary_json else {}
        replay_stats = {}
        if args.replay_root:
            rows, replay_stats = merge_replay_rows(
                rows,
                replay_root=Path(args.replay_root).expanduser().resolve(),
                replay_column=str(args.replay_column),
            )
        groups = group_rows(rows)
        if not groups:
            raise RuntimeError("no grouped rows found in predictions csv")

        combined_png = out_dir / "SL_visual_human_curves_all.png"
        save_combined_figure(groups, summary, combined_png, dpi=int(args.dpi))
        save_split_figures(groups, summary, out_dir, dpi=int(args.dpi))

        print(f"[OK] combined png: {combined_png}")
        for split_name, _, _ in groups:
            split_png = out_dir / f"SL_visual_human_curves_{split_name}.png"
            if split_png.exists():
                print(f"[OK] split png: {split_png}")
        if replay_stats:
            print(
                f"[OK] replay merge: bags_with_replay={replay_stats['bags_with_replay']} "
                f"rows_with_replay={replay_stats['rows_with_replay']}"
            )
        for split_name, session_name, group_rows_ in groups:
            print(f"[OK] panel: split={split_name} session={session_name} rows={len(group_rows_)}")
        return 0
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by Ctrl+C")
        return 130


if __name__ == "__main__":
    sys.exit(main())
