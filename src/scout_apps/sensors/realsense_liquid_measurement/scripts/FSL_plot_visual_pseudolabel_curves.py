#!/usr/bin/env python3
"""Plot val/test curves for the visual pseudolabel experiment."""

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


SERIES_SPECS = [
    ("target_slosh_height_mm", "target /slosh/height", "#111111", 2.0, 1.0),
    ("pred_visual", "visual prediction", "#d1495b", 1.8, 0.95),
    ("pred_peak_rel_mm_v2_affine", "peak affine baseline", "#2a9d8f", 1.4, 0.85),
    ("pred_center_rel_mm_v2_affine", "center affine baseline", "#457b9d", 1.4, 0.85),
    ("pred_train_mean_baseline", "train-mean baseline", "#a0a0a0", 1.2, 0.9),
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot val/test time-series curves from FSL_visual_pseudolabel_predictions.csv."
    )
    parser.add_argument("--predictions-csv", required=True, help="Path to FSL_visual_pseudolabel_predictions.csv.")
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <predictions_parent>/curves.",
    )
    parser.add_argument(
        "--summary-json",
        default="",
        help="Optional FSL_visual_pseudolabel_summary.json used for title annotations.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=160,
        help="Figure DPI. Default: 160.",
    )
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


def plot_panel(ax, rows: Sequence[Dict[str, str]], title: str):
    times = [float(finite_float(row.get("relative_time_s")) or 0.0) for row in rows]
    for column, label, color, linewidth, alpha in SERIES_SPECS:
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


def save_combined_figure(
    groups: Sequence[Tuple[str, str, List[Dict[str, str]]]],
    summary: Dict,
    out_path: Path,
    dpi: int,
):
    num_panels = len(groups)
    fig, axes = plt.subplots(num_panels, 1, figsize=(14, 4 * num_panels), squeeze=False, sharex=False)
    axes_flat = axes[:, 0]
    for ax, (split_name, session_name, rows) in zip(axes_flat, groups):
        plot_panel(ax, rows, format_panel_title(split_name, session_name, rows, summary))
    handles, labels = axes_flat[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=min(5, len(labels)), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=int(dpi), bbox_inches="tight")
    plt.close(fig)


def save_split_figures(
    groups: Sequence[Tuple[str, str, List[Dict[str, str]]]],
    summary: Dict,
    out_dir: Path,
    dpi: int,
):
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
            fig.legend(handles, labels, loc="upper center", ncol=min(5, len(labels)), frameon=False)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(out_dir / f"FSL_visual_curves_{split_name}.png", dpi=int(dpi), bbox_inches="tight")
        plt.close(fig)


def main() -> int:
    try:
        args = parse_args()
        predictions_csv = Path(args.predictions_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else (predictions_csv.parent / "curves").resolve()
        ensure_dir(out_dir)

        rows = read_csv_rows(predictions_csv)
        summary = load_optional_summary(Path(args.summary_json).expanduser().resolve()) if args.summary_json else {}
        groups = group_rows(rows)
        if not groups:
            raise RuntimeError("no grouped rows found in predictions csv")

        combined_png = out_dir / "FSL_visual_curves_all.png"
        save_combined_figure(groups, summary, combined_png, dpi=int(args.dpi))
        save_split_figures(groups, summary, out_dir, dpi=int(args.dpi))

        print(f"[OK] combined png: {combined_png}")
        for split_name, _, _ in groups:
            split_png = out_dir / f"FSL_visual_curves_{split_name}.png"
            if split_png.exists():
                print(f"[OK] split png: {split_png}")
        for split_name, session_name, rows in groups:
            print(f"[OK] panel: split={split_name} session={session_name} rows={len(rows)}")
        return 0
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by Ctrl+C")
        return 130


if __name__ == "__main__":
    sys.exit(main())
