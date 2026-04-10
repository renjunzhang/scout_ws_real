#!/usr/bin/env python3
"""Plot training history from an SL/FSL history CSV."""

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(description="Plot train_loss / train_mae / val_mae from a history CSV.")
    parser.add_argument("--history-csv", required=True, help="Path to history CSV.")
    parser.add_argument("--out-png", required=True, help="Output PNG path.")
    parser.add_argument("--title", default="", help="Optional plot title.")
    parser.add_argument(
        "--summary-json",
        default="",
        help="Optional training summary json used to infer target unit and best epoch context.",
    )
    parser.add_argument(
        "--metric-unit",
        default="",
        help="Optional override for MAE unit label, for example 'mm' or 'px'.",
    )
    return parser.parse_args()


def read_history(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(
                {
                    "epoch": float(row["epoch"]),
                    "train_loss": float(row["train_loss"]),
                    "train_mae": float(row["train_mae"]),
                    "val_mae": float(row["val_mae"]),
                }
            )
    if not rows:
        raise RuntimeError(f"history csv has no rows: {path}")
    return rows


def infer_metric_unit(summary_json: Path, metric_unit_override: str) -> str:
    if metric_unit_override.strip():
        return metric_unit_override.strip()
    if not summary_json.exists():
        return "mm"
    with summary_json.open("r", encoding="utf-8") as handle:
        summary = json.load(handle)
    target_column = str(summary.get("target_column", "")).strip()
    if target_column.endswith("_px"):
        return "px"
    return "mm"


def main():
    args = parse_args()
    history_csv = Path(args.history_csv).expanduser().resolve()
    out_png = Path(args.out_png).expanduser().resolve()
    summary_json = Path(args.summary_json).expanduser().resolve() if args.summary_json else Path("")
    out_png.parent.mkdir(parents=True, exist_ok=True)

    rows = read_history(history_csv)
    metric_unit = infer_metric_unit(summary_json, str(args.metric_unit))
    epochs = [row["epoch"] for row in rows]
    train_loss = [row["train_loss"] for row in rows]
    train_mae = [row["train_mae"] for row in rows]
    val_mae = [row["val_mae"] for row in rows]

    best_idx = min(range(len(rows)), key=lambda idx: val_mae[idx])
    best_epoch = epochs[best_idx]
    best_val_mae = val_mae[best_idx]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))

    axes[0].plot(epochs, train_loss, color="#355c7d", linewidth=2.0, label="train_loss")
    axes[0].axvline(best_epoch, color="#666666", linestyle="--", linewidth=1.2, label=f"best epoch={int(best_epoch)}")
    axes[0].set_title("Train Loss")
    axes[0].set_xlabel("epoch")
    axes[0].set_ylabel("loss")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")

    axes[1].plot(epochs, train_mae, color="#2a9d8f", linewidth=2.0, label="train_mae")
    axes[1].plot(epochs, val_mae, color="#d9485f", linewidth=2.0, label="val_mae")
    axes[1].axvline(best_epoch, color="#666666", linestyle="--", linewidth=1.2, label=f"best epoch={int(best_epoch)}")
    axes[1].scatter([best_epoch], [best_val_mae], color="#d9485f", zorder=5)
    axes[1].set_title("MAE")
    axes[1].set_xlabel("epoch")
    axes[1].set_ylabel(metric_unit)
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="upper right")

    title = args.title.strip() if args.title else history_csv.stem
    fig.suptitle(f"{title} | best_val_mae={best_val_mae:.4f} {metric_unit} @ epoch {int(best_epoch)}")
    fig.tight_layout()
    fig.subplots_adjust(top=0.82)
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

    print(f"[OK] out png: {out_png}")
    print(f"[OK] best epoch: {int(best_epoch)} | best val mae: {best_val_mae:.6f} {metric_unit}")


if __name__ == "__main__":
    main()
