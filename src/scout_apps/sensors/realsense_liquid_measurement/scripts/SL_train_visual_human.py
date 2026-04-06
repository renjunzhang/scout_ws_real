#!/usr/bin/env python3
"""Train a minimal visual regressor from ROI images to human peak labels."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from FSL_train_visual_pseudolabel import MinimalVisualRegressor, load_single_image
from SL_supervised_common import filter_rows_for_split, group_metrics, load_json, read_csv_rows, regression_metrics
from SL_train_baseline import choose_device, set_seed


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Train a minimal visual regressor from single-frame ROI images to human_peak_mm, "
            "using existing manifest/split artifacts."
        )
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest.csv.")
    parser.add_argument("--split-json", required=True, help="Path to split json with train/val/test bag ids.")
    parser.add_argument("--out-dir", required=True, help="Output directory for checkpoint and logs.")
    parser.add_argument(
        "--target-column",
        default="human_peak_mm",
        help="Human target column. Default: human_peak_mm.",
    )
    parser.add_argument(
        "--image-column",
        default="roi_debug_path",
        help="Manifest image column to use. Default: roi_debug_path.",
    )
    parser.add_argument("--image-height", type=int, default=96, help="Resized image height. Default: 96.")
    parser.add_argument("--image-width", type=int, default=192, help="Resized image width. Default: 192.")
    parser.add_argument("--hidden-dim", type=int, default=64, help="MLP hidden dim after CNN. Default: 64.")
    parser.add_argument("--epochs", type=int, default=60, help="Number of training epochs. Default: 60.")
    parser.add_argument("--batch-size", type=int, default=32, help="Mini-batch size. Default: 32.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate. Default: 1e-3.")
    parser.add_argument("--weight-decay", type=float, default=1e-5, help="AdamW weight decay. Default: 1e-5.")
    parser.add_argument("--patience", type=int, default=10, help="Early-stop patience on val MAE. Default: 10.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed. Default: 7.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N val/test rows. Default: 5.")
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


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
    return value if np.isfinite(value) else None


def build_image_samples(
    rows: Sequence[Dict[str, str]],
    image_column: str,
    target_column: str,
    image_height: int,
    image_width: int,
) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    samples: List[Dict[str, object]] = []
    skip_counts = {
        "missing_target": 0,
        "missing_image_path": 0,
        "image_not_found": 0,
        "image_load_error": 0,
    }
    for row in rows:
        target = finite_float(row.get(target_column))
        if target is None:
            skip_counts["missing_target"] += 1
            continue
        image_raw = str(row.get(image_column, "")).strip()
        if image_raw == "":
            skip_counts["missing_image_path"] += 1
            continue
        image_path = Path(image_raw).expanduser()
        if not image_path.exists():
            skip_counts["image_not_found"] += 1
            continue
        try:
            image_tensor = load_single_image(image_path, image_height=image_height, image_width=image_width)
        except (OSError, ValueError):
            skip_counts["image_load_error"] += 1
            continue
        samples.append(
            {
                "x": image_tensor.astype(np.float32),
                "y": float(target),
                "row_id": str(row.get("row_id", "")),
                "session_id": str(row.get("session_id", "")),
                "session_name": str(row.get("session_name", "")),
                "bag_id": str(row.get("bag_id", "")),
                "frame_index": int(float(row.get("frame_index", -1))),
                "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
                "image_path": str(image_path),
                "baseline_slosh_raw": finite_float(row.get("slosh_height_mm")),
                "baseline_peak_raw": finite_float(row.get("peak_rel_mm_v2")),
                "baseline_center_raw": finite_float(row.get("center_rel_mm_v2")),
            }
        )
    return samples, skip_counts


def image_stats_from_samples(samples: Sequence[Dict[str, object]]) -> Tuple[np.ndarray, np.ndarray]:
    x = np.asarray([sample["x"] for sample in samples], dtype=np.float32)
    mean = np.mean(x, axis=(0, 2, 3), dtype=np.float64).astype(np.float32)
    std = np.std(x, axis=(0, 2, 3), dtype=np.float64).astype(np.float32)
    std[std < 1e-6] = 1.0
    return mean, std


def normalize_image_samples(
    samples: Sequence[Dict[str, object]],
    image_mean: np.ndarray,
    image_std: np.ndarray,
) -> List[Dict[str, object]]:
    normalized: List[Dict[str, object]] = []
    mean = image_mean.reshape(1, -1, 1, 1)
    std = image_std.reshape(1, -1, 1, 1)
    for sample in samples:
        updated = dict(sample)
        x = np.asarray(sample["x"], dtype=np.float32)
        updated["x"] = ((x - mean[0]) / std[0]).astype(np.float32)
        normalized.append(updated)
    return normalized


def target_stats_from_samples(samples: Sequence[Dict[str, object]]) -> Dict[str, float]:
    targets = np.asarray([float(sample["y"]) for sample in samples], dtype=np.float32)
    mean = float(np.mean(targets))
    std = float(np.std(targets))
    if std < 1e-6:
        std = 1.0
    return {"mean": mean, "std": std}


def samples_to_loader(
    samples: Sequence[Dict[str, object]],
    batch_size: int,
    shuffle: bool,
    target_stats: Dict[str, float],
) -> DataLoader:
    x_np = np.asarray([sample["x"] for sample in samples], dtype=np.float32)
    y_np = np.asarray(
        [(float(sample["y"]) - float(target_stats["mean"])) / float(target_stats["std"]) for sample in samples],
        dtype=np.float32,
    )
    dataset = TensorDataset(torch.from_numpy(x_np), torch.from_numpy(y_np))
    return DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=bool(shuffle))


def predict_samples(
    model: nn.Module,
    samples: Sequence[Dict[str, object]],
    device: torch.device,
    target_stats: Dict[str, float],
    batch_size: int = 64,
) -> List[float]:
    if not samples:
        return []
    model.eval()
    preds: List[float] = []
    with torch.no_grad():
        for start_idx in range(0, len(samples), max(1, int(batch_size))):
            batch = samples[start_idx : start_idx + max(1, int(batch_size))]
            x_np = np.asarray([sample["x"] for sample in batch], dtype=np.float32)
            x_tensor = torch.from_numpy(x_np).to(device=device)
            pred_norm = model(x_tensor).detach().cpu().numpy()
            preds.extend(
                [
                    float(value * float(target_stats["std"]) + float(target_stats["mean"]))
                    for value in pred_norm
                ]
            )
    return preds


def collect_metrics(targets: Sequence[float], preds: Sequence[Optional[float]]) -> Dict[str, Optional[float]]:
    filtered_targets = []
    filtered_preds = []
    for target, pred in zip(targets, preds):
        if pred is None:
            continue
        filtered_targets.append(float(target))
        filtered_preds.append(float(pred))
    summary = regression_metrics(filtered_targets, filtered_preds)
    summary["coverage"] = 0.0 if not targets else float(len(filtered_preds) / len(targets))
    return summary


def fit_affine_baseline(
    targets: Sequence[float],
    baseline_values: Sequence[Optional[float]],
) -> Optional[Dict[str, float]]:
    x_values = []
    y_values = []
    for target, baseline in zip(targets, baseline_values):
        if baseline is None:
            continue
        x_values.append(float(baseline))
        y_values.append(float(target))
    if len(x_values) < 2:
        return None
    x_np = np.asarray(x_values, dtype=np.float64)
    y_np = np.asarray(y_values, dtype=np.float64)
    x_mean = float(np.mean(x_np))
    y_mean = float(np.mean(y_np))
    x_var = float(np.var(x_np))
    if x_var <= 1e-12:
        return None
    cov = float(np.mean((x_np - x_mean) * (y_np - y_mean)))
    slope = cov / x_var
    intercept = y_mean - slope * x_mean
    return {"slope": float(slope), "intercept": float(intercept)}


def apply_affine_baseline(
    affine: Optional[Dict[str, float]],
    baseline_values: Sequence[Optional[float]],
) -> List[Optional[float]]:
    if affine is None:
        return [None for _ in baseline_values]
    slope = float(affine["slope"])
    intercept = float(affine["intercept"])
    preds: List[Optional[float]] = []
    for baseline in baseline_values:
        if baseline is None:
            preds.append(None)
            continue
        preds.append(float(slope * float(baseline) + intercept))
    return preds


def write_history_csv(path: Path, history_rows: Sequence[Dict[str, object]]):
    fieldnames = ["epoch", "train_loss", "train_mae", "val_mae"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in history_rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_predictions_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "split",
        "row_id",
        "session_id",
        "session_name",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "image_path",
        "target_human_peak_mm",
        "baseline_slosh_height_mm",
        "pred_train_mean_baseline",
        "baseline_peak_rel_mm_v2",
        "pred_peak_rel_mm_v2_affine",
        "baseline_center_rel_mm_v2",
        "pred_center_rel_mm_v2_affine",
        "pred_visual",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> int:
    try:
        args = parse_args()
        set_seed(int(args.seed))
        device = choose_device(args.device)

        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        split_json_path = Path(args.split_json).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        manifest_rows = read_csv_rows(manifest_csv)
        split_json = load_json(split_json_path)

        train_rows = filter_rows_for_split(manifest_rows, split_json, "train")
        val_rows = filter_rows_for_split(manifest_rows, split_json, "val")
        test_rows = filter_rows_for_split(manifest_rows, split_json, "test")

        train_samples, train_skip_counts = build_image_samples(
            train_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
        )
        val_samples, val_skip_counts = build_image_samples(
            val_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
        )
        test_samples, test_skip_counts = build_image_samples(
            test_rows,
            image_column=args.image_column,
            target_column=args.target_column,
            image_height=int(args.image_height),
            image_width=int(args.image_width),
        )

        if not train_samples:
            raise RuntimeError("no train image samples after loading")
        if not val_samples:
            raise RuntimeError("no val image samples after loading")
        if not test_samples:
            raise RuntimeError("no test image samples after loading")

        image_mean, image_std = image_stats_from_samples(train_samples)
        train_samples = normalize_image_samples(train_samples, image_mean, image_std)
        val_samples = normalize_image_samples(val_samples, image_mean, image_std)
        test_samples = normalize_image_samples(test_samples, image_mean, image_std)
        target_stats = target_stats_from_samples(train_samples)

        train_loader = samples_to_loader(
            train_samples,
            batch_size=int(args.batch_size),
            shuffle=True,
            target_stats=target_stats,
        )

        model = MinimalVisualRegressor(
            image_height=int(args.image_height),
            image_width=int(args.image_width),
            hidden_dim=int(args.hidden_dim),
        ).to(device=device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=float(args.lr), weight_decay=float(args.weight_decay))
        criterion = nn.HuberLoss(delta=1.0)

        best_state = None
        best_epoch = -1
        best_val_mae = float("inf")
        epochs_without_improvement = 0
        history_rows: List[Dict[str, object]] = []

        for epoch in range(1, int(args.epochs) + 1):
            model.train()
            running_loss = 0.0
            batch_count = 0
            for batch_x, batch_y in train_loader:
                batch_x = batch_x.to(device=device)
                batch_y = batch_y.to(device=device)
                optimizer.zero_grad()
                pred_y = model(batch_x)
                loss = criterion(pred_y, batch_y)
                loss.backward()
                optimizer.step()
                running_loss += float(loss.item())
                batch_count += 1

            train_preds = predict_samples(model, train_samples, device=device, target_stats=target_stats)
            val_preds = predict_samples(model, val_samples, device=device, target_stats=target_stats)
            train_metrics = regression_metrics([float(sample["y"]) for sample in train_samples], train_preds)
            val_metrics = regression_metrics([float(sample["y"]) for sample in val_samples], val_preds)
            val_mae = float(val_metrics["mae"]) if val_metrics["mae"] is not None else float("inf")
            history_rows.append(
                {
                    "epoch": epoch,
                    "train_loss": running_loss / max(1, batch_count),
                    "train_mae": train_metrics["mae"],
                    "val_mae": val_metrics["mae"],
                }
            )
            if val_mae + 1e-9 < best_val_mae:
                best_val_mae = val_mae
                best_epoch = epoch
                best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
                epochs_without_improvement = 0
            else:
                epochs_without_improvement += 1
                if epochs_without_improvement >= int(args.patience):
                    break

        if best_state is None:
            raise RuntimeError("training failed to produce a best checkpoint")

        model.load_state_dict(best_state)
        train_preds = predict_samples(model, train_samples, device=device, target_stats=target_stats)
        val_preds = predict_samples(model, val_samples, device=device, target_stats=target_stats)
        test_preds = predict_samples(model, test_samples, device=device, target_stats=target_stats)

        train_targets = [float(sample["y"]) for sample in train_samples]
        val_targets = [float(sample["y"]) for sample in val_samples]
        test_targets = [float(sample["y"]) for sample in test_samples]
        train_slosh = [sample["baseline_slosh_raw"] for sample in train_samples]
        val_slosh = [sample["baseline_slosh_raw"] for sample in val_samples]
        test_slosh = [sample["baseline_slosh_raw"] for sample in test_samples]
        train_peak = [sample["baseline_peak_raw"] for sample in train_samples]
        val_peak = [sample["baseline_peak_raw"] for sample in val_samples]
        test_peak = [sample["baseline_peak_raw"] for sample in test_samples]
        train_center = [sample["baseline_center_raw"] for sample in train_samples]
        val_center = [sample["baseline_center_raw"] for sample in val_samples]
        test_center = [sample["baseline_center_raw"] for sample in test_samples]

        train_mean = float(np.mean(np.asarray(train_targets, dtype=np.float64)))
        peak_affine = fit_affine_baseline(train_targets, train_peak)
        center_affine = fit_affine_baseline(train_targets, train_center)
        mean_val = [train_mean for _ in val_targets]
        mean_test = [train_mean for _ in test_targets]
        peak_affine_val = apply_affine_baseline(peak_affine, val_peak)
        peak_affine_test = apply_affine_baseline(peak_affine, test_peak)
        center_affine_val = apply_affine_baseline(center_affine, val_center)
        center_affine_test = apply_affine_baseline(center_affine, test_center)

        checkpoint_path = out_dir / "SL_visual_human.pt"
        history_csv_path = out_dir / "SL_visual_human_history.csv"
        predictions_csv_path = out_dir / "SL_visual_human_predictions.csv"
        summary_json_path = out_dir / "SL_visual_human_summary.json"
        bagwise_csv_path = out_dir / "SL_visual_human_bagwise.csv"

        torch.save(
            {
                "model_type": "sl_visual_cnn_regressor_v1",
                "target_column": str(args.target_column),
                "image_column": str(args.image_column),
                "image_height": int(args.image_height),
                "image_width": int(args.image_width),
                "hidden_dim": int(args.hidden_dim),
                "image_mean": image_mean.tolist(),
                "image_std": image_std.tolist(),
                "target_mean": float(target_stats["mean"]),
                "target_std": float(target_stats["std"]),
                "best_epoch": int(best_epoch),
                "best_val_mae": float(best_val_mae),
                "model_state_dict": best_state,
                "manifest_csv": str(manifest_csv),
                "split_json": str(split_json_path),
            },
            checkpoint_path,
        )
        write_history_csv(history_csv_path, history_rows)

        prediction_rows: List[Dict[str, object]] = []
        for split_name, samples, preds, peak_affine_preds, center_affine_preds in (
            ("val", val_samples, val_preds, peak_affine_val, center_affine_val),
            ("test", test_samples, test_preds, peak_affine_test, center_affine_test),
        ):
            for sample, pred, peak_affine_pred, center_affine_pred in zip(
                samples, preds, peak_affine_preds, center_affine_preds
            ):
                prediction_rows.append(
                    {
                        "split": split_name,
                        "row_id": sample["row_id"],
                        "session_id": sample["session_id"],
                        "session_name": sample["session_name"],
                        "bag_id": sample["bag_id"],
                        "frame_index": int(sample["frame_index"]),
                        "relative_time_s": float(sample["relative_time_s"]),
                        "image_path": sample["image_path"],
                        "target_human_peak_mm": float(sample["y"]),
                        "baseline_slosh_height_mm": sample["baseline_slosh_raw"],
                        "pred_train_mean_baseline": train_mean,
                        "baseline_peak_rel_mm_v2": sample["baseline_peak_raw"],
                        "pred_peak_rel_mm_v2_affine": peak_affine_pred,
                        "baseline_center_rel_mm_v2": sample["baseline_center_raw"],
                        "pred_center_rel_mm_v2_affine": center_affine_pred,
                        "pred_visual": float(pred),
                    }
                )
        write_predictions_csv(predictions_csv_path, prediction_rows)

        bagwise_rows = []
        for model_name, pred_key in (
            ("mean", "pred_train_mean_baseline"),
            ("slosh_raw", "baseline_slosh_height_mm"),
            ("peak_affine", "pred_peak_rel_mm_v2_affine"),
            ("center_affine", "pred_center_rel_mm_v2_affine"),
            ("visual", "pred_visual"),
        ):
            for row in group_metrics(
                [
                    {
                        "bag_id": row["bag_id"],
                        "target": row["target_human_peak_mm"],
                        pred_key: row[pred_key],
                    }
                    for row in prediction_rows
                    if row["split"] == "test"
                ],
                pred_key,
            ):
                bagwise_rows.append({"model_name": model_name, **row})
        with bagwise_csv_path.open("w", newline="", encoding="utf-8") as handle:
            fieldnames = ["model_name", "bag_id", "count", "mae", "rmse", "bias_mean", "bias_median", "corr"]
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in bagwise_rows:
                writer.writerow(row)

        summary = {
            "checkpoint_path": str(checkpoint_path),
            "history_csv": str(history_csv_path),
            "predictions_csv": str(predictions_csv_path),
            "bagwise_csv": str(bagwise_csv_path),
            "manifest_csv": str(manifest_csv),
            "split_json": str(split_json_path),
            "task": "visual_human_peak_regression",
            "target_column": str(args.target_column),
            "image_column": str(args.image_column),
            "device": str(device),
            "image_height": int(args.image_height),
            "image_width": int(args.image_width),
            "hidden_dim": int(args.hidden_dim),
            "best_epoch": int(best_epoch),
            "best_val_mae": float(best_val_mae),
            "num_train_samples": len(train_samples),
            "num_val_samples": len(val_samples),
            "num_test_samples": len(test_samples),
            "train_skip_counts": train_skip_counts,
            "val_skip_counts": val_skip_counts,
            "test_skip_counts": test_skip_counts,
            "baseline_metrics": {
                "val_train_mean": collect_metrics(val_targets, mean_val),
                "test_train_mean": collect_metrics(test_targets, mean_test),
                "val_slosh_raw": collect_metrics(val_targets, val_slosh),
                "test_slosh_raw": collect_metrics(test_targets, test_slosh),
                "val_peak_affine": collect_metrics(val_targets, peak_affine_val),
                "test_peak_affine": collect_metrics(test_targets, peak_affine_test),
                "val_center_affine": collect_metrics(val_targets, center_affine_val),
                "test_center_affine": collect_metrics(test_targets, center_affine_test),
            },
            "visual_metrics": {
                "train": regression_metrics(train_targets, train_preds),
                "val": regression_metrics(val_targets, val_preds),
                "test": regression_metrics(test_targets, test_preds),
            },
        }
        with summary_json_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] checkpoint: {checkpoint_path}")
        print(f"[OK] history csv: {history_csv_path}")
        print(f"[OK] predictions csv: {predictions_csv_path}")
        print(f"[OK] summary json: {summary_json_path}")
        print(
            f"[OK] samples train={len(train_samples)} val={len(val_samples)} test={len(test_samples)} "
            f"| image_column={args.image_column} target={args.target_column}"
        )
        print(f"[OK] baseline test slosh raw mae: {summary['baseline_metrics']['test_slosh_raw']['mae']}")
        print(f"[OK] baseline test peak affine mae: {summary['baseline_metrics']['test_peak_affine']['mae']}")
        print(f"[OK] baseline test center affine mae: {summary['baseline_metrics']['test_center_affine']['mae']}")
        print(f"[OK] visual test metrics: {summary['visual_metrics']['test']}")
        show_first = max(0, int(args.show_first))
        for row in prediction_rows[:show_first]:
            print(
                "[OK] row={row_id} split={split} target={target_human_peak_mm:.6f} "
                "pred_visual={pred_visual:.6f}".format(**row)
            )
        return 0
    except RuntimeError as exc:
        print(f"[WARN] {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by Ctrl+C")
        return 130


if __name__ == "__main__":
    sys.exit(main())
