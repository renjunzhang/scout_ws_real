#!/usr/bin/env python3
"""Run a bag-split pseudolabel pretraining experiment using /slosh/height."""

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from torch import nn

from SL_supervised_common import (
    build_window_samples_from_columns,
    feature_stats_from_samples,
    normalize_samples,
    read_csv_rows,
    regression_metrics,
)
from SL_train_baseline import (
    build_model,
    choose_device,
    parse_positive_dims,
    predict_samples,
    samples_to_loader,
    set_seed,
    target_stats_from_samples,
)


DEFAULT_TRAIN_SESSIONS = "Q0_test1,Q5_test1"
DEFAULT_VAL_SESSIONS = "Q0_test2"
DEFAULT_TEST_SESSIONS = "Q5_test2"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run a bag-split pseudolabel pretraining loop: train on some bags' /slosh/height, "
            "validate/test on held-out bags."
        )
    )
    parser.add_argument(
        "--debug-root",
        default="/data/a/realsense_validation_v2/debug_reannotate/0330",
        help="Root containing debug-session directories. Default: /data/a/realsense_validation_v2/debug_reannotate/0330",
    )
    parser.add_argument(
        "--out-dir",
        default=(
            "/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/"
            "fsl_runs/FSL_pseudolabel_pretrain_0330"
        ),
        help="Output directory for checkpoints, predictions, and summaries.",
    )
    parser.add_argument(
        "--train-sessions",
        default=DEFAULT_TRAIN_SESSIONS,
        help=f"Comma-separated session names for train. Default: {DEFAULT_TRAIN_SESSIONS}",
    )
    parser.add_argument(
        "--val-sessions",
        default=DEFAULT_VAL_SESSIONS,
        help=f"Comma-separated session names for val. Default: {DEFAULT_VAL_SESSIONS}",
    )
    parser.add_argument(
        "--test-sessions",
        default=DEFAULT_TEST_SESSIONS,
        help=f"Comma-separated session names for test. Default: {DEFAULT_TEST_SESSIONS}",
    )
    parser.add_argument(
        "--models",
        default="b2_mlp,b2_tcn",
        help="Comma-separated model list. Choices: b2_mlp,b2_tcn. Default: b2_mlp,b2_tcn.",
    )
    parser.add_argument("--history-frames", type=int, default=20, help="History length K. Default: 20.")
    parser.add_argument("--stride", type=int, default=1, help="Frame stride inside each window. Default: 1.")
    parser.add_argument(
        "--max-window-gap-sec",
        type=float,
        default=0.20,
        help="Reject a window if adjacent frames exceed this gap. Default: 0.20 s.",
    )
    parser.add_argument("--epochs", type=int, default=60, help="Training epochs. Default: 60.")
    parser.add_argument("--batch-size", type=int, default=128, help="Mini-batch size. Default: 128.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate. Default: 1e-3.")
    parser.add_argument("--weight-decay", type=float, default=1e-5, help="AdamW weight decay. Default: 1e-5.")
    parser.add_argument("--patience", type=int, default=8, help="Early-stop patience. Default: 8.")
    parser.add_argument("--hidden-dims", default="128,64", help="MLP hidden dims. Default: 128,64.")
    parser.add_argument("--tcn-channels", default="32,64", help="TCN channels. Default: 32,64.")
    parser.add_argument("--tcn-kernel-size", type=int, default=3, help="TCN kernel size. Default: 3.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed. Default: 7.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print first N test rows. Default: 5.")
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def parse_name_list(raw_value: str, field_name: str) -> List[str]:
    names = [part.strip() for part in str(raw_value).split(",") if part.strip()]
    if not names:
        raise RuntimeError(f"{field_name} cannot be empty")
    if len(set(names)) != len(names):
        raise RuntimeError(f"{field_name} contains duplicates: {names}")
    return names


def parse_model_list(raw_value: str) -> List[str]:
    models = [part.strip() for part in str(raw_value).split(",") if part.strip()]
    if not models:
        raise RuntimeError("models list cannot be empty")
    allowed = {"b2_mlp", "b2_tcn"}
    unknown = [model for model in models if model not in allowed]
    if unknown:
        raise RuntimeError(f"unsupported models: {unknown}")
    return models


def validate_split_names(train_names: Sequence[str], val_names: Sequence[str], test_names: Sequence[str]):
    train_set = set(train_names)
    val_set = set(val_names)
    test_set = set(test_names)
    if train_set & val_set or train_set & test_set or val_set & test_set:
        raise RuntimeError("train/val/test sessions must be disjoint")


def infer_session_identity(debug_dir: Path, session_meta: Dict) -> Tuple[str, str, str]:
    bag_raw = str(session_meta.get("bag", "")).strip()
    bag_id = Path(bag_raw).stem if bag_raw else debug_dir.name
    date_id = debug_dir.parent.name
    session_name = debug_dir.name
    session_id = f"{date_id}/{session_name}" if date_id else session_name
    return session_id, session_name, bag_id


def load_session_rows(debug_dir: Path) -> Tuple[List[Dict[str, str]], Dict[str, object]]:
    session_csv = debug_dir / "debug_session.csv"
    session_json = debug_dir / "debug_session.json"
    rows = read_csv_rows(session_csv)
    if not session_json.exists():
        raise FileNotFoundError(str(session_json))
    with session_json.open("r", encoding="utf-8") as handle:
        session_meta = json.load(handle)
    if not isinstance(session_meta, dict):
        raise RuntimeError(f"invalid session json: {session_json}")
    session_id, session_name, bag_id = infer_session_identity(debug_dir, session_meta)
    prepared = []
    for raw in rows:
        row = dict(raw)
        frame_index = int(float(row.get("frame_index", -1)))
        row["row_id"] = f"{session_id}:{frame_index}"
        row["session_id"] = session_id
        row["session_name"] = session_name
        row["bag_id"] = bag_id
        row["debug_dir"] = str(debug_dir)
        prepared.append(row)
    meta = {
        "session_id": session_id,
        "session_name": session_name,
        "bag_id": bag_id,
        "bag_path": str(session_meta.get("bag", "")),
        "debug_dir": str(debug_dir),
        "num_rows": len(prepared),
    }
    return prepared, meta


def collect_debug_sessions(debug_root: Path) -> Dict[str, Dict[str, object]]:
    if not debug_root.exists():
        raise FileNotFoundError(str(debug_root))
    sessions: Dict[str, Dict[str, object]] = {}
    for session_dir in sorted(debug_root.glob("*/")):
        session_csv = session_dir / "debug_session.csv"
        session_json = session_dir / "debug_session.json"
        if not session_csv.exists() or not session_json.exists():
            continue
        rows, meta = load_session_rows(session_dir)
        sessions[str(meta["session_name"])] = {"rows": rows, "meta": meta}
    if not sessions:
        raise RuntimeError(f"no debug sessions found under {debug_root}")
    return sessions


def pick_rows_for_sessions(
    sessions: Dict[str, Dict[str, object]],
    session_names: Sequence[str],
) -> Tuple[List[Dict[str, str]], List[Dict[str, object]]]:
    missing = [name for name in session_names if name not in sessions]
    if missing:
        raise RuntimeError(f"requested sessions not found: {missing}")
    rows: List[Dict[str, str]] = []
    metas: List[Dict[str, object]] = []
    for name in session_names:
        rows.extend([dict(row) for row in sessions[name]["rows"]])
        metas.append(dict(sessions[name]["meta"]))
    return rows, metas


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


def train_model(
    model_name: str,
    train_samples: Sequence[Dict[str, object]],
    val_samples: Sequence[Dict[str, object]],
    args,
    device: torch.device,
) -> Tuple[nn.Module, Dict[str, object], List[Dict[str, object]]]:
    feature_mean, feature_std = feature_stats_from_samples(train_samples)
    train_norm = normalize_samples(train_samples, feature_mean, feature_std)
    val_norm = normalize_samples(val_samples, feature_mean, feature_std)
    target_stats = target_stats_from_samples(train_norm)
    hidden_dims = parse_positive_dims(args.hidden_dims, "hidden_dims")
    tcn_channels = parse_positive_dims(args.tcn_channels, "tcn_channels")

    model = build_model(
        model_type=model_name,
        history_frames=int(args.history_frames),
        feature_dim=int(train_norm[0]["x"].shape[1]),
        hidden_dims=hidden_dims,
        tcn_channels=tcn_channels,
        tcn_kernel_size=int(args.tcn_kernel_size),
    ).to(device=device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=float(args.lr), weight_decay=float(args.weight_decay))
    criterion = nn.HuberLoss(delta=1.0)
    train_loader = samples_to_loader(
        train_norm,
        batch_size=int(args.batch_size),
        shuffle=True,
        target_stats=target_stats,
        model_type=model_name,
    )

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

        train_preds = predict_samples(model, train_norm, device=device, target_stats=target_stats, model_type=model_name)
        val_preds = predict_samples(model, val_norm, device=device, target_stats=target_stats, model_type=model_name)
        train_metrics = regression_metrics([float(sample["y"]) for sample in train_norm], train_preds)
        val_metrics = regression_metrics([float(sample["y"]) for sample in val_norm], val_preds)
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
        raise RuntimeError(f"failed to train {model_name}")
    model.load_state_dict(best_state)
    metadata = {
        "model_name": model_name,
        "best_epoch": int(best_epoch),
        "best_val_mae": float(best_val_mae),
        "feature_mean": feature_mean,
        "feature_std": feature_std,
        "target_stats": target_stats,
        "hidden_dims": hidden_dims,
        "tcn_channels": tcn_channels,
        "tcn_kernel_size": int(args.tcn_kernel_size),
        "history_frames": int(args.history_frames),
        "feature_dim": int(train_norm[0]["x"].shape[1]),
    }
    return model, metadata, history_rows


def predict_with_training_metadata(
    model: nn.Module,
    model_name: str,
    metadata: Dict[str, object],
    samples: Sequence[Dict[str, object]],
    device: torch.device,
) -> List[float]:
    normalized = normalize_samples(samples, metadata["feature_mean"], metadata["feature_std"])
    return predict_samples(
        model,
        normalized,
        device=device,
        target_stats=metadata["target_stats"],
        model_type=model_name,
    )


def write_history_csv(path: Path, history_rows: Sequence[Dict[str, object]]):
    fieldnames = ["epoch", "train_loss", "train_mae", "val_mae"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in history_rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def save_checkpoint(path: Path, metadata: Dict[str, object], model_state_dict: Dict[str, torch.Tensor], args, debug_root: Path):
    torch.save(
        {
            "model_type": str(metadata["model_name"]),
            "target_column": "slosh_height_mm",
            "history_frames": int(metadata["history_frames"]),
            "stride": int(args.stride),
            "max_window_gap_sec": float(args.max_window_gap_sec),
            "hidden_dims": list(metadata["hidden_dims"]),
            "tcn_channels": list(metadata["tcn_channels"]),
            "tcn_kernel_size": int(metadata["tcn_kernel_size"]),
            "feature_names": [
                "v_mps",
                "omega_radps",
                "ax_cmd_mps2",
                "ay_model_mps2",
                "imu_ax_mps2",
                "imu_ay_mps2",
            ],
            "feature_mean": np.asarray(metadata["feature_mean"], dtype=np.float32).tolist(),
            "feature_std": np.asarray(metadata["feature_std"], dtype=np.float32).tolist(),
            "target_mean": float(metadata["target_stats"]["mean"]),
            "target_std": float(metadata["target_stats"]["std"]),
            "best_epoch": int(metadata["best_epoch"]),
            "best_val_mae": float(metadata["best_val_mae"]),
            "model_state_dict": model_state_dict,
            "debug_root": str(debug_root),
        },
        path,
    )


def write_predictions_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "split",
        "row_id",
        "session_id",
        "session_name",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "target_slosh_height_mm",
        "pred_train_mean_baseline",
        "baseline_peak_rel_mm_v2",
        "pred_peak_rel_mm_v2_affine",
        "baseline_center_rel_mm_v2",
        "pred_center_rel_mm_v2_affine",
        "pred_b2_mlp",
        "pred_b2_tcn",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def build_prediction_rows(
    split_name: str,
    samples: Sequence[Dict[str, object]],
    train_mean: float,
    peak_affine_preds: Sequence[Optional[float]],
    center_affine_preds: Sequence[Optional[float]],
) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for sample, peak_affine_pred, center_affine_pred in zip(samples, peak_affine_preds, center_affine_preds):
        rows.append(
            {
                "split": split_name,
                "row_id": sample["row_id"],
                "session_id": sample["session_id"],
                "session_name": sample["session_id"].split("/")[-1],
                "bag_id": sample["bag_id"],
                "frame_index": int(sample["frame_index"]),
                "relative_time_s": float(sample["relative_time_s"]),
                "target_slosh_height_mm": float(sample["y"]),
                "pred_train_mean_baseline": float(train_mean),
                "baseline_peak_rel_mm_v2": sample["baseline_b0"],
                "pred_peak_rel_mm_v2_affine": peak_affine_pred,
                "baseline_center_rel_mm_v2": sample["baseline_b1"],
                "pred_center_rel_mm_v2_affine": center_affine_pred,
                "pred_b2_mlp": None,
                "pred_b2_tcn": None,
            }
        )
    return rows


def main() -> int:
    try:
        args = parse_args()
        set_seed(int(args.seed))
        device = choose_device(args.device)

        train_names = parse_name_list(args.train_sessions, "train_sessions")
        val_names = parse_name_list(args.val_sessions, "val_sessions")
        test_names = parse_name_list(args.test_sessions, "test_sessions")
        validate_split_names(train_names, val_names, test_names)
        model_names = parse_model_list(args.models)

        debug_root = Path(args.debug_root).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)
        sessions = collect_debug_sessions(debug_root)

        train_rows, train_meta = pick_rows_for_sessions(sessions, train_names)
        val_rows, val_meta = pick_rows_for_sessions(sessions, val_names)
        test_rows, test_meta = pick_rows_for_sessions(sessions, test_names)

        train_samples, train_skip_counts = build_window_samples_from_columns(
            rows=train_rows,
            target_column="slosh_height_mm",
            primary_baseline_column="peak_rel_mm_v2",
            proxy_baseline_column="center_rel_mm_v2",
            history_frames=int(args.history_frames),
            stride=int(args.stride),
            max_window_gap_sec=float(args.max_window_gap_sec),
            sequence_key="session_id",
        )
        val_samples, val_skip_counts = build_window_samples_from_columns(
            rows=val_rows,
            target_column="slosh_height_mm",
            primary_baseline_column="peak_rel_mm_v2",
            proxy_baseline_column="center_rel_mm_v2",
            history_frames=int(args.history_frames),
            stride=int(args.stride),
            max_window_gap_sec=float(args.max_window_gap_sec),
            sequence_key="session_id",
        )
        test_samples, test_skip_counts = build_window_samples_from_columns(
            rows=test_rows,
            target_column="slosh_height_mm",
            primary_baseline_column="peak_rel_mm_v2",
            proxy_baseline_column="center_rel_mm_v2",
            history_frames=int(args.history_frames),
            stride=int(args.stride),
            max_window_gap_sec=float(args.max_window_gap_sec),
            sequence_key="session_id",
        )

        if not train_samples:
            raise RuntimeError("no train samples after window building")
        if not val_samples:
            raise RuntimeError("no val samples after window building")
        if not test_samples:
            raise RuntimeError("no test samples after window building")

        train_targets = [float(sample["y"]) for sample in train_samples]
        val_targets = [float(sample["y"]) for sample in val_samples]
        test_targets = [float(sample["y"]) for sample in test_samples]

        train_peak = [sample["baseline_b0"] for sample in train_samples]
        val_peak = [sample["baseline_b0"] for sample in val_samples]
        test_peak = [sample["baseline_b0"] for sample in test_samples]
        train_center = [sample["baseline_b1"] for sample in train_samples]
        val_center = [sample["baseline_b1"] for sample in val_samples]
        test_center = [sample["baseline_b1"] for sample in test_samples]

        train_mean = float(np.mean(np.asarray(train_targets, dtype=np.float64)))
        mean_val = [train_mean for _ in val_targets]
        mean_test = [train_mean for _ in test_targets]
        peak_affine = fit_affine_baseline(train_targets, train_peak)
        center_affine = fit_affine_baseline(train_targets, train_center)
        peak_affine_val = apply_affine_baseline(peak_affine, val_peak)
        peak_affine_test = apply_affine_baseline(peak_affine, test_peak)
        center_affine_val = apply_affine_baseline(center_affine, val_center)
        center_affine_test = apply_affine_baseline(center_affine, test_center)

        baseline_metrics = {
            "val_train_mean": collect_metrics(val_targets, mean_val),
            "test_train_mean": collect_metrics(test_targets, mean_test),
            "val_peak_rel_mm_v2": collect_metrics(val_targets, val_peak),
            "test_peak_rel_mm_v2": collect_metrics(test_targets, test_peak),
            "val_center_rel_mm_v2": collect_metrics(val_targets, val_center),
            "test_center_rel_mm_v2": collect_metrics(test_targets, test_center),
            "val_peak_rel_mm_v2_affine": collect_metrics(val_targets, peak_affine_val),
            "test_peak_rel_mm_v2_affine": collect_metrics(test_targets, peak_affine_test),
            "val_center_rel_mm_v2_affine": collect_metrics(val_targets, center_affine_val),
            "test_center_rel_mm_v2_affine": collect_metrics(test_targets, center_affine_test),
        }

        predictions_rows = {
            "val": build_prediction_rows("val", val_samples, train_mean, peak_affine_val, center_affine_val),
            "test": build_prediction_rows("test", test_samples, train_mean, peak_affine_test, center_affine_test),
        }

        model_summaries: Dict[str, Dict[str, object]] = {}
        for model_name in model_names:
            model, metadata, history_rows = train_model(
                model_name=model_name,
                train_samples=train_samples,
                val_samples=val_samples,
                args=args,
                device=device,
            )
            val_preds = predict_with_training_metadata(model, model_name, metadata, val_samples, device=device)
            test_preds = predict_with_training_metadata(model, model_name, metadata, test_samples, device=device)
            train_preds = predict_with_training_metadata(model, model_name, metadata, train_samples, device=device)

            checkpoint_path = out_dir / f"FSL_{model_name}_pseudolabel.pt"
            history_csv_path = out_dir / f"FSL_{model_name}_history.csv"
            save_checkpoint(
                checkpoint_path,
                metadata=metadata,
                model_state_dict={key: value.detach().cpu().clone() for key, value in model.state_dict().items()},
                args=args,
                debug_root=debug_root,
            )
            write_history_csv(history_csv_path, history_rows)

            model_summaries[model_name] = {
                "checkpoint_path": str(checkpoint_path),
                "history_csv": str(history_csv_path),
                "best_epoch": int(metadata["best_epoch"]),
                "best_val_mae": float(metadata["best_val_mae"]),
                "train_metrics": regression_metrics(train_targets, train_preds),
                "val_metrics": regression_metrics(val_targets, val_preds),
                "test_metrics": regression_metrics(test_targets, test_preds),
            }

            pred_key = f"pred_{model_name}"
            for row, pred in zip(predictions_rows["val"], val_preds):
                row[pred_key] = float(pred)
            for row, pred in zip(predictions_rows["test"], test_preds):
                row[pred_key] = float(pred)

        predictions_csv = out_dir / "FSL_pseudolabel_predictions.csv"
        write_predictions_csv(predictions_csv, predictions_rows["val"] + predictions_rows["test"])

        summary = {
            "experiment_type": "bag_split_pseudolabel_pretrain",
            "debug_root": str(debug_root),
            "target_column": "slosh_height_mm",
            "train_sessions": train_meta,
            "val_sessions": val_meta,
            "test_sessions": test_meta,
            "history_frames": int(args.history_frames),
            "stride": int(args.stride),
            "max_window_gap_sec": float(args.max_window_gap_sec),
            "device": str(device),
            "models": model_names,
            "num_samples": {
                "train": len(train_samples),
                "val": len(val_samples),
                "test": len(test_samples),
            },
            "skip_counts": {
                "train": train_skip_counts,
                "val": val_skip_counts,
                "test": test_skip_counts,
            },
            "baseline_metrics": baseline_metrics,
            "model_summaries": model_summaries,
            "predictions_csv": str(predictions_csv),
        }
        summary_json = out_dir / "FSL_pseudolabel_summary.json"
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] predictions csv: {predictions_csv}")
        print(f"[OK] train sessions: {[meta['session_name'] for meta in train_meta]}")
        print(f"[OK] val sessions: {[meta['session_name'] for meta in val_meta]}")
        print(f"[OK] test sessions: {[meta['session_name'] for meta in test_meta]}")
        print(f"[OK] train samples={len(train_samples)} val samples={len(val_samples)} test samples={len(test_samples)}")
        print(f"[OK] baseline test mean mae={baseline_metrics['test_train_mean']['mae']}")
        print(f"[OK] baseline test peak affine mae={baseline_metrics['test_peak_rel_mm_v2_affine']['mae']}")
        print(f"[OK] baseline test center affine mae={baseline_metrics['test_center_rel_mm_v2_affine']['mae']}")
        show_first = max(0, int(args.show_first))
        for model_name, model_summary in model_summaries.items():
            print(
                f"[OK] {model_name}: val_mae={model_summary['val_metrics']['mae']:.6f} "
                f"test_mae={model_summary['test_metrics']['mae']:.6f} "
                f"test_corr={model_summary['test_metrics']['corr']}"
            )
        for row in predictions_rows["test"][:show_first]:
            print(
                "[OK] test row={row_id} target={target_slosh_height_mm:.6f} "
                "peak_raw={baseline_peak_rel_mm_v2} center_raw={baseline_center_rel_mm_v2} "
                "mlp={pred_b2_mlp} tcn={pred_b2_tcn}".format(**row)
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
