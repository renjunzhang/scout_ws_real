#!/usr/bin/env python3
"""Train the minimal SL dynamics-only baselines with PyTorch."""

import argparse
import csv
import json
import random
import sys
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from SL_supervised_common import (
    FEATURE_NAMES,
    build_window_samples,
    feature_stats_from_samples,
    filter_rows_for_split,
    load_json,
    normalize_samples,
    read_csv_rows,
    regression_metrics,
    target_column_for_task,
)


class B2MLP(nn.Module):
    def __init__(self, input_dim: int, hidden_dims: Sequence[int]):
        super().__init__()
        dims = [int(input_dim)] + [int(dim) for dim in hidden_dims] + [1]
        layers: List[nn.Module] = []
        for idx in range(len(dims) - 2):
            layers.append(nn.Linear(dims[idx], dims[idx + 1]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(dims[-2], dims[-1]))
        self.network = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x).squeeze(-1)


class B2TCN(nn.Module):
    def __init__(self, input_channels: int, channels: Sequence[int], kernel_size: int):
        super().__init__()
        if kernel_size <= 0 or kernel_size % 2 == 0:
            raise RuntimeError("tcn kernel size must be a positive odd integer")
        dims = [int(input_channels)] + [int(channel) for channel in channels]
        layers: List[nn.Module] = []
        for idx in range(len(dims) - 1):
            dilation = 2 ** idx
            padding = ((int(kernel_size) - 1) * dilation) // 2
            layers.append(
                nn.Conv1d(
                    in_channels=dims[idx],
                    out_channels=dims[idx + 1],
                    kernel_size=int(kernel_size),
                    dilation=dilation,
                    padding=padding,
                )
            )
            layers.append(nn.ReLU())
        self.encoder = nn.Sequential(*layers)
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.head = nn.Linear(dims[-1], 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.transpose(1, 2)
        x = self.encoder(x)
        x = self.pool(x).squeeze(-1)
        return self.head(x).squeeze(-1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train the SL dynamics-only baselines from a supervised manifest and split file."
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest.csv.")
    parser.add_argument("--split-json", required=True, help="Path to SL_supervised_splits.json.")
    parser.add_argument("--out-dir", required=True, help="Output directory for checkpoint and logs.")
    parser.add_argument("--task", choices=("peak", "center"), default="peak", help="Training task. Default: peak.")
    parser.add_argument(
        "--model",
        choices=("b2_mlp", "b2_tcn"),
        default="b2_mlp",
        help="Model type. Default: b2_mlp.",
    )
    parser.add_argument("--history-frames", type=int, default=30, help="History length K. Default: 30.")
    parser.add_argument("--stride", type=int, default=1, help="Frame stride inside each window. Default: 1.")
    parser.add_argument(
        "--max-window-gap-sec",
        type=float,
        default=0.20,
        help="Reject a window if adjacent frames exceed this gap. Default: 0.20 s.",
    )
    parser.add_argument(
        "--hidden-dims",
        default="128,64",
        help="Comma-separated hidden dimensions for the MLP. Default: 128,64.",
    )
    parser.add_argument(
        "--tcn-channels",
        default="32,64",
        help="Comma-separated channel sizes for the TCN. Default: 32,64.",
    )
    parser.add_argument(
        "--tcn-kernel-size",
        type=int,
        default=3,
        help="Odd kernel size for the TCN. Default: 3.",
    )
    parser.add_argument("--epochs", type=int, default=80, help="Number of training epochs. Default: 80.")
    parser.add_argument("--batch-size", type=int, default=64, help="Mini-batch size. Default: 64.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate. Default: 1e-3.")
    parser.add_argument("--weight-decay", type=float, default=1e-5, help="AdamW weight decay. Default: 1e-5.")
    parser.add_argument("--patience", type=int, default=12, help="Early-stop patience on val MAE. Default: 12.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed. Default: 7.")
    parser.add_argument("--device", default="auto", help="Torch device: auto/cpu/cuda. Default: auto.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N samples per split. Default: 5.")
    return parser.parse_args()


def parse_positive_dims(raw_value: str, field_name: str) -> List[int]:
    dims = [int(part.strip()) for part in str(raw_value).split(",") if part.strip()]
    if not dims:
        raise RuntimeError(f"{field_name} must contain at least one layer")
    if any(dim <= 0 for dim in dims):
        raise RuntimeError(f"{field_name} must be positive integers")
    return dims


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def set_seed(seed: int):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def choose_device(raw_value: str) -> torch.device:
    value = str(raw_value).strip().lower()
    if value in ("", "auto"):
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(value)


def target_stats_from_samples(samples: Sequence[Dict[str, object]]) -> Dict[str, float]:
    targets = np.asarray([float(sample["y"]) for sample in samples], dtype=np.float32)
    mean = float(np.mean(targets))
    std = float(np.std(targets))
    if std < 1e-6:
        std = 1.0
    return {"mean": mean, "std": std}


def samples_to_model_array(samples: Sequence[Dict[str, object]], model_type: str) -> np.ndarray:
    if model_type == "b2_mlp":
        return np.asarray(
            [np.asarray(sample["x"], dtype=np.float32).reshape(-1) for sample in samples],
            dtype=np.float32,
        )
    if model_type == "b2_tcn":
        return np.asarray([np.asarray(sample["x"], dtype=np.float32) for sample in samples], dtype=np.float32)
    raise RuntimeError(f"unsupported model_type: {model_type}")


def samples_to_loader(
    samples: Sequence[Dict[str, object]],
    batch_size: int,
    shuffle: bool,
    target_stats: Dict[str, float],
    model_type: str,
) -> DataLoader:
    x_np = samples_to_model_array(samples, model_type=model_type)
    y_np = np.asarray(
        [(float(sample["y"]) - float(target_stats["mean"])) / float(target_stats["std"]) for sample in samples],
        dtype=np.float32,
    )
    dataset = TensorDataset(torch.from_numpy(x_np), torch.from_numpy(y_np))
    return DataLoader(dataset, batch_size=max(1, int(batch_size)), shuffle=bool(shuffle))


def build_model(
    model_type: str,
    history_frames: int,
    feature_dim: int,
    hidden_dims: Sequence[int],
    tcn_channels: Sequence[int],
    tcn_kernel_size: int,
) -> nn.Module:
    if model_type == "b2_mlp":
        return B2MLP(input_dim=int(history_frames) * int(feature_dim), hidden_dims=hidden_dims)
    if model_type == "b2_tcn":
        return B2TCN(input_channels=int(feature_dim), channels=tcn_channels, kernel_size=int(tcn_kernel_size))
    raise RuntimeError(f"unsupported model_type: {model_type}")


def build_model_from_checkpoint(checkpoint: Dict) -> nn.Module:
    return build_model(
        model_type=str(checkpoint["model_type"]),
        history_frames=int(checkpoint["history_frames"]),
        feature_dim=len(checkpoint["feature_names"]),
        hidden_dims=[int(dim) for dim in checkpoint.get("hidden_dims", [])] or [128, 64],
        tcn_channels=[int(dim) for dim in checkpoint.get("tcn_channels", [])] or [32, 64],
        tcn_kernel_size=int(checkpoint.get("tcn_kernel_size", 3)),
    )


def predict_samples(
    model: nn.Module,
    samples: Sequence[Dict[str, object]],
    device: torch.device,
    target_stats: Dict[str, float],
    model_type: str,
) -> List[float]:
    if not samples:
        return []
    x_np = samples_to_model_array(samples, model_type=model_type)
    x_tensor = torch.from_numpy(x_np).to(device=device)
    model.eval()
    with torch.no_grad():
        pred_norm = model(x_tensor).detach().cpu().numpy()
    return [float(value * float(target_stats["std"]) + float(target_stats["mean"])) for value in pred_norm]


def metrics_from_predictions(samples: Sequence[Dict[str, object]], preds: Sequence[float]) -> Dict[str, float]:
    targets = [float(sample["y"]) for sample in samples]
    metrics = regression_metrics(targets, preds)
    return {key: value for key, value in metrics.items()}


def write_history_csv(path: Path, history_rows: Sequence[Dict[str, object]]):
    fieldnames = ["epoch", "train_loss", "train_mae", "val_mae"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in history_rows:
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

        train_samples, train_skip_counts = build_window_samples(
            train_rows,
            task=args.task,
            history_frames=int(args.history_frames),
            stride=int(args.stride),
            max_window_gap_sec=float(args.max_window_gap_sec),
        )
        val_samples, val_skip_counts = build_window_samples(
            val_rows,
            task=args.task,
            history_frames=int(args.history_frames),
            stride=int(args.stride),
            max_window_gap_sec=float(args.max_window_gap_sec),
        )

        if not train_samples:
            raise RuntimeError("no train samples after window building; add labels or reduce constraints")
        if not val_samples:
            raise RuntimeError("no val samples after window building; add labels or reduce constraints")

        feature_mean, feature_std = feature_stats_from_samples(train_samples)
        train_samples = normalize_samples(train_samples, feature_mean, feature_std)
        val_samples = normalize_samples(val_samples, feature_mean, feature_std)
        target_stats = target_stats_from_samples(train_samples)

        hidden_dims = parse_positive_dims(args.hidden_dims, "hidden_dims")
        tcn_channels = parse_positive_dims(args.tcn_channels, "tcn_channels")
        if int(args.tcn_kernel_size) <= 0 or int(args.tcn_kernel_size) % 2 == 0:
            raise RuntimeError("tcn_kernel_size must be a positive odd integer")

        model = build_model(
            model_type=str(args.model),
            history_frames=int(args.history_frames),
            feature_dim=len(FEATURE_NAMES),
            hidden_dims=hidden_dims,
            tcn_channels=tcn_channels,
            tcn_kernel_size=int(args.tcn_kernel_size),
        ).to(device=device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=float(args.lr), weight_decay=float(args.weight_decay))
        criterion = nn.HuberLoss(delta=1.0)

        train_loader = samples_to_loader(
            train_samples,
            batch_size=int(args.batch_size),
            shuffle=True,
            target_stats=target_stats,
            model_type=str(args.model),
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

            train_preds = predict_samples(
                model,
                train_samples,
                device=device,
                target_stats=target_stats,
                model_type=str(args.model),
            )
            val_preds = predict_samples(
                model,
                val_samples,
                device=device,
                target_stats=target_stats,
                model_type=str(args.model),
            )
            train_metrics = metrics_from_predictions(train_samples, train_preds)
            val_metrics = metrics_from_predictions(val_samples, val_preds)
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
        train_preds = predict_samples(
            model,
            train_samples,
            device=device,
            target_stats=target_stats,
            model_type=str(args.model),
        )
        val_preds = predict_samples(
            model,
            val_samples,
            device=device,
            target_stats=target_stats,
            model_type=str(args.model),
        )
        final_train_metrics = metrics_from_predictions(train_samples, train_preds)
        final_val_metrics = metrics_from_predictions(val_samples, val_preds)

        checkpoint_path = out_dir / f"SL_{args.model}_{args.task}.pt"
        history_csv_path = out_dir / f"SL_{args.model}_{args.task}_history.csv"
        summary_json_path = out_dir / f"SL_{args.model}_{args.task}_summary.json"

        torch.save(
            {
                "model_type": str(args.model),
                "task": str(args.task),
                "target_column": target_column_for_task(args.task),
                "history_frames": int(args.history_frames),
                "stride": int(args.stride),
                "max_window_gap_sec": float(args.max_window_gap_sec),
                "hidden_dims": hidden_dims,
                "tcn_channels": tcn_channels,
                "tcn_kernel_size": int(args.tcn_kernel_size),
                "feature_names": list(FEATURE_NAMES),
                "feature_mean": feature_mean.tolist(),
                "feature_std": feature_std.tolist(),
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
        summary = {
            "checkpoint_path": str(checkpoint_path),
            "history_csv": str(history_csv_path),
            "task": str(args.task),
            "model": str(args.model),
            "device": str(device),
            "feature_names": list(FEATURE_NAMES),
            "hidden_dims": hidden_dims,
            "tcn_channels": tcn_channels,
            "tcn_kernel_size": int(args.tcn_kernel_size),
            "history_frames": int(args.history_frames),
            "stride": int(args.stride),
            "max_window_gap_sec": float(args.max_window_gap_sec),
            "train_skip_counts": train_skip_counts,
            "val_skip_counts": val_skip_counts,
            "num_train_samples": len(train_samples),
            "num_val_samples": len(val_samples),
            "best_epoch": int(best_epoch),
            "best_val_mae": float(best_val_mae),
            "train_metrics": final_train_metrics,
            "val_metrics": final_val_metrics,
        }
        with summary_json_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] checkpoint: {checkpoint_path}")
        print(f"[OK] history csv: {history_csv_path}")
        print(f"[OK] summary json: {summary_json_path}")
        print(f"[OK] train samples: {len(train_samples)} | skip_counts={train_skip_counts}")
        print(f"[OK] val samples: {len(val_samples)} | skip_counts={val_skip_counts}")
        print(f"[OK] best epoch: {best_epoch} | best val mae: {best_val_mae:.6f}")

        show_first = max(0, int(args.show_first))
        for sample, pred in list(zip(val_samples, val_preds))[:show_first]:
            print(
                "[OK] val row={row_id} bag={bag_id} target={target:.6f} pred={pred:.6f}".format(
                    row_id=sample["row_id"],
                    bag_id=sample["bag_id"],
                    target=float(sample["y"]),
                    pred=float(pred),
                )
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
