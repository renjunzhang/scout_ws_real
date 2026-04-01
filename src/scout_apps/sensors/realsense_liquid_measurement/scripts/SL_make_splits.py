#!/usr/bin/env python3
"""Generate bag/date-level train-val-test splits for the SL manifest."""

import argparse
import csv
import json
import math
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Set


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Create supervised-learning train/val/test splits from SL_supervised_manifest.csv "
            "without leaking frames from the same bag/date across splits."
        )
    )
    parser.add_argument(
        "--manifest-csv",
        required=True,
        help="Path to SL_supervised_manifest.csv.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="Output directory. Defaults to <manifest_parent>/splits.",
    )
    parser.add_argument(
        "--group-key",
        choices=("bag_id", "date_id", "session_id"),
        default="bag_id",
        help="Group rows by this key before splitting. Default: bag_id.",
    )
    parser.add_argument(
        "--label-column",
        default="human_peak_mm",
        help="Column used to count labeled rows per split. Default: human_peak_mm.",
    )
    parser.add_argument("--train-ratio", type=float, default=0.70, help="Train split ratio. Default: 0.70.")
    parser.add_argument("--val-ratio", type=float, default=0.15, help="Validation split ratio. Default: 0.15.")
    parser.add_argument("--test-ratio", type=float, default=0.15, help="Test split ratio. Default: 0.15.")
    parser.add_argument("--seed", type=int, default=7, help="Shuffle seed. Default: 7.")
    parser.add_argument(
        "--force-train-groups",
        default="",
        help="Comma-separated group ids forced into train.",
    )
    parser.add_argument(
        "--force-val-groups",
        default="",
        help="Comma-separated group ids forced into val.",
    )
    parser.add_argument(
        "--force-test-groups",
        default="",
        help="Comma-separated group ids forced into test.",
    )
    return parser.parse_args()


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"manifest csv has no rows: {path}")
    return rows


def finite_float(raw_value):
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


def parse_group_list(raw_value: str) -> Set[str]:
    if not raw_value.strip():
        return set()
    return {part.strip() for part in raw_value.split(",") if part.strip()}


def choose_count(total: int, ratio: float, minimum_if_possible: bool) -> int:
    if total <= 0 or ratio <= 0.0:
        return 0
    estimate = int(round(total * ratio))
    if minimum_if_possible and estimate <= 0:
        estimate = 1
    return min(total, estimate)


def summarize_split(rows: Sequence[Dict[str, str]], label_column: str) -> Dict[str, object]:
    groups = sorted({str(row["group_value"]) for row in rows})
    labeled_rows = sum(1 for row in rows if finite_float(row.get(label_column)) is not None)
    return {
        "num_groups": len(groups),
        "num_rows": len(rows),
        "num_labeled_rows": int(labeled_rows),
        "groups": groups,
    }


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        rows = read_csv_rows(manifest_csv)
        if args.group_key not in rows[0]:
            raise RuntimeError(f"missing group key in manifest: {args.group_key}")
        if args.label_column not in rows[0]:
            raise RuntimeError(f"missing label column in manifest: {args.label_column}")

        force_train = parse_group_list(args.force_train_groups)
        force_val = parse_group_list(args.force_val_groups)
        force_test = parse_group_list(args.force_test_groups)
        if (force_train & force_val) or (force_train & force_test) or (force_val & force_test):
            raise RuntimeError("forced train/val/test groups must be disjoint")

        groups_to_rows: Dict[str, List[Dict[str, str]]] = defaultdict(list)
        for raw in rows:
            group_value = str(raw.get(args.group_key, "")).strip()
            if group_value == "":
                raise RuntimeError(f"row missing group key {args.group_key}: {raw.get('row_id', '<no-row-id>')}")
            row = dict(raw)
            row["group_value"] = group_value
            groups_to_rows[group_value].append(row)

        all_groups = sorted(groups_to_rows.keys())
        if float(args.train_ratio) < 0.0 or float(args.val_ratio) < 0.0 or float(args.test_ratio) < 0.0:
            raise RuntimeError("split ratios must be non-negative")
        ratio_sum = float(args.train_ratio) + float(args.val_ratio) + float(args.test_ratio)
        if ratio_sum <= 0.0:
            raise RuntimeError("at least one split ratio must be positive")
        missing_forced = (force_train | force_val | force_test) - set(all_groups)
        if missing_forced:
            raise RuntimeError(f"forced groups not found in manifest: {sorted(missing_forced)}")

        remaining_groups = [group for group in all_groups if group not in force_train and group not in force_val and group not in force_test]
        rng = random.Random(int(args.seed))
        rng.shuffle(remaining_groups)

        total_groups = len(all_groups)
        required_split_slots = 1 + int(float(args.val_ratio) > 0.0) + int(float(args.test_ratio) > 0.0)
        if total_groups < required_split_slots:
            raise RuntimeError(
                f"insufficient groups for requested split ratios: groups={total_groups}, "
                f"required_at_least={required_split_slots}"
            )
        desired_test = choose_count(total_groups, float(args.test_ratio), minimum_if_possible=True)
        desired_val = choose_count(total_groups, float(args.val_ratio), minimum_if_possible=True)

        test_groups = set(force_test)
        val_groups = set(force_val)
        train_groups = set(force_train)

        for group in remaining_groups:
            if len(test_groups) < desired_test:
                test_groups.add(group)
                continue
            if len(val_groups) < desired_val:
                val_groups.add(group)
                continue
            train_groups.add(group)

        if not train_groups:
            if val_groups:
                train_groups.add(val_groups.pop())
            elif test_groups:
                train_groups.add(test_groups.pop())
            else:
                raise RuntimeError("failed to allocate any train groups")

        if not val_groups and len(train_groups) >= 2 and float(args.val_ratio) > 0.0:
            val_groups.add(sorted(train_groups)[-1])
            train_groups.remove(sorted(train_groups)[-1])
        if not test_groups and len(train_groups) >= 2 and float(args.test_ratio) > 0.0:
            test_groups.add(sorted(train_groups)[-1])
            train_groups.remove(sorted(train_groups)[-1])

        if float(args.val_ratio) > 0.0 and not val_groups:
            raise RuntimeError("validation split ended up empty; add more groups or change ratios/forced groups")
        if float(args.test_ratio) > 0.0 and not test_groups:
            raise RuntimeError("test split ended up empty; add more groups or change ratios/forced groups")

        split_order = [("train", train_groups), ("val", val_groups), ("test", test_groups)]
        assigned_rows: Dict[str, List[Dict[str, str]]] = {"train": [], "val": [], "test": []}
        for split_name, split_groups in split_order:
            for group in sorted(split_groups):
                assigned_rows[split_name].extend(groups_to_rows[group])

        out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else (manifest_csv.parent / "splits").resolve()
        ensure_dir(out_dir)
        split_json = out_dir / "SL_supervised_splits.json"
        split_groups_csv = out_dir / "SL_supervised_split_groups.csv"

        split_summary = {
            "manifest_csv": str(manifest_csv),
            "group_key": str(args.group_key),
            "label_column": str(args.label_column),
            "seed": int(args.seed),
            "ratios": {
                "train": float(args.train_ratio),
                "val": float(args.val_ratio),
                "test": float(args.test_ratio),
            },
            "forced_groups": {
                "train": sorted(force_train),
                "val": sorted(force_val),
                "test": sorted(force_test),
            },
            "splits": {
                "train": summarize_split(assigned_rows["train"], args.label_column),
                "val": summarize_split(assigned_rows["val"], args.label_column),
                "test": summarize_split(assigned_rows["test"], args.label_column),
            },
        }
        with split_json.open("w", encoding="utf-8") as handle:
            json.dump(split_summary, handle, ensure_ascii=False, indent=2)

        with split_groups_csv.open("w", newline="", encoding="utf-8") as handle:
            fieldnames = [
                "split",
                "group_value",
                "num_rows",
                "num_labeled_rows",
            ]
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for split_name, split_groups in split_order:
                for group in sorted(split_groups):
                    group_rows = groups_to_rows[group]
                    labeled_rows = sum(1 for row in group_rows if finite_float(row.get(args.label_column)) is not None)
                    writer.writerow(
                        {
                            "split": split_name,
                            "group_value": group,
                            "num_rows": len(group_rows),
                            "num_labeled_rows": int(labeled_rows),
                        }
                    )

        print(f"[OK] split json: {split_json}")
        print(f"[OK] split groups csv: {split_groups_csv}")
        for split_name in ("train", "val", "test"):
            summary = split_summary["splits"][split_name]
            print(
                f"[OK] {split_name}: groups={summary['num_groups']} rows={summary['num_rows']} "
                f"labeled={summary['num_labeled_rows']}"
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
