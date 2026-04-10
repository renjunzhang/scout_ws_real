#!/usr/bin/env python3
"""Build a unified supervised-learning manifest from debug sessions."""

import argparse
import csv
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


STANDARD_FIELDNAMES = [
    "row_id",
    "session_id",
    "session_name",
    "date_id",
    "bag_id",
    "bag_path",
    "debug_dir",
    "debug_session_csv",
    "debug_session_json",
    "human_labels_csv",
    "calibration_path",
    "config_path",
    "image_topic",
    "frame_index",
    "stamp",
    "relative_time_s",
    "photo_path",
    "roi_debug_path",
    "human_peak_y_rect_px",
    "human_center_y_rect_px",
    "human_peak_mm",
    "human_height_mm",
    "has_human_peak_label",
    "has_human_center_label",
    "has_any_human_label",
    "label_quality",
    "label_valid",
    "label_skip_reason",
    "peak_x_rect",
    "peak_y_rect",
    "center_x_rect",
    "center_y_rect",
    "peak_rel_mm_v2",
    "center_rel_mm_v2",
    "peak_rel_mm_signed_v2",
    "center_rel_mm_signed_v2",
    "slosh_height_mm",
    "slosh_pred_mm",
    "valid_v2",
    "accept_for_peak_report_v2",
    "confidence_v2",
    "peak_source_v2",
    "fit_rms_px_v2",
    "peak_local_rms_px_v2",
    "left_band_support_ratio_v2",
    "right_band_support_ratio_v2",
    "cmd_vel_linear_x",
    "cmd_vel_angular_z",
    "imu_ax",
    "imu_ay",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Merge one or more debug-session directories into a single supervised-learning "
            "manifest for the SL pipeline."
        )
    )
    parser.add_argument(
        "--debug-dir",
        action="append",
        default=[],
        help="Path to one debug-session directory. Can be passed multiple times.",
    )
    parser.add_argument(
        "--debug-root",
        action="append",
        default=[],
        help=(
            "Root directory searched recursively for debug sessions containing debug_session.csv. "
            "Can be passed multiple times."
        ),
    )
    parser.add_argument(
        "--session-json",
        action="append",
        default=[],
        help="Direct path to debug_session.json. Can be passed multiple times.",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="Output directory for the manifest CSV/JSON.",
    )
    parser.add_argument(
        "--label-mode",
        choices=("all", "any", "peak", "center"),
        default="all",
        help=(
            "Row filter. all=keep all rows, any=rows with any human label, "
            "peak=rows with any human peak label (mm or y_rect), "
            "center=rows with any human center label (mm or y_rect)."
        ),
    )
    parser.add_argument(
        "--show-first",
        type=int,
        default=5,
        help="Print the first N manifest rows as a smoke summary. Default: 5.",
    )
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
    return value if math.isfinite(value) else None


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_json(path: Path) -> Dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def normalize_debug_dir(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved.is_file():
        return resolved.parent
    return resolved


def discover_debug_dirs(args) -> List[Path]:
    discovered: List[Path] = []
    seen = set()

    def add_debug_dir(path: Path):
        debug_dir = normalize_debug_dir(path)
        marker = str(debug_dir)
        if marker in seen:
            return
        csv_path = debug_dir / "debug_session.csv"
        json_path = debug_dir / "debug_session.json"
        if not csv_path.exists() and not json_path.exists():
            raise RuntimeError(f"not a debug session directory: {debug_dir}")
        seen.add(marker)
        discovered.append(debug_dir)

    for raw in args.debug_dir:
        add_debug_dir(Path(raw))
    for raw in args.session_json:
        add_debug_dir(Path(raw).expanduser().resolve().parent)
    for raw in args.debug_root:
        root = Path(raw).expanduser().resolve()
        if not root.exists():
            raise FileNotFoundError(str(root))
        for csv_path in sorted(root.rglob("debug_session.csv")):
            add_debug_dir(csv_path.parent)

    if not discovered:
        raise RuntimeError("no debug sessions found; pass --debug-dir, --debug-root, or --session-json")
    return discovered


def load_human_label_overrides(path: Path) -> Dict[int, Dict[str, Optional[float]]]:
    if not path.exists():
        return {}
    overrides: Dict[int, Dict[str, Optional[float]]] = {}
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            frame_index = int(float(row["frame_index"]))
            overrides[frame_index] = {
                "human_peak_y_rect_px": finite_float(row.get("human_peak_y_rect_px")),
                "human_center_y_rect_px": finite_float(row.get("human_center_y_rect_px")),
                "human_height_mm": finite_float(row.get("human_height_mm")),
                "human_peak_mm": finite_float(row.get("human_peak_mm")),
            }
    return overrides


def infer_date_id(debug_dir: Path, bag_path: Path) -> str:
    parent_name = debug_dir.parent.name.strip()
    if re.fullmatch(r"\d{4}([_-].*)?", parent_name):
        return parent_name
    bag_stem = bag_path.stem.strip()
    match = re.search(r"(20\d{6})", bag_stem)
    if match:
        return match.group(1)
    return parent_name


def bool_text(value: bool) -> str:
    return "1" if bool(value) else "0"


def row_passes_label_mode(row: Dict[str, str], label_mode: str) -> bool:
    has_peak = bool(row.get("has_human_peak_label") == "1")
    has_center = bool(row.get("has_human_center_label") == "1")
    if label_mode == "all":
        return True
    if label_mode == "any":
        return has_peak or has_center
    if label_mode == "peak":
        return has_peak
    if label_mode == "center":
        return has_center
    raise RuntimeError(f"unsupported label mode: {label_mode}")


def session_rows_to_manifest(debug_dir: Path) -> Tuple[List[Dict[str, str]], Dict[str, object]]:
    debug_dir = normalize_debug_dir(debug_dir)
    session_csv = debug_dir / "debug_session.csv"
    session_json = debug_dir / "debug_session.json"
    human_labels_csv = debug_dir / "human_labels.csv"

    session_meta = load_json(session_json)
    rows = read_csv_rows(session_csv)
    label_overrides = load_human_label_overrides(human_labels_csv)

    bag_raw = str(session_meta.get("bag", "")).strip()
    bag_path = Path(bag_raw).expanduser() if bag_raw else Path()
    bag_id = bag_path.stem if bag_raw else debug_dir.name
    date_id = infer_date_id(debug_dir, bag_path) if bag_raw else debug_dir.parent.name
    session_name = debug_dir.name
    session_id = f"{date_id}/{session_name}" if date_id else session_name

    manifest_rows: List[Dict[str, str]] = []
    label_counts = Counter()

    for raw in rows:
        frame_index = int(float(raw["frame_index"]))
        override = label_overrides.get(frame_index, {})
        human_peak_y_rect_px = override.get("human_peak_y_rect_px")
        if human_peak_y_rect_px is None:
            human_peak_y_rect_px = finite_float(raw.get("human_peak_y_rect_px"))
        human_center_y_rect_px = override.get("human_center_y_rect_px")
        if human_center_y_rect_px is None:
            human_center_y_rect_px = finite_float(raw.get("human_center_y_rect_px"))
        human_peak_mm = override.get("human_peak_mm")
        if human_peak_mm is None:
            human_peak_mm = finite_float(raw.get("human_peak_mm"))
        human_height_mm = override.get("human_height_mm")
        if human_height_mm is None:
            human_height_mm = finite_float(raw.get("human_height_mm"))

        has_peak = human_peak_mm is not None or human_peak_y_rect_px is not None
        has_center = human_height_mm is not None or human_center_y_rect_px is not None
        has_any = has_peak or has_center

        if has_peak:
            label_counts["peak"] += 1
        if has_center:
            label_counts["center"] += 1
        if has_any:
            label_counts["any"] += 1

        row: Dict[str, str] = {
            "row_id": f"{session_id}:{frame_index}",
            "session_id": session_id,
            "session_name": session_name,
            "date_id": str(date_id),
            "bag_id": str(bag_id),
            "bag_path": str(bag_path) if bag_raw else "",
            "debug_dir": str(debug_dir),
            "debug_session_csv": str(session_csv),
            "debug_session_json": str(session_json),
            "human_labels_csv": str(human_labels_csv),
            "calibration_path": str(session_meta.get("calibration", "")),
            "config_path": str(session_meta.get("config", "")),
            "image_topic": str(session_meta.get("image_topic", "")),
            "frame_index": str(frame_index),
            "stamp": str(raw.get("stamp", "")),
            "relative_time_s": str(raw.get("relative_time_s", "")),
            "photo_path": str(raw.get("photo_path", "")),
            "roi_debug_path": str(raw.get("roi_debug_path", "")),
            "human_peak_y_rect_px": "" if human_peak_y_rect_px is None else f"{human_peak_y_rect_px:.3f}",
            "human_center_y_rect_px": "" if human_center_y_rect_px is None else f"{human_center_y_rect_px:.3f}",
            "human_peak_mm": "" if human_peak_mm is None else f"{human_peak_mm:.6f}",
            "human_height_mm": "" if human_height_mm is None else f"{human_height_mm:.6f}",
            "has_human_peak_label": bool_text(has_peak),
            "has_human_center_label": bool_text(has_center),
            "has_any_human_label": bool_text(has_any),
            "label_quality": str(raw.get("label_quality", "")),
            "label_valid": str(raw.get("label_valid", "")),
            "label_skip_reason": str(raw.get("label_skip_reason", "")),
        }

        for key in (
            "peak_x_rect",
            "peak_y_rect",
            "center_x_rect",
            "center_y_rect",
            "peak_rel_mm_v2",
            "center_rel_mm_v2",
            "peak_rel_mm_signed_v2",
            "center_rel_mm_signed_v2",
            "slosh_height_mm",
            "slosh_pred_mm",
            "valid_v2",
            "accept_for_peak_report_v2",
            "confidence_v2",
            "peak_source_v2",
            "fit_rms_px_v2",
            "peak_local_rms_px_v2",
            "left_band_support_ratio_v2",
            "right_band_support_ratio_v2",
            "cmd_vel_linear_x",
            "cmd_vel_angular_z",
            "imu_ax",
            "imu_ay",
        ):
            row[key] = str(raw.get(key, ""))

        for key, value in raw.items():
            if key not in row:
                row[key] = str(value)

        manifest_rows.append(row)

    session_summary = {
        "session_id": session_id,
        "session_name": session_name,
        "date_id": str(date_id),
        "bag_id": str(bag_id),
        "bag_path": str(bag_path) if bag_raw else "",
        "debug_dir": str(debug_dir),
        "num_rows": len(manifest_rows),
        "label_counts": dict(label_counts),
    }
    return manifest_rows, session_summary


def ordered_fieldnames(rows: Sequence[Dict[str, str]]) -> List[str]:
    known = list(STANDARD_FIELDNAMES)
    extra_keys = set()
    for row in rows:
        extra_keys.update(row.keys())
    extras = sorted(key for key in extra_keys if key not in STANDARD_FIELDNAMES)
    return known + extras


def write_manifest_csv(path: Path, rows: Sequence[Dict[str, str]]):
    fieldnames = ordered_fieldnames(rows)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main() -> int:
    try:
        args = parse_args()
        debug_dirs = discover_debug_dirs(args)

        all_rows: List[Dict[str, str]] = []
        session_summaries: List[Dict[str, object]] = []
        global_counts = Counter()

        for debug_dir in debug_dirs:
            session_rows, session_summary = session_rows_to_manifest(debug_dir)
            session_summaries.append(session_summary)
            for row in session_rows:
                if not row_passes_label_mode(row, args.label_mode):
                    continue
                all_rows.append(row)
                if row.get("has_human_peak_label") == "1":
                    global_counts["peak"] += 1
                if row.get("has_human_center_label") == "1":
                    global_counts["center"] += 1
                if row.get("has_any_human_label") == "1":
                    global_counts["any"] += 1

        if not all_rows:
            raise RuntimeError(f"no rows matched label mode: {args.label_mode}")

        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)
        manifest_csv = out_dir / "SL_supervised_manifest.csv"
        metadata_json = out_dir / "SL_supervised_manifest_metadata.json"

        write_manifest_csv(manifest_csv, all_rows)

        metadata = {
            "out_dir": str(out_dir),
            "manifest_csv": str(manifest_csv),
            "label_mode": str(args.label_mode),
            "num_sessions": len(session_summaries),
            "num_rows": len(all_rows),
            "global_label_counts": dict(global_counts),
            "sessions": session_summaries,
        }
        with metadata_json.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

        print(f"[OK] manifest csv: {manifest_csv}")
        print(f"[OK] metadata json: {metadata_json}")
        print(f"[OK] sessions: {len(session_summaries)}")
        print(f"[OK] rows: {len(all_rows)}")
        print(f"[OK] label counts: {dict(global_counts)}")

        show_first = max(0, int(args.show_first))
        for row in all_rows[:show_first]:
            print(
                "[OK] row={row_id} bag={bag_id} frame={frame_index} peak={human_peak_mm} center={human_height_mm}".format(
                    **row
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
