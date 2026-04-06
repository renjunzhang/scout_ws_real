#!/usr/bin/env python3
"""Export raw rectified ROI images from cached full-photo frames and a manifest."""

import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from SL_build_supervised_manifest import write_manifest_csv
from SL_supervised_common import read_csv_rows
from extract_liquid_height_from_bag import rectify_roi_and_calibration
from extract_liquid_height_v2_from_bag import load_v2_calibration


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Export raw rectified ROI images from manifest rows that already point to cached full-photo frames. "
            "This keeps the existing SL manifest/split structure and adds raw_rectified_roi_path."
        )
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to an existing SL_supervised_manifest.csv.")
    parser.add_argument("--out-dir", required=True, help="Output directory for raw ROI cache and updated manifest.")
    parser.add_argument(
        "--output-manifest-name",
        default="SL_supervised_manifest_raw_roi.csv",
        help="Filename for the updated manifest written inside out-dir.",
    )
    parser.add_argument(
        "--image-format",
        choices=("png", "jpg"),
        default="png",
        help="Image format for exported raw ROI images. Default: png.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Rebuild existing raw ROI images instead of reusing them.",
    )
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N exported rows. Default: 5.")
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
        return float(text)
    except (TypeError, ValueError):
        return None


def session_path_from_id(session_id: str) -> Path:
    parts = [part for part in str(session_id).strip().split("/") if part]
    if not parts:
        return Path("unknown_session")
    return Path(*parts)


def export_one_row(
    row: Dict[str, str],
    raw_root: Path,
    calibration_cache: Dict[str, object],
    image_format: str,
    overwrite: bool,
    skip_counts: Counter,
) -> Dict[str, str]:
    updated = dict(row)
    photo_raw = str(row.get("photo_path", "")).strip()
    calibration_raw = str(row.get("calibration_path", "")).strip()
    session_id = str(row.get("session_id", "")).strip()
    frame_index = int(float(row.get("frame_index", -1)))

    if photo_raw == "":
        skip_counts["missing_photo_path"] += 1
        updated["raw_rectified_roi_path"] = ""
        updated["raw_rectified_zero_y_px"] = ""
        updated["raw_rectified_rotation_deg"] = ""
        return updated
    if calibration_raw == "":
        skip_counts["missing_calibration_path"] += 1
        updated["raw_rectified_roi_path"] = ""
        updated["raw_rectified_zero_y_px"] = ""
        updated["raw_rectified_rotation_deg"] = ""
        return updated

    photo_path = Path(photo_raw).expanduser()
    if not photo_path.exists():
        skip_counts["photo_not_found"] += 1
        updated["raw_rectified_roi_path"] = ""
        updated["raw_rectified_zero_y_px"] = ""
        updated["raw_rectified_rotation_deg"] = ""
        return updated

    calibration_key = str(Path(calibration_raw).expanduser().resolve())
    calibration_v2 = calibration_cache.get(calibration_key)
    if calibration_v2 is None:
        calibration_v2 = load_v2_calibration(Path(calibration_key))
        calibration_cache[calibration_key] = calibration_v2

    legacy = calibration_v2.legacy
    session_dir = raw_root / session_path_from_id(session_id)
    ensure_dir(session_dir)
    export_path = session_dir / f"frame_{frame_index:06d}.{image_format}"

    if overwrite or not export_path.exists():
        photo_bgr = cv2.imread(str(photo_path), cv2.IMREAD_COLOR)
        if photo_bgr is None:
            skip_counts["photo_load_error"] += 1
            updated["raw_rectified_roi_path"] = ""
            updated["raw_rectified_zero_y_px"] = ""
            updated["raw_rectified_rotation_deg"] = ""
            return updated

        y0 = int(legacy.roi_y)
        y1 = int(legacy.roi_y + legacy.roi_h)
        x0 = int(legacy.roi_x)
        x1 = int(legacy.roi_x + legacy.roi_w)
        roi_bgr = photo_bgr[y0:y1, x0:x1]
        if roi_bgr.shape[0] != int(legacy.roi_h) or roi_bgr.shape[1] != int(legacy.roi_w):
            skip_counts["roi_crop_mismatch"] += 1
            updated["raw_rectified_roi_path"] = ""
            updated["raw_rectified_zero_y_px"] = ""
            updated["raw_rectified_rotation_deg"] = ""
            return updated

        rectified_roi_bgr, rectified_calibration = rectify_roi_and_calibration(roi_bgr, legacy)
        ok = cv2.imwrite(str(export_path), rectified_roi_bgr)
        if not ok:
            skip_counts["image_write_error"] += 1
            updated["raw_rectified_roi_path"] = ""
            updated["raw_rectified_zero_y_px"] = ""
            updated["raw_rectified_rotation_deg"] = ""
            return updated
        skip_counts["exported"] += 1
    else:
        dummy_roi = np.full((int(legacy.roi_h), int(legacy.roi_w), 3), 255, dtype=np.uint8)
        rectified_calibration = rectify_roi_and_calibration(dummy_roi, legacy)[1]
        skip_counts["reused"] += 1

    updated["raw_rectified_roi_path"] = str(export_path)
    updated["raw_rectified_zero_y_px"] = (
        "" if finite_float(calibration_v2.zero_y_rect) is None else f"{float(calibration_v2.zero_y_rect):.6f}"
    )
    updated["raw_rectified_rotation_deg"] = (
        ""
        if finite_float(rectified_calibration.rotation_deg) is None
        else f"{float(rectified_calibration.rotation_deg):.6f}"
    )
    return updated


def write_metadata_json(
    path: Path,
    manifest_csv: Path,
    output_manifest_csv: Path,
    raw_root: Path,
    rows: Sequence[Dict[str, str]],
    skip_counts: Counter,
):
    session_counts = Counter(str(row.get("session_id", "")).strip() for row in rows)
    payload = {
        "source_manifest_csv": str(manifest_csv),
        "output_manifest_csv": str(output_manifest_csv),
        "raw_rectified_roi_root": str(raw_root),
        "num_rows": len(rows),
        "num_sessions": len(session_counts),
        "session_row_counts": dict(session_counts),
        "skip_counts": dict(skip_counts),
    }
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def print_first_rows(rows: Sequence[Dict[str, str]], limit: int):
    if limit <= 0:
        return
    print("[INFO] first exported rows:")
    for row in rows[:limit]:
        print(
            "  "
            + json.dumps(
                {
                    "row_id": row.get("row_id", ""),
                    "bag_id": row.get("bag_id", ""),
                    "frame_index": row.get("frame_index", ""),
                    "raw_rectified_roi_path": row.get("raw_rectified_roi_path", ""),
                },
                ensure_ascii=False,
            )
        )


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        output_manifest_csv = out_dir / str(args.output_manifest_name)
        metadata_json = out_dir / "SL_raw_rectified_roi_metadata.json"
        raw_root = out_dir / "raw_rectified_roi"

        ensure_dir(out_dir)
        ensure_dir(raw_root)

        rows = read_csv_rows(manifest_csv)
        calibration_cache: Dict[str, object] = {}
        skip_counts: Counter = Counter()
        updated_rows: List[Dict[str, str]] = []

        for row in rows:
            updated_rows.append(
                export_one_row(
                    row=row,
                    raw_root=raw_root,
                    calibration_cache=calibration_cache,
                    image_format=str(args.image_format),
                    overwrite=bool(args.overwrite),
                    skip_counts=skip_counts,
                )
            )

        write_manifest_csv(output_manifest_csv, updated_rows)
        write_metadata_json(
            metadata_json,
            manifest_csv=manifest_csv,
            output_manifest_csv=output_manifest_csv,
            raw_root=raw_root,
            rows=updated_rows,
            skip_counts=skip_counts,
        )

        print(f"[OK] raw ROI manifest: {output_manifest_csv}")
        print(f"[OK] metadata json: {metadata_json}")
        print(
            "[OK] raw ROI export "
            f"rows={len(updated_rows)} exported={skip_counts.get('exported', 0)} reused={skip_counts.get('reused', 0)} "
            f"skips={dict(skip_counts)}"
        )
        print_first_rows(updated_rows, int(args.show_first))
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
