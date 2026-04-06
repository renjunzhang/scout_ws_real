#!/usr/bin/env python3
"""Export a non-destructive center-half crop from existing raw rectified ROI images."""

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from PIL import Image

from SL_build_supervised_manifest import write_manifest_csv
from SL_supervised_common import read_csv_rows


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Create a derived ROI dataset by cropping away the top 1/4 and bottom 1/4 "
            "from existing raw rectified ROI images. This is non-destructive and writes "
            "a new manifest with center_half_roi_path."
        )
    )
    parser.add_argument("--manifest-csv", required=True, help="Path to SL_supervised_manifest_raw_roi.csv.")
    parser.add_argument("--out-dir", required=True, help="Output directory for cropped images and manifest.")
    parser.add_argument(
        "--input-image-column",
        default="raw_rectified_roi_path",
        help="Manifest image column to crop. Default: raw_rectified_roi_path.",
    )
    parser.add_argument(
        "--input-zero-y-column",
        default="raw_rectified_zero_y_px",
        help="Manifest zero-y column in rectified ROI coordinates. Default: raw_rectified_zero_y_px.",
    )
    parser.add_argument(
        "--output-image-column",
        default="center_half_roi_path",
        help="Output manifest image column. Default: center_half_roi_path.",
    )
    parser.add_argument(
        "--output-zero-y-column",
        default="center_half_zero_y_px",
        help="Output manifest zero-y column after crop. Default: center_half_zero_y_px.",
    )
    parser.add_argument(
        "--output-manifest-name",
        default="SL_supervised_manifest_center_half_roi.csv",
        help="Filename for updated manifest written inside out-dir.",
    )
    parser.add_argument(
        "--image-format",
        choices=("png", "jpg"),
        default="png",
        help="Image format for derived crop images. Default: png.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Rebuild existing cropped images.")
    parser.add_argument("--show-first", type=int, default=5, help="Print the first N rows. Default: 5.")
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
    return Path(*parts) if parts else Path("unknown_session")


def export_one_row(
    row: Dict[str, str],
    crop_root: Path,
    input_image_column: str,
    input_zero_y_column: str,
    output_image_column: str,
    output_zero_y_column: str,
    image_format: str,
    overwrite: bool,
    skip_counts: Counter,
) -> Dict[str, str]:
    updated = dict(row)
    src_raw = str(row.get(input_image_column, "")).strip()
    session_id = str(row.get("session_id", "")).strip()
    frame_index = int(float(row.get("frame_index", -1) or -1))

    if src_raw == "":
        skip_counts["missing_input_image"] += 1
        updated[output_image_column] = ""
        updated[output_zero_y_column] = ""
        updated["center_half_crop_y0_px"] = ""
        updated["center_half_crop_h_px"] = ""
        return updated

    src = Path(src_raw).expanduser()
    if not src.exists():
        skip_counts["input_image_not_found"] += 1
        updated[output_image_column] = ""
        updated[output_zero_y_column] = ""
        updated["center_half_crop_y0_px"] = ""
        updated["center_half_crop_h_px"] = ""
        return updated

    with Image.open(src) as img:
        width, height = img.size
        crop_top = int(round(float(height) * 0.25))
        crop_bottom = int(round(float(height) * 0.25))
        y0 = max(0, min(height - 1, crop_top))
        y1 = max(y0 + 1, min(height, height - crop_bottom))
        cropped_h = int(y1 - y0)

        session_dir = crop_root / session_path_from_id(session_id)
        ensure_dir(session_dir)
        dst = session_dir / f"frame_{frame_index:06d}.{image_format}"

        if overwrite or not dst.exists():
            cropped = img.crop((0, y0, width, y1))
            cropped.save(dst)
            skip_counts["exported"] += 1
        else:
            skip_counts["reused"] += 1

    zero_y = finite_float(row.get(input_zero_y_column))
    adjusted_zero_y = None if zero_y is None else float(zero_y) - float(y0)
    updated[output_image_column] = str(dst)
    updated[output_zero_y_column] = "" if adjusted_zero_y is None else f"{adjusted_zero_y:.6f}"
    updated["center_half_crop_y0_px"] = str(int(y0))
    updated["center_half_crop_h_px"] = str(int(cropped_h))
    return updated


def write_metadata_json(
    path: Path,
    source_manifest_csv: Path,
    output_manifest_csv: Path,
    crop_root: Path,
    rows: Sequence[Dict[str, str]],
    output_image_column: str,
    output_zero_y_column: str,
    skip_counts: Counter,
):
    payload = {
        "source_manifest_csv": str(source_manifest_csv),
        "output_manifest_csv": str(output_manifest_csv),
        "derived_crop_root": str(crop_root),
        "output_image_column": output_image_column,
        "output_zero_y_column": output_zero_y_column,
        "num_rows": len(rows),
        "skip_counts": dict(skip_counts),
        "crop_rule": {
            "x0": 0,
            "w": "full_width",
            "y0": "round(height * 0.25)",
            "h": "height - round(height * 0.25) - round(height * 0.25)",
        },
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def print_first_rows(rows: Sequence[Dict[str, str]], output_image_column: str, limit: int):
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
                    output_image_column: row.get(output_image_column, ""),
                    "center_half_crop_y0_px": row.get("center_half_crop_y0_px", ""),
                    "center_half_crop_h_px": row.get("center_half_crop_h_px", ""),
                },
                ensure_ascii=False,
            )
        )


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        crop_root = out_dir / "center_half_roi"
        output_manifest_csv = out_dir / str(args.output_manifest_name)
        metadata_json = out_dir / "SL_center_half_roi_metadata.json"

        ensure_dir(out_dir)
        ensure_dir(crop_root)

        rows = read_csv_rows(manifest_csv)
        skip_counts: Counter = Counter()
        updated_rows: List[Dict[str, str]] = []

        for row in rows:
            updated_rows.append(
                export_one_row(
                    row=row,
                    crop_root=crop_root,
                    input_image_column=str(args.input_image_column),
                    input_zero_y_column=str(args.input_zero_y_column),
                    output_image_column=str(args.output_image_column),
                    output_zero_y_column=str(args.output_zero_y_column),
                    image_format=str(args.image_format),
                    overwrite=bool(args.overwrite),
                    skip_counts=skip_counts,
                )
            )

        write_manifest_csv(output_manifest_csv, updated_rows)
        write_metadata_json(
            metadata_json,
            source_manifest_csv=manifest_csv,
            output_manifest_csv=output_manifest_csv,
            crop_root=crop_root,
            rows=updated_rows,
            output_image_column=str(args.output_image_column),
            output_zero_y_column=str(args.output_zero_y_column),
            skip_counts=skip_counts,
        )

        print(f"[OK] center-half manifest: {output_manifest_csv}")
        print(f"[OK] metadata json: {metadata_json}")
        print(
            "[OK] center-half export "
            f"rows={len(updated_rows)} exported={skip_counts.get('exported', 0)} reused={skip_counts.get('reused', 0)} "
            f"skips={dict(skip_counts)}"
        )
        print_first_rows(updated_rows, str(args.output_image_column), int(args.show_first))
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
