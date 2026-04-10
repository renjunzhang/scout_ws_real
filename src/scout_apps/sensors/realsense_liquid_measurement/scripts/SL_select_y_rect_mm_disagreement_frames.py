#!/usr/bin/env python3
"""Select frames where human_peak_y_rect_px and human_peak_mm disagree the most."""

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import yaml


DEFAULT_OUT_DIR = "/data/a/realsense_validation_v2/sl_candidate_frames/0401_y_rect_mm_disagreement_v1"


@dataclass
class ReferencePoint:
    y_px: float
    h_mm: float


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Rank frames where human_peak_y_rect_px, after fixed F(y) mapping, disagrees most with "
            "human_peak_mm. Intended for label-consistency review."
        )
    )
    parser.add_argument(
        "--manifest-csv",
        default=(
            "/home/a/scout_ws/src/scout_apps/sensors/realsense_liquid_measurement/"
            "sl_artifacts/SL_human_peak_manifest_0401_all_raw_roi_relabel_refresh_v1/"
            "SL_supervised_manifest_raw_roi_motion_only.csv"
        ),
        help="Manifest with both human_peak_mm and human_peak_y_rect_px.",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help=f"Output directory for selected CSV/summary. Default: {DEFAULT_OUT_DIR}",
    )
    parser.add_argument(
        "--top-k-per-bag",
        type=int,
        default=25,
        help="How many disagreement frames to keep per bag after de-dup. Default: 25.",
    )
    parser.add_argument(
        "--min-frame-gap",
        type=int,
        default=10,
        help="Minimum frame gap between selected rows in the same bag. Default: 10.",
    )
    parser.add_argument(
        "--show-first",
        type=int,
        default=12,
        help="Print first N selected rows. Default: 12.",
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
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_reference_points(calibration_path: Path) -> List[ReferencePoint]:
    with calibration_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    raw_points = data.get("height_mapping", {}).get("reference_points", [])
    if len(raw_points) < 2:
        raise RuntimeError(f"invalid reference_points: {calibration_path}")
    points = [ReferencePoint(y_px=float(item["y_px"]), h_mm=float(item["h_mm"])) for item in raw_points]
    points.sort(key=lambda item: item.y_px, reverse=True)
    return points


def interpolate_h_from_y(first: ReferencePoint, second: ReferencePoint, y_px: float) -> float:
    delta = float(second.y_px - first.y_px)
    if abs(delta) < 1e-9:
        raise RuntimeError("reference point y values repeat")
    alpha = (float(y_px) - float(first.y_px)) / delta
    return float(first.h_mm + alpha * (second.h_mm - first.h_mm))


def evaluate_height_mapping(reference_points: Sequence[ReferencePoint], y_px: Optional[float]) -> Optional[float]:
    if y_px is None or not math.isfinite(float(y_px)):
        return None
    y_value = float(y_px)
    if y_value >= reference_points[0].y_px:
        return interpolate_h_from_y(reference_points[0], reference_points[1], y_value)
    if y_value <= reference_points[-1].y_px:
        return interpolate_h_from_y(reference_points[-2], reference_points[-1], y_value)
    for index in range(len(reference_points) - 1):
        first = reference_points[index]
        second = reference_points[index + 1]
        if first.y_px >= y_value >= second.y_px:
            return interpolate_h_from_y(first, second, y_value)
    raise RuntimeError("failed to map y_rect to height")


def select_rows_for_bag(rows: Sequence[Dict[str, object]], top_k: int, min_frame_gap: int) -> List[Dict[str, object]]:
    ranked = sorted(
        rows,
        key=lambda row: (
            -float(row["abs_gap_mm"]),
            -abs(float(row.get("signed_gap_mm", 0.0))),
            int(row.get("frame_index", -1)),
        ),
    )
    selected: List[Dict[str, object]] = []
    selected_frames: List[int] = []
    for row in ranked:
        if len(selected) >= int(top_k):
            break
        frame_index = int(row.get("frame_index", -1))
        if any(abs(frame_index - prev) < int(min_frame_gap) for prev in selected_frames):
            continue
        selected.append(dict(row))
        selected_frames.append(frame_index)
    return selected


def summarize_session(rows: Sequence[Dict[str, object]]) -> Dict[str, object]:
    gaps = [float(row["abs_gap_mm"]) for row in rows]
    signed = [float(row["signed_gap_mm"]) for row in rows]
    return {
        "count": len(rows),
        "mean_abs_gap_mm": float(sum(gaps) / len(gaps)) if gaps else None,
        "max_abs_gap_mm": max(gaps) if gaps else None,
        "mean_signed_gap_mm": float(sum(signed) / len(signed)) if signed else None,
    }


def main() -> int:
    try:
        args = parse_args()
        manifest_csv = Path(args.manifest_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        rows = read_csv_rows(manifest_csv)
        calibration_cache: Dict[str, List[ReferencePoint]] = {}
        enriched: List[Dict[str, object]] = []

        for row in rows:
            human_peak_y_rect_px = finite_float(row.get("human_peak_y_rect_px"))
            human_peak_mm = finite_float(row.get("human_peak_mm"))
            calibration_raw = str(row.get("calibration_path", "")).strip()
            if human_peak_y_rect_px is None or human_peak_mm is None or calibration_raw == "":
                continue
            calibration_path = Path(calibration_raw).expanduser().resolve()
            cache_key = str(calibration_path)
            if cache_key not in calibration_cache:
                calibration_cache[cache_key] = load_reference_points(calibration_path)
            reference_points = calibration_cache[cache_key]
            projected_peak_mm = evaluate_height_mapping(reference_points, human_peak_y_rect_px)
            if projected_peak_mm is None:
                continue
            signed_gap_mm = float(projected_peak_mm - human_peak_mm)
            enriched.append(
                {
                    "row_id": str(row.get("row_id", "")),
                    "session_id": str(row.get("session_id", "")),
                    "session_name": str(row.get("session_name", "")),
                    "bag_id": str(row.get("bag_id", "")),
                    "bag_path": str(row.get("bag_path", "")),
                    "debug_dir": str(row.get("debug_dir", "")),
                    "calibration_path": str(calibration_path),
                    "frame_index": int(float(row.get("frame_index", -1) or -1)),
                    "relative_time_s": float(finite_float(row.get("relative_time_s")) or 0.0),
                    "photo_path": str(row.get("photo_path", "")),
                    "roi_debug_path": str(row.get("roi_debug_path", "")),
                    "raw_rectified_roi_path": str(row.get("raw_rectified_roi_path", "")),
                    "human_peak_y_rect_px": float(human_peak_y_rect_px),
                    "human_peak_mm": float(human_peak_mm),
                    "projected_peak_mm_from_y_rect": float(projected_peak_mm),
                    "signed_gap_mm": float(signed_gap_mm),
                    "abs_gap_mm": float(abs(signed_gap_mm)),
                    "slosh_height_mm": finite_float(row.get("slosh_height_mm")),
                    "peak_rel_mm_v2": finite_float(row.get("peak_rel_mm_v2")),
                    "center_rel_mm_v2": finite_float(row.get("center_rel_mm_v2")),
                    "confidence_v2": finite_float(row.get("confidence_v2")),
                    "accept_for_peak_report_v2": str(row.get("accept_for_peak_report_v2", "")),
                    "raw_rectified_zero_y_px": finite_float(row.get("raw_rectified_zero_y_px")),
                }
            )

        if not enriched:
            raise RuntimeError("no rows with both human_peak_mm and human_peak_y_rect_px")

        by_bag: Dict[str, List[Dict[str, object]]] = defaultdict(list)
        for row in enriched:
            by_bag[str(row["bag_id"])].append(row)

        selected: List[Dict[str, object]] = []
        session_summary: Dict[str, object] = {}
        for bag_id, bag_rows in sorted(by_bag.items()):
            chosen = select_rows_for_bag(
                rows=bag_rows,
                top_k=int(args.top_k_per_bag),
                min_frame_gap=int(args.min_frame_gap),
            )
            selected.extend(chosen)
            session_summary[bag_rows[0]["session_name"]] = summarize_session(bag_rows)

        selected = sorted(
            selected,
            key=lambda row: (
                str(row["session_name"]),
                float(row["relative_time_s"]),
                int(row["frame_index"]),
            ),
        )

        selected_csv = out_dir / "SL_candidate_frames_selected.csv"
        fieldnames = [
            "row_id",
            "session_id",
            "session_name",
            "bag_id",
            "bag_path",
            "debug_dir",
            "calibration_path",
            "frame_index",
            "relative_time_s",
            "photo_path",
            "roi_debug_path",
            "raw_rectified_roi_path",
            "human_peak_y_rect_px",
            "human_peak_mm",
            "projected_peak_mm_from_y_rect",
            "signed_gap_mm",
            "abs_gap_mm",
            "slosh_height_mm",
            "peak_rel_mm_v2",
            "center_rel_mm_v2",
            "confidence_v2",
            "accept_for_peak_report_v2",
            "raw_rectified_zero_y_px",
        ]
        with selected_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(selected)

        summary = {
            "manifest_csv": str(manifest_csv),
            "selected_csv": str(selected_csv),
            "num_rows_with_both_labels": len(enriched),
            "num_selected_rows": len(selected),
            "top_k_per_bag": int(args.top_k_per_bag),
            "min_frame_gap": int(args.min_frame_gap),
            "session_summary": session_summary,
        }
        summary_json = out_dir / "SL_candidate_frames_summary.json"
        with summary_json.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)

        print(f"[OK] selected csv: {selected_csv}")
        print(f"[OK] summary json: {summary_json}")
        print(f"[OK] rows_with_both_labels: {len(enriched)}")
        print(f"[OK] selected_rows: {len(selected)}")
        for row in selected[: max(1, int(args.show_first))]:
            print(
                "[OK] "
                f"session={row['session_name']} frame={row['frame_index']} "
                f"human_mm={float(row['human_peak_mm']):.3f} "
                f"F(y)={float(row['projected_peak_mm_from_y_rect']):.3f} "
                f"gap={float(row['signed_gap_mm']):+.3f}"
            )
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
