#!/usr/bin/env python3
"""Render image-level debug panels for SL visual predictions against human labels."""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Render image-level debug sheets from SL_visual_human_predictions.csv, annotated with "
            "human peak, SL prediction, /slosh/height, and their errors."
        )
    )
    parser.add_argument("--predictions-csv", required=True, help="Path to SL_visual_human_predictions.csv.")
    parser.add_argument("--out-dir", required=True, help="Output directory for debug images and ranking csv.")
    parser.add_argument(
        "--split",
        default="test",
        help="Split to render. Default: test.",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=24,
        help="How many examples to render for each ranking. Default: 24.",
    )
    parser.add_argument(
        "--columns",
        type=int,
        default=4,
        help="Number of columns in contact sheets. Default: 4.",
    )
    parser.add_argument(
        "--tile-width",
        type=int,
        default=220,
        help="Rendered image width per tile. Default: 220.",
    )
    parser.add_argument(
        "--text-height",
        type=int,
        default=122,
        help="Text area height per tile. Default: 122.",
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


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"csv has no rows: {path}")
    return rows


def load_font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def enrich_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, object]]:
    enriched: List[Dict[str, object]] = []
    for row in rows:
        target = finite_float(row.get("target_human_peak_mm"))
        slosh = finite_float(row.get("baseline_slosh_height_mm"))
        visual = finite_float(row.get("pred_visual"))
        image_path = Path(str(row.get("image_path", "")).strip()).expanduser()
        if target is None or slosh is None or visual is None or not image_path.exists():
            continue
        slosh_error = abs(float(slosh) - float(target))
        visual_error = abs(float(visual) - float(target))
        enriched.append(
            {
                **row,
                "target": float(target),
                "slosh": float(slosh),
                "visual": float(visual),
                "slosh_error": float(slosh_error),
                "visual_error": float(visual_error),
                "gain_visual_minus_slosh": float(slosh_error - visual_error),
                "image_path_obj": image_path,
            }
        )
    if not enriched:
        raise RuntimeError("no rows with valid image + target + predictions")
    return enriched


def sort_rows(rows: Sequence[Dict[str, object]], key: str, reverse: bool = True, top_k: int = 24) -> List[Dict[str, object]]:
    ranked = sorted(rows, key=lambda row: float(row[key]), reverse=reverse)
    return [dict(row) for row in ranked[: max(1, int(top_k))]]


def draw_tile(
    row: Dict[str, object],
    tile_width: int,
    text_height: int,
    title_font,
    body_font,
) -> Image.Image:
    image = Image.open(row["image_path_obj"]).convert("RGB")
    scale = float(tile_width) / float(image.width)
    image_height = max(8, int(round(float(image.height) * scale)))
    image = image.resize((tile_width, image_height), Image.BILINEAR)

    total_height = image_height + int(text_height)
    panel = Image.new("RGB", (tile_width, total_height), color=(255, 255, 255))
    panel.paste(image, (0, 0))
    draw = ImageDraw.Draw(panel)

    visual_error = float(row["visual_error"])
    slosh_error = float(row["slosh_error"])
    if visual_error + 1e-12 < slosh_error:
        border_color = (40, 140, 70)
    elif slosh_error + 1e-12 < visual_error:
        border_color = (180, 100, 20)
    else:
        border_color = (120, 120, 120)
    draw.rectangle([(0, 0), (tile_width - 1, total_height - 1)], outline=border_color, width=3)

    lines = [
        f"{row['session_name']}  f={row['frame_index']}",
        f"t={float(row['relative_time_s']):.2f}s",
        f"human = {float(row['target']):.3f} mm",
        f"SL    = {float(row['visual']):.3f}  |e|={visual_error:.3f}",
        f"slosh = {float(row['slosh']):.3f}  |e|={slosh_error:.3f}",
        f"gain(SL) = {float(row['gain_visual_minus_slosh']):+.3f} mm",
    ]
    y = image_height + 8
    for idx, line in enumerate(lines):
        font = title_font if idx == 0 else body_font
        draw.text((8, y), line, fill=(20, 20, 20), font=font)
        y += 18 if idx == 0 else 16
    return panel


def build_sheet(
    rows: Sequence[Dict[str, object]],
    out_path: Path,
    sheet_title: str,
    columns: int,
    tile_width: int,
    text_height: int,
):
    columns = max(1, int(columns))
    title_font = load_font(15)
    body_font = load_font(13)
    tiles = [draw_tile(row, tile_width=tile_width, text_height=text_height, title_font=title_font, body_font=body_font) for row in rows]
    if not tiles:
        raise RuntimeError(f"no tiles for {out_path}")
    tile_height = tiles[0].height
    rows_count = int(math.ceil(len(tiles) / float(columns)))
    title_band = 30
    canvas = Image.new("RGB", (columns * tile_width, title_band + rows_count * tile_height), color=(248, 248, 248))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 6), sheet_title, fill=(20, 20, 20), font=title_font)
    for idx, tile in enumerate(tiles):
        row_idx = idx // columns
        col_idx = idx % columns
        x0 = col_idx * tile_width
        y0 = title_band + row_idx * tile_height
        canvas.paste(tile, (x0, y0))
    canvas.save(out_path)


def write_ranking_csv(path: Path, rows: Sequence[Dict[str, object]]):
    fieldnames = [
        "rank",
        "session_name",
        "bag_id",
        "frame_index",
        "relative_time_s",
        "image_path",
        "target_human_peak_mm",
        "pred_visual",
        "visual_abs_error_mm",
        "baseline_slosh_height_mm",
        "slosh_abs_error_mm",
        "gain_visual_minus_slosh_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for idx, row in enumerate(rows, start=1):
            writer.writerow(
                {
                    "rank": idx,
                    "session_name": row["session_name"],
                    "bag_id": row["bag_id"],
                    "frame_index": row["frame_index"],
                    "relative_time_s": row["relative_time_s"],
                    "image_path": str(row["image_path_obj"]),
                    "target_human_peak_mm": row["target"],
                    "pred_visual": row["visual"],
                    "visual_abs_error_mm": row["visual_error"],
                    "baseline_slosh_height_mm": row["slosh"],
                    "slosh_abs_error_mm": row["slosh_error"],
                    "gain_visual_minus_slosh_mm": row["gain_visual_minus_slosh"],
                }
            )


def main() -> int:
    try:
        args = parse_args()
        predictions_csv = Path(args.predictions_csv).expanduser().resolve()
        out_dir = Path(args.out_dir).expanduser().resolve()
        ensure_dir(out_dir)

        rows = read_rows(predictions_csv)
        split_rows = [row for row in rows if str(row.get("split", "")) == str(args.split)]
        if not split_rows:
            raise RuntimeError(f"no rows matched split={args.split}")
        enriched_rows = enrich_rows(split_rows)

        rankings = {
            "worst_visual": (
                sort_rows(enriched_rows, key="visual_error", reverse=True, top_k=int(args.top_k)),
                f"Worst visual error | split={args.split}",
            ),
            "worst_slosh": (
                sort_rows(enriched_rows, key="slosh_error", reverse=True, top_k=int(args.top_k)),
                f"Worst /slosh/height error | split={args.split}",
            ),
            "visual_best_gain": (
                sort_rows(enriched_rows, key="gain_visual_minus_slosh", reverse=True, top_k=int(args.top_k)),
                f"Visual beats /slosh/height most | split={args.split}",
            ),
            "slosh_best_gain": (
                sort_rows(enriched_rows, key="gain_visual_minus_slosh", reverse=False, top_k=int(args.top_k)),
                f"/slosh/height beats visual most | split={args.split}",
            ),
        }

        for name, (ranked_rows, title) in rankings.items():
            image_path = out_dir / f"SL_debug_{args.split}_{name}.png"
            csv_path = out_dir / f"SL_debug_{args.split}_{name}.csv"
            build_sheet(
                ranked_rows,
                out_path=image_path,
                sheet_title=title,
                columns=int(args.columns),
                tile_width=int(args.tile_width),
                text_height=int(args.text_height),
            )
            write_ranking_csv(csv_path, ranked_rows)
            print(f"[OK] sheet: {image_path}")
            print(f"[OK] ranking csv: {csv_path}")
        return 0
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
