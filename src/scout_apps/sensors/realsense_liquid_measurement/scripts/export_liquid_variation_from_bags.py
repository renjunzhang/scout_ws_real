#!/usr/bin/env python3
"""Export liquid-height variation CSVs from one run or a whole experiment group.

This is the operator-facing bridge between real-robot bag recording and the
red-liquid RGB inference stack:

  run one group -> point this script at the bag directory -> get per-run and
  group-level liquid variation CSV/plot outputs.

Default source is offline RGB inference, because paper/formal metrics should be
recomputed from the recorded camera stream.  If a bag already contains online
/liquid/* topics, --source online can export those directly for a fast field
check.
"""

import argparse
import bisect
import csv
import glob
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    import rosbag
    _HAS_ROSBAG = True
except ImportError:
    rosbag = None  # type: ignore[assignment]
    _HAS_ROSBAG = False

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402
    _HAS_MPL = True
except ImportError:
    plt = None  # type: ignore[assignment]
    _HAS_MPL = False

SCRIPT_DIR = Path(__file__).resolve().parent
RGB_INFER_SCRIPT = SCRIPT_DIR / "red_liquid_infer_from_bag.py"

TIMESERIES_HEADER = [
    "run_name", "source", "bag_path", "t_s", "stamp_sec",
    "max_lcr_mm", "left_mm", "center_mm", "right_mm", "median_mm",
    "max_lcr_corr_mm", "max_lcr_smooth_corr_mm", "main_mm",
    "conf_mean", "any_clipped",
]

SUMMARY_HEADER = [
    "run_name", "source", "bag_path", "timeseries_csv", "rgb_result_dir",
    "samples", "valid_samples", "valid_ratio", "duration_s",
    "main_peak_abs_mm", "main_peak_pos_mm", "main_peak_neg_mm",
    "main_p95_abs_mm", "main_rms_mm", "main_mean_abs_mm",
    "max_lcr_peak_abs_mm", "left_p95_abs_mm", "center_p95_abs_mm", "right_p95_abs_mm",
    "clipped_rate",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Export liquid variation data from RGB bags or recorded /liquid/* topics."
    )
    p.add_argument("paths", nargs="+",
                   help="Bag file(s), directories containing .bag files, or glob patterns.")
    p.add_argument("--out-dir", default="",
                   help="Output directory. Default: <first input dir>/liquid_variation_export")
    p.add_argument("--source", choices=["rgb", "online", "auto"], default="rgb",
                   help="rgb=recompute from camera frames; online=read /liquid/* from bag; auto=prefer rgb if calibration is provided.")
    p.add_argument("--calibration", default="",
                   help="Three-ruler calibration YAML. Required for --source rgb/auto-rgb.")
    p.add_argument("--topic", default="/camera/color/image_raw",
                   help="RGB image topic for offline inference.")
    p.add_argument("--online-height-topic", default="/liquid/height")
    p.add_argument("--online-lcr-topic", default="/liquid/height_lcr")
    p.add_argument("--recursive", action="store_true",
                   help="When a path is a directory, search for bags recursively.")
    p.add_argument("--pattern", default="*.bag",
                   help="Bag filename pattern used for directory inputs. Default: *.bag")
    p.add_argument("--allow-fail", action="store_true",
                   help="Continue processing later bags if one bag fails.")

    # Forwarded to red_liquid_infer_from_bag.py.
    p.add_argument("--time-offset", type=float, default=0.0)
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--every", type=int, default=1)
    p.add_argument("--debug-every", type=int, default=30)
    p.add_argument("--zero-correction-frames", type=int, default=30)
    p.add_argument("--smooth-frames", type=int, default=5)
    p.add_argument("--hue1-low", type=int, default=0)
    p.add_argument("--hue1-high", type=int, default=11)
    p.add_argument("--hue2-low", type=int, default=173)
    p.add_argument("--hue2-high", type=int, default=179)
    p.add_argument("--sat-min", type=int, default=80)
    p.add_argument("--val-min", type=int, default=162)
    p.add_argument("--morph-kernel", type=int, default=5)
    p.add_argument("--top-boundary-quantile", type=float, default=0.2)
    p.add_argument("--min-valid-column-fraction", type=float, default=0.15)
    p.add_argument("--min-component-area", type=int, default=30)
    p.add_argument("--bottom-touch-rows", type=int, default=15)
    return p.parse_args()


def _as_float(value: object) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, float):
        if math.isnan(value):
            return None
        return value
    text = str(value).strip()
    if not text:
        return None
    try:
        v = float(text)
    except ValueError:
        return None
    if math.isnan(v):
        return None
    return v


def _fmt(value: Optional[float], digits: int = 5) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def discover_bags(paths: Sequence[str], pattern: str, recursive: bool) -> List[Path]:
    bags: List[Path] = []
    for raw in paths:
        expanded = Path(raw).expanduser()
        matches: Iterable[str]
        if any(ch in raw for ch in "*?[]"):
            matches = glob.glob(str(expanded), recursive=recursive)
            bags.extend(Path(m).expanduser().resolve() for m in matches if Path(m).suffix == ".bag")
        elif expanded.is_dir():
            globber = expanded.rglob if recursive else expanded.glob
            bags.extend(p.resolve() for p in globber(pattern) if p.is_file())
        else:
            bags.append(expanded.resolve())
    unique = []
    seen = set()
    for bag in sorted(bags):
        if bag not in seen:
            seen.add(bag)
            unique.append(bag)
    return unique


def default_out_dir(first_input: str) -> Path:
    p = Path(first_input).expanduser()
    if any(ch in first_input for ch in "*?[]"):
        parent = Path(glob.glob(str(p))[0]).parent if glob.glob(str(p)) else Path.cwd()
    elif p.is_dir():
        parent = p
    else:
        parent = p.parent
    return (parent / "liquid_variation_export").resolve()


def bag_topics(bag_path: Path) -> Dict[str, object]:
    if not _HAS_ROSBAG:
        raise RuntimeError("rosbag is not available; source ROS before running this script")
    with rosbag.Bag(str(bag_path), "r") as bag:  # type: ignore[union-attr]
        return bag.get_type_and_topic_info().topics


def choose_source(args: argparse.Namespace, bag_path: Path) -> str:
    if args.source != "auto":
        return args.source
    topics = bag_topics(bag_path)
    if args.calibration and args.topic in topics:
        return "rgb"
    if args.online_height_topic in topics:
        return "online"
    raise RuntimeError(
        f"auto source failed for {bag_path}: no RGB topic {args.topic} with calibration and no {args.online_height_topic}"
    )


def run_rgb_inference(args: argparse.Namespace, bag_path: Path, run_out_dir: Path) -> Path:
    if not args.calibration:
        raise RuntimeError("--calibration is required for RGB inference")
    run_out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(RGB_INFER_SCRIPT),
        "--bag", str(bag_path),
        "--calibration", str(Path(args.calibration).expanduser()),
        "--topic", args.topic,
        "--out-dir", str(run_out_dir),
        "--time-offset", str(args.time_offset),
        "--max-frames", str(args.max_frames),
        "--every", str(args.every),
        "--debug-every", str(args.debug_every),
        "--zero-correction-frames", str(args.zero_correction_frames),
        "--smooth-frames", str(args.smooth_frames),
        "--hue1-low", str(args.hue1_low),
        "--hue1-high", str(args.hue1_high),
        "--hue2-low", str(args.hue2_low),
        "--hue2-high", str(args.hue2_high),
        "--sat-min", str(args.sat_min),
        "--val-min", str(args.val_min),
        "--morph-kernel", str(args.morph_kernel),
        "--top-boundary-quantile", str(args.top_boundary_quantile),
        "--min-valid-column-fraction", str(args.min_valid_column_fraction),
        "--min-component-area", str(args.min_component_area),
        "--bottom-touch-rows", str(args.bottom_touch_rows),
    ]
    subprocess.run(cmd, check=True)
    csv_path = run_out_dir / f"{bag_path.stem}_red_top.csv"
    if not csv_path.exists():
        raise RuntimeError(f"RGB inference did not create expected CSV: {csv_path}")
    return csv_path


def parse_rgb_csv(csv_path: Path, bag_path: Path) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            stamp = _as_float(r.get("stamp_sec"))
            left = _as_float(r.get("h_mm_left"))
            center = _as_float(r.get("h_mm_center"))
            right = _as_float(r.get("h_mm_right"))
            vals = [v for v in (left, center, right) if v is not None]
            max_lcr = _as_float(r.get("h_mm_max_lcr"))
            if max_lcr is None and vals:
                max_lcr = max(vals)
            max_lcr_corr = _as_float(r.get("h_mm_max_lcr_corr"))
            max_lcr_smooth_corr = _as_float(r.get("h_mm_max_lcr_smooth_corr"))
            median = _as_float(r.get("h_mm_final"))
            main = max_lcr_smooth_corr if max_lcr_smooth_corr is not None else (
                max_lcr_corr if max_lcr_corr is not None else max_lcr
            )
            rows.append({
                "run_name": bag_path.stem,
                "source": "rgb",
                "bag_path": str(bag_path),
                "stamp_sec": stamp,
                "max_lcr_mm": max_lcr,
                "left_mm": left,
                "center_mm": center,
                "right_mm": right,
                "median_mm": median,
                "max_lcr_corr_mm": max_lcr_corr,
                "max_lcr_smooth_corr_mm": max_lcr_smooth_corr,
                "main_mm": main,
                "conf_mean": _as_float(r.get("conf_mean")),
                "any_clipped": _as_float(r.get("any_clipped")),
            })
    return add_relative_time(rows)


def _nearest_lcr(stamp: float, lcr_stamps: List[float], lcr_vals: List[List[Optional[float]]]) -> List[Optional[float]]:
    if not lcr_stamps:
        return [None, None, None]
    pos = bisect.bisect_left(lcr_stamps, stamp)
    candidates = []
    if pos < len(lcr_stamps):
        candidates.append(pos)
    if pos > 0:
        candidates.append(pos - 1)
    best = min(candidates, key=lambda i: abs(lcr_stamps[i] - stamp))
    return lcr_vals[best]


def parse_online_topics(args: argparse.Namespace, bag_path: Path) -> List[Dict[str, object]]:
    if not _HAS_ROSBAG:
        raise RuntimeError("rosbag is not available; source ROS before running this script")
    topics = bag_topics(bag_path)
    if args.online_height_topic not in topics:
        raise RuntimeError(f"{bag_path} does not contain {args.online_height_topic}")

    height_samples: List[Tuple[float, Optional[float]]] = []
    lcr_stamps: List[float] = []
    lcr_vals: List[List[Optional[float]]] = []
    read_topics = [args.online_height_topic]
    if args.online_lcr_topic in topics:
        read_topics.append(args.online_lcr_topic)

    with rosbag.Bag(str(bag_path), "r") as bag:  # type: ignore[union-attr]
        for topic, msg, stamp in bag.read_messages(topics=read_topics):
            st = stamp.to_sec()
            if topic == args.online_height_topic:
                height_samples.append((st, _as_float(getattr(msg, "data", None))))
            elif topic == args.online_lcr_topic:
                vals = [None, None, None]
                for i, v in enumerate(list(getattr(msg, "data", []))[:3]):
                    vals[i] = _as_float(v)
                lcr_stamps.append(st)
                lcr_vals.append(vals)

    rows: List[Dict[str, object]] = []
    for st, h in height_samples:
        left, center, right = _nearest_lcr(st, lcr_stamps, lcr_vals)
        vals = [v for v in (left, center, right) if v is not None]
        rows.append({
            "run_name": bag_path.stem,
            "source": "online",
            "bag_path": str(bag_path),
            "stamp_sec": st,
            "max_lcr_mm": h,
            "left_mm": left,
            "center_mm": center,
            "right_mm": right,
            "median_mm": float(np.median(vals)) if vals else None,
            "max_lcr_corr_mm": h,
            "max_lcr_smooth_corr_mm": h,
            "main_mm": h,
            "conf_mean": None,
            "any_clipped": None,
        })
    return add_relative_time(rows)


def add_relative_time(rows: List[Dict[str, object]]) -> List[Dict[str, object]]:
    stamps = [_as_float(r.get("stamp_sec")) for r in rows]
    valid_stamps = [s for s in stamps if s is not None]
    t0 = valid_stamps[0] if valid_stamps else 0.0
    for r in rows:
        st = _as_float(r.get("stamp_sec"))
        r["t_s"] = (st - t0) if st is not None else None
    return rows


def write_timeseries(path: Path, rows: List[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=TIMESERIES_HEADER)
        w.writeheader()
        for r in rows:
            w.writerow({
                key: _fmt(r.get(key)) if key.endswith("_mm") or key in ("t_s", "stamp_sec", "conf_mean", "any_clipped") else r.get(key, "")
                for key in TIMESERIES_HEADER
            })


def _series(rows: List[Dict[str, object]], key: str) -> List[float]:
    return [v for v in (_as_float(r.get(key)) for r in rows) if v is not None]


def summarize_rows(rows: List[Dict[str, object]], timeseries_csv: Path, rgb_result_dir: Optional[Path]) -> Dict[str, object]:
    main = _series(rows, "main_mm")
    max_lcr = _series(rows, "max_lcr_mm")
    ts = _series(rows, "t_s")
    samples = len(rows)
    valid = len(main)
    abs_main = np.abs(np.asarray(main, dtype=float)) if main else np.asarray([], dtype=float)

    def p95_abs(key: str) -> Optional[float]:
        vals = _series(rows, key)
        return float(np.percentile(np.abs(vals), 95)) if vals else None

    clipped_vals = _series(rows, "any_clipped")
    row0 = rows[0] if rows else {}
    return {
        "run_name": row0.get("run_name", ""),
        "source": row0.get("source", ""),
        "bag_path": row0.get("bag_path", ""),
        "timeseries_csv": str(timeseries_csv),
        "rgb_result_dir": str(rgb_result_dir or ""),
        "samples": samples,
        "valid_samples": valid,
        "valid_ratio": (valid / samples) if samples else None,
        "duration_s": (max(ts) - min(ts)) if len(ts) >= 2 else None,
        "main_peak_abs_mm": float(np.max(abs_main)) if main else None,
        "main_peak_pos_mm": float(np.max(main)) if main else None,
        "main_peak_neg_mm": float(np.min(main)) if main else None,
        "main_p95_abs_mm": float(np.percentile(abs_main, 95)) if main else None,
        "main_rms_mm": float(np.sqrt(np.mean(np.square(main)))) if main else None,
        "main_mean_abs_mm": float(np.mean(abs_main)) if main else None,
        "max_lcr_peak_abs_mm": float(np.max(np.abs(max_lcr))) if max_lcr else None,
        "left_p95_abs_mm": p95_abs("left_mm"),
        "center_p95_abs_mm": p95_abs("center_mm"),
        "right_p95_abs_mm": p95_abs("right_mm"),
        "clipped_rate": float(np.mean(clipped_vals)) if clipped_vals else None,
    }


def write_summary(path: Path, rows: List[Dict[str, object]]) -> None:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_HEADER)
        w.writeheader()
        for r in rows:
            w.writerow({
                key: _fmt(r.get(key)) if isinstance(r.get(key), float) or r.get(key) is None else r.get(key, "")
                for key in SUMMARY_HEADER
            })


def write_plot(path: Path, grouped_rows: List[List[Dict[str, object]]]) -> None:
    if not _HAS_MPL:
        return
    fig, ax = plt.subplots(figsize=(12, 5.5))
    for rows in grouped_rows:
        ts = _series(rows, "t_s")
        ys = _series(rows, "main_mm")
        if not ts or not ys:
            continue
        n = min(len(ts), len(ys))
        label = str(rows[0].get("run_name", "run"))
        ax.plot(ts[:n], ys[:n], linewidth=1.5, alpha=0.9, label=label)
    ax.axhline(0.0, color="#666666", linewidth=0.8, linestyle=":")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("liquid variation main_mm (mm)")
    ax.set_title("Liquid variation by run")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    bags = discover_bags(args.paths, args.pattern, args.recursive)
    if not bags:
        print("[ERROR] no .bag files found", file=sys.stderr)
        return 2
    for bag in bags:
        if not bag.exists():
            print(f"[ERROR] bag not found: {bag}", file=sys.stderr)
            return 2

    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else default_out_dir(args.paths[0])
    runs_dir = out_dir / "runs"
    out_dir.mkdir(parents=True, exist_ok=True)
    runs_dir.mkdir(parents=True, exist_ok=True)

    all_rows: List[Dict[str, object]] = []
    summary_rows: List[Dict[str, object]] = []
    grouped_rows: List[List[Dict[str, object]]] = []
    failures = 0

    for bag in bags:
        print(f"[export_liquid_variation] processing {bag}")
        try:
            source = choose_source(args, bag)
            rgb_result_dir: Optional[Path] = None
            if source == "rgb":
                rgb_result_dir = runs_dir / f"{bag.stem}_rgb"
                red_csv = run_rgb_inference(args, bag, rgb_result_dir)
                rows = parse_rgb_csv(red_csv, bag)
            else:
                rows = parse_online_topics(args, bag)

            per_run_csv = runs_dir / f"{bag.stem}_liquid_variation.csv"
            write_timeseries(per_run_csv, rows)
            summary_rows.append(summarize_rows(rows, per_run_csv, rgb_result_dir))
            all_rows.extend(rows)
            grouped_rows.append(rows)
            print(f"[OK] per-run CSV: {per_run_csv}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"[ERROR] failed {bag}: {exc}", file=sys.stderr)
            if not args.allow_fail:
                return 10

    combined_csv = out_dir / "liquid_variation_timeseries.csv"
    summary_csv = out_dir / "liquid_variation_summary.csv"
    manifest_txt = out_dir / "liquid_variation_manifest.txt"
    plot_png = out_dir / "liquid_variation_runs.png"

    write_timeseries(combined_csv, all_rows)
    write_summary(summary_csv, summary_rows)
    write_plot(plot_png, grouped_rows)
    with manifest_txt.open("w") as f:
        f.write(f"source={args.source}\n")
        f.write(f"calibration={args.calibration}\n")
        f.write(f"topic={args.topic}\n")
        f.write(f"online_height_topic={args.online_height_topic}\n")
        f.write(f"online_lcr_topic={args.online_lcr_topic}\n")
        f.write(f"zero_correction_frames={args.zero_correction_frames}\n")
        f.write(f"smooth_frames={args.smooth_frames}\n")
        f.write(f"hsv=h1[{args.hue1_low},{args.hue1_high}] h2[{args.hue2_low},{args.hue2_high}] sat>={args.sat_min} val>={args.val_min}\n")
        f.write("bags:\n")
        for bag in bags:
            f.write(f"  - {bag}\n")

    print("[OK] group timeseries:", combined_csv)
    print("[OK] group summary:   ", summary_csv)
    if _HAS_MPL:
        print("[OK] group plot:      ", plot_png)
    print("[OK] manifest:        ", manifest_txt)
    return 0 if failures == 0 else 20


if __name__ == "__main__":
    sys.exit(main())
