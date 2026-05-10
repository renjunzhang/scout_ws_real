#!/usr/bin/env python3
"""Phase4 red-liquid visual metrics and /slosh/height alignment.

This is a narrow offline analysis script for the 2026-05-09 phase4 real bags.
It reuses red_liquid_infer_from_bag.py for RGB red-liquid detection and keeps
all outputs under the requested analysis directory.
"""

import argparse
import csv
import importlib.util
import math
import statistics
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag  # noqa: E402
from cv_bridge import CvBridge, CvBridgeError  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
RED_INFER_PATH = SCRIPT_DIR / "red_liquid_infer_from_bag.py"

DEFAULT_BAGS = [
    "slosh_Q0_20260509_203210_RAW_REAL_run01.bag",
    "slosh_Q0_20260509_203408_GEOREF_FIXED_MILD_REAL_run01.bag",
    "slosh_Q0_20260509_203540_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_run01.bag",
    "slosh_Q0_20260509_204602_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static.bag",
    "slosh_Q0_20260509_204629_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_run02.bag",
    "slosh_Q0_20260509_204850_GEOREF_FIXED_MILD_REAL_run02.bag",
    "slosh_Q0_20260509_205024_RAW_REAL_run02.bag",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag-dir", default="/data/a/slosh_bags/real/20260508_phase4")
    p.add_argument("--calibration", default="/home/a/scout_ws/red_liquid_3rulers.yaml")
    p.add_argument("--out-dir", default="/home/a/scout_ws/docs/Claude/分析数据/phase4_visual_20260509")
    p.add_argument("--topic", default="/camera/color/image_raw")
    p.add_argument("--encoding", default="bgr8")
    p.add_argument("--every", type=int, default=1)
    p.add_argument("--smooth-frames", type=int, default=5)
    p.add_argument("--max-interpolation-gap-sec", type=float, default=0.15)
    p.add_argument("--motion-threshold", type=float, default=0.03)
    p.add_argument("--motion-consecutive", type=int, default=3)
    p.add_argument("--hue1-low", type=int, default=0)
    p.add_argument("--hue1-high", type=int, default=12)
    p.add_argument("--hue2-low", type=int, default=168)
    p.add_argument("--hue2-high", type=int, default=179)
    p.add_argument("--sat-min", type=int, default=70)
    p.add_argument("--val-min", type=int, default=35)
    p.add_argument("--morph-kernel", type=int, default=5)
    p.add_argument("--top-boundary-quantile", type=float, default=0.2)
    p.add_argument("--min-valid-column-fraction", type=float, default=0.15)
    p.add_argument("--bottom-touch-rows", type=int, default=15)
    p.add_argument("--min-component-area", type=int, default=30)
    return p.parse_args()


def load_red_module():
    spec = importlib.util.spec_from_file_location("red_liquid_infer_from_bag", RED_INFER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {RED_INFER_PATH}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_detector_args(args: argparse.Namespace):
    class DetectorArgs:
        pass

    d = DetectorArgs()
    for name in [
        "hue1_low",
        "hue1_high",
        "hue2_low",
        "hue2_high",
        "sat_min",
        "val_min",
        "morph_kernel",
        "top_boundary_quantile",
        "min_valid_column_fraction",
        "bottom_touch_rows",
        "min_component_area",
    ]:
        setattr(d, name, getattr(args, name))
    d.roi_x = d.roi_y = d.roi_w = d.roi_h = None
    d.tube_left = d.tube_right = None
    d.rotation_deg = None
    d.center_col_fraction = 0.6
    return d


def condition_from_name(name: str) -> str:
    if "RAW_REAL" in name:
        return "RAW"
    if "GEOREF_FIXED_MILD_REAL" in name:
        return "FIXED_MILD"
    if "GEOREF_OSCRS_MEDIUM_ACTIVE_REAL" in name:
        return "OSCRS_MEDIUM_ACTIVE"
    return "UNKNOWN"


def run_id_from_name(name: str) -> str:
    if "static" in name:
        return "static"
    if "run01" in name:
        return "run01"
    if "run02" in name:
        return "run02"
    return ""


def finite(values: Sequence[Optional[float]]) -> List[float]:
    return [float(v) for v in values if v is not None and math.isfinite(float(v))]


def percentile(values: Sequence[float], q: float) -> float:
    vals = [v for v in values if math.isfinite(v)]
    return float(np.percentile(vals, q)) if vals else math.nan


def rms(values: Sequence[float]) -> float:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return math.nan
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def corr(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return math.nan
    xs = [p[0] for p in pairs]
    ys = [p[1] for p in pairs]
    mx = statistics.mean(xs)
    my = statistics.mean(ys)
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx <= 1e-12 or sy <= 1e-12:
        return math.nan
    return sum((x - mx) * (y - my) for x, y in pairs) / (sx * sy)


def fit_scale(source: Sequence[float], target: Sequence[float]) -> float:
    pairs = [(s, t) for s, t in zip(source, target) if math.isfinite(s) and math.isfinite(t)]
    denom = sum(s * s for s, _ in pairs)
    if denom <= 1e-12:
        return math.nan
    return sum(s * t for s, t in pairs) / denom


def rolling_median(series: Sequence[Optional[float]], window: int) -> List[Optional[float]]:
    if window <= 1:
        return list(series)
    out: List[Optional[float]] = []
    half = window // 2
    for idx in range(len(series)):
        vals = finite(series[max(0, idx - half):min(len(series), idx + half + 1)])
        out.append(float(np.median(vals)) if vals else None)
    return out


def interpolate(
    sample_t: Sequence[float],
    series_t: Sequence[float],
    series_v: Sequence[float],
    max_gap_sec: float,
) -> List[float]:
    if not series_t:
        return [math.nan] * len(sample_t)
    out: List[float] = []
    for t in sample_t:
        idx = int(np.searchsorted(series_t, t, side="left"))
        if t < float(series_t[0]) or t > float(series_t[-1]):
            out.append(math.nan)
        elif idx <= 0:
            out.append(float(series_v[0]))
        elif idx >= len(series_t):
            out.append(float(series_v[-1]))
        else:
            t0, t1 = float(series_t[idx - 1]), float(series_t[idx])
            v0, v1 = float(series_v[idx - 1]), float(series_v[idx])
            if t1 <= t0 or t1 - t0 > max_gap_sec:
                out.append(math.nan)
            else:
                alpha = (t - t0) / (t1 - t0)
                out.append(v0 + alpha * (v1 - v0))
    return out


def nearest_dts(sample_t: Sequence[float], series_t: Sequence[float]) -> List[float]:
    out: List[float] = []
    if not series_t:
        return out
    for t in sample_t:
        idx = int(np.searchsorted(series_t, t, side="left"))
        candidates = []
        if idx < len(series_t):
            candidates.append(abs(float(series_t[idx]) - t))
        if idx > 0:
            candidates.append(abs(float(series_t[idx - 1]) - t))
        if candidates:
            out.append(min(candidates))
    return out


def read_slosh_series(bag_path: Path) -> Tuple[List[float], List[float]]:
    rel_t: List[float] = []
    vals_mm: List[float] = []
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=["/slosh/height"]):
            rel_t.append(stamp.to_sec() - start)
            vals_mm.append(float(msg.data) * 1000.0)
    return rel_t, vals_mm


def read_start_times(bag_path: Path, motion_threshold: float, motion_consecutive: int) -> Dict[str, float]:
    tracking_start: Optional[float] = None
    motion_start: Optional[float] = None
    consec = 0
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=["/mpc_status", "/odom"]):
            rel_t = stamp.to_sec() - start
            if topic == "/mpc_status" and tracking_start is None:
                if str(getattr(msg, "data", "")).strip() == "TRACKING":
                    tracking_start = rel_t
            elif topic == "/odom" and motion_start is None:
                v = float(getattr(msg.twist.twist, "linear").x)
                if abs(v) > motion_threshold:
                    consec += 1
                    if consec >= max(1, motion_consecutive):
                        motion_start = rel_t
                else:
                    consec = 0
            if tracking_start is not None and motion_start is not None:
                break
    analysis_start = motion_start if motion_start is not None else (tracking_start or 0.0)
    return {
        "tracking_start_sec": tracking_start if tracking_start is not None else math.nan,
        "motion_start_sec": motion_start if motion_start is not None else math.nan,
        "analysis_start_sec": analysis_start,
    }


def infer_visual_bag(
    bag_path: Path,
    topic: str,
    encoding: str,
    every: int,
    mod,
    geom: Dict,
    rulers: Dict,
    detector_args,
) -> Dict:
    bridge = CvBridge()
    rows: List[Dict] = []
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        frame_idx = 0
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            if frame_idx % max(1, every) != 0:
                frame_idx += 1
                continue
            try:
                image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            except CvBridgeError:
                frame_idx += 1
                continue
            _ytops, h_mms, confs, clipped, _mask = mod.detect_red_liquid(
                image, geom, rulers, detector_args
            )
            h_final = mod.h_mm_final(h_mms)
            rows.append(
                {
                    "frame_index": frame_idx,
                    "t_rel_sec": stamp.to_sec() - start,
                    "h_left": h_mms[0],
                    "h_center": h_mms[1],
                    "h_right": h_mms[2],
                    "h_final": h_final,
                    "conf_mean": float(np.mean([c for c in confs if c > 0])) if any(c > 0 for c in confs) else 0.0,
                    "any_clipped": int(any(clipped)),
                }
            )
            frame_idx += 1
    return {"rows": rows}


def write_timeseries(path: Path, rows: Sequence[Dict], baseline: Dict[str, float], smooth: Sequence[Optional[float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "frame_index",
        "t_rel_sec",
        "h_left",
        "h_center",
        "h_right",
        "h_final",
        "h_left_rel",
        "h_center_rel",
        "h_right_rel",
        "h_visual_peak_rel_mm",
        "h_visual_peak_rel_smooth_mm",
        "conf_mean",
        "any_clipped",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row, h_smooth in zip(rows, smooth):
            rels = relative_values(row, baseline)
            out = {k: row.get(k) for k in fields if k in row}
            out.update(rels)
            out["h_visual_peak_rel_smooth_mm"] = h_smooth
            writer.writerow({k: format_value(out.get(k)) for k in fields})


def write_aligned(path: Path, rows: Sequence[Dict], smooth: Sequence[Optional[float]], slosh_interp: Sequence[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "frame_index",
        "t_rel_sec",
        "h_visual_peak_rel_mm",
        "h_visual_peak_rel_smooth_mm",
        "slosh_height_rel_interp_mm",
        "conf_mean",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row, h_smooth, slosh in zip(rows, smooth, slosh_interp):
            writer.writerow(
                {
                    "frame_index": row["frame_index"],
                    "t_rel_sec": f"{row['t_rel_sec']:.6f}",
                    "h_visual_peak_rel_mm": format_value(row.get("h_visual_peak_rel_mm")),
                    "h_visual_peak_rel_smooth_mm": format_value(h_smooth),
                    "slosh_height_rel_interp_mm": format_value(slosh),
                    "conf_mean": format_value(row.get("conf_mean")),
                }
            )


def format_value(value) -> str:
    if value is None:
        return ""
    try:
        val = float(value)
    except (TypeError, ValueError):
        return str(value)
    return f"{val:.6f}" if math.isfinite(val) else ""


def relative_values(row: Dict, baseline: Dict[str, float]) -> Dict[str, Optional[float]]:
    rels: Dict[str, Optional[float]] = {}
    vals = []
    for key in ["left", "center", "right"]:
        raw = row.get(f"h_{key}")
        rel = None if raw is None else float(raw) - baseline[f"h_{key}"]
        rels[f"h_{key}_rel"] = rel
        if rel is not None:
            vals.append(rel)
    rels["h_visual_peak_rel_mm"] = max(0.0, max(vals)) if vals else None
    return rels


def plot_bag(path: Path, bag_label: str, rows: Sequence[Dict], smooth: Sequence[Optional[float]], slosh_interp: Sequence[float], start_sec: float) -> None:
    xs = [float(r["t_rel_sec"]) - start_sec for r in rows if float(r["t_rel_sec"]) >= start_sec]
    ys_v = [s for r, s in zip(rows, smooth) if float(r["t_rel_sec"]) >= start_sec]
    ys_s = [s for r, s in zip(rows, slosh_interp) if float(r["t_rel_sec"]) >= start_sec]
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.plot(xs, ys_v, label="visual peak rel (smoothed)", linewidth=1.8, color="#d9485f")
    ax.plot(xs, ys_s, label="/slosh/height rel interp", linewidth=1.5, color="#4e79a7")
    ax.set_xlabel("Time since analysis start (s)")
    ax.set_ylabel("Height (mm)")
    ax.set_title(bag_label)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def summarize_bag(
    bag_path: Path,
    rows: Sequence[Dict],
    smooth: Sequence[Optional[float]],
    slosh_t: Sequence[float],
    slosh_rel: Sequence[float],
    slosh_interp: Sequence[float],
    static_slosh_median: float,
    starts: Dict[str, float],
) -> Dict:
    start_sec = float(starts["analysis_start_sec"])
    win = [(r, h, s) for r, h, s in zip(rows, smooth, slosh_interp) if float(r["t_rel_sec"]) >= start_sec]
    visual = [float(h) for _r, h, _s in win if h is not None]
    slosh_pairs = [(float(h), float(s)) for _r, h, s in win if h is not None and math.isfinite(float(s))]
    visual_pair = [p[0] for p in slosh_pairs]
    slosh_pair = [p[1] for p in slosh_pairs]
    times_pair = [float(r["t_rel_sec"]) for r, h, s in win if h is not None and math.isfinite(float(s))]
    confs = [float(r["conf_mean"]) for r, _h, _s in win]
    clipped = [int(r["any_clipped"]) for r, _h, _s in win]
    nearest = nearest_dts(times_pair, slosh_t)
    peak_idx = int(np.argmax(visual)) if visual else -1
    peak_time = math.nan
    if peak_idx >= 0:
        valid_rows = [(r, h) for r, h, _s in win if h is not None]
        peak_time = float(valid_rows[peak_idx][0]["t_rel_sec"]) - start_sec
    slosh_window = [max(0.0, float(v) - static_slosh_median) for t, v in zip(slosh_t, slosh_rel) if t >= start_sec]
    visual_p95 = percentile(visual, 95.0)
    slosh_p95 = percentile(slosh_pair, 95.0)
    visual_max = max(visual) if visual else math.nan
    slosh_max = max(slosh_pair) if slosh_pair else math.nan
    return {
        "bag": bag_path.name,
        "condition": condition_from_name(bag_path.name),
        "run_id": run_id_from_name(bag_path.name),
        "analysis_start_sec": start_sec,
        "tracking_start_sec": starts["tracking_start_sec"],
        "motion_start_sec": starts["motion_start_sec"],
        "frames_total_window": len(win),
        "frames_visual_valid": len(visual),
        "valid_ratio": len(visual) / len(win) if win else math.nan,
        "confidence_mean": statistics.mean(confs) if confs else math.nan,
        "clipped_ratio": sum(clipped) / len(clipped) if clipped else math.nan,
        "h_visual_rms": rms(visual),
        "h_visual_p95": visual_p95,
        "h_visual_max": visual_max,
        "peak_time": peak_time,
        "slosh_rms": rms(slosh_window),
        "slosh_p95": slosh_p95,
        "slosh_max": slosh_max,
        "paired_samples": len(slosh_pairs),
        "pair_dt_median_ms": percentile(nearest, 50.0) * 1000.0,
        "pair_dt_p95_ms": percentile(nearest, 95.0) * 1000.0,
        "pair_dt_max_ms": max(nearest) * 1000.0 if nearest else math.nan,
        "correlation": corr(visual_pair, slosh_pair),
        "scale_fit_slosh_per_visual": fit_scale(visual_pair, slosh_pair),
        "scale_ratio_visual_p95_over_slosh_p95": visual_p95 / slosh_p95 if slosh_p95 > 1e-12 else math.nan,
        "scale_ratio_visual_max_over_slosh_max": visual_max / slosh_max if slosh_max > 1e-12 else math.nan,
    }


def write_csv(path: Path, rows: Sequence[Dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: format_value(v) if isinstance(v, float) else v for k, v in row.items()})


def condition_summary(rows: Sequence[Dict]) -> List[Dict]:
    out = []
    for cond in ["RAW", "FIXED_MILD", "OSCRS_MEDIUM_ACTIVE"]:
        group = [r for r in rows if r["condition"] == cond]
        if not group:
            continue
        item = {"condition": cond, "n": len(group)}
        for key in ["h_visual_rms", "h_visual_p95", "h_visual_max", "slosh_rms", "slosh_p95", "slosh_max", "valid_ratio", "confidence_mean", "correlation"]:
            vals = [float(r[key]) for r in group if math.isfinite(float(r[key]))]
            item[f"{key}_mean"] = statistics.mean(vals) if vals else math.nan
        out.append(item)
    return out


def reduction(a: float, b: float) -> float:
    return (1.0 - b / a) * 100.0 if a and math.isfinite(a) and math.isfinite(b) else math.nan


def write_report(path: Path, summary: Sequence[Dict], cond_rows: Sequence[Dict], baseline: Dict[str, float], static_slosh: float) -> None:
    cond = {r["condition"]: r for r in cond_rows}
    raw = cond.get("RAW", {})
    oscrs = cond.get("OSCRS_MEDIUM_ACTIVE", {})
    vis_p95_red = reduction(float(raw.get("h_visual_p95_mean", math.nan)), float(oscrs.get("h_visual_p95_mean", math.nan)))
    vis_max_red = reduction(float(raw.get("h_visual_max_mean", math.nan)), float(oscrs.get("h_visual_max_mean", math.nan)))
    slosh_p95_red = reduction(float(raw.get("slosh_p95_mean", math.nan)), float(oscrs.get("slosh_p95_mean", math.nan)))
    verdict = (
        "支持同方向趋势"
        if math.isfinite(vis_p95_red) and math.isfinite(slosh_p95_red) and vis_p95_red > 0 and slosh_p95_red > 0
        else "暂不支持同方向趋势"
    )
    lines = [
        "# Phase4 RGB Red-Liquid Visual Validation",
        "",
        "## Method",
        "",
        "- Source topic: `/camera/color/image_raw`.",
        "- Detector: existing three-ruler red-liquid HSV pipeline in `red_liquid_infer_from_bag.py`.",
        "- Static zero: medians from `slosh_Q0_20260509_204602_GEOREF_OSCRS_MEDIUM_ACTIVE_REAL_static.bag`.",
        "- Visual metric: `max(left_rel, center_rel, right_rel, 0)` after per-ruler static zeroing; reported values use a 5-frame rolling median.",
        "",
        "## Static Baseline",
        "",
        f"- left={baseline['h_left']:.3f} mm, center={baseline['h_center']:.3f} mm, right={baseline['h_right']:.3f} mm.",
        f"- static `/slosh/height` median={static_slosh:.3f} mm.",
        "",
        "## Main Judgment",
        "",
        f"- Verdict: **{verdict}**.",
        f"- OSCRS vs RAW visual p95 reduction: {vis_p95_red:.1f}%.",
        f"- OSCRS vs RAW visual max reduction: {vis_max_red:.1f}%.",
        f"- OSCRS vs RAW `/slosh/height` p95 reduction: {slosh_p95_red:.1f}%.",
        "",
        "## Condition Means",
        "",
        "| condition | n | visual_rms | visual_p95 | visual_max | slosh_p95 | valid_ratio | corr |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in cond_rows:
        lines.append(
            "| {condition} | {n} | {h_visual_rms_mean:.3f} | {h_visual_p95_mean:.3f} | "
            "{h_visual_max_mean:.3f} | {slosh_p95_mean:.3f} | {valid_ratio_mean:.3f} | {correlation_mean:.3f} |".format(**row)
        )
    lines.extend(
        [
            "",
            "## Per-Bag Metrics",
            "",
            "| bag | visual_p95 | visual_max | slosh_p95 | corr | dt_p95_ms | valid | conf |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in summary:
        lines.append(
            "| {bag} | {h_visual_p95:.3f} | {h_visual_max:.3f} | {slosh_p95:.3f} | "
            "{correlation:.3f} | {pair_dt_p95_ms:.1f} | {valid_ratio:.3f} | {confidence_mean:.3f} |".format(**row)
        )
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- The evidence is scoped to phase4 and to the current camera/calibration geometry.",
            "- Correlation is secondary here because `/slosh/height` is a model-derived peak estimate while the visual curve is a sparse three-ruler surface proxy.",
            "- Use the per-bag aligned CSV files for frame-level inspection of any outlier segment.",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    bag_dir = Path(args.bag_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    mod = load_red_module()
    calib = mod.load_calibration(Path(args.calibration))
    geom, rulers = mod.resolve_geometry(args, calib)
    if rulers is None:
        raise RuntimeError("three-ruler calibration is required")
    detector_args = build_detector_args(args)

    bag_paths = [bag_dir / name for name in DEFAULT_BAGS]
    missing = [str(p) for p in bag_paths if not p.exists()]
    if missing:
        raise RuntimeError("missing bags: " + ", ".join(missing))

    static_path = [p for p in bag_paths if "static" in p.name][0]
    print(f"[INFO] static baseline: {static_path.name}")
    static_visual = infer_visual_bag(static_path, args.topic, args.encoding, args.every, mod, geom, rulers, detector_args)
    baseline = {
        "h_left": float(np.median(finite([r["h_left"] for r in static_visual["rows"]]))),
        "h_center": float(np.median(finite([r["h_center"] for r in static_visual["rows"]]))),
        "h_right": float(np.median(finite([r["h_right"] for r in static_visual["rows"]]))),
    }
    static_slosh_t, static_slosh_mm = read_slosh_series(static_path)
    static_slosh_median = float(np.median(static_slosh_mm)) if static_slosh_mm else 0.0

    summary_rows: List[Dict] = []
    ts_dir = out_dir / "timeseries"
    aligned_dir = out_dir / "aligned"
    plot_dir = out_dir / "plots"

    for bag_path in bag_paths:
        if "static" in bag_path.name:
            continue
        print(f"[INFO] processing {bag_path.name}")
        visual = infer_visual_bag(bag_path, args.topic, args.encoding, args.every, mod, geom, rulers, detector_args)
        rows = visual["rows"]
        for row in rows:
            row.update(relative_values(row, baseline))
        smooth = rolling_median([r.get("h_visual_peak_rel_mm") for r in rows], args.smooth_frames)
        slosh_t, slosh_mm = read_slosh_series(bag_path)
        slosh_rel = [max(0.0, v - static_slosh_median) for v in slosh_mm]
        slosh_interp = interpolate(
            [float(r["t_rel_sec"]) for r in rows],
            slosh_t,
            slosh_rel,
            args.max_interpolation_gap_sec,
        )
        starts = read_start_times(bag_path, args.motion_threshold, args.motion_consecutive)

        write_timeseries(ts_dir / f"{bag_path.stem}_visual_timeseries.csv", rows, baseline, smooth)
        write_aligned(aligned_dir / f"{bag_path.stem}_visual_slosh_aligned.csv", rows, smooth, slosh_interp)
        plot_bag(plot_dir / f"{bag_path.stem}_visual_vs_slosh.png", bag_path.stem, rows, smooth, slosh_interp, starts["analysis_start_sec"])

        summary_rows.append(
            summarize_bag(
                bag_path,
                rows,
                smooth,
                slosh_t,
                slosh_mm,
                slosh_interp,
                static_slosh_median,
                starts,
            )
        )

    cond_rows = condition_summary(summary_rows)
    write_csv(out_dir / "phase4_visual_summary.csv", summary_rows)
    write_csv(out_dir / "phase4_condition_summary.csv", cond_rows)
    write_report(out_dir / "phase4_visual_report.md", summary_rows, cond_rows, baseline, static_slosh_median)
    print(f"[OK] wrote summary: {out_dir / 'phase4_visual_summary.csv'}")
    print(f"[OK] wrote report:  {out_dir / 'phase4_visual_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
