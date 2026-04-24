#!/usr/bin/env python3
"""Batch compare grouped red-liquid visual curves and /slosh/height curves.

This script reuses the current red-liquid three-ruler pipeline and generates:
1. grouped visual liquid comparison plots using h_mm_smooth_corr and max(L,C,R)
2. grouped max(L,C,R)-only comparison plots
3. grouped /slosh/height comparison plots from bag topics
4. per-bag CSV caches for the visual pipeline

Designed for the 2026-04-22 red-liquid real-bag comparison sets.
"""

import argparse
import csv
import importlib.util
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import rosbag
from cv_bridge import CvBridge, CvBridgeError


SCRIPT_DIR = Path(__file__).resolve().parent
RED_INFER_PATH = SCRIPT_DIR / "red_liquid_infer_from_bag.py"


GROUPS: Dict[str, List[Tuple[str, str]]] = {
    "group1_main_compare": [
        ("NOM",   "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145046_block1_NOM.bag"),
        ("ISR",   "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145304_block1_ISR.bag"),
        ("FAS_Q5", "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145913_5.bag"),
        ("PROP2_Q5", "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145809_5.bag"),
    ],
    "group2_block2_q_sweep": [
        ("Q0",  "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150442_10block2_Q0.bag"),
        ("Q5",  "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150601_5.bag"),
        ("Q10", "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_150717_10.bag"),
    ],
    "group3_scheduler_compare": [
        ("PROP1_Q5", "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145652_5.bag"),
        ("FAS_Q5",   "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145913_5.bag"),
        ("PROP2_Q5", "/data/a/slosh_bags/real/0422/slosh_Q0_20260422_145809_5.bag"),
    ],
}


COLORS = [
    "#4E79A7", "#F28E2B", "#E15759", "#76B7B2", "#59A14F", "#EDC948",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot grouped red-liquid comparisons.")
    p.add_argument("--calibration", required=True, help="Three-ruler calibration YAML.")
    p.add_argument("--out-dir", required=True, help="Output directory.")
    p.add_argument("--topic", default="/camera/color/image_raw", help="RGB topic.")
    p.add_argument("--encoding", default="bgr8")
    p.add_argument("--zero-correction-frames", type=int, default=30)
    p.add_argument("--smooth-frames", type=int, default=5)
    p.add_argument("--hue1-low", type=int, default=0)
    p.add_argument("--hue1-high", type=int, default=11)
    p.add_argument("--hue2-low", type=int, default=173)
    p.add_argument("--hue2-high", type=int, default=179)
    p.add_argument("--sat-min", type=int, default=107)
    p.add_argument("--val-min", type=int, default=99)
    p.add_argument("--morph-kernel", type=int, default=5)
    p.add_argument("--top-boundary-quantile", type=float, default=0.2)
    p.add_argument("--min-valid-column-fraction", type=float, default=0.15)
    p.add_argument("--bottom-touch-rows", type=int, default=15)
    p.add_argument("--min-component-area", type=int, default=30)
    p.add_argument("--motion-threshold", type=float, default=0.03,
                   help="Absolute odom linear.x threshold for motion-start alignment. Default: 0.03 m/s")
    p.add_argument("--motion-consecutive", type=int, default=3,
                   help="Required consecutive odom samples above threshold for motion-start. Default: 3")
    p.add_argument("--group", action="append", default=[],
                   help="Optional subset of groups: group1_main_compare / group2_block2_q_sweep / group3_scheduler_compare")
    return p.parse_args()


def load_red_module():
    spec = importlib.util.spec_from_file_location("red_liquid_infer_from_bag", RED_INFER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load module from {RED_INFER_PATH}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def build_infer_args(mod, cli_args):
    class Args:
        pass
    a = Args()
    a.roi_x = None
    a.roi_y = None
    a.roi_w = None
    a.roi_h = None
    a.tube_left = None
    a.tube_right = None
    a.rotation_deg = None
    a.center_col_fraction = 0.6
    a.hue1_low = cli_args.hue1_low
    a.hue1_high = cli_args.hue1_high
    a.hue2_low = cli_args.hue2_low
    a.hue2_high = cli_args.hue2_high
    a.sat_min = cli_args.sat_min
    a.val_min = cli_args.val_min
    a.morph_kernel = cli_args.morph_kernel
    a.top_boundary_quantile = cli_args.top_boundary_quantile
    a.min_valid_column_fraction = cli_args.min_valid_column_fraction
    a.bottom_touch_rows = cli_args.bottom_touch_rows
    a.min_component_area = cli_args.min_component_area
    return a


def rolling_median(series: List[Optional[float]], window: int) -> List[Optional[float]]:
    if window <= 1:
        return list(series)
    out: List[Optional[float]] = []
    half = window // 2
    for i in range(len(series)):
        seg = [v for v in series[max(0, i - half):min(len(series), i + half + 1)] if v is not None]
        out.append(float(np.median(seg)) if seg else None)
    return out


def read_alignment_times(
    bag_path: Path,
    motion_threshold: float,
    motion_consecutive: int,
) -> Dict[str, float]:
    """Return alignment anchors in seconds relative to bag start."""
    tracking_t: Optional[float] = None
    motion_t: Optional[float] = None

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        consec = 0
        for topic, msg, stamp in bag.read_messages(topics=["/mpc_status", "/odom"]):
            rel_t = stamp.to_sec() - bag_start
            if topic == "/mpc_status" and tracking_t is None:
                if str(getattr(msg, "data", "")).strip() == "TRACKING":
                    tracking_t = rel_t
            elif topic == "/odom" and motion_t is None:
                try:
                    v = float(msg.twist.twist.linear.x)
                except Exception:
                    v = 0.0
                if abs(v) > motion_threshold:
                    consec += 1
                    if consec >= max(1, motion_consecutive):
                        motion_t = rel_t
                else:
                    consec = 0
            if tracking_t is not None and motion_t is not None:
                break

    return {
        "bag_start": 0.0,
        "tracking_start": tracking_t if tracking_t is not None else 0.0,
        "motion_start": motion_t if motion_t is not None else 0.0,
    }


def infer_bag_visual(
    bag_path: Path,
    geom: Dict,
    rulers: Optional[Dict],
    mod,
    infer_args,
    topic: str,
    encoding: str,
    zero_correction_frames: int,
    smooth_frames: int,
    align_times: Dict[str, float],
):
    bridge = CvBridge()
    frame_indices: List[int] = []
    rel_t: List[float] = []
    h_mms_all: List[List[Optional[float]]] = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        filtered_index = 0
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            try:
                image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            except CvBridgeError:
                filtered_index += 1
                continue
            _, h_mms, _, _, _ = mod.detect_red_liquid(image, geom, rulers, infer_args)
            frame_indices.append(filtered_index)
            rel_t.append(stamp.to_sec() - bag_start)
            h_mms_all.append(h_mms)
            filtered_index += 1

    h_final = [mod.h_mm_final(hms) for hms in h_mms_all]
    zero_pool = [h for h in h_final[:zero_correction_frames] if h is not None] if zero_correction_frames > 0 else []
    h0 = float(np.median(zero_pool)) if zero_pool else 0.0
    h_corr = [(h - h0) if h is not None else None for h in h_final]
    h_smooth_corr = rolling_median(h_corr, smooth_frames)
    h_max = [float(max([x for x in hms if x is not None])) if any(x is not None for x in hms) else None for hms in h_mms_all]

    return {
        "frame_indices": frame_indices,
        "rel_t": rel_t,
        "h_left": [h[0] for h in h_mms_all],
        "h_center": [h[1] for h in h_mms_all],
        "h_right": [h[2] for h in h_mms_all],
        "h_final": h_final,
        "h_corr": h_corr,
        "h_smooth_corr": h_smooth_corr,
        "h_max_lcr": h_max,
        "h0": h0,
        "align_times": align_times,
    }


def read_slosh_height_mm(bag_path: Path, align_times: Dict[str, float]):
    rel_t: List[float] = []
    vals_mm: List[float] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=["/slosh/height"]):
            rel_t.append(stamp.to_sec() - bag_start)
            vals_mm.append(float(msg.data) * 1000.0)
    return {"rel_t": rel_t, "vals_mm": vals_mm, "align_times": align_times}


def write_visual_csv(path: Path, result: Dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "frame_index", "t_rel_sec",
            "h_left", "h_center", "h_right",
            "h_final", "h_corr", "h_smooth_corr", "h_max_lcr",
        ])
        for i in range(len(result["frame_indices"])):
            def fmt(v):
                return "" if v is None else f"{float(v):.6f}"
            w.writerow([
                result["frame_indices"][i],
                f"{result['rel_t'][i]:.6f}",
                fmt(result["h_left"][i]),
                fmt(result["h_center"][i]),
                fmt(result["h_right"][i]),
                fmt(result["h_final"][i]),
                fmt(result["h_corr"][i]),
                fmt(result["h_smooth_corr"][i]),
                fmt(result["h_max_lcr"][i]),
            ])


def aligned_xy(ts: List[float], vals: List[Optional[float]], t0: float) -> Tuple[List[float], List[float]]:
    xs: List[float] = []
    ys: List[float] = []
    for t, v in zip(ts, vals):
        if v is None:
            continue
        ta = t - t0
        if ta < 0.0:
            continue
        xs.append(ta)
        ys.append(float(v))
    return xs, ys


def plot_group_visual(group_name: str, items: List[Tuple[str, Dict]], out_path: Path, align_key: str):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    for idx, (label, result) in enumerate(items):
        color = COLORS[idx % len(COLORS)]
        t0 = float(result["align_times"].get(align_key, 0.0))
        xs1, ys1 = aligned_xy(result["rel_t"], result["h_smooth_corr"], t0)
        xs2, ys2 = aligned_xy(result["rel_t"], result["h_max_lcr"], t0)
        if xs2:
            ax.plot(xs2, ys2, linewidth=1.2, alpha=0.35, color=color, linestyle="--", label=f"{label} max(L,C,R)")
        if xs1:
            ax.plot(xs1, ys1, linewidth=2.2, color=color, label=f"{label} corr+smooth")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Liquid height (mm)")
    ax.set_title(f"{group_name}: visual red-liquid comparison ({align_key})")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, facecolor="white", edgecolor="white")
    plt.close(fig)


def plot_group_visual_max(group_name: str, items: List[Tuple[str, Dict]], out_path: Path, align_key: str):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    for idx, (label, result) in enumerate(items):
        color = COLORS[idx % len(COLORS)]
        t0 = float(result["align_times"].get(align_key, 0.0))
        xs, ys = aligned_xy(result["rel_t"], result["h_max_lcr"], t0)
        if xs:
            ax.plot(xs, ys, linewidth=2.0, color=color, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Liquid height (mm)")
    ax.set_title(f"{group_name}: max(L,C,R) comparison ({align_key})")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, facecolor="white", edgecolor="white")
    plt.close(fig)


def plot_group_slosh_height(group_name: str, items: List[Tuple[str, Dict]], out_path: Path, align_key: str):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    for idx, (label, result) in enumerate(items):
        color = COLORS[idx % len(COLORS)]
        t0 = float(result["align_times"].get(align_key, 0.0))
        xs, ys = aligned_xy(result["rel_t"], result["vals_mm"], t0)
        if not xs:
            continue
        ax.plot(xs, ys, linewidth=2.0, color=color, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("/slosh/height (mm)")
    ax.set_title(f"{group_name}: /slosh/height comparison ({align_key})")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, facecolor="white", edgecolor="white")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    mod = load_red_module()
    calib = mod.load_calibration(Path(args.calibration).expanduser().resolve())
    geom, rulers = mod.resolve_geometry(args, calib)
    infer_args = build_infer_args(mod, args)

    selected = args.group if args.group else list(GROUPS.keys())

    summary_rows = []
    for group_name in selected:
        group_items = GROUPS[group_name]
        visual_items = []
        slosh_items = []
        group_dir = out_dir / group_name
        group_dir.mkdir(parents=True, exist_ok=True)
        csv_dir = group_dir / "csv"
        csv_dir.mkdir(parents=True, exist_ok=True)

        for label, bag_str in group_items:
            bag_path = Path(bag_str)
            align_times = read_alignment_times(
                bag_path=bag_path,
                motion_threshold=args.motion_threshold,
                motion_consecutive=args.motion_consecutive,
            )
            visual = infer_bag_visual(
                bag_path=bag_path,
                geom=geom,
                rulers=rulers,
                mod=mod,
                infer_args=infer_args,
                topic=args.topic,
                encoding=args.encoding,
                zero_correction_frames=args.zero_correction_frames,
                smooth_frames=args.smooth_frames,
                align_times=align_times,
            )
            write_visual_csv(csv_dir / f"{bag_path.stem}_visual.csv", visual)
            visual_items.append((label, visual))

            slosh = read_slosh_height_mm(bag_path, align_times)
            slosh_items.append((label, slosh))

            valid_corr = [v for v in visual["h_corr"] if v is not None]
            valid_smooth = [v for v in visual["h_smooth_corr"] if v is not None]
            valid_max = [v for v in visual["h_max_lcr"] if v is not None]
            summary_rows.append({
                "group": group_name,
                "label": label,
                "bag": str(bag_path),
                "visual_peak_corr_mm": max(valid_corr) if valid_corr else None,
                "visual_peak_smooth_corr_mm": max(valid_smooth) if valid_smooth else None,
                "visual_peak_max_lcr_mm": max(valid_max) if valid_max else None,
                "slosh_height_peak_mm": max(slosh["vals_mm"]) if slosh["vals_mm"] else None,
                "tracking_start_sec": align_times["tracking_start"],
                "motion_start_sec": align_times["motion_start"],
            })

        for align_key, suffix in [
            ("bag_start", "bag_start"),
            ("tracking_start", "tracking_start"),
            ("motion_start", "motion_start"),
        ]:
            align_dir = group_dir / suffix
            align_dir.mkdir(parents=True, exist_ok=True)
            plot_group_visual(group_name, visual_items, align_dir / "visual_compare.png", align_key)
            plot_group_visual_max(group_name, visual_items, align_dir / "visual_max_compare.png", align_key)
            plot_group_slosh_height(group_name, slosh_items, align_dir / "slosh_height_compare.png", align_key)

    summary_csv = out_dir / "group_peak_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=[
            "group", "label", "bag",
            "visual_peak_corr_mm", "visual_peak_smooth_corr_mm",
            "visual_peak_max_lcr_mm", "slosh_height_peak_mm",
            "tracking_start_sec", "motion_start_sec",
        ])
        w.writeheader()
        w.writerows(summary_rows)

    print(f"[OK] output dir: {out_dir}")
    print(f"[OK] summary csv: {summary_csv}")
    for group_name in selected:
        print(f"[OK] csv dir: {out_dir / group_name / 'csv'}")
        for suffix in ["bag_start", "tracking_start", "motion_start"]:
            print(f"[OK] group plot: {out_dir / group_name / suffix / 'visual_compare.png'}")
            print(f"[OK] max plot: {out_dir / group_name / suffix / 'visual_max_compare.png'}")
            print(f"[OK] slosh plot: {out_dir / group_name / suffix / 'slosh_height_compare.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
