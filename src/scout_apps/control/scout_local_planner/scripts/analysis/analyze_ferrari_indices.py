#!/usr/bin/env python3
"""analyze_ferrari_indices.py

按 Ferrari et al., RA-L 2026 论文 §IV 的统一指标口径，对每个实物 bag 输出：
  - gamma_model_pct  : 模型曲线 vs RGB 视觉曲线的归一化时间积分偏差 (Eq.25)
  - rmse_mm          : 模型 - RGB 的 RMSE
  - corr             : 模型与 RGB 的 Pearson 相关
  - U_p95_mm         : max(0, RGB_p95 - model_p95)，模型低估幅度
  - U_max_mm         : max(0, RGB_max - model_max)
  - 模型 / RGB 的 p95 / RMS / peak (mm)

可选：通过 --baseline-bag 指定对照 bag 计算 gamma_opt_pct (Eq.26):
  gamma_opt = 100 * (eta_max_baseline - eta_max_opt) / eta_max_baseline

与 docs/重要文档/红色液体视觉验证固定流程.md §7.1.2 / §8 的指标定义一致。

与既有 compute_ferrari_indices.py (2026-05-08) 的差别：
  - 支持 --bag-dir + --glob 批量处理 n=3
  - 用 /mpc_status + /terminal/mode 切 TRACKING→first_terminal 窗口（默认排除 terminal 前 1s）
  - RGB 默认使用 max(left, center, right)，对应 maximum wall-rise 口径
  - 完整给 RMSE / corr / U_p95 / U_max（对应流程文档 §8.3）
  - CSV 列名与 red_liquid_infer_from_bag.py 当前输出对齐（h_mm_smooth_corr）

数据源：
  - /slosh/height (std_msgs/Float32, 单位 m) 来自实物 bag
  - red_liquid_infer_from_bag.py 的 <bag stem>_red_top.csv（单位 mm）

时间配对：以 RGB CSV 行为锚点，对每个 t_rgb 找最近的 /slosh/height 样本；
gap > --pair-max-gap-s (默认 0.15s，对应流程文档 §8.6) 记为 NaN，不参与积分 / corr。

用法:
  python3 analyze_ferrari_indices.py \\
      --bag-dir /data/a/slosh_bags/real/20260522_fixed_path_cost_d200 \\
      --glob 'slosh_*_d200_*.bag' \\
      --red-infer-dir /data/a/slosh_bags/real/20260522_fixed_path_cost_d200/red_visual_analysis_20260522/red_infer \\
      --out-csv /data/a/slosh_bags/real/20260522_fixed_path_cost_d200/red_visual_analysis_20260522/ferrari_indices.csv \\
      --baseline-bag slosh_Q0_*_P2_s_curve_C_d200_run01.bag
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rosbag


SLOSH_TOPIC = "/slosh/height"
MPC_STATUS_TOPIC = "/mpc_status"
TERMINAL_MODE_TOPIC = "/terminal/mode"
RGB_TIME_COL = "stamp_sec"
RGB_SMOOTH_COL = "h_mm_smooth_corr"
RGB_LCR_COLS = ("h_mm_left", "h_mm_center", "h_mm_right")


def detect_tracking_window(bag_path: Path,
                           window: str,
                           terminal_exclusion_s: float,
                           tracking_extension_s: float
                           ) -> Tuple[Optional[float], Optional[float], Optional[float], float]:
    """Return (t_track_start, t_window_end, t0_wall) in wall-clock seconds.

    t_track_start = first /mpc_status == "TRACKING"
    t_terminal   = first /terminal/mode not in (IDLE/NONE/"")
    main:     t_window_end = t_terminal - terminal_exclusion_s
    residual: t_window_end = t_terminal + tracking_extension_s
    t0_wall       = bag.get_start_time()
    """
    t_track: Optional[float] = None
    t_terminal: Optional[float] = None
    with rosbag.Bag(str(bag_path)) as bag:
        t0 = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(
                topics=[MPC_STATUS_TOPIC, TERMINAL_MODE_TOPIC]):
            data = str(getattr(msg, "data", ""))
            t = stamp.to_sec()
            if topic == MPC_STATUS_TOPIC and t_track is None and data == "TRACKING":
                t_track = t
            elif (topic == TERMINAL_MODE_TOPIC and t_terminal is None
                  and data not in ("IDLE", "NONE", "")):
                t_terminal = t
            if t_track is not None and t_terminal is not None:
                break
    t_window_end: Optional[float] = None
    if t_terminal is not None and window == "main":
        t_window_end = max(t_track if t_track is not None else t_terminal,
                           t_terminal - terminal_exclusion_s)
    elif t_terminal is not None:
        t_window_end = t_terminal + tracking_extension_s
    return t_track, t_terminal, t_window_end, t0


def read_slosh_height(bag_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    times: List[float] = []
    heights_mm: List[float] = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, stamp in bag.read_messages(topics=[SLOSH_TOPIC]):
            times.append(stamp.to_sec())
            heights_mm.append(abs(float(getattr(msg, "data", 0.0))) * 1000.0)
    return np.asarray(times, dtype=float), np.asarray(heights_mm, dtype=float)


def _parse_float(row: Dict[str, str], col: str) -> Optional[float]:
    raw = (row.get(col) or "").strip()
    if not raw:
        return None
    try:
        val = float(raw)
    except ValueError:
        return None
    return val if math.isfinite(val) else None


def read_rgb_csv(csv_path: Path, rgb_height_mode: str) -> Tuple[np.ndarray, np.ndarray]:
    times: List[float] = []
    vals: List[float] = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"{csv_path}: empty CSV")
        if RGB_TIME_COL not in reader.fieldnames:
            raise RuntimeError(f"{csv_path}: missing required column {RGB_TIME_COL!r}")
        if rgb_height_mode == "max_lcr":
            missing = [c for c in RGB_LCR_COLS if c not in reader.fieldnames]
            if missing:
                raise RuntimeError(f"{csv_path}: missing max_lcr columns {missing}")
        elif RGB_SMOOTH_COL not in reader.fieldnames:
            raise RuntimeError(f"{csv_path}: missing required column {RGB_SMOOTH_COL!r}")
        for row in reader:
            t = _parse_float(row, RGB_TIME_COL)
            if t is None:
                continue
            if rgb_height_mode == "max_lcr":
                candidates = [_parse_float(row, col) for col in RGB_LCR_COLS]
                finite = [abs(v) for v in candidates if v is not None]
                if not finite:
                    continue
                v = max(finite)
            else:
                v = _parse_float(row, RGB_SMOOTH_COL)
                if v is None:
                    continue
                v = abs(v)
            times.append(t)
            vals.append(v)
    return np.asarray(times, dtype=float), np.asarray(vals, dtype=float)


def pair_in_window(t_rgb: np.ndarray, v_rgb: np.ndarray,
                   t_mdl: np.ndarray, v_mdl: np.ndarray,
                   t_lo: float, t_hi: float,
                   max_gap_s: float
                   ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """对每个 t_rgb 在窗内找最近的 model 样本，gap>max_gap_s 丢弃。

    Returns:
        t_paired, h_rgb_paired_mm, h_model_paired_mm
    """
    mask_rgb = (t_rgb >= t_lo) & (t_rgb <= t_hi)
    t_rgb_w = t_rgb[mask_rgb]
    v_rgb_w = v_rgb[mask_rgb]
    if t_mdl.size == 0 or t_rgb_w.size == 0:
        return np.empty(0), np.empty(0), np.empty(0)

    idx_right = np.clip(np.searchsorted(t_mdl, t_rgb_w), 0, t_mdl.size - 1)
    idx_left = np.clip(idx_right - 1, 0, t_mdl.size - 1)
    gap_right = np.abs(t_mdl[idx_right] - t_rgb_w)
    gap_left = np.abs(t_mdl[idx_left] - t_rgb_w)
    use_left = gap_left < gap_right
    best_idx = np.where(use_left, idx_left, idx_right)
    best_gap = np.where(use_left, gap_left, gap_right)

    keep = best_gap <= max_gap_s
    return t_rgb_w[keep], v_rgb_w[keep], v_mdl[best_idx[keep]]


def compute_metrics(t_rgb: np.ndarray, h_rgb_mm: np.ndarray, h_mdl_mm: np.ndarray
                    ) -> Dict[str, float]:
    """Ferrari Eq.25 + RGB 流程文档 §8.3 全套指标。"""
    out: Dict[str, float] = {
        "paired_samples": int(h_rgb_mm.size),
        "rgb_p95_mm": float(np.percentile(h_rgb_mm, 95)) if h_rgb_mm.size else float("nan"),
        "rgb_rms_mm": float(np.sqrt(np.mean(h_rgb_mm ** 2))) if h_rgb_mm.size else float("nan"),
        "rgb_peak_mm": float(np.max(h_rgb_mm)) if h_rgb_mm.size else float("nan"),
        "model_p95_mm": float(np.percentile(h_mdl_mm, 95)) if h_mdl_mm.size else float("nan"),
        "model_rms_mm": float(np.sqrt(np.mean(h_mdl_mm ** 2))) if h_mdl_mm.size else float("nan"),
        "model_peak_mm": float(np.max(h_mdl_mm)) if h_mdl_mm.size else float("nan"),
    }
    if h_rgb_mm.size < 8:
        out.update(dict(
            gamma_model_pct=float("nan"),
            rmse_mm=float("nan"),
            corr=float("nan"),
            U_p95_mm=float("nan"),
            U_max_mm=float("nan"),
        ))
        return out

    diff = h_mdl_mm - h_rgb_mm
    int_abs_diff = float(np.trapz(np.abs(diff), t_rgb))
    int_abs_model = float(np.trapz(np.abs(h_mdl_mm), t_rgb))
    out["gamma_model_pct"] = (100.0 * int_abs_diff / int_abs_model
                              if int_abs_model > 1e-9 else float("nan"))
    out["rmse_mm"] = float(np.sqrt(np.mean(diff ** 2)))
    out["corr"] = (float(np.corrcoef(h_mdl_mm, h_rgb_mm)[0, 1])
                   if np.std(h_mdl_mm) > 1e-9 and np.std(h_rgb_mm) > 1e-9
                   else float("nan"))
    out["U_p95_mm"] = max(0.0, out["rgb_p95_mm"] - out["model_p95_mm"])
    out["U_max_mm"] = max(0.0, out["rgb_peak_mm"] - out["model_peak_mm"])
    return out


def find_rgb_csv(red_infer_dir: Path, bag_path: Path) -> Optional[Path]:
    """Locate <bag stem>_red_top.csv under red_infer_dir (recursive)."""
    target = f"{bag_path.stem}_red_top.csv"
    direct = red_infer_dir / target
    if direct.exists():
        return direct
    for candidate in red_infer_dir.rglob(target):
        return candidate
    return None


def process_bag(bag_path: Path, red_infer_dir: Path,
                window: str, terminal_exclusion_s: float,
                tracking_extension_s: float, pair_max_gap_s: float,
                rgb_height_mode: str
                ) -> Optional[Dict[str, float]]:
    csv_path = find_rgb_csv(red_infer_dir, bag_path)
    if csv_path is None:
        sys.stderr.write(f"[warn] no RGB CSV for {bag_path.name}; skip\n")
        return None

    t_track, t_terminal, t_end, t0 = detect_tracking_window(
        bag_path, window, terminal_exclusion_s, tracking_extension_s)
    if t_track is None or t_terminal is None or t_end is None:
        sys.stderr.write(
            f"[warn] {bag_path.name}: tracking_start={t_track} first_terminal={t_terminal}; skip\n")
        return None

    t_mdl, h_mdl = read_slosh_height(bag_path)
    t_rgb, h_rgb = read_rgb_csv(csv_path, rgb_height_mode)
    t_p, h_rgb_p, h_mdl_p = pair_in_window(
        t_rgb, h_rgb, t_mdl, h_mdl, t_track, t_end, pair_max_gap_s)

    metrics = compute_metrics(t_p, h_rgb_p, h_mdl_p)
    metrics.update(dict(
        bag=bag_path.name,
        rgb_csv=csv_path.name,
        tracking_start_s=float(t_track - t0),
        first_terminal_s=float(t_terminal - t0),
        window=str(window),
        rgb_height_mode=str(rgb_height_mode),
        terminal_exclusion_s=float(terminal_exclusion_s),
        window_end_s=float(t_end - t0),
        tracking_extension_s=float(tracking_extension_s),
        pair_max_gap_s=float(pair_max_gap_s),
    ))
    return metrics


def append_gamma_opt(rows: List[Dict[str, float]], baseline_bag: Optional[str]) -> None:
    if not baseline_bag:
        for r in rows:
            r["gamma_opt_pct"] = float("nan")
        return
    base = next((r for r in rows if r["bag"] == baseline_bag), None)
    if base is None or not math.isfinite(base.get("rgb_peak_mm", float("nan"))):
        sys.stderr.write(
            f"[warn] baseline {baseline_bag!r} not found / rgb_peak NaN; gamma_opt skipped\n")
        for r in rows:
            r["gamma_opt_pct"] = float("nan")
        return
    base_peak = base["rgb_peak_mm"]
    for r in rows:
        peak = r.get("rgb_peak_mm", float("nan"))
        r["gamma_opt_pct"] = (100.0 * (base_peak - peak) / base_peak
                              if math.isfinite(peak) and base_peak > 1e-9
                              else float("nan"))


OUT_COLUMNS = [
    "bag", "rgb_csv",
    "tracking_start_s", "first_terminal_s", "window_end_s",
    "window", "rgb_height_mode", "terminal_exclusion_s",
    "tracking_extension_s", "pair_max_gap_s",
    "paired_samples",
    "rgb_p95_mm", "rgb_rms_mm", "rgb_peak_mm",
    "model_p95_mm", "model_rms_mm", "model_peak_mm",
    "gamma_model_pct", "rmse_mm", "corr",
    "U_p95_mm", "U_max_mm",
    "gamma_opt_pct",
]


def write_csv(out_csv: Path, rows: List[Dict[str, float]]) -> None:
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=OUT_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in OUT_COLUMNS})


def _fmt(x, fmt: str) -> str:
    if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
        return "n/a"
    try:
        return fmt.format(x)
    except (TypeError, ValueError):
        return str(x)


def print_summary(rows: List[Dict[str, float]]) -> None:
    if not rows:
        print("[ferrari] no rows")
        return
    print(f"{'bag':<48} {'paired':>7} {'γ_model%':>10} {'rmse_mm':>8} "
          f"{'corr':>6} {'U_p95':>6} {'U_max':>6} {'γ_opt%':>9}")
    print("-" * 108)
    for r in rows:
        print(
            "{bag:<48.48} {paired:>7d} {gm:>10} {rm:>8} {co:>6} "
            "{u95:>6} {umx:>6} {go:>9}".format(
                bag=r["bag"],
                paired=int(r["paired_samples"]),
                gm=_fmt(r.get("gamma_model_pct"), "{:.2f}"),
                rm=_fmt(r.get("rmse_mm"), "{:.3f}"),
                co=_fmt(r.get("corr"), "{:.3f}"),
                u95=_fmt(r.get("U_p95_mm"), "{:.3f}"),
                umx=_fmt(r.get("U_max_mm"), "{:.3f}"),
                go=_fmt(r.get("gamma_opt_pct"), "{:.2f}"),
            ))


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", action="append", help="单包路径（可重复）")
    src.add_argument("--bag-dir", help="bag 目录")
    ap.add_argument("--glob", default="*.bag", help="--bag-dir 下 glob (default: *.bag)")
    ap.add_argument("--red-infer-dir", required=True,
                    help="red_liquid_infer_from_bag.py 输出 <stem>_red_top.csv 目录（递归查找）")
    ap.add_argument("--out-csv", required=True, help="输出 CSV 路径")
    ap.add_argument("--baseline-bag", default="",
                    help="可选：作为 gamma_opt 基准的 bag 文件名（仅文件名，不含目录）")
    ap.add_argument("--rgb-height-mode", default="max_lcr",
                    choices=["max_lcr", "smooth_corr"],
                    help="RGB 高度口径: max_lcr=max(left,center,right), "
                         "smooth_corr=h_mm_smooth_corr (default: max_lcr)")
    ap.add_argument("--window", default="main", choices=["main", "residual"],
                    help="main=TRACKING 到 first terminal 前；residual=first terminal 后扩展 "
                         "(default: main)")
    ap.add_argument("--terminal-exclusion-s", type=float, default=1.0,
                    help="--window main 时从 first terminal 往前排除的秒数 (default: 1.0)")
    ap.add_argument("--tracking-extension-s", type=float, default=2.0,
                    help="--window residual 时 terminal 后再加多少秒覆盖残振")
    ap.add_argument("--pair-max-gap-s", type=float, default=0.15,
                    help="model-RGB 时间配对最大 gap (RGB 流程 §8.6)")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    out_csv = Path(args.out_csv)
    red_infer_dir = Path(args.red_infer_dir)
    if not red_infer_dir.exists():
        sys.stderr.write(f"[err] red-infer-dir not found: {red_infer_dir}\n")
        return 2

    if args.bag:
        bag_paths = [Path(p) for p in args.bag]
    else:
        bd = Path(args.bag_dir)
        if not bd.exists():
            sys.stderr.write(f"[err] bag-dir not found: {bd}\n")
            return 2
        bag_paths = sorted(bd.glob(args.glob))
    if not bag_paths:
        sys.stderr.write("[err] no bag matched\n")
        return 2

    rows: List[Dict[str, float]] = []
    for bp in bag_paths:
        try:
            r = process_bag(bp, red_infer_dir,
                            args.window, args.terminal_exclusion_s,
                            args.tracking_extension_s, args.pair_max_gap_s,
                            args.rgb_height_mode)
        except Exception as exc:
            sys.stderr.write(f"[err] {bp.name}: {exc}\n")
            continue
        if r is not None:
            rows.append(r)

    append_gamma_opt(rows, args.baseline_bag or None)
    write_csv(out_csv, rows)
    print_summary(rows)
    print(f"\n[ferrari] wrote {len(rows)} rows -> {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
