#!/usr/bin/env python3
"""Diagnose why /slosh/height peak is or is not reduced by MPC cost changes.

The script automates the peak replay checks used for Slosh-Priority MPC:

1. find /slosh/height peaks in TRACKING_PRE_TERMINAL and terminal-near windows;
2. identify whether the horizon maximum is at k=0 (current state) or future k>0;
3. report ax/ay/jerk context in the 0.5-1.0 s before the peak;
4. recommend whether to first fix excitation sources or add a slosh soft constraint;
5. keep terminal-near peaks separate from the main effect window.

It is intentionally offline-only and does not modify any control behavior.
"""

import argparse
import csv
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    import rosbag
except ImportError:
    print("ERROR: source ROS environment first.", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


IDLE_TERMINAL_MODES = {"", "NONE", "IDLE"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bags", nargs="*", help="Input bag files. If omitted, use --bag-dir/--glob.")
    p.add_argument("--bag-dir", default="", help="Directory containing bags.")
    p.add_argument("--glob", default="*.bag", help="Glob for --bag-dir. Default: *.bag")
    p.add_argument("--summary-csv", default="",
                   help="Optional fixed_path_cost_effect_summary.csv with tracking_start_s/first_terminal_s.")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--exclude-static", action="store_true")
    p.add_argument("--terminal-margin", type=float, default=0.5,
                   help="Last N seconds before terminal are reported as terminal-near. Default: 0.5")
    p.add_argument("--pre-windows", default="0.5,1.0",
                   help="Comma-separated pre-peak windows for ax/jerk diagnostics. Default: 0.5,1.0")
    p.add_argument("--plot", action="store_true", help="Save one replay plot per bag/window.")
    p.add_argument("--plot-window", type=float, default=2.0,
                   help="Seconds before/after peak for replay plots. Default: 2.0")
    return p.parse_args()


def condition_from_name(name: str) -> str:
    if "_G_" in name or "_G_slosh_axjerk_preview" in name:
        return "G"
    if "_F_" in name or "_F_slosh_axjerk" in name:
        return "F"
    if "_E_" in name or "_E_axjerk" in name:
        return "E"
    if "_D_" in name or "_D_run" in name:
        return "D"
    if "_C_" in name or "_C_run" in name:
        return "C"
    m = re.search(r"_Q(\d+)_", name)
    return f"Q{m.group(1)}" if m else "UNK"


def parse_windows(text: str) -> List[float]:
    out = []
    for part in text.split(","):
        part = part.strip()
        if part:
            out.append(float(part))
    return out or [0.5, 1.0]


def finite(value, default: float = float("nan")) -> float:
    try:
        v = float(value)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def nearest(ts: Sequence[float], vals: Sequence[float], t: float) -> float:
    if len(ts) == 0 or not math.isfinite(t):
        return float("nan")
    idx = int(np.searchsorted(ts, t))
    if idx <= 0:
        return float(vals[0])
    if idx >= len(ts):
        return float(vals[-1])
    return float(vals[idx] if abs(ts[idx] - t) < abs(ts[idx - 1] - t) else vals[idx - 1])


def values_between(ts: np.ndarray, vals: np.ndarray, t0: float, t1: float) -> np.ndarray:
    if len(ts) == 0:
        return np.asarray([], dtype=float)
    mask = (ts >= t0) & (ts <= t1) & np.isfinite(vals)
    return vals[mask]


def absmax_between(ts: np.ndarray, vals: np.ndarray, t0: float, t1: float) -> float:
    arr = values_between(ts, vals, t0, t1)
    return float(np.max(np.abs(arr))) if arr.size else float("nan")


def p95_abs(arr: np.ndarray) -> float:
    arr = np.asarray(arr, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.percentile(np.abs(arr), 95)) if arr.size else float("nan")


def rms(arr: np.ndarray) -> float:
    arr = np.asarray(arr, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(math.sqrt(np.mean(arr * arr))) if arr.size else float("nan")


def derivative(ts: np.ndarray, vals: np.ndarray) -> np.ndarray:
    if len(ts) < 3:
        return np.zeros_like(vals)
    return np.gradient(vals, ts, edge_order=1)


def first_tracking_start(status: List[Tuple[float, str]]) -> float:
    for t, value in status:
        if str(value).upper() == "TRACKING":
            return t
    return float("nan")


def first_terminal_time(modes: List[Tuple[float, str]]) -> float:
    for t, mode in modes:
        if str(mode).upper() not in IDLE_TERMINAL_MODES:
            return t
    return float("nan")


def load_summary(path: str) -> Dict[str, Tuple[float, float]]:
    if not path:
        return {}
    out = {}
    with Path(path).open("r", newline="") as f:
        for row in csv.DictReader(f):
            bag = row.get("bag", "")
            if bag:
                out[bag] = (finite(row.get("tracking_start_s")), finite(row.get("first_terminal_s")))
    return out


def collect_bags(args: argparse.Namespace) -> List[Path]:
    if args.bags:
        bags = [Path(p) for p in args.bags]
    elif args.bag_dir:
        bags = sorted(Path(args.bag_dir).glob(args.glob))
    else:
        raise SystemExit("provide bags or --bag-dir")
    if args.exclude_static:
        bags = [p for p in bags if "static" not in p.name.lower()]
    return bags


def read_bag(path: Path) -> Dict:
    series = {
        "height": ([], []),
        "odom_v": ([], []),
        "cmd_v": ([], []),
        "ref_ax": ([], []),
        "ref_ay": ([], []),
        "ref_jerk": ([], []),
        "pct_v": ([], []),
        "pct_slosh": ([], []),
        "horizon_hmax": ([], []),
        "horizon_kmax": ([], []),
        "horizon_axp95": ([], []),
        "horizon_ayp95": ([], []),
    }
    status: List[Tuple[float, str]] = []
    terminal_modes: List[Tuple[float, str]] = []

    topics = [
        "/slosh/height",
        "/odom",
        "/cmd_vel",
        "/reference/implied_ax",
        "/reference/implied_ay",
        "/reference/implied_jerk",
        "/mpc/cost_breakdown",
        "/mpc/slosh_horizon_summary",
        "/mpc_status",
        "/terminal/mode",
    ]
    with rosbag.Bag(str(path), "r") as bag:
        start_abs = bag.get_start_time()
        end_rel = bag.get_end_time() - start_abs
        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec() - start_abs
            if topic == "/slosh/height":
                series["height"][0].append(t)
                series["height"][1].append(abs(float(getattr(msg, "data", 0.0))) * 1000.0)
            elif topic == "/odom":
                series["odom_v"][0].append(t)
                series["odom_v"][1].append(float(msg.twist.twist.linear.x))
            elif topic == "/cmd_vel":
                series["cmd_v"][0].append(t)
                series["cmd_v"][1].append(float(msg.linear.x))
            elif topic == "/reference/implied_ax":
                series["ref_ax"][0].append(t)
                series["ref_ax"][1].append(float(getattr(msg, "data", 0.0)))
            elif topic == "/reference/implied_ay":
                series["ref_ay"][0].append(t)
                series["ref_ay"][1].append(float(getattr(msg, "data", 0.0)))
            elif topic == "/reference/implied_jerk":
                series["ref_jerk"][0].append(t)
                series["ref_jerk"][1].append(float(getattr(msg, "data", 0.0)))
            elif topic == "/mpc/cost_breakdown" and len(msg.data) >= 20:
                series["pct_v"][0].append(t)
                series["pct_v"][1].append(float(msg.data[13]))
                series["pct_slosh"][0].append(t)
                series["pct_slosh"][1].append(float(msg.data[19]))
            elif topic == "/mpc/slosh_horizon_summary" and len(msg.data) >= 13:
                series["horizon_hmax"][0].append(t)
                series["horizon_hmax"][1].append(float(msg.data[5]))
                series["horizon_kmax"][0].append(t)
                series["horizon_kmax"][1].append(float(msg.data[6]))
                series["horizon_axp95"][0].append(t)
                series["horizon_axp95"][1].append(float(msg.data[9]))
                series["horizon_ayp95"][0].append(t)
                series["horizon_ayp95"][1].append(float(msg.data[10]))
            elif topic == "/mpc_status":
                status.append((t, str(getattr(msg, "data", ""))))
            elif topic == "/terminal/mode":
                terminal_modes.append((t, str(getattr(msg, "data", ""))))

    arrays = {k: (np.asarray(t, dtype=float), np.asarray(v, dtype=float)) for k, (t, v) in series.items()}
    for base, deriv_name in (("odom_v", "odom_ax"), ("cmd_v", "cmd_ax")):
        ts, vals = arrays[base]
        arrays[deriv_name] = (ts, derivative(ts, vals))
    for base, deriv_name in (("odom_ax", "odom_jerk"), ("cmd_ax", "cmd_jerk")):
        ts, vals = arrays[base]
        arrays[deriv_name] = (ts, derivative(ts, vals))

    return {
        "start_abs": start_abs,
        "end_rel": end_rel,
        "series": arrays,
        "tracking_start": first_tracking_start(status),
        "terminal_time": first_terminal_time(terminal_modes),
    }


def peak_in_window(ts: np.ndarray, vals: np.ndarray, t0: float, t1: float) -> Tuple[float, float, float, float]:
    if not (math.isfinite(t0) and math.isfinite(t1)) or t1 <= t0 or len(ts) == 0:
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    mask = (ts >= t0) & (ts <= t1) & np.isfinite(vals)
    if not np.any(mask):
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    wts = ts[mask]
    wvals = vals[mask]
    idx = int(np.argmax(np.abs(wvals)))
    return float(wts[idx]), float(abs(wvals[idx])), p95_abs(wvals), rms(wvals)


def classify_k(k: float) -> str:
    if not math.isfinite(k):
        return "UNKNOWN"
    return "CURRENT_K0" if k <= 0.5 else "FUTURE_K_GT_0"


def diagnose_row(
    bag_name: str,
    condition: str,
    window_name: str,
    t0: float,
    t1: float,
    data: Dict,
    pre_windows: List[float],
    terminal_margin: float,
) -> Dict[str, object]:
    series = data["series"]
    ht, hv = series["height"]
    peak_t, peak_mm, p95_mm, rms_mm = peak_in_window(ht, hv, t0, t1)
    row: Dict[str, object] = {
        "bag": bag_name,
        "condition": condition,
        "window": window_name,
        "window_start_s": t0,
        "window_end_s": t1,
        "window_duration_s": t1 - t0 if math.isfinite(t0) and math.isfinite(t1) else float("nan"),
        "peak_t_s": peak_t,
        "peak_mm": peak_mm,
        "p95_mm": p95_mm,
        "rms_mm": rms_mm,
        "time_to_terminal_s": data["terminal_time"] - peak_t if math.isfinite(peak_t) else float("nan"),
        "is_terminal_near": int(math.isfinite(peak_t) and math.isfinite(data["terminal_time"]) and (data["terminal_time"] - peak_t) <= terminal_margin),
    }

    for key in ("odom_v", "cmd_v", "ref_ax", "ref_ay", "ref_jerk", "pct_v", "pct_slosh",
                "horizon_hmax", "horizon_kmax", "horizon_axp95", "horizon_ayp95"):
        ts, vals = series[key]
        row[f"{key}_at_peak"] = nearest(ts, vals, peak_t)

    kmax = finite(row["horizon_kmax_at_peak"])
    row["horizon_peak_class"] = classify_k(kmax)

    for win in pre_windows:
        suffix = str(win).replace(".", "p")
        p0 = peak_t - win if math.isfinite(peak_t) else float("nan")
        p1 = peak_t
        for key in ("ref_ax", "ref_ay", "ref_jerk", "odom_ax", "odom_jerk", "cmd_ax", "cmd_jerk"):
            ts, vals = series[key]
            row[f"{key}_absmax_pre_{suffix}s"] = absmax_between(ts, vals, p0, p1)

    if row["horizon_peak_class"] == "CURRENT_K0":
        row["recommendation"] = "FIX_PRE_PEAK_EXCITATION"
    elif row["horizon_peak_class"] == "FUTURE_K_GT_0":
        row["recommendation"] = "CONSIDER_SLOSH_SOFT_CONSTRAINT"
    else:
        row["recommendation"] = "INSUFFICIENT_HORIZON_DIAGNOSTICS"
    return row


def plot_replay(row: Dict[str, object], data: Dict, out_dir: Path, half_window: float) -> None:
    if plt is None:
        return
    peak_t = finite(row["peak_t_s"])
    if not math.isfinite(peak_t):
        return
    series = data["series"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    x0, x1 = peak_t - half_window, peak_t + half_window

    def draw(ax, key, label, scale=1.0):
        ts, vals = series[key]
        mask = (ts >= x0) & (ts <= x1)
        if np.any(mask):
            ax.plot(ts[mask], vals[mask] * scale, label=label)

    draw(axes[0], "height", "/slosh/height mm")
    draw(axes[0], "horizon_hmax", "horizon hmax mm")
    draw(axes[1], "odom_v", "odom_v")
    draw(axes[1], "cmd_v", "cmd_v")
    draw(axes[2], "ref_ax", "ref_ax")
    draw(axes[2], "odom_ax", "odom_ax")
    draw(axes[2], "cmd_ax", "cmd_ax")
    draw(axes[3], "ref_jerk", "ref_jerk")
    draw(axes[3], "pct_v", "pct_v")
    draw(axes[3], "pct_slosh", "pct_slosh")

    for ax in axes:
        ax.axvline(peak_t, color="r", linestyle="--", linewidth=1.0)
        ax.axvline(data["terminal_time"], color="k", linestyle=":", linewidth=1.0)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("bag-relative time (s)")
    fig.suptitle(f"{row['condition']} {row['window']} {row['bag']} peak={finite(row['peak_mm']):.3f}mm k={row['horizon_kmax_at_peak']}")
    fig.tight_layout()
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", f"{row['condition']}_{row['window']}_{Path(str(row['bag'])).stem}")
    fig.savefig(out_dir / f"{safe}_peak_replay.png", dpi=160)
    plt.close(fig)


def mean(rows: List[Dict[str, object]], key: str) -> float:
    vals = [finite(r.get(key)) for r in rows]
    vals = [v for v in vals if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else float("nan")


def pct(n: int, d: int) -> float:
    return 100.0 * n / d if d else 0.0


def write_report(out_dir: Path, rows: List[Dict[str, object]], group_rows: List[Dict[str, object]]) -> None:
    report = out_dir / "SLOSH_PEAK_DIAGNOSTIC_REPORT.md"
    pre_rows = [r for r in rows if r["window"] == "pre_terminal"]
    nonterminal_rows = [r for r in rows if r["window"] == "main_excluding_terminal_margin"]
    terminal_rows = [r for r in rows if r["window"] == "terminal_near"]
    pre_count = len(pre_rows)
    current_count = sum(1 for r in pre_rows if r["horizon_peak_class"] == "CURRENT_K0")
    future_count = sum(1 for r in pre_rows if r["horizon_peak_class"] == "FUTURE_K_GT_0")
    terminal_count = sum(1 for r in pre_rows if finite(r.get("is_terminal_near")) > 0.5)

    with report.open("w", encoding="utf-8") as f:
        f.write("# Slosh Peak Context Diagnostic Report\n\n")
        f.write("目的：判断 `/slosh/height` peak 没有随 slosh cost 降低时，问题应先归因于当前状态峰值、未来 horizon 峰值、还是 terminal/ax/jerk 源头。\n\n")
        f.write("## 总体判读\n\n")
        f.write("```text\n")
        f.write(f"pre_terminal peaks: {pre_count}\n")
        f.write(f"CURRENT_K0: {current_count} ({pct(current_count, pre_count):.1f}%)\n")
        f.write(f"FUTURE_K_GT_0: {future_count} ({pct(future_count, pre_count):.1f}%)\n")
        f.write(f"terminal_near within pre_terminal: {terminal_count} ({pct(terminal_count, pre_count):.1f}%)\n")
        f.write("```\n\n")
        if current_count >= future_count:
            f.write("结论：多数 peak 是 `k=0` 当前状态峰值。优先治理 peak 前 0.5-1.0s 的 ax/jerk 激励源头，不应首先加无 slack 的 slosh hard constraint。\n\n")
        else:
            f.write("结论：多数 peak 是 horizon 未来峰值。可以考虑加入 slosh height soft constraint / slack penalty，但仍需保留可行性。\n\n")
        if terminal_count:
            f.write("注意：存在 terminal-near peak。terminal 附近 peak 应单独统计，不混入主效果窗口。\n\n")

        f.write("## 分组摘要\n\n")
        f.write("| condition | window | n | peak mean mm | p95 mean mm | CURRENT_K0 | FUTURE_K>0 | terminal-near | ax pre0.5 | jerk pre0.5 | pct_v@peak | pct_slosh@peak |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for r in group_rows:
            f.write(
                f"| {r['condition']} | {r['window']} | {int(r['n'])} | "
                f"{finite(r['peak_mean_mm']):.3f} | {finite(r['p95_mean_mm']):.3f} | "
                f"{int(r['current_k0_count'])} | {int(r['future_k_count'])} | {int(r['terminal_near_count'])} | "
                f"{finite(r['ref_ax_absmax_pre_0p5s_mean']):.3f} | {finite(r['ref_jerk_absmax_pre_0p5s_mean']):.3f} | "
                f"{finite(r['pct_v_at_peak_mean']):.1f} | {finite(r['pct_slosh_at_peak_mean']):.1f} |\n"
            )

        f.write("\n## 调试顺序\n\n")
        f.write("```text\n")
        f.write("1. 先看 peak_context.csv 中 horizon_peak_class。\n")
        f.write("2. CURRENT_K0 占多数：查 peak 前 ref/odom/cmd ax 和 jerk，先治激励源头。\n")
        f.write("3. FUTURE_K_GT_0 占多数：考虑 slosh height soft constraint + slack。\n")
        f.write("4. terminal_near=1：从论文主效果窗口排除，单独看 terminal 停车策略。\n")
        f.write("5. 不要用 /slosh/height peak 单独替代 RGB 真实液面结论。\n")
        f.write("```\n")


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_dir = out_dir / "plots"
    if args.plot:
        plot_dir.mkdir(parents=True, exist_ok=True)
    bags = collect_bags(args)
    summary = load_summary(args.summary_csv)
    pre_windows = parse_windows(args.pre_windows)

    rows: List[Dict[str, object]] = []
    for bag in bags:
        data = read_bag(bag)
        t_start, t_terminal = summary.get(bag.name, (data["tracking_start"], data["terminal_time"]))
        if not math.isfinite(t_start):
            t_start = 0.0
        if not math.isfinite(t_terminal):
            t_terminal = data["end_rel"]
        condition = condition_from_name(bag.name)
        windows = [
            ("pre_terminal", t_start, t_terminal),
            ("main_excluding_terminal_margin", t_start, max(t_start, t_terminal - args.terminal_margin)),
            ("terminal_near", max(t_start, t_terminal - args.terminal_margin), t_terminal),
        ]
        for window_name, w0, w1 in windows:
            row = diagnose_row(bag.name, condition, window_name, w0, w1, data, pre_windows, args.terminal_margin)
            rows.append(row)
            if args.plot:
                plot_replay(row, data, plot_dir, args.plot_window)

    fieldnames = list(rows[0].keys()) if rows else []
    csv_path = out_dir / "slosh_peak_context.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    grouped: Dict[Tuple[str, str], List[Dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["condition"]), str(row["window"]))].append(row)
    group_rows: List[Dict[str, object]] = []
    for (condition, window), rs in sorted(grouped.items()):
        cls = Counter(str(r["horizon_peak_class"]) for r in rs)
        group_rows.append({
            "condition": condition,
            "window": window,
            "n": len(rs),
            "peak_mean_mm": mean(rs, "peak_mm"),
            "p95_mean_mm": mean(rs, "p95_mm"),
            "rms_mean_mm": mean(rs, "rms_mm"),
            "current_k0_count": cls["CURRENT_K0"],
            "future_k_count": cls["FUTURE_K_GT_0"],
            "unknown_count": cls["UNKNOWN"],
            "terminal_near_count": sum(1 for r in rs if finite(r.get("is_terminal_near")) > 0.5),
            "ref_ax_absmax_pre_0p5s_mean": mean(rs, "ref_ax_absmax_pre_0p5s"),
            "ref_jerk_absmax_pre_0p5s_mean": mean(rs, "ref_jerk_absmax_pre_0p5s"),
            "odom_ax_absmax_pre_0p5s_mean": mean(rs, "odom_ax_absmax_pre_0p5s"),
            "odom_jerk_absmax_pre_0p5s_mean": mean(rs, "odom_jerk_absmax_pre_0p5s"),
            "pct_v_at_peak_mean": mean(rs, "pct_v_at_peak"),
            "pct_slosh_at_peak_mean": mean(rs, "pct_slosh_at_peak"),
            "horizon_hmax_at_peak_mean_mm": mean(rs, "horizon_hmax_at_peak"),
            "time_to_terminal_mean_s": mean(rs, "time_to_terminal_s"),
        })
    group_csv = out_dir / "slosh_peak_group_summary.csv"
    with group_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(group_rows[0].keys()) if group_rows else [])
        writer.writeheader()
        writer.writerows(group_rows)

    write_report(out_dir, rows, group_rows)
    print(f"[OK] wrote {csv_path}")
    print(f"[OK] wrote {group_csv}")
    print(f"[OK] wrote {out_dir / 'SLOSH_PEAK_DIAGNOSTIC_REPORT.md'}")
    if args.plot:
        print(f"[OK] plots: {plot_dir}")


if __name__ == "__main__":
    main()
