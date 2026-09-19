#!/usr/bin/env python3
"""汇总 w_slosh 扫描结果, 找"峰值降下来、均值不升、omega 不抖"的拐点。

读取 <dir> 下 *_w<值>.bag(由 sweep_w_slosh.sh 生成), 复用 analyze 的 load() 解析。
主晃动量用 observer(/spmpc/debug/slosh_state), 与控制器内部模型无关, 跨 w_slosh 可比。
若目录里有 B0.bag, 作为 5 维 B0 基线一并列出(标 w=NA)。

用法: sweep_w_slosh_summary.py <bag_dir> [variant_prefix(默认 B_slosh)]
"""

import glob
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_b0_bslosh_compare import load, monotonic_frac  # noqa: E402


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return st.mean(xs) if xs else 0.0


def _peak(xs):
    xs = [x for x in xs if x is not None]
    return max(xs) if xs else 0.0


def _row(tag, w_label, d):
    return {
        "tag": tag,
        "w": w_label,
        "obs_peak": _peak(d["obs_eta_norm"]),
        "obs_mean": _mean(d["obs_eta_norm"]),
        "obs_dot_peak": _peak(d["obs_eta_dot_norm"]),
        "cmd_v": _mean(d["cmd_v"]),
        "cmd_w": _mean(d["cmd_omega"]),
        "J_slosh": _mean(d["J_slosh_eta"]),
        "solver_ms": _mean(d["solver_ms"]),
        "mono": monotonic_frac(d["progress"]),
        "n": len(d["cmd_v"]),
    }


def main():
    if len(sys.argv) < 2:
        print("用法: sweep_w_slosh_summary.py <bag_dir> [variant_prefix]")
        sys.exit(1)
    bag_dir = sys.argv[1].rstrip("/")
    prefix = sys.argv[2] if len(sys.argv) > 2 else "B_slosh"

    rows = []

    b0_path = os.path.join(bag_dir, "B0.bag")
    if os.path.exists(b0_path):
        d = load(b0_path)
        if d:
            rows.append(("__b0__", _row("B0(5维基线)", "NA", d)))

    for path in glob.glob(os.path.join(bag_dir, f"{prefix}_w*.bag")):
        name = os.path.basename(path)[:-4]
        try:
            w = float(name.split("_w", 1)[1])
        except (IndexError, ValueError):
            continue
        d = load(path)
        if d:
            rows.append((w, _row(name, f"{w:g}", d)))

    if not rows:
        print(f"[ERR] {bag_dir} 下没找到 {prefix}_w*.bag(或 B0.bag)")
        sys.exit(1)

    rows.sort(key=lambda kv: (kv[0] == "__b0__", kv[0] if kv[0] != "__b0__" else 0.0))
    table = [r for _, r in rows]

    print()
    print("w_slosh 扫描汇总 (observer 实际执行晃动; active 窗口, GOAL_REACHED 已排除)")
    print(f"{'tag':<16}{'w':>5}{'obs_peak':>11}{'obs_mean':>11}{'obsd_peak':>11}"
          f"{'cmd_v':>8}{'cmd_w':>8}{'J_slosh':>9}{'ms':>7}{'mono':>7}{'n':>6}")
    print("-" * 105)
    for r in table:
        mono = "NA" if r["mono"] is None else f"{r['mono']:.3f}"
        print(f"{r['tag']:<16}{r['w']:>5}{r['obs_peak']:>11.5f}{r['obs_mean']:>11.5f}{r['obs_dot_peak']:>11.5f}"
              f"{r['cmd_v']:>8.3f}{r['cmd_w']:>8.3f}{r['J_slosh']:>9.3f}{r['solver_ms']:>7.2f}{mono:>7}{r['n']:>6}")

    swept = [r for r in table if r["w"] != "NA"]
    if swept:
        best_peak = min(swept, key=lambda r: r["obs_peak"])
        best_mean = min(swept, key=lambda r: r["obs_mean"])
        print()
        print(f"  峰值最低: w_slosh={best_peak['w']}  (obs_peak={best_peak['obs_peak']:.5f})")
        print(f"  均值最低: w_slosh={best_mean['w']}  (obs_mean={best_mean['obs_mean']:.5f})")
        print("  拐点建议: 取 obs_peak 已明显下降、obs_mean 与 cmd_w 尚未回升的最小 w_slosh。")
        print("  (cmd_w 随 w 明显上升 = 控制开始'画龙'/抖动, 该值偏大。)")


if __name__ == "__main__":
    main()
