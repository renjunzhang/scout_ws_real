#!/usr/bin/env python3
"""
diagnose_speed_profile.py
-------------------------
分析 bag 文件中的全局速度剖面，区分三类根因：
  候选 A：global_spline kappa >> 实际路径几何（cubic spline 放大）
  候选 B：dkappa 有限差分噪声放大（dkappa >> 100 1/m²）
  候选 C：局部 reactive cap 仍在主导（全局剖面正确，但被覆盖）

使用方法：
  python3 diagnose_speed_profile.py <bag_file> [options]

Options:
  --omega-max   ω_max (rad/s), default=2.0
  --alpha-max   α_max (rad/s²), default=4.0
  --a-lat-max   a_lat_max (m/s²), default=1.0
  --a-max       a_max (m/s²，用于 backward/forward pass), default=0.5
  --v-des       期望速度 (m/s), default=1.6
  --ds          速度剖面采样间隔 (m), default=0.05
  --smooth-kappa 对 kappa 数组做移动平均后再差分 dkappa，模拟加平滑后的效果
"""

import argparse
import sys
import numpy as np

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag not found. Source your ROS workspace first.")
    sys.exit(1)

try:
    from scipy.interpolate import CubicSpline
except ImportError:
    print("ERROR: scipy not found. pip install scipy")
    sys.exit(1)


# ─── 样条曲率工具 ─────────────────────────────────────────────────────────────

def fit_cubic_spline(xs, ys):
    """用弧长参数化拟合 x(s)、y(s) 的三次样条."""
    dists = np.hypot(np.diff(xs), np.diff(ys))
    s = np.concatenate([[0.0], np.cumsum(dists)])
    # 去重（防止重复弧长导致 spline 报错）
    mask = np.concatenate([[True], np.diff(s) > 1e-9])
    s, xs, ys = s[mask], np.array(xs)[mask], np.array(ys)[mask]
    if len(s) < 4:
        return None, None, None
    cs_x = CubicSpline(s, xs)
    cs_y = CubicSpline(s, ys)
    return cs_x, cs_y, s[-1]


def eval_kappa_array(cs_x, cs_y, total_len, ds):
    """在均匀弧长采样点上计算曲率."""
    s_arr = np.arange(0.0, total_len + ds * 0.5, ds)
    s_arr = np.clip(s_arr, 0.0, total_len)
    dx  = cs_x(s_arr, 1)   # x'
    dy  = cs_y(s_arr, 1)   # y'
    ddx = cs_x(s_arr, 2)   # x''
    ddy = cs_y(s_arr, 2)   # y''
    denom = (dx**2 + dy**2)**1.5
    kappa = np.where(denom > 1e-9, (dx * ddy - dy * ddx) / denom, 0.0)
    return s_arr, kappa


def compute_dkappa(kappa_arr, ds, smooth_window=0):
    """中心差分估计 dkappa/ds；smooth_window>0 时先对 kappa 做移动平均."""
    k = kappa_arr.copy()
    if smooth_window > 1:
        kernel = np.ones(smooth_window) / smooth_window
        k = np.convolve(k, kernel, mode='same')
    n = len(k)
    dk = np.zeros(n)
    dk[0]    = (k[1] - k[0]) / ds if n > 1 else 0.0
    dk[-1]   = (k[-1] - k[-2]) / ds if n > 1 else 0.0
    dk[1:-1] = (k[2:] - k[:-2]) / (2.0 * ds)
    return dk


# ─── 速度剖面计算（复现 updateSpeedProfile 逻辑）────────────────────────────

def compute_speed_profile(kappa_arr, ds, v_des,
                          omega_max, alpha_max, a_lat_max, a_max,
                          smooth_kappa_window=0):
    """
    三步：
      ① v_geom(s) = min(v_des, ω_max/|κ|, √(α_max/|κ'|), √(a_lat_max/|κ|))
      ② backward pass（制动可行性）
      ③ forward pass（加速可行性）
    返回 (v_geom, v_profile)
    """
    kappa_abs = np.abs(kappa_arr)
    dk_abs    = np.abs(compute_dkappa(kappa_arr, ds, smooth_kappa_window))

    v_geom = np.full_like(kappa_arr, v_des)

    # 横向加速度约束
    if a_lat_max > 0.0:
        mask = kappa_abs > 1e-4
        v_geom[mask] = np.minimum(v_geom[mask],
                                   np.sqrt(a_lat_max / kappa_abs[mask]))
    # 角速度约束
    if omega_max > 1e-3:
        mask = kappa_abs > 1e-4
        v_geom[mask] = np.minimum(v_geom[mask], omega_max / kappa_abs[mask])

    # 角加速度约束
    if alpha_max > 1e-6:
        mask = dk_abs > 1e-4
        v_geom[mask] = np.minimum(v_geom[mask],
                                   np.sqrt(alpha_max / dk_abs[mask]))

    # backward pass
    v_back = v_geom.copy()
    v_back[-1] = 0.0   # 末端停止
    if a_max > 0.0:
        for i in range(len(v_back) - 2, -1, -1):
            v_lim = np.sqrt(max(0.0, v_back[i + 1]**2 + 2.0 * a_max * ds))
            v_back[i] = min(v_back[i], v_lim)

    # forward pass
    v_profile = v_back.copy()
    if a_max > 0.0:
        for i in range(1, len(v_profile)):
            v_lim = np.sqrt(max(0.0, v_profile[i - 1]**2 + 2.0 * a_max * ds))
            v_profile[i] = min(v_profile[i], v_lim)

    return v_geom, v_profile


# ─── 统计摘要 ─────────────────────────────────────────────────────────────────

def pct(arr, p):
    return float(np.percentile(arr, p))


def print_stats(label, arr, unit=""):
    print(f"  {label}: max={arr.max():.4f} p95={pct(arr,95):.4f} "
          f"p75={pct(arr,75):.4f} mean={arr.mean():.4f} {unit}")


# ─── 主流程 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="ROS bag 文件路径")
    parser.add_argument("--omega-max",  type=float, default=2.0)
    parser.add_argument("--alpha-max",  type=float, default=4.0)
    parser.add_argument("--a-lat-max",  type=float, default=1.0)
    parser.add_argument("--a-max",      type=float, default=0.5)
    parser.add_argument("--v-des",      type=float, default=1.6)
    parser.add_argument("--ds",         type=float, default=0.05)
    parser.add_argument("--smooth-kappa", type=int, default=0,
                        help="kappa 移动平均窗口（0=不平滑），用于模拟修复 dkappa 噪声后的效果")
    args = parser.parse_args()

    print(f"\n{'='*65}")
    print(f"  diagnose_speed_profile  |  {args.bag.split('/')[-1]}")
    print(f"{'='*65}")
    print(f"  参数: ω_max={args.omega_max} α_max={args.alpha_max} "
          f"a_lat_max={args.a_lat_max} a_max={args.a_max} "
          f"v_des={args.v_des} ds={args.ds}")
    if args.smooth_kappa > 1:
        print(f"  kappa 平滑窗口: {args.smooth_kappa} 点（≈{args.smooth_kappa*args.ds:.2f} m）")
    print()

    bag = rosbag.Bag(args.bag)

    # ── 读全局平滑路径（取第一条）────────────────────────────────────────────
    smooth_msg = None
    for _, msg, _ in bag.read_messages(topics=['/scout/global_path_smooth']):
        smooth_msg = msg
        break

    if smooth_msg is None:
        print("ERROR: /scout/global_path_smooth 未找到。请确认话题名。")
        bag.close()
        sys.exit(1)

    xs = [p.pose.position.x for p in smooth_msg.poses]
    ys = [p.pose.position.y for p in smooth_msg.poses]
    print(f"[全局平滑路径] n_poses={len(xs)}")

    cs_x, cs_y, total_len = fit_cubic_spline(xs, ys)
    if cs_x is None:
        print("ERROR: 路径点不足，无法拟合样条。")
        bag.close()
        sys.exit(1)
    print(f"  总弧长: {total_len:.2f} m\n")

    # ── 高精度采样曲率 ────────────────────────────────────────────────────────
    s_arr, kappa_arr = eval_kappa_array(cs_x, cs_y, total_len, args.ds)
    kappa_abs = np.abs(kappa_arr)
    dk_abs    = np.abs(compute_dkappa(kappa_arr, args.ds, args.smooth_kappa))

    print("[1] 曲率 κ 统计（global_spline 分辨率 = ds）")
    print_stats("|κ|", kappa_abs, "1/m")
    print(f"  min_radius = {1.0/max(kappa_abs.max(), 1e-9):.3f} m")
    print()

    print("[2] 曲率变化率 |κ'| 统计（中心差分，ds={:.3f} m）".format(args.ds))
    print_stats("|κ'|", dk_abs, "1/m²")
    print()

    # ── v_geom 各约束分量 ────────────────────────────────────────────────────
    print("[3] 各几何约束给出的 v_geom")
    v_lat  = np.where(kappa_abs > 1e-4,
                      np.sqrt(np.maximum(0.0, args.a_lat_max / np.maximum(kappa_abs, 1e-9))),
                      args.v_des)
    v_omega = np.where(kappa_abs > 1e-4,
                       args.omega_max / np.maximum(kappa_abs, 1e-9),
                       args.v_des)
    v_alpha = np.where(dk_abs > 1e-4,
                       np.sqrt(np.maximum(0.0, args.alpha_max / np.maximum(dk_abs, 1e-9))),
                       args.v_des)

    print(f"  √(a_lat/{kappa_abs.max():.3f}) = {np.sqrt(args.a_lat_max/max(kappa_abs.max(),1e-9)):.3f} m/s"
          f"  (横向加速度约束 min)")
    print(f"  ω_max/{kappa_abs.max():.3f}     = {args.omega_max/max(kappa_abs.max(),1e-9):.3f} m/s"
          f"  (角速度约束 min)")
    print(f"  √(α_max/{dk_abs.max():.1f}) = {np.sqrt(args.alpha_max/max(dk_abs.max(),1e-9)):.3f} m/s"
          f"  (角加速度约束 min)")

    bottleneck = np.argmin([v_lat.min(), v_omega.min(), v_alpha.min()])
    names = ["a_lat 约束", "ω_max 约束", "α_max/dkappa 约束"]
    print(f"\n  ★ 最紧约束: {names[bottleneck]}")
    print()

    # ── 完整速度剖面 ─────────────────────────────────────────────────────────
    v_geom, v_profile = compute_speed_profile(
        kappa_arr, args.ds, args.v_des,
        args.omega_max, args.alpha_max, args.a_lat_max, args.a_max,
        args.smooth_kappa)

    print("[4] 速度剖面统计")
    print(f"  v_geom_min  (几何约束下限，含三约束): {v_geom.min():.3f} m/s")
    print(f"  v_profile_min (backward+forward pass 后): {v_profile.min():.3f} m/s")
    print_stats("v_profile", v_profile, "m/s")
    pct5 = pct(v_profile, 5)
    pct25 = pct(v_profile, 25)
    print(f"  p5={pct5:.3f}  p25={pct25:.3f} m/s")
    print()

    if args.smooth_kappa > 1:
        v_geom2, v_profile2 = compute_speed_profile(
            kappa_arr, args.ds, args.v_des,
            args.omega_max, args.alpha_max, args.a_lat_max, args.a_max,
            smooth_kappa_window=0)
        print("[4b] 对比：不平滑 kappa 时的剖面")
        print(f"  v_geom_min  : {v_geom2.min():.3f} m/s  (平滑后: {v_geom.min():.3f} m/s)")
        print(f"  v_profile_min: {v_profile2.min():.3f} m/s  (平滑后: {v_profile.min():.3f} m/s)")
        print()

    # ── 与实际 cmd_vel 对比 ───────────────────────────────────────────────────
    t0 = None
    cmd_vs = []
    for _, msg, t in bag.read_messages(topics=['/cmd_vel']):
        ts = t.to_sec()
        if t0 is None:
            t0 = ts
        cmd_vs.append(msg.linear.x)
    bag.close()

    if cmd_vs:
        cmd_vs = np.array(cmd_vs)
        tracking = cmd_vs[cmd_vs > 0.01]
        print("[5] 实际 cmd_vel 对比")
        if len(tracking):
            print(f"  实际 v 均值 (>0.01): {tracking.mean():.3f} m/s")
            print(f"  实际 v 中位数:       {np.median(tracking):.3f} m/s")
            print(f"  v < 0.15 m/s: {np.mean(tracking<0.15)*100:.1f}%")
            print(f"  v < 0.30 m/s: {np.mean(tracking<0.30)*100:.1f}%")
        print()

    # ── 根因诊断 ─────────────────────────────────────────────────────────────
    print("=" * 65)
    print("  根因诊断")
    print("=" * 65)

    kappa_max = kappa_abs.max()
    dkappa_max = dk_abs.max()
    v_geom_min = v_geom.min()
    v_profile_min = v_profile.min()

    A = kappa_max > 10.0
    B = dkappa_max > 100.0 and v_alpha.min() < v_omega.min() * 0.5
    C = v_geom_min > 0.30 and (len(cmd_vs) == 0 or np.mean(np.array(cmd_vs) < 0.15) > 0.5)

    print()
    status = lambda ok: "✓" if ok else "✗"
    print(f"  候选A [global_spline kappa放大]: kappa_max={kappa_max:.2f} 1/m  {status(A)}")
    if A:
        print(f"    → global_spline_ 本身曲率过高，路径平滑缓存质量不足")
        print(f"    → 修复：让 global_spline_ 使用 B-spline 平滑后的缓存点拟合")
    else:
        print(f"    → kappa_max 正常（路径几何本身没有放大问题）")

    print()
    print(f"  候选B [dkappa噪声主导]: dkappa_max={dkappa_max:.1f} 1/m²  "
          f"v_alpha_min={v_alpha.min():.3f} m/s  {status(B)}")
    if B:
        print(f"    → alpha 约束把 v_geom 压到 {v_alpha.min():.3f} m/s")
        print(f"    → 修复：对 kappa_arr 做移动平均（窗口≈0.5m）再差分")
        print(f"           或完全禁用速度剖面中的 alpha 约束，仅保留 ω_max/κ")
        if args.smooth_kappa > 1:
            print(f"    → 平滑窗口={args.smooth_kappa}时: v_geom_min={v_geom.min():.3f} m/s")
    else:
        print(f"    → dkappa 在可接受范围")

    print()
    print(f"  候选C [局部reactive cap覆盖全局剖面]: "
          f"v_geom_min={v_geom_min:.3f} m/s  {status(C)}")
    if C:
        print(f"    → 全局速度剖面给出 v ≥ {v_geom_min:.2f} m/s，但实际 v_cmd 仍很低")
        print(f"    → 局部 v_cap_omega (tracking_curvature_speed_cap_enable_) 仍在主导")
        print(f"    → 修复：禁用 tracking_curvature_speed_cap_enable_，让全局剖面独立工作")

    if not A and not B and not C:
        print("  → 全部指标正常，低速可能来自其他机制（settling/goal_stop/reentry ramp）")

    print()
    print("  建议下一步:")
    if A:
        print("  [1] 修复 global_spline_ → 使用 global_points_map_smooth_ 拟合")
    if B:
        print("  [2] 在 updateSpeedProfile() 的 kappa_arr 计算 dkappa 前先做移动平均")
        print(f"      建议运行: python3 {sys.argv[0]} {args.bag} --smooth-kappa=10 --ds={args.ds}")
    if C:
        print("  [3] 在 mpc_params_sim.yaml 中设置 tracking_curvature_speed_cap_enable: false")
        print("      然后重跑仿真做 A/B 对比")
    print()


if __name__ == "__main__":
    main()
