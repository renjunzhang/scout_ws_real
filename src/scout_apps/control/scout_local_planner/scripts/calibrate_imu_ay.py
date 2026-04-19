#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU ay 离线标定脚本

功能：
    从 rosbag 中提取 /imu/data 和 /odom，自动分段估计：
    1. 零偏 (bias)：静止段 ay 均值
    2. 比例系数 (scale)：运动段线性拟合 ay_filtered vs v*omega

输出：
    imu_ay_calibration.yaml  ← 可直接追加到 mpc_params.yaml 或 slosh_experiment.launch

用法：
    # 先录一包标定 bag（用 run_imu_stage2_sequence.py 驱动）
    python3 calibrate_imu_ay.py <bag_file> [--output imu_ay_calibration.yaml] [--plot]

注意：
    标定 bag 录制前机器人必须先静止 5 秒，再做左右弧线运动。
    建议使用 scripts/run_imu_stage2_sequence.py 生成标准动作序列。
"""

import argparse
import math
import sys
import os

import rosbag
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ─────────────────────────────────────────────────────────────────────────────
# 参数
# ─────────────────────────────────────────────────────────────────────────────
STATIC_V_MAX   = 0.03    # m/s  静止判定速度门槛
STATIC_W_MAX   = 0.03    # rad/s 静止判定角速度门槛
MIN_STATIC_S   = 3.0     # 最少静止时长（秒）
MIN_MOTION_S   = 1.0     # 最少运动时长（秒）
EMA_ALPHA      = 0.3     # EMA 滤波系数（与节点一致）
TRIM_RATIO     = 0.10    # trimmed mean 截比

# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="IMU ay 离线标定")
    p.add_argument("bag", help="rosbag 路径")
    p.add_argument("--output", default="imu_ay_calibration.yaml", help="输出 yaml 路径")
    p.add_argument("--plot", action="store_true", help="保存标定验证图")
    p.add_argument("--imu-topic",  default="/imu/data")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--ema-alpha",  type=float, default=EMA_ALPHA)
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
def trimmed_mean(arr, ratio=TRIM_RATIO):
    a = sorted(arr)
    n = len(a)
    k = int(n * ratio)
    trimmed = a[k: n - k] if n - 2 * k > 0 else a
    return float(np.mean(trimmed))


def load_bag(bag_path, imu_topic, odom_topic):
    """返回时序对齐的 (t, ay_raw, v, omega) numpy array"""
    imu_msgs  = []   # (t, ay)
    odom_msgs = []   # (t, v, omega)

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=[imu_topic, odom_topic]):
            ts = msg.header.stamp.to_sec()
            if topic == imu_topic:
                imu_msgs.append((ts, msg.linear_acceleration.y))
            elif topic == odom_topic:
                v = msg.twist.twist.linear.x
                w = msg.twist.twist.angular.z
                odom_msgs.append((ts, v, w))

    if not imu_msgs or not odom_msgs:
        raise RuntimeError(f"话题数据为空：imu={len(imu_msgs)}, odom={len(odom_msgs)}")

    imu_arr  = np.array(imu_msgs)    # (N, 2): t, ay
    odom_arr = np.array(odom_msgs)   # (M, 3): t, v, omega

    # 插值到 IMU 时间轴
    t_imu = imu_arr[:, 0]
    v_interp = np.interp(t_imu, odom_arr[:, 0], odom_arr[:, 1])
    w_interp = np.interp(t_imu, odom_arr[:, 0], odom_arr[:, 2])

    return t_imu, imu_arr[:, 1], v_interp, w_interp


def apply_ema(data, alpha):
    out = np.empty_like(data)
    out[0] = data[0]
    for i in range(1, len(data)):
        out[i] = alpha * data[i] + (1 - alpha) * out[i - 1]
    return out


def detect_segments(t, v, omega, static_v_max, static_w_max, min_static_s, min_motion_s):
    """返回 (static_mask, motion_mask)"""
    is_static = (np.abs(v) < static_v_max) & (np.abs(omega) < static_w_max)

    def contiguous_long_enough(mask, min_dur):
        result = np.zeros_like(mask)
        i = 0
        while i < len(mask):
            if mask[i]:
                j = i
                while j < len(mask) and mask[j]:
                    j += 1
                dur = t[j - 1] - t[i]
                if dur >= min_dur:
                    result[i:j] = True
                i = j
            else:
                i += 1
        return result

    static_mask = contiguous_long_enough(is_static, min_static_s)
    motion_mask = contiguous_long_enough(~is_static, min_motion_s)
    return static_mask, motion_mask


# ─────────────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    if not os.path.isfile(args.bag):
        print(f"[ERROR] bag 不存在：{args.bag}")
        sys.exit(1)

    print(f"[calibrate_imu_ay] 读取：{args.bag}")
    t, ay_raw, v, omega = load_bag(args.bag, args.imu_topic, args.odom_topic)
    v_omega = v * omega   # 运动学离心加速度估计

    # ── EMA 滤波 ──────────────────────────────────────────────────────────────
    ay_ema = apply_ema(ay_raw, args.ema_alpha)

    # ── 分段 ──────────────────────────────────────────────────────────────────
    static_mask, motion_mask = detect_segments(
        t, v, omega,
        STATIC_V_MAX, STATIC_W_MAX, MIN_STATIC_S, MIN_MOTION_S)

    n_static = int(np.sum(static_mask))
    n_motion = int(np.sum(motion_mask))
    print(f"  静止样本：{n_static}  运动样本：{n_motion}")

    if n_static < 50:
        print("[WARN] 静止样本不足，请确认 bag 开头有 ≥5s 静止段。bias 设为 0.0")
        bias = 0.0
    else:
        bias = trimmed_mean(ay_ema[static_mask].tolist(), TRIM_RATIO)

    print(f"  估计 bias = {bias:.5f} m/s²")

    # ── 去零偏 ────────────────────────────────────────────────────────────────
    ay_unbiased = ay_ema - bias

    # ── 线性拟合 scale factor ─────────────────────────────────────────────────
    if n_motion < 50:
        print("[WARN] 运动样本不足，scale 设为 1.0")
        scale = 1.0
        calib_rms = float(np.std(ay_unbiased[static_mask] if n_static > 0 else ay_unbiased))
        corr_raw   = float(np.corrcoef(ay_raw[motion_mask], v_omega[motion_mask])[0, 1]) if n_motion > 1 else 0.0
        corr_calib = corr_raw
    else:
        x = v_omega[motion_mask]
        y = ay_unbiased[motion_mask]

        # 过滤 v*omega 极小（直行）段，避免 0 点拉偏拟合
        excite_mask = np.abs(x) > 0.01
        if np.sum(excite_mask) < 20:
            print("[WARN] 激励段样本不足（转弯幅度过小），scale 设为 1.0")
            scale = 1.0
        else:
            x_fit = x[excite_mask]
            y_fit = y[excite_mask]
            # 过原点线性拟合：y = scale * x
            scale = float(np.dot(x_fit, y_fit) / np.dot(x_fit, x_fit))

        ay_calib = ay_unbiased * scale if abs(scale) > 0.01 else ay_unbiased

        # 残差 & 相关性
        calib_rms = float(np.sqrt(np.mean((ay_calib[motion_mask] - v_omega[motion_mask]) ** 2)))
        raw_rms   = float(np.sqrt(np.mean((ay_unbiased[motion_mask] - v_omega[motion_mask]) ** 2)))
        corr_raw   = float(np.corrcoef(ay_unbiased[motion_mask], v_omega[motion_mask])[0, 1])
        corr_calib = float(np.corrcoef(ay_calib[motion_mask], v_omega[motion_mask])[0, 1])

        print(f"  估计 scale = {scale:.4f}")
        print(f"  残差 RMS  原始: {raw_rms:.4f}  标定后: {calib_rms:.4f} m/s²")
        print(f"  相关系数  原始: {corr_raw:.3f}   标定后: {corr_calib:.3f}")

    # ── 输出 yaml ─────────────────────────────────────────────────────────────
    yaml_content = f"""\
# IMU ay 标定结果 — 由 calibrate_imu_ay.py 生成
# bag: {os.path.basename(args.bag)}
#
# 使用方式（追加到 mpc_params.yaml 的 slosh_estimator 段，或在 launch 里 override）：
#
#   slosh_estimator/imu_ay_scale: {scale:.6f}
#
# 代码应用位置：local_planner_ros.cpp
#   imu_ay_unbiased_ = (ay_raw - bias) * imu_ay_scale_
#
# 注意：
#   bias 由代码在线估计（静止窗口），不需要写进 yaml。
#   scale 是离线标定值，需要手动写入。

slosh_estimator:
  imu_ay_scale: {scale:.6f}          # 标定比例系数（1.0 = 不校正）

# 诊断信息：
#   静止段样本数：{n_static}
#   运动段样本数：{n_motion}
#   bias 估计：{bias:.5f} m/s²
#   corr(ay_unbiased, v*omega) 原始：{corr_raw:.3f}
#   corr(ay_calib,    v*omega) 标定后：{corr_calib:.3f}
#   残差 RMS 标定后：{calib_rms:.4f} m/s²
"""

    with open(args.output, "w") as f:
        f.write(yaml_content)
    print(f"\n[calibrate_imu_ay] 已写出：{args.output}")

    # ── 可选：保存图 ──────────────────────────────────────────────────────────
    if args.plot and HAS_MPL:
        plot_path = args.output.replace(".yaml", "_plot.png")
        ay_calib_full = (ay_ema - bias) * scale

        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

        axes[0].plot(t - t[0], ay_raw,   "b", alpha=0.4, label="ay_raw")
        axes[0].plot(t - t[0], ay_ema,   "b", label="ay_ema")
        axes[0].plot(t - t[0], v_omega,  "r", label="v·ω (kinematic)")
        axes[0].axhline(bias, color="gray", linestyle="--", label=f"bias={bias:.4f}")
        axes[0].set_ylabel("m/s²"); axes[0].legend(fontsize=8); axes[0].set_title("原始 ay vs v·ω")

        axes[1].plot(t - t[0], ay_calib_full, "g", label=f"ay_calib (scale={scale:.4f})")
        axes[1].plot(t - t[0], v_omega,       "r", alpha=0.6, label="v·ω")
        axes[1].set_ylabel("m/s²"); axes[1].legend(fontsize=8); axes[1].set_title("标定后 ay vs v·ω")

        axes[2].plot(t - t[0], v,     "k",  label="v (m/s)")
        axes[2].plot(t - t[0], omega, "m",  label="ω (rad/s)")
        axes[2].fill_between(t - t[0], -0.5, 0.5, where=static_mask, alpha=0.15, color="blue",  label="静止段")
        axes[2].fill_between(t - t[0], -0.5, 0.5, where=motion_mask, alpha=0.15, color="green", label="运动段")
        axes[2].set_ylabel("速度"); axes[2].set_xlabel("时间 (s)"); axes[2].legend(fontsize=8)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=120)
        print(f"[calibrate_imu_ay] 图已保存：{plot_path}")
    elif args.plot:
        print("[WARN] matplotlib 不可用，跳过画图")

    # ── 结论 ──────────────────────────────────────────────────────────────────
    print("\n─── 标定结论 ───────────────────────────────────────────────")
    print(f"  imu_ay_scale = {scale:.4f}")
    if abs(scale - 1.0) < 0.05:
        print("  scale ≈ 1.0：IMU ay 无需比例校正，当前安装和杠杆臂影响可忽略。")
    elif scale > 1.2 or scale < 0.8:
        print(f"  scale 偏离 1.0 明显（{scale:.3f}），建议检查杠杆臂参数或安装。")
    else:
        print(f"  scale 偏离 1.0 中等（{scale:.3f}），建议写入 yaml 后重新验证。")
    if corr_calib < 0.7:
        print(f"  [WARN] corr(ay_calib, v·ω)={corr_calib:.3f} < 0.7，考虑增加转弯幅度或时长再重跑。")
    print("────────────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
