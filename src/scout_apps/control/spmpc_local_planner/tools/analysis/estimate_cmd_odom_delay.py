#!/usr/bin/env python3
"""
estimate_cmd_odom_delay.py
--------------------------
从实物 bag 提取 /cmd_vel 与 /odom，用微分信号互相关估计
线速度和角速度的执行延迟（cmd → odom 响应时间）。

用法
----
  python3 estimate_cmd_odom_delay.py <bag> [bag2 ...] [--plot] [--out_dir DIR]

方法
----
1. 将 cmd_vel / odom 插值到均匀时间轴（默认 200Hz）
2. 对插值信号求一阶差分（强调变化事件，去除直流/慢趋势）
3. 在 [0, max_lag] 窗口内寻找归一化互相关峰值
4. 峰值对应的时移即为 cmd→odom 延迟估计
"""

import argparse
import sys
from pathlib import Path
import numpy as np

try:
    import rosbag
except ImportError:
    print("错误: 需在 ROS 环境中运行（source devel/setup.bash）", file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ── 配置常量 ────────────────────────────────────────────────
CONFIGURED_LINEAR_DELAY_MS  = 150.0
CONFIGURED_ANGULAR_DELAY_MS = 220.0

INTERP_HZ     = 200.0   # 插值频率，5ms 精度
MAX_LAG_SEC   = 0.60    # 最大搜索延迟
SMOOTH_WIN_MS = 15.0    # 对 odom 的预平滑窗口（去高频噪声）
DERIV_SMOOTH  = 2       # 对微分信号的轻微平滑（样本数）


# ── 读取 bag ────────────────────────────────────────────────
def load_signals(bag_path):
    cmd_t, cmd_v, cmd_w = [], [], []
    odom_t, odom_v, odom_w = [], [], []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=["/cmd_vel", "/odom"]):
            ts = t.to_sec()
            if topic == "/cmd_vel":
                cmd_t.append(ts); cmd_v.append(msg.linear.x); cmd_w.append(msg.angular.z)
            else:
                odom_t.append(ts)
                odom_v.append(msg.twist.twist.linear.x)
                odom_w.append(msg.twist.twist.angular.z)
    return (np.array(cmd_t), np.array(cmd_v), np.array(cmd_w),
            np.array(odom_t), np.array(odom_v), np.array(odom_w))


# ── 工具 ─────────────────────────────────────────────────────
def resample(t_src, x_src, hz):
    t0, t1 = max(t_src[0], 0), t_src[-1]
    t_uni = np.arange(t0, t1, 1.0 / hz)
    return t_uni, np.interp(t_uni, t_src, x_src)


def smooth_ma(x, win):
    if win <= 1:
        return x
    k = np.ones(win) / win
    return np.convolve(x, k, mode="same")


def diff_signal(x, win=1):
    """一阶差分（相邻帧差），可选平滑。"""
    d = np.diff(x, prepend=x[0])
    return smooth_ma(d, win)


def xcorr_delay(a, b, dt, max_lag_sec):
    """
    在因果窗口 [0, max_lag_sec] 内估计 b 相对 a 的延迟。
    返回 (delay_sec, peak_value, lags_sec, corr)
    """
    a = a - a.mean()
    b = b - b.mean()
    n = len(a)
    norm = np.sqrt((a**2).sum() * (b**2).sum()) + 1e-30
    corr = np.correlate(a, b, mode="full") / norm
    # lags: 负值=b超前a, 正值=b滞后a
    lags = (np.arange(len(corr)) - (n - 1)) * dt

    # 只看因果延迟 [0, max_lag]
    valid = (lags >= 0) & (lags <= max_lag_sec)
    if not valid.any():
        return 0.0, 0.0, lags, corr

    idx = np.where(valid)[0]
    best = idx[np.argmax(corr[idx])]
    return lags[best], corr[best], lags, corr


# ── 核心估计 ─────────────────────────────────────────────────
def estimate(cmd_t, cmd_x, odom_t, odom_x, label=""):
    dt = 1.0 / INTERP_HZ
    smooth_win = max(1, int(SMOOTH_WIN_MS / 1000.0 * INTERP_HZ))

    # 共同时间范围
    t0 = max(cmd_t[0], odom_t[0])
    t1 = min(cmd_t[-1], odom_t[-1])
    t_uni = np.arange(t0, t1, dt)

    cmd_uni  = np.interp(t_uni, cmd_t, cmd_x)
    odom_uni = smooth_ma(np.interp(t_uni, odom_t, odom_x), smooth_win)

    # 微分信号（去直流，强调变化时刻）
    d_cmd  = diff_signal(cmd_uni,  DERIV_SMOOTH)
    d_odom = diff_signal(odom_uni, DERIV_SMOOTH)

    delay, peak, lags, corr = xcorr_delay(d_cmd, d_odom, dt, MAX_LAG_SEC)

    # 额外诊断：信号变化量（std of diff），太小说明信号近乎恒定，估计不可信
    excitation = d_cmd.std()
    return delay, peak, excitation, lags, corr


# ── 打印结果 ─────────────────────────────────────────────────
def print_result(name, ld, lc, le, ad, ac, ae):
    lin_err = ld * 1000 - CONFIGURED_LINEAR_DELAY_MS
    ang_err = ad * 1000 - CONFIGURED_ANGULAR_DELAY_MS
    LOW_EXCIT = 0.002  # 低激励阈值（m/s per sample）

    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"{'─'*60}")
    lin_warn = " [⚠ 低激励，可信度低]" if le < LOW_EXCIT else ""
    print(f"  线速度  实测: {ld*1000:6.1f}ms  配置: {CONFIGURED_LINEAR_DELAY_MS:.0f}ms"
          f"  误差: {lin_err:+.1f}ms  xcorr峰: {lc:.3f}{lin_warn}")
    ang_warn = " [⚠ 低激励，可信度低]" if ae < LOW_EXCIT * 0.2 else ""
    print(f"  角速度  实测: {ad*1000:6.1f}ms  配置: {CONFIGURED_ANGULAR_DELAY_MS:.0f}ms"
          f"  误差: {ang_err:+.1f}ms  xcorr峰: {ac:.3f}{ang_warn}")

    if abs(lin_err) > 50 or abs(ang_err) > 50:
        print(f"  ⚠️  误差>50ms，配置延迟与实测偏差较大，相位补偿可能显著失准。")
    elif abs(lin_err) > 30 or abs(ang_err) > 30:
        print(f"  ⚠️  误差>30ms，建议校准配置值。")
    else:
        print(f"  ✓  误差≤30ms，配置值基本可用。")
    print(f"{'='*60}")


# ── 绘图 ─────────────────────────────────────────────────────
def make_plot(results, out_path):
    if not HAS_MPL:
        return
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 3.5 * n), squeeze=False)
    fig.suptitle("cmd_vel -> odom Cross-Correlation Delay", fontsize=12)

    for i, r in enumerate(results):
        name = r["name"]
        for col, (lags, corr, delay, cfg_ms, title) in enumerate([
            (r["lin_lags"], r["lin_corr"], r["lin_delay"],
             CONFIGURED_LINEAR_DELAY_MS / 1000, "linear.x"),
            (r["ang_lags"], r["ang_corr"], r["ang_delay"],
             CONFIGURED_ANGULAR_DELAY_MS / 1000, "angular.z"),
        ]):
            ax = axes[i][col]
            mask = (lags >= -0.02) & (lags <= MAX_LAG_SEC + 0.02)
            ax.plot(lags[mask] * 1000, corr[mask], lw=1.2, color="steelblue")
            ax.axvline(delay * 1000, color="red",    ls="--", lw=1.8,
                       label=f"measured {delay*1000:.0f}ms")
            ax.axvline(cfg_ms * 1000, color="orange", ls=":",  lw=1.8,
                       label=f"configured {cfg_ms*1000:.0f}ms")
            ax.set_title(f"{name}  [{title}]", fontsize=9)
            ax.set_xlabel("lag (ms)")
            ax.set_ylabel("norm xcorr (diff)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    print(f"图保存: {out_path}")
    plt.close(fig)


# ── 主流程 ────────────────────────────────────────────────────
def analyze(bag_path):
    name = Path(bag_path).stem
    print(f"读取 {name} ...", end=" ", flush=True)
    cmd_t, cmd_v, cmd_w, odom_t, odom_v, odom_w = load_signals(bag_path)
    print(f"cmd={len(cmd_t)}  odom={len(odom_t)}")

    ld, lc, le, l_lags, l_corr = estimate(cmd_t, cmd_v, odom_t, odom_v, "linear")
    ad, ac, ae, a_lags, a_corr = estimate(cmd_t, cmd_w, odom_t, odom_w, "angular")

    print_result(name, ld, lc, le, ad, ac, ae)
    return dict(name=name,
                lin_delay=ld, lin_peak=lc, lin_excit=le,
                lin_lags=l_lags, lin_corr=l_corr,
                ang_delay=ad, ang_peak=ac, ang_excit=ae,
                ang_lags=a_lags, ang_corr=a_corr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--plot",    action="store_true")
    ap.add_argument("--out_dir", default="/tmp/delay_analysis")
    args = ap.parse_args()

    results = []
    for p in args.bags:
        if not Path(p).exists():
            print(f"跳过(不存在): {p}", file=sys.stderr)
            continue
        results.append(analyze(p))

    if not results:
        sys.exit(1)

    if len(results) > 1:
        lds = [r["lin_delay"] for r in results]
        ads = [r["ang_delay"] for r in results]
        print(f"\n{'─'*60}")
        print(f"  汇总 {len(results)} 个 bag")
        print(f"  线速度  均值={np.mean(lds)*1000:.1f}ms  "
              f"std={np.std(lds)*1000:.1f}ms  "
              f"范围=[{min(lds)*1000:.0f},{max(lds)*1000:.0f}]ms")
        print(f"  角速度  均值={np.mean(ads)*1000:.1f}ms  "
              f"std={np.std(ads)*1000:.1f}ms  "
              f"范围=[{min(ads)*1000:.0f},{max(ads)*1000:.0f}]ms")
        print(f"  建议值: linear_delay_sec: {np.mean(lds):.3f}")
        print(f"          angular_delay_sec: {np.mean(ads):.3f}")
        print(f"{'─'*60}")

    if args.plot:
        out = Path(args.out_dir)
        out.mkdir(parents=True, exist_ok=True)
        make_plot(results, str(out / "delay_xcorr.png"))


if __name__ == "__main__":
    main()
