#!/usr/bin/env python3
"""对比 B0 / B_slosh 两个 smoke bag, 判定 Phase 2 slosh cost 是否产生可观察差异。

字段索引按 diagnostics_publisher.cpp 当前布局 (改了那边要同步这里):
  /spmpc/cost_breakdown        : 0=total 1=J_contour 2=J_lag 3=J_progress 4=J_v
                                 5=J_control 6=J_smooth 7=J_terminal 8=J_corridor
                                 9=J_obstacle 10=J_slosh_eta 11=J_slosh_eta_dot
                                 21=pct_slosh_total
  /spmpc/slosh_horizon_summary : 0=h_peak_pred 1=h_p95_pred 2=eta_x_peak
                                 3=eta_y_peak 4=eta_dot_norm_peak 5=peak_k

用法: analyze_b0_bslosh_compare.py <bag_dir> <variant1> <variant2> ...
"""

import sys
import statistics as st

try:
    import rosbag
except ImportError:
    print("[ERR] 需要 rosbag (source devel/setup.bash 后再跑)")
    sys.exit(1)


def load(bag_path):
    d = {
        "cmd_v": [], "cmd_omega": [],
        "h_peak": [], "h_p95": [], "eta_dot_peak": [],
        "progress": [], "status": [], "solver_ms": [],
        "J_total": [], "J_contour": [], "J_progress": [], "J_control": [],
        "J_smooth": [], "J_slosh_eta": [], "J_slosh_eta_dot": [],
        "slosh_total_pct": [],
    }
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f"[ERR] 打不开 {bag_path}: {e}")
        return None
    with bag:
        for topic, msg, _ in bag.read_messages():
            if topic == "/cmd_vel":
                d["cmd_v"].append(msg.linear.x)
                d["cmd_omega"].append(msg.angular.z)
            elif topic == "/spmpc/cost_breakdown" and len(msg.data) >= 22:
                d["J_total"].append(msg.data[0])
                d["J_contour"].append(msg.data[1])
                d["J_progress"].append(msg.data[3])
                d["J_control"].append(msg.data[5])
                d["J_smooth"].append(msg.data[6])
                d["J_slosh_eta"].append(msg.data[10])
                d["J_slosh_eta_dot"].append(msg.data[11])
                d["slosh_total_pct"].append(msg.data[21])
            elif topic == "/spmpc/slosh_horizon_summary" and len(msg.data) >= 6:
                d["h_peak"].append(msg.data[0])
                d["h_p95"].append(msg.data[1])
                d["eta_dot_peak"].append(msg.data[4])
            elif topic == "/spmpc/debug/progress_s":
                d["progress"].append(msg.data)
            elif topic == "/spmpc/status":
                d["status"].append(msg.data)
            elif topic == "/spmpc/solver_time_ms":
                d["solver_ms"].append(msg.data)
    return d


def summ(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None
    return {"n": len(xs), "mean": st.mean(xs), "max": max(xs), "min": min(xs)}


def monotonic_frac(xs):
    """progress_s 单调非降的比例 (1.0 = 全程单调)。"""
    if len(xs) < 2:
        return None
    ok = sum(1 for a, b in zip(xs, xs[1:]) if b >= a - 1e-6)
    return ok / (len(xs) - 1)


def fmt(s):
    if s is None:
        return "   (无数据)"
    return f"n={s['n']:4d} mean={s['mean']:+.4f} max={s['max']:+.4f} min={s['min']:+.4f}"


def main():
    if len(sys.argv) < 4:
        print("用法: analyze_b0_bslosh_compare.py <bag_dir> <v1> <v2>")
        sys.exit(1)
    bag_dir = sys.argv[1].rstrip("/")
    variants = sys.argv[2:]
    data = {}
    for v in variants:
        d = load(f"{bag_dir}/{v}.bag")
        if d is None:
            print(f"[skip] {v}")
            continue
        data[v] = d

    if len(data) < 2:
        print("[ERR] 至少需要两个可读 bag 才能对比")
        sys.exit(1)

    print()
    for v, d in data.items():
        last_status = d["status"][-1] if d["status"] else "(无)"
        print(f"---- {v} ----  末态 status={last_status}")
        print(f"  cmd_v          {fmt(summ(d['cmd_v']))}")
        print(f"  cmd_omega      {fmt(summ(d['cmd_omega']))}")
        print(f"  h_peak_pred    {fmt(summ(d['h_peak']))}")
        print(f"  h_p95_pred     {fmt(summ(d['h_p95']))}")
        print(f"  eta_dot_peak   {fmt(summ(d['eta_dot_peak']))}")
        print(f"  J_total        {fmt(summ(d['J_total']))}")
        print(f"  J_progress     {fmt(summ(d['J_progress']))}")
        print(f"  J_contour      {fmt(summ(d['J_contour']))}")
        print(f"  J_control      {fmt(summ(d['J_control']))}")
        print(f"  J_smooth       {fmt(summ(d['J_smooth']))}")
        print(f"  J_slosh_eta    {fmt(summ(d['J_slosh_eta']))}")
        print(f"  J_slosh_eta_d  {fmt(summ(d['J_slosh_eta_dot']))}")
        print(f"  slosh_pct(%)   {fmt(summ(d['slosh_total_pct']))}  # 仅辅助; J_progress为负时可能>100%")
        print(f"  solver_ms      {fmt(summ(d['solver_ms']))}")
        mf = monotonic_frac(d["progress"])
        print(f"  progress 单调比 {'(无数据)' if mf is None else f'{mf:.3f}'}")
        print()

    # 判定: B0 vs B_slosh 是否有可观察差异
    a, b = variants[0], variants[1]
    if a in data and b in data:
        da, db = data[a], data[b]
        print(f"================ 判定 {a} vs {b} ================")

        def mean_or0(xs):
            xs = [x for x in xs if x is not None]
            return st.mean(xs) if xs else 0.0

        dv = abs(mean_or0(da["cmd_v"]) - mean_or0(db["cmd_v"]))
        dw = abs(mean_or0(da["cmd_omega"]) - mean_or0(db["cmd_omega"]))
        dh = abs(mean_or0(da["h_peak"]) - mean_or0(db["h_peak"]))
        print(f"  |Δ mean cmd_v|      = {dv:.5f} m/s")
        print(f"  |Δ mean cmd_omega|  = {dw:.5f} rad/s")
        print(f"  |Δ mean h_peak_pred|= {dh:.6f}")

        # 阈值: cmd_v 差 < 5mm/s 且 cmd_omega 差 < 0.01 rad/s 视为"几乎一样"
        if dv < 0.005 and dw < 0.01:
            print()
            print("  >>> 结论: B0 与 B_slosh 控制输出几乎一致。")
            print("      slosh cost 没有改变行为 —— 印证 solver '单对常值候选' 自由度不足 (问题1)。")
            print("      Phase 2 需升级 solver: 让候选优化 horizon 内一串 u_k (或上 SQP/OSQP)。")
        else:
            print()
            print("  >>> 结论: B0 与 B_slosh 产生了可观察差异。")
            print("      slosh cost 已在起作用; 检查是否 B_slosh 的 h_peak_pred 更低 (期望方向)。")
            if dh > 1e-6:
                better = "更低 (符合预期)" if mean_or0(db["h_peak"]) < mean_or0(da["h_peak"]) else "更高 (反常, 需查)"
                print(f"      B_slosh 预测峰值晃动相对 B0: {better}")


if __name__ == "__main__":
    main()
