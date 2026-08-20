#!/usr/bin/env python3
"""对比 B0 / B_slosh 两个 smoke bag, 判定 Phase 2 slosh cost 是否产生可观察差异。

字段索引按 diagnostics_publisher.cpp 当前布局 (改了那边要同步这里):
  /spmpc/cost_breakdown        : 0=total 1=J_contour 2=J_lag 3=J_progress 4=J_v
                                 5=J_control 6=J_smooth 7=J_terminal 8=J_corridor
                                 9=J_obstacle 10=J_slosh_eta 11=J_slosh_eta_dot
                                 21=pct_slosh_total
  /spmpc/slosh_horizon_summary : 0=h_peak_pred 1=h_p95_pred 2=eta_x_peak
                                 3=eta_y_peak 4=eta_dot_norm_peak 5=peak_k
  /spmpc/primitive             : 0=primitive_id 1=v_start 2=v_mid 3=v_end
                                 4=omega_start 5=omega_mid 6=omega_end

用法: analyze_b0_bslosh_compare.py [--include-reached] <bag_dir> <variant1> <variant2> ...
"""

import argparse
import sys
import statistics as st

try:
    import rosbag
except ImportError:
    print("[ERR] 需要 rosbag (source devel/setup.bash 后再跑)")
    sys.exit(1)


def load(bag_path, include_reached=False):
    d = {
        "cmd_v": [], "cmd_omega": [],
        "h_peak": [], "h_p95": [], "eta_dot_peak": [],
        "obs_eta_norm": [], "obs_eta_dot_norm": [],
        "progress": [], "status": [], "solver_ms": [],
        "J_total": [], "J_contour": [], "J_progress": [], "J_control": [],
        "J_smooth": [], "J_corridor": [], "J_obstacle": [],
        "J_slosh_eta": [], "J_slosh_eta_dot": [],
        "slosh_total_pct": [],
        "corridor_error": [], "corridor_violation": [], "corridor_viol_count": [],
        "guidance_id": [], "guidance_bias": [],
        "primitive_id": [], "v_start_scale": [], "v_mid_scale": [], "v_end_scale": [],
        "omega_start_scale": [], "omega_mid_scale": [], "omega_end_scale": [],
    }
    current_status = ""

    def active_sample():
        return include_reached or current_status != "GOAL_REACHED"

    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print(f"[ERR] 打不开 {bag_path}: {e}")
        return None
    with bag:
        for topic, msg, _ in bag.read_messages():
            if topic == "/cmd_vel":
                if active_sample():
                    d["cmd_v"].append(msg.linear.x)
                    d["cmd_omega"].append(msg.angular.z)
            elif topic == "/spmpc/cost_breakdown" and len(msg.data) >= 22:
                if active_sample():
                    d["J_total"].append(msg.data[0])
                    d["J_contour"].append(msg.data[1])
                    d["J_progress"].append(msg.data[3])
                    d["J_control"].append(msg.data[5])
                    d["J_smooth"].append(msg.data[6])
                    d["J_corridor"].append(msg.data[8])
                    d["J_obstacle"].append(msg.data[9])
                    d["J_slosh_eta"].append(msg.data[10])
                    d["J_slosh_eta_dot"].append(msg.data[11])
                    d["slosh_total_pct"].append(msg.data[21])
            elif topic == "/spmpc/corridor" and len(msg.data) >= 6:
                if active_sample():
                    d["corridor_error"].append(msg.data[2])
                    d["corridor_violation"].append(msg.data[3])
                    d["corridor_viol_count"].append(msg.data[4])
            elif topic == "/spmpc/guidance" and len(msg.data) >= 2:
                if active_sample():
                    d["guidance_id"].append(msg.data[0])
                    d["guidance_bias"].append(msg.data[1])
            elif topic == "/spmpc/primitive" and len(msg.data) >= 7:
                if active_sample():
                    d["primitive_id"].append(msg.data[0])
                    d["v_start_scale"].append(msg.data[1])
                    d["v_mid_scale"].append(msg.data[2])
                    d["v_end_scale"].append(msg.data[3])
                    d["omega_start_scale"].append(msg.data[4])
                    d["omega_mid_scale"].append(msg.data[5])
                    d["omega_end_scale"].append(msg.data[6])
            elif topic == "/spmpc/slosh_horizon_summary" and len(msg.data) >= 6:
                if active_sample():
                    d["h_peak"].append(msg.data[0])
                    d["h_p95"].append(msg.data[1])
                    d["eta_dot_peak"].append(msg.data[4])
            elif topic == "/spmpc/debug/slosh_state" and len(msg.data) >= 4:
                # observer(从 odom 估计的实际执行晃动), 与控制器内部模型无关 ->
                # 跨后端/跨模型公平对比 B0 vs B_slosh 的唯一同尺度晃动量。
                if active_sample():
                    eta_norm = (msg.data[0] ** 2 + msg.data[2] ** 2) ** 0.5
                    eta_dot_norm = (msg.data[1] ** 2 + msg.data[3] ** 2) ** 0.5
                    d["obs_eta_norm"].append(eta_norm)
                    d["obs_eta_dot_norm"].append(eta_dot_norm)
            elif topic == "/spmpc/debug/progress_s":
                if active_sample():
                    d["progress"].append(msg.data)
            elif topic == "/spmpc/status":
                current_status = msg.data
                d["status"].append(msg.data)
            elif topic == "/spmpc/solver_time_ms":
                if active_sample():
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


def cost_share_line(d):
    """各项代价占比 = |mean(J_i)| / Σ|mean(J_j)|, 有界且可解释(避免 |total| 含负 progress 致爆炸)。"""
    pairs = [
        ("contour", "J_contour"), ("progress", "J_progress"), ("control", "J_control"),
        ("smooth", "J_smooth"), ("corridor", "J_corridor"), ("obstacle", "J_obstacle"),
        ("slosh_eta", "J_slosh_eta"), ("slosh_etad", "J_slosh_eta_dot"),
    ]
    mags = {}
    for label, key in pairs:
        xs = [x for x in d.get(key, []) if x is not None]
        mags[label] = abs(st.mean(xs)) if xs else 0.0
    tot = sum(mags.values()) or 1.0
    items = sorted(mags.items(), key=lambda kv: -kv[1])
    return ", ".join(f"{k} {100.0 * v / tot:.0f}%" for k, v in items if v > 1e-9)


def top_counts(xs, limit=6):
    if not xs:
        return "(无数据)"
    counts = {}
    for x in xs:
        key = int(round(x))
        counts[key] = counts.get(key, 0) + 1
    items = sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:limit]
    return ", ".join(f"{k}:{v}" for k, v in items)


def anti_primitive_summary(xs):
    if not xs:
        return "(无数据)"
    total = len(xs)
    anti = sum(1 for x in xs if int(round(x)) >= 1000)
    return f"{anti}/{total} ({100.0 * anti / total:.1f}%)"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--include-reached", action="store_true", help="统计 GOAL_REACHED 后的零输出样本")
    parser.add_argument("bag_dir")
    parser.add_argument("variants", nargs="+")
    args = parser.parse_args()
    if len(args.variants) < 2:
        print("用法: analyze_b0_bslosh_compare.py [--include-reached] <bag_dir> <v1> <v2> ...")
        sys.exit(1)
    bag_dir = args.bag_dir.rstrip("/")
    variants = args.variants
    data = {}
    for v in variants:
        d = load(f"{bag_dir}/{v}.bag", include_reached=args.include_reached)
        if d is None:
            print(f"[skip] {v}")
            continue
        data[v] = d

    if len(data) < 2:
        print("[ERR] 至少需要两个可读 bag 才能对比")
        sys.exit(1)

    print()
    window_label = "full bag including GOAL_REACHED" if args.include_reached else "active solver window, GOAL_REACHED excluded"
    print(f"[统计窗口] {window_label}")
    print()
    for v, d in data.items():
        last_status = d["status"][-1] if d["status"] else "(无)"
        print(f"---- {v} ----  末态 status={last_status}")
        print(f"  cmd_v          {fmt(summ(d['cmd_v']))}")
        print(f"  cmd_omega      {fmt(summ(d['cmd_omega']))}")
        print(f"  h_peak_pred_mm {fmt(summ(d['h_peak']))}  # 控制器内部预测(mm); B0(5维)无slosh状态恒为0, 不可跨模型比")
        print(f"  h_p95_pred     {fmt(summ(d['h_p95']))}")
        print(f"  eta_dot_peak   {fmt(summ(d['eta_dot_peak']))}")
        print(f"  obs_eta_norm   {fmt(summ(d['obs_eta_norm']))}  # observer(实际执行晃动), 跨后端公平对比量")
        print(f"  obs_eta_dot    {fmt(summ(d['obs_eta_dot_norm']))}")
        print(f"  J_total        {fmt(summ(d['J_total']))}")
        print(f"  J_progress     {fmt(summ(d['J_progress']))}")
        print(f"  J_contour      {fmt(summ(d['J_contour']))}")
        print(f"  J_control      {fmt(summ(d['J_control']))}")
        print(f"  J_smooth       {fmt(summ(d['J_smooth']))}")
        print(f"  J_corridor     {fmt(summ(d['J_corridor']))}")
        print(f"  J_obstacle     {fmt(summ(d['J_obstacle']))}")
        print(f"  J_slosh_eta    {fmt(summ(d['J_slosh_eta']))}")
        print(f"  J_slosh_eta_d  {fmt(summ(d['J_slosh_eta_dot']))}")
        print(f"  cost占比        {cost_share_line(d)}")
        print(f"  corridor_err   {fmt(summ(d['corridor_error']))}")
        print(f"  corridor_viol  {fmt(summ(d['corridor_violation']))}")
        print(f"  corridor_nviol {fmt(summ(d['corridor_viol_count']))}")
        print(f"  guidance_id    {fmt(summ(d['guidance_id']))}")
        print(f"  guidance_bias  {fmt(summ(d['guidance_bias']))}")
        print(f"  primitive_id   {fmt(summ(d['primitive_id']))}")
        print(f"  primitive_top  {top_counts(d['primitive_id'])}")
        print(f"  anti_primitive {anti_primitive_summary(d['primitive_id'])}")
        print(f"  v_scales       start {fmt(summ(d['v_start_scale']))} | mid {fmt(summ(d['v_mid_scale']))} | end {fmt(summ(d['v_end_scale']))}")
        print(f"  omega_scales   start {fmt(summ(d['omega_start_scale']))} | mid {fmt(summ(d['omega_mid_scale']))} | end {fmt(summ(d['omega_end_scale']))}")
        print(f"  slosh_pct(%)   {fmt(summ(d['slosh_total_pct']))}  # 话题 pct(已改为各项绝对值占比, 有界)")
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

        def peak_or0(xs):
            xs = [x for x in xs if x is not None]
            return max(xs) if xs else 0.0

        dv = abs(mean_or0(da["cmd_v"]) - mean_or0(db["cmd_v"]))
        dw = abs(mean_or0(da["cmd_omega"]) - mean_or0(db["cmd_omega"]))
        # 主晃动判定用 observer(实际执行晃动), 而非预测 h_peak(B0 恒为0不可比)。
        oa_mean, ob_mean = mean_or0(da["obs_eta_norm"]), mean_or0(db["obs_eta_norm"])
        oa_peak, ob_peak = peak_or0(da["obs_eta_norm"]), peak_or0(db["obs_eta_norm"])
        print(f"  |Δ mean cmd_v|        = {dv:.5f} m/s")
        print(f"  |Δ mean cmd_omega|    = {dw:.5f} rad/s")
        print(f"  obs_eta_norm mean     {a}={oa_mean:.5f}  {b}={ob_mean:.5f}")
        print(f"  obs_eta_norm peak     {a}={oa_peak:.5f}  {b}={ob_peak:.5f}")

        have_obs = bool(da["obs_eta_norm"]) and bool(db["obs_eta_norm"])
        # 阈值: cmd_v 差 < 5mm/s 且 cmd_omega 差 < 0.01 rad/s 视为"几乎一样"
        if dv < 0.005 and dw < 0.01:
            print()
            print(f"  >>> 结论: {a} 与 {b} 控制输出几乎一致。")
            print("      cost 没有明显改变行为 —— 检查 w_slosh / 候选覆盖 / 后端是否生效。")
        elif not have_obs:
            print()
            print(f"  >>> 结论: {a} 与 {b} 行为有差异, 但缺 observer(/spmpc/debug/slosh_state) 数据,")
            print("      无法判定降晃。请确认录包含该话题且 slosh observer 已配置。")
        else:
            print()
            print(f"  >>> 结论: {a} 与 {b} 产生了可观察差异。")
            better_mean = "更低 (降晃, 符合预期)" if ob_mean < oa_mean else "更高 (反常, 需查 w_slosh 是否过大致行为畸变)"
            better_peak = "更低" if ob_peak < oa_peak else "更高"
            print(f"      observer 实际晃动 {b} 相对 {a}: mean {better_mean}; peak {better_peak}")


if __name__ == "__main__":
    main()
