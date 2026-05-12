#!/usr/bin/env python3
"""Offline tuning diagnosis for real-world scout_local_planner bags.

This script is intentionally heuristic. It does not try to "prove" the root
cause; it gives a short list of likely parameters to tune next based on the bag
signals that are already being recorded by record_slosh_experiment.sh.

Typical usage:
  python3 diagnose_real_tuning.py <bag>

Recommended usage on the robot IPC:
  source /opt/ros/noetic/setup.bash
  python3 diagnose_real_tuning.py /path/to/your.bag
"""

import argparse
import math
import statistics
import sys
from collections import Counter, defaultdict

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag not found. Source your ROS environment first.", file=sys.stderr)
    sys.exit(1)


TOPICS = [
    "/cmd_vel",
    "/odom",
    "/mpc/status_val",
    "/mpc_status",
    "/terminal/mode",
    "/terminal/recovery_latched",
    "/slosh/v_des_eff",
    "/slosh/speed_governor_active",
    "/risk_scheduler/rho_k",
    "/risk_scheduler/fallback_active",
    "/scout/global_path_smooth",
    "/mpc/reference_path",
    "/local_path",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input rosbag path")
    parser.add_argument("--slow-threshold", type=float, default=0.30,
                        help="Linear speed below this is considered slow [m/s]")
    parser.add_argument("--track-dist-threshold", type=float, default=0.12,
                        help="Heuristic p95 path distance threshold [m]")
    parser.add_argument("--heading-threshold-deg", type=float, default=15.0,
                        help="Heuristic p95 heading error threshold [deg]")
    parser.add_argument("--brief", action="store_true",
                        help="Only print the top recommended next-step parameter changes")
    return parser.parse_args()


def summarize(values):
    if not values:
        return None
    vals = sorted(values)
    return {
        "n": len(vals),
        "min": vals[0],
        "max": vals[-1],
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
        "p95": vals[max(0, int(0.95 * (len(vals) - 1)))],
    }


def fmt_stats(name, values, unit=""):
    s = summarize(values)
    if s is None:
        return f"- {name}: n=0"
    return (
        f"- {name}: n={s['n']} min={s['min']:.4g}{unit} max={s['max']:.4g}{unit} "
        f"mean={s['mean']:.4g}{unit} median={s['median']:.4g}{unit} p95={s['p95']:.4g}{unit}"
    )


def nearest_value(series, t, idx=0):
    if not series:
        return idx, None
    while idx + 1 < len(series) and abs(series[idx + 1][0] - t) <= abs(series[idx][0] - t):
        idx += 1
    return idx, series[idx][1]


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def path_points(msg):
    pts = []
    for pose_stamped in msg.poses:
        x = float(pose_stamped.pose.position.x)
        y = float(pose_stamped.pose.position.y)
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    return pts


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def triangle_area2(a, b, c):
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))


def curvature_three_points(a, b, c):
    ab = dist(a, b)
    bc = dist(b, c)
    ac = dist(a, c)
    denom = ab * bc * ac
    if denom < 1e-9:
        return None
    return 2.0 * triangle_area2(a, b, c) / denom


def analyze_polyline(points):
    if len(points) < 3:
        return {"n": len(points), "max_kappa": 0.0, "max_dkappa": 0.0, "min_seg": 0.0}
    segs = [dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    min_seg = min(segs) if segs else 0.0
    kappas = []
    centers_s = []
    s_acc = 0.0
    for i in range(1, len(points) - 1):
        kappa = curvature_three_points(points[i - 1], points[i], points[i + 1])
        if kappa is not None:
            kappas.append(kappa)
            centers_s.append(s_acc + 0.5 * segs[i - 1])
        s_acc += segs[i - 1]
    max_dkappa = 0.0
    for i in range(1, len(kappas)):
        ds = max(1e-6, centers_s[i] - centers_s[i - 1])
        max_dkappa = max(max_dkappa, abs(kappas[i] - kappas[i - 1]) / ds)
    return {
        "n": len(points),
        "max_kappa": max(kappas) if kappas else 0.0,
        "max_dkappa": max_dkappa,
        "min_seg": min_seg,
    }


def nearest_path_error(pose_xy, pose_yaw, points):
    """Return nearest distance and local path heading for a polyline."""
    if len(points) < 2:
        return None
    px, py = pose_xy
    best_dist = float("inf")
    best_heading = None
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            continue
        apx = px - ax
        apy = py - ay
        u = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
        qx = ax + u * abx
        qy = ay + u * aby
        d = math.hypot(px - qx, py - qy)
        if d < best_dist:
            best_dist = d
            best_heading = math.atan2(aby, abx)
    if best_heading is None:
        return None
    heading_err = abs(wrap_to_pi(pose_yaw - best_heading))
    return best_dist, heading_err


def add_recommendation(recs, category, why, params):
    recs.append({
        "category": category,
        "why": why,
        "params": params,
    })


def extract_param_key(param_line):
    head = param_line.split("->", 1)[0].strip()
    return head if "/" in head else None


def print_brief_recommendations(recs, limit=3):
    if not recs:
        print("Top Next Changes")
        print("- 没有命中明显单一瓶颈。优先人工查看 /slosh/v_des_eff、/cmd_vel、/terminal/mode、/local_path。")
        return

    chosen = []
    seen = set()
    for rec in recs:
        for param in rec["params"]:
            key = extract_param_key(param)
            if key is None or key in seen:
                continue
            seen.add(key)
            chosen.append((key, rec["category"], param))
            if len(chosen) >= limit:
                break
        if len(chosen) >= limit:
            break

    print("Top Next Changes")
    if not chosen:
        print("- 没有抽取到明确参数键。请查看完整建议。")
        return
    for i, (key, category, param) in enumerate(chosen, start=1):
        print(f"{i}. [{category}] {param}")


def main():
    args = parse_args()
    data = defaultdict(list)
    path_metrics = defaultdict(list)

    try:
        bag = rosbag.Bag(args.bag)
    except rosbag.bag.ROSBagUnindexedException:
        print("ERROR: bag is unindexed. Run `rosbag reindex <bag>` first.", file=sys.stderr)
        return 2

    with bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            t = stamp.to_sec() - start
            if topic == "/cmd_vel":
                data[topic].append((t, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                data[topic].append((t, {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": yaw_from_quat(msg.pose.pose.orientation),
                    "v": float(msg.twist.twist.linear.x),
                    "w": float(msg.twist.twist.angular.z),
                }))
            elif topic in ("/mpc_status", "/terminal/mode"):
                data[topic].append((t, str(msg.data)))
            elif topic in ("/scout/global_path_smooth", "/mpc/reference_path", "/local_path"):
                pts = path_points(msg)
                data[topic].append((t, pts))
                path_metrics[topic].append((t, analyze_polyline(pts)))
            else:
                data[topic].append((t, msg.data))

    duration = end - start
    cmd_v = [v for _, (v, _) in data["/cmd_vel"]]
    cmd_w = [w for _, (_, w) in data["/cmd_vel"]]
    odom_v = [s["v"] for _, s in data["/odom"]]
    v_des = [float(v) for _, v in data["/slosh/v_des_eff"]]
    solve_status = [int(v) for _, v in data["/mpc/status_val"]]
    rho_vals = [float(v) for _, v in data["/risk_scheduler/rho_k"]]
    fallback_vals = [bool(v) for _, v in data["/risk_scheduler/fallback_active"]]
    terminal_modes = Counter(v for _, v in data["/terminal/mode"])
    mpc_modes = Counter(v for _, v in data["/mpc_status"])
    terminal_latched = [int(v) for _, v in data["/terminal/recovery_latched"]]
    gov_active = [int(v) for _, v in data["/slosh/speed_governor_active"]]

    print(f"bag: {args.bag}")
    print(f"duration: {duration:.2f}s")
    print("\nSignals")
    print(fmt_stats("/cmd_vel.linear.x", cmd_v, " m/s"))
    print(fmt_stats("/cmd_vel.angular.z", cmd_w, " rad/s"))
    print(fmt_stats("/odom.twist.linear.x", odom_v, " m/s"))
    print(fmt_stats("/slosh/v_des_eff", v_des, " m/s"))
    print(fmt_stats("/risk_scheduler/rho_k", rho_vals))

    success_ratio = (sum(1 for v in solve_status if v == 1) / len(solve_status)) if solve_status else float("nan")
    print(f"- /mpc/status_val success_ratio={success_ratio:.3f}" if solve_status else "- /mpc/status_val: n=0")

    print("\nModes")
    print(f"- /mpc_status: {dict(mpc_modes)}")
    print(f"- /terminal/mode: {dict(terminal_modes)}")

    # Track error against nearest reference path during TRACKING.
    track_dists = []
    heading_errs = []
    mode_idx = 0
    ref_idx = 0
    for t, odom_state in data["/odom"]:
        mode_idx, mode = nearest_value(data["/mpc_status"], t, mode_idx)
        if mode != "TRACKING":
            continue
        ref_idx, ref_pts = nearest_value(data["/mpc/reference_path"], t, ref_idx)
        if not ref_pts:
            continue
        sample = nearest_path_error((odom_state["x"], odom_state["y"]), odom_state["yaw"], ref_pts)
        if sample is None:
            continue
        d, h = sample
        track_dists.append(d)
        heading_errs.append(math.degrees(h))

    print("\nTracking Error (vs nearest /mpc/reference_path)")
    print(fmt_stats("distance", track_dists, " m"))
    print(fmt_stats("heading_err", heading_errs, " deg"))

    print("\nPath Geometry")
    for topic in ("/scout/global_path_smooth", "/mpc/reference_path", "/local_path"):
        max_k = [m["max_kappa"] for _, m in path_metrics[topic]]
        max_dk = [m["max_dkappa"] for _, m in path_metrics[topic]]
        print(f"- {topic}:")
        print(fmt_stats("  max_kappa", max_k, " 1/m"))
        print(fmt_stats("  max_dkappa", max_dk, " 1/m^2"))

    recs = []

    vdes_stats = summarize(v_des) or {}
    cmd_stats = summarize(cmd_v) or {}
    dist_stats = summarize(track_dists) or {}
    heading_stats = summarize(heading_errs) or {}
    ref_k_stats = summarize([m["max_kappa"] for _, m in path_metrics["/mpc/reference_path"]]) or {}
    ref_dk_stats = summarize([m["max_dkappa"] for _, m in path_metrics["/mpc/reference_path"]]) or {}
    local_k_stats = summarize([m["max_kappa"] for _, m in path_metrics["/local_path"]]) or {}

    terminal_total = sum(terminal_modes.values()) or 1
    terminal_ratio = sum(
        terminal_modes.get(m, 0)
        for m in ("APPROACH_POINT", "ALIGN_TO_POINT", "ALIGN_FINAL_YAW", "TERMINAL_LATCHED", "GOAL_STOP_PENDING")
    ) / terminal_total
    latched_ratio = (sum(1 for v in terminal_latched if v > 0) / len(terminal_latched)) if terminal_latched else 0.0
    governor_ratio = (sum(1 for v in gov_active if v > 0) / len(gov_active)) if gov_active else 0.0
    fallback_ratio = (sum(1 for v in fallback_vals if v) / len(fallback_vals)) if fallback_vals else 0.0

    if solve_status and success_ratio < 0.95:
        add_recommendation(
            recs,
            "QP/约束可行性",
            f"求解成功率只有 {success_ratio:.3f}，先保证可行性，再谈速度和贴合。",
            [
                "vehicle/v_max -> 小步下调",
                "vehicle/omega_max -> 小步下调",
                "vehicle/alpha_max -> 小步上调或下调需结合日志",
                "path_handler/max_lat_accel -> 小步下调",
                "若在 anti-slosh 实验，先关闭 enable_slosh_box_constraint 与 speed_governor",
            ],
        )

    if terminal_ratio > 0.20 or latched_ratio > 0.20:
        add_recommendation(
            recs,
            "近终点恢复过早介入",
            f"terminal 模式占比 {terminal_ratio:.2%}，recovery_latched 占比 {latched_ratio:.2%}，速度慢可能不是 TRACKING 本体，而是 near-goal 逻辑在主导。",
            [
                "terminal_recovery/enter_distance -> 下调",
                "terminal_recovery/v_max -> 上调",
                "path_handler/goal_capture_distance -> 下调",
                "path_handler/goal_tolerance, yaw_tolerance -> 仅在确有必要时小幅放宽",
            ],
        )

    if governor_ratio > 0.20:
        add_recommendation(
            recs,
            "残余晃动 governor 在主动压速",
            f"/slosh/speed_governor_active 占比 {governor_ratio:.2%}。",
            [
                "slosh_speed_governor/enable -> 基线调参阶段先关",
                "slosh_speed_governor/k_eta -> 下调",
                "slosh_speed_governor/ay_max_base -> 上调",
                "slosh_speed_governor/preview_distance -> 下调",
            ],
        )

    if fallback_ratio > 0.20:
        add_recommendation(
            recs,
            "RiskScheduler fallback 频繁",
            f"/risk_scheduler/fallback_active 占比 {fallback_ratio:.2%}。",
            [
                "检查 /imu/data 是否稳定",
                "检查 IMU bias 初始化是否完成",
                "risk_scheduler/enable -> 基线调参阶段先关",
            ],
        )

    if vdes_stats and cmd_stats:
        vdes_p95 = vdes_stats["p95"]
        cmd_p95 = cmd_stats["p95"]
        if vdes_p95 < args.slow_threshold:
            why = (
                f"v_des_eff p95={vdes_p95:.3f} m/s，本身就很低，说明速度在进 MPC 前已经被压住。"
            )
            params = [
                "优先看 safety/tracking_curvature_speed_cap_enable",
                "vehicle/alpha_max -> 上调（若 dkappa 限速主导）",
                "path_handler/speed_profile_omega_max -> 上调（若 kappa/omega 限速主导）",
                "path_handler/max_lat_accel -> 上调（若横向加速度限速主导）",
            ]
            if ref_dk_stats and ref_dk_stats["p95"] > 20.0:
                why += f" 当前 reference_path 的 max_dkappa p95={ref_dk_stats['p95']:.1f} 1/m²，dkappa 限速很可能在压速。"
            add_recommendation(recs, "参考速度先天过低", why, params)
        elif cmd_p95 < 0.8 * vdes_p95 and success_ratio >= 0.95:
            add_recommendation(
                recs,
                "执行响应偏慢",
                f"cmd_vel p95={cmd_p95:.3f} m/s，显著低于 v_des_eff p95={vdes_p95:.3f} m/s，且求解成功率正常，更像执行层迟滞或转向建立过慢。",
                [
                    "filter/alpha_v -> 1.0",
                    "filter/alpha_omega -> 1.0",
                    "filter/kappa_boost -> 0.0",
                    "vehicle/alpha_max -> 上调",
                    "mpc/R_domega -> 下调一点",
                ],
            )

    if dist_stats and heading_stats:
        d95 = dist_stats["p95"]
        h95 = heading_stats["p95"]
        if d95 > args.track_dist_threshold:
            if h95 > args.heading_threshold_deg:
                add_recommendation(
                    recs,
                    "弯道跟不上，先提响应而不是先猛加误差权重",
                    f"reference_path 贴合误差 p95={d95:.3f} m，航向误差 p95={h95:.1f} deg，说明主要是姿态/转向建立不够快。",
                    [
                        "path_handler/lookahead_distance -> 下调（例如 0.70 -> 0.60 或 0.55）",
                        "vehicle/alpha_max -> 上调",
                        "mpc/R_domega -> 下调一点",
                        "若仍跟不上，再小步上调 vehicle/omega_max",
                    ],
                )
            else:
                add_recommendation(
                    recs,
                    "路径贴合意愿不足或前视过大",
                    f"reference_path 贴合误差 p95={d95:.3f} m，但航向误差 p95={h95:.1f} deg 不算大，更像 contour 跟踪不够强或前视太远。",
                    [
                        "path_handler/lookahead_distance -> 下调",
                        "mpc/Q_contour -> 上调",
                        "mpc/Q_ec -> 小步上调",
                        "必要时 mpc/Q_etheta -> 小步上调",
                    ],
                )

    if local_k_stats and ref_k_stats and local_k_stats["p95"] > 5.0 * max(ref_k_stats["p95"], 1e-6):
        add_recommendation(
            recs,
            "local_path 几何明显比 reference_path 更尖",
            f"local_path max_kappa p95={local_k_stats['p95']:.2f}，reference_path max_kappa p95={ref_k_stats['p95']:.2f}。",
            [
                "先检查定位噪声和 /local_path 是否出现短段",
                "path_handler/lookahead_distance -> 小步下调",
                "若是速度慢问题已解决但转弯仍切弯，优先调响应参数，不要先继续加局部限速",
            ],
        )

    print("\nTuning Suggestions")
    if not recs:
        print("- 没有命中明显单一瓶颈。优先人工查看 /cmd_vel、/slosh/v_des_eff、/terminal/mode、/local_path。")
        print("- 若当前主要问题是弯道贴合差，优先顺序建议：lookahead_distance -> alpha_max -> R_domega -> Q_contour/Q_etheta。")
        if args.brief:
            print()
            print_brief_recommendations(recs)
        return 0

    for i, rec in enumerate(recs, start=1):
        print(f"{i}. {rec['category']}")
        print(f"   判断: {rec['why']}")
        print("   建议参数:")
        for p in rec["params"]:
            print(f"   - {p}")

    print("\nRecommended Order")
    print("1. 先处理会破坏可比性的基础问题：QP 可行性、terminal 早介入、risk fallback。")
    print("2. 再区分是 v_des_eff 本身太低，还是 cmd_vel 跟不上 v_des_eff。")
    print("3. 若弯道贴合差，优先调响应链：lookahead_distance -> alpha_max -> R_domega。")
    print("4. 最后再动误差权重：Q_contour / Q_ec / Q_etheta。")

    if args.brief:
        print()
        print_brief_recommendations(recs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
