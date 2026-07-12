#!/usr/bin/env python3
"""Compare real fixed-path TEB and B_ours model-height metrics.

The primary window is normalized fixed-path progress in [0.10, 0.90]. TEB
runs use the recorded standalone /slosh/height when available. Older TEB bags
without that topic are replayed from /odom with the standalone monitor's fixed
20 ms modal model. B_ours uses its recorded internal /spmpc/slosh_height.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path

import rosbag


MODAL_ROOT = 1.8412
RADIUS_M = 0.0185
LIQUID_HEIGHT_M = 0.058
DAMPING_RATIO = 0.05
GRAVITY = 9.81
MODEL_DT_S = 0.02
FILTER_ALPHA = 0.3
MIN_ODOM_DT_S = 0.001
MAX_ODOM_DT_S = 0.1


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def rms(values):
    if not values:
        return math.nan
    return math.sqrt(sum(value * value for value in values) / len(values))


def interpolate(samples, stamp):
    if not samples or stamp < samples[0][0] or stamp > samples[-1][0]:
        return math.nan
    lo = 0
    hi = len(samples) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] <= stamp:
            lo = mid
        else:
            hi = mid
    t0, v0 = samples[lo]
    t1, v1 = samples[hi]
    if t1 <= t0:
        return v0
    weight = (stamp - t0) / (t1 - t0)
    return v0 + weight * (v1 - v0)


def path_length(path_msg):
    points = [pose.pose.position for pose in path_msg.poses]
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))


def second_order_discrete(dt):
    omega_n = math.sqrt(GRAVITY * (MODAL_ROOT / RADIUS_M) *
                        math.tanh(MODAL_ROOT * LIQUID_HEIGHT_M / RADIUS_M))
    decay = math.exp(-DAMPING_RATIO * omega_n * dt)
    omega_d = omega_n * math.sqrt(1.0 - DAMPING_RATIO * DAMPING_RATIO)
    sine = math.sin(omega_d * dt)
    cosine = math.cos(omega_d * dt)
    ratio = DAMPING_RATIO * omega_n / omega_d
    ad00 = decay * (cosine + ratio * sine)
    ad01 = decay * sine / omega_d
    ad10 = decay * (-omega_n * omega_n * sine / omega_d)
    ad11 = decay * (cosine - ratio * sine)

    rhs0 = -ad01
    rhs1 = 1.0 - ad11
    bd1 = rhs0
    bd0 = (-rhs1 - 2.0 * DAMPING_RATIO * omega_n * bd1) / (omega_n * omega_n)
    return (ad00, ad01, ad10, ad11), (bd0, bd1)


def height_coefficient():
    modal_mass_ratio = (
        2.0 * RADIUS_M * math.tanh(MODAL_ROOT * LIQUID_HEIGHT_M / RADIUS_M) /
        (MODAL_ROOT * LIQUID_HEIGHT_M * (MODAL_ROOT * MODAL_ROOT - 1.0))
    )
    return 4.0 * LIQUID_HEIGHT_M * modal_mass_ratio / RADIUS_M


def replay_standalone_height(odom):
    if len(odom) < 2:
        return []
    ad, bd = second_order_discrete(MODEL_DT_S)
    coeff = height_coefficient()
    state = [0.0, 0.0, 0.0, 0.0]
    ax_filtered = 0.0
    ay_filtered = 0.0
    output = [(odom[0][0], 0.0)]
    previous_t, previous_v, previous_w = odom[0]

    for stamp, velocity, omega in odom[1:]:
        dt = stamp - previous_t
        if MIN_ODOM_DT_S <= dt <= MAX_ODOM_DT_S:
            ax_raw = (velocity - previous_v) / dt
            ay_raw = velocity * omega
            ax_filtered = FILTER_ALPHA * ax_raw + (1.0 - FILTER_ALPHA) * ax_filtered
            ay_filtered = FILTER_ALPHA * ay_raw + (1.0 - FILTER_ALPHA) * ay_filtered

            x, vx, y, vy = state
            state = [
                ad[0] * x + ad[1] * vx + bd[0] * ax_filtered,
                ad[2] * x + ad[3] * vx + bd[1] * ax_filtered,
                ad[0] * y + ad[1] * vy + bd[0] * ay_filtered,
                ad[2] * y + ad[3] * vy + bd[1] * ay_filtered,
            ]
        previous_t, previous_v, previous_w = stamp, velocity, omega
        output.append((stamp, coeff * math.hypot(state[0], state[2])))
    return output


def analyze_bag(path):
    path = Path(path)
    is_teb = "TEB" in path.name
    method = "TEB-noobs-fixed" if is_teb else "B_ours_governor_off"
    fixed_path = None
    statuses = []
    commands = []
    odom = []
    teb_tracking = []
    teb_intervention = []
    standalone_height = []
    internal_height = []
    stage0 = []

    topics = [
        "/scout/global_path_fixed", "/cmd_vel", "/odom", "/slosh/height",
        "/spmpc/slosh_height", "/baseline/teb/status", "/spmpc/status",
        "/baseline/teb/tracking_error", "/baseline/teb/command_intervention",
        "/spmpc/debug/stage0_reference",
    ]
    with rosbag.Bag(str(path)) as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            time_s = stamp.to_sec()
            if topic == "/scout/global_path_fixed" and fixed_path is None and len(msg.poses) > 1:
                fixed_path = msg
            elif topic in ("/baseline/teb/status", "/spmpc/status"):
                statuses.append((time_s, msg.data))
            elif topic == "/cmd_vel":
                commands.append((time_s, msg.linear.x, msg.angular.z))
            elif topic == "/odom":
                odom.append((time_s, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            elif topic == "/baseline/teb/tracking_error" and len(msg.data) >= 7:
                teb_tracking.append((time_s, msg.data[4], msg.data[0], msg.data[1]))
            elif topic == "/baseline/teb/command_intervention" and len(msg.data) >= 8:
                teb_intervention.append((time_s, msg.data[6], msg.data[7]))
            elif topic == "/slosh/height":
                standalone_height.append((time_s, float(msg.data)))
            elif topic == "/spmpc/slosh_height":
                internal_height.append((time_s, float(msg.data)))
            elif topic == "/spmpc/debug/stage0_reference" and len(msg.data) >= 11:
                stage0.append((time_s, msg.data[0], msg.data[9], msg.data[10]))

    if fixed_path is None:
        raise RuntimeError(f"{path}: missing fixed path")
    length_m = path_length(fixed_path)
    if is_teb:
        progress = [(t, ratio) for t, ratio, _, _ in teb_tracking]
        projection = [(t, distance) for t, ratio, distance, _ in teb_tracking]
        if standalone_height:
            height = standalone_height
            height_source = "/slosh/height"
        else:
            height = replay_standalone_height(odom)
            height_source = "offline_standalone_replay_from_odom"
        height_scale_to_mm = 1000.0
        tracking_start = next((t for t, value in statuses if value == "TRACKING"), math.nan)
        goal_time = next((t for t, value in statuses if value == "GOAL_REACHED"), math.nan)
        start_time = tracking_start
        linear_limited = [value for _, value, _ in teb_intervention]
        angular_limited = [value for _, _, value in teb_intervention]
    else:
        progress = [(t, max(0.0, min(1.0, s0 / length_m))) for t, s0, _, _ in stage0]
        projection = [(t, math.hypot(contour, lag)) for t, _, contour, lag in stage0]
        height = internal_height
        height_source = "/spmpc/slosh_height"
        # The SPMPC diagnostics publisher converts its SI model state to mm.
        height_scale_to_mm = 1.0
        start_time = next((t for t, value in statuses if "ACADOS_OK" in value), math.nan)
        goal_time = next((t for t, value in statuses if "GOAL_REACHED" in value), math.nan)
        linear_limited = []
        angular_limited = []

    def in_core(stamp):
        ratio = interpolate(progress, stamp)
        return math.isfinite(ratio) and 0.10 <= ratio <= 0.90

    core_height_mm = [height_scale_to_mm * abs(value) for stamp, value in height if in_core(stamp)]
    common_replay = replay_standalone_height(odom)
    common_height_mm = [1000.0 * abs(value) for stamp, value in common_replay if in_core(stamp)]
    core_commands = [(v, w) for stamp, v, w in commands if in_core(stamp)]
    core_projection = [abs(value) for stamp, value in projection if in_core(stamp)]
    core_odom = [(abs(v), abs(w)) for stamp, v, w in odom if in_core(stamp)]
    progress_values = [ratio for _, ratio in progress]
    success = math.isfinite(goal_time)

    return {
        "bag": str(path),
        "run": path.stem,
        "method": method,
        "height_source": height_source,
        "success": int(success),
        "goal_time_s": goal_time - start_time if success and math.isfinite(start_time) else math.nan,
        "path_length_m": length_m,
        "progress_max": max(progress_values) if progress_values else math.nan,
        "model_sample_count_10_90": len(core_height_mm),
        "model_mean_mm_10_90": statistics.fmean(core_height_mm) if core_height_mm else math.nan,
        "model_p95_mm_10_90": percentile(core_height_mm, 0.95),
        "model_max_mm_10_90": max(core_height_mm) if core_height_mm else math.nan,
        "model_rms_mm_10_90": rms(core_height_mm),
        "common_eval_mean_mm_10_90": statistics.fmean(common_height_mm) if common_height_mm else math.nan,
        "common_eval_p95_mm_10_90": percentile(common_height_mm, 0.95),
        "common_eval_max_mm_10_90": max(common_height_mm) if common_height_mm else math.nan,
        "common_eval_rms_mm_10_90": rms(common_height_mm),
        "cmd_v_mean_mps_10_90": statistics.fmean(v for v, _ in core_commands) if core_commands else math.nan,
        "cmd_v_p95_mps_10_90": percentile([abs(v) for v, _ in core_commands], 0.95),
        "cmd_w_abs_p95_radps_10_90": percentile([abs(w) for _, w in core_commands], 0.95),
        "cmd_w_abs_max_radps_10_90": max([abs(w) for _, w in core_commands], default=math.nan),
        "odom_v_abs_mean_mps_10_90": statistics.fmean(v for v, _ in core_odom) if core_odom else math.nan,
        "projection_p95_m_10_90": percentile(core_projection, 0.95),
        "projection_max_m_10_90": max(core_projection) if core_projection else math.nan,
        "linear_limited_frac": statistics.fmean(linear_limited) if linear_limited else math.nan,
        "angular_limited_frac": statistics.fmean(angular_limited) if angular_limited else math.nan,
        "bag_duration_s": bag_end - bag_start,
    }


def aggregate(rows):
    output = []
    for method in sorted({row["method"] for row in rows}):
        group = [row for row in rows if row["method"] == method]
        result = {"method": method, "run_count": len(group), "success_count": sum(r["success"] for r in group)}
        for key in (
            "goal_time_s", "model_mean_mm_10_90", "model_p95_mm_10_90",
            "model_max_mm_10_90", "model_rms_mm_10_90", "common_eval_mean_mm_10_90",
            "common_eval_p95_mm_10_90", "common_eval_max_mm_10_90",
            "common_eval_rms_mm_10_90", "cmd_v_mean_mps_10_90",
            "cmd_w_abs_p95_radps_10_90", "cmd_w_abs_max_radps_10_90",
            "projection_p95_m_10_90", "projection_max_m_10_90",
        ):
            values = [row[key] for row in group if math.isfinite(row[key])]
            result[f"{key}_mean"] = statistics.fmean(values) if values else math.nan
            result[f"{key}_std"] = statistics.stdev(values) if len(values) > 1 else 0.0 if values else math.nan
        output.append(result)
    return output


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(rows[0].keys()), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--run-csv", type=Path, required=True)
    parser.add_argument("--aggregate-csv", type=Path, required=True)
    args = parser.parse_args()
    rows = [analyze_bag(path) for path in args.bags]
    write_csv(args.run_csv, rows)
    write_csv(args.aggregate_csv, aggregate(rows))
    for row in rows:
        print(
            f"{row['run']}: success={row['success']} goal={row['goal_time_s']:.3f}s "
            f"p95/max/rms={row['model_p95_mm_10_90']:.3f}/"
            f"{row['model_max_mm_10_90']:.3f}/{row['model_rms_mm_10_90']:.3f}mm "
            f"source={row['height_source']}"
        )


if __name__ == "__main__":
    main()
