"""F 层 — 候选几何/碰撞/执行可行性门。

从 anti_slosh_path_post_processor.py 的 check_candidate_collision (grid 层)
和 evaluate_candidate (geometry gates + geometry score) 迁移而来。
纯 Python，无 ROS 依赖。
"""

import math
from reference_generation.geometry_candidates import path_metrics
from oscrs.path_utils import direction_preserved, endpoint_error, safe_ratio


def check_collision_on_grid(points, ox, oy, res, width, height, data,
                             threshold, unknown_is_obstacle):
    """对已解包的 costmap grid 数据做 point-cost 碰撞检查。

    Returns:
        (status, idx, cost) — status ∈ {"accepted", "collision"}
    """
    for k, (x, y) in enumerate(points):
        ix = int(math.floor((x - ox) / res))
        iy = int(math.floor((y - oy) / res))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            if unknown_is_obstacle:
                return ("collision", k, -1)
            continue
        cost = data[iy * width + ix]
        if cost < 0:
            if unknown_is_obstacle:
                return ("collision", k, int(cost))
            continue
        if cost >= threshold:
            return ("collision", k, int(cost))
    return ("accepted", -1, -1)


def evaluate_geometry_feasibility(candidate, name, base, base_metrics, base_length,
                                   min_segment_length, max_drift, max_length_ratio,
                                   min_length_ratio, min_kappa_ratio, target_kappa_ratio,
                                   max_endpoint_error, ay_ratio_limit,
                                   predicted_ay_p95, predicted_vmax, base_predicted_ay_p95,
                                   base_predicted_vmax,
                                   candidate_levels, max_candidate_level,
                                   collision_status, collision_idx, collision_cost,
                                   w_kappa, w_dkappa, w_length, w_drift,
                                   w_shortening, w_over_smooth):
    """对一条候选执行全部 F gate，返回 gate 结果和 geometry score。

    碰撞结果 (collision_status/idx/cost) 由调用方预先通过 check_candidate_collision
    (ROS adapter) 或 check_collision_on_grid (纯函数) 获得后传入。

    Returns:
        dict with keys matching the legacy evaluate_candidate row fields.
    """
    metrics = path_metrics(candidate, base)
    length_ratio = metrics["length_m"] / base_length
    end_error = endpoint_error(candidate, base)
    reject_reasons = []
    ay_ratio = safe_ratio(predicted_ay_p95, base_predicted_ay_p95)

    if len(candidate) < 3:
        reject_reasons.append(f"too_few_points:{len(candidate)}")
    if name != "original" and metrics["min_seg_m"] < min_segment_length:
        reject_reasons.append(f"min_seg:{metrics['min_seg_m']:.3f}<{min_segment_length:.3f}")
    if metrics["max_drift_m"] > max_drift:
        reject_reasons.append(f"drift:{metrics['max_drift_m']:.3f}>{max_drift:.3f}")
    if length_ratio > max_length_ratio:
        reject_reasons.append(f"length:{length_ratio:.3f}>{max_length_ratio:.3f}")
    if length_ratio < min_length_ratio:
        reject_reasons.append(f"short:{length_ratio:.3f}<{min_length_ratio:.3f}")
    kappa_ratio = safe_ratio(metrics["kappa_p95"], base_metrics["kappa_p95"])
    dkappa_ratio = safe_ratio(metrics["dkappa_p95"], base_metrics["dkappa_p95"])
    if candidate_levels.get(name, 0) > candidate_levels[max_candidate_level]:
        reject_reasons.append(f"level:{name}>{max_candidate_level}")
    if ay_ratio > ay_ratio_limit:
        reject_reasons.append(f"ay:{ay_ratio:.3f}>{ay_ratio_limit:.3f}")
    if end_error > max_endpoint_error:
        reject_reasons.append(f"endpoint:{end_error:.3f}>{max_endpoint_error:.3f}")
    if not direction_preserved(candidate, base):
        reject_reasons.append("direction")

    if collision_status == "collision":
        reject_reasons.append(f"collision:idx={collision_idx}:cost={collision_cost}")
    elif collision_status == "no_costmap":
        reject_reasons.append("no_costmap")
    elif collision_status == "frame_mismatch":
        reject_reasons.append("frame_mismatch")

    accepted = not reject_reasons

    shortening_penalty = max(0.0, min_length_ratio - length_ratio)
    over_smooth_penalty = max(0.0, min_kappa_ratio - kappa_ratio)
    target_kappa_penalty = abs(kappa_ratio - target_kappa_ratio)
    geometry_score = (
        w_kappa * target_kappa_penalty
        + w_dkappa * dkappa_ratio
        + w_length * max(0.0, length_ratio - 1.0)
        + w_drift * metrics["max_drift_m"]
        + w_shortening * shortening_penalty
        + w_over_smooth * over_smooth_penalty
    )

    return {
        "accepted": accepted,
        "reject_reason": "accepted" if accepted else "|".join(reject_reasons),
        "geometry_score": geometry_score,
        "length_ratio": length_ratio,
        "kappa_ratio": kappa_ratio,
        "dkappa_ratio": dkappa_ratio,
        "predicted_ay_p95": predicted_ay_p95,
        "predicted_ay_ratio": ay_ratio,
        "predicted_vmax": predicted_vmax,
        "base_predicted_ay_p95": base_predicted_ay_p95,
        "base_predicted_vmax": base_predicted_vmax,
        "target_kappa_penalty": target_kappa_penalty,
        "endpoint_error_m": end_error,
        "collision_status": collision_status,
        "collision_idx": collision_idx,
        "collision_cost": collision_cost,
        **metrics,
    }
