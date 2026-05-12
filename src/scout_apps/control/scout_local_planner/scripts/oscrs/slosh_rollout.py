"""R 层 — 晃动动力学 rollout 与指标预测。

从 anti_slosh_path_post_processor.py 的 forward_profile / rollout_slosh_metrics /
apply_slosh_scores 迁移而来。所有函数为纯 Python，无 ROS 依赖。
"""

import math
from reference_generation.geometry_candidates import (
    cumulative_s, curvature_series, dkappa_series, percentile,
)


def forward_profile(points, predict_v_max, predict_ay_max, predict_a_max,
                    predict_v_init, offset_x=0.0, offset_y=0.0):
    """沿路径预测执行速度/加速度剖面（含容器偏移修正）。

    Returns:
        (s, kappa, v_values, ax_values, ay_values)
    """
    s = cumulative_s(points)
    kappa = curvature_series(points)
    dkappa = dkappa_series(points, kappa)
    v_prev = min(predict_v_init, predict_v_max)
    v_values = []
    ax_values = []
    ay_values = []
    for i, k in enumerate(kappa):
        if i == 0:
            ds = 0.0
        else:
            ds = max(0.0, s[i] - s[i - 1])
        k_abs = abs(k)
        if k_abs > 1e-6:
            v_curv = math.sqrt(predict_ay_max / k_abs)
        else:
            v_curv = predict_v_max
        v_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * predict_a_max * ds))
        v = min(predict_v_max, v_curv, v_accel)
        ax = (v * v - v_prev * v_prev) / (2.0 * ds) if ds > 1e-6 else 0.0
        omega = v * k
        alpha = ax * k + v * v * dkappa[i]
        ax_eff = ax - alpha * offset_y - omega * omega * offset_x
        ay_eff = v * v * k + alpha * offset_x - omega * omega * offset_y
        v_values.append(v)
        ax_values.append(ax_eff)
        ay_values.append(ay_eff)
        v_prev = v
    return s, kappa, v_values, ax_values, ay_values


def interp_piecewise(times, values, query_t):
    """分段线性插值。"""
    if not times:
        return 0.0
    if query_t <= times[0]:
        return values[0]
    if query_t >= times[-1]:
        return values[-1]
    lo = 0
    hi = len(times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < query_t:
            lo = mid + 1
        else:
            hi = mid
    i = max(1, lo)
    t0 = times[i - 1]
    t1 = times[i]
    if t1 <= t0:
        return values[i]
    r = (query_t - t0) / (t1 - t0)
    return values[i - 1] * (1.0 - r) + values[i] * r


def rollout_slosh_metrics(points, *,
                          predict_v_max, predict_ay_max, predict_a_max,
                          predict_v_init, offset_x, offset_y,
                          omega_n, zeta, rollout_dt, v_floor,
                          height_coeff, container_radius, use_parabola,
                          settle_duration, enable_shadow_or_active):
    """对路径执行 2 阶 ODE slosh 仿真，输出物理晃动指标。

    Returns:
        dict with keys: slosh_h_p95, slosh_h_max, slosh_h_residual_max,
        slosh_h_modal_p95, slosh_h_parabola_p95, slosh_eta_dot_rms,
        slosh_energy_rms, slosh_terminal_E
    """
    if len(points) < 2:
        return {
            "slosh_h_p95": float("inf"),
            "slosh_h_max": float("inf"),
            "slosh_h_residual_max": float("inf"),
            "slosh_h_modal_p95": float("inf"),
            "slosh_h_parabola_p95": float("inf"),
            "slosh_eta_dot_rms": float("inf"),
            "slosh_energy_rms": float("inf"),
            "slosh_terminal_E": float("inf"),
        }
    s, kappa, _, ax_values, ay_values = forward_profile(
        points, predict_v_max, predict_ay_max, predict_a_max,
        predict_v_init, offset_x, offset_y,
    )
    times = [0.0]
    _, _, v_values, _, _ = forward_profile(
        points, predict_v_max, predict_ay_max, predict_a_max,
        predict_v_init, offset_x, offset_y,
    )
    omega_values = [v * k for v, k in zip(v_values, kappa)]
    for i in range(1, len(s)):
        ds = max(0.0, s[i] - s[i - 1])
        times.append(times[-1] + ds / max(v_floor, v_values[i]))

    eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
    eta_dot_norm = []
    energy = []
    height_modal = []
    height_parabola = []
    height_total = []
    wn2 = omega_n * omega_n
    damping = 2.0 * zeta * omega_n
    radius2_over_4g = (container_radius * container_radius) / (4.0 * 9.81)
    t_end = times[-1] if times else 0.0
    t_final = t_end + (settle_duration if enable_shadow_or_active else 0.0)
    steps = max(1, int(math.ceil(t_final / rollout_dt)))
    for step in range(steps + 1):
        query_t = step * rollout_dt
        in_motion = query_t <= t_end
        if in_motion:
            ux = interp_piecewise(times, ax_values, query_t)
            uy = interp_piecewise(times, ay_values, query_t)
            omega = interp_piecewise(times, omega_values, query_t)
        else:
            ux = uy = omega = 0.0
        ddx = -damping * eta_x_dot - wn2 * eta_x - ux
        ddy = -damping * eta_y_dot - wn2 * eta_y - uy
        eta_x_dot += ddx * rollout_dt
        eta_y_dot += ddy * rollout_dt
        eta_x += eta_x_dot * rollout_dt
        eta_y += eta_y_dot * rollout_dt
        eta_dot = math.hypot(eta_x_dot, eta_y_dot)
        e = wn2 * (eta_x * eta_x + eta_y * eta_y) + eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot
        modal = height_coeff * math.hypot(eta_x, eta_y)
        parabola = radius2_over_4g * omega * omega if use_parabola else 0.0
        if in_motion:
            eta_dot_norm.append(eta_dot)
            energy.append(e)
            height_modal.append(modal)
            height_parabola.append(parabola)
            height_total.append(modal + parabola)
        else:
            height_total.append(modal + parabola)
    tracking_count = len(eta_dot_norm)
    residual_height = height_total[tracking_count:] if len(height_total) > tracking_count else []
    tracking_height = height_total[:tracking_count] if tracking_count > 0 else height_total
    return {
        "slosh_h_p95": percentile(tracking_height, 95.0),
        "slosh_h_max": max(tracking_height) if tracking_height else float("inf"),
        "slosh_h_residual_max": max(residual_height) if residual_height else 0.0,
        "slosh_h_modal_p95": percentile(height_modal, 95.0),
        "slosh_h_parabola_p95": percentile(height_parabola, 95.0),
        "slosh_eta_dot_rms": math.sqrt(sum(v * v for v in eta_dot_norm) / len(eta_dot_norm)) if eta_dot_norm else float("inf"),
        "slosh_energy_rms": math.sqrt(sum(v * v for v in energy) / len(energy)) if energy else float("inf"),
        "slosh_terminal_E": energy[-1] if energy else float("inf"),
    }


def predicted_lateral_profile(points, predict_v_max, predict_ay_max,
                               predict_a_max, predict_v_init, offset_x, offset_y):
    """Forward speed rollout，返回 (ay_p95, vmax)。"""
    if len(points) < 2:
        return (float("inf"), 0.0)
    _, _, v_values, _, ay_values = forward_profile(
        points, predict_v_max, predict_ay_max, predict_a_max,
        predict_v_init, offset_x, offset_y,
    )
    return (percentile([abs(v) for v in ay_values], 95.0), max(v_values) if v_values else 0.0)


def apply_slosh_scores(rows, w_h, w_energy, w_eta_dot, w_terminal,
                       w_kappa, w_dkappa, w_ay, w_length, w_drift):
    """旧版 slosh weighted score（在 OSCRS batch-norm score 之前）。

    修改 rows 中各 row["slosh_score"] 和 row["score"]。
    """
    accepted = [row for row, _ in rows if row["accepted"]]
    if len(accepted) < 2:
        return
    max_h = max(max(0.0, row["slosh_h_p95"]) for row in accepted)
    max_energy = max(max(0.0, row["slosh_energy_rms"]) for row in accepted)
    max_eta_dot = max(max(0.0, row["slosh_eta_dot_rms"]) for row in accepted)
    max_terminal = max(max(0.0, row["slosh_terminal_E"]) for row in accepted)
    max_k = max(max(0.0, row["kappa_p95"]) for row in accepted)
    max_dk = max(max(0.0, row["dkappa_p95"]) for row in accepted)
    max_drift = max(max(0.0, row["max_drift_m"]) for row in accepted)
    for row, _ in rows:
        if not row["accepted"]:
            continue
        _norm = lambda value, ref: value / max(1e-6, ref)
        length_penalty = max(0.0, row["length_ratio"] - 1.0)
        slosh_score = (
            w_h * _norm(row["slosh_h_p95"], max_h)
            + w_energy * _norm(row["slosh_energy_rms"], max_energy)
            + w_eta_dot * _norm(row["slosh_eta_dot_rms"], max_eta_dot)
            + w_terminal * _norm(row["slosh_terminal_E"], max_terminal)
            + w_kappa * _norm(row["kappa_p95"], max_k)
            + w_dkappa * _norm(row["dkappa_p95"], max_dk)
            + w_ay * max(0.0, row["predicted_ay_ratio"])
            + w_length * length_penalty
            + w_drift * _norm(row["max_drift_m"], max_drift)
        )
        row["slosh_score"] = slosh_score
        row["score"] = slosh_score
