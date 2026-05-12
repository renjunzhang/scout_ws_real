"""oscrs/pipeline.py — G→F→R→S 纯编排器。

将 path_callback() 中除 ROS I/O 外的全部决策逻辑提纯为无 ROS 依赖的函数。
接受 duck-typed params 对象（需提供预期属性），方便单元测试时注入假参数。
"""

from oscrs.feasibility import evaluate_geometry_feasibility
from oscrs.selector import (
    apply_oscrs_score,
    evaluate_oscrs_hard_gate,
    resolve_fixed_candidate,
    select_oscrs_candidate,
)
from oscrs.slosh_rollout import (
    apply_slosh_scores,
    predicted_lateral_profile,
    rollout_slosh_metrics,
)

_SLOSH_ZERO = {
    "slosh_h_p95": 0.0,
    "slosh_h_max": 0.0,
    "slosh_h_residual_max": 0.0,
    "slosh_h_modal_p95": 0.0,
    "slosh_h_parabola_p95": 0.0,
    "slosh_eta_dot_rms": 0.0,
    "slosh_energy_rms": 0.0,
    "slosh_terminal_E": 0.0,
}


def _evaluate_one(index, name, candidate, base, base_metrics, base_length,
                  collision_status, collision_idx, collision_cost, p):
    """对一条候选执行 F→R→S 纯计算链。p 为 duck-typed params 对象。"""
    base_ay_p95, base_vmax = predicted_lateral_profile(
        base, p.predict_v_max, p.predict_ay_max, p.predict_a_max, p.predict_v_init,
        p.slosh_offset_x, p.slosh_offset_y,
    )
    ay_p95, predicted_vmax = predicted_lateral_profile(
        candidate, p.predict_v_max, p.predict_ay_max, p.predict_a_max, p.predict_v_init,
        p.slosh_offset_x, p.slosh_offset_y,
    )
    f_result = evaluate_geometry_feasibility(
        candidate, name, base, base_metrics, base_length,
        p.min_segment_length, p.max_drift, p.max_length_ratio,
        p.min_length_ratio, p.min_kappa_ratio, p.target_kappa_ratio,
        p.max_endpoint_error, p.ay_ratio_limit,
        ay_p95, predicted_vmax, base_ay_p95, base_vmax,
        p.candidate_levels, p.max_candidate_level,
        collision_status, collision_idx, collision_cost,
        p.w_kappa, p.w_dkappa, p.w_length, p.w_drift,
        p.w_shortening, p.w_over_smooth,
    )
    accepted = f_result["accepted"]
    score = f_result["geometry_score"]
    slosh_rollout_enabled = p.slosh_score_enable or p.oscrs_shadow_enable or p.oscrs_active_enable
    if slosh_rollout_enabled:
        slosh_metrics = rollout_slosh_metrics(
            candidate,
            predict_v_max=p.predict_v_max,
            predict_ay_max=p.predict_ay_max,
            predict_a_max=p.predict_a_max,
            predict_v_init=p.predict_v_init,
            offset_x=p.slosh_offset_x,
            offset_y=p.slosh_offset_y,
            omega_n=p.slosh_omega_n,
            zeta=p.slosh_zeta,
            rollout_dt=p.slosh_rollout_dt,
            v_floor=p.slosh_v_floor,
            height_coeff=p.slosh_height_coeff,
            container_radius=p.slosh_container_radius,
            use_parabola=p.slosh_use_parabola,
            settle_duration=p.oscrs_settle_duration,
            enable_shadow_or_active=(p.oscrs_shadow_enable or p.oscrs_active_enable),
        )
    else:
        slosh_metrics = _SLOSH_ZERO
    oscrs_gate = evaluate_oscrs_hard_gate(
        accepted, slosh_metrics, p.oscrs_eta_lim, p.oscrs_residual_ratio, score,
    )
    return {
        "index": index,
        "name": name,
        "score": score,
        "geometry_score": score,
        "slosh_score": score,
        "base_predicted_ay_p95": base_ay_p95,
        "base_predicted_vmax": base_vmax,
        "oscrs_eta_lim": p.oscrs_eta_lim,
        **oscrs_gate,
        **slosh_metrics,
        **f_result,
    }


class PipelineResult:
    __slots__ = ("rows", "best", "geometry_best", "oscrs_best", "alarm_triggered",
                 "best_fallback", "fixed_not_found", "fixed_rejected", "fixed_reject_reason")

    def __init__(self, rows, best, geometry_best, oscrs_best, alarm_triggered,
                 best_fallback=False, fixed_not_found=False, fixed_rejected=False,
                 fixed_reject_reason=""):
        self.rows = rows
        self.best = best
        self.geometry_best = geometry_best
        self.oscrs_best = oscrs_best
        self.alarm_triggered = alarm_triggered
        self.best_fallback = best_fallback
        self.fixed_not_found = fixed_not_found
        self.fixed_rejected = fixed_rejected
        self.fixed_reject_reason = fixed_reject_reason


def run_pipeline(base, candidates, base_metrics, base_length, collision_results, p):
    """执行 G→F→R→S 全链路编排。

    Args:
        base: list[(x,y)] 已 resample 的基础路径。
        candidates: list[(name, points)] G 层输出的候选集。
        base_metrics: path_metrics(base, base) 结果。
        base_length: float，基础路径长度。
        collision_results: dict[name] -> (status, idx, cost)，由 adapter 预计算。
        p: duck-typed params 对象，需提供所有 gate/score/oscrs 参数属性。

    Returns:
        PipelineResult
    """
    rows = []
    for index, (name, candidate) in enumerate(candidates):
        col_status, col_idx, col_cost = collision_results.get(
            name, ("accepted", -1, -1),
        )
        row = _evaluate_one(
            index, name, candidate, base, base_metrics, base_length,
            col_status, col_idx, col_cost, p,
        )
        rows.append((row, candidate))

    if p.slosh_score_enable:
        apply_slosh_scores(
            rows, p.w_slosh_h, p.w_slosh_energy, p.w_slosh_eta_dot,
            p.w_slosh_terminal, p.w_slosh_kappa, p.w_slosh_dkappa,
            p.w_slosh_ay, p.w_slosh_length, p.w_slosh_drift,
        )

    best = None
    for row, candidate in rows:
        if row["accepted"] and (best is None or row["score"] < best[0]["score"]):
            best = (row, candidate)
    best_fallback = best is None
    if best_fallback:
        best = rows[0]

    geometry_best = best
    oscrs_best = None
    fixed_not_found = False
    fixed_rejected = False
    fixed_reject_reason = ""

    if p.fixed_candidate_name:
        fixed, status = resolve_fixed_candidate(rows, p.fixed_candidate_name)
        if status == "not_found":
            fixed_not_found = True
            best = rows[0]
        elif status == "found_rejected":
            fixed_rejected = True
            fixed_reject_reason = fixed[0]["reject_reason"]
            best = rows[0]
        else:
            best = fixed
    else:
        if (p.oscrs_shadow_enable or p.oscrs_active_enable) and not p.oscrs_use_legacy_score:
            apply_oscrs_score(
                rows, p.oscrs_score_w_h_p95, p.oscrs_score_w_energy,
                p.oscrs_score_w_eta_dot, p.oscrs_score_w_terminal,
                p.oscrs_score_w_geom, p.oscrs_score_batch_norm,
            )
        oscrs_best = select_oscrs_candidate(rows)
        if p.oscrs_active_enable and oscrs_best is not None:
            best = oscrs_best

    alarm = bool(
        (p.oscrs_shadow_enable or p.oscrs_active_enable)
        and not best[0]["oscrs_feasible"]
    )

    return PipelineResult(
        rows, best, geometry_best, oscrs_best, alarm,
        best_fallback, fixed_not_found, fixed_rejected, fixed_reject_reason,
    )
