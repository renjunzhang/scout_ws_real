"""S 层 — OSCRS hard gate + score + selection + fallback。

从 anti_slosh_path_post_processor.py 的 evaluate_candidate (OSCRS gate),
select_oscrs_candidate, apply_oscrs_score, path_callback (fixed candidate / fallback)
迁移而来。纯 Python，无 ROS 依赖。
"""


def evaluate_oscrs_hard_gate(accepted, slosh_metrics, oscrs_eta_lim,
                               oscrs_residual_ratio, geometry_score):
    """对一条候选计算 OSCRS hard gate 和 legacy score。

    Returns:
        dict: oscrs_feasible, oscrs_height_pass, oscrs_residual_pass,
              oscrs_violation, oscrs_score (legacy), oscrs_residual_limit.
    """
    oscrs_height_pass = slosh_metrics["slosh_h_max"] <= oscrs_eta_lim
    oscrs_residual_limit = oscrs_residual_ratio * oscrs_eta_lim
    oscrs_residual_pass = slosh_metrics["slosh_h_residual_max"] <= oscrs_residual_limit
    oscrs_feasible = accepted and oscrs_height_pass and oscrs_residual_pass
    oscrs_violation = (
        max(0.0, slosh_metrics["slosh_h_max"] / max(oscrs_eta_lim, 1e-9) - 1.0)
        + max(0.0, slosh_metrics["slosh_h_residual_max"] / max(oscrs_residual_limit, 1e-9) - 1.0)
    )
    oscrs_score = (
        2.0 * slosh_metrics["slosh_h_max"] / max(oscrs_eta_lim, 1e-9)
        + slosh_metrics["slosh_h_residual_max"] / max(oscrs_residual_limit, 1e-9)
        + 0.2 * geometry_score
    )
    return {
        "oscrs_feasible": oscrs_feasible,
        "oscrs_height_pass": oscrs_height_pass,
        "oscrs_residual_pass": oscrs_residual_pass,
        "oscrs_violation": oscrs_violation,
        "oscrs_score": oscrs_score,
        "oscrs_residual_limit": oscrs_residual_limit,
    }


def apply_oscrs_score(rows, score_w_h_p95, score_w_energy_rms, score_w_eta_dot_rms,
                      score_w_terminal_E, score_w_geom, score_batch_norm):
    """RA-L §4.1 Layer 2: OSCRS weighted score。

    只修改 S_full（非 original 的 feasible 候选）中的 row["oscrs_score"]。
    score_batch_norm=true 时使用 S_full batch normalization；false 时仍使用
    同一组权重，但直接对原始物理量加权，不能回退到 legacy score。
    """
    feasible = [
        row for row, _ in rows
        if row["oscrs_feasible"] and row["name"] != "original"
    ]
    if not feasible:
        return
    if score_batch_norm:
        max_h_p95 = max(max(0.0, row["slosh_h_p95"]) for row in feasible)
        max_energy = max(max(0.0, row["slosh_energy_rms"]) for row in feasible)
        max_eta_dot = max(max(0.0, row["slosh_eta_dot_rms"]) for row in feasible)
        max_terminal = max(max(0.0, row["slosh_terminal_E"]) for row in feasible)
        max_geom = max(max(0.0, row["geometry_score"]) for row in feasible)
    else:
        max_h_p95 = max_energy = max_eta_dot = max_terminal = max_geom = 1.0
    for row in feasible:
        norm = lambda value, ref: value / max(1e-9, ref) if score_batch_norm else value
        row["oscrs_score"] = (
            score_w_h_p95 * norm(row["slosh_h_p95"], max_h_p95)
            + score_w_energy_rms * norm(row["slosh_energy_rms"], max_energy)
            + score_w_eta_dot_rms * norm(row["slosh_eta_dot_rms"], max_eta_dot)
            + score_w_terminal_E * norm(row["slosh_terminal_E"], max_terminal)
            + score_w_geom * norm(row["geometry_score"], max_geom)
        )


def select_oscrs_candidate(rows):
    """S_full 选择：从非 original 的 feasible 候选中取 oscrs_score 最低者。

    Returns:
        (row, points) or None.
    """
    feasible = [
        (row, points)
        for row, points in rows
        if row["oscrs_feasible"] and row["name"] != "original"
    ]
    if feasible:
        return min(feasible, key=lambda item: item[0]["oscrs_score"])
    return None


def resolve_fixed_candidate(rows, fixed_candidate_name):
    """解析 fixed candidate 强制选择。

    Returns:
        (item, status) where status ∈ {"found_accepted", "found_rejected", "not_found"}
        and item is (row, points) or None.
    """
    if not fixed_candidate_name:
        return None, "not_found"
    fixed = next((item for item in rows if item[0]["name"] == fixed_candidate_name), None)
    if fixed is None:
        return None, "not_found"
    if fixed[0]["accepted"]:
        return fixed, "found_accepted"
    return fixed, "found_rejected"
