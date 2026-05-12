"""diagnostics — candidate_report / safety_alarm / metrics 格式化。

输出格式与旧 bag 解析脚本 (validate_georef_oscrs_bag.py) 严格兼容。
本模块不持有 ROS publisher，只做纯字符串/数值格式化。
"""


def compute_fb_codes(rows, oscrs_shadow_enable, oscrs_active_enable, best_row, geometry_row, oscrs_row):
    """计算 fb / fallback / takeover 语义码。

    Returns:
        (oscrs_fallback_code, oscrs_fallback, oscrs_takeover)
        fb: -1 未 active, 0 takeover 成功, 1 only original safe,
            2 几何可行但 slosh gate 全失败, 3 无可用几何候选
    """
    original_row = next((row for row, _ in rows if row["name"] == "original"), None)
    original_safe = bool(original_row is not None and original_row["oscrs_feasible"])
    accepted_non_original = any(row["accepted"] and row["name"] != "original" for row, _ in rows)

    if not (oscrs_shadow_enable or oscrs_active_enable):
        fb_code = -1
    elif oscrs_row is not None and oscrs_row["oscrs_feasible"]:
        fb_code = 0
    elif original_safe:
        fb_code = 1
    elif accepted_non_original:
        fb_code = 2
    else:
        fb_code = 3

    fallback = int(oscrs_active_enable and fb_code != 0)
    takeover = int(
        oscrs_active_enable
        and oscrs_row is not None
        and oscrs_row["oscrs_feasible"]
        and best_row["name"] == oscrs_row["name"]
        and best_row["name"] != geometry_row["name"]
    )
    return fb_code, fallback, takeover, original_safe


def format_summary(best_row, geometry_row, oscrs_row, oscrs_active_enable,
                   fb_code, fallback, original_safe, takeover):
    """格式化 summary 行。格式兼容 validate_georef_oscrs_bag.py 的 SUMMARY_RE。"""
    oscrs_name = oscrs_row["name"] if oscrs_row else "none"
    return (
        "summary:selected={selected},geo={geo},oscrs={oscrs},active={active},"
        "fallback={fallback},fb={fb},orig_safe={orig_safe},takeover={takeover}"
    ).format(
        selected=best_row["name"],
        geo=geometry_row["name"],
        oscrs=oscrs_name,
        active=int(oscrs_active_enable),
        fallback=fallback,
        fb=fb_code,
        orig_safe=int(original_safe),
        takeover=takeover,
    )


def format_candidate_row(row):
    """格式化单条候选行。格式兼容 validate_georef_oscrs_bag.py 的 ROW_RE。"""
    if row["collision_status"] == "collision":
        col_str = "col=hit:idx={}:cost={}".format(row["collision_idx"], row["collision_cost"])
    elif row["collision_status"] == "no_costmap":
        col_str = "col=no_costmap"
    elif row["collision_status"] == "frame_mismatch":
        col_str = "col=frame_mismatch"
    else:
        col_str = "col=ok"
    return (
        "{name}:accepted={accepted},reason={reason},score={score:.3f},"
        "len={length:.3f},drift={drift:.3f},end={end:.3f},"
        "k95={k95:.3f},dk95={dk95:.3f},kr={kr:.3f},dkr={dkr:.3f},"
        "ayr={ayr:.3f},vmaxp={vmaxp:.3f},gscore={gscore:.3f},sscore={sscore:.3f},"
        "sH={sH:.3g},sHm={sHm:.3g},sHp={sHp:.3g},sHr={sHr:.3g},"
        "sE={sE:.3g},sEdot={sEdot:.3g},os={os},oh={oh},or={or_},ov={ov:.3g},osc={osc:.3g},{col}"
    ).format(
        name=row["name"],
        accepted=int(row["accepted"]),
        reason=row["reject_reason"],
        score=row["score"],
        length=row["length_ratio"],
        drift=row["max_drift_m"],
        end=row["endpoint_error_m"],
        k95=row["kappa_p95"],
        dk95=row["dkappa_p95"],
        kr=row["kappa_ratio"],
        dkr=row["dkappa_ratio"],
        ayr=row["predicted_ay_ratio"],
        vmaxp=row["predicted_vmax"],
        gscore=row["geometry_score"],
        sscore=row["slosh_score"],
        sE=row["slosh_energy_rms"],
        sEdot=row["slosh_eta_dot_rms"],
        sH=row["slosh_h_p95"],
        sHm=row["slosh_h_modal_p95"],
        sHp=row["slosh_h_parabola_p95"],
        sHr=row["slosh_h_residual_max"],
        os=int(row["oscrs_feasible"]),
        oh=int(row["oscrs_height_pass"]),
        or_=int(row["oscrs_residual_pass"]),
        ov=row["oscrs_violation"],
        osc=row["oscrs_score"],
        col=col_str,
    )


def format_report(rows, best_row, geometry_row, oscrs_row,
                  oscrs_shadow_enable, oscrs_active_enable):
    """格式化完整 candidate_report 字符串。

    Returns:
        str: "summary:...; original:...; mild:...; ..." 格式的完整报告。
    """
    fb_code, fallback, takeover, original_safe = compute_fb_codes(
        rows, oscrs_shadow_enable, oscrs_active_enable, best_row, geometry_row, oscrs_row,
    )
    parts = [
        format_summary(best_row, geometry_row, oscrs_row, oscrs_active_enable,
                       fb_code, fallback, original_safe, takeover),
    ]
    for row, _ in rows:
        parts.append(format_candidate_row(row))
    return "; ".join(parts)


def format_safety_alarm(rows, geometry_row, published_row, eta_lim):
    """格式化 safety_alarm 消息字符串。

    Returns:
        str: "hard_gate_failed=1,..." 格式的消息，或 None（若不需要 alarm）。
    """
    feasible_count = sum(1 for row, _ in rows if row["oscrs_feasible"])
    min_violation = min(
        (row["oscrs_violation"] for row, _ in rows if row["accepted"]),
        default=float("inf"),
    )
    return (
        "hard_gate_failed=1,feasible_count={fc},min_violation={mv:.4g},"
        "geometry_best={gb},published={pb},eta_lim_mm={el:.1f}"
    ).format(
        fc=feasible_count,
        mv=min_violation if min_violation != float("inf") else -1.0,
        gb=geometry_row["name"],
        pb=published_row["name"],
        el=eta_lim * 1000.0,
    )


def build_metrics_array(best_row, rows):
    """构建 Float32MultiArray 的 data 列表。"""
    return [
        float(best_row["index"]),
        float(best_row["score"]),
        float(best_row["length_ratio"]),
        float(best_row["max_drift_m"]),
        float(best_row["endpoint_error_m"]),
        float(best_row["kappa_p95"]),
        float(best_row["kappa_max"]),
        float(best_row["dkappa_p95"]),
        float(best_row["dkappa_max"]),
        float(len(rows)),
        float(sum(1 for row, _ in rows if row["accepted"])),
    ]
