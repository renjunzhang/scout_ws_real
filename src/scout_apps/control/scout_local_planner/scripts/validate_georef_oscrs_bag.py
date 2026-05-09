#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validate GeoRef/OSCRS real-bag behavior.

This script checks whether a recorded bag actually exercised the intended
reference-generation branch. It is intentionally about behavior wiring, not
about proving anti-slosh effectiveness.

Examples:
  # Formal OSCRS bag: active must be 1 and diagnostics must exist.
  python3 validate_georef_oscrs_bag.py bag.bag --mode oscrs

  # Takeover smoke after relaxing ay_ratio_limit: must publish non-original.
  python3 validate_georef_oscrs_bag.py bag.bag --mode oscrs --require-non-original

  # Fixed baseline smoke: must select the configured non-original candidate.
  python3 validate_georef_oscrs_bag.py bag.bag --mode fixed --require-non-original
"""

import argparse
import math
import os
import re
import statistics
import sys

import rosbag


SUMMARY_RE = re.compile(r"summary:([^;]+)")
ROW_RE = re.compile(r"(original|mild|medium|mid|strong):([^;]+)")


# 参数实际位置表：让 diagnose() 给出"哪个文件哪个键"而不是只说"调 X"。
# default 值与代码当前默认保持一致；改 launch/yaml 时请同步本表。
PKG_REL_ROOT = "src/scout_apps/control/scout_local_planner"
PARAM_LOCATIONS = {
    # post_processor launch args
    "ay_ratio_limit":           ("launch/anti_slosh_path_post_processor.launch", '<arg name="ay_ratio_limit">',           "1.0",     "候选 ay 与 baseline ay 比上限；硬门"),
    "max_candidate_level":      ("launch/anti_slosh_path_post_processor.launch", '<arg name="max_candidate_level">',      "medium",  "候选强度封顶 (original|mild|mid|medium|strong)"),
    "fixed_candidate_name":     ("launch/anti_slosh_path_post_processor.launch", '<arg name="fixed_candidate_name">',     '""',      "fixed mode 强制选定的候选名"),
    "min_segment_length":       ("launch/anti_slosh_path_post_processor.launch", '<arg name="min_segment_length">',       "0.02",    "sanitize 段长阈值"),
    "enable_collision_check":   ("launch/anti_slosh_path_post_processor.launch", '<arg name="enable_collision_check">',   "false",   "是否对候选做 costmap 碰撞检查"),
    "costmap_topic":            ("launch/anti_slosh_path_post_processor.launch", '<arg name="costmap_topic">',            "/scout/mbf_costmap_nav/global_costmap/costmap", "全局 costmap topic"),
    "collision_threshold":      ("launch/anti_slosh_path_post_processor.launch", '<arg name="collision_threshold">',      "50",      "costmap 碰撞代价上限 0-100"),
    "prediction_v_max":         ("launch/anti_slosh_path_post_processor.launch", '<arg name="prediction_v_max">',         "2.0",     "OSCRS rollout 速度上限；必须与 MPC 真实速度上限一致"),
    "prediction_ay_max_budget": ("launch/anti_slosh_path_post_processor.launch", '<arg name="prediction_ay_max_budget">', "2.0",     "OSCRS rollout 横向加速度预算"),
    "mild_iters":               ("launch/anti_slosh_path_post_processor.launch", '<arg name="mild_iters">',               "18",      "mild 平滑迭代次数"),
    "mild_gain":                ("launch/anti_slosh_path_post_processor.launch", '<arg name="mild_gain">',                "0.35",    "mild 平滑增益"),
    "mild_max_drift":           ("launch/anti_slosh_path_post_processor.launch", '<arg name="mild_max_drift">',           "0.08",    "mild 平滑最大偏移"),
    "oscrs_active_enable":      ("launch/anti_slosh_path_post_processor.launch", '<arg name="oscrs_active_enable">',      "false",   "OSCRS 是否实际接管"),
    "oscrs_shadow_enable":      ("launch/anti_slosh_path_post_processor.launch", '<arg name="oscrs_shadow_enable">',      "false",   "OSCRS 是否运行 shadow rollout"),
    # oscrs yaml keys
    "oscrs.eta_lim_mm":         ("config/oscrs_container.yaml", "slosh_score.eta_lim_mm",         "25.0",              "η peak 上限 (mm)；hard gate sH < eta_lim_mm/1000"),
    "oscrs.residual_ratio":     ("config/oscrs_container.yaml", "slosh_score.residual_ratio",     "0.2",               "残振 gate = residual_ratio × eta_lim_mm；Ferrari 论文 0.2"),
    "oscrs.height_coeff_mode":  ("config/oscrs_container.yaml", "slosh_score.height_coeff_mode",  "observer_linear",   "晃动高度系数估计模式"),
}


def _hint_block(symptom, *param_actions):
    """生成多行建议：症状 + 每个参数的 (位置, default, 含义)。

    param_actions: list[(key, action_text)]，key 在 PARAM_LOCATIONS 中。
    """
    lines = [symptom]
    for key, action in param_actions:
        entry = PARAM_LOCATIONS.get(key)
        if entry is None:
            lines.append(f"  · {key}: {action}")
            continue
        rel_file, locator, default, comment = entry
        lines.append(f"  · {key}: {action}")
        lines.append(f"    位置 {PKG_REL_ROOT}/{rel_file} {locator} (default={default}) — {comment}")
    return "\n".join(lines)


def _latest_sH_summary(data):
    """从最后一帧 candidate_rows 中提取每条候选的 sH (m) 与 accepted。"""
    if not data.get("candidate_rows"):
        return ""
    last = data["candidate_rows"][-1]
    parts = []
    for name in ("original", "mild", "mid", "medium", "strong"):
        if name not in last:
            continue
        sH = last[name].get("sH")
        if sH is None:
            continue
        try:
            sH_mm = float(sH) * 1000.0
            parts.append(f"{name}={sH_mm:.1f}mm")
        except ValueError:
            continue
    return ", ".join(parts)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument(
        "--mode",
        choices=("auto", "raw", "georef", "fixed", "oscrs"),
        default="auto",
        help="Expected behavior mode. auto infers from bag filename.",
    )
    parser.add_argument(
        "--require-non-original",
        action="store_true",
        help="Fail unless at least one report selected a non-original candidate.",
    )
    parser.add_argument(
        "--require-takeover",
        action="store_true",
        help="Fail unless at least one OSCRS report has takeover=1.",
    )
    parser.add_argument(
        "--allow-safety-alarm",
        action="store_true",
        help="Do not warn/fail on /anti_slosh_path/safety_alarm.",
    )
    return parser.parse_args()


def infer_mode(path):
    name = os.path.basename(path)
    if "GEOREF_OSCRS_ACTIVE" in name:
        return "oscrs"
    if "GEOREF_FIXED_STRONG" in name:
        return "fixed"
    if "GEOREF_TUNED" in name:
        return "georef"
    if "RAW" in name:
        return "raw"
    return "raw"


def parse_kv(text):
    out = {}
    for item in text.split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def parse_report(text):
    summary = {}
    match = SUMMARY_RE.search(text)
    if match:
        summary = parse_kv(match.group(1))
    rows = {}
    for name, body in ROW_RE.findall(text):
        rows[name] = parse_kv(body)
    return summary, rows


def percentile(values, p):
    if not values:
        return float("nan")
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    rank = (len(xs) - 1) * p / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return xs[lo]
    return xs[lo] * (hi - rank) + xs[hi] * (rank - lo)


def collect(path):
    data = {
        "reports": [],
        "candidate_rows": [],
        "global_path_anti_slosh": 0,
        "safety_alarm": 0,
        "height": [],
        "eta_dot": [],
        "mpc_status": [],
    }
    topics = [
        "/anti_slosh_path/candidate_report",
        "/anti_slosh_path/safety_alarm",
        "/scout/global_path_anti_slosh",
        "/slosh/height",
        "/slosh/eta_dot_norm",
        "/mpc_status",
    ]
    with rosbag.Bag(path) as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            if topic == "/anti_slosh_path/candidate_report":
                summary, rows = parse_report(str(msg.data))
                if summary:
                    data["reports"].append(summary)
                if rows:
                    data["candidate_rows"].append(rows)
            elif topic == "/anti_slosh_path/safety_alarm":
                data["safety_alarm"] += 1
            elif topic == "/scout/global_path_anti_slosh":
                data["global_path_anti_slosh"] += 1
            elif topic == "/slosh/height":
                data["height"].append(abs(float(msg.data)))
            elif topic == "/slosh/eta_dot_norm":
                data["eta_dot"].append(abs(float(msg.data)))
            elif topic == "/mpc_status":
                data["mpc_status"].append(str(msg.data))
    return data


def count_where(reports, key, value):
    return sum(1 for row in reports if row.get(key) == value)


def validate(mode, data, args):
    failures = []
    warnings = []
    reports = data["reports"]
    report_count = len(reports)
    active_count = count_where(reports, "active", "1")
    takeover_count = count_where(reports, "takeover", "1")
    fallback_count = count_where(reports, "fallback", "1")
    non_original_count = sum(1 for row in reports if row.get("selected") not in ("", None, "original"))
    fb_missing = sum(1 for row in reports if "fb" not in row)

    if mode == "raw":
        if report_count:
            warnings.append("RAW bag contains candidate_report; check launch isolation.")
    else:
        if data["global_path_anti_slosh"] <= 0:
            failures.append("missing /scout/global_path_anti_slosh")
        if report_count <= 0:
            failures.append("missing /anti_slosh_path/candidate_report")

    if mode in ("georef", "fixed"):
        if active_count:
            failures.append(f"{mode} should not have OSCRS active reports, got {active_count}")
        if fb_missing:
            failures.append(f"candidate_report missing fb in {fb_missing} reports")

    if mode == "fixed":
        selected = {row.get("selected", "missing") for row in reports}
        if not any(item not in ("", None, "original", "missing") for item in selected) and args.require_non_original:
            failures.append("fixed candidate smoke did not select a non-original candidate")

    if mode == "oscrs":
        if report_count and active_count != report_count:
            failures.append(f"OSCRS active reports mismatch: active={active_count}, reports={report_count}")
        if fb_missing:
            failures.append(f"candidate_report missing fb in {fb_missing} reports")
        # OSCRS row diagnostics are required for debugging hard-gate decisions.
        if data["candidate_rows"]:
            required = {"os", "oh", "or", "ov", "osc"}
            missing_rows = 0
            for rows in data["candidate_rows"]:
                for row in rows.values():
                    if not required.issubset(row):
                        missing_rows += 1
            if missing_rows:
                failures.append(f"candidate rows missing OSCRS fields in {missing_rows} rows")
        elif report_count:
            failures.append("candidate_report has summary but no candidate rows")

    if args.require_non_original and non_original_count <= 0:
        failures.append("no report selected a non-original candidate")
    if args.require_takeover and takeover_count <= 0:
        failures.append("no OSCRS takeover report found")

    if data["safety_alarm"] and not args.allow_safety_alarm:
        warnings.append(f"safety_alarm messages={data['safety_alarm']}")
        if args.require_non_original or args.require_takeover:
            failures.append("safety_alarm present during takeover-required smoke")

    return failures, warnings


def candidate_reject_reasons(data):
    counts = {}
    for rows in data["candidate_rows"]:
        for name, row in rows.items():
            if name == "original":
                continue
            reason = row.get("reason", "")
            if not reason:
                continue
            for part in reason.split("|"):
                key = part.split(":", 1)[0]
                counts[key] = counts.get(key, 0) + 1
    return counts


def diagnose(mode, data, failures, warnings, args):
    reports = data["reports"]
    reject_counts = candidate_reject_reasons(data)
    selected = [row.get("selected", "") for row in reports]
    fb_values = [row.get("fb", "") for row in reports]
    active_count = count_where(reports, "active", "1")
    takeover_count = count_where(reports, "takeover", "1")
    non_original_count = sum(1 for item in selected if item and item != "original")

    reasons = []
    suggestions = []

    if failures:
        reasons.extend(failures)
    if warnings:
        reasons.extend(warnings)

    if mode != "raw" and not reports:
        suggestions.append("检查 post-processor 是否启动、record 脚本是否包含 /anti_slosh_path/candidate_report。")
    if mode != "raw" and data["global_path_anti_slosh"] <= 0:
        suggestions.append("检查 post-processor output_topic、MPC global_path_topic，以及 /scout/global_path 是否有输入。")

    # mild 是否实际通过 gate（accepted=1）。若通过但 selector 仍选 original,
    # 说明问题不在 gate 而在 selector 评分（SAFE mild 下 mild gscore 比 original 大）。
    mild_accepted = any(
        rows.get("mild", {}).get("accepted") == "1"
        for rows in data["candidate_rows"]
    )

    if reject_counts.get("ay", 0) > 0 and non_original_count <= 0 and not mild_accepted:
        suggestions.append(_hint_block(
            "非 original 候选被 ay gate 全部拦住：",
            ("ay_ratio_limit", "临时 1.0 → 3.0 做 takeover smoke；smoke 通过后回收 2.0 → 1.5；若 odom_ay_p95 偏高再继续收紧。"),
        ))
    elif reject_counts.get("ay", 0) > 0 and failures:
        # 部分候选被 ay 拦只在已有 failure 时才提示，避免在 PASS 包里产生噪声建议。
        suggestions.append(_hint_block(
            "部分候选被 ay gate 拦住：",
            ("ay_ratio_limit", "若 takeover 不足可小幅放宽；若 odom_ay_p95 偏高则回收。"),
        ))

    if any(key in reject_counts for key in ("collision", "no_costmap", "frame_mismatch")) and failures:
        suggestions.append(_hint_block(
            "碰撞 / 地图 gate 异常 — 不要先关 collision check，按下面顺序排查：",
            ("costmap_topic", "确认 topic 与 MBF 实际发布一致；frame_id 应与 path frame_id 同。"),
            ("collision_threshold", "由 50 提到 90 → 100 试 1 包 smoke；仍 collision:idx=0 才考虑下一步。"),
            ("enable_collision_check", "只在开阔场地通路 smoke 时临时 false；正式包必须 true。"),
        ))

    if any(key in reject_counts for key in ("drift", "length", "short", "direction", "endpoint", "min_seg")) and failures:
        suggestions.append(_hint_block(
            "几何候选被基础 gate 拒绝：",
            ("mild_gain", "降低 gain 减少与 original 的偏离。"),
            ("mild_max_drift", "降低 max_drift 让候选不偏离 baseline。"),
            ("max_candidate_level", "降低封顶（如 medium → mild），过滤掉过强候选。"),
            ("min_segment_length", "min_seg 问题：检查 ds 与本参数比例。"),
        ))

    # geometry / OSCRS 模式：mild 通过 gate 但 selector 仍选 original = SAFE mild 下
    # geometry-only score 认为 mild 无收益。这是 condition 设计上可保留的诊断结果，
    # 不建议改 selector 语义"不许选 original"——那会把 TUNED 变成 FIXED 的变体，
    # 丢失 selector baseline 的意义。
    if (mode in ("georef", "oscrs")
            and args.require_non_original
            and non_original_count == 0
            and mild_accepted):
        suggestions.append(_hint_block(
            "selector 选了 original 而非 mild — mild 通过 gate 但 geometry-only score "
            "在 SAFE mild 下认为 mild 无收益。若必须让 selector 选 mild，只能强化 mild 的几何差异：",
            ("mild_iters", "提升迭代次数（同时会增加与 original 的偏离）。"),
            ("mild_gain", "提升 gain。"),
            ("mild_max_drift", "放宽 max_drift（同时监控 ay 不超）。"),
        ))
        suggestions.append(
            "推荐：把当前结果当作 'SAFE mild 下 geometry-only selector 倾向 original' 的诊断证据保留，"
            "不要为通过 --require-non-original 而调 mild 参数；先把 RAW vs FIXED_MILD 的主验证线跑出来。"
        )

    if mode == "fixed" and args.require_non_original and non_original_count <= 0:
        suggestions.append(_hint_block(
            "fixed candidate 没接管 — 先看候选拒绝原因：",
            ("fixed_candidate_name", "确认本 launch 设的候选名是否被 gate 直接 reject（看 last_candidate_reasons）。"),
            ("ay_ratio_limit", "若被 ay 拦：临时放宽。"),
            ("collision_threshold", "若被 collision 拦：阈值上调到 90/100。"),
        ))

    if mode == "oscrs":
        if active_count <= 0:
            suggestions.append(_hint_block(
                "OSCRS 没 active：",
                ("oscrs_active_enable", "确认 launch 命令传了 :=true（GEOREF_OSCRS_ACTIVE_REAL 段）。"),
                ("oscrs_shadow_enable", "active 通常需要 shadow 同时为 true。"),
            ))
        if fb_values and set(fb_values) == {"3"}:
            suggestions.append(_hint_block(
                "fb=3 表示无可用几何候选 — 先修几何 gate，不要先调 OSCRS score：",
                ("ay_ratio_limit", "若是 ay 全 reject：临时放宽到 3.0。"),
                ("collision_threshold", "若是 collision：阈值上调。"),
                ("max_candidate_level", "若是 level cap：放开到 medium 看候选集是否变充裕。"),
            ))
        elif fb_values and "2" in fb_values:
            sH_summary = _latest_sH_summary(data)
            extra = f"  本包候选 sH = [{sH_summary}]" if sH_summary else ""
            suggestions.append(_hint_block(
                "fb=2 表示几何可行但 slosh hard gate 失败 — 不要直接调 score。" + extra,
                ("oscrs.eta_lim_mm", "由 25 临时放到 40 (留 mild 5-10mm 余量)，先看能否产生 fb=0；正式包再回收。"),
                ("oscrs.residual_ratio", "默认 0.2 (Ferrari)；除非 sHr 主导否则不动。"),
                ("oscrs.height_coeff_mode", "若 observer_linear 与实测明显不符再换模式。"),
                ("max_candidate_level", "SAFE mild 候选不足时可临时 → medium，但需复核 medium 在你场地不会过冲。"),
            ))
            suggestions.append(
                "约束：不要降低 prediction_v_max 来救 OSCRS — rollout 与 MPC 真实速度上限口径必须一致，"
                "压低 rollout 速度会让 hard gate 自欺，实车不一定真的低晃。"
            )
        elif fb_values and "1" in fb_values:
            suggestions.append(_hint_block(
                "fb=1 表示只有 original slosh-safe — 候选集饱和或太弱：",
                ("max_candidate_level", "若 SAFE mild 下饱和，可临时放到 medium。"),
                ("oscrs.eta_lim_mm", "若希望让更多候选通过 hard gate 才能放宽 eta_lim。"),
                ("mild_gain", "提升 mild 强度，但会使 mild 偏离 original 更多。"),
            ))
        if args.require_takeover and takeover_count <= 0:
            suggestions.append(
                "需要 takeover 但 takeover=0：OSCRS 与 geometry selector 选了同一路径或回退；"
                "先确认 --require-non-original 通过，再看 OSCRS score 权重 "
                f"({PKG_REL_ROOT}/config/oscrs_container.yaml slosh_score.score.*)."
            )

    if data["safety_alarm"] > 0:
        suggestions.append("有 safety_alarm：该包不要进正式有效性统计；先看 fb/reject reason，按上面定位修 gate 后重录。")

    if not suggestions and not failures:
        if mode == "raw":
            suggestions.append("RAW 行为正常：可作为 baseline；继续录对应 GeoRef/OSCRS。")
        elif args.require_non_original or args.require_takeover:
            suggestions.append("smoke 行为正常：可以进入正式录包，但仍需用 extract_slosh_metrics/RGB 判断是否真的降晃。")
        else:
            suggestions.append("行为链路正常：若要验证接管，请对 smoke 加 --require-non-original 或 --require-takeover。")

    normal = not failures
    return normal, reasons, suggestions


def print_summary(path, mode, data, failures, warnings):
    reports = data["reports"]
    selected_counts = {}
    fb_counts = {}
    for row in reports:
        selected_counts[row.get("selected", "missing")] = selected_counts.get(row.get("selected", "missing"), 0) + 1
        fb_counts[row.get("fb", "missing")] = fb_counts.get(row.get("fb", "missing"), 0) + 1
    active_count = count_where(reports, "active", "1")
    takeover_count = count_where(reports, "takeover", "1")
    fallback_count = count_where(reports, "fallback", "1")

    h = data["height"]
    eta_dot = data["eta_dot"]
    print(f"bag={path}")
    print(f"mode={mode}")
    print(
        "reports={} active={} takeover={} fallback={} global_path_anti_slosh={} safety_alarm={}".format(
            len(reports), active_count, takeover_count, fallback_count,
            data["global_path_anti_slosh"], data["safety_alarm"],
        )
    )
    print(f"selected_counts={selected_counts}")
    print(f"fb_counts={fb_counts}")
    if reports:
        last = reports[-1]
        print(
            "last="
            + ",".join(
                f"{key}={last.get(key, '')}"
                for key in ("selected", "geo", "oscrs", "active", "fallback", "fb", "takeover")
            )
        )
    reject_counts = candidate_reject_reasons(data)
    if reject_counts:
        print(f"reject_reason_counts={reject_counts}")
    if data["candidate_rows"]:
        last_rows = data["candidate_rows"][-1]
        compact = []
        for name in ("original", "mild", "medium", "mid", "strong"):
            if name not in last_rows:
                continue
            row = last_rows[name]
            compact.append(
                "{}:accepted={},reason={}".format(
                    name, row.get("accepted", "?"), row.get("reason", "?")
                )
            )
        if compact:
            print("last_candidate_reasons=" + " | ".join(compact))
    if h:
        rms = math.sqrt(statistics.mean(v * v for v in h))
        print(f"height_rms={rms:.6g} height_p95={percentile(h,95):.6g} height_max={max(h):.6g}")
    if eta_dot:
        rms = math.sqrt(statistics.mean(v * v for v in eta_dot))
        print(f"eta_dot_rms={rms:.6g} eta_dot_p95={percentile(eta_dot,95):.6g}")
    for item in warnings:
        print(f"WARN: {item}")
    for item in failures:
        print(f"FAIL: {item}")
    print("VERDICT=" + ("FAIL" if failures else "PASS"))


def print_human_verdict(path, mode, data, failures, warnings, args):
    normal, reasons, suggestions = diagnose(mode, data, failures, warnings, args)
    print("==== 行为验收结论 ====")
    print("正常性: " + ("正常" if normal else "不正常"))
    print(f"模式: {mode}")
    print(f"bag: {path}")
    print("")
    print("原因:")
    if reasons:
        for item in reasons:
            print(f"- {item}")
    else:
        print("- 未发现阻塞性行为问题。")
    print("")
    print("建议:")
    for item in suggestions:
        print(f"- {item}")
    print("")


def main():
    args = parse_args()
    mode = infer_mode(args.bag) if args.mode == "auto" else args.mode
    data = collect(args.bag)
    failures, warnings = validate(mode, data, args)
    print_human_verdict(args.bag, mode, data, failures, warnings, args)
    print_summary(args.bag, mode, data, failures, warnings)
    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
