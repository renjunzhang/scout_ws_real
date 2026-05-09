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

  # Strong fixed baseline smoke: must select strong or another non-original.
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
        if "strong" not in selected and args.require_non_original:
            failures.append("fixed strong smoke did not select strong/non-original")

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

    if reject_counts.get("ay", 0) > 0 and non_original_count <= 0:
        suggestions.append("非 original 被 ay gate 拦住：先把 ay_ratio_limit 临时放宽到 3.0 做 takeover smoke；通过后再试 2.0 -> 1.5。")
    elif reject_counts.get("ay", 0) > 0:
        suggestions.append("部分候选被 ay gate 拦住：若 takeover 不足，可小幅放宽 ay_ratio_limit；若 odom_ay_p95 偏高则回收阈值。")

    if any(key in reject_counts for key in ("collision", "no_costmap", "frame_mismatch")):
        suggestions.append("碰撞/地图 gate 异常：优先检查 costmap_topic、frame_id、collision_threshold，不要先关闭 collision check。")

    if any(key in reject_counts for key in ("drift", "length", "short", "direction", "endpoint", "min_seg")):
        suggestions.append("几何候选被基础 gate 拒绝：调小 smoothing gain/drift，或降低 max_candidate_level；min_seg 问题可检查 ds/min_segment_length。")

    if mode == "fixed" and args.require_non_original and "strong" not in selected:
        suggestions.append("fixed strong 没接管：优先放宽 ay_ratio_limit；若 strong 仍被 geometry gate 拒绝，降低 strong_gain/strong_max_drift。")

    if mode == "oscrs":
        if active_count <= 0:
            suggestions.append("OSCRS 没 active：检查 oscrs_active_enable:=true，确认使用 GEOREF_OSCRS_ACTIVE_REAL 启动段。")
        if fb_values and set(fb_values) == {"3"}:
            suggestions.append("fb=3 表示无可用几何候选：先修 ay_ratio_limit/costmap/geometry gate，OSCRS score 暂时不用调。")
        elif fb_values and "2" in fb_values:
            suggestions.append("fb=2 表示几何可行但 slosh hard gate 失败：先检查 oscrs.eta_lim_mm、residual_ratio、height_coeff_mode，不要直接调 score。")
        elif fb_values and "1" in fb_values:
            suggestions.append("fb=1 表示只有 original slosh-safe：候选集可能饱和或太弱，先看候选 slosh 指标，再考虑加强 GeoRef 或调 eta_lim。")
        if args.require_takeover and takeover_count <= 0:
            suggestions.append("需要 takeover 但 takeover=0：OSCRS 与 geometry selector 选同一路径或回退；先确认 --require-non-original 通过，再看 score 权重。")

    if data["safety_alarm"] > 0:
        suggestions.append("有 safety_alarm：该包不要进正式有效性统计；先看 fb/reject reason，修 gate 后重录。")

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
