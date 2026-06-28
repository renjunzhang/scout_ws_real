#!/usr/bin/env python3
"""Generate the fixed-path S-curve N=9 comparison workbook.

The workbook intentionally has only two visible data sheets:
- 每一次数据: normalized per-case rows
- 每组平均: group aggregates, including P10-P90 intervals computed from per-case rows
"""

from __future__ import annotations

import csv
import math
import shutil
import statistics
from collections import OrderedDict
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import xlsxwriter

TABLE_DIR = Path(
    "/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/"
    "20260626_fixed_path_s_curve_matrix_n3/tables"
)
OUT_PATH = Path(
    "/home/a/scout_ws/docs/实物实验注意事项/对比试验/仿真对比试验/"
    "20260627_外部对比算法_N9_固定路径S曲线对比表.xlsx"
)

SOURCE_FILES = {
    "internal_per_case": TABLE_DIR / "spmpc_internal_ablation_n9_per_case.csv",
    "hardcap_per_case": TABLE_DIR / "hardcap_1mm_0p85mm_n9_per_case.csv",
    "external_per_case": TABLE_DIR / "external_baselines_n9_per_case.csv",
}

PER_CASE_COLUMNS = [
    "实验组",
    "组别",
    "算法/方法",
    "显示名",
    "Run",
    "strict valid",
    "goal reached",
    "final status",
    "time to goal (s)",
    "tracking RMS (m)",
    "tracking max (m)",
    "final error (m)",
    "slosh peak (mm)",
    "slosh p95 (mm)",
    "slosh RMS (mm)",
    "driven |v| max (m/s)",
    "driven |omega| max (rad/s)",
    "linear accel p95 (m/s²)",
    "angular accel p95 (rad/s²)",
    "vs B0 max v reduction (%)",
    "vs B0 max omega reduction (%)",
    "vs B0 linear accel p95 reduction (%)",
    "vs B0 angular accel p95 reduction (%)",
    "vs B0 slosh p95 reduction (%)",
    "acados fail count",
    "failure status count",
    "freshness note",
    "reason",
    "bag path",
    "summary path",
]

AVG_COLUMNS = [
    "实验组",
    "组别",
    "算法/方法",
    "显示名",
    "N",
    "strict valid",
    "success",
    "fails",
    "time mean (s)",
    "time std (s)",
    "time P10-P90 (s)",
    "time mean±std (s)",
    "tracking RMS mean (m)",
    "tracking RMS std (m)",
    "tracking RMS P10-P90 (m)",
    "tracking RMS mean±std (m)",
    "tracking max mean (m)",
    "tracking max std (m)",
    "tracking max P10-P90 (m)",
    "final error mean (m)",
    "final error std (m)",
    "final error P10-P90 (m)",
    "slosh peak mean (mm)",
    "slosh peak std (mm)",
    "slosh peak P10-P90 (mm)",
    "slosh peak mean±std (mm)",
    "slosh p95 mean (mm)",
    "slosh p95 std (mm)",
    "slosh p95 P10-P90 (mm)",
    "slosh p95 mean±std (mm)",
    "driven |v| max mean (m/s)",
    "driven |v| max std (m/s)",
    "driven |v| max P10-P90 (m/s)",
    "driven |omega| max mean (rad/s)",
    "driven |omega| max std (rad/s)",
    "driven |omega| max P10-P90 (rad/s)",
    "linear accel p95 mean (m/s²)",
    "linear accel p95 std (m/s²)",
    "linear accel p95 P10-P90 (m/s²)",
    "angular accel p95 mean (rad/s²)",
    "angular accel p95 std (rad/s²)",
    "angular accel p95 P10-P90 (rad/s²)",
    "vs B0 max v reduction (%)",
    "vs B0 max omega reduction (%)",
    "vs B0 linear accel p95 reduction (%)",
    "vs B0 angular accel p95 reduction (%)",
    "vs B0 slosh p95 reduction (%)",
    "acados fail cases",
    "acados fail total",
    "failure status total",
    "说明",
]

NUMERIC_PER_CASE = {
    "time to goal (s)",
    "tracking RMS (m)",
    "tracking max (m)",
    "final error (m)",
    "slosh peak (mm)",
    "slosh p95 (mm)",
    "slosh RMS (mm)",
    "driven |v| max (m/s)",
    "driven |omega| max (rad/s)",
    "linear accel p95 (m/s²)",
    "angular accel p95 (rad/s²)",
    "vs B0 max v reduction (%)",
    "vs B0 max omega reduction (%)",
    "vs B0 linear accel p95 reduction (%)",
    "vs B0 angular accel p95 reduction (%)",
    "vs B0 slosh p95 reduction (%)",
    "acados fail count",
    "failure status count",
}
NUMERIC_AVG = {
    col
    for col in AVG_COLUMNS
    if any(
        token in col
        for token in [
            "mean",
            "std",
            "cases",
            "total",
            "reduction",
        ]
    )
}

GROUP_NOTES = {
    "SPMPC内部消融N9": "SPMPC internal ablation; slosh-cap only for hard variants in this block.",
    "SPMPC hard-cap阈值N9": "SPMPC B_ours_hard threshold sensitivity; slosh height hard-cap participates in control.",
    "外部baseline N9": "External baseline block; DWA/TEB/mpc_local_planner/LT-DWA do not use slosh feedback; slosh is evaluation-only.",
}

FAIRNESS_NOTES = [
    "参数公平设置 / 统计口径：",
    "1. 固定路径统一：GOAL_X=5.0, GOAL_Y=0.0, GOAL_YAW=0.0；PATH_TEMPLATE=s_curve；PATH_AMPLITUDE_RATIO=0.18；PATH_MIN_AMPLITUDE=0.25；PATH_MAX_AMPLITUDE=1.20；PATH_SIDE=left；PATH_SMOOTH_ITERATIONS=3。",
    "2. 地图统一：MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream；不得依赖 launch/script 默认地图。",
    "3. 正式数据统一为 strict fresh-sim：每个 case 单独启动/关闭 ROS/Gazebo；pre/post ROS/Gazebo reachability 必须为 false。",
    "4. FAIL 规则统一：RECORD_SEC=60；60s 内未 GOAL_REACHED 记 FAIL；单 case 外层 15min timeout；同组连续失败超过 3 次跳过剩余 run。",
    "5. 外部 baseline 公平约束：v_max=0.8 m/s, omega_max=1.2 rad/s, linear_accel_max=0.6 m/s², angular_accel_max=1.2 rad/s²。",
    "6. 几何约束：DWA/TEB/mpc_local_planner 使用 Scout Mini conservative footprint；LT-DWA 使用等效 robot_radius=0.426 m。",
    "7. SPMPC hard-cap 行的 slosh_height_max 区分为 1.0mm 或 0.85mm；外部 baseline 不使用 slosh feedback，/slosh/height 只作为外部评价指标。",
    "8. 每组平均 sheet 中 P10-P90 列为同组 N=9 per-case 指标的 10% 与 90% 分位区间（linear interpolation），不是置信区间。",
    "9. 速度/加速度影响列：内部消融 block 的 reduction (%) 均按同 run 的 SPMPC B0 计算，正值表示相对 B0 降低；外部 baseline 和 hard-cap 阈值补充 block 不填该列，避免跨批次误配。",
    "10. 本表数据来源为 Obsidian 机器表格 CSV；仿真未由 subagent 控制。",
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def parse_float(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    text = str(value).strip()
    if not text or text.lower() in {"nan", "none", "null"}:
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def boolish(value: Any) -> bool:
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "y"}


def yes_no(value: Any) -> str:
    return "yes" if boolish(value) else "no"


def metric_value(row: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = parse_float(row.get(key))
        if value is not None:
            return value
    return None


def freshness_note(row: dict[str, Any]) -> str:
    valid = boolish(row.get("valid_strict_case"))
    pre_ros = boolish(row.get("pre_ros_reachable"))
    pre_gazebo = boolish(row.get("pre_gazebo_reachable"))
    post_ros = boolish(row.get("post_ros_reachable"))
    post_gazebo = boolish(row.get("post_gazebo_reachable"))
    if valid:
        return "strict fresh-sim"
    if pre_ros or pre_gazebo or post_ros or post_gazebo:
        return "freshness violation"
    return "not strict-valid"


def ordered(row: dict[str, Any]) -> OrderedDict[str, Any]:
    return OrderedDict((col, row.get(col, "")) for col in PER_CASE_COLUMNS)


def normalize_internal(rows: Iterable[dict[str, str]]) -> list[OrderedDict[str, Any]]:
    out = []
    for row in rows:
        success = boolish(row.get("goal_reached"))
        out.append(
            ordered(
                {
                    "实验组": "SPMPC内部消融N9",
                    "组别": row.get("group", ""),
                    "算法/方法": row.get("method", ""),
                    "显示名": row.get("label", ""),
                    "Run": row.get("run_index", ""),
                    "strict valid": yes_no(row.get("valid_strict_case")),
                    "goal reached": yes_no(row.get("goal_reached")),
                    "final status": "GOAL_REACHED" if success else row.get("exit_status", ""),
                    "time to goal (s)": metric_value(row, "time_to_goal_sec"),
                    "tracking RMS (m)": metric_value(row, "tracking_rms_m"),
                    "tracking max (m)": metric_value(row, "tracking_max_m"),
                    "final error (m)": metric_value(row, "final_error_m"),
                    "slosh peak (mm)": metric_value(row, "slosh_peak_mm"),
                    "slosh p95 (mm)": metric_value(row, "slosh_p95_mm"),
                    "slosh RMS (mm)": metric_value(row, "slosh_rms_mm"),
                    "driven |v| max (m/s)": metric_value(row, "cmd_v_max_mps"),
                    "driven |omega| max (rad/s)": metric_value(row, "cmd_w_max_radps"),
                    "linear accel p95 (m/s²)": metric_value(row, "dvdt_p95_mps2"),
                    "angular accel p95 (rad/s²)": metric_value(row, "dwdt_p95_radps2"),
                    "freshness note": "strict fresh-sim" if boolish(row.get("valid_strict_case")) else "not strict-valid",
                    "bag path": row.get("bag_path", ""),
                    "summary path": row.get("summary_path", ""),
                }
            )
        )
    return out


def normalize_hardcap(rows: Iterable[dict[str, str]]) -> list[OrderedDict[str, Any]]:
    out = []
    for row in rows:
        cap_label = row.get("cap_label", "")
        cap_mm = row.get("cap_mm", "")
        method = f"spmpc_B_ours_hard_{cap_label}" if cap_label else "spmpc_B_ours_hard"
        out.append(
            ordered(
                {
                    "实验组": "SPMPC hard-cap阈值N9",
                    "组别": f"slosh_height_max={cap_mm} mm" if cap_mm else "slosh_height_max",
                    "算法/方法": method,
                    "显示名": f"SPMPC B_ours_hard ({cap_mm}mm cap)" if cap_mm else "SPMPC B_ours_hard",
                    "Run": row.get("run_index", ""),
                    "strict valid": yes_no(row.get("valid_strict_case")),
                    "goal reached": yes_no(row.get("goal_reached")),
                    "final status": row.get("summary_status", ""),
                    "time to goal (s)": metric_value(row, "time_to_goal_sec"),
                    "tracking RMS (m)": metric_value(row, "tracking_rms_m"),
                    "tracking max (m)": metric_value(row, "tracking_max_m"),
                    "final error (m)": metric_value(row, "final_error_m"),
                    "slosh peak (mm)": metric_value(row, "slosh_monitor_peak_mm"),
                    "slosh p95 (mm)": metric_value(row, "slosh_monitor_p95_mm"),
                    "driven |v| max (m/s)": metric_value(row, "cmd_v_max_mps"),
                    "driven |omega| max (rad/s)": metric_value(row, "cmd_w_max_radps"),
                    "acados fail count": metric_value(row, "acados_fail_count"),
                    "freshness note": freshness_note(row),
                    "reason": row.get("reason", ""),
                    "bag path": row.get("bag_path", ""),
                    "summary path": row.get("summary_path", ""),
                }
            )
        )
    return out


def normalize_external(rows: Iterable[dict[str, str]]) -> list[OrderedDict[str, Any]]:
    out = []
    for row in rows:
        algorithm = row.get("algorithm", "")
        group = "SPMPC hard baseline" if algorithm.startswith("spmpc") else "External baseline"
        out.append(
            ordered(
                {
                    "实验组": "外部baseline N9",
                    "组别": group,
                    "算法/方法": algorithm,
                    "显示名": row.get("algorithm_display", ""),
                    "Run": row.get("run_index", ""),
                    "strict valid": yes_no(row.get("valid_strict_case")),
                    "goal reached": yes_no(row.get("goal_reached")),
                    "final status": row.get("summary_status", ""),
                    "time to goal (s)": metric_value(row, "time_to_goal_sec"),
                    "tracking RMS (m)": metric_value(row, "tracking_rms_m"),
                    "tracking max (m)": metric_value(row, "tracking_max_m"),
                    "final error (m)": metric_value(row, "final_error_m"),
                    "slosh peak (mm)": metric_value(row, "slosh_monitor_peak_mm"),
                    "slosh p95 (mm)": metric_value(row, "slosh_monitor_p95_mm"),
                    "slosh RMS (mm)": metric_value(row, "slosh_monitor_rms_mm"),
                    "driven |v| max (m/s)": metric_value(row, "cmd_drive_v_max_mps", "cmd_v_max_mps"),
                    "driven |omega| max (rad/s)": metric_value(row, "cmd_drive_w_max_radps", "cmd_w_max_radps"),
                    "acados fail count": metric_value(row, "acados_fail_count"),
                    "failure status count": metric_value(row, "failure_status_count"),
                    "freshness note": freshness_note(row),
                    "reason": row.get("reason", ""),
                    "bag path": row.get("bag_path", ""),
                    "summary path": row.get("summary_path", ""),
                }
            )
        )
    return out


def quantile_linear(values: list[float], q: float) -> float | None:
    if not values:
        return None
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return xs[lo]
    return xs[lo] * (hi - pos) + xs[hi] * (pos - lo)


def values_for(rows: list[OrderedDict[str, Any]], col: str) -> list[float]:
    vals = []
    for row in rows:
        value = parse_float(row.get(col))
        if value is not None:
            vals.append(value)
    return vals


def mean(values: list[float]) -> float | None:
    return statistics.mean(values) if values else None


def stdev(values: list[float]) -> float | None:
    if len(values) <= 1:
        return 0.0 if values else None
    return statistics.stdev(values)


def interval(values: list[float], decimals: int = 4) -> str:
    p10 = quantile_linear(values, 0.10)
    p90 = quantile_linear(values, 0.90)
    if p10 is None or p90 is None:
        return ""
    return f"{p10:.{decimals}f}–{p90:.{decimals}f}"


def mean_std(values: list[float], decimals: int = 4) -> str:
    mu = mean(values)
    sd = stdev(values)
    if mu is None or sd is None:
        return ""
    return f"{mu:.{decimals}f}±{sd:.{decimals}f}"


def percent_reduction(baseline: Any, value: Any) -> float | None:
    baseline_f = parse_float(baseline)
    value_f = parse_float(value)
    if baseline_f is None or value_f is None or abs(baseline_f) < 1e-12:
        return None
    return (baseline_f - value_f) / baseline_f * 100.0


def annotate_internal_b0_reductions(per_case: list[OrderedDict[str, Any]]) -> None:
    """Add same-run percentage reductions against SPMPC B0 for internal ablation rows."""
    metric_pairs = [
        ("driven |v| max (m/s)", "vs B0 max v reduction (%)"),
        ("driven |omega| max (rad/s)", "vs B0 max omega reduction (%)"),
        ("linear accel p95 (m/s²)", "vs B0 linear accel p95 reduction (%)"),
        ("angular accel p95 (rad/s²)", "vs B0 angular accel p95 reduction (%)"),
        ("slosh p95 (mm)", "vs B0 slosh p95 reduction (%)"),
    ]
    b0_by_run = {
        str(row.get("Run")): row
        for row in per_case
        if row.get("实验组") == "SPMPC内部消融N9" and row.get("算法/方法") == "spmpc_B0"
    }
    for row in per_case:
        if row.get("实验组") != "SPMPC内部消融N9":
            continue
        baseline = b0_by_run.get(str(row.get("Run")))
        if baseline is None:
            continue
        for metric_col, reduction_col in metric_pairs:
            row[reduction_col] = 0.0 if row.get("算法/方法") == "spmpc_B0" else percent_reduction(baseline.get(metric_col), row.get(metric_col))


def build_averages(per_case: list[OrderedDict[str, Any]]) -> list[OrderedDict[str, Any]]:
    groups: OrderedDict[tuple[str, str, str, str], list[OrderedDict[str, Any]]] = OrderedDict()
    for row in per_case:
        key = (row["实验组"], row["组别"], row["算法/方法"], row["显示名"])
        groups.setdefault(key, []).append(row)

    out: list[OrderedDict[str, Any]] = []
    for (experiment, group, method, display), rows in groups.items():
        time_vals = values_for(rows, "time to goal (s)")
        rms_vals = values_for(rows, "tracking RMS (m)")
        max_vals = values_for(rows, "tracking max (m)")
        final_vals = values_for(rows, "final error (m)")
        peak_vals = values_for(rows, "slosh peak (mm)")
        p95_vals = values_for(rows, "slosh p95 (mm)")
        v_vals = values_for(rows, "driven |v| max (m/s)")
        w_vals = values_for(rows, "driven |omega| max (rad/s)")
        lin_acc_vals = values_for(rows, "linear accel p95 (m/s²)")
        ang_acc_vals = values_for(rows, "angular accel p95 (rad/s²)")
        v_reduction_vals = values_for(rows, "vs B0 max v reduction (%)")
        w_reduction_vals = values_for(rows, "vs B0 max omega reduction (%)")
        lin_acc_reduction_vals = values_for(rows, "vs B0 linear accel p95 reduction (%)")
        ang_acc_reduction_vals = values_for(rows, "vs B0 angular accel p95 reduction (%)")
        slosh_p95_reduction_vals = values_for(rows, "vs B0 slosh p95 reduction (%)")
        acados_vals = values_for(rows, "acados fail count")
        failure_vals = values_for(rows, "failure status count")

        strict_count = sum(1 for row in rows if str(row.get("strict valid", "")).lower() == "yes")
        success_count = sum(1 for row in rows if str(row.get("goal reached", "")).lower() == "yes")
        n = len(rows)
        acados_cases = sum(1 for v in acados_vals if v and v > 0)
        acados_total = sum(acados_vals) if acados_vals else 0
        failure_total = sum(failure_vals) if failure_vals else 0

        out.append(
            OrderedDict(
                [
                    ("实验组", experiment),
                    ("组别", group),
                    ("算法/方法", method),
                    ("显示名", display),
                    ("N", n),
                    ("strict valid", f"{strict_count}/{n}"),
                    ("success", f"{success_count}/{n}"),
                    ("fails", n - success_count),
                    ("time mean (s)", mean(time_vals)),
                    ("time std (s)", stdev(time_vals)),
                    ("time P10-P90 (s)", interval(time_vals, 3)),
                    ("time mean±std (s)", mean_std(time_vals, 3)),
                    ("tracking RMS mean (m)", mean(rms_vals)),
                    ("tracking RMS std (m)", stdev(rms_vals)),
                    ("tracking RMS P10-P90 (m)", interval(rms_vals, 4)),
                    ("tracking RMS mean±std (m)", mean_std(rms_vals, 4)),
                    ("tracking max mean (m)", mean(max_vals)),
                    ("tracking max std (m)", stdev(max_vals)),
                    ("tracking max P10-P90 (m)", interval(max_vals, 4)),
                    ("final error mean (m)", mean(final_vals)),
                    ("final error std (m)", stdev(final_vals)),
                    ("final error P10-P90 (m)", interval(final_vals, 4)),
                    ("slosh peak mean (mm)", mean(peak_vals)),
                    ("slosh peak std (mm)", stdev(peak_vals)),
                    ("slosh peak P10-P90 (mm)", interval(peak_vals, 3)),
                    ("slosh peak mean±std (mm)", mean_std(peak_vals, 3)),
                    ("slosh p95 mean (mm)", mean(p95_vals)),
                    ("slosh p95 std (mm)", stdev(p95_vals)),
                    ("slosh p95 P10-P90 (mm)", interval(p95_vals, 3)),
                    ("slosh p95 mean±std (mm)", mean_std(p95_vals, 3)),
                    ("driven |v| max mean (m/s)", mean(v_vals)),
                    ("driven |v| max std (m/s)", stdev(v_vals)),
                    ("driven |v| max P10-P90 (m/s)", interval(v_vals, 3)),
                    ("driven |omega| max mean (rad/s)", mean(w_vals)),
                    ("driven |omega| max std (rad/s)", stdev(w_vals)),
                    ("driven |omega| max P10-P90 (rad/s)", interval(w_vals, 3)),
                    ("linear accel p95 mean (m/s²)", mean(lin_acc_vals)),
                    ("linear accel p95 std (m/s²)", stdev(lin_acc_vals)),
                    ("linear accel p95 P10-P90 (m/s²)", interval(lin_acc_vals, 3)),
                    ("angular accel p95 mean (rad/s²)", mean(ang_acc_vals)),
                    ("angular accel p95 std (rad/s²)", stdev(ang_acc_vals)),
                    ("angular accel p95 P10-P90 (rad/s²)", interval(ang_acc_vals, 3)),
                    ("vs B0 max v reduction (%)", mean(v_reduction_vals)),
                    ("vs B0 max omega reduction (%)", mean(w_reduction_vals)),
                    ("vs B0 linear accel p95 reduction (%)", mean(lin_acc_reduction_vals)),
                    ("vs B0 angular accel p95 reduction (%)", mean(ang_acc_reduction_vals)),
                    ("vs B0 slosh p95 reduction (%)", mean(slosh_p95_reduction_vals)),
                    ("acados fail cases", acados_cases),
                    ("acados fail total", acados_total),
                    ("failure status total", failure_total),
                    ("说明", GROUP_NOTES.get(experiment, "")),
                ]
            )
        )
    return out


def safe_cell(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if value is None:
        return ""
    return value


def write_sheet(
    workbook: xlsxwriter.Workbook,
    name: str,
    title: str,
    headers: list[str],
    rows: list[OrderedDict[str, Any]],
    notes: list[str],
    numeric_cols: set[str],
) -> None:
    ws = workbook.add_worksheet(name)
    title_fmt = workbook.add_format({"bold": True, "font_size": 14, "bg_color": "#D9EAF7"})
    header_fmt = workbook.add_format({"bold": True, "font_color": "#FFFFFF", "bg_color": "#1F4E78", "border": 1})
    note_title_fmt = workbook.add_format({"bold": True, "bg_color": "#FFF2CC"})
    note_fmt = workbook.add_format({"text_wrap": True, "valign": "top"})
    num_fmt = workbook.add_format({"num_format": "0.0000"})
    count_fmt = workbook.add_format({"num_format": "0"})

    ws.merge_range(0, 0, 0, len(headers) - 1, title, title_fmt)
    for col, header in enumerate(headers):
        ws.write(1, col, header, header_fmt)

    for r_idx, row in enumerate(rows, start=2):
        for c_idx, header in enumerate(headers):
            value = safe_cell(row.get(header, ""))
            if header in numeric_cols and value != "":
                fmt = count_fmt if header in {"N", "fails", "acados fail cases", "acados fail total", "failure status total"} else num_fmt
                ws.write(r_idx, c_idx, value, fmt)
            else:
                ws.write(r_idx, c_idx, value)

    last_data_row = 1 + len(rows)
    ws.add_table(
        1,
        0,
        last_data_row,
        len(headers) - 1,
        {
            "name": "T_" + ("case" if name == "每一次数据" else "avg"),
            "style": "Table Style Medium 2",
            "columns": [{"header": h} for h in headers],
        },
    )
    ws.freeze_panes(2, 4)

    width_overrides = {
        "实验组": 20,
        "组别": 24,
        "算法/方法": 26,
        "显示名": 28,
        "final status": 18,
        "freshness note": 18,
        "reason": 24,
        "bag path": 72,
        "summary path": 72,
        "说明": 70,
    }
    for c_idx, header in enumerate(headers):
        width = width_overrides.get(header, 16)
        if "P10-P90" in header or "mean±std" in header:
            width = 20
        ws.set_column(c_idx, c_idx, width)

    note_start = last_data_row + 3
    ws.write(note_start, 0, notes[0], note_title_fmt)
    for idx, note in enumerate(notes[1:], start=1):
        ws.write(note_start + idx, 0, note, note_fmt)
    ws.set_row(note_start, None, note_title_fmt)
    for idx in range(1, len(notes)):
        ws.set_row(note_start + idx, 36)


def main() -> None:
    for label, path in SOURCE_FILES.items():
        if not path.exists():
            raise FileNotFoundError(f"missing source CSV for {label}: {path}")

    per_case: list[OrderedDict[str, Any]] = []
    per_case.extend(normalize_internal(read_csv(SOURCE_FILES["internal_per_case"])))
    per_case.extend(normalize_hardcap(read_csv(SOURCE_FILES["hardcap_per_case"])))
    per_case.extend(normalize_external(read_csv(SOURCE_FILES["external_per_case"])))
    annotate_internal_b0_reductions(per_case)
    avg_rows = build_averages(per_case)

    if len(per_case) != 117:
        raise RuntimeError(f"unexpected per-case row count: {len(per_case)} != 117")
    if len(avg_rows) != 13:
        raise RuntimeError(f"unexpected average row count: {len(avg_rows)} != 13")

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    if OUT_PATH.exists():
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = OUT_PATH.with_name(f"{OUT_PATH.stem}.bak_{stamp}{OUT_PATH.suffix}")
        shutil.copy2(OUT_PATH, backup)

    workbook = xlsxwriter.Workbook(str(OUT_PATH), {"nan_inf_to_errors": False})
    workbook.set_properties(
        {
            "title": "固定路径 S 曲线 N=9 全量对比表",
            "subject": "SPMPC internal ablation, hard-cap threshold, and external baselines",
            "comments": "Generated from Obsidian machine-readable CSV tables; includes P10-P90 intervals.",
        }
    )
    write_sheet(
        workbook,
        "每一次数据",
        "全量固定路径 S 曲线 N=9 对比：每一次数据",
        PER_CASE_COLUMNS,
        per_case,
        FAIRNESS_NOTES,
        NUMERIC_PER_CASE | {"Run"},
    )
    write_sheet(
        workbook,
        "每组平均",
        "全量固定路径 S 曲线 N=9 对比：每组平均（含 P10-P90）",
        AVG_COLUMNS,
        avg_rows,
        FAIRNESS_NOTES,
        NUMERIC_AVG | {"N", "fails", "failure status total"},
    )
    workbook.close()

    print(f"wrote {OUT_PATH}")
    print(f"per_case_rows={len(per_case)} average_rows={len(avg_rows)}")
    print("added P10-P90 interval columns and speed/acceleration impact analysis")


if __name__ == "__main__":
    main()
