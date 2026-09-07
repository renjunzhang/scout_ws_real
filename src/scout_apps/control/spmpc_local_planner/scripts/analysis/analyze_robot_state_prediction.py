#!/usr/bin/env python3
"""Offline C03 diagnosis: original vs independently checked robot initial state.

Reuses the existing C03 recorder and mocap/command-audit conventions. Lazy
rosbag import keeps --help, numerical tests and report tests usable without ROS.
This program never connects to a ROS master or publishes commands.
"""

import argparse
import csv
import datetime
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from analyze_mocap_execution_chain import quaternion_yaw, finite_summary
from analyze_same_bag_actuator_delay import file_sha256, stamp_sec
from robot_state_prediction_core import (
    ContractError, ROBOT_FIELDS, causal_support, check_command_support,
    check_snapshot_history, error_metrics, fit_state, reference_issues,
    replace_robot_initial_state, replay_robot, snapshot_contract, state_error,
    strict_rows, transform_mocap, wrap_angle,
)


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_REFERENCE = SCRIPT_DIR.parents[1] / "config/experiments/robot_state_reference.template.json"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
IMU_DEBUG_TOPIC = "/spmpc/debug/slosh_observer_imu"
SNAPSHOT_SCALARS = (
    "schema_version valid backend variant solver_status control_semantics "
    "state_width control_width parameter_width dt horizon_steps actuator_state_valid "
    "robot_x robot_y robot_yaw robot_v robot_omega s0 "
    "eta_x eta_x_dot eta_y eta_y_dot actuator_v_cmd actuator_omega_cmd "
    "actuator_a_cmd_memory actuator_delayed_v_cmd actuator_delayed_omega_cmd "
    "jerk_limit_enable jerk_max zero_liquid_initial_state"
).split()
SNAPSHOT_ARRAYS = ("parameter_names", "stage_parameters",
                   "actuator_linear_delay_queue", "actuator_angular_delay_queue")
AUDIT_FLAGS = ("solve_attempted solve_success command_was_published command_accepted "
               "command_contract_violation terminal_phase terminal_controller_intervened "
               "safety_gate_intervened zero_due_to_speed_safety").split()
INTERVENTIONS = ("terminal_phase", "terminal_controller_intervened",
                 "safety_gate_intervened", "zero_due_to_speed_safety",
                 "command_contract_violation")


def read_records(messages, tracker):
    """Also accepts synthetic ROS-shaped messages for meaningful reader tests."""
    data = {key: [] for key in ("snapshots", "audits", "cmd", "mocap", "odom", "imu",
                                "imu_debug", "effective_config")}
    counts, invalid = Counter(), Counter()
    timing, frames = {}, {"solver": set(), "mocap": set()}
    mocap_topic = "/vrpn_client_node/{}/pose".format(tracker)

    def record_stamp(topic, physical_stamp, bag_stamp):
        if physical_stamp <= 0:
            invalid[topic] += 1
            return False
        timing.setdefault(topic, []).append([physical_stamp, bag_stamp - physical_stamp])
        return True

    for topic, msg, bag_stamp in messages:
        counts[topic] += 1
        bag_time = stamp_sec(bag_stamp)
        if topic == SNAPSHOT_TOPIC:
            record = {key: getattr(msg, key, None) for key in SNAPSHOT_SCALARS}
            record.update({key: list(getattr(msg, key, [])) for key in SNAPSHOT_ARRAYS})
            record["cycle_id"] = int(msg.cycle_id)
            for field in ("solver_input_epoch", "raw_robot_state_stamp", "robot_state_stamp"):
                record[field] = stamp_sec(getattr(msg, field, None))
            record["frame"] = str(msg.header.frame_id)
            frames["solver"].add(record["frame"])
            data["snapshots"].append(record)
            record_stamp(topic, record["solver_input_epoch"], bag_time)
        elif topic == AUDIT_TOPIC:
            record = {key: bool(getattr(msg, key, False)) for key in AUDIT_FLAGS}
            record.update(cycle_id=int(msg.cycle_id), bag_time=bag_time,
                          epoch=stamp_sec(msg.solver_input_epoch),
                          stamp=stamp_sec(msg.command_publish_stamp),
                          v=float(msg.published_cmd_v), omega=float(msg.published_cmd_omega))
            data["audits"].append(record)
            if record["command_was_published"]:
                record_stamp(topic, record["stamp"], bag_time)
        elif topic == "/cmd_vel":
            # Headerless values used ONLY to check audit completeness, never phase.
            data["cmd"].append([bag_time, float(msg.linear.x), float(msg.angular.z)])
        elif topic == "/spmpc/debug/effective_config":
            value = str(msg.data)
            if value not in data["effective_config"]:
                data["effective_config"].append(value)
        elif topic == IMU_DEBUG_TOPIC:
            if bool(msg.valid) and bool(msg.filter_ready) and bool(msg.bias_ready):
                data["imu_debug"].append({
                    "measurement_stamp": stamp_sec(msg.measurement_stamp),
                    "accel_effective_stamp": stamp_sec(msg.accel_effective_stamp),
                    "gyro_effective_stamp": stamp_sec(msg.gyro_effective_stamp),
                    "axes": str(msg.excitation_axes_frame),
                    "point": str(msg.excitation_reference_point),
                    "ax": float(msg.ax_mps2), "ay": float(msg.ay_mps2),
                    "omega": float(msg.omega_z_radps),
                })
        elif topic in (mocap_topic, "/odom", "/imu/data"):
            stamp = stamp_sec(msg.header.stamp)
            if not record_stamp(topic, stamp, bag_time):
                continue
            if topic == mocap_topic:
                frames["mocap"].add(str(msg.header.frame_id))
                p, q = msg.pose.position, msg.pose.orientation
                norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if not math.isfinite(norm) or abs(norm - 1) > 0.01:
                    invalid["mocap_quaternion"] += 1
                    continue
                data["mocap"].append([stamp, p.x, p.y, quaternion_yaw(q)])
            elif topic == "/odom":
                data["odom"].append([stamp, msg.twist.twist.linear.x, msg.twist.twist.angular.z])
            else:
                data["imu"].append([stamp, msg.linear_acceleration.x,
                                    msg.linear_acceleration.y, msg.angular_velocity.z])
    data.update(counts=dict(counts), invalid=dict(invalid), timing=timing, frames=frames)
    return data


def read_bag(path, tracker):
    try:
        import rosbag
    except ImportError as exc:
        raise ContractError("缺少 rosbag：请在 ROS Noetic 终端 source 后使用 python3") from exc
    topics = [SNAPSHOT_TOPIC, AUDIT_TOPIC, "/cmd_vel", "/odom", "/imu/data",
              IMU_DEBUG_TOPIC, "/spmpc/debug/effective_config",
              "/vrpn_client_node/{}/pose".format(tracker)]
    with rosbag.Bag(str(path), "r") as bag:
        return read_records(bag.read_messages(topics=topics), tracker)


def command_ledger(audits, raw_commands, tolerance_sec=0.15):
    """Monotone one-to-one value+nearby-arrival join, not zip or a phase fit."""
    raw = np.asarray(raw_commands, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 3 or not len(raw):
        return {"valid": False, "reason": "RAW_CMD_MISSING"}
    used, missing, last = set(), [], -1
    emitted = [a for a in audits if a["command_was_published"]]
    for audit in emitted:
        first = max(last + 1, int(np.searchsorted(raw[:, 0], audit["bag_time"] - tolerance_sec)))
        end = np.searchsorted(raw[:, 0], audit["bag_time"] + tolerance_sec, side="right")
        found = next((i for i in range(first, end)
                      if np.allclose(raw[i, 1:], [audit["v"], audit["omega"]],
                                     atol=1e-9, rtol=0)), None)
        if found is None:
            missing.append(audit["cycle_id"])
        else:
            used.add(found)
            last = found
    extra_inside, outside_zero = [], 0
    if emitted:
        begin, end = emitted[0]["bag_time"], emitted[-1]["bag_time"]
        for i in range(len(raw)):
            if i in used:
                continue
            if (raw[i, 0] < begin - tolerance_sec or raw[i, 0] > end + tolerance_sec) and np.max(np.abs(raw[i, 1:])) < 1e-9:
                outside_zero += 1
            else:
                extra_inside.append(i)
    return {"valid": bool(emitted) and not missing and not extra_inside,
            "audit_published": len(emitted), "raw_commands": len(raw), "matched": len(used),
            "unmatched_audit_cycles": missing, "unmatched_raw_indices": extra_inside,
            "extra_zero_outside_audit_interval": outside_zero,
            "arrival_join_tolerance_sec": tolerance_sec}


def timing_summary(data):
    result = {}
    for topic, values in data["timing"].items():
        array = np.asarray(values)
        gaps = np.diff(array[:, 0])
        result[topic] = {"count": len(array), "arrival_minus_stamp_sec": finite_summary(array[:, 1]),
                         "stamp_interval_sec": finite_summary(gaps),
                         "repeated_or_regressing": int(np.sum(gaps <= 0))}
    return result


def evaluate(data, reference, args, bag_sha256):
    issues = reference_issues(reference, data["frames"]["solver"], data["frames"]["mocap"], bag_sha256)
    ledger = command_ledger(data["audits"], data["cmd"])
    if not ledger["valid"]:
        issues.append("COMMAND_LEDGER_INCOMPLETE")
    if data["invalid"]:
        issues.append("INVALID_SOURCE_MESSAGES")
    if not data["effective_config"]:
        issues.append("EFFECTIVE_CONFIG_MISSING")
    report = {"status": "INCONCLUSIVE", "issues": issues, "counts": data["counts"],
              "invalid_messages": data["invalid"], "timing": timing_summary(data),
              "command_ledger": ledger, "reference": reference,
              "effective_config": data["effective_config"],
              "whole_bag_runtime": {"solve_attempts": sum(a["solve_attempted"] for a in data["audits"]),
                  "solve_failures": sum(a["solve_attempted"] and not a["solve_success"] for a in data["audits"]),
                  **{flag: sum(a[flag] for a in data["audits"]) for flag in INTERVENTIONS}},
              "method": {"leads_sec": args.leads, "causal_cubic_window_sec": args.window_sec,
                         "max_mocap_gap_sec": args.max_mocap_gap_sec,
                         "max_command_gap_sec": args.max_command_gap_sec,
                         "integration_step_sec": 0.002, "fit_parameters": False,
                         "replay": "same published ZOH commands + frozen continuous FOPDT",
                         "future_comparison": "identical causal cubic kernel on model and mocap positions",
                         "no_unique_root_cause_claim": True}}
    initial_rows, future_rows, anchors, exclusions = [], [], [], []
    excluded = Counter()
    try:
        commands = strict_rows([[a["stamp"], a["v"], a["omega"]] for a in data["audits"]
                                if a["command_was_published"]], 3, "audit commands")
        mocap = transform_mocap(data["mocap"], reference)
        if any("FRAME_MISMATCH" in issue for issue in issues):
            raise ContractError("FRAME_MISMATCH: 不对不同坐标系生成误差")
        audits = {a["cycle_id"]: a for a in data["audits"]}
        if len(audits) != len(data["audits"]):
            raise ContractError("DUPLICATE_AUDIT_CYCLE_ID")
        snapshots = data["snapshots"]
        if len({s["cycle_id"] for s in snapshots}) != len(snapshots):
            raise ContractError("DUPLICATE_SNAPSHOT_CYCLE_ID")
        previous_epoch = 0.0
        model_signatures = set()
        conditions = set()
        for snapshot in snapshots:
            try:
                if not snapshot["valid"] or snapshot["solver_input_epoch"] <= 0:
                    raise ContractError("INVALID_SNAPSHOT")
                epoch = snapshot["solver_input_epoch"]
                if epoch <= previous_epoch:
                    raise ContractError("REPEATED_OR_REGRESSING_SOLVER_EPOCH")
                previous_epoch = epoch
                audit = audits.get(snapshot["cycle_id"])
                if audit is None or abs(audit["epoch"] - epoch) > 1e-6:
                    raise ContractError("AUDIT_CYCLE_OR_EPOCH_MISMATCH")
                if (not audit["solve_success"] or not audit["command_was_published"]
                        or not audit["command_accepted"] or any(audit[f] for f in INTERVENTIONS)):
                    raise ContractError("NON_CLEAN_ANCHOR")
                if abs(snapshot["robot_state_stamp"] - epoch) > 1e-6:
                    raise ContractError("ROBOT_STATE_EPOCH_MISMATCH")
                if max(args.leads) > snapshot["dt"] * snapshot["horizon_steps"] + 1e-6:
                    raise ContractError("LEAD_EXCEEDS_SNAPSHOT_HORIZON")
                params = snapshot_contract(snapshot)
                model_signatures.add(json.dumps(params, sort_keys=True))
                conditions.add(json.dumps({name: snapshot[name] for name in
                    ("variant", "jerk_limit_enable", "jerk_max", "zero_liquid_initial_state", "state_width")}, sort_keys=True))
                check_command_support(commands, epoch - max(params["delay_v"], params["delay_omega"]),
                                      epoch + max(args.leads), args.max_command_gap_sec)
                check_snapshot_history(snapshot, commands)
                support0 = causal_support(mocap[:, 0], epoch, args.window_sec, args.max_mocap_gap_sec)
                corrected, lateral = fit_state(mocap[support0, 0], mocap[support0, 1:], epoch, args.window_sec)
                original = np.array([snapshot["robot_" + name] for name in ROBOT_FIELDS])
                error0 = state_error(original, corrected)
                initial_row = {"cycle_id": snapshot["cycle_id"], "epoch": epoch,
                               "reference_lateral_mps": lateral,
                               "raw_to_solver_sec": epoch - snapshot["raw_robot_state_stamp"]}
                for j, name in enumerate(ROBOT_FIELDS):
                    initial_row.update({"original_" + name: original[j], "reference_" + name: corrected[j],
                                        "error_" + name: error0[j]})
                # Future support must lie strictly after the anchor; no shared past
                # samples are permitted to make B look artificially good at short leads.
                supports = [causal_support(mocap[:, 0], epoch + lead, args.window_sec,
                                           args.max_mocap_gap_sec) for lead in args.leads]
                indices = np.unique(np.concatenate(supports))
                times = mocap[indices, 0]
                if times[0] <= epoch:
                    raise ContractError("FUTURE_SUPPORT_OVERLAPS_INITIAL_WINDOW")
                modified = replace_robot_initial_state(snapshot, corrected)
                pair = []
                for source in (snapshot, modified):
                    state = np.array([source["robot_" + name] for name in ROBOT_FIELDS])
                    pair.append(replay_robot(state, epoch, times, commands, params))
                local_rows = []
                for lead, support in zip(args.leads, supports):
                    target = epoch + lead
                    ref, future_lateral = fit_state(mocap[support, 0], mocap[support, 1:], target, args.window_sec)
                    local = np.searchsorted(indices, support)
                    row = {"cycle_id": snapshot["cycle_id"], "epoch": epoch, "lead_sec": lead,
                           "target_epoch": target, "reference_lateral_mps": future_lateral,
                           "future_intervention": any(any(a[f] for f in INTERVENTIONS)
                               for a in data["audits"] if epoch < a["stamp"] <= target)}
                    for j, name in enumerate(ROBOT_FIELDS):
                        row["reference_" + name] = ref[j]
                    for label, states in zip(("A", "B"), pair):
                        matched, _ = fit_state(times[local], states[local, :3], target, args.window_sec)
                        error = state_error(matched, ref)
                        for j, name in enumerate(ROBOT_FIELDS):
                            row[label + "_" + name] = matched[j]
                            row[label + "_error_" + name] = error[j]
                        row[label + "_error_position"] = float(np.linalg.norm(error[:2]))
                    local_rows.append(row)
                initial_rows.append(initial_row)
                future_rows.extend(local_rows)
                held = {k: v for k, v in snapshot.items()
                        if k.startswith("actuator_") or k.startswith("eta_") or k == "s0"}
                anchors.append({"cycle_id": snapshot["cycle_id"], "epoch": epoch,
                                "original_robot": original.tolist(), "reference_robot": corrected.tolist(),
                                "unchanged_nonrobot_initial_state": held, "model": params})
            except (ContractError, TypeError, KeyError, ValueError) as exc:
                excluded[str(exc)] += 1
                exclusions.append({"cycle_id": snapshot.get("cycle_id"),
                                   "epoch": snapshot.get("solver_input_epoch"), "reason": str(exc)})
        report["frozen_models"] = [json.loads(value) for value in sorted(model_signatures)]
        report["conditions"] = [json.loads(value) for value in sorted(conditions)]
        if len(model_signatures) > 1:
            issues.append("ACTUATOR_PARAMETERS_CHANGED_WITHIN_BAG")
        if len(conditions) > 1:
            issues.append("ABLATION_SETTINGS_CHANGED_WITHIN_BAG")
    except (ContractError, TypeError, KeyError, ValueError) as exc:
        issues.append(str(exc))
    if not initial_rows:
        issues.append("NO_VALID_PAIRED_ANCHORS")
    report["excluded_anchors"] = dict(excluded)
    report["excluded_anchor_details"] = exclusions
    report["paired_anchors"] = len(initial_rows)
    report["initial_errors"] = {name: error_metrics([r["error_" + name] for r in initial_rows])
                                 for name in ROBOT_FIELDS}
    report["reference_lateral_velocity"] = finite_summary([r["reference_lateral_mps"] for r in initial_rows])
    report["future_errors"] = []
    for lead in args.leads:
        for group in ("all", "no_future_intervention"):
            rows = [r for r in future_rows if r["lead_sec"] == lead
                    and (group == "all" or not r["future_intervention"])]
            metrics = {"lead_sec": lead, "group": group, "count": len(rows)}
            for name in ("position", "yaw", "v", "omega"):
                a = error_metrics([r["A_error_" + name] for r in rows])
                b = error_metrics([r["B_error_" + name] for r in rows])
                improvement = 100 * (1 - b["rmse"] / a["rmse"]) if a.get("rmse", 0) > 1e-12 else None
                metrics[name] = {"A": a, "B": b, "rmse_reduction_pct": improvement}
            report["future_errors"].append(metrics)
    if not issues:
        report["status"] = "REFERENCE_CHECKED_REVIEW_REQUIRED"
    return report, initial_rows, future_rows, anchors


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_plots(output, data, report, initial):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # Same colorblind-safe blue/orange convention as the existing diagnostic plots.
    blue, orange = "#0072B2", "#D55E00"
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    note = "UNVERIFIED / INCONCLUSIVE - descriptive only" if report["issues"] else "Checked reference - exploratory A/B, not a root-cause verdict"
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), constrained_layout=True)
    origin = min((values[0][0] for values in data["timing"].values() if values), default=0)
    for topic, values in data["timing"].items():
        if topic == SNAPSHOT_TOPIC:
            continue
        values = np.asarray(values)
        if len(values) < 2:
            continue
        t = values[:, 0] - origin
        label = topic.rsplit("/", 1)[-1] if "vrpn" not in topic else "raw mocap"
        axes[0].plot(t, 1000 * values[:, 1], label=label, linewidth=0.7)
        axes[1].plot(t[1:], 1000 * np.diff(values[:, 0]), label=label, linewidth=0.7)
    axes[0].set(ylabel="Bag arrival - stamp (ms)", title="Transport bookkeeping; not a physical delay calibration")
    axes[1].set(xlabel="Time from first recorded source stamp (s)", ylabel="Stamp interval (ms)")
    for ax in axes:
        ax.grid(alpha=0.25)
        if ax.lines:
            ax.legend(loc="upper right", ncol=3, fontsize=8)
    fig.suptitle(note)
    fig.savefig(str(output / "01_timing.png"), dpi=150)
    plt.close(fig)
    if not initial:
        return
    t = np.array([r["epoch"] for r in initial])
    t -= t[0]
    fig, axes = plt.subplots(5, 2, figsize=(13, 12), constrained_layout=True)
    units = ("m", "m", "rad", "m/s", "rad/s")
    for i, (name, unit) in enumerate(zip(ROBOT_FIELDS, units)):
        original = np.array([r["original_" + name] for r in initial])
        reference = np.array([r["reference_" + name] for r in initial])
        if name == "yaw":
            original = np.unwrap(original)
            reference = original + wrap_angle(reference - original)
        axes[i, 0].plot(t, original, color=blue, label="Solver initial state")
        axes[i, 0].plot(t, reference, color=orange, linestyle="--", label="Past-only mocap fit")
        axes[i, 1].plot(t, [r["error_" + name] for r in initial], color=blue)
        axes[i, 1].axhline(0, color="0.5", linewidth=0.7)
        for ax in axes[i]:
            ax.set_ylabel(name + " (" + unit + ")")
            ax.grid(alpha=0.25)
    axes[0, 0].set_title("Current state at solver epoch (yaw unwrapped)")
    axes[0, 0].legend(fontsize=9)
    axes[0, 1].set_title("Original - reference (yaw wrapped)")
    for ax in axes[-1]:
        ax.set_xlabel("Time from first paired anchor (s)")
    fig.suptitle(note)
    fig.savefig(str(output / "02_current_state.png"), dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    metrics = [r for r in report["future_errors"] if r["group"] == "no_future_intervention" and r["count"]]
    for ax, name, unit in zip(axes.flat, ("position", "yaw", "v", "omega"), ("m", "rad", "m/s", "rad/s")):
        for label, color, style in (("A", blue, "o-"), ("B", orange, "s--")):
            ax.plot([r["lead_sec"] for r in metrics], [r[name][label]["rmse"] for r in metrics],
                    style, color=color, label=label + (": original initial state" if label == "A" else ": reference initial state"))
        ax.set(xlabel="Prediction lead (s)", ylabel=name + " RMSE (" + unit + ")", ylim=(0, None))
        ax.grid(alpha=0.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(note + "\nSame commands and model; matched derivative processing; no future intervention")
    fig.savefig(str(output / "03_future_error.png"), dpi=150)
    plt.close(fig)


def write_report(output, report, initial, future, anchors, data):
    output.joinpath("summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    output.joinpath("anchors.json").write_text(json.dumps(anchors, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_csv(output / "current_state.csv", initial)
    write_csv(output / "future_errors.csv", future)
    write_csv(output / "imu_reference_timing.csv", data["imu_debug"])
    lines = ["# 车体初态与未来运动预测诊断", "", "状态：`{}`；有效配对初态 **{}** 个。".format(report["status"], report["paired_anchors"]), "",
             "A：原求解车体初态；B：只替换为独立参考车体初态。两组使用同一实际发布命令历史和快照执行器参数，未重求解、未调参。", ""]
    if report["issues"]:
        lines += ["**暂不能归因。** 以下问题未解决时，图表仅作描述；模板中的零外参不是标定结果。", ""]
        lines += ["- " + issue for issue in report["issues"]]
    else:
        lines += ["参考合同已具备，但仍需结合下列误差、不确定度及重复实物包人工判断；这不是控制或降晃验收 PASS。"]
    uncertainty = report["reference"].get("uncertainty", {})
    lines += ["", "参考来源：{}。声明的不确定度：`{}`；脚本不自动验证标定记录的真实性。".format(
        report["reference"].get("provenance") or "未提供", uncertainty)]
    if uncertainty.get("time_sec") is not None:
        lines += ["时间不确定度在 5 Hz 下对应约 ±{:.1f}°；需要结合误差量级和重复试验人工判断。".format(
            360 * 5 * float(uncertainty["time_sec"]))]
    lines += ["", "当前状态误差（原初态 − 动捕参考，使用未来也具备完整覆盖的共同配对初态）：", "", "| 量 | RMSE | 绝对误差 P95 |", "|---|---:|---:|"]
    for name in ROBOT_FIELDS:
        metric = report["initial_errors"][name]
        if metric.get("count"):
            lines.append("| {} | {:.6g} | {:.6g} |".format(name, metric["rmse"], metric["p95_abs"]))
    lines += ["", "未来误差只列未遇到终端/安全干预的窗口；全部窗口也保存在 CSV/JSON。下降率为 `1 − B_RMSE/A_RMSE`，负数表示变差。", "",
              "| 提前量 / s | 配对数 | 量 | A RMSE | B RMSE | 下降率 |", "|---:|---:|---|---:|---:|---:|"]
    for row in report["future_errors"]:
        if row["group"] != "no_future_intervention" or not row["count"]:
            continue
        for name in ("position", "yaw", "v", "omega"):
            metric = row[name]
            reduction = metric["rmse_reduction_pct"]
            lines.append("| {:.3f} | {} | {} | {:.6g} | {:.6g} | {} |".format(
                row["lead_sec"], row["count"], name, metric["A"]["rmse"], metric["B"]["rmse"],
                "分母接近零" if reduction is None else "{:.1f}%".format(reduction)))
    lines += ["", "单位：x/y/position 为 m，yaw 为 rad，v 为 m/s，omega 为 rad/s。", "",
              "如何判断（逐个量判断，不强行二选一）：", "",
              "- B 在多个提前量明显改善，且差异大于参考不确定度：支持初态或对齐误差有贡献。",
              "- 当前初态在参考误差范围内一致，A/B 的未来误差仍明显：优先检查执行器/车体动态及模型外扰动。",
              "- B 部分改善但仍有残余：两方面都可能有贡献。B 更差也可能是参考噪声、外参或处理带宽问题。", "",
              "解释边界：", "",
              "- 未在评价轨迹上拟合坐标、时间、delay、tau 或 gain；独立标定需人工提供来源及不确定度。",
              "- 动捕当前初态只用物理时刻不晚于 t0 的过去样本。未来误差两边用同一后向三次拟合核，默认 120 ms；不把该滤波结果当 5 Hz 加速度真值。",
              "- 位置/速度属于已声明 base 参考点。非零侧向速度提示非完整车体模型的适用边界，也可能来自参考外参误差。",
              "- 回放是已知未来命令的连续 FOPDT 车体子系统诊断，不是在线 horizon、重新优化或液体/RGB 回放。液体、FIFO、命令记忆等原始初值存入 anchors.json；B 只改五个车体分量。",
              "- IMU 保留原始/measurement/effective 时间证据；不以未匹配处理的 IMU 加速度直接判模型错误，也不把 IMU 二次积分当位置真值。",
              "- 时间偏移不确定度在 5 Hz 下每 20 ms 对应 36°；低相干性或未核验外参时不能反算 delay/gain。",
              "- 同包滚动窗口高度重叠，配对数不是独立实验次数；无 RGB 时不能证明实物降晃收益。", "",
              "排除原因：`{}`。整包失败与干预统计：`{}`。".format(report["excluded_anchors"], report["whole_bag_runtime"]), "",
              "图：01 时间记录；02 当前初态及误差；03 两种初态下的未来误差。summary.json 保留文件 SHA、参数、时间与合同；CSV 保留每拍明细。", ""]
    output.joinpath("summary.md").write_text("\n".join(lines), encoding="utf-8")
    make_plots(output, data, report, initial)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="离线区分车体初态与未来运动预测误差；不会启动 ROS 或底盘。")
    parser.add_argument("bag", type=Path, help="已有 C03 full/J1 bag 的明确路径；不自动选择最新包")
    parser.add_argument("--reference-config", type=Path, default=DEFAULT_REFERENCE,
                        help="独立参考标定 JSON；默认模板未核验，只输出暂不能归因的诊断")
    parser.add_argument("--output-dir", type=Path, help="必须是不存在的新目录；默认在 bag 旁创建时间戳目录")
    parser.add_argument("--tracker", default="Tracker0")
    parser.add_argument("--leads", type=float, nargs="+", default=[1/6, 1/3, 2/3, 1.0, 2.0])
    parser.add_argument("--window-sec", type=float, default=0.12, help="后向三次拟合窗口；必须小于最短提前量")
    parser.add_argument("--max-mocap-gap-sec", type=float, default=0.035)
    parser.add_argument("--max-command-gap-sec", type=float, default=0.12)
    args = parser.parse_args(argv)
    values = args.leads + [args.window_sec, args.max_mocap_gap_sec, args.max_command_gap_sec]
    if any(not math.isfinite(value) or value <= 0 for value in values):
        parser.error("时间参数必须是有限正数")
    if (args.leads != sorted(set(args.leads)) or min(args.leads) <= args.window_sec
            or args.max_mocap_gap_sec >= args.window_sec):
        parser.error("leads 必须严格递增且大于拟合窗口；允许的动捕间隙必须小于拟合窗口")
    return args


def main(argv=None):
    args = parse_args(argv)
    output = args.output_dir or args.bag.parent / (args.bag.stem + "_state_prediction_" +
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    try:
        if not args.bag.is_file():
            raise ContractError("bag 不存在：" + str(args.bag))
        reference = json.loads(args.reference_config.read_text(encoding="utf-8"))
        if reference.get("schema_version") != 1:
            raise ContractError("不支持的 reference schema")
        output.mkdir(parents=True, exist_ok=False)
        bag_sha = file_sha256(args.bag)
        data = read_bag(args.bag, args.tracker)
        report, initial, future, anchors = evaluate(data, reference, args, bag_sha)
        git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(SCRIPT_DIR),
                             text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        report["provenance"] = {"bag": str(args.bag.resolve()), "bag_sha256": bag_sha,
            "reference_config": str(args.reference_config.resolve()),
            "reference_sha256": file_sha256(args.reference_config), "analysis_git": git.stdout.strip(),
            "analysis_files_sha256": {p.name: file_sha256(p) for p in
                (Path(__file__), SCRIPT_DIR / "robot_state_prediction_core.py",
                 SCRIPT_DIR / "analyze_mocap_execution_chain.py", SCRIPT_DIR / "analyze_same_bag_actuator_delay.py")},
            "command_line": sys.argv if argv is None else list(argv)}
        write_report(output, report, initial, future, anchors, data)
        print("{}：{} 个配对；报告 {}".format(report["status"], len(initial), output / "summary.md"))
        return 0 if initial else 2
    except (ContractError, OSError, ValueError, KeyError, TypeError) as exc:
        # Do not touch an existing directory on a failed re-run.
        print("诊断未完成：{}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
