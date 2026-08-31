#!/usr/bin/env python3
"""Identify command-to-motion timing from matched real-robot bags.

The tool reads the final ``/cmd_vel`` topic and compares it on the same bag
clock with raw NOKOV pose, wheel odometry, and ``/imu/data``.  It also audits
processed-IMU timestamp semantics and performs leave-one-bag-out evaluation.

It is an offline reader only: it never publishes ROS messages, changes a bag,
or writes planner parameters.
"""

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_mocap_execution_chain as mocap_tools  # noqa: E402
import same_bag_delay_core as core  # noqa: E402


CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data"
AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
IMU_DEBUG_TOPIC = "/spmpc/debug/slosh_observer_imu"
MOCAP_STATUS_TOPIC = "/mocap/status"
SCOUT_STATUS_TOPIC = "/scout_status"


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_sec(stamp):
    try:
        value = float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) and value > 0.0 else 0.0


def message_header_time(message, bag_time):
    header = getattr(message, "header", None)
    value = stamp_sec(getattr(header, "stamp", None))
    return value if value > 0.0 else float(bag_time)


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def read_bag(path, tracker):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is unavailable; source the ROS workspace") from exc

    mocap_topic = "/vrpn_client_node/{}/pose".format(tracker)
    raw = {
        "cmd": [],
        "odom_bag": [],
        "odom_header": [],
        "imu_bag": [],
        "imu_header": [],
        "mocap_bag": [],
        "mocap_header": [],
        "audit": [],
        "imu_debug": [],
        "mocap_status": [],
        "scout_status": [],
        "arrival_minus_header": {"odom": [], "imu": [], "mocap": []},
    }
    topics = [
        CMD_TOPIC,
        ODOM_TOPIC,
        IMU_TOPIC,
        mocap_topic,
        AUDIT_TOPIC,
        IMU_DEBUG_TOPIC,
        MOCAP_STATUS_TOPIC,
        SCOUT_STATUS_TOPIC,
    ]
    with rosbag.Bag(str(path), "r") as bag:
        raw["bag_start_sec"] = float(bag.get_start_time())
        raw["bag_end_sec"] = float(bag.get_end_time())
        available = set(bag.get_type_and_topic_info().topics.keys())
        required = {CMD_TOPIC, ODOM_TOPIC, IMU_TOPIC, mocap_topic, AUDIT_TOPIC}
        missing = sorted(required - available)
        if missing:
            raise RuntimeError("{} is missing required topics: {}".format(path, missing))
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            bag_time = float(bag_stamp.to_sec())
            if topic == CMD_TOPIC:
                raw["cmd"].append(
                    (bag_time, float(message.linear.x), float(message.angular.z))
                )
            elif topic == ODOM_TOPIC:
                header_time = message_header_time(message, bag_time)
                values = (
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.angular.z),
                )
                raw["odom_bag"].append((bag_time, *values))
                raw["odom_header"].append((header_time, *values))
                raw["arrival_minus_header"]["odom"].append(bag_time - header_time)
            elif topic == IMU_TOPIC:
                header_time = message_header_time(message, bag_time)
                values = (
                    float(message.angular_velocity.z),
                    float(message.linear_acceleration.x),
                    float(message.linear_acceleration.y),
                )
                raw["imu_bag"].append((bag_time, *values))
                raw["imu_header"].append((header_time, *values))
                raw["arrival_minus_header"]["imu"].append(bag_time - header_time)
            elif topic == mocap_topic:
                header_time = message_header_time(message, bag_time)
                values = (
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    quaternion_yaw(message.pose.orientation),
                )
                raw["mocap_bag"].append((bag_time, *values))
                raw["mocap_header"].append((header_time, *values))
                raw["arrival_minus_header"]["mocap"].append(bag_time - header_time)
            elif topic == AUDIT_TOPIC:
                publish_time = stamp_sec(message.command_publish_stamp) or bag_time
                raw["audit"].append(
                    {
                        "bag_time": bag_time,
                        "publish_time": publish_time,
                        "published": bool(message.command_was_published),
                        "solve_success": bool(message.solve_success),
                        "terminal": bool(message.terminal_phase),
                        "safety": bool(message.safety_gate_intervened),
                        "cmd_v": float(message.published_cmd_v),
                        "cmd_omega": float(message.published_cmd_omega),
                        "shift_available": bool(message.previous_shifted_plan_available),
                        "previous_a": float(message.previous_shifted_plan_a),
                        "previous_alpha": float(message.previous_shifted_plan_alpha),
                        "delta_a": float(message.replanned_minus_shifted_a),
                        "delta_alpha": float(message.replanned_minus_shifted_alpha),
                    }
                )
            elif topic == IMU_DEBUG_TOPIC:
                source = stamp_sec(message.source_stamp)
                measurement = stamp_sec(message.measurement_stamp)
                accel = stamp_sec(message.accel_effective_stamp)
                gyro = stamp_sec(message.gyro_effective_stamp)
                alpha = stamp_sec(message.alpha_effective_stamp)
                receive = stamp_sec(message.receive_stamp)
                state = stamp_sec(message.state_stamp)
                raw["imu_debug"].append(
                    {
                        "bag_time": bag_time,
                        "configured": bool(message.configured),
                        "valid": bool(message.valid),
                        "source_code": int(message.source),
                        "source": source,
                        "measurement": measurement,
                        "accel": accel,
                        "gyro": gyro,
                        "alpha": alpha,
                        "receive": receive,
                        "state": state,
                    }
                )
            elif topic == MOCAP_STATUS_TOPIC:
                raw["mocap_status"].append(str(message.data))
            elif topic == SCOUT_STATUS_TOPIC:
                raw["scout_status"].append(
                    {
                        "bag_time": bag_time,
                        "base_state": int(message.base_state),
                        "control_mode": int(message.control_mode),
                        "fault_code": int(message.fault_code),
                    }
                )
    return raw


def stream_stats(rows):
    array = core.unique_rows(rows)
    if array.shape[0] < 2:
        return {"count": int(array.shape[0])}
    deltas = np.diff(array[:, 0])
    positive = deltas[deltas > 0.0]
    duration = float(array[-1, 0] - array[0, 0])
    return {
        "count": int(array.shape[0]),
        "duration_sec": duration,
        "rate_hz": float((array.shape[0] - 1) / duration) if duration > 0.0 else 0.0,
        "median_period_sec": float(np.median(positive)) if positive.size else None,
        "p95_period_sec": float(np.percentile(positive, 95.0)) if positive.size else None,
        "maximum_period_sec": float(np.max(positive)) if positive.size else None,
        "nonpositive_delta_count": int(np.count_nonzero(deltas <= 0.0)),
    }


def nearest_command_audit(raw):
    command = core.unique_rows(raw["cmd"])
    if command.shape[0] < 2:
        return {"matched_count": 0}
    deltas_t = []
    deltas_v = []
    deltas_w = []
    for audit in raw["audit"]:
        if not audit["published"]:
            continue
        index = int(np.searchsorted(command[:, 0], audit["publish_time"]))
        candidates = [value for value in (index - 1, index) if 0 <= value < command.shape[0]]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda value: abs(command[value, 0] - audit["publish_time"]))
        time_delta = float(command[nearest, 0] - audit["publish_time"])
        if abs(time_delta) > 0.05:
            continue
        deltas_t.append(time_delta)
        deltas_v.append(float(command[nearest, 1] - audit["cmd_v"]))
        deltas_w.append(float(command[nearest, 2] - audit["cmd_omega"]))
    return {
        "matched_count": len(deltas_t),
        "cmd_bag_time_minus_audit_publish_sec": core.finite_summary(deltas_t),
        "cmd_topic_minus_audit_v": core.finite_summary(deltas_v),
        "cmd_topic_minus_audit_omega": core.finite_summary(deltas_w),
    }


def plan_replacement_audit(raw):
    rows = [
        item
        for item in raw["audit"]
        if item["published"]
        and item["solve_success"]
        and not item["terminal"]
        and not item["safety"]
        and item["shift_available"]
    ]
    a_flips = 0
    alpha_flips = 0
    for item in rows:
        current_a = item["previous_a"] + item["delta_a"]
        current_alpha = item["previous_alpha"] + item["delta_alpha"]
        if abs(item["previous_a"]) >= 0.01 and abs(current_a) >= 0.01:
            a_flips += int(item["previous_a"] * current_a < 0.0)
        if abs(item["previous_alpha"]) >= 0.01 and abs(current_alpha) >= 0.01:
            alpha_flips += int(item["previous_alpha"] * current_alpha < 0.0)
    denominator = max(1, len(rows))
    return {
        "count": len(rows),
        "replanned_minus_shifted_a": core.finite_summary(item["delta_a"] for item in rows),
        "replanned_minus_shifted_alpha": core.finite_summary(
            item["delta_alpha"] for item in rows
        ),
        "previous_shifted_a": core.finite_summary(item["previous_a"] for item in rows),
        "previous_shifted_alpha": core.finite_summary(
            item["previous_alpha"] for item in rows
        ),
        "a_sign_flip_fraction": float(a_flips / denominator),
        "alpha_sign_flip_fraction": float(alpha_flips / denominator),
    }


def processed_imu_audit(raw):
    rows = [
        item
        for item in raw["imu_debug"]
        if item["configured"] and item["source"] > 0.0
    ]

    def differences(left, right):
        return [1000.0 * (item[left] - item[right]) for item in rows if item[right] > 0.0]

    return {
        "configured_count": len(rows),
        "valid_count": sum(item["valid"] for item in rows),
        "source_codes": dict(Counter(item["source_code"] for item in rows)),
        "source_minus_measurement_ms": core.finite_summary(
            differences("source", "measurement")
        ),
        "source_minus_accel_effective_ms": core.finite_summary(
            differences("source", "accel")
        ),
        "source_minus_gyro_effective_ms": core.finite_summary(
            differences("source", "gyro")
        ),
        "source_minus_alpha_effective_ms": core.finite_summary(
            differences("source", "alpha")
        ),
        "state_minus_measurement_ms": core.finite_summary(
            [1000.0 * (item["state"] - item["measurement"]) for item in rows if item["state"] > 0.0 and item["measurement"] > 0.0]
        ),
        "receive_minus_source_ms": core.finite_summary(
            differences("receive", "source")
        ),
    }


def quality_report(path, raw):
    status_counts = Counter(raw["mocap_status"])
    non_ok = sum(count for value, count in status_counts.items() if not value.startswith("OK "))
    fault_counts = Counter(item["fault_code"] for item in raw["scout_status"])
    base_state_counts = Counter(item["base_state"] for item in raw["scout_status"])
    return {
        "bag": str(path),
        "duration_sec": raw["bag_end_sec"] - raw["bag_start_sec"],
        "streams": {
            "cmd_bag": stream_stats(raw["cmd"]),
            "odom_bag": stream_stats(raw["odom_bag"]),
            "odom_header": stream_stats(raw["odom_header"]),
            "imu_bag": stream_stats(raw["imu_bag"]),
            "imu_header": stream_stats(raw["imu_header"]),
            "mocap_bag": stream_stats(raw["mocap_bag"]),
            "mocap_header": stream_stats(raw["mocap_header"]),
        },
        "arrival_minus_header_sec": {
            key: core.finite_summary(values)
            for key, values in raw["arrival_minus_header"].items()
        },
        "mocap_status": {
            "count": len(raw["mocap_status"]),
            "non_ok_count": non_ok,
            "all_ok": non_ok == 0 and bool(raw["mocap_status"]),
        },
        "scout_status": {
            "count": len(raw["scout_status"]),
            "fault_code_counts": {str(key): value for key, value in sorted(fault_counts.items())},
            "base_state_counts": {str(key): value for key, value in sorted(base_state_counts.items())},
            "nonzero_fault_count": sum(
                count for value, count in fault_counts.items() if value != 0
            ),
        },
        "cmd_topic_vs_control_audit": nearest_command_audit(raw),
        "plan_replacement": plan_replacement_audit(raw),
        "processed_imu_timestamp": processed_imu_audit(raw),
    }


def aligned_signals(raw, time_source, args):
    pose_rows = raw["mocap_{}".format(time_source)]
    odom = core.unique_rows(raw["odom_{}".format(time_source)])
    imu = core.unique_rows(raw["imu_{}".format(time_source)])
    command = core.unique_rows(raw["cmd"])
    kinematics = mocap_tools.derive_mocap_kinematics(
        pose_rows,
        args.resample_hz,
        args.mocap_smooth_window_sec,
        args.tracker_to_base_yaw_rad,
        args.base_to_tracker_x_m,
        args.base_to_tracker_y_m,
    )
    start = max(kinematics["time"][0], command[0, 0], odom[0, 0], imu[0, 0])
    end = min(kinematics["time"][-1], command[-1, 0], odom[-1, 0], imu[-1, 0])
    mask = (kinematics["time"] >= start) & (kinematics["time"] <= end)
    time = kinematics["time"][mask]
    command_v = core.zoh_resample(command[:, 0], command[:, 1], time)
    command_w = core.zoh_resample(command[:, 0], command[:, 2], time)
    sensor_window = max(1, int(round(args.sensor_smooth_window_sec * args.resample_hz)))
    responses = {
        "linear_mocap": kinematics["v"][mask],
        "linear_odom": core.centered_moving_average(
            np.interp(time, odom[:, 0], odom[:, 1]), sensor_window
        ),
        "angular_mocap": kinematics["omega"][mask],
        "angular_odom": core.centered_moving_average(
            np.interp(time, odom[:, 0], odom[:, 2]), sensor_window
        ),
        "angular_imu": core.centered_moving_average(
            np.interp(time, imu[:, 0], imu[:, 1]), sensor_window
        ),
    }
    motion_start, motion_end = core.motion_window(
        time, command_v, command_w, args.motion_window_pad_sec
    )
    motion_mask = (time >= motion_start) & (time <= motion_end)
    return {
        "time": time[motion_mask] - time[motion_mask][0],
        "absolute_time": time[motion_mask],
        "command_v": command_v[motion_mask],
        "command_omega": command_w[motion_mask],
        "responses": {key: value[motion_mask] for key, value in responses.items()},
        "motion_start_sec": float(motion_start),
        "motion_end_sec": float(motion_end),
        "motion_duration_sec": float(motion_end - motion_start),
    }


def fit_aligned_signals(signals, args):
    report = {
        "motion_start_sec": signals["motion_start_sec"],
        "motion_end_sec": signals["motion_end_sec"],
        "motion_duration_sec": signals["motion_duration_sec"],
        "chains": {},
    }
    inputs = {}
    for name, response in signals["responses"].items():
        command = signals["command_v"] if name.startswith("linear_") else signals["command_omega"]
        model = core.fit_fopdt_grid(
            signals["time"],
            command,
            response,
            max_delay_sec=args.max_delay_sec,
            min_tau_sec=args.min_tau_sec,
            max_tau_sec=args.max_tau_sec,
            tau_count=args.tau_count,
            near_optimal_rmse_fraction=args.near_optimal_rmse_fraction,
        )
        split_delay = 0.15 if name.startswith("linear_") else 0.22
        report["chains"][name] = {
            "fopdt": model,
            "fixed_common_0p22": core.evaluate_fixed_delay(
                signals["time"], command, response, 0.22
            ),
            "fixed_split_0p15_0p22": core.evaluate_fixed_delay(
                signals["time"], command, response, split_delay
            ),
            "zero_delay": core.evaluate_fixed_delay(
                signals["time"], command, response, 0.0
            ),
        }
        inputs[name] = {
            "time": signals["time"],
            "command": command,
            "response": response,
        }
    report["nokov_relative_to_imu"] = core.estimate_relative_lag(
        signals["responses"]["angular_imu"],
        signals["responses"]["angular_mocap"],
        1.0 / args.resample_hz,
        args.relative_max_lag_sec,
    )
    return report, inputs


def filtered_zero_crossings(time, values, amplitude_threshold):
    crossings = []
    for direction in (-1, 1):
        for crossing in core.sustained_crossings(time, values, 0.0, direction):
            before = values[(time >= crossing - 0.75) & (time <= crossing - 0.20)]
            after = values[(time >= crossing + 0.20) & (time <= crossing + 0.75)]
            if before.size < 3 or after.size < 3:
                continue
            if (
                abs(float(np.median(before))) >= amplitude_threshold
                and abs(float(np.median(after))) >= amplitude_threshold
                and float(np.median(before)) * float(np.median(after)) < 0.0
            ):
                crossings.append((crossing, direction))
    return sorted(crossings)


def event_rows(run_label, signals):
    rows = []
    time = signals["time"]
    command_v = core.centered_moving_average(signals["command_v"], 5)
    command_w = core.centered_moving_average(signals["command_omega"], 5)
    max_v = float(np.percentile(command_v, 95.0))
    max_w = float(np.percentile(np.abs(command_w), 95.0))
    specifications = []
    for fraction in (0.1, 0.5, 0.9):
        specifications.append(("linear", fraction * max_v, 1, "level_{:.0f}pct_up".format(100 * fraction)))
        specifications.append(("linear", fraction * max_v, -1, "level_{:.0f}pct_down".format(100 * fraction)))
    for sign in (-1.0, 1.0):
        threshold = sign * 0.5 * max_w
        specifications.append(("angular", threshold, 1, "half_amplitude_up"))
        specifications.append(("angular", threshold, -1, "half_amplitude_down"))

    for channel, threshold, direction, event_type in specifications:
        command_values = command_v if channel == "linear" else command_w
        command_events = core.sustained_crossings(
            time, command_values, threshold, direction
        )
        response_names = (
            ("mocap", "linear_mocap"), ("odom", "linear_odom")
        ) if channel == "linear" else (
            ("mocap", "angular_mocap"),
            ("odom", "angular_odom"),
            ("imu", "angular_imu"),
        )
        for sensor, response_name in response_names:
            response_events = core.sustained_crossings(
                time,
                signals["responses"][response_name],
                threshold,
                direction,
            )
            for command_time, response_time, lag in core.match_crossing_events(
                command_events, response_events
            ):
                rows.append(
                    {
                        "bag": run_label,
                        "channel": channel,
                        "sensor": sensor,
                        "event_type": event_type,
                        "threshold": threshold,
                        "direction": direction,
                        "command_time_sec": command_time,
                        "response_time_sec": response_time,
                        "lag_sec": lag,
                        "valid": response_time is not None,
                        "method": "same-level crossing sustained for >=40 ms",
                    }
                )

    command_reversals = filtered_zero_crossings(time, command_w, 0.02)
    for sensor, response_name in (
        ("mocap", "angular_mocap"),
        ("odom", "angular_odom"),
        ("imu", "angular_imu"),
    ):
        response_reversals = filtered_zero_crossings(
            time, signals["responses"][response_name], 0.015
        )
        for direction in (-1, 1):
            command_times = [value for value, item_direction in command_reversals if item_direction == direction]
            response_times = [value for value, item_direction in response_reversals if item_direction == direction]
            for command_time, response_time, lag in core.match_crossing_events(
                command_times, response_times
            ):
                rows.append(
                    {
                        "bag": run_label,
                        "channel": "angular",
                        "sensor": sensor,
                        "event_type": "sign_reversal_zero_crossing",
                        "threshold": 0.0,
                        "direction": direction,
                        "command_time_sec": command_time,
                        "response_time_sec": response_time,
                        "lag_sec": lag,
                        "valid": response_time is not None,
                        "method": "zero crossing with opposite-sign plateaus and >=40 ms sustain",
                    }
                )
    return rows


def leave_one_bag_out(labels, per_bag_fit, inputs_by_bag):
    result = {"schema": "spmpc_same_bag_delay_leave_one_out_v1", "chains": {}}
    chain_names = sorted(inputs_by_bag[labels[0]])
    for chain in chain_names:
        evaluations = []
        for heldout in labels:
            training = [label for label in labels if label != heldout]
            frozen = core.frozen_median_model(
                [per_bag_fit[label]["chains"][chain]["fopdt"] for label in training]
            )
            values = inputs_by_bag[heldout][chain]
            fopdt = core.evaluate_fopdt(
                values["time"], values["command"], values["response"], frozen
            )
            split_delay = 0.15 if chain.startswith("linear_") else 0.22
            split = core.evaluate_fixed_delay(
                values["time"], values["command"], values["response"], split_delay
            )
            common = core.evaluate_fixed_delay(
                values["time"], values["command"], values["response"], 0.22
            )
            improvement = None
            if fopdt.get("valid") and split.get("valid") and split["rmse"] > 0.0:
                improvement = 1.0 - fopdt["rmse"] / split["rmse"]
            evaluations.append(
                {
                    "held_out_bag": heldout,
                    "training_bags": training,
                    "frozen_model": frozen,
                    "fopdt": fopdt,
                    "fixed_split_0p15_0p22": split,
                    "fixed_common_0p22": common,
                    "rmse_improvement_vs_split_fraction": improvement,
                }
            )
        result["chains"][chain] = {
            "evaluations": evaluations,
            "fopdt_r2": core.finite_summary(
                row["fopdt"].get("r2") for row in evaluations
            ),
            "rmse_improvement_vs_split_fraction": core.finite_summary(
                row["rmse_improvement_vs_split_fraction"] for row in evaluations
            ),
        }

    primary = ["linear_mocap", "angular_mocap"]
    promotable = True
    reasons = []
    for chain in primary:
        rows = result["chains"][chain]["evaluations"]
        if not all(
            row["fopdt"].get("valid")
            and row["fopdt"].get("r2", -math.inf) >= 0.80
            and row["rmse_improvement_vs_split_fraction"] is not None
            and row["rmse_improvement_vs_split_fraction"] >= 0.05
            for row in rows
        ):
            promotable = False
            reasons.append("{} did not improve every held-out bag by >=5% with R2>=0.80".format(chain))

        boundary_labels = []
        for label in labels:
            model = per_bag_fit[label]["chains"][chain]["fopdt"]
            if core.near_optimal_touches_grid_boundary(model):
                boundary_labels.append(label)
        if boundary_labels:
            promotable = False
            reasons.append(
                "{} has a <=1% RMSE near-optimal region touching the L/tau search boundary in {}"
                .format(chain, ", ".join(boundary_labels))
            )
    result["decision"] = (
        "PROMOTE_FOPDT_FOR_OFFLINE_REPLAY"
        if promotable
        else "INCONCLUSIVE_NEED_DEDICATED_IDENTIFICATION_BAG"
    )
    result["decision_scope"] = "offline replay candidate only; never an online parameter write"
    result["decision_reasons"] = reasons
    return result


def make_plot(path, labels, signals_by_bag):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    figure, axes = plt.subplots(len(labels), 2, figsize=(16, 4.2 * len(labels)), squeeze=False)
    for row_index, label in enumerate(labels):
        signals = signals_by_bag[label]
        time = signals["time"]
        axes[row_index, 0].plot(time, signals["command_v"], label="final /cmd_vel v", lw=1.5)
        axes[row_index, 0].plot(time, signals["responses"]["linear_mocap"], label="NOKOV v", lw=1.0)
        axes[row_index, 0].plot(time, signals["responses"]["linear_odom"], label="odom v", lw=1.0)
        axes[row_index, 0].set_ylabel("{}\nm/s".format(label))
        axes[row_index, 0].grid(True, alpha=0.25)
        axes[row_index, 0].legend(loc="best", ncol=3)
        axes[row_index, 1].plot(time, signals["command_omega"], label="final /cmd_vel omega", lw=1.5)
        axes[row_index, 1].plot(time, signals["responses"]["angular_mocap"], label="NOKOV omega", lw=1.0)
        axes[row_index, 1].plot(time, signals["responses"]["angular_odom"], label="odom omega", lw=1.0)
        axes[row_index, 1].plot(time, signals["responses"]["angular_imu"], label="IMU gyro z", lw=0.9, alpha=0.8)
        axes[row_index, 1].set_ylabel("{}\nrad/s".format(label))
        axes[row_index, 1].grid(True, alpha=0.25)
        axes[row_index, 1].legend(loc="best", ncol=2)
    axes[-1, 0].set_xlabel("time from motion window start (s)")
    axes[-1, 1].set_xlabel("time from motion window start (s)")
    axes[0, 0].set_title("Linear command and measured response")
    axes[0, 1].set_title("Angular command and measured response")
    figure.suptitle("Same-bag final command -> NOKOV / odom / IMU response")
    figure.tight_layout()
    figure.savefig(str(path), dpi=160)
    plt.close(figure)
    return True


def markdown_float(value, scale=1.0, digits=3):
    if value is None:
        return "NA"
    try:
        numeric = float(value) * scale
    except (TypeError, ValueError):
        return "NA"
    return "{:.{}f}".format(numeric, digits)


def write_decision_markdown(path, labels, reports, relative, loo):
    lines = [
        "# D02～D04 同包延迟辨识决策",
        "",
        "结论：`{}`。".format(loo["decision"]),
        "",
        "该结论只允许进入离线 replay，不会自动修改 `linear_delay_sec` 或 `angular_delay_sec`。",
        "",
        "## 自动判据说明",
        "",
    ]
    if loo.get("decision_reasons"):
        lines.extend(
            "- {}；".format(reason) for reason in loo["decision_reasons"]
        )
    else:
        lines.append("- 三轮留一包均通过 RMSE/R² 与参数可辨识性门槛；")
    lines.extend(
        [
        "- 该判据仍不替代平滑窗、拟合区间和外参敏感性复核。",
        "",
        "## 同包 FOPDT 拟合（bag 时间，运动窗口）",
        "",
        "| bag | chain | L (ms) | tau (ms) | K | R² | t90 (ms) |",
        "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for label in labels:
        for chain in ("linear_mocap", "linear_odom", "angular_mocap", "angular_odom", "angular_imu"):
            model = reports[label]["time_sources"]["bag_time"]["chains"][chain]["fopdt"]
            lines.append(
                "| `{}` | `{}` | {} | {} | {} | {} | {} |".format(
                    label,
                    chain,
                    markdown_float(model.get("delay_sec"), 1000.0, 0),
                    markdown_float(model.get("tau_sec"), 1000.0, 0),
                    markdown_float(model.get("gain"), 1.0, 3),
                    markdown_float(model.get("r2"), 1.0, 3),
                    markdown_float(model.get("response_time_sec", {}).get("t90"), 1000.0, 0),
                )
            )
    lines.extend(
        [
            "",
            "## NOKOV 相对 IMU 的同包结果",
            "",
            "| bag | lag (ms) | peak correlation | local interpretation |",
            "|---|---:|---:|---|",
        ]
    )
    for label in labels:
        row = relative[label]
        lines.append(
            "| `{}` | {} | {} | NOKOV 更早、IMU 更晚 |".format(
                label,
                markdown_float(row.get("lag_sec"), 1000.0, 0),
                markdown_float(row.get("peak_correlation_abs"), 1.0, 3),
            )
        )
    lines.extend(
        [
            "",
            "## 边界",
            "",
            "- 三包来自同一路径和同一控制模式，leave-one-bag-out 仍是 development 外推，不是独立 held-out；",
            "- `/cmd_vel` 是闭环生成的内生输入，FOPDT 是当前工况的描述模型，不等价于专用开环系统辨识；",
            "- Tracker 到 base 的 yaw 使用诊断值，杠杆臂仍为 0；",
            "- NOKOV 与 `/imu/data` 的相对时差不能分解出任一传感器的绝对延迟；",
            "- 在线参数继续保持 `0.15/0.22 s`，直到离线 replay 和新的专用辨识 bag 均支持替换。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--labels", nargs="*", default=[])
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--resample-hz", type=float, default=100.0)
    parser.add_argument("--mocap-smooth-window-sec", type=float, default=0.11)
    parser.add_argument("--sensor-smooth-window-sec", type=float, default=0.05)
    parser.add_argument("--motion-window-pad-sec", type=float, default=2.0)
    parser.add_argument("--tracker-to-base-yaw-rad", type=float, default=1.218)
    parser.add_argument("--base-to-tracker-x-m", type=float, default=0.0)
    parser.add_argument("--base-to-tracker-y-m", type=float, default=0.0)
    parser.add_argument("--max-delay-sec", type=float, default=0.60)
    parser.add_argument("--relative-max-lag-sec", type=float, default=0.30)
    parser.add_argument("--min-tau-sec", type=float, default=0.01)
    parser.add_argument("--max-tau-sec", type=float, default=1.50)
    parser.add_argument("--tau-count", type=int, default=64)
    parser.add_argument("--near-optimal-rmse-fraction", type=float, default=0.01)
    return parser.parse_args()


def main():
    args = parse_args()
    paths = [Path(value).expanduser().resolve() for value in args.bags]
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing bags: {}".format(missing))
    if args.labels and len(args.labels) != len(paths):
        raise ValueError("--labels must have exactly one value per bag")
    labels = args.labels or [path.stem for path in paths]
    if len(set(labels)) != len(labels):
        raise ValueError("bag labels must be unique")
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    protected_outputs = [
        output_dir / "per_bag_latency_fit.json",
        output_dir / "per_event_response.csv",
        output_dir / "nokov_imu_relative_lag.json",
        output_dir / "processed_imu_timestamp_audit.json",
        output_dir / "leave_one_bag_out_validation.json",
        output_dir / "cmd_vs_nokov_imu_odom.png",
        output_dir / "candidate_compensation_decision.md",
    ]
    existing = [str(path) for path in protected_outputs if path.exists()]
    if existing:
        raise FileExistsError("refusing to overwrite analysis artifacts: {}".format(existing))

    reports = {}
    inputs_by_bag = {}
    signals_by_bag = {}
    relative = {}
    quality = {}
    events = []
    for label, path in zip(labels, paths):
        print("[same-bag-delay] loading {}: {}".format(label, path), flush=True)
        raw = read_bag(path, args.mocap_tracker)
        quality[label] = quality_report(path, raw)
        reports[label] = {"bag": str(path), "time_sources": {}}
        for source_key, source_name in (("bag", "bag_time"), ("header", "header_stamp")):
            print("[same-bag-delay] fitting {} {}".format(label, source_name), flush=True)
            signals = aligned_signals(raw, source_key, args)
            fit, inputs = fit_aligned_signals(signals, args)
            reports[label]["time_sources"][source_name] = fit
            if source_key == "bag":
                signals_by_bag[label] = signals
                inputs_by_bag[label] = inputs
                relative[label] = fit["nokov_relative_to_imu"]
                events.extend(event_rows(label, signals))

    per_bag_payload = {
        "schema": "spmpc_same_bag_actuator_delay_v1",
        "processing": {
            "resample_hz": args.resample_hz,
            "mocap_smooth_window_sec": args.mocap_smooth_window_sec,
            "sensor_smooth_window_sec": args.sensor_smooth_window_sec,
            "motion_window_pad_sec": args.motion_window_pad_sec,
            "tracker_to_base_yaw_rad": args.tracker_to_base_yaw_rad,
            "base_to_tracker_x_m": args.base_to_tracker_x_m,
            "base_to_tracker_y_m": args.base_to_tracker_y_m,
            "max_delay_sec": args.max_delay_sec,
            "tau_range_sec": [args.min_tau_sec, args.max_tau_sec],
            "tau_count": args.tau_count,
            "command_source": "final /cmd_vel using rosbag arrival time and ZOH",
            "analyzer_sources": {
                str(Path(__file__).resolve()): file_sha256(Path(__file__).resolve()),
                str(Path(core.__file__).resolve()): file_sha256(Path(core.__file__).resolve()),
                str(Path(mocap_tools.__file__).resolve()): file_sha256(
                    Path(mocap_tools.__file__).resolve()
                ),
            },
        },
        "quality": quality,
        "bags": reports,
    }
    (output_dir / "per_bag_latency_fit.json").write_text(
        json.dumps(per_bag_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "nokov_imu_relative_lag.json").write_text(
        json.dumps(
            {
                "schema": "spmpc_same_bag_nokov_imu_relative_lag_v1",
                "sign_convention": "positive means NOKOV appears later than IMU",
                "bags": relative,
                "scope": "relative observation lag only; not either sensor's absolute latency",
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (output_dir / "processed_imu_timestamp_audit.json").write_text(
        json.dumps(
            {
                "schema": "spmpc_processed_imu_timestamp_audit_v1",
                "bags": {
                    label: quality[label]["processed_imu_timestamp"] for label in labels
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    event_fields = [
        "bag",
        "channel",
        "sensor",
        "event_type",
        "threshold",
        "direction",
        "command_time_sec",
        "response_time_sec",
        "lag_sec",
        "valid",
        "method",
    ]
    with (output_dir / "per_event_response.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=event_fields)
        writer.writeheader()
        writer.writerows(events)

    loo = leave_one_bag_out(labels, {
        label: reports[label]["time_sources"]["bag_time"] for label in labels
    }, inputs_by_bag)
    (output_dir / "leave_one_bag_out_validation.json").write_text(
        json.dumps(loo, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    make_plot(output_dir / "cmd_vs_nokov_imu_odom.png", labels, signals_by_bag)
    write_decision_markdown(
        output_dir / "candidate_compensation_decision.md",
        labels,
        reports,
        relative,
        loo,
    )
    print("[same-bag-delay] decision={}".format(loo["decision"]))
    print("[same-bag-delay] output={}".format(output_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
