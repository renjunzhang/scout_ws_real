#!/usr/bin/env python3
"""Analyze plan replacement, software command changes, and mocap response.

The analyzer uses ControlCycleAudit command timestamps and derives planar
velocity from the raw NOKOV pose.  It deliberately keeps Wi-Fi timestamp
uncertainty visible and does not treat /mocap/scout_odom.twist as truth.
"""

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data"


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def stamp_sec(stamp):
    try:
        return float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def message_stamp(message, bag_stamp):
    header = getattr(message, "header", None)
    value = stamp_sec(getattr(header, "stamp", None))
    return value if value > 0.0 else float(bag_stamp.to_sec())


def finite_summary(values):
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "median": float(np.median(array)),
        "p95_abs": float(np.percentile(np.abs(array), 95.0)),
        "max_abs": float(np.max(np.abs(array))),
    }


def correlation(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    mask = np.isfinite(first) & np.isfinite(second)
    first = first[mask]
    second = second[mask]
    if first.size < 3 or np.std(first) < 1e-12 or np.std(second) < 1e-12:
        return 0.0
    return float(np.corrcoef(first, second)[0, 1])


def rmse(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    mask = np.isfinite(first) & np.isfinite(second)
    if not np.any(mask):
        return float("nan")
    return float(np.sqrt(np.mean((first[mask] - second[mask]) ** 2)))


def moving_average(values, count):
    values = np.asarray(values, dtype=float)
    count = max(1, int(count))
    if count <= 1 or values.size < 3:
        return values.copy()
    if count % 2 == 0:
        count += 1
    count = min(count, values.size if values.size % 2 else values.size - 1)
    if count <= 1:
        return values.copy()
    half = count // 2
    padded = np.pad(values, (half, half), mode="edge")
    kernel = np.ones(count, dtype=float) / float(count)
    return np.convolve(padded, kernel, mode="valid")


def unique_rows(rows):
    """Sort rows by time and keep the last row for duplicate timestamps."""
    if not rows:
        return np.empty((0, 0), dtype=float)
    ordered = sorted(rows, key=lambda row: row[0])
    deduplicated = []
    for row in ordered:
        if deduplicated and abs(row[0] - deduplicated[-1][0]) <= 1e-12:
            deduplicated[-1] = row
        else:
            deduplicated.append(row)
    return np.asarray(deduplicated, dtype=float)


def zoh_resample(source_time, source_value, target_time):
    source_time = np.asarray(source_time, dtype=float)
    source_value = np.asarray(source_value, dtype=float)
    target_time = np.asarray(target_time, dtype=float)
    indices = np.searchsorted(source_time, target_time, side="right") - 1
    indices = np.clip(indices, 0, len(source_time) - 1)
    return source_value[indices]


def derive_mocap_kinematics(
    pose_rows,
    resample_hz,
    smooth_window_sec,
    tracker_to_base_yaw_rad=0.0,
    base_to_tracker_x_m=0.0,
    base_to_tracker_y_m=0.0,
):
    pose = unique_rows(pose_rows)
    if pose.shape[0] < 10:
        raise ValueError("mocap pose contains fewer than 10 unique samples")
    dt = 1.0 / float(resample_hz)
    time = np.arange(pose[0, 0], pose[-1, 0] + 0.25 * dt, dt)
    tracker_x = np.interp(time, pose[:, 0], pose[:, 1])
    tracker_y = np.interp(time, pose[:, 0], pose[:, 2])
    tracker_yaw = np.interp(time, pose[:, 0], np.unwrap(pose[:, 3]))
    base_yaw = tracker_yaw + float(tracker_to_base_yaw_rad)

    # r_BT is the base-origin -> tracker-origin vector expressed in base frame.
    cos_yaw = np.cos(base_yaw)
    sin_yaw = np.sin(base_yaw)
    offset_world_x = cos_yaw * base_to_tracker_x_m - sin_yaw * base_to_tracker_y_m
    offset_world_y = sin_yaw * base_to_tracker_x_m + cos_yaw * base_to_tracker_y_m
    base_x = tracker_x - offset_world_x
    base_y = tracker_y - offset_world_y

    window = max(1, int(round(float(smooth_window_sec) * resample_hz)))
    x_smooth = moving_average(base_x, window)
    y_smooth = moving_average(base_y, window)
    yaw_smooth = moving_average(base_yaw, window)
    vx_world = np.gradient(x_smooth, dt)
    vy_world = np.gradient(y_smooth, dt)
    omega = np.gradient(yaw_smooth, dt)
    longitudinal_v = np.cos(yaw_smooth) * vx_world + np.sin(yaw_smooth) * vy_world
    lateral_v = -np.sin(yaw_smooth) * vx_world + np.cos(yaw_smooth) * vy_world
    longitudinal_a = np.gradient(longitudinal_v, dt)
    angular_a = np.gradient(omega, dt)
    return {
        "time": time,
        "x": x_smooth,
        "y": y_smooth,
        "yaw": yaw_smooth,
        "v": longitudinal_v,
        "v_lateral": lateral_v,
        "omega": omega,
        "a": longitudinal_a,
        "alpha": angular_a,
        "dt": dt,
    }


def estimate_xcorr_delay(command, response, dt, max_lag_sec, derivative_smooth_samples=3):
    command = np.asarray(command, dtype=float)
    response = np.asarray(response, dtype=float)
    if command.size != response.size or command.size < 10:
        return {"delay_sec": None, "peak_correlation": 0.0, "lag_samples": 0}
    d_command = moving_average(np.gradient(command, dt), derivative_smooth_samples)
    d_response = moving_average(np.gradient(response, dt), derivative_smooth_samples)
    maximum = max(0, int(round(max_lag_sec / dt)))
    best = (0.0, 0)
    for lag in range(maximum + 1):
        if lag:
            first = d_command[:-lag]
            second = d_response[lag:]
        else:
            first = d_command
            second = d_response
        value = correlation(first, second)
        if value > best[0]:
            best = (value, lag)
    return {
        "delay_sec": float(best[1] * dt),
        "peak_correlation": float(best[0]),
        "lag_samples": int(best[1]),
        "command_derivative_std": float(np.std(d_command)),
        "response_derivative_std": float(np.std(d_response)),
    }


def simulate_first_order(time, command, delay_sec, tau_sec):
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    delayed = np.interp(
        time - delay_sec,
        time,
        command,
        left=command[0],
        right=command[-1],
    )
    filtered = np.empty_like(delayed)
    filtered[0] = delayed[0]
    for index in range(1, len(filtered)):
        dt = max(0.0, time[index] - time[index - 1])
        alpha = 1.0 - math.exp(-dt / max(1e-6, tau_sec))
        filtered[index] = filtered[index - 1] + alpha * (delayed[index] - filtered[index - 1])
    return filtered


def evaluate_first_order_model(time, command, response, model):
    if not model or not model.get("valid", False):
        return {"valid": False, "reason": "no frozen model"}
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    response = np.asarray(response, dtype=float)
    if time.size < 20 or command.size != time.size or response.size != time.size:
        return {"valid": False, "reason": "insufficient aligned samples"}
    predicted = float(model["offset"]) + float(model["gain"]) * simulate_first_order(
        time,
        command,
        float(model["delay_sec"]),
        float(model["tau_sec"]),
    )
    error = rmse(predicted, response)
    response_std = float(np.std(response))
    ss_res = float(np.sum((predicted - response) ** 2))
    ss_tot = float(np.sum((response - np.mean(response)) ** 2))
    return {
        "valid": True,
        "count": int(time.size),
        "rmse": error,
        "nrmse_by_std": error / max(response_std, 1e-12),
        "r2": 1.0 - ss_res / max(ss_tot, 1e-12),
    }


def fit_first_order_delay(
    time,
    command,
    response,
    max_lag_sec,
    min_tau_sec=0.01,
    max_tau_sec=0.80,
    tau_count=36,
):
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    response = np.asarray(response, dtype=float)
    if time.size < 20 or np.std(command) < 1e-6 or np.std(response) < 1e-6:
        return {"valid": False, "reason": "insufficient excitation"}
    dt = float(np.median(np.diff(time)))
    delays = np.arange(0.0, max_lag_sec + 0.5 * dt, dt)
    taus = np.geomspace(min_tau_sec, max_tau_sec, max(2, int(tau_count)))
    best = None
    for delay in delays:
        for tau in taus:
            basis = simulate_first_order(time, command, float(delay), float(tau))
            basis_centered = basis - np.mean(basis)
            denominator = float(np.dot(basis_centered, basis_centered))
            if denominator < 1e-12:
                continue
            gain = float(np.dot(basis_centered, response - np.mean(response)) / denominator)
            offset = float(np.mean(response) - gain * np.mean(basis))
            predicted = offset + gain * basis
            error = float(np.sqrt(np.mean((predicted - response) ** 2)))
            if best is None or error < best[0]:
                best = (error, float(delay), float(tau), gain, offset, predicted)
    if best is None:
        return {"valid": False, "reason": "fit grid produced no candidate"}
    response_std = float(np.std(response))
    ss_res = float(np.sum((best[5] - response) ** 2))
    ss_tot = float(np.sum((response - np.mean(response)) ** 2))
    return {
        "valid": True,
        "rmse": best[0],
        "nrmse_by_std": best[0] / max(response_std, 1e-12),
        "r2": 1.0 - ss_res / max(ss_tot, 1e-12),
        "delay_sec": best[1],
        "tau_sec": best[2],
        "gain": best[3],
        "offset": best[4],
    }


def sensor_comparison(reference_time, reference, sensor_rows, value_column):
    sensor = unique_rows(sensor_rows)
    if sensor.shape[0] < 3:
        return {"count": 0}
    start = max(reference_time[0], sensor[0, 0])
    end = min(reference_time[-1], sensor[-1, 0])
    mask = (reference_time >= start) & (reference_time <= end)
    if np.count_nonzero(mask) < 3:
        return {"count": 0}
    sensor_value = np.interp(reference_time[mask], sensor[:, 0], sensor[:, value_column])
    truth = reference[mask]
    return {
        "count": int(np.count_nonzero(mask)),
        "rmse": rmse(sensor_value, truth),
        "correlation": correlation(sensor_value, truth),
        "bias": float(np.mean(sensor_value - truth)),
    }


def load_bag(path, mocap_tracker, imu_topic):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag unavailable; source the ROS workspace: {}".format(exc))

    raw_mocap_topic = "/vrpn_client_node/{}/pose".format(mocap_tracker)
    audits = []
    pose_rows = []
    odom_rows = []
    imu_rows = []
    arrival_minus_header = []
    topics = [AUDIT_TOPIC, raw_mocap_topic, ODOM_TOPIC, imu_topic]
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            bag_time = bag_stamp.to_sec()
            if topic == AUDIT_TOPIC:
                publish_time = stamp_sec(message.command_publish_stamp)
                if publish_time <= 0.0:
                    publish_time = bag_time
                audits.append(
                    {
                        "time": publish_time,
                        "cycle_id": int(message.cycle_id),
                        "solve_success": bool(message.solve_success),
                        "command_published": bool(message.command_was_published),
                        "terminal_phase": bool(message.terminal_phase),
                        "terminal_intervened": bool(message.terminal_controller_intervened),
                        "safety_intervened": bool(message.safety_gate_intervened),
                        "solver_u0_a": float(message.solver_u0_a),
                        "solver_u0_alpha": float(message.solver_u0_alpha),
                        "solver_cmd_v": float(message.solver_cmd_v),
                        "solver_cmd_omega": float(message.solver_cmd_omega),
                        "terminal_cmd_v": float(message.terminal_cmd_v),
                        "terminal_cmd_omega": float(message.terminal_cmd_omega),
                        "post_gate_cmd_v": float(message.post_gate_cmd_v),
                        "post_gate_cmd_omega": float(message.post_gate_cmd_omega),
                        "published_cmd_v": float(message.published_cmd_v),
                        "published_cmd_omega": float(message.published_cmd_omega),
                        "shift_available": bool(message.previous_shifted_plan_available),
                        "replanned_minus_shifted_a": float(message.replanned_minus_shifted_a),
                        "replanned_minus_shifted_alpha": float(message.replanned_minus_shifted_alpha),
                    }
                )
            elif topic == raw_mocap_topic:
                header_time = message_stamp(message, bag_stamp)
                pose_rows.append(
                    (
                        header_time,
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        quaternion_yaw(message.pose.orientation),
                    )
                )
                arrival_minus_header.append(bag_time - header_time)
            elif topic == ODOM_TOPIC:
                odom_rows.append(
                    (
                        message_stamp(message, bag_stamp),
                        float(message.twist.twist.linear.x),
                        float(message.twist.twist.angular.z),
                    )
                )
            elif topic == imu_topic:
                imu_rows.append(
                    (
                        message_stamp(message, bag_stamp),
                        float(message.linear_acceleration.x),
                        float(message.linear_acceleration.y),
                        float(message.angular_velocity.z),
                    )
                )
    if not audits or not pose_rows:
        raise RuntimeError("{} lacks audit or raw mocap data".format(path))
    return {
        "audits": audits,
        "pose_rows": pose_rows,
        "odom_rows": odom_rows,
        "imu_rows": imu_rows,
        "arrival_minus_header": arrival_minus_header,
    }


def write_control_csv(path, audits):
    fields = list(audits[0].keys())
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audits)


def write_kinematics_csv(path, kinematics, command_v, command_omega):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time", "x", "y", "yaw", "v", "v_lateral", "omega", "a", "alpha", "cmd_v", "cmd_omega"]
        )
        for row in zip(
            kinematics["time"],
            kinematics["x"],
            kinematics["y"],
            kinematics["yaw"],
            kinematics["v"],
            kinematics["v_lateral"],
            kinematics["omega"],
            kinematics["a"],
            kinematics["alpha"],
            command_v,
            command_omega,
        ):
            writer.writerow(["{:.9f}".format(float(value)) for value in row])


def make_plot(path, name, kinematics, command_v, command_omega):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    relative_time = kinematics["time"] - kinematics["time"][0]
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(relative_time, command_v, label="published cmd v")
    axes[0].plot(relative_time, kinematics["v"], label="mocap longitudinal v")
    axes[0].set_ylabel("m/s")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(relative_time, command_omega, label="published cmd omega")
    axes[1].plot(relative_time, kinematics["omega"], label="mocap yaw rate")
    axes[1].set_xlabel("time from mocap start (s)")
    axes[1].set_ylabel("rad/s")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    figure.suptitle(name)
    figure.tight_layout()
    figure.savefig(str(path), dpi=150)
    plt.close(figure)
    return True


def analyze_bag(path, args, output_dir, role):
    raw = load_bag(path, args.mocap_tracker, args.imu_topic)
    kinematics = derive_mocap_kinematics(
        raw["pose_rows"],
        args.resample_hz,
        args.smooth_window_sec,
        args.tracker_to_base_yaw_rad,
        args.base_to_tracker_x_m,
        args.base_to_tracker_y_m,
    )
    command_samples = [item for item in raw["audits"] if item["command_published"]]
    if len(command_samples) < 10:
        raise RuntimeError("{} has fewer than 10 published command samples".format(path))
    command_rows = unique_rows(
        [
            (item["time"], item["published_cmd_v"], item["published_cmd_omega"])
            for item in command_samples
        ]
    )
    overlap = (kinematics["time"] >= command_rows[0, 0]) & (
        kinematics["time"] <= command_rows[-1, 0]
    )
    if np.count_nonzero(overlap) < 20:
        raise RuntimeError("{} has insufficient command/mocap time overlap".format(path))
    for key in ("time", "x", "y", "yaw", "v", "v_lateral", "omega", "a", "alpha"):
        kinematics[key] = kinematics[key][overlap]
    command_v = zoh_resample(command_rows[:, 0], command_rows[:, 1], kinematics["time"])
    command_omega = zoh_resample(command_rows[:, 0], command_rows[:, 2], kinematics["time"])

    linear_xcorr = estimate_xcorr_delay(
        command_v, kinematics["v"], kinematics["dt"], args.max_lag_sec
    )
    angular_xcorr = estimate_xcorr_delay(
        command_omega, kinematics["omega"], kinematics["dt"], args.max_lag_sec
    )
    linear_model = fit_first_order_delay(
        kinematics["time"], command_v, kinematics["v"], args.max_lag_sec
    )
    angular_model = fit_first_order_delay(
        kinematics["time"], command_omega, kinematics["omega"], args.max_lag_sec
    )

    successful = [item for item in command_samples if item["solve_success"]]
    nominal = [
        item
        for item in successful
        if not item["terminal_phase"] and not item["safety_intervened"]
    ]
    shifted = [item for item in nominal if item["shift_available"]]
    software = {
        "nominal_cycle_count": len(nominal),
        "shift_pair_count": len(shifted),
        "solver_to_published_delta_v": finite_summary(
            item["published_cmd_v"] - item["solver_cmd_v"] for item in nominal
        ),
        "solver_to_published_delta_omega": finite_summary(
            item["published_cmd_omega"] - item["solver_cmd_omega"] for item in nominal
        ),
        "replanned_minus_shifted_a": finite_summary(
            item["replanned_minus_shifted_a"] for item in shifted
        ),
        "replanned_minus_shifted_alpha": finite_summary(
            item["replanned_minus_shifted_alpha"] for item in shifted
        ),
        "terminal_intervention_count": sum(
            item["terminal_intervened"] for item in command_samples
        ),
        "safety_intervention_count": sum(
            item["safety_intervened"] for item in command_samples
        ),
    }
    sensors = {
        "odom_v_vs_mocap_v": sensor_comparison(
            kinematics["time"], kinematics["v"], raw["odom_rows"], 1
        ),
        "odom_omega_vs_mocap_omega": sensor_comparison(
            kinematics["time"], kinematics["omega"], raw["odom_rows"], 2
        ),
        "imu_omega_vs_mocap_omega": sensor_comparison(
            kinematics["time"], kinematics["omega"], raw["imu_rows"], 3
        ),
        "imu_ax_vs_mocap_longitudinal_a": sensor_comparison(
            kinematics["time"], kinematics["a"], raw["imu_rows"], 1
        ),
    }

    stem = path.stem
    control_csv = output_dir / (stem + "_control_chain.csv")
    kinematics_csv = output_dir / (stem + "_mocap_kinematics.csv")
    write_control_csv(control_csv, raw["audits"])
    write_kinematics_csv(kinematics_csv, kinematics, command_v, command_omega)
    plot_path = output_dir / (stem + "_published_vs_mocap.png")
    plot_written = make_plot(plot_path, stem, kinematics, command_v, command_omega) if args.plot else False

    report = {
        "bag": str(path),
        "role": role,
        "sample_counts": {
            "audit": len(raw["audits"]),
            "published_commands": len(command_samples),
            "published_successful_solve_commands": len(successful),
            "raw_mocap": len(raw["pose_rows"]),
            "derived_mocap": len(kinematics["time"]),
            "odom": len(raw["odom_rows"]),
            "imu": len(raw["imu_rows"]),
        },
        "processing": {
            "resample_hz": args.resample_hz,
            "smooth_window_sec": args.smooth_window_sec,
            "tracker_to_base_yaw_rad": args.tracker_to_base_yaw_rad,
            "base_to_tracker_x_m": args.base_to_tracker_x_m,
            "base_to_tracker_y_m": args.base_to_tracker_y_m,
            "imu_topic": args.imu_topic,
            "command_resampling": "zero_order_hold",
            "mocap_processing": "centered moving average plus centered numerical derivative; offline only",
        },
        "network_timing": {
            "raw_mocap_bag_time_minus_header_sec": finite_summary(raw["arrival_minus_header"]),
            "interpretation": "includes ROS receive/record scheduling and does not remove Wi-Fi transport uncertainty",
        },
        "software_chain": software,
        "execution_chain": {
            "linear": {"xcorr": linear_xcorr, "first_order_plus_delay": linear_model},
            "angular": {"xcorr": angular_xcorr, "first_order_plus_delay": angular_model},
        },
        "sensor_cross_checks": sensors,
        "artifacts": {
            "control_csv": str(control_csv),
            "mocap_kinematics_csv": str(kinematics_csv),
            "plot": str(plot_path) if plot_written else None,
        },
    }
    (output_dir / (stem + "_execution_chain_report.json")).write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    model_inputs = {
        "linear": {
            "time": kinematics["time"],
            "command": command_v,
            "response": kinematics["v"],
        },
        "angular": {
            "time": kinematics["time"],
            "command": command_omega,
            "response": kinematics["omega"],
        },
    }
    return report, model_inputs


def aggregate_reports(
    reports,
    model_inputs,
    training_count,
    stability_limit_sec,
    min_heldout_r2,
):
    training = reports[:training_count]
    validation = reports[training_count:]

    def model_values(group, channel, field):
        values = []
        for report in group:
            model = report["execution_chain"][channel]["first_order_plus_delay"]
            if model.get("valid") and math.isfinite(float(model.get(field, float("nan")))):
                values.append(float(model[field]))
        return values

    aggregate = {
        "training_bags": [report["bag"] for report in training],
        "held_out_bags": [report["bag"] for report in validation],
        "held_out_rule": (
            "Fit per-bag models only on training bags, freeze the median delay/tau/gain/offset, "
            "then apply those unchanged parameters to each held-out bag."
        ),
        "minimum_held_out_r2": min_heldout_r2,
        "channels": {},
    }
    decisions = []
    for channel in ("linear", "angular"):
        delays = model_values(training, channel, "delay_sec")
        taus = model_values(training, channel, "tau_sec")
        gains = model_values(training, channel, "gain")
        offsets = model_values(training, channel, "offset")
        heldout_refit_r2 = model_values(validation, channel, "r2")
        delay_summary = finite_summary(delays)
        all_training_valid = len(delays) == len(training) and len(training) >= 3
        stable = all_training_valid and float(
            delay_summary.get("std", float("inf"))
        ) <= stability_limit_sec
        gain_positive = len(gains) == len(training) and all(value > 0.0 for value in gains)
        frozen_model = {"valid": False}
        if all_training_valid and all(
            len(values) == len(training) for values in (taus, gains, offsets)
        ):
            frozen_model = {
                "valid": True,
                "construction": "median of training-bag fits",
                "delay_sec": statistics.median(delays),
                "tau_sec": statistics.median(taus),
                "gain": statistics.median(gains),
                "offset": statistics.median(offsets),
            }

        heldout_evaluations = []
        for index in range(training_count, len(reports)):
            inputs = model_inputs[index][channel]
            metrics = evaluate_first_order_model(
                inputs["time"],
                inputs["command"],
                inputs["response"],
                frozen_model,
            )
            heldout_evaluations.append({"bag": reports[index]["bag"], **metrics})
        heldout_r2 = [
            float(item["r2"])
            for item in heldout_evaluations
            if item.get("valid") and math.isfinite(float(item.get("r2", float("nan"))))
        ]
        heldout_pass = bool(validation) and len(heldout_r2) == len(validation) and min(
            heldout_r2
        ) >= min_heldout_r2
        aggregate["channels"][channel] = {
            "training_delay_sec": delay_summary,
            "training_tau_sec": finite_summary(taus),
            "training_gain": finite_summary(gains),
            "training_offset": finite_summary(offsets),
            "frozen_model": frozen_model,
            "held_out_frozen_model_evaluations": heldout_evaluations,
            "held_out_frozen_model_r2": finite_summary(heldout_r2),
            "held_out_refit_r2_diagnostic_only": finite_summary(heldout_refit_r2),
            "all_training_models_valid": all_training_valid,
            "delay_stable": stable,
            "gain_positive": gain_positive,
            "held_out_pass": heldout_pass,
        }
        decisions.append(stable and gain_positive and heldout_pass)
    aggregate["decision"] = (
        "CANDIDATE_FOR_FROZEN_ACTUATOR_MODEL"
        if all(decisions)
        else "INCONCLUSIVE_OR_REDESIGN"
    )
    return aggregate


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output-dir", default="/tmp/spmpc_mocap_execution_chain")
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--imu-topic", default=IMU_TOPIC)
    parser.add_argument("--resample-hz", type=float, default=100.0)
    parser.add_argument("--smooth-window-sec", type=float, default=0.11)
    parser.add_argument("--max-lag-sec", type=float, default=0.60)
    parser.add_argument("--tracker-to-base-yaw-rad", type=float, default=0.0)
    parser.add_argument("--base-to-tracker-x-m", type=float, default=0.0)
    parser.add_argument("--base-to-tracker-y-m", type=float, default=0.0)
    parser.add_argument("--training-count", type=int, default=3)
    parser.add_argument("--delay-stability-limit-sec", type=float, default=0.05)
    parser.add_argument("--min-heldout-r2", type=float, default=0.0)
    parser.add_argument("--plot", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.resample_hz <= 0.0 or args.smooth_window_sec <= 0.0 or args.max_lag_sec <= 0.0:
        raise ValueError("resample/smoothing/lag arguments must be positive")
    paths = [Path(item).expanduser().resolve() for item in args.bags]
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing bags: {}".format(missing))
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    training_count = max(1, min(int(args.training_count), len(paths)))
    reports = []
    model_inputs = []
    for index, path in enumerate(paths):
        role = "training" if index < training_count else "held_out_validation"
        print("[analyze_mocap_execution_chain] {} ({})".format(path, role))
        report, inputs = analyze_bag(path, args, output_dir, role)
        reports.append(report)
        model_inputs.append(inputs)
    aggregate = aggregate_reports(
        reports,
        model_inputs,
        training_count,
        args.delay_stability_limit_sec,
        args.min_heldout_r2,
    )
    heldout_output_path = output_dir / "heldout_validation_report.json"
    heldout_output_path.write_text(
        json.dumps(
            {
                "schema": "spmpc_mocap_execution_chain_heldout_validation_v1",
                "aggregate": aggregate,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    final = {
        "schema": "spmpc_mocap_execution_chain_analysis_v1",
        "bags": reports,
        "aggregate": aggregate,
        "artifacts": {"heldout_validation_report": str(heldout_output_path)},
        "scope": (
            "Development execution-chain evidence only. It does not establish RGB slosh benefit "
            "and Wi-Fi timestamp uncertainty remains explicit."
        ),
    }
    output_path = output_dir / "actuator_model_fit.json"
    output_path.write_text(
        json.dumps(final, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(aggregate, ensure_ascii=False, indent=2))
    print("[analyze_mocap_execution_chain] report={}".format(output_path))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("[analyze_mocap_execution_chain][ERR] {}".format(exc), file=sys.stderr)
        sys.exit(2)
