#!/usr/bin/env python3
"""Measure velocity-step delay, acceleration, braking, and stopping performance."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

import same_bag_delay_core as delay_core
import velocity_step_response_core as core


CMD_TOPIC = "/cmd_vel"
STAMPED_CMD_TOPIC = "/mocap_velocity_step/cmd_vel_stamped"
ODOM_TOPIC = "/odom"


def quaternion_yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def header_time(message, fallback):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return float(fallback)
    value = float(stamp.to_sec())
    return value if value > 0.0 else float(fallback)


def stream_rate(rows):
    if len(rows) < 2:
        return None
    times = np.asarray([row[0] for row in rows], dtype=float)
    times = np.unique(times[np.isfinite(times)])
    if times.size < 2:
        return None
    duration = float(times[-1] - times[0])
    return float((times.size - 1) / duration) if duration > 0.0 else None


def load_bag(
    path,
    tracker,
    imu_topic,
    stamped_command_topic,
    cmd_topic=CMD_TOPIC,
    odom_topic=ODOM_TOPIC,
):
    import rosbag

    mocap_topic = "/vrpn_client_node/{}/pose".format(tracker)
    data = {
        "command_bag": [],
        "command_header": [],
        "mocap_bag": [],
        "mocap_header": [],
        "imu_bag": [],
        "imu_header": [],
        "odom_bag": [],
        "odom_header": [],
    }
    topics = [cmd_topic, stamped_command_topic, mocap_topic, imu_topic, odom_topic]
    with rosbag.Bag(str(path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        required = {cmd_topic, stamped_command_topic, mocap_topic, imu_topic, odom_topic}
        missing = sorted(required - available)
        if missing:
            raise RuntimeError("bag is missing required topics: {}".format(", ".join(missing)))
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            bag_time = float(bag_stamp.to_sec())
            if topic == cmd_topic:
                data["command_bag"].append(
                    (bag_time, float(message.linear.x), float(message.angular.z))
                )
            elif topic == stamped_command_topic:
                data["command_header"].append(
                    (
                        header_time(message, bag_time),
                        float(message.twist.linear.x),
                        float(message.twist.angular.z),
                    )
                )
            elif topic == mocap_topic:
                row = (
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    quaternion_yaw(message.pose.orientation),
                )
                data["mocap_bag"].append((bag_time, *row))
                data["mocap_header"].append((header_time(message, bag_time), *row))
            elif topic == imu_topic:
                row = (
                    float(message.angular_velocity.z),
                    float(message.linear_acceleration.x),
                    float(message.linear_acceleration.y),
                    float(message.linear_acceleration.z),
                )
                data["imu_bag"].append((bag_time, *row))
                data["imu_header"].append((header_time(message, bag_time), *row))
            elif topic == odom_topic:
                row = (
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.angular.z),
                )
                data["odom_bag"].append((bag_time, *row))
                data["odom_header"].append((header_time(message, bag_time), *row))
    return data, mocap_topic


def unique_matrix(rows):
    if not rows:
        return np.empty((0, 0), dtype=float)
    array = np.asarray(rows, dtype=float)
    array = array[np.all(np.isfinite(array), axis=1)]
    array = array[np.argsort(array[:, 0], kind="mergesort")]
    output = []
    for row in array:
        if output and row[0] <= output[-1][0] + 1e-12:
            output[-1] = row
        else:
            output.append(row)
    return np.asarray(output, dtype=float)


def select_command(rows, axis):
    array = unique_matrix(rows)
    if array.shape[0] < 2 or array.shape[1] < 3:
        raise ValueError("too few command samples")
    column = 1 if axis == "linear" else 2
    return array[:, 0], array[:, column]


def select_odom(rows, axis, median_window):
    array = unique_matrix(rows)
    if array.shape[0] < 2 or array.shape[1] < 3:
        raise ValueError("too few odometry samples")
    column = 1 if axis == "linear" else 2
    return array[:, 0], core.centered_median(array[:, column], median_window)


def select_linear_imu_acceleration(rows, forward_axis, forward_sign):
    array = unique_matrix(rows)
    if array.shape[0] < 20 or array.shape[1] < 5:
        raise ValueError("too few full IMU samples for linear acceleration")
    columns = {"x": 2, "y": 3, "z": 4}
    column = columns[forward_axis]
    value = float(forward_sign) * array[:, column]
    # Preserve raw sample timing for onset detection.  A separately filtered
    # copy is made below for display only, so zero-phase smoothing cannot make
    # the reported IMU onset appear early.
    return array[:, 0], value


def attach_fopdt_fit(time, value, command_time, command_value, command_step, result, args):
    if not result.get("valid", False):
        result["fopdt"] = {"valid": False, "reason": "velocity response is invalid"}
        return
    start = float(command_step["start_time_sec"])
    stop = float(command_step["stop_time_sec"])
    mask = (
        (time >= start - float(args.baseline_sec))
        & (time <= stop + float(args.fopdt_post_sec))
    )
    fit_time = np.asarray(time, dtype=float)[mask]
    fit_value = np.asarray(value, dtype=float)[mask]
    if fit_time.size < 50:
        result["fopdt"] = {"valid": False, "reason": "insufficient FOPDT samples"}
        return
    fit_dt = float(np.median(np.diff(fit_time)))
    if not math.isfinite(fit_dt) or fit_dt <= 0.0:
        result["fopdt"] = {"valid": False, "reason": "invalid FOPDT time grid"}
        return
    uniform_time = np.arange(fit_time[0], fit_time[-1] + 0.25 * fit_dt, fit_dt)
    uniform_value = np.interp(uniform_time, fit_time, fit_value)
    fit_command = delay_core.zoh_resample(
        command_time, command_value, uniform_time
    )
    fit = delay_core.fit_fopdt_grid(
        uniform_time,
        fit_command,
        uniform_value,
        max_delay_sec=args.fopdt_max_delay_sec,
        min_tau_sec=args.fopdt_min_tau_sec,
        max_tau_sec=args.fopdt_max_tau_sec,
        tau_count=args.fopdt_tau_count,
        near_optimal_rmse_fraction=args.fopdt_near_optimal_fraction,
    )
    fit["fit_window"] = {
        "definition": (
            "command_on_rise_only"
            if float(args.fopdt_post_sec) == 0.0
            else "command_on_plus_explicit_post_window"
        ),
        "start_time_sec": float(uniform_time[0]),
        "stop_time_sec": float(uniform_time[-1]),
        "start_offset_sec": float(uniform_time[0] - start),
        "stop_offset_sec": float(uniform_time[-1] - start),
        "post_command_sec": float(args.fopdt_post_sec),
    }
    quality_reasons = []
    if fit.get("valid", False):
        if delay_core.near_optimal_touches_grid_boundary(fit):
            quality_reasons.append("near-optimal parameters touch the fit-grid boundary")
        if float(fit.get("r2", float("-inf"))) < float(args.fopdt_min_r2):
            quality_reasons.append(
                "R2 below {:.3f}".format(float(args.fopdt_min_r2))
            )
    else:
        quality_reasons.append(fit.get("reason", "fit invalid"))
    fit["identifiable"] = bool(fit.get("valid", False) and not quality_reasons)
    fit["quality_reasons"] = quality_reasons
    result["fopdt"] = fit


def attach_acceleration_curve(time, value, command_step, result, args):
    acceleration_time, acceleration = core.smooth_derivative(
        time,
        value,
        window_sec=args.acceleration_window_sec,
        median_window=args.acceleration_median_window,
    )
    result["acceleration_capability"] = core.acceleration_capability(
        acceleration_time, acceleration, command_step, result
    )
    return acceleration_time, acceleration


def derive_linear_mocap(rows, command_step, median_window, velocity_window_sec):
    pose = unique_matrix(rows)
    if pose.shape[0] < 20 or pose.shape[1] < 4:
        raise ValueError("too few NOKOV pose samples for linear velocity")
    time = pose[:, 0]
    x = pose[:, 1]
    y = pose[:, 2]
    start = float(command_step["start_time_sec"])
    stop = float(command_step["stop_time_sec"])
    before = (time >= start - 0.60) & (time <= start - 0.10)
    near_end = (time >= stop - 0.60) & (time <= stop - 0.10)
    if np.count_nonzero(before) < 5 or np.count_nonzero(near_end) < 5:
        raise ValueError("insufficient NOKOV positions around the linear step")
    p0 = np.asarray([np.median(x[before]), np.median(y[before])], dtype=float)
    p1 = np.asarray([np.median(x[near_end]), np.median(y[near_end])], dtype=float)
    displacement = p1 - p0
    displacement_norm = float(np.linalg.norm(displacement))
    if displacement_norm < 0.05:
        raise ValueError("NOKOV displacement is below 0.05 m; linear excitation is insufficient")
    command_sign = 1.0 if command_step["command_value"] > 0.0 else -1.0
    body_positive_axis = command_sign * displacement / displacement_norm
    lateral_axis = np.asarray([-body_positive_axis[1], body_positive_axis[0]])
    relative_position = np.column_stack((x, y)) - p0[None, :]
    along_position = relative_position @ body_positive_axis
    lateral_position = relative_position @ lateral_axis
    velocity_time, longitudinal = core.smooth_derivative(
        time,
        along_position,
        window_sec=velocity_window_sec,
        median_window=median_window,
    )
    return {
        "time": velocity_time,
        "velocity": longitudinal,
        "position_time": time,
        "along_position": along_position,
        "lateral_position": lateral_position,
        "body_positive_axis_xy": body_positive_axis,
        "step_displacement_m": displacement_norm,
    }


def derive_angular_mocap(rows, median_window, velocity_window_sec):
    pose = unique_matrix(rows)
    if pose.shape[0] < 5 or pose.shape[1] < 4:
        raise ValueError("too few NOKOV pose samples for angular velocity")
    yaw = np.unwrap(pose[:, 3])
    rate_time, omega = core.smooth_derivative(
        pose[:, 0],
        yaw,
        window_sec=velocity_window_sec,
        median_window=median_window,
    )
    return {
        "time": rate_time,
        "velocity": omega,
        "yaw_time": pose[:, 0],
        "yaw": yaw,
    }


def response_settings(axis, sensor, args):
    if axis == "linear":
        floor = args.linear_mocap_onset_floor if sensor == "mocap" else args.linear_odom_onset_floor
    elif sensor == "imu":
        floor = args.angular_imu_onset_floor
    elif sensor == "odom":
        floor = args.angular_odom_onset_floor
    else:
        floor = args.angular_mocap_onset_floor
    return {
        "baseline_sec": args.baseline_sec,
        "steady_sec": args.steady_sec,
        "guard_sec": args.guard_sec,
        "onset_floor": floor,
        "onset_noise_multiplier": args.onset_noise_multiplier,
        "onset_fraction": args.onset_fraction,
        "sustain_sec": args.sustain_sec,
    }


def attach_mocap_stopping_metrics(axis, kinematics, command_step, result):
    if not result.get("braking_valid", False):
        return
    direction = 1.0 if command_step["command_value"] > 0.0 else -1.0
    command_stop = float(command_step["stop_time_sec"])
    actual_stop = float(result["response_stop_time_sec"])
    if axis == "linear":
        position_time = kinematics["position_time"]
        along = kinematics["along_position"]
        lateral = kinematics["lateral_position"]
        position_at_command_stop = float(np.interp(command_stop, position_time, along))
        position_at_actual_stop = float(np.interp(actual_stop, position_time, along))
        result["stopping_distance_after_command_m"] = direction * (
            position_at_actual_stop - position_at_command_stop
        )
        motion_window = (position_time >= command_step["start_time_sec"]) & (
            position_time <= actual_stop
        )
        if np.any(motion_window):
            baseline_lateral = float(
                np.interp(command_step["start_time_sec"], position_time, lateral)
            )
            result["maximum_lateral_drift_m"] = float(
                np.max(np.abs(lateral[motion_window] - baseline_lateral))
            )
    else:
        yaw_at_command_stop = float(
            np.interp(command_stop, kinematics["yaw_time"], kinematics["yaw"])
        )
        yaw_at_actual_stop = float(
            np.interp(actual_stop, kinematics["yaw_time"], kinematics["yaw"])
        )
        result["rotation_after_command_deg"] = math.degrees(
            direction * (yaw_at_actual_stop - yaw_at_command_stop)
        )


def analyze_timebase(command_rows, mocap_rows, imu_rows, odom_rows, axis, args):
    command_time, command_value = select_command(command_rows, axis)
    minimum_command = args.minimum_command
    if minimum_command is None:
        minimum_command = 0.01 if axis == "linear" else 0.02
    command_step = core.find_command_step(
        command_time,
        command_value,
        minimum_command=minimum_command,
        minimum_duration=args.minimum_step_duration,
    )

    sensors = {}
    acceleration_crosscheck = {}
    signals = {
        "command": {"time": command_time, "value": command_value, "label": "final /cmd_vel"}
    }
    if axis == "linear":
        mocap = derive_linear_mocap(
            mocap_rows,
            command_step,
            args.mocap_median_window,
            args.mocap_velocity_window_sec,
        )
        odom_time, odom_value = select_odom(odom_rows, axis, args.odom_median_window)
        imu_accel_time, imu_accel_value = select_linear_imu_acceleration(
            imu_rows,
            args.imu_forward_axis,
            args.imu_forward_sign,
        )
        sensor_inputs = {
            "mocap": (mocap["time"], mocap["velocity"], "NOKOV longitudinal velocity"),
            "odom": (odom_time, odom_value, "odom linear velocity"),
        }
        imu_accel_display = core.centered_mean(
            core.centered_median(
                imu_accel_value, args.imu_acceleration_median_window
            ),
            max(
                1,
                int(
                    round(
                        args.imu_acceleration_window_sec
                        / max(float(np.median(np.diff(imu_accel_time))), 1e-6)
                    )
                ),
            ),
        )
        acceleration_crosscheck["imu"] = core.analyze_acceleration_onset(
            imu_accel_time,
            imu_accel_value,
            command_step,
            baseline_sec=args.baseline_sec,
            guard_sec=args.guard_sec,
            onset_floor=args.linear_imu_accel_onset_floor,
            onset_noise_multiplier=args.onset_noise_multiplier,
            sustain_sec=args.acceleration_onset_sustain_sec,
        )
        signals["imu_acceleration"] = {
            "time": imu_accel_time,
            "value": imu_accel_display,
            "label": "IMU body-forward acceleration",
        }
    else:
        mocap = derive_angular_mocap(
            mocap_rows,
            args.mocap_median_window,
            args.mocap_velocity_window_sec,
        )
        imu = unique_matrix(imu_rows)
        if imu.shape[0] < 2:
            raise ValueError("too few IMU samples")
        imu_value = core.centered_median(imu[:, 1], args.imu_median_window)
        odom_time, odom_value = select_odom(odom_rows, axis, args.odom_median_window)
        sensor_inputs = {
            "mocap": (mocap["time"], mocap["velocity"], "NOKOV yaw rate"),
            "imu": (imu[:, 0], imu_value, "IMU gyro-z"),
            "odom": (odom_time, odom_value, "odom angular velocity"),
        }

    for sensor, (time, value, label) in sensor_inputs.items():
        result = core.analyze_step_response(
            time,
            value,
            command_step,
            **response_settings(axis, sensor, args)
        )
        attach_fopdt_fit(
            np.asarray(time, dtype=float),
            np.asarray(value, dtype=float),
            command_time,
            command_value,
            command_step,
            result,
            args,
        )
        acceleration_time, acceleration = attach_acceleration_curve(
            time, value, command_step, result, args
        )
        sensors[sensor] = result
        signals[sensor] = {
            "time": time,
            "value": value,
            "label": label,
            "acceleration_time": acceleration_time,
            "acceleration": acceleration,
        }
    attach_mocap_stopping_metrics(axis, mocap, command_step, sensors["mocap"])
    return {
        "axis": axis,
        "command": command_step,
        "sensors": sensors,
        "acceleration_crosscheck": acceleration_crosscheck,
        "acceleration_window_sec": float(args.acceleration_window_sec),
        "signals": signals,
    }


def serializable_timebase(result):
    return {
        "axis": result["axis"],
        "command": result["command"],
        "sensors": result["sensors"],
        "acceleration_crosscheck": result["acceleration_crosscheck"],
        "acceleration_window_sec": result["acceleration_window_sec"],
    }


def milliseconds(value):
    return "n/a" if value is None else "{:.1f}".format(1000.0 * float(value))


def delay_rows(result):
    rows = {}
    for sensor, values in result["sensors"].items():
        fit = values.get("fopdt", {})
        identifiable = bool(fit.get("valid", False) and fit.get("identifiable", False))
        rows[sensor] = {
            "signal": "velocity",
            "onset_delay_sec": values.get("onset_delay_sec"),
            "fopdt_identifiable": identifiable,
            "fopdt_quality_reasons": fit.get("quality_reasons", []),
            "fopdt_delay_sec": fit.get("delay_sec") if identifiable else None,
            "fopdt_tau_sec": fit.get("tau_sec") if identifiable else None,
            "fopdt_gain": fit.get("gain") if identifiable else None,
            "fopdt_diagnostic_delay_sec": (
                fit.get("delay_sec") if fit.get("valid", False) else None
            ),
        }
    for sensor, values in result["acceleration_crosscheck"].items():
        rows["{}_acceleration".format(sensor)] = {
            "signal": "acceleration_onset_only",
            "onset_delay_sec": values.get("onset_delay_sec"),
            "fopdt_identifiable": False,
            "fopdt_quality_reasons": ["acceleration onset is not a velocity FOPDT fit"],
            "fopdt_delay_sec": None,
            "fopdt_tau_sec": None,
            "fopdt_gain": None,
            "fopdt_diagnostic_delay_sec": None,
        }
    return rows


def pairwise_delay_differences(rows, field):
    names = [name for name, values in sorted(rows.items()) if values.get(field) is not None]
    output = {}
    for left_index, left in enumerate(names):
        for right in names[left_index + 1:]:
            output["{}_minus_{}_ms".format(left, right)] = 1000.0 * (
                float(rows[left][field]) - float(rows[right][field])
            )
    return output


def build_delay_cross_validation(primary, secondary):
    bag_rows = delay_rows(primary)
    header_rows = delay_rows(secondary)
    timebase_delta = {}
    for sensor in sorted(set(bag_rows) & set(header_rows)):
        timebase_delta[sensor] = {}
        for field in ("onset_delay_sec", "fopdt_delay_sec"):
            bag_value = bag_rows[sensor].get(field)
            header_value = header_rows[sensor].get(field)
            timebase_delta[sensor]["bag_minus_header_{}_ms".format(field)] = (
                None
                if bag_value is None or header_value is None
                else 1000.0 * (float(bag_value) - float(header_value))
            )
    return {
        "interpretation": (
            "Cross-sensor differences include each sensor's own transport/timestamp delay; "
            "linear IMU uses acceleration onset and is supporting evidence, not an FOPDT velocity output."
        ),
        "bag_time": {
            "sensors": bag_rows,
            "onset_pairwise": pairwise_delay_differences(
                bag_rows, "onset_delay_sec"
            ),
            "fopdt_pairwise": pairwise_delay_differences(
                bag_rows, "fopdt_delay_sec"
            ),
        },
        "header_time": {
            "sensors": header_rows,
            "onset_pairwise": pairwise_delay_differences(
                header_rows, "onset_delay_sec"
            ),
            "fopdt_pairwise": pairwise_delay_differences(
                header_rows, "fopdt_delay_sec"
            ),
        },
        "bag_minus_header": timebase_delta,
    }


def print_sensor(name, result, axis):
    velocity_unit = "m/s" if axis == "linear" else "rad/s"
    acceleration_unit = "m/s^2" if axis == "linear" else "rad/s^2"
    if not result.get("valid", False):
        print("  {:<6} INVALID: {}".format(name, result.get("reason", "unknown")))
        return
    print(
        "  {:<6} onset={} ms  t50={} ms  t90={} ms  a_eff={:.4f} {}".format(
            name,
            milliseconds(result.get("onset_delay_sec")),
            milliseconds(result.get("t50_sec")),
            milliseconds(result.get("t90_sec")),
            result["effective_acceleration"],
            acceleration_unit,
        )
    )
    print(
        "         steady={:.4f} {}  threshold={:.4f} {}  noise_sigma={:.4f} {}".format(
            result["steady_value"],
            velocity_unit,
            result["onset_threshold"],
            velocity_unit,
            result["noise_sigma"],
            velocity_unit,
        )
    )
    fit = result.get("fopdt", {})
    if fit.get("valid", False):
        print(
            "         FOPDT-rise L={} ms  tau={} ms  K={:.4f}  R2={:.4f}  identifiable={}".format(
                milliseconds(fit["delay_sec"]),
                milliseconds(fit["tau_sec"]),
                fit["gain"],
                fit["r2"],
                "YES" if fit.get("identifiable", False) else "NO",
            )
        )
        if not fit.get("identifiable", False):
            print(
                "         FOPDT diagnostic only: {}".format(
                    "; ".join(fit.get("quality_reasons", ["quality gate failed"]))
                )
            )
    else:
        print("         FOPDT INVALID: {}".format(fit.get("reason", "unknown")))
    capability = result.get("acceleration_capability", {})
    if capability.get("valid", False):
        capability_text = "         curve a_p95={:.4f} {}".format(
            capability["acceleration_p95"], acceleration_unit
        )
        if capability.get("braking_valid", False):
            capability_text += "  decel_p95={:.4f} {}".format(
                capability["deceleration_p95"], acceleration_unit
            )
        print(capability_text)
    if result.get("braking_valid", False):
        print(
            "         brake_onset={} ms  stop={} ms  decel_eff={:.4f} {}".format(
                milliseconds(result["brake_onset_delay_sec"]),
                milliseconds(result["response_stop_delay_sec"]),
                result["effective_deceleration"],
                acceleration_unit,
            )
        )
    else:
        print("         braking INVALID: {}".format(result.get("braking_reason", "unknown")))
    if "stopping_distance_after_command_m" in result:
        print(
            "         stopping_distance={:.4f} m  max_lateral_drift={:.4f} m".format(
                result["stopping_distance_after_command_m"],
                result["maximum_lateral_drift_m"],
            )
        )
    if "rotation_after_command_deg" in result:
        print(
            "         rotation_after_command={:.2f} deg".format(
                result["rotation_after_command_deg"]
            )
        )


def print_report(bag_path, primary, secondary):
    axis = primary["axis"]
    command = primary["command"]
    unit = "m/s" if axis == "linear" else "rad/s"
    print("== Stationary {} velocity-step response ==".format(axis))
    print("bag       : {}".format(bag_path))
    print(
        "command   : {:+.3f} {} for {:.3f} s".format(
            command["command_value"], unit, command["duration_sec"]
        )
    )
    print("primary timebase: rosbag arrival time")
    for sensor, result in primary["sensors"].items():
        print_sensor(sensor.upper(), result, axis)
    print("header-stamp cross-check:")
    for sensor, result in secondary["sensors"].items():
        print_sensor(sensor.upper(), result, axis)
    for sensor, result in primary["acceleration_crosscheck"].items():
        if result.get("valid", False):
            print(
                "{} acceleration onset cross-check={} ms".format(
                    sensor.upper(), milliseconds(result["onset_delay_sec"])
                )
            )
        else:
            print(
                "{} acceleration onset cross-check INVALID: {}".format(
                    sensor.upper(), result.get("reason", "unknown")
                )
            )
    print("Note: effective acceleration uses the measured 10%-90% velocity slope.")


PLOT_COLORS = {
    "command": "#555555",
    "mocap": "#0072B2",
    "imu": "#E69F00",
    "odom": "#009E73",
}


def prepare_plotting():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def add_step_boundaries(axis, result):
    start = float(result["command"]["start_time_sec"])
    stop = float(result["command"]["stop_time_sec"])
    axis.axvline(0.0, color="#555555", linestyle="--", linewidth=0.9)
    axis.axvline(stop - start, color="#888888", linestyle="--", linewidth=0.9)
    axis.set_xlim(-1.0, stop - start + 3.0)
    axis.grid(True, alpha=0.25)


def make_velocity_plot(path, result, sensor):
    plt = prepare_plotting()
    axis_name = result["axis"]
    start = float(result["command"]["start_time_sec"])
    signal = result["signals"][sensor]
    command = result["signals"]["command"]
    sensor_result = result["sensors"][sensor]
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        command["time"] - start,
        command["value"],
        color=PLOT_COLORS["command"],
        linewidth=1.1,
        linestyle="--",
        label="command",
    )
    axis.plot(
        signal["time"] - start,
        signal["value"],
        color=PLOT_COLORS[sensor],
        linewidth=1.5,
        label=signal["label"],
    )
    if sensor_result.get("valid", False):
        direction = 1.0 if result["command"]["command_value"] > 0.0 else -1.0
        axis.axvline(
            sensor_result["onset_delay_sec"],
            color="#CC3311",
            linestyle=":",
            linewidth=1.1,
            label="detected onset",
        )
        axis.scatter(
            [sensor_result["t10_sec"], sensor_result["t90_sec"]],
            [
                sensor_result["baseline_value"]
                + direction * 0.10 * sensor_result["signed_amplitude"],
                sensor_result["baseline_value"]
                + direction * 0.90 * sensor_result["signed_amplitude"],
            ],
            color=PLOT_COLORS[sensor],
            s=30,
            zorder=5,
            label="10% / 90%",
        )
    add_step_boundaries(axis, result)
    axis.set_xlabel("Time from command step [s]")
    axis.set_ylabel(
        "Linear velocity [m/s]" if axis_name == "linear" else "Angular velocity [rad/s]"
    )
    axis.set_title("{} {} response".format(sensor.upper(), axis_name))
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def make_acceleration_plot(path, result, sensor):
    plt = prepare_plotting()
    axis_name = result["axis"]
    start = float(result["command"]["start_time_sec"])
    signal = result["signals"][sensor]
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        signal["acceleration_time"] - start,
        signal["acceleration"],
        color=PLOT_COLORS[sensor],
        linewidth=1.35,
        label="{} derived acceleration".format(sensor.upper()),
    )
    axis.axhline(0.0, color="#777777", linewidth=0.8)
    add_step_boundaries(axis, result)
    axis.set_xlabel("Time from command step [s]")
    axis.set_ylabel(
        "Linear acceleration [m/s^2]"
        if axis_name == "linear"
        else "Angular acceleration [rad/s^2]"
    )
    axis.set_title(
        "{} {} acceleration ({} s local slope)".format(
            sensor.upper(), axis_name, result["acceleration_window_sec"]
        )
    )
    capability = result["sensors"][sensor].get("acceleration_capability", {})
    response = result["sensors"][sensor]
    if capability.get("valid", False):
        annotation = "a_eff={:.3f}\na_P95={:.3f}".format(
            response["effective_acceleration"], capability["acceleration_p95"]
        )
        if capability.get("braking_valid", False):
            annotation += "\ndecel_eff={:.3f}\ndecel_P95={:.3f}".format(
                response["effective_deceleration"], capability["deceleration_p95"]
            )
        axis.text(
            0.985,
            0.76,
            annotation,
            transform=axis.transAxes,
            ha="right",
            va="top",
            bbox={"facecolor": "white", "edgecolor": "#BBBBBB", "alpha": 0.88},
        )
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def make_fopdt_plot(path, result, sensor):
    sensor_result = result["sensors"][sensor]
    fit = sensor_result.get("fopdt", {})
    if not fit.get("valid", False):
        return False
    plt = prepare_plotting()
    start = float(result["command"]["start_time_sec"])
    signal = result["signals"][sensor]
    command = result["signals"]["command"]
    fit_window = fit.get("fit_window", {})
    fit_start = float(fit_window.get("start_time_sec", signal["time"][0]))
    fit_stop = float(fit_window.get("stop_time_sec", signal["time"][-1]))
    fit_mask = (signal["time"] >= fit_start) & (signal["time"] <= fit_stop)
    plot_time = signal["time"][fit_mask]
    plot_value = signal["value"][fit_mask]
    if plot_time.size < 2:
        return False
    command_on_sensor_grid = delay_core.zoh_resample(
        command["time"], command["value"], plot_time
    )
    basis = delay_core.simulate_fopdt(
        plot_time, command_on_sensor_grid, fit["delay_sec"], fit["tau_sec"]
    )
    predicted = fit["offset"] + fit["gain"] * basis
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        plot_time - start,
        plot_value,
        color=PLOT_COLORS[sensor],
        linewidth=1.35,
        label="measured {}".format(sensor.upper()),
    )
    axis.plot(
        plot_time - start,
        predicted,
        color="#CC3311",
        linewidth=1.25,
        linestyle="--",
        label="FOPDT fit",
    )
    axis.axvline(0.0, color="#555555", linestyle="--", linewidth=0.9)
    axis.set_xlim(float(plot_time[0] - start), float(plot_time[-1] - start))
    axis.set_xlabel("Time from command step [s]")
    axis.set_ylabel(
        "Linear velocity [m/s]"
        if result["axis"] == "linear"
        else "Angular velocity [rad/s]"
    )
    axis.set_title(
        "{} rise FOPDT: L={:.1f} ms, tau={:.1f} ms, K={:.3f}, R2={:.3f} ({})".format(
            sensor.upper(),
            1000.0 * fit["delay_sec"],
            1000.0 * fit["tau_sec"],
            fit["gain"],
            fit["r2"],
            "accepted" if fit.get("identifiable", False) else "diagnostic only",
        )
    )
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)
    return True


def make_linear_imu_acceleration_plot(path, result):
    values = result["acceleration_crosscheck"].get("imu", {})
    signal = result["signals"].get("imu_acceleration")
    if signal is None:
        return False
    plt = prepare_plotting()
    start = float(result["command"]["start_time_sec"])
    baseline = float(values.get("baseline_value", 0.0))
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        signal["time"] - start,
        signal["value"] - baseline,
        color=PLOT_COLORS["imu"],
        linewidth=1.25,
        label="IMU forward acceleration minus static bias",
    )
    if values.get("valid", False):
        axis.axvline(
            values["onset_delay_sec"],
            color="#CC3311",
            linestyle=":",
            linewidth=1.1,
            label="detected onset",
        )
    axis.axhline(0.0, color="#777777", linewidth=0.8)
    add_step_boundaries(axis, result)
    axis.set_xlabel("Time from command step [s]")
    axis.set_ylabel("Linear acceleration [m/s^2]")
    axis.set_title("IMU linear-acceleration onset cross-check")
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)
    return True


def make_delay_bar(path, rows, field, title):
    available = [
        (name, values[field])
        for name, values in sorted(rows.items())
        if values.get(field) is not None
    ]
    if not available:
        return False
    plt = prepare_plotting()
    labels = [name.replace("_acceleration", "\naccel") for name, _ in available]
    values_ms = [1000.0 * float(value) for _, value in available]
    figure, axis = plt.subplots(figsize=(7.4, 4.6), constrained_layout=True)
    bars = axis.bar(labels, values_ms, color="#4477AA", width=0.58)
    for bar, value in zip(bars, values_ms):
        axis.text(
            bar.get_x() + 0.5 * bar.get_width(),
            bar.get_height(),
            "{:.1f}".format(value),
            ha="center",
            va="bottom",
        )
    axis.set_ylabel("Delay [ms]")
    axis.set_title(title)
    axis.grid(True, axis="y", alpha=0.25)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)
    return True


def make_separated_plots(plot_dir, result):
    plot_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for sensor in sorted(result["sensors"]):
        velocity_path = plot_dir / "{}_{}_velocity.png".format(result["axis"], sensor)
        make_velocity_plot(velocity_path, result, sensor)
        paths.append(velocity_path)
        acceleration_path = plot_dir / "{}_{}_acceleration.png".format(
            result["axis"], sensor
        )
        make_acceleration_plot(acceleration_path, result, sensor)
        paths.append(acceleration_path)
        fit_path = plot_dir / "{}_{}_fopdt_fit.png".format(result["axis"], sensor)
        if make_fopdt_plot(fit_path, result, sensor):
            paths.append(fit_path)
    if result["axis"] == "linear":
        imu_path = plot_dir / "linear_imu_acceleration.png"
        if make_linear_imu_acceleration_plot(imu_path, result):
            paths.append(imu_path)
    rows = delay_rows(result)
    onset_path = plot_dir / "delay_onset_crosscheck.png"
    if make_delay_bar(onset_path, rows, "onset_delay_sec", "Observed onset delay cross-check"):
        paths.append(onset_path)
    fit_delay_path = plot_dir / "delay_fopdt_crosscheck.png"
    if make_delay_bar(fit_delay_path, rows, "fopdt_delay_sec", "FOPDT dead-time cross-check"):
        paths.append(fit_delay_path)
    return paths


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--axis", choices=("linear", "angular"), required=True)
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--cmd-topic", default=CMD_TOPIC)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--stamped-command-topic", default=STAMPED_CMD_TOPIC)
    parser.add_argument(
        "--protocol-id",
        choices=("MOCAP_VELOCITY_STEP_V2", "MOCAP_HARDWARE_ACCEL_LIMIT_V1"),
        default="MOCAP_VELOCITY_STEP_V2",
    )
    parser.add_argument("--run-label", default="UNSPECIFIED")
    parser.add_argument(
        "--data-split",
        choices=("development", "validation", "final_test"),
        default="development",
    )
    parser.add_argument("--matrix-row", default="single")
    parser.add_argument("--attempt", default="01")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument(
        "--plot-dir",
        type=Path,
        help="Directory for separated sensor/metric plots",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        help="Deprecated compatibility alias; its stem becomes a separated plot directory",
    )
    parser.add_argument("--minimum-command", type=float)
    parser.add_argument("--minimum-step-duration", type=float, default=2.0)
    parser.add_argument("--baseline-sec", type=float, default=1.5)
    parser.add_argument("--steady-sec", type=float, default=0.6)
    parser.add_argument("--guard-sec", type=float, default=0.10)
    parser.add_argument("--sustain-sec", type=float, default=0.06)
    parser.add_argument("--onset-noise-multiplier", type=float, default=6.0)
    parser.add_argument("--onset-fraction", type=float, default=0.02)
    parser.add_argument("--linear-mocap-onset-floor", type=float, default=0.004)
    parser.add_argument("--linear-odom-onset-floor", type=float, default=0.005)
    parser.add_argument("--angular-mocap-onset-floor", type=float, default=0.008)
    parser.add_argument("--angular-imu-onset-floor", type=float, default=0.005)
    parser.add_argument("--angular-odom-onset-floor", type=float, default=0.008)
    parser.add_argument("--mocap-median-window", type=int, default=3)
    parser.add_argument("--mocap-velocity-window-sec", type=float, default=0.12)
    parser.add_argument("--imu-median-window", type=int, default=3)
    parser.add_argument("--odom-median-window", type=int, default=3)
    parser.add_argument("--acceleration-window-sec", type=float, default=0.12)
    parser.add_argument("--acceleration-median-window", type=int, default=3)
    parser.add_argument("--imu-acceleration-window-sec", type=float, default=0.08)
    parser.add_argument("--imu-acceleration-median-window", type=int, default=3)
    parser.add_argument("--acceleration-onset-sustain-sec", type=float, default=0.04)
    parser.add_argument("--linear-imu-accel-onset-floor", type=float, default=0.04)
    parser.add_argument("--imu-forward-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--imu-forward-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--fopdt-max-delay-sec", type=float, default=1.20)
    parser.add_argument("--fopdt-min-tau-sec", type=float, default=0.01)
    parser.add_argument("--fopdt-max-tau-sec", type=float, default=2.50)
    parser.add_argument("--fopdt-tau-count", type=int, default=96)
    parser.add_argument(
        "--fopdt-post-sec",
        type=float,
        default=0.0,
        help="Extra post-command data in the FOPDT fit; default 0 keeps the asymmetric braking edge out",
    )
    parser.add_argument("--fopdt-min-r2", type=float, default=0.80)
    parser.add_argument("--fopdt-near-optimal-fraction", type=float, default=0.01)
    return parser


def validate_analysis_args(args):
    positive = {
        "acceleration-window-sec": args.acceleration_window_sec,
        "mocap-velocity-window-sec": args.mocap_velocity_window_sec,
        "imu-acceleration-window-sec": args.imu_acceleration_window_sec,
        "acceleration-onset-sustain-sec": args.acceleration_onset_sustain_sec,
        "linear-imu-accel-onset-floor": args.linear_imu_accel_onset_floor,
        "fopdt-max-delay-sec": args.fopdt_max_delay_sec,
        "fopdt-min-tau-sec": args.fopdt_min_tau_sec,
        "fopdt-max-tau-sec": args.fopdt_max_tau_sec,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))
    if not math.isfinite(args.fopdt_post_sec) or args.fopdt_post_sec < 0.0:
        raise ValueError("fopdt-post-sec must be finite and non-negative")
    if not math.isfinite(args.fopdt_min_r2) or not 0.0 <= args.fopdt_min_r2 <= 1.0:
        raise ValueError("fopdt-min-r2 must be finite and within [0, 1]")
    if args.fopdt_min_tau_sec >= args.fopdt_max_tau_sec:
        raise ValueError("fopdt-min-tau-sec must be below fopdt-max-tau-sec")
    if args.fopdt_tau_count < 8:
        raise ValueError("fopdt-tau-count must be at least 8")
    for name, value in (
        ("acceleration-median-window", args.acceleration_median_window),
        ("imu-acceleration-median-window", args.imu_acceleration_median_window),
    ):
        if value < 1:
            raise ValueError("{} must be at least 1".format(name))


def main():
    args = build_parser().parse_args()
    try:
        validate_analysis_args(args)
    except ValueError as exc:
        print("[analyze_mocap_velocity_step][ERR] {}".format(exc), file=sys.stderr)
        return 2
    if not args.bag.is_file():
        print("bag does not exist: {}".format(args.bag), file=sys.stderr)
        return 2
    try:
        data, mocap_topic = load_bag(
            args.bag,
            args.mocap_tracker,
            args.imu_topic,
            args.stamped_command_topic,
            args.cmd_topic,
            args.odom_topic,
        )
        primary = analyze_timebase(
            data["command_bag"],
            data["mocap_bag"],
            data["imu_bag"],
            data["odom_bag"],
            args.axis,
            args,
        )
        secondary = analyze_timebase(
            data["command_header"],
            data["mocap_header"],
            data["imu_header"],
            data["odom_header"],
            args.axis,
            args,
        )
    except Exception as exc:
        print("[analyze_mocap_velocity_step][ERR] {}".format(exc), file=sys.stderr)
        return 2

    print_report(args.bag, primary, secondary)
    delay_cross_validation = build_delay_cross_validation(primary, secondary)
    report = {
        "protocol_id": args.protocol_id,
        "run_label": args.run_label,
        "data_split": args.data_split,
        "matrix_row": args.matrix_row,
        "attempt": args.attempt,
        "axis": args.axis,
        "bag": str(args.bag.resolve()),
        "topics": {
            "command": args.cmd_topic,
            "stamped_command": args.stamped_command_topic,
            "mocap": mocap_topic,
            "imu": args.imu_topic,
            "odom": args.odom_topic,
        },
        "primary_timebase": "rosbag_arrival_time",
        "bag_time": serializable_timebase(primary),
        "header_time": serializable_timebase(secondary),
        "delay_cross_validation": delay_cross_validation,
        "analysis_config": {
            "acceleration_window_sec": args.acceleration_window_sec,
            "mocap_velocity_window_sec": args.mocap_velocity_window_sec,
            "acceleration_median_window": args.acceleration_median_window,
            "imu_acceleration_window_sec": args.imu_acceleration_window_sec,
            "imu_acceleration_median_window": args.imu_acceleration_median_window,
            "linear_imu_accel_onset_floor": args.linear_imu_accel_onset_floor,
            "imu_forward_axis": args.imu_forward_axis,
            "imu_forward_sign": args.imu_forward_sign,
            "fopdt_max_delay_sec": args.fopdt_max_delay_sec,
            "fopdt_min_tau_sec": args.fopdt_min_tau_sec,
            "fopdt_max_tau_sec": args.fopdt_max_tau_sec,
            "fopdt_tau_count": args.fopdt_tau_count,
            "fopdt_post_sec": args.fopdt_post_sec,
            "fopdt_min_r2": args.fopdt_min_r2,
            "fopdt_near_optimal_fraction": args.fopdt_near_optimal_fraction,
        },
        "stream_rate_hz": {
            "command": stream_rate(data["command_bag"]),
            "mocap": stream_rate(data["mocap_bag"]),
            "imu": stream_rate(data["imu_bag"]),
            "odom": stream_rate(data["odom_bag"]),
        },
        "measurement_definition": {
            "mocap_velocity": "zero-phase local-linear slope of NOKOV position/yaw; the configured window is recorded and timing is cross-checked against raw IMU/odom",
            "onset": "first sustained response above max(floor, 6*sigma, 2% steady amplitude)",
            "effective_acceleration": "80% measured steady velocity divided by t10-to-t90 rise time",
            "effective_deceleration": "80% measured steady velocity divided by 90%-to-10% fall time",
            "acceleration_curve": "zero-phase local-linear derivative; window is recorded and peaks are supporting capability estimates",
            "fopdt": "rising-edge-only grid fit of y=offset+gain*first_order(command delayed by L); braking is evaluated separately; parameters are accepted only when the quality gate passes",
            "linear_imu": "bias-removed configured forward-axis acceleration is onset-only supporting evidence",
            "warning": "t90 includes gradual response build-up and is not pure dead time",
        },
    }
    plot_dir = args.plot_dir
    if plot_dir is None and args.plot is not None:
        plot_dir = args.plot.parent / "{}_plots".format(args.plot.stem)
        print("plot alias: writing separated plots to {}".format(plot_dir))
    if plot_dir is not None:
        plot_paths = make_separated_plots(plot_dir, primary)
        report["plots"] = [str(path.resolve()) for path in plot_paths]
        print("plot dir  : {} ({} files)".format(plot_dir, len(plot_paths)))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print("report    : {}".format(args.output_json))

    required_sensors = (
        ("mocap", "odom")
        if args.axis == "linear"
        else ("mocap", "imu", "odom")
    )
    complete = all(
        primary["sensors"][sensor].get("valid", False)
        and primary["sensors"][sensor].get("braking_valid", False)
        and primary["sensors"][sensor].get("fopdt", {}).get("valid", False)
        and primary["sensors"][sensor]
        .get("acceleration_capability", {})
        .get("valid", False)
        for sensor in required_sensors
    )
    if args.axis == "linear":
        complete = complete and primary["acceleration_crosscheck"].get("imu", {}).get(
            "valid", False
        )
    return 0 if complete else 2


if __name__ == "__main__":
    sys.exit(main())
