#!/usr/bin/env python3
"""Measure velocity-step delay, acceleration, braking, and stopping performance."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

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
                omega = float(message.angular_velocity.z)
                data["imu_bag"].append((bag_time, omega))
                data["imu_header"].append((header_time(message, bag_time), omega))
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


def derive_linear_mocap(rows, command_step, median_window):
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
    delta_time = np.diff(time)
    valid = delta_time > 0.0
    midpoint = 0.5 * (time[1:] + time[:-1])
    world_velocity = np.column_stack((np.diff(x), np.diff(y))) / delta_time[:, None]
    longitudinal = world_velocity @ body_positive_axis
    longitudinal = core.centered_median(longitudinal, median_window)
    relative_position = np.column_stack((x, y)) - p0[None, :]
    along_position = relative_position @ body_positive_axis
    lateral_position = relative_position @ lateral_axis
    return {
        "time": midpoint[valid],
        "velocity": longitudinal[valid],
        "position_time": time,
        "along_position": along_position,
        "lateral_position": lateral_position,
        "body_positive_axis_xy": body_positive_axis,
        "step_displacement_m": displacement_norm,
    }


def derive_angular_mocap(rows, median_window):
    pose = unique_matrix(rows)
    if pose.shape[0] < 5 or pose.shape[1] < 4:
        raise ValueError("too few NOKOV pose samples for angular velocity")
    yaw = np.unwrap(pose[:, 3])
    rate_time, omega = core.derive_yaw_rate(
        pose[:, 0], yaw, median_window=median_window
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
    signals = {
        "command": {"time": command_time, "value": command_value, "label": "final /cmd_vel"}
    }
    if axis == "linear":
        mocap = derive_linear_mocap(mocap_rows, command_step, args.mocap_median_window)
        odom_time, odom_value = select_odom(odom_rows, axis, args.odom_median_window)
        sensor_inputs = {
            "mocap": (mocap["time"], mocap["velocity"], "NOKOV longitudinal velocity"),
            "odom": (odom_time, odom_value, "odom linear velocity"),
        }
    else:
        mocap = derive_angular_mocap(mocap_rows, args.mocap_median_window)
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
        sensors[sensor] = result
        signals[sensor] = {"time": time, "value": value, "label": label}
    attach_mocap_stopping_metrics(axis, mocap, command_step, sensors["mocap"])
    return {"axis": axis, "command": command_step, "sensors": sensors, "signals": signals}


def serializable_timebase(result):
    return {
        "axis": result["axis"],
        "command": result["command"],
        "sensors": result["sensors"],
    }


def milliseconds(value):
    return "n/a" if value is None else "{:.1f}".format(1000.0 * float(value))


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
    print("Note: effective acceleration uses the measured 10%-90% velocity slope.")


def make_plot(path, result):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    axis_name = result["axis"]
    start = result["command"]["start_time_sec"]
    stop = result["command"]["stop_time_sec"]
    colors = {"command": "#173F67", "mocap": "#0072B2", "imu": "#E69F00", "odom": "#009E73"}
    figure, axis = plt.subplots(figsize=(10.0, 5.2), constrained_layout=True)
    for name, signal in result["signals"].items():
        axis.plot(
            signal["time"] - start,
            signal["value"],
            label=signal["label"],
            color=colors[name],
            linewidth=1.8 if name == "command" else 1.1,
            alpha=1.0 if name != "imu" else 0.82,
        )
    command_direction = 1.0 if result["command"]["command_value"] > 0.0 else -1.0
    for sensor, sensor_result in result["sensors"].items():
        if not sensor_result.get("valid", False):
            continue
        axis.axvline(
            sensor_result["onset_delay_sec"],
            color=colors[sensor],
            linestyle=":",
            linewidth=0.9,
        )
        axis.scatter(
            [sensor_result["t10_sec"], sensor_result["t90_sec"]],
            [
                sensor_result["baseline_value"]
                + command_direction * 0.10 * sensor_result["signed_amplitude"],
                sensor_result["baseline_value"]
                + command_direction * 0.90 * sensor_result["signed_amplitude"],
            ],
            color=colors[sensor],
            s=24,
            zorder=5,
        )
    axis.axvline(0.0, color="#555555", linestyle="--", linewidth=0.9)
    axis.axvline(stop - start, color="#888888", linestyle="--", linewidth=0.9)
    axis.set_xlim(-1.0, stop - start + 3.0)
    axis.set_xlabel("Time from command step [s]")
    axis.set_ylabel("Linear velocity [m/s]" if axis_name == "linear" else "Angular velocity [rad/s]")
    axis.set_title("Stationary {} velocity-step response".format(axis_name))
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=160)
    plt.close(figure)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--axis", choices=("linear", "angular"), required=True)
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--cmd-topic", default=CMD_TOPIC)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--stamped-command-topic", default=STAMPED_CMD_TOPIC)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--plot", type=Path)
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
    parser.add_argument("--imu-median-window", type=int, default=3)
    parser.add_argument("--odom-median-window", type=int, default=3)
    return parser


def main():
    args = build_parser().parse_args()
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
    report = {
        "protocol_id": "MOCAP_VELOCITY_STEP_V1",
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
        "stream_rate_hz": {
            "command": stream_rate(data["command_bag"]),
            "mocap": stream_rate(data["mocap_bag"]),
            "imu": stream_rate(data["imu_bag"]),
            "odom": stream_rate(data["odom_bag"]),
        },
        "measurement_definition": {
            "onset": "first sustained response above max(floor, 6*sigma, 2% steady amplitude)",
            "effective_acceleration": "80% measured steady velocity divided by t10-to-t90 rise time",
            "effective_deceleration": "80% measured steady velocity divided by 90%-to-10% fall time",
            "warning": "t90 includes gradual response build-up and is not pure dead time",
        },
    }
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print("report    : {}".format(args.output_json))
    if args.plot:
        make_plot(args.plot, primary)
        print("plot      : {}".format(args.plot))

    required_sensors = ("mocap", "odom") if args.axis == "linear" else ("mocap", "imu")
    complete = all(
        primary["sensors"][sensor].get("valid", False)
        and primary["sensors"][sensor].get("braking_valid", False)
        for sensor in required_sensors
    )
    return 0 if complete else 2


if __name__ == "__main__":
    sys.exit(main())
