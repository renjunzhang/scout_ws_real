#!/usr/bin/env python3
"""Analyze chassis speed continuity under a continuous velocity profile."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

import analyze_mocap_velocity_step as step_analyzer
import velocity_continuity_core as continuity_core
import velocity_step_response_core as response_core


CMD_TOPIC = "/cmd_vel"
STAMPED_CMD_TOPIC = "/mocap_velocity_continuity/cmd_vel_stamped"
ODOM_TOPIC = "/odom"


COLORS = {
    "command": "#555555",
    "mocap": "#0072B2",
    "imu": "#E69F00",
    "odom": "#009E73",
}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--axis", choices=("linear", "angular"), required=True)
    parser.add_argument("--profile", choices=continuity_core.PROFILES, required=True)
    parser.add_argument("--mocap-tracker", default="Tracker0")
    parser.add_argument("--cmd-topic", default=CMD_TOPIC)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--stamped-command-topic", default=STAMPED_CMD_TOPIC)
    parser.add_argument("--run-label", default="UNSPECIFIED")
    parser.add_argument(
        "--data-split",
        choices=("development", "validation", "final_test"),
        default="development",
    )
    parser.add_argument("--attempt", default="01")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--plot-dir", type=Path)
    parser.add_argument("--mocap-median-window", type=int, default=3)
    parser.add_argument("--mocap-velocity-window-sec", type=float, default=0.12)
    parser.add_argument("--odom-median-window", type=int, default=3)
    parser.add_argument("--imu-median-window", type=int, default=3)
    parser.add_argument("--acceleration-window-sec", type=float, default=0.12)
    parser.add_argument("--acceleration-median-window", type=int, default=3)
    parser.add_argument("--imu-forward-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--imu-forward-sign", choices=(-1.0, 1.0), type=float, default=1.0)
    parser.add_argument("--resample-dt-sec", type=float, default=0.02)
    parser.add_argument("--max-effective-lag-sec", type=float, default=1.20)
    return parser


def validate_args(args):
    for name, value in (
        ("mocap-velocity-window-sec", args.mocap_velocity_window_sec),
        ("acceleration-window-sec", args.acceleration_window_sec),
        ("resample-dt-sec", args.resample_dt_sec),
        ("max-effective-lag-sec", args.max_effective_lag_sec),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))
    for name, value in (
        ("mocap-median-window", args.mocap_median_window),
        ("odom-median-window", args.odom_median_window),
        ("imu-median-window", args.imu_median_window),
        ("acceleration-median-window", args.acceleration_median_window),
    ):
        if value < 1:
            raise ValueError("{} must be at least one".format(name))


def load_metadata(path, axis, profile):
    if not path.is_file():
        raise ValueError("metadata does not exist: {}".format(path))
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("protocol_id") != "MOCAP_VELOCITY_CONTINUITY_V1":
        raise ValueError("metadata protocol_id mismatch")
    if payload.get("axis") != axis or payload.get("profile") != profile:
        raise ValueError("metadata axis/profile mismatch")
    if int(payload.get("exit_code", -1)) != 0:
        raise ValueError("publisher metadata reports a failed motion")
    stamps = payload.get("phase_stamps_sec", {})
    required = ("ramp_up", "hold", "ramp_down")
    missing = [name for name in required if name not in stamps]
    if missing:
        raise ValueError("metadata is missing phases: {}".format(", ".join(missing)))
    phases = {
        "ramp_up_start": float(stamps["ramp_up"]["first"]),
        "ramp_up_stop": float(stamps["ramp_up"]["last"]),
        "hold_start": float(stamps["hold"]["first"]),
        "hold_stop": float(stamps["hold"]["last"]),
        "ramp_down_start": float(stamps["ramp_down"]["first"]),
        "ramp_down_stop": float(stamps["ramp_down"]["last"]),
    }
    if not (
        phases["ramp_up_start"] < phases["ramp_up_stop"]
        <= phases["hold_start"] < phases["hold_stop"]
        <= phases["ramp_down_start"] < phases["ramp_down_stop"]
    ):
        raise ValueError("metadata phase timestamps are not monotonic")
    return payload, phases


def select_angular_imu(rows, median_window):
    array = step_analyzer.unique_matrix(rows)
    if array.shape[0] < 20 or array.shape[1] < 5:
        raise ValueError("too few IMU samples for angular continuity")
    return array[:, 0], response_core.centered_median(array[:, 1], median_window)


def derive_velocity_signals(data, axis, phases, target_value, args):
    pseudo_command = {
        "start_time_sec": phases["ramp_up_start"],
        "stop_time_sec": phases["ramp_down_stop"],
        "command_value": target_value,
    }
    if axis == "linear":
        mocap = step_analyzer.derive_linear_mocap(
            data["mocap_header"],
            pseudo_command,
            args.mocap_median_window,
            args.mocap_velocity_window_sec,
        )
        mocap_time, mocap_value = mocap["time"], mocap["velocity"]
    else:
        mocap = step_analyzer.derive_angular_mocap(
            data["mocap_header"],
            args.mocap_median_window,
            args.mocap_velocity_window_sec,
        )
        mocap_time, mocap_value = mocap["time"], mocap["velocity"]

    odom_time, odom_value = step_analyzer.select_odom(
        data["odom_header"], axis, args.odom_median_window
    )
    signals = {
        "mocap": {
            "time": mocap_time,
            "value": mocap_value,
            "label": "NOKOV measured velocity",
        },
        "odom": {
            "time": odom_time,
            "value": odom_value,
            "label": "odom measured velocity",
        },
    }
    if axis == "angular":
        imu_time, imu_value = select_angular_imu(
            data["imu_header"], args.imu_median_window
        )
        signals["imu"] = {
            "time": imu_time,
            "value": imu_value,
            "label": "IMU measured angular velocity",
        }
    return signals


def command_mirror_audit(command_header_rows, command_bag_rows, axis, phases):
    stamped_time, stamped_value = step_analyzer.select_command(command_header_rows, axis)
    bag_time, bag_value = step_analyzer.select_command(command_bag_rows, axis)
    start = phases["ramp_up_start"]
    stop = phases["ramp_down_stop"]
    mask = (stamped_time >= start) & (stamped_time <= stop)
    query_time = stamped_time[mask]
    query_value = stamped_value[mask]
    if query_time.size < 20:
        raise ValueError("too few stamped command samples for mirror audit")
    time_error = []
    value_error = []
    for stamp, value in zip(query_time, query_value):
        right = int(np.searchsorted(bag_time, stamp))
        candidates = [index for index in (right - 1, right) if 0 <= index < bag_time.size]
        nearest = min(candidates, key=lambda index: abs(bag_time[index] - stamp))
        time_error.append(abs(float(bag_time[nearest] - stamp)))
        value_error.append(abs(float(bag_value[nearest] - value)))
    time_error = np.asarray(time_error)
    value_error = np.asarray(value_error)
    matched = time_error <= 0.05
    coverage = float(np.mean(matched))
    max_value_error = float(np.max(value_error[matched])) if np.any(matched) else None
    return {
        "sample_count": int(query_time.size),
        "matched_within_50ms_fraction": coverage,
        "time_difference_p95_ms": float(np.percentile(time_error, 95.0) * 1000.0),
        "time_difference_max_ms": float(np.max(time_error) * 1000.0),
        "value_error_p95": (
            float(np.percentile(value_error[matched], 95.0)) if np.any(matched) else None
        ),
        "value_error_max": max_value_error,
        "pass": bool(
            coverage >= 0.98
            and max_value_error is not None
            and max_value_error <= 1e-6
        ),
    }


def acceleration_signal(time, value, args):
    return response_core.smooth_derivative(
        time,
        value,
        window_sec=args.acceleration_window_sec,
        median_window=args.acceleration_median_window,
    )


def strip_plot_signals(metrics):
    output = dict(metrics)
    output.pop("plot_signals", None)
    return output


def stream_rates(data):
    return {
        "command": step_analyzer.stream_rate(data["command_header"]),
        "mocap": step_analyzer.stream_rate(data["mocap_header"]),
        "imu": step_analyzer.stream_rate(data["imu_header"]),
        "odom": step_analyzer.stream_rate(data["odom_header"]),
    }


def transport_timing(data):
    output = {}
    for sensor in ("mocap", "imu", "odom"):
        bag_rows = data["{}_bag".format(sensor)]
        header_rows = data["{}_header".format(sensor)]
        count = min(len(bag_rows), len(header_rows))
        if count < 2:
            continue
        delta_ms = 1000.0 * (
            np.asarray([row[0] for row in bag_rows[:count]])
            - np.asarray([row[0] for row in header_rows[:count]])
        )
        output[sensor] = {
            "sample_count": count,
            "arrival_minus_header_median_ms": float(np.median(delta_ms)),
            "arrival_minus_header_p95_ms": float(np.percentile(delta_ms, 95.0)),
            "arrival_minus_header_max_ms": float(np.max(delta_ms)),
        }
    return output


def prepare_plotting():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def add_phase_boundaries(axis, phases):
    start = phases["ramp_up_start"]
    boundaries = (
        (0.0, "ramp start"),
        (phases["ramp_up_stop"] - start, "hold start"),
        (phases["hold_stop"] - start, "ramp-down start"),
        (phases["ramp_down_stop"] - start, "profile end"),
    )
    for value, _ in boundaries:
        axis.axvline(value, color="#888888", linestyle="--", linewidth=0.8)
    axis.set_xlim(-1.0, boundaries[-1][0] + 1.5)
    axis.grid(True, alpha=0.25)


def make_velocity_plot(path, command, signal, metrics, phases, axis_name, sensor):
    plt = prepare_plotting()
    start = phases["ramp_up_start"]
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        command["time"] - start,
        command["value"],
        color=COLORS["command"],
        linestyle="--",
        linewidth=1.15,
        label="published command",
    )
    axis.plot(
        signal["time"] - start,
        signal["value"],
        color=COLORS[sensor],
        linewidth=1.4,
        label=signal["label"],
    )
    add_phase_boundaries(axis, phases)
    unit = "m/s" if axis_name == "linear" else "rad/s"
    hold = metrics["hold"]
    axis.set_xlabel("Time from ramp start [s]")
    axis.set_ylabel("Velocity [{}]".format(unit))
    axis.set_title(
        "{} {} speed continuity: hold std={:.4f} {}".format(
            sensor.upper(), axis_name, hold["speed_std"], unit
        )
    )
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def make_acceleration_plot(
    path, command_acceleration, signal_acceleration, phases, axis_name, sensor
):
    plt = prepare_plotting()
    start = phases["ramp_up_start"]
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        command_acceleration["time"] - start,
        command_acceleration["value"],
        color=COLORS["command"],
        linestyle="--",
        linewidth=1.15,
        label="command-derived acceleration",
    )
    axis.plot(
        signal_acceleration["time"] - start,
        signal_acceleration["value"],
        color=COLORS[sensor],
        linewidth=1.3,
        label="{} measured acceleration".format(sensor.upper()),
    )
    axis.axhline(0.0, color="#777777", linewidth=0.8)
    add_phase_boundaries(axis, phases)
    unit = "m/s²" if axis_name == "linear" else "rad/s²"
    axis.set_xlabel("Time from ramp start [s]")
    axis.set_ylabel("Acceleration [{}]".format(unit))
    axis.set_title("{} {} acceleration continuity".format(sensor.upper(), axis_name))
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def make_delta_velocity_plot(path, plot_signals, phases, axis_name, sensor, dt_sec):
    plt = prepare_plotting()
    start = phases["ramp_up_start"]
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        plot_signals["delta_time"] - start,
        plot_signals["delta_predicted"],
        color=COLORS["command"],
        linestyle="--",
        linewidth=1.05,
        label="aligned command Δv",
    )
    axis.plot(
        plot_signals["delta_time"] - start,
        plot_signals["delta_measured"],
        color=COLORS[sensor],
        linewidth=1.25,
        label="{} measured Δv".format(sensor.upper()),
    )
    axis.axhline(0.0, color="#777777", linewidth=0.8)
    add_phase_boundaries(axis, phases)
    unit = "m/s" if axis_name == "linear" else "rad/s"
    axis.set_xlabel("Time from ramp start [s]")
    axis.set_ylabel("Velocity change per {:.0f} ms [{}]".format(1000.0 * dt_sec, unit))
    axis.set_title("{} {} per-cycle speed change".format(sensor.upper(), axis_name))
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def make_linear_imu_acceleration_plot(
    path, data, command_acceleration, phases, args
):
    imu_time, imu_value = step_analyzer.select_linear_imu_acceleration(
        data["imu_header"], args.imu_forward_axis, args.imu_forward_sign
    )
    start = phases["ramp_up_start"]
    baseline_mask = (imu_time >= start - 1.5) & (imu_time <= start - 0.1)
    if np.count_nonzero(baseline_mask) < 10:
        raise ValueError("insufficient static IMU baseline")
    baseline = float(np.median(imu_value[baseline_mask]))
    filtered = response_core.centered_median(imu_value - baseline, 3)
    plt = prepare_plotting()
    figure, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    axis.plot(
        command_acceleration["time"] - start,
        command_acceleration["value"],
        color=COLORS["command"],
        linestyle="--",
        linewidth=1.15,
        label="command-derived acceleration",
    )
    axis.plot(
        imu_time - start,
        filtered,
        color=COLORS["imu"],
        linewidth=1.2,
        label="IMU forward acceleration minus static bias",
    )
    axis.axhline(0.0, color="#777777", linewidth=0.8)
    add_phase_boundaries(axis, phases)
    axis.set_xlabel("Time from ramp start [s]")
    axis.set_ylabel("Linear acceleration [m/s²]")
    axis.set_title("IMU linear acceleration continuity cross-check")
    axis.legend(loc="best")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)


def main():
    args = build_parser().parse_args()
    try:
        validate_args(args)
        if not args.bag.is_file():
            raise ValueError("bag does not exist: {}".format(args.bag))
        metadata, phases = load_metadata(args.metadata, args.axis, args.profile)
        data, mocap_topic = step_analyzer.load_bag(
            args.bag,
            args.mocap_tracker,
            args.imu_topic,
            args.stamped_command_topic,
            args.cmd_topic,
            args.odom_topic,
        )
        command_time, command_value = step_analyzer.select_command(
            data["command_header"], args.axis
        )
        target_value = float(metadata["target_value"])
        signals = derive_velocity_signals(
            data, args.axis, phases, target_value, args
        )
        mirror = command_mirror_audit(
            data["command_header"], data["command_bag"], args.axis, phases
        )
        if not mirror["pass"]:
            raise ValueError("final /cmd_vel does not match the stamped command mirror")

        command_acceleration_time, command_acceleration_value = acceleration_signal(
            command_time, command_value, args
        )
        command = {"time": command_time, "value": command_value}
        command_acceleration = {
            "time": command_acceleration_time,
            "value": command_acceleration_value,
        }
        sensor_metrics = {}
        sensor_acceleration = {}
        for sensor, signal in signals.items():
            metrics = continuity_core.continuity_metrics(
                command_time,
                command_value,
                signal["time"],
                signal["value"],
                phases,
                args.profile,
                target_value,
                args.axis,
                resample_dt_sec=args.resample_dt_sec,
                max_lag_sec=args.max_effective_lag_sec,
            )
            acceleration_time, acceleration_value = acceleration_signal(
                signal["time"], signal["value"], args
            )
            sensor_metrics[sensor] = metrics
            sensor_acceleration[sensor] = {
                "time": acceleration_time,
                "value": acceleration_value,
            }

        plot_paths = []
        if args.plot_dir:
            for sensor, signal in sorted(signals.items()):
                velocity_path = args.plot_dir / "{}_{}_velocity.png".format(
                    args.axis, sensor
                )
                make_velocity_plot(
                    velocity_path,
                    command,
                    signal,
                    sensor_metrics[sensor],
                    phases,
                    args.axis,
                    sensor,
                )
                plot_paths.append(velocity_path)
                # NOKOV is the physical-motion primary result.  Avoid a
                # redundant acceleration/delta-v plot set for every sensor;
                # odom/gyro velocity remains only as a compact cross-check.
                if sensor == "mocap":
                    acceleration_path = args.plot_dir / "{}_{}_acceleration.png".format(
                        args.axis, sensor
                    )
                    make_acceleration_plot(
                        acceleration_path,
                        command_acceleration,
                        sensor_acceleration[sensor],
                        phases,
                        args.axis,
                        sensor,
                    )
                    plot_paths.append(acceleration_path)
                    delta_path = args.plot_dir / "{}_{}_delta_velocity_20ms.png".format(
                        args.axis, sensor
                    )
                    make_delta_velocity_plot(
                        delta_path,
                        sensor_metrics[sensor]["plot_signals"],
                        phases,
                        args.axis,
                        sensor,
                        args.resample_dt_sec,
                    )
                    plot_paths.append(delta_path)
            if args.axis == "linear":
                imu_path = args.plot_dir / "linear_imu_acceleration.png"
                make_linear_imu_acceleration_plot(
                    imu_path, data, command_acceleration, phases, args
                )
                plot_paths.append(imu_path)

        report = {
            "protocol_id": "MOCAP_VELOCITY_CONTINUITY_V1",
            "run_label": args.run_label,
            "data_split": args.data_split,
            "attempt": args.attempt,
            "axis": args.axis,
            "profile": args.profile,
            "profile_definition": (
                "constant acceleration; trapezoidal velocity with linear ramps"
                if args.profile in ("constant_accel", "trapezoidal_velocity")
                else "acceleration rises linearly from zero; velocity is quadratic in ramp progress"
            ),
            "target_value": target_value,
            "bag": str(args.bag.resolve()),
            "metadata": str(args.metadata.resolve()),
            "primary_timebase": "message_header_stamp",
            "topics": {
                "command": args.cmd_topic,
                "stamped_command": args.stamped_command_topic,
                "mocap": mocap_topic,
                "imu": args.imu_topic,
                "odom": args.odom_topic,
            },
            "phases": phases,
            "stream_rate_hz": stream_rates(data),
            "command_mirror_audit": mirror,
            "transport_timing": transport_timing(data),
            "sensors": {
                sensor: strip_plot_signals(metrics)
                for sensor, metrics in sensor_metrics.items()
            },
            "analysis_config": {
                "mocap_velocity_window_sec": args.mocap_velocity_window_sec,
                "acceleration_window_sec": args.acceleration_window_sec,
                "resample_dt_sec": args.resample_dt_sec,
                "max_effective_lag_sec": args.max_effective_lag_sec,
                "significant_drop_threshold": (
                    0.005 if args.axis == "linear" else 0.010
                ),
            },
            "plots": [str(path.resolve()) for path in plot_paths],
        }
    except Exception as exc:
        print("[analyze_mocap_velocity_continuity][ERR] {}".format(exc), file=sys.stderr)
        return 2

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    velocity_unit = "m/s" if args.axis == "linear" else "rad/s"
    print("== {} {} velocity continuity ==".format(args.axis, args.profile))
    print("bag     : {}".format(args.bag))
    print("target  : {:+.3f} {}".format(target_value, velocity_unit))
    print("cmd copy: PASS; coverage={:.4f}".format(
        mirror["matched_within_50ms_fraction"]
    ))
    for sensor in sorted(sensor_metrics):
        metrics = sensor_metrics[sensor]
        alignment = metrics["effective_alignment"]
        hold = metrics["hold"]
        print(
            "{}: effective lag={:.1f} ms; hold std={:.5f}; "
            "|delta-v residual| P95={:.5f}; drops={}; accel flips={}".format(
                sensor.upper(),
                1000.0 * alignment["lag_sec"],
                hold["speed_std"],
                metrics["delta_velocity_residual_p95_abs"],
                metrics["ramp_significant_drop_count"],
                metrics["ramp_acceleration_sign_flip_count"],
            )
        )
    if args.output_json:
        print("report   : {}".format(args.output_json))
    if args.plot_dir:
        print("plot dir : {} ({} files)".format(args.plot_dir, len(plot_paths)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
