#!/usr/bin/env python3

import math
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

import velocity_step_response_core as core  # noqa: E402
import analyze_mocap_velocity_step as analyzer  # noqa: E402
import publish_mocap_velocity_step as publisher  # noqa: E402
import summarize_mocap_velocity_step_matrix as matrix_summary  # noqa: E402


def make_step(command=0.2, delay=0.07, tau=0.50, noise=0.0005, duration=4.0):
    # Keep four seconds after the zero command so stop detection is exercised,
    # rather than testing only the rising edge.
    time = np.arange(-3.0, duration + 4.0, 0.01)
    command_values = np.zeros_like(time)
    command_values[(time >= 0.0) & (time < duration)] = command
    direction = 1.0 if command > 0.0 else -1.0
    elapsed = np.maximum(0.0, time - delay)
    response = direction * abs(command) * (1.0 - np.exp(-elapsed / tau))
    response[time < delay] = 0.0
    delayed_stop = duration + delay
    stop_value = abs(command) * (1.0 - math.exp(-duration / tau))
    after = time >= delayed_stop
    response[after] = direction * stop_value * np.exp(
        -(time[after] - delayed_stop) / tau
    )
    response += noise * np.sin(2.0 * math.pi * 7.3 * time)
    return time, command_values, response


def test_find_command_step_and_positive_response_times():
    time, command, response = make_step()
    step = core.find_command_step(time, command)
    result = core.analyze_step_response(
        time,
        response,
        step,
        onset_floor=0.003,
        onset_noise_multiplier=6.0,
        onset_fraction=0.02,
    )
    assert result["valid"]
    assert abs(step["start_time_sec"] - 0.0) <= 0.011
    assert abs(step["stop_time_sec"] - 4.0) <= 0.011
    assert 0.07 <= result["onset_delay_sec"] <= 0.11
    assert abs(result["t50_sec"] - (0.07 + 0.50 * math.log(2.0))) < 0.03
    assert abs(result["t90_sec"] - (0.07 + 0.50 * math.log(10.0))) < 0.04
    assert result["braking_valid"]
    assert result["effective_acceleration"] > 0.0
    assert result["effective_deceleration"] > 0.0


def test_right_turn_is_projected_into_command_direction():
    time, command, response = make_step(command=-0.2)
    step = core.find_command_step(time, command)
    result = core.analyze_step_response(time, response, step)
    assert result["valid"]
    assert result["steady_value"] < 0.0
    assert result["signed_amplitude"] > 0.0
    assert 0.0 < result["onset_delay_sec"] < result["t50_sec"] < result["t90_sec"]


def test_short_noise_spike_is_not_accepted_as_onset():
    time, command, response = make_step(delay=0.20)
    spike = (time >= 0.05) & (time < 0.06)
    response[spike] = 0.05
    step = core.find_command_step(time, command)
    result = core.analyze_step_response(time, response, step, sustain_sec=0.06)
    assert result["valid"]
    assert result["onset_delay_sec"] >= 0.19


def test_stationary_noise_too_large_fails_closed():
    time, command, response = make_step(noise=0.0)
    baseline = time < 0.0
    response[baseline] = 0.10 * np.sin(2.0 * math.pi * 9.0 * time[baseline])
    step = core.find_command_step(time, command)
    result = core.analyze_step_response(time, response, step)
    assert not result["valid"]
    assert "noise" in result["reason"]


def test_mocap_yaw_derivative_recovers_constant_rate():
    time = np.arange(0.0, 3.0, 0.01)
    yaw = np.mod(3.0 + 0.2 * time + math.pi, 2.0 * math.pi) - math.pi
    derived_time, omega = core.derive_yaw_rate(time, yaw)
    assert np.allclose(derived_time, 0.5 * (time[1:] + time[:-1]))
    assert np.max(np.abs(omega[5:-5] - 0.2)) < 1e-6


def test_local_linear_derivative_recovers_constant_acceleration():
    time = np.arange(0.0, 3.0, 0.01)
    velocity = 0.4 * time + 0.0002 * np.sin(11.0 * time)
    derived_time, acceleration = core.smooth_derivative(
        time, velocity, window_sec=0.12
    )
    assert np.allclose(derived_time, time)
    assert np.max(np.abs(acceleration[10:-10] - 0.4)) < 0.01


def test_signed_imu_acceleration_onset_is_detected_for_both_directions():
    for command_value in (0.10, -0.10):
        time, command, response = make_step(
            command=command_value, delay=0.08, tau=0.28, noise=0.0
        )
        direction = 1.0 if command_value > 0.0 else -1.0
        acceleration = np.zeros_like(response)
        rising = (time >= 0.08) & (time < 4.08)
        acceleration[rising] = (
            direction
            * abs(command_value)
            / 0.28
            * np.exp(-(time[rising] - 0.08) / 0.28)
        )
        falling = time >= 4.08
        acceleration[falling] = -direction * (
            abs(command_value)
            * (1.0 - math.exp(-4.0 / 0.28))
            / 0.28
            * np.exp(-(time[falling] - 4.08) / 0.28)
        )
        step = core.find_command_step(time, command)
        result = core.analyze_acceleration_onset(
            time,
            acceleration,
            step,
            onset_floor=0.02,
            sustain_sec=0.04,
        )
        assert result["valid"]
        assert 0.07 <= result["onset_delay_sec"] <= 0.11
        assert result["braking_valid"]


def analyzer_args():
    return SimpleNamespace(
        minimum_command=None,
        minimum_step_duration=2.0,
        mocap_median_window=3,
        mocap_velocity_window_sec=0.12,
        imu_median_window=3,
        odom_median_window=3,
        baseline_sec=1.5,
        steady_sec=0.6,
        guard_sec=0.10,
        onset_noise_multiplier=6.0,
        onset_fraction=0.02,
        sustain_sec=0.06,
        linear_mocap_onset_floor=0.004,
        linear_odom_onset_floor=0.005,
        angular_mocap_onset_floor=0.008,
        angular_imu_onset_floor=0.005,
        angular_odom_onset_floor=0.008,
        acceleration_window_sec=0.12,
        acceleration_median_window=3,
        imu_acceleration_window_sec=0.08,
        imu_acceleration_median_window=3,
        acceleration_onset_sustain_sec=0.04,
        linear_imu_accel_onset_floor=0.02,
        imu_forward_axis="x",
        imu_forward_sign=1.0,
        fopdt_max_delay_sec=1.20,
        fopdt_min_tau_sec=0.01,
        fopdt_max_tau_sec=2.50,
        fopdt_tau_count=64,
        fopdt_post_sec=0.0,
        fopdt_min_r2=0.80,
        fopdt_near_optimal_fraction=0.01,
    )


def publisher_args(arm_token, **overrides):
    values = dict(
        arm_token=arm_token,
        trial_contract="low_speed_identification",
        hardware_test_id=None,
        hardware_accel_limit_arm_token=None,
        axis="linear",
        command_value=0.10,
        pre_sec=3.0,
        step_sec=4.0,
        post_sec=3.0,
        rate_hz=50.0,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_publisher_requires_explicit_arm_token():
    try:
        publisher.validate_args(publisher_args(None))
    except ValueError as exc:
        assert "arm-token" in str(exc)
    else:
        raise AssertionError("publisher accepted motion without an arm token")

    publisher.validate_args(publisher_args(publisher.REQUIRED_ARM_TOKEN))


def test_hardware_accel_limit_publisher_contract_is_frozen():
    for test_id, command_value, step_sec in (
        ("H01", 0.80, 2.0),
        ("H02", 1.50, 2.0),
        ("H03", 3.00, 1.0),
    ):
        valid = publisher_args(
            publisher.REQUIRED_ARM_TOKEN,
            trial_contract="hardware_accel_limit",
            hardware_test_id=test_id,
            hardware_accel_limit_arm_token=publisher.HARDWARE_ACCEL_LIMIT_ARM_TOKEN,
            command_value=command_value,
            step_sec=step_sec,
            post_sec=4.0,
        )
        publisher.validate_args(valid)

        for overrides, expected in (
            ({"hardware_accel_limit_arm_token": None}, "hardware-accel-limit-arm-token"),
            ({"command_value": command_value + 0.01}, "is frozen to"),
            ({"axis": "angular"}, "linear axis"),
            ({"step_sec": step_sec + 0.5}, "step-sec="),
        ):
            candidate = SimpleNamespace(**vars(valid))
            for name, value in overrides.items():
                setattr(candidate, name, value)
            try:
                publisher.validate_args(candidate)
            except ValueError as exc:
                assert expected in str(exc)
            else:
                raise AssertionError(
                    "hardware_accel_limit accepted invalid override {}".format(overrides)
                )

    missing_id = publisher_args(
        publisher.REQUIRED_ARM_TOKEN,
        trial_contract="hardware_accel_limit",
        hardware_accel_limit_arm_token=publisher.HARDWARE_ACCEL_LIMIT_ARM_TOKEN,
        command_value=0.80,
        step_sec=2.0,
        post_sec=4.0,
    )
    try:
        publisher.validate_args(missing_id)
    except ValueError as exc:
        assert "hardware-test-id" in str(exc)
    else:
        raise AssertionError("hardware_accel_limit accepted a missing hardware test ID")

    low_speed_with_id = publisher_args(
        publisher.REQUIRED_ARM_TOKEN, hardware_test_id="H01"
    )
    try:
        publisher.validate_args(low_speed_with_id)
    except ValueError as exc:
        assert "only valid" in str(exc)
    else:
        raise AssertionError("low-speed contract accepted a hardware test ID")


def test_analyzer_parser_accepts_custom_command_and_odom_topics():
    args = analyzer.build_parser().parse_args(
        [
            "trial.bag",
            "--axis",
            "linear",
            "--cmd-topic",
            "/custom_cmd",
            "--odom-topic",
            "/custom_odom",
        ]
    )
    assert args.cmd_topic == "/custom_cmd"
    assert args.odom_topic == "/custom_odom"
    assert args.protocol_id == "MOCAP_VELOCITY_STEP_V2"
    assert args.acceptance_mode == "full_identification"
    assert args.fopdt_post_sec == 0.0


def test_acceleration_capability_acceptance_does_not_require_fopdt():
    sensor = {
        "valid": True,
        "braking_valid": True,
        "fopdt": {"valid": False},
        "acceleration_capability": {"valid": True, "braking_valid": True},
    }
    result = {
        "sensors": {"mocap": dict(sensor), "odom": dict(sensor)},
        "acceleration_crosscheck": {
            "imu": {"valid": True, "braking_valid": True}
        },
    }
    assert analyzer.analysis_complete(result, "linear", "acceleration_capability")
    assert not analyzer.analysis_complete(result, "linear", "full_identification")


def test_one_second_full_scale_pulse_produces_capability_evidence():
    time, command, response = make_step(
        command=3.0, delay=0.08, tau=0.80, noise=0.0, duration=1.0
    )
    epoch_time = time + 150.0
    position = integrate_signal(time, response)
    acceleration = np.gradient(response, time)
    zeros = np.zeros_like(response)
    args = analyzer_args()
    args.minimum_step_duration = 0.8
    args.steady_sec = 0.25
    result = analyzer.analyze_timebase(
        list(zip(epoch_time, command, zeros)),
        list(zip(epoch_time, position, zeros, zeros)),
        list(zip(epoch_time, zeros, acceleration, zeros, 9.8 + zeros)),
        list(zip(epoch_time, response, zeros)),
        "linear",
        args,
    )
    assert result["command"]["duration_sec"] >= 0.99
    assert analyzer.analysis_complete(result, "linear", "acceleration_capability")


def test_rise_fopdt_excludes_asymmetric_fast_braking():
    time, command, response = make_step(delay=0.35, tau=0.40, noise=0.0)
    response[time >= 4.01] = 0.0
    epoch_time = time + 50.0
    yaw = integrate_signal(time, response)
    zeros = np.zeros_like(response)
    result = analyzer.analyze_timebase(
        list(zip(epoch_time, zeros, command)),
        list(zip(epoch_time, zeros, zeros, yaw)),
        list(zip(epoch_time, response, zeros, zeros, zeros)),
        list(zip(epoch_time, zeros, response)),
        "angular",
        analyzer_args(),
    )
    for sensor in result["sensors"].values():
        fit = sensor["fopdt"]
        assert fit["fit_window"]["definition"] == "command_on_rise_only"
        assert fit["identifiable"]
        assert abs(fit["delay_sec"] - 0.35) < 0.03
        assert abs(fit["tau_sec"] - 0.40) < 0.06


def integrate_signal(time, velocity):
    position = np.zeros_like(velocity)
    position[1:] = np.cumsum(
        0.5 * (velocity[1:] + velocity[:-1]) * np.diff(time)
    )
    return position


def test_angular_bag_analyzer_reports_three_sensors_without_ros():
    time, command, response = make_step()
    epoch_time = time + 100.0
    yaw = integrate_signal(time, response)
    result = analyzer.analyze_timebase(
        list(zip(epoch_time, np.zeros_like(command), command)),
        list(zip(epoch_time, np.zeros_like(yaw), np.zeros_like(yaw), yaw)),
        list(zip(epoch_time, response, np.zeros_like(response), np.zeros_like(response), np.zeros_like(response))),
        list(zip(epoch_time, np.zeros_like(response), response)),
        "angular",
        analyzer_args(),
    )
    sensors = result["sensors"]
    assert set(sensors) == {"mocap", "imu", "odom"}
    assert all(sensor["valid"] for sensor in sensors.values())
    assert all(sensor["braking_valid"] for sensor in sensors.values())
    assert abs(sensors["mocap"]["t90_sec"] - sensors["imu"]["t90_sec"]) < 0.03
    assert abs(sensors["odom"]["t90_sec"] - sensors["imu"]["t90_sec"]) < 0.03
    assert sensors["mocap"]["rotation_after_command_deg"] > 0.0
    assert all(sensor["fopdt"]["valid"] for sensor in sensors.values())
    assert abs(sensors["mocap"]["fopdt"]["delay_sec"] - 0.07) < 0.03
    assert abs(sensors["mocap"]["fopdt"]["tau_sec"] - 0.50) < 0.08
    assert all(sensor["acceleration_capability"]["valid"] for sensor in sensors.values())

    with tempfile.TemporaryDirectory() as directory:
        paths = analyzer.make_separated_plots(Path(directory), result)
        assert len(paths) == 11
        assert all(path.is_file() for path in paths)


def test_linear_bag_analyzer_reports_acceleration_and_stopping_distance():
    for command_value in (0.10, -0.10):
        time, command, response = make_step(
            command=command_value, delay=0.08, tau=0.28
        )
        epoch_time = time + 200.0
        x = integrate_signal(time, response)
        acceleration = np.gradient(response, time)
        zeros = np.zeros_like(response)
        result = analyzer.analyze_timebase(
            list(zip(epoch_time, command, zeros)),
            list(zip(epoch_time, x, zeros, zeros)),
            list(zip(epoch_time, zeros, acceleration, zeros, 9.8 + zeros)),
            list(zip(epoch_time, response, zeros)),
            "linear",
            analyzer_args(),
        )
        sensors = result["sensors"]
        assert set(sensors) == {"mocap", "odom"}
        assert sensors["mocap"]["valid"]
        assert sensors["mocap"]["braking_valid"]
        assert sensors["odom"]["valid"]
        assert sensors["odom"]["braking_valid"]
        assert abs(
            sensors["mocap"]["effective_acceleration"]
            - sensors["odom"]["effective_acceleration"]
        ) < 0.005
        assert 0.015 < sensors["mocap"]["stopping_distance_after_command_m"] < 0.05
        assert sensors["mocap"]["maximum_lateral_drift_m"] < 1e-9
        assert sensors["mocap"]["fopdt"]["valid"]
        assert sensors["odom"]["fopdt"]["valid"]
        assert result["acceleration_crosscheck"]["imu"]["valid"]
        with tempfile.TemporaryDirectory() as directory:
            paths = analyzer.make_separated_plots(Path(directory), result)
            assert len(paths) == 9
            assert all(path.is_file() for path in paths)


def test_matrix_summary_keeps_one_direction_and_metric_per_plot():
    time, command, response = make_step()
    epoch_time = time + 300.0
    yaw = integrate_signal(time, response)
    zeros = np.zeros_like(response)
    result = analyzer.analyze_timebase(
        list(zip(epoch_time, zeros, command)),
        list(zip(epoch_time, zeros, zeros, yaw)),
        list(zip(epoch_time, response, zeros, zeros, zeros)),
        list(zip(epoch_time, zeros, response)),
        "angular",
        analyzer_args(),
    )
    payload = {
        "protocol_id": matrix_summary.EXPECTED_PROTOCOL,
        "axis": "angular",
        "bag": "/tmp/synthetic.bag",
        "bag_time": analyzer.serializable_timebase(result),
    }
    row = matrix_summary.extract_row(Path("synthetic.json"), payload)
    assert row["axis"] == "angular"
    assert row["direction"] == "left"
    assert abs(row["command_magnitude"] - 0.2) < 1e-12
    groups = matrix_summary.aggregate_rows([row])
    assert len(groups) == 1
    assert groups[0]["trial_count"] == 1
    with tempfile.TemporaryDirectory() as directory:
        paths = matrix_summary.make_plots(Path(directory), [row])
        assert len(paths) == 8
        assert all(path.name.startswith("angular_left_") for path in paths)
        assert all(path.is_file() for path in paths)
