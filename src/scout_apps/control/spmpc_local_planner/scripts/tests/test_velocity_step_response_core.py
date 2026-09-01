#!/usr/bin/env python3

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

import velocity_step_response_core as core  # noqa: E402
import analyze_mocap_velocity_step as analyzer  # noqa: E402
import publish_mocap_velocity_step as publisher  # noqa: E402


def make_step(command=0.2, delay=0.07, tau=0.50, noise=0.0005):
    # Keep four seconds after the zero command so stop detection is exercised,
    # rather than testing only the rising edge.
    time = np.arange(-3.0, 8.0, 0.01)
    command_values = np.zeros_like(time)
    command_values[(time >= 0.0) & (time < 4.0)] = command
    direction = 1.0 if command > 0.0 else -1.0
    elapsed = np.maximum(0.0, time - delay)
    response = direction * abs(command) * (1.0 - np.exp(-elapsed / tau))
    response[time < delay] = 0.0
    stop_value = abs(command) * (1.0 - math.exp(-(4.0 - delay) / tau))
    after = time >= 4.0
    response[after] = direction * stop_value * np.exp(-(time[after] - 4.0) / tau)
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


def analyzer_args():
    return SimpleNamespace(
        minimum_command=None,
        minimum_step_duration=2.0,
        mocap_median_window=3,
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
    )


def publisher_args(arm_token):
    return SimpleNamespace(
        arm_token=arm_token,
        axis="linear",
        command_value=0.10,
        pre_sec=3.0,
        step_sec=4.0,
        post_sec=3.0,
        rate_hz=50.0,
    )


def test_publisher_requires_explicit_arm_token():
    try:
        publisher.validate_args(publisher_args(None))
    except ValueError as exc:
        assert "arm-token" in str(exc)
    else:
        raise AssertionError("publisher accepted motion without an arm token")

    publisher.validate_args(publisher_args(publisher.REQUIRED_ARM_TOKEN))


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
        list(zip(epoch_time, response)),
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


def test_linear_bag_analyzer_reports_acceleration_and_stopping_distance():
    for command_value in (0.10, -0.10):
        time, command, response = make_step(
            command=command_value, delay=0.08, tau=0.28
        )
        epoch_time = time + 200.0
        x = integrate_signal(time, response)
        zeros = np.zeros_like(response)
        result = analyzer.analyze_timebase(
            list(zip(epoch_time, command, zeros)),
            list(zip(epoch_time, x, zeros, zeros)),
            list(zip(epoch_time, zeros)),
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
