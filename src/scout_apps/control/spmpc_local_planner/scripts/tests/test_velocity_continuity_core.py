#!/usr/bin/env python3

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

import publish_mocap_velocity_continuity as publisher  # noqa: E402
import velocity_continuity_core as core  # noqa: E402


PHASES = {
    "ramp_up_start": 0.0,
    "ramp_up_stop": 3.0,
    "hold_start": 3.02,
    "hold_stop": 5.0,
    "ramp_down_start": 5.02,
    "ramp_down_stop": 8.02,
}


def make_profile(profile, target=0.10, dt=0.01):
    time = np.arange(-2.0, 11.0, dt)
    command = np.zeros_like(time)
    ramp_up = (time >= 0.0) & (time <= 3.0)
    command[ramp_up] = core.phase_value(profile, time[ramp_up] / 3.0, target)
    hold = (time > 3.0) & (time <= 5.02)
    command[hold] = target
    ramp_down = (time > 5.02) & (time <= 8.02)
    command[ramp_down] = core.phase_value(
        profile, (time[ramp_down] - 5.02) / 3.0, target, ramp_down=True
    )
    return time, command


def delayed_response(time, command, delay=0.18, gain=0.98, noise=0.00005):
    value = gain * np.interp(time - delay, time, command, left=0.0, right=0.0)
    value += noise * np.sin(2.0 * np.pi * 3.7 * time)
    return value


def test_frozen_profile_shapes_and_trapezoid_alias():
    progress = np.asarray([0.0, 0.25, 0.5, 0.75, 1.0])
    constant = core.profile_scale("constant_accel", progress)
    trapezoid = core.profile_scale("trapezoidal_velocity", progress)
    linear_acceleration = core.profile_scale("linear_accel", progress)
    assert np.allclose(constant, progress)
    assert np.allclose(trapezoid, constant)
    assert np.allclose(linear_acceleration, progress * progress)
    assert core.phase_value("trapezoidal_velocity", 0.0, 0.1) == 0.0
    assert core.phase_value("trapezoidal_velocity", 1.0, 0.1) == 0.1
    assert core.phase_value(
        "trapezoidal_velocity", 1.0, 0.1, ramp_down=True
    ) == 0.0


def test_effective_lag_and_continuity_for_both_physical_profiles():
    for profile in ("trapezoidal_velocity", "linear_accel"):
        time, command = make_profile(profile)
        response = delayed_response(time, command)
        result = core.continuity_metrics(
            time,
            command,
            time,
            response,
            PHASES,
            profile,
            0.10,
            "linear",
        )
        assert abs(result["effective_alignment"]["lag_sec"] - 0.18) <= 0.006
        assert abs(result["effective_alignment"]["gain"] - 0.98) < 0.01
        assert result["delta_velocity_residual_p95_abs"] < 0.0001
        assert result["ramp_significant_drop_count"] == 0
        assert result["ramp_velocity_shape"]["r2"] > 0.999
        assert result["hold"]["speed_std"] < 0.0001


def test_speed_drop_is_visible_in_delta_velocity_metrics():
    time, command = make_profile("trapezoidal_velocity")
    response = delayed_response(time, command)
    response[(time >= 1.60) & (time < 1.68)] -= 0.025
    result = core.continuity_metrics(
        time,
        command,
        time,
        response,
        PHASES,
        "trapezoidal_velocity",
        0.10,
        "linear",
    )
    assert result["ramp_significant_drop_count"] >= 1
    assert result["delta_velocity_residual_max_abs"] > 0.005


def publisher_args(profile="trapezoidal_velocity", arm_token=None):
    return SimpleNamespace(
        arm_token=arm_token,
        axis="linear",
        profile=profile,
        target_value=0.10,
        pre_sec=3.0,
        ramp_up_sec=3.0,
        hold_sec=2.0,
        ramp_down_sec=3.0,
        post_sec=3.0,
        rate_hz=50.0,
    )


def test_publisher_requires_explicit_arm_and_accepts_both_profiles():
    try:
        publisher.validate_args(publisher_args())
    except ValueError as exc:
        assert "arm-token" in str(exc)
    else:
        raise AssertionError("publisher accepted motion without explicit arm")

    for profile in ("constant_accel", "trapezoidal_velocity", "linear_accel"):
        publisher.validate_args(
            publisher_args(profile=profile, arm_token=publisher.REQUIRED_ARM_TOKEN)
        )
