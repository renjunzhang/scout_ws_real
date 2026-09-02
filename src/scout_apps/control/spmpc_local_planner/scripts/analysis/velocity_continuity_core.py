#!/usr/bin/env python3
"""Numerical helpers for continuous chassis velocity-profile tests."""

import math

import numpy as np


PROFILES = ("constant_accel", "trapezoidal_velocity", "linear_accel")


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def profile_scale(profile, progress):
    """Return normalized ramp velocity for one of the frozen profiles."""

    if profile not in PROFILES:
        raise ValueError("unknown profile: {}".format(profile))
    progress = np.clip(np.asarray(progress, dtype=float), 0.0, 1.0)
    if profile in ("constant_accel", "trapezoidal_velocity"):
        return progress
    return progress * progress


def profile_acceleration_scale(profile, progress):
    """Return d(profile_scale)/d(progress)."""

    if profile not in PROFILES:
        raise ValueError("unknown profile: {}".format(profile))
    progress = np.clip(np.asarray(progress, dtype=float), 0.0, 1.0)
    if profile in ("constant_accel", "trapezoidal_velocity"):
        return np.ones_like(progress)
    return 2.0 * progress


def phase_value(profile, progress, target, ramp_down=False):
    """Velocity command during a ramp-up or its continuous ramp-down."""

    target = _finite(target, "target")
    scale = profile_scale(profile, progress)
    if ramp_down:
        scale = 1.0 - scale
    return target * scale


def unique_samples(times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    finite = np.isfinite(times) & np.isfinite(values)
    times = times[finite]
    values = values[finite]
    if not times.size:
        return np.empty(0), np.empty(0)
    order = np.argsort(times, kind="mergesort")
    times = times[order]
    values = values[order]
    output_time = []
    output_value = []
    for stamp, value in zip(times, values):
        if output_time and stamp <= output_time[-1] + 1e-12:
            output_time[-1] = float(stamp)
            output_value[-1] = float(value)
        else:
            output_time.append(float(stamp))
            output_value.append(float(value))
    return np.asarray(output_time), np.asarray(output_value)


def percentile_abs(values, percentile=95.0):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return float(np.percentile(np.abs(values), float(percentile)))


def fit_effective_lag(
    command_time,
    command_value,
    response_time,
    response_value,
    active_start,
    active_stop,
    *,
    max_lag_sec=1.2,
    lag_step_sec=0.005,
):
    """Fit one effective lag plus affine gain over the complete profile.

    This is an alignment aid for continuity residuals, not a replacement for
    the velocity-step FOPDT identification.
    """

    command_time, command_value = unique_samples(command_time, command_value)
    response_time, response_value = unique_samples(response_time, response_value)
    active_start = _finite(active_start, "active_start")
    active_stop = _finite(active_stop, "active_stop")
    max_lag_sec = _finite(max_lag_sec, "max_lag_sec")
    lag_step_sec = _finite(lag_step_sec, "lag_step_sec")
    if command_time.size < 20 or response_time.size < 20:
        raise ValueError("too few samples for effective-lag fit")
    if active_stop <= active_start:
        raise ValueError("active_stop must be after active_start")
    if max_lag_sec < 0.0 or lag_step_sec <= 0.0:
        raise ValueError("invalid effective-lag grid")

    best = None
    for lag in np.arange(0.0, max_lag_sec + 0.5 * lag_step_sec, lag_step_sec):
        mask = (
            (response_time >= active_start + lag)
            & (response_time <= active_stop + lag)
            & (response_time - lag >= command_time[0])
            & (response_time - lag <= command_time[-1])
        )
        if np.count_nonzero(mask) < 30:
            continue
        measured = response_value[mask]
        delayed_command = np.interp(response_time[mask] - lag, command_time, command_value)
        design = np.column_stack((np.ones(delayed_command.size), delayed_command))
        coefficients, _, _, _ = np.linalg.lstsq(design, measured, rcond=None)
        offset, gain = (float(coefficients[0]), float(coefficients[1]))
        if not math.isfinite(gain) or gain <= 0.0:
            continue
        predicted = offset + gain * delayed_command
        residual = measured - predicted
        rmse = float(np.sqrt(np.mean(residual * residual)))
        if best is None or rmse < best["rmse"]:
            variance = float(np.sum((measured - np.mean(measured)) ** 2))
            squared_error = float(np.sum(residual * residual))
            r2 = 1.0 - squared_error / variance if variance > 1e-15 else 0.0
            best = {
                "lag_sec": float(lag),
                "offset": offset,
                "gain": gain,
                "rmse": rmse,
                "r2": float(r2),
                "sample_count": int(measured.size),
                "lag_step_sec": lag_step_sec,
                "max_lag_sec": max_lag_sec,
            }
    if best is None:
        raise ValueError("effective-lag fit found no valid candidate")
    return best


def polynomial_shape_fit(time, directed_velocity, start, stop, degree):
    time, directed_velocity = unique_samples(time, directed_velocity)
    mask = (time >= float(start)) & (time <= float(stop))
    if np.count_nonzero(mask) < max(10, degree + 3):
        raise ValueError("too few ramp samples for shape fit")
    elapsed = time[mask] - float(start)
    measured = directed_velocity[mask]
    coefficients = np.polyfit(elapsed, measured, int(degree))
    predicted = np.polyval(coefficients, elapsed)
    residual = measured - predicted
    variance = float(np.sum((measured - np.mean(measured)) ** 2))
    squared_error = float(np.sum(residual * residual))
    r2 = 1.0 - squared_error / variance if variance > 1e-15 else 0.0
    return {
        "degree": int(degree),
        "coefficients_descending": [float(value) for value in coefficients],
        "r2": float(r2),
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "sample_count": int(measured.size),
    }


def count_sign_flips(values, deadband):
    values = np.asarray(values, dtype=float)
    deadband = abs(float(deadband))
    signs = np.sign(values[np.abs(values) > deadband])
    if signs.size < 2:
        return 0
    return int(np.count_nonzero(signs[1:] != signs[:-1]))


def continuity_metrics(
    command_time,
    command_value,
    response_time,
    response_value,
    phases,
    profile,
    target_value,
    axis,
    *,
    resample_dt_sec=0.02,
    max_lag_sec=1.2,
):
    """Measure speed continuity after aligning one measured sensor."""

    if profile not in PROFILES:
        raise ValueError("unknown profile: {}".format(profile))
    if axis not in ("linear", "angular"):
        raise ValueError("axis must be linear or angular")
    target_value = _finite(target_value, "target_value")
    if target_value == 0.0:
        raise ValueError("target_value must be nonzero")
    resample_dt_sec = _finite(resample_dt_sec, "resample_dt_sec")
    if resample_dt_sec <= 0.0:
        raise ValueError("resample_dt_sec must be positive")

    command_time, command_value = unique_samples(command_time, command_value)
    response_time, response_value = unique_samples(response_time, response_value)
    active_start = float(phases["ramp_up_start"])
    ramp_up_stop = float(phases["ramp_up_stop"])
    hold_start = float(phases["hold_start"])
    hold_stop = float(phases["hold_stop"])
    active_stop = float(phases["ramp_down_stop"])
    ramp_duration = ramp_up_stop - active_start
    if ramp_duration <= 0.0 or active_stop <= hold_stop:
        raise ValueError("invalid phase boundaries")

    alignment = fit_effective_lag(
        command_time,
        command_value,
        response_time,
        response_value,
        active_start,
        active_stop,
        max_lag_sec=max_lag_sec,
    )
    lag = alignment["lag_sec"]
    grid_start = active_start + lag
    grid_stop = min(active_stop + lag, response_time[-1])
    grid = np.arange(grid_start, grid_stop + 0.25 * resample_dt_sec, resample_dt_sec)
    if grid.size < 30:
        raise ValueError("too few uniformly resampled continuity samples")
    measured = np.interp(grid, response_time, response_value)
    delayed_command = np.interp(grid - lag, command_time, command_value)
    predicted = alignment["offset"] + alignment["gain"] * delayed_command
    residual = measured - predicted
    delta_measured = np.diff(measured)
    delta_predicted = np.diff(predicted)
    delta_residual = delta_measured - delta_predicted
    midpoint = 0.5 * (grid[1:] + grid[:-1])
    acceleration = delta_measured / resample_dt_sec

    direction = 1.0 if target_value > 0.0 else -1.0
    directed_velocity = direction * response_value
    directed_acceleration = direction * acceleration
    ramp_mask = (midpoint >= active_start + lag) & (midpoint <= ramp_up_stop + lag)
    expected_peak_acceleration = (
        abs(target_value) / ramp_duration
        if profile in ("constant_accel", "trapezoidal_velocity")
        else 2.0 * abs(target_value) / ramp_duration
    )
    acceleration_deadband = max(1e-6, 0.15 * expected_peak_acceleration)
    ramp_acceleration = directed_acceleration[ramp_mask]
    drop_threshold = 0.005 if axis == "linear" else 0.010
    significant_drop = direction * delta_measured[ramp_mask] < -drop_threshold

    shape_degree = (
        1 if profile in ("constant_accel", "trapezoidal_velocity") else 2
    )
    shape = polynomial_shape_fit(
        response_time,
        directed_velocity,
        active_start + lag,
        ramp_up_stop + lag,
        shape_degree,
    )
    if profile in ("constant_accel", "trapezoidal_velocity"):
        shape["estimated_acceleration"] = float(shape["coefficients_descending"][0])
    else:
        quadratic, linear, _ = shape["coefficients_descending"]
        shape["estimated_jerk"] = float(2.0 * quadratic)
        shape["estimated_end_acceleration"] = float(
            linear + 2.0 * quadratic * ramp_duration
        )

    plateau_guard = min(0.30, 0.20 * max(0.0, hold_stop - hold_start))
    plateau_mask = (
        (response_time >= hold_start + lag + plateau_guard)
        & (response_time <= hold_stop + lag - 0.10)
    )
    if np.count_nonzero(plateau_mask) < 8:
        raise ValueError("too few hold samples for continuity analysis")
    plateau = direction * response_value[plateau_mask]
    plateau_center = float(np.median(plateau))
    plateau_residual = plateau - plateau_center

    output = {
        "effective_alignment": alignment,
        "resample_dt_sec": resample_dt_sec,
        "sample_count": int(grid.size),
        "velocity_residual_rmse": float(np.sqrt(np.mean(residual * residual))),
        "velocity_residual_p95_abs": percentile_abs(residual),
        "delta_velocity_p95_abs": percentile_abs(delta_measured),
        "delta_velocity_max_abs": percentile_abs(delta_measured, 100.0),
        "delta_velocity_residual_p95_abs": percentile_abs(delta_residual),
        "delta_velocity_residual_max_abs": percentile_abs(delta_residual, 100.0),
        "significant_drop_threshold": drop_threshold,
        "ramp_significant_drop_count": int(np.count_nonzero(significant_drop)),
        "ramp_negative_acceleration_fraction": (
            float(np.mean(ramp_acceleration < -acceleration_deadband))
            if ramp_acceleration.size
            else None
        ),
        "ramp_acceleration_sign_flip_count": count_sign_flips(
            ramp_acceleration, acceleration_deadband
        ),
        "acceleration_deadband": acceleration_deadband,
        "expected_peak_acceleration": expected_peak_acceleration,
        "ramp_velocity_shape": shape,
        "hold": {
            "sample_count": int(plateau.size),
            "median_speed": plateau_center,
            "speed_std": float(np.std(plateau)),
            "speed_p95_deviation": percentile_abs(plateau_residual),
            "speed_peak_to_peak": float(np.max(plateau) - np.min(plateau)),
            "normalized_std": float(np.std(plateau) / abs(target_value)),
        },
    }
    output["plot_signals"] = {
        "uniform_time": grid,
        "measured_velocity": measured,
        "predicted_velocity": predicted,
        "delayed_command": delayed_command,
        "delta_time": midpoint,
        "delta_measured": delta_measured,
        "delta_predicted": delta_predicted,
    }
    return output
