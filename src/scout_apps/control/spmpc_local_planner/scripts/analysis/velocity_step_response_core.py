#!/usr/bin/env python3
"""Numerical helpers for a stationary linear or angular velocity step test.

The functions in this module have no ROS dependency so the timing logic can be
tested with synthetic signals.  The reported onset is a *detectable motion*
time based on a frozen noise threshold, not a claim of zero-noise physical dead
time.
"""

import math

import numpy as np


def unique_samples(times, values):
    """Return finite, time-sorted samples with duplicate stamps collapsed."""

    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    finite = np.isfinite(times) & np.isfinite(values)
    times = times[finite]
    values = values[finite]
    if not times.size:
        return np.empty(0, dtype=float), np.empty(0, dtype=float)
    order = np.argsort(times, kind="mergesort")
    times = times[order]
    values = values[order]
    keep_time = []
    keep_value = []
    for stamp, value in zip(times, values):
        if keep_time and stamp <= keep_time[-1] + 1e-12:
            keep_time[-1] = float(stamp)
            keep_value[-1] = float(value)
        else:
            keep_time.append(float(stamp))
            keep_value.append(float(value))
    return np.asarray(keep_time), np.asarray(keep_value)


def centered_median(values, window=3):
    """Small robust filter with no net time shift on an offline signal."""

    values = np.asarray(values, dtype=float)
    window = max(1, int(window))
    if window % 2 == 0:
        window += 1
    if window <= 1 or values.size < 3:
        return values.copy()
    half = window // 2
    padded = np.pad(values, (half, half), mode="edge")
    return np.asarray(
        [np.median(padded[index:index + window]) for index in range(values.size)],
        dtype=float,
    )


def derive_yaw_rate(times, yaw, median_window=3):
    """Differentiate unwrapped yaw and timestamp each rate at its interval midpoint."""

    times, yaw = unique_samples(times, yaw)
    if times.size < 5:
        raise ValueError("at least five mocap pose samples are required")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("mocap timestamps must be strictly increasing")
    unwrapped = np.unwrap(yaw)
    delta_time = np.diff(times)
    midpoint_time = 0.5 * (times[1:] + times[:-1])
    omega = np.diff(unwrapped) / delta_time
    return midpoint_time, centered_median(omega, median_window)


def find_command_step(times, command, minimum_command=0.02, minimum_duration=1.0):
    """Find one zero -> nonzero -> zero velocity command step."""

    times, command = unique_samples(times, command)
    active = np.abs(command) >= float(minimum_command)
    starts = np.flatnonzero(active & np.r_[True, ~active[:-1]])
    if starts.size != 1:
        raise ValueError(
            "expected exactly one velocity command step, found {}".format(starts.size)
        )
    start_index = int(starts[0])
    stop_candidates = np.flatnonzero(~active[start_index + 1:])
    if not stop_candidates.size:
        raise ValueError("command step has no recorded return to zero")
    stop_index = start_index + 1 + int(stop_candidates[0])
    start_time = float(times[start_index])
    stop_time = float(times[stop_index])
    if stop_time - start_time < float(minimum_duration):
        raise ValueError("velocity command step is shorter than {:.3f} s".format(minimum_duration))
    active_values = command[start_index:stop_index]
    level = float(np.median(active_values))
    if abs(level) < float(minimum_command):
        raise ValueError("velocity command level is too small")
    opposite = np.count_nonzero(np.sign(active_values) == -np.sign(level))
    if opposite:
        raise ValueError("command changes direction inside the step")
    return {
        "start_time_sec": start_time,
        "stop_time_sec": stop_time,
        "duration_sec": stop_time - start_time,
        "command_value": level,
        "sample_count": int(stop_index - start_index),
    }


def robust_noise_sigma(values):
    values = np.asarray(values, dtype=float)
    center = float(np.median(values))
    mad = float(np.median(np.abs(values - center)))
    return 1.4826 * mad


def first_sustained_crossing(
    times,
    values,
    threshold,
    start_time,
    stop_time,
    sustain_sec=0.06,
    required_fraction=0.80,
):
    """Find the first upward crossing that remains present for a short window."""

    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    indices = np.flatnonzero((times >= start_time) & (times <= stop_time))
    if indices.size < 2:
        return None
    for index in indices:
        if values[index] < threshold:
            continue
        sustain_end = times[index] + float(sustain_sec)
        last = int(np.searchsorted(times, sustain_end, side="left"))
        if last >= times.size or times[last] > stop_time:
            continue
        window = values[index:last + 1]
        if window.size < 2:
            continue
        fraction = float(np.mean(window >= threshold))
        if fraction + 1e-12 < float(required_fraction):
            continue
        if index == 0 or values[index - 1] >= threshold:
            return max(float(start_time), float(times[index]))
        previous_value = float(values[index - 1])
        current_value = float(values[index])
        delta = current_value - previous_value
        fraction_to_threshold = 0.0
        if abs(delta) > 1e-12:
            fraction_to_threshold = (float(threshold) - previous_value) / delta
        fraction_to_threshold = min(1.0, max(0.0, fraction_to_threshold))
        return float(
            times[index - 1]
            + fraction_to_threshold * (times[index] - times[index - 1])
        )
    return None


def first_sustained_below(
    times,
    values,
    threshold,
    start_time,
    stop_time,
    sustain_sec=0.06,
    required_fraction=0.80,
):
    """Find the first downward crossing that remains below a threshold."""

    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    indices = np.flatnonzero((times >= start_time) & (times <= stop_time))
    if indices.size < 2:
        return None
    for index in indices:
        if values[index] > threshold:
            continue
        sustain_end = times[index] + float(sustain_sec)
        last = int(np.searchsorted(times, sustain_end, side="left"))
        if last >= times.size or times[last] > stop_time:
            continue
        window = values[index:last + 1]
        if window.size < 2:
            continue
        fraction = float(np.mean(window <= threshold))
        if fraction + 1e-12 < float(required_fraction):
            continue
        if index == 0 or values[index - 1] <= threshold:
            return max(float(start_time), float(times[index]))
        previous_value = float(values[index - 1])
        current_value = float(values[index])
        delta = current_value - previous_value
        fraction_to_threshold = 0.0
        if abs(delta) > 1e-12:
            fraction_to_threshold = (float(threshold) - previous_value) / delta
        fraction_to_threshold = min(1.0, max(0.0, fraction_to_threshold))
        return float(
            times[index - 1]
            + fraction_to_threshold * (times[index] - times[index - 1])
        )
    return None


def analyze_step_response(
    times,
    values,
    command_step,
    *,
    baseline_sec=1.5,
    steady_sec=0.6,
    guard_sec=0.10,
    onset_floor=0.005,
    onset_noise_multiplier=6.0,
    onset_fraction=0.02,
    sustain_sec=0.06,
):
    """Measure onset, rise time, effective acceleration, and braking.

    Baseline and steady-state levels are estimated with medians.  Response is
    projected into the commanded direction, allowing the same code to handle
    either command sign without taking an absolute value of noisy samples.
    """

    times, values = unique_samples(times, values)
    if times.size < 20:
        return {"valid": False, "reason": "too few response samples"}
    start = float(command_step["start_time_sec"])
    stop = float(command_step["stop_time_sec"])
    command = float(command_step["command_value"])
    baseline_mask = (
        (times >= start - float(baseline_sec))
        & (times <= start - float(guard_sec))
    )
    steady_mask = (
        (times >= stop - float(steady_sec))
        & (times <= stop - float(guard_sec))
    )
    if np.count_nonzero(baseline_mask) < 8:
        return {"valid": False, "reason": "insufficient stationary baseline"}
    if np.count_nonzero(steady_mask) < 5:
        return {"valid": False, "reason": "insufficient steady-state samples"}

    baseline_values = values[baseline_mask]
    steady_values = values[steady_mask]
    baseline = float(np.median(baseline_values))
    steady = float(np.median(steady_values))
    noise_sigma = robust_noise_sigma(baseline_values)
    direction = 1.0 if command > 0.0 else -1.0
    amplitude = direction * (steady - baseline)
    minimum_amplitude = max(
        4.0 * noise_sigma,
        2.0 * float(onset_floor),
        0.10 * abs(command),
    )
    if not math.isfinite(amplitude) or amplitude <= 0.0:
        return {
            "valid": False,
            "reason": "response sign mismatch or non-finite amplitude",
            "baseline_value": baseline,
            "steady_value": steady,
            "noise_sigma": noise_sigma,
            "signed_amplitude": amplitude,
            "minimum_amplitude": minimum_amplitude,
        }
    if amplitude <= minimum_amplitude:
        return {
            "valid": False,
            "reason": "stationary noise is too large or velocity excitation is insufficient",
            "baseline_value": baseline,
            "steady_value": steady,
            "noise_sigma": noise_sigma,
            "signed_amplitude": amplitude,
            "minimum_amplitude": minimum_amplitude,
        }

    projected = direction * (values - baseline)
    onset_threshold = max(
        float(onset_floor),
        float(onset_noise_multiplier) * noise_sigma,
        float(onset_fraction) * amplitude,
    )
    if onset_threshold >= 0.50 * amplitude:
        return {
            "valid": False,
            "reason": "stationary noise is too large for a reliable onset",
            "onset_threshold": onset_threshold,
            "signed_amplitude": amplitude,
        }

    search_stop = stop - float(guard_sec)
    crossing_times = {}
    for label, threshold in (
        ("onset", onset_threshold),
        ("t10", 0.10 * amplitude),
        ("t50", 0.50 * amplitude),
        ("t90", 0.90 * amplitude),
    ):
        crossing = first_sustained_crossing(
            times,
            projected,
            threshold,
            start,
            search_stop,
            sustain_sec=sustain_sec,
        )
        crossing_times[label] = crossing

    onset = crossing_times["onset"]
    t10 = crossing_times["t10"]
    t50 = crossing_times["t50"]
    t90 = crossing_times["t90"]
    if onset is None or t10 is None or t50 is None or t90 is None:
        missing = [name for name, value in crossing_times.items() if value is None]
        return {
            "valid": False,
            "reason": "missing sustained crossing: {}".format(", ".join(missing)),
            "baseline_value": baseline,
            "steady_value": steady,
            "noise_sigma": noise_sigma,
            "onset_threshold": onset_threshold,
            "signed_amplitude": amplitude,
        }

    rise_duration = t90 - t10
    if rise_duration <= 0.0:
        return {
            "valid": False,
            "reason": "non-positive t10-to-t90 rise duration",
        }

    result = {
        "valid": True,
        "sample_count": int(times.size),
        "baseline_sample_count": int(np.count_nonzero(baseline_mask)),
        "steady_sample_count": int(np.count_nonzero(steady_mask)),
        "baseline_value": baseline,
        "steady_value": steady,
        "signed_amplitude": amplitude,
        "gain": amplitude / abs(command),
        "noise_sigma": noise_sigma,
        "onset_threshold": onset_threshold,
        "onset_time_sec": onset,
        "t10_time_sec": t10,
        "t50_time_sec": t50,
        "t90_time_sec": t90,
        "onset_delay_sec": onset - start,
        "t10_sec": t10 - start,
        "t50_sec": t50 - start,
        "t90_sec": t90 - start,
        "rise_onset_to_90_sec": t90 - onset,
        "rise_10_to_90_sec": rise_duration,
        "effective_acceleration": 0.80 * amplitude / rise_duration,
        "sustain_sec": float(sustain_sec),
    }

    brake_crossings = {}
    brake_levels = (
        ("brake_onset", amplitude - onset_threshold),
        ("decel_t90", 0.90 * amplitude),
        ("decel_t50", 0.50 * amplitude),
        ("decel_t10", 0.10 * amplitude),
        ("stop", onset_threshold),
    )
    for label, threshold in brake_levels:
        brake_crossings[label] = first_sustained_below(
            times,
            projected,
            threshold,
            stop,
            float(times[-1]),
            sustain_sec=sustain_sec,
        )
    missing_brake = [
        name for name, value in brake_crossings.items() if value is None
    ]
    if missing_brake:
        result.update(
            {
                "braking_valid": False,
                "braking_reason": "missing sustained crossing: {}".format(
                    ", ".join(missing_brake)
                ),
            }
        )
        return result

    brake_onset = brake_crossings["brake_onset"]
    decel_t90 = brake_crossings["decel_t90"]
    decel_t50 = brake_crossings["decel_t50"]
    decel_t10 = brake_crossings["decel_t10"]
    response_stop = brake_crossings["stop"]
    decel_duration = decel_t10 - decel_t90
    if decel_duration <= 0.0:
        result.update(
            {
                "braking_valid": False,
                "braking_reason": "non-positive 90%-to-10% deceleration duration",
            }
        )
        return result

    result.update(
        {
            "braking_valid": True,
            "brake_onset_time_sec": brake_onset,
            "decel_t90_time_sec": decel_t90,
            "decel_t50_time_sec": decel_t50,
            "decel_t10_time_sec": decel_t10,
            "response_stop_time_sec": response_stop,
            "brake_onset_delay_sec": brake_onset - stop,
            "decel_t90_sec": decel_t90 - stop,
            "decel_t50_sec": decel_t50 - stop,
            "decel_t10_sec": decel_t10 - stop,
            "response_stop_delay_sec": response_stop - stop,
            "decel_90_to_10_sec": decel_duration,
            "effective_deceleration": 0.80 * amplitude / decel_duration,
        }
    )
    return result
