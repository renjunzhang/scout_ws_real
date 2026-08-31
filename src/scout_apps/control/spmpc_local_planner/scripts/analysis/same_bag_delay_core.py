#!/usr/bin/env python3
"""Pure numerical helpers for same-bag actuator-delay identification.

This module deliberately has no ROS dependency.  The bag reader and report
writer live in ``analyze_same_bag_actuator_delay.py`` so the fit, validation,
and event-timing math can be unit tested with synthetic signals.
"""

import math
import statistics

import numpy as np


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
        "p05": float(np.percentile(array, 5.0)),
        "p95": float(np.percentile(array, 95.0)),
        "minimum": float(np.min(array)),
        "maximum": float(np.max(array)),
        "p95_abs": float(np.percentile(np.abs(array), 95.0)),
        "max_abs": float(np.max(np.abs(array))),
    }


def unique_rows(rows):
    if not rows:
        return np.empty((0, 0), dtype=float)
    ordered = sorted(rows, key=lambda row: row[0])
    output = []
    for row in ordered:
        numeric = tuple(float(value) for value in row)
        if output and numeric[0] <= output[-1][0] + 1e-12:
            output[-1] = numeric
        else:
            output.append(numeric)
    return np.asarray(output, dtype=float)


def zoh_resample(source_time, source_value, target_time):
    source_time = np.asarray(source_time, dtype=float)
    source_value = np.asarray(source_value, dtype=float)
    target_time = np.asarray(target_time, dtype=float)
    if source_time.size == 0:
        return np.full(target_time.shape, np.nan, dtype=float)
    indices = np.searchsorted(source_time, target_time, side="right") - 1
    indices = np.clip(indices, 0, source_time.size - 1)
    return source_value[indices]


def centered_moving_average(values, count):
    values = np.asarray(values, dtype=float)
    count = max(1, int(count))
    if count % 2 == 0:
        count += 1
    maximum = values.size if values.size % 2 else values.size - 1
    count = min(count, maximum)
    if count <= 1:
        return values.copy()
    half = count // 2
    padded = np.pad(values, (half, half), mode="edge")
    return np.convolve(padded, np.ones(count, dtype=float) / count, mode="valid")


def motion_window(time, command_v, command_omega, pad_sec=2.0):
    time = np.asarray(time, dtype=float)
    command_v = np.asarray(command_v, dtype=float)
    command_omega = np.asarray(command_omega, dtype=float)
    active = (np.abs(command_v) >= 0.01) | (np.abs(command_omega) >= 0.01)
    indices = np.flatnonzero(active)
    if not indices.size:
        return float(time[0]), float(time[-1])
    return (
        max(float(time[0]), float(time[indices[0]]) - float(pad_sec)),
        min(float(time[-1]), float(time[indices[-1]]) + float(pad_sec)),
    )


def simulate_fopdt(time, command, delay_sec, tau_sec):
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    delayed = np.interp(
        time - float(delay_sec),
        time,
        command,
        left=float(command[0]),
        right=float(command[-1]),
    )
    output = np.empty_like(delayed)
    output[0] = delayed[0]
    tau_sec = max(1e-9, float(tau_sec))
    for index in range(1, output.size):
        dt = max(0.0, float(time[index] - time[index - 1]))
        alpha = 1.0 - math.exp(-dt / tau_sec)
        output[index] = output[index - 1] + alpha * (delayed[index] - output[index - 1])
    return output


def model_metrics(response, predicted):
    response = np.asarray(response, dtype=float)
    predicted = np.asarray(predicted, dtype=float)
    mask = np.isfinite(response) & np.isfinite(predicted)
    if np.count_nonzero(mask) < 3:
        return {"valid": False, "reason": "insufficient finite samples"}
    response = response[mask]
    predicted = predicted[mask]
    residual = response - predicted
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    centered = response - np.mean(response)
    denominator = float(np.dot(centered, centered))
    residual_centered = residual - np.mean(residual)
    lag1_denominator = float(np.dot(residual_centered, residual_centered))
    lag1 = 0.0
    if residual_centered.size >= 3 and lag1_denominator > 1e-12:
        lag1 = float(
            np.dot(residual_centered[:-1], residual_centered[1:]) / lag1_denominator
        )
    return {
        "valid": True,
        "count": int(response.size),
        "rmse": rmse,
        "nrmse_by_std": rmse / max(float(np.std(response)), 1e-12),
        "r2": 1.0 - float(np.dot(residual, residual)) / max(denominator, 1e-12),
        "residual_bias": float(np.mean(residual)),
        "residual_lag1_correlation": lag1,
    }


def evaluate_fopdt(time, command, response, model):
    if not model or not model.get("valid", False):
        return {"valid": False, "reason": "invalid model"}
    basis = simulate_fopdt(
        time,
        command,
        float(model["delay_sec"]),
        float(model["tau_sec"]),
    )
    predicted = float(model["offset"]) + float(model["gain"]) * basis
    return model_metrics(response, predicted)


def evaluate_fixed_delay(time, command, response, delay_sec):
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    predicted = np.interp(
        time - float(delay_sec),
        time,
        command,
        left=float(command[0]),
        right=float(command[-1]),
    )
    return model_metrics(response, predicted)


def fit_fopdt_grid(
    time,
    command,
    response,
    max_delay_sec=0.60,
    min_tau_sec=0.01,
    max_tau_sec=1.50,
    tau_count=64,
    near_optimal_rmse_fraction=0.01,
):
    """Fit a first-order-plus-dead-time model on an explicit frozen grid.

    The state recursion is vectorized over every tau candidate.  Delay remains
    on the native resampling grid so the returned precision never exceeds the
    declared data grid.
    """

    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    response = np.asarray(response, dtype=float)
    finite = np.isfinite(time) & np.isfinite(command) & np.isfinite(response)
    time = time[finite]
    command = command[finite]
    response = response[finite]
    if time.size < 50 or np.std(command) < 1e-7 or np.std(response) < 1e-7:
        return {"valid": False, "reason": "insufficient samples or excitation"}
    dt = float(np.median(np.diff(time)))
    if not math.isfinite(dt) or dt <= 0.0:
        return {"valid": False, "reason": "invalid time grid"}

    delays = np.arange(0.0, float(max_delay_sec) + 0.25 * dt, dt)
    taus = np.geomspace(float(min_tau_sec), float(max_tau_sec), max(3, int(tau_count)))
    response_mean = float(np.mean(response))
    response_centered = response - response_mean
    alpha = 1.0 - np.exp(-dt / taus)
    candidates = []
    best = None

    for delay in delays:
        delayed = np.interp(
            time - float(delay),
            time,
            command,
            left=float(command[0]),
            right=float(command[-1]),
        )
        filtered = np.empty((taus.size, delayed.size), dtype=float)
        state = np.full(taus.shape, delayed[0], dtype=float)
        filtered[:, 0] = state
        for index in range(1, delayed.size):
            state = state + alpha * (delayed[index] - state)
            filtered[:, index] = state

        basis_mean = np.mean(filtered, axis=1)
        centered = filtered - basis_mean[:, None]
        denominator = np.sum(centered * centered, axis=1)
        gains = np.divide(
            centered @ response_centered,
            denominator,
            out=np.full(denominator.shape, np.nan),
            where=denominator > 1e-12,
        )
        offsets = response_mean - gains * basis_mean
        predictions = offsets[:, None] + gains[:, None] * filtered
        rmse_values = np.sqrt(np.mean((predictions - response[None, :]) ** 2, axis=1))
        tau_index = int(np.nanargmin(rmse_values))
        row = {
            "delay_sec": float(delay),
            "tau_sec": float(taus[tau_index]),
            "gain": float(gains[tau_index]),
            "offset": float(offsets[tau_index]),
            "rmse": float(rmse_values[tau_index]),
        }
        candidates.append(row)
        if best is None or row["rmse"] < best["rmse"]:
            best = row

    if best is None:
        return {"valid": False, "reason": "fit grid produced no candidate"}
    result = {"valid": True, **best}
    result.update(evaluate_fopdt(time, command, response, result))
    near_limit = float(best["rmse"]) * (1.0 + float(near_optimal_rmse_fraction))
    near = [row for row in candidates if row["rmse"] <= near_limit]
    result["grid"] = {
        "dt_sec": dt,
        "delay_min_sec": float(delays[0]),
        "delay_max_sec": float(delays[-1]),
        "delay_count": int(delays.size),
        "tau_min_sec": float(taus[0]),
        "tau_max_sec": float(taus[-1]),
        "tau_count": int(taus.size),
        "near_optimal_rmse_fraction": float(near_optimal_rmse_fraction),
    }
    result["near_optimal_profile"] = {
        "count": len(near),
        "delay_min_sec": min(row["delay_sec"] for row in near),
        "delay_max_sec": max(row["delay_sec"] for row in near),
        "tau_min_sec": min(row["tau_sec"] for row in near),
        "tau_max_sec": max(row["tau_sec"] for row in near),
    }
    result["response_time_sec"] = {
        "t10": float(best["delay_sec"] - best["tau_sec"] * math.log(0.9)),
        "t50": float(best["delay_sec"] - best["tau_sec"] * math.log(0.5)),
        "t90": float(best["delay_sec"] - best["tau_sec"] * math.log(0.1)),
    }
    result["delay_profile"] = candidates
    return result


def frozen_median_model(models):
    valid = [model for model in models if model and model.get("valid", False)]
    if len(valid) != len(models) or not valid:
        return {"valid": False, "reason": "not every training model is valid"}
    return {
        "valid": True,
        "construction": "component-wise median of training-bag fits",
        "delay_sec": statistics.median(float(model["delay_sec"]) for model in valid),
        "tau_sec": statistics.median(float(model["tau_sec"]) for model in valid),
        "gain": statistics.median(float(model["gain"]) for model in valid),
        "offset": statistics.median(float(model["offset"]) for model in valid),
    }


def near_optimal_touches_grid_boundary(model, epsilon=1e-9):
    """Return whether a fit's near-optimal L/tau set reaches its search edge."""

    if not model or not model.get("valid", False):
        return True
    grid = model.get("grid", {})
    near = model.get("near_optimal_profile", {})
    required_grid = ("delay_min_sec", "delay_max_sec", "tau_min_sec", "tau_max_sec")
    required_near = required_grid
    if any(key not in grid for key in required_grid) or any(
        key not in near for key in required_near
    ):
        return True
    epsilon = abs(float(epsilon))
    return bool(
        near["delay_min_sec"] <= grid["delay_min_sec"] + epsilon
        or near["delay_max_sec"] >= grid["delay_max_sec"] - epsilon
        or near["tau_min_sec"] <= grid["tau_min_sec"] + epsilon
        or near["tau_max_sec"] >= grid["tau_max_sec"] - epsilon
    )


def correlation(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    mask = np.isfinite(first) & np.isfinite(second)
    first = first[mask]
    second = second[mask]
    if first.size < 10:
        return 0.0
    first = first - np.mean(first)
    second = second - np.mean(second)
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    return float(np.dot(first, second) / denominator) if denominator > 1e-12 else 0.0


def estimate_relative_lag(reference, response, dt_sec, max_abs_lag_sec=0.30):
    reference = np.asarray(reference, dtype=float)
    response = np.asarray(response, dtype=float)
    maximum = max(1, int(round(float(max_abs_lag_sec) / float(dt_sec))))
    rows = []
    for lag in range(-maximum, maximum + 1):
        if lag > 0:
            first, second = reference[:-lag], response[lag:]
        elif lag < 0:
            first, second = reference[-lag:], response[:lag]
        else:
            first, second = reference, response
        rows.append((lag, correlation(first, second)))
    best_lag, best_correlation = max(rows, key=lambda item: abs(item[1]))
    return {
        "lag_sec": float(best_lag * dt_sec),
        "lag_ms": float(best_lag * dt_sec * 1000.0),
        "peak_correlation_abs": abs(float(best_correlation)),
        "raw_peak_correlation": float(best_correlation),
        "zero_lag_correlation": float(rows[maximum][1]),
        "at_search_boundary": bool(abs(best_lag) == maximum),
    }


def sustained_crossings(time, values, threshold, direction, sustain_sec=0.04):
    """Return threshold crossings that remain on the new side for 40 ms."""

    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    if time.size < 3:
        return []
    dt = float(np.median(np.diff(time)))
    sustain = max(1, int(math.ceil(float(sustain_sec) / max(dt, 1e-9))))
    output = []
    for index in range(1, values.size - sustain):
        if direction > 0:
            crossed = values[index - 1] < threshold <= values[index]
            held = np.all(values[index : index + sustain] >= threshold)
        else:
            crossed = values[index - 1] > threshold >= values[index]
            held = np.all(values[index : index + sustain] <= threshold)
        if not crossed or not held:
            continue
        delta = values[index] - values[index - 1]
        fraction = 0.0 if abs(delta) < 1e-12 else (threshold - values[index - 1]) / delta
        crossing = float(time[index - 1] + fraction * (time[index] - time[index - 1]))
        if not output or crossing - output[-1] >= 0.25:
            output.append(crossing)
    return output


def match_crossing_events(command_times, response_times, minimum_lag=0.0, maximum_lag=2.0):
    """Match each command crossing to a later, unused response crossing.

    These rows are intended as causal response-time diagnostics.  A centered
    smoothing window can move a noisy crossing by a few samples, but accepting
    a response before its command silently turns an ambiguous crossing into an
    apparently valid negative actuator delay.  Such cases are therefore left
    unmatched unless a caller explicitly requests a negative lower bound.
    """
    response_times = list(response_times)
    rows = []
    used = set()
    for command_time in command_times:
        eligible = [
            (index, response_time)
            for index, response_time in enumerate(response_times)
            if index not in used
            and minimum_lag <= response_time - command_time <= maximum_lag
        ]
        if not eligible:
            rows.append((float(command_time), None, None))
            continue
        index, response_time = min(eligible, key=lambda item: abs(item[1] - command_time))
        used.add(index)
        rows.append(
            (float(command_time), float(response_time), float(response_time - command_time))
        )
    return rows
