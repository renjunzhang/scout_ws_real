#!/usr/bin/env python3
"""Pure helpers for timestamp-locked O0/I0/I1/L22 comparison.

The module intentionally has no ROS or plotting dependency so the frozen
alignment and release-decision rules can be unit tested in isolation.
"""

import bisect
import math
import statistics


METHODS = ("O0", "I0", "I1", "L22")


def percentile(values, fraction):
    clean = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not clean:
        return math.nan
    position = max(0.0, min(1.0, float(fraction))) * (len(clean) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def correlation(left, right):
    pairs = [
        (float(a), float(b))
        for a, b in zip(left, right)
        if math.isfinite(float(a)) and math.isfinite(float(b))
    ]
    if len(pairs) < 3:
        return math.nan
    xs = [item[0] for item in pairs]
    ys = [item[1] for item in pairs]
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    norm_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    norm_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    return numerator / (norm_x * norm_y) if norm_x > 0.0 and norm_y > 0.0 else math.nan


def centered_rolling_median(records, window=5):
    """Filter valid values while preserving an invalid center as invalid."""
    if int(window) != window or window < 1 or window % 2 == 0:
        raise ValueError("median window must be a positive odd integer")
    half = int(window) // 2
    output = []
    for index, (stamp, value) in enumerate(records):
        if value is None or not math.isfinite(float(value)):
            output.append((float(stamp), None))
            continue
        neighbors = [
            float(candidate)
            for _, candidate in records[max(0, index - half):index + half + 1]
            if candidate is not None and math.isfinite(float(candidate))
        ]
        output.append((float(stamp), statistics.median(neighbors) if neighbors else None))
    return output


def interpolate_at(query, times, values, max_gap_sec):
    if len(times) != len(values) or len(times) < 2:
        return None
    query = float(query)
    if query < times[0] or query > times[-1]:
        return None
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1.0e-12:
        value = float(values[index])
        return value if math.isfinite(value) else None
    if index == 0 or index >= len(times):
        return None
    t0 = float(times[index - 1])
    t1 = float(times[index])
    if t1 <= t0 or t1 - t0 > float(max_gap_sec):
        return None
    v0 = float(values[index - 1])
    v1 = float(values[index])
    if not (math.isfinite(v0) and math.isfinite(v1)):
        return None
    ratio = (query - t0) / (t1 - t0)
    return v0 + ratio * (v1 - v0)


def motion_window(command_records, linear_threshold=0.03, angular_threshold=0.03,
                  tail_sec=3.0):
    moving = [
        float(stamp)
        for stamp, linear, angular in command_records
        if math.isfinite(float(stamp))
        and (
            abs(float(linear)) > float(linear_threshold)
            or abs(float(angular)) > float(angular_threshold)
        )
    ]
    if not moving:
        raise ValueError("no effective motion command was found")
    return min(moving), max(moving) + max(0.0, float(tail_sec))


def align_and_score(visual_records, estimate_records, max_gap_sec=0.10):
    """Interpolate an estimate at visual stamps with fixed lag=0 and scale=1."""
    estimates = sorted(
        (float(stamp), float(value))
        for stamp, value in estimate_records
        if math.isfinite(float(stamp)) and math.isfinite(float(value))
    )
    visual = [
        (float(stamp), float(value))
        for stamp, value in visual_records
        if value is not None
        and math.isfinite(float(stamp))
        and math.isfinite(float(value))
    ]
    times = [item[0] for item in estimates]
    values = [item[1] for item in estimates]
    aligned = []
    for stamp, reference in visual:
        predicted = interpolate_at(stamp, times, values, max_gap_sec)
        if predicted is not None:
            aligned.append((stamp, reference, predicted, predicted - reference))
    errors = [row[3] for row in aligned]
    absolute = [abs(value) for value in errors]
    references = [row[1] for row in aligned]
    predictions = [row[2] for row in aligned]
    if aligned:
        reference_peak = max(aligned, key=lambda row: row[1])
        prediction_peak = max(aligned, key=lambda row: row[2])
        peak_timing_error_sec = prediction_peak[0] - reference_peak[0]
    else:
        peak_timing_error_sec = math.nan
    return {
        "eligible_visual_samples": len(visual),
        "aligned_samples": len(aligned),
        "coverage": len(aligned) / len(visual) if visual else 0.0,
        "mae_mm": statistics.fmean(absolute) if absolute else math.nan,
        "rmse_mm": math.sqrt(statistics.fmean(value * value for value in errors))
        if errors else math.nan,
        "p95_abs_error_mm": percentile(absolute, 0.95),
        "signed_bias_mm": statistics.fmean(errors) if errors else math.nan,
        "corr": correlation(predictions, references),
        "visual_peak_mm": max(references) if references else math.nan,
        "estimate_peak_mm": max(predictions) if predictions else math.nan,
        "peak_timing_error_sec": peak_timing_error_sec,
        "aligned_rows": aligned,
    }


def development_decision(trials, evidence_class="development", min_bags=3,
                         min_coverage=0.98):
    """Apply the preregistered I1-vs-I0 gate without hiding O0 or L22."""
    valid_trials = []
    for trial in trials:
        methods = trial.get("methods", {})
        i0 = methods.get("I0", {})
        i1 = methods.get("I1", {})
        required = (
            i0.get("mae_mm"), i1.get("mae_mm"),
            i0.get("coverage"), i1.get("coverage"),
        )
        if all(value is not None and math.isfinite(float(value)) for value in required):
            valid_trials.append(trial)

    result = {
        "evidence_class": str(evidence_class),
        "required_independent_bags": int(min_bags),
        "valid_bag_count": len(valid_trials),
        "minimum_coverage": float(min_coverage),
        "i1_better_count": 0,
        "required_directional_count": max(2, int(math.ceil(2.0 * max(len(valid_trials), min_bags) / 3.0))),
        "mean_i0_mae_mm": math.nan,
        "mean_i1_mae_mm": math.nan,
        "mean_relative_improvement": math.nan,
        "average_mae_pass": False,
        "direction_pass": False,
        "coverage_pass": False,
        "status": "INSUFFICIENT_INDEPENDENT_BAGS",
    }
    if valid_trials:
        i0_mae = [float(row["methods"]["I0"]["mae_mm"]) for row in valid_trials]
        i1_mae = [float(row["methods"]["I1"]["mae_mm"]) for row in valid_trials]
        result["mean_i0_mae_mm"] = statistics.fmean(i0_mae)
        result["mean_i1_mae_mm"] = statistics.fmean(i1_mae)
        result["mean_relative_improvement"] = (
            (result["mean_i0_mae_mm"] - result["mean_i1_mae_mm"])
            / result["mean_i0_mae_mm"]
            if result["mean_i0_mae_mm"] > 0.0 else math.nan
        )
        result["i1_better_count"] = sum(new < old for old, new in zip(i0_mae, i1_mae))
        result["average_mae_pass"] = result["mean_i1_mae_mm"] < result["mean_i0_mae_mm"]
        result["coverage_pass"] = all(
            float(row["methods"][method]["coverage"]) >= float(min_coverage)
            for row in valid_trials for method in METHODS
        )

    if len(valid_trials) < int(min_bags):
        return result
    result["required_directional_count"] = int(math.ceil(2.0 * len(valid_trials) / 3.0))
    result["direction_pass"] = result["i1_better_count"] >= result["required_directional_count"]
    if str(evidence_class) != "held_out":
        result["status"] = "DEVELOPMENT_ONLY_NOT_RELEASE_EVIDENCE"
    elif result["average_mae_pass"] and result["direction_pass"] and result["coverage_pass"]:
        result["status"] = "I1_OFFLINE_GATE_PASS"
    else:
        result["status"] = "I1_OFFLINE_GATE_FAIL"
    return result


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value
