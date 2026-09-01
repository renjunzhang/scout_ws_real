#!/usr/bin/env python3
"""Fail-closed postflight for one image-free G3 W5/Bsmooth RGB trial."""

import argparse
import collections
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

import rosbag


IMAGE_MESSAGE_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}
REQUIRED_TOPICS = (
    "/cmd_vel",
    "/odom",
    "/imu/data",
    "/scout/global_path_fixed",
    "/spmpc/status",
    "/spmpc/debug/stage0_reference",
    "/spmpc/debug/pre_solve_snapshot",
    "/spmpc/debug/slosh_observer_imu",
    "/spmpc/debug/slosh_observer_selection",
    "/spmpc/debug/effective_config",
    "/spmpc/debug/command_intervention",
    "/spmpc/solver_time_ms",
    "/camera/color/camera_info",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--condition", required=True)
    parser.add_argument(
        "--slosh-enabled",
        choices=("auto", "true", "false"),
        default="auto",
        help="Whether the solver consumes the selected liquid state; auto preserves legacy W5/Bsmooth behavior.",
    )
    parser.add_argument(
        "--smooth-priority-enabled",
        choices=("auto", "true", "false"),
        default="auto",
    )
    parser.add_argument(
        "--protocol",
        default="G3_processed_imu_W5_vs_Bsmooth_online_RGB_development_v1",
    )
    parser.add_argument("--report-suffix", default="_g3_postflight.json")
    parser.add_argument("--row", required=True)
    parser.add_argument("--block", required=True)
    parser.add_argument("--position", required=True)
    parser.add_argument("--measurement-topic", default="/liquid/measurement")
    parser.add_argument("--expected-weight", type=float, required=True)
    parser.add_argument("--expected-w-smooth", type=float)
    parser.add_argument("--expected-w-alpha", type=float)
    parser.add_argument("--expected-w-du-a", type=float)
    parser.add_argument("--expected-w-du-vs", type=float)
    parser.add_argument("--expected-delay-mode-code", type=float)
    parser.add_argument(
        "--expected-solver-source-code",
        type=int,
        help="Expected /spmpc/debug/solver_input_state source_code during motion.",
    )
    parser.add_argument(
        "--require-delay-compensation-applied",
        choices=("auto", "true", "false"),
        default="auto",
        help="Legacy aggregate gate: true when either robot or liquid compensation is applied.",
    )
    parser.add_argument(
        "--require-robot-delay-compensation-applied",
        choices=("auto", "true", "false"),
        default="auto",
        help="Gate the robot-specific application flag in solver_input_state.",
    )
    parser.add_argument(
        "--require-liquid-delay-compensation-applied",
        choices=("auto", "true", "false"),
        default="auto",
        help="Gate the liquid-specific application flag in solver_input_state.",
    )
    parser.add_argument("--require-state-diagnostics", action="store_true")
    parser.add_argument("--expected-v-ref", type=float, default=0.20)
    parser.add_argument("--expected-width", type=int, default=1920)
    parser.add_argument("--expected-height", type=int, default=1080)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--zero-frames", type=int, default=30)
    parser.add_argument("--max-zero-window-spread-mm", type=float)
    parser.add_argument("--initial-stability-sec", type=float, default=0.0)
    parser.add_argument(
        "--min-initial-stability-valid-fraction", type=float, default=0.0
    )
    parser.add_argument("--max-initial-h-vis-p95-mm", type=float)
    parser.add_argument("--max-initial-abs-height-p95-mm", type=float)
    parser.add_argument("--max-initial-half-median-drift-mm", type=float)
    parser.add_argument("--t-hvis-tail-sec", type=float, default=5.0)
    parser.add_argument("--t-motion-max-sec", type=float, default=42.0)
    parser.add_argument("--rolling-window", type=int, default=5)
    parser.add_argument("--min-duration-sec", type=float, default=65.0)
    parser.add_argument("--min-motion-sec", type=float, default=20.0)
    parser.add_argument("--min-pre-motion-sec", type=float, default=9.0)
    parser.add_argument("--max-pre-motion-sec", type=float, default=23.0)
    parser.add_argument("--min-online-pre-motion-sec", type=float, default=2.0)
    parser.add_argument("--min-online-rate-hz", type=float, default=10.0)
    parser.add_argument("--max-online-gap-sec", type=float, default=0.35)
    parser.add_argument("--min-online-valid-fraction", type=float, default=0.90)
    parser.add_argument("--max-p95-publish-lag-sec", type=float, default=0.50)
    parser.add_argument("--max-future-skew-sec", type=float, default=0.05)
    parser.add_argument("--min-source-fraction", type=float, default=0.98)
    parser.add_argument("--min-ready-fraction", type=float, default=0.98)
    parser.add_argument("--max-contour-p95-m", type=float, default=0.05)
    parser.add_argument("--max-yaw-p95-rad", type=float, default=0.15)
    parser.add_argument("--max-solver-p95-ms", type=float, default=25.0)
    parser.add_argument("--rgb-calibration-sha256", required=True)
    parser.add_argument("--online-config-sha256", required=True)
    parser.add_argument("--outcome-window-rule-sha256", required=True)
    parser.add_argument("--prereg-sha256", required=True)
    parser.add_argument("--source-report-sha256", required=True)
    parser.add_argument("--hash-bag", action="store_true")
    return parser.parse_args()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def fraction(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else 0.0


def application_fraction(values):
    clean = [float(value) for value in values if finite(value)]
    if not clean:
        return math.nan
    return fraction(sum(value > 0.5 for value in clean), len(clean))


def application_requirement_failure(label, values, requirement):
    """Return a gate failure while preserving the legacy ``auto`` behavior."""
    if requirement == "auto":
        return None
    clean = [float(value) for value in values if finite(value)]
    if not clean:
        return "{} diagnostics contain no application flags".format(label)
    observed = application_fraction(clean)
    expected = requirement == "true"
    ok = observed >= 0.98 if expected else observed <= 0.02
    if ok:
        return None
    return "{} application fraction {:.4f} violates required {}".format(
        label, observed, requirement
    )


def percentile(values, quantile):
    clean = sorted(float(value) for value in values if finite(value))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * float(quantile)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def rms(values):
    clean = [float(value) for value in values if finite(value)]
    if not clean:
        return math.nan
    return math.sqrt(sum(value * value for value in clean) / len(clean))


def rate_hz(times):
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def max_gap_sec(times):
    if len(times) < 2:
        return math.inf
    return max(b - a for a, b in zip(times, times[1:]))


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(8 * 1024 * 1024)
            if not block:
                return digest.hexdigest()
            digest.update(block)


def parse_multiarray(msg):
    dims = getattr(getattr(msg, "layout", None), "dim", [])
    label = dims[0].label if dims else ""
    fields = [part.strip() for part in label.split(",") if part.strip()]
    values = list(getattr(msg, "data", []))
    return {
        fields[index] if index < len(fields) else "field_{}".format(index): float(value)
        for index, value in enumerate(values)
    }


def close(actual, expected, tolerance=1.0e-5):
    return finite(actual) and abs(float(actual) - float(expected)) <= tolerance


def causal_rolling_median(records, window=5, max_gap=0.35):
    """Return (stamp, median) using current/previous valid samples only.

    A source-time gap larger than max_gap starts a new segment, so a stale
    pre-gap window can never leak into a later measurement.
    """
    if window <= 0:
        raise ValueError("rolling window must be positive")
    history = collections.deque(maxlen=window)
    output = []
    previous_stamp = None
    for stamp, value in records:
        stamp = float(stamp)
        value = float(value)
        if previous_stamp is not None and stamp - previous_stamp > max_gap:
            history.clear()
        history.append(value)
        output.append((stamp, float(statistics.median(history))))
        previous_stamp = stamp
    return output


def signed_stability_metrics(records, midpoint, window=5, max_gap=0.35):
    """Summarize a corrected-height window without hiding negative bias."""
    smoothed = causal_rolling_median(records, window=window, max_gap=max_gap)
    signed_values = [value for _, value in smoothed]
    first_half = [value for stamp, value in smoothed if stamp < midpoint]
    second_half = [value for stamp, value in smoothed if stamp >= midpoint]
    drift = (
        abs(statistics.median(second_half) - statistics.median(first_half))
        if first_half and second_half
        else math.nan
    )
    return {
        "h_vis_p95_mm": percentile([max(0.0, value) for value in signed_values], 0.95),
        "abs_height_p95_mm": percentile([abs(value) for value in signed_values], 0.95),
        "half_median_drift_mm": drift,
        "first_half_samples": len(first_half),
        "second_half_samples": len(second_half),
    }


def in_window(records, start, end, stamp_index=0):
    if start is None or end is None:
        return []
    return [record for record in records if start <= record[stamp_index] <= end]


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not args.report_suffix.startswith("_") or not args.report_suffix.endswith(".json"):
        print("[G3 postflight] --report-suffix must look like _name.json", file=sys.stderr)
        return 2
    out_path = bag_path.with_name(bag_path.stem + args.report_suffix)
    failures = []
    warnings = []
    if args.slosh_enabled == "auto" and args.condition not in ("Bsmooth", "W5"):
        print(
            "[G3 postflight] unknown condition requires explicit --slosh-enabled",
            file=sys.stderr,
        )
        return 2
    if args.expected_solver_source_code is not None and not (
        0 <= args.expected_solver_source_code <= 255
    ):
        print(
            "[G3 postflight] --expected-solver-source-code must be in [0,255]",
            file=sys.stderr,
        )
        return 2
    slosh_enabled = (
        args.condition == "W5"
        if args.slosh_enabled == "auto"
        else args.slosh_enabled == "true"
    )
    smooth_priority_enabled = (
        not slosh_enabled
        if args.smooth_priority_enabled == "auto"
        else args.smooth_priority_enabled == "true"
    )

    if not bag_path.is_file():
        print("[G3 postflight] missing bag: {}".format(bag_path), file=sys.stderr)
        return 2
    if Path(str(bag_path) + ".active").exists():
        print("[G3 postflight] active bag still exists", file=sys.stderr)
        return 2
    if args.rolling_window != 5:
        failures.append("G3 protocol requires rolling_window=5")
    if args.t_hvis_tail_sec <= 0.0:
        failures.append("T_HVIS_TAIL must be positive")
    if args.t_motion_max_sec <= 0.0:
        failures.append("T_MOTION_MAX must be positive")
    if args.initial_stability_sec < 0.0:
        failures.append("initial stability duration cannot be negative")
    if not 0.0 <= args.min_initial_stability_valid_fraction <= 1.0:
        failures.append("initial stability valid fraction must be in [0,1]")

    required_topics = REQUIRED_TOPICS + (args.measurement_topic,)
    counts = collections.Counter()
    topic_times = collections.defaultdict(list)
    cmd_active_times = []
    statuses = []
    measurements = []
    camera_shapes = set()
    selections = []
    imu_records = []
    effective_configs = []
    stage0_records = []
    solver_records = []
    intervention_records = []
    raw_state_records = []
    predicted_state_records = []
    solver_input_state_records = []
    image_topics = []

    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            start_time = float(bag.get_start_time())
            end_time = float(bag.get_end_time())
            topic_info = bag.get_type_and_topic_info().topics
            image_topics = sorted(
                topic
                for topic, info in topic_info.items()
                if info.msg_type in IMAGE_MESSAGE_TYPES and int(info.message_count) > 0
            )
            if image_topics:
                failures.append("image-free protocol violated by: " + ", ".join(image_topics))
            available = set(topic_info)
            state_topics = (
                "/spmpc/debug/raw_state",
                "/spmpc/debug/predicted_state",
                "/spmpc/debug/solver_input_state",
            )
            if args.require_state_diagnostics:
                for topic in state_topics:
                    if topic not in available:
                        failures.append("missing required state diagnostic: {}".format(topic))
            for topic in required_topics:
                if topic not in available:
                    failures.append("missing required topic: {}".format(topic))
            read_topics = [topic for topic in required_topics if topic in available]
            read_topics.extend(topic for topic in state_topics if topic in available)
            for topic, msg, stamp in bag.read_messages(topics=read_topics):
                bag_time = stamp.to_sec()
                counts[topic] += 1
                topic_times[topic].append(bag_time)
                if topic == "/cmd_vel":
                    if abs(float(msg.linear.x)) > 0.03 or abs(float(msg.angular.z)) > 0.03:
                        cmd_active_times.append(bag_time)
                elif topic == "/spmpc/status":
                    statuses.append((bag_time, str(msg.data)))
                elif topic == args.measurement_topic:
                    measurements.append(
                        {
                            "source_time": msg.header.stamp.to_sec(),
                            "bag_time": bag_time,
                            "valid": bool(msg.valid),
                            "zero_locked": bool(msg.zero_locked),
                            "status_code": int(msg.status_code),
                            "status": str(msg.status),
                            "height_mm": float(msg.height_max_lcr_mm),
                            "any_clipped": bool(msg.any_clipped),
                            "width": int(msg.image_width),
                            "height": int(msg.image_height),
                            "encoding": str(msg.image_encoding),
                            "process_every": int(msg.process_every),
                            "zero_count": int(msg.zero_valid_samples),
                            "zero_value": float(msg.zero_max_lcr_mm),
                            "zero_start": msg.zero_window_start_stamp.to_sec(),
                            "zero_end": msg.zero_window_end_stamp.to_sec(),
                            "zero_history": tuple(
                                float(value) for value in msg.zero_samples_max_lcr_raw_mm
                            ),
                        }
                    )
                elif topic == "/camera/color/camera_info":
                    camera_shapes.add((int(msg.width), int(msg.height)))
                elif topic == "/spmpc/debug/slosh_observer_selection":
                    selections.append((bag_time, msg))
                elif topic == "/spmpc/debug/slosh_observer_imu":
                    imu_records.append((bag_time, msg))
                elif topic == "/spmpc/debug/effective_config":
                    effective_configs.append((bag_time, parse_multiarray(msg)))
                elif topic == "/spmpc/debug/stage0_reference":
                    parsed = parse_multiarray(msg)
                    stage0_records.append((bag_time, parsed))
                elif topic == "/spmpc/solver_time_ms":
                    solver_records.append((bag_time, float(msg.data)))
                elif topic == "/spmpc/debug/command_intervention":
                    intervention_records.append((bag_time, parse_multiarray(msg)))
                elif topic == "/spmpc/debug/raw_state":
                    raw_state_records.append((bag_time, parse_multiarray(msg)))
                elif topic == "/spmpc/debug/predicted_state":
                    predicted_state_records.append((bag_time, parse_multiarray(msg)))
                elif topic == "/spmpc/debug/solver_input_state":
                    solver_input_state_records.append((bag_time, parse_multiarray(msg)))
    except Exception as exc:  # noqa: BLE001
        print("[G3 postflight] rosbag read failed: {}".format(exc), file=sys.stderr)
        return 2

    duration = end_time - start_time
    if duration < args.min_duration_sec:
        failures.append(
            "bag duration {:.3f}s < required {:.3f}s".format(duration, args.min_duration_sec)
        )
    motion_start = min(cmd_active_times) if cmd_active_times else None
    motion_end = max(cmd_active_times) if cmd_active_times else None
    motion_duration = (
        motion_end - motion_start
        if motion_start is not None and motion_end is not None
        else 0.0
    )
    if motion_duration < args.min_motion_sec:
        failures.append(
            "nonzero-command motion {:.3f}s < required {:.3f}s".format(
                motion_duration, args.min_motion_sec
            )
        )
    pre_motion = motion_start - start_time if motion_start is not None else 0.0
    if pre_motion < args.min_pre_motion_sec:
        failures.append(
            "recorded pre-motion window {:.3f}s < {:.3f}s".format(
                pre_motion, args.min_pre_motion_sec
            )
        )
    if pre_motion > args.max_pre_motion_sec:
        failures.append(
            "record-to-motion delay {:.3f}s > frozen {:.3f}s budget".format(
                pre_motion, args.max_pre_motion_sec
            )
        )

    arrival_candidates = [
        stamp
        for stamp, status in statuses
        if status == "GOAL_REACHED" and motion_start is not None and stamp >= motion_start
    ]
    first_arrival = min(arrival_candidates) if arrival_candidates else None
    motion_to_arrival = (
        first_arrival - motion_start
        if first_arrival is not None and motion_start is not None
        else math.nan
    )
    if first_arrival is None:
        failures.append("planner never published GOAL_REACHED after motion started")
        window_end = (
            motion_start + args.t_motion_max_sec + args.t_hvis_tail_sec
            if motion_start is not None
            else motion_end
        )
    else:
        if motion_to_arrival > args.t_motion_max_sec:
            failures.append(
                "first arrival {:.3f}s after motion start exceeds frozen {:.3f}s budget".format(
                    motion_to_arrival, args.t_motion_max_sec
                )
            )
        window_end = first_arrival + args.t_hvis_tail_sec
    if window_end is not None and end_time < window_end:
        failures.append(
            "bag ends before frozen RGB tail: {:.3f} < {:.3f}".format(
                end_time, window_end
            )
        )

    expected_shape = (args.expected_width, args.expected_height)
    if camera_shapes and camera_shapes != {expected_shape}:
        failures.append("camera_info shape mismatch: {}".format(sorted(camera_shapes)))
    camera_times = topic_times["/camera/color/camera_info"]
    camera_rate = rate_hz(camera_times)
    if camera_rate < args.expected_fps * 0.90:
        failures.append(
            "camera_info rate {:.3f}Hz < {:.3f}Hz".format(
                camera_rate, args.expected_fps * 0.90
            )
        )

    measurements.sort(key=lambda record: record["source_time"])
    source_times = [record["source_time"] for record in measurements if record["source_time"] > 0]
    missing_source_stamps = len(measurements) - len(source_times)
    if missing_source_stamps:
        failures.append(
            "{} online measurements have zero source stamps".format(missing_source_stamps)
        )
    measurement_shapes = {(r["width"], r["height"]) for r in measurements}
    if measurement_shapes and measurement_shapes != {expected_shape}:
        failures.append(
            "online measurement shape mismatch: {}".format(sorted(measurement_shapes))
        )
    process_every_values = sorted({r["process_every"] for r in measurements})
    if process_every_values and process_every_values != [args.process_every]:
        failures.append(
            "online process_every mismatch: expected {}, got {}".format(
                args.process_every, process_every_values
            )
        )

    locked = [record for record in measurements if record["zero_locked"]]
    zero_counts = sorted({record["zero_count"] for record in locked})
    zero_values = sorted({record["zero_value"] for record in locked})
    zero_windows = sorted({(record["zero_start"], record["zero_end"]) for record in locked})
    zero_histories = {record["zero_history"] for record in locked}
    zero_window_spread = math.nan
    if not locked:
        failures.append("online RGB never locked its zero reference")
    else:
        if zero_counts != [args.zero_frames]:
            failures.append(
                "zero sample count mismatch: expected {}, got {}".format(
                    args.zero_frames, zero_counts
                )
            )
        if len(zero_values) != 1 or not finite(zero_values[0]):
            failures.append("online zero value is not unique/finite")
        if len(zero_windows) != 1:
            failures.append("online zero timestamp window changed")
        else:
            zero_start, zero_end = zero_windows[0]
            if zero_start <= 0.0 or zero_end < zero_start:
                failures.append("invalid online zero timestamp window")
            if motion_start is not None and zero_end >= motion_start:
                failures.append("online zero estimation did not finish before first motion")
        if len(zero_histories) != 1:
            failures.append("online zero sample history changed within the bag")
        else:
            history = next(iter(zero_histories))
            if len(history) != args.zero_frames or not all(finite(value) for value in history):
                failures.append("online zero history is incomplete/non-finite")
            else:
                zero_window_spread = percentile(history, 0.95) - percentile(history, 0.05)
                if len(zero_values) == 1 and abs(statistics.median(history) - zero_values[0]) > 1e-4:
                    failures.append("online zero value does not match recorded history median")
                if (
                    args.max_zero_window_spread_mm is not None
                    and zero_window_spread > args.max_zero_window_spread_mm
                ):
                    failures.append(
                        "online zero-window P95-P05 {:.4f}mm > {:.4f}mm".format(
                            zero_window_spread, args.max_zero_window_spread_mm
                        )
                    )

    online_rate = rate_hz(source_times)
    online_gap = max_gap_sec(source_times)
    if online_rate < args.min_online_rate_hz:
        failures.append(
            "online scalar rate {:.3f}Hz < {:.3f}Hz".format(
                online_rate, args.min_online_rate_hz
            )
        )
    if online_gap > args.max_online_gap_sec:
        failures.append(
            "online scalar max gap {:.3f}s > {:.3f}s".format(
                online_gap, args.max_online_gap_sec
            )
        )
    publish_lags = [
        record["bag_time"] - record["source_time"]
        for record in measurements
        if record["source_time"] > 0.0
    ]
    p95_publish_lag = percentile(publish_lags, 0.95)
    min_publish_lag = min(publish_lags) if publish_lags else math.nan
    if finite(p95_publish_lag) and p95_publish_lag > args.max_p95_publish_lag_sec:
        failures.append(
            "online publish-lag P95 {:.3f}s > {:.3f}s".format(
                p95_publish_lag, args.max_p95_publish_lag_sec
            )
        )
    if finite(min_publish_lag) and min_publish_lag < -args.max_future_skew_sec:
        failures.append(
            "online source stamp future skew {:.3f}s > {:.3f}s".format(
                -min_publish_lag, args.max_future_skew_sec
            )
        )

    def measurement_valid(record):
        return (
            record["valid"]
            and record["zero_locked"]
            and record["status_code"] == 0
            and record["status"] == "OK"
            and finite(record["height_mm"])
            and not record["any_clipped"]
        )

    pre_start = motion_start - args.min_online_pre_motion_sec if motion_start is not None else None
    pre_records = [
        record
        for record in measurements
        if pre_start is not None and pre_start <= record["source_time"] < motion_start
    ]
    pre_valid_fraction = fraction(
        sum(measurement_valid(record) for record in pre_records), len(pre_records)
    )
    if not pre_records or pre_valid_fraction < args.min_online_valid_fraction:
        failures.append(
            "online RGB lacks clean {:.1f}s pre-motion coverage".format(
                args.min_online_pre_motion_sec
            )
        )

    initial_stability_start = (
        motion_start - args.initial_stability_sec
        if motion_start is not None and args.initial_stability_sec > 0.0
        else None
    )
    initial_stability_records = [
        record
        for record in measurements
        if initial_stability_start is not None
        and initial_stability_start <= record["source_time"] < motion_start
    ]
    initial_stability_valid_records = [
        record for record in initial_stability_records if measurement_valid(record)
    ]
    initial_stability_valid_fraction = fraction(
        len(initial_stability_valid_records), len(initial_stability_records)
    )
    initial_stability_complete = args.initial_stability_sec <= 0.0
    initial_h_vis_p95 = math.nan
    initial_abs_height_p95 = math.nan
    initial_half_median_drift = math.nan
    if args.initial_stability_sec > 0.0:
        initial_stability_complete = bool(initial_stability_records) and (
            initial_stability_records[0]["source_time"]
            <= initial_stability_start + args.max_online_gap_sec
            and initial_stability_records[-1]["source_time"]
            >= motion_start - args.max_online_gap_sec
        )
        if not initial_stability_complete:
            failures.append(
                "online RGB lacks the complete {:.1f}s initial-stability window".format(
                    args.initial_stability_sec
                )
            )
        if (
            initial_stability_valid_fraction + 1.0e-12
            < args.min_initial_stability_valid_fraction
        ):
            failures.append(
                "initial-stability RGB valid fraction {:.4f} < {:.4f}".format(
                    initial_stability_valid_fraction,
                    args.min_initial_stability_valid_fraction,
                )
            )
        initial_metrics = signed_stability_metrics(
            [
                (record["source_time"], record["height_mm"])
                for record in initial_stability_valid_records
            ],
            midpoint=initial_stability_start + 0.5 * args.initial_stability_sec,
            window=args.rolling_window,
            max_gap=args.max_online_gap_sec,
        )
        initial_h_vis_p95 = initial_metrics["h_vis_p95_mm"]
        initial_abs_height_p95 = initial_metrics["abs_height_p95_mm"]
        initial_half_median_drift = initial_metrics["half_median_drift_mm"]
        if not (
            initial_metrics["first_half_samples"]
            and initial_metrics["second_half_samples"]
        ):
            failures.append("initial-stability RGB half windows are incomplete")
        if (
            args.max_initial_h_vis_p95_mm is not None
            and (
                not finite(initial_h_vis_p95)
                or initial_h_vis_p95 > args.max_initial_h_vis_p95_mm
            )
        ):
            failures.append(
                "initial-stability H_vis P95 {:.4f}mm > {:.4f}mm".format(
                    initial_h_vis_p95, args.max_initial_h_vis_p95_mm
                )
            )
        if (
            args.max_initial_abs_height_p95_mm is not None
            and (
                not finite(initial_abs_height_p95)
                or initial_abs_height_p95 > args.max_initial_abs_height_p95_mm
            )
        ):
            failures.append(
                "initial-stability |signed height| P95 {:.4f}mm > {:.4f}mm".format(
                    initial_abs_height_p95,
                    args.max_initial_abs_height_p95_mm,
                )
            )
        if (
            args.max_initial_half_median_drift_mm is not None
            and (
                not finite(initial_half_median_drift)
                or initial_half_median_drift
                > args.max_initial_half_median_drift_mm
            )
        ):
            failures.append(
                "initial-stability half-median drift {:.4f}mm > {:.4f}mm".format(
                    initial_half_median_drift,
                    args.max_initial_half_median_drift_mm,
                )
            )

    window_records = [
        record
        for record in measurements
        if motion_start is not None
        and window_end is not None
        and motion_start <= record["source_time"] <= window_end
    ]
    valid_window_records = [record for record in window_records if measurement_valid(record)]
    valid_window_fraction = fraction(len(valid_window_records), len(window_records))
    if valid_window_fraction < args.min_online_valid_fraction:
        failures.append(
            "online RGB valid motion+tail coverage {:.4f} < {:.4f}".format(
                valid_window_fraction, args.min_online_valid_fraction
            )
        )
    if (
        valid_window_records
        and valid_window_records[-1]["source_time"]
        < window_end - args.max_online_gap_sec
    ):
        failures.append("online RGB scalar lacks the complete frozen post-arrival tail")

    smoothed = causal_rolling_median(
        [(record["source_time"], record["height_mm"]) for record in valid_window_records],
        window=args.rolling_window,
        max_gap=args.max_online_gap_sec,
    )
    h_vis = [(stamp, max(0.0, value)) for stamp, value in smoothed]
    h_values = [value for _, value in h_vis]
    tail_values = [
        value
        for stamp, value in h_vis
        if first_arrival is not None and first_arrival <= stamp <= window_end
    ]
    if not h_values:
        failures.append("no valid H_vis samples in frozen motion+tail window")
    if not tail_values:
        failures.append("no valid H_vis samples in frozen tail window")

    selection_motion = in_window(selections, motion_start, motion_end)
    selection_good = []
    for _, msg in selection_motion:
        common_good = (
            bool(msg.configured)
            and bool(msg.valid)
            and not bool(msg.fallback_active)
            and str(msg.nominal_source_name) == "processed_imu"
            and str(msg.effective_source_name) == "processed_imu"
            and str(msg.status) == "NOMINAL_PROCESSED_IMU"
            and bool(msg.imu_pipeline_ready)
        )
        consumes_good = bool(msg.solver_consumes_selected_state) == slosh_enabled
        if common_good and consumes_good:
            selection_good.append(msg)
    source_fraction = fraction(len(selection_good), len(selection_motion))
    fallback_samples = sum(bool(msg.fallback_active) for _, msg in selection_motion)
    if source_fraction < args.min_source_fraction:
        failures.append(
            "processed-IMU source/consumption coverage {:.4f} < {:.4f}".format(
                source_fraction, args.min_source_fraction
            )
        )
    if fallback_samples:
        failures.append("observer fallback active in {} motion samples".format(fallback_samples))

    imu_motion = in_window(imu_records, motion_start, motion_end)
    imu_ready = [
        msg
        for _, msg in imu_motion
        if bool(msg.valid)
        and str(msg.input_status) == "READY"
        and bool(msg.bias_ready)
        and bool(msg.filter_ready)
    ]
    ready_fraction = fraction(len(imu_ready), len(imu_motion))
    reset_epochs = sorted({int(msg.reset_epoch) for _, msg in imu_motion})
    if ready_fraction < args.min_ready_fraction:
        failures.append(
            "processed-IMU READY coverage {:.4f} < {:.4f}".format(
                ready_fraction, args.min_ready_fraction
            )
        )
    if reset_epochs not in ([], [0]):
        failures.append("processed-IMU reset epochs during motion: {}".format(reset_epochs))

    configs_motion = in_window(effective_configs, motion_start, motion_end)
    configs_motion = [config for _, config in configs_motion]
    config = configs_motion[-1] if configs_motion else {}
    if not close(config.get("v_ref"), args.expected_v_ref):
        failures.append("effective v_ref does not match frozen value")
    if not close(config.get("w_slosh"), args.expected_weight):
        failures.append("effective w_slosh does not match condition")
    expected_config_values = {
        "w_smooth": args.expected_w_smooth,
        "w_alpha": args.expected_w_alpha,
        "w_du_a": args.expected_w_du_a,
        "w_du_vs": args.expected_w_du_vs,
        "delay_phase_mode_code": args.expected_delay_mode_code,
    }
    for field, expected in expected_config_values.items():
        if expected is not None and not close(config.get(field), expected):
            failures.append("effective {} does not match frozen value".format(field))
    if not close(config.get("slosh_enable"), 1.0 if slosh_enabled else 0.0):
        failures.append("effective slosh_enable does not match condition")
    if not close(
        config.get("smooth_priority_enable"),
        1.0 if smooth_priority_enabled else 0.0,
    ):
        failures.append("effective smooth_priority_enable does not match condition")

    state_window_end = window_end if window_end is not None else motion_end
    imu_state_window = in_window(imu_records, motion_start, state_window_end)
    raw_state_window = in_window(raw_state_records, motion_start, state_window_end)
    predicted_state_window = in_window(predicted_state_records, motion_start, state_window_end)
    solver_input_state_window = in_window(
        solver_input_state_records, motion_start, state_window_end
    )
    raw_state_motion = in_window(raw_state_records, motion_start, motion_end)
    predicted_state_motion = in_window(predicted_state_records, motion_start, motion_end)
    solver_input_state_motion = in_window(
        solver_input_state_records, motion_start, motion_end
    )
    imu_modal_height_mm = [
        1000.0 * float(msg.modal_height_m)
        for _, msg in imu_state_window
        if bool(msg.valid) and finite(msg.modal_height_m)
    ]
    raw_modal_height_mm = [
        record.get("h_modal_mm", math.nan) for _, record in raw_state_window
    ]
    predicted_modal_height_mm = [
        record.get("h_modal_mm", math.nan)
        for _, record in predicted_state_window
        if record.get("valid", 0.0) > 0.5
    ]
    solver_input_modal_height_mm = [
        record.get("h_modal_mm", math.nan)
        for _, record in solver_input_state_window
    ]
    delay_applied_values = [
        record.get("delay_compensation_applied", math.nan)
        for _, record in solver_input_state_motion
        if finite(record.get("delay_compensation_applied"))
    ]
    delay_applied_fraction = fraction(
        sum(value > 0.5 for value in delay_applied_values),
        len(delay_applied_values),
    )
    robot_delay_applied_values = [
        record.get("robot_delay_compensation_applied", math.nan)
        for _, record in solver_input_state_motion
        if finite(record.get("robot_delay_compensation_applied"))
    ]
    robot_delay_applied_fraction = application_fraction(robot_delay_applied_values)
    liquid_delay_applied_values = [
        record.get("liquid_delay_compensation_applied", math.nan)
        for _, record in solver_input_state_motion
        if finite(record.get("liquid_delay_compensation_applied"))
    ]
    liquid_delay_applied_fraction = application_fraction(liquid_delay_applied_values)
    solver_source_values = [
        int(round(record.get("source_code")))
        for _, record in solver_input_state_motion
        if finite(record.get("source_code"))
    ]
    solver_source_fraction = (
        fraction(
            sum(value == args.expected_solver_source_code for value in solver_source_values),
            len(solver_source_values),
        )
        if args.expected_solver_source_code is not None
        else math.nan
    )
    if args.require_state_diagnostics:
        required_state_windows = {
            "raw_state": raw_state_motion,
            "predicted_state": predicted_state_motion,
            "solver_input_state": solver_input_state_motion,
        }
        for name, records in required_state_windows.items():
            if not records:
                failures.append("required {} has no samples during motion".format(name))
        required_modal_series = {
            "processed-IMU modal height": imu_modal_height_mm,
            "raw selected modal height": raw_modal_height_mm,
            "valid predicted modal height": predicted_modal_height_mm,
            "solver-input modal height": solver_input_modal_height_mm,
        }
        for name, values in required_modal_series.items():
            if not any(finite(value) for value in values):
                failures.append("required {} has no finite motion+tail samples".format(name))
        if not delay_applied_values:
            failures.append("solver-input diagnostics contain no delay-application flags")
    if (
        args.expected_solver_source_code is not None
        and solver_source_fraction < args.min_source_fraction
    ):
        failures.append(
            "solver-input source-code coverage {:.4f} < {:.4f} for expected {}".format(
                solver_source_fraction,
                args.min_source_fraction,
                args.expected_solver_source_code,
            )
        )
    application_requirements = (
        (
            "delay-compensation",
            delay_applied_values,
            args.require_delay_compensation_applied,
        ),
        (
            "robot-delay-compensation",
            robot_delay_applied_values,
            args.require_robot_delay_compensation_applied,
        ),
        (
            "liquid-delay-compensation",
            liquid_delay_applied_values,
            args.require_liquid_delay_compensation_applied,
        ),
    )
    for label, values, requirement in application_requirements:
        failure = application_requirement_failure(label, values, requirement)
        if failure:
            failures.append(failure)

    stage_motion = in_window(stage0_records, motion_start, motion_end)
    contour_values = [abs(record.get("contour_error", math.nan)) for _, record in stage_motion]
    yaw_values = [abs(record.get("yaw_error", math.nan)) for _, record in stage_motion]
    contour_p95 = percentile(contour_values, 0.95)
    yaw_p95 = percentile(yaw_values, 0.95)
    if not finite(contour_p95) or contour_p95 > args.max_contour_p95_m:
        failures.append(
            "stage-0 contour P95 {:.6f}m > {:.6f}m".format(
                contour_p95, args.max_contour_p95_m
            )
        )
    if not finite(yaw_p95) or yaw_p95 > args.max_yaw_p95_rad:
        failures.append(
            "stage-0 yaw P95 {:.6f}rad > {:.6f}rad".format(
                yaw_p95, args.max_yaw_p95_rad
            )
        )

    solver_motion = [value for _, value in in_window(solver_records, motion_start, motion_end)]
    solver_p95 = percentile(solver_motion, 0.95)
    if not finite(solver_p95) or solver_p95 >= args.max_solver_p95_ms:
        failures.append(
            "solver-time P95 {:.3f}ms >= {:.3f}ms".format(
                solver_p95, args.max_solver_p95_ms
            )
        )

    intervention_motion = [
        record for _, record in in_window(intervention_records, motion_start, motion_end)
    ]
    unsafe_zero_fields = (
        "zero_due_to_solver_failure",
        "zero_due_to_terminal_spin_fail",
        "zero_due_to_tracking_safety",
    )
    unsafe_zero_counts = {
        field: sum(record.get(field, 0.0) > 0.5 for record in intervention_motion)
        for field in unsafe_zero_fields
    }
    if any(unsafe_zero_counts.values()):
        failures.append("unsafe command-zero intervention observed: {}".format(unsafe_zero_counts))

    report = {
        "schema_version": 1,
        "protocol": args.protocol,
        "status": "PASS" if not failures else "FAIL",
        "condition": args.condition,
        "row": args.row,
        "block": args.block,
        "position": args.position,
        "bag": str(bag_path),
        "bag_size_bytes": bag_path.stat().st_size,
        "bag_sha256": sha256_file(bag_path) if args.hash_bag else "",
        "failures": failures,
        "warnings": warnings,
        "duration_sec": duration,
        "motion_start_sec": motion_start,
        "motion_end_sec": motion_end,
        "motion_duration_sec": motion_duration,
        "motion_to_arrival_sec": motion_to_arrival,
        "first_arrival_sec": first_arrival,
        "window_end_sec": window_end,
        "t_hvis_tail_sec": args.t_hvis_tail_sec,
        "t_motion_max_sec": args.t_motion_max_sec,
        "pre_motion_sec": pre_motion,
        "bindings": {
            "rgb_calibration_sha256": args.rgb_calibration_sha256,
            "online_config_sha256": args.online_config_sha256,
            "outcome_window_rule_sha256": args.outcome_window_rule_sha256,
            "prereg_sha256": args.prereg_sha256,
            "source_report_sha256": args.source_report_sha256,
        },
        "topic_counts": dict(sorted(counts.items())),
        "image_stream_audit": {
            "count": len(image_topics),
            "recorded_image_topics": image_topics,
        },
        "online_rgb": {
            "samples": len(measurements),
            "rate_hz": online_rate,
            "max_gap_sec": online_gap,
            "publish_lag_p95_sec": p95_publish_lag,
            "publish_lag_min_sec": min_publish_lag,
            "camera_info_rate_hz": camera_rate,
            "pre_window_samples": len(pre_records),
            "pre_valid_fraction": pre_valid_fraction,
            "motion_tail_samples": len(window_records),
            "motion_tail_valid_samples": len(valid_window_records),
            "motion_tail_valid_fraction": valid_window_fraction,
            "rolling_window": args.rolling_window,
            "h_vis_sample_count": len(h_values),
            "h_vis_p95_mm": percentile(h_values, 0.95),
            "h_vis_peak_mm": max(h_values) if h_values else math.nan,
            "h_vis_rms_mm": rms(h_values),
            "h_vis_tail_rms_mm": rms(tail_values),
            "zero_sample_counts": zero_counts,
            "zero_values_mm": zero_values,
            "zero_windows_sec": zero_windows,
            "zero_window_p95_p05_mm": zero_window_spread,
            "initial_stability_sec": args.initial_stability_sec,
            "initial_stability_complete": initial_stability_complete,
            "initial_stability_samples": len(initial_stability_records),
            "initial_stability_valid_samples": len(initial_stability_valid_records),
            "initial_stability_valid_fraction": initial_stability_valid_fraction,
            "initial_stability_h_vis_p95_mm": initial_h_vis_p95,
            "initial_stability_abs_height_p95_mm": initial_abs_height_p95,
            "initial_stability_half_median_drift_mm": initial_half_median_drift,
        },
        "processed_imu": {
            "selection_motion_samples": len(selection_motion),
            "source_contract_samples": len(selection_good),
            "source_contract_fraction": source_fraction,
            "fallback_samples": fallback_samples,
            "ready_motion_samples": len(imu_ready),
            "imu_motion_samples": len(imu_motion),
            "ready_fraction": ready_fraction,
            "reset_epochs": reset_epochs,
            "solver_consumes_selected_state": slosh_enabled,
        },
        "tracking": {
            "stage0_samples": len(stage_motion),
            "contour_p95_m": contour_p95,
            "yaw_p95_rad": yaw_p95,
        },
        "runtime": {
            "solver_samples": len(solver_motion),
            "solver_p95_ms": solver_p95,
            "solver_max_ms": max(solver_motion) if solver_motion else math.nan,
        },
        "internal_state": {
            "imu_modal_height_p95_mm": percentile(imu_modal_height_mm, 0.95),
            "raw_selected_modal_height_p95_mm": percentile(raw_modal_height_mm, 0.95),
            "predicted_modal_height_p95_mm": percentile(
                predicted_modal_height_mm, 0.95
            ),
            "solver_input_modal_height_p95_mm": percentile(
                solver_input_modal_height_mm, 0.95
            ),
            "delay_compensation_applied_fraction": delay_applied_fraction,
            "robot_delay_compensation_applied_fraction": robot_delay_applied_fraction,
            "liquid_delay_compensation_applied_fraction": liquid_delay_applied_fraction,
            "solver_source_code_expected": args.expected_solver_source_code,
            "solver_source_code_fraction": solver_source_fraction,
            "imu_samples": len(imu_modal_height_mm),
            "raw_samples": len(raw_modal_height_mm),
            "predicted_samples": len(predicted_modal_height_mm),
            "solver_input_samples": len(solver_input_modal_height_mm),
            "raw_motion_samples": len(raw_state_motion),
            "predicted_motion_samples": len(predicted_state_motion),
            "solver_input_motion_samples": len(solver_input_state_motion),
        },
        "command_intervention": {
            "motion_samples": len(intervention_motion),
            "unsafe_zero_counts": unsafe_zero_counts,
        },
        "effective_config_last": config,
        "planner_statuses": dict(collections.Counter(status for _, status in statuses)),
    }
    out_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print("[G3 postflight] {}: {}".format(report["status"], bag_path))
    print(
        "  motion={:.3f}s; H_vis P95={:.4f}mm; RGB valid={:.4f}; "
        "source={:.4f}; IMU_READY={:.4f}".format(
            motion_duration,
            report["online_rgb"]["h_vis_p95_mm"],
            valid_window_fraction,
            source_fraction,
            ready_fraction,
        )
    )
    print("  report = {}".format(out_path))
    for failure in failures:
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
