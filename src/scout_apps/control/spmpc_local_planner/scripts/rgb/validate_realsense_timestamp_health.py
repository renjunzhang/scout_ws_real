#!/usr/bin/env python3
"""Fail-closed pre-motion health gate for a stamped RealSense ROS topic."""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rospy
from sensor_msgs.msg import CameraInfo


def percentile(values, quantile):
    clean = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * float(quantile)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def evaluate_timestamp_records(
    records,
    max_future_skew_sec=0.05,
    max_p95_lag_sec=0.20,
    min_clock_rate_ratio=0.98,
    max_clock_rate_ratio=1.02,
    max_gap_sec=0.20,
):
    failures = []
    clean = []
    nonfinite_samples = 0
    for receive_time, source_time in records:
        try:
            record = (float(receive_time), float(source_time))
        except (TypeError, ValueError):
            nonfinite_samples += 1
            continue
        if not all(math.isfinite(value) for value in record):
            nonfinite_samples += 1
            continue
        clean.append(record)
    if nonfinite_samples:
        failures.append("{} non-finite timestamp samples".format(nonfinite_samples))
    if len(clean) < 2:
        failures.append("fewer than two finite stamped samples")
        return {
            "status": "FAIL",
            "failures": failures,
            "samples": len(clean),
            "nonfinite_samples": nonfinite_samples,
        }
    zero_stamps = sum(source_time <= 0.0 for _, source_time in clean)
    if zero_stamps:
        failures.append("{} zero/nonpositive source stamps".format(zero_stamps))
    positive = [(receive, source) for receive, source in clean if source > 0.0]
    if len(positive) < 2:
        failures.append("fewer than two positive source stamps")
        return {
            "status": "FAIL",
            "failures": failures,
            "samples": len(clean),
            "nonfinite_samples": nonfinite_samples,
            "zero_stamp_samples": zero_stamps,
        }

    receive_times = [record[0] for record in positive]
    source_times = [record[1] for record in positive]
    lags = [receive - source for receive, source in positive]
    receive_deltas = [right - left for left, right in zip(receive_times, receive_times[1:])]
    source_deltas = [right - left for left, right in zip(source_times, source_times[1:])]
    receive_span = receive_times[-1] - receive_times[0]
    source_span = source_times[-1] - source_times[0]
    clock_rate_ratio = source_span / receive_span if receive_span > 0.0 else math.nan
    min_lag = min(lags)
    p95_lag = percentile(lags, 0.95)
    max_lag = max(lags)
    max_source_gap = max(source_deltas) if source_deltas else math.nan
    source_regressions = sum(delta <= 0.0 for delta in source_deltas)
    receive_regressions = sum(delta <= 0.0 for delta in receive_deltas)

    if min_lag < -max_future_skew_sec:
        failures.append(
            "source stamp future skew {:.6f}s exceeds {:.6f}s".format(
                -min_lag, max_future_skew_sec
            )
        )
    if not math.isfinite(p95_lag) or p95_lag > max_p95_lag_sec:
        failures.append(
            "receive lag P95 {:.6f}s exceeds {:.6f}s".format(
                p95_lag, max_p95_lag_sec
            )
        )
    if not math.isfinite(clock_rate_ratio) or not (
        min_clock_rate_ratio <= clock_rate_ratio <= max_clock_rate_ratio
    ):
        failures.append(
            "source/receive clock-rate ratio {:.6f} outside [{:.6f}, {:.6f}]".format(
                clock_rate_ratio, min_clock_rate_ratio, max_clock_rate_ratio
            )
        )
    if not math.isfinite(max_source_gap) or max_source_gap > max_gap_sec:
        failures.append(
            "source max gap {:.6f}s exceeds {:.6f}s".format(
                max_source_gap, max_gap_sec
            )
        )
    if source_regressions:
        failures.append("{} source timestamp regressions/duplicates".format(source_regressions))
    if receive_regressions:
        failures.append("{} receive-time regressions/duplicates".format(receive_regressions))

    return {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "samples": len(clean),
        "nonfinite_samples": nonfinite_samples,
        "positive_stamp_samples": len(positive),
        "zero_stamp_samples": zero_stamps,
        "receive_span_sec": receive_span,
        "source_span_sec": source_span,
        "clock_rate_ratio": clock_rate_ratio,
        "min_receive_minus_source_sec": min_lag,
        "p95_receive_minus_source_sec": p95_lag,
        "max_receive_minus_source_sec": max_lag,
        "max_source_gap_sec": max_source_gap,
        "source_regressions": source_regressions,
        "receive_regressions": receive_regressions,
        "thresholds": {
            "max_future_skew_sec": max_future_skew_sec,
            "max_p95_lag_sec": max_p95_lag_sec,
            "min_clock_rate_ratio": min_clock_rate_ratio,
            "max_clock_rate_ratio": max_clock_rate_ratio,
            "max_gap_sec": max_gap_sec,
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/color/camera_info")
    parser.add_argument("--samples", type=int, default=90)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--max-future-skew-sec", type=float, default=0.05)
    parser.add_argument("--max-p95-lag-sec", type=float, default=0.20)
    parser.add_argument("--min-clock-rate-ratio", type=float, default=0.98)
    parser.add_argument("--max-clock-rate-ratio", type=float, default=1.02)
    parser.add_argument("--max-gap-sec", type=float, default=0.20)
    parser.add_argument(
        "--settle-until-pass",
        action="store_true",
        help="keep sampling until one consecutive window passes or timeout expires",
    )
    parser.add_argument("--report", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.samples < 2 or args.timeout_sec <= 0.0:
        print("[RealSense timestamp gate] invalid samples/timeout", file=sys.stderr)
        return 2
    report_path = Path(args.report).expanduser().resolve()
    records = []
    lock = threading.Lock()
    complete = threading.Event()
    passing_result = None
    windows_evaluated = 0

    rospy.init_node("realsense_timestamp_health_gate", anonymous=True, disable_signals=True)

    def callback(message):
        nonlocal passing_result, windows_evaluated
        receive_time = rospy.Time.now().to_sec()
        source_time = message.header.stamp.to_sec()
        with lock:
            if complete.is_set():
                return
            records.append((receive_time, source_time))
            if len(records) < args.samples:
                return
            if not args.settle_until_pass:
                complete.set()
                return
            windows_evaluated += 1
            candidate = evaluate_timestamp_records(
                records[-args.samples :],
                max_future_skew_sec=args.max_future_skew_sec,
                max_p95_lag_sec=args.max_p95_lag_sec,
                min_clock_rate_ratio=args.min_clock_rate_ratio,
                max_clock_rate_ratio=args.max_clock_rate_ratio,
                max_gap_sec=args.max_gap_sec,
            )
            if candidate["status"] == "PASS":
                passing_result = candidate
                complete.set()

    subscriber = rospy.Subscriber(args.topic, CameraInfo, callback, queue_size=1)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while not complete.is_set() and time.monotonic() < deadline and not rospy.is_shutdown():
            complete.wait(0.05)
    finally:
        subscriber.unregister()

    with lock:
        snapshot = list(records)
        accepted_result = dict(passing_result) if passing_result is not None else None
        evaluated_count = windows_evaluated
    if accepted_result is not None:
        result = accepted_result
    else:
        result = evaluate_timestamp_records(
            snapshot[-args.samples :],
            max_future_skew_sec=args.max_future_skew_sec,
            max_p95_lag_sec=args.max_p95_lag_sec,
            min_clock_rate_ratio=args.min_clock_rate_ratio,
            max_clock_rate_ratio=args.max_clock_rate_ratio,
            max_gap_sec=args.max_gap_sec,
        )
        if len(snapshot) >= args.samples:
            evaluated_count = max(evaluated_count, 1)
    result.update(
        {
            "schema_version": 1,
            "topic": args.topic,
            "requested_samples": args.samples,
            "timeout_sec": args.timeout_sec,
            "settle_until_pass": args.settle_until_pass,
            "observed_samples": len(snapshot),
            "windows_evaluated": evaluated_count,
            "timed_out": not complete.is_set(),
        }
    )
    if len(snapshot) < args.samples:
        result["status"] = "FAIL"
        result.setdefault("failures", []).append(
            "received {} of {} required samples".format(len(snapshot), args.samples)
        )
    elif args.settle_until_pass and accepted_result is None:
        result["status"] = "FAIL"
        result.setdefault("failures", []).append(
            "no healthy consecutive {}-sample window within {:.1f}s".format(
                args.samples, args.timeout_sec
            )
        )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(
        "[RealSense timestamp gate] {}: samples={}; min_lag={:.4f}s; "
        "p95_lag={:.4f}s; rate_ratio={:.6f}; max_gap={:.4f}s".format(
            result["status"],
            result.get("samples", 0),
            result.get("min_receive_minus_source_sec", math.nan),
            result.get("p95_receive_minus_source_sec", math.nan),
            result.get("clock_rate_ratio", math.nan),
            result.get("max_source_gap_sec", math.nan),
        )
    )
    print("  report = {}".format(report_path))
    for failure in result.get("failures", []):
        print("  FAIL: {}".format(failure), file=sys.stderr)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
