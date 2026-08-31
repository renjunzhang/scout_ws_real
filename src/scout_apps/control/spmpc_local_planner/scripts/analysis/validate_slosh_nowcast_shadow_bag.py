#!/usr/bin/env python3
"""Fail-closed postflight for the B0 O0/I0/I1/L22 shadow trial."""

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import rosbag


COMPARISON_TOPIC = "/spmpc/debug/slosh_estimator_comparison"
SELECTION_TOPIC = "/spmpc/debug/slosh_observer_selection"
AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
LIQUID_TOPIC = "/liquid/measurement"
METHODS = ("o0", "i0", "i1", "l22")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--report", required=True)
    parser.add_argument("--minimum-coverage", type=float, default=0.98)
    parser.add_argument("--minimum-rgb-clean-coverage", type=float, default=0.90)
    parser.add_argument("--max-prediction-sec", type=float, default=0.050)
    parser.add_argument("--max-excitation-age-sec", type=float, default=0.060)
    parser.add_argument("--max-future-skew-sec", type=float, default=0.005)
    parser.add_argument("--expected-v-safe-max", type=float, default=0.15)
    parser.add_argument("--speed-tolerance", type=float, default=0.0001)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.03)
    parser.add_argument("--angular-motion-threshold", type=float, default=0.03)
    parser.add_argument("--motion-tail-sec", type=float, default=3.0)
    return parser.parse_args()


def stamp_sec(stamp):
    try:
        return float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def percentile(values, fraction):
    clean = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not clean:
        return None
    position = max(0.0, min(1.0, fraction)) * (len(clean) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    weight = position - lower
    return clean[lower] * (1.0 - weight) + clean[upper] * weight


def summary(values):
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "median": statistics.median(clean),
        "p95": percentile(clean, 0.95),
        "min": min(clean),
        "max": max(clean),
    }


def target_time(message):
    value = stamp_sec(message.target_stamp)
    return value if value > 0.0 else stamp_sec(message.header.stamp)


def active_window(audits, linear_threshold, angular_threshold, tail_sec):
    stamps = [
        stamp_sec(message.command_publish_stamp) or stamp_sec(message.header.stamp)
        for message in audits
        if bool(message.command_was_published)
        and (
            abs(float(message.published_cmd_v)) > linear_threshold
            or abs(float(message.published_cmd_omega)) > angular_threshold
        )
    ]
    stamps = [value for value in stamps if value > 0.0]
    if not stamps:
        return None
    return min(stamps), max(stamps) + max(0.0, tail_sec)


def in_window(stamp, window):
    return window is not None and window[0] <= stamp <= window[1]


def validate(args):
    bag_path = Path(args.bag).expanduser().resolve()
    report_path = Path(args.report).expanduser().resolve()
    failures = []
    comparisons = []
    selections = []
    audits = []
    liquid = []
    available = {}

    if not bag_path.is_file():
        failures.append("bag does not exist: {}".format(bag_path))
    else:
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                available = bag.get_type_and_topic_info().topics
                expected_types = {
                    COMPARISON_TOPIC: "spmpc_local_planner/SloshEstimatorComparison",
                    SELECTION_TOPIC: "spmpc_local_planner/SloshObserverSelectionDebug",
                    AUDIT_TOPIC: "spmpc_local_planner/ControlCycleAudit",
                    LIQUID_TOPIC: "realsense_liquid_measurement/OnlineLiquidMeasurement",
                }
                for topic, expected_type in expected_types.items():
                    if topic not in available:
                        failures.append("missing required topic {}".format(topic))
                    elif available[topic].msg_type != expected_type:
                        failures.append(
                            "{} type is {}, expected {}".format(
                                topic, available[topic].msg_type, expected_type
                            )
                        )
                wanted = [topic for topic in expected_types if topic in available]
                for topic, message, bag_stamp in bag.read_messages(topics=wanted):
                    if topic == COMPARISON_TOPIC:
                        comparisons.append(message)
                    elif topic == SELECTION_TOPIC:
                        selections.append(message)
                    elif topic == AUDIT_TOPIC:
                        audits.append(message)
                    elif topic == LIQUID_TOPIC:
                        liquid.append((bag_stamp.to_sec(), message))
        except Exception as exc:
            failures.append("could not read bag: {}".format(exc))

    window = active_window(
        audits, args.linear_motion_threshold, args.angular_motion_threshold,
        args.motion_tail_sec,
    )
    if window is None:
        failures.append("no effective B0 motion window")
    active_comparisons = [message for message in comparisons if in_window(target_time(message), window)]
    active_selections = [
        message for message in selections
        if in_window(stamp_sec(message.header.stamp), window)
    ]
    active_audits = [
        message for message in audits
        if in_window(stamp_sec(message.header.stamp), window)
    ]
    active_liquid = [
        message for bag_stamp, message in liquid
        if in_window(stamp_sec(message.header.stamp) or bag_stamp, window)
    ]

    if len(active_comparisons) < 10:
        failures.append("fewer than 10 comparison messages in motion window")
    if len(active_selections) < 10:
        failures.append("fewer than 10 observer-selection messages in motion window")
    if len(active_audits) < 10:
        failures.append("fewer than 10 control audits in motion window")

    method_report = {}
    for method in METHODS:
        rows = [getattr(message, method) for message in active_comparisons]
        valid = [row for row in rows if bool(row.valid)]
        coverage = len(valid) / len(rows) if rows else 0.0
        applied = sum(bool(row.applied_to_solver) for row in rows)
        statuses = Counter(str(row.status) for row in rows)
        method_report[method.upper()] = {
            "count": len(rows),
            "valid": len(valid),
            "coverage": coverage,
            "applied_to_solver_count": applied,
            "statuses": dict(statuses),
        }
        if coverage < args.minimum_coverage:
            failures.append(
                "{} valid coverage {:.4f} is below {:.4f}".format(
                    method.upper(), coverage, args.minimum_coverage
                )
            )
        if applied:
            failures.append("{} entered the B0 solver {} times".format(method.upper(), applied))

    schema_bad = sum(int(message.schema_version) != 1 for message in active_comparisons)
    disabled = sum(not bool(message.comparison_enabled) for message in active_comparisons)
    liquid_consumed = sum(bool(message.solver_consumes_liquid) for message in active_comparisons)
    wrong_source = sum(
        int(message.selected_source) != 2 or str(message.selected_source_name) != "processed_imu"
        for message in active_comparisons
    )
    if schema_bad:
        failures.append("comparison schema mismatch count={}".format(schema_bad))
    if disabled:
        failures.append("comparison_enabled=false count={}".format(disabled))
    if liquid_consumed:
        failures.append("B0 comparison says solver consumed liquid count={}".format(liquid_consumed))
    if wrong_source:
        failures.append("comparison selected source is not processed_imu count={}".format(wrong_source))

    i1_valid = [message for message in active_comparisons if bool(message.i1.valid)]
    output_target_error = [
        stamp_sec(message.i1.output_state_stamp) - target_time(message)
        for message in i1_valid
    ]
    propagation = [float(message.i1.propagation_sec) for message in i1_valid]
    excitation_age = [float(message.i1.excitation_age_sec) for message in i1_valid]
    if any(abs(value) > 1.0e-6 for value in output_target_error):
        failures.append("I1 output timestamp differs from target timestamp")
    if any(value < -1.0e-9 or value > args.max_prediction_sec + 1.0e-9 for value in propagation):
        failures.append("I1 propagation exceeded [0, {:.6f}] s".format(args.max_prediction_sec))
    if any(
        value < -args.max_future_skew_sec - 1.0e-9
        or value > args.max_excitation_age_sec + 1.0e-9
        for value in excitation_age
    ):
        failures.append("I1 excitation age exceeded frozen bounds")

    fallback = sum(
        bool(message.fallback_active)
        or bool(message.fallback_latched)
        or int(message.effective_source) != 2
        or str(message.effective_source_name) != "processed_imu"
        or int(message.fallback_policy) != 2
        for message in active_selections
    )
    pipeline_not_ready = sum(not bool(message.imu_pipeline_ready) for message in active_selections)
    if fallback:
        failures.append("processed-IMU fallback/source contract failed count={}".format(fallback))
    if pipeline_not_ready:
        failures.append("processed-IMU pipeline not READY count={}".format(pipeline_not_ready))

    speed_limit_mismatch = sum(
        abs(float(message.v_safe_max) - args.expected_v_safe_max) > 1.0e-9
        for message in active_audits
    )
    overspeed = sum(
        abs(float(message.published_cmd_v))
        > args.expected_v_safe_max + args.speed_tolerance + 1.0e-9
        for message in active_audits
    )
    speed_faults = sum(
        bool(message.speed_safety_violation) or bool(message.speed_safety_latched)
        for message in active_audits
    )
    if speed_limit_mismatch:
        failures.append("v_safe_max differs from frozen 0.15 m/s")
    if overspeed:
        failures.append("published speed exceeded v_safe_max count={}".format(overspeed))
    if speed_faults:
        failures.append("speed safety violation/latch count={}".format(speed_faults))

    clean_rgb = sum(
        bool(message.valid)
        and bool(message.zero_locked)
        and int(message.status_code) == 0
        and not bool(message.any_clipped)
        and math.isfinite(float(message.height_max_lcr_mm))
        for message in active_liquid
    )
    rgb_coverage = clean_rgb / len(active_liquid) if active_liquid else 0.0
    if not active_liquid:
        failures.append("no online RGB liquid measurements in motion window")
    elif rgb_coverage < args.minimum_rgb_clean_coverage:
        failures.append(
            "clean RGB coverage {:.4f} is below {:.4f}".format(
                rgb_coverage, args.minimum_rgb_clean_coverage
            )
        )

    report = {
        "schema": "spmpc_slosh_nowcast_shadow_postflight_v1",
        "protocol": "SMPCC_slosh_state_nowcast_dev_v1",
        "status": "PASS" if not failures else "FAIL",
        "bag": str(bag_path),
        "failures": failures,
        "motion_window": {
            "start_sec": window[0] if window else None,
            "end_sec": window[1] if window else None,
        },
        "counts": {
            "comparison_all": len(comparisons),
            "comparison_motion": len(active_comparisons),
            "selection_motion": len(active_selections),
            "audit_motion": len(active_audits),
            "rgb_motion": len(active_liquid),
            "rgb_clean": clean_rgb,
        },
        "method_coverage": method_report,
        "contracts": {
            "schema_mismatch_count": schema_bad,
            "comparison_disabled_count": disabled,
            "solver_consumes_liquid_count": liquid_consumed,
            "wrong_selected_source_count": wrong_source,
            "fallback_or_wrong_policy_count": fallback,
            "pipeline_not_ready_count": pipeline_not_ready,
            "speed_limit_mismatch_count": speed_limit_mismatch,
            "overspeed_count": overspeed,
            "speed_fault_count": speed_faults,
            "rgb_clean_coverage": rgb_coverage,
        },
        "i1_timing": {
            "output_minus_target_sec": summary(output_target_error),
            "propagation_sec": summary(propagation),
            "excitation_age_sec": summary(excitation_age),
        },
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


def main():
    args = parse_args()
    report = validate(args)
    print("[slosh-nowcast-postflight] status={}".format(report["status"]))
    print("[slosh-nowcast-postflight] report={}".format(Path(args.report).expanduser().resolve()))
    for failure in report["failures"]:
        print("[slosh-nowcast-postflight] FAIL: {}".format(failure), file=sys.stderr)
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
