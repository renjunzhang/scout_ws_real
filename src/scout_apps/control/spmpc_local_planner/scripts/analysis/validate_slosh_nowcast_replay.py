#!/usr/bin/env python3
"""Validate an isolated, no-hardware slosh-nowcast planner replay bag."""

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

import rosbag


COMPARISON_TOPIC = "/spmpc/debug/slosh_estimator_comparison"
SELECTION_TOPIC = "/spmpc/debug/slosh_observer_selection"
AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--report", required=True)
    parser.add_argument("--isolated-cmd-topic", default="/spmpc_nowcast_replay/cmd_vel")
    parser.add_argument("--minimum-i1-coverage", type=float, default=0.95)
    parser.add_argument("--expected-v-safe-max", type=float, default=0.15)
    parser.add_argument("--speed-tolerance", type=float, default=0.0001)
    return parser.parse_args()


def stamp_sec(stamp):
    try:
        return float(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def validate(args):
    path = Path(args.bag).expanduser().resolve()
    failures = []
    comparisons = []
    selections = []
    audits = []
    commands = []
    topics = {}
    if not path.is_file():
        failures.append("replay output bag is missing")
    else:
        try:
            with rosbag.Bag(str(path), "r") as bag:
                topics = bag.get_type_and_topic_info().topics
                expected = {
                    COMPARISON_TOPIC: "spmpc_local_planner/SloshEstimatorComparison",
                    SELECTION_TOPIC: "spmpc_local_planner/SloshObserverSelectionDebug",
                    AUDIT_TOPIC: "spmpc_local_planner/ControlCycleAudit",
                    args.isolated_cmd_topic: "geometry_msgs/Twist",
                }
                for topic, msg_type in expected.items():
                    if topic not in topics:
                        failures.append("missing replay topic {}".format(topic))
                    elif topics[topic].msg_type != msg_type:
                        failures.append("{} has unexpected type {}".format(topic, topics[topic].msg_type))
                if "/cmd_vel" in topics:
                    failures.append("replay output contains real /cmd_vel topic")
                wanted = [topic for topic in expected if topic in topics]
                for topic, message, _ in bag.read_messages(topics=wanted):
                    if topic == COMPARISON_TOPIC:
                        comparisons.append(message)
                    elif topic == SELECTION_TOPIC:
                        selections.append(message)
                    elif topic == AUDIT_TOPIC:
                        audits.append(message)
                    else:
                        commands.append(message)
        except Exception as exc:
            failures.append("could not read replay bag: {}".format(exc))

    if len(comparisons) < 10:
        failures.append("replay produced fewer than 10 comparison cycles")
    if len(audits) < 10:
        failures.append("replay produced fewer than 10 audit cycles")
    if not commands:
        failures.append("replay produced no isolated commands")

    first_valid = next((index for index, msg in enumerate(comparisons) if bool(msg.i1.valid)), None)
    evaluated = comparisons[first_valid:] if first_valid is not None else comparisons
    i1_valid = [msg for msg in evaluated if bool(msg.i1.valid)]
    coverage = len(i1_valid) / len(evaluated) if evaluated else 0.0
    if coverage < args.minimum_i1_coverage:
        failures.append(
            "I1 replay coverage {:.4f} is below {:.4f}".format(
                coverage, args.minimum_i1_coverage
            )
        )
    schema_bad = sum(int(msg.schema_version) != 1 for msg in evaluated)
    disabled = sum(not bool(msg.comparison_enabled) for msg in evaluated)
    liquid_consumed = sum(bool(msg.solver_consumes_liquid) for msg in evaluated)
    wrong_source = sum(
        int(msg.selected_source) != 2 or str(msg.selected_source_name) != "processed_imu"
        for msg in evaluated
    )
    any_applied = sum(
        bool(getattr(msg, method).applied_to_solver)
        for msg in evaluated for method in ("o0", "i0", "i1", "l22")
    )
    output_epoch_error = [
        stamp_sec(msg.i1.output_state_stamp) - stamp_sec(msg.target_stamp)
        for msg in i1_valid
    ]
    propagation_bad = sum(
        not math.isfinite(float(msg.i1.propagation_sec))
        or float(msg.i1.propagation_sec) < -1.0e-9
        or float(msg.i1.propagation_sec) > 0.050000001
        for msg in i1_valid
    )
    if schema_bad:
        failures.append("schema mismatch count={}".format(schema_bad))
    if disabled:
        failures.append("comparison disabled count={}".format(disabled))
    if liquid_consumed:
        failures.append("B0 consumed liquid count={}".format(liquid_consumed))
    if wrong_source:
        failures.append("wrong selected source count={}".format(wrong_source))
    if any_applied:
        failures.append("shadow method applied to solver count={}".format(any_applied))
    if any(abs(value) > 1.0e-6 for value in output_epoch_error):
        failures.append("I1 replay output timestamp differs from target")
    if propagation_bad:
        failures.append("I1 replay propagation bound failures={}".format(propagation_bad))

    valid_selections = [msg for msg in selections if bool(msg.valid)]
    if len(valid_selections) < 10:
        failures.append("replay produced fewer than 10 valid processed-IMU selections")
    selection_failures = sum(
        int(msg.effective_source) != 2
        or str(msg.effective_source_name) != "processed_imu"
        or int(msg.fallback_policy) != 2
        or bool(msg.fallback_active)
        or bool(msg.fallback_latched)
        or not bool(msg.imu_pipeline_ready)
        for msg in valid_selections
    )
    if selection_failures:
        failures.append("selection/fallback failures={}".format(selection_failures))

    solve_audits = [msg for msg in audits if bool(msg.solve_attempted)]
    if len(solve_audits) < 10:
        failures.append("replay produced fewer than 10 solve audits")
    audit_failures = sum(
        str(msg.variant) != "B0"
        or bool(msg.state_alignment_required)
        or str(msg.state_alignment_status) != "LIQUID_NOT_CONSUMED"
        or abs(float(msg.v_safe_max) - args.expected_v_safe_max) > 1.0e-9
        or bool(msg.speed_safety_violation)
        or bool(msg.speed_safety_latched)
        or abs(float(msg.published_cmd_v))
        > args.expected_v_safe_max + args.speed_tolerance + 1.0e-9
        for msg in solve_audits
    )
    if audit_failures:
        failures.append("B0 audit/speed contract failures={}".format(audit_failures))
    command_overspeed = sum(
        abs(float(msg.linear.x)) > args.expected_v_safe_max + args.speed_tolerance + 1.0e-9
        for msg in commands
    )
    if command_overspeed:
        failures.append("isolated command overspeed count={}".format(command_overspeed))

    report = {
        "schema": "spmpc_slosh_nowcast_isolated_replay_v1",
        "status": "PASS" if not failures else "FAIL",
        "bag": str(path),
        "isolated_cmd_topic": args.isolated_cmd_topic,
        "real_cmd_topic_present": "/cmd_vel" in topics,
        "failures": failures,
        "counts": {
            "comparisons": len(comparisons),
            "evaluated_after_first_valid": len(evaluated),
            "i1_valid": len(i1_valid),
            "selections": len(selections),
            "audits": len(audits),
            "solve_audits": len(solve_audits),
            "valid_processed_imu_selections": len(valid_selections),
            "isolated_commands": len(commands),
        },
        "contracts": {
            "i1_coverage": coverage,
            "schema_mismatch_count": schema_bad,
            "comparison_disabled_count": disabled,
            "liquid_consumed_count": liquid_consumed,
            "wrong_source_count": wrong_source,
            "shadow_applied_count": any_applied,
            "propagation_failure_count": propagation_bad,
            "selection_failure_count": selection_failures,
            "audit_failure_count": audit_failures,
            "isolated_command_overspeed_count": command_overspeed,
            "i1_statuses": dict(Counter(str(msg.i1.status) for msg in evaluated)),
        },
    }
    report_path = Path(args.report).expanduser().resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


def main():
    args = parse_args()
    report = validate(args)
    print("[slosh-nowcast-replay] status={}".format(report["status"]))
    print("[slosh-nowcast-replay] report={}".format(Path(args.report).expanduser().resolve()))
    for failure in report["failures"]:
        print("[slosh-nowcast-replay] FAIL: {}".format(failure), file=sys.stderr)
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
