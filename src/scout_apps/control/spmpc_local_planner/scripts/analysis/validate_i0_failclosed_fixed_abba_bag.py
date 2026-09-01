#!/usr/bin/env python3
"""Fail-closed postflight for one literal B0/B_slosh I0+L22 ABBA unit.

This validator deliberately separates two facts which are easy to conflate:

* the selected/raw liquid observer is processed-IMU (I0), with fail-closed
  policy and no fallback; and
* ``fixed_closed_loop`` replaces the final solver input with the legacy L22
  command-history rollout.

For B0 the selected liquid state is diagnostic only.  The delay composer still
sets its liquid-application flag, but the six-state B0 solver does not consume
that field.  B_slosh must consume liquid and must satisfy the common-epoch
contract on every active solve.
"""

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import rosbag


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
ALIGNMENT_TOPIC = "/spmpc/debug/cmd_odom_alignment"
CONFIG_TOPIC = "/spmpc/debug/effective_config"
IMU_TOPIC = "/spmpc/debug/slosh_observer_imu"
INTERVENTION_TOPIC = "/spmpc/debug/command_intervention"
SELECTION_TOPIC = "/spmpc/debug/slosh_observer_selection"
SOLVER_INPUT_TOPIC = "/spmpc/debug/solver_input_state"
STATUS_TOPIC = "/spmpc/status"
WARM_START_TOPIC = "/spmpc/debug/warm_start"

REQUIRED_TOPICS = (
    AUDIT_TOPIC,
    ALIGNMENT_TOPIC,
    CONFIG_TOPIC,
    IMU_TOPIC,
    INTERVENTION_TOPIC,
    SELECTION_TOPIC,
    SOLVER_INPUT_TOPIC,
    STATUS_TOPIC,
    WARM_START_TOPIC,
)

BAD_ZERO_FIELDS = (
    "zero_due_to_solver_failure",
    "zero_due_to_waiting_for_odom",
    "zero_due_to_waiting_for_reference",
    "zero_due_to_waiting_for_tf",
    "zero_due_to_waiting_for_slosh_observer",
    "zero_due_to_terminal_spin_fail",
    "zero_due_to_tracking_safety",
    "zero_due_to_command_contract",
    "zero_due_to_speed_safety",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--condition", choices=("B0", "Bslosh"), required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--protocol", default="SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1")
    parser.add_argument("--expected-v-ref", type=float, default=0.20)
    parser.add_argument("--expected-v-safe-max", type=float, default=0.25)
    parser.add_argument("--expected-linear-delay-sec", type=float, default=0.15)
    parser.add_argument("--expected-angular-delay-sec", type=float, default=0.22)
    parser.add_argument("--minimum-application-fraction", type=float, default=1.0)
    parser.add_argument("--minimum-samples", type=int, default=10)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.03)
    parser.add_argument("--angular-motion-threshold", type=float, default=0.03)
    return parser.parse_args()


def stamp_sec(value):
    try:
        return float(value.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0


def message_stamp(message, bag_stamp):
    value = stamp_sec(getattr(getattr(message, "header", None), "stamp", None))
    return value if value > 0.0 else float(bag_stamp)


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def close(value, expected, tolerance=1.0e-6):
    return finite(value) and abs(float(value) - float(expected)) <= tolerance


def parse_multiarray(message):
    dimensions = getattr(getattr(message, "layout", None), "dim", [])
    label = dimensions[0].label if dimensions else ""
    names = [item.strip() for item in label.split(",") if item.strip()]
    values = list(getattr(message, "data", []))
    return {
        names[index] if index < len(names) else "field_{}".format(index): float(value)
        for index, value in enumerate(values)
    }


def in_window(records, start, end):
    if start is None or end is None:
        return []
    return [item for item in records if start <= item[0] <= end]


def true_fraction(records, field):
    values = [row.get(field) for _, row in records if finite(row.get(field))]
    if not values:
        return None
    return sum(float(value) > 0.5 for value in values) / len(values)


def require_fraction(failures, label, observed, minimum):
    if observed is None:
        failures.append("{} has no readable samples".format(label))
    elif observed + 1.0e-12 < minimum:
        failures.append(
            "{} fraction {:.6f} < {:.6f}".format(label, observed, minimum)
        )


def validate(args):
    bag_path = Path(args.bag).expanduser().resolve()
    report_path = Path(args.report).expanduser().resolve()
    expected_variant = "B0" if args.condition == "B0" else "B_slosh"
    expected_slosh = args.condition == "Bslosh"
    expected_weight = 5.0 if expected_slosh else 0.0
    failures = []

    audits = []
    alignments = []
    configs = []
    imu_rows = []
    interventions = []
    selections = []
    solver_inputs = []
    statuses = []
    warm_starts = []

    if not close(args.minimum_application_fraction, 1.0, tolerance=1.0e-12):
        failures.append(
            "minimum application fraction must be exactly 1.0 for this protocol"
        )

    if not bag_path.is_file():
        failures.append("bag does not exist: {}".format(bag_path))
    elif Path(str(bag_path) + ".active").exists():
        failures.append("bag still has a .active sibling")
    else:
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                available = bag.get_type_and_topic_info().topics
                for topic in REQUIRED_TOPICS:
                    if topic not in available:
                        failures.append("missing required topic {}".format(topic))
                wanted = [topic for topic in REQUIRED_TOPICS if topic in available]
                for topic, message, bag_stamp in bag.read_messages(topics=wanted):
                    when = message_stamp(message, bag_stamp.to_sec())
                    if topic == AUDIT_TOPIC:
                        audits.append((when, message))
                    elif topic == ALIGNMENT_TOPIC:
                        alignments.append((when, parse_multiarray(message)))
                    elif topic == CONFIG_TOPIC:
                        configs.append((when, parse_multiarray(message)))
                    elif topic == IMU_TOPIC:
                        imu_rows.append((when, message))
                    elif topic == INTERVENTION_TOPIC:
                        interventions.append((when, parse_multiarray(message)))
                    elif topic == SELECTION_TOPIC:
                        selections.append((when, message))
                    elif topic == SOLVER_INPUT_TOPIC:
                        solver_inputs.append((when, parse_multiarray(message)))
                    elif topic == STATUS_TOPIC:
                        statuses.append((when, str(message.data)))
                    elif topic == WARM_START_TOPIC:
                        warm_starts.append((when, parse_multiarray(message)))
        except Exception as exc:  # noqa: BLE001
            failures.append("could not read bag: {}".format(exc))

    active_command_stamps = [
        when
        for when, message in audits
        if bool(message.command_was_published)
        and (
            abs(float(message.published_cmd_v)) > args.linear_motion_threshold
            or abs(float(message.published_cmd_omega)) > args.angular_motion_threshold
        )
    ]
    motion_start = min(active_command_stamps) if active_command_stamps else None
    motion_end = max(active_command_stamps) if active_command_stamps else None
    if motion_start is None:
        failures.append("no effective motion window")

    motion_audits = in_window(audits, motion_start, motion_end)
    motion_alignments = in_window(alignments, motion_start, motion_end)
    motion_configs = in_window(configs, motion_start, motion_end)
    motion_imu = in_window(imu_rows, motion_start, motion_end)
    motion_interventions = in_window(interventions, motion_start, motion_end)
    motion_selections = in_window(selections, motion_start, motion_end)
    motion_solver_inputs = in_window(solver_inputs, motion_start, motion_end)
    motion_statuses = in_window(statuses, motion_start, motion_end)
    motion_warm_starts = in_window(warm_starts, motion_start, motion_end)

    minimum = args.minimum_samples
    for label, records in (
        ("control audits", motion_audits),
        ("alignment diagnostics", motion_alignments),
        ("effective configs", motion_configs),
        ("processed-IMU diagnostics", motion_imu),
        ("command interventions", motion_interventions),
        ("observer selections", motion_selections),
        ("solver-input diagnostics", motion_solver_inputs),
        ("warm-start diagnostics", motion_warm_starts),
    ):
        if len(records) < minimum:
            failures.append("{} {} < {}".format(label, len(records), minimum))

    config = motion_configs[-1][1] if motion_configs else {}
    expected_config = {
        "solver_backend_code": 1.0,
        "delay_phase_mode_code": 3.0,
        "delay_linear_sec": args.expected_linear_delay_sec,
        "delay_angular_sec": args.expected_angular_delay_sec,
        "slosh_enable": 1.0 if expected_slosh else 0.0,
        "slosh_constraint_enable": 0.0,
        "smooth_priority_enable": 0.0,
        "w_slosh": expected_weight,
        "v_ref": args.expected_v_ref,
        "state_timing_require_common_epoch": 1.0,
        "shared_linear_accel_limit_enable": 0.0,
        "shared_angular_limit_enable": 0.0,
        "speed_safety_enable": 1.0,
        "v_safe_max": args.expected_v_safe_max,
    }
    for field, expected in expected_config.items():
        if not close(config.get(field), expected):
            failures.append(
                "effective_config {}={!r}, expected {}".format(
                    field, config.get(field), expected
                )
            )

    selection_bad = 0
    consumption_bad = 0
    selection_reset_epochs = set()
    for _, message in motion_selections:
        selection_reset_epochs.add(int(message.imu_reset_epoch))
        if not (
            bool(message.configured)
            and bool(message.valid)
            and not bool(message.fallback_active)
            and not bool(message.fallback_latched)
            and int(message.nominal_source) == 2
            and str(message.nominal_source_name) == "processed_imu"
            and int(message.effective_source) == 2
            and str(message.effective_source_name) == "processed_imu"
            and int(message.fallback_policy) == 2
            and str(message.fallback_policy_name) == "fail_closed"
            and str(message.status) == "NOMINAL_PROCESSED_IMU"
            and bool(message.imu_pipeline_ready)
            and bool(message.imu_fresh)
        ):
            selection_bad += 1
        if bool(message.solver_consumes_selected_state) != expected_slosh:
            consumption_bad += 1
    if selection_bad:
        failures.append("processed-IMU/fail-closed selection mismatch count={}".format(selection_bad))
    if consumption_bad:
        failures.append("solver liquid-consumption mismatch count={}".format(consumption_bad))
    if len(selection_reset_epochs) != 1:
        failures.append(
            "observer-selection reset epoch changed during motion: {}".format(
                sorted(selection_reset_epochs)
            )
        )

    imu_bad = sum(
        not (
            bool(message.configured)
            and bool(message.valid)
            and str(message.input_status) == "READY"
            and bool(message.bias_ready)
            and bool(message.filter_ready)
        )
        for _, message in motion_imu
    )
    imu_reset_epochs = {int(message.reset_epoch) for _, message in motion_imu}
    if imu_bad:
        failures.append("processed-IMU READY contract mismatch count={}".format(imu_bad))
    if len(imu_reset_epochs) != 1:
        failures.append(
            "processed-IMU reset epoch changed during motion: {}".format(
                sorted(imu_reset_epochs)
            )
        )

    application_series = {
        "history_complete": (motion_alignments, "history_complete"),
        "shadow_valid": (motion_alignments, "shadow_valid"),
        "fixed_closed_loop_configured": (
            motion_alignments,
            "fixed_closed_loop_configured",
        ),
        "fixed_closed_loop_applied": (
            motion_alignments,
            "fixed_closed_loop_applied",
        ),
        "robot_delay_compensation_applied": (
            motion_solver_inputs,
            "robot_delay_compensation_applied",
        ),
        # The composer sets this flag for B0 too; B0 still does not consume it.
        "liquid_delay_compensation_applied": (
            motion_solver_inputs,
            "liquid_delay_compensation_applied",
        ),
    }
    application_fractions = {}
    application_unreadable_counts = {}
    for label, (records, field) in application_series.items():
        observed = true_fraction(records, field)
        application_fractions[label] = observed
        unreadable = sum(not finite(row.get(field)) for _, row in records)
        application_unreadable_counts[label] = unreadable
        if unreadable:
            failures.append(
                "{} has unreadable motion samples count={}".format(label, unreadable)
            )
        require_fraction(
            failures, label, observed, args.minimum_application_fraction
        )

    warm_start_unreadable_count = sum(
        not finite(row.get("used_fallback")) for _, row in motion_warm_starts
    )
    warm_start_fallback_count = sum(
        finite(row.get("used_fallback")) and float(row["used_fallback"]) > 0.5
        for _, row in motion_warm_starts
    )
    if warm_start_unreadable_count:
        failures.append(
            "warm-start used_fallback has unreadable motion samples count={}".format(
                warm_start_unreadable_count
            )
        )
    if warm_start_fallback_count:
        failures.append(
            "warm-start fallback used during motion count={}".format(
                warm_start_fallback_count
            )
        )
    alignment_mode_bad = sum(
        not close(row.get("mode_code"), 3.0) for _, row in motion_alignments
    )
    solver_source_bad = sum(
        not close(row.get("source_code"), 2.0) for _, row in motion_solver_inputs
    )
    if alignment_mode_bad:
        failures.append("delay alignment mode is not fixed_closed_loop count={}".format(alignment_mode_bad))
    if solver_source_bad:
        failures.append("solver-input source is not processed_imu count={}".format(solver_source_bad))

    audit_failures = collections.Counter()
    common_epoch_bad = 0
    for _, message in motion_audits:
        if str(message.variant) != expected_variant:
            audit_failures["wrong_variant"] += 1
        if int(message.observer_source) != 2:
            audit_failures["wrong_observer_source"] += 1
        if bool(message.solve_attempted) and not bool(message.solve_success):
            audit_failures["solver_failure"] += 1
        if bool(message.command_contract_violation):
            audit_failures["command_contract"] += 1
        if bool(message.safety_gate_intervened) and not bool(message.terminal_phase):
            audit_failures["safety_gate"] += 1
        if bool(message.linear_limited) or bool(message.angular_rate_limited) or bool(message.angular_accel_limited):
            audit_failures["post_solver_limiter"] += 1
        if bool(message.zero_due_to_speed_safety) or bool(message.speed_safety_violation) or bool(message.speed_safety_latched):
            audit_failures["speed_safety"] += 1
        if not close(message.v_safe_max, args.expected_v_safe_max):
            audit_failures["wrong_speed_limit"] += 1
        if bool(message.state_alignment_required) != expected_slosh:
            audit_failures["wrong_alignment_requirement"] += 1
        if not bool(message.state_time_aligned):
            common_epoch_bad += 1
            continue
        robot_stamp = stamp_sec(message.robot_state_stamp)
        liquid_stamp = stamp_sec(message.liquid_state_stamp)
        solver_epoch = stamp_sec(message.solver_input_epoch)
        if (
            str(message.state_alignment_status) != "DELAY_PREDICTED_COMMON_EPOCH"
            or robot_stamp <= 0.0
            or max(abs(robot_stamp - liquid_stamp), abs(robot_stamp - solver_epoch)) > 1.0e-6
        ):
            common_epoch_bad += 1
    if audit_failures:
        failures.append("control-audit failures: {}".format(dict(audit_failures)))
    if common_epoch_bad:
        failures.append("fixed common-epoch mismatch count={}".format(common_epoch_bad))

    zero_counts = {
        field: sum(row.get(field, 0.0) > 0.5 for _, row in motion_interventions)
        for field in BAD_ZERO_FIELDS
    }
    nonzero_zero_counts = {key: value for key, value in zero_counts.items() if value}
    if nonzero_zero_counts:
        failures.append("command zero/failure gate observed: {}".format(nonzero_zero_counts))

    bad_statuses = collections.Counter(
        status
        for _, status in motion_statuses
        if "FAIL" in status.upper() or "WAITING" in status.upper()
    )
    if bad_statuses:
        failures.append("failure/wait status during motion: {}".format(dict(bad_statuses)))
    goal_reached = any(
        status == "GOAL_REACHED"
        and motion_start is not None
        and when >= motion_start
        for when, status in statuses
    )
    if not goal_reached:
        failures.append("GOAL_REACHED was not recorded")

    report = {
        "schema": "spmpc_i0_failclosed_fixed_abba_postflight_v1",
        "protocol": args.protocol,
        "status": "PASS" if not failures else "FAIL",
        "condition": args.condition,
        "expected_variant": expected_variant,
        "bag": str(bag_path),
        "motion_window": {"start_sec": motion_start, "end_sec": motion_end},
        "counts": {
            "audit_motion": len(motion_audits),
            "alignment_motion": len(motion_alignments),
            "imu_motion": len(motion_imu),
            "selection_motion": len(motion_selections),
            "solver_input_motion": len(motion_solver_inputs),
            "warm_start_motion": len(motion_warm_starts),
        },
        "contracts": {
            "selected_observer": "processed_imu_I0",
            "fallback_policy": "fail_closed",
            "solver_consumes_liquid": expected_slosh,
            "final_liquid_input_when_consumed": "L22_command_history_rollout" if expected_slosh else "not_consumed",
            "delay_mode": "fixed_closed_loop",
            "delay_sec": [args.expected_linear_delay_sec, args.expected_angular_delay_sec],
            "selection_bad_count": selection_bad,
            "selection_consumption_bad_count": consumption_bad,
            "selection_reset_epochs": sorted(selection_reset_epochs),
            "imu_bad_count": imu_bad,
            "imu_reset_epochs": sorted(imu_reset_epochs),
            "application_fractions": application_fractions,
            "application_unreadable_counts": application_unreadable_counts,
            "warm_start_fallback_count": warm_start_fallback_count,
            "warm_start_unreadable_count": warm_start_unreadable_count,
            "alignment_mode_bad_count": alignment_mode_bad,
            "solver_source_bad_count": solver_source_bad,
            "common_epoch_bad_count": common_epoch_bad,
            "audit_failure_counts": dict(audit_failures),
            "zero_reason_counts": zero_counts,
            "goal_reached": goal_reached,
        },
        "effective_config_last": config,
        "failures": failures,
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
    print("[i0-fixed-postflight] status={}".format(report["status"]))
    print("[i0-fixed-postflight] report={}".format(Path(args.report).expanduser().resolve()))
    for failure in report["failures"]:
        print("[i0-fixed-postflight] FAIL: {}".format(failure), file=sys.stderr)
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
