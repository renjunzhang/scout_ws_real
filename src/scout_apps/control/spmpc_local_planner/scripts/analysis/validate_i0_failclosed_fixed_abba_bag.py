#!/usr/bin/env python3
"""Fail-closed postflight for one I0 B0/Bslosh ABBA unit.

This validator deliberately separates two facts which are easy to conflate:

* the selected/raw liquid observer is processed-IMU (I0), with fail-closed
  policy and no fallback; and
* the configured execution model and legacy delay phase match the registered
  profile; and
* command/actual diagnostics are complete for explicit-actuator profiles.

For B0 the selected liquid state is diagnostic only.  B_slosh must consume
liquid and satisfy the common-epoch contract on every active solve.
"""

import argparse
import collections
import json
import math
import sys
from pathlib import Path

import rosbag

ANALYSIS_DIR = Path(__file__).resolve().parent
if str(ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(ANALYSIS_DIR))

from liquid_cost_window_contract import (  # noqa: E402
    parse_expected_config_items,
    validate_config_fields,
    validate_deep_cycle_coverage,
    validate_effective_config as validate_liquid_effective_config,
    validate_metadata as validate_liquid_metadata,
    validate_snapshot_stage_weights,
)


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
ALIGNMENT_TOPIC = "/spmpc/debug/cmd_odom_alignment"
CONFIG_TOPIC = "/spmpc/debug/effective_config"
IMU_TOPIC = "/spmpc/debug/slosh_observer_imu"
INTERVENTION_TOPIC = "/spmpc/debug/command_intervention"
SELECTION_TOPIC = "/spmpc/debug/slosh_observer_selection"
SOLVER_INPUT_TOPIC = "/spmpc/debug/solver_input_state"
STATUS_TOPIC = "/spmpc/status"
WARM_START_TOPIC = "/spmpc/debug/warm_start"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"

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
    parser.add_argument(
        "--expected-w-slosh",
        type=float,
        help="override the historical Bslosh=5.0/B0=0.0 weight contract",
    )
    parser.add_argument("--report", required=True)
    parser.add_argument("--protocol", default="SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1")
    parser.add_argument(
        "--report-schema",
        default="spmpc_i0_failclosed_fixed_abba_postflight_v1",
    )
    parser.add_argument("--expected-variant")
    parser.add_argument("--expected-slosh-cost-horizon-steps", type=int)
    parser.add_argument("--expected-slosh-cost-tail-discount", type=float)
    parser.add_argument("--expected-slosh-eta-dot-ratio", type=float)
    parser.add_argument("--expected-robot-horizon-steps", type=int, default=60)
    parser.add_argument("--expected-dt-sec", type=float)
    parser.add_argument("--expected-control-frequency-hz", type=float)
    parser.add_argument(
        "--expected-config",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="repeatable effective_config field expectation (deep mode only)",
    )
    parser.add_argument("--expected-v-ref", type=float, default=0.20)
    parser.add_argument("--expected-v-safe-max", type=float, default=0.25)
    parser.add_argument("--expected-linear-delay-sec", type=float, default=0.15)
    parser.add_argument("--expected-angular-delay-sec", type=float, default=0.22)
    parser.add_argument("--expected-delay-mode-code", type=float, default=3.0)
    parser.add_argument(
        "--require-legacy-delay-application",
        choices=("true", "false"),
        default="true",
    )
    parser.add_argument("--expected-execution-model-code", type=float)
    parser.add_argument("--expected-state-width", type=int)
    parser.add_argument("--minimum-solver-schema-version", type=int, default=2)
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
    expected_variant = args.expected_variant or (
        "B0" if args.condition == "B0" else "B_slosh"
    )
    expected_slosh = args.condition == "Bslosh"
    expected_weight = (
        args.expected_w_slosh
        if args.expected_w_slosh is not None
        else 5.0 if expected_slosh else 0.0
    )
    legacy_delay_expected = args.require_legacy_delay_application == "true"
    explicit_actuator_expected = args.expected_execution_model_code == 1.0
    full_horizon_da_expected = (
        explicit_actuator_expected and args.expected_state_width in (24, 28)
    )
    failures = []
    if not math.isfinite(expected_weight) or expected_weight < 0.0:
        failures.append("expected w_slosh must be finite and non-negative")
    if not expected_slosh and abs(expected_weight) > 1.0e-12:
        failures.append("B0 requires expected w_slosh=0")

    extra_expected_config, expected_config_parse_failures = (
        parse_expected_config_items(args.expected_config)
    )
    failures.extend(expected_config_parse_failures)

    deep_values = (
        args.expected_slosh_cost_horizon_steps,
        args.expected_slosh_cost_tail_discount,
        args.expected_slosh_eta_dot_ratio,
        args.expected_dt_sec,
        args.expected_control_frequency_hz,
    )
    deep_cost_contract = all(value is not None for value in deep_values)
    if any(value is not None for value in deep_values) and not deep_cost_contract:
        failures.append(
            "liquid cost deep contract requires steps, tail, eta-dot ratio, dt, and control frequency together"
        )
    if args.expected_config and not deep_cost_contract:
        failures.append("--expected-config requires the deep liquid cost contract")
    if args.expected_robot_horizon_steps <= 0:
        failures.append("expected robot horizon steps must be positive")

    audits = []
    alignments = []
    configs = []
    imu_rows = []
    interventions = []
    selections = []
    solver_inputs = []
    statuses = []
    warm_starts = []
    snapshots = []
    horizons = []

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
                required_topics = list(REQUIRED_TOPICS)
                if deep_cost_contract:
                    required_topics.extend((SNAPSHOT_TOPIC, HORIZON_TOPIC))
                for topic in required_topics:
                    if topic not in available:
                        failures.append("missing required topic {}".format(topic))
                wanted = [topic for topic in required_topics if topic in available]
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
                    elif topic == SNAPSHOT_TOPIC:
                        snapshots.append((when, message))
                    elif topic == HORIZON_TOPIC:
                        horizons.append((when, message))
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
        "delay_phase_mode_code": args.expected_delay_mode_code,
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
    if legacy_delay_expected:
        expected_config.update(
            {
                "delay_linear_sec": args.expected_linear_delay_sec,
                "delay_angular_sec": args.expected_angular_delay_sec,
            }
        )
    if args.expected_execution_model_code is not None:
        expected_config["execution_model_mode_code"] = (
            args.expected_execution_model_code
        )
    config_field_failures = []
    if deep_cost_contract:
        for field, expected in extra_expected_config.items():
            if field in expected_config and not close(
                expected_config[field], expected
            ):
                config_field_failures.append(
                    "expected config {}={} conflicts with frozen value {}".format(
                        field, expected, expected_config[field]
                    )
                )
                continue
            expected_config[field] = expected
    configs_to_validate = (
        motion_configs if deep_cost_contract else motion_configs[-1:]
    )
    if not configs_to_validate:
        configs_to_validate = [(0.0, {})]
    deep_config_failures = []
    for config_index, (_, config_row) in enumerate(configs_to_validate):
        label = "effective_config[{}]".format(config_index)
        config_field_failures.extend(
            validate_config_fields(config_row, label, expected_config)
        )
        if deep_cost_contract:
            deep_config_failures.extend(
                validate_liquid_effective_config(
                    config_row,
                    label,
                    expected_slosh,
                    expected_weight,
                    args.expected_slosh_eta_dot_ratio,
                    args.expected_slosh_cost_horizon_steps,
                    args.expected_slosh_cost_tail_discount,
                    args.expected_robot_horizon_steps,
                    args.expected_dt_sec,
                    args.expected_control_frequency_hz,
                )
            )
    failures.extend(config_field_failures)
    failures.extend(deep_config_failures)

    solve_cycle_sequence = [
        int(message.cycle_id)
        for _, message in motion_audits
        if bool(message.solve_attempted)
    ]
    solve_cycle_ids = set(solve_cycle_sequence)
    cycle_audit_failures = []
    deep_selection_failures = []
    snapshot_coverage_failures = []
    horizon_coverage_failures = []
    selections_to_validate = motion_selections
    valid_snapshots = []
    valid_horizons = []
    if deep_cost_contract:
        coverage = validate_deep_cycle_coverage(
            motion_audits,
            selections,
            snapshots,
            horizons,
        )
        solve_cycle_sequence = list(coverage.solve_cycle_sequence)
        solve_cycle_ids = coverage.solve_cycle_ids
        cycle_audit_failures = list(coverage.audit_failures)
        deep_selection_failures = list(coverage.selection_failures)
        snapshot_coverage_failures = list(coverage.snapshot_failures)
        horizon_coverage_failures = list(coverage.horizon_failures)
        selections_to_validate = list(coverage.selection_records)
        valid_snapshots = [
            message for _, message in coverage.valid_snapshot_records
        ]
        valid_horizons = [
            message for _, message in coverage.valid_horizon_records
        ]
        failures.extend(cycle_audit_failures)
        failures.extend(deep_selection_failures)

    selection_bad = 0
    consumption_bad = 0
    selection_reset_epochs = set()
    for _, message in selections_to_validate:
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
        if legacy_delay_expected:
            require_fraction(
                failures, label, observed, args.minimum_application_fraction
            )
        elif observed is None:
            failures.append("{} has no readable samples".format(label))
        elif observed > 1.0 - args.minimum_application_fraction + 1.0e-12:
            failures.append(
                "{} true fraction {:.6f} exceeds {:.6f}".format(
                    label,
                    observed,
                    1.0 - args.minimum_application_fraction,
                )
            )

    if legacy_delay_expected:
        for label, field in (
            ("history_complete", "history_complete"),
            ("shadow_valid", "shadow_valid"),
        ):
            observed = true_fraction(motion_alignments, field)
            application_fractions[label] = observed
            require_fraction(
                failures, label, observed, args.minimum_application_fraction
            )

    actuator_valid_fraction = true_fraction(
        motion_solver_inputs, "actuator_state_valid"
    )
    if explicit_actuator_expected:
        require_fraction(
            failures,
            "actuator_state_valid",
            actuator_valid_fraction,
            args.minimum_application_fraction,
        )
    if full_horizon_da_expected:
        unreadable_accel_memory = sum(
            not finite(row.get("a_cmd_memory"))
            for _, row in motion_solver_inputs
        )
        if unreadable_accel_memory:
            failures.append(
                "solver-input a_cmd_memory unreadable count={}".format(
                    unreadable_accel_memory
                )
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
        not close(row.get("mode_code"), args.expected_delay_mode_code)
        for _, row in motion_alignments
    )
    solver_source_bad = sum(
        not close(row.get("source_code"), 2.0) for _, row in motion_solver_inputs
    )
    if alignment_mode_bad:
        failures.append(
            "delay alignment mode mismatch count={}".format(alignment_mode_bad)
        )
    if solver_source_bad:
        failures.append("solver-input source is not processed_imu count={}".format(solver_source_bad))

    audit_failures = collections.Counter()
    common_epoch_bad = 0
    expected_alignment_status = (
        "DELAY_PREDICTED_COMMON_EPOCH"
        if legacy_delay_expected
        else "EXPLICIT_ACTUATOR_PREFIX_ROLLOUT"
    )
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
            str(message.state_alignment_status) != expected_alignment_status
            or robot_stamp <= 0.0
            or max(abs(robot_stamp - liquid_stamp), abs(robot_stamp - solver_epoch)) > 1.0e-6
        ):
            common_epoch_bad += 1
    if audit_failures:
        failures.append("control-audit failures: {}".format(dict(audit_failures)))
    if common_epoch_bad:
        failures.append("common-epoch mismatch count={}".format(common_epoch_bad))

    liquid_artifact_failures = (
        snapshot_coverage_failures + horizon_coverage_failures
    )
    if deep_cost_contract:
        for message in valid_snapshots:
            label = "snapshot cycle {}".format(message.cycle_id)
            liquid_artifact_failures.extend(
                validate_liquid_metadata(
                    message,
                    label,
                    expected_variant,
                    expected_slosh,
                    args.expected_slosh_cost_horizon_steps,
                    args.expected_slosh_cost_tail_discount,
                    args.expected_robot_horizon_steps,
                    args.expected_dt_sec,
                )
            )
            if expected_slosh:
                liquid_artifact_failures.extend(
                    validate_snapshot_stage_weights(
                        message,
                        label,
                        expected_weight,
                        args.expected_slosh_eta_dot_ratio,
                        args.expected_slosh_cost_horizon_steps,
                        args.expected_slosh_cost_tail_discount,
                )
            )
            if int(message.schema_version) < args.minimum_solver_schema_version:
                liquid_artifact_failures.append(
                    "{}: schema_version={} < {}".format(
                        label,
                        message.schema_version,
                        args.minimum_solver_schema_version,
                    )
                )
            if args.expected_state_width is not None:
                state_width = int(message.state_width)
                if state_width != args.expected_state_width:
                    liquid_artifact_failures.append(
                        "{}: state_width={} expected {}".format(
                            label, state_width, args.expected_state_width
                        )
                    )
                expected_rows = int(message.horizon_steps) + 1
                if len(message.initial_guess_states) != expected_rows * state_width:
                    liquid_artifact_failures.append(
                        "{}: initial_guess_states length={} expected {}".format(
                            label,
                            len(message.initial_guess_states),
                            expected_rows * state_width,
                        )
                    )
                previous_len = len(message.previous_solution_states)
                if previous_len not in (0, expected_rows * state_width):
                    liquid_artifact_failures.append(
                        "{}: previous_solution_states length={} is not 0 or {}".format(
                            label, previous_len, expected_rows * state_width
                        )
                    )
                if full_horizon_da_expected and len(message.initial_guess_states) >= state_width:
                    initial_memory = message.initial_guess_states[23]
                    snapshot_memory = getattr(
                        message, "actuator_a_cmd_memory", None
                    )
                    if not close(initial_memory, snapshot_memory, tolerance=1.0e-8):
                        liquid_artifact_failures.append(
                            "{}: initial a_cmd_memory={} differs from actuator memory {}".format(
                                label, initial_memory, snapshot_memory
                            )
                        )
            if explicit_actuator_expected:
                if str(message.backend) != "continuous_mpcc_acados_explicit_actuator":
                    liquid_artifact_failures.append(
                        "{}: wrong explicit-actuator backend {!r}".format(
                            label, message.backend
                        )
                    )
                if str(message.control_semantics) != "a_cmd_alpha_cmd":
                    liquid_artifact_failures.append(
                        "{}: wrong control semantics {!r}".format(
                            label, message.control_semantics
                        )
                    )
                if not bool(message.actuator_state_valid):
                    liquid_artifact_failures.append(
                        "{}: actuator state is invalid".format(label)
                    )
                if full_horizon_da_expected and not finite(
                    getattr(message, "actuator_a_cmd_memory", None)
                ):
                    liquid_artifact_failures.append(
                        "{}: actuator a_cmd_memory is non-finite or missing".format(
                            label
                        )
                    )
                if len(message.actuator_linear_delay_queue) != 5:
                    liquid_artifact_failures.append(
                        "{}: linear FIFO width={} expected 5".format(
                            label, len(message.actuator_linear_delay_queue)
                        )
                    )
                if len(message.actuator_angular_delay_queue) != 10:
                    liquid_artifact_failures.append(
                        "{}: angular FIFO width={} expected 10".format(
                            label, len(message.actuator_angular_delay_queue)
                        )
                    )
                required_parameters = {
                    "actuator_dt",
                    "actuator_tau_v",
                    "actuator_tau_omega",
                    "actuator_gain_v",
                    "actuator_gain_omega",
                }
                missing_parameters = required_parameters.difference(
                    str(name) for name in message.parameter_names
                )
                if missing_parameters:
                    liquid_artifact_failures.append(
                        "{}: missing actuator parameters {}".format(
                            label, sorted(missing_parameters)
                        )
                    )
                if full_horizon_da_expected:
                    parameter_names = [str(name) for name in message.parameter_names]
                    try:
                        w_du_a_index = parameter_names.index("w_du_a")
                    except ValueError:
                        w_du_a_index = -1
                    expected_w_du_a = extra_expected_config.get("w_du_a")
                    if w_du_a_index < 0 or expected_w_du_a is None:
                        liquid_artifact_failures.append(
                            "{}: full-horizon Delta-a contract lacks w_du_a metadata".format(
                                label
                            )
                        )
                    else:
                        width = int(message.parameter_width)
                        for stage in range(int(message.horizon_steps) + 1):
                            value_index = stage * width + w_du_a_index
                            if value_index >= len(message.stage_parameters) or not close(
                                message.stage_parameters[value_index],
                                expected_w_du_a,
                            ):
                                liquid_artifact_failures.append(
                                    "{}: stage {} w_du_a is not {}".format(
                                        label, stage, expected_w_du_a
                                    )
                                )
                                break
        for message in valid_horizons:
            label = "horizon cycle {}".format(message.cycle_id)
            liquid_artifact_failures.extend(
                validate_liquid_metadata(
                    message,
                    label,
                    expected_variant,
                    expected_slosh,
                    args.expected_slosh_cost_horizon_steps,
                    args.expected_slosh_cost_tail_discount,
                    args.expected_robot_horizon_steps,
                    args.expected_dt_sec,
                )
            )
            if int(message.schema_version) < args.minimum_solver_schema_version:
                liquid_artifact_failures.append(
                    "{}: schema_version={} < {}".format(
                        label,
                        message.schema_version,
                        args.minimum_solver_schema_version,
                    )
                )
            if explicit_actuator_expected:
                expected_rows = int(message.horizon_steps) + 1
                state_fields = [
                    "v_cmd",
                    "omega_cmd",
                    "delayed_v_cmd",
                    "delayed_omega_cmd",
                    "a_actual",
                    "alpha_actual",
                ]
                if full_horizon_da_expected:
                    state_fields.append("a_cmd_memory")
                for field in state_fields:
                    values = list(getattr(message, field, []))
                    if len(values) != expected_rows:
                        liquid_artifact_failures.append(
                            "{}: {} length={} expected {}".format(
                                label, field, len(values), expected_rows
                            )
                        )
                    elif not all(finite(value) for value in values):
                        liquid_artifact_failures.append(
                            "{}: {} contains non-finite values".format(
                                label, field
                            )
                        )
                if full_horizon_da_expected:
                    memories = list(getattr(message, "a_cmd_memory", []))
                    controls = list(getattr(message, "a", []))
                    if len(memories) == expected_rows and len(controls) == expected_rows - 1:
                        mismatch_count = sum(
                            not close(memories[k + 1], controls[k], tolerance=1.0e-5)
                            for k in range(len(controls))
                        )
                        if mismatch_count:
                            liquid_artifact_failures.append(
                                "{}: a_cmd_memory(k+1)=a_cmd(k) mismatch count={}".format(
                                    label, mismatch_count
                                )
                            )
                if str(message.backend) != "continuous_mpcc_acados_explicit_actuator":
                    liquid_artifact_failures.append(
                        "{}: wrong explicit-actuator backend {!r}".format(
                            label, message.backend
                        )
                    )
                if str(message.control_semantics) != "a_cmd_alpha_cmd":
                    liquid_artifact_failures.append(
                        "{}: wrong control semantics {!r}".format(
                            label, message.control_semantics
                        )
                    )

        if explicit_actuator_expected:
            audits_by_cycle = {
                int(message.cycle_id): message
                for _, message in motion_audits
                if bool(message.solve_attempted) and bool(message.solve_success)
            }
            for message in valid_horizons:
                cycle_id = int(message.cycle_id)
                audit = audits_by_cycle.get(cycle_id)
                if audit is None:
                    continue
                if len(message.v_cmd) < 2 or len(message.omega_cmd) < 2:
                    continue
                if not close(audit.solver_cmd_v, message.v_cmd[1], 2.0e-5):
                    liquid_artifact_failures.append(
                        "cycle {}: solver_cmd_v={} != horizon x1.v_cmd={}".format(
                            cycle_id, audit.solver_cmd_v, message.v_cmd[1]
                        )
                    )
                if not close(
                    audit.solver_cmd_omega, message.omega_cmd[1], 2.0e-5
                ):
                    liquid_artifact_failures.append(
                        "cycle {}: solver_cmd_omega={} != horizon x1.omega_cmd={}".format(
                            cycle_id,
                            audit.solver_cmd_omega,
                            message.omega_cmd[1],
                        )
                    )
        failures.extend(liquid_artifact_failures)

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
        "schema": args.report_schema,
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
            "selection_joined_to_solve": len(selections_to_validate),
            "solver_input_motion": len(motion_solver_inputs),
            "warm_start_motion": len(motion_warm_starts),
            "valid_snapshot_motion": len(valid_snapshots),
            "valid_horizon_motion": len(valid_horizons),
        },
        "contracts": {
            "selected_observer": "processed_imu_I0",
            "fallback_policy": "fail_closed",
            "solver_consumes_liquid": expected_slosh,
            "final_liquid_input_when_consumed": (
                "L22_command_history_rollout"
                if expected_slosh and legacy_delay_expected
                else "I0_explicit_actuator_epoch"
                if expected_slosh
                else "not_consumed"
            ),
            "delay_mode_code": args.expected_delay_mode_code,
            "delay_sec": [args.expected_linear_delay_sec, args.expected_angular_delay_sec],
            "legacy_delay_application_required": legacy_delay_expected,
            "execution_model_code": args.expected_execution_model_code,
            "actuator_valid_fraction": actuator_valid_fraction,
            "expected_state_width": args.expected_state_width,
            "minimum_solver_schema_version": args.minimum_solver_schema_version,
            "selection_bad_count": selection_bad,
            "selection_consumption_bad_count": consumption_bad,
            "selection_cycle_failure_count": len(deep_selection_failures),
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
            "deep_liquid_cost_contract": deep_cost_contract,
            "expected_robot_horizon_steps": args.expected_robot_horizon_steps
            if deep_cost_contract
            else None,
            "expected_slosh_cost_horizon_steps": args.expected_slosh_cost_horizon_steps,
            "expected_slosh_cost_tail_discount": args.expected_slosh_cost_tail_discount,
            "expected_slosh_eta_dot_ratio": args.expected_slosh_eta_dot_ratio,
            "expected_dt_sec": args.expected_dt_sec,
            "expected_control_frequency_hz": args.expected_control_frequency_hz,
            "expected_effective_config": expected_config
            if deep_cost_contract
            else {},
            "effective_config_field_failure_count": len(config_field_failures),
            "liquid_config_failure_count": len(deep_config_failures),
            "cycle_audit_coverage_failure_count": len(cycle_audit_failures),
            "liquid_artifact_failure_count": len(liquid_artifact_failures),
            "solve_cycle_count": len(solve_cycle_ids),
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
