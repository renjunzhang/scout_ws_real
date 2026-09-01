#!/usr/bin/env python3
"""Pure-Python audit helpers for the solver liquid-cost window contract."""

import collections
import math


CycleCoverageResult = collections.namedtuple(
    "CycleCoverageResult",
    (
        "audit_failures",
        "selection_failures",
        "snapshot_failures",
        "horizon_failures",
        "audit_cycle_sequence",
        "solve_cycle_sequence",
        "solve_cycle_ids",
        "selection_records",
        "valid_snapshot_records",
        "valid_horizon_records",
    ),
)


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def close(value, expected, tolerance=1.0e-5):
    return finite(value) and abs(float(value) - float(expected)) <= tolerance


def parse_expected_config_items(items):
    """Parse repeated ``key=value`` effective-config expectations."""
    expected = {}
    failures = []
    for raw_item in items or ():
        item = str(raw_item)
        if "=" not in item:
            failures.append(
                "expected config item {!r} must use key=value".format(item)
            )
            continue
        key, raw_value = item.split("=", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if not key or not key.isidentifier():
            failures.append(
                "expected config key {!r} is not a valid field name".format(key)
            )
            continue
        if key in expected:
            failures.append("duplicate expected config key {!r}".format(key))
            continue
        if not finite(raw_value):
            failures.append(
                "expected config {}={!r} is not finite".format(key, raw_value)
            )
            continue
        expected[key] = float(raw_value)
    return expected, failures


def _cycle_id(message):
    try:
        return int(getattr(message, "cycle_id"))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _records_for_cycles(records, cycle_ids):
    return [
        record
        for record in records
        if _cycle_id(record[1]) in cycle_ids
    ]


def validate_deep_cycle_coverage(
    motion_audits,
    selections,
    snapshots,
    horizons,
):
    """Join deep evidence by cycle ID and audit its one-record-per-cycle contract.

    Inputs are ``(timestamp, message-like object)`` records.  Timestamps are
    deliberately not used for joining: solver artifacts and observer selection
    are published before the matching command-time control audit.
    """
    audit_failures = []
    audit_messages = []
    audit_cycle_sequence = []
    for index, (_, message) in enumerate(motion_audits):
        cycle_id = _cycle_id(message)
        if cycle_id is None:
            audit_failures.append(
                "motion audit record {} has no readable cycle_id".format(index)
            )
            continue
        audit_messages.append(message)
        audit_cycle_sequence.append(cycle_id)

    duplicate_ids = sorted(
        cycle_id
        for cycle_id, count in collections.Counter(audit_cycle_sequence).items()
        if count > 1
    )
    if duplicate_ids:
        audit_failures.append(
            "duplicate motion audit cycle IDs: {}".format(duplicate_ids)
        )
    for previous, current in zip(
        audit_cycle_sequence, audit_cycle_sequence[1:]
    ):
        if current <= previous:
            audit_failures.append(
                "motion audit cycle IDs are not strictly increasing: {} -> {}".format(
                    previous, current
                )
            )
        elif current != previous + 1:
            audit_failures.append(
                "missing motion audit cycle IDs between {} and {}".format(
                    previous, current
                )
            )

    solve_cycle_sequence = [
        cycle_id
        for cycle_id, message in zip(audit_cycle_sequence, audit_messages)
        if bool(getattr(message, "solve_attempted", False))
    ]
    solve_cycle_ids = set(solve_cycle_sequence)
    if not solve_cycle_sequence:
        audit_failures.append("motion window has no solve-attempted audit cycles")

    selection_records = _records_for_cycles(selections, solve_cycle_ids)
    snapshot_records = _records_for_cycles(snapshots, solve_cycle_ids)
    horizon_records = _records_for_cycles(horizons, solve_cycle_ids)
    valid_snapshot_records = [
        record
        for record in snapshot_records
        if bool(getattr(record[1], "valid", False))
    ]
    valid_horizon_records = [
        record
        for record in horizon_records
        if bool(getattr(record[1], "valid", False))
    ]

    selection_failures = []
    snapshot_failures = []
    horizon_failures = []
    selection_counts = collections.Counter(
        _cycle_id(message) for _, message in selection_records
    )
    snapshot_counts = collections.Counter(
        _cycle_id(message) for _, message in snapshot_records
    )
    valid_snapshot_counts = collections.Counter(
        _cycle_id(message) for _, message in valid_snapshot_records
    )
    horizon_counts = collections.Counter(
        _cycle_id(message) for _, message in horizon_records
    )
    valid_horizon_counts = collections.Counter(
        _cycle_id(message) for _, message in valid_horizon_records
    )
    for cycle_id in sorted(solve_cycle_ids):
        if selection_counts[cycle_id] != 1:
            selection_failures.append(
                "cycle {} observer selection count={}, expected 1".format(
                    cycle_id, selection_counts[cycle_id]
                )
            )
        if snapshot_counts[cycle_id] != 1:
            snapshot_failures.append(
                "cycle {} snapshot count={}, expected 1".format(
                    cycle_id, snapshot_counts[cycle_id]
                )
            )
        if valid_snapshot_counts[cycle_id] != 1:
            snapshot_failures.append(
                "cycle {} valid snapshot count={}, expected 1".format(
                    cycle_id, valid_snapshot_counts[cycle_id]
                )
            )
        if horizon_counts[cycle_id] != 1:
            horizon_failures.append(
                "cycle {} horizon count={}, expected 1".format(
                    cycle_id, horizon_counts[cycle_id]
                )
            )
        if valid_horizon_counts[cycle_id] != 1:
            horizon_failures.append(
                "cycle {} valid horizon count={}, expected 1".format(
                    cycle_id, valid_horizon_counts[cycle_id]
                )
            )

    return CycleCoverageResult(
        audit_failures=tuple(audit_failures),
        selection_failures=tuple(selection_failures),
        snapshot_failures=tuple(snapshot_failures),
        horizon_failures=tuple(horizon_failures),
        audit_cycle_sequence=tuple(audit_cycle_sequence),
        solve_cycle_sequence=tuple(solve_cycle_sequence),
        solve_cycle_ids=solve_cycle_ids,
        selection_records=tuple(selection_records),
        valid_snapshot_records=tuple(valid_snapshot_records),
        valid_horizon_records=tuple(valid_horizon_records),
    )


def stage_scale(stage, horizon_steps, trusted_last_stage, tail_discount):
    """Mirror ``sloshCostStageScale`` for one state-cost stage."""
    if stage < 0 or horizon_steps < 0 or stage > horizon_steps:
        return 0.0
    if trusted_last_stage < 0:
        return 1.0
    trusted_last = min(horizon_steps, max(0, trusted_last_stage))
    if stage <= trusted_last:
        return 1.0
    if not finite(tail_discount):
        return 0.0
    return max(0.0, min(1.0, float(tail_discount)))


def expected_stage_weights(
    stage,
    horizon_steps,
    trusted_last_stage,
    tail_discount,
    base_weight,
    eta_dot_ratio,
):
    scale = stage_scale(
        stage, horizon_steps, trusted_last_stage, tail_discount
    )
    return base_weight * scale, base_weight * eta_dot_ratio * scale


def expected_horizon_sec(cost_horizon_steps, dt):
    return -1.0 if cost_horizon_steps < 0 else cost_horizon_steps * float(dt)


def validate_metadata(
    record,
    label,
    expected_variant,
    expected_slosh_enabled,
    expected_cost_horizon_steps,
    expected_cost_tail_discount,
    expected_robot_horizon_steps,
    expected_dt_sec,
):
    failures = []
    schema_version = getattr(record, "schema_version", None)
    if schema_version is None or int(schema_version) < 2:
        failures.append("{}: schema_version={!r}, expected >=2".format(label, schema_version))
    if str(getattr(record, "variant", "")) != expected_variant:
        failures.append(
            "{}: variant={!r}, expected {!r}".format(
                label, getattr(record, "variant", None), expected_variant
            )
        )
    if bool(getattr(record, "slosh_enabled", False)) != expected_slosh_enabled:
        failures.append(
            "{}: slosh_enabled={!r}, expected {}".format(
                label,
                getattr(record, "slosh_enabled", None),
                expected_slosh_enabled,
            )
        )
    robot_steps = getattr(record, "horizon_steps", None)
    if robot_steps is None or int(robot_steps) != expected_robot_horizon_steps:
        failures.append(
            "{}: horizon_steps={!r}, expected {}".format(
                label, robot_steps, expected_robot_horizon_steps
            )
        )
    dt = getattr(record, "dt", None)
    if not finite(dt) or float(dt) <= 0.0:
        failures.append("{}: invalid dt={!r}".format(label, dt))
    elif not close(dt, expected_dt_sec):
        failures.append(
            "{}: dt={!r}, expected {}".format(label, dt, expected_dt_sec)
        )
    observed_steps = getattr(record, "slosh_cost_horizon_steps", None)
    if observed_steps is None or int(observed_steps) != expected_cost_horizon_steps:
        failures.append(
            "{}: slosh_cost_horizon_steps={!r}, expected {}".format(
                label, observed_steps, expected_cost_horizon_steps
            )
        )
    observed_tail = getattr(record, "slosh_cost_tail_discount", None)
    if not close(observed_tail, expected_cost_tail_discount):
        failures.append(
            "{}: slosh_cost_tail_discount={!r}, expected {}".format(
                label, observed_tail, expected_cost_tail_discount
            )
        )
    observed_sec = getattr(record, "slosh_cost_horizon_sec", None)
    if finite(dt):
        expected_sec = expected_horizon_sec(expected_cost_horizon_steps, dt)
        if not close(observed_sec, expected_sec):
            failures.append(
                "{}: slosh_cost_horizon_sec={!r}, expected {}".format(
                    label, observed_sec, expected_sec
                )
            )
    return failures


def validate_config_fields(config, label, expected):
    """Validate an explicit set of resolved effective-config fields."""
    failures = []
    for field, value in expected.items():
        if not close(config.get(field), value):
            failures.append(
                "{}: {}={!r}, expected {}".format(
                    label, field, config.get(field), value
                )
            )
    return failures


def validate_effective_config(
    config,
    label,
    expected_slosh_enabled,
    expected_weight,
    expected_eta_dot_ratio,
    expected_cost_horizon_steps,
    expected_cost_tail_discount,
    expected_robot_horizon_steps,
    expected_dt_sec,
    expected_control_frequency_hz,
):
    expected = {
        "slosh_enable": 1.0 if expected_slosh_enabled else 0.0,
        "w_slosh": expected_weight,
        "slosh_eta_dot_ratio": expected_eta_dot_ratio,
        "slosh_cost_horizon_steps": float(expected_cost_horizon_steps),
        "slosh_cost_tail_discount": expected_cost_tail_discount,
        "horizon_steps": float(expected_robot_horizon_steps),
        "dt": expected_dt_sec,
        "control_frequency": expected_control_frequency_hz,
    }
    failures = validate_config_fields(config, label, expected)
    dt = config.get("dt")
    if not finite(dt) or float(dt) <= 0.0:
        failures.append("{}: invalid dt={!r}".format(label, dt))
    else:
        expected_sec = expected_horizon_sec(expected_cost_horizon_steps, dt)
        if not close(config.get("slosh_cost_horizon_sec"), expected_sec):
            failures.append(
                "{}: slosh_cost_horizon_sec={!r}, expected {}".format(
                    label, config.get("slosh_cost_horizon_sec"), expected_sec
                )
            )
    return failures


def validate_snapshot_stage_weights(
    snapshot,
    label,
    expected_weight,
    expected_eta_dot_ratio,
    expected_cost_horizon_steps,
    expected_cost_tail_discount,
):
    failures = []
    names = list(getattr(snapshot, "parameter_names", []))
    width = int(getattr(snapshot, "parameter_width", 0))
    horizon_steps = int(getattr(snapshot, "horizon_steps", -1))
    values = list(getattr(snapshot, "stage_parameters", []))
    if width <= 0:
        return ["{}: parameter_width={} is not positive".format(label, width)]
    if len(names) != width:
        failures.append(
            "{}: parameter_names count {} != width {}".format(
                label, len(names), width
            )
        )
    duplicates = sorted(
        name for name, count in collections.Counter(names).items() if count > 1
    )
    if duplicates:
        failures.append("{}: duplicate parameter names {}".format(label, duplicates))
    expected_size = (horizon_steps + 1) * width
    if horizon_steps < 0 or len(values) != expected_size:
        failures.append(
            "{}: stage parameter size {} != {}".format(
                label, len(values), expected_size
            )
        )
    required = ("w_slosh_eta", "w_slosh_eta_dot")
    missing = [name for name in required if name not in names]
    if missing:
        failures.append("{}: missing parameters {}".format(label, missing))
    if failures:
        return failures

    indices = {name: names.index(name) for name in required}
    for stage in range(horizon_steps + 1):
        expected_eta, expected_eta_dot = expected_stage_weights(
            stage,
            horizon_steps,
            expected_cost_horizon_steps,
            expected_cost_tail_discount,
            expected_weight,
            expected_eta_dot_ratio,
        )
        observed_eta = values[stage * width + indices["w_slosh_eta"]]
        observed_eta_dot = values[
            stage * width + indices["w_slosh_eta_dot"]
        ]
        if not close(observed_eta, expected_eta):
            failures.append(
                "{}: stage {} w_slosh_eta={!r}, expected {}".format(
                    label, stage, observed_eta, expected_eta
                )
            )
        if not close(observed_eta_dot, expected_eta_dot):
            failures.append(
                "{}: stage {} w_slosh_eta_dot={!r}, expected {}".format(
                    label, stage, observed_eta_dot, expected_eta_dot
                )
            )
    return failures
