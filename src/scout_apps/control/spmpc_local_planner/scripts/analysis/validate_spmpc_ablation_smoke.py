#!/usr/bin/env python3
"""Additional schema-5 checks; existing postflights own runtime and coverage gates."""
import argparse
import json
import math
from pathlib import Path

from validate_explicit_actuator_runtime_smoke import parse_multiarray

SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
CONFIG_TOPIC = "/spmpc/debug/effective_config"
LIQUID_FIELDS = ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")


def close(value, expected, tolerance=1e-6):
    try:
        return math.isfinite(float(value)) and abs(float(value) - expected) <= tolerance
    except (ValueError, TypeError):
        return False


def expected_fields(liquid, zero, jerk, jerk_max):
    return {"slosh_enabled": liquid, "zero_liquid_initial_state": zero,
            "jerk_limit_enable": jerk, "jerk_max": jerk_max}


def validate_pair(snapshot, horizon, *, liquid, zero, jerk, jerk_max):
    failures = []
    fields = expected_fields(liquid, zero, jerk, jerk_max)
    for label, message in (("snapshot", snapshot), ("horizon", horizon)):
        if getattr(message, "schema_version", 0) < 5:
            failures.append(label + ": schema < 5")
        for field, expected in fields.items():
            if not close(getattr(message, field, None), expected):
                failures.append(label + ": wrong " + field)
        if not close(getattr(message, "dt", None), 1.0 / 30.0, 1e-9):
            failures.append(label + ": unexpected dt")
        bound = jerk_max / 30.0 if jerk else 1e15
        if not close(getattr(message, "delta_a_max", None), bound):
            failures.append(label + ": wrong delta_a_max")
        epoch = getattr(message, "solver_input_epoch", None)
        if epoch is None or not math.isfinite(epoch.to_sec()) or epoch.to_sec() <= 0:
            failures.append(label + ": invalid solver epoch")
    if snapshot.cycle_id != horizon.cycle_id:
        failures.append("cycle mismatch")
    if snapshot.solver_input_epoch != horizon.solver_input_epoch:
        failures.append("solver epoch mismatch")
    if liquid:
        for field in LIQUID_FIELDS:
            expected = 0.0 if zero else getattr(snapshot, "observed_" + field, math.nan)
            actual = getattr(snapshot, field, None)
            predicted = getattr(horizon, field, [])
            if not close(actual, expected):
                failures.append("snapshot liquid x0 mismatch: " + field)
            if not predicted or not close(predicted[0], expected):
                failures.append("horizon liquid x0 mismatch: " + field)
    controls = list(getattr(horizon, "a", []))
    memories = list(getattr(horizon, "a_cmd_memory", []))
    if (getattr(snapshot, "horizon_steps", 0) != 60
            or getattr(horizon, "horizon_steps", 0) != 60
            or len(controls) != 60 or len(memories) != 61):
        failures.append("incomplete 60-stage control/memory horizon")
        return failures, None
    previous = getattr(snapshot, "actuator_a_cmd_memory", math.nan)
    if not close(memories[0], previous):
        failures.append("stage 0 does not use snapshot published-command history")
    max_delta = 0.0
    for stage, acceleration in enumerate(controls):
        if not math.isfinite(acceleration) or not math.isfinite(previous):
            failures.append("non-finite control/history at stage " + str(stage))
            return failures, None
        delta = abs(acceleration - previous)
        max_delta = max(max_delta, delta)
        if jerk and delta > jerk_max / 30.0 + 1e-6:
            failures.append("hard acceleration-rate bound violated at stage " + str(stage))
        if not close(memories[stage + 1], acceleration, 1e-5):
            failures.append("memory propagation mismatch at stage " + str(stage))
        previous = acceleration
    return failures, max_delta


def validate_bag(args):
    import rosbag

    failures = []
    snapshots = {}
    horizons = {}
    config_count = 0
    audits = []
    expected_config = expected_fields(args.slosh_enable, args.zero_liquid_initial_state,
                                      args.jerk_limit_enable, args.jerk_max)
    expected_config["slosh_enable"] = expected_config.pop("slosh_enabled")
    expect_handoff = getattr(args, "expect_terminal_handoff", False)
    if expect_handoff:
        expected_config["terminal_mpc_stop_handoff_enable"] = 1.0
    # Pair by cycle ID and exact solver epoch, never bag receive time or fitted lag.
    with rosbag.Bag(args.bag) as bag:
        for topic, message, _ in bag.read_messages(
                topics=[SNAPSHOT_TOPIC, HORIZON_TOPIC, CONFIG_TOPIC, "/spmpc/debug/control_cycle_audit"]):
            if topic.endswith("control_cycle_audit"):
                audits.append(message)
                continue
            if topic == CONFIG_TOPIC:
                config_count += 1
                config = parse_multiarray(message)
                for field, value in expected_config.items():
                    if not close(config.get(field), value):
                        failures.append("effective_config mismatch: " + field)
            elif bool(getattr(message, "valid", False)):
                target = snapshots if topic == SNAPSHOT_TOPIC else horizons
                if message.cycle_id in target:
                    failures.append("duplicate cycle ID on " + topic)
                target[message.cycle_id] = message
    if expect_handoff:
        owned = False
        for audit in audits:
            if str(audit.solver_status) in ("TERMINAL_STOP", "GOAL_REACHED"):
                owned = True
            if owned and (audit.solve_attempted or audit.solve_success):
                failures.append("terminal stop incorrectly reentered/reported MPC at cycle " + str(audit.cycle_id))
        if not audits:
            failures.append("missing control audit for terminal ownership check")
    if config_count == 0:
        failures.append("missing effective_config")
    if len(horizons) < 10:
        failures.append("fewer than 10 valid horizons")
    maximum_delta = 0.0
    checked = 0
    for cycle, horizon in horizons.items():
        if cycle not in snapshots:
            failures.append("missing snapshot for cycle " + str(cycle))
            continue
        pair_failures, delta = validate_pair(
            snapshots[cycle], horizon, liquid=args.slosh_enable,
            zero=args.zero_liquid_initial_state, jerk=args.jerk_limit_enable,
            jerk_max=args.jerk_max)
        failures.extend("cycle {}: {}".format(cycle, item) for item in pair_failures)
        checked += 1
        if delta is not None:
            maximum_delta = max(maximum_delta, delta)
    return {"schema": "spmpc_ablation_smoke_postflight_v1",
            "status": "FAIL" if failures else "PASS", "bag": args.bag,
            "expected_config": expected_config, "checked_pairs": checked,
            "max_horizon_delta_a_mps2": maximum_delta,
            "failure_count": len(failures), "failures": failures[:100],
            "scope": "OCP ablation contract only; no physical slosh efficacy claim"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--report", required=True)
    parser.add_argument("--expect-terminal-handoff", action="store_true")
    for name in ("slosh-enable", "zero-liquid-initial-state", "jerk-limit-enable"):
        parser.add_argument("--" + name, choices=("true", "false"), required=True)
    parser.add_argument("--jerk-max", type=float, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.jerk_max) or args.jerk_max <= 0:
        parser.error("jerk-max must be finite and positive")
    for name in ("slosh_enable", "zero_liquid_initial_state", "jerk_limit_enable"):
        setattr(args, name, getattr(args, name) == "true")
    if args.zero_liquid_initial_state and not args.slosh_enable:
        parser.error("NoState requires liquid prediction")
    try:
        report = validate_bag(args)
    except Exception as exc:
        report = {"status": "FAIL", "failures": [str(exc)], "bag": args.bag}
    Path(args.report).write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("ablation postflight: " + report["status"])
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
