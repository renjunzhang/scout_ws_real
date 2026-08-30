#!/usr/bin/env python3
"""Validate one agile B0 delay-mode summary.

This gate is intentionally separate from the trial wrapper.  It owns the
machine-checkable A/B contract: frozen delay-mode semantics, effective speed
configuration, solver/publication maxima, and compensation application rate.
"""

import argparse
import json
import math
from pathlib import Path


EXPECTED_V_REF = 0.10
EXPECTED_V_SAFE_MAX = 0.15
EXPECTED_SPEED_TOLERANCE = 0.0001
OBSERVED_SPEED_LIMIT = EXPECTED_V_SAFE_MAX + EXPECTED_SPEED_TOLERANCE


def finite(value):
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def require_fraction(failures, delay, name, minimum=None, maximum=None):
    value = delay.get(name)
    if not finite(value):
        failures.append(f"{name} is missing/non-finite")
        return
    value = float(value)
    if minimum is not None and value < minimum:
        failures.append(f"{name}={value:.4f} < {minimum:.4f}")
    if maximum is not None and value > maximum:
        failures.append(f"{name}={value:.4f} > {maximum:.4f}")


def require_close(failures, name, value, expected, tolerance):
    if not finite(value):
        failures.append(f"{name} is missing/non-finite")
    elif abs(float(value) - expected) > tolerance:
        failures.append(
            f"{name}={float(value):.6f} != "
            f"{expected:.6f} +/- {tolerance:.6f}"
        )


def require_abs_max(failures, name, node, maximum):
    value = node.get("max") if isinstance(node, dict) else None
    count = node.get("n", 0) if isinstance(node, dict) else 0
    if not count or not finite(value):
        failures.append(f"{name} maximum is missing/non-finite")
    elif float(value) > maximum:
        failures.append(f"{name}={float(value):.6f} > {maximum:.6f}")


def validate_summary(summary, expected_mode, expected_code):
    failures = []
    intent = summary.get("intent", {})
    observed = summary.get("observed", {})
    metrics = summary.get("metrics", {})
    delay = metrics.get("delay_state", {})
    effective = metrics.get("effective_config_last", {})

    if str(intent.get("delay_phase_mode", "")) != expected_mode:
        failures.append(
            f"intent delay mode {intent.get('delay_phase_mode')!r} "
            f"!= {expected_mode!r}"
        )
    if str(intent.get("variant", "")) != "B0":
        failures.append(f"intent variant {intent.get('variant')!r} != 'B0'")
    if str(observed.get("controller_variant_last", "")) != "B0":
        failures.append(
            f"observed variant {observed.get('controller_variant_last')!r} "
            "!= 'B0'"
        )

    effective_code = effective.get("delay_phase_mode_code")
    if not finite(effective_code) or abs(float(effective_code) - expected_code) > 1e-6:
        failures.append(
            f"effective delay mode code {effective_code!r} "
            f"!= {expected_code:.0f}"
        )
    critical_missing = summary.get("topics", {}).get("critical_missing", [])
    if critical_missing:
        failures.append("critical topics missing: " + ", ".join(critical_missing))

    if expected_mode == "fixed_robot_only":
        require_fraction(
            failures, delay, "delay_compensation_applied_frac", minimum=0.99
        )
        require_fraction(
            failures,
            delay,
            "robot_delay_compensation_applied_frac",
            minimum=0.99,
        )
        require_fraction(
            failures,
            delay,
            "liquid_delay_compensation_applied_frac",
            maximum=0.05,
        )
        require_fraction(failures, delay, "history_complete_frac", minimum=0.99)
    elif expected_mode == "shadow":
        require_fraction(
            failures, delay, "delay_compensation_applied_frac", maximum=0.05
        )
        require_fraction(
            failures,
            delay,
            "robot_delay_compensation_applied_frac",
            maximum=0.05,
        )
        require_fraction(
            failures,
            delay,
            "liquid_delay_compensation_applied_frac",
            maximum=0.05,
        )
        # The alignment topic includes the intentional stationary startup
        # window.  Predicted-state validity is the correct shadow-mode signal.
        require_fraction(failures, delay, "predicted_valid_frac", minimum=0.95)
        require_fraction(failures, delay, "history_complete_frac", minimum=0.95)
    else:
        failures.append(
            f"unsupported expected mode {expected_mode!r}; "
            "expected shadow|fixed_robot_only"
        )

    require_close(
        failures, "intent.v_ref", intent.get("v_ref"), EXPECTED_V_REF, 1e-6
    )
    if str(intent.get("speed_safety_enable", "")).lower() not in {
        "1",
        "true",
        "yes",
        "on",
    }:
        failures.append(
            "intent speed_safety_enable="
            f"{intent.get('speed_safety_enable')!r} != true"
        )
    require_close(
        failures,
        "intent.v_safe_max",
        intent.get("v_safe_max"),
        EXPECTED_V_SAFE_MAX,
        1e-6,
    )
    require_close(
        failures,
        "intent.speed_safety_tolerance",
        intent.get("speed_safety_tolerance"),
        EXPECTED_SPEED_TOLERANCE,
        1e-8,
    )
    require_close(
        failures,
        "effective.v_ref",
        effective.get("v_ref"),
        EXPECTED_V_REF,
        1e-4,
    )
    require_close(
        failures,
        "effective.speed_safety_enable",
        effective.get("speed_safety_enable"),
        1.0,
        1e-6,
    )
    require_close(
        failures,
        "effective.v_safe_max",
        effective.get("v_safe_max"),
        EXPECTED_V_SAFE_MAX,
        1e-4,
    )
    require_close(
        failures,
        "effective.v_max",
        effective.get("v_max"),
        EXPECTED_V_SAFE_MAX,
        1e-4,
    )
    require_close(
        failures,
        "effective.effective_v_max",
        effective.get("effective_v_max"),
        EXPECTED_V_SAFE_MAX,
        1e-4,
    )
    require_close(
        failures,
        "effective.speed_safety_tolerance",
        effective.get("speed_safety_tolerance"),
        EXPECTED_SPEED_TOLERANCE,
        1e-6,
    )

    command = metrics.get("command_intervention", {})
    require_abs_max(
        failures,
        "solver_cmd_v",
        command.get("solver_cmd_v_abs", {}),
        OBSERVED_SPEED_LIMIT,
    )
    require_abs_max(
        failures,
        "post_gate_cmd_v",
        command.get("post_gate_cmd_v_abs", {}),
        OBSERVED_SPEED_LIMIT,
    )
    require_abs_max(
        failures,
        "published_cmd_v",
        command.get("published_cmd_v_abs", {}),
        OBSERVED_SPEED_LIMIT,
    )
    require_abs_max(
        failures,
        "bag_/cmd_vel.linear.x",
        metrics.get("cmd_vel", {}).get("linear_x_abs", {}),
        OBSERVED_SPEED_LIMIT,
    )
    require_close(
        failures,
        "speed_safety_violation_frac",
        command.get("speed_safety_violation_frac"),
        0.0,
        1e-12,
    )
    require_close(
        failures,
        "speed_safety_latched_frac",
        command.get("speed_safety_latched_frac"),
        0.0,
        1e-12,
    )

    return {
        "schema": "spmpc_mocap_b0_delay_mode_gate_v2",
        "expected_mode": expected_mode,
        "expected_mode_code": int(expected_code),
        "observed_mode_code": effective_code,
        "delay_state": delay,
        "speed_safety_effective": {
            key: effective.get(key)
            for key in (
                "platform_v_max",
                "speed_safety_enable",
                "v_safe_max",
                "effective_v_max",
                "speed_safety_tolerance",
            )
        },
        "command_intervention": command,
        "cmd_vel": metrics.get("cmd_vel", {}),
        "pass": not failures,
        "failures": failures,
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument(
        "--expected-mode", choices=("shadow", "fixed_robot_only"), required=True
    )
    parser.add_argument("--expected-code", type=int, choices=(2, 4), required=True)
    parser.add_argument("--report", type=Path, required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    report = validate_summary(summary, args.expected_mode, float(args.expected_code))
    report["summary"] = str(args.summary)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
