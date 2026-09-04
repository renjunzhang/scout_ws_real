#!/usr/bin/env python3
"""Validate runtime timing for one explicit-actuator B_slosh smoke bag.

The existing I0 fail-closed postflight owns the model, observer, solver, and
common-epoch contracts.  This companion check focuses on the runtime failure
mechanism seen in the first B_slosh run: dropped planner odom samples and
control callbacks that overrun the 30 Hz period.
"""

import argparse
import json
import math
import sys
from pathlib import Path


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
INTERVENTION_TOPIC = "/spmpc/debug/command_intervention"
ODOM_OBSERVER_TOPIC = "/spmpc/debug/slosh_observer_odom"

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
    parser.add_argument("--report", required=True)
    parser.add_argument(
        "--protocol",
        default="SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_RUNTIME_SMOKE_DEV_V1",
    )
    parser.add_argument("--minimum-samples", type=int, default=10)
    parser.add_argument("--linear-motion-threshold", type=float, default=0.03)
    parser.add_argument("--angular-motion-threshold", type=float, default=0.03)
    parser.add_argument("--max-planner-odom-gap-ms", type=float, default=50.0)
    parser.add_argument("--max-control-callback-p95-ms", type=float, default=30.0)
    parser.add_argument(
        "--callback-period-ms", type=float, default=1000.0 / 30.0
    )
    parser.add_argument(
        "--max-consecutive-callback-overrun", type=int, default=1
    )
    parser.add_argument("--max-delta-a0-p95", type=float)
    parser.add_argument("--max-turning-a0-5hz-amplitude", type=float)
    parser.add_argument("--max-strong-a0-sign-flips", type=int)
    parser.add_argument("--turning-omega-threshold", type=float, default=0.08)
    parser.add_argument("--strong-a0-threshold", type=float, default=0.5)
    parser.add_argument("--control-frequency-hz", type=float, default=30.0)
    parser.add_argument("--five-hz-band-min-hz", type=float, default=4.5)
    parser.add_argument("--five-hz-band-max-hz", type=float, default=5.5)
    return parser.parse_args()


def stamp_sec(value):
    try:
        result = float(value.to_sec())
    except (AttributeError, TypeError, ValueError):
        return 0.0
    return result if math.isfinite(result) else 0.0


def message_stamp(message, bag_stamp):
    value = stamp_sec(getattr(getattr(message, "header", None), "stamp", None))
    return value if value > 0.0 else float(bag_stamp)


def parse_multiarray(message):
    dimensions = getattr(getattr(message, "layout", None), "dim", [])
    label = dimensions[0].label if dimensions else ""
    names = [item.strip() for item in label.split(",") if item.strip()]
    values = list(getattr(message, "data", []))
    return {
        names[index] if index < len(names) else "field_{}".format(index): float(value)
        for index, value in enumerate(values)
    }


def percentile_linear(values, percentile):
    """Return NumPy-compatible linear percentile without a NumPy dependency."""
    if not values:
        return None
    if not 0.0 <= percentile <= 100.0:
        raise ValueError("percentile must be in [0, 100]")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def max_consecutive_true(flags):
    longest = 0
    current = 0
    for flag in flags:
        if flag:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def dominant_uniform_dft(values, sample_rate_hz, min_hz, max_hz):
    """Return the strongest one-sided DFT component in a frozen band.

    Turning samples are concatenated and treated as a 30 Hz sequence.  This
    exactly preserves the diagnostic convention used for the 0.07821 baseline.
    """
    samples = [float(value) for value in values]
    count = len(samples)
    if count < 2 or sample_rate_hz <= 0.0 or min_hz < 0.0 or max_hz < min_hz:
        return None, None
    first_bin = max(1, int(math.ceil(min_hz * count / sample_rate_hz)))
    last_bin = min(count // 2, int(math.floor(max_hz * count / sample_rate_hz)))
    if first_bin > last_bin:
        return None, None
    mean = sum(samples) / count
    centered = [value - mean for value in samples]
    best_frequency = None
    best_amplitude = None
    for bin_index in range(first_bin, last_bin + 1):
        real = 0.0
        imag = 0.0
        for sample_index, value in enumerate(centered):
            phase = 2.0 * math.pi * bin_index * sample_index / count
            real += value * math.cos(phase)
            imag -= value * math.sin(phase)
        amplitude = 2.0 * math.hypot(real, imag) / count
        if best_amplitude is None or amplitude > best_amplitude:
            best_frequency = bin_index * sample_rate_hz / count
            best_amplitude = amplitude
    return best_frequency, best_amplitude


def _in_window(records, start, end):
    if start is None or end is None:
        return []
    return [record for record in records if start <= record[0] <= end]


def _finite_positive_stamp(message, field):
    value = stamp_sec(getattr(message, field, None))
    return value if value > 0.0 else None


def compute_runtime_report(
    audits,
    odom_rows,
    interventions,
    *,
    protocol,
    bag,
    minimum_samples=10,
    linear_motion_threshold=0.03,
    angular_motion_threshold=0.03,
    max_planner_odom_gap_ms=50.0,
    max_control_callback_p95_ms=30.0,
    callback_period_ms=1000.0 / 30.0,
    max_consecutive_callback_overrun=1,
    max_delta_a0_p95=None,
    max_turning_a0_5hz_amplitude=None,
    max_strong_a0_sign_flips=None,
    turning_omega_threshold=0.08,
    strong_a0_threshold=0.5,
    control_frequency_hz=30.0,
    five_hz_band_min_hz=4.5,
    five_hz_band_max_hz=5.5,
    initial_failures=None,
):
    failures = list(initial_failures or [])
    active_stamps = [
        when
        for when, message in audits
        if bool(getattr(message, "command_was_published", False))
        and (
            abs(float(getattr(message, "published_cmd_v", 0.0)))
            > linear_motion_threshold
            or abs(float(getattr(message, "published_cmd_omega", 0.0)))
            > angular_motion_threshold
        )
    ]
    motion_start = min(active_stamps) if active_stamps else None
    motion_end = max(active_stamps) if active_stamps else None
    if motion_start is None:
        failures.append("no effective motion window")

    motion_audits = _in_window(audits, motion_start, motion_end)
    motion_odom = _in_window(odom_rows, motion_start, motion_end)
    motion_interventions = _in_window(interventions, motion_start, motion_end)

    if len(motion_audits) < minimum_samples:
        failures.append(
            "control audit samples {} < {}".format(
                len(motion_audits), minimum_samples
            )
        )
    if len(motion_odom) < minimum_samples:
        failures.append(
            "planner odom samples {} < {}".format(len(motion_odom), minimum_samples)
        )
    if len(motion_interventions) < minimum_samples:
        failures.append(
            "command intervention samples {} < {}".format(
                len(motion_interventions), minimum_samples
            )
        )

    callback_durations_ms = []
    callback_invalid_count = 0
    callback_overrun_flags = []
    common_epoch_failure_count = 0
    solver_failure_count = 0
    for _, message in motion_audits:
        start = _finite_positive_stamp(message, "cycle_start_stamp")
        published = _finite_positive_stamp(message, "command_publish_stamp")
        if start is None or published is None or published < start:
            callback_invalid_count += 1
            callback_overrun_flags.append(False)
        else:
            duration_ms = 1000.0 * (published - start)
            callback_durations_ms.append(duration_ms)
            callback_overrun_flags.append(duration_ms > callback_period_ms)
        if bool(getattr(message, "state_alignment_required", False)) and not bool(
            getattr(message, "state_time_aligned", False)
        ):
            common_epoch_failure_count += 1
        if bool(getattr(message, "solve_attempted", False)) and not bool(
            getattr(message, "solve_success", False)
        ):
            solver_failure_count += 1

    if callback_invalid_count:
        failures.append(
            "invalid control callback timestamps count={}".format(
                callback_invalid_count
            )
        )
    if len(callback_durations_ms) < minimum_samples:
        failures.append(
            "valid control callback durations {} < {}".format(
                len(callback_durations_ms), minimum_samples
            )
        )

    callback_p95_ms = percentile_linear(callback_durations_ms, 95.0)
    callback_max_ms = max(callback_durations_ms) if callback_durations_ms else None
    callback_overrun_count = sum(callback_overrun_flags)
    longest_callback_overrun = max_consecutive_true(callback_overrun_flags)

    # The observer diagnostic carries every odom sample accepted by the planner.
    # Its header/measurement stamp is the odom source time used in robot history;
    # this exposes dropped samples directly instead of measuring rosbag arrival.
    odom_measurement_stamps = []
    for _, message in motion_odom:
        stamp = _finite_positive_stamp(message, "measurement_stamp")
        if stamp is None:
            stamp = _finite_positive_stamp(message, "source_stamp")
        if stamp is not None:
            odom_measurement_stamps.append(stamp)
    odom_nonpositive_gap_count = 0
    odom_gaps_ms = []
    for previous, current in zip(
        odom_measurement_stamps, odom_measurement_stamps[1:]
    ):
        gap_ms = 1000.0 * (current - previous)
        if gap_ms <= 0.0:
            odom_nonpositive_gap_count += 1
        else:
            odom_gaps_ms.append(gap_ms)
    if odom_nonpositive_gap_count:
        failures.append(
            "planner odom non-positive gaps count={}".format(
                odom_nonpositive_gap_count
            )
        )
    if len(odom_gaps_ms) < max(1, minimum_samples - 1):
        failures.append(
            "planner odom readable gaps {} < {}".format(
                len(odom_gaps_ms), max(1, minimum_samples - 1)
            )
        )
    odom_max_gap_ms = max(odom_gaps_ms) if odom_gaps_ms else None
    odom_gap_over_limit_count = sum(
        gap > max_planner_odom_gap_ms for gap in odom_gaps_ms
    )

    zero_reason_counts = {
        field: sum(float(row.get(field, 0.0)) > 0.5 for _, row in motion_interventions)
        for field in BAD_ZERO_FIELDS
    }
    fault_zero_count = sum(
        any(float(row.get(field, 0.0)) > 0.5 for field in BAD_ZERO_FIELDS)
        for _, row in motion_interventions
    )

    valid_solver_rows = []
    for when, message in motion_audits:
        a0 = finite_float(getattr(message, "solver_u0_a", None))
        omega = finite_float(getattr(message, "published_cmd_omega", None))
        if (
            bool(getattr(message, "solve_success", False))
            and not bool(getattr(message, "terminal_phase", False))
            and str(getattr(message, "solver_status", "")).endswith("ACADOS_OK")
            and a0 is not None
            and omega is not None
        ):
            valid_solver_rows.append((when, a0, omega))
    a0_values = [row[1] for row in valid_solver_rows]
    delta_a0 = [
        abs(current - previous)
        for previous, current in zip(a0_values, a0_values[1:])
    ]
    delta_a0_p95 = percentile_linear(delta_a0, 95.0)
    turning_values = [
        a0 for _, a0, omega in valid_solver_rows
        if abs(omega) >= turning_omega_threshold
    ]
    five_hz_frequency, five_hz_amplitude = dominant_uniform_dft(
        turning_values,
        control_frequency_hz,
        five_hz_band_min_hz,
        five_hz_band_max_hz,
    )
    strong_sign_flips = sum(
        abs(previous[2]) >= turning_omega_threshold
        and abs(current[2]) >= turning_omega_threshold
        and abs(previous[1]) >= strong_a0_threshold
        and abs(current[1]) >= strong_a0_threshold
        and previous[1] * current[1] < 0.0
        for previous, current in zip(valid_solver_rows, valid_solver_rows[1:])
    )

    continuity_gate_enabled = any(
        threshold is not None
        for threshold in (
            max_delta_a0_p95,
            max_turning_a0_5hz_amplitude,
            max_strong_a0_sign_flips,
        )
    )
    if continuity_gate_enabled and len(valid_solver_rows) < minimum_samples:
        failures.append(
            "valid ACADOS a0 samples {} < {}".format(
                len(valid_solver_rows), minimum_samples
            )
        )
    if max_delta_a0_p95 is not None:
        if delta_a0_p95 is None:
            failures.append("no readable Delta-a0 samples")
        elif delta_a0_p95 > max_delta_a0_p95 + 1.0e-12:
            failures.append(
                "|Delta a0| P95 {:.6f} > {:.6f} m/s^2".format(
                    delta_a0_p95, max_delta_a0_p95
                )
            )
    if max_turning_a0_5hz_amplitude is not None:
        if len(turning_values) < minimum_samples or five_hz_amplitude is None:
            failures.append(
                "turning a0 samples {} insufficient for 5 Hz metric".format(
                    len(turning_values)
                )
            )
        elif five_hz_amplitude > max_turning_a0_5hz_amplitude + 1.0e-12:
            failures.append(
                "turning a0 ~5 Hz amplitude {:.6f} > {:.6f}".format(
                    five_hz_amplitude, max_turning_a0_5hz_amplitude
                )
            )
    if (
        max_strong_a0_sign_flips is not None
        and strong_sign_flips > max_strong_a0_sign_flips
    ):
        failures.append(
            "strong a0 sign flips {} > {}".format(
                strong_sign_flips, max_strong_a0_sign_flips
            )
        )

    if common_epoch_failure_count:
        failures.append(
            "common-epoch failure count={}".format(common_epoch_failure_count)
        )
    if solver_failure_count:
        failures.append("solver failure count={}".format(solver_failure_count))
    if fault_zero_count:
        failures.append("fault zero count={}".format(fault_zero_count))
    if odom_gap_over_limit_count:
        failures.append(
            "planner odom gap > {:.3f} ms count={}".format(
                max_planner_odom_gap_ms, odom_gap_over_limit_count
            )
        )
    if callback_p95_ms is not None and not (
        callback_p95_ms < max_control_callback_p95_ms
    ):
        failures.append(
            "control callback P95 {:.3f} ms is not < {:.3f} ms".format(
                callback_p95_ms, max_control_callback_p95_ms
            )
        )
    if longest_callback_overrun > max_consecutive_callback_overrun:
        failures.append(
            "max consecutive callback overruns {} > {}".format(
                longest_callback_overrun, max_consecutive_callback_overrun
            )
        )

    return {
        "schema": "spmpc_explicit_actuator_runtime_smoke_postflight_v2",
        "protocol": protocol,
        "status": "PASS" if not failures else "FAIL",
        "bag": str(bag),
        "motion_window": {"start_sec": motion_start, "end_sec": motion_end},
        "common_epoch_failure_count": common_epoch_failure_count,
        "solver_failure_count": solver_failure_count,
        "fault_zero_count": fault_zero_count,
        "zero_reason_counts": zero_reason_counts,
        "planner_odom_sample_count": len(motion_odom),
        "planner_odom_readable_gap_count": len(odom_gaps_ms),
        "planner_odom_nonpositive_gap_count": odom_nonpositive_gap_count,
        "planner_odom_max_gap_ms": odom_max_gap_ms,
        "planner_odom_gap_over_50ms_count": odom_gap_over_limit_count,
        "control_callback_sample_count": len(callback_durations_ms),
        "control_callback_invalid_count": callback_invalid_count,
        "control_callback_p95_ms": callback_p95_ms,
        "control_callback_max_ms": callback_max_ms,
        "callback_over_33_3ms_count": callback_overrun_count,
        "max_consecutive_callback_overrun": longest_callback_overrun,
        "control_continuity": {
            "valid_solver_sample_count": len(valid_solver_rows),
            "delta_a0_sample_count": len(delta_a0),
            "abs_delta_a0_p95_mps2": delta_a0_p95,
            "turning_sample_count": len(turning_values),
            "turning_a0_band_peak_frequency_hz": five_hz_frequency,
            "turning_a0_band_peak_amplitude_mps2": five_hz_amplitude,
            "strong_a0_sign_flip_count": strong_sign_flips,
        },
        "thresholds": {
            "max_planner_odom_gap_ms": max_planner_odom_gap_ms,
            "max_control_callback_p95_ms_strict": max_control_callback_p95_ms,
            "callback_period_ms": callback_period_ms,
            "max_consecutive_callback_overrun": max_consecutive_callback_overrun,
            "max_delta_a0_p95_mps2": max_delta_a0_p95,
            "max_turning_a0_5hz_amplitude_mps2": max_turning_a0_5hz_amplitude,
            "max_strong_a0_sign_flips": max_strong_a0_sign_flips,
            "turning_omega_threshold_radps": turning_omega_threshold,
            "strong_a0_threshold_mps2": strong_a0_threshold,
            "control_frequency_hz": control_frequency_hz,
            "five_hz_band_hz": [five_hz_band_min_hz, five_hz_band_max_hz],
        },
        "failures": failures,
    }


def validate(args):
    bag_path = Path(args.bag).expanduser().resolve()
    report_path = Path(args.report).expanduser().resolve()
    failures = []
    audits = []
    odom_rows = []
    interventions = []

    if args.minimum_samples < 2:
        failures.append("minimum samples must be >= 2")
    if args.max_planner_odom_gap_ms <= 0.0:
        failures.append("max planner odom gap must be positive")
    if args.max_control_callback_p95_ms <= 0.0:
        failures.append("max control callback P95 must be positive")
    if args.callback_period_ms <= 0.0:
        failures.append("callback period must be positive")
    if args.max_consecutive_callback_overrun < 0:
        failures.append("max consecutive callback overrun must be non-negative")
    for label, value in (
        ("max Delta-a0 P95", args.max_delta_a0_p95),
        ("max turning a0 5 Hz amplitude", args.max_turning_a0_5hz_amplitude),
    ):
        if value is not None and (not math.isfinite(value) or value < 0.0):
            failures.append("{} must be finite and non-negative".format(label))
    if args.max_strong_a0_sign_flips is not None and args.max_strong_a0_sign_flips < 0:
        failures.append("max strong a0 sign flips must be non-negative")
    if not math.isfinite(args.turning_omega_threshold) or args.turning_omega_threshold < 0.0:
        failures.append("turning omega threshold must be finite and non-negative")
    if not math.isfinite(args.strong_a0_threshold) or args.strong_a0_threshold < 0.0:
        failures.append("strong a0 threshold must be finite and non-negative")
    if not math.isfinite(args.control_frequency_hz) or args.control_frequency_hz <= 0.0:
        failures.append("control frequency must be finite and positive")
    if (
        not math.isfinite(args.five_hz_band_min_hz)
        or not math.isfinite(args.five_hz_band_max_hz)
        or args.five_hz_band_min_hz < 0.0
        or args.five_hz_band_max_hz < args.five_hz_band_min_hz
    ):
        failures.append("5 Hz band must be finite, non-negative, and ordered")

    if not bag_path.is_file():
        failures.append("bag does not exist: {}".format(bag_path))
    elif Path(str(bag_path) + ".active").exists():
        failures.append("bag still has a .active sibling")
    else:
        try:
            import rosbag  # pylint: disable=import-outside-toplevel

            with rosbag.Bag(str(bag_path), "r") as bag_handle:
                available = bag_handle.get_type_and_topic_info().topics
                for topic in (
                    AUDIT_TOPIC,
                    ODOM_OBSERVER_TOPIC,
                    INTERVENTION_TOPIC,
                ):
                    if topic not in available:
                        failures.append("missing required topic {}".format(topic))
                wanted = [
                    topic
                    for topic in (
                        AUDIT_TOPIC,
                        ODOM_OBSERVER_TOPIC,
                        INTERVENTION_TOPIC,
                    )
                    if topic in available
                ]
                for topic, message, bag_stamp in bag_handle.read_messages(
                    topics=wanted
                ):
                    when = message_stamp(message, bag_stamp.to_sec())
                    if topic == AUDIT_TOPIC:
                        audits.append((when, message))
                    elif topic == ODOM_OBSERVER_TOPIC:
                        odom_rows.append((when, message))
                    elif topic == INTERVENTION_TOPIC:
                        interventions.append((when, parse_multiarray(message)))
        except Exception as exc:  # noqa: BLE001
            failures.append("could not read bag: {}".format(exc))

    report = compute_runtime_report(
        audits,
        odom_rows,
        interventions,
        protocol=args.protocol,
        bag=bag_path,
        minimum_samples=args.minimum_samples,
        linear_motion_threshold=args.linear_motion_threshold,
        angular_motion_threshold=args.angular_motion_threshold,
        max_planner_odom_gap_ms=args.max_planner_odom_gap_ms,
        max_control_callback_p95_ms=args.max_control_callback_p95_ms,
        callback_period_ms=args.callback_period_ms,
        max_consecutive_callback_overrun=args.max_consecutive_callback_overrun,
        max_delta_a0_p95=args.max_delta_a0_p95,
        max_turning_a0_5hz_amplitude=args.max_turning_a0_5hz_amplitude,
        max_strong_a0_sign_flips=args.max_strong_a0_sign_flips,
        turning_omega_threshold=args.turning_omega_threshold,
        strong_a0_threshold=args.strong_a0_threshold,
        control_frequency_hz=args.control_frequency_hz,
        five_hz_band_min_hz=args.five_hz_band_min_hz,
        five_hz_band_max_hz=args.five_hz_band_max_hz,
        initial_failures=failures,
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


def main():
    args = parse_args()
    report = validate(args)
    print("[explicit-actuator-runtime-smoke] status={}".format(report["status"]))
    print(
        "[explicit-actuator-runtime-smoke] report={}".format(
            Path(args.report).expanduser().resolve()
        )
    )
    for failure in report["failures"]:
        print(
            "[explicit-actuator-runtime-smoke] FAIL: {}".format(failure),
            file=sys.stderr,
        )
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
