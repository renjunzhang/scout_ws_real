#!/usr/bin/env python3
"""Fail-closed postflight for the short-horizon matched hardware release."""

import argparse
import json
import math
import pathlib
import sys

import rosbag


AUDIT_TOPIC = "/spmpc/debug/control_cycle_audit"
SNAPSHOT_TOPIC = "/spmpc/debug/pre_solve_snapshot"
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
CONFIG_TOPIC = "/spmpc/debug/effective_config"
IMAGE_TYPES = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}


def stamp_sec(stamp):
    return stamp.to_sec() if stamp is not None else 0.0


def float_close(value, expected, tolerance=1e-5):
    return math.isfinite(value) and abs(value - expected) <= tolerance


def effective_config(msg):
    if not msg.layout.dim:
        return {}
    labels = [item.strip() for item in msg.layout.dim[0].label.split(",")]
    return dict(zip(labels, msg.data))


def validate(args):
    bag_path = pathlib.Path(args.bag).resolve()
    expected_weight = 0.0 if args.variant.endswith("matched0") else 5.0
    failures = []
    audits = []
    snapshots = []
    horizons = []
    configs = []

    with rosbag.Bag(str(bag_path), "r") as bag:
        info = bag.get_type_and_topic_info()[1]
        image_topics = sorted(
            topic for topic, topic_info in info.items()
            if topic_info.msg_type in IMAGE_TYPES
        )
        if image_topics:
            failures.append("image topics recorded: " + ",".join(image_topics))
        for topic, msg, _ in bag.read_messages(
                topics=[AUDIT_TOPIC, SNAPSHOT_TOPIC, HORIZON_TOPIC, CONFIG_TOPIC]):
            if topic == AUDIT_TOPIC:
                audits.append(msg)
            elif topic == SNAPSHOT_TOPIC:
                snapshots.append(msg)
            elif topic == HORIZON_TOPIC:
                horizons.append(msg)
            else:
                configs.append(effective_config(msg))

    if len(audits) < args.min_audit_cycles:
        failures.append(
            f"audit cycles {len(audits)} < {args.min_audit_cycles}")
    cycle_ids = [msg.cycle_id for msg in audits]
    if cycle_ids != sorted(cycle_ids) or len(cycle_ids) != len(set(cycle_ids)):
        failures.append("control-cycle audit IDs are not strictly increasing/unique")

    solve_audits = [msg for msg in audits if msg.solve_attempted]
    if len(solve_audits) < args.min_solve_cycles:
        failures.append(
            f"solve cycles {len(solve_audits)} < {args.min_solve_cycles}")
    for msg in solve_audits:
        prefix = f"cycle {msg.cycle_id}"
        if msg.variant != args.variant:
            failures.append(f"{prefix}: variant={msg.variant}")
        if msg.observer_source != msg.SOURCE_PROCESSED_IMU:
            failures.append(f"{prefix}: observer_source={msg.observer_source}")
        if not msg.imu_excitation_valid:
            failures.append(f"{prefix}: processed-IMU excitation audit invalid")
        if not msg.solve_success:
            failures.append(
                f"{prefix}: solver failed ({msg.solver_status})")
        if not msg.command_accepted or msg.safety_gate_intervened:
            failures.append(f"{prefix}: command rejected by a safety gate")
        if not args.allow_no_command_publish and (
                not msg.publish_cmd_vel or not msg.command_was_published):
            failures.append(f"{prefix}: command was not published")
        if not msg.state_alignment_required or not msg.state_time_aligned:
            failures.append(f"{prefix}: common-epoch state contract not satisfied")
        if abs(msg.raw_state_skew_sec) > args.max_raw_skew_sec + 1e-9:
            failures.append(
                f"{prefix}: raw skew {msg.raw_state_skew_sec:.6f}s")
        robot_stamp = stamp_sec(msg.robot_state_stamp)
        liquid_stamp = stamp_sec(msg.liquid_state_stamp)
        epoch = stamp_sec(msg.solver_input_epoch)
        if max(abs(robot_stamp - epoch), abs(liquid_stamp - epoch)) > 1e-6:
            failures.append(f"{prefix}: solver states do not share input epoch")
        if not (stamp_sec(msg.solve_start_stamp) <=
                stamp_sec(msg.solve_end_stamp) <=
                stamp_sec(msg.horizon_available_stamp) + 1e-9):
            failures.append(f"{prefix}: non-monotonic solve timestamps")
        if msg.command_contract_violation:
            failures.append(f"{prefix}: command contract violation")
        if msg.linear_limited or msg.angular_rate_limited or msg.angular_accel_limited:
            failures.append(f"{prefix}: post-solver limiter intervened")
        terminal_delta = max(
            abs(msg.terminal_cmd_v - msg.solver_cmd_v),
            abs(msg.terminal_cmd_omega - msg.solver_cmd_omega))
        if msg.terminal_controller_intervened != (terminal_delta > 1e-6):
            failures.append(f"{prefix}: terminal intervention flag mismatch")
        if msg.terminal_controller_intervened and not msg.terminal_phase:
            failures.append(f"{prefix}: terminal controller changed a non-terminal command")
        if (msg.previous_shifted_plan_available and
                msg.previous_plan_cycle_id + 1 != msg.cycle_id):
            failures.append(
                f"{prefix}: shifted plan is not from the adjacent cycle")

    valid_snapshots = [msg for msg in snapshots if msg.valid]
    if len(valid_snapshots) < args.min_solve_cycles:
        failures.append(
            f"valid snapshots {len(valid_snapshots)} < {args.min_solve_cycles}")
    snapshot_ids = {msg.cycle_id for msg in valid_snapshots}
    horizon_ids = {msg.cycle_id for msg in horizons if msg.valid}
    missing_horizons = sorted(snapshot_ids - horizon_ids)
    if missing_horizons:
        failures.append(
            f"snapshot cycles missing valid horizon: {missing_horizons[:10]}")

    for msg in valid_snapshots:
        prefix = f"snapshot cycle {msg.cycle_id}"
        if msg.schema_version < 2:
            failures.append(f"{prefix}: schema_version={msg.schema_version}")
        if msg.variant != args.variant:
            failures.append(f"{prefix}: variant={msg.variant}")
        if msg.slosh_cost_horizon_steps != 3 or not float_close(
                msg.slosh_cost_tail_discount, 0.0):
            failures.append(f"{prefix}: liquid cost window metadata mismatch")
            continue
        names = list(msg.parameter_names)
        try:
            weight_index = names.index("w_slosh_eta")
        except ValueError:
            failures.append(f"{prefix}: w_slosh_eta parameter missing")
            continue
        width = msg.parameter_width
        expected_size = (msg.horizon_steps + 1) * width
        if width <= 0 or len(msg.stage_parameters) != expected_size:
            failures.append(f"{prefix}: malformed stage parameter matrix")
            continue
        for stage in range(msg.horizon_steps + 1):
            value = msg.stage_parameters[stage * width + weight_index]
            expected = expected_weight if stage <= 3 else 0.0
            if not float_close(value, expected):
                failures.append(
                    f"{prefix}: stage {stage} w_slosh_eta={value}, expected={expected}")
                break

    for msg in horizons:
        if not msg.valid:
            continue
        if msg.schema_version < 2 or msg.slosh_cost_horizon_steps != 3 or not float_close(
                msg.slosh_cost_tail_discount, 0.0):
            failures.append(f"horizon cycle {msg.cycle_id}: trusted-window mismatch")

    if not configs:
        failures.append("effective_config topic missing")
    else:
        cfg = configs[-1]
        expected_config = {
            "slosh_enable": 1.0,
            "w_slosh": expected_weight,
            "w_control": 0.3,
            "w_smooth": 1.0,
            "w_alpha": 1.0,
            "w_du_a": 1.0,
            "w_du_vs": 1.0,
            "w_contour": 1.0,
            "w_lag": 0.2,
            "w_progress": 0.2,
            "w_v": 1.0,
            "w_vs": 0.3,
            "slosh_cost_horizon_steps": 3.0,
            "slosh_cost_tail_discount": 0.0,
            "state_timing_require_common_epoch": 1.0,
        }
        for key, expected in expected_config.items():
            if key not in cfg or not float_close(cfg[key], expected):
                failures.append(
                    f"effective_config {key}={cfg.get(key)!r}, expected={expected}")

    report = {
        "schema_version": 1,
        "bag": str(bag_path),
        "variant": args.variant,
        "expected_w_slosh": expected_weight,
        "audit_cycles": len(audits),
        "solve_cycles": len(solve_audits),
        "valid_snapshots": len(valid_snapshots),
        "valid_horizons": len(horizon_ids),
        "pass": not failures,
        "failures": failures,
    }
    report_path = pathlib.Path(args.report) if args.report else bag_path.with_name(
        bag_path.stem + "_short_horizon_postflight.json")
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["pass"] else 2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--variant",
        required=True,
        choices=("B_slosh_matched0", "B_slosh_matched5"))
    parser.add_argument("--report")
    parser.add_argument("--max-raw-skew-sec", type=float, default=0.080)
    parser.add_argument("--min-audit-cycles", type=int, default=30)
    parser.add_argument("--min-solve-cycles", type=int, default=10)
    parser.add_argument("--allow-no-command-publish", action="store_true")
    return validate(parser.parse_args())


if __name__ == "__main__":
    sys.exit(main())
