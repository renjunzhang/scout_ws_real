#!/usr/bin/env python3
"""Publish one stationary linear or angular velocity step with a stamped mirror."""

import argparse
import json
import math
import sys
import time
from pathlib import Path


REQUIRED_ARM_TOKEN = "MOCAP_VELOCITY_STEP_ARMED"
HARDWARE_ACCEL_LIMIT_ARM_TOKEN = "MOCAP_HARDWARE_ACCEL_LIMIT_ARMED"
TRIAL_CONTRACTS = ("low_speed_identification", "hardware_accel_limit")
HARDWARE_ACCEL_TESTS = {
    "H01": {"command_value": 0.80, "step_sec": 2.0},
    "H02": {"command_value": 1.50, "step_sec": 2.0},
    "H03": {"command_value": 3.00, "step_sec": 1.0},
}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument(
        "--stamped-topic", default="/mocap_velocity_step/cmd_vel_stamped"
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--axis", choices=("linear", "angular"), required=True)
    parser.add_argument(
        "--trial-contract", choices=TRIAL_CONTRACTS, default="low_speed_identification"
    )
    parser.add_argument(
        "--hardware-test-id",
        choices=tuple(sorted(HARDWARE_ACCEL_TESTS)),
        help="Frozen hardware acceleration row; required by hardware_accel_limit",
    )
    parser.add_argument("--hardware-accel-limit-arm-token")
    parser.add_argument("--command-value", type=float, required=True)
    parser.add_argument("--pre-sec", type=float, default=3.0)
    parser.add_argument("--step-sec", type=float, default=4.0)
    parser.add_argument("--post-sec", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--run-label", default="UNSPECIFIED")
    parser.add_argument(
        "--data-split",
        choices=("development", "validation", "final_test"),
        default="development",
    )
    parser.add_argument("--matrix-row", default="single")
    parser.add_argument("--attempt", default="01")
    parser.add_argument("--connection-timeout-sec", type=float, default=5.0)
    parser.add_argument("--minimum-cmd-connections", type=int, default=2)
    parser.add_argument("--stationary-check-sec", type=float, default=1.0)
    parser.add_argument("--max-static-v", type=float, default=0.02)
    parser.add_argument("--max-static-odom-w", type=float, default=0.03)
    parser.add_argument("--max-static-imu-w", type=float, default=0.03)
    parser.add_argument(
        "--arm-token",
        help="Required explicit acknowledgement for publishing a motion command",
    )
    parser.add_argument("--metadata", type=Path)
    return parser


def validate_args(args):
    if args.arm_token != REQUIRED_ARM_TOKEN:
        raise ValueError(
            "refusing motion without --arm-token {}".format(REQUIRED_ARM_TOKEN)
        )
    if not math.isfinite(args.command_value) or args.command_value == 0.0:
        raise ValueError("command-value must be finite and nonzero")
    if args.trial_contract == "hardware_accel_limit":
        if args.hardware_accel_limit_arm_token != HARDWARE_ACCEL_LIMIT_ARM_TOKEN:
            raise ValueError(
                "hardware_accel_limit requires --hardware-accel-limit-arm-token {}".format(
                    HARDWARE_ACCEL_LIMIT_ARM_TOKEN
                )
            )
        if args.axis != "linear":
            raise ValueError("hardware_accel_limit is frozen to the linear axis")
        if args.hardware_test_id not in HARDWARE_ACCEL_TESTS:
            raise ValueError("hardware_accel_limit requires hardware-test-id H01, H02, or H03")
        frozen_test = HARDWARE_ACCEL_TESTS[args.hardware_test_id]
        if abs(args.command_value - frozen_test["command_value"]) > 1e-9:
            raise ValueError(
                "hardware_accel_limit {} is frozen to +{:.2f} m/s".format(
                    args.hardware_test_id, frozen_test["command_value"]
                )
            )
        for name, value, frozen in (
            ("pre-sec", args.pre_sec, 3.0),
            ("step-sec", args.step_sec, frozen_test["step_sec"]),
            ("post-sec", args.post_sec, 4.0),
            ("rate-hz", args.rate_hz, 50.0),
        ):
            if abs(value - frozen) > 1e-9:
                raise ValueError(
                    "hardware_accel_limit requires {}={}".format(name, frozen)
                )
        limit = 3.00
    else:
        if args.hardware_test_id is not None:
            raise ValueError(
                "hardware-test-id is only valid with trial-contract hardware_accel_limit"
            )
        limit = 0.15 if args.axis == "linear" else 0.30
    unit = "m/s" if args.axis == "linear" else "rad/s"
    if abs(args.command_value) > limit:
        raise ValueError("require |command-value| <= {:.2f} {}".format(limit, unit))
    if not 2.0 <= args.pre_sec <= 10.0:
        raise ValueError("require 2 <= pre-sec <= 10")
    if args.trial_contract == "low_speed_identification":
        if not 2.5 <= args.step_sec <= 6.0:
            raise ValueError("require 2.5 <= step-sec <= 6")
    elif not 1.0 <= args.step_sec <= 2.0:
        raise ValueError("hardware_accel_limit requires 1.0 <= step-sec <= 2.0")
    if not 2.0 <= args.post_sec <= 10.0:
        raise ValueError("require 2 <= post-sec <= 10")
    if not 20.0 <= args.rate_hz <= 100.0:
        raise ValueError("require 20 <= rate-hz <= 100")


def main():
    args = build_parser().parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        print("[mocap_velocity_step][ERR] {}".format(exc), file=sys.stderr)
        return 2

    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    rospy.init_node("mocap_velocity_step_command", anonymous=True)
    if bool(rospy.get_param("/use_sim_time", False)):
        rospy.logerr("Real velocity-step publisher refuses /use_sim_time=true")
        return 2
    command_publisher = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
    stamped_publisher = rospy.Publisher(args.stamped_topic, TwistStamped, queue_size=10)
    odom_samples = []
    imu_samples = []

    def on_odom(message):
        odom_samples.append(
            (
                time.monotonic(),
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z),
            )
        )
        if len(odom_samples) > 500:
            del odom_samples[:-250]

    def on_imu(message):
        imu_samples.append((time.monotonic(), float(message.angular_velocity.z)))
        if len(imu_samples) > 500:
            del imu_samples[:-250]

    rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=50)
    rospy.Subscriber(args.imu_topic, Imu, on_imu, queue_size=100)
    rate = rospy.Rate(args.rate_hz)

    connection_deadline = time.monotonic() + args.connection_timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < connection_deadline:
        if (
            command_publisher.get_num_connections() >= args.minimum_cmd_connections
            and stamped_publisher.get_num_connections() >= 1
        ):
            break
        rospy.sleep(0.05)
    if command_publisher.get_num_connections() < args.minimum_cmd_connections:
        rospy.logerr(
            "Expected at least %d subscribers on %s (base + recorder), found %d",
            args.minimum_cmd_connections,
            args.cmd_topic,
            command_publisher.get_num_connections(),
        )
        return 2
    if stamped_publisher.get_num_connections() < 1:
        rospy.logerr("No recorder connected to %s", args.stamped_topic)
        return 2

    def publish_once(value, phase):
        now = rospy.Time.now()
        command = Twist()
        if args.axis == "linear":
            command.linear.x = value
        else:
            command.angular.z = value
        stamped = TwistStamped()
        stamped.header.stamp = now
        stamped.header.frame_id = phase
        stamped.twist = command
        command_publisher.publish(command)
        stamped_publisher.publish(stamped)
        return now.to_sec()

    def run_phase(duration, value, phase):
        start_monotonic = time.monotonic()
        first_stamp = None
        last_stamp = None
        while not rospy.is_shutdown() and time.monotonic() - start_monotonic < duration:
            stamp = publish_once(value, phase)
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
            rate.sleep()
        if first_stamp is None:
            raise RuntimeError("phase {} published no command".format(phase))
        return first_stamp, last_stamp

    def require_stationary():
        cutoff = time.monotonic() - args.stationary_check_sec
        recent_odom = [row for row in odom_samples if row[0] >= cutoff]
        recent_imu = [row for row in imu_samples if row[0] >= cutoff]
        minimum_odom = max(5, int(args.stationary_check_sec * 5.0))
        minimum_imu = max(10, int(args.stationary_check_sec * 10.0))
        if len(recent_odom) < minimum_odom or len(recent_imu) < minimum_imu:
            raise RuntimeError(
                "not enough odom/IMU samples for stationary preflight: {}/{}".format(
                    len(recent_odom), len(recent_imu)
                )
            )
        def p95_abs(values):
            ordered = sorted(abs(value) for value in values)
            index = min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)
            return ordered[index]

        p95_v = p95_abs(row[1] for row in recent_odom)
        p95_odom_w = p95_abs(row[2] for row in recent_odom)
        p95_imu_w = p95_abs(row[1] for row in recent_imu)
        if p95_v > args.max_static_v or p95_odom_w > args.max_static_odom_w or p95_imu_w > args.max_static_imu_w:
            raise RuntimeError(
                "robot is not stationary: p95 |v|={:.4f}, |odom w|={:.4f}, |imu w|={:.4f}".format(
                    p95_v, p95_odom_w, p95_imu_w
                )
            )
        return {
            "p95_abs_odom_v_m_s": p95_v,
            "p95_abs_odom_w_rad_s": p95_odom_w,
            "p95_abs_imu_w_rad_s": p95_imu_w,
            "odom_sample_count": len(recent_odom),
            "imu_sample_count": len(recent_imu),
        }

    phase_stamps = {}
    stationary = None
    exit_code = 0
    try:
        rospy.loginfo("Pre-zero for %.2f s", args.pre_sec)
        phase_stamps["pre_zero"] = run_phase(args.pre_sec, 0.0, "pre_zero")
        stationary = require_stationary()
        unit = "m/s" if args.axis == "linear" else "rad/s"
        rospy.loginfo(
            "%s velocity step %.3f %s for %.2f s",
            args.axis,
            args.command_value,
            unit,
            args.step_sec,
        )
        phase_stamps["step"] = run_phase(
            args.step_sec, args.command_value, "{}_step".format(args.axis)
        )
        rospy.loginfo("Post-zero for %.2f s", args.post_sec)
        phase_stamps["post_zero"] = run_phase(args.post_sec, 0.0, "post_zero")
    except Exception as exc:
        rospy.logerr("Velocity step aborted: %s", exc)
        exit_code = 2
    finally:
        for _ in range(max(10, int(args.rate_hz * 0.5))):
            if rospy.is_shutdown():
                break
            publish_once(0.0, "final_zero")
            rate.sleep()

    if args.metadata:
        payload = {
            "protocol_id": (
                "MOCAP_HARDWARE_ACCEL_LIMIT_V2"
                if args.trial_contract == "hardware_accel_limit"
                else "MOCAP_VELOCITY_STEP_V2"
            ),
            "run_label": args.run_label,
            "data_split": args.data_split,
            "matrix_row": args.matrix_row,
            "attempt": args.attempt,
            "trial_contract": args.trial_contract,
            "hardware_test_id": args.hardware_test_id,
            "axis": args.axis,
            "cmd_topic": args.cmd_topic,
            "stamped_topic": args.stamped_topic,
            "odom_topic": args.odom_topic,
            "imu_topic": args.imu_topic,
            "command_value": args.command_value,
            "command_unit": "m/s" if args.axis == "linear" else "rad/s",
            "pre_sec": args.pre_sec,
            "step_sec": args.step_sec,
            "post_sec": args.post_sec,
            "rate_hz": args.rate_hz,
            "stationary_preflight": stationary,
            "phase_stamps_sec": {
                key: {"first": value[0], "last": value[1]}
                for key, value in phase_stamps.items()
            },
            "exit_code": exit_code,
        }
        args.metadata.parent.mkdir(parents=True, exist_ok=True)
        args.metadata.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
