#!/usr/bin/env python3
"""Publish one continuous chassis velocity profile with a stamped mirror."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import velocity_continuity_core as continuity_core


REQUIRED_ARM_TOKEN = "MOCAP_VELOCITY_CONTINUITY_ARMED"


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument(
        "--stamped-topic", default="/mocap_velocity_continuity/cmd_vel_stamped"
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--axis", choices=("linear", "angular"), required=True)
    parser.add_argument("--profile", choices=continuity_core.PROFILES, required=True)
    parser.add_argument("--target-value", type=float, required=True)
    parser.add_argument("--pre-sec", type=float, default=3.0)
    parser.add_argument("--ramp-up-sec", type=float, default=3.0)
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--ramp-down-sec", type=float, default=3.0)
    parser.add_argument("--post-sec", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--run-label", default="UNSPECIFIED")
    parser.add_argument(
        "--data-split",
        choices=("development", "validation", "final_test"),
        default="development",
    )
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
    if not math.isfinite(args.target_value) or args.target_value == 0.0:
        raise ValueError("target-value must be finite and nonzero")
    # The chassis high-speed mode is rated to 3.0 m/s, but this direct-command
    # mocap protocol deliberately has a much lower site-safety ceiling.
    speed_limit = 0.80 if args.axis == "linear" else 0.30
    speed_unit = "m/s" if args.axis == "linear" else "rad/s"
    if abs(args.target_value) > speed_limit:
        raise ValueError(
            "require |target-value| <= {:.2f} {}".format(speed_limit, speed_unit)
        )
    for name, value, lower, upper in (
        ("pre-sec", args.pre_sec, 2.0, 10.0),
        ("ramp-up-sec", args.ramp_up_sec, 2.0, 10.0),
        ("hold-sec", args.hold_sec, 1.0, 6.0),
        ("ramp-down-sec", args.ramp_down_sec, 2.0, 10.0),
        ("post-sec", args.post_sec, 2.0, 10.0),
        ("rate-hz", args.rate_hz, 20.0, 100.0),
    ):
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError("require {} <= {} <= {}".format(lower, name, upper))
    acceleration_factor = (
        1.0
        if args.profile in ("constant_accel", "trapezoidal_velocity")
        else 2.0
    )
    peak_acceleration = acceleration_factor * abs(args.target_value) / min(
        args.ramp_up_sec, args.ramp_down_sec
    )
    acceleration_limit = 0.60 if args.axis == "linear" else 0.40
    acceleration_unit = "m/s^2" if args.axis == "linear" else "rad/s^2"
    if peak_acceleration > acceleration_limit:
        raise ValueError(
            "profile peak acceleration {:.3f} exceeds safe test limit {:.3f} {}".format(
                peak_acceleration, acceleration_limit, acceleration_unit
            )
        )
    if args.axis == "linear":
        if args.profile in ("constant_accel", "trapezoidal_velocity"):
            expected_motion = abs(args.target_value) * (
                0.5 * args.ramp_up_sec
                + args.hold_sec
                + 0.5 * args.ramp_down_sec
            )
        else:
            expected_motion = abs(args.target_value) * (
                args.ramp_up_sec / 3.0
                + args.hold_sec
                + 2.0 * args.ramp_down_sec / 3.0
            )
        if expected_motion > 5.0:
            raise ValueError(
                "profile expected travel {:.3f} m exceeds the 5.0 m site-safety limit".format(
                    expected_motion
                )
            )


def main():
    args = build_parser().parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        print("[mocap_velocity_continuity][ERR] {}".format(exc), file=sys.stderr)
        return 2

    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    rospy.init_node("mocap_velocity_continuity_command", anonymous=True)
    if bool(rospy.get_param("/use_sim_time", False)):
        rospy.logerr("Real velocity-continuity publisher refuses /use_sim_time=true")
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
            command.linear.x = float(value)
        else:
            command.angular.z = float(value)
        stamped = TwistStamped()
        stamped.header.stamp = now
        stamped.header.frame_id = phase
        stamped.twist = command
        command_publisher.publish(command)
        stamped_publisher.publish(stamped)
        return now.to_sec()

    def run_constant_phase(duration, value, phase):
        phase_start = time.monotonic()
        first_stamp = None
        last_stamp = None
        while not rospy.is_shutdown() and time.monotonic() - phase_start < duration:
            stamp = publish_once(value, phase)
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
            rate.sleep()
        if first_stamp is None:
            raise RuntimeError("phase {} published no command".format(phase))
        return first_stamp, last_stamp

    def run_ramp_phase(duration, phase, ramp_down=False):
        phase_start = time.monotonic()
        first_stamp = None
        last_stamp = None
        while not rospy.is_shutdown():
            elapsed = time.monotonic() - phase_start
            progress = min(1.0, elapsed / duration)
            value = continuity_core.phase_value(
                args.profile,
                progress,
                args.target_value,
                ramp_down=ramp_down,
            )
            stamp = publish_once(value, phase)
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
            if progress >= 1.0:
                break
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
        if (
            p95_v > args.max_static_v
            or p95_odom_w > args.max_static_odom_w
            or p95_imu_w > args.max_static_imu_w
        ):
            raise RuntimeError(
                "robot is not stationary: p95 |v|={:.4f}, |odom w|={:.4f}, "
                "|imu w|={:.4f}".format(p95_v, p95_odom_w, p95_imu_w)
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
        phase_stamps["pre_zero"] = run_constant_phase(args.pre_sec, 0.0, "pre_zero")
        stationary = require_stationary()
        rospy.loginfo(
            "%s %s ramp to %+.3f over %.2f s",
            args.axis,
            args.profile,
            args.target_value,
            args.ramp_up_sec,
        )
        phase_stamps["ramp_up"] = run_ramp_phase(
            args.ramp_up_sec, "{}_ramp_up".format(args.profile)
        )
        phase_stamps["hold"] = run_constant_phase(
            args.hold_sec, args.target_value, "hold"
        )
        phase_stamps["ramp_down"] = run_ramp_phase(
            args.ramp_down_sec,
            "{}_ramp_down".format(args.profile),
            ramp_down=True,
        )
        phase_stamps["post_zero"] = run_constant_phase(
            args.post_sec, 0.0, "post_zero"
        )
    except Exception as exc:
        rospy.logerr("Velocity continuity profile aborted: %s", exc)
        exit_code = 2
    finally:
        for _ in range(max(10, int(args.rate_hz * 0.5))):
            if rospy.is_shutdown():
                break
            publish_once(0.0, "final_zero")
            rate.sleep()

    if args.metadata:
        acceleration_factor = (
            1.0
            if args.profile in ("constant_accel", "trapezoidal_velocity")
            else 2.0
        )
        payload = {
            "protocol_id": "MOCAP_VELOCITY_CONTINUITY_V1",
            "run_label": args.run_label,
            "data_split": args.data_split,
            "attempt": args.attempt,
            "axis": args.axis,
            "profile": args.profile,
            "cmd_topic": args.cmd_topic,
            "stamped_topic": args.stamped_topic,
            "odom_topic": args.odom_topic,
            "imu_topic": args.imu_topic,
            "target_value": args.target_value,
            "velocity_unit": "m/s" if args.axis == "linear" else "rad/s",
            "acceleration_unit": "m/s^2" if args.axis == "linear" else "rad/s^2",
            "expected_peak_acceleration": (
                acceleration_factor
                * abs(args.target_value)
                / min(args.ramp_up_sec, args.ramp_down_sec)
            ),
            "pre_sec": args.pre_sec,
            "ramp_up_sec": args.ramp_up_sec,
            "hold_sec": args.hold_sec,
            "ramp_down_sec": args.ramp_down_sec,
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
