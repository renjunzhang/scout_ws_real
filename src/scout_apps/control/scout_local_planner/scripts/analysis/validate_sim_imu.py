#!/usr/bin/env python3
"""Validate the simulation IMU topic and IMU TF for the current Scout sim stack."""

import argparse
import math
import statistics
import sys
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SampleBuffer:
    def __init__(self):
        self.samples = []

    def add(self, value):
        self.samples.append(value)

    def count(self):
        return len(self.samples)

    def min(self):
        return min(self.samples) if self.samples else None

    def max(self):
        return max(self.samples) if self.samples else None

    def mean(self):
        return statistics.mean(self.samples) if self.samples else None


class ImuValidator:
    def __init__(self, args):
        self.args = args
        self.imu_msgs = []
        self.odom_msgs = []
        self.tf_listener = tf.TransformListener()
        self.cmd_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(args.imu_topic, Imu, self.imu_cb, queue_size=200)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=200)

    def imu_cb(self, msg):
        self.imu_msgs.append(msg)

    def odom_cb(self, msg):
        self.odom_msgs.append(msg)

    def sleep_and_spin(self, duration_s):
        end_t = time.time() + duration_s
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and time.time() < end_t:
            rate.sleep()

    def publish_cmd(self, linear_x, angular_z, duration_s):
        rate = rospy.Rate(20)
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_t = time.time() + duration_s
        while not rospy.is_shutdown() and time.time() < end_t:
            self.cmd_pub.publish(msg)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def get_tf(self):
        try:
            self.tf_listener.waitForTransform(
                self.args.base_frame,
                self.args.imu_frame,
                rospy.Time(0),
                rospy.Duration(self.args.tf_timeout),
            )
            trans, rot = self.tf_listener.lookupTransform(
                self.args.base_frame, self.args.imu_frame, rospy.Time(0)
            )
            return trans, rot, None
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            return None, None, str(exc)

    @staticmethod
    def estimate_rate(msgs):
        if len(msgs) < 2:
            return None
        stamps = [m.header.stamp.to_sec() for m in msgs if m.header.stamp.to_sec() > 0.0]
        if len(stamps) < 2:
            return None
        dt = stamps[-1] - stamps[0]
        if dt <= 0.0:
            return None
        return (len(stamps) - 1) / dt

    @staticmethod
    def summarize_imu(msgs):
        ay = SampleBuffer()
        wz = SampleBuffer()
        ax = SampleBuffer()
        frames = set()
        for msg in msgs:
            frames.add(msg.header.frame_id)
            ax.add(msg.linear_acceleration.x)
            ay.add(msg.linear_acceleration.y)
            wz.add(msg.angular_velocity.z)
        return {
            "count": len(msgs),
            "frames": sorted(frames),
            "ax_min": ax.min(),
            "ax_max": ax.max(),
            "ay_min": ay.min(),
            "ay_max": ay.max(),
            "wz_min": wz.min(),
            "wz_max": wz.max(),
        }

    def report_phase(self, name, imu_msgs, odom_msgs):
        imu_rate = self.estimate_rate(imu_msgs)
        odom_rate = self.estimate_rate(odom_msgs)
        imu_summary = self.summarize_imu(imu_msgs)

        print(f"\n[{name}]")
        print(f"- imu_msgs: {imu_summary['count']}")
        print(f"- imu_rate_hz: {imu_rate:.3f}" if imu_rate else "- imu_rate_hz: n/a")
        print(f"- odom_rate_hz: {odom_rate:.3f}" if odom_rate else "- odom_rate_hz: n/a")
        print(f"- imu_frame_ids: {imu_summary['frames']}")
        if imu_summary["count"] > 0:
            print(
                "- angular_velocity.z range: "
                f"[{imu_summary['wz_min']:.6f}, {imu_summary['wz_max']:.6f}]"
            )
            print(
                "- linear_acceleration.x range: "
                f"[{imu_summary['ax_min']:.6f}, {imu_summary['ax_max']:.6f}]"
            )
            print(
                "- linear_acceleration.y range: "
                f"[{imu_summary['ay_min']:.6f}, {imu_summary['ay_max']:.6f}]"
            )

    def validate(self):
        print("Waiting for initial IMU and TF...")
        self.sleep_and_spin(self.args.warmup_s)

        trans, rot, tf_err = self.get_tf()
        if tf_err:
            print(f"TF check failed: {tf_err}")
        else:
            print(
                "TF base->imu: "
                f"xyz=({trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}) "
                f"quat=({rot[0]:.3f}, {rot[1]:.3f}, {rot[2]:.3f}, {rot[3]:.3f})"
            )

        imu_count0 = len(self.imu_msgs)
        odom_count0 = len(self.odom_msgs)
        self.sleep_and_spin(self.args.passive_s)
        passive_imu = self.imu_msgs[imu_count0:]
        passive_odom = self.odom_msgs[odom_count0:]
        self.report_phase("passive", passive_imu, passive_odom)

        active_imu = []
        active_odom = []
        if self.args.exercise:
            print("\nRunning active exercise...")
            imu_count1 = len(self.imu_msgs)
            odom_count1 = len(self.odom_msgs)

            self.publish_cmd(self.args.forward_v, 0.0, self.args.forward_s)
            self.sleep_and_spin(self.args.pause_s)
            self.publish_cmd(0.0, self.args.spin_w, self.args.spin_s)
            self.sleep_and_spin(self.args.pause_s)

            active_imu = self.imu_msgs[imu_count1:]
            active_odom = self.odom_msgs[odom_count1:]
            self.report_phase("active", active_imu, active_odom)

        return self.evaluate(trans, tf_err, passive_imu, active_imu)

    def evaluate(self, trans, tf_err, passive_imu, active_imu):
        problems = []

        if tf_err:
            problems.append("TF base_link->imu_link unavailable")
        elif trans is not None:
            dx = abs(trans[0] - self.args.expect_x)
            dy = abs(trans[1] - self.args.expect_y)
            dz = abs(trans[2] - self.args.expect_z)
            if dx > self.args.tf_tol or dy > self.args.tf_tol or dz > self.args.tf_tol:
                problems.append(
                    "IMU TF differs from expected mounting pose: "
                    f"got ({trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f})"
                )

        passive_rate = self.estimate_rate(passive_imu)
        if passive_rate is None or passive_rate < self.args.min_rate:
            problems.append("IMU rate too low")

        if passive_imu:
            frames = {m.header.frame_id for m in passive_imu}
            if self.args.expect_frame and self.args.expect_frame not in frames:
                problems.append(
                    f"IMU header.frame_id not as expected, got {sorted(frames)}"
                )
        else:
            problems.append("No IMU messages received")

        if self.args.exercise:
            if not active_imu:
                problems.append("No IMU messages during active exercise")
            else:
                wz_peak = max(abs(m.angular_velocity.z) for m in active_imu)
                ay_peak = max(abs(m.linear_acceleration.y) for m in active_imu)
                if wz_peak < self.args.min_spin_response:
                    problems.append(
                        f"Angular velocity response too small during spin: {wz_peak:.6f}"
                    )
                if ay_peak < self.args.min_accel_response:
                    problems.append(
                        f"Lateral acceleration response too small during exercise: {ay_peak:.6f}"
                    )

        print("\n[verdict]")
        if problems:
            for item in problems:
                print(f"- FAIL: {item}")
            return 1

        print("- PASS: IMU topic, TF, and motion response look sane")
        return 0


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--imu-frame", default="imu_link")
    parser.add_argument("--expect-frame", default="imu_link")
    parser.add_argument("--expect-x", type=float, default=0.10)
    parser.add_argument("--expect-y", type=float, default=-0.045)
    parser.add_argument("--expect-z", type=float, default=0.0)
    parser.add_argument("--tf-tol", type=float, default=0.02)
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--passive-s", type=float, default=3.0)
    parser.add_argument("--exercise", action="store_true")
    parser.add_argument("--forward-v", type=float, default=0.25)
    parser.add_argument("--forward-s", type=float, default=2.0)
    parser.add_argument("--spin-w", type=float, default=0.6)
    parser.add_argument("--spin-s", type=float, default=2.0)
    parser.add_argument("--pause-s", type=float, default=0.5)
    parser.add_argument("--min-rate", type=float, default=40.0)
    parser.add_argument("--min-spin-response", type=float, default=0.05)
    parser.add_argument("--min-accel-response", type=float, default=0.02)
    return parser


def main():
    args = build_parser().parse_args()
    rospy.init_node("validate_sim_imu", anonymous=False)
    validator = ImuValidator(args)
    sys.exit(validator.validate())


if __name__ == "__main__":
    main()
