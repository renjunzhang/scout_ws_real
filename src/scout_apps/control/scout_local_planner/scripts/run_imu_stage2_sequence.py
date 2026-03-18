#!/usr/bin/env python3
"""Publish a deterministic cmd_vel sequence for stage-2 IMU ay validation."""

import argparse
import sys
import time

import rospy
from geometry_msgs.msg import Twist


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run the stage-2 IMU ay validation cmd_vel sequence."
    )
    parser.add_argument("--topic", default="/cmd_vel", help="cmd_vel topic (default: /cmd_vel)")
    parser.add_argument("--rate", type=float, default=20.0, help="publish rate in Hz (default: 20)")
    parser.add_argument("--stop-1", type=float, default=5.0, help="initial stop duration in s")
    parser.add_argument("--left-duration", type=float, default=5.0, help="left-arc duration in s")
    parser.add_argument("--stop-2", type=float, default=3.0, help="middle stop duration in s")
    parser.add_argument("--right-duration", type=float, default=5.0, help="right-arc duration in s")
    parser.add_argument("--stop-3", type=float, default=5.0, help="final stop duration in s")
    parser.add_argument("--linear", type=float, default=0.30, help="linear speed for arc motion in m/s")
    parser.add_argument("--omega", type=float, default=0.30, help="angular speed magnitude for arc motion in rad/s")
    return parser


def publish_for(pub, rate, duration, linear_x, angular_z, label):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    end_time = rospy.Time.now() + rospy.Duration.from_sec(duration)
    rospy.loginfo("Stage2 sequence: %s for %.2fs (vx=%.3f, wz=%.3f)",
                  label, duration, linear_x, angular_z)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        pub.publish(twist)
        rate.sleep()


def main():
    parser = build_parser()
    args = parser.parse_args()

    rospy.init_node("run_imu_stage2_sequence", anonymous=True)
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    rate = rospy.Rate(max(1.0, args.rate))

    # Give subscribers a short moment to connect.
    rospy.sleep(0.5)

    try:
        publish_for(pub, rate, args.stop_1, 0.0, 0.0, "stop-1")
        publish_for(pub, rate, args.left_duration, args.linear, abs(args.omega), "left-arc")
        publish_for(pub, rate, args.stop_2, 0.0, 0.0, "stop-2")
        publish_for(pub, rate, args.right_duration, args.linear, -abs(args.omega), "right-arc")
        publish_for(pub, rate, args.stop_3, 0.0, 0.0, "stop-3")
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rate.sleep()
        rospy.loginfo("Stage2 sequence finished, published final zero cmd_vel")

    return 0


if __name__ == "__main__":
    sys.exit(main())
