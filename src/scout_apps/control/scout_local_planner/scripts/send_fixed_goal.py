#!/usr/bin/env python3

import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish a fixed navigation goal to a ROS topic."
    )
    parser.add_argument("--goal-topic", default="/scout/goal")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--repeat-count", type=int, default=5)
    parser.add_argument("--repeat-rate", type=float, default=5.0)
    parser.add_argument("--wait-subscriber-timeout", type=float, default=2.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("send_fixed_goal")
    args = parse_args()

    pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=True)

    timeout = rospy.Time.now() + rospy.Duration(max(0.0, args.wait_subscriber_timeout))
    rate = rospy.Rate(20.0)
    while (not rospy.is_shutdown()
           and pub.get_num_connections() == 0
           and rospy.Time.now() < timeout):
        rate.sleep()

    msg = PoseStamped()
    msg.header.frame_id = args.frame
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(args.yaw)

    rospy.loginfo(
        "Publishing fixed goal: topic=%s frame=%s x=%.3f y=%.3f yaw=%.3f",
        args.goal_topic,
        args.frame,
        args.x,
        args.y,
        args.yaw,
    )

    pub_rate = rospy.Rate(max(0.1, args.repeat_rate))
    for _ in range(max(1, args.repeat_count)):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        pub_rate.sleep()

    rospy.loginfo("Goal publish done")


if __name__ == "__main__":
    main()
