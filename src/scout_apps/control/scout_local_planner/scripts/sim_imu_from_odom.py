#!/usr/bin/env python3
"""Publish a minimal simulated IMU stream from odometry for scheduler tests."""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SimImuFromOdom:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.include_gravity = rospy.get_param("~include_gravity", True)
        self.gravity = rospy.get_param("~gravity", 9.81)

        self.prev_stamp = None
        self.prev_v = 0.0

        self.pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        rospy.loginfo(
            "[sim_imu_from_odom] publishing %s from %s (frame_id=%s)",
            self.imu_topic,
            self.odom_topic,
            self.frame_id,
        )

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        ax = 0.0
        if self.prev_stamp is not None:
            dt = (stamp - self.prev_stamp).to_sec()
            if dt > 1e-4:
                ax = (v - self.prev_v) / dt

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity.z = omega
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = v * omega
        imu.linear_acceleration.z = self.gravity if self.include_gravity else 0.0

        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1

        self.pub.publish(imu)
        self.prev_stamp = stamp
        self.prev_v = v


if __name__ == "__main__":
    rospy.init_node("sim_imu_from_odom")
    SimImuFromOdom()
    rospy.spin()
