#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfBroadcaster:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.br = tf2_ros.TransformBroadcaster()
        self.last_stamp = None
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.cb, queue_size=100)

    def cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rospy.init_node("odom_tf_broadcaster", anonymous=False)
    OdomTfBroadcaster()
    rospy.spin()


if __name__ == "__main__":
    main()
