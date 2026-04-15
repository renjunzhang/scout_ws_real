#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry


class OdomMonotonicRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "/odom")
        self.output_topic = rospy.get_param("~output", "/odom_carto")
        self.last_stamp_ns = None

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=50)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=100)

    def cb(self, msg):
        stamp_ns = msg.header.stamp.to_nsec()
        if stamp_ns == 0:
            return
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns
        self.pub.publish(msg)


def main():
    rospy.init_node("odom_monotonic_relay", anonymous=False)
    OdomMonotonicRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
