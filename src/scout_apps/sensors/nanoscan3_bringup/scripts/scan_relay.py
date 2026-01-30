#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan


class ScanRelay:
    def __init__(self):
        self.in_topic = rospy.get_param("~input", "/scan")
        self.out_topic = rospy.get_param("~output", "/scan_front")
        self.frame_id = rospy.get_param("~frame_id", "")

        self.pub = rospy.Publisher(self.out_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, LaserScan, self.cb, queue_size=10)

    def cb(self, msg):
        if not self.frame_id:
            self.pub.publish(msg)
            return

        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self.pub.publish(out)


def main():
    rospy.init_node("scan_relay", anonymous=False)
    ScanRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
