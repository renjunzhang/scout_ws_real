#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan


class ScanDegToRad:
    def __init__(self):
        self.in_topic = rospy.get_param("~input", "/scan")
        self.out_topic = rospy.get_param("~output", "/scan_front")
        self.input_in_degrees = rospy.get_param("~input_in_degrees", True)
        self.auto_detect = rospy.get_param("~auto_detect", True)

        self.pub = rospy.Publisher(self.out_topic, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, LaserScan, self.cb, queue_size=10)
        self.warned = False

    def cb(self, msg):
        use_deg = self.input_in_degrees
        if self.auto_detect:
            if abs(msg.angle_min) > 2.0 * math.pi or abs(msg.angle_max) > 2.0 * math.pi:
                use_deg = True
            else:
                use_deg = False

        if use_deg and not self.warned:
            rospy.logwarn("LaserScan angles look like degrees, converting to radians.")
            self.warned = True

        if not use_deg:
            self.pub.publish(msg)
            return

        out = LaserScan()
        out.header = msg.header
        out.angle_min = math.radians(msg.angle_min)
        out.angle_max = math.radians(msg.angle_max)
        out.angle_increment = math.radians(msg.angle_increment)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self.pub.publish(out)


def main():
    rospy.init_node("scan_deg_to_rad", anonymous=False)
    ScanDegToRad()
    rospy.spin()


if __name__ == "__main__":
    main()
