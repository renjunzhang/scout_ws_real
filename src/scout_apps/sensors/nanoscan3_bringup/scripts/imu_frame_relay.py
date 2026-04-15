#!/usr/bin/env python3

import copy

import rospy
from sensor_msgs.msg import Imu


class ImuFrameRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "/imu/data_raw")
        self.output_topic = rospy.get_param("~output", "/imu/data")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=100)
        self.sub = rospy.Subscriber(self.input_topic, Imu, self.cb, queue_size=100)

    def cb(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.frame_id
        self.pub.publish(out)


def main():
    rospy.init_node("imu_frame_relay", anonymous=False)
    ImuFrameRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
