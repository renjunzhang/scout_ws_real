#!/usr/bin/env python3
"""Development Gazebo actuator: delayed held commands and identified first order response.

Replaces the owned case's acceleration guard; the existing planar drive accepts
the resulting velocity. ROS transport and the 120 Hz output remain discretized.
This is a vehicle model adapter, not an independent liquid plant.
"""
import argparse
from collections import deque
import math
from pathlib import Path
import threading

import yaml


class DelayedAxis:
    def __init__(self, delay, tau, gain, timeout, start):
        if not all(math.isfinite(x) and x > 0 for x in (delay, tau, gain, timeout)):
            raise ValueError('actuator parameters must be finite and positive')
        self.delay, self.tau, self.gain, self.timeout = delay, tau, gain, timeout
        self.time = start
        self.value = self.target = 0.
        self.expiry = math.inf
        self.events = deque()

    def command(self, stamp, value):
        edge = stamp + self.delay
        if not math.isfinite(value) or edge < self.time or (self.events and edge < self.events[-1][0]):
            raise ValueError('invalid or out of order actuator command')
        self.events.append((edge, value))

    def _integrate(self, end):
        self.value = self.target + (self.value-self.target)*math.exp(-(end-self.time)/self.tau)
        self.time = end

    def _advance(self, end):
        if self.expiry <= end:
            self._integrate(self.expiry)
            self.target, self.expiry = 0., math.inf
        self._integrate(end)

    def sample(self, now):
        if now < self.time:
            raise ValueError('simulation clock regressed')
        while self.events and self.events[0][0] <= now:
            edge, command = self.events.popleft()
            self._advance(edge)
            self.target = self.gain*command
            self.expiry = edge+self.timeout
        self._advance(now)
        return self.value


def main():
    import rospy
    from geometry_msgs.msg import Twist

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    params = yaml.safe_load(args.config.read_text())['execution_model']
    if params['mode'] != 'explicit_actuator':
        raise ValueError('requires explicit_actuator configuration')
    rospy.init_node('identified_actuator_plant')
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('this adapter only runs with simulated ROS time')
    start = rospy.Time.now().to_sec()
    axes = [DelayedAxis(params[prefix+'_delay_sec'], params[prefix+'_tau_sec'],
                        params[prefix+'_gain'], params['cmd_timeout_sec'], start)
            for prefix in ('linear', 'angular')]
    lock = threading.Lock()
    publisher = rospy.Publisher('/cmd_vel_drive', Twist, queue_size=1)

    def command(msg):
        with lock:
            stamp = rospy.Time.now().to_sec()
            for axis, value in zip(axes, (msg.linear.x, msg.angular.z)):
                axis.command(stamp, value)

    def update(_event):
        with lock:
            now = rospy.Time.now().to_sec()
            msg = Twist()
            msg.linear.x, msg.angular.z = [axis.sample(now) for axis in axes]
            publisher.publish(msg)

    subscriber = rospy.Subscriber('/cmd_vel', Twist, command, queue_size=100, tcp_nodelay=True)
    timer = rospy.Timer(rospy.Duration(1./120.), update)
    rospy.on_shutdown(lambda: publisher.publish(Twist()))
    rospy.loginfo('Identified actuator plant at 120 Hz: %s', params)
    rospy.spin()


if __name__ == '__main__':
    main()
