#!/usr/bin/env python3
import math
import sys

import rospy
import tf
from nav_msgs.msg import Path


class GoalBenchmarkMonitor:
    def __init__(self):
        self.path_topic = rospy.get_param('~path_topic', '/scout/global_path_fixed')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.duration_sec = float(rospy.get_param('~duration_sec', 60.0))
        self.goal_tolerance_m = float(rospy.get_param('~goal_tolerance_m', 0.30))
        self.progress_period_sec = float(rospy.get_param('~progress_period_sec', 5.0))
        self.goal = None
        self.listener = tf.TransformListener()
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)

    def path_callback(self, msg):
        if not msg.poses:
            return
        pose = msg.poses[-1]
        frame_id = pose.header.frame_id or msg.header.frame_id or self.target_frame
        self.goal = (frame_id, pose.pose.position.x, pose.pose.position.y)

    def lookup_pose(self):
        self.listener.waitForTransform(self.target_frame,
                                       self.base_frame,
                                       rospy.Time(0),
                                       rospy.Duration(0.2))
        trans, _ = self.listener.lookupTransform(self.target_frame, self.base_frame, rospy.Time(0))
        return trans[0], trans[1]

    def run(self):
        start_wait = rospy.Time.now()
        while not rospy.is_shutdown() and self.goal is None:
            if (rospy.Time.now() - start_wait).to_sec() > 10.0:
                print('RESULT FAIL reason=missing_goal_path')
                return 2
            rospy.sleep(0.1)

        _, goal_x, goal_y = self.goal
        start_time = rospy.Time.now()
        next_progress = 0.0
        start_distance = None
        min_distance = None
        final_x = float('nan')
        final_y = float('nan')
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            try:
                x, y = self.lookup_pose()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                if elapsed >= self.duration_sec:
                    print('RESULT FAIL reason=tf_unavailable elapsed={:.3f} error={}'.format(elapsed, ex))
                    return 2
                rate.sleep()
                continue

            final_x, final_y = x, y
            distance = math.hypot(goal_x - x, goal_y - y)
            if start_distance is None:
                start_distance = distance
                min_distance = distance
                print('START_POSE x={:.6f} y={:.6f} goal_x={:.6f} goal_y={:.6f} distance={:.6f}'.format(
                    x, y, goal_x, goal_y, distance))
            min_distance = min(min_distance, distance)

            if distance <= self.goal_tolerance_m:
                print('RESULT PASS reason=goal_reached elapsed={:.3f} reach_time={:.3f} start_distance={:.6f} min_distance={:.6f} final_x={:.6f} final_y={:.6f} goal_x={:.6f} goal_y={:.6f}'.format(
                    elapsed, elapsed, start_distance, min_distance, x, y, goal_x, goal_y))
                return 0

            if elapsed >= self.duration_sec:
                print('RESULT FAIL reason=timeout_60s_not_reached elapsed={:.3f} reach_time=NA start_distance={:.6f} final_distance={:.6f} min_distance={:.6f} final_x={:.6f} final_y={:.6f} goal_x={:.6f} goal_y={:.6f}'.format(
                    elapsed, start_distance, distance, min_distance, final_x, final_y, goal_x, goal_y))
                return 1

            if elapsed >= next_progress:
                print('PROGRESS t={:.1f} x={:.6f} y={:.6f} distance={:.6f} min_distance={:.6f}'.format(
                    elapsed, x, y, distance, min_distance))
                next_progress += self.progress_period_sec
            rate.sleep()

        print('RESULT FAIL reason=ros_shutdown')
        return 2


if __name__ == '__main__':
    rospy.init_node('lt_dwa_60s_goal_benchmark_monitor')
    sys.exit(GoalBenchmarkMonitor().run())
