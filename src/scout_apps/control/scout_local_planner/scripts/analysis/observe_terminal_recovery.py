#!/usr/bin/env python3

import argparse
import math

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TerminalRecoveryObserver:
    def __init__(self, args):
        self.args = args

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_goal = None
        self.latest_cmd = None
        self.latest_odom = None
        self.latest_status = None
        self.last_mode = "NONE"

        self.goal_behind_seen = False
        self.align_to_point_seen = False
        self.approach_seen = False
        self.align_final_yaw_seen = False
        self.reached_seen = False

        self.min_goal_dist = float("inf")
        self.max_abs_bearing = 0.0
        self.max_cmd_v = 0.0
        self.max_cmd_omega = 0.0

        rospy.Subscriber(args.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber(args.cmd_topic, Twist, self.cmd_callback, queue_size=10)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(args.status_topic, String, self.status_callback, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, args.rate_hz)), self.on_timer)
        rospy.on_shutdown(self.print_summary)

    def goal_callback(self, msg):
        self.latest_goal = msg

    def cmd_callback(self, msg):
        self.latest_cmd = msg
        self.max_cmd_v = max(self.max_cmd_v, abs(msg.linear.x))
        self.max_cmd_omega = max(self.max_cmd_omega, abs(msg.angular.z))

    def odom_callback(self, msg):
        self.latest_odom = msg

    def status_callback(self, msg):
        self.latest_status = msg.data
        if msg.data == "REACHED":
            self.reached_seen = True

    def compute_goal_info(self):
        if self.latest_goal is None:
            return None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.latest_goal.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception:
            return None

        goal_base = tf2_geometry_msgs.do_transform_pose(self.latest_goal, tf_msg)
        dx = goal_base.pose.position.x
        dy = goal_base.pose.position.y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        goal_yaw = normalize_angle(yaw_from_quaternion(goal_base.pose.orientation))

        self.min_goal_dist = min(self.min_goal_dist, dist)
        self.max_abs_bearing = max(self.max_abs_bearing, abs(bearing))

        return {
            "dx": dx,
            "dy": dy,
            "dist": dist,
            "bearing": bearing,
            "goal_yaw_err": goal_yaw,
        }

    def classify_mode(self, goal_info):
        if self.latest_cmd is None:
            return "NONE"

        cmd_v = self.latest_cmd.linear.x
        cmd_w = self.latest_cmd.angular.z
        dist = goal_info["dist"]
        bearing = goal_info["bearing"]
        dx = goal_info["dx"]
        goal_yaw_err = goal_info["goal_yaw_err"]

        near_zero_v = abs(cmd_v) <= self.args.stop_v_eps
        turning = abs(cmd_w) >= self.args.turning_omega_eps
        position_reached = dist < self.args.goal_tolerance
        within_terminal_band = dist < self.args.enter_distance

        if not within_terminal_band:
            return "NONE"

        if position_reached and abs(goal_yaw_err) > self.args.yaw_tolerance and near_zero_v and turning:
            return "ALIGN_FINAL_YAW"

        if (dx < self.args.goal_behind_x or abs(bearing) > self.args.align_angle) and near_zero_v and turning:
            return "ALIGN_TO_POINT"

        if dist > self.args.goal_tolerance:
            if cmd_v >= self.args.approach_v_min and cmd_v <= self.args.approach_v_max:
                return "APPROACH_POINT"

        return "NONE"

    def on_timer(self, _event):
        goal_info = self.compute_goal_info()
        if goal_info is None:
            return

        if goal_info["dx"] < self.args.goal_behind_x:
            self.goal_behind_seen = True

        mode = self.classify_mode(goal_info)
        if mode == "ALIGN_TO_POINT":
            self.align_to_point_seen = True
        elif mode == "APPROACH_POINT":
            self.approach_seen = True
        elif mode == "ALIGN_FINAL_YAW":
            self.align_final_yaw_seen = True

        if mode != self.last_mode:
            rospy.loginfo(
                "[TerminalObs] mode=%s dist=%.3f dx=%.3f dy=%.3f bearing=%.3f yaw_err=%.3f cmd=(%.3f, %.3f) status=%s",
                mode,
                goal_info["dist"],
                goal_info["dx"],
                goal_info["dy"],
                goal_info["bearing"],
                goal_info["goal_yaw_err"],
                self.latest_cmd.linear.x if self.latest_cmd else float("nan"),
                self.latest_cmd.angular.z if self.latest_cmd else float("nan"),
                self.latest_status if self.latest_status is not None else "-",
            )
            self.last_mode = mode

    def print_summary(self):
        rospy.loginfo("========== Terminal Recovery Summary ==========")
        rospy.loginfo("goal_behind_seen=%s", self.goal_behind_seen)
        rospy.loginfo("align_to_point_seen=%s", self.align_to_point_seen)
        rospy.loginfo("approach_seen=%s", self.approach_seen)
        rospy.loginfo("align_final_yaw_seen=%s", self.align_final_yaw_seen)
        rospy.loginfo("reached_seen=%s", self.reached_seen)
        rospy.loginfo("min_goal_dist=%.3f", self.min_goal_dist if math.isfinite(self.min_goal_dist) else float("nan"))
        rospy.loginfo("max_abs_bearing=%.3f", self.max_abs_bearing)
        rospy.loginfo("max_cmd_v=%.3f", self.max_cmd_v)
        rospy.loginfo("max_cmd_omega=%.3f", self.max_cmd_omega)
        rospy.loginfo("================================================")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Observe whether terminal recovery behaviors appear during bag replay."
    )
    parser.add_argument("--goal-topic", default="/scout/goal")
    parser.add_argument("--cmd-topic", default="/cmd_vel_replay")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--status-topic", default="/mpc_status")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--rate-hz", type=float, default=20.0)

    parser.add_argument("--goal-tolerance", type=float, default=0.20)
    parser.add_argument("--yaw-tolerance", type=float, default=0.20)
    parser.add_argument("--enter-distance", type=float, default=0.35)
    parser.add_argument("--goal-behind-x", type=float, default=-0.05)
    parser.add_argument("--align-angle", type=float, default=1.0)

    parser.add_argument("--stop-v-eps", type=float, default=0.03)
    parser.add_argument("--turning-omega-eps", type=float, default=0.10)
    parser.add_argument("--approach-v-min", type=float, default=0.03)
    parser.add_argument("--approach-v-max", type=float, default=0.25)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("observe_terminal_recovery")
    args = parse_args()
    TerminalRecoveryObserver(args)
    rospy.loginfo(
        "Watching terminal recovery: goal=%s cmd=%s odom=%s status=%s",
        args.goal_topic,
        args.cmd_topic,
        args.odom_topic,
        args.status_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
