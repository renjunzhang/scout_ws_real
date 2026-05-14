#!/usr/bin/env python3
"""Diagnose why Scout passes the terminal goal and turns back.

The script prefers the planner's own terminal debug topics:

    /terminal/goal_info
    /terminal/mode
    /terminal/recovery_latched

It can run live against ROS topics, or offline against a bag.
"""

import argparse
import math
import sys


GOAL_INFO_FIELDS = (
    "dx",
    "dy",
    "dist",
    "bearing",
    "goal_yaw_err",
    "has_goal_yaw",
    "position_reached",
    "pose_reached",
)


class TerminalOvershootAnalyzer:
    def __init__(self, args):
        self.args = args
        self.latest = {}
        self.sample_count = 0
        self.modes_seen = set()
        self.last_mode = None
        self.recovery_latched_seen = False
        self.overshoot_seen = False
        self.turnback_seen = False
        self.pose_not_reached_after_position = False
        self.solver_fail_near_goal = False

        self.min_dist = float("inf")
        self.min_dx = float("inf")
        self.max_abs_bearing_near = 0.0
        self.max_abs_yaw_err_near = 0.0
        self.max_cmd_v_near = 0.0
        self.max_odom_v_near = 0.0
        self.max_ref_v_near = 0.0
        self.max_pct_v_near = 0.0
        self.max_pct_slosh_near = 0.0

    def update(self, topic, msg, stamp):
        if topic == self.args.goal_info_topic:
            values = list(getattr(msg, "data", []))
            if len(values) >= 8:
                self.latest["goal_info"] = dict(zip(GOAL_INFO_FIELDS, values[:8]))
                self.latest["goal_stamp"] = stamp
                self.evaluate(stamp)
        elif topic == self.args.mode_topic:
            mode = str(getattr(msg, "data", ""))
            self.latest["mode"] = mode
            self.modes_seen.add(mode)
            if mode != self.last_mode:
                self.print_event(stamp, "mode_change", mode)
                self.last_mode = mode
            self.evaluate(stamp)
        elif topic == self.args.recovery_topic:
            latched = int(getattr(msg, "data", 0)) != 0
            self.latest["recovery_latched"] = latched
            self.recovery_latched_seen = self.recovery_latched_seen or latched
            self.evaluate(stamp)
        elif topic == self.args.cmd_topic:
            self.latest["cmd_v"] = float(getattr(msg.linear, "x", 0.0))
            self.latest["cmd_omega"] = float(getattr(msg.angular, "z", 0.0))
            self.evaluate(stamp)
        elif topic == self.args.odom_topic:
            self.latest["odom_v"] = float(getattr(msg.twist.twist.linear, "x", 0.0))
            self.latest["odom_omega"] = float(getattr(msg.twist.twist.angular, "z", 0.0))
            self.evaluate(stamp)
        elif topic == self.args.ref_v_topic:
            self.latest["ref_v"] = float(getattr(msg, "data", 0.0))
            self.evaluate(stamp)
        elif topic == self.args.status_val_topic:
            self.latest["status_val"] = int(getattr(msg, "data", 0))
            self.evaluate(stamp)
        elif topic == self.args.cost_topic:
            data = list(getattr(msg, "data", []))
            if len(data) >= 20:
                self.latest["pct_v"] = float(data[13])
                self.latest["pct_slosh_total"] = float(data[19])
            self.evaluate(stamp)

    def near_goal(self, info):
        return info["dist"] <= self.args.near_goal_dist

    def evaluate(self, stamp):
        info = self.latest.get("goal_info")
        if not info:
            return

        self.sample_count += 1
        self.min_dist = min(self.min_dist, info["dist"])
        self.min_dx = min(self.min_dx, info["dx"])

        mode = self.latest.get("mode", "UNKNOWN")
        near = self.near_goal(info)

        if info["dx"] < self.args.goal_behind_x:
            if not self.overshoot_seen:
                self.print_event(stamp, "overshoot", self.format_state())
            self.overshoot_seen = True

        if near:
            self.max_abs_bearing_near = max(self.max_abs_bearing_near, abs(info["bearing"]))
            self.max_abs_yaw_err_near = max(self.max_abs_yaw_err_near, abs(info["goal_yaw_err"]))
            self.max_cmd_v_near = max(self.max_cmd_v_near, abs(self.latest.get("cmd_v", 0.0)))
            self.max_odom_v_near = max(self.max_odom_v_near, abs(self.latest.get("odom_v", 0.0)))
            self.max_ref_v_near = max(self.max_ref_v_near, abs(self.latest.get("ref_v", 0.0)))
            self.max_pct_v_near = max(self.max_pct_v_near, abs(self.latest.get("pct_v", 0.0)))
            self.max_pct_slosh_near = max(
                self.max_pct_slosh_near,
                abs(self.latest.get("pct_slosh_total", 0.0)),
            )

            if int(info["position_reached"]) == 1 and int(info["pose_reached"]) == 0:
                self.pose_not_reached_after_position = True

            if int(self.latest.get("status_val", 1)) == 0:
                self.solver_fail_near_goal = True

        if mode == "ALIGN_TO_POINT" and (info["dx"] < self.args.goal_behind_x or self.overshoot_seen):
            if not self.turnback_seen:
                self.print_event(stamp, "turnback", self.format_state())
            self.turnback_seen = True

    def format_state(self):
        info = self.latest.get("goal_info", {})
        return (
            "mode={mode} latched={latched} dist={dist:.3f} dx={dx:.3f} dy={dy:.3f} "
            "bearing={bearing:.3f} yaw_err={yaw:.3f} cmd=({cmd_v:.3f},{cmd_w:.3f}) "
            "odom=({odom_v:.3f},{odom_w:.3f}) ref_v={ref_v:.3f} pct_v={pct_v:.1f} "
            "pct_slosh={pct_slosh:.1f}"
        ).format(
            mode=self.latest.get("mode", "UNKNOWN"),
            latched=int(bool(self.latest.get("recovery_latched", False))),
            dist=float(info.get("dist", float("nan"))),
            dx=float(info.get("dx", float("nan"))),
            dy=float(info.get("dy", float("nan"))),
            bearing=float(info.get("bearing", float("nan"))),
            yaw=float(info.get("goal_yaw_err", float("nan"))),
            cmd_v=float(self.latest.get("cmd_v", 0.0)),
            cmd_w=float(self.latest.get("cmd_omega", 0.0)),
            odom_v=float(self.latest.get("odom_v", 0.0)),
            odom_w=float(self.latest.get("odom_omega", 0.0)),
            ref_v=float(self.latest.get("ref_v", 0.0)),
            pct_v=float(self.latest.get("pct_v", 0.0)),
            pct_slosh=float(self.latest.get("pct_slosh_total", 0.0)),
        )

    def print_event(self, stamp, kind, detail):
        if self.args.quiet:
            return
        print("[{:.3f}] {}: {}".format(time_to_sec(stamp), kind, detail))

    def summary_lines(self):
        reasons = []
        suggestions = []

        if self.turnback_seen:
            reasons.append("GOAL_PASSED_THEN_TERMINAL_TURNBACK")
            suggestions.append(
                "terminal recovery entered ALIGN_TO_POINT after dx became negative; the turn-back is caused by passing the goal."
            )
        elif self.overshoot_seen:
            reasons.append("GOAL_PASSED")
            suggestions.append("dx became negative near the terminal goal, but ALIGN_TO_POINT was not observed.")

        if self.max_ref_v_near > self.args.ref_v_high:
            reasons.append("REFERENCE_SPEED_TOO_HIGH_NEAR_GOAL")
            suggestions.append(
                "reference/v_ref stayed high near the goal; check terminal speed profile, goal_capture, and fixed-path endpoint."
            )

        if self.max_cmd_v_near > self.args.cmd_v_high or self.max_odom_v_near > self.args.cmd_v_high:
            reasons.append("EXECUTION_SPEED_TOO_HIGH_NEAR_GOAL")
            suggestions.append(
                "cmd_vel or odom speed was still high near the goal; reduce terminal approach speed/rate or increase braking margin."
            )

        if self.pose_not_reached_after_position and self.max_abs_yaw_err_near > self.args.yaw_err_high:
            reasons.append("ENDPOINT_YAW_MISMATCH")
            suggestions.append(
                "position was reached but pose_reached stayed false with large yaw error; fix the fixed path endpoint orientation if yaw is not important."
            )

        if self.max_abs_bearing_near > self.args.bearing_high:
            reasons.append("LARGE_TERMINAL_BEARING")
            suggestions.append(
                "goal bearing was large near the end; terminal recovery will rotate before approaching."
            )

        if self.solver_fail_near_goal:
            reasons.append("SOLVER_FAILURE_NEAR_GOAL")
            suggestions.append("mpc/status_val reported failure near the goal; inspect solve fail logs and constraints.")

        if self.recovery_latched_seen and not self.overshoot_seen:
            reasons.append("TERMINAL_RECOVERY_ACTIVE")
            suggestions.append(
                "terminal recovery latched before overshoot; inspect terminal/mode to see whether it was aligning or approaching."
            )

        if not reasons:
            reasons.append("NO_CLEAR_OVERSHOOT_CAUSE_IN_RECORDED_TOPICS")
            suggestions.append("Record /terminal/goal_info, /terminal/mode, /cmd_vel, /odom, /reference/v_ref, and /mpc/cost_breakdown.")

        lines = [
            "==== terminal overshoot diagnosis ====",
            "samples={}".format(self.sample_count),
            "modes_seen={}".format(",".join(sorted(self.modes_seen)) if self.modes_seen else "-"),
            "recovery_latched_seen={}".format(int(self.recovery_latched_seen)),
            "overshoot_seen={}".format(int(self.overshoot_seen)),
            "turnback_seen={}".format(int(self.turnback_seen)),
            "min_dist={:.3f}".format(self.min_dist if math.isfinite(self.min_dist) else float("nan")),
            "min_dx={:.3f}".format(self.min_dx if math.isfinite(self.min_dx) else float("nan")),
            "max_abs_bearing_near={:.3f}".format(self.max_abs_bearing_near),
            "max_abs_yaw_err_near={:.3f}".format(self.max_abs_yaw_err_near),
            "max_ref_v_near={:.3f}".format(self.max_ref_v_near),
            "max_cmd_v_near={:.3f}".format(self.max_cmd_v_near),
            "max_odom_v_near={:.3f}".format(self.max_odom_v_near),
            "max_pct_v_near={:.1f}".format(self.max_pct_v_near),
            "max_pct_slosh_near={:.1f}".format(self.max_pct_slosh_near),
            "reasons={}".format(",".join(dict.fromkeys(reasons))),
            "",
            "suggestions:",
        ]
        lines.extend("- " + s for s in dict.fromkeys(suggestions))
        return lines

    def print_summary(self):
        for line in self.summary_lines():
            print(line)


def time_to_sec(stamp):
    if stamp is None:
        return 0.0
    if hasattr(stamp, "to_sec"):
        return stamp.to_sec()
    return float(stamp)


def live_monitor(args):
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32, Float32MultiArray, Int32, String

    rospy.init_node("diagnose_terminal_overshoot")
    analyzer = TerminalOvershootAnalyzer(args)

    def cb(topic):
        return lambda msg: analyzer.update(topic, msg, rospy.Time.now())

    rospy.Subscriber(args.goal_info_topic, Float32MultiArray, cb(args.goal_info_topic), queue_size=10)
    rospy.Subscriber(args.mode_topic, String, cb(args.mode_topic), queue_size=10)
    rospy.Subscriber(args.recovery_topic, Int32, cb(args.recovery_topic), queue_size=10)
    rospy.Subscriber(args.cmd_topic, Twist, cb(args.cmd_topic), queue_size=20)
    rospy.Subscriber(args.odom_topic, Odometry, cb(args.odom_topic), queue_size=20)
    rospy.Subscriber(args.ref_v_topic, Float32, cb(args.ref_v_topic), queue_size=20)
    rospy.Subscriber(args.status_val_topic, Int32, cb(args.status_val_topic), queue_size=20)
    rospy.Subscriber(args.cost_topic, Float32MultiArray, cb(args.cost_topic), queue_size=20)

    rospy.on_shutdown(analyzer.print_summary)
    print("Watching terminal overshoot topics. Press Ctrl+C to print summary.")
    rospy.spin()


def bag_monitor(args):
    import rosbag

    analyzer = TerminalOvershootAnalyzer(args)
    topics = [
        args.goal_info_topic,
        args.mode_topic,
        args.recovery_topic,
        args.cmd_topic,
        args.odom_topic,
        args.ref_v_topic,
        args.status_val_topic,
        args.cost_topic,
    ]
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            analyzer.update(topic, msg, stamp)
    analyzer.print_summary()


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", default="", help="Optional bag path. If omitted, run live ROS monitor.")
    parser.add_argument("--goal-info-topic", default="/terminal/goal_info")
    parser.add_argument("--mode-topic", default="/terminal/mode")
    parser.add_argument("--recovery-topic", default="/terminal/recovery_latched")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--ref-v-topic", default="/reference/v_ref")
    parser.add_argument("--status-val-topic", default="/mpc/status_val")
    parser.add_argument("--cost-topic", default="/mpc/cost_breakdown")
    parser.add_argument("--near-goal-dist", type=float, default=0.50)
    parser.add_argument("--goal-behind-x", type=float, default=-0.05)
    parser.add_argument("--ref-v-high", type=float, default=0.12)
    parser.add_argument("--cmd-v-high", type=float, default=0.10)
    parser.add_argument("--yaw-err-high", type=float, default=0.35)
    parser.add_argument("--bearing-high", type=float, default=0.80)
    parser.add_argument("--quiet", action="store_true", help="Suppress event lines and only print final summary.")
    return parser.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if args.bag:
        bag_monitor(args)
    else:
        live_monitor(args)


if __name__ == "__main__":
    main()
