#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def yaw_from_quaternion(quat):
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Capture, show, or replay one fixed simulation goal so repeated "
            "experiments can use the exact same goal pose."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("capture", "replay", "show"),
        required=True,
        help="capture current goal topic to file, replay file to goal topic, or show saved goal",
    )
    parser.add_argument(
        "--goal-file",
        required=True,
        help="JSON file used to save or load the fixed sim goal",
    )
    parser.add_argument("--goal-topic", default="/scout/goal")
    parser.add_argument("--capture-timeout", type=float, default=10.0)
    parser.add_argument("--repeat-count", type=int, default=5)
    parser.add_argument("--repeat-rate", type=float, default=5.0)
    parser.add_argument("--wait-subscriber-timeout", type=float, default=2.0)
    return parser.parse_args(rospy.myargv()[1:])


def ensure_parent_dir(path):
    path.parent.mkdir(parents=True, exist_ok=True)


def save_goal(path, msg):
    ensure_parent_dir(path)
    data = {
        "frame_id": msg.header.frame_id or "map",
        "x": float(msg.pose.position.x),
        "y": float(msg.pose.position.y),
        "yaw": float(yaw_from_quaternion(msg.pose.orientation)),
    }
    path.write_text(json.dumps(data, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    return data


def load_goal(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    return {
        "frame_id": str(data.get("frame_id", "map")).strip() or "map",
        "x": float(data["x"]),
        "y": float(data["y"]),
        "yaw": float(data.get("yaw", 0.0)),
    }


def capture_goal(args, goal_path):
    rospy.loginfo("Waiting for goal on %s ...", args.goal_topic)
    msg = rospy.wait_for_message(args.goal_topic, PoseStamped, timeout=max(0.1, args.capture_timeout))
    data = save_goal(goal_path, msg)
    rospy.loginfo(
        "Saved sim goal to %s: frame=%s x=%.3f y=%.3f yaw=%.3f",
        str(goal_path),
        data["frame_id"],
        data["x"],
        data["y"],
        data["yaw"],
    )


def show_goal(goal_path):
    data = load_goal(goal_path)
    print(f"goal_file: {goal_path}")
    print(f"frame_id: {data['frame_id']}")
    print(f"x: {data['x']:.6f}")
    print(f"y: {data['y']:.6f}")
    print(f"yaw: {data['yaw']:.6f}")


def replay_goal(args, goal_path):
    data = load_goal(goal_path)
    pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=True)

    timeout = rospy.Time.now() + rospy.Duration(max(0.0, args.wait_subscriber_timeout))
    wait_rate = rospy.Rate(20.0)
    while (not rospy.is_shutdown()
           and pub.get_num_connections() == 0
           and rospy.Time.now() < timeout):
        wait_rate.sleep()

    msg = PoseStamped()
    msg.header.frame_id = data["frame_id"]
    msg.pose.position.x = data["x"]
    msg.pose.position.y = data["y"]
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(data["yaw"])

    rospy.loginfo(
        "Replaying sim goal: topic=%s frame=%s x=%.3f y=%.3f yaw=%.3f",
        args.goal_topic,
        data["frame_id"],
        data["x"],
        data["y"],
        data["yaw"],
    )

    pub_rate = rospy.Rate(max(0.1, args.repeat_rate))
    for _ in range(max(1, args.repeat_count)):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        pub_rate.sleep()

    rospy.loginfo("Sim goal replay done")


def main():
    rospy.init_node("sim_fixed_goal_tool")
    args = parse_args()
    goal_path = Path(args.goal_file).expanduser()

    try:
        if args.mode == "capture":
            capture_goal(args, goal_path)
        elif args.mode == "show":
            show_goal(goal_path)
        else:
            replay_goal(args, goal_path)
    except Exception as exc:
        rospy.logerr("sim_fixed_goal_tool failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
