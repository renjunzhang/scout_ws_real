#!/usr/bin/env python3
"""Generate or replay a straight fixed global path for real-world R0 collection.

This helper is intentionally small: it publishes a nav_msgs/Path and, optionally,
the terminal goal pose. It does not publish /cmd_vel and does not start planners.
"""

import argparse
import json
import math
from pathlib import Path

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath


def yaw_from_quaternion(quat):
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def ensure_parent_dir(path):
    path.parent.mkdir(parents=True, exist_ok=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Publish a straight fixed path in map frame. In generate mode the path "
            "starts at the current robot pose and extends forward along current yaw. "
            "In replay mode it republishes a saved JSON path."
        )
    )
    parser.add_argument("--mode", choices=("generate", "replay"), default="generate")
    parser.add_argument("--frame", default="map", help="World frame used for the path.")
    parser.add_argument("--base-frame", default="base_link", help="Robot base frame used in generate mode.")
    parser.add_argument("--tf-timeout", type=float, default=0.5)
    parser.add_argument("--length", type=float, default=4.0, help="Straight path length in meters for generate mode.")
    parser.add_argument("--spacing", type=float, default=0.05, help="Path pose spacing in meters.")
    parser.add_argument("--path-id", default="", help="Optional path identifier stored in the JSON metadata.")
    parser.add_argument("--path-file", default="", help="JSON path file to save in generate mode or load in replay mode.")
    parser.add_argument("--output-topic", default="/scout/global_path_fixed")
    parser.add_argument("--publish-goal", action="store_true", help="Also publish the terminal pose as a PoseStamped goal.")
    parser.add_argument("--goal-topic", default="/scout/goal")
    parser.add_argument("--publish-rate", type=float, default=2.0)
    parser.add_argument(
        "--publish-count",
        type=int,
        default=0,
        help="Number of publishes. 0 means keep publishing until Ctrl-C.",
    )
    parser.add_argument("--wait-subscriber-timeout", type=float, default=2.0)
    return parser.parse_args(rospy.myargv()[1:])


def wait_for_connections(pub, timeout_sec, label):
    timeout_sec = max(0.0, float(timeout_sec))
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
        rate.sleep()
    if pub.get_num_connections() == 0:
        rospy.logwarn("No subscribers detected for %s before timeout", label)


def lookup_robot_pose(frame, base_frame, tf_timeout):
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.2)
    transform = tf_buffer.lookup_transform(
        frame,
        base_frame,
        rospy.Time(0),
        rospy.Duration(max(0.0, float(tf_timeout))),
    )
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return float(translation.x), float(translation.y), yaw_from_quaternion(rotation)


def pose_stamped(frame_id, x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(yaw)
    return msg


def generate_straight_path(frame_id, start_x, start_y, yaw, length, spacing):
    length = max(0.30, float(length))
    spacing = max(0.01, float(spacing))
    intervals = max(1, int(math.ceil(length / spacing)))
    path_msg = NavPath()
    path_msg.header.frame_id = frame_id
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for idx in range(intervals + 1):
        s = length * float(idx) / float(intervals)
        path_msg.poses.append(pose_stamped(frame_id, start_x + cos_yaw * s, start_y + sin_yaw * s, yaw))
    return path_msg


def path_to_dict(path_msg, metadata):
    poses = []
    for pose_stamped_msg in path_msg.poses:
        pose = pose_stamped_msg.pose
        poses.append(
            {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "qx": float(pose.orientation.x),
                "qy": float(pose.orientation.y),
                "qz": float(pose.orientation.z),
                "qw": float(pose.orientation.w),
            }
        )
    return {"frame_id": path_msg.header.frame_id, "poses": poses, "metadata": metadata}


def save_path(path_file, path_msg, metadata):
    ensure_parent_dir(path_file)
    path_file.write_text(json.dumps(path_to_dict(path_msg, metadata), ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    rospy.loginfo("Saved straight fixed path to %s", path_file)


def load_path(path_file, default_frame):
    data = json.loads(path_file.read_text(encoding="utf-8"))
    frame_id = str(data.get("frame_id", "")).strip() or default_frame
    path_msg = NavPath()
    path_msg.header.frame_id = frame_id
    for item in data.get("poses", []):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(item["x"])
        msg.pose.position.y = float(item["y"])
        msg.pose.position.z = float(item.get("z", 0.0))
        msg.pose.orientation.x = float(item["qx"])
        msg.pose.orientation.y = float(item["qy"])
        msg.pose.orientation.z = float(item["qz"])
        msg.pose.orientation.w = float(item["qw"])
        path_msg.poses.append(msg)
    if len(path_msg.poses) < 2:
        raise RuntimeError("straight fixed path file contains fewer than two poses")
    rospy.loginfo("Loaded straight fixed path from %s", path_file)
    return path_msg


def publish_loop(path_msg, path_pub, goal_pub, args):
    wait_for_connections(path_pub, args.wait_subscriber_timeout, args.output_topic)
    if goal_pub is not None:
        wait_for_connections(goal_pub, args.wait_subscriber_timeout, args.goal_topic)

    rate = rospy.Rate(max(0.1, float(args.publish_rate)))
    publish_count = int(args.publish_count)
    count = 0
    terminal_goal = path_msg.poses[-1]

    rospy.loginfo(
        "Publishing straight fixed path to %s (%d poses, frame=%s); terminal goal publish=%s",
        args.output_topic,
        len(path_msg.poses),
        path_msg.header.frame_id,
        bool(goal_pub),
    )

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        path_msg.header.stamp = now
        for pose in path_msg.poses:
            pose.header.stamp = now
        path_pub.publish(path_msg)

        if goal_pub is not None:
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = path_msg.header.frame_id
            goal.pose = terminal_goal.pose
            goal_pub.publish(goal)

        count += 1
        if publish_count > 0 and count >= publish_count:
            rospy.loginfo("Straight fixed path publish finished after %d publishes", count)
            return
        rate.sleep()


def main():
    args = parse_args()
    rospy.init_node("publish_straight_fixed_path")

    path_file = Path(args.path_file).expanduser().resolve() if args.path_file else None
    if args.mode == "replay" and path_file is None:
        raise RuntimeError("--path-file is required in replay mode")

    if args.mode == "generate":
        start_x, start_y, start_yaw = lookup_robot_pose(args.frame, args.base_frame, args.tf_timeout)
        path_msg = generate_straight_path(args.frame, start_x, start_y, start_yaw, args.length, args.spacing)
        metadata = {
            "path_id": args.path_id,
            "template": "straight",
            "length_m": float(args.length),
            "spacing_m": float(args.spacing),
            "start_x": start_x,
            "start_y": start_y,
            "start_yaw": start_yaw,
            "base_frame": args.base_frame,
            "note": "Generated from current robot pose; Map-vref should remain disabled during R0 collection.",
        }
        if path_file is not None:
            save_path(path_file, path_msg, metadata)
    else:
        path_msg = load_path(path_file, args.frame)

    path_pub = rospy.Publisher(args.output_topic, NavPath, queue_size=1, latch=True)
    goal_pub = None
    if args.publish_goal:
        goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=True)

    publish_loop(path_msg, path_pub, goal_pub, args)


if __name__ == "__main__":
    main()
