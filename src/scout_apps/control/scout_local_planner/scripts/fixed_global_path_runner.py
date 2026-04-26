#!/usr/bin/env python3

import argparse
import copy
import json
import math
from pathlib import Path

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath


def yaw_from_quaternion(quat):
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Capture a live /scout/global_path once and replay the same fixed path "
            "after the robot is aligned back to the captured start pose."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("capture", "replay", "capture_and_replay", "goal_only"),
        default="capture_and_replay",
        help="Capture a live path, replay a saved path, do both, or only publish the captured start pose as a goal.",
    )
    parser.add_argument(
        "--input-topic",
        default="/scout/global_path",
        help="Source topic used for one-shot path capture.",
    )
    parser.add_argument(
        "--output-topic",
        default="/scout/global_path_fixed",
        help="Topic used to replay the fixed path.",
    )
    parser.add_argument(
        "--path-file",
        default="",
        help="Optional JSON file used to save or load the captured fixed path.",
    )
    parser.add_argument(
        "--capture-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the first non-empty live path.",
    )
    parser.add_argument(
        "--default-frame",
        default="map",
        help="Fallback frame when the captured path has no frame_id.",
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="Robot base frame used for start-pose alignment checks.",
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=0.2,
        help="TF lookup timeout in seconds.",
    )
    parser.add_argument(
        "--skip-start-wait",
        action="store_true",
        help="Start replay immediately instead of waiting to re-enter the captured start pose.",
    )
    parser.add_argument(
        "--manual-start",
        action="store_true",
        help="Wait for a terminal Enter before start gating/replay, useful for manual pose adjustment on the robot.",
    )
    parser.add_argument(
        "--start-pos-tol",
        type=float,
        default=0.05,
        help="Position tolerance in meters for start-pose gating.",
    )
    parser.add_argument(
        "--start-yaw-tol",
        type=float,
        default=0.10,
        help="Yaw tolerance in radians for start-pose gating.",
    )
    parser.add_argument(
        "--start-hold-sec",
        type=float,
        default=0.5,
        help="Required continuous in-tolerance time before replay starts.",
    )
    parser.add_argument(
        "--status-rate",
        type=float,
        default=2.0,
        help="Status print rate while waiting at the captured start pose.",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=2.0,
        help="Replay publish rate in Hz.",
    )
    parser.add_argument(
        "--publish-count",
        type=int,
        default=0,
        help="Number of replay publishes. Zero means publish until Ctrl-C.",
    )
    parser.add_argument(
        "--publish-once-keepalive",
        action="store_true",
        help="Publish the latched fixed path once, then keep the publisher alive without re-sending it.",
    )
    parser.add_argument(
        "--wait-path-subscriber-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for at least one replay-topic subscriber.",
    )
    parser.add_argument(
        "--publish-start-goal",
        action="store_true",
        help="Also publish the captured start pose to a goal topic as a convenience helper.",
    )
    parser.add_argument(
        "--goal-topic",
        default="/scout/goal",
        help="Goal topic used when --publish-start-goal is enabled.",
    )
    parser.add_argument(
        "--goal-repeat-count",
        type=int,
        default=5,
        help="How many times to publish the captured start pose.",
    )
    parser.add_argument(
        "--goal-repeat-rate",
        type=float,
        default=5.0,
        help="Goal publish rate in Hz.",
    )
    parser.add_argument(
        "--wait-goal-subscriber-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for at least one goal-topic subscriber.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def ensure_parent_dir(path):
    path.parent.mkdir(parents=True, exist_ok=True)


def wait_for_connections(pub, timeout_sec, label):
    timeout_sec = max(0.0, float(timeout_sec))
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    rate = rospy.Rate(20.0)
    while (
        not rospy.is_shutdown()
        and pub.get_num_connections() == 0
        and rospy.Time.now() < deadline
    ):
        rate.sleep()
    if pub.get_num_connections() == 0:
        rospy.logwarn("No subscribers detected for %s before timeout", label)


def sanitize_path_frame(path_msg, default_frame):
    frame_id = path_msg.header.frame_id.strip()
    if frame_id:
        return frame_id
    for pose in path_msg.poses:
        pose_frame = pose.header.frame_id.strip()
        if pose_frame:
            return pose_frame
    return default_frame


def path_to_dict(path_msg):
    frame_id = path_msg.header.frame_id
    poses = []
    for pose_stamped in path_msg.poses:
        pose = pose_stamped.pose
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
    return {"frame_id": frame_id, "poses": poses}


def dict_to_path(data, default_frame):
    frame_id = str(data.get("frame_id", "")).strip() or default_frame
    poses = data.get("poses", [])
    if not poses:
        raise RuntimeError("fixed path file contains no poses")
    path_msg = NavPath()
    path_msg.header.frame_id = frame_id
    for item in poses:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = float(item["x"])
        pose_stamped.pose.position.y = float(item["y"])
        pose_stamped.pose.position.z = float(item.get("z", 0.0))
        pose_stamped.pose.orientation.x = float(item["qx"])
        pose_stamped.pose.orientation.y = float(item["qy"])
        pose_stamped.pose.orientation.z = float(item["qz"])
        pose_stamped.pose.orientation.w = float(item["qw"])
        path_msg.poses.append(pose_stamped)
    return path_msg


def save_path_file(path_file, path_msg):
    ensure_parent_dir(path_file)
    payload = path_to_dict(path_msg)
    path_file.write_text(json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    rospy.loginfo("Saved fixed path to %s", path_file)


def load_path_file(path_file, default_frame):
    if not path_file.exists():
        raise FileNotFoundError(str(path_file))
    data = json.loads(path_file.read_text(encoding="utf-8"))
    path_msg = dict_to_path(data, default_frame)
    rospy.loginfo("Loaded fixed path from %s", path_file)
    return path_msg


class FixedGlobalPathRunner:
    def __init__(self, args):
        self.args = args
        self.captured_path = None
        self.capture_sub = None
        self.path_pub = None
        self.goal_pub = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if args.mode in ("capture", "capture_and_replay"):
            self.capture_sub = rospy.Subscriber(
                args.input_topic,
                NavPath,
                self.capture_callback,
                queue_size=1,
            )
        if args.mode in ("replay", "capture_and_replay"):
            self.path_pub = rospy.Publisher(
                args.output_topic,
                NavPath,
                queue_size=1,
                latch=True,
            )
        if args.publish_start_goal:
            self.goal_pub = rospy.Publisher(
                args.goal_topic,
                PoseStamped,
                queue_size=1,
                latch=True,
            )

    def capture_callback(self, msg):
        if self.captured_path is not None:
            return
        if not msg.poses:
            rospy.logwarn_throttle(1.0, "Ignoring empty path on %s", self.args.input_topic)
            return

        captured = copy.deepcopy(msg)
        captured.header.frame_id = sanitize_path_frame(captured, self.args.default_frame)
        for pose in captured.poses:
            pose.header.frame_id = captured.header.frame_id
        self.captured_path = captured
        rospy.loginfo(
            "Captured fixed path from %s with %d poses in frame %s",
            self.args.input_topic,
            len(captured.poses),
            captured.header.frame_id,
        )

    def capture_once(self):
        timeout_sec = max(0.0, float(self.args.capture_timeout))
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20.0)
        while (
            not rospy.is_shutdown()
            and self.captured_path is None
            and rospy.Time.now() < deadline
        ):
            rate.sleep()
        if self.captured_path is None:
            raise RuntimeError(
                "Timed out waiting for the first non-empty path on {}".format(
                    self.args.input_topic
                )
            )
        if self.capture_sub is not None:
            self.capture_sub.unregister()
            self.capture_sub = None
        return copy.deepcopy(self.captured_path)

    def publish_start_goal(self, path_msg):
        if self.goal_pub is None:
            return
        wait_for_connections(
            self.goal_pub,
            self.args.wait_goal_subscriber_timeout,
            self.args.goal_topic,
        )
        start_pose = path_msg.poses[0]
        pub_rate = rospy.Rate(max(0.1, float(self.args.goal_repeat_rate)))
        rospy.loginfo(
            "Publishing captured start pose to %s in frame %s",
            self.args.goal_topic,
            path_msg.header.frame_id,
        )
        for _ in range(max(1, int(self.args.goal_repeat_count))):
            if rospy.is_shutdown():
                break
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = path_msg.header.frame_id
            msg.pose = copy.deepcopy(start_pose.pose)
            self.goal_pub.publish(msg)
            pub_rate.sleep()

    def lookup_robot_pose(self, frame_id):
        transform = self.tf_buffer.lookup_transform(
            frame_id,
            self.args.base_frame,
            rospy.Time(0),
            rospy.Duration(max(0.0, float(self.args.tf_timeout))),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )

    def wait_until_at_start(self, path_msg):
        if self.args.skip_start_wait:
            rospy.loginfo("Skipping start-pose wait by request")
            return

        start_pose = path_msg.poses[0].pose
        start_x = float(start_pose.position.x)
        start_y = float(start_pose.position.y)
        start_yaw = yaw_from_quaternion(start_pose.orientation)
        pos_tol = max(0.0, float(self.args.start_pos_tol))
        yaw_tol = max(0.0, float(self.args.start_yaw_tol))
        hold_sec = max(0.0, float(self.args.start_hold_sec))
        ready_since = None
        rate = rospy.Rate(max(0.2, float(self.args.status_rate)))

        while not rospy.is_shutdown():
            try:
                robot_x, robot_y, robot_yaw = self.lookup_robot_pose(path_msg.header.frame_id)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(1.0, "Start-pose TF lookup failed: %s", exc)
                rate.sleep()
                continue

            pos_err = math.hypot(robot_x - start_x, robot_y - start_y)
            yaw_err = abs(wrap_angle(robot_yaw - start_yaw))
            in_tol = pos_err <= pos_tol and yaw_err <= yaw_tol

            if in_tol:
                if ready_since is None:
                    ready_since = rospy.Time.now()
                held_sec = (rospy.Time.now() - ready_since).to_sec()
                rospy.loginfo_throttle(
                    1.0,
                    "At fixed-path start gate: pos_err=%.3f m yaw_err=%.3f rad hold=%.2f/%.2f s",
                    pos_err,
                    yaw_err,
                    held_sec,
                    hold_sec,
                )
                if held_sec >= hold_sec:
                    rospy.loginfo(
                        "Start pose aligned in frame %s: pos_err=%.3f m yaw_err=%.3f rad",
                        path_msg.header.frame_id,
                        pos_err,
                        yaw_err,
                    )
                    return
            else:
                ready_since = None
                rospy.loginfo_throttle(
                    1.0,
                    "Waiting at fixed-path start: pos_err=%.3f m yaw_err=%.3f rad",
                    pos_err,
                    yaw_err,
                )
            rate.sleep()

    def wait_for_manual_start(self):
        if not self.args.manual_start:
            return
        rospy.loginfo(
            "Manual start armed. Adjust the robot pose, then press Enter to continue."
        )
        try:
            input("Press Enter to continue fixed-path replay...")
        except EOFError:
            rospy.logwarn("stdin is unavailable; continuing fixed-path replay immediately")

    def replay_path(self, path_msg):
        if self.path_pub is None:
            return

        wait_for_connections(
            self.path_pub,
            self.args.wait_path_subscriber_timeout,
            self.args.output_topic,
        )

        pub_rate = rospy.Rate(max(0.1, float(self.args.publish_rate)))
        count = 0
        replay_msg = copy.deepcopy(path_msg)
        for pose in replay_msg.poses:
            pose.header.frame_id = replay_msg.header.frame_id

        rospy.loginfo(
            "Replaying fixed path to %s with %d poses in frame %s",
            self.args.output_topic,
            len(replay_msg.poses),
            replay_msg.header.frame_id,
        )

        if self.args.publish_once_keepalive:
            now = rospy.Time.now()
            replay_msg.header.stamp = now
            for pose in replay_msg.poses:
                pose.header.stamp = now
            self.path_pub.publish(replay_msg)
            rospy.loginfo(
                "Published fixed path once to %s and keeping the latched publisher alive",
                self.args.output_topic,
            )
            rospy.spin()
            return

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            replay_msg.header.stamp = now
            for pose in replay_msg.poses:
                pose.header.stamp = now
            self.path_pub.publish(replay_msg)
            count += 1
            if int(self.args.publish_count) > 0 and count >= int(self.args.publish_count):
                rospy.loginfo("Fixed-path replay finished after %d publishes", count)
                return
            pub_rate.sleep()


def main():
    args = parse_args()
    rospy.init_node("fixed_global_path_runner")

    if args.mode == "goal_only" and not args.publish_start_goal:
        raise RuntimeError("--publish-start-goal is required in goal_only mode")

    path_file = None
    if args.path_file:
        path_file = Path(args.path_file).expanduser().resolve()

    runner = FixedGlobalPathRunner(args)
    path_msg = None

    if args.mode in ("capture", "capture_and_replay"):
        path_msg = runner.capture_once()
        if path_file is not None:
            save_path_file(path_file, path_msg)

    if args.mode in ("replay", "goal_only"):
        if path_file is None:
            raise RuntimeError("--path-file is required in replay or goal_only mode")
        path_msg = load_path_file(path_file, args.default_frame)

    if args.mode == "capture":
        return

    if path_msg is None:
        raise RuntimeError("No fixed path is available for replay")

    runner.publish_start_goal(path_msg)
    if args.mode == "goal_only":
        return
    runner.wait_for_manual_start()
    runner.wait_until_at_start(path_msg)
    runner.replay_path(path_msg)


if __name__ == "__main__":
    main()
