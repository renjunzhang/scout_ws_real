#!/usr/bin/env python3

import argparse
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


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


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


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a template fixed path from the current robot pose to a clicked goal "
            "and publish it to /scout/global_path_fixed."
        )
    )
    parser.add_argument(
        "--template",
        choices=("straight", "single_turn", "s_curve", "mixed", "multi_s", "sharp_turn"),
        required=True,
        help="Template geometry used between the current robot pose and the clicked goal.",
    )
    parser.add_argument(
        "--goal-topic",
        default="/scout/goal",
        help="Topic providing the clicked terminal goal pose.",
    )
    parser.add_argument(
        "--output-topic",
        default="/scout/global_path_fixed",
        help="Topic used to publish the generated fixed path.",
    )
    parser.add_argument(
        "--path-file",
        default="",
        help="Optional JSON file used to save the generated fixed path.",
    )
    parser.add_argument(
        "--default-frame",
        default="map",
        help="Fallback frame when the clicked goal has no frame_id.",
    )
    parser.add_argument(
        "--base-frame",
        default="base_link",
        help="Robot base frame used as the path start pose.",
    )
    parser.add_argument(
        "--goal-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the clicked goal pose.",
    )
    parser.add_argument(
        "--tf-timeout",
        type=float,
        default=0.2,
        help="TF lookup timeout in seconds.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.05,
        help="Resampled path spacing in meters.",
    )
    parser.add_argument(
        "--amplitude-ratio",
        type=float,
        default=0.18,
        help="Template lateral offset scale relative to start-goal distance.",
    )
    parser.add_argument(
        "--min-amplitude",
        type=float,
        default=0.25,
        help="Minimum lateral offset in meters for curved templates.",
    )
    parser.add_argument(
        "--max-amplitude",
        type=float,
        default=1.20,
        help="Maximum lateral offset in meters for curved templates.",
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default="left",
        help="Template bending side for asymmetric path templates.",
    )
    parser.add_argument(
        "--start-heading",
        choices=("goal_chord", "current"),
        default="goal_chord",
        help=(
            "Path initial heading mode. goal_chord preserves the old behavior and points "
            "the template from start to clicked goal; current uses the robot's current "
            "heading as the initial path tangent while still ending at the clicked goal."
        ),
    )
    parser.add_argument(
        "--smooth-iterations",
        type=int,
        default=3,
        help="Number of Chaikin smoothing passes applied to control points.",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=2.0,
        help="Publish rate in Hz.",
    )
    parser.add_argument(
        "--publish-count",
        type=int,
        default=10,
        help="Number of publishes after generation. Use 0 to publish until Ctrl-C.",
    )
    parser.add_argument(
        "--wait-subscriber-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for at least one path subscriber.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def save_path_file(path_file, path_msg):
    ensure_parent_dir(path_file)
    payload = {"frame_id": path_msg.header.frame_id, "poses": []}
    for pose_stamped in path_msg.poses:
        pose = pose_stamped.pose
        payload["poses"].append(
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
    path_file.write_text(json.dumps(payload, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    rospy.loginfo("Saved generated fixed path to %s", path_file)


class TemplateFixedPathGenerator:
    def __init__(self, args):
        self.args = args
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.path_pub = rospy.Publisher(args.output_topic, NavPath, queue_size=1, latch=True)

    def wait_for_goal(self):
        timeout = max(0.1, float(self.args.goal_timeout))
        rospy.loginfo("Waiting for clicked goal on %s", self.args.goal_topic)
        goal_msg = rospy.wait_for_message(self.args.goal_topic, PoseStamped, timeout=timeout)
        frame_id = goal_msg.header.frame_id.strip() or self.args.default_frame
        goal_msg.header.frame_id = frame_id
        rospy.loginfo(
            "Received goal in frame %s: x=%.3f y=%.3f",
            frame_id,
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
        )
        return goal_msg

    def lookup_start_pose(self, frame_id):
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

    def compute_amplitude(self, chord_length):
        if self.args.template == "straight":
            return 0.0
        amplitude = chord_length * max(0.0, float(self.args.amplitude_ratio))
        amplitude = max(float(self.args.min_amplitude), amplitude)
        amplitude = min(float(self.args.max_amplitude), amplitude)
        sign = 1.0 if self.args.side == "left" else -1.0
        if self.args.template == "sharp_turn":
            amplitude *= 1.4
        elif self.args.template == "mixed":
            amplitude *= 0.95
        elif self.args.template == "multi_s":
            amplitude *= 0.9
        return sign * amplitude

    def control_points_local(self, chord_length, amplitude):
        L = chord_length
        A = amplitude
        if self.args.template == "straight":
            return [(0.0, 0.0), (L, 0.0)]
        if self.args.template == "single_turn":
            return [(0.0, 0.0), (0.25 * L, 0.0), (0.55 * L, A), (0.82 * L, 0.35 * A), (L, 0.0)]
        if self.args.template == "s_curve":
            return [(0.0, 0.0), (0.25 * L, A), (0.50 * L, 0.0), (0.75 * L, -A), (L, 0.0)]
        if self.args.template == "mixed":
            return [
                (0.0, 0.0),
                (0.18 * L, 0.0),
                (0.34 * L, 0.55 * A),
                (0.50 * L, 0.55 * A),
                (0.64 * L, -0.80 * A),
                (0.82 * L, 0.65 * A),
                (L, 0.0),
            ]
        if self.args.template == "multi_s":
            return [
                (0.0, 0.0),
                (0.18 * L, A),
                (0.36 * L, -A),
                (0.54 * L, A),
                (0.72 * L, -A),
                (L, 0.0),
            ]
        if self.args.template == "sharp_turn":
            return [
                (0.0, 0.0),
                (0.35 * L, 0.0),
                (0.46 * L, A),
                (0.62 * L, A),
                (0.82 * L, 0.20 * A),
                (L, 0.0),
            ]
        raise RuntimeError("Unsupported template {}".format(self.args.template))

    @staticmethod
    def chaikin_open(points, iterations):
        pts = list(points)
        for _ in range(max(0, int(iterations))):
            if len(pts) < 2:
                break
            refined = [pts[0]]
            for idx in range(len(pts) - 1):
                p0 = pts[idx]
                p1 = pts[idx + 1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                refined.extend([q, r])
            refined.append(pts[-1])
            pts = refined
        return pts

    @staticmethod
    def resample_polyline(points, spacing):
        if len(points) < 2:
            return list(points)
        spacing = max(1e-3, float(spacing))
        out = [points[0]]
        accumulated = 0.0
        prev = points[0]
        for current in points[1:]:
            seg_dx = current[0] - prev[0]
            seg_dy = current[1] - prev[1]
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len < 1e-9:
                prev = current
                continue
            while accumulated + seg_len >= spacing:
                remain = spacing - accumulated
                ratio = remain / seg_len
                new_pt = (prev[0] + ratio * seg_dx, prev[1] + ratio * seg_dy)
                out.append(new_pt)
                prev = new_pt
                seg_dx = current[0] - prev[0]
                seg_dy = current[1] - prev[1]
                seg_len = math.hypot(seg_dx, seg_dy)
                accumulated = 0.0
                if seg_len < 1e-9:
                    break
            accumulated += seg_len
            prev = current
        if math.hypot(out[-1][0] - points[-1][0], out[-1][1] - points[-1][1]) > 1e-6:
            out.append(points[-1])
        return out

    @staticmethod
    def rotate_to_world(start_x, start_y, theta, local_pts):
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        world_pts = []
        for x_local, y_local in local_pts:
            world_x = start_x + cos_t * x_local - sin_t * y_local
            world_y = start_y + sin_t * x_local + cos_t * y_local
            world_pts.append((world_x, world_y))
        return world_pts

    def build_chord_heading_points(self, start_x, start_y, goal_x, goal_y):
        dx = goal_x - start_x
        dy = goal_y - start_y
        chord_length = math.hypot(dx, dy)
        if chord_length < 0.30:
            raise RuntimeError("Clicked goal is too close to the robot start pose")
        theta = math.atan2(dy, dx)
        amplitude = self.compute_amplitude(chord_length)
        control_pts = self.control_points_local(chord_length, amplitude)
        smooth_pts = self.chaikin_open(control_pts, self.args.smooth_iterations)
        local_pts = self.resample_polyline(smooth_pts, self.args.spacing)
        return self.rotate_to_world(start_x, start_y, theta, local_pts)

    def build_current_heading_points(self, start_x, start_y, start_yaw, goal_x, goal_y):
        dx = goal_x - start_x
        dy = goal_y - start_y
        cos_t = math.cos(-start_yaw)
        sin_t = math.sin(-start_yaw)
        goal_x_local = cos_t * dx - sin_t * dy
        goal_y_local = sin_t * dx + cos_t * dy

        if goal_x_local < 0.30:
            raise RuntimeError(
                "Clicked goal must be at least 0.30 m in front of the robot for "
                "--start-heading current"
            )

        chord_length = math.hypot(goal_x_local, goal_y_local)
        amplitude = self.compute_amplitude(chord_length)
        control_pts = self.control_points_local(goal_x_local, amplitude)

        shaped_pts = []
        for x_local, y_local in control_pts:
            t = max(0.0, min(1.0, x_local / goal_x_local))
            # Cubic blend keeps the start tangent aligned with the current robot heading.
            y_goal_blend = (t ** 3) * goal_y_local
            shaped_pts.append((x_local, y_local + y_goal_blend))

        if len(shaped_pts) >= 2:
            guard_x = min(0.50, max(0.10, 0.15 * goal_x_local))
            shaped_pts.insert(1, (guard_x, 0.0))

        smooth_pts = self.chaikin_open(shaped_pts, self.args.smooth_iterations)
        local_pts = self.resample_polyline(smooth_pts, self.args.spacing)
        return self.rotate_to_world(start_x, start_y, start_yaw, local_pts)

    def build_world_points(self, start_x, start_y, start_yaw, goal_x, goal_y):
        if self.args.start_heading == "current":
            return self.build_current_heading_points(start_x, start_y, start_yaw, goal_x, goal_y)
        return self.build_chord_heading_points(start_x, start_y, goal_x, goal_y)

    @staticmethod
    def world_points_to_path(frame_id, world_pts):
        if len(world_pts) < 2:
            raise RuntimeError("Generated path contains too few points")
        path_msg = NavPath()
        path_msg.header.frame_id = frame_id
        for idx, (x, y) in enumerate(world_pts):
            if idx == 0:
                x_next, y_next = world_pts[1]
                yaw = math.atan2(y_next - y, x_next - x)
            elif idx == len(world_pts) - 1:
                x_prev, y_prev = world_pts[idx - 1]
                yaw = math.atan2(y - y_prev, x - x_prev)
            else:
                x_prev, y_prev = world_pts[idx - 1]
                x_next, y_next = world_pts[idx + 1]
                yaw = math.atan2(y_next - y_prev, x_next - x_prev)
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            path_msg.poses.append(pose_stamped)
        return path_msg

    def publish_path(self, path_msg):
        wait_for_connections(
            self.path_pub,
            self.args.wait_subscriber_timeout,
            self.args.output_topic,
        )
        publish_rate = rospy.Rate(max(0.1, float(self.args.publish_rate)))
        count = 0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            path_msg.header.stamp = now
            for pose in path_msg.poses:
                pose.header.stamp = now
            self.path_pub.publish(path_msg)
            count += 1
            if int(self.args.publish_count) > 0 and count >= int(self.args.publish_count):
                rospy.loginfo("Template fixed path publish finished after %d publishes", count)
                return
            publish_rate.sleep()


def main():
    args = parse_args()
    rospy.init_node("template_fixed_path_generator")

    path_file = Path(args.path_file).expanduser().resolve() if args.path_file else None
    generator = TemplateFixedPathGenerator(args)
    goal_msg = generator.wait_for_goal()
    frame_id = goal_msg.header.frame_id
    start_x, start_y, start_yaw = generator.lookup_start_pose(frame_id)
    goal_x = float(goal_msg.pose.position.x)
    goal_y = float(goal_msg.pose.position.y)
    world_pts = generator.build_world_points(start_x, start_y, start_yaw, goal_x, goal_y)
    path_msg = generator.world_points_to_path(frame_id, world_pts)

    rospy.loginfo(
        "Generated template path %s with %d poses in frame %s (start_heading=%s)",
        args.template,
        len(path_msg.poses),
        frame_id,
        args.start_heading,
    )

    if path_file is not None:
        save_path_file(path_file, path_msg)
    generator.publish_path(path_msg)


if __name__ == "__main__":
    main()
