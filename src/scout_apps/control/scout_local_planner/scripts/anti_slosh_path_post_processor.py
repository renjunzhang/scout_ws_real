#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Online low-excitation path post-processor for nav_msgs/Path.

This first version intentionally stays simple: it generates a few smoothed
geometry candidates, gates them by drift/length/endpoints, then publishes the
lowest curvature/curvature-rate candidate. It does not do costmap collision
checking, so it should only be used in open fields or known-safe corridors.
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from generate_anti_slosh_path_candidates import (
    cumulative_s,
    dist,
    path_metrics,
    resample_path,
    smooth_path,
)


METRIC_LABEL = (
    "selected_index,score,length_ratio,max_drift_m,endpoint_error_m,"
    "kappa_p95,kappa_max,dkappa_p95,dkappa_max,candidate_count,accepted_count"
)


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def points_from_path(msg):
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


def sanitize_points(points, min_segment_length):
    if not points:
        return []
    out = [points[0]]
    for point in points[1:]:
        if dist(out[-1], point) >= min_segment_length:
            out.append(point)
    return out


def endpoint_error(points, reference):
    if not points or not reference:
        return float("inf")
    return max(dist(points[0], reference[0]), dist(points[-1], reference[-1]))


def direction_preserved(points, reference):
    if len(points) < 2 or len(reference) < 2:
        return False
    ax = points[-1][0] - points[0][0]
    ay = points[-1][1] - points[0][1]
    bx = reference[-1][0] - reference[0][0]
    by = reference[-1][1] - reference[0][1]
    return ax * bx + ay * by >= 0.0


def path_to_msg(points, header):
    msg = NavPath()
    msg.header = header
    msg.poses = []
    for i, point in enumerate(points):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0.0
        if i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - point[1], points[i + 1][0] - point[0])
        elif i > 0:
            yaw = math.atan2(point[1] - points[i - 1][1], point[0] - points[i - 1][0])
        else:
            yaw = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        msg.poses.append(pose)
    return msg


def safe_ratio(value, reference):
    denom = abs(reference)
    if denom < 1e-6:
        return 1.0 if abs(value) < 1e-6 else 1e6
    return value / denom


class AntiSloshPathPostProcessor:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scout/global_path_raw")
        self.output_topic = rospy.get_param("~output_topic", "/scout/global_path_anti_slosh")
        self.ds = max(0.02, float(rospy.get_param("~ds", 0.10)))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.min_segment_length = max(1e-4, float(rospy.get_param("~gates/min_segment_length", 0.02)))
        self.max_drift = max(0.0, float(rospy.get_param("~gates/max_drift", 0.18)))
        self.max_length_ratio = max(1.0, float(rospy.get_param("~gates/max_length_ratio", 1.15)))
        self.max_endpoint_error = max(0.0, float(rospy.get_param("~gates/max_endpoint_error", 0.05)))
        self.enable_collision_check = bool(rospy.get_param("~gates/enable_collision_check", False))

        self.w_kappa = float(rospy.get_param("~score/w_kappa", 1.0))
        self.w_dkappa = float(rospy.get_param("~score/w_dkappa", 0.5))
        self.w_length = float(rospy.get_param("~score/w_length", 0.3))
        self.w_drift = float(rospy.get_param("~score/w_drift", 0.5))

        self.candidate_specs = [
            ("original", 0, 0.0, 0.0),
            ("mild", self._candidate_param("mild", "iters", 18), self._candidate_param("mild", "gain", 0.35), self._candidate_param("mild", "max_drift", 0.08)),
            ("medium", self._candidate_param("medium", "iters", 40), self._candidate_param("medium", "gain", 0.45), self._candidate_param("medium", "max_drift", 0.12)),
            ("strong", self._candidate_param("strong", "iters", 56), self._candidate_param("strong", "gain", 0.55), self._candidate_param("strong", "max_drift", 0.18)),
        ]

        self.path_pub = rospy.Publisher(self.output_topic, NavPath, queue_size=1, latch=True)
        self.metrics_pub = rospy.Publisher("/anti_slosh_path/metrics", Float32MultiArray, queue_size=1)
        self.debug_pubs = {}
        if self.publish_debug:
            for name in ("original", "mild", "medium", "strong"):
                self.debug_pubs[name] = rospy.Publisher(
                    f"/anti_slosh_path/debug/{name}", NavPath, queue_size=1, latch=True
                )

        self.sub = rospy.Subscriber(self.input_topic, NavPath, self.path_callback, queue_size=1)

        if self.enable_collision_check:
            rospy.logwarn(
                "[anti_slosh_path_post_processor] collision check requested but not implemented; "
                "candidate gating will use drift/length only"
            )
        rospy.loginfo(
            "[anti_slosh_path_post_processor] %s -> %s ds=%.3f max_drift=%.3f",
            self.input_topic,
            self.output_topic,
            self.ds,
            self.max_drift,
        )

    def _candidate_param(self, name, key, default):
        return rospy.get_param(f"~candidates/{name}/{key}", default)

    def path_callback(self, msg):
        raw = sanitize_points(points_from_path(msg), self.min_segment_length)
        if len(raw) < 3:
            rospy.logwarn_throttle(1.0, "[anti_slosh_path_post_processor] raw path too short: %d", len(raw))
            self.path_pub.publish(msg)
            return

        base = resample_path(raw, self.ds)
        if len(base) < 3:
            rospy.logwarn_throttle(1.0, "[anti_slosh_path_post_processor] resampled path too short: %d", len(base))
            self.path_pub.publish(msg)
            return

        best = None
        rows = []
        base_metrics = path_metrics(base, base)
        base_length = max(1e-6, base_metrics["length_m"])

        for index, (name, iters, gain, drift_limit) in enumerate(self.candidate_specs):
            candidate = base if name == "original" else smooth_path(base, int(iters), float(gain), float(drift_limit))
            row = self.evaluate_candidate(index, name, candidate, base, base_metrics, base_length)
            rows.append((row, candidate))
            if row["accepted"] and (best is None or row["score"] < best[0]["score"]):
                best = (row, candidate)

        if best is None:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] no candidate passed gates; publishing original path",
            )
            best = rows[0]

        self.publish_outputs(msg, rows, best)

    def evaluate_candidate(self, index, name, candidate, base, base_metrics, base_length):
        metrics = path_metrics(candidate, base)
        length_ratio = metrics["length_m"] / base_length
        end_error = endpoint_error(candidate, base)
        accepted = (
            len(candidate) >= 3
            and metrics["min_seg_m"] >= self.min_segment_length
            and metrics["max_drift_m"] <= self.max_drift
            and length_ratio <= self.max_length_ratio
            and end_error <= self.max_endpoint_error
            and direction_preserved(candidate, base)
        )
        score = (
            self.w_kappa * safe_ratio(metrics["kappa_p95"], base_metrics["kappa_p95"])
            + self.w_dkappa * safe_ratio(metrics["dkappa_p95"], base_metrics["dkappa_p95"])
            + self.w_length * max(0.0, length_ratio - 1.0)
            + self.w_drift * metrics["max_drift_m"]
        )
        return {
            "index": index,
            "name": name,
            "accepted": accepted,
            "score": score,
            "length_ratio": length_ratio,
            "endpoint_error_m": end_error,
            **metrics,
        }

    def publish_outputs(self, raw_msg, rows, best):
        best_row, best_points = best
        header = raw_msg.header
        if not header.frame_id:
            header.frame_id = "map"
        header.stamp = rospy.Time.now()

        if self.publish_debug:
            for row, points in rows:
                pub = self.debug_pubs.get(row["name"])
                if pub and pub.get_num_connections() > 0:
                    pub.publish(path_to_msg(points, header))

        self.path_pub.publish(path_to_msg(best_points, header))
        self.publish_metrics(best_row, rows)
        rospy.loginfo_throttle(
            1.0,
            "[anti_slosh_path_post_processor] selected=%s score=%.3f accepted=%d/%d k95=%.3f dk95=%.3f drift=%.3f",
            best_row["name"],
            best_row["score"],
            sum(1 for row, _ in rows if row["accepted"]),
            len(rows),
            best_row["kappa_p95"],
            best_row["dkappa_p95"],
            best_row["max_drift_m"],
        )

    def publish_metrics(self, best_row, rows):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = METRIC_LABEL
        dim.size = 11
        dim.stride = 11
        msg.layout.dim.append(dim)
        msg.data = [
            float(best_row["index"]),
            float(best_row["score"]),
            float(best_row["length_ratio"]),
            float(best_row["max_drift_m"]),
            float(best_row["endpoint_error_m"]),
            float(best_row["kappa_p95"]),
            float(best_row["kappa_max"]),
            float(best_row["dkappa_p95"]),
            float(best_row["dkappa_max"]),
            float(len(rows)),
            float(sum(1 for row, _ in rows if row["accepted"])),
        ]
        self.metrics_pub.publish(msg)


def main():
    rospy.init_node("anti_slosh_path_post_processor")
    AntiSloshPathPostProcessor()
    rospy.spin()


if __name__ == "__main__":
    main()
