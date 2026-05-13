#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Online low-excitation path post-processor for nav_msgs/Path.

This version generates a few smoothed geometry candidates, gates them by
drift/length/endpoints/collision, and selects a moderate curvature-reduction
candidate. Collision checks rely on the global costmap's inflation layer
(point-cost threshold), not an explicit footprint polygon.
"""

import math

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from generate_anti_slosh_path_candidates import path_metrics, resample_path
from oscrs.generators.georef import generate_georef_candidates_with_meta
from oscrs.path_utils import clamp, sanitize_points, yaw_to_quat
from oscrs.diagnostics import build_metrics_array, format_report, format_safety_alarm
from oscrs.feasibility import check_collision_on_grid
from oscrs.pipeline import run_pipeline


METRIC_LABEL = (
    "selected_index,score,length_ratio,max_drift_m,endpoint_error_m,"
    "kappa_p95,kappa_max,dkappa_p95,dkappa_max,candidate_count,accepted_count"
)


def points_from_path(msg):
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


def copy_orientation(orientation):
    q = Quaternion()
    q.x = orientation.x
    q.y = orientation.y
    q.z = orientation.z
    q.w = orientation.w
    return q


def path_to_msg(points, header, endpoint_orientations=None):
    msg = NavPath()
    msg.header = header
    msg.poses = []
    for i, point in enumerate(points):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0.0
        if endpoint_orientations and i == 0:
            pose.pose.orientation = copy_orientation(endpoint_orientations[0])
            msg.poses.append(pose)
            continue
        if endpoint_orientations and i == len(points) - 1:
            pose.pose.orientation = copy_orientation(endpoint_orientations[1])
            msg.poses.append(pose)
            continue

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


class AntiSloshPathPostProcessor:
    def __init__(self):
        self.config = self.load_config(str(rospy.get_param("~oscrs_config", "")))
        self.input_topic = rospy.get_param("~input_topic", "/scout/global_path_raw")
        self.output_topic = rospy.get_param("~output_topic", "/scout/global_path_anti_slosh")
        self.ds = max(0.02, float(rospy.get_param("~ds", 0.10)))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.fixed_candidate_name = str(rospy.get_param("~fixed_candidate_name", "")).strip().lower()

        self.min_segment_length = max(1e-4, float(rospy.get_param("~gates/min_segment_length", 0.02)))
        self.max_drift = max(0.0, float(rospy.get_param("~gates/max_drift", 0.18)))
        self.max_length_ratio = max(1.0, float(rospy.get_param("~gates/max_length_ratio", 1.15)))
        self.min_length_ratio = min(1.0, float(rospy.get_param("~gates/min_length_ratio", 0.995)))
        self.min_kappa_ratio = max(0.0, float(rospy.get_param("~gates/min_kappa_ratio", 0.20)))
        self.target_kappa_ratio = clamp(float(rospy.get_param("~score/target_kappa_ratio", 0.35)), 0.0, 1.0)
        self.max_endpoint_error = max(0.0, float(rospy.get_param("~gates/max_endpoint_error", 0.05)))
        self.enable_collision_check = bool(rospy.get_param("~gates/enable_collision_check", False))
        self.ay_ratio_limit = max(0.0, float(rospy.get_param("~gates/ay_ratio_limit", 1.0)))
        self.collision_threshold = int(clamp(int(rospy.get_param("~gates/collision_threshold", 50)), 1, 100))
        self.unknown_is_obstacle = bool(rospy.get_param("~gates/unknown_is_obstacle", True))
        self.costmap_topic = str(rospy.get_param("~costmap_topic", "/scout/mbf_costmap_nav/global_costmap/costmap"))
        self.max_candidate_level = str(rospy.get_param("~max_candidate_level", "medium")).lower()
        self.tail_protect_enable = bool(rospy.get_param("~tail_protect/enable", False))
        self.tail_gate_enable = bool(rospy.get_param("~tail_protect/gate_enable", False))
        self.tail_protect_distance = max(0.0, float(rospy.get_param("~tail_protect/distance", 0.6)))
        self.tail_protect_mode = str(rospy.get_param("~tail_protect/mode", "replace_raw_tail"))
        self.tail_deviation_limit = max(0.0, float(rospy.get_param("~tail_protect/deviation_limit", 0.05)))
        self.terminal_tail_heading_limit_deg = max(
            0.0, float(rospy.get_param("~tail_protect/heading_limit_deg", 10.0))
        )
        self.predict_v_max = max(0.01, float(rospy.get_param("~prediction/v_max", 2.0)))
        self.predict_ay_max = max(0.01, float(rospy.get_param("~prediction/ay_max_budget", 2.0)))
        self.predict_a_max = max(0.01, float(rospy.get_param("~prediction/a_max", 1.0)))
        self.predict_v_init = max(0.0, float(rospy.get_param("~prediction/v_init", 0.0)))

        self.slosh_score_enable = bool(self.param_or_cfg("~slosh_score/enable", "slosh_score.enable", False))
        self.slosh_omega_n = max(0.01, float(self.param_or_cfg("~slosh_score/omega_n", "slosh_score.omega_n", 31.25)))
        self.slosh_zeta = max(0.0, float(self.param_or_cfg("~slosh_score/zeta", "slosh.damping_ratio", 0.05)))
        self.slosh_rollout_dt = max(0.005, float(self.param_or_cfg("~slosh_score/dt", "oscrs.rollout_dt", 0.05)))
        self.slosh_v_floor = max(0.01, float(self.param_or_cfg("~slosh_score/v_floor", "oscrs.v_floor", 0.05)))
        self.slosh_height_coeff = float(self.param_or_cfg("~slosh_score/height_coeff", "slosh_score.height_coeff", 1.0))
        self.slosh_container_radius = max(0.0, float(self.param_or_cfg("~slosh_score/container_radius", "slosh.container_radius", 0.0185)))
        self.slosh_offset_x = float(self.param_or_cfg("~slosh_score/offset_x", "slosh.offset_x", 0.0))
        self.slosh_offset_y = float(self.param_or_cfg("~slosh_score/offset_y", "slosh.offset_y", 0.0))
        self.slosh_use_parabola = bool(self.param_or_cfg("~slosh_score/use_parabola_term", "slosh.use_parabola_term", True))
        self.w_slosh_h = float(self.param_or_cfg("~slosh_score/w_h", "slosh_score.w_h", 0.0))
        self.w_slosh_energy = float(self.param_or_cfg("~slosh_score/w_energy", "slosh_score.w_energy", 1.0))
        self.w_slosh_eta_dot = float(self.param_or_cfg("~slosh_score/w_eta_dot", "slosh_score.w_eta_dot", 0.5))
        self.w_slosh_terminal = float(self.param_or_cfg("~slosh_score/w_terminal", "slosh_score.w_terminal", 0.2))
        self.w_slosh_kappa = float(self.param_or_cfg("~slosh_score/w_kappa", "slosh_score.w_kappa", 1.0))
        self.w_slosh_dkappa = float(self.param_or_cfg("~slosh_score/w_dkappa", "slosh_score.w_dkappa", 0.5))
        self.w_slosh_ay = float(self.param_or_cfg("~slosh_score/w_ay", "slosh_score.w_ay", 0.0))
        self.w_slosh_length = float(self.param_or_cfg("~slosh_score/w_length", "slosh_score.w_length", 0.3))
        self.w_slosh_drift = float(self.param_or_cfg("~slosh_score/w_drift", "slosh_score.w_drift", 0.5))
        self.oscrs_shadow_enable = bool(rospy.get_param("~oscrs/shadow_enable", False))
        self.oscrs_active_enable = bool(rospy.get_param("~oscrs/active_enable", False))
        self.oscrs_eta_lim = max(1e-6, float(self.param_or_cfg("~oscrs/eta_lim_mm", "oscrs.eta_lim_mm", 25.0)) / 1000.0)
        self.oscrs_residual_ratio = max(0.0, float(self.param_or_cfg("~oscrs/residual_ratio", "oscrs.residual_ratio", 0.2)))
        self.oscrs_settle_duration = max(0.0, float(self.param_or_cfg("~oscrs/settle_duration", "oscrs.settle_duration", 2.0)))
        self.oscrs_use_legacy_score = bool(self.param_or_cfg("~oscrs/score/use_legacy_score", "oscrs.score.use_legacy_score", False))
        self.oscrs_score_batch_norm = bool(self.param_or_cfg("~oscrs/score/batch_normalize", "oscrs.score.batch_normalize", True))
        self.oscrs_score_w_h_p95 = float(self.param_or_cfg("~oscrs/score/w_h_p95", "oscrs.score.w_h_p95", 1.0))
        self.oscrs_score_w_energy = float(self.param_or_cfg("~oscrs/score/w_energy_rms", "oscrs.score.w_energy_rms", 0.3))
        self.oscrs_score_w_eta_dot = float(self.param_or_cfg("~oscrs/score/w_eta_dot_rms", "oscrs.score.w_eta_dot_rms", 0.3))
        self.oscrs_score_w_terminal = float(self.param_or_cfg("~oscrs/score/w_terminal_E", "oscrs.score.w_terminal_E", 0.2))
        self.oscrs_score_w_geom = float(self.param_or_cfg("~oscrs/score/w_geom", "oscrs.score.w_geom", 0.2))
        self.oscrs_alarm_topic = str(self.param_or_cfg("~oscrs/alarm/topic", "oscrs.alarm.topic", "/anti_slosh_path/safety_alarm"))
        self.oscrs_alarm_rate_limit = max(0.0, float(self.param_or_cfg("~oscrs/alarm/rate_limit_sec", "oscrs.alarm.rate_limit_sec", 5.0)))
        self.oscrs_alarm_last_t = 0.0

        self.w_kappa = float(rospy.get_param("~score/w_kappa", 1.0))
        self.w_dkappa = float(rospy.get_param("~score/w_dkappa", 0.5))
        self.w_length = float(rospy.get_param("~score/w_length", 0.3))
        self.w_drift = float(rospy.get_param("~score/w_drift", 0.5))
        self.w_shortening = float(rospy.get_param("~score/w_shortening", 10.0))
        self.w_over_smooth = float(rospy.get_param("~score/w_over_smooth", 2.0))

        self.candidate_specs = [
            ("original", 0, 0.0, 0.0),
            ("mild", self._candidate_param("mild", "iters", 18), self._candidate_param("mild", "gain", 0.35), self._candidate_param("mild", "max_drift", 0.08)),
            ("medium", self._candidate_param("medium", "iters", 40), self._candidate_param("medium", "gain", 0.45), self._candidate_param("medium", "max_drift", 0.12)),
            ("mid", self._candidate_param("mid", "iters", 24), self._candidate_param("mid", "gain", 0.30), self._candidate_param("mid", "max_drift", 0.07)),
            ("strong", self._candidate_param("strong", "iters", 56), self._candidate_param("strong", "gain", 0.55), self._candidate_param("strong", "max_drift", 0.18)),
        ]
        self.candidate_levels = {name: i for i, (name, _, _, _) in enumerate(self.candidate_specs)}
        if self.max_candidate_level not in self.candidate_levels:
            rospy.logwarn(
                "[anti_slosh_path_post_processor] invalid max_candidate_level=%s, using medium",
                self.max_candidate_level,
            )
            self.max_candidate_level = "medium"

        self.path_pub = rospy.Publisher(self.output_topic, NavPath, queue_size=1, latch=True)
        self.metrics_pub = rospy.Publisher("/anti_slosh_path/metrics", Float32MultiArray, queue_size=1)
        self.candidate_report_pub = rospy.Publisher("/anti_slosh_path/candidate_report", String, queue_size=1, latch=True)
        self.safety_alarm_pub = rospy.Publisher(self.oscrs_alarm_topic, String, queue_size=1, latch=True)
        self.debug_pubs = {}
        if self.publish_debug:
            for name in ("original", "mild", "medium", "mid", "strong"):
                self.debug_pubs[name] = rospy.Publisher(
                    f"/anti_slosh_path/debug/{name}", NavPath, queue_size=1, latch=True
                )

        self.latest_costmap = None
        if self.enable_collision_check:
            self.costmap_sub = rospy.Subscriber(
                self.costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1
            )
            rospy.loginfo(
                "[anti_slosh_path_post_processor] collision check enabled, costmap=%s thresh=%d unknown_obst=%s",
                self.costmap_topic,
                self.collision_threshold,
                self.unknown_is_obstacle,
            )

        self.sub = rospy.Subscriber(self.input_topic, NavPath, self.path_callback, queue_size=1)

        rospy.loginfo(
            "[anti_slosh_path_post_processor] %s -> %s ds=%.3f max_drift=%.3f slosh_score=%s oscrs_shadow=%s oscrs_active=%s",
            self.input_topic,
            self.output_topic,
            self.ds,
            self.max_drift,
            self.slosh_score_enable,
            self.oscrs_shadow_enable,
            self.oscrs_active_enable,
        )

    def _candidate_param(self, name, key, default):
        return rospy.get_param(f"~candidates/{name}/{key}", default)

    def load_config(self, path):
        if not path:
            return {}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            rospy.loginfo("[anti_slosh_path_post_processor] loaded oscrs_config=%s", path)
            return data
        except (OSError, yaml.YAMLError) as exc:
            rospy.logwarn("[anti_slosh_path_post_processor] failed to load oscrs_config=%s: %s", path, exc)
            return {}

    def cfg(self, dotted_key, default):
        cur = self.config
        for key in dotted_key.split("."):
            if not isinstance(cur, dict) or key not in cur:
                return default
            cur = cur[key]
        return cur

    def param_or_cfg(self, param_name, cfg_key, default):
        value = rospy.get_param(param_name, default)
        if value == default:
            return self.cfg(cfg_key, default)
        return value

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def check_candidate_collision(self, points, path_frame, skip_check):
        """Return (status, idx, cost)."""
        if skip_check:
            return ("accepted", -1, -1)
        grid = self.latest_costmap
        if grid is None:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] collision check enabled but no costmap received on %s",
                self.costmap_topic,
            )
            return ("no_costmap", -1, -1)
        costmap_frame = grid.header.frame_id or "map"
        path_frame = path_frame or "map"
        if path_frame != costmap_frame:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] path/costmap frame mismatch: %s != %s",
                path_frame,
                costmap_frame,
            )
            return ("frame_mismatch", -1, -1)
        info = grid.info
        res = info.resolution
        if res <= 0.0 or info.width <= 0 or info.height <= 0:
            return ("no_costmap", -1, -1)
        ox = info.origin.position.x
        oy = info.origin.position.y
        return check_collision_on_grid(
            points, ox, oy, res, info.width, info.height, grid.data,
            self.collision_threshold, self.unknown_is_obstacle,
        )

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

        base_metrics = path_metrics(base, base)
        base_length = max(1e-6, base_metrics["length_m"])

        candidates, generation_meta = generate_georef_candidates_with_meta(
            base, self.candidate_specs, self.min_segment_length, sanitize_points,
            self.candidate_levels, self.max_candidate_level,
            self.tail_protect_enable, self.tail_protect_distance, self.tail_protect_mode,
        )
        self.generation_policy = generation_meta.get("generation_policy", {})

        # Pre-compute collision results (requires ROS costmap)
        collision_results = {}
        for name, candidate in candidates:
            col = self.check_candidate_collision(
                candidate, msg.header.frame_id,
                skip_check=(not self.enable_collision_check) or name == "original",
            )
            collision_results[name] = col

        result = run_pipeline(
            base, candidates, base_metrics, base_length, collision_results, self,
        )

        if result.best_fallback:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] no candidate passed gates; publishing original path",
            )

        # ROS logging for fixed-candidate edge cases
        if result.fixed_not_found:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] fixed_candidate_name=%s not found; publishing original",
                self.fixed_candidate_name,
            )
        elif result.fixed_rejected:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] fixed candidate %s rejected (%s); publishing original",
                self.fixed_candidate_name,
                result.fixed_reject_reason,
            )

        if result.alarm_triggered:
            self.publish_safety_alarm(result.rows, result.geometry_best[0], result.best[0])

        self.publish_outputs(msg, result.rows, result.best, result.geometry_best, result.oscrs_best)

    def publish_safety_alarm(self, rows, geometry_row, published_row):
        now = rospy.get_time()
        if self.oscrs_alarm_rate_limit > 0 and (now - self.oscrs_alarm_last_t) < self.oscrs_alarm_rate_limit:
            return
        self.oscrs_alarm_last_t = now
        msg = format_safety_alarm(rows, geometry_row, published_row, self.oscrs_eta_lim)
        self.safety_alarm_pub.publish(String(data=msg))
        rospy.logwarn(
            "[anti_slosh_path_post_processor] OSCRS SAFETY_ALARM %s; published=%s slosh-fail",
            msg,
            published_row["name"],
        )

    def publish_outputs(self, raw_msg, rows, best, geometry_best, oscrs_best):
        best_row, best_points = best
        header = raw_msg.header
        if not header.frame_id:
            header.frame_id = "map"
        header.stamp = rospy.Time.now()
        endpoint_orientations = None
        if raw_msg.poses:
            endpoint_orientations = (
                raw_msg.poses[0].pose.orientation,
                raw_msg.poses[-1].pose.orientation,
            )

        if self.publish_debug:
            for row, points in rows:
                pub = self.debug_pubs.get(row["name"])
                if pub and pub.get_num_connections() > 0:
                    pub.publish(path_to_msg(points, header, endpoint_orientations))

        if best_row["name"] == "original":
            out_msg = raw_msg
            out_msg.header.stamp = header.stamp
            if not out_msg.header.frame_id:
                out_msg.header.frame_id = header.frame_id
            self.path_pub.publish(out_msg)
        else:
            self.path_pub.publish(path_to_msg(best_points, header, endpoint_orientations))
        self.publish_metrics(best_row, rows)
        self.publish_candidate_report(rows, best_row, geometry_best[0], oscrs_best[0] if oscrs_best else None)
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

    def publish_candidate_report(self, rows, best_row, geometry_row, oscrs_row):
        report = format_report(
            rows, best_row, geometry_row, oscrs_row,
            self.oscrs_shadow_enable, self.oscrs_active_enable,
        )
        self.candidate_report_pub.publish(String(data=report))
        rospy.loginfo_throttle(1.0, "[anti_slosh_path_post_processor] candidates: %s", report)

    def publish_metrics(self, best_row, rows):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = METRIC_LABEL
        dim.size = 11
        dim.stride = 11
        msg.layout.dim.append(dim)
        msg.data = build_metrics_array(best_row, rows)
        self.metrics_pub.publish(msg)


def main():
    rospy.init_node("anti_slosh_path_post_processor")
    AntiSloshPathPostProcessor()
    rospy.spin()


if __name__ == "__main__":
    main()
