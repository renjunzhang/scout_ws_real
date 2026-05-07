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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from generate_anti_slosh_path_candidates import (
    cumulative_s,
    curvature_series,
    dist,
    path_metrics,
    percentile,
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
    for point in points[1:-1]:
        if dist(out[-1], point) >= min_segment_length:
            out.append(point)
    if len(points) > 1:
        last = points[-1]
        if dist(out[-1], last) >= min_segment_length or len(out) < 2:
            out.append(last)
        else:
            out[-1] = last
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


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class AntiSloshPathPostProcessor:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scout/global_path_raw")
        self.output_topic = rospy.get_param("~output_topic", "/scout/global_path_anti_slosh")
        self.ds = max(0.02, float(rospy.get_param("~ds", 0.10)))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

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
        self.predict_v_max = max(0.01, float(rospy.get_param("~prediction/v_max", 2.0)))
        self.predict_ay_max = max(0.01, float(rospy.get_param("~prediction/ay_max_budget", 2.0)))
        self.predict_a_max = max(0.01, float(rospy.get_param("~prediction/a_max", 1.0)))
        self.predict_v_init = max(0.0, float(rospy.get_param("~prediction/v_init", 0.0)))

        self.slosh_score_enable = bool(rospy.get_param("~slosh_score/enable", False))
        self.slosh_omega_n = max(0.01, float(rospy.get_param("~slosh_score/omega_n", 31.25)))
        self.slosh_zeta = max(0.0, float(rospy.get_param("~slosh_score/zeta", 0.05)))
        self.slosh_rollout_dt = max(0.005, float(rospy.get_param("~slosh_score/dt", 0.05)))
        self.slosh_v_floor = max(0.01, float(rospy.get_param("~slosh_score/v_floor", 0.05)))
        self.slosh_height_coeff = float(rospy.get_param("~slosh_score/height_coeff", 1.0))
        self.slosh_container_radius = max(0.0, float(rospy.get_param("~slosh_score/container_radius", 0.0185)))
        self.slosh_use_parabola = bool(rospy.get_param("~slosh_score/use_parabola_term", True))
        self.w_slosh_h = float(rospy.get_param("~slosh_score/w_h", 0.0))
        self.w_slosh_energy = float(rospy.get_param("~slosh_score/w_energy", 1.0))
        self.w_slosh_eta_dot = float(rospy.get_param("~slosh_score/w_eta_dot", 0.5))
        self.w_slosh_terminal = float(rospy.get_param("~slosh_score/w_terminal", 0.2))
        self.w_slosh_kappa = float(rospy.get_param("~slosh_score/w_kappa", 1.0))
        self.w_slosh_dkappa = float(rospy.get_param("~slosh_score/w_dkappa", 0.5))
        self.w_slosh_ay = float(rospy.get_param("~slosh_score/w_ay", 0.0))
        self.w_slosh_length = float(rospy.get_param("~slosh_score/w_length", 0.3))
        self.w_slosh_drift = float(rospy.get_param("~slosh_score/w_drift", 0.5))

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
        self.candidate_report_pub = rospy.Publisher("/anti_slosh_path/candidate_report", String, queue_size=1)
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
            "[anti_slosh_path_post_processor] %s -> %s ds=%.3f max_drift=%.3f slosh_score=%s",
            self.input_topic,
            self.output_topic,
            self.ds,
            self.max_drift,
            self.slosh_score_enable,
        )

    def _candidate_param(self, name, key, default):
        return rospy.get_param(f"~candidates/{name}/{key}", default)

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def predicted_lateral_profile(self, points):
        """Forward speed rollout with curvature speed cap; returns (ay_p95, vmax)."""
        if len(points) < 2:
            return (float("inf"), 0.0)
        _, _, v_values, _, ay_values = self.forward_profile(points)
        return (percentile([abs(v) for v in ay_values], 95.0), max(v_values) if v_values else 0.0)

    def forward_profile(self, points):
        s = cumulative_s(points)
        kappa = curvature_series(points)
        v_prev = min(self.predict_v_init, self.predict_v_max)
        v_values = []
        ax_values = []
        ay_values = []
        for i, k in enumerate(kappa):
            if i == 0:
                ds = 0.0
            else:
                ds = max(0.0, s[i] - s[i - 1])
            k_abs = abs(k)
            if k_abs > 1e-6:
                v_curv = math.sqrt(self.predict_ay_max / k_abs)
            else:
                v_curv = self.predict_v_max
            v_accel = math.sqrt(max(0.0, v_prev * v_prev + 2.0 * self.predict_a_max * ds))
            v = min(self.predict_v_max, v_curv, v_accel)
            ax = (v * v - v_prev * v_prev) / (2.0 * ds) if ds > 1e-6 else 0.0
            v_values.append(v)
            ax_values.append(ax)
            ay_values.append(v * v * k)
            v_prev = v
        return s, kappa, v_values, ax_values, ay_values

    @staticmethod
    def interp_piecewise(times, values, query_t):
        if not times:
            return 0.0
        if query_t <= times[0]:
            return values[0]
        if query_t >= times[-1]:
            return values[-1]
        lo = 0
        hi = len(times) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if times[mid] < query_t:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        t0 = times[i - 1]
        t1 = times[i]
        if t1 <= t0:
            return values[i]
        r = (query_t - t0) / (t1 - t0)
        return values[i - 1] * (1.0 - r) + values[i] * r

    def rollout_slosh_metrics(self, points):
        if len(points) < 2:
            return {
                "slosh_h_p95": float("inf"),
                "slosh_h_max": float("inf"),
                "slosh_h_modal_p95": float("inf"),
                "slosh_h_parabola_p95": float("inf"),
                "slosh_eta_dot_rms": float("inf"),
                "slosh_energy_rms": float("inf"),
                "slosh_terminal_E": float("inf"),
            }
        s, kappa, _, ax_values, ay_values = self.forward_profile(points)
        times = [0.0]
        _, _, v_values, _, _ = self.forward_profile(points)
        omega_values = [v * k for v, k in zip(v_values, kappa)]
        for i in range(1, len(s)):
            ds = max(0.0, s[i] - s[i - 1])
            times.append(times[-1] + ds / max(self.slosh_v_floor, v_values[i]))

        eta_x = eta_x_dot = eta_y = eta_y_dot = 0.0
        eta_dot_norm = []
        energy = []
        height_modal = []
        height_parabola = []
        height_total = []
        wn2 = self.slosh_omega_n * self.slosh_omega_n
        damping = 2.0 * self.slosh_zeta * self.slosh_omega_n
        radius2_over_4g = (self.slosh_container_radius * self.slosh_container_radius) / (4.0 * 9.81)
        t_end = times[-1] if times else 0.0
        steps = max(1, int(math.ceil(t_end / self.slosh_rollout_dt)))
        for step in range(steps + 1):
            query_t = min(t_end, step * self.slosh_rollout_dt)
            ux = self.interp_piecewise(times, ax_values, query_t)
            uy = self.interp_piecewise(times, ay_values, query_t)
            omega = self.interp_piecewise(times, omega_values, query_t)
            ddx = -damping * eta_x_dot - wn2 * eta_x - ux
            ddy = -damping * eta_y_dot - wn2 * eta_y - uy
            eta_x_dot += ddx * self.slosh_rollout_dt
            eta_y_dot += ddy * self.slosh_rollout_dt
            eta_x += eta_x_dot * self.slosh_rollout_dt
            eta_y += eta_y_dot * self.slosh_rollout_dt
            eta_dot = math.hypot(eta_x_dot, eta_y_dot)
            e = wn2 * (eta_x * eta_x + eta_y * eta_y) + eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot
            modal = self.slosh_height_coeff * math.hypot(eta_x, eta_y)
            parabola = radius2_over_4g * omega * omega if self.slosh_use_parabola else 0.0
            eta_dot_norm.append(eta_dot)
            energy.append(e)
            height_modal.append(modal)
            height_parabola.append(parabola)
            height_total.append(modal + parabola)
        return {
            "slosh_h_p95": percentile(height_total, 95.0),
            "slosh_h_max": max(height_total) if height_total else float("inf"),
            "slosh_h_modal_p95": percentile(height_modal, 95.0),
            "slosh_h_parabola_p95": percentile(height_parabola, 95.0),
            "slosh_eta_dot_rms": math.sqrt(sum(v * v for v in eta_dot_norm) / len(eta_dot_norm)) if eta_dot_norm else float("inf"),
            "slosh_energy_rms": math.sqrt(sum(v * v for v in energy) / len(energy)) if energy else float("inf"),
            "slosh_terminal_E": energy[-1] if energy else float("inf"),
        }

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
        width = info.width
        height = info.height
        data = grid.data
        for k, (x, y) in enumerate(points):
            ix = int(math.floor((x - ox) / res))
            iy = int(math.floor((y - oy) / res))
            if ix < 0 or iy < 0 or ix >= width or iy >= height:
                if self.unknown_is_obstacle:
                    return ("collision", k, -1)
                continue
            cost = data[iy * width + ix]
            if cost < 0:
                if self.unknown_is_obstacle:
                    return ("collision", k, int(cost))
                continue
            if cost >= self.collision_threshold:
                return ("collision", k, int(cost))
        return ("accepted", -1, -1)

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
            if name != "original":
                candidate = sanitize_points(candidate, self.min_segment_length)
            row = self.evaluate_candidate(index, name, candidate, base, base_metrics, base_length, msg.header.frame_id)
            rows.append((row, candidate))

        if self.slosh_score_enable:
            self.apply_slosh_scores(rows)

        for row, candidate in rows:
            if row["accepted"] and (best is None or row["score"] < best[0]["score"]):
                best = (row, candidate)

        if best is None:
            rospy.logwarn_throttle(
                1.0,
                "[anti_slosh_path_post_processor] no candidate passed gates; publishing original path",
            )
            best = rows[0]

        self.publish_outputs(msg, rows, best)

    def evaluate_candidate(self, index, name, candidate, base, base_metrics, base_length, path_frame):
        metrics = path_metrics(candidate, base)
        length_ratio = metrics["length_m"] / base_length
        end_error = endpoint_error(candidate, base)
        reject_reasons = []
        base_ay_p95, base_vmax = self.predicted_lateral_profile(base)
        ay_p95, predicted_vmax = self.predicted_lateral_profile(candidate)
        ay_ratio = safe_ratio(ay_p95, base_ay_p95)
        if len(candidate) < 3:
            reject_reasons.append(f"too_few_points:{len(candidate)}")
        if metrics["min_seg_m"] < self.min_segment_length:
            reject_reasons.append(f"min_seg:{metrics['min_seg_m']:.3f}<{self.min_segment_length:.3f}")
        if metrics["max_drift_m"] > self.max_drift:
            reject_reasons.append(f"drift:{metrics['max_drift_m']:.3f}>{self.max_drift:.3f}")
        if length_ratio > self.max_length_ratio:
            reject_reasons.append(f"length:{length_ratio:.3f}>{self.max_length_ratio:.3f}")
        if length_ratio < self.min_length_ratio:
            reject_reasons.append(f"short:{length_ratio:.3f}<{self.min_length_ratio:.3f}")
        kappa_ratio = safe_ratio(metrics["kappa_p95"], base_metrics["kappa_p95"])
        dkappa_ratio = safe_ratio(metrics["dkappa_p95"], base_metrics["dkappa_p95"])
        if self.candidate_levels.get(name, 0) > self.candidate_levels[self.max_candidate_level]:
            reject_reasons.append(f"level:{name}>{self.max_candidate_level}")
        if ay_ratio > self.ay_ratio_limit:
            reject_reasons.append(f"ay:{ay_ratio:.3f}>{self.ay_ratio_limit:.3f}")
        if end_error > self.max_endpoint_error:
            reject_reasons.append(f"endpoint:{end_error:.3f}>{self.max_endpoint_error:.3f}")
        if not direction_preserved(candidate, base):
            reject_reasons.append("direction")
        col_status, col_idx, col_cost = self.check_candidate_collision(
            candidate, path_frame, skip_check=(not self.enable_collision_check) or name == "original"
        )
        if col_status == "collision":
            reject_reasons.append(f"collision:idx={col_idx}:cost={col_cost}")
        elif col_status == "no_costmap":
            reject_reasons.append("no_costmap")
        elif col_status == "frame_mismatch":
            reject_reasons.append("frame_mismatch")
        accepted = not reject_reasons
        shortening_penalty = max(0.0, self.min_length_ratio - length_ratio)
        over_smooth_penalty = max(0.0, self.min_kappa_ratio - kappa_ratio)
        target_kappa_penalty = abs(kappa_ratio - self.target_kappa_ratio)
        score = (
            self.w_kappa * target_kappa_penalty
            + self.w_dkappa * dkappa_ratio
            + self.w_length * max(0.0, length_ratio - 1.0)
            + self.w_drift * metrics["max_drift_m"]
            + self.w_shortening * shortening_penalty
            + self.w_over_smooth * over_smooth_penalty
        )
        slosh_metrics = self.rollout_slosh_metrics(candidate) if self.slosh_score_enable else {
            "slosh_h_p95": 0.0,
            "slosh_h_max": 0.0,
            "slosh_h_modal_p95": 0.0,
            "slosh_h_parabola_p95": 0.0,
            "slosh_eta_dot_rms": 0.0,
            "slosh_energy_rms": 0.0,
            "slosh_terminal_E": 0.0,
        }
        return {
            "index": index,
            "name": name,
            "accepted": accepted,
            "score": score,
            "geometry_score": score,
            "slosh_score": score,
            "length_ratio": length_ratio,
            "kappa_ratio": kappa_ratio,
            "dkappa_ratio": dkappa_ratio,
            "predicted_ay_p95": ay_p95,
            "predicted_ay_ratio": ay_ratio,
            "predicted_vmax": predicted_vmax,
            "base_predicted_ay_p95": base_ay_p95,
            "base_predicted_vmax": base_vmax,
            "target_kappa_penalty": target_kappa_penalty,
            "endpoint_error_m": end_error,
            "collision_status": col_status,
            "collision_idx": col_idx,
            "collision_cost": col_cost,
            "reject_reason": "accepted" if accepted else "|".join(reject_reasons),
            **slosh_metrics,
            **metrics,
        }

    def apply_slosh_scores(self, rows):
        accepted = [row for row, _ in rows if row["accepted"]]
        if len(accepted) < 2:
            return
        max_h = max(max(0.0, row["slosh_h_p95"]) for row in accepted)
        max_energy = max(max(0.0, row["slosh_energy_rms"]) for row in accepted)
        max_eta_dot = max(max(0.0, row["slosh_eta_dot_rms"]) for row in accepted)
        max_terminal = max(max(0.0, row["slosh_terminal_E"]) for row in accepted)
        max_kappa = max(max(0.0, row["kappa_p95"]) for row in accepted)
        max_dkappa = max(max(0.0, row["dkappa_p95"]) for row in accepted)
        max_drift = max(max(0.0, row["max_drift_m"]) for row in accepted)
        for row, _ in rows:
            if not row["accepted"]:
                continue
            norm = lambda value, ref: value / max(1e-6, ref)
            length_penalty = max(0.0, row["length_ratio"] - 1.0)
            slosh_score = (
                self.w_slosh_h * norm(row["slosh_h_p95"], max_h)
                + self.w_slosh_energy * norm(row["slosh_energy_rms"], max_energy)
                + self.w_slosh_eta_dot * norm(row["slosh_eta_dot_rms"], max_eta_dot)
                + self.w_slosh_terminal * norm(row["slosh_terminal_E"], max_terminal)
                + self.w_slosh_kappa * norm(row["kappa_p95"], max_kappa)
                + self.w_slosh_dkappa * norm(row["dkappa_p95"], max_dkappa)
                + self.w_slosh_ay * max(0.0, row["predicted_ay_ratio"])
                + self.w_slosh_length * length_penalty
                + self.w_slosh_drift * norm(row["max_drift_m"], max_drift)
            )
            row["slosh_score"] = slosh_score
            row["score"] = slosh_score

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

        if best_row["name"] == "original":
            out_msg = raw_msg
            out_msg.header.stamp = header.stamp
            if not out_msg.header.frame_id:
                out_msg.header.frame_id = header.frame_id
            self.path_pub.publish(out_msg)
        else:
            self.path_pub.publish(path_to_msg(best_points, header))
        self.publish_metrics(best_row, rows)
        self.publish_candidate_report(rows)
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

    def publish_candidate_report(self, rows):
        parts = []
        for row, _ in rows:
            if row["collision_status"] == "collision":
                col_str = "col=hit:idx={}:cost={}".format(row["collision_idx"], row["collision_cost"])
            elif row["collision_status"] == "no_costmap":
                col_str = "col=no_costmap"
            elif row["collision_status"] == "frame_mismatch":
                col_str = "col=frame_mismatch"
            else:
                col_str = "col=ok"
            parts.append(
                "{name}:accepted={accepted},reason={reason},score={score:.3f},"
                "len={length:.3f},drift={drift:.3f},end={end:.3f},"
                "k95={k95:.3f},dk95={dk95:.3f},kr={kr:.3f},dkr={dkr:.3f},"
                "ayr={ayr:.3f},vmaxp={vmaxp:.3f},gscore={gscore:.3f},sscore={sscore:.3f},"
                "sH={sH:.3g},sHm={sHm:.3g},sHp={sHp:.3g},sE={sE:.3g},sEdot={sEdot:.3g},{col}".format(
                    name=row["name"],
                    accepted=int(row["accepted"]),
                    reason=row["reject_reason"],
                    score=row["score"],
                    length=row["length_ratio"],
                    drift=row["max_drift_m"],
                    end=row["endpoint_error_m"],
                    k95=row["kappa_p95"],
                    dk95=row["dkappa_p95"],
                    kr=row["kappa_ratio"],
                    dkr=row["dkappa_ratio"],
                    ayr=row["predicted_ay_ratio"],
                    vmaxp=row["predicted_vmax"],
                    gscore=row["geometry_score"],
                    sscore=row["slosh_score"],
                    sE=row["slosh_energy_rms"],
                    sEdot=row["slosh_eta_dot_rms"],
                    sH=row["slosh_h_p95"],
                    sHm=row["slosh_h_modal_p95"],
                    sHp=row["slosh_h_parabola_p95"],
                    col=col_str,
                )
            )
        report = "; ".join(parts)
        self.candidate_report_pub.publish(String(data=report))
        rospy.loginfo_throttle(1.0, "[anti_slosh_path_post_processor] candidates: %s", report)

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
