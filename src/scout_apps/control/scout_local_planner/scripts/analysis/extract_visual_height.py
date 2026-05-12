#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Side-view ArUco + Hough liquid height extractor for RA-L visual GT (D5).

Subscribes to a colour image stream from a side-mounted RealSense D435i,
locates two ArUco markers on the container (top fiducial + scale fiducial),
crops a ROI just below the top marker, and runs Canny+Hough to find the
liquid line. Publishes /slosh/h_visual (mm) and /slosh/h_visual_quality
(0-1 confidence based on line strength + scale validity).

Status: SKELETON. Container ArUco placement, ROI mm offset, and Hough
parameters need real-camera calibration before publishing meaningful values.
"""

import math
import os
import sys
from collections import deque

try:
    import cv2
except ImportError:
    sys.stderr.write("error: opencv-python not installed; run `pip install opencv-contrib-python`\n")
    sys.exit(2)

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class VisualHeightExtractor:
    def __init__(self):
        config_path = rospy.get_param(
            "~config", "src/scout_apps/control/scout_local_planner/config/visual_height.yaml"
        )
        cfg = load_yaml(config_path).get("visual", {})
        topics = cfg.get("topics", {})
        self.bridge = CvBridge()
        self.color_topic = str(topics.get("color", "/camera/color/image_raw"))
        self.height_topic = str(topics.get("output_height", "/slosh/h_visual"))
        self.quality_topic = str(topics.get("output_quality", "/slosh/h_visual_quality"))

        aruco_cfg = cfg.get("aruco", {})
        self.aruco_dict = self._aruco_dict(aruco_cfg.get("dictionary", "DICT_4X4_50"))
        self.aruco_params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") else cv2.aruco.DetectorParameters()
        self.marker_size_mm = float(aruco_cfg.get("marker_size_mm", 30))
        self.top_id = int(aruco_cfg.get("top_marker_id", 0))
        self.scale_id = int(aruco_cfg.get("scale_marker_id", 1))

        roi_cfg = cfg.get("roi", {})
        self.roi_margin = int(roi_cfg.get("margin_px", 8))
        self.roi_height_mm = float(roi_cfg.get("height_below_top_marker_mm", 80))

        edge_cfg = cfg.get("liquid_edge", {})
        self.blur_kernel = int(edge_cfg.get("blur_kernel", 5))
        self.canny_low = int(edge_cfg.get("canny_low", 40))
        self.canny_high = int(edge_cfg.get("canny_high", 140))
        self.hough_threshold = int(edge_cfg.get("hough_threshold", 25))
        self.hough_min_len = int(edge_cfg.get("hough_min_line_length_px", 30))
        self.hough_max_gap = int(edge_cfg.get("hough_max_line_gap_px", 5))
        self.angle_tol = math.radians(float(edge_cfg.get("angle_tolerance_deg", 8)))

        pub_cfg = cfg.get("publishing", {})
        self.drop_no_aruco = bool(pub_cfg.get("drop_if_no_aruco", True))
        self.drop_no_line = bool(pub_cfg.get("drop_if_no_line", True))

        baseline_cfg = cfg.get("static_baseline", {})
        self.baseline_record_sec = float(baseline_cfg.get("record_first_n_seconds", 3.0))
        self.baseline_use = bool(baseline_cfg.get("use_baseline_as_zero", True))
        self.baseline_buffer = deque()
        self.baseline_zero_mm = None
        self.start_time = None

        self.h_pub = rospy.Publisher(self.height_topic, Float32, queue_size=10)
        self.q_pub = rospy.Publisher(self.quality_topic, Float32, queue_size=10)
        self.sub = rospy.Subscriber(self.color_topic, Image, self.image_cb, queue_size=1)
        rospy.loginfo("[extract_visual_height] listening on %s, publishing %s",
                      self.color_topic, self.height_topic)

    @staticmethod
    def _aruco_dict(name):
        attr = getattr(cv2.aruco, name, None)
        if attr is None:
            raise SystemExit(f"unknown aruco dictionary: {name}")
        if hasattr(cv2.aruco, "Dictionary_get"):
            return cv2.aruco.Dictionary_get(attr)
        return cv2.aruco.getPredefinedDictionary(attr)

    def image_cb(self, msg):
        if self.start_time is None:
            self.start_time = rospy.get_time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[extract_visual_height] bridge error: %s", exc)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, "detectMarkers"):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        else:
            corners, ids, _ = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params).detectMarkers(gray)
        if ids is None or len(ids) < 2:
            if not self.drop_no_aruco:
                self._publish(float("nan"), 0.0)
            return
        ids_flat = ids.flatten().tolist()
        if self.top_id not in ids_flat or self.scale_id not in ids_flat:
            if not self.drop_no_aruco:
                self._publish(float("nan"), 0.0)
            return
        top_corners = corners[ids_flat.index(self.top_id)][0]
        scale_corners = corners[ids_flat.index(self.scale_id)][0]
        px_per_mm = self._pixels_per_mm(top_corners, scale_corners)
        if px_per_mm is None or px_per_mm <= 0:
            self._publish(float("nan"), 0.0)
            return
        roi = self._extract_roi(gray, top_corners, scale_corners, px_per_mm)
        if roi is None:
            self._publish(float("nan"), 0.0)
            return
        height_mm, quality = self._detect_liquid_line(roi, top_corners, px_per_mm)
        if math.isnan(height_mm):
            if not self.drop_no_line:
                self._publish(float("nan"), 0.0)
            return
        if self.baseline_use and self.baseline_zero_mm is None:
            self.baseline_buffer.append(height_mm)
            if rospy.get_time() - self.start_time >= self.baseline_record_sec:
                self.baseline_zero_mm = float(np.median(self.baseline_buffer)) if self.baseline_buffer else 0.0
                rospy.loginfo("[extract_visual_height] baseline=%.2f mm (n=%d)",
                              self.baseline_zero_mm, len(self.baseline_buffer))
        offset_mm = height_mm - (self.baseline_zero_mm or 0.0) if self.baseline_use else height_mm
        self._publish(offset_mm, quality)

    def _pixels_per_mm(self, top_corners, scale_corners):
        edge_lengths = [
            np.linalg.norm(top_corners[(i + 1) % 4] - top_corners[i])
            for i in range(4)
        ]
        edge_lengths.extend(
            np.linalg.norm(scale_corners[(i + 1) % 4] - scale_corners[i]) for i in range(4)
        )
        avg_edge_px = float(np.mean(edge_lengths))
        if avg_edge_px <= 0:
            return None
        return avg_edge_px / self.marker_size_mm

    def _extract_roi(self, gray, top_corners, scale_corners, px_per_mm):
        bottom_y = float(max(top_corners[:, 1]))
        roi_top = int(bottom_y + self.roi_margin)
        roi_bottom = int(bottom_y + self.roi_margin + self.roi_height_mm * px_per_mm)
        x_min = int(min(np.concatenate([top_corners[:, 0], scale_corners[:, 0]]))) - self.roi_margin
        x_max = int(max(np.concatenate([top_corners[:, 0], scale_corners[:, 0]]))) + self.roi_margin
        x_min = max(0, x_min)
        x_max = min(gray.shape[1], x_max)
        roi_top = max(0, roi_top)
        roi_bottom = min(gray.shape[0], roi_bottom)
        if roi_bottom - roi_top < 10 or x_max - x_min < 10:
            return None
        return {
            "img": gray[roi_top:roi_bottom, x_min:x_max],
            "y_offset": roi_top,
            "x_offset": x_min,
            "top_baseline": bottom_y,
        }

    def _detect_liquid_line(self, roi, top_corners, px_per_mm):
        img = cv2.GaussianBlur(roi["img"], (self.blur_kernel, self.blur_kernel), 0)
        edges = cv2.Canny(img, self.canny_low, self.canny_high)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, self.hough_threshold,
            minLineLength=self.hough_min_len, maxLineGap=self.hough_max_gap,
        )
        if lines is None:
            return float("nan"), 0.0
        best = None
        best_len = 0.0
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = line
            angle = math.atan2(y2 - y1, x2 - x1)
            if abs(angle) > self.angle_tol and abs(abs(angle) - math.pi) > self.angle_tol:
                continue
            length = math.hypot(x2 - x1, y2 - y1)
            if length > best_len:
                best_len = length
                best = (x1, y1, x2, y2)
        if best is None:
            return float("nan"), 0.0
        x1, y1, x2, y2 = best
        y_mid_roi = 0.5 * (y1 + y2)
        liquid_y_image = roi["y_offset"] + y_mid_roi
        height_px = liquid_y_image - roi["top_baseline"]
        height_mm = height_px / px_per_mm
        quality = min(1.0, best_len / max(roi["img"].shape[1], 1))
        return float(height_mm), float(quality)

    def _publish(self, height_mm, quality):
        self.h_pub.publish(Float32(data=float(height_mm)))
        self.q_pub.publish(Float32(data=float(quality)))


def main():
    rospy.init_node("extract_visual_height")
    node = VisualHeightExtractor()
    rospy.spin()


if __name__ == "__main__":
    main()
