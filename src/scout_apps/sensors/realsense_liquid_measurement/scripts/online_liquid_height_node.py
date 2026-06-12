#!/usr/bin/env python3
"""在线液面高度监控节点。

复用离线 red_liquid_infer_from_bag 的逐帧检测核(detect_red_liquid / resolve_geometry),
保证在线监控数值与离线真值口径完全一致。

每天一次标定(ROI + 三标尺 + HSV)写入 calibration.yaml 后:
  订阅 RGB 图像 -> 每帧 detect_red_liquid -> 发布 max-LCR / 三列 / 中位数液面高度(mm)。

发布(话题名均可通过私有参数覆盖, 默认如下):
  /liquid/height          std_msgs/Float32          max(left,center,right) mm (主监控量, 无检测=NaN)
  /liquid/height_lcr      std_msgs/Float32MultiArray [left, center, right] mm (NaN=该列无检测)
  /liquid/height_median   std_msgs/Float32          三列中位数 mm (质量交叉检查)
  /liquid/debug_image     sensor_msgs/Image         可选 overlay(~publish_debug:=true)

参数(私有):
  ~calibration            (必填) 三标尺标定 yaml(与离线同一份)
  ~image_topic            默认 /camera/color/image_raw
  ~height_topic           默认 /liquid/height
  ~height_lcr_topic       默认 /liquid/height_lcr
  ~height_median_topic    默认 /liquid/height_median
  ~debug_image_topic      默认 /liquid/debug_image
  ~process_every          默认 1 (每 N 帧处理一次, 降 CPU)
  ~publish_debug          默认 false
  HSV/检测阈值   优先 calibration 的 hsv: 段, ~<key> 参数可覆盖, 否则用与离线一致的默认值。

监控: rqt_plot /liquid/height   或   rostopic echo /liquid/height
"""

import os
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from red_liquid_infer_from_bag import (  # noqa: E402
    detect_red_liquid,
    h_mm_final,
    load_calibration,
    render_debug,
    resolve_geometry,
)

# 必须与 red_liquid_infer_from_bag.py 的 parse_args 默认值一致(在线=离线)。
DETECT_DEFAULTS = {
    "hue1_low": 0, "hue1_high": 12, "hue2_low": 168, "hue2_high": 179,
    "sat_min": 70, "val_min": 35, "morph_kernel": 5,
    "top_boundary_quantile": 0.2, "min_valid_column_fraction": 0.15,
    "center_col_fraction": 0.6, "min_component_area": 30, "bottom_touch_rows": 15,
}


def build_detect_args(calib):
    """检测阈值: 默认 <- calibration 的 hsv: 段 <- ROS 私有参数(逐项覆盖)。"""
    vals = dict(DETECT_DEFAULTS)
    hsv = calib.get("hsv", {}) if isinstance(calib, dict) else {}
    for k in DETECT_DEFAULTS:
        if isinstance(hsv, dict) and k in hsv and hsv[k] is not None:
            vals[k] = hsv[k]
    for k in DETECT_DEFAULTS:
        p = rospy.get_param("~%s" % k, None)
        if p is not None:
            vals[k] = type(DETECT_DEFAULTS[k])(p)
    return SimpleNamespace(**vals)


class OnlineLiquidHeight:
    def __init__(self):
        calib_path = rospy.get_param("~calibration")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.height_topic = rospy.get_param("~height_topic", "/liquid/height")
        self.lcr_topic = rospy.get_param("~height_lcr_topic", "/liquid/height_lcr")
        self.median_topic = rospy.get_param("~height_median_topic", "/liquid/height_median")
        self.debug_topic = rospy.get_param("~debug_image_topic", "/liquid/debug_image")
        self.every = max(1, int(rospy.get_param("~process_every", 1)))
        self.publish_debug = bool(rospy.get_param("~publish_debug", False))

        calib = load_calibration(Path(calib_path).expanduser())
        self.args = build_detect_args(calib)
        self.geom, self.rulers = resolve_geometry(self.args, calib)
        if self.rulers is None:
            rospy.logwarn("[online_liquid_height] calibration 无 rulers 段, 退化为 legacy 单列; "
                          "max-LCR 不可用, 请用三标尺标定。")

        self.bridge = CvBridge()
        self.frame_i = 0
        self.height_bias_mm = float(rospy.get_param("~height_bias_mm",
            calib.get("height_bias_mm", 0.0) if isinstance(calib, dict) else 0.0))

        # running zero estimation from first frames
        self._zero_window = deque(maxlen=30)
        self._zero_samples = 0
        self._zero_value = 0.0
        self._zero_locked = False

        self.height_pub = rospy.Publisher(self.height_topic, Float32, queue_size=5)
        self.lcr_pub = rospy.Publisher(self.lcr_topic, Float32MultiArray, queue_size=5)
        self.median_pub = rospy.Publisher(self.median_topic, Float32, queue_size=5)
        self.debug_pub = (rospy.Publisher(self.debug_topic, Image, queue_size=2)
                          if self.publish_debug else None)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.on_image,
                                    queue_size=1, buff_size=2 ** 24)
        self.reset_srv = rospy.Service("~reset_zero", Empty, self.on_reset)

        rospy.loginfo("[online_liquid_height] calib=%s image=%s every=%d debug=%s outputs=(%s,%s,%s,%s)",
                      calib_path, self.image_topic, self.every, self.publish_debug,
                      self.height_topic, self.lcr_topic, self.median_topic,
                      self.debug_topic if self.publish_debug else "debug-disabled")
        rospy.loginfo("[online_liquid_height] ROI=(%d,%d,%d,%d) tube=[%d,%d] rulers=%s "
                      "HSV=h1[%d,%d] h2[%d,%d] s>=%d v>=%d bias=%.1fmm",
                      self.geom["roi_x"], self.geom["roi_y"], self.geom["roi_w"], self.geom["roi_h"],
                      self.geom["tube_left"], self.geom["tube_right"],
                      "3-ruler" if self.rulers else "legacy",
                      self.args.hue1_low, self.args.hue1_high, self.args.hue2_low, self.args.hue2_high,
                      self.args.sat_min, self.args.val_min, self.height_bias_mm)

    def on_image(self, msg):
        self.frame_i += 1
        if self.frame_i % self.every != 0:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(2.0, "[online_liquid_height] 图像转换失败: %s", exc)
            return
        try:
            y_tops, h_mms, confs, clipped, mask = detect_red_liquid(img, self.geom, self.rulers, self.args)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(2.0, "[online_liquid_height] 检测异常: %s", exc)
            return

        valid = [h for h in h_mms if h is not None]
        max_lcr = float(max(valid)) if valid else float("nan")
        median = h_mm_final(h_mms)

        # running zero estimation
        if not self._zero_locked and self._zero_samples < 30 and not (max_lcr != max_lcr):
            self._zero_window.append(max_lcr)
            self._zero_samples += 1
            if self._zero_samples == 30:
                self._zero_value = float(np.median(list(self._zero_window)))
                self._zero_locked = True
                rospy.loginfo("[online_liquid_height] zero locked: h0=%.2f mm (median of first 30 frames)", self._zero_value)

        h0 = self._zero_value if self._zero_locked else 0.0
        max_lcr_corrected = max_lcr - h0 - self.height_bias_mm

        self.height_pub.publish(Float32(data=max_lcr_corrected))
        self.median_pub.publish(Float32(data=float(median - h0 - self.height_bias_mm) if median is not None else float("nan")))
        lcr = Float32MultiArray()
        lcr.data = [float(h - h0 - self.height_bias_mm) if h is not None else float("nan") for h in h_mms]
        self.lcr_pub.publish(lcr)

        if self.debug_pub is not None:
            try:
                canvas = render_debug(img, self.geom, self.rulers, y_tops, h_mms, confs, clipped, mask, self.args)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0, "[online_liquid_height] debug 渲染失败: %s", exc)

        cols = ",".join("NA" if h is None else "%.1f" % h for h in h_mms)
        rospy.loginfo_throttle(1.0, "[online_liquid_height] max-LCR=%.2f mm  L/C/R=[%s] mm  h0=%.1f bias=%.1f",
                               max_lcr_corrected, cols, h0, self.height_bias_mm)

    def on_reset(self, req):
        self._zero_window.clear()
        self._zero_samples = 0
        self._zero_value = 0.0
        self._zero_locked = False
        rospy.loginfo("[online_liquid_height] zero reset — re-estimating h0 from next 30 frames")
        return EmptyResponse()


def main():
    rospy.init_node("online_liquid_height")
    OnlineLiquidHeight()
    rospy.spin()


if __name__ == "__main__":
    main()
