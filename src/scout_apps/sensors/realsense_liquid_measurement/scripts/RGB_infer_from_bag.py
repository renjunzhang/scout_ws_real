#!/usr/bin/env python3
"""
RGB-based liquid-height measurement from a ROS bag.

椭圆液面问题
------------
单侧摄像头以斜角观察圆柱试管时，液面（水平圆面）在图像中投影为椭圆：
- 椭圆上沿 = 液面远侧边缘（穿过管壁可见）
- 椭圆下沿 = 液面近侧边缘（靠摄像头一侧）
- 椭圆中点 y = 真实液位高度的像素坐标

因此检测算法默认采用 dual_edge 模式：
每列同时找两个负梯度峰（上沿+下沿），取平均值作为该列的液面估计。

另外，管壁两侧列受弯月面（meniscus）影响较大，通过 center_col_fraction
参数只使用管径中心部分的列，进一步减小误差。

算法流程
--------
1. 读取标定 YAML（由 RGB_calibrate.py 生成）。
2. 对每一帧：
   a. 裁出 ROI。
   b. 若 rotation_deg != 0 则对 ROI 进行仿射旋转矫正。
   c. 在管内中心区域逐列分析竖向梯度（Sobel Y）。
   d. dual_edge 模式：每列找 ≥2 个负梯度峰，取第一个（远侧）和最后一个（近侧）的均值；
      single_edge 模式：只取最大负梯度峰（适合摄像头水平正对液面的情况）。
   e. IQR 过滤异常列，取中位数作为液面 y_px。
   f. 分段线性插值 y_px → h_mm。
3. 输出 CSV + 液面曲线图；可选：保存每帧 debug 图。

Usage example
-------------
python RGB_infer_from_bag.py \\
    --bag /data/a/bags/Q0_test1.bag \\
    --calibration /data/a/bags/Q0_test1_rgb_calib.yaml \\
    --out-dir /data/a/bags/Q0_test1_rgb_results \\
    --topic /camera/color/image_raw \\
    --time-offset 5.0 \\
    --debug-every 30
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

try:
    import rosbag
    from cv_bridge import CvBridge, CvBridgeError
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402
    _HAS_MPL = True
except ImportError:
    plt = None  # type: ignore[assignment]
    _HAS_MPL = False


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="RGB pixel-based liquid height measurement from a rosbag."
    )
    p.add_argument("--bag", required=True, help="Path to the rosbag file.")
    p.add_argument(
        "--calibration", required=True,
        help="YAML calibration file produced by RGB_calibrate.py.",
    )
    p.add_argument(
        "--topic", default="/camera/color/image_raw",
        help="RGB image topic. Default: /camera/color/image_raw.",
    )
    p.add_argument(
        "--out-dir", default="",
        help="Output directory. Default: <bag_stem>_rgb_results next to the bag.",
    )
    p.add_argument(
        "--time-offset", type=float, default=0.0,
        help="Skip seconds from bag start before processing. Default 0.",
    )
    p.add_argument(
        "--max-frames", type=int, default=0,
        help="Stop after N frames (0 = process all). Default 0.",
    )
    p.add_argument(
        "--every", type=int, default=1,
        help="Process every N-th frame. Default 1.",
    )
    p.add_argument(
        "--debug-every", type=int, default=0,
        help="Save a debug image every N processed frames (0 = disabled). Default 0.",
    )
    p.add_argument(
        "--gradient-min-strength", type=float, default=None,
        help="Override calibration gradient_min_strength.",
    )
    p.add_argument(
        "--blur-kernel", type=int, default=None,
        help="Override calibration blur_kernel (must be odd).",
    )
    p.add_argument(
        "--search-top-fraction", type=float, default=None,
        help="Override calibration search_top_fraction.",
    )
    p.add_argument(
        "--search-bottom-fraction", type=float, default=None,
        help="Override calibration search_bottom_fraction.",
    )
    p.add_argument(
        "--detection-mode",
        choices=["dual_edge", "single_edge", "near_edge"],
        default=None,
        help=(
            "dual_edge：每列找远侧+近侧两个边缘，取均值。 "
            "single_edge：只取最大负梯度峰。 "
            "near_edge：只取最下方（近侧）负梯度峰，再减去标定的椭圆半高修正到中点；"
            "配合 --near-edge-only 标定使用（YAML 会自动选此模式）。"
        ),
    )
    p.add_argument(
        "--max-ellipse-height-px", type=float, default=None,
        help=(
            "dual_edge 模式下，两个边缘最大允许间距（像素）。超出则退化为单边缘。 "
            "默认值来自标定 YAML，初始值 40。"
        ),
    )
    p.add_argument(
        "--center-col-fraction", type=float, default=None,
        help=(
            "只使用管径中心 N 比例的列（0~1），避开管壁两侧弯月面区域。 "
            "默认 0.6（使用中心 60%%）。"
        ),
    )
    p.add_argument(
        "--calib-search-margin-px", type=int, default=None,
        help=(
            "near_edge 模式专用：在标定参考点 y 范围上下各扩展 N 像素作为搜索带。"
            "默认 20。调大可应对液面超出标定范围的情况。"
        ),
    )
    p.add_argument(
        "--tracker-window-px", type=float, default=None,
        help="跟踪窗口半宽（像素）。near_edge 模式下每帧只在预测位置 ±N px 内搜索。默认 15。",
    )
    p.add_argument(
        "--tracker-max-speed-px", type=float, default=None,
        help="允许的最大帧间液位变化量（像素/帧），超出部分被截断。默认 10。",
    )
    p.add_argument(
        "--tracker-reacquire-conf", type=float, default=None,
        help="置信度低于此值时触发重捕获（扩大搜索窗）。默认 0.3。",
    )
    p.add_argument(
        "--no-tracker", action="store_true", default=False,
        help="禁用时序跟踪，回退到全标定带静态搜索（调试用）。",
    )
    p.add_argument(
        "--encoding", default="bgr8",
        help="cv_bridge decoding encoding. Default bgr8.",
    )
    p.add_argument(
        "--list-topics", action="store_true",
        help="List image topics in the bag and exit.",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Calibration loading
# ---------------------------------------------------------------------------

def load_calibration(yaml_path: Path) -> Dict:
    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    ref_pts = data["calibration"]["reference_points"]
    for i in range(len(ref_pts) - 1):
        if ref_pts[i]["y_px_roi"] <= ref_pts[i + 1]["y_px_roi"]:
            print(
                f"[WARN] reference_points[{i}].y_px_roi={ref_pts[i]['y_px_roi']:.1f} is not "
                f"greater than reference_points[{i+1}].y_px_roi={ref_pts[i+1]['y_px_roi']:.1f}. "
                "Check calibration YAML ordering.",
                file=sys.stderr,
            )
    return data


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def mm_to_px(h_mm: float, ref_pts: List[Dict]) -> Optional[float]:
    """px_to_mm 的反向查询：给定高度 mm，返回对应的 y_px_roi。"""
    if not ref_pts:
        return None
    pts = sorted(ref_pts, key=lambda p: p["y_px_roi"], reverse=True)
    ys = [p["y_px_roi"] for p in pts]  # decreasing
    hs = [p["h_mm"] for p in pts]      # increasing
    if h_mm <= hs[0]:
        return ys[0]
    if h_mm >= hs[-1]:
        return ys[-1]
    for i in range(len(hs) - 1):
        h_lo, h_hi = hs[i], hs[i + 1]
        if h_lo <= h_mm <= h_hi:
            alpha = (h_mm - h_lo) / max(1e-6, h_hi - h_lo)
            return ys[i] + alpha * (ys[i + 1] - ys[i])
    return None


def px_to_mm(y_px_roi: float, ref_pts: List[Dict]) -> Optional[float]:
    """
    Piecewise-linear interpolation: y_px_roi (ROI pixels) → h_mm.

    ref_pts is sorted by h_mm ascending, which means y_px_roi is descending
    (larger y = lower in image = lower liquid level).
    """
    if not ref_pts:
        return None

    # Sort by y_px_roi descending (bottom of image first = lowest mm)
    pts = sorted(ref_pts, key=lambda p: p["y_px_roi"], reverse=True)
    ys = [p["y_px_roi"] for p in pts]   # decreasing
    hs = [p["h_mm"] for p in pts]       # increasing

    if y_px_roi >= ys[0]:
        return hs[0]
    if y_px_roi <= ys[-1]:
        return hs[-1]

    for i in range(len(ys) - 1):
        y_lo, y_hi = ys[i + 1], ys[i]     # y_hi > y_lo (both in px)
        h_lo, h_hi = hs[i], hs[i + 1]
        if y_lo <= y_px_roi <= y_hi:
            alpha = (y_hi - y_px_roi) / max(1e-6, y_hi - y_lo)
            return h_lo + alpha * (h_hi - h_lo)

    return hs[-1]


def _find_negative_peaks(profile: np.ndarray, threshold: float) -> List[int]:
    """
    返回 profile 中所有局部极小值的行索引，只保留值 < -threshold 的峰。
    使用简单的 3 点极小值条件，对高斯平滑后的梯度剖面足够稳定。
    """
    n = len(profile)
    peaks: List[int] = []
    for i in range(1, n - 1):
        if profile[i] < -threshold and profile[i] <= profile[i - 1] and profile[i] <= profile[i + 1]:
            peaks.append(i)
    # 如果首尾也满足条件，单独处理
    if n > 0 and profile[0] < -threshold and (n < 2 or profile[0] <= profile[1]):
        peaks.insert(0, 0)
    if n > 1 and profile[-1] < -threshold and profile[-1] <= profile[-2]:
        peaks.append(n - 1)
    return peaks


def _get_reference_y_for_height(calib: Dict, target_h_mm: float) -> Optional[float]:
    for rp in calib["calibration"]["reference_points"]:
        if abs(float(rp["h_mm"]) - target_h_mm) < 1e-6:
            return float(rp["y_px_roi"])
    return None


def _get_near_edge_search_band(calib: Dict, det_params: Dict, h_img: int) -> Tuple[int, int]:
    """
    near_edge 模式下的统一纵向搜索带。

    优先使用标定中的 0~6 mm 区间，避免顶部刻度线与底部黑边进入候选区。
    如果缺少 6 mm 标定点，再回退到完整参考点范围。
    """
    ref_ys = [float(rp["y_px_roi"]) for rp in calib["calibration"]["reference_points"]]
    max_ellipse_h = float(det_params.get("max_ellipse_height_px", 40.0))
    near_corr = float(det_params.get("near_edge_correction_px", max_ellipse_h / 2.0))

    # 允许通过参数覆盖默认的 6 mm 上界，但默认仍按当前实验口径走 0~6 mm。
    upper_h_mm = float(det_params.get("near_edge_upper_bound_mm", 6.0))
    upper_mid_y = _get_reference_y_for_height(calib, upper_h_mm)
    zero_mid_y = _get_reference_y_for_height(calib, 0.0)

    if zero_mid_y is None:
        zero_mid_y = max(ref_ys)

    if upper_mid_y is None:
        upper_mid_y = min(ref_ys)

    top_margin = int(det_params.get("near_edge_top_margin_px", 10))
    bottom_margin = int(det_params.get("near_edge_bottom_margin_px", 8))

    y_top = max(0, int(round(upper_mid_y)) - top_margin)
    y_bot = min(h_img, int(round(zero_mid_y + near_corr + bottom_margin)))

    # 退化保护：窗口太窄则回退到完整参考点范围。
    if y_bot - y_top < 8:
        full_margin = int(det_params.get("calib_search_margin_px", 20))
        y_top = max(0, int(min(ref_ys)) - full_margin)
        y_bot = min(h_img, int(max(ref_ys) + max_ellipse_h))

    return y_top, y_bot


# ---------------------------------------------------------------------------
# Temporal tracker
# ---------------------------------------------------------------------------

class LiquidTracker:
    """
    带时序先验的液面跟踪器（near_edge 模式专用）。
    维护前一帧液位 prev_y 和速度 prev_v（ROI 坐标，px/frame）。
    每帧预测 y_pred，约束搜索窗口，避免被试管底部等固定结构边缘吸附。
    """
    def __init__(self, det_params: Dict, calib: Dict):
        self._prev_y: Optional[float] = None
        self._prev_v: float = 0.0
        self.window_px = float(det_params.get("tracker_window_px", 15.0))
        self.max_speed_px = float(det_params.get("tracker_max_speed_px", 10.0))
        self.reacquire_conf_thresh = float(det_params.get("tracker_reacquire_conf", 0.3))

        # h_roi 在 search_window 时可用，这里先按标定 ROI 高度求统一搜索带。
        self._y_full_top, self._y_full_bot = _get_near_edge_search_band(
            calib, det_params, int(calib["roi"]["h"])
        )

    @property
    def initialized(self) -> bool:
        return self._prev_y is not None

    def predict(self) -> Optional[float]:
        if self._prev_y is None:
            return None
        return self._prev_y + self._prev_v

    def search_window(self, h_roi: int, reacquire: bool = False) -> Tuple[int, int]:
        """返回当前帧的搜索窗口 (y_top, y_bot)，ROI 坐标。"""
        y_bot_limit = min(h_roi, self._y_full_bot)
        y_pred = self.predict()
        if y_pred is None or reacquire:
            return self._y_full_top, y_bot_limit
        y_top = max(self._y_full_top, int(y_pred - self.window_px))
        y_bot = min(y_bot_limit, int(y_pred + self.window_px))
        if y_bot - y_top < 8:  # 退化保护：窗口太窄则回退到全带
            return self._y_full_top, y_bot_limit
        return y_top, y_bot

    def update(self, y_meas: Optional[float], reacquire: bool) -> None:
        if y_meas is None:
            self._prev_v *= 0.5  # 未检到时衰减速度
            return
        if self._prev_y is None:
            self._prev_y = y_meas
            self._prev_v = 0.0
            return
        raw_v = y_meas - self._prev_y
        v_clamped = float(np.clip(raw_v, -self.max_speed_px, self.max_speed_px))
        alpha = 0.15 if reacquire else 0.35  # 重捕获时速度更新更保守
        self._prev_v = (1.0 - alpha) * self._prev_v + alpha * v_clamped
        self._prev_y = y_meas


def detect_meniscus(
    roi_bgr: np.ndarray,
    calib: Dict,
    det_params: Dict,
    y_window: Optional[Tuple[int, int]] = None,  # (y_top, y_bot) 覆盖，None 则用内部逻辑
    y_pred: Optional[float] = None,              # 跟踪预测位置（ROI 坐标），用于近邻峰评分
) -> Tuple[Optional[float], float]:
    """
    返回 (y_px_roi, confidence)，y_px_roi 为 ROI 坐标系下的液面 y 坐标。

    检测模式
    --------
    dual_edge（默认）：
        对每列找两个负梯度峰——椭圆上沿（液面远侧）和椭圆下沿（液面近侧）。
        取两峰均值 = 椭圆中点 = 真实液位。
        要求两峰间距 ≤ max_ellipse_height_px；否则退化为单峰。
        同时只使用管径中心 center_col_fraction 比例的列，避开弯月面区域。

    single_edge：
        只取最大负梯度峰（摄像头正对液面、无斜视时使用）。
    """
    h_img, w_img = roi_bgr.shape[:2]
    rotation_deg = float(calib.get("rotation_deg", 0.0))
    working = roi_bgr.copy()

    # 试管倾斜矫正
    if abs(rotation_deg) > 0.1:
        cx, cy = (w_img - 1) * 0.5, (h_img - 1) * 0.5
        M = cv2.getRotationMatrix2D((cx, cy), -rotation_deg, 1.0)
        working = cv2.warpAffine(working, M, (w_img, h_img),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_REPLICATE)

    x_left = calib["tube_inner"]["x_left"]
    x_right = calib["tube_inner"]["x_right"]
    tube_width = x_right - x_left

    # 中心列范围：只用管径中心 center_col_fraction 的列，排除两侧弯月面
    center_frac = float(det_params.get("center_col_fraction", 0.6))
    center_frac = max(0.1, min(1.0, center_frac))
    half_excl = (1.0 - center_frac) / 2.0
    x_margin = int(det_params.get("x_margin_px", 3))
    x1 = max(0, x_left + x_margin + int(half_excl * tube_width))
    x2 = min(w_img, x_right - x_margin - int(half_excl * tube_width))
    if x2 - x1 < 4:
        # 回退：放弃 center 限制，只保留 x_margin
        x1 = max(0, x_left + x_margin)
        x2 = min(w_img, x_right - x_margin)
    if x2 - x1 < 4:
        return None, 0.0

    mode = str(det_params.get("detection_mode", "dual_edge"))
    max_ellipse_h = float(det_params.get("max_ellipse_height_px", 40.0))
    near_edge_correction = float(det_params.get("near_edge_correction_px", max_ellipse_h / 2.0))

    if y_window is not None:
        # 跟踪器传入的精确窗口，直接使用
        y_top = max(0, y_window[0])
        y_bot = min(h_img, y_window[1])
    elif mode == "near_edge":
        y_top, y_bot = _get_near_edge_search_band(calib, det_params, h_img)
    else:
        y_top = int(h_img * float(det_params.get("search_top_fraction", 0.02)))
        y_bot = int(h_img * float(det_params.get("search_bottom_fraction", 0.98)))
        y_top = max(0, y_top)
        y_bot = min(h_img, y_bot)
    if y_bot - y_top < 4:
        return None, 0.0

    region = working[y_top:y_bot, x1:x2]

    if region.ndim == 3:
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY).astype(np.float32)
    else:
        gray = region.astype(np.float32)

    ksize = int(det_params.get("blur_kernel", 5))
    if ksize % 2 == 0:
        ksize += 1
    if ksize > 1:
        gray = cv2.GaussianBlur(gray, (ksize, ksize), 0)

    grad = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)

    min_strength = float(det_params.get("gradient_min_strength", 3.0))
    num_cols = grad.shape[1]

    col_ys: List[float] = []
    dual_success = 0  # 成功使用双边缘的列数（dual_edge 模式）

    for col_idx in range(num_cols):
        col_grad = grad[:, col_idx]

        if mode == "near_edge":
            peaks = _find_negative_peaks(col_grad, min_strength)
            if peaks:
                if y_pred is not None:
                    # 有跟踪先验：选隐含中点最靠近预测位置的峰
                    # implied_mid = (y_top + pk) - near_edge_correction
                    best_pk = min(
                        peaks,
                        key=lambda pk: abs((y_top + pk) - near_edge_correction - y_pred),
                    )
                else:
                    # 无先验：取 far edge 向下 max_ellipse_h 内的最后一个峰
                    top_row = peaks[0]
                    best_pk = top_row
                    for pk in peaks[1:]:
                        if pk - top_row <= max_ellipse_h:
                            best_pk = pk
                        else:
                            break
                col_ys.append(float(y_top + best_pk) - near_edge_correction)

        elif mode == "dual_edge":
            peaks = _find_negative_peaks(col_grad, min_strength)
            if len(peaks) >= 2:
                top_row = peaks[0]
                bot_row = top_row
                for pk in peaks[1:]:
                    if pk - top_row <= max_ellipse_h:
                        bot_row = pk
                    else:
                        break
                if bot_row > top_row:
                    mid_y = (y_top + top_row + y_top + bot_row) / 2.0
                    col_ys.append(mid_y)
                    dual_success += 1
                    continue
            # 退化：单峰
            if peaks:
                col_ys.append(float(y_top + peaks[0]))
            else:
                min_val = float(np.min(col_grad))
                if min_val < -min_strength:
                    col_ys.append(float(y_top + int(np.argmin(col_grad))))

        else:
            # single_edge
            min_val = float(np.min(col_grad))
            if min_val < -min_strength:
                col_ys.append(float(y_top + int(np.argmin(col_grad))))

    if not col_ys:
        return None, 0.0

    ys = np.array(col_ys, dtype=np.float32)

    # IQR 过滤
    iqr_fence = float(det_params.get("iqr_fence", 1.5))
    q25, q75 = np.percentile(ys, [25, 75])
    iqr = q75 - q25
    lo = q25 - iqr_fence * iqr
    hi = q75 + iqr_fence * iqr
    mask = (ys >= lo) & (ys <= hi)
    filtered = ys[mask]
    if len(filtered) == 0:
        filtered = ys

    y_estimate = float(np.median(filtered))
    valid_frac = float(len(filtered)) / max(1, num_cols)
    dual_frac = float(dual_success) / max(1, num_cols) if mode == "dual_edge" else 1.0
    confidence = valid_frac * (0.5 + 0.5 * dual_frac)

    min_valid_frac = float(det_params.get("min_valid_column_fraction", 0.15))
    if valid_frac < min_valid_frac:
        return None, confidence

    return y_estimate, confidence


# ---------------------------------------------------------------------------
# Debug frame rendering
# ---------------------------------------------------------------------------

def render_debug_frame(
    roi_bgr: np.ndarray,
    y_detected: Optional[float],
    h_mm: Optional[float],
    confidence: float,
    calib: Dict,
    det_params: Dict,
    pred_y: Optional[float] = None,
    reacquired: bool = False,
) -> np.ndarray:
    canvas = roi_bgr.copy()
    h_img, w_img = canvas.shape[:2]

    x_left = calib["tube_inner"]["x_left"]
    x_right = calib["tube_inner"]["x_right"]
    tube_width = x_right - x_left

    # 全管宽范围（浅色）
    cv2.rectangle(canvas, (x_left, 0), (x_right, h_img - 1), (0, 180, 180), 1)

    # 实际使用的中心列范围（亮色）
    center_frac = float(det_params.get("center_col_fraction", 0.6))
    half_excl = (1.0 - center_frac) / 2.0
    x_margin = int(det_params.get("x_margin_px", 3))
    cx1 = max(0, x_left + x_margin + int(half_excl * tube_width))
    cx2 = min(w_img, x_right - x_margin - int(half_excl * tube_width))
    cv2.rectangle(canvas, (cx1, 0), (cx2, h_img - 1), (0, 255, 255), 1)

    # 检测到的液面中点（绿线）
    if y_detected is not None:
        y_int = int(round(y_detected))
        cv2.line(canvas, (x_left, y_int), (x_right, y_int), (0, 255, 0), 2)
        label = f"{h_mm:.2f} mm" if h_mm is not None else "N/A"
        cv2.putText(canvas, label, (x_right + 4, y_int + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        mode = str(det_params.get("detection_mode", "dual_edge"))
        if mode == "near_edge":
            # 橙线 = 近侧边缘（修正前），绿线 = 修正后中点
            corr = float(det_params.get("near_edge_correction_px",
                                        det_params.get("max_ellipse_height_px", 40.0) / 2.0))
            y_near = int(round(y_detected + corr))
            cv2.line(canvas, (cx1, y_near), (cx2, y_near), (0, 140, 255), 1)
        else:
            # 显示椭圆半高范围（示意）
            max_ell = float(det_params.get("max_ellipse_height_px", 40.0))
            half_h = int(round(max_ell / 2))
            cv2.line(canvas, (cx1, y_int - half_h), (cx2, y_int - half_h), (0, 200, 255), 1)
            cv2.line(canvas, (cx1, y_int + half_h), (cx2, y_int + half_h), (0, 200, 255), 1)

    # 标定参考线（黄色）
    for rp in calib["calibration"]["reference_points"]:
        y_r = int(round(rp["y_px_roi"]))
        cv2.line(canvas, (x_left, y_r), (x_right, y_r), (200, 200, 0), 1)
        cv2.putText(canvas, f"{rp['h_mm']:g}", (x_right + 4, y_r + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 0), 1)

    # 预测位置（品红虚线）
    if pred_y is not None:
        py = int(round(pred_y))
        cv2.line(canvas, (x_left, py), (x_right, py), (255, 0, 255), 1)

    mode = str(det_params.get("detection_mode", "dual_edge"))
    reacq_tag = " REACQ" if reacquired else ""
    cv2.putText(canvas, f"conf={confidence:.2f}  [{mode}]{reacq_tag}", (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    return canvas


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def save_curve_plot(
    stamps: List[float],
    heights: List[Optional[float]],
    confidences: List[float],
    out_path: Path,
    bag_name: str,
) -> None:
    if not _HAS_MPL:
        print("[WARN] matplotlib not available; skipping curve plot.", file=sys.stderr)
        return

    t0 = stamps[0] if stamps else 0.0
    ts = [s - t0 for s in stamps]

    valid_mask = [h is not None for h in heights]
    ts_v = [t for t, v in zip(ts, valid_mask) if v]
    hs_v = [h for h, v in zip(heights, valid_mask) if v]
    conf_v = [c for c, v in zip(confidences, valid_mask) if v]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)  # type: ignore[union-attr]

    ax0 = axes[0]
    sc = ax0.scatter(ts_v, hs_v, c=conf_v, cmap="RdYlGn", vmin=0, vmax=1,
                     s=10, linewidths=0, zorder=3)
    ax0.plot(ts_v, hs_v, color="steelblue", linewidth=0.8, alpha=0.6, zorder=2)
    plt.colorbar(sc, ax=ax0, label="detection confidence")  # type: ignore[union-attr]
    ax0.set_ylabel("Liquid height (mm)")
    ax0.set_title(f"RGB liquid height — {bag_name}")
    ax0.grid(True, linewidth=0.4)

    ax1 = axes[1]
    ax1.plot(ts, confidences, color="darkorange", linewidth=0.8)
    ax1.axhline(0.15, color="red", linewidth=0.6, linestyle="--", label="min conf threshold")
    ax1.set_ylabel("Confidence")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylim(0, 1.05)
    ax1.legend(fontsize=8)
    ax1.grid(True, linewidth=0.4)

    plt.tight_layout()  # type: ignore[union-attr]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=150)
    plt.close(fig)  # type: ignore[union-attr]
    print(f"[OK] saved curve plot: {out_path}")


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

def save_csv(
    out_path: Path,
    frame_indices: List[int],
    stamps: List[float],
    y_pxs: List[Optional[float]],
    heights: List[Optional[float]],
    confidences: List[float],
    pred_ys: Optional[List[Optional[float]]] = None,
    reacquireds: Optional[List[bool]] = None,
) -> None:
    import csv
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_index", "stamp_sec", "y_px_roi", "height_mm",
                         "confidence", "pred_y_roi", "reacquired"])
        for i, (fi, st, yp, hm, conf) in enumerate(
                zip(frame_indices, stamps, y_pxs, heights, confidences)):
            py = pred_ys[i] if pred_ys else None
            ra = reacquireds[i] if reacquireds else False
            writer.writerow([
                fi,
                f"{st:.9f}",
                f"{yp:.2f}" if yp is not None else "",
                f"{hm:.4f}" if hm is not None else "",
                f"{conf:.4f}",
                f"{py:.2f}" if py is not None else "",
                int(ra),
            ])
    print(f"[OK] saved CSV: {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if not _HAS_ROS:
        print("[ERROR] rosbag / cv_bridge not available. Run inside a ROS environment.", file=sys.stderr)
        return 1

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        print(f"[ERROR] bag not found: {bag_path}", file=sys.stderr)
        return 2

    calib_path = Path(args.calibration).expanduser().resolve()
    if not calib_path.exists():
        print(f"[ERROR] calibration not found: {calib_path}", file=sys.stderr)
        return 3

    out_dir = (
        Path(args.out_dir).expanduser().resolve()
        if args.out_dir
        else bag_path.parent / f"{bag_path.stem}_rgb_results"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    calib = load_calibration(calib_path)
    roi_x = calib["roi"]["x"]
    roi_y = calib["roi"]["y"]
    roi_w = calib["roi"]["w"]
    roi_h = calib["roi"]["h"]
    ref_pts = calib["calibration"]["reference_points"]

    # Merge CLI overrides into detection params
    det_params = dict(calib.get("detection_defaults", {}))

    # 从标定 source 字段自动读取 near_edge_correction_px
    calib_source = calib.get("source", {})
    if calib_source.get("near_edge_only", False):
        corr = calib_source.get("near_edge_correction_px")
        if corr is not None:
            det_params.setdefault("near_edge_correction_px", float(corr))

    if args.gradient_min_strength is not None:
        det_params["gradient_min_strength"] = args.gradient_min_strength
    if args.blur_kernel is not None:
        det_params["blur_kernel"] = args.blur_kernel
    if args.search_top_fraction is not None:
        det_params["search_top_fraction"] = args.search_top_fraction
    if args.search_bottom_fraction is not None:
        det_params["search_bottom_fraction"] = args.search_bottom_fraction
    if args.detection_mode is not None:
        det_params["detection_mode"] = args.detection_mode
    if args.max_ellipse_height_px is not None:
        det_params["max_ellipse_height_px"] = args.max_ellipse_height_px
    if args.center_col_fraction is not None:
        det_params["center_col_fraction"] = args.center_col_fraction
    if args.calib_search_margin_px is not None:
        det_params["calib_search_margin_px"] = args.calib_search_margin_px
    if args.tracker_window_px is not None:
        det_params["tracker_window_px"] = args.tracker_window_px
    if args.tracker_max_speed_px is not None:
        det_params["tracker_max_speed_px"] = args.tracker_max_speed_px
    if args.tracker_reacquire_conf is not None:
        det_params["tracker_reacquire_conf"] = args.tracker_reacquire_conf

    mode_str = det_params.get("detection_mode", "dual_edge")
    use_tracker = (mode_str == "near_edge") and not args.no_tracker
    print(f"[INFO] detection_mode     = {mode_str}")
    print(f"[INFO] center_col_fraction= {det_params.get('center_col_fraction', 0.6)}")
    print(f"[INFO] max_ellipse_height = {det_params.get('max_ellipse_height_px', 40)} px")
    if mode_str == "near_edge":
        print(f"[INFO] near_edge_correction={det_params.get('near_edge_correction_px', 'auto')} px")
        print(f"[INFO] tracker            = {'ON' if use_tracker else 'OFF'}")
        if use_tracker:
            print(f"[INFO] tracker_window_px  = {det_params.get('tracker_window_px', 15)}")
            print(f"[INFO] tracker_max_speed  = {det_params.get('tracker_max_speed_px', 10)} px/frame")
            print(f"[INFO] reacquire_conf     = {det_params.get('tracker_reacquire_conf', 0.3)}")

    bridge = CvBridge()  # type: ignore[possibly-undefined]

    # 跟踪器（仅 near_edge 模式启用）
    tracker = LiquidTracker(det_params, calib) if use_tracker else None

    frame_indices: List[int] = []
    stamps: List[float] = []
    y_pxs: List[Optional[float]] = []
    heights: List[Optional[float]] = []
    confidences: List[float] = []
    pred_ys: List[Optional[float]] = []
    reacquireds: List[bool] = []

    debug_dir = out_dir / "debug_frames" if args.debug_every > 0 else None
    if debug_dir:
        debug_dir.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(bag_path), "r") as bag:  # type: ignore[possibly-undefined]
        topic_info = bag.get_type_and_topic_info().topics

        if args.list_topics:
            print("Image topics in bag:")
            for t, info in sorted(topic_info.items()):
                if info.msg_type == "sensor_msgs/Image":
                    print(f"  {t}  ({info.message_count} msgs)")
            return 0

        if args.topic not in topic_info:
            print(f"[ERROR] topic not found: {args.topic}", file=sys.stderr)
            available = [t for t, info in topic_info.items()
                         if info.msg_type == "sensor_msgs/Image"]
            if available:
                print("[INFO] available image topics:", file=sys.stderr)
                for t in available:
                    print(f"  {t}", file=sys.stderr)
            return 4

        bag_start = bag.get_start_time()
        min_stamp = bag_start + max(0.0, args.time_offset)
        raw_idx = 0
        processed = 0

        for _, msg, stamp in bag.read_messages(topics=[args.topic]):  # type: ignore[misc]
            ts = stamp.to_sec()
            if ts < min_stamp:
                continue

            if raw_idx % max(args.every, 1) != 0:
                raw_idx += 1
                continue
            raw_idx += 1

            try:
                bgr = bridge.imgmsg_to_cv2(msg, desired_encoding=args.encoding)
            except CvBridgeError as exc:  # type: ignore[possibly-undefined]
                print(f"[WARN] frame {processed}: cv_bridge error: {exc}", file=sys.stderr)
                frame_indices.append(processed)
                stamps.append(ts)
                y_pxs.append(None)
                heights.append(None)
                confidences.append(0.0)
                pred_ys.append(None)
                reacquireds.append(False)
                processed += 1
                continue

            # Crop ROI
            img_h, img_w = bgr.shape[:2]
            x0 = max(0, roi_x)
            y0 = max(0, roi_y)
            x1 = min(img_w, roi_x + roi_w)
            y1 = min(img_h, roi_y + roi_h)
            roi_bgr = bgr[y0:y1, x0:x1]

            if roi_bgr.size == 0:
                print(f"[WARN] frame {processed}: empty ROI after clip.", file=sys.stderr)
                frame_indices.append(processed)
                stamps.append(ts)
                y_pxs.append(None)
                heights.append(None)
                confidences.append(0.0)
                pred_ys.append(None)
                reacquireds.append(False)
                processed += 1
                continue

            # --- 检测（含跟踪器） ---
            h_roi = roi_bgr.shape[0]
            reacquired = False
            y_pred_cur: Optional[float] = None

            if tracker is not None:
                y_pred_cur = tracker.predict()
                win = tracker.search_window(h_roi, reacquire=False)
                y_px, conf = detect_meniscus(roi_bgr, calib, det_params,
                                             y_window=win, y_pred=y_pred_cur)
                # 置信不足 → 重捕获（扩大窗口，y_pred 仍引导峰选择）
                if conf < tracker.reacquire_conf_thresh or y_px is None:
                    reacquired = True
                    win_r = tracker.search_window(h_roi, reacquire=True)
                    y_px, conf = detect_meniscus(roi_bgr, calib, det_params,
                                                 y_window=win_r, y_pred=y_pred_cur)
                tracker.update(y_px, reacquired)
            else:
                y_px, conf = detect_meniscus(roi_bgr, calib, det_params)

            h_mm = px_to_mm(y_px, ref_pts) if y_px is not None else None

            frame_indices.append(processed)
            stamps.append(ts)
            y_pxs.append(y_px)
            heights.append(h_mm)
            confidences.append(conf)
            pred_ys.append(y_pred_cur)
            reacquireds.append(reacquired)

            if args.debug_every > 0 and processed % args.debug_every == 0:
                dbg = render_debug_frame(roi_bgr, y_px, h_mm, conf, calib, det_params,
                                         pred_y=y_pred_cur, reacquired=reacquired)
                dbg_path = debug_dir / f"debug_{processed:06d}.png"  # type: ignore[operator]
                cv2.imwrite(str(dbg_path), dbg)

            if processed % 50 == 0:
                h_str = f"{h_mm:.2f} mm" if h_mm is not None else "N/A"
                reacq_tag = " [REACQ]" if reacquired else ""
                print(f"[INFO] frame {processed:5d}  t={ts - bag_start:.2f}s  "
                      f"h={h_str}  conf={conf:.2f}{reacq_tag}")

            processed += 1
            if args.max_frames > 0 and processed >= args.max_frames:
                break

    if not frame_indices:
        print("[WARN] no frames processed.", file=sys.stderr)
        return 5

    # Save outputs
    save_csv(out_dir / f"{bag_path.stem}_rgb_heights.csv",
             frame_indices, stamps, y_pxs, heights, confidences,
             pred_ys=pred_ys, reacquireds=reacquireds)

    plot_path = out_dir / f"{bag_path.stem}_rgb_curve.png"
    save_curve_plot(stamps, heights, confidences, plot_path, bag_path.stem)

    # Summary
    valid_heights = [h for h in heights if h is not None]
    valid_confs = [c for c, h in zip(confidences, heights) if h is not None]
    n_total = len(frame_indices)
    n_valid = len(valid_heights)
    print(f"[OK] processed {n_total} frames, {n_valid} with valid detection "
          f"({100.0 * n_valid / max(1, n_total):.1f}%)")
    if valid_heights:
        h_lo = float(min(h for h in valid_heights if h is not None))  # type: ignore[arg-type]
        h_hi = float(max(h for h in valid_heights if h is not None))  # type: ignore[arg-type]
        print(f"[OK] height range: {h_lo:.2f} mm ~ {h_hi:.2f} mm")
        print(f"[OK] mean confidence: {float(np.mean(valid_confs)):.3f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
