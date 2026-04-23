#!/usr/bin/env python3
"""
Interactive RGB ruler calibration for liquid-height measurement.

椭圆液面说明
------------
单侧摄像头斜视试管时，液面（水平圆面）在图像中投影为椭圆：
  - 椭圆上沿 = 液面远侧边缘（穿过管壁可见）
  - 椭圆下沿 = 液面近侧边缘（靠摄像头一侧）
  - 椭圆中点 = 真实液位高度

标定时必须使用椭圆中点对应的像素坐标，否则会引入系统性偏差。

标定流程
--------
1. 选 ROI（框选试管区域）。
2. 点击左侧管壁上端 → 下端（2 次）。
3. 点击右侧管壁上端 → 下端（2 次）。
4. 按顺序点击刻度标记（从低到高，对应 --ruler-heights-mm）：
   - --ruler-clicks-per-height 1：点击椭圆中点（目视估计上下沿均值处）。
   - --ruler-clicks-per-height 2（推荐）：先点椭圆上沿，再点椭圆下沿，
     脚本自动取均值 = 椭圆中点，无需目视估计。
5. 按 Enter 验证并保存；u 撤销；r 全部重置；q/Esc 退出。

输出
----
- YAML 标定文件（ROI、管壁 x 范围、分段线性 pixel_y → mm 映射）。
- 标注预览图（PNG）。

Usage example
-------------
python RGB_calibrate.py \\
    --image /data/a/bags/Q0_test1_frames/frame_000100.png \\
    --ruler-heights-mm 0,5,10,15,20 \\
    --ruler-clicks-per-height 2 \\
    --output-yaml /data/a/bags/Q0_test1_rgb_calib.yaml
"""

import argparse
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


WINDOW_NAME = "RGB Ruler Calibration"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_ruler_heights(raw: str) -> List[float]:
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) < 2:
        raise argparse.ArgumentTypeError("need at least 2 comma-separated heights, e.g. 0,5,10")
    try:
        vals = [float(p) for p in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"cannot parse --ruler-heights-mm: {raw}") from exc
    for prev, cur in zip(vals, vals[1:]):
        if cur <= prev:
            raise argparse.ArgumentTypeError("--ruler-heights-mm must be strictly increasing")
    return vals


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive RGB ruler calibration.")
    p.add_argument("--image", required=True, help="Reference frame PNG/JPG.")
    p.add_argument(
        "--ruler-heights-mm",
        required=True,
        type=parse_ruler_heights,
        help="Strictly-increasing comma-separated heights on the ruler, e.g. 0,5,10,15,20.",
    )
    p.add_argument(
        "--ruler-clicks-per-height",
        type=int,
        default=1,
        choices=[1, 2],
        help=(
            "1 = 每个刻度点一次：直接点椭圆上下沿的目视中点。 "
            "2（推荐）= 每个刻度点两次：先点椭圆上沿（液面远侧可见边），"
            "再点椭圆下沿（液面近侧边），脚本自动取 y 均值 = 真实液位像素坐标。"
        ),
    )
    p.add_argument("--output-yaml", default="", help="Output YAML path.")
    p.add_argument("--output-image", default="", help="Annotated preview PNG path.")
    p.add_argument(
        "--max-wall-x-skew-px",
        type=float,
        default=6.0,
        help="Warn if tube axis is tilted more than this many px per 100 px height. Default 6.0.",
    )
    p.add_argument(
        "--trace-zero-mm",
        action="store_true",
        default=False,
        help=(
            "启用 0mm 弧线描点模式：完成管壁点击后，用鼠标沿液面椭圆弧线点多个点，"
            "按 Space 确认。脚本自动用点集的 (y_min+y_max)/2 作为 0mm 标定参考，"
            "并从 y_max-y_min 推断 max_ellipse_height_px。"
            "之后剩余刻度正常按 --ruler-clicks-per-height 模式点击。"
        ),
    )
    p.add_argument(
        "--near-edge-only",
        action="store_true",
        default=False,
        help=(
            "0mm 描点后，后续刻度只需点击椭圆下沿（近侧）。"
            "脚本用 0mm 描点测量的椭圆高度自动修正到真实液位中点："
            "y_mid = y_near_edge - ellipse_height / 2。"
            "需配合 --trace-zero-mm 使用。"
        ),
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def avg_x_of_line(pts: List[Tuple[int, int]]) -> float:
    return sum(p[0] for p in pts) / len(pts)


def avg_y_of_line(pts: List[Tuple[int, int]]) -> float:
    return sum(p[1] for p in pts) / len(pts)


def rotation_deg_of_axis(top: Tuple[int, int], bot: Tuple[int, int]) -> float:
    dx = bot[0] - top[0]
    dy = bot[1] - top[1]
    if abs(dy) < 1e-6:
        return 0.0
    theta = math.degrees(math.atan2(dx, dy))  # deviation from vertical
    return theta


def apply_rotation(roi_hw: Tuple[int, int], pt_roi: Tuple[float, float], rot_deg: float) -> Tuple[float, float]:
    h, w = roi_hw
    cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
    rad = math.radians(rot_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    x, y = pt_roi[0] - cx, pt_roi[1] - cy
    xr = cos_r * x - sin_r * y + cx
    yr = sin_r * x + cos_r * y + cy
    return xr, yr


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

class CalibState:
    STEPS = [
        ("left_wall_top",    "Step 1: click LEFT inner wall – near TOP"),
        ("left_wall_bottom", "Step 2: click LEFT inner wall – near BOTTOM"),
        ("right_wall_top",   "Step 3: click RIGHT inner wall – near TOP"),
        ("right_wall_bottom","Step 4: click RIGHT inner wall – near BOTTOM"),
    ]
    TRACE_MIN_POINTS = 3  # 描点模式最少需要几个点

    def __init__(self, ruler_heights_mm: List[float], clicks_per_height: int,
                 trace_zero_mm: bool = False, near_edge_only: bool = False):
        self.geom: Dict[str, Optional[Tuple[int, int]]] = {k: None for k, _ in self.STEPS}
        self.ruler_heights_mm = ruler_heights_mm
        self.trace_zero_mm = trace_zero_mm
        self.near_edge_only = near_edge_only
        # near_edge_only 强制每个刻度只点一次（下沿）
        self.clicks_per_height = 1 if near_edge_only else clicks_per_height
        # ruler_clicks[i] = 该高度的点击列表
        # 当 trace_zero_mm=True 时，ruler_clicks[0] 存放弧线描点（可任意多个）
        # 其他 height 仍用 clicks_per_height 限制
        self.ruler_clicks: List[List[Tuple[int, int]]] = [[] for _ in ruler_heights_mm]
        # 描点模式：是否已按 Space 确认 0mm 弧线
        self.trace_finalized: bool = False
        # 由 finalize_trace() 填入，near_edge_only 修正时使用
        self.measured_ellipse_height_px: Optional[float] = None
        # near_edge_only 模式：非 0mm 刻度是否已 Space 确认（index 0 始终 False，由 trace_finalized 管）
        self.near_edge_confirmed: List[bool] = [False] * len(ruler_heights_mm)

    def geom_complete(self) -> bool:
        return all(v is not None for v in self.geom.values())

    def _in_trace_phase(self) -> bool:
        """当前是否处于 0mm 描点阶段（管壁已完成，描点未确认）。"""
        return (self.trace_zero_mm
                and self.geom_complete()
                and not self.trace_finalized)

    def _near_edge_active_idx(self) -> Optional[int]:
        """near_edge_only 模式下当前正在描点的刻度索引，全部完成返回 None。"""
        if not self.near_edge_only or not self.trace_finalized:
            return None
        for i in range(1, len(self.ruler_heights_mm)):
            if not self.near_edge_confirmed[i]:
                return i
        return None

    def ruler_complete(self) -> bool:
        if self.trace_zero_mm:
            if not self.trace_finalized:
                return False
            if self.near_edge_only:
                return all(self.near_edge_confirmed[i]
                           for i in range(1, len(self.ruler_heights_mm)))
            # 普通模式：检查剩余刻度点击数
            return all(
                len(self.ruler_clicks[i]) >= self.clicks_per_height
                for i in range(1, len(self.ruler_heights_mm))
            )
        return all(len(c) >= self.clicks_per_height for c in self.ruler_clicks)

    def complete(self) -> bool:
        return self.geom_complete() and self.ruler_complete()

    def current_step_label(self) -> str:
        for key, label in self.STEPS:
            if self.geom[key] is None:
                return label
        if self._in_trace_phase():
            n = len(self.ruler_clicks[0])
            h0 = self.ruler_heights_mm[0]
            if n < self.TRACE_MIN_POINTS:
                return (f"Trace {h0:g} mm arc – click along the ellipse "
                        f"({n}/{self.TRACE_MIN_POINTS} pts, need more) – Space=done")
            return (f"Trace {h0:g} mm arc – {n} pts – "
                    "Space=CONFIRM arc  u=undo last pt  r=reset")
        if self.near_edge_only:
            idx = self._near_edge_active_idx()
            if idx is not None:
                h = self.ruler_heights_mm[idx]
                n = len(self.ruler_clicks[idx])
                corr = (f"  corr={self.measured_ellipse_height_px/2:.1f}px"
                        if self.measured_ellipse_height_px is not None else "")
                if n == 0:
                    return (f"Trace {h:g} mm near edge – click along lower arc{corr}"
                            " – Space=CONFIRM")
                return (f"Trace {h:g} mm near edge – {n} pts{corr}"
                        " – Space=CONFIRM  u=undo last pt")
            return "Done – press Enter to save"
        start = 1 if self.trace_zero_mm else 0
        for i in range(start, len(self.ruler_clicks)):
            clicks = self.ruler_clicks[i]
            h = self.ruler_heights_mm[i]
            if len(clicks) < self.clicks_per_height:
                side_label = ""
                if self.clicks_per_height == 2:
                    side_label = " [椭圆上沿-远侧]" if len(clicks) == 0 else " [椭圆下沿-近侧]"
                return f"Step: click {h:g} mm mark{side_label}"
        return "Done – press Enter to save"

    def push_click(self, pt: Tuple[int, int]):
        for key, _ in self.STEPS:
            if self.geom[key] is None:
                self.geom[key] = pt
                return
        if self._in_trace_phase():
            self.ruler_clicks[0].append(pt)
            return
        if self.near_edge_only:
            idx = self._near_edge_active_idx()
            if idx is not None:
                self.ruler_clicks[idx].append(pt)
            return
        start = 1 if self.trace_zero_mm else 0
        for i in range(start, len(self.ruler_clicks)):
            if len(self.ruler_clicks[i]) < self.clicks_per_height:
                self.ruler_clicks[i].append(pt)
                return

    def finalize_trace(self) -> bool:
        """按 Space 确认描点，返回是否成功。"""
        if not self._in_trace_phase():
            return False
        if len(self.ruler_clicks[0]) < self.TRACE_MIN_POINTS:
            return False
        self.trace_finalized = True
        stats = self.trace_y_stats()
        if stats is not None:
            self.measured_ellipse_height_px = stats[1] - stats[0]
        return True

    def finalize_near_edge(self) -> bool:
        """Space 确认当前 near_edge 刻度，返回是否成功。"""
        idx = self._near_edge_active_idx()
        if idx is None:
            return False
        if len(self.ruler_clicks[idx]) < 1:
            return False
        self.near_edge_confirmed[idx] = True
        return True

    def undo(self):
        if self._in_trace_phase():
            if self.ruler_clicks[0]:
                self.ruler_clicks[0].pop()
            return
        if self.near_edge_only:
            # 先尝试撤销当前活跃刻度的最后一个点
            idx = self._near_edge_active_idx()
            if idx is not None and self.ruler_clicks[idx]:
                self.ruler_clicks[idx].pop()
                return
            # 撤销最后一个已确认的刻度
            for i in range(len(self.ruler_heights_mm) - 1, 0, -1):
                if self.near_edge_confirmed[i]:
                    self.near_edge_confirmed[i] = False
                    return
            # 撤销 0mm trace 确认
            if self.trace_finalized:
                self.trace_finalized = False
                self.measured_ellipse_height_px = None
                return
            if self.ruler_clicks[0]:
                self.ruler_clicks[0].pop()
            return
        # 普通模式
        start = 1 if self.trace_zero_mm else 0
        for i in range(len(self.ruler_clicks) - 1, start - 1, -1):
            if self.ruler_clicks[i]:
                self.ruler_clicks[i].pop()
                return
        if self.trace_zero_mm and self.trace_finalized:
            self.trace_finalized = False
            return
        for key, _ in reversed(self.STEPS):
            if self.geom[key] is not None:
                self.geom[key] = None
                return

    def reset(self):
        for k in self.geom:
            self.geom[k] = None
        for c in self.ruler_clicks:
            c.clear()
        self.trace_finalized = False
        self.measured_ellipse_height_px = None
        self.near_edge_confirmed = [False] * len(self.ruler_heights_mm)

    def trace_y_stats(self) -> Optional[Tuple[float, float, float]]:
        """返回 (y_min, y_max, y_mid)，仅在有描点时有效。"""
        pts = self.ruler_clicks[0]
        if not pts:
            return None
        ys = [p[1] for p in pts]
        y_min, y_max = float(min(ys)), float(max(ys))
        return y_min, y_max, (y_min + y_max) / 2.0


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_cross(canvas, pt: Tuple[int, int], color, size: int = 6, thickness: int = 2):
    x, y = pt
    cv2.line(canvas, (x - size, y), (x + size, y), color, thickness)
    cv2.line(canvas, (x, y - size), (x, y + size), color, thickness)


def draw_overlay(image, roi, state: CalibState) -> np.ndarray:
    canvas = image.copy()
    x, y, w, h = roi
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)

    # Wall lines
    lw_t = state.geom["left_wall_top"]
    lw_b = state.geom["left_wall_bottom"]
    rw_t = state.geom["right_wall_top"]
    rw_b = state.geom["right_wall_bottom"]

    if lw_t:
        draw_cross(canvas, lw_t, (255, 80, 80))
    if lw_b:
        draw_cross(canvas, lw_b, (255, 80, 80))
    if lw_t and lw_b:
        cv2.line(canvas, lw_t, lw_b, (255, 80, 80), 2)

    if rw_t:
        draw_cross(canvas, rw_t, (80, 80, 255))
    if rw_b:
        draw_cross(canvas, rw_b, (80, 80, 255))
    if rw_t and rw_b:
        cv2.line(canvas, rw_t, rw_b, (80, 80, 255), 2)

    # --- 0mm 描点可视化（trace mode） ---
    if state.trace_zero_mm:
        trace_pts = state.ruler_clicks[0]
        h0 = state.ruler_heights_mm[0]
        stats = state.trace_y_stats()
        # 已描的点：只画小圆点，不连线（连线遮挡视线）
        for pt in trace_pts:
            cv2.circle(canvas, pt, 3, (0, 255, 128), -1)
        # 描点确认后（Space）才显示 y_min/y_max/y_mid 参考线，描点过程中不显示
        if stats is not None and state.trace_finalized:
            y_min_v, y_max_v, y_mid_v = stats
            rx, ry, rw, _rh = roi
            lx, rx_end = rx, rx + rw
            # 上沿（远侧）：蓝色虚线
            cv2.line(canvas, (lx, int(y_min_v)), (rx_end, int(y_min_v)), (255, 180, 0), 1)
            cv2.putText(canvas, "far edge", (rx_end + 4, int(y_min_v) + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 180, 0), 1)
            # 下沿（近侧）：橙色虚线
            cv2.line(canvas, (lx, int(y_max_v)), (rx_end, int(y_max_v)), (0, 140, 255), 1)
            cv2.putText(canvas, "near edge", (rx_end + 4, int(y_max_v) + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 140, 255), 1)
            # 中点（真实液位）：绿色实线
            cv2.line(canvas, (lx, int(y_mid_v)), (rx_end, int(y_mid_v)), (0, 255, 0), 2)
            ell_h = y_max_v - y_min_v
            cv2.putText(canvas, f"{h0:g}mm mid  ell={ell_h:.1f}px",
                        (rx_end + 4, int(y_mid_v) + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # 描点已确认：画一个对勾标记
        if state.trace_finalized and stats is not None:
            y_mid_v = stats[2]
            cv2.putText(canvas, f"[TRACED {h0:g}mm]", (x, int(y_mid_v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    # --- 其余刻度点击可视化 ---
    start = 1 if state.trace_zero_mm else 0
    colors_by_side = [(0, 255, 0), (0, 200, 255)]
    for i in range(start, len(state.ruler_clicks)):
        clicks = state.ruler_clicks[i]
        h_mm = state.ruler_heights_mm[i]
        if state.near_edge_only:
            is_active = (i == state._near_edge_active_idx())
            is_confirmed = state.near_edge_confirmed[i]
            dot_color = (0, 200, 255) if is_active else (0, 100, 180)
            # 描的点：小圆点
            for pt in clicks:
                cv2.circle(canvas, pt, 3, dot_color, -1)
            # 确认后才显示参考线，描点过程中只显示圆点
            if is_confirmed and len(clicks) >= 1 and state.measured_ellipse_height_px is not None:
                y_near_avg = sum(p[1] for p in clicks) / len(clicks)
                y_mid = int(round(y_near_avg - state.measured_ellipse_height_px / 2.0))
                cv2.line(canvas, (x, int(y_near_avg)), (x + w, int(y_near_avg)),
                         (0, 140, 255), 1)
                cv2.line(canvas, (x, y_mid), (x + w, y_mid), (0, 255, 0), 2)
                cv2.putText(canvas, f"{h_mm:g}mm mid", (x + w + 4, y_mid + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
        else:
            for j, pt in enumerate(clicks):
                color = colors_by_side[j % 2]
                draw_cross(canvas, pt, color, size=8)
                cv2.putText(
                    canvas, f"{h_mm:g}mm",
                    (pt[0] + 6, pt[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                )
            # 若该刻度已点两次，画出中点线
            if len(clicks) == 2:
                y_avg = int(round((clicks[0][1] + clicks[1][1]) / 2.0))
                cv2.line(canvas, (x, y_avg), (x + w, y_avg), (0, 255, 0), 1)

    # --- HUD ---
    cv2.putText(canvas, "RGB Ruler Calibration", (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(canvas, state.current_step_label(), (x, y + h + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
    keys_hint = "u=undo  r=reset  Enter=save  q/Esc=quit"
    if state._in_trace_phase() or (state.near_edge_only and state._near_edge_active_idx() is not None):
        keys_hint = "Left-click=add pt  Space=CONFIRM  u=undo last pt  r=reset"
    cv2.putText(canvas, keys_hint,
                (x, y + h + 46), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
    return canvas


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate(state: CalibState, roi, max_skew_px: float) -> Tuple[List[str], List[str]]:
    warnings: List[str] = []
    errors: List[str] = []
    _roi_x, _roi_y, _roi_w, _roi_h = roi

    lw_t = state.geom["left_wall_top"]
    lw_b = state.geom["left_wall_bottom"]
    rw_t = state.geom["right_wall_top"]
    rw_b = state.geom["right_wall_bottom"]

    if lw_t is None or lw_b is None or rw_t is None or rw_b is None:
        errors.append("Wall geometry incomplete – complete all 4 wall clicks first.")
        return warnings, errors

    left_x_avg = avg_x_of_line([lw_t, lw_b])
    right_x_avg = avg_x_of_line([rw_t, rw_b])
    if left_x_avg >= right_x_avg:
        errors.append("Left wall is not to the left of the right wall – recheck clicks.")

    # Tilt check: how many px does the axis shift per 100 px vertically?
    axis_skew = abs(
        avg_x_of_line([lw_t, rw_t]) - avg_x_of_line([lw_b, rw_b])
    )
    tube_height_px = max(1, abs(lw_b[1] - lw_t[1]))
    skew_per_100 = axis_skew / tube_height_px * 100.0
    if skew_per_100 > max_skew_px:
        warnings.append(
            f"Tube axis skew = {skew_per_100:.1f} px / 100 px (threshold {max_skew_px} px). "
            "Consider enabling rectification in the inference script."
        )

    # Ruler clicks must be strictly increasing in height → strictly decreasing in y
    ref_pts = _compute_reference_points(state, roi)
    for idx in range(len(ref_pts) - 1):
        if ref_pts[idx]["y_px_roi"] <= ref_pts[idx + 1]["y_px_roi"]:
            errors.append(
                f"Ruler points are not ordered: h={ref_pts[idx]['h_mm']:g} mm should be LOWER "
                f"in the image (larger y_px) than h={ref_pts[idx+1]['h_mm']:g} mm. Recheck click order."
            )
            break

    return warnings, errors


def _compute_reference_points(state: CalibState, roi) -> List[dict]:
    _rx, roi_y, _rw, _rh = roi
    ref_pts = []
    for i, (h_mm, clicks) in enumerate(zip(state.ruler_heights_mm, state.ruler_clicks)):
        if not clicks:
            continue
        if state.trace_zero_mm and i == 0:
            # 描点模式：用 (y_min + y_max) / 2 作为标定参考
            ys = [p[1] for p in clicks]
            y_img = (min(ys) + max(ys)) / 2.0
        elif state.near_edge_only and state.measured_ellipse_height_px is not None:
            # 多点下沿取均值，再减去半椭圆高度得到真实液位中点
            y_near = sum(p[1] for p in clicks) / len(clicks)
            y_img = y_near - state.measured_ellipse_height_px / 2.0
        else:
            # 普通模式：所有点击的 y 均值
            y_img = sum(p[1] for p in clicks) / len(clicks)
        y_roi = y_img - roi_y
        ref_pts.append({"y_px_roi": float(y_roi), "h_mm": float(h_mm)})
    ref_pts.sort(key=lambda p: p["h_mm"])
    return ref_pts


# ---------------------------------------------------------------------------
# YAML output
# ---------------------------------------------------------------------------

def write_yaml(
    path: Path,
    image_path: Path,
    roi,
    state: CalibState,
    max_skew_px: float,
) -> None:
    x, y, w, h = roi
    lw_t: Tuple[int, int] = state.geom["left_wall_top"]    # type: ignore[assignment]
    lw_b: Tuple[int, int] = state.geom["left_wall_bottom"]  # type: ignore[assignment]
    rw_t: Tuple[int, int] = state.geom["right_wall_top"]    # type: ignore[assignment]
    rw_b: Tuple[int, int] = state.geom["right_wall_bottom"] # type: ignore[assignment]
    # write_yaml is only called after state.complete() is True, so none are None

    left_x_roi = int(round(avg_x_of_line([lw_t, lw_b]) - x))
    right_x_roi = int(round(avg_x_of_line([rw_t, rw_b]) - x))

    # Tilt angle of the tube centre axis (positive = tilted right)
    axis_top_x = avg_x_of_line([lw_t, rw_t])
    axis_bot_x = avg_x_of_line([lw_b, rw_b])
    axis_top_y = avg_y_of_line([lw_t, rw_t])
    axis_bot_y = avg_y_of_line([lw_b, rw_b])
    rotation_deg = rotation_deg_of_axis(
        (int(axis_top_x), int(axis_top_y)),
        (int(axis_bot_x), int(axis_bot_y)),
    )

    ref_pts = _compute_reference_points(state, roi)

    # 描点模式：计算椭圆高度（用于自动设置 max_ellipse_height_px）
    measured_ellipse_height_px: Optional[float] = None
    if state.trace_zero_mm:
        stats = state.trace_y_stats()
        if stats is not None:
            measured_ellipse_height_px = stats[1] - stats[0]  # y_max - y_min

    # Store all image-coordinate ruler clicks for reference
    ruler_clicks_image = []
    for i, (h_mm, clicks) in enumerate(zip(state.ruler_heights_mm, state.ruler_clicks)):
        is_trace = (state.trace_zero_mm and i == 0)
        for click_idx, pt in enumerate(clicks):
            ruler_clicks_image.append({
                "h_mm": float(h_mm),
                "click_type": "trace_arc" if is_trace else ("far_edge" if click_idx == 0 else "near_edge"),
                "x_px_image": float(pt[0]),
                "y_px_image": float(pt[1]),
            })

    # detection_defaults：如果有描点测量值，用实测椭圆高度（+10px 余量）
    default_ellipse_h = (
        int(round(measured_ellipse_height_px * 1.2)) + 4
        if measured_ellipse_height_px is not None
        else 40
    )

    data = {
        "source": {
            "image": str(image_path),
            "mode": "rgb_ruler_calibration",
            "annotated_at": datetime.now().isoformat(timespec="seconds"),
            "ruler_clicks_per_height": state.clicks_per_height,
            "trace_zero_mm": state.trace_zero_mm,
            "near_edge_only": state.near_edge_only,
            "measured_ellipse_height_px": (
                float(measured_ellipse_height_px) if measured_ellipse_height_px is not None else None
            ),
            "near_edge_correction_px": (
                float(measured_ellipse_height_px / 2.0)
                if state.near_edge_only and measured_ellipse_height_px is not None
                else None
            ),
        },
        "roi": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
        "tube_inner": {
            "x_left": left_x_roi,
            "x_right": right_x_roi,
            "note": "x coordinates relative to ROI top-left corner",
        },
        "rotation_deg": float(rotation_deg),
        "calibration": {
            "mode": "piecewise_linear",
            "coordinate_frame": "roi",
            "note": (
                "y_px_roi is measured from the ROI top edge. "
                "Higher liquid level = smaller y_px_roi."
            ),
            "reference_points": ref_pts,
        },
        "detection_defaults": {
            "detection_mode": "near_edge" if state.near_edge_only else "dual_edge",
            "max_ellipse_height_px": default_ellipse_h,
            "center_col_fraction": 0.6,
            "x_margin_px": 3,
            "blur_kernel": 5,
            "gradient_min_strength": 3.0,
            "search_top_fraction": 0.02,
            "search_bottom_fraction": 0.98,
            "iqr_fence": 1.5,
            "min_valid_column_fraction": 0.15,
        },
        "ruler_clicks_image": ruler_clicks_image,
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=False)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    if args.near_edge_only and not args.trace_zero_mm:
        print("[ERROR] --near-edge-only requires --trace-zero-mm", file=sys.stderr)
        return 1

    image_path = Path(args.image).expanduser().resolve()
    if not image_path.exists():
        print(f"[ERROR] image not found: {image_path}", file=sys.stderr)
        return 1

    image = cv2.imread(str(image_path))
    if image is None:
        print(f"[ERROR] failed to read image: {image_path}", file=sys.stderr)
        return 2

    out_yaml = (
        Path(args.output_yaml).expanduser().resolve()
        if args.output_yaml
        else image_path.parent / f"{image_path.stem}_rgb_calib.yaml"
    )
    out_img = (
        Path(args.output_image).expanduser().resolve()
        if args.output_image
        else image_path.parent / f"{image_path.stem}_rgb_calib_annotated.png"
    )

    # Step 1: select ROI
    roi = cv2.selectROI("Select tube ROI", image, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select tube ROI")
    if roi[2] <= 0 or roi[3] <= 0:
        print("[ERROR] ROI not selected.", file=sys.stderr)
        return 3

    state = CalibState(args.ruler_heights_mm, args.ruler_clicks_per_height,
                       trace_zero_mm=args.trace_zero_mm,
                       near_edge_only=args.near_edge_only)
    pending_warnings: List[str] = []
    pending_errors: List[str] = []
    warn_ack = False

    def on_mouse(event, mx, my, _flags, _ud):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state.complete():
            return
        pt = (mx, my)
        state.push_click(pt)
        nonlocal pending_warnings, pending_errors, warn_ack
        pending_warnings, pending_errors, warn_ack = [], [], False

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        canvas = draw_overlay(image, roi, state)

        # Show messages
        x_roi, y_roi, _w, _h = roi
        msg_y = y_roi + roi[3] + 70
        for i, msg in enumerate(pending_errors[:3]):
            cv2.putText(canvas, f"ERR: {msg}", (x_roi, msg_y + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
        for i, msg in enumerate(pending_warnings[:3]):
            cv2.putText(canvas, f"WARN: {msg}", (x_roi, msg_y + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
        if warn_ack:
            cv2.putText(canvas, "Press Enter again to save with warnings, or u/r to fix.",
                        (x_roi, msg_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 200, 255), 1)

        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(30) & 0xFF

        if key in (27, ord("q")):
            cv2.destroyAllWindows()
            print("[INFO] cancelled.")
            return 4

        if key == ord("r"):
            state.reset()
            pending_warnings, pending_errors, warn_ack = [], [], False
            continue

        if key == ord("u"):
            state.undo()
            pending_warnings, pending_errors, warn_ack = [], [], False
            continue

        if key == ord(" "):  # Space：确认当前描点
            if state._in_trace_phase():
                ok = state.finalize_trace()
                if not ok:
                    pending_errors = [
                        f"Need at least {CalibState.TRACE_MIN_POINTS} trace points before confirming."
                    ]
                else:
                    stats = state.trace_y_stats()
                    if stats:
                        ell_h = stats[1] - stats[0]
                        print(f"[INFO] trace confirmed: {len(state.ruler_clicks[0])} points, "
                              f"ellipse_height={ell_h:.1f}px, midpoint_y={stats[2]:.1f}px")
                    pending_warnings, pending_errors, warn_ack = [], [], False
            elif state.near_edge_only:
                idx = state._near_edge_active_idx()
                ok = state.finalize_near_edge()
                if not ok:
                    pending_errors = ["Need at least 1 near-edge point before confirming."]
                else:
                    n = len(state.ruler_clicks[idx])  # type: ignore[index]
                    h = state.ruler_heights_mm[idx]   # type: ignore[index]
                    y_near = sum(p[1] for p in state.ruler_clicks[idx]) / n  # type: ignore[index]
                    corr = state.measured_ellipse_height_px or 0.0
                    print(f"[INFO] near-edge confirmed: {h:g}mm  {n} pts  "
                          f"y_near={y_near:.1f}px  y_mid={y_near - corr/2:.1f}px")
                    pending_warnings, pending_errors, warn_ack = [], [], False
            continue

        if key in (10, 13):  # Enter
            if not state.complete():
                print("[WARN] annotation not complete yet.", file=sys.stderr)
                continue

            warnings, errors = validate(state, roi, args.max_wall_x_skew_px)
            if errors:
                pending_errors = errors
                pending_warnings = warnings
                warn_ack = False
                for e in errors:
                    print(f"[ERROR] {e}", file=sys.stderr)
                continue

            if warnings and not warn_ack:
                pending_warnings = warnings
                pending_errors = []
                warn_ack = True
                for w in warnings:
                    print(f"[WARN] {w}", file=sys.stderr)
                continue

            break  # OK

    cv2.destroyAllWindows()

    write_yaml(out_yaml, image_path, roi, state, args.max_wall_x_skew_px)

    annotated = draw_overlay(image, roi, state)
    out_img.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_img), annotated)

    ref_pts = _compute_reference_points(state, roi)
    x_r, y_r, w_r, h_r = roi
    print(f"[OK] saved calibration yaml : {out_yaml}")
    print(f"[OK] saved annotated image  : {out_img}")
    print(f"[OK] ROI                    : x={x_r} y={y_r} w={w_r} h={h_r}")
    lw_t_f: Tuple[int, int] = state.geom["left_wall_top"]    # type: ignore[assignment]
    lw_b_f: Tuple[int, int] = state.geom["left_wall_bottom"]  # type: ignore[assignment]
    rw_t_f: Tuple[int, int] = state.geom["right_wall_top"]    # type: ignore[assignment]
    rw_b_f: Tuple[int, int] = state.geom["right_wall_bottom"] # type: ignore[assignment]
    print(f"[OK] tube_inner.x_left      : {int(round(avg_x_of_line([lw_t_f, lw_b_f]) - x_r))}")
    print(f"[OK] tube_inner.x_right     : {int(round(avg_x_of_line([rw_t_f, rw_b_f]) - x_r))}")
    for rp in ref_pts:
        print(f"[OK] reference_point: h={rp['h_mm']:g} mm  y_px_roi={rp['y_px_roi']:.1f}")
    if state.trace_zero_mm:
        stats = state.trace_y_stats()
        if stats:
            ell_h = stats[1] - stats[0]
            print(f"[OK] traced ellipse height  : {ell_h:.1f} px  "
                  f"(saved as max_ellipse_height_px={int(round(ell_h * 1.2)) + 4})")
            print(f"[OK] trace point count      : {len(state.ruler_clicks[0])}")
            if state.near_edge_only:
                print(f"[OK] near-edge correction   : -{ell_h/2:.1f} px applied to all non-0mm marks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
