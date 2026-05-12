"""纯几何工具 — 无 ROS 依赖，可独立单元测试。

从 anti_slosh_path_post_processor.py 模块级函数迁移而来。
"""

import math


def yaw_to_quat(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


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


def safe_ratio(value, reference):
    denom = abs(reference)
    if denom < 1e-6:
        return 1.0 if abs(value) < 1e-6 else 1e6
    return value / denom


def clamp(value, lo, hi):
    return max(lo, min(hi, value))
