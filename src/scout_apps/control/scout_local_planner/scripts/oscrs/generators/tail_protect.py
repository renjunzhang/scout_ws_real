"""GeoRef terminal tail protection helpers.

These helpers are intentionally geometry-only and have no ROS dependency.
The first implementation uses raw-tail replacement because it is easy to
reason about and easy to disable for behavior-preserving regression tests.
"""

import math


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _tail_start_index(points, distance):
    if len(points) < 2 or distance <= 0.0:
        return len(points) - 1
    acc = 0.0
    for i in range(len(points) - 1, 0, -1):
        acc += _dist(points[i], points[i - 1])
        if acc >= distance:
            return i - 1
    return 0


def _point_to_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return _dist(p, a)
    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = max(0.0, min(1.0, t))
    q = (ax + t * vx, ay + t * vy)
    return _dist(p, q)


def _point_to_polyline_distance(p, polyline):
    if not polyline:
        return float("inf")
    if len(polyline) == 1:
        return _dist(p, polyline[0])
    return min(
        _point_to_segment_distance(p, polyline[i - 1], polyline[i])
        for i in range(1, len(polyline))
    )


def replace_raw_tail(candidate, reference, distance):
    """Replace the last distance meters of candidate with reference tail."""
    if len(candidate) < 2 or len(reference) < 2 or distance <= 0.0:
        return list(candidate)
    cand_start = _tail_start_index(candidate, distance)
    ref_start = _tail_start_index(reference, distance)
    head = list(candidate[:cand_start])
    tail = list(reference[ref_start:])
    if not head:
        return tail
    if tail and _dist(head[-1], tail[0]) < 1e-9:
        return head + tail[1:]
    return head + tail


def protect_tail(candidate, reference, distance, mode):
    """Apply terminal protection to a candidate path."""
    mode = (mode or "replace_raw_tail").lower()
    if mode == "replace_raw_tail":
        return replace_raw_tail(candidate, reference, distance)
    return list(candidate)


def tail_deviation(candidate, reference, distance):
    """Symmetric max deviation between candidate and reference tail."""
    if len(candidate) < 2 or len(reference) < 2:
        return float("inf")
    cand_tail = candidate[_tail_start_index(candidate, distance):]
    ref_tail = reference[_tail_start_index(reference, distance):]
    if not cand_tail or not ref_tail:
        return float("inf")
    cand_to_ref = max(_point_to_polyline_distance(p, ref_tail) for p in cand_tail)
    ref_to_cand = max(_point_to_polyline_distance(p, cand_tail) for p in ref_tail)
    return max(cand_to_ref, ref_to_cand)


def tail_heading_error_deg(candidate, reference):
    """Absolute terminal heading mismatch in degrees."""
    if len(candidate) < 2 or len(reference) < 2:
        return float("inf")
    cyaw = math.atan2(candidate[-1][1] - candidate[-2][1], candidate[-1][0] - candidate[-2][0])
    ryaw = math.atan2(reference[-1][1] - reference[-2][1], reference[-1][0] - reference[-2][0])
    err = math.atan2(math.sin(cyaw - ryaw), math.cos(cyaw - ryaw))
    return abs(math.degrees(err))
