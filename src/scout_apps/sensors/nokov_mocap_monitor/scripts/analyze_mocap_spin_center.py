#!/usr/bin/env python3
"""Fit a Tracker0 spin centre and the tracker-to-centre planar offset.

The fit is deliberately geometric.  It uses receive time only to crop the
recorded ``spin_ccw_hold`` and ``spin_cw_hold`` intervals; position and yaw
from the same mocap message are then fitted without differentiation::

    p_tracker = c_world + R(yaw_tracker) @ r_center_to_tracker

The two directions may have different world centres because the platform or
the marker can drift between segments.  They must agree on the body-fixed
offset.  This script never writes ROS parameters or controller configuration.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_MOCAP_TOPIC = "/vrpn_client_node/Tracker0/pose"
DEFAULT_SEGMENT_TOPIC = "/mocap_imu_calib/segment"
SPIN_LABELS = ("spin_ccw_hold", "spin_cw_hold")


def quaternion_yaw(quaternion) -> float:
    """Return planar yaw from a ROS quaternion-like object."""
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion norm is not positive and finite")
    x, y, z, w = (
        quaternion.x / norm,
        quaternion.y / norm,
        quaternion.z / norm,
        quaternion.w / norm,
    )
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def parse_segment_events(
    events: Iterable[Tuple[float, str]], labels: Sequence[str] = SPIN_LABELS
) -> Tuple[Dict[str, Tuple[float, float]], List[str]]:
    """Pair START/END markers using their bag receive times.

    The event payload may contain command metadata after the second pipe.  A
    malformed or incomplete pair is reported as an issue instead of being
    silently converted into a guessed interval.
    """
    wanted = set(labels)
    starts: Dict[str, List[float]] = {label: [] for label in labels}
    pairs: Dict[str, Tuple[float, float]] = {}
    issues: List[str] = []
    for stamp, payload in sorted(events, key=lambda item: float(item[0])):
        fields = str(payload).split("|")
        if len(fields) < 2 or fields[1] not in wanted:
            continue
        label = fields[1]
        kind = fields[0].upper()
        if kind == "START":
            starts[label].append(float(stamp))
        elif kind == "END":
            if not starts[label]:
                issues.append("{} has END without START".format(label))
                continue
            start = starts[label].pop(0)
            end = float(stamp)
            if end <= start:
                issues.append("{} has non-positive interval".format(label))
            elif label in pairs:
                issues.append("{} has more than one complete interval".format(label))
            else:
                pairs[label] = (start, end)
    for label in labels:
        if starts[label]:
            issues.append("{} has unmatched START".format(label))
        if label not in pairs:
            issues.append("{} interval is missing".format(label))
    return pairs, issues


def _fit_offset(xy: np.ndarray, yaw: np.ndarray) -> Dict[str, object]:
    """Fit ``c_world`` and ``r_center_to_tracker`` by linear least squares."""
    n = int(len(yaw))
    c = np.cos(yaw)
    s = np.sin(yaw)
    design = np.zeros((2 * n, 4), dtype=float)
    design[:n, 0] = 1.0
    design[n:, 1] = 1.0
    design[:n, 2] = c
    design[:n, 3] = -s
    design[n:, 2] = s
    design[n:, 3] = c
    observation = np.concatenate((xy[:, 0], xy[:, 1]))
    solution, _, rank, _ = np.linalg.lstsq(design, observation, rcond=None)
    prediction = design @ solution
    residual = np.column_stack(
        (prediction[:n] - observation[:n], prediction[n:] - observation[n:])
    )
    residual_norm = np.linalg.norm(residual, axis=1)
    return {
        "center_world_m": [float(solution[0]), float(solution[1])],
        "r_center_to_tracker_tracker_m": [float(solution[2]), float(solution[3])],
        "residual_m": residual_norm,
        "rank": int(rank),
    }


def _angle_uniform_indices(yaw: np.ndarray, count: int) -> np.ndarray:
    """Select approximately uniform yaw samples without receive-density weight."""
    if len(yaw) == 0:
        return np.empty(0, dtype=int)
    direction = 1.0 if yaw[-1] >= yaw[0] else -1.0
    progress = direction * (yaw - yaw[0])
    order = np.argsort(progress, kind="stable")
    progress = progress[order]
    if progress[-1] <= 0.0:
        return np.empty(0, dtype=int)
    targets = np.linspace(0.0, progress[-1], max(2, int(count)))
    selected: List[int] = []
    for target in targets:
        index = int(np.searchsorted(progress, target, side="left"))
        candidates = [max(0, min(len(order) - 1, index))]
        if index > 0:
            candidates.append(index - 1)
        nearest = min(candidates, key=lambda candidate: abs(progress[candidate] - target))
        selected.append(int(order[nearest]))
    return np.asarray(sorted(set(selected)), dtype=int)


def _metrics(fit: Dict[str, object]) -> Dict[str, float]:
    residual = np.asarray(fit["residual_m"], dtype=float)
    return {
        "fit_rmse_mm": float(1000.0 * np.sqrt(np.mean(residual * residual))),
        "fit_p95_mm": float(1000.0 * np.percentile(residual, 95.0)),
        "fit_peak_mm": float(1000.0 * np.max(residual)),
    }


def fit_direction(
    pose: np.ndarray,
    interval: Tuple[float, float],
    *,
    trim_sec: float = 1.0,
    angle_samples: int = 360,
    min_yaw_coverage_deg: float = 180.0,
    min_samples: int = 60,
    min_half_samples: int = 20,
    max_fit_rmse_mm: float = 3.0,
    max_fit_p95_mm: float = 8.0,
    max_fit_peak_mm: float = 15.0,
    max_center_half_drift_mm: float = 20.0,
    max_half_offset_delta_mm: float = 5.0,
    expected_direction: Optional[int] = None,
    max_reversal_fraction: float = 0.05,
) -> Dict[str, object]:
    """Fit one spin interval.

    ``pose`` columns are receive time, header time, x, y, unwrapped yaw.
    """
    start, end = map(float, interval)
    lo, hi = start + float(trim_sec), end - float(trim_sec)
    result: Dict[str, object] = {
        "interval_receive_sec": [start, end],
        "trimmed_receive_sec": [lo, hi],
        "status": "INSUFFICIENT_DATA",
        "rejection_reasons": [],
    }
    if pose.ndim != 2 or pose.shape[1] < 5 or not np.all(np.isfinite(pose[:, :5])):
        result["rejection_reasons"] = ["pose sequence contains invalid numeric data"]
        return result
    if hi <= lo:
        result["rejection_reasons"] = ["trimmed interval is empty"]
        return result
    selected = pose[(pose[:, 0] >= lo) & (pose[:, 0] <= hi)]
    if len(selected) < min_samples:
        result["sample_count_raw"] = int(len(selected))
        result["rejection_reasons"] = ["too few samples after trimming"]
        return result
    # Keep rosbag/receive order. Header time is metadata here; silently
    # sorting by it could hide transport or recorder ordering failures.
    if np.any(np.diff(selected[:, 0]) <= 0.0):
        result["rejection_reasons"] = ["non-increasing receive time in pose sequence"]
        return result
    yaw = np.unwrap(selected[:, 4])
    selected = selected.copy()
    selected[:, 4] = yaw
    if expected_direction not in (None, -1, 1):
        raise ValueError("expected_direction must be -1, 1, or None")
    net_yaw_deg = float(np.rad2deg(yaw[-1] - yaw[0]))
    travel_direction = 1.0 if yaw[-1] >= yaw[0] else -1.0
    progress = travel_direction * (yaw - yaw[0])
    reversal_deg = float(
        np.rad2deg(np.sum(np.maximum(0.0, -np.diff(progress))))
    )
    reversal_fraction = reversal_deg / max(abs(net_yaw_deg), 1.0e-12)
    coverage_deg = float(np.rad2deg(np.max(yaw) - np.min(yaw)))
    result["sample_count_raw"] = int(len(selected))
    result["net_yaw_change_deg"] = net_yaw_deg
    result["yaw_coverage_deg"] = coverage_deg
    result["yaw_reversal_deg"] = reversal_deg
    result["yaw_reversal_fraction"] = reversal_fraction
    result["expected_direction"] = expected_direction
    initial_reasons: List[str] = []
    if expected_direction is not None and np.sign(net_yaw_deg) != expected_direction:
        initial_reasons.append("yaw net direction does not match expected spin direction")
    if abs(net_yaw_deg) < float(min_yaw_coverage_deg):
        initial_reasons.append("net yaw coverage below threshold")
    if reversal_fraction > float(max_reversal_fraction):
        initial_reasons.append("yaw reversal/backtracking exceeds threshold")
    if initial_reasons:
        result["rejection_reasons"] = initial_reasons
        return result
    indices = _angle_uniform_indices(yaw, angle_samples)
    if len(indices) < min_samples:
        result["sample_count_angle_uniform"] = int(len(indices))
        result["rejection_reasons"] = ["too few angle-uniform samples"]
        return result
    sampled = selected[indices]
    fit = _fit_offset(sampled[:, 2:4], sampled[:, 4])
    result["sample_count_angle_uniform"] = int(len(sampled))
    result.update({key: value for key, value in fit.items() if key != "residual_m"})
    result.update(_metrics(fit))

    progress = travel_direction * (sampled[:, 4] - yaw[0])
    midpoint = 0.5 * (float(np.min(progress)) + float(np.max(progress)))
    first = sampled[progress <= midpoint]
    second = sampled[progress >= midpoint]
    reasons: List[str] = []
    if int(fit["rank"]) < 4:
        reasons.append("least-squares design is rank deficient")
    if result["fit_rmse_mm"] > max_fit_rmse_mm:
        reasons.append("fit RMSE exceeds threshold")
    if result["fit_p95_mm"] > max_fit_p95_mm:
        reasons.append("fit P95 residual exceeds threshold")
    if result["fit_peak_mm"] > max_fit_peak_mm:
        reasons.append("fit peak residual exceeds threshold")
    if len(first) < min_half_samples or len(second) < min_half_samples:
        reasons.append("too few samples in one angle half")
    else:
        first_fit = _fit_offset(first[:, 2:4], first[:, 4])
        second_fit = _fit_offset(second[:, 2:4], second[:, 4])
        first_r = np.asarray(first_fit["r_center_to_tracker_tracker_m"])
        second_r = np.asarray(second_fit["r_center_to_tracker_tracker_m"])
        first_c = np.asarray(first_fit["center_world_m"])
        second_c = np.asarray(second_fit["center_world_m"])
        result["half_sample_counts"] = [int(len(first)), int(len(second))]
        result["half_offset_delta_mm"] = float(1000.0 * np.linalg.norm(first_r - second_r))
        result["center_half_drift_mm"] = float(1000.0 * np.linalg.norm(first_c - second_c))
        result["first_half_offset_tracker_m"] = first_r.tolist()
        result["second_half_offset_tracker_m"] = second_r.tolist()
        result["first_half_center_world_m"] = first_c.tolist()
        result["second_half_center_world_m"] = second_c.tolist()
        if result["half_offset_delta_mm"] > max_half_offset_delta_mm:
            reasons.append("half-spin offset inconsistency exceeds threshold")
        if result["center_half_drift_mm"] > max_center_half_drift_mm:
            reasons.append("world-centre half-spin drift exceeds threshold")
    result["rejection_reasons"] = reasons
    result["status"] = "REJECT" if reasons else "PASS"
    return result


def compare_directions(
    fits: Dict[str, Dict[str, object]], max_direction_offset_delta_mm: float
) -> Tuple[str, Dict[str, object]]:
    """Apply cross-direction offset consistency without comparing world centres."""
    report: Dict[str, object] = {}
    if any(fits[label].get("status") == "REJECT" for label in SPIN_LABELS):
        return "REJECT", report
    if any(fits[label].get("status") != "PASS" for label in SPIN_LABELS):
        return "INSUFFICIENT_DATA", report
    ccw = np.asarray(fits["spin_ccw_hold"]["r_center_to_tracker_tracker_m"])
    cw = np.asarray(fits["spin_cw_hold"]["r_center_to_tracker_tracker_m"])
    delta = float(1000.0 * np.linalg.norm(ccw - cw))
    report["direction_offset_delta_mm"] = delta
    report["world_center_delta_m"] = float(
        np.linalg.norm(
            np.asarray(fits["spin_ccw_hold"]["center_world_m"])
            - np.asarray(fits["spin_cw_hold"]["center_world_m"])
        )
    )
    report["world_center_equality_required"] = False
    if delta > max_direction_offset_delta_mm:
        return "REJECT", report
    return "PASS", report


def load_bag(
    path: Path, mocap_topic: str, segment_topic: str
) -> Tuple[np.ndarray, List[Tuple[float, str]], List[str]]:
    """Load only pose and segment messages; receive time is retained for cropping."""
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is unavailable: {}".format(exc))
    rows: List[Tuple[float, float, float, float, float]] = []
    events: List[Tuple[float, str]] = []
    frame_ids = set()
    with rosbag.Bag(str(path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        if mocap_topic not in available:
            raise RuntimeError("bag is missing mocap topic {}".format(mocap_topic))
        for topic, message, bag_stamp in bag.read_messages(topics=[mocap_topic, segment_topic]):
            receive = float(bag_stamp.to_sec())
            if topic == segment_topic:
                events.append((receive, str(message.data)))
                continue
            if not math.isfinite(receive):
                raise RuntimeError("non-finite bag receive time in mocap sequence")
            raw_header = float(message.header.stamp.to_sec())
            if not math.isfinite(raw_header):
                raise RuntimeError("non-finite mocap header time")
            header = raw_header if raw_header > 0.0 else receive
            position = message.pose.position
            quaternion = message.pose.orientation
            values = (
                header,
                float(position.x),
                float(position.y),
                float(position.z),
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            )
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("non-finite mocap pose value")
            quaternion_norm = math.sqrt(sum(value * value for value in values[4:]))
            if not 0.95 <= quaternion_norm <= 1.05:
                raise RuntimeError(
                    "invalid mocap quaternion norm {:.9g}".format(quaternion_norm)
                )
            frame_ids.add(str(getattr(message.header, "frame_id", "")))
            rows.append(
                (
                    receive,
                    header,
                    float(position.x),
                    float(position.y),
                    quaternion_yaw(quaternion),
                )
            )
    if not rows:
        raise RuntimeError("no mocap samples on {}".format(mocap_topic))
    pose = np.asarray(rows, dtype=float)
    if len(frame_ids) > 1:
        raise RuntimeError("mocap frame_id changes within bag: {}".format(sorted(frame_ids)))
    if np.any(~np.isfinite(pose)):
        raise RuntimeError("non-finite mocap sequence after loading")
    if np.any(np.diff(pose[:, 0]) <= 0.0):
        raise RuntimeError("mocap bag receive times are not strictly increasing")
    if np.any(np.diff(pose[:, 1]) < 0.0):
        raise RuntimeError("mocap header times regress in bag order")
    return pose, events, sorted(frame_ids)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--mocap-topic", default=DEFAULT_MOCAP_TOPIC)
    parser.add_argument("--segment-topic", default=DEFAULT_SEGMENT_TOPIC)
    parser.add_argument("--trim-sec", type=float, default=1.0)
    parser.add_argument("--angle-samples", type=int, default=360)
    parser.add_argument("--min-yaw-coverage-deg", type=float, default=180.0)
    parser.add_argument("--min-samples", type=int, default=60)
    parser.add_argument("--min-half-samples", type=int, default=20)
    parser.add_argument("--max-fit-rmse-mm", type=float, default=3.0)
    parser.add_argument("--max-fit-p95-mm", type=float, default=8.0)
    parser.add_argument("--max-fit-peak-mm", type=float, default=15.0)
    parser.add_argument("--max-center-half-drift-mm", type=float, default=20.0)
    parser.add_argument("--max-half-offset-delta-mm", type=float, default=5.0)
    parser.add_argument("--max-direction-offset-delta-mm", type=float, default=5.0)
    parser.add_argument("--max-reversal-fraction", type=float, default=0.05)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    report: Dict[str, object] = {
        "schema_version": 1,
        "bag": str(args.bag),
        "mocap_topic": args.mocap_topic,
        "segment_topic": args.segment_topic,
        "parameters": {
            key: (str(value) if isinstance(value, Path) else value)
            for key, value in vars(args).items()
        },
        "method": {
            "fit": "p_tracker = c_world + R(yaw_tracker) @ r_center_to_tracker",
            "offset_frame": "Tracker yaw horizontal projection axes",
            "interpretation": (
                "valid rotation-centre/Tracker offset only; not geometric centre, "
                "base_link, cup, or a controller/slosh external"
            ),
            "receive_time_role": "coarse segment cropping only",
            "geometry_order": "rosbag receive order; no header-time reordering",
        },
        "status": "INSUFFICIENT_DATA",
    }
    try:
        pose, events, frame_ids = load_bag(args.bag, args.mocap_topic, args.segment_topic)
        report["mocap_frame_ids"] = frame_ids
        intervals, event_issues = parse_segment_events(events)
        report["segment_intervals_receive_sec"] = {
            label: [float(bounds[0]), float(bounds[1])]
            for label, bounds in intervals.items()
        }
        report["segment_event_issues"] = event_issues
        fits: Dict[str, Dict[str, object]] = {}
        for label in SPIN_LABELS:
            if label not in intervals:
                fits[label] = {"status": "INSUFFICIENT_DATA", "rejection_reasons": ["interval missing"]}
                continue
            fits[label] = fit_direction(
                pose,
                intervals[label],
                trim_sec=args.trim_sec,
                angle_samples=args.angle_samples,
                min_yaw_coverage_deg=args.min_yaw_coverage_deg,
                min_samples=args.min_samples,
                min_half_samples=args.min_half_samples,
                max_fit_rmse_mm=args.max_fit_rmse_mm,
                max_fit_p95_mm=args.max_fit_p95_mm,
                max_fit_peak_mm=args.max_fit_peak_mm,
                max_center_half_drift_mm=args.max_center_half_drift_mm,
                max_half_offset_delta_mm=args.max_half_offset_delta_mm,
                expected_direction=1 if label == "spin_ccw_hold" else -1,
                max_reversal_fraction=args.max_reversal_fraction,
            )
        report["fits"] = fits
        direction_status, direction_report = compare_directions(fits, args.max_direction_offset_delta_mm)
        report["direction_consistency"] = direction_report
        report["direction_consistency_status"] = direction_status
        if event_issues:
            report["status"] = "FAIL"
            report["failure_class"] = "EVENT_INVALID"
        elif direction_status == "PASS":
            report["status"] = "PASS"
            report["failure_class"] = None
        elif direction_status == "REJECT":
            report["status"] = "FAIL"
            report["failure_class"] = "REJECT"
        else:
            # A report is still produced for an incomplete run, but it is a
            # failed quality gate and must not be consumed as a calibration.
            report["status"] = "FAIL"
            report["failure_class"] = "INSUFFICIENT_DATA"
        if report["status"] == "PASS":
            offsets = [
                np.asarray(fits[label]["r_center_to_tracker_tracker_m"], dtype=float)
                for label in SPIN_LABELS
            ]
            report["r_center_to_tracker_tracker_m_average"] = np.mean(offsets, axis=0).tolist()
            report["average_offset_is_valid_candidate"] = True
        else:
            report["average_offset_is_valid_candidate"] = False
    except Exception as exc:  # Always leave a reviewable report for offline runs.
        report["status"] = "ERROR"
        report["error"] = "{}: {}".format(type(exc).__name__, exc)
    report.setdefault("average_offset_is_valid_candidate", False)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "report": str(args.report)}, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
