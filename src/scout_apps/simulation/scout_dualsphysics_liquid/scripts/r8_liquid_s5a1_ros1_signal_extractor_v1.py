#!/usr/bin/env python3
"""Bounded ROS1 signal extractor for the S5A1 one-shot worker.

This revision is byte-bound to the committed ROS1 Bag V2 reader.  It reuses
that reader's complete index/chunk/connection validation, temporarily wraps
its private message/decoder hooks in one process, and emits only odometry,
clock and TF witnesses plus QC-only topic counts.  It never selects a path,
writes a file, starts ROS, or executes bag content.
"""

from __future__ import annotations

import bisect
import hashlib
import importlib.util
import math
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
READER_PATH = ROOT / "scripts/r8_liquid_ros1_bag_v2_reader_v1.py"
READER_SHA256 = "bc3975ba446e22097c399e10a8f6c4e60b4f8e78c5c7d613394028daa76c94fc"
BRIDGE_PATH = ROOT / "scripts/r8_liquid_s5a1_motion_bridge_v1.py"
BRIDGE_SHA256 = "c840f7bb467d5145f55d2e9904b7b8b62781658818ae47f3e95bd139904e2aa1"
QC_ONLY_TOPICS = frozenset({
    "/cmd_vel", "/scout/global_path_fixed", "/scout/goal", "/slosh/height",
    "/spmpc/slosh_height", "/spmpc/status", "/spmpc/debug/effective_config",
    "/spmpc/terminal/debug", "/spmpc/terminal/mode",
})


class SignalExtractionError(ValueError):
    """Reader drift, malformed payload, or alignment mismatch."""


def _load_bound_module(path: Path, expected_sha256: str, module_name: str):
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise SignalExtractionError(f"frozen module hash drifted: {path}")
    specification = importlib.util.spec_from_file_location(module_name, path)
    if specification is None or specification.loader is None:
        raise SignalExtractionError(f"cannot load frozen module: {path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    specification.loader.exec_module(module)
    return module


reader = _load_bound_module(READER_PATH, READER_SHA256, "r8_liquid_s5a1_bound_bag_reader_v1")
bridge = _load_bound_module(BRIDGE_PATH, BRIDGE_SHA256, "r8_liquid_s5a1_bound_motion_bridge_v1")


def _full_odometry(payload: memoryview, limits: Any) -> dict[str, Any]:
    cursor = reader._PayloadCursor(payload, limits, label="nav_msgs/Odometry")
    header_time_ns, frame_id = reader._header(cursor, "header")
    child_frame_id = cursor.string("child_frame_id")
    values = cursor.doubles(85, "pose/twist/covariances")
    quaternion = tuple(values[3:7])
    bridge.normalize_quaternion(quaternion)
    cursor.require_eof()
    return {
        "odom_header_t_ns": header_time_ns, "frame_id": frame_id,
        "child_frame_id": child_frame_id, "x_m": values[0], "y_m": values[1],
        "z_m": values[2], "qx": values[3], "qy": values[4], "qz": values[5],
        "qw": values[6], "vx_m_s": values[43], "vy_m_s": values[44],
        "vz_m_s": values[45], "wx_rad_s": values[46], "wy_rad_s": values[47],
        "wz_rad_s": values[48],
    }


def _full_tf(payload: memoryview, limits: Any) -> list[dict[str, Any]]:
    cursor = reader._PayloadCursor(payload, limits, label="tf2_msgs/TFMessage")
    count = cursor.u32("transforms length")
    if count > limits.maximum_array_count:
        raise SignalExtractionError("TF transform count exceeds bound")
    values = []
    for index in range(count):
        stamp_ns, parent = reader._header(cursor, f"transforms[{index}].header")
        child = cursor.string(f"transforms[{index}].child_frame_id")
        transform = cursor.doubles(7, f"transforms[{index}].transform")
        bridge.normalize_quaternion(transform[3:7])
        values.append({"stamp_ns": stamp_ns, "parent": parent, "child": child,
                       "x_m": transform[0], "y_m": transform[1], "z_m": transform[2],
                       "qx": transform[3], "qy": transform[4], "qz": transform[5], "qw": transform[6]})
    cursor.require_eof()
    return values


def extract_bag_bytes(data: bytes, *, limits: Any | None = None) -> dict[str, Any]:
    selected_limits = limits or reader.ReaderLimits()
    record_times: dict[int, int] = {}
    odom: list[dict[str, Any]] = []
    clocks: list[dict[str, int]] = []
    transforms: list[dict[str, Any]] = []
    qc_counts: dict[str, int] = {}
    original_message = reader._Message
    original_decoder = reader._decode_registered_payload

    def capture_message(*args):
        message = original_message(*args)
        record_times[id(message.payload)] = message.record_time_ns
        return message

    def capture_decoder(topic: str, message_type: str, payload: memoryview, active_limits: Any):
        record_time_ns = record_times.get(id(payload))
        if record_time_ns is None:
            raise SignalExtractionError("message/decoder provenance association failed")
        if topic == "/odom" and message_type == "nav_msgs/Odometry":
            row = _full_odometry(payload, active_limits)
            row["bag_record_t_ns"] = record_time_ns
            odom.append(row)
        elif topic == "/clock" and message_type == "rosgraph_msgs/Clock":
            clocks.append({"bag_record_t_ns": record_time_ns,
                           "clock_time_ns": reader._decode_clock(payload, active_limits)})
        elif topic in {"/tf", "/tf_static"} and message_type == "tf2_msgs/TFMessage":
            for transform in _full_tf(payload, active_limits):
                transform["bag_record_t_ns"] = record_time_ns
                transform["topic"] = topic
                transforms.append(transform)
        elif topic in QC_ONLY_TOPICS:
            qc_counts[topic] = qc_counts.get(topic, 0) + 1
        return original_decoder(topic, message_type, payload, active_limits)

    reader._Message = capture_message
    reader._decode_registered_payload = capture_decoder
    try:
        summary = reader.parse_bag_v2(data, limits=selected_limits)
    finally:
        reader._Message = original_message
        reader._decode_registered_payload = original_decoder
    if len(odom) != summary["odom"]["message_count"]:
        raise SignalExtractionError("decoded odom count differs from reader census")
    if len(clocks) != summary["clock"]["message_count"]:
        raise SignalExtractionError("decoded clock count differs from reader census")
    return {"reader_summary": summary, "odom": odom, "clock": clocks,
            "transforms": transforms, "qc_only_topic_counts": qc_counts,
            "reader_sha256": READER_SHA256, "source_bag_executed": False,
            "ros_started": False, "optional_bag_read": False}


def nearest_clock_alignment(odom: Sequence[Mapping[str, Any]], clocks: Sequence[Mapping[str, int]]) -> list[dict[str, int]]:
    ordered = sorted(int(row["clock_time_ns"]) for row in clocks)
    if not ordered:
        raise SignalExtractionError("clock witness is empty")
    result = []
    for row in odom:
        stamp = int(row["odom_header_t_ns"])
        index = bisect.bisect_left(ordered, stamp)
        candidates = ordered[max(0, index - 1):min(len(ordered), index + 1)]
        nearest = min(candidates, key=lambda value: abs(value - stamp))
        result.append({"odom_header_t_ns": stamp, "nearest_clock_t_ns": nearest,
                       "abs_delta_ns": abs(nearest - stamp)})
    return result


def tf_cross_check(
    odom: Sequence[Mapping[str, Any]], transforms: Sequence[Mapping[str, Any]], *,
    maximum_time_delta_s: float, maximum_position_delta_m: float,
    maximum_angular_delta_rad: float,
) -> dict[str, Any]:
    matching = sorted(
        (row for row in transforms if row["topic"] == "/tf" and row["parent"] == "odom"
         and row["child"] == "base_footprint"), key=lambda row: row["stamp_ns"]
    )
    if not matching:
        raise SignalExtractionError("required odom->base_footprint TF witness is absent")
    stamps = [int(row["stamp_ns"]) for row in matching]
    max_time = max_position = max_angle = 0.0
    for pose in odom:
        stamp = int(pose["odom_header_t_ns"])
        index = bisect.bisect_left(stamps, stamp)
        choices = matching[max(0, index - 1):min(len(matching), index + 1)]
        selected = min(choices, key=lambda row: abs(int(row["stamp_ns"]) - stamp))
        time_delta = abs(int(selected["stamp_ns"]) - stamp) / 1e9
        position_delta = math.sqrt(sum((float(selected[key]) - float(pose[key])) ** 2
                                       for key in ("x_m", "y_m", "z_m")))
        angle_delta = bridge.quaternion_angular_distance(
            tuple(float(selected[key]) for key in ("qx", "qy", "qz", "qw")),
            tuple(float(pose[key]) for key in ("qx", "qy", "qz", "qw")),
        )
        max_time, max_position, max_angle = max(max_time, time_delta), max(max_position, position_delta), max(max_angle, angle_delta)
    if max_time > maximum_time_delta_s or max_position > maximum_position_delta_m or max_angle > maximum_angular_delta_rad:
        raise SignalExtractionError("TF pose witness exceeds frozen mismatch threshold")
    return {"status": "PASS_ALIGNMENT_WITNESS", "odom_frame": "odom",
            "base_frame": "base_footprint", "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT",
            "tf_used_as_motion": False, "sample_count": len(odom),
            "maximum_time_delta_s": max_time, "maximum_position_delta_m": max_position,
            "maximum_angular_delta_rad": max_angle}


def read_exact_primary(path: Path, *, expected_size: int, expected_mode: int, expected_sha256: str) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    data, before, after = reader.read_regular_file(
        path, limits=reader.ReaderLimits(), expected_sha256=expected_sha256,
        expected_size_bytes=expected_size, expected_mode=expected_mode,
    )
    return extract_bag_bytes(data), before, after


__all__ = ["SignalExtractionError", "extract_bag_bytes", "nearest_clock_alignment",
           "tf_cross_check", "read_exact_primary", "READER_PATH", "READER_SHA256"]
