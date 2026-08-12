#!/usr/bin/env python3
"""Synthetic-verified S5A1 ROS1 signal extractor pinned to reader v4.

Reader v4 normalizes through reader v2 and finally delegates semantic message
decoding to reader v1.  This module therefore installs its temporary capture
hooks on that exact reader-v1 module, verifies the complete delegation graph,
and restores every hook in ``finally``.  Motion rows contain only odometry
header time and pose; clock/TF are alignment witnesses and all command, path,
height and status topics are QC-only counters.

The module performs no path selection, output write, ROS execution, GPU use or
network access.  ``self-check`` reads only frozen source-code parents.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v1 as reader_core
import r8_liquid_ros1_bag_v2_reader_v4 as reader_v4
import r8_liquid_s5a1_ros1_signal_extractor_v1 as extractor_v1


sys.dont_write_bytecode = True
READER_CORE_PATH = MODULE_DIR / "r8_liquid_ros1_bag_v2_reader_v1.py"
READER_CORE_SHA256 = "bc3975ba446e22097c399e10a8f6c4e60b4f8e78c5c7d613394028daa76c94fc"
READER_V4_PATH = MODULE_DIR / "r8_liquid_ros1_bag_v2_reader_v4.py"
READER_V4_SHA256 = "dfb4f075504eef25753752179a2d8cf8f5585f2fbeb9260254f9b411d77f1b7b"
EXTRACTOR_V1_PATH = MODULE_DIR / "r8_liquid_s5a1_ros1_signal_extractor_v1.py"
EXTRACTOR_V1_SHA256 = "2ae263a2efa9ebc68d1fceb22201e5297db3a96df26002597d0f0a6ba60ad4bf"
BRIDGE_PATH = extractor_v1.BRIDGE_PATH
BRIDGE_SHA256 = extractor_v1.BRIDGE_SHA256
QC_ONLY_TOPICS = extractor_v1.QC_ONLY_TOPICS
SignalExtractionError = extractor_v1.SignalExtractionError
bridge = extractor_v1.bridge

MOTION_FIELDS = (
    "odom_header_t_ns", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
)
MOTION_SIGNALS = (
    "/odom.header.stamp",
    "/odom.pose.pose.position",
    "/odom.pose.pose.orientation",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _assert_reader_hook_topology() -> None:
    """Prove all reader-v4 semantic paths end at this reader-v1 object."""

    owners = (
        reader_v4.v1,
        reader_v4.v2.v1,
        reader_v4.v3.v1,
        reader_v4.v3.v2.v1,
    )
    if any(owner is not reader_core for owner in owners):
        raise SignalExtractionError("reader v4 semantic-hook topology drifted")
    for name in ("_Message", "_decode_registered_payload"):
        if not hasattr(reader_core, name):
            raise SignalExtractionError(f"reader v1 semantic hook is absent: {name}")


def _integer_ns(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SignalExtractionError(f"{label} is not a non-negative integer ns value")
    return value


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SignalExtractionError(f"{label} is not numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise SignalExtractionError(f"{label} is non-finite")
    return converted


def _validate_time_axis(
    rows: Sequence[Mapping[str, Any]], field: str, label: str, *, strict: bool
) -> None:
    previous: int | None = None
    for index, row in enumerate(rows):
        current = _integer_ns(row.get(field), f"{label}[{index}].{field}")
        if previous is not None and (current <= previous if strict else current < previous):
            relation = "strictly increasing" if strict else "monotonic"
            raise SignalExtractionError(f"{label} {field} is not {relation}")
        previous = current


def _validate_extracted(
    summary: Mapping[str, Any],
    odom: Sequence[Mapping[str, Any]],
    clocks: Sequence[Mapping[str, Any]],
    transforms: Sequence[Mapping[str, Any]],
    qc_counts: Mapping[str, int],
) -> dict[str, Any]:
    if not odom:
        raise SignalExtractionError("odom motion source is empty")
    anomalies = summary.get("anomalies")
    if not isinstance(anomalies, list) or anomalies:
        raise SignalExtractionError(f"reader time anomaly is not admissible: {anomalies!r}")
    _validate_time_axis(odom, "odom_header_t_ns", "odom", strict=True)
    _validate_time_axis(odom, "bag_record_t_ns", "odom record", strict=False)
    _validate_time_axis(clocks, "clock_time_ns", "clock", strict=True)
    _validate_time_axis(clocks, "bag_record_t_ns", "clock record", strict=False)

    for index, row in enumerate(odom):
        if row.get("frame_id") != "odom" or row.get("child_frame_id") != "base_footprint":
            raise SignalExtractionError(f"odom frame contract differs at index {index}")
        for field in MOTION_FIELDS[1:]:
            _finite(row.get(field), f"odom[{index}].{field}")
        bridge.normalize_quaternion(tuple(row[field] for field in ("qx", "qy", "qz", "qw")))
    for index, row in enumerate(transforms):
        _integer_ns(row.get("stamp_ns"), f"transform[{index}].stamp_ns")
        _integer_ns(row.get("bag_record_t_ns"), f"transform[{index}].bag_record_t_ns")
        for field in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"):
            _finite(row.get(field), f"transform[{index}].{field}")
        bridge.normalize_quaternion(tuple(row[field] for field in ("qx", "qy", "qz", "qw")))
    if set(qc_counts) - set(QC_ONLY_TOPICS):
        raise SignalExtractionError("non-QC topic entered the QC-only census")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in qc_counts.values()):
        raise SignalExtractionError("QC-only count is not a positive integer")
    return bridge.validate_odom_only_provenance(
        primary_time_source="/odom.header.stamp",
        pose_source="/odom.pose.pose",
        consumed_motion_signals=MOTION_SIGNALS,
    )


def extract_bag_bytes(data: bytes, *, limits: Any | None = None) -> dict[str, Any]:
    """Parse one bounded byte string while capturing only approved signals."""

    if not isinstance(data, bytes):
        raise SignalExtractionError("bag input must be immutable bytes")
    _assert_reader_hook_topology()
    selected_limits = limits or reader_v4.ReaderLimits()
    record_times: dict[int, int] = {}
    odom: list[dict[str, Any]] = []
    clocks: list[dict[str, int]] = []
    transforms: list[dict[str, Any]] = []
    qc_counts: dict[str, int] = {}
    original_message = reader_core._Message
    original_decoder = reader_core._decode_registered_payload
    original_helper_reader = extractor_v1.reader

    def capture_message(*args: Any):
        message = original_message(*args)
        payload_identity = id(message.payload)
        if payload_identity in record_times:
            raise SignalExtractionError("message payload identity was reused")
        record_times[payload_identity] = message.record_time_ns
        return message

    def capture_decoder(
        topic: str, message_type: str, payload: memoryview, active_limits: Any
    ):
        record_time_ns = record_times.get(id(payload))
        if record_time_ns is None:
            raise SignalExtractionError("message/decoder provenance association failed")
        if topic == "/odom" and message_type == "nav_msgs/Odometry":
            row = extractor_v1._full_odometry(payload, active_limits)
            row["bag_record_t_ns"] = record_time_ns
            odom.append(row)
        elif topic == "/clock" and message_type == "rosgraph_msgs/Clock":
            clocks.append({
                "bag_record_t_ns": record_time_ns,
                "clock_time_ns": reader_core._decode_clock(payload, active_limits),
            })
        elif topic in {"/tf", "/tf_static"} and message_type == "tf2_msgs/TFMessage":
            for transform in extractor_v1._full_tf(payload, active_limits):
                transform["bag_record_t_ns"] = record_time_ns
                transform["topic"] = topic
                transforms.append(transform)
        elif topic in QC_ONLY_TOPICS:
            qc_counts[topic] = qc_counts.get(topic, 0) + 1
        return original_decoder(topic, message_type, payload, active_limits)

    reader_core._Message = capture_message
    reader_core._decode_registered_payload = capture_decoder
    extractor_v1.reader = reader_core
    try:
        summary = reader_v4.parse_bag_v2(data, limits=selected_limits)
    finally:
        reader_core._Message = original_message
        reader_core._decode_registered_payload = original_decoder
        extractor_v1.reader = original_helper_reader

    if len(odom) != summary["odom"]["message_count"]:
        raise SignalExtractionError("decoded odom count differs from reader census")
    if len(clocks) != summary["clock"]["message_count"]:
        raise SignalExtractionError("decoded clock count differs from reader census")
    provenance = _validate_extracted(summary, odom, clocks, transforms, qc_counts)
    return {
        "reader_summary": summary,
        "odom": odom,
        "clock": clocks,
        "transforms": transforms,
        "qc_only_topic_counts": qc_counts,
        "motion_provenance": provenance,
        "reader_sha256": READER_V4_SHA256,
        "source_bag_executed": False,
        "ros_started": False,
        "optional_bag_read": False,
    }


def build_motion_samples(
    extracted: Mapping[str, Any], *, maximum_gap_s: float
) -> tuple[Any, ...]:
    """Build canonical relative motion from the closed odom field set only."""

    rows = extracted.get("odom")
    if not isinstance(rows, list):
        raise SignalExtractionError("extracted odom rows are absent")
    provenance = extracted.get("motion_provenance")
    if not isinstance(provenance, Mapping) or provenance.get("forbidden_signal_consumed") is not False:
        raise SignalExtractionError("odom-only motion provenance is absent")
    poses = [
        bridge.OdomPose(
            header_t_s=_integer_ns(row.get("odom_header_t_ns"), f"odom[{index}].time") / 1e9,
            x_m=_finite(row.get("x_m"), f"odom[{index}].x_m"),
            y_m=_finite(row.get("y_m"), f"odom[{index}].y_m"),
            z_m=_finite(row.get("z_m"), f"odom[{index}].z_m"),
            qx=_finite(row.get("qx"), f"odom[{index}].qx"),
            qy=_finite(row.get("qy"), f"odom[{index}].qy"),
            qz=_finite(row.get("qz"), f"odom[{index}].qz"),
            qw=_finite(row.get("qw"), f"odom[{index}].qw"),
        )
        for index, row in enumerate(rows)
    ]
    return bridge.make_relative_motion(poses, maximum_gap_s=maximum_gap_s)


nearest_clock_alignment = extractor_v1.nearest_clock_alignment
tf_cross_check = extractor_v1.tf_cross_check


def self_check() -> dict[str, Any]:
    expected = {
        READER_CORE_PATH: READER_CORE_SHA256,
        READER_V4_PATH: READER_V4_SHA256,
        EXTRACTOR_V1_PATH: EXTRACTOR_V1_SHA256,
        BRIDGE_PATH: BRIDGE_SHA256,
    }
    for path, digest in expected.items():
        if _sha256(path) != digest:
            raise SignalExtractionError(f"extractor v3 parent hash drifted: {path}")
    _assert_reader_hook_topology()
    return {
        "status": "PASS_S5A1_ROS1_SIGNAL_EXTRACTOR_V3_STATIC_CONTRACT",
        "reader_revision": "v4",
        "semantic_hook_owner": "r8_liquid_ros1_bag_v2_reader_v1",
        "hook_finally_restore": True,
        "motion_source": "ODOM_HEADER_STAMP_AND_POSE_ONLY",
        "qc_only_topic_count": len(QC_ONLY_TOPICS),
        "real_bag_read": False,
        "optional_bag_read": False,
        "external_write": False,
        "ros_started": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except (SignalExtractionError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "SignalExtractionError", "extract_bag_bytes", "build_motion_samples",
    "nearest_clock_alignment", "tf_cross_check", "self_check",
    "READER_V4_PATH", "READER_V4_SHA256",
]
