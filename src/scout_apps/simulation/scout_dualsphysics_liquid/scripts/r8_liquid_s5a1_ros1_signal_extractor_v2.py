#!/usr/bin/env python3
"""S5A1 signal extractor v2 pinned to the padded-header ROS1 reader v4.

Only odometry pose/header time may drive motion.  Clock and TF remain
alignment witnesses; every other selected topic is counted for QC only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v1 as reader_core
import r8_liquid_ros1_bag_v2_reader_v4 as reader_v4
import r8_liquid_s5a1_ros1_signal_extractor_v1 as v1


sys.dont_write_bytecode = True
READER_PATH = MODULE_DIR / "r8_liquid_ros1_bag_v2_reader_v4.py"
READER_SHA256 = "dfb4f075504eef25753752179a2d8cf8f5585f2fbeb9260254f9b411d77f1b7b"
BRIDGE_PATH = v1.BRIDGE_PATH
BRIDGE_SHA256 = v1.BRIDGE_SHA256
QC_ONLY_TOPICS = v1.QC_ONLY_TOPICS
SignalExtractionError = v1.SignalExtractionError
bridge = v1.bridge

# V1's payload helpers intentionally use private reader primitives.  Reader
# v4 normalizes through reader v2 but delegates the semantic parse and message
# decoding to reader v1, so bind and patch the exact module that owns those
# hooks.
v1.reader = reader_core


def extract_bag_bytes(data: bytes, *, limits: Any | None = None) -> dict[str, Any]:
    selected_limits = limits or reader_v4.ReaderLimits()
    record_times: dict[int, int] = {}
    odom: list[dict[str, Any]] = []
    clocks: list[dict[str, int]] = []
    transforms: list[dict[str, Any]] = []
    qc_counts: dict[str, int] = {}
    original_message = reader_core._Message
    original_decoder = reader_core._decode_registered_payload

    def capture_message(*args: Any):
        message = original_message(*args)
        record_times[id(message.payload)] = message.record_time_ns
        return message

    def capture_decoder(
        topic: str, message_type: str, payload: memoryview, active_limits: Any
    ):
        record_time_ns = record_times.get(id(payload))
        if record_time_ns is None:
            raise SignalExtractionError(
                "message/decoder provenance association failed"
            )
        if topic == "/odom" and message_type == "nav_msgs/Odometry":
            row = v1._full_odometry(payload, active_limits)
            row["bag_record_t_ns"] = record_time_ns
            odom.append(row)
        elif topic == "/clock" and message_type == "rosgraph_msgs/Clock":
            clocks.append({
                "bag_record_t_ns": record_time_ns,
                "clock_time_ns": reader_core._decode_clock(payload, active_limits),
            })
        elif topic in {"/tf", "/tf_static"} and message_type == "tf2_msgs/TFMessage":
            for transform in v1._full_tf(payload, active_limits):
                transform["bag_record_t_ns"] = record_time_ns
                transform["topic"] = topic
                transforms.append(transform)
        elif topic in QC_ONLY_TOPICS:
            qc_counts[topic] = qc_counts.get(topic, 0) + 1
        return original_decoder(topic, message_type, payload, active_limits)

    reader_core._Message = capture_message
    reader_core._decode_registered_payload = capture_decoder
    try:
        summary = reader_v4.parse_bag_v2(data, limits=selected_limits)
    finally:
        reader_core._Message = original_message
        reader_core._decode_registered_payload = original_decoder
    if len(odom) != summary["odom"]["message_count"]:
        raise SignalExtractionError("decoded odom count differs from reader census")
    if len(clocks) != summary["clock"]["message_count"]:
        raise SignalExtractionError("decoded clock count differs from reader census")
    return {
        "reader_summary": summary,
        "odom": odom,
        "clock": clocks,
        "transforms": transforms,
        "qc_only_topic_counts": qc_counts,
        "reader_sha256": READER_SHA256,
        "source_bag_executed": False,
        "ros_started": False,
        "optional_bag_read": False,
    }


nearest_clock_alignment = v1.nearest_clock_alignment
tf_cross_check = v1.tf_cross_check


def read_exact_primary(
    path: Path,
    *,
    expected_size: int,
    expected_mode: int,
    expected_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    data, before, after = reader_v4.read_regular_file(
        path,
        limits=reader_v4.ReaderLimits(),
        expected_sha256=expected_sha256,
        expected_size_bytes=expected_size,
        expected_mode=expected_mode,
    )
    return extract_bag_bytes(data), before, after


def self_check() -> dict[str, Any]:
    observed_reader = hashlib.sha256(READER_PATH.read_bytes()).hexdigest()
    observed_bridge = hashlib.sha256(BRIDGE_PATH.read_bytes()).hexdigest()
    if observed_reader != READER_SHA256 or observed_bridge != BRIDGE_SHA256:
        raise SignalExtractionError("extractor v2 parent hash drifted")
    return {
        "status": "PASS_S5A1_SIGNAL_EXTRACTOR_V2_STATIC_CONTRACT",
        "reader_revision": "v4",
        "reader_sha256": observed_reader,
        "motion_bridge_sha256": observed_bridge,
        "motion_source": "ODOM_POSE_AND_HEADER_STAMP_ONLY",
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
    "SignalExtractionError", "extract_bag_bytes", "nearest_clock_alignment",
    "tf_cross_check", "read_exact_primary", "READER_PATH", "READER_SHA256",
]
