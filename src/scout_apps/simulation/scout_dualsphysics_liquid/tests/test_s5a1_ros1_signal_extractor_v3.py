#!/usr/bin/env python3
"""Synthetic padded ROS1 Bag V2 tests for the S5A1 extractor v3."""

from __future__ import annotations

import hashlib
import math
import struct
import sys
import unittest
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_ros1_bag_v2_reader_v1 as reader_core  # noqa: E402
import r8_liquid_s5a1_ros1_signal_extractor_v1 as extractor_v1  # noqa: E402
import r8_liquid_s5a1_ros1_signal_extractor_v3 as extractor  # noqa: E402


def field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = b"".join(field(name, value) for name, value in fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def ros_time(value_ns: int) -> bytes:
    return struct.pack("<II", value_ns // 1_000_000_000, value_ns % 1_000_000_000)


def string(value: str) -> bytes:
    raw = value.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def header(stamp_ns: int, frame_id: str, *, sequence: int = 0) -> bytes:
    return struct.pack("<I", sequence) + ros_time(stamp_ns) + string(frame_id)


def odom_payload(stamp_ns: int, x_m: float, *, quaternion_w: float = 1.0) -> bytes:
    values = [0.0] * 85
    values[0] = x_m
    values[3:7] = [0.0, 0.0, 0.0, quaternion_w]
    values[43] = 0.25
    return header(stamp_ns, "odom") + string("base_footprint") + struct.pack("<85d", *values)


def tf_payload(stamp_ns: int, x_m: float) -> bytes:
    transform = header(stamp_ns, "odom") + string("base_footprint")
    transform += struct.pack("<7d", x_m, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    return struct.pack("<I", 1) + transform


def path_payload(stamp_ns: int, variant: int) -> bytes:
    pose = header(stamp_ns + variant, "map", sequence=variant)
    pose += struct.pack("<7d", 100.0 + variant, -50.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    return header(stamp_ns, "map") + struct.pack("<I", 1) + pose


def connection(spec: dict[str, Any]) -> bytes:
    data = b"".join([
        field("type", spec["type"].encode("utf-8")),
        field("md5sum", spec["md5"].encode("ascii")),
        field("message_definition", spec["definition"]),
        field("callerid", b"/synthetic_s5a1_v3"),
        field("topic", spec["topic"].encode("utf-8")),
    ])
    return record([
        ("op", bytes([reader_core.OP_CONNECTION])),
        ("conn", struct.pack("<I", spec["id"])),
        ("topic", spec["topic"].encode("utf-8")),
    ], data)


def padded_bag(
    *, qc_variant: int = 0, odom_stamps: tuple[int, int] = (1_000_000_000, 2_000_000_000),
    nonfinite_odom: bool = False,
) -> bytes:
    second_x = float("nan") if nonfinite_odom else 1.25
    specs: list[dict[str, Any]] = [
        {"id": 1, "topic": "/odom", "type": "nav_msgs/Odometry", "definition": b"synthetic odom\n",
         "messages": [(1_100_000_000, odom_payload(odom_stamps[0], 0.25)),
                      (2_100_000_000, odom_payload(odom_stamps[1], second_x))]},
        {"id": 2, "topic": "/clock", "type": "rosgraph_msgs/Clock", "definition": b"synthetic clock\n",
         "messages": [(1_120_000_000, ros_time(odom_stamps[0])),
                      (2_120_000_000, ros_time(odom_stamps[1]))]},
        {"id": 3, "topic": "/tf", "type": "tf2_msgs/TFMessage", "definition": b"synthetic tf\n",
         "messages": [(1_140_000_000, tf_payload(odom_stamps[0], 0.25)),
                      (2_140_000_000, tf_payload(odom_stamps[1], 1.25))]},
        # The following five sources are deliberately conflicting QC witnesses.
        {"id": 4, "topic": "/cmd_vel", "type": "geometry_msgs/Twist", "definition": b"synthetic cmd\n",
         "messages": [(1_300_000_000, struct.pack("<6d", *(float(qc_variant + i) for i in range(6))))]},
        {"id": 5, "topic": "/scout/global_path_fixed", "type": "nav_msgs/Path", "definition": b"synthetic path\n",
         "messages": [(1_320_000_000, path_payload(1_000_000_000, qc_variant))]},
        {"id": 6, "topic": "/slosh/height", "type": "std_msgs/Float32", "definition": b"float32 data\n",
         "messages": [(1_340_000_000, struct.pack("<f", 10.0 + qc_variant))]},
        {"id": 7, "topic": "/spmpc/slosh_height", "type": "std_msgs/Float32", "definition": b"float32 data\n",
         "messages": [(1_360_000_000, struct.pack("<f", -20.0 - qc_variant))]},
        {"id": 8, "topic": "/spmpc/status", "type": "std_msgs/String", "definition": b"string data\n",
         "messages": [(1_380_000_000, string(f"conflicting-status-{qc_variant}"))]},
    ]
    for spec in specs:
        spec["md5"] = f"{spec['id']:032x}"

    embedded = [connection(spec) for spec in specs]
    parts = list(embedded)
    offsets: dict[int, list[tuple[int, int]]] = {spec["id"]: [] for spec in specs}
    message_rows = sorted(
        (stamp, spec["id"], payload)
        for spec in specs for stamp, payload in spec["messages"]
    )
    running = sum(len(item) for item in parts)
    for stamp, connection_id, payload in message_rows:
        offsets[connection_id].append((stamp, running))
        encoded = record([
            ("op", bytes([reader_core.OP_MSG_DATA])),
            ("conn", struct.pack("<I", connection_id)),
            ("time", ros_time(stamp)),
        ], payload)
        parts.append(encoded)
        running += len(encoded)
    inner = b"".join(parts)

    # The complete record is 77 bytes without data; 4019 spaces make the
    # serialized file-header record exactly 4096 bytes including both lengths.
    padding = b" " * 4019
    zero_header = record([
        ("op", bytes([reader_core.OP_FILE_HEADER])),
        ("index_pos", struct.pack("<Q", 0)),
        ("conn_count", struct.pack("<I", len(specs))),
        ("chunk_count", struct.pack("<I", 1)),
    ], padding)
    if len(zero_header) != 4096:
        raise AssertionError("synthetic padded file header is not exactly 4096 bytes")
    chunk_pos = len(reader_core.MAGIC) + len(zero_header)
    chunk = record([
        ("op", bytes([reader_core.OP_CHUNK])),
        ("compression", b"none"),
        ("size", struct.pack("<I", len(inner))),
    ], inner)
    indexes = []
    for spec in specs:
        rows = offsets[spec["id"]]
        data = b"".join(ros_time(stamp) + struct.pack("<I", offset) for stamp, offset in rows)
        indexes.append(record([
            ("op", bytes([reader_core.OP_INDEX_DATA])),
            ("ver", struct.pack("<I", 1)),
            ("conn", struct.pack("<I", spec["id"])),
            ("count", struct.pack("<I", len(rows))),
        ], data))
    index_pos = chunk_pos + len(chunk) + sum(len(item) for item in indexes)
    file_header = record([
        ("op", bytes([reader_core.OP_FILE_HEADER])),
        ("index_pos", struct.pack("<Q", index_pos)),
        ("conn_count", struct.pack("<I", len(specs))),
        ("chunk_count", struct.pack("<I", 1)),
    ], padding)
    counts = b"".join(struct.pack("<II", spec["id"], len(offsets[spec["id"]])) for spec in specs)
    chunk_info = record([
        ("op", bytes([reader_core.OP_CHUNK_INFO])),
        ("ver", struct.pack("<I", 1)),
        ("chunk_pos", struct.pack("<Q", chunk_pos)),
        ("start_time", ros_time(message_rows[0][0])),
        ("end_time", ros_time(message_rows[-1][0])),
        ("count", struct.pack("<I", len(specs))),
    ], counts)
    return reader_core.MAGIC + file_header + chunk + b"".join(
        indexes + [connection(spec) for spec in specs] + [chunk_info]
    )


class S5A1SignalExtractorV3Tests(unittest.TestCase):
    def test_self_check_binds_final_reader_v1_hook_owner(self) -> None:
        result = extractor.self_check()
        self.assertEqual(result["reader_revision"], "v4")
        self.assertEqual(result["semantic_hook_owner"], "r8_liquid_ros1_bag_v2_reader_v1")
        self.assertTrue(result["hook_finally_restore"])
        self.assertFalse(result["real_bag_read"])

    def test_padded_multitopic_bag_captures_odom_clock_tf_and_qc(self) -> None:
        before = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        result = extractor.extract_bag_bytes(padded_bag())
        after = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        self.assertEqual(before, after)
        self.assertEqual(len(result["odom"]), 2)
        self.assertEqual(len(result["clock"]), 2)
        self.assertEqual(len(result["transforms"]), 2)
        self.assertEqual(result["odom"][1]["x_m"], 1.25)
        self.assertEqual(result["odom"][1]["odom_header_t_ns"], 2_000_000_000)
        self.assertEqual(result["qc_only_topic_counts"], {
            "/cmd_vel": 1,
            "/scout/global_path_fixed": 1,
            "/slosh/height": 1,
            "/spmpc/slosh_height": 1,
            "/spmpc/status": 1,
        })
        self.assertFalse(result["motion_provenance"]["forbidden_signal_consumed"])
        self.assertTrue(result["reader_summary"]["index_verified"])

    def test_cmd_path_proxy_modal_and_status_conflicts_cannot_change_motion(self) -> None:
        first_bytes = padded_bag(qc_variant=0)
        second_bytes = padded_bag(qc_variant=91)
        self.assertNotEqual(hashlib.sha256(first_bytes).digest(), hashlib.sha256(second_bytes).digest())
        first = extractor.extract_bag_bytes(first_bytes)
        second = extractor.extract_bag_bytes(second_bytes)
        self.assertEqual(first["odom"], second["odom"])
        self.assertEqual(
            extractor.build_motion_samples(first, maximum_gap_s=1.1),
            extractor.build_motion_samples(second, maximum_gap_s=1.1),
        )
        self.assertEqual(first["qc_only_topic_counts"], second["qc_only_topic_counts"])

    def test_hooks_restore_after_truncated_input_fails_closed(self) -> None:
        before = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        with self.assertRaises(reader_core.BagV2Error):
            extractor.extract_bag_bytes(padded_bag()[:-7])
        after = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        self.assertEqual(before, after)

    def test_nonfinite_motion_payload_fails_closed_and_restores_hooks(self) -> None:
        before = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        with self.assertRaisesRegex(reader_core.BagV2Error, "non-finite"):
            extractor.extract_bag_bytes(padded_bag(nonfinite_odom=True))
        after = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        self.assertEqual(before, after)

    def test_duplicate_and_backtracking_odom_header_times_fail_closed(self) -> None:
        for stamps in ((1_000_000_000, 1_000_000_000), (2_000_000_000, 1_000_000_000)):
            with self.subTest(stamps=stamps):
                with self.assertRaisesRegex(extractor.SignalExtractionError, "time|anomaly|increasing"):
                    extractor.extract_bag_bytes(padded_bag(odom_stamps=stamps))


if __name__ == "__main__":
    unittest.main()
