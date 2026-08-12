#!/usr/bin/env python3
"""In-memory synthetic ROS1 Bag V2 tests for the S6 v5 producer."""

from __future__ import annotations

import ast
import hashlib
import math
import struct
import sys
import unittest
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6_real_selected_signal_extractor_v5.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_ros1_bag_v2_reader_v1 as reader_core  # noqa: E402
import r8_liquid_s5a1_ros1_signal_extractor_v1 as extractor_v1  # noqa: E402
import r8_liquid_s6_real_selected_signal_extractor_v5 as selected  # noqa: E402


def field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header_bytes = b"".join(field(name, value) for name, value in fields)
    return struct.pack("<I", len(header_bytes)) + header_bytes + struct.pack("<I", len(data)) + data


def ros_time(value_ns: int) -> bytes:
    return struct.pack("<II", value_ns // 1_000_000_000, value_ns % 1_000_000_000)


def ros_string(value: str) -> bytes:
    raw = value.encode("utf-8")
    return struct.pack("<I", len(raw)) + raw


def odom_payload(stamp_ns: int, x_m: float) -> bytes:
    values = [0.0] * 85
    values[0] = x_m
    values[6] = 1.0
    return struct.pack("<I", 0) + ros_time(stamp_ns) + ros_string("odom") + \
        ros_string("base_footprint") + struct.pack("<85d", *values)


def connection(spec: dict[str, Any]) -> bytes:
    metadata = b"".join([
        field("type", spec["type"].encode("utf-8")),
        field("md5sum", f'{spec["id"]:032x}'.encode("ascii")),
        field("message_definition", spec["definition"]),
        field("callerid", b"/synthetic_s6_v5"),
        field("topic", spec["topic"].encode("utf-8")),
    ])
    return record([
        ("op", bytes([reader_core.OP_CONNECTION])),
        ("conn", struct.pack("<I", spec["id"])),
        ("topic", spec["topic"].encode("utf-8")),
    ], metadata)


def padded_bag(
    *,
    qc_variant: int = 0,
    odom_records: tuple[int, int, int] = (1_100_000_000, 2_100_000_000, 3_100_000_000),
    odom_headers: tuple[int, int, int] = (1_000_000_000, 2_001_000_000, 3_000_000_000),
    proxy_records: tuple[int, int] = (1_200_000_000, 2_200_000_000),
    proxy_values: tuple[float, float] = (0.001, 0.002),
    modal_records: tuple[int, int] = (1_300_000_000, 2_300_000_000),
    modal_values: tuple[float, float] = (1.5, 2.5),
    include_modal: bool = True,
    proxy_type: str = "std_msgs/Float32",
) -> bytes:
    specs: list[dict[str, Any]] = [
        {"id": 1, "topic": "/odom", "type": "nav_msgs/Odometry", "definition": b"synthetic odom\n",
         "messages": [(record_ns, odom_payload(header_ns, index * 0.25))
                      for index, (record_ns, header_ns) in enumerate(zip(odom_records, odom_headers))]},
        {"id": 2, "topic": "/slosh/height", "type": proxy_type, "definition": b"float32 data\n",
         "messages": [(stamp, struct.pack("<f", value)) for stamp, value in zip(proxy_records, proxy_values)]},
        {"id": 4, "topic": "/cmd_vel", "type": "geometry_msgs/Twist", "definition": b"synthetic cmd\n",
         "messages": [(1_400_000_000, struct.pack("<6d", *([float(qc_variant)] * 6)))]},
        {"id": 5, "topic": "/spmpc/status", "type": "std_msgs/String", "definition": b"string data\n",
         "messages": [(1_500_000_000, ros_string(f"conflict-{qc_variant}"))]},
    ]
    if include_modal:
        specs.append({
            "id": 3, "topic": "/spmpc/slosh_height", "type": "std_msgs/Float32",
            "definition": b"float32 data\n",
            "messages": [(stamp, struct.pack("<f", value)) for stamp, value in zip(modal_records, modal_values)],
        })

    embedded = [connection(spec) for spec in specs]
    parts = list(embedded)
    offsets: dict[int, list[tuple[int, int]]] = {spec["id"]: [] for spec in specs}
    rows = sorted((stamp, spec["id"], payload) for spec in specs for stamp, payload in spec["messages"])
    running = sum(len(item) for item in parts)
    for stamp, connection_id, payload in rows:
        offsets[connection_id].append((stamp, running))
        encoded = record([
            ("op", bytes([reader_core.OP_MSG_DATA])),
            ("conn", struct.pack("<I", connection_id)),
            ("time", ros_time(stamp)),
        ], payload)
        parts.append(encoded)
        running += len(encoded)
    inner = b"".join(parts)

    padding = b" " * 4019
    zero_header = record([
        ("op", bytes([reader_core.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", 0)),
        ("conn_count", struct.pack("<I", len(specs))), ("chunk_count", struct.pack("<I", 1)),
    ], padding)
    if len(zero_header) != 4096:
        raise AssertionError("padded file header is not 4096 bytes")
    chunk_pos = len(reader_core.MAGIC) + len(zero_header)
    chunk = record([
        ("op", bytes([reader_core.OP_CHUNK])), ("compression", b"none"),
        ("size", struct.pack("<I", len(inner))),
    ], inner)
    indexes = []
    for spec in specs:
        indexed = offsets[spec["id"]]
        indexes.append(record([
            ("op", bytes([reader_core.OP_INDEX_DATA])), ("ver", struct.pack("<I", 1)),
            ("conn", struct.pack("<I", spec["id"])), ("count", struct.pack("<I", len(indexed))),
        ], b"".join(ros_time(stamp) + struct.pack("<I", offset) for stamp, offset in indexed)))
    index_pos = chunk_pos + len(chunk) + sum(len(item) for item in indexes)
    file_header = record([
        ("op", bytes([reader_core.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", index_pos)),
        ("conn_count", struct.pack("<I", len(specs))), ("chunk_count", struct.pack("<I", 1)),
    ], padding)
    counts = b"".join(struct.pack("<II", spec["id"], len(offsets[spec["id"]])) for spec in specs)
    chunk_info = record([
        ("op", bytes([reader_core.OP_CHUNK_INFO])), ("ver", struct.pack("<I", 1)),
        ("chunk_pos", struct.pack("<Q", chunk_pos)), ("start_time", ros_time(rows[0][0])),
        ("end_time", ros_time(rows[-1][0])), ("count", struct.pack("<I", len(specs))),
    ], counts)
    return reader_core.MAGIC + file_header + chunk + b"".join(
        indexes + [connection(spec) for spec in specs] + [chunk_info]
    )


def produce(data: bytes, *, synthetic_fixture: bool = True) -> dict[str, Any]:
    return selected.produce_selected_signals(
        data,
        synthetic_fixture=synthetic_fixture,
        expected_source_sha256=hashlib.sha256(data).hexdigest(),
        s5a0_receipt_sha256="a" * 64,
        s5a1_transfer_sha256="b" * 64,
    )


class S6RealSelectedSignalExtractorV5Tests(unittest.TestCase):
    def test_golden_maps_both_series_to_odom_header_axis(self) -> None:
        result = produce(padded_bag())
        alignment = result["time_alignment"]
        self.assertEqual(alignment["record_to_odom_header_offset_ns"], -100_000_000)
        self.assertEqual(alignment["residual_max_abs_ns"], 1_000_000)
        self.assertAlmostEqual(alignment["residual_mean_ns"], 1_000_000 / 3)
        self.assertAlmostEqual(alignment["residual_rms_ns"], math.sqrt(1_000_000 ** 2 / 3))
        self.assertEqual(result["series"]["H_proxy"]["samples"][0]["mapped_odom_header_t_ns"], 1_100_000_000)
        self.assertAlmostEqual(
            result["series"]["H_proxy"]["samples"][1]["value_comparison_mm"], 2.0, places=6
        )
        self.assertAlmostEqual(result["series"]["H_modal"]["samples"][1]["value_comparison_mm"], 2.5)

    def test_qc_payload_conflicts_do_not_change_signal_or_alignment(self) -> None:
        first = produce(padded_bag(qc_variant=0))
        second = produce(padded_bag(qc_variant=91))
        self.assertNotEqual(first["integrity"]["source_bag_sha256"], second["integrity"]["source_bag_sha256"])
        self.assertEqual(first["time_alignment"], second["time_alignment"])
        self.assertEqual(first["series"], second["series"])
        self.assertEqual(first["provenance"]["qc_only_topics"], second["provenance"]["qc_only_topics"])

    def test_alignment_and_signal_anomalies_fail_closed(self) -> None:
        cases = [
            (padded_bag(odom_headers=(1_000_000_000, 2_020_000_000, 3_000_000_000)), "residual"),
            (padded_bag(odom_headers=(1_000_000_000, 1_000_000_000, 3_000_000_000)), "time|anomaly|increasing"),
            (padded_bag(proxy_records=(1_200_000_000, 1_200_000_000)), "time|anomaly|increasing"),
            (padded_bag(proxy_values=(0.001, float("nan"))), "non-finite"),
            (padded_bag(include_modal=False), "required topic"),
            (padded_bag(proxy_type="std_msgs/String"), "type"),
            (padded_bag(proxy_records=(900_000_000, 2_200_000_000)), "extrapolation"),
        ]
        for data, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(selected.SelectedSignalV5Error, message):
                produce(data)

    def test_hash_fixture_gate_and_hook_restoration(self) -> None:
        data = padded_bag()
        with self.assertRaisesRegex(selected.SelectedSignalV5Error, "NOT_ADMITTED"):
            produce(data, synthetic_fixture=False)
        with self.assertRaisesRegex(selected.SelectedSignalV5Error, "source bag SHA-256"):
            selected.produce_selected_signals(
                data, synthetic_fixture=True, expected_source_sha256="0" * 64,
                s5a0_receipt_sha256="a" * 64, s5a1_transfer_sha256="b" * 64,
            )
        before = (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader)
        with self.assertRaises(selected.SelectedSignalV5Error):
            produce(data[:-7])
        self.assertEqual(before, (reader_core._Message, reader_core._decode_registered_payload, extractor_v1.reader))

    def test_schema_ast_and_frozen_claims(self) -> None:
        selected.load_schema()
        result = produce(padded_bag())
        tampered = dict(result)
        tampered["unexpected"] = True
        with self.assertRaisesRegex(selected.SelectedSignalV5Error, "schema failure"):
            selected.validate_result(tampered)
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = {
            alias.name.split(".")[0]
            for node in ast.walk(tree) if isinstance(node, ast.Import)
            for alias in node.names
        }
        imports.update(
            node.module.split(".")[0] for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )
        writes_or_executes = [
            node.func.attr for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"write_bytes", "write_text", "system", "popen", "run", "Popen"}
        ]
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual(writes_or_executes, [])
        self.assertFalse(result["claims"]["stage6_pass"])
        self.assertFalse(result["claims"]["paired_ranking"])
        self.assertFalse(result["claims"]["formal"])
        self.assertFalse(result["claims"]["production"])

    def test_self_check_remains_fixture_only(self) -> None:
        result = selected.self_check()
        self.assertIn("FIXTURE_SELF_CHECK_OK_NOT_ADMITTED", result["status"])
        self.assertFalse(result["raw_real_bag_read"])
        self.assertFalse(result["optional_bag_read"])
        self.assertFalse(result["external_write_performed"])
        self.assertFalse(result["solver_or_gpu_executed"])
        self.assertFalse(result["stage6_pass"])


if __name__ == "__main__":
    unittest.main()
