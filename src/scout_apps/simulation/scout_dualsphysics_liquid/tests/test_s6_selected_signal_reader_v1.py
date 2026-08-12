#!/usr/bin/env python3
"""Synthetic-byte tests for the S6 selected-signal reader."""

from __future__ import annotations

import ast
import math
import struct
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6_selected_signal_reader_v1.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_ros1_bag_v2_reader_v1 as reader  # noqa: E402
import r8_liquid_s6_selected_signal_reader_v1 as selected  # noqa: E402


def field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = b"".join(field(name, value) for name, value in fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def ros_time(value_ns: int) -> bytes:
    return struct.pack("<II", value_ns // 1_000_000_000, value_ns % 1_000_000_000)


def synthetic_bag(*, include_modal: bool = True, duplicate_time: bool = False, nan_proxy: bool = False) -> bytes:
    topics = [(0, "/slosh/height", [0.001, 0.002, -0.001])]
    if include_modal:
        topics.append((1, "/spmpc/slosh_height", [1.0, 2.0, -1.0]))
    messages: list[tuple[int, int, bytes]] = []
    for connection, _topic, values in topics:
        for index, value in enumerate(values):
            stamp = 1_000_000_000 + index * 100_000_000
            if duplicate_time and index == 2:
                stamp = 1_100_000_000
            if nan_proxy and connection == 0 and index == 1:
                value = math.nan
            messages.append((connection, stamp, struct.pack("<f", value)))
    messages.sort(key=lambda row: (row[1], row[0]))
    inner_parts, offsets = [], []
    offset = 0
    for connection, stamp, payload in messages:
        item = record([("op", bytes([reader.OP_MSG_DATA])), ("conn", struct.pack("<I", connection)), ("time", ros_time(stamp))], payload)
        offsets.append((connection, stamp, offset))
        inner_parts.append(item)
        offset += len(item)
    inner = b"".join(inner_parts)
    header_zero = record([("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", 0)),
                          ("conn_count", struct.pack("<I", len(topics))), ("chunk_count", struct.pack("<I", 1))])
    chunk_pos = len(reader.MAGIC) + len(header_zero)
    chunk = record([("op", bytes([reader.OP_CHUNK])), ("compression", b"none"), ("size", struct.pack("<I", len(inner)))], inner)
    indexes = []
    for connection, _topic, _values in topics:
        entries = [(stamp, item_offset) for conn, stamp, item_offset in offsets if conn == connection]
        indexes.append(record([("op", bytes([reader.OP_INDEX_DATA])), ("ver", struct.pack("<I", 1)),
                               ("conn", struct.pack("<I", connection)), ("count", struct.pack("<I", len(entries)))],
                              b"".join(ros_time(stamp) + struct.pack("<I", item_offset) for stamp, item_offset in entries)))
    index_pos = chunk_pos + len(chunk) + sum(map(len, indexes))
    file_header = record([("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", index_pos)),
                          ("conn_count", struct.pack("<I", len(topics))), ("chunk_count", struct.pack("<I", 1))])
    connections = []
    for connection, topic, _values in topics:
        data = b"".join([field("type", b"std_msgs/Float32"), field("md5sum", b"73fcbf46b49191e672908e50842a83d4"),
                         field("message_definition", b"float32 data\n"), field("callerid", b"/synthetic_s6")])
        connections.append(record([("op", bytes([reader.OP_CONNECTION])), ("conn", struct.pack("<I", connection)),
                                   ("topic", topic.encode())], data))
    counts = b"".join(struct.pack("<II", connection, len(values)) for connection, _topic, values in topics)
    chunk_info = record([("op", bytes([reader.OP_CHUNK_INFO])), ("ver", struct.pack("<I", 1)),
                         ("chunk_pos", struct.pack("<Q", chunk_pos)), ("start_time", ros_time(min(row[1] for row in messages))),
                         ("end_time", ros_time(max(row[1] for row in messages))), ("count", struct.pack("<I", len(topics)))], counts)
    return reader.MAGIC + file_header + chunk + b"".join(indexes + connections) + chunk_info


class SelectedSignalReaderTests(unittest.TestCase):
    def test_extracts_both_series_and_freezes_units(self) -> None:
        data = synthetic_bag()
        result = selected.extract_selected_signals(data, synthetic_fixture=True)
        self.assertEqual(result["planned_denominator"], 1)
        self.assertAlmostEqual(result["series"]["H_proxy"]["samples"][1]["value_comparison_mm"], 2.0, places=5)
        self.assertAlmostEqual(result["series"]["H_modal"]["samples"][1]["value_comparison_mm"], 2.0, places=5)
        self.assertFalse(result["source"]["real_bag_read"])
        self.assertFalse(result["claims"]["paired_ranking"])

    def test_missing_duplicate_nonfinite_and_real_inputs_fail_closed(self) -> None:
        cases = [
            (synthetic_bag(include_modal=False), True, "required topic"),
            (synthetic_bag(duplicate_time=True), True, "strictly increasing"),
            (synthetic_bag(nan_proxy=True), True, "non-finite"),
            (synthetic_bag(), False, "NOT_ADMITTED"),
        ]
        for data, synthetic, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(selected.SelectedSignalError, message):
                selected.extract_selected_signals(data, synthetic_fixture=synthetic)

    def test_source_hash_and_size_are_bounded(self) -> None:
        data = synthetic_bag()
        with self.assertRaisesRegex(selected.SelectedSignalError, "SHA-256"):
            selected.extract_selected_signals(data, synthetic_fixture=True, expected_sha256="0" * 64)
        with mock.patch.object(selected, "MAXIMUM_SOURCE_BYTES", 10), self.assertRaisesRegex(selected.SelectedSignalError, "bound"):
            selected.extract_selected_signals(data, synthetic_fixture=True)

    def test_ast_has_no_path_reader_ros_network_or_execution(self) -> None:
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = set()
        calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr in {"open", "read_bytes", "system", "popen", "run", "Popen"}:
                    calls.append(node.func.attr)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual([], calls)


if __name__ == "__main__":
    unittest.main()
