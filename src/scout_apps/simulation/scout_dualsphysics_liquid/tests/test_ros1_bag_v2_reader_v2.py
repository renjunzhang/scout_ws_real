#!/usr/bin/env python3
"""Static fixtures and negative tests for chunk OP_CONNECTION remediation v2."""

from __future__ import annotations

import bz2
import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_ros1_bag_v2_reader_v2 as reader  # noqa: E402


def field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = b"".join(field(name, value) for name, value in fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def ros_time(value_ns: int) -> bytes:
    return struct.pack("<II", value_ns // 1_000_000_000, value_ns % 1_000_000_000)


def connection(
    *, definition: bytes = b"float32 x\n", op: int = reader.OP_CONNECTION,
    callerid: bytes = b"/fixture",
) -> bytes:
    data = b"".join([
        field("type", b"fixture_msgs/Value"), field("md5sum", b"1" * 32),
        field("message_definition", definition), field("callerid", callerid),
    ])
    return record([
        ("op", bytes([op])), ("conn", struct.pack("<I", 0)), ("topic", b"/fixture"),
    ], data)


def bag_fixture(
    *, embedded: list[bytes] | None = None, compression: str = "none",
    index_delta: int = 0, connection_after_message: bool = False,
    top_connection_after_chunk_info: bool = False,
) -> bytes:
    embedded = [connection()] if embedded is None else embedded
    stamp = 1_000_000_000
    message = record([
        ("op", bytes([reader.OP_MSG_DATA])), ("conn", struct.pack("<I", 0)),
        ("time", ros_time(stamp)),
    ], b"\x00")
    parts = ([message] + embedded) if connection_after_message else (embedded + [message])
    inner = b"".join(parts)
    message_offset = 0 if connection_after_message else sum(len(item) for item in embedded)
    payload = inner if compression == "none" else bz2.compress(inner)
    header_zero = record([
        ("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", 0)),
        ("conn_count", struct.pack("<I", 1)), ("chunk_count", struct.pack("<I", 1)),
    ])
    chunk_pos = len(reader.MAGIC) + len(header_zero)
    chunk = record([
        ("op", bytes([reader.OP_CHUNK])), ("compression", compression.encode()),
        ("size", struct.pack("<I", len(inner))),
    ], payload)
    index = record([
        ("op", bytes([reader.OP_INDEX_DATA])), ("ver", struct.pack("<I", 1)),
        ("conn", struct.pack("<I", 0)), ("count", struct.pack("<I", 1)),
    ], ros_time(stamp) + struct.pack("<I", message_offset + index_delta))
    index_pos = chunk_pos + len(chunk) + len(index)
    file_header = record([
        ("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", index_pos)),
        ("conn_count", struct.pack("<I", 1)), ("chunk_count", struct.pack("<I", 1)),
    ])
    top = connection()
    info = record([
        ("op", bytes([reader.OP_CHUNK_INFO])), ("ver", struct.pack("<I", 1)),
        ("chunk_pos", struct.pack("<Q", chunk_pos)), ("start_time", ros_time(stamp)),
        ("end_time", ros_time(stamp)), ("count", struct.pack("<I", 1)),
    ], struct.pack("<II", 0, 1))
    post = info + top if top_connection_after_chunk_info else top + info
    return reader.MAGIC + file_header + chunk + index + post


def multi_connection_fixture(specs: list[tuple[int, str, str, str, bytes, bytes]]) -> bytes:
    def conn_record(spec: tuple[int, str, str, str, bytes, bytes]) -> bytes:
        conn_id, topic, kind, md5, definition, _payload = spec
        data = b"".join([
            field("type", kind.encode()), field("md5sum", md5.encode()),
            field("message_definition", definition), field("callerid", b"/fixture"),
        ])
        return record([
            ("op", bytes([reader.OP_CONNECTION])), ("conn", struct.pack("<I", conn_id)),
            ("topic", topic.encode()),
        ], data)
    parts: list[bytes] = []
    offsets: dict[int, int] = {}
    stamps: dict[int, int] = {}
    for index, spec in enumerate(specs):
        conn_id, _topic, _kind, _md5, _definition, payload = spec
        parts.append(conn_record(spec))
        stamps[conn_id] = 1_000_000_000 + index
        offsets[conn_id] = sum(map(len, parts))
        parts.append(record([
            ("op", bytes([reader.OP_MSG_DATA])), ("conn", struct.pack("<I", conn_id)),
            ("time", ros_time(stamps[conn_id])),
        ], payload))
    inner = b"".join(parts)
    zero = record([
        ("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", 0)),
        ("conn_count", struct.pack("<I", len(specs))), ("chunk_count", struct.pack("<I", 1)),
    ])
    chunk_pos = len(reader.MAGIC) + len(zero)
    chunk = record([
        ("op", bytes([reader.OP_CHUNK])), ("compression", b"none"),
        ("size", struct.pack("<I", len(inner))),
    ], inner)
    indexes = [record([
        ("op", bytes([reader.OP_INDEX_DATA])), ("ver", struct.pack("<I", 1)),
        ("conn", struct.pack("<I", spec[0])), ("count", struct.pack("<I", 1)),
    ], ros_time(stamps[spec[0]]) + struct.pack("<I", offsets[spec[0]])) for spec in specs]
    index_pos = chunk_pos + len(chunk) + sum(map(len, indexes))
    header = record([
        ("op", bytes([reader.OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", index_pos)),
        ("conn_count", struct.pack("<I", len(specs))), ("chunk_count", struct.pack("<I", 1)),
    ])
    top = [conn_record(spec) for spec in specs]
    counts = b"".join(struct.pack("<II", spec[0], 1) for spec in specs)
    info = record([
        ("op", bytes([reader.OP_CHUNK_INFO])), ("ver", struct.pack("<I", 1)),
        ("chunk_pos", struct.pack("<Q", chunk_pos)),
        ("start_time", ros_time(min(stamps.values()))), ("end_time", ros_time(max(stamps.values()))),
        ("count", struct.pack("<I", len(specs))),
    ], counts)
    return reader.MAGIC + header + chunk + b"".join(indexes + top) + info


class ReaderV2Tests(unittest.TestCase):
    def test_matching_chunk_connection_is_accepted_for_none_and_bz2(self) -> None:
        for compression in ("none", "bz2"):
            with self.subTest(compression=compression):
                result = reader.parse_bag_v2(bag_fixture(compression=compression))
                self.assertTrue(result["index_verified"])
                self.assertEqual(result["total_message_count"], 1)
                self.assertEqual(result["compression"], [compression])

    def test_repeated_identical_chunk_connection_is_accepted(self) -> None:
        result = reader.parse_bag_v2(bag_fixture(embedded=[connection(), connection()]))
        self.assertEqual(result["connections"][0]["message_count"], 1)

    def test_conflicting_chunk_connection_fails_closed(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "conflicts with top-level"):
            reader.parse_bag_v2(bag_fixture(embedded=[connection(definition=b"float32 y\n")]))
        with self.assertRaisesRegex(reader.BagV2Error, "conflicts with top-level"):
            reader.parse_bag_v2(bag_fixture(embedded=[connection(callerid=b"/drift")]))

    def test_unknown_chunk_op_fails_closed(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "unknown op"):
            reader.parse_bag_v2(bag_fixture(embedded=[connection(op=0x09)]))

    def test_connection_order_drift_fails_closed(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "appears after message data"):
            reader.parse_bag_v2(bag_fixture(connection_after_message=True))
        with self.assertRaisesRegex(reader.BagV2Error, "after chunk-info"):
            reader.parse_bag_v2(bag_fixture(top_connection_after_chunk_info=True))

    def test_original_index_drift_fails_closed(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "index-data entries drift"):
            reader.parse_bag_v2(bag_fixture(index_delta=1))

    def test_chunk_connection_may_be_absent(self) -> None:
        result = reader.parse_bag_v2(bag_fixture(embedded=[]))
        self.assertEqual(result["total_message_count"], 1)

    def test_only_terminal_crlf_is_canonicalized_for_same_topic(self) -> None:
        specs = [
            (7, "/cmd_vel", "geometry_msgs/Twist", "9" * 32, b"float64[6] data\n", b"\0" * 48),
            (13, "/cmd_vel", "geometry_msgs/Twist", "9" * 32, b"float64[6] data", b"\0" * 48),
        ]
        result = reader.parse_bag_v2(multi_connection_fixture(specs))
        rows = result["connections"]
        self.assertNotEqual(rows[0]["message_definition_sha256"], rows[1]["message_definition_sha256"])
        self.assertNotEqual(rows[0]["message_definition_size_bytes"], rows[1]["message_definition_size_bytes"])
        self.assertEqual(rows[0]["canonical_message_definition_sha256"], rows[1]["canonical_message_definition_sha256"])
        changed = list(specs)
        changed[1] = (*changed[1][:4], b"float64[5] data", changed[1][5])
        with self.assertRaisesRegex(reader.BagV2Error, "definitions conflict"):
            reader.parse_bag_v2(multi_connection_fixture(changed))
        trailing_space = list(specs)
        trailing_space[1] = (*trailing_space[1][:4], b"float64[6] data ", trailing_space[1][5])
        with self.assertRaisesRegex(reader.BagV2Error, "definitions conflict"):
            reader.parse_bag_v2(multi_connection_fixture(trailing_space))
        md5_drift = list(specs)
        md5_drift[1] = (13, "/cmd_vel", "geometry_msgs/Twist", "a" * 32, b"float64[6] data", b"\0" * 48)
        with self.assertRaisesRegex(reader.BagV2Error, "definitions conflict"):
            reader.parse_bag_v2(multi_connection_fixture(md5_drift))

    def test_nonfinite_exception_is_exactly_terminal_debug(self) -> None:
        payload = struct.pack("<III", 0, 0, 1) + struct.pack("<f", float("nan"))
        allowed = [(1, "/spmpc/terminal/debug", "std_msgs/Float32MultiArray", "8" * 32, b"layout\n", payload)]
        result = reader.parse_bag_v2(multi_connection_fixture(allowed))
        self.assertEqual(result["qc_nonfinite_sentinel_count"], 1)
        forbidden = [(1, "/spmpc/debug/effective_config", "std_msgs/Float32MultiArray", "8" * 32, b"layout\n", payload)]
        with self.assertRaisesRegex(reader.BagV2Error, "non-finite"):
            reader.parse_bag_v2(multi_connection_fixture(forbidden))
        selected = [(1, "/slosh/height", "std_msgs/Float32", "7" * 32, b"float32 data\n", struct.pack("<f", float("nan")))]
        with self.assertRaisesRegex(reader.BagV2Error, "non-finite"):
            reader.parse_bag_v2(multi_connection_fixture(selected))


if __name__ == "__main__":
    unittest.main()
