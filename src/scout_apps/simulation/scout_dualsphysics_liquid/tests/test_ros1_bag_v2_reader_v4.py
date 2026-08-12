#!/usr/bin/env python3
"""Static fixtures for padded ROS1 Bag V2 file headers."""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))
import r8_liquid_ros1_bag_v2_reader_v1 as v1  # noqa: E402
import r8_liquid_ros1_bag_v2_reader_v2 as v2  # noqa: E402
import r8_liquid_ros1_bag_v2_reader_v4 as reader  # noqa: E402
import test_ros1_bag_v2_reader_v2 as fixtures  # noqa: E402
from test_ros1_bag_v2_reader_v3 import connection_with_topic  # noqa: E402


def topic_fixture() -> bytes:
    original = fixtures.connection
    try:
        fixtures.connection = connection_with_topic
        return fixtures.bag_fixture()
    finally:
        fixtures.connection = original


def add_file_header_padding(data: bytes, padding: bytes) -> bytes:
    limits = reader.ReaderLimits()
    records = []
    offset = len(reader.MAGIC)
    while offset < len(data):
        row = v1._parse_record(data, offset, limits, label="fixture top-level")
        records.append(row)
        offset = row.end
    shift = len(padding)
    output = []
    for index, row in enumerate(records):
        fields = []
        for name, raw in row.fields.items():
            value = bytes(raw)
            if index == 0 and name == "index_pos":
                value = struct.pack("<Q", v1._u64_field(row.fields, name, "header") + shift)
            elif row.op == reader.OP_CHUNK_INFO and name == "chunk_pos":
                value = struct.pack("<Q", v1._u64_field(row.fields, name, "chunk-info") + shift)
            fields.append((name, value))
        output.append(v2._record(fields, padding if index == 0 else bytes(row.data)))
    return reader.MAGIC + b"".join(output)


class ReaderV4Tests(unittest.TestCase):
    def test_unpadded_and_standard_padded_headers_parse_identically(self) -> None:
        raw = topic_fixture()
        plain = reader.parse_bag_v2(raw)
        padded = reader.parse_bag_v2(add_file_header_padding(raw, b" " * 4027))
        self.assertEqual(plain["total_message_count"], 1)
        self.assertEqual(padded["total_message_count"], 1)
        self.assertTrue(padded["index_verified"])
        self.assertTrue(padded["chunk_info_verified"])

    def test_nonspace_or_oversized_padding_fails_closed(self) -> None:
        raw = topic_fixture()
        with self.assertRaisesRegex(reader.BagV2Error, "padding is not bounded ASCII spaces"):
            reader.parse_bag_v2(add_file_header_padding(raw, b"\0" * 32))
        with self.assertRaisesRegex(reader.BagV2Error, "padding is not bounded ASCII spaces"):
            reader.parse_bag_v2(add_file_header_padding(raw, b" " * 4097))


if __name__ == "__main__":
    unittest.main()
