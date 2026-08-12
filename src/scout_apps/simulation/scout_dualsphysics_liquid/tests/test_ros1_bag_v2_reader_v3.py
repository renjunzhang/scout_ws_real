#!/usr/bin/env python3
"""Static fixtures for reader v3's exact connection-data topic contract."""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests"))
import r8_liquid_ros1_bag_v2_reader_v3 as reader  # noqa: E402
import test_ros1_bag_v2_reader_v2 as fixtures  # noqa: E402


def connection_with_topic(
    *, definition: bytes = b"float32 x\n", op: int = reader.OP_CONNECTION,
    callerid: bytes = b"/fixture", topic_data: bytes = b"/fixture",
    extra: tuple[str, bytes] | None = None,
) -> bytes:
    fields = [
        ("type", b"fixture_msgs/Value"), ("md5sum", b"1" * 32),
        ("message_definition", definition), ("callerid", callerid),
        ("topic", topic_data),
    ]
    if extra is not None:
        fields.append(extra)
    data = b"".join(fixtures.field(name, value) for name, value in fields)
    return fixtures.record([
        ("op", bytes([op])), ("conn", struct.pack("<I", 0)),
        ("topic", b"/fixture"),
    ], data)


class ReaderV3Tests(unittest.TestCase):
    def test_standard_topic_field_is_required_and_matching(self) -> None:
        original = fixtures.connection
        try:
            fixtures.connection = connection_with_topic
            result = reader.parse_bag_v2(fixtures.bag_fixture())
        finally:
            fixtures.connection = original
        self.assertEqual(result["total_message_count"], 1)
        self.assertTrue(result["index_verified"])

    def test_missing_topic_field_fails_closed(self) -> None:
        with self.assertRaisesRegex(reader.BagV2Error, "field 'topic' is absent"):
            reader.parse_bag_v2(fixtures.bag_fixture())

    def test_topic_mismatch_and_unknown_field_fail_closed(self) -> None:
        original = fixtures.connection
        try:
            fixtures.connection = lambda **kwargs: connection_with_topic(
                **kwargs, topic_data=b"/different"
            )
            with self.assertRaisesRegex(reader.BagV2Error, "topic differs"):
                reader.parse_bag_v2(fixtures.bag_fixture())
            fixtures.connection = lambda **kwargs: connection_with_topic(
                **kwargs, extra=("tcp_nodelay", b"1")
            )
            with self.assertRaisesRegex(reader.BagV2Error, "unknown fields"):
                reader.parse_bag_v2(fixtures.bag_fixture())
        finally:
            fixtures.connection = original


if __name__ == "__main__":
    unittest.main()
