#!/usr/bin/env python3
"""ROS1 Bag V2 reader v3 for standard connection-data ``topic`` fields.

Revision v2 already verifies embedded/top-level connection equality, index
offsets, canonical trailing-CR/LF definition identity, and the exact
terminal-debug non-finite sentinel exception.  Real ROS1 connection data also
contains a standard ``topic`` field.  This revision admits that one field only
when it byte-decodes to the exact topic in the enclosing connection header.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import r8_liquid_ros1_bag_v2_reader_v1 as v1
import r8_liquid_ros1_bag_v2_reader_v2 as v2


BagV2Error = v2.BagV2Error
ReaderLimits = v2.ReaderLimits
MAGIC = v2.MAGIC
OP_MSG_DATA = v2.OP_MSG_DATA
OP_FILE_HEADER = v2.OP_FILE_HEADER
OP_INDEX_DATA = v2.OP_INDEX_DATA
OP_CHUNK = v2.OP_CHUNK
OP_CHUNK_INFO = v2.OP_CHUNK_INFO
OP_CONNECTION = v2.OP_CONNECTION
SUPPORTED_COMPRESSION = v2.SUPPORTED_COMPRESSION
read_regular_file = v2.read_regular_file


def _connection_identity(
    record: Any, limits: ReaderLimits, label: str
) -> tuple[int, str, str, str, bytes, bytes]:
    if set(record.fields) != {"op", "conn", "topic"}:
        raise BagV2Error(f"{label} connection header fields differ")
    connection_id = v1._u32_field(record.fields, "conn", label)
    topic = v1._text_field(record.fields, "topic", label, maximum=limits.maximum_string_bytes)
    fields = v1._parse_fields(record.data, limits, label=f"{label} connection data")
    allowed = {"type", "md5sum", "message_definition", "callerid", "latching", "topic"}
    if not set(fields).issubset(allowed):
        raise BagV2Error(f"{label} connection data has unknown fields")
    data_topic = v1._text_field(fields, "topic", label, maximum=limits.maximum_string_bytes)
    if data_topic != topic:
        raise BagV2Error(f"{label} connection data topic differs from its header")
    message_type = v1._text_field(fields, "type", label, maximum=limits.maximum_string_bytes)
    md5sum = v1._text_field(fields, "md5sum", label, maximum=128)
    if len(md5sum) != 32 or any(character not in "0123456789abcdef" for character in md5sum):
        raise BagV2Error(f"{label} md5sum is invalid")
    definition = fields.get("message_definition")
    if not definition or len(definition) > limits.maximum_message_definition_bytes:
        raise BagV2Error(f"{label} message_definition is absent or oversized")
    return connection_id, topic, message_type, md5sum, bytes(definition), bytes(record.data)


def parse_bag_v2(data: bytes, *, limits: ReaderLimits | None = None) -> dict[str, Any]:
    original = v2._connection_identity
    v2._connection_identity = _connection_identity
    try:
        return v2.parse_bag_v2(data, limits=limits)
    finally:
        v2._connection_identity = original


def inspect_bag(
    path: str | Path,
    *,
    limits: ReaderLimits | None = None,
    expected_sha256: str | None = None,
    expected_size_bytes: int | None = None,
    expected_mode: int | None = None,
) -> dict[str, Any]:
    selected = limits or ReaderLimits()
    data, before, after = read_regular_file(
        path,
        limits=selected,
        expected_sha256=expected_sha256,
        expected_size_bytes=expected_size_bytes,
        expected_mode=expected_mode,
    )
    return {"source_before": before, "source_after": after, "bag": parse_bag_v2(data, limits=selected)}


__all__ = [
    "BagV2Error", "MAGIC", "OP_CHUNK", "OP_CHUNK_INFO", "OP_CONNECTION",
    "OP_FILE_HEADER", "OP_INDEX_DATA", "OP_MSG_DATA", "ReaderLimits",
    "SUPPORTED_COMPRESSION", "inspect_bag", "parse_bag_v2", "read_regular_file",
]
