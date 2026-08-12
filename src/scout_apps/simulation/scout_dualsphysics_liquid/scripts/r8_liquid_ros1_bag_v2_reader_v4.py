#!/usr/bin/env python3
"""ROS1 Bag V2 reader v4 preserving the standard padded file header.

ROS1 writes the first file-header record with space padding so the record is
4096 bytes long.  Reader v2's in-memory normalization retained offsets based
on that padded length but rebuilt an unpadded header.  V4 preserves the exact
source padding bytes in the normalized in-memory stream.  It never repairs or
writes the source file.
"""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Any

import r8_liquid_ros1_bag_v2_reader_v1 as v1
import r8_liquid_ros1_bag_v2_reader_v2 as v2
import r8_liquid_ros1_bag_v2_reader_v3 as v3


BagV2Error = v3.BagV2Error
ReaderLimits = v3.ReaderLimits
MAGIC = v3.MAGIC
OP_MSG_DATA = v3.OP_MSG_DATA
OP_FILE_HEADER = v3.OP_FILE_HEADER
OP_INDEX_DATA = v3.OP_INDEX_DATA
OP_CHUNK = v3.OP_CHUNK
OP_CHUNK_INFO = v3.OP_CHUNK_INFO
OP_CONNECTION = v3.OP_CONNECTION
SUPPORTED_COMPRESSION = v3.SUPPORTED_COMPRESSION
read_regular_file = v3.read_regular_file
_BASE_NORMALIZE = v2._normalize


def _normalize_preserving_file_header(
    data: bytes, limits: ReaderLimits
) -> tuple[bytes, list[str], int, dict[int, dict[str, int | str]], int]:
    original_header = v1._parse_record(data, len(MAGIC), limits, label="source file header")
    if original_header.op != OP_FILE_HEADER or set(original_header.fields) != {
        "op", "index_pos", "conn_count", "chunk_count"
    }:
        raise BagV2Error("source file-header fields differ")
    padding = bytes(original_header.data)
    if len(padding) > 4096 or (padding and set(padding) != {0x20}):
        raise BagV2Error("source file-header padding is not bounded ASCII spaces")

    saved_connection = v2._connection_identity
    v2._connection_identity = v3._connection_identity
    try:
        normalized, compressions, source_index_pos, witnesses, sentinel_count = _BASE_NORMALIZE(data, limits)
    finally:
        v2._connection_identity = saved_connection

    short_header = v1._parse_record(normalized, len(MAGIC), limits, label="normalized file header")
    padded_header = v2._record([
        ("op", bytes([OP_FILE_HEADER])),
        ("index_pos", bytes(short_header.fields["index_pos"])),
        ("conn_count", bytes(short_header.fields["conn_count"])),
        ("chunk_count", bytes(short_header.fields["chunk_count"])),
    ], padding)
    if len(padded_header) != original_header.end - original_header.offset:
        raise BagV2Error("normalized padded file-header length differs from source")
    rebuilt = MAGIC + padded_header + normalized[short_header.end:]

    rebuilt_header = v1._parse_record(rebuilt, len(MAGIC), limits, label="rebuilt file header")
    index_pos = v1._u64_field(rebuilt_header.fields, "index_pos", "rebuilt file header")
    boundaries: set[int] = set()
    offset = len(MAGIC)
    while offset < len(rebuilt):
        row = v1._parse_record(rebuilt, offset, limits, label="rebuilt top-level")
        boundaries.add(row.offset)
        offset = row.end
    if offset != len(rebuilt) or index_pos not in boundaries:
        raise BagV2Error("rebuilt file-header index_pos is not an exact record boundary")
    return rebuilt, compressions, source_index_pos, witnesses, sentinel_count


def parse_bag_v2(data: bytes, *, limits: ReaderLimits | None = None) -> dict[str, Any]:
    saved_normalize = v2._normalize
    saved_connection = v2._connection_identity
    v2._normalize = _normalize_preserving_file_header
    v2._connection_identity = v3._connection_identity
    try:
        return v2.parse_bag_v2(data, limits=limits)
    finally:
        v2._normalize = saved_normalize
        v2._connection_identity = saved_connection


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
