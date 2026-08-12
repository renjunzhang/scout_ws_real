#!/usr/bin/env python3
"""Create-new ROS1 Bag V2 reader remediation for chunk connection records.

ROS1 writers may repeat OP_CONNECTION records inside chunks before the first
message for a connection.  Revision v1 rejected every non-OP_MSG_DATA inner
record.  This revision accepts only byte-equivalent connection declarations,
proves the original indexes against original message offsets, removes the
redundant declarations in memory, and delegates the complete semantic census
to the frozen v1 reader.  It never repairs or writes the source bag.
"""

from __future__ import annotations

import bz2
import hashlib
import math
import struct
from pathlib import Path
from typing import Any

import r8_liquid_ros1_bag_v2_reader_v1 as v1


BagV2Error = v1.BagV2Error
ReaderLimits = v1.ReaderLimits
MAGIC = v1.MAGIC
OP_MSG_DATA = v1.OP_MSG_DATA
OP_FILE_HEADER = v1.OP_FILE_HEADER
OP_INDEX_DATA = v1.OP_INDEX_DATA
OP_CHUNK = v1.OP_CHUNK
OP_CHUNK_INFO = v1.OP_CHUNK_INFO
OP_CONNECTION = v1.OP_CONNECTION
SUPPORTED_COMPRESSION = v1.SUPPORTED_COMPRESSION
read_regular_file = v1.read_regular_file


def _decompress(compression: str, data: memoryview, announced: int, limits: ReaderLimits) -> bytes:
    if announced > limits.maximum_chunk_uncompressed_bytes:
        raise BagV2Error("chunk announced size exceeds the uncompressed bound")
    encoded = bytes(data)
    if compression == "none":
        if len(encoded) != announced:
            raise BagV2Error("uncompressed chunk size differs from its header")
        return encoded
    if compression == "bz2":
        try:
            decoder = bz2.BZ2Decompressor()
            result = decoder.decompress(encoded, max_length=announced + 1)
        except (OSError, EOFError) as exc:
            raise BagV2Error(f"invalid bz2 chunk: {exc}") from exc
        if len(result) != announced or not decoder.eof or decoder.unused_data:
            raise BagV2Error("bz2 chunk does not close at its exact announced size")
        return result
    raise BagV2Error(f"unsupported ROS1 Bag V2 compression: {compression!r}")


def _field(name: str, value: bytes) -> bytes:
    raw = name.encode("ascii") + b"=" + value
    return struct.pack("<I", len(raw)) + raw


def _record(fields: list[tuple[str, bytes]], data: bytes = b"") -> bytes:
    header = b"".join(_field(name, value) for name, value in fields)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def _connection_identity(
    record: Any, limits: ReaderLimits, label: str
) -> tuple[int, str, str, str, bytes, bytes]:
    if set(record.fields) != {"op", "conn", "topic"}:
        raise BagV2Error(f"{label} connection header fields differ")
    connection_id = v1._u32_field(record.fields, "conn", label)
    topic = v1._text_field(record.fields, "topic", label, maximum=limits.maximum_string_bytes)
    fields = v1._parse_fields(record.data, limits, label=f"{label} connection data")
    allowed = {"type", "md5sum", "message_definition", "callerid", "latching"}
    if not set(fields).issubset(allowed):
        raise BagV2Error(f"{label} connection data has unknown fields")
    message_type = v1._text_field(fields, "type", label, maximum=limits.maximum_string_bytes)
    md5sum = v1._text_field(fields, "md5sum", label, maximum=128)
    if len(md5sum) != 32 or any(char not in "0123456789abcdef" for char in md5sum):
        raise BagV2Error(f"{label} md5sum is invalid")
    definition = fields.get("message_definition")
    if not definition or len(definition) > limits.maximum_message_definition_bytes:
        raise BagV2Error(f"{label} message_definition is absent or oversized")
    # The final element binds every parsed data field, including callerid and
    # latching, byte-for-byte.  Canonical definition handling is deliberately
    # separate and is used only across distinct same-topic connections.
    return connection_id, topic, message_type, md5sum, bytes(definition), bytes(record.data)


def _canonical_definition(raw: bytes) -> bytes:
    """Remove only terminal CR/LF bytes; never strip spaces or rewrite text."""
    return raw.rstrip(b"\r\n")


def _sanitize_terminal_debug(payload: memoryview, limits: ReaderLimits) -> tuple[bytes, int]:
    """Validate Float32MultiArray structure and neutralize only QC sentinel values."""
    cursor = v1._PayloadCursor(payload, limits, label="/spmpc/terminal/debug")
    dimensions = cursor.u32("layout.dim length")
    if dimensions > limits.maximum_array_count:
        raise BagV2Error("terminal/debug dimension array exceeds the bound")
    prefix = bytearray(struct.pack("<I", dimensions))
    for index in range(dimensions):
        label = cursor.string(f"layout.dim[{index}].label")
        encoded = label.encode("utf-8")
        size = cursor.u32(f"layout.dim[{index}].size")
        stride = cursor.u32(f"layout.dim[{index}].stride")
        prefix.extend(struct.pack("<I", len(encoded)) + encoded + struct.pack("<II", size, stride))
    data_offset = cursor.u32("layout.data_offset")
    count = cursor.u32("data length")
    if count > limits.maximum_array_count:
        raise BagV2Error("terminal/debug data array exceeds the bound")
    raw = bytes(cursor.take(count * 4, "data"))
    cursor.require_eof()
    values = list(struct.unpack("<" + "f" * count, raw))
    nonfinite = sum(not math.isfinite(value) for value in values)
    sanitized = [value if math.isfinite(value) else 0.0 for value in values]
    prefix.extend(struct.pack("<II", data_offset, count))
    prefix.extend(struct.pack("<" + "f" * count, *sanitized))
    return bytes(prefix), nonfinite


def _index_entries(record: Any, limits: ReaderLimits) -> tuple[int, list[tuple[int, int]]]:
    if set(record.fields) != {"op", "ver", "conn", "count"}:
        raise BagV2Error("index-data header fields differ")
    if v1._u32_field(record.fields, "ver", "index data") != 1:
        raise BagV2Error("index-data version differs")
    connection_id = v1._u32_field(record.fields, "conn", "index data")
    count = v1._u32_field(record.fields, "count", "index data")
    if count > limits.maximum_messages or len(record.data) != count * 12:
        raise BagV2Error("index-data count/size differs")
    cursor = v1._Cursor(record.data, label="index data")
    entries: list[tuple[int, int]] = []
    for _ in range(count):
        sec, nsec, offset = struct.unpack("<III", cursor.take(12, "entry"))
        if nsec >= 1_000_000_000:
            raise BagV2Error("index-data timestamp nanoseconds are invalid")
        entries.append((sec * 1_000_000_000 + nsec, offset))
    cursor.require_eof()
    return connection_id, entries


def _encode_index(connection_id: int, entries: list[tuple[int, int]]) -> bytes:
    data = b"".join(
        struct.pack("<III", stamp // 1_000_000_000, stamp % 1_000_000_000, offset)
        for stamp, offset in entries
    )
    return _record([
        ("op", bytes([OP_INDEX_DATA])), ("ver", struct.pack("<I", 1)),
        ("conn", struct.pack("<I", connection_id)), ("count", struct.pack("<I", len(entries))),
    ], data)


def _normalize(
    data: bytes, limits: ReaderLimits
) -> tuple[bytes, list[str], int, dict[int, dict[str, int | str]], int]:
    if not data.startswith(MAGIC):
        raise BagV2Error("selected input is not ROS1 Bag V2")
    records = []
    offset = len(MAGIC)
    while offset < len(data):
        if len(records) >= limits.maximum_records:
            raise BagV2Error("top-level record count exceeds the bound")
        row = v1._parse_record(data, offset, limits, label="top-level")
        records.append(row)
        offset = row.end
    if offset != len(data) or not records or records[0].op != OP_FILE_HEADER:
        raise BagV2Error("bag lacks a leading file-header record")
    header = records[0]
    index_pos = v1._u64_field(header.fields, "index_pos", "file header")
    if index_pos not in {row.offset for row in records}:
        raise BagV2Error("file-header index_pos is not an exact record boundary")

    top_connections: dict[int, tuple[int, str, str, str, bytes, bytes]] = {}
    saw_chunk_info = False
    for row in records[1:]:
        if row.offset < index_pos:
            continue
        if row.op == OP_CONNECTION:
            if saw_chunk_info:
                raise BagV2Error("top-level connection appears after chunk-info")
            identity = _connection_identity(row, limits, "top-level")
            if identity[0] in top_connections:
                raise BagV2Error("duplicate top-level connection id")
            top_connections[identity[0]] = identity
        elif row.op == OP_CHUNK_INFO:
            saw_chunk_info = True
        else:
            raise BagV2Error("index section contains an unknown op")

    new_preindex: list[bytes] = []
    old_to_new_chunk: dict[int, int] = {}
    original_actual: dict[tuple[int, int], list[tuple[int, int]]] = {}
    normalized_actual: dict[tuple[int, int], list[tuple[int, int]]] = {}
    observed_indexes: dict[tuple[int, int], list[tuple[int, int]]] = {}
    active_old_chunk: int | None = None
    active_new_chunk: int | None = None
    original_compressions: set[str] = set()
    terminal_debug_nonfinite = 0
    prefix_size = len(MAGIC) + (header.end - header.offset)

    for row in records[1:]:
        if row.offset >= index_pos:
            break
        if row.op == OP_CHUNK:
            compression = v1._text_field(row.fields, "compression", "chunk", maximum=32)
            if compression not in SUPPORTED_COMPRESSION:
                raise BagV2Error(f"unsupported ROS1 Bag V2 compression: {compression!r}")
            original_compressions.add(compression)
            announced = v1._u32_field(row.fields, "size", "chunk")
            raw = _decompress(compression, row.data, announced, limits)
            output_parts: list[bytes] = []
            seen_message_connections: set[int] = set()
            inner_offset = 0
            new_inner_offset = 0
            while inner_offset < len(raw):
                inner = v1._parse_record(raw, inner_offset, limits, label="chunk record", maximum_data_bytes=limits.maximum_message_bytes)
                encoded = bytes(memoryview(raw)[inner.offset:inner.end])
                if inner.op == OP_CONNECTION:
                    identity = _connection_identity(inner, limits, "chunk")
                    expected = top_connections.get(identity[0])
                    if expected is None:
                        raise BagV2Error("chunk connection has no top-level declaration")
                    if identity != expected:
                        raise BagV2Error("chunk connection conflicts with top-level declaration")
                    if identity[0] in seen_message_connections:
                        raise BagV2Error("chunk connection appears after message data")
                elif inner.op == OP_MSG_DATA:
                    if set(inner.fields) != {"op", "conn", "time"}:
                        raise BagV2Error("message header fields differ")
                    connection_id = v1._u32_field(inner.fields, "conn", "message")
                    if connection_id not in top_connections:
                        raise BagV2Error("message references an undeclared connection")
                    stamp = v1._time_field_ns(inner.fields, "time", "message")
                    original_actual.setdefault((row.offset, connection_id), []).append((stamp, inner.offset))
                    normalized_actual.setdefault((row.offset, connection_id), []).append((stamp, new_inner_offset))
                    connection = top_connections[connection_id]
                    if connection[1:3] == ("/spmpc/terminal/debug", "std_msgs/Float32MultiArray"):
                        sanitized, count = _sanitize_terminal_debug(inner.data, limits)
                        terminal_debug_nonfinite += count
                        encoded = _record([
                            ("op", bytes([OP_MSG_DATA])), ("conn", struct.pack("<I", connection_id)),
                            ("time", bytes(inner.fields["time"])),
                        ], sanitized)
                    output_parts.append(encoded)
                    new_inner_offset += len(encoded)
                    seen_message_connections.add(connection_id)
                else:
                    raise BagV2Error(f"chunk contains unknown op 0x{inner.op:02x}")
                inner_offset = inner.end
            normalized = b"".join(output_parts)
            # Delegate through the frozen v1 semantic parser using an
            # uncompressed in-memory representation.  The returned census is
            # restored to the source compression identity below; the source
            # bytes are never changed.
            payload = normalized
            encoded_chunk = _record([
                ("op", bytes([OP_CHUNK])), ("compression", b"none"),
                ("size", struct.pack("<I", len(normalized))),
            ], payload)
            new_chunk_pos = prefix_size + sum(len(part) for part in new_preindex)
            old_to_new_chunk[row.offset] = new_chunk_pos
            new_preindex.append(encoded_chunk)
            active_old_chunk = row.offset
            active_new_chunk = new_chunk_pos
        elif row.op == OP_INDEX_DATA:
            if active_old_chunk is None or active_new_chunk is None:
                raise BagV2Error("index-data record appears before a chunk")
            connection_id, entries = _index_entries(row, limits)
            key = (active_old_chunk, connection_id)
            if key in observed_indexes:
                raise BagV2Error("duplicate index-data record for a chunk/connection")
            observed_indexes[key] = entries
            normalized_entries = normalized_actual.get(key)
            if normalized_entries is None:
                raise BagV2Error("index-data references a chunk/connection without messages")
            new_preindex.append(_encode_index(connection_id, normalized_entries))
        else:
            raise BagV2Error("pre-index section contains an unknown or misplaced op")
    if observed_indexes != original_actual:
        raise BagV2Error("index-data entries drift from original chunk messages")

    index_pos_new = prefix_size + sum(len(part) for part in new_preindex)
    file_header = _record([
        ("op", bytes([OP_FILE_HEADER])), ("index_pos", struct.pack("<Q", index_pos_new)),
        ("conn_count", struct.pack("<I", v1._u32_field(header.fields, "conn_count", "file header"))),
        ("chunk_count", struct.pack("<I", v1._u32_field(header.fields, "chunk_count", "file header"))),
    ])
    postindex: list[bytes] = []
    for row in records[1:]:
        if row.offset < index_pos:
            continue
        if row.op == OP_CONNECTION:
            identity = top_connections[v1._u32_field(row.fields, "conn", "connection")]
            fields = v1._parse_fields(row.data, limits, label="top-level connection data")
            normalized_fields = [
                ("type", fields["type"]), ("md5sum", fields["md5sum"]),
                ("message_definition", _canonical_definition(fields["message_definition"])),
            ]
            for optional in ("callerid", "latching"):
                if optional in fields:
                    normalized_fields.append((optional, fields[optional]))
            postindex.append(_record([
                ("op", bytes([OP_CONNECTION])), ("conn", struct.pack("<I", identity[0])),
                ("topic", identity[1].encode("utf-8")),
            ], b"".join(_field(name, value) for name, value in normalized_fields)))
        elif row.op == OP_CHUNK_INFO:
            old_chunk = v1._u64_field(row.fields, "chunk_pos", "chunk info")
            if old_chunk not in old_to_new_chunk:
                raise BagV2Error("chunk-info references an unknown chunk")
            postindex.append(_record([
                ("op", bytes([OP_CHUNK_INFO])),
                ("ver", struct.pack("<I", v1._u32_field(row.fields, "ver", "chunk info"))),
                ("chunk_pos", struct.pack("<Q", old_to_new_chunk[old_chunk])),
                ("start_time", bytes(row.fields["start_time"])),
                ("end_time", bytes(row.fields["end_time"])),
                ("count", struct.pack("<I", v1._u32_field(row.fields, "count", "chunk info"))),
            ], bytes(row.data)))
    witnesses = {
        connection_id: {
            "raw_sha256": hashlib.sha256(identity[4]).hexdigest(),
            "raw_size_bytes": len(identity[4]),
            "canonical_sha256": hashlib.sha256(_canonical_definition(identity[4])).hexdigest(),
            "canonical_size_bytes": len(_canonical_definition(identity[4])),
        }
        for connection_id, identity in top_connections.items()
    }
    return (
        MAGIC + file_header + b"".join(new_preindex + postindex),
        sorted(original_compressions), index_pos, witnesses, terminal_debug_nonfinite,
    )


def parse_bag_v2(data: bytes, *, limits: ReaderLimits | None = None) -> dict[str, Any]:
    selected = limits or ReaderLimits()
    selected.validate()
    if len(data) > selected.maximum_file_bytes:
        raise BagV2Error("bag byte string exceeds the file bound")
    normalized, source_compressions, source_index_pos, witnesses, sentinel_count = _normalize(data, selected)
    result = v1.parse_bag_v2(normalized, limits=selected)
    result["compression"] = source_compressions
    result["file_header"]["index_pos"] = source_index_pos
    for connection in result["connections"]:
        witness = witnesses[connection["connection_id"]]
        connection["message_definition_sha256"] = witness["raw_sha256"]
        connection["message_definition_size_bytes"] = witness["raw_size_bytes"]
        connection["canonical_message_definition_sha256"] = witness["canonical_sha256"]
        connection["canonical_message_definition_size_bytes"] = witness["canonical_size_bytes"]
    result["qc_nonfinite_sentinel_count"] = sentinel_count
    return result


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
        path, limits=selected, expected_sha256=expected_sha256,
        expected_size_bytes=expected_size_bytes, expected_mode=expected_mode,
    )
    return {"source_before": before, "source_after": after, "bag": parse_bag_v2(data, limits=selected)}


__all__ = [
    "BagV2Error", "MAGIC", "OP_CHUNK", "OP_CHUNK_INFO", "OP_CONNECTION",
    "OP_FILE_HEADER", "OP_INDEX_DATA", "OP_MSG_DATA", "ReaderLimits",
    "SUPPORTED_COMPRESSION", "inspect_bag", "parse_bag_v2", "read_regular_file",
]
