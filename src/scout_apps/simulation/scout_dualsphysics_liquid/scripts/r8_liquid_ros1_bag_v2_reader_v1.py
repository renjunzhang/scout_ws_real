#!/usr/bin/env python3
"""Bounded, dependency-free, read-only ROS1 Bag V2 reader for R8 S5A0.

The reader implements only the ROS1 Bag V2 records needed by the selected-bag
intake.  It never imports ROS, executes a bag, repairs an index, creates a
cache, or writes a file.  Inputs are read through ``O_NOFOLLOW`` and are
bounded before parsing or decompression.

Supported chunk compression is ``none`` and ``bz2``.  Every other compression
name fails closed.  The selected corpus must therefore be admitted only after
its observed compression appears in this reader's explicit support set.
"""

from __future__ import annotations

import bz2
import hashlib
import math
import os
import stat
import struct
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


MAGIC = b"#ROSBAG V2.0\n"

OP_MSG_DATA = 0x02
OP_FILE_HEADER = 0x03
OP_INDEX_DATA = 0x04
OP_CHUNK = 0x05
OP_CHUNK_INFO = 0x06
OP_CONNECTION = 0x07

SUPPORTED_COMPRESSION = frozenset({"none", "bz2"})


class BagV2Error(ValueError):
    """The bag, its filesystem identity, or a bounded payload is invalid."""


@dataclass(frozen=True)
class ReaderLimits:
    maximum_file_bytes: int = 64 * 1024 * 1024
    maximum_record_header_bytes: int = 1024 * 1024
    maximum_record_data_bytes: int = 64 * 1024 * 1024
    maximum_field_bytes: int = 4 * 1024 * 1024
    maximum_message_bytes: int = 16 * 1024 * 1024
    maximum_message_definition_bytes: int = 2 * 1024 * 1024
    maximum_chunk_uncompressed_bytes: int = 128 * 1024 * 1024
    maximum_total_uncompressed_bytes: int = 256 * 1024 * 1024
    maximum_records: int = 1_000_000
    maximum_messages: int = 1_000_000
    maximum_connections: int = 4096
    maximum_chunks: int = 4096
    maximum_array_count: int = 1_000_000
    maximum_string_bytes: int = 1024 * 1024

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise BagV2Error(f"reader limit {name} must be a positive integer")
        if self.maximum_message_bytes > self.maximum_record_data_bytes:
            raise BagV2Error("message limit exceeds record-data limit")
        if self.maximum_chunk_uncompressed_bytes > self.maximum_total_uncompressed_bytes:
            raise BagV2Error("single-chunk limit exceeds total-uncompressed limit")


@dataclass(frozen=True)
class _Record:
    offset: int
    end: int
    fields: dict[str, bytes]
    data: memoryview

    @property
    def op(self) -> int:
        raw = self.fields.get("op")
        if raw is None or len(raw) != 1:
            raise BagV2Error(f"record at {self.offset} lacks a one-byte op")
        return raw[0]


@dataclass(frozen=True)
class _Message:
    chunk_pos: int
    offset: int
    connection_id: int
    record_time_ns: int
    payload: memoryview


class _Cursor:
    def __init__(self, data: bytes | memoryview, *, label: str):
        self.data = memoryview(data)
        self.offset = 0
        self.label = label

    @property
    def remaining(self) -> int:
        return len(self.data) - self.offset

    def take(self, count: int, field: str) -> memoryview:
        if count < 0 or count > self.remaining:
            raise BagV2Error(
                f"truncated {self.label} {field}: need {count}, have {self.remaining}"
            )
        start = self.offset
        self.offset += count
        return self.data[start : self.offset]

    def u32(self, field: str) -> int:
        return struct.unpack("<I", self.take(4, field))[0]

    def require_eof(self) -> None:
        if self.remaining:
            raise BagV2Error(f"{self.label} has {self.remaining} trailing bytes")


class _PayloadCursor(_Cursor):
    def __init__(self, data: bytes | memoryview, limits: ReaderLimits, *, label: str):
        super().__init__(data, label=label)
        self.limits = limits

    def time_ns(self, field: str) -> int:
        raw = self.take(8, field)
        sec, nsec = struct.unpack("<II", raw)
        if nsec >= 1_000_000_000:
            raise BagV2Error(f"{self.label} {field} nanoseconds are invalid")
        return sec * 1_000_000_000 + nsec

    def string(self, field: str) -> str:
        count = self.u32(f"{field} length")
        if count > self.limits.maximum_string_bytes:
            raise BagV2Error(f"{self.label} {field} exceeds the string limit")
        raw = bytes(self.take(count, field))
        try:
            return raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise BagV2Error(f"{self.label} {field} is not UTF-8") from exc

    def doubles(self, count: int, field: str) -> tuple[float, ...]:
        if count > self.limits.maximum_array_count:
            raise BagV2Error(f"{self.label} {field} exceeds the array limit")
        values = struct.unpack("<" + "d" * count, self.take(8 * count, field))
        if any(not math.isfinite(value) for value in values):
            raise BagV2Error(f"{self.label} {field} contains a non-finite value")
        return values

    def floats(self, count: int, field: str) -> tuple[float, ...]:
        if count > self.limits.maximum_array_count:
            raise BagV2Error(f"{self.label} {field} exceeds the array limit")
        values = struct.unpack("<" + "f" * count, self.take(4 * count, field))
        if any(not math.isfinite(value) for value in values):
            raise BagV2Error(f"{self.label} {field} contains a non-finite value")
        return values


def _normalise_sha256(value: str | None) -> str | None:
    if value is None:
        return None
    result = value.lower()
    if len(result) != 64 or any(char not in "0123456789abcdef" for char in result):
        raise BagV2Error("expected SHA-256 must be 64 lowercase hexadecimal characters")
    return result


def _stat_identity(path: Path, metadata: os.stat_result, digest: str) -> dict[str, Any]:
    return {
        "path": str(path),
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
        "mtime_ns": metadata.st_mtime_ns,
        "ctime_ns": metadata.st_ctime_ns,
    }


def _identity_tuple(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_mode,
        metadata.st_uid,
        metadata.st_gid,
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_nlink,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def read_regular_file(
    path: str | os.PathLike[str],
    *,
    limits: ReaderLimits | None = None,
    expected_sha256: str | None = None,
    expected_size_bytes: int | None = None,
    expected_mode: int | None = None,
) -> tuple[bytes, dict[str, Any], dict[str, Any]]:
    """Read one stable regular file without following its final component."""

    selected_limits = limits or ReaderLimits()
    selected_limits.validate()
    expected_digest = _normalise_sha256(expected_sha256)
    selected = Path(path)
    if not selected.is_absolute() or ".." in selected.parts:
        raise BagV2Error("bag path must be absolute and lexically normalized")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(selected, flags)
    except OSError as exc:
        raise BagV2Error(f"cannot securely open selected bag {selected}: {exc}") from exc
    try:
        before_stat = os.fstat(descriptor)
        if not stat.S_ISREG(before_stat.st_mode):
            raise BagV2Error("selected bag is not a regular file")
        if before_stat.st_nlink != 1:
            raise BagV2Error("selected bag link count is not exactly one")
        if before_stat.st_size <= 0 or before_stat.st_size > selected_limits.maximum_file_bytes:
            raise BagV2Error("selected bag size is outside the bounded reader limit")
        if expected_size_bytes is not None and before_stat.st_size != expected_size_bytes:
            raise BagV2Error(
                f"selected bag size differs: expected {expected_size_bytes}, got {before_stat.st_size}"
            )
        if expected_mode is not None and stat.S_IMODE(before_stat.st_mode) != expected_mode:
            raise BagV2Error(
                f"selected bag mode differs: expected {expected_mode:04o}, "
                f"got {stat.S_IMODE(before_stat.st_mode):04o}"
            )
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        remaining = before_stat.st_size
        while remaining:
            chunk = os.read(descriptor, min(remaining, 1024 * 1024))
            if not chunk:
                raise BagV2Error("selected bag produced a short read")
            digest.update(chunk)
            chunks.append(chunk)
            remaining -= len(chunk)
        after_fd_stat = os.fstat(descriptor)
        if _identity_tuple(after_fd_stat) != _identity_tuple(before_stat):
            raise BagV2Error("selected bag identity changed while its descriptor was open")
    finally:
        os.close(descriptor)

    try:
        after_path_stat = os.lstat(selected)
    except OSError as exc:
        raise BagV2Error(f"cannot restat selected bag after read: {exc}") from exc
    if _identity_tuple(after_path_stat) != _identity_tuple(before_stat):
        raise BagV2Error("selected bag path identity changed during read")
    actual_digest = digest.hexdigest()
    if expected_digest is not None and actual_digest != expected_digest:
        raise BagV2Error(
            f"selected bag SHA-256 differs: expected {expected_digest}, got {actual_digest}"
        )
    data = b"".join(chunks)
    return (
        data,
        _stat_identity(selected, before_stat, actual_digest),
        _stat_identity(selected, after_path_stat, actual_digest),
    )


def _parse_fields(
    raw: bytes | memoryview, limits: ReaderLimits, *, label: str
) -> dict[str, bytes]:
    cursor = _Cursor(raw, label=label)
    fields: dict[str, bytes] = {}
    while cursor.remaining:
        length = cursor.u32("field length")
        if length <= 1 or length > limits.maximum_field_bytes:
            raise BagV2Error(f"{label} contains an invalid field length {length}")
        field = bytes(cursor.take(length, "field"))
        separator = field.find(b"=")
        if separator <= 0:
            raise BagV2Error(f"{label} contains a malformed field")
        try:
            key = field[:separator].decode("ascii")
        except UnicodeDecodeError as exc:
            raise BagV2Error(f"{label} contains a non-ASCII field name") from exc
        if not key.replace("_", "").isalnum() or key in fields:
            raise BagV2Error(f"{label} contains an invalid or duplicate field {key!r}")
        fields[key] = field[separator + 1 :]
    return fields


def _parse_record(
    data: bytes | memoryview,
    offset: int,
    limits: ReaderLimits,
    *,
    label: str,
    maximum_data_bytes: int | None = None,
) -> _Record:
    view = memoryview(data)
    if offset < 0 or offset + 4 > len(view):
        raise BagV2Error(f"truncated {label} record length at {offset}")
    header_length = struct.unpack_from("<I", view, offset)[0]
    if header_length <= 0 or header_length > limits.maximum_record_header_bytes:
        raise BagV2Error(f"{label} record header length is invalid at {offset}")
    header_start = offset + 4
    header_end = header_start + header_length
    if header_end + 4 > len(view):
        raise BagV2Error(f"truncated {label} record header at {offset}")
    data_length = struct.unpack_from("<I", view, header_end)[0]
    maximum = maximum_data_bytes or limits.maximum_record_data_bytes
    if data_length > maximum:
        raise BagV2Error(f"{label} record data exceeds its bound at {offset}")
    data_start = header_end + 4
    end = data_start + data_length
    if end > len(view):
        raise BagV2Error(f"truncated {label} record data at {offset}")
    fields = _parse_fields(view[header_start:header_end], limits, label=f"{label} header")
    return _Record(offset, end, fields, view[data_start:end])


def _u32_field(fields: dict[str, bytes], name: str, label: str) -> int:
    raw = fields.get(name)
    if raw is None or len(raw) != 4:
        raise BagV2Error(f"{label} field {name!r} is not uint32")
    return struct.unpack("<I", raw)[0]


def _u64_field(fields: dict[str, bytes], name: str, label: str) -> int:
    raw = fields.get(name)
    if raw is None or len(raw) != 8:
        raise BagV2Error(f"{label} field {name!r} is not uint64")
    return struct.unpack("<Q", raw)[0]


def _time_field_ns(fields: dict[str, bytes], name: str, label: str) -> int:
    raw = fields.get(name)
    if raw is None or len(raw) != 8:
        raise BagV2Error(f"{label} field {name!r} is not ROS time")
    sec, nsec = struct.unpack("<II", raw)
    if nsec >= 1_000_000_000:
        raise BagV2Error(f"{label} field {name!r} has invalid nanoseconds")
    return sec * 1_000_000_000 + nsec


def _text_field(
    fields: dict[str, bytes], name: str, label: str, *, maximum: int
) -> str:
    raw = fields.get(name)
    if raw is None or not raw or len(raw) > maximum:
        raise BagV2Error(f"{label} field {name!r} is absent, empty, or oversized")
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise BagV2Error(f"{label} field {name!r} is not UTF-8") from exc


def _decompress_chunk(
    compression: str, data: memoryview, announced_size: int, limits: ReaderLimits
) -> bytes:
    if announced_size > limits.maximum_chunk_uncompressed_bytes:
        raise BagV2Error("chunk announced size exceeds the uncompressed bound")
    compressed = bytes(data)
    if compression == "none":
        if len(compressed) != announced_size:
            raise BagV2Error("uncompressed chunk size differs from its header")
        return compressed
    if compression == "bz2":
        try:
            decoder = bz2.BZ2Decompressor()
            result = decoder.decompress(compressed, max_length=announced_size + 1)
        except (OSError, EOFError) as exc:
            raise BagV2Error(f"invalid bz2 chunk: {exc}") from exc
        if (
            len(result) != announced_size
            or not decoder.eof
            or decoder.unused_data
            or not decoder.needs_input
        ):
            raise BagV2Error("bz2 chunk does not close at its exact announced size")
        return result
    raise BagV2Error(f"unsupported ROS1 Bag V2 compression: {compression!r}")


def _header(cursor: _PayloadCursor, label: str) -> tuple[int, str]:
    cursor.u32(f"{label}.seq")
    stamp_ns = cursor.time_ns(f"{label}.stamp")
    frame_id = cursor.string(f"{label}.frame_id")
    return stamp_ns, frame_id


def _decode_odometry(
    payload: memoryview, limits: ReaderLimits
) -> dict[str, Any]:
    cursor = _PayloadCursor(payload, limits, label="nav_msgs/Odometry")
    header_time_ns, frame_id = _header(cursor, "header")
    child_frame_id = cursor.string("child_frame_id")
    values = cursor.doubles(85, "pose/twist/covariances")
    quaternion = values[3:7]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise BagV2Error("nav_msgs/Odometry contains a zero quaternion")
    cursor.require_eof()
    return {
        "header_time_ns": header_time_ns,
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
    }


def _decode_clock(payload: memoryview, limits: ReaderLimits) -> int:
    cursor = _PayloadCursor(payload, limits, label="rosgraph_msgs/Clock")
    value = cursor.time_ns("clock")
    cursor.require_eof()
    return value


def _decode_tf(
    payload: memoryview, limits: ReaderLimits
) -> list[dict[str, Any]]:
    cursor = _PayloadCursor(payload, limits, label="tf2_msgs/TFMessage")
    count = cursor.u32("transforms length")
    if count > limits.maximum_array_count:
        raise BagV2Error("TF transform array exceeds the bound")
    result: list[dict[str, Any]] = []
    for index in range(count):
        stamp_ns, parent = _header(cursor, f"transforms[{index}].header")
        child = cursor.string(f"transforms[{index}].child_frame_id")
        values = cursor.doubles(7, f"transforms[{index}].transform")
        quaternion = values[3:7]
        if math.sqrt(sum(value * value for value in quaternion)) <= 1e-12:
            raise BagV2Error("TF transform contains a zero quaternion")
        result.append({"stamp_ns": stamp_ns, "parent": parent, "child": child})
    cursor.require_eof()
    return result


def _decode_twist(payload: memoryview, limits: ReaderLimits) -> None:
    cursor = _PayloadCursor(payload, limits, label="geometry_msgs/Twist")
    cursor.doubles(6, "linear/angular")
    cursor.require_eof()


def _decode_pose_stamped(cursor: _PayloadCursor, label: str) -> None:
    _header(cursor, f"{label}.header")
    values = cursor.doubles(7, f"{label}.pose")
    if math.sqrt(sum(value * value for value in values[3:7])) <= 1e-12:
        raise BagV2Error(f"{label} contains a zero quaternion")


def _decode_path(payload: memoryview, limits: ReaderLimits) -> None:
    cursor = _PayloadCursor(payload, limits, label="nav_msgs/Path")
    _header(cursor, "header")
    count = cursor.u32("poses length")
    if count > limits.maximum_array_count:
        raise BagV2Error("Path pose array exceeds the bound")
    for index in range(count):
        _decode_pose_stamped(cursor, f"poses[{index}]")
    cursor.require_eof()


def _decode_float32(payload: memoryview, limits: ReaderLimits, label: str) -> None:
    cursor = _PayloadCursor(payload, limits, label=label)
    cursor.floats(1, "data")
    cursor.require_eof()


def _decode_string(payload: memoryview, limits: ReaderLimits, label: str) -> None:
    cursor = _PayloadCursor(payload, limits, label=label)
    cursor.string("data")
    cursor.require_eof()


def _decode_float32_multi_array(
    payload: memoryview, limits: ReaderLimits, label: str
) -> None:
    cursor = _PayloadCursor(payload, limits, label=label)
    dimensions = cursor.u32("layout.dim length")
    if dimensions > limits.maximum_array_count:
        raise BagV2Error(f"{label} dimension array exceeds the bound")
    for index in range(dimensions):
        cursor.string(f"layout.dim[{index}].label")
        cursor.u32(f"layout.dim[{index}].size")
        cursor.u32(f"layout.dim[{index}].stride")
    cursor.u32("layout.data_offset")
    count = cursor.u32("data length")
    cursor.floats(count, "data")
    cursor.require_eof()


def _decode_registered_payload(
    topic: str, message_type: str, payload: memoryview, limits: ReaderLimits
) -> dict[str, Any] | None:
    if message_type == "nav_msgs/Odometry" and topic == "/odom":
        return _decode_odometry(payload, limits)
    if message_type == "rosgraph_msgs/Clock" and topic == "/clock":
        return {"clock_time_ns": _decode_clock(payload, limits)}
    if message_type == "tf2_msgs/TFMessage" and topic in {"/tf", "/tf_static"}:
        return {"transforms": _decode_tf(payload, limits)}
    if message_type == "geometry_msgs/Twist" and topic == "/cmd_vel":
        _decode_twist(payload, limits)
    elif message_type == "geometry_msgs/PoseStamped" and topic == "/scout/goal":
        cursor = _PayloadCursor(payload, limits, label="geometry_msgs/PoseStamped")
        _decode_pose_stamped(cursor, "pose_stamped")
        cursor.require_eof()
    elif message_type == "nav_msgs/Path" and topic == "/scout/global_path_fixed":
        _decode_path(payload, limits)
    elif message_type == "std_msgs/Float32" and topic in {
        "/slosh/height",
        "/spmpc/slosh_height",
    }:
        _decode_float32(payload, limits, topic)
    elif message_type == "std_msgs/String" and topic in {
        "/spmpc/status",
        "/spmpc/terminal/mode",
    }:
        _decode_string(payload, limits, topic)
    elif message_type == "std_msgs/Float32MultiArray" and topic in {
        "/spmpc/debug/effective_config",
        "/spmpc/terminal/debug",
    }:
        _decode_float32_multi_array(payload, limits, topic)
    return None


def _time_range(values: Iterable[int]) -> dict[str, Any] | None:
    sequence = list(values)
    if not sequence:
        return None
    return {
        "first_ns": sequence[0],
        "last_ns": sequence[-1],
        "minimum_ns": min(sequence),
        "maximum_ns": max(sequence),
        "monotonic": all(right >= left for left, right in zip(sequence, sequence[1:])),
    }


def _edge_summary(
    observations: list[tuple[str, str, int]]
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[int]] = {}
    for parent, child, stamp in observations:
        grouped.setdefault((parent, child), []).append(stamp)
    return [
        {
            "parent_frame": parent,
            "child_frame": child,
            "message_count": len(times),
            "first_record_time_ns": times[0],
            "last_record_time_ns": times[-1],
        }
        for (parent, child), times in sorted(grouped.items())
    ]


def parse_bag_v2(data: bytes, *, limits: ReaderLimits | None = None) -> dict[str, Any]:
    """Parse and cross-check a complete indexed ROS1 Bag V2 byte string."""

    selected_limits = limits or ReaderLimits()
    selected_limits.validate()
    if len(data) > selected_limits.maximum_file_bytes:
        raise BagV2Error("bag byte string exceeds the file bound")
    if not data.startswith(MAGIC):
        raise BagV2Error("selected input is not ROS1 Bag V2")

    records: list[_Record] = []
    offset = len(MAGIC)
    while offset < len(data):
        if len(records) >= selected_limits.maximum_records:
            raise BagV2Error("top-level record count exceeds the bound")
        record = _parse_record(data, offset, selected_limits, label="top-level")
        records.append(record)
        offset = record.end
    if offset != len(data) or not records or records[0].op != OP_FILE_HEADER:
        raise BagV2Error("bag lacks a leading file-header record")
    if any(record.op == OP_FILE_HEADER for record in records[1:]):
        raise BagV2Error("bag contains more than one file-header record")

    file_header = records[0]
    index_pos = _u64_field(file_header.fields, "index_pos", "file header")
    connection_count = _u32_field(file_header.fields, "conn_count", "file header")
    chunk_count = _u32_field(file_header.fields, "chunk_count", "file header")
    if connection_count > selected_limits.maximum_connections:
        raise BagV2Error("file-header connection count exceeds the bound")
    if chunk_count <= 0 or chunk_count > selected_limits.maximum_chunks:
        raise BagV2Error("file-header chunk count is invalid")
    record_offsets = {record.offset for record in records}
    if index_pos not in record_offsets or index_pos <= file_header.end:
        raise BagV2Error("file-header index_pos is not an exact record boundary")

    chunks: dict[int, dict[str, Any]] = {}
    messages: list[_Message] = []
    indexes: dict[tuple[int, int], list[tuple[int, int]]] = {}
    connections: dict[int, dict[str, Any]] = {}
    chunk_infos: dict[int, dict[str, Any]] = {}
    active_chunk_pos: int | None = None
    total_uncompressed = 0
    compression_names: set[str] = set()

    for record in records[1:]:
        before_index = record.offset < index_pos
        if before_index and record.op not in {OP_CHUNK, OP_INDEX_DATA}:
            raise BagV2Error("pre-index section contains a non-chunk/index record")
        if not before_index and record.op not in {OP_CONNECTION, OP_CHUNK_INFO}:
            raise BagV2Error("index section contains a non-connection/chunk-info record")

        if record.op == OP_CHUNK:
            if len(chunks) >= selected_limits.maximum_chunks:
                raise BagV2Error("chunk count exceeds the bound")
            compression = _text_field(
                record.fields,
                "compression",
                "chunk",
                maximum=32,
            )
            if compression not in SUPPORTED_COMPRESSION:
                raise BagV2Error(f"unsupported ROS1 Bag V2 compression: {compression!r}")
            announced_size = _u32_field(record.fields, "size", "chunk")
            uncompressed = _decompress_chunk(
                compression, record.data, announced_size, selected_limits
            )
            total_uncompressed += len(uncompressed)
            if total_uncompressed > selected_limits.maximum_total_uncompressed_bytes:
                raise BagV2Error("total uncompressed chunk bytes exceed the bound")
            chunk_messages: list[_Message] = []
            inner_offset = 0
            while inner_offset < len(uncompressed):
                if len(messages) >= selected_limits.maximum_messages:
                    raise BagV2Error("message count exceeds the bound")
                inner = _parse_record(
                    uncompressed,
                    inner_offset,
                    selected_limits,
                    label="chunk message",
                    maximum_data_bytes=selected_limits.maximum_message_bytes,
                )
                if inner.op != OP_MSG_DATA:
                    raise BagV2Error("chunk contains a non-message record")
                connection_id = _u32_field(inner.fields, "conn", "message")
                record_time_ns = _time_field_ns(inner.fields, "time", "message")
                message = _Message(
                    record.offset,
                    inner.offset,
                    connection_id,
                    record_time_ns,
                    inner.data,
                )
                chunk_messages.append(message)
                messages.append(message)
                inner_offset = inner.end
            if inner_offset != len(uncompressed):
                raise BagV2Error("chunk message stream does not end exactly")
            chunks[record.offset] = {
                "compression": compression,
                "uncompressed_size": announced_size,
                "messages": chunk_messages,
            }
            compression_names.add(compression)
            active_chunk_pos = record.offset
        elif record.op == OP_INDEX_DATA:
            if active_chunk_pos is None:
                raise BagV2Error("index-data record appears before a chunk")
            version = _u32_field(record.fields, "ver", "index data")
            connection_id = _u32_field(record.fields, "conn", "index data")
            count = _u32_field(record.fields, "count", "index data")
            if version != 1 or count > selected_limits.maximum_messages:
                raise BagV2Error("index-data version/count is invalid")
            if len(record.data) != count * 12:
                raise BagV2Error("index-data payload size differs from count")
            key = (active_chunk_pos, connection_id)
            if key in indexes:
                raise BagV2Error("duplicate index-data record for a chunk/connection")
            entries = []
            cursor = _Cursor(record.data, label="index data")
            for _ in range(count):
                sec, nsec, message_offset = struct.unpack(
                    "<III", cursor.take(12, "entry")
                )
                if nsec >= 1_000_000_000:
                    raise BagV2Error("index-data timestamp nanoseconds are invalid")
                entries.append((sec * 1_000_000_000 + nsec, message_offset))
            cursor.require_eof()
            indexes[key] = entries
        elif record.op == OP_CONNECTION:
            connection_id = _u32_field(record.fields, "conn", "connection")
            if connection_id in connections:
                raise BagV2Error(f"duplicate connection id {connection_id}")
            topic = _text_field(
                record.fields,
                "topic",
                "connection",
                maximum=selected_limits.maximum_string_bytes,
            )
            connection_fields = _parse_fields(
                record.data, selected_limits, label=f"connection {connection_id} data"
            )
            message_type = _text_field(
                connection_fields,
                "type",
                "connection data",
                maximum=selected_limits.maximum_string_bytes,
            )
            md5sum = _text_field(
                connection_fields,
                "md5sum",
                "connection data",
                maximum=128,
            )
            if len(md5sum) != 32 or any(char not in "0123456789abcdef" for char in md5sum):
                raise BagV2Error("connection md5sum is not lowercase hexadecimal")
            definition = connection_fields.get("message_definition")
            if (
                definition is None
                or not definition
                or len(definition) > selected_limits.maximum_message_definition_bytes
            ):
                raise BagV2Error("connection message definition is absent or oversized")
            callerid_raw = connection_fields.get("callerid")
            latching_raw = connection_fields.get("latching")
            callerid = None
            if callerid_raw is not None:
                try:
                    callerid = callerid_raw.decode("utf-8")
                except UnicodeDecodeError as exc:
                    raise BagV2Error("connection callerid is not UTF-8") from exc
            latching = None
            if latching_raw is not None:
                if latching_raw not in {b"0", b"1"}:
                    raise BagV2Error("connection latching flag is invalid")
                latching = latching_raw == b"1"
            connections[connection_id] = {
                "connection_id": connection_id,
                "topic": topic,
                "type": message_type,
                "md5sum": md5sum,
                "message_definition_sha256": hashlib.sha256(definition).hexdigest(),
                "callerid": callerid,
                "latching": latching,
            }
        elif record.op == OP_CHUNK_INFO:
            version = _u32_field(record.fields, "ver", "chunk info")
            chunk_pos = _u64_field(record.fields, "chunk_pos", "chunk info")
            start_time_ns = _time_field_ns(record.fields, "start_time", "chunk info")
            end_time_ns = _time_field_ns(record.fields, "end_time", "chunk info")
            count = _u32_field(record.fields, "count", "chunk info")
            if version != 1 or chunk_pos in chunk_infos or count > selected_limits.maximum_connections:
                raise BagV2Error("chunk-info version/identity/count is invalid")
            if len(record.data) != count * 8:
                raise BagV2Error("chunk-info payload size differs from count")
            counts: dict[int, int] = {}
            cursor = _Cursor(record.data, label="chunk info")
            for _ in range(count):
                connection_id, message_count = struct.unpack(
                    "<II", cursor.take(8, "connection count")
                )
                if connection_id in counts:
                    raise BagV2Error("chunk info repeats a connection id")
                counts[connection_id] = message_count
            cursor.require_eof()
            chunk_infos[chunk_pos] = {
                "start_time_ns": start_time_ns,
                "end_time_ns": end_time_ns,
                "counts": counts,
            }

    if len(connections) != connection_count:
        raise BagV2Error("parsed connection count differs from file header")
    if len(chunks) != chunk_count or len(chunk_infos) != chunk_count:
        raise BagV2Error("parsed chunk/chunk-info count differs from file header")
    if not messages:
        raise BagV2Error("bag contains no messages")

    topic_definitions: dict[str, set[tuple[str, str, str]]] = {}
    for connection in connections.values():
        topic_definitions.setdefault(connection["topic"], set()).add(
            (
                connection["type"],
                connection["md5sum"],
                connection["message_definition_sha256"],
            )
        )
    conflicting_topics = sorted(
        topic for topic, definitions in topic_definitions.items() if len(definitions) != 1
    )
    if conflicting_topics:
        raise BagV2Error(
            "same-topic connection definitions conflict: " + ", ".join(conflicting_topics)
        )

    actual_by_chunk_connection: dict[tuple[int, int], list[tuple[int, int]]] = {}
    actual_counts: dict[int, int] = {}
    for message in messages:
        if message.connection_id not in connections:
            raise BagV2Error(
                f"message references undeclared connection {message.connection_id}"
            )
        actual_by_chunk_connection.setdefault(
            (message.chunk_pos, message.connection_id), []
        ).append((message.record_time_ns, message.offset))
        actual_counts[message.connection_id] = actual_counts.get(message.connection_id, 0) + 1
    if indexes != actual_by_chunk_connection:
        raise BagV2Error("index-data entries do not exactly match chunk messages")

    for chunk_pos, chunk in chunks.items():
        info = chunk_infos.get(chunk_pos)
        if info is None:
            raise BagV2Error("chunk has no matching chunk-info record")
        chunk_messages = chunk["messages"]
        counts: dict[int, int] = {}
        times = []
        for message in chunk_messages:
            counts[message.connection_id] = counts.get(message.connection_id, 0) + 1
            times.append(message.record_time_ns)
        if info["counts"] != counts:
            raise BagV2Error("chunk-info connection counts differ from chunk messages")
        if not times or info["start_time_ns"] != min(times) or info["end_time_ns"] != max(times):
            raise BagV2Error("chunk-info time bounds differ from chunk messages")

    odom_rows: list[tuple[int, int, str, str]] = []
    clock_rows: list[tuple[int, int]] = []
    dynamic_tf: list[tuple[str, str, int]] = []
    static_tf: list[tuple[str, str, int]] = []
    messages_by_topic: dict[str, list[_Message]] = {}
    for message in messages:
        connection = connections[message.connection_id]
        topic = connection["topic"]
        messages_by_topic.setdefault(topic, []).append(message)
        decoded = _decode_registered_payload(
            topic, connection["type"], message.payload, selected_limits
        )
        if topic == "/odom" and decoded is not None:
            odom_rows.append(
                (
                    message.record_time_ns,
                    int(decoded["header_time_ns"]),
                    str(decoded["frame_id"]),
                    str(decoded["child_frame_id"]),
                )
            )
        elif topic == "/clock" and decoded is not None:
            clock_rows.append((message.record_time_ns, int(decoded["clock_time_ns"])))
        elif topic in {"/tf", "/tf_static"} and decoded is not None:
            target = static_tf if topic == "/tf_static" else dynamic_tf
            for transform in decoded["transforms"]:
                target.append(
                    (
                        str(transform["parent"]),
                        str(transform["child"]),
                        message.record_time_ns,
                    )
                )

    connection_summaries = []
    for connection_id, connection in sorted(connections.items()):
        connection_messages = [
            message for message in messages if message.connection_id == connection_id
        ]
        connection_summaries.append(
            {
                **connection,
                "message_count": len(connection_messages),
                "record_time_range": _time_range(
                    message.record_time_ns for message in connection_messages
                ),
            }
        )

    topic_summaries = []
    anomalies: list[str] = []
    for topic in sorted(messages_by_topic):
        topic_messages = messages_by_topic[topic]
        ids = sorted(
            connection_id
            for connection_id, connection in connections.items()
            if connection["topic"] == topic
        )
        first_connection = connections[ids[0]]
        record_range = _time_range(message.record_time_ns for message in topic_messages)
        if record_range is not None and not record_range["monotonic"]:
            anomalies.append(f"NON_MONOTONIC_RECORD_TIME:{topic}")
        header_range = None
        header_monotonic = None
        if topic == "/odom":
            header_range = _time_range(row[1] for row in odom_rows)
            header_monotonic = header_range["monotonic"] if header_range else None
            if header_monotonic is False:
                anomalies.append("NON_MONOTONIC_HEADER_TIME:/odom")
        topic_summaries.append(
            {
                "topic": topic,
                "type": first_connection["type"],
                "md5sum": first_connection["md5sum"],
                "message_definition_sha256": first_connection[
                    "message_definition_sha256"
                ],
                "connection_ids": ids,
                "message_count": len(topic_messages),
                "record_time_range": record_range,
                "header_time_range": header_range,
                "header_time_monotonic": header_monotonic,
            }
        )

    frame_pairs: dict[tuple[str, str], int] = {}
    for _, _, frame_id, child_frame_id in odom_rows:
        frame_pairs[(frame_id, child_frame_id)] = (
            frame_pairs.get((frame_id, child_frame_id), 0) + 1
        )
    odom_record_range = _time_range(row[0] for row in odom_rows)
    odom_header_range = _time_range(row[1] for row in odom_rows)
    clock_record_range = _time_range(row[0] for row in clock_rows)
    clock_value_range = _time_range(row[1] for row in clock_rows)

    return {
        "format": "ROS1_BAG_V2",
        "version": "2.0",
        "indexed": True,
        "file_header": {
            "index_pos": index_pos,
            "connection_count": connection_count,
            "chunk_count": chunk_count,
        },
        "compression": sorted(compression_names),
        "total_message_count": len(messages),
        "record_time_range": _time_range(
            message.record_time_ns for message in messages
        ),
        "connections": connection_summaries,
        "topics": topic_summaries,
        "odom": {
            "message_count": len(odom_rows),
            "record_time_range": odom_record_range,
            "header_time_range": odom_header_range,
            "record_time_monotonic": (
                odom_record_range["monotonic"] if odom_record_range else False
            ),
            "header_time_monotonic": (
                odom_header_range["monotonic"] if odom_header_range else False
            ),
            "header_record_max_abs_delta_ns": (
                max(abs(record - header) for record, header, _, _ in odom_rows)
                if odom_rows
                else None
            ),
            "frame_pairs": [
                {
                    "frame_id": frame,
                    "child_frame_id": child,
                    "message_count": count,
                }
                for (frame, child), count in sorted(frame_pairs.items())
            ],
        },
        "clock": {
            "message_count": len(clock_rows),
            "record_time_range": clock_record_range,
            "clock_time_range": clock_value_range,
            "record_time_monotonic": (
                clock_record_range["monotonic"] if clock_record_range else False
            ),
            "clock_time_monotonic": (
                clock_value_range["monotonic"] if clock_value_range else False
            ),
        },
        "frame_graph": {
            "dynamic_edges": _edge_summary(dynamic_tf),
            "static_edges": _edge_summary(static_tf),
        },
        "index_verified": True,
        "chunk_info_verified": True,
        "topic_conflicts": [],
        "anomalies": sorted(anomalies),
        "limits": asdict(selected_limits),
    }


def inspect_bag(
    path: str | os.PathLike[str],
    *,
    limits: ReaderLimits | None = None,
    expected_sha256: str | None = None,
    expected_size_bytes: int | None = None,
    expected_mode: int | None = None,
) -> dict[str, Any]:
    """Securely read, parse, and return before/after identity plus bag census."""

    selected_limits = limits or ReaderLimits()
    data, before, after = read_regular_file(
        path,
        limits=selected_limits,
        expected_sha256=expected_sha256,
        expected_size_bytes=expected_size_bytes,
        expected_mode=expected_mode,
    )
    parsed = parse_bag_v2(data, limits=selected_limits)
    return {"source_before": before, "source_after": after, "bag": parsed}


__all__ = [
    "BagV2Error",
    "MAGIC",
    "OP_CHUNK",
    "OP_CHUNK_INFO",
    "OP_CONNECTION",
    "OP_FILE_HEADER",
    "OP_INDEX_DATA",
    "OP_MSG_DATA",
    "ReaderLimits",
    "SUPPORTED_COMPRESSION",
    "inspect_bag",
    "parse_bag_v2",
    "read_regular_file",
]
