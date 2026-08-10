#!/usr/bin/env python3
"""Strict, read-only reader for the JPartDataBi4 subset used by U3.

The implementation follows ``JBinaryData.cpp`` / ``JPartDataBi4.cpp`` from
the source materialization recorded for the liquid host.  It intentionally
supports only bounded, little-endian JBD files and never invokes a
DualSPHysics executable.
"""

from __future__ import annotations

import hashlib
import os
import stat
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_STRING_BYTES = 1024 * 1024
MAX_COLLECTION_COUNT = 1_000_000
MAX_ITEM_DEPTH = 16

JBD_HEADER_SIZE = 64
JBD_FILE_CODE = "JPartDataBi4"
JBD_ITEM_CODE = "\nITEM\n"
JBD_VALUES_CODE = "\nVALUES"
JBD_ARRAY_CODE = "\nARRAY"


class Bi4FormatError(ValueError):
    """Raised when a BI4 input violates the supported format or invariants."""


TYPE_NAMES = {
    0: "null",
    1: "text",
    2: "bool",
    3: "char",
    4: "uchar",
    5: "short",
    6: "ushort",
    7: "int",
    8: "uint",
    9: "llong",
    10: "ullong",
    11: "float",
    12: "double",
    20: "int3",
    21: "uint3",
    22: "float3",
    23: "double3",
}

TYPE_STRUCTS = {
    3: "b",
    4: "B",
    5: "h",
    6: "H",
    7: "i",
    8: "I",
    9: "q",
    10: "Q",
    11: "f",
    12: "d",
    20: "iii",
    21: "III",
    22: "fff",
    23: "ddd",
}


@dataclass(frozen=True)
class JbdArray:
    """One decoded JBinaryData array definition plus its exact payload."""

    name: str
    type_code: int
    count: int
    hidden: bool
    payload: bytes

    @property
    def type_name(self) -> str:
        return TYPE_NAMES.get(self.type_code, f"unknown-{self.type_code}")

    def records(self) -> list[Any]:
        """Decode numeric records using the source-defined native layout."""

        if self.type_code == 1:
            reader = _Reader(self.payload)
            values = [reader.read_string() for _ in range(self.count)]
            reader.require_eof("text array payload")
            return values
        if self.type_code == 2:
            if len(self.payload) != self.count:
                raise Bi4FormatError("boolean array payload has invalid size")
            return [bool(value) for value in self.payload]
        fmt = TYPE_STRUCTS.get(self.type_code)
        if fmt is None:
            raise Bi4FormatError(f"unsupported array type {self.type_code}")
        record = struct.Struct("<" + fmt)
        if len(self.payload) != record.size * self.count:
            raise Bi4FormatError(f"array {self.name!r} payload has invalid size")
        decoded = list(record.iter_unpack(self.payload))
        if len(fmt) == 1:
            return [value[0] for value in decoded]
        return decoded


@dataclass(frozen=True)
class JbdItem:
    """One recursively decoded JBinaryData item."""

    name: str
    hidden: bool
    values_hidden: bool
    fmt_float: str
    fmt_double: str
    values: dict[str, Any]
    value_types: dict[str, int]
    arrays: dict[str, JbdArray]
    items: tuple["JbdItem", ...]

    def child(self, name: str) -> "JbdItem":
        hits = [item for item in self.items if item.name == name]
        if len(hits) != 1:
            raise Bi4FormatError(
                f"expected exactly one child {name!r}, found {len(hits)}"
            )
        return hits[0]


@dataclass(frozen=True)
class SecureFile:
    path: Path
    data: bytes
    sha256: str
    size_bytes: int
    mode: int
    uid: int
    gid: int


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.offset = 0

    @property
    def remaining(self) -> int:
        return len(self.data) - self.offset

    def _take(self, count: int, label: str) -> bytes:
        if count < 0 or count > self.remaining:
            raise Bi4FormatError(
                f"truncated {label}: need {count} bytes, have {self.remaining}"
            )
        start = self.offset
        self.offset += count
        return self.data[start : self.offset]

    def read_struct(self, fmt: str, label: str) -> Any:
        parser = struct.Struct("<" + fmt)
        values = parser.unpack(self._take(parser.size, label))
        return values[0] if len(values) == 1 else values

    def read_u32(self, label: str) -> int:
        return int(self.read_struct("I", label))

    def read_string(self) -> str:
        length = self.read_u32("string length")
        if length > MAX_STRING_BYTES:
            raise Bi4FormatError(f"string length {length} exceeds safety limit")
        raw = self._take(length, "string")
        try:
            return raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise Bi4FormatError("JBD string is not valid UTF-8") from exc

    def require_offset(self, expected: int, label: str) -> None:
        if self.offset != expected:
            raise Bi4FormatError(
                f"{label} size mismatch: ended at {self.offset}, expected {expected}"
            )

    def require_eof(self, label: str) -> None:
        if self.remaining:
            raise Bi4FormatError(f"{label} has {self.remaining} trailing bytes")


def _read_canonical_bool_i32(reader: _Reader, label: str) -> bool:
    raw = int(reader.read_struct("i", label))
    if raw not in (0, 1):
        raise Bi4FormatError(f"{label} is not canonically encoded as 0 or 1")
    return bool(raw)


def _normalise_sha256(expected: str | None) -> str | None:
    if expected is None:
        return None
    value = expected.lower()
    if len(value) != 64 or any(ch not in "0123456789abcdef" for ch in value):
        raise ValueError("expected SHA-256 must be exactly 64 hexadecimal characters")
    return value


def read_regular_file(
    path: str | os.PathLike[str],
    *,
    expected_sha256: str | None = None,
    max_bytes: int = MAX_FILE_BYTES,
) -> SecureFile:
    """Read one bounded regular file without following a final symlink."""

    expected = _normalise_sha256(expected_sha256)
    path_obj = Path(path)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path_obj, flags)
    except OSError as exc:
        raise Bi4FormatError(f"cannot securely open {path_obj}: {exc}") from exc
    try:
        metadata = os.fstat(fd)
        if not stat.S_ISREG(metadata.st_mode):
            raise Bi4FormatError(f"input is not a regular file: {path_obj}")
        if metadata.st_size <= 0 or metadata.st_size > max_bytes:
            raise Bi4FormatError(
                f"input size {metadata.st_size} is outside 1..{max_bytes}: {path_obj}"
            )
        chunks: list[bytes] = []
        remaining = metadata.st_size
        while remaining:
            chunk = os.read(fd, min(remaining, 1024 * 1024))
            if not chunk:
                raise Bi4FormatError(f"short read from {path_obj}")
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
    finally:
        os.close(fd)

    digest = hashlib.sha256(data).hexdigest()
    if expected is not None and digest != expected:
        raise Bi4FormatError(
            f"SHA-256 mismatch for {path_obj}: expected {expected}, got {digest}"
        )
    return SecureFile(
        path=path_obj,
        data=data,
        sha256=digest,
        size_bytes=len(data),
        mode=stat.S_IMODE(metadata.st_mode),
        uid=metadata.st_uid,
        gid=metadata.st_gid,
    )


def _read_value(reader: _Reader) -> tuple[str, int, Any]:
    name = reader.read_string()
    type_code = int(reader.read_struct("i", "value type"))
    if type_code == 1:
        value: Any = reader.read_string()
    elif type_code == 2:
        value = _read_canonical_bool_i32(reader, "boolean value")
    else:
        fmt = TYPE_STRUCTS.get(type_code)
        if fmt is None:
            raise Bi4FormatError(f"unsupported JBD value type {type_code}")
        value = reader.read_struct(fmt, f"{TYPE_NAMES[type_code]} value")
    return name, type_code, value


def _read_array(reader: _Reader, *, si64: bool) -> JbdArray:
    definition_size = reader.read_u32("array definition size")
    if definition_size > MAX_STRING_BYTES or definition_size > reader.remaining:
        raise Bi4FormatError("array definition size is invalid")
    definition_end = reader.offset + definition_size
    if reader.read_string() != JBD_ARRAY_CODE:
        raise Bi4FormatError("invalid ARRAY validation code")
    name = reader.read_string()
    hidden = _read_canonical_bool_i32(reader, "array hidden flag")
    type_code = int(reader.read_struct("i", "array type"))
    if type_code not in TYPE_NAMES or type_code == 0:
        raise Bi4FormatError(f"unsupported array type {type_code}")
    size_fmt = "Q" if si64 else "I"
    count = int(reader.read_struct(size_fmt, "array count"))
    payload_size = int(reader.read_struct(size_fmt, "array payload size"))
    reader.require_offset(definition_end, f"array {name!r} definition")
    if count > MAX_COLLECTION_COUNT:
        raise Bi4FormatError(f"array {name!r} count {count} exceeds safety limit")
    if payload_size > reader.remaining:
        raise Bi4FormatError(f"array {name!r} payload exceeds remaining file")
    if type_code not in (1, 2):
        fmt = TYPE_STRUCTS.get(type_code)
        if fmt is None:
            raise Bi4FormatError(f"unsupported array type {type_code}")
        expected_size = struct.calcsize("<" + fmt) * count
        if payload_size != expected_size:
            raise Bi4FormatError(
                f"array {name!r} payload size {payload_size} != {expected_size}"
            )
    payload = reader._take(payload_size, f"array {name!r} payload")
    if type_code == 1:
        text_reader = _Reader(payload)
        for _ in range(count):
            text_reader.read_string()
        text_reader.require_eof(f"array {name!r} text payload")
    return JbdArray(name, type_code, count, hidden, payload)


def _read_item(reader: _Reader, *, si64: bool, depth: int) -> JbdItem:
    if depth > MAX_ITEM_DEPTH:
        raise Bi4FormatError("JBD item nesting exceeds safety limit")
    definition_size = reader.read_u32("item definition size")
    if definition_size > MAX_STRING_BYTES or definition_size > reader.remaining:
        raise Bi4FormatError("item definition size is invalid")
    definition_end = reader.offset + definition_size
    if reader.read_string() != JBD_ITEM_CODE:
        raise Bi4FormatError("invalid ITEM validation code")
    name = reader.read_string()
    hidden = _read_canonical_bool_i32(reader, "item hidden flag")
    values_hidden = _read_canonical_bool_i32(reader, "values hidden flag")
    fmt_float = reader.read_string()
    fmt_double = reader.read_string()
    array_count = reader.read_u32("item array count")
    item_count = reader.read_u32("child item count")
    values_size = reader.read_u32("values block size")
    reader.require_offset(definition_end, f"item {name!r} definition")
    if array_count > MAX_COLLECTION_COUNT or item_count > MAX_COLLECTION_COUNT:
        raise Bi4FormatError("item collection count exceeds safety limit")
    if values_size > reader.remaining:
        raise Bi4FormatError(f"item {name!r} values block exceeds remaining file")

    values: dict[str, Any] = {}
    value_types: dict[str, int] = {}
    if values_size:
        values_end = reader.offset + values_size
        if reader.read_string() != JBD_VALUES_CODE:
            raise Bi4FormatError("invalid VALUES validation code")
        value_count = reader.read_u32("value count")
        if value_count > MAX_COLLECTION_COUNT:
            raise Bi4FormatError("value count exceeds safety limit")
        for _ in range(value_count):
            value_name, value_type, value = _read_value(reader)
            if value_name in values:
                raise Bi4FormatError(f"duplicate value {value_name!r} in item {name!r}")
            values[value_name] = value
            value_types[value_name] = value_type
        reader.require_offset(values_end, f"item {name!r} values block")

    arrays: dict[str, JbdArray] = {}
    for _ in range(array_count):
        array = _read_array(reader, si64=si64)
        if array.name in arrays:
            raise Bi4FormatError(f"duplicate array {array.name!r} in item {name!r}")
        arrays[array.name] = array

    items = tuple(_read_item(reader, si64=si64, depth=depth + 1) for _ in range(item_count))
    return JbdItem(
        name=name,
        hidden=hidden,
        values_hidden=values_hidden,
        fmt_float=fmt_float,
        fmt_double=fmt_double,
        values=values,
        value_types=value_types,
        arrays=arrays,
        items=items,
    )


def parse_jpartdata_bi4(data: bytes) -> JbdItem:
    """Parse a complete JPartDataBi4 byte string and reject trailing data."""

    if len(data) < JBD_HEADER_SIZE + 4:
        raise Bi4FormatError("BI4 file is too small")
    header = data[:JBD_HEADER_SIZE]
    try:
        title = header[:58].decode("ascii").rstrip(" ")
    except UnicodeDecodeError as exc:
        raise Bi4FormatError("BI4 header title is not ASCII") from exc
    if title != f"#FileJBD {JBD_FILE_CODE}":
        raise Bi4FormatError(f"unexpected BI4 header title {title!r}")
    if header[58] != 0x0A or header[59] != 0:
        raise Bi4FormatError("BI4 header terminator is invalid")
    byte_order, si64, reserved2, reserved3 = header[60:64]
    if byte_order not in (0, 10):
        raise Bi4FormatError("only little-endian BI4 files are supported")
    if si64 not in (0, 1) or bool(byte_order == 10) != bool(si64):
        raise Bi4FormatError("BI4 64-bit size flags are inconsistent")
    if reserved2 or reserved3:
        raise Bi4FormatError("BI4 reserved header bytes are non-zero")

    reader = _Reader(data[JBD_HEADER_SIZE:])
    root = _read_item(reader, si64=bool(si64), depth=0)
    reader.require_eof("BI4 file")
    if root.name != JBD_FILE_CODE:
        raise Bi4FormatError(f"unexpected BI4 root item {root.name!r}")
    return root


def require_int(values: dict[str, Any], name: str) -> int:
    value = values.get(name)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise Bi4FormatError(f"required non-negative integer value {name!r} is invalid")
    return value


def extract_u3_particles(root: JbdItem) -> dict[str, Any]:
    """Extract and validate the four arrays required for a U3 static case."""

    if len(root.items) != 1:
        raise Bi4FormatError(f"expected one particle item, found {len(root.items)}")
    part = root.items[0]
    required_types = {"Idp": 8, "Posd": 23, "Vel": 22, "Rhop": 11}
    for name, expected_type in required_types.items():
        array = part.arrays.get(name)
        if array is None:
            raise Bi4FormatError(f"required particle array {name!r} is missing")
        if array.type_code != expected_type:
            raise Bi4FormatError(
                f"array {name!r} type is {array.type_code}, expected {expected_type}"
            )

    ids = part.arrays["Idp"].records()
    positions = part.arrays["Posd"].records()
    velocities = part.arrays["Vel"].records()
    densities = part.arrays["Rhop"].records()
    counts = {len(ids), len(positions), len(velocities), len(densities)}
    if len(counts) != 1:
        raise Bi4FormatError("particle arrays have inconsistent counts")
    particle_count = counts.pop()
    if particle_count != require_int(part.values, "Npok"):
        raise Bi4FormatError("particle array count does not match PART Npok")
    if particle_count != require_int(root.values, "CaseNp"):
        raise Bi4FormatError("particle array count does not match CaseNp")
    if ids != list(range(particle_count)):
        raise Bi4FormatError("U3 particle IDs are not exactly contiguous 0..N-1")

    fixed = require_int(root.values, "CaseNfixed")
    moving = require_int(root.values, "CaseNmoving")
    floating = require_int(root.values, "CaseNfloat")
    fluid = require_int(root.values, "CaseNfluid")
    if fixed + moving + floating + fluid != particle_count:
        raise Bi4FormatError("particle class counts do not sum to CaseNp")

    limits = {
        "x": [min(p[0] for p in positions), max(p[0] for p in positions)],
        "y": [min(p[1] for p in positions), max(p[1] for p in positions)],
        "z": [min(p[2] for p in positions), max(p[2] for p in positions)],
    }
    class_limits = {
        "fixed": fixed,
        "moving": fixed + moving,
        "floating": fixed + moving + floating,
    }
    classes: list[str] = []
    for particle_id in ids:
        if particle_id < class_limits["fixed"]:
            classes.append("fixed_boundary")
        elif particle_id < class_limits["moving"]:
            classes.append("moving_boundary")
        elif particle_id < class_limits["floating"]:
            classes.append("floating")
        else:
            classes.append("fluid")

    return {
        "part_name": part.name,
        "particle_count": particle_count,
        "counts": {
            "fixed_boundary": fixed,
            "moving_boundary": moving,
            "floating": floating,
            "fluid": fluid,
        },
        "limits_m": limits,
        "ids": ids,
        "classes": classes,
        "positions_m": positions,
        "velocities_m_s": velocities,
        "densities_kg_m3": densities,
        "array_metadata": {
            name: {
                "type": part.arrays[name].type_name,
                "count": part.arrays[name].count,
                "payload_bytes": len(part.arrays[name].payload),
            }
            for name in required_types
        },
    }


__all__ = [
    "Bi4FormatError",
    "JbdArray",
    "JbdItem",
    "SecureFile",
    "extract_u3_particles",
    "parse_jpartdata_bi4",
    "read_regular_file",
    "require_int",
]
