#!/usr/bin/env python3
"""No-executable tests for the strict U3 JPartDataBi4 reader."""

from __future__ import annotations

import hashlib
import struct
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_bi4_reader_v1 as reader  # noqa: E402


def _string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _value(name: str, type_code: int, payload: bytes) -> bytes:
    return _string(name) + struct.pack("<i", type_code) + payload


def _values(records: list[bytes]) -> bytes:
    return _string("\nVALUES") + struct.pack("<I", len(records)) + b"".join(records)


def _array(name: str, type_code: int, count: int, payload: bytes) -> bytes:
    definition = (
        _string("\nARRAY")
        + _string(name)
        + struct.pack("<iiII", 0, type_code, count, len(payload))
    )
    return struct.pack("<I", len(definition)) + definition + payload


def _item(
    name: str,
    values: list[bytes],
    arrays: list[bytes] | None = None,
    children: list[bytes] | None = None,
) -> bytes:
    arrays = arrays or []
    children = children or []
    value_block = _values(values)
    definition = (
        _string("\nITEM\n")
        + _string(name)
        + struct.pack("<ii", 0, 0)
        + _string("%.7E")
        + _string("%.15E")
        + struct.pack("<III", len(arrays), len(children), len(value_block))
    )
    return (
        struct.pack("<I", len(definition))
        + definition
        + value_block
        + b"".join(arrays)
        + b"".join(children)
    )


def make_bi4() -> bytes:
    ids = [0, 1, 2, 3]
    positions = [
        (-0.001, 0.0, 0.0),
        (0.001, 0.0, 0.002),
        (-0.001, 0.0, 0.004),
        (0.001, 0.0, 0.006),
    ]
    velocities = [(0.0, 0.0, 0.0)] * 4
    densities = [1000.0, 1000.0, 1000.5, 1000.25]
    arrays = [
        _array("Idp", 8, 4, b"".join(struct.pack("<I", value) for value in ids)),
        _array("Posd", 23, 4, b"".join(struct.pack("<ddd", *value) for value in positions)),
        _array("Vel", 22, 4, b"".join(struct.pack("<fff", *value) for value in velocities)),
        _array("Rhop", 11, 4, b"".join(struct.pack("<f", value) for value in densities)),
    ]
    part = _item(
        "PART_0000",
        [
            _value("Cpart", 8, struct.pack("<I", 0)),
            _value("Npok", 8, struct.pack("<I", 4)),
        ],
        arrays=arrays,
    )
    root = _item(
        "JPartDataBi4",
        [
            _value("CaseNp", 10, struct.pack("<Q", 4)),
            _value("CaseNfixed", 10, struct.pack("<Q", 2)),
            _value("CaseNmoving", 10, struct.pack("<Q", 0)),
            _value("CaseNfloat", 10, struct.pack("<Q", 0)),
            _value("CaseNfluid", 10, struct.pack("<Q", 2)),
            _value("Dp", 12, struct.pack("<d", 0.002)),
            _value("Rhop0", 12, struct.pack("<d", 1000.0)),
        ],
        children=[part],
    )
    title = b"#FileJBD JPartDataBi4".ljust(58, b" ")
    return title + b"\n\0\0\0\0\0" + root


class Bi4ReaderV1Tests(unittest.TestCase):
    def test_valid_file_extracts_contiguous_particle_classes(self) -> None:
        root = reader.parse_jpartdata_bi4(make_bi4())
        particles = reader.extract_u3_particles(root)
        self.assertEqual(particles["particle_count"], 4)
        self.assertEqual(
            particles["classes"],
            ["fixed_boundary", "fixed_boundary", "fluid", "fluid"],
        )
        self.assertEqual(particles["limits_m"]["z"], [0.0, 0.006])
        self.assertEqual(particles["array_metadata"]["Posd"]["type"], "double3")
        self.assertEqual(root.value_types["CaseNp"], 10)
        self.assertEqual(root.value_types["Dp"], 12)
        self.assertEqual(root.items[0].value_types["Cpart"], 8)

    def test_trailing_bytes_are_rejected(self) -> None:
        with self.assertRaisesRegex(reader.Bi4FormatError, "trailing bytes"):
            reader.parse_jpartdata_bi4(make_bi4() + b"unexpected")

    def test_non_contiguous_ids_are_rejected(self) -> None:
        data = bytearray(make_bi4())
        marker = data.index(b"\nARRAY")
        # Locate the exact Idp payload through the parsed array header shape.
        idp_payload = data.index(struct.pack("<IIII", 0, 1, 2, 3), marker)
        struct.pack_into("<I", data, idp_payload + 8, 9)
        root = reader.parse_jpartdata_bi4(bytes(data))
        with self.assertRaisesRegex(reader.Bi4FormatError, "not exactly contiguous"):
            reader.extract_u3_particles(root)

    def test_secure_reader_pins_hash_and_rejects_symlink(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-bi4-reader-") as temporary:
            base = Path(temporary)
            source = base / "case.bi4"
            source.write_bytes(make_bi4())
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            secure = reader.read_regular_file(source, expected_sha256=digest)
            self.assertEqual(secure.sha256, digest)
            with self.assertRaisesRegex(reader.Bi4FormatError, "SHA-256 mismatch"):
                reader.read_regular_file(source, expected_sha256="0" * 64)
            link = base / "link.bi4"
            link.symlink_to(source)
            with self.assertRaisesRegex(reader.Bi4FormatError, "cannot securely open"):
                reader.read_regular_file(link)


if __name__ == "__main__":
    unittest.main()
