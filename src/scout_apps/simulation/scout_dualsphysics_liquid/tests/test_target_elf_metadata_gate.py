#!/usr/bin/env python3
"""Mock-only tests for the MSI pure-Python offline ELF metadata gate."""

from __future__ import annotations

import copy
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_elf_metadata_gate as elf_gate  # noqa: E402


class _FakePopen:
    """In-memory completed process; no Git or ELF tool is ever started."""

    def __init__(self, stdout: bytes, stderr: bytes = b"", returncode: int = 0) -> None:
        self.stdout = io.BytesIO(stdout)
        self.stderr = io.BytesIO(stderr)
        self.returncode = returncode
        self.pid = 424242

    def poll(self) -> int:
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:  # noqa: ARG002
        return self.returncode


def _elf_fixture() -> bytes:
    data = bytearray(0x500)
    ident = b"\x7fELF\x02\x01\x01\x03" + b"\0" * 8
    struct.pack_into(
        "<16sHHIQQQIHHHHHH",
        data,
        0,
        ident,
        3,
        62,
        1,
        0,
        64,
        0,
        0,
        64,
        56,
        3,
        0,
        0,
        0,
    )
    base = 0x400000
    interpreter = b"/lib64/ld-linux-x86-64.so.2\0"
    strings = b"\0libc.so.6\0fixture.so\0"
    dynamic = [
        (elf_gate.DT_NEEDED, 1),
        (elf_gate.DT_SONAME, 11),
        (elf_gate.DT_STRTAB, base + 0x300),
        (elf_gate.DT_STRSZ, len(strings)),
        (elf_gate.DT_NULL, 0),
    ]
    struct.pack_into("<IIQQQQQQ", data, 64, elf_gate.PT_LOAD, 5, 0, base, base, len(data), len(data), 0x1000)
    struct.pack_into(
        "<IIQQQQQQ", data, 120, elf_gate.PT_DYNAMIC, 4, 0x200, base + 0x200, base + 0x200, len(dynamic) * 16, len(dynamic) * 16, 8
    )
    struct.pack_into(
        "<IIQQQQQQ", data, 176, elf_gate.PT_INTERP, 4, 0x100, base + 0x100, base + 0x100, len(interpreter), len(interpreter), 1
    )
    data[0x100 : 0x100 + len(interpreter)] = interpreter
    for index, (tag, value) in enumerate(dynamic):
        struct.pack_into("<qQ", data, 0x200 + index * 16, tag, value)
    data[0x300 : 0x300 + len(strings)] = strings
    return bytes(data)


class TargetElfMetadataPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(elf_gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_checked_in_policy_is_offline_and_exact(self) -> None:
        self.assertEqual(elf_gate.validate_policy(self.policy), [])
        self.assertFalse(self.policy["upstream_code_execution_authorized"])
        self.assertTrue(self.policy["invariants"]["no_external_elf_parser"])
        self.assertEqual(len(self.policy["elf_objects"]), 4)

    def test_policy_rejects_external_parser_or_different_elf_object(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["invariants"]["no_external_elf_parser"] = False
        changed["elf_objects"][0]["sha256"] = "0" * 64
        errors = elf_gate.validate_policy(changed)
        self.assertIn("ELF object set differs from the frozen static-inventory hashes", errors)
        self.assertIn("ELF metadata safety invariants are incomplete", errors)

    def test_pure_python_parser_reads_bounded_dynamic_metadata(self) -> None:
        metadata = elf_gate.parse_elf64_little_endian(_elf_fixture(), self.policy["elf_expectations"])
        self.assertEqual(metadata["class"], "ELF64")
        self.assertEqual(metadata["endianness"], "little")
        self.assertEqual(metadata["machine"], 62)
        self.assertEqual(metadata["interpreter"], "/lib64/ld-linux-x86-64.so.2")
        self.assertEqual(metadata["soname"], "fixture.so")
        self.assertEqual(metadata["needed"], ["libc.so.6"])

        unsafe = bytearray(_elf_fixture())
        struct.pack_into("<Q", unsafe, 32, len(unsafe) + 1)
        with self.assertRaises(elf_gate.ElfMetadataError):
            elf_gate.parse_elf64_little_endian(bytes(unsafe), self.policy["elf_expectations"])

        cross_segment = bytearray(_elf_fixture())
        struct.pack_into("<Q", cross_segment, 64 + 32, 0x310)
        with self.assertRaises(elf_gate.ElfMetadataError):
            elf_gate.parse_elf64_little_endian(bytes(cross_segment), self.policy["elf_expectations"])

    def test_blob_reader_is_mocked_and_never_invokes_git(self) -> None:
        payload = _elf_fixture()
        fake = _FakePopen(payload)
        with mock.patch.object(elf_gate.subprocess, "Popen", return_value=fake) as popen:
            evidence, observed = elf_gate._read_blob_bytes(
                Path("/mock/bare.git"), "a" * 40, len(payload), self.policy
            )
        popen.assert_called_once()
        self.assertEqual(observed, payload)
        self.assertEqual(evidence["bytes_read"], len(payload))
        self.assertEqual(evidence["returncode"], 0)

    def test_write_flow_is_mocked_and_refuses_receipt_reuse(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-elf-metadata-") as temporary:
            root = Path(temporary) / "root"
            audits = root / "audits"
            audits.mkdir(parents=True)
            receipt_path = audits / "u2_elf_metadata_v1_20260806T120000Z.json"
            self.assertEqual(
                elf_gate.validate_receipt_path(receipt_path, root),
                (receipt_path, Path(f"{receipt_path}.partial")),
            )
            preflight = {"status": "PASS_OFFLINE_ELF_METADATA_ADMITTED", "preflight_sha256": "f" * 64}
            profile = {"storage": {"approved_root": str(root)}}

            def fake_read(_: Path, object_id: str, size: int, __: dict[str, object]):
                entry = next(item for item in self.policy["elf_objects"] if item["object_id"] == object_id)
                return ({"sha256": entry["sha256"], "bytes_read": size}, b"fixture")

            parsed = {
                "class": "ELF64",
                "endianness": "little",
                "machine": 62,
                "needed": [],
            }
            with mock.patch.object(elf_gate, "build_preflight", return_value=preflight), mock.patch.object(
                elf_gate.profile_gate, "_read_json", return_value=profile
            ), mock.patch.object(elf_gate, "_approved_root", return_value=root), mock.patch.object(
                elf_gate, "_read_blob_bytes", side_effect=fake_read
            ), mock.patch.object(elf_gate, "parse_elf64_little_endian", return_value=parsed):
                receipt = elf_gate.write_elf_metadata_receipt(self.policy, receipt_path)
            self.assertEqual(receipt["status"], "PASS_OFFLINE_ELF_METADATA")
            self.assertFalse(receipt["network_used"])
            self.assertFalse(receipt["precompiled_binary_executed"])
            self.assertTrue(receipt_path.is_file())
            with self.assertRaises(elf_gate.ElfMetadataError):
                elf_gate.validate_receipt_path(receipt_path, root)


if __name__ == "__main__":
    unittest.main()
