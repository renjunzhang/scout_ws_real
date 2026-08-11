#!/usr/bin/env python3
"""Static/mock-only tests for the successful-campaign GPU audit gate."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
SCRIPT = PACKAGE / "scripts/r8_liquid_gpu_static_audit_gate_v1.py"
GENERATOR = PACKAGE / "scripts/r8_liquid_gpu_static_audit_profile_generator_v1.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


gate = load_module(SCRIPT, "r8_gpu_static_audit_gate_v1_tested")
generator = load_module(GENERATOR, "r8_gpu_static_audit_profile_generator_v1_tested")


class GpuStaticAuditGateV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = gate.policy()
        _, cls.object_entries = gate.build_evidence(cls.policy)

    def test_checked_in_contract_self_check_is_read_only(self):
        with mock.patch.object(gate.subprocess, "Popen") as popen:
            result = gate.self_check()
        popen.assert_not_called()
        self.assertEqual(result["status"], "PASS_GPU_STATIC_AUDIT_CONTRACT_SELF_CHECK")
        self.assertEqual(result["planned_command_count"], 142)
        self.assertEqual(result["object_inventory"]["count"], 131)
        self.assertFalse(result["candidate_executed"])

    def test_profile_derivation_removes_b_and_retains_only_successful_root(self):
        rendered = generator.render().decode("utf-8")
        self.assertIn(generator.PROFILE_NAME, rendered)
        self.assertIn(generator.NEW_A, rendered)
        self.assertNotIn(generator.OLD_A, rendered)
        self.assertNotIn(generator.OLD_B, rendered)
        self.assertNotIn("/dev/nvidia", rendered)
        self.assertNotIn("/usr/local/cuda-12.8/bin/nvcc ", rendered)

    def test_sandbox_argv_has_three_ro_binds_and_no_host_write_or_devices(self):
        command = self.policy["candidate_commands"][0]
        candidate = Path(self.policy["candidate"]["path"])
        argv = gate.build_sandbox_argv(
            self.policy,
            candidate,
            "/audit/input/DualSPHysics5.4_linux64",
            command["resource"],
            command["argv"],
        )
        self.assertEqual(argv.count("--ro-bind"), 3)
        for forbidden in ("--bind", "--dev-bind", "--proc", "--dev"):
            self.assertNotIn(forbidden, argv)
        for mandatory in ("--unshare-net", "--clearenv", "--disable-userns", "--assert-userns-disabled"):
            self.assertIn(mandatory, argv)
        changed = argv.copy()
        changed[changed.index("--ro-bind")] = "--bind"
        with self.assertRaises(gate.AuditError):
            gate.validate_sandbox_argv(
                changed,
                candidate,
                "/audit/input/DualSPHysics5.4_linux64",
                command["argv"],
            )

    def test_policy_rejects_safety_or_needed_allowlist_drift(self):
        changed = copy.deepcopy(self.policy)
        changed["sandbox"]["invariants"]["host_writable_bind_count"] = 1
        with self.assertRaises(gate.AuditError):
            gate.validate_policy(changed)
        changed = copy.deepcopy(self.policy)
        changed["candidate_commands"].append(
            {"id": "execute", "resource": "metadata", "argv": [self.policy["candidate"]["path"]]}
        )
        with self.assertRaises(gate.AuditError):
            gate.validate_policy(changed)

    def test_elf_parser_accepts_only_the_evidence_driven_loader_revision(self):
        outputs = {
            "file": "candidate: ELF 64-bit LSB pie executable, x86-64, dynamically linked\n",
            "readelf_header": (
                "  Class:                             ELF64\n"
                "  Data:                              2's complement, little endian\n"
                "  Type:                              DYN (Position-Independent Executable file)\n"
                "  Machine:                           Advanced Micro Devices X86-64\n"
                "  Entry point address:               0x7e5a0\n"
            ),
            "readelf_program_headers": (
                "      [Requesting program interpreter: /lib64/ld-linux-x86-64.so.2]\n"
                "  LOAD 0x015000 0x0000000000015000 0x0000000000015000 0x223c2b1 0x223c2b1 R E 0x1000\n"
                "  GNU_STACK 0x000000 0x0000000000000000 0x0000000000000000 0x000000 0x000000 RW 0x10\n"
            ),
            "readelf_dynamic": (
                " 0x1 (NEEDED) Shared library: [libgomp.so.1]\n"
                " 0x1 (NEEDED) Shared library: [libstdc++.so.6]\n"
                " 0x1 (NEEDED) Shared library: [libm.so.6]\n"
                " 0x1 (NEEDED) Shared library: [libgcc_s.so.1]\n"
                " 0x1 (NEEDED) Shared library: [libc.so.6]\n"
                " 0x1 (NEEDED) Shared library: [ld-linux-x86-64.so.2]\n"
                " 0x6ffffffb (FLAGS_1) Flags: NOW PIE\n"
            ),
        }
        with mock.patch.object(gate, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
            parsed = gate.parse_elf({key: {"id": key} for key in outputs}, self.policy)
        self.assertEqual(parsed["needed"][-1], "ld-linux-x86-64.so.2")
        self.assertFalse(parsed["loader_dependency_revision"]["silent_allowlist_change"])

        outputs["readelf_dynamic"] = outputs["readelf_dynamic"].replace(
            " 0x1 (NEEDED) Shared library: [ld-linux-x86-64.so.2]\n", ""
        )
        with mock.patch.object(gate, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
            with self.assertRaises(gate.AuditError):
                gate.parse_elf({key: {"id": key} for key in outputs}, self.policy)

    def test_cuda_parser_requires_11_cubin_11_ptx_and_only_sm120(self):
        list_elf = "".join(
            f"ELF file {index:4d}: DualSPHysics5.{index}.sm_120.cubin\n" for index in range(1, 12)
        )
        list_ptx = "".join(
            f"PTX file {index:4d}: DualSPHysics5.{index}.sm_120.ptx\n" for index in range(1, 12)
        )
        outputs = {
            "cuobjdump_list_elf": list_elf,
            "cuobjdump_list_ptx": list_ptx,
            "cuobjdump_dump_elf": (
                "arch = sm_120\n"
                " 39 11100 380 0 80 PROGBITS 6 3 59 .text.kernel\n"
            ),
            "cuobjdump_dump_elf_symbols": "STT_FUNC STB_GLOBAL STO_ENTRY kernel\n",
            "cuobjdump_dump_ptx": "arch = sm_120\n.target sm_120\n.visible .entry kernel(\n",
        }
        results = {key: {"id": key} for key in outputs}
        with mock.patch.object(gate, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
            parsed = gate.parse_cuda(results, self.policy)
        self.assertEqual(parsed["native_cubin_count"], 11)
        self.assertEqual(parsed["ptx_count"], 11)
        self.assertTrue(parsed["gencode_proof"]["compute_120_proven"])

        outputs["cuobjdump_list_ptx"] = outputs["cuobjdump_list_ptx"].replace("sm_120", "sm_90", 1)
        with mock.patch.object(gate, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
            with self.assertRaises(gate.AuditError):
                gate.parse_cuda(results, self.policy)

    def test_all_131_object_headers_are_required(self):
        results = [
            {"id": item["name"], "input": str(Path(self.policy["objects"]["root"]) / item["name"])}
            for item in self.object_entries
        ]
        header = (
            "  Type:                              REL (Relocatable file)\n"
            "  Machine:                           Advanced Micro Devices X86-64\n"
        )
        with mock.patch.object(gate, "output_text", return_value=header):
            parsed = gate.parse_objects(results, self.object_entries)
        self.assertEqual(parsed["count"], 131)
        with mock.patch.object(gate, "output_text", return_value=header):
            with self.assertRaises(gate.AuditError):
                gate.parse_objects(results[:-1], self.object_entries)

    def test_closed_receipt_schema_rejects_unknown_top_level_field(self):
        schema = json.loads(gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        minimal = {key: None for key in schema["required"]}
        minimal["unexpected"] = True
        messages = [error.message for error in Draft202012Validator(schema).iter_errors(minimal)]
        self.assertTrue(any("Additional properties are not allowed" in message for message in messages))

    def test_output_paths_are_outside_project_and_never_symlinks(self):
        for output in self.policy["outputs"].values():
            path = Path(output)
            self.assertTrue(path.is_absolute())
            self.assertTrue(str(path).startswith("/home/zrj/scout_liquid_lab/audits/"))
            # Before execution these paths are absent.  After a fail-closed
            # attempt, its evidence/lifecycle remain immutable by design.
            if path.exists() or path.is_symlink():
                self.assertFalse(path.is_symlink())


if __name__ == "__main__":
    unittest.main()
