#!/usr/bin/env python3
"""Static/mock-only regression tests for bounded static-audit revision v2."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path
from unittest import mock


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
SCRIPT = PACKAGE / "scripts/r8_liquid_gpu_static_audit_gate_v2.py"
V1_POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v1.json"
V1_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile"
V2_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2.profile"


def load_gate():
    spec = importlib.util.spec_from_file_location("r8_gpu_static_audit_gate_v2_tested", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


gate = load_gate()


def cuda_outputs(helper: dict[str, object]) -> dict[str, str]:
    return {
        "cuobjdump_list_elf": "".join(
            f"ELF file {index:4d}: DualSPHysics5.{index}.sm_120.cubin\n" for index in range(1, 12)
        ),
        "cuobjdump_list_ptx": "".join(
            f"PTX file {index:4d}: DualSPHysics5.{index}.sm_120.ptx\n" for index in range(1, 12)
        ),
        "cuobjdump_dump_elf": json.dumps(helper, sort_keys=True, separators=(",", ":")) + "\n",
        "cuobjdump_dump_elf_symbols": "STT_FUNC STB_GLOBAL STO_ENTRY kernel\n",
        "cuobjdump_dump_ptx": "arch = sm_120\n.target sm_120\n.visible .entry kernel(\n",
    }


def helper_fixture() -> dict[str, object]:
    records = []
    for index in range(1, 12):
        records.append(
            {
                "name": f"DualSPHysics5.{index}.sm_120.cubin",
                "size_bytes": 1024 + index,
                "sha256": f"{index:064x}",
                "elf_type": "EXEC (Executable file)",
                "machine": "NVIDIA CUDA architecture",
                "nonzero_executable_text": 1,
                "readelf_stdout_sha256": "a" * 64,
                "readelf_stderr_sha256": "b" * 64,
                "readelf_stderr_bytes": 0,
            }
        )
    return {
        "status": "PASS_EXTRACTED_CUBIN_STATIC_METADATA",
        "cubin_count": 11,
        "architecture": "sm_120",
        "nonzero_executable_text_section_count": 11,
        "extract_stdout_sha256": "c" * 64,
        "extract_stderr_sha256": "d" * 64,
        "records": records,
        "guest_tmpfs_only": True,
    }


class GpuStaticAuditGateV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = gate.policy()
        cls.v1_policy = json.loads(V1_POLICY.read_text(encoding="utf-8"))

    def test_revision_is_failure_bound_and_self_check_stays_read_only(self):
        revision = gate.revision_check()
        self.assertEqual(revision["status"], "PASS_BOUNDED_CUBIN_AUDIT_REVISION_V2")
        self.assertEqual(revision["semantic_delta_count"], 1)
        self.assertFalse(revision["candidate_rebuilt"])
        with mock.patch.object(gate.BASE.subprocess, "Popen") as popen:
            result = gate.BASE.self_check()
        popen.assert_not_called()
        self.assertEqual(result["planned_command_count"], 142)

    def test_profile_delta_is_only_name_and_two_python_execute_rules(self):
        old_lines = {
            line.strip().replace("r8-liquid-u3-gpu-static-audit-20260810T170339Z", "PROFILE")
            for line in V1_PROFILE.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        new_lines = {
            line.strip().replace("r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2", "PROFILE")
            for line in V2_PROFILE.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        self.assertEqual(new_lines - old_lines, {"/usr/bin/python3 rix,", "/usr/bin/python3.12 rix,"})
        self.assertEqual(old_lines - new_lines, set())

    def test_policy_delta_preserves_candidate_objects_limits_and_acceptance(self):
        for key in ("candidate", "objects", "sandbox", "acceptance", "safety", "parent_inputs", "object_command"):
            self.assertEqual(self.policy[key], self.v1_policy[key])
        self.assertEqual(self.policy["trusted_tools"][:-2], self.v1_policy["trusted_tools"])
        old_commands = {item["id"]: item for item in self.v1_policy["candidate_commands"]}
        new_commands = {item["id"]: item for item in self.policy["candidate_commands"]}
        self.assertEqual(set(old_commands), set(new_commands))
        for command_id in old_commands:
            if command_id != "cuobjdump_dump_elf":
                self.assertEqual(new_commands[command_id], old_commands[command_id])
        self.assertEqual(new_commands["cuobjdump_dump_elf"]["argv"][:4], ["/usr/bin/python3", "-I", "-S", "-c"])

    def test_helper_is_valid_python_and_has_no_shell_network_proc_or_device(self):
        command = next(item for item in self.policy["candidate_commands"] if item["id"] == "cuobjdump_dump_elf")
        helper = command["argv"][4]
        compile(helper, "<bounded-cubin-helper>", "exec")
        for forbidden in ("shell=True", "socket", "urllib", "requests", "/proc/", "/dev/", "os.system"):
            self.assertNotIn(forbidden, helper)
        self.assertIn("--extract-elf", helper)
        self.assertIn("guest_tmpfs_only", helper)

    def test_bounded_helper_parser_accepts_exact_11_sm120_records(self):
        helper = helper_fixture()
        outputs = cuda_outputs(helper)
        results = {key: {"id": key} for key in outputs}
        with mock.patch.object(gate.BASE, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
            parsed = gate.parse_cuda(results, self.policy)
        self.assertEqual(parsed["native_cubin_count"], 11)
        self.assertEqual(parsed["ptx_count"], 11)
        self.assertEqual(parsed["nonzero_executable_text_section_count"], 11)

    def test_bounded_helper_parser_rejects_guest_or_architecture_drift(self):
        for mutation in ("guest", "arch"):
            helper = helper_fixture()
            if mutation == "guest":
                helper["guest_tmpfs_only"] = False
            else:
                helper["architecture"] = "sm_90"
            outputs = cuda_outputs(helper)
            results = {key: {"id": key} for key in outputs}
            with mock.patch.object(gate.BASE, "output_text", side_effect=lambda result, label="stdout": outputs[result["id"]]):
                with self.assertRaises(gate.RevisionError):
                    gate.parse_cuda(results, self.policy)

    def test_revision_policy_rejects_helper_or_output_limit_drift(self):
        changed = copy.deepcopy(self.policy)
        command = next(item for item in changed["candidate_commands"] if item["id"] == "cuobjdump_dump_elf")
        command["argv"][4] += "\nprint('drift')\n"
        with self.assertRaises(gate.RevisionError):
            gate.validate_policy(changed)
        changed = copy.deepcopy(self.policy)
        changed["sandbox"]["output_limit_bytes"] += 1
        with self.assertRaises((gate.RevisionError, gate.BASE.AuditError)):
            gate.validate_policy(changed)

    def test_v1_failure_and_oversize_output_are_preserved(self):
        evidence = self.policy["revision_evidence"]
        self.assertEqual(evidence["oversize_output"]["size_bytes"], 268435456)
        self.assertEqual(gate.sha256_regular(Path(evidence["failed_lifecycle"]["path"])), evidence["failed_lifecycle"]["sha256"])
        self.assertEqual(gate.sha256_regular(Path(evidence["oversize_output"]["path"])), evidence["oversize_output"]["sha256"])


if __name__ == "__main__":
    unittest.main()
