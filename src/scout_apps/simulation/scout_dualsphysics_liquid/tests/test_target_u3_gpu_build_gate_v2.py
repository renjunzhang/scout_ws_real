"""Mock/static-only tests for the G2A-to-G4 Attempt A execution layer."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import stat
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gpu_build_gate_v2.py"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gpu_build_gate_v2", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TargetU3GpuBuildGateV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = cls.gate.policy()
        cls.schema = cls.gate.receipt_schema()

    def make_fixture(self, root: Path) -> dict[str, dict[str, object]]:
        expected: dict[str, dict[str, object]] = {}
        for index in range(352):
            relative = f"nested_{index % 7}/source_{index:03d}.txt"
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            data = f"sealed fixture {index}\n".encode("utf-8")
            path.write_bytes(data)
            os.chmod(path, 0o640)
            expected[relative] = {
                "sha256": hashlib.sha256(data).hexdigest(),
                "size_bytes": len(data),
            }
        return expected

    def exact_trace(self):
        states = self.gate.g1.SOURCE_COPY_STATES
        profile = self.policy["profiles"]["a_copy"]["name"]
        evidence = [
            {"create_new": True, "path": str(self.gate.SOURCE_COPY_START)},
            {"isolated": True, "copied_entry_count": 352},
            {
                "entry_count": 352,
                "extra_count": 0,
                "symlink_count": 0,
                "hardlink_count": 0,
                "elf_count": 0,
                "executable_count": 0,
            },
            {"profile_name": profile, "unloaded": True, "zero_residue": True},
            {
                "create_flags": ["O_WRONLY", "O_CREAT", "O_EXCL", "O_NOFOLLOW"],
                "mode_octal": "0600",
                "sha256": self.gate.g1.WRAPPER_SHA256,
                "size_bytes": 84,
                "after_unload_sequence": 4,
            },
            {
                "entry_count": 353,
                "sealed_entry_count": 352,
                "wrapper_entry_count": 1,
                "extra_count": 0,
                "symlink_count": 0,
                "hardlink_count": 0,
                "elf_count": 0,
                "executable_count": 0,
                "wrapper_sha256": self.gate.g1.WRAPPER_SHA256,
                "wrapper_mode_octal": "0600",
            },
            {
                "create_new": True,
                "path": str(self.gate.SOURCE_COPY_FINAL),
                "published": True,
                "after_complete_inventory_sequence": 6,
            },
        ]
        trace = []
        previous = 0
        for index, (state, facts) in enumerate(zip(states, evidence, strict=True), start=1):
            item = self.gate.trace_event(state, index, facts, previous)
            previous = item["captured_at_ns"]
            trace.append(item)
        return trace

    def test_policy_schema_parents_and_identity_are_frozen(self):
        review = self.gate.verify_policy_contract()
        parents = self.gate.verify_parent_hashes(review)
        self.assertEqual(len(parents), 9)
        self.assertEqual(review["campaign"]["parallel_jobs"], 1)
        self.assertEqual(review["build"]["wall_timeout_seconds"], 5400)
        self.assertEqual(review["build"]["cpu_limit_seconds"], 5400)
        self.assertEqual(review["build"]["address_space_limit_bytes"], 8589934592)
        self.assertEqual(review["build"]["minimum_available_memory_bytes"], 4294967296)
        self.assertEqual(review["deadlines"]["t0"], self.gate.T0_TEXT)
        self.assertTrue(review["deadlines"]["t0_reset_forbidden"])
        self.assertTrue(review["campaign"]["attempt_b_forbidden"])
        self.assertEqual(
            hashlib.sha256(self.gate.RECEIPT_SCHEMA_PATH.read_bytes()).hexdigest(),
            self.gate.RECEIPT_SCHEMA_SHA256,
        )

    def test_receipt_schema_is_recursive_object_closed_and_rejects_extras(self):
        Draft202012Validator.check_schema(self.schema)
        self.gate.verify_recursive_object_closure(self.schema)
        sample = self.gate.make_receipt(
            document_type="SMPCC_R8_LIQUID_U3_GPU_SOURCE_COPY_START",
            phase="prepare-source-copy",
            status="STATIC_MOCK",
            safety_data=self.gate.safety("NONE"),
            next_allowed_stage="STATIC_ONLY",
        )
        self.gate.validate_receipt(sample)
        extra = copy.deepcopy(sample)
        extra["unexpected"] = True
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_receipt(extra)
        nested = copy.deepcopy(sample)
        nested["gate_identity"]["unexpected"] = True
        with self.assertRaises(self.gate.GateError):
            self.gate.validate_receipt(nested)

    def test_command_surface_has_no_profile_or_sudo_mutator(self):
        self.assertEqual(tuple(self.policy["command_surface"]), self.gate.COMMANDS)
        self.assertNotIn("load-profile", self.gate.COMMANDS)
        self.assertNotIn("unload-profile", self.gate.COMMANDS)
        source = GATE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("sudo --", source)
        self.assertNotIn("apparmor_parser -a", source)
        self.assertNotIn("subprocess.run([\"/usr/bin/make\"", source)

    def test_source_copy_path_is_tightened_and_build_make_is_exact(self):
        copy_argv = self.gate.source_copy_argv(self.policy)
        self.assertEqual(copy_argv.count("PATH=/usr/bin"), 1)
        self.assertNotIn("PATH=/usr/bin:/usr/local/cuda-12.8/bin", copy_argv)
        build_argv = self.gate.build_argv()
        make_argv = list(self.gate.g1.MAKE_ARGV)
        self.assertEqual(build_argv[-len(make_argv) :], make_argv)
        self.assertEqual(make_argv.count("-j1"), 1)
        joined = "\n".join(make_argv)
        for token in ("-j2", "-j4", "-use_fast_math", "-march=native", "sm_86", "g++-13"):
            self.assertNotIn(token, joined)
        self.assertEqual(make_argv.count(self.gate.g1.GENCODE_ARG), 1)

    def test_static_audit_plan_is_557_exact_sandbox_commands(self):
        plan = self.gate.static_command_plan()
        self.assertEqual(len(plan), 557)
        self.assertEqual(sum(host == str(self.gate.CANDIDATE_PATH) for host, _ in plan), 11)
        cuda_suffixes = sum(suffix.startswith("cuda_object_") for _, suffix in plan)
        self.assertEqual(cuda_suffixes, 22)
        for host, suffix in (plan[0], plan[10], plan[11], plan[-1]):
            argv = self.gate.g1.build_static_audit_argv(host, suffix)
            self.gate.g1.validate_static_audit_argv(argv)
            self.assertNotIn("--bind", argv)
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)
            self.assertIn("--unshare-net", argv)
            self.assertIn("--clearenv", argv)

    def test_trace_rejects_wrong_order_and_early_receipt(self):
        trace = self.exact_trace()
        self.gate.g1.validate_source_copy_trace(self.gate.g1_trace(trace))
        wrong_order = copy.deepcopy(trace)
        wrong_order[3], wrong_order[4] = wrong_order[4], wrong_order[3]
        with self.assertRaises(self.gate.g1.GateError):
            self.gate.g1.validate_source_copy_trace(self.gate.g1_trace(wrong_order))
        early = copy.deepcopy(trace)
        early[-1]["evidence_json"] = json.dumps(
            {
                "create_new": True,
                "path": str(self.gate.SOURCE_COPY_FINAL),
                "published": True,
                "after_complete_inventory_sequence": 5,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        early[-1]["evidence_sha256"] = hashlib.sha256(
            early[-1]["evidence_json"].encode("utf-8")
        ).hexdigest()
        with self.assertRaises(self.gate.g1.GateError):
            self.gate.g1.validate_source_copy_trace(self.gate.g1_trace(early))

    def test_create_new_writer_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "receipt.json"
            self.gate.write_bytes_new(path, b"first\n", mode=0o640)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o640)
            with self.assertRaises(FileExistsError):
                self.gate.write_bytes_new(path, b"second\n", mode=0o640)
            self.assertEqual(path.read_bytes(), b"first\n")

    def test_inventory_accepts_exact_352_and_rejects_extra(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = self.make_fixture(root)
            summary, entries = self.gate.inventory_input_tree(
                root, expected, wrapper=False, label="mock_352"
            )
            self.assertEqual(summary["entry_count"], 352)
            self.assertEqual(len(entries), 352)
            (root / "extra.txt").write_text("extra\n", encoding="utf-8")
            with self.assertRaises(self.gate.GateError):
                self.gate.inventory_input_tree(root, expected, wrapper=False, label="extra")

    def test_inventory_rejects_symlink_hardlink_elf_and_execute_bit(self):
        mutations = ("symlink", "hardlink", "elf", "execute")
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                expected = self.make_fixture(root)
                victim = root / "nested_0/source_000.txt"
                if mutation == "symlink":
                    target = root / "safe-target.txt"
                    target.write_text("target\n", encoding="utf-8")
                    victim.unlink()
                    victim.symlink_to(target)
                elif mutation == "hardlink":
                    os.link(victim, root / "hardlink.txt")
                elif mutation == "elf":
                    data = b"\x7fELF" + b"mock\n"
                    victim.write_bytes(data)
                    expected["nested_0/source_000.txt"] = {
                        "sha256": hashlib.sha256(data).hexdigest(),
                        "size_bytes": len(data),
                    }
                else:
                    os.chmod(victim, 0o750)
                with self.assertRaises(self.gate.GateError):
                    self.gate.inventory_input_tree(root, expected, wrapper=False, label=mutation)

    def test_wrapper_is_o_excl_0600_and_inventory_rejects_hash_or_mode_drift(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = self.make_fixture(root)
            original = self.gate.OUTPUT_SOURCE
            self.gate.OUTPUT_SOURCE = root
            try:
                identity = self.gate.create_wrapper()
                self.assertEqual(identity["mode_octal"], "0600")
                self.assertEqual(identity["size_bytes"], 84)
                self.assertEqual(identity["sha256"], self.gate.g1.WRAPPER_SHA256)
                with self.assertRaises(FileExistsError):
                    self.gate.create_wrapper()
                summary, entries = self.gate.inventory_input_tree(
                    root, expected, wrapper=True, label="mock_353"
                )
                self.assertEqual(summary["entry_count"], 353)
                self.gate.g1.validate_build_input_inventory(entries, expected.keys())
                os.chmod(root / self.gate.WRAPPER_NAME, 0o640)
                with self.assertRaises(self.gate.GateError):
                    self.gate.inventory_input_tree(root, expected, wrapper=True, label="mode-drift")
            finally:
                self.gate.OUTPUT_SOURCE = original

    def test_candidate_disarm_uses_0400_and_rejects_linked_candidate(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = self.gate.CANDIDATE_PATH
            candidate = root / "candidate"
            candidate.write_bytes(b"not executed\n")
            os.chmod(candidate, 0o755)
            self.gate.CANDIDATE_PATH = candidate
            try:
                identity = self.gate.try_disarm_candidate()
                self.assertIsNotNone(identity)
                assert identity is not None
                self.assertEqual(identity["mode_octal"], "0400")
                os.chmod(candidate, 0o600)
                os.link(candidate, root / "candidate-hardlink")
                with self.assertRaises(self.gate.GateError):
                    self.gate.try_disarm_candidate()
            finally:
                self.gate.CANDIDATE_PATH = original

    def test_profile_lifecycle_and_b_boundary_are_closed(self):
        profiles = self.policy["profiles"]
        self.assertEqual(profiles["load_argv"][5], "-a")
        self.assertEqual(profiles["unload_argv"][5], "-R")
        self.assertTrue(profiles["replace_forbidden"])
        self.assertTrue(profiles["zero_residue_required"])
        self.assertTrue(self.policy["invariants"]["no_b_root_profile_or_receipt"])
        self.assertTrue(self.policy["invariants"]["no_g5_g6_g7"])
        self.assertFalse(os.path.lexists(self.gate.ROOT_B))


if __name__ == "__main__":
    unittest.main()
