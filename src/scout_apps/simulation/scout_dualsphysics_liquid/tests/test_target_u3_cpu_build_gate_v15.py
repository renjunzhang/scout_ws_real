"""Mock-only static checks for v15's directory/file output-tree contract."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate_v15.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v15.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v15.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate_v15", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def argv_pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == flag]


class TargetU3CpuBuildGateV15Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_is_closed_and_v14_is_byte_pinned(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertEqual(
            hashlib.sha256(self.gate.PREVIOUS_GATE_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_GATE_SHA256,
        )
        self.assertEqual(
            hashlib.sha256(self.gate.previous.POLICY_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_POLICY_SHA256,
        )
        self.assertNotEqual(self.gate.BUILD_ID, self.gate.previous.BUILD_ID)
        for key in (
            "source_input",
            "isolation",
            "resources",
            "source_copy_command",
            "build_command",
            "generated_wrapper",
            "profile_lifecycle",
            "invariants",
        ):
            self.assertEqual(self.policy[key], self.gate.core.read_json_object(self.gate.previous.POLICY_PATH)[key])
        audit = self.policy["static_artifact_audit"]
        self.assertEqual(audit["post_build_object_contract"], self.gate.OBJECT_CONTRACT)
        self.assertEqual(audit["post_build_output_directory_contract"], self.gate.OUTPUT_DIRECTORY_CONTRACT)
        self.assertTrue(self.policy["invariants"]["no_network"])
        self.assertTrue(self.policy["invariants"]["no_gpu"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])

    def test_profiles_and_bwrap_are_rebound_without_widening(self):
        for path, digest, name, old_path, old_name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE, self.gate.previous.COPY_PROFILE_PATH, self.gate.previous.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE, self.gate.previous.BUILD_PROFILE_PATH, self.gate.previous.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            effective = self.gate._effective_profile(text)
            normalized = (
                effective.replace(str(self.gate.ATTEMPT_ROOT), str(self.gate.previous.ATTEMPT_ROOT))
                .replace(str(self.gate.OUTPUT_ROOT), str(self.gate.previous.OUTPUT_ROOT))
                .replace(name, old_name)
            )
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
            self.assertEqual(normalized, self.gate.previous._effective_profile(old_path.read_text(encoding="utf-8")))
            self.assertNotIn("flags=(unconfined)", effective)
        for phase in ("source-copy", "build"):
            argv = self.gate.bwrap_prefix_v15(phase=phase)
            expected_ro = [["/usr", "/usr"]]
            if phase == "source-copy":
                expected_ro.append([str(self.gate.core.SOURCE_ROOT), "/work/input"])
            self.assertEqual(argv_pairs(argv, "--ro-bind"), expected_ro)
            self.assertEqual(argv_pairs(argv, "--symlink"), [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]])
            self.assertEqual(argv_pairs(argv, "--bind"), [[str(self.gate.OUTPUT_ROOT), "/work/output"]])
            self.assertIn("--unshare-net", argv)
            self.assertIn("--disable-userns", argv)
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)
            self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), argv)

    def test_output_tree_accepts_normal_directory_links_but_rejects_symlink_and_hardlink(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nested = root / "buildtree" / "src" / "source"
            nested.mkdir(parents=True)
            (nested / "safe.txt").write_bytes(b"safe regular file\n")
            self.assertGreater((root / "buildtree").stat().st_nlink, 1)
            self.gate.assert_safe_output_tree_entries_v15(root)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target.mkdir()
            (root / "directory-link").symlink_to(target, target_is_directory=True)
            with self.assertRaises(self.gate.core.GateError):
                self.gate.assert_safe_output_tree_entries_v15(root)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            primary = root / "primary.bin"
            primary.write_bytes(b"one link is required\n")
            os.link(primary, root / "hardlink.bin")
            with self.assertRaises(self.gate.core.GateError):
                self.gate.assert_safe_output_tree_entries_v15(root)

    def test_static_review_and_transition_use_the_v15_inventory(self):
        review = self.gate.verify_review_artifacts_v15()
        self.assertEqual(review["frozen_v14_gate"]["sha256"], self.gate.PREVIOUS_GATE_SHA256)
        self.assertEqual(review["object_contract"], self.gate.OBJECT_CONTRACT)
        self.assertEqual(review["output_directory_contract"], self.gate.OUTPUT_DIRECTORY_CONTRACT)
        self.assertIs(self.gate.core.inventory_build_output, self.gate.inventory_build_output_v15)
        self.assertIs(self.gate.core.execute_build, self.gate.execute_build_v15)


if __name__ == "__main__":
    unittest.main()
