"""Static contract checks for the v11 /lib guest-link U3 build admission."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate_v11.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v11.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v11.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate_v11", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def argv_pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == flag]


class TargetU3CpuBuildGateV11Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_schema_is_closed_and_fresh(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertEqual(self.policy["schema_version"], "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v11")
        self.assertEqual(self.policy["frozen_attempt"]["build_id"], self.gate.BUILD_ID)
        self.assertIn("v11", self.policy["frozen_attempt"]["source_copy_profile"])
        self.assertIn("v11", self.policy["frozen_attempt"]["build_profile"])
        self.assertTrue(self.policy["user_delegated_review_and_execution_approval"])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check", "preflight", "run-source-copy", "run-build"])
        self.assertEqual(self.policy["outer_privilege_handoff"]["required_supplementary_group_count"], 0)
        self.assertTrue(self.policy["invariants"]["no_network"])
        self.assertTrue(self.policy["invariants"]["no_gpu"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])

    def test_v10_core_is_byte_pinned_and_v11_rebinds_every_versioned_path(self):
        self.assertEqual(hashlib.sha256(self.gate.CORE_PATH.read_bytes()).hexdigest(), self.gate.CORE_SHA256)
        self.assertEqual(self.gate.core.BUILD_ID, self.gate.BUILD_ID)
        self.assertEqual(self.gate.core.POLICY_PATH, self.gate.POLICY_PATH)
        self.assertEqual(self.gate.core.SCHEMA_PATH, self.gate.SCHEMA_PATH)
        self.assertEqual(self.gate.core.COPY_PROFILE, self.gate.COPY_PROFILE)
        self.assertEqual(self.gate.core.BUILD_PROFILE, self.gate.BUILD_PROFILE)
        self.assertEqual(self.gate.core.__file__, str(self.gate.SCRIPT_PATH))
        self.assertNotEqual(self.gate.ATTEMPT_ROOT.name, "u3_source_cpu_build_20260806T182853Z.partial")

    def test_profiles_are_exactly_limited_to_two_guest_link_creation_rules(self):
        for path, expected_hash, profile_name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            effective = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected_hash)
            self.assertIn(f"profile {profile_name} flags=(attach_disconnected,mediate_deleted)", text)
            self.assertEqual(
                [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")],
                ["/newroot/lib wl,", "/newroot/lib64 wl,"],
            )
            self.assertNotIn("/newroot/lib/**", effective)
            self.assertNotIn("/newroot/lib rw", effective)
            self.assertNotIn("/oldroot/lib", effective)
            self.assertNotIn("flags=(unconfined)", effective)
            self.assertNotIn("change_profile", effective)
            self.assertNotIn("DualSPHysics5.4CPU_linux64 rix", effective)
            self.assertIn(str(self.gate.ATTEMPT_ROOT), effective)

    def test_bwrap_has_only_usr_system_bind_and_exact_relative_lib_links(self):
        for phase in ("source-copy", "build"):
            argv = self.gate.bwrap_prefix(phase=phase)
            expected_ro = [["/usr", "/usr"]]
            if phase == "source-copy":
                expected_ro.append([str(self.gate.SOURCE_ROOT), "/work/input"])
            self.assertEqual(argv_pairs(argv, "--ro-bind"), expected_ro)
            self.assertEqual(
                argv_pairs(argv, "--symlink"),
                [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]],
            )
            bind_pairs = argv_pairs(argv, "--ro-bind") + argv_pairs(argv, "--bind")
            self.assertFalse(any("/lib" in pair for pair in bind_pairs))
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)
            self.assertIn("--unshare-net", argv)
            self.assertIn("--disable-userns", argv)
            self.assertIn("--assert-userns-disabled", argv)
            self.assertNotIn("--share-net", argv)
            self.assertNotIn("usr/bin", [item for item in argv if item.startswith("usr/")])
        self.assertNotIn("/bin", self.gate.bwrap_prefix(phase="build"))

    def test_policy_and_gate_agree_on_only_guest_synthetic_links(self):
        self.assertEqual(self.policy["isolation"]["synthetic_symlinks"], self.gate.EXPECTED_SYMLINKS)
        self.assertEqual(self.policy["isolation"]["host_read_only_binds"], ["/usr", "sealed source during copy only"])
        self.assertEqual(self.policy["isolation"]["host_writable_binds"], ["one exact new output root"])
        self.assertEqual(self.policy["isolation"]["host_writable_bind_count"], 1)
        self.assertNotIn("/lib", self.policy["isolation"]["host_read_only_binds"])
        self.assertEqual(self.policy["generated_wrapper"]["content"], self.gate.WRAPPER_CONTENT)
        self.assertEqual(self.gate.WRAPPER_CONTENT.count("-include"), 1)
        self.assertIn("-include cstdint", self.gate.WRAPPER_CONTENT)


if __name__ == "__main__":
    unittest.main()
