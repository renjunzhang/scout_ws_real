"""Static contract checks for the v12 fresh-output rebinding correction."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate_v12.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v12.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v12.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate_v12", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def argv_pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == flag]


class TargetU3CpuBuildGateV12Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_is_closed_fresh_and_does_not_weaken_v11(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertEqual(self.policy["schema_version"], "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v12")
        self.assertEqual(self.policy["frozen_attempt"]["build_id"], self.gate.BUILD_ID)
        self.assertEqual(self.policy["isolation"]["synthetic_symlinks"], self.gate.EXPECTED_SYMLINKS)
        self.assertEqual(self.policy["isolation"]["host_read_only_binds"], ["/usr", "sealed source during copy only"])
        self.assertEqual(self.policy["isolation"]["host_writable_bind_count"], 1)
        self.assertTrue(self.policy["invariants"]["no_network"])
        self.assertTrue(self.policy["invariants"]["no_gpu"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])
        self.assertEqual(self.policy["resources"], self.gate.previous.core.read_json_object(self.gate.previous.POLICY_PATH)["resources"])

    def test_v11_review_layer_and_policy_are_byte_pinned(self):
        self.assertEqual(hashlib.sha256(self.gate.PREVIOUS_GATE_PATH.read_bytes()).hexdigest(), self.gate.PREVIOUS_GATE_SHA256)
        self.assertEqual(hashlib.sha256(self.gate.previous.POLICY_PATH.read_bytes()).hexdigest(), self.gate.PREVIOUS_POLICY_SHA256)
        self.assertNotEqual(self.gate.BUILD_ID, self.gate.previous.BUILD_ID)
        self.assertNotEqual(self.gate.ATTEMPT_ROOT, self.gate.previous.ATTEMPT_ROOT)

    def test_all_derived_core_paths_are_rebound_to_v12_output(self):
        self.assertEqual(self.gate.core.ATTEMPT_ROOT, self.gate.ATTEMPT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_ROOT, self.gate.OUTPUT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_SOURCE, self.gate.OUTPUT_SOURCE)
        self.assertEqual(self.gate.core.ARTIFACT_PATH, self.gate.ARTIFACT_PATH)
        self.assertTrue(self.gate.OUTPUT_SOURCE.is_relative_to(self.gate.OUTPUT_ROOT))
        self.assertTrue(self.gate.ARTIFACT_PATH.is_relative_to(self.gate.OUTPUT_ROOT))
        self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), str(self.gate.OUTPUT_SOURCE))
        self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), str(self.gate.ARTIFACT_PATH))
        self.assertEqual(self.gate.core.__file__, str(self.gate.SCRIPT_PATH))

    def test_profiles_and_bwrap_keep_the_exact_guest_link_boundary(self):
        for path, digest, name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            effective = "\n".join(line.split("#", 1)[0] for line in text.splitlines())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
            self.assertIn(f"profile {name} flags=(attach_disconnected,mediate_deleted)", text)
            self.assertEqual([line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")], ["/newroot/lib wl,", "/newroot/lib64 wl,"])
            self.assertNotIn("/newroot/lib/**", effective)
            self.assertNotIn("/oldroot/lib", effective)
            self.assertNotIn("flags=(unconfined)", effective)
        for phase in ("source-copy", "build"):
            argv = self.gate.bwrap_prefix(phase=phase)
            expected_ro = [["/usr", "/usr"]]
            if phase == "source-copy":
                expected_ro.append([str(self.gate.SOURCE_ROOT), "/work/input"])
            self.assertEqual(argv_pairs(argv, "--ro-bind"), expected_ro)
            self.assertEqual(argv_pairs(argv, "--symlink"), [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]])
            bind_pairs = argv_pairs(argv, "--ro-bind") + argv_pairs(argv, "--bind")
            self.assertFalse(any("/lib" in pair for pair in bind_pairs))
            self.assertIn("--unshare-net", argv)
            self.assertIn("--disable-userns", argv)
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)

    def test_static_review_catches_the_old_derived_path_bug(self):
        review = self.gate.verify_review_artifacts()
        self.assertEqual(review["frozen_v11_gate"]["sha256"], self.gate.PREVIOUS_GATE_SHA256)
        self.assertEqual(review["frozen_v11_policy"]["sha256"], self.gate.PREVIOUS_POLICY_SHA256)
        self.assertEqual(self.gate.core.OUTPUT_SOURCE, self.gate.OUTPUT_SOURCE)
        self.assertEqual(self.gate.core.ARTIFACT_PATH, self.gate.ARTIFACT_PATH)


if __name__ == "__main__":
    unittest.main()
