"""Static-only checks for the create-new v3 process admission layer."""

from __future__ import annotations

import hashlib
import importlib.util
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gpu_build_gate_v3.py"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gpu_build_gate_v3", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TargetU3GpuBuildGateV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()

    def test_v2_core_is_byte_pinned_and_g1_parents_remain_unchanged(self):
        self.assertEqual(
            hashlib.sha256(self.gate.PREVIOUS_GATE_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_GATE_SHA256,
        )
        self.assertEqual(self.gate.core.G1_GATE_SHA256, "e34ab0facc3c665c6befbb792c7344e0af6e652fd3c6b30dfa81ca8cbea5b502")
        self.assertEqual(self.gate.core.POLICY_SHA256, "17472c30021bd70d81180fca6be1317a84cf5ae6f104e0c7cc707dc88ed4b58d")
        self.assertEqual(self.gate.core.RECEIPT_SCHEMA_SHA256, "10ede832aae19632b9b5543b9916a7d1803e210c743db90910a66a37a1e02173")

    def test_unrelated_desktop_bwrap_is_not_a_build_conflict(self):
        for argv in (
            "bwrap --args 42 -- zotero -url",
            "bwrap --args 42 -- xdg-dbus-proxy --args=44",
        ):
            self.assertFalse(self.gate.is_conflicting_process("bwrap", argv))

    def test_liquid_sandboxes_and_build_tools_still_conflict(self):
        self.assertTrue(
            self.gate.is_conflicting_process(
                "bwrap", f"/usr/bin/bwrap --bind {self.gate.core.OUTPUT_ROOT} /work/output"
            )
        )
        self.assertTrue(
            self.gate.is_conflicting_process(
                "aa-exec", "aa-exec -p r8-liquid-u3-gpu-build-20260810T102641Z-a"
            )
        )
        for comm in self.gate.ALWAYS_CONFLICTING_NAMES:
            self.assertTrue(self.gate.is_conflicting_process(comm, comm))

    def test_only_classifier_self_check_and_script_identity_are_rebound(self):
        self.assertIs(self.gate.core.conflicting_processes, self.gate.conflicting_processes_v3)
        self.assertIs(self.gate.core.self_check, self.gate.self_check_v3)
        self.assertEqual(self.gate.core.SCRIPT_PATH, self.gate.SCRIPT_PATH)
        self.assertEqual(tuple(self.gate.core.COMMANDS), (
            "self-check",
            "preflight",
            "prepare-source-copy",
            "run-source-copy",
            "finalize-source-copy",
            "prepare-build",
            "run-build",
            "run-static-audit",
        ))

    def test_v3_self_check_preserves_static_no_action_state(self):
        result = self.gate.self_check_v3()
        self.assertEqual(result["status"], "PASS_G2A_EXECUTION_LAYER_STATIC_SELF_CHECK_V3")
        self.assertEqual(result["process_classifier"], {
            "zotero_bwrap_conflicts": False,
            "liquid_bwrap_conflicts": True,
            "make_conflicts": True,
        })
        self.assertFalse(result["system_actions_performed"])
        self.assertFalse(result["make_nvcc_compiler_run"])
        self.assertFalse(result["profile_loaded"])
        self.assertFalse(result["candidate_executed"])


if __name__ == "__main__":
    unittest.main()
