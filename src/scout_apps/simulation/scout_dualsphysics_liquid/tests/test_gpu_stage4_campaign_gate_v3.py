from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
SCRIPT = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v3.py"
PREVIOUS_POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T165034Z_v2.json"
PREVIOUS_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T165034Z-a.profile"
NEW_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T170339Z-a.profile"


def load_gate():
    spec = importlib.util.spec_from_file_location("gpu_stage4_campaign_gate_v3_tested", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuStage4CampaignGateV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = cls.gate.policy()
        cls.previous = json.loads(PREVIOUS_POLICY.read_text(encoding="utf-8"))

    def test_revision_is_previous_failure_bound_and_single_delta(self):
        result = self.gate.revision_check()
        self.assertEqual(result["status"], "PASS_POSIX_SH_ALIAS_REMEDIATION_CONTRACT")
        self.assertEqual(result["semantic_delta_count"], 1)
        self.assertEqual(result["sandbox_argv_delta"], ["--symlink", "usr/bin", "/bin"])
        self.assertTrue(result["previous_zero_residue"])

    def test_bwrap_delta_is_exactly_usr_bin_alias(self):
        old = self.gate.BASE_BUILD_CHILD_ARGV(self.policy)
        new = self.gate.build_child_argv(self.policy)
        self.assertEqual(len(new), len(old) + 3)
        index = new.index("/bin") - 2
        self.assertEqual(new[index:index + 3], ["--symlink", "usr/bin", "/bin"])
        self.assertEqual(new[:index] + new[index + 3:], old)
        self.assertIn("--unshare-net", new)
        self.assertFalse(any(item.startswith("/dev/nvidia") for item in new))

    def test_compiler_source_architecture_and_limits_are_unchanged(self):
        for key in ("source", "wrapper", "build", "object_contract", "invariants"):
            self.assertEqual(self.policy[key], self.previous[key])
        joined = "\n".join(self.policy["build"]["make_argv"])
        self.assertIn("code=[sm_120,compute_120]", joined)
        self.assertIn("g++-11", joined)
        self.assertNotIn("g++-13", joined)
        self.assertNotIn("-j2", joined)

    def test_profile_permission_delta_is_exactly_newroot_bin_symlink(self):
        old_lines = {
            line.strip()
            for line in PREVIOUS_PROFILE.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        new_lines = {
            line.strip()
            for line in NEW_PROFILE.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        normalize = lambda line: line.replace("20260810T165034Z", "CAMPAIGN").replace("20260810T170339Z", "CAMPAIGN")
        old_normalized = {normalize(line) for line in old_lines}
        new_normalized = {normalize(line) for line in new_lines}
        self.assertEqual(new_normalized - old_normalized, {"/newroot/bin wl,"})
        self.assertEqual(old_normalized - new_normalized, set())

    def test_fresh_root_and_receipts_are_unused(self):
        self.assertFalse(Path(self.policy["paths"]["attempt_root"]).exists())
        for receipt in self.policy["receipts"].values():
            self.assertFalse(Path(receipt).exists(), receipt)

    def test_base_validators_accept_tools_source_objects_and_profile(self):
        parent = self.gate.BASE.verify_parents_and_tools(self.policy)
        sealed = self.gate.BASE.validate_sealed_source(self.policy)
        self.assertEqual(parent["object_contract"]["total_object_count"], 131)
        self.assertEqual(sealed, {"file_count": 352, "total_bytes": 5473917})


if __name__ == "__main__":
    unittest.main()
