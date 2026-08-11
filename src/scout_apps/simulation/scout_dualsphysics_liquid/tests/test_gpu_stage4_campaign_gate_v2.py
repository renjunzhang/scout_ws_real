from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
SCRIPT = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v2.py"
PREVIOUS_POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T162710Z_v1.json"
PREVIOUS_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T162710Z-a.profile"
NEW_PROFILE = PACKAGE / "config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T165034Z-a.profile"


def load_gate():
    spec = importlib.util.spec_from_file_location("gpu_stage4_campaign_gate_v2_tested", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuStage4CampaignGateV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = cls.gate.policy()
        cls.previous = json.loads(PREVIOUS_POLICY.read_text(encoding="utf-8"))

    def test_revision_is_evidence_bound_and_single_delta(self):
        result = self.gate.revision_check()
        self.assertEqual(result["status"], "PASS_GENCODE_LIST_REMEDIATION_CONTRACT")
        self.assertEqual(result["semantic_delta_count"], 1)
        self.assertTrue(result["previous_zero_residue"])

    def test_corrected_cuda_list_is_exact_and_shell_quote_free(self):
        gencode = [item for item in self.policy["build"]["make_argv"] if item.startswith("GENCODE=")]
        self.assertEqual(gencode, [self.gate.EXACT_GENCODE])
        self.assertEqual(gencode[0].count("compute_120"), 2)
        self.assertEqual(gencode[0].count("sm_120"), 1)
        self.assertNotIn('"', gencode[0])
        self.assertIn("code=[sm_120,compute_120]", gencode[0])

    def test_build_contract_delta_is_only_gencode(self):
        previous_build = dict(self.previous["build"])
        current_build = dict(self.policy["build"])
        old_argv = list(previous_build.pop("make_argv"))
        new_argv = list(current_build.pop("make_argv"))
        self.assertEqual(previous_build, current_build)
        self.assertEqual(len(old_argv), len(new_argv))
        changed = [(old, new) for old, new in zip(old_argv, new_argv) if old != new]
        self.assertEqual(changed, [
            (
                'GENCODE=-gencode=arch=compute_120,code="sm_120,compute_120"',
                "GENCODE=-gencode=arch=compute_120,code=[sm_120,compute_120]",
            )
        ])
        for key in ("source", "wrapper", "object_contract", "invariants"):
            self.assertEqual(self.previous[key], self.policy[key])

    def test_profile_permission_set_did_not_expand(self):
        old = PREVIOUS_PROFILE.read_text(encoding="utf-8")
        new = NEW_PROFILE.read_text(encoding="utf-8")
        old_lines = {
            line.strip()
            for line in old.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        new_lines = {
            line.strip()
            for line in new.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        normalize = lambda line: line.replace("20260810T162710Z", "CAMPAIGN").replace("20260810T165034Z", "CAMPAIGN")
        self.assertEqual({normalize(line) for line in old_lines}, {normalize(line) for line in new_lines})

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
