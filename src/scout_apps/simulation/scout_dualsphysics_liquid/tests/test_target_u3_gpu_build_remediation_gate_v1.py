"""Static and negative tests for fresh-campaign GPU build remediation v1."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_gpu_build_remediation_gate_v1.py"
GATE_SHA256 = "1f759c59d7ede1ce03fde2b3f2885debc87052927cbf31191bf4e2f61e251986"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gpu_build_remediation_gate_v1", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TargetU3GpuBuildRemediationGateV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.schema = json.loads(cls.gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        cls.draft = json.loads(cls.gate.DRAFT_PATH.read_text(encoding="utf-8"))

    def test_gate_and_all_frozen_parents_are_byte_pinned(self):
        self.assertEqual(hashlib.sha256(GATE_PATH.read_bytes()).hexdigest(), GATE_SHA256)
        self.gate.verify_frozen_inputs()
        self.assertEqual(len(self.gate.G1_PARENT_HASHES), 9)
        self.assertEqual(len(self.gate.G2_PARENT_HASHES), 6)

    def test_schema_is_recursive_closed_and_accepts_only_exact_draft(self):
        Draft202012Validator.check_schema(self.schema)
        self.gate.verify_recursive_schema_closure(self.schema)
        Draft202012Validator(self.schema).validate(self.draft)
        mutated = copy.deepcopy(self.draft)
        mutated["semantic_delta"]["rule"]["extra"] = "forbidden"
        self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(mutated)))

    def test_template_diff_is_exactly_one_insertion_and_zero_deletions(self):
        result = self.gate.verify_template_delta()
        self.assertEqual(result["path"], "/newroot/work/tmp/")
        self.assertEqual(result["access_kind"], "rw_guest_tmp_only")
        self.assertEqual(result["insertion_count"], 1)
        self.assertEqual(result["deletion_count"], 0)
        self.assertEqual(result["apparmor_rule_sha256"], self.gate.DELTA_RULE_SHA256)

    def test_template_rejects_extra_path_permission_or_deletion(self):
        base = self.gate.BASE_TEMPLATE_PATH.read_bytes()
        good = self.gate.REMEDIATION_TEMPLATE_PATH.read_bytes()
        mutations = (
            good.replace(b"\n}", b"\n  /home/** rw,\n}", 1),
            good.replace(b"\n}", b"\n  /dev/nvidia* rw,\n}", 1),
            good.replace(b"\n}", b"\n  network stream,\n}", 1),
            good.replace(b"\n}", b"\n  change_profile unsafe,\n}", 1),
            good.replace(self.gate.DELTA_RULE.encode(), b"", 1),
            good.replace(b"  /newroot/work/tmp/ rw,", b"  /newroot/work/** rw,", 1),
        )
        for value in mutations:
            with self.subTest(value=value[-80:]):
                with self.assertRaises(self.gate.RemediationError):
                    self.gate.verify_template_delta(base, value)

    def test_schema_rejects_gxx13_toolchain_arch_and_permission_drift(self):
        validator = Draft202012Validator(self.schema)
        mutations = []
        value = copy.deepcopy(self.draft)
        value["invariants"]["gxx13_added"] = True
        mutations.append(value)
        value = copy.deepcopy(self.draft)
        value["invariants"]["toolchain"] = "GXX_13"
        mutations.append(value)
        value = copy.deepcopy(self.draft)
        value["invariants"]["architectures"] = ["compute_90", "sm_90"]
        mutations.append(value)
        value = copy.deepcopy(self.draft)
        value["semantic_delta"]["rule"]["path"] = "/home/zrj/"
        mutations.append(value)
        value = copy.deepcopy(self.draft)
        value["semantic_delta"]["rule"]["access_kind"] = "rw_host_output"
        mutations.append(value)
        value = copy.deepcopy(self.draft)
        value["invariants"]["network_permission_added"] = True
        mutations.append(value)
        for document in mutations:
            with self.subTest(document=document):
                self.assertTrue(list(validator.iter_errors(document)))

    def test_generator_is_deterministic_and_only_replaces_identity_tokens(self):
        name = "r8-liquid-u3-gpu-build-fresh-offline-test-v1"
        root = "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_fresh_offline_test_v1.partial"
        first = self.gate.render_profile(name, root)
        second = self.gate.render_profile(name, root)
        self.assertEqual(first, second)
        self.assertEqual(first.count(name.encode()), 3)
        self.assertEqual(first.count(root.encode()), 4)
        self.assertNotIn(b"@@", first)
        self.assertEqual(first.decode().splitlines().count("  /newroot/work/tmp/ rw,"), 1)

    def test_generator_rejects_old_relative_existing_and_outside_roots(self):
        valid_name = "r8-liquid-u3-gpu-build-fresh-test-v1"
        invalid = (
            (valid_name, str(self.gate.OLD_ROOT_A)),
            (valid_name, str(self.gate.OLD_ROOT_B)),
            (valid_name, "relative.partial"),
            (valid_name, "/tmp/outside.partial"),
            (
                "r8-liquid-u3-gpu-build-u3-source-gpu-build-sm120-20260810t102641z",
                "/home/zrj/scout_liquid_lab/build/fresh.partial",
            ),
        )
        for name, root in invalid:
            with self.subTest(name=name, root=root):
                with self.assertRaises(self.gate.RemediationError):
                    self.gate.render_profile(name, root)
        with self.assertRaises(self.gate.RemediationError):
            self.gate.render_profile(valid_name, str(self.gate.OLD_ROOT_A.parent))

    def test_rendered_active_rules_add_no_forbidden_surface(self):
        name = "r8-liquid-u3-gpu-build-fresh-surface-test-v1"
        root = "/home/zrj/scout_liquid_lab/build/fresh_surface_test_v1.partial"
        rendered = self.gate.render_profile(name, root)
        active = "\n".join(self.gate.active_lines(rendered))
        for token in (
            "g++-13",
            "/dev/nvidia",
            "network stream",
            "network packet",
            "flags=(unconfined)",
            "/home/** w",
        ):
            self.assertNotIn(token, active)

    def test_n1_evidence_requires_timebox_without_candidate_or_retry(self):
        index = self.gate.read_json(self.gate.N1_EVIDENCE_INDEX)
        final = self.gate.read_json(self.gate.N1_FINAL_FAILURE)
        self.assertEqual(final["status"], "TIMEBOX_EXHAUSTED")
        self.assertEqual(final["exact_blocker"], "T_PLUS_6_REACHED_WITHOUT_COMPLETE_CANDIDATE")
        self.assertFalse(final["attempt_b"]["retry_used"])
        self.assertEqual(index["g4_evidence"]["candidate"], "ABSENT")
        self.assertEqual(index["g4_evidence"]["kernel_audit_normalized_line_sha256"],
                         "6aff32b36b40233301ee853c23b9f98c925c48a42ca1232acef753e51b7b5b69")

    def test_future_identity_is_explicitly_not_frozen_or_materialized(self):
        self.assertEqual(self.draft["future_identity"], {
            "campaign_id": "NOT_FROZEN",
            "build_id": "NOT_FROZEN",
            "attempt_root": "NOT_FROZEN",
            "profile_name": "NOT_FROZEN",
            "profile_instance_path": "NOT_MATERIALIZED",
            "profile_instance_sha256": "NOT_MATERIALIZED",
        })
        self.gate.verify_non_materialization()

    def test_query_contract_is_nonloading_ephemeral_and_not_authorizable(self):
        observed = {}

        def fake_run(argv, **kwargs):
            if argv and argv[0] == str(self.gate.APPARMOR_PARSER):
                self.assertEqual(argv[1:5], ["-Q", "-K", "-T", "--"])
                path = Path(argv[5])
                self.assertTrue(path.is_file())
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
                observed["path"] = path
                return subprocess.CompletedProcess(argv, 0, b"", b"")
            return subprocess.CompletedProcess(argv, 0, b"stable-status", b"")

        with mock.patch.object(self.gate, "self_check", return_value={"status": "PASS"}), \
             mock.patch.object(self.gate.subprocess, "run", side_effect=fake_run):
            result = self.gate.query_template()
        self.assertEqual(result["status"], "PASS_NON_LOADING_APPARMOR_REMEDIATION_TEMPLATE_QUERY")
        self.assertFalse(result["profile_loaded"])
        self.assertFalse(result["sudo_used"])
        self.assertFalse(result["persistent_profile_instance"])
        self.assertFalse(result["authorization_eligible"])
        self.assertFalse(observed["path"].exists())

    def test_cli_surface_has_no_materialize_load_sudo_or_build_command(self):
        source = GATE_PATH.read_text(encoding="utf-8")
        self.assertIn('choices=("self-check", "query-template")', source)
        self.assertNotIn('"render-profile"', source)
        self.assertNotIn('"materialize"', source)
        self.assertNotIn('/usr/bin/sudo', source)
        self.assertNotIn('apparmor_parser", "-a"', source)
        self.assertNotIn('apparmor_parser", "-R"', source)

    def test_full_self_check_passes_without_system_or_campaign_action(self):
        result = self.gate.self_check()
        self.assertEqual(result["status"], "PASS_FRESH_CAMPAIGN_OFFLINE_REMEDIATION_SELF_CHECK")
        self.assertEqual(result["parent_count"], 15)
        self.assertFalse(result["profile_instance_persisted"])
        self.assertFalse(result["profile_instance_authorization_eligible"])
        self.assertFalse(result["new_campaign_created"])
        self.assertFalse(result["build_root_created"])
        self.assertFalse(result["source_copied"])
        self.assertFalse(result["sudo_used"])
        self.assertFalse(result["profile_loaded"])
        self.assertFalse(result["make_nvcc_candidate_run"])
        self.assertEqual(
            result["next_allowed_stage"],
            "FRESH_CAMPAIGN_AND_EXACT_PROFILE_HASH_USER_AUTHORIZATION_REQUIRED",
        )


if __name__ == "__main__":
    unittest.main()
