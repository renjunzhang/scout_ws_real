#!/usr/bin/env python3
"""Mock-only tests for the MSI AppArmor output-bind review draft."""

from __future__ import annotations

import copy
import inspect
import json
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_apparmor_output_bind_draft_gate as draft_gate  # noqa: E402


class TargetAppArmorOutputBindDraftTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(draft_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(draft_gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.draft = draft_gate.DRAFT_PATH.read_text(encoding="utf-8")

    def test_checked_in_artifact_is_static_only_and_default_deny(self) -> None:
        self.assertEqual(draft_gate.validate_policy(self.policy), [])
        self.assertEqual(draft_gate.validate_schema(self.schema), [])
        self.assertEqual(draft_gate.validate_draft_text(self.draft), [])
        self.assertTrue(self.policy["draft_review_authorized"])
        self.assertFalse(self.policy["profile_installation_authorized"])
        self.assertFalse(self.policy["profile_loading_authorized"])
        self.assertFalse(self.policy["profile_execution_authorized"])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check"])
        self.assertEqual(
            self.policy["profile_draft"]["approved_write_paths"],
            [
                "/home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/",
                (
                    "/home/zrj/scout_liquid_lab/build/"
                    "u3_bwrap_output_bind_smoke_v1_*/output/.r8_output_bind_smoke_v1"
                ),
            ],
        )

    def test_draft_rejects_network_mount_unconfined_and_broad_path_changes(self) -> None:
        with_network = self.draft.replace("  userns,", "  userns,\n  network,")
        errors = draft_gate.validate_draft_text(with_network)
        self.assertIn("draft has a forbidden permission token: network", errors)

        with_mount = self.draft.replace("  userns,", "  userns,\n  mount,")
        errors = draft_gate.validate_draft_text(with_mount)
        self.assertIn("draft has a forbidden permission token: mount", errors)

        unconfined = self.draft.replace(
            "flags=(attach_disconnected,mediate_deleted)", "flags=(unconfined)"
        )
        errors = draft_gate.validate_draft_text(unconfined)
        self.assertIn("draft has a forbidden permission token: flags=(unconfined)", errors)

        broad_path = self.draft.replace(
            "/home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/ w,",
            "/** rw,",
        )
        errors = draft_gate.validate_draft_text(broad_path)
        self.assertIn("draft has a forbidden permission token: /**", errors)

    def test_policy_rejects_activation_and_confinement_relaxations(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["profile_loading_authorized"] = True
        changed["confinement_contract"]["mount_operations"] = "allowed"
        changed["invariants"].pop("no_mount_permission")
        errors = draft_gate.validate_policy(changed)
        self.assertIn("policy field mismatch: profile_loading_authorized", errors)
        self.assertIn("confinement contract weakens default-deny boundary", errors)
        self.assertIn("AppArmor draft safety invariants are incomplete", errors)

    def test_static_report_declares_no_host_or_execution_action(self) -> None:
        report = draft_gate.static_report(
            self.policy,
            self.schema,
            self.draft,
            draft_gate._file_sha256(draft_gate.DRAFT_PATH),
        )
        self.assertEqual(report["status"], "PASS_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_ONLY")
        for key in (
            "profile_copied_to_system",
            "profile_parser_invoked",
            "profile_loaded",
            "profile_selected_for_execution",
            "namespace_attempted",
            "network_used",
            "source_checkout_created",
            "source_materialized",
            "upstream_code_executed",
            "build_executed",
            "sudo_used",
            "sysctl_or_apparmor_changed",
        ):
            self.assertFalse(report[key], key)

    def test_gate_has_no_process_or_write_execution_surface(self) -> None:
        source = inspect.getsource(draft_gate)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("os.system", source)
        self.assertNotIn("Popen(", source)
        self.assertNotIn("write_text(", source)
        self.assertNotIn("mkdir(", source)

    def test_schema_is_closed_and_draft_has_only_expected_effective_rules(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(
            draft_gate._effective_profile_lines(self.draft), draft_gate.EXPECTED_PROFILE_LINES
        )
        effective = "\n".join(draft_gate._effective_profile_lines(self.draft))
        for forbidden_prefix in self.policy["confinement_contract"]["forbidden_host_path_prefixes"]:
            self.assertNotIn(forbidden_prefix, effective)


if __name__ == "__main__":
    unittest.main()
