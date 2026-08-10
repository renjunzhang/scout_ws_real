#!/usr/bin/env python3
"""Mock-only tests for the source-only CPU-build AppArmor review draft."""

from __future__ import annotations

import copy
import inspect
import json
import sys
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_apparmor_source_cpu_build_draft_gate as draft_gate  # noqa: E402


class TargetAppArmorSourceCpuBuildDraftTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(draft_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(draft_gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.draft = draft_gate.DRAFT_PATH.read_text(encoding="utf-8")

    def test_checked_in_draft_is_static_only_and_build_remains_no_go(self) -> None:
        self.assertEqual(draft_gate.validate_policy(self.policy), [])
        self.assertEqual(draft_gate.validate_schema(self.schema), [])
        self.assertEqual(draft_gate.validate_draft_text(self.draft), [])
        self.assertTrue(self.policy["draft_review_authorized"])
        self.assertFalse(self.policy["profile_loading_authorized"])
        self.assertFalse(self.policy["source_copy_execution_authorized"])
        self.assertFalse(self.policy["build_execution_authorized"])
        self.assertFalse(self.policy["output_execution_authorized"])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check"])

    def test_draft_rejects_exec_mount_network_and_unconfined_expansions(self) -> None:
        with_exec = self.draft.replace("  userns,", "  userns,\n  /usr/bin/make rix,")
        errors = draft_gate.validate_draft_text(with_exec)
        self.assertIn("draft has a forbidden permission token: /usr/bin/", errors)

        with_mount = self.draft.replace("  userns,", "  userns,\n  mount,")
        errors = draft_gate.validate_draft_text(with_mount)
        self.assertIn("draft has a forbidden permission token: mount", errors)

        with_network = self.draft.replace("  userns,", "  userns,\n  network,")
        errors = draft_gate.validate_draft_text(with_network)
        self.assertIn("draft has a forbidden permission token: network", errors)

        unconfined = self.draft.replace(
            "flags=(attach_disconnected,mediate_deleted)", "flags=(unconfined)"
        )
        errors = draft_gate.validate_draft_text(unconfined)
        self.assertIn("draft has a forbidden permission token: flags=(unconfined)", errors)

    def test_policy_rejects_profile_or_build_authorization(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["profile_loading_authorized"] = True
        changed["source_copy_execution_authorized"] = True
        changed["build_execution_authorized"] = True
        changed["invariants"].pop("no_mount_permission")
        errors = draft_gate.validate_policy(changed)
        self.assertIn("policy field mismatch: profile_loading_authorized", errors)
        self.assertIn("policy field mismatch: source_copy_execution_authorized", errors)
        self.assertIn("policy field mismatch: build_execution_authorized", errors)
        self.assertIn("source CPU build draft safety invariants are incomplete", errors)

    def test_static_report_is_mocked_and_declares_no_runtime_action(self) -> None:
        materialization = self.policy["required_evidence"]["source_materialization"]
        source_receipt = {"receipt_sha256": materialization["receipt_sha256"]}
        with mock.patch.object(
            draft_gate, "_verify_target_profile", return_value={"approved_root": str(draft_gate.LIQUID_ROOT)}
        ), mock.patch.object(
            draft_gate,
            "_verify_simple_receipt",
            side_effect=[
                (source_receipt, {"status": materialization["status"]}),
                ({}, {"status": "PASS_RESTRICTED_ONE_MARKER_OUTPUT_BIND_SMOKE"}),
            ],
        ), mock.patch.object(
            draft_gate,
            "_verify_sealed_source_tree",
            return_value={"file_count": 352, "total_bytes": 5473917},
        ), mock.patch.object(
            draft_gate,
            "_verify_source_cpu_build_policy",
            return_value={"build_execution_authorized": False},
        ):
            report = draft_gate.static_report(
                self.policy,
                self.schema,
                self.draft,
                draft_gate._file_sha256(draft_gate.DRAFT_PATH),
            )
        self.assertEqual(report["status"], "PASS_APPARMOR_SOURCE_CPU_BUILD_DRAFT_STATIC_ONLY")
        for key in (
            "profile_copied_to_system",
            "profile_parser_invoked",
            "profile_loaded",
            "profile_selected_for_execution",
            "namespace_attempted",
            "source_copied",
            "source_checkout_created",
            "source_materialized",
            "make_executed",
            "compiler_executed",
            "output_executed",
            "precompiled_binary_executed",
            "network_used",
            "gpu_device_exposed",
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

    def test_schema_is_closed_and_profile_body_is_exact(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(
            draft_gate._effective_profile_lines(self.draft), draft_gate.EXPECTED_PROFILE_LINES
        )
        effective = "\n".join(draft_gate._effective_profile_lines(self.draft))
        for forbidden in draft_gate.FORBIDDEN_PROFILE_TOKENS:
            self.assertNotIn(forbidden, effective)


if __name__ == "__main__":
    unittest.main()
