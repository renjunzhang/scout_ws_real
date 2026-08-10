#!/usr/bin/env python3
"""Mock-only tests for the MSI transient AppArmor userns-probe record."""

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

import r8_liquid_target_apparmor_userns_probe_gate as probe_gate  # noqa: E402


class TargetAppArmorUsernsProbeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(probe_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(probe_gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.profile = probe_gate.PROFILE_PATH.read_text(encoding="utf-8")

    def test_checked_in_record_is_static_only_and_fail_closed_at_mount(self) -> None:
        self.assertEqual(probe_gate.validate_policy(self.policy), [])
        self.assertEqual(probe_gate.validate_schema(self.schema), [])
        self.assertEqual(probe_gate.validate_profile_text(self.profile), [])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check"])
        self.assertFalse(self.policy["profile_loading_or_replay_authorized"])
        self.assertFalse(self.policy["system_configuration_change_authorized"])
        self.assertEqual(self.policy["profile_contract"]["mount_rule"], "absent_default_deny")
        self.assertEqual(self.policy["observed_result"]["returncode"], 1)
        self.assertEqual(
            self.policy["observed_result"]["first_nonpermitted_operation"]["apparmor_class"],
            "mount",
        )

    def test_profile_rejects_mount_network_stream_and_unconfined_expansions(self) -> None:
        with_mount = self.profile.replace("  userns create,", "  userns create,\n  mount,")
        self.assertIn(
            "profile has a forbidden permission token: mount,",
            probe_gate.validate_profile_text(with_mount),
        )

        with_stream = self.profile.replace(
            "  network inet dgram,", "  network inet dgram,\n  network inet stream,"
        )
        self.assertIn(
            "profile has a forbidden permission token: network inet stream",
            probe_gate.validate_profile_text(with_stream),
        )

        unconfined = self.profile.replace(
            "flags=(attach_disconnected,mediate_deleted)", "flags=(unconfined)"
        )
        self.assertIn(
            "profile has a forbidden permission token: flags=(unconfined)",
            probe_gate.validate_profile_text(unconfined),
        )

    def test_policy_rejects_replay_writable_bind_and_status_relaxations(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["profile_loading_or_replay_authorized"] = True
        changed["fixed_probe_contract"]["host_writable_mounts"] = ["/home/zrj/scout_liquid_lab"]
        changed["status"] = "PASS_OUTPUT_BIND"
        errors = probe_gate.validate_policy(changed)
        self.assertIn("policy field mismatch: profile_loading_or_replay_authorized", errors)
        self.assertIn("fixed probe contract differs", errors)
        self.assertIn("policy field mismatch: status", errors)

    def test_static_report_declares_no_new_host_or_execution_action(self) -> None:
        report = probe_gate.static_report(
            self.policy,
            self.schema,
            self.profile,
            probe_gate._file_sha256(probe_gate.PROFILE_PATH),
        )
        self.assertEqual(report["status"], "PASS_APPARMOR_USERNS_PROBE_STATIC_RECORD")
        for key in (
            "profile_parser_invoked",
            "profile_loaded",
            "profile_selected_for_execution",
            "namespace_attempted",
            "mount_attempted",
            "network_used",
            "source_checkout_created",
            "upstream_code_executed",
            "build_executed",
            "sudo_used",
            "system_configuration_changed",
        ):
            self.assertFalse(report[key], key)

    def test_gate_has_no_process_or_write_execution_surface(self) -> None:
        source = inspect.getsource(probe_gate)
        for forbidden in ("subprocess", "os.system", "Popen(", "write_text(", "mkdir("):
            self.assertNotIn(forbidden, source)

    def test_schema_is_closed_and_profile_body_is_exact(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(
            probe_gate._effective_profile_lines(self.profile),
            probe_gate.EXPECTED_EFFECTIVE_PROFILE_LINES,
        )
        self.assertEqual(self.policy["observed_result"]["loaded_profile_match_count_after_removal"], 0)
        self.assertEqual(
            self.policy["observed_result"]["bwrap_or_aa_exec_process_residue_count_after_removal"],
            0,
        )


if __name__ == "__main__":
    unittest.main()
