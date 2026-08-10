#!/usr/bin/env python3
"""Mock-only tests for the MSI exact AppArmor mount-probe plan."""

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

import r8_liquid_target_apparmor_mount_probe_gate as mount_gate  # noqa: E402


class TargetAppArmorMountProbeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = json.loads(mount_gate.POLICY_PATH.read_text(encoding="utf-8"))
        self.schema = json.loads(mount_gate.SCHEMA_PATH.read_text(encoding="utf-8"))
        self.profile = mount_gate.PROFILE_PATH.read_text(encoding="utf-8")

    def test_checked_in_plan_allows_only_the_exact_rslave_transition(self) -> None:
        self.assertEqual(mount_gate.validate_policy(self.policy), [])
        self.assertEqual(mount_gate.validate_schema(self.schema), [])
        self.assertEqual(mount_gate.validate_profile_text(self.profile), [])
        self.assertFalse(self.policy["profile_loading_or_replay_authorized"])
        self.assertEqual(self.policy["allowed_gate_commands"], ["self-check"])
        self.assertEqual(
            self.policy["profile_contract"]["only_permitted_mount_rule"],
            "mount options=(rw, silent, rslave) -> /,",
        )
        self.assertEqual(self.policy["fixed_probe_contract"]["host_writable_mounts"], [])

    def test_profile_rejects_generic_bind_and_second_mount_rules(self) -> None:
        generic = self.profile.replace(
            "  mount options=(rw, silent, rslave) -> /,", "  mount,"
        )
        self.assertIn("profile has a forbidden permission token: mount,", mount_gate.validate_profile_text(generic))

        bind = self.profile.replace(
            "  mount options=(rw, silent, rslave) -> /,",
            "  mount options=(rw, silent, bind) -> /,",
        )
        self.assertIn("profile non-comment rules differ from the frozen mount probe body", mount_gate.validate_profile_text(bind))

        second_mount = self.profile.replace(
            "  mount options=(rw, silent, rslave) -> /,",
            "  mount options=(rw, silent, rslave) -> /,\n  mount options=(rw, silent, private) -> /,",
        )
        self.assertIn(
            "profile non-comment rules differ from the frozen mount probe body",
            mount_gate.validate_profile_text(second_mount),
        )

    def test_policy_rejects_replay_writable_bind_and_tool_drift(self) -> None:
        changed = copy.deepcopy(self.policy)
        changed["profile_loading_or_replay_authorized"] = True
        changed["fixed_probe_contract"]["host_writable_mounts"] = ["/home/zrj/scout_liquid_lab"]
        changed["trusted_system_tools"]["bwrap"]["sha256"] = "0" * 64
        errors = mount_gate.validate_policy(changed)
        self.assertIn("policy field mismatch: profile_loading_or_replay_authorized", errors)
        self.assertIn("fixed probe contract differs", errors)
        self.assertIn("trusted system tool identity differs", errors)

    def test_static_report_declares_no_new_host_or_execution_action(self) -> None:
        report = mount_gate.static_report(
            self.policy,
            self.schema,
            self.profile,
            mount_gate._file_sha256(mount_gate.PROFILE_PATH),
        )
        self.assertEqual(report["status"], "PASS_APPARMOR_MOUNT_PROBE_STATIC_PLAN")
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
        source = inspect.getsource(mount_gate)
        for forbidden in ("subprocess", "os.system", "Popen(", "write_text(", "mkdir("):
            self.assertNotIn(forbidden, source)

    def test_schema_is_closed_and_profile_body_is_exact(self) -> None:
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(
            mount_gate._effective_profile_lines(self.profile),
            mount_gate.EXPECTED_EFFECTIVE_PROFILE_LINES,
        )
        self.assertIn(
            "generic_mount",
            self.policy["profile_contract"]["forbidden_mount_operations"],
        )
        self.assertIn(
            "host_writable_bind",
            self.policy["profile_contract"]["forbidden_mount_operations"],
        )


if __name__ == "__main__":
    unittest.main()
