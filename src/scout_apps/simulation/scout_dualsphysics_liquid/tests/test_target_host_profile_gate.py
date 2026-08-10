#!/usr/bin/env python3
"""No-ROS tests for the read-only MSI target-host profile gate."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_target_profile_gate as profile_gate  # noqa: E402


class TargetHostProfileStaticTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = json.loads(profile_gate.PROFILE_PATH.read_text(encoding="utf-8"))

    def test_checked_in_profile_is_read_only_and_valid(self) -> None:
        self.assertEqual(profile_gate.validate_profile(self.profile), [])
        self.assertFalse(self.profile["execution_authorized"])
        self.assertEqual(self.profile["allowed_script_commands"], ["self-check"])

    def test_profile_rejects_an_execution_or_broad_root_change(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["execution_authorized"] = True
        changed["storage"]["approved_root"] = "/home/zrj"
        errors = profile_gate.validate_profile(changed)
        self.assertIn("profile field mismatch: execution_authorized", errors)
        self.assertIn("approved_root is too broad", errors)

    def test_profile_rejects_layout_escape(self) -> None:
        changed = copy.deepcopy(self.profile)
        changed["storage"]["required_directories"].append("../escape")
        self.assertTrue(any("unsafe layout relative path" in error for error in profile_gate.validate_profile(changed)))

    def test_symlink_component_is_detected(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r8-liquid-profile-") as temporary:
            base = Path(temporary)
            real = base / "real"
            real.mkdir()
            link = base / "link"
            link.symlink_to(real, target_is_directory=True)
            self.assertTrue(profile_gate._has_symlink_component(link / "child"))


if __name__ == "__main__":
    unittest.main()
