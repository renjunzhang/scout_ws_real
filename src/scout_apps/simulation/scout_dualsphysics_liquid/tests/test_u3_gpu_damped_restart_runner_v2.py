#!/usr/bin/env python3
"""Focused regression tests for the v2 exact damping-scheme inventory."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = PACKAGE_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v2 as runner
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_damped_restart_20260811T035641Z_v2.json"


def output_tree(root: Path, first: int, last: int, *, damping_scheme: bool) -> dict[str, object]:
    root.mkdir()
    data = root / "data"
    data.mkdir()
    for name in legacy.ROOT_OUTPUT_NAMES - {"data"}:
        (root / name).write_bytes(b"x")
    if damping_scheme:
        (root / "CfgDamping_Scheme.vtk").write_bytes(b"damping")
    for name in legacy.STATIC_DATA_NAMES:
        (data / name).write_bytes(b"x")
    for index in range(first, last + 1):
        (data / f"Part_{index:04d}.bi4").write_bytes(b"x")
    return {
        "part_first": first,
        "part_last": last,
        "part_count": last - first + 1,
        "maximum_output_bytes": 16 * 1024 * 1024,
    }


class DampedRestartRunnerV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        runner.configure()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))

    def test_v2_policy_and_parent_identities_load(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        self.assertEqual(loaded["parents"]["runner"]["path"], str(runner.SCRIPT_PATH))
        self.assertEqual(base.validate_xml_delta(loaded)["damped_zone_count"], 1)

    def test_initialization_requires_exact_damping_scheme_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "output"
            expected = output_tree(root, 0, 201, damping_scheme=True)
            inventory = runner.safe_output_inventory_v2(root, expected)
            self.assertIn("CfgDamping_Scheme.vtk", inventory["files"])
            (root / "CfgDamping_Scheme.vtk").unlink()
            with self.assertRaises(legacy.Stage4RunError):
                runner.safe_output_inventory_v2(root, expected)

    def test_undamped_tail_rejects_damping_scheme_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "output"
            expected = output_tree(root, 201, 401, damping_scheme=False)
            runner.safe_output_inventory_v2(root, expected)
            (root / "CfgDamping_Scheme.vtk").write_bytes(b"forbidden")
            with self.assertRaises(legacy.Stage4RunError):
                runner.safe_output_inventory_v2(root, expected)

    def test_v2_keeps_exact_solver_argv_and_thresholds(self) -> None:
        loaded = base.load_policy(POLICY_PATH)
        for phase_name in base.PHASE_KEYS:
            self.assertEqual(loaded["phases"][phase_name]["solver_argv"], base._exact_solver_argv(phase_name))
        candidate = copy.deepcopy(loaded)
        candidate["acceptance"]["metric_limits"]["speed_rms_m_s"] = 0.01
        with self.assertRaises(base.DampedRestartRunError):
            base.semantic_validate(candidate, POLICY_PATH)


if __name__ == "__main__":
    unittest.main()
