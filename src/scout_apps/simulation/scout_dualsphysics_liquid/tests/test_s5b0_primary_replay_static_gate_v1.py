#!/usr/bin/env python3
"""Pure static/mock tests for the create-new S5B0 v1 design contract."""

from __future__ import annotations

import ast
import copy
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s5b0_primary_replay_static_gate_v1 as gate  # noqa: E402


class S5B0PrimaryReplayStaticGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, cls.policy_sha = gate._read_json(gate.POLICY_PATH)
        cls.schema, cls.schema_sha = gate._read_json(gate.SCHEMA_PATH)
        cls.profile = Path(cls.policy["profile_contract"]["template_path"]).read_text(encoding="utf-8")

    @staticmethod
    def nominal_path() -> list[list[float]]:
        return [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [2.5, 0.4, 0.1, 0.0, 5.0, 0.0, 0.0],
            [5.0, 1.0, 0.2, 0.0, 10.0, 0.0, 0.0],
        ]

    def clone_manifest(self) -> dict[str, object]:
        files = []
        for item in self.policy["settled_clone_contract"]["required_files"]:
            files.append({
                "name": item["name"],
                "source_sha256": item["sha256"],
                "clone_sha256": item["sha256"],
                "mode": "0440",
                "nlink": 1,
            })
        return {
            "schema_version": "smpcc-r8-liquid-s5b0-settled-clone-manifest-v1",
            "materialized": True,
            "create_new": True,
            "source_unchanged": True,
            "fresh_root": True,
            "files": files,
        }

    def compatibility_receipt(self) -> dict[str, object]:
        return {
            "status": "PASS_S5_C1_TO_C1M_DEVELOPMENT_REPLAY_COMPATIBILITY_V1",
            "container": "C1",
            "c2_read_or_admitted": False,
            "optional_pair_read_or_admitted": False,
            "files_written": False,
            "solver_executed": False,
            "gpu_exposed": False,
            "contract_sha256": self.policy["frozen_parents"]["compatibility_contract"]["sha256"],
            "gate_sha256": self.policy["frozen_parents"]["compatibility_gate"]["sha256"],
        }

    def test_static_receipt_is_closed_and_explicitly_not_admitted(self) -> None:
        receipt = gate.self_check()
        self.assertEqual([], list(Draft202012Validator(self.schema).iter_errors(receipt)))
        self.assertEqual(receipt["status"], "NOT_ADMITTED_PARENT_TRANSFER_NOT_MATERIALIZED")
        self.assertEqual(receipt["parent_transfer"]["state"], "NOT_MATERIALIZED")
        self.assertFalse(receipt["admission"]["admitted"])
        self.assertEqual(receipt["selection"]["planned_denominator"], 1)
        self.assertFalse(receipt["safety"]["files_written"])

    def test_policy_rejects_optional_c2_and_denominator_promotion(self) -> None:
        for key, value in (("optional_authorized", True), ("c2_authorized", True), ("container", "C2"), ("planned_denominator", 2)):
            changed = copy.deepcopy(self.policy)
            changed["selection"][key] = value
            with self.subTest(key=key), self.assertRaisesRegex(gate.S5B0StaticError, "SELECTION_DRIFT"):
                gate.validate_policy(changed)

    def test_parent_hash_drift_fails_closed(self) -> None:
        parent = dict(self.policy["frozen_parents"]["gpu_candidate"])
        parent["sha256"] = "0" * 64
        with self.assertRaisesRegex(gate.S5B0StaticError, "PARENT_HASH_DRIFT"):
            gate.validate_parent_identity(self.policy, "gpu_candidate", parent)

    def test_c1_compatibility_receipt_rejects_c2_or_optional(self) -> None:
        gate.validate_compatibility_receipt(self.policy, self.compatibility_receipt())
        for key in ("c2_read_or_admitted", "optional_pair_read_or_admitted"):
            changed = self.compatibility_receipt()
            changed[key] = True
            with self.subTest(key=key), self.assertRaisesRegex(gate.S5B0StaticError, "C1_COMPATIBILITY_RECEIPT_INVALID"):
                gate.validate_compatibility_receipt(self.policy, changed)

    def test_settled_clone_requires_exact_part_and_head_hashes(self) -> None:
        manifest = self.clone_manifest()
        gate.validate_settled_clone_manifest(self.policy, manifest)
        missing_head = copy.deepcopy(manifest)
        missing_head["files"] = [missing_head["files"][0]]
        with self.assertRaisesRegex(gate.S5B0StaticError, "CLONE_REQUIRED_FILE_MISSING"):
            gate.validate_settled_clone_manifest(self.policy, missing_head)
        drifted = copy.deepcopy(manifest)
        drifted["files"][0]["clone_sha256"] = "f" * 64
        with self.assertRaisesRegex(gate.S5B0StaticError, "CLONE_HASH_DRIFT"):
            gate.validate_settled_clone_manifest(self.policy, drifted)

    def test_moving_domain_and_dry_mvpathfile_case_cover_full_tail(self) -> None:
        estimate = gate.estimate_moving_domain(self.policy, self.nominal_path())
        self.assertLess(estimate["estimated_gpu_bytes"], self.policy["moving_domain_budget"]["maximum_gpu_bytes"])
        plan = gate.generate_dry_case_plan(self.policy, self.nominal_path(), solver_tail_s=1.0)
        self.assertEqual(plan["mode"], "DRY_STATIC_ONLY")
        self.assertFalse(plan["files_written"])
        self.assertEqual(len(plan["motion_blocks"]), 2)
        self.assertEqual(plan["motion_blocks"][0], plan["motion_blocks"][1])
        self.assertIn('fieldtime="0" fieldx="1" fieldy="2" fieldz="3" fieldang1="4" fieldang2="5" fieldang3="6"', plan["motion_blocks"][0])
        self.assertIn('<axes value="ZYX" />', plan["motion_blocks"][0])
        self.assertIn('<mvnull id="2" />', plan["motion_blocks"][0])
        self.assertEqual(plan["gauge_contract"], "GAUGE_ZSURF_ALL_SLOTS_REQUIRED")
        self.assertEqual(plan["boundary_qc_contract"], "EXECUTED_BOUNDARY_MOTION_REQUIRED")
        self.assertIn("result_manifest.json", plan["result_inventory"])

    def test_nine_metre_swept_envelope_and_long_disk_budget_fail(self) -> None:
        nine_metres = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 9.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        with self.assertRaisesRegex(gate.S5B0StaticError, "MOVING_DOMAIN_AXIS_SPAN"):
            gate.estimate_moving_domain(self.policy, nine_metres)
        long_motion = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [120.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        with self.assertRaisesRegex(gate.S5B0StaticError, "OUTPUT_BUDGET"):
            gate.estimate_moving_domain(self.policy, long_motion)

    def test_first_identity_strict_time_and_tail_truncation_fail(self) -> None:
        changed = self.nominal_path()
        changed[0][1] = 0.01
        with self.assertRaisesRegex(gate.S5B0StaticError, "SOLVER_PATH_INITIAL_NOT_IDENTITY"):
            gate.validate_solver_path(changed)
        changed = self.nominal_path()
        changed[2][0] = changed[1][0]
        with self.assertRaisesRegex(gate.S5B0StaticError, "SOLVER_PATH_TIME_NOT_STRICT"):
            gate.validate_solver_path(changed)
        with self.assertRaisesRegex(gate.S5B0StaticError, "SOLVER_TAIL_TRUNCATED"):
            gate.generate_dry_case_plan(self.policy, self.nominal_path(), solver_tail_s=0.5)

    def test_profile_permission_drift_fails_static_query(self) -> None:
        result = gate.query_profile_template(self.policy, self.profile)
        self.assertEqual(result["state"], "NOT_INSTANTIATED_NOT_LOADED")
        drifted = self.profile.replace("  /case/solver_path.csv r,", "  /case/solver_path.csv rw,")
        with self.assertRaisesRegex(gate.S5B0StaticError, "PROFILE_PERMISSION_DRIFT"):
            gate.query_profile_template(self.policy, drifted)

    def test_schema_rejects_admission_or_runtime_claim_promotion(self) -> None:
        receipt = gate.self_check()
        for mutation in (
            lambda value: value["admission"].__setitem__("admitted", True),
            lambda value: value["safety"].__setitem__("gpu_exposed", True),
            lambda value: value["selection"].__setitem__("planned_denominator", 2),
            lambda value: value.update({"unknown": True}),
        ):
            changed = copy.deepcopy(receipt)
            mutation(changed)
            self.assertTrue(list(Draft202012Validator(self.schema).iter_errors(changed)))

    def test_gate_has_no_execution_or_write_interface(self) -> None:
        tree = ast.parse(Path(gate.__file__).read_text(encoding="utf-8"))
        imported = set()
        calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call):
                if isinstance(node.func, ast.Attribute):
                    calls.append(node.func.attr)
                elif isinstance(node.func, ast.Name):
                    calls.append(node.func.id)
        self.assertFalse(imported & {"subprocess", "socket", "os", "ctypes", "rospy", "rosbag", "rclpy"})
        self.assertFalse(set(calls) & {"run", "Popen", "system", "execv", "execve", "write_text", "write_bytes", "mkdir", "open"})


if __name__ == "__main__":
    unittest.main()
