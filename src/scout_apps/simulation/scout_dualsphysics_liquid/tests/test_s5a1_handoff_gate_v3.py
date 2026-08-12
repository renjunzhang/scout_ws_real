#!/usr/bin/env python3
"""Synthetic-only semantic tamper tests for S5A1 handoff gate v3."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Mapping


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v3 as gate  # noqa: E402


class S5A1HandoffGateV3Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        package, parents, policy_path = gate._synthetic_fixture()
        cls.baseline = package
        cls.parents = parents
        cls.policy_path = policy_path

    def package(self) -> dict[str, bytes]:
        return dict(self.baseline)

    def reseal(
        self,
        package: Mapping[str, bytes],
        manifest: Mapping[str, Any] | None = None,
    ) -> dict[str, bytes]:
        complete = dict(package)
        selected_manifest = (
            copy.deepcopy(dict(manifest))
            if manifest is not None
            else json.loads(complete["transfer_manifest.json"])
        )
        selected_manifest["package"]["artifacts"] = [
            {
                "name": name,
                "size_bytes": len(complete[name]),
                "sha256": gate.v1.sha256_bytes(complete[name]),
            }
            for name in gate.v1.DATA_ARTIFACTS
        ]
        complete["transfer_manifest.json"] = gate.v1.canonical_json(
            selected_manifest
        )
        complete["checksums.sha256"] = "".join(
            f"{gate.v1.sha256_bytes(complete[name])}  {name}\n"
            for name in gate.v1.CHECKSUMMED_FILES
        ).encode("ascii")
        return complete

    def mutate_json(
        self,
        package: Mapping[str, bytes],
        name: str,
        mutation: Callable[[dict[str, Any]], None],
    ) -> dict[str, bytes]:
        changed = dict(package)
        value = json.loads(changed[name])
        mutation(value)
        changed[name] = gate.v1.canonical_json(value)
        return self.reseal(changed)

    def assert_semantic_rejects(
        self,
        package: Mapping[str, bytes],
        *,
        parents: Mapping[str, bytes] | None = None,
        base_v1_passes: bool = True,
    ) -> None:
        if base_v1_passes:
            gate.v1.validate_package_bytes(package)
        with self.assertRaises((gate.HandoffError, ValueError)):
            gate.validate_package_bytes(
                package,
                parent_bytes=parents if parents is not None else self.parents,
                trusted_policy_path=self.policy_path,
            )

    def motion_rows(self, package: Mapping[str, bytes]) -> tuple[Any, ...]:
        rows = gate.v1._csv_rows(package["motion.csv"], "motion.csv")
        return tuple(
            gate.bridge.MotionSample(*(float(value) for value in row))
            for row in rows[1:]
        )

    def test_synthetic_baseline_and_self_check_are_safe(self) -> None:
        result = gate.validate_package_bytes(
            self.baseline,
            parent_bytes=self.parents,
            trusted_policy_path=self.policy_path,
        )
        self.assertEqual(
            result["status"], "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS"
        )
        self.assertEqual(result["odom_row_count"], 23)
        self.assertEqual(result["motion_row_count"], 23)
        self.assertEqual(result["solver_row_count"], 24)
        self.assertFalse(result["source_bag_read"])
        self.assertFalse(result["external_write"])

        report = gate.self_check()
        self.assertEqual(report["status"], "PASS_S5A1_HANDOFF_GATE_V3_SELF_CHECK")
        self.assertTrue(report["synthetic_fixture"])
        for flag in (
            "real_parent_read", "real_s5a0_receipt_read", "real_bag_read",
            "optional_bag_read", "files_written", "solver_executed",
            "gpu_exposed", "sudo_used", "network_used",
        ):
            self.assertFalse(report[flag], flag)

    def test_unresealed_artifact_fails_existing_checksum_gate(self) -> None:
        changed = self.package()
        changed["motion.csv"] += b"tamper\n"
        self.assert_semantic_rejects(changed, base_v1_passes=False)

    def test_directory_api_validates_private_synthetic_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "handoff.partial"
            gate.v1.materialize_create_new(root, self.baseline)
            result = gate.validate_package_root(
                root,
                parent_bytes=self.parents,
                trusted_policy_path=self.policy_path,
            )
            self.assertEqual(
                result["status"],
                "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
            )
            self.assertEqual(result["package_root"], str(root))

    def test_parent_bytes_and_manifest_parent_selection_are_rehashed(self) -> None:
        wrong_parents = dict(self.parents)
        compatibility_path = json.loads(
            self.baseline["transfer_manifest.json"]
        )["parents"]["compatibility_contract"]["path"]
        wrong_parents[compatibility_path] += b"drift"
        self.assert_semantic_rejects(
            self.baseline, parents=wrong_parents, base_v1_passes=True
        )

        changed = self.package()
        manifest = json.loads(changed["transfer_manifest.json"])
        forged_policy_path = "/mock/attacker_selected_policy.json"
        manifest["parents"]["policy"] = {
            "path": forged_policy_path,
            "sha256": gate.v1.sha256_bytes(self.parents[self.policy_path]),
        }
        changed = self.reseal(changed, manifest)
        supplied = dict(self.parents)
        supplied[forged_policy_path] = self.parents[self.policy_path]
        self.assert_semantic_rejects(changed, parents=supplied)

        changed = self.package()
        manifest = json.loads(changed["transfer_manifest.json"])
        forged_path = "/mock/forged_compatibility.json"
        forged_raw = b'{"forged":true}\n'
        forged_digest = hashlib.sha256(forged_raw).hexdigest()
        manifest["parents"]["compatibility_contract"] = {
            "path": forged_path, "sha256": forged_digest,
        }
        geometry = json.loads(changed["container_geometry_manifest.json"])
        geometry["compatibility_contract_sha256"] = forged_digest
        changed["container_geometry_manifest.json"] = gate.v1.canonical_json(geometry)
        changed = self.reseal(changed, manifest)
        supplied = dict(self.parents)
        supplied[forged_path] = forged_raw
        self.assert_semantic_rejects(changed, parents=supplied)

    def test_topic_allow_and_deny_lists_must_match_trusted_policy_exactly(self) -> None:
        for field in ("qc_only", "forbidden_motion_inputs"):
            with self.subTest(field=field):
                changed = self.mutate_json(
                    self.package(), "topic_contract.json",
                    lambda value, selected=field:
                        value[selected].pop(),
                )
                self.assert_semantic_rejects(changed)

    def test_interpolation_and_time_grid_evidence_is_recomputed(self) -> None:
        interpolation_mutations = (
            lambda value: value.__setitem__("smoothing", True),
            lambda value: value.__setitem__("pass", False),
            lambda value: value.__setitem__("grid_period_s", 0.06),
        )
        for mutation in interpolation_mutations:
            with self.subTest(kind="interpolation"):
                changed = self.mutate_json(
                    self.package(), "interpolation_qc.json", mutation
                )
                self.assert_semantic_rejects(changed)

        changed = self.mutate_json(
            self.package(), "time_window.json",
            lambda value: value.__setitem__("interpolation_period_s", 0.06),
        )
        self.assert_semantic_rejects(changed)

        changed = self.package()
        time_window = json.loads(changed["time_window.json"])
        manifest = json.loads(changed["transfer_manifest.json"])
        forged_first = time_window["first_effective_t_s"] + 0.05
        time_window["first_effective_t_s"] = forged_first
        manifest["window"]["first_effective_t_s"] = forged_first
        changed["time_window.json"] = gate.v1.canonical_json(time_window)
        changed = self.reseal(changed, manifest)
        self.assert_semantic_rejects(changed)

    def test_source_tf_and_frame_fields_cannot_drift(self) -> None:
        changed = self.mutate_json(
            self.package(), "source_bag_ref.json",
            lambda value: value.__setitem__(
                "relative_path", "forged/capture.bag"
            ),
        )
        self.assert_semantic_rejects(changed)

        changed = self.mutate_json(
            self.package(), "tf_alignment.json",
            lambda value: value.__setitem__("odom_frame", "forged_odom"),
        )
        self.assert_semantic_rejects(changed)

        changed = self.mutate_json(
            self.package(), "frame_contract.json",
            lambda value: value.__setitem__("container_frame", "forged_container"),
        )
        self.assert_semantic_rejects(changed, base_v1_passes=False)

    def test_odom_pose_change_cannot_keep_stale_motion(self) -> None:
        changed = self.package()
        rows = gate.v1._csv_rows(changed["odom_raw.csv"], "odom_raw.csv")
        x_index = list(gate.v1.ODOM_FIELDS).index("x_m")
        rows[-1][x_index] = format(float(rows[-1][x_index]) + 0.25, ".17g")
        changed["odom_raw.csv"] = gate.v1.render_csv(rows[0], rows[1:])
        changed = self.reseal(changed)
        self.assert_semantic_rejects(changed)

    def test_clock_rows_must_bind_corresponding_odom_headers(self) -> None:
        changed = self.package()
        rows = gate.v1._csv_rows(
            changed["clock_alignment.csv"], "clock_alignment.csv"
        )
        forged_header = int(rows[1][0]) + 10_000_000
        rows[1] = [str(forged_header), str(forged_header), "0"]
        changed["clock_alignment.csv"] = gate.v1.render_csv(rows[0], rows[1:])
        changed = self.reseal(changed)
        self.assert_semantic_rejects(changed)

    def test_solver_rows_are_bound_to_canonical_motion(self) -> None:
        changed = self.package()
        solver_rows = list(
            gate.bridge.parse_solver_path_csv(
                changed["solver_path.csv"].decode("utf-8")
            )
        )
        solver_rows[5] = replace(
            solver_rows[5], x_rel_m=solver_rows[5].x_rel_m + 1.0e-10
        )
        changed["solver_path.csv"] = gate.bridge.render_solver_path_csv(
            solver_rows
        ).encode("utf-8")

        motion = self.motion_rows(changed)
        solver_qc = json.loads(changed["solver_path_qc.json"])
        query_grid = gate.v1.complete_roundtrip_query_grid(
            motion, solver_qc["subdivisions_per_interval"]
        )
        report = gate.bridge.evaluate_solver_round_trip(
            motion, solver_rows[:-1], query_times_s=query_grid
        )
        solver_qc["max_position_error_m"] = report.max_position_error_m
        solver_qc["max_quaternion_angular_distance_rad"] = (
            report.max_quaternion_angular_distance_rad
        )
        solver_qc["max_rotation_matrix_angular_distance_rad"] = (
            report.max_rotation_matrix_angular_distance_rad
        )
        changed["solver_path_qc.json"] = gate.v1.canonical_json(solver_qc)
        manifest = json.loads(changed["transfer_manifest.json"])
        manifest["solver_path_qc"] = copy.deepcopy(solver_qc)
        changed = self.reseal(changed, manifest)
        self.assertLessEqual(report.max_position_error_m, 1.0e-9)
        self.assert_semantic_rejects(changed)

    def test_motion_manifest_metrics_are_recomputed(self) -> None:
        changed = self.package()
        motion_manifest = json.loads(changed["motion_manifest.json"])
        motion_manifest["bbox_min_m"][0] -= 1.0
        changed["motion_manifest.json"] = gate.v1.canonical_json(motion_manifest)
        manifest = json.loads(changed["transfer_manifest.json"])
        manifest["motion_qc"]["bbox_min_m"] = motion_manifest["bbox_min_m"]
        changed = self.reseal(changed, manifest)
        self.assert_semantic_rejects(changed)

    def test_solver_qc_row_count_is_recomputed_from_actual_csv(self) -> None:
        changed = self.package()
        solver_qc = json.loads(changed["solver_path_qc.json"])
        solver_qc["row_count"] = 999
        changed["solver_path_qc.json"] = gate.v1.canonical_json(solver_qc)
        manifest = json.loads(changed["transfer_manifest.json"])
        manifest["solver_path_qc"]["row_count"] = 999
        changed = self.reseal(changed, manifest)
        self.assert_semantic_rejects(changed)

    def test_geometry_mount_and_fluid_are_exact_policy_projections(self) -> None:
        cases = (
            (
                "container_geometry_manifest.json",
                lambda value: value.__setitem__(
                    "radius_m", value["radius_m"] + 0.001
                ),
            ),
            (
                "container_mount_manifest.json",
                lambda value: value["translation_m"].__setitem__(0, 0.01),
            ),
            (
                "fluid_spec.json",
                lambda value: value.__setitem__("density_kg_m3", 999.0),
            ),
        )
        for name, mutation in cases:
            with self.subTest(name=name):
                changed = self.mutate_json(self.package(), name, mutation)
                self.assert_semantic_rejects(changed)

    def test_run_spec_is_an_exact_solver_and_tail_projection(self) -> None:
        mutations = (
            lambda value: value.__setitem__("anglesunits", "radians"),
            lambda value: value.__setitem__("solver_path", "forged.csv"),
            lambda value: value.__setitem__(
                "recorded_motion_end_s",
                value["recorded_motion_end_s"] + 1.0,
            ),
            lambda value: value.__setitem__("tail_mode", "FORGED_HOLD"),
        )
        for mutation in mutations:
            with self.subTest(kind="run_spec"):
                changed = self.mutate_json(
                    self.package(), "run_spec.json", mutation
                )
                self.assert_semantic_rejects(changed)


if __name__ == "__main__":
    unittest.main()
