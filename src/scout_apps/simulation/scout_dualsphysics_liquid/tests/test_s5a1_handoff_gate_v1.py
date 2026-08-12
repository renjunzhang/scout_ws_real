#!/usr/bin/env python3
"""Mock/static-only tests for the S5A1 ABI-v3 package gate."""

from __future__ import annotations

import copy
import hashlib
import json
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_s5a1_handoff_gate_v1 as gate  # noqa: E402


class S5A1HandoffGateV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy, cls.policy_identity = gate.validate_policy()
        cls.schema, _ = gate.read_json(gate.SCHEMA_PATH)

    def inputs(self):
        receipt, records, clock, tf = gate.mock_inputs(self.policy)
        return receipt, records, clock, tf

    def build(self):
        receipt, records, clock, tf = self.inputs()
        return gate.build_package(
            policy=self.policy,
            policy_identity=self.policy_identity,
            s5a0_receipt_raw=receipt,
            s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
            odom_records=records,
            clock_alignment=clock,
            tf_alignment=tf,
        )

    def test_policy_freezes_exact_primary_not_corpus_or_optional_identity(self) -> None:
        source = self.policy["source"]
        self.assertEqual(source["bag_size_bytes"], 13996902)
        self.assertEqual(source["bag_sha256"], "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07")
        self.assertNotEqual(source["bag_sha256"], "7db18f7a0771c31e84f6dbefab571304241ef99438f4eb6891b4cddebf01f567")
        self.assertEqual(source["s5a0_receipt_identity"], "NOT_MATERIALIZED")
        self.assertEqual(source["source_provenance"], "PARTIAL_BAG_ONLY")
        self.assertEqual(self.policy["claim_ceiling"]["admission"], "NOT_ADMITTED_S5A0_RECEIPT_REQUIRED")
        serialized = json.dumps(self.policy, sort_keys=True)
        self.assertNotIn("Bslosh", serialized)
        self.assertFalse(source["optional_pair_read_or_admitted"])
        self.assertFalse(source["c2_read_or_admitted"])

    def test_schema_is_valid_and_every_object_definition_is_closed(self) -> None:
        Draft202012Validator.check_schema(self.schema)
        gate.assert_schema_deep_closed(self.schema)
        self.assertIs(self.schema["additionalProperties"], False)
        for name, definition in self.schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_all_frozen_parent_hashes_are_reverified(self) -> None:
        policy, identity = gate.validate_policy()
        self.assertEqual(policy["parents"]["motion_bridge"]["sha256"], "c840f7bb467d5145f55d2e9904b7b8b62781658818ae47f3e95bd139904e2aa1")
        self.assertEqual(policy["parents"]["compatibility_contract"]["sha256"], "f0786be1f7054fcbc29a9eb0d7f611c2bd3bdee8e9dbe4f022962713e70b6e73")
        self.assertEqual(identity["sha256"], hashlib.sha256(gate.POLICY_PATH.read_bytes()).hexdigest())

    def test_mock_package_has_exact_inventory_and_closed_manifest(self) -> None:
        package = self.build()
        self.assertEqual(tuple(package), gate.PACKAGE_INVENTORY)
        gate.validate_package_bytes(package, self.schema)
        manifest = json.loads(package["transfer_manifest.json"])
        Draft202012Validator(self.schema).validate(manifest)
        self.assertEqual(manifest["status"], "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED")
        self.assertFalse(manifest["claim_ceiling"]["gpu_replay_authorized"])
        self.assertEqual(manifest["parents"]["s5a0_receipt"]["path"], "/mock/s5a0_primary_receipt.json")
        self.assertRegex(manifest["parents"]["s5a0_receipt"]["sha256"], r"^[0-9a-f]{64}$")

    def test_csv_shapes_identity_slerp_zyx_and_tail_are_frozen(self) -> None:
        package = self.build()
        motion_lines = package["motion.csv"].decode().splitlines()
        self.assertEqual(motion_lines[0].split(","), list(gate.MOTION_FIELDS))
        self.assertEqual(len(motion_lines[1].split(",")), 8)
        first_motion = [float(value) for value in motion_lines[1].split(",")]
        self.assertEqual(first_motion, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        solver_lines = package["solver_path.csv"].decode().splitlines()
        self.assertTrue(solver_lines[0].startswith("# "))
        self.assertTrue(all(len(line.split(",")) == 7 for line in solver_lines[1:]))
        first_solver = [float(value) for value in solver_lines[1].split(",")]
        self.assertEqual(first_solver, [0.0] * 7)
        final_recorded = [float(value) for value in solver_lines[-2].split(",")]
        final_tail = [float(value) for value in solver_lines[-1].split(",")]
        self.assertAlmostEqual(final_tail[0] - final_recorded[0], 1.0)
        self.assertEqual(final_tail[1:], final_recorded[1:])
        qc = json.loads(package["solver_path_qc.json"])
        self.assertTrue(qc["pass"])
        self.assertEqual(qc["query_grid"], "EVERY_PATH_INTERVAL_QUARTERS")
        self.assertEqual(qc["subdivisions_per_interval"], 4)
        self.assertEqual(qc["query_count"], 4 * (len(motion_lines) - 2) + 1)
        self.assertLessEqual(qc["max_position_error_m"], 1e-9)
        self.assertLessEqual(qc["max_quaternion_angular_distance_rad"], 1e-7)
        self.assertLessEqual(qc["max_rotation_matrix_angular_distance_rad"], 1e-7)

    def test_odom_raw_only_uses_header_pose_for_motion(self) -> None:
        package = self.build()
        topic = json.loads(package["topic_contract.json"])
        self.assertEqual(set(topic["motion_inputs"]), {
            "/odom.header.stamp", "/odom.pose.pose.position", "/odom.pose.pose.orientation"
        })
        self.assertFalse(topic["forbidden_signal_consumed"])
        self.assertEqual(topic["qc_only_witness_counts"], {})
        source = json.loads(package["source_outcome.json"])
        self.assertEqual(source["source_outcome"], "UNKNOWN")
        self.assertFalse(source["outcome_recomputed_from_topics"])

    def test_each_forbidden_qc_conflict_fails_without_changing_candidate_motion(self) -> None:
        receipt, records, clock, tf = self.inputs()
        baseline = self.build()
        expected = hashlib.sha256(baseline["motion.csv"]).hexdigest()
        for topic in self.policy["signals"]["forbidden_motion_inputs"]:
            with self.subTest(topic=topic):
                with self.assertRaises(gate.ForbiddenSignalConflict) as context:
                    gate.build_package(
                        policy=self.policy, policy_identity=self.policy_identity,
                        s5a0_receipt_raw=receipt,
                        s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                        odom_records=records, clock_alignment=clock, tf_alignment=tf,
                        qc_conflict_topics=(topic,),
                    )
                self.assertEqual(context.exception.candidate_motion_sha256, expected)

    def test_qc_only_topic_counts_are_witness_only_and_closed(self) -> None:
        receipt, records, clock, tf = self.inputs()
        package = gate.build_package(
            policy=self.policy, policy_identity=self.policy_identity,
            s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0.json",
            odom_records=records, clock_alignment=clock, tf_alignment=tf,
            qc_only_witness_counts={"/cmd_vel": 7, "/spmpc/status": 2},
        )
        topic = json.loads(package["topic_contract.json"])
        self.assertEqual(topic["qc_only_witness_counts"], {"/cmd_vel": 7, "/spmpc/status": 2})
        self.assertFalse(topic["forbidden_signal_consumed"])
        for bad in ({"/unknown": 1}, {"/cmd_vel": -1}, {"/cmd_vel": True}):
            with self.subTest(bad=bad), self.assertRaises(gate.HandoffError):
                gate.build_package(
                    policy=self.policy, policy_identity=self.policy_identity,
                    s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0.json",
                    odom_records=records, clock_alignment=clock, tf_alignment=tf,
                    qc_only_witness_counts=bad,
                )

    def test_receipt_must_be_real_bound_identity_at_export_and_pass_safety(self) -> None:
        receipt_raw, records, clock, tf = self.inputs()
        receipt = json.loads(receipt_raw)
        mutations = (
            lambda value: value.__setitem__("status", "FAIL"),
            lambda value: value["source_bag"].__setitem__("sha256", "0" * 64),
            lambda value: value.__setitem__("source_bag_executed", True),
            lambda value: value.__setitem__("files_written_under_source_root", 1),
            lambda value: value["topic_contract"].__setitem__("conflicts", ["x"]),
        )
        for mutate in mutations:
            changed = copy.deepcopy(receipt)
            mutate(changed)
            with self.subTest(changed=changed):
                with self.assertRaises(gate.HandoffError):
                    gate.build_package(
                        policy=self.policy, policy_identity=self.policy_identity,
                        s5a0_receipt_raw=gate.canonical_json(changed),
                        s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                        odom_records=records, clock_alignment=clock, tf_alignment=tf,
                    )
        with self.assertRaises(gate.HandoffError):
            gate.validate_s5a0_receipt(receipt_raw, "relative.json", self.policy)

    def test_duplicate_backtrack_gap_alignment_nonfinite_and_pre_roll_fail(self) -> None:
        receipt, records, clock, tf = self.inputs()
        cases = []
        duplicate = copy.deepcopy(records); duplicate[1]["odom_header_t_ns"] = duplicate[0]["odom_header_t_ns"]
        cases.append(duplicate)
        backtrack = copy.deepcopy(records); backtrack[1]["odom_header_t_ns"] = backtrack[0]["odom_header_t_ns"] - 1
        cases.append(backtrack)
        gap = copy.deepcopy(records); gap[1]["odom_header_t_ns"] = gap[0]["odom_header_t_ns"] + 300_000_000
        cases.append(gap)
        alignment = copy.deepcopy(records); alignment[0]["bag_record_t_ns"] += 600_000_000
        cases.append(alignment)
        nonfinite = copy.deepcopy(records); nonfinite[0]["x_m"] = float("nan")
        cases.append(nonfinite)
        for changed in cases:
            with self.subTest(kind=len(cases)):
                with self.assertRaises((gate.HandoffError, ValueError)):
                    gate.build_package(
                        policy=self.policy, policy_identity=self.policy_identity,
                        s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                        odom_records=changed, clock_alignment=clock, tf_alignment=tf,
                    )
        early = copy.deepcopy(records)
        early[1]["x_m"] = 0.01
        with self.assertRaisesRegex(gate.HandoffError, "pre-roll"):
            gate.build_package(
                policy=self.policy, policy_identity=self.policy_identity,
                s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                odom_records=early, clock_alignment=clock, tf_alignment=tf,
            )

    def test_clock_and_tf_are_alignment_witnesses_only_and_fail_closed(self) -> None:
        receipt, records, clock, tf = self.inputs()
        bad_clock = copy.deepcopy(clock)
        bad_clock[0]["nearest_clock_t_ns"] += 600_000_000
        bad_clock[0]["abs_delta_ns"] = 600_000_000
        with self.assertRaisesRegex(gate.HandoffError, "clock alignment"):
            gate.build_package(
                policy=self.policy, policy_identity=self.policy_identity,
                s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                odom_records=records, clock_alignment=bad_clock, tf_alignment=tf,
            )
        bad_tf = dict(tf)
        bad_tf["tf_used_as_motion"] = True
        with self.assertRaisesRegex(gate.HandoffError, "TF alignment"):
            gate.build_package(
                policy=self.policy, policy_identity=self.policy_identity,
                s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                odom_records=records, clock_alignment=clock, tf_alignment=bad_tf,
            )

    def test_schema_and_checksums_reject_manifest_or_artifact_tamper(self) -> None:
        package = self.build()
        changed = dict(package)
        manifest = json.loads(changed["transfer_manifest.json"])
        manifest["unexpected"] = True
        changed["transfer_manifest.json"] = gate.canonical_json(manifest)
        with self.assertRaises((ValidationError, gate.HandoffError)):
            gate.validate_package_bytes(changed, self.schema)
        changed = dict(package)
        changed["motion.csv"] += b"tamper\n"
        with self.assertRaises(gate.HandoffError):
            gate.validate_package_bytes(changed, self.schema)

    def test_create_new_writer_uses_0700_0600_and_refuses_collision(self) -> None:
        package = self.build()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "transfer.partial"
            gate.materialize_create_new(root, package)
            self.assertEqual(stat.S_IMODE(root.stat().st_mode), 0o700)
            self.assertEqual(sorted(path.name for path in root.iterdir()), sorted(gate.PACKAGE_INVENTORY))
            for path in root.iterdir():
                self.assertTrue(path.is_file())
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
                self.assertEqual(path.stat().st_nlink, 1)
            with self.assertRaises(FileExistsError):
                gate.materialize_create_new(root, package)

    def test_self_check_is_static_mock_only(self) -> None:
        report = gate.self_check()
        self.assertEqual(report["status"], "PASS_S5A1_HANDOFF_GATE_V1_SELF_CHECK")
        self.assertFalse(report["files_written"])
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["optional_bag_read"])
        self.assertFalse(report["solver_executed"])
        self.assertFalse(report["gpu_exposed"])
        self.assertFalse(report["sudo_used"])
        self.assertFalse(report["network_used"])


if __name__ == "__main__":
    unittest.main()
