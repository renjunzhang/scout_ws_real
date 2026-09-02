#!/usr/bin/env python3
"""Contract tests for the development-only Stage 1 evidence gate."""

from __future__ import annotations

import copy
import json
import math
import shutil
import sys
import tempfile
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[3]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline import stage1_evidence
from acados.mainline.contract_source import (
    STAGE0_CONTRACT_SHA256,
    STAGE3_PROHIBITED_STATUS,
)
from acados.mainline.stage1_evidence import (
    REQUIRED_MISSING_AUTHORITIES,
    SOURCE_DOCUMENT_ROLES,
    STAGE1_BLOCKED_STATUS,
    STAGE1_EVIDENCE_SHA256,
    STAGE1_EVIDENCE_STATUS,
    Stage1EvidenceError,
    load_stage1_development_evidence,
)

EVIDENCE = (
    PACKAGE_ROOT
    / "config"
    / "mainline"
    / "contracts"
    / "stage1_development_evidence_v1.json"
)


class MainlineStage1DevelopmentEvidenceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.document = json.loads(EVIDENCE.read_text(encoding="utf-8"))

    def validate(self, value: dict) -> None:
        stage1_evidence._validate_stage1_development_evidence(
            value,
            WORKSPACE_ROOT,
            verify_repository_files=False,
        )

    def copy_authority_files(self, destination_root: Path) -> None:
        authority = self.document["authority"]
        records = [authority["stage0_contract"], *authority["source_documents"]]
        for record in records:
            source = WORKSPACE_ROOT / record["path"]
            destination = destination_root / record["path"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

    def test_repository_snapshot_is_pinned_and_stays_blocked(self) -> None:
        reference = load_stage1_development_evidence(EVIDENCE, WORKSPACE_ROOT)
        self.assertEqual(reference.evidence_sha256, STAGE1_EVIDENCE_SHA256)
        self.assertEqual(reference.status, STAGE1_EVIDENCE_STATUS)
        self.assertEqual(reference.stage0_contract_sha256, STAGE0_CONTRACT_SHA256)
        self.assertEqual(reference.stage1_status, STAGE1_BLOCKED_STATUS)
        self.assertEqual(reference.stage3_status, STAGE3_PROHIBITED_STATUS)
        self.assertFalse(reference.stage1_freeze_allowed)
        self.assertFalse(reference.stage3_codegen_allowed)
        self.assertEqual(
            reference.required_missing_authorities,
            REQUIRED_MISSING_AUTHORITIES,
        )
        self.assertEqual(len(reference.development_trials), 6)

    def test_reference_is_immutable_and_serializes_only_blocked_authority(self) -> None:
        reference = load_stage1_development_evidence(EVIDENCE, WORKSPACE_ROOT)
        with self.assertRaises(FrozenInstanceError):
            reference.status = "FROZEN"  # type: ignore[misc]
        with self.assertRaisesRegex(Stage1EvidenceError, "pinned loader"):
            replace(
                reference,
                _construction_token=object(),
                status="FROZEN",
            )
        value = reference.to_dict()
        self.assertFalse(value["stage1_freeze_allowed"])
        self.assertFalse(value["stage3_codegen_allowed"])
        self.assertNotIn("L_max_v_sec", value)
        self.assertNotIn("artifact_sha256", value)

    def test_source_documents_are_ordered_and_bound_to_exact_bytes(self) -> None:
        reference = load_stage1_development_evidence(EVIDENCE, WORKSPACE_ROOT)
        documents = self.document["authority"]["source_documents"]
        self.assertEqual(
            tuple(document["role"] for document in documents),
            SOURCE_DOCUMENT_ROLES,
        )
        self.assertEqual(
            reference.source_document_sha256,
            tuple(document["sha256"] for document in documents),
        )

    def test_all_known_bags_remain_unverified_development_evidence(self) -> None:
        partitions = self.document["dataset_partitions"]
        self.assertEqual(partitions["validation"], [])
        self.assertEqual(partitions["final_test"], [])
        self.assertEqual(len(partitions["development"]), 6)
        self.assertTrue(
            all(
                entry["availability"] == "EXTERNAL_BYTES_NOT_AVAILABLE_ON_THIS_HOST"
                for entry in partitions["development"]
            )
        )
        self.assertEqual(
            partitions["development"][2]["provenance_flags"],
            ["git_dirty=1"],
        )
        self.validate(self.document)

    def test_production_gate_or_status_cannot_be_promoted(self) -> None:
        mutations = (
            ("stage1_freeze_allowed", True),
            ("stage3_codegen_allowed", True),
            ("stage1_status", "FROZEN"),
            ("stage3_status", "GENERATED"),
            ("dataset_gate_status", "FROZEN"),
            ("fit_identifiability_status", "FROZEN"),
            ("lmax_status", "FROZEN"),
            ("execution_parameters_status", "FROZEN"),
        )
        for key, value in mutations:
            mutated = copy.deepcopy(self.document)
            mutated["gates"][key] = value
            with self.subTest(key=key), self.assertRaises(Stage1EvidenceError):
                self.validate(mutated)

    def test_validation_and_final_test_cannot_be_backfilled_from_development(
        self,
    ) -> None:
        for partition in ("validation", "final_test"):
            mutated = copy.deepcopy(self.document)
            mutated["dataset_partitions"][partition].append(
                copy.deepcopy(mutated["dataset_partitions"]["development"][0])
            )
            with (
                self.subTest(partition=partition),
                self.assertRaisesRegex(Stage1EvidenceError, "cannot populate"),
            ):
                self.validate(mutated)

    def test_dataset_identity_role_and_availability_fail_closed(self) -> None:
        mutations = []
        duplicate = copy.deepcopy(self.document)
        duplicate["dataset_partitions"]["development"].append(
            copy.deepcopy(duplicate["dataset_partitions"]["development"][0])
        )
        mutations.append(duplicate)

        unknown_kind = copy.deepcopy(self.document)
        unknown_kind["dataset_partitions"]["development"][0]["evidence_kind"] = (
            "DEDICATED_FINAL_TEST"
        )
        mutations.append(unknown_kind)

        false_availability = copy.deepcopy(self.document)
        false_availability["dataset_partitions"]["development"][0]["availability"] = (
            "VERIFIED"
        )
        mutations.append(false_availability)

        uppercase_hash = copy.deepcopy(self.document)
        uppercase_hash["dataset_partitions"]["development"][0]["bag_sha256"] = (
            uppercase_hash["dataset_partitions"]["development"][0]["bag_sha256"].upper()
        )
        mutations.append(uppercase_hash)

        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(Stage1EvidenceError):
                self.validate(mutated)

    def test_candidate_numbers_and_observed_ranges_fail_closed(self) -> None:
        mutations = []
        for channel, key, value in (
            ("linear", "descriptive_L_sec", True),
            ("linear", "tau_sec", -1.0),
            ("angular", "gain", math.inf),
            ("angular", "near_optimal_L_envelope_upper_sec", -0.1),
            ("angular", "near_optimal_L_envelope_lower_sec", 0.5),
            ("linear", "near_optimal_L_scope", "PER_TRIAL_INTERVAL"),
            ("angular", "fit_boundary_observation", "TAU_BOUNDARY"),
        ):
            mutated = copy.deepcopy(self.document)
            mutated["descriptive_actuator_candidate"][channel][key] = value
            mutations.append(mutated)
        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(Stage1EvidenceError):
                self.validate(mutated)

    def test_observer_and_nowcast_decisions_cannot_be_reversed(self) -> None:
        mutations = []
        selected_odom = copy.deepcopy(self.document)
        selected_odom["accepted_findings"]["observer_input"]["selected"] = "ODOM_O0"
        mutations.append(selected_odom)

        full_truth = copy.deepcopy(self.document)
        full_truth["accepted_findings"]["observer_input"][
            "full_four_state_truth_claimed"
        ] = True
        mutations.append(full_truth)

        i1_enabled = copy.deepcopy(self.document)
        i1_enabled["accepted_findings"]["extra_liquid_nowcast"]["I1"][
            "allowed_in_solver"
        ] = True
        mutations.append(i1_enabled)

        l22_enabled = copy.deepcopy(self.document)
        l22_enabled["accepted_findings"]["extra_liquid_nowcast"]["L22"][
            "allowed_in_solver"
        ] = True
        mutations.append(l22_enabled)

        baseline_promoted = copy.deepcopy(self.document)
        baseline_promoted["accepted_findings"]["tested_effective_window_baseline"][
            "allowed_as_new_mainline_fopdt_parameter"
        ] = True
        mutations.append(baseline_promoted)

        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(Stage1EvidenceError):
                self.validate(mutated)

    def test_source_document_byte_drift_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.copy_authority_files(root)
            reference = load_stage1_development_evidence(EVIDENCE, root)
            self.assertFalse(reference.stage1_freeze_allowed)

            source_path = (
                root / self.document["authority"]["source_documents"][0]["path"]
            )
            source_path.write_bytes(source_path.read_bytes() + b"\n")
            with self.assertRaisesRegex(Stage1EvidenceError, "source bytes"):
                load_stage1_development_evidence(EVIDENCE, root)

    def test_reencoded_stage0_cannot_supply_authority(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.copy_authority_files(root)
            stage0_path = root / self.document["authority"]["stage0_contract"]["path"]
            stage0_value = json.loads(stage0_path.read_text(encoding="utf-8"))
            stage0_path.write_text(
                json.dumps(stage0_value, ensure_ascii=False, sort_keys=True),
                encoding="utf-8",
            )
            with self.assertRaises(Stage1EvidenceError):
                load_stage1_development_evidence(EVIDENCE, root)

    def test_reencoded_or_duplicate_key_snapshot_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reencoded = root / "reencoded.json"
            reencoded.write_text(
                json.dumps(self.document, ensure_ascii=False, sort_keys=True),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(Stage1EvidenceError, "pinned immutable"):
                load_stage1_development_evidence(reencoded, WORKSPACE_ROOT)

            duplicate = root / "duplicate.json"
            duplicate.write_text('{"status":"a","status":"b"}', encoding="utf-8")
            with self.assertRaisesRegex(Stage1EvidenceError, "duplicate"):
                load_stage1_development_evidence(duplicate, WORKSPACE_ROOT)

    def test_unknown_fields_and_repository_escape_are_rejected(self) -> None:
        unknown = copy.deepcopy(self.document)
        unknown["artifact_sha256"] = "a" * 64
        with self.assertRaises(Stage1EvidenceError):
            self.validate(unknown)

        escaped = copy.deepcopy(self.document)
        escaped["authority"]["source_documents"][0]["path"] = "../outside.md"
        with self.assertRaisesRegex(Stage1EvidenceError, "inside the repository"):
            self.validate(escaped)


if __name__ == "__main__":
    unittest.main()
