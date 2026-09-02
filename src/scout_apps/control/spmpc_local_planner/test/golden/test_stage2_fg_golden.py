#!/usr/bin/env python3
"""Contract and freshness tests for the linked Stage 2f/2g golden."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE_ROOT / "test" / "golden"))

# isort: off
import generate_stage2_fg_header as generator
import stage2_fg_reference as reference
# isort: on


FIXTURE = (
    PACKAGE_ROOT / "test" / "fixtures" / "stage2_fg_execution_projection_golden_v1.json"
)


class Stage2FgGoldenTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = reference.load_fixture(FIXTURE)

    def assert_invalid_document(self, document: str, message: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(document, encoding="utf-8")
            with self.assertRaises(reference.FixtureError) as context:
                reference.load_fixture(path)
        self.assertIn(message, str(context.exception))

    def test_reference_recomputes_both_linked_case_arrays(self) -> None:
        reference.validate_expected(self.fixture)
        self.assertEqual(1, len(self.fixture["known_prefix_cases"]))
        self.assertEqual(1, len(self.fixture["nominal_commit_cases"]))
        prefix = self.fixture["known_prefix_cases"][0]
        projector = self.fixture["nominal_commit_cases"][0]
        self.assertEqual("KnownPrefixNonuniformPhysicalGolden", prefix["id"])
        self.assertEqual("NominalStraightPoseProjectionGolden", projector["id"])

    def test_projector_case_is_the_prefix_to_pose_chain_boundary(self) -> None:
        prefix = self.fixture["known_prefix_cases"][0]
        projector = self.fixture["nominal_commit_cases"][0]
        self.assertEqual(prefix["id"], projector["source_known_prefix_case_id"])
        self.assertNotIn("pose", projector)
        self.assertNotIn("clock", projector)
        self.assertNotIn("reset_epoch", projector)
        self.assertNotIn("target_cycle_id", projector)
        self.assertNotIn("expected_history_generation", projector)
        self.assertEqual(
            prefix["expected"]["history_generation"],
            projector["authority"]["history_generation"],
        )
        self.assertEqual(
            prefix["expected"]["last_emitted_cycle_id"],
            projector["authority"]["release_cycle_id"],
        )
        self.assertEqual(
            prefix["expected"]["history_generation"],
            projector["authority"]["release_generation"],
        )

    def test_receipt_jitter_does_not_change_planned_prefix_propagation(self) -> None:
        prefix = self.fixture["known_prefix_cases"][0]
        jittered = reference.calculate_known_prefix(prefix)
        zero_receipt = copy.deepcopy(prefix)
        for event in zero_receipt["history"]:
            event["actual_lateness_ns"] = 0
        self.assertEqual(jittered, reference.calculate_known_prefix(zero_receipt))

    def test_derived_dimensions_are_exposed_by_the_header(self) -> None:
        rendered = generator.render_header(self.fixture)
        self.assertIn("kHistoryCapacity = 16", rendered)
        self.assertIn("kLinearSelectorWidth = 4", rendered)
        self.assertIn("kAngularSelectorWidth = 4", rendered)
        self.assertIn("kLinearOlderCount = 2", rendered)
        self.assertIn("kAngularOlderCount = 2", rendered)
        self.assertIn("kSegmentCount = 6", rendered)
        self.assertIn("kVertexCapacity = 2", rendered)

    def test_authority_and_coverage_are_fail_closed(self) -> None:
        mutations = ("kind", "generation", "path", "gap")
        for mutation in mutations:
            value = copy.deepcopy(self.fixture)
            projector = value["nominal_commit_cases"][0]
            if mutation == "kind":
                projector["authority"]["kind"] = "frozen_start_interval"
            elif mutation == "generation":
                projector["authority"]["release_generation"] += 1
            elif mutation == "path":
                projector["authority"]["identity"]["path_id"] += 1
            else:
                value["known_prefix_cases"][0]["maximum_history_gap_ns"] -= 1
            with self.assertRaises(reference.FixtureError, msg=mutation):
                reference.validate_expected(value)

    def test_stage2e_complete_map_is_not_an_input_or_dependency(self) -> None:
        self.assertNotEqual(
            "stage2_execution_golden_v1", self.fixture["schema_version"]
        )
        self.assertNotIn("scenarios", self.fixture)
        self.assertNotIn("stage2_execution_golden_reference", reference.__dict__)

    def test_digest_is_embedded_in_the_generated_header(self) -> None:
        digest = reference.canonical_sha256(self.fixture)
        rendered = generator.render_header(self.fixture)
        self.assertIn(f'kCanonicalJsonSha256[] = "{digest}"', rendered)
        self.assertIn("namespace stage2_fg_golden {", rendered)
        self.assertEqual(64, len(digest))

    def test_generator_writes_and_checks_only_requested_build_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "build" / "generated" / "golden.hpp"
            generator.generate(FIXTURE, output)
            self.assertTrue(output.is_file())
            generated = output.read_text(encoding="utf-8")
            self.assertIn("kKnownPrefixCase0", generated)
            self.assertIn("kNominalProjectionCase0", generated)
            generator.generate(FIXTURE, output, check=True)
            output.write_text(generated + "// stale\n", encoding="utf-8")
            with self.assertRaises(reference.FixtureError):
                generator.generate(FIXTURE, output, check=True)

    def test_expected_drift_is_rejected_for_each_algorithm(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["known_prefix_cases"][0]["expected"]["segments"][0]["linear_target"] += (
            1.0
        )
        with self.assertRaises(reference.FixtureError):
            reference.validate_expected(value)

        value = copy.deepcopy(self.fixture)
        value["nominal_commit_cases"][0]["expected"]["s"] += 1.0
        with self.assertRaises(reference.FixtureError):
            reference.validate_expected(value)

    def test_duplicate_and_unknown_root_keys_are_rejected(self) -> None:
        self.assert_invalid_document(
            '{"schema_version":"stage2_fg_execution_projection_golden_v1",'
            '"schema_version":"stage2_fg_execution_projection_golden_v1",'
            '"numeric":{},"known_prefix_cases":[],"nominal_commit_cases":[]}',
            "duplicate JSON object key",
        )
        value = copy.deepcopy(self.fixture)
        value["unexpected"] = 1
        self.assert_invalid_document(json.dumps(value), "unknown fields")

    def test_nonfinite_json_constants_are_rejected(self) -> None:
        self.assert_invalid_document(
            '{"schema_version":"stage2_fg_execution_projection_golden_v1",'
            '"numeric":{"absolute_tolerance":NaN},'
            '"known_prefix_cases":[],"nominal_commit_cases":[]}',
            "non-finite JSON number",
        )

    def test_missing_root_fields_are_rejected(self) -> None:
        value = copy.deepcopy(self.fixture)
        del value["nominal_commit_cases"]
        self.assert_invalid_document(json.dumps(value), "missing fields")

    def test_unknown_source_case_is_rejected(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["nominal_commit_cases"][0]["source_known_prefix_case_id"] = "missing"
        self.assert_invalid_document(json.dumps(value), "unknown")


if __name__ == "__main__":
    unittest.main()
