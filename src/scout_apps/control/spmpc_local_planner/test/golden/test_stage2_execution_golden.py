#!/usr/bin/env python3
"""Contract tests for the Stage 2e JSON/reference/header boundary."""

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
import generate_stage2_execution_golden_header as generator
import stage2_execution_golden_reference as reference
# isort: on


FIXTURE = PACKAGE_ROOT / "test" / "fixtures" / "stage2_execution_golden_v1.json"


class Stage2ExecutionGoldenTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = reference.load_fixture(FIXTURE)
        self.scenario = self.fixture["scenarios"][0]

    def assert_invalid_document(self, document: str, message: str) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text(document, encoding="utf-8")
            with self.assertRaises(reference.FixtureError) as context:
                reference.load_fixture(path)
        self.assertIn(message, str(context.exception))

    def test_reference_recomputes_the_complete_map(self) -> None:
        reference.validate_expected(self.fixture)
        result = reference.calculate(self.scenario)
        self.assertEqual(set(result), {"issued", "segments", "next_state"})
        self.assertEqual(
            len(result["segments"]), len(self.scenario["expected"]["segments"])
        )

    def test_canonical_digest_is_stable_and_header_embeds_it(self) -> None:
        digest = reference.canonical_sha256(self.fixture)
        rendered = generator.render_header(self.fixture)
        self.assertIn(f'kCanonicalJsonSha256[] = "{digest}"', rendered)
        tolerance = self.fixture["numeric"]["absolute_tolerance"]
        self.assertIn(
            f"static constexpr double kAbsoluteTolerance = {tolerance!r};",
            rendered,
        )
        self.assertIn("namespace mainline {", rendered)
        self.assertEqual(64, len(digest))

    def test_generator_writes_only_requested_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated" / "golden.hpp"
            generator.generate(FIXTURE, output)
            self.assertTrue(output.is_file())
            generated = output.read_text(encoding="utf-8")
            self.assertIn("kExpectedNextState", generated)
            self.assertIn('kSchemaVersion[] = "stage2_execution_golden_v1"', generated)
            self.assertIn(
                'kScenarioId[] = "MatchesIndependentCompleteMapGolden"', generated
            )
            generator.generate(FIXTURE, output, check=True)
            output.write_text(generated + "// stale\n", encoding="utf-8")
            with self.assertRaises(reference.FixtureError):
                generator.generate(FIXTURE, output, check=True)

    def test_duplicate_keys_are_rejected(self) -> None:
        self.assert_invalid_document(
            '{"schema_version":"stage2_execution_golden_v1",'
            '"schema_version":"stage2_execution_golden_v1","scenarios":[]}',
            "duplicate JSON object key",
        )

    def test_unknown_fields_are_rejected(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["unexpected"] = 1
        self.assert_invalid_document(json.dumps(value), "unknown fields")

    def test_nonfinite_json_constants_are_rejected(self) -> None:
        self.assert_invalid_document(
            '{"schema_version":"stage2_execution_golden_v1",'
            '"scenarios":[{"id":"MatchesIndependentCompleteMapGolden",'
            '"config":NaN,"plant":{},"state":{},"control":{},"expected":{}}]}',
            "non-finite JSON number",
        )

    def test_fixed_array_lengths_are_rejected(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["scenarios"][0]["expected"]["segments"].pop()
        self.assert_invalid_document(
            json.dumps(value), "expected.segments must be an array of length 3"
        )

        value = copy.deepcopy(self.fixture)
        value["scenarios"][0]["state"]["linear_older"].append(9.0)
        config = self.scenario["config"]
        older_count = reference.delay_dimensions(
            config["maximum_linear_delay_sec"], config["dt_sec"]
        )["older_count"]
        self.assert_invalid_document(
            json.dumps(value),
            f"state.linear_older must be an array of length {older_count}",
        )

    def test_queue_width_is_derived_from_maximum_delay(self) -> None:
        config = self.scenario["config"]
        dimensions = reference.delay_dimensions(
            config["maximum_linear_delay_sec"], config["dt_sec"]
        )
        rendered = generator.render_header(self.fixture)
        self.assertIn(
            f"kLinearSelectorWidth = {dimensions['selector_width']};",
            rendered,
        )
        self.assertIn(f"kLinearOlderCount = {dimensions['older_count']};", rendered)

        value = copy.deepcopy(self.fixture)
        value["scenarios"][0]["config"]["maximum_linear_delay_sec"] += config["dt_sec"]
        self.assert_invalid_document(
            json.dumps(value),
            "state.linear_older must be an array of length",
        )

    def test_expected_drift_is_rejected(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["scenarios"][0]["expected"]["issued"]["linear_command"] += 1.0
        with self.assertRaises(reference.FixtureError):
            reference.validate_expected(value)

    def test_numeric_tolerance_is_explicit_and_positive(self) -> None:
        value = copy.deepcopy(self.fixture)
        value["numeric"]["absolute_tolerance"] = 0.0
        self.assert_invalid_document(
            json.dumps(value), "absolute_tolerance must be positive"
        )

        value = copy.deepcopy(self.fixture)
        del value["numeric"]
        self.assert_invalid_document(json.dumps(value), "missing fields: numeric")


if __name__ == "__main__":
    unittest.main()
