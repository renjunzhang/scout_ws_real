#!/usr/bin/env python3

import contextlib
import copy
import csv
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = (
    PACKAGE_ROOT / "tools" / "simulation" / "fit_phase_rejoin_recovery.py"
)

SPEC = importlib.util.spec_from_file_location(
    "fit_phase_rejoin_recovery", TOOL_PATH
)
FITTER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = FITTER
SPEC.loader.exec_module(FITTER)


class PhaseRejoinRecoveryFitTest(unittest.TestCase):

    @staticmethod
    def make_record(split, rollout_id, seed, phase_index, recovered, error):
        record = {
            "split": split,
            "rollout_id": rollout_id,
            "seed": str(seed),
            "phase_index": str(phase_index),
            "recovered": "1" if recovered else "0",
        }
        record.update(
            {name: format(error, ".17g") for name in FITTER.ERROR_COLUMNS}
        )
        return record

    @classmethod
    def make_rows(cls, held_out_unrecovered_error=0.08):
        rows = []

        def add_rollouts(split, label, count, recovered, error, seed_base):
            for rollout_index in range(count):
                rollout_id = "{}-{}-{}".format(split, label, rollout_index)
                seed = seed_base + rollout_index
                for phase_index in (0, 1):
                    rows.append(
                        cls.make_record(
                            split,
                            rollout_id,
                            seed,
                            phase_index,
                            recovered,
                            error,
                        )
                    )

        # Fit determines radii 0.3 for state errors and bounds 0.1 for
        # execution errors.  The 80 negative rollouts per phase make the
        # zero-false-accept 95% Wilson upper bound smaller than 0.05.
        add_rollouts("fit", "recovered", 2, True, 0.10, 1000)
        add_rollouts("tune", "recovered", 4, True, 0.04, 2000)
        add_rollouts("tune", "unrecovered", 80, False, 0.08, 2100)
        add_rollouts("held_out", "recovered", 4, True, 0.04, 3000)
        add_rollouts(
            "held_out",
            "unrecovered",
            80,
            False,
            held_out_unrecovered_error,
            3100,
        )
        return rows

    @staticmethod
    def write_rows(path, rows):
        with Path(path).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FITTER.INPUT_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

    @staticmethod
    def run_fit(input_path, output_path):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            return_code = FITTER.main(
                [
                    "fit",
                    "--input",
                    str(input_path),
                    "--out-dir",
                    str(output_path),
                    "--shrinkage-grid",
                    "1,0.5,0.25",
                ]
            )
        return return_code, stdout.getvalue(), stderr.getvalue()

    def assert_no_safety_claim(self, manifest, report):
        self.assertIs(manifest["safety_certificate"], False)
        self.assertIs(manifest["robust_invariant_set"], False)
        self.assertIs(manifest["formal_robot_release"], False)
        self.assertIs(manifest["physical_enforce_authorized"], False)
        self.assertIs(report["safety_certificate"], False)
        self.assertIs(report["formal_robot_release"], False)
        serialized = json.dumps({"manifest": manifest, "report": report})
        self.assertNotIn('"safety_certificate": true', serialized)

    def test_schema_and_successful_fit_tune_held_out(self):
        self.assertEqual(len(FITTER.STATE_ERROR_COLUMNS), 9)
        self.assertEqual(len(FITTER.EXECUTION_ERROR_COLUMNS), 14)
        self.assertEqual(FITTER.COMPILED_GATE_RADIUS_COUNT, 9)
        self.assertEqual(FITTER.COMPILED_EXECUTION_BOUND_COUNT, 14)
        self.assertEqual(
            FITTER.EXECUTION_ERROR_COLUMNS,
            (
                "linear_output",
                "angular_output",
                "linear_pending_0",
                "linear_pending_1",
                "linear_pending_2",
                "linear_pending_3",
                "linear_pending_4",
                "angular_pending_0",
                "angular_pending_1",
                "angular_pending_2",
                "angular_pending_3",
                "angular_pending_4",
                "angular_pending_5",
                "angular_pending_6",
            ),
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "recovery.csv"
            output_path = root / "evidence"
            self.write_rows(input_path, self.make_rows())

            return_code, stdout, stderr = self.run_fit(input_path, output_path)
            self.assertEqual(return_code, 0, stderr)
            self.assertIn("EMPIRICAL_HELD_OUT_PASS", stdout)
            self.assertEqual(
                {path.name for path in output_path.iterdir()},
                {
                    "phase_rejoin_recovery_radii_bounds.csv",
                    "held_out_report.json",
                    "manifest.json",
                    "manifest.sha256",
                },
            )

            manifest_path = output_path / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            report = json.loads(
                (output_path / "held_out_report.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["schema"], FITTER.MANIFEST_SCHEMA)
            self.assertEqual(manifest["input"]["schema"], FITTER.SCHEMA)
            self.assertEqual(
                manifest["outputs"]["scales"]["schema"], FITTER.SCALE_SCHEMA
            )
            self.assertEqual(report["schema"], FITTER.REPORT_SCHEMA)
            self.assertEqual(manifest["status"], "EMPIRICAL_HELD_OUT_PASS")
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(manifest["tune"]["selected_shrinkage"], 0.5)
            self.assertEqual(report["selected_shrinkage"], 0.5)
            self.assertEqual(report["held_out_evaluation_count"], 1)
            self.assertIs(report["held_out_influenced_fit"], False)
            self.assertIs(report["held_out_influenced_tuning"], False)
            global_metrics = report["evaluation"]["global"]
            self.assertGreater(global_metrics["accepted_count"], 0)
            self.assertIs(
                global_metrics["false_safe_among_accepted_defined"], True
            )
            self.assertIsNotNone(
                global_metrics["false_safe_among_accepted_wilson_lower"]
            )
            self.assertIsNotNone(
                global_metrics["false_safe_among_accepted_wilson_upper"]
            )
            self.assertEqual(
                manifest["compiled_contract"][
                    "execution_compatibility_contract"
                ],
                "phase_indexed_execution_box_v1",
            )
            self.assertEqual(
                manifest["compiled_contract"]["minimum_denominator"], 1.0e-9
            )

            with (output_path / "phase_rejoin_recovery_radii_bounds.csv").open(
                "r", encoding="utf-8", newline=""
            ) as stream:
                scale_rows = list(csv.DictReader(stream))
            self.assertEqual(tuple(scale_rows[0]), FITTER.OUTPUT_COLUMNS)
            self.assertEqual([row["phase_index"] for row in scale_rows], ["0", "1"])
            for row in scale_rows:
                self.assertEqual(float(row["shrinkage"]), 0.5)
                for name in FITTER.STATE_RADIUS_COLUMNS:
                    self.assertAlmostEqual(float(row[name]), 0.15)
                for name in FITTER.EXECUTION_BOUND_COLUMNS:
                    self.assertAlmostEqual(float(row[name]), 0.05)

            verified = FITTER.verify_manifest(manifest_path)
            self.assertEqual(verified["status"], "EMPIRICAL_HELD_OUT_PASS")
            self.assert_no_safety_claim(manifest, report)

    def test_split_pollution_is_rejected(self):
        baseline = self.make_rows()
        fit_row = next(row for row in baseline if row["split"] == "fit")
        tune_row_index = next(
            index
            for index, row in enumerate(baseline)
            if row["split"] == "tune"
        )

        cases = {}
        same_seed = copy.deepcopy(baseline)
        same_seed[tune_row_index]["seed"] = fit_row["seed"]
        cases["seed across splits"] = same_seed

        same_rollout = copy.deepcopy(baseline)
        same_rollout[tune_row_index]["rollout_id"] = fit_row["rollout_id"]
        cases["rollout across splits"] = same_rollout

        changed_seed = copy.deepcopy(baseline)
        second_fit_phase = next(
            index
            for index, row in enumerate(changed_seed)
            if row["rollout_id"] == fit_row["rollout_id"]
            and row["phase_index"] == "1"
        )
        changed_seed[second_fit_phase]["seed"] = "1999"
        cases["rollout across seeds"] = changed_seed

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for label, rows in cases.items():
                with self.subTest(label=label):
                    input_path = root / (label.replace(" ", "_") + ".csv")
                    self.write_rows(input_path, rows)
                    with self.assertRaisesRegex(
                        FITTER.RecoveryFitError, "crosses"
                    ):
                        FITTER.load_recovery_csv(input_path)

    def test_missing_and_noncontiguous_phase_coverage_is_rejected(self):
        baseline = self.make_rows()
        missing_in_held_out = [
            row
            for row in baseline
            if not (row["split"] == "held_out" and row["phase_index"] == "1")
        ]
        noncontiguous = copy.deepcopy(baseline)
        for row in noncontiguous:
            if row["phase_index"] == "1":
                row["phase_index"] = "2"

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cases = (
                ("missing_split_phase.csv", missing_in_held_out, "coverage differs"),
                ("noncontiguous.csv", noncontiguous, "missing phase index"),
            )
            for filename, rows, message in cases:
                with self.subTest(filename=filename):
                    input_path = root / filename
                    self.write_rows(input_path, rows)
                    with self.assertRaisesRegex(FITTER.RecoveryFitError, message):
                        FITTER.load_recovery_csv(input_path)

    def test_manifest_verifier_detects_bound_report_input_and_manifest_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "recovery.csv"
            output_path = root / "evidence"
            self.write_rows(input_path, self.make_rows())
            return_code, _, stderr = self.run_fit(input_path, output_path)
            self.assertEqual(return_code, 0, stderr)
            manifest_path = output_path / "manifest.json"

            targets = (
                output_path / "phase_rejoin_recovery_radii_bounds.csv",
                output_path / "held_out_report.json",
                input_path,
                manifest_path,
            )
            for target in targets:
                with self.subTest(target=target.name):
                    original = target.read_bytes()
                    target.write_bytes(original + b"\n")
                    with self.assertRaises(FITTER.RecoveryFitError):
                        FITTER.verify_manifest(manifest_path)
                    target.write_bytes(original)
                    FITTER.verify_manifest(manifest_path)

    def test_held_out_no_go_does_not_change_fit_or_tuning(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pass_input = root / "pass.csv"
            no_go_input = root / "no_go.csv"
            pass_output = root / "pass_evidence"
            no_go_output = root / "no_go_evidence"
            self.write_rows(pass_input, self.make_rows(0.08))
            self.write_rows(no_go_input, self.make_rows(0.04))

            pass_code, _, pass_stderr = self.run_fit(pass_input, pass_output)
            no_go_code, no_go_stdout, no_go_stderr = self.run_fit(
                no_go_input, no_go_output
            )
            self.assertEqual(pass_code, 0, pass_stderr)
            self.assertEqual(no_go_code, 4, no_go_stderr)
            self.assertIn("NO_GO", no_go_stdout)

            pass_manifest = json.loads(
                (pass_output / "manifest.json").read_text(encoding="utf-8")
            )
            no_go_manifest = json.loads(
                (no_go_output / "manifest.json").read_text(encoding="utf-8")
            )
            no_go_report = json.loads(
                (no_go_output / "held_out_report.json").read_text(encoding="utf-8")
            )
            self.assertEqual(no_go_manifest["status"], "NO_GO")
            self.assertEqual(no_go_manifest["held_out"]["status"], "NO_GO")
            self.assertEqual(no_go_report["status"], "NO_GO")
            self.assertEqual(pass_manifest["tune"], no_go_manifest["tune"])
            self.assertEqual(
                (pass_output / "phase_rejoin_recovery_radii_bounds.csv").read_bytes(),
                (no_go_output / "phase_rejoin_recovery_radii_bounds.csv").read_bytes(),
            )
            self.assertEqual(no_go_manifest["tune"]["selected_shrinkage"], 0.5)
            self.assertEqual(
                {path.name for path in no_go_output.iterdir()},
                {
                    "phase_rejoin_recovery_radii_bounds.csv",
                    "held_out_report.json",
                    "manifest.json",
                    "manifest.sha256",
                },
            )
            FITTER.verify_manifest(no_go_output / "manifest.json")
            self.assert_no_safety_claim(no_go_manifest, no_go_report)

    def test_zero_accepted_reports_conditional_false_safe_as_undefined(self):
        rows = self.make_rows()
        for row in rows:
            if row["split"] == "held_out":
                for name in FITTER.ERROR_COLUMNS:
                    row[name] = "1.0"

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "zero_accepted.csv"
            output_path = root / "evidence"
            self.write_rows(input_path, rows)

            return_code, _, stderr = self.run_fit(input_path, output_path)
            self.assertEqual(return_code, 4, stderr)
            report = json.loads(
                (output_path / "held_out_report.json").read_text(
                    encoding="utf-8"
                )
            )
            metrics = report["evaluation"]["global"]
            self.assertEqual(metrics["accepted_count"], 0)
            self.assertIs(
                metrics["false_safe_among_accepted_defined"], False
            )
            self.assertIsNone(metrics["false_safe_among_accepted"])
            self.assertIsNone(
                metrics["false_safe_among_accepted_wilson_lower"]
            )
            self.assertIsNone(
                metrics["false_safe_among_accepted_wilson_upper"]
            )

    def test_existing_output_directory_is_rejected_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "recovery.csv"
            output_path = root / "existing"
            output_path.mkdir()
            sentinel = output_path / "sentinel.txt"
            sentinel.write_text("preserve\n", encoding="utf-8")
            self.write_rows(input_path, self.make_rows())

            return_code, _, stderr = self.run_fit(input_path, output_path)
            self.assertEqual(return_code, 2)
            self.assertIn("output directory already exists", stderr)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve\n")
            self.assertEqual(list(output_path.iterdir()), [sentinel])

    def test_extreme_finite_input_fails_closed_without_overflow(self):
        rows = self.make_rows()
        for row in rows:
            if row["split"] == "fit":
                row["x"] = "1e308"

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "extreme.csv"
            output_path = root / "evidence"
            self.write_rows(input_path, rows)

            return_code, _, stderr = self.run_fit(input_path, output_path)
            self.assertEqual(return_code, 2)
            self.assertIn("non-finite", stderr)
            self.assertFalse(output_path.exists())


if __name__ == "__main__":
    unittest.main()
