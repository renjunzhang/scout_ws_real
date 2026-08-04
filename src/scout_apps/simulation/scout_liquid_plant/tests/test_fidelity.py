#!/usr/bin/env python3
"""Tests for offline, development-only plant-fidelity verification."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from scout_liquid_plant.fidelity import (  # noqa: E402
    COMPARISON_MANIFEST_SCHEMA_VERSION,
    REFERENCE_EVIDENCE_SCHEMA_VERSION,
    THRESHOLD_POLICY_SCHEMA_VERSION,
    FidelityValidationError,
    REPORT_SCHEMA_VERSION,
    REPORT_TYPE,
    sha256_file,
    validate_report_schema,
    verify_development_fidelity,
    write_report,
)


def _hash_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


class DevelopmentFidelityTest(unittest.TestCase):
    def _write_json(self, path: Path, value) -> None:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    @staticmethod
    def _signal_rows(amplitude: float, phase_rad: float = 0.0, damping: float = 0.04):
        frequency_hz = 0.8
        rows = []
        for index in range(1601):
            time_sec = index * 0.01
            value = amplitude * math.exp(-damping * 2.0 * math.pi * frequency_hz * time_sec) * math.sin(
                2.0 * math.pi * frequency_hz * time_sec + phase_rad
            )
            rows.append({"time_sec": time_sec, "value": value})
        return rows

    def _write_signal_json(self, path: Path, amplitude: float, phase_rad: float = 0.0) -> None:
        self._write_json(
            path,
            {
                "schema_version": "smpcc-liquid-plant-signal-v1",
                "unit": "m",
                "samples": self._signal_rows(amplitude, phase_rad),
            },
        )

    def _write_signal_csv(self, path: Path, amplitude: float, phase_rad: float = 0.0) -> None:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["time_sec", "value"])
            writer.writeheader()
            writer.writerows(self._signal_rows(amplitude, phase_rad))

    def _policy(self):
        return {
            "schema_version": THRESHOLD_POLICY_SCHEMA_VERSION,
            "policy_id": "DEV-TEST-FIDELITY-POLICY",
            "development_only": True,
            "formal": False,
            "amplitude": {"relative_error_max": 0.15},
            "frequency": {"relative_error_max": 0.05},
            "damping": {"absolute_error_max": 0.03},
            "phase": {"absolute_error_deg_max": 15.0},
            "ranking": {
                "metric": "amplitude",
                "minimum_cases": 2,
                "require_exact_order": True,
                "tie_relative_tolerance": 0.01,
            },
        }

    def _reference_evidence(self, case_id: str, signal_hash: str, *, source_topic: str = "/camera/rgb/liquid_height"):
        return {
            "schema_version": REFERENCE_EVIDENCE_SCHEMA_VERSION,
            "status": "FROZEN",
            "real_measurement": True,
            "measurement_independent_of_plant": True,
            "reference_kind": "REAL_RGB_LIQUID_HEIGHT",
            "case_id": case_id,
            "reference_signal_sha256": signal_hash,
            "reference_freeze_id": "REAL-H0-RGB-FREEZE-TEST",
            "source_bag_sha256": _hash_text("source bag " + case_id),
            "extraction_pipeline_sha256": _hash_text("extract " + case_id),
            "calibration_sha256": _hash_text("calibration " + case_id),
            "source_topic": source_topic,
        }

    def _write_complete_inputs(self, root: Path):
        cases = []
        for case_id, reference_amplitude, plant_amplitude, plant_format in (
            ("excitation_high", 0.030, 0.0285, "json"),
            ("excitation_low", 0.015, 0.0145, "csv"),
        ):
            reference_path = root / (case_id + "_reference.json")
            plant_path = root / (case_id + "_plant." + plant_format)
            self._write_signal_json(reference_path, reference_amplitude)
            if plant_format == "csv":
                self._write_signal_csv(plant_path, plant_amplitude, math.radians(4.0))
            else:
                self._write_signal_json(plant_path, plant_amplitude, math.radians(4.0))
            evidence_path = root / (case_id + "_reference_evidence.json")
            evidence = self._reference_evidence(case_id, sha256_file(reference_path))
            self._write_json(evidence_path, evidence)
            cases.append(
                {
                    "case_id": case_id,
                    "signal_unit": "m",
                    "plant_signal_path": str(plant_path.resolve()),
                    "plant_signal_sha256": sha256_file(plant_path),
                    "reference_signal_path": str(reference_path.resolve()),
                    "reference_signal_sha256": sha256_file(reference_path),
                    "reference_evidence_path": str(evidence_path.resolve()),
                    "reference_evidence_sha256": sha256_file(evidence_path),
                }
            )
        manifest_path = root / "comparison_manifest.json"
        self._write_json(
            manifest_path,
            {
                "schema_version": COMPARISON_MANIFEST_SCHEMA_VERSION,
                "comparison_id": "SIM-DEV-PLANT-FIDELITY-TEST",
                "development_only": True,
                "formal": False,
                "plant_identity": {
                    "plant_code_hash": _hash_text("plant code"),
                    "plant_parameter_hash": _hash_text("plant parameter"),
                    "plant_input_schema_hash": _hash_text("plant input"),
                    "plant_output_schema_hash": _hash_text("plant output"),
                },
                "cases": cases,
            },
        )
        policy_path = root / "threshold_policy.json"
        self._write_json(policy_path, self._policy())
        return manifest_path, policy_path

    def _verify(self, manifest_path: Path, policy_path: Path):
        return verify_development_fidelity(
            comparison_manifest_path=str(manifest_path.resolve()),
            comparison_manifest_sha256=sha256_file(manifest_path),
            threshold_policy_path=str(policy_path.resolve()),
            threshold_policy_sha256=sha256_file(policy_path),
        )

    def test_hash_bound_real_reference_can_complete_development_metrics_but_never_formal_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            manifest_path, policy_path = self._write_complete_inputs(Path(temporary))
            report = self._verify(manifest_path, policy_path)
            validate_report_schema(report)
            self.assertEqual(REPORT_TYPE, report["report_type"])
            self.assertEqual(REPORT_SCHEMA_VERSION, report["report_schema_version"])
            self.assertEqual("DEVELOPMENT_METRICS_COMPLETE_NOT_FORMAL", report["status"])
            self.assertFalse(report["formal"])
            self.assertTrue(report["development_only"])
            self.assertFalse(report["physical_primary_eligible"])
            self.assertEqual("UNVALIDATED_DEVELOPMENT_ONLY", report["fidelity_validation_status"])
            self.assertEqual(
                {"amplitude": "PASS", "frequency": "PASS", "damping": "PASS", "phase": "PASS", "ranking": "PASS"},
                report["validation_dimensions"],
            )
            self.assertFalse(report["formal_gate_compatibility"]["eligible"])
            self.assertEqual(["excitation_high", "excitation_low"], report["ranking"]["reference_order"])
            self.assertEqual(["excitation_high", "excitation_low"], report["ranking"]["plant_order"])

    def test_missing_frozen_reference_evidence_is_a_schema_valid_no_go(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["cases"][0].pop("reference_evidence_path")
            manifest["cases"][0].pop("reference_evidence_sha256")
            self._write_json(manifest_path, manifest)
            report = self._verify(manifest_path, policy_path)
            validate_report_schema(report)
            self.assertEqual("NO_GO", report["status"])
            self.assertTrue(any(error["code"] == "MISSING_FROZEN_REAL_REFERENCE" for error in report["errors"]))
            self.assertEqual(
                {"amplitude": "NOT_EVALUATED", "frequency": "NOT_EVALUATED", "damping": "NOT_EVALUATED", "phase": "NOT_EVALUATED", "ranking": "NOT_EVALUATED"},
                report["validation_dimensions"],
            )

    def test_signal_hash_mismatch_is_no_go_not_a_best_effort_comparison(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["cases"][1]["plant_signal_sha256"] = "0" * 64
            self._write_json(manifest_path, manifest)
            report = self._verify(manifest_path, policy_path)
            self.assertEqual("NO_GO", report["status"])
            self.assertTrue(any(error["code"] == "HASH_MISMATCH" for error in report["errors"]))
            self.assertFalse(report["case_metrics"])

    def test_h_proxy_or_h_modal_cannot_be_presented_as_real_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            evidence_path = Path(manifest["cases"][0]["reference_evidence_path"])
            evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
            evidence["source_topic"] = "/slosh/height"
            self._write_json(evidence_path, evidence)
            manifest["cases"][0]["reference_evidence_sha256"] = sha256_file(evidence_path)
            self._write_json(manifest_path, manifest)
            report = self._verify(manifest_path, policy_path)
            self.assertEqual("NO_GO", report["status"])
            self.assertTrue(any(error["code"] == "PROXY_REFERENCE_FORBIDDEN" for error in report["errors"]))

    def test_cli_writes_no_go_and_exits_nonzero_when_reference_evidence_is_missing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["cases"][0].pop("reference_evidence_path")
            manifest["cases"][0].pop("reference_evidence_sha256")
            self._write_json(manifest_path, manifest)
            output_path = root / "no_go_report.json"
            command = [
                sys.executable,
                str(PACKAGE_ROOT / "scripts" / "liquid_plant_fidelity_verify.py"),
                "--comparison-manifest",
                str(manifest_path.resolve()),
                "--comparison-manifest-sha256",
                sha256_file(manifest_path),
                "--threshold-policy",
                str(policy_path.resolve()),
                "--threshold-policy-sha256",
                sha256_file(policy_path),
                "--output",
                str(output_path.resolve()),
            ]
            result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            self.assertEqual(2, result.returncode, result.stdout + result.stderr)
            report = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual("NO_GO", report["status"])
            self.assertFalse(report["formal"])

    def test_report_writer_refuses_implicit_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            report = self._verify(manifest_path, policy_path)
            output = root / "report.json"
            write_report(output.resolve(), report)
            with self.assertRaises(FidelityValidationError) as context:
                write_report(output.resolve(), report)
            self.assertEqual("OUTPUT_EXISTS", context.exception.code)

    def test_report_schema_rejects_any_attempt_to_mark_development_report_formal(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path, policy_path = self._write_complete_inputs(root)
            report = self._verify(manifest_path, policy_path)
            report["formal"] = True
            with self.assertRaises(FidelityValidationError) as context:
                validate_report_schema(report)
            self.assertEqual("MALFORMED_REPORT", context.exception.code)

    def test_shipped_json_schema_documents_the_nonformal_contract(self):
        schema = json.loads(
            (PACKAGE_ROOT / "schema" / "liquid_plant_fidelity_report_v1.json").read_text(encoding="utf-8")
        )
        self.assertFalse(schema["properties"]["formal"]["const"])
        self.assertTrue(schema["properties"]["development_only"]["const"])
        self.assertFalse(schema["properties"]["physical_primary_eligible"]["const"])
        self.assertIn("NO_GO", schema["properties"]["status"]["enum"])


if __name__ == "__main__":
    unittest.main()
