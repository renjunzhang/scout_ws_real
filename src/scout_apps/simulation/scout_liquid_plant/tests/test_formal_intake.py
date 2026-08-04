#!/usr/bin/env python3
"""Adversarial tests for formal liquid-plant evidence intake.

All apparently formal documents here are ephemeral unit-test fixtures.  They
prove that the intake refuses incomplete or development evidence; they are
never release assets and cannot be used as a formal freeze.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from scout_liquid_plant.formal_intake import (  # noqa: E402
    DIMENSIONS,
    FORMAL_APPROVAL_REPORT_TYPE,
    FORMAL_APPROVAL_SCHEMA_VERSION,
    FORMAL_CAPABILITY_REPORT_TYPE,
    FORMAL_FIDELITY_REPORT_TYPE,
    FORMAL_FIDELITY_SCHEMA_VERSION,
    FORMAL_ISOLATION_EVIDENCE_REPORT_TYPE,
    FORMAL_ISOLATION_EVIDENCE_SCHEMA_VERSION,
    FORMAL_PLANT_SIGNAL_EVIDENCE_REPORT_TYPE,
    FORMAL_PLANT_SIGNAL_EVIDENCE_SCHEMA_VERSION,
    FORMAL_REFERENCE_EVIDENCE_REPORT_TYPE,
    FORMAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
    FORMAL_RELEASE_REPORT_TYPE,
    FORMAL_RELEASE_SCHEMA_VERSION,
    INTAKE_REQUEST_SCHEMA_VERSION,
    TOOLCHAIN_BINDING_TYPE,
    TOOLCHAIN_BINDING_SCHEMA_VERSION,
    FormalEvidenceError,
    assemble_formal_capability,
    canonical_hash,
    sha256_file,
    write_formal_capability_bundle,
)


def _load_toolchain():
    path = (
        PACKAGE_ROOT.parent
        / "spmpc_sim_local_planner"
        / "scripts"
        / "smpcc_sim_toolchain.py"
    )
    spec = importlib.util.spec_from_file_location("smpcc_sim_toolchain_for_formal_intake_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FormalEvidenceIntakeTest(unittest.TestCase):
    maxDiff = None

    @staticmethod
    def _write_json(path: Path, value) -> None:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    @staticmethod
    def _write_text(path: Path, value: str) -> None:
        path.write_text(value, encoding="utf-8")

    @staticmethod
    def _descriptor(path: Path):
        return {"path": str(path.resolve()), "sha256": sha256_file(path)}

    @staticmethod
    def _dimensions():
        return {key: "PASS" for key in DIMENSIONS}

    def _make_inputs(self, root: Path, code_source: str | None = None):
        """Construct one complete, independent test-only evidence chain."""

        release_id = "TEST-FORMAL-PLANT-RELEASE-001"
        sim_freeze_id = "TEST-SIM-FREEZE-001"
        code_path = root / "independent_formal_plant.py"
        self._write_text(
            code_path,
            code_source
            or (
                "# independently reviewed multi-mode plant source\n"
                "def integrate_executed_odom(sample):\n"
                "    return sample\n"
            ),
        )
        parameter_path = root / "formal_plant_parameters.json"
        self._write_json(
            parameter_path,
            {
                "document_type": "SMPCC_SIM_FORMAL_LIQUID_PLANT_PARAMETERS",
                "status": "FROZEN",
                "formal": True,
                "development_only": False,
                "release_id": release_id,
                "initial_state_rule_hash": hashlib.sha256(b"test initial state rule").hexdigest(),
                "integration_step_sec": 0.002,
                "container_parameter_set": "C1_TEST_FORMAL",
            },
        )
        input_path = root / "formal_plant_input_schema.json"
        self._write_json(
            input_path,
            {
                "document_type": "SMPCC_SIM_FORMAL_LIQUID_PLANT_INPUT_SCHEMA",
                "status": "FROZEN",
                "formal": True,
                "development_only": False,
                "release_id": release_id,
                "input": {
                    "topic": "/odom",
                    "semantic": "executed_simulated_base_motion",
                    "only_executed_base_motion": True,
                    "raw_command_input_forbidden": True,
                },
                "isolation": {
                    "implementation_isolated_from_controller": True,
                    "controller_hidden_state_access": False,
                    "controller_state_import_forbidden": True,
                    "controller_truth_subscription_forbidden": True,
                    "plant_reads_raw_command": False,
                    "plant_subscriptions": ["/odom"],
                },
            },
        )
        output_path = root / "formal_plant_output_schema.json"
        self._write_json(
            output_path,
            {
                "document_type": "SMPCC_SIM_FORMAL_LIQUID_PLANT_OUTPUT_SCHEMA",
                "status": "FROZEN",
                "formal": True,
                "development_only": False,
                "release_id": release_id,
                "truth_topic": "/sim_truth/liquid_height",
                "controller_feedback_forbidden": True,
                "outputs": {"liquid_height": {"topic": "/sim_truth/liquid_height", "unit": "m"}},
            },
        )
        artifacts = {
            "plant_code": self._descriptor(code_path),
            "plant_parameters": self._descriptor(parameter_path),
            "plant_input_schema": self._descriptor(input_path),
            "plant_output_schema": self._descriptor(output_path),
        }
        release_path = root / "formal_plant_release.json"
        release = {
            "schema_version": FORMAL_RELEASE_SCHEMA_VERSION,
            "report_type": FORMAL_RELEASE_REPORT_TYPE,
            "status": "FROZEN",
            "formal": True,
            "development_only": False,
            "release_id": release_id,
            "sim_freeze_id": sim_freeze_id,
            "plant": {
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
            },
            "artifacts": artifacts,
        }
        release["release_payload_hash"] = canonical_hash(release)
        self._write_json(release_path, release)
        release_descriptor = self._descriptor(release_path)

        verifier_path = root / "independent_formal_fidelity_verifier.py"
        self._write_text(
            verifier_path,
            "# external reviewed fidelity verifier\n"
            "def compare_real_reference(sample):\n"
            "    return sample\n",
        )
        evidence_entries = []
        plant_signal_entries = []
        for case_id, source_topic in (
            ("excitation_high", "/camera/rgb/liquid_height"),
            ("excitation_low", "/liquid_sensor/height"),
        ):
            signal_path = root / (case_id + "_real_signal.csv")
            bag_path = root / (case_id + "_real_source.bag")
            extraction_path = root / (case_id + "_rgb_or_sensor_extract.py")
            calibration_path = root / (case_id + "_calibration.json")
            self._write_text(signal_path, "time_sec,value\n0.0,0.0\n0.1,0.01\n")
            self._write_text(bag_path, "frozen real bag " + case_id + "\n")
            self._write_text(extraction_path, "# frozen external extraction " + case_id + "\n")
            self._write_json(calibration_path, {"case_id": case_id, "calibration": "frozen"})
            evidence_path = root / (case_id + "_real_reference_evidence.json")
            evidence = {
                "schema_version": FORMAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
                "report_type": FORMAL_REFERENCE_EVIDENCE_REPORT_TYPE,
                "status": "FROZEN",
                "formal": True,
                "development_only": False,
                "case_id": case_id,
                "reference_kind": "REAL_RGB_LIQUID_HEIGHT" if "rgb" in source_topic else "REAL_LIQUID_HEIGHT_SENSOR",
                "real_measurement": True,
                "measurement_independent_of_plant": True,
                "reference_freeze_id": "TEST-REAL-REFERENCE-FREEZE-001",
                "formal_release_manifest_hash": release_descriptor["sha256"],
                "reference_signal_path": str(signal_path.resolve()),
                "reference_signal_hash": sha256_file(signal_path),
                "source_bag_path": str(bag_path.resolve()),
                "source_bag_hash": sha256_file(bag_path),
                "extraction_pipeline_path": str(extraction_path.resolve()),
                "extraction_pipeline_hash": sha256_file(extraction_path),
                "calibration_path": str(calibration_path.resolve()),
                "calibration_hash": sha256_file(calibration_path),
                "source_topic": source_topic,
            }
            self._write_json(evidence_path, evidence)
            evidence_entries.append(
                {
                    "case_id": case_id,
                    "reference_evidence_path": str(evidence_path.resolve()),
                    "reference_evidence_hash": sha256_file(evidence_path),
                    "reference_signal_path": str(signal_path.resolve()),
                    "reference_signal_hash": sha256_file(signal_path),
                    "reference_kind": evidence["reference_kind"],
                }
            )
            plant_signal_path = root / (case_id + "_formal_plant_signal.csv")
            self._write_text(plant_signal_path, "time_sec,value\n0.0,0.0\n0.1,0.008\n")
            plant_run_path = root / (case_id + "_formal_plant_run_evidence.json")
            plant_run = {
                "schema_version": FORMAL_PLANT_SIGNAL_EVIDENCE_SCHEMA_VERSION,
                "report_type": FORMAL_PLANT_SIGNAL_EVIDENCE_REPORT_TYPE,
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "case_id": case_id,
                "formal_release_manifest_hash": release_descriptor["sha256"],
                "truth_topic": "/sim_truth/liquid_height",
                "plant_signal_path": str(plant_signal_path.resolve()),
                "plant_signal_hash": sha256_file(plant_signal_path),
                "plant_code_hash": artifacts["plant_code"]["sha256"],
                "plant_parameter_hash": artifacts["plant_parameters"]["sha256"],
                "plant_input_schema_hash": artifacts["plant_input_schema"]["sha256"],
                "plant_output_schema_hash": artifacts["plant_output_schema"]["sha256"],
            }
            self._write_json(plant_run_path, plant_run)
            plant_signal_entries.append(
                {
                    "case_id": case_id,
                    "plant_signal_path": str(plant_signal_path.resolve()),
                    "plant_signal_hash": sha256_file(plant_signal_path),
                    "plant_run_manifest_path": str(plant_run_path.resolve()),
                    "plant_run_manifest_hash": sha256_file(plant_run_path),
                    "plant_signal_topic": "/sim_truth/liquid_height",
                }
            )
        evidence_entries = sorted(evidence_entries, key=lambda item: item["case_id"])
        plant_signal_entries = sorted(plant_signal_entries, key=lambda item: item["case_id"])
        fidelity_path = root / "external_formal_fidelity_report.json"
        fidelity = {
            "schema_version": FORMAL_FIDELITY_SCHEMA_VERSION,
            "report_type": FORMAL_FIDELITY_REPORT_TYPE,
            "status": "PASS",
            "formal": True,
            "development_only": False,
            "fidelity_validation_status": "PASS",
            "truth_topic": "/sim_truth/liquid_height",
            "independently_produced": True,
            "formal_release_manifest_hash": release_descriptor["sha256"],
            "fidelity_verifier_source_path": str(verifier_path.resolve()),
            "fidelity_verifier_source_hash": sha256_file(verifier_path),
            "formal_reference_evidence": evidence_entries,
            "formal_reference_evidence_set_hash": canonical_hash(evidence_entries),
            "formal_plant_signal_evidence": plant_signal_entries,
            "formal_plant_signal_evidence_set_hash": canonical_hash(plant_signal_entries),
            "validation_dimensions": self._dimensions(),
            "plant_code_hash": artifacts["plant_code"]["sha256"],
            "plant_parameter_hash": artifacts["plant_parameters"]["sha256"],
            "plant_input_schema_hash": artifacts["plant_input_schema"]["sha256"],
            "plant_output_schema_hash": artifacts["plant_output_schema"]["sha256"],
        }
        self._write_json(fidelity_path, fidelity)
        fidelity_descriptor = self._descriptor(fidelity_path)

        graph_path = root / "live_graph_evidence.json"
        self._write_json(graph_path, {"node": "/smpcc/controller", "subscriptions": {"/odom": ["/smpcc/controller"]}})
        isolation_path = root / "controller_plant_isolation_evidence.json"
        isolation = {
            "schema_version": FORMAL_ISOLATION_EVIDENCE_SCHEMA_VERSION,
            "report_type": FORMAL_ISOLATION_EVIDENCE_REPORT_TYPE,
            "status": "PASS",
            "formal": True,
            "development_only": False,
            "formal_release_manifest_hash": release_descriptor["sha256"],
            "truth_topic": "/sim_truth/liquid_height",
            "observation_mode": "STATIC_AND_LIVE_ROS_GRAPH",
            "implementation_isolated_from_controller": True,
            "controller_hidden_state_access": False,
            "controller_state_import": False,
            "controller_subscription_to_truth": False,
            "plant_reads_raw_command": False,
            "plant_subscriptions": ["/odom"],
            "checkpoints": {"ready": "PASS", "pre_motion": "PASS", "postflight": "PASS"},
            "controller_nodes": ["/smpcc/controller", "/smpcc/tracker"],
            "graph_evidence": [self._descriptor(graph_path)],
            "plant_code_hash": artifacts["plant_code"]["sha256"],
            "plant_input_schema_hash": artifacts["plant_input_schema"]["sha256"],
            "plant_output_schema_hash": artifacts["plant_output_schema"]["sha256"],
        }
        self._write_json(isolation_path, isolation)
        isolation_descriptor = self._descriptor(isolation_path)

        approval_path = root / "external_formal_approval.json"
        approval = {
            "schema_version": FORMAL_APPROVAL_SCHEMA_VERSION,
            "report_type": FORMAL_APPROVAL_REPORT_TYPE,
            "status": "APPROVED",
            "formal": True,
            "development_only": False,
            "external_approval": True,
            "approval_id": "TEST-EXTERNAL-APPROVAL-001",
            "approval_authority": "external-test-review-board",
            "issued_at_utc": "2026-08-02T00:00:00Z",
            "approval_scope": "SMPCC_SIM_FORMAL_PHYSICAL_PRIMARY",
            "release_id": release_id,
            "sim_freeze_id": sim_freeze_id,
            "formal_release_manifest_hash": release_descriptor["sha256"],
            "fidelity_report_hash": fidelity_descriptor["sha256"],
            "controller_isolation_evidence_hash": isolation_descriptor["sha256"],
            "formal_reference_evidence_set_hash": canonical_hash(evidence_entries),
            "formal_plant_signal_evidence_set_hash": canonical_hash(plant_signal_entries),
            "validation_dimensions": self._dimensions(),
            "plant_code_hash": artifacts["plant_code"]["sha256"],
            "plant_parameter_hash": artifacts["plant_parameters"]["sha256"],
            "plant_input_schema_hash": artifacts["plant_input_schema"]["sha256"],
            "plant_output_schema_hash": artifacts["plant_output_schema"]["sha256"],
        }
        self._write_json(approval_path, approval)

        request_path = root / "formal_intake_request.json"
        request = {
            "schema_version": INTAKE_REQUEST_SCHEMA_VERSION,
            "request_id": "TEST-FORMAL-INTAKE-001",
            "request_purpose": "ASSEMBLE_EXTERNAL_FORMAL_CAPABILITY_ONLY",
            "formal": True,
            "development_only": False,
            "formal_release_manifest": release_descriptor,
            **artifacts,
            "fidelity_report": fidelity_descriptor,
            "controller_isolation_evidence": isolation_descriptor,
            "external_approval": self._descriptor(approval_path),
        }
        self._write_json(request_path, request)
        return {
            "request_path": request_path,
            "release_path": release_path,
            "code_path": code_path,
            "parameter_path": parameter_path,
            "input_path": input_path,
            "output_path": output_path,
            "fidelity_path": fidelity_path,
            "isolation_path": isolation_path,
            "approval_path": approval_path,
            "evidence_entries": evidence_entries,
            "plant_signal_entries": plant_signal_entries,
        }

    def _assemble(self, inputs):
        return assemble_formal_capability(
            str(inputs["request_path"].resolve()),
            sha256_file(inputs["request_path"]),
        )

    def test_external_formal_evidence_assembles_toolchain_compatible_binding(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            assembly = self._assemble(inputs)
            report = assembly.capability_report
            self.assertEqual(FORMAL_CAPABILITY_REPORT_TYPE, report["report_type"])
            self.assertTrue(report["formal"])
            self.assertFalse(report["development_only"])
            self.assertTrue(report["physical_primary_eligible"])
            self.assertFalse(report["fidelity_report_generated_by_intake"])
            self.assertFalse(report["runtime_execution_performed"])
            self.assertEqual("NOT_CONFIGURED", report["cryptographic_trust_anchor"])
            self.assertEqual(
                "NOT_INDEPENDENTLY_AUTHENTICATED",
                report["external_approval_authentication_status"],
            )
            self.assertEqual(self._dimensions(), {key: "PASS" for key in DIMENSIONS})
            self.assertEqual(2, len(report["formal_reference_evidence"]))
            self.assertEqual(2, len(report["formal_plant_signal_evidence"]))
            report_output = root / "formal_capability_report.json"
            binding_output = root / "toolchain_capability_binding.json"
            result = write_formal_capability_bundle(assembly, report_output.resolve(), binding_output.resolve())
            self.assertEqual("PASS", result["status"])
            self.assertEqual(0o444, report_output.stat().st_mode & 0o777)
            self.assertEqual(0o444, binding_output.stat().st_mode & 0o777)
            binding = json.loads(binding_output.read_text(encoding="utf-8"))
            self.assertEqual(TOOLCHAIN_BINDING_SCHEMA_VERSION, binding["schema_version"])
            self.assertEqual(TOOLCHAIN_BINDING_TYPE, binding["binding_type"])
            self.assertEqual(str(report_output.resolve()), binding["plant_capability_report_path"])
            self.assertEqual(sha256_file(report_output), binding["plant_capability_report_hash"])
            self.assertEqual(binding["plant_capability_report_hash"], binding["formal_intake_report_hash"])
            toolchain = _load_toolchain()
            passed = toolchain.validate_formal_liquid_plant_capability(binding)
            self.assertTrue(passed["eligible"], passed)

    def test_development_parameter_template_is_rejected_even_when_hash_bound(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            development = PACKAGE_ROOT / "config" / "C1_development_unvalidated.yaml"
            request["plant_parameters"] = self._descriptor(development)
            self._write_json(inputs["request_path"], request)
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("DEVELOPMENT_ARTIFACT_FORBIDDEN", context.exception.code)

    def test_current_development_fidelity_report_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            development = PACKAGE_ROOT / "schema" / "liquid_plant_fidelity_report_v1.json"
            request["fidelity_report"] = self._descriptor(development)
            self._write_json(inputs["request_path"], request)
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("DEVELOPMENT_ARTIFACT_FORBIDDEN", context.exception.code)

    def test_any_nonpass_dimension_is_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            fidelity = json.loads(inputs["fidelity_path"].read_text(encoding="utf-8"))
            fidelity["validation_dimensions"]["phase"] = "FAIL"
            self._write_json(inputs["fidelity_path"], fidelity)
            request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            request["fidelity_report"] = self._descriptor(inputs["fidelity_path"])
            # The approval is deliberately left bound to the old report hash;
            # fidelity itself must fail before stale approval can rescue it.
            self._write_json(inputs["request_path"], request)
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("FIDELITY_NOT_PASS", context.exception.code)

    def test_proxy_or_modal_reference_provenance_is_rejected(self):
        for forbidden_topic in ("/slosh/height", "/spmpc/slosh_height"):
            with self.subTest(forbidden_topic=forbidden_topic), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                inputs = self._make_inputs(root)
                entry = inputs["evidence_entries"][0]
                evidence_path = Path(entry["reference_evidence_path"])
                evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
                evidence["source_topic"] = forbidden_topic
                self._write_json(evidence_path, evidence)
                fidelity = json.loads(inputs["fidelity_path"].read_text(encoding="utf-8"))
                for fidelity_entry in fidelity["formal_reference_evidence"]:
                    if fidelity_entry["case_id"] == entry["case_id"]:
                        fidelity_entry["reference_evidence_hash"] = sha256_file(evidence_path)
                fidelity["formal_reference_evidence_set_hash"] = canonical_hash(
                    sorted(fidelity["formal_reference_evidence"], key=lambda item: item["case_id"])
                )
                self._write_json(inputs["fidelity_path"], fidelity)
                request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
                request["fidelity_report"] = self._descriptor(inputs["fidelity_path"])
                self._write_json(inputs["request_path"], request)
                with self.assertRaises(FormalEvidenceError) as context:
                    self._assemble(inputs)
                self.assertEqual("FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE", context.exception.code)

    def test_formal_code_cannot_reuse_liquidsloshmodel_or_unvalidated_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root, "# LiquidSloshModel development_only\n")
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE", context.exception.code)

    def test_proxy_or_modal_plant_signal_cannot_enter_fidelity_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            fidelity = json.loads(inputs["fidelity_path"].read_text(encoding="utf-8"))
            fidelity["formal_plant_signal_evidence"][0]["plant_signal_topic"] = "/spmpc/slosh_height"
            fidelity["formal_plant_signal_evidence_set_hash"] = canonical_hash(
                sorted(fidelity["formal_plant_signal_evidence"], key=lambda item: item["case_id"])
            )
            self._write_json(inputs["fidelity_path"], fidelity)
            request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            request["fidelity_report"] = self._descriptor(inputs["fidelity_path"])
            self._write_json(inputs["request_path"], request)
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("TRUTH_TOPIC_MISMATCH", context.exception.code)

    def test_isolation_evidence_requires_all_three_checkpoints_and_no_hidden_state_access(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            isolation = json.loads(inputs["isolation_path"].read_text(encoding="utf-8"))
            isolation["controller_hidden_state_access"] = True
            self._write_json(inputs["isolation_path"], isolation)
            request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            request["controller_isolation_evidence"] = self._descriptor(inputs["isolation_path"])
            self._write_json(inputs["request_path"], request)
            with self.assertRaises(FormalEvidenceError) as context:
                self._assemble(inputs)
            self.assertEqual("ISOLATION_SEMANTICS_MISMATCH", context.exception.code)

    def test_writer_does_not_overwrite_and_cli_leaves_no_output_for_no_go(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = self._make_inputs(root)
            assembly = self._assemble(inputs)
            report_output = root / "formal_capability_report.json"
            binding_output = root / "toolchain_capability_binding.json"
            write_formal_capability_bundle(assembly, report_output.resolve(), binding_output.resolve())
            with self.assertRaises(FormalEvidenceError) as context:
                write_formal_capability_bundle(assembly, report_output.resolve(), (root / "another.json").resolve())
            self.assertEqual("OUTPUT_EXISTS", context.exception.code)

            bad_request = json.loads(inputs["request_path"].read_text(encoding="utf-8"))
            bad_request["formal"] = False
            self._write_json(inputs["request_path"], bad_request)
            cli_report = root / "should_not_exist_report.json"
            cli_binding = root / "should_not_exist_binding.json"
            command = [
                sys.executable,
                str(PACKAGE_ROOT / "scripts" / "liquid_plant_formal_evidence_intake.py"),
                "--intake-request",
                str(inputs["request_path"].resolve()),
                "--intake-request-sha256",
                sha256_file(inputs["request_path"]),
                "--capability-report-output",
                str(cli_report.resolve()),
                "--toolchain-binding-output",
                str(cli_binding.resolve()),
            ]
            result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
            self.assertEqual(2, result.returncode, result.stdout + result.stderr)
            self.assertFalse(cli_report.exists())
            self.assertFalse(cli_binding.exists())

    def test_formal_intake_implementation_has_no_ros_or_gazebo_runtime_dependency(self):
        source = (PACKAGE_ROOT / "src" / "scout_liquid_plant" / "formal_intake.py").read_text(encoding="utf-8")
        self.assertNotIn("import rospy", source)
        self.assertNotIn("roslaunch", source)
        self.assertNotIn("subprocess", source)
        script = (PACKAGE_ROOT / "scripts" / "liquid_plant_formal_evidence_intake.py").read_text(encoding="utf-8")
        self.assertNotIn("roslaunch", script)
        self.assertNotIn("import rospy", script)
        self.assertNotIn("subprocess", script)


if __name__ == "__main__":
    unittest.main()
