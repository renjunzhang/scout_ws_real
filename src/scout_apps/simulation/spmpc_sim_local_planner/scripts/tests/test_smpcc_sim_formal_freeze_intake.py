#!/usr/bin/env python3
"""Offline/adversarial tests for two-phase formal-freeze intake.

Every apparent formal artifact in this file lives in ``TemporaryDirectory``
and is deliberately incomplete or behind a mocked shared gate.  Nothing here
is a release fixture, no ROS/Gazebo process is started, and no result can be
mistaken for formal experimental data.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1]
MODULE_PATH = SCRIPTS / "smpcc_sim_formal_freeze_intake.py"


def load_intake():
    module_name = "smpcc_sim_formal_freeze_intake_test_target"
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot dynamically load formal-freeze intake")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class FormalFreezeIntakeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.intake = load_intake()

    @staticmethod
    def _write_json(path: Path, value) -> None:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    @staticmethod
    def _sha(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def _descriptor(self, path: Path):
        return {"path": str(path.resolve()), "sha256": self._sha(path)}

    def _basic_external_input(self, root: Path):
        """Make a deliberately incomplete external payload for NO_GO tests."""
        support = root / "external_support.txt"
        support.write_text("external evidence placeholder; not a formal release\n", encoding="utf-8")
        payload = root / "external_pre_receipt_payload.json"
        self._write_json(
            payload,
            {
                "protocol_id": self.intake.toolchain.FORMAL_PROTOCOL_ID,
                "sample_size": 8,
                "real_freeze_id": "EXTERNAL-REAL-FREEZE-TEST",
                "sim_freeze_id": "EXTERNAL-SIM-FREEZE-TEST",
                "git_revision": "0123456789abcdef",
                "build_id": "test-build",
                # This makes artifact-catalog coverage meaningful while the
                # actual shared gate correctly rejects all missing evidence.
                "external_support_path": str(support.resolve()),
                "external_support_hash": self._sha(support),
            },
        )
        declaration = root / "external_declarations.json"
        self._write_json(
            declaration,
            {
                "schema_version": self.intake.DECLARATION_SCHEMA_VERSION,
                "document_type": self.intake.DECLARATION_DOCUMENT_TYPE,
                "declaration_id": "EPHEMERAL-TEST-DECLARATION",
                "purpose": self.intake.DECLARATION_PURPOSE,
                "formal": True,
                "development_only": False,
                "protocol_id": self.intake.toolchain.FORMAL_PROTOCOL_ID,
                "pre_receipt_payload": self._descriptor(payload),
                "artifact_declarations": [
                    {"role": "external_support", **self._descriptor(support)},
                ],
            },
        )
        return declaration, payload, support

    def _manual_prepared(self, declaration: Path, payload: Path, support: Path):
        payload_document = json.loads(payload.read_text(encoding="utf-8"))
        declaration_artifact = self.intake._bound_artifact(self._descriptor(declaration), "test declaration")
        payload_artifact = self.intake._bound_artifact(self._descriptor(payload), "test payload")
        support_artifact = self.intake._bound_artifact(self._descriptor(support), "test support")
        return self.intake.PreparedPayload(
            declaration=declaration_artifact,
            payload_source=payload_artifact,
            payload_document=payload_document,
            direct_artifacts=(support_artifact,),
            artifact_declarations=(("external_support", support_artifact),),
            freeze_payload_hash=self.intake._canonical_hash(payload_document),
        )

    def _write_external_receipt(self, root: Path, request_path: Path, payload: Path) -> Path:
        request = json.loads(request_path.read_text(encoding="utf-8"))
        payload_document = json.loads(payload.read_text(encoding="utf-8"))
        receipt = root / "external_receipt.json"
        self._write_json(
            receipt,
            {
                "schema_version": self.intake.RECEIPT_SCHEMA_VERSION,
                "report_type": self.intake.RECEIPT_DOCUMENT_TYPE,
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "receipt_id": "EPHEMERAL-EXTERNAL-RECEIPT",
                "receipt_authority": "ephemeral-test-authority",
                "protocol_id": self.intake.toolchain.FORMAL_PROTOCOL_ID,
                "freeze_id": payload_document["sim_freeze_id"],
                "freeze_payload_hash": self.intake._canonical_hash(payload_document),
                "pre_receipt_request_hash": self._sha(request_path),
                "validator_hash": hashlib.sha256(b"external-validator").hexdigest(),
            },
        )
        return receipt

    def test_missing_external_evidence_is_no_go_and_cli_creates_no_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, _payload, _support = self._basic_external_input(root)
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.prepare_external_payload(str(declaration.resolve()), self._sha(declaration))
            self.assertEqual("FORMAL_GATE_NO_GO", caught.exception.code)
            self.assertIn("formal_bslosh_release", caught.exception.message)

            pre_output = root / "must_not_exist_pre.json"
            request_output = root / "must_not_exist_request.json"
            with contextlib.redirect_stdout(io.StringIO()) as stream:
                status = self.intake.main(
                    [
                        "prepare",
                        "--declarations",
                        str(declaration.resolve()),
                        "--declarations-sha256",
                        self._sha(declaration),
                        "--pre-receipt-output",
                        str(pre_output.resolve()),
                        "--receipt-request-output",
                        str(request_output.resolve()),
                    ]
                )
            self.assertEqual(2, status)
            self.assertIn('"status": "NO_GO"', stream.getvalue())
            self.assertFalse(pre_output.exists())
            self.assertFalse(request_output.exists())

    def test_every_direct_payload_artifact_must_be_declared_exactly_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, _payload, _support = self._basic_external_input(root)
            document = json.loads(declaration.read_text(encoding="utf-8"))
            unrelated = root / "unrelated.txt"
            unrelated.write_text("not payload evidence\n", encoding="utf-8")
            document["artifact_declarations"] = [{"role": "unrelated", **self._descriptor(unrelated)}]
            self._write_json(declaration, document)
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.prepare_external_payload(str(declaration.resolve()), self._sha(declaration))
            self.assertEqual("UNDECLARED_ARTIFACT_REFERENCE", caught.exception.code)

    def test_prepayload_cannot_smuggle_a_receipt_or_rejected_w5_lineage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, payload, _support = self._basic_external_input(root)
            document = json.loads(payload.read_text(encoding="utf-8"))
            document["formal_freeze_receipt"] = {"report_path": "/does/not/matter", "report_hash": "0" * 64}
            self._write_json(payload, document)
            declarations = json.loads(declaration.read_text(encoding="utf-8"))
            declarations["pre_receipt_payload"] = self._descriptor(payload)
            self._write_json(declaration, declarations)
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.prepare_external_payload(str(declaration.resolve()), self._sha(declaration))
            self.assertEqual("CIRCULAR_RECEIPT", caught.exception.code)

            document.pop("formal_freeze_receipt")
            document["rejected_lineage"] = "W5_S10"
            self._write_json(payload, document)
            declarations["pre_receipt_payload"] = self._descriptor(payload)
            self._write_json(declaration, declarations)
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.prepare_external_payload(str(declaration.resolve()), self._sha(declaration))
            self.assertEqual("REJECTED_BSLOSH_LINEAGE", caught.exception.code)

    def test_prepare_copies_exact_external_bytes_readonly_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, payload, support = self._basic_external_input(root)
            prepared = self._manual_prepared(declaration, payload, support)
            pre_output = root / "pre_receipt.json"
            request_output = root / "receipt_request.json"
            result = self.intake.write_pre_receipt_bundle(prepared, pre_output.resolve(), request_output.resolve())
            self.assertEqual("READY_FOR_EXTERNAL_RECEIPT_NOT_FORMAL", result["status"])
            self.assertFalse(result["formal"])
            self.assertEqual(payload.read_bytes(), pre_output.read_bytes())
            self.assertEqual(0o444, stat.S_IMODE(pre_output.stat().st_mode) & 0o777)
            self.assertEqual(0o444, stat.S_IMODE(request_output.stat().st_mode) & 0o777)
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.write_pre_receipt_bundle(prepared, pre_output.resolve(), (root / "other_request.json").resolve())
            self.assertEqual("OUTPUT_EXISTS", caught.exception.code)

    def test_receipt_digest_semantics_match_existing_formal_gate_exactly(self):
        payload = {
            "protocol_id": self.intake.toolchain.FORMAL_PROTOCOL_ID,
            "sim_freeze_id": "TEST-FREEZE",
            "ordinary_value": {"b": 2, "a": 1},
        }
        final = dict(payload)
        final["formal_freeze_receipt"] = {"report_path": "/external/receipt.json", "report_hash": "a" * 64}
        existing_gate_payload = dict(final)
        existing_gate_payload.pop("formal_freeze_receipt")
        self.assertEqual(self.intake._canonical_hash(payload), self.intake.toolchain.canonical_hash(existing_gate_payload))

    def test_finalize_refuses_incomplete_payload_even_with_a_correctly_bound_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, payload, support = self._basic_external_input(root)
            prepared = self._manual_prepared(declaration, payload, support)
            pre_output = root / "pre_receipt.json"
            request_output = root / "receipt_request.json"
            self.intake.write_pre_receipt_bundle(prepared, pre_output.resolve(), request_output.resolve())
            receipt = self._write_external_receipt(root, request_output, pre_output)
            final_output = root / "must_not_be_formal_freeze.json"
            with self.assertRaises(self.intake.FormalFreezeIntakeError) as caught:
                self.intake.finalize_formal_freeze(
                    pre_output.resolve(),
                    self._sha(pre_output),
                    request_output.resolve(),
                    self._sha(request_output),
                    receipt.resolve(),
                    self._sha(receipt),
                    final_output.resolve(),
                )
            self.assertEqual("FORMAL_GATE_NO_GO", caught.exception.code)
            self.assertFalse(final_output.exists())

    def test_workflow_only_writes_final_after_shared_gate_passes_and_attaches_external_receipt(self):
        """The gate is mocked only to exercise output wiring, never as evidence.

        A real incomplete payload is tested above.  Here the mock models an
        already independently completed external evidence chain so we can
        assert that the intake calls the shared gate both before and after the
        no-overwrite write and does not create its own receipt.
        """
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declaration, payload, support = self._basic_external_input(root)
            pre_output = root / "pre_receipt.json"
            request_output = root / "receipt_request.json"
            final_output = root / "formal_freeze.json"
            calls = []

            def gate(value):
                calls.append(value)
                if "formal_freeze_receipt" not in value:
                    return {"status": "FAIL", "errors": ["external formal freeze receipt is missing"]}
                return {"status": "PASS", "errors": [], "freeze_hash": "f" * 64}

            with mock.patch.object(self.intake.toolchain, "validate_formal_freeze", side_effect=gate):
                prepared = self.intake.prepare_external_payload(str(declaration.resolve()), self._sha(declaration))
                self.intake.write_pre_receipt_bundle(prepared, pre_output.resolve(), request_output.resolve())
                receipt = self._write_external_receipt(root, request_output, pre_output)
                result = self.intake.finalize_formal_freeze(
                    pre_output.resolve(),
                    self._sha(pre_output),
                    request_output.resolve(),
                    self._sha(request_output),
                    receipt.resolve(),
                    self._sha(receipt),
                    final_output.resolve(),
                )
            self.assertEqual("PASS", result["status"])
            self.assertTrue(result["formal"])
            self.assertEqual(0o444, stat.S_IMODE(final_output.stat().st_mode) & 0o777)
            final = json.loads(final_output.read_text(encoding="utf-8"))
            self.assertEqual({"report_path": str(receipt.resolve()), "report_hash": self._sha(receipt)}, final["formal_freeze_receipt"])
            self.assertGreaterEqual(len(calls), 3)  # pre-receipt, pre-write, post-write
            self.assertNotIn("formal_freeze_receipt", calls[0])
            self.assertIn("formal_freeze_receipt", calls[-1])

    def test_module_has_no_runtime_launcher_dependency(self):
        source = MODULE_PATH.read_text(encoding="utf-8").lower()
        for forbidden in ("import subprocess", "popen(", "roslaunch", "import rospy", "import gazebo"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
