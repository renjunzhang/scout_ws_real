#!/usr/bin/env python3
"""Unit tests for the R8 source-freeze/master/GO release entry point."""

from __future__ import annotations

import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1]
MODULE_PATH = SCRIPT_DIR / "smpcc_sim_r8_release.py"
SPEC = importlib.util.spec_from_file_location("smpcc_sim_r8_release_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)


class R8ReleaseTest(unittest.TestCase):
    @staticmethod
    def _write_json(path: Path, value: object, *, readonly: bool = False) -> None:
        path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
        if readonly:
            os.chmod(path, 0o444)

    def _documents(self, root: Path) -> tuple[Path, Path]:
        gate = release.source_gate
        binding = gate.make_source_separation_binding()
        freeze = {
            "release_id": gate.SOURCE_SEPARATED_RELEASE_ID,
            "source_separation": binding,
        }
        master = {
            "release_id": gate.SOURCE_SEPARATED_RELEASE_ID,
            "source_separation_hash": gate.canonical_hash(binding),
            "execution_artifact_registry_hash": binding[
                "execution_artifact_registry_hash"
            ],
            "master_hash": "b" * 64,
        }
        evidence_root = root / "evidence"
        evidence_root.mkdir()
        checks = {}
        for name in sorted(gate.R8_GO_CHECKS):
            expected = gate._expected_go_check(name)
            evidence = evidence_root / f"{name}.log"
            text = "returncode=0\n"
            if expected["test_count"]:
                text += f"Ran {expected['test_count']} tests in 0.001s\n\nOK\n"
            else:
                text += "sim package build passed\n"
            evidence.write_text(text, encoding="utf-8")
            os.chmod(evidence, 0o444)
            checks[name] = {
                "status": "PASS",
                "command": expected["command"],
                "returncode": 0,
                "test_count": expected["test_count"],
                "test_source": expected["test_source"],
                "evidence": {"path": str(evidence), "sha256": gate.sha256_file(evidence)},
            }
        receipt = gate.build_r8_go_receipt(freeze, master, checks)
        receipt_path = root / "go_receipt.json"
        self._write_json(receipt_path, receipt, readonly=True)
        freeze["source_separation_go_receipt"] = {
            "path": str(receipt_path),
            "sha256": gate.sha256_file(receipt_path),
        }
        master["source_separation_go_receipt_hash"] = receipt["go_receipt_hash"]
        freeze_path = root / "freeze.json"
        master_path = root / "master.json"
        self._write_json(freeze_path, freeze, readonly=True)
        self._write_json(master_path, master, readonly=True)
        return freeze_path, master_path

    def test_gate_accepts_a_complete_readonly_r8_source_evidence_chain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            report = release.gate(freeze, master)
            self.assertEqual("PASS_R8_SOURCE_SEPARATION_ONLY_NOT_MATRIX_GO", report["status"])
            self.assertFalse(report["matrix_execution_authorized"])
            self.assertIn("independent liquid-plant", report["no_go_remaining"][1])

    def test_gate_rejects_missing_receipt_or_writable_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            os.chmod(freeze, 0o644)
            with self.assertRaisesRegex(release.R8ReleaseError, "read-only"):
                release.gate(freeze, master)

        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            os.chmod(freeze, 0o644)
            document = json.loads(freeze.read_text(encoding="utf-8"))
            document.pop("source_separation_go_receipt")
            self._write_json(freeze, document, readonly=True)
            with self.assertRaisesRegex(release.R8ReleaseError, "GO receipt"):
                release.gate(freeze, master)

    def test_cli_returns_nonzero_for_missing_freeze_master(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.assertEqual(
                2,
                release.main(
                    [
                        "gate",
                        "--freeze",
                        str(root / "missing_freeze.json"),
                        "--master",
                        str(root / "missing_master.json"),
                    ]
                ),
            )

    def test_create_go_cli_returns_nonzero_for_missing_pre_go_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.assertEqual(
                2,
                release.main(
                    [
                        "create-go-receipt",
                        "--freeze",
                        str(root / "missing_freeze.json"),
                        "--master",
                        str(root / "missing_master.json"),
                        "--output",
                        str(root / "must_not_exist.json"),
                    ]
                ),
            )
            self.assertFalse((root / "must_not_exist.json").exists())


if __name__ == "__main__":
    unittest.main()
