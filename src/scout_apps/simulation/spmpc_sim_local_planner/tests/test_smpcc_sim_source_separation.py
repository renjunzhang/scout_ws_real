#!/usr/bin/env python3
"""Fail-closed R8 source/binary/model/codegen execution-identity tests."""

from __future__ import annotations

import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/smpcc_sim_source_separation.py"
SPEC = importlib.util.spec_from_file_location("smpcc_sim_source_separation_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


class SourceSeparationExecutionGateTest(unittest.TestCase):
    @staticmethod
    def _sha(path: Path) -> str:
        return gate.sha256_file(path)

    def _documents(self, root: Path) -> tuple[dict, dict]:
        root = root / f"receipt_{len(list(root.iterdir())):02d}"
        root.mkdir()
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
            "master_hash": "a" * 64,
        }
        checks = {}
        for name in sorted(gate.R8_GO_CHECKS):
            expected = gate._expected_go_check(name)
            evidence = root / f"{name}.log"
            evidence.write_text(
                "returncode=0\n"
                f"Ran {expected['test_count']} tests in 0.001s\n\n"
                "OK\n",
                encoding="utf-8",
            )
            os.chmod(evidence, 0o444)
            checks[name] = {
                "status": "PASS",
                "command": expected["command"],
                "returncode": 0,
                "test_count": expected["test_count"],
                "test_source": expected["test_source"],
                "evidence": {"path": str(evidence), "sha256": self._sha(evidence)},
            }
        receipt = gate.build_r8_go_receipt(freeze, master, checks)
        receipt_path = root / "r8_go_receipt.json"
        receipt_path.write_text(
            json.dumps(receipt, sort_keys=True), encoding="utf-8"
        )
        os.chmod(receipt_path, 0o444)
        freeze["source_separation_go_receipt"] = {
            "path": str(receipt_path),
            "sha256": self._sha(receipt_path),
        }
        master["source_separation_go_receipt_hash"] = receipt["go_receipt_hash"]
        return freeze, master

    def test_valid_r8_cross_binding_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            accepted = gate.require_execution_identity(freeze, master)
            self.assertEqual(gate.SOURCE_SEPARATED_TARGET_ID, accepted["target_id"])
            self.assertEqual(
                gate.SIM_PACKAGE,
                accepted["execution_artifact_registry"]["controller_package"],
            )

    def test_r7_or_missing_cross_binding_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            freeze["release_id"] = "SIM-MECHANISM-40-64-88-R7"
            with self.assertRaisesRegex(gate.SourceSeparationError, "R8"):
                gate.require_execution_identity(freeze, master)
            freeze, master = self._documents(Path(temporary))
            master["source_separation_hash"] = "0" * 64
            with self.assertRaisesRegex(gate.SourceSeparationError, "cross-bind"):
                gate.require_execution_identity(freeze, master)

    def test_tampered_source_model_codegen_or_binary_registry_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            registry = freeze["source_separation"]["execution_artifact_registry"]
            registry["trees"]["controller_source"]["files"][
                next(iter(registry["trees"]["controller_source"]["files"]))
            ] = "0" * 64
            with self.assertRaisesRegex(gate.SourceSeparationError, "registry"):
                gate.require_execution_identity(freeze, master)

            freeze, master = self._documents(Path(temporary))
            registry = freeze["source_separation"]["execution_artifact_registry"]
            registry["trees"]["generated_solver_codegen"]["files"][
                next(iter(registry["trees"]["generated_solver_codegen"]["files"]))
            ] = "0" * 64
            with self.assertRaisesRegex(gate.SourceSeparationError, "registry"):
                gate.require_execution_identity(freeze, master)

            freeze, master = self._documents(Path(temporary))
            registry = freeze["source_separation"]["execution_artifact_registry"]
            registry["binaries"]["controller_library"]["sha256"] = "0" * 64
            with self.assertRaisesRegex(gate.SourceSeparationError, "registry"):
                gate.require_execution_identity(freeze, master)

    def test_real_controller_or_experiment_path_cannot_enter_artifact_registry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            registry = freeze["source_separation"]["execution_artifact_registry"]
            registry["binaries"]["controller_node"]["path"] = (
                "/home/a/scout_ws/devel/lib/spmpc_local_planner/spmpc_local_planner_node"
            )
            with self.assertRaisesRegex(gate.SourceSeparationError, "registry"):
                gate.require_execution_identity(freeze, master)

            self.assertTrue(
                gate._contains_forbidden_component(
                    Path("/home/a/scout_ws/src/scout_apps/control/slosh_models/launch/slosh_monitor.launch")
                )
            )

            freeze, master = self._documents(Path(temporary))
            registry = freeze["source_separation"]["execution_artifact_registry"]
            path_key = next(iter(registry["trees"]["runtime_tooling_and_tests"]["files"]))
            registry["trees"]["runtime_tooling_and_tests"]["files"].pop(path_key)
            registry["trees"]["runtime_tooling_and_tests"]["files"][
                "../control/spmpc_experiments/gtest/fake.py"
            ] = "0" * 64
            with self.assertRaisesRegex(gate.SourceSeparationError, "registry"):
                gate.require_execution_identity(freeze, master)

    def test_go_receipt_and_master_cross_bind_are_required(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            freeze, master = self._documents(Path(temporary))
            master.pop("source_separation_go_receipt_hash")
            with self.assertRaisesRegex(gate.SourceSeparationError, "GO receipt"):
                gate.require_execution_identity(freeze, master)

            freeze, master = self._documents(Path(temporary))
            receipt_path = Path(freeze["source_separation_go_receipt"]["path"])
            os.chmod(receipt_path, 0o644)
            with self.assertRaisesRegex(gate.SourceSeparationError, "read-only"):
                gate.require_execution_identity(freeze, master)


if __name__ == "__main__":
    unittest.main()
