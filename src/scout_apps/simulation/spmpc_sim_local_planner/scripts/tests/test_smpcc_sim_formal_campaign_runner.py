#!/usr/bin/env python3
"""Offline/mock coverage for the serial formal campaign dispatcher.

These tests never pass the campaign runner's explicit execution flag to a
real adapter and never invoke ROS, Gazebo, shell commands, or a formal row.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "smpcc_sim_formal_campaign_runner.py"


def load_target():
    module_name = "smpcc_sim_formal_campaign_runner_test_target"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load target: {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class FormalCampaignRunnerTest(unittest.TestCase):
    def setUp(self):
        self.runner = load_target()

    @staticmethod
    def _sha(value: str) -> str:
        return hashlib.sha256(value.encode("utf-8")).hexdigest()

    def _master(self):
        """Make a schema-light, fully ordered 88-row object for pure tests."""
        rows = []
        tables = {}
        for stage in self.runner.STAGE_ORDER:
            spec = self.runner.toolchain.STAGES[stage]
            assignments = []
            for block in range(1, 9):
                for position, condition in enumerate(spec["conditions"], start=1):
                    assignment = {
                        "stage": stage,
                        "path_id": spec["path_id"],
                        "container_id": spec["container_id"],
                        "condition_id": condition,
                        "block_id": f"b{block:02d}",
                        "order_position": position,
                    }
                    assignments.append(assignment)
                    row_id = "{}_{}_{}_{}_{}".format(
                        stage,
                        spec["path_id"],
                        spec["container_id"],
                        condition,
                        f"b{block:02d}",
                    )
                    rows.append(dict(assignment, planned_row_id=row_id, formal=True))
            tables[stage] = {"rows": assignments}
        return {"planned_rows": rows, "randomization_tables": tables}

    def _context(self, root: Path, records=()):
        master = self._master()
        schedule = self.runner.frozen_schedule(master)
        ledger = {"ledger_id": "LEDGER-T", "ledger_identity_hash": self._sha("ledger identity")}
        return self.runner.CampaignContext(
            sim_root=root,
            formal_freeze_path=root / "formal_freeze.json",
            formal_freeze_file_hash=self._sha("freeze file"),
            formal_freeze_hash=self._sha("freeze canonical"),
            formal_freeze={},
            formal_master_path=root / "formal_master.json",
            formal_master_file_hash=self._sha("master file"),
            formal_master_hash=self._sha("master identity"),
            formal_master=master,
            ledger=ledger,
            ledger_root=root / "ledger",
            index_path=root / "ledger" / "dataset_index.jsonl",
            schedule=schedule,
            records=tuple(records),
            summary={"status": "PASS", "N_plan": 88, "N_attempt": len(records)},
        )

    def _record(self, context, row_index, attempt=1, outcome="success"):
        row = context.schedule[row_index]
        values = {
            "formal": True,
            "formal_master_hash": context.formal_master_hash,
            "dataset_ledger_id": context.ledger["ledger_id"],
            "dataset_ledger_identity_hash": context.ledger["ledger_identity_hash"],
            "planned_row_id": row["planned_row_id"],
            "stage": row["stage"],
            "condition_id": row["condition_id"],
            "attempt_id": f"{row['planned_row_id']}_r{attempt:02d}",
            "entry_hash": self._sha(f"entry-{row_index}-{attempt}-{outcome}"),
        }
        if outcome == "success":
            return dict(values, failure_class="NONE", method_success=True, method_failure=False)
        if outcome == "method_failure":
            return dict(values, failure_class="METHOD_FAILURE", method_success=False, method_failure=True)
        if outcome == "protocol_failure":
            return dict(values, failure_class="PROTOCOL_FAILURE", method_success=False, method_failure=False)
        if outcome == "acquisition":
            return dict(values, failure_class="INFRASTRUCTURE_ACQUISITION", method_success=False, method_failure=False)
        raise AssertionError(outcome)

    def test_frozen_schedule_uses_table_list_order_not_row_sorting(self):
        master = self._master()
        s1_table = master["randomization_tables"]["SIM-S1_CORE"]["rows"]
        # A deliberate table-list order change is the frozen order; master
        # rows remain in their old order and must not override it.
        s1_table[0], s1_table[1] = s1_table[1], s1_table[0]
        schedule = self.runner.frozen_schedule(master)
        self.assertEqual(s1_table[0]["condition_id"], schedule[0]["condition_id"])
        self.assertEqual(88, len(schedule))
        self.assertEqual("SIM-S2A_SELECTIVITY", schedule[40]["stage"])
        self.assertEqual("SIM-S2B_TRANSFER", schedule[64]["stage"])

    def test_serial_history_refuses_an_out_of_order_formal_record(self):
        with tempfile.TemporaryDirectory() as temporary:
            context = self._context(Path(temporary))
            unsafe = self._record(context, 1)  # frozen row zero was skipped
            with self.assertRaisesRegex(self.runner.CampaignError, "frozen row order"):
                self.runner.validate_serial_history(
                    context.schedule,
                    [unsafe],
                    formal_master_hash=context.formal_master_hash,
                    ledger=context.ledger,
                )

    def test_unresolved_acquisition_holds_the_same_row_for_r02(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = self._context(root)
            record = self._record(first, 0, outcome="acquisition")
            context = self._context(root, [record])
            with self.assertRaisesRegex(self.runner.CampaignError, "requires an immutable retry authorization"):
                self.runner.select_next_dispatch(context)

    def test_terminal_method_failure_advances_without_replacing_the_failed_row(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = self._context(root)
            record = self._record(first, 0, outcome="method_failure")
            context = self._context(root, [record])
            dispatch = self.runner.select_next_dispatch(context)
            self.assertEqual(context.schedule[1]["planned_row_id"], dispatch.row["planned_row_id"])
            self.assertTrue(dispatch.attempt_id.endswith("_r01"))

    def test_s2a_is_no_go_without_s1_immutable_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            empty = self._context(root)
            records = [self._record(empty, index) for index in range(40)]
            context = self._context(root, records)
            with self.assertRaisesRegex(self.runner.CampaignError, "requires immutable frozen stage-entry evidence"):
                self.runner.select_next_dispatch(context)

    def _write_readonly_json(self, path: Path, value) -> None:
        path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
        path.chmod(0o444)

    def test_s2a_accepts_exact_readonly_s1_evidence_anchored_to_s1_head(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            empty = self._context(root)
            records = [self._record(empty, index) for index in range(40)]
            context = self._context(root, records)
            s1_head = records[-1]["entry_hash"]
            closure = root / "s1_closure.json"
            extension = root / "s1_extension.json"
            self._write_readonly_json(closure, {"dataset_index_head_hash": s1_head})
            self._write_readonly_json(extension, {"dataset_index_head_hash": s1_head})
            evidence = {
                "stage1_closure": {"report_path": str(closure), "report_hash": self.runner.toolchain.sha256_file(closure)},
                "stage1_extension_gate": {"report_path": str(extension), "report_hash": self.runner.toolchain.sha256_file(extension)},
            }
            evidence_path = root / "s2a_evidence.json"
            self._write_readonly_json(evidence_path, evidence)
            with mock.patch.object(self.runner.toolchain, "validate_stage_entry", return_value=None) as stage_gate:
                dispatch = self.runner.select_next_dispatch(context, stage_entry_evidence_path=evidence_path)
            self.assertEqual("SIM-S2A_SELECTIVITY", dispatch.row["stage"])
            self.assertEqual(evidence, dispatch.stage_entry_evidence)
            stage_gate.assert_called_once()

    def test_s2a_rejects_evidence_not_anchored_to_final_s1_head(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            empty = self._context(root)
            records = [self._record(empty, index) for index in range(40)]
            context = self._context(root, records)
            closure = root / "s1_closure.json"
            extension = root / "s1_extension.json"
            self._write_readonly_json(closure, {"dataset_index_head_hash": self._sha("stale")})
            self._write_readonly_json(extension, {"dataset_index_head_hash": self._sha("stale")})
            evidence = {
                "stage1_closure": {"report_path": str(closure), "report_hash": self.runner.toolchain.sha256_file(closure)},
                "stage1_extension_gate": {"report_path": str(extension), "report_hash": self.runner.toolchain.sha256_file(extension)},
            }
            evidence_path = root / "s2a_evidence.json"
            self._write_readonly_json(evidence_path, evidence)
            with self.assertRaisesRegex(self.runner.CampaignError, "terminal ledger head"):
                self.runner.select_next_dispatch(context, stage_entry_evidence_path=evidence_path)

    def test_s2b_requires_the_full_s1_s2a_selectivity_trigger_chain(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            empty = self._context(root)
            records = [self._record(empty, index) for index in range(64)]
            context = self._context(root, records)
            with self.assertRaisesRegex(self.runner.CampaignError, "requires immutable frozen stage-entry evidence"):
                self.runner.select_next_dispatch(context)

            s1_head, s2a_head = records[39]["entry_hash"], records[63]["entry_hash"]
            report_paths = {}
            for key, head in (
                ("stage1_closure", s1_head),
                ("stage1_extension_gate", s1_head),
                ("stage2a_completion", s2a_head),
                ("s2a_selectivity", s2a_head),
                ("stage2b_trigger", s2a_head),
            ):
                report_path = root / f"{key}.json"
                self._write_readonly_json(report_path, {"dataset_index_head_hash": head})
                report_paths[key] = {
                    "report_path": str(report_path),
                    "report_hash": self.runner.toolchain.sha256_file(report_path),
                }
            evidence_path = root / "s2b_evidence.json"
            self._write_readonly_json(evidence_path, report_paths)
            with mock.patch.object(self.runner.toolchain, "validate_stage_entry", return_value=None) as stage_gate:
                dispatch = self.runner.select_next_dispatch(context, stage_entry_evidence_path=evidence_path)
            self.assertEqual("SIM-S2B_TRANSFER", dispatch.row["stage"])
            stage_gate.assert_called_once()

    def test_no_explicit_execution_only_prepares_and_never_calls_adapter_execute(self):
        with tempfile.TemporaryDirectory() as temporary:
            context = self._context(Path(temporary))
            dispatch = self.runner.CampaignDispatch(
                row=context.schedule[0],
                attempt_id=context.schedule[0]["planned_row_id"] + "_r01",
                frozen_order_index=0,
                history=self.runner.SerialHistory(0, 1, frozenset(), {}),
                stage_entry_evidence=None,
                stage_entry_evidence_path=None,
                retry_authorization_path=None,
            )
            preparation = object()
            adapter_report = {"status": "FORMAL_ROW_PREPARED_NOT_EXECUTED"}
            with mock.patch.object(self.runner, "load_campaign_context", return_value=context), mock.patch.object(
                self.runner, "prepare_campaign_row", return_value=(dispatch, preparation)
            ), mock.patch.object(
                self.runner.adapter, "formal_row_preparation_report", return_value=adapter_report
            ), mock.patch.object(self.runner.adapter, "execute_prepared_formal_row") as execute:
                report = self.runner.execute_campaign_row(
                    formal_freeze_path=context.formal_freeze_path,
                    formal_master_path=context.formal_master_path,
                    authorize_execution=False,
                )
            self.assertEqual("FORMAL_CAMPAIGN_ROW_PREPARED_NOT_EXECUTED", report["status"])
            execute.assert_not_called()

    def test_explicit_execution_dispatches_exactly_one_mocked_row(self):
        with tempfile.TemporaryDirectory() as temporary:
            context = self._context(Path(temporary))
            dispatch = self.runner.CampaignDispatch(
                row=context.schedule[0],
                attempt_id=context.schedule[0]["planned_row_id"] + "_r01",
                frozen_order_index=0,
                history=self.runner.SerialHistory(0, 1, frozenset(), {}),
                stage_entry_evidence=None,
                stage_entry_evidence_path=None,
                retry_authorization_path=None,
            )
            preparation = object()
            appended = {"attempt_id": dispatch.attempt_id}
            with mock.patch.object(self.runner, "load_campaign_context", side_effect=[context, context]), mock.patch.object(
                self.runner, "prepare_campaign_row", return_value=(dispatch, preparation)
            ), mock.patch.object(
                self.runner.adapter, "execute_prepared_formal_row", return_value={"status": "PASS"}
            ) as execute, mock.patch.object(self.runner.toolchain, "load_dataset_index", return_value=[appended]):
                report = self.runner.execute_campaign_row(
                    formal_freeze_path=context.formal_freeze_path,
                    formal_master_path=context.formal_master_path,
                    authorize_execution=True,
                )
            self.assertEqual("FORMAL_CAMPAIGN_ROW_EXECUTED", report["status"])
            execute.assert_called_once_with(preparation, authorize_execution=True)
            self.assertTrue((context.ledger_root / self.runner.LOCK_FILENAME).is_file())

    def test_campaign_lock_refuses_a_second_concurrent_dispatcher(self):
        with tempfile.TemporaryDirectory() as temporary:
            context = self._context(Path(temporary))
            with self.runner.campaign_lock(context):
                with self.assertRaisesRegex(self.runner.CampaignError, "campaign lock is held"):
                    with self.runner.campaign_lock(context):
                        pass

    def test_cli_missing_formal_inputs_returns_no_go_without_execution(self):
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            code = self.runner.main(
                [
                    "--formal-freeze",
                    "/definitely/missing/formal_freeze.json",
                    "--formal-master",
                    "/definitely/missing/formal_master.json",
                ]
            )
        self.assertEqual(2, code)
        report = json.loads(captured.getvalue())
        self.assertEqual("NO_GO", report["status"])


if __name__ == "__main__":
    unittest.main()
