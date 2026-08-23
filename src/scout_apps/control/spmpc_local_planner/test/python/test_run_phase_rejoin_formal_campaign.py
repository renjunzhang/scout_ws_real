#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


TOOL = Path(__file__).resolve().parents[2] / \
    "tools/simulation/run_phase_rejoin_formal_campaign.py"
SPEC = importlib.util.spec_from_file_location("formal_campaign", TOOL)
FORMAL = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(FORMAL)


class FormalCampaignContractTest(unittest.TestCase):

    @staticmethod
    def _write_order(path, changed_block=None):
        lines = ["schema,block,seed,position,condition"]
        for block, seed in enumerate(FORMAL.FORMAL_SEEDS, start=1):
            for position, condition in enumerate(
                    FORMAL.REQUIRED_CONDITIONS, start=1):
                written_block = (
                    changed_block
                    if changed_block is not None and
                    block == 1 and position == 1
                    else block)
                lines.append(
                    "spmpc_phase_rejoin_formal_order_v1,{},{},{},{}".format(
                        written_block, seed, position, condition))
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def test_matching_readiness_is_accepted(self):
        session = Path("/tmp/frozen_session.yaml")
        digest = "a" * 64
        FORMAL.validate_readiness({
            "schema": FORMAL.READINESS_SCHEMA,
            "status": "READY_NOT_EXECUTED",
            "formal_trials_started": False,
            "reasons": [],
            "session": str(session),
            "session_sha256": digest,
        }, session, digest)

    def test_readiness_with_started_trials_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "does not authorize"):
            FORMAL.validate_readiness({
                "schema": FORMAL.READINESS_SCHEMA,
                "status": "READY_NOT_EXECUTED",
                "formal_trials_started": True,
                "reasons": [],
                "session": "/tmp/frozen_session.yaml",
                "session_sha256": "a" * 64,
            }, Path("/tmp/frozen_session.yaml"), "a" * 64)

    def test_human_approval_binds_exact_session_and_seeds(self):
        digest = "b" * 64
        approval = {
            "schema": FORMAL.APPROVAL_SCHEMA,
            "approved": True,
            "session_sha256": digest,
            "formal_seeds_authorized": FORMAL.FORMAL_SEEDS,
            "reviewer": "human-reviewer",
            "approved_at": "2026-08-23T18:00:00+08:00",
        }
        FORMAL.validate_approval(approval, digest)
        changed = dict(approval)
        changed["session_sha256"] = "c" * 64
        with self.assertRaisesRegex(RuntimeError, "absent or mismatched"):
            FORMAL.validate_approval(changed, digest)

    def test_no_implicit_human_approval(self):
        with self.assertRaisesRegex(RuntimeError, "absent or mismatched"):
            FORMAL.validate_approval({}, "d" * 64)

    def test_formal_order_requires_exact_block_seed_position_sequence(self):
        with tempfile.TemporaryDirectory() as temporary:
            order = Path(temporary) / "order.csv"
            self._write_order(order)
            self.assertEqual(len(FORMAL._formal_order(order)), 96)
            self._write_order(order, changed_block=2)
            with self.assertRaisesRegex(RuntimeError, "sequence is invalid"):
                FORMAL._formal_order(order)

    @staticmethod
    def _write_trial_outputs(root, status, task_success):
        cycle = root / "cycles.csv"
        summary = root / "summary.json"
        cycle.write_text("schema,cycle_id\ncycle_v3,0\n", encoding="utf-8")
        summary.write_text(json.dumps({
            "schema": FORMAL.SUMMARY_SCHEMA,
            "status": status,
            "seed": 3101,
            "condition_id": "C4",
            "formal_trials_started": True,
            "development_pilot_only": False,
            "frozen_session_sha256": "e" * 64,
            "trial": {
                "sequence_completed": status != "RUNTIME_ERROR",
                "task_success": task_success,
                "runtime_error": "solver failed"
                    if status == "RUNTIME_ERROR" else "",
            },
        }), encoding="utf-8")
        return cycle, summary

    def test_nonzero_runtime_error_summary_is_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle, summary = self._write_trial_outputs(
                root, "RUNTIME_ERROR", False)
            record = FORMAL._trial_record(
                {"block": 1, "seed": 3101, "position": 1,
                 "condition": "C4"},
                cycle, summary, "e" * 64, process_returncode=3)
            self.assertEqual(record["status"], "RUNTIME_ERROR")
            self.assertFalse(record["task_success"])
            self.assertEqual(record["process_returncode"], 3)

    def test_complete_with_failure_summary_is_retained(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle, summary = self._write_trial_outputs(
                root, "TRIAL_COMPLETE_WITH_FAILURE", False)
            record = FORMAL._trial_record(
                {"block": 1, "seed": 3101, "position": 1,
                 "condition": "C4"},
                cycle, summary, "e" * 64, process_returncode=0)
            self.assertEqual(
                record["status"], "TRIAL_COMPLETE_WITH_FAILURE")
            self.assertFalse(record["task_success"])
            self.assertEqual(record["process_returncode"], 0)

    def test_nonzero_complete_with_failure_summary_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle, summary = self._write_trial_outputs(
                root, "TRIAL_COMPLETE_WITH_FAILURE", False)
            with self.assertRaisesRegex(
                    FORMAL.TrialInfrastructureFailure,
                    "failed without a valid summary"):
                FORMAL._trial_record(
                    {"block": 1, "seed": 3101, "position": 1,
                     "condition": "C4"},
                    cycle, summary, "e" * 64, process_returncode=7)

    def test_nonzero_exit_without_valid_summary_is_infrastructure_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            cycle = root / "cycles.csv"
            cycle.write_text("schema,cycle_id\n", encoding="utf-8")
            with self.assertRaisesRegex(
                    FORMAL.TrialInfrastructureFailure,
                    "no readable summary"):
                FORMAL._trial_record(
                    {"block": 1, "seed": 3101, "position": 1,
                     "condition": "C4"},
                    cycle, root / "missing.json", "e" * 64,
                    process_returncode=3)

    def test_missing_approval_leaves_no_formal_output(self):
        fake_auditor = SimpleNamespace(
            require_formal_session_schema=lambda session: None,
            audit_formal_session=lambda session, path: ([], {}),
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = root / "session.yaml"
            readiness = root / "readiness.json"
            approval = root / "approval.json"
            output = root / "formal_output"
            session.write_text("schema: test\n", encoding="utf-8")
            digest = FORMAL._sha256_file(session)
            readiness.write_text(json.dumps({
                "schema": FORMAL.READINESS_SCHEMA,
                "status": "READY_NOT_EXECUTED",
                "formal_trials_started": False,
                "reasons": [],
                "session": str(session.resolve()),
                "session_sha256": digest,
            }), encoding="utf-8")
            approval.write_text("{}\n", encoding="utf-8")
            with mock.patch.object(FORMAL, "_load_auditor",
                                   return_value=fake_auditor), \
                    mock.patch.object(FORMAL, "ALLOWED_OUTPUT_ROOT", root):
                with self.assertRaisesRegex(
                        RuntimeError, "absent or mismatched"):
                    FORMAL.execute(session, readiness, approval, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
