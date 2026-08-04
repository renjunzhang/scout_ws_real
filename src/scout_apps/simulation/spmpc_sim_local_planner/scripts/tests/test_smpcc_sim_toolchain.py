#!/usr/bin/env python3
"""stdlib tests for the fail-closed SMPCC-SIM matrix toolchain.

These tests deliberately exercise the protocol layer only.  The one runner
test uses its explicit dry-run lifecycle and mocks the reachability probe, so
it neither starts nor contacts ROS/Gazebo.
"""

from __future__ import annotations

import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import math
import stat
import sys
import tempfile
import unittest
from collections import Counter, defaultdict
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "smpcc_sim_toolchain.py"


def load_toolchain():
    """Load the script directly, without requiring a ROS package install."""
    module_name = "smpcc_sim_toolchain_test_target"
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot dynamically import smpcc_sim_toolchain.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class SmpccSimToolchainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tool = load_toolchain()

    @staticmethod
    def _sha(text: str) -> str:
        return hashlib.sha256(text.encode("utf-8")).hexdigest()

    @staticmethod
    def _write_json(path: Path, value) -> None:
        path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True), encoding="utf-8")

    def _assert_freeze_no_go(self, report, *error_terms: str) -> None:
        """Require the intended fail-closed reason, not merely another missing field."""
        self.assertEqual("FAIL", report["status"], report)
        self.assertTrue(
            any(all(term.lower() in error.lower() for term in error_terms) for error in report["errors"]),
            report["errors"],
        )

    def _formal_freeze_probe(self, root: Path, manifest_release_id: str = "BSLOSH-FINAL-R1", include_w5: bool = False):
        """Small formal shell whose release artifact is real and hash-bound.

        It deliberately leaves unrelated formal prerequisites absent.  Each
        test asserts the specific release/classifier/registry NO-GO reason,
        preventing an unrelated missing freeze field from satisfying it.
        """
        release_id = "BSLOSH-FINAL-R1"
        bslosh_config = self._effective_config("Bslosh", v_ref=0.20)
        bslosh_config["w_slosh"] = 2.0
        bslosh_config_path = root / "Bslosh_effective_config.json"
        self._write_json(bslosh_config_path, bslosh_config)
        bslosh_hashes = {
            "effective_config_hash": self.tool.canonical_hash(bslosh_config),
            "observer_policy_hash": self.tool.canonical_hash(bslosh_config["observer"]),
            "delay_policy_hash": self.tool.canonical_hash(bslosh_config["delay"]),
        }
        release_manifest = {
            "report_type": "SMPCC_FORMAL_BSLOSH_RELEASE",
            "protocol_id": self.tool.FORMAL_PROTOCOL_ID,
            "status": "FORMAL_SELECTED",
            "release_id": manifest_release_id,
            "condition_id": "Bslosh",
            "real_freeze_id": "REAL-FREEZE-R1",
            "sim_freeze_id": "SIM-FREEZE-R1",
            "git_revision": "0123456789abcdef",
            "build_id": "build-r1",
            **bslosh_hashes,
            "source_specific_release": True,
            "source_specific_release_revision": "source-final-r1",
            "source_selection_report_hash": self._sha("source selection"),
            "final_candidate_report_hash": self._sha("final candidate"),
            "efficacy_report_hash": self._sha("efficacy"),
            "trajectory_replay_report_hash": self._sha("trajectory replay"),
            "comparator_fairness_report_hash": self._sha("comparator fairness"),
            "fallback_policy_hash": self._sha("fallback policy"),
            "formal_release_validator_hash": self._sha("release validator"),
        }
        if include_w5:
            release_manifest["rejected_lineage"] = "W5_S10"
        release_path = root / "formal_bslosh_release_manifest.json"
        self._write_json(release_path, release_manifest)
        return {
            "protocol_id": self.tool.FORMAL_PROTOCOL_ID,
            "sample_size": 8,
            "real_freeze_id": "REAL-FREEZE-R1",
            "sim_freeze_id": "SIM-FREEZE-R1",
            "git_revision": "0123456789abcdef",
            "build_id": "build-r1",
            "formal_bslosh_release": {
                "status": "FORMAL_SELECTED",
                "release_id": release_id,
                **bslosh_hashes,
                "release_manifest_path": str(release_path),
                "release_manifest_hash": self.tool.sha256_file(release_path),
            },
            # This valid Bslosh config causes formal-freeze validation to parse
            # the release manifest even though other freeze prerequisites are
            # intentionally absent from this focused negative fixture.
            "effective_configs": {
                "Bslosh": {
                    "effective_config": bslosh_config,
                    "effective_config_hash": bslosh_hashes["effective_config_hash"],
                    "effective_config_path": str(bslosh_config_path),
                    "effective_config_file_hash": self.tool.sha256_file(bslosh_config_path),
                }
            },
        }

    def _frozen_retry_classifier(self, root: Path, condition_blind: bool = True):
        """Write a self-describing classifier plus verifier for formal-freeze tests."""
        verifier_path = root / "retry_classifier_verifier.py"
        verifier_path.write_text("# immutable retry classifier verifier fixture\n", encoding="utf-8")
        verifier_path.chmod(0o555)
        verifier_command = [str(verifier_path.resolve())]
        rules = {
            "input_fields": ["preflight_endpoint_state", "recorder_start_state"],
            "decision_logic": "only pre-motion, pre-assignment infrastructure acquisition is retryable",
        }
        semantics = {
            "classifier_id": "SIM-RETRY-CLASSIFIER-R1",
            "classifier_rule_hash": self.tool.canonical_hash(rules),
            "rules": rules,
            "verifier_id": "SIM-RETRY-VERIFIER-R1",
            "condition_blind": condition_blind,
            "pre_motion_only": True,
            "retryable_failure_classes": ["INFRASTRUCTURE_ACQUISITION"],
            "nonretryable_failure_classes": ["METHOD_FAILURE", "PROTOCOL_FAILURE"],
            "max_retries_per_row": 1,
            "reason_codes": ["PRE_MOTION_ROS_GAZEBO_ACQUISITION"],
            "missingness_rule_hash": self._sha("missingness rule"),
            "stop_resume_rule_hash": self._sha("stop resume rule"),
        }
        classifier_manifest = {
            "document_type": "SMPCC_SIM_RETRY_CLASSIFIER",
            "protocol_id": self.tool.FORMAL_PROTOCOL_ID,
            "status": "FROZEN",
            "verifier_path": str(verifier_path),
            "verifier_hash": self.tool.sha256_file(verifier_path),
            "verifier_command": verifier_command,
            **semantics,
        }
        classifier_manifest_path = root / "retry_classifier_manifest.json"
        self._write_json(classifier_manifest_path, classifier_manifest)
        return {
            "classifier_manifest_path": str(classifier_manifest_path),
            "classifier_manifest_hash": self.tool.sha256_file(classifier_manifest_path),
            "verifier_path": str(verifier_path),
            "verifier_hash": self.tool.sha256_file(verifier_path),
            "verifier_command": verifier_command,
            **semantics,
        }

    def _frozen_stage_entry_policy(self, root: Path):
        """Create the minimum hash-bound policy needed by stage-entry tests."""
        reports = {}
        for report_type in (
            "SIM_S1_CLOSURE",
            self.tool.STAGE1_EXTENSION_REPORT_TYPE,
            "SIM_S2A_CLOSURE",
            self.tool.STAGE2A_SELECTIVITY_REPORT_TYPE,
            self.tool.STAGE2B_TRIGGER_REPORT_TYPE,
        ):
            reports[report_type] = {
                "report_type": report_type,
                "rule_hash": self._sha(report_type + " rule"),
                "validator_hash": self._sha(report_type + " validator"),
            }
        reports[self.tool.STAGE1_EXTENSION_REPORT_TYPE]["required_gate_ids"] = sorted(self.tool.REQUIRED_S1_EXTENSION_GATES)
        reports[self.tool.STAGE2A_SELECTIVITY_REPORT_TYPE]["allowed_selectivity_statuses"] = ["SUPPORTED"]
        policy = {
            "document_type": self.tool.STAGE_ENTRY_POLICY_DOCUMENT_TYPE,
            "protocol_id": self.tool.FORMAL_PROTOCOL_ID,
            "status": "FROZEN",
            "policy_id": "SIM-STAGE-ENTRY-POLICY-R1",
            "reports": reports,
        }
        policy["policy_hash"] = self.tool.canonical_hash(policy)
        path = root / "stage_entry_policy.json"
        self._write_json(path, policy)
        return {
            "policy_path": str(path),
            "policy_file_hash": self.tool.sha256_file(path),
            "policy_id": policy["policy_id"],
            "policy_hash": policy["policy_hash"],
        }, policy

    def _stage_entry_master_fixture(self, root: Path):
        """Make an append-only ledger prefix plus a formal-master shell."""
        policy_entry, policy = self._frozen_stage_entry_policy(root)
        ledger_root = root / "formal_ledger"
        ledger_root.mkdir()
        ledger = {
            "ledger_id": "SIM-LEDGER-R1",
            "ledger_root": str(ledger_root),
            "protocol_id": self.tool.FORMAL_PROTOCOL_ID,
            "sim_freeze_id": "SIM-FREEZE-R1",
            "ledger_policy_hash": self._sha("ledger policy"),
        }
        identity = {
            "ledger_id": ledger["ledger_id"],
            "ledger_root": str(ledger_root.resolve()),
            "protocol_id": ledger["protocol_id"],
            "sim_freeze_id": ledger["sim_freeze_id"],
            "ledger_policy_hash": ledger["ledger_policy_hash"],
        }
        ledger["ledger_identity_hash"] = self.tool.canonical_hash(identity)
        freeze = {
            "sim_freeze_id": "SIM-FREEZE-R1",
            "dataset_ledger": ledger,
            "stage_entry_policy": policy_entry,
        }
        freeze_path = root / "formal_freeze_for_stage_entry.json"
        self._write_json(freeze_path, freeze)
        registry = self._fixture_master()["contrast_registry"]
        master = {
            "master_hash": self._sha("stage-entry-master"),
            "freeze_hash": self.tool.canonical_hash(freeze),
            "formal_freeze_path": str(freeze_path),
            "formal_freeze_file_hash": self.tool.sha256_file(freeze_path),
            "contrast_registry": registry,
        }
        index = ledger_root / "dataset_index.jsonl"
        entry = self.tool.append_dataset_index(
            index,
            {
                "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01",
                "planned_row_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01",
                "failure_class": "NONE",
                "method_failure": False,
                "method_success": True,
            },
        )
        return master, freeze, policy, index, entry["entry_hash"]

    def _rehash_master(self, master):
        core = dict(master)
        core.pop("master_hash", None)
        master["master_hash"] = self.tool.canonical_hash(core)
        return master

    def _fixture_master(self):
        return self.tool.make_master_rows(
            {
                "fixture": True,
                "mode": "fixture",
                "protocol_id": "SIM-FIXTURE-40-64-88-test",
                "sample_size": 8,
            },
            "test-randomization-seed",
            fixture=True,
        )

    def _effective_config(self, condition_id: str, v_ref: float = 0.20):
        return {
            "condition_id": condition_id,
            "w_control": 0.3,
            "w_smooth": 1.0,
            "w_alpha": 1.1,
            "w_du_a": 1.2,
            "w_du_vs": 1.3,
            "w_slosh": 0.0,
            "v_ref": v_ref,
            "observer": {"source": "odom", "model": "nominal"},
            "delay": {"mode": "off", "linear_sec": -1.0, "angular_sec": -1.0},
        }

    def test_fixture_master_has_40_24_24_88_fixed_denominators_and_balance(self):
        master = self._fixture_master()
        report = self.tool.validate_master(master)

        self.assertEqual("PASS", report["status"])
        self.assertFalse(master["formal"])
        self.assertEqual(self.tool.FIXTURE_EVIDENCE_CLASS, master["evidence_class"])
        self.assertEqual(
            {"SIM-S1_CORE": 40, "SIM-S2A_SELECTIVITY": 24, "SIM-S2B_TRANSFER": 24},
            master["counts"]["by_stage"],
        )
        self.assertEqual(88, master["counts"]["total"])

        by_stage = Counter(row["stage"] for row in master["planned_rows"])
        self.assertEqual(Counter({"SIM-S1_CORE": 40, "SIM-S2A_SELECTIVITY": 24, "SIM-S2B_TRANSFER": 24}), by_stage)
        self.assertEqual(88, len({row["planned_row_id"] for row in master["planned_rows"]}))

        for stage, specification in self.tool.STAGES.items():
            rows = [row for row in master["planned_rows"] if row["stage"] == stage]
            self.assertEqual(len(specification["conditions"]) * 8, len(rows))
            positions = defaultdict(list)
            for row in rows:
                self.assertEqual(8, row["fixed_denominator"]["n_plan_condition"])
                self.assertEqual(len(rows), row["fixed_denominator"]["n_plan_stage"])
                self.assertEqual(88, row["fixed_denominator"]["n_plan_total"])
                positions[row["condition_id"]].append(row["order_position"])
            for condition in specification["conditions"]:
                counts = [positions[condition].count(position) for position in range(1, len(specification["conditions"]) + 1)]
                self.assertLessEqual(max(counts) - min(counts), 1, (stage, condition, counts))

        formal_only = self.tool.validate_master(master, require_formal=True)
        self.assertEqual("FAIL", formal_only["status"])
        self.assertIn("fixture/development master cannot be used as formal", formal_only["errors"])

    def test_master_rejects_row_that_no_longer_matches_its_randomization_assignment(self):
        master = copy.deepcopy(self._fixture_master())
        row = master["planned_rows"][0]
        row["planned_block_segment_id"] = row["planned_block_segment_id"] + "_TAMPERED"
        master_core = dict(master)
        master_core.pop("master_hash")
        master["master_hash"] = self.tool.canonical_hash(master_core)

        report = self.tool.validate_master(master)
        self.assertEqual("FAIL", report["status"])
        self.assertTrue(
            any("planned row does not match frozen randomization assignment" in error for error in report["errors"]),
            report["errors"],
        )

    def test_formal_shaped_master_requires_hash_bound_formal_freeze_artifact(self):
        master = copy.deepcopy(self._fixture_master())
        master["formal"] = True
        master["protocol_id"] = self.tool.FORMAL_PROTOCOL_ID
        master["evidence_class"] = "FORMAL_PLANNED_ROWS_NOT_EXECUTED"
        master["freeze_validation"] = {"status": "PASS"}
        master["freeze_hash"] = self._sha("formal freeze")
        registry = master["contrast_registry"]
        registry["protocol_id"] = self.tool.FORMAL_PROTOCOL_ID
        registry["status"] = "FROZEN"
        registry_core = dict(registry)
        registry_core.pop("registry_hash")
        registry["registry_hash"] = self.tool.canonical_hash(registry_core)
        for row in master["planned_rows"]:
            row["formal"] = True
            row["protocol_id"] = self.tool.FORMAL_PROTOCOL_ID
        # Deliberately omit formal_freeze_path/formal_freeze_file_hash while
        # otherwise making the fixture look formal; this must not validate.
        self._rehash_master(master)

        report = self.tool.validate_master(master, require_formal=True)
        self.assertEqual("FAIL", report["status"])
        self.assertTrue(
            any("formal master freeze" in error and "formal_freeze_path" in error for error in report["errors"]),
            report["errors"],
        )

    def test_missing_formal_freeze_fails_closed_and_cli_returns_nonzero(self):
        report = self.tool.validate_formal_freeze({})
        self.assertEqual("FAIL", report["status"])
        self.assertIn("formal input missing real_freeze_id", report["errors"])
        with self.assertRaisesRegex(self.tool.ContractError, "FORMAL_SIM_NO_GO"):
            self.tool.make_master_rows({}, "formal-seed", fixture=False)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            freeze_path = root / "incomplete_formal_freeze.json"
            output_path = root / "master.json"
            self._write_json(freeze_path, {})
            with contextlib.redirect_stdout(io.StringIO()):
                status = self.tool.main(
                    ["generate", "--freeze", str(freeze_path), "--seed", "formal-seed", "--output", str(output_path)]
                )
            self.assertEqual(2, status)
            self.assertFalse(output_path.exists())

    def test_formal_bslosh_release_manifest_rejects_w5_and_release_id_mismatch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            w5_report = self.tool.validate_formal_freeze(self._formal_freeze_probe(root, include_w5=True))
            self._assert_freeze_no_go(w5_report, "w5_s10")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            mismatch_report = self.tool.validate_formal_freeze(
                self._formal_freeze_probe(root, manifest_release_id="BSLOSH-OTHER-R2")
            )
            self._assert_freeze_no_go(mismatch_report, "release_id", "differs")

    def test_formal_freeze_explicitly_rejects_sim_only_bslosh_identity(self):
        """A SIM-ONLY release cannot be relabelled into the physical protocol."""
        with tempfile.TemporaryDirectory() as temporary:
            freeze = self._formal_freeze_probe(Path(temporary))
            freeze["candidate_import"] = {
                "protocol_id": "SMPCC-SIM-ONLY-BSLOSH-R1-v1",
                "condition_id": "SIM_Bslosh_R1",
            }
            report = self.tool.validate_formal_freeze(freeze)
            self._assert_freeze_no_go(report, "sim-only", "isolated")

    def test_formal_freeze_rejects_missing_mismatched_or_nonblind_retry_classifier(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            missing_report = self.tool.validate_formal_freeze(self._formal_freeze_probe(root))
            self._assert_freeze_no_go(missing_report, "retry", "classifier", "missing")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            mismatch_freeze = self._formal_freeze_probe(root)
            mismatch_freeze["retry_classifier"] = self._frozen_retry_classifier(root)
            mismatch_freeze["retry_classifier"]["classifier_id"] = "SIM-RETRY-CLASSIFIER-OTHER"
            mismatch_report = self.tool.validate_formal_freeze(mismatch_freeze)
            self._assert_freeze_no_go(mismatch_report, "classifier_id", "does not match")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nonblind_freeze = self._formal_freeze_probe(root)
            nonblind_freeze["retry_classifier"] = self._frozen_retry_classifier(root, condition_blind=False)
            nonblind_report = self.tool.validate_formal_freeze(nonblind_freeze)
            self._assert_freeze_no_go(nonblind_report, "condition", "blind")

    def test_formal_freeze_requires_runtime_launch_contract_and_dataset_ledger(self):
        with tempfile.TemporaryDirectory() as temporary:
            report = self.tool.validate_formal_freeze(self._formal_freeze_probe(Path(temporary)))
        self._assert_freeze_no_go(report, "runtime", "launch", "contract", "missing")
        self._assert_freeze_no_go(report, "runtime", "backend", "missing")
        self._assert_freeze_no_go(report, "dataset", "ledger", "missing")
        self._assert_freeze_no_go(report, "stage-entry", "policy", "missing")

    def test_missing_freeze_path_fails_closed_for_formal_gate_and_generate(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            missing = root / "does_not_exist.json"
            output = root / "must_not_exist.json"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(2, self.tool.main(["formal-gate", "--freeze", str(missing)]))
                self.assertEqual(2, self.tool.main(["generate", "--freeze", str(missing), "--output", str(output)]))
            self.assertFalse(output.exists())

    def test_formal_s1_primary_cannot_be_reassigned_away_from_bslosh_bsmooth(self):
        registry = copy.deepcopy(self._fixture_master()["contrast_registry"])
        registry["protocol_id"] = self.tool.FORMAL_PROTOCOL_ID
        registry["status"] = "FROZEN"
        for contrast in registry["contrasts"]:
            if contrast["stage"] != "SIM-S1_CORE":
                continue
            pair = frozenset((contrast["left_condition"], contrast["right_condition"]))
            if pair == frozenset(("Bslosh", "Bsmooth")):
                contrast["role"] = "novelty"
            elif pair == frozenset(("Bslosh", "FixedProfile")):
                contrast["role"] = "primary_physical"
        core = dict(registry)
        core.pop("registry_hash", None)
        registry["registry_hash"] = self.tool.canonical_hash(core)
        with self.assertRaisesRegex(self.tool.ContractError, "Bslosh-Bsmooth"):
            self.tool.validate_contrast_registry(registry, formal=True)

    def test_stage_entry_requires_frozen_s1_extension_gate_not_only_closure_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with mock.patch.object(self.tool, "DEFAULT_SIM_ROOT", root):
                master, freeze, policy, index, head_hash = self._stage_entry_master_fixture(root)
                report_rules = policy["reports"]
                closure = {
                    "report_type": "SIM_S1_CLOSURE",
                    "status": "PASS",
                    "stage": "SIM-S1_CORE",
                    "master_hash": master["master_hash"],
                    "N_plan": 40,
                    "all_attempts_classified": True,
                    "retry_chain_valid": True,
                    "formal_freeze_hash": self.tool.canonical_hash(freeze),
                    "contrast_registry_hash": master["contrast_registry"]["registry_hash"],
                    "stage_entry_policy_id": policy["policy_id"],
                    "stage_entry_policy_hash": policy["policy_hash"],
                    "rule_hash": report_rules["SIM_S1_CLOSURE"]["rule_hash"],
                    "validator_hash": report_rules["SIM_S1_CLOSURE"]["validator_hash"],
                    "dataset_index_path": str(index),
                    "dataset_index_head_hash": head_hash,
                }
                closure_path = root / "s1_closure.json"
                self._write_json(closure_path, closure)
                evidence = {
                    "stage1_closure": {
                        "report_path": str(closure_path),
                        "report_hash": self.tool.sha256_file(closure_path),
                    }
                }
                row = {"stage": "SIM-S2A_SELECTIVITY"}
                with self.assertRaisesRegex(self.tool.ContractError, "SIM_S1_EXTENSION_GATE evidence"):
                    self.tool.validate_stage_entry(master, row, evidence)

                extension = {
                    "report_type": self.tool.STAGE1_EXTENSION_REPORT_TYPE,
                    "status": "PASS",
                    "stage": "SIM-S1_CORE",
                    "master_hash": master["master_hash"],
                    "formal_freeze_hash": self.tool.canonical_hash(freeze),
                    "contrast_registry_hash": master["contrast_registry"]["registry_hash"],
                    "stage_entry_policy_id": policy["policy_id"],
                    "stage_entry_policy_hash": policy["policy_hash"],
                    "rule_hash": report_rules[self.tool.STAGE1_EXTENSION_REPORT_TYPE]["rule_hash"],
                    "validator_hash": report_rules[self.tool.STAGE1_EXTENSION_REPORT_TYPE]["validator_hash"],
                    "dataset_index_path": str(index),
                    "dataset_index_head_hash": head_hash,
                    "stage1_closure_hash": self._sha("forged unrelated closure"),
                    "gates": {name: "PASS" for name in self.tool.REQUIRED_S1_EXTENSION_GATES},
                }
                extension_path = root / "s1_extension.json"
                self._write_json(extension_path, extension)
                evidence["stage1_extension_gate"] = {
                    "report_path": str(extension_path),
                    "report_hash": self.tool.sha256_file(extension_path),
                }
                with self.assertRaisesRegex(self.tool.ContractError, "not bound to the Stage-I closure"):
                    self.tool.validate_stage_entry(master, row, evidence)

                extension["stage1_closure_hash"] = evidence["stage1_closure"]["report_hash"]
                self._write_json(extension_path, extension)
                evidence["stage1_extension_gate"]["report_hash"] = self.tool.sha256_file(extension_path)
                self.tool.validate_stage_entry(master, row, evidence)

    def test_smoothmatch_has_its_own_condition_and_rejects_extra_config_change(self):
        bsmooth = self._effective_config("Bsmooth", v_ref=0.20)
        smoothmatch = self._effective_config("SmoothMatch", v_ref=0.17)
        report = self.tool.compare_smoothmatch(bsmooth, smoothmatch)
        self.assertEqual("PASS", report["status"])
        self.assertEqual({"v_ref"}, set(report["diff"]))
        self.assertEqual(set(self.tool.REQUIRED_EFFECTIVE_CONFIG_FIELDS), set(report["coverage"]))

        changed = dict(smoothmatch)
        changed["w_smooth"] = 2.0
        with self.assertRaisesRegex(self.tool.ContractError, "only in v_ref"):
            self.tool.compare_smoothmatch(bsmooth, changed)

    def test_real_parameter_alignment_binds_readonly_real_freeze_and_all_control_fields(self):
        """A formal sim cannot select its five controls without real evidence."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            real_freeze_path = root / "real_freeze_manifest.json"
            real_freeze = {
                "protocol_id": self.tool.REAL_FORMAL_PROTOCOL_ID,
                "freeze_id": "REAL-FREEZE-ALIGNMENT-TEST",
                "formal": True,
                "development_only": False,
                "status": "PASS",
            }
            self._write_json(real_freeze_path, real_freeze)
            real_freeze_path.chmod(0o444)
            configs = {}
            alignment_rows = {}
            for condition in self.tool.CONDITION_BACKENDS:
                config = self._effective_config(condition, v_ref=0.20)
                if condition == "Bslosh":
                    config["w_slosh"] = 2.0
                real_config_path = root / f"real_{condition}_effective_config.json"
                self._write_json(real_config_path, config)
                real_config_path.chmod(0o444)
                config_hash = self.tool.canonical_hash(config)
                configs[condition] = {
                    "effective_config": config,
                    "effective_config_hash": config_hash,
                }
                alignment_rows[condition] = {
                    "status": "PASS",
                    "real_effective_config_path": str(real_config_path.resolve()),
                    "real_effective_config_file_hash": self.tool.sha256_file(real_config_path),
                    "real_effective_config_hash": config_hash,
                    "sim_effective_config_hash": config_hash,
                }
            alignment_path = root / "real_parameter_alignment.json"
            alignment = {
                "schema_version": self.tool.FORMAL_REAL_PARAMETER_ALIGNMENT_SCHEMA_VERSION,
                "document_type": self.tool.FORMAL_REAL_PARAMETER_ALIGNMENT_DOCUMENT_TYPE,
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "alignment_id": "REAL-SIM-CONTROL-ALIGNMENT-TEST",
                "alignment_rule_hash": self._sha("all control fields exactly equal"),
                "real_protocol_id": self.tool.REAL_FORMAL_PROTOCOL_ID,
                "real_freeze_id": "REAL-FREEZE-ALIGNMENT-TEST",
                "sim_freeze_id": "SIM-FREEZE-ALIGNMENT-TEST",
                "real_freeze_manifest_path": str(real_freeze_path.resolve()),
                "real_freeze_manifest_hash": self.tool.sha256_file(real_freeze_path),
                "conditions": alignment_rows,
            }
            self._write_json(alignment_path, alignment)
            alignment_path.chmod(0o444)
            freeze = {
                "real_freeze_id": "REAL-FREEZE-ALIGNMENT-TEST",
                "sim_freeze_id": "SIM-FREEZE-ALIGNMENT-TEST",
            }
            entry = {
                "report_path": str(alignment_path.resolve()),
                "report_hash": self.tool.sha256_file(alignment_path),
            }
            report = self.tool.validate_formal_real_parameter_alignment(entry, freeze, configs)
            self.assertEqual("PASS", report["status"])
            self.assertEqual(set(self.tool.CONDITION_BACKENDS), set(report["conditions"]))
            self.assertEqual(
                set(self.tool.REQUIRED_EFFECTIVE_CONFIG_FIELDS),
                set(report["conditions"]["Bslosh"]["covered_fields"]),
            )

            drifted = dict(configs["Bslosh"]["effective_config"])
            drifted["v_ref"] = 0.23
            configs["Bslosh"] = {
                "effective_config": drifted,
                "effective_config_hash": self.tool.canonical_hash(drifted),
            }
            # Rebuild the alignment document so the test reaches the actual
            # field-by-field comparison instead of failing on a stale hash.
            alignment = json.loads(alignment_path.read_text(encoding="utf-8"))
            alignment["conditions"]["Bslosh"]["sim_effective_config_hash"] = configs["Bslosh"]["effective_config_hash"]
            alignment_path.chmod(0o644)
            self._write_json(alignment_path, alignment)
            alignment_path.chmod(0o444)
            entry["report_hash"] = self.tool.sha256_file(alignment_path)
            with self.assertRaisesRegex(self.tool.ContractError, "Bslosh differs in control fields"):
                self.tool.validate_formal_real_parameter_alignment(entry, freeze, configs)

    def test_formal_freeze_requires_real_parameter_alignment_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            report = self.tool.validate_formal_freeze(self._formal_freeze_probe(Path(temporary)))
        self._assert_freeze_no_go(report, "real", "parameter", "alignment", "missing")

    def test_runtime_rejects_w5_in_spec_config_and_command_spellings(self):
        for spelling in ("W5", "W5_S10", "w5-s10", "w5 s10"):
            self.assertTrue(self.tool.has_forbidden_w5({"candidate": spelling}), spelling)
            with self.assertRaisesRegex(self.tool.ContractError, "W5"):
                self.tool.command_from_spec(["runner", spelling], "motion_command")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sim_root = root / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            assets = self.tool.h0_fixture_assets(output_root)
            config_path = Path(assets["effective_config_file"])
            config = json.loads(config_path.read_text(encoding="utf-8"))
            config["rejected_candidate"] = "W5-S10"
            self._write_json(config_path, config)
            with self.assertRaisesRegex(self.tool.ContractError, "W5"):
                self.tool.run_single_row(
                    row,
                    {
                        "dry_run": True,
                        "attempt_id": row["planned_row_id"] + "_r01",
                        "assets": assets,
                        "runtime_label": "safe-looking development smoke",
                    },
                    output_root,
                    sim_root,
                )
            config.pop("rejected_candidate")
            self._write_json(config_path, config)
            with self.assertRaisesRegex(self.tool.ContractError, "W5"):
                self.tool.run_single_row(
                    row,
                    {
                        "dry_run": True,
                        "attempt_id": row["planned_row_id"] + "_r01",
                        "assets": assets,
                        "runtime_label": "W5 alternate label",
                    },
                    output_root,
                    sim_root,
                )

    def test_fixture_s1_s2_rows_are_dry_run_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sim_root = root / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            fixture_row = self._fixture_master()["planned_rows"][0]
            with self.assertRaisesRegex(self.tool.ContractError, "fixture SIM-S1/S2 rows are dry-run only"):
                self.tool.run_single_row(fixture_row, {"dry_run": False}, output_root, sim_root)

    def test_h0_effective_config_readback_is_case_bound_and_exact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            declared = self._effective_config("Bsmooth", v_ref=0.25)
            readback_path = root / "effective_config_readback.json"
            payload = {
                "record_type": "SMPCC_SIM_DEV_EFFECTIVE_CONFIG_READBACK",
                "status": "PASS",
                "case_manifest_hash": self._sha("case"),
                "declared_config_hash": self.tool.canonical_hash(declared),
                "observed_effective_config": declared,
                "observed_effective_config_hash": self.tool.canonical_hash(declared),
            }
            self._write_json(readback_path, payload)
            self.assertEqual(
                "PASS",
                self.tool.validate_development_effective_config_readback(readback_path, self._sha("case"), declared)["status"],
            )
            payload["observed_effective_config"] = dict(declared, w_smooth=2.0)
            payload["observed_effective_config_hash"] = self.tool.canonical_hash(payload["observed_effective_config"])
            self._write_json(readback_path, payload)
            with self.assertRaisesRegex(self.tool.ContractError, "differs"):
                self.tool.validate_development_effective_config_readback(readback_path, self._sha("case"), declared)

    def test_frozen_path_rigid_transform_and_clearance_are_validated(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_path = root / "H1_source.json"
            derived_path = root / "H1_sim.json"
            source = {
                "path_id": "H1",
                "source_mode": "frozen_json_replay",
                "points": [
                    {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    {"x": 1.0, "y": 0.0, "yaw": 0.0},
                    {"x": 2.0, "y": 0.0, "yaw": 0.0},
                ],
                "zones": {"Z1": [0, 1], "Z2": [1, 2], "Z3": [1, 2], "Z4": [1, 2], "Z5": [1, 2]},
            }
            derived = {
                "path_id": "H1",
                "points": [
                    {"x": 2.0, "y": 3.0, "yaw": math.pi / 2.0},
                    {"x": 2.0, "y": 4.0, "yaw": math.pi / 2.0},
                    {"x": 2.0, "y": 5.0, "yaw": math.pi / 2.0},
                ],
                "zones": source["zones"],
            }
            self._write_json(source_path, source)
            self._write_json(derived_path, derived)
            world = {"bounds": [0.0, 4.0, 0.0, 7.0], "obstacles": []}
            transform = {"rotation_rad": math.pi / 2.0, "tx": 2.0, "ty": 3.0, "yaw_offset_rad": math.pi / 2.0}

            report = self.tool.validate_path_replay(source_path, derived_path, transform, world, clearance_m=0.75)
            self.assertEqual("PASS", report["status"])
            self.assertAlmostEqual(2.0, report["arc_length"])
            self.assertAlmostEqual(2.0, report["minimum_clearance_m"])
            with self.assertRaisesRegex(self.tool.ContractError, "fit fails clearance"):
                self.tool.validate_path_replay(source_path, derived_path, transform, world, clearance_m=2.01)

    def test_analytic_clearance_rejects_narrow_circle_between_h0_vertices(self):
        """A point sampler could skip x=.025; the analytic segment gate must not."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_path = root / "H0_source.json"
            derived_path = root / "H0_derived.json"
            path = {
                "path_id": "H0",
                "source_mode": "frozen_json_replay",
                "points": [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0}],
            }
            self._write_json(source_path, path)
            self._write_json(derived_path, dict(path, points=list(path["points"])))
            world = {
                "bounds": [-1.0, 2.0, -1.0, 1.0],
                "obstacles": [{"type": "circle", "x": 0.025, "y": 0.0, "radius": 0.001}],
            }
            transform = {"rotation_rad": 0.0, "tx": 0.0, "ty": 0.0, "yaw_offset_rad": 0.0}
            with self.assertRaisesRegex(self.tool.ContractError, "fit fails clearance"):
                self.tool.validate_path_replay(source_path, derived_path, transform, world, clearance_m=0.01)

    def test_path_replay_rejects_unsupported_or_malformed_world_obstacle(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_path = root / "H0_source.json"
            derived_path = root / "H0_derived.json"
            path = {
                "path_id": "H0",
                "source_mode": "frozen_json_replay",
                "points": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
            }
            self._write_json(source_path, path)
            self._write_json(derived_path, path)
            transform = {"rotation_rad": 0.0, "tx": 0.0, "ty": 0.0, "yaw_offset_rad": 0.0}
            unsupported_world = {
                "bounds": [-1.0, 2.0, -1.0, 1.0],
                "obstacles": [{"type": "polygon", "vertices": [[0.2, 0.2], [0.3, 0.2], [0.2, 0.3]]}],
            }
            malformed_world = {
                "bounds": [-1.0, 2.0, -1.0, 1.0],
                "obstacles": [{"type": "circle", "x": 0.5, "y": 0.0, "radius": 0.0}],
            }
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_path_replay(source_path, derived_path, transform, unsupported_world, clearance_m=0.01)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_path_replay(source_path, derived_path, transform, malformed_world, clearance_m=0.01)

    def test_fixed_profile_must_be_pre_generated_hash_matched_and_read_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            profile_path = root / "fixed_profile.csv"
            profile_path.write_text("time_s,v_ref\n0.0,0.20\n1.0,0.18\n", encoding="utf-8")
            profile_path.chmod(0o444)
            profile_hash = hashlib.sha256(profile_path.read_bytes()).hexdigest()
            profile = {
                "profile_path": str(profile_path),
                "profile_hash": profile_hash,
                "generator_hash": self._sha("generator"),
                "tracker_config_hash": self._sha("tracker"),
                "constraint_audit_hash": self._sha("audit"),
                "generated_before_run": True,
                "read_only_replay": True,
                "runtime_regeneration_forbidden": True,
            }
            report = self.tool.validate_fixed_profile(profile)
            self.assertEqual("PASS", report["status"])
            self.assertEqual("FORBIDDEN", report["runtime_generation"])
            self.assertEqual(0, stat.S_IMODE(profile_path.stat().st_mode) & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))

            regenerated = dict(profile)
            regenerated["generated_before_run"] = False
            with self.assertRaisesRegex(self.tool.ContractError, "pre-generated"):
                self.tool.validate_fixed_profile(regenerated)
            writable = dict(profile)
            writable["runtime_regeneration_forbidden"] = False
            with self.assertRaisesRegex(self.tool.ContractError, "forbid runtime regeneration"):
                self.tool.validate_fixed_profile(writable)

    def test_formal_runtime_reference_rejects_real_controller_experiment_and_slosh_model(self):
        self.tool.require_simulation_owned_runtime_reference(
            "/data/a/scout_sim_replacement/formal_backend.py", "test simulation delegate"
        )
        for reference in (
            "/home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/launch/spmpc.launch",
            "pkg:=spmpc_experiments",
            "/home/a/scout_ws/src/scout_apps/control/slosh_models/launch/slosh_monitor.launch",
            "/home/a/scout_ws/devel/lib/libslosh_models.so",
            "/home/a/scout_ws/devel/lib/libspmpc_local_planner.so",
        ):
            with self.assertRaisesRegex(self.tool.ContractError, "forbidden real-stack"):
                self.tool.require_simulation_owned_runtime_reference(reference, "test runtime reference")

    def test_missing_independent_truth_and_controller_subscription_are_no_go(self):
        truth = self.tool.validate_truth_capability(None)
        self.assertFalse(truth["eligible"])
        self.assertEqual("NO_INDEPENDENT_PLANT_CAPABILITY_MANIFEST", truth["status"])

        firewall = self.tool.validate_controller_firewall(
            {
                "subscriptions": {
                    "/odom": ["/smpcc/controller"],
                    "/slosh/height": ["/smpcc/controller"],
                    "/sim_truth/liquid_height": ["/observer_only"],
                }
            },
            ["/smpcc/controller"],
        )
        self.assertEqual("FAIL", firewall["status"])
        self.assertEqual(
            [{"node": "/smpcc/controller", "topic": "/slosh/height"}],
            firewall["violations"],
        )

        no_go = self.tool.validate_formal_freeze({"protocol_id": self.tool.FORMAL_PROTOCOL_ID, "sample_size": 8})
        self.assertEqual("FAIL", no_go["status"])
        self.assertTrue(any(error.startswith("liquid truth:") for error in no_go["errors"]))

    def test_h0_development_firewall_snapshot_requires_all_case_local_hash_bound_passes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case_dir = root / "case"
            case_dir.mkdir()
            roles = {
                "controller": "/spmpc_local_planner",
                "planner": "/spmpc_local_planner",
                "tracker": "/spmpc_local_planner",
                "cmd_gate": "/cmd_vel_guard",
            }
            paths = {
                checkpoint: str((case_dir / f"h0_controller_firewall_{checkpoint}.json").resolve())
                for checkpoint in self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS
            }
            contract = {
                "contract_id": "SMPCC_SIM_H0_DEVELOPMENT_SUBSCRIBER_FIREWALL_v1",
                "record_type": self.tool.DEVELOPMENT_H0_FIREWALL_RECORD_TYPE,
                "development_only": True,
                "formal": False,
                "physical_primary_eligible": False,
                "requires_development_liquid_plant": True,
                "record_only": True,
                "checkpoints": list(self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                "forbidden_topic_prefixes": ["/sim_truth/"],
                "node_roles": roles,
                "controller_nodes": sorted(set(roles.values())),
                "snapshot_paths": paths,
                "adapter_source_path": "/workspace/smpcc_sim_h0_runtime_adapter.py",
                "adapter_source_hash": self._sha("adapter source"),
            }
            contract["contract_hash"] = self.tool.canonical_hash(contract)
            case_manifest_hash = "c" * 64
            graph = {
                "subscriptions": {
                    "/cmd_vel": ["/cmd_vel_guard"],
                    "/odom": ["/spmpc_local_planner"],
                    "/sim_truth/liquid_height": ["/rosbag_record"],
                }
            }
            snapshot = {
                "record_type": self.tool.DEVELOPMENT_H0_FIREWALL_RECORD_TYPE,
                "checkpoint": "ready",
                "status": "PASS",
                "development_only": True,
                "formal": False,
                "physical_primary_eligible": False,
                "case_manifest_hash": case_manifest_hash,
                "firewall_contract_hash": contract["contract_hash"],
                "snapshot_path": paths["ready"],
                "node_roles": roles,
                "controller_nodes": sorted(set(roles.values())),
                "missing_controller_nodes": [],
                "forbidden_controller_subscribers": [],
                "all_sim_truth_subscribers": [
                    {"topic": "/sim_truth/liquid_height", "subscribers": ["/rosbag_record"]}
                ],
                "graph": graph,
                "graph_hash": self.tool.canonical_hash(graph),
            }
            self._write_json(Path(paths["ready"]), snapshot)
            report, report_hash = self.tool.load_development_h0_firewall_snapshot(
                Path(paths["ready"]), "ready", case_dir, case_manifest_hash, contract
            )
            self.assertEqual("PASS", report["status"])
            self.assertRegex(report_hash, r"^[0-9a-f]{64}$")
            self.assertFalse(report["physical_primary_eligible"])

            # The H0 runner cannot silently accept a missing checkpoint file.
            with self.assertRaisesRegex(self.tool.ContractError, "snapshot is missing"):
                self.tool.load_development_h0_firewall_snapshot(
                    Path(paths["pre_motion"]),
                    "pre_motion",
                    case_dir,
                    case_manifest_hash,
                    contract,
                )

            # A protected controller-side /sim_truth subscriber is an audit
            # failure even if an unprotected recorder subscriber is permitted.
            bad = dict(snapshot)
            bad["graph"] = {
                "subscriptions": {
                    "/cmd_vel": ["/cmd_vel_guard"],
                    "/odom": ["/spmpc_local_planner"],
                    "/sim_truth/liquid_height": ["/spmpc_local_planner"],
                }
            }
            with self.assertRaisesRegex(self.tool.ContractError, "/sim_truth controller subscriber"):
                self.tool.validate_development_h0_firewall_snapshot(
                    bad, "ready", case_dir, case_manifest_hash, contract
                )

    def test_formal_liquid_plant_rejects_bare_pass_reports_without_intake_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            code_path = root / "independent_plant.py"
            code_path.write_text("# isolated formal fixture plant\n", encoding="utf-8")
            parameter_path = root / "plant_parameters.json"
            input_path = root / "plant_input_schema.json"
            output_path = root / "plant_output_schema.json"
            self._write_json(parameter_path, {"container_id": "C1", "formal": True})
            self._write_json(input_path, {"topic": "/odom", "semantic": "executed_simulated_base_motion"})
            self._write_json(output_path, {"topic": "/sim_truth/liquid_height"})
            hashes = {
                "plant_code_hash": self.tool.sha256_file(code_path),
                "plant_parameter_hash": self.tool.sha256_file(parameter_path),
                "plant_input_schema_hash": self.tool.sha256_file(input_path),
                "plant_output_schema_hash": self.tool.sha256_file(output_path),
            }
            fidelity_path = root / "plant_fidelity.json"
            self._write_json(
                fidelity_path,
                {
                    "report_type": "SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION",
                    "status": "PASS",
                    "formal": True,
                    "development_only": False,
                    "fidelity_validation_status": "PASS",
                    "truth_topic": "/sim_truth/liquid_height",
                    "validation_dimensions": {
                        "amplitude": "PASS",
                        "frequency": "PASS",
                        "damping": "PASS",
                        "phase": "PASS",
                        "ranking": "PASS",
                    },
                    **hashes,
                },
            )
            fidelity_hash = self.tool.sha256_file(fidelity_path)
            capability_report_path = root / "plant_capability.json"
            capability_report = {
                "report_type": "SMPCC_SIM_INDEPENDENT_LIQUID_PLANT_CAPABILITY",
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "physical_primary_eligible": True,
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "fidelity_validation_status": "PASS",
                "fidelity_report_hash": fidelity_hash,
                **hashes,
            }
            self._write_json(capability_report_path, capability_report)
            capability = {
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "fidelity_validation_status": "PASS",
                "plant_code_path": str(code_path),
                "plant_parameter_path": str(parameter_path),
                "plant_input_schema_path": str(input_path),
                "plant_output_schema_path": str(output_path),
                "fidelity_report_path": str(fidelity_path),
                "fidelity_report_hash": fidelity_hash,
                "plant_capability_report_path": str(capability_report_path),
                "plant_capability_report_hash": self.tool.sha256_file(capability_report_path),
                **hashes,
            }
            # A handful of self-authored formal/PASS fields used to pass this
            # shallow check.  Formal primary evidence must now arrive through
            # the separate, hash-bound intake package; its positive ABI path
            # is exercised in scout_liquid_plant/tests/test_formal_intake.py.
            rejected_without_intake = self.tool.validate_formal_liquid_plant_capability(capability)
            self.assertFalse(rejected_without_intake["eligible"], rejected_without_intake)
            self.assertTrue(
                any("binding schema_version" in error for error in rejected_without_intake["errors"]),
                rejected_without_intake,
            )

            capability_report["development_only"] = True
            self._write_json(capability_report_path, capability_report)
            capability["plant_capability_report_hash"] = self.tool.sha256_file(capability_report_path)
            rejected = self.tool.validate_formal_liquid_plant_capability(capability)
            self.assertFalse(rejected["eligible"], rejected)
            self.assertTrue(any("development_only" in error for error in rejected["errors"]))

    def test_formal_runtime_environment_bindings_are_absolute_and_hash_bound(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            master_path = root / "formal_master.json"
            freeze_path = root / "formal_freeze.json"
            self._write_json(master_path, {"formal": True, "kind": "master"})
            self._write_json(freeze_path, {"formal": True, "kind": "freeze"})
            bindings = self.tool.formal_runtime_environment_bindings(
                master_path,
                self.tool.sha256_file(master_path),
                freeze_path,
                self.tool.sha256_file(freeze_path),
            )
            self.assertEqual(str(master_path), bindings["SMPCC_FORMAL_MASTER_PATH"])
            self.assertEqual(str(freeze_path), bindings["SMPCC_FORMAL_FREEZE_PATH"])
            with self.assertRaisesRegex(self.tool.ContractError, "binding hash mismatch"):
                self.tool.formal_runtime_environment_bindings(
                    master_path,
                    self._sha("wrong master hash"),
                    freeze_path,
                    self.tool.sha256_file(freeze_path),
                )

    def test_method_failure_cannot_retry_but_pre_motion_infra_failure_can(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            row_id = "SIM-S1_CORE_H1_C1_Bsmooth_b01"
            row = {"planned_row_id": row_id, "condition_id": "Bsmooth"}
            common = {
                "schema_version": self.tool.SCHEMA_VERSION,
                "planned_row_id": row_id,
                "condition_id": "Bsmooth",
                "hashes": {"effective_config_hash": self._sha("config")},
                "seed_bundle_hash": self._sha("seed"),
                "method_success": False,
            }
            method_manifest = root / "method_failure.json"
            self._write_json(
                method_manifest,
                dict(common, attempt_id=row_id + "_r01", failure_class="METHOD_FAILURE", method_failure=True, motion_started=True),
            )
            with self.assertRaisesRegex(self.tool.ContractError, "method/protocol failures"):
                self.tool.authorize_retry(method_manifest, root / "method_retry.json", row_id + "_r02", "classifier-a")

            infra_manifest = root / "infra_failure.json"
            self._write_json(
                infra_manifest,
                dict(
                    common,
                    attempt_id=row_id + "_r01",
                    failure_class="INFRASTRUCTURE_ACQUISITION",
                    method_failure=False,
                    motion_started=False,
                ),
            )
            authorization_path = root / "infra_retry_authorization.json"
            authorization = self.tool.authorize_retry(infra_manifest, authorization_path, row_id + "_r02", "classifier-a")
            self.assertEqual("INFRASTRUCTURE_RETRY_ONLY", authorization["authorization_type"])
            self.assertFalse(authorization["method_failure"])
            self.assertTrue(authorization_path.is_file())
            validated = self.tool.validate_retry_authorization(authorization_path, row, row_id + "_r02")
            self.assertEqual(row_id + "_r02", validated["authorized_attempt_id"])
            self.assertEqual((True, False), self.tool.validate_failure_class("METHOD_FAILURE", True, False))
            self.assertEqual((False, True), self.tool.validate_failure_class("INFRASTRUCTURE_ACQUISITION", False, False))

    def test_retry_rejects_skipped_number_prior_identity_mismatch_and_post_success_attempt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            row_id = "SIM-S1_CORE_H1_C1_Bsmooth_b01"
            row = {"planned_row_id": row_id, "condition_id": "Bsmooth"}
            previous = {
                "schema_version": self.tool.SCHEMA_VERSION,
                "attempt_id": row_id + "_r01",
                "planned_row_id": row_id,
                "condition_id": "Bsmooth",
                "failure_class": "INFRASTRUCTURE_ACQUISITION",
                "method_failure": False,
                "method_success": False,
                "motion_started": False,
                "hashes": {"effective_config_hash": self._sha("config")},
                "seed_bundle_hash": self._sha("seed"),
            }
            previous_path = root / "previous_r01.json"
            self._write_json(previous_path, previous)
            good_authorization_path = root / "good_r02_authorization.json"
            authorization = self.tool.authorize_retry(previous_path, good_authorization_path, row_id + "_r02", "classifier-a")
            self.assertEqual(row_id + "_r02", authorization["authorized_attempt_id"])

            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(previous_path, root / "skip_at_creation.json", row_id + "_r03", "classifier-a")

            skipped = copy.deepcopy(authorization)
            skipped["authorized_attempt_id"] = row_id + "_r03"
            skipped_core = dict(skipped)
            skipped_core.pop("authorization_hash")
            skipped["authorization_hash"] = self.tool.canonical_hash(skipped_core)
            skipped_path = root / "forged_skip_r03_authorization.json"
            self._write_json(skipped_path, skipped)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_retry_authorization(skipped_path, row, row_id + "_r03")

            wrong_row_previous = dict(previous, planned_row_id="SIM-S1_CORE_H1_C1_Bsmooth_b02")
            wrong_row_previous_path = root / "wrong_row_previous.json"
            self._write_json(wrong_row_previous_path, wrong_row_previous)
            wrong_row_authorization = copy.deepcopy(authorization)
            wrong_row_authorization["previous_attempt_manifest"] = str(wrong_row_previous_path.resolve())
            wrong_row_authorization["previous_attempt_manifest_hash"] = self.tool.sha256_file(wrong_row_previous_path)
            wrong_row_core = dict(wrong_row_authorization)
            wrong_row_core.pop("authorization_hash")
            wrong_row_authorization["authorization_hash"] = self.tool.canonical_hash(wrong_row_core)
            wrong_row_authorization_path = root / "wrong_row_authorization.json"
            self._write_json(wrong_row_authorization_path, wrong_row_authorization)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_retry_authorization(wrong_row_authorization_path, row, row_id + "_r02")

            wrong_condition_previous = dict(previous, condition_id="FixedProfile")
            wrong_condition_previous_path = root / "wrong_condition_previous.json"
            self._write_json(wrong_condition_previous_path, wrong_condition_previous)
            wrong_condition_authorization = copy.deepcopy(authorization)
            wrong_condition_authorization["previous_attempt_manifest"] = str(wrong_condition_previous_path.resolve())
            wrong_condition_authorization["previous_attempt_manifest_hash"] = self.tool.sha256_file(wrong_condition_previous_path)
            wrong_condition_core = dict(wrong_condition_authorization)
            wrong_condition_core.pop("authorization_hash")
            wrong_condition_authorization["authorization_hash"] = self.tool.canonical_hash(wrong_condition_core)
            wrong_condition_authorization_path = root / "wrong_condition_authorization.json"
            self._write_json(wrong_condition_authorization_path, wrong_condition_authorization)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_retry_authorization(wrong_condition_authorization_path, row, row_id + "_r02")

            master = self._fixture_master()
            planned_row = master["planned_rows"][0]
            index = root / "success_then_retry.jsonl"
            self.tool.append_dataset_index(
                index,
                {
                    "attempt_id": planned_row["planned_row_id"] + "_r01",
                    "planned_row_id": planned_row["planned_row_id"],
                    "failure_class": "NONE",
                    "method_failure": False,
                    "method_success": True,
                },
            )
            self.tool.append_dataset_index(
                index,
                {
                    "attempt_id": planned_row["planned_row_id"] + "_r02",
                    "planned_row_id": planned_row["planned_row_id"],
                    "failure_class": "INFRASTRUCTURE_ACQUISITION",
                    "method_failure": False,
                    "method_success": False,
                },
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.summarize_ledger(master, index)

    def test_formal_retry_requires_frozen_classifier_and_bound_external_decision(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            row_id = "SIM-S1_CORE_H1_C1_Bsmooth_b01"
            row = {"planned_row_id": row_id, "condition_id": "Bsmooth"}
            case_manifest_hash = self._sha("formal case launch manifest")
            classifier = self._frozen_retry_classifier(root)
            classifier_report = self.tool.validate_frozen_retry_classifier(classifier)
            self.assertEqual([str(Path(classifier["verifier_path"]).resolve())], classifier_report["verifier_command"])
            self.assertEqual(0o555, stat.S_IMODE(Path(classifier["verifier_path"]).stat().st_mode))
            bad_verifier_command = copy.deepcopy(classifier)
            bad_verifier_command["verifier_command"] = [str(root / "unbound_verifier")]
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_frozen_retry_classifier(bad_verifier_command)

            def write_failure_event(name: str, error_hash=None):
                event = {
                    "event_type": "SMPCC_SIM_PREMOTION_FAILURE_EVENT",
                    "attempt_id": row_id + "_r01",
                    "case_launch_manifest_hash": case_manifest_hash,
                    "error_hash": error_hash or self._sha("pre-motion infrastructure acquisition error"),
                }
                path = root / name
                self._write_json(path, event)
                path.chmod(0o444)
                return path

            def write_decision(name: str, failure_event_path: Path, verifier_hash=None, reason_code=None, failure_event_error_hash=None):
                failure_event = json.loads(failure_event_path.read_text(encoding="utf-8"))
                decision = {
                    "decision_type": "SMPCC_SIM_FAILURE_CLASSIFICATION",
                    "status": "PASS",
                    "attempt_id": row_id + "_r01",
                    "case_launch_manifest_hash": case_manifest_hash,
                    "failure_class": "INFRASTRUCTURE_ACQUISITION",
                    "motion_started": False,
                    "assignment_consumed": False,
                    "condition_blind": True,
                    "pre_motion_only": True,
                    "classifier_id": classifier_report["classifier_id"],
                    "classifier_rule_hash": classifier_report["classifier_rule_hash"],
                    "classifier_manifest_hash": classifier_report["classifier_manifest_hash"],
                    "verifier_id": classifier_report["verifier_id"],
                    "verifier_hash": verifier_hash or classifier_report["verifier_hash"],
                    "reason_code": reason_code or classifier_report["reason_codes"][0],
                    "failure_event_path": str(failure_event_path.resolve()),
                    "failure_event_hash": self.tool.sha256_file(failure_event_path),
                    "failure_event_error_hash": failure_event_error_hash or failure_event["error_hash"],
                }
                decision["decision_hash"] = self.tool.canonical_hash(decision)
                path = root / name
                self._write_json(path, decision)
                return path

            def write_previous(
                name: str,
                decision_path=None,
                decision_hash=None,
                failure_event_path=None,
                failure_event_hash=None,
                retry_allowed=True,
            ):
                previous = {
                    "schema_version": self.tool.SCHEMA_VERSION,
                    "attempt_id": row_id + "_r01",
                    "planned_row_id": row_id,
                    "condition_id": "Bsmooth",
                    "formal": True,
                    "failure_class": "INFRASTRUCTURE_ACQUISITION",
                    "method_failure": False,
                    "method_success": False,
                    "motion_started": False,
                    "retry_authorization_allowed": retry_allowed,
                    "case_launch_manifest_hash": case_manifest_hash,
                    "hashes": {"effective_config_hash": self._sha("formal config")},
                    "seed_bundle_hash": self._sha("formal seed"),
                    "failure_classification_path": str(decision_path.resolve()) if decision_path is not None else None,
                    "failure_classification_hash": decision_hash,
                    "failure_event_path": str(failure_event_path.resolve()) if failure_event_path is not None else None,
                    "failure_event_hash": failure_event_hash,
                }
                path = root / name
                self._write_json(path, previous)
                return path

            valid_event = write_failure_event("valid_pre_motion_failure_event.json")
            self.assertEqual(0o444, stat.S_IMODE(valid_event.stat().st_mode))
            valid_decision = write_decision("valid_decision.json", valid_event)
            valid_previous = write_previous(
                "valid_previous.json",
                valid_decision,
                self.tool.sha256_file(valid_decision),
                valid_event,
                self.tool.sha256_file(valid_event),
            )
            authorization_path = root / "valid_authorization.json"
            authorization = self.tool.authorize_retry(
                valid_previous,
                authorization_path,
                row_id + "_r02",
                retry_classifier=classifier,
            )
            self.assertEqual(row_id + "_r02", authorization["authorized_attempt_id"])
            validated = self.tool.validate_retry_authorization(
                authorization_path,
                row,
                row_id + "_r02",
                expected_classifier_id=classifier_report["classifier_id"],
                expected_classifier_rule_hash=classifier_report["classifier_rule_hash"],
                expected_classifier_manifest_hash=classifier_report["classifier_manifest_hash"],
                expected_verifier_id=classifier_report["verifier_id"],
                expected_verifier_hash=classifier_report["verifier_hash"],
                expected_reason_codes=classifier_report["reason_codes"],
                expected_max_retries=classifier_report["max_retries_per_row"],
            )
            self.assertEqual(classifier_report["reason_codes"][0], validated["reason_code"])

            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    valid_previous,
                    root / "unfrozen_classifier_authorization.json",
                    row_id + "_r02",
                    classifier_id=classifier_report["classifier_id"],
                    classifier_rule_hash=classifier_report["classifier_rule_hash"],
                )

            no_decision_previous = write_previous("no_decision_previous.json")
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    no_decision_previous,
                    root / "no_decision_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            wrong_verifier_decision = write_decision(
                "wrong_verifier_decision.json",
                valid_event,
                verifier_hash=self._sha("wrong verifier"),
            )
            wrong_verifier_previous = write_previous(
                "wrong_verifier_previous.json",
                wrong_verifier_decision,
                self.tool.sha256_file(wrong_verifier_decision),
                valid_event,
                self.tool.sha256_file(valid_event),
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    wrong_verifier_previous,
                    root / "wrong_verifier_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            wrong_reason_decision = write_decision("wrong_reason_decision.json", valid_event, reason_code="UNFROZEN_REASON")
            wrong_reason_previous = write_previous(
                "wrong_reason_previous.json",
                wrong_reason_decision,
                self.tool.sha256_file(wrong_reason_decision),
                valid_event,
                self.tool.sha256_file(valid_event),
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    wrong_reason_previous,
                    root / "wrong_reason_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            wrong_hash_previous = write_previous(
                "wrong_decision_hash_previous.json",
                valid_decision,
                self._sha("not the decision artifact"),
                valid_event,
                self.tool.sha256_file(valid_event),
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    wrong_hash_previous,
                    root / "wrong_decision_hash_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            wrong_event_hash_previous = write_previous(
                "wrong_event_hash_previous.json",
                valid_decision,
                self.tool.sha256_file(valid_decision),
                valid_event,
                self._sha("not the pre-motion event artifact"),
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    wrong_event_hash_previous,
                    root / "wrong_event_hash_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            wrong_error_hash_decision = write_decision(
                "wrong_event_error_hash_decision.json",
                valid_event,
                failure_event_error_hash=self._sha("not the recorded pre-motion error"),
            )
            wrong_error_hash_previous = write_previous(
                "wrong_event_error_hash_previous.json",
                wrong_error_hash_decision,
                self.tool.sha256_file(wrong_error_hash_decision),
                valid_event,
                self.tool.sha256_file(valid_event),
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    wrong_error_hash_previous,
                    root / "wrong_event_error_hash_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

            unapproved_previous = write_previous(
                "unapproved_previous.json",
                valid_decision,
                self.tool.sha256_file(valid_decision),
                valid_event,
                self.tool.sha256_file(valid_event),
                retry_allowed=False,
            )
            with self.assertRaises(self.tool.ContractError):
                self.tool.authorize_retry(
                    unapproved_previous,
                    root / "unapproved_authorization.json",
                    row_id + "_r02",
                    retry_classifier=classifier,
                )

    def test_append_only_ledger_chain_rejects_duplicates_and_method_failure_replacement(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            index = root / "dataset_index.jsonl"
            first = self.tool.append_dataset_index(index, {"attempt_id": "row_a_r01", "planned_row_id": "row_a"})
            second = self.tool.append_dataset_index(index, {"attempt_id": "row_b_r01", "planned_row_id": "row_b"})
            self.assertIsNone(first["previous_entry_hash"])
            self.assertEqual(first["entry_hash"], second["previous_entry_hash"])
            self.assertEqual(["row_a_r01", "row_b_r01"], [item["attempt_id"] for item in self.tool.load_dataset_index(index)])
            with self.assertRaisesRegex(self.tool.ContractError, "append-only index already contains"):
                self.tool.append_dataset_index(index, {"attempt_id": "row_a_r01", "planned_row_id": "row_a"})

            master = self._fixture_master()
            row = master["planned_rows"][0]
            protected_index = root / "protected_dataset_index.jsonl"
            self.tool.append_dataset_index(
                protected_index,
                {
                    "attempt_id": row["planned_row_id"] + "_r01",
                    "planned_row_id": row["planned_row_id"],
                    "failure_class": "METHOD_FAILURE",
                    "method_failure": True,
                    "method_success": False,
                },
            )
            self.tool.append_dataset_index(
                protected_index,
                {
                    "attempt_id": row["planned_row_id"] + "_r02",
                    "planned_row_id": row["planned_row_id"],
                    "failure_class": "NONE",
                    "method_failure": False,
                    "method_success": True,
                },
            )
            with self.assertRaisesRegex(self.tool.ContractError, "method failure.*replacement attempt"):
                self.tool.summarize_ledger(master, protected_index)

    def test_ledger_requires_embedded_contrast_registry_and_excludes_split_block_from_pairs(self):
        missing_registry_master = copy.deepcopy(self._fixture_master())
        missing_registry_master.pop("contrast_registry", None)
        self._rehash_master(missing_registry_master)
        missing_registry_report = self.tool.validate_master(missing_registry_master)
        self.assertEqual("FAIL", missing_registry_report["status"])
        self.assertTrue(any("contrast" in error.lower() and "registry" in error.lower() for error in missing_registry_report["errors"]))
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(self.tool.ContractError):
                self.tool.summarize_ledger(missing_registry_master, Path(temporary) / "empty.jsonl")

        master = self._fixture_master()
        self.assertIn("contrast_registry", master)
        block_rows = {
            row["condition_id"]: row
            for row in master["planned_rows"]
            if row["stage"] == "SIM-S1_CORE" and row["block_id"] == "b01" and row["condition_id"] in {"Bsmooth", "Bslosh"}
        }
        self.assertEqual({"Bsmooth", "Bslosh"}, set(block_rows))
        with tempfile.TemporaryDirectory() as temporary:
            index = Path(temporary) / "split_pair.jsonl"
            for condition, split_block in (("Bsmooth", False), ("Bslosh", True)):
                row = block_rows[condition]
                self.tool.append_dataset_index(
                    index,
                    {
                        "attempt_id": row["planned_row_id"] + "_r01",
                        "planned_row_id": row["planned_row_id"],
                        "stage": row["stage"],
                        "condition_id": row["condition_id"],
                        "failure_class": "NONE",
                        "method_failure": False,
                        "method_success": True,
                        "continuous_eligibility": True,
                        "split_block": split_block,
                    },
                )
            summary = self.tool.summarize_ledger(master, index)
        by_stage_condition = {
            (item["stage"], item["condition_id"]): item for item in summary["by_stage_condition"]
        }
        self.assertEqual(0, by_stage_condition[("SIM-S1_CORE", "Bsmooth")]["N_pair"])
        self.assertEqual(0, by_stage_condition[("SIM-S1_CORE", "Bslosh")]["N_pair"])

    def test_runtime_ack_missing_field_or_wrong_hash_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            seed_bundle = self.tool.make_seed_bundle("runtime-ack-seed", "SIM-S1_CORE", 1)
            hashes = {
                "effective_config_hash": self._sha("effective config"),
                "map_hash": self._sha("map"),
                "world_hash": self._sha("world"),
                "path_hash": self._sha("path"),
            }
            case_manifest_hash = self._sha("case launch manifest")
            ack = {
                "status": "PASS",
                "formal": True,
                "case_manifest_hash": case_manifest_hash,
                **hashes,
                "consumed_hashes": dict(hashes),
                "seed_bundle_hash": seed_bundle["seed_bundle_hash"],
                "seed_trace_hashes": {name: seed_bundle["traces"][name]["trace_hash"] for name in self.tool.SEED_STREAMS},
            }
            valid_path = root / "valid_ack.json"
            self._write_json(valid_path, ack)
            self.tool.validate_runtime_ack(valid_path, case_manifest_hash, hashes, seed_bundle, formal=True)

            missing_field = dict(ack)
            missing_field.pop("seed_trace_hashes")
            missing_path = root / "missing_seed_trace_ack.json"
            self._write_json(missing_path, missing_field)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_runtime_ack(missing_path, case_manifest_hash, hashes, seed_bundle, formal=True)

            missing_consumption = dict(ack)
            missing_consumption.pop("consumed_hashes")
            missing_consumption_path = root / "missing_consumed_hashes_ack.json"
            self._write_json(missing_consumption_path, missing_consumption)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_runtime_ack(missing_consumption_path, case_manifest_hash, hashes, seed_bundle, formal=True)

            wrong_hash = dict(ack)
            wrong_hash["world_hash"] = self._sha("other world")
            wrong_hash_path = root / "wrong_world_ack.json"
            self._write_json(wrong_hash_path, wrong_hash)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_runtime_ack(wrong_hash_path, case_manifest_hash, hashes, seed_bundle, formal=True)

            wrong_consumption = dict(ack)
            wrong_consumption["consumed_hashes"] = dict(hashes, world_hash=self._sha("unconsumed world"))
            wrong_consumption_path = root / "wrong_consumed_hashes_ack.json"
            self._write_json(wrong_consumption_path, wrong_consumption)
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_runtime_ack(wrong_consumption_path, case_manifest_hash, hashes, seed_bundle, formal=True)

    def test_development_executed_motion_and_goal_events_are_case_bound(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            case_hash = self._sha("case manifest")
            motion_path = root / "motion.json"
            goal_path = root / "goal.json"
            motion = {
                "record_type": "SMPCC_SIM_DEV_MOTION_EVENT",
                "status": "PASS",
                "case_manifest_hash": case_hash,
                "observed_topic": "/odom",
                "first_effective_motion_utc": "2026-08-02T00:00:00+00:00",
                "first_effective_motion_ros_time_sec": 10.0,
            }
            goal = {
                "record_type": "SMPCC_SIM_DEV_GOAL_EVENT",
                "status": "PASS",
                "controller_status": "GOAL_REACHED",
                "case_manifest_hash": case_hash,
                "first_arrival_utc": "2026-08-02T00:00:18+00:00",
                "first_arrival_ros_time_sec": 28.0,
            }
            self._write_json(motion_path, motion)
            self._write_json(goal_path, goal)
            self.assertEqual(
                "2026-08-02T00:00:00+00:00",
                self.tool.validate_development_motion_event(motion_path, case_hash)["first_effective_motion_utc"],
            )
            self.assertEqual(
                "2026-08-02T00:00:18+00:00",
                self.tool.validate_development_goal_event(goal_path, case_hash)["first_arrival_utc"],
            )
            self._write_json(goal_path, dict(goal, controller_status="NOT_GOAL_REACHED"))
            with self.assertRaises(self.tool.ContractError):
                self.tool.validate_development_goal_event(goal_path, case_hash)

    def test_h0_dry_run_writes_fresh_tail_and_proxy_only_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            assets = self.tool.h0_fixture_assets(output_root)
            spec = {
                "dry_run": True,
                "attempt_id": row["planned_row_id"] + "_r01",
                "assets": assets,
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "simulate_goal_reached": True,
                "ros_master_uri": "127.0.0.1:45131",
                "gazebo_master_uri": "127.0.0.1:45132",
            }
            unreachable = {"ros": False, "gazebo": False}
            with mock.patch.object(self.tool, "endpoints_reachable", return_value=unreachable) as reachability:
                result = self.tool.run_single_row(row, spec, output_root, sim_root)

            self.assertEqual("PASS", result["status"])
            self.assertFalse(result["formal"])
            self.assertEqual(self.tool.PROXY_EVIDENCE_CLASS, result["evidence_class"])
            self.assertEqual(2, reachability.call_count)
            manifest = json.loads(Path(result["attempt_manifest"]).read_text(encoding="utf-8"))
            postflight = json.loads(Path(result["postflight"]).read_text(encoding="utf-8"))
            self.assertFalse(manifest["formal"])
            self.assertEqual(self.tool.PROXY_EVIDENCE_CLASS, manifest["evidence_class"])
            self.assertEqual(self.tool.DEVELOPMENT_EVIDENCE_CLASS, manifest["development_class"])
            self.assertTrue(manifest["dry_run"])
            self.assertTrue(manifest["fresh_ros_gazebo"])
            self.assertFalse(manifest["pre_ros_reachable"])
            self.assertFalse(manifest["post_ros_reachable"])
            self.assertEqual(30.0, manifest["settle_sec"])
            self.assertEqual(60.0, manifest["goal_timeout_sec"])
            self.assertEqual(5.0, manifest["tail_sec"])
            self.assertTrue(postflight["recorder_before_motion"])
            self.assertTrue(postflight["tail_recorded"])
            self.assertTrue(postflight["owned_pid_cleanup_only"])
            self.assertFalse(manifest["primary_physical_efficacy_eligible"])
            self.assertEqual({"H_proxy", "H_modal"}, set(manifest["measurement_channels"]))
            self.assertEqual("/slosh/height", manifest["measurement_channels"]["H_proxy"]["topic"])
            self.assertEqual("/sim_spmpc/slosh_height", manifest["measurement_channels"]["H_modal"]["topic"])
            self.assertNotIn("H_plant", manifest["measurement_channels"])
            index_entries = self.tool.load_dataset_index(Path(result["dataset_index"]))
            self.assertEqual(1, len(index_entries))
            self.assertEqual(manifest["actual_block_segment_id"], index_entries[0]["actual_block_segment_id"])
            self.assertEqual(manifest["split_block"], index_entries[0]["split_block"])
            self.assertEqual(manifest["planned_block_segment_id"], manifest["actual_block_segment_id"])
            lifecycle_events = [item["event"] for item in manifest["lifecycle"]]
            self.assertLess(lifecycle_events.index("recorder_started"), lifecycle_events.index("motion_released"))
            self.assertIn("tail_recorded", lifecycle_events)

    def test_h0_unvalidated_independent_plant_is_manifested_but_never_primary(self):
        """No ROS is needed to prove the H0 H_plant evidence boundary."""
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            unvalidated_plant = {
                "capability_id": "SMPCC_SIM_H0_DEVELOPMENT_LIQUID_PLANT_v1",
                "status": "UNVALIDATED_DEVELOPMENT_H_PLANT_NOT_PHYSICAL_PRIMARY",
                "development_only": True,
                "formal": False,
                "physical_primary_eligible": False,
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "controller_input_permitted": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "state_topic": "/sim_truth/liquid_state",
                "metadata_topic": "/sim_truth/liquid_metadata",
                "record_only": True,
                "fidelity_validation_status": "UNVALIDATED",
                "fidelity_report_path": None,
                "fidelity_report_hash": None,
                "plant_code_hash": self._sha("development plant code"),
                "plant_parameter_hash": self._sha("development plant parameters"),
                "plant_input_schema_hash": self._sha("development plant input schema"),
                "plant_output_schema_hash": self._sha("development plant output schema"),
            }
            self.assertFalse(self.tool.validate_truth_capability(unvalidated_plant)["eligible"])
            spec = {
                "dry_run": True,
                "attempt_id": row["planned_row_id"] + "_r01",
                "assets": self.tool.h0_fixture_assets(output_root),
                "liquid_plant_capability": unvalidated_plant,
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "simulate_goal_reached": True,
                "ros_master_uri": "127.0.0.1:45141",
                "gazebo_master_uri": "127.0.0.1:45142",
            }
            with mock.patch.object(
                self.tool, "endpoints_reachable", return_value={"ros": False, "gazebo": False}
            ):
                result = self.tool.run_single_row(row, spec, output_root, sim_root)

            manifest = json.loads(Path(result["attempt_manifest"]).read_text(encoding="utf-8"))
            case_manifest = json.loads(
                Path(manifest["case_launch_manifest_path"]).read_text(encoding="utf-8")
            )
        self.assertEqual(self.tool.PROXY_EVIDENCE_CLASS, manifest["evidence_class"])
        self.assertEqual(unvalidated_plant, case_manifest["liquid_plant_capability"])
        self.assertEqual(unvalidated_plant, manifest["liquid_plant_capability"])
        self.assertFalse(case_manifest["physical_primary_eligible_at_launch"])
        self.assertEqual("NOT_LIVE_DRY_RUN", case_manifest["development_firewall_status"])
        self.assertEqual("NOT_LIVE_DRY_RUN", manifest["development_firewall_status"])
        self.assertEqual([], manifest["development_firewall_snapshots"])
        self.assertFalse(manifest["primary_physical_efficacy_eligible"])
        self.assertFalse(manifest["liquid_truth_capability"]["eligible"])
        self.assertEqual(
            {
                "topic": "/sim_truth/liquid_height",
                "semantic": "independent_development_surrogate_UNVALIDATED_not_physical_primary",
                "fidelity_validation_status": "UNVALIDATED",
                "physical_primary_eligible": False,
            },
            manifest["measurement_channels"]["H_plant"],
        )
        self.assertFalse(
            manifest["measurement_channels"]["H_proxy"]["physical_primary_eligible"]
        )
        self.assertFalse(
            manifest["measurement_channels"]["H_modal"]["physical_primary_eligible"]
        )

    def test_actual_lifecycle_waits_for_asynchronous_recorder_artifact_before_motion(self):
        """A recorder may create ``.bag.active`` after its launcher returns.

        The runner must wait for that artifact and its later growth instead of
        treating the first scheduler tick as a pre-motion protocol failure.
        No ROS/Gazebo process is started: every child and endpoint is mocked.
        """
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            case_dir = output_root / row["stage"] / row["block_id"] / "p01_Bsmooth" / "r01"
            artifact = case_dir / "capture.bag.active"
            readback_path = case_dir / "effective_config_readback.json"
            motion_event_path = case_dir / "first_effective_motion.json"
            assets = self.tool.h0_fixture_assets(output_root)
            declared_config_path = output_root / "h0_declared_config_envelope.json"
            declared_config = {
                "contract_type": "SIM_DEV_H0_DECLARED_CONFIG",
                "fields": json.loads(Path(assets["effective_config_file"]).read_text(encoding="utf-8")),
            }
            self._write_json(declared_config_path, declared_config)
            assets["declared_config_file"] = str(declared_config_path)
            unvalidated_plant = {
                "capability_id": "SMPCC_SIM_H0_DEVELOPMENT_LIQUID_PLANT_v1",
                "status": "UNVALIDATED_DEVELOPMENT_H_PLANT_NOT_PHYSICAL_PRIMARY",
                "development_only": True,
                "formal": False,
                "physical_primary_eligible": False,
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "controller_input_permitted": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "fidelity_validation_status": "UNVALIDATED",
                "record_only": True,
            }
            firewall_roles = {
                "controller": "/spmpc_local_planner",
                "planner": "/spmpc_local_planner",
                "tracker": "/spmpc_local_planner",
                "cmd_gate": "/cmd_vel_guard",
            }
            firewall_paths = {
                checkpoint: str(
                    (case_dir / f"h0_controller_firewall_{checkpoint}.json").resolve()
                )
                for checkpoint in self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS
            }
            firewall_contract = {
                "contract_id": "SMPCC_SIM_H0_DEVELOPMENT_SUBSCRIBER_FIREWALL_v1",
                "record_type": self.tool.DEVELOPMENT_H0_FIREWALL_RECORD_TYPE,
                "development_only": True,
                "formal": False,
                "physical_primary_eligible": False,
                "requires_development_liquid_plant": True,
                "record_only": True,
                "checkpoints": list(self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                "forbidden_topic_prefixes": ["/sim_truth/"],
                "node_roles": firewall_roles,
                "controller_nodes": sorted(set(firewall_roles.values())),
                "snapshot_paths": firewall_paths,
                "adapter_source_path": "/workspace/smpcc_sim_h0_runtime_adapter.py",
                "adapter_source_hash": self._sha("adapter source"),
            }
            firewall_contract["contract_hash"] = self.tool.canonical_hash(firewall_contract)
            spec = {
                "dry_run": False,
                "attempt_id": row["planned_row_id"] + "_r01",
                "assets": assets,
                "seed_bundle": self.tool.make_seed_bundle("SIM-DEV-H0-fixture-seed-v1", "SIM-DEV-H0_SMOKE", 1),
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "post_shutdown_sec": 30.0,
                "recorder_ready_timeout_sec": 2.0,
                "ros_master_uri": "127.0.0.1:45231",
                "gazebo_master_uri": "127.0.0.1:45232",
                "launch_command": ["fake-launch"],
                "ready_command": ["fake-ready"],
                "recorder_command": ["fake-recorder"],
                "recorder_artifact": str(artifact),
                "motion_command": ["fake-motion"],
                "motion_start_command": ["fake-motion-start"],
                "motion_event_path": str(motion_event_path),
                "goal_probe_command": ["fake-goal-probe"],
                "motion_stop_command": ["fake-stop"],
                "effective_config_readback_command": ["fake-effective-config-readback"],
                "effective_config_readback_path": str(readback_path),
                "liquid_plant_capability": unvalidated_plant,
                "development_firewall": firewall_contract,
                "development_firewall_snapshot_commands": {
                    checkpoint: ["fake-firewall-snapshot", "firewall-snapshot", checkpoint]
                    for checkpoint in self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS
                },
            }

            class FakeProcess:
                next_pid = 5000

                def __init__(self):
                    self.pid = FakeProcess.next_pid
                    FakeProcess.next_pid += 1

                def poll(self):
                    return None

            state = {"recorder_started": False, "created": False, "clock": 0.0, "goal_budgets": [], "firewall_checkpoints": []}

            class FakeChildren:
                def start(self, label, _command, _environment):
                    if label == "recorder":
                        state["recorder_started"] = True
                    return FakeProcess()

                def stop_label(self, label):
                    return {"label": label, "cleanup_error": None}

                def stop_all(self):
                    return [{"label": "fake", "cleanup_error": None}]

            def fake_sleep(_seconds):
                state["clock"] += _seconds
                if state["recorder_started"] and not state["created"]:
                    artifact.parent.mkdir(parents=True, exist_ok=True)
                    artifact.write_bytes(b"bag-header")
                    state["created"] = True
                elif artifact.exists():
                    with artifact.open("ab") as stream:
                        stream.write(b"+sample")

            completed = self.tool.subprocess.CompletedProcess(["fake"], 0)

            def fake_command(_command, environment, field, _timeout_sec):
                if field.startswith("development_firewall_snapshot."):
                    checkpoint = field.rsplit(".", 1)[1]
                    graph = {
                        "subscriptions": {
                            "/cmd_vel": ["/cmd_vel_guard"],
                            "/odom": ["/spmpc_local_planner"],
                            # This recorder-side subscription is retained for
                            # audit but is not a protected controller input.
                            "/sim_truth/liquid_height": ["/rosbag_record"],
                        }
                    }
                    payload = {
                        "record_type": self.tool.DEVELOPMENT_H0_FIREWALL_RECORD_TYPE,
                        "checkpoint": checkpoint,
                        "status": "PASS",
                        "development_only": True,
                        "formal": False,
                        "physical_primary_eligible": False,
                        "case_manifest_hash": environment["SMPCC_CASE_LAUNCH_MANIFEST_SHA256"],
                        "firewall_contract_hash": firewall_contract["contract_hash"],
                        "snapshot_path": firewall_paths[checkpoint],
                        "node_roles": firewall_roles,
                        "controller_nodes": sorted(set(firewall_roles.values())),
                        "missing_controller_nodes": [],
                        "forbidden_controller_subscribers": [],
                        "all_sim_truth_subscribers": [
                            {"topic": "/sim_truth/liquid_height", "subscribers": ["/rosbag_record"]}
                        ],
                        "graph": graph,
                        "graph_hash": self.tool.canonical_hash(graph),
                    }
                    self._write_json(Path(firewall_paths[checkpoint]), payload)
                    state["firewall_checkpoints"].append(checkpoint)
                if field == "motion_start_command":
                    # The executed-motion probe consumes 20s of the only
                    # permitted trajectory budget before GOAL polling begins.
                    state["clock"] += 20.0
                    payload = {
                        "record_type": "SMPCC_SIM_DEV_MOTION_EVENT",
                        "status": "PASS",
                        "case_manifest_hash": environment["SMPCC_CASE_LAUNCH_MANIFEST_SHA256"],
                        "observed_topic": "/odom",
                        "first_effective_motion_utc": "2026-08-02T00:00:00+00:00",
                        "first_effective_motion_ros_time_sec": 12.0,
                    }
                    self._write_json(motion_event_path, payload)
                if field == "effective_config_readback_command":
                    declared = json.loads(Path(assets["declared_config_file"]).read_text(encoding="utf-8"))
                    payload = {
                        "record_type": "SMPCC_SIM_DEV_EFFECTIVE_CONFIG_READBACK",
                        "status": "PASS",
                        "case_manifest_hash": environment["SMPCC_CASE_LAUNCH_MANIFEST_SHA256"],
                        "declared_config_hash": self.tool.canonical_hash(declared),
                        "observed_effective_config": declared,
                        "observed_effective_config_hash": self.tool.canonical_hash(declared),
                    }
                    self._write_json(readback_path, payload)
                return completed

            def fake_wait_for_goal(_command, _environment, timeout_sec):
                state["goal_budgets"].append(timeout_sec)
                return True

            with mock.patch.object(self.tool, "TrackedChildren", FakeChildren), mock.patch.object(
                self.tool, "endpoints_reachable", return_value={"ros": False, "gazebo": False}
            ), mock.patch.object(self.tool, "wait_for_endpoints", return_value={"ros": True, "gazebo": True}), mock.patch.object(
                self.tool, "run_command_with_timeout", side_effect=fake_command
            ), mock.patch.object(self.tool, "wait_for_goal", side_effect=fake_wait_for_goal), mock.patch.object(
                self.tool.time, "sleep", side_effect=fake_sleep
            ), mock.patch.object(self.tool.time, "monotonic", side_effect=lambda: state["clock"]
            ):
                result = self.tool.run_single_row(row, spec, output_root, sim_root)

            self.assertEqual("PASS", result["status"])
            manifest = json.loads(Path(result["attempt_manifest"]).read_text(encoding="utf-8"))
            events = [item["event"] for item in manifest["lifecycle"]]
            self.assertLess(events.index("recorder_started"), events.index("recorder_artifact_ready"))
            self.assertLess(events.index("recorder_artifact_ready"), events.index("motion_released"))
            self.assertLess(events.index("owned_pid_cleanup"), events.index("effective_config_readback"))
            self.assertTrue(manifest["method_success"])
            self.assertEqual(2.0, manifest["recorder_ready_timeout_sec"])
            self.assertEqual(str(readback_path), manifest["effective_config_readback_path"])
            self.assertEqual(self.tool.sha256_file(readback_path), manifest["effective_config_readback_hash"])
            self.assertEqual(
                "DEVELOPMENT_DECLARED_ENVELOPE_NOT_FORMAL_EFFECTIVE_CONFIG",
                manifest["hashes"]["effective_config_hash_semantic"],
            )
            self.assertEqual(declared_config, json.loads(Path(manifest["effective_config_path"]).read_text(encoding="utf-8")))
            self.assertEqual(1, len(state["goal_budgets"]))
            self.assertGreater(state["goal_budgets"][0], 38.0)
            self.assertLess(state["goal_budgets"][0], 41.0)
            self.assertEqual(
                list(self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                state["firewall_checkpoints"],
            )
            self.assertEqual("PASS", manifest["development_firewall_status"])
            self.assertTrue(manifest["development_firewall_checkpoints_ok"])
            self.assertEqual(
                list(self.tool.DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                [item["checkpoint"] for item in manifest["development_firewall_snapshots"]],
            )
            self.assertFalse(manifest["primary_physical_efficacy_eligible"])

    def test_h0_dry_run_postflight_probe_error_is_conservative_and_releases_master_lock(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            spec = {
                "dry_run": True,
                "attempt_id": row["planned_row_id"] + "_r01",
                "assets": self.tool.h0_fixture_assets(output_root),
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "simulate_goal_reached": True,
                "ros_master_uri": "127.0.0.1:45331",
                "gazebo_master_uri": "127.0.0.1:45332",
            }
            with mock.patch.object(
                self.tool,
                "endpoints_reachable",
                side_effect=[{"ros": False, "gazebo": False}, OSError("simulated postflight probe loss")],
            ) as reachability:
                result = self.tool.run_single_row(row, spec, output_root, sim_root)

            self.assertEqual("FAIL", result["status"])
            self.assertEqual(2, reachability.call_count)
            manifest = json.loads(Path(result["attempt_manifest"]).read_text(encoding="utf-8"))
            postflight = json.loads(Path(result["postflight"]).read_text(encoding="utf-8"))
            index_entries = self.tool.load_dataset_index(Path(result["dataset_index"]))
            self.assertTrue(manifest["post_ros_reachable"])
            self.assertTrue(manifest["post_gazebo_reachable"])
            self.assertFalse(manifest["fresh_ros_gazebo"])
            self.assertEqual("PROTOCOL_FAILURE", manifest["failure_class"])
            self.assertTrue(manifest["method_failure"])
            self.assertFalse(manifest["method_success"])
            self.assertEqual("FAIL", postflight["status"])
            self.assertTrue(postflight["post_ros_reachable"])
            self.assertTrue(postflight["post_gazebo_reachable"])
            self.assertTrue(any(item["event"] == "postflight_probe_failed" for item in manifest["lifecycle"]))
            self.assertEqual(1, len(index_entries))
            self.assertEqual("PROTOCOL_FAILURE", index_entries[0]["failure_class"])
            self.assertTrue(index_entries[0]["method_failure"])
            self.assertFalse(index_entries[0]["method_success"])
            self.assertEqual([], list(sim_root.glob(".smpcc_sim_master_*.lock")))

    def test_finalization_write_or_index_failure_leaves_recovery_receipt_and_releases_lock(self):
        failure_cases = (
            ("effective_config.json", "effective_config_write"),
            ("postflight.json", "postflight_write"),
            ("attempt_manifest.json", "attempt_manifest_write"),
            ("dataset_index_append", "dataset_index_append"),
        )
        for target, expected_stage in failure_cases:
            with self.subTest(target=target), tempfile.TemporaryDirectory() as temporary:
                sim_root = Path(temporary) / "sim_root"
                output_root = sim_root / "results"
                sim_root.mkdir()
                row = self.tool.minimal_h0_row()
                spec = {
                    "dry_run": True,
                    "attempt_id": row["planned_row_id"] + "_r01",
                    "assets": self.tool.h0_fixture_assets(output_root),
                    "settle_sec": 30.0,
                    "goal_timeout_sec": 60.0,
                    "tail_sec": 5.0,
                    "simulate_goal_reached": True,
                    "ros_master_uri": "127.0.0.1:45431",
                    "gazebo_master_uri": "127.0.0.1:45432",
                }
                if target == "dataset_index_append":
                    failure_patch = mock.patch.object(
                        self.tool,
                        "append_dataset_index",
                        side_effect=OSError("injected dataset-index failure"),
                    )
                else:
                    original_write = self.tool.write_json_new

                    def fail_selected(path, value, *, _target=target):
                        if Path(path).name == _target:
                            raise OSError(f"injected {_target} write failure")
                        return original_write(path, value)

                    failure_patch = mock.patch.object(self.tool, "write_json_new", side_effect=fail_selected)
                with mock.patch.object(self.tool, "endpoints_reachable", return_value={"ros": False, "gazebo": False}), failure_patch:
                    with self.assertRaises(OSError):
                        self.tool.run_single_row(row, spec, output_root, sim_root)

                markers = list(output_root.rglob("finalization_recovery.json"))
                self.assertEqual(1, len(markers))
                marker = json.loads(markers[0].read_text(encoding="utf-8"))
                self.assertEqual("RECOVERY_REQUIRED", marker["status"])
                self.assertEqual("UNCONFIRMED_DO_NOT_CONSUME", marker["ledger_admission"])
                self.assertEqual(expected_stage, marker["finalization_stage"])
                self.assertFalse((output_root / "dataset_index.jsonl").exists())
                self.assertEqual([], list(sim_root.glob(".smpcc_sim_master_*.lock")))

    def test_nonformal_dry_run_never_becomes_physical_primary_from_self_declared_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            sim_root = Path(temporary) / "sim_root"
            output_root = sim_root / "results"
            sim_root.mkdir()
            row = self.tool.minimal_h0_row()
            assets = self.tool.h0_fixture_assets(output_root)
            claimed_truth = {
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "fidelity_validation_status": "PASS",
                "plant_code_hash": self._sha("claimed plant code"),
                "plant_parameter_hash": self._sha("claimed plant parameters"),
                "plant_input_schema_hash": self._sha("claimed plant input schema"),
                "plant_output_schema_hash": self._sha("claimed plant output schema"),
                "fidelity_report_hash": self._sha("claimed fidelity report"),
            }
            self.assertTrue(self.tool.validate_truth_capability(claimed_truth)["eligible"])
            spec = {
                "dry_run": True,
                "attempt_id": row["planned_row_id"] + "_r01",
                "assets": assets,
                "liquid_plant_capability": claimed_truth,
                "controller_nodes": ["/smpcc/controller"],
                "firewall_graph": {"subscriptions": {"/odom": ["/smpcc/controller"]}},
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "simulate_goal_reached": True,
                "ros_master_uri": "127.0.0.1:45231",
                "gazebo_master_uri": "127.0.0.1:45232",
            }
            with mock.patch.object(self.tool, "endpoints_reachable", return_value={"ros": False, "gazebo": False}):
                result = self.tool.run_single_row(row, spec, output_root, sim_root)

            self.assertEqual("PASS", result["status"])
            manifest = json.loads(Path(result["attempt_manifest"]).read_text(encoding="utf-8"))
            postflight = json.loads(Path(result["postflight"]).read_text(encoding="utf-8"))
            self.assertFalse(manifest["formal"])
            self.assertTrue(manifest["dry_run"])
            self.assertTrue(manifest["liquid_truth_capability"]["eligible"])
            self.assertEqual(["PASS", "PASS", "PASS"], [report["status"] for report in postflight["firewall_reports"]])
            self.assertEqual(["ready", "pre_motion", "postflight"], [report["checkpoint"] for report in postflight["firewall_reports"]])
            self.assertFalse(manifest["primary_physical_efficacy_eligible"])


if __name__ == "__main__":
    unittest.main()
