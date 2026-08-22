#!/usr/bin/env python3

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "tools"
    / "simulation"
    / "analyze_phase_rejoin_simulation_campaign.py"
)
SPEC = importlib.util.spec_from_file_location("simulation_campaign_analysis", SCRIPT_PATH)
ANALYSIS = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ANALYSIS)


MODES = {
    "C0": "ordinary_mpcc",
    "C1": "smooth_match_mpcc",
    "C2": "offline_replay",
    "C3": "residual_no_gate",
    "C4": "phase_rejoin_full",
    "IS": "input_shaping",
}
VALUES = {
    "C0": 0.0040,
    "C1": 0.0038,
    "C2": 0.0032,
    "C3": 0.0036,
    "C4": 0.0030,
    "IS": 0.0035,
}
SESSION_HASH = "a" * 64
FORMAL_SEEDS = tuple(range(3101, 3117))


def trial_summary(seed, condition, formal=False, causal_ready=False):
    summary = {
        "schema": "spmpc_closed_loop_trial_summary_v1",
        "status": "TRIAL_COMPLETE",
        "implementation_complete": True,
        "condition_id": condition,
        "mode": MODES[condition],
        "implementation_id": "implementation_{}".format(condition),
        "seed": seed,
        "simulation_only": True,
        "formal_robot_release": False,
        "physical_parameter_claim": False,
        "preliminary_planar_r03_parameters_only": True,
        "plant_truth_visible_to_controller": False,
        "external_liquid_truth_used_for_control": False,
        "controller_observer_source": "plant_motion_derived_odom",
        "final_command_transaction": True,
        "command_history_source": "final_published_command",
        "dual_channel_execution_model": True,
        "plant_controller_parameter_independence": True,
        "plant_freeze_id": "SIM_PLANT_TEST_V1",
        "artifact_contract_id": "phase_rejoin_nominal_sequence_v3",
        "trial": {
            "control_rate_hz": 30.0,
            "motion_end_sec": 8.0,
            "fixed_tail_sec": 4.0,
            "publish_latency_sec": 0.01,
            "sequence_completed": True,
            "task_success": True,
            "completion_reason": "goal_reached",
            "runtime_error": "",
        },
        "primary_metric": {
            "name": "external_measured_height_q95_m",
            "window": "motion_plus_fixed_tail",
            "statistics_unit": "complete_trial",
            "quantile_method": "nearest_rank",
            "sample_count": 360,
            "value_m": VALUES[condition] + seed * 1.0e-6,
        },
        "secondary_metrics": {
            "tracking_q95_m": 0.02,
        },
        "controller_audit": {},
        "baseline_contract": {
            "formal_c3_c4_causal_comparison_ready": causal_ready,
            "c3_exact_c4_optimizer_match": causal_ready,
        },
    }
    if formal:
        summary["frozen_session_sha256"] = SESSION_HASH
    return summary


class SimulationCampaignAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def write_campaign(self, seeds=(11,), formal=False, causal_ready=False):
        for seed in seeds:
            for condition in MODES:
                path = self.root / "seed_{}_{}.json".format(seed, condition)
                path.write_text(
                    json.dumps(
                        trial_summary(seed, condition, formal, causal_ready)
                    ),
                    encoding="utf-8",
                )

    def rewrite(self, seed, condition, mutator):
        path = self.root / "seed_{}_{}.json".format(seed, condition)
        value = json.loads(path.read_text(encoding="utf-8"))
        mutator(value)
        path.write_text(json.dumps(value), encoding="utf-8")

    def test_pilot_is_trial_level_and_never_authorizes_paper_claim(self):
        self.write_campaign(seeds=(11, 12))
        report = ANALYSIS.analyze_campaign(self.root, "pilot")
        self.assertEqual(
            report["decision"]["status"],
            "PILOT_READY_FOR_DEVELOPMENT_REVIEW",
        )
        self.assertFalse(report["decision"]["paper_claim_authorized"])
        self.assertFalse(
            report["cycle_samples_used_as_independent_observations"]
        )
        self.assertEqual(report["campaign"]["trial_count"], 12)
        comparison = report["paired_comparisons"]["C4_minus_C0"]
        self.assertEqual(comparison["pair_count"], 2)
        self.assertAlmostEqual(comparison["median_difference_m"], -0.001)

    def test_failed_trial_is_retained_in_condition_and_pair(self):
        self.write_campaign()

        def fail(value):
            value["status"] = "TRIAL_COMPLETE_WITH_FAILURE"
            value["trial"]["sequence_completed"] = False
            value["trial"]["task_success"] = False
            value["trial"]["completion_reason"] = "goal_timeout"

        self.rewrite(11, "C4", fail)
        report = ANALYSIS.analyze_campaign(self.root, "pilot")
        self.assertEqual(report["campaign"]["failed_trial_count"], 1)
        self.assertEqual(
            report["condition_summaries"]["C4"]["failed_trial_count"], 1
        )
        comparison = report["paired_comparisons"]["C4_minus_C0"]
        self.assertEqual(comparison["failed_pair_count"], 1)
        self.assertTrue(comparison["pairs"][0]["failure_retained"])

    def test_duplicate_seed_condition_rejects_cycle_level_expansion(self):
        self.write_campaign()
        duplicate = trial_summary(11, "C0")
        (self.root / "duplicate.json").write_text(
            json.dumps(duplicate), encoding="utf-8"
        )
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "duplicate trial"):
            ANALYSIS.analyze_campaign(self.root, "pilot")

    def test_missing_condition_breaks_complete_pair_grid(self):
        self.write_campaign(seeds=(11, 12))
        (self.root / "seed_12_IS.json").unlink()
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "complete seed-by-condition"):
            ANALYSIS.analyze_campaign(self.root, "pilot")

    def test_truth_leakage_is_rejected(self):
        self.write_campaign()
        self.rewrite(
            11,
            "C4",
            lambda value: value.__setitem__(
                "external_liquid_truth_used_for_control", True
            ),
        )
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "must be false"):
            ANALYSIS.analyze_campaign(self.root, "pilot")

    def test_non_trial_primary_metric_is_rejected(self):
        self.write_campaign()
        self.rewrite(
            11,
            "C0",
            lambda value: value["primary_metric"].__setitem__(
                "statistics_unit", "control_cycle"
            ),
        )
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "non-trial"):
            ANALYSIS.analyze_campaign(self.root, "pilot")

    def test_incomplete_implementation_is_rejected(self):
        self.write_campaign()
        self.rewrite(
            11,
            "C2",
            lambda value: value.__setitem__("implementation_complete", False),
        )
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "not implementation-complete"):
            ANALYSIS.analyze_campaign(self.root, "pilot")

    def test_formal_requires_hash_threshold_and_summary_binding(self):
        self.write_campaign(seeds=(11, 12, 13))
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "requires --frozen"):
            ANALYSIS.analyze_campaign(self.root, "formal")
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "requires --preregistered"):
            ANALYSIS.analyze_campaign(
                self.root, "formal", frozen_session_sha256=SESSION_HASH
            )
        with self.assertRaisesRegex(ANALYSIS.CampaignError, "not bound"):
            ANALYSIS.analyze_campaign(
                self.root,
                "formal",
                frozen_session_sha256=SESSION_HASH,
                preregistered_min_effect_m=0.0005,
            )

    def test_formal_rejects_effect_threshold_different_from_session(self):
        self.write_campaign(
            seeds=FORMAL_SEEDS, formal=True, causal_ready=True
        )
        with self.assertRaisesRegex(
            ANALYSIS.CampaignError, "differs from frozen session"
        ):
            ANALYSIS.analyze_campaign(
                self.root,
                "formal",
                frozen_session_sha256=SESSION_HASH,
                preregistered_min_effect_m=0.0004,
            )

    def test_formal_pass_requires_effects_and_causal_c3_contract(self):
        self.write_campaign(
            seeds=FORMAL_SEEDS, formal=True, causal_ready=True
        )
        report = ANALYSIS.analyze_campaign(
            self.root,
            "formal",
            frozen_session_sha256=SESSION_HASH,
            preregistered_min_effect_m=0.0005,
        )
        self.assertEqual(report["decision"]["status"], "PASS")
        self.assertTrue(report["decision"]["paper_claim_authorized"])
        self.assertTrue(
            report["decision"]["checks"]["c3_c4_causal_contract_ready"]
        )

    def test_formal_pilot_only_c3_contract_forces_fail(self):
        self.write_campaign(
            seeds=FORMAL_SEEDS, formal=True, causal_ready=False
        )
        report = ANALYSIS.analyze_campaign(
            self.root,
            "formal",
            frozen_session_sha256=SESSION_HASH,
            preregistered_min_effect_m=0.0005,
        )
        self.assertEqual(report["decision"]["status"], "FAIL")
        self.assertFalse(report["decision"]["paper_claim_authorized"])
        self.assertFalse(
            report["decision"]["checks"]["c3_c4_causal_contract_ready"]
        )

    def test_formal_requires_exactly_sixteen_paired_blocks(self):
        self.write_campaign(
            seeds=FORMAL_SEEDS[:-1], formal=True, causal_ready=True
        )
        report = ANALYSIS.analyze_campaign(
            self.root,
            "formal",
            frozen_session_sha256=SESSION_HASH,
            preregistered_min_effect_m=0.0005,
        )
        self.assertEqual(report["decision"]["status"], "FAIL")
        self.assertFalse(
            report["decision"]["checks"]["exact_formal_paired_block_count"]
        )

    def test_formal_task_noninferiority_uses_paired_upper_bound(self):
        seeds = FORMAL_SEEDS
        self.write_campaign(seeds=seeds, formal=True, causal_ready=True)
        for seed in seeds:
            self.rewrite(
                seed,
                "C4",
                lambda value: value["trial"].__setitem__(
                    "motion_end_sec", 9.0
                ),
            )
        report = ANALYSIS.analyze_campaign(
            self.root,
            "formal",
            frozen_session_sha256=SESSION_HASH,
            preregistered_min_effect_m=0.0005,
        )
        self.assertEqual(report["decision"]["status"], "FAIL")
        self.assertFalse(
            report["decision"]["checks"]
            ["c4_vs_c1_completion_time_noninferior"]
        )

    def test_create_new_output_is_not_overwritten(self):
        self.write_campaign()
        output = self.root / "analysis.json"
        first = ANALYSIS.main(
            [
                "--input-dir",
                str(self.root),
                "--output",
                str(output),
                "--stage",
                "pilot",
            ]
        )
        self.assertEqual(first, 0)
        original = output.read_bytes()
        second = ANALYSIS.main(
            [
                "--input-dir",
                str(self.root),
                "--output",
                str(output),
                "--stage",
                "pilot",
            ]
        )
        self.assertEqual(second, 2)
        self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
