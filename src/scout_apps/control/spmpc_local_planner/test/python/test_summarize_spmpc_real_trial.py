#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "tools/analysis/summarize_spmpc_real_trial.py"
)
SPEC = importlib.util.spec_from_file_location("summarize_spmpc_real_trial", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def base_summary(variant="B0", field_readable=True):
    return {
        "topics": {"critical_missing": []},
        "intent": {"variant": variant, "delay_phase_mode": "off"},
        "observed": {"controller_variant_last": variant, "status_counts": {}},
        "metrics": {
            "delay_state": {},
            "effective_config_last": {},
            "optimizer_pressure": {},
            "command_intervention": {},
            "intent_effective": {"mismatches": []},
            "warm_start": {
                "used_fallback_field_readable": field_readable,
                "used_fallback_count": 0,
            },
        },
    }


class SummarizeSpmpcRealTrialTest(unittest.TestCase):
    def test_warm_start_layout_exposes_used_fallback(self):
        label = (
            "valid,used_flatness,used_previous_solution,used_fallback,"
            "used_slosh_rollout,bound_violation_count,max_v,max_omega,max_a,"
            "max_lateral_acc,max_slosh_height_pred,reference_fit_error"
        )
        msg = SimpleNamespace(
            layout=SimpleNamespace(dim=[SimpleNamespace(label=label)]),
            data=[1, 1, 0, 1, 0, 0, 0.2, 0.1, 0.3, 0.4, 5.0, 0.0],
        )
        parsed = MODULE.parse_multiarray(msg)
        self.assertEqual(parsed["valid"], 1.0)
        self.assertEqual(parsed["used_fallback"], 1.0)

    def test_b0_does_not_require_slosh_optimizer_signals(self):
        codes = {item["code"] for item in MODULE.build_red_flags(base_summary("B0"))}
        self.assertNotIn("horizon_peak_missing", codes)
        self.assertNotIn("slosh_cost_inactive", codes)

    def test_unreadable_fallback_field_is_a_red_flag(self):
        codes = {
            item["code"]
            for item in MODULE.build_red_flags(base_summary("B0", field_readable=False))
        }
        self.assertIn("warm_start_fallback_field_unreadable", codes)

    def test_b_slosh_requires_horizon_summary(self):
        codes = {
            item["code"] for item in MODULE.build_red_flags(base_summary("B_slosh"))
        }
        self.assertIn("horizon_peak_missing", codes)

    def test_robot_only_delay_split_has_no_false_red_flag(self):
        summary = base_summary("B0")
        summary["intent"]["delay_phase_mode"] = "fixed_robot_only"
        summary["metrics"]["effective_config_last"]["delay_phase_mode_code"] = 4.0
        summary["metrics"]["delay_state"] = {
            "delay_compensation_applied_frac": 1.0,
            "history_complete_frac": 1.0,
            "robot_delay_compensation_applied_frac": 1.0,
            "liquid_delay_compensation_applied_frac": 0.0,
        }
        codes = {item["code"] for item in MODULE.build_red_flags(summary)}
        self.assertNotIn("fixed_closed_loop_not_applied", codes)
        self.assertNotIn("robot_only_delay_not_applied", codes)
        self.assertNotIn("robot_only_delay_touched_liquid", codes)

    def test_robot_only_delay_split_detects_wrong_solver_input(self):
        summary = base_summary("B0")
        summary["intent"]["delay_phase_mode"] = "fixed_robot_only"
        summary["metrics"]["effective_config_last"]["delay_phase_mode_code"] = 4.0
        summary["metrics"]["delay_state"] = {
            "delay_compensation_applied_frac": 1.0,
            "history_complete_frac": 1.0,
            "robot_delay_compensation_applied_frac": 0.90,
            "liquid_delay_compensation_applied_frac": 0.10,
        }
        codes = {item["code"] for item in MODULE.build_red_flags(summary)}
        self.assertIn("robot_only_delay_not_applied", codes)
        self.assertIn("robot_only_delay_touched_liquid", codes)


if __name__ == "__main__":
    unittest.main()
