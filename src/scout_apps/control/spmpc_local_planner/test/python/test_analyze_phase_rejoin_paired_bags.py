#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools" / "analysis" / \
    "analyze_phase_rejoin_paired_bags.py"
SPEC = importlib.util.spec_from_file_location(
    "analyze_phase_rejoin_paired_bags", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def audit(cycle_id, stamp, v=0.1, omega=0.0, status="B_slosh_ACADOS_OK",
          command_publish_stamp=None):
    return {
        "t": stamp,
        "command_publish_t": (
            stamp if command_publish_stamp is None else command_publish_stamp),
        "cycle_id": cycle_id,
        "status": status,
        "solver_status": "B_slosh_ACADOS_OK",
        "solve_attempted": True,
        "solve_success": True,
        "command_accepted": True,
        "publish_cmd_vel": True,
        "command_was_published": True,
        "command_contract_violation": False,
        "safety_gate_intervened": False,
        "linear_limited": False,
        "angular_rate_limited": False,
        "angular_accel_limited": False,
        "published_cmd_v": v,
        "published_cmd_omega": omega,
    }


def phase(cycle_id, stamp, consistent=True, lead=0, liquid_steps=3,
          mode="enforce"):
    return {
        "t": stamp,
        "cycle_id": cycle_id,
        "mode_name": mode,
        "artifact_loaded": True,
        "contract_valid": True,
        "ready": True,
        "candidate_count": 3,
        "phase_lead_steps": lead,
        "front_steps": 0,
        "liquid_steps": liquid_steps,
        "solver_terminal_step": liquid_steps,
        "terminal_gate_accepted": True,
        "command_contract_consistent": consistent,
        "recovery_command_used": False,
        "controlled_stop_used": False,
        "prediction_valid": True,
        "status": "ENFORCE_TERMINAL_ACCEPTED",
    }


def analyzed_run(external, internal, valid=True, contract_valid=True):
    motion = {
        key: {
            "unit": unit, "rms": value, "p95": value, "peak": value,
        }
        for key, unit, value in (
            ("linear_speed_abs", "m/s", 0.2),
            ("angular_speed_abs", "rad/s", 0.3),
            ("linear_accel_abs", "m/s^2", 0.4),
            ("angular_accel_abs", "rad/s^2", 0.5),
        )
    }
    return {
        "height_metrics": {
            "external_slosh_height": {
                "rms_mm": external[0], "p95_mm": external[1],
                "peak_mm": external[2],
            },
            "internal_spmpc_slosh_height": {
                "rms_mm": internal[0], "p95_mm": internal[1],
                "peak_mm": internal[2],
            },
        },
        "motion_metrics": motion,
        "window": {"completion_time_sec": 10.0},
        "path_projection_distance": {"max_m": None},
        "evidence_valid": valid,
        "run_contract_valid": contract_valid,
    }


class PhaseRejoinPairedBagAnalysisTest(unittest.TestCase):
    def test_command_motion_metrics_include_pre_window_launch_transition(self):
        commands = [
            {"t": 9.9, "v": 0.0, "omega": 0.0},
            {"t": 10.0, "v": 0.1, "omega": -0.1},
            {"t": 10.1, "v": 0.3, "omega": 0.2},
            {"t": 10.2, "v": 0.2, "omega": 0.0},
        ]
        metrics = MODULE.command_motion_metrics(
            commands, start=10.0, goal=10.2)
        self.assertTrue(metrics["pre_window_anchor_used"])
        self.assertEqual(3, metrics["command_samples"])
        self.assertEqual(3, metrics["derivative_samples"])
        self.assertAlmostEqual(0.3, metrics["linear_speed_abs"]["peak"])
        self.assertAlmostEqual(2.0, metrics["linear_accel_abs"]["peak"])
        self.assertAlmostEqual(3.0, metrics["angular_accel_abs"]["peak"])

    def test_signal_metrics_remove_reported_pre_window_offset(self):
        series = [
            (8.5, 0.001), (9.5, 0.001),
            (10.0, 0.001), (10.1, 0.002),
            (10.2, 0.003), (10.3, 0.004),
        ]
        metrics = MODULE.signal_metrics(
            series, start=10.0, end=10.3,
            scale_to_mm=1000.0, native_unit="m")
        self.assertEqual("pre_window_median", metrics["initial_offset_source"])
        self.assertAlmostEqual(1.0, metrics["initial_offset_mm"])
        self.assertAlmostEqual(math.sqrt(3.5), metrics["rms_mm"])
        self.assertAlmostEqual(2.85, metrics["p95_mm"])
        self.assertAlmostEqual(3.0, metrics["peak_mm"])

    def test_signal_metrics_explicitly_fall_back_to_first_window_sample(self):
        metrics = MODULE.signal_metrics(
            [(10.0, 2.0), (10.1, 3.0), (10.2, 1.0)],
            start=10.0, end=10.2, scale_to_mm=1.0, native_unit="mm")
        self.assertEqual("first_window_sample", metrics["initial_offset_source"])
        self.assertAlmostEqual(2.0, metrics["initial_offset_mm"])
        self.assertAlmostEqual(1.0, metrics["peak_mm"])

    def test_window_uses_audited_actual_command_and_latched_goal(self):
        audits = [
            audit(1, 10.001, v=0.0, command_publish_stamp=10.0),
            audit(2, 11.001, v=0.2, command_publish_stamp=11.0),
        ]
        commands = [
            {"t": 10.0, "v": 0.0, "omega": 0.0},
            {"t": 11.0, "v": 0.2, "omega": 0.0},
        ]
        statuses = [
            {"t": 11.0, "value": "B_slosh_ACADOS_OK"},
            {"t": 15.0, "value": "GOAL_REACHED_LATCHED"},
        ]
        window = MODULE.find_active_window(
            audits, statuses, commands, bag_end=17.1)
        self.assertEqual(11.0, window["start"])
        self.assertEqual(15.0, window["goal"])
        self.assertEqual(17.0, window["end"])
        self.assertEqual(4.0, window["completion_time_sec"])

    def test_window_fails_closed_without_two_second_tail(self):
        with self.assertRaisesRegex(MODULE.AnalysisError, "required.*tail"):
            MODULE.find_active_window(
                [audit(1, 1.0)],
                [{"t": 2.0, "value": "GOAL_REACHED"}],
                [{"t": 1.0, "v": 0.1, "omega": 0.0}],
                bag_end=3.9)

    def test_phase_contract_detects_horizon_lead_and_consistency(self):
        audits = [audit(1, 1.0), audit(2, 2.0), audit(3, 3.0)]
        phases = [
            phase(1, 1.0),
            phase(2, 2.0, consistent=False, lead=2),
            phase(3, 3.0, liquid_steps=4),
        ]
        summary = MODULE.summarize_phase_contract(
            audits, phases, start=1.0, goal=4.0,
            expected_mode="enforce", expected_liquid_steps=3,
            phase_lead_min=-1, phase_lead_max=1)
        self.assertTrue(summary["mode_valid"])
        self.assertEqual(1, summary["liquid_steps"]["violation_count"])
        self.assertEqual(1, summary["phase_lead_steps"]["violation_count"])
        self.assertEqual(1, summary["command_contract"]["inconsistent_count"])
        self.assertEqual(0, summary["missing_phase_cycle_count"])

    def test_phase_contract_joins_boundary_evidence_by_cycle_id(self):
        audits = [audit(7, 10.000), audit(8, 10.030)]
        phases = [
            phase(7, 9.999),
            phase(8, 10.031),
            phase(6, 9.970, mode="unexpected_outside_window"),
        ]
        summary = MODULE.summarize_phase_contract(
            audits, phases, start=10.000, goal=11.000,
            expected_mode="enforce")
        self.assertEqual(2, summary["active_records"])
        self.assertEqual(2, summary["joined_successful_cycles"])
        self.assertEqual(0, summary["missing_phase_cycle_count"])
        self.assertEqual({"enforce": 2}, summary["mode_counts"])

    def test_phase_contract_still_rejects_a_genuinely_missing_cycle(self):
        audits = [audit(7, 10.000), audit(8, 10.030)]
        phases = [phase(7, 9.999)]
        summary = MODULE.summarize_phase_contract(
            audits, phases, start=10.000, goal=11.000,
            expected_mode="enforce")
        self.assertEqual(1, summary["joined_successful_cycles"])
        self.assertEqual(1, summary["missing_phase_cycle_count"])
        self.assertEqual([8], summary["missing_phase_cycle_ids"])

    def test_phase_contract_rejects_duplicates_and_excludes_goal_cycle(self):
        audits = [audit(7, 10.000), audit(8, 11.000)]
        phases = [
            phase(7, 9.999),
            phase(7, 10.001),
            phase(8, 10.999, mode="unexpected_goal_cycle"),
        ]
        summary = MODULE.summarize_phase_contract(
            audits, phases, start=10.000, goal=11.000,
            expected_mode="enforce")
        self.assertEqual(2, summary["active_records"])
        self.assertEqual(1, summary["duplicate_phase_cycle_count"])
        self.assertEqual([7], summary["duplicate_phase_cycle_ids"])
        self.assertEqual({"enforce": 2}, summary["mode_counts"])

    def test_comparison_requires_all_six_height_metrics_to_improve(self):
        # A baseline method anomaly is reported by analyze_run but does not
        # invalidate the source-separated comparison.  The enforce contract
        # remains a hard acceptance gate.
        baseline = analyzed_run(
            (2.0, 3.0, 5.0), (4.0, 5.0, 7.0), contract_valid=False)
        enforce = analyzed_run((1.0, 2.0, 4.0), (3.0, 4.0, 6.0))
        result = MODULE.compare_runs(baseline, enforce)
        self.assertTrue(result["acceptance"]["pass"])

        enforce["height_metrics"]["external_slosh_height"]["peak_mm"] = 5.1
        result = MODULE.compare_runs(baseline, enforce)
        self.assertFalse(result["acceptance"]["pass"])
        self.assertFalse(result["acceptance"]["checks"][
            "external_rms_p95_peak_all_improved"])

        enforce["height_metrics"]["external_slosh_height"]["peak_mm"] = 4.0
        enforce["run_contract_valid"] = False
        result = MODULE.compare_runs(baseline, enforce)
        self.assertFalse(result["acceptance"]["pass"])
        self.assertFalse(result["acceptance"]["checks"][
            "enforce_run_contract_valid"])

    def test_coverage_check_requires_both_window_edges(self):
        complete = [(1.01, 0.0), (2.0, 0.0), (2.95, 0.0)]
        incomplete = [(1.01, 0.0), (2.0, 0.0), (2.70, 0.0)]
        self.assertTrue(MODULE.coverage_check(complete, 1.0, 3.0)["ok"])
        self.assertFalse(MODULE.coverage_check(incomplete, 1.0, 3.0)["ok"])


if __name__ == "__main__":
    unittest.main()
