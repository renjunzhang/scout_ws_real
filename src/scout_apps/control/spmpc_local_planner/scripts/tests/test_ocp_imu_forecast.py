#!/usr/bin/env python3
"""Synthetic time/continuity contracts for the offline forecast diagnostic."""
import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import analyze_ocp_imu_forecast as tool


def monitor_row(stamp, count, eta=0.001):
    return dict(valid=True, configured=True, source=2, bias_ready=True,
                filter_ready=True, state_stamp=stamp, observer_update_count=count,
                reset_epoch=0, eta_x=eta, eta_x_dot=0.0, eta_y=0.0,
                eta_y_dot=0.0, modal_height_m=eta*2)


def fixture():
    epoch = 100.0
    h = dict(valid=True, slosh_enabled=True, zero_liquid_initial_state=False,
             schema_version=5, backend=tool.BACKEND, variant="B_slosh",
             solver_status="B_slosh_ACADOS_OK", cycle_id=1, solver_input_epoch=epoch,
             solve_start_stamp=epoch+.001, solve_end_stamp=epoch+.005,
             horizon_available_stamp=epoch+.005, dt=1/30, horizon_steps=60,
             t=list(np.arange(61)/30), h_modal=[.004]*61,
             eta_x=[.002]*61, eta_x_dot=[0.0]*61, eta_y=[0.0]*61, eta_y_dot=[0.0]*61)
    s = {k: v for k, v in h.items() if not isinstance(v, list)}
    s.update(state_width=28, eta_x=.002, eta_x_dot=0.0, eta_y=0.0, eta_y_dot=0.0)
    a = dict(cycle_id=1, solver_input_epoch=epoch, solve_attempted=True, solve_success=True,
             command_accepted=True, command_was_published=True, observer_source=2,
             command_publish_stamp=epoch+.006)
    monitor = tool.Monitor([monitor_row(99.9+i*.02, i) for i in range(161)], 2, .05)
    return h, s, a, monitor


class ForecastTest(unittest.TestCase):
    def test_hermite_uses_signed_state_before_magnitude(self):
        left, right = monitor_row(10, 1), monitor_row(10.02, 2)
        right.update(eta_x=-.001, modal_height_m=.002)
        monitor = tool.Monitor([left, right], 2, .05)
        height, _ = monitor.height(10.01)
        self.assertAlmostEqual(height, 0, places=12)

    def test_never_interpolates_across_invalid_gap_reset_or_count_jump(self):
        for kind in ("invalid", "gap", "reset", "count"):
            rows = [monitor_row(10, 1), monitor_row(10.02, 2)]
            if kind == "invalid":
                rows.insert(1, dict(valid=False))
            elif kind == "gap":
                rows[1]["state_stamp"] = 10.1
            elif kind == "reset":
                rows[1]["reset_epoch"] = 1
            else:
                rows[1]["observer_update_count"] = 3
            monitor = tool.Monitor(rows, 2, .05)
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "gap_or_reset"):
                monitor.height(10.01)

    def test_duplicates_and_extrapolation(self):
        first = monitor_row(10, 1)
        monitor = tool.Monitor([first, dict(first), monitor_row(10.02, 2)], 2, .05)
        self.assertEqual(monitor.counts["identical_duplicate"], 1)
        with self.assertRaisesRegex(ValueError, "extrapolation"):
            monitor.height(10.03)
        conflicting = dict(first, observer_update_count=9)
        with self.assertRaisesRegex(ValueError, "conflicting"):
            tool.Monitor([first, conflicting], 2, .05)

    def test_epoch_not_header_known_error_and_replanning(self):
        h, s, a, monitor = fixture()
        h["header_stamp"] = 5000  # intentionally unsuitable as a prediction epoch
        later = dict(a, cycle_id=2, command_publish_stamp=100.05)
        report, matches, _ = tool.analyze([h], [s], [a, later], monitor,
                                         dict(task=[99.9, 102.5], goal_post5s=[102.5, 107.5]), 2)
        self.assertEqual(len(matches), 5)
        self.assertAlmostEqual(matches[0]["target_epoch"], 100.1)
        self.assertEqual(matches[0]["future_replans"], 1)
        m = report["groups"]["task_common_origins"]["2.0"]
        self.assertAlmostEqual(m["bias_mm"], 2)
        self.assertAlmostEqual(m["rmse_mm"], 2)
        self.assertAlmostEqual(m["rms_ratio"], 2)
        later["terminal_controller_intervened"] = True
        report, _, _ = tool.analyze([h], [s], [a, later], monitor,
                                    dict(task=[99.9, 102.5], goal_post5s=[102.5, 107.5]), 2)
        self.assertEqual(report["groups"]["task_common_origins"]["2.0"]["count"], 1)
        self.assertEqual(report["groups"]["task_common_origins_no_intervention"]["2.0"]["count"], 0)

    def test_smooth_failed_solve_wrong_epoch_and_initial_state_are_rejected(self):
        for kind, reason in (("smooth", "requires_full"), ("fail", "origin_command"),
                             ("epoch", "epoch_mismatch"), ("x0", "x0_snapshot")):
            h, s, a, _ = fixture()
            if kind == "smooth":
                h["slosh_enabled"] = False
                h["h_modal"] = [0.0]*61
            elif kind == "fail":
                a["solve_success"] = False
            elif kind == "epoch":
                s["solver_input_epoch"] += .01
            else:
                s["eta_x"] = .004
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, reason):
                tool.validate_cycle(h, s, a, 2)

    def test_availability_and_cross_goal_are_separate(self):
        h, s, a, monitor = fixture()
        h["horizon_available_stamp"] = h["solve_end_stamp"] = 100.15
        a["command_publish_stamp"] = 100.16
        report, matches, _ = tool.analyze([h], [s], [a], monitor,
                                         dict(task=[99.9, 100.4], goal_post5s=[100.4, 105.4]), 2)
        self.assertEqual(report["rejected_pairs"]["target_not_future_when_available"], 1)
        self.assertEqual(report["groups"]["task"]["0.3"]["count"], 1)
        self.assertEqual(report["groups"]["task"]["0.5"]["count"], 0)
        self.assertEqual(report["groups"]["cross_goal"]["0.5"]["count"], 1)

    def test_future_cannot_cross_reset_even_with_valid_target_bracket(self):
        h, s, a, _ = fixture()
        rows = [monitor_row(99.9+i*.02, i) for i in range(161)]
        for row in rows:
            if row["state_stamp"] >= 100.15:
                row["reset_epoch"] = 1
        monitor = tool.Monitor(rows, 2, .05)
        report, matches, _ = tool.analyze([h], [s], [a], monitor,
                                         dict(task=[99.9, 102.5], goal_post5s=[102.5, 107.5]), 2)
        self.assertEqual(len(matches), 1)
        self.assertEqual(report["rejected_pairs"]["future_crosses_monitor_gap_or_reset"], 4)

    def test_duplicate_cycle_excluded(self):
        h, s, a, monitor = fixture()
        report, matches, _ = tool.analyze([h, copy.deepcopy(h)], [s], [a], monitor,
                                         dict(task=[99.9, 102.5], goal_post5s=[102.5, 107.5]), 2)
        self.assertEqual(matches, [])
        self.assertEqual(report["rejected_origins"]["duplicate_cycle"], 1)


if __name__ == "__main__":
    unittest.main()
