#!/usr/bin/env python3

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "analyze_mocap_spin_center.py"
)
SPEC = importlib.util.spec_from_file_location("spin_center", str(SCRIPT_PATH))
SPIN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SPIN)


def synthetic_pose(
    receive_start,
    yaw_start,
    yaw_end,
    center,
    offset,
    count=1200,
    noise_m=0.0002,
    offset_end=None,
):
    receive = np.linspace(receive_start, receive_start + 20.0, count)
    yaw = np.linspace(yaw_start, yaw_end, count)
    if offset_end is None:
        offsets = np.repeat(np.asarray(offset)[None, :], count, axis=0)
    else:
        offsets = np.linspace(offset, offset_end, count)
    xy = np.column_stack(
        (
            center[0] + np.cos(yaw) * offsets[:, 0] - np.sin(yaw) * offsets[:, 1],
            center[1] + np.sin(yaw) * offsets[:, 0] + np.cos(yaw) * offsets[:, 1],
        )
    )
    rng = np.random.default_rng(7)
    xy += rng.normal(scale=noise_m, size=xy.shape)
    return np.column_stack((receive, receive, xy, yaw))


class SpinCenterTest(unittest.TestCase):
    def fit(self, pose, interval=(0.0, 20.0), **kwargs):
        return SPIN.fit_direction(
            pose,
            interval,
            trim_sec=1.0,
            angle_samples=180,
            min_yaw_coverage_deg=180.0,
            min_samples=40,
            min_half_samples=15,
            max_fit_rmse_mm=3.0,
            max_fit_p95_mm=8.0,
            max_fit_peak_mm=15.0,
            max_center_half_drift_mm=20.0,
            max_half_offset_delta_mm=5.0,
            **kwargs,
        )

    def test_directions_recover_offset_with_different_world_centres(self):
        expected = np.array([0.24, -0.08])
        ccw = synthetic_pose(0.0, 0.0, 4.8, (1.3, -0.4), expected)
        cw = synthetic_pose(30.0, 0.0, -4.8, (-0.7, 2.1), expected)
        ccw_fit = self.fit(ccw)
        cw_fit = self.fit(cw, interval=(30.0, 50.0))
        self.assertEqual(ccw_fit["status"], "PASS")
        self.assertEqual(cw_fit["status"], "PASS")
        np.testing.assert_allclose(
            ccw_fit["r_center_to_tracker_tracker_m"], expected, atol=0.002
        )
        np.testing.assert_allclose(
            cw_fit["r_center_to_tracker_tracker_m"], expected, atol=0.002
        )
        status, comparison = SPIN.compare_directions(
            {"spin_ccw_hold": ccw_fit, "spin_cw_hold": cw_fit}, 5.0
        )
        self.assertEqual(status, "PASS")
        self.assertFalse(comparison["world_center_equality_required"])
        self.assertLess(comparison["direction_offset_delta_mm"], 2.0)

    def test_short_yaw_coverage_is_rejected_without_fit(self):
        pose = synthetic_pose(0.0, 0.0, np.deg2rad(150.0), (0.0, 0.0), (0.2, 0.1))
        result = self.fit(pose)
        self.assertEqual(result["status"], "INSUFFICIENT_DATA")
        self.assertIn("net yaw coverage below threshold", result["rejection_reasons"])
        self.assertNotIn("r_center_to_tracker_tracker_m", result)

    def test_half_spin_offset_drift_is_rejected(self):
        pose = synthetic_pose(
            0.0,
            0.0,
            5.0,
            (0.0, 0.0),
            np.array([0.2, 0.1]),
            noise_m=0.0,
            offset_end=np.array([0.22, 0.1]),
        )
        result = self.fit(pose)
        self.assertEqual(result["status"], "REJECT")
        self.assertIn("half-spin offset inconsistency exceeds threshold", result["rejection_reasons"])

    def test_segment_pairing_reports_missing_end(self):
        pairs, issues = SPIN.parse_segment_events(
            [(1.0, "START|spin_ccw_hold|v=0|omega=0.2"),
             (5.0, "END|spin_cw_hold|actual_duration=4")]
        )
        self.assertEqual(pairs, {})
        self.assertTrue(any("unmatched START" in issue for issue in issues))
        self.assertTrue(any("interval is missing" in issue for issue in issues))

    def test_main_event_issue_blocks_pass_and_returns_failure(self):
        expected = np.array([0.24, -0.08])
        pose = synthetic_pose(0.0, 0.0, 4.8, (1.3, -0.4), expected)
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "event_failure.json"
            with mock.patch.object(
                SPIN,
                "load_bag",
                return_value=(pose, [(0.0, "START|spin_ccw_hold")], ["world"]),
            ):
                code = SPIN.main(["--bag", "synthetic.bag", "--report", str(report_path)])
            report = json.loads(report_path.read_text())
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["failure_class"], "EVENT_INVALID")
        self.assertFalse(report["average_offset_is_valid_candidate"])

    def test_main_loader_error_is_nonzero_and_reviewable(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "loader_failure.json"
            with mock.patch.object(
                SPIN, "load_bag", side_effect=RuntimeError("mocked no ROS bag")
            ):
                code = SPIN.main(["--bag", "synthetic.bag", "--report", str(report_path)])
            report = json.loads(report_path.read_text())
        self.assertEqual(code, 1)
        self.assertEqual(report["status"], "ERROR")
        self.assertFalse(report.get("average_offset_is_valid_candidate", False))


if __name__ == "__main__":
    unittest.main()
