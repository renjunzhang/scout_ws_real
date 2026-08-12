#!/usr/bin/env python3
"""Synthetic and negative tests for the relative-SE(3) TF witness."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_motion_bridge_v1 as bridge  # noqa: E402
import r8_liquid_s5a1_ros1_signal_extractor_v5 as extractor  # noqa: E402


def row(stamp: int, position, quaternion, *, tf: bool = False):
    value = {"x_m": position[0], "y_m": position[1], "z_m": position[2],
             "qx": quaternion[0], "qy": quaternion[1], "qz": quaternion[2],
             "qw": quaternion[3]}
    if tf:
        value.update(topic="/tf", parent="odom", child="base_footprint", stamp_ns=stamp)
    else:
        value["odom_header_t_ns"] = stamp
    return value


class ExtractorV5Tests(unittest.TestCase):
    def test_fixed_world_offset_and_rotation_pass_relative_se3(self) -> None:
        fixed_q = bridge.intrinsic_zyx_to_quaternion(0.7, -0.1, 0.05)
        fixed_p = (3.5, -1.25, 0.4)
        odom, tf = [], []
        for index in range(5):
            stamp = index * 100_000_000
            p = (0.2 * index, 0.03 * index, 0.0)
            q = bridge.intrinsic_zyx_to_quaternion(0.04 * index, 0.0, 0.0)
            transformed_p_delta = bridge.rotate_vector(fixed_q, p)
            transformed_p = tuple(fixed_p[axis] + transformed_p_delta[axis] for axis in range(3))
            transformed_q = bridge.quaternion_multiply(fixed_q, q)
            odom.append(row(stamp, p, q))
            tf.append(row(stamp, transformed_p, transformed_q, tf=True))
        report = extractor.tf_cross_check(
            odom, tf, maximum_time_delta_s=0.5,
            maximum_position_delta_m=0.05, maximum_angular_delta_rad=0.1)
        self.assertEqual(report["status"], "PASS_ALIGNMENT_WITNESS")
        self.assertGreater(report["absolute_maximum_position_delta_m"], 1.0)
        self.assertLess(report["relative_maximum_translation_delta_m"], 1e-12)
        self.assertLess(report["relative_maximum_angular_delta_rad"], 1e-12)
        self.assertFalse(report["tf_used_as_motion"])

    def test_real_diagnostic_boundary_is_frozen_without_relaxation(self) -> None:
        value = extractor.REAL_DIAGNOSTIC_WITNESS
        self.assertEqual(value["absolute_maximum_position_delta_m"], 3.9934499955863565)
        self.assertEqual(value["maximum_time_delta_s"], 0.005)
        self.assertEqual(value["absolute_maximum_angular_delta_rad"], 0.023228249162941604)
        self.assertEqual(value["relative_maximum_translation_delta_m"], 0.03995630546880967)
        self.assertEqual(value["relative_rms_translation_delta_m"], 0.022725548786785066)
        self.assertEqual(value["relative_maximum_angular_delta_rad"], 0.003657518055773045)
        self.assertEqual((extractor.MAXIMUM_POSITION_DELTA_M,
                          extractor.MAXIMUM_ANGULAR_DELTA_RAD,
                          extractor.MAXIMUM_TIME_DELTA_S), (0.05, 0.1, 0.5))

    def test_relative_translation_over_original_threshold_fails(self) -> None:
        identity = (0.0, 0.0, 0.0, 1.0)
        odom = [row(0, (0, 0, 0), identity), row(100_000_000, (1, 0, 0), identity)]
        tf = [row(0, (3, 0, 0), identity, tf=True),
              row(100_000_000, (4.051, 0, 0), identity, tf=True)]
        with self.assertRaisesRegex(extractor.SignalExtractionError, "relative TF"):
            extractor.tf_cross_check(odom, tf, maximum_time_delta_s=0.5,
                                     maximum_position_delta_m=0.05,
                                     maximum_angular_delta_rad=0.1)

    def test_relative_angle_and_time_over_original_threshold_fail(self) -> None:
        identity = (0.0, 0.0, 0.0, 1.0)
        angle = bridge.intrinsic_zyx_to_quaternion(0.101, 0.0, 0.0)
        odom = [row(0, (0, 0, 0), identity), row(1_000_000_000, (1, 0, 0), identity)]
        angular_tf = [row(0, (0, 0, 0), identity, tf=True),
                      row(1_000_000_000, (1, 0, 0), angle, tf=True)]
        with self.assertRaises(extractor.SignalExtractionError):
            extractor.tf_cross_check(odom, angular_tf, maximum_time_delta_s=0.5,
                                     maximum_position_delta_m=0.05,
                                     maximum_angular_delta_rad=0.1)
        late_tf = [row(600_000_001, (0, 0, 0), identity, tf=True),
                   row(1_600_000_001, (1, 0, 0), identity, tf=True)]
        with self.assertRaises(extractor.SignalExtractionError):
            extractor.tf_cross_check(odom, late_tf, maximum_time_delta_s=0.5,
                                     maximum_position_delta_m=0.05,
                                     maximum_angular_delta_rad=0.1)

    def test_self_check_is_static(self) -> None:
        report = extractor.self_check()
        self.assertTrue(report["thresholds_unchanged"])
        self.assertFalse(report["real_bag_read"])
        self.assertFalse(report["files_written"])


if __name__ == "__main__":
    unittest.main()
