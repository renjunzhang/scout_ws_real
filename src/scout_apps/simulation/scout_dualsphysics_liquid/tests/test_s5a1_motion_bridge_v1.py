#!/usr/bin/env python3
"""Static, mathematical, CSV, and negative tests for S5A1 bridge v1."""

from __future__ import annotations

import ast
import math
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPT_PATH = PACKAGE_ROOT / "scripts/r8_liquid_s5a1_motion_bridge_v1.py"
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_s5a1_motion_bridge_v1 as bridge  # noqa: E402


def yaw_quaternion(degrees: float) -> bridge.Quaternion:
    return bridge.intrinsic_zyx_to_quaternion(math.radians(degrees), 0.0, 0.0)


def odom(
    time_s: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    quaternion: bridge.Quaternion = (0.0, 0.0, 0.0, 1.0),
) -> bridge.OdomPose:
    return bridge.OdomPose(time_s, x, y, z, *quaternion)


class S5A1MotionBridgeV1Tests(unittest.TestCase):
    def test_module_uses_only_python_standard_library(self) -> None:
        tree = ast.parse(SCRIPT_PATH.read_text(encoding="utf-8"), filename=str(SCRIPT_PATH))
        roots: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                roots.add(node.module.split(".")[0])
        self.assertEqual(
            roots,
            {
                "__future__",
                "argparse",
                "csv",
                "dataclasses",
                "io",
                "json",
                "math",
                "os",
                "pathlib",
                "stat",
                "sys",
                "typing",
            },
        )

    def test_odom_and_mount_are_converted_to_exact_relative_identity(self) -> None:
        start = yaw_quaternion(90.0)
        poses = (
            odom(50.0, x=5.0, y=-2.0, z=1.0, quaternion=start),
            odom(51.0, x=5.0, y=-1.0, z=1.0, quaternion=start),
        )
        motion = bridge.make_relative_motion(
            poses,
            maximum_gap_s=1.0,
            base_to_container=bridge.RigidPose(x_m=0.25),
        )
        self.assertEqual(motion[0], bridge.MotionSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
        # World +Y is the start container frame's +X after inverse yaw(90).
        self.assertAlmostEqual(motion[1].x_rel_m, 1.0, places=12)
        self.assertAlmostEqual(motion[1].y_rel_m, 0.0, places=12)
        self.assertAlmostEqual(motion[1].z_rel_m, 0.0, places=12)
        self.assertLess(
            bridge.quaternion_angular_distance(
                (motion[1].qx_rel, motion[1].qy_rel, motion[1].qz_rel, motion[1].qw_rel),
                (0.0, 0.0, 0.0, 1.0),
            ),
            1.0e-12,
        )

    def test_quaternions_are_normalized_and_sign_continuous(self) -> None:
        second = yaw_quaternion(20.0)
        poses = (
            odom(10.0, quaternion=(0.0, 0.0, 0.0, 2.0)),
            odom(11.0, quaternion=tuple(-3.0 * value for value in second)),
        )
        motion = bridge.make_relative_motion(poses, maximum_gap_s=1.0)
        first_q = (motion[0].qx_rel, motion[0].qy_rel, motion[0].qz_rel, motion[0].qw_rel)
        second_q = (motion[1].qx_rel, motion[1].qy_rel, motion[1].qz_rel, motion[1].qw_rel)
        self.assertAlmostEqual(sum(value * value for value in second_q), 1.0, places=14)
        self.assertGreaterEqual(sum(a * b for a, b in zip(first_q, second_q)), 0.0)
        self.assertLess(bridge.quaternion_angular_distance(second_q, second), 1.0e-12)

    def test_position_is_linear_and_orientation_uses_slerp(self) -> None:
        motion = bridge.make_relative_motion(
            (
                odom(20.0, x=1.0, quaternion=yaw_quaternion(30.0)),
                odom(22.0, x=3.0, quaternion=yaw_quaternion(120.0)),
            ),
            maximum_gap_s=2.0,
        )
        sampled = bridge.resample_motion(motion, (0.0, 1.0, 2.0))
        # The world-X displacement is expressed in the initial yaw(30) frame.
        self.assertAlmostEqual(sampled[1].x_rel_m, math.sqrt(3.0) / 2.0, places=12)
        self.assertAlmostEqual(sampled[1].y_rel_m, -0.5, places=12)
        expected_mid = yaw_quaternion(45.0)
        actual_mid = (
            sampled[1].qx_rel,
            sampled[1].qy_rel,
            sampled[1].qz_rel,
            sampled[1].qw_rel,
        )
        self.assertLess(bridge.quaternion_angular_distance(actual_mid, expected_mid), 1.0e-12)

    def test_intrinsic_zyx_yaw_is_continuously_unwrapped(self) -> None:
        samples = (
            bridge.MotionSample(0.0, 0.0, 0.0, 0.0, *yaw_quaternion(0.0)),
            bridge.MotionSample(1.0, 0.1, 0.0, 0.0, *yaw_quaternion(170.0)),
            bridge.MotionSample(2.0, 0.2, 0.0, 0.0, *yaw_quaternion(-170.0)),
        )
        rows = bridge.build_solver_path(samples)
        self.assertAlmostEqual(rows[0].ang1_yaw_z_deg, 0.0, places=12)
        self.assertAlmostEqual(rows[1].ang1_yaw_z_deg, 170.0, places=12)
        self.assertAlmostEqual(rows[2].ang1_yaw_z_deg, 190.0, places=12)

    def test_solver_csv_has_exactly_seven_fields_and_round_trips(self) -> None:
        motion = (
            bridge.MotionSample(0.0, 0.0, 0.0, 0.0, *yaw_quaternion(0.0)),
            bridge.MotionSample(1.0, 1.0, 2.0, 3.0, *yaw_quaternion(30.0)),
        )
        rows = bridge.build_solver_path(motion)
        rendered = bridge.render_solver_path_csv(rows)
        data_lines = rendered.splitlines()[1:]
        self.assertEqual(len(data_lines), 2)
        self.assertTrue(all(len(line.split(",")) == 7 for line in data_lines))
        self.assertEqual(bridge.parse_solver_path_csv(rendered, maximum_gap_s=1.0), rows)

    def test_rotation_matrix_and_quaternion_round_trip_are_bounded(self) -> None:
        orientations = (
            bridge.intrinsic_zyx_to_quaternion(0.0, 0.0, 0.0),
            bridge.intrinsic_zyx_to_quaternion(math.radians(20), math.radians(-15), math.radians(10)),
            bridge.intrinsic_zyx_to_quaternion(math.radians(40), math.radians(-10), math.radians(-5)),
        )
        motion = tuple(
            bridge.MotionSample(float(index), float(index), 0.0, 0.0, *orientation)
            for index, orientation in enumerate(orientations)
        )
        rows = bridge.build_solver_path(motion)
        report = bridge.evaluate_solver_round_trip(motion, rows)
        self.assertEqual(report.sample_count, 3)
        self.assertLessEqual(report.max_position_error_m, 1.0e-15)
        self.assertLessEqual(report.max_quaternion_angular_distance_rad, 3.0e-8)
        self.assertLessEqual(report.max_rotation_matrix_angular_distance_rad, 3.0e-8)

    def test_solver_linear_euler_queries_are_compared_to_canonical_slerp(self) -> None:
        motion = (
            bridge.MotionSample(0.0, 0.0, 0.0, 0.0, *yaw_quaternion(0.0)),
            bridge.MotionSample(2.0, 2.0, 0.0, 0.0, *yaw_quaternion(60.0)),
        )
        rows = bridge.build_solver_path(motion)
        report = bridge.evaluate_solver_round_trip(
            motion, rows, query_times_s=(0.0, 0.5, 1.0, 1.5, 2.0)
        )
        self.assertLessEqual(report.max_position_error_m, 1.0e-15)
        self.assertLessEqual(report.max_quaternion_angular_distance_rad, 3.0e-8)
        self.assertLessEqual(report.max_rotation_matrix_angular_distance_rad, 3.0e-8)

    def test_duplicate_backtrack_and_gap_are_rejected(self) -> None:
        cases = (
            ((odom(1.0), odom(1.0)), 1.0, "duplicate"),
            ((odom(2.0), odom(1.0)), 2.0, "backtrack"),
            ((odom(1.0), odom(3.1)), 2.0, "exceeds maximum"),
        )
        for poses, maximum_gap, expected in cases:
            with self.subTest(expected=expected):
                with self.assertRaisesRegex(bridge.MotionBridgeError, expected):
                    bridge.make_relative_motion(poses, maximum_gap_s=maximum_gap)

    def test_solver_csv_duplicate_backtrack_and_gap_are_rejected(self) -> None:
        cases = (
            ("0,0,0,0,0,0,0\n0,0,0,0,0,0,0\n", 1.0, "duplicate"),
            ("0,0,0,0,0,0,0\n2,0,0,0,0,0,0\n1,0,0,0,0,0,0\n", 2.0, "backtrack"),
            ("0,0,0,0,0,0,0\n2.1,0,0,0,0,0,0\n", 2.0, "exceeds maximum"),
        )
        for csv_text, maximum_gap, expected in cases:
            with self.subTest(expected=expected):
                with self.assertRaisesRegex(bridge.MotionBridgeError, expected):
                    bridge.parse_solver_path_csv(csv_text, maximum_gap_s=maximum_gap)

    def test_nonfinite_time_position_orientation_and_csv_are_rejected(self) -> None:
        invalid_poses = (
            (odom(math.nan), odom(1.0)),
            (odom(0.0, x=math.inf), odom(1.0)),
            (odom(0.0, quaternion=(0.0, math.nan, 0.0, 1.0)), odom(1.0)),
        )
        for poses in invalid_poses:
            with self.subTest(poses=poses):
                with self.assertRaisesRegex(bridge.MotionBridgeError, "non-finite"):
                    bridge.make_relative_motion(poses, maximum_gap_s=2.0)
        csv_text = bridge.SOLVER_PATH_COMMENT + "\n0,0,0,0,0,0,0\n1,0,0,0,nan,0,0\n"
        with self.assertRaisesRegex(bridge.MotionBridgeError, "non-finite"):
            bridge.parse_solver_path_csv(csv_text)

    def test_zero_and_too_small_quaternions_are_rejected(self) -> None:
        for quaternion in ((0.0, 0.0, 0.0, 0.0), (1.0e-20, 0.0, 0.0, 0.0)):
            with self.subTest(quaternion=quaternion):
                with self.assertRaisesRegex(bridge.MotionBridgeError, "norm is zero or too small"):
                    bridge.make_relative_motion(
                        (odom(0.0, quaternion=quaternion), odom(1.0)), maximum_gap_s=1.0
                    )

    def test_gimbal_lock_is_rejected_before_solver_path_generation(self) -> None:
        locked = bridge.intrinsic_zyx_to_quaternion(0.0, math.pi / 2.0, 0.0)
        samples = (
            bridge.MotionSample(0.0, 0.0, 0.0, 0.0, *yaw_quaternion(0.0)),
            bridge.MotionSample(1.0, 0.0, 0.0, 0.0, *locked),
        )
        with self.assertRaisesRegex(bridge.MotionBridgeError, "gimbal-lock"):
            bridge.build_solver_path(samples)

    def test_forbidden_signal_provenance_is_fail_closed(self) -> None:
        allowed = sorted(bridge.ALLOWED_MOTION_SIGNALS)
        report = bridge.validate_odom_only_provenance(
            primary_time_source=bridge.PRIMARY_TIME_SOURCE,
            pose_source=bridge.POSE_SOURCE,
            consumed_motion_signals=allowed,
        )
        self.assertIs(report["forbidden_signal_consumed"], False)
        forbidden = (
            "/cmd_vel",
            "/move_base/NavfnROS/plan",
            "/move_base/goal",
            "/sloshing/H_proxy",
            "/sloshing/H_modal",
            "/move_base/status",
            "/FixedProfile/lifecycle",
        )
        for signal in forbidden:
            with self.subTest(signal=signal):
                with self.assertRaisesRegex(bridge.MotionBridgeError, "forbidden-signal"):
                    bridge.validate_odom_only_provenance(
                        primary_time_source=bridge.PRIMARY_TIME_SOURCE,
                        pose_source=bridge.POSE_SOURCE,
                        consumed_motion_signals=allowed + [signal],
                    )

    def test_primary_time_pose_and_unknown_signal_provenance_are_rejected(self) -> None:
        allowed = sorted(bridge.ALLOWED_MOTION_SIGNALS)
        cases = (
            {"primary_time_source": "/clock", "pose_source": bridge.POSE_SOURCE, "signals": allowed},
            {"primary_time_source": bridge.PRIMARY_TIME_SOURCE, "pose_source": "/tf", "signals": allowed},
            {
                "primary_time_source": bridge.PRIMARY_TIME_SOURCE,
                "pose_source": bridge.POSE_SOURCE,
                "signals": allowed + ["/odom/twist"],
            },
        )
        for case in cases:
            with self.subTest(case=case):
                with self.assertRaises(bridge.MotionBridgeError):
                    bridge.validate_odom_only_provenance(
                        primary_time_source=case["primary_time_source"],
                        pose_source=case["pose_source"],
                        consumed_motion_signals=case["signals"],
                    )

    def test_malformed_extra_field_comment_blank_and_nonidentity_csv_are_rejected(self) -> None:
        invalid = (
            "0,0,0,0,0,0,0,8\n",
            "# wrong\n0,0,0,0,0,0,0\n",
            "0,0,0,0,0,0,0\n\n1,0,0,0,0,0,0\n",
            "0,0.1,0,0,0,0,0\n1,0,0,0,0,0,0\n",
        )
        for csv_text in invalid:
            with self.subTest(csv_text=csv_text):
                with self.assertRaises(bridge.MotionBridgeError):
                    bridge.parse_solver_path_csv(csv_text)

    def test_self_check_is_in_memory_only(self) -> None:
        result = bridge.self_check()
        self.assertEqual(result["status"], "PASS_S5A1_MOTION_BRIDGE_V1_SELF_CHECK")
        self.assertIs(result["bag_read"], False)
        self.assertIs(result["files_written"], False)
        self.assertIs(result["solver_executed"], False)
        self.assertIs(result["gpu_exposed"], False)
        self.assertIs(result["sudo_used"], False)
        self.assertIs(result["network_used"], False)


if __name__ == "__main__":
    unittest.main()
