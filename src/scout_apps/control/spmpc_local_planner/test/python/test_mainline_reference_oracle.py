"""Contract tests for the backend-neutral Stage 3-D reference oracle."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.reference_oracle import (
    ReferenceOracleError,
    evaluate_reference,
    evaluate_reference_at_s,
)
from acados.mainline.solver_parameter_layout import build_solver_parameter_layout

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


class MainlineReferenceOracleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.development_layout = build_development_layout(self.capacity)
        self.parameter_layout = build_solver_parameter_layout(
            self.capacity, self.development_layout
        )

    def stage(self, speed: float = 0.7) -> tuple[float, ...]:
        values = [0.0] * self.parameter_layout.NP
        offsets = self.parameter_layout.parameter_offsets
        values[offsets["ref_s_origin"]] = 10.0
        values[offsets["ref_s_scale"]] = 2.0
        for index, value in enumerate((1.0, 2.0, 3.0, 4.0)):
            values[offsets[f"ref_x_coeff[{index}]"]] = value
        for index, value in enumerate((-1.0, 1.0, 1.0, 1.0)):
            values[offsets[f"ref_y_coeff[{index}]"]] = value
        values[offsets["ref_speed"]] = speed
        return tuple(float(value) for value in values)

    def state(self, s: float, px: float = 0.0, py: float = 0.0) -> tuple[float, ...]:
        values = [0.0] * self.development_layout.NX
        offsets = self.development_layout.state_offsets
        values[offsets["s"]] = s
        values[offsets["px"]] = px
        values[offsets["py"]] = py
        return tuple(float(value) for value in values)

    def test_normalized_cubic_chain_rule_and_heading(self) -> None:
        stage = self.stage()
        result = evaluate_reference_at_s(10.5, stage, self.parameter_layout)
        xi = 0.25
        expected_x = 1.0 + 2.0 * xi + 3.0 * xi**2 + 4.0 * xi**3
        expected_y = -1.0 + xi + xi**2 + xi**3
        expected_dx = (2.0 + 6.0 * xi + 12.0 * xi**2) / 2.0
        expected_dy = (1.0 + 2.0 * xi + 3.0 * xi**2) / 2.0
        self.assertAlmostEqual(result.xi, xi)
        self.assertAlmostEqual(result.x_ref, expected_x)
        self.assertAlmostEqual(result.y_ref, expected_y)
        self.assertAlmostEqual(result.dx_ref_ds, expected_dx)
        self.assertAlmostEqual(result.dy_ref_ds, expected_dy)
        self.assertAlmostEqual(result.psi_ref, math.atan2(expected_dy, expected_dx))
        self.assertEqual(result.ref_speed, 0.7)

    def test_tracking_errors_use_frozen_mpcc_signs(self) -> None:
        stage = self.stage()
        geometry = evaluate_reference_at_s(10.0, stage, self.parameter_layout)
        result = evaluate_reference(
            self.state(10.0, geometry.x_ref + 2.0, geometry.y_ref + 3.0),
            stage,
            self.development_layout,
            self.parameter_layout,
        )
        expected_contour = (
            math.sin(geometry.psi_ref) * 2.0 - math.cos(geometry.psi_ref) * 3.0
        )
        expected_lag = (
            -math.cos(geometry.psi_ref) * 2.0 - math.sin(geometry.psi_ref) * 3.0
        )
        self.assertAlmostEqual(result.e_contour, expected_contour)
        self.assertAlmostEqual(result.e_lag, expected_lag)

    def test_speed_is_stage_parameter_not_geometry(self) -> None:
        first = evaluate_reference_at_s(10.0, self.stage(0.2), self.parameter_layout)
        last = evaluate_reference_at_s(10.0, self.stage(0.9), self.parameter_layout)
        self.assertEqual(first.ref_speed, 0.2)
        self.assertEqual(last.ref_speed, 0.9)
        self.assertEqual(first.x_ref, last.x_ref)

    def test_effective_domain_accepts_boundaries_and_rejects_extrapolation(
        self,
    ) -> None:
        stage = self.stage()
        for s in (10.0, 12.0):
            with self.subTest(s=s):
                evaluate_reference_at_s(s, stage, self.parameter_layout)
        for s in (9.999999999, 12.000000001, float("nan"), float("inf")):
            with self.subTest(s=s), self.assertRaises(ReferenceOracleError):
                evaluate_reference_at_s(s, stage, self.parameter_layout)

    def test_nonpositive_scale_and_degenerate_tangent_fail_closed(self) -> None:
        stage = list(self.stage())
        offset = self.parameter_layout.parameter_offsets
        stage[offset["ref_s_scale"]] = 0.0
        with self.assertRaises(ReferenceOracleError):
            evaluate_reference_at_s(10.0, tuple(stage), self.parameter_layout)

        stage = list(self.stage())
        stage[offset["ref_speed"]] = -0.1
        with self.assertRaises(ReferenceOracleError):
            evaluate_reference_at_s(10.0, tuple(stage), self.parameter_layout)

        stage = list(self.stage())
        for index in range(1, 4):
            stage[offset[f"ref_x_coeff[{index}]"]] = 0.0
            stage[offset[f"ref_y_coeff[{index}]"]] = 0.0
        with self.assertRaises(ReferenceOracleError):
            evaluate_reference_at_s(10.0, tuple(stage), self.parameter_layout)

    def test_inputs_are_strict_typed(self) -> None:
        with self.assertRaises(ReferenceOracleError):
            evaluate_reference_at_s(10, self.stage(), self.parameter_layout)  # type: ignore[arg-type]
        with self.assertRaises(ReferenceOracleError):
            evaluate_reference_at_s(10.0, list(self.stage()), self.parameter_layout)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
