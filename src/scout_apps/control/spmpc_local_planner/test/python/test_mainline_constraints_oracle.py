#!/usr/bin/env python3
"""Contract tests for the Stage 3-D explicit-bound constraints oracle."""

from __future__ import annotations

import ast
import sys
import unittest
from dataclasses import replace
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.constraints_oracle import (
    CONSTRAINT_RESIDUAL_ORDER,
    CONSTRAINT_SCHEMA,
    CONSTRAINT_VALUE_STATUS,
    ConstraintBounds,
    ConstraintOracleError,
    evaluate_constraint_residuals,
    evaluate_constraints,
)
from acados.mainline.cost_schedule import (
    ExperimentCondition,
    build_liquid_cost_schedule,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.discrete_dynamics import IssuedCommandValues
from acados.mainline.parameter_values import (
    REFERENCE_SPEED_COUNT,
    ActuatorResponseParameters,
    CommonStageParameterValues,
    NormalizationParameters,
    ReferenceHorizonParameters,
    ReferencePolynomialParameters,
    RunningWeightParameters,
    SloshParameters,
    TerminalWeightParameters,
)
from acados.mainline.runtime_parameter_assembler import (
    assemble_runtime_stage_parameters,
)
from acados.mainline.runtime_schedule import (
    build_runtime_fractional_delay_schedule,
)

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


class IssuedCommandValuesChild(IssuedCommandValues):
    """Subclass used to prove that the oracle requires the exact type."""


def make_stage_parameters(capacity, development_layout) -> tuple[float, ...]:
    values = CommonStageParameterValues(
        actuator=ActuatorResponseParameters(0.12, 1.03, 0.45, 0.98),
        reference=ReferenceHorizonParameters(
            ReferencePolynomialParameters(
                0.0,
                2.0,
                (0.0, 2.0, 0.1, -0.05),
                (0.0, 0.2, 0.05, 0.02),
                0.5,
            ),
            tuple(
                0.25 - 0.1 * stage / float(REFERENCE_SPEED_COUNT - 1)
                for stage in range(REFERENCE_SPEED_COUNT)
            ),
        ),
        slosh=SloshParameters(5.0, 0.05, 1.1, 0.9, 0.01, 0.3),
        normalization=NormalizationParameters(
            0.1,
            0.2,
            0.5,
            1.0,
            0.5,
            0.8,
            1.2,
            2.0,
            3.0,
        ),
        running_weight=RunningWeightParameters(
            1.0,
            0.2,
            0.3,
            0.7,
            0.4,
            0.1,
            0.15,
            0.05,
            0.08,
        ),
        terminal_weight=TerminalWeightParameters(2.0, 0.4, 1.5, 0.6),
    )
    runtime = build_runtime_fractional_delay_schedule(
        capacity,
        0.1,
        0.2,
        1.0e-12,
        1.0e-12,
    )
    liquid = build_liquid_cost_schedule(
        ExperimentCondition.Bslosh,
        8,
        4.0,
        7.0,
    )
    snapshot = assemble_runtime_stage_parameters(
        capacity,
        development_layout,
        runtime,
        values,
        liquid,
    )
    return snapshot.stage_parameters[0]


class MainlineConstraintsOracleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.development_layout = build_development_layout(self.capacity)
        self.state = tuple(0.0 for _ in range(self.development_layout.NX))
        self.control = (0.4, -0.5, 0.7)
        self.issued = IssuedCommandValues(0.8, -0.6, 0.2, -0.3)
        self.bounds = ConstraintBounds(
            1.0,
            1.0,
            0.5,
            0.5,
            0.6,
            0.6,
            1.0,
        )

    def test_all_constraint_shapes_have_separate_residuals(self) -> None:
        result = evaluate_constraint_residuals(
            self.control,
            self.issued,
            self.bounds,
            self.development_layout,
        )
        self.assertEqual(
            tuple(name for name, _ in result.residuals),
            CONSTRAINT_RESIDUAL_ORDER,
        )
        expected = {
            "q_issue_v_lower": -1.8,
            "q_issue_v_upper": -0.2,
            "q_issue_omega_lower": -0.4,
            "q_issue_omega_upper": -1.6,
            "a_issue_lower": -0.7,
            "a_issue_upper": -0.3,
            "alpha_issue_lower": -0.2,
            "alpha_issue_upper": -0.8,
            "j_issue_v_lower": -1.0,
            "j_issue_v_upper": -0.2,
            "j_issue_omega_lower": -0.1,
            "j_issue_omega_upper": -1.1,
            "v_s_lower": -0.7,
            "v_s_upper": -0.3,
        }
        residuals = dict(result.residuals)
        for name, expected_value in expected.items():
            with self.subTest(name=name):
                self.assertAlmostEqual(residuals[name], expected_value)
        self.assertTrue(result.feasible)
        self.assertNotIn("liquid", result.to_dict()["residuals"])

    def test_lower_and_upper_residuals_are_not_abs_proxy(self) -> None:
        positive = evaluate_constraint_residuals(
            self.control,
            IssuedCommandValues(1.5, 0.0, 0.0, 0.0),
            self.bounds,
            self.development_layout,
        )
        negative = evaluate_constraint_residuals(
            self.control,
            IssuedCommandValues(-1.5, 0.0, 0.0, 0.0),
            self.bounds,
            self.development_layout,
        )
        positive_residuals = dict(positive.residuals)
        negative_residuals = dict(negative.residuals)
        self.assertEqual(positive_residuals["q_issue_v_upper"], 0.5)
        self.assertEqual(positive_residuals["q_issue_v_lower"], -2.5)
        self.assertEqual(negative_residuals["q_issue_v_upper"], -2.5)
        self.assertEqual(negative_residuals["q_issue_v_lower"], 0.5)
        self.assertFalse(positive.feasible)
        self.assertFalse(negative.feasible)

    def test_v_s_has_nonnegative_and_explicit_upper_bound(self) -> None:
        for progress_velocity, expected_lower, expected_upper in (
            (0.0, 0.0, -1.0),
            (1.0, -1.0, 0.0),
            (-0.1, 0.1, -1.1),
            (1.1, -1.1, 0.1),
        ):
            with self.subTest(progress_velocity=progress_velocity):
                result = evaluate_constraint_residuals(
                    (0.0, 0.0, progress_velocity),
                    self.issued,
                    self.bounds,
                    self.development_layout,
                )
                residuals = dict(result.residuals)
                self.assertAlmostEqual(residuals["v_s_lower"], expected_lower)
                self.assertAlmostEqual(residuals["v_s_upper"], expected_upper)
                self.assertEqual(
                    result.feasible,
                    0.0 <= progress_velocity <= self.bounds.v_s_max,
                )

    def test_boundary_values_are_feasible(self) -> None:
        result = evaluate_constraint_residuals(
            (-0.6, 0.6, 0.0),
            IssuedCommandValues(-1.0, 1.0, -0.5, 0.5),
            self.bounds,
            self.development_layout,
        )
        self.assertTrue(result.feasible)
        self.assertTrue(all(value <= 0.0 for _, value in result.residuals))
        self.assertEqual(dict(result.residuals)["v_s_lower"], 0.0)

    def test_every_bound_is_required_and_strictly_positive_finite_float(self) -> None:
        with self.assertRaises(TypeError):
            ConstraintBounds(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)  # type: ignore[call-arg]
        for index in range(7):
            values = [1.0] * 7
            for invalid in (0.0, -1.0, float("nan"), float("inf"), 1):
                values[index] = invalid
                with (
                    self.subTest(index=index, invalid=invalid),
                    self.assertRaises(ConstraintOracleError),
                ):
                    ConstraintBounds(*values)
                values[index] = 1.0

        snapshot = self.bounds.to_dict()
        self.assertEqual(snapshot["constraint_schema"], CONSTRAINT_SCHEMA)
        self.assertEqual(snapshot["value_status"], CONSTRAINT_VALUE_STATUS)
        self.assertEqual(set(snapshot["values"]), set(vars(self.bounds)))

    def test_issued_requires_exact_finite_float_dataclass(self) -> None:
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraint_residuals(
                self.control,
                IssuedCommandValuesChild(0.0, 0.0, 0.0, 0.0),
                self.bounds,
                self.development_layout,
            )
        for value in (1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ConstraintOracleError):
                evaluate_constraint_residuals(
                    self.control,
                    replace(self.issued, q_issue_v=value),  # type: ignore[arg-type]
                    self.bounds,
                    self.development_layout,
                )

    def test_control_and_bounds_are_strictly_typed(self) -> None:
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraint_residuals(
                [0.0, 0.0, 0.0],  # type: ignore[arg-type]
                self.issued,
                self.bounds,
                self.development_layout,
            )
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraint_residuals(
                (0.0, 0.0, 1),  # type: ignore[arg-type]
                self.issued,
                self.bounds,
                self.development_layout,
            )
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraint_residuals(
                self.control,
                self.issued,
                replace(self.bounds, v_s_max=1),  # type: ignore[arg-type]
                self.development_layout,
            )

    def test_nonfinite_derived_residual_fails_closed(self) -> None:
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraint_residuals(
                self.control,
                IssuedCommandValues(-1.7e308, 0.0, 0.0, 0.0),
                replace(self.bounds, q_issue_v_max=1.7e308),
                self.development_layout,
            )

    def test_wrapper_builds_map_from_valid_d2b_parameters(self) -> None:
        result = evaluate_constraints(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            make_stage_parameters(self.capacity, self.development_layout),
            self.bounds,
        )
        self.assertTrue(result.feasible)

    def test_wrapper_uses_map_for_state_validation(self) -> None:
        with self.assertRaises(ConstraintOracleError):
            evaluate_constraints(
                self.capacity,
                self.development_layout,
                list(self.state),  # type: ignore[arg-type]
                self.control,
                make_stage_parameters(self.capacity, self.development_layout),
                self.bounds,
            )

    def test_module_has_no_ros_casadi_or_legacy_authority_dependency(self) -> None:
        module_path = (
            PACKAGE_ROOT / "scripts" / "acados" / "mainline" / "constraints_oracle.py"
        )
        tree = ast.parse(module_path.read_text(encoding="utf-8"))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)
        self.assertFalse(
            any(name in {"casadi", "rospy", "rospkg"} for name in imported)
        )
        self.assertFalse(
            any(name.endswith((".layout", ".manifest")) for name in imported)
        )


if __name__ == "__main__":
    unittest.main()
