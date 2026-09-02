"""Contract tests for the Stage 3-D cost oracle."""

from __future__ import annotations

import ast
import inspect
import math
import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.cost_oracle import (
    CostOracleError,
    evaluate_boundary_cost,
    evaluate_stage_cost,
    evaluate_terminal_cost,
    liquid_state_cost,
)
from acados.mainline.cost_schedule import (
    ExperimentCondition,
    build_liquid_cost_schedule,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.discrete_dynamics import evaluate_discrete_map
from acados.mainline.parameter_values import (
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
from acados.mainline.runtime_schedule import build_runtime_fractional_delay_schedule
from acados.mainline.solver_parameter_layout import build_solver_parameter_layout

CAPACITY_PATH = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


def _make_values() -> CommonStageParameterValues:
    return CommonStageParameterValues(
        actuator=ActuatorResponseParameters(0.12, 1.03, 0.45, 0.98),
        reference=ReferenceHorizonParameters(
            polynomial=ReferencePolynomialParameters(
                ref_s_origin=2.0,
                ref_s_scale=0.8,
                ref_x_coeff=(0.1, 0.8, 0.05, -0.02),
                ref_y_coeff=(-0.2, 0.1, 0.03, 0.01),
                minimum_tangent_norm_per_s=0.5,
            ),
            ref_speed=tuple(0.2 - 0.1 * stage / 60.0 for stage in range(61)),
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


class MainlineCostOracleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY_PATH)
        self.development_layout = build_development_layout(self.capacity)
        self.parameter_layout = build_solver_parameter_layout(
            self.capacity, self.development_layout
        )
        runtime = build_runtime_fractional_delay_schedule(
            self.capacity, 0.05, 0.07, 1.0e-12, 1.0e-12
        )
        values = _make_values()
        self.b0 = assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            runtime,
            values,
            build_liquid_cost_schedule(ExperimentCondition.B0, 8, 4.0, 7.0),
        )
        self.bslosh = assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            runtime,
            values,
            build_liquid_cost_schedule(ExperimentCondition.Bslosh, 8, 4.0, 7.0),
        )
        self.state = self._state((0.1, 0.2, 0.3, 0.4))
        self.control = (0.1, 0.2, 0.3)

    def _state(self, liquid: tuple[float, float, float, float]) -> tuple[float, ...]:
        offsets = self.development_layout.state_offsets
        values = [0.0] * self.development_layout.NX
        values[offsets["px"]] = 0.4
        values[offsets["py"]] = -0.1
        values[offsets["s"]] = 2.2
        values[offsets["v_actual"]] = 0.25
        values[offsets["omega_actual"]] = 0.04
        values[offsets["q_prev_v"]] = 0.2
        values[offsets["q_prev_omega"]] = 0.1
        values[offsets["a_prev"]] = 0.1
        values[offsets["alpha_prev"]] = -0.02
        for name, value in zip(("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"), liquid):
            values[offsets[name]] = value
        return tuple(values)

    def _map(self, stage_parameters: tuple[float, ...]):
        return evaluate_discrete_map(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            stage_parameters,
        )

    def _set_parameter(
        self, row: tuple[float, ...], name: str, value: float
    ) -> tuple[float, ...]:
        result = list(row)
        result[self.parameter_layout.parameter_offsets[name]] = value
        return tuple(result)

    def test_robot_running_cost_divides_by_horizon_exactly_once(self) -> None:
        row = self.b0.stage_parameters[0]
        for name in (
            "weight_contour",
            "weight_lag",
            "weight_progress",
            "weight_v_actual",
            "weight_v_s",
            "weight_a_issue",
            "weight_alpha_issue",
            "weight_jerk_v",
            "weight_jerk_omega",
        ):
            row = self._set_parameter(row, name, 0.0)
        row = self._set_parameter(row, "weight_a_issue", 1.0)
        row = self._set_parameter(row, "norm_a_issue", 2.0)
        result = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            row,
        )
        issued_a = result.map_evaluation.issued.a_issue
        expected = (issued_a / 2.0) ** 2 / self.development_layout.horizon_steps
        self.assertAlmostEqual(result.robot_running_cost, expected)
        self.assertNotAlmostEqual(result.robot_running_cost, expected / 60.0)

    def test_running_liquid_uses_map_right_endpoint_without_dt_or_horizon_scale(
        self,
    ) -> None:
        row = self.bslosh.stage_parameters[0]
        result_map = self._map(row)
        self.assertNotEqual(result_map.next_state, self.state)
        result = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            row,
        )
        coefficient = row[self.parameter_layout.parameter_offsets["liquid_run_coeff"]]
        expected = coefficient * liquid_state_cost(
            result_map.next_state,
            row,
            self.development_layout,
            self.parameter_layout,
            running=True,
        )
        self.assertAlmostEqual(result.liquid_running_cost, expected)
        self.assertNotAlmostEqual(result.liquid_running_cost, expected / 60.0)
        self.assertNotAlmostEqual(
            result.liquid_running_cost,
            expected * float(self.development_layout.release_period_sec),
        )
        self.assertNotAlmostEqual(
            result.liquid_running_cost,
            coefficient
            * liquid_state_cost(
                self.state,
                row,
                self.development_layout,
                self.parameter_layout,
                running=True,
            ),
        )

    def test_boundary_uses_shooting_state_and_fixed_velocity_ratio(self) -> None:
        row = self.bslosh.stage_parameters[8]
        row = self._set_parameter(row, "slosh_running_eta_dot_ratio", 11.0)
        expected = row[
            self.parameter_layout.parameter_offsets["liquid_boundary_coeff"]
        ] * (
            liquid_state_cost(
                self.state,
                row,
                self.development_layout,
                self.parameter_layout,
                running=False,
            )
        )
        result = evaluate_boundary_cost(
            self.state, row, self.development_layout, self.parameter_layout
        )
        self.assertAlmostEqual(result, expected)
        running_ratio_cost = liquid_state_cost(
            self.state,
            row,
            self.development_layout,
            self.parameter_layout,
            running=True,
        )
        self.assertNotAlmostEqual(
            result,
            row[self.parameter_layout.parameter_offsets["liquid_boundary_coeff"]]
            * running_ratio_cost,
        )

    def test_ordinary_stage_reports_boundary_separately_and_sums_all_terms(
        self,
    ) -> None:
        row = self.bslosh.stage_parameters[8]
        result = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            row,
        )
        expected_boundary = evaluate_boundary_cost(
            self.state, row, self.development_layout, self.parameter_layout
        )
        self.assertAlmostEqual(result.liquid_boundary_cost, expected_boundary)
        self.assertAlmostEqual(
            result.total_cost,
            result.robot_running_cost
            + result.liquid_running_cost
            + result.liquid_boundary_cost,
        )
        self.assertGreater(result.liquid_boundary_cost, 0.0)

    def test_terminal_has_state_and_parameter_signature_only_and_no_liquid_term(
        self,
    ) -> None:
        signature = inspect.signature(evaluate_terminal_cost)
        self.assertNotIn("control", signature.parameters)
        row = self.bslosh.stage_parameters[-1]
        baseline = evaluate_terminal_cost(
            self.state, row, self.development_layout, self.parameter_layout
        )
        altered_state = self._state((100.0, -80.0, 60.0, -40.0))
        altered_row = self._set_parameter(row, "liquid_run_coeff", 100.0)
        altered_row = self._set_parameter(altered_row, "liquid_boundary_coeff", 200.0)
        altered_row = self._set_parameter(
            altered_row, "slosh_running_eta_dot_ratio", 99.0
        )
        altered = evaluate_terminal_cost(
            altered_state, altered_row, self.development_layout, self.parameter_layout
        )
        self.assertEqual(altered.total_cost, baseline.total_cost)
        with self.assertRaises(TypeError):
            evaluate_terminal_cost(  # type: ignore[call-arg]
                self.state,
                row,
                self.development_layout,
                self.parameter_layout,
                self.control,
            )

    def test_stage_and_boundary_require_exact_typed_sources(self) -> None:
        self.assertNotIn(
            "map_evaluation", inspect.signature(evaluate_stage_cost).parameters
        )
        row = self.b0.stage_parameters[0]
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                object(),
                self.development_layout,
                self.parameter_layout,
                self.state,
                self.control,
                row,
            )  # type: ignore[arg-type]
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                self.capacity,
                object(),
                self.parameter_layout,
                self.state,
                self.control,
                row,
            )  # type: ignore[arg-type]
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                self.capacity,
                self.development_layout,
                object(),
                self.state,
                self.control,
                row,
            )  # type: ignore[arg-type]
        with self.assertRaises(CostOracleError):
            evaluate_boundary_cost(self.state, row, object(), self.parameter_layout)  # type: ignore[arg-type]
        with self.assertRaises(CostOracleError):
            evaluate_boundary_cost(self.state, row, self.development_layout, object())  # type: ignore[arg-type]

    def test_b0_and_bslosh_share_robot_terms_and_differ_only_in_liquid_terms(
        self,
    ) -> None:
        b0_running = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            self.b0.stage_parameters[0],
        )
        slosh_running = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            self.bslosh.stage_parameters[0],
        )
        self.assertAlmostEqual(
            b0_running.robot_running_cost, slosh_running.robot_running_cost
        )
        self.assertEqual(b0_running.liquid_boundary_cost, 0.0)
        self.assertEqual(b0_running.liquid_running_cost, 0.0)
        self.assertGreater(slosh_running.liquid_running_cost, 0.0)
        self.assertAlmostEqual(
            slosh_running.total_cost - b0_running.total_cost,
            slosh_running.liquid_running_cost,
        )

        b0_boundary = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            self.b0.stage_parameters[8],
        )
        slosh_boundary = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            self.bslosh.stage_parameters[8],
        )
        self.assertAlmostEqual(
            b0_boundary.robot_running_cost, slosh_boundary.robot_running_cost
        )
        self.assertEqual(b0_boundary.liquid_boundary_cost, 0.0)
        self.assertEqual(b0_boundary.liquid_running_cost, 0.0)
        self.assertGreater(slosh_boundary.liquid_boundary_cost, 0.0)

    def test_nonfinite_zero_normalization_and_bad_tuple_fail_closed(self) -> None:
        row = self.b0.stage_parameters[0]
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                self.capacity,
                self.development_layout,
                self.parameter_layout,
                self.state,
                self.control,
                row[:-1],
            )
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                self.capacity,
                self.development_layout,
                self.parameter_layout,
                self.state,
                self.control,
                self._set_parameter(row, "norm_contour", 0.0),
            )
        invalid_state = list(self.state)
        invalid_state[0] = math.inf
        with self.assertRaises(CostOracleError):
            evaluate_boundary_cost(
                tuple(invalid_state),
                self.bslosh.stage_parameters[8],
                self.development_layout,
                self.parameter_layout,
            )

    def test_module_has_no_ros_casadi_or_legacy_layout_dependencies(self) -> None:
        module_path = (
            PACKAGE_ROOT / "scripts" / "acados" / "mainline" / "cost_oracle.py"
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
