"""Contract and parity tests for the lazy Stage 3-D CasADi graph."""

from __future__ import annotations

import ast
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
MAINLINE_ROOT = SCRIPTS_ROOT / "acados" / "mainline"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.casadi_adapter import (
    CasadiDependencyError,
    CasadiGraphConstructionError,
    build_casadi_graph,
)
from acados.mainline.casadi_graph_contract import (
    DIAGNOSTIC_RESIDUAL_ROLE,
    graph_semantic_sha256,
)
from acados.mainline.constraints_oracle import (
    CONSTRAINT_RESIDUAL_ORDER,
    CONSTRAINT_SCHEMA,
    CONTROL_BOX_ORDER,
    STAGE_NONLINEAR_H_ORDER,
    ConstraintBounds,
    evaluate_constraints,
)
from acados.mainline.cost_oracle import (
    CostOracleError,
    evaluate_stage_cost,
    evaluate_terminal_cost,
)
from acados.mainline.cost_schedule import (
    ExperimentCondition,
    build_liquid_cost_schedule,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.discrete_dynamics import evaluate_discrete_map
from acados.mainline.identity import sha256_json
from acados.mainline.model_contract import (
    COST_SCHEMA,
    DISCRETIZATION_SCHEMA,
    MODEL_ID,
)
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
from acados.mainline.runtime_schedule import (
    build_runtime_fractional_delay_schedule,
)
from acados.mainline.solver_parameter_layout import (
    REFERENCE_SCHEMA,
    build_solver_parameter_layout,
)

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)
PARITY_TOLERANCE = 5.0e-13


def _casadi_available() -> bool:
    try:
        import casadi  # noqa: F401
    except (ImportError, OSError):
        return False
    return True


def _floats(value) -> list[float]:
    return [float(item) for item in value.full().ravel()]


class MainlineCasadiAdapterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.development_layout = build_development_layout(self.capacity)
        self.parameter_layout = build_solver_parameter_layout(
            self.capacity,
            self.development_layout,
        )
        runtime = build_runtime_fractional_delay_schedule(
            self.capacity,
            0.05,
            0.07,
            1.0e-12,
            1.0e-12,
        )
        values = CommonStageParameterValues(
            actuator=ActuatorResponseParameters(0.2, 1.1, 0.4, 0.9),
            reference=ReferenceHorizonParameters(
                ReferencePolynomialParameters(
                    0.0,
                    2.0,
                    (0.0, 2.0, 0.1, 0.01),
                    (0.0, 0.2, 0.05, 0.01),
                    1.0e-3,
                ),
                tuple(0.3 + 0.001 * index for index in range(61)),
            ),
            slosh=SloshParameters(1.3, 0.2, 1.1, 0.9, 0.1, 0.3),
            normalization=NormalizationParameters(*([1.0] * 9)),
            running_weight=RunningWeightParameters(*([1.0] * 9)),
            terminal_weight=TerminalWeightParameters(*([1.0] * 4)),
        )
        liquid = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            8,
            2.0,
            1.0,
        )
        b0_liquid = build_liquid_cost_schedule(
            ExperimentCondition.B0,
            8,
            2.0,
            1.0,
        )
        self.snapshot = assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            runtime,
            values,
            liquid,
        )
        self.b0_snapshot = assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            runtime,
            values,
            b0_liquid,
        )
        zero_runtime = build_runtime_fractional_delay_schedule(
            self.capacity,
            0.0,
            0.0,
            1.0e-12,
            1.0e-12,
        )
        self.zero_snapshot = assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            zero_runtime,
            values,
            liquid,
        )
        self.bounds = ConstraintBounds(
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
        )
        self.state = tuple(
            (index % 7 - 3) * 0.01 for index in range(self.development_layout.NX)
        )
        self.control = (0.2, -0.1, 0.3)

    def test_import_surface_has_no_eager_backend_or_legacy_dependency(self) -> None:
        for source in sorted(MAINLINE_ROOT.glob("casadi_*.py")):
            tree = ast.parse(source.read_text(), filename=str(source))
            top_level_modules = []
            for node in tree.body:
                if isinstance(node, ast.Import):
                    top_level_modules.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    top_level_modules.append(node.module)
            with self.subTest(source=source.name):
                self.assertFalse(
                    any(
                        name.startswith(("casadi.", "acados_template."))
                        or name in {"casadi", "acados_template"}
                        or "legacy" in name
                        for name in top_level_modules
                    )
                )

        code = (
            "import sys; "
            f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r}); "
            "import acados.mainline.casadi_adapter; "
            "print('casadi' in sys.modules, 'acados_template' in sys.modules)"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "False False")

    def test_typed_authorities_fail_before_dependency_loading(self) -> None:
        with patch(
            "acados.mainline.casadi_adapter._require_casadi"
        ) as dependency_loader:
            with self.assertRaises(CasadiGraphConstructionError):
                build_casadi_graph(
                    object(),  # type: ignore[arg-type]
                    self.development_layout,
                    self.bounds,
                )
            with self.assertRaises(CasadiGraphConstructionError):
                build_casadi_graph(
                    self.capacity,
                    self.development_layout,
                    object(),  # type: ignore[arg-type]
                )
        dependency_loader.assert_not_called()

    def test_missing_dependency_fails_without_file_side_effect(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work_directory = Path(directory)
            previous_directory = Path.cwd()
            sentinel = sys.modules.pop("casadi", None)
            try:
                os.chdir(work_directory)
                before = sorted(work_directory.rglob("*"))
                raises_dependency = self.assertRaises(CasadiDependencyError)
                with patch.dict(sys.modules, {"casadi": None}), raises_dependency:
                    build_casadi_graph(
                        self.capacity,
                        self.development_layout,
                        self.bounds,
                    )
                after = sorted(work_directory.rglob("*"))
            finally:
                os.chdir(previous_directory)
                if sentinel is not None:
                    sys.modules["casadi"] = sentinel
            self.assertEqual(before, after)

    @unittest.skipUnless(_casadi_available(), "CasADi is not installed")
    def test_successful_graph_build_creates_no_artifact_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work_directory = Path(directory)
            previous_directory = Path.cwd()
            try:
                os.chdir(work_directory)
                before = sorted(work_directory.rglob("*"))
                graph = build_casadi_graph(
                    self.capacity,
                    self.development_layout,
                    self.bounds,
                )
                after = sorted(work_directory.rglob("*"))
            finally:
                os.chdir(previous_directory)
            self.assertEqual(graph.graph_status, "GRAPH_BUILT")
            self.assertEqual(graph.artifact_status, "NO_ARTIFACT")
            self.assertEqual(before, after)

    @unittest.skipUnless(_casadi_available(), "CasADi is not installed")
    def test_graph_metadata_terminal_inputs_and_numeric_parity(self) -> None:
        import casadi as ca

        graph = build_casadi_graph(
            self.capacity,
            self.development_layout,
            self.bounds,
        )
        self.assertEqual((graph.nx, graph.nu, graph.np), (48, 3, 162))
        self.assertEqual(graph.horizon_steps, 60)
        self.assertEqual(graph.graph_status, "GRAPH_BUILT")
        self.assertEqual(graph.artifact_status, "NO_ARTIFACT")
        self.assertEqual(graph.model_id, MODEL_ID)
        document = graph.to_dict()
        json.dumps(document, allow_nan=False)
        self.assertEqual(
            document["dimensions"],
            {"N": 60, "NX": 48, "NU": 3, "NP": 162},
        )
        self.assertEqual(
            document["status"],
            {"graph": "GRAPH_BUILT", "artifact": "NO_ARTIFACT"},
        )
        self.assertEqual(document["model_id"], MODEL_ID)
        self.assertEqual(
            document["schemas"],
            {
                "discretization": DISCRETIZATION_SCHEMA,
                "reference": REFERENCE_SCHEMA,
                "cost": COST_SCHEMA,
                "constraints": CONSTRAINT_SCHEMA,
            },
        )
        self.assertEqual(
            document["horizon"],
            {
                "N": 60,
                "parameter_vector_count": 61,
                "release_frequency_hz": 30,
                "release_period_sec": {"numerator": 1, "denominator": 30},
            },
        )
        self.assertEqual(document["stage_semantics"], "pre_issue_at_T_k_minus")
        self.assertEqual(
            document["runtime_parameter_input_policy"],
            "CALLER_VALIDATES_CANONICAL_RUNTIME_SNAPSHOT_BEFORE_GRAPH_INJECTION",
        )
        self.assertEqual(len(graph.graph_semantic_sha256), 64)
        self.assertEqual(
            document["graph_semantic_identity"]["sha256"],
            graph.graph_semantic_sha256,
        )
        self.assertEqual(
            graph_semantic_sha256(
                capacity_contract_sha256=graph.capacity_contract_sha256,
                development_layout_sha256=graph.development_layout_sha256,
                solver_parameter_layout_sha256=(graph.solver_parameter_layout_sha256),
                horizon_steps=graph.horizon_steps,
                parameter_vector_count=graph.parameter_vector_count,
                nx=graph.nx,
                nu=graph.nu,
                np=graph.np,
                state_order=graph.state_order,
                control_order=graph.control_order,
                parameter_order=graph.parameter_order,
                control_indices=graph.stage_constraints.control_indices,
            ),
            graph.graph_semantic_sha256,
        )
        self.assertEqual(
            tuple(document["orders"]["state"]),
            self.development_layout.state_names,
        )
        self.assertEqual(
            tuple(document["orders"]["control"]),
            self.development_layout.control_names,
        )
        self.assertEqual(
            tuple(document["orders"]["parameter"]),
            self.parameter_layout.parameter_names,
        )
        self.assertEqual(
            document["source_identity"],
            {
                "capacity_contract_raw_bytes_sha256": self.capacity.contract_sha256,
                "development_layout_semantic_sha256": sha256_json(
                    self.development_layout.to_dict()
                ),
                "solver_parameter_layout_semantic_sha256": sha256_json(
                    self.parameter_layout.to_dict()
                ),
            },
        )
        self.assertEqual(
            document["constraints"]["bounds_snapshot_sha256"],
            sha256_json(self.bounds.to_dict()),
        )
        self.assertEqual(
            document["constraints"]["diagnostic_residuals"]["role"],
            DIAGNOSTIC_RESIDUAL_ROLE,
        )
        self.assertEqual(
            document["terminal"],
            {
                "input_order": ["x_N", "p_N"],
                "control_policy": "NO_U_N_ACCESS",
                "liquid_cost_policy": "IDENTICALLY_ZERO",
            },
        )
        self.assertEqual(len(graph.execution_slots), 3)

        alternate_bounds = ConstraintBounds(
            9.0,
            8.0,
            7.0,
            6.0,
            5.0,
            4.0,
            3.0,
        )
        alternate_graph = build_casadi_graph(
            self.capacity,
            self.development_layout,
            alternate_bounds,
        )
        self.assertEqual(
            alternate_graph.graph_semantic_sha256,
            graph.graph_semantic_sha256,
        )
        self.assertNotEqual(
            alternate_graph.to_dict()["constraints"]["bounds_snapshot_sha256"],
            document["constraints"]["bounds_snapshot_sha256"],
        )

        combined = ca.vertcat(
            graph.x_next,
            graph.stage_costs.total,
            graph.terminal.total_cost,
            graph.stage_constraints.nonlinear_h,
        )
        self.assertEqual(
            {str(symbol).split("_")[0] for symbol in ca.symvar(combined)},
            {"x", "u", "p"},
        )
        terminal_combined = ca.vertcat(
            graph.terminal.total_cost,
            graph.terminal.reference_domain_h,
        )
        self.assertEqual(
            {str(symbol).split("_")[0] for symbol in ca.symvar(terminal_combined)},
            {"x", "p"},
        )

        stage_function = ca.Function(
            "mainline_graph_parity",
            [graph.x, graph.u, graph.p],
            [
                graph.x_next,
                ca.vertcat(
                    graph.issued.q_issue_v,
                    graph.issued.q_issue_omega,
                    graph.issued.a_issue,
                    graph.issued.alpha_issue,
                ),
                graph.reference.x_ref,
                graph.reference.y_ref,
                graph.reference.dx_ref_ds,
                graph.reference.dy_ref_ds,
                graph.reference.psi_ref,
                graph.reference.e_contour,
                graph.reference.e_lag,
                graph.stage_costs.robot_running,
                graph.stage_costs.liquid_running,
                graph.stage_costs.liquid_boundary,
                graph.stage_costs.total,
                graph.stage_constraints.nonlinear_h,
                graph.stage_constraints.diagnostic_residual,
            ],
        )
        parameter_row = self.snapshot.stage_parameters[0]
        numeric_map = evaluate_discrete_map(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            parameter_row,
        )
        numeric_stage = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            parameter_row,
        )
        numeric_constraints = evaluate_constraints(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            parameter_row,
            self.bounds,
        )
        actual = stage_function(self.state, self.control, parameter_row)
        expected = [
            list(numeric_map.next_state),
            [
                numeric_map.issued.q_issue_v,
                numeric_map.issued.q_issue_omega,
                numeric_map.issued.a_issue,
                numeric_map.issued.alpha_issue,
            ],
            [numeric_stage.reference.x_ref],
            [numeric_stage.reference.y_ref],
            [numeric_stage.reference.dx_ref_ds],
            [numeric_stage.reference.dy_ref_ds],
            [numeric_stage.reference.psi_ref],
            [numeric_stage.reference.e_contour],
            [numeric_stage.reference.e_lag],
            [numeric_stage.robot_running_cost],
            [numeric_stage.liquid_running_cost],
            [numeric_stage.liquid_boundary_cost],
            [numeric_stage.total_cost],
            [
                numeric_map.issued.q_issue_v,
                numeric_map.issued.q_issue_omega,
                numeric_map.issued.a_issue,
                numeric_map.issued.alpha_issue,
            ],
            [value for _, value in numeric_constraints.residuals],
        ]
        for actual_values, expected_values in zip(actual, expected):
            self.assertEqual(len(_floats(actual_values)), len(expected_values))
            for left, right in zip(_floats(actual_values), expected_values):
                self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)

        boundary_row = self.snapshot.stage_parameters[8]
        numeric_boundary_stage = evaluate_stage_cost(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            self.state,
            self.control,
            boundary_row,
        )
        boundary_actual = stage_function(
            self.state,
            self.control,
            boundary_row,
        )
        self.assertGreater(numeric_boundary_stage.liquid_boundary_cost, 0.0)
        for output_index, expected_value in (
            (9, numeric_boundary_stage.robot_running_cost),
            (10, numeric_boundary_stage.liquid_running_cost),
            (11, numeric_boundary_stage.liquid_boundary_cost),
            (12, numeric_boundary_stage.total_cost),
        ):
            self.assertAlmostEqual(
                float(boundary_actual[output_index]),
                expected_value,
                delta=PARITY_TOLERANCE,
            )

        terminal_row = self.snapshot.stage_parameters[-1]
        numeric_terminal = evaluate_terminal_cost(
            self.state,
            terminal_row,
            self.development_layout,
            self.parameter_layout,
        )
        terminal_function = ca.Function(
            "mainline_terminal_parity",
            [graph.x, graph.p],
            [graph.terminal.total_cost],
        )
        self.assertAlmostEqual(
            float(terminal_function(self.state, terminal_row)),
            numeric_terminal.total_cost,
            delta=PARITY_TOLERANCE,
        )

        segment_function = ca.Function(
            "mainline_segment_parity",
            [graph.x, graph.u, graph.p],
            [
                ca.vertcat(*(slot.duration_sec for slot in graph.execution_slots)),
                ca.vertcat(*(slot.q_target_v for slot in graph.execution_slots)),
                ca.vertcat(*(slot.q_target_omega for slot in graph.execution_slots)),
            ],
        )
        segment_values = [
            _floats(output)
            for output in segment_function(
                self.state,
                self.control,
                parameter_row,
            )
        ]
        for actual_values, expected_values in zip(
            segment_values,
            (
                [segment.duration_sec for segment in numeric_map.segments],
                [segment.q_target_v for segment in numeric_map.segments],
                [segment.q_target_omega for segment in numeric_map.segments],
            ),
        ):
            for left, right in zip(actual_values, expected_values):
                self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)

    @unittest.skipUnless(_casadi_available(), "CasADi is not installed")
    def test_zero_delay_tail_is_identity_and_experiment_arms_share_graph(self) -> None:
        import casadi as ca

        graph = build_casadi_graph(
            self.capacity,
            self.development_layout,
            self.bounds,
        )
        zero_function = ca.Function(
            "mainline_zero_delay_map",
            [graph.x, graph.u, graph.p],
            [
                graph.x_next,
                ca.vertcat(*(slot.duration_sec for slot in graph.execution_slots)),
            ],
        )
        zero_row = self.zero_snapshot.stage_parameters[0]
        numeric_zero = evaluate_discrete_map(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            zero_row,
        )
        symbolic_zero = zero_function(self.state, self.control, zero_row)
        self.assertEqual(_floats(symbolic_zero[1])[1:], [0.0, 0.0])
        for left, right in zip(_floats(symbolic_zero[0]), numeric_zero.next_state):
            self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)

        comparison = ca.Function(
            "mainline_arm_comparison",
            [graph.x, graph.u, graph.p],
            [
                graph.x_next,
                graph.reference.x_ref,
                graph.reference.e_contour,
                graph.stage_costs.robot_running,
                graph.stage_costs.liquid_running,
                graph.stage_costs.liquid_boundary,
                graph.stage_constraints.nonlinear_h,
                graph.stage_constraints.diagnostic_residual,
            ],
        )
        b0 = comparison(
            self.state,
            self.control,
            self.b0_snapshot.stage_parameters[0],
        )
        bslosh = comparison(
            self.state,
            self.control,
            self.snapshot.stage_parameters[0],
        )
        b0_values = [_floats(output) for output in b0]
        bslosh_values = [_floats(output) for output in bslosh]
        for index in (0, 1, 2, 3, 6, 7):
            self.assertEqual(len(b0_values[index]), len(bslosh_values[index]))
            for left, right in zip(b0_values[index], bslosh_values[index]):
                self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)
        self.assertEqual(b0_values[4], [0.0])
        self.assertEqual(b0_values[5], [0.0])

    @unittest.skipUnless(_casadi_available(), "CasADi is not installed")
    def test_constraints_and_stage_terminal_reference_domains(self) -> None:
        import casadi as ca

        graph = build_casadi_graph(
            self.capacity,
            self.development_layout,
            self.bounds,
        )
        constraints = graph.stage_constraints
        values = ca.Function(
            "mainline_constraint_contract",
            [graph.x, graph.u, graph.p],
            [
                constraints.nonlinear_h,
                constraints.diagnostic_residual,
                constraints.reference_domain_h,
            ],
        )(
            self.state,
            self.control,
            self.snapshot.stage_parameters[0],
        )
        numeric = evaluate_constraints(
            self.capacity,
            self.development_layout,
            self.state,
            self.control,
            self.snapshot.stage_parameters[0],
            self.bounds,
        )
        expected_h = [
            numeric.residuals[1][1] + self.bounds.q_issue_v_max,
            numeric.residuals[3][1] + self.bounds.q_issue_omega_max,
            numeric.residuals[5][1] + self.bounds.a_issue_max,
            numeric.residuals[7][1] + self.bounds.alpha_issue_max,
        ]
        for left, right in zip(_floats(values[0]), expected_h):
            self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)
        for left, right in zip(
            _floats(values[1]),
            [value for _, value in numeric.residuals],
        ):
            self.assertAlmostEqual(left, right, delta=PARITY_TOLERANCE)
        self.assertEqual(constraints.nonlinear_h_order, STAGE_NONLINEAR_H_ORDER)
        self.assertEqual(
            constraints.diagnostic_residual_order, CONSTRAINT_RESIDUAL_ORDER
        )
        self.assertEqual(constraints.control_order, CONTROL_BOX_ORDER)
        self.assertEqual(constraints.control_indices, (0, 1, 2))
        self.assertEqual(
            constraints.nonlinear_lower,
            (-10.0, -10.0, -10.0, -10.0),
        )
        self.assertEqual(
            constraints.nonlinear_upper,
            (10.0, 10.0, 10.0, 10.0),
        )
        self.assertEqual(constraints.control_lower, (-10.0, -10.0, 0.0))
        self.assertEqual(constraints.control_upper, (10.0, 10.0, 10.0))

        xi_stage = ca.Function(
            "mainline_stage_xi",
            [graph.x, graph.p],
            [constraints.reference_domain_h],
        )
        at_end = list(self.state)
        at_end[self.development_layout.state_offsets["s"]] = 2.0
        row = self.snapshot.stage_parameters[0]
        self.assertEqual(float(values[2]), 0.0)
        self.assertEqual(float(xi_stage(tuple(at_end), row)), 1.0)

        outside = list(self.state)
        outside[self.development_layout.state_offsets["s"]] = 3.0
        with self.assertRaises(CostOracleError):
            evaluate_stage_cost(
                self.capacity,
                self.development_layout,
                self.parameter_layout,
                tuple(outside),
                self.control,
                row,
            )
        self.assertGreater(float(xi_stage(tuple(outside), row)), 1.0)

        terminal_row = list(self.snapshot.stage_parameters[-1])
        terminal_row[self.parameter_layout.parameter_offsets["ref_s_origin"]] = 0.25
        terminal_row[self.parameter_layout.parameter_offsets["ref_s_scale"]] = 0.5
        terminal_state = list(self.state)
        terminal_state[self.development_layout.state_offsets["s"]] = 0.5
        terminal_xi = ca.Function(
            "mainline_terminal_xi",
            [graph.x, graph.p],
            [graph.terminal.reference_domain_h],
        )
        self.assertAlmostEqual(
            float(xi_stage(tuple(terminal_state), row)),
            0.25,
            delta=PARITY_TOLERANCE,
        )
        self.assertAlmostEqual(
            float(terminal_xi(tuple(terminal_state), tuple(terminal_row))),
            0.5,
            delta=PARITY_TOLERANCE,
        )

    @unittest.skipUnless(_casadi_available(), "CasADi is not installed")
    def test_complete_graph_jacobian_shape_and_finiteness(self) -> None:
        import casadi as ca

        graph = build_casadi_graph(
            self.capacity,
            self.development_layout,
            self.bounds,
        )
        outputs = ca.vertcat(
            graph.x_next,
            graph.stage_costs.total,
            graph.terminal.total_cost,
            graph.stage_constraints.nonlinear_h,
            graph.stage_constraints.diagnostic_residual,
            graph.stage_constraints.reference_domain_h,
            graph.terminal.reference_domain_h,
        )
        inputs = ca.vertcat(graph.x, graph.u, graph.p)
        jacobian = ca.jacobian(outputs, inputs)
        self.assertEqual(jacobian.shape, (70, 213))
        function = ca.Function(
            "mainline_complete_jacobian",
            [graph.x, graph.u, graph.p],
            [jacobian],
        )
        values = _floats(
            function(
                self.state,
                self.control,
                self.snapshot.stage_parameters[0],
            )
        )
        self.assertEqual(len(values), 70 * 213)
        self.assertTrue(all(math.isfinite(value) for value in values))


if __name__ == "__main__":
    unittest.main()
