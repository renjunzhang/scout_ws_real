"""Integration contract tests for artifact-free Stage 3-D Acados OCP assembly."""

from __future__ import annotations

import ast
import importlib.util
import json
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

from acados.mainline.acados_ocp_adapter import (
    AcadosOcpConstructionError,
    AcadosOcpDependencyError,
    assemble_acados_ocp,
)
from acados.mainline.acados_ocp_contract import (
    DIAGNOSTIC_RESIDUAL_POLICY,
    DYNAMICS_IDENTITY_FUNCTION,
    INITIAL_STATE_POLICY,
    OCP_IDENTITY_SCOPE,
    PARAMETER_INITIALIZATION_POLICY,
)
from acados.mainline.casadi_adapter import build_casadi_graph
from acados.mainline.constraints_oracle import ConstraintBounds
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import sha256_bytes, sha256_json
from acados.mainline.model_contract import MODEL_ID
from acados.mainline.solver_options import build_solver_options_snapshot

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)
CASADI_AVAILABLE = importlib.util.find_spec("casadi") is not None
ACADOS_AVAILABLE = importlib.util.find_spec("acados_template") is not None
BACKEND_REQUIRED = os.environ.get("SPMPC_REQUIRE_ACADOS_BACKEND") == "1"


def _empty_expression(value) -> bool:
    if value is None:
        return True
    if isinstance(value, (list, tuple)):
        return len(value) == 0
    return bool(value.is_empty())


class MainlineAcadosOcpLazyBoundaryTest(unittest.TestCase):
    def setUp(self) -> None:
        capacity = load_development_capacity(CAPACITY)
        self.layout = build_development_layout(capacity)
        self.options = build_solver_options_snapshot(self.layout)

    def test_import_is_lazy_and_does_not_load_numeric_backends(self) -> None:
        code = (
            "import sys; "
            f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r}); "
            "import acados.mainline.acados_ocp_adapter; "
            "print('casadi' in sys.modules, 'numpy' in sys.modules, "
            "'acados_template' in sys.modules)"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "False False False")

    def test_backend_requirement_flag_cannot_silently_skip_integration(self) -> None:
        if BACKEND_REQUIRED:
            self.assertTrue(CASADI_AVAILABLE)
            self.assertTrue(ACADOS_AVAILABLE)

    def test_invalid_typed_input_fails_before_backend_loading(self) -> None:
        with (
            patch(
                "acados.mainline.acados_ocp_adapter._require_acados_backend"
            ) as loader,
            self.assertRaises(AcadosOcpConstructionError),
        ):
            assemble_acados_ocp(
                object(),  # type: ignore[arg-type]
                self.options,
            )
        loader.assert_not_called()

    def test_adapter_never_imports_solver_codegen_or_legacy_modules(self) -> None:
        for source_name in ("acados_ocp_adapter.py", "acados_ocp_contract.py"):
            source = MAINLINE_ROOT / source_name
            tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
            imported = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imported.append(node.module)
            with self.subTest(source=source_name):
                self.assertFalse(
                    any(
                        "AcadosOcpSolver" in name
                        or "legacy" in name
                        or name.endswith(("stage1_evidence", "manifest"))
                        for name in imported
                    )
                )


@unittest.skipUnless(CASADI_AVAILABLE, "CasADi is not installed")
class MainlineAcadosOcpDependencyFailureTest(unittest.TestCase):
    def test_missing_acados_template_fails_without_file_side_effect(self) -> None:
        capacity = load_development_capacity(CAPACITY)
        layout = build_development_layout(capacity)
        graph = build_casadi_graph(
            capacity,
            layout,
            ConstraintBounds(*([10.0] * 7)),
        )
        options = build_solver_options_snapshot(layout)
        with tempfile.TemporaryDirectory() as directory:
            work_directory = Path(directory)
            previous_directory = Path.cwd()
            sentinel = sys.modules.pop("acados_template", None)
            try:
                os.chdir(work_directory)
                before = sorted(work_directory.rglob("*"))
                raises_dependency = self.assertRaises(AcadosOcpDependencyError)
                with (
                    patch.dict(sys.modules, {"acados_template": None}),
                    raises_dependency,
                ):
                    assemble_acados_ocp(graph, options)
                after = sorted(work_directory.rglob("*"))
            finally:
                os.chdir(previous_directory)
                if sentinel is not None:
                    sys.modules["acados_template"] = sentinel
            self.assertEqual(before, after)


@unittest.skipUnless(
    CASADI_AVAILABLE and ACADOS_AVAILABLE,
    "CasADi and acados_template are not installed",
)
class MainlineAcadosOcpBackendTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.layout = build_development_layout(self.capacity)
        self.bounds = ConstraintBounds(*([10.0] * 7))
        self.graph = build_casadi_graph(
            self.capacity,
            self.layout,
            self.bounds,
        )
        self.options = build_solver_options_snapshot(self.layout)

    def _assemble(self):
        return assemble_acados_ocp(self.graph, self.options)

    def test_successful_make_consistent_assembly_creates_no_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work_directory = Path(directory)
            previous_directory = Path.cwd()
            try:
                os.chdir(work_directory)
                before = sorted(work_directory.rglob("*"))
                assembly = self._assemble()
                after = sorted(work_directory.rglob("*"))
                latent_export_directory = Path(
                    assembly.ocp.code_gen_opts.code_export_directory
                )
            finally:
                os.chdir(previous_directory)
            self.assertEqual(before, after)
            self.assertFalse(latent_export_directory.exists())
            self.assertEqual(assembly.ocp_status, "OCP_ASSEMBLED_AND_CONSISTENT")
            self.assertEqual(assembly.artifact_status, "NO_ARTIFACT")
            self.assertEqual(assembly.promotion_status, "DEV_UNVALIDATED")

    def test_dimensions_node_coverage_and_discrete_model_are_explicit(self) -> None:
        import casadi as ca
        import numpy as np

        assembly = self._assemble()
        ocp = assembly.ocp
        expected_dimensions = {
            "nx": 48,
            "nu": 3,
            "np": 162,
            "nx_next": 48,
            "nh_0": 5,
            "nh": 5,
            "nh_e": 1,
            "nbu": 3,
            "nbx_0": 48,
        }
        for name, expected in expected_dimensions.items():
            with self.subTest(dimension=name):
                self.assertEqual(getattr(ocp.dims, name), expected)
        self.assertEqual(ocp.solver_options.N_horizon, 60)

        self.assertEqual(ocp.model.name, MODEL_ID)
        self.assertTrue(ca.is_equal(ocp.model.disc_dyn_expr, self.graph.x_next, 2))
        self.assertTrue(_empty_expression(ocp.model.f_expl_expr))
        self.assertTrue(_empty_expression(ocp.model.f_impl_expr))
        self.assertEqual(
            (ocp.cost.cost_type_0, ocp.cost.cost_type, ocp.cost.cost_type_e),
            ("EXTERNAL", "EXTERNAL", "EXTERNAL"),
        )
        self.assertTrue(
            ca.is_equal(
                ocp.model.cost_expr_ext_cost_0,
                self.graph.stage_costs.total,
                2,
            )
        )
        self.assertTrue(
            ca.is_equal(
                ocp.model.cost_expr_ext_cost,
                self.graph.stage_costs.total,
                2,
            )
        )
        self.assertTrue(
            ca.is_equal(
                ocp.model.cost_expr_ext_cost_e,
                self.graph.terminal.total_cost,
                2,
            )
        )
        self.assertEqual(
            {
                str(symbol).split("_")[0]
                for symbol in ca.symvar(
                    ca.vertcat(
                        ocp.model.cost_expr_ext_cost_e,
                        ocp.model.con_h_expr_e,
                    )
                )
            },
            {"x", "p"},
        )
        self.assertTrue(ocp.constraints.has_x0)
        np.testing.assert_array_equal(ocp.constraints.idxbx_0, np.arange(48))
        np.testing.assert_array_equal(ocp.constraints.idxbxe_0, np.arange(48))
        np.testing.assert_array_equal(ocp.constraints.lbx_0, np.zeros(48))
        np.testing.assert_array_equal(ocp.constraints.ubx_0, np.zeros(48))
        np.testing.assert_array_equal(ocp.parameter_values, np.zeros(162))

    def test_stage_zero_path_terminal_constraints_and_scaling_match_contract(
        self,
    ) -> None:
        import casadi as ca
        import numpy as np

        assembly = self._assemble()
        ocp = assembly.ocp
        expected_stage_h = ca.vertcat(
            self.graph.stage_constraints.nonlinear_h,
            self.graph.stage_constraints.reference_domain_h,
        )
        self.assertTrue(ca.is_equal(ocp.model.con_h_expr_0, expected_stage_h, 2))
        self.assertTrue(ca.is_equal(ocp.model.con_h_expr, expected_stage_h, 2))
        self.assertTrue(
            ca.is_equal(
                ocp.model.con_h_expr_e,
                self.graph.terminal.reference_domain_h,
                2,
            )
        )
        self.assertEqual(
            assembly.stage_constraint_order,
            (
                "q_issue_v",
                "q_issue_omega",
                "a_issue",
                "alpha_issue",
                "xi_k",
            ),
        )
        np.testing.assert_array_equal(
            ocp.constraints.lh_0,
            np.asarray((-10.0, -10.0, -10.0, -10.0, 0.0)),
        )
        np.testing.assert_array_equal(ocp.constraints.lh, ocp.constraints.lh_0)
        np.testing.assert_array_equal(
            ocp.constraints.uh_0,
            np.asarray((10.0, 10.0, 10.0, 10.0, 1.0)),
        )
        np.testing.assert_array_equal(ocp.constraints.uh, ocp.constraints.uh_0)
        np.testing.assert_array_equal(ocp.constraints.lh_e, np.asarray((0.0,)))
        np.testing.assert_array_equal(ocp.constraints.uh_e, np.asarray((1.0,)))
        np.testing.assert_array_equal(ocp.constraints.idxbu, np.asarray((0, 1, 2)))
        np.testing.assert_array_equal(
            ocp.constraints.lbu,
            np.asarray((-10.0, -10.0, 0.0)),
        )
        np.testing.assert_array_equal(
            ocp.constraints.ubu,
            np.asarray((10.0, 10.0, 10.0)),
        )
        np.testing.assert_array_equal(ocp.solver_options.cost_scaling, np.ones(61))
        np.testing.assert_array_equal(
            ocp.solver_options.time_steps,
            np.full(60, 1.0 / 30.0),
        )

    def test_metadata_combines_graph_bounds_options_and_backend_identity(self) -> None:
        import casadi as ca

        assembly = self._assemble()
        document = assembly.to_dict()
        json.dumps(document, allow_nan=False)
        identity = document.pop("semantic_identity")
        self.assertEqual(identity["scope"], OCP_IDENTITY_SCOPE)
        self.assertEqual(identity["sha256"], sha256_json(document))
        self.assertEqual(identity["sha256"], assembly.semantic_sha256)
        self.assertEqual(document["model_id"], MODEL_ID)
        self.assertEqual(
            document["dimensions"],
            {"N": 60, "NX": 48, "NU": 3, "NP": 162, "parameter_vector_count": 61},
        )
        self.assertEqual(
            document["source_identity"]["graph_semantic_sha256"],
            self.graph.graph_semantic_sha256,
        )
        self.assertEqual(
            document["source_identity"]["bounds_snapshot_sha256"],
            sha256_json(self.bounds.to_dict()),
        )
        self.assertEqual(
            document["source_identity"]["solver_options_semantic_sha256"],
            self.options.semantic_sha256,
        )
        self.assertNotEqual(document["backend"]["acados_git_commit"], "unknown")
        self.assertEqual(
            document["backend"]["python_interface_and_library_binding"],
            "MATCHED_SOURCE_ROOT",
        )
        self.assertEqual(
            len(document["backend"]["interface_source_identity"]["sha256"]),
            64,
        )
        expected_dynamics_sha256 = sha256_bytes(
            ca.Function(
                DYNAMICS_IDENTITY_FUNCTION,
                [self.graph.x, self.graph.u, self.graph.p],
                [self.graph.x_next],
            )
            .serialize()
            .encode("utf-8")
        )
        self.assertEqual(
            document["symbolic_expression_identity"]["dynamics"]["sha256"],
            expected_dynamics_sha256,
        )
        for value in document["symbolic_expression_identity"].values():
            if isinstance(value, dict):
                self.assertEqual(len(value["sha256"]), 64)
        self.assertEqual(document["constraints"]["initial_state"], INITIAL_STATE_POLICY)
        self.assertEqual(
            document["constraints"]["diagnostic_residuals"],
            DIAGNOSTIC_RESIDUAL_POLICY,
        )
        self.assertEqual(
            document["runtime_parameters"]["initialization"],
            PARAMETER_INITIALIZATION_POLICY,
        )
        self.assertEqual(
            document["runtime_parameters"]["required_stage_range_inclusive"],
            [0, 60],
        )
        self.assertEqual(
            document["comparison_identity"],
            {
                "arms": ["B0", "Bslosh"],
                "same_ocp": True,
                "future_artifact_policy": "ONE_SHARED_ARTIFACT_REQUIRED",
                "runtime_only_difference": [
                    "liquid_run_coeff",
                    "liquid_boundary_coeff",
                ],
            },
        )

    def test_ocp_identity_includes_explicit_bound_snapshot(self) -> None:
        baseline = self._assemble()
        alternate_graph = build_casadi_graph(
            self.capacity,
            self.layout,
            ConstraintBounds(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0),
        )
        alternate = assemble_acados_ocp(alternate_graph, self.options)
        self.assertEqual(
            alternate.graph_semantic_sha256,
            baseline.graph_semantic_sha256,
        )
        self.assertEqual(
            alternate.solver_options_semantic_sha256,
            baseline.solver_options_semantic_sha256,
        )
        self.assertEqual(
            alternate.dynamics_expression_sha256,
            baseline.dynamics_expression_sha256,
        )
        self.assertEqual(
            alternate.stage_cost_expression_sha256,
            baseline.stage_cost_expression_sha256,
        )
        self.assertNotEqual(
            alternate.bounds_snapshot_sha256,
            baseline.bounds_snapshot_sha256,
        )
        self.assertNotEqual(alternate.semantic_sha256, baseline.semantic_sha256)

    def test_repeated_assembly_has_deterministic_composite_identity(self) -> None:
        first = self._assemble()
        second = self._assemble()
        self.assertEqual(first.semantic_sha256, second.semantic_sha256)
        self.assertEqual(first.to_dict(), second.to_dict())

    def test_wrong_option_type_is_rejected_before_backend_work(self) -> None:
        with (
            patch(
                "acados.mainline.acados_ocp_adapter._require_acados_backend"
            ) as loader,
            self.assertRaises(AcadosOcpConstructionError),
        ):
            assemble_acados_ocp(
                self.graph,
                object(),  # type: ignore[arg-type]
            )
        loader.assert_not_called()


if __name__ == "__main__":
    unittest.main()
