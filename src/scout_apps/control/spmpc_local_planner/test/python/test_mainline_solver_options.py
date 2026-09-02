"""Contract tests for the dependency-free Stage 3-D solver options."""

from __future__ import annotations

import ast
import dataclasses
import json
import sys
import unittest
from fractions import Fraction
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "solver_options.py"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import sha256_json
from acados.mainline.solver_options import (
    COST_SCALING_POLICY,
    SOLVER_OPTIONS_SCOPE,
    SolverOptionsError,
    SolverOptionsSnapshot,
    build_solver_options_snapshot,
)

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


class MainlineSolverOptionsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.layout = build_development_layout(self.capacity)
        self.snapshot = build_solver_options_snapshot(self.layout)

    def test_unique_discrete_development_options_are_explicit(self) -> None:
        snapshot = self.snapshot
        self.assertEqual(snapshot.horizon_steps, 60)
        self.assertEqual(snapshot.time_step_sec, Fraction(1, 30))
        self.assertEqual(snapshot.time_horizon_sec, Fraction(2, 1))
        self.assertEqual(snapshot.time_steps, (Fraction(1, 30),) * 60)
        self.assertEqual(snapshot.cost_scaling, (1.0,) * 61)
        self.assertEqual(snapshot.integrator_type, "DISCRETE")
        self.assertEqual(snapshot.cost_discretization, "EULER")
        self.assertEqual(snapshot.nlp_solver_type, "SQP_RTI")
        self.assertEqual(snapshot.nlp_solver_max_iter, 1)
        self.assertEqual(snapshot.hessian_approx, "EXACT")
        self.assertTrue(snapshot.exact_hess_dyn)
        self.assertTrue(snapshot.exact_hess_cost)
        self.assertTrue(snapshot.exact_hess_constr)
        self.assertEqual(snapshot.ext_cost_num_hess, 0)
        self.assertEqual(snapshot.regularize_method, "PROJECT")
        self.assertEqual(snapshot.levenberg_marquardt, 1.0e-3)
        self.assertEqual(snapshot.globalization, "FIXED_STEP")
        self.assertEqual(snapshot.qp_solver, "PARTIAL_CONDENSING_HPIPM")
        self.assertEqual(snapshot.qp_solver_cond_N, 60)
        self.assertEqual(snapshot.qp_solver_warm_start, 0)
        self.assertFalse(snapshot.nlp_solver_warm_start_first_qp)
        self.assertEqual(snapshot.hpipm_mode, "BALANCE")
        self.assertEqual(snapshot.status, "DEV_UNVALIDATED")
        self.assertEqual(snapshot.target_performance_status, "NOT_BENCHMARKED")
        self.assertEqual(snapshot.artifact_status, "NO_ARTIFACT")

    def test_snapshot_identity_and_serialization_are_canonical(self) -> None:
        document = self.snapshot.to_dict()
        json.dumps(document, allow_nan=False)
        identity = document.pop("semantic_identity")
        self.assertEqual(identity["scope"], SOLVER_OPTIONS_SCOPE)
        self.assertEqual(identity["sha256"], sha256_json(document))
        self.assertEqual(identity["sha256"], self.snapshot.semantic_sha256)
        self.assertEqual(
            document["horizon"]["cost_scaling"],
            {
                "policy": COST_SCALING_POLICY,
                "count": 61,
                "uniform_value": 1.0,
            },
        )
        self.assertEqual(
            document["horizon"]["time_steps"],
            {
                "policy": "UNIFORM_RELEASE_PERIOD",
                "count": 60,
                "uniform_value_sec": {"numerator": 1, "denominator": 30},
            },
        )
        self.assertEqual(
            build_solver_options_snapshot(self.layout).to_dict(),
            self.snapshot.to_dict(),
        )

    def test_snapshot_requires_the_canonical_builder_and_is_frozen(self) -> None:
        fields = {
            field.name: getattr(self.snapshot, field.name)
            for field in dataclasses.fields(self.snapshot)
            if field.init
        }
        with self.assertRaises(SolverOptionsError):
            SolverOptionsSnapshot(**fields)
        with self.assertRaises(SolverOptionsError):
            dataclasses.replace(self.snapshot, qp_solver_iter_max=49)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            self.snapshot.print_level = 1  # type: ignore[misc]
        with self.assertRaises(SolverOptionsError):
            build_solver_options_snapshot(object())  # type: ignore[arg-type]

    def test_module_has_no_backend_legacy_or_gate_dependency(self) -> None:
        tree = ast.parse(MODULE.read_text(encoding="utf-8"), filename=str(MODULE))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
        self.assertFalse(
            any(
                name.startswith(("casadi", "acados_template"))
                or "legacy" in name
                or name.endswith(("stage1_evidence", "manifest"))
                for name in imports
            )
        )


if __name__ == "__main__":
    unittest.main()
