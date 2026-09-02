#!/usr/bin/env python3
"""Contract tests for the full Stage 3-D per-stage parameter order."""

from __future__ import annotations

import ast
import copy
import json
import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.solver_parameter_layout import (
    ARTIFACT_STATUS,
    GRAPH_STATUS,
    LIQUID_BOUNDARY_POLICY,
    LIQUID_BOUNDARY_RATIO_POLICY,
    LIQUID_BOUNDARY_STATE_POLICY,
    LIQUID_RUNNING_RATIO_POLICY,
    LIQUID_RUNNING_STATE_POLICY,
    REF_SPEED_ROLE,
    REF_SPEED_SCHEDULE_POLICY,
    REFERENCE_COORDINATE_FORMULA,
    REFERENCE_DOMAIN_POLICY,
    REFERENCE_HEADING_FORMULA,
    REFERENCE_POLYNOMIAL_BASIS,
    REFERENCE_POSITION_FORMULA,
    REFERENCE_SCALE_POLICY,
    REFERENCE_SCHEMA,
    REFERENCE_TANGENT_FORMULA,
    REFERENCE_TANGENT_POLICY,
    RUNNING_OMEGA_POLICY,
    SLOSH_RUNNING_RATIO_PARAMETER,
    SOLVER_PARAMETER_LAYOUT_SCOPE,
    TERMINAL_ASSIGNMENT_POLICY,
    TERMINAL_CONTROL_POLICY,
    TERMINAL_LIQUID_POLICY,
    TERMINAL_STAGE_POLICY,
    ParameterBlockRange,
    SolverParameterLayoutError,
    build_solver_parameter_layout,
    solver_parameter_layout_from_dict,
)

CAPACITY = (
    PACKAGE_ROOT
    / "config"
    / "mainline"
    / "contracts"
    / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "solver_parameter_layout.py"


class MainlineSolverParameterLayoutTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.development_layout = build_development_layout(self.capacity)
        self.layout = build_solver_parameter_layout(
            self.capacity,
            self.development_layout,
        )
        self.document = self.layout.to_dict()

    def changed(self, path: tuple[object, ...], value: object) -> dict:
        result = copy.deepcopy(self.document)
        target = result
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        return result

    def validate(self, value: dict) -> None:
        solver_parameter_layout_from_dict(
            value,
            self.capacity,
            self.development_layout,
        )

    def test_execution_prefix_is_preserved_and_suffix_order_is_exact(self) -> None:
        expected_suffix = (
            "ref_s_origin",
            "ref_s_scale",
            "ref_x_coeff[0]",
            "ref_x_coeff[1]",
            "ref_x_coeff[2]",
            "ref_x_coeff[3]",
            "ref_y_coeff[0]",
            "ref_y_coeff[1]",
            "ref_y_coeff[2]",
            "ref_y_coeff[3]",
            "ref_speed",
            "slosh_two_zeta_omega_n",
            "slosh_omega_n_sq",
            "slosh_kappa_x",
            "slosh_kappa_y",
            "slosh_eta_ref",
            "slosh_running_eta_dot_ratio",
            "norm_contour",
            "norm_lag",
            "norm_v_actual",
            "norm_omega_actual",
            "norm_v_s",
            "norm_a_issue",
            "norm_alpha_issue",
            "norm_jerk_v",
            "norm_jerk_omega",
            "weight_contour",
            "weight_lag",
            "weight_progress",
            "weight_v_actual",
            "weight_v_s",
            "weight_a_issue",
            "weight_alpha_issue",
            "weight_jerk_v",
            "weight_jerk_omega",
            "weight_terminal_contour",
            "weight_terminal_lag",
            "weight_terminal_v_actual",
            "weight_terminal_omega_actual",
            "liquid_run_coeff",
            "liquid_boundary_coeff",
        )
        self.assertEqual(
            self.layout.parameter_names[:121],
            self.development_layout.execution_parameter_names,
        )
        self.assertEqual(self.layout.parameter_names[121:], expected_suffix)
        self.assertEqual(len(expected_suffix), 41)
        self.assertEqual(self.layout.NP, 162)
        self.assertEqual(len(set(self.layout.parameter_names)), 162)
        self.assertEqual(
            [
                self.layout.parameter_offsets[name]
                for name in self.layout.parameter_names
            ],
            list(range(162)),
        )

    def test_block_ranges_have_exact_contiguous_boundaries(self) -> None:
        expected = {
            "execution_prefix": (0, 121),
            "reference": (121, 132),
            "slosh": (132, 138),
            "normalization": (138, 147),
            "running_weight": (147, 156),
            "terminal_weight": (156, 160),
            "stage_cost": (160, 162),
        }
        self.assertEqual(tuple(self.layout.block_ranges), tuple(expected))
        for name, (begin, end_exclusive) in expected.items():
            block_range = self.layout.block_ranges[name]
            self.assertEqual(
                block_range,
                ParameterBlockRange(begin, end_exclusive),
            )
            self.assertEqual(block_range.size, end_exclusive - begin)
            self.assertEqual(
                self.layout.parameter_names[begin:end_exclusive],
                tuple(
                    name
                    for name in self.layout.parameter_names
                    if begin
                    <= self.layout.parameter_offsets[name]
                    < end_exclusive
                ),
            )

    def test_stage_vector_and_terminal_assignment_semantics_are_explicit(self) -> None:
        self.assertEqual(self.document["scope"], SOLVER_PARAMETER_LAYOUT_SCOPE)
        self.assertEqual(
            self.document["capability_status"],
            {"graph": GRAPH_STATUS, "artifact": ARTIFACT_STATUS},
        )
        self.assertEqual(self.document["reference_schema"], REFERENCE_SCHEMA)
        stage_contract = self.document["stage_vector_contract"]
        self.assertEqual(stage_contract["shooting_intervals"], 60)
        self.assertEqual(stage_contract["parameter_vector_count"], 61)
        self.assertEqual(stage_contract["common_width"], 162)
        self.assertIs(stage_contract["common_order_for_all_stages"], True)
        self.assertEqual(
            stage_contract["terminal_assignment_policy"],
            TERMINAL_ASSIGNMENT_POLICY,
        )

    def test_source_identity_binds_capacity_bytes_and_complete_d1_layout(self) -> None:
        source = self.document["source_identity"]
        self.assertEqual(
            source["development_capacity_raw_bytes_sha256"],
            self.capacity.contract_sha256,
        )
        d1_identity = source["development_layout"]
        self.assertEqual(
            d1_identity["schema_version"],
            "spmpc_mainline_development_layout_v1",
        )
        self.assertEqual(
            d1_identity["scope"],
            "DEVELOPMENT_STATE_CONTROL_EXECUTION_PREFIX_ONLY",
        )
        self.assertEqual(len(d1_identity["semantic_sha256"]), 64)

    def test_reference_geometry_and_speed_semantics_are_explicit(self) -> None:
        reference = self.document["semantic_contracts"]["reference"]
        self.assertEqual(
            reference["coordinate_formula"],
            REFERENCE_COORDINATE_FORMULA,
        )
        self.assertEqual(
            reference["polynomial_basis"],
            list(REFERENCE_POLYNOMIAL_BASIS),
        )
        self.assertEqual(
            reference["position_formula"],
            REFERENCE_POSITION_FORMULA,
        )
        self.assertEqual(
            reference["tangent_formula"],
            REFERENCE_TANGENT_FORMULA,
        )
        self.assertEqual(
            reference["heading_formula"],
            REFERENCE_HEADING_FORMULA,
        )
        self.assertEqual(reference["effective_domain"], {"lower": 0, "upper": 1})
        self.assertEqual(reference["domain_policy"], REFERENCE_DOMAIN_POLICY)
        self.assertEqual(reference["scale_policy"], REFERENCE_SCALE_POLICY)
        self.assertEqual(reference["tangent_policy"], REFERENCE_TANGENT_POLICY)
        self.assertEqual(reference["ref_speed_role"], REF_SPEED_ROLE)
        self.assertEqual(
            reference["ref_speed_schedule_policy"],
            REF_SPEED_SCHEDULE_POLICY,
        )

    def test_terminal_liquid_schedule_and_arm_identity_are_explicit(self) -> None:
        semantics = self.document["semantic_contracts"]
        terminal = semantics["terminal"]
        self.assertEqual(terminal["stage_policy"], TERMINAL_STAGE_POLICY)
        self.assertEqual(
            terminal["residual_order"],
            ["contour", "lag", "v_actual-ref_speed", "omega_actual"],
        )
        self.assertEqual(
            terminal["normalization_order"],
            ["norm_contour", "norm_lag", "norm_v_actual", "norm_omega_actual"],
        )
        self.assertEqual(terminal["control_policy"], TERMINAL_CONTROL_POLICY)
        self.assertEqual(terminal["liquid_policy"], TERMINAL_LIQUID_POLICY)
        liquid_schedule = semantics["liquid_schedule"]
        self.assertEqual(
            semantics["running_cost"]["residual_order"],
            [
                "contour",
                "lag",
                "negative_progress",
                "v_actual-ref_speed",
                "v_s-ref_speed",
                "a_issue",
                "alpha_issue",
                "j_issue_v",
                "j_issue_omega",
            ],
        )
        self.assertEqual(
            semantics["running_cost"]["omega_actual_policy"],
            RUNNING_OMEGA_POLICY,
        )
        self.assertEqual(
            liquid_schedule["running_state_policy"],
            LIQUID_RUNNING_STATE_POLICY,
        )
        self.assertEqual(
            liquid_schedule["boundary_policy"], LIQUID_BOUNDARY_POLICY
        )
        self.assertEqual(
            liquid_schedule["boundary_state_policy"],
            LIQUID_BOUNDARY_STATE_POLICY,
        )
        self.assertEqual(
            liquid_schedule["running_eta_dot_ratio_policy"],
            LIQUID_RUNNING_RATIO_POLICY,
        )
        self.assertEqual(
            liquid_schedule["running_eta_dot_ratio_parameter"],
            SLOSH_RUNNING_RATIO_PARAMETER,
        )
        self.assertEqual(
            liquid_schedule["boundary_eta_dot_ratio_policy"],
            LIQUID_BOUNDARY_RATIO_POLICY,
        )
        comparison = semantics["comparison_identity"]
        self.assertEqual(comparison["arms"], ["B0", "Bslosh"])
        self.assertEqual(
            comparison["only_parameter_fields_allowed_to_differ"],
            ["liquid_run_coeff", "liquid_boundary_coeff"],
        )
        self.assertEqual(
            comparison["must_be_identical"],
            ["dynamics", "constraints", "all_other_stage_parameters"],
        )

    def test_strict_round_trip_returns_the_builder_result(self) -> None:
        rebuilt = solver_parameter_layout_from_dict(
            self.document,
            self.capacity,
            self.development_layout,
        )
        self.assertEqual(rebuilt, self.layout)
        self.assertEqual(rebuilt.to_dict(), self.document)

    def test_unknown_fields_and_numeric_aliases_are_rejected(self) -> None:
        top = copy.deepcopy(self.document)
        top["weight_defaults"] = {}
        nested = copy.deepcopy(self.document)
        nested["stage_vector_contract"]["terminal_may_omit_unused"] = True
        mutations = (
            top,
            nested,
            self.changed(("stage_vector_contract", "shooting_intervals"), 60.0),
            self.changed(("stage_vector_contract", "parameter_vector_count"), True),
            self.changed(("parameter_layout", "dimension"), 162.0),
            self.changed(
                (
                    "parameter_layout",
                    "block_ranges",
                    "execution_prefix",
                    "begin",
                ),
                False,
            ),
        )
        for index, value in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                SolverParameterLayoutError
            ):
                self.validate(value)

    def test_order_offset_range_identity_and_policy_tampering_are_rejected(
        self,
    ) -> None:
        swapped = copy.deepcopy(self.document)
        names = swapped["parameter_layout"]["ordered"]
        names[121], names[122] = names[122], names[121]
        mutations = (
            swapped,
            self.changed(
                ("parameter_layout", "offsets", "ref_s_origin"),
                122,
            ),
            self.changed(
                (
                    "parameter_layout",
                    "block_ranges",
                    "reference",
                    "end_exclusive",
                ),
                133,
            ),
            self.changed(
                ("source_identity", "development_capacity_raw_bytes_sha256"),
                "0" * 64,
            ),
            self.changed(
                (
                    "source_identity",
                    "development_layout",
                    "semantic_sha256",
                ),
                "1" * 64,
            ),
            self.changed(("reference_schema",), "global_polynomial_v0"),
            self.changed(
                (
                    "semantic_contracts",
                    "reference",
                    "effective_domain",
                    "upper",
                ),
                2,
            ),
            self.changed(
                ("semantic_contracts", "terminal", "control_policy"),
                "READ_U_N",
            ),
            self.changed(
                ("stage_vector_contract", "terminal_assignment_policy"),
                "OMIT_UNUSED",
            ),
            self.changed(("capability_status", "graph"), "GRAPH_READY"),
        )
        for index, value in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                SolverParameterLayoutError
            ):
                self.validate(value)

    def test_builder_rejects_raw_and_force_mutated_typed_sources(self) -> None:
        with self.assertRaises(SolverParameterLayoutError):
            build_solver_parameter_layout(
                self.capacity.to_dict(),  # type: ignore[arg-type]
                self.development_layout,
            )
        with self.assertRaises(SolverParameterLayoutError):
            build_solver_parameter_layout(
                self.capacity,
                self.development_layout.to_dict(),  # type: ignore[arg-type]
            )

        forged_capacity = copy.copy(self.capacity)
        object.__setattr__(forged_capacity, "np_exec", 120)
        with self.assertRaisesRegex(SolverParameterLayoutError, "capacity"):
            build_solver_parameter_layout(
                forged_capacity,
                self.development_layout,
            )

        forged_layout = copy.copy(self.development_layout)
        object.__setattr__(forged_layout, "np_exec", 120)
        with self.assertRaisesRegex(SolverParameterLayoutError, "development layout"):
            build_solver_parameter_layout(self.capacity, forged_layout)

    def test_layout_offsets_and_block_ranges_are_deeply_immutable(self) -> None:
        with self.assertRaises(TypeError):
            self.layout.parameter_offsets["ref_s_origin"] = 0  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.block_ranges["reference"] = ParameterBlockRange(0, 1)  # type: ignore[index]
        with self.assertRaises(FrozenInstanceError):
            self.layout.block_ranges["reference"].begin = 0  # type: ignore[misc]
        with self.assertRaises(TypeError):
            self.layout.parameter_names[0] = "changed"  # type: ignore[index]
        with self.assertRaises(FrozenInstanceError):
            self.layout.np = 161  # type: ignore[misc]
        with self.assertRaisesRegex(SolverParameterLayoutError, "requires"):
            replace(self.layout, np=161)

    def test_module_dependency_boundary_is_static_and_effective(self) -> None:
        source = MODULE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imports = set()
        builders = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module)
            elif isinstance(
                node, (ast.FunctionDef, ast.AsyncFunctionDef)
            ) and node.name.startswith("build_"):
                builders.append(node.name)
        forbidden = {
            "layout",
            "manifest",
            "contract_source",
            "stage1_evidence",
            "casadi",
            "rosbag",
            "rospy",
        }
        for imported in imports:
            self.assertFalse(
                any(
                    imported == name or imported.endswith(f".{name}")
                    for name in forbidden
                ),
                msg=f"forbidden dependency imported: {imported}",
            )
        self.assertEqual(builders, ["build_solver_parameter_layout"])

        probe = "\n".join(
            (
                "import json",
                "import sys",
                f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r})",
                "import acados.mainline.solver_parameter_layout",
                "forbidden = [",
                "    'acados.mainline.layout',",
                "    'acados.mainline.manifest',",
                "    'acados.mainline.contract_source',",
                "    'acados.mainline.stage1_evidence',",
                "    'casadi', 'rosbag', 'rospy',",
                "]",
                "print(json.dumps([name for name in forbidden if name in sys.modules]))",
            )
        )
        completed = subprocess.run(
            [sys.executable, "-c", probe],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(json.loads(completed.stdout), [])

    def test_schema_contains_no_numeric_weight_defaults(self) -> None:
        self.assertNotIn("weight_defaults", self.document)
        self.assertNotIn("defaults", self.document)
        parameter_layout = self.document["parameter_layout"]
        self.assertEqual(
            set(parameter_layout),
            {"ordered", "offsets", "block_order", "block_ranges", "dimension"},
        )


if __name__ == "__main__":
    unittest.main()
