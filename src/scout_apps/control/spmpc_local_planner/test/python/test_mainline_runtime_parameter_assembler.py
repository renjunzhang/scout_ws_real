#!/usr/bin/env python3
"""Contract tests for complete Stage 3-D2b runtime parameter assembly."""

from __future__ import annotations

import ast
import copy
import json
import math
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.cost_schedule import (
    ExperimentCondition,
    build_liquid_cost_schedule,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import sha256_json
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
    ALLOWED_ARM_PARAMETER_DIFFERENCES,
    RuntimeParameterAssemblyError,
    assemble_runtime_stage_parameters,
    build_matched_runtime_parameter_pair,
    require_matched_runtime_parameter_pair,
    require_runtime_parameter_snapshot,
    runtime_parameter_snapshot_from_dict,
)
from acados.mainline.runtime_schedule import (
    build_runtime_fractional_delay_schedule,
)
from acados.mainline.solver_parameter_layout import (
    build_solver_parameter_layout,
)

CAPACITY_PATH = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "runtime_parameter_assembler.py"


def make_values() -> CommonStageParameterValues:
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


class MainlineRuntimeParameterAssemblerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY_PATH)
        self.development_layout = build_development_layout(self.capacity)
        self.layout = build_solver_parameter_layout(
            self.capacity,
            self.development_layout,
        )
        self.runtime = build_runtime_fractional_delay_schedule(
            self.capacity,
            0.05,
            0.07,
            1.0e-12,
            1.0e-12,
        )
        self.values = make_values()
        self.b0_schedule = build_liquid_cost_schedule(
            ExperimentCondition.B0,
            8,
            4.0,
            7.0,
        )
        self.bslosh_schedule = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            8,
            4.0,
            7.0,
        )

    def assemble(self, schedule=None):
        return assemble_runtime_stage_parameters(
            self.capacity,
            self.development_layout,
            self.runtime,
            self.values,
            self.bslosh_schedule if schedule is None else schedule,
        )

    def test_every_stage_has_the_complete_typed_order_and_width(self) -> None:
        snapshot = self.assemble()
        self.assertEqual((snapshot.N, snapshot.NP), (60, 162))
        self.assertEqual(len(snapshot.stage_parameters), 61)
        self.assertEqual(snapshot.parameter_names, self.layout.parameter_names)
        self.assertTrue(all(len(row) == 162 for row in snapshot.stage_parameters))
        self.assertTrue(
            all(
                type(value) is float and math.isfinite(value)
                for row in snapshot.stage_parameters
                for value in row
            )
        )

    def test_execution_schedule_is_filled_in_all_rows_from_typed_layout(self) -> None:
        snapshot = self.assemble()
        expected = (
            *self.values.actuator.execution_scalar_values,
            *self.runtime.duration,
            *(value for slot in self.runtime.selector_v for value in slot),
            *(value for slot in self.runtime.selector_omega for value in slot),
        )
        execution_range = self.layout.block_ranges["execution_prefix"]
        for row in snapshot.stage_parameters:
            self.assertEqual(
                row[execution_range.begin : execution_range.end_exclusive],
                expected,
            )
        self.assertEqual(snapshot.runtime_schedule_sha256, self.runtime.sha256)

    def test_reference_speed_and_liquid_coefficients_are_stage_specific(self) -> None:
        snapshot = self.assemble()
        speed_offset = self.layout.parameter_offsets["ref_speed"]
        run_offset = self.layout.parameter_offsets["liquid_run_coeff"]
        boundary_offset = self.layout.parameter_offsets["liquid_boundary_coeff"]
        for stage, row in enumerate(snapshot.stage_parameters):
            self.assertEqual(row[speed_offset], self.values.reference.ref_speed[stage])
            self.assertEqual(
                row[run_offset],
                self.bslosh_schedule.liquid_run_coeff[stage],
            )
            self.assertEqual(
                row[boundary_offset],
                self.bslosh_schedule.liquid_boundary_coeff[stage],
            )
        self.assertEqual(snapshot.stage_parameters[-1][run_offset], 0.0)
        self.assertEqual(snapshot.stage_parameters[-1][boundary_offset], 0.0)
        self.assertNotEqual(snapshot.stage_parameters[-1][speed_offset], 0.0)

    def test_terminal_row_is_fully_assigned_instead_of_zero_filled(self) -> None:
        terminal = self.assemble().stage_parameters[-1]
        first_execution = self.layout.parameter_offsets["act_inv_tau_v"]
        normalization = self.layout.parameter_offsets["norm_contour"]
        terminal_weight = self.layout.parameter_offsets["weight_terminal_contour"]
        self.assertGreater(terminal[first_execution], 0.0)
        self.assertGreater(terminal[normalization], 0.0)
        self.assertGreater(terminal[terminal_weight], 0.0)

    def test_snapshot_is_immutable_hashed_and_strictly_round_trips(self) -> None:
        snapshot = self.assemble()
        document = snapshot.to_dict()
        json.dumps(document, allow_nan=False)
        self.assertEqual(snapshot.sha256, sha256_json(document))
        self.assertEqual(
            runtime_parameter_snapshot_from_dict(
                document,
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.bslosh_schedule,
            ),
            snapshot,
        )
        with self.assertRaises(FrozenInstanceError):
            snapshot.condition = ExperimentCondition.B0  # type: ignore[misc]
        with self.assertRaises(TypeError):
            snapshot.stage_parameters[0][0] = 0.0  # type: ignore[index]

        document["stage_parameters"][60][0] = 0.0
        with self.assertRaises(RuntimeParameterAssemblyError):
            runtime_parameter_snapshot_from_dict(
                document,
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.bslosh_schedule,
            )

    def test_force_mutated_typed_inputs_and_snapshot_lose_authority(self) -> None:
        forged_values = copy.copy(self.values)
        object.__setattr__(forged_values, "slosh", self.values.normalization)
        with self.assertRaises(RuntimeParameterAssemblyError):
            assemble_runtime_stage_parameters(
                self.capacity,
                self.development_layout,
                self.runtime,
                forged_values,
                self.bslosh_schedule,
            )

        snapshot = self.assemble()
        forged_snapshot = copy.copy(snapshot)
        rows = list(snapshot.stage_parameters)
        first = list(rows[0])
        first[10] = 0.5
        rows[0] = tuple(first)
        object.__setattr__(forged_snapshot, "stage_parameters", tuple(rows))
        with self.assertRaises(RuntimeParameterAssemblyError):
            require_runtime_parameter_snapshot(
                forged_snapshot,
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.bslosh_schedule,
            )

    def test_matched_pair_whitelists_only_the_two_liquid_fields(self) -> None:
        pair = build_matched_runtime_parameter_pair(
            self.capacity,
            self.development_layout,
            self.runtime,
            self.values,
            self.b0_schedule,
            self.bslosh_schedule,
        )
        self.assertEqual(
            pair.allowed_parameter_names,
            ALLOWED_ARM_PARAMETER_DIFFERENCES,
        )
        self.assertEqual(
            pair.allowed_parameter_offsets,
            tuple(
                self.layout.parameter_offsets[name]
                for name in ALLOWED_ARM_PARAMETER_DIFFERENCES
            ),
        )
        self.assertEqual(pair.b0.condition, ExperimentCondition.B0)
        self.assertEqual(pair.bslosh.condition, ExperimentCondition.Bslosh)
        self.assertTrue(pair.differing_stage_offsets)
        self.assertTrue(
            all(
                offset in pair.allowed_parameter_offsets
                for _, offset in pair.differing_stage_offsets
            )
        )
        require_matched_runtime_parameter_pair(
            pair,
            self.capacity,
            self.development_layout,
            self.runtime,
            self.values,
            self.b0_schedule,
            self.bslosh_schedule,
        )
        json.dumps(pair.to_dict(), allow_nan=False)

    def test_pair_rejects_mismatched_window_weight_order_and_mutation(self) -> None:
        mismatched = (
            build_liquid_cost_schedule(
                ExperimentCondition.Bslosh,
                10,
                4.0,
                7.0,
            ),
            build_liquid_cost_schedule(
                ExperimentCondition.Bslosh,
                8,
                5.0,
                7.0,
            ),
        )
        for schedule in mismatched:
            with (
                self.subTest(schedule=schedule),
                self.assertRaises(RuntimeParameterAssemblyError),
            ):
                build_matched_runtime_parameter_pair(
                    self.capacity,
                    self.development_layout,
                    self.runtime,
                    self.values,
                    self.b0_schedule,
                    schedule,
                )
        with self.assertRaises(RuntimeParameterAssemblyError):
            build_matched_runtime_parameter_pair(
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.bslosh_schedule,
                self.b0_schedule,
            )

        pair = build_matched_runtime_parameter_pair(
            self.capacity,
            self.development_layout,
            self.runtime,
            self.values,
            self.b0_schedule,
            self.bslosh_schedule,
        )
        forged = copy.copy(pair)
        object.__setattr__(forged, "allowed_parameter_offsets", (0, 1))
        with self.assertRaises(RuntimeParameterAssemblyError):
            require_matched_runtime_parameter_pair(
                forged,
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.b0_schedule,
                self.bslosh_schedule,
            )

        forged_matrix_pair = copy.copy(pair)
        forged_bslosh = copy.copy(pair.bslosh)
        rows = list(forged_bslosh.stage_parameters)
        first_row = list(rows[0])
        first_row[0] += 1.0
        rows[0] = tuple(first_row)
        object.__setattr__(forged_bslosh, "stage_parameters", tuple(rows))
        object.__setattr__(forged_matrix_pair, "bslosh", forged_bslosh)
        with self.assertRaises(RuntimeParameterAssemblyError):
            require_matched_runtime_parameter_pair(
                forged_matrix_pair,
                self.capacity,
                self.development_layout,
                self.runtime,
                self.values,
                self.b0_schedule,
                self.bslosh_schedule,
            )

    def test_matched_pair_allows_zero_weights_with_no_actual_difference(self) -> None:
        b0 = build_liquid_cost_schedule(ExperimentCondition.B0, 8, 0.0, 0.0)
        bslosh = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            8,
            0.0,
            0.0,
        )
        pair = build_matched_runtime_parameter_pair(
            self.capacity,
            self.development_layout,
            self.runtime,
            self.values,
            b0,
            bslosh,
        )
        self.assertEqual(pair.differing_stage_offsets, ())
        self.assertEqual(pair.b0.stage_parameters, pair.bslosh.stage_parameters)

    def test_module_is_backend_neutral_and_does_not_duplicate_offsets(self) -> None:
        source = MODULE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imports = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module)
        forbidden = (
            "casadi",
            "acados_template",
            "rospy",
            "rosbag",
            ".layout",
            ".manifest",
            "stage1_evidence",
        )
        for imported in imports:
            self.assertFalse(
                any(imported == name or imported.endswith(name) for name in forbidden),
                imported,
            )
        self.assertNotIn("[160]", source)
        self.assertNotIn("[161]", source)


if __name__ == "__main__":
    unittest.main()
