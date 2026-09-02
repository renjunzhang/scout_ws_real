#!/usr/bin/env python3
"""Parity and contract tests for the Stage 3-D2c numeric discrete map."""

from __future__ import annotations

import ast
import json
import math
import random
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
GOLDEN_ROOT = PACKAGE_ROOT / "test" / "golden"
for import_root in (SCRIPTS_ROOT, GOLDEN_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from acados.mainline.cost_schedule import (
    ExperimentCondition,
    build_liquid_cost_schedule,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.discrete_dynamics import (
    DISCRETIZATION_SCHEMA,
    DiscreteDynamicsError,
    evaluate_discrete_map,
)
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
from stage2_execution_golden_reference import calculate as stage2_calculate

CAPACITY_PATH = (
    PACKAGE_ROOT
    / "config"
    / "mainline"
    / "contracts"
    / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "discrete_dynamics.py"


def make_values() -> CommonStageParameterValues:
    return CommonStageParameterValues(
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


class MainlineDiscreteDynamicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY_PATH)
        self.layout = build_development_layout(self.capacity)
        self.values = make_values()
        self.state_by_name = {
            "px": 0.1,
            "py": -0.2,
            "theta": 0.3,
            "s": 10.0,
            "v_actual": 0.2,
            "omega_actual": -0.1,
            "q_prev_v": 0.4,
            "q_prev_omega": -0.3,
            "a_prev": 0.1,
            "alpha_prev": -0.2,
            "eta_x": 0.01,
            "eta_x_dot": -0.02,
            "eta_y": 0.03,
            "eta_y_dot": -0.04,
        }
        self.older_v = tuple(
            0.35 - 0.025 * index for index in range(self.layout.d_v)
        )
        self.older_omega = tuple(
            -0.25 + 0.015 * index for index in range(self.layout.d_omega)
        )
        self.state_by_name.update(
            {
                f"older_v[{index}]": value
                for index, value in enumerate(self.older_v)
            }
        )
        self.state_by_name.update(
            {
                f"older_omega[{index}]": value
                for index, value in enumerate(self.older_omega)
            }
        )
        self.state = tuple(
            self.state_by_name[name] for name in self.layout.state_names
        )
        self.control = (0.4, -0.3, 0.7)

    def runtime_and_parameters(
        self,
        delay_v_sec: float,
        delay_omega_sec: float,
        condition: ExperimentCondition = ExperimentCondition.Bslosh,
    ):
        runtime = build_runtime_fractional_delay_schedule(
            self.capacity,
            delay_v_sec,
            delay_omega_sec,
            1.0e-12,
            1.0e-12,
        )
        liquid = build_liquid_cost_schedule(condition, 8, 4.0, 7.0)
        snapshot = assemble_runtime_stage_parameters(
            self.capacity,
            self.layout,
            runtime,
            self.values,
            liquid,
        )
        return runtime, snapshot

    def stage2_scenario(self, runtime) -> dict[str, Any]:
        return {
            "config": {
                "dt_sec": runtime.dt_sec,
                "maximum_linear_delay_sec": float(self.capacity.v.l_max_sec),
                "maximum_angular_delay_sec": float(
                    self.capacity.omega.l_max_sec
                ),
                "linear_delay_sec": runtime.delay_v_sec,
                "angular_delay_sec": runtime.delay_omega_sec,
                "integer_snap_tolerance_ratio": (
                    runtime.integer_snap_tolerance_sec / runtime.dt_sec
                ),
                "duration_tolerance_sec": runtime.duration_tolerance_sec,
            },
            "plant": {
                "linear_actuator": {
                    "tau_sec": self.values.actuator.tau_v_sec,
                    "gain": self.values.actuator.gain_v,
                },
                "angular_actuator": {
                    "tau_sec": self.values.actuator.tau_omega_sec,
                    "gain": self.values.actuator.gain_omega,
                },
                "liquid": {
                    "natural_frequency_rad_per_sec": (
                        self.values.slosh.omega_n_rad_per_sec
                    ),
                    "damping_ratio": self.values.slosh.damping_ratio,
                    "longitudinal_coupling": self.values.slosh.slosh_kappa_x,
                    "lateral_coupling": self.values.slosh.slosh_kappa_y,
                },
            },
            "state": {
                "physical": {
                    "pose": {
                        "x": self.state_by_name["px"],
                        "y": self.state_by_name["py"],
                        "heading": self.state_by_name["theta"],
                    },
                    "actual": {
                        "linear_velocity": self.state_by_name["v_actual"],
                        "angular_velocity": self.state_by_name[
                            "omega_actual"
                        ],
                    },
                    "liquid": {
                        "eta_x": self.state_by_name["eta_x"],
                        "eta_x_dot": self.state_by_name["eta_x_dot"],
                        "eta_y": self.state_by_name["eta_y"],
                        "eta_y_dot": self.state_by_name["eta_y_dot"],
                    },
                },
                "progress": self.state_by_name["s"],
                "publisher": {
                    "previous_linear_command": self.state_by_name[
                        "q_prev_v"
                    ],
                    "previous_angular_command": self.state_by_name[
                        "q_prev_omega"
                    ],
                    "previous_linear_acceleration": self.state_by_name[
                        "a_prev"
                    ],
                    "previous_angular_acceleration": self.state_by_name[
                        "alpha_prev"
                    ],
                },
                "linear_older": list(self.older_v),
                "angular_older": list(self.older_omega),
            },
            "control": {
                "linear_jerk": self.control[0],
                "angular_jerk": self.control[1],
                "progress_velocity": self.control[2],
            },
        }

    def evaluation_as_stage2_result(self, evaluation) -> dict[str, Any]:
        values = {
            name: evaluation.next_state[index]
            for index, name in enumerate(self.layout.state_names)
        }
        return {
            "issued": {
                "linear_command": evaluation.issued.q_issue_v,
                "angular_command": evaluation.issued.q_issue_omega,
                "linear_acceleration": evaluation.issued.a_issue,
                "angular_acceleration": evaluation.issued.alpha_issue,
            },
            "segments": [
                {
                    "duration_sec": segment.duration_sec,
                    "linear_target": segment.q_target_v,
                    "angular_target": segment.q_target_omega,
                }
                for segment in evaluation.segments
            ],
            "next_state": {
                "physical": {
                    "pose": {
                        "x": values["px"],
                        "y": values["py"],
                        "heading": values["theta"],
                    },
                    "actual": {
                        "linear_velocity": values["v_actual"],
                        "angular_velocity": values["omega_actual"],
                    },
                    "liquid": {
                        "eta_x": values["eta_x"],
                        "eta_x_dot": values["eta_x_dot"],
                        "eta_y": values["eta_y"],
                        "eta_y_dot": values["eta_y_dot"],
                    },
                },
                "progress": values["s"],
                "publisher": {
                    "previous_linear_command": values["q_prev_v"],
                    "previous_angular_command": values["q_prev_omega"],
                    "previous_linear_acceleration": values["a_prev"],
                    "previous_angular_acceleration": values["alpha_prev"],
                },
                "linear_older": [
                    values[f"older_v[{index}]"]
                    for index in range(self.layout.d_v)
                ],
                "angular_older": [
                    values[f"older_omega[{index}]"]
                    for index in range(self.layout.d_omega)
                ],
            },
        }

    def assert_nested_close(
        self,
        actual: Any,
        expected: Any,
        path: str = "result",
    ) -> None:
        if type(expected) is dict:
            self.assertEqual(set(actual), set(expected), path)
            for key in expected:
                self.assert_nested_close(actual[key], expected[key], f"{path}.{key}")
            return
        if type(expected) is list:
            self.assertEqual(len(actual), len(expected), path)
            for index, item in enumerate(expected):
                self.assert_nested_close(actual[index], item, f"{path}[{index}]")
            return
        self.assertAlmostEqual(actual, expected, delta=5.0e-13, msg=path)

    def test_matches_independent_stage2_oracle_across_delay_boundaries(self) -> None:
        dt = float(self.capacity.release_period_sec)
        cases = (
            (0.0, 0.0),
            (4.0 * dt, 7.0 * dt),
            (0.05, 0.07),
            (float(self.capacity.v.l_max_sec), float(self.capacity.omega.l_max_sec)),
        )
        for delay_v, delay_omega in cases:
            with self.subTest(delay_v=delay_v, delay_omega=delay_omega):
                runtime, snapshot = self.runtime_and_parameters(
                    delay_v,
                    delay_omega,
                )
                evaluation = evaluate_discrete_map(
                    self.capacity,
                    self.layout,
                    self.state,
                    self.control,
                    snapshot.stage_parameters[0],
                )
                independent = stage2_calculate(self.stage2_scenario(runtime))
                self.assert_nested_close(
                    self.evaluation_as_stage2_result(evaluation),
                    independent,
                )

    def test_dynamics_ignore_reference_cost_stage_and_experiment_arm(self) -> None:
        _, bslosh = self.runtime_and_parameters(0.05, 0.07)
        _, b0 = self.runtime_and_parameters(
            0.05,
            0.07,
            ExperimentCondition.B0,
        )
        results = tuple(
            evaluate_discrete_map(
                self.capacity,
                self.layout,
                self.state,
                self.control,
                parameters,
            ).next_state
            for parameters in (
                bslosh.stage_parameters[0],
                bslosh.stage_parameters[-1],
                b0.stage_parameters[0],
            )
        )
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0], results[2])

    def test_random_delay_pairs_match_the_independent_stage2_oracle(self) -> None:
        generator = random.Random(20260903)
        for case_index in range(64):
            delay_v = generator.random() * float(self.capacity.v.l_max_sec)
            delay_omega = generator.random() * float(
                self.capacity.omega.l_max_sec
            )
            runtime, snapshot = self.runtime_and_parameters(delay_v, delay_omega)
            evaluation = evaluate_discrete_map(
                self.capacity,
                self.layout,
                self.state,
                self.control,
                snapshot.stage_parameters[0],
            )
            independent = stage2_calculate(self.stage2_scenario(runtime))
            self.assert_nested_close(
                self.evaluation_as_stage2_result(evaluation),
                independent,
                f"random_case[{case_index}]",
            )

    def test_issue_shift_progress_and_three_slot_diagnostics_are_explicit(self) -> None:
        runtime, snapshot = self.runtime_and_parameters(0.05, 0.07)
        result = evaluate_discrete_map(
            self.capacity,
            self.layout,
            self.state,
            self.control,
            snapshot.stage_parameters[0],
        )
        dt = runtime.dt_sec
        expected_a = self.state_by_name["a_prev"] + dt * self.control[0]
        expected_q = (
            self.state_by_name["q_prev_v"]
            + dt * self.state_by_name["a_prev"]
            + 0.5 * dt * dt * self.control[0]
        )
        self.assertEqual(result.issued.a_issue, expected_a)
        self.assertEqual(result.issued.q_issue_v, expected_q)
        next_values = dict(zip(self.layout.state_names, result.next_state))
        self.assertEqual(next_values["q_prev_v"], result.issued.q_issue_v)
        self.assertEqual(next_values["a_prev"], result.issued.a_issue)
        self.assertEqual(next_values["older_v[0]"], self.state_by_name["q_prev_v"])
        self.assertEqual(next_values["older_v[1]"], self.older_v[0])
        self.assertEqual(next_values["s"], 10.0 + dt * self.control[2])
        self.assertEqual(len(result.segments), 3)
        self.assertEqual(
            tuple(segment.duration_sec for segment in result.segments),
            runtime.duration,
        )

    def test_malformed_vectors_dynamic_parameters_and_schedule_fail_closed(self) -> None:
        _, snapshot = self.runtime_and_parameters(0.05, 0.07)
        parameters = snapshot.stage_parameters[0]
        invalid_calls = (
            (self.state[:-1], self.control, parameters),
            (self.state, self.control[:-1], parameters),
            (self.state, self.control, parameters[:-1]),
            ((math.nan, *self.state[1:]), self.control, parameters),
            (self.state, (0.4, -0.3, -0.1), parameters),
            (self.state, (0.4, -0.3, 1), parameters),
        )
        for state, control, stage_parameters in invalid_calls:
            with self.subTest(
                lengths=(len(state), len(control), len(stage_parameters))
            ), self.assertRaises(DiscreteDynamicsError):
                evaluate_discrete_map(
                    self.capacity,
                    self.layout,
                    state,  # type: ignore[arg-type]
                    control,  # type: ignore[arg-type]
                    stage_parameters,
                )

        parameter_offsets = snapshot.parameter_names.index
        mutations = (
            (parameter_offsets("act_gain_v"), 0.0),
            (parameter_offsets("slosh_omega_n_sq"), 0.0),
            (parameter_offsets("act_seg_dt[0]"), 0.0),
            (parameter_offsets("act_sel_v[0][2]"), 0.0),
        )
        for offset, replacement in mutations:
            forged = list(parameters)
            forged[offset] = replacement
            with self.subTest(offset=offset), self.assertRaises(
                DiscreteDynamicsError
            ):
                evaluate_discrete_map(
                    self.capacity,
                    self.layout,
                    self.state,
                    self.control,
                    tuple(forged),
                )

    def test_result_is_immutable_json_finite_and_module_is_isolated(self) -> None:
        _, snapshot = self.runtime_and_parameters(0.05, 0.07)
        result = evaluate_discrete_map(
            self.capacity,
            self.layout,
            self.state,
            self.control,
            snapshot.stage_parameters[0],
        )
        self.assertEqual(result.to_dict()["discretization_schema"], DISCRETIZATION_SCHEMA)
        json.dumps(result.to_dict(), allow_nan=False)
        with self.assertRaises(FrozenInstanceError):
            result.issued.a_issue = 0.0  # type: ignore[misc]
        with self.assertRaises(TypeError):
            result.next_state[0] = 0.0  # type: ignore[index]

        tree = ast.parse(MODULE.read_text(encoding="utf-8"))
        imported_modules = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module)
        forbidden = (
            "casadi",
            "acados_template",
            "rospy",
            "rosbag",
            ".layout",
            ".manifest",
            "stage1_evidence",
            "runtime_parameter_assembler",
            "stage2_execution_golden_reference",
        )
        for imported in imported_modules:
            self.assertFalse(
                any(imported == name or imported.endswith(name) for name in forbidden),
                imported,
            )


if __name__ == "__main__":
    unittest.main()
