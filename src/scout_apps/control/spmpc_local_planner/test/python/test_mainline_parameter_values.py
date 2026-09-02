#!/usr/bin/env python3
"""Contract tests for explicit Stage 3-D runtime parameter blocks."""

from __future__ import annotations

import ast
import copy
import inspect
import json
import math
import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError, fields
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.parameter_values import (
    REFERENCE_SPEED_COUNT,
    ActuatorResponseParameters,
    CommonStageParameterValues,
    NormalizationParameters,
    ParameterValueError,
    ReferenceHorizonParameters,
    ReferencePolynomialParameters,
    RunningWeightParameters,
    SloshParameters,
    TerminalWeightParameters,
    require_common_stage_parameter_values,
)
from acados.mainline.solver_parameter_layout import (
    NORMALIZATION_PARAMETER_ORDER,
    REFERENCE_PARAMETER_ORDER,
    RUNNING_WEIGHT_PARAMETER_ORDER,
    SLOSH_PARAMETER_ORDER,
    TERMINAL_WEIGHT_PARAMETER_ORDER,
)

MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "parameter_values.py"


def make_common_values() -> CommonStageParameterValues:
    polynomial = ReferencePolynomialParameters(
        ref_s_origin=2.0,
        ref_s_scale=0.8,
        ref_x_coeff=(0.1, 0.8, 0.05, -0.02),
        ref_y_coeff=(-0.2, 0.1, 0.03, 0.01),
        minimum_tangent_norm_per_s=0.5,
    )
    return CommonStageParameterValues(
        actuator=ActuatorResponseParameters(
            tau_v_sec=0.12,
            gain_v=1.03,
            tau_omega_sec=0.45,
            gain_omega=0.98,
        ),
        reference=ReferenceHorizonParameters(
            polynomial=polynomial,
            ref_speed=tuple(0.2 - 0.1 * stage / 60.0 for stage in range(61)),
        ),
        slosh=SloshParameters(
            omega_n_rad_per_sec=5.0,
            damping_ratio=0.05,
            slosh_kappa_x=1.1,
            slosh_kappa_y=0.9,
            slosh_eta_ref=0.01,
            slosh_running_eta_dot_ratio=0.3,
        ),
        normalization=NormalizationParameters(
            norm_contour=0.1,
            norm_lag=0.2,
            norm_v_actual=0.5,
            norm_omega_actual=1.0,
            norm_v_s=0.5,
            norm_a_issue=0.8,
            norm_alpha_issue=1.2,
            norm_jerk_v=2.0,
            norm_jerk_omega=3.0,
        ),
        running_weight=RunningWeightParameters(
            weight_contour=1.0,
            weight_lag=0.2,
            weight_progress=0.3,
            weight_v_actual=0.7,
            weight_v_s=0.4,
            weight_a_issue=0.1,
            weight_alpha_issue=0.15,
            weight_jerk_v=0.05,
            weight_jerk_omega=0.08,
        ),
        terminal_weight=TerminalWeightParameters(
            weight_terminal_contour=2.0,
            weight_terminal_lag=0.4,
            weight_terminal_v_actual=1.5,
            weight_terminal_omega_actual=0.6,
        ),
    )


class MainlineParameterValuesTest(unittest.TestCase):
    def test_canonical_blocks_match_the_frozen_parameter_orders(self) -> None:
        values = make_common_values()
        self.assertEqual(
            values.actuator.execution_scalar_values,
            (1.0 / 0.12, 1.03, 1.0 / 0.45, 0.98),
        )
        self.assertEqual(
            dict(zip(REFERENCE_PARAMETER_ORDER, values.reference.values_for_stage(0))),
            {
                "ref_s_origin": 2.0,
                "ref_s_scale": 0.8,
                "ref_x_coeff[0]": 0.1,
                "ref_x_coeff[1]": 0.8,
                "ref_x_coeff[2]": 0.05,
                "ref_x_coeff[3]": -0.02,
                "ref_y_coeff[0]": -0.2,
                "ref_y_coeff[1]": 0.1,
                "ref_y_coeff[2]": 0.03,
                "ref_y_coeff[3]": 0.01,
                "ref_speed": 0.2,
            },
        )
        self.assertEqual(
            dict(zip(SLOSH_PARAMETER_ORDER, values.slosh.ordered_values)),
            {
                "slosh_two_zeta_omega_n": 0.5,
                "slosh_omega_n_sq": 25.0,
                "slosh_kappa_x": 1.1,
                "slosh_kappa_y": 0.9,
                "slosh_eta_ref": 0.01,
                "slosh_running_eta_dot_ratio": 0.3,
            },
        )
        self.assertEqual(
            tuple(field.name for field in fields(values.normalization)),
            NORMALIZATION_PARAMETER_ORDER,
        )
        self.assertEqual(
            tuple(field.name for field in fields(values.running_weight)),
            RUNNING_WEIGHT_PARAMETER_ORDER,
        )
        self.assertEqual(
            tuple(field.name for field in fields(values.terminal_weight)),
            TERMINAL_WEIGHT_PARAMETER_ORDER,
        )
        self.assertEqual(len(values.reference.ref_speed), REFERENCE_SPEED_COUNT)
        self.assertEqual(len(values.reference.values_for_stage(0)), 11)
        self.assertEqual(len(values.reference.values_for_stage(60)), 11)
        self.assertEqual(
            require_common_stage_parameter_values(values),
            values,
        )

    def test_snapshot_is_complete_and_json_compatible(self) -> None:
        document = make_common_values().to_dict()
        self.assertEqual(
            document["schema_version"], "spmpc_mainline_parameter_values_v1"
        )
        self.assertEqual(len(document["reference"]["ref_speed"]), 61)
        self.assertEqual(
            document["reference"]["polynomial"]["validation_policy"][
                "effective_xi_domain"
            ],
            {"lower": 0, "upper": 1},
        )
        self.assertGreater(
            document["reference"]["polynomial"]["validation_policy"][
                "validated_minimum_tangent_norm_per_s"
            ],
            0.5,
        )
        json.dumps(document, allow_nan=False)

    def test_reference_rejects_interior_and_endpoint_tangent_degeneracy(
        self,
    ) -> None:
        invalid_coefficients = (
            ((0.0, 1.0, -1.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
            ((0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
            ((0.0, 1.0, -0.5, 0.0), (0.0, 0.0, 0.0, 0.0)),
        )
        for x_coeff, y_coeff in invalid_coefficients:
            with self.subTest(x_coeff=x_coeff), self.assertRaises(
                ParameterValueError
            ):
                ReferencePolynomialParameters(
                    0.0,
                    1.0,
                    x_coeff,
                    y_coeff,
                    1.0e-6,
                )

    def test_reference_checks_vector_tangent_over_only_the_effective_domain(
        self,
    ) -> None:
        rotating = ReferencePolynomialParameters(
            0.0,
            1.0,
            (0.0, 1.0, -1.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            0.7,
        )
        self.assertGreaterEqual(
            rotating.to_dict()["validation_policy"][
                "validated_minimum_tangent_norm_per_s"
            ],
            0.7,
        )
        outside_only = ReferencePolynomialParameters(
            0.0,
            1.0,
            (0.0, 1.0, -0.25, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            0.5,
        )
        self.assertGreaterEqual(
            outside_only.to_dict()["validation_policy"][
                "validated_minimum_tangent_norm_per_s"
            ],
            0.5,
        )

    def test_reference_speed_shape_values_and_stage_index_fail_closed(self) -> None:
        polynomial = make_common_values().reference.polynomial
        invalid_schedules = (
            (0.2,) * 60,
            (0.2,) * 60 + (-0.1,),
            (0.2,) * 60 + (float("nan"),),
            (0.2,) * 60 + (1,),
            [0.2] * 61,
        )
        for schedule in invalid_schedules:
            with self.subTest(schedule_type=type(schedule)), self.assertRaises(
                ParameterValueError
            ):
                ReferenceHorizonParameters(
                    polynomial,
                    schedule,  # type: ignore[arg-type]
                )
        reference = make_common_values().reference
        for stage in (-1, 61, 0.0, True):
            with self.subTest(stage=stage), self.assertRaises(
                ParameterValueError
            ):
                reference.values_for_stage(stage)  # type: ignore[arg-type]

    def test_actuator_slosh_normalization_and_weight_domains_are_explicit(
        self,
    ) -> None:
        ActuatorResponseParameters(0.1, 1.0, 0.2, 1.0)
        SloshParameters(5.0, 0.0, 1.0, 1.0, 0.01, 0.0)
        RunningWeightParameters(*((0.0,) * 9))
        TerminalWeightParameters(*((0.0,) * 4))

        invalid_actuator = (
            (0.0, 1.0, 0.2, 1.0),
            (0.1, -1.0, 0.2, 1.0),
            (float.fromhex("0x0.0000000000001p-1022"), 1.0, 0.2, 1.0),
            (0.1, 1, 0.2, 1.0),
        )
        for arguments in invalid_actuator:
            with self.subTest(arguments=arguments), self.assertRaises(
                ParameterValueError
            ):
                ActuatorResponseParameters(*arguments)  # type: ignore[arg-type]

        with self.assertRaises(ParameterValueError):
            SloshParameters(0.0, 0.1, 1.0, 1.0, 0.01, 0.3)
        with self.assertRaises(ParameterValueError):
            SloshParameters(5.0, -0.1, 1.0, 1.0, 0.01, 0.3)
        with self.assertRaises(ParameterValueError):
            SloshParameters(
                float.fromhex("0x0.0000000000001p-1022"),
                1.0,
                1.0,
                1.0,
                0.01,
                0.3,
            )
        with self.assertRaises(ParameterValueError):
            NormalizationParameters(*((1.0,) * 8), 0.0)
        with self.assertRaises(ParameterValueError):
            NormalizationParameters(
                float.fromhex("0x0.0000000000001p-1022"),
                *((1.0,) * 8),
            )
        with self.assertRaises(ParameterValueError):
            RunningWeightParameters(*((0.0,) * 8), -1.0)
        with self.assertRaises(ParameterValueError):
            TerminalWeightParameters(*((0.0,) * 3), float("inf"))

    def test_numeric_aliases_and_nonfinite_values_are_rejected_everywhere(
        self,
    ) -> None:
        values = make_common_values()
        mutations = (
            ("actuator", "gain_v", True),
            ("slosh", "slosh_eta_ref", float("nan")),
            ("normalization", "norm_contour", 1),
            ("running_weight", "weight_contour", False),
            ("terminal_weight", "weight_terminal_contour", float("inf")),
        )
        for block_name, field_name, replacement in mutations:
            block = copy.copy(getattr(values, block_name))
            object.__setattr__(block, field_name, replacement)
            forged = copy.copy(values)
            object.__setattr__(forged, block_name, block)
            with self.subTest(block=block_name), self.assertRaises(
                ParameterValueError
            ):
                require_common_stage_parameter_values(forged)

        polynomial = copy.copy(values.reference.polynomial)
        object.__setattr__(polynomial, "ref_s_origin", 2)
        reference = copy.copy(values.reference)
        object.__setattr__(reference, "polynomial", polynomial)
        forged = copy.copy(values)
        object.__setattr__(forged, "reference", reference)
        with self.assertRaises(ParameterValueError):
            require_common_stage_parameter_values(forged)

    def test_force_mutation_and_wrong_nested_types_lose_authority(self) -> None:
        values = make_common_values()
        forged_reference = copy.copy(values.reference)
        object.__setattr__(forged_reference, "ref_speed", (0.2,) * 60)
        forged = copy.copy(values)
        object.__setattr__(forged, "reference", forged_reference)
        with self.assertRaises(ParameterValueError):
            require_common_stage_parameter_values(forged)
        with self.assertRaises(ParameterValueError):
            CommonStageParameterValues(
                values.actuator,
                values.reference.to_dict(),  # type: ignore[arg-type]
                values.slosh,
                values.normalization,
                values.running_weight,
                values.terminal_weight,
            )

    def test_nested_negative_zero_is_rebuilt_to_canonical_positive_zero(
        self,
    ) -> None:
        values = make_common_values()
        polynomial = copy.copy(values.reference.polynomial)
        object.__setattr__(polynomial, "ref_s_origin", -0.0)
        reference = ReferenceHorizonParameters(
            polynomial,
            values.reference.ref_speed,
        )
        self.assertEqual(math.copysign(1.0, reference.polynomial.ref_s_origin), 1.0)

        forged = copy.copy(values)
        object.__setattr__(forged, "reference", reference)
        canonical = require_common_stage_parameter_values(forged)
        self.assertEqual(
            math.copysign(
                1.0,
                canonical.reference.polynomial.ref_s_origin,
            ),
            1.0,
        )

    def test_all_constructor_fields_are_explicit_and_outputs_are_immutable(
        self,
    ) -> None:
        classes = (
            ActuatorResponseParameters,
            ReferencePolynomialParameters,
            ReferenceHorizonParameters,
            SloshParameters,
            NormalizationParameters,
            RunningWeightParameters,
            TerminalWeightParameters,
            CommonStageParameterValues,
        )
        for block_type in classes:
            with self.subTest(block_type=block_type.__name__):
                parameters = inspect.signature(block_type).parameters.values()
                self.assertTrue(
                    all(
                        parameter.default is inspect.Parameter.empty
                        for parameter in parameters
                    )
                )
        values = make_common_values()
        with self.assertRaises(FrozenInstanceError):
            values.actuator.gain_v = 2.0  # type: ignore[misc]
        with self.assertRaises(TypeError):
            values.reference.ref_speed[0] = 0.0  # type: ignore[index]

    def test_module_has_no_config_solver_or_legacy_authority_dependency(
        self,
    ) -> None:
        source = MODULE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imports = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module)
        forbidden = (
            "yaml",
            "rospy",
            "rosbag",
            "casadi",
            "acados_template",
            "layout",
            "manifest",
            "contract_source",
            "stage1_evidence",
        )
        for imported in imports:
            self.assertFalse(
                any(
                    imported == name or imported.endswith(f".{name}")
                    for name in forbidden
                ),
                msg=f"forbidden dependency imported: {imported}",
            )

        probe = "\n".join(
            (
                "import json",
                "import sys",
                f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r})",
                "import acados.mainline.parameter_values",
                "forbidden = [",
                " 'yaml', 'rospy', 'rosbag', 'casadi', 'acados_template',",
                " 'acados.mainline.layout', 'acados.mainline.manifest',",
                " 'acados.mainline.contract_source',",
                " 'acados.mainline.stage1_evidence',",
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
        self.assertNotIn("open(", source)


if __name__ == "__main__":
    unittest.main()
