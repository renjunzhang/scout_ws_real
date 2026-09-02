"""Contract tests for the independent Stage 3-D liquid cost schedule."""

from __future__ import annotations

import ast
import copy
import inspect
import json
import math
import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.cost_schedule import (
    LIQUID_SCHEDULE_LENGTH,
    ExperimentCondition,
    LiquidCostSchedule,
    LiquidCostScheduleError,
    build_liquid_cost_schedule,
    liquid_objective_scale,
    require_liquid_cost_schedule,
)

MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "cost_schedule.py"


class MainlineCostScheduleTest(unittest.TestCase):
    def test_candidate_and_edge_windows_use_right_endpoint_schedule_without_extra_scaling(
        self,
    ) -> None:
        for k_liquid in (1, 3, 5, 8, 10, 59):
            with self.subTest(k_liquid=k_liquid):
                schedule = build_liquid_cost_schedule(
                    ExperimentCondition.Bslosh,
                    k_liquid,
                    5.0,
                    2.5,
                )
                self.assertEqual(schedule.N, 60)
                self.assertEqual(
                    len(schedule.liquid_run_coeff), LIQUID_SCHEDULE_LENGTH
                )
                self.assertEqual(
                    len(schedule.liquid_boundary_coeff),
                    LIQUID_SCHEDULE_LENGTH,
                )
                self.assertTrue(
                    all(
                        value == 5.0 / k_liquid
                        for value in schedule.liquid_run_coeff[:k_liquid]
                    )
                )
                self.assertTrue(
                    all(value == 0.0 for value in schedule.liquid_run_coeff[k_liquid:])
                )
                self.assertEqual(schedule.liquid_boundary_coeff[k_liquid], 2.5)
                self.assertEqual(
                    sum(value != 0.0 for value in schedule.liquid_boundary_coeff),
                    1,
                )
                self.assertTrue(
                    all(
                        value == 0.0
                        for value in schedule.liquid_boundary_coeff[:k_liquid]
                    )
                )
                self.assertTrue(
                    all(
                        value == 0.0
                        for value in schedule.liquid_boundary_coeff[k_liquid + 1 :]
                    )
                )
                self.assertAlmostEqual(
                    math.fsum(schedule.liquid_run_coeff), 5.0, places=12
                )

    def test_b0_is_all_zero_even_with_positive_weights(self) -> None:
        schedule = build_liquid_cost_schedule(
            ExperimentCondition.B0,
            8,
            7.0,
            3.0,
        )
        self.assertEqual(liquid_objective_scale(ExperimentCondition.B0), 0.0)
        self.assertEqual(schedule.objective_scale, 0.0)
        self.assertTrue(all(value == 0.0 for value in schedule.liquid_run_coeff))
        self.assertTrue(
            all(value == 0.0 for value in schedule.liquid_boundary_coeff)
        )

    def test_zero_boundary_weight_has_no_nonzero_boundary(self) -> None:
        schedule = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            10,
            2.0,
            0.0,
        )
        self.assertEqual(schedule.boundary_stage, 10)
        self.assertTrue(
            all(value == 0.0 for value in schedule.liquid_boundary_coeff)
        )

    def test_zero_running_weight_and_negative_zero_are_canonical_zero(self) -> None:
        schedule = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            8,
            -0.0,
            -0.0,
        )
        self.assertEqual(math.copysign(1.0, schedule.W_run), 1.0)
        self.assertEqual(math.copysign(1.0, schedule.W_boundary), 1.0)
        self.assertTrue(all(value == 0.0 for value in schedule.liquid_run_coeff))
        self.assertTrue(
            all(value == 0.0 for value in schedule.liquid_boundary_coeff)
        )

    def test_extreme_running_weights_fail_closed_without_raw_numeric_errors(
        self,
    ) -> None:
        minimum_subnormal = float.fromhex("0x0.0000000000001p-1022")
        for value in (minimum_subnormal, sys.float_info.max):
            with self.subTest(value=value), self.assertRaises(
                LiquidCostScheduleError
            ):
                build_liquid_cost_schedule(
                    ExperimentCondition.Bslosh,
                    3,
                    value,
                    0.0,
                )

    def test_condition_is_strongly_typed_and_scale_is_closed(self) -> None:
        self.assertEqual(liquid_objective_scale(ExperimentCondition.Bslosh), 1.0)
        for condition in ("B0", "Bslosh", None, 0):
            with self.subTest(condition=condition), self.assertRaises(
                LiquidCostScheduleError
            ):
                liquid_objective_scale(condition)  # type: ignore[arg-type]

        with self.assertRaises(LiquidCostScheduleError):
            build_liquid_cost_schedule("B0", 8, 1.0, 1.0)  # type: ignore[arg-type]

    def test_invalid_window_values_fail_closed(self) -> None:
        invalid = (True, False, 8.0, "8", 0, 60, -1, None)
        for value in invalid:
            with self.subTest(K_liquid=value), self.assertRaises(
                LiquidCostScheduleError
            ):
                build_liquid_cost_schedule(
                    ExperimentCondition.Bslosh,
                    value,  # type: ignore[arg-type]
                    1.0,
                    1.0,
                )

    def test_invalid_weights_fail_closed(self) -> None:
        invalid_weights = (True, False, "1", float("nan"), float("inf"), -1.0, None)
        for value in invalid_weights:
            for position in ("run", "boundary"):
                with self.subTest(value=value, position=position), self.assertRaises(
                    LiquidCostScheduleError
                ):
                    build_liquid_cost_schedule(
                        ExperimentCondition.Bslosh,
                        8,
                        value if position == "run" else 1.0,  # type: ignore[arg-type]
                        value if position == "boundary" else 1.0,  # type: ignore[arg-type]
                    )

    def test_output_is_immutable_and_builder_has_no_hidden_defaults(self) -> None:
        schedule = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh, 8, 1.0, 1.0
        )
        with self.assertRaises(FrozenInstanceError):
            schedule.K_liquid = 10  # type: ignore[misc]
        with self.assertRaises(TypeError):
            schedule.liquid_run_coeff[0] = 1.0  # type: ignore[index]
        with self.assertRaises(FrozenInstanceError):
            schedule.liquid_boundary_coeff += (1.0,)  # type: ignore[misc]

        parameters = inspect.signature(build_liquid_cost_schedule).parameters
        self.assertEqual(
            tuple(parameters), ("condition", "K_liquid", "W_run", "W_boundary")
        )
        self.assertTrue(all(parameter.default is inspect.Parameter.empty for parameter in parameters.values()))

        document = schedule.to_dict()
        self.assertEqual(
            document["schema_version"],
            "spmpc_mainline_right_endpoint_liquid_schedule_v1",
        )
        self.assertEqual(
            document["horizon"], {"N": 60, "parameter_vector_count": 61}
        )
        self.assertEqual(document["condition"], "Bslosh")
        self.assertEqual(document["inputs"]["K_liquid"], 8)
        self.assertEqual(len(document["liquid_run_coeff"]), 61)
        self.assertEqual(len(document["liquid_boundary_coeff"]), 61)

    def test_noncanonical_or_force_mutated_schedule_is_rejected(self) -> None:
        boundary = [0.0] * LIQUID_SCHEDULE_LENGTH
        boundary[8] = 1.0
        misplaced_run = [0.0] * LIQUID_SCHEDULE_LENGTH
        misplaced_run[8] = 1.0
        with self.assertRaisesRegex(
            LiquidCostScheduleError,
            "uniform on stages",
        ):
            LiquidCostSchedule(
                condition=ExperimentCondition.Bslosh,
                K_liquid=8,
                W_run=1.0,
                W_boundary=1.0,
                objective_scale=1.0,
                liquid_run_coeff=tuple(misplaced_run),
                liquid_boundary_coeff=tuple(boundary),
            )

        schedule = build_liquid_cost_schedule(
            ExperimentCondition.Bslosh,
            8,
            1.0,
            1.0,
        )
        forged = copy.copy(schedule)
        object.__setattr__(forged, "W_run", 1)
        with self.assertRaisesRegex(LiquidCostScheduleError, "canonical"):
            require_liquid_cost_schedule(forged)

        negative_zero = copy.copy(schedule)
        coefficients = list(schedule.liquid_run_coeff)
        coefficients[-1] = -0.0
        object.__setattr__(negative_zero, "liquid_run_coeff", tuple(coefficients))
        with self.assertRaisesRegex(LiquidCostScheduleError, "canonical"):
            require_liquid_cost_schedule(negative_zero)

    def test_module_is_independent_of_old_contracts_and_solver_dependencies(
        self,
    ) -> None:
        tree = ast.parse(MODULE.read_text(encoding="utf-8"))
        imported_modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module)
        forbidden = (
            "layout",
            "manifest",
            "contract_source",
            "stage1_evidence",
            "rosbag",
            "casadi",
            "acados_template",
        )
        for imported in imported_modules:
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
                "import acados.mainline.cost_schedule",
                "forbidden = [",
                "    'acados.mainline.layout',",
                "    'acados.mainline.manifest',",
                "    'acados.mainline.contract_source',",
                "    'acados.mainline.stage1_evidence',",
                "    'rosbag',",
                "    'casadi',",
                "    'acados_template',",
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

        source = MODULE.read_text(encoding="utf-8")
        self.assertNotIn("AcadosModel", source)
        self.assertNotIn("generate_", source)


if __name__ == "__main__":
    unittest.main()
