"""Contract tests for the ROS-free Stage 3-D2b runtime delay compiler."""

from __future__ import annotations

import ast
import copy
import inspect
import math
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.identity import sha256_json
from acados.mainline.runtime_schedule import (
    RUNTIME_SCHEDULE_SCHEMA_VERSION,
    RuntimeFractionalDelaySchedule,
    RuntimeScheduleError,
    build_runtime_fractional_delay_schedule,
    require_runtime_fractional_delay_schedule,
    runtime_fractional_delay_schedule_from_dict,
)

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "runtime_schedule.py"


class MainlineRuntimeScheduleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)

    def build(
        self,
        v: float,
        omega: float,
        snap: float = 1e-12,
        duration_tolerance: float = 1e-12,
    ) -> RuntimeFractionalDelaySchedule:
        return build_runtime_fractional_delay_schedule(
            self.capacity, v, omega, snap, duration_tolerance
        )

    @staticmethod
    def indices(selectors: tuple[tuple[float, ...], ...]) -> tuple[int, ...]:
        return tuple(slot.index(1.0) for slot in selectors)

    def test_zero_integer_and_maximum_are_integer_boundaries(self) -> None:
        dt = float(self.capacity.release_period_sec)
        zero = self.build(0.0, 0.0, 0.0)
        self.assertEqual(
            (zero.m_v, zero.beta_v, zero.m_omega, zero.beta_omega), (0, 0.0, 0, 0.0)
        )
        self.assertEqual(zero.duration, (dt, 0.0, 0.0))
        self.assertEqual(self.indices(zero.selector_v), (0, 0, 0))
        self.assertEqual(self.indices(zero.selector_omega), (0, 0, 0))

        integer = self.build(4.0 / 30.0, 7.0 / 30.0, 0.0)
        self.assertEqual((integer.m_v, integer.m_omega), (4, 7))
        self.assertEqual((integer.beta_v, integer.beta_omega), (0.0, 0.0))
        self.assertEqual(self.indices(integer.selector_v), (4, 0, 0))
        self.assertEqual(self.indices(integer.selector_omega), (7, 0, 0))

        maximum = self.build(0.4, 0.8, 0.0)
        self.assertEqual(
            (maximum.m_v, maximum.m_omega),
            (self.capacity.NQ_v - 1, self.capacity.NQ_omega - 1),
        )
        self.assertEqual((maximum.beta_v, maximum.beta_omega), (0.0, 0.0))
        self.assertEqual(
            self.indices(maximum.selector_v), (self.capacity.NQ_v - 1, 0, 0)
        )
        self.assertEqual(
            self.indices(maximum.selector_omega), (self.capacity.NQ_omega - 1, 0, 0)
        )

    def test_fractional_delays_use_exact_union_and_final_remainder(self) -> None:
        schedule = self.build(0.05, 0.07)
        self.assertEqual((schedule.m_v, schedule.m_omega), (1, 2))
        self.assertAlmostEqual(schedule.beta_v, 0.5)
        self.assertAlmostEqual(schedule.beta_omega, 0.1)
        self.assertEqual(self.indices(schedule.selector_v), (2, 2, 1))
        self.assertEqual(self.indices(schedule.selector_omega), (3, 2, 2))
        self.assertEqual(
            schedule.duration[-1],
            schedule.dt_sec - sum(schedule.duration[:-1]),
        )
        self.assertEqual(sum(schedule.duration), schedule.dt_sec)
        self.assertEqual(schedule.duration[0], schedule.beta_omega * schedule.dt_sec)
        self.assertEqual(
            schedule.duration[1],
            schedule.beta_v * schedule.dt_sec - schedule.duration[0],
        )

    def test_snap_tolerance_is_seconds_and_duration_contract_matches_cpp(self) -> None:
        dt = float(self.capacity.release_period_sec)
        tolerance = 1e-9
        for ratio in (2.0 - 0.5 * tolerance / dt, 2.0 + 0.5 * tolerance / dt):
            schedule = self.build(ratio * dt, ratio * dt, tolerance)
            self.assertEqual((schedule.m_v, schedule.m_omega), (2, 2))
            self.assertEqual((schedule.beta_v, schedule.beta_omega), (0.0, 0.0))

        below_maximum = self.build(0.4 - 1.0e-14, 0.8 - 1.0e-14, 0.0)
        self.assertGreater(below_maximum.beta_v, 0.0)
        self.assertGreater(below_maximum.beta_omega, 0.0)

        self.assertEqual(self.build(0.0, 0.0, 0.0, 0.5 * dt).duration[0], dt)
        for snap in (-1e-12, 0.5 * dt, math.inf, math.nan):
            with self.subTest(snap=snap), self.assertRaises(RuntimeScheduleError):
                self.build(0.0, 0.0, snap)
        for duration_tolerance in (-1e-12, dt, math.inf, math.nan):
            with (
                self.subTest(duration_tolerance=duration_tolerance),
                self.assertRaises(RuntimeScheduleError),
            ):
                self.build(0.0, 0.0, 0.0, duration_tolerance)

    def test_invalid_values_and_out_of_range_delays_fail_closed(self) -> None:
        invalid = (
            (True, 0.0, 0.0, 1e-12),
            (0.0, 0, 0.0, 1e-12),
            (0.0, 0.0, True, 1e-12),
            (0.0, 0.0, 0.0, 0),
            (-1e-9, 0.0, 0.0, 1e-12),
            (0.0, math.nan, 0.0, 1e-12),
            (0.0, 0.0, math.inf, 1e-12),
        )
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(RuntimeScheduleError):
                build_runtime_fractional_delay_schedule(self.capacity, *args)
        with self.assertRaises(RuntimeScheduleError):
            self.build(0.4 + 1e-10, 0.0)
        with self.assertRaises(RuntimeScheduleError):
            self.build(0.0, 0.8 + 1e-10)

    def test_widths_are_read_from_pinned_capacity_and_forged_capacity_fails(
        self,
    ) -> None:
        schedule = self.build(0.05, 0.07)
        self.assertEqual(
            (schedule.nq_v, schedule.nq_omega),
            (self.capacity.NQ_v, self.capacity.NQ_omega),
        )
        self.assertEqual(len(schedule.selector_v[0]), self.capacity.NQ_v)
        self.assertEqual(len(schedule.selector_omega[0]), self.capacity.NQ_omega)

        forged = copy.copy(self.capacity)
        object.__setattr__(forged, "v", self.capacity.omega)
        with self.assertRaises(RuntimeScheduleError):
            build_runtime_fractional_delay_schedule(forged, 0.0, 0.0, 0.0, 1e-12)

    def test_snapshot_is_immutable_canonical_hashed_and_strictly_round_trips(
        self,
    ) -> None:
        schedule = self.build(0.05, 0.07)
        with self.assertRaises(FrozenInstanceError):
            schedule.m_v = 4  # type: ignore[misc]
        with self.assertRaises(TypeError):
            schedule.duration[0] = 1.0  # type: ignore[index]

        document = schedule.to_dict()
        self.assertEqual(document["schema_version"], RUNTIME_SCHEDULE_SCHEMA_VERSION)
        self.assertEqual(schedule.sha256, sha256_json(document))
        document["merged_schedule"]["duration_sec"][0] = 1.0
        self.assertNotEqual(document, schedule.to_dict())
        self.assertEqual(
            runtime_fractional_delay_schedule_from_dict(
                schedule.to_dict(), self.capacity
            ),
            schedule,
        )
        self.assertIs(
            require_runtime_fractional_delay_schedule(schedule, self.capacity), schedule
        )

        forged = copy.copy(schedule)
        object.__setattr__(forged, "m_v", 99)
        with self.assertRaises(RuntimeScheduleError):
            require_runtime_fractional_delay_schedule(forged, self.capacity)
        canonical_zero = self.build(0.0, 0.0, 0.0)
        negative_zero = copy.copy(canonical_zero)
        object.__setattr__(
            negative_zero,
            "duration",
            (canonical_zero.duration[0], -0.0, 0.0),
        )
        with self.assertRaises(RuntimeScheduleError):
            require_runtime_fractional_delay_schedule(negative_zero, self.capacity)

        forged_document = schedule.to_dict()
        forged_document["merged_schedule"]["duration_sec"][0] = 1.0
        with self.assertRaises(RuntimeScheduleError):
            runtime_fractional_delay_schedule_from_dict(forged_document, self.capacity)
        unknown = schedule.to_dict()
        unknown["merged_schedule"]["extra"] = 0.0
        with self.assertRaises(RuntimeScheduleError):
            runtime_fractional_delay_schedule_from_dict(unknown, self.capacity)

    def test_public_surface_and_module_dependencies_are_minimal(self) -> None:
        parameters = inspect.signature(
            build_runtime_fractional_delay_schedule
        ).parameters
        self.assertEqual(
            tuple(parameters),
            (
                "capacity",
                "delay_v_sec",
                "delay_omega_sec",
                "integer_snap_tolerance_sec",
                "duration_tolerance_sec",
            ),
        )
        self.assertTrue(
            all(
                parameter.default is inspect.Parameter.empty
                for parameter in parameters.values()
            )
        )
        self.assertEqual(
            [
                name
                for name in RuntimeFractionalDelaySchedule.__dict__
                if name == "sha256"
            ],
            ["sha256"],
        )
        self.assertEqual(
            sum(
                name.startswith("build")
                for name in __import__(
                    "acados.mainline.runtime_schedule", fromlist=["*"]
                ).__all__
            ),
            1,
        )

        tree = ast.parse(MODULE.read_text(encoding="utf-8"))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
        forbidden = (
            "ros",
            "casadi",
            "acados_template",
            "layout",
            "manifest",
            "stage1",
            "contract_source",
        )
        for module in imported:
            self.assertFalse(any(item in module.lower() for item in forbidden), module)
        source = MODULE.read_text(encoding="utf-8")
        self.assertNotIn("ceil(", source)
        self.assertNotIn("build_runtime_schedule", source)


if __name__ == "__main__":
    unittest.main()
