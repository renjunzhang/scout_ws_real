#!/usr/bin/env python3
"""Contract tests for the Stage 3-D development prefix layout."""

from __future__ import annotations

import ast
import copy
import inspect
import json
import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError, replace
from fractions import Fraction
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.development_capacity import (
    ChannelCapacity,
    load_development_capacity,
)
from acados.mainline.development_layout import (
    DEVELOPMENT_LAYOUT_SCOPE,
    DevelopmentLayoutError,
    build_development_layout,
    development_layout_from_dict,
    validate_development_layout_snapshot,
)

CAPACITY = (
    PACKAGE_ROOT
    / "config"
    / "mainline"
    / "contracts"
    / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "development_layout.py"


class MainlineDevelopmentLayoutTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.layout = build_development_layout(self.capacity)
        self.document = self.layout.to_dict()

    def changed(self, path: tuple[object, ...], value: object) -> dict:
        result = copy.deepcopy(self.document)
        target = result
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        return result

    def validate(self, value: dict) -> None:
        development_layout_from_dict(value, self.capacity)

    def test_capacity_identity_horizon_and_dimensions_are_exact(self) -> None:
        layout = self.layout
        self.assertEqual((layout.r_v, layout.r_omega), (12, 24))
        self.assertEqual((layout.d_v, layout.d_omega), (11, 23))
        self.assertEqual((layout.nq_v, layout.nq_omega), (13, 25))
        self.assertEqual((layout.NX, layout.NU, layout.NP_exec), (48, 3, 121))
        self.assertEqual(layout.horizon_steps, 60)

        document = self.document
        self.assertEqual(document["scope"], DEVELOPMENT_LAYOUT_SCOPE)
        self.assertEqual(
            document["capacity_identity"]["raw_bytes_sha256"],
            self.capacity.contract_sha256,
        )
        self.assertEqual(
            document["capacity_identity"]["v"],
            {"R": 12, "D": 11, "NQ": 13},
        )
        self.assertEqual(
            document["capacity_identity"]["omega"],
            {"R": 24, "D": 23, "NQ": 25},
        )
        self.assertEqual(
            document["horizon"],
            {"N": 60, "period_sec": {"numerator": 1, "denominator": 30}},
        )
        self.assertEqual(
            document["dimensions"],
            {"NX": 48, "NU": 3, "NP_exec": 121},
        )
        self.assertNotIn("NP", document["dimensions"])

    def test_state_and_control_order_and_offsets_are_complete(self) -> None:
        expected_state = [
            "px",
            "py",
            "theta",
            "s",
            "v_actual",
            "omega_actual",
            "q_prev_v",
            "q_prev_omega",
            "a_prev",
            "alpha_prev",
        ]
        expected_state.extend(f"older_v[{index}]" for index in range(11))
        expected_state.extend(f"older_omega[{index}]" for index in range(23))
        expected_state.extend(("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"))
        self.assertEqual(list(self.layout.state_names), expected_state)
        self.assertEqual(
            [self.layout.state_offsets[name] for name in expected_state],
            list(range(48)),
        )
        self.assertEqual(self.layout.state_offsets["older_v[0]"], 10)
        self.assertEqual(self.layout.state_offsets["older_omega[0]"], 21)
        self.assertEqual(self.layout.state_offsets["eta_x"], 44)

        self.assertEqual(
            self.layout.control_names,
            ("j_issue_v", "j_issue_omega", "v_s"),
        )
        self.assertEqual(
            [
                self.layout.control_offsets[name]
                for name in self.layout.control_names
            ],
            [0, 1, 2],
        )

    def test_three_slot_execution_prefix_is_complete_and_contiguous(self) -> None:
        names = self.layout.execution_parameter_names
        self.assertEqual(
            names[:7],
            (
                "act_inv_tau_v",
                "act_gain_v",
                "act_inv_tau_omega",
                "act_gain_omega",
                "act_seg_dt[0]",
                "act_seg_dt[1]",
                "act_seg_dt[2]",
            ),
        )
        self.assertEqual(len(names), 121)
        self.assertEqual(
            [self.layout.execution_parameter_offsets[name] for name in names],
            list(range(121)),
        )
        self.assertEqual(names[7], "act_sel_v[0][0]")
        self.assertEqual(names[46], "act_sel_omega[0][0]")
        self.assertEqual(names[-1], "act_sel_omega[2][24]")

        all_selector_offsets = []
        for channel, width in (("v", 13), ("omega", 25)):
            slots = self.layout.selector_offsets[channel]
            self.assertEqual(len(slots), 3)
            self.assertTrue(all(len(slot) == width for slot in slots))
            all_selector_offsets.extend(
                offset for slot in slots for offset in slot
            )
        self.assertEqual(all_selector_offsets, list(range(7, 121)))

    def test_typed_round_trip_returns_the_builder_result(self) -> None:
        rebuilt = development_layout_from_dict(self.document, self.capacity)
        self.assertEqual(rebuilt, self.layout)
        self.assertEqual(rebuilt.to_dict(), self.document)
        self.assertEqual(
            validate_development_layout_snapshot(self.layout, self.capacity),
            self.layout,
        )

    def test_unknown_fields_fail_closed_at_top_and_nested_levels(self) -> None:
        top = copy.deepcopy(self.document)
        top["full_parameter_layout"] = []
        nested = copy.deepcopy(self.document)
        nested["capacity_identity"]["production_gate"] = False
        selector = copy.deepcopy(self.document)
        selector["execution_parameter_layout"]["selector_offsets"]["extra"] = []
        for value in (top, nested, selector):
            with self.assertRaises(DevelopmentLayoutError):
                self.validate(value)

    def test_type_order_offset_dimension_and_capacity_drift_are_rejected(self) -> None:
        swapped_state = copy.deepcopy(self.document)
        ordered = swapped_state["state_layout"]["ordered"]
        ordered[0], ordered[1] = ordered[1], ordered[0]

        swapped_parameter = copy.deepcopy(self.document)
        ordered_parameters = swapped_parameter["execution_parameter_layout"][
            "ordered"
        ]
        ordered_parameters[7], ordered_parameters[8] = (
            ordered_parameters[8],
            ordered_parameters[7],
        )
        mutations = (
            self.changed(("horizon", "N"), 60.0),
            self.changed(("dimensions", "NX"), True),
            self.changed(("dimensions", "NP_exec"), 120),
            self.changed(("state_layout", "offsets", "px"), 1),
            self.changed(("control_layout", "offsets", "v_s"), 1),
            self.changed(
                ("execution_parameter_layout", "selector_offsets", "v", 0, 0),
                8,
            ),
            self.changed(
                ("capacity_identity", "raw_bytes_sha256"),
                "0" * 64,
            ),
            self.changed(("capacity_identity", "v", "R"), 13),
            swapped_state,
            swapped_parameter,
        )
        for index, value in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                DevelopmentLayoutError
            ):
                self.validate(value)

    def test_nested_offsets_and_layout_are_immutable(self) -> None:
        with self.assertRaises(TypeError):
            self.layout.state_offsets["px"] = 1  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.execution_parameter_offsets[
                "act_gain_v"
            ] = 0  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.selector_offsets["v"] = ()  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.selector_offsets["v"][0][0] = 8  # type: ignore[index]
        with self.assertRaises(FrozenInstanceError):
            self.layout.nx = 49  # type: ignore[misc]
        with self.assertRaisesRegex(DevelopmentLayoutError, "requires"):
            replace(self.layout, nx=49)

    def test_builder_and_validator_accept_only_typed_pinned_capacity(self) -> None:
        with self.assertRaises(DevelopmentLayoutError):
            build_development_layout(self.capacity.to_dict())  # type: ignore[arg-type]
        with self.assertRaises(DevelopmentLayoutError):
            development_layout_from_dict(
                self.document,
                self.capacity.to_dict(),  # type: ignore[arg-type]
            )
        with self.assertRaises(TypeError):
            build_development_layout(self.capacity, 0.4)  # type: ignore[call-arg]
        self.assertEqual(
            tuple(inspect.signature(build_development_layout).parameters),
            ("capacity",),
        )

        forged = copy.copy(self.layout)
        object.__setattr__(forged, "nx", 49)
        with self.assertRaisesRegex(DevelopmentLayoutError, "typed layout snapshot"):
            validate_development_layout_snapshot(forged, self.capacity)

        forged = copy.copy(self.capacity)
        object.__setattr__(
            forged,
            "v",
            ChannelCapacity(1, 0, 2, Fraction(1, 30)),
        )
        object.__setattr__(forged, "nx", 37)
        object.__setattr__(forged, "np_exec", 88)
        self.assertEqual(forged.contract_sha256, self.capacity.contract_sha256)
        with self.assertRaisesRegex(DevelopmentLayoutError, "complete pinned"):
            build_development_layout(forged)

        numeric_alias = copy.copy(self.capacity)
        object.__setattr__(numeric_alias, "nx", 48.0)
        with self.assertRaisesRegex(DevelopmentLayoutError, "complete pinned"):
            build_development_layout(numeric_alias)

    def test_module_is_isolated_from_the_legacy_stage3a_path(self) -> None:
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
        forbidden_imports = {
            "layout",
            "manifest",
            "contract_source",
            "stage1_evidence",
            "rosbag",
        }
        for imported in imports:
            self.assertFalse(
                any(
                    imported == forbidden
                    or imported.endswith(f".{forbidden}")
                    for forbidden in forbidden_imports
                ),
                msg=f"forbidden dependency imported: {imported}",
            )
        self.assertEqual(builders, ["build_development_layout"])
        self.assertNotIn("build_" + "synthetic_layout", source)

        serialized = json.dumps(self.document, sort_keys=True).lower()
        self.assertNotIn("full_parameter_layout", serialized)
        self.assertNotIn("artifact", serialized)
        self.assertNotIn("generated", serialized)
        self.assertNotIn("production_gate", serialized)

    def test_import_does_not_eager_load_old_contract_or_layout_modules(self) -> None:
        probe = "\n".join(
            (
                "import json",
                "import sys",
                f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r})",
                "import acados.mainline.development_layout",
                "forbidden = [",
                "    'acados.mainline.contract_source',",
                "    'acados.mainline.layout',",
                "    'acados.mainline.manifest',",
                "    'acados.mainline.stage1_evidence',",
                "    'rosbag',",
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


if __name__ == "__main__":
    unittest.main()
