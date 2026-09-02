#!/usr/bin/env python3
"""Contract tests for the independent Stage 3-D development capacity."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import FrozenInstanceError, replace
from fractions import Fraction
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline import development_capacity
from acados.mainline.development_capacity import (
    DEVELOPMENT_ARTIFACT_CLASS,
    DEVELOPMENT_CAPACITY_SHA256,
    DEVELOPMENT_CAPACITY_STATUS,
    DevelopmentCapacityError,
    load_development_capacity,
)

CONTRACT = (
    PACKAGE_ROOT
    / "config"
    / "mainline"
    / "contracts"
    / "development_capacity_v1.json"
)
MODULE = SCRIPTS_ROOT / "acados" / "mainline" / "development_capacity.py"


class MainlineDevelopmentCapacityTest(unittest.TestCase):
    def setUp(self) -> None:
        self.document = json.loads(CONTRACT.read_text(encoding="utf-8"))

    def validate(self, value: dict) -> None:
        development_capacity._validate_development_capacity(value)

    def changed(self, path: tuple[str, ...], value: object) -> dict:
        result = copy.deepcopy(self.document)
        target = result
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        return result

    def test_pinned_snapshot_exposes_integer_authority_and_exact_seconds(self) -> None:
        contract = load_development_capacity(CONTRACT)
        self.assertEqual(
            hashlib.sha256(CONTRACT.read_bytes()).hexdigest(),
            DEVELOPMENT_CAPACITY_SHA256,
        )
        self.assertEqual(contract.contract_sha256, DEVELOPMENT_CAPACITY_SHA256)
        self.assertEqual(contract.release_frequency_hz, 30)
        self.assertEqual(contract.release_period_sec, Fraction(1, 30))
        self.assertEqual((contract.R_v, contract.R_omega), (12, 24))
        self.assertEqual((contract.D_v, contract.D_omega), (11, 23))
        self.assertEqual((contract.NQ_v, contract.NQ_omega), (13, 25))
        self.assertEqual(contract.v.l_max_sec, Fraction(2, 5))
        self.assertEqual(contract.omega.l_max_sec, Fraction(4, 5))
        self.assertEqual((contract.NX, contract.NU, contract.NP_exec), (48, 3, 121))

        serialized = contract.to_dict()
        self.assertEqual(serialized, self.document)
        self.assertEqual(serialized["status"], DEVELOPMENT_CAPACITY_STATUS)
        self.assertEqual(serialized["artifact_class"], DEVELOPMENT_ARTIFACT_CLASS)
        self.assertEqual(
            serialized["capacity"]["v"]["derived"]["L_max_sec"],
            {"numerator": 12, "denominator": 30},
        )
        self.assertEqual(
            serialized["capacity"]["omega"]["derived"]["L_max_sec"],
            {"numerator": 24, "denominator": 30},
        )

    def test_dimensions_are_recomputed_from_release_intervals(self) -> None:
        dimensions = self.document["derived_dimensions"]
        capacity = self.document["capacity"]
        d_v = capacity["v"]["derived"]["D"]
        d_omega = capacity["omega"]["derived"]["D"]
        nq_v = capacity["v"]["derived"]["NQ"]
        nq_omega = capacity["omega"]["derived"]["NQ"]
        basis = self.document["layout_basis"]
        self.assertEqual(dimensions["NX"], basis["base_state_count"] + d_v + d_omega)
        self.assertEqual(dimensions["NU"], basis["control_count"])
        self.assertEqual(
            dimensions["NP_exec"],
            basis["execution_fixed_scalar_count"]
            + basis["execution_subsegment_slots"] * (nq_v + nq_omega),
        )
        self.validate(self.document)

    def test_unknown_fields_are_rejected_at_every_contract_layer(self) -> None:
        mutations = []
        top = copy.deepcopy(self.document)
        top["production_gate"] = True
        mutations.append(top)
        nested = copy.deepcopy(self.document)
        nested["capacity"]["v"]["derived"]["rounded_L_max_sec"] = 0.4
        mutations.append(nested)
        source = copy.deepcopy(self.document)
        source["source_identity"]["bag_path"] = "/tmp/not-authority.bag"
        mutations.append(source)
        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                DevelopmentCapacityError
            ):
                self.validate(mutated)

    def test_boolean_and_float_integer_aliases_are_rejected(self) -> None:
        mutations = (
            self.changed(("release_grid", "frequency_hz"), True),
            self.changed(("capacity", "v", "release_intervals"), True),
            self.changed(("capacity", "omega", "release_intervals"), 24.0),
            self.changed(
                ("capacity", "v", "derived", "L_max_sec", "numerator"),
                True,
            ),
            self.changed(("derived_dimensions", "NX"), True),
        )
        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                DevelopmentCapacityError
            ):
                self.validate(mutated)

    def test_nonpositive_or_untyped_capacity_values_are_rejected(self) -> None:
        mutations = (
            self.changed(("release_grid", "frequency_hz"), 0),
            self.changed(("capacity", "v", "release_intervals"), 0),
            self.changed(("capacity", "omega", "release_intervals"), -24),
            self.changed(("capacity", "v", "release_intervals"), "12"),
            self.changed(("capacity", "v", "derived", "D"), -1),
            self.changed(("capacity", "omega", "derived", "NQ"), 0),
            self.changed(("derived_dimensions", "NP_exec"), -1),
        )
        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                DevelopmentCapacityError
            ):
                self.validate(mutated)

    def test_every_derived_count_and_dimension_fails_closed_on_drift(self) -> None:
        mutations = (
            self.changed(("capacity", "v", "derived", "D"), 12),
            self.changed(("capacity", "omega", "derived", "D"), 22),
            self.changed(("capacity", "v", "derived", "NQ"), 12),
            self.changed(("capacity", "omega", "derived", "NQ"), 26),
            self.changed(("derived_dimensions", "NX"), 49),
            self.changed(("derived_dimensions", "NU"), 4),
            self.changed(("derived_dimensions", "NP_exec"), 120),
        )
        for index, mutated in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(
                DevelopmentCapacityError
            ):
                self.validate(mutated)

    def test_lmax_must_be_direct_exact_r_over_frequency_rational(self) -> None:
        reduced = self.changed(
            ("capacity", "v", "derived", "L_max_sec"),
            {"numerator": 2, "denominator": 5},
        )
        decimal = self.changed(
            ("capacity", "omega", "derived", "L_max_sec"),
            0.8,
        )
        wrong_period = self.changed(
            ("release_grid", "period_sec", "denominator"),
            29,
        )
        for mutated in (reduced, decimal, wrong_period):
            with self.assertRaises(DevelopmentCapacityError):
                self.validate(mutated)

    def test_duplicate_keys_and_reencoded_snapshot_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            duplicate = root / "duplicate.json"
            duplicate.write_text(
                '{"schema_version":"first","schema_version":"second"}',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(DevelopmentCapacityError, "duplicate"):
                load_development_capacity(duplicate)

            reencoded = root / "reencoded.json"
            reencoded.write_text(
                json.dumps(self.document, ensure_ascii=False, sort_keys=True),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(DevelopmentCapacityError, "pinned immutable"):
                load_development_capacity(reencoded)

    def test_loader_has_no_external_evidence_or_production_gate_dependency(
        self,
    ) -> None:
        tree = ast.parse(MODULE.read_text(encoding="utf-8"))
        imported_modules = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module)
        forbidden = ("stage1", "contract_source", "rosbag")
        for imported in imported_modules:
            self.assertFalse(
                any(name in imported for name in forbidden),
                msg=f"forbidden dependency imported: {imported}",
            )

        flattened = json.dumps(self.document, sort_keys=True)
        self.assertNotIn("bag_path", flattened)
        self.assertNotIn("git_clean", flattened)
        self.assertNotIn("production_gate", flattened)
        with tempfile.TemporaryDirectory() as temporary:
            copied = Path(temporary) / "capacity.json"
            copied.write_bytes(CONTRACT.read_bytes())
            self.assertEqual(load_development_capacity(copied).NX, 48)

    def test_isolated_import_does_not_eager_load_legacy_authority_modules(
        self,
    ) -> None:
        program = f"""
import sys
sys.path.insert(0, {str(SCRIPTS_ROOT)!r})
import acados.mainline.development_capacity
forbidden = (
    'acados.mainline.contract_source',
    'acados.mainline.layout',
    'acados.mainline.manifest',
    'acados.mainline.stage1_evidence',
)
loaded = [name for name in forbidden if name in sys.modules]
if loaded:
    raise SystemExit('unexpected eager imports: ' + ','.join(loaded))
"""
        result = subprocess.run(
            [sys.executable, "-c", program],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr or result.stdout)

    def test_reference_is_immutable_and_cannot_be_relabelled(self) -> None:
        contract = load_development_capacity(CONTRACT)
        with self.assertRaises(FrozenInstanceError):
            contract.nx = 49  # type: ignore[misc]
        with self.assertRaisesRegex(DevelopmentCapacityError, "pinned loader"):
            replace(contract, nx=49)


if __name__ == "__main__":
    unittest.main()
