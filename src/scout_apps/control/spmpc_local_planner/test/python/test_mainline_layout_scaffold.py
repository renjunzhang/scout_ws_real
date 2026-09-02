#!/usr/bin/env python3
"""Contract tests for the non-generating Stage 3A layout scaffold."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import sys
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.contract_source import (
    STAGE0_CONTRACT_SHA256,
    STAGE1_BLOCKED_STATUS,
    STAGE3_PROHIBITED_STATUS,
    ContractSourceError,
    Stage0ContractReference,
    load_stage0_contract_reference,
)
from acados.mainline.layout import (
    DEFAULT_DT_SEC,
    MAX_RETAINED_COMMANDS_PER_CHANNEL,
    LayoutError,
    LayoutPurpose,
    LayoutSpec,
    build_layout,
    build_synthetic_layout,
    layout_from_dict,
)
from acados.mainline.manifest import (
    MANIFEST_STATUS,
    ManifestError,
    build_scaffold_manifest,
    canonical_json,
    scaffold_identity_hash,
    sha256_json,
    validate_scaffold_manifest,
)

STAGE0_CONTRACT = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "stage0_contract_v1.json"
)
SCENARIO_ID = "layout_formula_exact_and_fractional"


class MainlineLayoutScaffoldTest(unittest.TestCase):
    def setUp(self) -> None:
        # v is exactly three grid intervals; omega exercises ceil(4.2).
        self.layout = build_synthetic_layout(
            0.1,
            0.14,
            scenario_id=SCENARIO_ID,
        )

    def test_dimension_formulas_and_scope_are_explicit(self) -> None:
        layout = self.layout
        self.assertEqual((layout.r_v, layout.d_v, layout.nq_v), (3, 2, 4))
        self.assertEqual((layout.r_omega, layout.d_omega, layout.nq_omega), (5, 4, 6))
        self.assertEqual(layout.nx, 14 + layout.d_v + layout.d_omega)
        self.assertEqual((layout.nx, layout.nu, layout.np_exec), (20, 3, 37))
        self.assertEqual((layout.NX, layout.NU, layout.NP_exec), (20, 3, 37))

        document = layout.to_dict()
        self.assertEqual(
            document["scope"],
            "STATE_CONTROL_AND_EXECUTION_PARAMETER_PREFIX_ONLY",
        )
        self.assertNotIn("NP", document["dimensions"])
        self.assertIn("full_parameter_layout", document["missing_before_artifact"])
        self.assertFalse(document["authority"]["artifact_generation_allowed"])

    def test_zero_and_integer_boundary_dimensions_do_not_shift(self) -> None:
        zero = build_synthetic_layout(0.0, 0.0, scenario_id="zero_delay_bound")
        self.assertEqual((zero.d_v, zero.nq_v, zero.nx, zero.np_exec), (0, 1, 14, 13))

        boundary = 3.0 * DEFAULT_DT_SEC
        below = DEFAULT_DT_SEC * (3.0 - 1e-12)
        above = DEFAULT_DT_SEC * (3.0 + 1e-12)
        layouts = [
            build_synthetic_layout(value, 0.0, scenario_id=f"boundary_{index}")
            for index, value in enumerate((below, boundary, above))
        ]
        self.assertEqual([layout.r_v for layout in layouts], [3, 3, 4])

    def test_state_control_names_and_offsets_are_complete(self) -> None:
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
            "older_v[0]",
            "older_v[1]",
            "older_omega[0]",
            "older_omega[1]",
            "older_omega[2]",
            "older_omega[3]",
            "eta_x",
            "eta_x_dot",
            "eta_y",
            "eta_y_dot",
        ]
        self.assertEqual(list(self.layout.state_names), expected_state)
        self.assertEqual(
            list(self.layout.control_names),
            ["j_issue_v", "j_issue_omega", "v_s"],
        )
        self.assertEqual(
            [self.layout.state_offsets[name] for name in self.layout.state_names],
            list(range(self.layout.nx)),
        )
        self.assertEqual(
            [self.layout.control_offsets[name] for name in self.layout.control_names],
            list(range(self.layout.nu)),
        )
        self.assertEqual(self.layout.state_offsets["q_prev_v"], 6)
        self.assertEqual(self.layout.state_offsets["eta_x"], 16)

    def test_execution_parameter_prefix_is_contiguous_and_typed(self) -> None:
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
        self.assertEqual(len(names), self.layout.np_exec)
        self.assertEqual(names[7], "act_sel_v[0][0]")
        self.assertEqual(names[7 + 3 * self.layout.nq_v], "act_sel_omega[0][0]")
        self.assertEqual(
            [self.layout.execution_parameter_offsets[name] for name in names],
            list(range(self.layout.np_exec)),
        )
        for channel, width in (
            ("v", self.layout.nq_v),
            ("omega", self.layout.nq_omega),
        ):
            slots = self.layout.selector_offsets[channel]
            self.assertEqual(len(slots), 3)
            self.assertTrue(all(len(slot) == width for slot in slots))

    def test_single_purpose_has_no_boolean_production_escape(self) -> None:
        fields = LayoutSpec.__dataclass_fields__
        self.assertNotIn("lmax_frozen", fields)
        self.assertNotIn("synthetic", fields)
        with self.assertRaisesRegex(LayoutError, "only STAGE3A_SYNTHETIC"):
            build_layout(
                LayoutSpec(
                    purpose="STAGE3A_SYNTHETIC_NO_ARTIFACT",  # type: ignore[arg-type]
                    scenario_id=SCENARIO_ID,
                    l_max_v_sec=0.1,
                    l_max_omega_sec=0.14,
                )
            )
        with self.assertRaises(LayoutError):
            build_synthetic_layout(0.1, 0.14, scenario_id="Production Layout")
        self.assertIs(self.layout.purpose, LayoutPurpose.STAGE3A_SYNTHETIC)

    def test_invalid_numbers_grid_horizon_and_resource_guard_fail_closed(self) -> None:
        for value in (
            -1.0,
            math.nan,
            math.inf,
            -math.inf,
            True,
            "0.1",
            10**10000,
        ):
            with self.subTest(value=value), self.assertRaises(LayoutError):
                build_synthetic_layout(value, 0.14, scenario_id="bad_lmax")
        with self.assertRaises(LayoutError):
            build_synthetic_layout(0.1, math.nan, scenario_id="bad_omega")
        with self.assertRaises(LayoutError):
            build_synthetic_layout(
                0.1,
                0.14,
                scenario_id="bad_grid",
                dt_sec=0.0333,
            )
        with self.assertRaises(LayoutError):
            build_synthetic_layout(
                0.1,
                0.14,
                scenario_id="bad_horizon",
                horizon_steps=59,
            )
        too_many = (MAX_RETAINED_COMMANDS_PER_CHANNEL + 0.5) * DEFAULT_DT_SEC
        with self.assertRaisesRegex(LayoutError, "scaffold guard"):
            build_synthetic_layout(too_many, 0.14, scenario_id="guard_overflow")
        self.assertEqual(
            self.layout.to_dict()["delay_layout"]["generation_resource_guard"][
                "max_retained_commands_per_channel"
            ],
            MAX_RETAINED_COMMANDS_PER_CHANNEL,
        )

    def test_builder_output_and_nested_indices_are_immutable(self) -> None:
        with self.assertRaises(TypeError):
            self.layout.state_offsets["px"] = 99  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.selector_offsets["v"] = ()  # type: ignore[index]
        with self.assertRaises(TypeError):
            self.layout.selector_offsets["v"][0][0] = 99  # type: ignore[index]
        with self.assertRaises(FrozenInstanceError):
            self.layout.nx = 99  # type: ignore[misc]
        rebuilt = layout_from_dict(self.layout.to_dict())
        self.assertEqual(rebuilt.to_dict(), self.layout.to_dict())

        numeric_alias = build_synthetic_layout(
            0.0, 0.0, scenario_id="strict_numeric_identity"
        ).to_dict()
        numeric_alias["dimensions"]["NX"] = 14.0
        with self.assertRaisesRegex(LayoutError, "formulae"):
            layout_from_dict(numeric_alias)


class MainlineScaffoldManifestTest(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = build_synthetic_layout(
            0.1,
            0.14,
            scenario_id=SCENARIO_ID,
        )
        self.source = load_stage0_contract_reference(STAGE0_CONTRACT)
        self.manifest = build_scaffold_manifest(self.layout, STAGE0_CONTRACT)

    def test_stage0_reference_is_derived_from_exact_source_bytes(self) -> None:
        expected_sha = hashlib.sha256(STAGE0_CONTRACT.read_bytes()).hexdigest()
        self.assertEqual(expected_sha, STAGE0_CONTRACT_SHA256)
        self.assertEqual(self.source.contract_sha256, expected_sha)
        self.assertEqual(self.source.stage1_gate, STAGE1_BLOCKED_STATUS)
        self.assertEqual(self.source.stage3_gate, STAGE3_PROHIBITED_STATUS)
        self.assertEqual(self.source.lmax_authority, "UNFROZEN_IN_STAGE0")

    def test_stage0_loader_rejects_gate_or_unfrozen_field_drift(self) -> None:
        original = json.loads(STAGE0_CONTRACT.read_text(encoding="utf-8"))
        cases = []
        stage1_open = copy.deepcopy(original)
        stage1_open["stage_status"]["stage1"] = "PASS"
        cases.append(stage1_open)
        stage3_open = copy.deepcopy(original)
        stage3_open["stage_status"]["stage3"] = "READY"
        cases.append(stage3_open)
        missing_lmax = copy.deepcopy(original)
        missing_lmax["unfrozen_parameters"].remove("execution_model.L_max_v_sec")
        cases.append(missing_lmax)
        with tempfile.TemporaryDirectory() as directory:
            for index, value in enumerate(cases):
                path = Path(directory) / f"drift_{index}.json"
                path.write_text(json.dumps(value), encoding="utf-8")
                with self.subTest(index=index), self.assertRaises(ContractSourceError):
                    load_stage0_contract_reference(path)

            duplicate = Path(directory) / "duplicate.json"
            duplicate.write_text(
                '{"contract_id":"first","contract_id":"second"}',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ContractSourceError, "duplicate"):
                load_stage0_contract_reference(duplicate)

    def test_hand_built_or_reencoded_stage0_reference_is_rejected(self) -> None:
        forged = Stage0ContractReference(
            schema_version=self.source.schema_version,
            contract_id=self.source.contract_id,
            contract_sha256="a" * 64,
            status=self.source.status,
            stage1_gate=self.source.stage1_gate,
            stage3_gate=self.source.stage3_gate,
            lmax_authority=self.source.lmax_authority,
            dataset_gate_authority=self.source.dataset_gate_authority,
        )
        with self.assertRaisesRegex(ManifestError, "path must be"):
            build_scaffold_manifest(self.layout, forged)

        with tempfile.TemporaryDirectory() as directory:
            reencoded = Path(directory) / "reencoded_stage0.json"
            reencoded.write_text(
                json.dumps(
                    json.loads(STAGE0_CONTRACT.read_text(encoding="utf-8")),
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ContractSourceError, "pinned immutable"):
                load_stage0_contract_reference(reencoded)

    def test_manifest_is_layout_only_and_has_no_generated_state(self) -> None:
        validate_scaffold_manifest(self.manifest)
        self.assertEqual(self.manifest["status"], MANIFEST_STATUS)
        self.assertFalse(self.manifest["artifact_generation"]["allowed"])
        self.assertEqual(
            self.manifest["identity_sha256"],
            sha256_json(self.manifest["identity"]),
        )
        self.assertEqual(
            scaffold_identity_hash(self.manifest),
            self.manifest["identity_sha256"],
        )
        self.assertNotIn("artifact_sha256", self.manifest)
        self.assertNotIn("generation_command", self.manifest)

    def test_manifest_tampering_and_artifact_claims_are_rejected(self) -> None:
        tampered_layout = copy.deepcopy(self.manifest)
        tampered_layout["identity"]["layout_scaffold"]["dimensions"]["NX"] += 1
        tampered_layout["identity_sha256"] = sha256_json(tampered_layout["identity"])
        with self.assertRaisesRegex(ManifestError, "formulae"):
            validate_scaffold_manifest(tampered_layout)

        tampered_source = copy.deepcopy(self.manifest)
        tampered_source["identity"]["source_contract"]["stage1_gate"] = "PASS"
        tampered_source["identity_sha256"] = sha256_json(tampered_source["identity"])
        with self.assertRaisesRegex(ManifestError, "source_contract"):
            validate_scaffold_manifest(tampered_source)

        bad_hash = copy.deepcopy(self.manifest)
        bad_hash["identity_sha256"] = "b" * 64
        with self.assertRaisesRegex(ManifestError, "identity_sha256"):
            validate_scaffold_manifest(bad_hash)

        artifact_claim = copy.deepcopy(self.manifest)
        artifact_claim["artifact_sha256"] = "c" * 64
        with self.assertRaisesRegex(ManifestError, "keys"):
            validate_scaffold_manifest(artifact_claim)

        boolean_alias = copy.deepcopy(self.manifest)
        boolean_alias["artifact_generation"]["allowed"] = 0
        with self.assertRaisesRegex(ManifestError, "cannot allow"):
            validate_scaffold_manifest(boolean_alias)

    def test_manifest_builder_rejects_untyped_inputs(self) -> None:
        with self.assertRaises(ManifestError):
            build_scaffold_manifest(self.layout.to_dict(), STAGE0_CONTRACT)  # type: ignore[arg-type]
        with self.assertRaises(ManifestError):
            build_scaffold_manifest(self.layout, self.source.to_dict())  # type: ignore[arg-type]

    def test_canonical_encoding_rejects_nonfinite_and_normalizes_zero(self) -> None:
        self.assertEqual(canonical_json({"x": -0.0}), '{"x":0.0}')
        with self.assertRaises(ManifestError):
            canonical_json({"x": math.inf})
        with self.assertRaises(ManifestError):
            canonical_json({1: "non-string-key"})


if __name__ == "__main__":
    unittest.main()
