"""Contract tests for the Stage 3-D4 development codegen policy."""

from __future__ import annotations

import ast
import copy
import json
import math
import os
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.codegen_options import (
    ACADOS_JSON_FILENAME,
    ARTIFACT_IDENTITY_POLICY,
    CODEGEN_ARTIFACT_STATUS,
    CODEGEN_OPTIONS_STATUS,
    CODEGEN_TARGET_PERFORMANCE_STATUS,
    COMPILER_ENVIRONMENT_NAMES,
    EXT_FUN_COMPILE_FLAGS,
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
    OUTPUT_DIRECTORY_POLICY,
    RUNTIME_IDENTITY_EXCLUSIONS,
    SOURCE_ROOT_BINDING_POLICY,
    CodegenOptionsError,
    CodegenOptionsSnapshot,
    apply_codegen_options,
    build_codegen_options_snapshot,
    require_codegen_options_snapshot,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import sha256_json
from acados.mainline.model_contract import MODEL_ID
from acados.mainline.runtime_schedule import RUNTIME_SCHEDULE_SCHEMA_VERSION

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


class MainlineCodegenOptionsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.layout = build_development_layout(self.capacity)
        self.options = build_codegen_options_snapshot(
            self.capacity,
            self.layout,
            1.0e-12,
            1.0e-12,
            "-O2",
        )

    def test_policy_is_explicit_canonical_and_contains_no_runtime_values(self) -> None:
        value = self.options.to_dict()
        json.dumps(value, allow_nan=False)
        identity = value.pop("semantic_identity")
        self.assertEqual(identity["sha256"], sha256_json(value))
        self.assertEqual(identity["sha256"], self.options.semantic_sha256)
        self.assertEqual(value["model_id"], MODEL_ID)
        self.assertEqual(
            value["status"],
            {
                "codegen_options": CODEGEN_OPTIONS_STATUS,
                "artifact": CODEGEN_ARTIFACT_STATUS,
                "target_performance": CODEGEN_TARGET_PERFORMANCE_STATUS,
            },
        )
        self.assertEqual(value["horizon"]["N"], 60)
        self.assertEqual(
            value["horizon"]["release_period_sec"],
            {"numerator": 1, "denominator": 30},
        )
        schedule = value["runtime_schedule_contract"]
        self.assertEqual(schedule["schema_version"], RUNTIME_SCHEDULE_SCHEMA_VERSION)
        self.assertEqual(schedule["integer_snap_tolerance_sec"], 1.0e-12)
        self.assertEqual(schedule["duration_tolerance_sec"], 1.0e-12)
        self.assertEqual(
            value["runtime_identity_exclusions"],
            list(RUNTIME_IDENTITY_EXCLUSIONS),
        )
        identity_payload = dict(value)
        identity_payload.pop("runtime_identity_exclusions")
        text = json.dumps(identity_payload, sort_keys=True)
        for forbidden in (
            '"delay_v_sec"',
            '"delay_omega_sec"',
            '"m_v"',
            '"beta_v"',
            '"condition"',
            '"stage_parameter_matrix"',
            '"runtime_schedule_hash"',
        ):
            self.assertNotIn(forbidden, text)

    def test_codegen_and_output_policies_are_frozen(self) -> None:
        codegen = self.options.to_dict()["acados_codegen"]
        external = codegen["external_functions"]
        build = codegen["build"]
        self.assertEqual(external["compile_flags"], EXT_FUN_COMPILE_FLAGS)
        self.assertEqual(
            external,
            {
                "compile_flags": "-O2",
                "expand_constraints": False,
                "expand_cost": False,
                "expand_dynamics": False,
                "expand_precompute": False,
            },
        )
        self.assertEqual(build["system"], "MAKEFILE")
        self.assertEqual(build["target"], "OCP_SHARED_LIBRARY")
        self.assertIs(build["with_cython"], False)
        self.assertEqual(build["cmake_builder"], "NONE")
        self.assertEqual(
            set(build["compiler_environment"]),
            set(COMPILER_ENVIRONMENT_NAMES),
        )
        self.assertEqual(codegen["output_directory_policy"], OUTPUT_DIRECTORY_POLICY)
        self.assertEqual(
            codegen["source_root_binding_policy"], SOURCE_ROOT_BINDING_POLICY
        )
        self.assertEqual(codegen["artifact_identity_policy"], ARTIFACT_IDENTITY_POLICY)
        self.assertEqual(
            codegen["filenames"],
            {
                "acados_json": ACADOS_JSON_FILENAME,
                "model_contract": MODEL_CONTRACT_FILENAME,
                "generated_header": GENERATED_HEADER_FILENAME,
            },
        )

    def test_backend_mapping_and_compiler_environment_are_explicit(self) -> None:
        target = SimpleNamespace()
        apply_codegen_options(target, self.options)
        self.assertEqual(target.ext_fun_compile_flags, "-O2")
        self.assertIs(target.ext_fun_expand_constr, False)
        self.assertIs(target.ext_fun_expand_cost, False)
        self.assertIs(target.ext_fun_expand_dyn, False)
        self.assertIs(target.ext_fun_expand_precompute, False)
        self.assertEqual(target.custom_update_filename, "")
        self.assertEqual(target.custom_update_header_filename, "")
        self.assertIs(target.custom_update_copy, False)
        self.assertEqual(target.custom_templates, [])

        original_cflags = dict(self.options.compiler_environment)["CFLAGS"]
        captured_cflags = original_cflags + " -DSPMPC_TEST_CAPTURE=1"
        with patch.dict(os.environ, {"CFLAGS": captured_cflags}):
            captured = build_codegen_options_snapshot(
                self.capacity,
                self.layout,
                1.0e-12,
                1.0e-12,
                "-O2",
            )
        self.assertEqual(dict(captured.compiler_environment)["CFLAGS"], captured_cflags)
        self.assertNotEqual(captured.semantic_sha256, self.options.semantic_sha256)

    def test_tolerances_and_compile_flags_have_no_hidden_defaults(self) -> None:
        bad_values = (
            (0, 1.0e-12, "-O2"),
            (1.0e-12, 0, "-O2"),
            (-1.0, 1.0e-12, "-O2"),
            (math.nan, 1.0e-12, "-O2"),
            (1.0 / 60.0, 1.0e-12, "-O2"),
            (1.0e-12, 1.0 / 30.0, "-O2"),
            (1.0e-12, 1.0e-12, ""),
            (1.0e-12, 1.0e-12, "-O3"),
        )
        for snap, duration, flags in bad_values:
            with (
                self.subTest(snap=snap, duration=duration, flags=flags),
                self.assertRaises(CodegenOptionsError),
            ):
                build_codegen_options_snapshot(
                    self.capacity,
                    self.layout,
                    snap,  # type: ignore[arg-type]
                    duration,  # type: ignore[arg-type]
                    flags,
                )
        with self.assertRaises(TypeError):
            build_codegen_options_snapshot(self.capacity)  # type: ignore[call-arg]

    def test_snapshot_requires_builder_is_frozen_and_detects_force_mutation(
        self,
    ) -> None:
        with self.assertRaises(TypeError):
            CodegenOptionsSnapshot()  # type: ignore[call-arg]
        with self.assertRaises(FrozenInstanceError):
            self.options.duration_tolerance_sec = 0.0  # type: ignore[misc]
        forged = copy.copy(self.options)
        object.__setattr__(forged, "duration_tolerance_sec", 2.0e-12)
        with self.assertRaises(CodegenOptionsError):
            require_codegen_options_snapshot(forged)
        self.assertIs(require_codegen_options_snapshot(self.options), self.options)

    def test_module_has_no_backend_evidence_or_legacy_dependency(self) -> None:
        source = SCRIPTS_ROOT / "acados" / "mainline" / "codegen_options.py"
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)
        self.assertFalse(
            any(
                name in {"casadi", "numpy", "acados_template"}
                or "legacy" in name
                or name.endswith(("stage1_evidence", "contract_source", "manifest"))
                for name in imported
            )
        )


if __name__ == "__main__":
    unittest.main()
