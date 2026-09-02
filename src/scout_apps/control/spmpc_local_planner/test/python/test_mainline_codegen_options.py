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

from acados.mainline.acados_solver_options_identity import (
    ACADOS_D4_SOLVER_OPTION_FIELDS,
    AcadosSolverOptionsIdentityError,
    acados_solver_options_baseline_payload,
    acados_solver_options_baseline_sha256,
)
from acados.mainline.codegen_options import (
    ACADOS_JSON_FILENAME,
    ARTIFACT_IDENTITY_POLICY,
    CODEGEN_ARTIFACT_STATUS,
    CODEGEN_OPTIONS_STATUS,
    CODEGEN_TARGET_PERFORMANCE_STATUS,
    COMPILER_ENVIRONMENT_NAMES,
    COMPILER_ENVIRONMENT_POLICY,
    EXT_FUN_COMPILE_FLAGS,
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
    OUTPUT_DIRECTORY_POLICY,
    RUNTIME_IDENTITY_EXCLUSIONS,
    SOURCE_ROOT_BINDING_POLICY,
    CodegenOptionsError,
    CodegenOptionsSnapshot,
    apply_acados_solver_codegen_options,
    build_codegen_options_snapshot,
    require_codegen_compiler_environment,
    require_codegen_options_snapshot,
    validate_applied_acados_solver_codegen_options,
    validate_codegen_options_document,
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
            tuple(build["compiler_environment"]),
            COMPILER_ENVIRONMENT_NAMES,
        )
        self.assertEqual(
            build["compiler_environment_policy"],
            COMPILER_ENVIRONMENT_POLICY,
        )
        self.assertEqual(
            COMPILER_ENVIRONMENT_NAMES,
            (
                "PATH",
                "CC",
                "CXX",
                "AR",
                "RANLIB",
                "LD",
                "AS",
                "NM",
                "STRIP",
                "CFLAGS",
                "CXXFLAGS",
                "CPPFLAGS",
                "LDFLAGS",
                "MAKEFLAGS",
                "GNUMAKEFLAGS",
                "MFLAGS",
                "MAKEFILES",
                "MAKEOVERRIDES",
                "LIBRARY_PATH",
                "CPATH",
                "C_INCLUDE_PATH",
                "CPLUS_INCLUDE_PATH",
                "COMPILER_PATH",
                "GCC_EXEC_PREFIX",
                "LD_RUN_PATH",
                "PKG_CONFIG_PATH",
                "SOURCE_DATE_EPOCH",
                "ZERO_AR_DATE",
            ),
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
        target = SimpleNamespace(
            ext_fun_compile_flags="",
            ext_fun_expand_constr=True,
            ext_fun_expand_cost=True,
            ext_fun_expand_dyn=True,
            ext_fun_expand_precompute=True,
            custom_update_filename="wrong.c",
            custom_update_header_filename="wrong.h",
            custom_update_copy=True,
            custom_templates=[("wrong.in", "wrong.out")],
        )
        apply_acados_solver_codegen_options(target, self.options)
        self.assertEqual(target.ext_fun_compile_flags, "-O2")
        self.assertIs(target.ext_fun_expand_constr, False)
        self.assertIs(target.ext_fun_expand_cost, False)
        self.assertIs(target.ext_fun_expand_dyn, False)
        self.assertIs(target.ext_fun_expand_precompute, False)
        self.assertEqual(target.custom_update_filename, "")
        self.assertEqual(target.custom_update_header_filename, "")
        self.assertIs(target.custom_update_copy, False)
        self.assertEqual(target.custom_templates, [])
        validate_applied_acados_solver_codegen_options(target, self.options)

        with self.assertRaises(CodegenOptionsError):
            apply_acados_solver_codegen_options(SimpleNamespace(), self.options)
        target.custom_update_copy = True
        with self.assertRaises(CodegenOptionsError):
            validate_applied_acados_solver_codegen_options(target, self.options)

        require_codegen_compiler_environment(self.options)
        original_cflags = dict(self.options.compiler_environment)["CFLAGS"] or ""
        captured_cflags = original_cflags + " -DSPMPC_TEST_CAPTURE=1"
        with patch.dict(os.environ, {"CFLAGS": captured_cflags}):
            with self.assertRaises(CodegenOptionsError):
                require_codegen_compiler_environment(self.options)
            captured = build_codegen_options_snapshot(
                self.capacity,
                self.layout,
                1.0e-12,
                1.0e-12,
                "-O2",
            )
            require_codegen_compiler_environment(captured)
        self.assertEqual(dict(captured.compiler_environment)["CFLAGS"], captured_cflags)
        self.assertNotEqual(captured.semantic_sha256, self.options.semantic_sha256)

    def test_complete_backend_option_identity_excludes_only_d4_fields(self) -> None:
        backend_options = {
            "N_horizon": 60,
            "qp_solver_mu0": 0.0,
            "collocation_type": "GAUSS_LEGENDRE",
            "time_steps": [1.0 / 30.0] * 60,
            **{
                name: (
                    []
                    if name == "custom_templates"
                    else False
                    if name.startswith("ext_fun_expand")
                    or name in {"custom_update_copy"}
                    else "-O2"
                    if name == "ext_fun_compile_flags"
                    else ""
                )
                for name in ACADOS_D4_SOLVER_OPTION_FIELDS
            },
        }
        baseline = acados_solver_options_baseline_sha256(backend_options)
        payload = acados_solver_options_baseline_payload(backend_options)
        self.assertNotIn("qp_solver_mu0", ACADOS_D4_SOLVER_OPTION_FIELDS)
        self.assertEqual(
            payload["excluded_d4_fields"],
            list(ACADOS_D4_SOLVER_OPTION_FIELDS),
        )
        for name in ACADOS_D4_SOLVER_OPTION_FIELDS:
            changed = copy.deepcopy(backend_options)
            changed[name] = (
                [("custom.in", "custom.out")]
                if name == "custom_templates"
                else not changed[name]
                if type(changed[name]) is bool
                else str(changed[name]) + "-changed"
            )
            self.assertEqual(
                acados_solver_options_baseline_sha256(changed),
                baseline,
            )
        for name, value in (
            ("qp_solver_mu0", 1.0),
            ("collocation_type", "GAUSS_RADAU_IIA"),
        ):
            changed = copy.deepcopy(backend_options)
            changed[name] = value
            self.assertNotEqual(
                acados_solver_options_baseline_sha256(changed),
                baseline,
            )
        added = copy.deepcopy(backend_options)
        added["future_backend_option"] = 1
        self.assertNotEqual(
            acados_solver_options_baseline_sha256(added),
            baseline,
        )
        deleted = copy.deepcopy(backend_options)
        deleted.pop("qp_solver_mu0")
        self.assertNotEqual(
            acados_solver_options_baseline_sha256(deleted),
            baseline,
        )
        missing = copy.deepcopy(backend_options)
        missing.pop("custom_templates")
        with self.assertRaises(AcadosSolverOptionsIdentityError):
            acados_solver_options_baseline_sha256(missing)

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
            with self.subTest(
                snap=snap, duration=duration, flags=flags
            ), self.assertRaises(CodegenOptionsError):
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
        structurally_forged = copy.copy(self.options)
        object.__setattr__(structurally_forged, "compiler_environment", ())
        forged_document = structurally_forged.to_dict()
        forged_document.pop("semantic_identity")
        object.__setattr__(
            structurally_forged,
            "semantic_sha256",
            sha256_json(forged_document),
        )
        with self.assertRaises(CodegenOptionsError):
            require_codegen_options_snapshot(structurally_forged)
        self.assertIs(require_codegen_options_snapshot(self.options), self.options)

    def test_serialized_document_parser_rejects_resigned_policy_mutations(self) -> None:
        document = self.options.to_dict()
        layout_sha256 = document["source_identity"][
            "development_layout_semantic_sha256"
        ]
        self.assertIs(
            validate_codegen_options_document(
                document,
                expected_development_layout_sha256=layout_sha256,
            ),
            document,
        )
        forged = copy.deepcopy(document)
        forged["acados_codegen"]["external_functions"]["expand_dynamics"] = True
        payload = {
            key: value for key, value in forged.items() if key != "semantic_identity"
        }
        forged["semantic_identity"]["sha256"] = sha256_json(payload)
        with self.assertRaises(CodegenOptionsError):
            validate_codegen_options_document(forged)

        forged = copy.deepcopy(document)
        environment = forged["acados_codegen"]["build"]["compiler_environment"]
        setting = environment.pop("CC")
        environment["CXX"] = setting
        payload = {
            key: value for key, value in forged.items() if key != "semantic_identity"
        }
        forged["semantic_identity"]["sha256"] = sha256_json(payload)
        with self.assertRaises(CodegenOptionsError):
            validate_codegen_options_document(forged)

        forged = copy.deepcopy(document)
        forged["runtime_schedule_contract"]["duration_tolerance_sec"] = 1.0 / 30.0
        payload = {
            key: value for key, value in forged.items() if key != "semantic_identity"
        }
        forged["semantic_identity"]["sha256"] = sha256_json(payload)
        with self.assertRaises(CodegenOptionsError):
            validate_codegen_options_document(forged)

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
