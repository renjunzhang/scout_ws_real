"""Focused tests for the D4 artifact identity and model-contract boundary."""

from __future__ import annotations

import ast
import copy
import hashlib
import importlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
INCLUDE_ROOT = PACKAGE_ROOT / "include"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.acados_ocp_schema import (
    ACADOS_INTERFACE_SOURCE_PATHS,
    acados_interface_source_sha256_from_inventory,
)
from acados.mainline.artifact_files import (
    ACADOS_JSON_ROLE,
    GENERATED_C_HEADER_ROLE,
    GENERATED_C_SOURCE_ROLE,
    SOLVER_LIBRARY_BASENAMES,
    generated_file_records_from_dict,
    generated_tree_sha256,
    inventory_generated_tree,
    validate_generated_tree,
)
from acados.mainline.codegen_options import (
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
)
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import canonical_json, sha256_json
from acados.mainline.model_contract import COST_SCHEMA, DISCRETIZATION_SCHEMA, MODEL_ID
from acados.mainline.provenance_acados import (
    ACADOS_COMMIT_MARKER_LOGICAL_NAME,
    ACADOS_IDENTITY_SCHEMA,
    ACADOS_INCLUDE_TREE_LOGICAL_ROOT,
    ACADOS_INTERFACE_TREE_LOGICAL_ROOT,
    ACADOS_LIBRARY_NAMES,
    ACADOS_LIBRARY_SONAMES,
    ACADOS_LINK_LIBS_LOGICAL_NAME,
    ACADOS_PREFIX_POLICY,
    ACADOS_REQUIRED_TAG,
    REQUIRED_ACADOS_SUBMODULES,
    TERA_SOURCE_BINDING_STATUS,
)
from acados.mainline.provenance_common import (
    GIT_DIRTY_POLICY,
    REPOSITORY_LOCATION_POLICY,
)
from acados.mainline.provenance_files import (
    FILE_IDENTITY_SCHEMA,
    TOOL_IDENTITY_SCHEMA,
    TREE_IDENTITY_SCHEMA,
)
from acados.mainline.provenance_git import (
    MAINLINE_BASE_SHA,
    MAINLINE_BRANCH,
    REPOSITORY_IDENTITY_SCHEMA,
    REPOSITORY_SOURCE_LOGICAL_ROOT,
)
from acados.mainline.provenance_python import (
    CASADI_PACKAGE_FILE_LOGICAL_NAMES,
    NUMPY_PACKAGE_FILE_LOGICAL_NAMES,
    PYTHON_IDENTITY_SCHEMA,
)
from acados.mainline.provenance_schema import (
    BUILD_COMMANDS,
    GENERATOR_API,
    GENERATOR_ARGUMENTS,
    PROVENANCE_ARTIFACT_CLASS,
    PROVENANCE_PROMOTION_STATUS,
    PROVENANCE_SCHEMA,
    PROVENANCE_SCOPE,
    PROVENANCE_STATUS,
    STAGING_LOCATION_POLICY,
    TOOL_ROLES,
)
from acados.mainline.solver_parameter_layout import build_solver_parameter_layout

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)


def _write_codegen_fixture(root: Path) -> None:
    (root / "model").mkdir(parents=True)
    (root / f"acados_ocp_{MODEL_ID}.json").write_text(
        '{"model":"mainline"}\n', encoding="utf-8"
    )
    (root / "Makefile").write_text("ocp_shared_lib:\n\t@true\n", encoding="utf-8")
    (root / "model" / "model.c").write_text(
        "int model(void) { return 0; }\n", encoding="utf-8"
    )
    (root / "model" / "model.h").write_text("int model(void);\n", encoding="utf-8")
    (root / SOLVER_LIBRARY_BASENAMES[0]).write_bytes(b"ELF-mainline-solver")


def _serialized_provenance_fixture(
    compiler_environment: dict[str, str | None],
) -> dict:
    """Build a complete canonical provenance document without backends."""

    def semantic(payload: dict, scope: str | None = None) -> dict:
        document = copy.deepcopy(payload)
        identity = {"sha256": sha256_json(payload)}
        if scope is not None:
            identity["scope"] = scope
        document["semantic_identity"] = identity
        return document

    def linked(
        name: str,
        executable: bool = False,
        *,
        path: str | None = None,
    ) -> dict:
        path = path or f"/fixture/{name}"
        return {
            "schema_version": FILE_IDENTITY_SCHEMA,
            "logical_name": name,
            "requested_path": path,
            "resolved_path": path,
            "leaf_symlink_chain": [],
            "size_bytes": 1,
            "raw_sha256": "1" * 64,
            "executable": executable,
        }

    def source_tree(logical_root: str, paths: tuple[str, ...]) -> dict:
        return semantic(
            {
                "schema_version": TREE_IDENTITY_SCHEMA,
                "logical_root": logical_root,
                "capture_root": REPOSITORY_LOCATION_POLICY,
                "files": [
                    {
                        "relative_path": path,
                        "size_bytes": 1,
                        "raw_sha256": "2" * 64,
                    }
                    for path in paths
                ],
            }
        )

    repository = semantic(
        {
            "schema_version": REPOSITORY_IDENTITY_SCHEMA,
            "repository_root": REPOSITORY_LOCATION_POLICY,
            "branch": MAINLINE_BRANCH,
            "head_sha": "c" * 40,
            "base_sha": MAINLINE_BASE_SHA,
            "worktree": {
                "clean": True,
                "dirty_policy": GIT_DIRTY_POLICY,
                "status_entry_count": 0,
                "status_porcelain_sha256": "3" * 64,
            },
            "sources": source_tree(
                REPOSITORY_SOURCE_LOGICAL_ROOT,
                ("fixture.py",),
            ),
        }
    )

    probe_arguments = {
        "git": (("version", ["--version"]),),
        "python": (("version", ["--version"]),),
        "tera": (("version", ["--version"]),),
        "make": (("version", ["--version"]),),
        "nm": (("version", ["--version"]),),
        "readelf": (("version", ["--version"]),),
        "cc": (
            ("version", ["--version"]),
            ("target", ["-dumpmachine"]),
            ("full_version", ["-dumpfullversion", "-dumpversion"]),
        ),
        "cxx": (
            ("version", ["--version"]),
            ("target", ["-dumpmachine"]),
            ("full_version", ["-dumpfullversion", "-dumpversion"]),
        ),
        "ar": (("version", ["--version"]),),
        "ranlib": (("version", ["--version"]),),
    }
    requested_commands = {
        "git": "git",
        "python": "/fixture/tool-python",
        "tera": "/fixture/tool-tera",
        "make": "make",
        "nm": "nm",
        "readelf": "readelf",
        "cc": compiler_environment["CC"] or "cc",
        "cxx": compiler_environment["CXX"] or "c++",
        "ar": compiler_environment["AR"] or "ar",
        "ranlib": compiler_environment["RANLIB"] or "ranlib",
    }
    tools = []
    for role in TOOL_ROLES:
        command = requested_commands[role]
        executable_path = command if command.startswith("/") else f"/fixture/bin/{role}"
        tools.append(
            semantic(
                {
                    "schema_version": TOOL_IDENTITY_SCHEMA,
                    "role": role,
                    "requested_command": command,
                    "executable": linked(
                        f"tool:{role}",
                        executable=True,
                        path=executable_path,
                    ),
                    "probes": [
                        {
                            "name": name,
                            "arguments": arguments,
                            "output_text": "fixture version",
                            "output_raw_sha256": "4" * 64,
                        }
                        for name, arguments in probe_arguments[role]
                    ],
                }
            )
        )

    casadi_version = "3.fixture"
    python_runtime = semantic(
        {
            "schema_version": PYTHON_IDENTITY_SCHEMA,
            "implementation": "CPython",
            "version": "3.fixture",
            "version_info": [3, 11, 0],
            "executable_tool_sha256": tools[1]["semantic_identity"]["sha256"],
            "sys_prefix": "/fixture/python",
            "sys_base_prefix": "/fixture/python",
            "sys_path": ["/fixture/python/lib"],
            "PYTHONPATH": None,
            "packages": [
                {
                    "name": "casadi",
                    "version": casadi_version,
                    "files": [
                        linked(name) for name in CASADI_PACKAGE_FILE_LOGICAL_NAMES
                    ],
                },
                {
                    "name": "numpy",
                    "version": "2.fixture",
                    "files": [
                        linked(name) for name in NUMPY_PACKAGE_FILE_LOGICAL_NAMES
                    ],
                },
            ],
        }
    )

    acados_libraries = [
        {
            "logical_name": name,
            "file": linked(f"acados/lib/{name}"),
            "soname": ACADOS_LIBRARY_SONAMES[name],
            "needed": [],
            "rpath": None,
            "runpath": None,
            "dynamic_section_sha256": "5" * 64,
        }
        for name in ACADOS_LIBRARY_NAMES
    ]
    acados = semantic(
        {
            "schema_version": ACADOS_IDENTITY_SCHEMA,
            "install_root": "/fixture/acados",
            "install_prefix_policy": ACADOS_PREFIX_POLICY,
            "source_repository": {
                "root": "/fixture/acados",
                "head_sha": "1234567890" + "a" * 30,
                "exact_tag": ACADOS_REQUIRED_TAG,
                "worktree_clean": True,
                "dirty_policy": GIT_DIRTY_POLICY,
                "status_porcelain_sha256": "6" * 64,
            },
            "commit_marker": "1234567890",
            "commit_marker_file": linked(ACADOS_COMMIT_MARKER_LOGICAL_NAME),
            "link_libs": {
                "file": linked(ACADOS_LINK_LIBS_LOGICAL_NAME),
                "canonical_json_sha256": "7" * 64,
            },
            "interface_source_binding_status": "MATCHED_SOURCE_ROOT",
            "tera_source_binding_status": TERA_SOURCE_BINDING_STATUS,
            "interface_tree": source_tree(
                ACADOS_INTERFACE_TREE_LOGICAL_ROOT,
                ACADOS_INTERFACE_SOURCE_PATHS,
            ),
            "include_tree": source_tree(
                ACADOS_INCLUDE_TREE_LOGICAL_ROOT,
                ("fixture.h",),
            ),
            "submodules": [
                {
                    "path": path,
                    "commit_sha": "b" * 40,
                    "initialized": True,
                    "worktree_matches_index": True,
                }
                for path in sorted(REQUIRED_ACADOS_SUBMODULES)
            ],
            "libraries": acados_libraries,
        }
    )

    return semantic(
        {
            "schema_version": PROVENANCE_SCHEMA,
            "scope": PROVENANCE_SCOPE,
            "status": {
                "provenance": PROVENANCE_STATUS,
                "artifact_class": PROVENANCE_ARTIFACT_CLASS,
                "promotion": PROVENANCE_PROMOTION_STATUS,
            },
            "repository": repository,
            "compiler_environment": dict(compiler_environment),
            "tools": tools,
            "python_runtime": python_runtime,
            "acados": acados,
            "host": {
                "system": "Linux",
                "release": "fixture",
                "machine": "x86_64",
                "libc": ["glibc", "2.fixture"],
                "byteorder": "little",
            },
            "logical_codegen_commands": {
                "generator_api": GENERATOR_API,
                "generator_arguments": dict(GENERATOR_ARGUMENTS),
                "build_commands": [list(item) for item in BUILD_COMMANDS],
                "staging_location": STAGING_LOCATION_POLICY,
            },
        },
        PROVENANCE_SCOPE,
    )


def _resign_semantic_document(document: dict) -> None:
    payload = {
        key: value for key, value in document.items() if key != "semantic_identity"
    }
    document["semantic_identity"]["sha256"] = sha256_json(payload)


def _resign_artifact_document(document: dict) -> None:
    from acados.mainline import artifact_contract as contract_module

    payload = {
        key: value
        for key, value in document.items()
        if key not in {"semantic_identity", "artifact_identity"}
    }
    semantic_sha256 = sha256_json(payload)
    document["semantic_identity"]["sha256"] = semantic_sha256
    document["artifact_identity"]["sha256"] = contract_module._artifact_sha256(
        semantic_sha256,
        payload,
    )


class MainlineArtifactContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.capacity = load_development_capacity(CAPACITY)
        self.development_layout = build_development_layout(self.capacity)
        self.parameter_layout = build_solver_parameter_layout(
            self.capacity, self.development_layout
        )

    @staticmethod
    def _semantic_document(payload, scope):
        document = copy.deepcopy(payload)
        document["semantic_identity"] = {
            "sha256": sha256_json(payload),
            "scope": scope,
        }
        return document

    def _artifact_contract(self, root: Path):
        from acados.mainline import acados_ocp_contract as ocp_contract
        from acados.mainline import artifact_contract as contract_module
        from acados.mainline.acados_codegen_backend import (
            ACADOS_CODEGEN_RESULT_SCHEMA,
            ACADOS_CODEGEN_RESULT_SCOPE,
        )
        from acados.mainline.acados_codegen_result_schema import (
            REQUIRED_SOLVER_SYMBOLS,
        )
        from acados.mainline.acados_ocp_contract import OCP_IDENTITY_SCOPE
        from acados.mainline.artifact_files import solver_library_record
        from acados.mainline.casadi_graph_contract import (
            BOUND_SNAPSHOT_SCOPE,
            CASADI_GRAPH_SCHEMA,
            DIAGNOSTIC_RESIDUAL_ROLE,
            GRAPH_IDENTITY_SCOPE,
            GRAPH_STATUS,
            RUNTIME_PARAMETER_INPUT_POLICY,
            STAGE_REFERENCE_DOMAIN_ORDER,
            STAGE_SEMANTICS,
            TERMINAL_CONTROL_POLICY,
            TERMINAL_REFERENCE_DOMAIN_ORDER,
            graph_semantic_sha256,
        )
        from acados.mainline.codegen_options import (
            RUNTIME_IDENTITY_EXCLUSIONS,
            build_codegen_options_snapshot,
        )
        from acados.mainline.constraints_oracle import (
            CONSTRAINT_RESIDUAL_ORDER,
            CONSTRAINT_SCHEMA,
            CONSTRAINT_VALUE_STATUS,
            CONTROL_BOX_ORDER,
            STAGE_NONLINEAR_H_ORDER,
        )
        from acados.mainline.solver_options import build_solver_options_snapshot

        _write_codegen_fixture(root)
        records = inventory_generated_tree(root)
        library = solver_library_record(records)
        layout_sha256 = sha256_json(self.development_layout.to_dict())
        parameter_sha256 = sha256_json(self.parameter_layout.to_dict())
        bounds_snapshot = {
            "constraint_schema": CONSTRAINT_SCHEMA,
            "value_status": CONSTRAINT_VALUE_STATUS,
            "values": {
                "q_issue_v_max": 1.0,
                "q_issue_omega_max": 2.0,
                "a_issue_max": 3.0,
                "alpha_issue_max": 4.0,
                "jerk_v_max": 5.0,
                "jerk_omega_max": 6.0,
                "v_s_max": 7.0,
            },
        }
        bounds_sha256 = sha256_json(bounds_snapshot)
        graph_sha256 = graph_semantic_sha256(
            capacity_contract_sha256=self.capacity.contract_sha256,
            development_layout_sha256=layout_sha256,
            solver_parameter_layout_sha256=parameter_sha256,
            horizon_steps=60,
            parameter_vector_count=61,
            nx=48,
            nu=3,
            np=162,
            state_order=self.development_layout.state_names,
            control_order=self.development_layout.control_names,
            parameter_order=self.parameter_layout.parameter_names,
            control_indices=(0, 1, 2),
        )

        graph_document = {
            "schema_version": CASADI_GRAPH_SCHEMA,
            "status": {"graph": GRAPH_STATUS, "artifact": "NO_ARTIFACT"},
            "model_id": MODEL_ID,
            "casadi_version": "3.fixture",
            "graph_identity_scope": GRAPH_IDENTITY_SCOPE,
            "runtime_parameter_values": "EXCLUDED_FROM_GRAPH_IDENTITY",
            "runtime_parameter_input_policy": RUNTIME_PARAMETER_INPUT_POLICY,
            "horizon": {
                "N": 60,
                "parameter_vector_count": 61,
                "release_frequency_hz": 30,
                "release_period_sec": {"numerator": 1, "denominator": 30},
            },
            "stage_semantics": STAGE_SEMANTICS,
            "schemas": {
                "discretization": DISCRETIZATION_SCHEMA,
                "reference": "normalized_local_cubic_xy_v1",
                "cost": COST_SCHEMA,
                "constraints": CONSTRAINT_SCHEMA,
            },
            "execution": {
                "slot_count": 3,
                "schedule_policy": "FIXED_WIDTH_RUNTIME_VALUES_ALL_SLOTS_EXPANDED",
            },
            "dimensions": {"N": 60, "NX": 48, "NU": 3, "NP": 162},
            "orders": {
                "state": list(self.development_layout.state_names),
                "control": list(self.development_layout.control_names),
                "parameter": list(self.parameter_layout.parameter_names),
            },
            "source_identity": {
                "capacity_contract_raw_bytes_sha256": (self.capacity.contract_sha256),
                "development_layout_semantic_sha256": layout_sha256,
                "solver_parameter_layout_semantic_sha256": parameter_sha256,
            },
            "graph_semantic_identity": {
                "sha256": graph_sha256,
                "scope": GRAPH_IDENTITY_SCOPE,
            },
            "constraints": {
                "value_status": CONSTRAINT_VALUE_STATUS,
                "stage_nonlinear_h": {
                    "order": list(STAGE_NONLINEAR_H_ORDER),
                    "lower": [-1.0, -2.0, -3.0, -4.0],
                    "upper": [1.0, 2.0, 3.0, 4.0],
                },
                "stage_reference_domain": {
                    "order": list(STAGE_REFERENCE_DOMAIN_ORDER),
                    "lower": [0.0],
                    "upper": [1.0],
                },
                "terminal_reference_domain": {
                    "order": list(TERMINAL_REFERENCE_DOMAIN_ORDER),
                    "lower": [0.0],
                    "upper": [1.0],
                },
                "control_box": {
                    "indices": [0, 1, 2],
                    "order": list(CONTROL_BOX_ORDER),
                    "lower": [-5.0, -6.0, 0.0],
                    "upper": [5.0, 6.0, 7.0],
                },
                "diagnostic_residuals": {
                    "role": DIAGNOSTIC_RESIDUAL_ROLE,
                    "order": list(CONSTRAINT_RESIDUAL_ORDER),
                    "feasible_upper": [0.0] * len(CONSTRAINT_RESIDUAL_ORDER),
                },
                "bounds_snapshot_scope": BOUND_SNAPSHOT_SCOPE,
                "bounds_snapshot": bounds_snapshot,
                "bounds_snapshot_sha256": bounds_sha256,
            },
            "terminal": {
                "input_order": ["x_N", "p_N"],
                "control_policy": TERMINAL_CONTROL_POLICY,
                "liquid_cost_policy": "IDENTICALLY_ZERO",
            },
            "comparison_identity": {
                "arms": ["B0", "Bslosh"],
                "same_symbolic_graph": True,
                "only_parameter_fields_allowed_to_differ": [
                    "liquid_run_coeff",
                    "liquid_boundary_coeff",
                ],
                "must_be_identical": [
                    "dynamics",
                    "reference",
                    "robot_cost",
                    "constraints",
                    "all_other_stage_parameters",
                ],
                "liquid_hard_constraints": "DISABLED_FOR_B0_AND_BSLOSH",
            },
        }
        solver_snapshot = build_solver_options_snapshot(self.development_layout)
        solver_document = solver_snapshot.to_dict()
        codegen_snapshot = build_codegen_options_snapshot(
            self.capacity,
            self.development_layout,
            1.0e-12,
            1.0e-12,
            "-O2",
        )
        codegen_document = codegen_snapshot.to_dict()
        self.assertEqual(
            codegen_document["runtime_identity_exclusions"],
            list(RUNTIME_IDENTITY_EXCLUSIONS),
        )
        compiler_environment = codegen_document["acados_codegen"]["build"][
            "compiler_environment"
        ]
        provenance_document = _serialized_provenance_fixture(compiler_environment)
        interface_inventory = {
            item["relative_path"]: item["raw_sha256"]
            for item in provenance_document["acados"]["interface_tree"]["files"]
        }
        interface_source_sha256 = acados_interface_source_sha256_from_inventory(
            interface_inventory
        )
        self.assertEqual(set(interface_inventory), set(ACADOS_INTERFACE_SOURCE_PATHS))
        ocp_fixture = SimpleNamespace(
            schema_version=ocp_contract.ACADOS_OCP_SCHEMA,
            model_id=MODEL_ID,
            ocp_status=ocp_contract.OCP_STATUS,
            artifact_status=ocp_contract.OCP_ARTIFACT_STATUS,
            promotion_status=ocp_contract.OCP_PROMOTION_STATUS,
            horizon_steps=60,
            nx=48,
            nu=3,
            np=162,
            graph_semantic_sha256=graph_sha256,
            bounds_snapshot_sha256=bounds_sha256,
            solver_options_semantic_sha256=solver_document["semantic_identity"][
                "sha256"
            ],
            backend_solver_options_baseline_sha256="2" * 64,
            capacity_contract_sha256=self.capacity.contract_sha256,
            development_layout_sha256=layout_sha256,
            solver_parameter_layout_sha256=parameter_sha256,
            casadi_version="3.fixture",
            acados_git_commit="1234567890",
            acados_interface_source_sha256=interface_source_sha256,
            acados_backend_binding_status="MATCHED_SOURCE_ROOT",
            dynamics_expression_sha256="4" * 64,
            stage_cost_expression_sha256="5" * 64,
            terminal_cost_expression_sha256="6" * 64,
            stage_h_expression_sha256="7" * 64,
            terminal_h_expression_sha256="8" * 64,
            stage_constraint_order=tuple(STAGE_NONLINEAR_H_ORDER)
            + tuple(STAGE_REFERENCE_DOMAIN_ORDER),
            stage_constraint_lower=(-1.0, -2.0, -3.0, -4.0, 0.0),
            stage_constraint_upper=(1.0, 2.0, 3.0, 4.0, 1.0),
            terminal_constraint_order=tuple(TERMINAL_REFERENCE_DOMAIN_ORDER),
            terminal_constraint_lower=(0.0,),
            terminal_constraint_upper=(1.0,),
            control_order=tuple(CONTROL_BOX_ORDER),
            control_indices=(0, 1, 2),
            control_lower=(-5.0, -6.0, 0.0),
            control_upper=(5.0, 6.0, 7.0),
        )
        ocp_document = self._semantic_document(
            ocp_contract._assembly_payload(ocp_fixture),
            OCP_IDENTITY_SCOPE,
        )
        result_payload = {
            "schema_version": ACADOS_CODEGEN_RESULT_SCHEMA,
            "scope": ACADOS_CODEGEN_RESULT_SCOPE,
            "model_id": MODEL_ID,
            "status": {
                "codegen": "GENERATED_AND_BUILT",
                "artifact_class": "DEV_UNVALIDATED",
                "promotion": "NOT_PROMOTED",
                "target_performance": "NOT_BENCHMARKED",
            },
            "output_directory": "ABSOLUTE_STAGING_ROOT_EXCLUDED_FROM_ARTIFACT_BYTES",
            "failure_output_policy": "PARTIAL_STAGING_RETAINED_NEVER_PROMOTED",
            "generated_tree": {
                "sha256": generated_tree_sha256(records),
                "files": [item.to_dict() for item in records],
            },
            "solver_library": {
                "relative_path": library.relative_path,
                "size_bytes": library.size_bytes,
                "raw_sha256": library.raw_sha256,
                "format": "ELF",
                "elf_class": 64,
                "elf_machine": 62,
                "required_exported_symbols": list(REQUIRED_SOLVER_SYMBOLS),
                "load_check": "PASSED_IN_ISOLATED_PROCESS",
            },
        }
        result_document = self._semantic_document(
            result_payload, ACADOS_CODEGEN_RESULT_SCOPE
        )
        graph = SimpleNamespace(to_dict=lambda: copy.deepcopy(graph_document))
        assembly = SimpleNamespace(to_dict=lambda: copy.deepcopy(ocp_document))
        solver = solver_snapshot
        codegen = codegen_snapshot
        provenance = SimpleNamespace(to_dict=lambda: copy.deepcopy(provenance_document))
        result = SimpleNamespace(to_dict=lambda: copy.deepcopy(result_document))
        payload = contract_module._artifact_payload(
            self.capacity,
            self.development_layout,
            self.parameter_layout,
            graph,
            assembly,
            solver,
            codegen,
            provenance,
            result,
        )
        semantic_sha256 = sha256_json(payload)
        artifact_sha256 = contract_module._artifact_sha256(semantic_sha256, payload)
        return contract_module.ArtifactContract(
            canonical_json(payload),
            semantic_sha256,
            artifact_sha256,
            _construction_token=contract_module._CONSTRUCTION_TOKEN,
        )

    def test_model_contract_authority_is_dependency_free_and_json_canonical(
        self,
    ) -> None:
        source = SCRIPTS_ROOT / "acados" / "mainline" / "model_contract.py"
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)
        self.assertFalse(
            any(
                name.split(".", 1)[0] in {"casadi", "numpy", "acados_template"}
                for name in imported
            )
        )
        authority = {
            "model_id": MODEL_ID,
            "discretization": DISCRETIZATION_SCHEMA,
            "cost": COST_SCHEMA,
        }
        canonical = json.dumps(
            authority,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        self.assertEqual(
            sha256_json(authority), hashlib.sha256(canonical.encode()).hexdigest()
        )

    def test_dimensions_order_and_offsets_are_one_contiguous_authority(self) -> None:
        layout = self.development_layout
        parameters = self.parameter_layout
        self.assertEqual((layout.NX, layout.NU, parameters.np), (48, 3, 162))
        self.assertEqual(tuple(layout.state_offsets.values()), tuple(range(layout.NX)))
        self.assertEqual(
            tuple(layout.control_offsets.values()), tuple(range(layout.NU))
        )
        self.assertEqual(
            tuple(parameters.parameter_offsets.values()), tuple(range(parameters.np))
        )
        self.assertEqual(
            tuple(item.end_exclusive for item in parameters.block_ranges.values()),
            tuple(item.begin + item.size for item in parameters.block_ranges.values()),
        )
        self.assertEqual(
            parameters.block_ranges["execution_prefix"].end_exclusive,
            layout.NP_exec,
        )
        self.assertEqual(
            parameters.parameter_names[-2:],
            ("liquid_run_coeff", "liquid_boundary_coeff"),
        )

    def test_artifact_serialization_is_canonical_root_independent_and_nonrecursive(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            first_root = Path(directory) / "first"
            second_root = Path(directory) / "second"
            first_root.mkdir()
            second_root.mkdir()
            _write_codegen_fixture(first_root)
            _write_codegen_fixture(second_root)
            for root in (first_root, second_root):
                (root / MODEL_CONTRACT_FILENAME).write_text(
                    "contract", encoding="utf-8"
                )
                (root / GENERATED_HEADER_FILENAME).write_text(
                    "header", encoding="utf-8"
                )
            first = inventory_generated_tree(first_root)
            second = inventory_generated_tree(second_root)
            self.assertEqual(
                generated_tree_sha256(first), generated_tree_sha256(second)
            )
            self.assertEqual(
                tuple(record.relative_path for record in first),
                tuple(sorted(record.relative_path for record in first)),
            )
            self.assertEqual(
                sum(record.role == ACADOS_JSON_ROLE for record in first), 1
            )
            self.assertEqual(
                sum(record.role == GENERATED_C_SOURCE_ROLE for record in first), 1
            )
            self.assertEqual(
                sum(record.role == GENERATED_C_HEADER_ROLE for record in first), 1
            )
            self.assertNotIn(
                MODEL_CONTRACT_FILENAME,
                {record.relative_path for record in first},
            )
            self.assertNotIn(
                GENERATED_HEADER_FILENAME,
                {record.relative_path for record in first},
            )

    def test_artifact_and_hash_tampering_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_codegen_fixture(root)
            baseline = inventory_generated_tree(root)
            serialized = [record.to_dict() for record in baseline]
            tampered_bytes = copy.deepcopy(serialized)
            tampered_bytes[0]["raw_sha256"] = "0" * 64
            forged = generated_file_records_from_dict(tampered_bytes)
            self.assertNotEqual(
                generated_tree_sha256(forged), generated_tree_sha256(baseline)
            )
            with self.assertRaises(ValueError):
                validate_generated_tree(root, forged)
            tampered_shape = copy.deepcopy(serialized)
            tampered_shape[0]["relative_path"] = "../escape"
            with self.assertRaises(ValueError):
                generated_file_records_from_dict(tampered_shape)

    def test_codegen_result_schema_pins_model_specific_symbols(self) -> None:
        from acados.mainline.acados_codegen_result_schema import (
            validate_acados_codegen_result_document,
        )

        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            result = contract.to_dict()["artifact"]
            self.assertIs(validate_acados_codegen_result_document(result), result)

            forged = copy.deepcopy(result)
            forged["solver_library"]["required_exported_symbols"][-1] += "_tampered"
            payload = {
                key: value
                for key, value in forged.items()
                if key != "semantic_identity"
            }
            forged["semantic_identity"]["sha256"] = sha256_json(payload)
            with self.assertRaisesRegex(ValueError, "exported-symbol"):
                validate_acados_codegen_result_document(forged)

    def test_b0_and_bslosh_policy_has_one_shared_artifact_identity(self) -> None:
        ocp_contract = importlib.import_module("acados.mainline.acados_ocp_contract")
        graph_contract = importlib.import_module(
            "acados.mainline.casadi_graph_contract"
        )
        self.assertEqual(ocp_contract.MODEL_ID, MODEL_ID)
        self.assertEqual(graph_contract.MODEL_ID, MODEL_ID)
        self.assertEqual(
            ocp_contract.OCP_ARTIFACT_STATUS,
            "NO_ARTIFACT",
        )
        payload = ocp_contract._assembly_payload
        source = payload.__module__
        self.assertIn("ONE_SHARED_ARTIFACT_REQUIRED", payload.__code__.co_consts)
        self.assertNotIn("B0", source)

    def test_rendered_model_header_is_cpp14_and_exposes_the_same_dimensions(
        self,
    ) -> None:
        header_module = importlib.import_module("acados.mainline.model_contract_header")
        renderer = header_module.render_model_contract_header
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            header = renderer(contract)
        self.assertIsInstance(header, str)
        self.assertIn("static_assert", header)
        for expected in ("NX", "NU", "NP", "N", "48", "3", "162", "60"):
            self.assertIn(expected, header)
        self.assertNotIn("#include <json", header)
        self.assertIn(contract.semantic_sha256, header)
        self.assertIn(contract.artifact_sha256, header)
        self.assertIn("kParameterLiquidRunCoeff", header)
        with self.assertRaises((TypeError, ValueError)):
            renderer()  # type: ignore[call-arg]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            generated = root / "model_contract_generated.h"
            generated.write_text(header, encoding="utf-8")
            source = root / "check.cpp"
            source.write_text(
                '#include "model_contract_generated.h"\nint main() { return 0; }\n',
                encoding="utf-8",
            )
            compiler = shutil.which("c++") or shutil.which("g++")
            if compiler is None:
                self.skipTest("a C++ compiler is unavailable")
            subprocess.run(
                [compiler, "-std=c++14", "-Werror", "-fsyntax-only", str(source)],
                check=True,
                capture_output=True,
            )

    def test_json_and_header_freshness_reject_exact_output_drift(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        header_module = importlib.import_module("acados.mainline.model_contract_header")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            rendered_json = artifact_module.render_model_contract_json(contract)
            self.assertIs(
                artifact_module.validate_model_contract_json(
                    contract,
                    rendered_json,
                ),
                rendered_json,
            )
            with self.assertRaisesRegex(ValueError, "stale or modified"):
                artifact_module.validate_model_contract_json(
                    contract,
                    rendered_json[:-1],
                )

            rendered_header = header_module.render_model_contract_header(contract)
            self.assertIs(
                header_module.validate_model_contract_header(
                    contract,
                    rendered_header,
                ),
                rendered_header,
            )
            forged_sha_header = rendered_header.replace(
                contract.semantic_sha256,
                "0" * 64,
                1,
            )
            self.assertNotEqual(forged_sha_header, rendered_header)
            with self.assertRaisesRegex(ValueError, "stale or modified"):
                header_module.validate_model_contract_header(
                    contract,
                    forged_sha_header,
                )
            forged_name_header = rendered_header.replace('"px"', '"px_forged"', 1)
            self.assertNotEqual(forged_name_header, rendered_header)
            with self.assertRaisesRegex(ValueError, "stale or modified"):
                header_module.validate_model_contract_header(
                    contract,
                    forged_name_header,
                )

            compiler = shutil.which("c++") or shutil.which("g++")
            if compiler is None:
                self.skipTest("a C++ compiler is unavailable")
            generated = Path(directory) / "forged_sha_model_contract.h"
            generated.write_text(forged_sha_header, encoding="utf-8")
            source = Path(directory) / "forged_sha_check.cpp"
            source.write_text(
                '#include "forged_sha_model_contract.h"\nint main() { return 0; }\n',
                encoding="utf-8",
            )
            subprocess.run(
                [compiler, "-std=c++14", "-Werror", "-fsyntax-only", str(source)],
                check=True,
                capture_output=True,
            )

    def test_contract_outputs_are_write_once_and_reload_from_disk(self) -> None:
        publication = importlib.import_module("acados.mainline.artifact_publication")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            contract = self._artifact_contract(root)
            written = publication.write_artifact_contract_outputs(root, contract)
            loaded = publication.load_artifact_contract_directory(root)
            self.assertEqual(written.to_dict(), contract.to_dict())
            self.assertEqual(loaded.to_dict(), contract.to_dict())
            self.assertTrue((root / MODEL_CONTRACT_FILENAME).is_file())
            self.assertTrue((root / GENERATED_HEADER_FILENAME).is_file())
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                publication.write_artifact_contract_outputs(root, contract)

    def test_contract_output_reader_rejects_byte_and_path_tampering(self) -> None:
        publication = importlib.import_module("acados.mainline.artifact_publication")
        for mutation in ("crlf", "duplicate_json", "header_symlink"):
            temporary_directory = tempfile.TemporaryDirectory()
            with self.subTest(mutation=mutation), temporary_directory as directory:
                root = Path(directory)
                contract = self._artifact_contract(root)
                if mutation == "header_symlink":
                    outside = root.parent / f"{root.name}-outside-header"
                    outside.write_text("outside\n", encoding="utf-8")
                    (root / GENERATED_HEADER_FILENAME).symlink_to(outside)
                    try:
                        with self.assertRaises(RuntimeError):
                            publication.write_artifact_contract_outputs(root, contract)
                        self.assertEqual(
                            outside.read_text(encoding="utf-8"),
                            "outside\n",
                        )
                    finally:
                        outside.unlink()
                    continue

                publication.write_artifact_contract_outputs(root, contract)
                json_path = root / MODEL_CONTRACT_FILENAME
                if mutation == "crlf":
                    payload = json_path.read_bytes()
                    json_path.write_bytes(payload.replace(b"\n", b"\r\n"))
                else:
                    json_path.write_bytes(b'{"duplicate":1,"duplicate":2}\n')
                with self.assertRaises(RuntimeError):
                    publication.load_artifact_contract_directory(root)

    def test_staging_publication_is_atomic_and_never_replaces_a_target(self) -> None:
        from unittest.mock import patch

        publication = importlib.import_module("acados.mainline.artifact_publication")
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            target = parent / "published"
            staging = publication.create_sibling_staging_directory(target)
            (staging / "payload").write_text("first\n", encoding="utf-8")
            published = publication.publish_staging_directory(staging, target)
            self.assertEqual(published, target)
            self.assertFalse(staging.exists())
            self.assertEqual((target / "payload").read_text(), "first\n")

            second_staging = parent / "second-staging"
            second_staging.mkdir()
            (second_staging / "payload").write_text("second\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "already exists"):
                publication.publish_staging_directory(second_staging, target)
            self.assertEqual((target / "payload").read_text(), "first\n")
            self.assertEqual(
                (second_staging / "payload").read_text(),
                "second\n",
            )

            race_target = parent / "race-target"
            race_staging = parent / "race-staging"
            race_staging.mkdir()
            (race_staging / "payload").write_text("candidate\n", encoding="utf-8")
            real_rename = publication._renameat2_noreplace

            def create_competing_target(descriptor, source, destination):
                race_target.mkdir()
                (race_target / "owner").write_text("other\n", encoding="utf-8")
                return real_rename(descriptor, source, destination)

            rename_patch = patch.object(
                publication,
                "_renameat2_noreplace",
                side_effect=create_competing_target,
            )
            expected_failure = self.assertRaisesRegex(RuntimeError, "appeared")
            with rename_patch, expected_failure:
                publication.publish_staging_directory(race_staging, race_target)
            self.assertEqual((race_target / "owner").read_text(), "other\n")
            self.assertEqual((race_staging / "payload").read_text(), "candidate\n")

    def test_post_commit_sync_failure_reports_the_published_target(self) -> None:
        from unittest.mock import patch

        publication = importlib.import_module("acados.mainline.artifact_publication")
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            target = parent / "published"
            staging = publication.create_sibling_staging_directory(target)
            (staging / "payload").write_text("committed\n", encoding="utf-8")
            sync_failure = patch.object(
                publication.os,
                "fsync",
                side_effect=OSError("injected parent sync failure"),
            )
            expected_failure = self.assertRaises(
                publication.ArtifactPublicationCommittedError
            )
            with sync_failure, expected_failure as raised:
                publication.publish_staging_directory(staging, target)
            self.assertEqual(raised.exception.published_target, target)
            self.assertFalse(staging.exists())
            self.assertEqual((target / "payload").read_text(), "committed\n")

    def test_artifact_contract_requires_exact_type_and_rejects_hash_mutation(
        self,
    ) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with self.assertRaises(ValueError):
            artifact_module.require_artifact_contract(object())
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            parsed = artifact_module.artifact_contract_from_dict(contract.to_dict())
            self.assertEqual(parsed.to_dict(), contract.to_dict())

            forged = copy.copy(contract)
            object.__setattr__(forged, "artifact_sha256", "0" * 64)
            with self.assertRaises(ValueError):
                artifact_module.require_artifact_contract(forged)

            tampered = contract.to_dict()
            tampered["layouts"]["state"]["offsets"]["px"] = 1
            payload = {
                key: value
                for key, value in tampered.items()
                if key not in {"semantic_identity", "artifact_identity"}
            }
            semantic_sha256 = sha256_json(payload)
            tampered["semantic_identity"]["sha256"] = semantic_sha256
            tampered["artifact_identity"]["sha256"] = artifact_module._artifact_sha256(
                semantic_sha256, payload
            )
            with self.assertRaises(ValueError):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_graph_semantic_identity_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            tampered["typed_authorities"]["casadi_graph"]["graph_semantic_identity"][
                "sha256"
            ] = "0" * 64
            payload = {
                key: value
                for key, value in tampered.items()
                if key not in {"semantic_identity", "artifact_identity"}
            }
            tampered["semantic_identity"]["sha256"] = sha256_json(payload)
            tampered["artifact_identity"]["sha256"] = artifact_module._artifact_sha256(
                tampered["semantic_identity"]["sha256"], payload
            )
            with self.assertRaisesRegex(
                ValueError, "graph semantic identity|CasADi graph"
            ):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_graph_casadi_version_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            tampered["typed_authorities"]["casadi_graph"]["casadi_version"] = (
                "3.tampered"
            )
            payload = {
                key: value
                for key, value in tampered.items()
                if key not in {"semantic_identity", "artifact_identity"}
            }
            tampered["semantic_identity"]["sha256"] = sha256_json(payload)
            tampered["artifact_identity"]["sha256"] = artifact_module._artifact_sha256(
                tampered["semantic_identity"]["sha256"], payload
            )
            with self.assertRaisesRegex(ValueError, "CasADi version|CasADi graph"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_provenance_logical_identity_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            provenance = tampered["typed_authorities"]["provenance"]
            provenance["logical_codegen_commands"]["generator_api"] = (
                "ForgedGenerator.generate"
            )
            provenance_payload = {
                key: value
                for key, value in provenance.items()
                if key != "semantic_identity"
            }
            provenance["semantic_identity"]["sha256"] = sha256_json(provenance_payload)
            payload = {
                key: value
                for key, value in tampered.items()
                if key not in {"semantic_identity", "artifact_identity"}
            }
            tampered["semantic_identity"]["sha256"] = sha256_json(payload)
            tampered["artifact_identity"]["sha256"] = artifact_module._artifact_sha256(
                tampered["semantic_identity"]["sha256"], payload
            )
            with self.assertRaisesRegex(ValueError, "provenance"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_graph_policy_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            tampered["typed_authorities"]["casadi_graph"]["comparison_identity"][
                "same_symbolic_graph"
            ] = False
            _resign_artifact_document(tampered)
            with self.assertRaisesRegex(ValueError, "CasADi graph|comparison"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_ocp_source_binding_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            ocp = tampered["typed_authorities"]["acados_ocp"]
            ocp["source_identity"]["graph_semantic_sha256"] = "9" * 64
            _resign_semantic_document(ocp)
            _resign_artifact_document(tampered)
            with self.assertRaisesRegex(ValueError, "authority hash bindings"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_acados_interface_inventory_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            provenance = tampered["typed_authorities"]["provenance"]
            interface_tree = provenance["acados"]["interface_tree"]
            interface_tree["files"][0]["raw_sha256"] = "a" * 64
            _resign_semantic_document(interface_tree)
            _resign_semantic_document(provenance["acados"])
            _resign_semantic_document(provenance)
            _resign_artifact_document(tampered)
            with self.assertRaisesRegex(ValueError, "interface identity"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_codegen_result_policy_tamper_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            tampered = contract.to_dict()
            result = tampered["artifact"]
            result["failure_output_policy"] = "FORGED_OVERWRITE_ALLOWED"
            _resign_semantic_document(result)
            _resign_artifact_document(tampered)
            with self.assertRaisesRegex(ValueError, "codegen result"):
                artifact_module.artifact_contract_from_dict(tampered)

    def test_rehashed_bool_int_policy_confusion_fails_closed(self) -> None:
        artifact_module = importlib.import_module("acados.mainline.artifact_contract")
        with tempfile.TemporaryDirectory() as directory:
            contract = self._artifact_contract(Path(directory))
            for label, mutate in (
                (
                    "parameter block false-as-zero",
                    lambda value: value["layouts"]["parameter_blocks"][
                        "execution_prefix"
                    ].__setitem__("begin", False),
                ),
                (
                    "comparison one-as-true",
                    lambda value: value["comparison_contract"].__setitem__(
                        "shared_artifact_required", 1
                    ),
                ),
            ):
                with self.subTest(label=label):
                    tampered = contract.to_dict()
                    mutate(tampered)
                    _resign_artifact_document(tampered)
                    with self.assertRaises(ValueError):
                        artifact_module.artifact_contract_from_dict(tampered)


if __name__ == "__main__":
    unittest.main()
