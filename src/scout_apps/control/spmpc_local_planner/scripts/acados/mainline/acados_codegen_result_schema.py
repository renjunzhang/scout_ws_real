"""Dependency-free schema and identity checks for one Acados codegen result.

The backend owns generation and filesystem checks.  This module owns the
serialized result policy so offline artifact readers enforce the same status,
inventory, ELF, load-check, and model-specific symbol contract.
"""

from __future__ import annotations

from typing import Any

from .artifact_files import (
    generated_file_records_from_dict,
    generated_tree_sha256,
    solver_library_record,
)
from .identity import IdentityError, require_sha256, sha256_json
from .model_contract import MODEL_ID

ACADOS_CODEGEN_RESULT_SCHEMA = "spmpc_mainline_acados_codegen_result_v1"
ACADOS_CODEGEN_RESULT_SCOPE = "CANONICAL_GENERATED_TREE_AND_LOADABLE_SOLVER"
ACADOS_CODEGEN_STATUS = "GENERATED_AND_BUILT"
ACADOS_CODEGEN_ARTIFACT_CLASS = "DEV_UNVALIDATED"
ACADOS_CODEGEN_PROMOTION_STATUS = "NOT_PROMOTED"
ACADOS_CODEGEN_PERFORMANCE_STATUS = "NOT_BENCHMARKED"
CODEGEN_FAILURE_OUTPUT_POLICY = "PARTIAL_STAGING_RETAINED_NEVER_PROMOTED"
OUTPUT_ROOT_IDENTITY_POLICY = "ABSOLUTE_STAGING_ROOT_EXCLUDED_FROM_ARTIFACT_BYTES"

ELF_SHARED_OBJECT_TYPE = 3
ELF_MACHINE_BY_PLATFORM = {
    "x86_64": 62,
    "amd64": 62,
    "aarch64": 183,
    "arm64": 183,
    "armv7l": 40,
    "i386": 3,
    "i686": 3,
}
SUPPORTED_ELF_IDENTITIES = frozenset(
    {
        (64, 62),
        (64, 183),
        (32, 40),
        (32, 3),
    }
)
REQUIRED_SOLVER_SYMBOL_SUFFIXES = (
    "acados_create_capsule",
    "acados_create",
    "acados_solve",
    "acados_free",
    "acados_free_capsule",
    "acados_update_params",
    "acados_update_params_sparse",
)
REQUIRED_SOLVER_SYMBOLS = tuple(
    sorted(f"{MODEL_ID}_{suffix}" for suffix in REQUIRED_SOLVER_SYMBOL_SUFFIXES)
)
SOLVER_LIBRARY_FORMAT = "ELF"
SOLVER_LOAD_CHECK_POLICY = "PASSED_IN_ISOLATED_PROCESS"


class AcadosCodegenResultSchemaError(ValueError):
    """A serialized Acados codegen result is malformed or policy-invalid."""


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict or set(value) != keys:
        raise AcadosCodegenResultSchemaError(f"{label} keys do not match the v1 schema")
    return value


def _validate_payload(value: Any) -> dict[str, Any]:
    """Validate a result payload excluding ``semantic_identity``."""

    artifact = _object(
        value,
        {
            "schema_version",
            "scope",
            "model_id",
            "status",
            "output_directory",
            "failure_output_policy",
            "generated_tree",
            "solver_library",
        },
        "Acados codegen result payload",
    )
    if (
        artifact["schema_version"] != ACADOS_CODEGEN_RESULT_SCHEMA
        or artifact["scope"] != ACADOS_CODEGEN_RESULT_SCOPE
        or artifact["model_id"] != MODEL_ID
    ):
        raise AcadosCodegenResultSchemaError(
            "generated artifact schema/model identity drifted"
        )
    if artifact["status"] != {
        "codegen": ACADOS_CODEGEN_STATUS,
        "artifact_class": ACADOS_CODEGEN_ARTIFACT_CLASS,
        "promotion": ACADOS_CODEGEN_PROMOTION_STATUS,
        "target_performance": ACADOS_CODEGEN_PERFORMANCE_STATUS,
    }:
        raise AcadosCodegenResultSchemaError("generated artifact status drifted")
    if artifact["output_directory"] != OUTPUT_ROOT_IDENTITY_POLICY:
        raise AcadosCodegenResultSchemaError(
            "generated artifact staging policy drifted"
        )
    if artifact["failure_output_policy"] != CODEGEN_FAILURE_OUTPUT_POLICY:
        raise AcadosCodegenResultSchemaError(
            "generated artifact failure policy drifted"
        )

    generated_tree = _object(
        artifact["generated_tree"],
        {"sha256", "files"},
        "generated artifact tree",
    )
    try:
        records = generated_file_records_from_dict(generated_tree["files"])
        require_sha256(generated_tree["sha256"], "generated tree identity")
        if generated_tree_sha256(records) != generated_tree["sha256"]:
            raise AcadosCodegenResultSchemaError(
                "generated tree identity is inconsistent"
            )
    except AcadosCodegenResultSchemaError:
        raise
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise AcadosCodegenResultSchemaError(
            "generated artifact tree is malformed"
        ) from exc

    solver_library = _object(
        artifact["solver_library"],
        {
            "relative_path",
            "size_bytes",
            "raw_sha256",
            "format",
            "elf_class",
            "elf_machine",
            "required_exported_symbols",
            "load_check",
        },
        "generated artifact solver library",
    )
    try:
        library = solver_library_record(records)
        require_sha256(solver_library["raw_sha256"], "solver library identity")
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise AcadosCodegenResultSchemaError(
            "generated artifact solver library is malformed"
        ) from exc
    if (
        solver_library["relative_path"] != library.relative_path
        or solver_library["size_bytes"] != library.size_bytes
        or solver_library["raw_sha256"] != library.raw_sha256
    ):
        raise AcadosCodegenResultSchemaError("solver library inventory binding drifted")
    if solver_library["format"] != SOLVER_LIBRARY_FORMAT:
        raise AcadosCodegenResultSchemaError("solver library format drifted")
    elf_class = solver_library["elf_class"]
    elf_machine = solver_library["elf_machine"]
    if (
        type(elf_class) is not int
        or type(elf_machine) is not int
        or (elf_class, elf_machine) not in SUPPORTED_ELF_IDENTITIES
    ):
        raise AcadosCodegenResultSchemaError(
            "solver ELF class/machine pair is unsupported"
        )
    if solver_library["required_exported_symbols"] != list(REQUIRED_SOLVER_SYMBOLS):
        raise AcadosCodegenResultSchemaError(
            "solver exported-symbol set is not the canonical model set"
        )
    if solver_library["load_check"] != SOLVER_LOAD_CHECK_POLICY:
        raise AcadosCodegenResultSchemaError("solver load-check status drifted")
    return artifact


def validate_acados_codegen_result_document(value: Any) -> dict[str, Any]:
    """Validate a complete serialized result and recompute its semantic SHA."""

    if type(value) is not dict:
        raise AcadosCodegenResultSchemaError(
            "Acados codegen result document must be a JSON object"
        )
    expected_keys = {
        "schema_version",
        "scope",
        "model_id",
        "status",
        "output_directory",
        "failure_output_policy",
        "generated_tree",
        "solver_library",
        "semantic_identity",
    }
    if set(value) != expected_keys:
        raise AcadosCodegenResultSchemaError(
            "Acados codegen result document keys do not match the v1 schema"
        )
    identity = _object(
        value["semantic_identity"],
        {"sha256", "scope"},
        "Acados codegen result semantic identity",
    )
    if identity["scope"] != ACADOS_CODEGEN_RESULT_SCOPE:
        raise AcadosCodegenResultSchemaError(
            "Acados codegen result semantic scope drifted"
        )
    try:
        require_sha256(identity["sha256"], "Acados codegen result identity")
        payload = {
            key: item for key, item in value.items() if key != "semantic_identity"
        }
        _validate_payload(payload)
        if sha256_json(payload) != identity["sha256"]:
            raise AcadosCodegenResultSchemaError(
                "Acados codegen result semantic identity is inconsistent"
            )
    except AcadosCodegenResultSchemaError:
        raise
    except (IdentityError, KeyError, TypeError, ValueError) as exc:
        raise AcadosCodegenResultSchemaError(
            "Acados codegen result document is malformed"
        ) from exc
    return value


__all__ = [
    "ACADOS_CODEGEN_ARTIFACT_CLASS",
    "ACADOS_CODEGEN_PERFORMANCE_STATUS",
    "ACADOS_CODEGEN_PROMOTION_STATUS",
    "ACADOS_CODEGEN_RESULT_SCHEMA",
    "ACADOS_CODEGEN_RESULT_SCOPE",
    "ACADOS_CODEGEN_STATUS",
    "CODEGEN_FAILURE_OUTPUT_POLICY",
    "ELF_MACHINE_BY_PLATFORM",
    "ELF_SHARED_OBJECT_TYPE",
    "MODEL_ID",
    "OUTPUT_ROOT_IDENTITY_POLICY",
    "REQUIRED_SOLVER_SYMBOLS",
    "REQUIRED_SOLVER_SYMBOL_SUFFIXES",
    "SOLVER_LIBRARY_FORMAT",
    "SOLVER_LOAD_CHECK_POLICY",
    "SUPPORTED_ELF_IDENTITIES",
    "AcadosCodegenResultSchemaError",
    "validate_acados_codegen_result_document",
]
