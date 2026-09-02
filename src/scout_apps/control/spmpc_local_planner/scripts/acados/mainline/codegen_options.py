"""Explicit, backend-neutral inputs for one Stage 3-D4 codegen run."""

from __future__ import annotations

import math
import os
from dataclasses import InitVar, dataclass
from fractions import Fraction
from typing import Any

from .development_capacity import DevelopmentCapacityContract
from .development_layout import (
    DevelopmentLayout,
    DevelopmentLayoutError,
    validate_development_layout_snapshot,
)
from .identity import IdentityError, require_sha256, sha256_json
from .model_contract import MODEL_ID
from .runtime_schedule import RUNTIME_SCHEDULE_SCHEMA_VERSION

CODEGEN_OPTIONS_SCHEMA = "spmpc_mainline_codegen_options_v1"
CODEGEN_OPTIONS_SCOPE = "DEVELOPMENT_CODEGEN_POLICY_NO_RUNTIME_VALUES"
CODEGEN_OPTIONS_STATUS = "DEV_UNVALIDATED"
CODEGEN_ARTIFACT_STATUS = "NO_ARTIFACT"
CODEGEN_TARGET_PERFORMANCE_STATUS = "NOT_BENCHMARKED"
CODEGEN_BACKEND_CONTRACT = "acados_template_v0.5.4_compatible"
CODEGEN_API = "AcadosOcpSolver.generate+AcadosOcpSolver.build"

EXT_FUN_COMPILE_FLAGS = "-O2"
EXT_FUN_EXPAND_CONSTR = False
EXT_FUN_EXPAND_COST = False
EXT_FUN_EXPAND_DYN = False
EXT_FUN_EXPAND_PRECOMPUTE = False
CUSTOM_UPDATE_FILENAME = ""
CUSTOM_UPDATE_HEADER_FILENAME = ""
CUSTOM_UPDATE_COPY = False
CUSTOM_TEMPLATES: tuple[tuple[str, ...], ...] = ()

BUILD_SYSTEM = "MAKEFILE"
BUILD_TARGET = "OCP_SHARED_LIBRARY"
WITH_CYTHON = False
CODEGEN_VERBOSE = False
CMAKE_BUILDER = "NONE"
OUTPUT_DIRECTORY_POLICY = "EXPLICIT_NEW_OR_EMPTY_DIRECTORY_NO_SYMLINK"
SOURCE_ROOT_BINDING_POLICY = "REQUIRE_MATCHED_SOURCE_ROOT"
GIT_DIRTY_POLICY = "RECORDED_NOT_GATED"
COMPILER_ENVIRONMENT_POLICY = "RECORDED_NOT_GATED"
COMPILER_ENVIRONMENT_NAMES = (
    "CC",
    "CXX",
    "CFLAGS",
    "CXXFLAGS",
    "CPPFLAGS",
    "LDFLAGS",
    "MAKEFLAGS",
)
GENERATED_FILE_INVENTORY_POLICY = (
    "ALL_REGULAR_FILES_EXCEPT_MODEL_CONTRACT_JSON_AND_GENERATED_HEADER"
)
ARTIFACT_IDENTITY_POLICY = (
    "CONTRACT_SHA256_PLUS_SORTED_RELATIVE_PATH_SIZE_AND_RAW_SHA256"
)
ACADOS_JSON_FILENAME = f"acados_ocp_{MODEL_ID}.json"
MODEL_CONTRACT_FILENAME = "model_contract.json"
GENERATED_HEADER_FILENAME = "model_contract_generated.h"

RUNTIME_IDENTITY_EXCLUSIONS = (
    "runtime_schedule_hash",
    "delay_m_beta_segment_durations_and_selectors",
    "actuator_delay_tau_gain_values",
    "liquid_window_and_weight_values",
    "experiment_condition",
    "liquid_run_and_boundary_coefficients",
    "initial_state",
    "stage_parameter_matrix",
    "command_history",
    "path_identity",
    "cycle_timestamps_and_generation",
    "trial_effective_config_hash",
    "bag_and_experiment_evidence",
)

_SNAPSHOT_TOKEN = object()


class CodegenOptionsError(ValueError):
    """The sole development codegen policy is malformed or inconsistent."""


def _explicit_tolerance(value: Any, label: str, upper: float) -> float:
    if type(value) is not float:
        raise CodegenOptionsError(f"{label} must be an explicit float")
    result = 0.0 if value == 0.0 else value
    if not math.isfinite(result) or result < 0.0 or result >= upper:
        raise CodegenOptionsError(f"{label} must be finite in [0, {upper})")
    return result


def _compiler_environment() -> tuple[tuple[str, str], ...]:
    """Capture only inherited build variables that can alter compilation."""

    return tuple(
        (name, os.environ.get(name, "")) for name in COMPILER_ENVIRONMENT_NAMES
    )


@dataclass(frozen=True)
class CodegenOptionsSnapshot:
    """Immutable generation policy with no output path or trial-specific values."""

    development_layout_sha256: str
    horizon_steps: int
    release_frequency_hz: int
    release_period_sec: Fraction
    integer_snap_tolerance_sec: float
    duration_tolerance_sec: float
    compiler_environment: tuple[tuple[str, str], ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = CODEGEN_OPTIONS_SCHEMA
    status: str = CODEGEN_OPTIONS_STATUS
    artifact_status: str = CODEGEN_ARTIFACT_STATUS
    target_performance_status: str = CODEGEN_TARGET_PERFORMANCE_STATUS
    model_id: str = MODEL_ID
    backend_contract: str = CODEGEN_BACKEND_CONTRACT
    codegen_api: str = CODEGEN_API
    ext_fun_compile_flags: str = EXT_FUN_COMPILE_FLAGS
    ext_fun_expand_constr: bool = EXT_FUN_EXPAND_CONSTR
    ext_fun_expand_cost: bool = EXT_FUN_EXPAND_COST
    ext_fun_expand_dyn: bool = EXT_FUN_EXPAND_DYN
    ext_fun_expand_precompute: bool = EXT_FUN_EXPAND_PRECOMPUTE
    custom_update_filename: str = CUSTOM_UPDATE_FILENAME
    custom_update_header_filename: str = CUSTOM_UPDATE_HEADER_FILENAME
    custom_update_copy: bool = CUSTOM_UPDATE_COPY
    custom_templates: tuple[tuple[str, ...], ...] = CUSTOM_TEMPLATES
    build_system: str = BUILD_SYSTEM
    build_target: str = BUILD_TARGET
    with_cython: bool = WITH_CYTHON
    verbose: bool = CODEGEN_VERBOSE
    cmake_builder: str = CMAKE_BUILDER
    output_directory_policy: str = OUTPUT_DIRECTORY_POLICY
    source_root_binding_policy: str = SOURCE_ROOT_BINDING_POLICY
    git_dirty_policy: str = GIT_DIRTY_POLICY
    compiler_environment_policy: str = COMPILER_ENVIRONMENT_POLICY
    generated_file_inventory_policy: str = GENERATED_FILE_INVENTORY_POLICY
    artifact_identity_policy: str = ARTIFACT_IDENTITY_POLICY
    acados_json_filename: str = ACADOS_JSON_FILENAME
    model_contract_filename: str = MODEL_CONTRACT_FILENAME
    generated_header_filename: str = GENERATED_HEADER_FILENAME

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _SNAPSHOT_TOKEN:
            raise CodegenOptionsError(
                "CodegenOptionsSnapshot requires build_codegen_options_snapshot"
            )
        if (
            self.schema_version != CODEGEN_OPTIONS_SCHEMA
            or self.status != CODEGEN_OPTIONS_STATUS
            or self.artifact_status != CODEGEN_ARTIFACT_STATUS
            or self.target_performance_status != CODEGEN_TARGET_PERFORMANCE_STATUS
            or self.model_id != MODEL_ID
        ):
            raise CodegenOptionsError("codegen option status or identity drifted")
        if (
            tuple(name for name, _ in self.compiler_environment)
            != COMPILER_ENVIRONMENT_NAMES
        ):
            raise CodegenOptionsError("compiler environment keys are not canonical")
        if any(type(value) is not str for _, value in self.compiler_environment):
            raise CodegenOptionsError("compiler environment values must be strings")
        try:
            require_sha256(
                self.development_layout_sha256,
                "codegen development layout identity",
            )
            require_sha256(self.semantic_sha256, "codegen option identity")
        except IdentityError as exc:
            raise CodegenOptionsError(str(exc)) from exc

    def to_dict(self) -> dict[str, Any]:
        """Return the canonical JSON-compatible policy document."""

        payload = _snapshot_payload(self)
        payload["semantic_identity"] = {
            "sha256": self.semantic_sha256,
            "scope": CODEGEN_OPTIONS_SCOPE,
        }
        return payload


def _snapshot_payload(snapshot: CodegenOptionsSnapshot) -> dict[str, Any]:
    return {
        "schema_version": snapshot.schema_version,
        "scope": CODEGEN_OPTIONS_SCOPE,
        "model_id": snapshot.model_id,
        "status": {
            "codegen_options": snapshot.status,
            "artifact": snapshot.artifact_status,
            "target_performance": snapshot.target_performance_status,
        },
        "source_identity": {
            "development_layout_semantic_sha256": (snapshot.development_layout_sha256),
        },
        "horizon": {
            "N": snapshot.horizon_steps,
            "release_frequency_hz": snapshot.release_frequency_hz,
            "release_period_sec": {
                "numerator": snapshot.release_period_sec.numerator,
                "denominator": snapshot.release_period_sec.denominator,
            },
        },
        "runtime_schedule_contract": {
            "schema_version": RUNTIME_SCHEDULE_SCHEMA_VERSION,
            "integer_snap_tolerance_sec": snapshot.integer_snap_tolerance_sec,
            "duration_tolerance_sec": snapshot.duration_tolerance_sec,
            "actual_schedule_values": "EXCLUDED_FROM_ARTIFACT_IDENTITY",
        },
        "acados_codegen": {
            "backend_contract": snapshot.backend_contract,
            "api": snapshot.codegen_api,
            "external_functions": {
                "compile_flags": snapshot.ext_fun_compile_flags,
                "expand_constraints": snapshot.ext_fun_expand_constr,
                "expand_cost": snapshot.ext_fun_expand_cost,
                "expand_dynamics": snapshot.ext_fun_expand_dyn,
                "expand_precompute": snapshot.ext_fun_expand_precompute,
            },
            "custom_update": {
                "filename": snapshot.custom_update_filename,
                "header_filename": snapshot.custom_update_header_filename,
                "copy": snapshot.custom_update_copy,
                "templates": [list(item) for item in snapshot.custom_templates],
            },
            "build": {
                "system": snapshot.build_system,
                "target": snapshot.build_target,
                "with_cython": snapshot.with_cython,
                "verbose": snapshot.verbose,
                "cmake_builder": snapshot.cmake_builder,
                "compiler_environment_policy": (snapshot.compiler_environment_policy),
                "compiler_environment": dict(snapshot.compiler_environment),
            },
            "output_directory_policy": snapshot.output_directory_policy,
            "source_root_binding_policy": snapshot.source_root_binding_policy,
            "git_dirty_policy": snapshot.git_dirty_policy,
            "generated_file_inventory_policy": (
                snapshot.generated_file_inventory_policy
            ),
            "artifact_identity_policy": snapshot.artifact_identity_policy,
            "filenames": {
                "acados_json": snapshot.acados_json_filename,
                "model_contract": snapshot.model_contract_filename,
                "generated_header": snapshot.generated_header_filename,
            },
        },
        "runtime_identity_exclusions": list(RUNTIME_IDENTITY_EXCLUSIONS),
    }


def build_codegen_options_snapshot(
    capacity: DevelopmentCapacityContract,
    development_layout: DevelopmentLayout,
    integer_snap_tolerance_sec: float,
    duration_tolerance_sec: float,
    ext_fun_compile_flags: str,
) -> CodegenOptionsSnapshot:
    """Freeze one D4 development policy without loading a numeric backend."""

    try:
        checked_layout = validate_development_layout_snapshot(
            development_layout,
            capacity,
        )
    except DevelopmentLayoutError as exc:
        raise CodegenOptionsError(
            "development layout does not match the pinned capacity"
        ) from exc
    if (
        checked_layout.horizon_steps != 60
        or checked_layout.release_frequency_hz != 30
        or checked_layout.release_period_sec != Fraction(1, 30)
        or checked_layout.nx != 48
        or checked_layout.nu != 3
        or checked_layout.np_exec != 121
    ):
        raise CodegenOptionsError("development layout is not the canonical D4 layout")
    try:
        layout_sha256 = sha256_json(checked_layout.to_dict())
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise CodegenOptionsError("development layout identity is malformed") from exc
    if (
        type(ext_fun_compile_flags) is not str
        or ext_fun_compile_flags != EXT_FUN_COMPILE_FLAGS
    ):
        raise CodegenOptionsError(
            f"external-function compile flags must be exactly {EXT_FUN_COMPILE_FLAGS!r}"
        )
    period_sec = float(checked_layout.release_period_sec)
    snap = _explicit_tolerance(
        integer_snap_tolerance_sec,
        "integer_snap_tolerance_sec",
        0.5 * period_sec,
    )
    duration = _explicit_tolerance(
        duration_tolerance_sec,
        "duration_tolerance_sec",
        period_sec,
    )
    values = {
        "development_layout_sha256": layout_sha256,
        "horizon_steps": checked_layout.horizon_steps,
        "release_frequency_hz": checked_layout.release_frequency_hz,
        "release_period_sec": checked_layout.release_period_sec,
        "integer_snap_tolerance_sec": snap,
        "duration_tolerance_sec": duration,
        "compiler_environment": _compiler_environment(),
    }
    provisional = CodegenOptionsSnapshot(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_SNAPSHOT_TOKEN,
    )
    try:
        semantic_sha256 = sha256_json(_snapshot_payload(provisional))
    except IdentityError as exc:
        raise CodegenOptionsError("codegen option identity cannot be encoded") from exc
    return CodegenOptionsSnapshot(
        **values,
        semantic_sha256=semantic_sha256,
        _construction_token=_SNAPSHOT_TOKEN,
    )


def require_codegen_options_snapshot(value: Any) -> CodegenOptionsSnapshot:
    """Reject wrong-type or force-mutated codegen policies before file writes."""

    if type(value) is not CodegenOptionsSnapshot:
        raise CodegenOptionsError(
            "codegen_options must be the exact CodegenOptionsSnapshot type"
        )
    try:
        actual_sha256 = sha256_json(_snapshot_payload(value))
    except (AttributeError, IdentityError, TypeError, ValueError) as exc:
        raise CodegenOptionsError("typed codegen option snapshot is malformed") from exc
    if actual_sha256 != value.semantic_sha256:
        raise CodegenOptionsError("typed codegen option identity is inconsistent")
    return value


def apply_codegen_options(target: Any, snapshot: CodegenOptionsSnapshot) -> None:
    """Apply every generation-affecting Acados solver option explicitly."""

    checked = require_codegen_options_snapshot(snapshot)
    for name in (
        "ext_fun_compile_flags",
        "ext_fun_expand_constr",
        "ext_fun_expand_cost",
        "ext_fun_expand_dyn",
        "ext_fun_expand_precompute",
        "custom_update_filename",
        "custom_update_header_filename",
        "custom_update_copy",
    ):
        setattr(target, name, getattr(checked, name))
    target.custom_templates = [tuple(item) for item in checked.custom_templates]


__all__ = [
    "ACADOS_JSON_FILENAME",
    "ARTIFACT_IDENTITY_POLICY",
    "BUILD_SYSTEM",
    "BUILD_TARGET",
    "CODEGEN_API",
    "CODEGEN_ARTIFACT_STATUS",
    "CODEGEN_BACKEND_CONTRACT",
    "CODEGEN_OPTIONS_SCHEMA",
    "CODEGEN_OPTIONS_SCOPE",
    "CODEGEN_OPTIONS_STATUS",
    "CODEGEN_TARGET_PERFORMANCE_STATUS",
    "COMPILER_ENVIRONMENT_NAMES",
    "EXT_FUN_COMPILE_FLAGS",
    "GENERATED_FILE_INVENTORY_POLICY",
    "GENERATED_HEADER_FILENAME",
    "MODEL_CONTRACT_FILENAME",
    "OUTPUT_DIRECTORY_POLICY",
    "RUNTIME_IDENTITY_EXCLUSIONS",
    "SOURCE_ROOT_BINDING_POLICY",
    "CodegenOptionsError",
    "CodegenOptionsSnapshot",
    "apply_codegen_options",
    "build_codegen_options_snapshot",
    "require_codegen_options_snapshot",
]
