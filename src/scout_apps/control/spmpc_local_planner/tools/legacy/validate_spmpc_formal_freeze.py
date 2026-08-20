#!/usr/bin/env python3
"""Fail-closed, read-only validator for an S-MPCC formal freeze manifest."""

from __future__ import annotations

import argparse
import hashlib
import math
import re
import stat
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on an invalid host setup.
    print(
        "ERROR: PyYAML is required to validate the formal freeze manifest: " + str(exc),
        file=sys.stderr,
    )
    raise SystemExit(2)


EXPECTED_PROTOCOL_ID = "SMPCC-REAL-40-88-v1.0"
EXPECTED_EXPERIMENTAL_DESIGN_VERSION = "v1.2"
EXPECTED_K6_PROTOCOL_ID = "K6-FID-v1.0"
EXPECTED_NOMINAL_V_REF = 0.20
EXPECTED_W_SLOSH_CANDIDATES: Tuple[float, ...] = (1.0, 2.0, 5.0)

REQUIRED_GATES: Tuple[str, ...] = (
    "parameter_pilot_pass",
    "smooth_match_pilot_pass",
    "h0_h1_l1_freeze_pass",
    "c1_c2_freeze_pass",
    "recorder_warm_start_smoke_pass",
    "actual_zero_replay_smoke_pass",
    "nominal_replay_reproduction_pass",
    "visual_sync_smoke_pass",
    "k6_fid_v1_0_no_go_check_pass",
    "all_formal_prerequisites_pass",
)

# Every GO manifest must name and authenticate these protocol-critical files.
# Empty placeholders are never accepted, even when the selected trial would not
# consume that artifact directly (for example, a C1 trial still requires the
# already-frozen C2 config and all pre-registered order tables).
REQUIRED_ARTIFACT_PAIRS: Tuple[Tuple[str, str, str], ...] = (
    ("upstream_protocols", "experimental_design_file", "experimental_design_sha256"),
    ("upstream_protocols", "matrix_index_file", "matrix_index_sha256"),
    ("upstream_protocols", "k6_protocol_file", "k6_protocol_sha256"),
    ("upstream_protocols", "field_matrix_file", "field_matrix_sha256"),
    ("upstream_protocols", "field_commands_file", "field_commands_sha256"),
    ("method_release", "dynamics_source", "dynamics_sha256"),
    ("method_release", "state_propagation_source", "state_propagation_sha256"),
    ("method_release", "cost_structure_source", "cost_structure_sha256"),
    ("software", "build_log", "build_log_sha256"),
    ("methods", "baseline_config", "baseline_sha256"),
    ("methods", "smooth_config", "smooth_sha256"),
    ("methods", "spmpc_config", "spmpc_sha256"),
    ("methods", "smooth_match_config", "smooth_match_sha256"),
    (
        "pilot_evidence",
        "standalone_monitor_config",
        "standalone_monitor_config_sha256",
    ),
    (
        "pilot_evidence",
        "delta_model_weight_decision_file",
        "delta_model_weight_decision_file_sha256",
    ),
    (
        "pilot_evidence",
        "p3a_endpoint_acceptance_report",
        "p3a_endpoint_acceptance_report_sha256",
    ),
    (
        "pilot_evidence",
        "p4_completion_match_report",
        "p4_completion_match_report_sha256",
    ),
    ("paths", "h0_file", "h0_sha256"),
    ("paths", "h1_file", "h1_sha256"),
    ("paths", "l1_file", "l1_sha256"),
    ("paths", "geometry_summary_file", "geometry_summary_sha256"),
    ("containers", "c1_config_file", "c1_config_sha256"),
    ("containers", "c2_config_file", "c2_config_sha256"),
    ("vision_and_sync", "camera_config_file", "camera_config_sha256"),
    ("vision_and_sync", "calibration_file", "calibration_sha256"),
    ("vision_and_sync", "k6_fidelity_manifest", "k6_fidelity_manifest_sha256"),
    ("vision_and_sync", "k6_protocol_smoke_report", "k6_protocol_smoke_report_sha256"),
    ("vision_and_sync", "nominal_replay_report", "nominal_replay_report_sha256"),
    ("randomization", "parameter_pilot_file", "parameter_pilot_sha256"),
    ("randomization", "smooth_match_pilot_file", "smooth_match_pilot_sha256"),
    ("randomization", "s1_order_file", "s1_order_sha256"),
    ("randomization", "e1_order_file", "e1_order_sha256"),
    ("randomization", "e2_c2_order_file", "e2_c2_order_sha256"),
    ("analysis_tools", "rgb_script", "rgb_script_sha256"),
    ("analysis_tools", "actual_zero_replay_script", "actual_zero_replay_script_sha256"),
    ("analysis_tools", "k6_script", "k6_script_sha256"),
    ("analysis_tools", "runtime_script", "runtime_script_sha256"),
    (
        "execution_smoke",
        "recorder_warm_start_report",
        "recorder_warm_start_report_sha256",
    ),
    (
        "execution_smoke",
        "actual_zero_replay_report",
        "actual_zero_replay_report_sha256",
    ),
    ("manifest_validation", "validator_script", "validator_script_sha256"),
)

METHOD_ALIASES = {
    "B0": "B0",
    "Baseline": "B0",
    "Bsmooth": "Bsmooth",
    "B_smooth": "Bsmooth",
    "Bslosh": "Bslosh",
    "B_slosh": "Bslosh",
    "SmoothMatch": "SmoothMatch",
    "Smooth-match": "SmoothMatch",
}

EXPECTED_VARIANTS = {
    "B0": "B0",
    "Bsmooth": "B_smooth",
    "Bslosh": "B_slosh",
    "SmoothMatch": "B_smooth",
}

LEGAL_FORMAL_COMBINATIONS = {
    ("S1", "E2"): ("H1", "C1", frozenset({"B0", "Bsmooth", "Bslosh"})),
    ("S1", "E3"): ("H1", "C1", frozenset({"Bslosh", "SmoothMatch"})),
    ("S2A", "E1"): ("L1", "C1", frozenset({"B0", "Bsmooth", "Bslosh"})),
    ("S2B", "E2"): ("H1", "C2", frozenset({"B0", "Bsmooth", "Bslosh"})),
}

ARTIFACT_PATH_SUFFIXES = (
    "_file",
    "_path",
    "_script",
    "_log",
    "_manifest",
    "_source",
)
KNOWN_PATH_EXTENSIONS = {
    ".bag",
    ".csv",
    ".json",
    ".launch",
    ".log",
    ".md",
    ".py",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
FREEZE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{2,127}$")


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate keys instead of silently replacing them."""


def _construct_unique_mapping(
    loader: UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False
) -> MutableMapping[Any, Any]:
    mapping: MutableMapping[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True)
class ValidationResult:
    freeze_id: str
    t_settle_sec: float
    method: str
    variant: str
    v_ref: float
    w_slosh: float
    smooth_match_v_ref: float
    smooth_match_safe_v_ref_min: float
    smooth_match_safe_v_ref_max: float
    stage: str
    group: str


class FreezeValidationError(Exception):
    def __init__(self, errors: Sequence[str]):
        self.errors = list(errors)
        super().__init__("; ".join(self.errors))


class Validator:
    def __init__(self) -> None:
        self.errors: List[str] = []
        self._validated_artifacts: set[Tuple[Path, str]] = set()

    def error(self, message: str) -> None:
        self.errors.append(message)

    def require_mapping(self, value: Any, label: str) -> Mapping[str, Any]:
        if not isinstance(value, Mapping):
            self.error(f"{label} must be a YAML mapping")
            return {}
        if not all(isinstance(key, str) for key in value):
            self.error(f"{label} must use string keys")
        return value  # type: ignore[return-value]

    def require_string(self, value: Any, label: str) -> str:
        if not isinstance(value, str) or not value.strip():
            self.error(f"{label} must be a non-empty string")
            return ""
        return value.strip()

    def require_finite_number(self, value: Any, label: str, *, positive: bool = False) -> float:
        if isinstance(value, bool):
            self.error(f"{label} must be a finite number, not a boolean")
            return math.nan
        try:
            number = float(value)
        except (TypeError, ValueError):
            self.error(f"{label} must be a finite number")
            return math.nan
        if not math.isfinite(number):
            self.error(f"{label} must be finite")
            return math.nan
        if positive and number <= 0.0:
            self.error(f"{label} must be greater than zero")
        return number

    def require_equal(self, actual: Any, expected: Any, label: str) -> None:
        if actual != expected:
            self.error(f"{label} must equal {expected!r}, got {actual!r}")

    def require_float_equal(self, actual: float, expected: float, label: str) -> None:
        if not (math.isfinite(actual) and math.isfinite(expected)):
            return
        if not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-12):
            self.error(f"{label} mismatch: manifest={expected:.12g}, runtime={actual:.12g}")

    def validate_artifact(
        self, path_value: str, digest_value: Any, repo_root: Path, label: str
    ) -> None:
        digest = self.require_string(digest_value, f"{label}.sha256")
        if digest and not SHA256_RE.fullmatch(digest):
            self.error(f"{label}.sha256 must be exactly 64 hexadecimal characters")
            return

        artifact_path = Path(path_value).expanduser()
        if not artifact_path.is_absolute():
            artifact_path = repo_root / artifact_path
        artifact_path = artifact_path.resolve()
        identity = (artifact_path, digest.lower())
        if identity in self._validated_artifacts:
            return
        self._validated_artifacts.add(identity)

        if not artifact_path.is_file():
            self.error(f"{label} artifact does not exist or is not a file: {artifact_path}")
            return
        actual_digest = sha256_file(artifact_path)
        if digest and actual_digest != digest.lower():
            self.error(
                f"{label} sha256 mismatch for {artifact_path}: "
                f"manifest={digest.lower()}, actual={actual_digest}"
            )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        raise FreezeValidationError([f"manifest does not exist or is not a file: {path}"])
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle, Loader=UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise FreezeValidationError([f"cannot read manifest {path}: {exc}"]) from exc
    if not isinstance(data, Mapping):
        raise FreezeValidationError(["manifest root must be a YAML mapping"])
    return data  # type: ignore[return-value]


def _run_git(repo_root: Path, args: Sequence[str]) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        raise FreezeValidationError([f"git {' '.join(args)} failed: {detail.strip()}"]) from exc
    return completed.stdout.strip()


def _resolve_artifact(path_value: str, repo_root: Path) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = repo_root / path
    return path.resolve()


def _hash_key_candidates(key: str) -> Iterable[str]:
    yield f"{key}_sha256"
    for suffix in (
        "_file",
        "_path",
        "_script",
        "_log",
        "_manifest",
        "_source",
        "_config",
    ):
        if key.endswith(suffix):
            yield f"{key[:-len(suffix)]}_sha256"


def _looks_like_artifact_path(key: str, value: str) -> bool:
    if key.endswith(ARTIFACT_PATH_SUFFIXES):
        return True
    if key.endswith("_config"):
        return "/" in value or Path(value).suffix.lower() in KNOWN_PATH_EXTENSIONS
    return False


def validate_declared_artifacts(
    validator: Validator,
    mapping: Mapping[str, Any],
    repo_root: Path,
    prefix: str = "manifest",
) -> None:
    for key, value in mapping.items():
        label = f"{prefix}.{key}"
        if isinstance(value, Mapping):
            validate_declared_artifacts(validator, value, repo_root, label)
            continue
        if not isinstance(value, str) or not value.strip() or not _looks_like_artifact_path(key, value):
            continue
        digest_key = next((candidate for candidate in _hash_key_candidates(key) if candidate in mapping), None)
        if digest_key is None:
            validator.error(f"{label} declares an artifact path but has no sibling sha256 field")
            continue
        validator.validate_artifact(value.strip(), mapping[digest_key], repo_root, label)


def validate_required_artifacts(
    validator: Validator, manifest: Mapping[str, Any], repo_root: Path
) -> None:
    for section_name, path_key, digest_key in REQUIRED_ARTIFACT_PAIRS:
        section = validator.require_mapping(manifest.get(section_name), section_name)
        path_value = validator.require_string(
            section.get(path_key), f"{section_name}.{path_key}"
        )
        digest_value = section.get(digest_key)
        if path_value:
            validator.validate_artifact(
                path_value,
                digest_value,
                repo_root,
                f"{section_name}.{path_key}",
            )


def validate_validator_identity(
    validator: Validator, manifest: Mapping[str, Any], repo_root: Path
) -> None:
    section = validator.require_mapping(
        manifest.get("manifest_validation"), "manifest_validation"
    )
    declared = validator.require_string(
        section.get("validator_script"), "manifest_validation.validator_script"
    )
    if not declared:
        return
    declared_path = _resolve_artifact(declared, repo_root)
    running_path = Path(__file__).resolve()
    if declared_path != running_path:
        validator.error(
            "the running validator does not match manifest_validation.validator_script: "
            f"manifest={declared_path}, running={running_path}"
        )


def validate_git(
    validator: Validator, repo_root: Path, software: Mapping[str, Any]
) -> None:
    try:
        top_level = Path(_run_git(repo_root, ["rev-parse", "--show-toplevel"])).resolve()
        head = _run_git(repo_root, ["rev-parse", "HEAD"])
        status = _run_git(repo_root, ["status", "--porcelain", "--untracked-files=normal"])
    except FreezeValidationError as exc:
        validator.errors.extend(exc.errors)
        return

    if top_level != repo_root.resolve():
        validator.error(f"repo-root must be the Git top-level: expected {top_level}, got {repo_root.resolve()}")
    declared_revision = validator.require_string(software.get("git_revision"), "software.git_revision")
    if declared_revision and head != declared_revision:
        validator.error(f"Git HEAD mismatch: manifest={declared_revision}, actual={head}")
    if software.get("git_clean") is not True:
        validator.error("software.git_clean must be boolean true")
    if status:
        preview = " | ".join(status.splitlines()[:5])
        validator.error(f"Git worktree is not clean: {preview}")


def validate_method(
    validator: Validator,
    methods: Mapping[str, Any],
    method: str,
    variant: str,
    v_ref: float,
    w_slosh: float,
) -> Tuple[str, float, float, float]:
    canonical_method = METHOD_ALIASES.get(method)
    if canonical_method is None:
        validator.error(
            "method must be one of B0, Bsmooth, Bslosh, SmoothMatch "
            f"(got {method!r})"
        )
        return "", math.nan, math.nan, math.nan

    expected_variant = EXPECTED_VARIANTS[canonical_method]
    validator.require_equal(variant, expected_variant, "runtime variant")

    final_w_slosh = validator.require_finite_number(
        methods.get("final_w_slosh"), "methods.final_w_slosh", positive=True
    )
    if math.isfinite(final_w_slosh) and not any(
        math.isclose(final_w_slosh, candidate, rel_tol=1e-9, abs_tol=1e-12)
        for candidate in EXPECTED_W_SLOSH_CANDIDATES
    ):
        validator.error(
            "methods.final_w_slosh must equal one of the frozen v1.0 candidates "
            "{1, 2, 5}"
        )
    smooth_match_v_ref = validator.require_finite_number(
        methods.get("smooth_match_v_ref"), "methods.smooth_match_v_ref", positive=True
    )
    safe_min = validator.require_finite_number(
        methods.get("smooth_match_safe_v_ref_min"),
        "methods.smooth_match_safe_v_ref_min",
        positive=True,
    )
    safe_max = validator.require_finite_number(
        methods.get("smooth_match_safe_v_ref_max"),
        "methods.smooth_match_safe_v_ref_max",
        positive=True,
    )
    if math.isfinite(safe_min) and math.isfinite(safe_max) and safe_min >= safe_max:
        validator.error("methods.smooth_match_safe_v_ref_min must be strictly less than safe_v_ref_max")
    if (
        math.isfinite(smooth_match_v_ref)
        and math.isfinite(safe_min)
        and math.isfinite(safe_max)
        and not (safe_min <= smooth_match_v_ref <= safe_max)
    ):
        validator.error("methods.smooth_match_v_ref lies outside its frozen safe interval")

    if canonical_method == "SmoothMatch":
        validator.require_float_equal(v_ref, smooth_match_v_ref, "runtime v_ref")
        expected_w_slosh = 0.0
    else:
        validator.require_float_equal(v_ref, EXPECTED_NOMINAL_V_REF, "runtime v_ref")
        expected_w_slosh = final_w_slosh if canonical_method == "Bslosh" else 0.0
    validator.require_float_equal(w_slosh, expected_w_slosh, "runtime w_slosh")
    return canonical_method, smooth_match_v_ref, safe_min, safe_max


def validate_all_container_contracts(
    validator: Validator, containers: Mapping[str, Any], repo_root: Path
) -> None:
    for prefix in ("c1", "c2"):
        config_name = validator.require_string(
            containers.get(f"{prefix}_config"), f"containers.{prefix}_config"
        )
        config_file = validator.require_string(
            containers.get(f"{prefix}_config_file"), f"containers.{prefix}_config_file"
        )
        radius = validator.require_finite_number(
            containers.get(f"{prefix}_radius_m"),
            f"containers.{prefix}_radius_m",
            positive=True,
        )
        height = validator.require_finite_number(
            containers.get(f"{prefix}_liquid_height_m"),
            f"containers.{prefix}_liquid_height_m",
            positive=True,
        )
        damping = validator.require_finite_number(
            containers.get(f"{prefix}_damping_ratio"),
            f"containers.{prefix}_damping_ratio",
            positive=True,
        )
        validator.require_finite_number(
            containers.get(f"{prefix}_freeboard_m"),
            f"containers.{prefix}_freeboard_m",
            positive=True,
        )
        validator.require_finite_number(
            containers.get(f"{prefix}_f1_hz"),
            f"containers.{prefix}_f1_hz",
            positive=True,
        )
        frames_per_cycle = validator.require_finite_number(
            containers.get(f"{prefix}_camera_frames_per_cycle"),
            f"containers.{prefix}_camera_frames_per_cycle",
            positive=True,
        )
        if math.isfinite(frames_per_cycle) and frames_per_cycle < 6.0:
            validator.error(f"containers.{prefix}_camera_frames_per_cycle must be at least 6")

        if not config_file:
            continue
        config_path = _resolve_artifact(config_file, repo_root)
        if config_name and config_path.stem != config_name:
            validator.error(
                f"containers.{prefix}_config does not match config file stem: "
                f"name={config_name!r}, file={config_path}"
            )
        try:
            data = load_yaml(config_path)
        except FreezeValidationError as exc:
            validator.errors.extend(
                f"containers.{prefix}_config_file: {message}" for message in exc.errors
            )
            continue
        slosh = validator.require_mapping(data.get("slosh"), f"containers.{prefix} YAML slosh")
        yaml_radius = validator.require_finite_number(
            slosh.get("container_radius"),
            f"containers.{prefix} YAML slosh.container_radius",
            positive=True,
        )
        yaml_height = validator.require_finite_number(
            slosh.get("liquid_height"),
            f"containers.{prefix} YAML slosh.liquid_height",
            positive=True,
        )
        yaml_damping = validator.require_finite_number(
            slosh.get("damping_ratio"),
            f"containers.{prefix} YAML slosh.damping_ratio",
            positive=True,
        )
        validator.require_float_equal(yaml_radius, radius, f"containers.{prefix} radius")
        validator.require_float_equal(yaml_height, height, f"containers.{prefix} liquid height")
        validator.require_float_equal(yaml_damping, damping, f"containers.{prefix} damping ratio")

    validator.require_finite_number(
        containers.get("lambda_h"), "containers.lambda_h", positive=True
    )


def validate_path(
    validator: Validator,
    paths: Mapping[str, Any],
    repo_root: Path,
    path_id: str,
    path_file: Path,
) -> None:
    canonical_id = path_id.upper()
    if canonical_id not in {"H0", "H1", "L1"}:
        validator.error(f"path-id must be H0, H1, or L1 (got {path_id!r})")
        return
    prefix = canonical_id.lower()
    declared = validator.require_string(paths.get(f"{prefix}_file"), f"paths.{prefix}_file")
    digest = paths.get(f"{prefix}_sha256")
    if not declared:
        return
    declared_path = _resolve_artifact(declared, repo_root)
    runtime_path = path_file.expanduser().resolve()
    if declared_path != runtime_path:
        validator.error(f"runtime path-file mismatch: manifest={declared_path}, runtime={runtime_path}")
    validator.validate_artifact(declared, digest, repo_root, f"paths.{prefix}_file")


def validate_formal_combination(
    validator: Validator,
    stage: str,
    group: str,
    path_id: str,
    container_id: str,
    canonical_method: str,
) -> None:
    key = (stage, group)
    contract = LEGAL_FORMAL_COMBINATIONS.get(key)
    if contract is None:
        allowed = ", ".join(f"{item[0]}/{item[1]}" for item in LEGAL_FORMAL_COMBINATIONS)
        validator.error(f"illegal formal stage/group {stage}/{group}; allowed: {allowed}")
        return
    expected_path, expected_container, allowed_methods = contract
    if path_id.upper() != expected_path:
        validator.error(
            f"illegal path for {stage}/{group}: expected {expected_path}, got {path_id}"
        )
    if container_id.upper() != expected_container:
        validator.error(
            f"illegal container for {stage}/{group}: expected {expected_container}, got {container_id}"
        )
    if canonical_method not in allowed_methods:
        validator.error(
            f"illegal method for {stage}/{group}: expected one of "
            f"{','.join(sorted(allowed_methods))}, got {canonical_method or '<invalid>'}"
        )


def validate_container(
    validator: Validator,
    containers: Mapping[str, Any],
    repo_root: Path,
    container_id: str,
    container_config: str,
    container_yaml: Path,
    radius: float,
    liquid_height: float,
    damping_ratio: float,
) -> None:
    canonical_id = container_id.upper()
    if canonical_id not in {"C1", "C2"}:
        validator.error(f"container-id must be C1 or C2 (got {container_id!r})")
        return
    prefix = canonical_id.lower()
    declared_config = validator.require_string(
        containers.get(f"{prefix}_config"), f"containers.{prefix}_config"
    )
    if declared_config:
        validator.require_equal(container_config, declared_config, "runtime container-config")

    runtime_yaml = container_yaml.expanduser().resolve()
    declared_file = containers.get(f"{prefix}_config_file")
    if declared_file not in (None, ""):
        declared_file_str = validator.require_string(
            declared_file, f"containers.{prefix}_config_file"
        )
        if declared_file_str and _resolve_artifact(declared_file_str, repo_root) != runtime_yaml:
            validator.error(
                "runtime container-yaml mismatch: "
                f"manifest={_resolve_artifact(declared_file_str, repo_root)}, runtime={runtime_yaml}"
            )

    validator.validate_artifact(
        str(runtime_yaml),
        containers.get(f"{prefix}_config_sha256"),
        repo_root,
        f"containers.{prefix}_config",
    )

    declared_radius = validator.require_finite_number(
        containers.get(f"{prefix}_radius_m"), f"containers.{prefix}_radius_m", positive=True
    )
    declared_height = validator.require_finite_number(
        containers.get(f"{prefix}_liquid_height_m"),
        f"containers.{prefix}_liquid_height_m",
        positive=True,
    )
    declared_damping_value = containers.get(f"{prefix}_damping_ratio")
    declared_damping = (
        validator.require_finite_number(
            declared_damping_value,
            f"containers.{prefix}_damping_ratio",
            positive=True,
        )
        if declared_damping_value is not None
        else math.nan
    )
    validator.require_float_equal(radius, declared_radius, "runtime container radius")
    validator.require_float_equal(liquid_height, declared_height, "runtime liquid height")
    if math.isfinite(declared_damping):
        validator.require_float_equal(damping_ratio, declared_damping, "runtime damping ratio")

    try:
        container_data = load_yaml(runtime_yaml)
    except FreezeValidationError as exc:
        validator.errors.extend(f"container YAML: {message}" for message in exc.errors)
        return
    slosh = validator.require_mapping(container_data.get("slosh"), "container YAML slosh")
    yaml_radius = validator.require_finite_number(
        slosh.get("container_radius"), "container YAML slosh.container_radius", positive=True
    )
    yaml_height = validator.require_finite_number(
        slosh.get("liquid_height"), "container YAML slosh.liquid_height", positive=True
    )
    yaml_damping = validator.require_finite_number(
        slosh.get("damping_ratio"), "container YAML slosh.damping_ratio", positive=True
    )
    validator.require_float_equal(radius, yaml_radius, "container YAML radius")
    validator.require_float_equal(liquid_height, yaml_height, "container YAML liquid height")
    validator.require_float_equal(damping_ratio, yaml_damping, "container YAML damping ratio")


def validate_freeze(args: argparse.Namespace) -> ValidationResult:
    manifest_path = Path(args.manifest).expanduser().resolve()
    repo_root = Path(args.repo_root).expanduser().resolve()
    manifest = load_yaml(manifest_path)
    validator = Validator()

    try:
        manifest_mode = manifest_path.stat().st_mode
    except OSError as exc:
        validator.error(f"cannot stat manifest {manifest_path}: {exc}")
    else:
        write_mask = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH
        if manifest_mode & write_mask:
            validator.error(
                "formal manifest must be read-only (no owner/group/other write bits); "
                f"run chmod a-w {manifest_path}"
            )

    validator.require_equal(manifest.get("protocol_id"), EXPECTED_PROTOCOL_ID, "protocol_id")
    validator.require_equal(
        manifest.get("experimental_design_version"),
        EXPECTED_EXPERIMENTAL_DESIGN_VERSION,
        "experimental_design_version",
    )
    validator.require_equal(
        manifest.get("k6_protocol_id"), EXPECTED_K6_PROTOCOL_ID, "k6_protocol_id"
    )
    validator.require_equal(manifest.get("status"), "GO", "status")
    if manifest.get("e4_enabled") is not False:
        validator.error("e4_enabled must be boolean false")

    freeze_id = validator.require_string(manifest.get("freeze_id"), "freeze_id")
    if freeze_id and not FREEZE_ID_RE.fullmatch(freeze_id):
        validator.error(
            "freeze_id must contain 3-128 characters using only letters, digits, '.', '_', or '-'"
        )
    t_settle = validator.require_finite_number(
        manifest.get("t_settle_sec"), "t_settle_sec", positive=True
    )

    method_release = validator.require_mapping(manifest.get("method_release"), "method_release")
    if method_release.get("rotation_consistent_enabled") is not False:
        validator.error("method_release.rotation_consistent_enabled must be boolean false")
    if method_release.get("signed_power_enabled") is not False:
        validator.error("method_release.signed_power_enabled must be boolean false")
    if method_release.get("phase_energy_cost_enabled") is not False:
        validator.error("method_release.phase_energy_cost_enabled must be boolean false")
    validator.require_string(method_release.get("release_id"), "method_release.release_id")

    gates = validator.require_mapping(manifest.get("gates"), "gates")
    for gate in REQUIRED_GATES:
        if gates.get(gate) is not True:
            validator.error(f"gates.{gate} must be boolean true")
    for gate, value in gates.items():
        if value is not True:
            validator.error(f"all declared gates must be true; gates.{gate}={value!r}")

    software = validator.require_mapping(manifest.get("software"), "software")
    methods = validator.require_mapping(manifest.get("methods"), "methods")
    paths = validator.require_mapping(manifest.get("paths"), "paths")
    containers = validator.require_mapping(manifest.get("containers"), "containers")

    validator.require_string(software.get("acados_version"), "software.acados_version")
    codegen_hash = validator.require_string(software.get("codegen_hash"), "software.codegen_hash")
    if codegen_hash and not SHA256_RE.fullmatch(codegen_hash):
        validator.error("software.codegen_hash must be exactly 64 hexadecimal characters")

    vision_and_sync = validator.require_mapping(
        manifest.get("vision_and_sync"), "vision_and_sync"
    )
    validator.require_string(
        vision_and_sync.get("camera_serial"), "vision_and_sync.camera_serial"
    )
    validator.require_finite_number(
        vision_and_sync.get("tau_cal_sec"), "vision_and_sync.tau_cal_sec"
    )

    runtime_rules = validator.require_mapping(manifest.get("runtime_rules"), "runtime_rules")
    solve_budget = validator.require_finite_number(
        runtime_rules.get("solve_budget_ms"), "runtime_rules.solve_budget_ms", positive=True
    )
    validator.require_float_equal(solve_budget, 33.3333333, "runtime_rules.solve_budget_ms")
    validator.require_equal(
        runtime_rules.get("solve_budget_metric"),
        "solver_time_ms_overrun_rate",
        "runtime_rules.solve_budget_metric",
    )
    validator.require_equal(
        runtime_rules.get("interarrival_metric"),
        "observed_command_intervention_inter_arrival_gap_rate",
        "runtime_rules.interarrival_metric",
    )
    validator.require_finite_number(
        runtime_rules.get("interarrival_gap_threshold_sec"),
        "runtime_rules.interarrival_gap_threshold_sec",
        positive=True,
    )
    validator.require_string(
        runtime_rules.get("interarrival_window"), "runtime_rules.interarrival_window"
    )
    validator.require_string(
        runtime_rules.get("interarrival_denominator"),
        "runtime_rules.interarrival_denominator",
    )
    if runtime_rules.get("strict_control_cycle_deadline_claim_enabled") is not False:
        validator.error(
            "runtime_rules.strict_control_cycle_deadline_claim_enabled must be boolean false"
        )

    runtime_v_ref = validator.require_finite_number(
        args.v_ref, "runtime v_ref", positive=True
    )
    runtime_w_slosh = validator.require_finite_number(args.w_slosh, "runtime w_slosh")
    if math.isfinite(runtime_w_slosh) and runtime_w_slosh < 0.0:
        validator.error("runtime w_slosh must be non-negative")
    runtime_radius = validator.require_finite_number(
        args.container_radius, "runtime container radius", positive=True
    )
    runtime_height = validator.require_finite_number(
        args.liquid_height, "runtime liquid height", positive=True
    )
    runtime_damping = validator.require_finite_number(
        args.damping_ratio, "runtime damping ratio", positive=True
    )

    validate_git(validator, repo_root, software)
    validate_required_artifacts(validator, manifest, repo_root)
    validate_declared_artifacts(validator, manifest, repo_root)
    validate_validator_identity(validator, manifest, repo_root)
    validate_all_container_contracts(validator, containers, repo_root)
    canonical_method, smooth_match_v_ref, safe_min, safe_max = validate_method(
        validator,
        methods,
        args.method,
        args.variant,
        runtime_v_ref,
        runtime_w_slosh,
    )
    validate_formal_combination(
        validator,
        args.stage,
        args.group,
        args.path_id,
        args.container_id,
        canonical_method,
    )
    validate_path(
        validator,
        paths,
        repo_root,
        args.path_id,
        Path(args.path_file),
    )
    validate_container(
        validator,
        containers,
        repo_root,
        args.container_id,
        args.container_config,
        Path(args.container_yaml),
        runtime_radius,
        runtime_height,
        runtime_damping,
    )

    if validator.errors:
        raise FreezeValidationError(validator.errors)
    return ValidationResult(
        freeze_id=freeze_id,
        t_settle_sec=t_settle,
        method=canonical_method,
        variant=args.variant,
        v_ref=runtime_v_ref,
        w_slosh=runtime_w_slosh,
        smooth_match_v_ref=smooth_match_v_ref,
        smooth_match_safe_v_ref_min=safe_min,
        smooth_match_safe_v_ref_max=safe_max,
        stage=args.stage,
        group=args.group,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate a read-only S-MPCC formal freeze manifest against this runtime."
    )
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--group", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument("--variant", required=True)
    parser.add_argument("--v-ref", required=True, type=float)
    parser.add_argument("--w-slosh", required=True, type=float)
    parser.add_argument("--path-id", required=True)
    parser.add_argument("--path-file", required=True)
    parser.add_argument("--container-id", required=True)
    parser.add_argument("--container-config", required=True)
    parser.add_argument("--container-yaml", required=True)
    parser.add_argument("--container-radius", required=True, type=float)
    parser.add_argument("--liquid-height", required=True, type=float)
    parser.add_argument("--damping-ratio", required=True, type=float)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = validate_freeze(args)
    except FreezeValidationError as exc:
        print("FORMAL_FREEZE_VALIDATION=FAIL", file=sys.stderr)
        for error in exc.errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print("FORMAL_FREEZE_VALIDATION=PASS")
    print(f"FREEZE_ID={result.freeze_id}")
    print(f"T_SETTLE={result.t_settle_sec:.12g}")
    print(f"STAGE={result.stage}")
    print(f"GROUP={result.group}")
    print(f"METHOD={result.method}")
    print(f"VARIANT={result.variant}")
    print(f"V_REF={result.v_ref:.12g}")
    print(f"W_SLOSH={result.w_slosh:.12g}")
    print(f"SMOOTH_MATCH_V_REF={result.smooth_match_v_ref:.12g}")
    print(
        "SMOOTH_MATCH_SAFE_V_REF_MIN="
        f"{result.smooth_match_safe_v_ref_min:.12g}"
    )
    print(
        "SMOOTH_MATCH_SAFE_V_REF_MAX="
        f"{result.smooth_match_safe_v_ref_max:.12g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
