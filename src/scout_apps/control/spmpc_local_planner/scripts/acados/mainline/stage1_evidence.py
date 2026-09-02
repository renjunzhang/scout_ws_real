"""Development-only Stage 1 evidence registry and fail-closed gate.

The measured bags named here are useful development evidence, but they do not
form the dedicated validation/final-test authority required to freeze ``L_max``
or generate a solver.  This module can only return a blocked evidence
reference; it intentionally exposes no freezer and no production-layout API.
"""

from __future__ import annotations

import math
import re
from dataclasses import InitVar, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .contract_source import (
    STAGE0_CONTRACT_SHA256,
    STAGE3_PROHIBITED_STATUS,
    ContractSourceError,
    load_stage0_contract_reference,
)
from .identity import IdentityError, read_strict_json, require_sha256, sha256_bytes

STAGE1_EVIDENCE_SCHEMA_VERSION = "spmpc_mainline_stage1_development_evidence_v1"
STAGE1_EVIDENCE_ID = "SPMPC-MAINLINE-STAGE1-DEVELOPMENT-20260902"
STAGE1_EVIDENCE_SHA256 = (
    "b18d3b5bc7c7782faf6b5e9262e2f8fa13e360e90af39cc58976d613f4644210"
)
STAGE1_EVIDENCE_STATUS = "DEVELOPMENT_EVIDENCE_ONLY_STAGE1_NOT_FROZEN"
STAGE1_BLOCKED_STATUS = "BLOCKED_MISSING_DEDICATED_VALIDATION_AND_FINAL_TEST"
DATASET_GATE_STATUS = "INCOMPLETE_DEVELOPMENT_ONLY"
FIT_IDENTIFIABILITY_STATUS = "INCONCLUSIVE_SEARCH_BOUNDARY_AND_WIDE_NEAR_OPTIMUM"
LMAX_STATUS = "UNFROZEN_NO_PHYSICAL_MARGIN_EVIDENCE"
EXECUTION_PARAMETERS_STATUS = "UNFROZEN_DEDICATED_IDENTIFICATION_REQUIRED"
EXTERNAL_AVAILABILITY = "EXTERNAL_BYTES_NOT_AVAILABLE_ON_THIS_HOST"

SOURCE_DOCUMENT_ROLES = (
    "ACTUATOR_DEVELOPMENT_ANALYSIS",
    "SLOSH_OBSERVER_DEVELOPMENT_ANALYSIS",
)
EVIDENCE_KIND_TO_SOURCE_ROLE = {
    "ACTUATOR_CLOSED_LOOP_PATH_RESPONSE": "ACTUATOR_DEVELOPMENT_ANALYSIS",
    "SLOSH_OBSERVER_SHADOW_COMPARISON": "SLOSH_OBSERVER_DEVELOPMENT_ANALYSIS",
}
REQUIRED_MISSING_AUTHORITIES = (
    "dedicated_open_loop_identification_trials",
    "verified_bag_paths_bytes_and_motion_windows",
    "actuator_validation_partition",
    "actuator_final_test_partition",
    "frozen_delay_tau_gain",
    "frozen_lmax_and_queue_dimensions",
    "frozen_acceleration_and_jerk_ranges",
    "validation_and_final_test_gate_hash",
)

_TRIAL_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_REFERENCE_CONSTRUCTION_TOKEN = object()


class Stage1EvidenceError(ValueError):
    """Raised when development evidence drifts or claims frozen authority."""


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise Stage1EvidenceError(f"{label} must be a JSON object")
    if set(value) != keys:
        raise Stage1EvidenceError(f"{label} keys do not match the v1 schema")
    return value


def _array(value: Any, label: str) -> list[Any]:
    if type(value) is not list:
        raise Stage1EvidenceError(f"{label} must be a JSON array")
    return value


def _string(value: Any, label: str) -> str:
    if type(value) is not str or not value:
        raise Stage1EvidenceError(f"{label} must be a non-empty string")
    return value


def _strict_int(value: Any, label: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise Stage1EvidenceError(f"{label} must be an integer >= {minimum}")
    return value


def _finite_number(
    value: Any,
    label: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Stage1EvidenceError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise Stage1EvidenceError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise Stage1EvidenceError(f"{label} must be finite")
    if minimum is not None and result < minimum:
        raise Stage1EvidenceError(f"{label} must be >= {minimum}")
    if maximum is not None and result > maximum:
        raise Stage1EvidenceError(f"{label} must be <= {maximum}")
    return result


def _sha256(value: Any, label: str) -> str:
    try:
        return require_sha256(value, label)
    except IdentityError as exc:
        raise Stage1EvidenceError(str(exc)) from exc


def _relative_repository_path(value: Any, label: str) -> PurePosixPath:
    path_text = _string(value, label)
    pure_path = PurePosixPath(path_text)
    if pure_path.is_absolute() or ".." in pure_path.parts:
        raise Stage1EvidenceError(f"{label} must stay inside the repository")
    return pure_path


def _repository_file(
    repository_root: Path,
    relative_path: Any,
    expected_sha256: Any,
    label: str,
) -> Path:
    pure_path = _relative_repository_path(relative_path, f"{label}.path")
    expected_digest = _sha256(expected_sha256, f"{label}.sha256")
    try:
        root = repository_root.resolve(strict=True)
        candidate = (root / Path(*pure_path.parts)).resolve(strict=True)
        candidate.relative_to(root)
    except (OSError, ValueError) as exc:
        raise Stage1EvidenceError(f"{label}.path is not a repository file") from exc
    if not candidate.is_file():
        raise Stage1EvidenceError(f"{label}.path is not a regular file")
    try:
        actual_digest = sha256_bytes(candidate.read_bytes())
    except (OSError, IdentityError) as exc:
        raise Stage1EvidenceError(f"cannot hash {label}.path") from exc
    if actual_digest != expected_digest:
        raise Stage1EvidenceError(f"{label}.sha256 does not match source bytes")
    return candidate


def _validate_authority(
    value: Any,
    repository_root: Path,
    *,
    verify_repository_files: bool,
) -> tuple[str, ...]:
    authority = _object(
        value,
        {"stage0_contract", "source_documents"},
        "authority",
    )
    stage0 = _object(
        authority["stage0_contract"],
        {"path", "sha256"},
        "authority.stage0_contract",
    )
    if _sha256(stage0["sha256"], "authority.stage0_contract.sha256") != (
        STAGE0_CONTRACT_SHA256
    ):
        raise Stage1EvidenceError("authority does not reference pinned Stage 0 bytes")
    if verify_repository_files:
        stage0_path = _repository_file(
            repository_root,
            stage0["path"],
            stage0["sha256"],
            "authority.stage0_contract",
        )
        try:
            load_stage0_contract_reference(stage0_path)
        except ContractSourceError as exc:
            raise Stage1EvidenceError("Stage 0 authority is invalid") from exc
    else:
        _relative_repository_path(stage0["path"], "authority.stage0_contract.path")

    documents = _array(authority["source_documents"], "authority.source_documents")
    roles: list[str] = []
    paths: set[str] = set()
    digests: set[str] = set()
    ordered_digests: list[str] = []
    for index, item in enumerate(documents):
        label = f"authority.source_documents[{index}]"
        document = _object(item, {"role", "path", "sha256"}, label)
        role = _string(document["role"], f"{label}.role")
        path = str(_relative_repository_path(document["path"], f"{label}.path"))
        digest = _sha256(document["sha256"], f"{label}.sha256")
        if role not in SOURCE_DOCUMENT_ROLES:
            raise Stage1EvidenceError(f"{label}.role is not a v1 evidence role")
        if role in roles or path in paths or digest in digests:
            raise Stage1EvidenceError(
                "source document roles, paths, and hashes must be unique"
            )
        roles.append(role)
        paths.add(path)
        digests.add(digest)
        ordered_digests.append(digest)
        if verify_repository_files:
            _repository_file(repository_root, path, digest, label)
    if tuple(roles) != SOURCE_DOCUMENT_ROLES:
        raise Stage1EvidenceError("source documents do not match the frozen role order")
    return tuple(ordered_digests)


def _validate_dataset_partitions(value: Any) -> tuple[tuple[str, str], ...]:
    partitions = _object(
        value,
        {"development", "validation", "final_test"},
        "dataset_partitions",
    )
    validation = _array(partitions["validation"], "dataset_partitions.validation")
    final_test = _array(partitions["final_test"], "dataset_partitions.final_test")
    if validation or final_test:
        raise Stage1EvidenceError(
            "development evidence cannot populate validation or final_test"
        )

    development = _array(partitions["development"], "dataset_partitions.development")
    if not development:
        raise Stage1EvidenceError("development partition must not be empty")
    trial_ids: set[str] = set()
    digests: set[str] = set()
    kinds: set[str] = set()
    references: list[tuple[str, str]] = []
    for index, item in enumerate(development):
        label = f"dataset_partitions.development[{index}]"
        entry = _object(
            item,
            {
                "trial_id",
                "evidence_kind",
                "bag_sha256",
                "source_document_role",
                "availability",
                "provenance_flags",
            },
            label,
        )
        trial_id = _string(entry["trial_id"], f"{label}.trial_id")
        if not _TRIAL_ID_RE.fullmatch(trial_id):
            raise Stage1EvidenceError(f"{label}.trial_id has invalid syntax")
        evidence_kind = _string(entry["evidence_kind"], f"{label}.evidence_kind")
        expected_role = EVIDENCE_KIND_TO_SOURCE_ROLE.get(evidence_kind)
        if expected_role is None:
            raise Stage1EvidenceError(f"{label}.evidence_kind is unknown")
        if entry["source_document_role"] != expected_role:
            raise Stage1EvidenceError(f"{label} is bound to the wrong source role")
        digest = _sha256(entry["bag_sha256"], f"{label}.bag_sha256")
        if entry["availability"] != EXTERNAL_AVAILABILITY:
            raise Stage1EvidenceError(
                f"{label}.availability cannot claim local byte verification"
            )
        flags = _array(entry["provenance_flags"], f"{label}.provenance_flags")
        if any(type(flag) is not str or not flag for flag in flags):
            raise Stage1EvidenceError(f"{label}.provenance_flags must contain strings")
        if len(flags) != len(set(flags)) or any(
            flag != "git_dirty=1" for flag in flags
        ):
            raise Stage1EvidenceError(f"{label}.provenance_flags are invalid")
        if trial_id in trial_ids or digest in digests:
            raise Stage1EvidenceError(
                "development trial IDs and bag hashes must be unique"
            )
        trial_ids.add(trial_id)
        digests.add(digest)
        kinds.add(evidence_kind)
        references.append((trial_id, digest))
    if kinds != set(EVIDENCE_KIND_TO_SOURCE_ROLE):
        raise Stage1EvidenceError(
            "development partition must retain both evidence kinds"
        )
    return tuple(references)


def _validate_accepted_findings(value: Any) -> None:
    findings = _object(
        value,
        {
            "scope",
            "observer_input",
            "extra_liquid_nowcast",
            "tested_effective_window_baseline",
        },
        "accepted_findings",
    )
    if findings["scope"] != "DEVELOPMENT_DIRECTION_ONLY":
        raise Stage1EvidenceError("accepted findings cannot exceed development scope")

    observer = _object(
        findings["observer_input"],
        {
            "selected",
            "i0_vs_o0_mean_mae_improvement_fraction",
            "improved_trial_count",
            "trial_count",
            "full_four_state_truth_claimed",
        },
        "accepted_findings.observer_input",
    )
    if observer["selected"] != "PROCESSED_IMU_I0":
        raise Stage1EvidenceError("processed-IMU I0 is the only accepted input")
    _finite_number(
        observer["i0_vs_o0_mean_mae_improvement_fraction"],
        "accepted_findings.observer_input.i0_vs_o0_mean_mae_improvement_fraction",
        minimum=0.0,
        maximum=1.0,
    )
    improved = _strict_int(
        observer["improved_trial_count"],
        "accepted_findings.observer_input.improved_trial_count",
        minimum=1,
    )
    trial_count = _strict_int(
        observer["trial_count"],
        "accepted_findings.observer_input.trial_count",
        minimum=1,
    )
    if improved > trial_count:
        raise Stage1EvidenceError("observer improved_trial_count exceeds trial_count")
    if observer["full_four_state_truth_claimed"] is not False:
        raise Stage1EvidenceError("development evidence cannot claim four-state truth")

    nowcast = _object(
        findings["extra_liquid_nowcast"],
        {"I1", "L22"},
        "accepted_findings.extra_liquid_nowcast",
    )
    i1 = _object(
        nowcast["I1"],
        {
            "allowed_in_solver",
            "mean_mae_regression_fraction",
            "regressed_trial_count",
            "trial_count",
        },
        "accepted_findings.extra_liquid_nowcast.I1",
    )
    if i1["allowed_in_solver"] is not False:
        raise Stage1EvidenceError("I1 must remain outside the solver")
    _finite_number(
        i1["mean_mae_regression_fraction"],
        "accepted_findings.extra_liquid_nowcast.I1.mean_mae_regression_fraction",
        minimum=0.0,
    )
    regressed = _strict_int(
        i1["regressed_trial_count"],
        "accepted_findings.extra_liquid_nowcast.I1.regressed_trial_count",
        minimum=1,
    )
    i1_count = _strict_int(
        i1["trial_count"],
        "accepted_findings.extra_liquid_nowcast.I1.trial_count",
        minimum=1,
    )
    if regressed > i1_count:
        raise Stage1EvidenceError("I1 regressed_trial_count exceeds trial_count")
    l22 = _object(
        nowcast["L22"],
        {"allowed_in_solver", "reason"},
        "accepted_findings.extra_liquid_nowcast.L22",
    )
    if l22["allowed_in_solver"] is not False or l22["reason"] != (
        "LEGACY_DIAGNOSTIC_WITH_COMMAND_HISTORY_NOT_ACTUAL_EXCITATION"
    ):
        raise Stage1EvidenceError("L22 must remain a legacy diagnostic")

    baseline = _object(
        findings["tested_effective_window_baseline"],
        {
            "status",
            "linear_sec",
            "angular_sec",
            "allowed_as_new_mainline_fopdt_parameter",
        },
        "accepted_findings.tested_effective_window_baseline",
    )
    if baseline["status"] != (
        "PROVISIONAL_TESTED_LEGACY_EFFECTIVE_WINDOW_NOT_FOPDT_DELAY"
    ):
        raise Stage1EvidenceError("legacy effective-window status drifted")
    _finite_number(baseline["linear_sec"], "baseline.linear_sec", minimum=0.0)
    _finite_number(baseline["angular_sec"], "baseline.angular_sec", minimum=0.0)
    if baseline["allowed_as_new_mainline_fopdt_parameter"] is not False:
        raise Stage1EvidenceError("legacy windows cannot become FOPDT parameters")


def _validate_descriptive_candidate(value: Any) -> None:
    candidate = _object(
        value,
        {"status", "linear", "angular"},
        "descriptive_actuator_candidate",
    )
    if candidate["status"] != "INCONCLUSIVE_OFFLINE_DESCRIPTION_ONLY":
        raise Stage1EvidenceError("descriptive candidate cannot claim frozen status")
    expected_boundary_observations = {
        "linear": "TAU_LOWER_SEARCH_BOUND_TOUCHED",
        "angular": "L_LOWER_SEARCH_BOUND_TOUCHED",
    }
    for channel in ("linear", "angular"):
        label = f"descriptive_actuator_candidate.{channel}"
        values = _object(
            candidate[channel],
            {
                "descriptive_L_sec",
                "tau_sec",
                "gain",
                "near_optimal_L_envelope_lower_sec",
                "near_optimal_L_envelope_upper_sec",
                "near_optimal_L_scope",
                "fit_boundary_observation",
            },
            label,
        )
        descriptive_l = _finite_number(
            values["descriptive_L_sec"], f"{label}.descriptive_L_sec", minimum=0.0
        )
        _finite_number(values["tau_sec"], f"{label}.tau_sec", minimum=1.0e-15)
        _finite_number(values["gain"], f"{label}.gain", minimum=1.0e-15)
        lower = _finite_number(
            values["near_optimal_L_envelope_lower_sec"],
            f"{label}.near_optimal_L_envelope_lower_sec",
            minimum=0.0,
        )
        upper = _finite_number(
            values["near_optimal_L_envelope_upper_sec"],
            f"{label}.near_optimal_L_envelope_upper_sec",
            minimum=0.0,
        )
        if upper < lower or not lower <= descriptive_l <= upper:
            raise Stage1EvidenceError(f"{label} candidate lies outside observed range")
        if values["near_optimal_L_scope"] != (
            "CROSS_BAG_ENVELOPE_NOT_PER_TRIAL_INTERVAL"
        ):
            raise Stage1EvidenceError(f"{label} must identify the cross-bag envelope")
        if (
            values["fit_boundary_observation"]
            != expected_boundary_observations[channel]
        ):
            raise Stage1EvidenceError(f"{label} fit boundary observation drifted")


def _validate_gates(value: Any) -> tuple[str, ...]:
    gates = _object(
        value,
        {
            "stage1_freeze_allowed",
            "stage3_codegen_allowed",
            "stage1_status",
            "stage3_status",
            "dataset_gate_status",
            "fit_identifiability_status",
            "lmax_status",
            "execution_parameters_status",
            "required_missing_authorities",
        },
        "gates",
    )
    if gates["stage1_freeze_allowed"] is not False:
        raise Stage1EvidenceError("development evidence cannot freeze Stage 1")
    if gates["stage3_codegen_allowed"] is not False:
        raise Stage1EvidenceError("development evidence cannot authorize codegen")
    expected_values = {
        "stage1_status": STAGE1_BLOCKED_STATUS,
        "stage3_status": STAGE3_PROHIBITED_STATUS,
        "dataset_gate_status": DATASET_GATE_STATUS,
        "fit_identifiability_status": FIT_IDENTIFIABILITY_STATUS,
        "lmax_status": LMAX_STATUS,
        "execution_parameters_status": EXECUTION_PARAMETERS_STATUS,
    }
    for key, expected in expected_values.items():
        if gates[key] != expected:
            raise Stage1EvidenceError(f"gates.{key} drifted from the blocked v1 gate")
    missing = _array(
        gates["required_missing_authorities"],
        "gates.required_missing_authorities",
    )
    if tuple(missing) != REQUIRED_MISSING_AUTHORITIES:
        raise Stage1EvidenceError(
            "required missing authorities are incomplete or reordered"
        )
    return tuple(missing)


@dataclass(frozen=True)
class _Stage1DevelopmentEvidenceReference:
    """Immutable proof that known evidence is development-only and blocked."""

    _construction_token: InitVar[object]
    schema_version: str
    evidence_id: str
    evidence_sha256: str
    status: str
    stage0_contract_sha256: str
    source_document_sha256: tuple[str, ...]
    development_trials: tuple[tuple[str, str], ...]
    stage1_status: str
    stage3_status: str
    required_missing_authorities: tuple[str, ...]

    def __post_init__(self, construction_token: object) -> None:
        if construction_token is not _REFERENCE_CONSTRUCTION_TOKEN:
            raise Stage1EvidenceError(
                "Stage 1 evidence references can only come from the pinned loader"
            )
        expected = {
            "schema_version": STAGE1_EVIDENCE_SCHEMA_VERSION,
            "evidence_id": STAGE1_EVIDENCE_ID,
            "evidence_sha256": STAGE1_EVIDENCE_SHA256,
            "status": STAGE1_EVIDENCE_STATUS,
            "stage0_contract_sha256": STAGE0_CONTRACT_SHA256,
            "stage1_status": STAGE1_BLOCKED_STATUS,
            "stage3_status": STAGE3_PROHIBITED_STATUS,
        }
        for key, expected_value in expected.items():
            if getattr(self, key) != expected_value:
                raise Stage1EvidenceError(f"blocked reference {key} is invalid")
        if self.required_missing_authorities != REQUIRED_MISSING_AUTHORITIES:
            raise Stage1EvidenceError(
                "blocked reference is missing required authorities"
            )
        if len(self.source_document_sha256) != len(SOURCE_DOCUMENT_ROLES):
            raise Stage1EvidenceError(
                "blocked reference source document count is invalid"
            )
        for index, digest in enumerate(self.source_document_sha256):
            _sha256(digest, f"blocked reference source_document_sha256[{index}]")
        trial_ids = [trial_id for trial_id, _ in self.development_trials]
        trial_hashes = [digest for _, digest in self.development_trials]
        if len(trial_ids) != len(set(trial_ids)) or len(trial_hashes) != len(
            set(trial_hashes)
        ):
            raise Stage1EvidenceError(
                "blocked reference development trials are not unique"
            )
        for index, (trial_id, digest) in enumerate(self.development_trials):
            if not _TRIAL_ID_RE.fullmatch(trial_id):
                raise Stage1EvidenceError(
                    f"blocked reference development_trials[{index}] ID is invalid"
                )
            _sha256(digest, f"blocked reference development_trials[{index}] hash")

    @property
    def stage1_freeze_allowed(self) -> bool:
        return False

    @property
    def stage3_codegen_allowed(self) -> bool:
        return False

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "evidence_id": self.evidence_id,
            "evidence_sha256": self.evidence_sha256,
            "status": self.status,
            "stage0_contract_sha256": self.stage0_contract_sha256,
            "source_document_sha256": list(self.source_document_sha256),
            "development_trials": [
                {"trial_id": trial_id, "bag_sha256": digest}
                for trial_id, digest in self.development_trials
            ],
            "stage1_status": self.stage1_status,
            "stage3_status": self.stage3_status,
            "stage1_freeze_allowed": False,
            "stage3_codegen_allowed": False,
            "required_missing_authorities": list(self.required_missing_authorities),
        }


def _validate_stage1_development_evidence(
    value: Any,
    repository_root: Path | str,
    *,
    verify_repository_files: bool = True,
) -> None:
    """Validate schema only; this private helper never creates authority."""

    root = Path(repository_root) if isinstance(repository_root, (str, Path)) else None
    if root is None:
        raise Stage1EvidenceError("repository_root must be str or Path")
    document = _object(
        value,
        {
            "schema_version",
            "evidence_id",
            "status",
            "authority",
            "dataset_partitions",
            "accepted_findings",
            "descriptive_actuator_candidate",
            "gates",
        },
        "Stage 1 development evidence",
    )
    if document["schema_version"] != STAGE1_EVIDENCE_SCHEMA_VERSION:
        raise Stage1EvidenceError("unsupported Stage 1 evidence schema")
    if document["evidence_id"] != STAGE1_EVIDENCE_ID:
        raise Stage1EvidenceError("unexpected Stage 1 evidence ID")
    if document["status"] != STAGE1_EVIDENCE_STATUS:
        raise Stage1EvidenceError("Stage 1 evidence cannot claim frozen status")
    _validate_authority(
        document["authority"], root, verify_repository_files=verify_repository_files
    )
    _validate_dataset_partitions(document["dataset_partitions"])
    _validate_accepted_findings(document["accepted_findings"])
    _validate_descriptive_candidate(document["descriptive_actuator_candidate"])
    _validate_gates(document["gates"])


def load_stage1_development_evidence(
    path: Path | str,
    repository_root: Path | str,
) -> _Stage1DevelopmentEvidenceReference:
    """Load the pinned v1 snapshot and prove it cannot authorize production."""

    try:
        value, payload = read_strict_json(path, label="Stage 1 development evidence")
    except IdentityError as exc:
        raise Stage1EvidenceError(str(exc)) from exc
    digest = sha256_bytes(payload)
    if digest != STAGE1_EVIDENCE_SHA256:
        raise Stage1EvidenceError(
            "Stage 1 development evidence is not the pinned immutable v1 snapshot"
        )
    _validate_stage1_development_evidence(value, repository_root)
    source_hashes = tuple(
        item["sha256"] for item in value["authority"]["source_documents"]
    )
    trials = tuple(
        (item["trial_id"], item["bag_sha256"])
        for item in value["dataset_partitions"]["development"]
    )
    missing = tuple(value["gates"]["required_missing_authorities"])
    return _Stage1DevelopmentEvidenceReference(
        _construction_token=_REFERENCE_CONSTRUCTION_TOKEN,
        schema_version=STAGE1_EVIDENCE_SCHEMA_VERSION,
        evidence_id=STAGE1_EVIDENCE_ID,
        evidence_sha256=digest,
        status=STAGE1_EVIDENCE_STATUS,
        stage0_contract_sha256=STAGE0_CONTRACT_SHA256,
        source_document_sha256=source_hashes,
        development_trials=trials,
        stage1_status=STAGE1_BLOCKED_STATUS,
        stage3_status=STAGE3_PROHIBITED_STATUS,
        required_missing_authorities=missing,
    )
