"""Fail-closed intake for externally approved formal liquid-plant evidence.

This module is deliberately separate from :mod:`scout_liquid_plant.fidelity`.
The latter is a development-only comparison utility and can *never* create a
formal PASS.  This module does not estimate fidelity, launch ROS, start
Gazebo, or manufacture an approval.  It only assembles a formal capability
report after checking a caller-supplied, hash-bound set of independently
produced formal artifacts.

The resulting toolchain binding is intentionally compatible with
``smpcc_sim_toolchain.validate_formal_liquid_plant_capability``.  It is not a
substitute for an external release, real-reference evidence, or approval:
all of those must already exist and remain hash-bound in the emitted report.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

try:  # ``python3-yaml`` is already an exec/test dependency of this package.
    import yaml
except ImportError:  # pragma: no cover - exercised only in a broken package install.
    yaml = None


TOOL_ID = "SMPCC-SIM-LIQUID-PLANT-FORMAL-EVIDENCE-INTAKE-v1"
INTAKE_REQUEST_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-intake-request-v1"
FORMAL_RELEASE_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-release-v1"
FORMAL_CAPABILITY_REPORT_SCHEMA_VERSION = "smpcc-sim-independent-liquid-plant-capability-v1"
TOOLCHAIN_BINDING_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-toolchain-binding-v1"
FORMAL_FIDELITY_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-fidelity-validation-v1"
FORMAL_REFERENCE_EVIDENCE_SCHEMA_VERSION = "smpcc-real-liquid-reference-evidence-v1"
FORMAL_PLANT_SIGNAL_EVIDENCE_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-signal-evidence-v1"
FORMAL_ISOLATION_EVIDENCE_SCHEMA_VERSION = "smpcc-sim-controller-plant-isolation-evidence-v1"
FORMAL_APPROVAL_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-approval-v1"

FORMAL_RELEASE_REPORT_TYPE = "SMPCC_SIM_FORMAL_LIQUID_PLANT_RELEASE"
FORMAL_FIDELITY_REPORT_TYPE = "SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION"
FORMAL_REFERENCE_EVIDENCE_REPORT_TYPE = "SMPCC_REAL_LIQUID_REFERENCE_EVIDENCE"
FORMAL_PLANT_SIGNAL_EVIDENCE_REPORT_TYPE = "SMPCC_SIM_FORMAL_LIQUID_PLANT_SIGNAL_EVIDENCE"
FORMAL_ISOLATION_EVIDENCE_REPORT_TYPE = "SMPCC_SIM_CONTROLLER_PLANT_ISOLATION_EVIDENCE"
FORMAL_APPROVAL_REPORT_TYPE = "SMPCC_SIM_FORMAL_LIQUID_PLANT_APPROVAL"
FORMAL_CAPABILITY_REPORT_TYPE = "SMPCC_SIM_INDEPENDENT_LIQUID_PLANT_CAPABILITY"
TOOLCHAIN_BINDING_TYPE = "SMPCC_SIM_FORMAL_LIQUID_PLANT_TOOLCHAIN_BINDING"
CRYPTOGRAPHIC_TRUST_ANCHOR_STATUS = "NOT_CONFIGURED"
EXTERNAL_APPROVAL_AUTHENTICATION_STATUS = "NOT_INDEPENDENTLY_AUTHENTICATED"

TRUTH_TOPIC = "/sim_truth/liquid_height"
DIMENSIONS = ("amplitude", "frequency", "damping", "phase", "ranking")
REAL_REFERENCE_KINDS = frozenset(("REAL_RGB_LIQUID_HEIGHT", "REAL_LIQUID_HEIGHT_SENSOR"))
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

# These terms identify forbidden truth sources or the known development
# implementation.  They are intentionally checked only in fields that claim
# a plant/reference source (or in formal plant/verifier source code), not in
# arbitrary explanatory prose.  A firewall graph may legitimately *mention*
# an internal diagnostic publisher while proving it is not a subscriber.
FORBIDDEN_TRUTH_SOURCE_TOKENS = (
    "/slosh/height",
    "/spmpc/slosh_height",
    "h_proxy",
    "h_modal",
    "liquidsloshmodel",
    "w5_s10",
)
FORBIDDEN_DEVELOPMENT_SOURCE_TOKENS = (
    "development_only",
    "development-only",
    "development_template",
    "unvalidated",
    "current_sim_only",
)


class FormalEvidenceError(ValueError):
    """A formal-evidence defect that must stop capability assembly."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class BoundArtifact:
    """One absolute, existing file with a verified SHA-256 binding."""

    path: Path
    sha256: str


@dataclass(frozen=True)
class FormalCapabilityAssembly:
    """Validated report plus fields needed to bind it into a formal freeze."""

    capability_report: Mapping[str, Any]
    capability_fields: Mapping[str, Any]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_sha256(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _fail(code: str, message: str) -> None:
    raise FormalEvidenceError(code, message)


def _require(condition: bool, code: str, message: str) -> None:
    if not condition:
        _fail(code, message)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        _fail("MALFORMED_INPUT", "{} must be a JSON/YAML object".format(label))
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        _fail("MALFORMED_INPUT", "{} must be a non-empty string".format(label))
    return value.strip()


def _positive_finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail("MALFORMED_INPUT", "{} must be a positive finite number".format(label))
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        _fail("MALFORMED_INPUT", "{} must be a positive finite number".format(label))
    return result


def _absolute_existing_path(value: Any, label: str) -> Path:
    raw = _nonempty_string(value, label)
    path = Path(raw)
    if not path.is_absolute():
        _fail("MALFORMED_INPUT", "{} path must be absolute: {}".format(label, path))
    if not path.is_file():
        _fail("MISSING_ARTIFACT", "{} file is missing: {}".format(label, path))
    return path.resolve()


def _bound_artifact(value: Any, label: str) -> BoundArtifact:
    descriptor = _mapping(value, label)
    expected_keys = {"path", "sha256"}
    if set(descriptor) != expected_keys:
        _fail(
            "MALFORMED_INPUT",
            "{} descriptor must contain exactly path and sha256".format(label),
        )
    path = _absolute_existing_path(descriptor.get("path"), label)
    expected_hash = descriptor.get("sha256")
    if not is_sha256(expected_hash):
        _fail("MISSING_HASH", "{} requires a lowercase SHA-256".format(label))
    actual_hash = sha256_file(path)
    if actual_hash != expected_hash:
        _fail("HASH_MISMATCH", "{} SHA-256 mismatch for {}".format(label, path))
    return BoundArtifact(path=path, sha256=actual_hash)


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            return _mapping(json.load(stream), label)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        _fail("MALFORMED_JSON", "cannot read {}: {}".format(label, exc))
    raise AssertionError("unreachable")


def _read_json_or_yaml(path: Path, label: str) -> Mapping[str, Any]:
    suffix = path.suffix.lower()
    if suffix == ".json":
        return _read_json(path, label)
    if suffix not in {".yaml", ".yml"}:
        _fail("MALFORMED_INPUT", "{} must be a JSON or YAML file".format(label))
    if yaml is None:
        _fail("MISSING_DEPENDENCY", "python3-yaml is required to read {}".format(label))
    try:
        with path.open("r", encoding="utf-8") as stream:
            return _mapping(yaml.safe_load(stream), label)
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:  # type: ignore[union-attr]
        _fail("MALFORMED_YAML", "cannot read {}: {}".format(label, exc))
    raise AssertionError("unreachable")


def _bound_json(value: Any, label: str) -> Tuple[BoundArtifact, Mapping[str, Any]]:
    artifact = _bound_artifact(value, label)
    return artifact, _read_json(artifact.path, label)


def _same_artifact(left: BoundArtifact, right: BoundArtifact, label: str) -> None:
    _require(
        left.path == right.path and left.sha256 == right.sha256,
        "CROSS_BINDING_MISMATCH",
        "{} differs from the hash-bound formal release/request artifact".format(label),
    )


def _require_formal_status(document: Mapping[str, Any], label: str, status: str) -> None:
    _require(document.get("status") == status, "NONFORMAL_ARTIFACT", "{} status must be {}".format(label, status))
    _require(document.get("formal") is True, "NONFORMAL_ARTIFACT", "{} must set formal=true".format(label))
    _require(
        document.get("development_only") is False,
        "DEVELOPMENT_ARTIFACT_FORBIDDEN",
        "{} must set development_only=false".format(label),
    )


def _require_sha(document: Mapping[str, Any], key: str, label: str) -> str:
    value = document.get(key)
    _require(is_sha256(value), "MISSING_HASH", "{}.{} must be a lowercase SHA-256".format(label, key))
    return str(value)


def _source_contains_forbidden_token(text: str) -> Optional[str]:
    lower = text.casefold()
    for token in FORBIDDEN_TRUTH_SOURCE_TOKENS + FORBIDDEN_DEVELOPMENT_SOURCE_TOKENS:
        if token.casefold() in lower:
            return token
    return None


def _forbidden_truth_token(value: Any) -> Optional[str]:
    if not isinstance(value, str):
        return None
    lower = value.casefold()
    for token in FORBIDDEN_TRUTH_SOURCE_TOKENS:
        if token.casefold() in lower:
            return token
    return None


def _forbidden_development_value(value: Any) -> Optional[str]:
    if not isinstance(value, str):
        return None
    lower = value.casefold()
    for token in FORBIDDEN_DEVELOPMENT_SOURCE_TOKENS:
        if token.casefold() in lower:
            return token
    return None


def _package_root() -> Path:
    # .../scout_liquid_plant/src/scout_liquid_plant/formal_intake.py
    return Path(__file__).resolve().parents[2]


def _known_development_artifacts() -> frozenset[Path]:
    root = _package_root()
    candidates = (
        root / "config" / "C1_development_unvalidated.yaml",
        root / "config" / "C2_development_unvalidated.yaml",
        root / "schema" / "liquid_plant_io_schema_v1.json",
        root / "schema" / "liquid_plant_fidelity_report_v1.json",
        root / "launch" / "liquid_plant_development.launch",
        root / "scripts" / "liquid_plant_node.py",
        root / "scripts" / "liquid_plant_fidelity_verify.py",
        root / "src" / "scout_liquid_plant" / "core.py",
        root / "src" / "scout_liquid_plant" / "fidelity.py",
    )
    return frozenset(path.resolve() for path in candidates if path.exists())


def _reject_known_development_artifact(artifact: BoundArtifact, label: str) -> None:
    if artifact.path in _known_development_artifacts() or "development_unvalidated" in artifact.path.name.casefold():
        _fail(
            "DEVELOPMENT_ARTIFACT_FORBIDDEN",
            "{} is a shipped/current development-only artifact: {}".format(label, artifact.path),
        )


def _validate_formal_code(artifact: BoundArtifact) -> None:
    _reject_known_development_artifact(artifact, "formal plant code")
    try:
        source = artifact.path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        _fail(
            "MALFORMED_FORMAL_CODE",
            "formal plant code must be an independently reviewable UTF-8 source artifact: {}".format(exc),
        )
    _require(bool(source.strip()), "MALFORMED_FORMAL_CODE", "formal plant code is empty")
    token = _source_contains_forbidden_token(source)
    if token is not None:
        _fail(
            "FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE",
            "formal plant code contains forbidden source token {!r}".format(token),
        )


def _validate_parameter_document(document: Mapping[str, Any], release_id: str) -> None:
    _require(
        document.get("document_type") == "SMPCC_SIM_FORMAL_LIQUID_PLANT_PARAMETERS",
        "MALFORMED_PARAMETERS",
        "formal plant parameters have wrong document_type",
    )
    _require_formal_status(document, "formal plant parameters", "FROZEN")
    _require(document.get("release_id") == release_id, "CROSS_BINDING_MISMATCH", "formal plant parameter release_id mismatch")
    _require_sha(document, "initial_state_rule_hash", "formal plant parameters")
    _positive_finite(document.get("integration_step_sec"), "formal plant parameters.integration_step_sec")
    for key in ("condition_template_id", "fidelity_validation_status"):
        token = _forbidden_development_value(document.get(key))
        if token is not None:
            _fail("DEVELOPMENT_ARTIFACT_FORBIDDEN", "formal plant parameters contain {!r}".format(token))


def _validate_input_schema(document: Mapping[str, Any], release_id: str) -> None:
    _require(
        document.get("document_type") == "SMPCC_SIM_FORMAL_LIQUID_PLANT_INPUT_SCHEMA",
        "MALFORMED_IO_SCHEMA",
        "formal plant input schema has wrong document_type",
    )
    _require_formal_status(document, "formal plant input schema", "FROZEN")
    _require(document.get("release_id") == release_id, "CROSS_BINDING_MISMATCH", "formal input schema release_id mismatch")
    input_spec = _mapping(document.get("input"), "formal plant input schema.input")
    _require(input_spec.get("topic") == "/odom", "ISOLATION_SEMANTICS_MISMATCH", "formal plant input must be /odom")
    _require(
        input_spec.get("semantic") == "executed_simulated_base_motion",
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal plant input semantic must be executed_simulated_base_motion",
    )
    _require(
        input_spec.get("only_executed_base_motion") is True,
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal plant input must attest only_executed_base_motion=true",
    )
    _require(
        input_spec.get("raw_command_input_forbidden") is True,
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal plant input must forbid raw command input",
    )
    isolation = _mapping(document.get("isolation"), "formal plant input schema.isolation")
    expected = {
        "implementation_isolated_from_controller": True,
        "controller_hidden_state_access": False,
        "controller_state_import_forbidden": True,
        "controller_truth_subscription_forbidden": True,
        "plant_reads_raw_command": False,
    }
    for key, expected_value in expected.items():
        _require(
            isolation.get(key) == expected_value,
            "ISOLATION_SEMANTICS_MISMATCH",
            "formal plant input isolation.{} must be {!r}".format(key, expected_value),
        )
    subscriptions = isolation.get("plant_subscriptions")
    _require(
        subscriptions == ["/odom"],
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal plant must subscribe only to /odom",
    )
    for value in (input_spec.get("topic"), input_spec.get("semantic")):
        token = _forbidden_truth_token(value)
        if token is not None:
            _fail("FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE", "formal plant input contains {!r}".format(token))


def _validate_output_schema(document: Mapping[str, Any], release_id: str) -> None:
    _require(
        document.get("document_type") == "SMPCC_SIM_FORMAL_LIQUID_PLANT_OUTPUT_SCHEMA",
        "MALFORMED_IO_SCHEMA",
        "formal plant output schema has wrong document_type",
    )
    _require_formal_status(document, "formal plant output schema", "FROZEN")
    _require(document.get("release_id") == release_id, "CROSS_BINDING_MISMATCH", "formal output schema release_id mismatch")
    _require(document.get("truth_topic") == TRUTH_TOPIC, "TRUTH_TOPIC_MISMATCH", "formal truth topic must be {}".format(TRUTH_TOPIC))
    outputs = _mapping(document.get("outputs"), "formal plant output schema.outputs")
    height = _mapping(outputs.get("liquid_height"), "formal plant output schema.outputs.liquid_height")
    _require(height.get("topic") == TRUTH_TOPIC, "TRUTH_TOPIC_MISMATCH", "formal liquid_height output topic mismatch")
    _require(
        document.get("controller_feedback_forbidden") is True,
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal plant output schema must forbid controller feedback",
    )
    token = _forbidden_truth_token(document.get("truth_topic"))
    if token is not None:
        _fail("FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE", "formal truth topic contains {!r}".format(token))


def _release_artifact(document: Mapping[str, Any], key: str) -> BoundArtifact:
    artifacts = _mapping(document.get("artifacts"), "formal plant release.artifacts")
    return _bound_artifact(artifacts.get(key), "formal plant release.artifacts.{}".format(key))


def _validate_release(
    artifact: BoundArtifact,
    document: Mapping[str, Any],
    direct_artifacts: Mapping[str, BoundArtifact],
) -> Dict[str, Any]:
    _reject_known_development_artifact(artifact, "formal plant release")
    _require(
        document.get("schema_version") == FORMAL_RELEASE_SCHEMA_VERSION,
        "MALFORMED_RELEASE",
        "formal plant release has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_RELEASE_REPORT_TYPE,
        "MALFORMED_RELEASE",
        "formal plant release has wrong report_type",
    )
    _require_formal_status(document, "formal plant release", "FROZEN")
    release_id = _nonempty_string(document.get("release_id"), "formal plant release.release_id")
    sim_freeze_id = _nonempty_string(document.get("sim_freeze_id"), "formal plant release.sim_freeze_id")
    payload = dict(document)
    declared_payload_hash = payload.pop("release_payload_hash", None)
    _require(
        declared_payload_hash == canonical_hash(payload),
        "HASH_MISMATCH",
        "formal plant release_payload_hash does not bind its immutable payload",
    )
    plant = _mapping(document.get("plant"), "formal plant release.plant")
    expected = {
        "independent_plant": True,
        "implementation_isolated_from_controller": True,
        "controller_hidden_state_access": False,
        "driven_by": "executed_simulated_base_motion",
        "truth_topic": TRUTH_TOPIC,
    }
    for key, expected_value in expected.items():
        _require(
            plant.get(key) == expected_value,
            "ISOLATION_SEMANTICS_MISMATCH",
            "formal plant release.plant.{} must be {!r}".format(key, expected_value),
        )
    for key, direct in direct_artifacts.items():
        release_artifact = _release_artifact(document, key)
        _same_artifact(release_artifact, direct, "formal plant release artifact {}".format(key))
    return {
        "release_id": release_id,
        "sim_freeze_id": sim_freeze_id,
        "plant": dict(plant),
        "release_payload_hash": str(declared_payload_hash),
    }


def _normalize_reference_entries(entries: Sequence[Any], fidelity_report: Mapping[str, Any]) -> List[Dict[str, str]]:
    _require(
        len(entries) >= 2,
        "MISSING_REAL_REFERENCE_PROVENANCE",
        "formal fidelity report requires at least two real-reference cases for ranking",
    )
    normalized: List[Dict[str, str]] = []
    seen: set[str] = set()
    for raw in entries:
        entry = _mapping(raw, "formal fidelity reference entry")
        required = {
            "case_id",
            "reference_evidence_path",
            "reference_evidence_hash",
            "reference_signal_path",
            "reference_signal_hash",
            "reference_kind",
        }
        _require(
            set(entry) == required,
            "MALFORMED_REFERENCE_EVIDENCE",
            "formal fidelity reference entry has missing/unknown fields",
        )
        case_id = _nonempty_string(entry.get("case_id"), "formal fidelity reference case_id")
        _require(case_id not in seen, "MALFORMED_REFERENCE_EVIDENCE", "duplicate formal reference case_id {}".format(case_id))
        seen.add(case_id)
        evidence = _bound_artifact(
            {"path": entry.get("reference_evidence_path"), "sha256": entry.get("reference_evidence_hash")},
            "formal reference evidence for {}".format(case_id),
        )
        signal = _bound_artifact(
            {"path": entry.get("reference_signal_path"), "sha256": entry.get("reference_signal_hash")},
            "formal reference signal for {}".format(case_id),
        )
        evidence_document = _read_json(evidence.path, "formal reference evidence for {}".format(case_id))
        _validate_reference_evidence(evidence_document, evidence, signal, case_id, entry.get("reference_kind"), fidelity_report)
        normalized.append(
            {
                "case_id": case_id,
                "reference_evidence_path": str(evidence.path),
                "reference_evidence_hash": evidence.sha256,
                "reference_signal_path": str(signal.path),
                "reference_signal_hash": signal.sha256,
                "reference_kind": str(entry["reference_kind"]),
            }
        )
    return sorted(normalized, key=lambda item: item["case_id"])


def _validate_reference_evidence(
    document: Mapping[str, Any],
    artifact: BoundArtifact,
    signal: BoundArtifact,
    case_id: str,
    declared_kind: Any,
    fidelity_report: Mapping[str, Any],
) -> None:
    _reject_known_development_artifact(artifact, "formal real-reference evidence")
    _require(
        document.get("schema_version") == FORMAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
        "MALFORMED_REFERENCE_EVIDENCE",
        "formal real-reference evidence has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_REFERENCE_EVIDENCE_REPORT_TYPE,
        "MALFORMED_REFERENCE_EVIDENCE",
        "formal real-reference evidence has wrong report_type",
    )
    _require_formal_status(document, "formal real-reference evidence", "FROZEN")
    _require(document.get("case_id") == case_id, "CROSS_BINDING_MISMATCH", "formal real-reference case_id mismatch")
    kind = document.get("reference_kind")
    _require(kind in REAL_REFERENCE_KINDS, "MISSING_REAL_REFERENCE_PROVENANCE", "reference_kind must be real RGB or real liquid sensor")
    _require(kind == declared_kind, "CROSS_BINDING_MISMATCH", "formal fidelity/reference evidence kind mismatch")
    _require(document.get("real_measurement") is True, "MISSING_REAL_REFERENCE_PROVENANCE", "reference is not a real measurement")
    _require(
        document.get("measurement_independent_of_plant") is True,
        "MISSING_REAL_REFERENCE_PROVENANCE",
        "reference measurement is not independent of the plant",
    )
    _nonempty_string(document.get("reference_freeze_id"), "formal real-reference evidence.reference_freeze_id")
    _require(
        document.get("formal_release_manifest_hash") == fidelity_report.get("formal_release_manifest_hash"),
        "CROSS_BINDING_MISMATCH",
        "formal real-reference evidence does not bind the fidelity release",
    )
    evidence_signal = _bound_artifact(
        {"path": document.get("reference_signal_path"), "sha256": document.get("reference_signal_hash")},
        "formal real-reference evidence signal",
    )
    _same_artifact(evidence_signal, signal, "formal real-reference signal")
    for label, path_key, hash_key in (
        ("source bag", "source_bag_path", "source_bag_hash"),
        ("extraction pipeline", "extraction_pipeline_path", "extraction_pipeline_hash"),
        ("calibration", "calibration_path", "calibration_hash"),
    ):
        _bound_artifact({"path": document.get(path_key), "sha256": document.get(hash_key)}, "formal reference {}".format(label))
    source_topic = _nonempty_string(document.get("source_topic"), "formal real-reference evidence.source_topic")
    for value in (source_topic, str(kind), str(signal.path.name)):
        token = _forbidden_truth_token(value)
        if token is not None:
            _fail(
                "FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE",
                "formal real-reference evidence contains forbidden truth source {!r}".format(token),
            )
        token = _forbidden_development_value(value)
        if token is not None:
            _fail("DEVELOPMENT_ARTIFACT_FORBIDDEN", "formal real-reference evidence contains {!r}".format(token))


def _normalize_plant_signal_entries(
    entries: Sequence[Any],
    fidelity_report: Mapping[str, Any],
    direct_artifacts: Mapping[str, BoundArtifact],
) -> List[Dict[str, str]]:
    """Bind every simulated signal that entered an external fidelity report.

    A report that merely labels its truth topic can otherwise hide a
    `/slosh/height` or controller-modal trace behind a copied numeric file.
    This requires a separate formal run-evidence record for each plant trace.
    """

    _require(
        len(entries) >= 2,
        "MISSING_PLANT_SIGNAL_PROVENANCE",
        "formal fidelity report requires at least two hash-bound plant-signal cases",
    )
    normalized: List[Dict[str, str]] = []
    seen: set[str] = set()
    for raw in entries:
        entry = _mapping(raw, "formal fidelity plant-signal entry")
        required = {
            "case_id",
            "plant_signal_path",
            "plant_signal_hash",
            "plant_run_manifest_path",
            "plant_run_manifest_hash",
            "plant_signal_topic",
        }
        _require(
            set(entry) == required,
            "MALFORMED_PLANT_SIGNAL_EVIDENCE",
            "formal fidelity plant-signal entry has missing/unknown fields",
        )
        case_id = _nonempty_string(entry.get("case_id"), "formal fidelity plant-signal case_id")
        _require(case_id not in seen, "MALFORMED_PLANT_SIGNAL_EVIDENCE", "duplicate plant-signal case_id {}".format(case_id))
        seen.add(case_id)
        _require(
            entry.get("plant_signal_topic") == TRUTH_TOPIC,
            "TRUTH_TOPIC_MISMATCH",
            "formal fidelity plant signal must use {}".format(TRUTH_TOPIC),
        )
        signal = _bound_artifact(
            {"path": entry.get("plant_signal_path"), "sha256": entry.get("plant_signal_hash")},
            "formal plant signal for {}".format(case_id),
        )
        run_manifest = _bound_artifact(
            {
                "path": entry.get("plant_run_manifest_path"),
                "sha256": entry.get("plant_run_manifest_hash"),
            },
            "formal plant run evidence for {}".format(case_id),
        )
        _reject_known_development_artifact(signal, "formal plant signal")
        _reject_known_development_artifact(run_manifest, "formal plant run evidence")
        run_document = _read_json(run_manifest.path, "formal plant run evidence for {}".format(case_id))
        _validate_plant_signal_evidence(
            run_document,
            signal,
            case_id,
            fidelity_report,
            direct_artifacts,
        )
        for value in (str(entry.get("plant_signal_topic")), str(signal.path.name)):
            token = _forbidden_truth_token(value)
            if token is not None:
                _fail(
                    "FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE",
                    "formal plant-signal evidence contains forbidden truth source {!r}".format(token),
                )
            token = _forbidden_development_value(value)
            if token is not None:
                _fail("DEVELOPMENT_ARTIFACT_FORBIDDEN", "formal plant-signal evidence contains {!r}".format(token))
        normalized.append(
            {
                "case_id": case_id,
                "plant_signal_path": str(signal.path),
                "plant_signal_hash": signal.sha256,
                "plant_run_manifest_path": str(run_manifest.path),
                "plant_run_manifest_hash": run_manifest.sha256,
                "plant_signal_topic": TRUTH_TOPIC,
            }
        )
    return sorted(normalized, key=lambda item: item["case_id"])


def _validate_plant_signal_evidence(
    document: Mapping[str, Any],
    signal: BoundArtifact,
    case_id: str,
    fidelity_report: Mapping[str, Any],
    direct_artifacts: Mapping[str, BoundArtifact],
) -> None:
    _require(
        document.get("schema_version") == FORMAL_PLANT_SIGNAL_EVIDENCE_SCHEMA_VERSION,
        "MALFORMED_PLANT_SIGNAL_EVIDENCE",
        "formal plant run evidence has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_PLANT_SIGNAL_EVIDENCE_REPORT_TYPE,
        "MALFORMED_PLANT_SIGNAL_EVIDENCE",
        "formal plant run evidence has wrong report_type",
    )
    _require_formal_status(document, "formal plant run evidence", "PASS")
    _require(document.get("case_id") == case_id, "CROSS_BINDING_MISMATCH", "formal plant run evidence case_id mismatch")
    _require(
        document.get("formal_release_manifest_hash") == fidelity_report.get("formal_release_manifest_hash"),
        "CROSS_BINDING_MISMATCH",
        "formal plant run evidence release hash mismatch",
    )
    _require(document.get("truth_topic") == TRUTH_TOPIC, "TRUTH_TOPIC_MISMATCH", "formal plant run evidence truth_topic mismatch")
    evidence_signal = _bound_artifact(
        {"path": document.get("plant_signal_path"), "sha256": document.get("plant_signal_hash")},
        "formal plant run evidence signal",
    )
    _same_artifact(evidence_signal, signal, "formal plant run signal")
    for document_key, artifact_key in (
        ("plant_code_hash", "plant_code"),
        ("plant_parameter_hash", "plant_parameters"),
        ("plant_input_schema_hash", "plant_input_schema"),
        ("plant_output_schema_hash", "plant_output_schema"),
    ):
        _require(
            document.get(document_key) == direct_artifacts[artifact_key].sha256,
            "CROSS_BINDING_MISMATCH",
            "formal plant run evidence {} mismatch".format(document_key),
        )


def _validate_fidelity(
    artifact: BoundArtifact,
    document: Mapping[str, Any],
    release_artifact: BoundArtifact,
    direct_artifacts: Mapping[str, BoundArtifact],
) -> Dict[str, Any]:
    _reject_known_development_artifact(artifact, "formal plant fidelity report")
    _require(
        document.get("schema_version") == FORMAL_FIDELITY_SCHEMA_VERSION,
        "MALFORMED_FIDELITY_REPORT",
        "formal plant fidelity report has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_FIDELITY_REPORT_TYPE,
        "MALFORMED_FIDELITY_REPORT",
        "formal plant fidelity report has wrong report_type",
    )
    _require_formal_status(document, "formal plant fidelity report", "PASS")
    _require(
        document.get("fidelity_validation_status") == "PASS",
        "MALFORMED_FIDELITY_REPORT",
        "formal plant fidelity report must set fidelity_validation_status=PASS",
    )
    _require(document.get("truth_topic") == TRUTH_TOPIC, "TRUTH_TOPIC_MISMATCH", "formal plant fidelity truth_topic mismatch")
    _require(
        document.get("independently_produced") is True,
        "MALFORMED_FIDELITY_REPORT",
        "formal plant fidelity report must attest independently_produced=true",
    )
    _require(
        document.get("formal_release_manifest_hash") == release_artifact.sha256,
        "CROSS_BINDING_MISMATCH",
        "formal plant fidelity report release hash mismatch",
    )
    for key, direct in direct_artifacts.items():
        report_key = {
            "plant_code": "plant_code_hash",
            "plant_parameters": "plant_parameter_hash",
            "plant_input_schema": "plant_input_schema_hash",
            "plant_output_schema": "plant_output_schema_hash",
        }[key]
        _require(
            document.get(report_key) == direct.sha256,
            "CROSS_BINDING_MISMATCH",
            "formal plant fidelity report {} mismatch".format(report_key),
        )
    dimensions = _mapping(document.get("validation_dimensions"), "formal plant fidelity validation_dimensions")
    _require(
        set(dimensions) == set(DIMENSIONS) and all(dimensions.get(key) == "PASS" for key in DIMENSIONS),
        "FIDELITY_NOT_PASS",
        "formal plant fidelity report must PASS amplitude/frequency/damping/phase/ranking",
    )
    verifier = _bound_artifact(
        {
            "path": document.get("fidelity_verifier_source_path"),
            "sha256": document.get("fidelity_verifier_source_hash"),
        },
        "formal fidelity verifier source",
    )
    _reject_known_development_artifact(verifier, "formal fidelity verifier source")
    try:
        verifier_source = verifier.path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        _fail("MALFORMED_FIDELITY_REPORT", "formal fidelity verifier source cannot be read: {}".format(exc))
    token = _source_contains_forbidden_token(verifier_source)
    if token is not None:
        _fail(
            "FORBIDDEN_TRUTH_OR_DEVELOPMENT_SOURCE",
            "formal fidelity verifier source contains forbidden token {!r}".format(token),
        )
    entries_value = document.get("formal_reference_evidence")
    _require(
        isinstance(entries_value, Sequence) and not isinstance(entries_value, (str, bytes)),
        "MISSING_REAL_REFERENCE_PROVENANCE",
        "formal plant fidelity report lacks formal_reference_evidence array",
    )
    normalized = _normalize_reference_entries(list(entries_value), document)
    set_hash = canonical_hash(normalized)
    _require(
        document.get("formal_reference_evidence_set_hash") == set_hash,
        "CROSS_BINDING_MISMATCH",
        "formal plant fidelity report reference-evidence set hash mismatch",
    )
    plant_entries_value = document.get("formal_plant_signal_evidence")
    _require(
        isinstance(plant_entries_value, Sequence) and not isinstance(plant_entries_value, (str, bytes)),
        "MISSING_PLANT_SIGNAL_PROVENANCE",
        "formal plant fidelity report lacks formal_plant_signal_evidence array",
    )
    normalized_plant = _normalize_plant_signal_entries(
        list(plant_entries_value),
        document,
        direct_artifacts,
    )
    _require(
        [entry["case_id"] for entry in normalized_plant] == [entry["case_id"] for entry in normalized],
        "CROSS_BINDING_MISMATCH",
        "formal plant-signal cases must exactly match real-reference cases",
    )
    plant_set_hash = canonical_hash(normalized_plant)
    _require(
        document.get("formal_plant_signal_evidence_set_hash") == plant_set_hash,
        "CROSS_BINDING_MISMATCH",
        "formal plant fidelity report plant-signal-evidence set hash mismatch",
    )
    return {
        "fidelity_verifier_source_path": str(verifier.path),
        "fidelity_verifier_source_hash": verifier.sha256,
        "formal_reference_evidence": normalized,
        "formal_reference_evidence_set_hash": set_hash,
        "formal_plant_signal_evidence": normalized_plant,
        "formal_plant_signal_evidence_set_hash": plant_set_hash,
    }


def _validate_isolation_evidence(
    artifact: BoundArtifact,
    document: Mapping[str, Any],
    release_artifact: BoundArtifact,
    direct_artifacts: Mapping[str, BoundArtifact],
) -> Dict[str, Any]:
    _reject_known_development_artifact(artifact, "formal controller/plant isolation evidence")
    _require(
        document.get("schema_version") == FORMAL_ISOLATION_EVIDENCE_SCHEMA_VERSION,
        "MALFORMED_ISOLATION_EVIDENCE",
        "formal controller/plant isolation evidence has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_ISOLATION_EVIDENCE_REPORT_TYPE,
        "MALFORMED_ISOLATION_EVIDENCE",
        "formal controller/plant isolation evidence has wrong report_type",
    )
    _require_formal_status(document, "formal controller/plant isolation evidence", "PASS")
    _require(
        document.get("formal_release_manifest_hash") == release_artifact.sha256,
        "CROSS_BINDING_MISMATCH",
        "formal controller/plant isolation evidence release hash mismatch",
    )
    _require(document.get("truth_topic") == TRUTH_TOPIC, "TRUTH_TOPIC_MISMATCH", "formal isolation evidence truth_topic mismatch")
    _require(
        document.get("observation_mode") == "STATIC_AND_LIVE_ROS_GRAPH",
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal isolation evidence must use STATIC_AND_LIVE_ROS_GRAPH",
    )
    expected = {
        "implementation_isolated_from_controller": True,
        "controller_hidden_state_access": False,
        "controller_state_import": False,
        "controller_subscription_to_truth": False,
        "plant_reads_raw_command": False,
    }
    for key, expected_value in expected.items():
        _require(
            document.get(key) == expected_value,
            "ISOLATION_SEMANTICS_MISMATCH",
            "formal isolation evidence.{} must be {!r}".format(key, expected_value),
        )
    _require(
        document.get("plant_subscriptions") == ["/odom"],
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal isolation evidence must show plant subscriptions exactly [/odom]",
    )
    checkpoints = _mapping(document.get("checkpoints"), "formal isolation evidence.checkpoints")
    _require(
        set(checkpoints) == {"ready", "pre_motion", "postflight"}
        and all(checkpoints.get(key) == "PASS" for key in checkpoints),
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal isolation evidence must PASS ready/pre_motion/postflight",
    )
    controller_nodes = document.get("controller_nodes")
    _require(
        isinstance(controller_nodes, list)
        and controller_nodes
        and controller_nodes == sorted(controller_nodes)
        and all(isinstance(node, str) and node.startswith("/") for node in controller_nodes),
        "ISOLATION_SEMANTICS_MISMATCH",
        "formal isolation evidence needs a sorted non-empty absolute controller node set",
    )
    graph_evidence = document.get("graph_evidence")
    _require(
        isinstance(graph_evidence, Sequence) and not isinstance(graph_evidence, (str, bytes)) and graph_evidence,
        "MALFORMED_ISOLATION_EVIDENCE",
        "formal isolation evidence requires hash-bound graph evidence",
    )
    for index, entry in enumerate(graph_evidence):
        _bound_artifact(entry, "formal isolation graph evidence[{}]".format(index))
    for report_key, direct in (
        ("plant_code_hash", direct_artifacts["plant_code"]),
        ("plant_input_schema_hash", direct_artifacts["plant_input_schema"]),
        ("plant_output_schema_hash", direct_artifacts["plant_output_schema"]),
    ):
        _require(
            document.get(report_key) == direct.sha256,
            "CROSS_BINDING_MISMATCH",
            "formal isolation evidence {} mismatch".format(report_key),
        )
    return {"controller_nodes": list(controller_nodes)}


def _validate_approval(
    artifact: BoundArtifact,
    document: Mapping[str, Any],
    release_artifact: BoundArtifact,
    fidelity_artifact: BoundArtifact,
    isolation_artifact: BoundArtifact,
    direct_artifacts: Mapping[str, BoundArtifact],
    fidelity_details: Mapping[str, Any],
    release_details: Mapping[str, Any],
) -> Dict[str, str]:
    _reject_known_development_artifact(artifact, "external formal liquid-plant approval")
    _require(
        document.get("schema_version") == FORMAL_APPROVAL_SCHEMA_VERSION,
        "MALFORMED_APPROVAL",
        "external formal liquid-plant approval has wrong schema_version",
    )
    _require(
        document.get("report_type") == FORMAL_APPROVAL_REPORT_TYPE,
        "MALFORMED_APPROVAL",
        "external formal liquid-plant approval has wrong report_type",
    )
    _require_formal_status(document, "external formal liquid-plant approval", "APPROVED")
    _require(
        document.get("external_approval") is True,
        "MALFORMED_APPROVAL",
        "formal liquid-plant approval must explicitly set external_approval=true",
    )
    approval_id = _nonempty_string(document.get("approval_id"), "external formal approval.approval_id")
    issuer = _nonempty_string(document.get("approval_authority"), "external formal approval.approval_authority")
    _nonempty_string(document.get("issued_at_utc"), "external formal approval.issued_at_utc")
    _require(
        document.get("approval_scope") == "SMPCC_SIM_FORMAL_PHYSICAL_PRIMARY",
        "MALFORMED_APPROVAL",
        "formal liquid-plant approval must cover SMPCC_SIM_FORMAL_PHYSICAL_PRIMARY",
    )
    _require(document.get("release_id") == release_details["release_id"], "CROSS_BINDING_MISMATCH", "approval release_id mismatch")
    _require(document.get("sim_freeze_id") == release_details["sim_freeze_id"], "CROSS_BINDING_MISMATCH", "approval sim_freeze_id mismatch")
    for key, expected in (
        ("formal_release_manifest_hash", release_artifact.sha256),
        ("fidelity_report_hash", fidelity_artifact.sha256),
        ("controller_isolation_evidence_hash", isolation_artifact.sha256),
        ("formal_reference_evidence_set_hash", fidelity_details["formal_reference_evidence_set_hash"]),
        ("formal_plant_signal_evidence_set_hash", fidelity_details["formal_plant_signal_evidence_set_hash"]),
    ):
        _require(document.get(key) == expected, "CROSS_BINDING_MISMATCH", "approval {} mismatch".format(key))
    for document_key, artifact_key in (
        ("plant_code_hash", "plant_code"),
        ("plant_parameter_hash", "plant_parameters"),
        ("plant_input_schema_hash", "plant_input_schema"),
        ("plant_output_schema_hash", "plant_output_schema"),
    ):
        _require(
            document.get(document_key) == direct_artifacts[artifact_key].sha256,
            "CROSS_BINDING_MISMATCH",
            "approval {} mismatch".format(document_key),
        )
    dimensions = _mapping(document.get("validation_dimensions"), "external formal approval.validation_dimensions")
    _require(
        set(dimensions) == set(DIMENSIONS) and all(dimensions.get(key) == "PASS" for key in DIMENSIONS),
        "FIDELITY_NOT_PASS",
        "external approval must attest five fidelity dimensions PASS",
    )
    return {"approval_id": approval_id, "approval_authority": issuer}


def _validate_request(document: Mapping[str, Any]) -> Dict[str, BoundArtifact]:
    required = {
        "schema_version",
        "request_id",
        "request_purpose",
        "formal",
        "development_only",
        "formal_release_manifest",
        "plant_code",
        "plant_parameters",
        "plant_input_schema",
        "plant_output_schema",
        "fidelity_report",
        "controller_isolation_evidence",
        "external_approval",
    }
    _require(set(document) == required, "MALFORMED_INPUT", "formal intake request has missing/unknown fields")
    _require(
        document.get("schema_version") == INTAKE_REQUEST_SCHEMA_VERSION,
        "MALFORMED_INPUT",
        "formal intake request has wrong schema_version",
    )
    _nonempty_string(document.get("request_id"), "formal intake request.request_id")
    _require(
        document.get("request_purpose") == "ASSEMBLE_EXTERNAL_FORMAL_CAPABILITY_ONLY",
        "MALFORMED_INPUT",
        "formal intake request must be limited to ASSEMBLE_EXTERNAL_FORMAL_CAPABILITY_ONLY",
    )
    _require(document.get("formal") is True, "NONFORMAL_ARTIFACT", "formal intake request must set formal=true")
    _require(
        document.get("development_only") is False,
        "DEVELOPMENT_ARTIFACT_FORBIDDEN",
        "formal intake request must set development_only=false",
    )
    return {
        key: _bound_artifact(document.get(key), "formal intake request.{}".format(key))
        for key in (
            "formal_release_manifest",
            "plant_code",
            "plant_parameters",
            "plant_input_schema",
            "plant_output_schema",
            "fidelity_report",
            "controller_isolation_evidence",
            "external_approval",
        )
    }


def assemble_formal_capability(
    intake_request_path: str | Path,
    intake_request_sha256: str,
) -> FormalCapabilityAssembly:
    """Validate external evidence and assemble (but do not write) a report.

    The caller supplies both the absolute intake-request path and its expected
    SHA-256.  This function has no ROS, Gazebo, process, or network side
    effects.  Any missing/changed/nonformal input raises
    :class:`FormalEvidenceError` instead of returning a best-effort result.
    """

    request_artifact = _bound_artifact(
        {"path": str(intake_request_path), "sha256": intake_request_sha256},
        "formal liquid-plant intake request",
    )
    request = _read_json(request_artifact.path, "formal liquid-plant intake request")
    artifacts = _validate_request(request)
    release_document = _read_json(artifacts["formal_release_manifest"].path, "formal plant release")
    direct_artifacts = {
        "plant_code": artifacts["plant_code"],
        "plant_parameters": artifacts["plant_parameters"],
        "plant_input_schema": artifacts["plant_input_schema"],
        "plant_output_schema": artifacts["plant_output_schema"],
    }
    # Reject known development assets before any cross-binding comparison.
    # Otherwise a caller could claim a stale release mismatch rather than get
    # the more important explicit development-artifact NO-GO.
    for label, direct_artifact in direct_artifacts.items():
        _reject_known_development_artifact(direct_artifact, "formal {}".format(label))
    release = _validate_release(artifacts["formal_release_manifest"], release_document, direct_artifacts)
    _validate_formal_code(direct_artifacts["plant_code"])
    parameters = _read_json_or_yaml(direct_artifacts["plant_parameters"].path, "formal plant parameters")
    _reject_known_development_artifact(direct_artifacts["plant_parameters"], "formal plant parameters")
    _validate_parameter_document(parameters, release["release_id"])
    input_schema = _read_json(direct_artifacts["plant_input_schema"].path, "formal plant input schema")
    _reject_known_development_artifact(direct_artifacts["plant_input_schema"], "formal plant input schema")
    _validate_input_schema(input_schema, release["release_id"])
    output_schema = _read_json(direct_artifacts["plant_output_schema"].path, "formal plant output schema")
    _reject_known_development_artifact(direct_artifacts["plant_output_schema"], "formal plant output schema")
    _validate_output_schema(output_schema, release["release_id"])

    fidelity_document = _read_json(artifacts["fidelity_report"].path, "formal plant fidelity report")
    fidelity = _validate_fidelity(
        artifacts["fidelity_report"],
        fidelity_document,
        artifacts["formal_release_manifest"],
        direct_artifacts,
    )
    isolation_document = _read_json(
        artifacts["controller_isolation_evidence"].path,
        "formal controller/plant isolation evidence",
    )
    isolation = _validate_isolation_evidence(
        artifacts["controller_isolation_evidence"],
        isolation_document,
        artifacts["formal_release_manifest"],
        direct_artifacts,
    )
    approval_document = _read_json(artifacts["external_approval"].path, "external formal liquid-plant approval")
    approval = _validate_approval(
        artifacts["external_approval"],
        approval_document,
        artifacts["formal_release_manifest"],
        artifacts["fidelity_report"],
        artifacts["controller_isolation_evidence"],
        direct_artifacts,
        fidelity,
        release,
    )

    common_fields: Dict[str, Any] = {
        "formal": True,
        "development_only": False,
        "physical_primary_eligible": True,
        "independent_plant": True,
        "implementation_isolated_from_controller": True,
        "controller_hidden_state_access": False,
        "driven_by": "executed_simulated_base_motion",
        "truth_topic": TRUTH_TOPIC,
        "fidelity_validation_status": "PASS",
        "plant_code_path": str(direct_artifacts["plant_code"].path),
        "plant_code_hash": direct_artifacts["plant_code"].sha256,
        "plant_parameter_path": str(direct_artifacts["plant_parameters"].path),
        "plant_parameter_hash": direct_artifacts["plant_parameters"].sha256,
        "plant_input_schema_path": str(direct_artifacts["plant_input_schema"].path),
        "plant_input_schema_hash": direct_artifacts["plant_input_schema"].sha256,
        "plant_output_schema_path": str(direct_artifacts["plant_output_schema"].path),
        "plant_output_schema_hash": direct_artifacts["plant_output_schema"].sha256,
        "fidelity_report_path": str(artifacts["fidelity_report"].path),
        "fidelity_report_hash": artifacts["fidelity_report"].sha256,
        "formal_intake_tool_id": TOOL_ID,
        "formal_intake_request_path": str(request_artifact.path),
        "formal_intake_request_hash": request_artifact.sha256,
        "formal_release_manifest_path": str(artifacts["formal_release_manifest"].path),
        "formal_release_manifest_hash": artifacts["formal_release_manifest"].sha256,
        "external_approval_path": str(artifacts["external_approval"].path),
        "external_approval_hash": artifacts["external_approval"].sha256,
        "external_approval_id": approval["approval_id"],
        "external_approval_authority": approval["approval_authority"],
        # This repository has no pinned approval public key or organizational
        # trust store.  The approval document is structurally hash-bound, but
        # this offline assembler must not claim that it authenticated its
        # issuer.  A formal governance process may impose a stronger policy.
        "cryptographic_trust_anchor": CRYPTOGRAPHIC_TRUST_ANCHOR_STATUS,
        "external_approval_authentication_status": EXTERNAL_APPROVAL_AUTHENTICATION_STATUS,
        "controller_isolation_evidence_path": str(artifacts["controller_isolation_evidence"].path),
        "controller_isolation_evidence_hash": artifacts["controller_isolation_evidence"].sha256,
        "controller_nodes": isolation["controller_nodes"],
        "fidelity_verifier_source_path": fidelity["fidelity_verifier_source_path"],
        "fidelity_verifier_source_hash": fidelity["fidelity_verifier_source_hash"],
        "formal_reference_evidence": fidelity["formal_reference_evidence"],
        "formal_reference_evidence_set_hash": fidelity["formal_reference_evidence_set_hash"],
        "formal_plant_signal_evidence": fidelity["formal_plant_signal_evidence"],
        "formal_plant_signal_evidence_set_hash": fidelity["formal_plant_signal_evidence_set_hash"],
        "fidelity_report_generated_by_intake": False,
        "runtime_execution_performed": False,
    }
    report: Dict[str, Any] = {
        "report_type": FORMAL_CAPABILITY_REPORT_TYPE,
        "report_schema_version": FORMAL_CAPABILITY_REPORT_SCHEMA_VERSION,
        "tool_id": TOOL_ID,
        "generated_at_utc": utc_now(),
        "status": "PASS",
        **common_fields,
    }
    report["capability_report_payload_hash"] = canonical_hash(report)
    return FormalCapabilityAssembly(capability_report=report, capability_fields=common_fields)


def make_toolchain_capability_binding(
    assembly: FormalCapabilityAssembly,
    capability_report_path: str | Path,
    capability_report_hash: str,
) -> Dict[str, Any]:
    """Create the direct ``liquid_plant_capability`` mapping for a freeze.

    The report is written first, then this separate binding records its exact
    file hash.  Keeping them separate avoids an impossible self-hash while
    preserving the existing toolchain ABI.
    """

    path = _absolute_existing_path(str(capability_report_path), "formal intake capability report")
    _require(is_sha256(capability_report_hash), "MISSING_HASH", "formal intake capability report hash is invalid")
    _require(
        sha256_file(path) == capability_report_hash,
        "HASH_MISMATCH",
        "formal intake capability report changed before toolchain binding",
    )
    report = _read_json(path, "formal intake capability report")
    _require(
        report == assembly.capability_report,
        "HASH_MISMATCH",
        "formal intake capability report differs from the validated in-memory assembly",
    )
    binding: Dict[str, Any] = {
        "schema_version": TOOLCHAIN_BINDING_SCHEMA_VERSION,
        "binding_type": TOOLCHAIN_BINDING_TYPE,
        "status": "PASS",
        **dict(assembly.capability_fields),
        "plant_capability_report_path": str(path),
        "plant_capability_report_hash": capability_report_hash,
        "formal_intake_report_path": str(path),
        "formal_intake_report_hash": capability_report_hash,
    }
    binding["binding_payload_hash"] = canonical_hash(binding)
    return binding


def _validate_output_path(value: str | Path, label: str) -> Path:
    path = Path(value)
    _require(path.is_absolute(), "MALFORMED_OUTPUT", "{} must be absolute".format(label))
    _require(path.parent.is_dir(), "MALFORMED_OUTPUT", "{} parent directory is missing".format(label))
    _require(not path.exists(), "OUTPUT_EXISTS", "{} already exists: {}".format(label, path))
    return path.resolve()


def _write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    try:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(path, 0o444)
    except FileExistsError:
        _fail("OUTPUT_EXISTS", "output already exists: {}".format(path))
    except OSError as exc:
        _fail("OUTPUT_WRITE_FAILED", "cannot write {}: {}".format(path, exc))


def write_formal_capability_bundle(
    assembly: FormalCapabilityAssembly,
    capability_report_output: str | Path,
    toolchain_binding_output: str | Path,
) -> Dict[str, Any]:
    """Write immutable report and binding after all evidence was validated.

    Existing files are never overwritten.  The function writes no fidelity
    report and performs no runtime action; it serializes only the already
    validated external-evidence intake result.
    """

    report_path = _validate_output_path(capability_report_output, "capability report output")
    binding_path = _validate_output_path(toolchain_binding_output, "toolchain binding output")
    _require(report_path != binding_path, "MALFORMED_OUTPUT", "report and toolchain binding outputs must differ")
    _write_json_new(report_path, assembly.capability_report)
    report_hash = sha256_file(report_path)
    binding = make_toolchain_capability_binding(assembly, report_path, report_hash)
    _write_json_new(binding_path, binding)
    return {
        "capability_report_path": str(report_path),
        "capability_report_hash": report_hash,
        "toolchain_binding_path": str(binding_path),
        "toolchain_binding_hash": sha256_file(binding_path),
        "status": "PASS",
        "formal": True,
        "runtime_execution_performed": False,
    }
