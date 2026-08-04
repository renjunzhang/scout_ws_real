#!/usr/bin/env python3
"""Two-phase, fail-closed intake for an externally approved formal freeze.

This tool deliberately does *not* select a controller, derive a path,
generate a profile/randomisation table, estimate liquid fidelity, launch ROS,
or start Gazebo.  It only copies an already-existing, hash-bound external
freeze payload into an immutable pre-receipt artifact, then attaches a
separately supplied external receipt after that receipt binds the payload's
canonical digest.

The split avoids the usual receipt/freeze circularity:

1. ``prepare`` verifies every direct artifact reference declared by the
   external payload, verifies the shared formal gate except for its purposely
   absent receipt, and writes the exact source JSON bytes as a read-only
   pre-receipt payload plus a read-only receipt request.
2. An independent reviewer supplies a receipt that names the immutable
   canonical payload digest and the receipt-request hash.
3. ``finalize`` re-verifies every input and the *real* shared formal gate
   before it writes one read-only final freeze.  It never writes a receipt.

Consequently a missing, changed, development, fixture, proxy, W5_S10, or
otherwise nonconforming input remains ``NO_GO``.  Test code may construct
ephemeral dummy artifacts, but this module contains no formal fixture or
fallback release data of its own.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


def _load_toolchain():
    """Load the sibling protocol module without a package installation."""
    existing = sys.modules.get("smpcc_sim_toolchain")
    if existing is not None:
        return existing
    module_path = Path(__file__).with_name("smpcc_sim_toolchain.py")
    spec = importlib.util.spec_from_file_location("smpcc_sim_toolchain", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SMPCC-SIM toolchain: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["smpcc_sim_toolchain"] = module
    spec.loader.exec_module(module)
    return module


toolchain = _load_toolchain()


INTAKE_TOOL_ID = "SMPCC-SIM-FORMAL-FREEZE-INTAKE-v1"
DECLARATION_SCHEMA_VERSION = "smpcc-sim-formal-freeze-external-declarations-v1"
DECLARATION_DOCUMENT_TYPE = "SMPCC_SIM_FORMAL_FREEZE_EXTERNAL_DECLARATIONS"
DECLARATION_PURPOSE = "ASSEMBLE_FORMAL_FREEZE_FROM_EXTERNAL_EVIDENCE_ONLY"
RECEIPT_REQUEST_SCHEMA_VERSION = "smpcc-sim-formal-freeze-receipt-request-v1"
RECEIPT_REQUEST_DOCUMENT_TYPE = "SMPCC_SIM_FORMAL_FREEZE_RECEIPT_REQUEST"
RECEIPT_SCHEMA_VERSION = "smpcc-sim-formal-freeze-receipt-v1"
RECEIPT_DOCUMENT_TYPE = "SMPCC_SIM_FORMAL_FREEZE_RECEIPT"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class FormalFreezeIntakeError(RuntimeError):
    """A formal-freeze intake error that must prevent output creation."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def _fail(code: str, message: str) -> None:
    raise FormalFreezeIntakeError(code, message)


def _require(condition: bool, code: str, message: str) -> None:
    if not condition:
        _fail(code, message)


def _is_sha256(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _canonical_hash(value: Any) -> str:
    return str(toolchain.canonical_hash(value))


def _sha256_file(path: Path) -> str:
    return str(toolchain.sha256_file(path))


@dataclass(frozen=True)
class BoundArtifact:
    """An absolute existing file that matches a caller-supplied SHA-256."""

    path: Path
    sha256: str

    def descriptor(self) -> Dict[str, str]:
        return {"path": str(self.path), "sha256": self.sha256}


def _as_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        _fail("MALFORMED_INPUT", f"{label} must be a JSON object")
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        _fail("MALFORMED_INPUT", f"{label} must be a non-empty string")
    return value.strip()


def _bound_artifact(value: Any, label: str) -> BoundArtifact:
    descriptor = _as_mapping(value, label)
    _require(
        set(descriptor) == {"path", "sha256"},
        "MALFORMED_INPUT",
        f"{label} descriptor must contain exactly path and sha256",
    )
    raw_path = _nonempty_string(descriptor.get("path"), f"{label}.path")
    path = Path(raw_path)
    _require(path.is_absolute(), "MALFORMED_INPUT", f"{label} path must be absolute: {path}")
    _require(path.is_file(), "MISSING_ARTIFACT", f"{label} file is missing: {path}")
    expected = descriptor.get("sha256")
    _require(_is_sha256(expected), "MISSING_HASH", f"{label} sha256 must be a lowercase SHA-256")
    resolved = path.resolve()
    actual = _sha256_file(resolved)
    _require(actual == expected, "HASH_MISMATCH", f"{label} SHA-256 mismatch for {resolved}")
    return BoundArtifact(path=resolved, sha256=actual)


def _read_json(artifact: BoundArtifact, label: str) -> Mapping[str, Any]:
    try:
        with artifact.path.open("r", encoding="utf-8") as stream:
            return _as_mapping(json.load(stream), label)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        _fail("MALFORMED_JSON", f"cannot read {label}: {exc}")
    raise AssertionError("unreachable")


def _assert_artifact_unchanged(artifact: BoundArtifact, label: str) -> None:
    _require(artifact.path.is_file(), "MISSING_ARTIFACT", f"{label} disappeared: {artifact.path}")
    _require(
        _sha256_file(artifact.path) == artifact.sha256,
        "HASH_MISMATCH",
        f"{label} changed after it was hash-bound: {artifact.path}",
    )


def _validate_output_path(value: Path, label: str) -> Path:
    path = Path(value)
    _require(path.is_absolute(), "MALFORMED_OUTPUT", f"{label} must be an absolute path")
    _require(path.parent.is_dir(), "MALFORMED_OUTPUT", f"{label} parent directory is missing: {path.parent}")
    _require(not path.exists(), "OUTPUT_EXISTS", f"{label} already exists: {path}")
    return path.resolve()


def _write_bytes_new_readonly(path: Path, payload: bytes, label: str) -> None:
    """Write exactly once, fsync, and remove all write bits.

    ``open(..., 'xb')`` makes accidental output overwrite fail even if a
    caller races another invocation.  A partial pre-receipt artifact is not a
    formal freeze, and finalization never accepts it without the companion
    hash-bound receipt request.
    """
    try:
        with path.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(path, 0o444)
    except FileExistsError:
        _fail("OUTPUT_EXISTS", f"{label} already exists: {path}")
    except OSError as exc:
        _fail("OUTPUT_WRITE_FAILED", f"cannot write {label} {path}: {exc}")


def _json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _write_json_new_readonly(path: Path, value: Mapping[str, Any], label: str) -> None:
    _write_bytes_new_readonly(path, _json_bytes(value), label)


def _descriptor_from_path_hash(path: Path, sha256: str) -> Dict[str, str]:
    return {"path": str(path.resolve()), "sha256": sha256}


def _payload_hash_pair(mapping: Mapping[str, Any], key: str) -> Optional[str]:
    """Return the hash field paired with one path/file field in a freeze.

    The shared formal gate intentionally uses a few historical names such as
    ``effective_config_file_hash`` and ``profile_hash``.  This small explicit
    map prevents intake from silently treating those artifact references as
    untracked convenience strings.
    """
    special = {
        "map_file": "map_hash",
        "world_file": "world_hash",
        "world_geometry_file": "world_geometry_hash",
        "robot_model_file": "robot_model_hash",
        "physical_parameter_file": "physical_parameter_hash",
        "effective_config_path": "effective_config_file_hash",
        "profile_path": "profile_hash",
        "policy_path": "policy_file_hash",
        "contract_path": "contract_file_hash",
        "registry_path": "registry_file_hash",
    }
    if key in special:
        return special[key]
    if key.endswith("_path"):
        stem = key[: -len("_path")]
        candidates = (f"{stem}_hash", f"{stem}_file_hash")
        present = [candidate for candidate in candidates if candidate in mapping]
        if len(present) == 1:
            return present[0]
        if len(present) > 1:
            _fail("MALFORMED_INPUT", f"artifact path {key} has ambiguous hash fields: {present}")
        return None
    return None


def _collect_direct_payload_artifacts(value: Any, location: str = "freeze") -> List[BoundArtifact]:
    """Collect every direct ``path/hash`` reference in the external payload.

    This is intentionally limited to the payload itself.  Documents reachable
    through it are separately parsed and hash-checked by the shared formal
    gate, so the intake neither reimplements nor weakens their validators.
    """
    discovered: List[BoundArtifact] = []
    if isinstance(value, Mapping):
        for key, raw in value.items():
            if not isinstance(key, str):
                _fail("MALFORMED_INPUT", f"{location} contains a non-string object key")
            hash_key = _payload_hash_pair(value, key)
            if hash_key is not None:
                if isinstance(raw, str) and raw.startswith("/"):
                    descriptor = {"path": raw, "sha256": value.get(hash_key)}
                    discovered.append(_bound_artifact(descriptor, f"{location}.{key}"))
                elif raw is not None:
                    _fail("MALFORMED_INPUT", f"{location}.{key} must be an absolute artifact path")
            elif key.endswith("_path") and isinstance(raw, str) and raw.startswith("/"):
                _fail(
                    "UNDECLARED_ARTIFACT_REFERENCE",
                    f"{location}.{key} has no recognised hash pair; external artifacts must be hash-bound",
                )
            elif key.endswith("_file") and isinstance(raw, str) and raw.startswith("/"):
                _fail(
                    "UNDECLARED_ARTIFACT_REFERENCE",
                    f"{location}.{key} has no recognised hash pair; external artifacts must be hash-bound",
                )
        for key, raw in value.items():
            discovered.extend(_collect_direct_payload_artifacts(raw, f"{location}.{key}"))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            discovered.extend(_collect_direct_payload_artifacts(item, f"{location}[{index}]"))
    return discovered


def _validate_artifact_declarations(value: Any) -> List[Tuple[str, BoundArtifact]]:
    _require(isinstance(value, list) and value, "MALFORMED_INPUT", "artifact_declarations must be a non-empty list")
    output: List[Tuple[str, BoundArtifact]] = []
    roles: List[str] = []
    paths: set[str] = set()
    for index, raw in enumerate(value):
        item = _as_mapping(raw, f"artifact_declarations[{index}]")
        _require(
            set(item) == {"role", "path", "sha256"},
            "MALFORMED_INPUT",
            f"artifact_declarations[{index}] must contain exactly role, path, sha256",
        )
        role = _nonempty_string(item.get("role"), f"artifact_declarations[{index}].role")
        _require(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}", role) is not None, "MALFORMED_INPUT", f"artifact declaration role is invalid: {role!r}")
        roles.append(role)
        artifact = _bound_artifact({"path": item.get("path"), "sha256": item.get("sha256")}, f"artifact_declarations[{index}]")
        _require(str(artifact.path) not in paths, "MALFORMED_INPUT", f"artifact_declarations repeats path: {artifact.path}")
        paths.add(str(artifact.path))
        output.append((role, artifact))
    _require(len(roles) == len(set(roles)), "MALFORMED_INPUT", "artifact declaration roles must be unique")
    _require(roles == sorted(roles), "MALFORMED_INPUT", "artifact declarations must be sorted by role")
    return output


def _pre_receipt_errors(gate_report: Mapping[str, Any]) -> List[str]:
    """Return all formal-gate errors except its intentionally absent receipt."""
    raw_errors = gate_report.get("errors")
    if not isinstance(raw_errors, list) or not all(isinstance(item, str) for item in raw_errors):
        return ["shared formal gate returned malformed error report"]
    return [error for error in raw_errors if error != "external formal freeze receipt is missing"]


@dataclass(frozen=True)
class PreparedPayload:
    """Validated pre-receipt payload and its all-external evidence boundary."""

    declaration: BoundArtifact
    payload_source: BoundArtifact
    payload_document: Mapping[str, Any]
    direct_artifacts: Tuple[BoundArtifact, ...]
    artifact_declarations: Tuple[Tuple[str, BoundArtifact], ...]
    freeze_payload_hash: str


def prepare_external_payload(declaration_path: str | Path, declaration_sha256: str) -> PreparedPayload:
    """Validate an external-declarations document without writing anything.

    It cannot manufacture a missing formal input: the real formal gate must
    return only its expected "receipt missing" error.  All other missing or
    invalid evidence remains a NO_GO before a reviewer is asked to issue a
    receipt.
    """
    declaration = _bound_artifact(
        {"path": str(declaration_path), "sha256": declaration_sha256},
        "formal freeze external declarations",
    )
    document = _read_json(declaration, "formal freeze external declarations")
    required = {
        "schema_version",
        "document_type",
        "declaration_id",
        "purpose",
        "formal",
        "development_only",
        "protocol_id",
        "pre_receipt_payload",
        "artifact_declarations",
    }
    _require(set(document) == required, "MALFORMED_INPUT", "external declarations has missing/unknown fields")
    _require(document.get("schema_version") == DECLARATION_SCHEMA_VERSION, "MALFORMED_INPUT", "external declarations schema_version mismatch")
    _require(document.get("document_type") == DECLARATION_DOCUMENT_TYPE, "MALFORMED_INPUT", "external declarations document_type mismatch")
    _nonempty_string(document.get("declaration_id"), "external declarations.declaration_id")
    _require(document.get("purpose") == DECLARATION_PURPOSE, "MALFORMED_INPUT", "external declarations purpose is not external-evidence-only assembly")
    _require(document.get("formal") is True, "NONFORMAL_ARTIFACT", "external declarations must set formal=true")
    _require(document.get("development_only") is False, "DEVELOPMENT_ARTIFACT_FORBIDDEN", "external declarations must set development_only=false")
    _require(document.get("protocol_id") == toolchain.FORMAL_PROTOCOL_ID, "MALFORMED_INPUT", "external declarations protocol_id mismatch")

    payload_source = _bound_artifact(document.get("pre_receipt_payload"), "external declarations.pre_receipt_payload")
    payload = _read_json(payload_source, "external pre-receipt payload")
    _require("formal_freeze_receipt" not in payload, "CIRCULAR_RECEIPT", "pre-receipt payload must not already contain formal_freeze_receipt")
    _require(payload.get("fixture") is not True and payload.get("mode") != "fixture", "NONFORMAL_ARTIFACT", "fixture payload can never enter formal intake")
    _require(not toolchain.has_forbidden_w5(payload), "REJECTED_BSLOSH_LINEAGE", "W5_S10 is rejected and cannot enter formal intake")

    declared = _validate_artifact_declarations(document.get("artifact_declarations"))
    direct = _collect_direct_payload_artifacts(payload)
    direct_by_path: Dict[str, str] = {}
    for artifact in direct:
        old = direct_by_path.get(str(artifact.path))
        _require(old in (None, artifact.sha256), "HASH_MISMATCH", f"payload binds one artifact to conflicting hashes: {artifact.path}")
        direct_by_path[str(artifact.path)] = artifact.sha256
    declared_by_path = {str(artifact.path): artifact.sha256 for _role, artifact in declared}
    _require(
        direct_by_path == declared_by_path,
        "UNDECLARED_ARTIFACT_REFERENCE",
        "artifact_declarations must cover exactly every direct payload artifact reference",
    )

    gate = toolchain.validate_formal_freeze(payload)
    nonreceipt_errors = _pre_receipt_errors(gate)
    _require(
        gate.get("status") == "FAIL" and not nonreceipt_errors,
        "FORMAL_GATE_NO_GO",
        "external payload does not pass every formal prerequisite before receipt: " + "; ".join(nonreceipt_errors or ["shared formal gate did not report the expected missing receipt"]),
    )
    return PreparedPayload(
        declaration=declaration,
        payload_source=payload_source,
        payload_document=payload,
        direct_artifacts=tuple(sorted({(str(item.path), item.sha256): item for item in direct}.values(), key=lambda item: str(item.path))),
        artifact_declarations=tuple(declared),
        freeze_payload_hash=_canonical_hash(payload),
    )


def _receipt_request_document(prepared: PreparedPayload, pre_payload_output: BoundArtifact) -> Dict[str, Any]:
    declared = [
        {"role": role, **artifact.descriptor()}
        for role, artifact in prepared.artifact_declarations
    ]
    document: Dict[str, Any] = {
        "schema_version": RECEIPT_REQUEST_SCHEMA_VERSION,
        "document_type": RECEIPT_REQUEST_DOCUMENT_TYPE,
        "status": "PENDING_EXTERNAL_FORMAL_FREEZE_RECEIPT",
        "intake_tool_id": INTAKE_TOOL_ID,
        "formal": True,
        "development_only": False,
        "protocol_id": toolchain.FORMAL_PROTOCOL_ID,
        "freeze_id": prepared.payload_document.get("sim_freeze_id"),
        "freeze_payload_hash": prepared.freeze_payload_hash,
        "source_payload": prepared.payload_source.descriptor(),
        "pre_receipt_payload": pre_payload_output.descriptor(),
        "external_declarations": prepared.declaration.descriptor(),
        "artifact_declarations": declared,
        "artifact_declarations_hash": _canonical_hash(declared),
    }
    return document


def _validate_receipt_request(document: Mapping[str, Any]) -> None:
    required = {
        "schema_version",
        "document_type",
        "status",
        "intake_tool_id",
        "formal",
        "development_only",
        "protocol_id",
        "freeze_id",
        "freeze_payload_hash",
        "source_payload",
        "pre_receipt_payload",
        "external_declarations",
        "artifact_declarations",
        "artifact_declarations_hash",
    }
    _require(set(document) == required, "MALFORMED_INPUT", "receipt request has missing/unknown fields")
    _require(document.get("schema_version") == RECEIPT_REQUEST_SCHEMA_VERSION, "MALFORMED_INPUT", "receipt request schema_version mismatch")
    _require(document.get("document_type") == RECEIPT_REQUEST_DOCUMENT_TYPE, "MALFORMED_INPUT", "receipt request document_type mismatch")
    _require(document.get("status") == "PENDING_EXTERNAL_FORMAL_FREEZE_RECEIPT", "MALFORMED_INPUT", "receipt request status mismatch")
    _require(document.get("intake_tool_id") == INTAKE_TOOL_ID, "MALFORMED_INPUT", "receipt request intake_tool_id mismatch")
    _require(document.get("formal") is True and document.get("development_only") is False, "NONFORMAL_ARTIFACT", "receipt request must be formal and non-development")
    _require(document.get("protocol_id") == toolchain.FORMAL_PROTOCOL_ID, "MALFORMED_INPUT", "receipt request protocol_id mismatch")
    _nonempty_string(document.get("freeze_id"), "receipt request.freeze_id")
    _require(_is_sha256(document.get("freeze_payload_hash")), "MISSING_HASH", "receipt request freeze_payload_hash is invalid")
    _bound_artifact(document.get("source_payload"), "receipt request.source_payload")
    _bound_artifact(document.get("pre_receipt_payload"), "receipt request.pre_receipt_payload")
    _bound_artifact(document.get("external_declarations"), "receipt request.external_declarations")
    declared = _validate_artifact_declarations(document.get("artifact_declarations"))
    declared_wire = [{"role": role, **artifact.descriptor()} for role, artifact in declared]
    _require(document.get("artifact_declarations") == declared_wire, "MALFORMED_INPUT", "receipt request artifact declarations are not canonical")
    _require(document.get("artifact_declarations_hash") == _canonical_hash(declared_wire), "HASH_MISMATCH", "receipt request artifact_declarations_hash mismatch")


def write_pre_receipt_bundle(
    prepared: PreparedPayload,
    pre_receipt_output: str | Path,
    receipt_request_output: str | Path,
) -> Dict[str, Any]:
    """Write exact source payload bytes and its independent receipt request.

    This function serializes neither a controller release nor a receipt.  It
    copies the already hash-bound external JSON bytes so the external reviewer
    can inspect the exact pre-receipt payload that its receipt will bind.
    """
    pre_path = _validate_output_path(Path(pre_receipt_output), "pre-receipt payload output")
    request_path = _validate_output_path(Path(receipt_request_output), "receipt request output")
    _require(pre_path != request_path, "MALFORMED_OUTPUT", "pre-receipt and receipt-request outputs must differ")
    _require(pre_path != prepared.payload_source.path and request_path != prepared.payload_source.path, "MALFORMED_OUTPUT", "output must not alias external source payload")

    _assert_artifact_unchanged(prepared.declaration, "external declarations")
    _assert_artifact_unchanged(prepared.payload_source, "external pre-receipt payload")
    for artifact in prepared.direct_artifacts:
        _assert_artifact_unchanged(artifact, "direct formal payload artifact")
    for _role, artifact in prepared.artifact_declarations:
        _assert_artifact_unchanged(artifact, "declared formal artifact")

    source_bytes = prepared.payload_source.path.read_bytes()
    _require(_sha256_file(prepared.payload_source.path) == prepared.payload_source.sha256, "HASH_MISMATCH", "external pre-receipt source changed before write")
    _write_bytes_new_readonly(pre_path, source_bytes, "pre-receipt payload")
    pre_artifact = _bound_artifact(_descriptor_from_path_hash(pre_path, _sha256_file(pre_path)), "written pre-receipt payload")
    _require(_canonical_hash(_read_json(pre_artifact, "written pre-receipt payload")) == prepared.freeze_payload_hash, "HASH_MISMATCH", "written pre-receipt payload canonical digest changed")
    request = _receipt_request_document(prepared, pre_artifact)
    _write_json_new_readonly(request_path, request, "receipt request")
    request_hash = _sha256_file(request_path)
    return {
        "status": "READY_FOR_EXTERNAL_RECEIPT_NOT_FORMAL",
        "formal": False,
        "pre_receipt_payload_path": str(pre_path),
        "pre_receipt_payload_hash": pre_artifact.sha256,
        "receipt_request_path": str(request_path),
        "receipt_request_hash": request_hash,
        "freeze_payload_hash": prepared.freeze_payload_hash,
        "external_artifact_count": len(prepared.artifact_declarations),
    }


def _validate_external_receipt(
    receipt: Mapping[str, Any],
    request: Mapping[str, Any],
    request_hash: str,
    payload: Mapping[str, Any],
) -> None:
    required = {
        "schema_version",
        "report_type",
        "status",
        "formal",
        "development_only",
        "receipt_id",
        "receipt_authority",
        "protocol_id",
        "freeze_id",
        "freeze_payload_hash",
        "pre_receipt_request_hash",
        "validator_hash",
    }
    _require(set(receipt) == required, "MALFORMED_RECEIPT", "external formal freeze receipt has missing/unknown fields")
    _require(receipt.get("schema_version") == RECEIPT_SCHEMA_VERSION, "MALFORMED_RECEIPT", "external formal freeze receipt schema_version mismatch")
    _require(receipt.get("report_type") == RECEIPT_DOCUMENT_TYPE, "MALFORMED_RECEIPT", "external formal freeze receipt report_type mismatch")
    _require(receipt.get("status") == "PASS", "RECEIPT_NOT_PASS", "external formal freeze receipt is not PASS")
    _require(receipt.get("formal") is True and receipt.get("development_only") is False, "NONFORMAL_ARTIFACT", "external formal freeze receipt must be formal and non-development")
    _nonempty_string(receipt.get("receipt_id"), "external formal freeze receipt.receipt_id")
    _nonempty_string(receipt.get("receipt_authority"), "external formal freeze receipt.receipt_authority")
    _require(receipt.get("protocol_id") == toolchain.FORMAL_PROTOCOL_ID, "MALFORMED_RECEIPT", "external formal freeze receipt protocol_id mismatch")
    _require(receipt.get("freeze_id") == request.get("freeze_id") == payload.get("sim_freeze_id"), "RECEIPT_BINDING_MISMATCH", "external formal freeze receipt freeze_id does not bind pre-receipt payload")
    _require(receipt.get("freeze_payload_hash") == _canonical_hash(payload) == request.get("freeze_payload_hash"), "RECEIPT_BINDING_MISMATCH", "external formal freeze receipt does not bind immutable pre-receipt payload digest")
    _require(receipt.get("pre_receipt_request_hash") == request_hash, "RECEIPT_BINDING_MISMATCH", "external formal freeze receipt does not bind the supplied receipt request")
    _require(_is_sha256(receipt.get("validator_hash")), "MISSING_HASH", "external formal freeze receipt validator_hash is invalid")
    _require(not toolchain.has_forbidden_w5(receipt), "REJECTED_BSLOSH_LINEAGE", "external formal freeze receipt revives rejected W5_S10")


def finalize_formal_freeze(
    pre_receipt_payload_path: str | Path,
    pre_receipt_payload_sha256: str,
    receipt_request_path: str | Path,
    receipt_request_sha256: str,
    receipt_path: str | Path,
    receipt_sha256: str,
    output_path: str | Path,
) -> Dict[str, Any]:
    """Attach an external receipt and write a formal freeze only on PASS.

    No synthetic receipt is created in memory.  The only constructed value is
    the final freeze mapping whose receipt *pointer* is exactly the caller's
    separately hash-bound external receipt.  The shared formal gate is the
    final authority and is run before and after the no-overwrite write.
    """
    pre_payload = _bound_artifact(
        {"path": str(pre_receipt_payload_path), "sha256": pre_receipt_payload_sha256},
        "pre-receipt payload",
    )
    request_artifact = _bound_artifact(
        {"path": str(receipt_request_path), "sha256": receipt_request_sha256},
        "receipt request",
    )
    receipt_artifact = _bound_artifact(
        {"path": str(receipt_path), "sha256": receipt_sha256},
        "external formal freeze receipt",
    )
    request = _read_json(request_artifact, "receipt request")
    _validate_receipt_request(request)
    _require(request.get("pre_receipt_payload") == pre_payload.descriptor(), "RECEIPT_BINDING_MISMATCH", "receipt request does not bind the supplied pre-receipt payload")
    payload = _read_json(pre_payload, "pre-receipt payload")
    _require("formal_freeze_receipt" not in payload, "CIRCULAR_RECEIPT", "pre-receipt payload already contains a receipt")
    _require(payload.get("sim_freeze_id") == request.get("freeze_id"), "RECEIPT_BINDING_MISMATCH", "pre-receipt payload freeze_id differs from receipt request")
    _require(_canonical_hash(payload) == request.get("freeze_payload_hash"), "RECEIPT_BINDING_MISMATCH", "pre-receipt payload digest differs from receipt request")

    # Re-validate the original external declaration and every direct artifact
    # before writing.  The request records the source declaration, so a caller
    # cannot substitute a pre-payload whose source evidence later changed.
    declaration_artifact = _bound_artifact(request.get("external_declarations"), "receipt request.external_declarations")
    prepared = prepare_external_payload(str(declaration_artifact.path), declaration_artifact.sha256)
    _require(
        prepared.freeze_payload_hash == _canonical_hash(payload)
        and prepared.payload_source.descriptor() == request.get("source_payload"),
        "RECEIPT_BINDING_MISMATCH",
        "receipt request/source payload no longer matches validated external declarations",
    )

    receipt = _read_json(receipt_artifact, "external formal freeze receipt")
    _validate_external_receipt(receipt, request, request_artifact.sha256, payload)
    final_freeze: Dict[str, Any] = dict(payload)
    final_freeze["formal_freeze_receipt"] = {
        "report_path": str(receipt_artifact.path),
        "report_hash": receipt_artifact.sha256,
    }
    gate = toolchain.validate_formal_freeze(final_freeze)
    _require(
        gate.get("status") == "PASS",
        "FORMAL_GATE_NO_GO",
        "shared formal gate rejected final freeze: " + "; ".join(str(item) for item in gate.get("errors", [])),
    )

    final_path = _validate_output_path(Path(output_path), "formal freeze output")
    _require(final_path not in {pre_payload.path, request_artifact.path, receipt_artifact.path}, "MALFORMED_OUTPUT", "formal freeze output must not alias an input artifact")
    _write_json_new_readonly(final_path, final_freeze, "formal freeze")
    final_artifact = _bound_artifact(_descriptor_from_path_hash(final_path, _sha256_file(final_path)), "written formal freeze")
    final_gate = toolchain.validate_formal_freeze(_read_json(final_artifact, "written formal freeze"))
    _require(final_gate.get("status") == "PASS", "FORMAL_GATE_NO_GO", "written formal freeze failed shared formal gate")
    return {
        "status": "PASS",
        "formal": True,
        "formal_freeze_path": str(final_path),
        "formal_freeze_file_hash": final_artifact.sha256,
        "freeze_hash": final_gate.get("freeze_hash"),
        "freeze_payload_hash": request.get("freeze_payload_hash"),
        "receipt_path": str(receipt_artifact.path),
        "receipt_hash": receipt_artifact.sha256,
    }


def _emit(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def command_preflight(args: argparse.Namespace) -> int:
    prepared = prepare_external_payload(args.declarations, args.declarations_sha256)
    _emit(
        {
            "status": "READY_FOR_EXTERNAL_RECEIPT_NOT_FORMAL",
            "formal": False,
            "freeze_payload_hash": prepared.freeze_payload_hash,
            "external_artifact_count": len(prepared.artifact_declarations),
            "message": "all non-receipt formal gates PASS; no formal freeze or receipt was written",
        }
    )
    return 0


def command_prepare(args: argparse.Namespace) -> int:
    prepared = prepare_external_payload(args.declarations, args.declarations_sha256)
    _emit(write_pre_receipt_bundle(prepared, args.pre_receipt_output, args.receipt_request_output))
    return 0


def command_finalize(args: argparse.Namespace) -> int:
    _emit(
        finalize_formal_freeze(
            args.pre_receipt_payload,
            args.pre_receipt_payload_sha256,
            args.receipt_request,
            args.receipt_request_sha256,
            args.receipt,
            args.receipt_sha256,
            args.output,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Two-phase fail-closed SMPCC-SIM formal-freeze intake")
    sub = parser.add_subparsers(dest="command", required=True)

    item = sub.add_parser("preflight", help="check hash-bound external declarations; writes nothing")
    item.add_argument("--declarations", type=Path, required=True)
    item.add_argument("--declarations-sha256", required=True)
    item.set_defaults(func=command_preflight)

    item = sub.add_parser("prepare", help="write immutable pre-receipt payload and external receipt request")
    item.add_argument("--declarations", type=Path, required=True)
    item.add_argument("--declarations-sha256", required=True)
    item.add_argument("--pre-receipt-output", type=Path, required=True)
    item.add_argument("--receipt-request-output", type=Path, required=True)
    item.set_defaults(func=command_prepare)

    item = sub.add_parser("finalize", help="attach independently supplied receipt only if shared formal gate PASSes")
    item.add_argument("--pre-receipt-payload", type=Path, required=True)
    item.add_argument("--pre-receipt-payload-sha256", required=True)
    item.add_argument("--receipt-request", type=Path, required=True)
    item.add_argument("--receipt-request-sha256", required=True)
    item.add_argument("--receipt", type=Path, required=True)
    item.add_argument("--receipt-sha256", required=True)
    item.add_argument("--output", type=Path, required=True)
    item.set_defaults(func=command_finalize)
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    try:
        args = build_parser().parse_args(argv)
        return int(args.func(args))
    except (FormalFreezeIntakeError, toolchain.ContractError) as exc:
        _emit({"status": "NO_GO", "formal": False, "error_code": getattr(exc, "code", "CONTRACT_ERROR"), "errors": [str(exc)]})
        return 2


if __name__ == "__main__":
    sys.exit(main())
