"""Identity document for the non-generating Stage 3A layout scaffold.

This is intentionally not the final codegen manifest described by the main
plan. It has no ``GENERATED`` state, accepts no artifact digest, and records
the Stage 0 gate that still prohibits Stage 3 production code generation.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Any

from .contract_source import (
    STAGE3_PROHIBITED_STATUS,
    ContractSourceError,
    contract_reference_from_dict,
    load_stage0_contract_reference,
)
from .layout import (
    LayoutError,
    MainlineLayoutScaffold,
    layout_from_dict,
)

MANIFEST_SCHEMA_VERSION = "spmpc_mainline_layout_scaffold_manifest_v1"
MANIFEST_STATUS = "LAYOUT_ONLY_STAGE3A"
ARTIFACT_GENERATION_GATE: Mapping[str, Any] = MappingProxyType(
    {
        "allowed": False,
        "status": STAGE3_PROHIBITED_STATUS,
        "required_next_authority": ("STAGE1_FROZEN_CONTRACT_SHA256_AND_DATASET_GATE"),
    }
)
_SHA256_LENGTH = 64


class ManifestError(ValueError):
    """Raised when a scaffold identity is malformed or overclaims an artifact."""


def _canonical_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if type(key) is not str:
                raise ManifestError("canonical mapping keys must be strings")
            if key in result:
                raise ManifestError(f"duplicate canonical key: {key}")
            result[key] = _canonical_value(item)
        return result
    if isinstance(value, (list, tuple)):
        return [_canonical_value(item) for item in value]
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ManifestError("canonical JSON cannot contain NaN or infinity")
        return 0.0 if value == 0.0 else value
    raise ManifestError(f"unsupported canonical value: {type(value)!r}")


def canonical_json(value: Any) -> str:
    """Return the deterministic JSON representation used for identity hashes."""

    return json.dumps(
        _canonical_value(value),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _sha256(value: Any, label: str) -> str:
    if type(value) is not str or len(value) != _SHA256_LENGTH:
        raise ManifestError(f"{label} must be a 64-character SHA-256")
    if any(character not in "0123456789abcdef" for character in value):
        raise ManifestError(f"{label} must be lowercase hexadecimal")
    return value


def build_scaffold_manifest(
    layout: MainlineLayoutScaffold,
    source_contract_path: Path | str,
) -> dict[str, Any]:
    """Build a layout-only identity that cannot represent an artifact."""

    if not isinstance(layout, MainlineLayoutScaffold):
        raise ManifestError("layout must be a MainlineLayoutScaffold")
    try:
        checked_source = load_stage0_contract_reference(source_contract_path)
        checked_layout = layout_from_dict(layout.to_dict())
    except (ContractSourceError, LayoutError) as exc:
        raise ManifestError(f"cannot build scaffold identity: {exc}") from exc
    identity = {
        "source_contract": checked_source.to_dict(),
        "layout_scaffold": checked_layout.to_dict(),
    }
    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "status": MANIFEST_STATUS,
        "artifact_generation": dict(ARTIFACT_GENERATION_GATE),
        "identity": identity,
        "identity_sha256": sha256_json(identity),
    }
    validate_scaffold_manifest(manifest)
    return manifest


def validate_scaffold_manifest(value: Any) -> None:
    """Fail closed on drift, runtime pollution, or artifact claims."""

    if type(value) is not dict:
        raise ManifestError("scaffold manifest must be a JSON object")
    expected_top = {
        "schema_version",
        "status",
        "artifact_generation",
        "identity",
        "identity_sha256",
    }
    if set(value) != expected_top:
        raise ManifestError("scaffold manifest keys do not match schema")
    if value["schema_version"] != MANIFEST_SCHEMA_VERSION:
        raise ManifestError("unsupported scaffold manifest schema version")
    if value["status"] != MANIFEST_STATUS:
        raise ManifestError("scaffold manifest cannot claim another status")
    gate = value["artifact_generation"]
    if type(gate) is not dict or set(gate) != {
        "allowed",
        "status",
        "required_next_authority",
    }:
        raise ManifestError("artifact generation gate keys do not match schema")
    if gate["allowed"] is not False:
        raise ManifestError("Stage 3A cannot allow artifact generation")
    if gate != ARTIFACT_GENERATION_GATE:
        raise ManifestError("artifact generation gate does not match Stage 0")

    identity = value["identity"]
    if type(identity) is not dict or set(identity) != {
        "source_contract",
        "layout_scaffold",
    }:
        raise ManifestError("scaffold identity keys do not match schema")
    try:
        source = contract_reference_from_dict(identity["source_contract"])
        layout = layout_from_dict(identity["layout_scaffold"])
    except (ContractSourceError, LayoutError) as exc:
        raise ManifestError(f"invalid scaffold identity: {exc}") from exc

    expected_identity = {
        "source_contract": source.to_dict(),
        "layout_scaffold": layout.to_dict(),
    }
    if identity != expected_identity:
        raise ManifestError("scaffold identity is not canonical")
    digest = _sha256(value["identity_sha256"], "identity_sha256")
    if digest != sha256_json(identity):
        raise ManifestError("identity_sha256 does not match scaffold identity")


def scaffold_identity_hash(value: dict[str, Any]) -> str:
    validate_scaffold_manifest(value)
    return value["identity_sha256"]
