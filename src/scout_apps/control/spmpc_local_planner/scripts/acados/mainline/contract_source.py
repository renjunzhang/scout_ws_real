"""Typed reference to the immutable Stage 0 source contract.

Stage 3A records where its frozen structural formulae came from while also
proving that Stage 1 is still blocked and production code generation remains
prohibited. This module does not interpret or manufacture Stage 1 evidence.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

STAGE0_SCHEMA_VERSION = "spmpc_mainline_stage0_contract_v1"
STAGE0_CONTRACT_ID = "SPMPC-MAINLINE-STAGE0-20260902"
STAGE0_CONTRACT_SHA256 = (
    "2a9b1fdd91a474af5894a23b9cb882a94580204ea76cea1fc95ec13ce80508bd"
)
STAGE0_STATUS = "CONTRACT_FROZEN_NUMERICS_UNFROZEN"
STAGE1_BLOCKED_STATUS = "BLOCKED_PENDING_DEDICATED_IDENTIFICATION_EVIDENCE"
STAGE3_PROHIBITED_STATUS = "PROHIBITED_UNTIL_STAGE1_L_MAX_IS_FROZEN"
LMAX_AUTHORITY = "UNFROZEN_IN_STAGE0"
DATASET_GATE_AUTHORITY = "UNFROZEN_IN_STAGE0"

_REQUIRED_UNFROZEN_FIELDS = {
    "execution_model.L_max_v_sec",
    "execution_model.L_max_omega_sec",
    "stage1.dataset_partition_and_gate_hash",
}
_SHA256_LENGTH = 64


class ContractSourceError(ValueError):
    """Raised when Stage 0 cannot authorize even a synthetic scaffold."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ContractSourceError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_constant(value: str) -> None:
    raise ContractSourceError(f"non-finite JSON number is forbidden: {value}")


def _sha256(value: Any, label: str) -> str:
    if type(value) is not str or len(value) != _SHA256_LENGTH:
        raise ContractSourceError(f"{label} must be a 64-character SHA-256")
    if any(character not in "0123456789abcdef" for character in value):
        raise ContractSourceError(f"{label} must be lowercase hexadecimal")
    return value


@dataclass(frozen=True)
class Stage0ContractReference:
    """Minimal immutable authority carried by a Stage 3A scaffold manifest."""

    schema_version: str
    contract_id: str
    contract_sha256: str
    status: str
    stage1_gate: str
    stage3_gate: str
    lmax_authority: str
    dataset_gate_authority: str

    def to_dict(self) -> dict[str, str]:
        return {
            "schema_version": self.schema_version,
            "contract_id": self.contract_id,
            "contract_sha256": self.contract_sha256,
            "status": self.status,
            "stage1_gate": self.stage1_gate,
            "stage3_gate": self.stage3_gate,
            "lmax_authority": self.lmax_authority,
            "dataset_gate_authority": self.dataset_gate_authority,
        }


def contract_reference_from_dict(value: Any) -> Stage0ContractReference:
    """Strictly reconstruct a serialized Stage 0 reference."""

    if type(value) is not dict:
        raise ContractSourceError("source_contract must be a JSON object")
    expected_keys = {
        "schema_version",
        "contract_id",
        "contract_sha256",
        "status",
        "stage1_gate",
        "stage3_gate",
        "lmax_authority",
        "dataset_gate_authority",
    }
    if set(value) != expected_keys:
        raise ContractSourceError(
            "source_contract keys do not match the Stage 3A reference schema"
        )
    reference = Stage0ContractReference(
        schema_version=value["schema_version"],
        contract_id=value["contract_id"],
        contract_sha256=_sha256(
            value["contract_sha256"], "source_contract.contract_sha256"
        ),
        status=value["status"],
        stage1_gate=value["stage1_gate"],
        stage3_gate=value["stage3_gate"],
        lmax_authority=value["lmax_authority"],
        dataset_gate_authority=value["dataset_gate_authority"],
    )
    expected = {
        "schema_version": STAGE0_SCHEMA_VERSION,
        "contract_id": STAGE0_CONTRACT_ID,
        "contract_sha256": STAGE0_CONTRACT_SHA256,
        "status": STAGE0_STATUS,
        "stage1_gate": STAGE1_BLOCKED_STATUS,
        "stage3_gate": STAGE3_PROHIBITED_STATUS,
        "lmax_authority": LMAX_AUTHORITY,
        "dataset_gate_authority": DATASET_GATE_AUTHORITY,
    }
    actual = reference.to_dict()
    for key, expected_value in expected.items():
        if actual[key] != expected_value:
            raise ContractSourceError(
                f"source_contract.{key} is not the frozen Stage 3A value"
            )
    return reference


def load_stage0_contract_reference(
    path: Path | str,
) -> Stage0ContractReference:
    """Load Stage 0 and prove that Stage 1/3 remain closed.

    The SHA is over the exact source bytes. Full Stage 0 schema validation is
    still owned by ``validate_stage0_contract.py``; this narrow loader checks
    only the fields that authorize or prohibit this scaffold.
    """

    if not isinstance(path, (str, Path)):
        raise ContractSourceError("Stage 0 contract path must be str or Path")
    contract_path = Path(path)
    try:
        payload = contract_path.read_bytes()
        text = payload.decode("utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise ContractSourceError(
            f"cannot read Stage 0 contract {contract_path}: {exc}"
        ) from exc
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except ContractSourceError:
        raise
    except json.JSONDecodeError as exc:
        raise ContractSourceError(
            f"invalid Stage 0 JSON in {contract_path}: {exc}"
        ) from exc
    if type(value) is not dict:
        raise ContractSourceError("Stage 0 contract root must be a JSON object")
    for key, expected in (
        ("schema_version", STAGE0_SCHEMA_VERSION),
        ("contract_id", STAGE0_CONTRACT_ID),
        ("status", STAGE0_STATUS),
    ):
        if value.get(key) != expected:
            raise ContractSourceError(f"Stage 0 {key} does not match the scaffold")

    stage_status = value.get("stage_status")
    if type(stage_status) is not dict:
        raise ContractSourceError("Stage 0 stage_status must be an object")
    if stage_status.get("stage1") != STAGE1_BLOCKED_STATUS:
        raise ContractSourceError("Stage 1 is not the expected blocked gate")
    if stage_status.get("stage3") != STAGE3_PROHIBITED_STATUS:
        raise ContractSourceError("Stage 3 is not explicitly prohibited")

    unfrozen = value.get("unfrozen_parameters")
    if type(unfrozen) is not list or any(type(item) is not str for item in unfrozen):
        raise ContractSourceError("Stage 0 unfrozen_parameters must be a string array")
    if len(unfrozen) != len(set(unfrozen)):
        raise ContractSourceError("Stage 0 unfrozen_parameters contains duplicates")
    missing = sorted(_REQUIRED_UNFROZEN_FIELDS - set(unfrozen))
    if missing:
        raise ContractSourceError(
            "Stage 0 no longer marks required Stage 1 fields unfrozen: "
            + ", ".join(missing)
        )

    contract_sha256 = hashlib.sha256(payload).hexdigest()
    if contract_sha256 != STAGE0_CONTRACT_SHA256:
        raise ContractSourceError(
            "Stage 0 content SHA-256 is not the pinned immutable v1 contract"
        )
    reference = Stage0ContractReference(
        schema_version=STAGE0_SCHEMA_VERSION,
        contract_id=STAGE0_CONTRACT_ID,
        contract_sha256=contract_sha256,
        status=STAGE0_STATUS,
        stage1_gate=STAGE1_BLOCKED_STATUS,
        stage3_gate=STAGE3_PROHIBITED_STATUS,
        lmax_authority=LMAX_AUTHORITY,
        dataset_gate_authority=DATASET_GATE_AUTHORITY,
    )
    return contract_reference_from_dict(reference.to_dict())
