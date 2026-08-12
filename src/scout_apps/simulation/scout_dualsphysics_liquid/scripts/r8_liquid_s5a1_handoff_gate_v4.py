#!/usr/bin/env python3
"""Deterministic v3_v2 package-policy/schema adapter for handoff gate v3.

The adapter changes only transfer identity and exact output roots.  It never
opens a ROS bag and delegates all structural and strong semantic validation to
the frozen v1/v3 implementation after expanding the two closed deltas.
"""

from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_handoff_gate_v3 as v3


sys.dont_write_bytecode = True
v1 = v3.v1
HandoffError = v3.HandoffError
ROOT = MODULE_DIR.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v2.json"
SCHEMA_PATH = ROOT / "schema/target_host_s5a1_handoff_package_v2.json"
BASE_POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v1.json"
BASE_SCHEMA_PATH = ROOT / "schema/target_host_s5a1_handoff_package_v1.json"
BASE_POLICY_SHA256 = "1ba40330581c3736442dcd6191d19f798cef5d658d092af6026de924e4b93c2c"
BASE_SCHEMA_SHA256 = "4e1c10d7bd721e072def81dfd491c1d6d72e2aa45e4b2e8e773a1f69dc471496"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v2"
PARTIAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial"
FINAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}"
_ORIGINAL_READ_JSON = v1.read_json
_ORIGINAL_READ_PARENT = v3._read_parent


def _json(raw: bytes, label: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise HandoffError(f"{label} is not an object")
    return value


def _read_exact(path: Path, expected: str) -> tuple[bytes, dict[str, Any]]:
    raw, identity = v1.read_regular(path, maximum=v1.MAX_JSON_BYTES)
    if hashlib.sha256(raw).hexdigest() != expected:
        raise HandoffError(f"base identity drifted: {path}")
    return raw, identity


def expand_policy(raw: bytes | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    if raw is None:
        raw, identity = v1.read_regular(POLICY_PATH, maximum=v1.MAX_JSON_BYTES)
    else:
        identity = {"path": str(POLICY_PATH), "sha256": hashlib.sha256(raw).hexdigest(),
                    "size_bytes": len(raw)}
    delta = _json(raw, "v3_v2 policy delta")
    if set(delta) != {"schema_version", "document_type", "base", "overrides"}:
        raise HandoffError("v3_v2 policy delta is not closed")
    if delta["base"] != {"path": str(BASE_POLICY_PATH), "sha256": BASE_POLICY_SHA256}:
        raise HandoffError("v3_v2 base policy identity differs")
    expected_overrides = {
        "policy_id": "liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v2",
        "transfer_id": TRANSFER_ID, "planned_partial_root": PARTIAL_ROOT,
        "planned_final_root": FINAL_ROOT,
    }
    if delta["overrides"] != expected_overrides:
        raise HandoffError("v3_v2 policy override differs")
    base_raw, _ = _read_exact(BASE_POLICY_PATH, BASE_POLICY_SHA256)
    effective = copy.deepcopy(_json(base_raw, "base package policy"))
    effective["policy_id"] = expected_overrides["policy_id"]
    effective["transfer_id"] = TRANSFER_ID
    effective["package"]["planned_partial_root"] = PARTIAL_ROOT
    effective["package"]["planned_final_root"] = FINAL_ROOT
    return effective, identity


def expand_schema() -> tuple[dict[str, Any], dict[str, Any]]:
    raw, identity = v1.read_regular(SCHEMA_PATH, maximum=v1.MAX_JSON_BYTES)
    delta = _json(raw, "v2 package schema delta")
    if delta.get("revision_kind") != "DETERMINISTIC_CLOSED_SCHEMA_DELTA_V2":
        raise HandoffError("v2 schema delta kind differs")
    if delta.get("base") != {"path": str(BASE_SCHEMA_PATH), "sha256": BASE_SCHEMA_SHA256}:
        raise HandoffError("v2 base schema identity differs")
    expected = {"schema_id": "https://scout.local/schema/target_host_s5a1_handoff_package_v2.json",
                "transfer_id": TRANSFER_ID}
    if delta.get("overrides") != expected:
        raise HandoffError("v2 schema override differs")
    base_raw, _ = _read_exact(BASE_SCHEMA_PATH, BASE_SCHEMA_SHA256)
    effective = copy.deepcopy(_json(base_raw, "base package schema"))
    effective["$id"] = expected["schema_id"]
    effective["properties"]["transfer_id"] = {"const": TRANSFER_ID}
    Draft202012Validator.check_schema(effective)
    v1.assert_schema_deep_closed(effective)
    return effective, identity


def read_json(path: str | Path) -> tuple[dict[str, Any], dict[str, Any]]:
    candidate = Path(path)
    if candidate == POLICY_PATH:
        return expand_policy()
    if candidate == SCHEMA_PATH:
        return expand_schema()
    return _ORIGINAL_READ_JSON(candidate)


def _read_parent(path: str, supplied: Mapping[str, bytes], *, capture: bool, maximum: int):
    raw, digest, identity = _ORIGINAL_READ_PARENT(
        path, supplied, capture=capture, maximum=maximum
    )
    if Path(path) != POLICY_PATH:
        return raw, digest, identity
    if raw is None:
        raise HandoffError("v3_v2 policy bytes were not captured")
    effective, _ = expand_policy(raw)
    return v1.canonical_json(effective), digest, identity


def install() -> None:
    v1.POLICY_PATH = POLICY_PATH
    v1.SCHEMA_PATH = SCHEMA_PATH
    v1.read_json = read_json
    v3._read_parent = _read_parent


def validate_package_bytes(package: Mapping[str, bytes], schema=None, **kwargs):
    install()
    effective_schema, _ = expand_schema()
    return v3.validate_package_bytes(package, effective_schema, **kwargs)


def validate_package_root(root: str | Path, **kwargs):
    install()
    return v3.validate_package_root(root, trusted_policy_path=POLICY_PATH, **kwargs)


def self_check() -> dict[str, Any]:
    policy, policy_identity = expand_policy()
    schema, schema_identity = expand_schema()
    if policy["transfer_id"] != TRANSFER_ID or policy["package"]["planned_partial_root"] != PARTIAL_ROOT:
        raise HandoffError("effective v3_v2 package policy differs")
    if schema["properties"]["transfer_id"] != {"const": TRANSFER_ID}:
        raise HandoffError("effective v3_v2 package schema differs")
    return {
        "status": "PASS_S5A1_HANDOFF_GATE_V4_V3_V2_ADAPTER_STATIC_CONTRACT",
        "transfer_id": TRANSFER_ID,
        "policy_delta_sha256": policy_identity["sha256"],
        "schema_delta_sha256": schema_identity["sha256"],
        "real_bag_read": False, "optional_bag_read": False,
        "files_written": False, "bwrap_run": False, "solver_executed": False,
        "gpu_exposed": False, "sudo_used": False, "network_used": False,
    }


def __getattr__(name: str):
    if hasattr(v3, name):
        return getattr(v3, name)
    return getattr(v1, name)


if __name__ == "__main__":
    print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
