#!/usr/bin/env python3
"""Fail-closed v3_v4 identity adapter over the frozen strong semantic gate."""

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

import r8_liquid_s5a1_handoff_gate_v4 as base


sys.dont_write_bytecode = True
v1 = base.v1
v3 = base.v3
HandoffError = base.HandoffError
ROOT = MODULE_DIR.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v4.json"
SCHEMA_PATH = ROOT / "schema/target_host_s5a1_handoff_package_v4.json"
BASE_POLICY_PATH = base.BASE_POLICY_PATH
BASE_POLICY_SHA256 = base.BASE_POLICY_SHA256
BASE_SCHEMA_PATH = base.BASE_SCHEMA_PATH
BASE_SCHEMA_SHA256 = base.BASE_SCHEMA_SHA256
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v4"
PARTIAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial"
FINAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}"
POLICY_SCHEMA_VERSION = "smpcc-r8-liquid-s5a1-policy-delta-v4"
POLICY_DOCUMENT_TYPE = "SMPCC_R8_LIQUID_S5A1_PRIMARY_HANDOFF_POLICY_DELTA_V4"
SCHEMA_REVISION_KIND = "DETERMINISTIC_CLOSED_SCHEMA_DELTA_V4"


def _object(raw: bytes, label: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise HandoffError(f"{label} is not an object")
    return value


def _read_exact(path: Path, expected_sha256: str) -> bytes:
    raw, _identity = v1.read_regular(path, maximum=v1.MAX_JSON_BYTES)
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise HandoffError(f"base identity drifted: {path}")
    return raw


def expand_policy(raw: bytes | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    if raw is None:
        raw, identity = v1.read_regular(POLICY_PATH, maximum=v1.MAX_JSON_BYTES)
    else:
        identity = {"path": str(POLICY_PATH), "sha256": hashlib.sha256(raw).hexdigest(),
                    "size_bytes": len(raw)}
    delta = _object(raw, "v3_v4 policy delta")
    expected_base = {"path": str(BASE_POLICY_PATH), "sha256": BASE_POLICY_SHA256}
    expected_overrides = {
        "policy_id": "liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v4",
        "transfer_id": TRANSFER_ID,
        "planned_partial_root": PARTIAL_ROOT,
        "planned_final_root": FINAL_ROOT,
    }
    if set(delta) != {"schema_version", "document_type", "base", "overrides"}:
        raise HandoffError("v3_v4 policy delta is not closed")
    if delta["schema_version"] != POLICY_SCHEMA_VERSION:
        raise HandoffError("v3_v4 policy schema_version differs")
    if delta["document_type"] != POLICY_DOCUMENT_TYPE:
        raise HandoffError("v3_v4 policy document_type differs")
    if delta["base"] != expected_base:
        raise HandoffError("v3_v4 base policy identity differs")
    if delta["overrides"] != expected_overrides:
        raise HandoffError("v3_v4 policy overrides differ")
    effective = copy.deepcopy(_object(_read_exact(BASE_POLICY_PATH, BASE_POLICY_SHA256),
                                      "base package policy"))
    effective["policy_id"] = expected_overrides["policy_id"]
    effective["transfer_id"] = TRANSFER_ID
    effective["package"]["planned_partial_root"] = PARTIAL_ROOT
    effective["package"]["planned_final_root"] = FINAL_ROOT
    return effective, identity


def expand_schema(raw: bytes | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    if raw is None:
        raw, identity = v1.read_regular(SCHEMA_PATH, maximum=v1.MAX_JSON_BYTES)
    else:
        identity = {"path": str(SCHEMA_PATH), "sha256": hashlib.sha256(raw).hexdigest(),
                    "size_bytes": len(raw)}
    delta = _object(raw, "v4 package schema delta")
    expected_base = {"path": str(BASE_SCHEMA_PATH), "sha256": BASE_SCHEMA_SHA256}
    expected_overrides = {
        "schema_id": "https://scout.local/schema/target_host_s5a1_handoff_package_v4.json",
        "transfer_id": TRANSFER_ID,
    }
    if set(delta) != {"revision_kind", "base", "overrides"}:
        raise HandoffError("v4 package schema delta is not closed")
    if delta["revision_kind"] != SCHEMA_REVISION_KIND:
        raise HandoffError("v4 package schema revision_kind differs")
    if delta["base"] != expected_base:
        raise HandoffError("v4 base schema identity differs")
    if delta["overrides"] != expected_overrides:
        raise HandoffError("v4 package schema overrides differ")
    effective = copy.deepcopy(_object(_read_exact(BASE_SCHEMA_PATH, BASE_SCHEMA_SHA256),
                                      "base package schema"))
    effective["$id"] = expected_overrides["schema_id"]
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
    return base.read_json(candidate)


def install() -> None:
    base.POLICY_PATH = POLICY_PATH
    base.SCHEMA_PATH = SCHEMA_PATH
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.expand_policy = expand_policy
    base.expand_schema = expand_schema
    base.read_json = read_json
    base.install()


def validate_package_root(root: str | Path, **kwargs: Any):
    install()
    return base.validate_package_root(root, **kwargs)


def validate_package_bytes(package: Mapping[str, bytes], schema=None, **kwargs: Any):
    install()
    effective, _identity = expand_schema()
    return base.validate_package_bytes(package, effective if schema is None else schema, **kwargs)


def self_check() -> dict[str, Any]:
    policy, policy_identity = expand_policy()
    schema, schema_identity = expand_schema()
    if policy["transfer_id"] != TRANSFER_ID:
        raise HandoffError("effective v3_v4 policy identity differs")
    if schema["properties"]["transfer_id"] != {"const": TRANSFER_ID}:
        raise HandoffError("effective v4 schema identity differs")
    return {
        "status": "PASS_S5A1_HANDOFF_GATE_V6_V3_V4_ADAPTER_STATIC_CONTRACT",
        "transfer_id": TRANSFER_ID,
        "policy_sha256": policy_identity["sha256"],
        "schema_sha256": schema_identity["sha256"],
        "real_bag_read": False,
        "files_written": False,
    }


def __getattr__(name: str):
    if hasattr(base, name):
        return getattr(base, name)
    return getattr(v1, name)


if __name__ == "__main__":
    print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
