#!/usr/bin/env python3
"""Fail-closed fresh v3_v10 identity adapter over the frozen semantic gate."""

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
import r8_liquid_s5a1_handoff_gate_v11 as prior

sys.dont_write_bytecode = True
base, v1, HandoffError = prior.base, prior.v1, prior.HandoffError
ROOT = MODULE_DIR.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v10.json"
SCHEMA_PATH = ROOT / "schema/target_host_s5a1_handoff_package_v10.json"
BASE_POLICY_PATH, BASE_POLICY_SHA256 = prior.BASE_POLICY_PATH, prior.BASE_POLICY_SHA256
BASE_SCHEMA_PATH, BASE_SCHEMA_SHA256 = prior.BASE_SCHEMA_PATH, prior.BASE_SCHEMA_SHA256
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v10"
PARTIAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial"
FINAL_ROOT = f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}"


def _object(raw: bytes, label: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise HandoffError(f"{label} is not an object")
    return value


def _read_exact(path: Path, digest: str) -> bytes:
    raw, _identity = v1.read_regular(path, maximum=v1.MAX_JSON_BYTES)
    if hashlib.sha256(raw).hexdigest() != digest:
        raise HandoffError(f"base identity drifted: {path}")
    return raw


def expand_policy(raw: bytes | None = None):
    if raw is None:
        raw, identity = v1.read_regular(POLICY_PATH, maximum=v1.MAX_JSON_BYTES)
    else:
        identity = {
            "path": str(POLICY_PATH),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "size_bytes": len(raw),
        }
    expected = {
        "schema_version": "smpcc-r8-liquid-s5a1-policy-delta-v10",
        "document_type": "SMPCC_R8_LIQUID_S5A1_PRIMARY_HANDOFF_POLICY_DELTA_V10",
        "base": {"path": str(BASE_POLICY_PATH), "sha256": BASE_POLICY_SHA256},
        "overrides": {
            "policy_id": "liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v10",
            "transfer_id": TRANSFER_ID,
            "planned_partial_root": PARTIAL_ROOT,
            "planned_final_root": FINAL_ROOT,
        },
    }
    if _object(raw, "v3_v10 policy delta") != expected:
        raise HandoffError("v3_v10 policy delta differs or is not closed")
    effective = copy.deepcopy(
        _object(_read_exact(BASE_POLICY_PATH, BASE_POLICY_SHA256), "base package policy")
    )
    effective["policy_id"] = expected["overrides"]["policy_id"]
    effective["transfer_id"] = TRANSFER_ID
    effective["package"]["planned_partial_root"] = PARTIAL_ROOT
    effective["package"]["planned_final_root"] = FINAL_ROOT
    return effective, identity


def expand_schema(raw: bytes | None = None):
    if raw is None:
        raw, identity = v1.read_regular(SCHEMA_PATH, maximum=v1.MAX_JSON_BYTES)
    else:
        identity = {
            "path": str(SCHEMA_PATH),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "size_bytes": len(raw),
        }
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_SCHEMA_DELTA_V10",
        "base": {"path": str(BASE_SCHEMA_PATH), "sha256": BASE_SCHEMA_SHA256},
        "overrides": {
            "schema_id": "https://scout.local/schema/target_host_s5a1_handoff_package_v10.json",
            "transfer_id": TRANSFER_ID,
        },
    }
    if _object(raw, "v10 package schema delta") != expected:
        raise HandoffError("v10 package schema delta differs or is not closed")
    effective = copy.deepcopy(
        _object(_read_exact(BASE_SCHEMA_PATH, BASE_SCHEMA_SHA256), "base package schema")
    )
    effective["$id"] = expected["overrides"]["schema_id"]
    effective["properties"]["transfer_id"] = {"const": TRANSFER_ID}
    Draft202012Validator.check_schema(effective)
    v1.assert_schema_deep_closed(effective)
    return effective, identity


def read_json(path: str | Path):
    candidate = Path(path)
    if candidate == POLICY_PATH:
        return expand_policy()
    if candidate == SCHEMA_PATH:
        return expand_schema()
    return base.read_json(candidate)


def install() -> None:
    base.POLICY_PATH, base.SCHEMA_PATH = POLICY_PATH, SCHEMA_PATH
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = (
        TRANSFER_ID,
        PARTIAL_ROOT,
        FINAL_ROOT,
    )
    base.expand_policy, base.expand_schema, base.read_json = (
        expand_policy,
        expand_schema,
        read_json,
    )
    base.install()


def validate_package_root(root: str | Path, **kwargs: Any):
    install()
    return base.validate_package_root(root, **kwargs)


def validate_package_bytes(
    package: Mapping[str, bytes], schema=None, **kwargs: Any
):
    install()
    effective, _identity = expand_schema()
    return base.validate_package_bytes(
        package, effective if schema is None else schema, **kwargs
    )


def self_check() -> dict[str, Any]:
    policy, policy_identity = expand_policy()
    schema, schema_identity = expand_schema()
    if (
        policy["transfer_id"] != TRANSFER_ID
        or schema["properties"]["transfer_id"] != {"const": TRANSFER_ID}
    ):
        raise HandoffError("effective v3_v10 identity differs")
    return {
        "status": "PASS_S5A1_HANDOFF_GATE_V12_V3_V10_ADAPTER_STATIC_CONTRACT",
        "transfer_id": TRANSFER_ID,
        "policy_sha256": policy_identity["sha256"],
        "schema_sha256": schema_identity["sha256"],
        "real_bag_read": False,
        "files_written": False,
    }


def __getattr__(name: str):
    return getattr(base, name) if hasattr(base, name) else getattr(v1, name)


if __name__ == "__main__":
    print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
