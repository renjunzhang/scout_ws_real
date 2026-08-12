#!/usr/bin/env python3
"""Create-new exact authorization token for one S5B0 primary replay v9.

The public CLI is self-check only.  ``materialize`` is deliberately an import
API for a separately authorized operator path; static checks never read the
bag, candidate, build receipt, GPU, journal, or any replay root.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
GATE_PATH = SCRIPTS / "r8_liquid_s5b0_replay_execution_gate_v9.py"
RUNTIME_PATH = SCRIPTS / "r8_liquid_s5b0_replay_runtime_supervisor_v9.py"
LIFECYCLE_PATH = SCRIPTS / "r8_liquid_s5b0_replay_lifecycle_supervisor_v9.py"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v9.json"
PROFILE_PATH = ROOT / "config/apparmor_drafts/r8-liquid-s5b0-primary-bsmooth-b01-20260812t110635z-v9.profile"
TOKEN_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_authorization_token_v9.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_execution_policy_v9.json"
RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_execution_receipt_v9.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_result_package_v9.json"


class TokenV9Error(RuntimeError):
    """An exact identity, authorization, or create-new invariant failed."""


def _load_gate():
    spec = importlib.util.spec_from_file_location("s5b0_gate_v9_for_token", GATE_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _write_all(descriptor: int, raw: bytes) -> None:
    view = memoryview(raw)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            raise TokenV9Error("short token write")
        view = view[count:]


def write_new(path: Path, raw: bytes, mode: int = 0o600) -> dict[str, Any]:
    gate = _load_gate()
    if os.path.lexists(path):
        raise TokenV9Error(f"create-new token target exists: {path}")
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode
    )
    try:
        os.fchmod(descriptor, mode)
        _write_all(descriptor, raw)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        os.fsync(parent)
    finally:
        os.close(parent)
    return gate._file_identity(path, 64 * 1024)


def build_token(authorization_reference: str, *, created_at: str | None = None) -> dict[str, Any]:
    if not isinstance(authorization_reference, str) or not 16 <= len(authorization_reference) <= 4096:
        raise TokenV9Error("exact authorization reference is absent or unbounded")
    gate = _load_gate()
    policy, _policy_sha = gate.read_policy()
    gate.validate_static_allow_built(policy)
    candidate = gate._file_identity(Path(policy["build_dependency"]["candidate_path"]))
    if candidate["mode"] != "0400":
        raise TokenV9Error("candidate mode is not frozen 0400")
    build_receipt = gate._file_identity(
        Path(policy["build_dependency"]["campaign_final_receipt"]), 16 * 1024 * 1024
    )
    receipt_value = json.loads(Path(build_receipt["path"]).read_bytes())
    if receipt_value.get("status") != policy["build_dependency"]["required_status"]:
        raise TokenV9Error("build final/static audit status differs")
    schema_targets = {
        "policy": POLICY_SCHEMA_PATH,
        "authorization_token": TOKEN_SCHEMA_PATH,
        "execution_receipt": RECEIPT_SCHEMA_PATH,
        "result_package": RESULT_SCHEMA_PATH,
    }
    value = {
        "schema_version": "smpcc-r8-liquid-s5b0-replay-authorization-token-v9",
        "decision": "USER_AUTHORIZED_EXACT_SINGLE_PRIMARY_REPLAY_V9",
        "created_at": created_at or utc_now(),
        "authorization_reference": authorization_reference,
        "authorization_reference_sha256": sha256_bytes(authorization_reference.encode("utf-8")),
        "attempt_id": policy["selection"]["attempt_id"],
        "planned_denominator": 1,
        "replay_id": policy["replay"]["replay_id"],
        "policy": gate._file_identity(POLICY_PATH),
        "gate": gate._file_identity(GATE_PATH),
        "runtime_supervisor": gate._file_identity(RUNTIME_PATH),
        "lifecycle_supervisor": gate._file_identity(LIFECYCLE_PATH),
        "token_producer": gate._file_identity(Path(__file__).resolve()),
        "profile": gate._file_identity(PROFILE_PATH),
        "candidate": candidate,
        "build_final_receipt": build_receipt,
        "schemas": {key: gate._file_identity(path) for key, path in schema_targets.items()},
        "static_audit_status": policy["build_dependency"]["required_status"],
        "roots": {key: policy["replay"][key] for key in ("stage_root", "partial_root", "final_root", "audit_root")},
        "devices": list(policy["devices"]),
        "authorization": {
            "user_authorized": True, "candidate_execution_count": 1,
            "profile_lifecycle_count": 1,
            "sudo_scope": "EXACT_PROFILE_LOAD_STATUS_UNLOAD_AND_SETUID_DROP_ONLY",
        },
        "claims": {
            "optional_bag_read": False, "optional_execution": False,
            "c2_execution": False, "second_execution": False,
            "network_use": False, "cross_method_ranking": False,
        },
    }
    Draft202012Validator(json.loads(TOKEN_SCHEMA_PATH.read_bytes())).validate(value)
    return value


def materialize(authorization_reference: str) -> dict[str, Any]:
    gate = _load_gate()
    policy, _ = gate.read_policy()
    path = Path(policy["replay"]["token_path"])
    value = build_token(authorization_reference)
    identity = write_new(path, gate.canonical_json(value), 0o600)
    gate.validate_token(path)
    return identity


def self_check() -> dict[str, Any]:
    gate = _load_gate()
    policy, policy_sha = gate.read_policy()
    gate.validate_static(policy)
    schema = json.loads(TOKEN_SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.assert_deep_closed(schema)
    gate._assert_required_equals_properties(schema)
    return {
        "status": "PASS_S5B0_REPLAY_AUTHORIZATION_TOKEN_V9_STATIC_ONLY",
        "policy_sha256": policy_sha,
        "bound_components": ["policy", "gate", "runtime_supervisor", "lifecycle_supervisor", "token_producer", "profile", "candidate", "build_final_receipt"],
        "bound_schemas": ["policy", "authorization_token", "execution_receipt", "result_package"],
        "bound_roots": ["stage_root", "partial_root", "final_root", "audit_root"],
        "token_materialized": False, "external_source_read": False,
        "candidate_executed": False, "gpu_exposed": False, "sudo_used": False,
        "profile_loaded": False, "network_used": False, "optional_bag_read": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check")
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_REPLAY_AUTHORIZATION_TOKEN_V9", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
