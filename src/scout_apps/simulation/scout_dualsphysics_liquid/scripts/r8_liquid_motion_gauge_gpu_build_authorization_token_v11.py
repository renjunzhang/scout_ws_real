#!/usr/bin/env python3
"""Create one exact, user-authorized Motion-Gauge GPU v11 token.

The default ``self-check`` is read-only.  ``materialize`` is deliberately not
an authorization decision: it requires an explicit human authorization
reference and the SHA-256 of that exact UTF-8 reference, then binds every
execution-layer and AppArmor byte identity into one O_EXCL token.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT.parents[3]
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v11.json"
GATE_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v11.py"
SUPERVISOR_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v11.py"
TOKEN_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_authorization_token_v11.json"
SUPERVISOR_RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_supervisor_receipt_v11.json"
CAMPAIGN_ID = "motion_gauge_gpu_build_sm120_20260812T073037Z_v11"
TOKEN_PATH = Path("/home/zrj/scout_liquid_lab/audits") / f"{CAMPAIGN_ID}_a.authorization.json"
IDENTITY_KEYS = ("path", "mode_octal", "size_bytes", "sha256")


class TokenError(RuntimeError):
    """An authorization input, frozen byte identity, or fresh-state check failed."""


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def resolve_pinned(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    if not path.parts or ".." in path.parts:
        raise TokenError(f"unsafe relative pinned path: {path_text}")
    return WORKSPACE / path


def file_identity(path: Path, *, expected_mode: str | None = None,
                  expected_size: int | None = None,
                  expected_sha256: str | None = None) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise TokenError(f"unsafe identity input: {path}")
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise TokenError(f"short identity read: {path}")
            digest.update(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise TokenError(f"identity input grew while read: {path}")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    before_tuple = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    after_tuple = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if before_tuple != after_tuple:
        raise TokenError(f"identity input drifted while read: {path}")
    result = {
        "path": str(path.resolve()),
        "mode_octal": format(stat.S_IMODE(after.st_mode), "04o"),
        "size_bytes": after.st_size,
        "sha256": digest.hexdigest(),
    }
    expected = (expected_mode, expected_size, expected_sha256)
    actual = (result["mode_octal"], result["size_bytes"], result["sha256"])
    if any(value is not None for value in expected) and any(
        want is not None and got != want for got, want in zip(actual, expected)
    ):
        raise TokenError(f"pinned identity drift: {path}")
    return result


def read_json(path: Path, *, limit: int = 4 * 1024 * 1024) -> dict[str, Any]:
    identity = file_identity(path)
    if identity["size_bytes"] > limit:
        raise TokenError(f"JSON input exceeds bound: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TokenError(f"JSON input is not an object: {path}")
    return value


def write_all(descriptor: int, raw: bytes) -> None:
    offset = 0
    while offset < len(raw):
        count = os.write(descriptor, raw[offset:])
        if count <= 0:
            raise TokenError("short create-new token write")
        offset += count


def write_new(path: Path, raw: bytes) -> dict[str, Any]:
    if os.path.lexists(path):
        raise TokenError(f"authorization token already exists: {path}")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    try:
        os.fchmod(descriptor, 0o600)
        write_all(descriptor, raw)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
    return file_identity(path, expected_mode="0600", expected_size=len(raw),
                         expected_sha256=sha256_bytes(raw))


def frozen_inputs() -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    policy = read_json(POLICY_PATH)
    if policy.get("schema_version") != "smpcc-r8-liquid-motion-gauge-gpu-build-execution-policy-v11":
        raise TokenError("v11 policy identity marker drift")
    if policy.get("campaign", {}).get("campaign_id") != CAMPAIGN_ID:
        raise TokenError("v11 campaign identity drift")
    authorization = policy.get("authorization", {})
    if authorization.get("default_authorized") is not False or authorization.get(
        "exact_new_user_authorization_required"
    ) is not True:
        raise TokenError("policy no longer requires an exact new user authorization")
    if resolve_pinned(authorization.get("token_schema_path", "")) != TOKEN_SCHEMA_PATH:
        raise TokenError("policy token schema path drift")
    file_identity(TOKEN_SCHEMA_PATH, expected_sha256=authorization["token_schema_sha256"])
    if resolve_pinned(authorization.get("token_producer_path", "")) != Path(__file__).resolve():
        raise TokenError("policy token producer path drift")
    if resolve_pinned(authorization.get("supervisor_path", "")) != SUPERVISOR_PATH:
        raise TokenError("policy supervisor path drift")
    if resolve_pinned(authorization.get("supervisor_receipt_schema_path", "")) != SUPERVISOR_RECEIPT_SCHEMA_PATH:
        raise TokenError("policy supervisor receipt schema path drift")
    file_identity(
        SUPERVISOR_RECEIPT_SCHEMA_PATH,
        expected_sha256=authorization["supervisor_receipt_schema_sha256"],
    )
    profiles = policy.get("profiles")
    if not isinstance(profiles, list) or [item.get("role") for item in profiles] != [
        "SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT"
    ]:
        raise TokenError("exact four-profile order drift")
    for item in profiles:
        observed = file_identity(
            resolve_pinned(item["path"]), expected_mode=item["mode_octal"],
            expected_size=item["size_bytes"], expected_sha256=item["sha256"],
        )
        expected_path = str(resolve_pinned(item["path"]).resolve())
        if observed["path"] != expected_path:
            raise TokenError("profile resolved path drift")
    schema = read_json(TOKEN_SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    return policy, schema, profiles


def static_gate_report() -> dict[str, Any]:
    spec = __import__("importlib.util", fromlist=["spec_from_file_location"])
    module_spec = spec.spec_from_file_location("r8_motion_v11_gate_for_token", GATE_PATH)
    if module_spec is None or module_spec.loader is None:
        raise TokenError("cannot load exact v11 gate for static admission")
    module = spec.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    report = module.self_check()
    if report.get("status") != "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY":
        raise TokenError("v11 gate static admission is not PASS")
    if any(report.get(key) for key in (
        "files_written", "system_actions_performed", "make_run", "compiler_run",
        "candidate_executed", "profile_loaded",
    )):
        raise TokenError("v11 gate static admission safety claims drift")
    return report


def build_token(authorization_reference: str,
                authorization_reference_sha256: str) -> dict[str, Any]:
    if not 16 <= len(authorization_reference) <= 4096:
        raise TokenError("authorization reference length is outside the closed schema")
    observed_reference_sha = sha256_bytes(authorization_reference.encode("utf-8"))
    if observed_reference_sha != authorization_reference_sha256:
        raise TokenError("authorization reference SHA-256 mismatch")
    policy, schema, profiles = frozen_inputs()
    static_gate_report()
    identities = {
        "policy": file_identity(POLICY_PATH),
        "gate": file_identity(GATE_PATH),
        "supervisor": file_identity(SUPERVISOR_PATH),
        "supervisor_receipt_schema": file_identity(SUPERVISOR_RECEIPT_SCHEMA_PATH),
        "token_producer": file_identity(Path(__file__).resolve()),
    }
    value = {
        "schema_version": "smpcc-r8-liquid-motion-gauge-gpu-build-authorization-token-v11",
        "campaign_id": CAMPAIGN_ID,
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "authorization_reference": authorization_reference,
        "authorization_reference_sha256": authorization_reference_sha256,
        **identities,
        "profiles": [
            {key: item[key] for key in ("role", "name", "path", "mode_octal", "size_bytes", "sha256")}
            for item in profiles
        ],
        "user_authorized": True,
    }
    Draft202012Validator(schema).validate(value)
    if value["policy"]["sha256"] != file_identity(POLICY_PATH)["sha256"]:
        raise TokenError("policy drifted while token was assembled")
    if policy["campaign"]["authorization_token_path"] != str(TOKEN_PATH):
        raise TokenError("policy authorization token path drift")
    return value


def self_check() -> dict[str, Any]:
    policy, schema, profiles = frozen_inputs()
    gate_report = static_gate_report()
    identities = {
        "policy": file_identity(POLICY_PATH), "gate": file_identity(GATE_PATH),
        "supervisor": file_identity(SUPERVISOR_PATH),
        "supervisor_receipt_schema": file_identity(SUPERVISOR_RECEIPT_SCHEMA_PATH),
        "token_producer": file_identity(Path(__file__).resolve()),
    }
    if os.path.lexists(TOKEN_PATH):
        raise TokenError("fresh authorization token path is already consumed")
    return {
        "status": "PASS_MOTION_GAUGE_GPU_BUILD_TOKEN_PRODUCER_V11_STATIC_ONLY",
        "campaign_id": CAMPAIGN_ID,
        "policy_sha256": identities["policy"]["sha256"],
        "gate_sha256": identities["gate"]["sha256"],
        "supervisor_sha256": identities["supervisor"]["sha256"],
        "token_producer_sha256": identities["token_producer"]["sha256"],
        "token_schema_sha256": file_identity(TOKEN_SCHEMA_PATH)["sha256"],
        "profile_sha256": [item["sha256"] for item in profiles],
        "schema_closed": schema.get("additionalProperties") is False,
        "gate_static_status": gate_report["status"],
        "token_materialized": False,
        "user_authorized": False,
        "system_actions_performed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "materialize"), nargs="?", default="self-check")
    parser.add_argument("--authorization-reference")
    parser.add_argument("--authorization-reference-sha256")
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            if args.authorization_reference or args.authorization_reference_sha256:
                raise TokenError("self-check rejects authorization material")
            report = self_check()
        else:
            if args.authorization_reference is None or args.authorization_reference_sha256 is None:
                raise TokenError("materialize requires explicit authorization reference and SHA-256")
            value = build_token(args.authorization_reference, args.authorization_reference_sha256)
            identity = write_new(TOKEN_PATH, canonical_json(value))
            report = {
                "status": "PASS_EXACT_USER_AUTHORIZATION_TOKEN_V11_CREATE_NEW",
                "token": identity,
                "authorization_reference_sha256": args.authorization_reference_sha256,
                "user_authorized": True,
                "system_actions_performed": False,
            }
        print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_BUILD_TOKEN_V11", "error": str(exc)},
                         ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
