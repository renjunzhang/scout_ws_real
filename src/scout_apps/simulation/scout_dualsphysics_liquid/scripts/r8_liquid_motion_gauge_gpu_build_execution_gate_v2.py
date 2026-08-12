#!/usr/bin/env python3
"""Static, fail-closed v2 gate for a future fresh motion-Gauge GPU build.

This create-new revision deliberately exposes no real execution path.  It
freezes and validates the identities and mock evidence required by the future
source-copy, six-file patch, wrapper, one-Make and static-audit phases.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v2.json"
POLICY_SCHEMA = ROOT / "schema/target_host_motion_gauge_gpu_build_execution_policy_v2.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_motion_gauge_gpu_build_execution_receipt_v2.json"
REAL_PHASES = ("source-copy", "patch", "wrapper", "build", "static-audit")


class GateError(ValueError):
    pass


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(json.dumps(value, ensure_ascii=False, allow_nan=False,
                                   sort_keys=True, separators=(",", ":")).encode())


def assert_deep_closed(node: object, where: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise GateError(f"open schema object: {where}")
        for key, value in node.items():
            assert_deep_closed(value, f"{where}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{where}/{index}")


def safe_repo_path(relative: str) -> Path:
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise GateError(f"unsafe repository path: {relative}")
    resolved = (ROOT / path).resolve(strict=True)
    if ROOT not in resolved.parents:
        raise GateError(f"repository path escapes root: {relative}")
    return resolved


def file_identity(path: Path) -> dict[str, Any]:
    info = os.lstat(path)
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode) or info.st_nlink != 1:
        raise GateError(f"unsafe regular-file identity: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        digest = hashlib.sha256()
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    fields = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns")
    if any(getattr(before, field) != getattr(after, field) for field in fields):
        raise GateError(f"file changed while hashing: {path}")
    return {"mode_octal": format(stat.S_IMODE(after.st_mode), "04o"),
            "size_bytes": after.st_size, "sha256": digest.hexdigest()}


def load_and_validate() -> tuple[dict[str, Any], str]:
    raw = POLICY_PATH.read_bytes()
    policy = json.loads(raw)
    policy_schema = json.loads(POLICY_SCHEMA.read_bytes())
    receipt_schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    for schema in (policy_schema, receipt_schema):
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(policy_schema).validate(policy)
    return policy, sha256_bytes(raw)


def validate_static_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    for group in ("motion_patch_v2", "g1", "g2"):
        for expected in policy["parents"][group]:
            actual = file_identity(safe_repo_path(expected["path"]))
            if actual["sha256"] != expected["sha256"]:
                raise GateError(f"parent hash drift: {expected['path']}")
            if group == "motion_patch_v2" and actual != {key: expected[key] for key in actual}:
                raise GateError(f"motion patch mode/size/hash drift: {expected['path']}")

    profiles = policy["profiles"]
    if [item["role"] for item in profiles] != ["SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT"]:
        raise GateError("profile role/order drift")
    for expected in profiles:
        actual = file_identity(safe_repo_path(expected["path"]))
        if actual != {key: expected[key] for key in actual}:
            raise GateError(f"exact profile identity drift: {expected['role']}")

    patch = policy["patch_transition"]
    names = [item["path"] for item in patch["files"]]
    expected_names = ["JDsGaugeItem.cpp", "JDsGaugeItem.h", "JDsGaugeSystem.cpp",
                      "JDsGaugeSystem.h", "JSph.cpp", "JSph.h"]
    if names != expected_names or patch["changed_count"] + patch["unchanged_count"] != 352:
        raise GateError("exact 6+346 patch transition drift")
    wrapper = policy["wrapper"]
    encoded = wrapper["content_utf8"].encode()
    if len(encoded) != 84 or sha256_bytes(encoded) != wrapper["sha256"]:
        raise GateError("84-byte wrapper drift")
    build = policy["build"]
    required = {"-j1", "CC=/usr/bin/x86_64-linux-gnu-g++-11",
                "NCC=/usr/local/cuda-12.8/bin/nvcc",
                "GENCODE=-gencode=arch=compute_120,code=\"sm_120,compute_120\""}
    if build["make_argv"][0] != "/usr/bin/make" or not required.issubset(build["make_argv"]):
        raise GateError("Make/CUDA/sm_120 argv drift")
    if build["make_limit"] != 1 or build["parallel_jobs"] != 1 or build["wall_timeout_seconds"] != 5400:
        raise GateError("one-Make resource boundary drift")
    g1 = json.loads(safe_repo_path(policy["parents"]["g1"][0]["path"]).read_bytes())
    objects = g1["object_contract"]["object_names"]
    if len(objects) != 131 or len(set(objects)) != 131 or canonical_sha256(objects) != build["object_names_canonical_sha256"]:
        raise GateError("131-object parent inventory drift")
    if (policy["authorization"]["default_authorized"] or
            policy["authorization"]["all_real_phases_authorized"] or
            any(policy["execution_implementation"][key]
            for key in ("real_phase_runner_implemented", "receipt_producer_implemented",
                        "dirfd_anchored_toctou_implemented", "apparmor_lifecycle_implemented",
                        "resource_monitor_implemented"))):
        raise GateError("v2 static freeze must remain unauthorized and non-executing")
    if policy["runtime_smoke"]["included"] or policy["runtime_smoke"]["candidate_execution_authorized"]:
        raise GateError("runtime smoke crossed the separate-contract boundary")
    return {"parents": 14, "profiles": 4, "source_entries": 352, "changed": 6,
            "unchanged": 346, "wrapper_bytes": 84, "objects": 131,
            "static_commands": 557}


def validate_mock_phase_evidence(policy: Mapping[str, Any], phase: str,
                                 evidence: Mapping[str, Any]) -> None:
    if phase not in REAL_PHASES or not isinstance(evidence, Mapping):
        raise GateError("unknown mock phase/evidence")
    required = {"authorized", "create_new", "o_excl", "o_nofollow", "fd_rechecked",
                "sealed_source_unchanged", "profile_zero_residue", "candidate_executed",
                "gpu_exposed", "network_used", "make_count"}
    if set(evidence) != required:
        raise GateError("mock phase evidence is not closed")
    if evidence["authorized"] is not True or any(evidence[key] is not True for key in
            ("create_new", "o_excl", "o_nofollow", "fd_rechecked",
             "sealed_source_unchanged", "profile_zero_residue")):
        raise GateError("mock evidence lacks authorization/create-new/TOCTOU/lifecycle proof")
    if any(evidence[key] is not False for key in ("candidate_executed", "gpu_exposed", "network_used")):
        raise GateError("mock evidence crosses candidate/GPU/network boundary")
    expected_make = 1 if phase == "build" else 0
    if evidence["make_count"] != expected_make:
        raise GateError("mock one-Make count differs")


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_and_validate()
    counts = validate_static_contract(policy)
    root_absent = not os.path.lexists(policy["campaign"]["attempt_root"])
    audit_parent = Path(policy["campaign"]["audit_prefix"]).parent
    receipt_absent = not any(audit_parent.glob(Path(policy["campaign"]["audit_prefix"]).name + "*"))
    if not root_absent or not receipt_absent:
        raise GateError("frozen campaign root or receipt prefix is no longer fresh")
    return {"status":"PASS_STATIC_EXECUTION_V2_NOT_AUTHORIZED_NOT_EXECUTABLE",
            "policy_sha256":policy_sha, "counts":counts, "root_absent":root_absent,
            "receipt_prefix_absent":receipt_absent, "files_written":False,
            "receipt_producer_implemented":False, "apparmor_lifecycle_implemented":False,
            "real_phase_runner_implemented":False, "candidate_executed":False,
            "gpu_exposed":False, "runtime_smoke_included":False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", *REAL_PHASES))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--authorization-token-file")
    args = parser.parse_args(argv)
    try:
        if args.command != "self-check":
            raise GateError("NOT_IMPLEMENTED_NOT_AUTHORIZED: create-new v3 runner required")
        if args.execute or args.authorization_token_file:
            raise GateError("self-check rejects execution authorization inputs")
        report = self_check()
    except Exception as exc:
        print(json.dumps({"status":"FAIL_MOTION_GAUGE_GPU_BUILD_EXECUTION_V2",
                          "error":str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
