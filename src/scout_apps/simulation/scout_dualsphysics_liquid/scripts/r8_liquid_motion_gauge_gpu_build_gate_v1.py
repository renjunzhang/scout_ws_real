#!/usr/bin/env python3
"""Fail-closed static contract for a future patched sm_120 GPU build campaign."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_policy_v1.json"
POLICY_SCHEMA = ROOT / "schema/target_host_motion_gauge_gpu_build_policy_v1.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_motion_gauge_gpu_build_receipt_v1.json"
PROFILE_GENERATOR = ROOT / "scripts/r8_liquid_motion_gauge_gpu_build_profile_generator_v1.py"


class BuildContractError(ValueError):
    pass


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(json.dumps(value, ensure_ascii=False, allow_nan=False,
                                   sort_keys=True, separators=(",", ":")).encode())


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise BuildContractError(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def load_profile_generator() -> ModuleType:
    spec = importlib.util.spec_from_file_location("motion_gauge_gpu_profile_v1", PROFILE_GENERATOR)
    if spec is None or spec.loader is None:
        raise BuildContractError("cannot import profile generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_and_validate() -> tuple[dict[str, Any], str]:
    raw = POLICY_PATH.read_bytes()
    policy = json.loads(raw)
    policy_schema = json.loads(POLICY_SCHEMA.read_bytes())
    receipt_schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    for schema in (policy_schema, receipt_schema):
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(policy_schema).validate(policy)
    for parent_set in (policy["parents"]["g1"], policy["parents"]["g2"]):
        for identity in parent_set.values():
            path = ROOT / identity["path"]
            if sha256_bytes(path.read_bytes()) != identity["sha256"]:
                raise BuildContractError(f"byte-pinned parent differs: {identity['path']}")
    return policy, sha256_bytes(raw)


def validate_patch_inventories(policy: Mapping[str, Any], before: Mapping[str, str],
                               after: Mapping[str, str]) -> dict[str, Any]:
    """Validate a mocked/observed full-tree transition without touching the tree."""
    sha_pattern = re.compile(r"^[0-9a-f]{64}$")
    if not isinstance(before, Mapping) or not isinstance(after, Mapping):
        raise BuildContractError("inventory is not a mapping")
    if len(before) != 352 or set(before) != set(after):
        raise BuildContractError("full source inventory is not the exact 352-entry set")
    if any(not isinstance(path, str) or not path or path.startswith("/") or ".." in Path(path).parts
           or not isinstance(digest, str) or sha_pattern.fullmatch(digest) is None
           for inventory in (before, after) for path, digest in inventory.items()):
        raise BuildContractError("unsafe inventory entry")
    expected = {item["path"]: item for item in policy["patch_transition"]["files"]}
    changed = {path for path in before if before[path] != after[path]}
    if changed != set(expected):
        raise BuildContractError("patch changed set is not the exact six files")
    for path, contract in expected.items():
        if before.get(path) != contract["before_sha256"] or after.get(path) != contract["after_sha256"]:
            raise BuildContractError(f"patch before/after hash differs: {path}")
    unchanged = set(before) - changed
    if len(unchanged) != 346 or any(before[path] != after[path] for path in unchanged):
        raise BuildContractError("346-file unchanged proof differs")
    return {"source_entries": 352, "changed_count": 6, "unchanged_count": 346,
            "before_inventory_sha256": canonical_sha256(sorted(before.items())),
            "after_inventory_sha256": canonical_sha256(sorted(after.items()))}


def validate_post_wrapper_inventory(policy: Mapping[str, Any], inventory: Mapping[str, str],
                                    wrapper_bytes: bytes, wrapper_mode: int) -> dict[str, Any]:
    if len(inventory) != 353 or "U3GpuBuild.mk" not in inventory:
        raise BuildContractError("post-wrapper inventory is not exactly 353 entries")
    wrapper = policy["wrapper"]
    if len(wrapper_bytes) != 84 or sha256_bytes(wrapper_bytes) != wrapper["sha256"]:
        raise BuildContractError("84-byte wrapper identity differs")
    if wrapper_mode != 0o600 or inventory["U3GpuBuild.mk"] != wrapper["sha256"]:
        raise BuildContractError("wrapper mode/inventory binding differs")
    return {"entry_count": 353, "wrapper_mode_octal": "0600",
            "inventory_sha256": canonical_sha256(sorted(inventory.items()))}


def validate_build_evidence(policy: Mapping[str, Any], *, make_count: int, make_argv: Sequence[str],
                            object_names: Sequence[str], candidate_mode: int,
                            candidate_regular: bool, candidate_nlink: int,
                            candidate_size: int) -> None:
    if make_count != 1 or list(make_argv) != policy["build"]["make_argv"]:
        raise BuildContractError("Make invocation differs or is not exactly one")
    g1 = json.loads((ROOT / policy["parents"]["g1"]["policy"]["path"]).read_bytes())
    expected = g1["object_contract"]["object_names"]
    if list(object_names) != expected or len(set(object_names)) != 131:
        raise BuildContractError("131-object contract differs")
    if candidate_mode != 0o400 or not candidate_regular or candidate_nlink != 1 or candidate_size <= 0:
        raise BuildContractError("candidate is not regular/nlink1/positive/0400")


def validate_static_audit_evidence(policy: Mapping[str, Any], *, command_ids: Sequence[str],
                                   candidate_before: Mapping[str, Any],
                                   candidate_after: Mapping[str, Any],
                                   objects_before: Mapping[str, str],
                                   objects_after: Mapping[str, str]) -> None:
    expected = list(policy["static_audit"]["candidate_command_ids"])
    expected += [f"object_readelf_header_{name}" for name in objects_before]
    if list(command_ids) != expected or len(command_ids) != 142:
        raise BuildContractError("fresh static-audit command catalog differs")
    if dict(candidate_before) != dict(candidate_after) or dict(objects_before) != dict(objects_after):
        raise BuildContractError("static audit mutated candidate or objects")
    if len(objects_before) != 131:
        raise BuildContractError("static audit object inventory differs")


def validate_static_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    campaign = policy["campaign"]
    prefix = f"/home/zrj/scout_liquid_lab/build/{campaign['build_id']}.partial"
    if campaign["attempt_root"] != prefix or campaign["output_root"] != prefix + "/output":
        raise BuildContractError("fresh campaign root derivation differs")
    if campaign["source_root"] != prefix + "/output/buildtree/src/source":
        raise BuildContractError("source root derivation differs")
    if campaign["candidate_path"] != prefix + "/output/artifacts/DualSPHysics5.4_linux64":
        raise BuildContractError("candidate path derivation differs")

    patch = policy["patch_transition"]
    names = [item["path"] for item in patch["files"]]
    expected = ["JDsGaugeItem.cpp", "JDsGaugeItem.h", "JDsGaugeSystem.cpp",
                "JDsGaugeSystem.h", "JSph.cpp", "JSph.h"]
    if names != expected or len(set(names)) != 6:
        raise BuildContractError("patch file set/order differs")
    if patch["changed_count"] + patch["unchanged_count"] != policy["source_copy"]["entry_count"]:
        raise BuildContractError("352-file patch transition arithmetic differs")
    if policy["wrapper"]["content_utf8"].encode().__len__() != 84:
        raise BuildContractError("wrapper is not exactly 84 bytes")
    if sha256_bytes(policy["wrapper"]["content_utf8"].encode()) != policy["wrapper"]["sha256"]:
        raise BuildContractError("wrapper hash differs")

    argv = policy["build"]["make_argv"]
    required = {"-j1", "CC=/usr/bin/x86_64-linux-gnu-g++-11",
                "NCC=/usr/local/cuda-12.8/bin/nvcc",
                "GENCODE=-gencode=arch=compute_120,code=\"sm_120,compute_120\""}
    if argv[0] != "/usr/bin/make" or not required.issubset(argv) or any("g++-13" in token for token in argv):
        raise BuildContractError("exact g++11/CUDA12.8/sm120 Make contract differs")
    if policy["build"]["make_limit"] != 1 or policy["build"]["wall_timeout_seconds"] != 5400:
        raise BuildContractError("one-shot Make resource contract differs")
    audit = policy["static_audit"]
    if audit["candidate_command_count"] + audit["object_command_count"] != audit["total_command_count"]:
        raise BuildContractError("static-audit command arithmetic differs")

    parent = policy["parents"]["motion_patch_v2"]
    auth = policy["authorization"]
    if parent != {"path": "scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py",
                  "sha256": None, "contract_status": "PENDING_FINAL_PARENT_HASH",
                  "execution_parent_ready": False}:
        raise BuildContractError("pending v2 parent must be exact fail-closed placeholder")
    if any(auth.values()) or policy["claims"]["execution_admitted"]:
        raise BuildContractError("repository contract may not authorize execution")
    return {"changed": 6, "unchanged": 346, "source_entries": 352,
            "post_wrapper_entries": 353, "objects": 131, "audit_commands": 142}


def build_not_admitted_receipt(policy: Mapping[str, Any], policy_sha: str) -> dict[str, Any]:
    receipt = {
        "schema_version": "smpcc-r8-liquid-motion-gauge-gpu-build-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_MOTION_GAUGE_GPU_BUILD_RECEIPT_V1",
        "campaign_id": policy["campaign"]["campaign_id"],
        "build_id": policy["campaign"]["build_id"],
        "phase": "SOURCE_COPY", "kind": "FAILURE",
        "status": "NOT_ADMITTED_MOTION_PATCH_V2_PARENT_HASH_REQUIRED",
        "policy_sha256": policy_sha, "parent_patch_v2_sha256": None,
        "inventory": None, "patch": None, "build": None, "static_audit": None,
        "safety": {"create_new_only": True, "sealed_source_unchanged": True,
                   "candidate_executed": False, "gpu_exposed": False,
                   "optional_bag_read": False, "network_used": False,
                   "sudo_used": False, "apparmor_loaded": False},
        "next_allowed_stage": "STOP_PARENT_HASH_REQUIRED",
    }
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator(schema).validate(receipt)
    return receipt


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_and_validate()
    counts = validate_static_contract(policy)
    profile_report = load_profile_generator().self_check()
    receipt = build_not_admitted_receipt(policy, policy_sha)
    return {
        "status": "PASS_MOTION_GAUGE_GPU_BUILD_V1_STATIC_CONTRACT_NOT_ADMITTED",
        "policy_sha256": policy_sha,
        "campaign_id": policy["campaign"]["campaign_id"],
        "counts": counts,
        "profile_status": profile_report["status"],
        "profile_hashes": {key: value["sha256"] for key, value in profile_report["profiles"].items()},
        "receipt_status": receipt["status"],
        "motion_patch_v2_parent_sha256": None,
        "external_root_created": False, "files_written": False,
        "sudo_used": False, "apparmor_loaded": False,
        "compiler_run": False, "make_run": False,
        "candidate_executed": False, "gpu_exposed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "admission"))
    args = parser.parse_args(argv)
    try:
        if args.command == "admission":
            raise BuildContractError("NOT_ADMITTED: final motion patch v2 parent hash is absent")
        report = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_BUILD_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
