#!/usr/bin/env python3
"""Fresh v8 S5A1 one-shot contract with a strict CLI and closed receipts.

``self-check`` is static: it reads only frozen contracts and preserved receipts.
Real bag access remains behind ``run-one-shot`` plus the exact execution-id and
execution-policy hash authorization.  The worker remains sandbox-only.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_execution_supervisor_v6 as v6
import r8_liquid_s5a1_handoff_gate_v6 as gate
import r8_liquid_s5a1_ros1_signal_extractor_v4 as extractor


sys.dont_write_bytecode = True
base = v6.base
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v8.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v8.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v8.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v8.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T221159Z_v8"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT = Path(gate.PARTIAL_ROOT)
FINAL_ROOT = Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v8.json")
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
V6_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v2.execution_v6.json")
V6_PARTIAL = Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v2.partial")
V6_RECEIPT_SHA256 = "3051aedaf265a388e949a27807436d31b544fff7efa87a3433ef8ee081ef2408"
V6_RECEIPT_SCHEMA_SHA256 = "9709267b9d0abe0469da11a40e73159b03066a26e1c16fb036a9a3d6873e0484"
HOST_GROUPS = list(v6.HOST_GROUPS)
MAPPED_GROUPS = list(v6.MAPPED_GROUPS)
HOST_ENV = dict(v6.HOST_ENV)
CHILD_ENV = dict(v6.CHILD_ENV)
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None


class ExecutionV8Error(ValueError):
    """The v8 frozen execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    raw, full = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "mode": full["mode"], "size_bytes": full["size_bytes"]}


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV8Error(f"frozen file identity drifted: {path}")
    return observed


def preserved_v6() -> dict[str, Any]:
    receipt = file_identity(V6_RECEIPT)
    metadata = os.lstat(V6_RECEIPT)
    if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
            or receipt["sha256"] != V6_RECEIPT_SHA256 or receipt["mode"] != "0440"):
        raise ExecutionV8Error("preserved v6 receipt identity drifted")
    receipt_value = json.loads(V6_RECEIPT.read_bytes())
    v6_schema = json.loads(v6.RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator(v6_schema).validate(receipt_value)
    if (receipt_value["status"] != "S5A1_EXECUTION_FAILED_PRESERVED"
            or not receipt_value["failure"]):
        raise ExecutionV8Error("preserved v6 receipt is not the exact failure")
    root_metadata = os.lstat(V6_PARTIAL)
    if (not stat.S_ISDIR(root_metadata.st_mode) or V6_PARTIAL.is_symlink()
            or stat.S_IMODE(root_metadata.st_mode) != 0o700
            or root_metadata.st_nlink != 2 or any(V6_PARTIAL.iterdir())):
        raise ExecutionV8Error("preserved v6 partial is not an exact empty 0700 directory")
    return {
        "failure_receipt": receipt,
        "partial_root": {
            "path": str(V6_PARTIAL), "inode": root_metadata.st_ino,
            "mode": f"{stat.S_IMODE(root_metadata.st_mode):04o}",
            "size_bytes": root_metadata.st_size, "nlink": root_metadata.st_nlink,
            "entry_count": 0, "inventory_sha256": hashlib.sha256(b"").hexdigest(),
        },
    }


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    expected_keys = {"revision_kind", "base", "schema_id", "schema_version",
                     "document_type", "execution_id", "receipt_id",
                     "semantic_success_status"}
    if set(value) != expected_keys:
        raise ExecutionV8Error("v8 receipt schema delta is not closed")
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V8",
        "base": {"path": str(v6.RECEIPT_SCHEMA), "sha256": V6_RECEIPT_SCHEMA_SHA256},
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v8.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v8",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V8",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v8",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V6_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV8Error("v8 receipt schema delta values differ")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    raw = v6.RECEIPT_SCHEMA.read_bytes()
    if hashlib.sha256(raw).hexdigest() != V6_RECEIPT_SCHEMA_SHA256:
        raise ExecutionV8Error("base v6 receipt schema drifted")
    schema = copy.deepcopy(json.loads(raw))
    schema["$id"] = delta["schema_id"]
    schema["properties"]["schema_version"] = {"const": delta["schema_version"]}
    schema["properties"]["document_type"] = {"const": delta["document_type"]}
    schema["properties"]["execution_id"] = {"const": EXECUTION_ID}
    schema["properties"]["receipt_id"] = {"const": delta["receipt_id"]}
    schema["required"] += ["preserved_v6_failure", "v8_contract"]
    schema["properties"]["preserved_v6_failure"] = {"$ref": "#/$defs/preservedV6"}
    schema["properties"]["v8_contract"] = {"$ref": "#/$defs/v8Contract"}
    schema["$defs"]["preservedV6"] = {
        "type": "object", "additionalProperties": False,
        "required": ["failure_receipt", "partial_root"],
        "properties": {"failure_receipt": {"$ref": "#/$defs/file"},
                       "partial_root": {"$ref": "#/$defs/dirIdentity"}},
    }
    contract_names = ["execution_policy", "policy_schema", "receipt_schema", "supervisor",
                      "tests", "parent_v7_policy", "parent_v7_policy_schema",
                      "parent_v7_receipt_schema", "parent_v7_supervisor", "parent_v7_tests",
                      "base_v6_policy", "extractor_v4", "package_policy", "package_schema",
                      "package_gate", "package_tests"]
    schema["$defs"]["v8Contract"] = {
        "type": "object", "additionalProperties": False, "required": contract_names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in contract_names},
    }
    execution = schema["$defs"]["execution"]
    execution["required"].append("failure_receipt_schema_v8")
    execution["properties"]["failure_receipt_schema_v8"] = {"const": True}
    safety = schema["$defs"]["safety"]
    safety["required"].append("v6_evidence_preserved")
    safety["properties"]["v6_evidence_preserved"] = {"const": True}
    semantic = schema["$defs"]["semantic"]
    semantic["properties"]["status"] = {
        "enum": [delta["semantic_success_status"], "NOT_RUN_DUE_TO_FAILURE"]}
    schema["allOf"][0]["then"]["properties"]["semantic_validation"]["properties"]["status"] = {
        "const": delta["semantic_success_status"]}
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    return schema


def _load_policy(expected_sha256: str | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value = json.loads(raw)
    schema = json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    identity = {"path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
                "mode": full["mode"], "size_bytes": full["size_bytes"]}
    return value, identity


def configure_paths() -> None:
    v6._configure_paths()
    gate.install()
    base.SCRIPT = SCRIPT
    base.POLICY = gate.POLICY_PATH
    base.PACKAGE_SCHEMA = gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA = v6.RECEIPT_SCHEMA
    base.GATE = Path(gate.__file__).resolve()
    base.gate = gate
    base.EXTRACTOR = Path(extractor.__file__).resolve()
    base.extractor = extractor
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL
    v6.PARTIAL_ROOT = PARTIAL_ROOT
    v6.FINAL_ROOT = FINAL_ROOT
    v6.EXECUTION_ID = EXECUTION_ID


def raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int,
              source_fd: int) -> list[str]:
    argv = v6._raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    bind = argv.index("--bind")
    additions: list[str] = []
    dependencies = [Path(v6.__file__).resolve(), Path(gate.base.__file__).resolve(),
                    Path(extractor.extractor_v3.__file__).resolve()]
    for path in dependencies:
        if str(path) not in argv:
            additions += ["--ro-bind", str(path), str(path)]
    argv[bind:bind] = additions
    return argv


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102) -> list[str]:
    configure_paths()
    argv = raw_build(v6.v5.prior.S5A0_INNER_RECEIPT,
                     Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
                     PARTIAL_ROOT, receipt_fd=receipt_fd, source_fd=source_fd)
    return v6._canonical(argv, receipt_fd, source_fd)


def _iter_file_refs(value: Any):
    if isinstance(value, dict):
        if {"path", "sha256", "mode", "size_bytes"}.issubset(value):
            yield value
        for child in value.values():
            yield from _iter_file_refs(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_file_refs(child)


def admit_policy(expected_sha256: str | None = None) -> dict[str, Any]:
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV8Error("host supplementary groups differ from exact frozen list")
    policy, identity = _load_policy(expected_sha256)
    path_map = {
        "parent_v7": {
            "policy": ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v7.json",
            "policy_schema": ROOT / "schema/target_host_s5a1_execution_policy_v7.json",
            "receipt_schema": ROOT / "schema/target_host_s5a1_execution_receipt_v7.json",
            "supervisor": ROOT / "scripts/r8_liquid_s5a1_execution_supervisor_v7.py",
            "tests": ROOT / "tests/test_s5a1_execution_supervisor_v7.py",
        },
        "package_v4": {"policy": gate.POLICY_PATH, "schema": gate.SCHEMA_PATH,
                       "gate": Path(gate.__file__).resolve(),
                       "tests": ROOT / "tests/test_s5a1_handoff_gate_v6.py"},
        "v8_contract": {"policy_schema": POLICY_SCHEMA, "receipt_schema": RECEIPT_SCHEMA,
                        "supervisor": SCRIPT, "tests": TESTS},
    }
    for section, paths in path_map.items():
        for name, path in paths.items():
            assert_identity(path, policy[section][name])
    assert_identity(v6.POLICY, policy["base_v6_policy"])
    assert_identity(Path(extractor.__file__).resolve(), policy["extractor_v4"])
    assert_identity(ROOT / "tests/test_s5a1_ros1_signal_extractor_v4.py",
                    policy["extractor_v4_tests"])
    base_v6_policy, base_v6_identity = v6._load_policy(policy["base_v6_policy"]["sha256"])
    if base_v6_identity != policy["base_v6_policy"]:
        raise ExecutionV8Error("base v6 policy identity differs")
    for section in ("package_revision", "v6_contract"):
        for parent in base_v6_policy[section].values():
            assert_identity(Path(parent["path"]), parent)
    assert_identity(Path(base_v6_policy["base_v5"]["path"]), base_v6_policy["base_v5"])
    base_v5_policy, _base_v5_identity = v6.v5._load_policy(base_v6_policy["base_v5"]["sha256"])
    for parent in base_v5_policy["contracts"].values():
        v6.v5._file_identity(Path(parent["path"]), parent)
    for tool in base_v5_policy["tools"].values():
        v6.v5._tool_identity(Path(tool["path"]), tool)
    if preserved_v6() != policy["preserved_v6"]:
        raise ExecutionV8Error("preserved v6 evidence differs")
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    canonical = canonical_argv()
    if (v6._argv_hash(canonical) != policy["sandbox"]["canonical_argv_sha256"]
            or len(canonical) != policy["sandbox"]["canonical_token_count"]):
        raise ExecutionV8Error("v8 canonical argv differs")
    if str(SCRIPT) not in canonical or canonical.count("--bind") != 1:
        raise ExecutionV8Error("v8 worker path or writable bind count differs")
    index = canonical.index("--bind")
    if canonical[index + 1:index + 3] != [str(PARTIAL_ROOT), str(PARTIAL_ROOT)]:
        raise ExecutionV8Error("v8 writable bind differs")
    v6.v5._assert_child_environment(canonical)
    return {"canonical_argv_sha256": v6._argv_hash(canonical),
            "canonical_token_count": len(canonical)}


def build_bwrap_argv(receipt: Path, source: Path, partial: Path, *, receipt_fd=None,
                     source_fd=None) -> list[str]:
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV8Error("admitted v8 policy and both fds are required")
    before = (base.publish_reserved_receipt, base.mock_failure_receipt)
    argv = raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    if before != (base.publish_reserved_receipt, base.mock_failure_receipt):
        raise ExecutionV8Error("v8 publication hooks were clobbered")
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if (v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]
            or len(canonical) != ACTIVE_POLICY["sandbox"]["canonical_token_count"]):
        raise ExecutionV8Error("runtime argv expansion differs")
    return argv


def load_package(root: Path) -> dict[str, bytes]:
    package = v6._BASE_LOAD_PACKAGE(root)
    report = gate.validate_package_root(root)
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV8Error("v8 package strong semantic validation failed")
    v6._SEMANTIC_ROOTS.append(str(root))
    v6._SEMANTIC_REPORT = dict(report)
    return package


def _ref(value: Mapping[str, Any]) -> dict[str, str]:
    return {"path": value["path"], "sha256": value["sha256"]}


def v8_contract() -> dict[str, dict[str, str]]:
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV8Error("v8 policy is not admitted")
    p = ACTIVE_POLICY
    return {
        "execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
        "policy_schema": _ref(p["v8_contract"]["policy_schema"]),
        "receipt_schema": _ref(p["v8_contract"]["receipt_schema"]),
        "supervisor": _ref(p["v8_contract"]["supervisor"]),
        "tests": _ref(p["v8_contract"]["tests"]),
        "parent_v7_policy": _ref(p["parent_v7"]["policy"]),
        "parent_v7_policy_schema": _ref(p["parent_v7"]["policy_schema"]),
        "parent_v7_receipt_schema": _ref(p["parent_v7"]["receipt_schema"]),
        "parent_v7_supervisor": _ref(p["parent_v7"]["supervisor"]),
        "parent_v7_tests": _ref(p["parent_v7"]["tests"]),
        "base_v6_policy": _ref(p["base_v6_policy"]),
        "extractor_v4": _ref(p["extractor_v4"]),
        "package_policy": _ref(p["package_v4"]["policy"]),
        "package_schema": _ref(p["package_v4"]["schema"]),
        "package_gate": _ref(p["package_v4"]["gate"]),
        "package_tests": _ref(p["package_v4"]["tests"]),
    }


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    value = copy.deepcopy(value)
    value["execution"]["supplementary_groups"] = HOST_GROUPS
    transformed = v6.transform_receipt(value)
    transformed["schema_version"] = "smpcc-r8-liquid-s5a1-execution-receipt-v8"
    transformed["document_type"] = "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V8"
    transformed["execution_id"] = EXECUTION_ID
    transformed["receipt_id"] = f"{TRANSFER_ID}_execution_v8"
    transformed["preserved_v6_failure"] = preserved_v6()
    transformed["v8_contract"] = v8_contract()
    transformed["execution"]["failure_receipt_schema_v8"] = True
    transformed["safety"]["v6_evidence_preserved"] = True
    if transformed["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        transformed["semantic_validation"]["status"] = "PASS_S5A1_HANDOFF_GATE_V6_STRONG_SEMANTICS"
    return transformed


def reserve_receipt() -> int:
    return v6._BASE_RESERVE(EXECUTION_RECEIPT_PARTIAL)


def publish_reserved_receipt(descriptor: int, partial_path: Path, final_path: Path,
                             value: Mapping[str, Any]) -> None:
    if base.publish_reserved_receipt is not publish_reserved_receipt:
        raise ExecutionV8Error("v8 publish hook was clobbered")
    transformed = transform_receipt(value)
    Draft202012Validator(receipt_schema()).validate(transformed)
    v6._BASE_PUBLISH(descriptor, partial_path, final_path, transformed)


def mock_failure_receipt() -> dict[str, Any]:
    return transform_receipt(v6._BASE_MOCK_FAILURE())


def install_runtime() -> None:
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV8Error("v8 policy is not admitted")
    configure_paths()
    base_v6_policy, base_v6_identity = v6._load_policy(ACTIVE_POLICY["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY = copy.deepcopy(base_v6_policy)
    v6._ACTIVE_POLICY_IDENTITY = base_v6_identity
    base_v5_policy, _identity = v6.v5._load_policy(base_v6_policy["base_v5"]["sha256"])
    expected = {Path(item["path"]): item["sha256"] for item in base_v5_policy["contracts"].values()}
    for item in _iter_file_refs(ACTIVE_POLICY):
        expected[Path(item["path"])] = item["sha256"]
    base.EXPECTED = expected
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt
    base.subprocess = v6._SUBPROCESS_PROXY


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != MAPPED_GROUPS:
        raise ExecutionV8Error("worker mapped supplementary groups differ")
    configure_paths()
    report = v6._BASE_WORKER(receipt_path, source, partial)
    report["worker_supplementary_groups"] = list(MAPPED_GROUPS)
    return report


def static_self_check() -> dict[str, Any]:
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    report = admit_policy()
    install_runtime()
    effective_receipt_schema = receipt_schema()
    Draft202012Validator(effective_receipt_schema).validate(mock_failure_receipt())
    gate_report = gate.self_check()
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V8_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID, "transfer_id": TRANSFER_ID,
        "policy_sha256": ACTIVE_POLICY_IDENTITY["sha256"],
        "canonical_argv_sha256": report["canonical_argv_sha256"],
        "canonical_token_count": report["canonical_token_count"],
        "package_gate_status": gate_report["status"],
        "receipt_schema_deep_closed": True, "strict_cli": True,
        "real_bag_read": False, "optional_bag_read": False, "worker_run": False,
        "bwrap_run": False, "files_written": False, "gpu_exposed": False,
        "solver_executed": False, "sudo_used": False, "network_used": False,
    }


def execute_one_shot(receipt_path: Path, receipt_sha256: str,
                     policy_sha256: str) -> dict[str, Any]:
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    v6._WORKER_GROUP_WITNESS = False
    admit_policy(policy_sha256)
    install_runtime()
    v6._BASE_EXECUTE(receipt_path, receipt_sha256)
    value = json.loads(EXECUTION_RECEIPT.read_bytes())
    Draft202012Validator(receipt_schema()).validate(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    run = subparsers.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt", type=Path, required=True)
    run.add_argument("--s5a0-receipt-sha256", required=True)
    run.add_argument("--execution-policy-sha256", required=True)
    run.add_argument("--authorize-execution-id", required=True)
    sandbox = subparsers.add_parser("_worker")
    sandbox.add_argument("--s5a0-receipt", type=Path, required=True)
    sandbox.add_argument("--source", type=Path, required=True)
    sandbox.add_argument("--partial-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "run-one-shot":
            if args.authorize_execution_id != EXECUTION_ID:
                raise ExecutionV8Error("exact v8 execution authorization differs")
            result = execute_one_shot(args.s5a0_receipt, args.s5a0_receipt_sha256,
                                      args.execution_policy_sha256)
        else:
            result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "execution_id": EXECUTION_ID,
                          "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
