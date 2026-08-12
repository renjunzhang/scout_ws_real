#!/usr/bin/env python3
"""Fresh S5A1 v13 contract with a complete shared-runtime binding guard."""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from jsonschema import Draft202012Validator

MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5a1_execution_supervisor_v12 as v12
import r8_liquid_s5a1_handoff_gate_v11 as gate

sys.dont_write_bytecode = True
v11, v10, v9, v6 = v12.v11, v12.v10, v12.v9, v12.v6
base, extractor = v12.base, v12.extractor
parent_v4 = v6.v5.prior
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v13.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v13.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v13.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v13.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260812T000300Z_v13"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(
    f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v13.json"
)
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
PRIMARY_BAG = Path(
    "/home/zrj/slosh_bags/matrix_bags/"
    "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
)

V12_PARTIAL, V12_FINAL = v12.PARTIAL_ROOT, v12.FINAL_ROOT
V12_RECEIPT, V12_RESERVATION = v12.EXECUTION_RECEIPT, v12.EXECUTION_RECEIPT_PARTIAL
V12_RECEIPT_SHA256 = "b91ea39194869e9b3363f9c7d83b2f54b2991df93a579b44570f3fdee7221d7a"
V12_PARTIAL_INODE = 15748917
V12_PARTIAL_INVENTORY_SHA256 = (
    "20939ba803430ff4bcd90123f6d884cd50f036557f167c4d424c612ea250ccca"
)
PRIMARY_BAG_SHA256 = "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"

S5A0_FINAL_RECEIPT = parent_v4.S5A0_FINAL_RECEIPT
S5A0_EVIDENCE_ROOT = parent_v4.S5A0_EVIDENCE_ROOT
S5A0_INNER_RECEIPT = parent_v4.S5A0_INNER_RECEIPT
BRIDGE = parent_v4.BRIDGE_V1
READER = parent_v4.READERS[3]
COMPAT_GATE = parent_v4.COMPAT_GATE
HOST_GROUPS, MAPPED_GROUPS = list(v12.HOST_GROUPS), list(v12.MAPPED_GROUPS)
HOST_ENV = dict(v12.HOST_ENV)

RUNTIME_BINDING_FIELDS = (
    "SCRIPT",
    "POLICY",
    "PACKAGE_SCHEMA",
    "RECEIPT_SCHEMA",
    "GATE",
    "EXTRACTOR",
    "BRIDGE",
    "READER",
    "COMPAT_GATE",
    "TRANSFER_ID",
    "PARTIAL_ROOT",
    "FINAL_ROOT",
    "EXECUTION_RECEIPT",
    "EXECUTION_RECEIPT_PARTIAL",
    "S5A0_FINAL_RECEIPT",
    "S5A0_EVIDENCE_ROOT",
    "S5A0_INNER_RECEIPT",
    "EXPECTED",
    "gate",
    "extractor",
    "build_bwrap_argv",
    "load_package",
    "reserve_receipt",
    "publish_reserved_receipt",
    "mock_failure_receipt",
    "subprocess",
)
IDENTITY_BINDING_FIELDS = {
    "EXPECTED",
    "gate",
    "extractor",
    "build_bwrap_argv",
    "load_package",
    "reserve_receipt",
    "publish_reserved_receipt",
    "mock_failure_receipt",
    "subprocess",
}

ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None


class ExecutionV13Error(ValueError):
    """The v13 execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    raw, full = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "mode": full["mode"],
        "size_bytes": full["size_bytes"],
    }


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV13Error(f"frozen file identity drifted: {path}")
    return observed


def _expected_witness(value: Mapping[Path, str]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ExecutionV13Error("EXPECTED is not a mapping")
    normalized: list[tuple[str, str]] = []
    for path, digest in value.items():
        candidate = Path(path)
        if not isinstance(digest, str) or len(digest) != 64:
            raise ExecutionV13Error(f"EXPECTED has an invalid digest: {candidate}")
        normalized.append((str(candidate), digest))
    normalized.sort()
    raw = b"".join(f"{path}\0{digest}\n".encode() for path, digest in normalized)
    return {
        "entry_count": len(normalized),
        "canonical_sha256": hashlib.sha256(raw).hexdigest(),
        "entries": normalized,
    }


def snapshot_runtime_bindings() -> dict[str, Any]:
    bindings = {name: getattr(base, name) for name in RUNTIME_BINDING_FIELDS}
    expected = bindings["EXPECTED"]
    if not isinstance(expected, dict):
        raise ExecutionV13Error("EXPECTED must be a mutable dict for guarded restoration")
    return {
        "bindings": bindings,
        "expected_items": dict(expected),
        "expected_witness": _expected_witness(expected),
    }


def assert_runtime_bindings(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    bindings = snapshot.get("bindings")
    if not isinstance(bindings, dict) or tuple(bindings) != RUNTIME_BINDING_FIELDS:
        raise ExecutionV13Error("runtime binding snapshot fields differ from the exact 26-field contract")
    for name in RUNTIME_BINDING_FIELDS:
        observed, expected = getattr(base, name), bindings[name]
        if name in IDENTITY_BINDING_FIELDS:
            if observed is not expected:
                raise ExecutionV13Error(f"runtime binding object identity drifted: {name}")
        elif observed != expected:
            raise ExecutionV13Error(f"runtime binding value drifted: {name}")
    if _expected_witness(base.EXPECTED) != snapshot["expected_witness"]:
        raise ExecutionV13Error("EXPECTED content drifted in place")
    return {
        "status": "PASS_RUNTIME_BINDING_GUARD",
        "field_count": len(RUNTIME_BINDING_FIELDS),
        "fields": list(RUNTIME_BINDING_FIELDS),
        "expected_entry_count": snapshot["expected_witness"]["entry_count"],
        "expected_canonical_sha256": snapshot["expected_witness"]["canonical_sha256"],
    }


def restore_runtime_bindings(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    bindings = snapshot["bindings"]
    expected = bindings["EXPECTED"]
    expected.clear()
    expected.update(snapshot["expected_items"])
    for name in RUNTIME_BINDING_FIELDS:
        setattr(base, name, bindings[name])
    return assert_runtime_bindings(snapshot)


def _guard_parent_call(function: Callable[..., Any], /, *args: Any, **kwargs: Any) -> Any:
    snapshot = snapshot_runtime_bindings()
    assert_runtime_bindings(snapshot)
    try:
        return function(*args, **kwargs)
    finally:
        restore_runtime_bindings(snapshot)


def _directory_identity(path: Path) -> dict[str, Any]:
    metadata = os.lstat(path)
    if (
        path.is_symlink()
        or not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_ino != V12_PARTIAL_INODE
        or stat.S_IMODE(metadata.st_mode) != 0o700
        or metadata.st_nlink != 2
    ):
        raise ExecutionV13Error("preserved v12 partial root identity drifted")
    observed = base.root_identity(path)
    expected = {
        "path": str(path),
        "exists": True,
        "mode": "0700",
        "entry_count": 20,
        "inventory_sha256": V12_PARTIAL_INVENTORY_SHA256,
    }
    if observed != expected:
        raise ExecutionV13Error("preserved v12 partial inventory drifted")
    return {
        "path": str(path),
        "inode": metadata.st_ino,
        "mode": "0700",
        "size_bytes": metadata.st_size,
        "nlink": metadata.st_nlink,
        "entry_count": 20,
        "inventory_sha256": V12_PARTIAL_INVENTORY_SHA256,
    }


def preserved_v12() -> dict[str, Any]:
    partial = _directory_identity(V12_PARTIAL)
    if os.path.lexists(V12_FINAL):
        raise ExecutionV13Error("unexpected v12 final transfer exists")
    if os.path.lexists(V12_RESERVATION):
        raise ExecutionV13Error("unexpected v12 receipt reservation exists")
    receipt_identity = file_identity(V12_RECEIPT)
    expected_receipt = {
        "path": str(V12_RECEIPT),
        "sha256": V12_RECEIPT_SHA256,
        "mode": "0440",
        "size_bytes": 28472,
    }
    if receipt_identity != expected_receipt:
        raise ExecutionV13Error("preserved v12 receipt identity drifted")
    receipt = json.loads(V12_RECEIPT.read_bytes())
    safety = receipt.get("safety", {})
    source_before = receipt.get("inputs", {}).get("source_before")
    source_after = receipt.get("inputs", {}).get("source_after")
    if (
        receipt.get("schema_version") != "smpcc-r8-liquid-s5a1-execution-receipt-v12"
        or receipt.get("execution_id") != v12.EXECUTION_ID
        or receipt.get("status") != "S5A1_EXECUTION_FAILED_PRESERVED"
        or receipt.get("failure") != "manifest policy parent.path differs"
        or receipt.get("execution", {}).get("return_code") != 0
        or safety.get("real_bag_read_attempted") is not True
        or safety.get("real_bag_read_confirmed") is not True
        or safety.get("bwrap_run") is not True
        or safety.get("optional_bag_read") is not False
        or safety.get("gpu_exposed") is not False
        or safety.get("solver_executed") is not False
        or safety.get("network_used") is not False
        or safety.get("sudo_used") is not False
        or source_before != source_after
        or source_after.get("sha256") != PRIMARY_BAG_SHA256
        or receipt.get("roots", {}).get("partial_after", {}).get("inventory_sha256")
        != V12_PARTIAL_INVENTORY_SHA256
    ):
        raise ExecutionV13Error("preserved v12 receipt facts drifted")
    report = gate.prior.validate_package_root(V12_PARTIAL)
    gate.install()
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV13Error("preserved v12 package no longer passes its exact semantic gate")
    return {
        "partial_root": partial,
        "final_root_absent": True,
        "receipt": receipt_identity,
        "receipt_reservation_absent": True,
        "status": "S5A1_EXECUTION_FAILED_PRESERVED",
        "failure": "manifest policy parent.path differs",
        "return_code": 0,
        "primary_bag_post_sha256": PRIMARY_BAG_SHA256,
        "real_bag_read_attempted": True,
        "real_bag_read_confirmed": True,
        "bwrap_run": True,
        "package_entries": 20,
    }


def assert_fresh_v13() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV13Error(f"v13 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    parent_paths = {
        "policy": v12.POLICY,
        "policy_schema": v12.POLICY_SCHEMA,
        "receipt_schema": v12.RECEIPT_SCHEMA,
        "supervisor": Path(v12.__file__).resolve(),
        "tests": v12.TESTS,
    }
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V13",
        "parent_v12": {
            name: {"path": str(path), "sha256": file_identity(path)["sha256"]}
            for name, path in parent_paths.items()
        },
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v13.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v13",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V13",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v13",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V11_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV13Error("v13 receipt schema delta differs or is not closed")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    schema = copy.deepcopy(v12.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v12", "v13_contract"]
    schema["properties"]["preserved_v12"] = {"$ref": "#/$defs/preservedV12"}
    schema["properties"]["v13_contract"] = {"$ref": "#/$defs/v13Contract"}
    schema["$defs"]["preservedV12Dir"] = {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "path", "inode", "mode", "size_bytes", "nlink", "entry_count",
            "inventory_sha256",
        ],
        "properties": {
            "path": {"const": str(V12_PARTIAL)},
            "inode": {"const": V12_PARTIAL_INODE},
            "mode": {"const": "0700"},
            "size_bytes": {"const": 4096},
            "nlink": {"const": 2},
            "entry_count": {"const": 20},
            "inventory_sha256": {"const": V12_PARTIAL_INVENTORY_SHA256},
        },
    }
    schema["$defs"]["preservedV12"] = {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "partial_root", "final_root_absent", "receipt",
            "receipt_reservation_absent", "status", "failure", "return_code",
            "primary_bag_post_sha256", "real_bag_read_attempted",
            "real_bag_read_confirmed", "bwrap_run", "package_entries",
        ],
        "properties": {
            "partial_root": {"$ref": "#/$defs/preservedV12Dir"},
            "final_root_absent": {"const": True},
            "receipt": {
                "type": "object",
                "additionalProperties": False,
                "required": ["path", "sha256", "mode", "size_bytes"],
                "properties": {
                    "path": {"const": str(V12_RECEIPT)},
                    "sha256": {"const": V12_RECEIPT_SHA256},
                    "mode": {"const": "0440"},
                    "size_bytes": {"const": 28472},
                },
            },
            "receipt_reservation_absent": {"const": True},
            "status": {"const": "S5A1_EXECUTION_FAILED_PRESERVED"},
            "failure": {"const": "manifest policy parent.path differs"},
            "return_code": {"const": 0},
            "primary_bag_post_sha256": {"const": PRIMARY_BAG_SHA256},
            "real_bag_read_attempted": {"const": True},
            "real_bag_read_confirmed": {"const": True},
            "bwrap_run": {"const": True},
            "package_entries": {"const": 20},
        },
    }
    names = [
        "execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
        "parent_v12_policy", "parent_v12_policy_schema", "parent_v12_receipt_schema",
        "parent_v12_supervisor", "parent_v12_tests", "package_policy", "package_schema",
        "package_gate", "package_tests",
    ]
    schema["$defs"]["v13Contract"] = {
        "type": "object",
        "additionalProperties": False,
        "required": names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in names},
    }
    execution = schema["$defs"]["execution"]
    for name in ("failure_receipt_schema_v13", "runtime_bindings_guarded"):
        if name not in execution["required"]:
            execution["required"].append(name)
        execution["properties"][name] = {"const": True}
    safety = schema["$defs"]["safety"]
    for name in ("v12_evidence_preserved", "runtime_bindings_guarded"):
        if name not in safety["required"]:
            safety["required"].append(name)
        safety["properties"][name] = {"const": True}
    schema["$defs"]["semantic"]["properties"]["status"] = {
        "enum": [delta["semantic_success_status"], "NOT_RUN_DUE_TO_FAILURE"]
    }
    schema["allOf"][0]["then"]["properties"]["semantic_validation"]["properties"]["status"] = {
        "const": delta["semantic_success_status"]
    }
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    return schema


def _load_policy(expected_sha256: str | None = None):
    raw, full = base.read_file_identity(
        POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024
    )
    value = json.loads(raw)
    schema = json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    return value, {
        "path": str(POLICY),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "mode": full["mode"],
        "size_bytes": full["size_bytes"],
    }


def _apply_v13_paths() -> None:
    base.SCRIPT = SCRIPT
    base.POLICY = gate.POLICY_PATH
    base.PACKAGE_SCHEMA = gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA = v6.RECEIPT_SCHEMA
    base.GATE = Path(gate.__file__).resolve()
    base.EXTRACTOR = Path(extractor.__file__).resolve()
    base.BRIDGE, base.READER, base.COMPAT_GATE = BRIDGE, READER, COMPAT_GATE
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = (
        TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    )
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL
    base.S5A0_FINAL_RECEIPT = S5A0_FINAL_RECEIPT
    base.S5A0_EVIDENCE_ROOT = S5A0_EVIDENCE_ROOT
    base.S5A0_INNER_RECEIPT = S5A0_INNER_RECEIPT
    base.gate, base.extractor = gate, extractor
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = (
        PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID
    )


def _install_runtime_bindings(expected: dict[Path, str] | None = None) -> dict[str, Any]:
    _apply_v13_paths()
    if expected is not None:
        base.EXPECTED = expected
    elif not isinstance(base.EXPECTED, dict):
        raise ExecutionV13Error("existing EXPECTED binding is not a dict")
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt
    base.subprocess = SUBPROCESS_PROXY
    snapshot = snapshot_runtime_bindings()
    assert_runtime_bindings(snapshot)
    return snapshot


def configure_paths() -> None:
    v12.configure_paths()
    gate.install()
    _apply_v13_paths()


def _local_imports(path: Path) -> set[Path]:
    tree = ast.parse(path.read_bytes(), filename=str(path))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".", 1)[0])
    paths: set[Path] = set()
    for name in names:
        if not name.startswith("r8_liquid_"):
            continue
        candidate = (MODULE_DIR / f"{name}.py").resolve()
        if candidate.parent != MODULE_DIR or not candidate.is_file() or candidate.is_symlink():
            raise ExecutionV13Error(f"unresolved or unsafe local import: {name}")
        paths.add(candidate)
    return paths


def local_import_closure(start: Path = SCRIPT) -> tuple[dict[Path, str], dict[str, Any]]:
    pending = [start.resolve()]
    seen: set[Path] = set()
    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        pending.extend(sorted(_local_imports(path) - seen, key=str))
    identities = {path: file_identity(path)["sha256"] for path in sorted(seen, key=str)}
    raw = b"".join(
        f"{path}\0{identities[path]}\n".encode() for path in sorted(identities, key=str)
    )
    return identities, {
        "count": len(identities),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "paths": [str(path) for path in sorted(identities, key=str)],
    }


def _ro_bind_paths(argv: Sequence[str]) -> set[Path]:
    bound: set[Path] = set()
    for index, token in enumerate(argv):
        if token == "--ro-bind" and index + 2 < len(argv) and argv[index + 1] == argv[index + 2]:
            bound.add(Path(argv[index + 2]))
    return bound


def assert_import_closure_bound(argv: Sequence[str]) -> dict[str, Any]:
    closure, witness = local_import_closure()
    bound = _ro_bind_paths(argv)
    missing = sorted(str(path) for path in closure if path not in bound)
    required_data = {gate.POLICY_PATH, gate.SCHEMA_PATH}
    missing_data = sorted(str(path) for path in required_data if path not in bound)
    if missing or missing_data:
        raise ExecutionV13Error(
            "sandbox import/data closure is incomplete: " + ",".join(missing + missing_data)
        )
    return {"local_import_closure": witness, "missing": [], "data_missing": []}


def raw_build(
    receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int
) -> list[str]:
    outer_runtime = snapshot_runtime_bindings()
    try:
        configure_paths()
        runtime = _install_runtime_bindings()
        argv = _guard_parent_call(
            v12.raw_build,
            receipt,
            source,
            partial,
            receipt_fd=receipt_fd,
            source_fd=source_fd,
        )
        assert_runtime_bindings(runtime)
        marker = argv.index("--")
        if argv[marker + 1:marker + 3] != ["/usr/bin/python3", str(v12.SCRIPT)]:
            raise ExecutionV13Error("parent v12 worker command identity differs")
        bound = _ro_bind_paths(argv)
        closure, _witness = local_import_closure()
        additions: list[str] = []
        for path in sorted(set(closure) | {gate.POLICY_PATH, gate.SCHEMA_PATH}, key=str):
            if path not in bound:
                additions += ["--ro-bind", str(path), str(path)]
        bind = argv.index("--bind")
        argv[bind:bind] = additions
        marker = argv.index("--")
        argv[marker + 2] = str(SCRIPT)
        assert_import_closure_bound(argv)
        assert_runtime_bindings(runtime)
        return argv
    finally:
        restore_runtime_bindings(outer_runtime)


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102) -> list[str]:
    return v6._canonical(
        raw_build(
            S5A0_INNER_RECEIPT,
            PRIMARY_BAG,
            PARTIAL_ROOT,
            receipt_fd=receipt_fd,
            source_fd=source_fd,
        ),
        receipt_fd,
        source_fd,
    )


def _iter_file_refs(value: Any):
    if isinstance(value, dict):
        if {"path", "sha256", "mode", "size_bytes"}.issubset(value):
            yield value
        for child in value.values():
            yield from _iter_file_refs(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_file_refs(child)


def recursive_expected_map(policy: Mapping[str, Any]) -> tuple[dict[Path, str], dict[str, Any]]:
    parent = json.loads(v12.POLICY.read_bytes())
    expected, parent_witness = v12.recursive_expected_map(parent)
    excluded = set(parent_witness["excluded_paths"])
    for item in _iter_file_refs(policy):
        path = Path(item["path"])
        if item["size_bytes"] == 0:
            raise ExecutionV13Error(f"unexpected zero-byte v13 file reference: {path}")
        if path in expected and expected[path] != item["sha256"]:
            raise ExecutionV13Error(f"conflicting recursive parent hash: {path}")
        expected[path] = item["sha256"]
    policy_identity = file_identity(POLICY)
    expected[POLICY] = policy_identity["sha256"]
    exact_exclusions = {
        str(v11.V8_RESERVATION),
        str(v11.V10_RESERVATION),
        str(v12.V11_RESERVATION),
    }
    if excluded != exact_exclusions:
        raise ExecutionV13Error("exact v8/v10/v11 reservations must remain the only exclusions")
    closure, closure_witness = local_import_closure()
    sandbox = policy["sandbox"]
    if (
        closure_witness["count"] != sandbox["local_python_import_closure_count"]
        or closure_witness["sha256"] != sandbox["local_python_import_closure_sha256"]
    ):
        raise ExecutionV13Error("local Python import closure differs")
    for path, digest in closure.items():
        if path in expected and expected[path] != digest:
            raise ExecutionV13Error(f"conflicting local import identity: {path}")
        expected[path] = digest
    return expected, {
        "excluded_paths": sorted(excluded),
        "excluded_unique_count": 3,
        "parent_v12": parent_witness,
        "local_import_closure": closure_witness,
        "all_other_zero_byte_refs_fail_closed": True,
    }


def required_base_receipt_parent_paths() -> tuple[Path, ...]:
    return tuple(
        dict.fromkeys(
            (
                base.SCRIPT,
                base.POLICY,
                base.PACKAGE_SCHEMA,
                base.RECEIPT_SCHEMA,
                base.GATE,
                base.EXTRACTOR,
                base.BRIDGE,
                base.READER,
                base.S5A0_SCHEMA,
                base.S5A0_FINAL_SCHEMA,
                base.S5A0_SUPERVISOR_POLICY,
                base.COMPAT_GATE,
            )
        )
    )


def assert_base_receipt_parent_coverage(expected: Mapping[Path, str]) -> dict[str, Any]:
    required = required_base_receipt_parent_paths()
    missing = [str(path) for path in required if path not in expected]
    if missing:
        raise ExecutionV13Error("base receipt parent coverage missing: " + ",".join(missing))
    return {
        "required_count": len(required),
        "missing": [],
        "status": "PASS_BASE_RECEIPT_PARENT_COVERAGE",
    }


def recursive_parent_v12_admission(expected_sha256: str) -> dict[str, Any]:
    original_fresh = v12.assert_fresh_v12
    previous_parent = (
        v12.ACTIVE_POLICY,
        v12.ACTIVE_POLICY_IDENTITY,
        v12.CAPTURED_TF_WITNESS,
    )
    runtime = snapshot_runtime_bindings()

    def preserved_instead_of_fresh() -> None:
        preserved_v12()

    try:
        v12.assert_fresh_v12 = preserved_instead_of_fresh
        return v12.admit_policy(expected_sha256)
    finally:
        v12.assert_fresh_v12 = original_fresh
        (
            v12.ACTIVE_POLICY,
            v12.ACTIVE_POLICY_IDENTITY,
            v12.CAPTURED_TF_WITNESS,
        ) = previous_parent
        restore_runtime_bindings(runtime)


def admit_policy(expected_sha256: str | None = None) -> dict[str, Any]:
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV13Error("host supplementary groups drifted")
    assert_fresh_v13()
    policy, identity = _load_policy(expected_sha256)
    sections = {
        "parent_v12": {
            "policy": v12.POLICY,
            "policy_schema": v12.POLICY_SCHEMA,
            "receipt_schema": v12.RECEIPT_SCHEMA,
            "supervisor": Path(v12.__file__).resolve(),
            "tests": v12.TESTS,
        },
        "package_v9": {
            "policy": gate.POLICY_PATH,
            "schema": gate.SCHEMA_PATH,
            "gate": Path(gate.__file__).resolve(),
            "tests": ROOT / "tests/test_s5a1_handoff_gate_v11.py",
        },
        "v13_contract": {
            "policy": POLICY_SCHEMA,
            "schema": RECEIPT_SCHEMA,
            "gate": SCRIPT,
            "tests": TESTS,
        },
    }
    for section, paths in sections.items():
        for name, path in paths.items():
            assert_identity(path, policy[section][name])
    if preserved_v12() != policy["preserved_v12"]:
        raise ExecutionV13Error("v12 preserved evidence differs")
    if policy["tf_contract"] != json.loads(v12.POLICY.read_bytes())["tf_contract"]:
        raise ExecutionV13Error("TF contract drifted")
    if tuple(policy["sandbox"]["runtime_binding_fields"]) != RUNTIME_BINDING_FIELDS:
        raise ExecutionV13Error("policy runtime binding field order differs")
    configure_paths()
    initial_runtime = _install_runtime_bindings()
    recursive_parent_v12_admission(policy["parent_v12"]["policy"]["sha256"])
    assert_runtime_bindings(initial_runtime)
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    expected, recursive = recursive_expected_map(policy)
    configure_paths()
    runtime = _install_runtime_bindings(expected)
    coverage = assert_base_receipt_parent_coverage(expected)
    closure, closure_witness = local_import_closure()
    for path, digest in closure.items():
        if expected.get(path) != digest:
            raise ExecutionV13Error(f"local import lacks frozen recursive identity: {path}")
    sandbox = policy["sandbox"]
    if (
        closure_witness["count"] != sandbox["local_python_import_closure_count"]
        or closure_witness["sha256"] != sandbox["local_python_import_closure_sha256"]
    ):
        raise ExecutionV13Error("local Python import closure differs")
    argv = canonical_argv()
    import_report = assert_import_closure_bound(argv)
    if (
        v6._argv_hash(argv) != sandbox["canonical_argv_sha256"]
        or len(argv) != sandbox["canonical_token_count"]
    ):
        raise ExecutionV13Error("canonical argv differs")
    marker = argv.index("--")
    if argv[marker + 2] != str(SCRIPT) or argv.count("--bind") != 1:
        raise ExecutionV13Error("worker path or writable bind differs")
    binding_report = assert_runtime_bindings(runtime)
    return {
        "canonical_argv_sha256": v6._argv_hash(argv),
        "canonical_token_count": len(argv),
        "generic_expected_count": len(expected),
        "recursive_coverage": recursive,
        "base_coverage": coverage,
        "import_coverage": import_report,
        "recursive_parent_v12_admission": True,
        "runtime_binding_guard": binding_report,
    }


def build_bwrap_argv(
    receipt: Path,
    source: Path,
    partial: Path,
    *,
    receipt_fd: int | None = None,
    source_fd: int | None = None,
) -> list[str]:
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV13Error("admission and both fds are required")
    before = snapshot_runtime_bindings()
    assert_runtime_bindings(before)
    argv = raw_build(
        receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd
    )
    restore_runtime_bindings(before)
    assert_import_closure_bound(argv)
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV13Error("runtime argv differs")
    assert_runtime_bindings(before)
    return argv


def load_package(root: Path) -> dict[str, bytes]:
    runtime = snapshot_runtime_bindings()
    assert_runtime_bindings(runtime)
    try:
        package = v6._BASE_LOAD_PACKAGE(root)
        report = gate.validate_package_root(root)
        if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
            raise ExecutionV13Error("strong semantic gate failed")
        v6._SEMANTIC_ROOTS.append(str(root))
        v6._SEMANTIC_REPORT = dict(report)
        return package
    finally:
        restore_runtime_bindings(runtime)


def _ref(value: Mapping[str, Any]) -> dict[str, str]:
    return {"path": value["path"], "sha256": value["sha256"]}


def v13_contract() -> dict[str, dict[str, str]]:
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV13Error("policy not admitted")
    policy = ACTIVE_POLICY
    return {
        "execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
        "policy_schema": _ref(policy["v13_contract"]["policy"]),
        "receipt_schema": _ref(policy["v13_contract"]["schema"]),
        "supervisor": _ref(policy["v13_contract"]["gate"]),
        "tests": _ref(policy["v13_contract"]["tests"]),
        "parent_v12_policy": _ref(policy["parent_v12"]["policy"]),
        "parent_v12_policy_schema": _ref(policy["parent_v12"]["policy_schema"]),
        "parent_v12_receipt_schema": _ref(policy["parent_v12"]["receipt_schema"]),
        "parent_v12_supervisor": _ref(policy["parent_v12"]["supervisor"]),
        "parent_v12_tests": _ref(policy["parent_v12"]["tests"]),
        "package_policy": _ref(policy["package_v9"]["policy"]),
        "package_schema": _ref(policy["package_v9"]["schema"]),
        "package_gate": _ref(policy["package_v9"]["gate"]),
        "package_tests": _ref(policy["package_v9"]["tests"]),
    }


def assert_raw_v1_origin(value: Mapping[str, Any]) -> None:
    try:
        v12.assert_raw_v1_origin(value)
    except Exception as exc:
        raise ExecutionV13Error(str(exc)) from exc


def _parent_v12_transform(value: Mapping[str, Any]) -> dict[str, Any]:
    if ACTIVE_POLICY is None:
        raise ExecutionV13Error("policy not admitted")
    parent = json.loads(v12.POLICY.read_bytes())
    previous = (
        v12.ACTIVE_POLICY,
        v12.ACTIVE_POLICY_IDENTITY,
        v12.CAPTURED_TF_WITNESS,
    )
    runtime = snapshot_runtime_bindings()
    try:
        v12.ACTIVE_POLICY = parent
        v12.ACTIVE_POLICY_IDENTITY = ACTIVE_POLICY["parent_v12"]["policy"]
        v12.CAPTURED_TF_WITNESS = copy.deepcopy(CAPTURED_TF_WITNESS)
        try:
            return v12.transform_receipt(value)
        except Exception as exc:
            raise ExecutionV13Error(f"parent v12 transform failed: {exc}") from exc
    finally:
        (
            v12.ACTIVE_POLICY,
            v12.ACTIVE_POLICY_IDENTITY,
            v12.CAPTURED_TF_WITNESS,
        ) = previous
        restore_runtime_bindings(runtime)


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    runtime = snapshot_runtime_bindings()
    assert_runtime_bindings(runtime)
    assert_raw_v1_origin(value)
    out = _parent_v12_transform(value)
    out.update(
        schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v13",
        document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V13",
        execution_id=EXECUTION_ID,
        receipt_id=f"{TRANSFER_ID}_execution_v13",
    )
    out["preserved_v12"] = preserved_v12()
    out["v13_contract"] = v13_contract()
    out["execution"].update(
        failure_receipt_schema_v13=True,
        runtime_bindings_guarded=True,
    )
    current_attempted = out["safety"]["real_bag_read_attempted"]
    current_confirmed = out["safety"]["real_bag_read_confirmed"]
    if not isinstance(current_attempted, bool) or not isinstance(current_confirmed, bool):
        raise ExecutionV13Error("current v13 real-bag read evidence is not boolean")
    if current_confirmed and not current_attempted:
        raise ExecutionV13Error("current v13 confirmed read without attempted read")
    out["safety"].update(
        v12_evidence_preserved=True,
        runtime_bindings_guarded=True,
        bwrap_run=current_attempted,
    )
    out["semantic_validation"]["gate"] = _ref(ACTIVE_POLICY["package_v9"]["gate"])
    if out["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and not (
        current_attempted and current_confirmed and out["safety"]["bwrap_run"]
    ):
        raise ExecutionV13Error("v13 success lacks current real-bag/bwrap evidence")
    if out["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"] = (
            "PASS_S5A1_HANDOFF_GATE_V11_STRONG_SEMANTICS"
        )
    assert_runtime_bindings(runtime)
    return out


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL) -> int:
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o440,
    )
    os.fchmod(descriptor, 0o440)
    return descriptor


def publish_reserved_receipt(
    descriptor: int,
    partial_path: Path,
    final_path: Path,
    value: Mapping[str, Any],
) -> None:
    runtime = snapshot_runtime_bindings()
    assert_runtime_bindings(runtime)
    try:
        assert_raw_v1_origin(value)
        transformed = transform_receipt(value)
        Draft202012Validator(receipt_schema()).validate(transformed)
        view = memoryview(gate.v1.canonical_json(transformed))
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExecutionV13Error("short v13 receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
        restore_runtime_bindings(runtime)
    base.rename_noreplace(partial_path, final_path)


def mock_failure_receipt() -> dict[str, Any]:
    value = v6._BASE_MOCK_FAILURE()
    value["execution"]["supplementary_groups"] = HOST_GROUPS
    return transform_receipt(value)


def _sanitized_subprocess_run(command: Sequence[str], **kwargs: Any):
    global CAPTURED_TF_WITNESS
    if "env" in kwargs:
        raise ExecutionV13Error("unfrozen host environment")
    runtime = snapshot_runtime_bindings()
    try:
        completed = v6.v5._REAL_SUBPROCESS_RUN(
            command, env=dict(HOST_ENV), **kwargs
        )
    finally:
        restore_runtime_bindings(runtime)
    v6._WORKER_GROUP_WITNESS = False
    CAPTURED_TF_WITNESS = None
    if completed.returncode == 0:
        try:
            report = json.loads(completed.stdout)
            v6._WORKER_GROUP_WITNESS = (
                report.get("worker_supplementary_groups") == MAPPED_GROUPS
            )
            witness = report.get("tf_qc")
            CAPTURED_TF_WITNESS = witness if isinstance(witness, dict) else None
        except Exception:
            CAPTURED_TF_WITNESS = None
    return completed


class _SubprocessProxy:
    DEVNULL, PIPE = v6._subprocess.DEVNULL, v6._subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY = _SubprocessProxy()


def install_runtime() -> dict[str, Any]:
    if ACTIVE_POLICY is None:
        raise ExecutionV13Error("policy not admitted")
    configure_paths()
    parent9 = json.loads(v9.POLICY.read_bytes())
    base_v6, base_v6_identity = v6._load_policy(
        parent9["base_v6_policy"]["sha256"]
    )
    v6._ACTIVE_POLICY = copy.deepcopy(base_v6)
    v6._ACTIVE_POLICY_IDENTITY = base_v6_identity
    expected, recursive = recursive_expected_map(ACTIVE_POLICY)
    coverage = assert_base_receipt_parent_coverage(expected)
    runtime = _install_runtime_bindings(expected)
    return {
        "recursive_coverage": recursive,
        "base_coverage": coverage,
        "runtime_binding_guard": assert_runtime_bindings(runtime),
    }


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != MAPPED_GROUPS:
        raise ExecutionV13Error("worker mapped supplementary groups differ")
    configure_paths()
    runtime = _install_runtime_bindings()
    try:
        report = v6._BASE_WORKER(receipt_path, source, partial)
    finally:
        restore_runtime_bindings(runtime)
    report["worker_supplementary_groups"] = MAPPED_GROUPS
    return report


def static_self_check() -> dict[str, Any]:
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS = None
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    report = admit_policy()
    runtime = install_runtime()
    schema = receipt_schema()
    Draft202012Validator(schema).validate(mock_failure_receipt())
    gate_report = gate.self_check()
    binding_report = assert_runtime_bindings(snapshot_runtime_bindings())
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V13_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "transfer_id": TRANSFER_ID,
        "policy_sha256": ACTIVE_POLICY_IDENTITY["sha256"],
        **report,
        "runtime_coverage": runtime,
        "package_gate_status": gate_report["status"],
        "receipt_schema_deep_closed": True,
        "strict_cli": True,
        "preserved_v12": preserved_v12(),
        "runtime_binding_guard": binding_report,
        "real_bag_read_attempted": False,
        "real_bag_read_confirmed": False,
        "worker_run": False,
        "bwrap_run": False,
        "files_written": False,
        "gpu_exposed": False,
        "solver_executed": False,
        "sudo_used": False,
        "network_used": False,
    }


def execute_one_shot(
    receipt_path: Path, receipt_sha256: str, policy_sha256: str
) -> dict[str, Any]:
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS = None
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    v6._WORKER_GROUP_WITNESS = False
    admit_policy(policy_sha256)
    install_runtime()
    runtime = snapshot_runtime_bindings()
    try:
        v6._BASE_EXECUTE(receipt_path, receipt_sha256)
    finally:
        restore_runtime_bindings(runtime)
    value = json.loads(EXECUTION_RECEIPT.read_bytes())
    Draft202012Validator(receipt_schema()).validate(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("self-check")
    run = commands.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt", type=Path, required=True)
    run.add_argument("--s5a0-receipt-sha256", required=True)
    run.add_argument("--execution-policy-sha256", required=True)
    run.add_argument("--authorize-execution-id", required=True)
    sandbox = commands.add_parser("_worker")
    sandbox.add_argument("--s5a0-receipt", type=Path, required=True)
    sandbox.add_argument("--source", type=Path, required=True)
    sandbox.add_argument("--partial-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "run-one-shot":
            if args.authorize_execution_id != EXECUTION_ID:
                raise ExecutionV13Error("exact v13 execution authorization differs")
            result = execute_one_shot(
                args.s5a0_receipt,
                args.s5a0_receipt_sha256,
                args.execution_policy_sha256,
            )
        else:
            result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as exc:
        print(
            json.dumps(
                {
                    "status": "FAIL_CLOSED",
                    "execution_id": EXECUTION_ID,
                    "error": str(exc),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
