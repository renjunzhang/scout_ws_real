#!/usr/bin/env python3
"""Fresh S5A1 v14 wrapper adding the missing v6 module-state guard."""

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
import r8_liquid_s5a1_execution_supervisor_v13 as parent
import r8_liquid_s5a1_handoff_gate_v12 as gate

sys.dont_write_bytecode = True
base, v6 = parent.base, parent.v6
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v14.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v14.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v14.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v14.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260812T010107Z_v14"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(
    f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v14.json"
)
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
PRIMARY_BAG = parent.PRIMARY_BAG
HOST_GROUPS, MAPPED_GROUPS, HOST_ENV = (
    parent.HOST_GROUPS,
    parent.MAPPED_GROUPS,
    parent.HOST_ENV,
)

V13_PARTIAL, V13_FINAL = parent.PARTIAL_ROOT, parent.FINAL_ROOT
V13_RECEIPT, V13_RESERVATION = (
    parent.EXECUTION_RECEIPT,
    parent.EXECUTION_RECEIPT_PARTIAL,
)
V13_FINAL_INODE = 15749004
V13_FINAL_INVENTORY_SHA256 = (
    "34a08d169bb6122be30be90445715ba0d9ac3380f2c4b80eecfea2a7bbd1c59c"
)
V13_RESERVATION_INODE = 14966687
V13_PUBLICATION_ERROR = (
    "parent v12 transform failed: parent v11 transform failed: "
    "success lacks both strong semantic validations"
)
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()
V6_MODULE_STATE_FIELDS = ("PARTIAL_ROOT", "FINAL_ROOT", "EXECUTION_ID")
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None


class ExecutionV14Error(ValueError):
    """The v14 execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    return parent.file_identity(path)


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV14Error(f"frozen file identity drifted: {path}")
    return observed


def snapshot_v6_module_state() -> dict[str, Any]:
    return {name: getattr(v6, name) for name in V6_MODULE_STATE_FIELDS}


def assert_v6_module_state(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    if tuple(snapshot) != V6_MODULE_STATE_FIELDS:
        raise ExecutionV14Error("v6 module-state fields differ")
    for name in V6_MODULE_STATE_FIELDS:
        if getattr(v6, name) != snapshot[name]:
            raise ExecutionV14Error(f"v6 module state drifted: {name}")
    return {
        "status": "PASS_V6_MODULE_STATE_GUARD",
        "field_count": 3,
        "fields": list(V6_MODULE_STATE_FIELDS),
    }


def restore_v6_module_state(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    for name in V6_MODULE_STATE_FIELDS:
        setattr(v6, name, snapshot[name])
    return assert_v6_module_state(snapshot)


def _fd_verify_empty(path: Path) -> dict[str, Any]:
    before = os.lstat(path)
    if path.is_symlink():
        raise ExecutionV14Error("v13 reservation is a symlink")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (
            before.st_ino != metadata.st_ino
            or not stat.S_ISREG(metadata.st_mode)
            or metadata.st_ino != V13_RESERVATION_INODE
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o440
            or metadata.st_size != 0
            or os.read(descriptor, 1) != b""
        ):
            raise ExecutionV14Error("v13 receipt reservation drifted")
    finally:
        os.close(descriptor)
    return {
        "path": str(path),
        "inode": metadata.st_ino,
        "mode": "0440",
        "size_bytes": 0,
        "nlink": 1,
        "sha256": EMPTY_SHA256,
    }


def preserved_v13() -> dict[str, Any]:
    if os.path.lexists(V13_PARTIAL) or os.path.lexists(V13_RECEIPT):
        raise ExecutionV14Error("unexpected v13 partial or final receipt exists")
    metadata = os.lstat(V13_FINAL)
    if (
        V13_FINAL.is_symlink()
        or not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_ino != V13_FINAL_INODE
        or stat.S_IMODE(metadata.st_mode) != 0o700
        or metadata.st_nlink != 2
    ):
        raise ExecutionV14Error("preserved v13 final root identity drifted")
    observed = base.root_identity(V13_FINAL)
    expected = {
        "path": str(V13_FINAL),
        "exists": True,
        "mode": "0700",
        "entry_count": 20,
        "inventory_sha256": V13_FINAL_INVENTORY_SHA256,
    }
    if observed != expected:
        raise ExecutionV14Error("preserved v13 final inventory drifted")
    report = gate.prior.validate_package_root(V13_FINAL)
    gate.install()
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV14Error("preserved v13 package semantic gate failed")
    manifest = json.loads((V13_FINAL / "transfer_manifest.json").read_bytes())
    safety = manifest.get("safety", {})
    if (
        safety.get("real_bag_read") is not True
        or safety.get("optional_bag_read") is not False
        or safety.get("gpu_exposed") is not False
        or safety.get("solver_executed") is not False
        or safety.get("network_used") is not False
        or safety.get("sudo_used") is not False
    ):
        raise ExecutionV14Error("preserved v13 package safety facts drifted")
    return {
        "partial_root_absent": True,
        "final_root": {
            "path": str(V13_FINAL),
            "inode": metadata.st_ino,
            "mode": "0700",
            "size_bytes": metadata.st_size,
            "nlink": metadata.st_nlink,
            "entry_count": 20,
            "inventory_sha256": V13_FINAL_INVENTORY_SHA256,
        },
        "empty_receipt_reservation": _fd_verify_empty(V13_RESERVATION),
        "final_receipt_absent": True,
        "cli_return_code": 2,
        "publication_error": V13_PUBLICATION_ERROR,
        "primary_bag_post_sha256": parent.PRIMARY_BAG_SHA256,
        "real_bag_read_attempted": True,
        "real_bag_read_confirmed": True,
        "bwrap_run": True,
        "package_entries": 20,
        "package_gate_status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
    }


def assert_fresh_v14() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV14Error(f"v14 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    parent_paths = {
        "policy": parent.POLICY,
        "policy_schema": parent.POLICY_SCHEMA,
        "receipt_schema": parent.RECEIPT_SCHEMA,
        "supervisor": Path(parent.__file__).resolve(),
        "tests": parent.TESTS,
    }
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V14",
        "parent_v13": {
            name: {"path": str(path), "sha256": file_identity(path)["sha256"]}
            for name, path in parent_paths.items()
        },
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v14.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v14",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V14",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v14",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V12_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV14Error("v14 receipt schema delta differs")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    schema = copy.deepcopy(parent.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v13", "v14_contract"]
    schema["properties"]["preserved_v13"] = {"$ref": "#/$defs/preservedV13"}
    schema["properties"]["v14_contract"] = {"$ref": "#/$defs/v14Contract"}
    policy_schema = json.loads(POLICY_SCHEMA.read_bytes())
    schema["$defs"]["finalDir"] = copy.deepcopy(
        policy_schema["$defs"]["finalDir"]
    )
    schema["$defs"]["reservation"] = copy.deepcopy(
        policy_schema["$defs"]["reservation"]
    )
    schema["$defs"]["preservedV13"] = copy.deepcopy(
        policy_schema["$defs"]["preservedV13"]
    )
    names = [
        "execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
        "parent_v13_policy", "parent_v13_policy_schema", "parent_v13_receipt_schema",
        "parent_v13_supervisor", "parent_v13_tests", "package_policy", "package_schema",
        "package_gate", "package_tests",
    ]
    schema["$defs"]["v14Contract"] = {
        "type": "object",
        "additionalProperties": False,
        "required": names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in names},
    }
    execution = schema["$defs"]["execution"]
    for name in ("failure_receipt_schema_v14", "v6_module_state_guarded"):
        if name not in execution["required"]:
            execution["required"].append(name)
        execution["properties"][name] = {"const": True}
    safety_schema = schema["$defs"]["safety"]
    for name in ("v13_evidence_preserved", "v6_module_state_guarded"):
        if name not in safety_schema["required"]:
            safety_schema["required"].append(name)
        safety_schema["properties"][name] = {"const": True}
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


def _apply_v14_paths() -> None:
    base.SCRIPT = SCRIPT
    base.POLICY = gate.POLICY_PATH
    base.PACKAGE_SCHEMA = gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA = v6.RECEIPT_SCHEMA
    base.GATE = Path(gate.__file__).resolve()
    base.EXTRACTOR = Path(parent.extractor.__file__).resolve()
    base.BRIDGE, base.READER, base.COMPAT_GATE = (
        parent.BRIDGE, parent.READER, parent.COMPAT_GATE
    )
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = (
        TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    )
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL
    base.S5A0_FINAL_RECEIPT = parent.S5A0_FINAL_RECEIPT
    base.S5A0_EVIDENCE_ROOT = parent.S5A0_EVIDENCE_ROOT
    base.S5A0_INNER_RECEIPT = parent.S5A0_INNER_RECEIPT
    base.gate, base.extractor = gate, parent.extractor
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = (
        PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID
    )


def _install_runtime_bindings(expected: dict[Path, str] | None = None):
    _apply_v14_paths()
    if expected is not None:
        base.EXPECTED = expected
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt
    base.subprocess = SUBPROCESS_PROXY
    runtime = parent.snapshot_runtime_bindings()
    parent.assert_runtime_bindings(runtime)
    v6_state = snapshot_v6_module_state()
    assert_v6_module_state(v6_state)
    return runtime, v6_state


def _restore_all(runtime: Mapping[str, Any], v6_state: Mapping[str, Any]) -> None:
    parent.restore_runtime_bindings(runtime)
    restore_v6_module_state(v6_state)


def configure_paths() -> None:
    parent.configure_paths()
    gate.install()
    _apply_v14_paths()


def local_import_closure(start: Path = SCRIPT):
    return parent.local_import_closure(start)


def assert_import_closure_bound(argv: Sequence[str]):
    closure, witness = local_import_closure()
    bound = parent._ro_bind_paths(argv)
    missing = sorted(str(path) for path in closure if path not in bound)
    required = {gate.POLICY_PATH, gate.SCHEMA_PATH}
    missing_data = sorted(str(path) for path in required if path not in bound)
    if missing or missing_data:
        raise ExecutionV14Error(
            "sandbox import/data closure is incomplete: " + ",".join(missing + missing_data)
        )
    return {"local_import_closure": witness, "missing": [], "data_missing": []}


def raw_build(
    receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int
) -> list[str]:
    outer_runtime = parent.snapshot_runtime_bindings()
    outer_v6 = snapshot_v6_module_state()
    try:
        configure_paths()
        runtime, v6_state = _install_runtime_bindings()
        try:
            argv = parent.raw_build(
                receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd
            )
        finally:
            _restore_all(runtime, v6_state)
        marker = argv.index("--")
        if argv[marker + 1:marker + 3] != ["/usr/bin/python3", str(parent.SCRIPT)]:
            raise ExecutionV14Error("parent v13 worker command identity differs")
        bound = parent._ro_bind_paths(argv)
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
        return argv
    finally:
        _restore_all(outer_runtime, outer_v6)


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102):
    return v6._canonical(
        raw_build(
            parent.S5A0_INNER_RECEIPT,
            PRIMARY_BAG,
            PARTIAL_ROOT,
            receipt_fd=receipt_fd,
            source_fd=source_fd,
        ),
        receipt_fd,
        source_fd,
    )


def _iter_file_refs(value: Any):
    yield from parent._iter_file_refs(value)


def recursive_expected_map(policy: Mapping[str, Any]):
    parent_policy = json.loads(parent.POLICY.read_bytes())
    expected, parent_witness = parent.recursive_expected_map(parent_policy)
    excluded = set(parent_witness["excluded_paths"])
    exact_zero = policy["preserved_v13"]["empty_receipt_reservation"]
    for item in _iter_file_refs(policy):
        path = Path(item["path"])
        if item["size_bytes"] == 0:
            if path != V13_RESERVATION:
                raise ExecutionV14Error(f"unexpected zero-byte v14 reference: {path}")
            excluded.add(str(path))
            continue
        if path in expected and expected[path] != item["sha256"]:
            raise ExecutionV14Error(f"conflicting recursive parent hash: {path}")
        expected[path] = item["sha256"]
    expected[POLICY] = file_identity(POLICY)["sha256"]
    if _fd_verify_empty(V13_RESERVATION) != exact_zero:
        raise ExecutionV14Error("v13 reservation witness differs")
    closure, closure_witness = local_import_closure()
    sandbox = policy["sandbox"]
    if (
        closure_witness["count"] != sandbox["local_python_import_closure_count"]
        or closure_witness["sha256"] != sandbox["local_python_import_closure_sha256"]
    ):
        raise ExecutionV14Error("local Python import closure differs")
    for path, digest in closure.items():
        if path in expected and expected[path] != digest:
            raise ExecutionV14Error(f"conflicting local import identity: {path}")
        expected[path] = digest
    exact_exclusions = {
        str(parent.v11.V8_RESERVATION),
        str(parent.v11.V10_RESERVATION),
        str(parent.v12.V11_RESERVATION),
        str(V13_RESERVATION),
    }
    if excluded != exact_exclusions:
        raise ExecutionV14Error("exact zero-byte exclusions differ")
    return expected, {
        "excluded_paths": sorted(excluded),
        "excluded_unique_count": 4,
        "parent_v13": parent_witness,
        "local_import_closure": closure_witness,
        "all_other_zero_byte_refs_fail_closed": True,
    }


def recursive_parent_v13_admission(expected_sha256: str):
    original_fresh = parent.assert_fresh_v13
    previous = (
        parent.ACTIVE_POLICY,
        parent.ACTIVE_POLICY_IDENTITY,
        parent.CAPTURED_TF_WITNESS,
    )
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()

    def preserved_instead() -> None:
        preserved_v13()

    try:
        parent.assert_fresh_v13 = preserved_instead
        return parent.admit_policy(expected_sha256)
    finally:
        parent.assert_fresh_v13 = original_fresh
        (
            parent.ACTIVE_POLICY,
            parent.ACTIVE_POLICY_IDENTITY,
            parent.CAPTURED_TF_WITNESS,
        ) = previous
        _restore_all(runtime, v6_state)


def admit_policy(expected_sha256: str | None = None):
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV14Error("host supplementary groups drifted")
    assert_fresh_v14()
    policy, identity = _load_policy(expected_sha256)
    sections = {
        "parent_v13": {
            "policy": parent.POLICY,
            "policy_schema": parent.POLICY_SCHEMA,
            "receipt_schema": parent.RECEIPT_SCHEMA,
            "supervisor": Path(parent.__file__).resolve(),
            "tests": parent.TESTS,
        },
        "package_v10": {
            "policy": gate.POLICY_PATH,
            "schema": gate.SCHEMA_PATH,
            "gate": Path(gate.__file__).resolve(),
            "tests": ROOT / "tests/test_s5a1_handoff_gate_v12.py",
        },
        "v14_contract": {
            "policy": POLICY_SCHEMA,
            "schema": RECEIPT_SCHEMA,
            "gate": SCRIPT,
            "tests": TESTS,
        },
    }
    for section, paths in sections.items():
        for name, path in paths.items():
            assert_identity(path, policy[section][name])
    if preserved_v13() != policy["preserved_v13"]:
        raise ExecutionV14Error("v13 evidence differs")
    if policy["tf_contract"] != json.loads(parent.POLICY.read_bytes())["tf_contract"]:
        raise ExecutionV14Error("TF contract drifted")
    if tuple(policy["sandbox"]["v6_module_state_fields"]) != V6_MODULE_STATE_FIELDS:
        raise ExecutionV14Error("v6 module-state fields differ")
    configure_paths()
    runtime, v6_state = _install_runtime_bindings()
    recursive_parent_v13_admission(policy["parent_v13"]["policy"]["sha256"])
    parent.assert_runtime_bindings(runtime)
    assert_v6_module_state(v6_state)
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    expected, recursive = recursive_expected_map(policy)
    configure_paths()
    runtime, v6_state = _install_runtime_bindings(expected)
    coverage = parent.assert_base_receipt_parent_coverage(expected)
    argv = canonical_argv()
    if (
        v6._argv_hash(argv) != policy["sandbox"]["canonical_argv_sha256"]
        or len(argv) != policy["sandbox"]["canonical_token_count"]
    ):
        raise ExecutionV14Error("canonical argv differs")
    import_report = assert_import_closure_bound(argv)
    parent.assert_runtime_bindings(runtime)
    v6_report = assert_v6_module_state(v6_state)
    return {
        "canonical_argv_sha256": v6._argv_hash(argv),
        "canonical_token_count": len(argv),
        "generic_expected_count": len(expected),
        "recursive_coverage": recursive,
        "base_coverage": coverage,
        "import_coverage": import_report,
        "recursive_parent_v13_admission": True,
        "runtime_binding_guard": parent.assert_runtime_bindings(runtime),
        "v6_module_state_guard": v6_report,
    }


def build_bwrap_argv(
    receipt: Path,
    source: Path,
    partial: Path,
    *,
    receipt_fd: int | None = None,
    source_fd: int | None = None,
):
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV14Error("admission and both fds are required")
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()
    try:
        argv = raw_build(
            receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd
        )
    finally:
        _restore_all(runtime, v6_state)
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV14Error("runtime argv differs")
    assert_import_closure_bound(argv)
    parent.assert_runtime_bindings(runtime)
    assert_v6_module_state(v6_state)
    return argv


def load_package(root: Path):
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()
    try:
        package = v6._BASE_LOAD_PACKAGE(root)
        report = gate.validate_package_root(root)
        if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
            raise ExecutionV14Error("strong semantic gate failed")
        v6._SEMANTIC_ROOTS.append(str(root))
        v6._SEMANTIC_REPORT = dict(report)
        return package
    finally:
        _restore_all(runtime, v6_state)


def _ref(value: Mapping[str, Any]):
    return {"path": value["path"], "sha256": value["sha256"]}


def v14_contract():
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV14Error("policy not admitted")
    p = ACTIVE_POLICY
    return {
        "execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
        "policy_schema": _ref(p["v14_contract"]["policy"]),
        "receipt_schema": _ref(p["v14_contract"]["schema"]),
        "supervisor": _ref(p["v14_contract"]["gate"]),
        "tests": _ref(p["v14_contract"]["tests"]),
        "parent_v13_policy": _ref(p["parent_v13"]["policy"]),
        "parent_v13_policy_schema": _ref(p["parent_v13"]["policy_schema"]),
        "parent_v13_receipt_schema": _ref(p["parent_v13"]["receipt_schema"]),
        "parent_v13_supervisor": _ref(p["parent_v13"]["supervisor"]),
        "parent_v13_tests": _ref(p["parent_v13"]["tests"]),
        "package_policy": _ref(p["package_v10"]["policy"]),
        "package_schema": _ref(p["package_v10"]["schema"]),
        "package_gate": _ref(p["package_v10"]["gate"]),
        "package_tests": _ref(p["package_v10"]["tests"]),
    }


def _parent_v13_transform(value: Mapping[str, Any]):
    if ACTIVE_POLICY is None:
        raise ExecutionV14Error("policy not admitted")
    parent_policy = json.loads(parent.POLICY.read_bytes())
    previous = (
        parent.ACTIVE_POLICY,
        parent.ACTIVE_POLICY_IDENTITY,
        parent.CAPTURED_TF_WITNESS,
    )
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()
    try:
        parent.ACTIVE_POLICY = parent_policy
        parent.ACTIVE_POLICY_IDENTITY = ACTIVE_POLICY["parent_v13"]["policy"]
        parent.CAPTURED_TF_WITNESS = copy.deepcopy(CAPTURED_TF_WITNESS)
        return parent.transform_receipt(value)
    except Exception as exc:
        raise ExecutionV14Error(f"parent v13 transform failed: {exc}") from exc
    finally:
        (
            parent.ACTIVE_POLICY,
            parent.ACTIVE_POLICY_IDENTITY,
            parent.CAPTURED_TF_WITNESS,
        ) = previous
        _restore_all(runtime, v6_state)


def transform_receipt(value: Mapping[str, Any]):
    parent.assert_raw_v1_origin(value)
    out = _parent_v13_transform(value)
    out.update(
        schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v14",
        document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V14",
        execution_id=EXECUTION_ID,
        receipt_id=f"{TRANSFER_ID}_execution_v14",
    )
    out["preserved_v13"] = preserved_v13()
    out["v14_contract"] = v14_contract()
    out["execution"].update(
        failure_receipt_schema_v14=True,
        v6_module_state_guarded=True,
    )
    out["safety"].update(
        v13_evidence_preserved=True,
        v6_module_state_guarded=True,
    )
    out["semantic_validation"]["gate"] = _ref(ACTIVE_POLICY["package_v10"]["gate"])
    if out["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"] = (
            "PASS_S5A1_HANDOFF_GATE_V12_STRONG_SEMANTICS"
        )
    return out


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL):
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o440,
    )
    os.fchmod(descriptor, 0o440)
    return descriptor


def publish_reserved_receipt(
    descriptor: int, partial_path: Path, final_path: Path, value: Mapping[str, Any]
) -> None:
    try:
        parent.assert_raw_v1_origin(value)
        transformed = transform_receipt(value)
        Draft202012Validator(receipt_schema()).validate(transformed)
        view = memoryview(gate.v1.canonical_json(transformed))
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExecutionV14Error("short v14 receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    base.rename_noreplace(partial_path, final_path)


def mock_failure_receipt():
    value = v6._BASE_MOCK_FAILURE()
    value["execution"]["supplementary_groups"] = HOST_GROUPS
    return transform_receipt(value)


def _sanitized_subprocess_run(command: Sequence[str], **kwargs: Any):
    global CAPTURED_TF_WITNESS
    if "env" in kwargs:
        raise ExecutionV14Error("unfrozen host environment")
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()
    try:
        completed = v6.v5._REAL_SUBPROCESS_RUN(
            command, env=dict(HOST_ENV), **kwargs
        )
    finally:
        _restore_all(runtime, v6_state)
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
            pass
    return completed


class _SubprocessProxy:
    DEVNULL, PIPE = v6._subprocess.DEVNULL, v6._subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY = _SubprocessProxy()


def install_runtime():
    if ACTIVE_POLICY is None:
        raise ExecutionV14Error("policy not admitted")
    configure_paths()
    parent9 = json.loads(parent.v9.POLICY.read_bytes())
    base_v6, identity = v6._load_policy(parent9["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY, v6._ACTIVE_POLICY_IDENTITY = copy.deepcopy(base_v6), identity
    expected, recursive = recursive_expected_map(ACTIVE_POLICY)
    coverage = parent.assert_base_receipt_parent_coverage(expected)
    runtime, v6_state = _install_runtime_bindings(expected)
    return {
        "recursive_coverage": recursive,
        "base_coverage": coverage,
        "runtime_binding_guard": parent.assert_runtime_bindings(runtime),
        "v6_module_state_guard": assert_v6_module_state(v6_state),
    }


def worker(receipt_path: Path, source: Path, partial: Path):
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != MAPPED_GROUPS:
        raise ExecutionV14Error("worker mapped supplementary groups differ")
    configure_paths()
    runtime, v6_state = _install_runtime_bindings()
    try:
        report = v6._BASE_WORKER(receipt_path, source, partial)
    finally:
        _restore_all(runtime, v6_state)
    report["worker_supplementary_groups"] = MAPPED_GROUPS
    return report


def static_self_check():
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS = None
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    report = admit_policy()
    runtime = install_runtime()
    Draft202012Validator(receipt_schema()).validate(mock_failure_receipt())
    gate_report = gate.self_check()
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V14_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "transfer_id": TRANSFER_ID,
        "policy_sha256": ACTIVE_POLICY_IDENTITY["sha256"],
        **report,
        "runtime_coverage": runtime,
        "package_gate_status": gate_report["status"],
        "receipt_schema_deep_closed": True,
        "strict_cli": True,
        "preserved_v13": preserved_v13(),
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


def execute_one_shot(receipt_path: Path, receipt_sha256: str, policy_sha256: str):
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS = None
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    v6._WORKER_GROUP_WITNESS = False
    admit_policy(policy_sha256)
    install_runtime()
    runtime = parent.snapshot_runtime_bindings()
    v6_state = snapshot_v6_module_state()
    try:
        v6._BASE_EXECUTE(receipt_path, receipt_sha256)
    finally:
        _restore_all(runtime, v6_state)
    value = json.loads(EXECUTION_RECEIPT.read_bytes())
    Draft202012Validator(receipt_schema()).validate(value)
    return value


def main(argv: Sequence[str] | None = None):
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
                raise ExecutionV14Error("exact v14 execution authorization differs")
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
                {"status": "FAIL_CLOSED", "execution_id": EXECUTION_ID, "error": str(exc)},
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
