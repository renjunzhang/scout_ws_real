#!/usr/bin/env python3
"""Fresh S5A1 v12 contract with import-closure and publication-hook guards."""

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
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator

MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5a1_execution_supervisor_v11 as v11
import r8_liquid_s5a1_handoff_gate_v10 as gate

sys.dont_write_bytecode = True
v10, v9, v6, base, extractor = v11.v10, v11.v9, v11.v6, v11.base, v11.extractor
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v12.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v12.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v12.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v12.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T234000Z_v12"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v12.json")
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
V11_PARTIAL, V11_FINAL = v11.PARTIAL_ROOT, v11.FINAL_ROOT
V11_RECEIPT, V11_RESERVATION = v11.EXECUTION_RECEIPT, v11.EXECUTION_RECEIPT_PARTIAL
EMPTY_SHA256 = v11.EMPTY_SHA256
HOST_GROUPS, MAPPED_GROUPS = list(v11.HOST_GROUPS), list(v11.MAPPED_GROUPS)
HOST_ENV = dict(v11.HOST_ENV)
PRIMARY_BAG = Path(
    "/home/zrj/slosh_bags/matrix_bags/"
    "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
)
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None


class ExecutionV12Error(ValueError):
    """The v12 execution contract failed closed."""


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
        raise ExecutionV12Error(f"frozen file identity drifted: {path}")
    return observed


def _fd_verify_empty(path: Path, *, inode: int, mode: int) -> dict[str, Any]:
    before = os.lstat(path)
    if path.is_symlink():
        raise ExecutionV12Error(f"specialized empty reservation is a symlink: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (
            before.st_ino != metadata.st_ino
            or not stat.S_ISREG(metadata.st_mode)
            or metadata.st_ino != inode
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != mode
            or metadata.st_size != 0
            or os.read(descriptor, 1) != b""
        ):
            raise ExecutionV12Error(f"specialized empty reservation drifted: {path}")
    finally:
        os.close(descriptor)
    return {"path": str(path), "sha256": EMPTY_SHA256, "mode": f"{mode:04o}", "size_bytes": 0}


def preserved_v11() -> dict[str, Any]:
    metadata = os.lstat(V11_PARTIAL)
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or V11_PARTIAL.is_symlink()
        or metadata.st_ino != 15748876
        or stat.S_IMODE(metadata.st_mode) != 0o700
        or metadata.st_nlink != 2
        or any(V11_PARTIAL.iterdir())
    ):
        raise ExecutionV12Error("preserved v11 partial root drifted")
    reservation = _fd_verify_empty(V11_RESERVATION, inode=14966666, mode=0o440)
    if os.path.lexists(V11_FINAL) or os.path.lexists(V11_RECEIPT):
        raise ExecutionV12Error("unexpected v11 final output exists")
    return {
        "partial_root": {
            "path": str(V11_PARTIAL),
            "inode": metadata.st_ino,
            "mode": "0700",
            "size_bytes": metadata.st_size,
            "nlink": metadata.st_nlink,
            "entry_count": 0,
            "inventory_sha256": EMPTY_SHA256,
        },
        "empty_receipt_reservation": reservation,
        "final_root_absent": True,
        "final_receipt_absent": True,
        "cli_return_code": 2,
        "worker_error": "ModuleNotFoundError: No module named 'r8_liquid_s5a1_execution_supervisor_v9'",
        "publication_error": "Additional properties are not allowed ('parents' was unexpected)",
        "primary_bag_post_sha256": "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07",
        "real_bag_read_attempted": "UNKNOWN",
        "real_bag_read_confirmed": "UNKNOWN",
        "bwrap_run": "UNKNOWN",
        "package_entries": 0,
    }


def assert_fresh_v12() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV12Error(f"v12 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    parent_paths = {
        "policy": v11.POLICY,
        "policy_schema": v11.POLICY_SCHEMA,
        "receipt_schema": v11.RECEIPT_SCHEMA,
        "supervisor": Path(v11.__file__).resolve(),
        "tests": v11.TESTS,
    }
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V12",
        "parent_v11": {
            name: {"path": str(path), "sha256": file_identity(path)["sha256"]}
            for name, path in parent_paths.items()
        },
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v12.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v12",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V12",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v12",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V10_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV12Error("v12 receipt schema delta differs or is not closed")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    schema = copy.deepcopy(v11.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v11", "v12_contract"]
    schema["properties"]["preserved_v11"] = {"$ref": "#/$defs/preservedV11"}
    schema["properties"]["v12_contract"] = {"$ref": "#/$defs/v12Contract"}
    unknown = {"const": "UNKNOWN"}
    schema["$defs"]["preservedV11"] = {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "partial_root", "empty_receipt_reservation", "final_root_absent",
            "final_receipt_absent", "cli_return_code", "worker_error",
            "publication_error", "primary_bag_post_sha256", "real_bag_read_attempted",
            "real_bag_read_confirmed", "bwrap_run", "package_entries",
        ],
        "properties": {
            "partial_root": {"$ref": "#/$defs/dirIdentity"},
            "empty_receipt_reservation": {"$ref": "#/$defs/file"},
            "final_root_absent": {"const": True},
            "final_receipt_absent": {"const": True},
            "cli_return_code": {"const": 2},
            "worker_error": {"const": "ModuleNotFoundError: No module named 'r8_liquid_s5a1_execution_supervisor_v9'"},
            "publication_error": {"const": "Additional properties are not allowed ('parents' was unexpected)"},
            "primary_bag_post_sha256": {"const": "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"},
            "real_bag_read_attempted": unknown,
            "real_bag_read_confirmed": unknown,
            "bwrap_run": unknown,
            "package_entries": {"const": 0},
        },
    }
    names = [
        "execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
        "parent_v11_policy", "parent_v11_policy_schema", "parent_v11_receipt_schema",
        "parent_v11_supervisor", "parent_v11_tests", "package_policy", "package_schema",
        "package_gate", "package_tests",
    ]
    schema["$defs"]["v12Contract"] = {
        "type": "object",
        "additionalProperties": False,
        "required": names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in names},
    }
    execution = schema["$defs"]["execution"]
    for name in ("failure_receipt_schema_v12", "publication_hooks_guarded", "raw_v1_origin_validated"):
        if name not in execution["required"]:
            execution["required"].append(name)
        execution["properties"][name] = {"const": True}
    safety = schema["$defs"]["safety"]
    for name in ("v11_evidence_preserved", "import_closure_complete", "publication_hooks_guarded"):
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
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value = json.loads(raw)
    schema = json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    return value, {
        "path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
        "mode": full["mode"], "size_bytes": full["size_bytes"],
    }


def _apply_v12_paths() -> None:
    base.SCRIPT, base.POLICY, base.PACKAGE_SCHEMA = SCRIPT, gate.POLICY_PATH, gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA, base.GATE, base.gate = v6.RECEIPT_SCHEMA, Path(gate.__file__).resolve(), gate
    base.EXTRACTOR, base.extractor = Path(extractor.__file__).resolve(), extractor
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    base.EXECUTION_RECEIPT, base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID


def configure_paths() -> None:
    v11.configure_paths()
    gate.install()
    _apply_v12_paths()


def publication_hooks() -> tuple[Any, Any, Any]:
    return (base.reserve_receipt, base.publish_reserved_receipt, base.mock_failure_receipt)


def _install_publication_hooks() -> None:
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt


def assert_publication_hooks() -> None:
    if publication_hooks() != (reserve_receipt, publish_reserved_receipt, mock_failure_receipt):
        raise ExecutionV12Error("v12 publication hooks were clobbered")


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
            raise ExecutionV12Error(f"unresolved or unsafe local import: {name}")
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
    raw = b"".join(f"{path}\0{identities[path]}\n".encode() for path in sorted(identities, key=str))
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
        raise ExecutionV12Error(
            "sandbox import/data closure is incomplete: " + ",".join(missing + missing_data)
        )
    return {"local_import_closure": witness, "missing": [], "data_missing": []}


def raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int):
    configure_paths()
    _install_publication_hooks()
    assert_publication_hooks()
    try:
        argv = v11.raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    finally:
        _apply_v12_paths()
        _install_publication_hooks()
        assert_publication_hooks()
    marker = argv.index("--")
    if argv[marker + 1:marker + 3] != ["/usr/bin/python3", str(v11.SCRIPT)]:
        raise ExecutionV12Error("parent worker command identity differs")
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
    assert_publication_hooks()
    return argv


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102):
    return v6._canonical(
        raw_build(v6.v5.prior.S5A0_INNER_RECEIPT, PRIMARY_BAG, PARTIAL_ROOT,
                  receipt_fd=receipt_fd, source_fd=source_fd),
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
    parent = json.loads(v11.POLICY.read_bytes())
    expected, parent_witness = v11.recursive_expected_map(parent)
    exact_zero = policy["preserved_v11"]["empty_receipt_reservation"]
    excluded = set(parent_witness["excluded_paths"])
    for item in _iter_file_refs(policy):
        path = Path(item["path"])
        if item["size_bytes"] == 0:
            if path != V11_RESERVATION or dict(item) != dict(exact_zero):
                raise ExecutionV12Error(f"unexpected zero-byte file reference: {path}")
            excluded.add(str(path))
            continue
        if item["size_bytes"] < 0:
            raise ExecutionV12Error(f"invalid negative file size: {path}")
        if path in expected and expected[path] != item["sha256"]:
            raise ExecutionV12Error(f"conflicting recursive parent hash: {path}")
        expected[path] = item["sha256"]
    policy_identity = file_identity(POLICY)
    expected[POLICY] = policy_identity["sha256"]
    exact_exclusions = {
        str(v11.V8_RESERVATION), str(v11.V10_RESERVATION), str(V11_RESERVATION),
    }
    if excluded != exact_exclusions:
        raise ExecutionV12Error("exact v8/v10/v11 reservations must be the only exclusions")
    if preserved_v11()["empty_receipt_reservation"] != exact_zero:
        raise ExecutionV12Error("v11 specialized witness differs from exclusion")
    closure, closure_witness = local_import_closure()
    sandbox = policy["sandbox"]
    if (
        closure_witness["count"] != sandbox["local_python_import_closure_count"]
        or closure_witness["sha256"] != sandbox["local_python_import_closure_sha256"]
    ):
        raise ExecutionV12Error("local Python import closure differs")
    for path, digest in closure.items():
        if path in expected and expected[path] != digest:
            raise ExecutionV12Error(f"conflicting local import identity: {path}")
        expected[path] = digest
    return expected, {
        "excluded_paths": sorted(excluded),
        "excluded_unique_count": 3,
        "parent_v11": parent_witness,
        "local_import_closure": closure_witness,
        "all_other_zero_byte_refs_fail_closed": True,
    }


def required_base_receipt_parent_paths() -> tuple[Path, ...]:
    return tuple(dict.fromkeys((
        base.SCRIPT, base.POLICY, base.PACKAGE_SCHEMA, base.RECEIPT_SCHEMA,
        base.GATE, base.EXTRACTOR, base.BRIDGE, base.READER, base.S5A0_SCHEMA,
        base.S5A0_FINAL_SCHEMA, base.S5A0_SUPERVISOR_POLICY, base.COMPAT_GATE,
    )))


def assert_base_receipt_parent_coverage(expected: Mapping[Path, str]) -> dict[str, Any]:
    required = required_base_receipt_parent_paths()
    missing = [str(path) for path in required if path not in expected]
    if missing:
        raise ExecutionV12Error("base receipt parent coverage missing: " + ",".join(missing))
    return {"required_count": len(required), "missing": [], "status": "PASS_BASE_RECEIPT_PARENT_COVERAGE"}


def recursive_parent_v11_admission(expected_sha256: str) -> dict[str, Any]:
    original = v11.assert_fresh_v11

    def preserved_instead_of_fresh() -> None:
        preserved_v11()

    configure_paths()
    _install_publication_hooks()
    try:
        v11.assert_fresh_v11 = preserved_instead_of_fresh
        return v11.admit_policy(expected_sha256)
    finally:
        v11.assert_fresh_v11 = original
        _apply_v12_paths()
        _install_publication_hooks()
        assert_publication_hooks()


def admit_policy(expected_sha256: str | None = None):
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV12Error("host supplementary groups drifted")
    assert_fresh_v12()
    policy, identity = _load_policy(expected_sha256)
    sections = {
        "parent_v11": {
            "policy": v11.POLICY, "policy_schema": v11.POLICY_SCHEMA,
            "receipt_schema": v11.RECEIPT_SCHEMA, "supervisor": Path(v11.__file__).resolve(),
            "tests": v11.TESTS,
        },
        "package_v8": {
            "policy": gate.POLICY_PATH, "schema": gate.SCHEMA_PATH,
            "gate": Path(gate.__file__).resolve(), "tests": ROOT / "tests/test_s5a1_handoff_gate_v10.py",
        },
        "v12_contract": {
            "policy": POLICY_SCHEMA, "schema": RECEIPT_SCHEMA,
            "gate": SCRIPT, "tests": TESTS,
        },
    }
    for section, paths in sections.items():
        for name, path in paths.items():
            assert_identity(path, policy[section][name])
    if preserved_v11() != policy["preserved_v11"]:
        raise ExecutionV12Error("v11 evidence differs")
    if policy["tf_contract"] != json.loads(v11.POLICY.read_bytes())["tf_contract"]:
        raise ExecutionV12Error("TF contract drifted")
    configure_paths()
    _install_publication_hooks()
    assert_publication_hooks()
    recursive_parent_v11_admission(policy["parent_v11"]["policy"]["sha256"])
    assert_publication_hooks()
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    expected, recursive = recursive_expected_map(policy)
    configure_paths()
    _install_publication_hooks()
    coverage = assert_base_receipt_parent_coverage(expected)
    closure, closure_witness = local_import_closure()
    for path, digest in closure.items():
        if expected.get(path) != digest:
            raise ExecutionV12Error(f"local import lacks frozen recursive identity: {path}")
    sandbox = policy["sandbox"]
    if (
        closure_witness["count"] != sandbox["local_python_import_closure_count"]
        or closure_witness["sha256"] != sandbox["local_python_import_closure_sha256"]
    ):
        raise ExecutionV12Error("local Python import closure differs")
    argv = canonical_argv()
    import_report = assert_import_closure_bound(argv)
    if v6._argv_hash(argv) != sandbox["canonical_argv_sha256"] or len(argv) != sandbox["canonical_token_count"]:
        raise ExecutionV12Error("canonical argv differs")
    marker = argv.index("--")
    if argv[marker + 2] != str(SCRIPT) or argv.count("--bind") != 1:
        raise ExecutionV12Error("worker path or writable bind differs")
    assert_publication_hooks()
    return {
        "canonical_argv_sha256": v6._argv_hash(argv),
        "canonical_token_count": len(argv),
        "generic_expected_count": len(expected),
        "recursive_coverage": recursive,
        "base_coverage": coverage,
        "import_coverage": import_report,
        "recursive_parent_v11_admission": True,
        "publication_hooks_guarded": True,
    }


def build_bwrap_argv(receipt: Path, source: Path, partial: Path, *, receipt_fd=None, source_fd=None):
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV12Error("admission and both fds are required")
    assert_publication_hooks()
    argv = raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    assert_publication_hooks()
    assert_import_closure_bound(argv)
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV12Error("runtime argv differs")
    return argv


def load_package(root: Path):
    package = v6._BASE_LOAD_PACKAGE(root)
    report = gate.validate_package_root(root)
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV12Error("strong semantic gate failed")
    v6._SEMANTIC_ROOTS.append(str(root))
    v6._SEMANTIC_REPORT = dict(report)
    return package


def _ref(value: Mapping[str, Any]):
    return {"path": value["path"], "sha256": value["sha256"]}


def v12_contract():
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV12Error("policy not admitted")
    policy = ACTIVE_POLICY
    return {
        "execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
        "policy_schema": _ref(policy["v12_contract"]["policy"]),
        "receipt_schema": _ref(policy["v12_contract"]["schema"]),
        "supervisor": _ref(policy["v12_contract"]["gate"]),
        "tests": _ref(policy["v12_contract"]["tests"]),
        "parent_v11_policy": _ref(policy["parent_v11"]["policy"]),
        "parent_v11_policy_schema": _ref(policy["parent_v11"]["policy_schema"]),
        "parent_v11_receipt_schema": _ref(policy["parent_v11"]["receipt_schema"]),
        "parent_v11_supervisor": _ref(policy["parent_v11"]["supervisor"]),
        "parent_v11_tests": _ref(policy["parent_v11"]["tests"]),
        "package_policy": _ref(policy["package_v8"]["policy"]),
        "package_schema": _ref(policy["package_v8"]["schema"]),
        "package_gate": _ref(policy["package_v8"]["gate"]),
        "package_tests": _ref(policy["package_v8"]["tests"]),
    }


RAW_V1_KEYS = {
    "schema_version", "document_type", "receipt_id", "status", "parents", "execution",
    "inputs", "roots", "package", "compatibility", "safety", "failure", "next",
}


def assert_raw_v1_origin(value: Mapping[str, Any]) -> None:
    if not isinstance(value, Mapping) or set(value) != RAW_V1_KEYS:
        raise ExecutionV12Error("receipt transform input is not the exact raw v1 shape")
    if (
        value.get("schema_version") != "smpcc-r8-liquid-s5a1-execution-receipt-v1"
        or value.get("document_type") != "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V1"
        or "contract" in value
        or "semantic_validation" in value
        or "execution_id" in value
        or not isinstance(value.get("parents"), Mapping)
        or value.get("execution", {}).get("supplementary_groups") != HOST_GROUPS
    ):
        raise ExecutionV12Error("receipt transform input is not the exact raw v1 origin")


def _parent_v11_transform(value: Mapping[str, Any]):
    parent = json.loads(v11.POLICY.read_bytes())
    previous = (v11.ACTIVE_POLICY, v11.ACTIVE_POLICY_IDENTITY, v11.CAPTURED_TF_WITNESS)
    try:
        v11.ACTIVE_POLICY = parent
        v11.ACTIVE_POLICY_IDENTITY = ACTIVE_POLICY["parent_v11"]["policy"]
        v11.CAPTURED_TF_WITNESS = copy.deepcopy(CAPTURED_TF_WITNESS)
        try:
            return v11.transform_receipt(value)
        except Exception as exc:
            raise ExecutionV12Error(f"parent v11 transform failed: {exc}") from exc
    finally:
        v11.ACTIVE_POLICY, v11.ACTIVE_POLICY_IDENTITY, v11.CAPTURED_TF_WITNESS = previous


def transform_receipt(value: Mapping[str, Any]):
    assert_raw_v1_origin(value)
    out = _parent_v11_transform(value)
    out.update(
        schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v12",
        document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V12",
        execution_id=EXECUTION_ID,
        receipt_id=f"{TRANSFER_ID}_execution_v12",
    )
    out["preserved_v11"] = preserved_v11()
    out["v12_contract"] = v12_contract()
    out["execution"].update(
        failure_receipt_schema_v12=True,
        publication_hooks_guarded=True,
        raw_v1_origin_validated=True,
    )
    current_attempted = out["safety"]["real_bag_read_attempted"]
    current_confirmed = out["safety"]["real_bag_read_confirmed"]
    if not isinstance(current_attempted, bool) or not isinstance(current_confirmed, bool):
        raise ExecutionV12Error("current v12 real-bag read evidence is not boolean")
    if current_confirmed and not current_attempted:
        raise ExecutionV12Error("current v12 confirmed read without attempted read")
    out["safety"].update(
        v11_evidence_preserved=True,
        import_closure_complete=True,
        publication_hooks_guarded=True,
        bwrap_run=current_attempted,
    )
    if out["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and not (
        current_attempted and current_confirmed and out["safety"]["bwrap_run"]
    ):
        raise ExecutionV12Error("v12 success lacks current real-bag/bwrap evidence")
    if out["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"] = "PASS_S5A1_HANDOFF_GATE_V10_STRONG_SEMANTICS"
    return out


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL):
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o440
    )
    os.fchmod(descriptor, 0o440)
    return descriptor


def publish_reserved_receipt(
    descriptor: int, partial_path: Path, final_path: Path, value: Mapping[str, Any]
) -> None:
    assert_publication_hooks()
    try:
        assert_raw_v1_origin(value)
        transformed = transform_receipt(value)
        Draft202012Validator(receipt_schema()).validate(transformed)
        view = memoryview(gate.v1.canonical_json(transformed))
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExecutionV12Error("short v12 receipt write")
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
        raise ExecutionV12Error("unfrozen host environment")
    completed = v6.v5._REAL_SUBPROCESS_RUN(command, env=dict(HOST_ENV), **kwargs)
    v6._WORKER_GROUP_WITNESS = False
    CAPTURED_TF_WITNESS = None
    if completed.returncode == 0:
        try:
            report = json.loads(completed.stdout)
            v6._WORKER_GROUP_WITNESS = report.get("worker_supplementary_groups") == MAPPED_GROUPS
            witness = report.get("tf_qc")
            CAPTURED_TF_WITNESS = witness if isinstance(witness, dict) else None
        except Exception:
            CAPTURED_TF_WITNESS = None
    return completed


class _SubprocessProxy:
    DEVNULL, PIPE = v6._subprocess.DEVNULL, v6._subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY = _SubprocessProxy()


def install_runtime():
    if ACTIVE_POLICY is None:
        raise ExecutionV12Error("policy not admitted")
    configure_paths()
    parent9 = json.loads(v9.POLICY.read_bytes())
    base_v6, base_v6_identity = v6._load_policy(parent9["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY, v6._ACTIVE_POLICY_IDENTITY = copy.deepcopy(base_v6), base_v6_identity
    expected, recursive = recursive_expected_map(ACTIVE_POLICY)
    coverage = assert_base_receipt_parent_coverage(expected)
    base.EXPECTED = expected
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    _install_publication_hooks()
    base.subprocess = SUBPROCESS_PROXY
    assert_publication_hooks()
    return {"recursive_coverage": recursive, "base_coverage": coverage}


def worker(receipt_path: Path, source: Path, partial: Path):
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != MAPPED_GROUPS:
        raise ExecutionV12Error("worker mapped supplementary groups differ")
    configure_paths()
    report = v6._BASE_WORKER(receipt_path, source, partial)
    report["worker_supplementary_groups"] = MAPPED_GROUPS
    return report


def static_self_check():
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS = None
    v6._SEMANTIC_ROOTS, v6._SEMANTIC_REPORT = [], None
    report = admit_policy()
    runtime = install_runtime()
    schema = receipt_schema()
    Draft202012Validator(schema).validate(mock_failure_receipt())
    gate_report = gate.self_check()
    assert_publication_hooks()
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V12_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "transfer_id": TRANSFER_ID,
        "policy_sha256": ACTIVE_POLICY_IDENTITY["sha256"],
        **report,
        "runtime_coverage": runtime,
        "package_gate_status": gate_report["status"],
        "receipt_schema_deep_closed": True,
        "strict_cli": True,
        "preserved_v11": preserved_v11(),
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
    v6._BASE_EXECUTE(receipt_path, receipt_sha256)
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
                raise ExecutionV12Error("exact v12 execution authorization differs")
            result = execute_one_shot(
                args.s5a0_receipt, args.s5a0_receipt_sha256, args.execution_policy_sha256
            )
        else:
            result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as exc:
        print(
            json.dumps({"status": "FAIL_CLOSED", "execution_id": EXECUTION_ID, "error": str(exc)}, sort_keys=True),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
