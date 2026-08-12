#!/usr/bin/env python3
"""Fresh S5A1 v10 one-shot contract with exact zero-byte reservation handling."""

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

import r8_liquid_s5a1_execution_supervisor_v9 as v9
import r8_liquid_s5a1_handoff_gate_v8 as gate


sys.dont_write_bytecode = True
v6, base, extractor = v9.v6, v9.base, v9.extractor
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v10.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v10.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v10.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v10.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T225436Z_v10"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v10.json")
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
V8_RESERVATION = v9.V8_RESERVATION
HOST_GROUPS, MAPPED_GROUPS = list(v9.HOST_GROUPS), list(v9.MAPPED_GROUPS)
HOST_ENV = dict(v9.HOST_ENV)
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None


class ExecutionV10Error(ValueError):
    """The v10 execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    raw, full = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "mode": full["mode"], "size_bytes": full["size_bytes"]}


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV10Error(f"frozen file identity drifted: {path}")
    return observed


def preserved_v8() -> dict[str, Any]:
    """Delegate semantic identity to v9, then repeat fd/O_NOFOLLOW verification."""
    evidence = v9.preserved_v8()
    descriptor = os.open(V8_RESERVATION, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
                or metadata.st_ino != 14966631 or stat.S_IMODE(metadata.st_mode) != 0o440
                or metadata.st_size != 0 or os.read(descriptor, 1) != b""):
            raise ExecutionV10Error("fd-verified v8 reservation identity drifted")
    finally:
        os.close(descriptor)
    if evidence["empty_receipt_reservation"] != {
        "path": str(V8_RESERVATION), "sha256": v9.EMPTY_SHA256,
        "mode": "0440", "size_bytes": 0,
    }:
        raise ExecutionV10Error("v9 specialized v8 reservation witness drifted")
    return evidence


def _v9_output_paths() -> tuple[Path, ...]:
    return (v9.PARTIAL_ROOT, v9.FINAL_ROOT, v9.EXECUTION_RECEIPT, v9.EXECUTION_RECEIPT_PARTIAL)


def preserved_v9_preflight() -> dict[str, Any]:
    if any(os.path.lexists(path) for path in _v9_output_paths()):
        raise ExecutionV10Error("preserved v9 output absence drifted")
    return {
        "execution_id": v9.EXECUTION_ID,
        "status": "FAIL_CLOSED",
        "return_code": 2,
        "error": f"unsafe input identity: {V8_RESERVATION}",
        "outputs_absent": {"partial_root": True, "final_root": True,
                           "partial_receipt": True, "final_receipt": True},
        "real_bag_read": False,
        "files_written": False,
    }


def assert_fresh_v10() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV10Error(f"v10 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    parent_paths = {
        "policy": v9.POLICY, "policy_schema": v9.POLICY_SCHEMA,
        "receipt_schema": v9.RECEIPT_SCHEMA, "supervisor": Path(v9.__file__).resolve(),
        "tests": v9.TESTS,
    }
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V10",
        "parent_v9": {name: {"path": str(path), "sha256": file_identity(path)["sha256"]}
                      for name, path in parent_paths.items()},
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v10.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v10",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V10",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v10",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V8_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV10Error("v10 receipt schema delta differs or is not closed")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    schema = copy.deepcopy(v9.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v9_preflight", "v10_contract"]
    schema["properties"]["preserved_v9_preflight"] = {"$ref": "#/$defs/preservedV9"}
    schema["properties"]["v10_contract"] = {"$ref": "#/$defs/v10Contract"}
    schema["$defs"]["outputsAbsent"] = {
        "type": "object", "additionalProperties": False,
        "required": ["partial_root", "final_root", "partial_receipt", "final_receipt"],
        "properties": {name: {"const": True} for name in
                       ("partial_root", "final_root", "partial_receipt", "final_receipt")},
    }
    schema["$defs"]["preservedV9"] = {
        "type": "object", "additionalProperties": False,
        "required": ["execution_id", "status", "return_code", "error", "outputs_absent",
                     "real_bag_read", "files_written"],
        "properties": {
            "execution_id": {"const": v9.EXECUTION_ID}, "status": {"const": "FAIL_CLOSED"},
            "return_code": {"const": 2},
            "error": {"const": f"unsafe input identity: {V8_RESERVATION}"},
            "outputs_absent": {"$ref": "#/$defs/outputsAbsent"},
            "real_bag_read": {"const": False}, "files_written": {"const": False},
        },
    }
    names = ["execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
             "parent_v9_policy", "parent_v9_policy_schema", "parent_v9_receipt_schema",
             "parent_v9_supervisor", "parent_v9_tests", "package_policy", "package_schema",
             "package_gate", "package_tests"]
    schema["$defs"]["v10Contract"] = {
        "type": "object", "additionalProperties": False, "required": names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in names},
    }
    execution = schema["$defs"]["execution"]
    execution["required"].append("failure_receipt_schema_v10")
    execution["properties"]["failure_receipt_schema_v10"] = {"const": True}
    safety = schema["$defs"]["safety"]
    safety["required"] += ["v9_preflight_preserved", "zero_byte_parent_exclusion_exact"]
    safety["properties"]["v9_preflight_preserved"] = {"const": True}
    safety["properties"]["zero_byte_parent_exclusion_exact"] = {"const": True}
    schema["$defs"]["semantic"]["properties"]["status"] = {
        "enum": [delta["semantic_success_status"], "NOT_RUN_DUE_TO_FAILURE"]}
    schema["allOf"][0]["then"]["properties"]["semantic_validation"]["properties"]["status"] = {
        "const": delta["semantic_success_status"]}
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    return schema


def _load_policy(expected_sha256: str | None = None):
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value, schema = json.loads(raw), json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    return value, {"path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
                   "mode": full["mode"], "size_bytes": full["size_bytes"]}


def configure_paths() -> None:
    v9.configure_paths(); gate.install()
    base.SCRIPT, base.POLICY, base.PACKAGE_SCHEMA = SCRIPT, gate.POLICY_PATH, gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA, base.GATE, base.gate = v6.RECEIPT_SCHEMA, Path(gate.__file__).resolve(), gate
    base.EXTRACTOR, base.extractor = Path(extractor.__file__).resolve(), extractor
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    base.EXECUTION_RECEIPT, base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID


def raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int):
    configure_paths()
    argv = v9.raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    bind, additions = argv.index("--bind"), []
    for path in (Path(v9.__file__).resolve(), Path(gate.prior.__file__).resolve()):
        if str(path) not in argv:
            additions += ["--ro-bind", str(path), str(path)]
    argv[bind:bind] = additions
    return argv


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102):
    return v6._canonical(raw_build(v6.v5.prior.S5A0_INNER_RECEIPT,
        Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
        PARTIAL_ROOT, receipt_fd=receipt_fd, source_fd=source_fd), receipt_fd, source_fd)


def _iter_file_refs(value: Any):
    if isinstance(value, dict):
        if {"path", "sha256", "mode", "size_bytes"}.issubset(value):
            yield value
        for child in value.values():
            yield from _iter_file_refs(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_file_refs(child)


def expected_nonempty_parent_map(policy: Mapping[str, Any]) -> tuple[dict[Path, str], dict[str, Any]]:
    """Build EXPECTED while excluding exactly one specially verified empty reservation."""
    parent_v9 = json.loads(v9.POLICY.read_bytes())
    base_v6, _identity = v6._load_policy(parent_v9["base_v6_policy"]["sha256"])
    base_v5, _identity = v6.v5._load_policy(base_v6["base_v5"]["sha256"])
    expected = {Path(item["path"]): item["sha256"] for item in base_v5["contracts"].values()}
    exact_ref = policy["preserved_v8"]["empty_receipt_reservation"]
    excluded: list[dict[str, Any]] = []
    for item in _iter_file_refs(policy):
        path = Path(item["path"])
        if item["size_bytes"] == 0:
            if path != V8_RESERVATION or dict(item) != dict(exact_ref):
                raise ExecutionV10Error(f"unexpected zero-byte file reference: {path}")
            excluded.append(dict(item))
            continue
        expected[path] = item["sha256"]
    if len(excluded) != 1:
        raise ExecutionV10Error("exactly one v8 zero-byte reservation must be excluded")
    witness = preserved_v8()
    if witness["empty_receipt_reservation"] != excluded[0]:
        raise ExecutionV10Error("specialized v8 witness differs from excluded reference")
    return expected, {"excluded_path": str(V8_RESERVATION), "excluded_count": 1,
                      "specialized_validator": "v9.preserved_v8+lstat+O_NOFOLLOW+fstat",
                      "all_other_zero_byte_file_refs_fail_closed": True}


def admit_policy(expected_sha256: str | None = None):
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV10Error("host groups drifted")
    assert_fresh_v10()
    policy, identity = _load_policy(expected_sha256)
    sections = {
        "parent_v9": {"policy": v9.POLICY, "policy_schema": v9.POLICY_SCHEMA,
                      "receipt_schema": v9.RECEIPT_SCHEMA, "supervisor": Path(v9.__file__).resolve(),
                      "tests": v9.TESTS},
        "package_v6": {"policy": gate.POLICY_PATH, "schema": gate.SCHEMA_PATH,
                       "gate": Path(gate.__file__).resolve(), "tests": ROOT/"tests/test_s5a1_handoff_gate_v8.py"},
        "v10_contract": {"policy": POLICY_SCHEMA, "schema": RECEIPT_SCHEMA,
                         "gate": SCRIPT, "tests": TESTS},
    }
    for section, paths in sections.items():
        for name, path in paths.items():
            assert_identity(path, policy[section][name])
    parent_report = v9.admit_policy(policy["parent_v9"]["policy"]["sha256"])
    if v9.ACTIVE_POLICY_IDENTITY != policy["parent_v9"]["policy"]:
        raise ExecutionV10Error("recursive v9 admission identity differs")
    if preserved_v8() != policy["preserved_v8"]:
        raise ExecutionV10Error("v8 evidence differs")
    if preserved_v9_preflight() != policy["preserved_v9_preflight"]:
        raise ExecutionV10Error("v9 preflight evidence differs")
    if policy["tf_contract"] != json.loads(v9.POLICY.read_bytes())["tf_contract"]:
        raise ExecutionV10Error("TF contract drifted")
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    expected, exclusion = expected_nonempty_parent_map(policy)
    argv, sandbox = canonical_argv(), policy["sandbox"]
    if v6._argv_hash(argv) != sandbox["canonical_argv_sha256"] or len(argv) != sandbox["canonical_token_count"]:
        raise ExecutionV10Error("canonical argv differs")
    if str(SCRIPT) not in argv or argv.count("--bind") != 1:
        raise ExecutionV10Error("worker path/bind differs")
    return {"canonical_argv_sha256": v6._argv_hash(argv), "canonical_token_count": len(argv),
            "generic_expected_count": len(expected),
            "parent_v9_recursive_admission": True,
            "parent_v9_canonical_argv_sha256": parent_report["canonical_argv_sha256"],
            **exclusion}


def build_bwrap_argv(receipt: Path, source: Path, partial: Path, *, receipt_fd=None, source_fd=None):
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV10Error("admission/fds required")
    argv = raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV10Error("runtime argv differs")
    return argv


def load_package(root: Path):
    package = v6._BASE_LOAD_PACKAGE(root)
    report = gate.validate_package_root(root)
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV10Error("strong semantic gate failed")
    v6._SEMANTIC_ROOTS.append(str(root)); v6._SEMANTIC_REPORT = dict(report)
    return package


def _ref(value: Mapping[str, Any]):
    return {"path": value["path"], "sha256": value["sha256"]}


def v10_contract():
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV10Error("policy not admitted")
    p = ACTIVE_POLICY
    return {
        "execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
        "policy_schema": _ref(p["v10_contract"]["policy"]),
        "receipt_schema": _ref(p["v10_contract"]["schema"]),
        "supervisor": _ref(p["v10_contract"]["gate"]), "tests": _ref(p["v10_contract"]["tests"]),
        "parent_v9_policy": _ref(p["parent_v9"]["policy"]),
        "parent_v9_policy_schema": _ref(p["parent_v9"]["policy_schema"]),
        "parent_v9_receipt_schema": _ref(p["parent_v9"]["receipt_schema"]),
        "parent_v9_supervisor": _ref(p["parent_v9"]["supervisor"]),
        "parent_v9_tests": _ref(p["parent_v9"]["tests"]),
        "package_policy": _ref(p["package_v6"]["policy"]),
        "package_schema": _ref(p["package_v6"]["schema"]),
        "package_gate": _ref(p["package_v6"]["gate"]),
        "package_tests": _ref(p["package_v6"]["tests"]),
    }


def _parent_v9_transform(value: Mapping[str, Any]):
    parent_policy = json.loads(v9.POLICY.read_bytes())
    previous = (v9.ACTIVE_POLICY, v9.ACTIVE_POLICY_IDENTITY, v9.CAPTURED_TF_WITNESS)
    try:
        v9.ACTIVE_POLICY = parent_policy
        v9.ACTIVE_POLICY_IDENTITY = ACTIVE_POLICY["parent_v9"]["policy"]
        v9.CAPTURED_TF_WITNESS = copy.deepcopy(CAPTURED_TF_WITNESS)
        return v9.transform_receipt(value)
    finally:
        v9.ACTIVE_POLICY, v9.ACTIVE_POLICY_IDENTITY, v9.CAPTURED_TF_WITNESS = previous


def transform_receipt(value: Mapping[str, Any]):
    out = _parent_v9_transform(value)
    out.update(schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v10",
               document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V10",
               execution_id=EXECUTION_ID, receipt_id=f"{TRANSFER_ID}_execution_v10")
    out["preserved_v9_preflight"] = preserved_v9_preflight()
    out["v10_contract"] = v10_contract()
    out["execution"]["failure_receipt_schema_v10"] = True
    out["safety"].update(v9_preflight_preserved=True, zero_byte_parent_exclusion_exact=True)
    if out["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"] = "PASS_S5A1_HANDOFF_GATE_V8_STRONG_SEMANTICS"
    return out


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL):
    descriptor = os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC, 0o440)
    os.fchmod(descriptor, 0o440)
    return descriptor


def publish_reserved_receipt(descriptor: int, partial_path: Path, final_path: Path,
                             value: Mapping[str, Any]) -> None:
    if base.publish_reserved_receipt is not publish_reserved_receipt:
        raise ExecutionV10Error("v10 publisher hook was clobbered")
    try:
        transformed = transform_receipt(value)
        Draft202012Validator(receipt_schema()).validate(transformed)
        view = memoryview(gate.v1.canonical_json(transformed))
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExecutionV10Error("short v10 receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    base.rename_noreplace(partial_path, final_path)


def mock_failure_receipt():
    return transform_receipt(v6._BASE_MOCK_FAILURE())


def _sanitized_subprocess_run(command: Sequence[str], **kwargs: Any):
    global CAPTURED_TF_WITNESS
    if "env" in kwargs:
        raise ExecutionV10Error("unfrozen host environment")
    completed = v6.v5._REAL_SUBPROCESS_RUN(command, env=dict(HOST_ENV), **kwargs)
    v6._WORKER_GROUP_WITNESS = False; CAPTURED_TF_WITNESS = None
    if completed.returncode == 0:
        try:
            report = json.loads(completed.stdout)
            v6._WORKER_GROUP_WITNESS = report.get("worker_supplementary_groups") == MAPPED_GROUPS
            witness = report.get("tf_qc")
            if isinstance(witness, dict):
                CAPTURED_TF_WITNESS = witness
        except Exception:
            CAPTURED_TF_WITNESS = None
    return completed


class _SubprocessProxy:
    DEVNULL, PIPE = v6._subprocess.DEVNULL, v6._subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY = _SubprocessProxy()


def install_runtime() -> dict[str, Any]:
    if ACTIVE_POLICY is None:
        raise ExecutionV10Error("policy not admitted")
    configure_paths()
    parent_v9 = json.loads(v9.POLICY.read_bytes())
    base_v6, base_v6_identity = v6._load_policy(parent_v9["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY, v6._ACTIVE_POLICY_IDENTITY = copy.deepcopy(base_v6), base_v6_identity
    expected, exclusion = expected_nonempty_parent_map(ACTIVE_POLICY)
    base.EXPECTED = expected
    base.build_bwrap_argv = build_bwrap_argv; base.load_package = load_package
    base.reserve_receipt = reserve_receipt; base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt; base.subprocess = SUBPROCESS_PROXY
    return exclusion


def worker(receipt_path: Path, source: Path, partial: Path):
    if os.getuid()!=1000 or os.getgid()!=1000 or os.getgroups()!=MAPPED_GROUPS:
        raise ExecutionV10Error("worker mapped groups differ")
    configure_paths()
    report = v6._BASE_WORKER(receipt_path, source, partial)
    report["worker_supplementary_groups"] = MAPPED_GROUPS
    return report


def static_self_check():
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None; v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None
    report=admit_policy(); exclusion=install_runtime(); schema=receipt_schema()
    Draft202012Validator(schema).validate(mock_failure_receipt()); gate_report=gate.self_check()
    return {"status":"PASS_S5A1_EXECUTION_SUPERVISOR_V10_STATIC_CONTRACT",
            "execution_id":EXECUTION_ID,"transfer_id":TRANSFER_ID,
            "policy_sha256":ACTIVE_POLICY_IDENTITY["sha256"],**report,
            "package_gate_status":gate_report["status"],
            "receipt_schema_deep_closed":True,"direct_publisher":True,
            "parent_publisher_used":False,"strict_cli":True,
            "zero_byte_exclusion":exclusion,"preserved_v9_preflight":preserved_v9_preflight(),
            "real_bag_read":False,"bwrap_run":False,"files_written":False,
            "solver_executed":False,"sudo_used":False}


def execute_one_shot(receipt_path: Path, receipt_sha256: str, policy_sha256: str):
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None; v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None; v6._WORKER_GROUP_WITNESS=False
    admit_policy(policy_sha256); install_runtime(); v6._BASE_EXECUTE(receipt_path,receipt_sha256)
    value=json.loads(EXECUTION_RECEIPT.read_bytes())
    Draft202012Validator(receipt_schema()).validate(value)
    return value


def main(argv: Sequence[str] | None=None):
    parser=argparse.ArgumentParser(description=__doc__); subs=parser.add_subparsers(dest="command",required=True)
    subs.add_parser("self-check"); run=subs.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt",type=Path,required=True);run.add_argument("--s5a0-receipt-sha256",required=True)
    run.add_argument("--execution-policy-sha256",required=True);run.add_argument("--authorize-execution-id",required=True)
    worker_parser=subs.add_parser("_worker");worker_parser.add_argument("--s5a0-receipt",type=Path,required=True)
    worker_parser.add_argument("--source",type=Path,required=True);worker_parser.add_argument("--partial-root",type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        if args.command=="self-check": result=static_self_check()
        elif args.command=="run-one-shot":
            if args.authorize_execution_id!=EXECUTION_ID:
                raise ExecutionV10Error("exact v10 authorization differs")
            result=execute_one_shot(args.s5a0_receipt,args.s5a0_receipt_sha256,args.execution_policy_sha256)
        else: result=worker(args.s5a0_receipt,args.source,args.partial_root)
    except Exception as exc:
        print(json.dumps({"status":"FAIL_CLOSED","execution_id":EXECUTION_ID,"error":str(exc)},sort_keys=True),file=sys.stderr);return 2
    print(json.dumps(result,allow_nan=False,sort_keys=True,separators=(",",":")));return 0


if __name__=="__main__": raise SystemExit(main())
