#!/usr/bin/env python3
"""Fresh S5A1 v9 one-shot contract with relative TF and direct publication."""

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

import r8_liquid_s5a1_execution_supervisor_v8 as v8
import r8_liquid_s5a1_handoff_gate_v7 as gate
import r8_liquid_s5a1_ros1_signal_extractor_v5 as extractor


sys.dont_write_bytecode = True
v6, base = v8.v6, v8.base
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v9.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v9.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v9.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v9.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T223334Z_v9"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v9.json")
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
V8_PARTIAL = Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v4.partial")
V8_FINAL = Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v4")
V8_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v4.execution_v8.json")
V8_RESERVATION = Path(str(V8_RECEIPT) + ".partial")
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()
HOST_GROUPS, MAPPED_GROUPS = list(v6.HOST_GROUPS), list(v6.MAPPED_GROUPS)
HOST_ENV = dict(v6.HOST_ENV)
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None
PARENT_PUBLISHER_USED = False


class ExecutionV9Error(ValueError):
    """The v9 execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    raw, full = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "mode": full["mode"], "size_bytes": full["size_bytes"]}


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV9Error(f"frozen file identity drifted: {path}")
    return observed


def preserved_v8() -> dict[str, Any]:
    root = os.lstat(V8_PARTIAL)
    if (not stat.S_ISDIR(root.st_mode) or V8_PARTIAL.is_symlink()
            or root.st_ino != 15748711 or stat.S_IMODE(root.st_mode) != 0o700
            or root.st_nlink != 2 or any(V8_PARTIAL.iterdir())):
        raise ExecutionV9Error("preserved v8 partial root drifted")
    reservation_metadata = os.lstat(V8_RESERVATION)
    if V8_RESERVATION.is_symlink():
        raise ExecutionV9Error("preserved v8 reservation is a symlink")
    reservation_raw = V8_RESERVATION.read_bytes()
    reservation = {"path": str(V8_RESERVATION),
                   "sha256": hashlib.sha256(reservation_raw).hexdigest(),
                   "mode": f"{stat.S_IMODE(reservation_metadata.st_mode):04o}",
                   "size_bytes": reservation_metadata.st_size}
    if (not stat.S_ISREG(reservation_metadata.st_mode) or reservation_metadata.st_nlink != 1
            or reservation_metadata.st_ino != 14966631 or reservation["mode"] != "0440"
            or reservation["size_bytes"] != 0 or reservation["sha256"] != EMPTY_SHA256):
        raise ExecutionV9Error("preserved v8 empty receipt reservation drifted")
    if os.path.lexists(V8_FINAL) or os.path.lexists(V8_RECEIPT):
        raise ExecutionV9Error("unexpected v8 final output exists")
    return {
        "partial_root": {"path": str(V8_PARTIAL), "inode": root.st_ino, "mode": "0700",
                         "size_bytes": root.st_size, "nlink": root.st_nlink,
                         "entry_count": 0, "inventory_sha256": EMPTY_SHA256},
        "empty_receipt_reservation": reservation,
        "final_root_absent": True, "final_receipt_absent": True,
    }


def assert_fresh_v9() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV9Error(f"v9 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V9",
        "parent_v8_receipt_delta": {"path": str(v8.RECEIPT_SCHEMA),
            "sha256": "7eb80dab78e18454d623afedc76e2aafc49c5aecf5e8f4210e3b12e45f907135"},
        "parent_v8_supervisor": {"path": str(Path(v8.__file__).resolve()),
            "sha256": "429c45cde074ca64ff0e9abd14690b0a622210b393fc85cf11d154b909b8b0ab"},
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v9.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v9",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V9",
        "execution_id": EXECUTION_ID, "receipt_id": f"{TRANSFER_ID}_execution_v9",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V7_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV9Error("v9 receipt schema delta differs or is not closed")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta()
    schema = copy.deepcopy(v8.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v8_failure", "tf_alignment_witness", "v9_contract"]
    schema["properties"]["preserved_v8_failure"] = {"$ref": "#/$defs/preservedV8"}
    schema["properties"]["tf_alignment_witness"] = {
        "anyOf": [{"$ref": "#/$defs/tfWitness"}, {"type": "null"}]}
    schema["properties"]["v9_contract"] = {"$ref": "#/$defs/v9Contract"}
    schema["$defs"]["preservedV8"] = {
        "type": "object", "additionalProperties": False,
        "required": ["partial_root", "empty_receipt_reservation", "final_root_absent", "final_receipt_absent"],
        "properties": {"partial_root": {"$ref": "#/$defs/dirIdentity"},
                       "empty_receipt_reservation": {"$ref": "#/$defs/file"},
                       "final_root_absent": {"const": True},
                       "final_receipt_absent": {"const": True}},
    }
    threshold = {"type": "object", "additionalProperties": False,
                 "required": ["maximum_time_delta_s", "maximum_position_delta_m", "maximum_angular_delta_rad"],
                 "properties": {"maximum_time_delta_s": {"const": 0.5},
                                "maximum_position_delta_m": {"const": 0.05},
                                "maximum_angular_delta_rad": {"const": 0.1}}}
    schema["$defs"]["tfWitness"] = {
        "type": "object", "additionalProperties": False,
        "required": ["status", "odom_frame", "base_frame", "container_mount_source",
                     "tf_used_as_motion", "comparison_frame", "sample_count",
                     "maximum_time_delta_s", "absolute_maximum_position_delta_m",
                     "absolute_maximum_angular_delta_rad", "relative_maximum_translation_delta_m",
                     "relative_rms_translation_delta_m", "relative_maximum_angular_delta_rad",
                     "frozen_thresholds"],
        "properties": {
            "status": {"const": "PASS_ALIGNMENT_WITNESS"}, "odom_frame": {"const": "odom"},
            "base_frame": {"const": "base_footprint"},
            "container_mount_source": {"const": "FROZEN_COMPATIBILITY_CONTRACT"},
            "tf_used_as_motion": {"const": False},
            "comparison_frame": {"const": "T0_INVERSE_TIMES_T_RELATIVE_SE3"},
            "sample_count": {"type": "integer", "minimum": 1},
            "maximum_time_delta_s": {"type": "number", "minimum": 0, "maximum": 0.5},
            "absolute_maximum_position_delta_m": {"type": "number", "minimum": 0},
            "absolute_maximum_angular_delta_rad": {"type": "number", "minimum": 0},
            "relative_maximum_translation_delta_m": {"type": "number", "minimum": 0, "maximum": 0.05},
            "relative_rms_translation_delta_m": {"type": "number", "minimum": 0, "maximum": 0.05},
            "relative_maximum_angular_delta_rad": {"type": "number", "minimum": 0, "maximum": 0.1},
            "frozen_thresholds": threshold,
        },
    }
    contract_names = ["execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
                      "parent_v8_policy", "parent_v8_policy_schema", "parent_v8_receipt_schema",
                      "parent_v8_supervisor", "parent_v8_tests", "base_v6_policy", "extractor_v5",
                      "extractor_v5_tests", "package_policy", "package_schema", "package_gate", "package_tests"]
    schema["$defs"]["v9Contract"] = {
        "type": "object", "additionalProperties": False, "required": contract_names,
        "properties": {name: {"$ref": "#/$defs/identity"} for name in contract_names},
    }
    execution = schema["$defs"]["execution"]
    execution["required"].append("failure_receipt_schema_v9")
    execution["properties"]["failure_receipt_schema_v9"] = {"const": True}
    safety = schema["$defs"]["safety"]
    safety["required"] += ["v8_evidence_preserved", "parent_publisher_used"]
    safety["properties"]["v8_evidence_preserved"] = {"const": True}
    safety["properties"]["parent_publisher_used"] = {"const": False}
    schema["$defs"]["semantic"]["properties"]["status"] = {
        "enum": [delta["semantic_success_status"], "NOT_RUN_DUE_TO_FAILURE"]}
    schema["allOf"][0]["then"]["properties"]["semantic_validation"]["properties"]["status"] = {
        "const": delta["semantic_success_status"]}
    schema["allOf"].append({
        "if": {"properties": {"status": {"const": "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"}},
               "required": ["status"]},
        "then": {"properties": {"tf_alignment_witness": {"$ref": "#/$defs/tfWitness"}}},
    })
    Draft202012Validator.check_schema(schema)
    gate.v1.assert_schema_deep_closed(schema)
    return schema


def _load_policy(expected_sha256: str | None = None):
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value, schema = json.loads(raw), json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema); gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    return value, {"path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
                   "mode": full["mode"], "size_bytes": full["size_bytes"]}


def configure_paths() -> None:
    v8.configure_paths(); gate.install()
    base.SCRIPT, base.POLICY, base.PACKAGE_SCHEMA = SCRIPT, gate.POLICY_PATH, gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA, base.GATE, base.gate = v6.RECEIPT_SCHEMA, Path(gate.__file__).resolve(), gate
    base.EXTRACTOR, base.extractor = Path(extractor.__file__).resolve(), extractor
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    base.EXECUTION_RECEIPT, base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID


def raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int):
    argv = v8.raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    bind, additions = argv.index("--bind"), []
    for path in (Path(v8.__file__).resolve(), Path(extractor.base.__file__).resolve()):
        if str(path) not in argv:
            additions += ["--ro-bind", str(path), str(path)]
    argv[bind:bind] = additions
    return argv


def canonical_argv(receipt_fd: int = 101, source_fd: int = 102):
    configure_paths()
    return v6._canonical(raw_build(v6.v5.prior.S5A0_INNER_RECEIPT,
        Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
        PARTIAL_ROOT, receipt_fd=receipt_fd, source_fd=source_fd), receipt_fd, source_fd)


def _iter_file_refs(value: Any):
    if isinstance(value, dict):
        if {"path", "sha256", "mode", "size_bytes"}.issubset(value): yield value
        for child in value.values(): yield from _iter_file_refs(child)
    elif isinstance(value, list):
        for child in value: yield from _iter_file_refs(child)


def admit_policy(expected_sha256: str | None = None):
    global ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS: raise ExecutionV9Error("host groups drifted")
    assert_fresh_v9(); policy, identity = _load_policy(expected_sha256)
    sections = {
        "parent_v8": {"policy": v8.POLICY, "policy_schema": v8.POLICY_SCHEMA,
                      "receipt_schema": v8.RECEIPT_SCHEMA, "supervisor": Path(v8.__file__).resolve(),
                      "tests": v8.TESTS},
        "package_v5": {"policy": gate.POLICY_PATH, "schema": gate.SCHEMA_PATH,
                       "gate": Path(gate.__file__).resolve(), "tests": ROOT/"tests/test_s5a1_handoff_gate_v7.py"},
        "v9_contract": {"policy": POLICY_SCHEMA, "schema": RECEIPT_SCHEMA,
                        "gate": SCRIPT, "tests": TESTS},
    }
    for section, paths in sections.items():
        for name, path in paths.items(): assert_identity(path, policy[section][name])
    assert_identity(v6.POLICY, policy["base_v6_policy"])
    assert_identity(Path(extractor.__file__).resolve(), policy["extractor_v5"])
    assert_identity(ROOT/"tests/test_s5a1_ros1_signal_extractor_v5.py", policy["extractor_v5_tests"])
    base_v6, base_v6_identity = v6._load_policy(policy["base_v6_policy"]["sha256"])
    if base_v6_identity != policy["base_v6_policy"]: raise ExecutionV9Error("base v6 identity differs")
    for section in ("package_revision", "v6_contract"):
        for parent in base_v6[section].values(): assert_identity(Path(parent["path"]), parent)
    assert_identity(Path(base_v6["base_v5"]["path"]), base_v6["base_v5"])
    if preserved_v8() != policy["preserved_v8"]: raise ExecutionV9Error("v8 evidence differs")
    if policy["tf_contract"]["real_diagnostic_witness"] != extractor.REAL_DIAGNOSTIC_WITNESS:
        raise ExecutionV9Error("TF diagnostic witness differs")
    ACTIVE_POLICY, ACTIVE_POLICY_IDENTITY = policy, identity
    argv = canonical_argv(); sandbox = policy["sandbox"]
    if v6._argv_hash(argv) != sandbox["canonical_argv_sha256"] or len(argv) != sandbox["canonical_token_count"]:
        raise ExecutionV9Error("canonical argv differs")
    if str(SCRIPT) not in argv or argv.count("--bind") != 1: raise ExecutionV9Error("worker path/bind differs")
    return {"canonical_argv_sha256": v6._argv_hash(argv), "canonical_token_count": len(argv)}


def build_bwrap_argv(receipt: Path, source: Path, partial: Path, *, receipt_fd=None, source_fd=None):
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None: raise ExecutionV9Error("admission/fds required")
    argv = raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    canonical = v6._canonical(argv, receipt_fd, source_fd)
    if v6._argv_hash(canonical) != ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV9Error("runtime argv differs")
    return argv


def load_package(root: Path):
    package = v6._BASE_LOAD_PACKAGE(root); report = gate.validate_package_root(root)
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV9Error("strong semantic gate failed")
    v6._SEMANTIC_ROOTS.append(str(root)); v6._SEMANTIC_REPORT = dict(report)
    return package


def _ref(value: Mapping[str, Any]): return {"path": value["path"], "sha256": value["sha256"]}


def _parent_v8_contract():
    parent = ACTIVE_POLICY["parent_v8"]
    parent_policy = json.loads(v8.POLICY.read_bytes())
    previous_policy, previous_identity = v8.ACTIVE_POLICY, v8.ACTIVE_POLICY_IDENTITY
    try:
        v8.ACTIVE_POLICY, v8.ACTIVE_POLICY_IDENTITY = parent_policy, parent["policy"]
        return v8.v8_contract()
    finally:
        v8.ACTIVE_POLICY, v8.ACTIVE_POLICY_IDENTITY = previous_policy, previous_identity


def v9_contract():
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None: raise ExecutionV9Error("policy not admitted")
    p = ACTIVE_POLICY
    return {"execution_policy": _ref(ACTIVE_POLICY_IDENTITY),
            "policy_schema": _ref(p["v9_contract"]["policy"]),
            "receipt_schema": _ref(p["v9_contract"]["schema"]),
            "supervisor": _ref(p["v9_contract"]["gate"]), "tests": _ref(p["v9_contract"]["tests"]),
            "parent_v8_policy": _ref(p["parent_v8"]["policy"]),
            "parent_v8_policy_schema": _ref(p["parent_v8"]["policy_schema"]),
            "parent_v8_receipt_schema": _ref(p["parent_v8"]["receipt_schema"]),
            "parent_v8_supervisor": _ref(p["parent_v8"]["supervisor"]),
            "parent_v8_tests": _ref(p["parent_v8"]["tests"]),
            "base_v6_policy": _ref(p["base_v6_policy"]), "extractor_v5": _ref(p["extractor_v5"]),
            "extractor_v5_tests": _ref(p["extractor_v5_tests"]),
            "package_policy": _ref(p["package_v5"]["policy"]),
            "package_schema": _ref(p["package_v5"]["schema"]),
            "package_gate": _ref(p["package_v5"]["gate"]), "package_tests": _ref(p["package_v5"]["tests"])}


def transform_receipt(value: Mapping[str, Any]):
    value = copy.deepcopy(value); value["execution"]["supplementary_groups"] = HOST_GROUPS
    out = v6.transform_receipt(value)
    out.update(schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v9",
               document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V9",
               execution_id=EXECUTION_ID, receipt_id=f"{TRANSFER_ID}_execution_v9")
    out["preserved_v6_failure"] = v8.preserved_v6(); out["v8_contract"] = _parent_v8_contract()
    out["preserved_v8_failure"] = preserved_v8(); out["v9_contract"] = v9_contract()
    out["tf_alignment_witness"] = copy.deepcopy(CAPTURED_TF_WITNESS)
    out["execution"].update(failure_receipt_schema_v8=True, failure_receipt_schema_v9=True)
    out["safety"].update(v6_evidence_preserved=True, v8_evidence_preserved=True,
                         parent_publisher_used=False)
    if out["semantic_validation"]["status"] != "NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"] = "PASS_S5A1_HANDOFF_GATE_V7_STRONG_SEMANTICS"
    if out["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and CAPTURED_TF_WITNESS is None:
        raise ExecutionV9Error("success lacks complete TF witness")
    return out


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL):
    descriptor = os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC, 0o440)
    os.fchmod(descriptor, 0o440); return descriptor


def publish_reserved_receipt(descriptor: int, partial_path: Path, final_path: Path,
                             value: Mapping[str, Any]) -> None:
    if base.publish_reserved_receipt is not publish_reserved_receipt:
        raise ExecutionV9Error("v9 publisher hook was clobbered")
    try:
        transformed = transform_receipt(value)
        Draft202012Validator(receipt_schema()).validate(transformed)
        view = memoryview(gate.v1.canonical_json(transformed))
        while view:
            written = os.write(descriptor, view)
            if written <= 0: raise ExecutionV9Error("short v9 receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    base.rename_noreplace(partial_path, final_path)


def mock_failure_receipt(): return transform_receipt(v6._BASE_MOCK_FAILURE())


def _sanitized_subprocess_run(command: Sequence[str], **kwargs: Any):
    global CAPTURED_TF_WITNESS
    if "env" in kwargs: raise ExecutionV9Error("unfrozen host environment")
    completed = v6.v5._REAL_SUBPROCESS_RUN(command, env=dict(HOST_ENV), **kwargs)
    v6._WORKER_GROUP_WITNESS = False; CAPTURED_TF_WITNESS = None
    if completed.returncode == 0:
        try:
            report = json.loads(completed.stdout)
            v6._WORKER_GROUP_WITNESS = report.get("worker_supplementary_groups") == MAPPED_GROUPS
            witness = report.get("tf_qc")
            if isinstance(witness, dict): CAPTURED_TF_WITNESS = witness
        except Exception:
            CAPTURED_TF_WITNESS = None
    return completed


class _SubprocessProxy:
    DEVNULL, PIPE = v6._subprocess.DEVNULL, v6._subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY = _SubprocessProxy()


def install_runtime() -> None:
    if ACTIVE_POLICY is None: raise ExecutionV9Error("policy not admitted")
    configure_paths(); parent_v8 = json.loads(v8.POLICY.read_bytes())
    base_v6, base_v6_identity = v6._load_policy(parent_v8["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY, v6._ACTIVE_POLICY_IDENTITY = copy.deepcopy(base_v6), base_v6_identity
    base_v5, _identity = v6.v5._load_policy(base_v6["base_v5"]["sha256"])
    expected = {Path(item["path"]): item["sha256"] for item in base_v5["contracts"].values()}
    for item in _iter_file_refs(ACTIVE_POLICY): expected[Path(item["path"])] = item["sha256"]
    base.EXPECTED = expected; base.build_bwrap_argv = build_bwrap_argv; base.load_package = load_package
    base.reserve_receipt = reserve_receipt; base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt; base.subprocess = SUBPROCESS_PROXY


def worker(receipt_path: Path, source: Path, partial: Path):
    if os.getuid()!=1000 or os.getgid()!=1000 or os.getgroups()!=MAPPED_GROUPS:
        raise ExecutionV9Error("worker mapped groups differ")
    configure_paths(); report=v6._BASE_WORKER(receipt_path,source,partial)
    report["worker_supplementary_groups"]=MAPPED_GROUPS; return report


def static_self_check():
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None; v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None
    report=admit_policy(); install_runtime(); schema=receipt_schema()
    Draft202012Validator(schema).validate(mock_failure_receipt()); extractor_report=extractor.self_check()
    return {"status":"PASS_S5A1_EXECUTION_SUPERVISOR_V9_STATIC_CONTRACT",
            "execution_id":EXECUTION_ID,"transfer_id":TRANSFER_ID,
            "policy_sha256":ACTIVE_POLICY_IDENTITY["sha256"],**report,
            "extractor_status":extractor_report["status"],"receipt_schema_deep_closed":True,
            "direct_publisher":True,"parent_publisher_used":False,"strict_cli":True,
            "real_bag_read":False,"bwrap_run":False,"files_written":False,"solver_executed":False,"sudo_used":False}


def execute_one_shot(receipt_path: Path, receipt_sha256: str, policy_sha256: str):
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None; v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None; v6._WORKER_GROUP_WITNESS=False
    admit_policy(policy_sha256); install_runtime(); v6._BASE_EXECUTE(receipt_path,receipt_sha256)
    value=json.loads(EXECUTION_RECEIPT.read_bytes()); Draft202012Validator(receipt_schema()).validate(value); return value


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
            if args.authorize_execution_id!=EXECUTION_ID: raise ExecutionV9Error("exact v9 authorization differs")
            result=execute_one_shot(args.s5a0_receipt,args.s5a0_receipt_sha256,args.execution_policy_sha256)
        else: result=worker(args.s5a0_receipt,args.source,args.partial_root)
    except Exception as exc:
        print(json.dumps({"status":"FAIL_CLOSED","execution_id":EXECUTION_ID,"error":str(exc)},sort_keys=True),file=sys.stderr);return 2
    print(json.dumps(result,allow_nan=False,sort_keys=True,separators=(",",":")));return 0


if __name__=="__main__": raise SystemExit(main())
