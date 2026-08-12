#!/usr/bin/env python3
"""Fresh S5A1 v11 contract with recursive parent identity coverage."""

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
import r8_liquid_s5a1_execution_supervisor_v10 as v10
import r8_liquid_s5a1_handoff_gate_v9 as gate

sys.dont_write_bytecode = True
v9, v6, base, extractor = v10.v9, v10.v6, v10.base, v10.extractor
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v11.py"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v11.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v11.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v11.json"
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T230930Z_v11"
TRANSFER_ID = gate.TRANSFER_ID
PARTIAL_ROOT, FINAL_ROOT = Path(gate.PARTIAL_ROOT), Path(gate.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v11.json")
EXECUTION_RECEIPT_PARTIAL = Path(str(EXECUTION_RECEIPT) + ".partial")
V8_RESERVATION = v10.V8_RESERVATION
V10_PARTIAL, V10_FINAL = v10.PARTIAL_ROOT, v10.FINAL_ROOT
V10_RECEIPT, V10_RESERVATION = v10.EXECUTION_RECEIPT, v10.EXECUTION_RECEIPT_PARTIAL
EMPTY_SHA256 = v9.EMPTY_SHA256
HOST_GROUPS, MAPPED_GROUPS = list(v10.HOST_GROUPS), list(v10.MAPPED_GROUPS)
HOST_ENV = dict(v10.HOST_ENV)
ACTIVE_POLICY: dict[str, Any] | None = None
ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
CAPTURED_TF_WITNESS: dict[str, Any] | None = None


class ExecutionV11Error(ValueError):
    """The v11 execution contract failed closed."""


def file_identity(path: Path) -> dict[str, Any]:
    raw, full = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "mode": full["mode"], "size_bytes": full["size_bytes"]}


def assert_identity(path: Path, expected: Mapping[str, Any]) -> dict[str, Any]:
    observed = file_identity(path)
    if observed != dict(expected):
        raise ExecutionV11Error(f"frozen file identity drifted: {path}")
    return observed


def _fd_verify_empty(path: Path, *, inode: int, mode: int) -> dict[str, Any]:
    before = os.lstat(path)
    if path.is_symlink():
        raise ExecutionV11Error(f"specialized empty reservation is a symlink: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (before.st_ino != metadata.st_ino or not stat.S_ISREG(metadata.st_mode)
                or metadata.st_ino != inode or metadata.st_nlink != 1
                or stat.S_IMODE(metadata.st_mode) != mode or metadata.st_size != 0
                or os.read(descriptor, 1) != b""):
            raise ExecutionV11Error(f"specialized empty reservation drifted: {path}")
    finally:
        os.close(descriptor)
    return {"path": str(path), "sha256": EMPTY_SHA256,
            "mode": f"{mode:04o}", "size_bytes": 0}


def preserved_v8_reservation() -> dict[str, Any]:
    evidence = v10.preserved_v8()
    observed = _fd_verify_empty(V8_RESERVATION, inode=14966631, mode=0o440)
    if evidence["empty_receipt_reservation"] != observed:
        raise ExecutionV11Error("recursive v8 specialized witness differs")
    return observed


def preserved_v10() -> dict[str, Any]:
    root = os.lstat(V10_PARTIAL)
    if (not stat.S_ISDIR(root.st_mode) or V10_PARTIAL.is_symlink()
            or root.st_ino != 15748816 or stat.S_IMODE(root.st_mode) != 0o700
            or root.st_nlink != 2 or any(V10_PARTIAL.iterdir())):
        raise ExecutionV11Error("preserved v10 partial root drifted")
    reservation = _fd_verify_empty(V10_RESERVATION, inode=14966656, mode=0o440)
    if os.path.lexists(V10_FINAL) or os.path.lexists(V10_RECEIPT):
        raise ExecutionV11Error("unexpected v10 final output exists")
    return {
        "partial_root": {"path": str(V10_PARTIAL), "inode": root.st_ino, "mode": "0700",
                         "size_bytes": root.st_size, "nlink": root.st_nlink,
                         "entry_count": 0, "inventory_sha256": EMPTY_SHA256},
        "empty_receipt_reservation": reservation,
        "final_root_absent": True, "final_receipt_absent": True,
        "cli_return_code": 2,
        "surface_error": "'/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_s5a1_ros1_signal_extractor_v5.py'",
        "primary_bag_post_sha256": "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07",
        "real_bag_read_attempted": "UNKNOWN", "real_bag_read_confirmed": "UNKNOWN",
        "bwrap_run": "UNKNOWN", "package_entries": 0,
    }


def assert_fresh_v11() -> None:
    for path in (PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL):
        if os.path.lexists(path):
            raise ExecutionV11Error(f"v11 path is not fresh: {path}")


def _receipt_delta() -> dict[str, Any]:
    value = json.loads(RECEIPT_SCHEMA.read_bytes())
    paths = {"policy": v10.POLICY, "policy_schema": v10.POLICY_SCHEMA,
             "receipt_schema": v10.RECEIPT_SCHEMA, "supervisor": Path(v10.__file__).resolve(),
             "tests": v10.TESTS}
    expected = {
        "revision_kind": "DETERMINISTIC_CLOSED_RECEIPT_SCHEMA_DELTA_V11",
        "parent_v10": {name: {"path": str(path), "sha256": file_identity(path)["sha256"]}
                       for name, path in paths.items()},
        "schema_id": "https://scout.local/schema/target_host_s5a1_execution_receipt_v11.json",
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v11",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V11",
        "execution_id": EXECUTION_ID, "receipt_id": f"{TRANSFER_ID}_execution_v11",
        "semantic_success_status": "PASS_S5A1_HANDOFF_GATE_V9_STRONG_SEMANTICS",
    }
    if value != expected:
        raise ExecutionV11Error("v11 receipt schema delta differs or is not closed")
    return value


def receipt_schema() -> dict[str, Any]:
    delta = _receipt_delta(); schema = copy.deepcopy(v10.receipt_schema())
    schema["$id"] = delta["schema_id"]
    for key in ("schema_version", "document_type", "execution_id", "receipt_id"):
        schema["properties"][key] = {"const": delta[key]}
    schema["required"] += ["preserved_v10", "v11_contract"]
    schema["properties"]["preserved_v10"] = {"$ref": "#/$defs/preservedV10"}
    schema["properties"]["v11_contract"] = {"$ref": "#/$defs/v11Contract"}
    unknown = {"const": "UNKNOWN"}
    schema["$defs"]["preservedV10"] = {
        "type": "object", "additionalProperties": False,
        "required": ["partial_root", "empty_receipt_reservation", "final_root_absent",
                     "final_receipt_absent", "cli_return_code", "surface_error",
                     "primary_bag_post_sha256", "real_bag_read_attempted",
                     "real_bag_read_confirmed", "bwrap_run", "package_entries"],
        "properties": {
            "partial_root": {"$ref": "#/$defs/dirIdentity"},
            "empty_receipt_reservation": {"$ref": "#/$defs/file"},
            "final_root_absent": {"const": True}, "final_receipt_absent": {"const": True},
            "cli_return_code": {"const": 2},
            "surface_error": {"const": "'/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_s5a1_ros1_signal_extractor_v5.py'"},
            "primary_bag_post_sha256": {"const": "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"},
            "real_bag_read_attempted": unknown, "real_bag_read_confirmed": unknown,
            "bwrap_run": unknown, "package_entries": {"const": 0},
        },
    }
    names = ["execution_policy", "policy_schema", "receipt_schema", "supervisor", "tests",
             "parent_v10_policy", "parent_v10_policy_schema", "parent_v10_receipt_schema",
             "parent_v10_supervisor", "parent_v10_tests", "package_policy", "package_schema",
             "package_gate", "package_tests"]
    schema["$defs"]["v11Contract"] = {"type": "object", "additionalProperties": False,
        "required": names, "properties": {name: {"$ref": "#/$defs/identity"} for name in names}}
    execution = schema["$defs"]["execution"]
    execution["required"].append("failure_receipt_schema_v11")
    execution["properties"]["failure_receipt_schema_v11"] = {"const": True}
    safety = schema["$defs"]["safety"]
    for name in ("v10_evidence_preserved", "recursive_parent_coverage", "bwrap_run"):
        if name not in safety["required"]:
            safety["required"].append(name)
    safety["properties"].update(v10_evidence_preserved={"const": True},
        recursive_parent_coverage={"const": True}, bwrap_run={"type": "boolean"})
    schema["$defs"]["semantic"]["properties"]["status"] = {
        "enum": [delta["semantic_success_status"], "NOT_RUN_DUE_TO_FAILURE"]}
    schema["allOf"][0]["then"]["properties"]["semantic_validation"]["properties"]["status"] = {
        "const": delta["semantic_success_status"]}
    Draft202012Validator.check_schema(schema); gate.v1.assert_schema_deep_closed(schema)
    return schema


def _load_policy(expected_sha256: str | None = None):
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4*1024*1024)
    value, schema = json.loads(raw), json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema); gate.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)
    return value, {"path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
                   "mode": full["mode"], "size_bytes": full["size_bytes"]}


def configure_paths() -> None:
    v10.configure_paths(); gate.install()
    base.SCRIPT, base.POLICY, base.PACKAGE_SCHEMA = SCRIPT, gate.POLICY_PATH, gate.SCHEMA_PATH
    base.RECEIPT_SCHEMA, base.GATE, base.gate = v6.RECEIPT_SCHEMA, Path(gate.__file__).resolve(), gate
    base.EXTRACTOR, base.extractor = Path(extractor.__file__).resolve(), extractor
    base.TRANSFER_ID, base.PARTIAL_ROOT, base.FINAL_ROOT = TRANSFER_ID, PARTIAL_ROOT, FINAL_ROOT
    base.EXECUTION_RECEIPT, base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL
    v6.PARTIAL_ROOT, v6.FINAL_ROOT, v6.EXECUTION_ID = PARTIAL_ROOT, FINAL_ROOT, EXECUTION_ID


def raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int):
    configure_paths(); argv=v9.raw_build(receipt,source,partial,receipt_fd=receipt_fd,source_fd=source_fd)
    bind, additions = argv.index("--bind"), []
    for path in (Path(v10.__file__).resolve(), Path(gate.prior.__file__).resolve()):
        if str(path) not in argv: additions += ["--ro-bind",str(path),str(path)]
    argv[bind:bind]=additions; return argv


def canonical_argv(receipt_fd: int=101,source_fd: int=102):
    return v6._canonical(raw_build(v6.v5.prior.S5A0_INNER_RECEIPT,
        Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
        PARTIAL_ROOT,receipt_fd=receipt_fd,source_fd=source_fd),receipt_fd,source_fd)


def _iter_file_refs(value: Any):
    if isinstance(value,dict):
        if {"path","sha256","mode","size_bytes"}.issubset(value): yield value
        for child in value.values(): yield from _iter_file_refs(child)
    elif isinstance(value,list):
        for child in value: yield from _iter_file_refs(child)


def _add_nonempty(expected: dict[Path,str], item: Mapping[str,Any], zero_refs: dict[Path,dict[str,Any]], excluded: set[Path]) -> None:
    path=Path(item["path"])
    if item["size_bytes"]==0:
        if path not in zero_refs or dict(item)!=zero_refs[path]:
            raise ExecutionV11Error(f"unexpected zero-byte file reference: {path}")
        excluded.add(path); return
    if item["size_bytes"]<0:
        raise ExecutionV11Error(f"invalid negative file size: {path}")
    if path in expected and expected[path]!=item["sha256"]:
        raise ExecutionV11Error(f"conflicting recursive parent hash: {path}")
    expected[path]=item["sha256"]


def recursive_expected_map(policy: Mapping[str,Any]) -> tuple[dict[Path,str],dict[str,Any]]:
    parent10=json.loads(v10.POLICY.read_bytes());parent9=json.loads(v9.POLICY.read_bytes())
    base_v6,_=v6._load_policy(parent9["base_v6_policy"]["sha256"])
    base_v5,_=v6.v5._load_policy(base_v6["base_v5"]["sha256"])
    expected:dict[Path,str]={}
    for item in base_v5["contracts"].values():
        if item["size_bytes"]<=0: raise ExecutionV11Error(f"zero-byte base contract: {item['path']}")
        _add_nonempty(expected,item,{},set())
    zero_refs={V8_RESERVATION:parent10["preserved_v8"]["empty_receipt_reservation"],
               V10_RESERVATION:policy["preserved_v10"]["empty_receipt_reservation"]}
    excluded:set[Path]=set();source_counts={}
    base_v6_contract_refs=[base_v6["base_v5"],*base_v6["package_revision"].values(),
                           *base_v6["v6_contract"].values()]
    for item in base_v6_contract_refs:
        _add_nonempty(expected,item,zero_refs,excluded)
    source_counts["base_v6_contract_refs"]=len(base_v6_contract_refs)
    for name,value in (("v11",policy),("parent_v10",parent10),("parent_v9",parent9)):
        refs=list(_iter_file_refs(value));source_counts[name]=len(refs)
        for item in refs:_add_nonempty(expected,item,zero_refs,excluded)
    if excluded!={V8_RESERVATION,V10_RESERVATION}:
        raise ExecutionV11Error("exact v8 and v10 reservations must be the only exclusions")
    if preserved_v8_reservation()!=zero_refs[V8_RESERVATION]:
        raise ExecutionV11Error("v8 exclusion differs from specialized witness")
    if preserved_v10()["empty_receipt_reservation"]!=zero_refs[V10_RESERVATION]:
        raise ExecutionV11Error("v10 exclusion differs from specialized witness")
    parent10_nonempty={Path(x["path"]) for x in _iter_file_refs(parent10) if x["size_bytes"]>0}
    parent9_nonempty={Path(x["path"]) for x in _iter_file_refs(parent9) if x["size_bytes"]>0}
    if not parent10_nonempty.issubset(expected) or not parent9_nonempty.issubset(expected):
        raise ExecutionV11Error("recursive parent nonempty coverage incomplete")
    return expected,{"excluded_paths":sorted(map(str,excluded)),"excluded_unique_count":2,
        "recursive_source_ref_counts":source_counts,"parent_v10_nonempty_covered":len(parent10_nonempty),
        "parent_v9_nonempty_covered":len(parent9_nonempty),"all_other_zero_byte_refs_fail_closed":True}


def required_base_receipt_parent_paths() -> tuple[Path,...]:
    return tuple(dict.fromkeys((base.SCRIPT,base.POLICY,base.PACKAGE_SCHEMA,base.RECEIPT_SCHEMA,
        base.GATE,base.EXTRACTOR,base.BRIDGE,base.READER,base.S5A0_SCHEMA,base.S5A0_FINAL_SCHEMA,
        base.S5A0_SUPERVISOR_POLICY,base.COMPAT_GATE)))


def assert_base_receipt_parent_coverage(expected: Mapping[Path,str]) -> dict[str,Any]:
    required=required_base_receipt_parent_paths();missing=[str(path) for path in required if path not in expected]
    if missing: raise ExecutionV11Error("base receipt parent coverage missing: "+",".join(missing))
    return {"required_count":len(required),"missing":[],"status":"PASS_BASE_RECEIPT_PARENT_COVERAGE"}


def recursive_parent_v10_admission(expected_sha256: str) -> dict[str,Any]:
    original=v10.assert_fresh_v10
    def preserved_instead_of_fresh(): preserved_v10()
    try:
        v10.assert_fresh_v10=preserved_instead_of_fresh
        return v10.admit_policy(expected_sha256)
    finally:
        v10.assert_fresh_v10=original
        configure_paths()


def admit_policy(expected_sha256: str|None=None):
    global ACTIVE_POLICY,ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups())!=HOST_GROUPS: raise ExecutionV11Error("host groups drifted")
    assert_fresh_v11();policy,identity=_load_policy(expected_sha256)
    sections={"parent_v10":{"policy":v10.POLICY,"policy_schema":v10.POLICY_SCHEMA,
        "receipt_schema":v10.RECEIPT_SCHEMA,"supervisor":Path(v10.__file__).resolve(),"tests":v10.TESTS},
        "package_v7":{"policy":gate.POLICY_PATH,"schema":gate.SCHEMA_PATH,"gate":Path(gate.__file__).resolve(),
                      "tests":ROOT/"tests/test_s5a1_handoff_gate_v9.py"},
        "v11_contract":{"policy":POLICY_SCHEMA,"schema":RECEIPT_SCHEMA,"gate":SCRIPT,"tests":TESTS}}
    for section,paths in sections.items():
        for name,path in paths.items():assert_identity(path,policy[section][name])
    if preserved_v10()!=policy["preserved_v10"]:raise ExecutionV11Error("v10 evidence differs")
    if policy["tf_contract"]!=json.loads(v10.POLICY.read_bytes())["tf_contract"]:raise ExecutionV11Error("TF contract drifted")
    recursive_parent_v10_admission(policy["parent_v10"]["policy"]["sha256"])
    ACTIVE_POLICY,ACTIVE_POLICY_IDENTITY=policy,identity
    expected,witness=recursive_expected_map(policy);configure_paths();coverage=assert_base_receipt_parent_coverage(expected)
    argv,sandbox=canonical_argv(),policy["sandbox"]
    if v6._argv_hash(argv)!=sandbox["canonical_argv_sha256"] or len(argv)!=sandbox["canonical_token_count"]:
        raise ExecutionV11Error("canonical argv differs")
    if str(SCRIPT) not in argv or argv.count("--bind")!=1:raise ExecutionV11Error("worker path/bind differs")
    return {"canonical_argv_sha256":v6._argv_hash(argv),"canonical_token_count":len(argv),
            "generic_expected_count":len(expected),"recursive_coverage":witness,"base_coverage":coverage}


def build_bwrap_argv(receipt:Path,source:Path,partial:Path,*,receipt_fd=None,source_fd=None):
    if ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:raise ExecutionV11Error("admission/fds required")
    argv=raw_build(receipt,source,partial,receipt_fd=receipt_fd,source_fd=source_fd)
    if v6._argv_hash(v6._canonical(argv,receipt_fd,source_fd))!=ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV11Error("runtime argv differs")
    return argv


def load_package(root:Path):
    package=v6._BASE_LOAD_PACKAGE(root);report=gate.validate_package_root(root)
    if report.get("status")!="PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":raise ExecutionV11Error("strong semantic gate failed")
    v6._SEMANTIC_ROOTS.append(str(root));v6._SEMANTIC_REPORT=dict(report);return package


def _ref(value:Mapping[str,Any]):return {"path":value["path"],"sha256":value["sha256"]}


def v11_contract():
    if ACTIVE_POLICY is None or ACTIVE_POLICY_IDENTITY is None:raise ExecutionV11Error("policy not admitted")
    p=ACTIVE_POLICY
    return {"execution_policy":_ref(ACTIVE_POLICY_IDENTITY),"policy_schema":_ref(p["v11_contract"]["policy"]),
        "receipt_schema":_ref(p["v11_contract"]["schema"]),"supervisor":_ref(p["v11_contract"]["gate"]),
        "tests":_ref(p["v11_contract"]["tests"]),"parent_v10_policy":_ref(p["parent_v10"]["policy"]),
        "parent_v10_policy_schema":_ref(p["parent_v10"]["policy_schema"]),
        "parent_v10_receipt_schema":_ref(p["parent_v10"]["receipt_schema"]),
        "parent_v10_supervisor":_ref(p["parent_v10"]["supervisor"]),"parent_v10_tests":_ref(p["parent_v10"]["tests"]),
        "package_policy":_ref(p["package_v7"]["policy"]),"package_schema":_ref(p["package_v7"]["schema"]),
        "package_gate":_ref(p["package_v7"]["gate"]),"package_tests":_ref(p["package_v7"]["tests"])}


def _parent_v10_transform(value:Mapping[str,Any]):
    parent=json.loads(v10.POLICY.read_bytes());previous=(v10.ACTIVE_POLICY,v10.ACTIVE_POLICY_IDENTITY,v10.CAPTURED_TF_WITNESS)
    try:
        v10.ACTIVE_POLICY=parent;v10.ACTIVE_POLICY_IDENTITY=ACTIVE_POLICY["parent_v10"]["policy"]
        v10.CAPTURED_TF_WITNESS=copy.deepcopy(CAPTURED_TF_WITNESS);return v10.transform_receipt(value)
    finally:v10.ACTIVE_POLICY,v10.ACTIVE_POLICY_IDENTITY,v10.CAPTURED_TF_WITNESS=previous


def transform_receipt(value:Mapping[str,Any]):
    out=_parent_v10_transform(value);out.update(schema_version="smpcc-r8-liquid-s5a1-execution-receipt-v11",
        document_type="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V11",execution_id=EXECUTION_ID,
        receipt_id=f"{TRANSFER_ID}_execution_v11")
    out["preserved_v10"]=preserved_v10();out["v11_contract"]=v11_contract();out["execution"]["failure_receipt_schema_v11"]=True
    current_attempted=out["safety"]["real_bag_read_attempted"]
    current_confirmed=out["safety"]["real_bag_read_confirmed"]
    if not isinstance(current_attempted,bool) or not isinstance(current_confirmed,bool):
        raise ExecutionV11Error("current v11 real-bag read evidence is not boolean")
    if current_confirmed and not current_attempted:
        raise ExecutionV11Error("current v11 confirmed read without attempted read")
    out["safety"].update(v10_evidence_preserved=True,recursive_parent_coverage=True,
        bwrap_run=current_attempted)
    if out["status"]=="S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and not (
            current_attempted and current_confirmed and out["safety"]["bwrap_run"]):
        raise ExecutionV11Error("v11 success lacks current real-bag/bwrap evidence")
    if out["semantic_validation"]["status"]!="NOT_RUN_DUE_TO_FAILURE":
        out["semantic_validation"]["status"]="PASS_S5A1_HANDOFF_GATE_V9_STRONG_SEMANTICS"
    return out


def reserve_receipt(path:Path=EXECUTION_RECEIPT_PARTIAL):
    descriptor=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o440);os.fchmod(descriptor,0o440);return descriptor


def publish_reserved_receipt(descriptor:int,partial_path:Path,final_path:Path,value:Mapping[str,Any])->None:
    if base.publish_reserved_receipt is not publish_reserved_receipt:raise ExecutionV11Error("v11 publisher hook clobbered")
    try:
        transformed=transform_receipt(value);Draft202012Validator(receipt_schema()).validate(transformed);view=memoryview(gate.v1.canonical_json(transformed))
        while view:
            written=os.write(descriptor,view)
            if written<=0:raise ExecutionV11Error("short v11 receipt write")
            view=view[written:]
        os.fsync(descriptor)
    finally:os.close(descriptor)
    base.rename_noreplace(partial_path,final_path)


def mock_failure_receipt():return transform_receipt(v6._BASE_MOCK_FAILURE())


def _sanitized_subprocess_run(command:Sequence[str],**kwargs:Any):
    global CAPTURED_TF_WITNESS
    if "env" in kwargs:raise ExecutionV11Error("unfrozen host environment")
    completed=v6.v5._REAL_SUBPROCESS_RUN(command,env=dict(HOST_ENV),**kwargs);v6._WORKER_GROUP_WITNESS=False;CAPTURED_TF_WITNESS=None
    if completed.returncode==0:
        try:
            report=json.loads(completed.stdout);v6._WORKER_GROUP_WITNESS=report.get("worker_supplementary_groups")==MAPPED_GROUPS
            witness=report.get("tf_qc");CAPTURED_TF_WITNESS=witness if isinstance(witness,dict) else None
        except Exception:CAPTURED_TF_WITNESS=None
    return completed


class _SubprocessProxy:
    DEVNULL,PIPE=v6._subprocess.DEVNULL,v6._subprocess.PIPE
    run=staticmethod(_sanitized_subprocess_run)


SUBPROCESS_PROXY=_SubprocessProxy()


def install_runtime():
    if ACTIVE_POLICY is None:raise ExecutionV11Error("policy not admitted")
    configure_paths()
    parent9=json.loads(v9.POLICY.read_bytes())
    base_v6,base_v6_identity=v6._load_policy(parent9["base_v6_policy"]["sha256"])
    v6._ACTIVE_POLICY,v6._ACTIVE_POLICY_IDENTITY=copy.deepcopy(base_v6),base_v6_identity
    expected,witness=recursive_expected_map(ACTIVE_POLICY);coverage=assert_base_receipt_parent_coverage(expected)
    base.EXPECTED=expected;base.build_bwrap_argv=build_bwrap_argv;base.load_package=load_package
    base.reserve_receipt=reserve_receipt;base.publish_reserved_receipt=publish_reserved_receipt
    base.mock_failure_receipt=mock_failure_receipt;base.subprocess=SUBPROCESS_PROXY
    return {"recursive_coverage":witness,"base_coverage":coverage}


def worker(receipt_path:Path,source:Path,partial:Path):
    if os.getuid()!=1000 or os.getgid()!=1000 or os.getgroups()!=MAPPED_GROUPS:raise ExecutionV11Error("worker mapped groups differ")
    configure_paths();report=v6._BASE_WORKER(receipt_path,source,partial);report["worker_supplementary_groups"]=MAPPED_GROUPS;return report


def static_self_check():
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None;v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None
    report=admit_policy();runtime=install_runtime();schema=receipt_schema();Draft202012Validator(schema).validate(mock_failure_receipt());gate_report=gate.self_check()
    return {"status":"PASS_S5A1_EXECUTION_SUPERVISOR_V11_STATIC_CONTRACT","execution_id":EXECUTION_ID,"transfer_id":TRANSFER_ID,
        "policy_sha256":ACTIVE_POLICY_IDENTITY["sha256"],**report,"runtime_coverage":runtime,"package_gate_status":gate_report["status"],
        "receipt_schema_deep_closed":True,"recursive_parent_v10_admission":True,"strict_cli":True,
        "preserved_v10":preserved_v10(),"real_bag_read_attempted":False,"real_bag_read_confirmed":False,
        "bwrap_run":False,"files_written":False,"solver_executed":False,"sudo_used":False}


def execute_one_shot(receipt_path:Path,receipt_sha256:str,policy_sha256:str):
    global CAPTURED_TF_WITNESS
    CAPTURED_TF_WITNESS=None;v6._SEMANTIC_ROOTS,v6._SEMANTIC_REPORT=[],None;v6._WORKER_GROUP_WITNESS=False
    admit_policy(policy_sha256);install_runtime();v6._BASE_EXECUTE(receipt_path,receipt_sha256)
    value=json.loads(EXECUTION_RECEIPT.read_bytes());Draft202012Validator(receipt_schema()).validate(value);return value


def main(argv:Sequence[str]|None=None):
    parser=argparse.ArgumentParser(description=__doc__);subs=parser.add_subparsers(dest="command",required=True);subs.add_parser("self-check")
    run=subs.add_parser("run-one-shot");run.add_argument("--s5a0-receipt",type=Path,required=True);run.add_argument("--s5a0-receipt-sha256",required=True)
    run.add_argument("--execution-policy-sha256",required=True);run.add_argument("--authorize-execution-id",required=True)
    wp=subs.add_parser("_worker");wp.add_argument("--s5a0-receipt",type=Path,required=True);wp.add_argument("--source",type=Path,required=True);wp.add_argument("--partial-root",type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        if args.command=="self-check":result=static_self_check()
        elif args.command=="run-one-shot":
            if args.authorize_execution_id!=EXECUTION_ID:raise ExecutionV11Error("exact v11 authorization differs")
            result=execute_one_shot(args.s5a0_receipt,args.s5a0_receipt_sha256,args.execution_policy_sha256)
        else:result=worker(args.s5a0_receipt,args.source,args.partial_root)
    except Exception as exc:
        print(json.dumps({"status":"FAIL_CLOSED","execution_id":EXECUTION_ID,"error":str(exc)},sort_keys=True),file=sys.stderr);return 2
    print(json.dumps(result,allow_nan=False,sort_keys=True,separators=(",",":")));return 0


if __name__=="__main__":raise SystemExit(main())
