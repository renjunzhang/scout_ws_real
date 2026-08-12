#!/usr/bin/env python3
"""S5A1 v6 one-shot supervisor with mapped-group and hook regressions fixed.

Self-check is static.  The preserved v5 empty partial/root reservation are
read-only parents and are never removed, overwritten, or reused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import subprocess as _subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_execution_supervisor_v1 as base
import r8_liquid_s5a1_execution_supervisor_v5 as v5
import r8_liquid_s5a1_handoff_gate_v4 as gate_v4


sys.dont_write_bytecode = True
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T213146Z_v6"
TRANSFER_ID = gate_v4.TRANSFER_ID
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v6.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v6.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v6.json"
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v6.py"
PACKAGE_TESTS = ROOT / "tests/test_s5a1_handoff_gate_v4.py"
PARTIAL_ROOT = Path(gate_v4.PARTIAL_ROOT)
FINAL_ROOT = Path(gate_v4.FINAL_ROOT)
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v6.json")
EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT.with_name(EXECUTION_RECEIPT.name + ".partial")
V5_PARTIAL = Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1.partial")
V5_RESERVATION = Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1.execution_v5.json.partial")
V5_FINAL_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1.execution_v5.json")
MAPPED_GROUPS = [65534,65534,65534,65534,65534,65534,65534,65534,1000]
HOST_GROUPS = [4,24,27,30,46,122,135,136,1000]
HOST_ENV = dict(v5.HOST_ENV)
CHILD_ENV = dict(v5.CHILD_ENV)
_BASE_BUILD = v5.prior._BASE_BUILD_BWRAP
_BASE_LOAD_PACKAGE = v5.prior._BASE_LOAD_PACKAGE
_BASE_RESERVE = v5.prior._BASE_RESERVE_RECEIPT
_BASE_PUBLISH = v5.prior._BASE_PUBLISH_RECEIPT
_BASE_MOCK_FAILURE = v5.prior._BASE_MOCK_FAILURE
_BASE_EXECUTE = v5.prior._BASE_EXECUTE
_BASE_WORKER = v5.prior._BASE_WORKER
_ACTIVE_POLICY: dict[str, Any] | None = None
_ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
_SEMANTIC_ROOTS: list[str] = []
_SEMANTIC_REPORT: dict[str, Any] | None = None
_WORKER_GROUP_WITNESS = False


class ExecutionV6Error(ValueError):
    """The v6 frozen contract failed closed."""


def _identity(path: Path, expected: Mapping[str, Any] | None = None) -> dict[str, Any]:
    raw, value = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    compact = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
               "mode": value["mode"], "size_bytes": value["size_bytes"]}
    if expected is not None and compact != dict(expected):
        raise ExecutionV6Error(f"file identity drifted: {path}")
    return compact


def _preserved_v5() -> dict[str, Any]:
    metadata = os.lstat(V5_PARTIAL)
    if (not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o700
            or metadata.st_nlink != 2 or any(V5_PARTIAL.iterdir())):
        raise ExecutionV6Error("preserved v5 partial is not exact empty 0700 directory")
    reservation_metadata = os.lstat(V5_RESERVATION)
    if not stat.S_ISREG(reservation_metadata.st_mode) or reservation_metadata.st_nlink != 1:
        raise ExecutionV6Error("preserved v5 reservation is not a regular single-link file")
    reservation_raw = V5_RESERVATION.read_bytes()
    reservation = {"path": str(V5_RESERVATION),
                   "sha256": hashlib.sha256(reservation_raw).hexdigest(),
                   "mode": f"{stat.S_IMODE(reservation_metadata.st_mode):04o}",
                   "size_bytes": reservation_metadata.st_size}
    if reservation["sha256"] != hashlib.sha256(b"").hexdigest() or reservation["size_bytes"] != 0 or reservation["mode"] != "0440":
        raise ExecutionV6Error("preserved v5 reservation is not exact empty 0440 file")
    if os.path.lexists(V5_FINAL_RECEIPT):
        raise ExecutionV6Error("unexpected v5 final receipt exists")
    return {
        "partial_root": {"path": str(V5_PARTIAL), "inode": metadata.st_ino,
                         "mode": "0700", "size_bytes": metadata.st_size,
                         "nlink": metadata.st_nlink, "entry_count": 0,
                         "inventory_sha256": hashlib.sha256(b"").hexdigest()},
        "empty_reservation": reservation, "final_receipt_absent": True,
    }


def _load_policy(expected_sha256: str | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, full = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value = json.loads(raw)
    schema = json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(value)
    identity = {"path": str(POLICY), "sha256": hashlib.sha256(raw).hexdigest(),
                "mode": full["mode"], "size_bytes": full["size_bytes"]}
    return value, identity


def _configure_paths() -> None:
    v5.prior._install_revision()
    gate_v4.install()
    base.SCRIPT = SCRIPT
    base.POLICY = gate_v4.POLICY_PATH
    base.PACKAGE_SCHEMA = gate_v4.SCHEMA_PATH
    base.RECEIPT_SCHEMA = RECEIPT_SCHEMA
    base.GATE = Path(gate_v4.__file__).resolve()
    base.gate = gate_v4
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL


def _raw_build(receipt: Path, source: Path, partial: Path, *, receipt_fd: int, source_fd: int) -> list[str]:
    argv = _BASE_BUILD(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    clear = argv.index("--clearenv") + 1
    argv[clear:clear] = ["--cap-drop", "ALL", "--uid", "1000", "--gid", "1000", "--unshare-net"]
    dependencies = [
        v5.SCRIPT, v5.prior.SCRIPT, v5.prior.SUPERVISOR_V1,
        gate_v4.BASE_POLICY_PATH, gate_v4.BASE_SCHEMA_PATH,
        v5.prior.GATE_V1, v5.prior.GATE_V2, v5.prior.GATE_V3,
        v5.prior.EXTRACTOR_V1, v5.prior.EXTRACTOR_V2,
        *v5.prior.READERS[:3], v5.prior.RECEIPT_SCHEMA_V1,
    ]
    bind = argv.index("--bind")
    additions: list[str] = []
    for path in dependencies:
        token = str(path)
        if token not in argv:
            additions += ["--ro-bind", token, token]
    argv[bind:bind] = additions
    return argv


def _canonical(argv: Sequence[str], receipt_fd: int, source_fd: int) -> list[str]:
    return v5._canonicalize_fds(argv, receipt_fd, source_fd)


def _argv_hash(argv: Sequence[str]) -> str:
    return v5._nul_hash(argv)


def _verify_policy(policy: Mapping[str, Any]) -> dict[str, Any]:
    base_v5, base_identity = v5._load_policy(policy["base_v5"]["sha256"])
    v5._verify_policy_contract(base_v5)
    if base_identity != policy["base_v5"]:
        raise ExecutionV6Error("base v5 policy identity differs")
    expected_paths = {
        "policy_delta": gate_v4.POLICY_PATH, "schema_delta": gate_v4.SCHEMA_PATH,
        "gate": Path(gate_v4.__file__).resolve(), "tests": PACKAGE_TESTS,
    }
    for name, path in expected_paths.items():
        _identity(path, policy["package_revision"][name])
    expected_v6 = {"policy_schema": POLICY_SCHEMA, "receipt_schema": RECEIPT_SCHEMA,
                   "supervisor": SCRIPT, "tests": TESTS}
    for name, path in expected_v6.items():
        _identity(path, policy["v6_contract"][name])
    if _preserved_v5() != policy["preserved_v5_failure"]:
        raise ExecutionV6Error("preserved v5 failure identities differ")
    _configure_paths()
    sample = _raw_build(v5.prior.S5A0_INNER_RECEIPT,
                        Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
                        PARTIAL_ROOT, receipt_fd=101, source_fd=102)
    canonical = _canonical(sample, 101, 102)
    sandbox = policy["sandbox"]
    contract = sandbox["canonical_argv_contract"]
    if (_argv_hash(canonical) != contract["sha256"]
            or len(canonical) != contract["token_count"]):
        raise ExecutionV6Error("v6 canonical argv differs")
    if canonical.count("--bind") != 1:
        raise ExecutionV6Error("v6 writable bind count differs")
    index = canonical.index("--bind")
    if canonical[index + 1:index + 3] != [str(PARTIAL_ROOT), str(PARTIAL_ROOT)]:
        raise ExecutionV6Error("v6 exact writable bind differs")
    v5._assert_child_environment(canonical)
    return {"canonical_argv_sha256": _argv_hash(canonical)}


def _admit_policy(expected_sha256: str | None = None) -> dict[str, Any]:
    global _ACTIVE_POLICY, _ACTIVE_POLICY_IDENTITY
    if sorted(os.getgroups()) != HOST_GROUPS:
        raise ExecutionV6Error("host supplementary groups differ from exact frozen list")
    policy, identity = _load_policy(expected_sha256)
    report = _verify_policy(policy)
    _ACTIVE_POLICY, _ACTIVE_POLICY_IDENTITY = policy, identity
    return report


def build_bwrap_argv(receipt: Path, source: Path, partial: Path, *, receipt_fd=None, source_fd=None) -> list[str]:
    if _ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV6Error("admitted v6 policy and both fds are required")
    before = (base.reserve_receipt, base.publish_reserved_receipt, base.mock_failure_receipt)
    argv = _raw_build(receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd)
    if before != (base.reserve_receipt, base.publish_reserved_receipt, base.mock_failure_receipt):
        raise ExecutionV6Error("build argv clobbered v6 publication hooks")
    canonical = _canonical(argv, receipt_fd, source_fd)
    contract = _ACTIVE_POLICY["sandbox"]["canonical_argv_contract"]
    if _argv_hash(canonical) != contract["sha256"] or len(canonical) != contract["token_count"]:
        raise ExecutionV6Error("runtime argv expansion differs")
    return argv


def load_package(root: Path) -> dict[str, bytes]:
    global _SEMANTIC_REPORT
    package = _BASE_LOAD_PACKAGE(root)
    report = gate_v4.validate_package_root(root)
    if report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS":
        raise ExecutionV6Error("gate v4 strong semantic validation failed")
    _SEMANTIC_ROOTS.append(str(root))
    _SEMANTIC_REPORT = dict(report)
    return package


def _ref(value: Mapping[str, Any]) -> dict[str, str]:
    return {"path": value["path"], "sha256": value["sha256"]}


def _contract() -> dict[str, Any]:
    if _ACTIVE_POLICY is None or _ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV6Error("v6 policy is not admitted")
    p = _ACTIVE_POLICY
    return {"execution_policy": _ref(_ACTIVE_POLICY_IDENTITY),
            "policy_schema": _ref(p["v6_contract"]["policy_schema"]),
            "receipt_schema": _ref(p["v6_contract"]["receipt_schema"]),
            "supervisor": _ref(p["v6_contract"]["supervisor"]),
            "tests": _ref(p["v6_contract"]["tests"]),
            "package_policy": _ref(p["package_revision"]["policy_delta"]),
            "package_schema": _ref(p["package_revision"]["schema_delta"]),
            "package_gate": _ref(p["package_revision"]["gate"]),
            "package_tests": _ref(p["package_revision"]["tests"]),
            "base_v5_policy": _ref(p["base_v5"])}


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    observed_host_groups = sorted(value["execution"]["supplementary_groups"])
    if observed_host_groups != HOST_GROUPS:
        raise ExecutionV6Error("base receipt host supplementary groups differ")
    complete = _SEMANTIC_ROOTS == [str(PARTIAL_ROOT), str(FINAL_ROOT)]
    semantic_hash = (hashlib.sha256(v5._canonical_json(_SEMANTIC_REPORT)).hexdigest()
                     if complete and _SEMANTIC_REPORT is not None else None)
    transformed = {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v6",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V6",
        "execution_id": EXECUTION_ID, "receipt_id": f"{TRANSFER_ID}_execution_v6",
        "status": value["status"], "contract": _contract(),
        "preserved_v5_failure": _preserved_v5(),
        "execution": {"uid": 1000, "gid": 1000,
                      "host_supplementary_groups": observed_host_groups,
                      "worker_supplementary_groups": MAPPED_GROUPS,
                      "worker_group_witness": _WORKER_GROUP_WITNESS,
                      "argv_sha256": value["execution"]["argv_sha256"],
                      "timeout_seconds": 180, "return_code": value["execution"]["return_code"],
                      "one_shot": True, "hooks_not_clobbered": True,
                      "failure_receipt_schema_v6": True},
        "inputs": value["inputs"], "roots": value["roots"], "package": value["package"],
        "semantic_validation": {"gate": _ref(_ACTIVE_POLICY["package_revision"]["gate"]),
            "before_atomic_rename": bool(_SEMANTIC_ROOTS and _SEMANTIC_ROOTS[0] == str(PARTIAL_ROOT)),
            "after_atomic_rename": complete, "report_sha256": semantic_hash,
            "status": "PASS_S5A1_HANDOFF_GATE_V4_STRONG_SEMANTICS" if complete else "NOT_RUN_DUE_TO_FAILURE"},
        "compatibility": value["compatibility"],
        "safety": {"real_bag_read_attempted": value["safety"]["real_bag_read_attempted"],
                   "real_bag_read_confirmed": value["safety"]["real_bag_read_confirmed"],
                   "source_bag_executed": False, "optional_bag_read": False,
                   "gpu_exposed": False, "solver_executed": False, "network_used": False,
                   "sudo_used": False, "failure_preserved": value["safety"]["failure_preserved"],
                   "v5_evidence_preserved": True},
        "failure": value["failure"], "next": value["next"],
    }
    if transformed["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and not complete:
        raise ExecutionV6Error("success lacks both strong semantic validations")
    return transformed


def reserve_receipt() -> int:
    return _BASE_RESERVE(EXECUTION_RECEIPT_PARTIAL)


def publish_reserved_receipt(descriptor: int, partial_path: Path, final_path: Path,
                             value: Mapping[str, Any]) -> None:
    if base.publish_reserved_receipt is not publish_reserved_receipt:
        raise ExecutionV6Error("v6 publish hook was clobbered")
    transformed = transform_receipt(value)
    Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(transformed)
    _BASE_PUBLISH(descriptor, partial_path, final_path, transformed)


def mock_failure_receipt() -> dict[str, Any]:
    value = _BASE_MOCK_FAILURE()
    value["execution"]["supplementary_groups"] = HOST_GROUPS
    return transform_receipt(value)


def _install_runtime() -> None:
    _configure_paths()
    assert _ACTIVE_POLICY is not None
    base_policy, _ = v5._load_policy(_ACTIVE_POLICY["base_v5"]["sha256"])
    expected = {Path(value["path"]): value["sha256"] for value in base_policy["contracts"].values()}
    for section in ("package_revision", "v6_contract"):
        expected.update({Path(value["path"]): value["sha256"] for value in _ACTIVE_POLICY[section].values()})
    expected[gate_v4.POLICY_PATH] = _ACTIVE_POLICY["package_revision"]["policy_delta"]["sha256"]
    expected[gate_v4.SCHEMA_PATH] = _ACTIVE_POLICY["package_revision"]["schema_delta"]["sha256"]
    base.EXPECTED = expected
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt
    base.subprocess = _SUBPROCESS_PROXY


def _sanitized_subprocess_run(command: Sequence[str], **kwargs: Any):
    global _WORKER_GROUP_WITNESS
    if "env" in kwargs:
        raise ExecutionV6Error("caller supplied unfrozen host environment")
    completed = v5._REAL_SUBPROCESS_RUN(command, env=dict(HOST_ENV), **kwargs)
    _WORKER_GROUP_WITNESS = False
    if completed.returncode == 0:
        try:
            report = json.loads(completed.stdout)
            _WORKER_GROUP_WITNESS = report.get("worker_supplementary_groups") == MAPPED_GROUPS
        except Exception:
            _WORKER_GROUP_WITNESS = False
    return completed


class _SubprocessProxy:
    DEVNULL = _subprocess.DEVNULL
    PIPE = _subprocess.PIPE
    run = staticmethod(_sanitized_subprocess_run)


_SUBPROCESS_PROXY = _SubprocessProxy()


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != MAPPED_GROUPS:
        raise ExecutionV6Error("worker mapped supplementary groups differ from exact frozen list")
    _configure_paths()
    report = _BASE_WORKER(receipt_path, source, partial)
    report["worker_supplementary_groups"] = list(MAPPED_GROUPS)
    return report


def static_self_check() -> dict[str, Any]:
    global _SEMANTIC_ROOTS, _SEMANTIC_REPORT
    _SEMANTIC_ROOTS, _SEMANTIC_REPORT = [], None
    report = _admit_policy()
    _install_runtime()
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate_v4.v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(mock_failure_receipt())
    gate_report = gate_v4.self_check()
    return {"status": "PASS_S5A1_EXECUTION_SUPERVISOR_V6_STATIC_CONTRACT",
            "execution_id": EXECUTION_ID, "transfer_id": TRANSFER_ID,
            "policy_sha256": _ACTIVE_POLICY_IDENTITY["sha256"],
            "canonical_argv_sha256": report["canonical_argv_sha256"],
            "mapped_groups": MAPPED_GROUPS, "v5_failure_preserved": True,
            "package_gate_status": gate_report["status"], "hooks_not_clobbered": True,
            "real_bag_read": False, "optional_bag_read": False, "bwrap_run": False,
            "files_written": False, "gpu_exposed": False, "solver_executed": False,
            "sudo_used": False, "network_used": False}


def execute_one_shot(receipt_path: Path, receipt_sha256: str, policy_sha256: str):
    global _SEMANTIC_ROOTS, _SEMANTIC_REPORT
    _SEMANTIC_ROOTS, _SEMANTIC_REPORT = [], None
    _admit_policy(policy_sha256)
    _install_runtime()
    _BASE_EXECUTE(receipt_path, receipt_sha256)
    value = json.loads(EXECUTION_RECEIPT.read_bytes())
    Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check")
    run = sub.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt", type=Path, required=True)
    run.add_argument("--s5a0-receipt-sha256", required=True)
    run.add_argument("--execution-policy-sha256", required=True)
    run.add_argument("--authorize-execution-id", required=True)
    sandbox = sub.add_parser("_worker")
    sandbox.add_argument("--s5a0-receipt", type=Path, required=True)
    sandbox.add_argument("--source", type=Path, required=True)
    sandbox.add_argument("--partial-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check": result = static_self_check()
        elif args.command == "run-one-shot":
            if args.authorize_execution_id != EXECUTION_ID:
                raise ExecutionV6Error("exact v6 execution authorization differs")
            result = execute_one_shot(args.s5a0_receipt, args.s5a0_receipt_sha256,
                                      args.execution_policy_sha256)
        else: result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as exc:
        print(json.dumps({"status":"FAIL_CLOSED","execution_id":EXECUTION_ID,"error":str(exc)},sort_keys=True),file=sys.stderr)
        return 2
    print(json.dumps(result,allow_nan=False,sort_keys=True,separators=(",",":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
