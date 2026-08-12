#!/usr/bin/env python3
"""S5A1 execution v5: pre-frozen one-shot contract and strong publication gate.

``self-check`` is static.  It does not read a bag, run bubblewrap, or write any
output.  Real execution additionally requires the externally authorized exact
execution-policy SHA-256.
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
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_execution_supervisor_v1 as base
import r8_liquid_s5a1_execution_supervisor_v4 as prior


sys.dont_write_bytecode = True
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T210719Z_v5"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v5.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5a1_execution_policy_v5.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v5.json"
TESTS = ROOT / "tests/test_s5a1_execution_supervisor_v5.py"
PARTIAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial")
FINAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v5.json")
EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT.with_name(EXECUTION_RECEIPT.name + ".partial")
HOST_ENV = {"PATH": "/usr/bin", "HOME": "/nonexistent", "PYTHONDONTWRITEBYTECODE": "1"}
CHILD_ENV = dict(HOST_ENV)
FD_RECEIPT = "@S5A0_RECEIPT_FD@"
FD_SOURCE = "@SOURCE_BAG_FD@"
_REAL_SUBPROCESS_RUN = _subprocess.run
_ACTIVE_POLICY: dict[str, Any] | None = None
_ACTIVE_POLICY_IDENTITY: dict[str, Any] | None = None
_SEMANTIC_ROOTS: list[str] = []


class ExecutionV5Error(ValueError):
    """The pre-frozen execution contract or one-shot state failed closed."""


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")).encode()


def _nul_hash(argv: Sequence[str]) -> str:
    return hashlib.sha256("\0".join(argv).encode()).hexdigest()


def _file_identity(path: Path, expected: Mapping[str, Any] | None = None) -> dict[str, Any]:
    _raw, identity = base.read_file_identity(path, maximum_bytes=64 * 1024 * 1024)
    compact = {"path": str(path), "sha256": identity["sha256"],
               "mode": identity["mode"], "size_bytes": identity["size_bytes"]}
    if expected is not None and compact != dict(expected):
        raise ExecutionV5Error(f"frozen file identity drifted: {path}")
    return compact


def _tool_identity(path: Path, expected: Mapping[str, Any] | None = None) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    metadata = os.stat(resolved, follow_symlinks=False)
    if not stat.S_ISREG(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o755:
        raise ExecutionV5Error(f"tool is not exact 0755 regular file: {path}")
    digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    observed = {"path": str(path), "resolved_path": str(resolved), "sha256": digest,
                "mode": "0755", "size_bytes": metadata.st_size}
    if expected is not None and observed != dict(expected):
        raise ExecutionV5Error(f"frozen tool identity drifted: {path}")
    return observed


def _load_policy(expected_sha256: str | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, full_identity = base.read_file_identity(POLICY, expected_sha256, maximum_bytes=4 * 1024 * 1024)
    value = json.loads(raw)
    schema = json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(value)
    identity = {"path": str(POLICY), "sha256": full_identity["sha256"],
                "mode": full_identity["mode"], "size_bytes": full_identity["size_bytes"]}
    return value, identity


def _canonicalize_fds(argv: Sequence[str], receipt_fd: int, source_fd: int) -> list[str]:
    replacements = {f"/proc/self/fd/{receipt_fd}": f"/proc/self/fd/{FD_RECEIPT}",
                    f"/proc/self/fd/{source_fd}": f"/proc/self/fd/{FD_SOURCE}"}
    return [replacements.get(token, token) for token in argv]


def _install_paths() -> None:
    prior._install_revision()
    base.SCRIPT = SCRIPT
    base.POLICY = prior.PACKAGE_POLICY
    base.RECEIPT_SCHEMA = RECEIPT_SCHEMA
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL


def _raw_build_bwrap_argv(receipt: Path, source: Path, partial: Path,
                          *, receipt_fd: int, source_fd: int) -> list[str]:
    _install_paths()
    argv = prior.build_bwrap_argv(receipt, source, partial,
                                  receipt_fd=receipt_fd, source_fd=source_fd)
    bind_index = argv.index("--bind")
    v4_path = str(prior.SCRIPT)
    if v4_path not in argv:
        argv[bind_index:bind_index] = ["--ro-bind", v4_path, v4_path]
    return argv


def _assert_child_environment(argv: Sequence[str]) -> None:
    if argv.count("--clearenv") != 1:
        raise ExecutionV5Error("child clearenv contract differs")
    observed: dict[str, str] = {}
    for index, token in enumerate(argv):
        if token == "--setenv":
            key, value = argv[index + 1:index + 3]
            if key in observed:
                raise ExecutionV5Error(f"duplicate child environment key: {key}")
            observed[key] = value
    if observed != CHILD_ENV:
        raise ExecutionV5Error("complete child environment differs")


def _verify_policy_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    required = {
        "execution_policy_schema_v5", "execution_receipt_schema_v5",
        "execution_supervisor_v5", "execution_supervisor_v5_tests",
        "handoff_gate_v3", "handoff_gate_v3_tests", "signal_extractor_v3",
        "signal_extractor_v3_tests", "package_policy", "package_schema",
        "handoff_gate_v1", "handoff_gate_v2", "signal_extractor_v1",
        "signal_extractor_v2", "motion_bridge_v1", "reader_v1", "reader_v2",
        "reader_v3", "reader_v4", "s5a0_inner_receipt_schema_v2",
        "s5a0_outer_receipt_schema_v5", "s5a0_supervisor_policy_v5",
        "compatibility_gate_v1", "execution_supervisor_v1",
        "execution_supervisor_v3", "execution_supervisor_v4",
        "execution_receipt_schema_v1", "execution_receipt_schema_v3",
        "execution_receipt_schema_v4", "s5a0_outer_final_receipt",
        "s5a0_inner_receipt",
    }
    contracts = policy["contracts"]
    if set(contracts) != required:
        raise ExecutionV5Error("execution dependency inventory differs")
    for identity in contracts.values():
        _file_identity(Path(identity["path"]), identity)
    for identity in policy["tools"].values():
        _tool_identity(Path(identity["path"]), identity)
    sandbox = policy["sandbox"]
    if hashlib.sha256(_canonical_json(sandbox["child_environment"])).hexdigest() != hashlib.sha256(_canonical_json(CHILD_ENV)).hexdigest():
        raise ExecutionV5Error("child environment hash differs")
    if hashlib.sha256(_canonical_json(sandbox["host_subprocess_environment"])).hexdigest() != hashlib.sha256(_canonical_json(HOST_ENV)).hexdigest():
        raise ExecutionV5Error("host environment hash differs")
    sample = _raw_build_bwrap_argv(
        prior.S5A0_INNER_RECEIPT,
        Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),
        PARTIAL_ROOT, receipt_fd=101, source_fd=102,
    )
    canonical = _canonicalize_fds(sample, 101, 102)
    _assert_child_environment(canonical)
    if canonical != sandbox["canonical_argv"] or _nul_hash(canonical) != sandbox["canonical_argv_sha256"]:
        raise ExecutionV5Error("canonical argv differs from frozen policy")
    if canonical.count("--bind") != 1:
        raise ExecutionV5Error("writable bind count differs")
    index = canonical.index("--bind")
    if canonical[index + 1:index + 3] != [str(PARTIAL_ROOT), str(PARTIAL_ROOT)]:
        raise ExecutionV5Error("exact writable bind differs")
    return {"canonical_argv": canonical, "canonical_argv_sha256": _nul_hash(canonical)}


def _admit_policy(expected_sha256: str | None = None) -> dict[str, Any]:
    global _ACTIVE_POLICY, _ACTIVE_POLICY_IDENTITY
    policy, identity = _load_policy(expected_sha256)
    report = _verify_policy_contract(policy)
    _ACTIVE_POLICY, _ACTIVE_POLICY_IDENTITY = policy, identity
    return report


def build_bwrap_argv(receipt: Path, source: Path, partial: Path,
                     *, receipt_fd: int | None = None, source_fd: int | None = None) -> list[str]:
    if _ACTIVE_POLICY is None or receipt_fd is None or source_fd is None:
        raise ExecutionV5Error("pre-frozen policy and both opened fds are required")
    argv = _raw_build_bwrap_argv(receipt, source, partial,
                                 receipt_fd=receipt_fd, source_fd=source_fd)
    canonical = _canonicalize_fds(argv, receipt_fd, source_fd)
    if canonical != _ACTIVE_POLICY["sandbox"]["canonical_argv"]:
        raise ExecutionV5Error("expanded runtime argv differs from frozen canonical argv")
    if _nul_hash(canonical) != _ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"]:
        raise ExecutionV5Error("expanded runtime argv hash differs")
    _assert_child_environment(argv)
    return argv


def load_package(root: Path) -> dict[str, bytes]:
    package = prior.load_package(root)
    _SEMANTIC_ROOTS.append(str(root))
    return package


def _subprocess_run(command: Sequence[str], **kwargs: Any) -> Any:
    if "env" in kwargs:
        raise ExecutionV5Error("caller supplied an unfrozen host environment")
    return _REAL_SUBPROCESS_RUN(command, env=dict(HOST_ENV), **kwargs)


_SUBPROCESS_PROXY = SimpleNamespace(run=_subprocess_run, DEVNULL=_subprocess.DEVNULL, PIPE=_subprocess.PIPE)


def _receipt_contract() -> dict[str, Any]:
    if _ACTIVE_POLICY is None or _ACTIVE_POLICY_IDENTITY is None:
        raise ExecutionV5Error("execution policy was not admitted")
    contracts = _ACTIVE_POLICY["contracts"]
    def ref(name: str) -> dict[str, str]:
        value = contracts[name]
        return {"path": value["path"], "sha256": value["sha256"]}
    env_hash = hashlib.sha256(_canonical_json(CHILD_ENV)).hexdigest()
    return {
        "execution_policy": {"path": _ACTIVE_POLICY_IDENTITY["path"], "sha256": _ACTIVE_POLICY_IDENTITY["sha256"]},
        "execution_policy_schema": ref("execution_policy_schema_v5"),
        "receipt_schema": ref("execution_receipt_schema_v5"),
        "supervisor": ref("execution_supervisor_v5"),
        "tests": ref("execution_supervisor_v5_tests"),
        "gate_v3": ref("handoff_gate_v3"), "gate_v3_tests": ref("handoff_gate_v3_tests"),
        "extractor_v3": ref("signal_extractor_v3"), "extractor_v3_tests": ref("signal_extractor_v3_tests"),
        "tools": _ACTIVE_POLICY["tools"],
        "canonical_argv_sha256": _ACTIVE_POLICY["sandbox"]["canonical_argv_sha256"],
        "child_environment_sha256": env_hash, "host_environment_sha256": env_hash,
        "apparmor": "NOT_USED",
    }


def _semantic_receipt() -> dict[str, Any]:
    roots = list(_SEMANTIC_ROOTS)
    complete = roots == [str(PARTIAL_ROOT), str(FINAL_ROOT)]
    prior_value = prior._semantic_receipt()
    return {
        "gate": prior_value["gate"], "validator": "validate_package_root",
        "before_atomic_rename": len(roots) >= 1 and roots[0] == str(PARTIAL_ROOT),
        "after_atomic_rename": complete,
        "report_sha256": prior_value["report_sha256"],
        "status": prior_value["status"] if complete else "NOT_RUN_DUE_TO_FAILURE",
    }


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    transformed = {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v5",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V5",
        "execution_id": EXECUTION_ID, "receipt_id": f"{TRANSFER_ID}_execution_v5",
        "status": value["status"], "contract": _receipt_contract(),
        "parents": prior._parent_map(), "execution": dict(value["execution"]),
        "inputs": value["inputs"], "roots": value["roots"], "package": value["package"],
        "semantic_validation": _semantic_receipt(), "compatibility": value["compatibility"],
        "safety": value["safety"], "failure": value["failure"], "next": value["next"],
    }
    transformed["execution"]["supplementary_groups"] = []
    if transformed["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" and not transformed["semantic_validation"]["after_atomic_rename"]:
        raise ExecutionV5Error("success lacks semantic validation before rename and after final publication")
    return transformed


def reserve_receipt() -> int:
    return prior._BASE_RESERVE_RECEIPT(EXECUTION_RECEIPT_PARTIAL)


def publish_reserved_receipt(descriptor: int, partial_path: Path, final_path: Path,
                             value: Mapping[str, Any]) -> None:
    transformed = transform_receipt(value)
    Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(transformed)
    prior._BASE_PUBLISH_RECEIPT(descriptor, partial_path, final_path, transformed)


def mock_failure_receipt() -> dict[str, Any]:
    return transform_receipt(prior._BASE_MOCK_FAILURE())


def _install_runtime() -> None:
    _install_paths()
    assert _ACTIVE_POLICY is not None
    base.EXPECTED = {Path(value["path"]): value["sha256"] for value in _ACTIVE_POLICY["contracts"].values()}
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt
    base.subprocess = _SUBPROCESS_PROXY


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != []:
        raise ExecutionV5Error("worker identity is not uid/gid 1000 with zero groups")
    _install_paths()
    return prior._BASE_WORKER(receipt_path, source, partial)


def static_self_check() -> dict[str, Any]:
    global _SEMANTIC_ROOTS
    _SEMANTIC_ROOTS = []
    prior._SEMANTIC_REPORT = None
    report = _admit_policy()
    _install_runtime()
    receipt_schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(receipt_schema)
    prior.gate_v3.assert_schema_deep_closed(receipt_schema)
    Draft202012Validator(receipt_schema).validate(mock_failure_receipt())
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V5_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "policy_sha256": _ACTIVE_POLICY_IDENTITY["sha256"],
        "policy_schema_sha256": _file_identity(POLICY_SCHEMA)["sha256"],
        "receipt_schema_sha256": _file_identity(RECEIPT_SCHEMA)["sha256"],
        "supervisor_sha256": _file_identity(SCRIPT)["sha256"],
        "tests_sha256": _file_identity(TESTS)["sha256"],
        "canonical_argv_sha256": report["canonical_argv_sha256"],
        "contract_file_count": len(_ACTIVE_POLICY["contracts"]),
        "tool_count": len(_ACTIVE_POLICY["tools"]),
        "real_bag_read": False, "worker_run": False, "bwrap_run": False,
        "files_written": False, "network_used": False, "gpu_exposed": False,
        "solver_executed": False, "sudo_used": False, "apparmor": "NOT_USED",
    }


def execute_one_shot(receipt_path: Path, receipt_sha256: str,
                     execution_policy_sha256: str) -> dict[str, Any]:
    global _SEMANTIC_ROOTS
    _SEMANTIC_ROOTS = []
    prior._SEMANTIC_REPORT = None
    _admit_policy(execution_policy_sha256)
    _install_runtime()
    prior._BASE_EXECUTE(receipt_path, receipt_sha256)
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
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "run-one-shot":
            if args.authorize_execution_id != EXECUTION_ID:
                raise ExecutionV5Error("exact execution authorization differs")
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
