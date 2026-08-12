#!/usr/bin/env python3
"""S5A1 execution v4 with host-side strong semantic directory admission.

This create-new revision keeps the frozen S5A1 transfer roots and the exact
S5A0 v5 outer/inner parents.  It replaces the signal extractor with v3 and
requires handoff gate v3's public ``validate_package_root`` on the host before
any successful execution receipt can be published.  ``self-check`` is static:
it never opens the source bag, runs bubblewrap, or writes an output.
"""

from __future__ import annotations

import argparse
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

import r8_liquid_s5a1_execution_supervisor_v1 as base
import r8_liquid_s5a1_handoff_gate_v3 as gate_v3
import r8_liquid_s5a1_ros1_signal_extractor_v3 as extractor_v3


sys.dont_write_bytecode = True
ROOT = MODULE_DIR.parent
SCRIPT = Path(__file__).resolve()
EXECUTION_ID = "s5a1_primary_bsmooth_b01_20260811T204651Z_v4"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
PACKAGE_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v1.json"
PACKAGE_SCHEMA = ROOT / "schema/target_host_s5a1_handoff_package_v1.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v4.json"
S5A0_INNER_SCHEMA = ROOT / "schema/target_host_s5a0_selected_bag_receipt_v2.json"
S5A0_OUTER_SCHEMA = ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v5.json"
S5A0_SUPERVISOR_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v5.json"
GATE_V1 = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v1.py"
GATE_V2 = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v2.py"
GATE_V3 = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v3.py"
EXTRACTOR_V1 = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v1.py"
EXTRACTOR_V2 = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v2.py"
EXTRACTOR_V3 = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v3.py"
BRIDGE_V1 = ROOT / "scripts/r8_liquid_s5a1_motion_bridge_v1.py"
READERS = tuple(ROOT / f"scripts/r8_liquid_ros1_bag_v2_reader_v{revision}.py" for revision in range(1, 5))
COMPAT_GATE = ROOT / "scripts/r8_liquid_s5_c1_compatibility_gate_v1.py"
SUPERVISOR_V1 = ROOT / "scripts/r8_liquid_s5a1_execution_supervisor_v1.py"
SUPERVISOR_V3 = ROOT / "scripts/r8_liquid_s5a1_execution_supervisor_v3.py"
RECEIPT_SCHEMA_V1 = ROOT / "schema/target_host_s5a1_execution_receipt_v1.json"
RECEIPT_SCHEMA_V3 = ROOT / "schema/target_host_s5a1_execution_receipt_v3.json"
PARTIAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial")
FINAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v4.json")
EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT.with_name(EXECUTION_RECEIPT.name + ".partial")
S5A0_FINAL_RECEIPT = Path(
    "/home/zrj/scout_liquid_lab/audits/s5a0_primary_bsmooth_b01_20260811T200154Z_v5.host.final.json"
)
S5A0_EVIDENCE_ROOT = Path(
    "/home/zrj/scout_liquid_lab/audits/s5a0_primary_bsmooth_b01_20260811T200154Z_v5.host.evidence"
)
S5A0_INNER_RECEIPT = S5A0_EVIDENCE_ROOT / "selected_bag_inner_receipt_v2.json"
S5A0_FINAL_SHA256 = "709ccd9e2e5da97a4d7d71291c566d9c90d8e4f0499ef35c838f55cca0b8cda1"
S5A0_INNER_SHA256 = "ae294e881608f38e7dae60dccf1492a49e9863b029e46fe3935276f58ad2fc54"

# Gate v3 is finalized by its owning task before v4 self-check is accepted.
GATE_V3_SHA256 = "8d3b665358e9d9287731778a8ee20060067bf8f27becda9d0bace42e709fe0b5"
EXTRACTOR_V3_SHA256 = "c9671c9b138481dc413b8d26b943c66960c52a62f94adc53aab3aa021e386922"

FROZEN_PARENTS: dict[str, tuple[Path, str]] = {
    "package_policy": (PACKAGE_POLICY, "1ba40330581c3736442dcd6191d19f798cef5d658d092af6026de924e4b93c2c"),
    "package_schema": (PACKAGE_SCHEMA, "4e1c10d7bd721e072def81dfd491c1d6d72e2aa45e4b2e8e773a1f69dc471496"),
    "handoff_gate_v1": (GATE_V1, "7556f7aeebd9a358299c485ad232f8f6263990a437bba9c9ec06feb0d1fc9fd6"),
    "handoff_gate_v2": (GATE_V2, "d2922c381e7d2fabbadb09337be60cc0478344ca4ccb41655381ec19a5b9a5a0"),
    "handoff_gate_v3": (GATE_V3, GATE_V3_SHA256),
    "signal_extractor_v1": (EXTRACTOR_V1, "2ae263a2efa9ebc68d1fceb22201e5297db3a96df26002597d0f0a6ba60ad4bf"),
    "signal_extractor_v2": (EXTRACTOR_V2, "9183fedfe3a28353ac8efd0592c69755bdabc29452fc319c46818e9a9552f161"),
    "signal_extractor_v3": (EXTRACTOR_V3, EXTRACTOR_V3_SHA256),
    "motion_bridge_v1": (BRIDGE_V1, "c840f7bb467d5145f55d2e9904b7b8b62781658818ae47f3e95bd139904e2aa1"),
    "reader_v1": (READERS[0], "bc3975ba446e22097c399e10a8f6c4e60b4f8e78c5c7d613394028daa76c94fc"),
    "reader_v2": (READERS[1], "7b2e4d773a94547af4eb8e609a715d6846828596545e8fa87ac87d63217db113"),
    "reader_v3": (READERS[2], "6b9e27e194c8315c7122096cfa18d522e56cc1a0e96b97f8b52270ef1f55c0ce"),
    "reader_v4": (READERS[3], "dfb4f075504eef25753752179a2d8cf8f5585f2fbeb9260254f9b411d77f1b7b"),
    "s5a0_inner_receipt_schema_v2": (S5A0_INNER_SCHEMA, "ec78b8e62d9e31fd7adedeaa8a46b10645fce45c7f9a4e80f23eb6f825310976"),
    "s5a0_outer_receipt_schema_v5": (S5A0_OUTER_SCHEMA, "636c03e4f0457cdbb0d56d0837a669a1efcb0a4574e41f243676b0a05a5371a2"),
    "s5a0_supervisor_policy_v5": (S5A0_SUPERVISOR_POLICY, "ee7e37e6adca9cfb0903ee5f09afd0334846bd9fbf6eee2a9ebabd6d57a74472"),
    "s5a0_outer_final_receipt": (S5A0_FINAL_RECEIPT, S5A0_FINAL_SHA256),
    "s5a0_inner_receipt": (S5A0_INNER_RECEIPT, S5A0_INNER_SHA256),
    "compatibility_gate_v1": (COMPAT_GATE, "8c924726550a85fc7d5d0bf7af03f7f92ca60218dbe3d97c75df4ea43defba4b"),
    "execution_supervisor_v1": (SUPERVISOR_V1, "2bc6e7241f8e5df8c45558beca8082f87b71ca81782b848e26333853a3f0ef20"),
    "execution_supervisor_v3": (SUPERVISOR_V3, "3f14d142d7f9e67521cd799c82a6c186a239adc25aabd0ec21c6ab26c989d3a0"),
    "execution_receipt_schema_v1": (RECEIPT_SCHEMA_V1, "f682f6300e55264c22a116e5060eff3869c1297b923a8ab32ca80ece14fccd6a"),
    "execution_receipt_schema_v3": (RECEIPT_SCHEMA_V3, "cfaed8f437d34468f085aa242f6ab60f45252d3353137f54c53cbdebe5046297"),
}

_BASE_BUILD_BWRAP = base.build_bwrap_argv
_BASE_LOAD_PACKAGE = base.load_package
_BASE_RESERVE_RECEIPT = base.reserve_receipt
_BASE_PUBLISH_RECEIPT = base.publish_reserved_receipt
_BASE_MOCK_FAILURE = base.mock_failure_receipt
_BASE_EXECUTE = base.execute_one_shot
_BASE_WORKER = base.worker
_SEMANTIC_REPORT: dict[str, Any] | None = None


class ExecutionV4Error(ValueError):
    """Frozen identity, sandbox or semantic publication contract failed."""


def _sha256_regular(path: Path, expected: str | None = None) -> str:
    raw, identity = base.read_file_identity(path, expected, maximum_bytes=64 * 1024 * 1024)
    if identity["mode"] not in {"0440", "0600", "0644", "0664", "0755"}:
        raise ExecutionV4Error(f"frozen parent mode is not admitted: {path}")
    return hashlib.sha256(raw).hexdigest()


def _verify_frozen_parents() -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}
    for name, (path, expected) in FROZEN_PARENTS.items():
        observed = _sha256_regular(path, expected)
        result[name] = {"path": str(path), "sha256": observed}
    return result


def _parent_map() -> dict[str, dict[str, str]]:
    parents = _verify_frozen_parents()
    parents["execution_supervisor_v4"] = {
        "path": str(SCRIPT), "sha256": _sha256_regular(SCRIPT)
    }
    parents["execution_receipt_schema_v4"] = {
        "path": str(RECEIPT_SCHEMA), "sha256": _sha256_regular(RECEIPT_SCHEMA)
    }
    return parents


def build_bwrap_argv(
    receipt: Path, source: Path, partial: Path, *, receipt_fd: int | None = None,
    source_fd: int | None = None,
) -> list[str]:
    argv = _BASE_BUILD_BWRAP(
        receipt, source, partial, receipt_fd=receipt_fd, source_fd=source_fd
    )
    clear_index = argv.index("--clearenv") + 1
    argv[clear_index:clear_index] = [
        "--cap-drop", "ALL", "--uid", "1000", "--gid", "1000", "--unshare-net",
    ]
    dependencies = (
        SUPERVISOR_V1, GATE_V1, GATE_V2, EXTRACTOR_V1, EXTRACTOR_V2,
        READERS[0], READERS[1], READERS[2], RECEIPT_SCHEMA_V1,
    )
    bind_index = argv.index("--bind")
    bindings: list[str] = []
    for path in dependencies:
        if str(path) not in argv:
            bindings.extend(("--ro-bind", str(path), str(path)))
    argv[bind_index:bind_index] = bindings
    if argv.count("--bind") != 1:
        raise ExecutionV4Error("partial root is not the unique writable bind")
    writable_index = argv.index("--bind")
    if argv[writable_index + 1:writable_index + 3] != [str(PARTIAL_ROOT), str(PARTIAL_ROOT)]:
        raise ExecutionV4Error("writable bind is not the exact partial root")
    if argv.count("--cap-drop") != 1 or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise ExecutionV4Error("capability drop differs")
    if argv.count("--uid") != 1 or argv[argv.index("--uid") + 1] != "1000":
        raise ExecutionV4Error("sandbox uid differs")
    if argv.count("--gid") != 1 or argv[argv.index("--gid") + 1] != "1000":
        raise ExecutionV4Error("sandbox gid differs")
    if argv.count("--unshare-net") != 1:
        raise ExecutionV4Error("explicit network namespace differs")
    if argv[argv.index("--setenv") + 1:argv.index("--setenv") + 3] != ["PATH", "/usr/bin"]:
        raise ExecutionV4Error("sandbox PATH differs")
    joined = "\0".join(argv).lower()
    for forbidden in ("sudo", "/dev/nvidia", "nvidia-smi", "roscore", "dualsphysics5.4_linux64"):
        if forbidden in joined:
            raise ExecutionV4Error(f"forbidden sandbox token: {forbidden}")
    if "--proc" in argv or "--dev" in argv:
        raise ExecutionV4Error("proc/dev mount token is present")
    return argv


def _semantic_report_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")).encode()


def load_package(root: Path) -> dict[str, bytes]:
    """Run structural loading, then the public strong directory validator."""

    global _SEMANTIC_REPORT
    package = _BASE_LOAD_PACKAGE(root)
    report = gate_v3.validate_package_root(
        root, trusted_policy_path=PACKAGE_POLICY
    )
    if (
        not isinstance(report, dict)
        or report.get("status") != "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS"
        or report.get("package_root") != str(root)
        or report.get("source_bag_read") is not False
        or report.get("external_write") is not False
    ):
        raise ExecutionV4Error("strong semantic directory validator did not return exact PASS")
    _SEMANTIC_REPORT = dict(report)
    return package


def _semantic_receipt() -> dict[str, Any]:
    gate_identity = {"path": str(GATE_V3), "sha256": GATE_V3_SHA256}
    if _SEMANTIC_REPORT is None:
        return {
            "gate": gate_identity, "validator": "validate_package_root",
            "validated": False, "report_sha256": None,
            "status": "NOT_RUN_DUE_TO_FAILURE",
        }
    return {
        "gate": gate_identity, "validator": "validate_package_root",
        "validated": True,
        "report_sha256": hashlib.sha256(_semantic_report_bytes(_SEMANTIC_REPORT)).hexdigest(),
        "status": _SEMANTIC_REPORT["status"],
    }


def transform_receipt(value: Mapping[str, Any]) -> dict[str, Any]:
    transformed = {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v4",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V4",
        "execution_id": EXECUTION_ID,
        "receipt_id": f"{TRANSFER_ID}_execution_v4",
        "status": value["status"],
        "parents": _parent_map(),
        "execution": dict(value["execution"]),
        "inputs": value["inputs"],
        "roots": value["roots"],
        "package": value["package"],
        "semantic_validation": _semantic_receipt(),
        "compatibility": value["compatibility"],
        "safety": value["safety"],
        "failure": value["failure"],
        "next": value["next"],
    }
    transformed["execution"]["supplementary_groups"] = []
    if (
        transformed["status"] == "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
        and transformed["semantic_validation"]["validated"] is not True
    ):
        raise ExecutionV4Error("successful receipt lacks host strong semantic validation")
    return transformed


def reserve_receipt() -> int:
    return _BASE_RESERVE_RECEIPT(EXECUTION_RECEIPT_PARTIAL)


def publish_reserved_receipt(
    descriptor: int, partial_path: Path, final_path: Path,
    value: Mapping[str, Any],
) -> None:
    transformed = transform_receipt(value)
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator(schema).validate(transformed)
    _BASE_PUBLISH_RECEIPT(descriptor, partial_path, final_path, transformed)


def mock_failure_receipt() -> dict[str, Any]:
    return transform_receipt(_BASE_MOCK_FAILURE())


def _operational_expected() -> dict[Path, str]:
    names = (
        "package_policy", "package_schema", "handoff_gate_v3",
        "signal_extractor_v3", "motion_bridge_v1", "reader_v4",
        "s5a0_inner_receipt_schema_v2", "s5a0_outer_receipt_schema_v5",
        "s5a0_supervisor_policy_v5", "compatibility_gate_v1",
    )
    return {FROZEN_PARENTS[name][0]: FROZEN_PARENTS[name][1] for name in names}


def _install_revision() -> None:
    base.SCRIPT = SCRIPT
    base.POLICY = PACKAGE_POLICY
    base.PACKAGE_SCHEMA = PACKAGE_SCHEMA
    base.RECEIPT_SCHEMA = RECEIPT_SCHEMA
    base.S5A0_SCHEMA = S5A0_INNER_SCHEMA
    base.S5A0_FINAL_SCHEMA = S5A0_OUTER_SCHEMA
    base.S5A0_SUPERVISOR_POLICY = S5A0_SUPERVISOR_POLICY
    base.GATE = GATE_V3
    base.EXTRACTOR = EXTRACTOR_V3
    base.BRIDGE = BRIDGE_V1
    base.READER = READERS[3]
    base.COMPAT_GATE = COMPAT_GATE
    base.TRANSFER_ID = TRANSFER_ID
    base.PARTIAL_ROOT = PARTIAL_ROOT
    base.FINAL_ROOT = FINAL_ROOT
    base.EXECUTION_RECEIPT = EXECUTION_RECEIPT
    base.EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT_PARTIAL
    base.S5A0_FINAL_RECEIPT = S5A0_FINAL_RECEIPT
    base.S5A0_EVIDENCE_ROOT = S5A0_EVIDENCE_ROOT
    base.S5A0_INNER_RECEIPT = S5A0_INNER_RECEIPT
    base.EXPECTED = _operational_expected()
    base.gate = gate_v3
    base.extractor = extractor_v3
    base.build_bwrap_argv = build_bwrap_argv
    base.load_package = load_package
    base.reserve_receipt = reserve_receipt
    base.publish_reserved_receipt = publish_reserved_receipt
    base.mock_failure_receipt = mock_failure_receipt


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000 or os.getgroups() != []:
        raise ExecutionV4Error("sandbox worker identity is not uid/gid 1000 with zero groups")
    _install_revision()
    return _BASE_WORKER(receipt_path, source, partial)


def static_self_check() -> dict[str, Any]:
    global _SEMANTIC_REPORT
    _SEMANTIC_REPORT = None
    parents = _parent_map()
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate_v3.assert_schema_deep_closed(schema)
    _install_revision()
    Draft202012Validator(schema).validate(mock_failure_receipt())
    gate_report = gate_v3.self_check()
    extractor_report = extractor_v3.self_check()
    argv = build_bwrap_argv(S5A0_INNER_RECEIPT, Path(
        "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
    ), PARTIAL_ROOT)
    writable = [
        (argv[index + 1], argv[index + 2])
        for index, token in enumerate(argv) if token == "--bind"
    ]
    return {
        "status": "PASS_S5A1_EXECUTION_SUPERVISOR_V4_STATIC_CONTRACT",
        "execution_id": EXECUTION_ID,
        "verified_parent_count": len(parents),
        "gate_v3_status": gate_report["status"],
        "extractor_v3_status": extractor_report["status"],
        "receipt_schema_sha256": _sha256_regular(RECEIPT_SCHEMA),
        "supervisor_sha256": _sha256_regular(SCRIPT),
        "argv_sha256": hashlib.sha256("\0".join(argv).encode()).hexdigest(),
        "writable_binds": writable,
        "worker_uid": 1000,
        "worker_gid": 1000,
        "worker_supplementary_groups": [],
        "source_parent_present": S5A0_FINAL_RECEIPT.is_file(),
        "transfer_roots_fresh": not any(os.path.lexists(path) for path in (
            PARTIAL_ROOT, FINAL_ROOT, EXECUTION_RECEIPT, EXECUTION_RECEIPT_PARTIAL,
        )),
        "real_bag_read": False,
        "optional_bag_read": False,
        "worker_run": False,
        "bwrap_run": False,
        "files_written": False,
        "network_used": False,
        "gpu_exposed": False,
        "ros_started": False,
        "solver_executed": False,
        "sudo_used": False,
    }


def execute_one_shot(receipt_path: Path, receipt_sha256: str) -> dict[str, Any]:
    global _SEMANTIC_REPORT
    _SEMANTIC_REPORT = None
    _install_revision()
    _BASE_EXECUTE(receipt_path, receipt_sha256)
    raw = EXECUTION_RECEIPT.read_bytes()
    value = json.loads(raw)
    Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check")
    run = sub.add_parser("run-one-shot")
    run.add_argument("--s5a0-receipt", type=Path, required=True)
    run.add_argument("--s5a0-receipt-sha256", required=True)
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
                raise ExecutionV4Error("exact execution authorization differs")
            result = execute_one_shot(args.s5a0_receipt, args.s5a0_receipt_sha256)
        else:
            result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as exc:
        print(json.dumps({
            "status": "FAIL_CLOSED", "execution_id": EXECUTION_ID,
            "error": str(exc),
        }, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
