#!/usr/bin/env python3
"""Fail-closed gate for one TCG ``-machine none`` QMP smoke.

``self-check`` never creates a namespace and never invokes QEMU.  The only
execution entry is ``run-smoke``.  The current admission deliberately keeps
that entry NO-GO until the ESM kernel update, reboot, new boot binding and a
fresh independent review are complete.

This module has no qemu-img, disk, firmware, KVM, guest-code, build, GenCase,
ROS, Gazebo or solver execution path.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hashlib
import json
import os
import resource
import select
import selectors
import signal
import socket
import stat
import struct
import subprocess
import sys
sys.dont_write_bytecode = True
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, BinaryIO, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from jsonschema.exceptions import SchemaError


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_qemu_binary_probe_gate as binary_gate  # noqa: E402
import r8_liquid_qemu_install_gate_v2 as install_gate  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


SCHEMA_VERSION = "smpcc-r8-liquid-qemu-machine-none-result-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_QEMU_MACHINE_NONE_RESULT"
POLICY_PATH = PACKAGE_ROOT / "config/sandbox/p0b_qemu_machine_none_admission_v1.json"
ADMISSION_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_machine_none_admission_v1.json"
ATTEMPT_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_machine_none_attempt_v1.json"
RECEIPT_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_machine_none_receipt_v1.json"
POLICY_RELATIVE = "config/sandbox/p0b_qemu_machine_none_admission_v1.json"
ADMISSION_SCHEMA_RELATIVE = "schema/qemu_machine_none_admission_v1.json"
ATTEMPT_SCHEMA_RELATIVE = "schema/qemu_machine_none_attempt_v1.json"
RECEIPT_SCHEMA_RELATIVE = "schema/qemu_machine_none_receipt_v1.json"
QEMU = Path("/usr/bin/qemu-system-x86_64")
SCRATCH_DIR = safety.APPROVED_ROOT / "scratch/vm"
LOCK_PATH = safety.APPROVED_ROOT / "locks"
AUDIT_DIR = safety.APPROVED_ROOT / "audits/sandbox"
RECEIPT_NAME = "qemu_machine_none_v1_boot_20260805T094926Z.json"
ATTEMPT_NAME = "qemu_machine_none_v1_boot_20260805T094926Z.attempt.json"
PREDECESSOR_RECEIPT = AUDIT_DIR / "qemu_binary_probe_v1_boot_20260805T094926Z.json"
PREDECESSOR_FILE_SHA256 = "011038cdd9b8d0a95bab52b2663443a24cfa93404e4122f59b93786c45cc5978"
PREDECESSOR_INNER_HASH = "f14772ebec5da5f0e3f3484984e5804575dfd4f573231070f182d63ea800d1a8"
EXPECTED_QEMU_SHA256 = binary_gate.EXPECTED_QEMU_SHA256
BOOT_ID_SHA256 = binary_gate.BOOT_ID_SHA256
UNREADABLE_PROCESS_ALLOWLIST = binary_gate.UNREADABLE_PROCESS_ALLOWLIST
EXPECTED_SERVICE = binary_gate.EXPECTED_SERVICE
EXPECTED_QEMU_PACKAGE = binary_gate.EXPECTED_QEMU_PACKAGE
EXACT_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"}
EXACT_QEMU_ARGV = (
    str(QEMU),
    "-no-user-config",
    "-machine", "none",
    "-accel", "tcg,thread=single",
    "-S",
    "-nodefaults",
    "-display", "none",
    "-monitor", "none",
    "-serial", "none",
    "-parallel", "none",
    "-nic", "none",
    "-sandbox", "on,obsolete=deny,elevateprivileges=deny,spawn=deny,resourcecontrol=deny",
    "-qmp", "stdio",
)
QMP_COMMANDS = (
    {"execute": "qmp_capabilities", "id": "capabilities"},
    {"execute": "query-kvm", "id": "kvm"},
    {"execute": "query-block", "id": "block"},
    {"execute": "query-status", "id": "status"},
    {"execute": "quit", "id": "quit"},
)
EXPECTED_QMP_GREETING = {
    "QMP": {
        "version": {
            "qemu": {"major": 4, "minor": 2, "micro": 1},
            "package": "Debian 1:4.2-3ubuntu6.30+esm3",
        },
        "capabilities": [],
    }
}
EXPECTED_QMP_PRE_QUIT_RESPONSES = (
    {"return": {}, "id": "capabilities"},
    {"return": {"enabled": False, "present": True}, "id": "kvm"},
    {"return": [], "id": "block"},
    {"return": {"running": False, "singlestep": False, "status": "prelaunch"}, "id": "status"},
)
EXPECTED_QMP_QUIT_RESPONSE = {"return": {}, "id": "quit"}
MAX_STDOUT = 16384
MAX_STDERR = 8192
MAX_QMP_LINE = 4096
MAX_QMP_FRAMES = 7
MAX_COMMAND_BYTES = 512
TIMEOUT_SECONDS = 8.0
TERM_GRACE_SECONDS = 1.0
MIN_MEM_AVAILABLE = 6 * 1024**3
PR_SET_NO_NEW_PRIVS = binary_gate.PR_SET_NO_NEW_PRIVS
PR_SET_PDEATHSIG = binary_gate.PR_SET_PDEATHSIG
PR_CAPBSET_DROP = 24
PR_CAP_AMBIENT = 47
PR_CAP_AMBIENT_CLEAR_ALL = 4
LINUX_CAPABILITY_VERSION_3 = 0x20080522
CLONE_NEWUSER = 0x10000000
CLONE_NEWNET = 0x40000000
SYS_PIDFD_OPEN_X86_64 = 434
SIOCGIFFLAGS = 0x8913
SIOCGIFADDR = 0x8915
IFF_UP = 0x1
MAX_JSON_BYTES = 1024 * 1024


class QemuMachineNoneError(RuntimeError):
    """Any ambiguity is a hard NO-GO."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise QemuMachineNoneError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def read_json_and_hash(path: Path) -> Tuple[Dict[str, Any], str]:
    try:
        return install_gate.read_json_and_hash(path)
    except install_gate.QemuInstallGateError as exc:
        raise QemuMachineNoneError(str(exc)) from exc


def validate_with_schema(instance: Mapping[str, Any], schema: Mapping[str, Any], name: str) -> None:
    try:
        install_gate.validate_with_schema(instance, schema, name)
    except install_gate.QemuInstallGateError as exc:
        raise QemuMachineNoneError(str(exc)) from exc


def _canonical_qmp_wire(command: Mapping[str, Any]) -> bytes:
    return json.dumps(
        command,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii") + b"\r\n"


def _implementation_evidence() -> Dict[str, str]:
    paths = {
        "gate_path": Path(__file__).resolve(),
        "binary_gate_path": Path(binary_gate.__file__).resolve(),
        "install_gate_path": Path(install_gate.__file__).resolve(),
        "safety_gate_path": Path(safety.__file__).resolve(),
    }
    evidence: Dict[str, str] = {}
    for key, path in paths.items():
        try:
            relative = path.relative_to(PACKAGE_ROOT)
        except ValueError as exc:
            raise QemuMachineNoneError(f"implementation escaped package root: {path}") from exc
        digest, _ = install_gate.file_hash_and_observation(path)
        evidence[key] = str(relative)
        evidence[key.replace("_path", "_sha256")] = digest
    return evidence


def validate_policy_semantics(policy: Mapping[str, Any]) -> None:
    require(policy["admission_scope"] == "EXACT_QEMU_TCG_MACHINE_NONE_QMP_SMOKE_ONLY", "policy scope differs")
    require(policy["development_only"] is True and policy["formal"] is False, "development status differs")
    require(policy["physical_primary_eligible"] is False, "physical-primary status differs")
    predecessor = policy["predecessor_binary_probe_receipt"]
    require(
        (predecessor["path"], predecessor["file_sha256"], predecessor["receipt_hash"])
        == (str(PREDECESSOR_RECEIPT), PREDECESSOR_FILE_SHA256, PREDECESSOR_INNER_HASH),
        "predecessor binary receipt binding differs",
    )
    require(predecessor["preserved"] is True and predecessor["execution_authorization_inherited"] is False, "predecessor semantics differ")
    require(
        policy["boot_session"]
        == {
            "boot_id_sha256": BOOT_ID_SHA256,
            "uid": 1000,
            "unreadable_process_allowlist_exact": [dict(item) for item in UNREADABLE_PROCESS_ALLOWLIST],
        },
        "boot-session binding differs",
    )
    kernel = policy["host_kernel"]
    require(
        kernel
        == {
            "observed_release_exact": "5.15.0-139-generic",
            "installed_hwe_meta_version": "5.15.0.139.149~20.04.1",
            "esm_candidate_hwe_meta_version": "5.15.0.186.196~20.04.1",
            "namespace_security_update_required": True,
            "live_namespace_or_qemu_execution_authorized": False,
            "new_policy_and_boot_binding_required_after_update": True,
        },
        "kernel NO-GO binding differs",
    )
    binary = policy["binary"]
    require(tuple(binary["argv_exact"]) == EXACT_QEMU_ARGV, "QEMU argv differs")
    require(
        {key: binary[key] for key in ("path", "dpkg_package", "dpkg_version", "sha256", "size", "mode", "uid", "gid", "security_capabilities")}
        == {
            "path": str(QEMU), "dpkg_package": "qemu-system-x86",
            "dpkg_version": "1:4.2-3ubuntu6.30+esm3", "sha256": EXPECTED_QEMU_SHA256,
            "size": 16287496, "mode": "0755", "uid": 0, "gid": 0,
            "security_capabilities": [],
        },
        "QEMU binary identity differs",
    )
    forbidden_argv = {
        "-drive", "-blockdev", "-device", "-bios", "-kernel", "-initrd",
        "-option-rom", "-netdev", "-enable-kvm", "-daemonize", "-chardev",
    }
    require(not forbidden_argv.intersection(EXACT_QEMU_ARGV), "QEMU argv contains a forbidden option")
    require("/dev/kvm" not in "\0".join(EXACT_QEMU_ARGV), "QEMU argv references KVM")
    namespace = policy["namespace_boundary"]
    require(namespace["host_runtime_uid"] == 1000 and namespace["host_runtime_gid"] == 1000, "namespace host identity differs")
    require(namespace["unshare_syscall_only"] is True and namespace["external_namespace_helper_executed"] is False, "namespace helper policy differs")
    require(namespace["clone_newuser"] is True and namespace["clone_newnet"] is True, "required namespaces differ")
    for key in ("clone_newns", "clone_newpid", "clone_newipc", "clone_newuts", "clone_newcgroup", "clone_newtime"):
        require(namespace[key] is False, f"unexpected namespace admitted: {key}")
    require(namespace["uid_map_exact"] == "1000 1000 1\n" and namespace["gid_map_exact"] == "1000 1000 1\n", "namespace ID maps differ")
    require(namespace["setgroups_exact"] == "deny\n", "setgroups policy differs")
    require(namespace["persistent_namespace"] is False and namespace["persistent_bind_mount"] is False, "persistent namespace admitted")
    require(namespace["namespace_fds_pinned_until_child_reap"] is False and namespace["namespace_fds_closed_before_exec_ack"] is True, "namespace FD lifetime differs")
    require(namespace["pidfd_pinned_until_child_reap"] is True, "pidfd lifetime differs")
    require(namespace["setns_allowed"] is False, "setns is admitted")
    require(namespace["two_stage_parent_ack_required"] is True, "two-stage parent ACK is not required")
    require(namespace["supplementary_groups_empty_before_exec"] is True, "supplementary groups are not required empty")
    require(namespace["capability_sets_zero_before_exec"] is True, "capability sets are not required empty")
    require(namespace["interface_names_exact"] == ["lo"] and namespace["loopback_up"] is False, "isolated interface contract differs")
    require(namespace["loopback_ipv4_configured"] is False and namespace["loopback_ipv6_configured"] is False, "loopback address contract differs")
    require(namespace["ipv4_routes_must_be_empty"] is True and namespace["ipv6_routes_must_be_empty"] is True, "route contract differs")
    require(namespace["inet_sockets_must_be_empty"] is True, "socket contract differs")
    qmp = policy["qmp_contract"]
    require(qmp["commands_exact"] == [dict(item) for item in QMP_COMMANDS], "QMP command sequence differs")
    require(qmp["expected_kvm"] == {"enabled": False, "present": True}, "QMP KVM expectation differs")
    require(qmp["expected_block"] == [], "QMP block expectation differs")
    require(qmp["expected_status"] == {"running": False, "singlestep": False, "status": "prelaunch"}, "QMP status expectation differs")
    require(qmp["greeting_version_exact"] == {"major": 4, "minor": 2, "micro": 1}, "QMP version expectation differs")
    require(qmp["greeting_package_exact"] == "Debian 1:4.2-3ubuntu6.30+esm3", "QMP package expectation differs")
    require(qmp["events_before_quit_allowed"] is False, "pre-quit QMP events are admitted")
    require(qmp["shutdown_event_after_quit_allowed_exact"] == {"guest": False, "reason": "host-qmp-quit"}, "post-quit event policy differs")
    require(qmp["quit_response_may_be_absent_on_clean_eof"] is True and qmp["maximum_frames"] == MAX_QMP_FRAMES, "QMP quit/frame policy differs")
    require(qmp["command_wire_format"] == "canonical-json-crlf" and qmp["command_total_max_bytes"] == MAX_COMMAND_BYTES, "QMP wire policy differs")
    require(qmp["clean_quit_required"] is True, "clean QMP quit is not required")
    boundary = policy["execution_boundary"]
    require(boundary["self_check_executes_qemu"] is False and boundary["self_check_creates_namespaces"] is False, "self-check boundary differs")
    require(boundary["run_smoke_is_only_execution_entry"] is True, "execution entry differs")
    require(boundary["cwd"] == str(SCRATCH_DIR) and boundary["environment_exact"] == EXACT_ENV, "execution environment differs")
    require(boundary["stdout_max_bytes"] == MAX_STDOUT and boundary["stderr_max_bytes"] == MAX_STDERR, "output limits differ")
    require(boundary["qmp_line_max_bytes"] == MAX_QMP_LINE and boundary["timeout_seconds"] == 8, "QMP limits differ")
    require(boundary["pidfd_open_required"] is True, "pidfd ownership is not required")
    require(boundary["parent_publication_umask"] == "0027", "parent publication umask differs")
    require(
        boundary["rlimits"]
        == {
            "core_bytes": 0,
            "cpu_seconds_soft": 3,
            "cpu_seconds_hard": 4,
            "file_bytes": 0,
            "open_files": 64,
            "address_space_bytes": 1073741824,
            "memlock_bytes": 0,
        },
        "resource limits differ",
    )
    require(boundary["same_open_fd_hash_and_exec"] is True and boundary["exec_path_template"] == "/proc/self/fd/{fd}", "FD execution binding differs")
    require(boundary["snapshot_sequence"] == ["A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP"], "snapshot sequence differs")
    require(boundary["minimum_mem_available_bytes"] == MIN_MEM_AVAILABLE, "memory threshold differs")
    require(all(value is True for value in policy["admitted_effects"].values()), "admitted effect is not explicit")
    require(all(value is False for value in policy["forbidden_effects"].values()), "policy admits a forbidden effect")
    receipt = policy["receipt"]
    require(receipt["directory"] == str(AUDIT_DIR) and receipt["filename_exact"] == RECEIPT_NAME, "receipt path policy differs")
    require(receipt["attempt_marker_filename_exact"] == ATTEMPT_NAME, "attempt marker policy differs")
    require(receipt["attempt_marker_schema_relative"] == ATTEMPT_SCHEMA_RELATIVE, "attempt marker schema policy differs")
    require(receipt["attempt_marker_create_new_and_fsync_before_fork"] is True, "attempt marker durability differs")
    require(receipt["attempt_marker_consumes_any_namespace_or_qemu_attempt"] is True, "attempt reuse is admitted")
    require(receipt["attempt_marker_never_auto_deleted"] is True, "attempt marker cleanup is admitted")
    require(receipt["create_new"] is True and receipt["maximum_successful_receipts"] == 1, "receipt singleton policy differs")
    require(receipt["pass_only"] is True and receipt["failure_receipt_forbidden"] is True, "failure receipt is admitted")
    require(receipt["future_execution_authorized"] is False and receipt["authorization_reusable"] is False, "future authority is admitted")


def load_contract() -> Tuple[Dict[str, Any], str, Dict[str, Any], str, Dict[str, Any], str, Dict[str, Any], str]:
    policy, policy_hash = read_json_and_hash(POLICY_PATH)
    admission, admission_hash = read_json_and_hash(ADMISSION_SCHEMA_PATH)
    attempt_schema, attempt_schema_hash = read_json_and_hash(ATTEMPT_SCHEMA_PATH)
    receipt_schema, receipt_schema_hash = read_json_and_hash(RECEIPT_SCHEMA_PATH)
    install_gate.Draft202012Validator.check_schema(admission)
    install_gate.Draft202012Validator.check_schema(attempt_schema)
    install_gate.Draft202012Validator.check_schema(receipt_schema)
    validate_with_schema(policy, admission, "machine-none admission")
    validate_policy_semantics(policy)
    return policy, policy_hash, admission, admission_hash, attempt_schema, attempt_schema_hash, receipt_schema, receipt_schema_hash


def verify_predecessor(policy: Mapping[str, Any]) -> Dict[str, Any]:
    expected = policy["predecessor_binary_probe_receipt"]
    semantic_valid = False
    try:
        receipt, file_hash = read_json_and_hash(Path(expected["path"]))
        inner = receipt.get("receipt_hash")
        core = dict(receipt)
        core.pop("receipt_hash", None)
        self_hash_valid = isinstance(inner, str) and inner == safety.canonical_hash(core)
        old_policy, _, _, _, old_receipt_schema, _ = binary_gate.load_contract()
        binary_gate.validate_with_schema(receipt, old_receipt_schema, "predecessor binary receipt")
        binary_gate.validate_report_semantics(receipt, old_policy, receipt=True)
        semantic_valid = True
    except (OSError, ValueError, QemuMachineNoneError, binary_gate.QemuBinaryProbeError, install_gate.QemuInstallGateError):
        file_hash, inner, self_hash_valid = None, None, False
    return {
        "path": expected["path"],
        "expected_file_sha256": expected["file_sha256"],
        "actual_file_sha256": file_hash,
        "expected_receipt_hash": expected["receipt_hash"],
        "actual_receipt_hash": inner,
        "self_hash_valid": self_hash_valid,
        "semantic_valid": semantic_valid,
    }


def predecessor_errors(observation: Mapping[str, Any]) -> List[str]:
    errors = []
    if observation["actual_file_sha256"] != observation["expected_file_sha256"]:
        errors.append("predecessor binary receipt file hash differs")
    if observation["actual_receipt_hash"] != observation["expected_receipt_hash"]:
        errors.append("predecessor binary receipt inner hash differs")
    if observation["self_hash_valid"] is not True:
        errors.append("predecessor binary receipt self-hash is invalid")
    if observation["semantic_valid"] is not True:
        errors.append("predecessor binary receipt semantic validation failed")
    return errors


def _read_small_text(path: Path, limit: int = 65536) -> str:
    payload = path.read_bytes()
    require(len(payload) <= limit, f"kernel metadata is too large: {path}")
    return payload.decode("ascii", errors="strict")


def _host_kernel_observation() -> Dict[str, Any]:
    return {
        "release": os.uname().release,
        "unprivileged_userns_clone": _read_small_text(Path("/proc/sys/kernel/unprivileged_userns_clone"), 32).strip(),
        "max_user_namespaces": int(_read_small_text(Path("/proc/sys/user/max_user_namespaces"), 64).strip()),
    }


def _host_namespace_observation() -> Dict[str, str]:
    return {
        "user": os.readlink("/proc/self/ns/user"),
        "network": os.readlink("/proc/self/ns/net"),
    }


def collect_snapshot(policy: Mapping[str, Any], phase: str) -> Dict[str, Any]:
    binary = binary_gate.binary_observation(policy)
    processes = binary_gate.process_snapshot(binary)
    storage_mount, affecting_mounts = safety._mountinfo_for_root(safety.APPROVED_ROOT)
    storage_findmnt = safety._findmnt_snapshot(safety.STORAGE_ANCHOR)
    return {
        "phase": phase,
        "boot_id_sha256": binary_gate._boot_hash(),
        "runtime_uid": os.getuid(),
        "runtime_gid": os.getgid(),
        "runtime_thread_count": binary_gate._thread_count(),
        "mem_available_bytes": binary_gate._mem_available(),
        "qemu_kvm_service": install_gate.service_observation(),
        "autostart_link_exists": install_gate.QEMU_KVM_AUTOSTART_LINK.exists(),
        "autostart_link_is_symlink": install_gate.QEMU_KVM_AUTOSTART_LINK.is_symlink(),
        "ksm_value": install_gate.read_ksm(),
        "simulation_ports": binary_gate._simulation_ports(),
        "package_state": binary_gate._package_state(),
        "scratch": binary_gate._directory_observation(SCRATCH_DIR),
        "lock": binary_gate._directory_observation(LOCK_PATH),
        "host_kernel": _host_kernel_observation(),
        "host_namespaces": _host_namespace_observation(),
        "storage_mount": storage_mount,
        "storage_findmnt": storage_findmnt,
        "storage_anchor_identity": safety._path_identity(safety.STORAGE_ANCHOR),
        "approved_root_identity": safety._path_identity(safety.APPROVED_ROOT),
        "audit_dir_identity": safety._path_identity(AUDIT_DIR),
        "mount_points_affecting_root": list(affecting_mounts),
        "implementation": _implementation_evidence(),
        **processes,
        "binary": binary,
    }


def snapshot_errors(snapshot: Mapping[str, Any], policy: Mapping[str, Any]) -> List[str]:
    errors: List[str] = []
    if snapshot["boot_id_sha256"] != policy["boot_session"]["boot_id_sha256"]:
        errors.append("boot session identity differs")
    if (snapshot["runtime_uid"], snapshot["runtime_gid"]) != (1000, 1000):
        errors.append("runtime UID/GID differs")
    if snapshot["runtime_thread_count"] != 1:
        errors.append("machine-none gate process is not single-threaded")
    if snapshot["mem_available_bytes"] < policy["execution_boundary"]["minimum_mem_available_bytes"]:
        errors.append("MemAvailable is below 6 GiB")
    if snapshot["qemu_kvm_service"] != EXPECTED_SERVICE:
        errors.append("qemu-kvm service is not disabled/inactive")
    if snapshot["autostart_link_exists"] or snapshot["autostart_link_is_symlink"]:
        errors.append("qemu-kvm autostart link exists")
    if snapshot["ksm_value"] != "0":
        errors.append("KSM is not disabled")
    if snapshot["qemu_processes"]:
        errors.append("QEMU process set is not empty")
    if snapshot["simulation_processes"]:
        errors.append("ROS/Gazebo process set is not empty")
    if snapshot["simulation_ports"]:
        errors.append("ROS/Gazebo reserved port set is not empty")
    if snapshot["process_visibility_errors"]:
        errors.append("process visibility is incomplete")
    if snapshot["process_visibility_exceptions"] != [dict(item) for item in UNREADABLE_PROCESS_ALLOWLIST]:
        errors.append("boot-session process exception set differs")
    if snapshot["implementation"] != _implementation_evidence():
        errors.append("gate implementation or dependency identity differs")
    expected = policy["binary"]
    for field in ("path", "uid", "gid", "mode", "size", "sha256", "security_capabilities"):
        if snapshot["binary"][field] != expected[field]:
            errors.append(f"QEMU binary identity differs: {field}")
    package = snapshot["package_state"]
    if package["required_package"] != EXPECTED_QEMU_PACKAGE:
        errors.append("QEMU package identity differs")
    if package["qemu_namespace_installed"] != list(install_gate.EXPECTED_QEMU_NAMESPACE):
        errors.append("installed QEMU namespace differs")
    if package["dpkg_verify"] != {"returncode": 0, "stdout_empty": True, "stderr_empty": True}:
        errors.append("dpkg verification is not clean")
    if package["binary_dpkg_owner"] != "qemu-system-x86" or package["binary_diverted"] is not False:
        errors.append("QEMU binary package ownership differs")
    for key, path in (("scratch", SCRATCH_DIR), ("lock", LOCK_PATH)):
        observed = snapshot[key]
        if (
            observed["path"] != str(path)
            or observed["uid"] != 1000
            or observed["gid"] != 1000
            or observed["mode"] != "0750"
            or observed["entries"] != []
        ):
            errors.append(f"{key} directory is not empty and frozen")
    kernel = snapshot["host_kernel"]
    if kernel["release"] != policy["host_kernel"]["observed_release_exact"]:
        errors.append("running kernel differs from the frozen pre-update kernel")
    if kernel["unprivileged_userns_clone"] != "1" or kernel["max_user_namespaces"] <= 0:
        errors.append("unprivileged user namespace kernel controls are unavailable")
    if policy["host_kernel"]["namespace_security_update_required"] is True:
        errors.append("ESM kernel security update and reboot are required before namespace execution")
    if policy["host_kernel"]["live_namespace_or_qemu_execution_authorized"] is not True:
        errors.append("this current-boot policy does not authorize live namespace or QEMU execution")
    try:
        safety._require_expected_storage_identity(
            snapshot["storage_mount"],
            "machine-none snapshot",
            keys=("target", "source", "fstype", "major_minor", "fsroot"),
        )
    except safety.LiquidSafetyError:
        errors.append("fixed /data mount identity differs")
    try:
        safety._require_expected_storage_identity(snapshot["storage_findmnt"], "machine-none findmnt snapshot")
    except safety.LiquidSafetyError:
        errors.append("fixed /data UUID or findmnt identity differs")
    root_identity = snapshot["approved_root_identity"]
    anchor_identity = snapshot["storage_anchor_identity"]
    audit_identity = snapshot["audit_dir_identity"]
    if (
        root_identity["path"] != str(safety.APPROVED_ROOT)
        or root_identity["resolved_path"] != str(safety.APPROVED_ROOT)
        or root_identity["uid"] != 1000
        or root_identity["gid"] != 1000
        or root_identity["mode"] != "0750"
        or root_identity["is_symlink"] is not False
    ):
        errors.append("approved root identity differs")
    if (
        anchor_identity["path"] != str(safety.STORAGE_ANCHOR)
        or anchor_identity["resolved_path"] != str(safety.STORAGE_ANCHOR)
        or anchor_identity["uid"] != 1000
        or anchor_identity["gid"] != 1000
        or anchor_identity["mode"] != "0755"
        or anchor_identity["is_symlink"] is not False
    ):
        errors.append("fixed /data/a storage anchor identity differs")
    if (
        audit_identity["path"] != str(AUDIT_DIR)
        or audit_identity["resolved_path"] != str(AUDIT_DIR)
        or audit_identity["uid"] != 1000
        or audit_identity["gid"] != 1000
        or audit_identity["mode"] != "0750"
        or audit_identity["is_symlink"] is not False
    ):
        errors.append("audit directory identity differs")
    if not (anchor_identity["device_id"] == root_identity["device_id"] == audit_identity["device_id"]):
        errors.append("storage anchor/root/audit device identity differs")
    if snapshot["mount_points_affecting_root"]:
        errors.append("an unexpected mount affects the approved root")
    return sorted(set(errors))


def stable_snapshot_projection(snapshot: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        key: snapshot[key]
        for key in (
            "boot_id_sha256", "runtime_uid", "runtime_gid", "runtime_thread_count",
            "qemu_kvm_service", "autostart_link_exists", "autostart_link_is_symlink",
            "ksm_value", "qemu_processes", "simulation_processes", "simulation_ports",
            "process_visibility_errors", "process_visibility_exceptions", "package_state",
            "scratch", "lock", "host_kernel", "host_namespaces", "storage_mount", "storage_findmnt",
            "storage_anchor_identity", "approved_root_identity", "audit_dir_identity",
            "mount_points_affecting_root", "implementation", "binary",
        )
    }


def validate_runtime_paths() -> None:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    require(SCRATCH_DIR == root / "scratch/vm" and LOCK_PATH == root / "locks", "runtime paths escaped approved root")
    require(AUDIT_DIR == root / "audits/sandbox", "audit path escaped approved root")
    for directory in (SCRATCH_DIR, LOCK_PATH, AUDIT_DIR):
        fd = safety.open_directory_nofollow(directory)
        os.close(fd)


def _snapshot_record(snapshot: Mapping[str, Any], policy: Mapping[str, Any]) -> Dict[str, Any]:
    canonical = safety.canonical_bytes(snapshot).decode("utf-8")
    return {
        "phase": snapshot["phase"],
        "sha256": safety.canonical_hash(snapshot),
        "canonical": canonical,
        "invariants_pass": not snapshot_errors(snapshot, policy),
    }


def _base_report(
    policy: Mapping[str, Any],
    policy_hash: str,
    admission_hash: str,
    attempt_schema_hash: str,
    receipt_schema_hash: str,
    predecessor: Mapping[str, Any],
    snapshots: Sequence[Mapping[str, Any]],
    attempt: Optional[Mapping[str, Any]],
    namespace: Optional[Mapping[str, Any]],
    smoke: Optional[Mapping[str, Any]],
    errors: Sequence[str],
    status: str,
) -> Dict[str, Any]:
    stable_hashes = [safety.canonical_hash(stable_snapshot_projection(item)) for item in snapshots]
    executed = smoke is not None
    effects = {
        "qemu_executed": executed,
        "qemu_main_loop_entered": executed,
        "machine_type_none_selected": executed,
        "tcg_selected": executed,
        "user_namespace_created": namespace is not None,
        "network_namespace_created": namespace is not None,
        "qmp_commands_executed": executed,
        "qemu_img_executed": False,
        "guest_code_executed": False,
        "vcpu_ran": False,
        "vm_started": False,
        "kvm_enabled": False,
        "network_backend_configured": False,
        "persistent_namespace_created": False,
        "filesystem_sandbox_claimed": False,
        "image_created": False,
        "block_backend_configured": False,
        "firmware_configured": False,
        "host_mount_configured": False,
        "build_started": False,
        "gencase_started": False,
        "upstream_code_executed": False,
        "ros_or_gazebo_started": False,
    }
    core: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "policy": {
            "path": POLICY_RELATIVE,
            "sha256": policy_hash,
            "admission_schema_path": ADMISSION_SCHEMA_RELATIVE,
            "admission_schema_sha256": admission_hash,
            "attempt_schema_path": ATTEMPT_SCHEMA_RELATIVE,
            "attempt_schema_sha256": attempt_schema_hash,
            "receipt_schema_path": RECEIPT_SCHEMA_RELATIVE,
            "receipt_schema_sha256": receipt_schema_hash,
        },
        "implementation": dict(snapshots[0]["implementation"]),
        "predecessor_binary_probe_receipt": dict(predecessor),
        "attempt_marker": None if attempt is None else dict(attempt),
        "snapshot_sequence": [_snapshot_record(item, policy) for item in snapshots],
        "stability": {
            "stable_projection_sha256": stable_hashes,
            "ab_equal": len(stable_hashes) >= 2 and stable_hashes[0] == stable_hashes[1],
            "bc_equal": len(stable_hashes) >= 3 and stable_hashes[1] == stable_hashes[2],
        },
        "namespace_result": None if namespace is None else dict(namespace),
        "smoke_result": None if smoke is None else dict(smoke),
        "effects": effects,
        "execution_was_explicitly_requested": executed,
        "future_execution_authorized": False,
        "authorization_reusable": False,
        "status": status,
        "errors": list(errors),
    }
    return dict(core, receipt_hash=safety.canonical_hash(core))


def build_self_check() -> Dict[str, Any]:
    validate_runtime_paths()
    policy, policy_hash, _, admission_hash, _, attempt_schema_hash, _, receipt_schema_hash = load_contract()
    predecessor = verify_predecessor(policy)
    snapshots = [collect_snapshot(policy, phase) for phase in policy["execution_boundary"]["snapshot_sequence"]]
    errors = predecessor_errors(predecessor)
    for snapshot in snapshots:
        errors.extend(f"{snapshot['phase']}: {item}" for item in snapshot_errors(snapshot, policy))
    stable_hashes = [safety.canonical_hash(stable_snapshot_projection(item)) for item in snapshots]
    if len(set(stable_hashes)) != 1:
        errors.append("runtime state drifted across read-only A/B/C collection")
    residue = _residue()
    if residue:
        errors.append(f"one-shot machine-none residue exists; retry forbidden: {residue}")
    if ATTEMPT_NAME in residue and RECEIPT_NAME not in residue:
        status = "ATTEMPT_CONSUMED_NO_OUTCOME"
    elif RECEIPT_NAME in residue:
        status = "NO_GO_ALREADY_COMPLETED"
    else:
        status = "PASS_READY_EXECUTION_NOT_PERFORMED" if not errors else "NO_GO"
    return _base_report(policy, policy_hash, admission_hash, attempt_schema_hash, receipt_schema_hash, predecessor, snapshots, None, None, None, errors, status)


def _normalize_id_map(payload: str) -> str:
    rows = []
    for line in payload.splitlines():
        if not line.strip():
            continue
        fields = line.split()
        require(len(fields) == 3 and all(field.isdigit() for field in fields), "namespace ID map is malformed")
        rows.append(" ".join(fields))
    return "\n".join(rows) + ("\n" if rows else "")


def _namespace_inode(link: str, kind: str) -> int:
    prefix = f"{kind}:["
    require(link.startswith(prefix) and link.endswith("]"), f"{kind} namespace identity is malformed")
    value = link[len(prefix):-1]
    require(value.isdigit(), f"{kind} namespace inode is malformed")
    return int(value)


def _network_table_rows(path: Path) -> List[str]:
    lines = _read_small_text(path).splitlines()
    return [line.strip() for line in lines[1:] if line.strip()]


def _interface_names(path: Path) -> List[str]:
    lines = _read_small_text(path).splitlines()
    names = []
    for line in lines[2:]:
        if not line.strip():
            continue
        require(":" in line, "network interface table is malformed")
        names.append(line.split(":", 1)[0].strip())
    return sorted(names)


def _status_no_new_privs(path: Path) -> str:
    matches = []
    for line in _read_small_text(path).splitlines():
        if line.startswith("NoNewPrivs:"):
            matches.append(line.split(":", 1)[1].strip())
    require(len(matches) == 1, "child NoNewPrivs status is unavailable")
    return matches[0]


def _status_security(path: Path) -> Dict[str, Any]:
    wanted = {"Groups", "CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb", "NoNewPrivs", "Seccomp"}
    values: Dict[str, str] = {}
    for line in _read_small_text(path).splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key in wanted:
            require(key not in values, f"duplicate child status field: {key}")
            values[key] = value.strip()
    require(set(values) == wanted, "child security status is incomplete")
    groups = [] if not values["Groups"] else [int(item) for item in values["Groups"].split()]
    capabilities = {key: values[key] for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}
    require(all(len(value) == 16 and all(char in "0123456789abcdefABCDEF" for char in value) for value in capabilities.values()), "child capability encoding is malformed")
    return {
        "supplementary_groups": groups,
        "capabilities": {key: value.lower() for key, value in capabilities.items()},
        "no_new_privs": values["NoNewPrivs"],
        "seccomp": values["Seccomp"],
    }


def _pidfd_open(pid: int) -> int:
    libc = ctypes.CDLL(None, use_errno=True)
    fd = libc.syscall(SYS_PIDFD_OPEN_X86_64, pid, 0)
    if fd < 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    return int(fd)


def validate_namespace_evidence(observation: Mapping[str, Any], policy: Mapping[str, Any]) -> None:
    expected = policy["namespace_boundary"]
    require(observation["pid"] == observation["pgid"], "namespace child PID/PGID differs")
    require(observation["user_namespace_distinct"] is True and observation["network_namespace_distinct"] is True, "child namespaces are not isolated")
    require(_namespace_inode(observation["child_user_namespace"], "user") == observation["child_user_namespace_inode"], "pinned user namespace identity differs")
    require(_namespace_inode(observation["child_network_namespace"], "net") == observation["child_network_namespace_inode"], "pinned network namespace identity differs")
    require(observation["uid_map"] == expected["uid_map_exact"] and observation["gid_map"] == expected["gid_map_exact"], "child namespace ID maps differ")
    require(observation["setgroups"] == expected["setgroups_exact"], "child setgroups state differs")
    require(observation["no_new_privs"] == "1", "child no-new-privs is not armed")
    require(observation["supplementary_groups"] == [], "child supplementary groups are not empty")
    require(all(value == "0000000000000000" for value in observation["capabilities"].values()), "child capability sets are not empty")
    require(observation["interface_names"] == expected["interface_names_exact"], "isolated interface inventory differs")
    require(not observation["ipv4_routes"] and not observation["ipv6_routes"], "isolated namespace contains routes")
    require(not any(observation["inet_socket_rows"].values()), "isolated namespace contains INET sockets")
    require(observation["loopback_up"] is False, "isolated loopback is up")
    require(observation["loopback_ipv4_configured"] is False and observation["loopback_ipv6_configured"] is False, "isolated loopback has an address")
    require(observation["verified_before_exec"] is True, "namespace was not verified before exec")
    require(observation["namespace_fds_pinned_during_verification"] is True and observation["namespace_fds_closed_before_exec_ack"] is True, "namespace FD lifetime differs")
    require(observation["pidfd_pinned_until_child_reap"] is True, "pidfd lifetime differs")
    require(observation["persistent_bind_mount"] is False and observation["external_namespace_helper_executed"] is False, "persistent or helper namespace path was used")


def _namespace_observation(
    pid: int,
    expected_starttime: int,
    pidfd: int,
    child_claim: Mapping[str, Any],
    policy: Mapping[str, Any],
) -> Tuple[Dict[str, Any], Tuple[int, int]]:
    entry = Path("/proc") / str(pid)
    leader = binary_gate._pid_group_record(entry)
    require(leader is not None, "namespace child disappeared before verification")
    require(leader["starttime_ticks"] == expected_starttime, "namespace child PID was reused")
    require(leader["pgrp"] == pid, "namespace child PGID differs")
    require(os.fstat(pidfd).st_ino > 0, "pidfd identity is unavailable")
    # /proc/PID/ns/* are intentional procfs magic links.  Follow only these
    # two fixed names after binding PID/starttime/PGID and holding pidfd.
    user_fd = os.open(entry / "ns/user", os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
    try:
        net_fd = os.open(entry / "ns/net", os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
    except BaseException:
        os.close(user_fd)
        raise
    try:
        parent = _host_namespace_observation()
        child_user = os.readlink(entry / "ns/user")
        child_net = os.readlink(entry / "ns/net")
        user_stat = os.fstat(user_fd)
        net_stat = os.fstat(net_fd)
        uid_map = _normalize_id_map(_read_small_text(entry / "uid_map"))
        gid_map = _normalize_id_map(_read_small_text(entry / "gid_map"))
        setgroups = _read_small_text(entry / "setgroups", 32)
        interfaces = _interface_names(entry / "net/dev")
        ipv4_routes = _network_table_rows(entry / "net/route")
        ipv6_routes = [line.strip() for line in _read_small_text(entry / "net/ipv6_route").splitlines() if line.strip()]
        inet_socket_rows = {
            name: _network_table_rows(entry / f"net/{name}")
            for name in ("tcp", "tcp6", "udp", "udp6", "raw", "raw6")
        }
        security = _status_security(entry / "status")
        observation = {
            "pid": pid,
            "pgid": leader["pgrp"],
            "starttime_ticks": leader["starttime_ticks"],
            "pidfd_opened": True,
            "parent_user_namespace": parent["user"],
            "parent_network_namespace": parent["network"],
            "child_user_namespace": child_user,
            "child_network_namespace": child_net,
            "child_user_namespace_device": user_stat.st_dev,
            "child_user_namespace_inode": user_stat.st_ino,
            "child_network_namespace_device": net_stat.st_dev,
            "child_network_namespace_inode": net_stat.st_ino,
            "user_namespace_distinct": child_user != parent["user"],
            "network_namespace_distinct": child_net != parent["network"],
            "uid_map": uid_map,
            "gid_map": gid_map,
            "setgroups": setgroups,
            "supplementary_groups": security["supplementary_groups"],
            "capabilities": security["capabilities"],
            "no_new_privs": security["no_new_privs"],
            "interface_names": interfaces,
            "ipv4_routes": ipv4_routes,
            "ipv6_routes": ipv6_routes,
            "inet_socket_rows": inet_socket_rows,
            "loopback_up": bool(child_claim["loopback_up"]),
            "loopback_ipv4_configured": bool(child_claim["loopback_ipv4_configured"]),
            "loopback_ipv6_configured": bool(child_claim["loopback_ipv6_configured"]),
            "verified_before_exec": True,
            "namespace_fds_pinned_during_verification": True,
            "namespace_fds_closed_before_exec_ack": True,
            "pidfd_pinned_until_child_reap": True,
            "persistent_bind_mount": False,
            "external_namespace_helper_executed": False,
        }
        validate_namespace_evidence(observation, policy)
        leader_after = binary_gate._pid_group_record(entry)
        require(leader_after is not None and leader_after["starttime_ticks"] == expected_starttime and leader_after["pgrp"] == pid, "namespace child identity drifted during verification")
        return observation, (user_fd, net_fd)
    except BaseException:
        os.close(net_fd)
        os.close(user_fd)
        raise


def _write_proc_control(path: str, payload: bytes) -> None:
    fd = os.open(path, os.O_WRONLY | getattr(os, "O_CLOEXEC", 0))
    try:
        require(os.write(fd, payload) == len(payload), f"short write to {path}")
    finally:
        os.close(fd)


def _unshare(flags: int) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.unshare(flags) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


class _CapHeader(ctypes.Structure):
    _fields_ = [("version", ctypes.c_uint32), ("pid", ctypes.c_int)]


class _CapData(ctypes.Structure):
    _fields_ = [("effective", ctypes.c_uint32), ("permitted", ctypes.c_uint32), ("inheritable", ctypes.c_uint32)]


def _drop_all_capabilities() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    last_cap = int(_read_small_text(Path("/proc/sys/kernel/cap_last_cap"), 32).strip())
    require(0 <= last_cap <= 255, "kernel capability range is invalid")
    for capability in range(last_cap + 1):
        if libc.prctl(PR_CAPBSET_DROP, capability, 0, 0, 0) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))
    if libc.prctl(PR_CAP_AMBIENT, PR_CAP_AMBIENT_CLEAR_ALL, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        if error_number != errno.EINVAL:
            raise OSError(error_number, os.strerror(error_number))
    header = _CapHeader(LINUX_CAPABILITY_VERSION_3, 0)
    data = (_CapData * 2)()
    if libc.capset(ctypes.byref(header), ctypes.byref(data)) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def _child_loopback_claim() -> Dict[str, bool]:
    request = struct.pack("16sH14x", b"lo", 0)
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM | getattr(socket, "SOCK_CLOEXEC", 0))
    try:
        response = fcntl.ioctl(probe.fileno(), SIOCGIFFLAGS, request)
        flags = struct.unpack("16sH14x", response)[1]
        ipv4_configured = True
        try:
            fcntl.ioctl(probe.fileno(), SIOCGIFADDR, request)
        except OSError as exc:
            require(exc.errno in (errno.EADDRNOTAVAIL, errno.ENODEV, errno.EINVAL), "loopback IPv4 observation failed")
            ipv4_configured = False
    finally:
        probe.close()
    ipv6_configured = bool(_read_small_text(Path("/proc/net/if_inet6")).strip())
    return {
        "loopback_up": bool(flags & IFF_UP),
        "loopback_ipv4_configured": ipv4_configured,
        "loopback_ipv6_configured": ipv6_configured,
    }


def _child_namespace_setup(parent_pid: int, host_uid: int, host_gid: int) -> Dict[str, bool]:
    require((host_uid, host_gid) == (1000, 1000), "host UID/GID differs before namespace setup")
    _unshare(CLONE_NEWUSER)
    os.setgroups([])
    _write_proc_control("/proc/self/setgroups", b"deny\n")
    _write_proc_control("/proc/self/uid_map", f"{host_uid} {host_uid} 1\n".encode("ascii"))
    _write_proc_control("/proc/self/gid_map", f"{host_gid} {host_gid} 1\n".encode("ascii"))
    _unshare(CLONE_NEWNET)
    claim = _child_loopback_claim()
    _drop_all_capabilities()
    binary_gate._child_parent_death_guard(parent_pid)
    return claim


def _child_hardening() -> None:
    os.umask(0o077)
    limits = (
        (resource.RLIMIT_CORE, 0, 0),
        (resource.RLIMIT_CPU, 3, 4),
        (resource.RLIMIT_FSIZE, 0, 0),
        (resource.RLIMIT_NOFILE, 64, 64),
        (resource.RLIMIT_AS, 1073741824, 1073741824),
        (resource.RLIMIT_MEMLOCK, 0, 0),
    )
    for limit, soft, hard in limits:
        resource.setrlimit(limit, (soft, hard))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def _write_child_error(fd: int, error: BaseException) -> None:
    error_number = error.errno if isinstance(error, OSError) and error.errno else 255
    payload = f"E{int(error_number)}\n".encode("ascii", errors="strict")[:32]
    try:
        os.write(fd, payload)
    except OSError:
        pass


def _child_exec(
    parent_pid: int,
    host_uid: int,
    host_gid: int,
    qemu_fd: int,
    scratch_fd: int,
    qmp_read: int,
    stdout_write: int,
    stderr_write: int,
    exec_error_write: int,
    ready_write: int,
    ack_read: int,
) -> None:
    try:
        binary_gate._child_parent_death_guard(parent_pid)
        os.setsid()
        binary_gate._reset_child_signals()
        os.fchdir(scratch_fd)
        _child_hardening()
        require(os.write(ready_write, b"S\n") == 2, "short stage-one readiness write")
        require(os.read(ack_read, 1) == b"N", "parent did not authorize namespace setup")
        claim = _child_namespace_setup(parent_pid, host_uid, host_gid)
        ready_payload = b"R" + safety.canonical_bytes(claim) + b"\n"
        require(len(ready_payload) <= 1024, "namespace ready payload is too large")
        require(os.write(ready_write, ready_payload) == len(ready_payload), "short namespace ready write")
        require(os.read(ack_read, 1) == b"E", "parent did not authorize child exec")
        os.dup2(qmp_read, 0, inheritable=True)
        os.dup2(stdout_write, 1, inheritable=True)
        os.dup2(stderr_write, 2, inheritable=True)
        fcntl.fcntl(qemu_fd, fcntl.F_SETFD, fcntl.FD_CLOEXEC)
        binary_gate._close_child_fds((qemu_fd, exec_error_write))
        os.execve(f"/proc/self/fd/{qemu_fd}", list(EXACT_QEMU_ARGV), dict(EXACT_ENV))
    except BaseException as exc:
        _write_child_error(exec_error_write, exc)
        os._exit(127)


class MachineChild:
    def __init__(
        self,
        pid: int,
        qmp_write: int,
        stdout: BinaryIO,
        stderr: BinaryIO,
        exec_error: BinaryIO,
        identity: Mapping[str, int],
        deadline_monotonic: float,
        namespace: Mapping[str, Any],
        pinned_fds: Sequence[int],
    ) -> None:
        self.pid = pid
        self.qmp_write = qmp_write
        self.stdout = stdout
        self.stderr = stderr
        self.exec_error = exec_error
        self.identity = dict(identity)
        self.deadline_monotonic = deadline_monotonic
        self.namespace = dict(namespace)
        self.pinned_fds = list(pinned_fds)
        self.returncode: Optional[int] = None

    def wait(self, timeout: float) -> int:
        if self.returncode is not None:
            return self.returncode
        deadline = time.monotonic() + timeout
        while True:
            waited, status = os.waitpid(self.pid, os.WNOHANG)
            if waited == self.pid:
                if os.WIFEXITED(status):
                    self.returncode = os.WEXITSTATUS(status)
                elif os.WIFSIGNALED(status):
                    self.returncode = -os.WTERMSIG(status)
                else:
                    raise QemuMachineNoneError("machine-none child entered an unsupported wait state")
                return self.returncode
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(list(EXACT_QEMU_ARGV), timeout)
            time.sleep(0.005)

    def close_pipes(self) -> None:
        try:
            os.close(self.qmp_write)
        except OSError:
            pass
        for stream in (self.stdout, self.stderr, self.exec_error):
            try:
                stream.close()
            except OSError:
                pass

    def close_pinned_fds(self) -> None:
        for fd in self.pinned_fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self.pinned_fds.clear()


def _pipe_cloexec() -> Tuple[int, int]:
    read_fd, write_fd = os.pipe2(getattr(os, "O_CLOEXEC", 0))
    require(read_fd > 2 and write_fd > 2, "launcher pipe collided with standard descriptors")
    return read_fd, write_fd


def _read_ready_payload(fd: int, deadline: float) -> Dict[str, Any]:
    payload = bytearray()
    while b"\n" not in payload:
        remaining = deadline - time.monotonic()
        require(remaining > 0, "namespace child did not become ready before deadline")
        readable, _, _ = select.select([fd], [], [], remaining)
        require(bool(readable), "namespace child did not become ready before deadline")
        chunk = os.read(fd, 1025 - len(payload))
        require(bool(chunk), "namespace child closed readiness pipe")
        payload.extend(chunk)
        require(len(payload) <= 1024, "namespace readiness payload exceeded limit")
    require(payload.endswith(b"\n") and payload.count(b"\n") == 1 and payload.startswith(b"R"), "namespace readiness framing differs")
    try:
        claim = json.loads(
            bytes(payload[1:-1]).decode("utf-8", errors="strict"),
            object_pairs_hook=install_gate._reject_duplicate_keys,
            parse_constant=install_gate._reject_nonfinite,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, install_gate.QemuInstallGateError) as exc:
        raise QemuMachineNoneError("namespace readiness payload is invalid") from exc
    require(
        claim
        == {"loopback_up": False, "loopback_ipv4_configured": False, "loopback_ipv6_configured": False},
        "namespace child claims an unsafe loopback state",
    )
    return claim


def _read_stage_one(fd: int, deadline: float) -> None:
    remaining = deadline - time.monotonic()
    require(remaining > 0, "launcher deadline expired before stage-one readiness")
    readable, _, _ = select.select([fd], [], [], remaining)
    require(bool(readable), "child did not reach stage-one readiness")
    require(os.read(fd, 2) == b"S\n", "stage-one readiness framing differs")


def _spawn_machine_none(
    qemu_fd: int,
    scratch_fd: int,
    policy: Mapping[str, Any],
    pre_exec_check: Any,
) -> MachineChild:
    require(binary_gate._thread_count() == 1, "fork-based machine-none launcher requires one parent thread")
    require(qemu_fd > 2 and scratch_fd > 2 and qemu_fd != scratch_fd, "audited QEMU/scratch descriptors collided")
    parent_pid = os.getpid()
    host_uid, host_gid = os.getuid(), os.getgid()
    started_ns = time.monotonic_ns()
    deadline = time.monotonic() + TIMEOUT_SECONDS
    qmp_read, qmp_write = _pipe_cloexec()
    stdout_read, stdout_write = _pipe_cloexec()
    stderr_read, stderr_write = _pipe_cloexec()
    exec_error_read, exec_error_write = _pipe_cloexec()
    ready_read, ready_write = _pipe_cloexec()
    ack_read, ack_write = _pipe_cloexec()
    all_fds = (
        qmp_read, qmp_write, stdout_read, stdout_write, stderr_read, stderr_write,
        exec_error_read, exec_error_write, ready_read, ready_write, ack_read, ack_write,
    )
    try:
        pid = os.fork()
    except BaseException:
        for fd in all_fds:
            os.close(fd)
        raise
    if pid == 0:
        for fd in (qmp_write, stdout_read, stderr_read, exec_error_read, ready_read, ack_write):
            os.close(fd)
        _child_exec(
            parent_pid, host_uid, host_gid, qemu_fd, scratch_fd, qmp_read,
            stdout_write, stderr_write, exec_error_write, ready_write, ack_read,
        )
        os._exit(127)
    for fd in (qmp_read, stdout_write, stderr_write, exec_error_write, ready_write, ack_read):
        os.close(fd)
    process: Optional[MachineChild] = None
    ack_sent = False
    pidfd = -1
    namespace_fds: Tuple[int, ...] = ()
    try:
        _read_stage_one(ready_read, deadline)
        leader = binary_gate._pid_group_record(Path("/proc") / str(pid))
        require(leader is not None, "machine-none child disappeared before identity capture")
        require(leader["pgrp"] == pid, "machine-none child did not establish an owned process group")
        identity = {
            "pid": pid, "pgid": pid, "starttime_ticks": int(leader["starttime_ticks"]),
            "started_monotonic_ns": started_ns, "term_sent": 0, "kill_sent": 0,
        }
        pidfd = _pidfd_open(pid)
        require(os.write(ack_write, b"N") == 1, "failed to authorize namespace setup")
        claim = _read_ready_payload(ready_read, deadline)
        namespace, namespace_fds = _namespace_observation(pid, identity["starttime_ticks"], pidfd, claim, policy)
        pre_exec_check(namespace, deadline)
        for fd in namespace_fds:
            os.close(fd)
        namespace_fds = ()
        stdout_stream = os.fdopen(stdout_read, "rb", buffering=0)
        try:
            stderr_stream = os.fdopen(stderr_read, "rb", buffering=0)
            exec_error_stream = os.fdopen(exec_error_read, "rb", buffering=0)
        except BaseException:
            stdout_stream.close()
            raise
        process = MachineChild(
            pid, qmp_write, stdout_stream, stderr_stream, exec_error_stream,
            identity, deadline, namespace, (pidfd,),
        )
        pidfd = -1
        require(hasattr(signal, "pthread_sigmask"), "pthread signal-mask control is unavailable")
        blocked = {signal.SIGHUP, signal.SIGINT, signal.SIGQUIT, signal.SIGTERM}
        previous_mask = signal.pthread_sigmask(signal.SIG_BLOCK, blocked)
        try:
            require(time.monotonic() < deadline, "absolute deadline expired before exec authorization")
            require(os.write(ack_write, b"E") == 1, "failed to release namespace child for exec")
            ack_sent = True
            os.close(ready_read)
            ready_read = -1
            os.close(ack_write)
            ack_write = -1
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)
        return process
    except BaseException:
        if process is not None:
            if ack_sent:
                binary_gate._ensure_probe_stopped(process)
            else:
                binary_gate._kill_unready_direct_child(pid)
            process.close_pipes()
            process.close_pinned_fds()
        else:
            binary_gate._kill_unready_direct_child(pid)
            for fd in namespace_fds:
                try:
                    os.close(fd)
                except OSError:
                    pass
            if pidfd >= 0:
                try:
                    os.close(pidfd)
                except OSError:
                    pass
            for fd in (qmp_write, stdout_read, stderr_read, exec_error_read):
                try:
                    os.close(fd)
                except OSError:
                    pass
        raise
    finally:
        for fd in (ready_read, ack_write):
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass


def _strict_qmp_frame(payload: bytes, *, allow_event: bool = False) -> Dict[str, Any]:
    require(payload.endswith(b"\n"), "QMP frame lacks newline termination")
    require(len(payload) <= MAX_QMP_LINE, "QMP frame exceeded line limit")
    stripped = payload.rstrip(b"\r\n")
    require(bool(stripped) and b"\n" not in stripped and b"\r" not in stripped, "QMP frame contains embedded line breaks")
    try:
        value = json.loads(
            stripped.decode("utf-8", errors="strict"),
            object_pairs_hook=install_gate._reject_duplicate_keys,
            parse_constant=install_gate._reject_nonfinite,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, install_gate.QemuInstallGateError) as exc:
        raise QemuMachineNoneError("QMP frame is invalid strict JSON") from exc
    require(isinstance(value, dict), "QMP frame is not an object")
    require("error" not in value, "QMP error is forbidden")
    require(allow_event or "event" not in value, "QMP event is forbidden before quit")
    return value


def _is_exact_shutdown_event(frame: Mapping[str, Any]) -> bool:
    if set(frame) != {"event", "data", "timestamp"}:
        return False
    if frame.get("event") != "SHUTDOWN" or frame.get("data") != {"guest": False, "reason": "host-qmp-quit"}:
        return False
    timestamp = frame.get("timestamp")
    return (
        isinstance(timestamp, dict)
        and set(timestamp) == {"seconds", "microseconds"}
        and isinstance(timestamp["seconds"], int)
        and timestamp["seconds"] >= 0
        and isinstance(timestamp["microseconds"], int)
        and 0 <= timestamp["microseconds"] < 1_000_000
    )


def validate_qmp_frames(frames: Sequence[Mapping[str, Any]]) -> Tuple[bool, bool]:
    require(5 <= len(frames) <= MAX_QMP_FRAMES, "QMP frame count differs")
    require(list(frames[:5]) == [EXPECTED_QMP_GREETING, *EXPECTED_QMP_PRE_QUIT_RESPONSES], "pre-quit QMP transcript differs")
    post_quit = list(frames[5:])
    require(all(frame == EXPECTED_QMP_QUIT_RESPONSE or _is_exact_shutdown_event(frame) for frame in post_quit), "post-quit QMP transcript differs")
    quit_count = sum(frame == EXPECTED_QMP_QUIT_RESPONSE for frame in post_quit)
    shutdown_count = sum(_is_exact_shutdown_event(frame) for frame in post_quit)
    require(quit_count <= 1, "duplicate QMP quit response")
    require(shutdown_count <= 1, "duplicate QMP shutdown event")
    return bool(quit_count), bool(shutdown_count)


def _post_greeting_attestation(process: MachineChild) -> Dict[str, Any]:
    entry = Path("/proc") / str(process.pid)
    leader = binary_gate._pid_group_record(entry)
    require(leader is not None, "QEMU disappeared during post-greeting attestation")
    require(
        leader["starttime_ticks"] == process.identity["starttime_ticks"]
        and leader["pgrp"] == process.identity["pgid"] == process.pid,
        "QEMU identity drifted during post-greeting attestation",
    )
    require(os.readlink(entry / "ns/user") == process.namespace["child_user_namespace"], "QEMU user namespace drifted")
    require(os.readlink(entry / "ns/net") == process.namespace["child_network_namespace"], "QEMU network namespace drifted")
    security = _status_security(entry / "status")
    require(security["supplementary_groups"] == [], "QEMU inherited supplementary groups")
    require(all(value == "0000000000000000" for value in security["capabilities"].values()), "QEMU capability set is nonzero")
    require(security["no_new_privs"] == "1", "QEMU lost no-new-privs")
    require(security["seccomp"] == "2", "QEMU seccomp filter is not active")
    fd_dir = entry / "fd"
    classes = {"pipe": 0, "anon_inode": 0, "dev_null": 0, "other": 0}
    forbidden_targets: List[str] = []
    sensitive_prefixes = (
        "/dev/kvm", "/dev/net/tun", "/dev/vhost", "/dev/dri", "/dev/vfio",
        "/dev/bus/usb", "/data/", "/home/",
    )
    for descriptor in fd_dir.iterdir():
        if not descriptor.name.isdigit():
            continue
        try:
            target = os.readlink(descriptor)
            observed = os.stat(descriptor)
        except (FileNotFoundError, ProcessLookupError):
            continue
        if target.startswith("pipe:["):
            classes["pipe"] += 1
        elif target.startswith("anon_inode:["):
            classes["anon_inode"] += 1
        elif target == "/dev/null":
            classes["dev_null"] += 1
        else:
            classes["other"] += 1
        if target.startswith("socket:[") or target.startswith(sensitive_prefixes) or stat.S_ISBLK(observed.st_mode):
            forbidden_targets.append(target)
    require(not forbidden_targets, "QEMU opened a forbidden device, socket, block file or host path")
    require(classes["other"] == 0, "QEMU opened an unexpected file descriptor target")
    leader_after = binary_gate._pid_group_record(entry)
    require(leader_after is not None and leader_after["starttime_ticks"] == process.identity["starttime_ticks"], "QEMU identity drifted while scanning FDs")
    return {
        "pid_starttime_pgid_stable": True,
        "user_namespace_unchanged": True,
        "network_namespace_unchanged": True,
        "supplementary_groups_empty": True,
        "capability_sets_zero": True,
        "no_new_privs": True,
        "seccomp_mode_filter": True,
        "fd_classes": classes,
        "forbidden_fd_count": 0,
        "verified_after_qmp_greeting": True,
    }


def _execute_qmp(process: MachineChild, binary_sha256: str, policy: Mapping[str, Any]) -> Dict[str, Any]:
    selector = selectors.DefaultSelector()
    buffers = {"stdout": bytearray(), "stderr": bytearray(), "exec_error": bytearray()}
    stdout_pending = bytearray()
    frames: List[Dict[str, Any]] = []
    frame_wires: List[bytes] = []
    command_wires: List[bytes] = []
    post_greeting: Optional[Dict[str, Any]] = None
    quit_response_observed = False
    shutdown_event_observed = False
    try:
        for stream, label in ((process.stdout, "stdout"), (process.stderr, "stderr"), (process.exec_error, "exec_error")):
            os.set_blocking(stream.fileno(), False)
            selector.register(stream, selectors.EVENT_READ, label)

        def next_frame() -> Tuple[Dict[str, Any], bytes]:
            while b"\n" not in stdout_pending:
                remaining = process.deadline_monotonic - time.monotonic()
                require(remaining > 0, "QMP absolute deadline expired")
                events = selector.select(min(remaining, 0.1))
                require(bool(events) or time.monotonic() < process.deadline_monotonic, "QMP absolute deadline expired")
                for key, _ in events:
                    label = key.data
                    limit = 32 if label == "exec_error" else (MAX_STDOUT if label == "stdout" else MAX_STDERR)
                    chunk = os.read(key.fileobj.fileno(), min(4096, limit + 1))
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    buffers[label].extend(chunk)
                    require(len(buffers[label]) <= limit, f"QMP {label} exceeded byte limit")
                    if label == "stdout":
                        stdout_pending.extend(chunk)
                        if b"\n" not in stdout_pending:
                            require(len(stdout_pending) <= MAX_QMP_LINE, "QMP unterminated frame exceeded line limit")
                    elif label == "exec_error":
                        raise QemuMachineNoneError(f"QEMU exec failed: {bytes(buffers[label])!r}")
            line, remainder = bytes(stdout_pending).split(b"\n", 1)
            stdout_pending.clear()
            stdout_pending.extend(remainder)
            wire = line + b"\n"
            frame = _strict_qmp_frame(wire)
            frames.append(frame)
            frame_wires.append(wire)
            require(len(frames) <= MAX_QMP_FRAMES, "QMP emitted too many frames")
            return frame, wire

        greeting, _ = next_frame()
        require(greeting == EXPECTED_QMP_GREETING, "QMP greeting differs")
        require(not stdout_pending, "QMP emitted extra data with its greeting")
        post_greeting = _post_greeting_attestation(process)
        for command, expected_response in zip(QMP_COMMANDS[:-1], EXPECTED_QMP_PRE_QUIT_RESPONSES):
            wire = _canonical_qmp_wire(command)
            command_wires.append(wire)
            require(sum(len(item) for item in command_wires) <= MAX_COMMAND_BYTES, "QMP command bytes exceeded limit")
            require(os.write(process.qmp_write, wire) == len(wire), "short QMP command write")
            response, _ = next_frame()
            require(response == expected_response, f"QMP response differs for {command['id']}")
            require(not stdout_pending, f"QMP emitted extra data before command {command['id']} completed")
        quit_wire = _canonical_qmp_wire(QMP_COMMANDS[-1])
        command_wires.append(quit_wire)
        require(sum(len(item) for item in command_wires) <= MAX_COMMAND_BYTES, "QMP command bytes exceeded limit")
        require(os.write(process.qmp_write, quit_wire) == len(quit_wire), "short QMP quit write")
        os.close(process.qmp_write)
        process.qmp_write = -1

        while selector.get_map():
            remaining = process.deadline_monotonic - time.monotonic()
            require(remaining > 0, "QMP process did not close streams before deadline")
            for key, _ in selector.select(min(remaining, 0.1)):
                label = key.data
                limit = 32 if label == "exec_error" else (MAX_STDOUT if label == "stdout" else MAX_STDERR)
                chunk = os.read(key.fileobj.fileno(), min(4096, limit + 1))
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                buffers[label].extend(chunk)
                require(len(buffers[label]) <= limit, f"QMP {label} exceeded byte limit")
                if label == "stdout":
                    stdout_pending.extend(chunk)
                    while b"\n" in stdout_pending:
                        line, remainder = bytes(stdout_pending).split(b"\n", 1)
                        stdout_pending.clear()
                        stdout_pending.extend(remainder)
                        frame = _strict_qmp_frame(line + b"\n", allow_event=True)
                        frames.append(frame)
                        frame_wires.append(line + b"\n")
                        require(len(frames) <= MAX_QMP_FRAMES, "QMP emitted too many frames")
                        if frame == EXPECTED_QMP_QUIT_RESPONSE and not quit_response_observed:
                            quit_response_observed = True
                        elif _is_exact_shutdown_event(frame) and not shutdown_event_observed:
                            shutdown_event_observed = True
                        else:
                            raise QemuMachineNoneError("QMP emitted an unexpected or duplicate post-quit frame")
                    require(len(stdout_pending) <= MAX_QMP_LINE, "QMP trailing frame exceeded line limit")
                elif label == "exec_error":
                    raise QemuMachineNoneError(f"QEMU exec failed: {bytes(buffers[label])!r}")
        require(not stdout_pending, "QMP emitted trailing or extra output")
        require(not buffers["stderr"], "QEMU wrote to stderr")
        require(not buffers["exec_error"], "QEMU exec-error pipe is nonempty")
        members = binary_gate._process_group_members(process.identity["pgid"])
        while not binary_gate._only_leader_zombie(process, members):
            require(time.monotonic() < process.deadline_monotonic, "QEMU did not reach a clean exit")
            time.sleep(0.005)
            members = binary_gate._process_group_members(process.identity["pgid"])
        process.wait(timeout=max(0.0, process.deadline_monotonic - time.monotonic()))
        require(not binary_gate._process_group_members(process.identity["pgid"]), "QEMU left a descendant process group")
        require(process.returncode == 0, "QEMU machine-none process did not exit zero")
        observed_quit, observed_shutdown = validate_qmp_frames(frames)
        require(observed_quit == quit_response_observed and observed_shutdown == shutdown_event_observed, "QMP post-quit observation is detached")
        finished_ns = time.monotonic_ns()
        stdout = bytes(buffers["stdout"])
        commands = b"".join(command_wires)
        try:
            stdout_text = stdout.decode("utf-8", errors="strict")
        except UnicodeDecodeError as exc:
            raise QemuMachineNoneError("QMP transcript is not UTF-8") from exc
        return {
            "argv": list(EXACT_QEMU_ARGV),
            "environment": dict(EXACT_ENV),
            "cwd": str(SCRATCH_DIR),
            "binary_sha256": binary_sha256,
            "pid": process.identity["pid"],
            "pgid": process.identity["pgid"],
            "starttime_ticks": process.identity["starttime_ticks"],
            "started_monotonic_ns": process.identity["started_monotonic_ns"],
            "finished_monotonic_ns": finished_ns,
            "exec_succeeded": True,
            "returncode": 0,
            "signal": None,
            "timed_out": False,
            "qmp_commands": [dict(item) for item in QMP_COMMANDS],
            "qmp_command_wire_sha256": hashlib.sha256(commands).hexdigest(),
            "qmp_command_bytes": len(commands),
            "qmp_frames": frames,
            "qmp_frame_count": len(frames),
            "quit_command_write_complete": True,
            "quit_response_observed": quit_response_observed,
            "shutdown_event_observed": shutdown_event_observed,
            "post_greeting_attestation": post_greeting,
            "stdout_bytes": len(stdout),
            "stderr_bytes": 0,
            "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
            "stderr_sha256": hashlib.sha256(b"").hexdigest(),
            "stdout_text": stdout_text,
            "stderr_text": "",
            "unexpected_frames": 0,
            "cleanup": {"term_sent": False, "kill_sent": False, "owned_process_group_residual": False},
            "hardening": {
                "start_new_session": True,
                "fork_exec_launcher": True,
                "subprocess_preexec_fn": False,
                "pidfd_opened": True,
                "parent_death_signal": "SIGKILL",
                "parent_ack_after_namespace_verification": True,
                "signals_reset_before_exec": True,
                "signal_mask_cleared_before_exec": True,
                "close_fds": True,
                "no_new_privs": True,
                "umask": "0077",
                "same_open_fd_hash_and_exec": True,
                "rlimits": dict(policy["execution_boundary"]["rlimits"]),
            },
        }
    except BaseException:
        binary_gate._ensure_probe_stopped(process)
        raise
    finally:
        selector.close()
        process.close_pipes()


@contextmanager
def exclusive_smoke_lock() -> Iterator[Dict[str, int]]:
    fd = safety.open_directory_nofollow(LOCK_PATH)
    try:
        observed = os.fstat(fd)
        require(stat.S_ISDIR(observed.st_mode) and observed.st_uid == os.getuid(), "smoke lock directory identity differs")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise QemuMachineNoneError("another liquid sandbox gate holds the lock") from exc
        yield {"device_id": observed.st_dev, "inode": observed.st_ino}
    finally:
        os.close(fd)


@contextmanager
def fixed_publication_umask() -> Iterator[None]:
    require(binary_gate._thread_count() == 1, "publication umask requires a single-threaded gate process")
    previous = os.umask(0o027)
    try:
        yield
    finally:
        os.umask(previous)


def _attempt_path() -> Path:
    return AUDIT_DIR / ATTEMPT_NAME


def read_attempt_marker_nofollow(path: Path) -> Tuple[Dict[str, Any], str, os.stat_result]:
    require(path == _attempt_path(), "attempt marker path differs")
    directory_fd = safety.open_directory_nofollow(AUDIT_DIR)
    try:
        fd = os.open(
            path.name,
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=directory_fd,
        )
        try:
            before = os.fstat(fd)
            require(stat.S_ISREG(before.st_mode), "attempt marker is not a regular file")
            require(before.st_nlink == 1, "attempt marker link count differs")
            require((before.st_uid, before.st_gid) == (1000, 1000), "attempt marker ownership differs")
            require(stat.S_IMODE(before.st_mode) == 0o600, "attempt marker mode differs")
            require(0 < before.st_size <= MAX_JSON_BYTES, "attempt marker size differs")
            chunks = []
            remaining = before.st_size
            while remaining:
                chunk = os.read(fd, min(remaining, 65536))
                require(bool(chunk), "attempt marker ended before its recorded size")
                chunks.append(chunk)
                remaining -= len(chunk)
            require(not os.read(fd, 1), "attempt marker grew while reading")
            after = os.fstat(fd)
        finally:
            os.close(fd)
    finally:
        os.close(directory_fd)
    require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_nlink)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_nlink),
        "attempt marker changed while reading",
    )
    payload = b"".join(chunks)
    require(payload.endswith(b"\n"), "attempt marker lacks final newline")
    try:
        decoded = json.loads(
            payload.decode("utf-8", errors="strict"),
            object_pairs_hook=install_gate._reject_duplicate_keys,
            parse_constant=install_gate._reject_nonfinite,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, install_gate.QemuInstallGateError) as exc:
        raise QemuMachineNoneError("attempt marker is not strict JSON") from exc
    require(isinstance(decoded, dict), "attempt marker is not an object")
    return decoded, hashlib.sha256(payload).hexdigest(), after


def _residue() -> List[str]:
    directory_fd = safety.open_directory_nofollow(AUDIT_DIR)
    try:
        names = sorted(os.listdir(directory_fd))
    finally:
        os.close(directory_fd)
    prefixes = (f".{RECEIPT_NAME}.partial", f".{ATTEMPT_NAME}.partial")
    return [name for name in names if name in (RECEIPT_NAME, ATTEMPT_NAME) or name.startswith(prefixes)]


def _write_attempt_marker(policy_hash: str, admission_hash: str, attempt_schema_hash: str, receipt_schema_hash: str) -> Dict[str, Any]:
    path = _attempt_path()
    core = {
        "schema_version": "smpcc-r8-liquid-qemu-machine-none-attempt-v1",
        "document_type": "SMPCC_R8_LIQUID_QEMU_MACHINE_NONE_ATTEMPT",
        "scope": "ONE_MACHINE_NONE_NAMESPACE_OR_QEMU_ATTEMPT",
        "created_utc": utc_now(),
        "boot_id_sha256": binary_gate._boot_hash(),
        "runtime_uid": os.getuid(),
        "runtime_gid": os.getgid(),
        "policy": {"path": POLICY_RELATIVE, "sha256": policy_hash},
        "admission_schema": {"path": ADMISSION_SCHEMA_RELATIVE, "sha256": admission_hash},
        "attempt_schema": {"path": ATTEMPT_SCHEMA_RELATIVE, "sha256": attempt_schema_hash},
        "receipt_schema": {"path": RECEIPT_SCHEMA_RELATIVE, "sha256": receipt_schema_hash},
        "implementation": _implementation_evidence(),
        "namespace_or_qemu_attempt_consumed": True,
        "namespace_state_at_commit": "NOT_CREATED",
        "qemu_execution_state_at_commit": "NOT_EXECUTED",
        "future_execution_authorized": False,
        "authorization_reusable": False,
        "never_auto_delete": True,
    }
    payload = dict(core, marker_hash=safety.canonical_hash(core))
    attempt_schema, observed_attempt_schema_hash = read_json_and_hash(ATTEMPT_SCHEMA_PATH)
    require(observed_attempt_schema_hash == attempt_schema_hash, "attempt schema hash drifted")
    validate_with_schema(payload, attempt_schema, "machine-none attempt marker")
    data = safety.canonical_bytes(payload) + b"\n"
    directory_fd = safety.open_directory_nofollow(AUDIT_DIR)
    try:
        fd = os.open(
            path.name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=directory_fd,
        )
        try:
            offset = 0
            while offset < len(data):
                written = os.write(fd, data[offset:])
                require(written > 0, "attempt marker write made no progress")
                offset += written
            os.fsync(fd)
        finally:
            os.close(fd)
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    stored, stored_hash, stored_stat = read_attempt_marker_nofollow(path)
    stored_core = dict(stored)
    marker_hash = stored_core.pop("marker_hash", None)
    require(stored == payload and marker_hash == safety.canonical_hash(stored_core), "stored attempt marker semantic verification failed")
    return {
        "path": str(path),
        "file_sha256": stored_hash,
        "bytes": stored_stat.st_size,
        "marker_hash": marker_hash,
        "stored_semantic_verification": True,
    }


def validate_receipt_path(path: Path) -> Path:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    path = safety.ensure_within_approved_root(path, root=root)
    require(path.parent == AUDIT_DIR and path.name == RECEIPT_NAME, "receipt path is not the fixed singleton")
    require(not path.exists() and not path.is_symlink(), "receipt path already exists")
    require(not _residue(), "a machine-none receipt, attempt marker or partial residue already exists")
    return path


def read_published_receipt(path: Path) -> Tuple[Dict[str, Any], str, Dict[str, Any]]:
    require(path == AUDIT_DIR / RECEIPT_NAME, "published receipt path differs")
    directory_fd = safety.open_directory_nofollow(AUDIT_DIR)
    try:
        fd = os.open(
            path.name,
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=directory_fd,
        )
        try:
            before = os.fstat(fd)
            require(stat.S_ISREG(before.st_mode), "published receipt is not a regular file")
            require(before.st_nlink == 1, "published receipt link count differs")
            require((before.st_uid, before.st_gid) == (1000, 1000), "published receipt ownership differs")
            require(stat.S_IMODE(before.st_mode) == 0o640, "published receipt mode differs")
            require(0 < before.st_size <= MAX_JSON_BYTES, "published receipt size differs")
            chunks = []
            remaining = before.st_size
            while remaining:
                chunk = os.read(fd, min(remaining, 65536))
                require(bool(chunk), "published receipt ended before its recorded size")
                chunks.append(chunk)
                remaining -= len(chunk)
            require(not os.read(fd, 1), "published receipt grew while reading")
            after = os.fstat(fd)
        finally:
            os.close(fd)
    finally:
        os.close(directory_fd)
    require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_nlink)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_nlink),
        "published receipt changed while reading",
    )
    payload = b"".join(chunks)
    require(payload.endswith(b"\n"), "published receipt lacks final newline")
    try:
        decoded = json.loads(
            payload.decode("utf-8", errors="strict"),
            object_pairs_hook=install_gate._reject_duplicate_keys,
            parse_constant=install_gate._reject_nonfinite,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, install_gate.QemuInstallGateError) as exc:
        raise QemuMachineNoneError("published receipt is not strict JSON") from exc
    require(isinstance(decoded, dict), "published receipt is not an object")
    return decoded, hashlib.sha256(payload).hexdigest(), {
        "device_id": after.st_dev,
        "inode": after.st_ino,
        "size": after.st_size,
        "mode": "0640",
        "uid": after.st_uid,
        "gid": after.st_gid,
        "link_count": after.st_nlink,
    }


def build_smoke_receipt(receipt_path: Path) -> Dict[str, Any]:
    validate_runtime_paths()
    policy, policy_hash, _, admission_hash, _, attempt_schema_hash, receipt_schema, receipt_schema_hash = load_contract()
    # The checked-in current-boot policy is intentionally fail-closed here.
    require(policy["host_kernel"]["live_namespace_or_qemu_execution_authorized"] is True, "current kernel/boot policy does not authorize namespace or QEMU execution")
    predecessor = verify_predecessor(policy)
    require(not predecessor_errors(predecessor), "predecessor binary receipt is not admissible")
    snapshots: List[Mapping[str, Any]] = []
    process: Optional[MachineChild] = None
    with exclusive_smoke_lock() as locked_directory, fixed_publication_umask():
        receipt_path = validate_receipt_path(receipt_path)
        snapshot_a = collect_snapshot(policy, "A_PRELOCKED")
        require(not snapshot_errors(snapshot_a, policy), "snapshot A is NO-GO")
        require(
            (snapshot_a["lock"]["device_id"], snapshot_a["lock"]["inode"])
            == (locked_directory["device_id"], locked_directory["inode"]),
            "locked directory differs from snapshot A",
        )
        snapshots.append(snapshot_a)
        qemu_fd, opened_binary = binary_gate.open_verified_qemu_fd(policy)
        scratch_fd: Optional[int] = None
        try:
            scratch_fd = safety.open_directory_nofollow(SCRATCH_DIR)
            attempt_result = _write_attempt_marker(policy_hash, admission_hash, attempt_schema_hash, receipt_schema_hash)

            def pre_exec_check(namespace: Mapping[str, Any], deadline: float) -> None:
                require(time.monotonic() < deadline, "absolute deadline expired before snapshot B")
                snapshot_b = collect_snapshot(policy, "B_PREEXEC")
                require(not snapshot_errors(snapshot_b, policy), "snapshot B is NO-GO")
                require(stable_snapshot_projection(snapshot_a) == stable_snapshot_projection(snapshot_b), "runtime state drifted across A/B")
                require(
                    (opened_binary["device_id"], opened_binary["inode"], opened_binary["sha256"])
                    == (snapshot_b["binary"]["device_id"], snapshot_b["binary"]["inode"], snapshot_b["binary"]["sha256"]),
                    "pre-exec binary identity differs from open FD",
                )
                scratch_stat = os.fstat(scratch_fd)
                require((scratch_stat.st_dev, scratch_stat.st_ino) == (snapshot_b["scratch"]["device_id"], snapshot_b["scratch"]["inode"]), "pre-opened scratch differs")
                snapshots.append(snapshot_b)

            process = _spawn_machine_none(qemu_fd, scratch_fd, policy, pre_exec_check)
            namespace_result = dict(process.namespace)
            try:
                smoke_result = _execute_qmp(process, opened_binary["sha256"], policy)
            finally:
                if process is not None:
                    process.close_pinned_fds()
                snapshot_c = collect_snapshot(policy, "C_POSTCLEANUP")
                snapshots.append(snapshot_c)
        finally:
            if scratch_fd is not None:
                os.close(scratch_fd)
            os.close(qemu_fd)
        require(len(snapshots) == 3, "snapshot sequence is incomplete")
        require(not snapshot_errors(snapshot_c, policy), "snapshot C is NO-GO")
        require(stable_snapshot_projection(snapshots[1]) == stable_snapshot_projection(snapshot_c), "runtime state drifted across B/C")
        report = _base_report(
            policy, policy_hash, admission_hash, attempt_schema_hash, receipt_schema_hash, predecessor,
            snapshots, attempt_result, namespace_result, smoke_result, [],
            "PASS_TCG_MACHINE_NONE_QMP_CLEAN_QUIT_ONLY",
        )
        validate_with_schema(report, receipt_schema, "machine-none receipt")
        validate_report_semantics(report, policy)
        safety.atomic_write_json_new(receipt_path, report)
        stored, _, _ = read_published_receipt(receipt_path)
        require(stored == report, "published receipt bytes decode to a different object")
        validate_with_schema(stored, receipt_schema, "stored machine-none receipt")
        validate_report_semantics(stored, policy)
        return stored


def validate_report_semantics(report: Mapping[str, Any], policy: Mapping[str, Any]) -> None:
    core = dict(report)
    actual_hash = core.pop("receipt_hash")
    require(actual_hash == safety.canonical_hash(core), "result self-hash differs")
    require(report["status"] == "PASS_TCG_MACHINE_NONE_QMP_CLEAN_QUIT_ONLY", "receipt status differs")
    require(not report["errors"], "PASS receipt contains errors")

    current_policy, policy_hash, _, admission_hash, attempt_schema, attempt_schema_hash, receipt_schema, receipt_schema_hash = load_contract()
    require(current_policy == policy, "receipt policy object differs from current policy")
    require(
        report["policy"]
        == {
            "path": POLICY_RELATIVE,
            "sha256": policy_hash,
            "admission_schema_path": ADMISSION_SCHEMA_RELATIVE,
            "admission_schema_sha256": admission_hash,
            "attempt_schema_path": ATTEMPT_SCHEMA_RELATIVE,
            "attempt_schema_sha256": attempt_schema_hash,
            "receipt_schema_path": RECEIPT_SCHEMA_RELATIVE,
            "receipt_schema_sha256": receipt_schema_hash,
        },
        "receipt is detached from policy or schemas",
    )
    require(report["implementation"] == _implementation_evidence(), "receipt implementation identity differs")

    observed_predecessor = verify_predecessor(policy)
    require(report["predecessor_binary_probe_receipt"] == observed_predecessor, "receipt predecessor evidence is detached")
    require(not predecessor_errors(observed_predecessor), "receipt predecessor is not admissible")

    marker_report = report["attempt_marker"]
    require(marker_report["path"] == str(_attempt_path()), "receipt attempt marker path differs")
    marker, marker_file_hash, marker_stat = read_attempt_marker_nofollow(_attempt_path())
    validate_with_schema(marker, attempt_schema, "stored machine-none attempt marker")
    marker_core = dict(marker)
    marker_inner_hash = marker_core.pop("marker_hash")
    require(marker_inner_hash == safety.canonical_hash(marker_core), "stored attempt marker self-hash differs")
    require(marker_file_hash == marker_report["file_sha256"], "receipt attempt marker file hash is detached")
    require(marker_stat.st_size == marker_report["bytes"], "receipt attempt marker byte count is detached")
    require(marker_inner_hash == marker_report["marker_hash"], "receipt attempt marker inner hash is detached")
    require(marker_report["stored_semantic_verification"] is True, "receipt attempt marker is not verified")
    require(marker["boot_id_sha256"] == policy["boot_session"]["boot_id_sha256"], "attempt marker boot binding differs")
    require(marker["runtime_uid"] == marker["runtime_gid"] == 1000, "attempt marker runtime identity differs")
    require(marker["policy"] == {"path": POLICY_RELATIVE, "sha256": policy_hash}, "attempt marker policy binding differs")
    require(marker["admission_schema"] == {"path": ADMISSION_SCHEMA_RELATIVE, "sha256": admission_hash}, "attempt marker admission schema binding differs")
    require(marker["attempt_schema"] == {"path": ATTEMPT_SCHEMA_RELATIVE, "sha256": attempt_schema_hash}, "attempt marker schema binding differs")
    require(marker["receipt_schema"] == {"path": RECEIPT_SCHEMA_RELATIVE, "sha256": receipt_schema_hash}, "attempt marker receipt schema binding differs")
    require(marker["implementation"] == _implementation_evidence(), "attempt marker implementation binding differs")

    require(len(report["snapshot_sequence"]) == 3, "receipt snapshot sequence differs")
    strict_snapshot_schema = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$defs": receipt_schema["$defs"],
        "$ref": "#/$defs/snapshot",
    }
    snapshots: List[Mapping[str, Any]] = []
    expected_phases = ("A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP")
    for record, expected_phase in zip(report["snapshot_sequence"], expected_phases):
        try:
            snapshot = json.loads(
                record["canonical"],
                object_pairs_hook=install_gate._reject_duplicate_keys,
                parse_constant=install_gate._reject_nonfinite,
            )
        except (json.JSONDecodeError, install_gate.QemuInstallGateError) as exc:
            raise QemuMachineNoneError("canonical runtime snapshot is invalid") from exc
        require(safety.canonical_bytes(snapshot).decode("utf-8") == record["canonical"], "snapshot canonical encoding differs")
        validate_with_schema(snapshot, strict_snapshot_schema, "canonical machine-none snapshot")
        require(snapshot["phase"] == record["phase"] == expected_phase, "snapshot phase is detached")
        require(record["sha256"] == safety.canonical_hash(snapshot), "snapshot hash is detached")
        require(record["invariants_pass"] is True and not snapshot_errors(snapshot, policy), "canonical snapshot is not admissible")
        snapshots.append(snapshot)
    stable_hashes = [safety.canonical_hash(stable_snapshot_projection(item)) for item in snapshots]
    require(
        report["stability"]
        == {"stable_projection_sha256": stable_hashes, "ab_equal": True, "bc_equal": True},
        "receipt stability evidence is detached",
    )
    require(len(set(stable_hashes)) == 1, "runtime state drifted across receipt snapshots")

    namespace = report["namespace_result"]
    smoke = report["smoke_result"]
    require(isinstance(namespace, dict) and namespace["verified_before_exec"] is True, "namespace evidence is missing")
    require(namespace["user_namespace_distinct"] is True and namespace["network_namespace_distinct"] is True, "namespace separation differs")
    require(namespace["parent_user_namespace"] == snapshots[1]["host_namespaces"]["user"], "parent user namespace evidence is detached")
    require(namespace["parent_network_namespace"] == snapshots[1]["host_namespaces"]["network"], "parent network namespace evidence is detached")
    require(namespace["child_user_namespace"] != namespace["parent_user_namespace"], "child user namespace equals the host namespace")
    require(namespace["child_network_namespace"] != namespace["parent_network_namespace"], "child network namespace equals the host namespace")
    require(namespace["uid_map"] == policy["namespace_boundary"]["uid_map_exact"], "receipt UID map differs")
    require(namespace["gid_map"] == policy["namespace_boundary"]["gid_map_exact"], "receipt GID map differs")
    require(namespace["setgroups"] == "deny\n" and namespace["supplementary_groups"] == [], "namespace group isolation differs")
    require(all(value == "0000000000000000" for value in namespace["capabilities"].values()), "namespace capability evidence differs")
    require(namespace["no_new_privs"] == "1", "namespace no-new-privs evidence differs")
    require(namespace["interface_names"] == ["lo"], "namespace interface evidence differs")
    require(not namespace["ipv4_routes"] and not namespace["ipv6_routes"], "namespace route evidence differs")
    require(not any(namespace["inet_socket_rows"].values()), "namespace INET socket evidence differs")
    require(namespace["loopback_up"] is False and namespace["loopback_ipv4_configured"] is False and namespace["loopback_ipv6_configured"] is False, "namespace loopback evidence differs")
    require(namespace["namespace_fds_pinned_during_verification"] is True and namespace["namespace_fds_closed_before_exec_ack"] is True, "namespace FD lifetime evidence differs")
    require(namespace["pidfd_pinned_until_child_reap"] is True and namespace["persistent_bind_mount"] is False, "namespace persistence evidence differs")
    require(_namespace_inode(namespace["child_user_namespace"], "user") == namespace["child_user_namespace_inode"], "user namespace inode evidence differs")
    require(_namespace_inode(namespace["child_network_namespace"], "net") == namespace["child_network_namespace_inode"], "network namespace inode evidence differs")

    require(isinstance(smoke, dict) and tuple(smoke["argv"]) == EXACT_QEMU_ARGV, "smoke argv differs")
    require(smoke["environment"] == EXACT_ENV and smoke["binary_sha256"] == EXPECTED_QEMU_SHA256, "smoke execution identity differs")
    require(smoke["exec_succeeded"] is True and smoke["returncode"] == 0 and smoke["signal"] is None and smoke["timed_out"] is False, "smoke exit differs")
    require(smoke["finished_monotonic_ns"] >= smoke["started_monotonic_ns"], "smoke monotonic timing differs")
    require(smoke["qmp_commands"] == [dict(item) for item in QMP_COMMANDS], "QMP commands differ")
    command_wire = b"".join(_canonical_qmp_wire(item) for item in smoke["qmp_commands"])
    require(smoke["qmp_command_bytes"] == len(command_wire), "QMP command byte count is detached")
    require(smoke["qmp_command_wire_sha256"] == hashlib.sha256(command_wire).hexdigest(), "QMP command wire hash is detached")

    stdout_payload = smoke["stdout_text"].encode("utf-8")
    stderr_payload = smoke["stderr_text"].encode("utf-8")
    require(smoke["stdout_bytes"] == len(stdout_payload) and smoke["stderr_bytes"] == len(stderr_payload), "QMP output byte counts are detached")
    require(smoke["stdout_sha256"] == hashlib.sha256(stdout_payload).hexdigest(), "QMP stdout hash is detached")
    require(smoke["stderr_sha256"] == hashlib.sha256(stderr_payload).hexdigest(), "QMP stderr hash is detached")
    require(smoke["stderr_text"] == "" and smoke["stderr_bytes"] == 0, "QEMU stderr is not empty")
    raw_lines = stdout_payload.splitlines(keepends=True)
    require(raw_lines and b"".join(raw_lines) == stdout_payload and all(line.endswith(b"\n") for line in raw_lines), "QMP transcript framing differs")
    parsed_frames = [_strict_qmp_frame(line, allow_event=index >= 5) for index, line in enumerate(raw_lines)]
    require(parsed_frames == smoke["qmp_frames"], "QMP parsed transcript is detached")
    require(smoke["qmp_frame_count"] == len(smoke["qmp_frames"]), "QMP frame count is detached")
    require(smoke["unexpected_frames"] == 0 and smoke["quit_command_write_complete"] is True, "QMP quit evidence differs")
    observed_quit, observed_shutdown = validate_qmp_frames(smoke["qmp_frames"])
    require(smoke["quit_response_observed"] == observed_quit, "quit response evidence is detached")
    require(smoke["shutdown_event_observed"] == observed_shutdown, "shutdown event evidence is detached")
    require(smoke["post_greeting_attestation"]["verified_after_qmp_greeting"] is True, "post-greeting attestation is missing")
    attestation = smoke["post_greeting_attestation"]
    require(all(attestation[key] is True for key in (
        "pid_starttime_pgid_stable", "user_namespace_unchanged", "network_namespace_unchanged",
        "supplementary_groups_empty", "capability_sets_zero", "no_new_privs",
        "seccomp_mode_filter", "verified_after_qmp_greeting",
    )), "post-greeting security attestation differs")
    require(attestation["forbidden_fd_count"] == 0 and attestation["fd_classes"]["other"] == 0, "post-greeting FD attestation differs")
    require(smoke["pid"] == smoke["pgid"] == namespace["pid"] == namespace["pgid"], "child PID/PGID binding differs")
    require(smoke["starttime_ticks"] == namespace["starttime_ticks"], "child starttime binding differs")
    require(
        smoke["cleanup"] == {"term_sent": False, "kill_sent": False, "owned_process_group_residual": False},
        "successful cleanup evidence differs",
    )
    require(
        smoke["hardening"]
        == {
            "start_new_session": True,
            "fork_exec_launcher": True,
            "subprocess_preexec_fn": False,
            "pidfd_opened": True,
            "parent_death_signal": "SIGKILL",
            "parent_ack_after_namespace_verification": True,
            "signals_reset_before_exec": True,
            "signal_mask_cleared_before_exec": True,
            "close_fds": True,
            "no_new_privs": True,
            "umask": "0077",
            "same_open_fd_hash_and_exec": True,
            "rlimits": dict(policy["execution_boundary"]["rlimits"]),
        },
        "smoke hardening evidence differs",
    )
    effects = report["effects"]
    require(all(effects[key] is True for key in policy["admitted_effects"]), "admitted effect evidence differs")
    require(all(effects[key] is False for key in policy["forbidden_effects"]), "forbidden effect occurred")
    require(report["execution_was_explicitly_requested"] is True, "receipt lacks explicit execution request")
    require(report["future_execution_authorized"] is False and report["authorization_reusable"] is False, "receipt grants future authority")
    require("/home/" not in json.dumps(report, ensure_ascii=False, sort_keys=True), "receipt contains a home path")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check", help="read-only check; never creates namespaces or invokes QEMU")
    run = sub.add_parser("run-smoke", help="current policy is NO-GO until kernel update and new boot admission")
    run.add_argument("--receipt", required=True, type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "self-check":
            report = build_self_check()
        else:
            report = build_smoke_receipt(args.receipt)
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if report["status"].startswith("PASS_") else 2
    except (
        QemuMachineNoneError, binary_gate.QemuBinaryProbeError,
        install_gate.QemuInstallGateError, safety.LiquidSafetyError,
        OSError, ValueError, json.JSONDecodeError, subprocess.SubprocessError, SchemaError,
    ) as exc:
        print(f"R8_LIQUID_QEMU_MACHINE_NONE_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
