#!/usr/bin/env python3
"""Fail-closed gate for one QEMU binary-load/version probe.

``self-check`` is read-only and never invokes QEMU.  ``run-probe`` is the only
QEMU execution entry and is restricted to the exact ``-no-user-config
-version`` argv.  This module has no VM, image, qemu-img, network, build,
GenCase, ROS, Gazebo, or solver execution path.
"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import hashlib
import json
import os
import resource
import select
import selectors
import signal
import stat
import subprocess
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, BinaryIO, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple

from jsonschema.exceptions import SchemaError


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_qemu_install_gate_v2 as install_gate  # noqa: E402
import r8_liquid_safety as safety  # noqa: E402


SCHEMA_VERSION = "smpcc-r8-liquid-qemu-binary-probe-result-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_QEMU_BINARY_PROBE_RESULT"
POLICY_PATH = PACKAGE_ROOT / "config/sandbox/p0b_qemu_binary_probe_admission_v1.json"
ADMISSION_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_binary_probe_admission_v1.json"
RECEIPT_SCHEMA_PATH = PACKAGE_ROOT / "schema/qemu_binary_probe_receipt_v1.json"
POLICY_RELATIVE = "config/sandbox/p0b_qemu_binary_probe_admission_v1.json"
ADMISSION_SCHEMA_RELATIVE = "schema/qemu_binary_probe_admission_v1.json"
RECEIPT_SCHEMA_RELATIVE = "schema/qemu_binary_probe_receipt_v1.json"
QEMU = Path("/usr/bin/qemu-system-x86_64")
QEMU_IMG = Path("/usr/bin/qemu-img")
EXACT_QEMU_ARGV = (str(QEMU), "-no-user-config", "-version")
EXACT_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"}
SCRATCH_DIR = safety.APPROVED_ROOT / "scratch/vm"
LOCK_PATH = safety.APPROVED_ROOT / "locks"
MIN_MEM_AVAILABLE = 6 * 1024**3
MAX_OUTPUT = 4096
TIMEOUT_SECONDS = 5.0
TERM_GRACE_SECONDS = 1.0
EXPECTED_QEMU_SHA256 = "6e8225fd4ffcdc6c4696dd5bf181cade84876689965946e46a50834ad4c5a718"
PREDECESSOR_RECEIPT = Path(
    "/data/a/scout_sim_replacement/r8_liquid/audits/sandbox/"
    "qemu_install_preflight_v2_20260805T091213Z.json"
)
PREDECESSOR_FILE_SHA256 = "a32240c319487f6c2560c46ced91d426930b12949ae02dec52db6fea54347292"
PREDECESSOR_INNER_HASH = "a9f601fec190d269633d7638b916a9b9e40d93c915a141461f3d8afee5f055d7"
BOOT_ID_SHA256 = "18264e164ef3fb41d6dbac52d1ca8d989a9b5ab9e7154494af42f92e76730603"
EXPECTED_SERVICE = {
    "load_state": "loaded",
    "unit_file_state": "disabled",
    "active_state": "inactive",
    "sub_state": "dead",
    "fragment_path": "/lib/systemd/system/qemu-kvm.service",
}
UNREADABLE_PROCESS_ALLOWLIST = (
    {"pid": 1994, "uid": 1000, "starttime_ticks": 3269, "ppid": 1993, "parent_starttime_ticks": 3268, "cgroup_sha256": "c3847cdb4d628b175f450f9c786b40a059d371547cab40f2e0126b2080d35a2b", "comm": "(sd-pam)", "argv0": "(sd-pam)"},
    {"pid": 2007, "uid": 1000, "starttime_ticks": 3278, "ppid": 1, "parent_starttime_ticks": 12, "cgroup_sha256": "0027afc54b687534dcd9f0e69d182c658d5ce010a6502b5c4b2093e69e603918", "comm": "gnome-keyring-d", "argv0": "/usr/bin/gnome-keyring-daemon"},
    {"pid": 2083, "uid": 1000, "starttime_ticks": 3455, "ppid": 2031, "parent_starttime_ticks": 3449, "cgroup_sha256": "0027afc54b687534dcd9f0e69d182c658d5ce010a6502b5c4b2093e69e603918", "comm": "ssh-agent", "argv0": "/usr/bin/ssh-agent"},
)
SIMULATION_NAMES = frozenset(("roscore", "rosmaster", "roslaunch", "rosbag", "gzserver", "gzclient", "gazebo", "rviz"))
RECEIPT_NAME = "qemu_binary_probe_v1_boot_20260805T094926Z.json"
MAX_JSON_BYTES = 1024 * 1024
PR_SET_NO_NEW_PRIVS = 38
PR_SET_PDEATHSIG = 1
EXPECTED_QEMU_PACKAGE = {
    "name": "qemu-system-x86",
    "installed": True,
    "architecture": "amd64",
    "version": "1:4.2-3ubuntu6.30+esm3",
    "status": "install ok installed",
}


class QemuBinaryProbeError(RuntimeError):
    """Any ambiguity is a hard NO-GO."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise QemuBinaryProbeError(message)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def read_json_and_hash(path: Path) -> Tuple[Dict[str, Any], str]:
    try:
        return install_gate.read_json_and_hash(path)
    except install_gate.QemuInstallGateError as exc:
        raise QemuBinaryProbeError(str(exc)) from exc


def validate_with_schema(instance: Mapping[str, Any], schema: Mapping[str, Any], name: str) -> None:
    try:
        install_gate.validate_with_schema(instance, schema, name)
    except install_gate.QemuInstallGateError as exc:
        raise QemuBinaryProbeError(str(exc)) from exc


def _file_sha256_fd(fd: int) -> Tuple[str, os.stat_result]:
    before = os.fstat(fd)
    require(stat.S_ISREG(before.st_mode), "QEMU executable is not a regular file")
    os.lseek(fd, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    while True:
        chunk = os.read(fd, 1024 * 1024)
        if not chunk:
            break
        digest.update(chunk)
    after = os.fstat(fd)
    require(
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
        == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
        "QEMU executable changed while hashing",
    )
    os.lseek(fd, 0, os.SEEK_SET)
    return digest.hexdigest(), after


def open_verified_qemu_fd(policy: Mapping[str, Any]) -> Tuple[int, Dict[str, Any]]:
    require(not QEMU.is_symlink(), "QEMU final path is a symlink")
    parent_fd = safety.open_directory_nofollow(QEMU.parent)
    try:
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(QEMU.name, flags, dir_fd=parent_fd)
    finally:
        os.close(parent_fd)
    try:
        digest, observed = _file_sha256_fd(fd)
        capabilities = install_gate._capabilities(fd, QEMU)
        binary = policy["binary"]
        require(digest == binary["sha256"] == EXPECTED_QEMU_SHA256, "QEMU SHA-256 differs")
        require(observed.st_size == binary["size"], "QEMU size differs")
        require(f"{stat.S_IMODE(observed.st_mode):04o}" == binary["mode"], "QEMU mode differs")
        require((observed.st_uid, observed.st_gid) == (binary["uid"], binary["gid"]), "QEMU ownership differs")
        require(capabilities == binary["security_capabilities"], "QEMU file capabilities differ")
        return fd, {
            "path": str(QEMU), "file_type": "regular", "symlink": False,
            "uid": observed.st_uid, "gid": observed.st_gid,
            "mode": f"{stat.S_IMODE(observed.st_mode):04o}", "size": observed.st_size,
            "sha256": digest, "security_capabilities": capabilities,
            "device_id": observed.st_dev, "inode": observed.st_ino, "mtime_ns": observed.st_mtime_ns,
        }
    except Exception:
        os.close(fd)
        raise


def binary_observation(policy: Mapping[str, Any]) -> Dict[str, Any]:
    fd, observation = open_verified_qemu_fd(policy)
    os.close(fd)
    return observation


def _pid_details(entry: Path) -> Optional[Tuple[int, int, int]]:
    try:
        raw = (entry / "stat").read_text(encoding="ascii", errors="strict")
    except (FileNotFoundError, ProcessLookupError):
        return None
    except OSError as exc:
        raise QemuBinaryProbeError(f"cannot establish stable PID identity for {entry.name}: {exc}") from exc
    try:
        pid_text, remainder = raw.split(" (", 1)
        fields = remainder.rsplit(") ", 1)[1].split()
        result = (int(pid_text), int(fields[1]), int(fields[19]))
    except (IndexError, ValueError) as exc:
        raise QemuBinaryProbeError(f"malformed process identity for PID {entry.name}") from exc
    require(result[0] == int(entry.name), "PID identity changed")
    return result


def _pid_identity(entry: Path) -> Optional[Tuple[int, int]]:
    details = _pid_details(entry)
    return None if details is None else (details[0], details[2])


def _cgroup_hash(entry: Path) -> str:
    payload = (entry / "cgroup").read_bytes()
    require(len(payload) <= 65536, "process cgroup metadata is too large")
    return hashlib.sha256(payload).hexdigest()


def process_snapshot(binary: Mapping[str, Any], *, proc_root: Path = Path("/proc")) -> Dict[str, Any]:
    visibility_errors = install_gate._proc_mount_visibility(proc_root)
    exceptions: List[Dict[str, Any]] = []
    qemu_found: List[Dict[str, Any]] = []
    simulation_found: List[Dict[str, Any]] = []
    admitted_inode = (int(binary["device_id"]), int(binary["inode"]))
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        before = _pid_identity(entry)
        if before is None:
            continue
        try:
            uid = os.stat(entry).st_uid
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            if _pid_identity(entry) == before:
                visibility_errors.append(f"stable PID {entry.name} basic identity is unreadable: {exc}")
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
            argv = [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]
            cmdline_denied = False
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            argv, cmdline_denied = [], True
        try:
            exe = os.readlink(entry / "exe")
            if exe.endswith(" (deleted)"):
                exe = exe[:-10]
            exe_stat = os.stat(entry / "exe")
            exe_inode: Optional[Tuple[int, int]] = (exe_stat.st_dev, exe_stat.st_ino)
            exe_denied = False
        except PermissionError:
            exe, exe_inode, exe_denied = None, None, True
        except (FileNotFoundError, ProcessLookupError, OSError):
            exe, exe_inode, exe_denied = None, None, False
        if _pid_identity(entry) != before:
            continue
        argv0 = argv[0] if argv else ""
        process_names = {comm.lower()}
        if argv0:
            process_names.add(Path(argv0).name.lower())
        argument_names = set(process_names)
        argument_names.update(Path(token).name.lower() for token in argv if token)
        qemu_reasons = []
        if any(name.startswith("qemu") or name in ("qemu-kvm", "kvm") for name in process_names):
            qemu_reasons.append("process_name")
        if exe == str(QEMU) or exe == str(QEMU_IMG) or (exe and exe.startswith("/usr/bin/qemu-")):
            qemu_reasons.append("exe_path")
        if exe_inode == admitted_inode:
            qemu_reasons.append("exe_inode")
        matched_sim = sorted(argument_names.intersection(SIMULATION_NAMES))
        if qemu_reasons:
            qemu_found.append({"pid": int(entry.name), "uid": uid, "comm": comm, "exe": exe, "reasons": sorted(set(qemu_reasons))})
        if matched_sim:
            simulation_found.append({"pid": int(entry.name), "uid": uid, "comm": comm, "exe": exe, "reasons": matched_sim})
        if exe_denied and uid >= 1000 and not qemu_reasons and not matched_sim:
            details = _pid_details(entry)
            parent = None if details is None else _pid_identity(proc_root / str(details[1]))
            if details is None or parent is None:
                visibility_errors.append(f"stable PID {entry.name} parent identity is unavailable")
                continue
            candidate = {"pid": details[0], "uid": uid, "starttime_ticks": details[2], "ppid": details[1], "parent_starttime_ticks": parent[1], "cgroup_sha256": _cgroup_hash(entry), "comm": comm, "argv0": argv0}
            if candidate in UNREADABLE_PROCESS_ALLOWLIST:
                exceptions.append(candidate)
            else:
                visibility_errors.append(f"stable unprivileged PID {entry.name} executable identity is unreadable")
        if cmdline_denied and exe_denied and uid < 1000:
            # Root-owned processes are an explicit host-configuration trust boundary.
            continue
    return {
        "qemu_processes": sorted(qemu_found, key=lambda item: item["pid"]),
        "simulation_processes": sorted(simulation_found, key=lambda item: item["pid"]),
        "process_visibility_errors": sorted(set(visibility_errors)),
        "process_visibility_exceptions": sorted(exceptions, key=lambda item: item["pid"]),
    }


def _boot_hash() -> str:
    payload = Path("/proc/sys/kernel/random/boot_id").read_bytes()
    require(len(payload) <= 128, "boot identity is malformed")
    return hashlib.sha256(payload).hexdigest()


def _mem_available() -> int:
    matches = []
    for line in Path("/proc/meminfo").read_text(encoding="ascii", errors="strict").splitlines():
        if line.startswith("MemAvailable:"):
            fields = line.split()
            require(len(fields) == 3 and fields[2] == "kB", "MemAvailable encoding is invalid")
            matches.append(int(fields[1]) * 1024)
    require(len(matches) == 1, "MemAvailable is missing or ambiguous")
    return matches[0]


def _thread_count() -> int:
    task_dir = Path("/proc/self/task")
    entries = [entry for entry in task_dir.iterdir() if entry.name.isdigit()]
    require(bool(entries), "current process thread inventory is unavailable")
    return len(entries)


def _package_state() -> Dict[str, Any]:
    verify = install_gate.run_collector("dpkg-verify")
    return {
        "required_package": install_gate.package_observation("qemu-system-x86"),
        "qemu_namespace_installed": install_gate.qemu_namespace(),
        "dpkg_verify": {
            "returncode": verify.returncode,
            "stdout_empty": not bool(verify.stdout),
            "stderr_empty": not bool(verify.stderr),
        },
        "binary_dpkg_owner": install_gate.package_owner(QEMU),
        "binary_diverted": install_gate.is_diverted(QEMU),
    }


def _directory_observation(path: Path) -> Dict[str, Any]:
    directory_fd = safety.open_directory_nofollow(path)
    try:
        observed = os.fstat(directory_fd)
        entries = sorted(os.listdir(directory_fd))
    finally:
        os.close(directory_fd)
    return {
        "path": str(path),
        "uid": observed.st_uid,
        "gid": observed.st_gid,
        "mode": f"{stat.S_IMODE(observed.st_mode):04o}",
        "device_id": observed.st_dev,
        "inode": observed.st_ino,
        "entries": entries,
    }


def _simulation_ports() -> List[int]:
    listening = safety._listening_ports_from_proc()
    return sorted(set(listening).intersection(safety.SIMULATION_PORTS))


def collect_snapshot(policy: Mapping[str, Any], phase: str) -> Dict[str, Any]:
    binary = binary_observation(policy)
    processes = process_snapshot(binary)
    return {
        "phase": phase,
        "boot_id_sha256": _boot_hash(),
        "runtime_uid": os.getuid(),
        "runtime_gid": os.getgid(),
        "runtime_thread_count": _thread_count(),
        "mem_available_bytes": _mem_available(),
        "qemu_kvm_service": install_gate.service_observation(),
        "autostart_link_exists": install_gate.QEMU_KVM_AUTOSTART_LINK.exists(),
        "autostart_link_is_symlink": install_gate.QEMU_KVM_AUTOSTART_LINK.is_symlink(),
        "ksm_value": install_gate.read_ksm(),
        "simulation_ports": _simulation_ports(),
        "package_state": _package_state(),
        "scratch": _directory_observation(SCRATCH_DIR),
        "lock": _directory_observation(LOCK_PATH),
        "implementation": _implementation_evidence(),
        **processes,
        "binary": binary,
    }


def snapshot_errors(snapshot: Mapping[str, Any], policy: Mapping[str, Any]) -> List[str]:
    errors = []
    if snapshot["boot_id_sha256"] != policy["boot_session"]["boot_id_sha256"]:
        errors.append("boot session identity differs")
    if (snapshot["runtime_uid"], snapshot["runtime_gid"]) != (1000, 1000):
        errors.append("runtime UID/GID differs")
    if snapshot["runtime_thread_count"] != 1:
        errors.append("binary-probe gate process is not single-threaded")
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
    package_state = snapshot["package_state"]
    if package_state["required_package"] != EXPECTED_QEMU_PACKAGE:
        errors.append("QEMU package identity differs")
    if package_state["qemu_namespace_installed"] != list(install_gate.EXPECTED_QEMU_NAMESPACE):
        errors.append("installed QEMU namespace differs")
    if package_state["dpkg_verify"] != {
        "returncode": 0,
        "stdout_empty": True,
        "stderr_empty": True,
    }:
        errors.append("dpkg verification is not clean")
    if package_state["binary_dpkg_owner"] != "qemu-system-x86":
        errors.append("QEMU binary package owner differs")
    if package_state["binary_diverted"] is not False:
        errors.append("QEMU binary is diverted")
    scratch = snapshot["scratch"]
    if any(
        (
            scratch["path"] != str(SCRATCH_DIR),
            scratch["uid"] != 1000,
            scratch["gid"] != 1000,
            scratch["mode"] != "0750",
            scratch["entries"] != [],
        )
    ):
        errors.append("binary-probe scratch directory is not empty and frozen")
    lock = snapshot["lock"]
    if any(
        (
            lock["path"] != str(LOCK_PATH),
            lock["uid"] != 1000,
            lock["gid"] != 1000,
            lock["mode"] != "0750",
            lock["entries"] != [],
        )
    ):
        errors.append("binary-probe lock directory is not empty and frozen")
    return sorted(set(errors))


def stable_snapshot_projection(snapshot: Mapping[str, Any]) -> Dict[str, Any]:
    """Fields that must not drift around the one exact exec."""

    return {
        key: snapshot[key]
        for key in (
            "boot_id_sha256",
            "runtime_uid",
            "runtime_gid",
            "runtime_thread_count",
            "qemu_kvm_service",
            "autostart_link_exists",
            "autostart_link_is_symlink",
            "ksm_value",
            "qemu_processes",
            "simulation_processes",
            "simulation_ports",
            "process_visibility_errors",
            "process_visibility_exceptions",
            "package_state",
            "scratch",
            "lock",
            "implementation",
            "binary",
        )
    }


def validate_policy_semantics(policy: Mapping[str, Any]) -> None:
    require(policy["admission_scope"] == "EXACT_QEMU_BINARY_LOAD_AND_VERSION_ONLY", "policy scope differs")
    predecessor = policy["predecessor_install_receipt"]
    require((predecessor["path"], predecessor["file_sha256"], predecessor["receipt_hash"]) == (str(PREDECESSOR_RECEIPT), PREDECESSOR_FILE_SHA256, PREDECESSOR_INNER_HASH), "predecessor receipt binding differs")
    require(predecessor["preserved"] is True and predecessor["execution_authorization_inherited"] is False, "predecessor semantics differ")
    require(policy["boot_session"] == {"boot_id_sha256": BOOT_ID_SHA256, "uid": 1000, "unreadable_process_allowlist_exact": [dict(item) for item in UNREADABLE_PROCESS_ALLOWLIST]}, "boot-session binding differs")
    binary = policy["binary"]
    require(
        binary
        == {
            "path": str(QEMU),
            "dpkg_package": "qemu-system-x86",
            "dpkg_version": "1:4.2-3ubuntu6.30+esm3",
            "sha256": EXPECTED_QEMU_SHA256,
            "size": 16287496,
            "mode": "0755",
            "uid": 0,
            "gid": 0,
            "security_capabilities": [],
            "argv_exact": list(EXACT_QEMU_ARGV),
            "expected_version_line": "QEMU emulator version 4.2.1 (Debian 1:4.2-3ubuntu6.30+esm3)",
        },
        "QEMU binary policy object differs",
    )
    require(tuple(binary["argv_exact"]) == EXACT_QEMU_ARGV, "QEMU argv differs")
    require(binary["path"] == str(QEMU) and binary["sha256"] == EXPECTED_QEMU_SHA256, "QEMU binary policy differs")
    require(
        (binary["dpkg_package"], binary["dpkg_version"])
        == ("qemu-system-x86", "1:4.2-3ubuntu6.30+esm3"),
        "QEMU package policy differs",
    )
    require(
        binary["expected_version_line"]
        == "QEMU emulator version 4.2.1 (Debian 1:4.2-3ubuntu6.30+esm3)",
        "QEMU version output policy differs",
    )
    boundary = policy["execution_boundary"]
    require(
        boundary
        == {
            "self_check_executes_qemu": False,
            "run_probe_is_only_execution_entry": True,
            "cwd": str(SCRATCH_DIR),
            "environment_exact": dict(EXACT_ENV),
            "stdin": "/dev/null",
            "stdout_max_bytes": 4096,
            "stderr_max_bytes": 4096,
            "timeout_seconds": 5,
            "term_grace_seconds": 1,
            "start_new_session": True,
            "fork_exec_launcher": True,
            "subprocess_preexec_fn": False,
            "parent_death_signal": "SIGKILL",
            "parent_ack_before_exec": True,
            "signals_reset_before_exec": True,
            "signal_mask_cleared_before_exec": True,
            "close_fds": True,
            "no_new_privs": True,
            "umask": "0077",
            "rlimits": {
                "core_bytes": 0,
                "cpu_seconds_soft": 2,
                "cpu_seconds_hard": 3,
                "file_bytes": 8192,
                "open_files": 64,
                "address_space_bytes": 1073741824,
            },
            "same_open_fd_hash_and_exec": True,
            "exec_path_template": "/proc/self/fd/{fd}",
            "exclusive_lock": str(LOCK_PATH),
            "lock_creates_file": False,
            "scratch_must_be_empty_and_unchanged": True,
            "package_state_revalidated": True,
            "ab_stable_projection_required_equal": True,
            "bc_stable_projection_required_equal": True,
            "snapshot_sequence": ["A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP"],
            "minimum_mem_available_bytes": MIN_MEM_AVAILABLE,
        },
        "execution-boundary policy object differs",
    )
    require(boundary["self_check_executes_qemu"] is False and boundary["run_probe_is_only_execution_entry"] is True, "execution entry policy differs")
    require(boundary["cwd"] == str(SCRATCH_DIR) and boundary["environment_exact"] == EXACT_ENV, "execution environment differs")
    require(boundary["same_open_fd_hash_and_exec"] is True and boundary["exec_path_template"] == "/proc/self/fd/{fd}", "FD execution binding differs")
    require(boundary["exclusive_lock"] == str(LOCK_PATH), "lock path differs")
    require(
        boundary["lock_creates_file"] is False
        and boundary["scratch_must_be_empty_and_unchanged"] is True
        and boundary["package_state_revalidated"] is True
        and boundary["ab_stable_projection_required_equal"] is True
        and boundary["bc_stable_projection_required_equal"] is True,
        "runtime stability policy differs",
    )
    require(
        boundary["rlimits"]
        == {
            "core_bytes": 0,
            "cpu_seconds_soft": 2,
            "cpu_seconds_hard": 3,
            "file_bytes": 8192,
            "open_files": 64,
            "address_space_bytes": 1073741824,
        },
        "resource-limit policy differs",
    )
    require(tuple(boundary["snapshot_sequence"]) == ("A_PRELOCKED", "B_PREEXEC", "C_POSTCLEANUP"), "snapshot sequence differs")
    require(all(value is False for value in policy["forbidden_effects"].values()), "policy admits a forbidden effect")
    require(
        policy["receipt"] == {
            "directory": str(safety.APPROVED_ROOT / "audits/sandbox"),
            "filename_exact": RECEIPT_NAME,
            "create_new": True,
            "maximum_successful_receipts": 1,
            "pass_only": True,
            "failure_receipt_forbidden": True,
            "pii_and_cmdline_forbidden": True,
            "authorization_reusable": False,
        },
        "receipt policy differs",
    )


def verify_predecessor(policy: Mapping[str, Any]) -> Dict[str, Any]:
    expected = policy["predecessor_install_receipt"]
    try:
        receipt, file_hash = read_json_and_hash(Path(expected["path"]))
        inner = receipt.get("receipt_hash")
        core = dict(receipt)
        core.pop("receipt_hash", None)
        self_hash_valid = isinstance(inner, str) and inner == safety.canonical_hash(core)
    except (OSError, QemuBinaryProbeError, ValueError):
        file_hash, inner, self_hash_valid = None, None, False
    return {"path": expected["path"], "expected_file_sha256": expected["file_sha256"], "actual_file_sha256": file_hash, "expected_receipt_hash": expected["receipt_hash"], "actual_receipt_hash": inner, "self_hash_valid": self_hash_valid}


def predecessor_errors(observation: Mapping[str, Any]) -> List[str]:
    if not observation["self_hash_valid"]:
        return ["predecessor receipt self-hash is invalid"]
    if observation["actual_file_sha256"] != observation["expected_file_sha256"] or observation["actual_receipt_hash"] != observation["expected_receipt_hash"]:
        return ["predecessor receipt identity differs"]
    return []


def validate_runtime_paths() -> None:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    require(SCRATCH_DIR == root / "scratch/vm" and LOCK_PATH == root / "locks", "runtime paths escaped approved root")
    for directory in (SCRATCH_DIR, LOCK_PATH):
        fd = safety.open_directory_nofollow(directory)
        os.close(fd)
    observed = os.lstat(LOCK_PATH)
    require(
        stat.S_ISDIR(observed.st_mode)
        and not LOCK_PATH.is_symlink()
        and observed.st_uid == os.getuid(),
        "lock directory identity differs",
    )


def _snapshot_record(snapshot: Mapping[str, Any], policy: Mapping[str, Any]) -> Dict[str, Any]:
    canonical = safety.canonical_bytes(snapshot).decode("utf-8")
    return {"phase": snapshot["phase"], "sha256": safety.canonical_hash(snapshot), "canonical": canonical, "invariants_pass": not snapshot_errors(snapshot, policy)}


def _implementation_evidence() -> Dict[str, str]:
    paths = {
        "gate_path": Path(__file__).resolve(),
        "install_gate_path": Path(install_gate.__file__).resolve(),
        "safety_gate_path": Path(safety.__file__).resolve(),
    }
    evidence: Dict[str, str] = {}
    for key, path in paths.items():
        try:
            relative = path.relative_to(PACKAGE_ROOT)
        except ValueError as exc:
            raise QemuBinaryProbeError(f"implementation escaped package root: {path}") from exc
        digest, _ = install_gate.file_hash_and_observation(path)
        evidence[key] = str(relative)
        evidence[key.replace("_path", "_sha256")] = digest
    return evidence


def _base_report(policy: Mapping[str, Any], policy_hash: str, admission_hash: str, receipt_schema_hash: str, predecessor: Mapping[str, Any], snapshots: Sequence[Mapping[str, Any]], probe_result: Optional[Mapping[str, Any]], qemu_executed: bool, errors: Sequence[str], status: str) -> Dict[str, Any]:
    stable_hashes = [
        safety.canonical_hash(stable_snapshot_projection(snapshot))
        for snapshot in snapshots
    ]
    core: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION, "document_type": DOCUMENT_TYPE, "created_utc": utc_now(),
        "development_only": True, "formal": False, "physical_primary_eligible": False,
        "policy": {"path": POLICY_RELATIVE, "sha256": policy_hash, "admission_schema_path": ADMISSION_SCHEMA_RELATIVE, "admission_schema_sha256": admission_hash, "receipt_schema_path": RECEIPT_SCHEMA_RELATIVE, "receipt_schema_sha256": receipt_schema_hash},
        "implementation": dict(snapshots[0]["implementation"]),
        "predecessor_install_receipt": dict(predecessor),
        "snapshot_sequence": [_snapshot_record(snapshot, policy) for snapshot in snapshots],
        "stability": {
            "stable_projection_sha256": stable_hashes,
            "ab_equal": len(stable_hashes) >= 2 and stable_hashes[0] == stable_hashes[1],
            "bc_equal": len(stable_hashes) >= 3 and stable_hashes[1] == stable_hashes[2],
        },
        "probe_result": None if probe_result is None else dict(probe_result),
        "qemu_executed": qemu_executed, "qemu_img_executed": False, "vm_started": False,
        "machine_initialized": False, "network_backend_initialized": False,
        "network_namespace_created": False, "filesystem_sandbox_claimed": False,
        "image_created": False, "build_started": False, "gencase_started": False,
        "upstream_code_executed": False,
        "execution_was_explicitly_requested": qemu_executed,
        "future_execution_authorized": False, "authorization_reusable": False,
        "status": status, "errors": list(errors),
    }
    return dict(core, receipt_hash=safety.canonical_hash(core))


def load_contract() -> Tuple[Dict[str, Any], str, Dict[str, Any], str, Dict[str, Any], str]:
    policy, policy_hash = read_json_and_hash(POLICY_PATH)
    admission, admission_hash = read_json_and_hash(ADMISSION_SCHEMA_PATH)
    receipt_schema, receipt_hash = read_json_and_hash(RECEIPT_SCHEMA_PATH)
    validate_with_schema(policy, admission, "binary-probe admission")
    validate_policy_semantics(policy)
    return policy, policy_hash, admission, admission_hash, receipt_schema, receipt_hash


def validate_report_semantics(report: Mapping[str, Any], policy: Mapping[str, Any], *, receipt: bool) -> None:
    core = dict(report)
    actual_hash = core.pop("receipt_hash")
    require(actual_hash == safety.canonical_hash(core), "result self-hash differs")
    require(not report["errors"], "PASS result contains errors")
    current_policy, policy_hash = read_json_and_hash(POLICY_PATH)
    _, admission_hash = read_json_and_hash(ADMISSION_SCHEMA_PATH)
    receipt_schema, receipt_schema_hash = read_json_and_hash(RECEIPT_SCHEMA_PATH)
    require(current_policy == policy, "result policy object differs from current policy")
    require(
        report["policy"]
        == {
            "path": POLICY_RELATIVE,
            "sha256": policy_hash,
            "admission_schema_path": ADMISSION_SCHEMA_RELATIVE,
            "admission_schema_sha256": admission_hash,
            "receipt_schema_path": RECEIPT_SCHEMA_RELATIVE,
            "receipt_schema_sha256": receipt_schema_hash,
        },
        "result is detached from policy or schemas",
    )
    require(
        report["implementation"] == _implementation_evidence(),
        "result is detached from gate implementation",
    )
    require(
        report["predecessor_install_receipt"] == verify_predecessor(policy)
        and not predecessor_errors(report["predecessor_install_receipt"]),
        "result predecessor receipt is detached or not admissible",
    )
    require(all(item["invariants_pass"] for item in report["snapshot_sequence"]), "snapshot invariant failed")
    strict_snapshot_schema = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$defs": receipt_schema["$defs"],
        "$ref": "#/$defs/snapshot",
    }
    for record in report["snapshot_sequence"]:
        snapshot = json.loads(record["canonical"], object_pairs_hook=install_gate._reject_duplicate_keys, parse_constant=install_gate._reject_nonfinite)
        validate_with_schema(snapshot, strict_snapshot_schema, "canonical runtime snapshot")
        require(snapshot["phase"] == record["phase"], "snapshot phase is detached")
        require(safety.canonical_hash(snapshot) == record["sha256"], "snapshot hash is detached")
        require(not snapshot_errors(snapshot, policy), "canonical snapshot is not admissible")
    stability = report["stability"]
    expected_stable_hashes = [
        safety.canonical_hash(
            stable_snapshot_projection(
                json.loads(
                    record["canonical"],
                    object_pairs_hook=install_gate._reject_duplicate_keys,
                    parse_constant=install_gate._reject_nonfinite,
                )
            )
        )
        for record in report["snapshot_sequence"]
    ]
    require(
        stability["stable_projection_sha256"] == expected_stable_hashes,
        "stable projection hashes are detached",
    )
    require(
        stability["ab_equal"] is True
        and stability["bc_equal"] is True
        and len(set(expected_stable_hashes)) == 1,
        "runtime state drifted across A/B/C",
    )
    require(
        report["qemu_img_executed"] is False
        and report["vm_started"] is False
        and report["machine_initialized"] is False
        and report["network_backend_initialized"] is False
        and report["network_namespace_created"] is False
        and report["filesystem_sandbox_claimed"] is False
        and report["image_created"] is False
        and report["build_started"] is False
        and report["gencase_started"] is False
        and report["upstream_code_executed"] is False
        and report["future_execution_authorized"] is False
        and report["authorization_reusable"] is False,
        "result grants a forbidden effect or future authority",
    )
    if receipt:
        require(report["status"] == "PASS_BINARY_VERSION_PROBE_ONLY" and report["qemu_executed"] is True, "receipt does not prove exactly one probe")
        result = report["probe_result"]
        require(isinstance(result, dict), "probe result is missing")
        require(tuple(result["argv"]) == EXACT_QEMU_ARGV and result["environment"] == EXACT_ENV, "probe execution contract differs")
        require(
            result["exec_succeeded"] is True
            and result["returncode"] == 0
            and result["signal"] is None
            and result["timed_out"] is False
            and result["stderr_bytes"] == 0
            and result["stderr_text"] == ""
            and result["finished_monotonic_ns"] >= result["started_monotonic_ns"],
            "probe did not exit cleanly",
        )
        require(
            hashlib.sha256(result["stdout_text"].encode("utf-8")).hexdigest()
            == result["stdout_sha256"]
            and hashlib.sha256(result["stderr_text"].encode("utf-8")).hexdigest()
            == result["stderr_sha256"],
            "probe output hashes are detached",
        )
        require(
            result["stdout_bytes"] == len(result["stdout_text"].encode("utf-8"))
            and result["stderr_bytes"] == len(result["stderr_text"].encode("utf-8")),
            "probe output byte counts are detached",
        )
        require(result["pid"] == result["pgid"], "probe PID/PGID ownership differs")
        require(
            result["cleanup"]
            == {"term_sent": False, "kill_sent": False, "owned_process_group_residual": False},
            "successful probe cleanup evidence differs",
        )
        require(result["version_line"] == policy["binary"]["expected_version_line"], "QEMU version line differs")
        canonical_snapshots = [
            json.loads(
                record["canonical"],
                object_pairs_hook=install_gate._reject_duplicate_keys,
                parse_constant=install_gate._reject_nonfinite,
            )
            for record in report["snapshot_sequence"]
        ]
        require(
            result["binary_sha256"] == policy["binary"]["sha256"]
            and all(
                snapshot["binary"]["sha256"] == result["binary_sha256"]
                for snapshot in canonical_snapshots
            ),
            "probe binary identity is detached from policy or snapshots",
        )
        require(result["hardening"] == {
            "start_new_session": True,
            "fork_exec_launcher": True,
            "subprocess_preexec_fn": False,
            "parent_death_signal": "SIGKILL",
            "parent_ack_before_exec": True,
            "signals_reset_before_exec": True,
            "signal_mask_cleared_before_exec": True,
            "close_fds": True,
            "no_new_privs": True,
            "umask": "0077",
            "same_open_fd_hash_and_exec": True,
            "rlimits": dict(policy["execution_boundary"]["rlimits"]),
        }, "probe hardening evidence differs")
        require(
            report["execution_was_explicitly_requested"] is True,
            "probe receipt lacks explicit execution request",
        )
    else:
        require(report["status"] == "PASS_READY_EXECUTION_NOT_PERFORMED" and report["qemu_executed"] is False and report["probe_result"] is None, "self-check unexpectedly records execution")
        require(
            report["execution_was_explicitly_requested"] is False,
            "self-check claims an execution request",
        )


def build_self_check() -> Dict[str, Any]:
    validate_runtime_paths()
    policy, policy_hash, _, admission_hash, receipt_schema, receipt_schema_hash = load_contract()
    predecessor = verify_predecessor(policy)
    snapshots = [collect_snapshot(policy, phase) for phase in policy["execution_boundary"]["snapshot_sequence"]]
    errors = predecessor_errors(predecessor)
    for snapshot in snapshots:
        errors.extend(f"{snapshot['phase']}: {item}" for item in snapshot_errors(snapshot, policy))
    stable_hashes = [
        safety.canonical_hash(stable_snapshot_projection(snapshot))
        for snapshot in snapshots
    ]
    if len(set(stable_hashes)) != 1:
        errors.append("runtime state drifted across read-only A/B/C collection")
    status = "PASS_READY_EXECUTION_NOT_PERFORMED" if not errors else "NO_GO"
    report = _base_report(policy, policy_hash, admission_hash, receipt_schema_hash, predecessor, snapshots, None, False, errors, status)
    validate_with_schema(report, receipt_schema, "binary-probe self-check")
    if not errors:
        validate_report_semantics(report, policy, receipt=False)
    return report


class OwnedChild:
    """One directly forked child; wait() is only called after group quiescence."""

    def __init__(
        self,
        pid: int,
        stdout: BinaryIO,
        stderr: BinaryIO,
        exec_error: BinaryIO,
        identity: Mapping[str, int],
        deadline_monotonic: float,
    ) -> None:
        self.pid = pid
        self.stdout = stdout
        self.stderr = stderr
        self.exec_error = exec_error
        self.identity = dict(identity)
        self.deadline_monotonic = deadline_monotonic
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
                    raise QemuBinaryProbeError("probe child entered an unsupported wait state")
                return self.returncode
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(list(EXACT_QEMU_ARGV), timeout)
            time.sleep(0.005)

    def close_pipes(self) -> None:
        for stream in (self.stdout, self.stderr, self.exec_error):
            try:
                stream.close()
            except OSError:
                pass


def _child_parent_death_guard(parent_pid: int) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGKILL, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    if os.getppid() != parent_pid:
        raise OSError("parent disappeared before PDEATHSIG was armed")


def _child_hardening() -> None:
    os.umask(0o077)
    limits = (
        (resource.RLIMIT_CORE, 0, 0),
        (resource.RLIMIT_CPU, 2, 3),
        (resource.RLIMIT_FSIZE, 8192, 8192),
        (resource.RLIMIT_NOFILE, 64, 64),
        (resource.RLIMIT_AS, 1073741824, 1073741824),
    )
    for limit, soft, hard in limits:
        resource.setrlimit(limit, (soft, hard))
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def _reset_child_signals() -> None:
    reset = [
        signal.SIGHUP,
        signal.SIGINT,
        signal.SIGQUIT,
        signal.SIGPIPE,
        signal.SIGTERM,
        signal.SIGCHLD,
        signal.SIGALRM,
    ]
    for optional_name in ("SIGXFZ", "SIGXFSZ"):
        if hasattr(signal, optional_name):
            reset.append(getattr(signal, optional_name))
    for signum in reset:
        signal.signal(signum, signal.SIG_DFL)
    if hasattr(signal, "pthread_sigmask"):
        signal.pthread_sigmask(signal.SIG_SETMASK, [])


def _close_child_fds(keep: Sequence[int]) -> None:
    keep_set = set(keep)
    for name in os.listdir("/proc/self/fd"):
        if not name.isdigit():
            continue
        descriptor = int(name)
        if descriptor <= 2 or descriptor in keep_set:
            continue
        try:
            os.close(descriptor)
        except OSError:
            pass


def _write_child_error(fd: int, error: BaseException) -> None:
    error_number = error.errno if isinstance(error, OSError) and error.errno else 255
    payload = f"E{int(error_number)}\n".encode("ascii", errors="strict")[:32]
    try:
        os.write(fd, payload)
    except OSError:
        pass


def _child_exec(
    parent_pid: int,
    qemu_fd: int,
    scratch_fd: int,
    stdout_write: int,
    stderr_write: int,
    exec_error_write: int,
    ready_write: int,
    ack_read: int,
) -> None:
    try:
        _child_parent_death_guard(parent_pid)
        os.setsid()
        os.write(ready_write, b"R")
        require(os.read(ack_read, 1) == b"A", "parent did not authorize child exec")
        _reset_child_signals()
        os.fchdir(scratch_fd)
        _child_hardening()
        devnull = os.open("/dev/null", os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
        try:
            os.dup2(devnull, 0, inheritable=True)
            os.dup2(stdout_write, 1, inheritable=True)
            os.dup2(stderr_write, 2, inheritable=True)
        finally:
            if devnull > 2:
                os.close(devnull)
        _close_child_fds((qemu_fd, exec_error_write))
        os.execve(f"/proc/self/fd/{qemu_fd}", list(EXACT_QEMU_ARGV), dict(EXACT_ENV))
    except BaseException as exc:
        _write_child_error(exec_error_write, exc)
        os._exit(127)


def _pid_group_record(entry: Path) -> Optional[Dict[str, Any]]:
    def parse(raw: str) -> Dict[str, Any]:
        try:
            pid_text, remainder = raw.split(" (", 1)
            fields = remainder.rsplit(") ", 1)[1].split()
            record = {
                "pid": int(pid_text),
                "state": fields[0],
                "ppid": int(fields[1]),
                "pgrp": int(fields[2]),
                "starttime_ticks": int(fields[19]),
            }
        except (IndexError, ValueError) as exc:
            raise QemuBinaryProbeError(f"malformed process-group identity for PID {entry.name}") from exc
        require(record["pid"] == int(entry.name), "process stat PID differs from proc entry")
        return record

    try:
        raw_before = (entry / "stat").read_text(encoding="ascii", errors="strict")
        first = parse(raw_before)
        raw_after = (entry / "stat").read_text(encoding="ascii", errors="strict")
        second = parse(raw_after)
    except (FileNotFoundError, ProcessLookupError):
        return None
    require(
        (first["pid"], first["pgrp"], first["starttime_ticks"])
        == (second["pid"], second["pgrp"], second["starttime_ticks"]),
        "process-group identity changed during observation",
    )
    return second


def _process_group_members(pgid: int) -> List[Dict[str, Any]]:
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        record = _pid_group_record(entry)
        if record is not None and record["pgrp"] == pgid:
            members.append(record)
    return sorted(members, key=lambda item: item["pid"])


def _only_leader_zombie(process: OwnedChild, members: Sequence[Mapping[str, Any]]) -> bool:
    return (
        len(members) == 1
        and members[0]["pid"] == process.pid
        and members[0]["state"] == "Z"
        and members[0]["starttime_ticks"] == process.identity["starttime_ticks"]
    )


def _validate_owned_group_before_signal(process: OwnedChild) -> None:
    leader = _pid_group_record(Path("/proc") / str(process.pid))
    require(leader is not None, "owned group leader disappeared before cleanup")
    require(
        leader["starttime_ticks"] == process.identity["starttime_ticks"],
        "probe PID was reused; refusing to signal",
    )
    require(leader["pgrp"] == process.identity["pgid"] == process.pid, "probe process group identity changed")


def _terminate_owned_group(process: OwnedChild) -> None:
    _validate_owned_group_before_signal(process)
    term_sent = False
    kill_sent = False
    try:
        os.killpg(process.identity["pgid"], signal.SIGTERM)
        term_sent = True
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + TERM_GRACE_SECONDS
    members = _process_group_members(process.identity["pgid"])
    while not _only_leader_zombie(process, members) and time.monotonic() < deadline:
        time.sleep(0.01)
        members = _process_group_members(process.identity["pgid"])
    if not _only_leader_zombie(process, members):
        _validate_owned_group_before_signal(process)
        try:
            os.killpg(process.identity["pgid"], signal.SIGKILL)
            kill_sent = True
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + TERM_GRACE_SECONDS
        members = _process_group_members(process.identity["pgid"])
        while not _only_leader_zombie(process, members) and time.monotonic() < deadline:
            time.sleep(0.01)
            members = _process_group_members(process.identity["pgid"])
    require(
        _only_leader_zombie(process, members),
        "CRITICAL_RESIDUAL_REVIEW_REQUIRED: owned process group survived cleanup",
    )
    process.wait(timeout=TERM_GRACE_SECONDS)
    require(
        not _process_group_members(process.identity["pgid"]),
        "CRITICAL_RESIDUAL_REVIEW_REQUIRED: process group remained after leader reap",
    )
    process.identity["term_sent"] = int(term_sent)
    process.identity["kill_sent"] = int(kill_sent)


def _ensure_probe_stopped(process: OwnedChild) -> None:
    if process.returncode is not None:
        require(not _process_group_members(process.pid), "reaped probe left a process group")
        return
    _terminate_owned_group(process)


def _kill_unready_direct_child(pid: int) -> None:
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        os.waitpid(pid, 0)
    except ChildProcessError:
        pass


def _pipe_cloexec() -> Tuple[int, int]:
    read_fd, write_fd = os.pipe2(getattr(os, "O_CLOEXEC", 0))
    require(read_fd > 2 and write_fd > 2, "probe pipe collided with standard descriptors")
    return read_fd, write_fd


def _spawn_probe(qemu_fd: int, scratch_fd: int) -> OwnedChild:
    require(_thread_count() == 1, "fork-based probe launcher requires one parent thread")
    require(
        qemu_fd > 2 and scratch_fd > 2 and qemu_fd != scratch_fd,
        "audited QEMU/scratch descriptors collided with standard descriptors",
    )
    parent_pid = os.getpid()
    started_ns = time.monotonic_ns()
    deadline = time.monotonic() + TIMEOUT_SECONDS
    stdout_read, stdout_write = _pipe_cloexec()
    stderr_read, stderr_write = _pipe_cloexec()
    exec_error_read, exec_error_write = _pipe_cloexec()
    ready_read, ready_write = _pipe_cloexec()
    ack_read, ack_write = _pipe_cloexec()
    all_fds = (stdout_read, stdout_write, stderr_read, stderr_write, exec_error_read, exec_error_write, ready_read, ready_write, ack_read, ack_write)
    try:
        pid = os.fork()
    except BaseException:
        for descriptor in all_fds:
            os.close(descriptor)
        raise
    if pid == 0:
        for descriptor in (stdout_read, stderr_read, exec_error_read, ready_read, ack_write):
            os.close(descriptor)
        _child_exec(parent_pid, qemu_fd, scratch_fd, stdout_write, stderr_write, exec_error_write, ready_write, ack_read)
        os._exit(127)
    for descriptor in (stdout_write, stderr_write, exec_error_write, ready_write, ack_read):
        os.close(descriptor)
    process: Optional[OwnedChild] = None
    ack_sent = False
    try:
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([ready_read], [], [], remaining)
        require(bool(readable) and os.read(ready_read, 1) == b"R", "probe child did not enter its owned session before deadline")
        leader = _pid_group_record(Path("/proc") / str(pid))
        require(leader is not None, "probe child disappeared before identity capture")
        require(leader["pgrp"] == pid, "probe child did not establish an owned process group")
        identity = {
            "pid": pid,
            "pgid": pid,
            "starttime_ticks": int(leader["starttime_ticks"]),
            "started_monotonic_ns": started_ns,
            "term_sent": 0,
            "kill_sent": 0,
        }
        stdout_stream = os.fdopen(stdout_read, "rb", buffering=0)
        try:
            stderr_stream = os.fdopen(stderr_read, "rb", buffering=0)
        except BaseException:
            stdout_stream.close()
            raise
        try:
            exec_error_stream = os.fdopen(exec_error_read, "rb", buffering=0)
        except BaseException:
            stdout_stream.close()
            stderr_stream.close()
            raise
        process = OwnedChild(
            pid,
            stdout_stream,
            stderr_stream,
            exec_error_stream,
            identity,
            deadline,
        )
        require(hasattr(signal, "pthread_sigmask"), "pthread signal-mask control is unavailable")
        blocked = {signal.SIGHUP, signal.SIGINT, signal.SIGQUIT, signal.SIGTERM}
        previous_mask = signal.pthread_sigmask(signal.SIG_BLOCK, blocked)
        try:
            require(
                time.monotonic() < deadline,
                "probe absolute deadline expired before exec authorization",
            )
            require(os.write(ack_write, b"A") == 1, "failed to release probe child for exec")
            ack_sent = True
            os.close(ready_read)
            ready_read = -1
            os.close(ack_write)
            ack_write = -1
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous_mask)
        require(process is not None and ack_sent, "probe child handoff did not complete")
        return process
    except BaseException:
        if process is not None:
            if ack_sent:
                _ensure_probe_stopped(process)
            else:
                _kill_unready_direct_child(pid)
            process.close_pipes()
        else:
            _kill_unready_direct_child(pid)
            for descriptor in (stdout_read, stderr_read, exec_error_read):
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        raise
    finally:
        for descriptor in (ready_read, ack_write):
            if descriptor >= 0:
                try:
                    os.close(descriptor)
                except OSError:
                    pass


def _bounded_wait(process: OwnedChild) -> Tuple[bytes, bytes]:
    selector = selectors.DefaultSelector()
    buffers = {"stdout": bytearray(), "stderr": bytearray(), "exec_error": bytearray()}
    try:
        for stream, label in ((process.stdout, "stdout"), (process.stderr, "stderr"), (process.exec_error, "exec_error")):
            os.set_blocking(stream.fileno(), False)
            selector.register(stream, selectors.EVENT_READ, label)
        while selector.get_map():
            remaining = process.deadline_monotonic - time.monotonic()
            if remaining <= 0:
                raise QemuBinaryProbeError("QEMU version probe timed out")
            for key, _ in selector.select(min(remaining, 0.1)):
                limit = 32 if key.data == "exec_error" else MAX_OUTPUT
                chunk = os.read(key.fileobj.fileno(), limit + 1)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                target = buffers[key.data]
                target.extend(chunk)
                if len(target) > limit:
                    raise QemuBinaryProbeError(f"probe {key.data} exceeded {limit} bytes")
        require(not buffers["exec_error"], f"probe exec failed: {bytes(buffers['exec_error'])!r}")
        members = _process_group_members(process.identity["pgid"])
        while not _only_leader_zombie(process, members):
            if time.monotonic() >= process.deadline_monotonic:
                raise QemuBinaryProbeError("QEMU version probe did not reach a clean exit")
            time.sleep(0.005)
            members = _process_group_members(process.identity["pgid"])
        process.wait(timeout=max(0.0, process.deadline_monotonic - time.monotonic()))
        require(not _process_group_members(process.identity["pgid"]), "probe left a descendant process group")
        return bytes(buffers["stdout"]), bytes(buffers["stderr"])
    except BaseException:
        _ensure_probe_stopped(process)
        raise
    finally:
        selector.close()
        process.close_pipes()


def execute_probe(qemu_fd: int, scratch_fd: int, binary_sha256: str, policy: Mapping[str, Any]) -> Dict[str, Any]:
    process = _spawn_probe(qemu_fd, scratch_fd)
    try:
        stdout, stderr = _bounded_wait(process)
    except BaseException:
        _ensure_probe_stopped(process)
        raise
    finished_monotonic_ns = time.monotonic_ns()
    try:
        stdout_text = stdout.decode("utf-8", errors="strict")
        stderr_text = stderr.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise QemuBinaryProbeError("probe output is not UTF-8") from exc
    lines = stdout_text.splitlines()
    returncode = int(process.returncode)
    return {
        "argv": list(EXACT_QEMU_ARGV), "environment": dict(EXACT_ENV), "cwd": str(SCRATCH_DIR),
        "binary_sha256": binary_sha256,
        "pid": process.identity["pid"], "pgid": process.identity["pgid"],
        "starttime_ticks": process.identity["starttime_ticks"],
        "started_monotonic_ns": process.identity["started_monotonic_ns"],
        "finished_monotonic_ns": finished_monotonic_ns,
        "exec_succeeded": True, "returncode": returncode,
        "signal": None if returncode >= 0 else -returncode, "timed_out": False,
        "stdout_bytes": len(stdout), "stderr_bytes": len(stderr),
        "stdout_sha256": hashlib.sha256(stdout).hexdigest(), "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        "stdout_text": stdout_text, "stderr_text": stderr_text,
        "version_line": lines[0] if lines else "",
        "cleanup": {"term_sent": False, "kill_sent": False, "owned_process_group_residual": False},
        "hardening": {
            "start_new_session": True,
            "fork_exec_launcher": True,
            "subprocess_preexec_fn": False,
            "parent_death_signal": "SIGKILL",
            "parent_ack_before_exec": True,
            "signals_reset_before_exec": True,
            "signal_mask_cleared_before_exec": True,
            "close_fds": True,
            "no_new_privs": True, "umask": "0077",
            "same_open_fd_hash_and_exec": True,
            "rlimits": dict(policy["execution_boundary"]["rlimits"]),
        },
    }


@contextmanager
def exclusive_probe_lock() -> Iterator[Dict[str, int]]:
    fd = safety.open_directory_nofollow(LOCK_PATH)
    try:
        observed = os.fstat(fd)
        require(
            stat.S_ISDIR(observed.st_mode) and observed.st_uid == os.getuid(),
            "probe lock directory identity differs",
        )
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise QemuBinaryProbeError("another binary probe gate holds the lock") from exc
        yield {"device_id": observed.st_dev, "inode": observed.st_ino}
    finally:
        os.close(fd)


def _existing_probe_receipt_residue() -> List[str]:
    audit_dir = safety.APPROVED_ROOT / "audits/sandbox"
    directory_fd = safety.open_directory_nofollow(audit_dir)
    try:
        names = sorted(os.listdir(directory_fd))
    finally:
        os.close(directory_fd)
    return [
        name
        for name in names
        if name == RECEIPT_NAME
        or name.startswith(f".{RECEIPT_NAME}.partial")
    ]


def build_probe_receipt(receipt_path: Path) -> Dict[str, Any]:
    validate_runtime_paths()
    policy, policy_hash, _, admission_hash, receipt_schema, receipt_schema_hash = load_contract()
    predecessor = verify_predecessor(policy)
    require(not predecessor_errors(predecessor), "predecessor install receipt is not admissible")
    snapshots: List[Mapping[str, Any]] = []
    with exclusive_probe_lock() as locked_directory:
        receipt_path = validate_receipt_path(receipt_path)
        require(
            not _existing_probe_receipt_residue(),
            "a prior binary-probe receipt or partial residue already exists",
        )
        snapshot_a = collect_snapshot(policy, "A_PRELOCKED")
        require(not snapshot_errors(snapshot_a, policy), "snapshot A is NO-GO")
        require(
            (snapshot_a["lock"]["device_id"], snapshot_a["lock"]["inode"])
            == (locked_directory["device_id"], locked_directory["inode"]),
            "locked directory differs from snapshot A",
        )
        snapshots.append(snapshot_a)
        fd, opened_binary = open_verified_qemu_fd(policy)
        scratch_fd: Optional[int] = None
        try:
            scratch_fd = safety.open_directory_nofollow(SCRATCH_DIR)
            snapshot_b = collect_snapshot(policy, "B_PREEXEC")
            require(not snapshot_errors(snapshot_b, policy), "snapshot B is NO-GO")
            require(
                stable_snapshot_projection(snapshot_a)
                == stable_snapshot_projection(snapshot_b),
                "runtime state drifted across A/B pre-exec collection",
            )
            require((opened_binary["device_id"], opened_binary["inode"], opened_binary["sha256"]) == (snapshot_b["binary"]["device_id"], snapshot_b["binary"]["inode"], snapshot_b["binary"]["sha256"]), "pre-exec binary identity differs from open FD")
            scratch_stat = os.fstat(scratch_fd)
            require(
                (scratch_stat.st_dev, scratch_stat.st_ino)
                == (snapshot_b["scratch"]["device_id"], snapshot_b["scratch"]["inode"]),
                "pre-opened scratch directory differs from snapshot B",
            )
            snapshots.append(snapshot_b)
            try:
                probe_result = execute_probe(fd, scratch_fd, opened_binary["sha256"], policy)
            finally:
                snapshot_c = collect_snapshot(policy, "C_POSTCLEANUP")
                snapshots.append(snapshot_c)
        finally:
            if scratch_fd is not None:
                os.close(scratch_fd)
            os.close(fd)
        require(not snapshot_errors(snapshot_c, policy), "snapshot C is NO-GO")
        require(
            stable_snapshot_projection(snapshot_b)
            == stable_snapshot_projection(snapshot_c),
            "runtime state drifted across B/C execution boundary",
        )
        require(
            probe_result["returncode"] == 0
            and probe_result["stderr_bytes"] == 0
            and probe_result["stdout_bytes"] > 0,
            "QEMU version probe did not exit cleanly: "
            f"returncode={probe_result['returncode']} signal={probe_result['signal']} "
            f"stdout_bytes={probe_result['stdout_bytes']} stderr_bytes={probe_result['stderr_bytes']}",
        )
        require(probe_result["version_line"] == policy["binary"]["expected_version_line"], "QEMU version line differs")
        report = _base_report(policy, policy_hash, admission_hash, receipt_schema_hash, predecessor, snapshots, probe_result, True, [], "PASS_BINARY_VERSION_PROBE_ONLY")
        validate_with_schema(report, receipt_schema, "binary-probe receipt")
        validate_report_semantics(report, policy, receipt=True)
        safety.atomic_write_json_new(receipt_path, report)
        return report


def validate_receipt_path(path: Path) -> Path:
    root = safety.validate_approved_root(safety.APPROVED_ROOT, require_exists=True)
    path = safety.ensure_within_approved_root(path, root=root)
    require(path.parent == root / "audits/sandbox", "receipt must be directly under audits/sandbox")
    require(path.name == RECEIPT_NAME, "receipt filename is not the current-boot singleton")
    require(not path.exists() and not path.is_symlink(), "receipt path already exists")
    partial = path.with_name(f".{path.name}.partial")
    require(not partial.exists() and not partial.is_symlink(), "receipt partial path already exists")
    require(
        not any(
            name.startswith(f".{path.name}.partial.")
            for name in os.listdir(path.parent)
        ),
        "receipt retained partial path already exists",
    )
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check", help="read-only readiness check; never executes QEMU")
    run = sub.add_parser("run-probe", help="run only the exact binary version probe and publish PASS")
    run.add_argument("--receipt", required=True, type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "self-check":
            report = build_self_check()
        else:
            receipt_path = validate_receipt_path(args.receipt)
            report = build_probe_receipt(receipt_path)
            require(report["status"] == "PASS_BINARY_VERSION_PROBE_ONLY", "failure is never published")
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if report["status"].startswith("PASS_") else 2
    except (QemuBinaryProbeError, install_gate.QemuInstallGateError, safety.LiquidSafetyError, OSError, ValueError, json.JSONDecodeError, subprocess.SubprocessError, SchemaError) as exc:
        print(f"R8_LIQUID_QEMU_BINARY_PROBE_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
