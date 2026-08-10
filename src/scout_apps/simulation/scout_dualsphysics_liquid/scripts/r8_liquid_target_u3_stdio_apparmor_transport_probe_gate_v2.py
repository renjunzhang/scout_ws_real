#!/usr/bin/env python3
"""Static-only gate for the harmless U3 stdio/AppArmor transport probe v2.

This initial revision deliberately supports only self-check.  It cannot parse
or load AppArmor policy, create a namespace or mount, start a child, write an
attempt, or execute the embedded helper.  It freezes reviewed bytes, the
versioned stdin/success-frame protocol and the intended least-authority command
so a separate successor can be reviewed one mount denial at a time.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import io
import json
import os
import stat
import struct
import sys
from pathlib import Path
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v2.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_stdio_apparmor_transport_probe_policy_v2.json"
PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-stdio-transport-probe-v2.profile"

PROBE_ID = "u3_stdio_apparmor_transport_probe_v2_20260807T141934Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{PROBE_ID}.partial"
EXECUTION_RECEIPT = LIQUID_ROOT / "audits" / f"{PROBE_ID}.execution.json"
LIFECYCLE_RECEIPT = LIQUID_ROOT / "audits" / f"{PROBE_ID}.lifecycle.json"
LIFECYCLE_FAILURE_RECEIPT = LIQUID_ROOT / "audits" / f"{PROBE_ID}.lifecycle_incomplete.json"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")

BOOTSTRAP_PROFILE = "r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z"
RUNTIME_PROFILE = "r8-liquid-u3-stdio-transport-runtime-v2-20260807t141934z"
HOST_UID = 1000
HOST_GID = 1000
TMPFS_BYTES = 67_108_864
STDIN_MAGIC = b"R8LQSTDIOPROBE2\x00"
SUCCESS_MAGIC = b"R8LQSTDIOPASSV2\x00"
MAX_SUCCESS_FRAME_BYTES = 16_384
PROFILE_SHA256 = "afe6f209d02f93ad58bbc02aa64ec08ef027bbb07203065c073fa614b9f9b396"

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sleep": (Path("/usr/bin/sleep"), "a65efec857cfac0d4f43fc53affa73794ff1d1fdcb547931d4b92a98bda8e646"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

INPUT_PAYLOADS: dict[str, bytes] = {
    "probe_alpha.bin": b"R8_STDIO_PROBE_ALPHA_V2\n" * 11,
    "probe_beta.bin": bytes(range(256)) * 2 + b"BETA_V2",
    "probe_gamma.bin": hashlib.sha256(b"r8-stdio-probe-v2-gamma-seed").digest() * 31 + b"G",
}
INPUT_CONTRACT: dict[str, tuple[int, str]] = {
    "probe_alpha.bin": (264, "5cefd89d7437670de4daa12d70d7bffd00c4d527297907baa87ec668f3ab3557"),
    "probe_beta.bin": (519, "19a64358adb59e1d78ee82a6cf1bf37990c9eb1297b93eb047dfd41657c7fa0d"),
    "probe_gamma.bin": (993, "f7a3e62ee3472c01a7dd435ab94ac63fe187140400874032b6c1c5c171fcdcbe"),
}

HELPER_SOURCE = r'''import errno
import hashlib
import json
import os
import signal
import stat
import struct
import subprocess
import sys
import time

READ_EXACT = FRAME_READ_EXACT
STREAM = FRAME_STREAM
BOOTSTRAP_PROFILE = "r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z"
RUNTIME_PROFILE = "r8-liquid-u3-stdio-transport-runtime-v2-20260807t141934z"
SUCCESS_MAGIC = b"R8LQSTDIOPASSV2\x00"
INPUTS = (
    ("probe_alpha.bin", 264, "5cefd89d7437670de4daa12d70d7bffd00c4d527297907baa87ec668f3ab3557"),
    ("probe_beta.bin", 519, "19a64358adb59e1d78ee82a6cf1bf37990c9eb1297b93eb047dfd41657c7fa0d"),
    ("probe_gamma.bin", 993, "f7a3e62ee3472c01a7dd435ab94ac63fe187140400874032b6c1c5c171fcdcbe"),
)
ENV = {"HOME": "/nonexistent", "PATH": "/usr/bin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"}


class ProbeError(RuntimeError):
    pass


def read_status(path):
    fields = {}
    with open(path, "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    try:
        result = {
            "uid": [int(value) for value in fields["Uid"].split()],
            "gid": [int(value) for value in fields["Gid"].split()],
            "groups": [int(value) for value in fields.get("Groups", "").split()],
            "capabilities": {
                key: int(fields[key], 16)
                for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")
            },
            "no_new_privs": int(fields["NoNewPrivs"]),
        }
    except (KeyError, ValueError) as exc:
        raise ProbeError("status_parse") from exc
    if result["uid"] != [0, 0, 0, 0] or result["gid"] != [0, 0, 0, 0]:
        raise ProbeError("identity")
    if result["groups"] or any(result["capabilities"].values()) or result["no_new_privs"] != 1:
        raise ProbeError("privilege_boundary")
    return result


def live_fds(expected):
    live = set()
    for name in os.listdir("/proc/self/fd"):
        try:
            descriptor = int(name)
        except ValueError:
            continue
        try:
            os.fstat(descriptor)
        except OSError as exc:
            if exc.errno == errno.EBADF:
                continue
            raise
        live.add(descriptor)
    if live != set(expected):
        raise ProbeError("fd_leak")
    return sorted(live)


def verify_work_tmpfs():
    filesystem = os.statvfs("/work")
    total_bytes = filesystem.f_blocks * filesystem.f_frsize
    if total_bytes != 67_108_864:
        raise ProbeError("tmpfs_size")
    matches = []
    with open("/proc/self/mountinfo", "r", encoding="ascii") as source:
        for line in source:
            try:
                left, right = line.rstrip("\n").split(" - ", 1)
            except ValueError as exc:
                raise ProbeError("mountinfo_parse") from exc
            fields = left.split()
            filesystem_fields = right.split()
            if len(fields) < 6 or len(filesystem_fields) < 3:
                raise ProbeError("mountinfo_fields")
            if fields[4] == "/work":
                matches.append(
                    {
                        "filesystem_type": filesystem_fields[0],
                        "mount_options": fields[5].split(","),
                    }
                )
    if len(matches) != 1 or matches[0]["filesystem_type"] != "tmpfs":
        raise ProbeError("tmpfs_identity")
    return {
        "filesystem_type": "tmpfs",
        "total_bytes": total_bytes,
        "mount_options": matches[0]["mount_options"],
    }


def write_inputs():
    os.mkdir("/work/input", 0o700)
    observed = {}
    for name, expected_size, expected_hash in INPUTS:
        header = READ_EXACT(40)
        announced_size = struct.unpack(">Q", header[:8])[0]
        announced_hash = header[8:].hex()
        if announced_size != expected_size or announced_hash != expected_hash:
            raise ProbeError("input_header")
        descriptor = os.open(
            "/work/input/" + name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o400,
        )
        digest = hashlib.sha256()
        remaining = expected_size
        try:
            while remaining:
                block = READ_EXACT(min(4096, remaining))
                digest.update(block)
                remaining -= len(block)
                view = memoryview(block)
                while view:
                    count = os.write(descriptor, view)
                    if count <= 0:
                        raise ProbeError("short_write")
                    view = view[count:]
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o400)
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise ProbeError("input_inode")
        finally:
            os.close(descriptor)
        if digest.hexdigest() != expected_hash:
            raise ProbeError("input_digest")
        observed[name] = {"size_bytes": expected_size, "sha256": expected_hash}
    if STREAM.read(1) != b"":
        raise ProbeError("trailing_input")
    STREAM.close()
    try:
        os.close(0)
    except OSError as exc:
        if exc.errno != errno.EBADF:
            raise
    read_end, write_end = os.pipe()
    try:
        os.close(write_end)
        if read_end != 0:
            os.dup2(read_end, 0, inheritable=True)
            os.close(read_end)
        else:
            os.set_inheritable(0, True)
    except BaseException:
        if read_end not in (0,):
            try:
                os.close(read_end)
            except OSError:
                pass
        raise
    if os.read(0, 1) != b"" or not os.get_inheritable(0):
        raise ProbeError("stdin_eof_pipe")
    return observed


def terminate_runtime(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        stdout, stderr = process.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        stdout, stderr = process.communicate(timeout=2)
        raise ProbeError("runtime_term_timeout")
    return stdout, stderr


def run_runtime():
    process = subprocess.Popen(
        ["/usr/bin/sleep", "30"],
        stdin=None,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/work",
        env=ENV,
        close_fds=True,
        start_new_session=True,
    )
    if process.stdin is not None or process.stdout is None or process.stderr is None:
        raise ProbeError("runtime_pipe")
    label_path = f"/proc/{process.pid}/attr/current"
    status_path = f"/proc/{process.pid}/status"
    expected_label = RUNTIME_PROFILE + " (enforce)"
    deadline = time.monotonic() + 1
    label = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        try:
            with open(label_path, "r", encoding="ascii") as source:
                label = source.read().strip()
        except FileNotFoundError:
            time.sleep(0.01)
            continue
        if label == expected_label:
            break
        time.sleep(0.01)
    if label != expected_label:
        terminate_runtime(process)
        raise ProbeError("runtime_label")
    runtime_identity = read_status(status_path)
    stdout, stderr = terminate_runtime(process)
    if process.returncode != -signal.SIGTERM or stdout or stderr:
        raise ProbeError("runtime_exit")
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        pass
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        raise ProbeError("runtime_residue")
    return {
        "label": label,
        "identity": runtime_identity,
        "returncode": process.returncode,
        "stdin": "inherited_fixed_guest_eof_pipe_fd0",
        "stdout_stderr": "internal_empty_pipes",
    }


def emit_success(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if len(payload) > 8192:
        raise ProbeError("success_payload_size")
    frame = SUCCESS_MAGIC + struct.pack(">I", len(payload)) + payload + hashlib.sha256(payload).digest()
    view = memoryview(frame)
    while view:
        count = os.write(1, view)
        if count <= 0:
            raise ProbeError("success_write")
        view = view[count:]


def main():
    bootstrap_identity = read_status("/proc/self/status")
    work_tmpfs = verify_work_tmpfs()
    observed = write_inputs()
    fds_after_stdin = live_fds({0, 1, 2})
    runtime = run_runtime()
    fds_before_success = live_fds({0, 1, 2})
    emit_success(
        {
            "status": "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V2",
            "bootstrap_profile": BOOTSTRAP_PROFILE,
            "bootstrap_identity": bootstrap_identity,
            "work_tmpfs": work_tmpfs,
            "inputs": observed,
            "host_stdin_consumed_and_fd0_replaced_with_eof_pipe": True,
            "fds_after_stdin": fds_after_stdin,
            "runtime": runtime,
            "fds_before_success": fds_before_success,
            "host_writable_mounts": [],
        }
    )
    return 0


try:
    raise SystemExit(main())
except (ProbeError, OSError, ValueError, UnicodeError, subprocess.SubprocessError):
    try:
        os.write(2, b"R8_STDIO_PROBE_V2_NO_GO\n")
    except OSError:
        pass
    raise SystemExit(2)
'''

HELPER_BYTES = HELPER_SOURCE.encode("utf-8")
HELPER_SHA256 = hashlib.sha256(HELPER_BYTES).hexdigest()
HELPER_SIZE_BYTES = len(HELPER_BYTES)

LOADER_SOURCE = f'''import hashlib,struct,sys
B=sys.stdin.buffer
def r(n):
 d=bytearray()
 while len(d)<n:
  x=B.read(n-len(d))
  if not x: raise SystemExit(91)
  d.extend(x)
 return bytes(d)
if r(16)!={STDIN_MAGIC!r}: raise SystemExit(92)
n=struct.unpack(">I",r(4))[0]
h=r(32)
if n!={HELPER_SIZE_BYTES} or h.hex()!="{HELPER_SHA256}": raise SystemExit(93)
p=r(n)
if hashlib.sha256(p).digest()!=h: raise SystemExit(94)
g={{"__name__":"__main__","FRAME_READ_EXACT":r,"FRAME_STREAM":B}}
exec(compile(p,"<r8-stdio-probe-helper-v2>","exec"),g,g)
'''
LOADER_BYTES = LOADER_SOURCE.encode("ascii")
LOADER_SHA256 = hashlib.sha256(LOADER_BYTES).hexdigest()
LOADER_SIZE_BYTES = len(LOADER_BYTES)

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A static contract mismatch that keeps the probe NO-GO."""


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def read_regular_bytes(path: Path, *, limit: int = 2 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise GateError(f"unsafe reviewed artifact: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise GateError(f"reviewed artifact exceeds limit: {path}")
    finally:
        os.close(descriptor)


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid reviewed JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"reviewed JSON is not an object: {path}")
    return value


def sha256_regular_file(path: Path, *, limit: int = 2 * 1024 * 1024) -> str:
    return hashlib.sha256(read_regular_bytes(path, limit=limit)).hexdigest()


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {
        name: {"path": str(path), "sha256": digest}
        for name, (path, digest) in TRUSTED_TOOLS.items()
    }


def require_tool(name: str) -> dict[str, Any]:
    path, expected_digest = TRUSTED_TOOLS[name]
    metadata = os.stat(path, follow_symlinks=False)
    if (
        not stat.S_ISREG(metadata.st_mode)
        or stat.S_ISLNK(metadata.st_mode)
        or metadata.st_uid != 0
        or metadata.st_gid != 0
        or not metadata.st_mode & stat.S_IXUSR
    ):
        raise GateError(f"trusted root-owned tool contract differs: {path}")
    observed = sha256_regular_file(path, limit=16 * 1024 * 1024)
    if observed != expected_digest:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": observed}


def input_policy() -> list[dict[str, Any]]:
    result = []
    for name, (size, digest) in INPUT_CONTRACT.items():
        result.append(
            {
                "name": name,
                "size_bytes": size,
                "sha256": digest,
                "guest_path": f"/work/input/{name}",
                "guest_mode": "0400",
                "classification": "FIXED_SYNTHETIC_HARMLESS_PROBE_BYTES",
            }
        )
    return result


def bwrap_argv_template() -> list[str]:
    return [
        "/usr/bin/bwrap",
        "--die-with-parent",
        "--new-session",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--uid",
        "0",
        "--gid",
        "0",
        "--cap-drop",
        "ALL",
        "--hostname",
        "r8-liquid-stdio-probe-v2",
        "--clearenv",
        "--setenv",
        "HOME",
        "/nonexistent",
        "--setenv",
        "PATH",
        "/usr/bin",
        "--setenv",
        "LC_ALL",
        "C.UTF-8",
        "--setenv",
        "LANG",
        "C.UTF-8",
        "--setenv",
        "TZ",
        "UTC",
        "--ro-bind",
        "/usr",
        "/usr",
        "--symlink",
        "usr/lib",
        "/lib",
        "--symlink",
        "usr/lib64",
        "/lib64",
        "--size",
        str(TMPFS_BYTES),
        "--tmpfs",
        "/work",
        "--proc",
        "/proc",
        "--chdir",
        "/work",
        "--",
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-c",
        "<PINNED_LOADER_SOURCE>",
    ]


def outer_argv_template() -> list[str]:
    return [
        "/usr/bin/timeout",
        "--foreground",
        "--signal=TERM",
        "--kill-after=2s",
        "15s",
        "/usr/bin/setpriv",
        "--reuid=1000",
        "--regid=1000",
        "--clear-groups",
        "--no-new-privs",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
        "--",
        "/usr/bin/aa-exec",
        "-p",
        BOOTSTRAP_PROFILE,
        "--",
        *bwrap_argv_template(),
    ]


def expected_policy() -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-target-u3-stdio-apparmor-transport-probe-policy-v2",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_STDIO_APPARMOR_TRANSPORT_PROBE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_STDIO_APPARMOR_TRANSPORT_PROBE_V2",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "static_review_authorized": True,
        "profile_parse_load_select_authorized": False,
        "harmless_probe_execution_authorized": False,
        "production_runtime_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "frozen_identity": {
            "probe_id": PROBE_ID,
            "attempt_root": str(ATTEMPT_ROOT),
            "execution_receipt": str(EXECUTION_RECEIPT),
            "lifecycle_receipt": str(LIFECYCLE_RECEIPT),
            "lifecycle_failure_receipt": str(LIFECYCLE_FAILURE_RECEIPT),
            "snapshot_root": str(SNAPSHOT_ROOT),
            "bootstrap_profile": BOOTSTRAP_PROFILE,
            "runtime_profile": RUNTIME_PROFILE,
        },
        "security_decision": {
            "transport": "VERSIONED_STDIN_FRAME_AND_SINGLE_SUCCESS_ONLY_STDOUT_FRAME",
            "removed_designs": [
                "bwrap_file_fd_input",
                "bwrap_bind_fd_output",
                "host_writable_output_mount",
            ],
            "production_design_dependency": "none_until_a_fresh_probe_success_and_fresh_runtime_review",
            "workspace_binary_execution": "forbidden",
            "gencase_or_solver_execution": "forbidden",
        },
        "reviewed_bytes": {
            "loader": {
                "delivery": "exact_python_-c_argument",
                "encoding": "ascii",
                "sha256": LOADER_SHA256,
                "size_bytes": LOADER_SIZE_BYTES,
                "maximum_size_bytes": 2048,
            },
            "helper": {
                "delivery": "stdin_after_versioned_header_then_hash_before_compile_exec",
                "encoding": "utf-8",
                "sha256": HELPER_SHA256,
                "size_bytes": HELPER_SIZE_BYTES,
                "maximum_size_bytes": 16_384,
            },
            "inputs": input_policy(),
        },
        "trusted_system_tools": trusted_tool_policy(),
        "stdio_transport": {
            "stdin_magic_hex": STDIN_MAGIC.hex(),
            "stdin_layout": [
                "magic_16",
                "helper_size_u32_be",
                "helper_sha256_raw_32",
                "helper_exact_bytes",
                "three_repetitions_of_size_u64_be_sha256_raw_32_exact_bytes",
                "strict_eof",
            ],
            "input_count": 3,
            "loader_verifies_helper_before_compile_exec": True,
            "helper_verifies_each_announced_and_observed_size_hash": True,
            "host_stdin_consumed_to_strict_eof": True,
            "original_fd0_closed": True,
            "fd0_replaced_by_guest_internal_eof_pipe": True,
            "runtime_inherits_only_eof_pipe_on_fd0": True,
            "no_pass_fds": True,
            "stdout_magic_hex": SUCCESS_MAGIC.hex(),
            "stdout_layout": [
                "success_magic_16",
                "json_size_u32_be",
                "canonical_ascii_json",
                "json_sha256_raw_32",
            ],
            "stdout_success_only": True,
            "stdout_maximum_frame_bytes": MAX_SUCCESS_FRAME_BYTES,
            "stderr_failure_marker_maximum_bytes": 64,
        },
        "identity_contract": {
            "outer_setpriv_uid_gid": [HOST_UID, HOST_GID],
            "supplementary_groups": [],
            "host_child_capability_sets_all_zero": True,
            "host_child_no_new_privs": 1,
            "guest_uid_fields": [0, 0, 0, 0],
            "guest_gid_fields": [0, 0, 0, 0],
            "guest_groups": [],
            "guest_capability_sets_all_zero": True,
            "guest_no_new_privs": 1,
            "bwrap_cap_drop": "ALL",
        },
        "isolation": {
            "network_namespace": "unshared_no_host_network",
            "pid_namespace": "unshared",
            "ipc_namespace": "unshared",
            "uts_namespace": "unshared",
            "nested_userns_disabled": True,
            "host_read_only_mounts": ["/usr"],
            "host_writable_mounts": [],
            "guest_dev": "absent",
            "guest_proc": "/proc_for_identity_label_and_fd_checks",
            "guest_work_tmpfs_bytes": TMPFS_BYTES,
            "guest_work_tmpfs_verification": "mountinfo_exact_tmpfs_and_statvfs_total_bytes_exact",
            "guest_work_tmpfs_inode_ceiling": "UNCLAIMED_BWRAP_0_9_HAS_NO_NR_INODES_OPTION",
            "guest_created_regular_files_exactly": [f"/work/input/{name}" for name in INPUT_CONTRACT],
            "bwrap_argv_template": bwrap_argv_template(),
        },
        "fixed_command": {
            "outer_argv_template": outer_argv_template(),
            "loader_placeholder_occurrences": 1,
            "loader_placeholder_replacement": "exact_LOADER_SOURCE_ascii_text",
            "environment": {
                "HOME": "/nonexistent",
                "PATH": "/usr/bin",
                "LC_ALL": "C.UTF-8",
                "LANG": "C.UTF-8",
                "TZ": "UTC",
            },
            "shell": False,
            "close_fds": True,
            "pass_fds": [],
        },
        "runtime_transition": {
            "argv": ["/usr/bin/sleep", "30"],
            "forced_rule": f"/usr/bin/sleep rpx -> {RUNTIME_PROFILE},",
            "expected_attr_current": f"{RUNTIME_PROFILE} (enforce)",
            "stdin": "inherited_guest_internal_anonymous_eof_pipe_fixed_on_fd0",
            "stdout": "guest_internal_pipe_must_be_empty",
            "stderr": "guest_internal_pipe_must_be_empty",
            "close_fds": True,
            "start_new_session": True,
            "bootstrap_sends": ["SIGTERM", "SIGKILL_IF_TERM_TIMEOUT", "SIGNAL_0_RESIDUE_CHECK"],
            "term_grace_seconds": 2,
            "expected_returncode": -15,
            "process_group_residue_allowed": False,
        },
        "resources": {
            "outer_wall_timeout_seconds": 15,
            "outer_kill_after_seconds": 2,
            "runtime_sleep_argument_seconds": 30,
            "runtime_label_observation_deadline_seconds": 1,
            "runtime_term_grace_seconds": 2,
            "stdin_frame_maximum_bytes": 65_536,
            "stdout_frame_maximum_bytes": MAX_SUCCESS_FRAME_BYTES,
            "attempts_per_identity": 1,
        },
        "mount_rule_discovery": {
            "effective_mount_rules": [],
            "sanitized_denial_receipts": [],
            "rules_may_be_added_by_analogy": False,
            "this_revision_may_be_parsed_loaded_or_selected": False,
            "this_revision_may_attempt_bwrap": False,
            "successor_requires_new_profile_policy_gate_and_attempt_identity": True,
            "one_new_exact_observed_rule_per_successor": True,
        },
        "profile_lifecycle": {
            "persistent_install": "forbidden",
            "load_source": "future_reviewed_root_owned_tmpfs_snapshot_only",
            "initial_host_sysctl_snapshot_required": True,
            "initial_loaded_profile_set_snapshot_required": True,
            "cleanup_requires_no_matching_proc_attr_current": True,
            "cleanup_requires_profile_unloaded": True,
            "cleanup_requires_sysctls_unchanged": True,
            "cleanup_requires_sudo_timestamp_cleared": True,
            "execution_receipt_never_claims_cleanup_complete": True,
            "separate_lifecycle_receipt_required": True,
        },
        "evidence_interpretation": {
            "static_pass_authorizes_execution": False,
            "future_denial_authorizes_only_one_fresh_rule_review": True,
            "future_probe_pass_authorizes_gencase": False,
            "future_probe_pass_authorizes_solver": False,
            "future_probe_pass_authorizes_production_profile_reuse": False,
            "fresh_runtime_successor_and_independent_go_required": True,
        },
        "invariants": {
            "no_sudo_in_this_gate": True,
            "no_subprocess_in_this_gate": True,
            "no_profile_parser_or_load_in_this_gate": True,
            "no_namespace_or_mount_in_this_gate": True,
            "no_attempt_or_receipt_write_in_this_gate": True,
            "no_host_writable_mount": True,
            "no_fd_transport_mount": True,
            "no_network_access": True,
            "no_workspace_binary_execution": True,
            "no_gencase_or_solver": True,
            "no_source_or_seed_read": True,
            "no_ros_or_gazebo": True,
            "no_gpu": True,
            "no_apt_driver_kernel_or_sysctl_change": True,
            "no_delete_overwrite_or_reuse": True,
        },
        "status": "STATIC_NO_GO_V2_MOUNT_RULES_UNOBSERVED_EXECUTION_FORBIDDEN",
        "next_allowed_stage": "INDEPENDENT_STATIC_REVIEW_THEN_FRESH_SINGLE_DENIAL_DISCOVERY_SUCCESSOR",
    }


def build_stdin_frame() -> bytes:
    frame = bytearray(STDIN_MAGIC)
    frame.extend(struct.pack(">I", HELPER_SIZE_BYTES))
    frame.extend(bytes.fromhex(HELPER_SHA256))
    frame.extend(HELPER_BYTES)
    for name, (expected_size, expected_hash) in INPUT_CONTRACT.items():
        payload = INPUT_PAYLOADS[name]
        if len(payload) != expected_size or hashlib.sha256(payload).hexdigest() != expected_hash:
            raise GateError(f"synthetic input contract differs: {name}")
        frame.extend(struct.pack(">Q", expected_size))
        frame.extend(bytes.fromhex(expected_hash))
        frame.extend(payload)
    if len(frame) > 65_536:
        raise GateError("frozen stdin frame exceeds its ceiling")
    return bytes(frame)


def parse_stdin_frame(frame: bytes) -> dict[str, Any]:
    source = io.BytesIO(frame)

    def exact(count: int) -> bytes:
        value = source.read(count)
        if len(value) != count:
            raise GateError("stdin frame ended early")
        return value

    if exact(16) != STDIN_MAGIC:
        raise GateError("stdin frame magic differs")
    helper_size = struct.unpack(">I", exact(4))[0]
    helper_digest = exact(32).hex()
    helper = exact(helper_size)
    if (
        helper_size != HELPER_SIZE_BYTES
        or helper_digest != HELPER_SHA256
        or hashlib.sha256(helper).hexdigest() != HELPER_SHA256
    ):
        raise GateError("stdin helper identity differs")
    inputs: dict[str, dict[str, Any]] = {}
    for name, (expected_size, expected_hash) in INPUT_CONTRACT.items():
        size = struct.unpack(">Q", exact(8))[0]
        announced_hash = exact(32).hex()
        payload = exact(size)
        if (
            size != expected_size
            or announced_hash != expected_hash
            or hashlib.sha256(payload).hexdigest() != expected_hash
        ):
            raise GateError(f"stdin payload identity differs: {name}")
        inputs[name] = {"size_bytes": size, "sha256": announced_hash}
    if source.read(1):
        raise GateError("stdin frame contains trailing data")
    return {
        "magic_hex": STDIN_MAGIC.hex(),
        "helper": {"size_bytes": helper_size, "sha256": helper_digest},
        "inputs": inputs,
        "frame_size_bytes": len(frame),
        "frame_sha256": hashlib.sha256(frame).hexdigest(),
    }


def verify_loader_and_helper() -> dict[str, Any]:
    if HELPER_SIZE_BYTES > 16_384 or LOADER_SIZE_BYTES > 2048:
        raise GateError("reviewed Python byte ceiling differs")
    try:
        ast.parse(HELPER_SOURCE, filename="<helper>", mode="exec")
        ast.parse(LOADER_SOURCE, filename="<loader>", mode="exec")
    except SyntaxError as exc:
        raise GateError("reviewed Python bytes do not parse") from exc
    for required in (
        "READ_EXACT = FRAME_READ_EXACT",
        "STREAM = FRAME_STREAM",
        '["/usr/bin/sleep", "30"]',
        "stdin=None",
        "stdout=subprocess.PIPE",
        "stderr=subprocess.PIPE",
        "close_fds=True",
        "os.close(0)",
        "read_end, write_end = os.pipe()",
        "os.close(write_end)",
        "os.dup2(read_end, 0, inheritable=True)",
        "os.set_inheritable(0, True)",
        "os.killpg(process.pid, signal.SIGTERM)",
        "os.killpg(process.pid, signal.SIGKILL)",
        "live_fds({0, 1, 2})",
        'os.statvfs("/work")',
        'open("/proc/self/mountinfo"',
        'matches[0]["filesystem_type"] != "tmpfs"',
        'expected_label = RUNTIME_PROFILE + " (enforce)"',
    ):
        if required not in HELPER_SOURCE:
            raise GateError(f"helper lacks required boundary: {required}")
    for forbidden in (
        "rPx",
        "GenCase",
        "solver",
        "subprocess.DEVNULL",
        "shell=True",
        "import socket",
        "/home/",
        "/dev/",
        "--file",
        "bind-fd",
        "pass_fds",
        "eval(",
        "__import__",
    ):
        if forbidden in HELPER_SOURCE:
            raise GateError(f"helper contains forbidden authority: {forbidden}")
    for required in (
        "hashlib.sha256(p).digest()!=h",
        "compile(p",
        "exec(",
        HELPER_SHA256,
        str(HELPER_SIZE_BYTES),
    ):
        if required not in LOADER_SOURCE:
            raise GateError(f"loader lacks required identity check: {required}")
    for forbidden in ("subprocess", "socket", "open(", "os.", "eval(", "__import__"):
        if forbidden in LOADER_SOURCE:
            raise GateError(f"loader contains forbidden surface: {forbidden}")
    frame = parse_stdin_frame(build_stdin_frame())
    return {
        "loader": {"sha256": LOADER_SHA256, "size_bytes": LOADER_SIZE_BYTES},
        "helper": {"sha256": HELPER_SHA256, "size_bytes": HELPER_SIZE_BYTES},
        "stdin_frame": frame,
    }


def effective_profile_lines(text: str) -> tuple[str, ...]:
    lines = []
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            lines.append(line)
    return tuple(lines)


def verify_profile() -> dict[str, Any]:
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    digest = hashlib.sha256(raw).hexdigest()
    if digest != PROFILE_SHA256:
        raise GateError("stdio probe profile digest differs")
    text = raw.decode("utf-8")
    for marker in (
        "STATIC_REVIEW_ONLY: DO_NOT_PARSE_LOAD_SELECT_OR_EXECUTE.",
        "Bubblewrap filesystem-mediation rules are intentionally absent.",
        "Nothing in this draft authorizes an attempt or a production run.",
    ):
        if marker not in text:
            raise GateError(f"profile safety marker is missing: {marker}")
    lines = effective_profile_lines(text)
    effective = "\n".join(lines)
    bootstrap_header = f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    runtime_header = f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    if effective.count(bootstrap_header) != 1 or effective.count(runtime_header) != 1:
        raise GateError("stdio probe label identity differs")
    transition = f"/usr/bin/sleep rpx -> {RUNTIME_PROFILE},"
    if effective.count(transition) != 1:
        raise GateError("stdio probe forced harmless transition differs")
    bootstrap, runtime = effective.split(runtime_header, 1)
    for required in (
        "/usr/bin/bwrap rix,",
        "/usr/bin/python3.12 rix,",
        transition,
        "userns create,",
        f"signal (send) set=(term,kill,exists) peer={RUNTIME_PROFILE},",
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
    ):
        if required not in bootstrap:
            raise GateError(f"bootstrap profile lacks required boundary: {required}")
    for name in INPUT_CONTRACT:
        if f"/work/input/{name} rw," not in bootstrap:
            raise GateError(f"bootstrap lacks exact harmless input path: {name}")
    for forbidden_rule_prefix in ("mount ", "remount ", "pivot_root ", "umount "):
        if any(line.startswith(forbidden_rule_prefix) for line in lines):
            raise GateError("profile guesses an unobserved filesystem-mediation rule")
    for forbidden in (
        "rPx",
        "GenCase",
        "solver",
        "/home/",
        "/dev/",
        "/opt/ros/",
        "/dev/nvidia",
        "network inet stream",
        "network inet6 stream",
        "flags=(unconfined)",
        "change_profile",
        "ux,",
        "Ux,",
    ):
        if forbidden in effective:
            raise GateError(f"profile violates forbidden boundary: {forbidden}")
    for forbidden in (
        " w,",
        " rw,",
        "userns ",
        "capability ",
        "network ",
        "/proc",
        "/dev",
        "/home",
        " rix,",
        " rpx",
        " ix,",
    ):
        if forbidden in runtime:
            raise GateError(f"runtime label has forbidden authority: {forbidden}")
    for required in (
        "/usr/bin/sleep rm,",
        f"signal (receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
    ):
        if required not in runtime:
            raise GateError(f"runtime label lacks required narrow rule: {required}")
    return {
        "path": str(PROFILE_PATH),
        "sha256": digest,
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        "effective_mount_rules": [],
        "parser_invoked": False,
        "loaded": False,
        "selected": False,
    }


def validate_schema_instance(instance: Any, schema: Mapping[str, Any], path: str = "$") -> None:
    if "const" in schema and instance != schema["const"]:
        raise GateError(f"schema const differs at {path}")
    declared_type = schema.get("type")
    if declared_type == "object":
        if not isinstance(instance, dict):
            raise GateError(f"schema object type differs at {path}")
        required = schema.get("required", [])
        properties = schema.get("properties", {})
        if not isinstance(required, list) or not isinstance(properties, dict):
            raise GateError(f"schema object contract malformed at {path}")
        if set(required) - set(instance):
            raise GateError(f"schema required fields missing at {path}")
        if schema.get("additionalProperties") is False and set(instance) - set(properties):
            raise GateError(f"schema extra fields present at {path}")
        for name, subschema in properties.items():
            if name in instance:
                if not isinstance(subschema, dict):
                    raise GateError(f"schema property malformed at {path}.{name}")
                validate_schema_instance(instance[name], subschema, f"{path}.{name}")
    elif declared_type == "string":
        if not isinstance(instance, str):
            raise GateError(f"schema string type differs at {path}")
        minimum = schema.get("minLength")
        if minimum is not None and len(instance) < int(minimum):
            raise GateError(f"schema string length differs at {path}")
    elif declared_type is not None:
        raise GateError(f"unsupported frozen schema type at {path}: {declared_type}")


def verify_argv_contract() -> dict[str, Any]:
    bwrap = bwrap_argv_template()
    outer = outer_argv_template()
    forbidden_options = ("--file", "--bind-fd", "--ro-bind-fd", "--bind", "--dev-bind", "--share-net", "--dev")
    for option in forbidden_options:
        if option in bwrap:
            raise GateError(f"stdio-only bwrap argv contains forbidden option: {option}")
    if bwrap.count("--ro-bind") != 1:
        raise GateError("stdio-only bwrap host read-only bind count differs")
    ro_index = bwrap.index("--ro-bind")
    if bwrap[ro_index + 1 : ro_index + 3] != ["/usr", "/usr"]:
        raise GateError("stdio-only bwrap host read-only bind differs")
    if bwrap.count("--size") != 1:
        raise GateError("stdio-only tmpfs size option count differs")
    size_index = bwrap.index("--size")
    if bwrap[size_index : size_index + 4] != ["--size", str(TMPFS_BYTES), "--tmpfs", "/work"]:
        raise GateError("stdio-only 64-MiB tmpfs ordering differs")
    if bwrap.count("<PINNED_LOADER_SOURCE>") != 1:
        raise GateError("loader placeholder count differs")
    if bwrap[-5:] != ["/usr/bin/python3.12", "-I", "-B", "-c", "<PINNED_LOADER_SOURCE>"]:
        raise GateError("isolated fixed Python loader command differs")
    for required in (
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--cap-drop",
    ):
        if bwrap.count(required) != 1:
            raise GateError(f"stdio-only isolation option differs: {required}")
    cap_index = bwrap.index("--cap-drop")
    if bwrap[cap_index + 1] != "ALL":
        raise GateError("stdio-only guest capability drop differs")
    if outer.count("/usr/bin/setpriv") != 1 or outer.count("/usr/bin/aa-exec") != 1:
        raise GateError("outer privilege/profile chain differs")
    for required in (
        "--clear-groups",
        "--no-new-privs",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
    ):
        if outer.count(required) != 1:
            raise GateError(f"outer privilege drop differs: {required}")
    return {
        "bwrap_argv_template_sha256": canonical_hash(bwrap),
        "outer_argv_template_sha256": canonical_hash(outer),
        "loader_placeholder_count": 1,
        "host_writable_mounts": [],
        "guest_dev_present": False,
    }


def require_fresh_identity() -> dict[str, bool]:
    paths = (ATTEMPT_ROOT, EXECUTION_RECEIPT, LIFECYCLE_RECEIPT, LIFECYCLE_FAILURE_RECEIPT, SNAPSHOT_ROOT)
    result: dict[str, bool] = {}
    for path in paths:
        try:
            os.lstat(path)
        except FileNotFoundError:
            result[str(path)] = False
            continue
        raise GateError(f"static-only probe identity is already consumed: {path}")
    return result


def verify_review_artifacts() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    expected = expected_policy()
    if policy != expected:
        differing = sorted(
            name for name in set(policy) | set(expected) if policy.get(name) != expected.get(name)
        )
        raise GateError(f"stdio probe policy differs in fields: {differing}")
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise GateError("stdio probe schema draft differs")
    if schema.get("$id") != "https://scout.local/schema/target-host-u3-stdio-apparmor-transport-probe-policy-v2.json":
        raise GateError("stdio probe schema identity differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        raise GateError("stdio probe schema is not a closed object")
    if set(schema.get("required", [])) != set(expected):
        raise GateError("stdio probe schema required-field set differs")
    validate_schema_instance(policy, schema)
    tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": sha256_regular_file(SCHEMA_PATH)},
        "profile": verify_profile(),
        "reviewed_bytes": verify_loader_and_helper(),
        "argv": verify_argv_contract(),
        "trusted_tools": tools,
        "frozen_identity_paths_exist": require_fresh_identity(),
        "status": policy["status"],
        "static_only": True,
        "helper_executed": False,
        "subprocess_started": False,
        "profile_parser_invoked": False,
        "profile_loaded": False,
        "namespace_attempted": False,
        "mount_attempted": False,
        "host_state_changed": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    arguments = parser.parse_args(argv)
    if arguments.command != "self-check":
        raise GateError("unsupported static-only command")
    try:
        review = verify_review_artifacts()
        result = {
            "status": "PASS_STATIC_STDIO_APPARMOR_PROBE_PLAN_REMAINS_NO_GO",
            "review": review,
            "next_allowed_stage": "INDEPENDENT_STATIC_REVIEW_THEN_FRESH_SINGLE_DENIAL_DISCOVERY_SUCCESSOR",
        }
        returncode = 0
    except (GateError, OSError, ValueError, UnicodeError) as exc:
        result = {
            "status": "NO_GO_STATIC_STDIO_APPARMOR_PROBE_INVALID",
            "error": str(exc),
            "static_only": True,
        }
        returncode = 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return returncode


if __name__ == "__main__":
    raise SystemExit(main())
