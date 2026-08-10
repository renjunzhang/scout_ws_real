#!/usr/bin/env python3
"""Static gate and unprivileged runner for the fresh harmless v3 probe.

``self-check`` is read-only and never invokes a subprocess. ``internal-run``
is a supervisor-only entry point: it requires the exact root-owned snapshot,
UID/GID 1000, empty groups, zero capabilities, NNP=1, and an explicit
one-shot admission token.  It never writes a host file.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
import re
import selectors
import signal
import stat
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
SNAPSHOT_ENV = "R8_LIQUID_U3_STDIO_PROBE_V3_SNAPSHOT"
ADMISSION_ENV = "R8_LIQUID_U3_STDIO_PROBE_V3_ADMISSION"
ADMISSION_FD_ENV = "R8_LIQUID_U3_STDIO_PROBE_V3_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_STDIO_PROBE_V3_ADMISSION_SHA256"
PROBE_ID = "u3_stdio_apparmor_transport_probe_v3_20260807T144659Z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")
ADMISSION_TOKEN = f"{PROBE_ID}:single-harmless-attempt"
BOOTSTRAP_PROFILE = "r8-liquid-u3-stdio-transport-bootstrap-v3-20260807t144659z"
RUNTIME_PROFILE = "r8-liquid-u3-stdio-transport-runtime-v3-20260807t144659z"
HOST_UID = 1000
HOST_GID = 1000
TMPFS_BYTES = 67_108_864

POLICY_NAME = "liquid_zrj_msi_u2404_u3_stdio_apparmor_transport_probe_policy_v3.json"
SCHEMA_NAME = "target_host_u3_stdio_apparmor_transport_probe_policy_v3.json"
PROFILE_NAME = "r8-liquid-u3-stdio-transport-probe-v3.profile"
SUPERVISOR_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v3.py"

if SNAPSHOT_ENV in os.environ:
    BASE = Path(os.environ[SNAPSHOT_ENV])
    POLICY_PATH = BASE / POLICY_NAME
    SCHEMA_PATH = BASE / SCHEMA_NAME
    PROFILE_PATH = BASE / PROFILE_NAME
    SUPERVISOR_PATH = BASE / SUPERVISOR_NAME
else:
    BASE = PACKAGE_DIR
    POLICY_PATH = BASE / "config/target_hosts" / POLICY_NAME
    SCHEMA_PATH = BASE / "schema" / SCHEMA_NAME
    PROFILE_PATH = BASE / "config/apparmor_drafts" / PROFILE_NAME
    SUPERVISOR_PATH = SCRIPT_PATH.with_name(SUPERVISOR_NAME)

STDIN_MAGIC = b"R8LQSTDIOPROBE3\x00"
SUCCESS_MAGIC = b"R8LQSTDIOPASSV3\x00"
MAX_STDIN_FRAME_BYTES = 65_536
MAX_SUCCESS_FRAME_BYTES = 16_384
MAX_CHILD_STDERR_BYTES = 4_096
OUTER_DEADLINE_SECONDS = 20

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "sleep": (Path("/usr/bin/sleep"), "a65efec857cfac0d4f43fc53affa73794ff1d1fdcb547931d4b92a98bda8e646"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

INPUT_PAYLOADS: dict[str, bytes] = {
    "probe_alpha.bin": b"R8_STDIO_PROBE_ALPHA_V3\n" * 11,
    "probe_beta.bin": bytes(range(256)) * 2 + b"BETA_V3",
    "probe_gamma.bin": hashlib.sha256(b"r8-stdio-probe-v3-gamma-seed").digest() * 31 + b"G",
}
INPUT_CONTRACT: dict[str, tuple[int, str]] = {
    name: (len(raw), hashlib.sha256(raw).hexdigest()) for name, raw in INPUT_PAYLOADS.items()
}

BASELINE_MOUNT_RULES = (
    "mount options=(rw, silent, rslave) -> /,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,',
    'mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,',
    'pivot_root oldroot=/tmp/oldroot/ /tmp/,',
    "mount options=(rw, silent, rprivate) -> /oldroot/,",
    'umount /oldroot/,',
    'pivot_root oldroot=/newroot/ /newroot/,',
    "umount /,",
    "mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,",
    "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,',
)

EXPECTED_PROFILE_LINES = (
    f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "userns create,",
    "/usr/bin/bwrap rix,",
    "/usr/bin/python3.12 rix,",
    f"/usr/bin/sleep rpx -> {RUNTIME_PROFILE},",
    "/ r,",
    "/usr/ r,",
    "/usr/lib/** mr,",
    "/usr/lib64/** mr,",
    "/lib/** mr,",
    "/lib64/** mr,",
    "/etc/ld.so.cache r,",
    "/etc/ld.so.conf r,",
    "/etc/ld.so.conf.d/** r,",
    "/work/ rw,",
    "/work/input/ rw,",
    "/work/input/probe_alpha.bin rw,",
    "/work/input/probe_beta.bin rw,",
    "/work/input/probe_gamma.bin rw,",
    "owner /proc/** r,",
    "owner /proc/*/uid_map w,",
    "owner /proc/*/gid_map w,",
    "owner /proc/*/setgroups w,",
    "/proc/filesystems r,",
    "/proc/sys/kernel/overflowuid r,",
    "/proc/sys/kernel/overflowgid r,",
    "/proc/sys/user/max_user_namespaces w,",
    *BASELINE_MOUNT_RULES,
    "/tmp/newroot/ rw,",
    "/tmp/newroot/** rw,",
    "/tmp/oldroot/ rw,",
    "/tmp/oldroot/** rw,",
    "/newroot/usr/ rw,",
    "/newroot/lib wl,",
    "/newroot/lib64 wl,",
    "/newroot/work/ rw,",
    "capability sys_admin,",
    "capability sys_ptrace,",
    "capability sys_resource,",
    "capability setpcap,",
    "capability net_admin,",
    f"ptrace (read, readby) peer={BOOTSTRAP_PROFILE},",
    f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
    "signal (receive) set=(term,kill,exists) peer=unconfined,",
    f"signal (send) set=(term,kill,exists) peer={RUNTIME_PROFILE},",
    "network unix dgram,",
    "network inet dgram,",
    "network inet6 dgram,",
    "network netlink raw,",
    "}",
    f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "/usr/bin/sleep rm,",
    "/usr/lib/** mr,",
    "/usr/lib64/** mr,",
    "/lib/** mr,",
    "/lib64/** mr,",
    "/etc/ld.so.cache r,",
    f"signal (receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
    "signal (receive) set=(term,kill,exists) peer=unconfined,",
    "}",
)
EXPECTED_PROFILE_LINES_SHA256 = hashlib.sha256(
    ("\n".join(EXPECTED_PROFILE_LINES) + "\n").encode("utf-8")
).hexdigest()


HELPER_SOURCE = rf'''import errno
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
BOOTSTRAP_PROFILE = {BOOTSTRAP_PROFILE!r}
RUNTIME_PROFILE = {RUNTIME_PROFILE!r}
SUCCESS_MAGIC = {SUCCESS_MAGIC!r}
INPUTS = {tuple((name, size, digest) for name, (size, digest) in INPUT_CONTRACT.items())!r}
ENV = {{"HOME": "/nonexistent", "PATH": "/usr/bin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"}}

class ProbeError(RuntimeError):
    pass

def read_status(path):
    fields = {{}}
    with open(path, "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    try:
        result = {{
            "uid": [int(v) for v in fields["Uid"].split()],
            "gid": [int(v) for v in fields["Gid"].split()],
            "groups": [int(v) for v in fields["Groups"].split()],
            "capabilities": {{k: int(fields[k], 16) for k in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}},
            "no_new_privs": int(fields["NoNewPrivs"]),
        }}
    except (KeyError, ValueError) as exc:
        raise ProbeError("status_parse") from exc
    if result["uid"] != [0, 0, 0, 0] or result["gid"] != [0, 0, 0, 0]:
        raise ProbeError("identity")
    if result["groups"] or any(result["capabilities"].values()) or result["no_new_privs"] != 1:
        raise ProbeError("privilege_boundary")
    return result

def read_label(path, expected):
    with open(path, "r", encoding="ascii") as source:
        observed = source.read().strip()
    if observed != expected + " (enforce)":
        raise ProbeError("label")
    return observed

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
    if total_bytes != {TMPFS_BYTES}:
        raise ProbeError("tmpfs_size")
    matches = []
    with open("/proc/self/mountinfo", "r", encoding="ascii") as source:
        for line in source:
            left, separator, right = line.rstrip("\n").partition(" - ")
            if not separator:
                raise ProbeError("mountinfo_parse")
            fields = left.split()
            filesystem_fields = right.split()
            if len(fields) < 6 or len(filesystem_fields) < 3:
                raise ProbeError("mountinfo_fields")
            if fields[4] == "/work":
                matches.append({{"filesystem_type": filesystem_fields[0], "mount_options": fields[5].split(",")}})
    if len(matches) != 1 or matches[0]["filesystem_type"] != "tmpfs":
        raise ProbeError("tmpfs_identity")
    if not {{"rw", "nosuid", "nodev"}}.issubset(set(matches[0]["mount_options"])):
        raise ProbeError("tmpfs_mount_options")
    return {{"filesystem_type": "tmpfs", "total_bytes": total_bytes, "mount_options": matches[0]["mount_options"]}}

def write_inputs():
    os.mkdir("/work/input", 0o700)
    observed = {{}}
    for name, expected_size, expected_hash in INPUTS:
        header = READ_EXACT(40)
        announced_size = struct.unpack(">Q", header[:8])[0]
        announced_hash = header[8:].hex()
        if announced_size != expected_size or announced_hash != expected_hash:
            raise ProbeError("input_header")
        descriptor = os.open("/work/input/" + name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o400)
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
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
                    or metadata.st_size != expected_size or metadata.st_uid != 0
                    or metadata.st_gid != 0 or stat.S_IMODE(metadata.st_mode) != 0o400):
                raise ProbeError("input_inode")
        finally:
            os.close(descriptor)
        if digest.hexdigest() != expected_hash:
            raise ProbeError("input_digest")
        observed[name] = {{"size_bytes": expected_size, "sha256": expected_hash}}
    if STREAM.read(1) != b"":
        raise ProbeError("trailing_input")
    STREAM.close()
    try:
        os.close(0)
    except OSError as exc:
        if exc.errno != errno.EBADF:
            raise
    read_end, write_end = os.pipe2(os.O_CLOEXEC)
    os.close(write_end)
    if read_end != 0:
        os.dup2(read_end, 0, inheritable=True)
        os.close(read_end)
    else:
        os.set_inheritable(0, True)
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
        if stdout or stderr:
            raise ProbeError("runtime_output")
        raise ProbeError("runtime_term_timeout")
    return stdout, stderr

def run_runtime():
    process = subprocess.Popen(["/usr/bin/sleep", "30"], stdin=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/work", env=ENV, close_fds=True, start_new_session=True)
    if process.stdin is not None or process.stdout is None or process.stderr is None:
        raise ProbeError("runtime_pipe")
    label_path = f"/proc/{{process.pid}}/attr/current"
    status_path = f"/proc/{{process.pid}}/status"
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
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.01)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        raise ProbeError("runtime_residue")
    return {{"label": label, "identity": runtime_identity, "returncode": process.returncode, "stdin": "guest_internal_eof_pipe_fd0", "stdout_stderr": "internal_empty_pipes"}}

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
    bootstrap_label = read_label("/proc/self/attr/current", BOOTSTRAP_PROFILE)
    bootstrap_identity = read_status("/proc/self/status")
    work_tmpfs = verify_work_tmpfs()
    observed = write_inputs()
    fds_after_stdin = live_fds({{0, 1, 2}})
    runtime = run_runtime()
    fds_before_success = live_fds({{0, 1, 2}})
    emit_success({{"status": "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V3", "bootstrap_profile": BOOTSTRAP_PROFILE, "bootstrap_label": bootstrap_label, "bootstrap_identity": bootstrap_identity, "work_tmpfs": work_tmpfs, "inputs": observed, "host_stdin_consumed_and_fd0_replaced_with_eof_pipe": True, "fds_after_stdin": fds_after_stdin, "runtime": runtime, "fds_before_success": fds_before_success, "host_writable_mounts": []}})
    return 0

try:
    raise SystemExit(main())
except (ProbeError, OSError, ValueError, UnicodeError, subprocess.SubprocessError):
    try:
        os.write(2, b"R8_STDIO_PROBE_V3_NO_GO\n")
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
exec(compile(p,"<r8-stdio-probe-helper-v3>","exec",dont_inherit=True,optimize=2),g,g)
'''
LOADER_BYTES = LOADER_SOURCE.encode("ascii")
LOADER_SHA256 = hashlib.sha256(LOADER_BYTES).hexdigest()
LOADER_SIZE_BYTES = len(LOADER_BYTES)

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed static, identity, framing, or conduit error."""


def read_regular_bytes(path: Path, *, limit: int = 2 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise GateError(f"unsafe regular file: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise GateError(f"regular file exceeds ceiling: {path}")
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def require_tool(name: str) -> dict[str, Any]:
    path, expected = TRUSTED_TOOLS[name]
    raw = read_regular_bytes(path, limit=512 * 1024 * 1024)
    metadata = os.stat(path, follow_symlinks=False)
    if metadata.st_uid != 0 or metadata.st_gid != 0 or not metadata.st_mode & stat.S_IXUSR:
        raise GateError(f"trusted tool metadata differs: {path}")
    if sha256_bytes(raw) != expected:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": expected, "size_bytes": len(raw)}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def build_stdin_frame() -> bytes:
    parts = [STDIN_MAGIC, struct.pack(">I", HELPER_SIZE_BYTES), bytes.fromhex(HELPER_SHA256), HELPER_BYTES]
    for name, raw in INPUT_PAYLOADS.items():
        size, digest = INPUT_CONTRACT[name]
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise GateError(f"synthetic payload identity differs: {name}")
        parts.extend((struct.pack(">Q", size), bytes.fromhex(digest), raw))
    frame = b"".join(parts)
    if len(frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("stdin frame exceeds its fixed ceiling")
    return frame


def parse_stdin_frame(frame: bytes) -> dict[str, Any]:
    if len(frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("stdin frame exceeds ceiling")
    view = memoryview(frame)
    position = 0

    def take(size: int) -> bytes:
        nonlocal position
        if size < 0 or position + size > len(view):
            raise GateError("stdin frame truncation")
        result = bytes(view[position:position + size])
        position += size
        return result

    if take(16) != STDIN_MAGIC:
        raise GateError("stdin frame magic differs")
    helper_size = struct.unpack(">I", take(4))[0]
    helper_hash = take(32).hex()
    helper = take(helper_size)
    if helper_size != HELPER_SIZE_BYTES or helper_hash != HELPER_SHA256 or sha256_bytes(helper) != helper_hash:
        raise GateError("stdin helper identity differs")
    inputs: dict[str, dict[str, Any]] = {}
    for name, (expected_size, expected_hash) in INPUT_CONTRACT.items():
        size = struct.unpack(">Q", take(8))[0]
        digest = take(32).hex()
        raw = take(size)
        if size != expected_size or digest != expected_hash or sha256_bytes(raw) != digest:
            raise GateError(f"stdin input identity differs: {name}")
        inputs[name] = {"size_bytes": size, "sha256": digest}
    if position != len(view):
        raise GateError("stdin frame has trailing bytes")
    return {"helper": {"size_bytes": helper_size, "sha256": helper_hash}, "inputs": inputs, "size_bytes": len(frame), "sha256": sha256_bytes(frame)}


def parse_success_frame(frame: bytes) -> dict[str, Any]:
    if not 52 <= len(frame) <= MAX_SUCCESS_FRAME_BYTES or not frame.startswith(SUCCESS_MAGIC):
        raise GateError("success frame size or magic differs")
    size = struct.unpack(">I", frame[16:20])[0]
    if size < 2 or 20 + size + 32 != len(frame):
        raise GateError("success frame declared length differs")
    payload = frame[20:20 + size]
    if hashlib.sha256(payload).digest() != frame[-32:]:
        raise GateError("success frame digest differs")
    try:
        value = json.loads(payload.decode("ascii"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError("success frame JSON differs") from exc
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if canonical != payload or not isinstance(value, dict):
        raise GateError("success frame is not canonical")
    expected_keys = {
        "status", "bootstrap_profile", "bootstrap_label", "bootstrap_identity",
        "work_tmpfs", "inputs", "host_stdin_consumed_and_fd0_replaced_with_eof_pipe",
        "fds_after_stdin", "runtime", "fds_before_success", "host_writable_mounts",
    }
    if set(value) != expected_keys:
        raise GateError("success frame key set differs")
    if value["status"] != "PASS_HARMLESS_STDIO_APPARMOR_TRANSPORT_PROBE_V3":
        raise GateError("success status differs")
    if value["bootstrap_profile"] != BOOTSTRAP_PROFILE or value["bootstrap_label"] != BOOTSTRAP_PROFILE + " (enforce)":
        raise GateError("bootstrap label evidence differs")
    if value["host_writable_mounts"] != [] or value["host_stdin_consumed_and_fd0_replaced_with_eof_pipe"] is not True:
        raise GateError("success isolation evidence differs")
    if (type(value["fds_after_stdin"]) is not list or type(value["fds_before_success"]) is not list
            or any(type(descriptor) is not int for descriptor in value["fds_after_stdin"] + value["fds_before_success"])
            or value["fds_after_stdin"] != [0, 1, 2] or value["fds_before_success"] != [0, 1, 2]):
        raise GateError("success descriptor evidence differs")

    def require_guest_identity(observed: Any, context: str) -> None:
        if not isinstance(observed, dict) or set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
            raise GateError(f"{context} identity key set differs")
        uid, gid, groups = observed["uid"], observed["gid"], observed["groups"]
        if (type(uid) is not list or type(gid) is not list or type(groups) is not list
                or any(type(item) is not int for item in uid + gid + groups)
                or uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups != []):
            raise GateError(f"{context} identity differs")
        capabilities = observed["capabilities"]
        cap_keys = {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        if (type(capabilities) is not dict or set(capabilities) != cap_keys
                or any(type(capabilities[key]) is not int or capabilities[key] != 0 for key in cap_keys)):
            raise GateError(f"{context} capability evidence differs")
        if type(observed["no_new_privs"]) is not int or observed["no_new_privs"] != 1:
            raise GateError(f"{context} NNP evidence differs")

    require_guest_identity(value["bootstrap_identity"], "bootstrap")
    expected_inputs = {name: {"size_bytes": size, "sha256": digest} for name, (size, digest) in INPUT_CONTRACT.items()}
    if value["inputs"] != expected_inputs:
        raise GateError("success synthetic input evidence differs")
    work_tmpfs = value["work_tmpfs"]
    if not isinstance(work_tmpfs, dict) or set(work_tmpfs) != {"filesystem_type", "total_bytes", "mount_options"}:
        raise GateError("success tmpfs key set differs")
    options = work_tmpfs["mount_options"]
    if (work_tmpfs["filesystem_type"] != "tmpfs" or work_tmpfs["total_bytes"] != TMPFS_BYTES
            or isinstance(work_tmpfs["total_bytes"], bool) or not isinstance(options, list)
            or any(not isinstance(option, str) for option in options) or len(options) != len(set(options))
            or not {"rw", "nosuid", "nodev"}.issubset(set(options))):
        raise GateError("success tmpfs evidence differs")
    runtime = value["runtime"]
    if not isinstance(runtime, dict) or set(runtime) != {"label", "identity", "returncode", "stdin", "stdout_stderr"}:
        raise GateError("success runtime key set differs")
    require_guest_identity(runtime["identity"], "runtime")
    if (runtime["label"] != RUNTIME_PROFILE + " (enforce)" or runtime["returncode"] != -int(signal.SIGTERM)
            or isinstance(runtime["returncode"], bool) or runtime["stdin"] != "guest_internal_eof_pipe_fd0"
            or runtime["stdout_stderr"] != "internal_empty_pipes"):
        raise GateError("success runtime evidence differs")
    return value


def bwrap_argv() -> list[str]:
    return [
        "/usr/bin/timeout", "--foreground", "--signal=TERM", "--kill-after=2s", "15s",
        "/usr/bin/aa-exec", "-p", BOOTSTRAP_PROFILE, "--",
        "/usr/bin/bwrap", "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--uid", "0", "--gid", "0", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-stdio-probe-v3", "--clearenv",
        "--setenv", "HOME", "/nonexistent", "--setenv", "PATH", "/usr/bin",
        "--setenv", "LC_ALL", "C.UTF-8", "--setenv", "LANG", "C.UTF-8", "--setenv", "TZ", "UTC",
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--size", str(TMPFS_BYTES), "--tmpfs", "/work", "--proc", "/proc", "--chdir", "/work", "--",
        "/usr/bin/python3.12", "-I", "-B", "-c", LOADER_SOURCE,
    ]


def verify_argv_contract(argv: list[str]) -> None:
    if argv != bwrap_argv():
        raise GateError("fixed argv differs")
    for forbidden in ("--bind", "--bind-fd", "--file", "--dev", "--dev-bind", "--share-net"):
        if forbidden in argv:
            raise GateError(f"forbidden bwrap option present: {forbidden}")
    if argv.count("--ro-bind") != 1 or argv[argv.index("--ro-bind") + 1:argv.index("--ro-bind") + 3] != ["/usr", "/usr"]:
        raise GateError("read-only host bind differs")
    index = argv.index("--size")
    if argv[index:index + 4] != ["--size", str(TMPFS_BYTES), "--tmpfs", "/work"]:
        raise GateError("bounded /work tmpfs differs")
    if argv[argv.index("--proc"):argv.index("--proc") + 2] != ["--proc", "/proc"]:
        raise GateError("single proc discovery target differs")


def _effective_lines(text: str) -> tuple[str, ...]:
    return tuple(line.strip() for line in (raw.split("#", 1)[0] for raw in text.splitlines()) if line.strip())


def verify_profile() -> dict[str, Any]:
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    text = raw.decode("utf-8")
    lines = _effective_lines(text)
    if lines != EXPECTED_PROFILE_LINES:
        raise GateError("effective AppArmor rule set differs from the exact reviewed tuple")
    lines_sha256 = hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()
    if lines_sha256 != EXPECTED_PROFILE_LINES_SHA256:
        raise GateError("effective AppArmor rule-set digest differs")
    if lines.count(f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted) {{") != 1:
        raise GateError("bootstrap profile identity differs")
    if lines.count(f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{") != 1:
        raise GateError("runtime profile identity differs")
    transition = f"/usr/bin/sleep rpx -> {RUNTIME_PROFILE},"
    if lines.count(transition) != 1 or any(" rPx" in line for line in lines):
        raise GateError("lowercase fail-closed rpx transition differs")
    mount_lines = tuple(line for line in lines if line.startswith(("mount ", "remount ", "pivot_root ", "umount ")))
    if mount_lines != BASELINE_MOUNT_RULES:
        raise GateError("provenance-pinned mount baseline differs")
    forbidden = ("/newroot/proc", "fstype=proc", "/dev/", "/dev ", "GenCase", "DualSPHysics", "/home/zrj/scout_ws", "/opt/ros", "flags=(unconfined)", " mount,")
    effective = "\n".join(lines)
    if any(token in effective for token in forbidden):
        raise GateError("profile contains forbidden or guessed authority")
    runtime = effective.split(f"profile {RUNTIME_PROFILE}", 1)[1]
    for token in ("userns ", "mount ", "capability ", "network ", "/proc", "/dev", "/work", "/usr/bin/python"):
        if token in runtime:
            raise GateError(f"runtime profile contains forbidden authority: {token}")
    return {
        "path": str(PROFILE_PATH),
        "sha256": sha256_bytes(raw),
        "effective_lines_sha256": lines_sha256,
        "effective_line_count": len(lines),
        "mount_rules": list(mount_lines),
        "production_authorized": False,
    }


def _assert_schema_objects_closed(node: Any, path: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise GateError(f"schema object is not closed: {path}")
        for key, value in node.items():
            _assert_schema_objects_closed(value, f"{path}.{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            _assert_schema_objects_closed(value, f"{path}[{index}]")


def _json_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return set(left) == set(right) and all(_json_equal(left[key], right[key]) for key in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(_json_equal(a, b) for a, b in zip(left, right))
    return left == right


def _validate_schema_instance(instance: Any, node: Mapping[str, Any], root: Mapping[str, Any], path: str = "$") -> None:
    reference = node.get("$ref")
    if reference is not None:
        if not isinstance(reference, str) or not reference.startswith("#/"):
            raise GateError(f"unsupported schema reference: {path}")
        target: Any = root
        for part in reference[2:].split("/"):
            target = target[part.replace("~1", "/").replace("~0", "~")]
        if not isinstance(target, dict):
            raise GateError(f"schema reference target differs: {path}")
        _validate_schema_instance(instance, target, root, path)
        return
    if "const" in node and not _json_equal(instance, node["const"]):
        raise GateError(f"policy const differs: {path}")
    if "enum" in node and not any(_json_equal(instance, option) for option in node["enum"]):
        raise GateError(f"policy enum differs: {path}")
    expected_type = node.get("type")
    type_ok = {
        "object": isinstance(instance, dict),
        "array": isinstance(instance, list),
        "string": isinstance(instance, str),
        "integer": isinstance(instance, int) and not isinstance(instance, bool),
        "boolean": isinstance(instance, bool),
    }.get(expected_type, True)
    if not type_ok:
        raise GateError(f"policy type differs: {path}")
    if expected_type == "object":
        properties = node.get("properties", {})
        required = node.get("required", [])
        if not isinstance(properties, dict) or not isinstance(required, list) or any(key not in instance for key in required):
            raise GateError(f"policy required object key differs: {path}")
        if node.get("additionalProperties") is False and any(key not in properties for key in instance):
            raise GateError(f"policy object has an additional key: {path}")
        for key, value in instance.items():
            child = properties.get(key)
            if isinstance(child, dict):
                _validate_schema_instance(value, child, root, f"{path}.{key}")
    elif expected_type == "array":
        if len(instance) < node.get("minItems", 0) or len(instance) > node.get("maxItems", 1 << 60):
            raise GateError(f"policy array length differs: {path}")
        child = node.get("items")
        if isinstance(child, dict):
            for index, value in enumerate(instance):
                _validate_schema_instance(value, child, root, f"{path}[{index}]")
    elif expected_type == "string":
        if len(instance) < node.get("minLength", 0) or len(instance) > node.get("maxLength", 1 << 60):
            raise GateError(f"policy string length differs: {path}")
        pattern = node.get("pattern")
        if pattern is not None and re.search(pattern, instance) is None:
            raise GateError(f"policy string pattern differs: {path}")
    elif expected_type == "integer":
        if instance < node.get("minimum", -(1 << 63)) or instance > node.get("maximum", 1 << 63):
            raise GateError(f"policy integer range differs: {path}")


def artifact_paths() -> dict[str, Path]:
    return {"gate": SCRIPT_PATH, "supervisor": SUPERVISOR_PATH, "profile": PROFILE_PATH, "schema": SCHEMA_PATH}


def verify_static() -> dict[str, Any]:
    policy = read_json(POLICY_PATH)
    schema = read_json(SCHEMA_PATH)
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("closed top-level policy/schema contract differs")
    expected = {
        "schema_version": "smpcc-r8-liquid-target-u3-stdio-apparmor-transport-probe-policy-v3",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_STDIO_APPARMOR_TRANSPORT_PROBE_V3",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "status": "STATIC_READY_V3_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT",
        "next_allowed_stage": "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT",
    }
    for key, value in expected.items():
        if policy.get(key) != value:
            raise GateError(f"policy identity differs: {key}")
    authorization = policy.get("authorization", {})
    if authorization.get("static_task_execution_performed") is not False or authorization.get("attempts_per_identity") != 1:
        raise GateError("static-only one-shot authorization differs")
    if authorization.get("admission_token") != ADMISSION_TOKEN:
        raise GateError("explicit admission token differs")
    frozen = policy.get("frozen_identity", {})
    if frozen.get("probe_id") != PROBE_ID or frozen.get("snapshot_root") != str(SNAPSHOT_ROOT):
        raise GateError("frozen probe identity differs")
    if frozen.get("bootstrap_profile") != BOOTSTRAP_PROFILE or frozen.get("runtime_profile") != RUNTIME_PROFILE:
        raise GateError("frozen labels differ")
    if policy.get("trusted_system_tools", {}).get("unprivileged_runner") != trusted_tool_policy():
        raise GateError("unprivileged trusted-tool contract differs")
    reviewed = policy.get("reviewed_bytes", {})
    expected_inputs = [{"name": name, "size_bytes": size, "sha256": digest} for name, (size, digest) in INPUT_CONTRACT.items()]
    if reviewed.get("helper") != {"size_bytes": HELPER_SIZE_BYTES, "sha256": HELPER_SHA256}:
        raise GateError("helper bytes policy differs")
    if reviewed.get("loader") != {"size_bytes": LOADER_SIZE_BYTES, "sha256": LOADER_SHA256}:
        raise GateError("loader bytes policy differs")
    if reviewed.get("inputs") != expected_inputs:
        raise GateError("synthetic input policy differs")
    frame = build_stdin_frame()
    parsed_frame = parse_stdin_frame(frame)
    if reviewed.get("stdin_frame") != {"size_bytes": len(frame), "sha256": sha256_bytes(frame), "maximum_size_bytes": MAX_STDIN_FRAME_BYTES}:
        raise GateError("stdin frame policy differs")
    verify_argv_contract(bwrap_argv())
    if policy.get("fixed_commands", {}).get("bwrap_argv_sha256") != sha256_bytes(json.dumps(bwrap_argv(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")):
        raise GateError("fixed bwrap argv hash differs")
    profile = verify_profile()
    if policy.get("mount_discovery", {}).get("effective_mount_rules") != list(BASELINE_MOUNT_RULES):
        raise GateError("policy mount baseline differs")
    if policy.get("mount_discovery", {}).get("effective_profile_lines_sha256") != EXPECTED_PROFILE_LINES_SHA256:
        raise GateError("policy effective profile semantics digest differs")
    if policy.get("mount_discovery", {}).get("proc_rules") != []:
        raise GateError("policy guesses proc rules")
    artifacts = policy.get("trusted_artifacts", {})
    if set(artifacts) != set(artifact_paths()):
        raise GateError("trusted artifact set differs")
    observed_artifacts: dict[str, dict[str, Any]] = {}
    for name, path in artifact_paths().items():
        raw = read_regular_bytes(path)
        entry = artifacts[name]
        if entry.get("sha256") != sha256_bytes(raw) or entry.get("size_bytes") != len(raw):
            raise GateError(f"trusted artifact identity differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}
    return {"policy_sha256": sha256_bytes(read_regular_bytes(POLICY_PATH)), "artifacts": observed_artifacts, "profile": profile, "stdin_frame": parsed_frame, "helper_sha256": HELPER_SHA256, "loader_sha256": LOADER_SHA256, "execution_performed": False}


def read_identity_status(path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    try:
        return {
            "uid": [int(value) for value in fields["Uid"].split()],
            "gid": [int(value) for value in fields["Gid"].split()],
            "groups": [int(value) for value in fields["Groups"].split()],
            "capabilities": {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
            "no_new_privs": int(fields["NoNewPrivs"]),
        }
    except (KeyError, ValueError) as exc:
        raise GateError("cannot parse host child identity") from exc


def verify_host_identity(status_value: Mapping[str, Any] | None = None) -> dict[str, Any]:
    observed = dict(read_identity_status() if status_value is None else status_value)
    if set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
        raise GateError("host gate identity key set differs")
    uid, gid, groups = observed.get("uid"), observed.get("gid"), observed.get("groups")
    if (type(uid) is not list or type(gid) is not list
            or any(type(item) is not int for item in uid + gid)
            or uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4):
        raise GateError("host gate UID/GID fields differ")
    if type(groups) is not list or any(type(item) is not int for item in groups) or groups != []:
        raise GateError("host gate supplementary groups differ")
    caps = observed.get("capabilities")
    if (type(caps) is not dict or set(caps) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
            or any(type(value) is not int or value != 0 for value in caps.values())):
        raise GateError("host gate capability sets differ")
    if type(observed.get("no_new_privs")) is not int or observed.get("no_new_privs") != 1:
        raise GateError("host gate NNP must be one before aa-exec")
    return observed


def verify_snapshot_runtime() -> dict[str, Any]:
    if SNAPSHOT_ENV not in os.environ or Path(os.environ[SNAPSHOT_ENV]) != SNAPSHOT_ROOT:
        raise GateError("internal-run requires the exact snapshot environment")
    if SCRIPT_PATH.parent != SNAPSHOT_ROOT:
        raise GateError("internal-run gate is not executing from the snapshot")
    root_metadata = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root_metadata.st_mode) or root_metadata.st_uid != 0 or root_metadata.st_gid != 0 or stat.S_IMODE(root_metadata.st_mode) != 0o555:
        raise GateError("snapshot directory ownership or mode differs")
    for path in (SCRIPT_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH, SUPERVISOR_PATH):
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444:
            raise GateError(f"snapshot artifact metadata differs: {path.name}")
    return {"root": str(SNAPSHOT_ROOT), "mode": "0555", "owner": [0, 0]}


def consume_root_admission_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[ADMISSION_FD_ENV])
        expected_sha256 = os.environ[ADMISSION_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise GateError("root admission FD capability metadata is absent") from exc
    if descriptor < 3 or len(expected_sha256) != 64 or any(character not in "0123456789abcdef" for character in expected_sha256):
        raise GateError("root admission FD capability metadata differs")
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or stat.S_IMODE(metadata.st_mode) != 0o600 or not os.get_inheritable(descriptor)):
            raise GateError("root admission FD capability inode differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise GateError("root admission FD capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError("root admission FD capability has trailing bytes")
    finally:
        os.close(descriptor)
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise GateError("root admission FD capability digest differs")
    os.environ.pop(ADMISSION_FD_ENV, None)
    os.environ.pop(ADMISSION_SHA256_ENV, None)
    return {
        "transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ",
        "size_bytes": len(payload),
        "sha256": expected_sha256,
        "pipe_uid": metadata.st_uid,
        "pipe_gid": metadata.st_gid,
        "pipe_mode": "0600",
    }


def _proc_starttime(pid: int) -> int | None:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except FileNotFoundError:
        return None
    end = raw.rfind(")")
    fields = raw[end + 2:].split() if end >= 0 else []
    if len(fields) < 20:
        return None
    try:
        return int(fields[19])
    except ValueError:
        return None


def _proc_labeled_members(proc_root: Path = Path("/proc")) -> list[dict[str, Any]]:
    """Enumerate all threads in the fixed UID1000 gate tree under fresh labels.

    This is deliberately local unprivileged cleanup evidence, not a claim of
    all-UID label absence.  The root supervisor performs the authoritative
    all-task scan before and after profile lifecycle operations.
    """

    members: list[dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal():
            continue
        pid = int(entry.name)
        try:
            status_raw = (entry / "status").read_text(encoding="ascii")
            uid_line = next((line for line in status_raw.splitlines() if line.startswith("Uid:")), "")
            uid_fields = [int(value) for value in uid_line.split()[1:]]
            if len(uid_fields) != 4:
                raise GateError(f"cannot parse task UID authority for pid {pid}")
            if uid_fields != [HOST_UID] * 4:
                continue
            leader_stat_raw = (entry / "stat").read_text(encoding="ascii")
            task_entries = tuple((entry / "task").iterdir())
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ESRCH):
                continue
            raise GateError(f"cannot read labeled task authority for pid {pid}: {exc}") from exc
        end = leader_stat_raw.rfind(")")
        leader_fields = leader_stat_raw[end + 2:].split() if end >= 0 else []
        if len(leader_fields) < 20:
            raise GateError(f"cannot parse labeled task stat for pid {pid}")
        tgid_starttime = int(leader_fields[19])
        if tgid_starttime < 1:
            raise GateError(f"non-positive tgid starttime for pid {pid}")
        for task_entry in task_entries:
            if not task_entry.name.isdecimal():
                continue
            tid = int(task_entry.name)
            try:
                attr_raw = (task_entry / "attr/current").read_text(encoding="utf-8")
                task_stat_raw = (task_entry / "stat").read_text(encoding="ascii")
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                if exc.errno in (errno.ENOENT, errno.ESRCH):
                    continue
                raise GateError(f"cannot read labeled thread authority for tgid {pid} tid {tid}: {exc}") from exc
            task_end = task_stat_raw.rfind(")")
            task_fields = task_stat_raw[task_end + 2:].split() if task_end >= 0 else []
            if len(task_fields) < 20:
                raise GateError(f"cannot parse labeled thread stat for tgid {pid} tid {tid}")
            tid_starttime = int(task_fields[19])
            if tid_starttime < 1:
                raise GateError(f"non-positive tid starttime for tgid {pid} tid {tid}")
            label = attr_raw.strip().split(" (", 1)[0]
            if label in (BOOTSTRAP_PROFILE, RUNTIME_PROFILE):
                members.append({
                    "tgid": pid,
                    "tid": tid,
                    "pgrp": int(task_fields[2]),
                    "tgid_starttime": tgid_starttime,
                    "tid_starttime": tid_starttime,
                    "label": label,
                })
    return sorted(members, key=lambda item: (item["tgid"], item["tid"]))


def _signal_open_pidfd(pidfd: int, sig: signal.Signals) -> bool:
    try:
        signal.pidfd_send_signal(pidfd, sig)
        return True
    except ProcessLookupError:
        return False


def _signal_pid(pid: int, sig: signal.Signals, starttime: int) -> bool:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise GateError("pidfd signaling is unavailable")
    if not isinstance(starttime, int) or isinstance(starttime, bool) or starttime < 1:
        raise GateError("refusing to signal a task without a frozen positive starttime")
    try:
        pidfd = os.pidfd_open(pid, 0)
    except ProcessLookupError:
        return False
    try:
        if _proc_starttime(pid) != starttime:
            return False
        return _signal_open_pidfd(pidfd, sig)
    finally:
        os.close(pidfd)


def _terminate_probe_tree(process: subprocess.Popen[bytes], leader_pidfd: int) -> dict[str, Any]:
    """Bounded pidfd cleanup independent of leader lifetime and process group."""

    evidence: dict[str, Any] = {"term": [], "kill": [], "stable_zero_scans": []}
    if process.poll() is None and _signal_open_pidfd(leader_pidfd, signal.SIGTERM):
        evidence["term"].append({"pid": process.pid, "role": "leader"})
    signaled_tgids: set[int] = set()
    for item in _proc_labeled_members():
        if item["tgid"] == process.pid or item["tgid"] in signaled_tgids:
            continue
        if _signal_pid(item["tgid"], signal.SIGTERM, item["tgid_starttime"]):
            signaled_tgids.add(item["tgid"])
            evidence["term"].append(item)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if process.poll() is not None and not _proc_labeled_members():
            break
        time.sleep(0.05)
    if process.poll() is None and _signal_open_pidfd(leader_pidfd, signal.SIGKILL):
        evidence["kill"].append({"pid": process.pid, "role": "leader"})
    signaled_tgids = set()
    for item in _proc_labeled_members():
        if item["tgid"] == process.pid or item["tgid"] in signaled_tgids:
            continue
        if _signal_pid(item["tgid"], signal.SIGKILL, item["tgid_starttime"]):
            signaled_tgids.add(item["tgid"])
            evidence["kill"].append(item)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and _proc_labeled_members():
        time.sleep(0.05)
    for index in range(3):
        current = _proc_labeled_members()
        evidence["stable_zero_scans"].append(current)
        if current:
            raise GateError(f"labeled task residue remains after bounded cleanup: {current}")
        if index < 2:
            time.sleep(0.05)
    return evidence


def _bounded_extend(target: bytearray, block: bytes, limit: int) -> bool:
    remaining = max(0, limit - len(target))
    target.extend(block[:remaining])
    return len(block) > remaining


def run_bounded_guest(argv: list[str], input_frame: bytes) -> tuple[int, bytes, bytes, int]:
    if len(input_frame) > MAX_STDIN_FRAME_BYTES:
        raise GateError("guest stdin exceeds its hard ceiling")
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise GateError("pidfd signaling is unavailable")
    process = subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
        close_fds=True,
        start_new_session=True,
    )
    try:
        leader_pidfd = os.pidfd_open(process.pid, 0)
    except BaseException:
        # The unreaped direct child PID cannot be reused here, so Popen.kill is
        # the only safe fallback when a birth-time pidfd cannot be retained.
        process.kill()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        raise
    selector: selectors.BaseSelector | None = None
    stdout = bytearray()
    stderr = bytearray()
    position = 0
    failure: str | None = None
    cleanup_failure: str | None = None
    deadline = time.monotonic() + OUTER_DEADLINE_SECONDS
    try:
        if process.stdin is None or process.stdout is None or process.stderr is None:
            raise GateError("guest conduit pipes are unavailable")
        descriptors = {"stdin": process.stdin.fileno(), "stdout": process.stdout.fileno(), "stderr": process.stderr.fileno()}
        for descriptor in descriptors.values():
            os.set_blocking(descriptor, False)
        selector = selectors.DefaultSelector()
        selector.register(descriptors["stdin"], selectors.EVENT_WRITE, "stdin")
        selector.register(descriptors["stdout"], selectors.EVENT_READ, "stdout")
        selector.register(descriptors["stderr"], selectors.EVENT_READ, "stderr")
        while selector.get_map():
            if time.monotonic() >= deadline:
                failure = failure or "guest conduit exceeded its outer deadline"
                break
            events = selector.select(timeout=0.1)
            for key, _mask in events:
                descriptor = key.fd
                channel = key.data
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, input_frame[position:position + 4096])
                    except BrokenPipeError:
                        count = 0
                    except BlockingIOError:
                        continue
                    if count:
                        position += count
                    if not count or position == len(input_frame):
                        selector.unregister(descriptor)
                        process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 4096)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    (process.stdout if channel == "stdout" else process.stderr).close()
                    continue
                target = stdout if channel == "stdout" else stderr
                limit = MAX_SUCCESS_FRAME_BYTES if channel == "stdout" else MAX_CHILD_STDERR_BYTES
                if _bounded_extend(target, block, limit):
                    failure = f"guest {channel} exceeded its hard byte ceiling"
                    break
            if failure is not None:
                break
    finally:
        if selector is not None:
            selector.close()
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                stream.close()
            except (OSError, AttributeError):
                pass
        try:
            members = _proc_labeled_members()
            if process.poll() is None or members:
                _terminate_probe_tree(process, leader_pidfd)
        except (GateError, OSError) as exc:
            cleanup_failure = f"bounded labeled-task cleanup failed: {exc}"
        finally:
            if process.poll() is None:
                try:
                    _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                except OSError as exc:
                    cleanup_failure = cleanup_failure or f"final leader pidfd kill failed: {exc}"
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                cleanup_failure = cleanup_failure or "guest leader did not reap after final pidfd kill"
            os.close(leader_pidfd)
        if cleanup_failure is not None:
            raise GateError(cleanup_failure)
    returncode = process.returncode
    if returncode is None:
        raise GateError("guest leader return code is unavailable after reap")
    if failure is not None:
        raise GateError(failure)
    return returncode, bytes(stdout), bytes(stderr), position


def internal_run() -> tuple[int, bytes, bytes]:
    if os.environ.get(ADMISSION_ENV) != ADMISSION_TOKEN:
        raise GateError("explicit one-shot admission token is absent")
    verify_snapshot_runtime()
    verify_host_identity()
    consume_root_admission_capability()
    review = verify_static()
    for name in TRUSTED_TOOLS:
        require_tool(name)
    frame = build_stdin_frame()
    if parse_stdin_frame(frame)["sha256"] != review["stdin_frame"]["sha256"]:
        raise GateError("runtime stdin frame differs from static review")
    returncode, stdout, stderr, consumed = run_bounded_guest(bwrap_argv(), frame)
    if returncode == 0:
        if consumed != len(frame):
            raise GateError("successful guest did not consume the complete fixed stdin frame")
        if stderr:
            raise GateError("successful guest emitted stderr")
        parse_success_frame(stdout)
        return returncode, stdout, b""
    if stdout:
        raise GateError("failed guest emitted forbidden stdout")
    return returncode, b"", stderr


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "internal-run"))
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = verify_static()
            print(json.dumps({"status": "PASS_V3_STATIC_DRAFT_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        returncode, stdout, stderr = internal_run()
        if stdout:
            sys.stdout.buffer.write(stdout)
            sys.stdout.buffer.flush()
        if stderr:
            sys.stderr.buffer.write(stderr)
            sys.stderr.buffer.flush()
        return 0 if returncode == 0 else min(125, max(1, returncode))
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        stream = sys.stderr if arguments.command == "internal-run" else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=stream)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
