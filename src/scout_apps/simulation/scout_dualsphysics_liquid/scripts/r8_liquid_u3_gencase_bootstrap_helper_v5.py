#!/usr/bin/env python3
"""Guest-only bootstrap helper for the admitted U3 C1 GenCase v5 runtime.

This file is never a host launcher.  A fixed isolated-Python loader reads these
reviewed bytes from a bounded host-to-guest stdin frame, checks the frozen byte
length and SHA-256, and compiles/executes them directly from memory.  This helper
then consumes three fixed-order, fixed-length input segments from the same fd,
requires EOF, replaces stdin with a guest-internal EOF pipe, verifies all three
final guest copies, and executes exactly one fixed GenCase argv.  Every file
lives on the bounded guest-only /work tmpfs.
"""

from __future__ import annotations

import hashlib
import json
import os
import resource
import select
import signal
import stat
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from typing import Any


RUNTIME_ROOT = "/work/runtime"
OUTPUT_ROOT = "/work/output"
SHA256SUM = "/usr/bin/sha256sum"
GENCASE = f"{RUNTIME_ROOT}/GenCase_linux64"
BOOTSTRAP_PROFILE = "r8-liquid-u3-gencase-bootstrap-v5-20260808t070530z"

INPUTS: dict[str, tuple[str, int, int]] = {
    "GenCase_linux64": (
        "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226",
        5_809_384,
        0o500,
    ),
    "DsphConfig.xml": (
        "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
        293,
        0o400,
    ),
    "C1_static_Def.xml": (
        "d738d303300b3f339ac37ca3a604fbe8dff9e034d8c8ad7aacb2654434525819",
        5_714,
        0o400,
    ),
}
EXPECTED_OUTPUTS: dict[str, tuple[int, int]] = {
    "C1_static.bi4": (1, 16_777_216),
    "C1_static.xml": (1, 524_288),
    "C1_static.out": (1, 1_048_576),
}
GENCASE_ARGV = [
    GENCASE,
    f"{RUNTIME_ROOT}/C1_static_Def",
    f"{OUTPUT_ROOT}/C1_static",
    "-dp:0.002",
    "-threads:1",
    "-save:-all,+bi",
    "-createdirs:1",
]
ENVIRONMENT = {
    "HOME": "/nonexistent",
    "PATH": "/usr/bin",
    "LC_ALL": "C.UTF-8",
    "LANG": "C.UTF-8",
    "TZ": "UTC0",
}
CONSOLE_LIMIT = 1_048_576
RUNTIME_TIMEOUT_SECONDS = 105
FRAME_MAGIC = b"R8C1GENCASEV5\0\0\0"
FRAME_HEADER = struct.Struct(">16sIIQQQQ")
FRAME_VERSION = 1
FRAME_METADATA_LIMIT = 65_536
INPUT_MAGIC = b"R8C1INPUTV5\0\0\0\0\0"
WORK_TMPFS_CEILING = 67_108_864


class GuestError(RuntimeError):
    """A fail-closed guest copy, execution, audit, or export error."""


def _close_transport_descriptors() -> None:
    """Make a second, explicit close-on-entry barrier before candidate exec."""

    soft_limit = resource.getrlimit(resource.RLIMIT_NOFILE)[0]
    ceiling = 1_048_576 if soft_limit == resource.RLIM_INFINITY else int(soft_limit)
    os.closerange(3, max(3, ceiling))


def _open_regular(path: str, *, expected_size: int | None = None, require_empty: bool = False) -> int:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise GuestError(f"unsafe regular-file contract: {path}")
        if expected_size is not None and metadata.st_size != expected_size:
            raise GuestError(f"unexpected file size: {path}")
        if require_empty and metadata.st_size != 0:
            raise GuestError(f"pre-created export is not empty: {path}")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _read_all(descriptor: int, *, limit: int) -> bytes:
    result = bytearray()
    while True:
        block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
        if not block:
            return bytes(result)
        result.extend(block)
        if len(result) > limit:
            raise GuestError("regular file exceeds its frozen byte ceiling")


def _read_exact_stdin(size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        block = os.read(0, size - len(result))
        if not block:
            raise GuestError("host-to-guest input frame ended early")
        result.extend(block)
    return bytes(result)


def _copy_framed_inputs() -> dict[str, dict[str, Any]]:
    if _read_exact_stdin(len(INPUT_MAGIC)) != INPUT_MAGIC:
        raise GuestError("host-to-guest input frame magic differs")
    os.mkdir(RUNTIME_ROOT, 0o700)
    copied: dict[str, dict[str, Any]] = {}
    for name, (expected_hash, expected_size, final_mode) in INPUTS.items():
        destination = os.open(
            f"{RUNTIME_ROOT}/{name}",
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o400,
        )
        digest = hashlib.sha256()
        written = 0
        try:
            remaining = expected_size
            while remaining:
                block = _read_exact_stdin(min(1 << 20, remaining))
                digest.update(block)
                written += len(block)
                remaining -= len(block)
                view = memoryview(block)
                while view:
                    count = os.write(destination, view)
                    if count <= 0:
                        raise GuestError(f"short guest copy: {name}")
                    view = view[count:]
            if written != expected_size or digest.hexdigest() != expected_hash:
                raise GuestError(f"transport identity differs: {name}")
            os.fsync(destination)
        finally:
            os.close(destination)
        copied[name] = {"sha256": digest.hexdigest(), "size_bytes": written}

    if os.read(0, 1) != b"":
        raise GuestError("host-to-guest input frame has trailing bytes")
    os.close(0)
    # Keep descriptor number 0 permanently occupied by a guest-internal EOF
    # pipe.  Otherwise a later open()/pipe() could reuse fd 0 and accidentally
    # become candidate stdin despite close_fds=True.
    read_end, write_end = os.pipe2(os.O_CLOEXEC)
    os.close(write_end)
    if read_end != 0:
        os.dup2(read_end, 0, inheritable=True)
        os.close(read_end)
    else:
        os.set_inheritable(0, True)

    ordered_paths = [f"{RUNTIME_ROOT}/{name}" for name in INPUTS]
    checked = subprocess.run(
        [SHA256SUM, "--binary", "--", *ordered_paths],
        # fd 0 was consumed to strict EOF and closed; do not reopen /dev/null
        # because the guest intentionally has no /dev mount.
        stdin=None,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env=ENVIRONMENT,
        close_fds=True,
        timeout=15,
        check=False,
    )
    if checked.returncode != 0 or len(checked.stdout) > 4096 or checked.stderr:
        raise GuestError("pinned sha256sum rejected the final guest copies")
    lines = checked.stdout.decode("ascii", "strict").splitlines()
    if len(lines) != len(ordered_paths):
        raise GuestError("pinned sha256sum output count differs")
    for line, name, path in zip(lines, INPUTS, ordered_paths, strict=True):
        expected_hash = INPUTS[name][0]
        if line != f"{expected_hash} *{path}":
            raise GuestError(f"pinned sha256sum output differs: {name}")

    for name, (_digest, _size, final_mode) in INPUTS.items():
        os.chmod(f"{RUNTIME_ROOT}/{name}", final_mode, follow_symlinks=False)
    return copied


def _apply_candidate_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (1_073_741_824, 1_073_741_824))
    resource.setrlimit(resource.RLIMIT_CPU, (90, 95))
    resource.setrlimit(resource.RLIMIT_NPROC, (8, 8))
    resource.setrlimit(resource.RLIMIT_NOFILE, (256, 256))
    resource.setrlimit(resource.RLIMIT_FSIZE, (16_777_216, 16_777_216))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def _verify_guest_identity() -> dict[str, Any]:
    fields: dict[str, str] = {}
    with open("/proc/self/status", "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    try:
        uid = [int(value) for value in fields["Uid"].split()]
        gid = [int(value) for value in fields["Gid"].split()]
        groups = [int(value) for value in fields.get("Groups", "").split()]
        caps = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}
        no_new_privs = int(fields["NoNewPrivs"])
    except (KeyError, ValueError) as exc:
        raise GuestError("cannot parse final guest identity status") from exc
    if uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups:
        raise GuestError("final guest UID/GID/group identity differs")
    if any(caps.values()) or no_new_privs != 1:
        raise GuestError("final guest capabilities or NoNewPrivs boundary differs")
    return {"uid": uid, "gid": gid, "groups": groups, "capabilities": caps, "no_new_privs": no_new_privs}


def _verify_guest_label() -> str:
    with open("/proc/self/attr/current", "r", encoding="ascii") as source:
        observed = source.read().strip()
    expected = BOOTSTRAP_PROFILE + " (enforce)"
    if observed != expected:
        raise GuestError("guest helper AppArmor label differs")
    return observed


def _verify_work_tmpfs() -> dict[str, Any]:
    matches: list[dict[str, Any]] = []
    with open("/proc/self/mountinfo", "r", encoding="utf-8") as source:
        for line in source:
            fields = line.rstrip("\n").split()
            try:
                separator = fields.index("-")
            except ValueError as exc:
                raise GuestError("guest mountinfo entry lacks a separator") from exc
            if len(fields) <= separator + 2:
                raise GuestError("guest mountinfo entry is truncated")
            if fields[4] == "/work":
                matches.append(
                    {
                        "mountpoint": fields[4],
                        "mount_options": fields[5].split(","),
                        "filesystem": fields[separator + 1],
                        "super_options": fields[separator + 3].split(",") if len(fields) > separator + 3 else [],
                    }
                )
    if len(matches) != 1 or matches[0]["filesystem"] != "tmpfs":
        raise GuestError("/work is not exactly one guest tmpfs mount")
    facts = os.statvfs("/work")
    total = facts.f_blocks * facts.f_frsize
    if total != WORK_TMPFS_CEILING:
        raise GuestError("/work tmpfs does not have the exact 64-MiB byte ceiling")
    matches[0]["total_bytes"] = total
    matches[0]["inode_ceiling_claimed"] = False
    return matches[0]


def _run_candidate() -> tuple[int, bytes, bool, bool, str]:
    started = time.monotonic()
    process = subprocess.Popen(
        GENCASE_ARGV,
        # Inherit the deliberately closed fd 0; never add a guest /dev mount.
        stdin=None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=RUNTIME_ROOT,
        env=ENVIRONMENT,
        close_fds=True,
        # A private process group lets the bootstrap label prove that no forked
        # GenCase descendant survives into output audit/export.
        start_new_session=True,
    )
    if process.stdout is None:
        raise GuestError("candidate console pipe is unavailable")
    candidate_label = ""
    label_deadline = time.monotonic() + 2
    while time.monotonic() < label_deadline:
        try:
            with open(f"/proc/{process.pid}/attr/current", "r", encoding="ascii") as source:
                candidate_label = source.read().strip()
        except FileNotFoundError:
            if process.poll() is not None:
                break
            time.sleep(0.01)
            continue
        if candidate_label == BOOTSTRAP_PROFILE + " (enforce)":
            break
        if process.poll() is not None:
            break
        time.sleep(0.01)
    if candidate_label != BOOTSTRAP_PROFILE + " (enforce)":
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2)
        raise GuestError("GenCase did not inherit the exact bootstrap AppArmor label")
    descriptor = process.stdout.fileno()
    os.set_blocking(descriptor, False)
    console = bytearray()
    overflow = False
    timed_out = False
    eof = False
    termination_started: float | None = None
    sent_kill = False
    while not eof or process.poll() is None:
        elapsed = time.monotonic() - started
        if elapsed >= RUNTIME_TIMEOUT_SECONDS and termination_started is None:
            timed_out = True
            termination_started = time.monotonic()
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if termination_started is not None and not sent_kill and time.monotonic() - termination_started >= 5:
            sent_kill = True
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        readable, _writable, _exceptional = select.select([descriptor], [], [], 0.2)
        if readable:
            try:
                block = os.read(descriptor, 65_536)
            except BlockingIOError:
                block = b""
            if block:
                remaining = max(0, CONSOLE_LIMIT - len(console))
                console.extend(block[:remaining])
                overflow = overflow or len(block) > remaining
            elif process.poll() is not None:
                eof = True
        elif process.poll() is not None:
            try:
                block = os.read(descriptor, 65_536)
            except BlockingIOError:
                block = b""
            if block:
                remaining = max(0, CONSOLE_LIMIT - len(console))
                console.extend(block[:remaining])
                overflow = overflow or len(block) > remaining
            else:
                eof = True
    returncode = process.wait()
    # A leader can exit while a forked descendant remains in its process group.
    # Such residue is terminated and always makes the attempt fail before audit.
    group_residue = False
    try:
        os.killpg(process.pid, 0)
        group_residue = True
    except ProcessLookupError:
        pass
    if group_residue:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        # Do not merely send KILL: observe the group absent repeatedly.  Any
        # escaped set/session descendant remains covered by the outer
        # attr/current label scan before host export.
        for _scan in range(3):
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                time.sleep(0.05)
                continue
            raise GuestError("candidate process group still exists after cleanup")
        raise GuestError("candidate process-group residue existed after leader exit")
    return returncode, bytes(console), overflow, timed_out, candidate_label


def _audit_transient_outputs() -> tuple[dict[str, dict[str, Any]], dict[str, bytes]]:
    names = os.listdir(OUTPUT_ROOT)
    if sorted(names) != sorted(EXPECTED_OUTPUTS):
        raise GuestError(f"transient output set differs: {sorted(names)}")
    observed: dict[str, dict[str, Any]] = {}
    payloads: dict[str, bytes] = {}
    for name, (minimum, maximum) in EXPECTED_OUTPUTS.items():
        descriptor = _open_regular(f"{OUTPUT_ROOT}/{name}")
        try:
            metadata = os.fstat(descriptor)
            if not minimum <= metadata.st_size <= maximum:
                raise GuestError(f"transient output size differs: {name}")
            raw = _read_all(descriptor, limit=maximum)
        finally:
            os.close(descriptor)
        observed[name] = {
            "sha256": hashlib.sha256(raw).hexdigest(),
            "size_bytes": len(raw),
        }
        payloads[name] = raw
        if name.endswith(".bi4"):
            _audit_bi4_header(raw)
        if name.endswith(".xml"):
            try:
                root = ET.fromstring(raw)
            except ET.ParseError as exc:
                raise GuestError("generated C1 XML is malformed") from exc
            if root.tag != "case" or root.find("./casedef/geometry/definition[@dp='0.002']") is None:
                raise GuestError("generated C1 XML does not retain the frozen case/dp contract")
            boundary = root.findall(".//drawcylinder[@mask='2']")
            if len(boundary) != 1:
                raise GuestError("generated C1 XML does not retain exactly one boundary mask=2 cylinder")
            parameters = root.findall(".//parameter")
            keyed = {element.get("key"): element.get("value") for element in parameters}
            if keyed.get("MinFluidStop") != "1" or "PartsOutMax" in keyed:
                raise GuestError("generated C1 XML lost the MinFluidStop/PartsOutMax correction")
            forbidden = {"motion", "floating", "inout", "wavegen"}
            if forbidden.intersection(element.tag for element in root.iter()):
                raise GuestError("generated C1 XML enables a forbidden dynamic feature")
        if name.endswith(".out"):
            _audit_generated_out(raw)
    return observed, payloads


def _audit_generated_out(raw: bytes) -> None:
    """Require the deterministic GenCase summary needed for host re-audit."""

    if b"\0" in raw:
        raise GuestError("generated C1 OUT contains a NUL byte")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise GuestError("generated C1 OUT is not strict UTF-8 text") from exc
    required = (
        "GenCase v5.4.354.01",
        "Distance between points (Dp): 0.002",
        "Total particles: 9,078 (bound=2669 (fx=2669 mv=0 ft=0) fluid=6409)",
        "X range: -0.017 to 0.019 [m]",
        "Y range: -0.017 to 0.019 [m]",
        "Z range: 0 to 0.066 [m]",
        "Finished execution",
    )
    if any(marker not in text for marker in required):
        raise GuestError("generated C1 OUT lacks the frozen GenCase/particle summary")


def _audit_bi4_header(raw: bytes) -> None:
    """Minimum source-derived JBinaryData/JPartDataBi4 structure check."""

    if len(raw) < 94 or not raw.startswith(b"#FileJBD JPartDataBi4"):
        raise GuestError("generated BI4 file header/code differs")
    if raw[58] != 0x0A or raw[59] != 0 or raw[60] != 0 or raw[61] != 0 or raw[62:64] != b"\0\0":
        raise GuestError("generated BI4 byte-order/si64 header differs")
    size_item_definition = struct.unpack_from("<I", raw, 64)[0]
    if size_item_definition < 30 or size_item_definition > len(raw) - 68:
        raise GuestError("generated BI4 item-definition size is out of bounds")
    if struct.unpack_from("<I", raw, 68)[0] != 6 or raw[72:78] != b"\nITEM\n":
        raise GuestError("generated BI4 item-definition marker differs")
    if struct.unpack_from("<I", raw, 78)[0] != 12 or raw[82:94] != b"JPartDataBi4":
        raise GuestError("generated BI4 root item name differs")


def _success_frame(
    *,
    copied: dict[str, dict[str, Any]],
    identity: dict[str, Any],
    work_tmpfs: dict[str, Any],
    guest_label: str,
    candidate_label: str,
    observed: dict[str, dict[str, Any]],
    payloads: dict[str, bytes],
    console: bytes,
) -> bytes:
    metadata = {
        "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_GUEST_FRAME_V5",
        "status": "GUEST_FRAME_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT",
        "gencase_argv": GENCASE_ARGV,
        "guest_inputs": copied,
        "guest_identity": identity,
        "guest_label": guest_label,
        "candidate_label": candidate_label,
        "guest_work_tmpfs": work_tmpfs,
        "guest_outputs": observed,
        "candidate_console": {
            "sha256": hashlib.sha256(console).hexdigest(),
            "size_bytes": len(console),
            "framed_for_separate_0440_console_log": True,
        },
        "stdin_frame_consumed_to_eof_then_replaced_by_guest_eof_pipe": True,
        "candidate_stdout_stderr_were_internal_pipe_only": True,
    }
    encoded = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if len(encoded) > FRAME_METADATA_LIMIT:
        raise GuestError("canonical frame metadata exceeds its fixed ceiling")
    bi4 = payloads["C1_static.bi4"]
    xml = payloads["C1_static.xml"]
    out = payloads["C1_static.out"]
    return (
        FRAME_HEADER.pack(FRAME_MAGIC, FRAME_VERSION, len(encoded), len(bi4), len(xml), len(out), len(console))
        + encoded
        + bi4
        + xml
        + out
        + console
    )


def main() -> int:
    console = b""
    try:
        _close_transport_descriptors()
        copied = _copy_framed_inputs()
        os.mkdir(OUTPUT_ROOT, 0o700)
        _apply_candidate_limits()
        identity = _verify_guest_identity()
        guest_label = _verify_guest_label()
        work_tmpfs = _verify_work_tmpfs()
        returncode, console, overflow, timed_out, candidate_label = _run_candidate()
        if overflow:
            raise GuestError("candidate console exceeded the one-MiB ceiling")
        if timed_out or returncode != 0:
            raise GuestError(f"GenCase failed or timed out: rc={returncode}, timeout={timed_out}")
        observed, payloads = _audit_transient_outputs()
        frame = _success_frame(
            copied=copied,
            identity=identity,
            work_tmpfs=work_tmpfs,
            guest_label=guest_label,
            candidate_label=candidate_label,
            observed=observed,
            payloads=payloads,
            console=console,
        )
        sys.stdout.buffer.write(frame)
        sys.stdout.buffer.flush()
        return 0
    except (GuestError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        failure = {"status": "GUEST_NO_GO", "error": str(exc)}
        sys.stderr.write(json.dumps(failure, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n")
        sys.stderr.flush()
        if console:
            sys.stderr.buffer.write(console)
            sys.stderr.buffer.flush()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
