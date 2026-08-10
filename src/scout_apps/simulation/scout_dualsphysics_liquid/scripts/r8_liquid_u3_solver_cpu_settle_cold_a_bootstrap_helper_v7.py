#!/usr/bin/env python3
"""Guest-only helper for one frozen U3 C1M CPU-solver cold-A settle attempt.

The reviewed helper is compiled from a hash-pinned stdin prefix.  It consumes
the remaining fixed-length input segments, copies every executable, library and
case byte into guest-only tmpfs, closes the transport, applies resource limits,
and starts exactly one copied dynamic loader / CPU solver argv.  A successful
run is returned as one bounded canonical multi-file frame; no guest path is
ever mounted writable on the host.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import resource
import select
import signal
import stat
import struct
import subprocess
import sys
import time
from pathlib import PurePosixPath
from typing import Any, NamedTuple


CASE_ID = "u3_c1m_solver_cpu_settle_cold_a_v7_20260809T051659Z"
CAMPAIGN_ID = "u3_c1m_solver_cpu_settle_ab_campaign_20260809T051659Z"
BOOTSTRAP_PROFILE = "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v7-20260809t051659z"
WORK_ROOT = "/work"
RUNTIME_ROOT = "/work/runtime"
LIB_ROOT = "/work/runtime/lib"
CASE_ROOT = "/work/case"
OUTPUT_ROOT = "/work/output"
TMP_ROOT = "/work/tmp"

INPUT_MAGIC = b"R8SOLVERINPUT7\0\0"
INPUTS: tuple[tuple[str, str, str, int, int], ...] = (
    (
        "DualSPHysics5.4CPU_linux64",
        f"{RUNTIME_ROOT}/DualSPHysics5.4CPU_linux64",
        "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202",
        32_649_520,
        0o500,
    ),
    (
        "DsphConfig.xml",
        f"{RUNTIME_ROOT}/DsphConfig.xml",
        "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
        293,
        0o400,
    ),
    (
        "C1M_zero.xml",
        f"{CASE_ROOT}/C1M_zero.xml",
        "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb",
        7_800,
        0o400,
    ),
    (
        "C1M_zero.bi4",
        f"{CASE_ROOT}/C1M_zero.bi4",
        "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7",
        400_842,
        0o400,
    ),
    (
        "ld-linux-x86-64.so.2",
        f"{RUNTIME_ROOT}/ld-linux-x86-64.so.2",
        "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81",
        236_616,
        0o500,
    ),
    (
        "libgomp.so.1",
        f"{LIB_ROOT}/libgomp.so.1",
        "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1",
        352_304,
        0o400,
    ),
    (
        "libstdc++.so.6",
        f"{LIB_ROOT}/libstdc++.so.6",
        "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11",
        2_592_224,
        0o400,
    ),
    (
        "libm.so.6",
        f"{LIB_ROOT}/libm.so.6",
        "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91",
        952_616,
        0o400,
    ),
    (
        "libgcc_s.so.1",
        f"{LIB_ROOT}/libgcc_s.so.1",
        "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb",
        183_024,
        0o400,
    ),
    (
        "libc.so.6",
        f"{LIB_ROOT}/libc.so.6",
        "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161",
        2_125_328,
        0o400,
    ),
)

SOLVER_ARGV = [
    f"{RUNTIME_ROOT}/ld-linux-x86-64.so.2",
    "--inhibit-cache",
    "--library-path",
    LIB_ROOT,
    f"{RUNTIME_ROOT}/DualSPHysics5.4CPU_linux64",
    f"{CASE_ROOT}/C1M_zero",
    OUTPUT_ROOT,
    "-cpu",
    "-ompthreads:1",
    "-stable:1",
    "-vres:0",
    "-cellmode:full",
    "-tmax:8.05",
    "-tout:0.05",
    "-sv:binx,info",
    "-svres:1",
    "-svtimers:0",
    "-svdomainvtk:0",
    "-saveposdouble:1",
    "-nortimes:1",
    "-createdirs:1",
    "-csvsep:0",
]

ENVIRONMENT = {
    "HOME": "/nonexistent",
    "PATH": "/usr/bin",
    "LC_ALL": "C",
    "LANG": "C",
    "TZ": "UTC0",
    "TMPDIR": TMP_ROOT,
    "OMP_NUM_THREADS": "1",
    "OMP_DYNAMIC": "FALSE",
    "OMP_PROC_BIND": "FALSE",
}

EXPECTED_ROOT_FILES = {
    "CfgInit_Domain.vtk",
    "CfgInit_MapCells.vtk",
    "Run.csv",
    "Run.out",
    "RunPARTs.csv",
}
EXPECTED_DATA_FILES = {
    "Part_Head.ibi4",
    "PartInfo.ibi4",
    "PartOut_000.obi4",
    "PartMotionRef.ibi4",
    *(f"Part_{index:04d}.bi4" for index in range(162)),
}
EXPECTED_FILE_COUNT = 171

EXPECTED_EXACT_FILE_SIZES = {
    "CfgInit_Domain.vtk": 1_010,
    "CfgInit_MapCells.vtk": 2_386,
    "data/PartInfo.ibi4": 115_257,
    "data/PartMotionRef.ibi4": 40_833,
    "data/PartOut_000.obi4": 584,
    "data/Part_Head.ibi4": 1_554,
    **{f"data/Part_{index:04d}.bi4": 401_318 for index in range(162)},
}

WORK_TMPFS_BYTES = 268_435_456
OUTPUT_TOTAL_LIMIT = 83_886_080
OUTPUT_FILE_LIMIT = 16_777_216
TEXT_FILE_LIMIT = 65_536
CONSOLE_LIMIT = 4_194_304
RUNTIME_TIMEOUT_SECONDS = 10_710
EOF_EXIT_GRACE_SECONDS = 0.2
TERM_GRACE_SECONDS = 10.0
KILL_GRACE_SECONDS = 3.0
GROUP_SCAN_INTERVAL_SECONDS = 0.05

FRAME_MAGIC = b"R8SOLVEROUTV7\0\0\0"
FRAME_VERSION = 7
FRAME_HEADER = struct.Struct(">16sIIQQQ")
FRAME_METADATA_LIMIT = 262_144


class GuestError(RuntimeError):
    """A fail-closed guest copy, execution, output or framing error."""


class ProcessMember(NamedTuple):
    pid: int
    state: str
    pgrp: int
    session: int
    starttime: int


class ProcessGroupIdentity(NamedTuple):
    pid: int
    pgid: int
    session: int
    starttime: int
    pidfd: int


def _close_transport_descriptors() -> None:
    soft = resource.getrlimit(resource.RLIMIT_NOFILE)[0]
    ceiling = 1_048_576 if soft == resource.RLIM_INFINITY else int(soft)
    os.closerange(3, max(3, ceiling))


def _read_exact_stdin(size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        block = os.read(0, size - len(result))
        if not block:
            raise GuestError("host-to-guest input frame ended early")
        result.extend(block)
    return bytes(result)


def _write_all(descriptor: int, raw: bytes) -> None:
    view = memoryview(raw)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            raise GuestError("short guest tmpfs write")
        view = view[count:]


def _copy_inputs() -> dict[str, dict[str, Any]]:
    if _read_exact_stdin(len(INPUT_MAGIC)) != INPUT_MAGIC:
        raise GuestError("host-to-guest input magic differs")
    for path in (RUNTIME_ROOT, LIB_ROOT, CASE_ROOT, OUTPUT_ROOT, TMP_ROOT):
        os.mkdir(path, 0o700)
    observed: dict[str, dict[str, Any]] = {}
    for name, destination, expected_hash, expected_size, final_mode in INPUTS:
        descriptor = os.open(
            destination,
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
                _write_all(descriptor, block)
                written += len(block)
                remaining -= len(block)
            os.fsync(descriptor)
            if written != expected_size or digest.hexdigest() != expected_hash:
                raise GuestError(f"input byte identity differs: {name}")
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise GuestError(f"input inode contract differs: {name}")
            os.fchmod(descriptor, final_mode)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        observed[name] = {
            "guest_path": destination,
            "sha256": digest.hexdigest(),
            "size_bytes": written,
            "mode": f"{final_mode:04o}",
        }
    if os.read(0, 1) != b"":
        raise GuestError("host-to-guest input frame has trailing bytes")
    os.close(0)
    read_end, write_end = os.pipe2(os.O_CLOEXEC)
    os.close(write_end)
    if read_end != 0:
        os.dup2(read_end, 0, inheritable=True)
        os.close(read_end)
    else:
        os.set_inheritable(0, True)
    return observed


def _apply_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (2_147_483_648, 2_147_483_648))
    resource.setrlimit(resource.RLIMIT_CPU, (10_200, 10_210))
    resource.setrlimit(resource.RLIMIT_NPROC, (16, 16))
    resource.setrlimit(resource.RLIMIT_NOFILE, (256, 256))
    resource.setrlimit(resource.RLIMIT_FSIZE, (OUTPUT_FILE_LIMIT, OUTPUT_FILE_LIMIT))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def _status_fields(path: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    with open(path, "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    return fields


def _verify_identity() -> dict[str, Any]:
    fields = _status_fields("/proc/self/status")
    try:
        uid = [int(value) for value in fields["Uid"].split()]
        gid = [int(value) for value in fields["Gid"].split()]
        groups = [int(value) for value in fields.get("Groups", "").split()]
        caps = {
            key: int(fields[key], 16)
            for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")
        }
        no_new_privs = int(fields["NoNewPrivs"])
    except (KeyError, ValueError) as exc:
        raise GuestError("cannot parse guest identity") from exc
    if uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups:
        raise GuestError("guest UID/GID/group identity differs")
    if any(caps.values()) or no_new_privs != 1:
        raise GuestError("guest capabilities or NoNewPrivs differs")
    return {
        "uid": uid,
        "gid": gid,
        "groups": groups,
        "capabilities": caps,
        "no_new_privs": no_new_privs,
    }


def _verify_label(path: str = "/proc/self/attr/current") -> str:
    with open(path, "r", encoding="ascii") as source:
        observed = source.read().strip()
    expected = BOOTSTRAP_PROFILE + " (enforce)"
    if observed != expected:
        raise GuestError("guest AppArmor label differs")
    return observed


def _verify_tmpfs() -> dict[str, Any]:
    matches: list[dict[str, Any]] = []
    with open("/proc/self/mountinfo", "r", encoding="utf-8") as source:
        for line in source:
            fields = line.rstrip("\n").split()
            if "-" not in fields:
                raise GuestError("guest mountinfo entry is malformed")
            separator = fields.index("-")
            if fields[4] == WORK_ROOT:
                matches.append({"mountpoint": fields[4], "filesystem": fields[separator + 1]})
    if matches != [{"mountpoint": WORK_ROOT, "filesystem": "tmpfs"}]:
        raise GuestError("/work is not exactly one guest tmpfs")
    facts = os.statvfs(WORK_ROOT)
    total = facts.f_blocks * facts.f_frsize
    if total != WORK_TMPFS_BYTES:
        raise GuestError("/work tmpfs byte ceiling differs")
    return {**matches[0], "total_bytes": total, "inode_ceiling_claimed": False}


def _read_process_member(
    pid: int, *, skip_ungrouped_kernel_task: bool = False
) -> ProcessMember | None:
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="ascii") as source:
            raw = source.read(4_097)
    except FileNotFoundError:
        return None
    if len(raw) > 4_096 or not raw.endswith("\n"):
        raise GuestError(f"solver process stat is malformed for pid {pid}")
    close = raw.rfind(")")
    if close < 2 or not raw[:close].startswith(f"{pid} ("):
        raise GuestError(f"solver process stat identity is malformed for pid {pid}")
    fields = raw[close + 2 :].split()
    if len(fields) < 20 or len(fields[0]) != 1:
        raise GuestError(f"solver process stat fields are malformed for pid {pid}")
    try:
        pgrp = int(fields[2])
        session_id = int(fields[3])
        starttime = int(fields[19])
    except ValueError as exc:
        raise GuestError(f"solver process stat numeric fields are malformed for pid {pid}") from exc
    if pgrp < 0 or session_id < 0 or starttime < 1:
        raise GuestError(f"solver process stat contains a non-positive identity for pid {pid}")
    if pgrp == 0 and session_id == 0 and skip_ungrouped_kernel_task:
        return None
    if pgrp == 0 or session_id == 0:
        raise GuestError(f"solver process stat contains a non-positive identity for pid {pid}")
    return ProcessMember(pid, fields[0], pgrp, session_id, starttime)


def _scan_group_members_once(pgid: int) -> list[ProcessMember]:
    members: list[ProcessMember] = []
    with os.scandir("/proc") as entries:
        for entry in entries:
            if not entry.name.isascii() or not entry.name.isdecimal():
                continue
            member = _read_process_member(
                int(entry.name), skip_ungrouped_kernel_task=True
            )
            if member is not None and member.pgrp == pgid:
                members.append(member)
    return sorted(members)


def _group_members(pgid: int) -> list[ProcessMember]:
    observed: dict[tuple[int, int, int, int], ProcessMember] = {}
    for scan in (_scan_group_members_once(pgid), _scan_group_members_once(pgid)):
        for member in scan:
            key = (member.pid, member.pgrp, member.session, member.starttime)
            previous = observed.get(key)
            if previous is None:
                observed[key] = member
            elif previous.state != member.state:
                observed[key] = member._replace(state="?")
    return sorted(observed.values())


def _capture_process_group(process: subprocess.Popen[bytes]) -> ProcessGroupIdentity:
    if not hasattr(os, "pidfd_open"):
        raise GuestError("pidfd_open is unavailable for solver ownership")
    pidfd = os.pidfd_open(process.pid, 0)
    try:
        first = _read_process_member(process.pid)
        second = _read_process_member(process.pid)
        if first is None or second is None:
            raise GuestError("solver leader disappeared before identity freeze")
        first_identity = (first.pid, first.pgrp, first.session, first.starttime)
        second_identity = (second.pid, second.pgrp, second.session, second.starttime)
        if first_identity != second_identity:
            raise GuestError("solver leader identity changed during freeze")
        pgid = os.getpgid(process.pid)
        session_id = os.getsid(process.pid)
        if (
            process.pid != pgid
            or process.pid != session_id
            or first.pgrp != pgid
            or first.session != session_id
        ):
            raise GuestError("solver leader is not its frozen session and group leader")
        return ProcessGroupIdentity(process.pid, pgid, session_id, first.starttime, pidfd)
    except BaseException:
        os.close(pidfd)
        raise


def _validate_leader_anchor(identity: ProcessGroupIdentity) -> ProcessMember:
    first = _read_process_member(identity.pid)
    second = _read_process_member(identity.pid)
    if first is None or second is None:
        raise GuestError("solver leader disappeared before explicit reap")
    expected = (identity.pid, identity.pgid, identity.session, identity.starttime)
    if (
        (first.pid, first.pgrp, first.session, first.starttime) != expected
        or (second.pid, second.pgrp, second.session, second.starttime) != expected
    ):
        raise GuestError("solver leader birth, PGID, or SID changed")
    return first if first.state == second.state else first._replace(state="?")


def _preflight_group_scan(identity: ProcessGroupIdentity) -> list[ProcessMember]:
    """Exercise the full /proc scan before allowing a long-running solver."""

    members = _group_members(identity.pgid)
    expected = (identity.pid, identity.pgid, identity.session, identity.starttime)
    if not any(
        (member.pid, member.pgrp, member.session, member.starttime) == expected
        for member in members
    ):
        raise GuestError("solver leader is absent from the initial process-group scan")
    return members


def _only_owned_leader_zombie(
    members: list[ProcessMember], identity: ProcessGroupIdentity
) -> bool:
    return members == [
        ProcessMember(
            identity.pid,
            "Z",
            identity.pgid,
            identity.session,
            identity.starttime,
        )
    ]


def _wait_only_owned_leader_zombie(
    identity: ProcessGroupIdentity, timeout_seconds: float
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_seconds)
    while True:
        if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(GROUP_SCAN_INTERVAL_SECONDS, remaining))


def _signal_owned_group(identity: ProcessGroupIdentity, sig: signal.Signals) -> None:
    _validate_leader_anchor(identity)
    try:
        os.killpg(identity.pgid, sig)
    except ProcessLookupError:
        if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
            return
        raise GuestError("solver process group disappeared inconsistently during signal")


def _reap_owned_leader(
    process: subprocess.Popen[bytes], identity: ProcessGroupIdentity
) -> int:
    if not _only_owned_leader_zombie(_group_members(identity.pgid), identity):
        raise GuestError("refusing to reap solver leader before all same-PGID descendants are gone")
    returncode = process.wait()
    if process.poll() != returncode:
        raise GuestError("solver return code changed after explicit reap")
    residue = _group_members(identity.pgid)
    if residue:
        raise GuestError(f"solver process group remains after leader reap: {residue!r}")
    return returncode


def _stop_group(
    process: subprocess.Popen[bytes], identity: ProcessGroupIdentity
) -> int:
    members = _group_members(identity.pgid)
    if _only_owned_leader_zombie(members, identity):
        return _reap_owned_leader(process, identity)
    _signal_owned_group(identity, signal.SIGTERM)
    if not _wait_only_owned_leader_zombie(identity, TERM_GRACE_SECONDS):
        _signal_owned_group(identity, signal.SIGKILL)
        if not _wait_only_owned_leader_zombie(identity, KILL_GRACE_SECONDS):
            raise GuestError("solver process group remains after TERM/KILL cleanup")
    return _reap_owned_leader(process, identity)


def _finish_after_console_eof(
    process: subprocess.Popen[bytes],
    identity: ProcessGroupIdentity,
    runtime_deadline: float,
) -> int:
    remaining = max(0.0, runtime_deadline - time.monotonic())
    grace = min(EOF_EXIT_GRACE_SECONDS, remaining)
    if not _wait_only_owned_leader_zombie(identity, grace):
        raise GuestError("solver leader or same-PGID descendant remained alive after console EOF grace")
    return _reap_owned_leader(process, identity)


def _raise_primary_or_cleanup(
    primary: BaseException | None, cleanup: BaseException | None
) -> None:
    if primary is not None:
        if cleanup is not None:
            if not isinstance(primary, Exception):
                primary.add_note(f"solver cleanup failure: {cleanup}")
                raise primary
            raise GuestError(f"{primary}; solver cleanup failure: {cleanup}") from primary
        raise primary
    if cleanup is not None:
        raise cleanup


def _run_solver() -> tuple[int, bytes, bool, str]:
    process = subprocess.Popen(
        SOLVER_ARGV,
        stdin=None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=RUNTIME_ROOT,
        env=ENVIRONMENT,
        close_fds=True,
        start_new_session=True,
    )
    identity: ProcessGroupIdentity | None = None
    candidate_label = ""
    console = bytearray()
    overflow = False
    timed_out = False
    primary_error: BaseException | None = None
    cleanup_error: BaseException | None = None
    returncode: int | None = None
    try:
        identity = _capture_process_group(process)
        _preflight_group_scan(identity)
        if process.stdout is None:
            primary_error = GuestError("solver console pipe is unavailable")
        label_deadline = time.monotonic() + 3
        while primary_error is None and time.monotonic() < label_deadline:
            try:
                with open(f"/proc/{process.pid}/attr/current", "r", encoding="ascii") as source:
                    candidate_label = source.read().strip()
            except FileNotFoundError:
                if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
                    break
                time.sleep(0.01)
                continue
            if candidate_label == BOOTSTRAP_PROFILE + " (enforce)":
                break
            if _only_owned_leader_zombie(_group_members(identity.pgid), identity):
                break
            time.sleep(0.01)
        if primary_error is None and candidate_label != BOOTSTRAP_PROFILE + " (enforce)":
            primary_error = GuestError("copied loader did not inherit the bootstrap label")

        if primary_error is None:
            descriptor = process.stdout.fileno()
            os.set_blocking(descriptor, False)
            eof = False
            runtime_deadline = time.monotonic() + RUNTIME_TIMEOUT_SECONDS
            while not eof:
                if time.monotonic() >= runtime_deadline:
                    timed_out = True
                    break
                readable, _writable, _exceptional = select.select(
                    [descriptor], [], [], 0.2
                )
                if not readable:
                    continue
                try:
                    block = os.read(descriptor, 65_536)
                except BlockingIOError:
                    continue
                if block:
                    room = max(0, CONSOLE_LIMIT - len(console))
                    console.extend(block[:room])
                    overflow = overflow or len(block) > room
                else:
                    eof = True
            if not timed_out:
                try:
                    returncode = _finish_after_console_eof(
                        process, identity, runtime_deadline
                    )
                except BaseException as exc:
                    primary_error = exc
    except BaseException as exc:
        primary_error = exc
    finally:
        if identity is not None and returncode is None:
            try:
                returncode = _stop_group(process, identity)
            except BaseException as exc:
                cleanup_error = exc
        elif identity is None:
            try:
                # start_new_session=True completed before Popen returned, and
                # this unreaped child PID cannot be reused.  Its numeric PID is
                # therefore still the initial solver PGID even if identity
                # capture failed; kill the whole initial group, not just leader.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                returncode = process.wait(timeout=2)
            except BaseException as exc:
                cleanup_error = exc
        if process.stdout is not None:
            try:
                process.stdout.close()
            except OSError as exc:
                cleanup_error = cleanup_error or exc
        if identity is not None:
            try:
                os.close(identity.pidfd)
            except OSError as exc:
                cleanup_error = cleanup_error or exc
    if primary_error is None and overflow:
        primary_error = GuestError("solver console exceeded four MiB")
    if primary_error is None and timed_out and cleanup_error is not None:
        primary_error = GuestError(
            f"CPU solver failed or timed out: rc={returncode}, timeout=True"
        )
    _raise_primary_or_cleanup(primary_error, cleanup_error)
    if returncode is None:
        raise GuestError("solver return code is unavailable after cleanup")
    return returncode, bytes(console), timed_out, candidate_label


def _read_regular(path: str, *, limit: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or metadata.st_size < 1
            or metadata.st_size > limit
        ):
            raise GuestError(f"unsafe or oversized output: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise GuestError(f"short output read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GuestError(f"output grew during audit: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def _decode_text(name: str, raw: bytes) -> str:
    if b"\0" in raw:
        raise GuestError(f"text output contains NUL: {name}")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeDecodeError as exc:
        raise GuestError(f"text output is not strict UTF-8: {name}") from exc
    if re.search(r"(?<![A-Za-z])(?:nan|[-+]?inf)(?![A-Za-z])", text, re.IGNORECASE):
        raise GuestError(f"text output reports non-finite data: {name}")
    return text


def _integer(text: str, label: str) -> int:
    try:
        return int(text.strip().replace(",", ""))
    except ValueError as exc:
        raise GuestError(f"invalid integer {label}") from exc


def _float(text: str, label: str) -> float:
    try:
        value = float(text.strip())
    except ValueError as exc:
        raise GuestError(f"invalid float {label}") from exc
    if not (-1.0e300 < value < 1.0e300):
        raise GuestError(f"non-finite float {label}")
    return value


def _audit_runparts(raw: bytes) -> dict[str, Any]:
    text = _decode_text("RunPARTs.csv", raw)
    lines = [line for line in text.splitlines() if line]
    try:
        header_index = next(index for index, line in enumerate(lines) if line.startswith("Part;"))
    except StopIteration as exc:
        raise GuestError("RunPARTs.csv header is absent") from exc
    header = lines[header_index].split(";")
    rows = []
    for line in lines[header_index + 1 :]:
        if line.startswith("#"):
            break
        fields = line.split(";")
        if len(fields) != len(header):
            raise GuestError("RunPARTs.csv row width differs")
        rows.append(dict(zip(header, fields, strict=True)))
    if len(rows) != 162:
        raise GuestError("RunPARTs.csv does not contain exactly 162 PART rows")
    required = {
        "Part",
        "TimeStep [s]",
        "NpSave",
        "NpSim",
        "NpOut",
        "NpbSim",
        "NpfSim",
        "NpNormal",
        "NpOutPos",
        "NpOutRho",
        "NpOutMov",
    }
    if not required.issubset(header):
        raise GuestError("RunPARTs.csv required columns are absent")
    times: list[float] = []
    for expected_part, row in enumerate(rows):
        if _integer(row["Part"], "Part") != expected_part:
            raise GuestError("RunPARTs.csv PART sequence differs")
        time_value = _float(row["TimeStep [s]"], "TimeStep")
        if times and time_value <= times[-1]:
            raise GuestError("RunPARTs.csv time is not strictly increasing")
        times.append(time_value)
        for field, expected in (
            ("NpSave", 9_078),
            ("NpSim", 9_078),
            ("NpOut", 0),
            ("NpbSim", 2_669),
            ("NpfSim", 6_409),
            ("NpNormal", 9_078),
            ("NpOutPos", 0),
            ("NpOutRho", 0),
            ("NpOutMov", 0),
        ):
            if _integer(row[field], field) != expected:
                raise GuestError(f"RunPARTs.csv {field} contract differs")
    if times[0] != 0 or not 8.05 <= times[-1] < 8.10:
        raise GuestError("RunPARTs.csv physical-time coverage differs")
    return {"rows": len(rows), "first_time": times[0], "final_time": times[-1]}


def _audit_run_csv(raw: bytes) -> dict[str, Any]:
    text = _decode_text("Run.csv", raw)
    lines = [line for line in text.splitlines() if line]
    if len(lines) != 2:
        raise GuestError("Run.csv must contain exactly one header and one data row")
    header = lines[0].lstrip("#").split(";")
    values = lines[1].split(";")
    if len(header) != len(values):
        raise GuestError("Run.csv row width differs")
    row = dict(zip(header, values, strict=True))
    for field in ("Np", "PhysicalTime", "PartFiles", "PartsOut", "Nbound", "Nfixed"):
        if field not in row:
            raise GuestError(f"Run.csv required field is absent: {field}")
    physical_time = _float(row["PhysicalTime"], "Run.csv PhysicalTime")
    if (
        _integer(row["Np"], "Run.csv Np") != 9_078
        or _integer(row["PartFiles"], "Run.csv PartFiles") != 162
        or _integer(row["PartsOut"], "Run.csv PartsOut") != 0
        or _integer(row["Nbound"], "Run.csv Nbound") != 2_669
        or _integer(row["Nfixed"], "Run.csv Nfixed") != 0
        or not 8.05 <= physical_time < 8.10
    ):
        raise GuestError("Run.csv completion summary differs")
    return {"physical_time": physical_time, "part_files": 162, "parts_out": 0}


def _audit_run_out(raw: bytes) -> None:
    text = _decode_text("Run.out", raw)
    required = (
        "DualSPHysics5 v6.4.355",
        "[Simulation finished",
        "Particles of simulation (initial):",
        "CaseNfixed=0",
        "CaseNmoving=2,669",
        "Excluded particles...............: 0",
        "Finished execution (code=0).",
    )
    if any(marker not in text for marker in required):
        raise GuestError("Run.out completion markers differ")
    forbidden = (
        "[Simulation INTERRUPTED",
        "minimum number of fluid particles",
        "*** Exception",
    )
    if any(marker in text for marker in forbidden):
        raise GuestError("Run.out reports interruption, loss or exception")


def _audit_jbd_header(name: str, raw: bytes) -> None:
    if len(raw) < 64 or not raw.startswith(b"#FileJBD ") or raw[58] != 0x0A:
        raise GuestError(f"JBD header differs: {name}")
    if name.startswith("data/Part_") and name.endswith(".bi4"):
        if raw[:58].decode("ascii", "strict").rstrip(" ") != "#FileJBD JPartDataBi4":
            raise GuestError(f"PART BI4 code differs: {name}")
    if name == "data/PartMotionRef.ibi4":
        if raw[:58].decode("ascii", "strict").rstrip(" ") != "#FileJBD JPartMotRefBi4":
            raise GuestError("moving-reference BI4 code differs")


def _audit_outputs() -> tuple[dict[str, bytes], dict[str, Any]]:
    root_entries = {entry.name: entry for entry in os.scandir(OUTPUT_ROOT)}
    if set(root_entries) != EXPECTED_ROOT_FILES | {"data"}:
        raise GuestError(f"solver output root set differs: {sorted(root_entries)}")
    data_entry = root_entries["data"]
    if not data_entry.is_dir(follow_symlinks=False) or data_entry.is_symlink():
        raise GuestError("solver data output is not one real directory")
    data_entries = {entry.name: entry for entry in os.scandir(f"{OUTPUT_ROOT}/data")}
    if set(data_entries) != EXPECTED_DATA_FILES:
        raise GuestError(f"solver data file set differs: {sorted(data_entries)}")
    if list(os.scandir(TMP_ROOT)):
        raise GuestError("solver left unexpected guest temporary files")

    paths = sorted(EXPECTED_ROOT_FILES) + [f"data/{name}" for name in sorted(EXPECTED_DATA_FILES)]
    if len(paths) != EXPECTED_FILE_COUNT:
        raise GuestError("internal expected output count differs")
    payloads: dict[str, bytes] = {}
    total = 0
    for relative in paths:
        pure = PurePosixPath(relative)
        if pure.is_absolute() or ".." in pure.parts or len(pure.parts) not in (1, 2):
            raise GuestError("unsafe internal output relative path")
        limit = TEXT_FILE_LIMIT if pure.suffix in (".csv", ".out", ".vtk") else OUTPUT_FILE_LIMIT
        raw = _read_regular(f"{OUTPUT_ROOT}/{relative}", limit=limit)
        exact_size = EXPECTED_EXACT_FILE_SIZES.get(relative)
        if exact_size is not None and len(raw) != exact_size:
            raise GuestError(f"solver output exact size differs: {relative}")
        total += len(raw)
        if total > OUTPUT_TOTAL_LIMIT:
            raise GuestError("solver output aggregate exceeds 80 MiB")
        payloads[relative] = raw
        if pure.suffix in (".bi4", ".ibi4", ".obi4"):
            _audit_jbd_header(relative, raw)
    _audit_run_out(payloads["Run.out"])
    run_csv = _audit_run_csv(payloads["Run.csv"])
    runparts = _audit_runparts(payloads["RunPARTs.csv"])
    return payloads, {
        "file_count": len(payloads),
        "total_bytes": total,
        "run_csv": run_csv,
        "runparts": runparts,
    }


def _build_success_frame(
    *,
    inputs: dict[str, dict[str, Any]],
    identity: dict[str, Any],
    label: str,
    candidate_label: str,
    tmpfs: dict[str, Any],
    payloads: dict[str, bytes],
    output_audit: dict[str, Any],
    console: bytes,
) -> bytes:
    manifest = [
        {"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}
        for path, raw in sorted(payloads.items())
    ]
    metadata = {
        "document_type": "SMPCC_R8_LIQUID_U3_C1M_CPU_SOLVER_SETTLE_COLD_A_GUEST_FRAME_V7",
        "status": "GUEST_SOLVER_V7_COLD_A_SETTLE_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT",
        "solver_argv": SOLVER_ARGV,
        "environment": ENVIRONMENT,
        "guest_inputs": inputs,
        "guest_identity": identity,
        "guest_label": label,
        "candidate_label": candidate_label,
        "guest_work_tmpfs": tmpfs,
        "output_audit": output_audit,
        "output_manifest": manifest,
        "console": {"sha256": hashlib.sha256(console).hexdigest(), "size_bytes": len(console)},
        "stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe": True,
        "host_writable_bind_count": 0,
    }
    metadata_raw = json.dumps(
        metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    if len(metadata_raw) > FRAME_METADATA_LIMIT:
        raise GuestError("solver frame metadata exceeds fixed ceiling")
    ordered_payload = b"".join(payloads[item["path"]] for item in manifest)
    if len(ordered_payload) != output_audit["total_bytes"]:
        raise GuestError("solver frame payload aggregate differs")
    return (
        FRAME_HEADER.pack(
            FRAME_MAGIC,
            FRAME_VERSION,
            len(metadata_raw),
            len(manifest),
            len(ordered_payload),
            len(console),
        )
        + metadata_raw
        + ordered_payload
        + console
    )


def main() -> int:
    console = b""
    candidate_pid: int | None = None
    try:
        _close_transport_descriptors()
        inputs = _copy_inputs()
        _apply_limits()
        identity = _verify_identity()
        label = _verify_label()
        tmpfs = _verify_tmpfs()
        returncode, console, timed_out, candidate_label = _run_solver()
        if timed_out or returncode != 0:
            raise GuestError(f"CPU solver failed or timed out: rc={returncode}, timeout={timed_out}")
        payloads, output_audit = _audit_outputs()
        frame = _build_success_frame(
            inputs=inputs,
            identity=identity,
            label=label,
            candidate_label=candidate_label,
            tmpfs=tmpfs,
            payloads=payloads,
            output_audit=output_audit,
            console=console,
        )
        sys.stdout.buffer.write(frame)
        sys.stdout.buffer.flush()
        return 0
    except (GuestError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        failure = {"status": "GUEST_SOLVER_V7_COLD_A_NO_GO", "error": str(exc)}
        sys.stderr.write(json.dumps(failure, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n")
        if console:
            sys.stderr.buffer.write(console[:CONSOLE_LIMIT])
        sys.stderr.buffer.flush()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
