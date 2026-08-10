#!/usr/bin/env python3
"""Static gate and unprivileged runner for one fresh proc-stat boundary probe.

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
WORKSPACE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
SNAPSHOT_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V1_SNAPSHOT"
ADMISSION_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V1_ADMISSION"
ADMISSION_FD_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V1_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V1_ADMISSION_SHA256"
PROBE_ID = "u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")
ADMISSION_TOKEN = f"{PROBE_ID}:single-proc-stat-boundary-candidate-attempt"
FROZEN_V6_FAILURE = {
    "case_id": "u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z",
    "campaign_id": "u3_c1m_solver_cpu_settle_ab_campaign_20260808T212044Z",
    "policy": {
        "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_solver_cpu_settle_cold_a_execution_policy_v6.json",
        "sha256": "0affbcabd5b587464b1a32c38ebd2abb139063c4f50000e0252801d199d59270",
        "size_bytes": 39_382,
    },
    "receipts": {
        "start": {
            "path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.start.json",
            "sha256": "07d1c2c3e212f8cd791531a59933695e7a078c7953808c842d28a19bd21f365f",
            "size_bytes": 11_634,
            "status": "V6_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
        },
        "execution": {
            "path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.execution.json",
            "sha256": "7cba88d4793a58e05d2b7a0c6d843a8d6f0fec2e19541a9ff138a814ab1d4316",
            "size_bytes": 13_787,
            "status": "V6_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        },
        "lifecycle_incomplete": {
            "path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.lifecycle_incomplete.json",
            "sha256": "03958f6def1c609d21bd56120827d818085d5c3b96768a1da032f8b83c46b59f",
            "size_bytes": 6_239,
            "status": "V6_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        },
    },
    "stat_denials": {
        "line_sha256": [
            "7fabb53adb6cf6f8626bdc0056f663a73896dc133dde2bc14d12d9fcf73ad8e2",
            "e511e301bf3cec1956573781301b3827b64085c7b9f17c84d5251ab1def23d17",
        ],
        "line_size_bytes": [320, 320],
        "name": "/proc/3/stat",
        "operation": "open",
        "class": "file",
        "requested_mask": "r",
        "denied_mask": "r",
        "fsuid": "1000",
        "ouid": "0",
    },
    "source_use": "failure_provenance_only_no_snapshot_output_or_runtime_reuse",
    "identity_consumed": True,
    "retry_forbidden": True,
    "cold_b_authorized": False,
    "production_authorized": False,
}
BOOTSTRAP_PROFILE = "r8-liquid-u3-proc-stat-boundary-bootstrap-v1-20260809t002729z"
RUNTIME_PROFILE = "r8-liquid-u3-proc-stat-boundary-runtime-v1-20260809t002729z"
HOST_UID = 1000
HOST_GID = 1000
SUDO_GROUP_GID = 27
SUDO_GROUP_MEMBERSHIP_CONTRACT = {
    "path": "/etc/group",
    "owner": [0, 0],
    "mode": "0644",
    "record": "sudo:x:27:zrj",
    "group_name": "sudo",
    "gid": SUDO_GROUP_GID,
    "members": ["zrj"],
}
SUDO_CLEANUP_BOUNDING_MODE = "preserve_host_only"
SUDO_CLEANUP_IDENTITY_CONTRACT = {
    "reuid": HOST_UID,
    "regid": HOST_GID,
    "supplementary_groups": [SUDO_GROUP_GID],
    "groups_mode": "EXACT_NUMERIC_GROUP_LIST_ONLY",
    "bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
    "bounding_argv_tokens": [],
    "explicit_bounding_change_forbidden": True,
    "inheritable_capabilities": "-all",
    "ambient_capabilities": "-all",
    "no_new_privs_option_present": False,
    "shell": False,
    "start_new_session": False,
    "forbidden_group_options": ["--clear-groups", "--init-groups", "--keep-groups"],
}
TMPFS_BYTES = 67_108_864

POLICY_NAME = "liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v1.json"
SCHEMA_NAME = "target_host_u3_proc_stat_boundary_probe_policy_v1.json"
PROFILE_NAME = "r8-liquid-u3-proc-stat-boundary-probe-v1.profile"
SUPERVISOR_NAME = "r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v1.py"
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_ROOT / "scripts" / SUPERVISOR_NAME
AA_STATUS_JSON_ARGV = ("/usr/sbin/aa-status", "--json")
FROZEN_RECEIPT_PATHS = {
    "start_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.start.json",
    "preflight_failure_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.preflight_incomplete.json",
    "execution_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.execution.json",
    "lifecycle_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.lifecycle.json",
    "lifecycle_failure_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.lifecycle_incomplete.json",
    "recovery_receipt": f"/home/zrj/scout_liquid_lab/audits/{PROBE_ID}.recovery.json",
}

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

PAYLOAD_MAGIC = b"R8PROCSTATPROBE1\x00"
SUCCESS_MAGIC = b"R8PROCSTATPASSV1\x00"
MAX_PAYLOAD_FRAME_BYTES = 65_536
MAX_SUCCESS_FRAME_BYTES = 16_384
MAX_CHILD_STDERR_BYTES = 4_096
OUTER_DEADLINE_SECONDS = 20

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}


PROC_MOUNT_RULE = 'mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,'

EVIDENCE_CLASSIFICATION_CONTRACT = {'success_requires_closed_frame': True, 'successful_gate_exit_is_pass': False, 'success_pass_requires': 'returncode_zero_empty_stderr_byte_exact_success_frame_and_exactly_one_expected_dumpable_zero_child_status_denial_in_closed_boot_cursor_window', 'audit_storage_limit': 16, 'audit_line_limit_bytes': 4096, 'expected_counts': {'matching_total': 1, 'stored_count': 1, 'dropped_count': 0, 'expected_status_total': 1, 'stat_denial_total': 0, 'unexpected_total': 0, 'storage_overflow': False}, 'expected_status_denial': {'apparmor': 'DENIED', 'operation': 'open', 'class': 'file', 'profile': 'r8-liquid-u3-proc-stat-boundary-bootstrap-v1-20260809t002729z', 'name_template': '/proc/<SUCCESS_FRAME_CHILD_PID>/status', 'name_must_match_success_frame_child_pid': True, 'comm': 'python3.12', 'requested_mask': 'r', 'denied_mask': 'r', 'fsuid': '1000', 'ouid': '0', 'allowed_field_names': ['apparmor', 'operation', 'class', 'profile', 'name', 'pid', 'comm', 'requested_mask', 'denied_mask', 'fsuid', 'ouid']}, 'journal_boundary_required': True, 'strict_utf8_required': True, 'duplicate_or_escaped_critical_field_is_pass': False, 'audit_overflow_is_pass': False, 'gate_no_go_is_pass': False, 'zero_logged_denials_required': False, 'zero_denied_operations_claimed': False, 'stderr_contract': {'returncode': 0, 'stdout': 'byte_exact_canonical_success_frame', 'stderr_utf8': ''}, 'any_other_failure': 'UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING', 'success_authorizes': 'only_independent_review_and_static_design_of_a_fresh_cold_a_successor_no_solver_gencase_or_production'}

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
    'mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,',
)

EXPECTED_PROFILE_LINES = (
    f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "userns create,",
    "/usr/bin/bwrap rix,",
    "/usr/bin/python3.12 rix,",
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
    "/proc/ r,",
    "owner /proc/** r,",
    "owner /proc/*/uid_map w,",
    "owner /proc/*/gid_map w,",
    "owner /proc/*/setgroups w,",
    "/proc/[0-9]*/stat r,",
    "/proc/filesystems r,",
    "/proc/sys/kernel/overflowuid r,",
    "/proc/sys/kernel/overflowgid r,",
    "/proc/sys/user/max_user_namespaces w,",
    "/proc/stat r,",
    *BASELINE_MOUNT_RULES,
    "/tmp/newroot/ rw,",
    "/tmp/newroot/** rw,",
    "/tmp/oldroot/ rw,",
    "/tmp/oldroot/** rw,",
    "/newroot/usr/ rw,",
    "/newroot/lib wl,",
    "/newroot/lib64 wl,",
    "/newroot/work/ rw,",
    "/newroot/proc/ rw,",
    "deny capability dac_override,",
    "capability sys_admin,",
    "capability sys_ptrace,",
    "capability sys_resource,",
    "capability setpcap,",
    "capability net_admin,",
    f"ptrace (read, readby) peer={BOOTSTRAP_PROFILE},",
    f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
    "signal (receive) set=(term,kill,exists) peer=unconfined,",
    "network unix dgram,",
    "network inet dgram,",
    "network inet6 dgram,",
    "network netlink raw,",
    "}",
    f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted) {{",
    "}",
)
EXPECTED_PROFILE_LINES_SHA256 = hashlib.sha256(
    ("\n".join(EXPECTED_PROFILE_LINES) + "\n").encode("utf-8")
).hexdigest()


HELPER_SOURCE = rf'''import ctypes
import errno
import hashlib
import json
import os
import signal
import stat
import struct
import sys
from collections import namedtuple

READ_EXACT = FRAME_READ_EXACT
STREAM = FRAME_STREAM
BOOTSTRAP_PROFILE = {BOOTSTRAP_PROFILE!r}
SUCCESS_MAGIC = {SUCCESS_MAGIC!r}
WORK_TMPFS_BYTES = {TMPFS_BYTES}
ProcessMember = namedtuple("ProcessMember", "pid state pgrp session starttime file_uid file_gid")

class ProbeError(RuntimeError):
    pass

def read_self_identity():
    fields = {{}}
    with open("/proc/self/status", "r", encoding="ascii") as source:
        for line in source:
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key] = value.strip()
    try:
        result = {{
            "uid": [int(value) for value in fields["Uid"].split()],
            "gid": [int(value) for value in fields["Gid"].split()],
            "groups": [int(value) for value in fields["Groups"].split()],
            "capabilities": {{
                key: int(fields[key], 16)
                for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")
            }},
            "no_new_privs": int(fields["NoNewPrivs"]),
        }}
    except (KeyError, ValueError) as exc:
        raise ProbeError("self_status_parse") from exc
    if result["uid"] != [0, 0, 0, 0] or result["gid"] != [0, 0, 0, 0]:
        raise ProbeError("guest_identity")
    if result["groups"] or any(result["capabilities"].values()):
        raise ProbeError("guest_capability_boundary")
    if result["no_new_privs"] != 1:
        raise ProbeError("guest_no_new_privs")
    return result

def read_label():
    with open("/proc/self/attr/current", "r", encoding="ascii") as source:
        observed = source.read().strip()
    expected = BOOTSTRAP_PROFILE + " (enforce)"
    if observed != expected:
        raise ProbeError("bootstrap_label")
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
    facts = os.statvfs("/work")
    total_bytes = facts.f_blocks * facts.f_frsize
    if total_bytes != WORK_TMPFS_BYTES:
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
                matches.append({{
                    "filesystem_type": filesystem_fields[0],
                    "mount_options": fields[5].split(","),
                }})
    if len(matches) != 1 or matches[0]["filesystem_type"] != "tmpfs":
        raise ProbeError("tmpfs_identity")
    if not {{"rw", "nosuid", "nodev"}}.issubset(set(matches[0]["mount_options"])):
        raise ProbeError("tmpfs_mount_options")
    return {{
        "filesystem_type": "tmpfs",
        "total_bytes": total_bytes,
        "mount_options": matches[0]["mount_options"],
    }}

def consume_payload_and_install_eof():
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

def write_all(descriptor, raw):
    view = memoryview(raw)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            raise ProbeError("short_pipe_write")
        view = view[count:]

def read_child_frame(descriptor):
    value = bytearray()
    while True:
        block = os.read(descriptor, 512)
        if not block:
            break
        value.extend(block)
        if len(value) > 4096:
            raise ProbeError("child_ready_frame_size")
    if not value.endswith(b"\n") or value.count(b"\n") != 1:
        raise ProbeError("child_ready_frame_boundary")
    try:
        observed = json.loads(value[:-1].decode("ascii"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ProbeError("child_ready_frame_json") from exc
    canonical = json.dumps(observed, sort_keys=True, separators=(",", ":")).encode("ascii") + b"\n"
    if canonical != bytes(value):
        raise ProbeError("child_ready_frame_canonical")
    return observed

def member_record(member):
    return {{
        "pid": member.pid,
        "state": member.state,
        "pgrp": member.pgrp,
        "session": member.session,
        "starttime": member.starttime,
        "stat_uid": member.file_uid,
        "stat_gid": member.file_gid,
    }}

def read_process_member(pid, skip_ungrouped_kernel_task=False):
    path = f"/proc/{{pid}}/stat"
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except FileNotFoundError:
        return None
    try:
        metadata = os.fstat(descriptor)
        raw = bytearray()
        while len(raw) <= 4096:
            block = os.read(descriptor, 4097 - len(raw))
            if not block:
                break
            raw.extend(block)
        if len(raw) > 4096 or os.read(descriptor, 1) != b"":
            raise ProbeError("process_stat_size")
    finally:
        os.close(descriptor)
    try:
        text = bytes(raw).decode("ascii")
    except UnicodeError as exc:
        raise ProbeError("process_stat_ascii") from exc
    if not text.endswith("\n"):
        raise ProbeError("process_stat_newline")
    close = text.rfind(")")
    if close < 2 or not text[:close].startswith(f"{{pid}} ("):
        raise ProbeError("process_stat_identity")
    fields = text[close + 2:].split()
    if len(fields) < 20 or len(fields[0]) != 1:
        raise ProbeError("process_stat_fields")
    try:
        pgrp = int(fields[2])
        session_id = int(fields[3])
        starttime = int(fields[19])
    except ValueError as exc:
        raise ProbeError("process_stat_numeric") from exc
    if pgrp < 0 or session_id < 0 or starttime < 1:
        raise ProbeError("process_stat_nonpositive")
    if pgrp == 0 and session_id == 0 and skip_ungrouped_kernel_task:
        return None
    if pgrp == 0 or session_id == 0:
        raise ProbeError("process_stat_zero_group")
    return ProcessMember(
        pid, fields[0], pgrp, session_id, starttime, metadata.st_uid, metadata.st_gid
    )

def scan_group_members_once(pgid):
    members = []
    with os.scandir("/proc") as entries:
        for entry in entries:
            if not entry.name.isascii() or not entry.name.isdecimal():
                continue
            member = read_process_member(
                int(entry.name), skip_ungrouped_kernel_task=True
            )
            if member is not None and member.pgrp == pgid:
                members.append(member)
    return sorted(members)

def group_members(pgid):
    first = scan_group_members_once(pgid)
    second = scan_group_members_once(pgid)
    if first != second:
        raise ProbeError("process_group_double_scan_unstable")
    return first

def capture_group(pid):
    if not hasattr(os, "pidfd_open") or not hasattr(os, "P_PIDFD"):
        raise ProbeError("pidfd_unavailable")
    pidfd = os.pidfd_open(pid, 0)
    try:
        first = read_process_member(pid)
        second = read_process_member(pid)
        if first is None or second is None:
            raise ProbeError("leader_disappeared_during_freeze")
        identity = (pid, first.pgrp, first.session, first.starttime)
        if identity != (pid, second.pgrp, second.session, second.starttime):
            raise ProbeError("leader_identity_changed_during_freeze")
        if (
            first.state != second.state
            or first.file_uid != second.file_uid
            or first.file_gid != second.file_gid
        ):
            raise ProbeError("leader_state_or_owner_changed_during_freeze")
        if (
            pid != first.pgrp
            or pid != first.session
            or os.getpgid(pid) != pid
            or os.getsid(pid) != pid
        ):
            raise ProbeError("leader_not_session_group_anchor")
        return pidfd, identity, first
    except BaseException:
        os.close(pidfd)
        raise

def require_pair(pid, expected_identity, expected_state):
    first = read_process_member(pid)
    second = read_process_member(pid)
    if first is None or second is None:
        raise ProbeError("stat_pair_missing")
    records = [first, second]
    for member in records:
        identity = (member.pid, member.pgrp, member.session, member.starttime)
        if identity != expected_identity:
            raise ProbeError("stat_pair_identity")
        if expected_state == "live" and member.state == "Z":
            raise ProbeError("live_stat_is_zombie")
        if expected_state == "zombie" and member.state != "Z":
            raise ProbeError("zombie_stat_state")
    if (
        first.state != second.state
        or first.file_uid != second.file_uid
        or first.file_gid != second.file_gid
    ):
        raise ProbeError("stat_pair_unstable")
    return [member_record(first), member_record(second)]

def preflight_group_scan(expected_identity):
    members = group_members(expected_identity[1])
    if not any(
        (member.pid, member.pgrp, member.session, member.starttime)
        == expected_identity
        for member in members
    ):
        raise ProbeError("leader_absent_from_group_scan")
    return [member_record(member) for member in members]

def read_overflow_owner():
    with open("/proc/sys/kernel/overflowuid", "r", encoding="ascii") as source:
        overflow_uid = int(source.read().strip())
    with open("/proc/sys/kernel/overflowgid", "r", encoding="ascii") as source:
        overflow_gid = int(source.read().strip())
    if overflow_uid < 1 or overflow_gid < 1:
        raise ProbeError("overflow_owner")
    return {{"uid": overflow_uid, "gid": overflow_gid}}

def require_status_eacces_once(pid):
    path = f"/proc/{{pid}}/status"
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    except PermissionError as exc:
        if type(exc.errno) is not int or exc.errno != errno.EACCES:
            raise ProbeError("status_wrong_errno") from exc
        return {{"attempt_count": 1, "path": path, "errno": errno.EACCES}}
    else:
        os.close(descriptor)
        raise ProbeError("status_unexpectedly_readable")

def child_main(ready_fd, release_fd):
    try:
        os.setsid()
        identity = read_self_identity()
        libc = ctypes.CDLL(None, use_errno=True)
        ctypes.set_errno(0)
        if libc.prctl(4, 0, 0, 0, 0) != 0:
            raise ProbeError("pr_set_dumpable")
        if libc.prctl(3, 0, 0, 0, 0) != 0:
            raise ProbeError("pr_get_dumpable")
        identity.update({{
            "pid": os.getpid(),
            "pgrp": os.getpgrp(),
            "session": os.getsid(0),
            "dumpable": 0,
        }})
        raw = json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("ascii") + b"\n"
        write_all(ready_fd, raw)
        os.close(ready_fd)
        released = os.read(release_fd, 2)
        os.close(release_fd)
        os._exit(0 if released == b"R" else 92)
    except BaseException:
        try:
            os.close(ready_fd)
        except OSError:
            pass
        try:
            os.close(release_fd)
        except OSError:
            pass
        os._exit(91)

def run_proc_stat_probe():
    ready_read, ready_write = os.pipe2(os.O_CLOEXEC)
    release_read, release_write = os.pipe2(os.O_CLOEXEC)
    pid = os.fork()
    if pid == 0:
        os.close(ready_read)
        os.close(release_write)
        child_main(ready_write, release_read)
    os.close(ready_write)
    os.close(release_read)
    pidfd = -1
    released = False
    reaped = False
    try:
        child_identity = read_child_frame(ready_read)
        os.close(ready_read)
        ready_read = -1
        expected_child_keys = {{
            "uid", "gid", "groups", "capabilities", "no_new_privs",
            "pid", "pgrp", "session", "dumpable",
        }}
        if set(child_identity) != expected_child_keys:
            raise ProbeError("child_identity_keys")
        if (
            child_identity["pid"] != pid
            or child_identity["pgrp"] != pid
            or child_identity["session"] != pid
            or child_identity["dumpable"] != 0
            or child_identity["uid"] != [0, 0, 0, 0]
            or child_identity["gid"] != [0, 0, 0, 0]
            or child_identity["groups"] != []
            or child_identity["no_new_privs"] != 1
            or any(child_identity["capabilities"].values())
        ):
            raise ProbeError("child_identity_boundary")
        pidfd, identity, frozen = capture_group(pid)
        live_group = preflight_group_scan(identity)
        live_stat_pair = require_pair(pid, identity, "live")
        overflow_owner = read_overflow_owner()
        if any(
            member["stat_uid"] != overflow_owner["uid"]
            or member["stat_gid"] != overflow_owner["gid"]
            for member in live_stat_pair
        ):
            raise ProbeError("live_stat_not_non_owner_root_projection")
        status_denial = require_status_eacces_once(pid)
        write_all(release_write, b"R")
        os.close(release_write)
        release_write = -1
        released = True
        observed = os.waitid(os.P_PIDFD, pidfd, os.WEXITED | os.WNOWAIT)
        if (
            observed is None
            or observed.si_pid != pid
            or observed.si_code != os.CLD_EXITED
            or observed.si_status != 0
        ):
            raise ProbeError("child_waitid_wnowait")
        zombie_stat_pair = require_pair(pid, identity, "zombie")
        if any(
            member["stat_uid"] != overflow_owner["uid"]
            or member["stat_gid"] != overflow_owner["gid"]
            for member in zombie_stat_pair
        ):
            raise ProbeError("zombie_stat_owner_changed")
        zombie_members = group_members(identity[1])
        if zombie_members != [
            ProcessMember(
                pid, "Z", identity[1], identity[2], identity[3],
                frozen.file_uid, frozen.file_gid,
            )
        ]:
            raise ProbeError("zombie_group_not_exact")
        reaped_result = os.waitid(os.P_PIDFD, pidfd, os.WEXITED)
        reaped = True
        if (
            reaped_result is None
            or reaped_result.si_pid != pid
            or reaped_result.si_code != os.CLD_EXITED
            or reaped_result.si_status != 0
        ):
            raise ProbeError("child_final_reap")
        final_scans = [
            [member_record(member) for member in scan_group_members_once(identity[1])],
            [member_record(member) for member in scan_group_members_once(identity[1])],
        ]
        if final_scans != [[], []]:
            raise ProbeError("post_reap_group_residue")
        return {{
            "child_identity": child_identity,
            "frozen_identity": {{
                "pid": identity[0],
                "pgrp": identity[1],
                "session": identity[2],
                "starttime": identity[3],
                "pidfd_opened": True,
            }},
            "live_group_scan": live_group,
            "live_stat_pair": live_stat_pair,
            "inner_proc_owner_projection": overflow_owner,
            "status_denial": status_denial,
            "waitid_wnowait": {{
                "pid": observed.si_pid,
                "code": observed.si_code,
                "status": observed.si_status,
            }},
            "zombie_stat_pair": zombie_stat_pair,
            "final_reap": {{
                "pid": reaped_result.si_pid,
                "code": reaped_result.si_code,
                "status": reaped_result.si_status,
            }},
            "post_reap_group_scans": final_scans,
        }}
    finally:
        if ready_read >= 0:
            try:
                os.close(ready_read)
            except OSError:
                pass
        if not reaped:
            if not released and release_write >= 0:
                try:
                    write_all(release_write, b"R")
                    released = True
                except (OSError, ProbeError):
                    pass
            if release_write >= 0:
                try:
                    os.close(release_write)
                except OSError:
                    pass
                release_write = -1
            try:
                waited, _status = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                waited = pid
            if waited == 0:
                try:
                    if pidfd >= 0:
                        signal.pidfd_send_signal(pidfd, signal.SIGKILL)
                    else:
                        os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    os.waitpid(pid, 0)
                except ChildProcessError:
                    pass
        elif release_write >= 0:
            try:
                os.close(release_write)
            except OSError:
                pass
        if pidfd >= 0:
            os.close(pidfd)

def emit_success(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if len(payload) > 8192:
        raise ProbeError("success_payload_size")
    frame = SUCCESS_MAGIC + struct.pack(">I", len(payload)) + payload + hashlib.sha256(payload).digest()
    write_all(1, frame)

def main():
    bootstrap_label = read_label()
    bootstrap_identity = read_self_identity()
    work_tmpfs = verify_work_tmpfs()
    consume_payload_and_install_eof()
    fds_after_payload = live_fds({{0, 1, 2}})
    probe = run_proc_stat_probe()
    fds_before_success = live_fds({{0, 1, 2}})
    emit_success({{
        "status": "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1",
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "bootstrap_label": bootstrap_label,
        "bootstrap_identity": bootstrap_identity,
        "work_tmpfs": work_tmpfs,
        "host_payload_consumed_and_fd0_replaced_with_eof_pipe": True,
        "fds_after_payload": fds_after_payload,
        "proc_stat_probe": probe,
        "fds_before_success": fds_before_success,
        "host_writable_mounts": [],
    }})
    return 0

try:
    raise SystemExit(main())
except (ProbeError, OSError, ValueError, UnicodeError):
    try:
        os.write(2, b"R8_PROC_STAT_PROBE_V1_NO_GO\n")
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
if r({len(PAYLOAD_MAGIC)})!={PAYLOAD_MAGIC!r}: raise SystemExit(92)
n=struct.unpack(">I",r(4))[0]
h=r(32)
if n!={HELPER_SIZE_BYTES} or h.hex()!="{HELPER_SHA256}": raise SystemExit(93)
p=r(n)
if hashlib.sha256(p).digest()!=h: raise SystemExit(94)
g={{"__name__":"__main__","FRAME_READ_EXACT":r,"FRAME_STREAM":B}}
exec(compile(p,"<r8-proc-stat-boundary-helper-v1>","exec",dont_inherit=True,optimize=2),g,g)
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
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise GateError(f"duplicate JSON key in {path}: {key}")
            value[key] = item
        return value

    try:
        value = json.loads(
            read_regular_bytes(path).decode("utf-8"),
            object_pairs_hook=reject_duplicates,
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def verify_v6_failure_provenance() -> dict[str, Any]:
    frozen = FROZEN_V6_FAILURE
    policy_record = frozen["policy"]
    policy_raw = read_regular_bytes(Path(policy_record["path"]))
    if (
        len(policy_raw) != policy_record["size_bytes"]
        or sha256_bytes(policy_raw) != policy_record["sha256"]
    ):
        raise GateError("frozen v6 policy bytes differ")
    try:
        v6_policy = json.loads(
            policy_raw.decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, "frozen v6 policy"),
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError("frozen v6 policy JSON differs") from exc
    attempt = v6_policy.get("frozen_attempt", {})
    if (
        attempt.get("case_id") != frozen["case_id"]
        or attempt.get("campaign_id") != frozen["campaign_id"]
        or attempt.get("campaign_role") != "cold_a"
        or attempt.get("cold_b_identity_allocated") is not False
        or v6_policy.get("invariants", {}).get("production_authorized") is not False
    ):
        raise GateError("frozen v6 policy failure identity differs")
    documents: dict[str, dict[str, Any]] = {}
    for name, record in frozen["receipts"].items():
        path = Path(record["path"])
        raw = read_regular_bytes(path)
        metadata = os.stat(path, follow_symlinks=False)
        if (
            len(raw) != record["size_bytes"]
            or sha256_bytes(raw) != record["sha256"]
            or metadata.st_uid != HOST_UID
            or metadata.st_gid != HOST_GID
            or stat.S_IMODE(metadata.st_mode) != 0o440
            or metadata.st_nlink != 1
        ):
            raise GateError(f"frozen v6 receipt bytes or inode differ: {name}")
        try:
            document = json.loads(
                raw.decode("utf-8"),
                object_pairs_hook=lambda pairs: _strict_pairs(pairs, f"frozen v6 receipt {name}"),
            )
        except (json.JSONDecodeError, UnicodeError) as exc:
            raise GateError(f"frozen v6 receipt JSON differs: {name}") from exc
        if (
            document.get("case_id") != frozen["case_id"]
            or document.get("campaign_id") != frozen["campaign_id"]
            or document.get("campaign_role") != "cold_a"
            or document.get("status") != record["status"]
            or document.get("policy_sha256") not in (None, policy_record["sha256"])
        ):
            raise GateError(f"frozen v6 receipt identity differs: {name}")
        documents[name] = document
    start_link = documents["execution"].get("start_receipt", {})
    execution_link = documents["lifecycle_incomplete"].get("execution_receipt", {})
    if (
        start_link.get("path") != frozen["receipts"]["start"]["path"]
        or start_link.get("sha256") != frozen["receipts"]["start"]["sha256"]
        or start_link.get("size_bytes") != frozen["receipts"]["start"]["size_bytes"]
        or execution_link.get("path") != frozen["receipts"]["execution"]["path"]
        or execution_link.get("sha256") != frozen["receipts"]["execution"]["sha256"]
        or execution_link.get("size_bytes") != frozen["receipts"]["execution"]["size_bytes"]
    ):
        raise GateError("frozen v6 receipt chain differs")
    execution = documents["execution"]
    audit = execution.get("audit", {})
    denials = audit.get("unexpected_denials")
    if (
        audit.get("capture_valid") is not True
        or audit.get("capture_errors") != []
        or audit.get("matching_total") != 2
        or audit.get("stored_count") != 2
        or audit.get("unexpected_total") != 2
        or audit.get("storage_overflow") is not False
        or not isinstance(denials, list)
        or len(denials) != 2
        or audit.get("sanitized_denials") != denials
    ):
        raise GateError("frozen v6 stat denial accounting differs")
    expected = frozen["stat_denials"]
    expected_fields = {
        "apparmor": "DENIED",
        "operation": expected["operation"],
        "class": expected["class"],
        "profile": "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v6-20260808t212044z",
        "name": expected["name"],
        "comm": "python3.12",
        "requested_mask": expected["requested_mask"],
        "denied_mask": expected["denied_mask"],
        "fsuid": expected["fsuid"],
        "ouid": expected["ouid"],
    }
    for index, denial in enumerate(denials):
        fields = denial.get("fields")
        if not isinstance(fields, list):
            raise GateError("frozen v6 denial fields differ")
        if (
            len(fields) != 11
            or any(
                not isinstance(field, dict)
                or set(field) != {"key", "raw_value", "value"}
                or not isinstance(field["raw_value"], str)
                for field in fields
            )
        ):
            raise GateError("frozen v6 denial field records differ")
        keys = [field["key"] for field in fields]
        if any(not isinstance(key, str) for key in keys) or len(set(keys)) != 11:
            raise GateError("frozen v6 denial field keys are not unique")
        values = {field["key"]: field["value"] for field in fields}
        audit_pid = values.pop("pid", None)
        if (
            values != expected_fields
            or not isinstance(audit_pid, str)
            or not audit_pid.isdecimal()
            or int(audit_pid) < 1
            or denial.get("line_sha256") != expected["line_sha256"][index]
            or denial.get("line_size_bytes") != expected["line_size_bytes"][index]
            or denial.get("parse_status") != "PARSED_UNIQUE_KV"
            or denial.get("parse_error") is not None
            or denial.get("line_truncated") is not False
        ):
            raise GateError("frozen v6 denial semantics differ")
    incomplete = documents["lifecycle_incomplete"]
    cleanup = incomplete.get("cleanup_observation", {})
    if (
        incomplete.get("primary_error")
        != "SupervisorError: AppArmor journal window is not closed with zero unexpected logged denials"
        or incomplete.get("cleanup_errors") != []
        or incomplete.get("next_allowed_stage") != "STOP_COLD_A_FAILED_NO_COLD_B_ADMISSION"
        or any(incomplete.get(key) is not False for key in (
            "production_authorized", "settled_state_authorized", "cold_b_admission_authorized"
        ))
        or cleanup.get("initial") != []
        or cleanup.get("after_term") != []
        or cleanup.get("stable_zero_scans") != [[], [], []]
        or cleanup.get("post_unload_stable_zero_scans") != [[], [], []]
    ):
        raise GateError("frozen v6 failure or cleanup closure differs")
    return {
        "policy": policy_record,
        "receipts": frozen["receipts"],
        "stat_denials": expected,
        "identity_consumed": True,
        "retry_forbidden": True,
        "snapshot_read": False,
        "output_reused": False,
    }


def _strict_pairs(pairs: list[tuple[str, Any]], context: str) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise GateError(f"duplicate JSON key in {context}: {key}")
        value[key] = item
    return value


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


def build_payload_frame() -> bytes:
    frame = b"".join((
        PAYLOAD_MAGIC,
        struct.pack(">I", HELPER_SIZE_BYTES),
        bytes.fromhex(HELPER_SHA256),
        HELPER_BYTES,
    ))
    if len(frame) > MAX_PAYLOAD_FRAME_BYTES:
        raise GateError("payload frame exceeds its fixed ceiling")
    return frame


def parse_payload_frame(frame: bytes) -> dict[str, Any]:
    if len(frame) > MAX_PAYLOAD_FRAME_BYTES:
        raise GateError("payload frame exceeds ceiling")
    view = memoryview(frame)
    position = 0

    def take(size: int) -> bytes:
        nonlocal position
        if size < 0 or position + size > len(view):
            raise GateError("payload frame truncation")
        result = bytes(view[position:position + size])
        position += size
        return result

    if take(len(PAYLOAD_MAGIC)) != PAYLOAD_MAGIC:
        raise GateError("payload frame magic differs")
    helper_size = struct.unpack(">I", take(4))[0]
    helper_hash = take(32).hex()
    helper = take(helper_size)
    if helper_size != HELPER_SIZE_BYTES or helper_hash != HELPER_SHA256 or sha256_bytes(helper) != helper_hash:
        raise GateError("payload helper identity differs")
    if position != len(view):
        raise GateError("payload frame has trailing bytes")
    return {
        "helper": {"size_bytes": helper_size, "sha256": helper_hash},
        "size_bytes": len(frame),
        "sha256": sha256_bytes(frame),
    }


def parse_success_frame(frame: bytes) -> dict[str, Any]:
    prefix_size = len(SUCCESS_MAGIC)
    payload_offset = prefix_size + 4
    if not payload_offset + 2 + 32 <= len(frame) <= MAX_SUCCESS_FRAME_BYTES or not frame.startswith(SUCCESS_MAGIC):
        raise GateError("success frame size or magic differs")
    size = struct.unpack(">I", frame[prefix_size:payload_offset])[0]
    if size < 2 or payload_offset + size + 32 != len(frame):
        raise GateError("success frame declared length differs")
    payload = frame[payload_offset:payload_offset + size]
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
        "work_tmpfs", "host_payload_consumed_and_fd0_replaced_with_eof_pipe",
        "fds_after_payload", "proc_stat_probe", "fds_before_success",
        "host_writable_mounts",
    }
    if set(value) != expected_keys:
        raise GateError("success frame key set differs")
    if value["status"] != "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1":
        raise GateError("success status differs")
    if value["bootstrap_profile"] != BOOTSTRAP_PROFILE or value["bootstrap_label"] != BOOTSTRAP_PROFILE + " (enforce)":
        raise GateError("bootstrap label evidence differs")
    if value["host_writable_mounts"] != [] or value["host_payload_consumed_and_fd0_replaced_with_eof_pipe"] is not True:
        raise GateError("success isolation evidence differs")
    if (type(value["fds_after_payload"]) is not list or type(value["fds_before_success"]) is not list
            or any(type(descriptor) is not int for descriptor in value["fds_after_payload"] + value["fds_before_success"])
            or value["fds_after_payload"] != [0, 1, 2] or value["fds_before_success"] != [0, 1, 2]):
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
    work_tmpfs = value["work_tmpfs"]
    if not isinstance(work_tmpfs, dict) or set(work_tmpfs) != {"filesystem_type", "total_bytes", "mount_options"}:
        raise GateError("success tmpfs key set differs")
    options = work_tmpfs["mount_options"]
    if (work_tmpfs["filesystem_type"] != "tmpfs" or work_tmpfs["total_bytes"] != TMPFS_BYTES
            or isinstance(work_tmpfs["total_bytes"], bool) or not isinstance(options, list)
            or any(not isinstance(option, str) for option in options) or len(options) != len(set(options))
            or not {"rw", "nosuid", "nodev"}.issubset(set(options))):
        raise GateError("success tmpfs evidence differs")
    probe = value["proc_stat_probe"]
    expected_probe_keys = {
        "child_identity", "frozen_identity", "live_group_scan",
        "live_stat_pair", "inner_proc_owner_projection", "status_denial",
        "waitid_wnowait", "zombie_stat_pair", "final_reap",
        "post_reap_group_scans",
    }
    if not isinstance(probe, dict) or set(probe) != expected_probe_keys:
        raise GateError("proc-stat probe evidence key set differs")
    child = probe["child_identity"]
    child_extra = {"pid", "pgrp", "session", "dumpable"}
    if not isinstance(child, dict) or set(child) != {
        "uid", "gid", "groups", "capabilities", "no_new_privs", *child_extra
    }:
        raise GateError("dumpable-zero child identity key set differs")
    require_guest_identity(
        {key: child[key] for key in ("uid", "gid", "groups", "capabilities", "no_new_privs")},
        "dumpable-zero child",
    )
    pid = child["pid"]
    if (
        type(pid) is not int
        or pid < 2
        or child["pgrp"] != pid
        or child["session"] != pid
        or type(child["pgrp"]) is not int
        or type(child["session"]) is not int
        or type(child["dumpable"]) is not int
        or child["dumpable"] != 0
    ):
        raise GateError("dumpable-zero child process identity differs")
    frozen = probe["frozen_identity"]
    if (
        not isinstance(frozen, dict)
        or set(frozen) != {"pid", "pgrp", "session", "starttime", "pidfd_opened"}
        or frozen["pid"] != pid
        or frozen["pgrp"] != pid
        or frozen["session"] != pid
        or type(frozen["starttime"]) is not int
        or frozen["starttime"] < 1
        or frozen["pidfd_opened"] is not True
    ):
        raise GateError("frozen child birth identity differs")
    owner = probe["inner_proc_owner_projection"]
    if (
        not isinstance(owner, dict)
        or set(owner) != {"uid", "gid"}
        or type(owner["uid"]) is not int
        or type(owner["gid"]) is not int
        or owner["uid"] < 1
        or owner["gid"] < 1
    ):
        raise GateError("inner proc owner projection differs")

    member_keys = {"pid", "state", "pgrp", "session", "starttime", "stat_uid", "stat_gid"}

    def require_member(member: Any, *, zombie: bool) -> None:
        if not isinstance(member, dict) or set(member) != member_keys:
            raise GateError("proc-stat member key set differs")
        if (
            member["pid"] != pid
            or member["pgrp"] != pid
            or member["session"] != pid
            or member["starttime"] != frozen["starttime"]
            or member["stat_uid"] != owner["uid"]
            or member["stat_gid"] != owner["gid"]
            or type(member["state"]) is not str
            or len(member["state"]) != 1
            or (member["state"] == "Z") is not zombie
        ):
            raise GateError("proc-stat member identity, owner, or state differs")

    live_pair = probe["live_stat_pair"]
    zombie_pair = probe["zombie_stat_pair"]
    if not isinstance(live_pair, list) or len(live_pair) != 2:
        raise GateError("live stat pair count differs")
    if not isinstance(zombie_pair, list) or len(zombie_pair) != 2:
        raise GateError("zombie stat pair count differs")
    for member in live_pair:
        require_member(member, zombie=False)
    for member in zombie_pair:
        require_member(member, zombie=True)
    if live_pair[0] != live_pair[1] or zombie_pair[0] != zombie_pair[1]:
        raise GateError("live or zombie stat double-read is unstable")
    group = probe["live_group_scan"]
    if not isinstance(group, list) or len(group) != 1:
        raise GateError("live process group must contain exactly the isolated child")
    require_member(group[0], zombie=False)
    if group[0] != live_pair[0]:
        raise GateError("full group scan and live stat reads differ")
    denial = probe["status_denial"]
    if (
        not isinstance(denial, dict)
        or set(denial) != {"attempt_count", "path", "errno"}
        or type(denial["attempt_count"]) is not int
        or denial["attempt_count"] != 1
        or denial["path"] != f"/proc/{pid}/status"
        or type(denial["errno"]) is not int
        or denial["errno"] != errno.EACCES
    ):
        raise GateError("single child status EACCES evidence differs")
    expected_wait = {"pid": pid, "code": os.CLD_EXITED, "status": 0}
    for name in ("waitid_wnowait", "final_reap"):
        wait = probe[name]
        if (
            not isinstance(wait, dict)
            or set(wait) != {"pid", "code", "status"}
            or any(type(wait[key]) is not int for key in ("pid", "code", "status"))
            or wait != expected_wait
        ):
            raise GateError("WNOWAIT or final reap evidence differs")
    if probe["post_reap_group_scans"] != [[], []]:
        raise GateError("post-reap process group is not double-empty")
    return value


def sudo_timestamp_argvs() -> tuple[list[str], list[str]]:
    prefix = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27",
        "--inh-caps=-all", "--ambient-caps=-all", "--",
    ]
    argvs = (
        prefix + ["/usr/bin/sudo", "-K"],
        prefix + ["/usr/bin/sudo", "-n", "/usr/bin/true"],
    )
    validate_sudo_cleanup_argv_contract(argvs[0], ["/usr/bin/sudo", "-K"])
    validate_sudo_cleanup_argv_contract(argvs[1], ["/usr/bin/sudo", "-n", "/usr/bin/true"])
    return argvs


def validate_sudo_cleanup_argv_contract(argv: list[str], command_tail: list[str]) -> None:
    expected_prefix = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--groups=27",
        "--inh-caps=-all", "--ambient-caps=-all", "--",
    ]
    if type(argv) is not list or any(type(token) is not str for token in argv):
        raise GateError("sudo cleanup argv type differs")
    if command_tail not in (["/usr/bin/sudo", "-K"], ["/usr/bin/sudo", "-n", "/usr/bin/true"]):
        raise GateError("sudo cleanup inner command differs")
    if argv != expected_prefix + command_tail:
        raise GateError("sudo cleanup argv differs or contains injected arguments")
    if any(token == "--bounding-set" or token.startswith("--bounding-set=") for token in argv):
        raise GateError("sudo cleanup must preserve the host bounding set without a bounding argv token")
    if argv.count("--") != 1 or any(token in argv for token in ("--clear-groups", "--init-groups", "--keep-groups", "--no-new-privs")):
        raise GateError("sudo cleanup group, delimiter, or NNP argv contract differs")


def bwrap_argv() -> list[str]:
    return [
        "/usr/bin/timeout", "--foreground", "--signal=TERM", "--kill-after=2s", "15s",
        "/usr/bin/aa-exec", "-p", BOOTSTRAP_PROFILE, "--",
        "/usr/bin/bwrap", "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--uid", "0", "--gid", "0", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-proc-stat-v1", "--clearenv",
        "--setenv", "HOME", "/nonexistent", "--setenv", "PATH", "/usr/bin",
        "--setenv", "LC_ALL", "C.UTF-8", "--setenv", "LANG", "C.UTF-8", "--setenv", "TZ", "UTC0",
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--size", str(TMPFS_BYTES), "--tmpfs", "/work", "--proc", "/proc", "--chdir", "/work", "--",
        "/usr/bin/python3.12", "-I", "-B", "-S", "-c", LOADER_SOURCE,
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
    for raw_line in text.splitlines():
        stripped = raw_line.lstrip()
        if re.match(r"#\s*include\b", stripped) or re.match(r"include\b", stripped):
            raise GateError("AppArmor include directives are forbidden")
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
    candidate_rule = "/proc/[0-9]*/stat r,"
    if lines.count(candidate_rule) != 1:
        raise GateError("exact numeric proc-stat candidate rule count differs")
    if lines.count("deny capability dac_override,") != 1:
        raise GateError("exact evidence-derived silent dac_override deny differs")
    mount_lines = tuple(line for line in lines if line.startswith(("mount ", "remount ", "pivot_root ", "umount ")))
    if mount_lines != BASELINE_MOUNT_RULES:
        raise GateError("provenance-pinned mount baseline differs")
    if lines.count("/newroot/proc/ rw,") != 1:
        raise GateError("exact bootstrap /newroot/proc directory permission differs")
    if lines.count(PROC_MOUNT_RULE) != 1:
        raise GateError("exact evidence-derived proc mount rule differs")
    forbidden = (
        "/newroot/proc/**", "/newroot/proc/*", "/dev/", "/dev ",
        "GenCase", "DualSPHysics", "/home/zrj/scout_ws", "/opt/ros",
        "flags=(unconfined)", " mount,", "/proc/*/stat r,",
        "/proc/**/stat", "/proc/[0-9]*/**", "/proc/[0-9]*/status",
        "/proc/[0-9]*/{stat,status}", "/**/stat", "@{PROC}",
    )
    effective = "\n".join(lines)
    if any(token in effective for token in forbidden):
        raise GateError("profile contains forbidden or guessed authority")
    bootstrap, runtime = effective.split(f"profile {RUNTIME_PROFILE}", 1)
    if candidate_rule not in bootstrap or candidate_rule in runtime:
        raise GateError("numeric proc-stat rule is not bootstrap-only")
    for token in ("userns ", "mount ", "capability ", "network ", "/proc", "/dev", "/work", "/usr/bin/python"):
        if token in runtime:
            raise GateError(f"runtime profile contains forbidden authority: {token}")
    return {
        "path": str(PROFILE_PATH),
        "sha256": sha256_bytes(raw),
        "effective_lines_sha256": lines_sha256,
        "effective_line_count": len(lines),
        "mount_rules": list(mount_lines),
        "candidate_rule": candidate_rule,
        "candidate_rule_count": 1,
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


def artifact_policy_paths() -> dict[str, Path]:
    return {
        "gate": WORKSPACE_ROOT / "scripts" / SCRIPT_PATH.name,
        "supervisor": WORKSPACE_SUPERVISOR_PATH,
        "profile": WORKSPACE_ROOT / "config/apparmor_drafts" / PROFILE_NAME,
        "schema": WORKSPACE_ROOT / "schema" / SCHEMA_NAME,
    }


def verify_static() -> dict[str, Any]:
    policy = read_json(POLICY_PATH)
    schema = read_json(SCHEMA_PATH)
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("closed top-level policy/schema contract differs")

    identities = {
        "schema_version": "smpcc-r8-liquid-target-u3-proc-stat-apparmor-boundary-probe-policy-v1",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
    }
    if any(policy.get(key) != value for key, value in identities.items()):
        raise GateError("fresh proc-stat policy identity differs")
    draft_status = "STATIC_DRAFT_V1_PENDING_REVIEWED_BYTE_HASH_FREEZE_AND_INDEPENDENT_REVIEW"
    ready_status = "STATIC_READY_V1_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT"
    draft_next = "FREEZE_REVIEWED_BYTES_THEN_INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
    ready_next = "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
    if (policy.get("status"), policy.get("next_allowed_stage")) not in {
        (draft_status, draft_next), (ready_status, ready_next),
    }:
        raise GateError("fresh policy review state differs")
    frozen_ready = policy["status"] == ready_status

    authorization = policy.get("authorization", {})
    if (
        authorization.get("static_review_authorized") is not True
        or authorization.get("static_task_execution_performed") is not False
        or authorization.get("execution_authorized_now") is not False
        or authorization.get("attempts_per_identity") != 1
        or authorization.get("same_identity_retry") != "forbidden"
        or authorization.get("admission_token") != ADMISSION_TOKEN
        or authorization.get("public_token_is_authority") is not False
        or authorization.get("root_owned_fd_capability_required") is not True
        or authorization.get("solver_or_gencase_authorized") is not False
        or authorization.get("cold_a_successor_execution_authorized") is not False
        or authorization.get("production_authorized") is not False
    ):
        raise GateError("static-only one-shot authorization differs")
    expected_frozen = {
        "probe_id": PROBE_ID,
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        **FROZEN_RECEIPT_PATHS,
    }
    if policy.get("frozen_identity") != expected_frozen:
        raise GateError("frozen probe identity, labels, or receipt paths differ")
    if policy.get("provenance") != {"cold_a_v6_failure": FROZEN_V6_FAILURE}:
        raise GateError("frozen v6 failure provenance policy differs")
    v6 = verify_v6_failure_provenance()

    if policy.get("trusted_system_tools", {}).get("unprivileged_runner") != trusted_tool_policy():
        raise GateError("unprivileged trusted-tool contract differs")
    root_python = policy.get("trusted_system_tools", {}).get("root_supervisor", {}).get("python")
    if root_python != trusted_tool_policy()["python"]:
        raise GateError("root Python entrypoint identity differs")

    frame = build_payload_frame()
    parsed_frame = parse_payload_frame(frame)
    reviewed = policy.get("reviewed_bytes", {})
    expected_reviewed = {
        "all_frozen": True,
        "helper": {"size_bytes": HELPER_SIZE_BYTES, "sha256": HELPER_SHA256},
        "loader": {"size_bytes": LOADER_SIZE_BYTES, "sha256": LOADER_SHA256},
        "payload_frame": {
            "size_bytes": len(frame),
            "sha256": sha256_bytes(frame),
            "maximum_size_bytes": MAX_PAYLOAD_FRAME_BYTES,
        },
    }
    if frozen_ready:
        if reviewed != expected_reviewed:
            raise GateError("reviewed payload bytes differ")
    else:
        if (
            reviewed.get("all_frozen") is not False
            or reviewed.get("helper") != {"size_bytes": 0, "sha256": "0" * 64}
            or reviewed.get("loader") != {"size_bytes": 0, "sha256": "0" * 64}
            or reviewed.get("payload_frame") != {
                "size_bytes": 0, "sha256": "0" * 64,
                "maximum_size_bytes": MAX_PAYLOAD_FRAME_BYTES,
            }
        ):
            raise GateError("draft reviewed-byte placeholders differ")

    payload = policy.get("payload_conduit", {})
    if (
        payload.get("frame_magic_hex") != PAYLOAD_MAGIC.hex()
        or payload.get("success_magic_hex") != SUCCESS_MAGIC.hex()
        or payload.get("success_status") != "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1"
        or payload.get("external_payload_count") != 0
        or payload.get("host_writable_transport") is not False
        or payload.get("maximum_payload_frame_bytes") != MAX_PAYLOAD_FRAME_BYTES
        or payload.get("maximum_success_frame_bytes") != MAX_SUCCESS_FRAME_BYTES
        or payload.get("maximum_child_stderr_bytes") != MAX_CHILD_STDERR_BYTES
        or payload.get("outer_deadline_seconds") != OUTER_DEADLINE_SECONDS
        or payload.get("isolation", {}).get("host_read_only_mounts") != ["/usr"]
        or payload.get("isolation", {}).get("host_writable_mounts") != []
        or payload.get("isolation", {}).get("namespaces") != ["user", "pid", "network", "ipc", "uts"]
        or payload.get("isolation", {}).get("guest_dev") != "absent"
        or payload.get("isolation", {}).get("solver_gencase_ros_gazebo_gpu") != "not_executed_not_exposed"
    ):
        raise GateError("payload conduit or isolation policy differs")

    verify_argv_contract(bwrap_argv())
    commands = policy.get("fixed_commands", {})
    bwrap_hash = sha256_bytes(
        json.dumps(bwrap_argv(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    )
    if frozen_ready:
        if commands.get("argv_hashes_frozen") is not True or commands.get("bwrap_argv_sha256") != bwrap_hash:
            raise GateError("frozen bwrap argv identity differs")
    elif commands.get("argv_hashes_frozen") is not False or commands.get("bwrap_argv_sha256") != "0" * 64:
        raise GateError("draft bwrap argv placeholder differs")
    if commands.get("aa_status_json_argv") != list(AA_STATUS_JSON_ARGV):
        raise GateError("fixed aa-status argv differs")
    expected_handoff = [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups",
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all",
        "--no-new-privs", "--", "/usr/bin/python3.12", "-I", "-B",
        str(SNAPSHOT_ROOT / SCRIPT_PATH.name), "internal-run",
    ]
    if commands.get("gate_handoff_argv") != expected_handoff:
        raise GateError("fixed gate handoff argv differs")
    profile_path = str(SNAPSHOT_ROOT / PROFILE_NAME)
    parser_contract = {
        "parser_parse_argv": ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path],
        "parser_load_argv": ["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path],
        "parser_unload_argv": ["/usr/sbin/apparmor_parser", "-R", "-K", profile_path],
    }
    if any(commands.get(key) != value for key, value in parser_contract.items()):
        raise GateError("fixed parser argv differs")
    journal_commands = {
        "journal_boot_id_path": "/proc/sys/kernel/random/boot_id",
        "journal_anchor_argv_template": ["/usr/bin/journalctl", "-k", "--no-pager", "--quiet", "--boot=<BOOT_ID_32HEX>", "--lines=0", "--show-cursor"],
        "journal_sync_argv": ["/usr/bin/journalctl", "--sync"],
        "journal_query_argv_template": ["/usr/bin/journalctl", "-k", "--no-pager", "--quiet", "--output=short-iso-precise", "--boot=<BOOT_ID_32HEX>", "--after-cursor=<PRE_RUN_CURSOR>", "--show-cursor"],
        "journal_anchor_stdout_ceiling_bytes": 4096,
        "journal_query_stdout_ceiling_bytes": 131072,
        "journal_stderr_ceiling_bytes": 16384,
        "audit_line_ceiling_bytes": 4096,
        "audit_storage_ceiling": 16,
    }
    if any(commands.get(key) != value for key, value in journal_commands.items()):
        raise GateError("fixed journal evidence argv differs")
    clear_argv, verify_argv = sudo_timestamp_argvs()
    if (
        commands.get("sudo_timestamp_clear_argv") != clear_argv
        or commands.get("sudo_timestamp_verify_argv") != verify_argv
        or commands.get("shell") is not False
        or commands.get("workspace_run_forbidden") is not True
    ):
        raise GateError("fixed sudo or shell boundary differs")
    snapshot_supervisor = str(SNAPSHOT_ROOT / SUPERVISOR_NAME)
    if commands.get("run_argv_template") != [
        "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", snapshot_supervisor,
        "run", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
        "--admission-token", ADMISSION_TOKEN,
    ]:
        raise GateError("fixed run argv differs")
    if commands.get("recover_argv_template") != [
        "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", snapshot_supervisor,
        "recover-only", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
        "--recovery-token", f"{PROBE_ID}:cleanup-only-no-probe",
    ]:
        raise GateError("fixed recovery argv differs")
    operator = commands.get("operator_boundary", {})
    if any(operator.get(key) is not True for key in (
        "run_and_recover_foreground_pty_required",
        "inherited_dev_tty_password_input_only", "sudo_dash_S_forbidden",
        "password_environment_variable_forbidden", "fd0_redirection_forbidden",
        "background_or_detached_invocation_forbidden", "shell_interpolation_forbidden",
    )):
        raise GateError("operator foreground PTY boundary differs")

    profile = verify_profile()
    semantics = policy.get("profile_semantics", {})
    if (
        semantics.get("effective_line_count") != len(EXPECTED_PROFILE_LINES)
        or semantics.get("effective_lines_sha256") != EXPECTED_PROFILE_LINES_SHA256
        or semantics.get("effective_lines_sha256") != profile["effective_lines_sha256"]
        or semantics.get("external_includes") != []
        or semantics.get("named_only") is not True
        or semantics.get("persistent_install") is not False
        or semantics.get("runtime_profile_loaded_but_unreachable") is not True
        or semantics.get("runtime_profile_effective_rules") != []
    ):
        raise GateError("profile semantics differ")
    candidate = policy.get("candidate_boundary", {})
    if (
        candidate.get("candidate_rule") != "/proc/[0-9]*/stat r,"
        or candidate.get("candidate_rule_count") != 1
        or candidate.get("candidate_rule_profile") != "bootstrap_only"
        or candidate.get("status_rule_present") is not False
        or candidate.get("child_creation") != "os_fork_single_child_no_exec"
        or candidate.get("child_count") != 1
        or candidate.get("live_phase", {}).get("direct_stat_reads") != 2
        or candidate.get("status_boundary_control", {}).get("open_attempts") != 1
        or candidate.get("status_boundary_control", {}).get("required_errno") != errno.EACCES
        or candidate.get("zombie_phase", {}).get("direct_stat_reads") != 2
        or candidate.get("reap_phase", {}).get("post_reap_group_scans") != 2
        or candidate.get("host_file_writes") != 0
        or candidate.get("solver_or_gencase_execution") is not False
    ):
        raise GateError("candidate state-machine policy differs")
    journal = policy.get("journal_contract", {})
    if (
        journal.get("expected_status_denials") != 1
        or journal.get("unexpected_denials") != 0
        or journal.get("storage_overflow_allowed") is not False
        or journal.get("matching_labels") != [BOOTSTRAP_PROFILE, RUNTIME_PROFILE]
        or journal.get("same_boot_id_required") is not True
    ):
        raise GateError("journal status-denial policy differs")
    if policy.get("evidence_classification") != EVIDENCE_CLASSIFICATION_CONTRACT:
        raise GateError("status-denial evidence classification policy differs")
    lifecycle = policy.get("lifecycle_contract", {})
    if (
        lifecycle.get("stable_zero_label_scans") != 3
        or lifecycle.get("unload_requires_zero_labels") is not True
        or lifecycle.get("guest_child_cleanup")
        != "pidfd anchored release or SIGKILL then final reap without reopening child status"
        or lifecycle.get("recovery", {}).get("token") != f"{PROBE_ID}:cleanup-only-no-probe"
        or lifecycle.get("recovery", {}).get("probe_execution") is not False
    ):
        raise GateError("lifecycle policy differs")
    receipt = policy.get("receipt_contract", {})
    if (
        receipt.get("sudo_group_membership") != SUDO_GROUP_MEMBERSHIP_CONTRACT
        or receipt.get("sudo_cleanup_identity") != SUDO_CLEANUP_IDENTITY_CONTRACT
        or receipt.get("sudo_cleanup_bounding_mode") != SUDO_CLEANUP_BOUNDING_MODE
    ):
        raise GateError("sudo cleanup policy differs")

    bootstrap = policy.get("snapshot_bootstrap", {})
    if (
        bootstrap.get("all_frozen") is not frozen_ready
        or bootstrap.get("file_mode") != "0444"
        or bootstrap.get("runtime_supervisor_entrypoint")
        != "/usr/bin/python3.12 -I -B reads root-owned 0444 snapshot supervisor"
        or bootstrap.get("direct_snapshot_supervisor_exec_forbidden") is not True
        or bootstrap.get("root_loader_c_source_contract")
        != "reviewed_hash_pinned_literal_single_argv_token_after_-c"
        or bootstrap.get("producer_stdout_use")
        != "stdin_of_the_reviewed_hash_pinned_root_loader_only"
        or bootstrap.get("workspace_snapshot_producer_root_forbidden") is not True
        or bootstrap.get("workspace_snapshot_producer_exact_path") != str(WORKSPACE_SUPERVISOR_PATH)
        or bootstrap.get("workspace_snapshot_producer_uid") != [HOST_UID] * 4
        or bootstrap.get("workspace_snapshot_producer_gid") != [HOST_GID] * 4
    ):
        raise GateError("snapshot bootstrap boundary differs")
    for key in ("source_sha256", "loader_sha256"):
        digest = bootstrap.get(key)
        if frozen_ready:
            if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None or digest == "0" * 64:
                raise GateError("snapshot bootstrap digest is not frozen")
        elif digest != "0" * 64:
            raise GateError("draft snapshot bootstrap digest differs")

    artifacts = policy.get("trusted_artifacts", {})
    if set(artifacts) != {"all_frozen", *artifact_paths()}:
        raise GateError("trusted artifact set differs")
    if artifacts.get("all_frozen") is not frozen_ready:
        raise GateError("trusted artifact aggregate freeze state differs")
    observed_artifacts: dict[str, dict[str, Any]] = {}
    policy_paths = artifact_policy_paths()
    for name, path in artifact_paths().items():
        raw = read_regular_bytes(path)
        entry = artifacts[name]
        bytes_match = entry.get("sha256") == sha256_bytes(raw) and entry.get("size_bytes") == len(raw)
        if entry.get("path") != str(policy_paths[name]) or bytes_match is not frozen_ready:
            raise GateError(f"trusted artifact freeze state differs: {name}")
        observed_artifacts[name] = {
            "path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw),
        }

    return {
        "policy_sha256": sha256_bytes(read_regular_bytes(POLICY_PATH)),
        "artifacts": observed_artifacts,
        "profile": profile,
        "payload_frame": parsed_frame,
        "helper_sha256": HELPER_SHA256,
        "loader_sha256": LOADER_SHA256,
        "v6_failure": v6,
        "freeze_ready": frozen_ready,
        "execution_performed": False,
    }


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
    if len(input_frame) > MAX_PAYLOAD_FRAME_BYTES:
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
    if review.get("freeze_ready") is not True:
        raise GateError("internal-run requires the fully frozen STATIC_READY policy")
    for name in TRUSTED_TOOLS:
        require_tool(name)
    frame = build_payload_frame()
    if parse_payload_frame(frame)["sha256"] != review["payload_frame"]["sha256"]:
        raise GateError("runtime payload frame differs from static review")
    returncode, stdout, stderr, consumed = run_bounded_guest(bwrap_argv(), frame)
    if returncode == 0:
        if consumed != len(frame):
            raise GateError("successful guest did not consume the complete fixed payload frame")
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
            state = "STATIC_READY" if review["freeze_ready"] else "STATIC_DRAFT"
            print(json.dumps({"status": f"PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1_{state}_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
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
