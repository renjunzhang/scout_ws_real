#!/usr/bin/env python3
"""Root lifecycle supervisor for one fresh proc-stat boundary probe.

The workspace copy supports read-only ``self-check`` only.  ``run`` refuses
unless this exact script and all peer artifacts are already in the fixed
root-owned snapshot.  Snapshot creation is performed by a separately reviewed
minimal Python ``-I -B -c`` bootstrap that copies bytes but never imports or
executes workspace code.
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
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
WORKSPACE_PACKAGE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v2.py"
PROBE_ID = "u3_proc_stat_apparmor_boundary_probe_v2_20260809T014926Z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{PROBE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V2_SNAPSHOT"
ADMISSION_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V2_ADMISSION"
ADMISSION_FD_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V2_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_PROC_STAT_BOUNDARY_PROBE_V2_ADMISSION_SHA256"
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
        "start": {"path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.start.json", "sha256": "07d1c2c3e212f8cd791531a59933695e7a078c7953808c842d28a19bd21f365f", "size_bytes": 11_634, "status": "V6_COLD_A_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY"},
        "execution": {"path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.execution.json", "sha256": "7cba88d4793a58e05d2b7a0c6d843a8d6f0fec2e19541a9ff138a814ab1d4316", "size_bytes": 13_787, "status": "V6_COLD_A_ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY"},
        "lifecycle_incomplete": {"path": "/home/zrj/scout_liquid_lab/audits/u3_c1m_solver_cpu_settle_cold_a_v6_20260808T212044Z.lifecycle_incomplete.json", "sha256": "03958f6def1c609d21bd56120827d818085d5c3b96768a1da032f8b83c46b59f", "size_bytes": 6_239, "status": "V6_COLD_A_LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY"},
    },
    "stat_denials": {
        "line_sha256": ["7fabb53adb6cf6f8626bdc0056f663a73896dc133dde2bc14d12d9fcf73ad8e2", "e511e301bf3cec1956573781301b3827b64085c7b9f17c84d5251ab1def23d17"],
        "line_size_bytes": [320, 320], "name": "/proc/3/stat", "operation": "open", "class": "file", "requested_mask": "r", "denied_mask": "r", "fsuid": "1000", "ouid": "0",
    },
    "source_use": "failure_provenance_only_no_snapshot_output_or_runtime_reuse",
    "identity_consumed": True, "retry_forbidden": True,
    "cold_b_authorized": False, "production_authorized": False,
}
V6_PROFILE_BASELINE = {
    "path": "/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-solver-cpu-settle-cold-a-v6.profile",
    "sha256": "a6841cd0fad78fa31e1a2a7b04eab0eb9158856884162112216ddcb8a01bfe0a",
    "size_bytes": 3_540,
    "bootstrap_profile": "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-bootstrap-v6-20260808t212044z",
    "runtime_profile": "r8-liquid-u3-c1m-solver-cpu-settle-cold-a-runtime-v6-20260808t212044z",
    "effective_line_count": 64,
}
FROZEN_V1_NO_GO = {
    "probe_id": "u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z",
    "decision": "STATIC_NO_GO_FULL_PROFILE_AUTHORITY_DELTA_VIOLATION",
    "blocker": {
        "reason": "v1_added_three_dynamic_linker_read_allows_beyond_the_single_candidate_rule",
        "exact_differences": [
            {
                "v1_rule": "/etc/ld.so.cache r,",
                "v6_rule": "deny /etc/ld.so.cache r,",
                "classification": "frozen_explicit_deny_replaced_by_allow",
            },
            {
                "v1_rule": "/etc/ld.so.conf r,",
                "v6_rule": None,
                "classification": "new_allow_absent_from_v6",
            },
            {
                "v1_rule": "/etc/ld.so.conf.d/** r,",
                "v6_rule": None,
                "classification": "new_allow_absent_from_v6",
            },
        ],
    },
    "snapshot": {
        "root": "/run/r8-liquid-u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.snapshot",
        "owner": [0, 0],
        "mode": "0555",
        "nlink": 2,
        "preserve_unchanged": True,
        "files": {
            "gate": {
                "name": "r8_liquid_target_u3_proc_stat_boundary_probe_gate_v1.py",
                "sha256": "07c4b9e0f962b9edc50945e3ad0eca09899b58487593ca021e4e8a537b9dcd32",
                "size_bytes": 97_647,
                "owner": [0, 0], "mode": "0444", "nlink": 1,
            },
            "supervisor": {
                "name": "r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v1.py",
                "sha256": "060467492da3fee5689f342b1ab6ecbe3ca33f99894e59a8967a871c07725e34",
                "size_bytes": 147_548,
                "owner": [0, 0], "mode": "0444", "nlink": 1,
            },
            "profile": {
                "name": "r8-liquid-u3-proc-stat-boundary-probe-v1.profile",
                "sha256": "d4949a663426e465c1f523170717f622420d5527792a6864239a7e46fbbcdfa8",
                "size_bytes": 3_217,
                "owner": [0, 0], "mode": "0444", "nlink": 1,
            },
            "schema": {
                "name": "target_host_u3_proc_stat_boundary_probe_policy_v1.json",
                "sha256": "b496cd89adab90fcb787d7c17fd0c25c47649c7856a7db84bb026956e82b8116",
                "size_bytes": 35_116,
                "owner": [0, 0], "mode": "0444", "nlink": 1,
            },
            "policy": {
                "name": "liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v1.json",
                "sha256": "27395abee5ae0a7784e9d49ae38d6c3a68a62b9f123a9e7a3854cd9be0605f0a",
                "size_bytes": 26_497,
                "owner": [0, 0], "mode": "0444", "nlink": 1,
            },
        },
    },
    "receipt_paths": {
        "start": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.start.json",
        "preflight_failure": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.preflight_incomplete.json",
        "execution": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.execution.json",
        "lifecycle": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.lifecycle.json",
        "lifecycle_failure": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.lifecycle_incomplete.json",
        "recovery": "/home/zrj/scout_liquid_lab/audits/u3_proc_stat_apparmor_boundary_probe_v1_20260809T002729Z.recovery.json",
    },
    "receipts_observed_absent": True,
    "observed_facts": {
        "snapshot_created": True,
        "profile_loaded": False,
        "probe_executed": False,
        "receipt_created": False,
    },
    "source_use": "static_no_go_provenance_only_no_runtime_baseline_or_snapshot_artifact_reuse",
    "identity_consumed": True,
    "retry_forbidden": True,
    "production_authorized": False,
}
RECOVERY_TOKEN = f"{PROBE_ID}:cleanup-only-no-probe"
POLICY_NAME = "liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v2.json"
SCHEMA_NAME = "target_host_u3_proc_stat_boundary_probe_policy_v2.json"
PROFILE_NAME = "r8-liquid-u3-proc-stat-boundary-probe-v2.profile"
GATE_NAME = "r8_liquid_target_u3_proc_stat_boundary_probe_gate_v2.py"
SUPERVISOR_NAME = SCRIPT_PATH.name
BOOTSTRAP_PROFILE = "r8-liquid-u3-proc-stat-boundary-bootstrap-v2-20260809t014926z"
RUNTIME_PROFILE = "r8-liquid-u3-proc-stat-boundary-runtime-v2-20260809t014926z"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)
HOST_UID = 1000
HOST_GID = 1000
SUDO_GROUP_GID = 27
SUDO_GROUP_PATH = Path("/etc/group")
SUDO_GROUP_RECORD = "sudo:x:27:zrj"
SUDO_GROUP_MEMBERSHIP_CONTRACT = {
    "path": str(SUDO_GROUP_PATH),
    "owner": [0, 0],
    "mode": "0644",
    "record": SUDO_GROUP_RECORD,
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
SUCCESS_MAGIC = b"R8PROCSTATPASSV2\x00"
MAX_SUCCESS_FRAME_BYTES = 16_384
TMPFS_BYTES = 67_108_864
PROC_MOUNT_RULE = 'mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,'
BASELINE_MOUNT_RULES = (
    "mount options=(rw, silent, rslave) -> /,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,',
    'mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,',
    'pivot_root oldroot=/tmp/oldroot/ /tmp/,',
    "mount options=(rw, silent, rprivate) -> /oldroot/,",
    'umount /oldroot/,',
    'pivot_root oldroot=/newroot/ /newroot/,',
    "umount /,",
    'mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,',
    "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,",
    'mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,',
    PROC_MOUNT_RULE,
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
    "deny /etc/ld.so.cache r,",
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
EXPECTED_FULL_AUTHORITY_ADDITIONS = ("/proc/[0-9]*/stat r,",)
EXPECTED_FULL_AUTHORITY_REMOVALS = (
    "/work/runtime/ld-linux-x86-64.so.2 rix,",
    "/work/** rw,",
    "/work/runtime/ld-linux-x86-64.so.2 mr,",
    "/work/runtime/DualSPHysics5.4CPU_linux64 mr,",
    "/work/runtime/lib/** mr,",
)
EXPECTED_NORMALIZED_PROFILE_LINES_SHA256 = "93bd45c83d3a8cece8f9acd327f478322e0d85dc35a01ba725da86635e3097b6"
EVIDENCE_CLASSIFICATION_CONTRACT = {'success_requires_closed_frame': True, 'successful_gate_exit_is_pass': False, 'success_pass_requires': 'returncode_zero_empty_stderr_byte_exact_success_frame_and_exactly_one_expected_dumpable_zero_child_status_denial_in_closed_boot_cursor_window', 'audit_storage_limit': 16, 'audit_line_limit_bytes': 4096, 'expected_counts': {'matching_total': 1, 'stored_count': 1, 'dropped_count': 0, 'expected_status_total': 1, 'stat_denial_total': 0, 'unexpected_total': 0, 'storage_overflow': False}, 'expected_status_denial': {'apparmor': 'DENIED', 'operation': 'open', 'class': 'file', 'profile': 'r8-liquid-u3-proc-stat-boundary-bootstrap-v2-20260809t014926z', 'name_template': '/proc/<SUCCESS_FRAME_CHILD_PID>/status', 'name_must_match_success_frame_child_pid': True, 'comm': 'python3.12', 'requested_mask': 'r', 'denied_mask': 'r', 'fsuid': '1000', 'ouid': '0', 'allowed_field_names': ['apparmor', 'operation', 'class', 'profile', 'name', 'pid', 'comm', 'requested_mask', 'denied_mask', 'fsuid', 'ouid']}, 'journal_boundary_required': True, 'strict_utf8_required': True, 'duplicate_or_escaped_critical_field_is_pass': False, 'audit_overflow_is_pass': False, 'gate_no_go_is_pass': False, 'zero_logged_denials_required': False, 'zero_denied_operations_claimed': False, 'stderr_contract': {'returncode': 0, 'stdout': 'byte_exact_canonical_success_frame', 'stderr_utf8': ''}, 'any_other_failure': 'UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING', 'success_authorizes': 'only_independent_review_and_static_design_of_a_fresh_cold_a_successor_no_solver_gencase_or_production'}
AUDIT_STORAGE_CEILING = 16
AUDIT_LINE_MAX_BYTES = 4_096
AUDIT_QUERY_STDOUT_LIMIT = 131_072
AUDIT_QUERY_STDERR_LIMIT = 16_384
BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
BOOT_ID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
JOURNAL_CURSOR_RE = re.compile(r"^[A-Za-z0-9_.:;=+/-]{1,2048}$")
AUDIT_TOKEN_RE = re.compile(
    r'([A-Za-z_][A-Za-z0-9_]*)=("(?:[^"\\]|\\.)*"|[^\s"]+)(?:\s+|$)'
)
AUDIT_CRITICAL_FIELDS = frozenset(
    {
        "apparmor", "operation", "class", "profile", "name", "pid", "comm",
        "requested_mask", "denied_mask", "fsuid", "ouid",
    }
)
AUDIT_STATUS_ALLOWED_FIELDS = frozenset(
    {
        "apparmor", "operation", "class", "profile", "name", "pid", "comm",
        "requested_mask", "denied_mask", "fsuid", "ouid",
    }
)
AUDIT_FORBIDDEN_TOKENS = (
    "/home/zrj/scout_ws", "GenCase", "DualSPHysics", "/dev/nvidia", "/opt/ros",
)
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
AUDIT_ROOT = LIQUID_ROOT / "audits"
START_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.start.json"
PREFLIGHT_FAILURE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.preflight_incomplete.json"
EXECUTION_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.execution.json"
LIFECYCLE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.lifecycle.json"
LIFECYCLE_FAILURE_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.lifecycle_incomplete.json"
RECOVERY_RECEIPT = AUDIT_ROOT / f"{PROBE_ID}.recovery.json"
FROZEN_RECEIPT_PATHS = {
    "start_receipt": str(START_RECEIPT),
    "preflight_failure_receipt": str(PREFLIGHT_FAILURE_RECEIPT),
    "execution_receipt": str(EXECUTION_RECEIPT),
    "lifecycle_receipt": str(LIFECYCLE_RECEIPT),
    "lifecycle_failure_receipt": str(LIFECYCLE_FAILURE_RECEIPT),
    "recovery_receipt": str(RECOVERY_RECEIPT),
}

AUDIT_DIRECTORY_POLICY = {
    Path("/"): (0, 0, 0o755),
    Path("/home"): (0, 0, 0o755),
    Path("/home/zrj"): (HOST_UID, HOST_GID, 0o750),
    LIQUID_ROOT: (HOST_UID, HOST_GID, 0o750),
    AUDIT_ROOT: (HOST_UID, HOST_GID, 0o750),
}

REPOSITORY_PATHS = {
    "gate": PACKAGE_DIR / "scripts" / GATE_NAME,
    "supervisor": SCRIPT_PATH,
    "profile": PACKAGE_DIR / "config/apparmor_drafts" / PROFILE_NAME,
    "schema": PACKAGE_DIR / "schema" / SCHEMA_NAME,
    "policy": PACKAGE_DIR / "config/target_hosts" / POLICY_NAME,
}
SNAPSHOT_PATHS = {name: SNAPSHOT_ROOT / path.name for name, path in REPOSITORY_PATHS.items()}

SYSCTL_PATHS = (
    Path("/proc/sys/user/max_user_namespaces"),
    Path("/proc/sys/kernel/unprivileged_userns_clone"),
    Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"),
)
KERNEL_PROFILE_PATH = Path("/sys/kernel/security/apparmor/profiles")
AA_STATUS_JSON_ARGV = ("/usr/sbin/aa-status", "--json")

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "journalctl": (Path("/usr/bin/journalctl"), "c49bd25d7e7655b9a44ff867923952ed5a5e0a65e9df7a0510e239bf0558e3fa"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sudo": (Path("/usr/bin/sudo"), "136f2e48b0295b9fc595b8259cf2411ac43f27ddbfe02b956649ddaa2e92b9fa"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
    "true": (Path("/usr/bin/true"), "4b5a5694e3c0e8b1d58fc52ac6ef076e55e72c2f53195243ac86d5ff517cc2f6"),
}

SNAPSHOT_BOOTSTRAP_SOURCE = r'''import hashlib,json,os,stat,sys
ROOT="/run"
NAME="r8-liquid-u3_proc_stat_apparmor_boundary_probe_v2_20260809T014926Z.snapshot"
SOURCES=(
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_proc_stat_boundary_probe_gate_v2.py","r8_liquid_target_u3_proc_stat_boundary_probe_gate_v2.py"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v2.py","r8_liquid_target_u3_proc_stat_boundary_probe_supervisor_v2.py"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-proc-stat-boundary-probe-v2.profile","r8-liquid-u3-proc-stat-boundary-probe-v2.profile"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/schema/target_host_u3_proc_stat_boundary_probe_policy_v2.json","target_host_u3_proc_stat_boundary_probe_policy_v2.json"),
 ("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/config/target_hosts/liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v2.json","liquid_zrj_msi_u2404_u3_proc_stat_boundary_probe_policy_v2.json"),
)
def open_abs(path):
 parts=[p for p in path.split("/") if p]
 d=os.open("/",os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:
  for part in parts[:-1]:
   n=os.open(part,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
   os.close(d); d=n
  return os.open(parts[-1],os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
 finally:
  os.close(d)
def read_source(path,size,digest):
 f=open_abs(path)
 try:
  m=os.fstat(f)
  if not stat.S_ISREG(m.st_mode) or m.st_nlink!=1 or m.st_size!=size: raise SystemExit(71)
  b=bytearray()
  while len(b)<size:
   q=os.read(f,min(1048576,size-len(b)))
   if not q: raise SystemExit(72)
   b.extend(q)
  if os.read(f,1)!=b"" or hashlib.sha256(b).hexdigest()!=digest: raise SystemExit(73)
  return bytes(b)
 finally: os.close(f)
if os.geteuid()!=0 or len(sys.argv)!=1+2*len(SOURCES): raise SystemExit(70)
manifest=[]
payloads=[]
for i,(source,name) in enumerate(SOURCES):
 digest=sys.argv[1+2*i]; size=int(sys.argv[2+2*i])
 if len(digest)!=64 or size<1 or size>2097152: raise SystemExit(74)
 raw=read_source(source,size,digest); payloads.append((name,raw,digest,size))
runfd=os.open(ROOT,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
try:
 os.mkdir(NAME,0o700,dir_fd=runfd)
 snapfd=os.open(NAME,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=runfd)
 try:
  for name,raw,digest,size in payloads:
   f=os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o400,dir_fd=snapfd)
   try:
    view=memoryview(raw)
    while view:
     n=os.write(f,view)
     if n<=0: raise SystemExit(75)
     view=view[n:]
    os.fsync(f); os.fchmod(f,0o444); os.fsync(f); m=os.fstat(f)
    if m.st_uid!=0 or m.st_gid!=0 or stat.S_IMODE(m.st_mode)!=0o444 or m.st_nlink!=1: raise SystemExit(76)
   finally: os.close(f)
   manifest.append({"name":name,"sha256":digest,"size_bytes":size})
  os.fchmod(snapfd,0o555); os.fsync(snapfd)
 finally: os.close(snapfd)
 os.fsync(runfd)
finally: os.close(runfd)
os.write(1,(json.dumps({"status":"ROOT_PROC_STAT_PROBE_V2_SNAPSHOT_CREATED_NOT_EXECUTED","files":manifest},sort_keys=True,separators=(",",":"))+"\n").encode("ascii"))
'''
SNAPSHOT_BOOTSTRAP_BYTES = SNAPSHOT_BOOTSTRAP_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_BYTES).hexdigest()
SNAPSHOT_BOOTSTRAP_LOADER_SOURCE = f'''import hashlib,sys
B=sys.stdin.buffer
n={len(SNAPSHOT_BOOTSTRAP_BYTES)}
p=bytearray()
while len(p)<n:
 q=B.read(n-len(p))
 if not q: raise SystemExit(81)
 p.extend(q)
p=bytes(p)
if B.read(1)!=b"" or hashlib.sha256(p).hexdigest()!="{SNAPSHOT_BOOTSTRAP_SHA256}": raise SystemExit(81)
g={{"__name__":"__main__"}}
exec(compile(p,"<r8-liquid-root-proc-stat-probe-bootstrap-v2>","exec",dont_inherit=True,optimize=2),g,g)
'''
SNAPSHOT_BOOTSTRAP_LOADER_BYTES = SNAPSHOT_BOOTSTRAP_LOADER_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_LOADER_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_LOADER_BYTES).hexdigest()

sys.dont_write_bytecode = True


class SupervisorError(RuntimeError):
    """A fail-closed snapshot, execution, evidence, or lifecycle error."""


class SupervisorTermination(SupervisorError):
    """A catchable external termination request that must enter cleanup."""


class TerminationGuard:
    def __init__(self) -> None:
        self.received: list[int] = []
        self.cleanup_started = False
        self.preflight_sudo_cleanup_attempted = False
        self.preflight_sudo_cleanup_evidence: dict[str, Any] = {}
        self.preflight_sudo_cleanup_error: str | None = None
        self.previous: dict[signal.Signals, Any] = {}

    def _handle(self, signum: int, _frame: Any) -> None:
        self.received.append(signum)

    def checkpoint(self) -> None:
        if self.received and not self.cleanup_started:
            raise SupervisorTermination(f"external termination signal received: {self.received[-1]}")

    def __enter__(self) -> "TerminationGuard":
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM, signal.SIGQUIT, signal.SIGTSTP):
            self.previous[sig] = signal.getsignal(sig)
            signal.signal(sig, self._handle)
        return self

    def begin_cleanup(self) -> None:
        self.cleanup_started = True

    def __exit__(self, _exc_type: Any, _exc: Any, _traceback: Any) -> None:
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_regular_bytes(path: Path, *, limit: int = 4 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise SupervisorError(f"unsafe regular file: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, min(1 << 20, limit + 1 - len(result)))
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise SupervisorError(f"regular file exceeds ceiling: {path}")
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            read_regular_bytes(path).decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, str(path)),
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON root is not object: {path}")
    return value


def _assert_schema_objects_closed(node: Any, path: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise SupervisorError(f"schema object is not closed: {path}")
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
            raise SupervisorError(f"unsupported schema reference: {path}")
        target: Any = root
        for part in reference[2:].split("/"):
            target = target[part.replace("~1", "/").replace("~0", "~")]
        if not isinstance(target, dict):
            raise SupervisorError(f"schema reference target differs: {path}")
        _validate_schema_instance(instance, target, root, path)
        return
    if "const" in node and not _json_equal(instance, node["const"]):
        raise SupervisorError(f"policy const differs: {path}")
    if "enum" in node and not any(_json_equal(instance, option) for option in node["enum"]):
        raise SupervisorError(f"policy enum differs: {path}")
    expected_type = node.get("type")
    type_ok = {
        "object": isinstance(instance, dict),
        "array": isinstance(instance, list),
        "string": isinstance(instance, str),
        "integer": isinstance(instance, int) and not isinstance(instance, bool),
        "boolean": isinstance(instance, bool),
    }.get(expected_type, True)
    if not type_ok:
        raise SupervisorError(f"policy type differs: {path}")
    if expected_type == "object":
        properties = node.get("properties", {})
        required = node.get("required", [])
        if not isinstance(properties, dict) or not isinstance(required, list) or any(key not in instance for key in required):
            raise SupervisorError(f"policy required object key differs: {path}")
        if node.get("additionalProperties") is False and any(key not in properties for key in instance):
            raise SupervisorError(f"policy object has an additional key: {path}")
        for key, value in instance.items():
            child = properties.get(key)
            if isinstance(child, dict):
                _validate_schema_instance(value, child, root, f"{path}.{key}")
    elif expected_type == "array":
        if len(instance) < node.get("minItems", 0) or len(instance) > node.get("maxItems", 1 << 60):
            raise SupervisorError(f"policy array length differs: {path}")
        child = node.get("items")
        if isinstance(child, dict):
            for index, value in enumerate(instance):
                _validate_schema_instance(value, child, root, f"{path}[{index}]")
    elif expected_type == "string":
        if len(instance) < node.get("minLength", 0) or len(instance) > node.get("maxLength", 1 << 60):
            raise SupervisorError(f"policy string length differs: {path}")
        pattern = node.get("pattern")
        if pattern is not None and re.search(pattern, instance) is None:
            raise SupervisorError(f"policy string pattern differs: {path}")
    elif expected_type == "integer":
        if instance < node.get("minimum", -(1 << 63)) or instance > node.get("maximum", 1 << 63):
            raise SupervisorError(f"policy integer range differs: {path}")


def verify_v6_failure_provenance() -> dict[str, Any]:
    frozen = FROZEN_V6_FAILURE
    policy_record = frozen["policy"]
    policy_raw = read_regular_bytes(Path(policy_record["path"]))
    if (
        len(policy_raw) != policy_record["size_bytes"]
        or sha256_bytes(policy_raw) != policy_record["sha256"]
    ):
        raise SupervisorError("frozen v6 policy bytes differ")
    try:
        v6_policy = json.loads(
            policy_raw.decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, "frozen v6 policy"),
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("frozen v6 policy JSON differs") from exc
    attempt = v6_policy.get("frozen_attempt", {})
    if (
        attempt.get("case_id") != frozen["case_id"]
        or attempt.get("campaign_id") != frozen["campaign_id"]
        or attempt.get("campaign_role") != "cold_a"
        or attempt.get("cold_b_identity_allocated") is not False
        or v6_policy.get("invariants", {}).get("production_authorized") is not False
    ):
        raise SupervisorError("frozen v6 policy failure identity differs")
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
            raise SupervisorError(f"frozen v6 receipt bytes or inode differ: {name}")
        try:
            document = json.loads(
                raw.decode("utf-8"),
                object_pairs_hook=lambda pairs: _strict_pairs(pairs, f"frozen v6 receipt {name}"),
            )
        except (json.JSONDecodeError, UnicodeError) as exc:
            raise SupervisorError(f"frozen v6 receipt JSON differs: {name}") from exc
        if (
            document.get("case_id") != frozen["case_id"]
            or document.get("campaign_id") != frozen["campaign_id"]
            or document.get("campaign_role") != "cold_a"
            or document.get("status") != record["status"]
            or document.get("policy_sha256") not in (None, policy_record["sha256"])
        ):
            raise SupervisorError(f"frozen v6 receipt identity differs: {name}")
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
        raise SupervisorError("frozen v6 receipt chain differs")
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
        raise SupervisorError("frozen v6 stat denial accounting differs")
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
            raise SupervisorError("frozen v6 denial fields differ")
        if (
            len(fields) != 11
            or any(
                not isinstance(field, dict)
                or set(field) != {"key", "raw_value", "value"}
                or not isinstance(field["raw_value"], str)
                for field in fields
            )
        ):
            raise SupervisorError("frozen v6 denial field records differ")
        keys = [field["key"] for field in fields]
        if any(not isinstance(key, str) for key in keys) or len(set(keys)) != 11:
            raise SupervisorError("frozen v6 denial field keys are not unique")
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
            raise SupervisorError("frozen v6 denial semantics differ")
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
        raise SupervisorError("frozen v6 failure or cleanup closure differs")
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
            raise SupervisorError(f"duplicate JSON key in {context}: {key}")
        value[key] = item
    return value



def verify_v1_no_go_provenance() -> dict[str, Any]:
    frozen = FROZEN_V1_NO_GO
    snapshot = frozen["snapshot"]
    root = Path(snapshot["root"])
    metadata = os.lstat(root)
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or [metadata.st_uid, metadata.st_gid] != snapshot["owner"]
        or stat.S_IMODE(metadata.st_mode) != int(snapshot["mode"], 8)
        or metadata.st_nlink != snapshot["nlink"]
    ):
        raise SupervisorError("frozen v1 NO-GO snapshot root metadata differs")
    expected_names = {record["name"] for record in snapshot["files"].values()}
    if set(os.listdir(root)) != expected_names:
        raise SupervisorError("frozen v1 NO-GO snapshot file set differs")
    observed_files: dict[str, dict[str, Any]] = {}
    for name, record in snapshot["files"].items():
        path = root / record["name"]
        file_metadata = os.lstat(path)
        raw = read_regular_bytes(path)
        if (
            not stat.S_ISREG(file_metadata.st_mode)
            or [file_metadata.st_uid, file_metadata.st_gid] != record["owner"]
            or stat.S_IMODE(file_metadata.st_mode) != int(record["mode"], 8)
            or file_metadata.st_nlink != record["nlink"]
            or len(raw) != record["size_bytes"]
            or sha256_bytes(raw) != record["sha256"]
        ):
            raise SupervisorError(f"frozen v1 NO-GO snapshot artifact differs: {name}")
        observed_files[name] = {
            "path": str(path),
            "sha256": record["sha256"],
            "size_bytes": record["size_bytes"],
            "owner": list(record["owner"]),
            "mode": record["mode"],
            "nlink": record["nlink"],
        }
    policy_record = snapshot["files"]["policy"]
    try:
        v1_policy = json.loads(
            read_regular_bytes(root / policy_record["name"]).decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, "frozen v1 NO-GO policy"),
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("frozen v1 NO-GO snapshot policy JSON differs") from exc
    if (
        v1_policy.get("schema_version")
        != "smpcc-r8-liquid-target-u3-proc-stat-apparmor-boundary-probe-policy-v1"
        or v1_policy.get("policy_id")
        != "LIQUID_ZRJ_MSI_U2404_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V1"
        or v1_policy.get("status")
        != "STATIC_READY_V1_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT"
        or v1_policy.get("frozen_identity", {}).get("probe_id") != frozen["probe_id"]
        or v1_policy.get("authorization", {}).get("static_task_execution_performed") is not False
        or v1_policy.get("authorization", {}).get("execution_authorized_now") is not False
        or v1_policy.get("authorization", {}).get("solver_or_gencase_authorized") is not False
        or v1_policy.get("authorization", {}).get("production_authorized") is not False
    ):
        raise SupervisorError("frozen v1 NO-GO snapshot policy identity or non-execution state differs")
    for receipt_path in frozen["receipt_paths"].values():
        try:
            os.lstat(receipt_path)
        except FileNotFoundError:
            continue
        raise SupervisorError(f"frozen v1 receipt must remain absent: {receipt_path}")
    return {
        "decision": frozen["decision"],
        "blocker": frozen["blocker"],
        "snapshot": {
            "root": str(root),
            "owner": list(snapshot["owner"]),
            "mode": snapshot["mode"],
            "nlink": snapshot["nlink"],
            "files": observed_files,
            "preserved_unchanged": True,
        },
        "receipts_absent": True,
        "observed_facts": dict(frozen["observed_facts"]),
        "runtime_baseline_used": False,
        "snapshot_artifact_reused": False,
        "identity_consumed": True,
        "retry_forbidden": True,
    }




def _effective_profile_lines(text: str) -> tuple[str, ...]:
    return tuple(
        line.strip()
        for line in (raw.split("#", 1)[0] for raw in text.splitlines())
        if line.strip()
    )


def _normalize_profile_authority(
    lines: tuple[str, ...], bootstrap_label: str, runtime_label: str,
) -> tuple[str, ...]:
    return tuple(
        line.replace(bootstrap_label, "r8-liquid-u3-NORMALIZED-bootstrap").replace(
            runtime_label, "r8-liquid-u3-NORMALIZED-runtime"
        )
        for line in lines
    )


def _ordered_multiset_delta(
    candidate: tuple[str, ...], baseline: tuple[str, ...],
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    baseline_remaining = Counter(baseline)
    additions: list[str] = []
    for line in candidate:
        if baseline_remaining[line] > 0:
            baseline_remaining[line] -= 1
        else:
            additions.append(line)
    candidate_remaining = Counter(candidate)
    removals: list[str] = []
    for line in baseline:
        if candidate_remaining[line] > 0:
            candidate_remaining[line] -= 1
        else:
            removals.append(line)
    return tuple(additions), tuple(removals)


def verify_full_authority_delta(candidate_lines: tuple[str, ...]) -> dict[str, Any]:
    baseline_path = Path(V6_PROFILE_BASELINE["path"])
    raw = read_regular_bytes(baseline_path, limit=128 * 1024)
    if (
        len(raw) != V6_PROFILE_BASELINE["size_bytes"]
        or sha256_bytes(raw) != V6_PROFILE_BASELINE["sha256"]
    ):
        raise SupervisorError("frozen v6 profile authority baseline bytes differ")
    try:
        baseline_lines = _effective_profile_lines(raw.decode("utf-8"))
    except UnicodeError as exc:
        raise SupervisorError("frozen v6 profile authority baseline is not UTF-8") from exc
    if len(baseline_lines) != V6_PROFILE_BASELINE["effective_line_count"]:
        raise SupervisorError("frozen v6 profile effective-line count differs")
    normalized_candidate = _normalize_profile_authority(
        candidate_lines, BOOTSTRAP_PROFILE, RUNTIME_PROFILE
    )
    normalized_baseline = _normalize_profile_authority(
        baseline_lines,
        V6_PROFILE_BASELINE["bootstrap_profile"],
        V6_PROFILE_BASELINE["runtime_profile"],
    )
    normalized_digest = hashlib.sha256(
        ("\n".join(normalized_candidate) + "\n").encode("utf-8")
    ).hexdigest()
    if normalized_digest != EXPECTED_NORMALIZED_PROFILE_LINES_SHA256:
        raise SupervisorError("normalized v2 full-profile digest differs")
    additions, removals = _ordered_multiset_delta(
        normalized_candidate, normalized_baseline
    )
    if additions != EXPECTED_FULL_AUTHORITY_ADDITIONS:
        raise SupervisorError("full normalized profile authority additions exceed the single candidate")
    if removals != EXPECTED_FULL_AUTHORITY_REMOVALS:
        raise SupervisorError("full normalized profile authority removals differ")
    return {
        "baseline": dict(V6_PROFILE_BASELINE),
        "comparison": "ordered_multiset_effective_lines_after_exact_profile_label_normalization",
        "candidate_effective_line_count": len(candidate_lines),
        "normalized_candidate_sha256": normalized_digest,
        "additions": list(additions),
        "removals": list(removals),
        "only_addition_is_candidate": True,
    }


def verify_profile_bytes(path: Path | None = None) -> dict[str, Any]:
    """Independently review the candidate profile without importing gate code."""

    profile_path = REPOSITORY_PATHS["profile"] if path is None else path
    raw = read_regular_bytes(profile_path, limit=128 * 1024)
    text = raw.decode("utf-8")
    for raw_line in text.splitlines():
        stripped = raw_line.lstrip()
        if re.match(r"#\s*include\b", stripped) or re.match(r"include\b", stripped):
            raise SupervisorError("AppArmor include directives are forbidden")
    lines = _effective_profile_lines(text)
    if lines != EXPECTED_PROFILE_LINES:
        raise SupervisorError("effective AppArmor rule set differs from supervisor tuple")
    digest = hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()
    if digest != EXPECTED_PROFILE_LINES_SHA256:
        raise SupervisorError("effective AppArmor rule-set digest differs")
    candidate = "/proc/[0-9]*/stat r,"
    if lines.count(candidate) != 1:
        raise SupervisorError("numeric proc-stat candidate rule count differs")
    if lines.count("deny /etc/ld.so.cache r,") != 1:
        raise SupervisorError("frozen v6 dynamic-linker cache deny differs")
    if any(line in lines for line in (
        "/etc/ld.so.cache r,", "/etc/ld.so.conf r,", "/etc/ld.so.conf.d/** r,"
    )):
        raise SupervisorError("dynamic-linker read authority exceeds frozen v6")
    effective = "\n".join(lines)
    bootstrap, runtime = effective.split(f"profile {RUNTIME_PROFILE}", 1)
    if candidate not in bootstrap or candidate in runtime:
        raise SupervisorError("numeric proc-stat candidate is not bootstrap-only")
    forbidden = (
        "/newroot/proc/**", "/newroot/proc/*", "/dev/", "/dev ",
        "GenCase", "DualSPHysics", "/home/zrj/scout_ws", "/opt/ros",
        "flags=(unconfined)", " mount,", "/proc/*/stat r,",
        "/proc/**/stat", "/proc/[0-9]*/**", "/proc/[0-9]*/status",
        "/proc/[0-9]*/{stat,status}", "/**/stat", "@{PROC}",
    )
    if any(token in effective for token in forbidden):
        raise SupervisorError("profile contains forbidden or guessed authority")
    for token in ("userns ", "mount ", "capability ", "network ", "/proc", "/dev", "/work", "/usr/bin/python"):
        if token in runtime:
            raise SupervisorError(f"runtime profile contains authority: {token}")
    mount_lines = tuple(
        line for line in lines
        if line.startswith(("mount ", "remount ", "pivot_root ", "umount "))
    )
    if mount_lines != BASELINE_MOUNT_RULES:
        raise SupervisorError("mount baseline differs")
    authority_delta = verify_full_authority_delta(lines)
    return {
        "path": str(profile_path),
        "sha256": sha256_bytes(raw),
        "effective_lines_sha256": digest,
        "effective_line_count": len(lines),
        "candidate_rule": candidate,
        "candidate_rule_count": 1,
        "full_effective_authority_delta": authority_delta,
        "mount_rules": list(mount_lines),
        "production_authorized": False,
    }


def repository_static_review() -> dict[str, Any]:
    policy = read_json(REPOSITORY_PATHS["policy"])
    schema = read_json(REPOSITORY_PATHS["schema"])
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("closed policy/schema top-level contract differs")

    identities = {
        "schema_version": "smpcc-r8-liquid-target-u3-proc-stat-apparmor-boundary-probe-policy-v2",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
    }
    if any(policy.get(key) != value for key, value in identities.items()):
        raise SupervisorError("fresh proc-stat policy identity differs")
    draft_status = "STATIC_DRAFT_V2_PENDING_REVIEWED_BYTE_HASH_FREEZE_AND_INDEPENDENT_REVIEW"
    ready_status = "STATIC_READY_V2_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT"
    draft_next = "FREEZE_REVIEWED_BYTES_THEN_INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
    ready_next = "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
    if (policy.get("status"), policy.get("next_allowed_stage")) not in {
        (draft_status, draft_next), (ready_status, ready_next),
    }:
        raise SupervisorError("fresh proc-stat policy review state differs")
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
        raise SupervisorError("static-only one-shot authorization differs")

    expected_frozen = {
        "probe_id": PROBE_ID,
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        **FROZEN_RECEIPT_PATHS,
    }
    if policy.get("frozen_identity") != expected_frozen:
        raise SupervisorError("frozen probe identity, labels, or receipt paths differ")
    expected_provenance = {
        "cold_a_v6_failure": FROZEN_V6_FAILURE,
        "proc_stat_v1_static_no_go": FROZEN_V1_NO_GO,
    }
    if policy.get("provenance") != expected_provenance:
        raise SupervisorError("frozen v6 and v1 NO-GO provenance policy differs")
    v6 = verify_v6_failure_provenance()
    v1_no_go = verify_v1_no_go_provenance()

    profile = verify_profile_bytes()
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
        or semantics.get("explicit_silent_denials")
        != ["deny /etc/ld.so.cache r,", "deny capability dac_override,"]
        or semantics.get("full_effective_authority_delta")
        != profile["full_effective_authority_delta"]
        or semantics.get("dynamic_linker_contract") != {
            "cache_rule": "deny /etc/ld.so.cache r,",
            "ld_so_conf_rule_present": False,
            "ld_so_conf_d_rule_present": False,
            "frozen_v6_system_python_started_with_cache_deny": True,
        }
    ):
        raise SupervisorError("profile semantics policy differs")
    candidate = policy.get("candidate_boundary", {})
    if (
        candidate.get("candidate_rule") != "/proc/[0-9]*/stat r,"
        or candidate.get("candidate_rule_count") != 1
        or candidate.get("candidate_rule_profile") != "bootstrap_only"
        or candidate.get("candidate_delta_from_v6")
        != "full_effective_authority_additions_after_exact_label_normalization_equal_only_the_bootstrap_numeric_pid_stat_rule"
        or candidate.get("status_rule_present") is not False
        or candidate.get("child_creation") != "os_fork_single_child_no_exec"
        or candidate.get("child_count") != 1
        or candidate.get("success_frame_required") is not True
        or candidate.get("host_file_writes") != 0
        or candidate.get("solver_or_gencase_execution") is not False
        or candidate.get("live_phase", {}).get("direct_stat_reads") != 2
        or candidate.get("status_boundary_control", {}).get("open_attempts") != 1
        or candidate.get("status_boundary_control", {}).get("required_errno") != errno.EACCES
        or candidate.get("zombie_phase", {}).get("direct_stat_reads") != 2
        or candidate.get("reap_phase", {}).get("post_reap_group_scans") != 2
    ):
        raise SupervisorError("candidate proc-stat state-machine policy differs")

    payload = policy.get("payload_conduit", {})
    if (
        payload.get("frame_magic_hex") != "523850524f435354415450524f42453200"
        or payload.get("success_magic_hex") != SUCCESS_MAGIC.hex()
        or payload.get("success_status") != "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2"
        or payload.get("external_payload_count") != 0
        or payload.get("host_writable_transport") is not False
        or payload.get("maximum_payload_frame_bytes") != 65_536
        or payload.get("maximum_success_frame_bytes") != MAX_SUCCESS_FRAME_BYTES
        or payload.get("maximum_child_stderr_bytes") != 4_096
        or payload.get("outer_deadline_seconds") != 20
        or payload.get("isolation", {}).get("host_read_only_mounts") != ["/usr"]
        or payload.get("isolation", {}).get("host_writable_mounts") != []
        or payload.get("isolation", {}).get("namespaces") != ["user", "pid", "network", "ipc", "uts"]
        or payload.get("isolation", {}).get("guest_dev") != "absent"
        or payload.get("isolation", {}).get("solver_gencase_ros_gazebo_gpu") != "not_executed_not_exposed"
    ):
        raise SupervisorError("payload conduit or isolation policy differs")

    expected_unprivileged = {
        name: {"path": str(TRUSTED_TOOLS[name][0]), "sha256": TRUSTED_TOOLS[name][1]}
        for name in ("aa_exec", "bwrap", "python", "timeout")
    }
    expected_root = {
        name: {"path": str(TRUSTED_TOOLS[name][0]), "sha256": TRUSTED_TOOLS[name][1]}
        for name in ("aa_status", "apparmor_parser", "journalctl", "python", "setpriv", "sudo", "timeout", "true")
    }
    if policy.get("trusted_system_tools") != {
        "unprivileged_runner": expected_unprivileged,
        "root_supervisor": expected_root,
    }:
        raise SupervisorError("trusted system-tool policy differs")

    commands = policy.get("fixed_commands", {})
    profile_path = str(SNAPSHOT_PATHS["profile"])
    expected_commands = {
        "aa_status_json_argv": list(AA_STATUS_JSON_ARGV),
        "gate_handoff_argv": gate_handoff_argv(),
        "parser_parse_argv": ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path],
        "parser_load_argv": ["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path],
        "parser_unload_argv": ["/usr/sbin/apparmor_parser", "-R", "-K", profile_path],
        "snapshot_producer_argv": ["/usr/bin/python3.12", "-I", "-B", str(SCRIPT_PATH), "emit-bootstrap"],
        "snapshot_root_loader_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", "-c",
            "<PINNED_SNAPSHOT_BOOTSTRAP_LOADER_SOURCE>", "<TEN_SHA256_SIZE_MANIFEST_ARGUMENTS>",
        ],
        "run_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["supervisor"]),
            "run", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
            "--admission-token", ADMISSION_TOKEN,
        ],
        "recover_argv_template": [
            "/usr/bin/sudo", "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["supervisor"]),
            "recover-only", "--policy-sha256", "<EXTERNALLY_FROZEN_POLICY_SHA256>",
            "--recovery-token", RECOVERY_TOKEN,
        ],
        "sudo_timestamp_clear_argv": sudo_timestamp_argvs()[0],
        "sudo_timestamp_verify_argv": sudo_timestamp_argvs()[1],
    }
    for name, expected in expected_commands.items():
        if commands.get(name) != expected:
            raise SupervisorError(f"fixed command policy differs: {name}")
    operator_boundary = commands.get("operator_boundary", {})
    if (
        commands.get("operator_transport")
        != "snapshot producer stdout is only pinned root-loader stdin; root loader -c source is a reviewed hash-pinned literal argv token; run and recovery read only the root-owned snapshot supervisor"
        or commands.get("shell") is not False
        or commands.get("workspace_run_forbidden") is not True
        or any(operator_boundary.get(key) is not True for key in (
            "run_and_recover_foreground_pty_required",
            "inherited_dev_tty_password_input_only",
            "sudo_dash_S_forbidden",
            "password_environment_variable_forbidden",
            "fd0_redirection_forbidden",
            "background_or_detached_invocation_forbidden",
            "shell_interpolation_forbidden",
        ))
    ):
        raise SupervisorError("operator transport boundary differs")
    hash_fields_frozen = (
        commands.get("argv_hashes_frozen") is True
        and isinstance(commands.get("bwrap_argv_sha256"), str)
        and re.fullmatch(r"[0-9a-f]{64}", commands["bwrap_argv_sha256"]) is not None
        and commands["bwrap_argv_sha256"] != "0" * 64
    )
    if hash_fields_frozen is not frozen_ready:
        raise SupervisorError("fixed argv freeze state differs")

    journal = policy.get("journal_contract", {})
    if (
        journal.get("expected_status_denials") != 1
        or journal.get("unexpected_denials") != 0
        or journal.get("storage_overflow_allowed") is not False
        or journal.get("matching_labels") != list(LABELS)
        or journal.get("same_boot_id_required") is not True
        or journal.get("start_cursor_required") is not True
        or journal.get("end_cursor_required") is not True
    ):
        raise SupervisorError("journal boundary policy differs")
    if policy.get("evidence_classification") != EVIDENCE_CLASSIFICATION_CONTRACT:
        raise SupervisorError("status-denial evidence classification policy differs")
    lifecycle = policy.get("lifecycle_contract", {})
    invariants = lifecycle.get("static_and_runtime_invariants", {})
    if (
        lifecycle.get("stable_zero_label_scans") != 3
        or lifecycle.get("unload_requires_zero_labels") is not True
        or lifecycle.get("guest_child_cleanup")
        != "pidfd anchored release or SIGKILL then final reap without reopening child status"
        or lifecycle.get("recovery", {}).get("token") != RECOVERY_TOKEN
        or lifecycle.get("recovery", {}).get("probe_execution") is not False
        or any(invariants.get(key) is not True for key in (
            "no_sudo_in_static_checks", "no_parser_in_static_checks",
            "no_namespace_in_static_checks", "no_probe_in_static_checks",
            "no_v6_snapshot_read_or_reuse", "no_v6_output_reuse",
            "no_solver_or_gencase", "no_host_writable_mount", "no_dev",
            "no_external_network_access", "no_host_initial_namespace_sysctl_write",
            "no_persistent_profile", "no_ros_gazebo_gpu", "no_production_authority",
        ))
    ):
        raise SupervisorError("lifecycle invariant policy differs")
    receipt = policy.get("receipt_contract", {})
    if (
        receipt.get("sudo_group_membership") != SUDO_GROUP_MEMBERSHIP_CONTRACT
        or receipt.get("sudo_cleanup_identity") != SUDO_CLEANUP_IDENTITY_CONTRACT
        or receipt.get("sudo_cleanup_bounding_mode") != SUDO_CLEANUP_BOUNDING_MODE
        or receipt.get("creation") != "O_EXCL_O_NOFOLLOW_ONE_SHOT"
        or receipt.get("file_owner") != [HOST_UID, HOST_GID]
        or receipt.get("file_mode") != "0440"
    ):
        raise SupervisorError("receipt or sudo cleanup policy differs")

    reviewed = policy.get("reviewed_bytes", {})
    reviewed_frozen = (
        reviewed.get("all_frozen") is True
        and reviewed.get("helper", {}).get("size_bytes", 0) > 0
        and reviewed.get("loader", {}).get("size_bytes", 0) > 0
        and reviewed.get("payload_frame", {}).get("size_bytes", 0) > 0
        and all(
            isinstance(reviewed.get(name, {}).get("sha256"), str)
            and re.fullmatch(r"[0-9a-f]{64}", reviewed[name]["sha256"]) is not None
            and reviewed[name]["sha256"] != "0" * 64
            for name in ("helper", "loader", "payload_frame")
        )
        and reviewed.get("payload_frame", {}).get("maximum_size_bytes") == 65_536
    )
    if reviewed_frozen is not frozen_ready:
        raise SupervisorError("reviewed payload-byte freeze state differs")

    bootstrap = policy.get("snapshot_bootstrap", {})
    bootstrap_frozen = (
        bootstrap.get("all_frozen") is True
        and bootstrap.get("source_size_bytes") == len(SNAPSHOT_BOOTSTRAP_BYTES)
        and bootstrap.get("source_sha256") == SNAPSHOT_BOOTSTRAP_SHA256
        and bootstrap.get("loader_size_bytes") == len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES)
        and bootstrap.get("loader_sha256") == SNAPSHOT_BOOTSTRAP_LOADER_SHA256
    )
    if bootstrap_frozen is not frozen_ready:
        raise SupervisorError("snapshot bootstrap freeze state differs")
    if (
        bootstrap.get("manifest_artifacts") != ["gate", "supervisor", "profile", "schema", "policy"]
        or bootstrap.get("snapshot_owner") != [0, 0]
        or bootstrap.get("snapshot_mode") != "0555"
        or bootstrap.get("file_owner") != [0, 0]
        or bootstrap.get("file_mode") != "0444"
        or bootstrap.get("root_loader_c_source_contract")
        != "reviewed_hash_pinned_literal_single_argv_token_after_-c"
        or bootstrap.get("producer_stdout_use")
        != "stdin_of_the_reviewed_hash_pinned_root_loader_only"
        or bootstrap.get("workspace_snapshot_producer_root_forbidden") is not True
        or bootstrap.get("workspace_snapshot_producer_exact_path") != str(WORKSPACE_SUPERVISOR_PATH)
        or bootstrap.get("workspace_snapshot_producer_uid") != [HOST_UID] * 4
        or bootstrap.get("workspace_snapshot_producer_gid") != [HOST_GID] * 4
    ):
        raise SupervisorError("snapshot bootstrap boundary differs")

    artifacts = policy.get("trusted_artifacts", {})
    if set(artifacts) != {"all_frozen", "gate", "supervisor", "profile", "schema"}:
        raise SupervisorError("trusted runtime artifact set differs")
    observed: dict[str, dict[str, Any]] = {}
    artifacts_frozen = artifacts.get("all_frozen") is True
    for name in ("gate", "supervisor", "profile", "schema"):
        raw = read_regular_bytes(REPOSITORY_PATHS[name])
        entry = artifacts[name]
        path_matches = entry.get("path") == str(REPOSITORY_PATHS[name])
        bytes_match = entry.get("sha256") == sha256_bytes(raw) and entry.get("size_bytes") == len(raw)
        if not path_matches or bytes_match is not frozen_ready:
            raise SupervisorError(f"trusted artifact freeze state differs: {name}")
        observed[name] = {
            "path": str(REPOSITORY_PATHS[name]),
            "sha256": sha256_bytes(raw),
            "size_bytes": len(raw),
        }
    if artifacts_frozen is not frozen_ready:
        raise SupervisorError("trusted artifact aggregate freeze state differs")

    policy_raw = read_regular_bytes(REPOSITORY_PATHS["policy"])
    return {
        "policy_sha256": sha256_bytes(policy_raw),
        "policy_size_bytes": len(policy_raw),
        "artifacts": observed,
        "profile": profile,
        "v6_failure": v6,
        "v1_static_no_go": v1_no_go,
        "snapshot_bootstrap_sha256": SNAPSHOT_BOOTSTRAP_SHA256,
        "freeze_ready": frozen_ready,
        "execution_performed": False,
    }


def snapshot_manifest_arguments(review: Mapping[str, Any]) -> list[str]:
    arguments: list[str] = []
    for name in ("gate", "supervisor", "profile", "schema"):
        entry = review["artifacts"][name]
        arguments.extend((entry["sha256"], str(entry["size_bytes"])))
    arguments.extend((review["policy_sha256"], str(review["policy_size_bytes"])))
    return arguments


def verify_snapshot_producer_identity() -> dict[str, Any]:
    """Forbid root or non-canonical workspace execution of the byte producer."""

    if SCRIPT_PATH != WORKSPACE_SUPERVISOR_PATH:
        raise SupervisorError("snapshot producer must be the exact workspace proc-stat v2 supervisor")
    if (os.getuid(), os.geteuid(), os.getgid(), os.getegid()) != (HOST_UID, HOST_UID, HOST_GID, HOST_GID):
        raise SupervisorError("snapshot producer must never execute as root or another identity")
    fields: dict[str, str] = {}
    for line in Path("/proc/self/status").read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    try:
        uid = [int(value) for value in fields["Uid"].split()]
        gid = [int(value) for value in fields["Gid"].split()]
        capabilities = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapAmb")}
    except (KeyError, ValueError) as exc:
        raise SupervisorError("cannot parse snapshot producer identity") from exc
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or any(capabilities.values()):
        raise SupervisorError("snapshot producer UID/GID or active capability sets differ")
    return {"uid": uid, "gid": gid, "active_capabilities": capabilities, "root_forbidden": True, "script_path": str(SCRIPT_PATH)}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def require_tool(name: str) -> dict[str, Any]:
    path, expected = TRUSTED_TOOLS[name]
    raw = read_regular_bytes(path, limit=512 * 1024 * 1024)
    metadata = os.stat(path, follow_symlinks=False)
    if metadata.st_uid != 0 or metadata.st_gid != 0 or not metadata.st_mode & stat.S_IXUSR or sha256_bytes(raw) != expected:
        raise SupervisorError(f"trusted tool identity differs: {path}")
    return {"path": str(path), "sha256": expected, "size_bytes": len(raw)}


def verify_snapshot(policy_sha256: str) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    if os.geteuid() != 0 or SCRIPT_PATH != SNAPSHOT_PATHS["supervisor"]:
        raise SupervisorError("run requires the exact snapshot supervisor with euid 0")
    root = os.lstat(SNAPSHOT_ROOT)
    if (
        not stat.S_ISDIR(root.st_mode)
        or root.st_uid != 0
        or root.st_gid != 0
        or root.st_nlink != 2
        or stat.S_IMODE(root.st_mode) != 0o555
    ):
        raise SupervisorError("snapshot root ownership, link count, or mode differs")
    policy_raw = read_regular_bytes(SNAPSHOT_PATHS["policy"])
    if (
        not isinstance(policy_sha256, str)
        or re.fullmatch(r"[0-9a-f]{64}", policy_sha256) is None
        or sha256_bytes(policy_raw) != policy_sha256
    ):
        raise SupervisorError("externally frozen snapshot policy digest differs")
    try:
        policy = json.loads(
            policy_raw.decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, "snapshot policy"),
        )
        schema = json.loads(
            read_regular_bytes(SNAPSHOT_PATHS["schema"]).decode("utf-8"),
            object_pairs_hook=lambda pairs: _strict_pairs(pairs, "snapshot schema"),
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("snapshot policy or schema JSON differs") from exc
    if not isinstance(policy, dict) or not isinstance(schema, dict):
        raise SupervisorError("snapshot policy or schema root differs")
    _assert_schema_objects_closed(schema)
    _validate_schema_instance(policy, schema, schema)
    if set(policy) != set(schema.get("required", [])):
        raise SupervisorError("snapshot policy top-level key set differs")
    if (
        policy.get("status") != "STATIC_READY_V2_PENDING_INDEPENDENT_REVIEW_AND_EXPLICIT_SINGLE_ATTEMPT"
        or policy.get("next_allowed_stage")
        != "INDEPENDENT_STATIC_REVIEW_ONLY_THEN_EXPLICIT_ROOT_SNAPSHOT_SINGLE_ATTEMPT"
        or policy.get("authorization", {}).get("admission_token") != ADMISSION_TOKEN
        or policy.get("authorization", {}).get("attempts_per_identity") != 1
        or policy.get("authorization", {}).get("same_identity_retry") != "forbidden"
        or policy.get("authorization", {}).get("execution_authorized_now") is not False
        or policy.get("authorization", {}).get("public_token_is_authority") is not False
        or policy.get("authorization", {}).get("root_owned_fd_capability_required") is not True
    ):
        raise SupervisorError("snapshot policy is not the frozen fresh one-shot identity")
    if policy.get("frozen_identity") != {
        "probe_id": PROBE_ID,
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        **FROZEN_RECEIPT_PATHS,
    }:
        raise SupervisorError("snapshot frozen identity differs")
    if policy.get("provenance") != {
        "cold_a_v6_failure": FROZEN_V6_FAILURE,
        "proc_stat_v1_static_no_go": FROZEN_V1_NO_GO,
    }:
        raise SupervisorError("snapshot v6 and v1 NO-GO provenance differs")
    verify_v6_failure_provenance()
    verify_v1_no_go_provenance()

    artifacts = policy.get("trusted_artifacts", {})
    if artifacts.get("all_frozen") is not True:
        raise SupervisorError("snapshot trusted artifacts are not frozen")
    observed: dict[str, dict[str, Any]] = {}
    if set(SNAPSHOT_PATHS) != {"gate", "supervisor", "profile", "schema", "policy"}:
        raise SupervisorError("snapshot artifact path set differs")
    for name, path in SNAPSHOT_PATHS.items():
        metadata = os.lstat(path)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o444
        ):
            raise SupervisorError(f"snapshot artifact metadata differs: {name}")
        raw = read_regular_bytes(path)
        if name == "policy":
            expected_digest, expected_size = policy_sha256, len(policy_raw)
        else:
            entry = artifacts.get(name)
            if not isinstance(entry, dict) or entry.get("path") != str(REPOSITORY_PATHS[name]):
                raise SupervisorError(f"snapshot artifact policy path differs: {name}")
            expected_digest, expected_size = entry.get("sha256"), entry.get("size_bytes")
        if sha256_bytes(raw) != expected_digest or len(raw) != expected_size:
            raise SupervisorError(f"snapshot artifact bytes differ: {name}")
        observed[name] = {
            "path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw),
            "uid": metadata.st_uid, "gid": metadata.st_gid, "mode": "0444",
        }

    verify_profile_bytes(SNAPSHOT_PATHS["profile"])
    if policy.get("evidence_classification") != EVIDENCE_CLASSIFICATION_CONTRACT:
        raise SupervisorError("snapshot status-denial evidence contract differs")
    reviewed = policy.get("reviewed_bytes", {})
    bootstrap = policy.get("snapshot_bootstrap", {})
    commands = policy.get("fixed_commands", {})
    if (
        reviewed.get("all_frozen") is not True
        or bootstrap.get("all_frozen") is not True
        or bootstrap.get("source_sha256") != SNAPSHOT_BOOTSTRAP_SHA256
        or bootstrap.get("source_size_bytes") != len(SNAPSHOT_BOOTSTRAP_BYTES)
        or bootstrap.get("loader_sha256") != SNAPSHOT_BOOTSTRAP_LOADER_SHA256
        or bootstrap.get("loader_size_bytes") != len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES)
        or commands.get("argv_hashes_frozen") is not True
        or commands.get("gate_handoff_argv") != gate_handoff_argv()
    ):
        raise SupervisorError("snapshot reviewed-byte or fixed-command freeze differs")
    return policy, observed


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


def _signal_open_pidfd(descriptor: int, sig: signal.Signals) -> bool:
    try:
        signal.pidfd_send_signal(descriptor, sig)
        return True
    except ProcessLookupError:
        return False


def _signal_pidfd(pid: int, sig: signal.Signals, *, expected_starttime: int) -> bool:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise SupervisorError("pidfd signaling is unavailable")
    if not isinstance(expected_starttime, int) or isinstance(expected_starttime, bool) or expected_starttime < 1:
        raise SupervisorError("refusing to signal a task without a frozen positive starttime")
    try:
        descriptor = os.pidfd_open(pid, 0)
    except ProcessLookupError:
        return False
    try:
        if _proc_starttime(pid) != expected_starttime:
            return False
        return _signal_open_pidfd(descriptor, sig)
    finally:
        os.close(descriptor)


def _bounded_extend(target: bytearray, block: bytes, limit: int) -> bool:
    remaining = max(0, limit - len(target))
    target.extend(block[:remaining])
    return len(block) > remaining


def run_bounded_command(
    argv: list[str],
    *,
    stdin_bytes: bytes | None = b"",
    stdout_limit: int = 65_536,
    stderr_limit: int = 65_536,
    timeout_seconds: float = 30,
    env: Mapping[str, str] | None = None,
    pass_fds: tuple[int, ...] = (),
    start_new_session: bool = True,
) -> dict[str, Any]:
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise SupervisorError("pidfd signaling is unavailable")
    process = subprocess.Popen(
        argv,
        stdin=None if stdin_bytes is None else subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env=dict(env or {"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"}),
        close_fds=True,
        pass_fds=pass_fds,
        start_new_session=start_new_session,
    )
    try:
        leader_pidfd = os.pidfd_open(process.pid, 0)
    except BaseException:
        # The direct child is still unreaped, so its numeric PID cannot be
        # reused while this birth-time pidfd fallback kills and reaps it.
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
    streams: dict[str, Any] = {}
    failure: str | None = None
    cleanup_failure: str | None = None
    deadline = time.monotonic() + timeout_seconds
    try:
        if process.stdout is None or process.stderr is None or (stdin_bytes is not None and process.stdin is None):
            raise SupervisorError("bounded command pipes are unavailable")
        selector = selectors.DefaultSelector()
        streams = {"stdout": process.stdout, "stderr": process.stderr}
        for name, stream in streams.items():
            os.set_blocking(stream.fileno(), False)
            selector.register(stream.fileno(), selectors.EVENT_READ, name)
        if stdin_bytes is not None:
            os.set_blocking(process.stdin.fileno(), False)
            selector.register(process.stdin.fileno(), selectors.EVENT_WRITE, "stdin")
        while selector.get_map():
            if time.monotonic() >= deadline:
                failure = failure or "deadline"
                break
            events = selector.select(timeout=0.1)
            for key, _mask in events:
                channel = key.data
                descriptor = key.fd
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, stdin_bytes[position:position + 4096])
                    except (BrokenPipeError, ConnectionResetError):
                        count = 0
                    except BlockingIOError:
                        continue
                    if count:
                        position += count
                    if not count or position == len(stdin_bytes):
                        selector.unregister(descriptor)
                        process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 4096)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    streams[channel].close()
                    continue
                target = stdout if channel == "stdout" else stderr
                limit = stdout_limit if channel == "stdout" else stderr_limit
                if _bounded_extend(target, block, limit):
                    failure = f"{channel}_overflow"
                    break
            if failure is not None:
                break
        if failure is not None:
            _signal_open_pidfd(leader_pidfd, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
    finally:
        if selector is not None:
            selector.close()
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
        if process.poll() is None:
            try:
                process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                failure = failure or "leader_alive_after_pipe_close"
                try:
                    _signal_open_pidfd(leader_pidfd, signal.SIGKILL)
                except OSError as exc:
                    cleanup_failure = f"final leader pidfd kill failed: {exc}"
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            cleanup_failure = cleanup_failure or "bounded command leader remained unreaped"
        os.close(leader_pidfd)
        if cleanup_failure is not None:
            raise SupervisorError(cleanup_failure)
    returncode = process.returncode
    if returncode is None:
        raise SupervisorError("bounded command return code is unavailable after reap")
    return {
        "argv": argv,
        "returncode": returncode,
        "stdout": bytes(stdout),
        "stderr": bytes(stderr),
        "stdin_size_bytes": None if stdin_bytes is None else len(stdin_bytes),
        "stdin_fully_written": stdin_bytes is None or position == len(stdin_bytes),
        "failure": failure,
        "start_new_session": start_new_session,
    }


def _command_evidence(result: Mapping[str, Any], *, include_prefix: bool = False) -> dict[str, Any]:
    stdout = result["stdout"]
    stderr = result["stderr"]
    evidence = {
        "argv": result["argv"],
        "returncode": result["returncode"],
        "stdout_size_bytes": len(stdout),
        "stdout_sha256": sha256_bytes(stdout),
        "stderr_size_bytes": len(stderr),
        "stderr_sha256": sha256_bytes(stderr),
        "failure": result["failure"],
        "start_new_session": result["start_new_session"],
    }
    if include_prefix:
        evidence["stderr_utf8_prefix"] = stderr[:4096].decode("utf-8", "replace")
    return evidence


def run_checked(
    argv: list[str],
    *,
    timeout_seconds: float = 30,
    stdout_limit: int = 65_536,
    stderr_limit: int = 65_536,
    require_silent: bool = False,
) -> dict[str, Any]:
    result = run_bounded_command(argv, timeout_seconds=timeout_seconds, stdout_limit=stdout_limit, stderr_limit=stderr_limit)
    if (result["returncode"] != 0 or result["failure"] is not None
            or (require_silent and (result["stdout"] or result["stderr"]))):
        raise SupervisorError(f"trusted command failed: {argv[0]}")
    return result


def _read_proc_file(proc_root: Path, pid: int, relative: str, *, limit: int = 4096) -> bytes:
    directory = os.open(proc_root / str(pid), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        descriptor = os.open(relative, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory)
        try:
            result = bytearray()
            while True:
                block = os.read(descriptor, min(4096, limit + 1 - len(result)))
                if not block:
                    return bytes(result)
                result.extend(block)
                if len(result) > limit:
                    raise SupervisorError(f"proc file exceeds ceiling: {pid}/{relative}")
        finally:
            os.close(descriptor)
    finally:
        os.close(directory)


def _stat_starttime(stat_raw: str, *, identity: str) -> int:
    end = stat_raw.rfind(")")
    fields = stat_raw[end + 2:].split() if end >= 0 else []
    if len(fields) < 20:
        raise SupervisorError(f"cannot parse proc starttime for {identity}")
    try:
        starttime = int(fields[19])
    except ValueError as exc:
        raise SupervisorError(f"cannot parse proc starttime for {identity}") from exc
    if starttime < 1:
        raise SupervisorError(f"non-positive proc starttime for {identity}")
    return starttime


def _task_record(proc_root: Path, tgid: int, tid: int) -> dict[str, Any]:
    attr = _read_proc_file(proc_root, tgid, f"task/{tid}/attr/current").decode("utf-8")
    task_stat = _read_proc_file(proc_root, tgid, f"task/{tid}/stat").decode("ascii")
    leader_stat = _read_proc_file(proc_root, tgid, "stat").decode("ascii")
    return {
        "tgid": tgid,
        "tid": tid,
        "label": attr.strip().split(" (", 1)[0],
        "attr_current": attr.strip(),
        "tgid_starttime": _stat_starttime(leader_stat, identity=f"tgid {tgid}"),
        "tid_starttime": _stat_starttime(task_stat, identity=f"tgid {tgid} tid {tid}"),
    }


def labeled_processes(proc_root: Path = Path("/proc")) -> list[dict[str, Any]]:
    """Authoritatively enumerate every task/thread carrying a fresh v2 label."""

    observed: list[dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            task_entries = tuple((entry / "task").iterdir())
        except (FileNotFoundError, ProcessLookupError) as exc:
            if isinstance(exc, OSError) and exc.errno not in (None, errno.ENOENT, errno.ESRCH):
                raise
            continue
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ESRCH):
                continue
            raise SupervisorError(f"cannot enumerate authoritative tasks for tgid {entry.name}: {exc}") from exc
        tgid = int(entry.name)
        for task_entry in task_entries:
            if not task_entry.name.isdecimal():
                continue
            tid = int(task_entry.name)
            try:
                record = _task_record(proc_root, tgid, tid)
            except (FileNotFoundError, ProcessLookupError):
                continue
            except OSError as exc:
                if exc.errno in (errno.ENOENT, errno.ESRCH):
                    continue
                raise SupervisorError(f"cannot read authoritative attr/current for tgid {tgid} tid {tid}: {exc}") from exc
            if record["label"] in LABELS:
                observed.append(record)
    return sorted(observed, key=lambda item: (item["tgid"], item["tid"]))


def require_stable_zero_labels(*, scans: int = 3, interval: float = 0.1) -> list[list[dict[str, Any]]]:
    if scans != 3:
        raise SupervisorError("exactly three stable-zero scans are required")
    history: list[list[dict[str, Any]]] = []
    for index in range(scans):
        current = labeled_processes()
        history.append(current)
        if current:
            raise SupervisorError(f"labeled process residue remains: {current}")
        if index + 1 < scans:
            time.sleep(interval)
    return history


def _signal_labeled(records: Iterable[Mapping[str, Any]], sig: signal.Signals) -> list[dict[str, Any]]:
    sent: list[dict[str, Any]] = []
    signaled_tgids: set[int] = set()
    for item in records:
        tgid = int(item["tgid"])
        tid = int(item["tid"])
        if tgid in signaled_tgids:
            continue
        tgid_starttime = int(item["tgid_starttime"])
        tid_starttime = int(item["tid_starttime"])
        try:
            current = _task_record(Path("/proc"), tgid, tid)
        except (FileNotFoundError, ProcessLookupError):
            continue
        if (current["tgid_starttime"] != tgid_starttime or current["tid_starttime"] != tid_starttime
                or current["label"] not in LABELS):
            continue
        if _signal_pidfd(tgid, sig, expected_starttime=tgid_starttime):
            signaled_tgids.add(tgid)
            sent.append({
                "tgid": tgid,
                "trigger_tid": tid,
                "tgid_starttime": tgid_starttime,
                "trigger_tid_starttime": tid_starttime,
                "label": current["label"],
                "signal": int(sig),
            })
    return sent


def terminate_labeled_processes() -> dict[str, Any]:
    initial = labeled_processes()
    term_sent = _signal_labeled(initial, signal.SIGTERM)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if not labeled_processes():
            break
        time.sleep(0.1)
    after_term = labeled_processes()
    kill_sent = _signal_labeled(after_term, signal.SIGKILL)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if not labeled_processes():
            break
        time.sleep(0.1)
    stable = require_stable_zero_labels()
    return {"initial": initial, "term_sent": term_sent, "after_term": after_term, "kill_sent": kill_sent, "stable_zero_scans": stable}


def read_sysctls() -> dict[str, dict[str, str]]:
    result: dict[str, dict[str, str]] = {}
    for path in SYSCTL_PATHS:
        raw = read_regular_bytes(path, limit=128)
        try:
            value = raw.decode("ascii")
        except UnicodeError as exc:
            raise SupervisorError(f"sysctl is not ASCII: {path}") from exc
        result[str(path)] = {"value": value, "sha256": sha256_bytes(raw)}
    if set(result) != {str(path) for path in SYSCTL_PATHS}:
        raise SupervisorError("exact three-sysctl snapshot differs")
    return result


def read_kernel_profile_entries() -> dict[str, list[str]]:
    raw = read_regular_bytes(KERNEL_PROFILE_PATH, limit=8 * 1024 * 1024)
    entries: dict[str, list[str]] = {label: [] for label in LABELS}
    for line in raw.decode("utf-8", "strict").splitlines():
        name = line.split(" (", 1)[0]
        if name in entries:
            entries[name].append(line)
    return entries


def read_kernel_profile_counts() -> dict[str, int]:
    return {label: len(lines) for label, lines in read_kernel_profile_entries().items()}


def profile_state() -> dict[str, Any]:
    entries = read_kernel_profile_entries()
    counts = {label: len(lines) for label, lines in entries.items()}
    enforce = {label: lines == [f"{label} (enforce)"] for label, lines in entries.items()}
    result = run_bounded_command(list(AA_STATUS_JSON_ARGV), stdout_limit=2 * 1024 * 1024, stderr_limit=16_384, timeout_seconds=15)
    if result["returncode"] != 0 or result["failure"] is not None or result["stderr"]:
        raise SupervisorError("aa-status profile query failed")
    try:
        parsed = json.loads(result["stdout"].decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("aa-status JSON differs") from exc
    if type(parsed) is not dict or type(parsed.get("profiles")) is not dict:
        raise SupervisorError("aa-status JSON root/profiles object differs")
    profiles = parsed["profiles"]
    if any(type(name) is not str or type(mode) is not str for name, mode in profiles.items()):
        raise SupervisorError("aa-status JSON profile mapping differs")
    presence = {label: label in profiles for label in LABELS}
    modes = {label: profiles.get(label) for label in LABELS}
    for label in LABELS:
        if presence[label] != (counts[label] > 0):
            raise SupervisorError(f"aa-status/kernel profile state disagrees: {label}")
        if presence[label] and modes[label] != "enforce":
            raise SupervisorError(f"aa-status profile mode is not exact enforce: {label}")
    return {
        "kernel_exact_counts": counts,
        "kernel_exact_lines": entries,
        "kernel_exact_enforce": enforce,
        "aa_status_exact_presence": presence,
        "aa_status_exact_modes": modes,
        "aa_status_stdout_sha256": sha256_bytes(result["stdout"]),
        "aa_status_stdout_size_bytes": len(result["stdout"]),
    }


def require_profile_counts(state: Mapping[str, Any], expected: int) -> None:
    counts = state.get("kernel_exact_counts", {})
    lines = state.get("kernel_exact_lines", {})
    enforce = state.get("kernel_exact_enforce", {})
    presence = state.get("aa_status_exact_presence", {})
    modes = state.get("aa_status_exact_modes", {})
    if any(counts.get(label) != expected for label in LABELS):
        raise SupervisorError(f"exact profile count differs from {expected}")
    if expected == 1 and any(lines.get(label) != [f"{label} (enforce)"] or enforce.get(label) is not True for label in LABELS):
        raise SupervisorError("loaded profile mode is not exact enforce")
    if expected == 0 and any(lines.get(label) != [] for label in LABELS):
        raise SupervisorError("unloaded profile lines are not empty")
    if any(presence.get(label) is not (expected > 0) for label in LABELS):
        raise SupervisorError(f"aa-status profile presence differs from {expected > 0}")
    expected_mode = "enforce" if expected == 1 else None
    if any(modes.get(label) != expected_mode for label in LABELS):
        raise SupervisorError(f"aa-status profile mode differs from {expected_mode!r}")


def _open_absolute_directory(path: Path) -> int:
    if not path.is_absolute() or path != AUDIT_ROOT:
        raise SupervisorError("receipt directory must be the exact frozen audit root")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        current = Path("/")
        metadata = os.fstat(descriptor)
        expected = AUDIT_DIRECTORY_POLICY[current]
        if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != expected[0]
                or metadata.st_gid != expected[1] or stat.S_IMODE(metadata.st_mode) != expected[2]):
            raise SupervisorError(f"receipt directory component metadata differs: {current}")
        for part in path.parts[1:]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
            current /= part
            expected = AUDIT_DIRECTORY_POLICY.get(current)
            if expected is None:
                raise SupervisorError(f"receipt directory component is outside frozen policy: {current}")
            metadata = os.fstat(descriptor)
            if (not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != expected[0]
                    or metadata.st_gid != expected[1] or stat.S_IMODE(metadata.st_mode) != expected[2]):
                raise SupervisorError(f"receipt directory component metadata differs: {current}")
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> dict[str, Any]:
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    directory = _open_absolute_directory(path.parent)
    try:
        descriptor = os.open(path.name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o400, dir_fd=directory)
        try:
            view = memoryview(raw)
            while view:
                count = os.write(descriptor, view)
                if count <= 0:
                    raise SupervisorError(f"short one-shot O_EXCL write: {path.name}")
                view = view[count:]
            os.fsync(descriptor)
            os.fchown(descriptor, HOST_UID, HOST_GID)
            os.fchmod(descriptor, mode)
            os.fsync(descriptor)
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != mode:
                raise SupervisorError(f"append-only receipt inode differs: {path.name}")
        finally:
            os.close(descriptor)
        os.fsync(directory)
    finally:
        os.close(directory)
    return {
        "path": str(path),
        "sha256": sha256_bytes(raw),
        "size_bytes": len(raw),
        "uid": HOST_UID,
        "gid": HOST_GID,
        "mode": f"{mode:04o}",
        "creation": "O_EXCL_NOFOLLOW_ONE_SHOT_NOT_IMMUTABLE_PARENT_OWNER_CAN_REMOVE",
        "file_and_parent_fsynced": True,
    }


def _assert_one_shot_paths_absent() -> None:
    for path in (
        START_RECEIPT,
        PREFLIGHT_FAILURE_RECEIPT,
        EXECUTION_RECEIPT,
        LIFECYCLE_RECEIPT,
        LIFECYCLE_FAILURE_RECEIPT,
        RECOVERY_RECEIPT,
    ):
        try:
            os.lstat(path)
        except FileNotFoundError:
            continue
        raise SupervisorError(f"one-shot evidence path already exists: {path}")


def parse_success_frame(frame: bytes) -> dict[str, Any]:
    prefix_size = len(SUCCESS_MAGIC)
    payload_offset = prefix_size + 4
    if not payload_offset + 2 + 32 <= len(frame) <= MAX_SUCCESS_FRAME_BYTES or not frame.startswith(SUCCESS_MAGIC):
        raise SupervisorError("success frame size or magic differs")
    size = struct.unpack(">I", frame[prefix_size:payload_offset])[0]
    if size < 2 or payload_offset + size + 32 != len(frame):
        raise SupervisorError("success frame declared length differs")
    payload = frame[payload_offset:payload_offset + size]
    if hashlib.sha256(payload).digest() != frame[-32:]:
        raise SupervisorError("success frame digest differs")
    try:
        value = json.loads(payload.decode("ascii"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("success frame JSON differs") from exc
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("ascii")
    if canonical != payload or not isinstance(value, dict):
        raise SupervisorError("success frame is not canonical")
    expected_keys = {
        "status", "bootstrap_profile", "bootstrap_label", "bootstrap_identity",
        "work_tmpfs", "host_payload_consumed_and_fd0_replaced_with_eof_pipe",
        "fds_after_payload", "proc_stat_probe", "fds_before_success",
        "host_writable_mounts",
    }
    if set(value) != expected_keys:
        raise SupervisorError("success frame key set differs")
    if value["status"] != "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2":
        raise SupervisorError("success status differs")
    if value["bootstrap_profile"] != BOOTSTRAP_PROFILE or value["bootstrap_label"] != BOOTSTRAP_PROFILE + " (enforce)":
        raise SupervisorError("bootstrap label evidence differs")
    if value["host_writable_mounts"] != [] or value["host_payload_consumed_and_fd0_replaced_with_eof_pipe"] is not True:
        raise SupervisorError("success isolation evidence differs")
    if (type(value["fds_after_payload"]) is not list or type(value["fds_before_success"]) is not list
            or any(type(descriptor) is not int for descriptor in value["fds_after_payload"] + value["fds_before_success"])
            or value["fds_after_payload"] != [0, 1, 2] or value["fds_before_success"] != [0, 1, 2]):
        raise SupervisorError("success descriptor evidence differs")

    def require_guest_identity(observed: Any, context: str) -> None:
        if not isinstance(observed, dict) or set(observed) != {"uid", "gid", "groups", "capabilities", "no_new_privs"}:
            raise SupervisorError(f"{context} identity key set differs")
        uid, gid, groups = observed["uid"], observed["gid"], observed["groups"]
        if (type(uid) is not list or type(gid) is not list or type(groups) is not list
                or any(type(item) is not int for item in uid + gid + groups)
                or uid != [0, 0, 0, 0] or gid != [0, 0, 0, 0] or groups != []):
            raise SupervisorError(f"{context} identity differs")
        capabilities = observed["capabilities"]
        cap_keys = {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        if (type(capabilities) is not dict or set(capabilities) != cap_keys
                or any(type(capabilities[key]) is not int or capabilities[key] != 0 for key in cap_keys)):
            raise SupervisorError(f"{context} capability evidence differs")
        if type(observed["no_new_privs"]) is not int or observed["no_new_privs"] != 1:
            raise SupervisorError(f"{context} NNP evidence differs")

    require_guest_identity(value["bootstrap_identity"], "bootstrap")
    work_tmpfs = value["work_tmpfs"]
    if not isinstance(work_tmpfs, dict) or set(work_tmpfs) != {"filesystem_type", "total_bytes", "mount_options"}:
        raise SupervisorError("success tmpfs key set differs")
    options = work_tmpfs["mount_options"]
    if (work_tmpfs["filesystem_type"] != "tmpfs" or work_tmpfs["total_bytes"] != TMPFS_BYTES
            or isinstance(work_tmpfs["total_bytes"], bool) or not isinstance(options, list)
            or any(not isinstance(option, str) for option in options) or len(options) != len(set(options))
            or not {"rw", "nosuid", "nodev"}.issubset(set(options))):
        raise SupervisorError("success tmpfs evidence differs")
    probe = value["proc_stat_probe"]
    expected_probe_keys = {
        "child_identity", "frozen_identity", "live_group_scan",
        "live_stat_pair", "inner_proc_owner_projection", "status_denial",
        "waitid_wnowait", "zombie_stat_pair", "final_reap",
        "post_reap_group_scans",
    }
    if not isinstance(probe, dict) or set(probe) != expected_probe_keys:
        raise SupervisorError("proc-stat probe evidence key set differs")
    child = probe["child_identity"]
    child_extra = {"pid", "pgrp", "session", "dumpable"}
    if not isinstance(child, dict) or set(child) != {
        "uid", "gid", "groups", "capabilities", "no_new_privs", *child_extra
    }:
        raise SupervisorError("dumpable-zero child identity key set differs")
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
        raise SupervisorError("dumpable-zero child process identity differs")
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
        raise SupervisorError("frozen child birth identity differs")
    owner = probe["inner_proc_owner_projection"]
    if (
        not isinstance(owner, dict)
        or set(owner) != {"uid", "gid"}
        or type(owner["uid"]) is not int
        or type(owner["gid"]) is not int
        or owner["uid"] < 1
        or owner["gid"] < 1
    ):
        raise SupervisorError("inner proc owner projection differs")

    member_keys = {"pid", "state", "pgrp", "session", "starttime", "stat_uid", "stat_gid"}

    def require_member(member: Any, *, zombie: bool) -> None:
        if not isinstance(member, dict) or set(member) != member_keys:
            raise SupervisorError("proc-stat member key set differs")
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
            raise SupervisorError("proc-stat member identity, owner, or state differs")

    live_pair = probe["live_stat_pair"]
    zombie_pair = probe["zombie_stat_pair"]
    if not isinstance(live_pair, list) or len(live_pair) != 2:
        raise SupervisorError("live stat pair count differs")
    if not isinstance(zombie_pair, list) or len(zombie_pair) != 2:
        raise SupervisorError("zombie stat pair count differs")
    for member in live_pair:
        require_member(member, zombie=False)
    for member in zombie_pair:
        require_member(member, zombie=True)
    if live_pair[0] != live_pair[1] or zombie_pair[0] != zombie_pair[1]:
        raise SupervisorError("live or zombie stat double-read is unstable")
    group = probe["live_group_scan"]
    if not isinstance(group, list) or len(group) != 1:
        raise SupervisorError("live process group must contain exactly the isolated child")
    require_member(group[0], zombie=False)
    if group[0] != live_pair[0]:
        raise SupervisorError("full group scan and live stat reads differ")
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
        raise SupervisorError("single child status EACCES evidence differs")
    expected_wait = {"pid": pid, "code": os.CLD_EXITED, "status": 0}
    for name in ("waitid_wnowait", "final_reap"):
        wait = probe[name]
        if (
            not isinstance(wait, dict)
            or set(wait) != {"pid", "code", "status"}
            or any(type(wait[key]) is not int for key in ("pid", "code", "status"))
            or wait != expected_wait
        ):
            raise SupervisorError("WNOWAIT or final reap evidence differs")
    if probe["post_reap_group_scans"] != [[], []]:
        raise SupervisorError("post-reap process group is not double-empty")
    return value



def read_boot_id() -> str:
    raw = BOOT_ID_PATH.read_bytes()
    if len(raw) != 37 or not raw.endswith(b"\n"):
        raise SupervisorError("kernel boot-id framing differs")
    try:
        value = raw[:-1].decode("ascii", "strict")
    except UnicodeError as exc:
        raise SupervisorError("kernel boot-id is not strict ASCII") from exc
    if BOOT_ID_RE.fullmatch(value) is None:
        raise SupervisorError("kernel boot-id syntax differs")
    return value


def _parse_cursor_output(raw: bytes) -> str:
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError as exc:
        raise SupervisorError("journal cursor output is not strict UTF-8") from exc
    lines = text.splitlines()
    if len(lines) != 1 or not lines[0].startswith("-- cursor: "):
        raise SupervisorError("journal cursor framing differs")
    cursor = lines[0][11:]
    if JOURNAL_CURSOR_RE.fullmatch(cursor) is None:
        raise SupervisorError("journal cursor syntax differs")
    return cursor


def _cursor_matches_boot(cursor: str, boot_id: str) -> bool:
    parts = cursor.split(";")
    fields: dict[str, str] = {}
    for part in parts:
        if part.count("=") != 1:
            return False
        key, value = part.split("=", 1)
        if not key or not value or key in fields:
            return False
        fields[key] = value
    return fields.get("b") == boot_id.replace("-", "")


def capture_journal_anchor() -> dict[str, Any]:
    boot_id = read_boot_id()
    compact_boot_id = boot_id.replace("-", "")
    argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet", f"--boot={compact_boot_id}",
        "--lines=0", "--show-cursor",
    ]
    result = run_bounded_command(
        argv, stdout_limit=4_096, stderr_limit=4_096, timeout_seconds=15,
    )
    if (type(result.get("returncode")) is not int or result["returncode"] != 0
            or result.get("failure") is not None or result.get("stderr") != b""):
        raise SupervisorError("bounded pre-run journal cursor query failed")
    cursor = _parse_cursor_output(result["stdout"])
    if not _cursor_matches_boot(cursor, boot_id):
        raise SupervisorError("journal cursor boot identity differs")
    return {
        "boot_id": boot_id,
        "cursor": cursor,
        "journal_anchor_argv": argv,
        "raw_stdout_sha256": sha256_bytes(result["stdout"]),
        "raw_stdout_size_bytes": len(result["stdout"]),
    }


def _decode_quoted_audit_value(raw: str) -> str:
    if not (raw.startswith('"') and raw.endswith('"')):
        return raw
    body = raw[1:-1]
    output: list[str] = []
    index = 0
    while index < len(body):
        character = body[index]
        if character != "\\":
            output.append(character)
            index += 1
            continue
        index += 1
        if index >= len(body):
            raise ValueError("trailing_escape")
        escaped = body[index]
        if escaped in {'"', "\\"}:
            output.append(escaped)
            index += 1
        elif escaped in {"n", "r", "t"}:
            output.append({"n": "\n", "r": "\r", "t": "\t"}[escaped])
            index += 1
        elif escaped == "x" and index + 2 < len(body) and re.fullmatch(r"[0-9A-Fa-f]{2}", body[index + 1:index + 3]):
            output.append(chr(int(body[index + 1:index + 3], 16)))
            index += 3
        else:
            raise ValueError("illegal_escape")
    value = "".join(output)
    if any(ord(character) < 0x20 or ord(character) == 0x7F for character in value):
        raise ValueError("decoded_control_character")
    return value


def _parse_audit_line(line: str) -> dict[str, Any]:
    raw = line.encode("utf-8")
    entry: dict[str, Any] = {
        "line_sha256": sha256_bytes(raw),
        "line_size_bytes": len(raw),
        "line_truncated": False,
        "sanitized_line": None,
        "parse_status": "UNPARSEABLE",
        "parse_error": None,
        "fields": [],
    }
    if len(raw) > AUDIT_LINE_MAX_BYTES:
        entry["line_truncated"] = True
        entry["parse_error"] = "LINE_TOO_LONG"
        return entry
    sanitized = re.sub(r"\b(pid|ppid|task|peer)=([0-9]+)", r"\1=<redacted>", line)
    if len(sanitized.encode("utf-8")) > AUDIT_LINE_MAX_BYTES:
        entry["line_truncated"] = True
        entry["parse_error"] = "SANITIZED_LINE_TOO_LONG"
        return entry
    entry["sanitized_line"] = sanitized
    start_match = re.search(r"(?<![A-Za-z0-9_])apparmor=", line)
    if start_match is None:
        entry["parse_error"] = "APPARMOR_FIELD_MISSING"
        return entry
    payload = line[start_match.start():]
    position = 0
    seen: set[str] = set()
    fields: list[dict[str, str]] = []
    try:
        while position < len(payload):
            match = AUDIT_TOKEN_RE.match(payload, position)
            if match is None:
                raise ValueError("TOKEN_SYNTAX")
            key, raw_value = match.group(1), match.group(2)
            if key in seen:
                raise ValueError("DUPLICATE_FIELD")
            if len(key) > 64 or len(raw_value.encode("utf-8")) > 1_024 or len(fields) >= 64:
                raise ValueError("FIELD_BOUND_EXCEEDED")
            if key in AUDIT_CRITICAL_FIELDS and "\\" in raw_value:
                raise ValueError("CRITICAL_FIELD_ESCAPE")
            value = _decode_quoted_audit_value(raw_value)
            seen.add(key)
            fields.append({"key": key, "raw_value": raw_value, "value": value})
            position = match.end()
    except ValueError as exc:
        entry["parse_error"] = str(exc)
        entry["fields"] = fields
        return entry
    entry["parse_status"] = "PARSED_UNIQUE_KV"
    entry["fields"] = fields
    return entry


def _entry_fields(entry: Mapping[str, Any]) -> dict[str, str]:
    fields = entry.get("fields")
    if type(fields) is not list:
        return {}
    observed: dict[str, str] = {}
    for field in fields:
        if (type(field) is not dict or set(field) != {"key", "raw_value", "value"}
                or any(type(field.get(key)) is not str for key in ("key", "raw_value", "value"))
                or field["key"] in observed):
            return {}
        observed[field["key"]] = field["value"]
    return observed


def _is_exact_status_denial(entry: Mapping[str, Any], child_pid: int | None) -> bool:
    if (
        type(child_pid) is not int
        or child_pid < 2
        or entry.get("parse_status") != "PARSED_UNIQUE_KV"
        or entry.get("parse_error") is not None
        or entry.get("line_truncated") is not False
        or type(entry.get("sanitized_line")) is not str
    ):
        return False
    fields = _entry_fields(entry)
    return (
        set(fields) == AUDIT_STATUS_ALLOWED_FIELDS
        and fields.get("apparmor") == "DENIED"
        and fields.get("operation") == "open"
        and fields.get("class") == "file"
        and fields.get("profile") == BOOTSTRAP_PROFILE
        and fields.get("name") == f"/proc/{child_pid}/status"
        and fields.get("comm") == "python3.12"
        and fields.get("requested_mask") == "r"
        and fields.get("denied_mask") == "r"
        and fields.get("fsuid") == "1000"
        and fields.get("ouid") == "0"
        and type(fields.get("pid")) is str
        and fields["pid"].isdecimal()
        and int(fields["pid"]) > 0
        and RUNTIME_PROFILE not in entry["sanitized_line"]
        and not any(token in entry["sanitized_line"] for token in AUDIT_FORBIDDEN_TOKENS)
    )


def _is_proc_stat_denial(entry: Mapping[str, Any]) -> bool:
    fields = _entry_fields(entry)
    return (
        fields.get("apparmor") == "DENIED"
        and fields.get("operation") == "open"
        and fields.get("class") == "file"
        and type(fields.get("name")) is str
        and re.fullmatch(r"/proc/[0-9]+/stat", fields["name"]) is not None
    )


def _empty_audit(
    anchor: Mapping[str, Any],
    boot_after: str,
    query_argv: list[str] | None,
    child_pid: int | None,
) -> dict[str, Any]:
    return {
        "capture_valid": False,
        "capture_errors": [],
        "boot_id_before": anchor.get("boot_id"),
        "boot_id_after": boot_after,
        "start_cursor": anchor.get("cursor"),
        "end_cursor": None,
        "journal_anchor_argv": anchor.get("journal_anchor_argv"),
        "journal_query_argv": query_argv,
        "raw_stdout_sha256": sha256_bytes(b""),
        "raw_stdout_size_bytes": 0,
        "storage_ceiling": AUDIT_STORAGE_CEILING,
        "expected_child_pid": child_pid,
        "matching_total": 0,
        "stored_count": 0,
        "dropped_count": 0,
        "storage_overflow": False,
        "expected_status_total": 0,
        "stat_denial_total": 0,
        "unexpected_total": 0,
        "sanitized_denials": [],
        "expected_status_denials": [],
        "unexpected_denials": [],
    }


def capture_apparmor_denials(
    anchor: Mapping[str, Any], child_pid: int | None = None,
) -> dict[str, Any]:
    boot_after = read_boot_id()
    boot_before = anchor.get("boot_id")
    cursor = anchor.get("cursor")
    if (type(boot_before) is not str or BOOT_ID_RE.fullmatch(boot_before) is None
            or type(cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(cursor) is None):
        observed = _empty_audit(anchor, boot_after, None, child_pid)
        observed["capture_errors"] = ["INVALID_PRE_RUN_ANCHOR"]
        return observed
    if boot_after != boot_before:
        observed = _empty_audit(anchor, boot_after, None, child_pid)
        observed["capture_errors"] = ["BOOT_ID_CHANGED"]
        return observed
    compact_boot_id = boot_before.replace("-", "")
    argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
        "--output=short-iso-precise", f"--boot={compact_boot_id}",
        f"--after-cursor={cursor}", "--show-cursor",
    ]
    observed = _empty_audit(anchor, boot_after, argv, child_pid)
    result = run_bounded_command(
        argv,
        stdout_limit=AUDIT_QUERY_STDOUT_LIMIT,
        stderr_limit=AUDIT_QUERY_STDERR_LIMIT,
        timeout_seconds=15,
    )
    raw = result.get("stdout") if type(result.get("stdout")) is bytes else b""
    observed["raw_stdout_sha256"] = sha256_bytes(raw)
    observed["raw_stdout_size_bytes"] = len(raw)
    errors: list[str] = []
    if type(result.get("returncode")) is not int or result.get("returncode") != 0:
        errors.append("JOURNAL_QUERY_RETURN_CODE")
    if result.get("failure") is not None:
        errors.append("JOURNAL_QUERY_BOUNDED_FAILURE")
    if result.get("stderr") != b"":
        errors.append("JOURNAL_QUERY_STDERR")
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError:
        observed["capture_errors"] = errors + ["JOURNAL_QUERY_INVALID_UTF8"]
        return observed
    lines = text.splitlines()
    cursor_indices = [index for index, line in enumerate(lines) if line.startswith("-- cursor: ")]
    if len(cursor_indices) != 1 or cursor_indices[0] != len(lines) - 1:
        observed["capture_errors"] = errors + ["END_CURSOR_FRAMING"]
        return observed
    end_cursor = lines[-1][11:]
    if (JOURNAL_CURSOR_RE.fullmatch(end_cursor) is None
            or not _cursor_matches_boot(end_cursor, boot_before)):
        observed["capture_errors"] = errors + ["END_CURSOR_INVALID_OR_STALE"]
        return observed
    observed["end_cursor"] = end_cursor
    sanitized: list[dict[str, Any]] = []
    expected: list[dict[str, Any]] = []
    unexpected: list[dict[str, Any]] = []
    matching_total = expected_total = stat_denial_total = unexpected_total = 0
    for line in lines[:-1]:
        if not any(label in line for label in LABELS) or "apparmor" not in line or "DENIED" not in line:
            continue
        matching_total += 1
        entry = _parse_audit_line(line)
        if len(sanitized) < AUDIT_STORAGE_CEILING:
            sanitized.append(entry)
        if _is_proc_stat_denial(entry):
            stat_denial_total += 1
        if _is_exact_status_denial(entry, child_pid):
            expected_total += 1
            if len(expected) < AUDIT_STORAGE_CEILING:
                expected.append(entry)
        else:
            unexpected_total += 1
            if len(unexpected) < AUDIT_STORAGE_CEILING:
                unexpected.append(entry)
    stored_count = len(sanitized)
    observed.update({
        "capture_valid": not errors,
        "capture_errors": errors,
        "matching_total": matching_total,
        "stored_count": stored_count,
        "dropped_count": matching_total - stored_count,
        "storage_overflow": matching_total > stored_count,
        "expected_status_total": expected_total,
        "stat_denial_total": stat_denial_total,
        "unexpected_total": unexpected_total,
        "sanitized_denials": sanitized,
        "expected_status_denials": expected,
        "unexpected_denials": unexpected,
    })
    return observed


def gate_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups",
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--no-new-privs", "--",
        "/usr/bin/python3.12", "-I", "-B", str(SNAPSHOT_PATHS["gate"]), "internal-run",
    ]


def create_root_admission_pipe() -> tuple[int, dict[str, Any]]:
    """Create a root-owned, single-consumer anonymous-FD capability."""

    if os.geteuid() != 0:
        raise SupervisorError("admission capability creation requires euid 0")
    read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
    try:
        for descriptor in (read_fd, write_fd):
            metadata = os.fstat(descriptor)
            if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                    or stat.S_IMODE(metadata.st_mode) != 0o600):
                raise SupervisorError("root admission pipe metadata differs")
        nonce = os.getrandom(32)
        view = memoryview(nonce)
        while view:
            count = os.write(write_fd, view)
            if count <= 0:
                raise SupervisorError("short root admission capability write")
            view = view[count:]
        digest = sha256_bytes(nonce)
    except BaseException:
        os.close(read_fd)
        raise
    finally:
        os.close(write_fd)
    return read_fd, {
        "transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ",
        "size_bytes": 32,
        "sha256": digest,
        "pipe_uid": 0,
        "pipe_gid": 0,
        "pipe_mode": "0600",
        "public_environment_token_is_not_authority": True,
    }



def _audit_is_closed_success_window(
    audit: Mapping[str, Any], child_pid: int | None,
) -> bool:
    count_keys = (
        "storage_ceiling", "matching_total", "stored_count", "dropped_count",
        "expected_status_total", "stat_denial_total", "unexpected_total",
        "raw_stdout_size_bytes",
    )
    if not all(type(audit.get(key)) is int for key in count_keys):
        return False
    if (audit.get("capture_valid") is not True or audit.get("capture_errors") != []
            or audit.get("storage_ceiling") != AUDIT_STORAGE_CEILING
            or type(child_pid) is not int
            or child_pid < 2
            or type(audit.get("expected_child_pid")) is not int
            or audit.get("expected_child_pid") != child_pid
            or audit.get("matching_total") != 1
            or audit.get("stored_count") != 1
            or audit.get("dropped_count") != 0
            or audit.get("storage_overflow") is not False
            or audit.get("expected_status_total") != 1
            or audit.get("stat_denial_total") != 0
            or audit.get("unexpected_total") != 0
            or audit.get("raw_stdout_size_bytes") < 1
            or type(audit.get("raw_stdout_sha256")) is not str
            or re.fullmatch(r"[0-9a-f]{64}", audit["raw_stdout_sha256"]) is None):
        return False
    boot_before, boot_after = audit.get("boot_id_before"), audit.get("boot_id_after")
    start_cursor, end_cursor = audit.get("start_cursor"), audit.get("end_cursor")
    if (type(boot_before) is not str or BOOT_ID_RE.fullmatch(boot_before) is None
            or boot_after != boot_before
            or type(start_cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(start_cursor) is None
            or type(end_cursor) is not str or JOURNAL_CURSOR_RE.fullmatch(end_cursor) is None
            or end_cursor == start_cursor
            or not _cursor_matches_boot(start_cursor, boot_before)
            or not _cursor_matches_boot(end_cursor, boot_before)):
        return False
    compact_boot_id = boot_before.replace("-", "")
    expected_anchor_argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet", f"--boot={compact_boot_id}",
        "--lines=0", "--show-cursor",
    ]
    expected_query_argv = [
        "/usr/bin/journalctl", "-k", "--no-pager", "--quiet",
        "--output=short-iso-precise", f"--boot={compact_boot_id}",
        f"--after-cursor={start_cursor}", "--show-cursor",
    ]
    if (audit.get("journal_anchor_argv") != expected_anchor_argv
            or audit.get("journal_query_argv") != expected_query_argv):
        return False
    started, ended = audit.get("run_started_epoch"), audit.get("run_ended_epoch")
    if (type(started) is not float or type(ended) is not float
            or started <= 0.0 or ended < started):
        return False
    cleanup = audit.get("prequery_label_cleanup")
    if (type(cleanup) is not dict or cleanup.get("initial") != []
            or cleanup.get("term_sent") != [] or cleanup.get("after_term") != []
            or cleanup.get("kill_sent") != []
            or cleanup.get("stable_zero_scans") != [[], [], []]):
        return False
    profile_observation = audit.get("profiles_before_audit_query")
    if (type(profile_observation) is not dict
            or profile_observation.get("kernel_exact_counts") != {label: 1 for label in LABELS}
            or profile_observation.get("kernel_exact_enforce") != {label: True for label in LABELS}
            or profile_observation.get("aa_status_exact_presence") != {label: True for label in LABELS}
            or profile_observation.get("aa_status_exact_modes") != {label: "enforce" for label in LABELS}):
        return False
    empty_sha256 = sha256_bytes(b"")
    expected_sync = {
        "argv": ["/usr/bin/journalctl", "--sync"],
        "returncode": 0,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": 0,
        "stderr_sha256": empty_sha256,
        "failure": None,
        "start_new_session": True,
    }
    if audit.get("journal_sync") != expected_sync:
        return False
    postquery_profile = audit.get("profiles_after_audit_query")
    if (audit.get("postquery_stable_zero_labels") != [[], [], []]
            or type(postquery_profile) is not dict
            or postquery_profile.get("kernel_exact_counts") != {label: 1 for label in LABELS}
            or postquery_profile.get("kernel_exact_enforce") != {label: True for label in LABELS}
            or postquery_profile.get("aa_status_exact_presence") != {label: True for label in LABELS}
            or postquery_profile.get("aa_status_exact_modes") != {label: "enforce" for label in LABELS}):
        return False
    sanitized = audit.get("sanitized_denials")
    expected = audit.get("expected_status_denials")
    unexpected = audit.get("unexpected_denials")
    if (type(sanitized) is not list or type(expected) is not list
            or type(unexpected) is not list or len(sanitized) != 1
            or len(expected) != 1 or unexpected != [] or sanitized != expected
            or not _is_exact_status_denial(expected[0], child_pid)
            or _is_proc_stat_denial(expected[0])):
        return False
    return True


def classify_execution(result: Mapping[str, Any], audit: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    stdout, stderr = result.get("stdout"), result.get("stderr")
    evidence = _command_evidence(result, include_prefix=True)
    success_frame: dict[str, Any] | None = None
    if type(stdout) is bytes:
        try:
            success_frame = parse_success_frame(stdout)
        except SupervisorError:
            pass
    evidence["success_frame"] = success_frame
    returncode = result.get("returncode")
    child_pid = (
        success_frame.get("proc_stat_probe", {}).get("child_identity", {}).get("pid")
        if success_frame is not None else None
    )
    exact_success_window = _audit_is_closed_success_window(audit, child_pid)
    if (result.get("failure") is None and type(returncode) is int and returncode == 0
            and stderr == b"" and success_frame is not None and exact_success_window):
        return "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_CLEANUP_PENDING", evidence
    # Promote only a byte-exact frame paired with its one expected child-status
    # denial; every stat or additional denial remains fail-closed.
    return "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING", evidence


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
        raise SupervisorError("sudo cleanup argv type differs")
    if command_tail not in (["/usr/bin/sudo", "-K"], ["/usr/bin/sudo", "-n", "/usr/bin/true"]):
        raise SupervisorError("sudo cleanup inner command differs")
    if argv != expected_prefix + command_tail:
        raise SupervisorError("sudo cleanup argv differs or contains injected arguments")
    if any(token == "--bounding-set" or token.startswith("--bounding-set=") for token in argv):
        raise SupervisorError("sudo cleanup must preserve the host bounding set without a bounding argv token")
    if argv.count("--") != 1 or any(token in argv for token in ("--clear-groups", "--init-groups", "--keep-groups", "--no-new-privs")):
        raise SupervisorError("sudo cleanup group, delimiter, or NNP argv contract differs")


def verify_sudo_group_membership() -> dict[str, Any]:
    raw = read_regular_bytes(SUDO_GROUP_PATH, limit=1 << 20)
    metadata = os.stat(SUDO_GROUP_PATH, follow_symlinks=False)
    if (metadata.st_uid != 0 or metadata.st_gid != 0
            or stat.S_IMODE(metadata.st_mode) != 0o644):
        raise SupervisorError("/etc/group owner or mode differs from sudo cleanup contract")
    try:
        lines = raw.decode("utf-8", "strict").splitlines()
    except UnicodeError as exc:
        raise SupervisorError("/etc/group is not strict UTF-8") from exc
    sudo_or_gid_records: list[str] = []
    for line in lines:
        fields = line.split(":")
        if len(fields) != 4:
            raise SupervisorError("/etc/group contains a malformed record")
        if fields[0] == "sudo" or fields[2] == str(SUDO_GROUP_GID):
            sudo_or_gid_records.append(line)
    if sudo_or_gid_records != [SUDO_GROUP_RECORD]:
        raise SupervisorError("exact sudo:x:27:zrj membership record differs")
    return dict(SUDO_GROUP_MEMBERSHIP_CONTRACT)


def clear_invoking_user_sudo_timestamp() -> dict[str, Any]:
    # This helper is also the fail-safe for errors that happen before the
    # ordinary trusted-tool preflight completes.  Verify every executable it
    # may launch before inspecting the PTY or spawning a cleanup command.
    for name in ("setpriv", "sudo", "true"):
        require_tool(name)
    membership = verify_sudo_group_membership()
    if not os.isatty(0):
        raise SupervisorError("sudo timestamp cleanup requires the original invoking PTY on fd0")
    tty = {"stdin_isatty": True}
    try:
        metadata = os.fstat(0)
        foreground_pgrp = os.tcgetpgrp(0)
        current_pgrp = os.getpgrp()
        if foreground_pgrp != current_pgrp:
            raise SupervisorError("sudo timestamp cleanup requires the supervisor foreground process group")
        tty.update({
            "stdin_device": metadata.st_dev,
            "stdin_inode": metadata.st_ino,
            "stdin_rdev": metadata.st_rdev,
            "session_id": os.getsid(0),
            "process_group": current_pgrp,
            "foreground_process_group": foreground_pgrp,
        })
    except OSError as exc:
        raise SupervisorError("cannot capture inherited PTY identity") from exc
    clear_argv, verify_argv = sudo_timestamp_argvs()
    validate_sudo_cleanup_argv_contract(clear_argv, ["/usr/bin/sudo", "-K"])
    validate_sudo_cleanup_argv_contract(verify_argv, ["/usr/bin/sudo", "-n", "/usr/bin/true"])
    clear = run_bounded_command(clear_argv, stdin_bytes=None, stdout_limit=4096, stderr_limit=4096, timeout_seconds=15, start_new_session=False)
    verify = run_bounded_command(verify_argv, stdin_bytes=None, stdout_limit=4096, stderr_limit=4096, timeout_seconds=15, start_new_session=False)
    if (clear["argv"] != clear_argv or clear["returncode"] != 0 or isinstance(clear["returncode"], bool)
            or clear["failure"] is not None or clear["start_new_session"] is not False
            or clear["stdout"] or clear["stderr"]):
        raise SupervisorError("UID1000 sudo -K all-timestamp invalidation failed")
    if (verify["argv"] != verify_argv or verify["returncode"] != 1 or isinstance(verify["returncode"], bool)
            or verify["failure"] is not None or verify["start_new_session"] is not False
            or verify["stdout"] or verify["stderr"] != b"sudo: a password is required\n"):
        raise SupervisorError("sudo all-timestamp invalidation was not proven by -n true")
    return {
        "scope": "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE",
        "bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "pty": tty,
        "membership": membership,
        "identity_contract": dict(SUDO_CLEANUP_IDENTITY_CONTRACT),
        "clear": _command_evidence(clear),
        "noninteractive_true_must_fail": _command_evidence(verify, include_prefix=True),
    }


def validate_sudo_cleanup_evidence(sudo_clear: Mapping[str, Any]) -> None:
    expected_keys = {
        "scope", "bounding_mode", "pty", "membership", "identity_contract", "clear",
        "noninteractive_true_must_fail",
    }
    if type(sudo_clear) is not dict or set(sudo_clear) != expected_keys:
        raise SupervisorError("sudo timestamp cleanup evidence key set differs")
    clear_argv, verify_argv = sudo_timestamp_argvs()
    empty_sha256 = sha256_bytes(b"")
    expected_clear = {
        "argv": clear_argv,
        "returncode": 0,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": 0,
        "stderr_sha256": empty_sha256,
        "failure": None,
        "start_new_session": False,
    }
    password_stderr = b"sudo: a password is required\n"
    expected_verify = {
        "argv": verify_argv,
        "returncode": 1,
        "stdout_size_bytes": 0,
        "stdout_sha256": empty_sha256,
        "stderr_size_bytes": len(password_stderr),
        "stderr_sha256": sha256_bytes(password_stderr),
        "failure": None,
        "start_new_session": False,
        "stderr_utf8_prefix": password_stderr.decode("ascii"),
    }
    pty = sudo_clear["pty"]
    pty_keys = {
        "stdin_isatty", "stdin_device", "stdin_inode", "stdin_rdev", "session_id",
        "process_group", "foreground_process_group",
    }
    if (sudo_clear["scope"] != "ALL_UID1000_SUDO_TIMESTAMPS_INVALIDATED_BY_SUDO_-K_OTHER_TERMINALS_MUST_REAUTHENTICATE"
            or sudo_clear["bounding_mode"] != SUDO_CLEANUP_BOUNDING_MODE
            or not _json_equal(sudo_clear["membership"], SUDO_GROUP_MEMBERSHIP_CONTRACT)
            or not _json_equal(sudo_clear["identity_contract"], SUDO_CLEANUP_IDENTITY_CONTRACT)
            or not _json_equal(sudo_clear["clear"], expected_clear)
            or not _json_equal(sudo_clear["noninteractive_true_must_fail"], expected_verify)
            or type(pty) is not dict or set(pty) != pty_keys
            or pty["stdin_isatty"] is not True
            or any(type(pty[key]) is not int for key in pty_keys - {"stdin_isatty"})
            or any(pty[key] < 0 for key in {"stdin_device", "stdin_inode", "stdin_rdev"})
            or pty["session_id"] < 1 or pty["process_group"] < 1
            or pty["foreground_process_group"] != pty["process_group"]):
        raise SupervisorError("sudo timestamp cleanup evidence differs")


def execution_receipt_document(
    *,
    status: str,
    policy_sha256: str,
    start_receipt: Mapping[str, Any],
    snapshot: Mapping[str, Any],
    tools: Mapping[str, Any],
    sysctls_before: Mapping[str, Any],
    profiles_before: Mapping[str, Any],
    profiles_loaded: Mapping[str, Any] | None,
    gate_evidence: Mapping[str, Any] | None,
    admission_capability: Mapping[str, Any] | None,
    audit: Mapping[str, Any] | None,
    error: str | None,
) -> dict[str, Any]:
    if not status.endswith("CLEANUP_PENDING"):
        raise SupervisorError("execution receipt must remain cleanup-pending")
    return {
        "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_EXECUTION_RECEIPT",
        "probe_id": PROBE_ID,
        "status": status,
        "cleanup_complete": False,
        "production_authorized": False,
        "gencase_or_solver_executed": False,
        "host_writable_mount_used": False,
        "network_or_device_exposed": False,
        "policy_sha256": policy_sha256,
        "start_receipt": dict(start_receipt),
        "snapshot": dict(snapshot),
        "trusted_tools": dict(tools),
        "sysctls_before": dict(sysctls_before),
        "profiles_before": dict(profiles_before),
        "profiles_loaded": None if profiles_loaded is None else dict(profiles_loaded),
        "gate": None if gate_evidence is None else dict(gate_evidence),
        "root_admission_capability": None if admission_capability is None else dict(admission_capability),
        "apparmor_audit": None if audit is None else dict(audit),
        "error": error,
        "next_required_evidence": str(LIFECYCLE_RECEIPT),
    }


def lifecycle_document(
    *,
    execution_receipt: Mapping[str, Any],
    cleanup: Mapping[str, Any],
    profiles_after: Mapping[str, Any],
    sysctls_before: Mapping[str, Any],
    sysctls_after: Mapping[str, Any],
    sudo_clear: Mapping[str, Any],
) -> dict[str, Any]:
    stable = cleanup.get("stable_zero_scans")
    post_unload_stable = cleanup.get("post_unload_stable_zero_scans")
    if (cleanup.get("initial") != [] or cleanup.get("term_sent") != []
            or cleanup.get("after_term") != [] or cleanup.get("kill_sent") != []
            or stable != [[], [], []] or post_unload_stable != [[], [], []]):
        raise SupervisorError("lifecycle requires exact pre- and post-unload all-task empty label scans")
    require_profile_counts(profiles_after, 0)
    if sysctls_before != sysctls_after:
        raise SupervisorError("host sysctls changed across probe lifecycle")
    validate_sudo_cleanup_evidence(sudo_clear)
    return {
        "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_LIFECYCLE_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_LIFECYCLE_CLEANUP",
        "execution_receipt": dict(execution_receipt),
        "cleanup": dict(cleanup),
        "profiles_after": dict(profiles_after),
        "sysctls": {"before": dict(sysctls_before), "after": dict(sysctls_after), "unchanged": True},
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp": dict(sudo_clear),
        "production_authorized": False,
        "next_allowed_stage": "REVIEW_THIS_SINGLE_ATTEMPT_RECEIPT_AND_CREATE_FRESH_SUCCESSOR_ONLY",
    }


def run_once(*, policy_sha256: str, admission_token: str) -> dict[str, Any]:
    with TerminationGuard() as termination_guard:
        try:
            return _run_once_guarded(
                policy_sha256=policy_sha256,
                admission_token=admission_token,
                termination_guard=termination_guard,
            )
        except BaseException as exc:
            if termination_guard.cleanup_started:
                raise
            primary_error = f"{type(exc).__name__}: {exc}"[:4096]
            if not termination_guard.preflight_sudo_cleanup_attempted:
                termination_guard.preflight_sudo_cleanup_attempted = True
                try:
                    sudo_evidence = clear_invoking_user_sudo_timestamp()
                    validate_sudo_cleanup_evidence(sudo_evidence)
                    termination_guard.preflight_sudo_cleanup_evidence = dict(sudo_evidence)
                except BaseException as cleanup_exc:
                    termination_guard.preflight_sudo_cleanup_error = (
                        f"{type(cleanup_exc).__name__}: {cleanup_exc}"[:4096]
                    )
            termination_guard.begin_cleanup()
            sudo_clear = dict(termination_guard.preflight_sudo_cleanup_evidence)
            cleanup_error = termination_guard.preflight_sudo_cleanup_error
            cleanup_proven = cleanup_error is None and bool(sudo_clear)
            document = {
                "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_PREFLIGHT_FAILURE_RECEIPT",
                "probe_id": PROBE_ID,
                "status": (
                    "PRE_START_FAILURE_IDENTITY_CONSUMED_NO_RETRY_SUDO_TIMESTAMP_CLEANED_NO_PROFILE_LOAD_OR_PROBE_EXECUTED"
                    if cleanup_proven else
                    "PRE_START_FAILURE_IDENTITY_CONSUMED_NO_RETRY_SUDO_TIMESTAMP_CLEANUP_INCOMPLETE_NO_PROFILE_LOAD_OR_PROBE_EXECUTED"
                ),
                "requested_policy_sha256": policy_sha256,
                "primary_error": primary_error,
                "start_receipt_created_by_this_run": False,
                "parser_invoked_by_this_run": False,
                "profile_load_attempted_by_this_run": False,
                "probe_executed_by_this_run": False,
                "sudo_timestamp_cleanup_attempted": termination_guard.preflight_sudo_cleanup_attempted,
                "sudo_timestamp_cleanup_proven": cleanup_proven,
                "sudo_timestamp_cleanup_error": cleanup_error,
                "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
                "sudo_timestamp_observation": sudo_clear,
                "identity_consumed": True,
                "production_authorized": False,
            }
            try:
                record = write_json_new(PREFLIGHT_FAILURE_RECEIPT, document)
            except BaseException as receipt_exc:
                receipt_error = f"{type(receipt_exc).__name__}: {receipt_exc}"[:4096]
                raise SupervisorError(
                    "pre-start failure and preflight receipt incomplete; "
                    f"primary={primary_error}; sudo_cleanup={cleanup_error}; receipt={receipt_error}"
                ) from exc
            cleanup_status = "proven" if cleanup_proven else "incomplete"
            raise SupervisorError(f"pre-start failure; sudo timestamp cleanup {cleanup_status}; preserved receipt {record['path']}") from exc


def _run_once_guarded(
    *,
    policy_sha256: str,
    admission_token: str,
    termination_guard: TerminationGuard,
) -> dict[str, Any]:
    termination_guard.checkpoint()
    if admission_token != ADMISSION_TOKEN:
        raise SupervisorError("explicit one-shot admission token differs")
    policy, snapshot = verify_snapshot(policy_sha256)
    _assert_one_shot_paths_absent()
    if policy.get("authorization", {}).get("attempts_per_identity") != 1:
        raise SupervisorError("one-shot policy differs")
    tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    termination_guard.preflight_sudo_cleanup_attempted = True
    try:
        sudo_admission = clear_invoking_user_sudo_timestamp()
        validate_sudo_cleanup_evidence(sudo_admission)
        termination_guard.preflight_sudo_cleanup_evidence = dict(sudo_admission)
    except BaseException as exc:
        termination_guard.preflight_sudo_cleanup_error = f"{type(exc).__name__}: {exc}"[:4096]
        raise
    termination_guard.checkpoint()
    sysctls_before = read_sysctls()
    if policy.get("lifecycle_contract", {}).get("expected_sysctls") != sysctls_before:
        raise SupervisorError("pre-attempt sysctls differ from the frozen policy baseline")
    profiles_before = profile_state()
    require_profile_counts(profiles_before, 0)
    initial_zero = require_stable_zero_labels()
    audit_anchor = capture_journal_anchor()
    start_document = {
        "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_START_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "ONE_SHOT_STARTED_NO_RETRY_CLEANUP_REQUIRED",
        "policy_sha256": policy_sha256,
        "snapshot": snapshot,
        "profiles_before": profiles_before,
        "initial_stable_zero_labels": initial_zero,
        "journal_anchor": audit_anchor,
        "sysctls_before": sysctls_before,
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp_admission": sudo_admission,
    }
    start_record = write_json_new(START_RECEIPT, start_document)
    load_attempted = False
    profiles_loaded: dict[str, Any] | None = None
    admission_capability: dict[str, Any] | None = None
    execution_record: dict[str, Any] | None = None
    execution_status = "UNCLASSIFIED_PROBE_FAILURE_CLEANUP_PENDING"
    primary_error: str | None = None
    cleanup_errors: list[str] = []
    cleanup: dict[str, Any] = {"stable_zero_scans": None}
    profiles_after: dict[str, Any] = {
        "kernel_exact_counts": {label: -1 for label in LABELS},
        "aa_status_exact_presence": {label: True for label in LABELS},
        "aa_status_exact_modes": {label: "unknown" for label in LABELS},
    }
    sudo_clear: dict[str, Any] = {}
    sysctls_after: dict[str, Any] = {}
    try:
        termination_guard.checkpoint()
        profile_path = str(SNAPSHOT_PATHS["profile"])
        run_checked(["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
        termination_guard.checkpoint()
        load_attempted = True
        run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
        termination_guard.checkpoint()
        profiles_loaded = profile_state()
        require_profile_counts(profiles_loaded, 1)
        termination_guard.checkpoint()
        started = time.time()
        gate_env = {
            "HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC",
            SNAPSHOT_ENV: str(SNAPSHOT_ROOT), ADMISSION_ENV: ADMISSION_TOKEN,
        }
        termination_guard.checkpoint()
        admission_fd, admission_capability = create_root_admission_pipe()
        gate_env[ADMISSION_FD_ENV] = str(admission_fd)
        gate_env[ADMISSION_SHA256_ENV] = admission_capability["sha256"]
        try:
            gate_result = run_bounded_command(
                gate_handoff_argv(),
                stdout_limit=MAX_SUCCESS_FRAME_BYTES,
                stderr_limit=8192,
                timeout_seconds=25,
                env=gate_env,
                pass_fds=(admission_fd,),
            )
        finally:
            os.close(admission_fd)
        termination_guard.checkpoint()
        ended = time.time()
        prequery_label_cleanup = terminate_labeled_processes()
        if (prequery_label_cleanup.get("initial") != []
                or prequery_label_cleanup.get("term_sent") != []
                or prequery_label_cleanup.get("after_term") != []
                or prequery_label_cleanup.get("kill_sent") != []
                or prequery_label_cleanup.get("stable_zero_scans") != [[], [], []]):
            raise SupervisorError("pre-query label cleanup found residue or required a signal")
        profiles_before_audit_query = profile_state()
        require_profile_counts(profiles_before_audit_query, 1)
        journal_sync_result = run_checked(
            ["/usr/bin/journalctl", "--sync"], timeout_seconds=15,
            stdout_limit=4_096, stderr_limit=4_096, require_silent=True,
        )
        success_child_pid: int | None = None
        if type(gate_result.get("stdout")) is bytes:
            try:
                parsed_for_audit = parse_success_frame(gate_result["stdout"])
                success_child_pid = parsed_for_audit["proc_stat_probe"]["child_identity"]["pid"]
            except SupervisorError:
                pass
        audit = capture_apparmor_denials(audit_anchor, success_child_pid)
        audit["run_started_epoch"] = started
        audit["run_ended_epoch"] = ended
        audit["prequery_label_cleanup"] = prequery_label_cleanup
        audit["profiles_before_audit_query"] = profiles_before_audit_query
        audit["journal_sync"] = _command_evidence(journal_sync_result)
        audit["postquery_stable_zero_labels"] = require_stable_zero_labels()
        audit["profiles_after_audit_query"] = profile_state()
        require_profile_counts(audit["profiles_after_audit_query"], 1)
        termination_guard.checkpoint()
        execution_status, gate_evidence = classify_execution(gate_result, audit)
        document = execution_receipt_document(
            status=execution_status,
            policy_sha256=policy_sha256,
            start_receipt=start_record,
            snapshot=snapshot,
            tools=tools,
            sysctls_before=sysctls_before,
            profiles_before=profiles_before,
            profiles_loaded=profiles_loaded,
            gate_evidence=gate_evidence,
            admission_capability=admission_capability,
            audit=audit,
            error=None,
        )
        execution_record = write_json_new(EXECUTION_RECEIPT, document)
    except BaseException as exc:
        primary_error = f"{type(exc).__name__}: {exc}"[:4096]
        try:
            document = execution_receipt_document(
                status="ATTEMPT_ABORTED_CLEANUP_PENDING",
                policy_sha256=policy_sha256,
                start_receipt=start_record,
                snapshot=snapshot,
                tools=tools,
                sysctls_before=sysctls_before,
                profiles_before=profiles_before,
                profiles_loaded=profiles_loaded,
                gate_evidence=None,
                admission_capability=admission_capability,
                audit=None,
                error=primary_error,
            )
            execution_record = write_json_new(EXECUTION_RECEIPT, document)
            execution_status = document["status"]
        except BaseException as receipt_exc:
            cleanup_errors.append(f"execution_receipt: {type(receipt_exc).__name__}: {receipt_exc}"[:4096])
    finally:
        termination_guard.begin_cleanup()
        try:
            cleanup = terminate_labeled_processes()
        except BaseException as exc:
            cleanup_errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
        pre_unload_zero_proven = cleanup.get("stable_zero_scans") == [[], [], []]
        if not pre_unload_zero_proven:
            try:
                fallback_zero = require_stable_zero_labels()
                cleanup = {**cleanup, "stable_zero_scans": fallback_zero}
                pre_unload_zero_proven = True
            except BaseException as exc:
                cleanup_errors.append(f"pre_unload_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            if not pre_unload_zero_proven:
                raise SupervisorError("profile unload forbidden until all-task label zero is proven")
            current_counts = read_kernel_profile_counts()
            if load_attempted or any(current_counts.get(label, 0) for label in LABELS):
                unload = run_bounded_command(["/usr/sbin/apparmor_parser", "-R", "-K", str(SNAPSHOT_PATHS["profile"])], stdout_limit=65_536, stderr_limit=65_536, timeout_seconds=15)
                if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                    raise SupervisorError("profile unload failed")
            profiles_after = profile_state()
            require_profile_counts(profiles_after, 0)
        except BaseException as exc:
            cleanup_errors.append(f"profile_cleanup: {type(exc).__name__}: {exc}"[:4096])
        try:
            final_zero = require_stable_zero_labels()
            if cleanup.get("stable_zero_scans") != [[], [], []]:
                raise SupervisorError("cleanup label scans differ")
            cleanup = {**cleanup, "post_unload_stable_zero_scans": final_zero}
        except BaseException as exc:
            cleanup_errors.append(f"final_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            sudo_clear = clear_invoking_user_sudo_timestamp()
        except BaseException as exc:
            cleanup_errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])
        try:
            sysctls_after = read_sysctls()
            if sysctls_after != sysctls_before:
                raise SupervisorError("host sysctls changed")
        except BaseException as exc:
            cleanup_errors.append(f"sysctl_postcondition: {type(exc).__name__}: {exc}"[:4096])
        cleanup = {**cleanup, "termination_signals": list(termination_guard.received)}

    if execution_record is not None and not cleanup_errors:
        try:
            lifecycle = lifecycle_document(
                execution_receipt=execution_record,
                cleanup=cleanup,
                profiles_after=profiles_after,
                sysctls_before=sysctls_before,
                sysctls_after=sysctls_after,
                sudo_clear=sudo_clear,
            )
            lifecycle_record = write_json_new(LIFECYCLE_RECEIPT, lifecycle)
            return {"execution_status": execution_status, "execution_receipt": execution_record, "lifecycle_receipt": lifecycle_record, "primary_error": primary_error, "cleanup_errors": []}
        except BaseException as exc:
            cleanup_errors.append(f"lifecycle_receipt: {type(exc).__name__}: {exc}"[:4096])

    failure = {
        "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_LIFECYCLE_INCOMPLETE_RECEIPT",
        "probe_id": PROBE_ID,
        "status": "LIFECYCLE_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
        "start_receipt": start_record,
        "execution_receipt": execution_record,
        "execution_status": execution_status,
        "primary_error": primary_error,
        "cleanup_errors": cleanup_errors,
        "cleanup_observation": cleanup,
        "profiles_after_observation": profiles_after,
        "sysctls_after_observation": sysctls_after,
        "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
        "sudo_timestamp_observation": sudo_clear,
        "production_authorized": False,
    }
    failure_record = write_json_new(LIFECYCLE_FAILURE_RECEIPT, failure)
    raise SupervisorError(f"fresh proc-stat lifecycle incomplete; preserved receipt {failure_record['path']}")


def recover_only(*, policy_sha256: str, recovery_token: str) -> dict[str, Any]:
    """Idempotent cleanup path that can never create an admission FD or run gate."""

    if recovery_token != RECOVERY_TOKEN:
        raise SupervisorError("explicit cleanup-only recovery token differs")
    policy, snapshot = verify_snapshot(policy_sha256)
    try:
        recovery_receipt_preexisting = stat.S_ISREG(os.lstat(RECOVERY_RECEIPT).st_mode)
    except FileNotFoundError:
        recovery_receipt_preexisting = False
    errors: list[str] = []
    recovery_tools: dict[str, Any] = {}
    try:
        sysctls_before = read_sysctls()
        if policy.get("lifecycle_contract", {}).get("expected_sysctls") != sysctls_before:
            errors.append("preexisting_sysctls_differ_from_frozen_baseline")
    except BaseException as exc:
        sysctls_before = {}
        errors.append(f"sysctls_before: {type(exc).__name__}: {exc}"[:4096])
    try:
        before_entries = read_kernel_profile_entries()
    except BaseException as exc:
        before_entries = {label: [] for label in LABELS}
        errors.append(f"kernel_profiles_before: {type(exc).__name__}: {exc}"[:4096])
    profiles_before = {"kernel_exact_lines": before_entries, "kernel_exact_counts": {label: len(lines) for label, lines in before_entries.items()}}
    if any(count not in (0, 1) for count in profiles_before["kernel_exact_counts"].values()):
        errors.append("impossible_duplicate_exact_profile_labels")
    with TerminationGuard() as termination_guard:
        termination_guard.begin_cleanup()
        try:
            cleanup = terminate_labeled_processes()
        except BaseException as exc:
            cleanup = {"stable_zero_scans": None}
            errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
        label_zero_proven = cleanup.get("stable_zero_scans") == [[], [], []]
        if not label_zero_proven:
            errors.append("pre_unload_all_task_label_zero_not_proven")
        try:
            current_entries = read_kernel_profile_entries()
            current_counts = {label: len(lines) for label, lines in current_entries.items()}
            profile_presence_unknown = False
        except BaseException as exc:
            current_entries = {label: [] for label in LABELS}
            current_counts = {label: -1 for label in LABELS}
            profile_presence_unknown = True
            errors.append(f"kernel_profiles_pre_unload: {type(exc).__name__}: {exc}"[:4096])
        unload_evidence: dict[str, Any] | None = None
        if profile_presence_unknown or any(current_counts.get(label, 0) for label in LABELS):
            if not label_zero_proven:
                errors.append("profile_unload_skipped_until_label_zero_is_proven")
            else:
                try:
                    recovery_tools["apparmor_parser"] = require_tool("apparmor_parser")
                    unload = run_bounded_command(
                        ["/usr/sbin/apparmor_parser", "-R", "-K", str(SNAPSHOT_PATHS["profile"])],
                        stdout_limit=65_536,
                        stderr_limit=65_536,
                        timeout_seconds=15,
                    )
                    if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                        raise SupervisorError("cleanup-only exact profile unload failed")
                    unload_evidence = _command_evidence(unload, include_prefix=True)
                except BaseException as exc:
                    errors.append(f"profile_unload: {type(exc).__name__}: {exc}"[:4096])
        try:
            after_entries = read_kernel_profile_entries()
        except BaseException as exc:
            after_entries = {label: ["UNKNOWN"] for label in LABELS}
            errors.append(f"kernel_profiles_after: {type(exc).__name__}: {exc}"[:4096])
        profiles_after_kernel = {"kernel_exact_lines": after_entries, "kernel_exact_counts": {label: len(lines) for label, lines in after_entries.items()}}
        if any(profiles_after_kernel["kernel_exact_counts"].values()):
            errors.append("exact_profiles_remain_after_recovery")
        try:
            post_unload_zero = require_stable_zero_labels()
        except BaseException as exc:
            post_unload_zero = []
            errors.append(f"post_unload_label_scan: {type(exc).__name__}: {exc}"[:4096])
        try:
            recovery_tools["aa_status"] = require_tool("aa_status")
            profiles_after = profile_state()
            require_profile_counts(profiles_after, 0)
        except BaseException as exc:
            profiles_after = profiles_after_kernel
            errors.append(f"profile_corroboration: {type(exc).__name__}: {exc}"[:4096])
        try:
            for name in ("setpriv", "sudo", "true"):
                recovery_tools[name] = require_tool(name)
            sudo_clear = clear_invoking_user_sudo_timestamp()
        except BaseException as exc:
            sudo_clear = {}
            errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])
        try:
            sysctls_after = read_sysctls()
            expected_sysctls = policy.get("lifecycle_contract", {}).get("expected_sysctls")
            if sysctls_after != expected_sysctls or (sysctls_before and sysctls_after != sysctls_before):
                errors.append("sysctls_not_equal_to_frozen_baseline_after_recovery")
        except BaseException as exc:
            sysctls_after = {}
            errors.append(f"sysctls_after: {type(exc).__name__}: {exc}"[:4096])
        if errors:
            raise SupervisorError("cleanup-only recovery incomplete: " + " | ".join(errors))
        document = {
            "document_type": "SMPCC_R8_LIQUID_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_RECOVERY_RECEIPT",
            "probe_id": PROBE_ID,
            "status": "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_CLEANUP_ONLY_NO_PROBE_EXECUTED",
            "policy_sha256": policy_sha256,
            "snapshot": snapshot,
            "trusted_tools": recovery_tools,
            "cleanup": {**cleanup, "post_unload_stable_zero_scans": post_unload_zero},
            "profiles_before": profiles_before,
            "profile_unload": unload_evidence,
            "profiles_after": profiles_after,
            "sysctls": {"before": sysctls_before, "after": sysctls_after, "unchanged": True},
            "sudo_cleanup_bounding_mode": SUDO_CLEANUP_BOUNDING_MODE,
            "sudo_timestamp": sudo_clear,
            "termination_signals_ignored_during_cleanup": list(termination_guard.received),
            "probe_executed": False,
            "admission_fd_created": False,
            "production_authorized": False,
        }
        if recovery_receipt_preexisting:
            record = {
                "path": str(RECOVERY_RECEIPT),
                "preexisting_not_rewritten": True,
                "same_uid_parent_can_remove_or_replace_so_existing_bytes_are_not_retrusted": True,
            }
        else:
            record = write_json_new(RECOVERY_RECEIPT, document)
    return {"status": document["status"], "recovery_receipt": record}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("emit-bootstrap")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--policy-sha256", required=True)
    run_parser.add_argument("--admission-token", required=True)
    recover_parser = subparsers.add_parser("recover-only")
    recover_parser.add_argument("--policy-sha256", required=True)
    recover_parser.add_argument("--recovery-token", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = repository_static_review()
            state = "STATIC_READY" if review["freeze_ready"] else "STATIC_DRAFT"
            print(json.dumps({"status": f"PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_{state}_EXECUTION_NOT_PERFORMED", "review": review, "snapshot_bootstrap": {"source_sha256": SNAPSHOT_BOOTSTRAP_SHA256, "source_size_bytes": len(SNAPSHOT_BOOTSTRAP_BYTES), "loader_sha256": SNAPSHOT_BOOTSTRAP_LOADER_SHA256, "loader_size_bytes": len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES), "manifest_arguments": snapshot_manifest_arguments(review)}}, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "emit-bootstrap":
            # Unprivileged delivery only. The root-side fixed -c loader verifies
            # exact length/hash and strict EOF before compiling these bytes.
            verify_snapshot_producer_identity()
            sys.stdout.buffer.write(SNAPSHOT_BOOTSTRAP_BYTES)
            sys.stdout.buffer.flush()
            return 0
        if arguments.command == "recover-only":
            result = recover_only(policy_sha256=arguments.policy_sha256, recovery_token=arguments.recovery_token)
            print(json.dumps({"status": "PROC_STAT_BOUNDARY_PROBE_V2_CLEANUP_ONLY_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
            return 0
        result = run_once(policy_sha256=arguments.policy_sha256, admission_token=arguments.admission_token)
        print(json.dumps({"status": "PROC_STAT_BOUNDARY_PROBE_V2_ONE_SHOT_LIFECYCLE_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
        return 0 if result["execution_status"] == "PASS_U3_PROC_STAT_APPARMOR_BOUNDARY_PROBE_V2_CLEANUP_PENDING" else 2
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        target = sys.stderr if arguments.command == "emit-bootstrap" else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=target)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
