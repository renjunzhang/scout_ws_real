#!/usr/bin/env python3
"""Static and UID-1000 child gate for one admitted U3 C1M GenCase v8 run.

The frozen v11 harmless probe established the exact bwrap mount grammar and
NNP-compatible AppArmor `rix` inheritance used here.  ``self-check`` is
read-only.  ``internal-run`` additionally requires a root-owned single-use
anonymous-FD capability and a root-owned 0444 snapshot before it can launch the
fixed network/device-free bubblewrap command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import selectors
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
SNAPSHOT_ENV = "R8_LIQUID_U3_GENCASE_V8_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_GENCASE_V8_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_GENCASE_V8_ADMISSION_SHA256"
SNAPSHOT_DIR = Path(os.environ[SNAPSHOT_ENV]) if SNAPSHOT_ENV in os.environ else None

CASE_ID = "u3_c1m_gencase_v8_20260808T153753Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-c1m-gencase-bootstrap-v8-20260808t153753z"
RUNTIME_PROFILE = "r8-liquid-u3-c1m-gencase-runtime-v8-20260808t153753z"

POLICY_REPOSITORY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v8.json"
SCHEMA_REPOSITORY_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v8.json"
PROFILE_REPOSITORY_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v8.profile"
HELPER_REPOSITORY_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gencase_bootstrap_helper_v8.py")
SUPERVISOR_REPOSITORY_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gencase_supervisor_v8.py")

if SNAPSHOT_DIR is None:
    POLICY_PATH = POLICY_REPOSITORY_PATH
    SCHEMA_PATH = SCHEMA_REPOSITORY_PATH
    PROFILE_PATH = PROFILE_REPOSITORY_PATH
    HELPER_PATH = HELPER_REPOSITORY_PATH
    SUPERVISOR_PATH = SUPERVISOR_REPOSITORY_PATH
else:
    POLICY_PATH = SNAPSHOT_DIR / POLICY_REPOSITORY_PATH.name
    SCHEMA_PATH = SNAPSHOT_DIR / SCHEMA_REPOSITORY_PATH.name
    PROFILE_PATH = SNAPSHOT_DIR / PROFILE_REPOSITORY_PATH.name
    HELPER_PATH = SNAPSHOT_DIR / HELPER_REPOSITORY_PATH.name
    SUPERVISOR_PATH = SNAPSHOT_DIR / SUPERVISOR_REPOSITORY_PATH.name

SEED_ID = "u3_c1m_gencase_seed_v3_20260808T150802Z"
SEED_ROOT = Path(f"/home/zrj/scout_liquid_lab/dependency/runtime/{SEED_ID}.partial")
SEED_INPUT_ROOT = SEED_ROOT / "input"
SEED_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json")
SEED_RECEIPT_SHA256 = "79f43b54594843e0107aa78ad01db34937fa97f0a34a16d96e74ac9cd90095ed"
SEED_RECEIPT_STATUS = "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION"
V7_CASE_ID = "u3_c1m_gencase_v7_20260808T151500Z"
V7_START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V7_CASE_ID}.start.json")
V7_EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V7_CASE_ID}.execution.json")
V7_LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{V7_CASE_ID}.lifecycle_incomplete.json")
V7_START_RECEIPT_SHA256 = "85bb282afd9201c9e1a185060d7389c7a8a5ba69767ed62d7b5cbb5d739439d2"
V7_EXECUTION_RECEIPT_SHA256 = "d9a90ae5a2464c2e3c5b24675af07feac1155237f7a1105c331f757821e0633c"
V7_LIFECYCLE_RECEIPT_SHA256 = "00f2924a6213315f8a96b2ee0a13003d7df48b2009a3e011d829fa5efa5f918d"
ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.start.json")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")

INPUT_CONTRACT: dict[str, tuple[str, int]] = {
    "GenCase_linux64": ("a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226", 5_809_384),
    "DsphConfig.xml": ("0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd", 293),
    "C1M_moving_zero_Def.xml": ("1711b10f366770cb1b6aa2af799c093596c9d606c914c4a68b9bcdb9fa844b52", 6_070),
}
EXPECTED_OUTPUTS = ("C1M_zero.bi4", "C1M_zero.xml", "C1M_zero.out")
TMPFS_OUTPUT_BYTES = 67_108_864
GUEST_FRAME_HEADER_BYTES = 56
GUEST_STDOUT_LIMIT = GUEST_FRAME_HEADER_BYTES + 65_536 + 16_777_216 + 524_288 + 1_048_576 + 1_048_576
GUEST_STDERR_LIMIT = 1_114_112
HELPER_MAGIC = b"R8C1HELPERV8\0\0\0\0\0"
INPUT_MAGIC = b"R8C1INPUTV8\0\0\0\0\0"

TRUSTED_TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sha256sum": (Path("/usr/bin/sha256sum"), "9992e1f1feb6f0f396bc8d6691ebc1adbfc269fd628bce84eda1d4ba5c3995c7"),
    "sudo": (Path("/usr/bin/sudo"), "136f2e48b0295b9fc595b8259cf2411ac43f27ddbfe02b956649ddaa2e92b9fa"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

def guest_loader(*, helper_size: int, helper_sha256: str) -> str:
    return (
        "import hashlib,os\n"
        "def _r(n):\n"
        " b=bytearray()\n"
        " while len(b)<n:\n"
        "  q=os.read(0,n-len(b))\n"
        "  if not q: raise SystemExit(90)\n"
        "  b.extend(q)\n"
        " return bytes(b)\n"
        f"m=_r({len(HELPER_MAGIC)})\n"
        f"h=_r({helper_size})\n"
        f"if m!={HELPER_MAGIC!r} or hashlib.sha256(h).hexdigest()!={helper_sha256!r}: raise SystemExit(91)\n"
        "g={'__name__':'__main__','__file__':'<r8-liquid-u3-gencase-helper-v8>'}\n"
        "exec(compile(h,g['__file__'],'exec',dont_inherit=True,optimize=2),g,g)\n"
    )


def guest_command(*, helper_size: int, helper_sha256: str) -> list[str]:
    return [
    "/usr/bin/python3.12",
    "-I",
    "-B",
    "-S",
    "-c",
    guest_loader(helper_size=helper_size, helper_sha256=helper_sha256),
    ]
GENCASE_ARGV = [
    "/work/runtime/GenCase_linux64",
    "/work/runtime/C1M_moving_zero_Def",
    "/work/output/C1M_zero",
    "-dp:0.002",
    "-threads:1",
    "-save:-all,+bi",
    "-createdirs:1",
]

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed static, identity, transport, or command-contract error."""


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
            block = os.read(descriptor, 1 << 20)
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise GateError(f"reviewed artifact exceeds limit: {path}")
    finally:
        os.close(descriptor)


def sha256_regular_file(path: Path, *, limit: int = 2 * 1024 * 1024) -> str:
    return hashlib.sha256(read_regular_bytes(path, limit=limit)).hexdigest()


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid reviewed JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"reviewed JSON is not an object: {path}")
    return value


def require_tool(name: str) -> dict[str, Any]:
    path, expected = TRUSTED_TOOLS[name]
    metadata = os.stat(path, follow_symlinks=False)
    if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise GateError(f"trusted tool is not a real regular file: {path}")
    if metadata.st_uid != 0 or metadata.st_gid != 0 or not metadata.st_mode & stat.S_IXUSR:
        raise GateError(f"trusted tool ownership or execute mode differs: {path}")
    observed = sha256_regular_file(path, limit=512 * 1024 * 1024)
    if observed != expected:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": observed}


def trusted_tool_policy() -> dict[str, dict[str, str]]:
    return {name: {"path": str(path), "sha256": digest} for name, (path, digest) in TRUSTED_TOOLS.items()}


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def verify_profile(*, run_parser: bool) -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    expected_hash = policy["trusted_artifacts"]["profile"]["sha256"]
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_hash:
        raise GateError("v8 AppArmor profile digest differs")
    text = raw.decode("utf-8")
    effective = _effective_profile(text)
    lines = tuple(line.strip() for line in effective.splitlines() if line.strip())
    bootstrap_header = f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    runtime_header = f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    if effective.count(bootstrap_header) != 1 or effective.count(runtime_header) != 1:
        raise GateError("v8 AppArmor label identity differs")
    inherited_exec = "/work/runtime/GenCase_linux64 rix,"
    if lines.count(inherited_exec) != 1:
        raise GateError("GenCase must have exactly one NNP-compatible rix rule")
    if any("rpx" in line.lower() or " -> " in line for line in lines if "GenCase_linux64" in line):
        raise GateError("GenCase profile transition is forbidden under NNP")
    bootstrap, runtime = effective.split(runtime_header, 1)
    for required in (
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
        f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
        "deny capability dac_override,",
        "deny /proc/stat r,",
        "/work/** rw,",
    ):
        if required not in bootstrap:
            raise GateError(f"bootstrap profile lacks required narrow rule: {required}")
    if lines.count("deny capability dac_override,") != 1:
        raise GateError("exact quiet dac_override denial differs")
    if lines.count("deny /proc/stat r,") != 1:
        raise GateError("exact quiet /proc/stat denial differs")
    mount_lines = tuple(
        line for line in lines
        if line.startswith(("mount ", "remount ", "pivot_root ", "umount "))
    )
    expected_mount_lines = (
        "mount options=(rw, silent, rslave) -> /,",
        "mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/ ,".replace("/ ,", "/,"),
        "mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/ ,".replace("/ ,", "/,"),
        "pivot_root oldroot=/tmp/oldroot/ /tmp/ ,".replace("/ ,", "/,"),
        "mount options=(rw, silent, rprivate) -> /oldroot/ ,".replace("/ ,", "/,"),
        "umount /oldroot/ ,".replace("/ ,", "/,"),
        "pivot_root oldroot=/newroot/ /newroot/ ,".replace("/ ,", "/,"),
        "umount /,",
        "mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/ ,".replace("/ ,", "/,"),
        "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/ ,".replace("/ ,", "/,"),
        "mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/ ,".replace("/ ,", "/,"),
        "mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/ ,".replace("/ ,", "/,"),
    )
    if mount_lines != expected_mount_lines:
        raise GateError("AppArmor mount grammar differs from the frozen v11 success set")
    for forbidden in (
        "/work", "/usr", "/proc", "/dev", "/home/", "userns ", "mount ",
        "remount ", "pivot_root ", "umount ", "capability ", "network ",
        "ptrace ", "signal ", "change_profile", " rpx", " rix,", " ix,",
    ):
        if forbidden in runtime:
            raise GateError(f"unreachable runtime profile contains authority: {forbidden}")
    for forbidden_global in (
        "flags=(unconfined)",
        "/home/zrj/scout_ws",
        "/home/zrj/ros2_ws",
        "/opt/ros/jazzy",
        "/dev/nvidia",
        "ux,",
        "Ux,",
        "/work/export",
        "/work/transport",
        "change_profile",
    ):
        if forbidden_global in effective:
            raise GateError(f"profile violates a global forbidden boundary: {forbidden_global}")
    parser_result: dict[str, Any] = {"executed": False}
    if run_parser:
        require_tool("apparmor_parser")
        completed = subprocess.run(
            ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", str(PROFILE_PATH)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd="/",
            env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC0"},
            timeout=15,
            check=False,
        )
        if completed.returncode != 0 or len(completed.stdout) > 16_384 or len(completed.stderr) > 16_384:
            raise GateError("AppArmor profile parser-only check failed")
        parser_result = {
            "executed": True,
            "returncode": completed.returncode,
            "stdout": completed.stdout.decode("utf-8", "replace"),
            "stderr": completed.stderr.decode("utf-8", "replace"),
        }
    return {
        "path": str(PROFILE_PATH),
        "sha256": digest,
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
        "production_load_allowed": True,
        "parser_static_check": parser_result,
    }


def _artifact_paths() -> dict[str, Path]:
    return {
        "gate": SCRIPT_PATH,
        "helper": HELPER_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def expected_rejected_v7_attempt() -> dict[str, Any]:
    return {
        "identity": V7_CASE_ID,
        "permanent_status": "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
        "failure_class": "SOURCE_PROVEN_DUAL_MOTION_COPY_AUDITOR_FALSE_NEGATIVE",
        "reason": (
            "GenCase completed with the exact C1M particle summary, but the v7 guest auditor "
            "rejected the generated XML because it required all_motion == [execution_motion] "
            "and did not allow the source-proven casedef/motion copy retained alongside "
            "normalized execution/motion"
        ),
        "start_receipt": str(V7_START_RECEIPT),
        "start_receipt_sha256": V7_START_RECEIPT_SHA256,
        "execution_receipt": str(V7_EXECUTION_RECEIPT),
        "execution_receipt_sha256": V7_EXECUTION_RECEIPT_SHA256,
        "lifecycle_receipt": str(V7_LIFECYCLE_RECEIPT),
        "lifecycle_receipt_sha256": V7_LIFECYCLE_RECEIPT_SHA256,
        "gencase_completed": True,
        "particle_summary": {
            "fixed": 0,
            "moving": 2669,
            "floating": 0,
            "fluid": 6409,
            "total": 9078,
        },
        "case_exported": False,
        "cleanup_complete": True,
    }


def verify_rejected_v7_attempt() -> dict[str, Any]:
    receipts = {
        "start": (V7_START_RECEIPT, V7_START_RECEIPT_SHA256),
        "execution": (V7_EXECUTION_RECEIPT, V7_EXECUTION_RECEIPT_SHA256),
        "lifecycle": (V7_LIFECYCLE_RECEIPT, V7_LIFECYCLE_RECEIPT_SHA256),
    }
    documents: dict[str, dict[str, Any]] = {}
    evidence: dict[str, dict[str, Any]] = {}
    for name, (path, expected_digest) in receipts.items():
        observed_digest = sha256_regular_file(path)
        if observed_digest != expected_digest:
            raise GateError(f"frozen v7 {name} receipt digest differs")
        documents[name] = read_json_object(path)
        evidence[name] = {"path": str(path), "sha256": observed_digest}
    start = documents["start"]
    execution = documents["execution"]
    lifecycle = documents["lifecycle"]
    gate = execution.get("gate", {})
    audit = execution.get("audit", {})
    profiles = lifecycle.get("profiles_after_observation", {})
    if (
        start.get("case_id") != V7_CASE_ID
        or start.get("status") != "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY"
        or execution.get("case_id") != V7_CASE_ID
        or execution.get("status") != "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY"
        or execution.get("primary_error")
        != "SupervisorError: UID-1000 gate did not return one clean successful frame"
        or execution.get("cleanup_errors") != []
        or gate.get("returncode") != 2
        or gate.get("stdout_size_bytes") != 0
        or not gate.get("stdin_fully_written")
        or not str(gate.get("stderr_utf8_prefix", "")).startswith(
            '{"error":"generated C1M XML motion container location or attributes differ"'
        )
        or audit.get("capture_valid") is not True
        or audit.get("capture_errors") != []
        or audit.get("matching_total") != 0
        or audit.get("unexpected_total") != 0
        or audit.get("sanitized_denials") != []
        or execution.get("export") is not None
        or execution.get("export_verification") is not None
        or lifecycle.get("case_id") != V7_CASE_ID
        or lifecycle.get("status") != "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY"
        or lifecycle.get("cleanup_errors") != []
        or any(profiles.get("kernel_exact_counts", {}).values())
        or any(profiles.get("aa_status_exact_presence", {}).values())
        or os.path.lexists(f"/home/zrj/scout_liquid_lab/cases/{V7_CASE_ID}.partial")
        or os.path.lexists(f"/home/zrj/scout_liquid_lab/audits/{V7_CASE_ID}.lifecycle.json")
        or os.path.lexists(f"/home/zrj/scout_liquid_lab/audits/{V7_CASE_ID}.console.log")
    ):
        raise GateError("frozen v7 dual-motion audit failure provenance differs")
    return evidence


def verify_review_artifacts(*, run_parser: bool = True) -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("v8 policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-gencase-execution-policy-v8",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GENCASE_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GENCASE_EXECUTION_V8",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "execution_scope": "one_exact_c1m_zero_motion_gencase_v8_attempt_after_frozen_v7_dual_motion_audit_failure_and_v11_probe",
        "allowed_gate_commands": ["self-check", "internal-run"],
        "status": "ADMITTED_SINGLE_C1M_ZERO_MOTION_GENCASE_RUNTIME_V8_AFTER_FROZEN_V7_DUAL_MOTION_AUDIT_FAILURE_AND_V11_ZERO_DENIAL_PROBE",
        "next_allowed_stage": "EXECUTE_ONCE_THEN_VALIDATE_C1M_IDENTITY_OR_FREEZE_FAILURE",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise GateError(f"v8 policy field differs: {key}")
    if policy.get("trusted_tools") != trusted_tool_policy():
        raise GateError("v8 trusted-tool policy differs")
    observed_tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    artifacts = policy.get("trusted_artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != set(_artifact_paths()):
        raise GateError("v8 trusted-artifact set differs")
    observed_artifacts: dict[str, dict[str, str]] = {}
    for name, path in _artifact_paths().items():
        expected = artifacts[name]
        if expected.get("path") != policy_artifact_relative_path(name):
            raise GateError(f"v8 artifact path differs: {name}")
        digest = sha256_regular_file(path, limit=2 * 1024 * 1024)
        if digest != expected.get("sha256"):
            raise GateError(f"v8 artifact digest differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": digest}
    frozen = policy["frozen_attempt"]
    expected_frozen = {
        "case_id": CASE_ID,
        "attempt_root": str(ATTEMPT_ROOT),
        "output_root": str(OUTPUT_ROOT),
        "start_receipt": str(START_RECEIPT),
        "execution_receipt": str(EXECUTION_RECEIPT),
        "lifecycle_receipt": str(LIFECYCLE_RECEIPT),
        "lifecycle_failure_receipt": str(LIFECYCLE_FAILURE_RECEIPT),
        "console_log": str(CONSOLE_LOG),
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
    }
    if frozen != expected_frozen:
        raise GateError("v8 frozen attempt identity differs")
    if policy.get("rejected_v7_attempt") != expected_rejected_v7_attempt():
        raise GateError("v8 frozen v7 failure policy differs")
    rejected_v7_evidence = verify_rejected_v7_attempt()
    expected_seed_provenance = {
        "seed_id": SEED_ID,
        "seed_root": str(SEED_ROOT),
        "input_root": str(SEED_INPUT_ROOT),
        "receipt": str(SEED_RECEIPT),
        "receipt_sha256": SEED_RECEIPT_SHA256,
        "receipt_status": SEED_RECEIPT_STATUS,
        "files": {
            "GenCase_linux64": {
                "sha256": INPUT_CONTRACT["GenCase_linux64"][0],
                "size_bytes": INPUT_CONTRACT["GenCase_linux64"][1],
                "seed_mode": "0400",
                "snapshot_mode": "0444",
            },
            "DsphConfig.xml": {
                "sha256": INPUT_CONTRACT["DsphConfig.xml"][0],
                "size_bytes": INPUT_CONTRACT["DsphConfig.xml"][1],
                "seed_mode": "0400",
                "snapshot_mode": "0444",
            },
            "C1M_moving_zero_Def.xml": {
                "sha256": INPUT_CONTRACT["C1M_moving_zero_Def.xml"][0],
                "size_bytes": INPUT_CONTRACT["C1M_moving_zero_Def.xml"][1],
                "seed_mode": "0400",
                "snapshot_mode": "0444",
                "corrections": [
                    "drawcylinder mask=2",
                    "MinFluidStop=1",
                    "PartsOutMax absent",
                    "casedef motion ref=0/mvnull",
                    "Shifting=1",
                    "DtAllParticles=1",
                ],
            },
        },
    }
    if policy.get("seed_provenance") != expected_seed_provenance:
        raise GateError("v8 C1M seed v3 provenance differs")
    transport = policy["immutable_transport"]
    if transport.get("input_delivery") != "fixed_no_declared_lengths_stdin_frame_to_hash_pinned_in_memory_helper":
        raise GateError("v8 framed stdin transport differs")
    if transport.get("output_delivery") != "success_only_single_bounded_stdout_binary_frame_then_uid1000_O_EXCL":
        raise GateError("v8 framed stdout transport differs")
    expected_output_frame = {
        "magic": "R8C1GENCASEV8",
        "version": 1,
        "header_format": ">16sIIQQQQ",
        "canonical_metadata_max_bytes": 65_536,
        "bi4_max_bytes": 16_777_216,
        "xml_max_bytes": 524_288,
        "out_max_bytes": 1_048_576,
        "console_max_bytes": 1_048_576,
        "total_max_bytes": GUEST_STDOUT_LIMIT,
        "trailing_bytes": "forbidden",
    }
    if transport.get("output_frame") != expected_output_frame:
        raise GateError("v8 fixed three-payload output frame differs")
    if policy.get("output_contract", {}).get("guest_exact_files") != list(EXPECTED_OUTPUTS):
        raise GateError("v8 exact guest output set differs")
    if policy.get("isolation", {}).get("quiet_denials") != [
        "deny capability dac_override,", "deny /proc/stat r,"
    ]:
        raise GateError("v8 quiet-denial policy differs")
    if policy["isolation"].get("guest_output_tmpfs_byte_ceiling") != TMPFS_OUTPUT_BYTES or policy["isolation"].get("guest_output_tmpfs_inode_ceiling") != "UNCLAIMED_BWRAP_0_9_HAS_NO_NR_INODES_OPTION":
        raise GateError("v8 tmpfs byte/inode claim differs")
    helper = policy["trusted_artifacts"]["helper"]
    expected_guest = guest_command(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    command_policy = policy["fixed_guest_command"]
    if (command_policy.get("bootstrap_helper_argv_canonical_sha256") != canonical_hash(expected_guest)
            or command_policy.get("gencase_argv") != GENCASE_ARGV):
        raise GateError("v8 fixed guest command differs")
    probe = policy["required_harmless_probes"]
    if (probe.get("admission") != "PASS_FROZEN_V11_ZERO_DENIAL_STDIO_APPARMOR_TRANSPORT_PROBE"
            or probe.get("required_receipt_sha256") != "02a9421481850986e8e4d7ef488e1ed8a6d177b8f40a683625a916d1fc563f71"
            or probe.get("required_lifecycle_receipt_sha256") != "8f56909f33a4192becb9f194a5e168e5152eb5d5416e4313fec8ec010467e9f8"):
        raise GateError("frozen v11 admission provenance differs")
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": observed_artifacts["schema"]["sha256"]},
        "artifacts": observed_artifacts,
        "profile": verify_profile(run_parser=run_parser),
        "trusted_tools": observed_tools,
        "rejected_v7_attempt": rejected_v7_evidence,
        "status": policy["status"],
    }


def policy_artifact_relative_path(name: str) -> str:
    return {
        "gate": "scripts/r8_liquid_target_u3_gencase_gate_v8.py",
        "helper": "scripts/r8_liquid_u3_gencase_bootstrap_helper_v8.py",
        "supervisor": "scripts/r8_liquid_target_u3_gencase_supervisor_v8.py",
        "profile": "config/apparmor_drafts/r8-liquid-u3-gencase-v8.profile",
        "schema": "schema/target_host_u3_gencase_execution_policy_v8.json",
    }[name]


def read_identity_status(path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
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
        raise GateError("cannot parse the child identity status") from exc
    return {"uid": uid, "gid": gid, "groups": groups, "capabilities": caps, "no_new_privs": no_new_privs}


def verify_child_identity(
    status: Mapping[str, Any] | None = None,
    *,
    expected_no_new_privs: int = 1,
) -> dict[str, Any]:
    observed = dict(read_identity_status() if status is None else status)
    if observed.get("uid") != [HOST_UID] * 4 or observed.get("gid") != [HOST_GID] * 4:
        raise GateError("all real/effective/saved/fs UID and GID fields must equal the named user")
    if observed.get("groups") != []:
        raise GateError("supplementary groups were not cleared")
    caps = observed.get("capabilities")
    if not isinstance(caps, dict) or set(caps) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"} or any(caps.values()):
        raise GateError("child capability sets are not all zero")
    if observed.get("no_new_privs") != expected_no_new_privs:
        raise GateError("host child NoNewPrivs stage differs")
    return observed


def bwrap_argv(*, helper_size: int, helper_sha256: str) -> list[str]:
    argv = [
        "/usr/bin/timeout", "--foreground", "--signal=TERM", "--kill-after=5s", "120s",
        "/usr/bin/aa-exec", "-p", BOOTSTRAP_PROFILE, "--",
        "/usr/bin/bwrap",
        "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--uid", "0", "--gid", "0", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-gencase-v8",
        "--clearenv",
        "--setenv", "HOME", "/nonexistent",
        "--setenv", "PATH", "/usr/bin",
        "--setenv", "LC_ALL", "C.UTF-8",
        "--setenv", "LANG", "C.UTF-8",
        "--setenv", "TZ", "UTC0",
        "--ro-bind", "/usr", "/usr",
        "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64",
        "--size", str(TMPFS_OUTPUT_BYTES), "--tmpfs", "/work",
        "--proc", "/proc",
    ]
    command = guest_command(helper_size=helper_size, helper_sha256=helper_sha256)
    argv.extend(["--chdir", "/work", "--", *command])
    verify_argv_contract(argv, expected_guest_command=command)
    return argv


def _pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1:index + 3] for index, value in enumerate(argv[:-2]) if value == flag]


def verify_argv_contract(argv: list[str], *, expected_guest_command: list[str]) -> None:
    if _pairs(argv, "--ro-bind") != [["/usr", "/usr"]] or _pairs(argv, "--bind"):
        raise GateError("bwrap host path bind surface differs")
    if _pairs(argv, "--bind-fd"):
        raise GateError("bwrap must not expose any host output fd")
    if _pairs(argv, "--file"):
        raise GateError("bwrap must not receive host file descriptors or --file inputs")
    if [argv[index + 1:index + 3] for index, value in enumerate(argv[:-2]) if value == "--size"] != [[str(TMPFS_OUTPUT_BYTES), "--tmpfs"]]:
        raise GateError("bwrap tmpfs size option ordering differs")
    output_index = argv.index(str(TMPFS_OUTPUT_BYTES))
    if argv[output_index:output_index + 3] != [str(TMPFS_OUTPUT_BYTES), "--tmpfs", "/work"]:
        raise GateError("bwrap 64-MiB ceiling is not attached to the complete /work tmpfs")
    for required in ("--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--disable-userns", "--assert-userns-disabled", "--cap-drop"):
        if argv.count(required) != 1:
            raise GateError(f"bwrap exact isolation option differs: {required}")
    cap_index = argv.index("--cap-drop")
    if argv[cap_index + 1] != "ALL":
        raise GateError("bwrap child capabilities are not dropped with ALL")
    if [argv[index + 1] for index, value in enumerate(argv[:-1]) if value == "--proc"] != ["/proc"]:
        raise GateError("bwrap must mount exactly one guest proc for bootstrap identity verification")
    for forbidden in ("--dev", "--dev-bind", "--share-net", "--bind-fd", "--file", "/home/zrj/scout_ws", "/opt/ros/jazzy", "/dev/nvidia0"):
        if forbidden in argv:
            raise GateError(f"bwrap argv violates a forbidden boundary: {forbidden}")
    if argv[-len(expected_guest_command):] != expected_guest_command:
        raise GateError("fixed guest helper command differs")


def require_admitted_v8_policy() -> None:
    policy = read_json_object(POLICY_PATH)
    if policy.get("status") != "ADMITTED_SINGLE_C1M_ZERO_MOTION_GENCASE_RUNTIME_V8_AFTER_FROZEN_V7_DUAL_MOTION_AUDIT_FAILURE_AND_V11_ZERO_DENIAL_PROBE":
        raise GateError("v8 runtime policy is not the exact admitted one-shot revision")


def verify_snapshot_runtime() -> dict[str, Any]:
    if SNAPSHOT_DIR != SNAPSHOT_ROOT or SCRIPT_PATH.parent != SNAPSHOT_ROOT:
        raise GateError("internal-run requires the exact v8 snapshot environment")
    root = os.lstat(SNAPSHOT_ROOT)
    if (not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0
            or stat.S_IMODE(root.st_mode) != 0o555):
        raise GateError("v8 snapshot directory metadata differs")
    for path in (SCRIPT_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH, HELPER_PATH, SUPERVISOR_PATH):
        metadata = os.lstat(path)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444):
            raise GateError(f"v8 snapshot artifact metadata differs: {path.name}")
    return {"root": str(SNAPSHOT_ROOT), "uid": 0, "gid": 0, "mode": "0555"}


def consume_root_admission_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[ADMISSION_FD_ENV])
        expected_sha256 = os.environ[ADMISSION_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise GateError("root admission FD capability is absent") from exc
    if descriptor < 3 or len(expected_sha256) != 64:
        raise GateError("root admission FD capability metadata differs")
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or stat.S_IMODE(metadata.st_mode) != 0o600 or not os.get_inheritable(descriptor)):
            raise GateError("root admission pipe inode differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise GateError("root admission capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError("root admission capability has trailing bytes")
    finally:
        os.close(descriptor)
    observed = hashlib.sha256(payload).hexdigest()
    if observed != expected_sha256:
        raise GateError("root admission capability digest differs")
    os.environ.pop(ADMISSION_FD_ENV, None)
    os.environ.pop(ADMISSION_SHA256_ENV, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "size_bytes": 32, "sha256": observed}


def build_input_frame(helper_bytes: bytes, inputs: Mapping[str, bytes]) -> bytes:
    if set(inputs) != set(INPUT_CONTRACT):
        raise GateError("input frame file set differs")
    policy = read_json_object(POLICY_PATH)
    expected_helper = policy["trusted_artifacts"]["helper"]
    if len(helper_bytes) != expected_helper["size_bytes"] or hashlib.sha256(helper_bytes).hexdigest() != expected_helper["sha256"]:
        raise GateError("input frame helper identity differs")
    parts = [HELPER_MAGIC, helper_bytes, INPUT_MAGIC]
    for name, (digest, size) in INPUT_CONTRACT.items():
        raw = inputs[name]
        if len(raw) != size or hashlib.sha256(raw).hexdigest() != digest:
            raise GateError(f"input frame seed identity differs: {name}")
        parts.append(raw)
    return b"".join(parts)


def run_bounded_guest(argv: list[str], input_frame: bytes) -> tuple[int, bytes, bytes]:
    """Concurrently stream all three pipes and enforce hard byte ceilings."""

    process = subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC0"},
        close_fds=True,
        start_new_session=True,
    )
    if process.stdin is None or process.stdout is None or process.stderr is None:
        raise GateError("guest conduit pipes are unavailable")
    descriptors = {
        "stdin": process.stdin.fileno(),
        "stdout": process.stdout.fileno(),
        "stderr": process.stderr.fileno(),
    }
    for descriptor in descriptors.values():
        os.set_blocking(descriptor, False)
    selector = selectors.DefaultSelector()
    selector.register(descriptors["stdin"], selectors.EVENT_WRITE, "stdin")
    selector.register(descriptors["stdout"], selectors.EVENT_READ, "stdout")
    selector.register(descriptors["stderr"], selectors.EVENT_READ, "stderr")
    position = 0
    stdout = bytearray()
    stderr = bytearray()
    deadline = time.monotonic() + 135

    def stop_group() -> None:
        process_group = process.pid

        def group_exists() -> bool:
            # poll() also reaps the direct child when it has exited; killpg(0)
            # then checks every remaining member of the original group.
            process.poll()
            try:
                os.killpg(process_group, 0)
                return True
            except ProcessLookupError:
                return False

        try:
            os.killpg(process_group, signal.SIGTERM)
        except ProcessLookupError:
            process.poll()
        term_deadline = time.monotonic() + 5
        while group_exists() and time.monotonic() < term_deadline:
            time.sleep(0.05)
        if group_exists():
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        kill_deadline = time.monotonic() + 2
        while group_exists() and time.monotonic() < kill_deadline:
            time.sleep(0.05)
        if group_exists():
            raise GateError("outer guest process group remains after TERM/KILL cleanup")

    try:
        while selector.get_map():
            if time.monotonic() >= deadline:
                stop_group()
                raise GateError("guest conduit exceeded its outer 135-second deadline")
            events = selector.select(timeout=0.2)
            for key, _mask in events:
                descriptor = key.fd
                channel = key.data
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, input_frame[position:position + 65_536])
                    except BrokenPipeError:
                        count = 0
                        selector.unregister(descriptor)
                        process.stdin.close()
                    if count:
                        position += count
                        if position == len(input_frame):
                            selector.unregister(descriptor)
                            process.stdin.close()
                    continue
                try:
                    block = os.read(descriptor, 65_536)
                except BlockingIOError:
                    continue
                if not block:
                    selector.unregister(descriptor)
                    if channel == "stdout":
                        process.stdout.close()
                    else:
                        process.stderr.close()
                    continue
                target = stdout if channel == "stdout" else stderr
                ceiling = GUEST_STDOUT_LIMIT if channel == "stdout" else GUEST_STDERR_LIMIT
                room = max(0, ceiling + 1 - len(target))
                target.extend(block[:room])
                if len(block) > room or len(target) > ceiling:
                    stop_group()
                    raise GateError(f"guest {channel} exceeded its hard byte ceiling")
            if process.poll() is not None and not events:
                # The read ends will report EOF on a following selector pass.
                continue
    finally:
        selector.close()
        if process.poll() is None:
            stop_group()
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                stream.close()
            except (BrokenPipeError, OSError):
                pass
    returncode = process.wait()
    if position != len(input_frame):
        raise GateError("guest did not consume the complete fixed input frame")
    return returncode, bytes(stdout), bytes(stderr)


def internal_child(input_frame: bytes) -> tuple[int, bytes, bytes]:
    # All checks occur before aa-exec, bwrap, or GenCase.
    require_admitted_v8_policy()
    verify_snapshot_runtime()
    verify_child_identity()
    consume_root_admission_capability()
    verify_review_artifacts(run_parser=False)
    policy = read_json_object(POLICY_PATH)
    helper = policy["trusted_artifacts"]["helper"]
    expected_input_size = len(HELPER_MAGIC) + helper["size_bytes"] + len(INPUT_MAGIC) + sum(size for _digest, size in INPUT_CONTRACT.values())
    if len(input_frame) != expected_input_size:
        raise GateError("host-to-guest input frame has an unexpected total length")
    argv = bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    return run_bounded_guest(argv, input_frame)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("internal-run", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            review = verify_review_artifacts(run_parser=True)
            print(json.dumps({"status": "PASS_V8_STATIC_ADMITTED_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        frame = sys.stdin.buffer.read(6_000_000)
        if sys.stdin.buffer.read(1) != b"":
            raise GateError("outer input frame exceeds its fixed ceiling")
        returncode, stdout, stderr = internal_child(frame)
        if returncode != 0:
            # A failing helper must expose zero stdout bytes.  Never forward a
            # partial/spoofed success frame from a nonzero guest execution.
            sys.stderr.buffer.write(stderr)
            if stdout:
                sys.stderr.write("\nNO_GO: discarded nonzero-rc guest stdout bytes\n")
            sys.stderr.buffer.flush()
            return returncode
        if stderr or len(stdout) < GUEST_FRAME_HEADER_BYTES:
            raise GateError("successful guest must return one nonempty frame and zero stderr bytes")
        sys.stdout.buffer.write(stdout)
        sys.stdout.buffer.flush()
        return 0
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        stream = sys.stderr if arguments.command == "internal-run" else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=stream)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
