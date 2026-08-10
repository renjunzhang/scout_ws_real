#!/usr/bin/env python3
"""Static gate and UID-1000 conduit for one U3 C1M CPU-solver smoke v3.

``self-check`` is read-only and never executes an ELF.  ``internal-run`` is a
hidden one-shot entry accepted only from the exact root-owned snapshot, with a
root-created nonce FD and a fixed stdin frame.  The bwrap guest receives no host
descriptor except its bounded stdio pipes and has no host-writable bind.
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
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
CASE_ID = "u3_c1m_solver_cpu_smoke_v3_20260808T160108Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-c1m-solver-cpu-bootstrap-v3-20260808t160108z"
RUNTIME_PROFILE = "r8-liquid-u3-c1m-solver-cpu-runtime-v3-20260808t160108z"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_SOLVER_CPU_V3_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_V3_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_V3_ADMISSION_SHA256"

POLICY_NAME = "liquid_zrj_msi_u2404_u3_solver_cpu_execution_policy_v3.json"
SCHEMA_NAME = "target_host_u3_solver_cpu_execution_policy_v3.json"
PROFILE_NAME = "r8-liquid-u3-solver-cpu-v3.profile"
GATE_NAME = "r8_liquid_target_u3_solver_cpu_gate_v3.py"
HELPER_NAME = "r8_liquid_u3_solver_cpu_bootstrap_helper_v3.py"
SUPERVISOR_NAME = "r8_liquid_target_u3_solver_cpu_supervisor_v3.py"
HARNESS_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"

IN_SNAPSHOT = SCRIPT_PATH.parent == SNAPSHOT_ROOT
BASE = SNAPSHOT_ROOT if IN_SNAPSHOT else PACKAGE_ROOT
POLICY_PATH = BASE / POLICY_NAME if IN_SNAPSHOT else BASE / "config/target_hosts" / POLICY_NAME
SCHEMA_PATH = BASE / SCHEMA_NAME if IN_SNAPSHOT else BASE / "schema" / SCHEMA_NAME
PROFILE_PATH = BASE / PROFILE_NAME if IN_SNAPSHOT else BASE / "config/apparmor_drafts" / PROFILE_NAME
HELPER_PATH = BASE / HELPER_NAME if IN_SNAPSHOT else BASE / "scripts" / HELPER_NAME
SUPERVISOR_PATH = BASE / SUPERVISOR_NAME if IN_SNAPSHOT else BASE / "scripts" / SUPERVISOR_NAME
HARNESS_PATH = BASE / HARNESS_NAME if IN_SNAPSHOT else BASE / "scripts" / HARNESS_NAME

HELPER_MAGIC = b"R8SOLVERHELPV3\0\0"
INPUT_MAGIC = b"R8SOLVERINPUT3\0\0"
POLICY_STATUS = "REVIEWED_FRESH_SINGLE_C1M_CPU_SOLVER_SMOKE_V3_TWO_KNOWN_QUIET_DENY_RULES_PENDING_STATIC_VERIFICATION"
SEED_ID = "u3_c1m_gencase_seed_v3_20260808T150802Z"
SEED_RECEIPT_PROVENANCE = {
    "path": f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json",
    "sha256": "79f43b54594843e0107aa78ad01db34937fa97f0a34a16d96e74ac9cd90095ed",
    "size_bytes": 12_382,
    "status": "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION",
    "seed_id": SEED_ID,
}
CACHE_QUIET_DENY_RULE = "deny /etc/ld.so.cache r,"
CAPABILITY_QUIET_DENY_RULE = "deny capability dac_override,"
KNOWN_QUIET_DENY_RULES = (CACHE_QUIET_DENY_RULE, CAPABILITY_QUIET_DENY_RULE)
KNOWN_QUIET_DENY_ORIGINS = {
    CACHE_QUIET_DENY_RULE: "carried_forward_from_v2_after_v1_observed_bwrap_dynamic_loader_probe",
    CAPABILITY_QUIET_DENY_RULE: "carried_forward_from_v2_preexisting_v1_bwrap_capability_rule",
}
DENIAL_VISIBILITY_BOUNDARY = (
    "closed_zero_unexpected_logged_DENIED_records_with_two_explicit_unlogged_deny_rules;"
    "both_carried_from_successful_v2;"
    "does_not_claim_zero_denied_operations"
)
V1_CASE_ID = "u3_c1_solver_cpu_smoke_v1_20260808T124816Z"
V1_PREDECESSOR_RECEIPTS = {
    "start": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.start.json",
        "sha256": "779feb5bdccdb2c833a4c04dd06ebf6241434b3343a3506753a9b82544758238",
        "size_bytes": 10_274,
        "status": "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
    },
    "execution": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.execution.json",
        "sha256": "6c8a2600bcb40c1039976061ec296509c4a2d6d6e89e941de9b7bfb4edf48494",
        "size_bytes": 7_614,
        "status": "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
    },
    "lifecycle_incomplete": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V1_CASE_ID}.lifecycle_incomplete.json",
        "sha256": "412d7e8a5c1adfc340707f6af608ab29c23f2a9446dfa6c32453a12b48caa090",
        "size_bytes": 5_037,
        "status": "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
    },
}
V2_CASE_ID = "u3_c1_solver_cpu_smoke_v2_20260808T134452Z"
V2_PREDECESSOR_RECEIPTS = {
    "start": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.start.json",
        "sha256": "6050a28d4f1093761d94a502df8635bb5964eaa161aa3f69c20e63c056799a63",
        "size_bytes": 10_966,
        "status": "V2_ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
    },
    "execution": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.execution.json",
        "sha256": "bfa560359e045158a25da9c75850f4a3c40f0fb22012e410f2abab46f36c88d0",
        "size_bytes": 42_463,
        "status": "PASS_U3_C1_CPU_SOLVER_V2_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING",
    },
    "lifecycle": {
        "path": f"/home/zrj/scout_liquid_lab/audits/{V2_CASE_ID}.lifecycle.json",
        "sha256": "e01f81c864fa444b312dbe68b6d5490d45f53d8f99d180e3c71600fbcf6bad39",
        "size_bytes": 6_337,
        "status": "PASS_U3_C1_CPU_SOLVER_V2_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES",
    },
}
INPUT_CONTRACT: tuple[tuple[str, str, int], ...] = (
    ("DualSPHysics5.4CPU_linux64", "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202", 32_649_520),
    ("DsphConfig.xml", "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd", 293),
    ("C1M_zero.xml", "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb", 7_800),
    ("C1M_zero.bi4", "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7", 400_842),
    ("ld-linux-x86-64.so.2", "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81", 236_616),
    ("libgomp.so.1", "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1", 352_304),
    ("libstdc++.so.6", "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11", 2_592_224),
    ("libm.so.6", "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91", 952_616),
    ("libgcc_s.so.1", "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb", 183_024),
    ("libc.so.6", "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161", 2_125_328),
)
C1M_CASE_SEMANTICS = {
    "particle_count": 9_078,
    "fixed_boundary": 0,
    "moving_boundary": 2_669,
    "fluid": 6_409,
    "floating": 0,
    "dp_m": 0.002,
    "moving_refmotion": 0,
    "zero_motion_copies": 2,
    "zero_motion_primitive": "objreal_ref0_begin_mov1_start0_mvnull_id1",
    "shifting": 1,
    "dt_all_particles": 1,
    "expected_motion_output": "data/PartMotionRef.ibi4",
}

SOLVER_ARGV = [
    "/work/runtime/ld-linux-x86-64.so.2",
    "--inhibit-cache",
    "--library-path",
    "/work/runtime/lib",
    "/work/runtime/DualSPHysics5.4CPU_linux64",
    "/work/case/C1M_zero",
    "/work/output",
    "-cpu",
    "-ompthreads:1",
    "-stable:1",
    "-vres:0",
    "-cellmode:full",
    "-tmax:1.0",
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
    "TMPDIR": "/work/tmp",
    "OMP_NUM_THREADS": "1",
    "OMP_DYNAMIC": "FALSE",
    "OMP_PROC_BIND": "FALSE",
}

TMPFS_BYTES = 268_435_456
INPUT_FRAME_LIMIT = 50_331_648
OUTPUT_TOTAL_LIMIT = 67_108_864
OUTPUT_METADATA_LIMIT = 262_144
OUTPUT_CONSOLE_LIMIT = 4_194_304
OUTPUT_HEADER_BYTES = 48
GUEST_STDOUT_LIMIT = OUTPUT_HEADER_BYTES + OUTPUT_METADATA_LIMIT + OUTPUT_TOTAL_LIMIT + OUTPUT_CONSOLE_LIMIT
GUEST_STDERR_LIMIT = 4_718_592

TOOLS: dict[str, tuple[Path, str]] = {
    "aa_exec": (Path("/usr/bin/aa-exec"), "f28cbce3c8664cab5154492fdbc55ecb937a3e7ce1a9478c881a5f5965d7ce3e"),
    "aa_status": (Path("/usr/sbin/aa-status"), "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"),
    "apparmor_parser": (Path("/usr/sbin/apparmor_parser"), "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"),
    "bwrap": (Path("/usr/bin/bwrap"), "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712"),
    "python": (Path("/usr/bin/python3.12"), "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118"),
    "setpriv": (Path("/usr/bin/setpriv"), "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"),
    "sudo": (Path("/usr/bin/sudo"), "136f2e48b0295b9fc595b8259cf2411ac43f27ddbfe02b956649ddaa2e92b9fa"),
    "timeout": (Path("/usr/bin/timeout"), "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08"),
}

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A fail-closed static, identity, transport or command error."""


def canonical_hash(value: Any) -> str:
    raw = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def read_regular_bytes(path: Path, *, limit: int = 80 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= limit:
            raise GateError(f"unsafe regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise GateError(f"short regular-file read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise GateError(f"regular file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path, limit=2 * 1024 * 1024).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GateError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return value


def sha256_file(path: Path, *, limit: int = 80 * 1024 * 1024) -> str:
    return hashlib.sha256(read_regular_bytes(path, limit=limit)).hexdigest()


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
        "g={'__name__':'__main__','__file__':'<r8-liquid-u3-c1m-solver-cpu-helper-v3>'}\n"
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


def bwrap_argv(*, helper_size: int, helper_sha256: str) -> list[str]:
    command = guest_command(helper_size=helper_size, helper_sha256=helper_sha256)
    argv = [
        "/usr/bin/timeout",
        "--foreground",
        "--signal=TERM",
        "--kill-after=10s",
        "1800s",
        "/usr/bin/aa-exec",
        "-p",
        BOOTSTRAP_PROFILE,
        "--",
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
        "r8-liquid-solver-cpu-v3",
        "--clearenv",
    ]
    for key, value in ENVIRONMENT.items():
        argv.extend(("--setenv", key, value))
    argv.extend(
        (
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
            *command,
        )
    )
    verify_argv_contract(argv, expected_guest_command=command)
    return argv


def _pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == flag]


def verify_argv_contract(argv: list[str], *, expected_guest_command: list[str]) -> None:
    if _pairs(argv, "--ro-bind") != [["/usr", "/usr"]] or _pairs(argv, "--bind"):
        raise GateError("bwrap host bind surface differs")
    if _pairs(argv, "--bind-fd") or _pairs(argv, "--file") or _pairs(argv, "--dev-bind"):
        raise GateError("bwrap exposes a forbidden host descriptor or device")
    if [argv[index + 1 : index + 3] for index, value in enumerate(argv[:-2]) if value == "--size"] != [[str(TMPFS_BYTES), "--tmpfs"]]:
        raise GateError("bwrap tmpfs size grammar differs")
    size_index = argv.index(str(TMPFS_BYTES))
    if argv[size_index : size_index + 3] != [str(TMPFS_BYTES), "--tmpfs", "/work"]:
        raise GateError("bwrap tmpfs ceiling is not attached to /work")
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
        if argv.count(required) != 1:
            raise GateError(f"bwrap exact isolation option differs: {required}")
    if argv[argv.index("--cap-drop") + 1] != "ALL":
        raise GateError("bwrap child capabilities are not dropped")
    if [argv[index + 1] for index, value in enumerate(argv[:-1]) if value == "--proc"] != ["/proc"]:
        raise GateError("bwrap proc mount count differs")
    forbidden = (
        "--dev",
        "--share-net",
        "/dev",
        "/sys",
        "/home/zrj/scout_ws",
        "/home/zrj/scout_liquid_lab",
        "/dev/nvidia0",
    )
    if any(value in argv for value in forbidden):
        raise GateError("bwrap argv crosses a forbidden boundary")
    if argv[-len(expected_guest_command) :] != expected_guest_command:
        raise GateError("fixed guest helper command differs")


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def verify_profile() -> dict[str, Any]:
    raw = read_regular_bytes(PROFILE_PATH, limit=128 * 1024)
    text = raw.decode("utf-8", "strict")
    effective = _effective_profile(text)
    if effective.count(f"profile {BOOTSTRAP_PROFILE} ") != 1 or effective.count(f"profile {RUNTIME_PROFILE} ") != 1:
        raise GateError("fresh solver profile labels differ")
    bootstrap_body, runtime_section = effective.split(f"profile {RUNTIME_PROFILE}", 1)
    bootstrap_lines = [line.strip() for line in bootstrap_body.splitlines() if line.strip()]
    runtime_body = runtime_section.split("}", 1)[0]
    runtime_lines = [line.strip() for line in runtime_body.splitlines() if line.strip()]
    deny_lines = [line for line in bootstrap_lines if "deny " in line]
    cache_authority_lines = [line for line in bootstrap_lines if "/etc/ld.so" in line]
    capability_authority_lines = [line for line in bootstrap_lines if "capability dac_override" in line]
    if deny_lines != list(KNOWN_QUIET_DENY_RULES):
        raise GateError("bootstrap explicit deny set is not the exact two-rule contract")
    if cache_authority_lines != [CACHE_QUIET_DENY_RULE]:
        raise GateError("bootstrap ld.so cache authority is not the exact quiet deny")
    if capability_authority_lines != [CAPABILITY_QUIET_DENY_RULE]:
        raise GateError("bootstrap dac_override authority is not the exact quiet deny")
    if any(
        "deny " in line or "/etc/ld.so" in line or "capability dac_override" in line
        for line in runtime_lines
    ):
        raise GateError("unreachable runtime profile contains a quiet deny or related authority")
    required = (
        "/usr/bin/bwrap rix,",
        "/usr/bin/python3.12 rix,",
        "/work/runtime/ld-linux-x86-64.so.2 rix,",
        "/work/runtime/ld-linux-x86-64.so.2 mr,",
        "/work/runtime/DualSPHysics5.4CPU_linux64 mr,",
        "/work/runtime/lib/** mr,",
        "/proc/stat r,",
        CACHE_QUIET_DENY_RULE,
        CAPABILITY_QUIET_DENY_RULE,
        "userns create,",
        "mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/ ,".replace("/ ,", "/,"),
        "mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/ ,".replace("/ ,", "/,"),
        f"signal (send,receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
    )
    if any(marker not in effective for marker in required):
        raise GateError("solver AppArmor profile lacks a required narrow rule")
    if "/work/runtime/DualSPHysics5.4CPU_linux64 rix," in effective:
        raise GateError("solver must be mapped only by the copied loader")
    for forbidden in (
        "/home/zrj/scout_ws",
        "/home/zrj/scout_liquid_lab",
        "/dev/nvidia",
        "change_profile",
        "flags=(unconfined)",
    ):
        if forbidden in effective:
            raise GateError(f"solver profile exposes forbidden authority: {forbidden}")
    authority_tokens = (" /work", " /usr", " userns ", " mount ", " capability ", " network ", " signal ", " ptrace ", " rix,", " mr,", " rw,")
    if any(token in runtime_body for token in authority_tokens):
        raise GateError("unreachable runtime profile is not empty")
    return {"path": str(PROFILE_PATH), "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}


def _artifact_paths() -> dict[str, Path]:
    return {
        "gate": SCRIPT_PATH,
        "helper": HELPER_PATH,
        "supervisor": SUPERVISOR_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def verify_review_artifacts(*, verify_tools: bool = True) -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("solver policy/schema top-level contract differs")
    if policy.get("status") != POLICY_STATUS:
        raise GateError("solver policy status differs")
    if policy.get("allowed_gate_commands") != ["self-check", "internal-run"]:
        raise GateError("solver gate command surface differs")
    expected_artifacts = policy.get("trusted_artifacts")
    if not isinstance(expected_artifacts, dict) or set(expected_artifacts) != set(_artifact_paths()):
        raise GateError("trusted solver artifact set differs")
    observed_artifacts: dict[str, Any] = {}
    for name, path in _artifact_paths().items():
        raw = read_regular_bytes(path, limit=4 * 1024 * 1024)
        observed = {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)}
        entry = expected_artifacts[name]
        if observed["sha256"] != entry.get("sha256") or observed["size_bytes"] != entry.get("size_bytes"):
            raise GateError(f"trusted solver artifact differs: {name}")
        observed_artifacts[name] = observed
    profile = verify_profile()

    if (
        policy.get("provenance", {}).get("gencase_seed_v3_receipt")
        != SEED_RECEIPT_PROVENANCE
    ):
        raise GateError("C1M GenCase seed v3 provenance differs")

    predecessor = policy.get("provenance", {}).get("consumed_v1_attempt", {})
    if (
        predecessor.get("case_id") != V1_CASE_ID
        or predecessor.get("identity_consumed") is not True
        or predecessor.get("retry_forbidden") is not True
        or predecessor.get("output_exported") is not False
        or predecessor.get("receipts") != V1_PREDECESSOR_RECEIPTS
    ):
        raise GateError("consumed v1 solver predecessor provenance differs")

    completed_v2 = policy.get("provenance", {}).get("completed_c1_v2_attempt", {})
    if (
        completed_v2.get("case_id") != V2_CASE_ID
        or completed_v2.get("identity_consumed") is not True
        or completed_v2.get("retry_forbidden") is not True
        or completed_v2.get("output_exported") is not True
        or completed_v2.get("receipts") != V2_PREDECESSOR_RECEIPTS
    ):
        raise GateError("completed C1 v2 solver predecessor provenance differs")

    expected_inputs = {
        name: {"sha256": digest, "size_bytes": size}
        for name, digest, size in INPUT_CONTRACT
    }
    if policy.get("input_contract", {}).get("files") != expected_inputs:
        raise GateError("solver fixed input contract differs")
    if policy.get("input_contract", {}).get("c1m_case_semantics") != C1M_CASE_SEMANTICS:
        raise GateError("solver C1M case semantics contract differs")
    command = policy.get("fixed_guest_command", {})
    if command.get("solver_argv") != SOLVER_ARGV or command.get("environment") != ENVIRONMENT:
        raise GateError("solver argv or environment differs")
    helper = expected_artifacts["helper"]
    expected_guest = guest_command(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    if command.get("guest_loader_argv_canonical_sha256") != canonical_hash(expected_guest):
        raise GateError("guest loader argv digest differs")
    isolation = policy.get("isolation", {})
    if (
        isolation.get("host_writable_bind_count") != 0
        or isolation.get("gpu_device_nodes") != []
        or isolation.get("network") != "new_empty_network_namespace"
        or isolation.get("guest_work_tmpfs_bytes") != TMPFS_BYTES
    ):
        raise GateError("solver isolation contract differs")
    output = policy.get("output_contract", {})
    if output.get("exact_file_count") != 30 or output.get("host_export") != "post_cleanup_uid1000_dirfd_nofollow_o_excl_only":
        raise GateError("solver output/export contract differs")
    resources = policy.get("resources", {})
    expected_resources = {
        "wall_timeout_seconds": 1800,
        "outer_conduit_timeout_seconds": 1830,
        "cpu_seconds": 1200,
        "address_space_bytes": 2_147_483_648,
        "process_limit": 16,
        "open_file_limit": 256,
        "file_size_limit_bytes": 16_777_216,
        "core_dump_bytes": 0,
        "guest_work_tmpfs_bytes": TMPFS_BYTES,
        "guest_output_total_limit_bytes": OUTPUT_TOTAL_LIMIT,
        "guest_console_limit_bytes": OUTPUT_CONSOLE_LIMIT,
    }
    if resources != expected_resources:
        raise GateError("solver resource contract differs")
    lifecycle = policy.get("profile_lifecycle", {})
    if (
        lifecycle.get("known_quiet_deny_rules") != list(KNOWN_QUIET_DENY_RULES)
        or lifecycle.get("known_quiet_deny_origins") != KNOWN_QUIET_DENY_ORIGINS
        or lifecycle.get("denial_visibility_boundary") != DENIAL_VISIBILITY_BOUNDARY
        or lifecycle.get("journal_success") != "closed_zero_unexpected_logged_DENIED_records_same_boot_no_overflow"
    ):
        raise GateError("solver known quiet deny or visibility boundary differs")
    observed_tools: dict[str, Any] = {}
    if verify_tools:
        for name, (path, digest) in TOOLS.items():
            observed = sha256_file(path, limit=8 * 1024 * 1024)
            if observed != digest:
                raise GateError(f"trusted system tool differs: {name}")
            observed_tools[name] = {"path": str(path), "sha256": observed}
    harness_raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    harness = policy.get("lifecycle_harness", {})
    if (
        hashlib.sha256(harness_raw).hexdigest() != harness.get("sha256")
        or len(harness_raw) != harness.get("size_bytes")
    ):
        raise GateError("frozen v11 lifecycle harness differs")
    bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_file(POLICY_PATH)},
        "artifacts": observed_artifacts,
        "profile": profile,
        "tools": observed_tools,
        "lifecycle_harness": {"path": str(HARNESS_PATH), "sha256": harness["sha256"], "size_bytes": len(harness_raw)},
        "execution_performed": False,
    }


def read_identity_status(path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    return {
        "uid": [int(value) for value in fields["Uid"].split()],
        "gid": [int(value) for value in fields["Gid"].split()],
        "groups": [int(value) for value in fields.get("Groups", "").split()],
        "capabilities": {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")},
        "no_new_privs": int(fields["NoNewPrivs"]),
    }


def verify_child_identity(status: Mapping[str, Any] | None = None) -> dict[str, Any]:
    observed = dict(read_identity_status() if status is None else status)
    if observed.get("uid") != [HOST_UID] * 4 or observed.get("gid") != [HOST_GID] * 4 or observed.get("groups") != []:
        raise GateError("host gate UID/GID/group identity differs")
    caps = observed.get("capabilities")
    if not isinstance(caps, dict) or set(caps) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"} or any(caps.values()):
        raise GateError("host gate capabilities are not all zero")
    if observed.get("no_new_privs") != 1:
        raise GateError("host gate NoNewPrivs differs")
    return observed


def verify_snapshot_runtime() -> dict[str, Any]:
    if not IN_SNAPSHOT or SCRIPT_PATH.parent != SNAPSHOT_ROOT or os.environ.get(SNAPSHOT_ENV) != str(SNAPSHOT_ROOT):
        raise GateError("internal-run requires the exact solver snapshot")
    root = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0 or stat.S_IMODE(root.st_mode) != 0o555:
        raise GateError("solver snapshot directory metadata differs")
    for path in (SCRIPT_PATH, POLICY_PATH, SCHEMA_PATH, PROFILE_PATH, HELPER_PATH, SUPERVISOR_PATH, HARNESS_PATH):
        metadata = os.lstat(path)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o444
        ):
            raise GateError(f"solver snapshot artifact metadata differs: {path.name}")
    return {"root": str(SNAPSHOT_ROOT), "uid": 0, "gid": 0, "mode": "0555"}


def consume_root_admission_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[ADMISSION_FD_ENV])
        expected = os.environ[ADMISSION_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise GateError("root admission FD capability is absent") from exc
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (
            descriptor < 3
            or len(expected) != 64
            or not stat.S_ISFIFO(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or stat.S_IMODE(metadata.st_mode) != 0o600
            or not os.get_inheritable(descriptor)
        ):
            raise GateError("root admission pipe metadata differs")
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
    if observed != expected:
        raise GateError("root admission capability digest differs")
    os.environ.pop(ADMISSION_FD_ENV, None)
    os.environ.pop(ADMISSION_SHA256_ENV, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "sha256": observed, "size_bytes": 32}


def build_input_frame(helper_bytes: bytes, inputs: Mapping[str, bytes]) -> bytes:
    expected_names = [name for name, _digest, _size in INPUT_CONTRACT]
    if set(inputs) != set(expected_names):
        raise GateError("solver input frame set differs")
    policy = read_json_object(POLICY_PATH)
    helper = policy["trusted_artifacts"]["helper"]
    if len(helper_bytes) != helper["size_bytes"] or hashlib.sha256(helper_bytes).hexdigest() != helper["sha256"]:
        raise GateError("solver helper identity differs")
    parts = [HELPER_MAGIC, helper_bytes, INPUT_MAGIC]
    for name, digest, size in INPUT_CONTRACT:
        raw = inputs[name]
        if len(raw) != size or hashlib.sha256(raw).hexdigest() != digest:
            raise GateError(f"solver framed input differs: {name}")
        parts.append(raw)
    frame = b"".join(parts)
    if len(frame) > INPUT_FRAME_LIMIT:
        raise GateError("solver input frame exceeds fixed ceiling")
    return frame


def _stop_group(process: subprocess.Popen[bytes]) -> None:
    process.poll()

    def exists() -> bool:
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + 10
    while exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    if exists():
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 3
    while exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    if exists():
        raise GateError("outer guest process group remains after cleanup")


def run_bounded_guest(argv: list[str], input_frame: bytes) -> tuple[int, bytes, bytes]:
    process = subprocess.Popen(
        argv,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/",
        env={"HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C", "LANG": "C", "TZ": "UTC0"},
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
    for name, descriptor in descriptors.items():
        selector.register(descriptor, selectors.EVENT_WRITE if name == "stdin" else selectors.EVENT_READ, name)
    position = 0
    stdout = bytearray()
    stderr = bytearray()
    deadline = time.monotonic() + 1_820
    try:
        while selector.get_map():
            if time.monotonic() >= deadline:
                _stop_group(process)
                raise GateError("solver guest conduit exceeded 1820 seconds")
            events = selector.select(timeout=0.2)
            for key, _mask in events:
                descriptor = key.fd
                channel = key.data
                if channel == "stdin":
                    try:
                        count = os.write(descriptor, input_frame[position : position + 65_536])
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
                    (process.stdout if channel == "stdout" else process.stderr).close()
                    continue
                target = stdout if channel == "stdout" else stderr
                ceiling = GUEST_STDOUT_LIMIT if channel == "stdout" else GUEST_STDERR_LIMIT
                room = max(0, ceiling + 1 - len(target))
                target.extend(block[:room])
                if len(block) > room or len(target) > ceiling:
                    _stop_group(process)
                    raise GateError(f"guest {channel} exceeded hard byte ceiling")
    finally:
        selector.close()
        if process.poll() is None:
            _stop_group(process)
        for stream in (process.stdin, process.stdout, process.stderr):
            try:
                stream.close()
            except (BrokenPipeError, OSError):
                pass
    returncode = process.wait()
    if position != len(input_frame):
        raise GateError("guest did not consume complete solver input frame")
    return returncode, bytes(stdout), bytes(stderr)


def internal_child(frame: bytes) -> tuple[int, bytes, bytes]:
    verify_snapshot_runtime()
    verify_child_identity()
    consume_root_admission_capability()
    verify_review_artifacts(verify_tools=True)
    policy = read_json_object(POLICY_PATH)
    helper = policy["trusted_artifacts"]["helper"]
    expected_size = len(HELPER_MAGIC) + helper["size_bytes"] + len(INPUT_MAGIC) + sum(size for _name, _digest, size in INPUT_CONTRACT)
    if len(frame) != expected_size:
        raise GateError("solver input frame total length differs")
    argv = bwrap_argv(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    return run_bounded_guest(argv, frame)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("internal-run", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            review = verify_review_artifacts(verify_tools=True)
            print(json.dumps({"status": "PASS_SOLVER_CPU_V3_STATIC_EXECUTION_NOT_PERFORMED", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        frame = sys.stdin.buffer.read(INPUT_FRAME_LIMIT + 1)
        if len(frame) > INPUT_FRAME_LIMIT or sys.stdin.buffer.read(1) != b"":
            raise GateError("outer solver input frame exceeds fixed ceiling")
        returncode, stdout, stderr = internal_child(frame)
        if returncode != 0:
            sys.stderr.buffer.write(stderr)
            if stdout:
                sys.stderr.write("\nNO_GO: discarded nonzero-rc guest stdout bytes\n")
            sys.stderr.buffer.flush()
            return returncode
        if stderr or len(stdout) < OUTPUT_HEADER_BYTES:
            raise GateError("successful solver guest did not return one clean frame")
        sys.stdout.buffer.write(stdout)
        sys.stdout.buffer.flush()
        return 0
    except (GateError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        stream = sys.stderr if arguments.command == "internal-run" else sys.stdout
        print(json.dumps({"status": "SOLVER_CPU_V3_NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=stream)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
