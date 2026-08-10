#!/usr/bin/env python3
"""Static and child-side gate for the U3 C1 GenCase v2 runtime design.

v2 is deliberately NO-GO until a separate stdin-only harmless AppArmor
lowercase-rpx/NNP/signal/timeout probe has passed and an independently reviewed successor
freezes the observed mount rules.  ``self-check`` is read-only.  The internal
child entry point is implemented for review and mock testing but refuses the
current NO-GO policy before it can launch bubblewrap or GenCase.
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
SNAPSHOT_ENV = "R8_LIQUID_U3_GENCASE_V2_SNAPSHOT"
SNAPSHOT_DIR = Path(os.environ[SNAPSHOT_ENV]) if SNAPSHOT_ENV in os.environ else None

CASE_ID = "u3_c1_gencase_v2_20260807T140057Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-gencase-bootstrap-v2"
RUNTIME_PROFILE = "r8-liquid-u3-gencase-runtime-v2"

POLICY_REPOSITORY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v2.json"
SCHEMA_REPOSITORY_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v2.json"
PROFILE_REPOSITORY_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v2.profile"
HELPER_REPOSITORY_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gencase_bootstrap_helper_v2.py")
SUPERVISOR_REPOSITORY_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gencase_supervisor_v2.py")

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

SEED_ID = "u3_c1_gencase_seed_v2_20260807T135341Z"
SEED_ROOT = Path(f"/home/zrj/scout_liquid_lab/dependency/runtime/{SEED_ID}.partial")
SEED_INPUT_ROOT = SEED_ROOT / "input"
SEED_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json")
ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")

INPUT_CONTRACT: dict[str, tuple[str, int]] = {
    "GenCase_linux64": ("a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226", 5_809_384),
    "DsphConfig.xml": ("0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd", 293),
    "C1_static_Def.xml": ("d738d303300b3f339ac37ca3a604fbe8dff9e034d8c8ad7aacb2654434525819", 5_714),
}
EXPECTED_OUTPUTS = ("C1_static.bi4", "C1_static.xml")
TMPFS_OUTPUT_BYTES = 67_108_864
GUEST_STDOUT_LIMIT = 48 + 65_536 + 16_777_216 + 524_288 + 1_048_576
GUEST_STDERR_LIMIT = 1_114_112
HELPER_MAGIC = b"R8C1HELPERV2\0\0\0\0\0"
INPUT_MAGIC = b"R8C1INPUTV2\0\0\0\0\0"

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
        "g={'__name__':'__main__','__file__':'<r8-liquid-u3-gencase-helper-v2>'}\n"
        "exec(compile(h,g['__file__'],'exec',dont_inherit=True,optimize=2),g,g)\n"
    )


def guest_command(*, helper_size: int, helper_sha256: str) -> list[str]:
    return [
    "/usr/bin/setpriv",
    "--clear-groups",
    "--inh-caps=-all",
    "--ambient-caps=-all",
    "--bounding-set=-all",
    "--no-new-privs",
    "--",
    "/usr/bin/python3.12",
    "-I",
    "-B",
    "-c",
    guest_loader(helper_size=helper_size, helper_sha256=helper_sha256),
    ]
GENCASE_ARGV = [
    "/work/runtime/GenCase_linux64",
    "/work/runtime/C1_static_Def",
    "/work/output/C1_static",
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
        raise GateError("v2 AppArmor draft digest differs")
    text = raw.decode("utf-8")
    effective = _effective_profile(text)
    bootstrap_header = f"profile {BOOTSTRAP_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    runtime_header = f"profile {RUNTIME_PROFILE} flags=(attach_disconnected,mediate_deleted)"
    if effective.count(bootstrap_header) != 1 or effective.count(runtime_header) != 1:
        raise GateError("v2 AppArmor label identity differs")
    transition = f"/work/runtime/GenCase_linux64 rpx -> {RUNTIME_PROFILE},"
    if effective.count(transition) != 1:
        raise GateError("GenCase must have exactly one bootstrap-to-runtime transition")
    bootstrap, runtime = effective.split(runtime_header, 1)
    for required in (
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
        f"signal (send) set=(term,kill,exists) peer={RUNTIME_PROFILE},",
    ):
        if required not in bootstrap:
            raise GateError(f"bootstrap profile lacks required narrow rule: {required}")
    if "/work/export" in bootstrap:
        raise GateError("bootstrap profile must have no host export path")
    for forbidden in (
        "/work/export",
        "/work/transport",
        "userns ",
        "mount ",
        "remount ",
        "pivot_root ",
        "umount ",
        "capability ",
        "network ",
        "/proc",
        "/dev",
        "/home/",
        "/usr/bin/",
        "ptrace ",
        "change_profile",
        " rpx",
        " rix,",
        " ix,",
    ):
        if forbidden in runtime:
            raise GateError(f"runtime profile contains forbidden authority: {forbidden}")
    for required in (
        "/work/runtime/GenCase_linux64 rm,",
        "/work/runtime/DsphConfig.xml r,",
        "/work/runtime/C1_static_Def.xml r,",
        "/work/output/** rw,",
        f"signal (receive) set=(term,kill,exists) peer={BOOTSTRAP_PROFILE},",
        "signal (receive) set=(term,kill,exists) peer=unconfined,",
    ):
        if required not in runtime:
            raise GateError(f"runtime profile lacks required narrow rule: {required}")
    for forbidden_global in (
        "flags=(unconfined)",
        "/home/zrj/scout_ws",
        "/home/zrj/ros2_ws",
        "/opt/ros/jazzy",
        "/dev/nvidia",
        "ux,",
        "Ux,",
    ):
        if forbidden_global in effective:
            raise GateError(f"profile violates a global forbidden boundary: {forbidden_global}")
    # v2 remains a draft precisely because probe-derived mount rules are absent.
    if any(token in effective for token in ("mount ", "remount ", "pivot_root ", "umount ")):
        raise GateError("v2 draft unexpectedly claims unprobed bwrap mount grammar")
    parser_result: dict[str, Any] = {"executed": False}
    if run_parser:
        require_tool("apparmor_parser")
        completed = subprocess.run(
            ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", str(PROFILE_PATH)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd="/",
            env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
            timeout=15,
            check=False,
        )
        if completed.returncode != 0 or len(completed.stdout) > 16_384 or len(completed.stderr) > 16_384:
            raise GateError("AppArmor draft parser-only check failed")
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
        "production_load_allowed": False,
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


def verify_review_artifacts(*, run_parser: bool = True) -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise GateError("v2 policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-gencase-execution-policy-v2",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GENCASE_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GENCASE_EXECUTION_V2",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "execution_scope": "one_exact_c1_gencase_v2_attempt_after_separate_probe_and_go",
        "allowed_gate_commands": ["self-check"],
        "status": "NO_GO_V2_PENDING_HARMLESS_RPX_SIGNAL_TIMEOUT_PROBE_AND_INDEPENDENT_STATIC_GO",
        "next_allowed_stage": "DESIGN_AND_RUN_SEPARATE_HARMLESS_RPX_SIGNAL_TIMEOUT_PROBE_WITHOUT_GENCASE",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise GateError(f"v2 policy field differs: {key}")
    if policy.get("trusted_tools") != trusted_tool_policy():
        raise GateError("v2 trusted-tool policy differs")
    observed_tools = {name: require_tool(name) for name in TRUSTED_TOOLS}
    artifacts = policy.get("trusted_artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != set(_artifact_paths()):
        raise GateError("v2 trusted-artifact set differs")
    observed_artifacts: dict[str, dict[str, str]] = {}
    for name, path in _artifact_paths().items():
        expected = artifacts[name]
        if expected.get("path") != policy_artifact_relative_path(name):
            raise GateError(f"v2 artifact path differs: {name}")
        digest = sha256_regular_file(path, limit=2 * 1024 * 1024)
        if digest != expected.get("sha256"):
            raise GateError(f"v2 artifact digest differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": digest}
    frozen = policy["frozen_attempt"]
    expected_frozen = {
        "case_id": CASE_ID,
        "attempt_root": str(ATTEMPT_ROOT),
        "output_root": str(OUTPUT_ROOT),
        "execution_receipt": str(EXECUTION_RECEIPT),
        "lifecycle_receipt": str(LIFECYCLE_RECEIPT),
        "lifecycle_failure_receipt": str(LIFECYCLE_FAILURE_RECEIPT),
        "console_log": str(CONSOLE_LOG),
        "snapshot_root": str(SNAPSHOT_ROOT),
        "bootstrap_profile": BOOTSTRAP_PROFILE,
        "runtime_profile": RUNTIME_PROFILE,
    }
    if frozen != expected_frozen:
        raise GateError("v2 frozen attempt identity differs")
    transport = policy["immutable_transport"]
    if transport.get("input_delivery") != "fixed_no_declared_lengths_stdin_frame_to_hash_pinned_in_memory_helper":
        raise GateError("v2 framed stdin transport differs")
    if transport.get("output_delivery") != "success_only_single_bounded_stdout_binary_frame_then_uid1000_O_EXCL":
        raise GateError("v2 framed stdout transport differs")
    if policy["isolation"].get("guest_output_tmpfs_byte_ceiling") != TMPFS_OUTPUT_BYTES or policy["isolation"].get("guest_output_tmpfs_inode_ceiling") != "UNCLAIMED_BWRAP_0_9_HAS_NO_NR_INODES_OPTION":
        raise GateError("v2 tmpfs byte/inode claim differs")
    helper = policy["trusted_artifacts"]["helper"]
    expected_guest = guest_command(helper_size=helper["size_bytes"], helper_sha256=helper["sha256"])
    if policy["fixed_guest_command"].get("bootstrap_helper_argv") != expected_guest or policy["fixed_guest_command"].get("gencase_argv") != GENCASE_ARGV:
        raise GateError("v2 fixed guest command differs")
    if policy["required_harmless_probes"].get("admission") != "REQUIRED_NOT_YET_EXECUTED":
        raise GateError("v2 must remain NO-GO until the harmless probe exists")
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": observed_artifacts["schema"]["sha256"]},
        "artifacts": observed_artifacts,
        "profile": verify_profile(run_parser=run_parser),
        "trusted_tools": observed_tools,
        "status": policy["status"],
    }


def policy_artifact_relative_path(name: str) -> str:
    return {
        "gate": "scripts/r8_liquid_target_u3_gencase_gate_v2.py",
        "helper": "scripts/r8_liquid_u3_gencase_bootstrap_helper_v2.py",
        "supervisor": "scripts/r8_liquid_target_u3_gencase_supervisor_v2.py",
        "profile": "config/apparmor_drafts/r8-liquid-u3-gencase-v2.profile",
        "schema": "schema/target_host_u3_gencase_execution_policy_v2.json",
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
    expected_no_new_privs: int = 0,
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
        "/usr/bin/timeout", "--foreground", "--kill-after=5s", "120s",
        "/usr/bin/aa-exec", "-p", BOOTSTRAP_PROFILE, "--",
        "/usr/bin/bwrap",
        "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--uid", "0", "--gid", "0", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-u3-gencase-v2",
        "--clearenv",
        "--setenv", "HOME", "/nonexistent",
        "--setenv", "PATH", "/usr/bin",
        "--setenv", "LC_ALL", "C.UTF-8",
        "--setenv", "LANG", "C.UTF-8",
        "--setenv", "TZ", "UTC",
        "--ro-bind", "/usr", "/usr",
        "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64",
        "--size", str(TMPFS_OUTPUT_BYTES), "--tmpfs", "/work",
        "--proc", "/proc",
        "--dir", "/work/output",
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
    for forbidden in ("--dev", "--share-net", "/home/zrj/scout_ws", "/opt/ros/jazzy", "/dev/nvidia0"):
        if forbidden in argv:
            raise GateError(f"bwrap argv violates a forbidden boundary: {forbidden}")
    if argv[-len(expected_guest_command):] != expected_guest_command:
        raise GateError("fixed guest helper command differs")


def refuse_runtime_while_v2_is_no_go() -> None:
    policy = read_json_object(POLICY_PATH)
    if policy.get("status") != "ADMITTED_SINGLE_GENCASE_RUNTIME_V2_AFTER_PROBES":
        raise GateError("v2 runtime is intentionally NO-GO until separate harmless probes and a new independent GO")


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
        env={"PATH": "/usr/bin:/usr/sbin", "HOME": "/nonexistent", "LC_ALL": "C.UTF-8", "LANG": "C.UTF-8", "TZ": "UTC"},
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
    # This check occurs before tool execution or any host mutation.
    refuse_runtime_while_v2_is_no_go()
    verify_child_identity()
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
            print(json.dumps({"status": "PASS_STATIC_DRAFT_NO_GO", "review": review}, ensure_ascii=False, sort_keys=True))
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
        if stderr or len(stdout) < 48:
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
