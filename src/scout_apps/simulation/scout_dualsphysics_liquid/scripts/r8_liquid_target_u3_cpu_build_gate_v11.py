#!/usr/bin/env python3
"""One-shot U3 v11 source-copy and CPU-build gate for this MSI host.

v11 reuses the byte-pinned v10 implementation as an immutable safety core and
changes only its reviewed configuration: a fresh attempt identity and the
synthetic guest ``/lib -> usr/lib`` link needed by GNU ld's absolute linker
scripts.  It never binds host ``/lib`` and retains the v10 no-network,
no-GPU, source-read-only and no-artifact-execution boundaries.
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import stat
import sys
from pathlib import Path
from types import ModuleType
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
CORE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_cpu_build_gate.py")
CORE_SHA256 = "04945b212ad15a0aa9e59d3e24cc996f82b770e7d85f1a7fb50886370f6e0c67"

BUILD_ID = "u3_source_cpu_build_20260806T184504Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v11.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v11.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v11.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v11.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v11"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v11"
COPY_PROFILE_SHA256 = "cf812644498c836d097ea66c90427dafc881758eac8da275c653b06f9a59e67e"
BUILD_PROFILE_SHA256 = "9ed658c2ddc8169c6225e30fc5946ff4b37f8533c38802530f0633ae69c6997a"

EXPECTED_SYMLINKS = [
    {"link": "/lib", "target": "usr/lib"},
    {"link": "/lib64", "target": "usr/lib64"},
]

# The gate is executed from a user-writable workspace but must never leave a
# Python bytecode artifact there; this is set before importing the pinned core.
sys.dont_write_bytecode = True


def sha256_regular_file(path: Path, *, limit: int = 4 * 1024 * 1024) -> str:
    """Hash a pinned local review file without following links."""

    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeError(f"unsafe frozen-core file: {path}")
        if metadata.st_size > limit:
            raise RuntimeError(f"frozen-core file is unexpectedly large: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def load_frozen_core() -> ModuleType:
    if sha256_regular_file(CORE_PATH) != CORE_SHA256:
        raise RuntimeError("the byte-pinned v10 safety core differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_cpu_build_v10_frozen_core", CORE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the byte-pinned v10 safety core")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = load_frozen_core()


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def verify_review_artifacts_v11() -> dict[str, Any]:
    """Verify the full v11 contract before either state-changing phase."""

    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise core.GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v11",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V11",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V11_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise core.GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise core.GateError("policy command surface differs")
    handoff = policy.get("outer_privilege_handoff")
    if not isinstance(handoff, dict) or handoff.get("must_drop_before_aa_exec") is not True:
        raise core.GateError("policy does not require the pre-aa-exec privilege drop")
    if handoff.get("required_uid") != 1000 or handoff.get("required_gid") != 1000:
        raise core.GateError("policy privilege-drop identity differs")
    if handoff.get("required_supplementary_group_count") != 0:
        raise core.GateError("policy does not require empty supplementary groups")
    if handoff.get("only_privileged_child_command", [None] * 7)[6] != "r8_liquid_target_u3_cpu_build_gate_v11.py":
        raise core.GateError("policy names an unexpected privileged child gate")
    attempt = policy.get("frozen_attempt")
    if not isinstance(attempt, dict) or attempt.get("build_id") != BUILD_ID:
        raise core.GateError("policy build attempt differs")
    if attempt.get("attempt_root") != str(ATTEMPT_ROOT) or attempt.get("output_root") != str(OUTPUT_ROOT):
        raise core.GateError("policy output path differs")
    if attempt.get("source_copy_profile_sha256") != COPY_PROFILE_SHA256:
        raise core.GateError("policy source-copy profile digest differs")
    if attempt.get("build_profile_sha256") != BUILD_PROFILE_SHA256:
        raise core.GateError("policy build profile digest differs")
    if (
        attempt.get("source_copy_profile") != COPY_PROFILE
        or attempt.get("build_profile") != BUILD_PROFILE
        or attempt.get("source_copy_profile_path") != "config/apparmor_drafts/r8-liquid-u3-source-copy-v11.profile"
        or attempt.get("build_profile_path") != "config/apparmor_drafts/r8-liquid-u3-cpu-build-v11.profile"
    ):
        raise core.GateError("policy profile identity differs")
    source = policy.get("source_input")
    if not isinstance(source, dict) or source.get("root") != str(core.SOURCE_ROOT):
        raise core.GateError("policy source path differs")
    if source.get("receipt") != str(core.SOURCE_RECEIPT) or source.get("receipt_sha256") != core.SOURCE_RECEIPT_SHA256:
        raise core.GateError("policy source receipt differs")
    if source.get("file_count") != core.SOURCE_FILE_COUNT or source.get("total_bytes") != core.SOURCE_TOTAL_BYTES:
        raise core.GateError("policy source inventory differs")
    if policy.get("generated_wrapper", {}).get("content") != core.WRAPPER_CONTENT:
        raise core.GateError("policy generated wrapper differs")
    isolation = policy.get("isolation", {})
    if isolation.get("synthetic_symlinks") != EXPECTED_SYMLINKS:
        raise core.GateError("policy synthetic-link boundary differs")
    if isolation.get("host_read_only_binds") != ["/usr", "sealed source during copy only"]:
        raise core.GateError("policy host read-only bind boundary differs")
    if isolation.get("host_writable_binds") != ["one exact new output root"] or isolation.get("host_writable_bind_count") != 1:
        raise core.GateError("policy writable bind boundary differs")
    if isolation.get("guest_proc_and_dev") != "absent" or isolation.get("guest_tmpfs") != ["/work"]:
        raise core.GateError("policy guest proc/dev or tmpfs boundary differs")
    expected_disable_userns = {
        "bubblewrap_package_version": "0.9.0-1ubuntu0.1",
        "proc_path": "/proc/sys/user/max_user_namespaces",
        "open_mode": "O_WRONLY",
        "write_value": "1",
        "write_namespace": "first-level bubblewrap user namespace only",
        "kernel_write_capability": "CAP_SYS_RESOURCE in that first-level user namespace only",
        "host_initial_namespace_postcondition": "exact host value unchanged before and after the child",
    }
    if isolation.get("disable_userns_mechanism") != expected_disable_userns:
        raise core.GateError("policy disable-userns namespace-scoped write contract differs")
    if isolation.get("required_profile_capabilities") != ["sys_admin", "sys_ptrace", "sys_resource", "setpcap", "net_admin"]:
        raise core.GateError("policy capability surface differs")
    if policy.get("invariants", {}).get("no_host_initial_namespace_sysctl_or_apparmor_global_setting_change") is not True:
        raise core.GateError("policy does not forbid host/global sysctl changes")
    if policy.get("build_command", [None])[0] != "/usr/bin/make" or "SHELL=/usr/bin/dash" not in policy.get("build_command", []):
        raise core.GateError("policy build executable differs")
    effective_profiles: dict[Path, str] = {}
    for profile_path, expected_sha in ((COPY_PROFILE_PATH, COPY_PROFILE_SHA256), (BUILD_PROFILE_PATH, BUILD_PROFILE_SHA256)):
        if core.sha256_file(profile_path, limit=128 * 1024) != expected_sha:
            raise core.GateError(f"reviewed profile digest differs: {profile_path}")
        text = profile_path.read_text(encoding="utf-8")
        if "REVIEWED_SINGLE_ATTEMPT_ONLY" not in text or "flags=(attach_disconnected,mediate_deleted)" not in text:
            raise core.GateError(f"reviewed profile header differs: {profile_path}")
        effective = _effective_profile(text)
        for token in core.FORBIDDEN_PROFILE_TOKENS:
            if token in effective:
                raise core.GateError(f"reviewed profile violates a forbidden boundary: {token}")
        effective_profiles[profile_path] = effective
        proc_sys_writes = [line.strip() for line in effective.splitlines() if "/proc/sys" in line and " w" in line]
        if proc_sys_writes != ["/proc/sys/user/max_user_namespaces w,"]:
            raise core.GateError("reviewed profile proc-sys write surface differs")
        capabilities = [line.strip() for line in effective.splitlines() if line.strip().startswith("capability ")]
        if capabilities != ["capability sys_admin,", "capability sys_ptrace,", "capability sys_resource,", "capability setpcap,", "capability net_admin,"]:
            raise core.GateError("reviewed profile capability surface differs")
        link_rules = [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")]
        if link_rules != ["/newroot/lib wl,", "/newroot/lib64 wl,"]:
            raise core.GateError("reviewed profile synthetic-link creation surface differs")
        if "/oldroot/lib" in effective or " -> /newroot/lib" in effective:
            raise core.GateError("reviewed profile adds an impermissible host /lib bind")
    build_rules = {line.strip() for line in effective_profiles[BUILD_PROFILE_PATH].splitlines() if line.strip()}
    required_header_rules = {"/usr/include/ r,", "/usr/include/** r,"}
    if not required_header_rules <= build_rules:
        raise core.GateError("v11 build profile lacks the exact read-only standard-header rules")
    if any(line.startswith("/usr/include") and line not in required_header_rules for line in build_rules):
        raise core.GateError("v11 build profile expands the standard-header access surface")
    if "/usr/include" in effective_profiles[COPY_PROFILE_PATH]:
        raise core.GateError("source-copy profile must not gain compiler header access")
    if "DualSPHysics5.4CPU_linux64 rix" in BUILD_PROFILE_PATH.read_text(encoding="utf-8"):
        raise core.GateError("build profile permits the output artifact")
    for phase in ("source-copy", "build"):
        argv = bwrap_prefix_v11(phase=phase)
        ro_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--ro-bind"]
        expected_ro_pairs = [["/usr", "/usr"]]
        if phase == "source-copy":
            expected_ro_pairs.append([str(core.SOURCE_ROOT), "/work/input"])
        if ro_pairs != expected_ro_pairs:
            raise core.GateError("bwrap host read-only bind boundary differs")
        pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--symlink"]
        if pairs != [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]]:
            raise core.GateError("bwrap synthetic-link boundary differs")
        bind_pairs = [
            argv[index + 1 : index + 3]
            for index, item in enumerate(argv[:-2])
            if item in {"--ro-bind", "--bind"}
        ]
        if any(pair[0] in {"/lib", "/lib64"} or pair[1] in {"/lib", "/lib64"} for pair in bind_pairs):
            raise core.GateError("bwrap must not bind a host lib path")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": core.sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": core.canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": core.sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "frozen_v10_core": {"path": str(CORE_PATH), "sha256": CORE_SHA256},
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
    }


def bwrap_prefix_v11(*, phase: str) -> list[str]:
    """Return the v10 fixed argv with only the reviewed guest /lib link added."""

    if phase == "source-copy":
        profile = COPY_PROFILE
        timeout_seconds = "300s"
        address_space = "1073741824:1073741824"
        cpu_limit = "120:125"
        child = [
            "/usr/bin/env", "-i", "HOME=/nonexistent", "PATH=/usr/bin", "LC_ALL=C.UTF-8", "LANG=C.UTF-8", "TZ=UTC",
            "/usr/bin/prlimit", f"--as={address_space}", f"--cpu={cpu_limit}", "--nproc=4096:4096", "--nofile=1024:1024", "--fsize=2147483648:2147483648", "--core=0:0", "--",
            "/usr/bin/cp", "--recursive", "--no-preserve=mode,ownership", "--", "/work/input/.", "/work/output/buildtree/src/source/",
        ]
        extra_binds = ["--ro-bind", str(core.SOURCE_ROOT), "/work/input"]
        chdir = "/work"
        hostname = "r8-liquid-u3-source-copy"
    elif phase == "build":
        profile = BUILD_PROFILE
        timeout_seconds = "3600s"
        address_space = "8589934592:8589934592"
        cpu_limit = "3600:3605"
        child = [
            "/usr/bin/env", "-i", "HOME=/nonexistent", "PATH=/usr/bin", "LC_ALL=C.UTF-8", "LANG=C.UTF-8", "TZ=UTC",
            "/usr/bin/prlimit", f"--as={address_space}", f"--cpu={cpu_limit}", "--nproc=4096:4096", "--nofile=1024:1024", "--fsize=2147483648:2147483648", "--core=0:0", "--",
            "/usr/bin/make", "--no-builtin-rules", "--no-builtin-variables", "-f", core.WRAPPER_NAME, "-j4", "SHELL=/usr/bin/dash",
            "CC=/usr/bin/x86_64-linux-gnu-g++-13", "USE_DEBUG=NO", "USE_FAST_MATH=NO", "USE_NATIVE_CPU_OPTIMIZATIONS=NO",
            "COMPILE_CHRONO=NO", "COMPILE_WAVEGEN=NO", "COMPILE_MOORDYNPLUS=NO", "LIBS_DIRECTORIES=",
            "EXECS_DIRECTORY=/work/output/artifacts", "/work/output/artifacts/DualSPHysics5.4CPU_linux64",
        ]
        extra_binds = []
        chdir = "/work/output/buildtree/src/source"
        hostname = "r8-liquid-u3-cpu-build"
    else:
        raise core.GateError("unknown bwrap phase")
    return [
        "/usr/bin/timeout", "--foreground", "--kill-after=5s", timeout_seconds,
        "/usr/bin/aa-exec", "-p", profile, "--",
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled", "--hostname", hostname, "--clearenv",
        "--setenv", "HOME", "/nonexistent", "--setenv", "PATH", "/usr/bin", "--setenv", "LC_ALL", "C.UTF-8", "--setenv", "LANG", "C.UTF-8", "--setenv", "TZ", "UTC",
        "--uid", "0", "--gid", "0", "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--tmpfs", "/work",
        *extra_binds,
        "--bind", str(OUTPUT_ROOT), "/work/output", "--chdir", chdir, "--", *child,
    ]


# The imported functions resolve module globals dynamically.  Freeze the core
# first, then substitute every version-specific value and the two reviewed
# functions before handing control to its unchanged one-shot state machine.
core.__file__ = str(SCRIPT_PATH)
core.BUILD_ID = BUILD_ID
core.LIQUID_ROOT = LIQUID_ROOT
core.ATTEMPT_ROOT = ATTEMPT_ROOT
core.OUTPUT_ROOT = OUTPUT_ROOT
core.COPY_RECEIPT = COPY_RECEIPT
core.BUILD_RECEIPT = BUILD_RECEIPT
core.POLICY_PATH = POLICY_PATH
core.SCHEMA_PATH = SCHEMA_PATH
core.COPY_PROFILE_PATH = COPY_PROFILE_PATH
core.BUILD_PROFILE_PATH = BUILD_PROFILE_PATH
core.COPY_PROFILE = COPY_PROFILE
core.BUILD_PROFILE = BUILD_PROFILE
core.COPY_PROFILE_SHA256 = COPY_PROFILE_SHA256
core.BUILD_PROFILE_SHA256 = BUILD_PROFILE_SHA256
core.verify_review_artifacts = verify_review_artifacts_v11
core.bwrap_prefix = bwrap_prefix_v11

# Expose the reviewed core API so tests and the outer privileged runner can use
# this gate exactly as they used v10.
for _name in dir(core):
    if not _name.startswith("_") and _name not in globals():
        globals()[_name] = getattr(core, _name)


if __name__ == "__main__":
    raise SystemExit(core.main())
