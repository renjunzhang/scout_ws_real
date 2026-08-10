#!/usr/bin/env python3
"""One-shot U3 v12 source-copy and CPU-build gate for this MSI host.

v11 was safely stopped before bwrap because its wrapper left two derived v10
output paths unchanged.  v12 pins that failed wrapper byte-for-byte, retains
its exact sandbox argv and policy boundaries, and changes only the new attempt
identity plus the previously omitted OUTPUT_SOURCE and ARTIFACT_PATH rebinding.
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
PREVIOUS_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_cpu_build_gate_v11.py")
PREVIOUS_GATE_SHA256 = "857c9ed0cb07a1e118c99febfafdc743dde72be796c9735a2adac7f072884623"
PREVIOUS_POLICY_SHA256 = "e6e4bb6d5d998a34f02277d0aba9696eb477d639813700a57c5f0dde2148c2bb"

BUILD_ID = "u3_source_cpu_build_20260807T011031Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v12.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v12.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v12.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v12.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v12"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v12"
COPY_PROFILE_SHA256 = "e7e8ca57235bf9d6fe942b0076d3090f2a016df133e1753196c622542bf9b436"
BUILD_PROFILE_SHA256 = "c24c3cd77547a53f9d8c772fbd3d90cb3c7c31d66a30208da5d035298c1fb7e7"
EXPECTED_SYMLINKS = [
    {"link": "/lib", "target": "usr/lib"},
    {"link": "/lib64", "target": "usr/lib64"},
]

# Do not create bytecode under the user-writable workspace while importing the
# two pinned review layers.
sys.dont_write_bytecode = True


def sha256_regular_file(path: Path, *, limit: int = 4 * 1024 * 1024) -> str:
    """Hash a frozen local review layer without following links."""

    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeError(f"unsafe frozen review layer: {path}")
        if metadata.st_size > limit:
            raise RuntimeError(f"frozen review layer is unexpectedly large: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def load_frozen_previous_gate() -> ModuleType:
    if sha256_regular_file(PREVIOUS_GATE_PATH) != PREVIOUS_GATE_SHA256:
        raise RuntimeError("the byte-pinned v11 review layer differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_cpu_build_v11_frozen_layer", PREVIOUS_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the byte-pinned v11 review layer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.sha256_regular_file(module.POLICY_PATH, limit=256 * 1024) != PREVIOUS_POLICY_SHA256:
        raise RuntimeError("the byte-pinned v11 policy differs")
    return module


previous = load_frozen_previous_gate()
core = previous.core
OUTPUT_SOURCE = OUTPUT_ROOT / core.OUTPUT_SOURCE_RELATIVE
ARTIFACT_PATH = OUTPUT_ROOT / core.ARTIFACT_RELATIVE


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def bwrap_prefix_v12(*, phase: str) -> list[str]:
    """Reuse the byte-pinned v11 argv, replacing only phase identity/output."""

    argv = previous.bwrap_prefix_v11(phase=phase)
    replacements = {
        previous.COPY_PROFILE: COPY_PROFILE,
        previous.BUILD_PROFILE: BUILD_PROFILE,
        str(previous.OUTPUT_ROOT): str(OUTPUT_ROOT),
    }
    result = [replacements.get(item, item) for item in argv]
    if any(item in {previous.COPY_PROFILE, previous.BUILD_PROFILE, str(previous.OUTPUT_ROOT)} for item in result):
        raise core.GateError("v12 argv retains a v11 profile or output path")
    return result


def verify_review_artifacts_v12() -> dict[str, Any]:
    """Fail closed unless v12 differs from frozen v11 only as reviewed."""

    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    previous_policy = core.read_json_object(previous.POLICY_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise core.GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v12",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V12",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V12_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise core.GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise core.GateError("policy command surface differs")
    handoff = policy.get("outer_privilege_handoff")
    if handoff != {
        "only_privileged_child_command": ["/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups", "--", "/usr/bin/python3", "r8_liquid_target_u3_cpu_build_gate_v12.py", "<fixed_phase>"],
        "must_drop_before_aa_exec": True,
        "required_uid": 1000,
        "required_gid": 1000,
        "required_supplementary_group_count": 0,
        "root_access_to_source_or_output": "forbidden_after_exec",
    }:
        raise core.GateError("policy privilege handoff differs")
    attempt = policy.get("frozen_attempt")
    if attempt != {
        "build_id": BUILD_ID,
        "attempt_root": str(ATTEMPT_ROOT),
        "output_root": str(OUTPUT_ROOT),
        "source_copy_profile": COPY_PROFILE,
        "source_copy_profile_path": "config/apparmor_drafts/r8-liquid-u3-source-copy-v12.profile",
        "build_profile": BUILD_PROFILE,
        "build_profile_path": "config/apparmor_drafts/r8-liquid-u3-cpu-build-v12.profile",
        "source_copy_profile_sha256": COPY_PROFILE_SHA256,
        "build_profile_sha256": BUILD_PROFILE_SHA256,
    }:
        raise core.GateError("policy frozen attempt identity differs")
    for key in (
        "source_input",
        "isolation",
        "resources",
        "source_copy_command",
        "build_command",
        "generated_wrapper",
        "static_artifact_audit",
        "profile_lifecycle",
        "invariants",
    ):
        if policy.get(key) != previous_policy.get(key):
            raise core.GateError(f"v12 widens or differs from frozen v11 policy field: {key}")
    if policy.get("isolation", {}).get("synthetic_symlinks") != EXPECTED_SYMLINKS:
        raise core.GateError("policy synthetic-link boundary differs")
    if policy.get("generated_wrapper", {}).get("content") != core.WRAPPER_CONTENT:
        raise core.GateError("policy generated wrapper differs")
    effective_profiles: dict[Path, str] = {}
    for profile_path, expected_sha, profile_name in (
        (COPY_PROFILE_PATH, COPY_PROFILE_SHA256, COPY_PROFILE),
        (BUILD_PROFILE_PATH, BUILD_PROFILE_SHA256, BUILD_PROFILE),
    ):
        if core.sha256_file(profile_path, limit=128 * 1024) != expected_sha:
            raise core.GateError(f"reviewed profile digest differs: {profile_path}")
        text = profile_path.read_text(encoding="utf-8")
        if f"profile {profile_name} flags=(attach_disconnected,mediate_deleted)" not in text:
            raise core.GateError(f"reviewed profile identity differs: {profile_path}")
        if "REVIEWED_SINGLE_ATTEMPT_ONLY" not in text:
            raise core.GateError(f"reviewed profile header differs: {profile_path}")
        effective = _effective_profile(text)
        for token in core.FORBIDDEN_PROFILE_TOKENS:
            if token in effective:
                raise core.GateError(f"reviewed profile violates a forbidden boundary: {token}")
        if [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")] != ["/newroot/lib wl,", "/newroot/lib64 wl,"]:
            raise core.GateError("reviewed profile synthetic-link creation surface differs")
        if "/newroot/lib/**" in effective or "/newroot/lib rw" in effective or "/oldroot/lib" in effective:
            raise core.GateError("reviewed profile adds an impermissible host /lib surface")
        proc_sys_writes = [line.strip() for line in effective.splitlines() if "/proc/sys" in line and " w" in line]
        if proc_sys_writes != ["/proc/sys/user/max_user_namespaces w,"]:
            raise core.GateError("reviewed profile proc-sys write surface differs")
        capabilities = [line.strip() for line in effective.splitlines() if line.strip().startswith("capability ")]
        if capabilities != ["capability sys_admin,", "capability sys_ptrace,", "capability sys_resource,", "capability setpcap,", "capability net_admin,"]:
            raise core.GateError("reviewed profile capability surface differs")
        effective_profiles[profile_path] = effective
    required_headers = {"/usr/include/ r,", "/usr/include/** r,"}
    build_rules = {line.strip() for line in effective_profiles[BUILD_PROFILE_PATH].splitlines() if line.strip()}
    if not required_headers <= build_rules or any(line.startswith("/usr/include") and line not in required_headers for line in build_rules):
        raise core.GateError("build profile header surface differs")
    if "/usr/include" in effective_profiles[COPY_PROFILE_PATH]:
        raise core.GateError("source-copy profile must not gain compiler header access")
    for phase in ("source-copy", "build"):
        argv = bwrap_prefix_v12(phase=phase)
        ro_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--ro-bind"]
        expected_ro_pairs = [["/usr", "/usr"]]
        if phase == "source-copy":
            expected_ro_pairs.append([str(core.SOURCE_ROOT), "/work/input"])
        if ro_pairs != expected_ro_pairs:
            raise core.GateError("bwrap read-only bind boundary differs")
        symlink_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--symlink"]
        if symlink_pairs != [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]]:
            raise core.GateError("bwrap synthetic-link boundary differs")
        bind_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item in {"--ro-bind", "--bind"}]
        if any(pair[0] in {"/lib", "/lib64"} or pair[1] in {"/lib", "/lib64"} for pair in bind_pairs):
            raise core.GateError("bwrap must not bind a host lib path")
        if "--proc" in argv or "--dev" in argv or "--share-net" in argv:
            raise core.GateError("bwrap guest device or network boundary differs")
    if core.OUTPUT_SOURCE != OUTPUT_SOURCE or core.ARTIFACT_PATH != ARTIFACT_PATH:
        raise core.GateError("derived output paths were not rebound to the fresh v12 root")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": core.sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": core.canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": core.sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "frozen_v11_gate": {"path": str(PREVIOUS_GATE_PATH), "sha256": PREVIOUS_GATE_SHA256},
        "frozen_v11_policy": {"path": str(previous.POLICY_PATH), "sha256": PREVIOUS_POLICY_SHA256},
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
    }


# All state-changing functions remain the unchanged v10 state machine.  These
# are the only version-specific values it resolves through module globals.
core.__file__ = str(SCRIPT_PATH)
core.BUILD_ID = BUILD_ID
core.LIQUID_ROOT = LIQUID_ROOT
core.ATTEMPT_ROOT = ATTEMPT_ROOT
core.OUTPUT_ROOT = OUTPUT_ROOT
core.OUTPUT_SOURCE = OUTPUT_SOURCE
core.ARTIFACT_PATH = ARTIFACT_PATH
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
core.verify_review_artifacts = verify_review_artifacts_v12
core.bwrap_prefix = bwrap_prefix_v12

for _name in dir(core):
    if not _name.startswith("_") and _name not in globals():
        globals()[_name] = getattr(core, _name)


if __name__ == "__main__":
    raise SystemExit(core.main())
