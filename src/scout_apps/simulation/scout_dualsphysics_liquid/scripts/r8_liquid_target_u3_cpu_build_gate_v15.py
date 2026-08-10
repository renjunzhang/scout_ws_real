#!/usr/bin/env python3
"""One-shot U3 v15 source-copy and CPU-build gate for this MSI host.

v14 compiled and linked successfully, but its outer-output inventory applied the
single-link regular-file rule to normal directories.  v15 byte-pins v14 and
changes only that classification: directories remain non-symlink directories,
while every file remains a regular non-symlink single-link file.
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import stat
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
PREVIOUS_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_cpu_build_gate_v14.py")
PREVIOUS_GATE_SHA256 = "6fdd25bc32465d6db1dd85ca49cbeb7745f86ce7b508575574bb3e8dbed5a94b"
PREVIOUS_POLICY_SHA256 = "58a0d79d19b68532ab4e1cddb821534528e8693f815cc15df3eee1384b8dc88b"

BUILD_ID = "u3_source_cpu_build_20260807T023724Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v15.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v15.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v15.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v15.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v15"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v15"
COPY_PROFILE_SHA256 = "3d6ef41054dda30a64b16ee4586c3b0cf21e7e66a756ae873dbab11a0f43b116"
BUILD_PROFILE_SHA256 = "55e0d63e17acb6a30e19f13e268929c6bb8781448cd57e253a918347c4d3d8ef"
EXPECTED_SYMLINKS = [
    {"link": "/lib", "target": "usr/lib"},
    {"link": "/lib64", "target": "usr/lib64"},
]
OUTPUT_DIRECTORY_CONTRACT = {
    "directory_entries": "must_be_real_non_symlink_directories; POSIX_directory_link_count_not_constrained",
    "file_entries": "must_be_regular_single_link_non_symlink_files",
    "symlinks": "forbidden",
}

sys.dont_write_bytecode = True


def sha256_regular_file(path: Path, *, limit: int = 4 * 1024 * 1024) -> str:
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
        raise RuntimeError("the byte-pinned v14 review layer differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_cpu_build_v14_frozen_layer", PREVIOUS_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the byte-pinned v14 review layer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.sha256_regular_file(module.POLICY_PATH, limit=256 * 1024) != PREVIOUS_POLICY_SHA256:
        raise RuntimeError("the byte-pinned v14 policy differs")
    return module


previous = load_frozen_previous_gate()
core = previous.core
OBJECT_CONTRACT = previous.OBJECT_CONTRACT
OUTPUT_SOURCE = OUTPUT_ROOT / core.OUTPUT_SOURCE_RELATIVE
ARTIFACT_PATH = OUTPUT_ROOT / core.ARTIFACT_RELATIVE


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def assert_safe_output_tree_entries_v15(root: Path) -> None:
    """Reject linked/special files but admit ordinary directory link counts."""

    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if not stat.S_ISDIR(directory_info.st_mode) or stat.S_ISLNK(directory_info.st_mode):
            raise core.GateError(f"unsafe build output directory: {directory_path}")
        for name in subdirs:
            candidate = directory_path / name
            candidate_info = os.lstat(candidate)
            if not stat.S_ISDIR(candidate_info.st_mode) or stat.S_ISLNK(candidate_info.st_mode):
                raise core.GateError(f"unsafe build output directory entry: {candidate}")
        for name in files:
            candidate = directory_path / name
            candidate_info = os.lstat(candidate)
            if (
                not stat.S_ISREG(candidate_info.st_mode)
                or stat.S_ISLNK(candidate_info.st_mode)
                or candidate_info.st_nlink != 1
            ):
                raise core.GateError(f"unsafe build output file entry: {candidate}")


def inventory_build_output_v15(expected: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    core.assert_directory(OUTPUT_ROOT, mode=0o700)
    object_names = previous.parse_cpu_object_manifest_v14(OUTPUT_SOURCE, expected)
    built_source = previous.inventory_built_source_v14(OUTPUT_SOURCE, expected, object_names)
    assert_safe_output_tree_entries_v15(OUTPUT_ROOT)
    unexpected: list[str] = []
    total = 0
    artifact_seen = False
    for directory, _subdirs, files in os.walk(OUTPUT_ROOT, topdown=True, followlinks=False):
        directory_path = Path(directory)
        for filename in files:
            path = directory_path / filename
            relative = path.relative_to(OUTPUT_ROOT)
            total += os.lstat(path).st_size
            if path == ARTIFACT_PATH:
                artifact_seen = True
            elif relative.parts[:3] == ("buildtree", "src", "source"):
                continue
            else:
                unexpected.append(str(relative))
    if not artifact_seen:
        raise core.GateError("static-audited CPU artifact is absent from output inventory")
    if unexpected:
        raise core.GateError(f"unexpected build output files: {unexpected}")
    if total > 2 * 1024 * 1024 * 1024:
        raise core.GateError("build output exceeds the frozen two-GiB ceiling")
    return {
        "cloned_source": {
            key: built_source[key]
            for key in ("root", "file_count", "total_bytes", "manifest_sha256", "allowed_extra_files")
        },
        "object_contract": OBJECT_CONTRACT,
        "output_directory_contract": OUTPUT_DIRECTORY_CONTRACT,
        "object_count": built_source["object_count"],
        "object_names_canonical_sha256": built_source["object_names_canonical_sha256"],
        "objects": built_source["objects"],
        "total_bytes": total,
    }


def static_elf_audit_v15(path: Path) -> dict[str, Any]:
    """Use v14's byte-pinned PIE audit, including descriptor-level disarm."""

    return previous.static_elf_audit_v14(path)


def bwrap_prefix_v15(*, phase: str) -> list[str]:
    argv = previous.bwrap_prefix_v14(phase=phase)
    replacements = {
        previous.COPY_PROFILE: COPY_PROFILE,
        previous.BUILD_PROFILE: BUILD_PROFILE,
        str(previous.OUTPUT_ROOT): str(OUTPUT_ROOT),
    }
    result = [replacements.get(item, item) for item in argv]
    if any(item in {previous.COPY_PROFILE, previous.BUILD_PROFILE, str(previous.OUTPUT_ROOT)} for item in result):
        raise core.GateError("v15 argv retains a v14 profile or output path")
    return result


def verify_review_artifacts_v15() -> dict[str, Any]:
    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    previous_policy = core.read_json_object(previous.POLICY_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise core.GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v15",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V15",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V15_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected_value in expected_top.items():
        if policy.get(key) != expected_value:
            raise core.GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise core.GateError("policy command surface differs")
    if policy.get("outer_privilege_handoff") != {
        "only_privileged_child_command": ["/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups", "--", "/usr/bin/python3", "r8_liquid_target_u3_cpu_build_gate_v15.py", "<fixed_phase>"],
        "must_drop_before_aa_exec": True,
        "required_uid": 1000,
        "required_gid": 1000,
        "required_supplementary_group_count": 0,
        "root_access_to_source_or_output": "forbidden_after_exec",
    }:
        raise core.GateError("policy privilege handoff differs")
    if policy.get("frozen_attempt") != {
        "build_id": BUILD_ID,
        "attempt_root": str(ATTEMPT_ROOT),
        "output_root": str(OUTPUT_ROOT),
        "source_copy_profile": COPY_PROFILE,
        "source_copy_profile_path": "config/apparmor_drafts/r8-liquid-u3-source-copy-v15.profile",
        "build_profile": BUILD_PROFILE,
        "build_profile_path": "config/apparmor_drafts/r8-liquid-u3-cpu-build-v15.profile",
        "source_copy_profile_sha256": COPY_PROFILE_SHA256,
        "build_profile_sha256": BUILD_PROFILE_SHA256,
    }:
        raise core.GateError("policy frozen attempt identity differs")
    for key in ("source_input", "isolation", "resources", "source_copy_command", "build_command", "generated_wrapper", "profile_lifecycle", "invariants"):
        if policy.get(key) != previous_policy.get(key):
            raise core.GateError(f"v15 widens or differs from frozen v14 policy field: {key}")
    expected_audit = {
        **previous_policy["static_artifact_audit"],
        "post_build_object_contract": OBJECT_CONTRACT,
        "post_build_output_directory_contract": OUTPUT_DIRECTORY_CONTRACT,
    }
    if policy.get("static_artifact_audit") != expected_audit:
        raise core.GateError("v15 static-artifact/output contract differs")
    if policy.get("isolation", {}).get("synthetic_symlinks") != EXPECTED_SYMLINKS:
        raise core.GateError("policy synthetic-link boundary differs")
    if policy.get("generated_wrapper", {}).get("content") != core.WRAPPER_CONTENT:
        raise core.GateError("policy generated wrapper differs")
    expected_source = core._source_entries_from_receipt()
    source_objects = previous.parse_cpu_object_manifest_v14(core.SOURCE_ROOT, expected_source)
    if source_objects != tuple(sorted(source_objects)):
        raise core.GateError("sealed CPU object manifest is not canonically ordered")
    for profile_path, expected_sha, profile_name, previous_path, previous_name in (
        (COPY_PROFILE_PATH, COPY_PROFILE_SHA256, COPY_PROFILE, previous.COPY_PROFILE_PATH, previous.COPY_PROFILE),
        (BUILD_PROFILE_PATH, BUILD_PROFILE_SHA256, BUILD_PROFILE, previous.BUILD_PROFILE_PATH, previous.BUILD_PROFILE),
    ):
        if core.sha256_file(profile_path, limit=128 * 1024) != expected_sha:
            raise core.GateError(f"reviewed profile digest differs: {profile_path}")
        text = profile_path.read_text(encoding="utf-8")
        if f"profile {profile_name} flags=(attach_disconnected,mediate_deleted)" not in text or "REVIEWED_SINGLE_ATTEMPT_ONLY" not in text:
            raise core.GateError(f"reviewed profile identity/header differs: {profile_path}")
        effective = _effective_profile(text)
        for token in core.FORBIDDEN_PROFILE_TOKENS:
            if token in effective:
                raise core.GateError(f"reviewed profile violates a forbidden boundary: {token}")
        normalized = (
            effective.replace(str(ATTEMPT_ROOT), str(previous.ATTEMPT_ROOT))
            .replace(str(OUTPUT_ROOT), str(previous.OUTPUT_ROOT))
            .replace(profile_name, previous_name)
        )
        if normalized != previous._effective_profile(previous_path.read_text(encoding="utf-8")):
            raise core.GateError("v15 profile differs from v14 beyond reviewed identity/output substitutions")
        if [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")] != ["/newroot/lib wl,", "/newroot/lib64 wl,"]:
            raise core.GateError("reviewed profile synthetic-link creation surface differs")
    for phase in ("source-copy", "build"):
        argv = bwrap_prefix_v15(phase=phase)
        ro_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--ro-bind"]
        expected_ro = [["/usr", "/usr"]] + ([[str(core.SOURCE_ROOT), "/work/input"]] if phase == "source-copy" else [])
        if ro_pairs != expected_ro:
            raise core.GateError("bwrap read-only bind boundary differs")
        if [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--symlink"] != [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]]:
            raise core.GateError("bwrap synthetic-link boundary differs")
        if [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--bind"] != [[str(OUTPUT_ROOT), "/work/output"]]:
            raise core.GateError("bwrap must retain exactly one writable output bind")
        bind_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item in {"--ro-bind", "--bind"}]
        if any(pair[0] in {"/lib", "/lib64"} or pair[1] in {"/lib", "/lib64"} for pair in bind_pairs):
            raise core.GateError("bwrap must not bind a host lib path")
        if [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--tmpfs"] != ["/work"]:
            raise core.GateError("bwrap guest tmpfs boundary differs")
        if "--proc" in argv or "--dev" in argv or "--share-net" in argv:
            raise core.GateError("bwrap guest device or network boundary differs")
    if core.OUTPUT_SOURCE != OUTPUT_SOURCE or core.ARTIFACT_PATH != ARTIFACT_PATH:
        raise core.GateError("derived output paths were not rebound to fresh v15 output")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": core.sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": core.canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": core.sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "frozen_v14_gate": {"path": str(PREVIOUS_GATE_PATH), "sha256": PREVIOUS_GATE_SHA256},
        "frozen_v14_policy": {"path": str(previous.POLICY_PATH), "sha256": PREVIOUS_POLICY_SHA256},
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
        "object_contract": OBJECT_CONTRACT,
        "output_directory_contract": OUTPUT_DIRECTORY_CONTRACT,
    }


def execute_build_v15() -> int:
    """Run the v14 state transition with only v15 output-tree classification."""

    phase = "build"
    try:
        preflight_data = core.preflight(phase)
    except core.GateError as exc:
        print(core.json.dumps({"status": "NO_GO_PREFLIGHT", "phase": phase, "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    partial = BUILD_RECEIPT.with_suffix(BUILD_RECEIPT.suffix + ".partial")
    core.write_json_new(partial, {"status": "STARTED_U3_CPU_BUILD", "phase": phase, "build_id": BUILD_ID, "started_at_utc": core.utc_now(), "preflight": preflight_data})
    result = core.run_fixed_argv(core.bwrap_prefix(phase=phase))
    final: dict[str, Any] = {
        "document_type": "SMPCC_R8_LIQUID_U3_CPU_BUILD_RECEIPT",
        "build_id": BUILD_ID,
        "created_at_utc": core.utc_now(),
        "phase": phase,
        "preflight": preflight_data,
        "execution": result,
        "partial_start_record": str(partial),
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "compiled_artifact_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
    }
    try:
        final["host_initial_namespace_user_max_user_namespaces"] = core.host_userns_postcondition(preflight_data)
    except core.GateError as exc:
        final.update({"status": "CPU_BUILD_HOST_SYSCTL_POSTCONDITION_NO_GO", "postflight_error": str(exc), "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        core.write_json_new(BUILD_RECEIPT, final)
        print(core.json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 5
    if final["host_initial_namespace_user_max_user_namespaces"]["unchanged"] is not True:
        final.update({"status": "CPU_BUILD_HOST_SYSCTL_POSTCONDITION_NO_GO", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        core.write_json_new(BUILD_RECEIPT, final)
        print(core.json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 5
    if result["returncode"] != 0:
        final.update({"status": "CPU_BUILD_FAILED_NO_RETRY", "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        core.write_json_new(BUILD_RECEIPT, final)
        print(core.json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 3
    try:
        expected = core._source_entries_from_receipt()
        if not ARTIFACT_PATH.exists() or ARTIFACT_PATH.is_symlink():
            raise core.GateError("candidate CPU artifact is absent")
        audit = static_elf_audit_v15(ARTIFACT_PATH)
        output = inventory_build_output_v15(expected)
        final.update({
            "status": "PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT",
            "artifact": {**audit, "path": str(ARTIFACT_PATH), "mode_after_audit": audit["mode_after_disarm"]},
            "output_inventory": output,
            "next_allowed_stage": "SEPARATE_GENCASE_AND_SOLVER_EXECUTION_ADMISSION_REQUIRED",
        })
    except (core.GateError, OSError, UnicodeError, ValueError) as exc:
        failure_artifact: dict[str, Any] = {}
        try:
            info = os.lstat(ARTIFACT_PATH)
            if stat.S_ISREG(info.st_mode) and not stat.S_ISLNK(info.st_mode) and info.st_nlink == 1:
                mode = stat.S_IMODE(info.st_mode)
                failure_artifact = {"path": str(ARTIFACT_PATH), "mode_at_failure": oct(mode), "execute_disabled_at_failure": not bool(mode & 0o111)}
        except OSError:
            pass
        final.update({"status": "CPU_BUILD_POSTFLIGHT_NO_GO", "postflight_error": str(exc), "artifact_at_failure": failure_artifact, "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED"})
        core.write_json_new(BUILD_RECEIPT, final)
        print(core.json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
        return 4
    core.write_json_new(BUILD_RECEIPT, final)
    print(core.json.dumps({"status": final["status"], "receipt": str(BUILD_RECEIPT)}, ensure_ascii=False))
    return 0


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
core.static_elf_audit = static_elf_audit_v15
core.inventory_build_output = inventory_build_output_v15
core.verify_review_artifacts = verify_review_artifacts_v15
core.bwrap_prefix = bwrap_prefix_v15
core.execute_build = execute_build_v15

for _name in dir(core):
    if not _name.startswith("_") and _name not in globals():
        globals()[_name] = getattr(core, _name)


if __name__ == "__main__":
    raise SystemExit(core.main())
