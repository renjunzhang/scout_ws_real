#!/usr/bin/env python3
"""One-shot U3 v14 source-copy and CPU-build gate for this MSI host.

v13 compiled and linked successfully, but its post-build inventory called the
source-only inventory before it could classify the direct Makefile_cpu target's
ordinary object files.  v14 byte-pins v13 and changes only that post-build
classification through a sealed-Makefile-derived exact object manifest.
"""

from __future__ import annotations

import hashlib
import importlib.util
import os
import stat
import struct
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
PREVIOUS_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_cpu_build_gate_v13.py")
PREVIOUS_GATE_SHA256 = "2ac5e17ec1c8eba136934aefbc35b799713ff88190172bf27ecac8531427f51c"
PREVIOUS_POLICY_SHA256 = "184f14f94cbc8e3533628600513bae4ef0a44b7904e4dcbe27f8e5e57a50db51"

BUILD_ID = "u3_source_cpu_build_20260807T020703Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v14.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v14.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v14.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v14.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v14"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v14"
COPY_PROFILE_SHA256 = "ab81fd901c0153c1aab8cbfca974671f2630485386af988d3d9ce48687aafeb3"
BUILD_PROFILE_SHA256 = "086e54f0291b9b5b5154c32769eaa3a7005af4ddfa545c4af45b8a55f1a18fa2"
EXPECTED_SYMLINKS = [
    {"link": "/lib", "target": "usr/lib"},
    {"link": "/lib64", "target": "usr/lib64"},
]
MAKEFILE_CPU_SHA256 = "4038165b761e233b207b7196f320dc59b3092737d5eaf8c98c9a282b9e900a91"
OBJECT_COUNT = 109
OBJECT_NAMES_CANONICAL_SHA256 = "7e680df8de6af025b3811f52a725db896514d5f5b6fc924a3988aef60b534b2f"
OBJECT_SECTION_HEADER_COUNT_MAX = 16_384
OBJECT_VARIABLES = (
    "OBJXML",
    "OBJSPHMOTION",
    "OBCOMMON",
    "OBCOMMONDSPH",
    "OBSPH",
    "OBSPHSINGLE",
    "OBWAVERZ",
    "OBCHRONO",
    "OBMOORDYNPLUS",
    "OBINOUT",
    "OBMESH",
    "OBVRES",
    "OBFLEXSTRUC",
)
OBJECT_CONTRACT = {
    "makefile_cpu_sha256": MAKEFILE_CPU_SHA256,
    "object_count": OBJECT_COUNT,
    "object_names_canonical_sha256": OBJECT_NAMES_CANONICAL_SHA256,
    "object_location": "only_directly_under_buildtree_src_source",
    "all_manifest_objects_required": True,
    "object_elf": "ELF64_LITTLE_ENDIAN_X86_64_ET_REL_WITHOUT_PROGRAM_HEADERS",
    "section_header_count": {"minimum": 1, "maximum": OBJECT_SECTION_HEADER_COUNT_MAX},
    "object_execute_bits": "forbidden",
    "unexpected_output_files": "forbidden",
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
        raise RuntimeError("the byte-pinned v13 review layer differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_cpu_build_v13_frozen_layer", PREVIOUS_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the byte-pinned v13 review layer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.sha256_regular_file(module.POLICY_PATH, limit=256 * 1024) != PREVIOUS_POLICY_SHA256:
        raise RuntimeError("the byte-pinned v13 policy differs")
    return module


previous = load_frozen_previous_gate()
core = previous.core
OUTPUT_SOURCE = OUTPUT_ROOT / core.OUTPUT_SOURCE_RELATIVE
ARTIFACT_PATH = OUTPUT_ROOT / core.ARTIFACT_RELATIVE


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def parse_cpu_object_manifest_v14(root: Path, expected: Mapping[str, Mapping[str, Any]]) -> tuple[str, ...]:
    """Derive the exact direct CPU target object names from sealed Makefile_cpu."""

    makefile = root / "Makefile_cpu"
    makefile_fact = expected.get("Makefile_cpu")
    if not isinstance(makefile_fact, Mapping) or makefile_fact.get("sha256") != MAKEFILE_CPU_SHA256:
        raise core.GateError("sealed Makefile_cpu manifest differs from the reviewed object contract")
    if core.sha256_file(makefile) != MAKEFILE_CPU_SHA256:
        raise core.GateError("observed Makefile_cpu differs from the reviewed object contract")
    values: dict[str, list[str]] = {}
    for raw_line in makefile.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        for variable in OBJECT_VARIABLES:
            plain_prefix = f"{variable}="
            append_prefix = f"{variable}:="
            if line.startswith(plain_prefix):
                if variable in values:
                    raise core.GateError("sealed Makefile_cpu repeats an object variable")
                tokens = line[len(plain_prefix) :].split()
                if not tokens:
                    raise core.GateError("sealed Makefile_cpu has an empty initial object variable")
                values[variable] = tokens
                break
            if line.startswith(append_prefix):
                if variable not in values:
                    raise core.GateError("sealed Makefile_cpu appends an undefined object variable")
                suffix = line[len(append_prefix) :]
                retained = f"$({variable})"
                if not suffix.startswith(retained):
                    raise core.GateError("sealed Makefile_cpu object append differs from the reviewed grammar")
                tokens = suffix[len(retained) :].split()
                if not tokens:
                    raise core.GateError("sealed Makefile_cpu has an empty object append")
                values[variable].extend(tokens)
                break
    if set(values) != set(OBJECT_VARIABLES):
        raise core.GateError("sealed Makefile_cpu object-variable set differs")
    listed = [name for variable in OBJECT_VARIABLES for name in values[variable]]
    if len(listed) != len(set(listed)):
        raise core.GateError("sealed Makefile_cpu has duplicate object names")
    for name in listed:
        if (
            not name.endswith(".o")
            or "/" in name
            or "\\" in name
            or name.startswith(".")
            or not all(character.isalnum() or character in "_." for character in name)
            or f"{name[:-2]}.cpp" not in expected
        ):
            raise core.GateError("sealed Makefile_cpu object name leaves the direct sealed C++ surface")
    names = tuple(sorted(listed))
    if len(names) != OBJECT_COUNT or core.canonical_hash(list(names)) != OBJECT_NAMES_CANONICAL_SHA256:
        raise core.GateError("sealed Makefile_cpu object manifest count or digest differs")
    return names


def audit_rel_object_v14(path: Path) -> dict[str, Any]:
    """Statically validate one retained compiler object without executing it."""

    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_nlink != 1
            or info.st_mode & 0o111
            or info.st_size < 64
            or info.st_size > 512 * 1024 * 1024
        ):
            raise core.GateError("compiler object metadata is unsafe")
        header = core.read_exact(path, 0, 64, descriptor)
        if header[:4] != b"\x7fELF" or header[4] != 2 or header[5] != 1 or header[6] != 1:
            raise core.GateError("compiler object is not an ELF64 little-endian file")
        values = struct.unpack("<16sHHIQQQIHHHHHH", header)
        e_type, e_machine, e_phoff, e_shoff, e_phentsize, e_phnum, e_shentsize, e_shnum = (
            values[1], values[2], values[5], values[6], values[9], values[10], values[11], values[12]
        )
        if (
            e_type != 1
            or e_machine != 62
            or e_phoff != 0
            or e_phentsize != 0
            or e_phnum != 0
            or e_shoff <= 0
            or e_shentsize != 64
            or not 1 <= e_shnum <= OBJECT_SECTION_HEADER_COUNT_MAX
            or e_shoff + e_shentsize * e_shnum > info.st_size
        ):
            raise core.GateError("compiler object is not the required x86-64 ET_REL shape")
        return {
            "path": str(path),
            "sha256": core.sha256_file(path, limit=512 * 1024 * 1024),
            "size_bytes": info.st_size,
            "elf_type": "ET_REL",
            "section_header_count": e_shnum,
            "execute_bits": "absent",
        }
    finally:
        os.close(descriptor)


def inventory_built_source_v14(
    root: Path,
    expected: Mapping[str, Mapping[str, Any]],
    object_names: tuple[str, ...],
) -> dict[str, Any]:
    """Validate sealed sources, one exact wrapper, and one exact object set."""

    core.assert_no_symlink_components(root, require_exists=True)
    if not root.is_dir() or root.is_symlink():
        raise core.GateError("built source root is unsafe")
    observed: dict[str, dict[str, Any]] = {}
    objects: dict[str, dict[str, Any]] = {}
    wrappers: list[str] = []
    expected_objects = set(object_names)
    for directory, subdirs, files in os.walk(root, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if not stat.S_ISDIR(directory_info.st_mode) or stat.S_ISLNK(directory_info.st_mode):
            raise core.GateError(f"built source directory is unsafe: {directory_path}")
        for name in [*subdirs, *files]:
            candidate = directory_path / name
            if stat.S_ISLNK(os.lstat(candidate).st_mode):
                raise core.GateError(f"built source symlink is forbidden: {candidate}")
        for filename in files:
            path = directory_path / filename
            info = os.lstat(path)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise core.GateError(f"built source file is unsafe: {path}")
            relative = str(path.relative_to(root))
            if relative in expected:
                if info.st_size != expected[relative]["size_bytes"]:
                    raise core.GateError(f"built source byte count differs: {relative}")
                digest = core.sha256_file(path)
                if digest != expected[relative]["sha256"]:
                    raise core.GateError(f"built source digest differs: {relative}")
                observed[relative] = {"sha256": digest, "size_bytes": info.st_size}
                continue
            if relative == core.WRAPPER_NAME:
                if stat.S_IMODE(info.st_mode) != 0o600 or path.read_text(encoding="utf-8") != core.WRAPPER_CONTENT:
                    raise core.GateError("generated Make wrapper differs")
                wrappers.append(relative)
                continue
            relative_path = Path(relative)
            if len(relative_path.parts) == 1 and relative in expected_objects and f"{relative[:-2]}.cpp" in expected:
                objects[relative] = audit_rel_object_v14(path)
                continue
            raise core.GateError(f"unexpected built source file: {relative}")
    if set(observed) != set(expected):
        raise core.GateError("built source file set differs from source receipt")
    if wrappers != [core.WRAPPER_NAME]:
        raise core.GateError("built source wrapper set differs")
    if set(objects) != expected_objects:
        missing = sorted(expected_objects - set(objects))
        extra = sorted(set(objects) - expected_objects)
        raise core.GateError(f"built source object manifest differs: missing={missing} extra={extra}")
    return {
        "root": str(root),
        "file_count": len(observed),
        "total_bytes": sum(item["size_bytes"] for item in observed.values()),
        "manifest_sha256": core.canonical_hash(observed),
        "allowed_extra_files": wrappers,
        "object_count": len(objects),
        "object_names_canonical_sha256": core.canonical_hash(sorted(objects)),
        "objects": [objects[name] for name in sorted(objects)],
    }


def inventory_build_output_v14(expected: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    core.assert_directory(OUTPUT_ROOT, mode=0o700)
    object_names = parse_cpu_object_manifest_v14(OUTPUT_SOURCE, expected)
    built_source = inventory_built_source_v14(OUTPUT_SOURCE, expected, object_names)
    unexpected: list[str] = []
    total = 0
    artifact_seen = False
    for directory, subdirs, files in os.walk(OUTPUT_ROOT, topdown=True, followlinks=False):
        directory_path = Path(directory)
        directory_info = os.lstat(directory_path)
        if not stat.S_ISDIR(directory_info.st_mode) or stat.S_ISLNK(directory_info.st_mode):
            raise core.GateError(f"unsafe build output directory: {directory_path}")
        for name in [*subdirs, *files]:
            candidate = directory_path / name
            candidate_info = os.lstat(candidate)
            if stat.S_ISLNK(candidate_info.st_mode) or candidate_info.st_nlink != 1:
                raise core.GateError(f"unsafe build output entry: {candidate}")
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
        "object_count": built_source["object_count"],
        "object_names_canonical_sha256": built_source["object_names_canonical_sha256"],
        "objects": built_source["objects"],
        "total_bytes": total,
    }


def static_elf_audit_v14(path: Path) -> dict[str, Any]:
    """Use v13's byte-pinned PIE audit, including descriptor-level disarm."""

    return previous.static_elf_audit_v13(path)


def bwrap_prefix_v14(*, phase: str) -> list[str]:
    argv = previous.bwrap_prefix_v13(phase=phase)
    replacements = {
        previous.COPY_PROFILE: COPY_PROFILE,
        previous.BUILD_PROFILE: BUILD_PROFILE,
        str(previous.OUTPUT_ROOT): str(OUTPUT_ROOT),
    }
    result = [replacements.get(item, item) for item in argv]
    if any(item in {previous.COPY_PROFILE, previous.BUILD_PROFILE, str(previous.OUTPUT_ROOT)} for item in result):
        raise core.GateError("v14 argv retains a v13 profile or output path")
    return result


def verify_review_artifacts_v14() -> dict[str, Any]:
    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    previous_policy = core.read_json_object(previous.POLICY_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise core.GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v14",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V14",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V14_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected_value in expected_top.items():
        if policy.get(key) != expected_value:
            raise core.GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise core.GateError("policy command surface differs")
    if policy.get("outer_privilege_handoff") != {
        "only_privileged_child_command": ["/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups", "--", "/usr/bin/python3", "r8_liquid_target_u3_cpu_build_gate_v14.py", "<fixed_phase>"],
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
        "source_copy_profile_path": "config/apparmor_drafts/r8-liquid-u3-source-copy-v14.profile",
        "build_profile": BUILD_PROFILE,
        "build_profile_path": "config/apparmor_drafts/r8-liquid-u3-cpu-build-v14.profile",
        "source_copy_profile_sha256": COPY_PROFILE_SHA256,
        "build_profile_sha256": BUILD_PROFILE_SHA256,
    }:
        raise core.GateError("policy frozen attempt identity differs")
    for key in ("source_input", "isolation", "resources", "source_copy_command", "build_command", "generated_wrapper", "profile_lifecycle", "invariants"):
        if policy.get(key) != previous_policy.get(key):
            raise core.GateError(f"v14 widens or differs from frozen v13 policy field: {key}")
    expected_audit = {**previous_policy["static_artifact_audit"], "post_build_object_contract": OBJECT_CONTRACT}
    if policy.get("static_artifact_audit") != expected_audit:
        raise core.GateError("v14 static-artifact/object contract differs")
    if policy.get("isolation", {}).get("synthetic_symlinks") != EXPECTED_SYMLINKS:
        raise core.GateError("policy synthetic-link boundary differs")
    if policy.get("generated_wrapper", {}).get("content") != core.WRAPPER_CONTENT:
        raise core.GateError("policy generated wrapper differs")
    expected_source = core._source_entries_from_receipt()
    source_objects = parse_cpu_object_manifest_v14(core.SOURCE_ROOT, expected_source)
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
            raise core.GateError("v14 profile differs from v13 beyond reviewed identity/output substitutions")
        if [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")] != ["/newroot/lib wl,", "/newroot/lib64 wl,"]:
            raise core.GateError("reviewed profile synthetic-link creation surface differs")
    for phase in ("source-copy", "build"):
        argv = bwrap_prefix_v14(phase=phase)
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
        raise core.GateError("derived output paths were not rebound to fresh v14 output")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": core.sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": core.canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": core.sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "frozen_v13_gate": {"path": str(PREVIOUS_GATE_PATH), "sha256": PREVIOUS_GATE_SHA256},
        "frozen_v13_policy": {"path": str(previous.POLICY_PATH), "sha256": PREVIOUS_POLICY_SHA256},
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
        "object_contract": OBJECT_CONTRACT,
    }


def execute_build_v14() -> int:
    """Run the v13 state transition with only v14 post-build inventory changed."""

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
        audit = static_elf_audit_v14(ARTIFACT_PATH)
        output = inventory_build_output_v14(expected)
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
core.static_elf_audit = static_elf_audit_v14
core.inventory_build_output = inventory_build_output_v14
core.verify_review_artifacts = verify_review_artifacts_v14
core.bwrap_prefix = bwrap_prefix_v14
core.execute_build = execute_build_v14

for _name in dir(core):
    if not _name.startswith("_") and _name not in globals():
        globals()[_name] = getattr(core, _name)


if __name__ == "__main__":
    raise SystemExit(core.main())
