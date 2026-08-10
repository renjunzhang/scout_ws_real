#!/usr/bin/env python3
"""One-shot U3 v13 source-copy and CPU-build gate for this MSI host.

v13 preserves v12's byte-pinned isolation/state-machine layer.  Its only
functional change is a stricter-aware static ELF audit: accept one native
ET_DYN executable only when DT_FLAGS_1 contains DF_1_PIE, rather than disabling
Ubuntu's default PIE hardening to force the legacy ET_EXEC shape.
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
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
PREVIOUS_GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_cpu_build_gate_v12.py")
PREVIOUS_GATE_SHA256 = "15281579053585e76c2cb55adf0d9a093383320b3bb9083efc3ef2c972766715"
PREVIOUS_POLICY_SHA256 = "5beb99c8224720684fb9e88d13b163bab64bd9cedcb282f6a656db8d4b74b475"

BUILD_ID = "u3_source_cpu_build_20260807T013023Z"
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")
ATTEMPT_ROOT = LIQUID_ROOT / "build" / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
COPY_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_source_copy.json"
BUILD_RECEIPT = LIQUID_ROOT / "audits" / f"{BUILD_ID}_cpu_build.json"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v13.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v13.json"
COPY_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-source-copy-v13.profile"
BUILD_PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-cpu-build-v13.profile"
COPY_PROFILE = "r8-liquid-u3-source-copy-v13"
BUILD_PROFILE = "r8-liquid-u3-cpu-build-v13"
COPY_PROFILE_SHA256 = "9c1f93cbefe1327c857eddf3fed309a069f248522dd3b19a981c0679cb4dafbd"
BUILD_PROFILE_SHA256 = "2c4a06b4d17c7c5386410866c27844d89b8d8d9a415c6a8977b10f8799ca0027"
EXPECTED_SYMLINKS = [
    {"link": "/lib", "target": "usr/lib"},
    {"link": "/lib64", "target": "usr/lib64"},
]
ET_DYN = 3
PT_LOAD = 1
PT_GNU_STACK = 0x6474E551
PF_X = 1
DT_FLAGS_1 = 0x6FFFFFFB
DF_1_PIE = 0x08000000

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
        raise RuntimeError("the byte-pinned v12 review layer differs")
    spec = importlib.util.spec_from_file_location("r8_liquid_u3_cpu_build_v12_frozen_layer", PREVIOUS_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the byte-pinned v12 review layer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.sha256_regular_file(module.POLICY_PATH, limit=256 * 1024) != PREVIOUS_POLICY_SHA256:
        raise RuntimeError("the byte-pinned v12 policy differs")
    return module


previous = load_frozen_previous_gate()
core = previous.core
OUTPUT_SOURCE = OUTPUT_ROOT / core.OUTPUT_SOURCE_RELATIVE
ARTIFACT_PATH = OUTPUT_ROOT / core.ARTIFACT_RELATIVE


def _effective_profile(text: str) -> str:
    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def assert_et_dyn_pie(e_type: int, flags_1_values: list[int]) -> int:
    """Accept exactly one ET_DYN image only when it declares DF_1_PIE."""

    if e_type != ET_DYN:
        raise core.GateError("candidate artifact is not an ET_DYN PIE executable")
    if len(flags_1_values) != 1:
        raise core.GateError("candidate artifact has an ambiguous DT_FLAGS_1 surface")
    flags_1 = flags_1_values[0]
    if not flags_1 & DF_1_PIE:
        raise core.GateError("candidate ET_DYN artifact lacks the required DF_1_PIE flag")
    return flags_1


def assert_elf_load_shape(
    entry_point: int,
    program_headers: list[tuple[int, int, int, int, int, int]],
) -> None:
    """Require a concrete executable entry and an explicitly non-exec stack.

    An ET_DYN image alone is not sufficient evidence of an executable PIE.
    Keep the shape check independent of file I/O so it has direct mock-only
    negative coverage and can fail before any dynamic-string interpretation.
    """

    if entry_point <= 0:
        raise core.GateError("candidate artifact has a null entry point")
    executable_entry = False
    stacks: list[tuple[int, int, int, int, int, int]] = []
    for p_type, p_flags, _p_offset, p_vaddr, p_filesz, p_memsz in program_headers:
        if p_type == PT_LOAD:
            if p_memsz < p_filesz:
                raise core.GateError("candidate artifact PT_LOAD memory size leaves file image")
            if p_flags & PF_X and p_vaddr <= entry_point < p_vaddr + p_filesz:
                executable_entry = True
        elif p_type == PT_GNU_STACK:
            stacks.append((p_type, p_flags, _p_offset, p_vaddr, p_filesz, p_memsz))
    if not executable_entry:
        raise core.GateError("candidate artifact entry point is outside executable PT_LOAD data")
    if len(stacks) != 1:
        raise core.GateError("candidate artifact must declare exactly one PT_GNU_STACK")
    if stacks[0][1] & PF_X:
        raise core.GateError("candidate artifact requests an executable GNU stack")


def static_elf_audit_v13(path: Path) -> dict[str, Any]:
    """Disarm, then read and validate a PIE ELF without loading it."""

    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise core.GateError("candidate artifact metadata is unsafe")
        # This is intentionally before any parser work.  The descriptor was
        # opened O_NOFOLLOW and checked as one regular inode, so fchmod cannot
        # race onto a different path target.  A later static rejection still
        # leaves this fresh v13 candidate non-executable at rest.
        os.fchmod(descriptor, 0o400)
        disarmed_mode = stat.S_IMODE(os.fstat(descriptor).st_mode)
        if disarmed_mode != 0o400:
            raise core.GateError("candidate artifact could not be disabled before static audit")
        if info.st_size < 64 or info.st_size > 512 * 1024 * 1024:
            raise core.GateError("candidate artifact size is unsafe")
        header = core.read_exact(path, 0, 64, descriptor)
        if header[:4] != b"\x7fELF" or header[4] != 2 or header[5] != 1 or header[6] != 1:
            raise core.GateError("candidate artifact is not a current little-endian ELF64")
        values = struct.unpack("<16sHHIQQQIHHHHHH", header)
        e_type, e_machine, e_entry, e_phoff, e_phentsize, e_phnum = values[1], values[2], values[4], values[5], values[9], values[10]
        if e_machine != 62 or e_phentsize != 56 or not 1 <= e_phnum <= 64:
            raise core.GateError("candidate artifact ELF header differs from native x86-64 policy")
        program_headers: list[tuple[int, int, int, int, int, int]] = []
        interpreter: str | None = None
        dynamic_region: tuple[int, int] | None = None
        for index in range(e_phnum):
            entry = core.read_exact(path, e_phoff + index * e_phentsize, e_phentsize, descriptor)
            p_type, p_flags, p_offset, p_vaddr, _paddr, p_filesz, p_memsz, _p_align = struct.unpack("<IIQQQQQQ", entry)
            if p_offset + p_filesz > info.st_size:
                raise core.GateError("candidate artifact program header leaves file bounds")
            program_headers.append((p_type, p_flags, p_offset, p_vaddr, p_filesz, p_memsz))
            if p_type == 3:
                if interpreter is not None or not 1 <= p_filesz <= 4096:
                    raise core.GateError("candidate artifact has an unsafe interpreter header")
                raw = core.read_exact(path, p_offset, p_filesz, descriptor)
                if not raw.endswith(b"\0"):
                    raise core.GateError("candidate artifact interpreter is unterminated")
                interpreter = raw[:-1].decode("utf-8", "strict")
            elif p_type == 2:
                if dynamic_region is not None or p_filesz > 65536 or p_filesz % 16:
                    raise core.GateError("candidate artifact dynamic section is unsafe")
                dynamic_region = (p_offset, p_filesz)
        assert_elf_load_shape(e_entry, program_headers)
        if interpreter != "/lib64/ld-linux-x86-64.so.2" or dynamic_region is None:
            raise core.GateError("candidate artifact interpreter/dynamic/stack policy differs")
        dynamic = core.read_exact(path, dynamic_region[0], dynamic_region[1], descriptor)
        string_address: int | None = None
        string_size: int | None = None
        needed_offsets: list[int] = []
        flags_1_values: list[int] = []
        rpath_seen = False
        runpath_seen = False
        terminated = False
        for offset in range(0, len(dynamic), 16):
            tag, value = struct.unpack("<QQ", dynamic[offset : offset + 16])
            if tag == 0:
                terminated = True
                break
            if tag == 1:
                needed_offsets.append(value)
            elif tag == 5:
                string_address = value
            elif tag == 10:
                string_size = value
            elif tag == 15:
                rpath_seen = True
            elif tag == 29:
                runpath_seen = True
            elif tag == DT_FLAGS_1:
                flags_1_values.append(value)
        if not terminated:
            raise core.GateError("candidate artifact dynamic section lacks a terminator")
        flags_1 = assert_et_dyn_pie(e_type, flags_1_values)
        if string_address is None or string_size is None or not 1 <= string_size <= 1_048_576:
            raise core.GateError("candidate artifact dynamic string table is unsafe")
        string_offset: int | None = None
        for p_type, _flags, p_offset, p_vaddr, p_filesz, _memsz in program_headers:
            if p_type == 1 and p_vaddr <= string_address < p_vaddr + p_filesz:
                string_offset = p_offset + string_address - p_vaddr
                break
        if string_offset is None:
            raise core.GateError("candidate artifact dynamic string table is unmapped")
        strings = core.read_exact(path, string_offset, string_size, descriptor)
        needed = [core.c_string(strings, value) for value in needed_offsets]
        required = {"libgomp.so.1", "libstdc++.so.6", "libgcc_s.so.1", "libc.so.6"}
        forbidden = ("chrono", "dsph", "cuda", "nvidia", "ros", "gazebo", "wavegen", "moordyn")
        if not required.issubset(needed) or rpath_seen or runpath_seen:
            raise core.GateError("candidate artifact dynamic dependency policy differs")
        if any(token in soname.lower() for soname in needed for token in forbidden):
            raise core.GateError("candidate artifact needs a forbidden library")
        return {
            "sha256": core.sha256_file(path, limit=512 * 1024 * 1024),
            "size_bytes": info.st_size,
            "mode_disabled_before_audit": True,
            "mode_after_disarm": oct(disarmed_mode),
            "elf_type": "ET_DYN",
            "entry_point": e_entry,
            "dt_flags_1": flags_1,
            "df_1_pie": True,
            "interpreter": interpreter,
            "needed": needed,
            "rpath": rpath_seen,
            "runpath": runpath_seen,
            "gnu_stack_executable": False,
        }
    finally:
        os.close(descriptor)


def bwrap_prefix_v13(*, phase: str) -> list[str]:
    argv = previous.bwrap_prefix_v12(phase=phase)
    replacements = {
        previous.COPY_PROFILE: COPY_PROFILE,
        previous.BUILD_PROFILE: BUILD_PROFILE,
        str(previous.OUTPUT_ROOT): str(OUTPUT_ROOT),
    }
    result = [replacements.get(item, item) for item in argv]
    if any(item in {previous.COPY_PROFILE, previous.BUILD_PROFILE, str(previous.OUTPUT_ROOT)} for item in result):
        raise core.GateError("v13 argv retains a v12 profile or output path")
    return result


def verify_review_artifacts_v13() -> dict[str, Any]:
    policy = core.read_json_object(POLICY_PATH)
    schema = core.read_json_object(SCHEMA_PATH)
    previous_policy = core.read_json_object(previous.POLICY_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise core.GateError("execution policy schema is not a closed exact top-level contract")
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-u3-cpu-build-execution-policy-v13",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_CPU_BUILD_EXECUTION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_CPU_BUILD_EXECUTION_V13",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_source_copy_and_one_exact_cpu_build_attempt",
        "status": "REVIEWED_SINGLE_ATTEMPT_CPU_BUILD_V13_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "RUN_ONE_SOURCE_COPY_THEN_ONE_CPU_BUILD_AND_STATIC_ELF_AUDIT",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            raise core.GateError(f"policy field differs: {key}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "run-source-copy", "run-build"]:
        raise core.GateError("policy command surface differs")
    if policy.get("outer_privilege_handoff") != {
        "only_privileged_child_command": ["/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups", "--", "/usr/bin/python3", "r8_liquid_target_u3_cpu_build_gate_v13.py", "<fixed_phase>"],
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
        "source_copy_profile_path": "config/apparmor_drafts/r8-liquid-u3-source-copy-v13.profile",
        "build_profile": BUILD_PROFILE,
        "build_profile_path": "config/apparmor_drafts/r8-liquid-u3-cpu-build-v13.profile",
        "source_copy_profile_sha256": COPY_PROFILE_SHA256,
        "build_profile_sha256": BUILD_PROFILE_SHA256,
    }:
        raise core.GateError("policy frozen attempt identity differs")
    for key in ("source_input", "isolation", "resources", "source_copy_command", "build_command", "generated_wrapper", "profile_lifecycle", "invariants"):
        if policy.get(key) != previous_policy.get(key):
            raise core.GateError(f"v13 widens or differs from frozen v12 policy field: {key}")
    expected_audit = {
        "candidate": "output/artifacts/DualSPHysics5.4CPU_linux64",
        "execution": "forbidden_during_and_after_this_gate",
        "elf_type": "ET_DYN_ONLY_WITH_DF_1_PIE",
        "accepted_e_type": ET_DYN,
        "required_dt_flags_1_pie_mask": DF_1_PIE,
        "entry_point": "nonzero_within_executable_pt_load",
        "gnu_stack": "exactly_one_non_executable_pt_gnu_stack",
        "interpreter": "/lib64/ld-linux-x86-64.so.2",
        "required_needed": ["libgomp.so.1", "libstdc++.so.6", "libgcc_s.so.1", "libc.so.6"],
        "forbidden_needed_substrings": ["chrono", "dsph", "cuda", "nvidia", "ros", "gazebo", "wavegen", "moordyn"],
        "rpath_or_runpath": "forbidden",
        "disable_opened_regular_candidate_before_audit": True,
        "artifact_mode_after_audit": "0400",
    }
    if policy.get("static_artifact_audit") != expected_audit:
        raise core.GateError("v13 PIE static-artifact contract differs")
    if policy.get("isolation", {}).get("synthetic_symlinks") != EXPECTED_SYMLINKS:
        raise core.GateError("policy synthetic-link boundary differs")
    if policy.get("generated_wrapper", {}).get("content") != core.WRAPPER_CONTENT:
        raise core.GateError("policy generated wrapper differs")
    effective_profiles: dict[Path, str] = {}
    for profile_path, expected_sha, profile_name in ((COPY_PROFILE_PATH, COPY_PROFILE_SHA256, COPY_PROFILE), (BUILD_PROFILE_PATH, BUILD_PROFILE_SHA256, BUILD_PROFILE)):
        if core.sha256_file(profile_path, limit=128 * 1024) != expected_sha:
            raise core.GateError(f"reviewed profile digest differs: {profile_path}")
        text = profile_path.read_text(encoding="utf-8")
        if f"profile {profile_name} flags=(attach_disconnected,mediate_deleted)" not in text or "REVIEWED_SINGLE_ATTEMPT_ONLY" not in text:
            raise core.GateError(f"reviewed profile identity/header differs: {profile_path}")
        effective = _effective_profile(text)
        for token in core.FORBIDDEN_PROFILE_TOKENS:
            if token in effective:
                raise core.GateError(f"reviewed profile violates a forbidden boundary: {token}")
        if [line.strip() for line in effective.splitlines() if line.strip().startswith("/newroot/lib")] != ["/newroot/lib wl,", "/newroot/lib64 wl,"]:
            raise core.GateError("reviewed profile synthetic-link creation surface differs")
        if "/newroot/lib/**" in effective or "/newroot/lib rw" in effective or "/oldroot/lib" in effective:
            raise core.GateError("reviewed profile adds an impermissible host /lib surface")
        if [line.strip() for line in effective.splitlines() if "/proc/sys" in line and " w" in line] != ["/proc/sys/user/max_user_namespaces w,"]:
            raise core.GateError("reviewed profile proc-sys write surface differs")
        if [line.strip() for line in effective.splitlines() if line.strip().startswith("capability ")] != ["capability sys_admin,", "capability sys_ptrace,", "capability sys_resource,", "capability setpcap,", "capability net_admin,"]:
            raise core.GateError("reviewed profile capability surface differs")
        effective_profiles[profile_path] = effective
    headers = {"/usr/include/ r,", "/usr/include/** r,"}
    build_rules = {line.strip() for line in effective_profiles[BUILD_PROFILE_PATH].splitlines() if line.strip()}
    if not headers <= build_rules or any(line.startswith("/usr/include") and line not in headers for line in build_rules):
        raise core.GateError("build profile header surface differs")
    if "/usr/include" in effective_profiles[COPY_PROFILE_PATH]:
        raise core.GateError("source-copy profile must not gain compiler header access")
    for phase in ("source-copy", "build"):
        argv = bwrap_prefix_v13(phase=phase)
        ro_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--ro-bind"]
        expected_ro = [["/usr", "/usr"]] + ([[str(core.SOURCE_ROOT), "/work/input"]] if phase == "source-copy" else [])
        if ro_pairs != expected_ro:
            raise core.GateError("bwrap read-only bind boundary differs")
        if [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--symlink"] != [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]]:
            raise core.GateError("bwrap synthetic-link boundary differs")
        bind_pairs = [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item in {"--ro-bind", "--bind"}]
        if any(pair[0] in {"/lib", "/lib64"} or pair[1] in {"/lib", "/lib64"} for pair in bind_pairs):
            raise core.GateError("bwrap must not bind a host lib path")
        if [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == "--bind"] != [[str(OUTPUT_ROOT), "/work/output"]]:
            raise core.GateError("bwrap must retain exactly one writable output bind")
        if [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == "--tmpfs"] != ["/work"]:
            raise core.GateError("bwrap guest tmpfs boundary differs")
        if "--proc" in argv or "--dev" in argv or "--share-net" in argv:
            raise core.GateError("bwrap guest device or network boundary differs")
    if core.OUTPUT_SOURCE != OUTPUT_SOURCE or core.ARTIFACT_PATH != ARTIFACT_PATH:
        raise core.GateError("derived output paths were not rebound to fresh v13 output")
    return {
        "policy_path": str(POLICY_PATH),
        "policy_sha256": core.sha256_file(POLICY_PATH, limit=256 * 1024),
        "policy_canonical_sha256": core.canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_sha256": core.sha256_file(SCHEMA_PATH, limit=256 * 1024),
        "frozen_v12_gate": {"path": str(PREVIOUS_GATE_PATH), "sha256": PREVIOUS_GATE_SHA256},
        "frozen_v12_policy": {"path": str(previous.POLICY_PATH), "sha256": PREVIOUS_POLICY_SHA256},
        "source_copy_profile": {"path": str(COPY_PROFILE_PATH), "sha256": COPY_PROFILE_SHA256},
        "build_profile": {"path": str(BUILD_PROFILE_PATH), "sha256": BUILD_PROFILE_SHA256},
    }


def execute_build_v13() -> int:
    """Run the inherited fixed build state machine with fail-closed artifact mode.

    This is intentionally a narrow copy of the v10 state transition.  Its only
    behavioral difference is that the opened, verified candidate inode is made
    0400 before parsing it, including when parsing later rejects it.  No
    candidate is ever executed.
    """

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
        audit = static_elf_audit_v13(ARTIFACT_PATH)
        output = core.inventory_build_output(expected)
        final.update({
            "status": "PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT",
            "artifact": {
                **audit,
                "path": str(ARTIFACT_PATH),
                "mode_after_audit": audit["mode_after_disarm"],
            },
            "output_inventory": output,
            "next_allowed_stage": "SEPARATE_GENCASE_AND_SOLVER_EXECUTION_ADMISSION_REQUIRED",
        })
    except (core.GateError, OSError, UnicodeError, ValueError) as exc:
        failure_artifact: dict[str, Any] = {}
        try:
            info = os.lstat(ARTIFACT_PATH)
            if stat.S_ISREG(info.st_mode) and not stat.S_ISLNK(info.st_mode) and info.st_nlink == 1:
                mode = stat.S_IMODE(info.st_mode)
                failure_artifact = {
                    "path": str(ARTIFACT_PATH),
                    "mode_at_failure": oct(mode),
                    "execute_disabled_at_failure": not bool(mode & 0o111),
                }
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
core.static_elf_audit = static_elf_audit_v13
core.verify_review_artifacts = verify_review_artifacts_v13
core.bwrap_prefix = bwrap_prefix_v13
core.execute_build = execute_build_v13

for _name in dir(core):
    if not _name.startswith("_") and _name not in globals():
        globals()[_name] = getattr(core, _name)


if __name__ == "__main__":
    raise SystemExit(core.main())
