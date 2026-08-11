#!/usr/bin/env python3
"""Bounded-output revision of the RTX 5080 static-audit gate.

V1 remains immutable evidence.  V2 changes one parser suffix: raw
``cuobjdump --dump-elf`` is replaced by an exact in-sandbox Python
orchestrator that extracts 11 cubins to guest tmpfs, runs readelf there, and
returns bounded JSON.  Candidate, objects, parent hashes, limits, isolation,
and all remaining checks are inherited byte-for-byte from V1.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import re
import stat
import sys
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping, Sequence


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_static_audit_gate_v1.py"
BASE_GATE_SHA256 = "c21ef005e4375ec24c92fd19bec4fabd3e11ba630b6a5b34a6c2981a4e7cbe28"
POLICY_PATH = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v2.json"
)
POLICY_SHA256 = "18e3869bcf0ad280ce06f1ca9803d6b9420f5df45066c3dae6048c28fba600d5"
PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2"
PROFILE_SHA256 = "b3a2225ed5d988932387cbc487dab637bd88a565de219d74a7cf5afd9a60647f"
V1_FAILURE_SHA256 = "48d9ef83a0673df5a2befa3d3e5fa1074218268699928abc9cc3fb13340ed656"
OVERSIZE_SHA256 = "d8804829915fc3809357375cf82dae201dfefe0e637700dba7ed982fa9e74731"
GENERATOR_SHA256 = "2c163e6fff6a1054cb64a819d4a3186e68389ba4beaed7cc270152c5f66dd86d"
HELPER_SHA256 = "e83b48c3d41167bb3f3f3c1453b689228ba8d207d1357c88ebed31d43a95180d"


class RevisionError(RuntimeError):
    pass


def sha256_regular(path: Path) -> str:
    info = path.lstat()
    if path.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise RevisionError(f"unsafe revision input: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def load_base() -> ModuleType:
    if sha256_regular(BASE_GATE_PATH) != BASE_GATE_SHA256:
        raise RevisionError("frozen v1 static-audit gate hash drift")
    if sha256_regular(POLICY_PATH) != POLICY_SHA256:
        raise RevisionError("v2 static-audit policy hash drift")
    spec = importlib.util.spec_from_file_location("r8_gpu_static_audit_gate_v1_for_v2", BASE_GATE_PATH)
    if spec is None or spec.loader is None:
        raise RevisionError("cannot import frozen v1 static-audit gate")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BASE = load_base()
BASE_VALIDATE_POLICY = BASE.validate_policy
BASE_VERIFY_PARENT_AND_TOOLS = BASE.verify_parent_and_tool_hashes


def policy() -> dict[str, Any]:
    return BASE.read_json(POLICY_PATH)


def validate_policy(current: Mapping[str, Any]) -> dict[str, Any]:
    if set(current) != {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "campaign_id",
        "build_id",
        "parent_inputs",
        "implementation",
        "candidate",
        "objects",
        "profile",
        "trusted_tools",
        "sandbox",
        "candidate_commands",
        "object_command",
        "acceptance",
        "outputs",
        "safety",
        "revision_evidence",
    }:
        raise RevisionError("v2 policy closed top-level keys drift")
    if (
        current.get("schema_version") != "r8-liquid-gpu-static-audit-policy-v2"
        or current.get("policy_id") != "LIQUID_ZRJ_MSI_U2404_GPU_STATIC_AUDIT_20260810T170339Z_V2"
    ):
        raise RevisionError("v2 policy identity drift")

    profile = current["profile"]
    if profile != {
        "name": PROFILE_NAME,
        "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2.profile",
        "mode_octal": "0600",
        "size_bytes": 61138,
        "sha256": PROFILE_SHA256,
    }:
        raise RevisionError("v2 profile identity drift")
    revision = current["revision_evidence"]
    if (
        revision.get("root_cause") != "RAW_CUOBJDUMP_DUMP_ELF_EXCEEDED_FROZEN_256_MIB_OUTPUT_LIMIT"
        or revision.get("single_semantic_delta")
        != "EXTRACT_11_CUBINS_TO_GUEST_TMPFS_AND_EMIT_BOUNDED_READELF_JSON"
        or revision.get("failed_lifecycle", {}).get("sha256") != V1_FAILURE_SHA256
        or revision.get("oversize_output", {}).get("size_bytes") != 268435456
        or revision.get("oversize_output", {}).get("sha256") != OVERSIZE_SHA256
        or revision.get("candidate_rebuilt") is not False
        or revision.get("candidate_executed") is not False
        or revision.get("network_or_gpu_added") is not False
    ):
        raise RevisionError("v2 failure-bound single-delta evidence drift")

    trusted = current["trusted_tools"]
    if not isinstance(trusted, list) or len(trusted) != 14:
        raise RevisionError("v2 trusted tool set must add exactly two Python path identities")
    expected_python = [
        {
            "path": "/usr/bin/python3",
            "realpath": "/usr/bin/python3.12",
            "sha256": "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118",
        },
        {
            "path": "/usr/bin/python3.12",
            "realpath": "/usr/bin/python3.12",
            "sha256": "1643dacd9feaedc58f3cc581e4d22577dfe25c09b10282936186ccf0f2e61118",
        },
    ]
    if trusted[-2:] != expected_python:
        raise RevisionError("v2 Python helper tool identities drift")

    commands = [item for item in current["candidate_commands"] if item.get("id") == "cuobjdump_dump_elf"]
    if len(commands) != 1:
        raise RevisionError("v2 bounded cubin helper command is not unique")
    helper_argv = commands[0].get("argv")
    if (
        not isinstance(helper_argv, list)
        or len(helper_argv) != 5
        or helper_argv[:4] != ["/usr/bin/python3", "-I", "-S", "-c"]
        or hashlib.sha256(helper_argv[4].encode("utf-8")).hexdigest() != HELPER_SHA256
        or commands[0].get("resource") != "cuobjdump_dump"
    ):
        raise RevisionError("v2 bounded cubin helper bytes/argv drift")
    helper = helper_argv[4]
    for required in (
        "--extract-elf",
        "/audit/tmp",
        "/usr/bin/readelf",
        "PASS_EXTRACTED_CUBIN_STATIC_METADATA",
        "guest_tmpfs_only",
    ):
        if required not in helper:
            raise RevisionError(f"v2 helper missing invariant: {required}")
    for forbidden in ("shell=True", "socket", "urllib", "requests", "/dev/", "/proc/", "os.system"):
        if forbidden in helper:
            raise RevisionError(f"v2 helper contains forbidden capability: {forbidden}")

    # Reuse all v1 checks after normalizing only the revision identity/profile
    # and presenting the original 12-tool prefix.  No candidate/object/limit/
    # acceptance field is normalized or relaxed.
    normalized = copy.deepcopy(dict(current))
    normalized.pop("revision_evidence")
    normalized["schema_version"] = "r8-liquid-gpu-static-audit-policy-v1"
    normalized["policy_id"] = "LIQUID_ZRJ_MSI_U2404_GPU_STATIC_AUDIT_20260810T170339Z_V1"
    normalized["profile"] = {
        "name": "r8-liquid-u3-gpu-static-audit-20260810T170339Z",
        "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile",
        "mode_octal": "0600",
        "size_bytes": 61078,
        "sha256": "d38092826e9b17d59c20d9ef1be3b30517c5cc106e316c394877c45c72f693ed",
    }
    normalized["trusted_tools"] = normalized["trusted_tools"][:12]
    saved_name, saved_hash = BASE.PROFILE_NAME, BASE.PROFILE_SHA256
    BASE.PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z"
    BASE.PROFILE_SHA256 = "d38092826e9b17d59c20d9ef1be3b30517c5cc106e316c394877c45c72f693ed"
    try:
        BASE_VALIDATE_POLICY(normalized)
    finally:
        BASE.PROFILE_NAME, BASE.PROFILE_SHA256 = saved_name, saved_hash
    return dict(current)


def verify_parent_and_tool_hashes(current: Mapping[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(dict(current))
    normalized["trusted_tools"] = normalized["trusted_tools"][:12]
    result = BASE_VERIFY_PARENT_AND_TOOLS(normalized)
    for key in ("parent_policy", "parent_profile", "failed_lifecycle", "oversize_output"):
        entry = current["revision_evidence"][key]
        path = Path(entry["path"])
        if sha256_regular(path) != entry["sha256"]:
            raise RevisionError(f"v2 revision evidence hash drift: {key}")
    if current["revision_evidence"]["oversize_output"]["size_bytes"] != Path(
        current["revision_evidence"]["oversize_output"]["path"]
    ).lstat().st_size:
        raise RevisionError("v1 oversize output size drift")
    for item in current["trusted_tools"][-2:]:
        path = Path(item["path"])
        if str(path.resolve(strict=True)) != item["realpath"] or sha256_regular(Path(item["realpath"])) != item["sha256"]:
            raise RevisionError(f"v2 Python tool drift: {path}")
    if current["implementation"]["profile_generator"]["sha256"] != GENERATOR_SHA256:
        raise RevisionError("v2 revision generator identity drift")
    result["tool_count"] = 14
    result["revision_evidence_count"] = 4
    return result


def parse_cuda(results: Mapping[str, Mapping[str, Any]], current: Mapping[str, Any]) -> dict[str, Any]:
    list_elf = BASE.output_text(results["cuobjdump_list_elf"])
    cubins = re.findall(r"^ELF file\s+\d+:\s+(\S+\.sm_(\d+)\.cubin)\s*$", list_elf, re.MULTILINE)
    list_ptx = BASE.output_text(results["cuobjdump_list_ptx"])
    ptx = re.findall(r"^PTX file\s+\d+:\s+(\S+\.sm_(\d+)\.ptx)\s*$", list_ptx, re.MULTILINE)
    native_arches = sorted({f"sm_{arch}" for _, arch in cubins})
    ptx_arches = sorted({f"sm_{arch}" for _, arch in ptx})
    if len(cubins) != 11 or len(ptx) != 11:
        raise RevisionError(f"fatbin cardinality drift cubin={len(cubins)} ptx={len(ptx)}")
    if native_arches != ["sm_120"] or ptx_arches != ["sm_120"]:
        raise RevisionError("fatbin contains a non-sm_120 architecture")

    helper_text = BASE.output_text(results["cuobjdump_dump_elf"])
    helper_lines = [line for line in helper_text.splitlines() if line.strip()]
    if len(helper_lines) != 1:
        raise RevisionError("bounded cubin helper emitted a non-single JSON record")
    try:
        helper = json.loads(helper_lines[0])
    except json.JSONDecodeError as exc:
        raise RevisionError("bounded cubin helper output is not JSON") from exc
    if not isinstance(helper, dict) or set(helper) != {
        "status",
        "cubin_count",
        "architecture",
        "nonzero_executable_text_section_count",
        "extract_stdout_sha256",
        "extract_stderr_sha256",
        "records",
        "guest_tmpfs_only",
    }:
        raise RevisionError("bounded cubin helper closed output keys drift")
    records = helper.get("records")
    if (
        helper.get("status") != "PASS_EXTRACTED_CUBIN_STATIC_METADATA"
        or helper.get("cubin_count") != 11
        or helper.get("architecture") != "sm_120"
        or helper.get("guest_tmpfs_only") is not True
        or not isinstance(helper.get("nonzero_executable_text_section_count"), int)
        or helper["nonzero_executable_text_section_count"] < 1
        or not isinstance(records, list)
        or len(records) != 11
    ):
        raise RevisionError("bounded cubin helper acceptance failed")
    expected_names = [f"DualSPHysics5.{index}.sm_120.cubin" for index in range(1, 12)]
    for record, expected_name in zip(records, expected_names):
        if (
            not isinstance(record, dict)
            or set(record) != {
                "name",
                "size_bytes",
                "sha256",
                "elf_type",
                "machine",
                "nonzero_executable_text",
                "readelf_stdout_sha256",
                "readelf_stderr_sha256",
                "readelf_stderr_bytes",
            }
            or record.get("name") != expected_name
            or record.get("elf_type") != "EXEC (Executable file)"
            or record.get("machine") != "NVIDIA CUDA architecture"
            or not isinstance(record.get("size_bytes"), int)
            or record["size_bytes"] <= 0
            or not isinstance(record.get("nonzero_executable_text"), int)
            or record["nonzero_executable_text"] < 1
            or re.fullmatch(r"[0-9a-f]{64}", str(record.get("sha256"))) is None
        ):
            raise RevisionError(f"bounded cubin record failed: {expected_name}")

    symbols = BASE.output_text(results["cuobjdump_dump_elf_symbols"])
    function_symbol_count = sum(
        1 for line in symbols.splitlines() if "STT_FUNC" in line and "STO_ENTRY" in line
    )
    if function_symbol_count < 1:
        raise RevisionError("native cubin symbols lack a kernel function")
    dump_ptx = BASE.output_text(results["cuobjdump_dump_ptx"])
    dump_arches = sorted(set(re.findall(r"^arch = (sm_\d+)\s*$", dump_ptx, re.MULTILINE)))
    targets = sorted(set(re.findall(r"^\.target\s+(sm_\d+)\s*$", dump_ptx, re.MULTILINE)))
    if dump_arches != ["sm_120"] or targets != ["sm_120"] or ".visible .entry " not in dump_ptx:
        raise RevisionError("PTX dump lacks paired sm_120 target/kernel evidence")

    build_stdout = Path(current["parent_inputs"]["build_stdout"]["path"]).read_text(
        encoding="utf-8", errors="replace"
    )
    nvcc_lines = [line for line in build_stdout.splitlines() if line.startswith("/usr/local/cuda-12.8/bin/nvcc ")]
    exact_gencode = current["acceptance"]["exact_gencode"]
    exact = all(line.split().count(exact_gencode) == 1 for line in nvcc_lines)
    if len(nvcc_lines) != 11 or not exact:
        raise RevisionError("NVCC log does not prove 11 exact compute_120/sm_120 translations")
    return {
        "native_cubin_count": len(cubins),
        "native_arches": native_arches,
        "ptx_count": len(ptx),
        "ptx_arches": ptx_arches,
        "nonzero_executable_text_section_count": helper["nonzero_executable_text_section_count"],
        "function_symbol_count": function_symbol_count,
        "gencode_proof": {
            "nvcc_command_count": len(nvcc_lines),
            "all_use_cuda_12_8": True,
            "all_use_exact_gencode": exact,
            "compute_120_proven": bool(ptx) and exact,
        },
    }


def revision_check() -> dict[str, Any]:
    current = validate_policy(policy())
    parents = verify_parent_and_tool_hashes(current)
    return {
        "status": "PASS_BOUNDED_CUBIN_AUDIT_REVISION_V2",
        "base_gate_sha256": BASE_GATE_SHA256,
        "policy_sha256": POLICY_SHA256,
        "profile_sha256": PROFILE_SHA256,
        "helper_sha256": HELPER_SHA256,
        "v1_failure_sha256": V1_FAILURE_SHA256,
        "tool_count": parents["tool_count"],
        "semantic_delta_count": 1,
        "candidate_rebuilt": False,
        "candidate_executed": False,
    }


# Patch only the revision points in the frozen implementation module.  Python
# global name lookup in V1 then resolves these exact functions/constants.
BASE.POLICY_PATH = POLICY_PATH
BASE.GATE_PATH = Path(__file__).resolve()
BASE.PROFILE_NAME = PROFILE_NAME
BASE.PROFILE_SHA256 = PROFILE_SHA256
BASE.policy = policy
BASE.validate_policy = validate_policy
BASE.verify_parent_and_tool_hashes = verify_parent_and_tool_hashes
BASE.parse_cuda = parse_cuda


def main(argv: Sequence[str] | None = None) -> int:
    try:
        revision_check()
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, RevisionError, BASE.AuditError) as exc:
        print(json.dumps({"status": "FAIL_GPU_BUILD_STATIC_AUDIT", "phase": "revision-check", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    return BASE.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
