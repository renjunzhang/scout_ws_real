"""Static-only closed contract gate for the RTX 5080 DualSPHysics GPU campaign.

G1 deliberately exposes no source-copy, build, profile-load, parser execution,
or candidate-execution command.  It validates frozen bytes and provides pure
validators/builders that later goals must reuse after separate authorization.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from jsonschema import Draft202012Validator


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_DIR.parents[3]
POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gpu_build_execution_policy_v1.json"

PLAN_PATH = (
    WORKSPACE_ROOT
    / "docs/实物实验注意事项/对比试验/仿真接入液体/"
    "20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
)
PLAN_COMMIT = "a7409aff3bd280392491606503893e36f1cc5888"
PLAN_BLOB = "7b2eb275d0fb6cb882ec701aabd7b7af229f353c"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"

CAMPAIGN_ID = "u3_source_gpu_build_sm120_20260810T102641Z"
BUILD_ID_A = CAMPAIGN_ID + "_a"
BUILD_ID_B = CAMPAIGN_ID + "_b"
ROOT_A = Path(f"/home/zrj/scout_liquid_lab/build/{BUILD_ID_A}.partial")
ROOT_B = Path(f"/home/zrj/scout_liquid_lab/build/{BUILD_ID_B}.partial")
OUTPUT_A = ROOT_A / "output"
OUTPUT_B = ROOT_B / "output"

PARALLEL_JOBS = 1
WALL_TIMEOUT_SECONDS = 5400
CPU_LIMIT_SECONDS = 5400
MINIMUM_AVAILABLE_MEMORY_BYTES = 4294967296
PARALLEL_JOBS_MEMORY_THRESHOLD_BYTES = 8589934592
ADDRESS_SPACE_LIMIT_BYTES = 8589934592
MONITOR_INTERVAL_SECONDS = 20
MONITOR_INTERVAL_MINIMUM_SECONDS = 10
MONITOR_INTERVAL_MAXIMUM_SECONDS = 30
STATIC_OUTPUT_LIMIT_BYTES = 268435456
STREAM_PREFIX_LIMIT_BYTES = 65536

GENCODE_ARG = (
    'GENCODE=-gencode=arch=compute_120,code=\\"sm_120,compute_120\\"'
)
MAKE_ARGV = (
    "/usr/bin/make",
    "--no-builtin-rules",
    "--no-builtin-variables",
    "-f",
    "U3GpuBuild.mk",
    "-j1",
    "SHELL=/usr/bin/dash",
    "CC=/usr/bin/x86_64-linux-gnu-g++-11",
    "NCC=/usr/local/cuda-12.8/bin/nvcc",
    "CUDA=12",
    "DIRTOOLKIT=/usr/local/cuda-12.8",
    GENCODE_ARG,
    "USE_DEBUG=NO",
    "USE_FAST_MATH=NO",
    "USE_NATIVE_CPU_OPTIMIZATIONS=NO",
    "COMPILE_CHRONO=NO",
    "COMPILE_WAVEGEN=NO",
    "COMPILE_MOORDYNPLUS=NO",
    "LIBS_DIRECTORIES=",
    "EXECS_DIRECTORY=/work/output/artifacts",
    "/work/output/artifacts/DualSPHysics5.4_linux64",
)

WRAPPER_BYTES = (
    b".SUFFIXES:\n"
    b".SUFFIXES: .cpp .o\n"
    b"include Makefile\n"
    b"override CCFLAGS += -include cstdint\n"
)
WRAPPER_MODE = 0o600
WRAPPER_SHA256 = "33883dcdde741c253a09d78decb3c21c5ab92c7c9152acf793d7dd025f07300d"
OBJECT_CANONICAL_SHA256 = (
    "38023e8b24f1d1731ba3a7d03bbb5fdf2e74e5623341402d607cba046b08d7e2"
)
MAKEFILE_SHA256 = "0e7d60ed96437ae22c1d411ec7d12fcae1adad8a9e91f39dd0ea6023846c63f1"

SOURCE_COPY_STATES = (
    "START_RECORD_CREATE_NEW",
    "ISOLATED_COPY_COMPLETE",
    "SEALED_352_INVENTORY_VERIFIED",
    "COPY_PROFILE_UNLOADED_ZERO_RESIDUE",
    "WRAPPER_O_EXCL_0600_CREATED",
    "COMPLETE_353_BUILD_INPUT_INVENTORY_VERIFIED",
    "FINAL_RECEIPT_CREATE_NEW",
)
EVENT_KEYS = {"state", "sequence", "captured_at_ns", "evidence"}
STATE_EVIDENCE_KEYS = {
    SOURCE_COPY_STATES[0]: {"create_new", "path"},
    SOURCE_COPY_STATES[1]: {"isolated", "copied_entry_count"},
    SOURCE_COPY_STATES[2]: {
        "entry_count",
        "extra_count",
        "symlink_count",
        "hardlink_count",
        "elf_count",
        "executable_count",
    },
    SOURCE_COPY_STATES[3]: {"profile_name", "unloaded", "zero_residue"},
    SOURCE_COPY_STATES[4]: {
        "create_flags",
        "mode_octal",
        "sha256",
        "size_bytes",
        "after_unload_sequence",
    },
    SOURCE_COPY_STATES[5]: {
        "entry_count",
        "sealed_entry_count",
        "wrapper_entry_count",
        "extra_count",
        "symlink_count",
        "hardlink_count",
        "elf_count",
        "executable_count",
        "wrapper_sha256",
        "wrapper_mode_octal",
    },
    SOURCE_COPY_STATES[6]: {
        "create_new",
        "path",
        "published",
        "after_complete_inventory_sequence",
    },
}
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")
UTC_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$")

OBJECT_VARIABLES = {
    "OBJXML",
    "OBJSPHMOTION",
    "OBCOMMON",
    "OBCOMMONDSPH",
    "OBSPH",
    "OBSPHSINGLE",
    "OBCOMMONGPU",
    "OBSPHGPU",
    "OBSPHSINGLEGPU",
    "OBCUDA",
    "OBWAVERZ",
    "OBWAVERZCUDA",
    "OBCHRONO",
    "OBMOORDYNPLUS",
    "OBINOUT",
    "OBINOUTGPU",
    "OBMESH",
    "OBVRES",
    "OBFLEXSTRUC",
    "OBJECTS",
}


class GateError(RuntimeError):
    """Fail-closed static contract error."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)


def git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def canonical_hash(value: Any) -> str:
    data = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(data)


def read_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise GateError(f"expected JSON object: {path}")
    return value


def repo_path(relative: str) -> Path:
    candidate = WORKSPACE_ROOT / relative
    resolved_parent = candidate.parent.resolve()
    workspace = WORKSPACE_ROOT.resolve()
    if workspace != resolved_parent and workspace not in resolved_parent.parents:
        raise GateError(f"repository path escapes workspace: {relative}")
    return candidate


def require_exact_keys(value: Mapping[str, Any], keys: Iterable[str], label: str) -> None:
    expected = set(keys)
    actual = set(value)
    if actual != expected:
        raise GateError(
            f"{label} keys drift: missing={sorted(expected - actual)} "
            f"extra={sorted(actual - expected)}"
        )


def require_hex64(value: Any, label: str) -> str:
    if not isinstance(value, str) or HEX64_RE.fullmatch(value) is None:
        raise GateError(f"{label} is not a lowercase SHA-256")
    return value


def verify_plan_identity() -> dict[str, str]:
    data = PLAN_PATH.read_bytes()
    actual_sha = sha256_bytes(data)
    actual_blob = git_blob_sha1(data)
    if actual_sha != PLAN_SHA256 or actual_blob != PLAN_BLOB:
        raise GateError(
            f"frozen plan identity drift: sha256={actual_sha} blob={actual_blob}"
        )
    return {
        "commit": PLAN_COMMIT,
        "blob": actual_blob,
        "sha256": actual_sha,
    }


def verify_recursive_closed_schema(schema: Any, location: str = "$") -> None:
    if isinstance(schema, dict):
        if schema.get("type") == "object":
            if schema.get("additionalProperties") is not False:
                raise GateError(f"schema object is not closed at {location}")
            properties = schema.get("properties")
            required = schema.get("required")
            if not isinstance(properties, dict) or set(required or ()) != set(properties):
                raise GateError(f"schema required/properties drift at {location}")
        if schema.get("type") == "array":
            prefix = schema.get("prefixItems")
            if not isinstance(prefix, list) or schema.get("items") is not False:
                raise GateError(f"schema array is not exact at {location}")
            if schema.get("minItems") != len(prefix) or schema.get("maxItems") != len(prefix):
                raise GateError(f"schema array length is not frozen at {location}")
        for key, item in schema.items():
            verify_recursive_closed_schema(item, f"{location}/{key}")
    elif isinstance(schema, list):
        for index, item in enumerate(schema):
            verify_recursive_closed_schema(item, f"{location}/{index}")


def validate_policy_schema(
    policy: Mapping[str, Any] | None = None,
    schema: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    policy_obj = dict(policy) if policy is not None else read_json_object(POLICY_PATH)
    schema_obj = dict(schema) if schema is not None else read_json_object(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema_obj)
    verify_recursive_closed_schema(schema_obj)
    errors = sorted(
        Draft202012Validator(schema_obj).iter_errors(policy_obj),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        first = errors[0]
        raise GateError(
            "closed schema rejection at "
            + "/".join(str(part) for part in first.absolute_path)
            + f": {first.message}"
        )
    return policy_obj


def verify_frozen_references(policy: Mapping[str, Any]) -> list[dict[str, str]]:
    verified: list[dict[str, str]] = []
    for item in policy["frozen_reference_inputs"]:
        path = repo_path(item["path"])
        actual = sha256_file(path)
        if actual != item["sha256"]:
            raise GateError(f"frozen CPU v15 reference drift: {item['path']}")
        verified.append({"path": item["path"], "sha256": actual})
    return verified


def verify_source_inputs(policy: Mapping[str, Any]) -> dict[str, Any]:
    contract = policy["source_input"]
    receipt_path = Path(contract["source_receipt"])
    cpu_receipt_path = Path(contract["cpu_build_receipt"])
    makefile_path = Path(contract["makefile_path"])
    checks = {
        "source_receipt_sha256": sha256_file(receipt_path),
        "cpu_build_receipt_sha256": sha256_file(cpu_receipt_path),
        "makefile_sha256": sha256_file(makefile_path),
    }
    if checks["source_receipt_sha256"] != contract["source_receipt_sha256"]:
        raise GateError("sealed source receipt SHA-256 drift")
    if checks["cpu_build_receipt_sha256"] != contract["cpu_build_receipt_sha256"]:
        raise GateError("CPU build receipt SHA-256 drift")
    if checks["makefile_sha256"] != MAKEFILE_SHA256:
        raise GateError("sealed Makefile SHA-256 drift")

    receipt = read_json_object(receipt_path)
    sealed = receipt["results"]["materialization"]["sealed_output"]
    exact = {
        "canonical": receipt["receipt_sha256"],
        "manifest": sealed["manifest_sha256"],
        "file_count": sealed["file_count"],
        "total_bytes": sealed["total_bytes"],
        "symlink_count": sealed["symlink_count"],
        "hardlink_count": sealed["hardlink_count"],
    }
    expected = {
        "canonical": contract["source_receipt_canonical_sha256"],
        "manifest": contract["sealed_manifest_sha256"],
        "file_count": 352,
        "total_bytes": 5473917,
        "symlink_count": 0,
        "hardlink_count": 0,
    }
    if exact != expected:
        raise GateError(f"sealed source receipt contract drift: {exact!r}")
    return {**checks, **exact}


def _expand_make_value(value: str, variables: Mapping[str, str]) -> str:
    pattern = re.compile(r"\$\(([^)]+)\)")
    for _ in range(100):
        updated = pattern.sub(lambda match: variables.get(match.group(1), ""), value)
        if updated == value:
            return value
        value = updated
    raise GateError("recursive object variable expansion in sealed Makefile")


def static_make_object_contract(makefile_path: Path) -> dict[str, Any]:
    """Parse only fixed object assignments/rules; never invoke Make."""

    text = makefile_path.read_text(encoding="utf-8")
    variables: dict[str, str] = {}
    assignment = re.compile(r"^([A-Z][A-Z0-9_]*)\s*(:=|=)\s*(.*)$")
    for line in text.splitlines():
        if line.startswith((" ", "\t", "#")):
            continue
        match = assignment.match(line)
        if match is None or match.group(1) not in OBJECT_VARIABLES:
            continue
        name, operator, value = match.groups()
        variables[name] = (
            _expand_make_value(value, variables) if operator == ":=" else value
        )
    if "OBJECTS" not in variables:
        raise GateError("sealed Makefile has no OBJECTS assignment")
    objects = _expand_make_value(variables["OBJECTS"], variables).split()
    cuda_pattern = re.compile(r"^(\S+\.o):\s+\S+\.cu\s*$")
    cuda_objects = [
        match.group(1)
        for line in text.splitlines()
        if (match := cuda_pattern.match(line)) is not None
    ]
    cuda_set = set(cuda_objects)
    cpp_objects = [name for name in objects if name not in cuda_set]
    return {
        "object_names": sorted(objects),
        "cpp_object_names": sorted(cpp_objects),
        "cuda_object_names": sorted(cuda_objects),
        "total_object_count": len(objects),
        "cpp_object_count": len(cpp_objects),
        "cuda_object_count": len(cuda_objects),
        "duplicate_count": len(objects) - len(set(objects)),
        "object_names_canonical_sha256": sha256_bytes(
            json.dumps(sorted(objects), separators=(",", ":")).encode("utf-8")
        ),
    }


def verify_object_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    observed = static_make_object_contract(Path(policy["source_input"]["makefile_path"]))
    expected = policy["object_contract"]
    for key in (
        "object_names",
        "cpp_object_names",
        "cuda_object_names",
        "total_object_count",
        "cpp_object_count",
        "cuda_object_count",
        "duplicate_count",
        "object_names_canonical_sha256",
    ):
        if observed[key] != expected[key]:
            raise GateError(f"131-object contract drift at {key}")
    if observed["total_object_count"] != 131:
        raise GateError("object count is not 131")
    if observed["cpp_object_count"] != 120 or observed["cuda_object_count"] != 11:
        raise GateError("object language split is not 120 C++ plus 11 CUDA")
    if observed["object_names_canonical_sha256"] != OBJECT_CANONICAL_SHA256:
        raise GateError("object canonical SHA-256 constant drift")
    return observed


def render_profile(template: str, profile_name: str, attempt_root: str) -> str:
    for token in ("@@PROFILE_NAME@@", "@@ATTEMPT_ROOT@@"):
        if token not in template:
            raise GateError(f"profile template is missing token {token}")
    rendered = template.replace("@@PROFILE_NAME@@", profile_name).replace(
        "@@ATTEMPT_ROOT@@", attempt_root
    )
    if "@@" in rendered:
        raise GateError("profile rendering left an unresolved token")
    return rendered


def _verify_profile_hash(path: Path, expected: str) -> str:
    actual = sha256_file(path)
    if actual != expected:
        raise GateError(f"profile/template SHA-256 drift: {path}")
    return path.read_text(encoding="utf-8")


def validate_exact_root_profile_text(
    text: str, expected_root: str, forbidden_root: str, label: str
) -> None:
    if expected_root not in text or forbidden_root in text:
        raise GateError(f"{label} exact-root mapping drift")
    root_pattern = re.compile(
        r"(/home/zrj/scout_liquid_lab/build/"
        r"u3_source_gpu_build_sm120_[^/\s]+\.partial)"
    )
    writable_roots: set[str] = set()
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.endswith((" rw,", " mrw,")):
            continue
        match = root_pattern.search(stripped)
        if match is not None:
            writable_roots.add(match.group(1))
    if writable_roots != {expected_root}:
        raise GateError(
            f"{label} writable-root set drift: {sorted(writable_roots)}"
        )


def verify_profiles(policy: Mapping[str, Any]) -> dict[str, str]:
    profiles = policy["profiles"]
    copy_template_path = repo_path(profiles["copy_template"]["path"])
    build_template_path = repo_path(profiles["build_template"]["path"])
    copy_path = repo_path(profiles["attempt_a_copy"]["path"])
    build_path = repo_path(profiles["attempt_a_build"]["path"])
    audit_path = repo_path(profiles["campaign_static_audit"]["path"])

    copy_template = _verify_profile_hash(
        copy_template_path, profiles["copy_template"]["sha256"]
    )
    build_template = _verify_profile_hash(
        build_template_path, profiles["build_template"]["sha256"]
    )
    copy_text = _verify_profile_hash(copy_path, profiles["attempt_a_copy"]["sha256"])
    build_text = _verify_profile_hash(
        build_path, profiles["attempt_a_build"]["sha256"]
    )
    audit_text = _verify_profile_hash(
        audit_path, profiles["campaign_static_audit"]["sha256"]
    )

    if render_profile(copy_template, profiles["attempt_a_copy"]["name"], str(ROOT_A)) != copy_text:
        raise GateError("A-copy is not deterministic template rendering")
    if render_profile(build_template, profiles["attempt_a_build"]["name"], str(ROOT_A)) != build_text:
        raise GateError("A-build is not deterministic template rendering")

    root_a, root_b = str(ROOT_A), str(ROOT_B)
    for label, text in (("A-copy", copy_text), ("A-build", build_text)):
        validate_exact_root_profile_text(text, root_a, root_b, label)
        for forbidden in (
            "/dev/nvidia",
            "flags=(unconfined)",
            "network inet stream",
            "network inet6 stream",
            "/home/zrj/scout_ws",
        ):
            if forbidden in text:
                raise GateError(f"{label} contains forbidden permission: {forbidden}")

    for path in policy["toolchain"]["attempt_a_forbidden_tools"]:
        if path in build_text:
            raise GateError(f"A-build contains B-only compiler tool: {path}")
    for forbidden_exec in (
        "/usr/bin/x86_64-linux-gnu-g++-11 rix,",
        "/usr/local/cuda-12.8/bin/nvcc rix,",
        "/usr/bin/make rix,",
    ):
        if forbidden_exec in copy_text:
            raise GateError(f"A-copy contains build execution permission: {forbidden_exec}")
    for required_exec in (
        "/usr/bin/x86_64-linux-gnu-g++-11 rix,",
        "/usr/lib/gcc/x86_64-linux-gnu/11/cc1plus rix,",
        "/usr/lib/gcc/x86_64-linux-gnu/11/collect2 rix,",
        "/usr/local/cuda-12.8/bin/nvcc rix,",
        "/usr/local/cuda-12.8/bin/cudafe++ rix,",
        "/usr/local/cuda-12.8/bin/ptxas rix,",
        "/usr/local/cuda-12.8/bin/fatbinary rix,",
        "/usr/local/cuda-12.8/bin/nvlink rix,",
        "/usr/local/cuda-12.8/nvvm/bin/cicc rix,",
    ):
        if required_exec not in build_text:
            raise GateError(f"A-build is missing exact tool permission: {required_exec}")

    if "*.o" in audit_text:
        raise GateError("static-audit profile contains wildcard object input")
    for forbidden_exec in (
        "/usr/bin/make rix,",
        "/usr/bin/dash rix,",
        "/usr/bin/x86_64-linux-gnu-g++-11 rix,",
        "/usr/bin/x86_64-linux-gnu-g++-13 rix,",
        "/usr/local/cuda-12.8/bin/nvcc rix,",
        "/usr/bin/ld rix,",
    ):
        if forbidden_exec in audit_text:
            raise GateError(
                f"static-audit profile contains forbidden executable: {forbidden_exec}"
            )
    for root in (root_a, root_b):
        candidate = f"{root}/output/artifacts/DualSPHysics5.4_linux64"
        if f"  {candidate} r," not in audit_text:
            raise GateError(f"static-audit candidate read rule missing: {candidate}")
        for line in audit_text.splitlines():
            if root in line and line.strip().endswith((" rw,", " mrw,")):
                raise GateError(f"static-audit host attempt path is writable: {line}")
        for name in policy["object_contract"]["object_names"]:
            host_input = f"{root}/output/buildtree/src/source/{name}"
            guest_input = f"/newroot/audit/input/{name}"
            if f"  {host_input} r," not in audit_text:
                raise GateError(f"static-audit exact object read rule missing: {host_input}")
            mount_rule = (
                f"mount options=(rw, rbind) /oldroot{host_input} -> {guest_input},"
            )
            if mount_rule not in audit_text:
                raise GateError(f"static-audit exact object mount rule missing: {name}")

    for forbidden in (
        "flags=(unconfined)",
        "network inet stream",
        "network inet6 stream",
        "/dev/nvidia",
        "/home/zrj/scout_ws",
    ):
        if forbidden in audit_text:
            raise GateError(f"static-audit profile forbidden token: {forbidden}")
    return {
        "copy_template": sha256_file(copy_template_path),
        "build_template": sha256_file(build_template_path),
        "a_copy": sha256_file(copy_path),
        "a_build": sha256_file(build_path),
        "static_audit": sha256_file(audit_path),
    }


def verify_resources_and_make_argv(policy: Mapping[str, Any]) -> dict[str, Any]:
    resources = policy["resources"]
    expected = {
        "parallel_jobs": PARALLEL_JOBS,
        "parallel_jobs_memory_threshold_bytes": PARALLEL_JOBS_MEMORY_THRESHOLD_BYTES,
        "minimum_available_memory_bytes": MINIMUM_AVAILABLE_MEMORY_BYTES,
        "wall_timeout_seconds": WALL_TIMEOUT_SECONDS,
        "cpu_limit_seconds": CPU_LIMIT_SECONDS,
        "address_space_limit_bytes": ADDRESS_SPACE_LIMIT_BYTES,
        "memory_monitor_interval_seconds": MONITOR_INTERVAL_SECONDS,
        "memory_monitor_interval_minimum_seconds": MONITOR_INTERVAL_MINIMUM_SECONDS,
        "memory_monitor_interval_maximum_seconds": MONITOR_INTERVAL_MAXIMUM_SECONDS,
    }
    for key, value in expected.items():
        if resources.get(key) != value:
            raise GateError(f"resource contract drift at {key}")
    if not (
        10
        <= resources["memory_monitor_interval_seconds"]
        <= 30
        and resources["memory_monitor_interval_minimum_seconds"] == 10
        and resources["memory_monitor_interval_maximum_seconds"] == 30
    ):
        raise GateError("memory monitor interval is outside frozen 10..30 seconds")

    argv = tuple(policy["build_contract"]["make_argv"])
    if argv != MAKE_ARGV:
        raise GateError("Make argv does not equal the literal gate constant")
    if argv.count(GENCODE_ARG) != 1 or argv.count("-j1") != 1:
        raise GateError("Make argv lacks unique GENCODE or literal -j1")
    joined = "\n".join(argv)
    for forbidden in (
        "-j2",
        "-j4",
        "-use_fast_math",
        "-ffast-math",
        "-march=native",
        "--allow-unsupported-compiler",
        "sm_61",
        "sm_70",
        "sm_86",
        "compute_61",
        "compute_70",
        "compute_86",
    ):
        if forbidden in joined:
            raise GateError(f"Make argv contains forbidden token: {forbidden}")
    required_assignments = {
        "USE_FAST_MATH=NO",
        "USE_NATIVE_CPU_OPTIMIZATIONS=NO",
        "COMPILE_CHRONO=NO",
        "COMPILE_WAVEGEN=NO",
        "COMPILE_MOORDYNPLUS=NO",
        "LIBS_DIRECTORIES=",
    }
    if not required_assignments.issubset(argv):
        raise GateError("Make argv lacks disabled optional-feature assignments")
    if policy["toolchain"]["gencode_argv"] != GENCODE_ARG:
        raise GateError("policy GENCODE drift")
    return {"resources": expected, "make_argv": list(argv)}


def verify_wrapper_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    wrapper = policy["wrapper_contract"]
    if WRAPPER_BYTES.decode("utf-8") != wrapper["content_utf8"]:
        raise GateError("wrapper bytes drift")
    if len(WRAPPER_BYTES) != 84 or wrapper["size_bytes"] != 84:
        raise GateError("wrapper size drift")
    if sha256_bytes(WRAPPER_BYTES) != WRAPPER_SHA256 or wrapper["sha256"] != WRAPPER_SHA256:
        raise GateError("wrapper SHA-256 drift")
    if wrapper["mode_octal"] != "0600" or wrapper["create_flags"] != [
        "O_WRONLY",
        "O_CREAT",
        "O_EXCL",
        "O_NOFOLLOW",
    ]:
        raise GateError("wrapper create-new/mode contract drift")
    if wrapper["attempt_a_ccflags_only"] is not True or wrapper["nccflags_preinclude"] is not False:
        raise GateError("Attempt A wrapper contains an NCCFLAGS fallback")
    return {"size_bytes": 84, "sha256": WRAPPER_SHA256, "mode_octal": "0600"}


def verify_tool_identities(policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    verified: list[dict[str, Any]] = []
    for item in policy["toolchain"]["tools"]:
        path = Path(item["path"])
        try:
            info = path.stat()
        except OSError as exc:
            raise GateError(f"frozen tool unavailable: {path}: {exc}") from exc
        observed = {
            "path": item["path"],
            "realpath": str(path.resolve()),
            "mode_octal": format(stat.S_IMODE(info.st_mode), "04o"),
            "size_bytes": info.st_size,
            "sha256": sha256_file(path),
        }
        for key, value in observed.items():
            if value != item[key]:
                raise GateError(f"frozen tool identity drift at {path}: {key}")
        verified.append(observed)
    return verified


def validate_source_copy_trace(
    trace: Sequence[Mapping[str, Any]], policy: Mapping[str, Any] | None = None
) -> None:
    contract = (policy or read_json_object(POLICY_PATH))["source_copy_contract"]
    if not isinstance(trace, Sequence) or len(trace) != len(SOURCE_COPY_STATES):
        raise GateError("source-copy trace must contain exactly seven events")
    timestamps: list[int] = []
    for index, (event, expected_state) in enumerate(
        zip(trace, SOURCE_COPY_STATES, strict=True), start=1
    ):
        require_exact_keys(event, EVENT_KEYS, f"source-copy event {index}")
        if event["state"] != expected_state or event["sequence"] != index:
            raise GateError(f"source-copy state order drift at sequence {index}")
        if not isinstance(event["captured_at_ns"], int):
            raise GateError("source-copy timestamp must be integer nanoseconds")
        timestamps.append(event["captured_at_ns"])
        evidence = event["evidence"]
        if not isinstance(evidence, Mapping):
            raise GateError("source-copy evidence must be an object")
        require_exact_keys(
            evidence, STATE_EVIDENCE_KEYS[expected_state], f"{expected_state} evidence"
        )
    if timestamps != sorted(set(timestamps)):
        raise GateError("source-copy event timestamps are not strictly increasing")

    e = [item["evidence"] for item in trace]
    if e[0] != {"create_new": True, "path": contract["start_record"]}:
        raise GateError("start record is not create-new at the exact path")
    if e[1] != {"isolated": True, "copied_entry_count": 352}:
        raise GateError("isolated source copy evidence drift")
    if e[2] != {
        "entry_count": 352,
        "extra_count": 0,
        "symlink_count": 0,
        "hardlink_count": 0,
        "elf_count": 0,
        "executable_count": 0,
    }:
        raise GateError("sealed 352-entry inventory evidence drift")
    if e[3]["unloaded"] is not True or e[3]["zero_residue"] is not True:
        raise GateError("copy profile was not unloaded before wrapper creation")
    if e[3]["profile_name"] != (policy or read_json_object(POLICY_PATH))["profiles"]["attempt_a_copy"]["name"]:
        raise GateError("copy profile identity drift in state trace")
    if e[4] != {
        "create_flags": ["O_WRONLY", "O_CREAT", "O_EXCL", "O_NOFOLLOW"],
        "mode_octal": "0600",
        "sha256": WRAPPER_SHA256,
        "size_bytes": 84,
        "after_unload_sequence": 4,
    }:
        raise GateError("wrapper publication order/bytes/mode drift")
    if e[5] != {
        "entry_count": 353,
        "sealed_entry_count": 352,
        "wrapper_entry_count": 1,
        "extra_count": 0,
        "symlink_count": 0,
        "hardlink_count": 0,
        "elf_count": 0,
        "executable_count": 0,
        "wrapper_sha256": WRAPPER_SHA256,
        "wrapper_mode_octal": "0600",
    }:
        raise GateError("post-wrapper complete 353-entry inventory evidence drift")
    if e[6] != {
        "create_new": True,
        "path": contract["final_receipt"],
        "published": True,
        "after_complete_inventory_sequence": 6,
    }:
        raise GateError("final receipt was published before complete inventory")


def validate_build_input_inventory(
    entries: Sequence[Mapping[str, Any]], sealed_paths: Iterable[str]
) -> None:
    sealed = set(sealed_paths)
    wrapper_path = "U3GpuBuild.mk"
    if len(sealed) != 352 or wrapper_path in sealed:
        raise GateError("sealed path fixture must contain exactly 352 non-wrapper paths")
    if len(entries) != 353:
        raise GateError("complete build input must contain exactly 353 entries")
    required_keys = {
        "path",
        "kind",
        "symlink",
        "nlink",
        "mode_octal",
        "size_bytes",
        "sha256",
        "is_elf",
    }
    by_path: dict[str, Mapping[str, Any]] = {}
    for entry in entries:
        require_exact_keys(entry, required_keys, "build input entry")
        path = entry["path"]
        if not isinstance(path, str) or path in by_path:
            raise GateError("duplicate or non-string build input path")
        by_path[path] = entry
        if (
            entry["kind"] != "regular"
            or entry["symlink"] is not False
            or entry["nlink"] != 1
        ):
            raise GateError(f"unsafe file type/symlink/hardlink: {path}")
        mode = entry["mode_octal"]
        if not isinstance(mode, str) or re.fullmatch(r"0[0-7]{3}", mode) is None:
            raise GateError(f"invalid file mode: {path}")
        if int(mode, 8) & 0o111:
            raise GateError(f"execute bit is forbidden in build input: {path}")
        if entry["is_elf"] is not False:
            raise GateError(f"ELF input is forbidden before build: {path}")
        if not isinstance(entry["size_bytes"], int) or entry["size_bytes"] < 0:
            raise GateError(f"invalid input size: {path}")
        require_hex64(entry["sha256"], f"build input {path} sha256")
    if set(by_path) != sealed | {wrapper_path}:
        raise GateError("build input has missing or extra paths")
    wrapper = by_path[wrapper_path]
    if (
        wrapper["mode_octal"] != "0600"
        or wrapper["size_bytes"] != 84
        or wrapper["sha256"] != WRAPPER_SHA256
    ):
        raise GateError("wrapper mode/size/hash drift in complete inventory")


def _policy(policy: Mapping[str, Any] | None = None) -> Mapping[str, Any]:
    return policy if policy is not None else read_json_object(POLICY_PATH)


def allowed_static_input_mapping(
    policy: Mapping[str, Any] | None = None,
) -> dict[str, tuple[str, str, bool]]:
    current = _policy(policy)
    audit = current["static_audit_contract"]
    mapping: dict[str, tuple[str, str, bool]] = {}
    for item in audit["candidate_inputs"]:
        mapping[item["host_path"]] = (item["guest_path"], "candidate", False)
    cuda = set(current["object_contract"]["cuda_object_names"])
    for build_id in audit["object_input_contract"]["build_ids"]:
        root = f"/home/zrj/scout_liquid_lab/build/{build_id}.partial"
        for name in audit["object_input_contract"]["object_names"]:
            host = f"{root}/output/buildtree/src/source/{name}"
            mapping[host] = (f"/audit/input/{name}", "object", name in cuda)
    if len(mapping) != 2 + 2 * 131:
        raise GateError("static-audit exact input mapping cardinality drift")
    return mapping


def _suffix_for_input(
    suffix_id: str,
    guest_path: str,
    kind: str,
    cuda_object: bool,
    policy: Mapping[str, Any],
) -> tuple[str, list[str]]:
    audit = policy["static_audit_contract"]
    if kind == "candidate":
        catalog = audit["candidate_tool_suffixes"]
    else:
        catalog = audit["object_tool_suffix_templates"]
    matches = [item for item in catalog if item["id"] == suffix_id]
    if len(matches) != 1:
        raise GateError(f"tool suffix is not uniquely frozen: {suffix_id}")
    item = matches[0]
    if item.get("cuda_only", False) and not cuda_object:
        raise GateError("CUDA-only suffix requested for a C++ object")
    if kind == "candidate" and "<" in "\n".join(item["argv"]):
        raise GateError("candidate suffix contains unresolved placeholder")
    argv = [
        token.replace("/audit/input/<EXACT_OBJECT_NAME>", guest_path)
        .replace("/audit/input/<EXACT_CUDA_OBJECT_NAME>", guest_path)
        for token in item["argv"]
    ]
    if any("<" in token or ">" in token for token in argv):
        raise GateError("static-audit suffix left an unresolved token")
    return item["resource_class"], argv


def build_static_audit_argv(
    host_input: str,
    suffix_id: str,
    policy: Mapping[str, Any] | None = None,
) -> list[str]:
    current = dict(_policy(policy))
    audit = current["static_audit_contract"]
    mapping = allowed_static_input_mapping(current)
    if host_input not in mapping:
        raise GateError("static-audit host input is not exact-allowlisted")
    guest, kind, cuda_object = mapping[host_input]
    resource_name, tool_argv = _suffix_for_input(
        suffix_id, guest, kind, cuda_object, current
    )
    resource = audit["resource_classes"][resource_name]
    argv = [
        "/usr/bin/timeout",
        "--foreground",
        "--kill-after=5s",
        f"{resource['wall_seconds']}s",
        *audit["aa_exec_prefix"],
        *audit["bwrap_prefix_before_exact_input"],
        "--ro-bind",
        host_input,
        guest,
        *audit["bwrap_suffix_after_exact_input"],
        "/usr/bin/prlimit",
        f"--cpu={resource['cpu_seconds']}",
        f"--as={resource['address_space_bytes']}",
        "--nproc=64",
        "--nofile=128",
        "--core=0",
        "--",
        *tool_argv,
    ]
    validate_static_audit_argv(argv, current, _skip_rebuild=True)
    return argv


def validate_static_audit_argv(
    argv: Sequence[str],
    policy: Mapping[str, Any] | None = None,
    _skip_rebuild: bool = False,
) -> dict[str, str]:
    current = dict(_policy(policy))
    required_flags = {
        "--die-with-parent",
        "--new-session",
        "--clearenv",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--disable-userns",
        "--assert-userns-disabled",
        "--kill-after=5s",
    }
    if not required_flags.issubset(argv):
        raise GateError("static-audit argv lacks a mandatory isolation/timeout flag")
    if "--bind" in argv or "--proc" in argv or "--dev" in argv:
        raise GateError("static-audit argv exposes a writable bind, proc, or dev")
    ro_pairs = [
        list(argv[index + 1 : index + 3])
        for index, token in enumerate(argv[:-2])
        if token == "--ro-bind"
    ]
    if len(ro_pairs) != 3:
        raise GateError("static-audit must have exactly three read-only binds")
    if ro_pairs[:2] != [["/usr", "/usr"], ["/etc/magic", "/etc/magic"]]:
        raise GateError("static-audit system/data bind prefix drift")
    host_input, guest_input = ro_pairs[2]
    mapping = allowed_static_input_mapping(current)
    if host_input not in mapping or mapping[host_input][0] != guest_input:
        raise GateError("static-audit exact input bind mapping drift")
    if _skip_rebuild:
        return {"host_input": host_input, "guest_input": guest_input}

    _, kind, cuda_object = mapping[host_input]
    catalog = (
        current["static_audit_contract"]["candidate_tool_suffixes"]
        if kind == "candidate"
        else current["static_audit_contract"]["object_tool_suffix_templates"]
    )
    for item in catalog:
        if item.get("cuda_only", False) and not cuda_object:
            continue
        expected = build_static_audit_argv(host_input, item["id"], current)
        if list(argv) == expected:
            return {
                "host_input": host_input,
                "guest_input": guest_input,
                "suffix_id": item["id"],
            }
    raise GateError("static-audit argv does not match any exact frozen tool suffix")


def validate_stream_summary(
    output_bytes: int, output_sha256: str, bounded_prefix: bytes
) -> None:
    if not isinstance(output_bytes, int) or not (0 <= output_bytes <= STATIC_OUTPUT_LIMIT_BYTES):
        raise GateError("static-audit output exceeds the frozen 256 MiB limit")
    require_hex64(output_sha256, "static-audit output sha256")
    if len(bounded_prefix) > STREAM_PREFIX_LIMIT_BYTES:
        raise GateError("static-audit diagnostic prefix exceeds 64 KiB")


def validate_candidate_identity_unchanged(
    before: Mapping[str, Any],
    after: Mapping[str, Any],
    policy: Mapping[str, Any] | None = None,
) -> None:
    fields = _policy(policy)["static_audit_contract"]["candidate_identity_fields"]
    require_exact_keys(before, fields, "candidate identity before")
    require_exact_keys(after, fields, "candidate identity after")
    if dict(before) != dict(after):
        raise GateError("candidate identity changed across static audit")
    if (
        before["regular"] is not True
        or before["symlink"] is not False
        or before["nlink"] != 1
        or before["mode_octal"] != "0400"
    ):
        raise GateError("candidate identity is not regular/non-symlink/nlink1/mode0400")
    require_hex64(before["sha256"], "candidate identity sha256")


def expected_b_make_argv(
    selected_delta_id: str, policy: Mapping[str, Any] | None = None
) -> list[str]:
    current = _policy(policy)
    catalog = {
        item["id"]: item for item in current["b_admission_contract"]["delta_catalog"]
    }
    if selected_delta_id not in catalog:
        raise GateError("unknown B delta")
    delta = catalog[selected_delta_id]
    if delta["admissible_for_this_campaign"] is not True:
        raise GateError("selected B delta is not admissible for this campaign")
    argv = list(current["build_contract"]["make_argv"])
    replacement = delta.get("make_replacement")
    if replacement is not None:
        if argv.count(replacement["from"]) != 1:
            raise GateError("B compiler replacement source is not unique")
        argv[argv.index(replacement["from"])] = replacement["to"]
    return argv


def validate_gxx13_helper_identities(
    observed: Mapping[str, str], policy: Mapping[str, Any] | None = None
) -> None:
    expected = {
        item["path"]: item["sha256"]
        for item in _policy(policy)["b_admission_contract"]["gxx13_exact_tools"]
    }
    if dict(observed) != expected:
        raise GateError("g++-13/helper identity or SHA-256 drift")


def _verify_b_profile_file(
    profile: Mapping[str, Any],
    planned: Mapping[str, Any],
    root_b: str,
    root_a: str,
    selected_delta: Mapping[str, Any],
    copy_phase: bool,
) -> None:
    require_exact_keys(profile, {"name", "path", "sha256", "attempt_root"}, "B profile")
    if (
        profile["name"] != planned["name"]
        or profile["path"] != planned["planned_path"]
        or profile["attempt_root"] != root_b
    ):
        raise GateError("B profile identity/path/root drift")
    require_hex64(profile["sha256"], "B profile sha256")
    path = repo_path(profile["path"])
    text = path.read_text(encoding="utf-8")
    if sha256_file(path) != profile["sha256"]:
        raise GateError("B profile file hash drift")
    validate_exact_root_profile_text(text, root_b, root_a, "B profile")
    gxx13_paths = {
        item["path"]
        for item in read_json_object(POLICY_PATH)["b_admission_contract"][
            "gxx13_exact_tools"
        ]
    }
    if copy_phase and any(path_item in text for path_item in gxx13_paths):
        raise GateError("B-copy contains compiler permission")
    if not copy_phase:
        has_all = all(path_item in text for path_item in gxx13_paths)
        if selected_delta["gxx13_permission"] is True and not has_all:
            raise GateError("compiler B-build lacks all exact g++-13 helpers")
        if selected_delta["gxx13_permission"] is False and any(
            path_item in text for path_item in gxx13_paths
        ):
            raise GateError("non-compiler B-build contains g++-13 permission")


def validate_b_admission(
    document: Mapping[str, Any],
    *,
    verify_parent_files: bool = True,
    policy: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    current = dict(_policy(policy))
    contract = current["b_admission_contract"]
    require_exact_keys(document, contract["manifest_required_keys"], "B admission")
    if document["schema_version"] != contract["manifest_schema_version"]:
        raise GateError("B admission schema version drift")
    if document["document_type"] != contract["manifest_document_type"]:
        raise GateError("B admission document type drift")
    if (
        document["campaign_id"] != CAMPAIGN_ID
        or document["build_id"] != BUILD_ID_B
        or document["attempt_root"] != str(ROOT_B)
    ):
        raise GateError("B admission identity/root drift")
    require_hex64(document["root_cause_evidence_sha256"], "B root-cause evidence")

    catalog = {item["id"]: item for item in contract["delta_catalog"]}
    selected_id = document["selected_delta_id"]
    if selected_id not in catalog or catalog[selected_id]["admissible_for_this_campaign"] is not True:
        raise GateError("B admission selected an unknown or inadmissible delta")
    selected = catalog[selected_id]
    if document["make_argv"] != expected_b_make_argv(selected_id, current):
        raise GateError("B Make argv is not the exact single-delta derivation")
    expected_wrapper = contract["wrapper_variants"][selected["wrapper_variant"]]["sha256"]
    if document["wrapper_sha256"] != expected_wrapper:
        raise GateError("B wrapper variant SHA-256 drift")

    parent_hashes = document["parent_hashes"]
    if not isinstance(parent_hashes, Mapping):
        raise GateError("B parent_hashes must be an object")
    require_exact_keys(parent_hashes, contract["parent_file_paths"], "B parent_hashes")
    for path, digest in parent_hashes.items():
        require_hex64(digest, f"B parent hash {path}")
        if verify_parent_files and sha256_file(repo_path(path)) != digest:
            raise GateError(f"B parent file drift: {path}")

    permission = document["permission_delta"]
    require_exact_keys(
        permission, {"execute_tools", "read_paths", "apparmor_rules"}, "B permission delta"
    )
    for field in permission:
        if not isinstance(permission[field], list):
            raise GateError(f"B permission delta {field} must be a list")
    gxx13_paths = [item["path"] for item in contract["gxx13_exact_tools"]]
    serialized_permission = json.dumps(permission, sort_keys=True)
    if selected["gxx13_permission"] is True:
        if permission["execute_tools"] != gxx13_paths:
            raise GateError("compiler delta lacks exact g++-13 execute set")
        if permission["read_paths"] != [selected["gcc13_read_root"]]:
            raise GateError("compiler delta GCC 13 read root drift")
        if permission["apparmor_rules"]:
            raise GateError("compiler delta contains unrelated AppArmor rules")
    else:
        if any(path in serialized_permission for path in gxx13_paths):
            raise GateError("non-compiler B delta contains g++-13 permission")
        if permission["execute_tools"]:
            raise GateError("non-compiler B delta contains executable permission")
        expected_reads = (
            [selected["only_read_path"]]
            if selected_id == "cudart_static_read_visibility"
            else []
        )
        if permission["read_paths"] != expected_reads:
            raise GateError("non-compiler B read-path delta drift")
        if selected_id == "apparmor_exact_evidence_permission":
            if len(permission["apparmor_rules"]) != 1:
                raise GateError("AppArmor evidence delta must contain exactly one rule")
            rule = permission["apparmor_rules"][0]
            if not isinstance(rule, Mapping):
                raise GateError("AppArmor evidence rule must be a closed object")
            require_exact_keys(rule, {"path", "access"}, "AppArmor evidence rule")
            path_value = rule["path"]
            access = rule["access"]
            allowed_access = selected["allowed_access_kinds"]
            if (
                not isinstance(path_value, str)
                or not path_value.startswith("/")
                or any(token in path_value for token in ("*", "?", "[", "]", "{", "}"))
                or any(
                    prefix in path_value
                    for prefix in contract["forbidden_permission_prefixes"]
                )
                or not any(
                    path_value.startswith(prefix)
                    for prefix in (
                        "/usr/",
                        "/lib/",
                        "/lib64/",
                        "/work/tmp/",
                        "/newroot/work/tmp/",
                        str(ROOT_B) + "/output/",
                    )
                )
                or access not in allowed_access
            ):
                raise GateError("AppArmor evidence rule crosses a forbidden boundary")
            if access == "rw_guest_tmp_only" and not path_value.startswith(
                ("/work/tmp/", "/newroot/work/tmp/")
            ):
                raise GateError("AppArmor write delta is not guest-tmp-only")
        elif permission["apparmor_rules"]:
            raise GateError("B delta contains unrelated AppArmor permission")

    planned_copy = current["profiles"]["attempt_b_copy"]
    planned_build = current["profiles"]["attempt_b_build"]
    if verify_parent_files:
        if os.path.lexists(ROOT_B):
            raise GateError("B root exists before B-specific admission")
        _verify_b_profile_file(
            document["copy_profile"], planned_copy, str(ROOT_B), str(ROOT_A), selected, True
        )
        _verify_b_profile_file(
            document["build_profile"], planned_build, str(ROOT_B), str(ROOT_A), selected, False
        )
    else:
        for profile, planned in (
            (document["copy_profile"], planned_copy),
            (document["build_profile"], planned_build),
        ):
            require_exact_keys(
                profile, {"name", "path", "sha256", "attempt_root"}, "B profile"
            )
            if (
                profile["name"] != planned["name"]
                or profile["path"] != planned["planned_path"]
                or profile["attempt_root"] != str(ROOT_B)
            ):
                raise GateError("B profile mock identity/path/root drift")
            require_hex64(profile["sha256"], "B profile mock sha256")

    if not isinstance(document["created_at_utc"], str) or UTC_RE.fullmatch(document["created_at_utc"]) is None:
        raise GateError("B admission timestamp is not exact UTC Z form")
    if document["status"] != contract["manifest_status"]:
        raise GateError("B admission status drift")
    if document["next_allowed_stage"] != contract["manifest_next_allowed_stage"]:
        raise GateError("B admission next stage drift")
    return {"selected_delta_id": selected_id, "build_id": BUILD_ID_B}


def verify_b_not_materialized(policy: Mapping[str, Any]) -> dict[str, bool]:
    if os.path.lexists(ROOT_A) or os.path.lexists(ROOT_B):
        raise GateError("G1 must not create A or B attempt roots")
    for key in ("attempt_b_copy", "attempt_b_build"):
        planned = repo_path(policy["profiles"][key]["planned_path"])
        if os.path.lexists(planned):
            raise GateError(f"G1 materialized forbidden {key} profile")
    campaign_b = policy["campaign"]["attempt_b"]
    if any(
        campaign_b[key] is not False
        for key in (
            "root_created",
            "copy_profile_materialized",
            "build_profile_materialized",
            "receipt_materialized",
        )
    ):
        raise GateError("policy falsely marks B materialized")
    return {
        "a_root_exists": False,
        "b_root_exists": False,
        "b_copy_profile_exists": False,
        "b_build_profile_exists": False,
    }


def verify_static_audit_contract(policy: Mapping[str, Any]) -> dict[str, Any]:
    audit = policy["static_audit_contract"]
    if audit["output_limit_bytes"] != STATIC_OUTPUT_LIMIT_BYTES:
        raise GateError("static-audit output limit drift")
    invariants = audit["sandbox_invariants"]
    expected = {
        "host_writable_bind_count": 0,
        "network_namespace": "new_empty",
        "proc_exposed": False,
        "dev_exposed": False,
        "gpu_exposed": False,
        "candidate_executable": False,
        "home": "/nonexistent",
        "shell_allowed": False,
        "compiler_allowed": False,
        "make_allowed": False,
        "ldd_allowed": False,
    }
    if invariants != expected:
        raise GateError("static-audit sandbox invariant drift")
    mapping = allowed_static_input_mapping(policy)
    samples = [
        (audit["candidate_inputs"][0]["host_path"], "file"),
        (
            f"{ROOT_A}/output/buildtree/src/source/{policy['object_contract']['cpp_object_names'][0]}",
            "object_readelf_header",
        ),
        (
            f"{ROOT_B}/output/buildtree/src/source/{policy['object_contract']['cuda_object_names'][0]}",
            "cuda_object_list_elf",
        ),
    ]
    for host, suffix_id in samples:
        validate_static_audit_argv(build_static_audit_argv(host, suffix_id, policy), policy)
    return {
        "exact_input_count": len(mapping),
        "candidate_suffix_count": len(audit["candidate_tool_suffixes"]),
        "object_suffix_template_count": len(audit["object_tool_suffix_templates"]),
        "output_limit_bytes": STATIC_OUTPUT_LIMIT_BYTES,
    }


def self_check() -> dict[str, Any]:
    policy = validate_policy_schema()
    if policy["status"] != "A_CONTRACT_AND_PROFILES_FROZEN":
        raise GateError("policy state is not G1 frozen")
    if policy["gpu_build_status"] != "NOT_RUN":
        raise GateError("policy falsely claims a GPU build")
    if policy["g0_input"]["parallel_jobs"] != PARALLEL_JOBS:
        raise GateError("G0 parallel_jobs drift")
    result = {
        "status": "A_CONTRACT_AND_PROFILES_FROZEN",
        "plan_identity": verify_plan_identity(),
        "frozen_references": verify_frozen_references(policy),
        "source_inputs": verify_source_inputs(policy),
        "object_contract": verify_object_contract(policy),
        "wrapper_contract": verify_wrapper_contract(policy),
        "resources_and_make": verify_resources_and_make_argv(policy),
        "tool_identity_count": len(verify_tool_identities(policy)),
        "profiles": verify_profiles(policy),
        "static_audit": verify_static_audit_contract(policy),
        "non_materialization": verify_b_not_materialized(policy),
        "system_actions_performed": False,
        "gpu_build_started": False,
        "profile_loaded": False,
        "candidate_executed": False,
    }
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command", choices=("self-check", "validate-b-admission")
    )
    parser.add_argument("manifest", nargs="?")
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            if args.manifest is not None:
                raise GateError("self-check takes no manifest")
            result = self_check()
        else:
            if args.manifest is None:
                raise GateError("validate-b-admission requires one manifest path")
            result = validate_b_admission(read_json_object(Path(args.manifest)))
    except (GateError, OSError, UnicodeError, ValueError, KeyError, TypeError) as exc:
        print(
            json.dumps(
                {"status": "FAIL_GPU_BUILD_ADMISSION", "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
