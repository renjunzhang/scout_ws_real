#!/usr/bin/env python3
"""Static-only validator for the MSI source-only CPU-build policy.

This module deliberately has no checkout, materialization, namespace, build,
receipt-writing, or generic-command entry point.  ``self-check`` reads only
the checked-in policy and schema, then reports whether their frozen contract
is internally consistent.  A passing report is *not* an execution admission.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
POLICY_PATH = (
    PACKAGE_DIR
    / "config/target_hosts/liquid_zrj_msi_u2404_source_cpu_build_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_source_cpu_build_policy_v1.json"

EXPECTED_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
EXPECTED_TREE = "cef458cb358712f4694b9d2148f638440418e9dc"
EXPECTED_URL = "https://github.com/DualSPHysics/DualSPHysics.git"
EXPECTED_BUILD_ID_RE = r"^u3_source_cpu_build_[0-9]{8}T[0-9]{6}Z$"


class SourceCpuBuildPolicyError(RuntimeError):
    """Any policy mismatch remains a hard no-go for future execution."""


def canonical_hash(value: Any) -> str:
    """Return the collaboration hash for JSON content, independent of layout."""

    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise SourceCpuBuildPolicyError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SourceCpuBuildPolicyError(f"JSON file is not an object: {path}")
    return value


def _expect_exact(
    errors: list[str], policy: Mapping[str, Any], key: str, expected: Any, label: str
) -> None:
    if policy.get(key) != expected:
        errors.append(label)


def _expected_evidence() -> dict[str, Any]:
    return {
        "target_profile": {
            "profile_id": "LIQUID_ZRJ_MSI_U2404_P0_V1",
            "canonical_sha256": "fec0b980ca46df04d877290b73b3f3406af97887ccc20c9d52d58092c36f8838",
            "required_status": "PASS_PROFILE_READ_ONLY_ONLY",
        },
        "full_source_fetch": {
            "receipt_relative_path": "audits/u2_full_source_fetch_attempt_3_20260806T074600Z.json",
            "file_sha256": "23745c0076c2c06fc549665c89f4e039761832bab66f4f568f63eff696ead18d",
            "receipt_sha256": "90bd0c1cc03b022dc42dd4b0368a84f46af10b9b5b8c54108765d1fd16df0426",
            "status": "PASS_FULL_BARE_SOURCE_FETCH",
        },
        "static_inventory": {
            "receipt_relative_path": "audits/u2_full_source_static_inventory_v1_20260806T080500Z.json",
            "file_sha256": "a3be7e2a7a5bbeae8483706dbd4a135c7af80513bd5fbd1f982122b4380efb28",
            "receipt_sha256": "5eac669287c6711de0835409ed9f3ec9dc6829a3b03cef24cb6838ebb07e32ff",
            "status": "PASS_OFFLINE_FULL_SOURCE_STATIC_INVENTORY",
        },
        "elf_metadata": {
            "receipt_relative_path": "audits/u2_elf_metadata_v1_20260806T082300Z.json",
            "file_sha256": "47e411b391afd677fc4d1b60ca6547c73545c287ef080bea92c2e66128ef6a74",
            "receipt_sha256": "7686d2df11332dc4c5809f64985959ed8e05fd96b3b44c5120e516bbdd850024",
            "status": "PASS_OFFLINE_ELF_METADATA",
            "required_next_allowed_stage": "HUMAN_REVIEW_ELF_METADATA_NO_EXECUTION",
        },
    }


def _expected_source() -> dict[str, Any]:
    return {
        "repository_url": EXPECTED_URL,
        "commit": EXPECTED_COMMIT,
        "tree": EXPECTED_TREE,
        "full_bare_relative_path": (
            "dependency/source/"
            "DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
        ),
        "future_checkout_root_relative_template": "build/<build_id>.partial/input_checkout",
        "source_readonly_guest_path": "/work/input",
        "source_mount_mode": "read_only",
        "source_materialization_requires_separate_admission": True,
        "materialized_input_allowlist": ["src/source/**"],
        "materialized_input_forbidden_prefixes": [
            ".git/",
            "bin/",
            "src/lib/",
            "examples/",
            "src_mphase/",
        ],
        "no_network": True,
        "no_submodules": True,
        "no_lfs_smudge": True,
        "no_hooks": True,
        "precompiled_elf_execution": "forbidden",
        "precompiled_elf_mount_to_guest": "forbidden",
    }


def _expected_build_layout() -> dict[str, Any]:
    return {
        "build_id_regex": EXPECTED_BUILD_ID_RE,
        "build_root_relative_template": "build/<build_id>.partial",
        "output_root_relative_template": "build/<build_id>.partial/output",
        "build_tree_guest_path": "/work/output/buildtree/src/source",
        "artifact_guest_path": "/work/output/artifacts/DualSPHysics5.4CPU_linux64",
        "new_paths_only": True,
        "no_hardlinks": True,
        "no_symlinks": True,
        "input_to_output_copy_required": True,
        "copy_include": ["src/source/**"],
        "copy_exclude": [".git/**", "bin/**", "src/lib/**", "examples/**", "src_mphase/**"],
        "workspace_as_input": False,
        "host_writable_bind_count": 1,
    }


def _expected_tools() -> dict[str, Any]:
    return {
        "bwrap": {
            "path": "/usr/bin/bwrap",
            "sha256": "52231e1caf55bcbc667b269f49c63599a6f7db4767ae6a039580d0ff853db712",
        },
        "timeout": {
            "path": "/usr/bin/timeout",
            "sha256": "4fccd5b0192653a2446b745d5385ea547b78e466150e07ade9e2caff2b7f4e08",
        },
        "env": {
            "path": "/usr/bin/env",
            "sha256": "0aefff8f912fb75716c5d4de3b6acde93edbe8fa280fc8ee895c1226d3e373ef",
        },
        "prlimit": {
            "path": "/usr/bin/prlimit",
            "sha256": "f27cfd8c1512a4cc6541b59b80cb4cdfd6ef28c34aa21db4299b48264cd0d128",
        },
        "make": {
            "path": "/usr/bin/make",
            "sha256": "d78b8f1d099fbcfb6f2f49ab87223b9b68fb3956642f92d6ec6de812e8afa965",
        },
        "gpp": {
            "path": "/usr/bin/x86_64-linux-gnu-g++-13",
            "sha256": "1353e9bdd29a7295c7226bf6c63abccce056d8cac31f112e5cdbecc3f28c2769",
        },
        "cmake": {
            "path": "/usr/bin/cmake",
            "sha256": "1c5227af4edd22d8d689def545e18ee458260c0fd579eba2187967f38817e638",
        },
    }


def _expected_bwrap_argv() -> list[str]:
    return [
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
        "--hostname",
        "r8-liquid-cpu-build",
        "--clearenv",
        "--setenv",
        "HOME",
        "/nonexistent",
        "--setenv",
        "PATH",
        "/usr/bin:/bin",
        "--setenv",
        "LC_ALL",
        "C.UTF-8",
        "--setenv",
        "LANG",
        "C.UTF-8",
        "--setenv",
        "TZ",
        "UTC",
        "--uid",
        "0",
        "--gid",
        "0",
        "--ro-bind",
        "/usr",
        "/usr",
        "--symlink",
        "usr/bin",
        "/bin",
        "--symlink",
        "usr/lib",
        "/lib",
        "--symlink",
        "usr/lib64",
        "/lib64",
        "--proc",
        "/proc",
        "--dev",
        "/dev",
        "--tmpfs",
        "/tmp",
        "--dir",
        "/work",
        "--ro-bind",
        "<LIQUID_ROOT>/build/<build_id>.partial/input_checkout",
        "/work/input",
        "--bind",
        "<LIQUID_ROOT>/build/<build_id>.partial/output",
        "/work/output",
        "--chdir",
        "/work/output/buildtree/src/source",
        "--",
    ]


def _expected_sandbox() -> dict[str, Any]:
    return {
        "backend": "bubblewrap",
        "current_proof_status": "SYSTEM_TRUE_ONLY_NO_WRITABLE_HOST_BIND_PROOF",
        "network": "none",
        "gpu_device_nodes": [],
        "host_read_only_binds": [
            {"source": "/usr", "destination": "/usr"},
            {
                "source": "<LIQUID_ROOT>/build/<build_id>.partial/input_checkout",
                "destination": "/work/input",
            },
        ],
        "host_writable_binds": [
            {
                "source": "<LIQUID_ROOT>/build/<build_id>.partial/output",
                "destination": "/work/output",
                "must_be_new": True,
            }
        ],
        "forbidden_host_path_prefixes": [
            "/home/zrj/scout_ws",
            "/home/zrj/ros2_ws",
            "/opt/ros/jazzy",
            "/data",
            "/home/zrj/scout_liquid_lab/dependency/source",
        ],
        "guest_environment": {
            "HOME": "/nonexistent",
            "PATH": "/usr/bin:/bin",
            "LC_ALL": "C.UTF-8",
            "LANG": "C.UTF-8",
            "TZ": "UTC",
        },
        "bwrap_argv_template": _expected_bwrap_argv(),
        "environment_injection_forbidden": [
            "LD_LIBRARY_PATH",
            "LD_PRELOAD",
            "PYTHONPATH",
            "ROS_DISTRO",
            "AMENT_PREFIX_PATH",
            "COLCON_PREFIX_PATH",
            "GAZEBO_RESOURCE_PATH",
            "GAZEBO_PLUGIN_PATH",
        ],
        "output_execution": "forbidden",
        "writable_bind_smoke_requires_separate_admission": True,
    }


def _expected_resources() -> dict[str, Any]:
    return {
        "vcpu_count": 4,
        "minimum_available_memory_bytes": 8589934592,
        "address_space_limit_bytes": 8589934592,
        "disk_reservation_bytes": 21474836480,
        "wall_timeout_seconds": 3600,
        "kill_after_seconds": 5,
        "cpu_time_seconds": 3600,
        "process_limit": 128,
        "open_file_limit": 1024,
        "file_size_limit_bytes": 21474836480,
        "core_dump_bytes": 0,
        "resource_enforcement_requires_separate_admission": True,
        "parent_timeout_argv_prefix": [
            "/usr/bin/timeout",
            "--foreground",
            "--kill-after=5s",
            "3600s",
        ],
        "inside_sandbox_prlimit_argv_prefix": [
            "/usr/bin/env",
            "-i",
            "HOME=/nonexistent",
            "PATH=/usr/bin:/bin",
            "LC_ALL=C.UTF-8",
            "LANG=C.UTF-8",
            "TZ=UTC",
            "/usr/bin/prlimit",
            "--as=8589934592:8589934592",
            "--cpu=3600:3605",
            "--nproc=128:128",
            "--nofile=1024:1024",
            "--fsize=21474836480:21474836480",
            "--core=0:0",
            "--",
        ],
    }


def _expected_toolchain() -> dict[str, Any]:
    return {
        "makefile_relative_path": "src/source/Makefile_cpu",
        "makefile_sha256": "4038165b761e233b207b7196f320dc59b3092737d5eaf8c98c9a282b9e900a91",
        "cmake": {"authorized": False, "path": "/usr/bin/cmake", "allowed_argv": []},
        "make_working_directory": "/work/output/buildtree/src/source",
        "make_argv": [
            "/usr/bin/make",
            "--no-builtin-rules",
            "--no-builtin-variables",
            "-f",
            "Makefile_cpu",
            "-j4",
            "CC=/usr/bin/x86_64-linux-gnu-g++-13",
            "USE_DEBUG=NO",
            "USE_FAST_MATH=NO",
            "USE_NATIVE_CPU_OPTIMIZATIONS=NO",
            "COMPILE_CHRONO=NO",
            "COMPILE_WAVEGEN=NO",
            "COMPILE_MOORDYNPLUS=NO",
            "LIBS_DIRECTORIES=",
            "EXECS_DIRECTORY=/work/output/artifacts",
            "/work/output/artifacts/DualSPHysics5.4CPU_linux64",
        ],
        "compiler_compile_argv_template": [
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "-c",
            "-O3",
            "-fopenmp",
            "-std=c++0x",
            "-D_WITHMR",
            "-DDISABLE_CHRONO",
            "-DDISABLE_WAVEGEN",
            "-DDISABLE_MOORDYNPLUS",
            "<source.cpp>",
        ],
        "compiler_link_argv_contract": {
            "compiler": "/usr/bin/x86_64-linux-gnu-g++-13",
            "required_tokens": [
                "<makefile_ordered_object_list>",
                "-fopenmp",
                "-lgomp",
                "-o",
                "/work/output/artifacts/DualSPHysics5.4CPU_linux64",
            ],
            "forbidden_tokens": [
                "-ffast-math",
                "-march=native",
                "-use_fast_math",
                "-ldsphchrono",
                "-lChronoEngine",
                "-ljwavegen_64",
                "-ldsphmoordynplus_64",
            ],
        },
        "explicitly_forbidden_make_targets": ["all", "clean", "compile_libs"],
        "direct_compiler_execution_authorized": False,
    }


def _expected_dynamic_library_policy() -> dict[str, Any]:
    return {
        "output_execution_authorized": False,
        "static_audit_requires_separate_admission": True,
        "required_interpreter": "/lib64/ld-linux-x86-64.so.2",
        "rpath_or_runpath": "forbidden",
        "allowed_dt_needed_sonames": [
            "libgomp.so.1",
            "libstdc++.so.6",
            "libm.so.6",
            "libgcc_s.so.1",
            "libpthread.so.0",
            "libc.so.6",
            "librt.so.1",
            "libdl.so.2",
        ],
        "required_dt_needed_sonames": [
            "libgomp.so.1",
            "libstdc++.so.6",
            "libgcc_s.so.1",
            "libc.so.6",
        ],
        "forbidden_soname_substrings": [
            "Chrono",
            "dsph",
            "cuda",
            "cudart",
            "nvidia",
            "ros",
            "gazebo",
            "jwavegen",
            "moordyn",
        ],
        "system_library_allowlist": [
            {
                "soname": "libgomp.so.1",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libgomp.so.1.0.0",
                "sha256": "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1",
            },
            {
                "soname": "libstdc++.so.6",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33",
                "sha256": "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11",
            },
            {
                "soname": "libm.so.6",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libm.so.6",
                "sha256": "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91",
            },
            {
                "soname": "libgcc_s.so.1",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libgcc_s.so.1",
                "sha256": "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb",
            },
            {
                "soname": "libpthread.so.0",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libpthread.so.0",
                "sha256": "a27ffa9bf233d61a5f02ddb0cf770dd6579021afc1aa8aec0fb58ee4a965281a",
            },
            {
                "soname": "libc.so.6",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libc.so.6",
                "sha256": "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161",
            },
            {
                "soname": "librt.so.1",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/librt.so.1",
                "sha256": "c6e6288545e24b0b3cfbf33320bda9236521625d8c3d628f3444f1ed40e5c7c5",
            },
            {
                "soname": "libdl.so.2",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/libdl.so.2",
                "sha256": "292d5f5af2e7360b3e18c56591a4960115373ecf40627660f9149b6c68a33f80",
            },
            {
                "soname": "ld-linux-x86-64.so.2",
                "resolved_path": "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2",
                "sha256": "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81",
            },
        ],
    }


def _expected_coordination() -> dict[str, Any]:
    return {
        "policy_exchange_allowed": [
            "Git commit or reviewed patch",
            "policy/schema/script/test SHA-256 manifest",
            "static self-check JSON/text report",
        ],
        "source_or_binary_transfer_to_gazebo_host": "forbidden",
        "precompiled_elf_transfer_for_execution": "forbidden",
        "build_artifact_transfer_for_execution": "forbidden",
        "future_motion_or_result_transfer_authorized": False,
        "secrets_or_credentials_in_handoff": "forbidden",
        "each_handoff_requires_manifest_and_sha256": True,
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate the entire frozen, non-executable strategy contract."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-source-cpu-build-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_CPU_BUILD_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_SOURCE_CPU_BUILD_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "policy_review_authorized": True,
        "source_materialization_authorized": False,
        "sandbox_execution_authorized": False,
        "build_execution_authorized": False,
        "output_execution_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_POLICY_REVIEWED_BUILD_NOT_AUTHORIZED_TO_RUN",
        "next_allowed_stage": (
            "HUMAN_REVIEW_HARMLESS_OUTPUT_BIND_SMOKE_AND_SOURCE_MATERIALIZATION_POLICY"
        ),
    }
    for key, expected in expected_top.items():
        _expect_exact(errors, policy, key, expected, f"policy field mismatch: {key}")

    sections = (
        ("required_evidence", _expected_evidence(), "required evidence differs from frozen receipts"),
        ("source", _expected_source(), "source-only input declaration differs"),
        ("build_layout", _expected_build_layout(), "build layout weakens the single-output-bind rule"),
        ("trusted_system_tools", _expected_tools(), "trusted tool paths or hashes differ"),
        ("sandbox", _expected_sandbox(), "sandbox bind, network, or environment policy differs"),
        ("resources", _expected_resources(), "resource limits or launcher argv differ"),
        ("toolchain", _expected_toolchain(), "CMake/Make/compiler argv policy differs"),
        (
            "dynamic_library_policy",
            _expected_dynamic_library_policy(),
            "dynamic-library allowlist or output-audit rule differs",
        ),
        ("cross_host_coordination", _expected_coordination(), "two-host handoff boundary differs"),
    )
    for key, expected, message in sections:
        _expect_exact(errors, policy, key, expected, message)

    expected_invariants = {
        "no_sudo",
        "no_apt",
        "no_network",
        "no_source_checkout_now",
        "no_source_materialization_now",
        "no_sandbox_now",
        "no_cmake",
        "no_make",
        "no_compiler",
        "no_upstream_code_execution",
        "no_precompiled_binary_execution",
        "no_dlopen",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_system_change",
        "no_destructive_cleanup",
        "no_workspace_mount",
        "no_output_execution",
        "no_receipt_that_claims_build",
    }
    invariants = policy.get("invariants")
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("source-only build safety invariants are incomplete")
    return errors


def validate_schema(schema: Mapping[str, Any]) -> list[str]:
    """Check only the schema's no-execution top-level contract."""

    errors: list[str] = []
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema draft differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        errors.append("schema must be a closed top-level object")
    required = schema.get("required")
    if not isinstance(required, list) or set(required) != {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "physical_primary_eligible",
        "policy_review_authorized",
        "source_materialization_authorized",
        "sandbox_execution_authorized",
        "build_execution_authorized",
        "output_execution_authorized",
        "allowed_gate_commands",
        "required_evidence",
        "source",
        "build_layout",
        "trusted_system_tools",
        "sandbox",
        "resources",
        "toolchain",
        "dynamic_library_policy",
        "cross_host_coordination",
        "invariants",
        "status",
        "next_allowed_stage",
    }:
        errors.append("schema required-field set differs")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        errors.append("schema properties are missing")
        return errors
    expected_consts = {
        "schema_version": "smpcc-r8-liquid-target-source-cpu-build-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_SOURCE_CPU_BUILD_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_SOURCE_CPU_BUILD_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "policy_review_authorized": True,
        "source_materialization_authorized": False,
        "sandbox_execution_authorized": False,
        "build_execution_authorized": False,
        "output_execution_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_POLICY_REVIEWED_BUILD_NOT_AUTHORIZED_TO_RUN",
        "next_allowed_stage": (
            "HUMAN_REVIEW_HARMLESS_OUTPUT_BIND_SMOKE_AND_SOURCE_MATERIALIZATION_POLICY"
        ),
    }
    for key, expected in expected_consts.items():
        if properties.get(key, {}).get("const") != expected:
            errors.append(f"schema const differs: {key}")
    return errors


def static_report(policy: Mapping[str, Any], schema: Mapping[str, Any]) -> dict[str, Any]:
    """Produce an in-memory report; it does not inspect the host or source tree."""

    errors = [*validate_policy(policy), *validate_schema(schema)]
    return {
        "status": (
            "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY"
            if not errors
            else "NO_GO_SOURCE_CPU_BUILD_POLICY_STATIC_INVALID"
        ),
        "policy_path": str(POLICY_PATH),
        "policy_canonical_sha256": canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_canonical_sha256": canonical_hash(schema),
        "errors": errors,
        "source_checkout_created": False,
        "source_materialized": False,
        "sandbox_created": False,
        "network_used": False,
        "cmake_executed": False,
        "make_executed": False,
        "compiler_executed": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "output_executed": False,
        "next_allowed_stage": (
            "HUMAN_REVIEW_HARMLESS_OUTPUT_BIND_SMOKE_AND_SOURCE_MATERIALIZATION_POLICY"
            if not errors
            else "REVIEW_POLICY_MISMATCH"
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    if args.command != "self-check":  # Defensive despite argparse's fixed choice.
        raise SourceCpuBuildPolicyError("unsupported static-only command")
    try:
        report = static_report(_read_json_object(POLICY_PATH), _read_json_object(SCHEMA_PATH))
    except (OSError, ValueError, SourceCpuBuildPolicyError) as exc:
        report = {
            "status": "NO_GO_SOURCE_CPU_BUILD_POLICY_STATIC_INVALID",
            "errors": [str(exc)],
            "source_checkout_created": False,
            "source_materialized": False,
            "sandbox_created": False,
            "network_used": False,
            "cmake_executed": False,
            "make_executed": False,
            "compiler_executed": False,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
            "output_executed": False,
            "next_allowed_stage": "REVIEW_POLICY_MISMATCH",
        }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
