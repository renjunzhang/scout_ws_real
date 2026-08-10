#!/usr/bin/env python3
"""Static validator for the recorded MSI transient AppArmor userns probe.

This gate has one read-only ``self-check`` action.  It validates the checked-in
profile, policy, and schema that describe an already-completed, bounded probe.
It deliberately has no path to call sudo, apparmor_parser, aa-exec, bwrap, or
any process-creation API.  A passing result preserves the fail-closed mount
denial as evidence; it never authorizes replaying the probe or activating a
profile.
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
    / "config/target_hosts/liquid_zrj_msi_u2404_apparmor_userns_probe_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_apparmor_userns_probe_policy_v1.json"
PROFILE_RELATIVE_PATH = "config/apparmor_drafts/r8-liquid-system-true-userns-probe.profile"
PROFILE_PATH = PACKAGE_DIR / PROFILE_RELATIVE_PATH
PROFILE_SHA256 = "5b6fe28c63184d41224f462e9f8ee05b8b6f28aca3202fa3bdc66264a0694b31"

EXPECTED_EFFECTIVE_PROFILE_LINES = (
    "abi <abi/4.0>,",
    "include <tunables/global>",
    "profile r8-liquid-system-true-userns-probe flags=(attach_disconnected,mediate_deleted) {",
    "userns create,",
    "/usr/bin/bwrap rix,",
    "/usr/bin/true rix,",
    "/usr/lib/** mr,",
    "/usr/lib64/** mr,",
    "/lib/** mr,",
    "/lib64/** mr,",
    "/etc/ld.so.cache r,",
    "/etc/ld.so.conf r,",
    "/etc/ld.so.conf.d/** r,",
    "owner /proc/** r,",
    "owner /proc/*/uid_map w,",
    "owner /proc/*/gid_map w,",
    "owner /proc/*/setgroups w,",
    "/proc/filesystems r,",
    "/proc/sys/kernel/overflowuid r,",
    "/proc/sys/kernel/overflowgid r,",
    "/dev/null rw,",
    "/dev/zero r,",
    "/dev/random r,",
    "/dev/urandom r,",
    "capability sys_admin,",
    "capability sys_ptrace,",
    "capability setpcap,",
    "capability net_admin,",
    "ptrace (read, readby) peer=r8-liquid-system-true-userns-probe,",
    "network unix dgram,",
    "network inet dgram,",
    "network inet6 dgram,",
    "network netlink raw,",
    "}",
)
FORBIDDEN_PROFILE_TOKENS = (
    "mount,",
    "umount",
    "remount",
    "change_profile",
    "flags=(unconfined)",
    "/home/",
    "/data/",
    "/opt/ros/",
    "/usr/bin/sh",
    "/usr/bin/env",
    "/usr/bin/python",
    "network inet stream",
    "network inet6 stream",
    "capability dac_override",
)


class AppArmorUsernsProbeError(RuntimeError):
    """Any mismatch is a fail-closed collaboration result."""


def canonical_hash(value: Any) -> str:
    """Return a formatting-independent collaboration hash for JSON content."""

    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_json_object(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise AppArmorUsernsProbeError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AppArmorUsernsProbeError(f"JSON file is not an object: {path}")
    return value


def _read_profile_text(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise AppArmorUsernsProbeError(f"profile file is missing or unsafe: {path}")
    text = path.read_text(encoding="utf-8")
    if len(text.encode("utf-8")) > 65536:
        raise AppArmorUsernsProbeError("profile file exceeds the static review size limit")
    return text


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _effective_profile_lines(text: str) -> tuple[str, ...]:
    """Remove comments and whitespace before comparing the closed profile body."""

    rules: list[str] = []
    for raw_line in text.splitlines():
        rule = raw_line.split("#", 1)[0].strip()
        if rule:
            rules.append(rule)
    return tuple(rules)


def _expected_profile_contract() -> dict[str, Any]:
    return {
        "relative_path": PROFILE_RELATIVE_PATH,
        "profile_name": "r8-liquid-system-true-userns-probe",
        "file_sha256": PROFILE_SHA256,
        "attachment_mode": "named_only_no_attachment_path",
        "persistent_installation": "forbidden",
        "activation_lifecycle": [
            "manual single-use workspace load only",
            "select the exact named profile only for the fixed bounded probe",
            "remove the profile immediately after the probe",
            "verify zero loaded-profile matches and zero bwrap_or_aa_exec residues",
        ],
        "only_permitted_exec_paths": ["/usr/bin/bwrap", "/usr/bin/true"],
        "only_owner_proc_write_paths": [
            "/proc/*/uid_map",
            "/proc/*/gid_map",
            "/proc/*/setgroups",
        ],
        "mount_rule": "absent_default_deny",
        "network_rules": [
            "network unix dgram",
            "network inet dgram",
            "network inet6 dgram",
            "network netlink raw",
        ],
        "capabilities": ["sys_admin", "sys_ptrace", "setpcap", "net_admin"],
    }


def _expected_probe_contract() -> dict[str, Any]:
    return {
        "outer_argv": [
            "/usr/bin/timeout",
            "--preserve-status",
            "--signal=TERM",
            "--kill-after=2s",
            "30s",
            "/usr/bin/prlimit",
            "--as=1073741824",
            "--cpu=10",
            "--nproc=4096",
            "--nofile=128",
            "--",
            "/usr/bin/aa-exec",
            "-p",
            "r8-liquid-system-true-userns-probe",
            "--",
            "/usr/bin/bwrap",
            "--unshare-user",
            "--unshare-pid",
            "--unshare-net",
            "--unshare-ipc",
            "--unshare-uts",
            "--",
            "/usr/bin/true",
        ],
        "wall_timeout_seconds": 30,
        "address_space_bytes": 1073741824,
        "cpu_time_seconds": 10,
        "process_limit": 4096,
        "open_file_limit": 128,
        "network_namespace": "required",
        "host_writable_mounts": [],
        "source_or_output_bind": "forbidden",
        "upstream_command": "forbidden",
    }


def _expected_observed_result() -> dict[str, Any]:
    return {
        "observed_at_local": "2026-08-06T21:15:23+08:00",
        "returncode": 1,
        "terminal_stderr": "bwrap: Failed to make / slave: Permission denied",
        "user_namespace_admission": "reached_uid_gid_mapping_then_mount_boundary",
        "first_nonpermitted_operation": {
            "apparmor_class": "mount",
            "mount_target": "/",
            "flags": ["rw", "silent", "rslave"],
            "result": "DENIED_BY_ABSENT_APPARMOR_MOUNT_RULE",
        },
        "namespace_completed": False,
        "host_writable_mounts_created": [],
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_executed": False,
        "profile_removed_after_probe": True,
        "loaded_profile_match_count_after_removal": 0,
        "bwrap_or_aa_exec_process_residue_count_after_removal": 0,
        "receipt": {
            "relative_path": "audits/u2_apparmor_userns_probe_v1_20260806T131601Z.json",
            "file_sha256": "b36b9233815b6f1c68bbf85a68d68538f85f4fd7110e4ebe48f74638bde71123",
            "mode_octal": "0640",
            "status": "PASS_USERNS_ADMISSION_MOUNT_DENIED",
        },
    }


def _expected_cross_host_coordination() -> dict[str, Any]:
    return {
        "allowed_exchange": [
            "Git commit or reviewed patch",
            "profile/policy/schema/gate/test SHA-256 manifest",
            "static self-check report",
            "sanitized AppArmor audit excerpt that contains no credentials",
        ],
        "system_profile_or_privileged_command_transfer": "forbidden",
        "source_or_binary_transfer_for_execution": "forbidden",
        "secrets_or_credentials_in_handoff": "forbidden",
        "result_interpretation": (
            "userns admission evidence only; not output-bind, build, GenCase, solver, ROS, GPU, or mount authorization"
        ),
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate the closed static record; no host state is inspected."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-userns-probe-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_USERNS_PROBE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_USERNS_PROBE_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "static_review_authorized": True,
        "profile_loading_or_replay_authorized": False,
        "system_configuration_change_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "OBSERVED_USERNS_ADMISSION_MOUNT_DENIED",
        "next_allowed_stage": "INDEPENDENT_REVIEW_OF_EXACT_MOUNT_CONFINEMENT_ONLY",
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            errors.append(f"policy field mismatch: {key}")

    sections = (
        ("profile_contract", _expected_profile_contract(), "profile contract differs"),
        ("fixed_probe_contract", _expected_probe_contract(), "fixed probe contract differs"),
        ("observed_result", _expected_observed_result(), "observed result differs"),
        (
            "cross_host_coordination",
            _expected_cross_host_coordination(),
            "cross-host handoff boundary differs",
        ),
    )
    for key, expected, message in sections:
        if policy.get(key) != expected:
            errors.append(message)

    expected_invariants = {
        "no_persistent_system_profile_install",
        "no_sysctl_change",
        "no_driver_or_package_change",
        "no_mount_rule",
        "no_host_writable_bind",
        "no_network_execution",
        "no_source_checkout_or_materialization",
        "no_upstream_code_or_precompiled_elf_execution",
        "no_cmake_or_make",
        "no_ros_or_gazebo",
        "no_gpu_device",
        "no_destructive_cleanup",
        "no_future_probe_replay_from_this_static_gate",
    }
    invariants = policy.get("invariants")
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("userns-probe safety invariants are incomplete")
    return errors


def validate_schema(schema: Mapping[str, Any]) -> list[str]:
    """Validate enough schema structure to make collaboration changes visible."""

    errors: list[str] = []
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema draft differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        errors.append("schema must be a closed top-level object")
    expected_required = {
        "schema_version",
        "document_type",
        "policy_id",
        "host_id",
        "development_only",
        "formal",
        "physical_primary_eligible",
        "static_review_authorized",
        "profile_loading_or_replay_authorized",
        "system_configuration_change_authorized",
        "allowed_gate_commands",
        "profile_contract",
        "fixed_probe_contract",
        "observed_result",
        "cross_host_coordination",
        "invariants",
        "status",
        "next_allowed_stage",
    }
    if not isinstance(schema.get("required"), list) or set(schema["required"]) != expected_required:
        errors.append("schema required-field set differs")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        return [*errors, "schema properties are missing"]
    expected_consts = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-userns-probe-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_USERNS_PROBE_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_USERNS_PROBE_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "static_review_authorized": True,
        "profile_loading_or_replay_authorized": False,
        "system_configuration_change_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "OBSERVED_USERNS_ADMISSION_MOUNT_DENIED",
        "next_allowed_stage": "INDEPENDENT_REVIEW_OF_EXACT_MOUNT_CONFINEMENT_ONLY",
    }
    for key, expected in expected_consts.items():
        if properties.get(key, {}).get("const") != expected:
            errors.append(f"schema const differs: {key}")
    return errors


def validate_profile_text(text: str) -> list[str]:
    """Reject any profile expansion beyond the exact recorded probe body."""

    errors: list[str] = []
    for marker in (
        "EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.",
        "has no attachment path",
        "copied to /etc/apparmor.d or enabled for boot-time loading",
        "all mount operations remain denied",
    ):
        if marker not in text:
            errors.append(f"profile header marker is missing: {marker}")
    effective = _effective_profile_lines(text)
    if effective != EXPECTED_EFFECTIVE_PROFILE_LINES:
        errors.append("profile non-comment rules differ from the frozen probe body")
    joined = "\n".join(effective)
    for token in FORBIDDEN_PROFILE_TOKENS:
        if token in joined:
            errors.append(f"profile has a forbidden permission token: {token}")
    return errors


def static_report(
    policy: Mapping[str, Any], schema: Mapping[str, Any], profile_text: str, profile_sha256: str
) -> dict[str, Any]:
    """Return a purely in-memory report about the recorded safety contract."""

    errors = [
        *validate_policy(policy),
        *validate_schema(schema),
        *validate_profile_text(profile_text),
    ]
    if profile_sha256 != PROFILE_SHA256:
        errors.append("profile file SHA-256 differs")
    return {
        "status": (
            "PASS_APPARMOR_USERNS_PROBE_STATIC_RECORD"
            if not errors
            else "NO_GO_APPARMOR_USERNS_PROBE_STATIC_INVALID"
        ),
        "policy_path": str(POLICY_PATH),
        "policy_canonical_sha256": canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_canonical_sha256": canonical_hash(schema),
        "profile_path": str(PROFILE_PATH),
        "profile_file_sha256": profile_sha256,
        "recorded_dynamic_status": policy.get("status"),
        "errors": errors,
        "static_validation_only": True,
        "profile_parser_invoked": False,
        "profile_loaded": False,
        "profile_selected_for_execution": False,
        "namespace_attempted": False,
        "mount_attempted": False,
        "network_used": False,
        "source_checkout_created": False,
        "upstream_code_executed": False,
        "build_executed": False,
        "sudo_used": False,
        "system_configuration_changed": False,
        "next_allowed_stage": (
            "INDEPENDENT_REVIEW_OF_EXACT_MOUNT_CONFINEMENT_ONLY"
            if not errors
            else "REVIEW_STATIC_USERNS_PROBE_RECORD_MISMATCH"
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    if args.command != "self-check":
        raise AppArmorUsernsProbeError("unsupported static-only command")
    try:
        policy = _read_json_object(POLICY_PATH)
        schema = _read_json_object(SCHEMA_PATH)
        profile_text = _read_profile_text(PROFILE_PATH)
        report = static_report(policy, schema, profile_text, _file_sha256(PROFILE_PATH))
    except (OSError, ValueError, AppArmorUsernsProbeError) as exc:
        report = {
            "status": "NO_GO_APPARMOR_USERNS_PROBE_STATIC_INVALID",
            "errors": [str(exc)],
            "static_validation_only": True,
        }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS_APPARMOR_USERNS_PROBE_STATIC_RECORD" else 1


if __name__ == "__main__":
    sys.exit(main())
