#!/usr/bin/env python3
"""Static-only validator for the MSI AppArmor output-bind review draft.

The gate reads checked-in policy, schema, and draft text only.  It deliberately
has no system-profile, process, namespace, or file-writing command path.
Passing means the review artifact is internally frozen; it is not permission
to install, load, select, or otherwise activate an AppArmor profile.
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
    / "config/target_hosts/liquid_zrj_msi_u2404_apparmor_output_bind_draft_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_apparmor_output_bind_draft_policy_v1.json"
DRAFT_RELATIVE_PATH = "config/apparmor_drafts/r8-liquid-output-bind-smoke.profile"
DRAFT_PATH = PACKAGE_DIR / DRAFT_RELATIVE_PATH
PROFILE_SHA256 = "4046975cdd27c064c7297a8b0cdec2740610c77840e476e3898dee3b5c05f560"

EXPECTED_PROFILE_LINES = (
    "abi <abi/4.0>,",
    "include <tunables/global>",
    "profile r8-liquid-output-bind-smoke flags=(attach_disconnected,mediate_deleted) {",
    "userns,",
    "/home/ r,",
    "/home/zrj/ r,",
    "/home/zrj/scout_liquid_lab/ r,",
    "/home/zrj/scout_liquid_lab/build/ r,",
    "/home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/ r,",
    "/home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/ w,",
    (
        "/home/zrj/scout_liquid_lab/build/"
        "u3_bwrap_output_bind_smoke_v1_*/output/.r8_output_bind_smoke_v1 rw,"
    ),
    "}",
)
FORBIDDEN_PROFILE_TOKENS = (
    "network",
    "mount",
    "umount",
    "remount",
    "capability",
    "change_profile",
    "dbus",
    "unix",
    "mqueue",
    "ptrace",
    "signal",
    "flags=(unconfined)",
    "/**",
    " ix",
    " px",
    " cx",
    " ux",
)


class AppArmorOutputBindDraftError(RuntimeError):
    """Any draft or policy ambiguity is a hard no-go."""


def canonical_hash(value: Any) -> str:
    """Return a formatting-independent collaboration hash for JSON content."""

    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_json_object(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise AppArmorOutputBindDraftError(f"JSON file is missing or unsafe: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AppArmorOutputBindDraftError(f"JSON file is not an object: {path}")
    return value


def _read_draft_text(path: Path) -> str:
    if path.is_symlink() or not path.is_file():
        raise AppArmorOutputBindDraftError(f"draft file is missing or unsafe: {path}")
    text = path.read_text(encoding="utf-8")
    if len(text.encode("utf-8")) > 65536:
        raise AppArmorOutputBindDraftError("draft file exceeds the static review size limit")
    return text


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _effective_profile_lines(text: str) -> tuple[str, ...]:
    """Remove comments and whitespace before comparing the closed draft body."""

    lines: list[str] = []
    for raw_line in text.splitlines():
        rule = raw_line.split("#", 1)[0].strip()
        if rule:
            lines.append(rule)
    return tuple(lines)


def _expected_evidence() -> dict[str, Any]:
    return {
        "output_bind_smoke_no_go": {
            "receipt_relative_path": "audits/u3_bwrap_output_bind_smoke_v1_20260806T111936Z.json",
            "file_sha256": "be0de443d5a25a8273366ad823a892a920ed574f46c258a0d39157bf668c24d0",
            "status": "NO_GO_STATIC_PRECHECK",
            "blocking_condition": "kernel.apparmor_restrict_unprivileged_userns=1",
        },
        "source_cpu_build_policy": {
            "policy_canonical_sha256": (
                "bcad3eed1bda70e24e6e0bd317a2241eef54637b5fb83f1411f10cb1cf944dcb"
            ),
            "required_status": "PASS_SOURCE_CPU_BUILD_POLICY_STATIC_ONLY",
            "build_execution_authorized": False,
        },
        "host_apparmor_observation": {
            "observed_at_utc": "2026-08-06",
            "apparmor_package_version": "4.0.1really4.0.1-0ubuntu0.24.04.7",
            "caller_label": "vscode (unconfined)",
            "unprivileged_userns_restriction": 1,
        },
    }


def _expected_profile_draft() -> dict[str, Any]:
    return {
        "relative_path": DRAFT_RELATIVE_PATH,
        "profile_name": "r8-liquid-output-bind-smoke",
        "file_sha256": PROFILE_SHA256,
        "review_state": "DRAFT_ONLY_NOT_APPROVED_FOR_LOADING",
        "attachment_mode": "named_only_no_attachment_path",
        "static_text_validation_only": True,
        "required_header_markers": [
            "DRAFT_ONLY: NOT_APPROVED_FOR_LOADING",
            "has no attachment path",
            "cannot authorize bubblewrap",
        ],
        "approved_write_paths": [
            "/home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/",
            (
                "/home/zrj/scout_liquid_lab/build/"
                "u3_bwrap_output_bind_smoke_v1_*/output/.r8_output_bind_smoke_v1"
            ),
        ],
    }


def _expected_confinement_contract() -> dict[str, Any]:
    return {
        "user_namespace": "review_target_only_not_an_activation_grant",
        "network": "default_deny_no_network_rule",
        "mount_operations": "default_deny_no_mount_umount_or_remount_rule",
        "capabilities": "default_deny_no_capability_rule",
        "executables": "default_deny_no_execute_rule",
        "broad_filesystem_rule": "forbidden",
        "unconfined_flag": "forbidden",
        "output_marker_creation": "not_authorized_by_this_draft",
        "forbidden_host_path_prefixes": [
            "/home/zrj/scout_ws",
            "/home/zrj/ros2_ws",
            "/opt/ros/jazzy",
            "/data",
            "/home/zrj/scout_liquid_lab/dependency",
            "/home/zrj/scout_liquid_lab/incoming",
            "/home/zrj/scout_liquid_lab/outgoing",
            "/home/zrj/scout_liquid_lab/results",
        ],
    }


def _expected_admin_review_requirements() -> list[str]:
    return [
        "Independently review this draft against Ubuntu 24.04 AppArmor 4.0.1 semantics before any activation proposal.",
        "Derive an exact minimal mount-namespace plan from the frozen one-marker bwrap argv; do not add broad mount rules or capability sys_admin.",
        "Review profile transition and child-execution semantics so that only the future fixed argv can reach the marker path.",
        "Create a new reviewed profile revision and a separate execution admission before any system installation, loading, profile selection, namespace attempt, or retry.",
    ]


def _expected_cross_host_coordination() -> dict[str, Any]:
    return {
        "allowed_exchange": [
            "Git commit or reviewed patch",
            "policy/schema/profile/gate/test SHA-256 manifest",
            "static self-check JSON/text report",
        ],
        "profile_transfer_to_gazebo_host": "review_text_and_hash_only",
        "system_profile_or_privileged_command_transfer": "forbidden",
        "source_or_binary_transfer_for_execution": "forbidden",
        "secrets_or_credentials_in_handoff": "forbidden",
        "each_handoff_requires_manifest_and_sha256": True,
    }


def validate_policy(policy: Mapping[str, Any]) -> list[str]:
    """Validate the frozen static review contract, not the host system."""

    errors: list[str] = []
    expected_top = {
        "schema_version": "smpcc-r8-liquid-target-apparmor-output-bind-draft-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_OUTPUT_BIND_DRAFT_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_OUTPUT_BIND_DRAFT_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "draft_review_authorized": True,
        "profile_installation_authorized": False,
        "profile_loading_authorized": False,
        "profile_execution_authorized": False,
        "system_configuration_change_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_APPARMOR_DRAFT_REVIEWED_NOT_APPROVED_TO_LOAD",
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_NARROW_MOUNT_PLAN_AND_PROFILE_REVISION"
        ),
    }
    for key, expected in expected_top.items():
        if policy.get(key) != expected:
            errors.append(f"policy field mismatch: {key}")

    sections = (
        ("required_evidence", _expected_evidence(), "required evidence differs"),
        ("profile_draft", _expected_profile_draft(), "profile draft identity or scope differs"),
        (
            "confinement_contract",
            _expected_confinement_contract(),
            "confinement contract weakens default-deny boundary",
        ),
        (
            "admin_review_requirements",
            _expected_admin_review_requirements(),
            "administrator review requirements differ",
        ),
        (
            "cross_host_coordination",
            _expected_cross_host_coordination(),
            "two-host AppArmor handoff boundary differs",
        ),
    )
    for key, expected, message in sections:
        if policy.get(key) != expected:
            errors.append(message)

    expected_invariants = {
        "no_sudo",
        "no_system_profile_install",
        "no_profile_load",
        "no_profile_enable",
        "no_profile_execution",
        "no_aa_exec",
        "no_profile_parser",
        "no_sysctl_change",
        "no_apparmor_change",
        "no_network_permission",
        "no_mount_permission",
        "no_capability_permission",
        "no_unconfined_flag",
        "no_broad_file_rule",
        "no_workspace_or_ros_access",
        "no_source_or_precompiled_binary_access",
        "no_namespace_attempt",
        "no_upstream_code_execution",
        "no_build",
        "no_destructive_cleanup",
    }
    invariants = policy.get("invariants")
    if not isinstance(invariants, dict) or set(invariants) != expected_invariants or any(
        invariants.get(key) is not True for key in expected_invariants
    ):
        errors.append("AppArmor draft safety invariants are incomplete")
    return errors


def validate_schema(schema: Mapping[str, Any]) -> list[str]:
    """Check the closed top-level schema for the static-only contract."""

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
        "draft_review_authorized",
        "profile_installation_authorized",
        "profile_loading_authorized",
        "profile_execution_authorized",
        "system_configuration_change_authorized",
        "allowed_gate_commands",
        "required_evidence",
        "profile_draft",
        "confinement_contract",
        "admin_review_requirements",
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
        "schema_version": "smpcc-r8-liquid-target-apparmor-output-bind-draft-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_APPARMOR_OUTPUT_BIND_DRAFT_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_APPARMOR_OUTPUT_BIND_DRAFT_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "draft_review_authorized": True,
        "profile_installation_authorized": False,
        "profile_loading_authorized": False,
        "profile_execution_authorized": False,
        "system_configuration_change_authorized": False,
        "allowed_gate_commands": ["self-check"],
        "status": "STATIC_APPARMOR_DRAFT_REVIEWED_NOT_APPROVED_TO_LOAD",
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_NARROW_MOUNT_PLAN_AND_PROFILE_REVISION"
        ),
    }
    for key, expected in expected_consts.items():
        if properties.get(key, {}).get("const") != expected:
            errors.append(f"schema const differs: {key}")
    return errors


def validate_draft_text(draft_text: str) -> list[str]:
    """Check that the profile remains the exact default-deny review artifact."""

    errors: list[str] = []
    for marker in _expected_profile_draft()["required_header_markers"]:
        if marker not in draft_text:
            errors.append(f"draft header marker is missing: {marker}")
    effective = _effective_profile_lines(draft_text)
    if effective != EXPECTED_PROFILE_LINES:
        errors.append("draft non-comment rules differ from the frozen default-deny body")
    joined = "\n".join(effective)
    for token in FORBIDDEN_PROFILE_TOKENS:
        if token in joined:
            errors.append(f"draft has a forbidden permission token: {token}")
    return errors


def static_report(
    policy: Mapping[str, Any], schema: Mapping[str, Any], draft_text: str, draft_sha256: str
) -> dict[str, Any]:
    """Return an in-memory report without inspecting or changing the host."""

    errors = [*validate_policy(policy), *validate_schema(schema), *validate_draft_text(draft_text)]
    if draft_sha256 != PROFILE_SHA256:
        errors.append("draft file SHA-256 differs")
    return {
        "status": (
            "PASS_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_ONLY"
            if not errors
            else "NO_GO_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_INVALID"
        ),
        "policy_path": str(POLICY_PATH),
        "policy_canonical_sha256": canonical_hash(policy),
        "schema_path": str(SCHEMA_PATH),
        "schema_canonical_sha256": canonical_hash(schema),
        "draft_path": str(DRAFT_PATH),
        "draft_file_sha256": draft_sha256,
        "errors": errors,
        "static_text_validation_only": True,
        "profile_copied_to_system": False,
        "profile_parser_invoked": False,
        "profile_loaded": False,
        "profile_selected_for_execution": False,
        "namespace_attempted": False,
        "network_used": False,
        "source_checkout_created": False,
        "source_materialized": False,
        "upstream_code_executed": False,
        "build_executed": False,
        "sudo_used": False,
        "sysctl_or_apparmor_changed": False,
        "next_allowed_stage": (
            "INDEPENDENT_ADMIN_REVIEW_OF_NARROW_MOUNT_PLAN_AND_PROFILE_REVISION"
            if not errors
            else "REVIEW_STATIC_DRAFT_MISMATCH"
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    if args.command != "self-check":
        raise AppArmorOutputBindDraftError("unsupported static-only command")
    try:
        policy = _read_json_object(POLICY_PATH)
        schema = _read_json_object(SCHEMA_PATH)
        draft_text = _read_draft_text(DRAFT_PATH)
        report = static_report(policy, schema, draft_text, _file_sha256(DRAFT_PATH))
    except (OSError, ValueError, AppArmorOutputBindDraftError) as exc:
        report = {
            "status": "NO_GO_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_INVALID",
            "errors": [str(exc)],
            "static_text_validation_only": True,
            "profile_copied_to_system": False,
            "profile_parser_invoked": False,
            "profile_loaded": False,
            "profile_selected_for_execution": False,
            "namespace_attempted": False,
            "network_used": False,
            "source_checkout_created": False,
            "source_materialized": False,
            "upstream_code_executed": False,
            "build_executed": False,
            "sudo_used": False,
            "sysctl_or_apparmor_changed": False,
            "next_allowed_stage": "REVIEW_STATIC_DRAFT_MISMATCH",
        }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS_APPARMOR_OUTPUT_BIND_DRAFT_STATIC_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
