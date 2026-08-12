#!/usr/bin/env python3
"""Create-new S6 real-execution v2 static contract.

This revision freezes the real command surface and the complete S5B0-v4
sixteen-probe Gauge contract.  The frozen parent is deliberately absent, so
every command fails closed without reading a bag, solver tree, optional input,
or writing outside this repository.  Library validators model the future real
admission, media, publication, and acceptance gates for negative testing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_final_delivery_policy_v2.json"
S5B0_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v4.json"
S5B0_SCHEMA = ROOT / "schema/target_host_s5b0_replay_admission_policy_v4.json"
SCHEMAS = {
    "policy": ROOT / "schema/target_host_s6_final_delivery_policy_v2.json",
    "admission": ROOT / "schema/target_host_s6_final_delivery_admission_receipt_v2.json",
    "execution": ROOT / "schema/target_host_s6_final_delivery_execution_receipt_v2.json",
    "final": ROOT / "schema/target_host_s6_final_delivery_receipt_v2.json",
}
COMMANDS = ("admit", "extract-selected", "analyze", "render-figure", "render-media", "publish", "accept", "run-one-shot")
ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
NOT_ADMITTED = "NOT_ADMITTED_FINALIZED_S5B0_REPLAY_REQUIRED"
ADMITTED = "ADMITTED_FINALIZED_S5B0_PRIMARY_PACKAGE_V2"
FINAL = "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY"
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")


class S6ExecutionV2Error(ValueError):
    """A frozen identity, closed schema, manifest, or claim differs."""


def canonical_json(value: Any) -> bytes:
    try:
        return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                           separators=(",", ":")) + "\n").encode()
    except (TypeError, ValueError) as exc:
        raise S6ExecutionV2Error("non-canonical JSON value") from exc


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_path(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"),
                       parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise S6ExecutionV2Error(f"JSON root is not an object: {path.name}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6ExecutionV2Error(f"schema object open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_contracts() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    policy = _json(POLICY)
    schemas = {name: _json(path) for name, path in SCHEMAS.items()}
    for schema in schemas.values():
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(schemas["policy"]).validate(policy)
    verify_s5b0_v4_binding(policy)
    return policy, schemas


def verify_s5b0_v4_binding(policy: Mapping[str, Any]) -> None:
    binding = policy["s5b0_v4_binding"]
    if sha256_path(S5B0_POLICY) != binding["policy_sha256"] or sha256_path(S5B0_SCHEMA) != binding["policy_schema_sha256"]:
        raise S6ExecutionV2Error("S5B0 v4 policy/schema identity drift")
    source = _json(S5B0_POLICY)["gauge_contract"]
    if sha256_json(source) != binding["gauge_contract_sha256"]:
        raise S6ExecutionV2Error("S5B0 v4 Gauge contract digest drift")
    if source != policy["gauge_contract"]:
        raise S6ExecutionV2Error("S6 v2 Gauge contract is not exact S5B0 v4")
    expected_names = [f"s5b0_p{index:02d}" for index in range(16)]
    if [probe["name"] for probe in source["probes"]] != expected_names:
        raise S6ExecutionV2Error("sixteen-probe order drift")


def _side_effects() -> dict[str, bool]:
    return {"bag_read": False, "optional_bag_read": False, "solver_executed": False,
            "gpu_exposed": False, "network_used": False, "sudo_used": False,
            "external_write_performed": False}


def _claims(stage6_pass: bool = False) -> dict[str, bool]:
    return {"stage6_pass": stage6_pass, "single_row": True,
            "physical_reference_pending": True, "physical_fidelity_validated": False,
            "paired_ranking": False, "cross_method_ranking": False,
            "formal": False, "production": False, "physical_primary": False}


def _execution_claims() -> dict[str, bool]:
    value = _claims()
    value["selected_trajectory_cpu_comparison"] = False
    return value


def _false_admission_checks() -> dict[str, bool]:
    names = ("exact_root", "no_symlink_components", "no_hardlinks", "no_special_files", "no_toctou",
             "exact_inventory", "execution_finalized", "execution_status_exact", "result_qc_pass",
             "frame_manifest_status_exact", "frame_manifest_integrity_pass", "checksums_complete",
             "gauge_bound_by_checksums", "gauge_contract_v4_exact", "attempt_exact")
    result = {name: False for name in names}
    result["gauge_contract_v4_exact"] = True
    result["attempt_exact"] = True
    result["optional_unread"] = True
    return result


def admit() -> dict[str, Any]:
    policy, schemas = load_contracts()
    if policy["s5b0_parent"]["state"] != "NOT_MATERIALIZED":
        raise S6ExecutionV2Error("repository v2 is static; real parent requires create-new revision")
    result = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-admission-receipt-v2",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_ADMISSION_RECEIPT_V2",
        "status": NOT_ADMITTED, "attempt_id": ATTEMPT, "planned_denominator": 1,
        "policy_sha256": sha256_path(POLICY), "package_root": None,
        "identities": {name: None for name in ("execution_receipt", "result_qc", "frame_manifest", "gauge_manifest", "checksums")},
        "checks": _false_admission_checks(), "side_effects": _side_effects(), "claims": _claims(),
    }
    Draft202012Validator(schemas["admission"]).validate(result)
    return result


def static_command(command: str, admission: Mapping[str, Any] | None = None) -> dict[str, Any]:
    if command not in COMMANDS[1:]:
        raise S6ExecutionV2Error("unknown post-admission command")
    policy, schemas = load_contracts()
    receipt = dict(admission or admit())
    Draft202012Validator(schemas["admission"]).validate(receipt)
    if receipt["status"] == ADMITTED:
        raise S6ExecutionV2Error("real execution is not implemented by static revision v2")
    result = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-execution-receipt-v2",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_EXECUTION_RECEIPT_V2",
        "command": command, "status": NOT_ADMITTED, "attempt_id": ATTEMPT,
        "planned_denominator": 1, "policy_sha256": sha256_path(POLICY),
        "admission_receipt_sha256": sha256_json(receipt),
        "checks": {"real_parent_materialized": False, "admission_pass": False,
                   "command_surface_closed": tuple(policy["execution_contract"]["commands"]) == COMMANDS,
                   "gauge_contract_v4_exact": True, "optional_unread": True, "external_write_absent": True},
        "publication_plan": {"required_inventory": policy["execution_contract"]["required_inventory"],
                             "inventory_count": 17, "artifacts_materialized": False,
                             "ledger_append_performed": False},
        "side_effects": _side_effects(), "claims": _execution_claims(),
    }
    Draft202012Validator(schemas["execution"]).validate(result)
    return result


def validate_admitted_receipt(value: Mapping[str, Any]) -> None:
    _, schemas = load_contracts()
    Draft202012Validator(schemas["admission"]).validate(dict(value))
    if value["status"] != ADMITTED:
        raise S6ExecutionV2Error("real command requires admitted parent")


def validate_native_gauge_manifest(value: Mapping[str, Any]) -> None:
    """Validate exact sixteen-file native Gauge manifest without opening files."""
    policy, _ = load_contracts()
    required = {"schema_version", "attempt_id", "gauge_contract_sha256", "time_grid_sha256", "files"}
    if set(value) != required or value["schema_version"] != "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1":
        raise S6ExecutionV2Error("Gauge manifest is not closed")
    if value["attempt_id"] != ATTEMPT or value["gauge_contract_sha256"] != policy["s5b0_v4_binding"]["gauge_contract_sha256"]:
        raise S6ExecutionV2Error("Gauge manifest parent binding drift")
    files = value["files"]
    if not isinstance(files, list) or len(files) != 16:
        raise S6ExecutionV2Error("Gauge manifest must contain sixteen files")
    expected = [probe["name"] for probe in policy["gauge_contract"]["probes"]]
    if [item.get("probe_name") for item in files] != expected:
        raise S6ExecutionV2Error("Gauge file order drift")
    for item in files:
        if set(item) != {"probe_name", "relative_path", "sha256", "size_bytes", "time_grid_sha256"}:
            raise S6ExecutionV2Error("Gauge file identity is open")
        path = PurePosixPath(item["relative_path"])
        if path.is_absolute() or ".." in path.parts or item["time_grid_sha256"] != value["time_grid_sha256"]:
            raise S6ExecutionV2Error("Gauge file path/time grid drift")
        if SHA_RE.fullmatch(str(item["sha256"])) is None or not isinstance(item["size_bytes"], int) or item["size_bytes"] < 1:
            raise S6ExecutionV2Error("Gauge file identity invalid")


def validate_media_manifest(value: Mapping[str, Any]) -> None:
    required = {"schema_version", "frame_manifest_sha256", "frames", "decode_qa", "keyframes"}
    if set(value) != required or value["schema_version"] != "smpcc-r8-liquid-s6-media-manifest-v2":
        raise S6ExecutionV2Error("media manifest is not closed")
    frames = value["frames"]
    if not isinstance(frames, list) or len(frames) < 3:
        raise S6ExecutionV2Error("media manifest requires at least three frames")
    previous = -math.inf
    for frame in frames:
        if set(frame) != {"index", "time_s", "sha256", "class_counts_sha256", "probe_overlay_sha256", "container_frame"}:
            raise S6ExecutionV2Error("media frame binding is open")
        time_s = frame["time_s"]
        if isinstance(time_s, bool) or not isinstance(time_s, (int, float)) or not math.isfinite(time_s) or time_s <= previous:
            raise S6ExecutionV2Error("media time grid invalid")
        previous = float(time_s)
        for key in ("sha256", "class_counts_sha256", "probe_overlay_sha256"):
            if SHA_RE.fullmatch(str(frame[key])) is None:
                raise S6ExecutionV2Error("media frame hash invalid")
        if frame["container_frame"] != "MOVING_CONTAINER_REFERENCE_REF_0":
            raise S6ExecutionV2Error("media container binding drift")
    if value["decode_qa"] != {"mp4_full_decode": True, "gif_full_decode": True, "decoded_frame_count": len(frames)}:
        raise S6ExecutionV2Error("media full-decode QA failed")
    if value["keyframes"] != {"first": 0, "middle": len(frames) // 2, "last": len(frames) - 1}:
        raise S6ExecutionV2Error("media keyframe contract drift")


def validate_publication_inventory(inventory: Sequence[str], checksums: Mapping[str, str],
                                   ledger_lines: Sequence[Mapping[str, Any]]) -> None:
    policy, _ = load_contracts()
    expected = policy["execution_contract"]["required_inventory"]
    if list(inventory) != expected or len(set(inventory)) != 17 or set(checksums) != set(expected):
        raise S6ExecutionV2Error("publication inventory/checksums are not exact")
    if any(SHA_RE.fullmatch(str(value)) is None for value in checksums.values()):
        raise S6ExecutionV2Error("publication checksum invalid")
    previous = "0" * 64
    seen: set[str] = set()
    for entry in ledger_lines:
        if set(entry) != {"entry_sha256", "previous_entry_sha256", "attempt_id", "stage6_status"}:
            raise S6ExecutionV2Error("ledger entry is not closed")
        if entry["previous_entry_sha256"] != previous or entry["attempt_id"] in seen:
            raise S6ExecutionV2Error("ledger duplicate or chain break")
        previous = entry["entry_sha256"]
        seen.add(entry["attempt_id"])


def validate_tree_snapshot(before: Mapping[str, tuple[int, int, int, int]],
                           after: Mapping[str, tuple[int, int, int, int]]) -> None:
    """Reject links, special files, inventory drift, and metadata TOCTOU."""
    if before != after or not before:
        raise S6ExecutionV2Error("tree inventory/TOCTOU drift")
    for relative, metadata in before.items():
        path = PurePosixPath(relative)
        mode, nlink, _size, _mtime_ns = metadata
        if path.is_absolute() or ".." in path.parts or not stat.S_ISREG(mode) or nlink != 1:
            raise S6ExecutionV2Error("tree contains link, special file, or unsafe path")


def final_receipt(gates: Mapping[str, Mapping[str, Any]], publication: Mapping[str, Any],
                  checks: Mapping[str, bool], *, request_pass: bool) -> dict[str, Any]:
    _, schemas = load_contracts()
    gate_names = ("admission", "extract_selected", "analysis", "figure", "media", "publish")
    if set(gates) != set(gate_names):
        raise S6ExecutionV2Error("final gate set differs")
    passed = request_pass and all(gates[name].get("pass") is True and SHA_RE.fullmatch(str(gates[name].get("receipt_sha256"))) for name in gate_names)
    passed = bool(passed and all(checks.values()) and all(publication.get(name) is not None for name in
                  ("comparison_manifest", "evidence_index", "ledger", "ledger_append_receipt", "checksums", "acceptance_receipt")))
    result = {
        "schema_version": "smpcc-r8-liquid-s6-final-delivery-receipt-v2",
        "document_type": "SMPCC_R8_LIQUID_S6_FINAL_DELIVERY_RECEIPT_V2",
        "status": FINAL if passed else NOT_ADMITTED, "attempt_id": ATTEMPT, "planned_denominator": 1,
        "gates": dict(gates), "publication": dict(publication), "acceptance_checks": dict(checks),
        "side_effects": _side_effects(),
        "claims": {**_execution_claims(), "development_only": True, "stage6_pass": passed},
    }
    Draft202012Validator(schemas["final"]).validate(result)
    return result


def self_check() -> dict[str, Any]:
    policy, schemas = load_contracts()
    admission = admit()
    commands = {command: static_command(command, admission)["status"] for command in COMMANDS[1:]}
    return {"status": "S6_FINAL_DELIVERY_EXECUTION_V2_SELF_CHECK_OK_NOT_ADMITTED",
            "admission": admission["status"], "commands": commands,
            "command_surface": list(COMMANDS), "schemas_deep_closed": len(schemas) == 4,
            "s5b0_v4_gauge_contract_bound": True, "probe_count": policy["gauge_contract"]["probe_count"],
            "required_inventory_count": len(policy["execution_contract"]["required_inventory"]),
            "optional_bag_read": False, "external_write_performed": False,
            "gpu_or_solver_executed": False, "network_used": False, "sudo_used": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", *COMMANDS))
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check()
        elif args.command == "admit":
            result = admit()
        else:
            result = static_command(args.command)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    except (S6ExecutionV2Error, ValidationError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_FINAL_DELIVERY_EXECUTION_V2_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["ADMITTED", "COMMANDS", "FINAL", "NOT_ADMITTED", "S6ExecutionV2Error",
           "admit", "assert_deep_closed", "canonical_json", "final_receipt", "load_contracts",
           "self_check", "sha256_bytes", "sha256_json", "static_command",
           "validate_admitted_receipt", "validate_media_manifest", "validate_native_gauge_manifest",
           "validate_publication_inventory", "validate_tree_snapshot", "verify_s5b0_v4_binding"]
