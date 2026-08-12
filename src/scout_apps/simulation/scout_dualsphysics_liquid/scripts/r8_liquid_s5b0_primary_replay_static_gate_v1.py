#!/usr/bin/env python3
"""Static-only S5B0 primary replay admission gate revision v1.

The gate reads only its repository policy, receipt schema, and uninstantiated
AppArmor template.  It can validate in-memory transfer/clone/motion fixtures
and render a dry case plan, but it never creates a root, loads a profile,
starts bubblewrap, exposes a GPU, or executes DualSPHysics.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_primary_replay_static_policy_v1.json"
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_primary_replay_static_receipt_v1.json"
MAX_STATIC_BYTES = 2 * 1024 * 1024
EXPECTED_POLICY_ID = "liquid_zrj_msi_u2404_s5b0_primary_replay_static_v1"
EXPECTED_TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
EXPECTED_ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"


class S5B0StaticError(ValueError):
    """A static S5B0 identity, budget, case, profile, or claim drifted."""


def _fail(code: str, message: str) -> None:
    raise S5B0StaticError(f"{code}: {message}")


def _is_sha(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        _fail("NONFINITE_VALUE", label)
    return float(value)


def _read_static(path: Path) -> bytes:
    raw = path.read_bytes()
    if not raw or len(raw) > MAX_STATIC_BYTES:
        _fail("STATIC_FILE_SIZE", str(path))
    return raw


def _read_json(path: Path) -> tuple[dict[str, Any], str]:
    raw = _read_static(path)
    try:
        value = json.loads(raw)
    except (UnicodeError, json.JSONDecodeError) as exc:
        _fail("STATIC_JSON_INVALID", f"{path}: {exc}")
    if not isinstance(value, dict):
        _fail("STATIC_JSON_ROOT", str(path))
    return value, hashlib.sha256(raw).hexdigest()


def assert_deep_closed(value: object, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if (kind == "object" or (isinstance(kind, list) and "object" in kind)) and value.get("additionalProperties") is not False:
            _fail("SCHEMA_OPEN_OBJECT", location)
        for key, item in value.items():
            assert_deep_closed(item, f"{location}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(item, f"{location}/{index}")


def validate_policy(policy: Mapping[str, Any]) -> None:
    expected_root = {
        "schema_version", "document_type", "policy_id", "status", "selection",
        "parent_transfer", "frozen_parents", "settled_clone_contract", "geometry",
        "moving_domain_budget", "runtime_budget", "mvpathfile_contract",
        "profile_contract", "result_contract", "execution_boundary", "next",
    }
    if set(policy) != expected_root:
        _fail("POLICY_ROOT_NOT_CLOSED", "root keys differ")
    if policy["schema_version"] != "smpcc-r8-liquid-s5b0-primary-replay-static-policy-v1" or policy["policy_id"] != EXPECTED_POLICY_ID:
        _fail("POLICY_IDENTITY", "version/id differs")
    if policy["status"] != "STATIC_DESIGN_ONLY_NOT_ADMITTED":
        _fail("POLICY_STATUS", "static policy was promoted")

    selection = policy["selection"]
    if selection != {
        "attempt_id": EXPECTED_ATTEMPT,
        "role": "PRIMARY_BASELINE",
        "container": "C1",
        "source_domain": "SIM_R7_EXECUTED_GAZEBO_MOTION",
        "source_provenance": "PARTIAL_BAG_ONLY",
        "planned_denominator": 1,
        "optional_authorized": False,
        "c2_authorized": False,
    }:
        _fail("SELECTION_DRIFT", "only the primary C1 row with denominator one is allowed")

    transfer = policy["parent_transfer"]
    if transfer != {
        "transfer_id": EXPECTED_TRANSFER_ID,
        "abi": "R8-LIQUID-HANDOFF-ABI-v3",
        "state": "NOT_MATERIALIZED",
        "receipt_path": None,
        "receipt_sha256": None,
        "package_root": None,
        "manifest_sha256": None,
    }:
        _fail("TRANSFER_STATE_DRIFT", "this revision must remain explicitly NOT_MATERIALIZED")

    parents = policy["frozen_parents"]
    if set(parents) != {"gpu_candidate", "c1m_case_bi4", "c1m_case_xml", "compatibility_contract", "compatibility_gate", "settled_receipt"}:
        _fail("PARENT_SET_DRIFT", "frozen parent names differ")
    for name, identity in parents.items():
        if set(identity) != {"path", "sha256"} or not isinstance(identity["path"], str) or not identity["path"].startswith("/") or not _is_sha(identity["sha256"]):
            _fail("PARENT_IDENTITY_INVALID", name)
    if parents["compatibility_contract"]["sha256"] != "f0786be1f7054fcbc29a9eb0d7f611c2bd3bdee8e9dbe4f022962713e70b6e73" or parents["compatibility_gate"]["sha256"] != "8c924726550a85fc7d5d0bf7af03f7f92ca60218dbe3d97c75df4ea43defba4b":
        _fail("C1_COMPATIBILITY_DRIFT", "C1 compatibility identity differs")

    clone = policy["settled_clone_contract"]
    required = {item.get("name"): item.get("sha256") for item in clone.get("required_files", [])}
    if required != {
        "Part_0901.bi4": "023c2ae47281351f7c601a60709799cf8155745193d7e2aa5ee862256ad9b1f4",
        "Part_Head.ibi4": "6511cbf54319ce0fbdde9aefc6303f75dee53b56530db3aa25b18212281c266a",
    }:
        _fail("SETTLED_PARENT_DRIFT", "Part_0901/Part_Head identity differs")
    if clone.get("clone_state") != "NOT_MATERIALIZED" or clone.get("fresh_create_new_required") is not True or clone.get("source_unchanged_required") is not True or clone.get("dynamic_state_reuse_forbidden") is not True:
        _fail("CLONE_CONTRACT_DRIFT", "fresh-clone boundary differs")
    if clone.get("part_index") != 901 or clone.get("part_time_s") != 45.05001991890928 or clone.get("particle_count") != 9078:
        _fail("SETTLED_SEMANTICS_DRIFT", "settled checkpoint semantics differ")

    geometry = policy["geometry"]
    if geometry != {"container_radius_m": 0.0185, "boundary_height_m": 0.066, "rotation_center_m": [0.0, 0.0, 0.0], "particle_spacing_m": 0.002}:
        _fail("GEOMETRY_DRIFT", "C1M geometry differs")
    for section in (policy["moving_domain_budget"], policy["runtime_budget"]):
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0 for value in section.values()):
            _fail("BUDGET_INVALID", "budget values must be positive")
    if policy["moving_domain_budget"]["maximum_gpu_bytes"] > 12884901888 or policy["runtime_budget"]["maximum_output_bytes"] > 1073741824:
        _fail("BUDGET_PROMOTED", "GPU/output ceiling increased")

    motion = policy["mvpathfile_contract"]
    required_motion = {
        "mode": "DRY_STATIC_ONLY", "filename": "solver_path.csv", "fields": 7,
        "field_indexes": [0, 1, 2, 3, 4, 5, 6], "anglesunits": "degrees",
        "axes": "ZYX", "intrinsic": True, "movecenter": True,
        "center_m": [0.0, 0.0, 0.0], "begin_start_s": 45.05001991890928,
        "duration_source": "FINAL_TRANSFER_SOLVER_PATH_LAST_T_S", "next_motion_id": 2,
        "final_hold_motion": "mvnull", "solver_tail_s": 1.0,
        "motion_block_replacement_count": 2, "first_row_identity_required": True,
        "strict_time_required": True,
    }
    if motion != required_motion:
        _fail("MVPATHFILE_CONTRACT_DRIFT", "solver bridge or full-tail contract differs")

    boundary = policy["execution_boundary"]
    if set(boundary) != {"transfer_materialized", "compatibility_receipt_materialized", "settled_clone_materialized", "gpu_replay_authorized", "profile_instantiated", "profile_loaded", "solver_executed", "external_root_created", "formal", "physical_fidelity_validated"} or any(value is not False for value in boundary.values()):
        _fail("EXECUTION_BOUNDARY_PROMOTED", "static revision claims runtime activity")
    result = policy["result_contract"]
    if len(result.get("required_inventory", [])) != 11 or len(set(result["required_inventory"])) != 11:
        _fail("RESULT_INVENTORY_DRIFT", "result inventory is not exact")
    for required_name in ("result_manifest.json", "executed_boundary_motion.csv", "gauge_zsurf.csv", "qc_report.json", "checksums.sha256"):
        if required_name not in result["required_inventory"]:
            _fail("RESULT_INVENTORY_MISSING", required_name)
    if result.get("fresh_partial_root_required") is not True or result.get("atomic_noreplace_publish_required") is not True or result.get("failure_preserves_partial") is not True:
        _fail("RESULT_LIFECYCLE_DRIFT", "create-new/failure preservation differs")
    if result.get("gauge_slots_complete_required") is not True or result.get("boundary_motion_qc_required") is not True or result.get("nout_required") != 0 or result.get("finite_required") is not True:
        _fail("RESULT_QC_DRIFT", "Gauge/boundary/Nout/finite QC differs")


def query_profile_template(policy: Mapping[str, Any], text: str) -> dict[str, Any]:
    contract = policy["profile_contract"]
    if contract["template_state"] != "NOT_INSTANTIATED_NOT_LOADED" or contract["load_authorized"] is not False or contract["query_mode"] != "STATIC_TEMPLATE_TEXT_ONLY":
        _fail("PROFILE_STATE_DRIFT", "profile template was promoted")
    if "NOT INSTANTIATED, NOT LOADABLE AS-IS" not in text:
        _fail("PROFILE_MARKER_MISSING", "static marker absent")
    for placeholder in contract["required_placeholders"]:
        if placeholder not in text:
            _fail("PROFILE_PLACEHOLDER_MISSING", placeholder)
    for rule in contract["required_rules"]:
        if text.splitlines().count(rule) != 1:
            _fail("PROFILE_PERMISSION_DRIFT", rule)
    for rule in contract["forbidden_rules"]:
        if text.splitlines().count(rule):
            _fail("PROFILE_PERMISSION_PROMOTED", rule)
    return {
        "status": "PASS_S5B0_APPARMOR_TEMPLATE_STATIC_QUERY_V1",
        "state": "NOT_INSTANTIATED_NOT_LOADED",
        "required_rule_count": len(contract["required_rules"]),
        "profile_queried": False,
        "profile_loaded": False,
    }


def validate_parent_identity(policy: Mapping[str, Any], name: str, observed: Mapping[str, Any]) -> None:
    expected = policy["frozen_parents"].get(name)
    if expected is None or set(observed) != {"path", "sha256"} or observed != expected:
        _fail("PARENT_HASH_DRIFT", name)


def validate_compatibility_receipt(policy: Mapping[str, Any], receipt: Mapping[str, Any]) -> None:
    expected = {
        "status": "PASS_S5_C1_TO_C1M_DEVELOPMENT_REPLAY_COMPATIBILITY_V1",
        "container": "C1",
        "c2_read_or_admitted": False,
        "optional_pair_read_or_admitted": False,
        "files_written": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "contract_sha256": policy["frozen_parents"]["compatibility_contract"]["sha256"],
        "gate_sha256": policy["frozen_parents"]["compatibility_gate"]["sha256"],
    }
    if dict(receipt) != expected:
        _fail("C1_COMPATIBILITY_RECEIPT_INVALID", "receipt differs or admits C2/optional")


def validate_settled_clone_manifest(policy: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    if set(manifest) != {"schema_version", "materialized", "create_new", "source_unchanged", "fresh_root", "files"}:
        _fail("CLONE_MANIFEST_NOT_CLOSED", "keys differ")
    if manifest["schema_version"] != "smpcc-r8-liquid-s5b0-settled-clone-manifest-v1" or manifest["materialized"] is not True or manifest["create_new"] is not True or manifest["source_unchanged"] is not True or manifest["fresh_root"] is not True:
        _fail("CLONE_LIFECYCLE_INVALID", "clone is not a fresh byte-exact materialization")
    expected = {item["name"]: item["sha256"] for item in policy["settled_clone_contract"]["required_files"]}
    files = manifest["files"]
    if not isinstance(files, list) or {item.get("name") for item in files if isinstance(item, Mapping)} != set(expected):
        _fail("CLONE_REQUIRED_FILE_MISSING", "Part_0901 and Part_Head are both mandatory")
    for item in files:
        if set(item) != {"name", "source_sha256", "clone_sha256", "mode", "nlink"}:
            _fail("CLONE_FILE_NOT_CLOSED", "file row differs")
        digest = expected[item["name"]]
        if item["source_sha256"] != digest or item["clone_sha256"] != digest:
            _fail("CLONE_HASH_DRIFT", item["name"])
        if item["mode"] != "0440" or item["nlink"] != 1:
            _fail("CLONE_IDENTITY_INVALID", item["name"])


def _rotation_zyx(ang1_deg: float, ang2_deg: float, ang3_deg: float) -> tuple[tuple[float, float, float], ...]:
    yaw, pitch, roll = map(math.radians, (ang1_deg, ang2_deg, ang3_deg))
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def validate_solver_path(samples: Sequence[Sequence[object]]) -> list[tuple[float, ...]]:
    if len(samples) < 2:
        _fail("SOLVER_PATH_TOO_SHORT", "at least two rows are required")
    rows: list[tuple[float, ...]] = []
    for row_index, sample in enumerate(samples):
        if len(sample) != 7:
            _fail("SOLVER_PATH_FIELDS", f"row {row_index} does not have seven fields")
        rows.append(tuple(_finite(value, f"row {row_index}") for value in sample))
    if any(abs(value) > 1e-12 for value in rows[0]):
        _fail("SOLVER_PATH_INITIAL_NOT_IDENTITY", "first row must be t=0 and identity")
    if any(right[0] <= left[0] for left, right in zip(rows, rows[1:])):
        _fail("SOLVER_PATH_TIME_NOT_STRICT", "time must be strictly increasing")
    return rows


def estimate_moving_domain(policy: Mapping[str, Any], samples: Sequence[Sequence[object]]) -> dict[str, Any]:
    rows = validate_solver_path(samples)
    radius = policy["geometry"]["container_radius_m"]
    height = policy["geometry"]["boundary_height_m"]
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    cumulative = 0.0
    for index, row in enumerate(rows):
        _, x, y, z, ang1, ang2, ang3 = row
        if index:
            previous = rows[index - 1]
            cumulative += math.dist((previous[1], previous[2], previous[3]), (x, y, z))
        rotation = _rotation_zyx(ang1, ang2, ang3)
        translation = (x, y, z)
        for axis in range(3):
            radial = radius * math.hypot(rotation[axis][0], rotation[axis][1])
            axial = rotation[axis][2] * height
            lower[axis] = min(lower[axis], translation[axis] + min(0.0, axial) - radial)
            upper[axis] = max(upper[axis], translation[axis] + max(0.0, axial) + radial)
    budget = policy["moving_domain_budget"]
    margin = budget["margin_m"]
    lower = [value - margin for value in lower]
    upper = [value + margin for value in upper]
    spans = [high - low for low, high in zip(lower, upper)]
    if any(span > budget["maximum_axis_span_m"] for span in spans):
        _fail("MOVING_DOMAIN_AXIS_SPAN", f"swept span exceeds {budget['maximum_axis_span_m']} m")
    cells = [max(1, math.ceil(span / budget["cell_size_m"])) for span in spans]
    total_cells = math.prod(cells)
    bits = sum(max(1, math.ceil(math.log2(count + 1))) for count in cells)
    gpu_bytes = budget["fixed_gpu_bytes"] + total_cells * budget["bytes_per_cell"] + policy["settled_clone_contract"]["particle_count"] * budget["bytes_per_particle"]
    if total_cells > budget["maximum_cells"] or bits > budget["maximum_cell_code_bits"] or gpu_bytes > budget["maximum_gpu_bytes"]:
        _fail("MOVING_DOMAIN_BUDGET", "cell-grid/cell-code/VRAM budget exceeded")
    physical_seconds = rows[-1][0] + policy["mvpathfile_contract"]["solver_tail_s"]
    runtime = policy["runtime_budget"]
    output_bytes = math.ceil(runtime["fixed_output_bytes"] + physical_seconds * runtime["output_bytes_per_physical_second"])
    if output_bytes > runtime["maximum_output_bytes"]:
        _fail("OUTPUT_BUDGET", "complete motion and tail exceed disk budget")
    return {
        "bbox_min_m": lower,
        "bbox_max_m": upper,
        "axis_spans_m": spans,
        "cell_counts": cells,
        "total_cells": total_cells,
        "cell_code_bits": bits,
        "estimated_gpu_bytes": gpu_bytes,
        "estimated_output_bytes": output_bytes,
        "motion_duration_s": rows[-1][0],
        "solver_tail_s": policy["mvpathfile_contract"]["solver_tail_s"],
        "cumulative_translation_m": cumulative,
    }


def _number(value: float) -> str:
    return format(value, ".15g")


def generate_dry_case_plan(policy: Mapping[str, Any], samples: Sequence[Sequence[object]], *, solver_tail_s: float) -> dict[str, Any]:
    required_tail = policy["mvpathfile_contract"]["solver_tail_s"]
    if _finite(solver_tail_s, "solver_tail_s") != required_tail:
        _fail("SOLVER_TAIL_TRUNCATED", f"tail must be exactly {required_tail} s")
    estimate = estimate_moving_domain(policy, samples)
    motion = policy["mvpathfile_contract"]
    duration = estimate["motion_duration_s"]
    block = "\n".join([
        "<motion>",
        "  <objreal ref=\"0\">",
        f"    <begin mov=\"1\" start=\"{_number(motion['begin_start_s'])}\" />",
        f"    <mvpathfile id=\"1\" duration=\"{_number(duration)}\" next=\"2\" anglesunits=\"degrees\">",
        "      <file name=\"solver_path.csv\" fields=\"7\" fieldtime=\"0\" fieldx=\"1\" fieldy=\"2\" fieldz=\"3\" fieldang1=\"4\" fieldang2=\"5\" fieldang3=\"6\" />",
        "      <center x=\"0\" y=\"0\" z=\"0\" />",
        "      <movecenter value=\"true\" />",
        "      <intrinsic value=\"true\" />",
        "      <axes value=\"ZYX\" />",
        "    </mvpathfile>",
        "    <mvnull id=\"2\" />",
        "  </objreal>",
        "</motion>",
    ])
    tmax = motion["begin_start_s"] + duration + required_tail
    return {
        "mode": "DRY_STATIC_ONLY",
        "files_written": False,
        "motion_blocks": [block, block],
        "motion_block_replacement_count": 2,
        "simulation_domain": {"minimum_m": estimate["bbox_min_m"], "maximum_m": estimate["bbox_max_m"]},
        "solver_argv": ["-partbegin:901:901", "/restart", f"-tmax:{_number(tmax)}", "-tout:0.05"],
        "estimate": estimate,
        "gauge_contract": "GAUGE_ZSURF_ALL_SLOTS_REQUIRED",
        "boundary_qc_contract": "EXECUTED_BOUNDARY_MOTION_REQUIRED",
        "result_inventory": list(policy["result_contract"]["required_inventory"]),
        "gpu_exposed": False,
        "solver_executed": False,
        "profile_loaded": False,
    }


def build_static_receipt(policy: Mapping[str, Any], policy_sha: str, schema_sha: str, profile_sha: str) -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-s5b0-primary-replay-static-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S5B0_PRIMARY_REPLAY_STATIC_RECEIPT_V1",
        "receipt_id": "liquid_zrj_msi_u2404_s5b0_primary_replay_static_v1_receipt",
        "status": "NOT_ADMITTED_PARENT_TRANSFER_NOT_MATERIALIZED",
        "policy": {
            "policy_id": EXPECTED_POLICY_ID, "path": str(POLICY_PATH), "sha256": policy_sha,
            "schema_path": str(SCHEMA_PATH), "schema_sha256": schema_sha,
            "profile_template_path": policy["profile_contract"]["template_path"],
            "profile_template_sha256": profile_sha,
        },
        "parent_transfer": dict(policy["parent_transfer"]),
        "selection": {key: policy["selection"][key] for key in ("attempt_id", "role", "container", "planned_denominator", "optional_authorized", "c2_authorized")},
        "checks": {
            "policy_valid": True, "schema_deep_closed": True, "profile_template_valid": True,
            "frozen_parent_identities_present": True, "transfer_materialized": False,
            "compatibility_execution_receipt_verified": False, "settled_clone_verified": False,
            "moving_domain_budget_verified": False, "mvpathfile_case_generated": False,
            "runtime_profile_instantiated": False, "result_contract_executed": False,
        },
        "estimates": None,
        "safety": {
            "real_bag_read": False, "external_root_created": False, "gpu_exposed": False,
            "solver_executed": False, "sudo_used": False, "bwrap_used": False,
            "profile_queried": False, "profile_loaded": False, "files_written": False,
        },
        "admission": {
            "admitted": False, "gpu_replay_authorized": False, "profile_load_authorized": False,
            "reason": "EXACT_S5A1_TRANSFER_AND_CREATE_NEW_EXECUTION_AUTHORIZATION_REQUIRED",
            "claim_ceiling": "DEVELOPMENT_ONLY_NOT_EXECUTED",
        },
        "next": policy["next"],
    }


def self_check() -> dict[str, Any]:
    policy, policy_sha = _read_json(POLICY_PATH)
    schema, schema_sha = _read_json(SCHEMA_PATH)
    validate_policy(policy)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    profile_path = Path(policy["profile_contract"]["template_path"])
    profile_raw = _read_static(profile_path)
    query_profile_template(policy, profile_raw.decode("utf-8"))
    for name in ("compatibility_contract", "compatibility_gate"):
        identity = policy["frozen_parents"][name]
        observed = {"path": identity["path"], "sha256": hashlib.sha256(_read_static(Path(identity["path"]))).hexdigest()}
        validate_parent_identity(policy, name, observed)
    receipt = build_static_receipt(policy, policy_sha, schema_sha, hashlib.sha256(profile_raw).hexdigest())
    errors = list(Draft202012Validator(schema).iter_errors(receipt))
    if errors:
        _fail("STATIC_RECEIPT_SCHEMA", errors[0].message)
    return receipt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("profile-query")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            result = self_check()
        else:
            policy, _ = _read_json(POLICY_PATH)
            validate_policy(policy)
            raw = _read_static(Path(policy["profile_contract"]["template_path"]))
            result = query_profile_template(policy, raw.decode("utf-8"))
    except (S5B0StaticError, OSError, UnicodeError, ValueError) as exc:
        print(json.dumps({"status": "FAIL_STATIC_CLOSED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
