#!/usr/bin/env python3
"""Static-only S5B0 v4 admission for a future motion-attached Gauge candidate."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v4.json"
POLICY_SCHEMA = ROOT / "schema/target_host_s5b0_replay_admission_policy_v4.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5b0_replay_static_receipt_v4.json"
PROFILE_GENERATOR = ROOT / "scripts/r8_liquid_s5b0_profile_generator_v4.py"

TRANSFER_MANIFEST = Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v10/transfer_manifest.json")
TRANSFER_CHECKSUMS = TRANSFER_MANIFEST.with_name("checksums.sha256")
SOLVER_PATH = TRANSFER_MANIFEST.with_name("solver_path.csv")
EXECUTION_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v10.execution_v14.json")

G1_G2_FILES = {
    "g1_policy_sha256": ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json",
    "g1_schema_sha256": ROOT / "schema/target_host_u3_gpu_build_execution_policy_v1.json",
    "g1_gate_sha256": ROOT / "scripts/r8_liquid_target_u3_gpu_build_gate_v1.py",
    "g1_tests_sha256": ROOT / "tests/test_target_u3_gpu_build_gate_v1.py",
    "g2_policy_sha256": ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v2.json",
    "g2_receipt_schema_sha256": ROOT / "schema/target_host_u3_gpu_build_execution_receipt_v1.json",
    "g2_gate_sha256": ROOT / "scripts/r8_liquid_target_u3_gpu_build_gate_v2.py",
    "g2_tests_sha256": ROOT / "tests/test_target_u3_gpu_build_gate_v2.py",
}
OLD_CANDIDATE_SHA256 = "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"


class AdmissionV4Error(ValueError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1 << 20):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise AdmissionV4Error(f"not a JSON object: {path}")
    return value, hashlib.sha256(raw).hexdigest()


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise AdmissionV4Error(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise AdmissionV4Error(f"non-finite {label}")
    return float(value)


def validate_static_boundary(policy: Mapping[str, Any]) -> None:
    if policy["status"] != "NOT_ADMITTED_MOTION_ATTACHED_GAUGE_CANDIDATE_REQUIRED":
        raise AdmissionV4Error("v4 static policy was promoted")
    if policy["selection"] != {
        "attempt_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01", "role": "PRIMARY_BASELINE",
        "container": "C1", "planned_denominator": 1, "optional_authorized": False, "c2_authorized": False,
    }:
        raise AdmissionV4Error("primary/optional/C2 selection drifted")
    future = policy["future_candidate"]
    if any(future[name] is not None for name in (
        "candidate_path", "candidate_sha256", "build_receipt_path", "build_receipt_sha256",
        "static_audit_receipt_path", "static_audit_receipt_sha256",
    )):
        raise AdmissionV4Error("future candidate or receipt placeholder was materialized")
    if future["compiler"] != "g++-11" or future["gpp13_allowed"] or future["built"] or future["static_audit_passed"] or future["candidate_execution_authorized"]:
        raise AdmissionV4Error("future candidate state/compile boundary drifted")
    if policy["frozen_parents"]["old_candidate_sha256"] != OLD_CANDIDATE_SHA256 or policy["frozen_parents"]["old_candidate_admitted"]:
        raise AdmissionV4Error("old fixed-Gauge candidate was admitted")
    if any(policy["authorization"].values()):
        raise AdmissionV4Error("v4 repository policy contains runtime authorization")
    if any(value is not None for key, value in policy["lifecycle_placeholders"].items() if key in {
        "replay_id", "stage_root", "partial_root", "final_root", "start_receipt", "final_receipt",
        "failure_receipt", "profile_name", "profile_path", "profile_sha256",
    }):
        raise AdmissionV4Error("runtime lifecycle placeholder was materialized")


def validate_transfer(policy: Mapping[str, Any]) -> None:
    transfer = policy["finalized_transfer"]
    exact = {
        TRANSFER_MANIFEST: transfer["manifest_sha256"],
        TRANSFER_CHECKSUMS: transfer["checksums_sha256"],
        SOLVER_PATH: transfer["solver_path_sha256"],
        EXECUTION_RECEIPT: transfer["execution_receipt_sha256"],
    }
    for path, expected in exact.items():
        if sha256_file(path) != expected:
            raise AdmissionV4Error(f"finalized transfer parent drift: {path}")
    manifest, _ = read_json(TRANSFER_MANIFEST)
    receipt, _ = read_json(EXECUTION_RECEIPT)
    for value in (manifest, receipt):
        if value.get("status") != transfer["required_status"]:
            raise AdmissionV4Error("finalized v10 status differs")
    if manifest.get("transfer_id") != transfer["transfer_id"]:
        raise AdmissionV4Error("finalized v10 transfer id differs")
    if receipt.get("safety", {}).get("optional_bag_read") is not False or receipt.get("safety", {}).get("gpu_exposed") is not False:
        raise AdmissionV4Error("v10 receipt safety boundary differs")
    for key, path in G1_G2_FILES.items():
        if sha256_file(path) != policy["frozen_parents"][key]:
            raise AdmissionV4Error(f"G1/G2 parent drift: {key}")


def solver_rows(path: Path = SOLVER_PATH) -> tuple[tuple[float, ...], ...]:
    rows: list[tuple[float, ...]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) != 7:
                raise AdmissionV4Error("solver path field count differs")
            rows.append(tuple(_finite(float(item), "solver path") for item in row))
    if len(rows) != 990 or any(abs(value) > 1e-12 for value in rows[0]):
        raise AdmissionV4Error("solver path rows/t0 differ")
    if any(right[0] <= left[0] for left, right in zip(rows, rows[1:])):
        raise AdmissionV4Error("solver path time is not strict")
    if abs(rows[-1][0] - rows[-2][0] - 1.0) > 1e-12 or rows[-1][1:] != rows[-2][1:]:
        raise AdmissionV4Error("solver tail differs")
    return tuple(rows)


def _rotation_zyx(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[tuple[float, float, float], ...]:
    yaw, pitch, roll = map(math.radians, (yaw_deg, pitch_deg, roll_deg))
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr), (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def _query_times(rows: Sequence[Sequence[float]]) -> tuple[float, ...]:
    times = {row[0] for row in rows}
    for left, right in zip(rows, rows[1:]):
        for part in range(1, 4):
            times.add(left[0] + (right[0] - left[0]) * part / 4)
    slot = 0
    while slot * 0.05 < rows[-1][0] - 1e-12:
        times.add(slot * 0.05)
        slot += 1
    times.add(rows[-1][0])
    return tuple(sorted(times))


def _interpolate(rows: Sequence[Sequence[float]], time_s: float) -> tuple[float, ...]:
    if time_s <= rows[0][0]:
        return tuple(rows[0])
    for left, right in zip(rows, rows[1:]):
        if time_s <= right[0] + 1e-12:
            ratio = (time_s-left[0])/(right[0]-left[0])
            return (time_s, *(left[index]+ratio*(right[index]-left[index]) for index in range(1, 7)))
    return tuple(rows[-1])


def calculate_envelope(rows: Sequence[Sequence[float]]) -> dict[str, Any]:
    lower, upper = [math.inf]*3, [-math.inf]*3
    times = _query_times(rows)
    for time_s in times:
        _, x, y, z, yaw, pitch, roll = _interpolate(rows, time_s)
        rotation = _rotation_zyx(yaw, pitch, roll)
        for axis, translation in enumerate((x, y, z)):
            radial = 0.0185*math.hypot(rotation[axis][0], rotation[axis][1])
            axial = rotation[axis][2]*0.066
            lower[axis] = min(lower[axis], translation+min(0.0, axial)-radial)
            upper[axis] = max(upper[axis], translation+max(0.0, axial)+radial)
    lower, upper = [value-0.012 for value in lower], [value+0.012 for value in upper]
    spans = [right-left for left, right in zip(lower, upper)]
    counts = [max(1, math.ceil(span/0.004)) for span in spans]
    cells = math.prod(counts)
    estimated_vram = 2147483648 + cells*64 + 9078*2048 + 16*12
    return {"bbox_min_m":lower,"bbox_max_m":upper,"axis_spans_m":spans,"maximum_axis_span_m":max(spans),
            "cell_counts":counts,"total_cells":cells,"query_count":len(times),"estimated_vram_bytes":estimated_vram}


def validate_envelope(policy: Mapping[str, Any], rows: Sequence[Sequence[float]]) -> dict[str, Any]:
    observed = calculate_envelope(rows)
    expected = policy["moving_envelope"]
    if expected["old_maximum_axis_span_m"] >= observed["maximum_axis_span_m"] or expected["old_gate_pass"]:
        raise AdmissionV4Error("legacy 8 m gate was silently promoted")
    for key in ("bbox_min_m", "bbox_max_m", "axis_spans_m"):
        if any(abs(a-b) > 1e-12 for a, b in zip(observed[key], expected[key])):
            raise AdmissionV4Error(f"moving envelope differs: {key}")
    for key in ("cell_counts", "total_cells", "query_count"):
        if observed[key] != expected[key]:
            raise AdmissionV4Error(f"moving envelope differs: {key}")
    if abs(observed["maximum_axis_span_m"]-expected["maximum_axis_span_m"]) > 1e-12:
        raise AdmissionV4Error("8.883 m maximum span differs")
    if observed["estimated_vram_bytes"] != policy["resource_estimate"]["estimated_vram_bytes"]:
        raise AdmissionV4Error("VRAM estimate differs")
    return observed


def validate_gauge_contract(contract: Mapping[str, Any]) -> None:
    if contract["measurement_source"] != "RAW_NATIVE_JGAUGESWL_CSV" or contract["class_name"] != "JGaugeSwl" or not contract["is_solver_gauge"]:
        raise AdmissionV4Error("q98/non-Gauge source cannot satisfy v4")
    if contract["world_fixed_xml_gauge_allowed"] or contract["attachment_frame"] != "MOVING_CONTAINER_REFERENCE_REF_0":
        raise AdmissionV4Error("fixed-world Gauge cannot satisfy v4")
    if contract["q98_proxy_allowed"]:
        raise AdmissionV4Error("q98 proxy was admitted")
    probes = contract["probes"]
    if len(probes) != 16 or len({probe["name"] for probe in probes}) != 16:
        raise AdmissionV4Error("16 Gauge identities are not exact")
    for index, probe in enumerate(probes):
        if probe["name"] != f"s5b0_p{index:02d}" or abs(probe["angle_deg"]-index*22.5) > 1e-12:
            raise AdmissionV4Error("Gauge ordering/angle differs")
        if abs(math.hypot(probe["x_m"], probe["y_m"])-contract["probe_radius_m"]) > 1e-12:
            raise AdmissionV4Error("Gauge probe radius differs")


def load_profile_generator():
    spec = importlib.util.spec_from_file_location("s5b0_profile_v4", PROFILE_GENERATOR)
    if spec is None or spec.loader is None:
        raise AdmissionV4Error("cannot load v4 profile generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_receipt(policy: Mapping[str, Any], policy_sha: str, template_sha: str) -> dict[str, Any]:
    return {
        "schema_version":"smpcc-r8-liquid-s5b0-replay-static-receipt-v4",
        "document_type":"SMPCC_R8_LIQUID_S5B0_REPLAY_STATIC_RECEIPT_V4",
        "status":"NOT_ADMITTED_MOTION_ATTACHED_GAUGE_CANDIDATE_REQUIRED",
        "policy_sha256":policy_sha,
        "transfer":{"finalized":True,"transfer_id":policy["finalized_transfer"]["transfer_id"],"manifest_sha256":policy["finalized_transfer"]["manifest_sha256"],"execution_receipt_sha256":policy["finalized_transfer"]["execution_receipt_sha256"],"solver_path_sha256":policy["finalized_transfer"]["solver_path_sha256"]},
        "candidate":{"old_candidate_admitted":False,"new_candidate_materialized":False,"build_receipt_materialized":False,"static_audit_receipt_materialized":False,"required_capability":"MOTION_ATTACHED_16_RAW_JGAUGESWL"},
        "envelope":{"old_8m_gate_pass":False,"maximum_axis_span_m":8.883036647976011,"total_cells":24264425,"estimated_vram_bytes":3718998784,"dynamic_vram_observed":False},
        "gauge":{"probe_count":16,"source":"RAW_NATIVE_JGAUGESWL_CSV","motion_attached":True,"world_fixed_allowed":False,"q98_allowed":False,"raw_outputs_read":False},
        "profile":{"template_sha256":template_sha,"rendered_only":True,"queried":False,"loaded":False},
        "authorization":dict(policy["authorization"]),
        "claims":{"optional_bag_read":False,"c2_admitted":False,"gpp13_admitted":False,"files_written":False,"solver_executed":False,"gpu_exposed":False,"sudo_used":False,"apparmor_loaded":False},
    }


def self_check() -> dict[str, Any]:
    policy, policy_sha = read_json(POLICY_PATH)
    policy_schema, _ = read_json(POLICY_SCHEMA)
    receipt_schema, _ = read_json(RECEIPT_SCHEMA)
    for schema in (policy_schema, receipt_schema):
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(policy_schema).validate(policy)
    validate_static_boundary(policy)
    validate_transfer(policy)
    rows = solver_rows()
    envelope = validate_envelope(policy, rows)
    validate_gauge_contract(policy["gauge_contract"])
    profile_result = load_profile_generator().self_check()
    receipt = build_receipt(policy, policy_sha, str(profile_result["template_sha256"]))
    Draft202012Validator(receipt_schema).validate(receipt)
    return {**receipt, "schemas_deep_closed":True, "solver_path_rows":len(rows),
            "moving_query_count":envelope["query_count"], "profile_status":profile_result["status"]}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:
        print(json.dumps({"status":"FAIL_S5B0_ADMISSION_V4","error":str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
