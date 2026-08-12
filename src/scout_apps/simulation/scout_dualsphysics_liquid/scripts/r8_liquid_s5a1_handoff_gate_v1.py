#!/usr/bin/env python3
"""Static/export gate for the primary R8-LIQUID-HANDOFF-ABI-v3 package.

The gate consumes already decoded, caller-supplied odometry records and a
sealed S5A0 receipt.  It does not open a ROS bag.  ``self-check`` uses only an
in-memory fixture.  Package publication, when separately authorized, is an
O_EXCL/create-new operation and never overwrites or repairs a partial root.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v1.json"
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a1_handoff_package_v1.json"
S5A0_RECEIPT_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_selected_bag_receipt_v1.json"
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_s5a1_motion_bridge_v1 as bridge  # noqa: E402


MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_ARTIFACT_BYTES = 16 * 1024 * 1024
ODOM_FIELDS = (
    "bag_record_t_ns", "odom_header_t_ns", "frame_id", "child_frame_id",
    "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
    "vx_m_s", "vy_m_s", "vz_m_s", "wx_rad_s", "wy_rad_s", "wz_rad_s",
)
MOTION_FIELDS = (
    "t_s", "x_rel_m", "y_rel_m", "z_rel_m",
    "qx_rel", "qy_rel", "qz_rel", "qw_rel",
)
PACKAGE_INVENTORY = (
    "transfer_manifest.json", "selected_bag_receipt_ref.json", "source_bag_ref.json",
    "source_outcome.json", "topic_contract.json", "time_window.json",
    "frame_contract.json", "odom_raw.csv", "clock_alignment.csv", "tf_alignment.json",
    "motion.csv", "solver_path.csv", "motion_manifest.json", "interpolation_qc.json",
    "solver_path_qc.json", "container_geometry_manifest.json",
    "container_mount_manifest.json", "fluid_spec.json", "run_spec.json",
    "checksums.sha256",
)
DATA_ARTIFACTS = PACKAGE_INVENTORY[1:-1]
CHECKSUMMED_FILES = PACKAGE_INVENTORY[:-1]


class HandoffError(ValueError):
    """Parent drift, bad receipt/records, QC failure, or unsafe publication."""


class ForbiddenSignalConflict(HandoffError):
    """QC-only signal conflicts with odom; motion remains odom-derived only."""

    def __init__(self, topics: Sequence[str], candidate_motion_sha256: str) -> None:
        self.topics = tuple(topics)
        self.candidate_motion_sha256 = candidate_motion_sha256
        super().__init__(
            f"QC_ONLY_SIGNAL_CONFLICT topics={list(topics)} "
            f"candidate_motion_sha256={candidate_motion_sha256}"
        )


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_regular(path: Path, maximum: int = MAX_JSON_BYTES) -> tuple[bytes, dict[str, Any]]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise HandoffError(f"unsafe regular parent: {path}")
        if not 0 < metadata.st_size <= maximum:
            raise HandoffError(f"parent size outside bound: {path}")
        chunks = bytearray()
        while len(chunks) < metadata.st_size:
            block = os.read(descriptor, min(1024 * 1024, metadata.st_size - len(chunks)))
            if not block:
                raise HandoffError(f"short parent read: {path}")
            chunks.extend(block)
    finally:
        os.close(descriptor)
    raw = bytes(chunks)
    return raw, {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}


def read_json(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, identity = read_regular(path)
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as error:
        raise HandoffError(f"invalid JSON: {path}") from error
    if not isinstance(value, dict):
        raise HandoffError(f"JSON root is not an object: {path}")
    return value, identity


def assert_schema_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if kind == "object" and value.get("additionalProperties") is not False:
            raise HandoffError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_schema_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_schema_deep_closed(child, f"{location}/{index}")


def validate_policy() -> tuple[dict[str, Any], dict[str, Any]]:
    policy, policy_identity = read_json(POLICY_PATH)
    expected_top = {
        "schema_version", "document_type", "policy_id", "abi", "transfer_id",
        "source", "parents", "time_window", "motion_contract", "solver_bridge",
        "compatibility", "signals", "package", "claim_ceiling",
    }
    if set(policy) != expected_top:
        raise HandoffError("policy top-level keys drifted")
    if policy["schema_version"] != "smpcc-r8-liquid-s5a1-policy-v1":
        raise HandoffError("policy schema version drifted")
    if policy["abi"] != bridge.ABI_ID or policy["source"]["domain"] != bridge.SOURCE_DOMAIN:
        raise HandoffError("ABI/source domain drifted")
    if tuple(policy["package"]["inventory"]) != PACKAGE_INVENTORY:
        raise HandoffError("package inventory/order drifted")
    source = policy["source"]
    if (source["bag_size_bytes"] != 13996902
            or source["bag_sha256"] != "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"
            or source["s5a0_receipt_identity"] != "NOT_MATERIALIZED"
            or source["optional_pair_read_or_admitted"]
            or source["c2_read_or_admitted"]):
        raise HandoffError("primary/S5A0 admission identity drifted")
    if policy["claim_ceiling"]["admission"] != "NOT_ADMITTED_S5A0_RECEIPT_REQUIRED":
        raise HandoffError("S5A1 was admitted before the S5A0 receipt")
    if (set(policy["signals"]["motion_inputs"]) != bridge.ALLOWED_MOTION_SIGNALS
            or len(policy["signals"]["motion_inputs"]) != len(bridge.ALLOWED_MOTION_SIGNALS)):
        raise HandoffError("closed odom-only signal set drifted")
    if policy["solver_bridge"] != {
        "motion_element": "mvpathfile", "field_count": 7,
        "field_indices": [0, 1, 2, 3, 4, 5, 6], "anglesunits": "degrees",
        "axes": "ZYX", "intrinsic": True, "movecenter": True,
        "rotation_center_m": [0.0, 0.0, 0.0],
        "angle_order": ["continuous_yaw_Z", "pitch_Y", "roll_X"],
        "candidate_executable": False,
    }:
        raise HandoffError("solver bridge drifted")
    for name, expected in policy["parents"].items():
        raw, observed = read_regular(Path(expected["path"]), MAX_ARTIFACT_BYTES)
        del raw
        if observed["sha256"] != expected["sha256"]:
            raise HandoffError(f"frozen parent hash drifted: {name}")
    compatibility, _ = read_json(Path(policy["parents"]["compatibility_contract"]["path"]))
    if compatibility["status"] != policy["compatibility"]["contract_status"]:
        raise HandoffError("compatibility status drifted")
    if compatibility["claim_ceiling"]["admission"] != "S5A1_EXPORT_PRECONDITION_ONLY":
        raise HandoffError("compatibility admission ceiling drifted")
    if compatibility["source_scope"]["optional_pair_read_or_admitted"]:
        raise HandoffError("optional bag was admitted")
    return policy, policy_identity


def validate_s5a0_receipt(raw: bytes, path: str, policy: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, str]]:
    if not 0 < len(raw) <= MAX_JSON_BYTES:
        raise HandoffError("S5A0 receipt size is outside the bound")
    if not path.startswith("/") or "\x00" in path:
        raise HandoffError("S5A0 receipt path is not absolute")
    try:
        receipt = json.loads(raw)
    except json.JSONDecodeError as error:
        raise HandoffError("S5A0 receipt JSON is invalid") from error
    if receipt.get("document_type") == "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V1":
        receipt_schema, _ = read_json(S5A0_RECEIPT_SCHEMA_PATH)
        Draft202012Validator(receipt_schema).validate(receipt)
        source = policy["source"]
        selection = receipt["selection"]
        selected = receipt["source"]["selected_after"]
        if (receipt["status"] != source["required_s5a0_status"]
                or selection["source_domain"] != source["domain"]
                or selection["attempt_id"] != source["attempt_id"]
                or selection["selected_role"] != source["role"]
                or selection["absolute_path"] != source["bag_absolute_path"]
                or selection["relative_path"] != source["bag_relative_path"]
                or selection["optional_pair_read_or_admitted"]
                or selected["path"] != source["bag_absolute_path"]
                or selected["size_bytes"] != source["bag_size_bytes"]
                or selected["mode"] != source["bag_mode"]
                or selected["sha256"] != source["bag_sha256"]
                or receipt["sandbox"]["source_bag_executed"]
                or receipt["source"]["files_written_under_source_root"] != 0
                or not receipt["topic_contract"]["validated"]
                or receipt["bag"]["topic_conflicts"]
                or receipt["bag"]["anomalies"]):
            raise HandoffError("S5A0 receipt failed exact primary/safety/topic binding")
        return receipt, {"path": path, "sha256": sha256_bytes(raw)}
    if not path.startswith("/mock/"):
        raise HandoffError("non-mock S5A0 receipt must use the frozen closed receipt schema")
    required = {
        "status", "source_provenance", "source_bag", "topic_contract",
        "source_bag_executed", "files_written_under_source_root",
    }
    if not isinstance(receipt, dict) or set(receipt) != required:
        raise HandoffError("S5A0 receipt is not the closed expected object")
    source = policy["source"]
    expected_bag = {
        "path": source["bag_absolute_path"], "size_bytes": source["bag_size_bytes"],
        "mode": source["bag_mode"], "sha256": source["bag_sha256"],
    }
    if receipt["status"] != source["required_s5a0_status"]:
        raise HandoffError("S5A0 receipt status is not PASS")
    if receipt["source_provenance"] != "PARTIAL_BAG_ONLY" or receipt["source_bag"] != expected_bag:
        raise HandoffError("S5A0 source identity/provenance drifted")
    if receipt["source_bag_executed"] or receipt["files_written_under_source_root"] != 0:
        raise HandoffError("S5A0 safety evidence failed")
    topic = receipt["topic_contract"]
    if set(topic) != {"schema", "odom_message_count", "required_topics_pass", "conflicts"}:
        raise HandoffError("S5A0 topic contract is open or incomplete")
    if topic["schema"] != "SPMPC_NON_FIXED" or not topic["required_topics_pass"] or topic["conflicts"]:
        raise HandoffError("S5A0 topic contract did not pass")
    if not isinstance(topic["odom_message_count"], int) or topic["odom_message_count"] < 2:
        raise HandoffError("S5A0 odom count is invalid")
    return receipt, {"path": path, "sha256": sha256_bytes(raw)}


def s5a0_odom_message_count(receipt: Mapping[str, Any]) -> int:
    if receipt.get("document_type") == "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V1":
        value = receipt["bag"]["odom"]["message_count"]
    else:
        value = receipt["topic_contract"]["odom_message_count"]
    if not isinstance(value, int) or isinstance(value, bool) or value < 2:
        raise HandoffError("S5A0 odom count is invalid")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise HandoffError(f"non-finite/invalid numeric field: {label}")
    return float(value)


def validate_odom_records(records: Sequence[Mapping[str, Any]], policy: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    if len(records) < 2:
        raise HandoffError("fewer than two odom records")
    checked = []
    previous_header: int | None = None
    for index, record in enumerate(records):
        if set(record) != set(ODOM_FIELDS):
            raise HandoffError(f"odom record {index} fields are not closed")
        row = dict(record)
        for name in ODOM_FIELDS[:2]:
            if isinstance(row[name], bool) or not isinstance(row[name], int) or row[name] < 0:
                raise HandoffError(f"odom record {index} has invalid {name}")
        if previous_header is not None:
            if row["odom_header_t_ns"] == previous_header:
                raise HandoffError("duplicate odom header time")
            if row["odom_header_t_ns"] < previous_header:
                raise HandoffError("odom header time backtrack")
            gap = (row["odom_header_t_ns"] - previous_header) / 1e9
            if gap > policy["time_window"]["maximum_odom_gap_s"]:
                raise HandoffError("odom header gap exceeds frozen maximum")
        previous_header = row["odom_header_t_ns"]
        if abs(row["bag_record_t_ns"] - row["odom_header_t_ns"]) / 1e9 > policy["time_window"]["maximum_header_record_delta_s"]:
            raise HandoffError("header/record alignment exceeds frozen tolerance")
        if not isinstance(row["frame_id"], str) or not isinstance(row["child_frame_id"], str):
            raise HandoffError("odom frame is invalid")
        for name in ODOM_FIELDS[4:]:
            row[name] = _number(row[name], f"odom[{index}].{name}")
        checked.append(row)
    frames = {(row["frame_id"], row["child_frame_id"]) for row in checked}
    if frames != {("odom", "base_footprint")}:
        raise HandoffError(f"odom frame contract differs: {sorted(frames)}")
    return tuple(checked)


def derive_motion(records: Sequence[Mapping[str, Any]], policy: Mapping[str, Any]) -> tuple[bridge.MotionSample, ...]:
    checked = validate_odom_records(records, policy)
    poses = tuple(
        bridge.OdomPose(row["odom_header_t_ns"] / 1e9, row["x_m"], row["y_m"], row["z_m"],
                        row["qx"], row["qy"], row["qz"], row["qw"])
        for row in checked
    )
    mount = policy["compatibility"]
    raw_motion = bridge.make_relative_motion(
        poses, maximum_gap_s=policy["time_window"]["maximum_odom_gap_s"],
        base_to_container=bridge.RigidPose(*mount["mount_translation_m"], *mount["mount_quaternion_xyzw"]),
    )
    period = policy["time_window"]["interpolation_period_s"]
    duration = raw_motion[-1].t_s
    count = int(math.floor(duration / period + 1e-12))
    grid = [index * period for index in range(count + 1)]
    if duration - grid[-1] > 1e-12:
        grid.append(duration)
    else:
        grid[-1] = duration
    return bridge.resample_motion(raw_motion, tuple(grid))


def validate_alignment_witnesses(
    records: Sequence[Mapping[str, Any]],
    clock_alignment: Sequence[Mapping[str, int]],
    tf_alignment: Mapping[str, Any],
    policy: Mapping[str, Any],
) -> tuple[tuple[dict[str, int], ...], dict[str, Any]]:
    if len(clock_alignment) != len(records):
        raise HandoffError("clock alignment count differs from odom")
    checked_clock: list[dict[str, int]] = []
    maximum_delta_ns = int(policy["time_window"]["maximum_header_clock_delta_s"] * 1e9)
    for index, (odom, witness) in enumerate(zip(records, clock_alignment)):
        if set(witness) != {"odom_header_t_ns", "nearest_clock_t_ns", "abs_delta_ns"}:
            raise HandoffError(f"clock alignment row {index} is not closed")
        if any(isinstance(witness[name], bool) or not isinstance(witness[name], int) or witness[name] < 0
               for name in witness):
            raise HandoffError(f"clock alignment row {index} is invalid")
        expected_delta = abs(witness["odom_header_t_ns"] - witness["nearest_clock_t_ns"])
        if witness["odom_header_t_ns"] != odom["odom_header_t_ns"] or witness["abs_delta_ns"] != expected_delta:
            raise HandoffError(f"clock alignment row {index} does not bind odom")
        if expected_delta > maximum_delta_ns:
            raise HandoffError(f"clock alignment row {index} exceeds frozen tolerance")
        checked_clock.append(dict(witness))
    expected_tf = {
        "status": "PASS_ALIGNMENT_WITNESS",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT",
        "tf_used_as_motion": False,
    }
    if dict(tf_alignment) != expected_tf:
        raise HandoffError("TF alignment witness drifted or was used as motion")
    return tuple(checked_clock), expected_tf


def render_csv(header: Sequence[str], rows: Sequence[Sequence[Any]]) -> bytes:
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(header)
    for row in rows:
        writer.writerow(format(value, ".17g") if isinstance(value, float) else value for value in row)
    return output.getvalue().encode("utf-8")


def _motion_csv(motion: Sequence[bridge.MotionSample]) -> bytes:
    return render_csv(MOTION_FIELDS, [
        (row.t_s, row.x_rel_m, row.y_rel_m, row.z_rel_m,
         row.qx_rel, row.qy_rel, row.qz_rel, row.qw_rel) for row in motion
    ])


def _motion_metrics(motion: Sequence[bridge.MotionSample]) -> dict[str, Any]:
    positions = [(row.x_rel_m, row.y_rel_m, row.z_rel_m) for row in motion]
    bbox_min = [min(position[axis] for position in positions) for axis in range(3)]
    bbox_max = [max(position[axis] for position in positions) for axis in range(3)]
    displacements = [math.sqrt(sum(value * value for value in position)) for position in positions]
    length = sum(
        math.sqrt(sum((right[axis] - left[axis]) ** 2 for axis in range(3)))
        for left, right in zip(positions, positions[1:])
    )
    return {"sample_count": len(motion), "first_pose_identity": True, "strict_time": True,
            "quaternion_normalized": True, "quaternion_sign_continuous": True,
            "bbox_min_m": bbox_min, "bbox_max_m": bbox_max,
            "maximum_displacement_m": max(displacements), "trajectory_length_m": length,
            "forbidden_signal_consumed": False}


def complete_roundtrip_query_grid(
    motion: Sequence[bridge.MotionSample], subdivisions: int
) -> tuple[float, ...]:
    if isinstance(subdivisions, bool) or not isinstance(subdivisions, int) or subdivisions < 2:
        raise HandoffError("round-trip subdivisions are invalid")
    queries: list[float] = []
    for left, right in zip(motion, motion[1:]):
        span = right.t_s - left.t_s
        queries.extend(left.t_s + span * index / subdivisions for index in range(subdivisions))
    queries.append(motion[-1].t_s)
    return tuple(queries)


def build_package(
    *, policy: Mapping[str, Any], policy_identity: Mapping[str, Any],
    s5a0_receipt_raw: bytes, s5a0_receipt_path: str,
    odom_records: Sequence[Mapping[str, Any]],
    clock_alignment: Sequence[Mapping[str, int]], tf_alignment: Mapping[str, Any],
    qc_conflict_topics: Sequence[str] = (),
    qc_only_witness_counts: Mapping[str, int] | None = None,
    real_bag_read: bool = False,
) -> dict[str, bytes]:
    receipt, receipt_identity = validate_s5a0_receipt(s5a0_receipt_raw, s5a0_receipt_path, policy)
    if s5a0_odom_message_count(receipt) != len(odom_records):
        raise HandoffError("S5A0 odom count differs from supplied records")
    checked_odom = validate_odom_records(odom_records, policy)
    checked_clock, checked_tf = validate_alignment_witnesses(
        checked_odom, clock_alignment, tf_alignment, policy
    )
    motion = derive_motion(odom_records, policy)
    motion_csv = _motion_csv(motion)
    bridge.validate_odom_only_provenance(
        primary_time_source=policy["time_window"]["primary_time_source"],
        pose_source=bridge.POSE_SOURCE,
        consumed_motion_signals=policy["signals"]["motion_inputs"],
    )
    unknown_conflicts = sorted(set(qc_conflict_topics) - set(policy["signals"]["qc_only"]))
    if unknown_conflicts:
        raise HandoffError(f"unregistered conflict topic: {unknown_conflicts}")
    if qc_conflict_topics:
        raise ForbiddenSignalConflict(sorted(set(qc_conflict_topics)), sha256_bytes(motion_csv))
    witness_counts = dict(qc_only_witness_counts or {})
    if (set(witness_counts) - set(policy["signals"]["forbidden_motion_inputs"])
            or any(isinstance(value, bool) or not isinstance(value, int) or value < 0
                   for value in witness_counts.values())):
        raise HandoffError("QC-only witness counts are not closed nonnegative integers")

    rows = bridge.build_solver_path(
        motion, gimbal_lock_cos_threshold=policy["motion_contract"]["gimbal_lock_cos_threshold"]
    )
    subdivisions = policy["motion_contract"]["roundtrip_subdivisions_per_interval"]
    if (policy["motion_contract"]["roundtrip_query_grid"] != "EVERY_PATH_INTERVAL_QUARTERS"
            or subdivisions != 4):
        raise HandoffError("round-trip query grid contract drifted")
    query_grid = complete_roundtrip_query_grid(motion, subdivisions)
    round_trip = bridge.evaluate_solver_round_trip(motion, rows, query_times_s=query_grid)
    if round_trip.max_position_error_m > policy["motion_contract"]["maximum_roundtrip_position_error_m"]:
        raise HandoffError("solver-path position round-trip exceeds threshold")
    if max(round_trip.max_quaternion_angular_distance_rad,
           round_trip.max_rotation_matrix_angular_distance_rad) > policy["motion_contract"]["maximum_roundtrip_angular_error_rad"]:
        raise HandoffError("solver-path angular round-trip exceeds threshold")
    tail = policy["time_window"]["solver_relaxation_tail_s"]
    final = rows[-1]
    solver_rows = rows + (bridge.SolverPathRow(final.t_s + tail, *final.numeric_fields()[1:]),)
    solver_csv = bridge.render_solver_path_csv(solver_rows).encode("utf-8")

    start_ns, end_ns = checked_odom[0]["odom_header_t_ns"], checked_odom[-1]["odom_header_t_ns"]
    first_effective = None
    origin = motion[0]
    for sample in motion[1:]:
        displacement = math.sqrt(sample.x_rel_m ** 2 + sample.y_rel_m ** 2 + sample.z_rel_m ** 2)
        angle = math.degrees(bridge.quaternion_angular_distance(
            (origin.qx_rel, origin.qy_rel, origin.qz_rel, origin.qw_rel),
            (sample.qx_rel, sample.qy_rel, sample.qz_rel, sample.qw_rel)))
        if displacement >= policy["time_window"]["first_effective_translation_m"] or angle >= policy["time_window"]["first_effective_rotation_deg"]:
            first_effective = sample.t_s
            break
    if first_effective is None:
        raise HandoffError("first-effective-motion was not found")
    if first_effective + 1e-12 < policy["time_window"]["pre_roll_s"]:
        raise HandoffError("frozen pre-roll is not present")

    source = policy["source"]
    compatibility = policy["compatibility"]
    window = {"start_header_t_ns": start_ns, "end_header_t_ns": end_ns,
              "duration_s": (end_ns - start_ns) / 1e9, "sample_count": len(motion),
              "first_effective_t_s": first_effective,
              "pre_roll_s": policy["time_window"]["pre_roll_s"],
              "terminal_provenance": policy["time_window"]["terminal_provenance"],
              "recorded_tail_rule": policy["time_window"]["recorded_tail_rule"],
              "solver_relaxation_tail_s": tail}
    motion_qc = _motion_metrics(motion)
    solver_qc = {"row_count": len(solver_rows), "field_count": 7,
                 "first_row_identity": True, "gimbal_lock": False, "nonfinite": False,
                 "query_grid": "EVERY_PATH_INTERVAL_QUARTERS",
                 "subdivisions_per_interval": subdivisions,
                 "query_count": len(query_grid),
                 "max_position_error_m": round_trip.max_position_error_m,
                 "max_quaternion_angular_distance_rad": round_trip.max_quaternion_angular_distance_rad,
                 "max_rotation_matrix_angular_distance_rad": round_trip.max_rotation_matrix_angular_distance_rad,
                 "pass": True}

    data: dict[str, bytes] = {}
    def put_json(name: str, value: Any) -> None:
        data[name] = canonical_json(value)
    put_json("selected_bag_receipt_ref.json", {**receipt_identity, "status": receipt["status"]})
    put_json("source_bag_ref.json", {"attempt_id": source["attempt_id"], "role": source["role"],
             "relative_path": source["bag_relative_path"], "absolute_path": source["bag_absolute_path"],
             "size_bytes": source["bag_size_bytes"], "sha256": source["bag_sha256"],
             "selected_bag_receipt_sha256": receipt_identity["sha256"], "bag_copied": False})
    put_json("source_outcome.json", {"source_outcome": "UNKNOWN", "source_provenance": "PARTIAL_BAG_ONLY",
             "outcome_recomputed_from_topics": False, "do_not_override_r7_release": True})
    put_json("topic_contract.json", {"schema": "SPMPC_NON_FIXED", "odom_message_count": len(checked_odom),
             "selected_bag_receipt_sha256": receipt_identity["sha256"],
             "motion_inputs": policy["signals"]["motion_inputs"],
             "qc_only": policy["signals"]["qc_only"], "forbidden_motion_inputs": policy["signals"]["forbidden_motion_inputs"],
             "qc_only_witness_counts": witness_counts, "forbidden_signal_consumed": False})
    put_json("time_window.json", {**policy["time_window"], **window})
    put_json("frame_contract.json", {"odom_frame": "odom", "base_frame": "base_footprint",
             "container_frame": "c1m_container_reference", "relative_transform": policy["motion_contract"]["relative_transform"],
             "first_pose": "IDENTITY"})
    data["odom_raw.csv"] = render_csv(ODOM_FIELDS, [[row[name] for name in ODOM_FIELDS] for row in checked_odom])
    data["clock_alignment.csv"] = render_csv(
        ("odom_header_t_ns", "nearest_clock_t_ns", "abs_delta_ns"),
        [(item["odom_header_t_ns"], item["nearest_clock_t_ns"], item["abs_delta_ns"]) for item in checked_clock])
    put_json("tf_alignment.json", checked_tf)
    data["motion.csv"] = motion_csv
    data["solver_path.csv"] = solver_csv
    put_json("motion_manifest.json", {**motion_qc, "canonical_orientation": "NORMALIZED_QUATERNION_SLERP",
             "position_interpolation": "LINEAR", "source": "/odom.header.stamp+/odom.pose.pose"})
    put_json("interpolation_qc.json", {"grid_period_s": policy["time_window"]["interpolation_period_s"],
             "sample_count": len(motion), "strict_time": True, "maximum_source_gap_s": policy["time_window"]["maximum_odom_gap_s"],
             "smoothing": False, "denoising": False, "path_projection": False, "pass": True})
    put_json("solver_path_qc.json", solver_qc)
    put_json("container_geometry_manifest.json", {"container": "C1", "replay_geometry": "C1M",
             "development_compatibility": True, "radius_m": compatibility["container_radius_m"],
             "particle_spacing_m": compatibility["particle_spacing_m"], "particle_count": compatibility["particle_count"],
             "moving_boundary_count": compatibility["moving_boundary_count"],
             "moving_boundary_position_sha256": compatibility["moving_boundary_position_sha256"],
             "compatibility_contract_sha256": policy["parents"]["compatibility_contract"]["sha256"]})
    put_json("container_mount_manifest.json", {"parent_frame": "base_footprint", "child_frame": "c1m_container_reference",
             "translation_m": compatibility["mount_translation_m"], "quaternion_xyzw": compatibility["mount_quaternion_xyzw"],
             "rotation_center_m": policy["solver_bridge"]["rotation_center_m"], "development_assumption": True,
             "physical_mount_validated": False})
    put_json("fluid_spec.json", {"container": "C1", "liquid_height_m": compatibility["liquid_height_m"],
             "density_kg_m3": compatibility["density_kg_m3"], "particle_count": compatibility["particle_count"],
             "fluid_count": compatibility["fluid_count"], "settled_part_index": compatibility["settled_part_index"],
             "settled_checkpoint_sha256": compatibility["settled_checkpoint_sha256"]})
    put_json("run_spec.json", {"motion_element": "mvpathfile", "solver_path": "solver_path.csv", "field_count": 7,
             "anglesunits": "degrees", "axes": "ZYX", "intrinsic": True, "movecenter": True,
             "recorded_motion_end_s": motion[-1].t_s, "solver_relaxation_tail_s": tail,
             "tail_mode": "FINAL_POSE_ZERO_HOLD", "solver_execution_authorized": False})
    if tuple(data) != DATA_ARTIFACTS:
        raise HandoffError("internal data-artifact order/inventory differs")

    artifact_identities = [{"name": name, "size_bytes": len(data[name]), "sha256": sha256_bytes(data[name])}
                           for name in DATA_ARTIFACTS]
    manifest = {
        "schema_version": "smpcc-r8-liquid-s5a1-package-v1",
        "document_type": "SMPCC_R8_LIQUID_S5A1_TRANSFER_MANIFEST_V1",
        "transfer_id": policy["transfer_id"], "abi": policy["abi"],
        "status": "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED",
        "claim_ceiling": {"development_only": True, "formal_eligible": False,
                          "physical_primary_eligible": False, "gpu_replay_authorized": False},
        "parents": {"policy": {"path": str(POLICY_PATH), "sha256": policy_identity["sha256"]},
                    "s5a0_receipt": receipt_identity,
                    "compatibility_contract": policy["parents"]["compatibility_contract"],
                    "motion_bridge": policy["parents"]["motion_bridge"],
                    "settled_checkpoint": policy["parents"]["settled_checkpoint"]},
        "source": {"domain": source["domain"], "attempt_id": source["attempt_id"], "role": source["role"],
                   "source_provenance": "PARTIAL_BAG_ONLY", "source_outcome": "UNKNOWN", "bag_path": source["bag_absolute_path"],
                   "bag_size_bytes": source["bag_size_bytes"], "bag_sha256": source["bag_sha256"],
                   "receipt_status": source["required_s5a0_status"]},
        "contract": {"primary_time_source": "/odom.header.stamp", "position_interpolation": "LINEAR",
                     "orientation_interpolation": "NORMALIZED_QUATERNION_SLERP",
                     "euler_convention": "INTRINSIC_ZYX_CONTINUOUS_YAW_PITCH_ROLL_DEGREES",
                     "solver_field_count": 7, "compatibility_status": compatibility["contract_status"]},
        "package": {"inventory": list(PACKAGE_INVENTORY), "checksummed_files": list(CHECKSUMMED_FILES),
                    "artifacts": artifact_identities, "file_mode": "0600", "create_new": True},
        "window": window, "motion_qc": motion_qc, "solver_path_qc": solver_qc,
        "safety": {"real_bag_read": bool(real_bag_read), "optional_bag_read": False, "source_written": False,
                   "solver_executed": False, "gpu_exposed": False, "sudo_used": False, "network_used": False},
        "next": "S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION",
    }
    schema, _ = read_json(SCHEMA_PATH)
    Draft202012Validator(schema).validate(manifest)
    complete = {"transfer_manifest.json": canonical_json(manifest), **data}
    checksums = "".join(f"{sha256_bytes(complete[name])}  {name}\n" for name in CHECKSUMMED_FILES).encode("ascii")
    complete["checksums.sha256"] = checksums
    if tuple(complete) != PACKAGE_INVENTORY:
        raise HandoffError("final package inventory/order differs")
    validate_package_bytes(complete, schema)
    return complete


def validate_package_bytes(package: Mapping[str, bytes], schema: Mapping[str, Any] | None = None) -> None:
    if tuple(package) != PACKAGE_INVENTORY or set(package) != set(PACKAGE_INVENTORY):
        raise HandoffError("package inventory is not exact")
    for name, raw in package.items():
        if not isinstance(raw, bytes) or not 0 < len(raw) <= MAX_ARTIFACT_BYTES:
            raise HandoffError(f"artifact bytes invalid: {name}")
    manifest = json.loads(package["transfer_manifest.json"])
    selected_schema = schema if schema is not None else read_json(SCHEMA_PATH)[0]
    Draft202012Validator(selected_schema).validate(manifest)
    expected_artifacts = {item["name"]: (item["size_bytes"], item["sha256"])
                          for item in manifest["package"]["artifacts"]}
    if set(expected_artifacts) != set(DATA_ARTIFACTS):
        raise HandoffError("manifest data inventory differs")
    for name in DATA_ARTIFACTS:
        if expected_artifacts[name] != (len(package[name]), sha256_bytes(package[name])):
            raise HandoffError(f"manifest artifact identity differs: {name}")
    expected_checksums = "".join(f"{sha256_bytes(package[name])}  {name}\n" for name in CHECKSUMMED_FILES).encode("ascii")
    if package["checksums.sha256"] != expected_checksums:
        raise HandoffError("checksums.sha256 differs")
    validate_companion_semantics(package, manifest)


def _closed_json_artifact(package: Mapping[str, bytes], name: str, keys: set[str]) -> dict[str, Any]:
    try:
        value = json.loads(package[name])
    except json.JSONDecodeError as error:
        raise HandoffError(f"invalid companion JSON: {name}") from error
    if not isinstance(value, dict) or set(value) != keys:
        raise HandoffError(f"companion JSON is not closed: {name}")
    return value


def _csv_rows(raw: bytes, name: str) -> list[list[str]]:
    try:
        text = raw.decode("utf-8")
        rows = list(csv.reader(io.StringIO(text, newline=""), strict=True))
    except (UnicodeError, csv.Error) as error:
        raise HandoffError(f"invalid companion CSV: {name}") from error
    if not rows or any(not row for row in rows):
        raise HandoffError(f"empty companion CSV row: {name}")
    return rows


def validate_companion_semantics(package: Mapping[str, bytes], manifest: Mapping[str, Any]) -> None:
    receipt_ref = _closed_json_artifact(
        package, "selected_bag_receipt_ref.json", {"path", "sha256", "status"}
    )
    source_ref = _closed_json_artifact(
        package, "source_bag_ref.json",
        {"attempt_id", "role", "relative_path", "absolute_path", "size_bytes", "sha256",
         "selected_bag_receipt_sha256", "bag_copied"},
    )
    outcome = _closed_json_artifact(
        package, "source_outcome.json",
        {"source_outcome", "source_provenance", "outcome_recomputed_from_topics", "do_not_override_r7_release"},
    )
    topic = _closed_json_artifact(
        package, "topic_contract.json",
        {"schema", "odom_message_count", "selected_bag_receipt_sha256", "motion_inputs", "qc_only",
         "forbidden_motion_inputs", "qc_only_witness_counts", "forbidden_signal_consumed"},
    )
    time_window = _closed_json_artifact(
        package, "time_window.json",
        {"primary_time_source", "bag_record_time_role", "clock_role", "maximum_odom_gap_s",
         "maximum_header_record_delta_s", "maximum_header_clock_delta_s", "interpolation_period_s",
         "first_effective_translation_m", "first_effective_rotation_deg", "pre_roll_s",
         "terminal_provenance", "recorded_tail_rule", "solver_relaxation_tail_s",
         "solver_relaxation_tail_mode", "start_header_t_ns", "end_header_t_ns", "duration_s",
         "sample_count", "first_effective_t_s"},
    )
    frame = _closed_json_artifact(
        package, "frame_contract.json",
        {"odom_frame", "base_frame", "container_frame", "relative_transform", "first_pose"},
    )
    tf = _closed_json_artifact(
        package, "tf_alignment.json",
        {"status", "odom_frame", "base_frame", "container_mount_source", "tf_used_as_motion"},
    )
    motion_manifest = _closed_json_artifact(
        package, "motion_manifest.json",
        {"sample_count", "first_pose_identity", "strict_time", "quaternion_normalized",
         "quaternion_sign_continuous", "bbox_min_m", "bbox_max_m", "maximum_displacement_m",
         "trajectory_length_m", "forbidden_signal_consumed", "canonical_orientation",
         "position_interpolation", "source"},
    )
    _closed_json_artifact(
        package, "interpolation_qc.json",
        {"grid_period_s", "sample_count", "strict_time", "maximum_source_gap_s", "smoothing",
         "denoising", "path_projection", "pass"},
    )
    solver_qc = _closed_json_artifact(
        package, "solver_path_qc.json",
        {"row_count", "field_count", "first_row_identity", "gimbal_lock", "nonfinite",
         "query_grid", "subdivisions_per_interval", "query_count",
         "max_position_error_m", "max_quaternion_angular_distance_rad",
         "max_rotation_matrix_angular_distance_rad", "pass"},
    )
    geometry = _closed_json_artifact(
        package, "container_geometry_manifest.json",
        {"container", "replay_geometry", "development_compatibility", "radius_m", "particle_spacing_m",
         "particle_count", "moving_boundary_count", "moving_boundary_position_sha256",
         "compatibility_contract_sha256"},
    )
    mount = _closed_json_artifact(
        package, "container_mount_manifest.json",
        {"parent_frame", "child_frame", "translation_m", "quaternion_xyzw", "rotation_center_m",
         "development_assumption", "physical_mount_validated"},
    )
    fluid = _closed_json_artifact(
        package, "fluid_spec.json",
        {"container", "liquid_height_m", "density_kg_m3", "particle_count", "fluid_count",
         "settled_part_index", "settled_checkpoint_sha256"},
    )
    run_spec = _closed_json_artifact(
        package, "run_spec.json",
        {"motion_element", "solver_path", "field_count", "anglesunits", "axes", "intrinsic", "movecenter",
         "recorded_motion_end_s", "solver_relaxation_tail_s", "tail_mode", "solver_execution_authorized"},
    )

    source = manifest["source"]
    if receipt_ref != {**manifest["parents"]["s5a0_receipt"], "status": source["receipt_status"]}:
        raise HandoffError("selected receipt reference differs from manifest")
    if (source_ref["attempt_id"] != source["attempt_id"] or source_ref["role"] != source["role"]
            or source_ref["absolute_path"] != source["bag_path"]
            or source_ref["size_bytes"] != source["bag_size_bytes"]
            or source_ref["sha256"] != source["bag_sha256"] or source_ref["bag_copied"]
            or source_ref["selected_bag_receipt_sha256"] != receipt_ref["sha256"]):
        raise HandoffError("source bag reference differs from manifest/receipt")
    if outcome != {"source_outcome": "UNKNOWN", "source_provenance": "PARTIAL_BAG_ONLY",
                   "outcome_recomputed_from_topics": False, "do_not_override_r7_release": True}:
        raise HandoffError("source outcome claim ceiling drifted")
    if (topic["schema"] != "SPMPC_NON_FIXED" or topic["selected_bag_receipt_sha256"] != receipt_ref["sha256"]
            or set(topic["motion_inputs"]) != bridge.ALLOWED_MOTION_SIGNALS
            or topic["forbidden_signal_consumed"]):
        raise HandoffError("topic contract/provenance drifted")
    if (set(topic["qc_only_witness_counts"]) - set(topic["forbidden_motion_inputs"])
            or any(isinstance(value, bool) or not isinstance(value, int) or value < 0
                   for value in topic["qc_only_witness_counts"].values())):
        raise HandoffError("QC-only witness counts drifted")
    if (time_window["primary_time_source"] != "/odom.header.stamp"
            or time_window["bag_record_time_role"] != "ALIGNMENT_WITNESS_ONLY"
            or time_window["clock_role"] != "ALIGNMENT_WITNESS_ONLY"
            or any(time_window[key] != manifest["window"][key] for key in manifest["window"])):
        raise HandoffError("time-window contract differs from manifest")
    if frame != {"odom_frame": "odom", "base_frame": "base_footprint",
                 "container_frame": "c1m_container_reference",
                 "relative_transform": "inverse(T_odom_container(t0))*T_odom_container(t)",
                 "first_pose": "IDENTITY"}:
        raise HandoffError("frame/relative-transform contract drifted")
    if tf["tf_used_as_motion"] or tf["status"] != "PASS_ALIGNMENT_WITNESS":
        raise HandoffError("TF was not witness-only")
    if {key: motion_manifest[key] for key in manifest["motion_qc"]} != manifest["motion_qc"]:
        raise HandoffError("motion manifest differs from transfer manifest")
    if solver_qc != manifest["solver_path_qc"]:
        raise HandoffError("solver QC differs from transfer manifest")
    if (geometry["container"] != "C1" or geometry["replay_geometry"] != "C1M"
            or not geometry["development_compatibility"]
            or geometry["compatibility_contract_sha256"] != manifest["parents"]["compatibility_contract"]["sha256"]
            or mount["physical_mount_validated"] or not mount["development_assumption"]
            or fluid["container"] != "C1"
            or fluid["settled_checkpoint_sha256"] != manifest["parents"]["settled_checkpoint"]["sha256"]):
        raise HandoffError("C1/C1M geometry, mount, fluid, or settled identity drifted")
    if (run_spec["motion_element"] != "mvpathfile" or run_spec["field_count"] != 7
            or run_spec["axes"] != "ZYX" or not run_spec["intrinsic"] or not run_spec["movecenter"]
            or run_spec["solver_execution_authorized"]):
        raise HandoffError("run specification drifted or execution was authorized")

    odom_rows = _csv_rows(package["odom_raw.csv"], "odom_raw.csv")
    if odom_rows[0] != list(ODOM_FIELDS) or len(odom_rows) - 1 != topic["odom_message_count"]:
        raise HandoffError("odom_raw header/count differs")
    previous_header: int | None = None
    for index, row in enumerate(odom_rows[1:], start=1):
        if len(row) != len(ODOM_FIELDS):
            raise HandoffError(f"odom_raw row {index} field count differs")
        try:
            record_time, header_time = int(row[0]), int(row[1])
            numeric = [float(value) for value in row[4:]]
        except ValueError as error:
            raise HandoffError(f"odom_raw row {index} numeric field differs") from error
        if any(not math.isfinite(value) for value in numeric):
            raise HandoffError(f"odom_raw row {index} has non-finite value")
        if previous_header is not None and header_time <= previous_header:
            raise HandoffError("odom_raw time is not strictly increasing")
        if abs(record_time - header_time) / 1e9 > time_window["maximum_header_record_delta_s"]:
            raise HandoffError("odom_raw header/record alignment drifted")
        previous_header = header_time

    clock_rows = _csv_rows(package["clock_alignment.csv"], "clock_alignment.csv")
    if clock_rows[0] != ["odom_header_t_ns", "nearest_clock_t_ns", "abs_delta_ns"] or len(clock_rows) != len(odom_rows):
        raise HandoffError("clock alignment header/count differs")
    for row in clock_rows[1:]:
        if len(row) != 3:
            raise HandoffError("clock alignment field count differs")
        header, clock, delta = (int(value) for value in row)
        if delta != abs(header - clock) or delta / 1e9 > time_window["maximum_header_clock_delta_s"]:
            raise HandoffError("clock alignment value/tolerance differs")

    motion_rows = _csv_rows(package["motion.csv"], "motion.csv")
    if motion_rows[0] != list(MOTION_FIELDS):
        raise HandoffError("motion.csv header differs")
    motion: list[bridge.MotionSample] = []
    for index, row in enumerate(motion_rows[1:], start=1):
        if len(row) != 8:
            raise HandoffError(f"motion.csv row {index} field count differs")
        try:
            values = [float(value) for value in row]
        except ValueError as error:
            raise HandoffError(f"motion.csv row {index} numeric field differs") from error
        if any(not math.isfinite(value) for value in values):
            raise HandoffError(f"motion.csv row {index} has non-finite value")
        motion.append(bridge.MotionSample(*values))
    canonical_rows = bridge.build_solver_path(tuple(motion))
    solver_rows = bridge.parse_solver_path_csv(package["solver_path.csv"].decode("utf-8"))
    if len(solver_rows) != len(canonical_rows) + 1:
        raise HandoffError("solver relaxation tail row differs")
    final_recorded, final_tail = solver_rows[-2], solver_rows[-1]
    if (final_tail.numeric_fields()[1:] != final_recorded.numeric_fields()[1:]
            or abs((final_tail.t_s - final_recorded.t_s) - run_spec["solver_relaxation_tail_s"]) > 1e-12):
        raise HandoffError("solver tail is not an exact final-pose hold")
    query_grid = complete_roundtrip_query_grid(
        tuple(motion), solver_qc["subdivisions_per_interval"]
    )
    if (solver_qc["query_grid"] != "EVERY_PATH_INTERVAL_QUARTERS"
            or solver_qc["query_count"] != len(query_grid)):
        raise HandoffError("solver round-trip query grid evidence differs")
    round_trip = bridge.evaluate_solver_round_trip(
        tuple(motion), solver_rows[:-1], query_times_s=query_grid
    )
    observed_qc = (round_trip.max_position_error_m,
                   round_trip.max_quaternion_angular_distance_rad,
                   round_trip.max_rotation_matrix_angular_distance_rad)
    expected_qc = (solver_qc["max_position_error_m"],
                   solver_qc["max_quaternion_angular_distance_rad"],
                   solver_qc["max_rotation_matrix_angular_distance_rad"])
    if any(abs(left - right) > 1e-15 for left, right in zip(observed_qc, expected_qc)):
        raise HandoffError("solver round-trip evidence differs")


def materialize_create_new(
    root: Path, package: Mapping[str, bytes], *, actual_execution: bool = False
) -> None:
    validate_package_bytes(package)
    if actual_execution and not json.loads(package["transfer_manifest.json"])["safety"]["real_bag_read"]:
        raise HandoffError("actual publication cannot report real_bag_read=false")
    if root.exists() or root.is_symlink():
        raise FileExistsError(f"transfer root already exists: {root}")
    os.mkdir(root, 0o700)
    os.chmod(root, 0o700)
    for name in PACKAGE_INVENTORY:
        target = root / name
        descriptor = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            os.fchmod(descriptor, 0o600)
            view = memoryview(package[name])
            while view:
                written = os.write(descriptor, view)
                if written <= 0:
                    raise OSError("short package write")
                view = view[written:]
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def mock_inputs(policy: Mapping[str, Any]) -> tuple[bytes, list[dict[str, Any]], list[dict[str, int]], dict[str, Any]]:
    bag = policy["source"]
    receipt = {"status": bag["required_s5a0_status"], "source_provenance": "PARTIAL_BAG_ONLY",
               "source_bag": {"path": bag["bag_absolute_path"], "size_bytes": bag["bag_size_bytes"],
                              "mode": bag["bag_mode"], "sha256": bag["bag_sha256"]},
               "topic_contract": {"schema": "SPMPC_NON_FIXED", "odom_message_count": 23,
                                  "required_topics_pass": True, "conflicts": []},
               "source_bag_executed": False, "files_written_under_source_root": 0}
    records = []
    for index in range(23):
        time_ns = 100_000_000_000 + index * 50_000_000
        x = 0.0 if index < 20 else (index - 19) * 0.001
        yaw = 0.0 if index < 20 else math.radians(index - 19) * 0.1
        q = bridge.intrinsic_zyx_to_quaternion(yaw, 0.0, 0.0)
        records.append(dict(zip(ODOM_FIELDS, (time_ns + 1_000_000, time_ns, "odom", "base_footprint",
                                              x, 0.0, 0.0, *q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))))
    clock = [{"odom_header_t_ns": row["odom_header_t_ns"], "nearest_clock_t_ns": row["odom_header_t_ns"], "abs_delta_ns": 0}
             for row in records]
    tf = {"status": "PASS_ALIGNMENT_WITNESS", "odom_frame": "odom", "base_frame": "base_footprint",
          "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT", "tf_used_as_motion": False}
    return canonical_json(receipt), records, clock, tf


def self_check() -> dict[str, Any]:
    policy, identity = validate_policy()
    schema, schema_identity = read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    assert_schema_deep_closed(schema)
    receipt, records, clock, tf = mock_inputs(policy)
    package = build_package(policy=policy, policy_identity=identity,
                            s5a0_receipt_raw=receipt, s5a0_receipt_path="/mock/s5a0_primary_receipt.json",
                            odom_records=records, clock_alignment=clock, tf_alignment=tf)
    return {"status": "PASS_S5A1_HANDOFF_GATE_V1_SELF_CHECK", "policy": identity,
            "schema": schema_identity, "inventory": list(package),
            "package_digest": sha256_bytes(b"".join(package[name] for name in PACKAGE_INVENTORY)),
            "files_written": False, "real_bag_read": False, "optional_bag_read": False,
            "solver_executed": False, "gpu_exposed": False, "sudo_used": False, "network_used": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as error:  # fail closed at the CLI boundary
        print(json.dumps({"status": "FAIL", "error": str(error)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
