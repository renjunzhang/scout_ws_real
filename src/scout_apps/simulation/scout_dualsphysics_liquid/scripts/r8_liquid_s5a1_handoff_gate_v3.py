#!/usr/bin/env python3
"""Fail-closed semantic validator for the S5A1 ABI-v3 handoff package.

Gate v1 remains the package producer and structural/checksum validator.  Gate
v2 remains the S5A0 receipt adapter.  This revision independently binds every
derived artifact back to a trusted policy, the sealed S5A0 receipt, and the
raw odometry rows.  In particular it recomputes the odom-to-motion-to-solver
chain instead of trusting a consistently re-sealed package.

The validator never opens the source ROS bag.  Parent files are hashed through
bounded no-follow reads, or supplied as immutable bytes for synthetic tests.
Self-check is fully in-memory apart from frozen repository source/schema reads.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_handoff_gate_v2 as v2


sys.dont_write_bytecode = True
v1 = v2.v1
bridge = v1.bridge
HandoffError = v1.HandoffError
ForbiddenSignalConflict = v1.ForbiddenSignalConflict

FLOAT_ABS_TOLERANCE = 1.0e-12
PRE_ROLL_ABS_TOLERANCE = 1.0e-9
MAX_FROZEN_PARENT_BYTES = 8 * 1024 * 1024 * 1024
MOCK_POLICY_PATH = "/mock/s5a1_primary_handoff_policy.json"
MOCK_RECEIPT_PATH = "/mock/s5a0_primary_receipt.json"

_POLICY_TOP_KEYS = {
    "schema_version", "document_type", "policy_id", "abi", "transfer_id",
    "source", "parents", "time_window", "motion_contract", "solver_bridge",
    "compatibility", "signals", "package", "claim_ceiling",
}
_SOURCE_KEYS = {
    "domain", "attempt_id", "role", "container", "bag_relative_path",
    "bag_absolute_path", "bag_size_bytes", "bag_mode", "bag_sha256",
    "required_s5a0_status", "s5a0_receipt_identity",
    "receipt_sha256_binding", "source_provenance", "source_outcome",
    "optional_pair_read_or_admitted", "c2_read_or_admitted",
}
_TIME_WINDOW_KEYS = {
    "primary_time_source", "bag_record_time_role", "clock_role",
    "maximum_odom_gap_s", "maximum_header_record_delta_s",
    "maximum_header_clock_delta_s", "interpolation_period_s",
    "first_effective_translation_m", "first_effective_rotation_deg",
    "pre_roll_s", "terminal_provenance", "recorded_tail_rule",
    "solver_relaxation_tail_s", "solver_relaxation_tail_mode",
}
_COMPATIBILITY_KEYS = {
    "contract_status", "container_radius_m", "liquid_height_m",
    "density_kg_m3", "particle_spacing_m", "particle_count",
    "moving_boundary_count", "fluid_count",
    "moving_boundary_position_sha256", "mount_translation_m",
    "mount_quaternion_xyzw", "settled_part_index",
    "settled_checkpoint_sha256",
}


def _json_object(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise HandoffError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise HandoffError(f"{label} JSON root is not an object")
    return value


def _artifact_json(package: Mapping[str, bytes], name: str) -> dict[str, Any]:
    return _json_object(package[name], name)


def _is_sha256(value: object) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return value == value.lower()


def _absolute_path(value: object, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or "\x00" in value
        or not Path(value).is_absolute()
    ):
        raise HandoffError(f"{label} is not an absolute path")
    return value


def _identity_ref(value: object, label: str) -> dict[str, str]:
    if not isinstance(value, Mapping) or set(value) != {"path", "sha256"}:
        raise HandoffError(f"{label} is not a closed path/hash identity")
    path = _absolute_path(value["path"], f"{label}.path")
    digest = value["sha256"]
    if not _is_sha256(digest):
        raise HandoffError(f"{label}.sha256 is invalid")
    return {"path": path, "sha256": digest}


def _normalized_parent_bytes(
    supplied: Mapping[str | Path, bytes] | None,
) -> dict[str, bytes]:
    if supplied is None:
        return {}
    if not isinstance(supplied, Mapping):
        raise HandoffError("parent_bytes is not a mapping")
    normalized: dict[str, bytes] = {}
    for raw_path, raw in supplied.items():
        path = _absolute_path(str(raw_path), "parent_bytes key")
        if path in normalized:
            raise HandoffError(f"duplicate parent_bytes path: {path}")
        if not isinstance(raw, bytes) or not raw:
            raise HandoffError(f"parent_bytes value is not non-empty immutable bytes: {path}")
        if len(raw) > MAX_FROZEN_PARENT_BYTES:
            raise HandoffError(f"parent_bytes value exceeds bound: {path}")
        normalized[path] = raw
    return normalized


def _read_parent(
    path: str,
    supplied: Mapping[str, bytes],
    *,
    capture: bool,
    maximum: int = MAX_FROZEN_PARENT_BYTES,
) -> tuple[bytes | None, str, int]:
    """Return optional bytes, SHA-256, and size without following symlinks."""

    if path in supplied:
        raw = supplied[path]
        if not 0 < len(raw) <= maximum:
            raise HandoffError(f"supplied parent size outside bound: {path}")
        return (raw if capture else None, v1.sha256_bytes(raw), len(raw))

    descriptor = os.open(
        path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
    )
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise HandoffError(f"unsafe regular parent: {path}")
        if not 0 < before.st_size <= maximum:
            raise HandoffError(f"parent size outside bound: {path}")
        digest = hashlib.sha256()
        captured = bytearray() if capture else None
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            if not block:
                raise HandoffError(f"short parent read: {path}")
            digest.update(block)
            if captured is not None:
                captured.extend(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise HandoffError(f"parent grew during read: {path}")
        after = os.fstat(descriptor)
        before_identity = (
            before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns,
            before.st_ctime_ns, before.st_nlink,
        )
        after_identity = (
            after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns,
            after.st_ctime_ns, after.st_nlink,
        )
        if before_identity != after_identity:
            raise HandoffError(f"parent identity changed during read: {path}")
    finally:
        os.close(descriptor)
    return (
        bytes(captured) if captured is not None else None,
        digest.hexdigest(),
        before.st_size,
    )


def _rehash_ref(
    value: object,
    label: str,
    supplied: Mapping[str, bytes],
    *,
    expected: object | None = None,
    capture: bool = False,
    maximum: int = MAX_FROZEN_PARENT_BYTES,
) -> tuple[dict[str, str], bytes | None]:
    identity = _identity_ref(value, label)
    if expected is not None:
        expected_identity = _identity_ref(expected, f"trusted {label}")
        if identity != expected_identity:
            raise HandoffError(f"{label} differs from trusted policy")
    raw, observed_digest, _ = _read_parent(
        identity["path"], supplied, capture=capture, maximum=maximum
    )
    if observed_digest != identity["sha256"]:
        raise HandoffError(f"{label} parent hash differs from bytes")
    return identity, raw


def _semantic_equal(
    actual: object,
    expected: object,
    label: str,
    *,
    tolerance: float = FLOAT_ABS_TOLERANCE,
) -> None:
    if isinstance(expected, bool):
        if actual is not expected:
            raise HandoffError(f"{label} differs")
        return
    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping) or set(actual) != set(expected):
            raise HandoffError(f"{label} keys differ")
        for key, value in expected.items():
            _semantic_equal(actual[key], value, f"{label}.{key}", tolerance=tolerance)
        return
    if isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
            raise HandoffError(f"{label} sequence shape differs")
        for index, (left, right) in enumerate(zip(actual, expected)):
            _semantic_equal(left, right, f"{label}[{index}]", tolerance=tolerance)
        return
    if isinstance(expected, int):
        if isinstance(actual, bool) or not isinstance(actual, int) or actual != expected:
            raise HandoffError(f"{label} integer differs")
        return
    if isinstance(expected, float):
        if isinstance(actual, bool) or not isinstance(actual, (int, float)):
            raise HandoffError(f"{label} is not numeric")
        observed = float(actual)
        if not math.isfinite(observed) or abs(observed - expected) > tolerance:
            raise HandoffError(f"{label} numeric value differs")
        return
    if actual != expected:
        raise HandoffError(f"{label} differs")


def _assert_trusted_policy(policy: Mapping[str, Any]) -> None:
    if set(policy) != _POLICY_TOP_KEYS:
        raise HandoffError("trusted policy top-level keys differ")
    if (
        policy["schema_version"] != "smpcc-r8-liquid-s5a1-policy-v1"
        or policy["abi"] != bridge.ABI_ID
    ):
        raise HandoffError("trusted policy schema/ABI differs")
    source = policy["source"]
    if not isinstance(source, Mapping) or set(source) != _SOURCE_KEYS:
        raise HandoffError("trusted policy source is not closed")
    if (
        source["domain"] != bridge.SOURCE_DOMAIN
        or source["source_provenance"] != "PARTIAL_BAG_ONLY"
        or source["source_outcome"] != "UNKNOWN"
        or source["optional_pair_read_or_admitted"] is not False
        or source["c2_read_or_admitted"] is not False
        or not isinstance(source["bag_size_bytes"], int)
        or isinstance(source["bag_size_bytes"], bool)
        or source["bag_size_bytes"] <= 0
        or not _is_sha256(source["bag_sha256"])
    ):
        raise HandoffError("trusted policy source identity/safety differs")
    _absolute_path(source["bag_absolute_path"], "trusted source bag path")

    package_contract = policy["package"]
    if (
        not isinstance(package_contract, Mapping)
        or tuple(package_contract.get("inventory", ())) != v1.PACKAGE_INVENTORY
        or package_contract.get("file_mode") != "0600"
        or package_contract.get("write_semantics") != "O_EXCL_CREATE_NEW"
    ):
        raise HandoffError("trusted package inventory/write contract differs")

    signals = policy["signals"]
    if not isinstance(signals, Mapping) or set(signals) != {
        "motion_inputs", "qc_only", "forbidden_motion_inputs"
    }:
        raise HandoffError("trusted signal policy is not closed")
    motion_inputs = signals["motion_inputs"]
    qc_only = signals["qc_only"]
    forbidden = signals["forbidden_motion_inputs"]
    if (
        not isinstance(motion_inputs, list)
        or set(motion_inputs) != bridge.ALLOWED_MOTION_SIGNALS
        or len(motion_inputs) != len(bridge.ALLOWED_MOTION_SIGNALS)
        or not isinstance(qc_only, list)
        or not isinstance(forbidden, list)
        or any(not isinstance(item, str) or not item for item in qc_only + forbidden)
        or len(set(qc_only)) != len(qc_only)
        or len(set(forbidden)) != len(forbidden)
        or not set(forbidden).issubset(set(qc_only))
    ):
        raise HandoffError("trusted closed signal sets differ")
    bridge.validate_odom_only_provenance(
        primary_time_source=policy["time_window"]["primary_time_source"],
        pose_source=bridge.POSE_SOURCE,
        consumed_motion_signals=motion_inputs,
    )

    time_window = policy["time_window"]
    if not isinstance(time_window, Mapping) or set(time_window) != _TIME_WINDOW_KEYS:
        raise HandoffError("trusted time-window policy is not closed")
    if (
        time_window["bag_record_time_role"] != "ALIGNMENT_WITNESS_ONLY"
        or time_window["clock_role"] != "ALIGNMENT_WITNESS_ONLY"
        or time_window["solver_relaxation_tail_mode"] != "FINAL_POSE_ZERO_HOLD"
        or any(
            isinstance(time_window[name], bool)
            or not isinstance(time_window[name], (int, float))
            or not math.isfinite(float(time_window[name]))
            or float(time_window[name]) <= 0.0
            for name in (
                "maximum_odom_gap_s", "maximum_header_record_delta_s",
                "maximum_header_clock_delta_s", "interpolation_period_s",
                "first_effective_translation_m", "first_effective_rotation_deg",
                "pre_roll_s", "solver_relaxation_tail_s",
            )
        )
    ):
        raise HandoffError("trusted time-window values differ")

    motion_contract = policy["motion_contract"]
    required_motion = {
        "position_interpolation": "LINEAR",
        "orientation_interpolation": "NORMALIZED_QUATERNION_SLERP",
        "smoothing": False,
        "denoising": False,
        "path_projection": False,
        "result_driven_resampling": False,
        "relative_transform": "inverse(T_odom_container(t0))*T_odom_container(t)",
        "first_pose": "IDENTITY",
        "angles": "INTRINSIC_ZYX_CONTINUOUS_YAW_PITCH_ROLL_DEGREES",
        "roundtrip_query_grid": "EVERY_PATH_INTERVAL_QUARTERS",
        "roundtrip_subdivisions_per_interval": 4,
    }
    if not isinstance(motion_contract, Mapping):
        raise HandoffError("trusted motion contract is not an object")
    for key, value in required_motion.items():
        if motion_contract.get(key) != value:
            raise HandoffError(f"trusted motion contract differs: {key}")

    expected_solver_bridge = {
        "motion_element": "mvpathfile", "field_count": 7,
        "field_indices": [0, 1, 2, 3, 4, 5, 6],
        "anglesunits": "degrees", "axes": "ZYX", "intrinsic": True,
        "movecenter": True, "rotation_center_m": [0.0, 0.0, 0.0],
        "angle_order": ["continuous_yaw_Z", "pitch_Y", "roll_X"],
        "candidate_executable": False,
    }
    _semantic_equal(policy["solver_bridge"], expected_solver_bridge, "trusted solver bridge")

    compatibility = policy["compatibility"]
    if not isinstance(compatibility, Mapping) or set(compatibility) != _COMPATIBILITY_KEYS:
        raise HandoffError("trusted compatibility projection is not closed")
    for name, count_minimum in (
        ("particle_count", 1), ("moving_boundary_count", 1),
        ("fluid_count", 1), ("settled_part_index", 0),
    ):
        value = compatibility[name]
        if isinstance(value, bool) or not isinstance(value, int) or value < count_minimum:
            raise HandoffError(f"trusted compatibility count differs: {name}")
    if (
        len(compatibility["mount_translation_m"]) != 3
        or len(compatibility["mount_quaternion_xyzw"]) != 4
        or not _is_sha256(compatibility["moving_boundary_position_sha256"])
        or not _is_sha256(compatibility["settled_checkpoint_sha256"])
    ):
        raise HandoffError("trusted compatibility geometry/hash differs")

    parents = policy["parents"]
    if not isinstance(parents, Mapping):
        raise HandoffError("trusted parent set is not an object")
    for name in ("compatibility_contract", "motion_bridge", "settled_checkpoint"):
        _identity_ref(parents.get(name), f"trusted policy parent {name}")
    if (
        compatibility["settled_checkpoint_sha256"]
        != parents["settled_checkpoint"]["sha256"]
    ):
        raise HandoffError("trusted settled checkpoint projections differ")


def _parse_odom(
    package: Mapping[str, bytes],
    policy: Mapping[str, Any],
    expected_count: int,
) -> tuple[dict[str, Any], ...]:
    rows = v1._csv_rows(package["odom_raw.csv"], "odom_raw.csv")
    if rows[0] != list(v1.ODOM_FIELDS) or len(rows) - 1 != expected_count:
        raise HandoffError("odom_raw header/count differs from receipt")
    records: list[dict[str, Any]] = []
    for index, row in enumerate(rows[1:]):
        if len(row) != len(v1.ODOM_FIELDS):
            raise HandoffError(f"odom_raw row {index} field count differs")
        try:
            values: list[Any] = [
                int(row[0]), int(row[1]), row[2], row[3],
                *(float(value) for value in row[4:]),
            ]
        except ValueError as exc:
            raise HandoffError(f"odom_raw row {index} numeric value differs") from exc
        records.append(dict(zip(v1.ODOM_FIELDS, values)))
    return v1.validate_odom_records(records, policy)


def _parse_clock(
    package: Mapping[str, bytes],
    odom: Sequence[Mapping[str, Any]],
    policy: Mapping[str, Any],
) -> tuple[dict[str, int], ...]:
    rows = v1._csv_rows(package["clock_alignment.csv"], "clock_alignment.csv")
    header = ["odom_header_t_ns", "nearest_clock_t_ns", "abs_delta_ns"]
    if rows[0] != header or len(rows) - 1 != len(odom):
        raise HandoffError("clock alignment header/count differs from odom")
    parsed: list[dict[str, int]] = []
    for index, (row, odom_row) in enumerate(zip(rows[1:], odom)):
        if len(row) != 3:
            raise HandoffError(f"clock alignment row {index} field count differs")
        try:
            values = tuple(int(value) for value in row)
        except ValueError as exc:
            raise HandoffError(f"clock alignment row {index} integer differs") from exc
        if any(value < 0 for value in values):
            raise HandoffError(f"clock alignment row {index} is negative")
        witness = dict(zip(header, values))
        if witness["odom_header_t_ns"] != odom_row["odom_header_t_ns"]:
            raise HandoffError(f"clock alignment row {index} does not bind odom")
        parsed.append(witness)
    tf = _artifact_json(package, "tf_alignment.json")
    checked, _ = v1.validate_alignment_witnesses(odom, parsed, tf, policy)
    return checked


def _parse_motion(package: Mapping[str, bytes]) -> tuple[Any, ...]:
    rows = v1._csv_rows(package["motion.csv"], "motion.csv")
    if rows[0] != list(v1.MOTION_FIELDS) or len(rows) < 3:
        raise HandoffError("motion.csv header/count differs")
    parsed = []
    for index, row in enumerate(rows[1:]):
        if len(row) != len(v1.MOTION_FIELDS):
            raise HandoffError(f"motion.csv row {index} field count differs")
        try:
            values = tuple(float(value) for value in row)
        except ValueError as exc:
            raise HandoffError(f"motion.csv row {index} numeric value differs") from exc
        if any(not math.isfinite(value) for value in values):
            raise HandoffError(f"motion.csv row {index} contains non-finite data")
        parsed.append(bridge.MotionSample(*values))
    return tuple(parsed)


def _sample_fields(sample: Any) -> tuple[float, ...]:
    return tuple(getattr(sample, name) for name in v1.MOTION_FIELDS)


def _solver_fields(row: Any) -> tuple[float, ...]:
    return tuple(row.numeric_fields())


def _first_effective_time(
    motion: Sequence[Any], policy: Mapping[str, Any]
) -> float:
    origin = motion[0]
    for sample in motion[1:]:
        displacement = math.sqrt(
            sample.x_rel_m ** 2 + sample.y_rel_m ** 2 + sample.z_rel_m ** 2
        )
        angle_deg = math.degrees(
            bridge.quaternion_angular_distance(
                (origin.qx_rel, origin.qy_rel, origin.qz_rel, origin.qw_rel),
                (sample.qx_rel, sample.qy_rel, sample.qz_rel, sample.qw_rel),
            )
        )
        if (
            displacement >= policy["time_window"]["first_effective_translation_m"]
            or angle_deg >= policy["time_window"]["first_effective_rotation_deg"]
        ):
            first = sample.t_s
            break
    else:
        raise HandoffError("first-effective motion was not found")
    pre_roll = float(policy["time_window"]["pre_roll_s"])
    if abs(first - pre_roll) > PRE_ROLL_ABS_TOLERANCE:
        raise HandoffError("first-effective motion does not equal frozen pre-roll")
    return first


def validate_package_bytes(
    package: Mapping[str, bytes],
    schema: Mapping[str, Any] | None = None,
    *,
    parent_bytes: Mapping[str | Path, bytes] | None = None,
    trusted_policy_path: str | Path | None = None,
) -> dict[str, Any]:
    """Validate one complete package without opening its source bag."""

    v1.validate_package_bytes(package, schema)
    manifest = _artifact_json(package, "transfer_manifest.json")
    if schema is not None:
        frozen_schema, _ = v1.read_json(v1.SCHEMA_PATH)
        Draft202012Validator(frozen_schema).validate(manifest)

    supplied = _normalized_parent_bytes(parent_bytes)
    trusted_path = _absolute_path(
        str(trusted_policy_path) if trusted_policy_path is not None else str(v1.POLICY_PATH),
        "trusted_policy_path",
    )
    policy_raw, policy_digest, _ = _read_parent(
        trusted_path, supplied, capture=True, maximum=v1.MAX_JSON_BYTES
    )
    assert policy_raw is not None
    policy = _json_object(policy_raw, "trusted policy")
    _assert_trusted_policy(policy)

    parents = manifest["parents"]
    expected_policy_ref = {"path": trusted_path, "sha256": policy_digest}
    _semantic_equal(parents["policy"], expected_policy_ref, "manifest policy parent")

    compatibility_ref, _ = _rehash_ref(
        parents["compatibility_contract"], "compatibility_contract",
        supplied, expected=policy["parents"]["compatibility_contract"],
    )
    motion_bridge_ref, _ = _rehash_ref(
        parents["motion_bridge"], "motion_bridge",
        supplied, expected=policy["parents"]["motion_bridge"],
    )
    settled_ref, _ = _rehash_ref(
        parents["settled_checkpoint"], "settled_checkpoint",
        supplied, expected=policy["parents"]["settled_checkpoint"],
    )
    receipt_ref, receipt_raw = _rehash_ref(
        parents["s5a0_receipt"], "s5a0_receipt", supplied,
        capture=True, maximum=v1.MAX_JSON_BYTES,
    )
    assert receipt_raw is not None
    receipt, observed_receipt_ref = v2.validate_s5a0_receipt(
        receipt_raw, receipt_ref["path"], policy
    )
    _semantic_equal(observed_receipt_ref, receipt_ref, "validated receipt identity")
    odom_count = v2.s5a0_odom_message_count(receipt)

    source = policy["source"]
    expected_manifest_source = {
        "domain": source["domain"], "attempt_id": source["attempt_id"],
        "role": source["role"], "source_provenance": "PARTIAL_BAG_ONLY",
        "source_outcome": "UNKNOWN", "bag_path": source["bag_absolute_path"],
        "bag_size_bytes": source["bag_size_bytes"],
        "bag_sha256": source["bag_sha256"],
        "receipt_status": source["required_s5a0_status"],
    }
    _semantic_equal(manifest["source"], expected_manifest_source, "manifest source")

    receipt_reference = _artifact_json(package, "selected_bag_receipt_ref.json")
    expected_receipt_reference = {
        **receipt_ref, "status": receipt["status"],
    }
    _semantic_equal(
        receipt_reference, expected_receipt_reference,
        "selected_bag_receipt_ref",
    )
    source_reference = _artifact_json(package, "source_bag_ref.json")
    expected_source_reference = {
        "attempt_id": source["attempt_id"], "role": source["role"],
        "relative_path": source["bag_relative_path"],
        "absolute_path": source["bag_absolute_path"],
        "size_bytes": source["bag_size_bytes"], "sha256": source["bag_sha256"],
        "selected_bag_receipt_sha256": receipt_ref["sha256"],
        "bag_copied": False,
    }
    _semantic_equal(source_reference, expected_source_reference, "source_bag_ref")

    expected_outcome = {
        "source_outcome": "UNKNOWN", "source_provenance": "PARTIAL_BAG_ONLY",
        "outcome_recomputed_from_topics": False,
        "do_not_override_r7_release": True,
    }
    _semantic_equal(
        _artifact_json(package, "source_outcome.json"),
        expected_outcome, "source_outcome",
    )

    topic = _artifact_json(package, "topic_contract.json")
    witness_counts = topic.get("qc_only_witness_counts")
    if not isinstance(witness_counts, Mapping):
        raise HandoffError("QC-only witness counts are not an object")
    allowed_witnesses = set(policy["signals"]["forbidden_motion_inputs"])
    if (
        set(witness_counts) - allowed_witnesses
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            for value in witness_counts.values()
        )
    ):
        raise HandoffError("QC-only witness counts are not closed nonnegative integers")
    expected_topic = {
        "schema": "SPMPC_NON_FIXED", "odom_message_count": odom_count,
        "selected_bag_receipt_sha256": receipt_ref["sha256"],
        "motion_inputs": policy["signals"]["motion_inputs"],
        "qc_only": policy["signals"]["qc_only"],
        "forbidden_motion_inputs": policy["signals"]["forbidden_motion_inputs"],
        "qc_only_witness_counts": dict(witness_counts),
        "forbidden_signal_consumed": False,
    }
    _semantic_equal(topic, expected_topic, "topic contract")

    odom = _parse_odom(package, policy, odom_count)
    _parse_clock(package, odom, policy)

    expected_tf = {
        "status": "PASS_ALIGNMENT_WITNESS", "odom_frame": "odom",
        "base_frame": "base_footprint",
        "container_mount_source": "FROZEN_COMPATIBILITY_CONTRACT",
        "tf_used_as_motion": False,
    }
    _semantic_equal(
        _artifact_json(package, "tf_alignment.json"), expected_tf, "TF alignment"
    )
    expected_frame = {
        "odom_frame": "odom", "base_frame": "base_footprint",
        "container_frame": "c1m_container_reference",
        "relative_transform": policy["motion_contract"]["relative_transform"],
        "first_pose": policy["motion_contract"]["first_pose"],
    }
    _semantic_equal(
        _artifact_json(package, "frame_contract.json"),
        expected_frame, "frame contract",
    )

    observed_motion = _parse_motion(package)
    expected_motion = v1.derive_motion(odom, policy)
    if len(observed_motion) != len(expected_motion):
        raise HandoffError("motion row count differs from odom-derived motion")
    for index, (observed, expected) in enumerate(
        zip(observed_motion, expected_motion)
    ):
        _semantic_equal(
            _sample_fields(observed), _sample_fields(expected),
            f"odom-derived motion row {index}",
        )

    motion_qc = v1._motion_metrics(expected_motion)
    expected_motion_manifest = {
        **motion_qc,
        "canonical_orientation": "NORMALIZED_QUATERNION_SLERP",
        "position_interpolation": "LINEAR",
        "source": "/odom.header.stamp+/odom.pose.pose",
    }
    _semantic_equal(
        _artifact_json(package, "motion_manifest.json"),
        expected_motion_manifest, "motion manifest",
    )
    _semantic_equal(manifest["motion_qc"], motion_qc, "manifest motion QC")

    first_effective = _first_effective_time(expected_motion, policy)
    start_ns = odom[0]["odom_header_t_ns"]
    end_ns = odom[-1]["odom_header_t_ns"]
    window_projection = {
        "start_header_t_ns": start_ns, "end_header_t_ns": end_ns,
        "duration_s": (end_ns - start_ns) / 1e9,
        "sample_count": len(expected_motion),
        "first_effective_t_s": first_effective,
        "pre_roll_s": policy["time_window"]["pre_roll_s"],
        "terminal_provenance": policy["time_window"]["terminal_provenance"],
        "recorded_tail_rule": policy["time_window"]["recorded_tail_rule"],
        "solver_relaxation_tail_s":
            policy["time_window"]["solver_relaxation_tail_s"],
    }
    expected_time_window = {**policy["time_window"], **window_projection}
    _semantic_equal(
        _artifact_json(package, "time_window.json"),
        expected_time_window, "time window",
    )
    _semantic_equal(manifest["window"], window_projection, "manifest window")

    expected_interpolation = {
        "grid_period_s": policy["time_window"]["interpolation_period_s"],
        "sample_count": len(expected_motion), "strict_time": True,
        "maximum_source_gap_s": policy["time_window"]["maximum_odom_gap_s"],
        "smoothing": False, "denoising": False,
        "path_projection": False, "pass": True,
    }
    _semantic_equal(
        _artifact_json(package, "interpolation_qc.json"),
        expected_interpolation, "interpolation QC",
    )

    canonical_solver = bridge.build_solver_path(
        expected_motion,
        gimbal_lock_cos_threshold=
            policy["motion_contract"]["gimbal_lock_cos_threshold"],
    )
    try:
        solver_rows = bridge.parse_solver_path_csv(
            package["solver_path.csv"].decode("utf-8"),
        )
    except (UnicodeError, ValueError) as exc:
        raise HandoffError("solver_path.csv is invalid") from exc
    if len(solver_rows) != len(canonical_solver) + 1:
        raise HandoffError("solver path row count differs from canonical path plus tail")
    for index, (observed, expected) in enumerate(
        zip(solver_rows[:-1], canonical_solver)
    ):
        _semantic_equal(
            _solver_fields(observed), _solver_fields(expected),
            f"canonical solver row {index}",
        )
    final_recorded = solver_rows[-2]
    final_tail = solver_rows[-1]
    if _solver_fields(final_tail)[1:] != _solver_fields(final_recorded)[1:]:
        raise HandoffError("solver tail is not an exact final-pose hold")
    expected_tail_time = (
        final_recorded.t_s + policy["time_window"]["solver_relaxation_tail_s"]
    )
    _semantic_equal(final_tail.t_s, expected_tail_time, "solver tail time")

    subdivisions = policy["motion_contract"][
        "roundtrip_subdivisions_per_interval"
    ]
    query_grid = v1.complete_roundtrip_query_grid(expected_motion, subdivisions)
    round_trip = bridge.evaluate_solver_round_trip(
        expected_motion, solver_rows[:-1], query_times_s=query_grid
    )
    if (
        round_trip.max_position_error_m
        > policy["motion_contract"]["maximum_roundtrip_position_error_m"]
        or max(
            round_trip.max_quaternion_angular_distance_rad,
            round_trip.max_rotation_matrix_angular_distance_rad,
        )
        > policy["motion_contract"]["maximum_roundtrip_angular_error_rad"]
    ):
        raise HandoffError("solver round-trip exceeds trusted thresholds")
    expected_solver_qc = {
        "row_count": len(solver_rows),
        "field_count": policy["solver_bridge"]["field_count"],
        "first_row_identity": True, "gimbal_lock": False, "nonfinite": False,
        "query_grid": policy["motion_contract"]["roundtrip_query_grid"],
        "subdivisions_per_interval": subdivisions,
        "query_count": len(query_grid),
        "max_position_error_m": round_trip.max_position_error_m,
        "max_quaternion_angular_distance_rad":
            round_trip.max_quaternion_angular_distance_rad,
        "max_rotation_matrix_angular_distance_rad":
            round_trip.max_rotation_matrix_angular_distance_rad,
        "pass": True,
    }
    _semantic_equal(
        _artifact_json(package, "solver_path_qc.json"),
        expected_solver_qc, "solver path QC",
    )
    _semantic_equal(
        manifest["solver_path_qc"], expected_solver_qc,
        "manifest solver path QC",
    )

    compatibility = policy["compatibility"]
    expected_geometry = {
        "container": "C1", "replay_geometry": "C1M",
        "development_compatibility": True,
        "radius_m": compatibility["container_radius_m"],
        "particle_spacing_m": compatibility["particle_spacing_m"],
        "particle_count": compatibility["particle_count"],
        "moving_boundary_count": compatibility["moving_boundary_count"],
        "moving_boundary_position_sha256":
            compatibility["moving_boundary_position_sha256"],
        "compatibility_contract_sha256": compatibility_ref["sha256"],
    }
    expected_mount = {
        "parent_frame": "base_footprint",
        "child_frame": "c1m_container_reference",
        "translation_m": compatibility["mount_translation_m"],
        "quaternion_xyzw": compatibility["mount_quaternion_xyzw"],
        "rotation_center_m": policy["solver_bridge"]["rotation_center_m"],
        "development_assumption": True, "physical_mount_validated": False,
    }
    expected_fluid = {
        "container": "C1", "liquid_height_m": compatibility["liquid_height_m"],
        "density_kg_m3": compatibility["density_kg_m3"],
        "particle_count": compatibility["particle_count"],
        "fluid_count": compatibility["fluid_count"],
        "settled_part_index": compatibility["settled_part_index"],
        "settled_checkpoint_sha256": settled_ref["sha256"],
    }
    expected_run_spec = {
        "motion_element": policy["solver_bridge"]["motion_element"],
        "solver_path": "solver_path.csv",
        "field_count": policy["solver_bridge"]["field_count"],
        "anglesunits": policy["solver_bridge"]["anglesunits"],
        "axes": policy["solver_bridge"]["axes"],
        "intrinsic": policy["solver_bridge"]["intrinsic"],
        "movecenter": policy["solver_bridge"]["movecenter"],
        "recorded_motion_end_s": expected_motion[-1].t_s,
        "solver_relaxation_tail_s":
            policy["time_window"]["solver_relaxation_tail_s"],
        "tail_mode": policy["time_window"]["solver_relaxation_tail_mode"],
        "solver_execution_authorized": False,
    }
    for name, expected in (
        ("container_geometry_manifest.json", expected_geometry),
        ("container_mount_manifest.json", expected_mount),
        ("fluid_spec.json", expected_fluid),
        ("run_spec.json", expected_run_spec),
    ):
        _semantic_equal(_artifact_json(package, name), expected, name)

    expected_contract = {
        "primary_time_source": policy["time_window"]["primary_time_source"],
        "position_interpolation":
            policy["motion_contract"]["position_interpolation"],
        "orientation_interpolation":
            policy["motion_contract"]["orientation_interpolation"],
        "euler_convention": policy["motion_contract"]["angles"],
        "solver_field_count": policy["solver_bridge"]["field_count"],
        "compatibility_status": compatibility["contract_status"],
    }
    _semantic_equal(manifest["contract"], expected_contract, "manifest contract")

    package_digest = v1.sha256_bytes(
        b"".join(package[name] for name in v1.PACKAGE_INVENTORY)
    )
    return {
        "status": "PASS_S5A1_HANDOFF_GATE_V3_STRONG_SEMANTICS",
        "inventory": list(v1.PACKAGE_INVENTORY),
        "odom_row_count": len(odom),
        "motion_row_count": len(expected_motion),
        "solver_row_count": len(solver_rows),
        "package_digest": package_digest,
        "parent_sha256": {
            "policy": policy_digest,
            "s5a0_receipt": receipt_ref["sha256"],
            "compatibility_contract": compatibility_ref["sha256"],
            "motion_bridge": motion_bridge_ref["sha256"],
            "settled_checkpoint": settled_ref["sha256"],
        },
        "source_bag_read": False,
        "external_write": False,
    }


def validate_package_root(
    root: str | Path,
    *,
    parent_bytes: Mapping[str | Path, bytes] | None = None,
    trusted_policy_path: str | Path | None = None,
) -> dict[str, Any]:
    """Read an exact, private package directory and validate all 20 files."""

    package_root = Path(root)
    metadata = os.lstat(package_root)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise HandoffError("package root is not a real directory")
    if stat.S_IMODE(metadata.st_mode) != 0o700:
        raise HandoffError("package root mode differs")
    names = tuple(entry.name for entry in os.scandir(package_root))
    if set(names) != set(v1.PACKAGE_INVENTORY) or len(names) != len(v1.PACKAGE_INVENTORY):
        raise HandoffError("package root inventory is not exact")
    package: dict[str, bytes] = {}
    for name in v1.PACKAGE_INVENTORY:
        raw, identity = v1.read_regular(
            package_root / name, maximum=v1.MAX_ARTIFACT_BYTES
        )
        file_metadata = os.lstat(package_root / name)
        if stat.S_IMODE(file_metadata.st_mode) != 0o600:
            raise HandoffError(f"package file mode differs: {name}")
        if identity["size_bytes"] != len(raw):
            raise HandoffError(f"package file identity differs: {name}")
        package[name] = raw
    result = validate_package_bytes(
        package, parent_bytes=parent_bytes,
        trusted_policy_path=trusted_policy_path,
    )
    return {**result, "package_root": str(package_root)}


def _reseal_manifest_only(package: Mapping[str, bytes], manifest: Mapping[str, Any]) -> dict[str, bytes]:
    complete = dict(package)
    complete["transfer_manifest.json"] = v1.canonical_json(manifest)
    complete["checksums.sha256"] = "".join(
        f"{v1.sha256_bytes(complete[name])}  {name}\n"
        for name in v1.CHECKSUMMED_FILES
    ).encode("ascii")
    return complete


def _synthetic_fixture() -> tuple[dict[str, bytes], dict[str, bytes], str]:
    """Build a package and all five trusted parents without real input reads."""

    policy = _json_object(v1.POLICY_PATH.read_bytes(), "repository policy fixture")
    policy = copy.deepcopy(policy)
    compatibility_raw = v1.canonical_json(
        {"fixture": "S5A1_COMPATIBILITY_PARENT", "real_input": False}
    )
    motion_bridge_raw = v1.canonical_json(
        {"fixture": "S5A1_MOTION_BRIDGE_PARENT", "real_input": False}
    )
    settled_raw = b"SYNTHETIC_S5A1_SETTLED_CHECKPOINT_NO_REAL_BI4\n"
    compatibility_path = "/mock/s5_c1_compatibility_contract.json"
    motion_bridge_path = "/mock/s5a1_motion_bridge.py"
    settled_path = "/mock/Part_0901.bi4"
    policy["parents"]["compatibility_contract"] = {
        "path": compatibility_path,
        "sha256": v1.sha256_bytes(compatibility_raw),
    }
    policy["parents"]["motion_bridge"] = {
        "path": motion_bridge_path,
        "sha256": v1.sha256_bytes(motion_bridge_raw),
    }
    policy["parents"]["settled_checkpoint"] = {
        "path": settled_path, "sha256": v1.sha256_bytes(settled_raw),
    }
    policy["compatibility"]["settled_checkpoint_sha256"] = v1.sha256_bytes(
        settled_raw
    )
    policy_raw = v1.canonical_json(policy)
    receipt_raw, records, clock, tf = v1.mock_inputs(policy)
    package = v1.build_package(
        policy=policy,
        policy_identity={
            "path": MOCK_POLICY_PATH,
            "sha256": v1.sha256_bytes(policy_raw),
            "size_bytes": len(policy_raw),
        },
        s5a0_receipt_raw=receipt_raw,
        s5a0_receipt_path=MOCK_RECEIPT_PATH,
        odom_records=records,
        clock_alignment=clock,
        tf_alignment=tf,
    )
    manifest = _artifact_json(package, "transfer_manifest.json")
    manifest["parents"]["policy"] = {
        "path": MOCK_POLICY_PATH, "sha256": v1.sha256_bytes(policy_raw),
    }
    package = _reseal_manifest_only(package, manifest)
    parents = {
        MOCK_POLICY_PATH: policy_raw,
        MOCK_RECEIPT_PATH: receipt_raw,
        compatibility_path: compatibility_raw,
        motion_bridge_path: motion_bridge_raw,
        settled_path: settled_raw,
    }
    return package, parents, MOCK_POLICY_PATH


def self_check() -> dict[str, Any]:
    package, parents, policy_path = _synthetic_fixture()
    result = validate_package_bytes(
        package, parent_bytes=parents, trusted_policy_path=policy_path
    )
    return {
        **result,
        "status": "PASS_S5A1_HANDOFF_GATE_V3_SELF_CHECK",
        "synthetic_fixture": True,
        "real_parent_read": False,
        "real_s5a0_receipt_read": False,
        "real_bag_read": False,
        "optional_bag_read": False,
        "files_written": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "sudo_used": False,
        "network_used": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:  # fail closed at the CLI boundary
        print(
            json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


def __getattr__(name: str) -> Any:
    return getattr(v2, name)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "HandoffError", "ForbiddenSignalConflict", "validate_package_bytes",
    "validate_package_root", "self_check",
]
