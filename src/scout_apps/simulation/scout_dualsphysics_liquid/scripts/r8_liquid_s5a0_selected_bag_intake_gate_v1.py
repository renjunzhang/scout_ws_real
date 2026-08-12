#!/usr/bin/env python3
"""Static-first S5A0 selected ROS1 bag intake gate.

The production policy admits one exact primary bag.  This module can build a
deterministic, read-only and network-unshared bubblewrap argv, but it never
launches it.  The sandbox worker and test helper use the dependency-free ROS1
Bag V2 reader and publish at most one create-new receipt with ``O_EXCL``.

No function imports ROS, opens a network socket, executes the source bag,
repairs an index, starts a solver, or exposes a GPU.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import r8_liquid_ros1_bag_v2_reader_v1 as bag_reader


sys.dont_write_bytecode = True

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
GATE_PATH = Path(__file__).resolve()
READER_PATH = GATE_PATH.with_name("r8_liquid_ros1_bag_v2_reader_v1.py")
POLICY_PATH = (
    PACKAGE_ROOT
    / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_selected_bag_policy_v1.json"
)
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_selected_bag_receipt_v1.json"

POLICY_SCHEMA_VERSION = "smpcc-r8-liquid-s5a0-selected-bag-policy-v1"
RECEIPT_SCHEMA_VERSION = "smpcc-r8-liquid-s5a0-selected-bag-receipt-v1"
PASS_STATUS = "S5A0_SELECTED_R7_BAG_SEALED_DEVELOPMENT_ONLY"
EXPECTED_POLICY_ID = "liquid_zrj_msi_u2404_s5a0_primary_bsmooth_b01_v1"
EXPECTED_ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
EXPECTED_SOURCE_ROOT = "/home/zrj/slosh_bags/matrix_bags"
EXPECTED_PRIMARY = (
    "/home/zrj/slosh_bags/matrix_bags/"
    "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
)
EXPECTED_OPTIONAL = (
    "/home/zrj/slosh_bags/matrix_bags/"
    "SIM-S1_CORE_H1_C1_Bslosh_b01_r01/capture.bag"
)
EXPECTED_SIZE = 13_996_902
EXPECTED_MODE = "0755"
EXPECTED_SHA256 = "c82c1f16b41bced51ab0aff63e4ef40b469501e185851344789fc3d62399fc07"
MAXIMUM_POLICY_BYTES = 256 * 1024
MAXIMUM_SCHEMA_BYTES = 512 * 1024

READER_LIMIT_KEYS = (
    "maximum_file_bytes",
    "maximum_record_header_bytes",
    "maximum_record_data_bytes",
    "maximum_field_bytes",
    "maximum_message_bytes",
    "maximum_message_definition_bytes",
    "maximum_chunk_uncompressed_bytes",
    "maximum_total_uncompressed_bytes",
    "maximum_records",
    "maximum_messages",
    "maximum_connections",
    "maximum_chunks",
    "maximum_array_count",
    "maximum_string_bytes",
)


class S5A0GateError(ValueError):
    """A fail-closed S5A0 policy, path, parser, or receipt failure."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


def _fail(code: str, message: str) -> None:
    raise S5A0GateError(code, message)


def _exact_keys(value: Mapping[str, Any], expected: set[str], label: str) -> None:
    actual = set(value)
    if actual != expected:
        _fail(
            "POLICY_KEYS_INVALID",
            f"{label} keys differ; missing={sorted(expected - actual)!r}, "
            f"extra={sorted(actual - expected)!r}",
        )


def _sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _canonical_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def canonical_sha256(value: object) -> str:
    return _sha256_bytes(_canonical_bytes(value))


def _read_bounded_json(path: Path, maximum_bytes: int, label: str) -> tuple[dict[str, Any], bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        _fail("STATIC_INPUT_READ_FAILED", f"cannot read {label}: {exc}")
    if not raw or len(raw) > maximum_bytes:
        _fail("STATIC_INPUT_SIZE_INVALID", f"{label} is empty or oversized")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        _fail("STATIC_INPUT_JSON_INVALID", f"{label} is not valid JSON: {exc}")
    if not isinstance(value, dict):
        _fail("STATIC_INPUT_JSON_INVALID", f"{label} root is not an object")
    return value, raw


def load_policy(path: Path = POLICY_PATH) -> tuple[dict[str, Any], str]:
    policy, raw = _read_bounded_json(path, MAXIMUM_POLICY_BYTES, "S5A0 policy")
    validate_policy(policy)
    return policy, _sha256_bytes(raw)


def load_schema(path: Path = SCHEMA_PATH) -> dict[str, Any]:
    schema, _ = _read_bounded_json(path, MAXIMUM_SCHEMA_BYTES, "S5A0 receipt schema")
    return schema


def _is_lower_hex(value: object, count: int) -> bool:
    return (
        isinstance(value, str)
        and len(value) == count
        and all(character in "0123456789abcdef" for character in value)
    )


def _absolute_normalized(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.startswith("/"):
        _fail("PATH_NOT_ABSOLUTE", f"{label} must be an absolute string")
    if "\x00" in value or os.path.normpath(value) != value or ".." in Path(value).parts:
        _fail("PATH_NOT_NORMALIZED", f"{label} is not lexically normalized")
    return value


def _relative_normalized(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or value.startswith("/"):
        _fail("PATH_NOT_RELATIVE", f"{label} must be a non-empty relative path")
    if os.path.normpath(value) != value or ".." in Path(value).parts:
        _fail("PATH_NOT_NORMALIZED", f"{label} is not lexically normalized")
    if any(marker in value for marker in ("*", "?", "[", "]")):
        _fail("PATH_GLOB_FORBIDDEN", f"{label} contains a glob marker")
    return value


def validate_policy(policy: Mapping[str, Any]) -> None:
    _exact_keys(
        policy,
        {
            "schema_version",
            "document_type",
            "policy_id",
            "status",
            "parents",
            "selection",
            "filesystem_contract",
            "topic_contract",
            "sandbox_contract",
            "limits",
            "claim_ceiling",
            "next",
        },
        "policy",
    )
    if policy["schema_version"] != POLICY_SCHEMA_VERSION:
        _fail("POLICY_VERSION_INVALID", "unexpected policy schema version")
    if policy["document_type"] != "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_POLICY_V1":
        _fail("POLICY_DOCUMENT_TYPE_INVALID", "unexpected policy document type")

    selection = policy["selection"]
    if not isinstance(selection, Mapping):
        _fail("POLICY_SELECTION_INVALID", "selection is not an object")
    _exact_keys(
        selection,
        {
            "source_domain",
            "source_provenance",
            "source_root",
            "attempt_id",
            "selected_role",
            "selected_count",
            "relative_path",
            "absolute_path",
            "expected_size_bytes",
            "expected_mode",
            "expected_sha256",
            "optional_pair_authorized",
            "forbidden_optional_path",
        },
        "selection",
    )
    source_root = _absolute_normalized(selection["source_root"], "selection.source_root")
    absolute_path = _absolute_normalized(selection["absolute_path"], "selection.absolute_path")
    relative_path = _relative_normalized(selection["relative_path"], "selection.relative_path")
    forbidden_optional = _absolute_normalized(
        selection["forbidden_optional_path"], "selection.forbidden_optional_path"
    )
    if os.path.join(source_root, relative_path) != absolute_path:
        _fail("PATH_BINDING_INVALID", "absolute and relative selected paths differ")
    if absolute_path == forbidden_optional:
        _fail("OPTIONAL_PATH_ALIAS", "primary and forbidden optional paths alias")
    if Path(relative_path).parts != (selection["attempt_id"], "capture.bag"):
        _fail("ATTEMPT_PATH_MISMATCH", "attempt id is not bound to capture.bag")
    if selection["selected_role"] != "PRIMARY" or selection["selected_count"] != 1:
        _fail("SELECTION_ROLE_INVALID", "only one PRIMARY is admitted")
    if selection["optional_pair_authorized"] is not False:
        _fail("OPTIONAL_PAIR_NOT_AUTHORIZED", "optional pair must remain disabled")
    if not isinstance(selection["expected_size_bytes"], int) or isinstance(
        selection["expected_size_bytes"], bool
    ) or selection["expected_size_bytes"] <= 0:
        _fail("EXPECTED_SIZE_INVALID", "expected size must be a positive integer")
    if not isinstance(selection["expected_mode"], str) or len(selection["expected_mode"]) != 4:
        _fail("EXPECTED_MODE_INVALID", "expected mode must be four octal digits")
    try:
        int(selection["expected_mode"], 8)
    except ValueError:
        _fail("EXPECTED_MODE_INVALID", "expected mode is not octal")
    if not _is_lower_hex(selection["expected_sha256"], 64):
        _fail("EXPECTED_SHA256_INVALID", "expected SHA-256 is invalid")

    filesystem = policy["filesystem_contract"]
    if not isinstance(filesystem, Mapping):
        _fail("FILESYSTEM_POLICY_INVALID", "filesystem contract is not an object")
    required_true = {
        "exact_path_only",
        "glob_forbidden",
        "basename_search_forbidden",
        "latest_file_selection_forbidden",
        "regular_file_required",
        "parent_symlinks_forbidden",
        "special_files_forbidden",
        "active_marker_forbidden",
        "source_bag_execution_forbidden",
    }
    if any(filesystem.get(name) is not True for name in required_true):
        _fail("FILESYSTEM_POLICY_INVALID", "a fail-closed filesystem control is disabled")
    if filesystem.get("nlink_required") != 1 or filesystem.get("source_root_writes_allowed") != 0:
        _fail("FILESYSTEM_POLICY_INVALID", "nlink or source-write ceiling differs")
    if filesystem.get("selected_attempt_inventory") != ["capture.bag"]:
        _fail("FILESYSTEM_POLICY_INVALID", "selected attempt inventory is not exact")

    topic_contract = policy["topic_contract"]
    if not isinstance(topic_contract, Mapping):
        _fail("TOPIC_POLICY_INVALID", "topic contract is not an object")
    topics = topic_contract.get("required_topics")
    if not isinstance(topics, list) or len(topics) != 13:
        _fail("TOPIC_POLICY_INVALID", "exactly 13 required topics are needed")
    names: set[str] = set()
    for index, item in enumerate(topics):
        if not isinstance(item, Mapping) or set(item) != {"topic", "type", "md5sum"}:
            _fail("TOPIC_POLICY_INVALID", f"topic contract row {index} is not closed")
        if not isinstance(item["topic"], str) or not item["topic"].startswith("/"):
            _fail("TOPIC_POLICY_INVALID", f"topic contract row {index} has an invalid name")
        if item["topic"] in names or not isinstance(item["type"], str) or not item["type"]:
            _fail("TOPIC_POLICY_INVALID", f"topic contract row {index} duplicates or lacks type")
        if not _is_lower_hex(item["md5sum"], 32):
            _fail("TOPIC_POLICY_INVALID", f"topic contract row {index} has invalid MD5")
        names.add(item["topic"])
    if topic_contract.get("allow_extra_topics") is not False:
        _fail("TOPIC_POLICY_INVALID", "extra topics must fail closed")
    for flag in (
        "record_time_witness_required",
        "clock_witness_required",
        "dynamic_tf_witness_required",
        "static_tf_witness_required",
        "definition_conflicts_forbidden",
    ):
        if topic_contract.get(flag) is not True:
            _fail("TOPIC_POLICY_INVALID", f"required topic control {flag} is disabled")
    if topic_contract.get("odom_primary_time") != "/odom.header.stamp":
        _fail("TOPIC_POLICY_INVALID", "odom header time is not the sole primary time")

    limits = policy["limits"]
    if not isinstance(limits, Mapping):
        _fail("LIMIT_POLICY_INVALID", "limits are not an object")
    for name in READER_LIMIT_KEYS + (
        "timeout_seconds",
        "maximum_rss_bytes",
        "maximum_address_space_bytes",
        "maximum_receipt_bytes",
        "maximum_output_files",
        "maximum_source_entries",
        "maximum_parent_components",
    ):
        value = limits.get(name)
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            _fail("LIMIT_POLICY_INVALID", f"limit {name} is not a positive integer")
    if limits["maximum_output_files"] != 1:
        _fail("LIMIT_POLICY_INVALID", "only one receipt output is allowed")
    reader_limits_from_policy(policy).validate()

    sandbox = policy["sandbox_contract"]
    if not isinstance(sandbox, Mapping):
        _fail("SANDBOX_POLICY_INVALID", "sandbox contract is not an object")
    if sandbox.get("backend") != "BWRAP_READ_ONLY_UNSHARED_NETWORK":
        _fail("SANDBOX_POLICY_INVALID", "unexpected sandbox backend")
    for flag in (
        "network_unshared_required",
        "read_only_bind_required",
        "clear_environment_required",
    ):
        if sandbox.get(flag) is not True:
            _fail("SANDBOX_POLICY_INVALID", f"sandbox control {flag} is disabled")
    for flag in ("gpu_devices_exposed", "ros_started", "solver_started", "source_bag_executed"):
        if sandbox.get(flag) is not False:
            _fail("SANDBOX_POLICY_INVALID", f"forbidden runtime flag {flag} is enabled")
    for name in (
        "bwrap_path",
        "timeout_path",
        "prlimit_path",
        "python_path",
        "source_visible_path",
        "policy_visible_path",
        "receipt_visible_root",
        "audit_root",
    ):
        _absolute_normalized(sandbox.get(name), f"sandbox_contract.{name}")

    claims = policy["claim_ceiling"]
    if not isinstance(claims, Mapping):
        _fail("CLAIM_CEILING_INVALID", "claim ceiling is not an object")
    if claims.get("development_only") is not True or claims.get("source_outcome") != "UNKNOWN":
        _fail("CLAIM_CEILING_INVALID", "development/source-outcome ceiling differs")
    for flag in ("formal", "physical_robot_bag", "physical_primary_eligible", "r8_release"):
        if claims.get(flag) is not False:
            _fail("CLAIM_CEILING_INVALID", f"claim {flag} was promoted")


def validate_frozen_policy(policy: Mapping[str, Any]) -> None:
    validate_policy(policy)
    selection = policy["selection"]
    expected = {
        "policy_id": EXPECTED_POLICY_ID,
        "attempt_id": EXPECTED_ATTEMPT,
        "source_root": EXPECTED_SOURCE_ROOT,
        "absolute_path": EXPECTED_PRIMARY,
        "forbidden_optional_path": EXPECTED_OPTIONAL,
        "expected_size_bytes": EXPECTED_SIZE,
        "expected_mode": EXPECTED_MODE,
        "expected_sha256": EXPECTED_SHA256,
    }
    actual = {
        "policy_id": policy["policy_id"],
        "attempt_id": selection["attempt_id"],
        "source_root": selection["source_root"],
        "absolute_path": selection["absolute_path"],
        "forbidden_optional_path": selection["forbidden_optional_path"],
        "expected_size_bytes": selection["expected_size_bytes"],
        "expected_mode": selection["expected_mode"],
        "expected_sha256": selection["expected_sha256"],
    }
    if actual != expected:
        _fail("FROZEN_POLICY_DRIFT", f"frozen primary identity differs: {actual!r}")


def reader_limits_from_policy(policy: Mapping[str, Any]) -> bag_reader.ReaderLimits:
    limits = policy["limits"]
    return bag_reader.ReaderLimits(**{name: limits[name] for name in READER_LIMIT_KEYS})


def validate_selected_path(policy: Mapping[str, Any], candidate: str) -> Path:
    validate_policy(policy)
    normalized = _absolute_normalized(candidate, "selected bag candidate")
    selection = policy["selection"]
    if normalized == selection["forbidden_optional_path"]:
        _fail("OPTIONAL_PAIR_NOT_AUTHORIZED", "optional bag was not authorized")
    if normalized != selection["absolute_path"]:
        _fail("EXACT_PATH_MISMATCH", "candidate differs from the frozen exact primary path")
    return Path(normalized)


def _identity(path: Path, metadata: os.stat_result) -> dict[str, Any]:
    if stat.S_ISDIR(metadata.st_mode):
        kind = "directory"
    elif stat.S_ISREG(metadata.st_mode):
        kind = "regular"
    else:
        _fail("SPECIAL_PATH_FORBIDDEN", f"special path is forbidden: {path}")
    return {
        "path": str(path),
        "kind": kind,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
        "size_bytes": metadata.st_size,
        "mtime_ns": metadata.st_mtime_ns,
        "ctime_ns": metadata.st_ctime_ns,
        "symlink": False,
    }


def audit_path_chain(path: Path, maximum_components: int) -> list[dict[str, Any]]:
    raw = _absolute_normalized(str(path), "path-chain target")
    parts = Path(raw).parts
    if len(parts) > maximum_components:
        _fail("PATH_CHAIN_TOO_DEEP", "path has too many components")
    current = Path(parts[0])
    result: list[dict[str, Any]] = []
    for index, component in enumerate(parts):
        if index:
            current = current / component
        try:
            metadata = os.lstat(current)
        except OSError as exc:
            _fail("PATH_COMPONENT_UNREADABLE", f"cannot lstat {current}: {exc}")
        if stat.S_ISLNK(metadata.st_mode):
            _fail("SYMLINK_FORBIDDEN", f"symlink path component is forbidden: {current}")
        identity = _identity(current, metadata)
        if index < len(parts) - 1 and identity["kind"] != "directory":
            _fail("PARENT_NOT_DIRECTORY", f"parent component is not a directory: {current}")
        result.append(identity)
    if result[-1]["kind"] != "regular":
        _fail("SOURCE_NOT_REGULAR", "selected bag is not a regular file")
    if result[-1]["nlink"] != 1:
        _fail("SOURCE_NLINK_INVALID", "selected bag nlink is not exactly one")
    return result


def validate_attempt_inventory(source_path: Path, expected: Sequence[str]) -> None:
    attempt_dir = source_path.parent
    try:
        with os.scandir(attempt_dir) as iterator:
            names = sorted(entry.name for entry in iterator)
    except OSError as exc:
        _fail("ATTEMPT_INVENTORY_FAILED", f"cannot inventory selected attempt: {exc}")
    if ".active" in names or any(name.endswith(".active") for name in names):
        _fail("ACTIVE_MARKER_FORBIDDEN", "selected attempt contains an active marker")
    if names != sorted(expected):
        _fail("ATTEMPT_INVENTORY_INVALID", f"selected attempt inventory differs: {names!r}")


def _snapshot_entry(root: Path, path: Path, metadata: os.stat_result) -> dict[str, Any]:
    if stat.S_ISDIR(metadata.st_mode):
        kind = "directory"
    elif stat.S_ISREG(metadata.st_mode):
        kind = "regular"
        if metadata.st_nlink != 1:
            _fail("SOURCE_NLINK_INVALID", f"source file has nlink != 1: {path}")
    elif stat.S_ISLNK(metadata.st_mode):
        _fail("SYMLINK_FORBIDDEN", f"source-root symlink is forbidden: {path}")
    else:
        _fail("SPECIAL_PATH_FORBIDDEN", f"source-root special file is forbidden: {path}")
    return {
        "relative_path": path.relative_to(root).as_posix(),
        "kind": kind,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
        "size_bytes": metadata.st_size,
        "mtime_ns": metadata.st_mtime_ns,
        "ctime_ns": metadata.st_ctime_ns,
    }


def snapshot_source_root(root: Path, maximum_entries: int) -> dict[str, Any]:
    root = Path(_absolute_normalized(str(root), "source snapshot root"))
    try:
        root_stat = os.lstat(root)
    except OSError as exc:
        _fail("SOURCE_ROOT_UNREADABLE", f"cannot lstat source root: {exc}")
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        _fail("SOURCE_ROOT_INVALID", "source root is not a non-symlink directory")
    entries: list[dict[str, Any]] = []
    pending = [root]
    while pending:
        directory = pending.pop()
        try:
            with os.scandir(directory) as iterator:
                children = sorted(iterator, key=lambda entry: entry.name, reverse=True)
        except OSError as exc:
            _fail("SOURCE_ROOT_INVENTORY_FAILED", f"cannot scan {directory}: {exc}")
        for child in children:
            try:
                metadata = child.stat(follow_symlinks=False)
            except OSError as exc:
                _fail("SOURCE_ROOT_INVENTORY_FAILED", f"cannot stat {child.path}: {exc}")
            row = _snapshot_entry(root, Path(child.path), metadata)
            entries.append(row)
            if len(entries) > maximum_entries:
                _fail("SOURCE_ROOT_ENTRY_LIMIT", "source-root inventory exceeds its bound")
            if row["kind"] == "directory":
                pending.append(Path(child.path))
    entries.sort(key=lambda row: row["relative_path"])
    if not entries:
        _fail("SOURCE_ROOT_EMPTY", "source-root inventory is empty")
    return {
        "root": str(root),
        "entry_count": len(entries),
        "sha256": canonical_sha256(entries),
        "entries": entries,
    }


def _unescape_mountinfo(value: str) -> str:
    for encoded, decoded in (("\\040", " "), ("\\011", "\t"), ("\\012", "\n"), ("\\134", "\\")):
        value = value.replace(encoded, decoded)
    return value


def inspect_read_only_mount(
    path: Path,
    mountinfo_text: str,
    *,
    statvfs_flags: int,
) -> dict[str, Any]:
    selected = _absolute_normalized(str(path), "mount target")
    candidates: list[dict[str, Any]] = []
    for line in mountinfo_text.splitlines():
        fields = line.split()
        if "-" not in fields:
            continue
        separator = fields.index("-")
        if separator < 6 or len(fields) <= separator + 3:
            continue
        mount_point = _unescape_mountinfo(fields[4])
        prefix = mount_point.rstrip("/") + "/"
        if selected != mount_point and not selected.startswith(prefix):
            continue
        try:
            mount_id = int(fields[0])
        except ValueError:
            continue
        candidates.append(
            {
                "mount_id": mount_id,
                "device": fields[2],
                "root": _unescape_mountinfo(fields[3]),
                "mount_point": mount_point,
                "filesystem_type": fields[separator + 1],
                "mount_source": _unescape_mountinfo(fields[separator + 2]),
                "mount_options": sorted(fields[5].split(",")),
                "super_options": sorted(fields[separator + 3].split(",")),
            }
        )
    if not candidates:
        _fail("MOUNT_IDENTITY_MISSING", "no mountinfo row covers the selected bag")
    mount = max(candidates, key=lambda item: len(item["mount_point"]))
    mount_ro = "ro" in mount["mount_options"] and "rw" not in mount["mount_options"]
    statvfs_ro = bool(statvfs_flags & getattr(os, "ST_RDONLY", 1))
    if not mount_ro or not statvfs_ro:
        _fail("READ_ONLY_MOUNT_NOT_ENFORCED", "mountinfo/statvfs do not both prove read-only")
    return {**mount, "mountinfo_read_only": True, "statvfs_read_only": True}


def validate_topic_contract(policy: Mapping[str, Any], bag: Mapping[str, Any]) -> dict[str, Any]:
    if bag.get("format") != "ROS1_BAG_V2" or bag.get("version") != "2.0":
        _fail("BAG_FORMAT_INVALID", "selected source is not ROS1 Bag V2")
    if bag.get("indexed") is not True or bag.get("index_verified") is not True:
        _fail("BAG_INDEX_INVALID", "ROS1 Bag V2 index is not exact")
    if bag.get("chunk_info_verified") is not True:
        _fail("BAG_CHUNK_INFO_INVALID", "chunk-info cross-check failed")
    if bag.get("topic_conflicts") != []:
        _fail("TOPIC_DEFINITION_CONFLICT", "reader reported a topic definition conflict")
    if bag.get("anomalies") != []:
        _fail("BAG_TIME_ANOMALY", f"reader anomalies are non-empty: {bag.get('anomalies')!r}")

    required_rows = policy["topic_contract"]["required_topics"]
    required = {row["topic"]: row for row in required_rows}
    observed_rows = bag.get("topics")
    if not isinstance(observed_rows, list):
        _fail("TOPIC_CENSUS_INVALID", "reader topic census is absent")
    observed: dict[str, Mapping[str, Any]] = {}
    for row in observed_rows:
        if not isinstance(row, Mapping) or not isinstance(row.get("topic"), str):
            _fail("TOPIC_CENSUS_INVALID", "reader topic row is malformed")
        if row["topic"] in observed:
            _fail("TOPIC_CENSUS_INVALID", f"reader repeats topic {row['topic']}")
        observed[row["topic"]] = row
    missing = sorted(set(required) - set(observed))
    unexpected = sorted(set(observed) - set(required))
    if missing:
        _fail("REQUIRED_TOPIC_MISSING", ", ".join(missing))
    if unexpected:
        _fail("UNEXPECTED_TOPIC", ", ".join(unexpected))

    for name, expected in required.items():
        row = observed[name]
        if row.get("type") != expected["type"] or row.get("md5sum") != expected["md5sum"]:
            _fail("TOPIC_SCHEMA_MISMATCH", f"type/MD5 differs for {name}")
        if not _is_lower_hex(row.get("message_definition_sha256"), 64):
            _fail("TOPIC_DEFINITION_HASH_INVALID", f"definition hash is invalid for {name}")
        if not isinstance(row.get("message_count"), int) or row["message_count"] <= 0:
            _fail("TOPIC_MESSAGE_COUNT_INVALID", f"message count is invalid for {name}")
        time_range = row.get("record_time_range")
        if not isinstance(time_range, Mapping) or time_range.get("monotonic") is not True:
            _fail("TOPIC_RECORD_TIME_INVALID", f"record time is non-monotonic for {name}")

    definitions: dict[str, set[tuple[str, str, str]]] = {}
    connections = bag.get("connections")
    if not isinstance(connections, list):
        _fail("CONNECTION_CENSUS_INVALID", "connection census is absent")
    for row in connections:
        if not isinstance(row, Mapping) or row.get("topic") not in required:
            _fail("CONNECTION_CENSUS_INVALID", "connection has an unregistered topic")
        definitions.setdefault(row["topic"], set()).add(
            (str(row.get("type")), str(row.get("md5sum")), str(row.get("message_definition_sha256")))
        )
    conflicts = sorted(name for name, values in definitions.items() if len(values) != 1)
    if conflicts:
        _fail("TOPIC_DEFINITION_CONFLICT", ", ".join(conflicts))

    odom = bag.get("odom")
    if not isinstance(odom, Mapping) or odom.get("message_count", 0) <= 0:
        _fail("ODOM_WITNESS_MISSING", "decoded /odom witness is absent")
    if odom.get("record_time_monotonic") is not True or odom.get("header_time_monotonic") is not True:
        _fail("ODOM_TIME_INVALID", "/odom record/header time is non-monotonic")
    if not odom.get("frame_pairs"):
        _fail("ODOM_FRAME_WITNESS_MISSING", "/odom frame pair inventory is empty")
    clock = bag.get("clock")
    if not isinstance(clock, Mapping) or clock.get("message_count", 0) <= 0:
        _fail("CLOCK_WITNESS_MISSING", "decoded /clock witness is absent")
    if clock.get("record_time_monotonic") is not True or clock.get("clock_time_monotonic") is not True:
        _fail("CLOCK_TIME_INVALID", "/clock record/value time is non-monotonic")
    frame_graph = bag.get("frame_graph")
    if not isinstance(frame_graph, Mapping):
        _fail("FRAME_GRAPH_MISSING", "TF frame graph is absent")
    if not frame_graph.get("dynamic_edges") or not frame_graph.get("static_edges"):
        _fail("FRAME_GRAPH_MISSING", "dynamic or static TF edge inventory is empty")
    return {
        "contract_id": policy["topic_contract"]["contract_id"],
        "required_topic_count": len(required),
        "missing_topics": [],
        "unexpected_topics": [],
        "definition_conflicts": [],
        "all_record_times_monotonic": True,
        "validated": True,
    }


def validate_output_path(policy: Mapping[str, Any], receipt_path: str) -> Path:
    raw = _absolute_normalized(receipt_path, "receipt path")
    source_root = Path(policy["selection"]["source_root"])
    output = Path(raw)
    try:
        output.relative_to(source_root)
    except ValueError:
        pass
    else:
        _fail("OUTPUT_UNDER_SOURCE_ROOT", "receipt output is inside the source root")
    audit_root = Path(policy["sandbox_contract"]["audit_root"])
    if output.parent != audit_root:
        _fail("OUTPUT_ROOT_MISMATCH", "receipt must be a direct child of the frozen audit root")
    if output.name in {"", ".", ".."} or not output.name.endswith(".json"):
        _fail("OUTPUT_NAME_INVALID", "receipt name must be a concrete JSON basename")
    return output


def build_bwrap_argv(policy: Mapping[str, Any], receipt_path: str) -> list[str]:
    validate_policy(policy)
    validate_selected_path(policy, policy["selection"]["absolute_path"])
    output = validate_output_path(policy, receipt_path)
    sandbox = policy["sandbox_contract"]
    limits = policy["limits"]
    visible_receipt = str(Path(sandbox["receipt_visible_root"]) / output.name)
    argv = [
        sandbox["bwrap_path"],
        "--die-with-parent",
        "--new-session",
        "--unshare-all",
        "--unshare-net",
        "--clearenv",
        "--ro-bind",
        "/usr",
        "/usr",
        "--ro-bind",
        "/lib",
        "/lib",
        "--ro-bind",
        "/lib64",
        "/lib64",
        "--proc",
        "/proc",
        "--dev",
        "/dev",
        "--tmpfs",
        "/tmp",
        "--dir",
        "/app",
        "--ro-bind",
        str(GATE_PATH),
        "/app/gate.py",
        "--ro-bind",
        str(READER_PATH),
        "/app/r8_liquid_ros1_bag_v2_reader_v1.py",
        "--dir",
        "/policy",
        "--ro-bind",
        str(POLICY_PATH),
        sandbox["policy_visible_path"],
        "--dir",
        "/selected",
        "--ro-bind",
        policy["selection"]["absolute_path"],
        sandbox["source_visible_path"],
        "--dir",
        sandbox["receipt_visible_root"],
        "--bind",
        sandbox["audit_root"],
        sandbox["receipt_visible_root"],
        "--setenv",
        "PYTHONDONTWRITEBYTECODE",
        "1",
        "--setenv",
        "PYTHONHASHSEED",
        "0",
        "--chdir",
        "/app",
        "--",
        sandbox["timeout_path"],
        "--foreground",
        "--signal=KILL",
        "--kill-after=2s",
        f"{limits['timeout_seconds']}s",
        sandbox["prlimit_path"],
        f"--rss={limits['maximum_rss_bytes']}:{limits['maximum_rss_bytes']}",
        f"--as={limits['maximum_address_space_bytes']}:{limits['maximum_address_space_bytes']}",
        "--nofile=64:64",
        "--nproc=1:1",
        sandbox["python_path"],
        "-I",
        "-B",
        "/app/gate.py",
        "sandbox-inspect",
        "--policy",
        sandbox["policy_visible_path"],
        "--source",
        sandbox["source_visible_path"],
        "--receipt",
        visible_receipt,
    ]
    if policy["selection"]["forbidden_optional_path"] in argv:
        _fail("OPTIONAL_PATH_EXPOSED", "optional bag leaked into the sandbox argv")
    return argv


def _same_path_chain(left: Sequence[Mapping[str, Any]], right: Sequence[Mapping[str, Any]]) -> bool:
    return list(left) == list(right)


def inspect_selected(
    policy: Mapping[str, Any],
    *,
    source_path: Path,
    receipt_path: Path,
    mountinfo_text: str,
    statvfs_flags: int,
    policy_sha256: str | None = None,
    captured_at_utc: str = "2000-01-01T00:00:00Z",
    snapshot_root: Path | None = None,
    source_is_sandbox_visible_path: bool = False,
) -> dict[str, Any]:
    """Inspect one fixture or sandbox-visible exact bag without writing output."""

    validate_policy(policy)
    if source_is_sandbox_visible_path:
        expected_visible = policy["sandbox_contract"]["source_visible_path"]
        if str(source_path) != expected_visible:
            _fail("SANDBOX_SOURCE_PATH_MISMATCH", "sandbox source path differs")
    else:
        validate_selected_path(policy, str(source_path))
    output_host_path = validate_output_path(policy, str(receipt_path))
    limits = policy["limits"]
    root = snapshot_root or source_path.parent.parent
    try:
        output_host_path.relative_to(root)
    except ValueError:
        pass
    else:
        _fail("OUTPUT_UNDER_SOURCE_ROOT", "receipt output aliases the observed source root")

    path_before = audit_path_chain(source_path, limits["maximum_parent_components"])
    validate_attempt_inventory(
        source_path, policy["filesystem_contract"]["selected_attempt_inventory"]
    )
    directory_before = snapshot_source_root(root, limits["maximum_source_entries"])
    mount = inspect_read_only_mount(
        source_path, mountinfo_text, statvfs_flags=statvfs_flags
    )
    reader_limits = reader_limits_from_policy(policy)
    try:
        inspection = bag_reader.inspect_bag(
            source_path,
            limits=reader_limits,
            expected_sha256=policy["selection"]["expected_sha256"],
            expected_size_bytes=policy["selection"]["expected_size_bytes"],
            expected_mode=int(policy["selection"]["expected_mode"], 8),
        )
    except bag_reader.BagV2Error as exc:
        _fail("ROS1_BAG_V2_REJECTED", str(exc))
    topic_result = validate_topic_contract(policy, inspection["bag"])
    validate_attempt_inventory(
        source_path, policy["filesystem_contract"]["selected_attempt_inventory"]
    )
    directory_after = snapshot_source_root(root, limits["maximum_source_entries"])
    path_after = audit_path_chain(source_path, limits["maximum_parent_components"])
    if directory_before != directory_after:
        _fail("SOURCE_ROOT_CHANGED", "source-root metadata inventory changed during intake")
    if not _same_path_chain(path_before, path_after):
        _fail("SOURCE_PATH_CHANGED", "source path chain changed during intake")
    if inspection["source_before"] != inspection["source_after"]:
        _fail("SOURCE_FILE_CHANGED", "selected file identity changed during intake")

    argv = build_bwrap_argv(policy, str(output_host_path))
    selection = policy["selection"]
    claims = policy["claim_ceiling"]
    runtime_limits = {
        "timeout_seconds": limits["timeout_seconds"],
        "maximum_rss_bytes": limits["maximum_rss_bytes"],
        "maximum_address_space_bytes": limits["maximum_address_space_bytes"],
        "maximum_receipt_bytes": limits["maximum_receipt_bytes"],
        "maximum_output_files": limits["maximum_output_files"],
        "maximum_chunk_uncompressed_bytes": limits["maximum_chunk_uncompressed_bytes"],
        "maximum_total_uncompressed_bytes": limits["maximum_total_uncompressed_bytes"],
    }
    return {
        "schema_version": RECEIPT_SCHEMA_VERSION,
        "document_type": "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V1",
        "receipt_id": f"S5A0-{selection['attempt_id']}-PRIMARY-v1",
        "captured_at_utc": captured_at_utc,
        "status": PASS_STATUS,
        "policy": {
            "policy_id": policy["policy_id"],
            "policy_sha256": policy_sha256 or canonical_sha256(policy),
            "continuation_plan_sha256": policy["parents"]["continuation_plan_sha256"],
            "reader_sha256": policy["parents"]["reader_sha256"],
        },
        "selection": {
            "source_domain": selection["source_domain"],
            "source_provenance": selection["source_provenance"],
            "source_root": selection["source_root"],
            "attempt_id": selection["attempt_id"],
            "selected_role": selection["selected_role"],
            "selected_count": selection["selected_count"],
            "relative_path": selection["relative_path"],
            "absolute_path": selection["absolute_path"],
            "optional_pair_read_or_admitted": False,
        },
        "sandbox": {
            "backend": policy["sandbox_contract"]["backend"],
            "read_only_enforced": True,
            "network_unshared": True,
            "clear_environment": True,
            "source_visible_path": str(source_path),
            "source_bag_executed": False,
            "ros_started": False,
            "gpu_exposed": False,
            "solver_started": False,
            "bwrap_argv_sha256": canonical_sha256(argv),
            "limits": runtime_limits,
        },
        "mount": mount,
        "path_chain": {
            "before": path_before,
            "after": path_after,
            "unchanged": True,
        },
        "source": {
            "directory_before": directory_before,
            "directory_after": directory_after,
            "directory_unchanged": True,
            "selected_before": inspection["source_before"],
            "selected_after": inspection["source_after"],
            "selected_unchanged": True,
            "active_marker_present": False,
            "files_written_under_source_root": 0,
        },
        "bag": inspection["bag"],
        "topic_contract": topic_result,
        "claims": {
            "development_only": claims["development_only"],
            "formal": claims["formal"],
            "physical_robot_bag": claims["physical_robot_bag"],
            "physical_primary_eligible": claims["physical_primary_eligible"],
            "r8_release": claims["r8_release"],
            "source_outcome": claims["source_outcome"],
        },
        "output": {
            "receipt_path": str(output_host_path),
            "create_new": True,
            "o_excl": True,
            "o_nofollow": True,
            "mode": "0440",
            "maximum_output_files": limits["maximum_output_files"],
            "maximum_receipt_bytes": limits["maximum_receipt_bytes"],
        },
    }


def write_receipt_create_new(
    path: Path,
    receipt: Mapping[str, Any],
    *,
    maximum_bytes: int,
) -> dict[str, Any]:
    """Write exactly one bounded receipt without following or replacing a path."""

    path = Path(_absolute_normalized(str(path), "receipt write path"))
    source_root = Path(receipt["selection"]["source_root"])
    try:
        path.relative_to(source_root)
    except ValueError:
        pass
    else:
        _fail("OUTPUT_UNDER_SOURCE_ROOT", "receipt write target is inside the source root")
    try:
        parent_stat = os.lstat(path.parent)
    except OSError as exc:
        _fail("OUTPUT_PARENT_INVALID", f"cannot lstat receipt parent: {exc}")
    if stat.S_ISLNK(parent_stat.st_mode) or not stat.S_ISDIR(parent_stat.st_mode):
        _fail("OUTPUT_PARENT_INVALID", "receipt parent is not a non-symlink directory")
    raw = _canonical_bytes(receipt) + b"\n"
    if len(raw) > maximum_bytes:
        _fail("RECEIPT_SIZE_LIMIT", "serialized receipt exceeds the output bound")
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(path, flags, 0o440)
    except OSError as exc:
        _fail("RECEIPT_CREATE_NEW_FAILED", f"cannot create-new receipt: {exc}")
    try:
        os.fchmod(descriptor, 0o440)
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            if written <= 0:
                _fail("RECEIPT_WRITE_FAILED", "receipt write made no progress")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        _fail("RECEIPT_IDENTITY_INVALID", "new receipt is not a single-link regular file")
    return {
        "path": str(path),
        "bytes": len(raw),
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "sha256": _sha256_bytes(raw),
    }


def assert_deep_closed(schema: object, location: str = "$") -> None:
    if isinstance(schema, dict):
        if schema.get("type") == "object" or "properties" in schema:
            if schema.get("additionalProperties") is not False:
                _fail("SCHEMA_NOT_DEEP_CLOSED", f"object schema is open at {location}")
        for key, value in schema.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(schema, list):
        for index, value in enumerate(schema):
            assert_deep_closed(value, f"{location}/{index}")


def _sha256_file_bounded(path: Path, maximum_bytes: int, label: str) -> str:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        _fail("STATIC_PARENT_READ_FAILED", f"cannot read {label}: {exc}")
    if len(raw) > maximum_bytes:
        _fail("STATIC_PARENT_OVERSIZED", f"{label} exceeds its bound")
    return _sha256_bytes(raw)


def self_check() -> dict[str, Any]:
    """Validate only repository policy/schema/parents; never touch a source bag."""

    policy, policy_sha256 = load_policy()
    validate_frozen_policy(policy)
    schema = load_schema()
    assert_deep_closed(schema)
    parent_plan_hash = _sha256_file_bounded(
        Path(policy["parents"]["continuation_plan_path"]), 2 * 1024 * 1024, "continuation plan"
    )
    reader_hash = _sha256_file_bounded(READER_PATH, 2 * 1024 * 1024, "ROS1 bag reader")
    if parent_plan_hash != policy["parents"]["continuation_plan_sha256"]:
        _fail("CONTINUATION_PLAN_HASH_DRIFT", "continuation plan hash differs")
    if reader_hash != policy["parents"]["reader_sha256"]:
        _fail("READER_HASH_DRIFT", "ROS1 bag reader hash differs")
    planned_receipt = (
        Path(policy["sandbox_contract"]["audit_root"])
        / f"s5a0_{policy['selection']['attempt_id']}_primary_receipt_v1.json"
    )
    argv = build_bwrap_argv(policy, str(planned_receipt))
    delimiter = argv.index("--")
    if "--unshare-net" not in argv[:delimiter] or "--clearenv" not in argv[:delimiter]:
        _fail("BWRAP_NETWORK_POLICY_INVALID", "bwrap argv lacks network/env isolation")
    if argv.count(policy["selection"]["absolute_path"]) != 1:
        _fail("BWRAP_SOURCE_BIND_INVALID", "exact primary is not bound exactly once")
    return {
        "status": "PASS_S5A0_SELECTED_BAG_STATIC_CONTRACT_V1",
        "policy_id": policy["policy_id"],
        "policy_sha256": policy_sha256,
        "schema_deep_closed": True,
        "selected_role": "PRIMARY",
        "selected_count": 1,
        "optional_pair_authorized": False,
        "real_bag_opened": False,
        "bwrap_executed": False,
        "ros_started": False,
        "gpu_exposed": False,
        "solver_started": False,
        "bwrap_argv_sha256": canonical_sha256(argv),
    }


def _sandbox_inspect(policy_path: Path, source_path: Path, receipt_path: Path) -> dict[str, Any]:
    policy, policy_sha256 = load_policy(policy_path)
    validate_frozen_policy(policy)
    if str(source_path) != policy["sandbox_contract"]["source_visible_path"]:
        _fail("SANDBOX_SOURCE_PATH_MISMATCH", "worker source path differs from policy")
    if str(receipt_path.parent) != policy["sandbox_contract"]["receipt_visible_root"]:
        _fail("SANDBOX_RECEIPT_PATH_MISMATCH", "worker receipt root differs from policy")
    reader_hash = _sha256_file_bounded(READER_PATH, 2 * 1024 * 1024, "sandbox reader")
    if reader_hash != policy["parents"]["reader_sha256"]:
        _fail("READER_HASH_DRIFT", "sandbox reader hash differs")
    try:
        mountinfo_text = Path("/proc/self/mountinfo").read_text(encoding="utf-8")
        statvfs_flags = os.statvfs(source_path).f_flag
    except OSError as exc:
        _fail("SANDBOX_MOUNT_PROBE_FAILED", str(exc))
    host_receipt = Path(policy["sandbox_contract"]["audit_root"]) / receipt_path.name
    receipt = inspect_selected(
        policy,
        source_path=source_path,
        receipt_path=host_receipt,
        mountinfo_text=mountinfo_text,
        statvfs_flags=statvfs_flags,
        policy_sha256=policy_sha256,
        captured_at_utc=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        snapshot_root=source_path.parent,
        source_is_sandbox_visible_path=True,
    )
    return write_receipt_create_new(
        receipt_path,
        receipt,
        maximum_bytes=policy["limits"]["maximum_receipt_bytes"],
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check", help="validate static files without touching a bag")
    plan = subparsers.add_parser("plan-bwrap", help="print argv only; never execute it")
    plan.add_argument("--receipt", required=True)
    worker = subparsers.add_parser("sandbox-inspect", help=argparse.SUPPRESS)
    worker.add_argument("--policy", required=True)
    worker.add_argument("--source", required=True)
    worker.add_argument("--receipt", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            result = self_check()
        elif arguments.command == "plan-bwrap":
            policy, _ = load_policy()
            validate_frozen_policy(policy)
            result = {"argv": build_bwrap_argv(policy, arguments.receipt), "executed": False}
        else:
            result = _sandbox_inspect(
                Path(arguments.policy), Path(arguments.source), Path(arguments.receipt)
            )
    except S5A0GateError as exc:
        print(json.dumps({"status": "STOP_AND_PRESERVE_EVIDENCE", "code": exc.code, "error": exc.message}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "EXPECTED_OPTIONAL",
    "EXPECTED_PRIMARY",
    "PASS_STATUS",
    "POLICY_PATH",
    "SCHEMA_PATH",
    "S5A0GateError",
    "assert_deep_closed",
    "audit_path_chain",
    "build_bwrap_argv",
    "canonical_sha256",
    "inspect_read_only_mount",
    "inspect_selected",
    "load_policy",
    "load_schema",
    "reader_limits_from_policy",
    "self_check",
    "snapshot_source_root",
    "validate_frozen_policy",
    "validate_policy",
    "validate_selected_path",
    "validate_topic_contract",
    "write_receipt_create_new",
]
