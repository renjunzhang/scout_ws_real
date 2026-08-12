#!/usr/bin/env python3
"""S5A1 ABI-v3 gate adapter for the closed S5A0 inner receipt v2.

The package semantics remain the frozen handoff-gate v1 contract.  This
revision only admits the private-tmpfs S5A0 receipt shape and projects its
separately verified host provenance into the v1 producer's host identity
view.  The guest copy identity is still checked and never treated as a host
path.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_s5a1_handoff_gate_v1 as v1


sys.dont_write_bytecode = True
PACKAGE_ROOT = MODULE_DIR.parent
S5A0_RECEIPT_SCHEMA_PATH = (
    PACKAGE_ROOT / "schema/target_host_s5a0_selected_bag_receipt_v2.json"
)
S5A0_RECEIPT_SCHEMA_SHA256 = (
    "ec78b8e62d9e31fd7adedeaa8a46b10645fce45c7f9a4e80f23eb6f825310976"
)
_V1_VALIDATE_S5A0_RECEIPT = v1.validate_s5a0_receipt
_V1_S5A0_ODOM_MESSAGE_COUNT = v1.s5a0_odom_message_count


def _exact_host_projection(
    receipt: Mapping[str, Any], policy: Mapping[str, Any]
) -> dict[str, Any]:
    source = policy["source"]
    provenance = receipt["host_source_provenance"]
    path = Path(provenance["absolute_path"])
    metadata = os.lstat(path)
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_nlink != 1
        or f"{stat.S_IMODE(metadata.st_mode):04o}" != source["bag_mode"]
        or metadata.st_size != source["bag_size_bytes"]
    ):
        raise v1.HandoffError("S5A0 host provenance path identity differs")
    projected = copy.deepcopy(dict(receipt))
    projected["source"]["selected_after"] = {
        "path": str(path),
        "sha256": provenance["expected_sha256"],
        "size_bytes": metadata.st_size,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
    }
    return projected


def validate_s5a0_receipt(
    raw: bytes, path: str, policy: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, str]]:
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise v1.HandoffError("S5A0 receipt JSON is invalid") from exc
    if value.get("document_type") != "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V2":
        return _V1_VALIDATE_S5A0_RECEIPT(raw, path, policy)
    if not path.startswith("/") or "\x00" in path or not 0 < len(raw) <= v1.MAX_JSON_BYTES:
        raise v1.HandoffError("S5A0 v2 receipt path/size is invalid")
    schema_raw = S5A0_RECEIPT_SCHEMA_PATH.read_bytes()
    if hashlib.sha256(schema_raw).hexdigest() != S5A0_RECEIPT_SCHEMA_SHA256:
        raise v1.HandoffError("S5A0 v2 receipt schema hash drifted")
    schema = json.loads(schema_raw)
    v1.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(value)

    source = policy["source"]
    selection = value["selection"]
    provenance = value["host_source_provenance"]
    guest_before = value["source"]["selected_before"]
    guest_after = value["source"]["selected_after"]
    if (
        value["status"] != source["required_s5a0_status"]
        or selection["source_domain"] != source["domain"]
        or selection["attempt_id"] != source["attempt_id"]
        or selection["selected_role"] != source["role"]
        or selection["absolute_path"] != source["bag_absolute_path"]
        or selection["relative_path"] != source["bag_relative_path"]
        or selection["selected_count"] != 1
        or selection["optional_pair_read_or_admitted"] is not False
        or provenance["absolute_path"] != source["bag_absolute_path"]
        or provenance["expected_mode"] != source["bag_mode"]
        or provenance["expected_size_bytes"] != source["bag_size_bytes"]
        or provenance["expected_sha256"] != source["bag_sha256"]
        or provenance["guest_copy_mode"] != "0400"
        or provenance["guest_copy_distinct_inode"] is not True
    ):
        raise v1.HandoffError("S5A0 v2 primary/provenance identity differs")
    for guest in (guest_before, guest_after):
        if (
            guest["path"] != "/selected/capture.bag"
            or guest["mode"] != "0400"
            or guest["size_bytes"] != source["bag_size_bytes"]
            or guest["sha256"] != source["bag_sha256"]
            or guest["nlink"] != 1
        ):
            raise v1.HandoffError("S5A0 v2 private guest identity differs")
    sandbox = value["sandbox"]
    source_evidence = value["source"]
    topic = value["topic_contract"]
    bag = value["bag"]
    if (
        sandbox["read_only_enforced"] is not True
        or sandbox["network_unshared"] is not True
        or sandbox["source_bag_executed"] is not False
        or sandbox["ros_started"] is not False
        or sandbox["gpu_exposed"] is not False
        or sandbox["solver_started"] is not False
        or source_evidence["selected_unchanged"] is not True
        or source_evidence["directory_unchanged"] is not True
        or source_evidence["active_marker_present"] is not False
        or source_evidence["files_written_under_source_root"] != 0
        or topic["validated"] is not True
        or topic["missing_topics"]
        or topic["unexpected_topics"]
        or topic["definition_conflicts"]
        or bag["format"] != "ROS1_BAG_V2"
        or bag["indexed"] is not True
        or bag["index_verified"] is not True
        or bag["chunk_info_verified"] is not True
        or bag["topic_conflicts"]
        or bag["anomalies"]
    ):
        raise v1.HandoffError("S5A0 v2 safety/topic/index evidence did not pass")
    projected = _exact_host_projection(value, policy)
    return projected, {"path": path, "sha256": v1.sha256_bytes(raw)}


def s5a0_odom_message_count(receipt: Mapping[str, Any]) -> int:
    if receipt.get("document_type") == "SMPCC_R8_LIQUID_S5A0_SELECTED_BAG_RECEIPT_V2":
        value = receipt["bag"]["odom"]["message_count"]
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise v1.HandoffError("S5A0 v2 odom count is invalid")
        return value
    return _V1_S5A0_ODOM_MESSAGE_COUNT(receipt)


# Reused v1 builders resolve these names in the v1 module's global scope.
v1.validate_s5a0_receipt = validate_s5a0_receipt
v1.s5a0_odom_message_count = s5a0_odom_message_count


def self_check() -> dict[str, Any]:
    result = v1.self_check()
    return {
        **result,
        "status": "PASS_S5A1_HANDOFF_GATE_V2_STATIC_CONTRACT",
        "s5a0_inner_receipt_revision": "v2",
        "s5a0_inner_schema_sha256": S5A0_RECEIPT_SCHEMA_SHA256,
        "real_s5a0_receipt_read": False,
        "real_bag_read": False,
        "files_written": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except (v1.HandoffError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


def __getattr__(name: str) -> Any:
    return getattr(v1, name)


if __name__ == "__main__":
    raise SystemExit(main())
