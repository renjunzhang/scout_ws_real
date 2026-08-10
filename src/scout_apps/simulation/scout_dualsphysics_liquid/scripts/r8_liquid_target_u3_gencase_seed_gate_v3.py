#!/usr/bin/env python3
"""Fail-closed, non-executing materialization gate for the U3 C1M seed v3.

The only subprocess this gate can start is the fixed /usr/bin/git cat-file
operation used to stream two pinned blobs from the already audited bare
repository.  GenCase and every produced seed file remain read-only and
non-executable on the host.  The rejected v1 seed and fixed-boundary v2 seed
are inspected only as append-only, non-reusable evidence; none of their bytes
or paths are a v3 materialization source.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
LIQUID_ROOT = Path("/home/zrj/scout_liquid_lab")

SEED_ID = "u3_c1m_gencase_seed_v3_20260808T150802Z"
RUNTIME_PARENT = LIQUID_ROOT / "dependency/runtime"
SEED_ROOT = RUNTIME_PARENT / f"{SEED_ID}.partial"
INPUT_ROOT = SEED_ROOT / "input"
RECEIPT = LIQUID_ROOT / "audits" / f"{SEED_ID}.json"
PARTIAL_RECEIPT = RECEIPT.with_suffix(RECEIPT.suffix + ".partial")

V1_SEED_ID = "u3_c1_gencase_seed_20260807T031624Z"
V1_SEED_ROOT = RUNTIME_PARENT / f"{V1_SEED_ID}.partial"
V1_INPUT_ROOT = V1_SEED_ROOT / "input"
V1_RECEIPT = LIQUID_ROOT / "audits" / f"{V1_SEED_ID}.json"
V1_PARTIAL_RECEIPT = V1_RECEIPT.with_suffix(V1_RECEIPT.suffix + ".partial")
V1_RECEIPT_SHA256 = "19ccbcc3c945a3e457b9cce99611c061ac8e6dfaf099129920cb555818fd6b6a"
V1_PARTIAL_RECEIPT_SHA256 = "a265d086aead8e23054323f8892b6d99d02166ead35150ba65a7ce0fc6da5ac3"
V1_CASE_TEMPLATE_SHA256 = "ef57582f69714f622e5b8a348c7626d233c203ad0b1eb6332b58fb1ffc09fa1a"

V2_SEED_ID = "u3_c1_gencase_seed_v2_20260807T135341Z"
V2_SEED_ROOT = RUNTIME_PARENT / f"{V2_SEED_ID}.partial"
V2_INPUT_ROOT = V2_SEED_ROOT / "input"
V2_RECEIPT = LIQUID_ROOT / "audits" / f"{V2_SEED_ID}.json"
V2_PARTIAL_RECEIPT = V2_RECEIPT.with_suffix(V2_RECEIPT.suffix + ".partial")
V2_RECEIPT_SHA256 = "1bbf958dfe2f7ce026ce05d77e7ee2c2516c5d0ddc4345b021904e355003009d"
V2_PARTIAL_RECEIPT_SHA256 = "412b5fb1bc3fd482531cc3847c974fd4f4dba94c02eb78fe976d3cda117cb412"
V2_CASE_TEMPLATE_SHA256 = "d738d303300b3f339ac37ca3a604fbe8dff9e034d8c8ad7aacb2654434525819"
V2_CASE_TEMPLATE_SIZE_BYTES = 5714

POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_seed_materialization_policy_v3.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_seed_materialization_policy_v3.json"
CASE_TEMPLATE = PACKAGE_DIR / "config/cases/u3_c1m_moving_zero_v1.xml"
BARE_REPOSITORY = LIQUID_ROOT / "dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
FULL_FETCH_RECEIPT = LIQUID_ROOT / "audits/u2_full_source_fetch_attempt_3_20260806T074600Z.json"

HOST_UID = 1000
HOST_GID = 1000
COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
FULL_FETCH_RECEIPT_SHA256 = "23745c0076c2c06fc549665c89f4e039761832bab66f4f568f63eff696ead18d"
CASE_TEMPLATE_SHA256 = "1711b10f366770cb1b6aa2af799c093596c9d606c914c4a68b9bcdb9fa844b52"
CASE_TEMPLATE_SIZE_BYTES = 6070
CASE_TEMPLATE_XML_CONTRACT_SHA256 = "26c186923097e3adce03a4ea9bfc0c281762b1847b6adf2886bf8f28a94b0e23"
GIT_PATH = Path("/usr/bin/git")
GIT_SHA256 = "2a8c18fbf43da9f692d75474c72bea9dfd796c260b0f3dfe456376abc3bbd668"

REVISION_REASON = (
    'The v1 seed is permanently rejected because its C1 boundary cylinder used mask="1 | 2", '
    "which removed both end caps and left no bottom, and because PartsOutMax is deprecated and "
    "ignored. The corrected v2 seed remains fixed-boundary evidence only: its 2669 fixed and zero "
    "moving particles cannot be reused for a replay-compatible moving-boundary restart. V3 uses a "
    "wholly new append-only C1M identity, rematerializes the two pinned upstream blobs directly "
    "from the audited bare repository, and binds only the pinned workspace C1M template with one "
    "casedef motion object at ref=0, begin mov=1/start=0, mvnull id=1, Shifting=1 and "
    "DtAllParticles=1. No v1 or v2 seed byte or path is a v3 materialization source."
)

SEED_FILES: tuple[dict[str, Any], ...] = (
    {
        "destination": "GenCase_linux64",
        "tree_path": "bin/linux/GenCase_linux64",
        "blob_sha1": "4db4dadbae8f6e3885cad3bb752340812036278a",
        "sha256": "a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226",
        "size_bytes": 5_809_384,
        "kind": "git_blob",
    },
    {
        "destination": "DsphConfig.xml",
        "tree_path": "bin/linux/DsphConfig.xml",
        "blob_sha1": "d99d0f85d19a622f7faf2d5f1195b11f17951846",
        "sha256": "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
        "size_bytes": 293,
        "kind": "git_blob",
    },
    {
        "destination": "C1M_moving_zero_Def.xml",
        "sha256": CASE_TEMPLATE_SHA256,
        "size_bytes": CASE_TEMPLATE_SIZE_BYTES,
        "kind": "workspace_template_c1m_v1",
    },
)

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A condition that consumes or blocks this one-shot admission."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def xml_contract(element: ET.Element) -> dict[str, Any]:
    """Return the exact parsed XML contract, excluding formatting and comments."""

    return {
        "tag": element.tag,
        "attributes": dict(sorted(element.attrib.items())),
        "text": (element.text or "").strip(),
        "children": [xml_contract(child) for child in list(element)],
    }


def assert_no_symlink_components(path: Path, *, require_exists: bool) -> None:
    current = Path("/")
    parts = path.absolute().parts[1:]
    for index, part in enumerate(parts):
        current /= part
        if not current.exists() and index == len(parts) - 1 and not require_exists:
            continue
        try:
            metadata = os.lstat(current)
        except FileNotFoundError as exc:
            raise GateError(f"required path component is missing: {current}") from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise GateError(f"symlink component is forbidden: {current}")


def assert_directory(path: Path, *, mode: int | None = None) -> os.stat_result:
    assert_no_symlink_components(path, require_exists=True)
    metadata = os.stat(path, follow_symlinks=False)
    if not stat.S_ISDIR(metadata.st_mode):
        raise GateError(f"not a directory: {path}")
    if metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID:
        raise GateError(f"unexpected directory ownership on {path}")
    if mode is not None and stat.S_IMODE(metadata.st_mode) != mode:
        raise GateError(f"unexpected directory mode on {path}: {oct(stat.S_IMODE(metadata.st_mode))}")
    return metadata


def sha256_regular_file(path: Path, *, limit: int = 64 * 1024 * 1024) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise GateError(f"unsafe regular-file contract: {path}")
        if metadata.st_size > limit:
            raise GateError(f"bounded file is too large: {path}")
        digest = hashlib.sha256()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def read_regular_bytes(path: Path, *, limit: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise GateError(f"unsafe read input: {path}")
        if metadata.st_size > limit:
            raise GateError(f"read input exceeds limit: {path}")
        chunks: list[bytes] = []
        remaining = limit + 1
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                break
            chunks.append(block)
            remaining -= len(block)
        data = b"".join(chunks)
        if len(data) != metadata.st_size:
            raise GateError(f"short or changing read: {path}")
        return data
    finally:
        os.close(descriptor)


def read_json_object(path: Path) -> dict[str, Any]:
    data = read_regular_bytes(path, limit=8 * 1024 * 1024)
    try:
        parsed = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GateError(f"invalid JSON object: {path}") from exc
    if not isinstance(parsed, dict):
        raise GateError(f"JSON root is not an object: {path}")
    return parsed


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    assert_no_symlink_components(path.parent, require_exists=True)
    encoded = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o640,
    )
    try:
        total = 0
        while total < len(encoded):
            written = os.write(descriptor, encoded[total:])
            if written <= 0:
                raise GateError(f"short receipt write: {path}")
            total += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def require_tool(path: Path, expected_sha256: str) -> dict[str, Any]:
    assert_no_symlink_components(path, require_exists=True)
    metadata = os.stat(path, follow_symlinks=False)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0:
        raise GateError(f"trusted tool ownership/type differs: {path}")
    if not metadata.st_mode & stat.S_IXUSR:
        raise GateError(f"trusted tool is not executable: {path}")
    digest = sha256_regular_file(path, limit=512 * 1024 * 1024)
    if digest != expected_sha256:
        raise GateError(f"trusted tool digest differs: {path}")
    return {"path": str(path), "sha256": digest}


def set_mode_nofollow(path: Path, mode: int, *, directory: bool) -> None:
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
    if directory:
        flags |= os.O_DIRECTORY
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        correct_type = stat.S_ISDIR(metadata.st_mode) if directory else stat.S_ISREG(metadata.st_mode)
        correct_links = metadata.st_nlink >= 1 if directory else metadata.st_nlink == 1
        if not correct_type or not correct_links:
            raise GateError(f"unsafe mode-change target: {path}")
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)


def expected_attempt() -> dict[str, Any]:
    return {
        "seed_id": SEED_ID,
        "seed_root": str(SEED_ROOT),
        "input_root": str(INPUT_ROOT),
        "receipt": str(RECEIPT),
        "partial_receipt": str(PARTIAL_RECEIPT),
    }


def expected_nonreusable_predecessors() -> dict[str, Any]:
    return {
        "rejected_v1": {
            "seed_id": V1_SEED_ID,
            "seed_root": str(V1_SEED_ROOT),
            "input_root": str(V1_INPUT_ROOT),
            "receipt": str(V1_RECEIPT),
            "receipt_sha256": V1_RECEIPT_SHA256,
            "partial_receipt": str(V1_PARTIAL_RECEIPT),
            "partial_receipt_sha256": V1_PARTIAL_RECEIPT_SHA256,
            "case_template_sha256": V1_CASE_TEMPLATE_SHA256,
            "disposition": "PERMANENT_NO_GO_DO_NOT_EXECUTE_OR_REUSE",
            "rejection_reasons": [
                "mask_1_or_2_removed_both_cylinder_end_caps_and_left_no_bottom",
                "deprecated_PartsOutMax_was_ignored",
            ],
            "preserved_append_only": True,
            "is_v3_materialization_source": False,
        },
        "fixed_boundary_v2": {
            "seed_id": V2_SEED_ID,
            "seed_root": str(V2_SEED_ROOT),
            "input_root": str(V2_INPUT_ROOT),
            "receipt": str(V2_RECEIPT),
            "receipt_sha256": V2_RECEIPT_SHA256,
            "partial_receipt": str(V2_PARTIAL_RECEIPT),
            "partial_receipt_sha256": V2_PARTIAL_RECEIPT_SHA256,
            "case_template_sha256": V2_CASE_TEMPLATE_SHA256,
            "disposition": "GEOMETRY_BASELINE_ONLY_FIXED_BOUNDARY_DO_NOT_REUSE_FOR_C1M",
            "nonreuse_reasons": [
                "v2_materializes_2669_fixed_and_zero_moving_boundary_particles",
                "settled_fixed_particles_cannot_be_reclassified_for_restart",
                "c1m_requires_a_fresh_gencase_particle_identity",
            ],
            "preserved_append_only": True,
            "is_v3_materialization_source": False,
        },
    }


def expected_source_provenance() -> dict[str, Any]:
    return {
        "full_bare_repository": str(BARE_REPOSITORY),
        "full_fetch_receipt": str(FULL_FETCH_RECEIPT),
        "full_fetch_receipt_sha256": FULL_FETCH_RECEIPT_SHA256,
        "commit": COMMIT,
        "gencase": {
            "tree_path": SEED_FILES[0]["tree_path"],
            "blob_sha1": SEED_FILES[0]["blob_sha1"],
            "sha256": SEED_FILES[0]["sha256"],
            "size_bytes": SEED_FILES[0]["size_bytes"],
            "host_mode_after_materialization": "0400",
            "host_execution": "forbidden",
        },
        "dsph_config": {
            "tree_path": SEED_FILES[1]["tree_path"],
            "blob_sha1": SEED_FILES[1]["blob_sha1"],
            "sha256": SEED_FILES[1]["sha256"],
            "size_bytes": SEED_FILES[1]["size_bytes"],
            "host_mode_after_materialization": "0400",
        },
        "case_template": {
            "workspace_relative_path": "config/cases/u3_c1m_moving_zero_v1.xml",
            "sha256": CASE_TEMPLATE_SHA256,
            "size_bytes": CASE_TEMPLATE_SIZE_BYTES,
            "host_filename": "C1M_moving_zero_Def.xml",
            "host_mode_after_materialization": "0400",
            "classification": "SIM_ONLY_UNVALIDATED_C1M_MOVING_ZERO_STATIC_REVIEW_V1",
        },
    }


def expected_materialization_contract() -> dict[str, Any]:
    return {
        "new_parent_if_absent": str(RUNTIME_PARENT),
        "runtime_parent_mode": "0750",
        "seed_root_mode_after_success": "0500",
        "input_root_mode_after_success": "0500",
        "allowed_regular_files": [entry["destination"] for entry in SEED_FILES],
        "all_files_regular_single_link_no_symlink": True,
        "all_files_read_only_nonexecutable": True,
        "no_existing_attempt_reuse": True,
        "no_delete_or_overwrite": True,
        "predecessor_seeds_as_source": "forbidden",
        "upstream_blob_source": "audited_bare_repository_only",
        "case_source": "pinned_workspace_c1m_moving_zero_v1_template_only",
        "git_operation": [
            "/usr/bin/git",
            "--no-replace-objects",
            "--git-dir",
            "<frozen_bare_repo>",
            "cat-file",
            "blob",
            "<frozen_blob_sha1>",
        ],
        "network": "not_used",
        "upstream_code_execution": "forbidden",
    }


def expected_invariants() -> dict[str, Any]:
    return {
        "no_sudo": True,
        "no_apt": True,
        "no_driver_or_kernel_change": True,
        "no_sysctl_or_apparmor_change": True,
        "no_network": True,
        "no_gpu": True,
        "no_ros_or_gazebo": True,
        "no_precompiled_elf_execution": True,
        "no_compiled_artifact_execution": True,
        "no_source_write": True,
        "no_workspace_mount": True,
        "no_predecessor_seed_input_reuse": True,
        "predecessor_evidence_must_remain_present": True,
        "no_delete_or_overwrite": True,
    }


def expected_policy() -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-target-u3-gencase-seed-materialization-policy-v3",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GENCASE_SEED_MATERIALIZATION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GENCASE_SEED_MATERIALIZATION_V3",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_nonexecutable_gencase_seed_v3_materialization",
        "revision_reason": REVISION_REASON,
        "allowed_gate_commands": ["self-check", "preflight", "materialize"],
        "frozen_attempt": expected_attempt(),
        "nonreusable_predecessors": expected_nonreusable_predecessors(),
        "source_provenance": expected_source_provenance(),
        "trusted_tools": {"git": {"path": str(GIT_PATH), "sha256": GIT_SHA256}},
        "materialization_contract": expected_materialization_contract(),
        "invariants": expected_invariants(),
        "status": "REVIEWED_NONEXECUTABLE_SEED_MATERIALIZATION_V3_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "MATERIALIZE_FIXED_READ_ONLY_SEED_V3_THEN_CREATE_SEPARATE_GENCASE_RUNTIME_V3_ADMISSION",
    }


def validate_schema_instance(instance: Any, schema: Mapping[str, Any], path: str = "$") -> None:
    """Validate the deliberately small JSON-Schema feature set used by v3."""

    if "const" in schema and instance != schema["const"]:
        raise GateError(f"schema const mismatch at {path}")
    declared_type = schema.get("type")
    if declared_type == "object":
        if not isinstance(instance, dict):
            raise GateError(f"schema type mismatch at {path}")
        required = schema.get("required", [])
        if not isinstance(required, list) or any(not isinstance(item, str) for item in required):
            raise GateError(f"invalid schema required list at {path}")
        missing = set(required) - set(instance)
        if missing:
            raise GateError(f"schema-required policy fields missing at {path}: {sorted(missing)}")
        properties = schema.get("properties", {})
        if not isinstance(properties, dict):
            raise GateError(f"invalid schema properties at {path}")
        if schema.get("additionalProperties") is False:
            extra = set(instance) - set(properties)
            if extra:
                raise GateError(f"schema-forbidden policy fields at {path}: {sorted(extra)}")
        for name, subschema in properties.items():
            if name in instance:
                if not isinstance(subschema, dict):
                    raise GateError(f"invalid subschema at {path}.{name}")
                validate_schema_instance(instance[name], subschema, f"{path}.{name}")
    elif declared_type == "string":
        if not isinstance(instance, str):
            raise GateError(f"schema string mismatch at {path}")
        minimum = schema.get("minLength")
        if minimum is not None and (not isinstance(minimum, int) or len(instance) < minimum):
            raise GateError(f"schema minLength mismatch at {path}")
    elif declared_type is not None:
        raise GateError(f"unsupported schema type in frozen v3 schema at {path}: {declared_type}")


def template_facts() -> dict[str, Any]:
    raw = read_regular_bytes(CASE_TEMPLATE, limit=256 * 1024)
    digest = hashlib.sha256(raw).hexdigest()
    if digest != CASE_TEMPLATE_SHA256 or len(raw) != CASE_TEMPLATE_SIZE_BYTES:
        raise GateError("C1M XML template digest or size differs")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise GateError("C1M XML template is malformed") from exc
    if root.tag != "case" or root.attrib:
        raise GateError("C1M XML root differs")

    if [child.tag for child in list(root)] != ["casedef", "execution"]:
        raise GateError("C1M top-level structure differs")
    casedef = root.find("./casedef")
    execution = root.find("./execution")
    if casedef is None or execution is None or casedef.attrib or execution.attrib:
        raise GateError("C1M casedef/execution contract differs")
    if [child.tag for child in list(casedef)] != ["constantsdef", "mkconfig", "geometry", "motion"]:
        raise GateError("C1M casedef child ordering differs")
    if [child.tag for child in list(execution)] != ["parameters"]:
        raise GateError("C1M execution must contain only parameters")

    forbidden_tags = {
        "drawfilestl",
        "drawfileply",
        "drawfilevtk",
        "drawfilevtm",
        "floating",
        "inout",
        "wavegen",
        "mvfile",
        "mvrectfile",
        "mvrotfile",
        "mvrotadvfile",
        "mvpathfile",
        "wait",
        "mvrect",
        "mvrectace",
        "mvrot",
        "mvrotace",
        "mvcir",
        "mvcirace",
        "mvrectsinu",
        "mvrotsinu",
        "mvcirsinu",
    }
    observed_tags = {element.tag for element in root.iter()}
    if observed_tags & forbidden_tags:
        raise GateError("C1M XML contains a forbidden external, nonzero-motion or coupling feature")
    expected_tags = {
        "case", "casedef", "constantsdef", "gravity", "rhop0", "rhopgradient", "hswl",
        "gamma", "speedsystem", "coefsound", "speedsound", "coefh", "cflnumber", "mkconfig",
        "geometry", "definition", "pointmin", "pointmax", "commands", "mainlist", "setshapemode",
        "setdrawmode", "setmkfluid", "drawcylinder", "point", "setmkbound", "motion", "objreal",
        "begin", "mvnull", "execution", "parameters", "parameter", "simulationdomain", "posmin",
        "posmax",
    }
    if observed_tags != expected_tags:
        raise GateError("C1M XML tag set differs from the closed zero-motion contract")

    constants = root.find("./casedef/constantsdef")
    if constants is None or constants.attrib:
        raise GateError("C1M constants block differs")
    expected_constants: tuple[tuple[str, dict[str, str]], ...] = (
        ("gravity", {"x": "0", "y": "0", "z": "-9.81"}),
        ("rhop0", {"value": "1000"}),
        ("rhopgradient", {"value": "2"}),
        ("hswl", {"value": "0.058", "auto": "false"}),
        ("gamma", {"value": "7"}),
        ("speedsystem", {"value": "0", "auto": "true"}),
        ("coefsound", {"value": "20"}),
        ("speedsound", {"value": "0", "auto": "true"}),
        ("coefh", {"value": "1.0"}),
        ("cflnumber", {"value": "0.2"}),
    )
    if [child.tag for child in list(constants)] != [item[0] for item in expected_constants]:
        raise GateError("C1M constants order or multiplicity differs")
    for element, (tag, required_attributes) in zip(list(constants), expected_constants, strict=True):
        if element.tag != tag or any(element.attrib.get(key) != value for key, value in required_attributes.items()):
            raise GateError(f"C1M physical constant differs: {tag}")

    mkconfig = root.find("./casedef/mkconfig")
    if mkconfig is None or mkconfig.attrib != {"boundcount": "1", "fluidcount": "1"}:
        raise GateError("C1M mkconfig differs")

    definition = root.find("./casedef/geometry/definition")
    if definition is None or definition.attrib != {"dp": "0.002", "units_comment": "metres"}:
        raise GateError("C1M particle-spacing definition differs")
    point_min = definition.find("./pointmin")
    point_max = definition.find("./pointmax")
    if point_min is None or point_min.attrib != {"x": "-0.021", "y": "-0.021", "z": "-0.002"}:
        raise GateError("C1M geometry minimum differs")
    if point_max is None or point_max.attrib != {"x": "0.021", "y": "0.021", "z": "0.070"}:
        raise GateError("C1M geometry maximum differs")

    mainlist = root.find("./casedef/geometry/commands/mainlist")
    if mainlist is None:
        raise GateError("C1M command list is missing")
    command_tags = [child.tag for child in list(mainlist)]
    if command_tags != [
        "setshapemode",
        "setdrawmode",
        "setmkfluid",
        "drawcylinder",
        "setmkbound",
        "drawcylinder",
    ]:
        raise GateError("C1M fluid-to-boundary command ordering differs")
    shape_mode = mainlist.find("./setshapemode")
    draw_mode = mainlist.find("./setdrawmode")
    set_fluid = mainlist.find("./setmkfluid")
    set_bound = mainlist.find("./setmkbound")
    if shape_mode is None or (shape_mode.text or "").strip() != "dp | bound" or shape_mode.attrib:
        raise GateError("C1M shape mode differs")
    if draw_mode is None or draw_mode.attrib != {"mode": "full"}:
        raise GateError("C1M draw mode differs")
    if set_fluid is None or set_fluid.attrib != {"mk": "0"}:
        raise GateError("C1M fluid mk differs")
    if set_bound is None or set_bound.attrib != {"mk": "0"}:
        raise GateError("C1M boundary mk differs")
    cylinders = mainlist.findall("./drawcylinder")
    if len(cylinders) != 2:
        raise GateError("C1M must contain exactly two cylinders")
    if cylinders[0].attrib != {"radius": "0.0185"}:
        raise GateError("C1M fluid-cylinder contract differs")
    if cylinders[1].attrib != {"radius": "0.0185", "mask": "2"}:
        raise GateError("C1M open-container boundary contract differs")
    expected_points = (
        ({"x": "0", "y": "0", "z": "0"}, {"x": "0", "y": "0", "z": "0.058"}),
        ({"x": "0", "y": "0", "z": "0"}, {"x": "0", "y": "0", "z": "0.066"}),
    )
    for cylinder, (expected_p1, expected_p2) in zip(cylinders, expected_points, strict=True):
        points = cylinder.findall("./point")
        if len(points) != 2 or points[0].attrib != expected_p1 or points[1].attrib != expected_p2:
            raise GateError("C1M cylinder endpoint contract differs")

    motions = root.findall(".//motion")
    motion = root.find("./casedef/motion")
    if len(motions) != 1 or motion is None or motions[0] is not motion or motion.attrib:
        raise GateError("C1M must contain exactly one motion under casedef")
    if root.find("./execution/motion") is not None:
        raise GateError("C1M seed motion is forbidden under execution")
    if [child.tag for child in list(motion)] != ["objreal"]:
        raise GateError("C1M motion must contain one real object")
    objreal = motion.find("./objreal")
    if objreal is None or objreal.attrib != {"ref": "0"}:
        raise GateError("C1M moving reference must be exactly ref=0")
    if [child.tag for child in list(objreal)] != ["begin", "mvnull"]:
        raise GateError("C1M real object must contain only begin then mvnull")
    begin = objreal.find("./begin")
    mvnull = objreal.find("./mvnull")
    if begin is None or begin.attrib != {"mov": "1", "start": "0"}:
        raise GateError("C1M zero-motion begin must be mov=1/start=0")
    if mvnull is None or mvnull.attrib != {"id": "1"}:
        raise GateError("C1M zero motion must be exactly mvnull id=1")

    parameters_node = root.find("./execution/parameters")
    if parameters_node is None or parameters_node.attrib:
        raise GateError("C1M parameters block differs")
    parameter_children = list(parameters_node)
    if not parameter_children or [child.tag for child in parameter_children[:-1]] != ["parameter"] * (len(parameter_children) - 1):
        raise GateError("C1M parameter structure differs")
    if parameter_children[-1].tag != "simulationdomain":
        raise GateError("C1M simulationdomain must be the final parameters child")
    parameters = root.findall("./execution/parameters/parameter")
    parameter_items: list[tuple[str, str]] = []
    for parameter in parameters:
        key = parameter.attrib.get("key")
        value = parameter.attrib.get("value")
        if key is None or value is None or any(existing_key == key for existing_key, _ in parameter_items):
            raise GateError("C1M parameter key/value set is malformed or duplicated")
        parameter_items.append((key, value))
    expected_parameters = [
        ("SavePosDouble", "1"), ("StepAlgorithm", "2"), ("Kernel", "2"),
        ("ViscoTreatment", "3"), ("Visco", "0.000001"), ("ViscoBoundFactor", "1"),
        ("DensityDT", "2"), ("DensityDTvalue", "0.1"), ("Shifting", "1"),
        ("ShiftCoef", "-2"), ("ShiftTFS", "2.75"), ("RigidAlgorithm", "1"),
        ("FtPause", "0"), ("CoefDtMin", "0.05"), ("DtIni", "0"), ("DtMin", "0"),
        ("DtFixed", "0"), ("DtFixedFile", "NONE"), ("DtAllParticles", "1"),
        ("TimeMax", "1.0"), ("TimeOut", "0.05"), ("MinFluidStop", "1"),
        ("RhopOutMin", "700"), ("RhopOutMax", "1300"),
    ]
    if parameter_items != expected_parameters:
        raise GateError("C1M complete parameter order/value contract differs")
    domain = root.find("./execution/parameters/simulationdomain")
    if domain is None:
        raise GateError("C1M simulation domain is missing")
    domain_min = domain.find("./posmin")
    domain_max = domain.find("./posmax")
    if domain_min is None or domain_min.attrib != {"x": "-0.021", "y": "-0.021", "z": "-0.002"}:
        raise GateError("C1M simulation-domain minimum differs")
    if domain_max is None or domain_max.attrib != {"x": "0.021", "y": "0.021", "z": "0.070"}:
        raise GateError("C1M simulation-domain maximum differs")
    if canonical_hash(xml_contract(root)) != CASE_TEMPLATE_XML_CONTRACT_SHA256:
        raise GateError("C1M closed parsed XML contract differs")
    return {
        "path": str(CASE_TEMPLATE),
        "sha256": digest,
        "size_bytes": len(raw),
        "boundary_mask": "2",
        "min_fluid_stop": "1",
        "moving_reference": "0",
        "motion_begin": {"mov": "1", "start": "0"},
        "motion": "mvnull_id_1",
        "shifting": "1",
        "dt_all_particles": "1",
        "deprecated_parts_out_max_present": False,
        "tags": sorted(observed_tags),
    }


def verify_full_fetch_receipt() -> dict[str, Any]:
    if sha256_regular_file(FULL_FETCH_RECEIPT) != FULL_FETCH_RECEIPT_SHA256:
        raise GateError("full bare-fetch receipt digest differs")
    receipt = read_json_object(FULL_FETCH_RECEIPT)
    if receipt.get("status") != "PASS_FULL_BARE_SOURCE_FETCH":
        raise GateError("full bare-fetch receipt is not PASS")
    for field in (
        "upstream_code_executed",
        "precompiled_binary_executed",
        "system_packages_changed",
        "gpu_device_exposed",
    ):
        if receipt.get(field) is not False:
            raise GateError(f"full bare-fetch receipt violates no-execution invariant: {field}")
    try:
        verification = receipt["results"]["verification"]
        completeness = verification["reachable_object_completeness"]
    except (KeyError, TypeError) as exc:
        raise GateError("full bare-fetch receipt lacks verification data") from exc
    if verification.get("tree") != "cef458cb358712f4694b9d2148f638440418e9dc":
        raise GateError("full bare-fetch tree identity differs")
    if completeness.get("expected_commit_seen") is not True or completeness.get("missing_object_count") != 0:
        raise GateError("full bare-fetch receipt lacks the pinned complete commit")
    return {"path": str(FULL_FETCH_RECEIPT), "sha256": FULL_FETCH_RECEIPT_SHA256}


def inventory_fixed_input(
    root: Path,
    *,
    root_mode: int,
    expected_files: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    assert_directory(root, mode=root_mode)
    observed: dict[str, dict[str, Any]] = {}
    with os.scandir(root) as entries:
        for directory_entry in entries:
            expected = expected_files.get(directory_entry.name)
            if expected is None:
                raise GateError(f"unexpected fixed input file: {directory_entry.name}")
            metadata = directory_entry.stat(follow_symlinks=False)
            if directory_entry.is_symlink() or not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise GateError(f"unsafe fixed input entry: {directory_entry.name}")
            if metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID:
                raise GateError(f"fixed input ownership differs: {directory_entry.name}")
            expected_mode = int(str(expected["mode"]), 8)
            if stat.S_IMODE(metadata.st_mode) != expected_mode or metadata.st_mode & 0o111:
                raise GateError(f"fixed input mode differs: {directory_entry.name}")
            digest = sha256_regular_file(Path(directory_entry.path), limit=16 * 1024 * 1024)
            if digest != expected["sha256"] or metadata.st_size != expected["size_bytes"]:
                raise GateError(f"fixed input digest or size differs: {directory_entry.name}")
            observed[directory_entry.name] = {
                "sha256": digest,
                "size_bytes": metadata.st_size,
                "mode": str(expected["mode"]),
            }
    if set(observed) != set(expected_files):
        raise GateError("fixed input file set differs")
    return {"root": str(root), "files": observed, "manifest_sha256": canonical_hash(observed)}


def verify_rejected_v1_predecessor() -> dict[str, Any]:
    if SEED_ROOT == V1_SEED_ROOT or INPUT_ROOT == V1_INPUT_ROOT or RECEIPT == V1_RECEIPT:
        raise GateError("v3 attempt identity aliases rejected v1")
    receipt_digest = sha256_regular_file(V1_RECEIPT)
    partial_digest = sha256_regular_file(V1_PARTIAL_RECEIPT)
    if receipt_digest != V1_RECEIPT_SHA256 or partial_digest != V1_PARTIAL_RECEIPT_SHA256:
        raise GateError("rejected v1 receipt evidence digest differs")
    receipt = read_json_object(V1_RECEIPT)
    partial = read_json_object(V1_PARTIAL_RECEIPT)
    if receipt.get("seed_id") != V1_SEED_ID or partial.get("seed_id") != V1_SEED_ID:
        raise GateError("rejected v1 seed identity differs")
    if receipt.get("status") != "PASS_NONEXECUTABLE_GENCASE_SEED_MATERIALIZATION":
        raise GateError("rejected v1 historical materialization receipt differs")
    if partial.get("status") != "STARTED_NONEXECUTABLE_SEED_MATERIALIZATION":
        raise GateError("rejected v1 historical partial receipt differs")
    for record in (receipt, partial):
        for field in (
            "upstream_code_executed",
            "precompiled_binary_executed",
            "compiled_artifact_executed",
            "network_used",
            "gpu_device_exposed",
        ):
            if record.get(field) is not False:
                raise GateError(f"rejected v1 receipt violates no-execution evidence: {field}")
    expected_files = {
        "GenCase_linux64": {
            "sha256": SEED_FILES[0]["sha256"],
            "size_bytes": SEED_FILES[0]["size_bytes"],
            "mode": "0400",
        },
        "DsphConfig.xml": {
            "sha256": SEED_FILES[1]["sha256"],
            "size_bytes": SEED_FILES[1]["size_bytes"],
            "mode": "0400",
        },
        "C1_static_Def.xml": {
            "sha256": V1_CASE_TEMPLATE_SHA256,
            "size_bytes": 5446,
            "mode": "0400",
        },
    }
    assert_directory(V1_SEED_ROOT, mode=0o500)
    inventory = inventory_fixed_input(V1_INPUT_ROOT, root_mode=0o500, expected_files=expected_files)
    receipt_seed_input = receipt.get("seed_input")
    if not isinstance(receipt_seed_input, dict):
        raise GateError("rejected v1 receipt lacks seed inventory")
    if receipt_seed_input.get("root") != str(V1_INPUT_ROOT):
        raise GateError("rejected v1 receipt input root differs")
    if receipt_seed_input.get("files") != inventory["files"]:
        raise GateError("rejected v1 receipt file inventory differs")
    return {
        "seed_id": V1_SEED_ID,
        "receipt": str(V1_RECEIPT),
        "receipt_sha256": receipt_digest,
        "partial_receipt": str(V1_PARTIAL_RECEIPT),
        "partial_receipt_sha256": partial_digest,
        "disposition": "PERMANENT_NO_GO_DO_NOT_EXECUTE_OR_REUSE",
        "preserved_inventory": inventory,
        "used_as_v3_source": False,
    }


def verify_fixed_v2_predecessor() -> dict[str, Any]:
    if SEED_ROOT == V2_SEED_ROOT or INPUT_ROOT == V2_INPUT_ROOT or RECEIPT == V2_RECEIPT:
        raise GateError("v3 attempt identity aliases fixed-boundary v2")
    receipt_digest = sha256_regular_file(V2_RECEIPT)
    partial_digest = sha256_regular_file(V2_PARTIAL_RECEIPT)
    if receipt_digest != V2_RECEIPT_SHA256 or partial_digest != V2_PARTIAL_RECEIPT_SHA256:
        raise GateError("fixed-boundary v2 receipt evidence digest differs")
    receipt = read_json_object(V2_RECEIPT)
    partial = read_json_object(V2_PARTIAL_RECEIPT)
    if receipt.get("seed_id") != V2_SEED_ID or partial.get("seed_id") != V2_SEED_ID:
        raise GateError("fixed-boundary v2 seed identity differs")
    if receipt.get("status") != "PASS_NONEXECUTABLE_GENCASE_SEED_V2_MATERIALIZATION":
        raise GateError("fixed-boundary v2 materialization receipt differs")
    if partial.get("status") != "STARTED_NONEXECUTABLE_GENCASE_SEED_V2_MATERIALIZATION":
        raise GateError("fixed-boundary v2 partial receipt differs")
    for record in (receipt, partial):
        for field in (
            "upstream_code_executed",
            "precompiled_binary_executed",
            "compiled_artifact_executed",
            "network_used",
            "gpu_device_exposed",
        ):
            if record.get(field) is not False:
                raise GateError(f"fixed-boundary v2 receipt violates no-execution evidence: {field}")
    expected_files = {
        "GenCase_linux64": {
            "sha256": SEED_FILES[0]["sha256"],
            "size_bytes": SEED_FILES[0]["size_bytes"],
            "mode": "0400",
        },
        "DsphConfig.xml": {
            "sha256": SEED_FILES[1]["sha256"],
            "size_bytes": SEED_FILES[1]["size_bytes"],
            "mode": "0400",
        },
        "C1_static_Def.xml": {
            "sha256": V2_CASE_TEMPLATE_SHA256,
            "size_bytes": V2_CASE_TEMPLATE_SIZE_BYTES,
            "mode": "0400",
        },
    }
    assert_directory(V2_SEED_ROOT, mode=0o500)
    inventory = inventory_fixed_input(V2_INPUT_ROOT, root_mode=0o500, expected_files=expected_files)
    receipt_seed_input = receipt.get("seed_input")
    if not isinstance(receipt_seed_input, dict):
        raise GateError("fixed-boundary v2 receipt lacks seed inventory")
    if receipt_seed_input.get("root") != str(V2_INPUT_ROOT):
        raise GateError("fixed-boundary v2 receipt input root differs")
    if receipt_seed_input.get("files") != inventory["files"]:
        raise GateError("fixed-boundary v2 receipt file inventory differs")
    return {
        "seed_id": V2_SEED_ID,
        "receipt": str(V2_RECEIPT),
        "receipt_sha256": receipt_digest,
        "partial_receipt": str(V2_PARTIAL_RECEIPT),
        "partial_receipt_sha256": partial_digest,
        "disposition": "GEOMETRY_BASELINE_ONLY_FIXED_BOUNDARY_DO_NOT_REUSE_FOR_C1M",
        "preserved_inventory": inventory,
        "used_as_v3_source": False,
    }


def verify_nonreusable_predecessors() -> dict[str, Any]:
    return {
        "rejected_v1": verify_rejected_v1_predecessor(),
        "fixed_boundary_v2": verify_fixed_v2_predecessor(),
    }


def verify_review_artifacts() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    expected = expected_policy()
    if policy != expected:
        differing = sorted(
            key for key in set(policy) | set(expected) if policy.get(key) != expected.get(key)
        )
        raise GateError(f"v3 seed policy differs in fields: {differing}")
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise GateError("v3 seed schema draft differs")
    if schema.get("$id") != "https://scout.local/schema/target-host-u3-gencase-seed-materialization-policy-v3.json":
        raise GateError("v3 seed schema identity differs")
    if schema.get("type") != "object" or schema.get("additionalProperties") is not False:
        raise GateError("v3 seed schema is not a closed top-level object")
    if set(schema.get("required", [])) != set(expected):
        raise GateError("v3 seed schema required-field set differs")
    validate_schema_instance(policy, schema)
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": sha256_regular_file(SCHEMA_PATH)},
        "template": template_facts(),
        "full_fetch_receipt": verify_full_fetch_receipt(),
        "nonreusable_predecessors": verify_nonreusable_predecessors(),
    }


def require_absent(path: Path) -> None:
    current = Path("/")
    for part in path.absolute().parts[1:]:
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            break
        if stat.S_ISLNK(metadata.st_mode):
            raise GateError(f"symlink component is forbidden: {current}")
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise GateError(f"one-shot path already exists: {path}")


def preflight() -> dict[str, Any]:
    if os.geteuid() != HOST_UID or os.getegid() != HOST_GID:
        raise GateError("v3 seed materialization must run as the named unprivileged host user")
    assert_directory(LIQUID_ROOT, mode=0o750)
    assert_directory(LIQUID_ROOT / "dependency", mode=0o750)
    assert_directory(LIQUID_ROOT / "audits", mode=0o750)
    assert_directory(BARE_REPOSITORY, mode=0o750)
    require_absent(SEED_ROOT)
    require_absent(RECEIPT)
    require_absent(PARTIAL_RECEIPT)
    if RUNTIME_PARENT.exists():
        assert_directory(RUNTIME_PARENT, mode=0o750)
    else:
        assert_no_symlink_components(RUNTIME_PARENT, require_exists=False)
    return {
        "review": verify_review_artifacts(),
        "trusted_tools": {"git": require_tool(GIT_PATH, GIT_SHA256)},
        "identity": {
            "uid": os.geteuid(),
            "gid": os.getegid(),
            "supplementary_groups": list(os.getgroups()),
        },
        "paths": {
            "seed_root": str(SEED_ROOT),
            "input_root": str(INPUT_ROOT),
            "receipt": str(RECEIPT),
        },
        "predecessor_read_purpose": "append_only_nonreusable_evidence_verification_only",
        "predecessor_used_as_materialization_source": False,
    }


def mkdir_new(path: Path, mode: int) -> None:
    assert_no_symlink_components(path.parent, require_exists=True)
    try:
        os.mkdir(path, mode)
    except FileExistsError as exc:
        raise GateError(f"new directory already exists: {path}") from exc
    metadata = os.lstat(path)
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or stat.S_IMODE(metadata.st_mode) != mode
        or metadata.st_uid != HOST_UID
        or metadata.st_gid != HOST_GID
    ):
        raise GateError(f"new directory mode/type/ownership differs: {path}")


def materialize_git_blob(entry: Mapping[str, Any], destination: Path) -> dict[str, Any]:
    descriptor = os.open(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    stderr = b""
    returncode = -1
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as output:
            completed = subprocess.run(
                [
                    str(GIT_PATH),
                    "--no-replace-objects",
                    f"--git-dir={BARE_REPOSITORY}",
                    "cat-file",
                    "blob",
                    str(entry["blob_sha1"]),
                ],
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=subprocess.PIPE,
                cwd="/",
                env={
                    "PATH": "/usr/bin",
                    "HOME": "/nonexistent",
                    "GIT_CONFIG_NOSYSTEM": "1",
                    "GIT_CONFIG_GLOBAL": "/dev/null",
                    "GIT_ATTR_NOSYSTEM": "1",
                    "GIT_OPTIONAL_LOCKS": "0",
                    "GIT_TERMINAL_PROMPT": "0",
                    "GIT_NO_REPLACE_OBJECTS": "1",
                    "LC_ALL": "C.UTF-8",
                    "LANG": "C.UTF-8",
                    "TZ": "UTC",
                },
                timeout=60,
                check=False,
            )
            output.flush()
            os.fsync(output.fileno())
            stderr = completed.stderr[:16_384]
            returncode = completed.returncode
    except subprocess.TimeoutExpired as exc:
        raise GateError(f"fixed git blob materialization timed out: {entry['destination']}") from exc
    finally:
        os.close(descriptor)
    if returncode != 0:
        raise GateError(f"fixed git blob materialization failed for {entry['destination']}: rc={returncode}")
    metadata = os.lstat(destination)
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_nlink != 1
        or metadata.st_uid != HOST_UID
        or metadata.st_gid != HOST_GID
    ):
        raise GateError(f"materialized blob has unsafe type/ownership: {destination}")
    if metadata.st_size != entry["size_bytes"]:
        raise GateError(f"materialized blob size differs: {destination}")
    digest = sha256_regular_file(destination, limit=16 * 1024 * 1024)
    if digest != entry["sha256"]:
        raise GateError(f"materialized blob digest differs: {destination}")
    set_mode_nofollow(destination, 0o400, directory=False)
    return {
        "destination": str(destination),
        "source": {
            "audited_bare_repository": str(BARE_REPOSITORY),
            "tree_path": entry["tree_path"],
            "blob_sha1": entry["blob_sha1"],
        },
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode_after_materialization": "0400",
        "git_returncode": returncode,
        "git_stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        "git_stderr_bytes": len(stderr),
        "predecessor_seed_used": False,
    }


def materialize_template(destination: Path) -> dict[str, Any]:
    raw = read_regular_bytes(CASE_TEMPLATE, limit=256 * 1024)
    if hashlib.sha256(raw).hexdigest() != CASE_TEMPLATE_SHA256 or len(raw) != CASE_TEMPLATE_SIZE_BYTES:
        raise GateError("source C1M v3 XML digest or size changed before materialization")
    descriptor = os.open(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    try:
        total = 0
        while total < len(raw):
            written = os.write(descriptor, raw[total:])
            if written <= 0:
                raise GateError("short C1M v3 XML materialization write")
            total += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    digest = sha256_regular_file(destination, limit=256 * 1024)
    metadata = os.lstat(destination)
    if digest != CASE_TEMPLATE_SHA256 or metadata.st_size != CASE_TEMPLATE_SIZE_BYTES:
        raise GateError("materialized C1M v3 XML digest or size differs")
    set_mode_nofollow(destination, 0o400, directory=False)
    return {
        "destination": str(destination),
        "source": {"workspace_template_c1m_v1": str(CASE_TEMPLATE)},
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode_after_materialization": "0400",
        "predecessor_seed_used": False,
    }


def inventory_seed_input() -> dict[str, Any]:
    assert_directory(SEED_ROOT, mode=0o500)
    expected = {
        entry["destination"]: {
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
            "mode": "0400",
        }
        for entry in SEED_FILES
    }
    return inventory_fixed_input(INPUT_ROOT, root_mode=0o500, expected_files=expected)


def materialize() -> int:
    try:
        preflight_data = preflight()
    except GateError as exc:
        print(json.dumps({"status": "NO_GO_PREFLIGHT", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    write_json_new(
        PARTIAL_RECEIPT,
        {
            "document_type": "SMPCC_R8_LIQUID_U3_GENCASE_SEED_V3_MATERIALIZATION_PARTIAL",
            "seed_id": SEED_ID,
            "status": "STARTED_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION",
            "created_at_utc": utc_now(),
            "preflight": preflight_data,
            "nonreusable_predecessors": expected_nonreusable_predecessors(),
            "predecessor_seed_used_as_source": False,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
            "compiled_artifact_executed": False,
            "network_used": False,
            "gpu_device_exposed": False,
        },
    )
    final: dict[str, Any] = {
        "document_type": "SMPCC_R8_LIQUID_U3_GENCASE_SEED_V3_MATERIALIZATION_RECEIPT",
        "seed_id": SEED_ID,
        "created_at_utc": utc_now(),
        "partial_start_record": str(PARTIAL_RECEIPT),
        "preflight": preflight_data,
        "nonreusable_predecessors": expected_nonreusable_predecessors(),
        "predecessor_seed_used_as_source": False,
        "upstream_code_executed": False,
        "precompiled_binary_executed": False,
        "compiled_artifact_executed": False,
        "network_used": False,
        "gpu_device_exposed": False,
        "source_checkout_created": False,
        "system_packages_changed": False,
        "sudo_used": False,
    }
    try:
        if not RUNTIME_PARENT.exists():
            mkdir_new(RUNTIME_PARENT, 0o750)
        mkdir_new(SEED_ROOT, 0o700)
        mkdir_new(INPUT_ROOT, 0o700)
        files: list[dict[str, Any]] = []
        for entry in SEED_FILES:
            destination = INPUT_ROOT / str(entry["destination"])
            if entry["kind"] == "git_blob":
                files.append(materialize_git_blob(entry, destination))
            elif entry["kind"] == "workspace_template_c1m_v1":
                files.append(materialize_template(destination))
            else:
                raise GateError("unsupported frozen v3 seed-file kind")
        set_mode_nofollow(INPUT_ROOT, 0o500, directory=True)
        set_mode_nofollow(SEED_ROOT, 0o500, directory=True)
        final.update(
            {
                "status": "PASS_NONEXECUTABLE_GENCASE_SEED_V3_MATERIALIZATION",
                "seed_input": inventory_seed_input(),
                "files": files,
                "next_allowed_stage": "SEPARATE_GENCASE_RUNTIME_V3_ADMISSION_REQUIRED",
            }
        )
        write_json_new(RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(RECEIPT)}, ensure_ascii=False, sort_keys=True))
        return 0
    except (GateError, OSError, ValueError, UnicodeError) as exc:
        final.update(
            {
                "status": "GENCASE_SEED_V3_MATERIALIZATION_FAILED_NO_RETRY",
                "error": str(exc),
                "next_allowed_stage": "PRESERVE_PARTIAL_AND_CREATE_NEW_REVIEWED_ID",
            }
        )
        try:
            write_json_new(RECEIPT, final)
        except (GateError, OSError):
            pass
        print(json.dumps({"status": final["status"], "receipt": str(RECEIPT)}, ensure_ascii=False, sort_keys=True))
        return 3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "preflight", "materialize"))
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            print(
                json.dumps(
                    {"status": "PASS_STATIC_SEED_V3_REVIEW", "review": verify_review_artifacts()},
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        if arguments.command == "preflight":
            print(
                json.dumps(
                    {"status": "PASS_SEED_V3_PREFLIGHT", "preflight": preflight()},
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        return materialize()
    except GateError as exc:
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
