#!/usr/bin/env python3
"""Fail-closed, non-executing materialization gate for the U3 C1 GenCase seed.

The only subprocess this gate can start is a fixed ``git cat-file blob``
operation against the already audited bare repository.  It never gives an
execute bit to GenCase on the host and it does not create a sandbox, namespace,
network connection, GPU context, ROS process or solver process.
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
SEED_ID = "u3_c1_gencase_seed_20260807T031624Z"
RUNTIME_PARENT = LIQUID_ROOT / "dependency/runtime"
SEED_ROOT = RUNTIME_PARENT / f"{SEED_ID}.partial"
INPUT_ROOT = SEED_ROOT / "input"
RECEIPT = LIQUID_ROOT / "audits" / f"{SEED_ID}.json"
PARTIAL_RECEIPT = RECEIPT.with_suffix(RECEIPT.suffix + ".partial")
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_seed_materialization_policy_v1.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_seed_materialization_policy_v1.json"
CASE_TEMPLATE = PACKAGE_DIR / "config/cases/u3_c1_static_v1.xml"
BARE_REPOSITORY = LIQUID_ROOT / "dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git"
FULL_FETCH_RECEIPT = LIQUID_ROOT / "audits/u2_full_source_fetch_attempt_3_20260806T074600Z.json"

HOST_UID = 1000
HOST_GID = 1000
COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
FULL_FETCH_RECEIPT_SHA256 = "23745c0076c2c06fc549665c89f4e039761832bab66f4f568f63eff696ead18d"
CASE_TEMPLATE_SHA256 = "ef57582f69714f622e5b8a348c7626d233c203ad0b1eb6332b58fb1ffc09fa1a"
GIT_PATH = Path("/usr/bin/git")
GIT_SHA256 = "2a8c18fbf43da9f692d75474c72bea9dfd796c260b0f3dfe456376abc3bbd668"

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
        "destination": "C1_static_Def.xml",
        "sha256": CASE_TEMPLATE_SHA256,
        "kind": "workspace_template",
    },
)

sys.dont_write_bytecode = True


class GateError(RuntimeError):
    """A condition that must stop the one-shot gate without retry."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


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
            total += os.write(descriptor, encoded[total:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def require_tool(path: Path, expected_sha256: str) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise GateError(f"trusted tool is missing or symlinked: {path}")
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
        expected_type = stat.S_ISDIR(metadata.st_mode) if directory else stat.S_ISREG(metadata.st_mode)
        expected_links = metadata.st_nlink >= 1 if directory else metadata.st_nlink == 1
        if not expected_type or not expected_links:
            raise GateError(f"unsafe mode-change target: {path}")
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)


def template_facts() -> dict[str, Any]:
    raw = read_regular_bytes(CASE_TEMPLATE, limit=256 * 1024)
    digest = hashlib.sha256(raw).hexdigest()
    if digest != CASE_TEMPLATE_SHA256:
        raise GateError("C1 XML template digest differs")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise GateError("C1 XML template is malformed") from exc
    if root.tag != "case":
        raise GateError("C1 XML root differs")
    forbidden_tags = {"drawfilestl", "drawfileply", "drawfilevtm", "motion", "floating", "inout", "wavegen"}
    observed_tags = {element.tag for element in root.iter()}
    if observed_tags & forbidden_tags:
        raise GateError("C1 XML contains an external-asset, motion or coupling feature")
    definition = root.find("./casedef/geometry/definition")
    if definition is None or definition.attrib.get("dp") != "0.002":
        raise GateError("C1 XML particle spacing differs")
    cylinders = root.findall("./casedef/geometry/commands/mainlist/drawcylinder")
    if len(cylinders) != 2:
        raise GateError("C1 XML must contain exactly the fluid and boundary cylinders")
    if cylinders[0].attrib != {"radius": "0.0185"}:
        raise GateError("C1 fluid-cylinder contract differs")
    if cylinders[1].attrib != {"radius": "0.0185", "mask": "1 | 2"}:
        raise GateError("C1 open-container boundary contract differs")
    if root.find(".//parameter[@key='TimeMax'][@value='1.0']") is None:
        raise GateError("C1 settle duration differs")
    return {"path": str(CASE_TEMPLATE), "sha256": digest, "size_bytes": len(raw), "tags": sorted(observed_tags)}


def verify_full_fetch_receipt() -> dict[str, Any]:
    if sha256_regular_file(FULL_FETCH_RECEIPT) != FULL_FETCH_RECEIPT_SHA256:
        raise GateError("full bare-fetch receipt digest differs")
    receipt = read_json_object(FULL_FETCH_RECEIPT)
    if receipt.get("status") != "PASS_FULL_BARE_SOURCE_FETCH":
        raise GateError("full bare-fetch receipt is not PASS")
    for field in ("upstream_code_executed", "precompiled_binary_executed", "system_packages_changed", "gpu_device_exposed"):
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


def expected_policy_top_level() -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-target-u3-gencase-seed-materialization-policy-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_U3_GENCASE_SEED_MATERIALIZATION_POLICY",
        "policy_id": "LIQUID_ZRJ_MSI_U2404_U3_GENCASE_SEED_MATERIALIZATION_V1",
        "host_id": "LIQUID_ZRJ_MSI_U2404",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "user_delegated_review_and_execution_approval": True,
        "execution_scope": "one_exact_nonexecutable_gencase_seed_materialization",
        "status": "REVIEWED_NONEXECUTABLE_SEED_MATERIALIZATION_V1_PENDING_STATIC_VERIFICATION",
        "next_allowed_stage": "MATERIALIZE_FIXED_READ_ONLY_SEED_THEN_CREATE_SEPARATE_GENCASE_RUNTIME_ADMISSION",
    }


def verify_review_artifacts() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if set(policy) != set(schema.get("required", [])) or schema.get("additionalProperties") is not False:
        raise GateError("policy schema is not a closed exact top-level contract")
    for name, value in expected_policy_top_level().items():
        if policy.get(name) != value:
            raise GateError(f"policy field differs: {name}")
    if policy.get("allowed_gate_commands") != ["self-check", "preflight", "materialize"]:
        raise GateError("policy command surface differs")
    expected_attempt = {
        "seed_id": SEED_ID,
        "seed_root": str(SEED_ROOT),
        "input_root": str(INPUT_ROOT),
        "receipt": str(RECEIPT),
        "partial_receipt": str(PARTIAL_RECEIPT),
    }
    if policy.get("frozen_attempt") != expected_attempt:
        raise GateError("policy attempt identity differs")
    provenance = policy.get("source_provenance")
    if not isinstance(provenance, dict):
        raise GateError("policy provenance is missing")
    if provenance.get("full_bare_repository") != str(BARE_REPOSITORY):
        raise GateError("policy bare repository differs")
    if provenance.get("full_fetch_receipt") != str(FULL_FETCH_RECEIPT) or provenance.get("full_fetch_receipt_sha256") != FULL_FETCH_RECEIPT_SHA256:
        raise GateError("policy full-fetch provenance differs")
    if provenance.get("commit") != COMMIT:
        raise GateError("policy commit differs")
    expected_source_files = {entry["destination"]: entry for entry in SEED_FILES}
    for source_key, destination in (("gencase", "GenCase_linux64"), ("dsph_config", "DsphConfig.xml")):
        actual = provenance.get(source_key)
        expected = expected_source_files[destination]
        if not isinstance(actual, dict):
            raise GateError(f"policy {source_key} is missing")
        for key in ("tree_path", "blob_sha1", "sha256", "size_bytes"):
            if actual.get(key) != expected[key]:
                raise GateError(f"policy {source_key} differs: {key}")
        if actual.get("host_mode_after_materialization") != "0400":
            raise GateError(f"policy {source_key} mode differs")
    case = provenance.get("case_template")
    if not isinstance(case, dict) or case != {
        "workspace_relative_path": "config/cases/u3_c1_static_v1.xml",
        "sha256": CASE_TEMPLATE_SHA256,
        "host_filename": "C1_static_Def.xml",
        "host_mode_after_materialization": "0400",
        "classification": "SIM_ONLY_UNVALIDATED",
    }:
        raise GateError("policy C1 template contract differs")
    tools = policy.get("trusted_tools")
    if tools != {"git": {"path": str(GIT_PATH), "sha256": GIT_SHA256}}:
        raise GateError("policy trusted-tools contract differs")
    contract = policy.get("materialization_contract")
    if not isinstance(contract, dict):
        raise GateError("policy materialization contract is missing")
    expected_contract = {
        "new_parent_if_absent": str(RUNTIME_PARENT),
        "runtime_parent_mode": "0750",
        "seed_root_mode_after_success": "0500",
        "input_root_mode_after_success": "0500",
        "allowed_regular_files": [entry["destination"] for entry in SEED_FILES],
        "all_files_regular_single_link_no_symlink": True,
        "all_files_read_only_nonexecutable": True,
        "no_existing_attempt_reuse": True,
        "no_delete_or_overwrite": True,
        "git_operation": ["/usr/bin/git", "--no-replace-objects", "--git-dir", "<frozen_bare_repo>", "cat-file", "blob", "<frozen_blob_sha1>"],
        "network": "not_used",
        "upstream_code_execution": "forbidden",
    }
    if contract != expected_contract:
        raise GateError("policy materialization contract differs")
    expected_invariants = {
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
        "no_delete_or_overwrite": True,
    }
    if policy.get("invariants") != expected_invariants:
        raise GateError("policy invariant contract differs")
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_regular_file(POLICY_PATH)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": sha256_regular_file(SCHEMA_PATH)},
        "template": template_facts(),
        "full_fetch_receipt": verify_full_fetch_receipt(),
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
        raise GateError("seed materialization must run as the named unprivileged host user")
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
        "trusted_tools": {
            "git": require_tool(GIT_PATH, GIT_SHA256),
        },
        "identity": {"uid": os.geteuid(), "gid": os.getegid(), "supplementary_groups": list(os.getgroups())},
        "paths": {"seed_root": str(SEED_ROOT), "input_root": str(INPUT_ROOT), "receipt": str(RECEIPT)},
    }


def mkdir_new(path: Path, mode: int) -> None:
    assert_no_symlink_components(path.parent, require_exists=True)
    try:
        os.mkdir(path, mode)
    except FileExistsError as exc:
        raise GateError(f"new directory already exists: {path}") from exc
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        raise GateError(f"new directory mode/type differs: {path}")


def materialize_git_blob(entry: Mapping[str, Any], destination: Path) -> dict[str, Any]:
    blob_sha1 = entry["blob_sha1"]
    descriptor = os.open(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    stderr: bytes
    returncode: int
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as output:
            completed = subprocess.run(
                [
                    str(GIT_PATH),
                    "--no-replace-objects",
                    f"--git-dir={BARE_REPOSITORY}",
                    "cat-file",
                    "blob",
                    blob_sha1,
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
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise GateError(f"materialized blob has unsafe type: {destination}")
    if metadata.st_size != entry["size_bytes"]:
        raise GateError(f"materialized blob size differs: {destination}")
    digest = sha256_regular_file(destination, limit=16 * 1024 * 1024)
    if digest != entry["sha256"]:
        raise GateError(f"materialized blob digest differs: {destination}")
    set_mode_nofollow(destination, 0o400, directory=False)
    return {
        "destination": str(destination),
        "source": {"tree_path": entry["tree_path"], "blob_sha1": blob_sha1},
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode_after_materialization": "0400",
        "git_returncode": returncode,
        "git_stderr_sha256": hashlib.sha256(stderr).hexdigest(),
        "git_stderr_bytes": len(stderr),
    }


def materialize_template(destination: Path) -> dict[str, Any]:
    raw = read_regular_bytes(CASE_TEMPLATE, limit=256 * 1024)
    descriptor = os.open(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    try:
        total = 0
        while total < len(raw):
            total += os.write(descriptor, raw[total:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    digest = sha256_regular_file(destination, limit=256 * 1024)
    if digest != CASE_TEMPLATE_SHA256:
        raise GateError("materialized C1 XML digest differs")
    set_mode_nofollow(destination, 0o400, directory=False)
    metadata = os.lstat(destination)
    return {
        "destination": str(destination),
        "source": {"workspace_template": str(CASE_TEMPLATE)},
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode_after_materialization": "0400",
    }


def inventory_seed_input() -> dict[str, Any]:
    assert_directory(SEED_ROOT, mode=0o500)
    assert_directory(INPUT_ROOT, mode=0o500)
    expected = {entry["destination"]: entry for entry in SEED_FILES}
    observed: dict[str, dict[str, Any]] = {}
    with os.scandir(INPUT_ROOT) as entries:
        for directory_entry in entries:
            if directory_entry.name not in expected:
                raise GateError(f"unexpected seed file: {directory_entry.name}")
            metadata = directory_entry.stat(follow_symlinks=False)
            if directory_entry.is_symlink() or not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise GateError(f"unsafe seed entry: {directory_entry.name}")
            if stat.S_IMODE(metadata.st_mode) != 0o400 or metadata.st_mode & 0o111:
                raise GateError(f"seed entry is not read-only/non-executable: {directory_entry.name}")
            digest = sha256_regular_file(Path(directory_entry.path), limit=16 * 1024 * 1024)
            expected_entry = expected[directory_entry.name]
            if digest != expected_entry["sha256"]:
                raise GateError(f"seed entry digest differs: {directory_entry.name}")
            if "size_bytes" in expected_entry and metadata.st_size != expected_entry["size_bytes"]:
                raise GateError(f"seed entry size differs: {directory_entry.name}")
            observed[directory_entry.name] = {"sha256": digest, "size_bytes": metadata.st_size, "mode": "0400"}
    if set(observed) != set(expected):
        raise GateError("seed file set differs")
    return {"root": str(INPUT_ROOT), "files": observed, "manifest_sha256": canonical_hash(observed)}


def materialize() -> int:
    try:
        preflight_data = preflight()
    except GateError as exc:
        print(json.dumps({"status": "NO_GO_PREFLIGHT", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2
    write_json_new(
        PARTIAL_RECEIPT,
        {
            "document_type": "SMPCC_R8_LIQUID_U3_GENCASE_SEED_MATERIALIZATION_PARTIAL",
            "seed_id": SEED_ID,
            "status": "STARTED_NONEXECUTABLE_SEED_MATERIALIZATION",
            "created_at_utc": utc_now(),
            "preflight": preflight_data,
            "upstream_code_executed": False,
            "precompiled_binary_executed": False,
            "compiled_artifact_executed": False,
            "network_used": False,
            "gpu_device_exposed": False,
        },
    )
    final: dict[str, Any] = {
        "document_type": "SMPCC_R8_LIQUID_U3_GENCASE_SEED_MATERIALIZATION_RECEIPT",
        "seed_id": SEED_ID,
        "created_at_utc": utc_now(),
        "partial_start_record": str(PARTIAL_RECEIPT),
        "preflight": preflight_data,
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
            destination = INPUT_ROOT / entry["destination"]
            if entry["kind"] == "git_blob":
                files.append(materialize_git_blob(entry, destination))
            elif entry["kind"] == "workspace_template":
                files.append(materialize_template(destination))
            else:
                raise GateError("unsupported frozen seed-file kind")
        set_mode_nofollow(INPUT_ROOT, 0o500, directory=True)
        set_mode_nofollow(SEED_ROOT, 0o500, directory=True)
        final.update(
            {
                "status": "PASS_NONEXECUTABLE_GENCASE_SEED_MATERIALIZATION",
                "seed_input": inventory_seed_input(),
                "files": files,
                "next_allowed_stage": "SEPARATE_GENCASE_RUNTIME_ADMISSION_REQUIRED",
            }
        )
        write_json_new(RECEIPT, final)
        print(json.dumps({"status": final["status"], "receipt": str(RECEIPT)}, ensure_ascii=False, sort_keys=True))
        return 0
    except (GateError, OSError, ValueError, UnicodeError) as exc:
        final.update(
            {
                "status": "GENCASE_SEED_MATERIALIZATION_FAILED_NO_RETRY",
                "error": str(exc),
                "next_allowed_stage": "INSPECT_PARTIAL_AND_CREATE_NEW_REVIEW_IF_NEEDED",
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
            print(json.dumps({"status": "PASS_STATIC_SEED_REVIEW", "review": verify_review_artifacts()}, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "preflight":
            print(json.dumps({"status": "PASS_SEED_PREFLIGHT", "preflight": preflight()}, ensure_ascii=False, sort_keys=True))
            return 0
        return materialize()
    except GateError as exc:
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
