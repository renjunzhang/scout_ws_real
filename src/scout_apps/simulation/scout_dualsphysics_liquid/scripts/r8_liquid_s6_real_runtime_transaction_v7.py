#!/usr/bin/env python3
"""Recoverable, create-new S6 v7 publication transaction core.

The public CLI is deliberately limited to ``self-check``.  Real filesystem
operations are available only through the explicit library API and require an
exact :class:`TransactionSpec`.  A transaction is ordered as follows::

    complete and fsync staging
    -> append/fsync PREPARED (stage6_pass=false)
    -> renameat2(RENAME_NOREPLACE) and fsync the parent
    -> revalidate the complete final tree
    -> append/fsync COMMITTED
    -> create-new O_EXCL final receipt and fsync its parent

The stored final receipt intentionally does *not* contain its own digest.  The
consumer computes that digest externally, then accepts PASS only after the
final tree, PREPARED entry, COMMITTED entry, and receipt all agree.  This keeps
the transaction free of hash self-reference.

This module does not select or read bags, execute ROS/solver/GPU code, use
sudo, access a network, or discover a "latest" directory.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hashlib
import json
import os
import re
import stat
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
TRANSACTION_SCHEMA_PATH = (
    ROOT / "schema/target_host_s6_real_runtime_transaction_receipt_v7.json"
)
RUNTIME_CONTRACT_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_runtime_contract_v7.json"
ARTIFACT_BUNDLE_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_runtime_artifact_bundle_v7.json"
FRAME_READER_SHA256 = "aecd5125625ce4da91b9782f6a28eed017ff5e163056fc648b414aca96e2af4c"
FRAME_SCHEMA_SHA256 = "4f75694305cd8530ebc0b3c744ec8f3e9d880a0c63d7160cb9a7fb6b6b92853a"
CANONICAL_IDS_SHA256 = "dbce99d36546067ed3a903e5892383885b6bb9b94feb1f4171411743fec9bdcf"
ATTEMPT_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
PLANNED_DENOMINATOR = 1
SOURCE_OUTCOME = "UNKNOWN"
ZERO_SHA256 = "0" * 64
SHA_RE = re.compile(r"[0-9a-f]{64}\Z")
TRANSACTION_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{7,127}\Z")
FILE_MODE = 0o440
DIRECTORY_MODE = 0o700
LEDGER_MODE = 0o600
MAXIMUM_FILES = 4096
MAXIMUM_FILE_BYTES = 64 * 1024 * 1024
MAXIMUM_TOTAL_BYTES = 256 * 1024 * 1024
MAXIMUM_LEDGER_BYTES = 16 * 1024 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1
FAILURE_POINTS = frozenset({
    "after_staging",
    "after_prepared",
    "after_publish",
    "after_revalidate",
    "after_committed",
    "after_receipt",
})


class S6TransactionV7Error(ValueError):
    """An identity, ordering, filesystem, ledger, or receipt invariant failed."""


class InjectedFailure(S6TransactionV7Error):
    """A test-only deterministic fault was injected after a durable phase."""


@dataclass(frozen=True)
class TransactionSpec:
    """Exact, pre-frozen paths and identities for one primary-only transaction."""

    transaction_id: str
    runtime_contract_sha256: str
    expected_previous_ledger_sha256: str
    partial_root: Path
    final_root: Path
    ledger_path: Path
    final_receipt_path: Path


def canonical_json(value: Any) -> bytes:
    """Return finite, deterministic UTF-8 JSON with one trailing newline."""

    try:
        encoded = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exc:
        raise S6TransactionV7Error("value is not finite canonical JSON") from exc
    return (encoded + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def assert_deep_closed(node: Any, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise S6TransactionV7Error(f"schema object is open at {location}")
        for key, child in node.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(node, list):
        for index, child in enumerate(node):
            assert_deep_closed(child, f"{location}/{index}")


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise S6TransactionV7Error(f"{label} is not an exact lowercase SHA-256")
    return value


def _absolute(path: Path, label: str) -> Path:
    path = Path(path)
    if not path.is_absolute() or str(path) != os.path.normpath(str(path)):
        raise S6TransactionV7Error(f"{label} is not a normalized absolute path")
    return path


def _assert_existing_parent_chain(path: Path) -> None:
    cursor = Path(path.anchor)
    for part in path.parts[1:-1]:
        cursor /= part
        metadata = os.lstat(cursor)
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise S6TransactionV7Error(f"unsafe parent component: {cursor}")


def validate_spec(spec: TransactionSpec) -> TransactionSpec:
    if not isinstance(spec, TransactionSpec):
        raise S6TransactionV7Error("transaction spec has the wrong type")
    if TRANSACTION_ID_RE.fullmatch(spec.transaction_id) is None:
        raise S6TransactionV7Error("transaction_id is invalid")
    _sha(spec.runtime_contract_sha256, "runtime contract")
    _sha(spec.expected_previous_ledger_sha256, "ledger predecessor")
    partial = _absolute(spec.partial_root, "partial_root")
    final = _absolute(spec.final_root, "final_root")
    ledger = _absolute(spec.ledger_path, "ledger_path")
    receipt = _absolute(spec.final_receipt_path, "final_receipt_path")
    if partial.parent != final.parent or partial.name != final.name + ".partial":
        raise S6TransactionV7Error("partial/final root relationship differs")
    if len({partial, final, ledger, receipt}) != 4:
        raise S6TransactionV7Error("transaction paths are not distinct")
    for path in (partial, final, ledger, receipt):
        _assert_existing_parent_chain(path)
    return spec


def _relative(value: str) -> str:
    if not isinstance(value, str):
        raise S6TransactionV7Error("artifact path is not a string")
    pure = PurePosixPath(value)
    if (
        not value
        or pure.is_absolute()
        or str(pure) != value
        or any(part in ("", ".", "..") for part in pure.parts)
    ):
        raise S6TransactionV7Error(f"unsafe artifact path: {value!r}")
    return value


def _normalise_artifacts(artifacts: Mapping[str, bytes]) -> dict[str, bytes]:
    if not isinstance(artifacts, Mapping) or not 1 <= len(artifacts) <= MAXIMUM_FILES:
        raise S6TransactionV7Error("artifact count is outside the frozen bound")
    result: dict[str, bytes] = {}
    total = 0
    for raw_name, payload in artifacts.items():
        name = _relative(raw_name)
        if name in result or not isinstance(payload, bytes) or not payload:
            raise S6TransactionV7Error("artifact name is duplicate or payload is not non-empty bytes")
        if len(payload) > MAXIMUM_FILE_BYTES:
            raise S6TransactionV7Error("artifact exceeds per-file bound")
        total += len(payload)
        if total > MAXIMUM_TOTAL_BYTES:
            raise S6TransactionV7Error("artifact bundle exceeds total bound")
        result[name] = payload
    return {name: result[name] for name in sorted(result)}


def _expected_inventory(artifacts: Mapping[str, bytes]) -> list[dict[str, Any]]:
    return [
        {
            "relative_path": name,
            "mode": f"{FILE_MODE:04o}",
            "size_bytes": len(payload),
            "sha256": sha256_bytes(payload),
        }
        for name, payload in artifacts.items()
    ]


def bundle_identity(spec: TransactionSpec, artifacts: Mapping[str, bytes]) -> dict[str, Any]:
    """Build the stable identity used by all four transaction witnesses."""

    validate_spec(spec)
    normal = _normalise_artifacts(artifacts)
    inventory = _expected_inventory(normal)
    inventory_sha256 = sha256_json(inventory)
    descriptor = {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-bundle-identity-v7",
        "transaction_id": spec.transaction_id,
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": spec.runtime_contract_sha256,
        "final_root": str(spec.final_root),
        "inventory_sha256": inventory_sha256,
        "file_count": len(inventory),
    }
    return {
        "inventory": inventory,
        "inventory_sha256": inventory_sha256,
        "bundle_sha256": sha256_json(descriptor),
        "file_count": len(inventory),
    }


def precommit_bundle_sha256(artifact_bundle: Mapping[str, Any]) -> str:
    """Return the exact canonical admission-document identity."""

    return sha256_bytes(canonical_json(dict(artifact_bundle)))


def _write_all(fd: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        written = os.write(fd, view)
        if written <= 0:
            raise S6TransactionV7Error("short filesystem write")
        view = view[written:]


def _open_directory_at(parent_fd: int, parts: Sequence[str], *, create: bool) -> tuple[int, list[int]]:
    cursor = os.dup(parent_fd)
    owned = [cursor]
    try:
        for part in parts:
            if create:
                try:
                    os.mkdir(part, DIRECTORY_MODE, dir_fd=cursor)
                except FileExistsError:
                    pass
            child = os.open(
                part,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=cursor,
            )
            owned.append(child)
            cursor = child
        return cursor, owned
    except BaseException:
        for descriptor in reversed(owned):
            os.close(descriptor)
        raise


def _fsync_directories(root_fd: int, names: Sequence[str]) -> None:
    directories = {PurePosixPath(name).parent for name in names}
    directories.discard(PurePosixPath("."))
    for pure in sorted(directories, key=lambda value: (-len(value.parts), str(value))):
        descriptor, owned = _open_directory_at(root_fd, pure.parts, create=False)
        try:
            os.fsync(descriptor)
        finally:
            for item in reversed(owned):
                os.close(item)
    os.fsync(root_fd)


def _stage_fresh(spec: TransactionSpec, artifacts: Mapping[str, bytes]) -> None:
    if spec.partial_root.exists() or spec.final_root.exists():
        raise S6TransactionV7Error("staging/publication root is not fresh")
    parent_fd = os.open(
        spec.final_root.parent,
        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
    )
    try:
        os.mkdir(spec.partial_root.name, DIRECTORY_MODE, dir_fd=parent_fd)
        root_fd = os.open(
            spec.partial_root.name,
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=parent_fd,
        )
        try:
            os.fchmod(root_fd, DIRECTORY_MODE)
            for relative, payload in artifacts.items():
                parts = PurePosixPath(relative).parts
                directory_fd, owned = _open_directory_at(root_fd, parts[:-1], create=True)
                try:
                    descriptor = os.open(
                        parts[-1],
                        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                        FILE_MODE,
                        dir_fd=directory_fd,
                    )
                    try:
                        os.fchmod(descriptor, FILE_MODE)
                        _write_all(descriptor, payload)
                        os.fsync(descriptor)
                    finally:
                        os.close(descriptor)
                finally:
                    for item in reversed(owned):
                        os.close(item)
            _fsync_directories(root_fd, tuple(artifacts))
        finally:
            os.close(root_fd)
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def _read_fd(fd: int, maximum: int) -> bytes:
    chunks: list[bytes] = []
    total = 0
    while True:
        block = os.read(fd, min(1 << 20, maximum + 1 - total))
        if not block:
            return b"".join(chunks)
        total += len(block)
        if total > maximum:
            raise S6TransactionV7Error("file exceeds read bound")
        chunks.append(block)


def _scan_tree(root: Path) -> list[dict[str, Any]]:
    root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    records: list[dict[str, Any]] = []
    try:
        root_before = os.fstat(root_fd)

        def walk(directory_fd: int, prefix: str = "") -> None:
            for name in sorted(os.listdir(directory_fd)):
                relative = f"{prefix}/{name}" if prefix else name
                metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
                if stat.S_ISDIR(metadata.st_mode):
                    child = os.open(
                        name,
                        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                        dir_fd=directory_fd,
                    )
                    try:
                        anchored = os.fstat(child)
                        if (anchored.st_dev, anchored.st_ino) != (metadata.st_dev, metadata.st_ino):
                            raise S6TransactionV7Error("directory TOCTOU detected")
                        walk(child, relative)
                    finally:
                        os.close(child)
                elif stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1:
                    descriptor = os.open(
                        name,
                        os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
                        dir_fd=directory_fd,
                    )
                    try:
                        before = os.fstat(descriptor)
                        payload = _read_fd(descriptor, MAXIMUM_FILE_BYTES)
                        after = os.fstat(descriptor)
                    finally:
                        os.close(descriptor)
                    identity_before = (
                        before.st_dev, before.st_ino, before.st_mode, before.st_nlink,
                        before.st_size, before.st_mtime_ns, before.st_ctime_ns,
                    )
                    identity_after = (
                        after.st_dev, after.st_ino, after.st_mode, after.st_nlink,
                        after.st_size, after.st_mtime_ns, after.st_ctime_ns,
                    )
                    if identity_before != identity_after or len(payload) != before.st_size:
                        raise S6TransactionV7Error("file TOCTOU detected")
                    records.append({
                        "relative_path": relative,
                        "mode": f"{stat.S_IMODE(before.st_mode):04o}",
                        "size_bytes": len(payload),
                        "sha256": sha256_bytes(payload),
                    })
                else:
                    raise S6TransactionV7Error("tree contains symlink, hardlink, or special file")

        walk(root_fd)
        root_after = os.fstat(root_fd)
        if (
            root_before.st_dev,
            root_before.st_ino,
            root_before.st_mode,
            root_before.st_mtime_ns,
            root_before.st_ctime_ns,
        ) != (
            root_after.st_dev,
            root_after.st_ino,
            root_after.st_mode,
            root_after.st_mtime_ns,
            root_after.st_ctime_ns,
        ):
            raise S6TransactionV7Error("root TOCTOU detected")
    finally:
        os.close(root_fd)
    return sorted(records, key=lambda row: row["relative_path"])


def _verify_tree(root: Path, identity: Mapping[str, Any]) -> None:
    observed = _scan_tree(root)
    if observed != identity["inventory"]:
        raise S6TransactionV7Error("tree inventory, mode, size, or hash differs")
    if sha256_json(observed) != identity["inventory_sha256"]:
        raise S6TransactionV7Error("tree inventory digest differs")


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise S6TransactionV7Error("renameat2 is unavailable; no weaker fallback is allowed")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        AT_FDCWD,
        os.fsencode(source),
        AT_FDCWD,
        os.fsencode(destination),
        RENAME_NOREPLACE,
    )
    if result != 0:
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise S6TransactionV7Error("rename-noreplace destination collision")
        raise OSError(number, os.strerror(number), str(destination))


def _publish(spec: TransactionSpec) -> None:
    parent_fd = os.open(
        spec.final_root.parent,
        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
    )
    try:
        _rename_noreplace(spec.partial_root, spec.final_root)
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def _parse_json_object(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            raw,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    except (UnicodeError, ValueError) as exc:
        raise S6TransactionV7Error(f"invalid JSON in {label}") from exc
    if not isinstance(value, dict) or canonical_json(value) != raw:
        raise S6TransactionV7Error(f"non-canonical or non-object JSON in {label}")
    return value


def validate_runtime_contract_and_bundle(runtime_contract: Mapping[str, Any],
                                         artifact_bundle: Mapping[str, Any]) -> None:
    """Reject ABI v1 or claim-promoted transaction inputs before staging."""
    contract_schema = json.loads(RUNTIME_CONTRACT_SCHEMA_PATH.read_bytes())
    bundle_schema = json.loads(ARTIFACT_BUNDLE_SCHEMA_PATH.read_bytes())
    for schema in (contract_schema, bundle_schema):
        Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    Draft202012Validator(contract_schema).validate(dict(runtime_contract))
    Draft202012Validator(bundle_schema).validate(dict(artifact_bundle))
    canonical = artifact_bundle["canonical_inputs"]
    if (canonical["finalized_frame_manifest"]["schema_version"] !=
            "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v2"
            or canonical["finalized_frame_manifest"]["status"] !=
            "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2"
            or canonical["frame_reader_sha256"] != FRAME_READER_SHA256
            or canonical["frame_schema_sha256"] != FRAME_SCHEMA_SHA256
            or canonical["canonical_ids_sha256"] != CANONICAL_IDS_SHA256):
        raise S6TransactionV7Error("artifact canonical finalized-frame v2 binding differs")
    contract_raw = canonical_json(runtime_contract)
    if (artifact_bundle["contract"]["sha256"] != sha256_bytes(contract_raw)
            or artifact_bundle["contract"]["size_bytes"] != len(contract_raw)):
        raise S6TransactionV7Error("artifact/runtime contract SHA binding differs")
    for claims in (runtime_contract["claims"], artifact_bundle["claims"]):
        if (claims["stage6_pass"] is not False
                or not claims["development_only"] or not claims["physical_reference_pending"]
                or any(claims[key] for key in ("physical_fidelity_validated", "paired_ranking",
                    "cross_method_ranking", "selected_trajectory_cpu_comparison", "formal",
                    "production", "physical_primary"))):
            raise S6TransactionV7Error("single-bag claim ceiling promotion detected")


def validate_precommit_admission(
    spec: TransactionSpec,
    artifacts: Mapping[str, bytes],
    runtime_contract: Mapping[str, Any],
    artifact_bundle: Mapping[str, Any],
) -> None:
    """Bind the exact contract, mandatory inventory, bytes and QA before staging."""

    validate_runtime_contract_and_bundle(runtime_contract, artifact_bundle)
    contract_sha = sha256_bytes(canonical_json(runtime_contract))
    if spec.runtime_contract_sha256 != contract_sha:
        raise S6TransactionV7Error("transaction spec/runtime contract SHA differs")
    runtime_paths = runtime_contract["runtime_paths"]
    expected_paths = {
        "partial_root": str(spec.partial_root),
        "final_root": str(spec.final_root),
        "comparison_ledger": str(spec.ledger_path),
        "final_transaction_receipt": str(spec.final_receipt_path),
        "evidence_index": str(spec.final_root / "evidence_index.json"),
        "checksums": str(spec.final_root / "checksums.sha256"),
    }
    if runtime_paths != expected_paths:
        raise S6TransactionV7Error("runtime paths differ from exact transaction spec")
    admission = runtime_contract["transaction_admission"]
    if (admission["transaction_id"] != spec.transaction_id
            or admission["expected_previous_ledger_sha256"] !=
            spec.expected_previous_ledger_sha256):
        raise S6TransactionV7Error("transaction admission identity differs")
    normal = _normalise_artifacts(artifacts)
    required = artifact_bundle["required_artifacts"]
    if list(normal) != required:
        raise S6TransactionV7Error("mandatory stage6 artifact inventory differs")
    inventory = _expected_inventory(normal)
    if (artifact_bundle["inventory"] != inventory
            or artifact_bundle["inventory_sha256"] != sha256_json(inventory)):
        raise S6TransactionV7Error("artifact bundle byte inventory differs")
    checksums = normal["checksums.sha256"].decode("ascii", "strict").splitlines()
    expected_checksums = [
        f"{sha256_bytes(normal[name])}  {name}"
        for name in sorted(set(normal) - {"checksums.sha256"})
    ]
    if checksums != expected_checksums:
        raise S6TransactionV7Error("checksums manifest does not cover exact bundle")
    quality = artifact_bundle["quality"]
    quality_bindings = {
        "figure_manifest_sha256": "reports/figure_manifest.json",
        "media_manifest_sha256": "reports/media_manifest.json",
        "visual_qa_sha256": "reports/visual_qa.json",
        "comparison_manifest_sha256": "comparison_manifest.json",
        "evidence_index_sha256": "evidence_index.json",
        "checksums_sha256": "checksums.sha256",
    }
    if any(quality[key] != sha256_bytes(normal[path])
           for key, path in quality_bindings.items()):
        raise S6TransactionV7Error("quality evidence hash binding differs")

    def exact_json(path: str, label: str) -> dict[str, Any]:
        return _parse_json_object(normal[path], label)

    analysis = exact_json("reports/analysis_result.json", "analysis result")
    comparison = exact_json("comparison_manifest.json", "comparison manifest")
    figure = exact_json("reports/figure_manifest.json", "figure manifest")
    media = exact_json("reports/media_manifest.json", "media manifest")
    visual = exact_json("reports/visual_qa.json", "visual QA")
    evidence = exact_json("evidence_index.json", "evidence index")
    quality_control = exact_json("reports/quality_control.json", "quality control")
    if (analysis.get("status") != quality["analysis_status"]
            or analysis.get("attempt_id") != ATTEMPT_ID
            or analysis.get("planned_denominator") != PLANNED_DENOMINATOR
            or analysis.get("source_outcome") != SOURCE_OUTCOME):
        raise S6TransactionV7Error("analysis result identity/status differs")
    ceiling = {
        "paired_ranking": False, "cross_method_ranking": False,
        "selected_trajectory_cpu_comparison": False,
        "physical_reference_pending": True,
        "physical_fidelity_validated": False, "formal": False,
        "production": False,
    }
    if (comparison.get("status") !=
            "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY"
            or comparison.get("attempt_id") != ATTEMPT_ID
            or comparison.get("planned_denominator") != PLANNED_DENOMINATOR
            or comparison.get("source_outcome") != SOURCE_OUTCOME
            or any(comparison.get(key) != value for key, value in ceiling.items())):
        raise S6TransactionV7Error("comparison manifest status/claim ceiling differs")
    if (figure.get("source_analysis_sha256") != sha256_json(analysis)
            or figure.get("layout") != "THREE_VERTICAL_SHARED_X_PANELS"
            or figure.get("formats") != quality["figure_formats"]
            or figure.get("dual_y_axes") is not False
            or figure.get("palette") != "OKABE_ITO"
            or figure.get("redundant_line_styles") is not True):
        raise S6TransactionV7Error("figure manifest semantic binding differs")
    figure_qa = figure.get("qa", {})
    expected_figure_qa = {
        "color_render_pass": True, "grayscale_render_pass": True,
        "svg_render_pass": True, "no_clipping": True,
        "no_missing_glyphs": True, "no_dual_y_axis": True,
        "source_data_hash_bound": True, "multimodal_visual_review": True,
    }
    if figure_qa != expected_figure_qa:
        raise S6TransactionV7Error("figure programmatic/multimodal QA differs")
    figure_files = {
        "PNG": "figures/primary_shared_x_timeseries.png",
        "PDF": "figures/primary_shared_x_timeseries.pdf",
        "SVG": "figures/primary_shared_x_timeseries.svg",
        "GRAYSCALE_PNG": "figures/primary_shared_x_timeseries_grayscale.png",
    }
    if set(figure.get("artifacts", {})) != set(figure_files.values()):
        raise S6TransactionV7Error("figure artifact manifest set differs")
    for path in figure_files.values():
        expected = {"sha256": sha256_bytes(normal[path]), "size_bytes": len(normal[path])}
        if figure["artifacts"].get(path) != expected:
            raise S6TransactionV7Error("figure artifact byte binding differs")
    expected_visual = {
        "schema_version": "smpcc-r8-liquid-s6-multimodal-visual-qa-v7",
        "status": "PASS_S6_MULTIMODAL_VISUAL_QA_V7",
        "reviewed_preview_sha256": sha256_bytes(normal[figure_files["PNG"]]),
        "reviewed_grayscale_sha256": sha256_bytes(normal[figure_files["GRAYSCALE_PNG"]]),
        "no_clipping": True, "no_missing_glyphs": True,
        "no_legend_occlusion": True, "panel_alignment": True,
        "grayscale_distinguishable": True, "data_not_visually_clipped": True,
        "cross_panel_units_consistent": True,
    }
    if visual != expected_visual:
        raise S6TransactionV7Error("multimodal visual evidence differs")
    media_files = {
        "animation/primary.mp4", "animation/primary_preview.gif",
        "keyframes/primary_first.png", "keyframes/primary_middle.png",
        "keyframes/primary_last.png",
    }
    if (media.get("schema_version") != "smpcc-r8-liquid-s6-media-manifest-v7"
            or media.get("attempt_id") != ATTEMPT_ID
            or media.get("numeric_fact_source") is not False
            or media.get("frame_count") != len(media.get("frames", []))
            or media.get("frame_count", 0) < 3
            or media.get("fps", 0) <= 0
            or abs(media.get("duration_s", -1) - media["frame_count"] / media["fps"]) > 1e-9
            or set(media.get("artifacts", {})) != media_files
            or len(media.get("keyframes", {})) != 3):
        raise S6TransactionV7Error("media manifest semantic contract differs")
    for path in media_files:
        expected = {"sha256": sha256_bytes(normal[path]), "size_bytes": len(normal[path])}
        if media["artifacts"].get(path) != expected:
            raise S6TransactionV7Error("media artifact byte binding differs")
    frame_rows = media["frames"]
    if [row.get("index") for row in frame_rows] != list(range(len(frame_rows))):
        raise S6TransactionV7Error("media frame index order differs")
    for path, expected_index in {
        "keyframes/primary_first.png": 0,
        "keyframes/primary_middle.png": len(frame_rows) // 2,
        "keyframes/primary_last.png": len(frame_rows) - 1,
    }.items():
        row = media["keyframes"].get(path, {})
        if (row.get("sha256") != sha256_bytes(normal[path])
                or row.get("source_index") != expected_index
                or row.get("source_rendered_png_sha256") !=
                frame_rows[expected_index].get("rendered_png_sha256")):
            raise S6TransactionV7Error("keyframe source/hash binding differs")
    if (quality_control != {
            "canonical_grid": True, "valid_probes_per_slot": 16,
            "optional_unread": True, "PHYSICAL_REFERENCE_PENDING": True,
            "visual_qa_programmatic": True, "visual_qa_human_pending": False}):
        raise S6TransactionV7Error("quality-control claim differs")
    expected_evidence_paths = sorted(set(normal) - {"evidence_index.json", "checksums.sha256"})
    expected_evidence = [
        {"relative_path": path, "sha256": sha256_bytes(normal[path]),
         "size_bytes": len(normal[path])}
        for path in expected_evidence_paths
    ]
    if (evidence != {
            "schema_version": "smpcc-r8-liquid-s6-evidence-index-v7",
            "attempt_id": ATTEMPT_ID, "planned_denominator": PLANNED_DENOMINATOR,
            "source_outcome": SOURCE_OUTCOME, "entries": expected_evidence,
            "excluded_self_referential_paths": ["checksums.sha256", "evidence_index.json"],
            "optional_unread": True, "physical_reference_pending": True}):
        raise S6TransactionV7Error("evidence index coverage or byte binding differs")


def _entry(payload: Mapping[str, Any], previous_sha256: str) -> dict[str, Any]:
    return {
        "entry_sha256": sha256_json(payload),
        "previous_entry_sha256": _sha(previous_sha256, "entry predecessor"),
        "payload": dict(payload),
    }


def _prepared_payload(spec: TransactionSpec, identity: Mapping[str, Any],
                      admission_sha256: str) -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-ledger-entry-v7",
        "phase": "PREPARED",
        "transaction_id": spec.transaction_id,
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": spec.runtime_contract_sha256,
        "precommit_bundle_sha256": _sha(admission_sha256, "precommit bundle"),
        "expected_previous_ledger_sha256": spec.expected_previous_ledger_sha256,
        "final_root": str(spec.final_root),
        "bundle_sha256": identity["bundle_sha256"],
        "inventory_sha256": identity["inventory_sha256"],
        "file_count": identity["file_count"],
        "staging_complete": True,
        "staging_fsynced": True,
        "stage6_pass": False,
    }


def _committed_payload(
    spec: TransactionSpec,
    identity: Mapping[str, Any],
    prepared_entry_sha256: str,
    admission_sha256: str,
) -> dict[str, Any]:
    return {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-ledger-entry-v7",
        "phase": "COMMITTED",
        "transaction_id": spec.transaction_id,
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": spec.runtime_contract_sha256,
        "precommit_bundle_sha256": _sha(admission_sha256, "precommit bundle"),
        "final_root": str(spec.final_root),
        "bundle_sha256": identity["bundle_sha256"],
        "inventory_sha256": identity["inventory_sha256"],
        "file_count": identity["file_count"],
        "prepared_entry_sha256": prepared_entry_sha256,
        "publication_rename_noreplace": True,
        "publication_parent_fsynced": True,
        "final_tree_revalidated": True,
        "stage6_pass": False,
    }


def _validate_ledger_entries(raw: bytes) -> list[dict[str, Any]]:
    if len(raw) > MAXIMUM_LEDGER_BYTES or (raw and not raw.endswith(b"\n")):
        raise S6TransactionV7Error("ledger is oversized or truncated")
    previous = ZERO_SHA256
    transaction_phases: dict[str, list[str]] = {}
    entries: list[dict[str, Any]] = []
    for index, line in enumerate(raw.splitlines(keepends=True)):
        value = _parse_json_object(line, f"ledger line {index}")
        if set(value) != {"entry_sha256", "previous_entry_sha256", "payload"}:
            raise S6TransactionV7Error("ledger wrapper is not closed")
        if value["previous_entry_sha256"] != previous:
            raise S6TransactionV7Error("ledger chain predecessor differs")
        payload = value["payload"]
        if not isinstance(payload, dict) or value["entry_sha256"] != sha256_json(payload):
            raise S6TransactionV7Error("ledger entry digest differs")
        transaction_id = payload.get("transaction_id")
        phase = payload.get("phase")
        if TRANSACTION_ID_RE.fullmatch(str(transaction_id)) is None or phase not in {"PREPARED", "COMMITTED"}:
            raise S6TransactionV7Error("ledger payload identity or phase differs")
        phases = transaction_phases.setdefault(str(transaction_id), [])
        phases.append(str(phase))
        if phases not in (["PREPARED"], ["PREPARED", "COMMITTED"]):
            raise S6TransactionV7Error("ledger has duplicate or out-of-order transaction phase")
        entries.append(value)
        previous = value["entry_sha256"]
    return entries


def _open_locked_ledger(spec: TransactionSpec) -> tuple[int, list[dict[str, Any]], bool]:
    existed = spec.ledger_path.exists()
    descriptor = os.open(
        spec.ledger_path,
        os.O_RDWR | os.O_APPEND | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC,
        LEDGER_MODE,
    )
    try:
        if not existed:
            os.fchmod(descriptor, LEDGER_MODE)
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != LEDGER_MODE
        ):
            raise S6TransactionV7Error("ledger is not regular, single-link, mode 0600")
        os.lseek(descriptor, 0, os.SEEK_SET)
        entries = _validate_ledger_entries(_read_fd(descriptor, MAXIMUM_LEDGER_BYTES))
        return descriptor, entries, existed
    except BaseException:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)
        raise


def _append_locked(descriptor: int, entry: Mapping[str, Any]) -> None:
    encoded = canonical_json(entry)
    before = os.fstat(descriptor)
    _write_all(descriptor, encoded)
    os.fsync(descriptor)
    after = os.fstat(descriptor)
    if (
        (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino)
        or after.st_size != before.st_size + len(encoded)
    ):
        raise S6TransactionV7Error("ledger append identity or size differs")


def _fsync_parent(path: Path) -> None:
    descriptor = os.open(
        path.parent,
        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _transaction_entries(
    entries: Sequence[Mapping[str, Any]], transaction_id: str
) -> list[Mapping[str, Any]]:
    return [entry for entry in entries if entry["payload"]["transaction_id"] == transaction_id]


def _expected_entries(
    spec: TransactionSpec,
    identity: Mapping[str, Any],
    entries: Sequence[Mapping[str, Any]],
    admission_sha256: str,
) -> tuple[Mapping[str, Any] | None, Mapping[str, Any] | None]:
    selected = _transaction_entries(entries, spec.transaction_id)
    if len(selected) > 2:
        raise S6TransactionV7Error("transaction has duplicate ledger entries")
    prepared = selected[0] if selected else None
    committed = selected[1] if len(selected) == 2 else None
    expected_prepared_payload = _prepared_payload(spec, identity, admission_sha256)
    if prepared is not None:
        if (
            prepared["payload"] != expected_prepared_payload
            or prepared["previous_entry_sha256"] != spec.expected_previous_ledger_sha256
            or prepared["entry_sha256"] != sha256_json(expected_prepared_payload)
        ):
            raise S6TransactionV7Error("PREPARED entry differs from exact transaction")
    if committed is not None:
        expected_committed_payload = _committed_payload(
            spec, identity, prepared["entry_sha256"], admission_sha256
        )
        if (
            committed["payload"] != expected_committed_payload
            or committed["previous_entry_sha256"] != prepared["entry_sha256"]
            or committed["entry_sha256"] != sha256_json(expected_committed_payload)
        ):
            raise S6TransactionV7Error("COMMITTED entry differs from exact transaction")
    return prepared, committed


def _tip(entries: Sequence[Mapping[str, Any]]) -> str:
    return str(entries[-1]["entry_sha256"]) if entries else ZERO_SHA256


def _inject(selected: str | None, location: str) -> None:
    if selected == location:
        raise InjectedFailure(f"injected failure: {location}")


def _receipt_payload(
    spec: TransactionSpec,
    identity: Mapping[str, Any],
    prepared: Mapping[str, Any],
    committed: Mapping[str, Any],
    admission_sha256: str,
) -> dict[str, Any]:
    """Build stored receipt bytes; deliberately omit the receipt's own hash."""

    return {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-final-transaction-receipt-v7",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_RUNTIME_FINAL_TRANSACTION_RECEIPT_V7",
        "status": "FINAL_TRANSACTION_RECEIPT_CREATED_AWAITING_CONSUMER_V7",
        "transaction_id": spec.transaction_id,
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": spec.runtime_contract_sha256,
        "precommit_bundle_sha256": _sha(admission_sha256, "precommit bundle"),
        "expected_previous_ledger_sha256": spec.expected_previous_ledger_sha256,
        "final_root": str(spec.final_root),
        "bundle_sha256": identity["bundle_sha256"],
        "inventory_sha256": identity["inventory_sha256"],
        "file_count": identity["file_count"],
        "ledger_path": str(spec.ledger_path),
        "prepared_entry_sha256": prepared["entry_sha256"],
        "committed_entry_sha256": committed["entry_sha256"],
        "create_new_o_excl": True,
        "stage6_pass": False,
        "physical_reference_pending": True,
        "physical_fidelity_validated": False,
    }


def _read_exact_receipt(path: Path) -> tuple[bytes, os.stat_result]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or stat.S_IMODE(before.st_mode) != FILE_MODE
        ):
            raise S6TransactionV7Error("receipt is not regular, single-link, mode 0440")
        raw = _read_fd(descriptor, MAXIMUM_FILE_BYTES)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (
        before.st_dev,
        before.st_ino,
        before.st_mode,
        before.st_nlink,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    ) != (
        after.st_dev,
        after.st_ino,
        after.st_mode,
        after.st_nlink,
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    ):
        raise S6TransactionV7Error("receipt TOCTOU detected")
    return raw, after


def _create_or_verify_receipt(path: Path, expected: Mapping[str, Any]) -> tuple[bytes, os.stat_result]:
    encoded = canonical_json(expected)
    try:
        descriptor = os.open(
            path,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            FILE_MODE,
        )
    except FileExistsError:
        raw, metadata = _read_exact_receipt(path)
        if raw != encoded:
            raise S6TransactionV7Error("existing final receipt differs")
        return raw, metadata
    try:
        os.fchmod(descriptor, FILE_MODE)
        _write_all(descriptor, encoded)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    _fsync_parent(path)
    raw, metadata = _read_exact_receipt(path)
    if raw != encoded:
        raise S6TransactionV7Error("created final receipt bytes differ")
    return raw, metadata


def execute_transaction(
    spec: TransactionSpec,
    artifacts: Mapping[str, bytes],
    runtime_contract: Mapping[str, Any],
    artifact_bundle: Mapping[str, Any],
    *,
    fail_after: str | None = None,
) -> dict[str, Any]:
    """Execute or recover one exact transaction, then run the strict consumer.

    ``fail_after`` is exclusively a deterministic unit-test hook.  Recalling
    this function with the same immutable spec and artifacts resumes from a
    valid durable boundary; it never overwrites a root, ledger entry, or
    receipt.
    """

    validate_spec(spec)
    validate_precommit_admission(spec, artifacts, runtime_contract, artifact_bundle)
    if fail_after is not None and fail_after not in FAILURE_POINTS:
        raise S6TransactionV7Error("unknown fault injection point")
    normal = _normalise_artifacts(artifacts)
    identity = bundle_identity(spec, normal)
    admission_sha256 = precommit_bundle_sha256(artifact_bundle)

    final_exists = spec.final_root.exists()
    partial_exists = spec.partial_root.exists()
    if final_exists and partial_exists:
        raise S6TransactionV7Error("both partial and final roots exist")
    if not final_exists and not partial_exists:
        _stage_fresh(spec, normal)
        _verify_tree(spec.partial_root, identity)
    elif partial_exists:
        _verify_tree(spec.partial_root, identity)
    else:
        _verify_tree(spec.final_root, identity)
    _inject(fail_after, "after_staging")

    ledger_fd, entries, ledger_existed = _open_locked_ledger(spec)
    try:
        prepared, committed = _expected_entries(spec, identity, entries, admission_sha256)
        if prepared is None:
            if spec.final_root.exists() or not spec.partial_root.exists():
                raise S6TransactionV7Error("final root exists without matching PREPARED entry")
            if _tip(entries) != spec.expected_previous_ledger_sha256:
                raise S6TransactionV7Error("stale expected ledger predecessor")
            prepared = _entry(
                _prepared_payload(spec, identity, admission_sha256),
                spec.expected_previous_ledger_sha256,
            )
            _append_locked(ledger_fd, prepared)
            entries = [*entries, prepared]
            if not ledger_existed:
                _fsync_parent(spec.ledger_path)
        if _tip(entries) not in {prepared["entry_sha256"], (
            committed["entry_sha256"] if committed is not None else ""
        )}:
            raise S6TransactionV7Error("transaction ledger phases were interleaved")
        _inject(fail_after, "after_prepared")

        if committed is None:
            if spec.partial_root.exists():
                if spec.final_root.exists():
                    raise S6TransactionV7Error("publication roots conflict")
                _publish(spec)
            elif not spec.final_root.exists():
                raise S6TransactionV7Error("PREPARED transaction has no recoverable root")
            _inject(fail_after, "after_publish")
            _verify_tree(spec.final_root, identity)
            _inject(fail_after, "after_revalidate")
            if _tip(entries) != prepared["entry_sha256"]:
                raise S6TransactionV7Error("COMMITTED predecessor is not PREPARED")
            committed = _entry(
                _committed_payload(spec, identity, prepared["entry_sha256"], admission_sha256),
                prepared["entry_sha256"],
            )
            _append_locked(ledger_fd, committed)
            entries = [*entries, committed]
        else:
            if spec.partial_root.exists() or not spec.final_root.exists():
                raise S6TransactionV7Error("COMMITTED transaction root state differs")
            _verify_tree(spec.final_root, identity)
        _inject(fail_after, "after_committed")
    finally:
        try:
            fcntl.flock(ledger_fd, fcntl.LOCK_UN)
        finally:
            os.close(ledger_fd)

    expected_receipt = _receipt_payload(spec, identity, prepared, committed, admission_sha256)
    _create_or_verify_receipt(spec.final_receipt_path, expected_receipt)
    _inject(fail_after, "after_receipt")
    return consume_transaction(spec, artifacts, runtime_contract, artifact_bundle)


def consume_transaction(
    spec: TransactionSpec,
    artifacts: Mapping[str, bytes],
    runtime_contract: Mapping[str, Any],
    artifact_bundle: Mapping[str, Any],
) -> dict[str, Any]:
    """Return PASS only when final, PREPARED, COMMITTED and receipt agree."""

    validate_spec(spec)
    validate_precommit_admission(spec, artifacts, runtime_contract, artifact_bundle)
    identity = bundle_identity(spec, _normalise_artifacts(artifacts))
    admission_sha256 = precommit_bundle_sha256(artifact_bundle)
    if spec.partial_root.exists() or not spec.final_root.exists():
        raise S6TransactionV7Error("consumer requires only the exact final root")
    _verify_tree(spec.final_root, identity)

    descriptor = os.open(
        spec.ledger_path,
        os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
    )
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != LEDGER_MODE
        ):
            raise S6TransactionV7Error("consumer ledger identity differs")
        raw_ledger = _read_fd(descriptor, MAXIMUM_LEDGER_BYTES)
    finally:
        os.close(descriptor)
    entries = _validate_ledger_entries(raw_ledger)
    prepared, committed = _expected_entries(spec, identity, entries, admission_sha256)
    if prepared is None or committed is None:
        raise S6TransactionV7Error("consumer requires PREPARED and COMMITTED")
    selected = _transaction_entries(entries, spec.transaction_id)
    if selected != [prepared, committed]:
        raise S6TransactionV7Error("consumer transaction phase set differs")

    raw_receipt, receipt_metadata = _read_exact_receipt(spec.final_receipt_path)
    receipt = _parse_json_object(raw_receipt, "final receipt")
    expected_receipt = _receipt_payload(spec, identity, prepared, committed, admission_sha256)
    if receipt != expected_receipt:
        raise S6TransactionV7Error("consumer final receipt binding differs")
    receipt_sha256 = sha256_bytes(raw_receipt)

    report = {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-transaction-receipt-v7",
        "document_type": "SMPCC_R8_LIQUID_S6_REAL_RUNTIME_TRANSACTION_RECEIPT_V7",
        "status": "COMMITTED_RECEIPT_CONSISTENT",
        "transaction_id": spec.transaction_id,
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": spec.runtime_contract_sha256,
        "precommit_bundle_sha256": admission_sha256,
        "expected_previous_ledger_sha256": spec.expected_previous_ledger_sha256,
        "staging": {
            "partial_root": str(spec.partial_root),
            "complete": True,
            "bundle_sha256": identity["bundle_sha256"],
            "inventory_sha256": identity["inventory_sha256"],
            "file_count": identity["file_count"],
            "fsynced": True,
        },
        "publication": {
            "final_root": str(spec.final_root),
            "performed": True,
            "rename_noreplace": True,
            "parent_fsynced": True,
            "final_root_inventory_sha256": identity["inventory_sha256"],
        },
        "ledger": {
            "path": str(spec.ledger_path),
            "prepared_appended": True,
            "prepared_entry_sha256": prepared["entry_sha256"],
            "prepared_stage6_pass": False,
            "committed_appended": True,
            "committed_entry_sha256": committed["entry_sha256"],
            "fsynced": True,
        },
        "receipt": {
            "path": str(spec.final_receipt_path),
            "created": True,
            "create_new_o_excl": True,
            "sha256": receipt_sha256,
        },
        "consumer_acceptance": {
            "accepted": True,
            "exact_final_root": True,
            "prepared_matches_final": True,
            "committed_matches_final": True,
            "receipt_matches_committed": True,
            "three_way_consistent": True,
        },
        "claims": {
            "stage6_pass": True,
            "development_only": True,
            "physical_reference_pending": True,
            "physical_fidelity_validated": False,
            "paired_ranking": False,
            "cross_method_ranking": False,
            "selected_trajectory_cpu_comparison": False,
            "formal": False,
            "production": False,
            "physical_primary": False,
        },
    }
    schema = json.loads(TRANSACTION_SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(report)
    if receipt_metadata.st_size != len(raw_receipt):
        raise S6TransactionV7Error("consumer receipt size differs")
    return report


def self_check() -> dict[str, Any]:
    """Perform schema/static checks only; never materialize a transaction."""

    schemas = [json.loads(path.read_bytes()) for path in (
        TRANSACTION_SCHEMA_PATH, RUNTIME_CONTRACT_SCHEMA_PATH, ARTIFACT_BUNDLE_SCHEMA_PATH)]
    for schema in schemas:
        Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    sample = {
        "schema_version": "smpcc-r8-liquid-s6-real-runtime-bundle-identity-v7",
        "transaction_id": "s6-v7-static-self-check",
        "attempt_id": ATTEMPT_ID,
        "planned_denominator": PLANNED_DENOMINATOR,
        "source_outcome": SOURCE_OUTCOME,
        "runtime_contract_sha256": "a" * 64,
        "final_root": "/not-materialized/s6-v7",
        "inventory_sha256": "b" * 64,
        "file_count": 1,
    }
    digest = sha256_json(sample)
    return {
        "status": "S6_REAL_RUNTIME_TRANSACTION_V7_SELF_CHECK_OK_NOT_ADMITTED",
        "transaction_schema_deep_closed": True,
        "runtime_contract_schema_deep_closed": True,
        "artifact_bundle_schema_deep_closed": True,
        "canonical_frame_v2_and_claim_ceiling_required": True,
        "bundle_identity_sha256": digest,
        "ordered_phases": [
            "STAGING_FSYNCED",
            "PREPARED_FSYNCED_STAGE6_FALSE",
            "RENAME_NOREPLACE_PARENT_FSYNCED",
            "FINAL_TREE_REVALIDATED",
            "COMMITTED_FSYNCED",
            "EXTERNAL_O_EXCL_RECEIPT",
            "FOUR_WAY_CONSUMER_ACCEPTANCE",
        ],
        "receipt_hash_self_reference": False,
        "external_root_materialized": False,
        "real_bag_read": False,
        "optional_bag_read": False,
        "candidate_executed": False,
        "solver_or_gpu_executed": False,
        "sudo_used": False,
        "network_used": False,
        "stage6_pass": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        report = self_check()
    except (OSError, S6TransactionV7Error, ValueError) as exc:
        print(
            json.dumps({"status": "FAIL_CLOSED", "error": str(exc)}, sort_keys=True),
            file=sys.stderr,
        )
        return 2
    print(canonical_json(report).decode("utf-8"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ATTEMPT_ID",
    "FAILURE_POINTS",
    "InjectedFailure",
    "S6TransactionV7Error",
    "TransactionSpec",
    "assert_deep_closed",
    "bundle_identity",
    "canonical_json",
    "consume_transaction",
    "execute_transaction",
    "self_check",
    "sha256_bytes",
    "sha256_json",
]
