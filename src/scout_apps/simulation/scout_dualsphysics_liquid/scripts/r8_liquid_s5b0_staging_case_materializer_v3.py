#!/usr/bin/env python3
"""Create-new S5B0 staging/case materializer with a fixture-only CLI."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5a1_motion_bridge_v1 as bridge


ROOT = MODULE_DIR.parent
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_staging_case_manifest_v3.json"
ROLES = (
    "candidate", "dsph_config", "case_xml", "case_bi4",
    "restart_part", "restart_head", "solver_path",
)
DESTINATIONS: dict[str, tuple[str, int]] = {
    "candidate": ("runtime/candidate", 0o500),
    "dsph_config": ("runtime/DsphConfig.xml", 0o400),
    "case_xml": ("case/C1M_case.xml", 0o400),
    "case_bi4": ("case/C1M_case.bi4", 0o400),
    "restart_part": ("restart/Part_0901.bi4", 0o400),
    "restart_head": ("restart/Part_Head.ibi4", 0o400),
    "solver_path": ("case/solver_path.csv", 0o400),
}
SOURCE_IDENTITY_KEYS = {"path", "mode", "size_bytes", "device", "inode", "nlink", "sha256"}
MAX_SOURCE_BYTES = 256 * 1024 * 1024
MAX_XML_BYTES = 2 * 1024 * 1024
MAX_SOLVER_PATH_BYTES = 16 * 1024 * 1024
FLOAT_TOLERANCE = 1e-12
MOTION_BLOCK_RE = re.compile(
    rb"(?ms)^(?P<indent>[ \t]*)<motion>[\s\S]*?^(?P=indent)</motion>"
)


class StagingMaterializerError(ValueError):
    """An external identity, motion contract, or fresh-root invariant failed."""


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")).encode()


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise StagingMaterializerError(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def _exact_absolute(path: Path, label: str) -> Path:
    text = str(path)
    if not path.is_absolute() or text != os.path.normpath(text):
        raise StagingMaterializerError(f"{label} is not an exact absolute path")
    return path


def _assert_no_symlink_components(path: Path, *, final_may_be_absent: bool = False) -> None:
    current = Path(path.anchor)
    for index, part in enumerate(path.parts[1:], start=1):
        current /= part
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            if final_may_be_absent and index == len(path.parts) - 1:
                return
            raise StagingMaterializerError(f"path component is absent: {current}")
        if stat.S_ISLNK(metadata.st_mode):
            raise StagingMaterializerError(f"symlink path component: {current}")
        if index < len(path.parts) - 1 and not stat.S_ISDIR(metadata.st_mode):
            raise StagingMaterializerError(f"non-directory path component: {current}")


def _identity(path: str, metadata: os.stat_result, digest: str) -> dict[str, Any]:
    return {
        "path": path,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "size_bytes": metadata.st_size,
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
        "sha256": digest,
    }


def _read_opened(descriptor: int, path: str, *, maximum: int) -> tuple[bytes, dict[str, Any], os.stat_result]:
    before = os.fstat(descriptor)
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
        raise StagingMaterializerError(f"source is not a single-link regular file: {path}")
    if before.st_size > maximum:
        raise StagingMaterializerError(f"source exceeds bound: {path}")
    chunks: list[bytes] = []
    digest = hashlib.sha256()
    total = 0
    while True:
        block = os.read(descriptor, min(1 << 20, maximum + 1 - total))
        if not block:
            break
        total += len(block)
        if total > maximum:
            raise StagingMaterializerError(f"source grew beyond bound: {path}")
        digest.update(block)
        chunks.append(block)
    after = os.fstat(descriptor)
    if (
        (before.st_dev, before.st_ino, before.st_mode, before.st_nlink, before.st_size, before.st_mtime_ns)
        != (after.st_dev, after.st_ino, after.st_mode, after.st_nlink, after.st_size, after.st_mtime_ns)
        or total != before.st_size
    ):
        raise StagingMaterializerError(f"source TOCTOU detected: {path}")
    return b"".join(chunks), _identity(path, before, digest.hexdigest()), before


def observe_source(path: Path, *, maximum: int = MAX_SOURCE_BYTES) -> dict[str, Any]:
    path = _exact_absolute(Path(path), "source")
    _assert_no_symlink_components(path)
    path_metadata = os.lstat(path)
    if not stat.S_ISREG(path_metadata.st_mode) or path_metadata.st_nlink != 1:
        raise StagingMaterializerError(f"source is not a single-link regular file: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        _data, identity, _metadata = _read_opened(descriptor, str(path), maximum=maximum)
        return identity
    finally:
        os.close(descriptor)


def _open_exact_sources(
    expected: Mapping[str, Mapping[str, Any]],
) -> tuple[dict[str, bytes], dict[str, int], dict[str, os.stat_result]]:
    if set(expected) != set(ROLES):
        raise StagingMaterializerError("external source role set differs")
    payloads: dict[str, bytes] = {}
    descriptors: dict[str, int] = {}
    metadata: dict[str, os.stat_result] = {}
    try:
        for role in ROLES:
            specification = expected[role]
            if not isinstance(specification, Mapping) or set(specification) != SOURCE_IDENTITY_KEYS:
                raise StagingMaterializerError(f"source identity is not closed: {role}")
            path = _exact_absolute(Path(str(specification["path"])), role)
            _assert_no_symlink_components(path)
            path_metadata = os.lstat(path)
            if not stat.S_ISREG(path_metadata.st_mode) or path_metadata.st_nlink != 1:
                raise StagingMaterializerError(f"source is not a single-link regular file: {path}")
            descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
            descriptors[role] = descriptor
            maximum = MAX_XML_BYTES if role == "case_xml" else MAX_SOLVER_PATH_BYTES if role == "solver_path" else MAX_SOURCE_BYTES
            data, observed, source_metadata = _read_opened(descriptor, str(path), maximum=maximum)
            if observed != dict(specification):
                raise StagingMaterializerError(f"external source identity differs: {role}")
            if role == "candidate" and stat.S_IMODE(source_metadata.st_mode) & 0o111:
                raise StagingMaterializerError("source candidate is executable")
            payloads[role] = data
            metadata[role] = source_metadata
        return payloads, descriptors, metadata
    except BaseException:
        for descriptor in descriptors.values():
            os.close(descriptor)
        raise


def _postverify_sources(
    expected: Mapping[str, Mapping[str, Any]],
    descriptors: Mapping[str, int],
    initial: Mapping[str, os.stat_result],
) -> None:
    for role in ROLES:
        descriptor = descriptors[role]
        before = initial[role]
        after = os.fstat(descriptor)
        path = Path(str(expected[role]["path"]))
        current = os.lstat(path)
        fields = lambda value: (
            value.st_dev, value.st_ino, value.st_mode, value.st_nlink,
            value.st_size, value.st_mtime_ns,
        )
        if fields(after) != fields(before) or fields(current) != fields(before):
            raise StagingMaterializerError(f"source TOCTOU detected after staging: {role}")
        os.lseek(descriptor, 0, os.SEEK_SET)
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise StagingMaterializerError(f"source shortened after staging: {role}")
            remaining -= len(block)
            digest.update(block)
        if digest.hexdigest() != expected[role]["sha256"]:
            raise StagingMaterializerError(f"source content changed after staging: {role}")


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise StagingMaterializerError(f"non-finite {label}")
    return float(value)


def _solver_rows(source: bytes, solver_tail_s: float) -> tuple[bridge.SolverPathRow, ...]:
    try:
        text = source.decode("utf-8")
        rows = bridge.parse_solver_path_csv(text)
    except (UnicodeError, bridge.MotionBridgeError) as exc:
        raise StagingMaterializerError(f"solver_path.csv is invalid: {exc}") from exc
    tail = _finite(solver_tail_s, "solver tail")
    if tail <= 0 or len(rows) < 3:
        raise StagingMaterializerError("solver path lacks motion plus tail")
    first = rows[0].numeric_fields()
    if any(abs(value) > FLOAT_TOLERANCE for value in first):
        raise StagingMaterializerError("solver path t0 is not identity")
    penultimate, final = rows[-2].numeric_fields(), rows[-1].numeric_fields()
    if abs((final[0] - penultimate[0]) - tail) > FLOAT_TOLERANCE or final[1:] != penultimate[1:]:
        raise StagingMaterializerError("solver path final row is not the exact pose-hold tail")
    return rows


def _number(value: float) -> str:
    return format(value, ".17g")


def _motion_block(indent: bytes, settled_time_s: float, duration_s: float) -> bytes:
    prefix = indent.decode("ascii")
    lines = [
        f"{prefix}<motion>",
        f"{prefix}    <objreal ref=\"0\">",
        f"{prefix}        <begin mov=\"1\" start=\"{_number(settled_time_s)}\" />",
        f"{prefix}        <mvpathfile id=\"1\" duration=\"{_number(duration_s)}\" next=\"2\" anglesunits=\"degrees\">",
        f"{prefix}            <file name=\"solver_path.csv\" fields=\"7\" fieldtime=\"0\" fieldx=\"1\" fieldy=\"2\" fieldz=\"3\" fieldang1=\"4\" fieldang2=\"5\" fieldang3=\"6\" />",
        f"{prefix}            <center x=\"0\" y=\"0\" z=\"0\" />",
        f"{prefix}            <movecenter value=\"true\" />",
        f"{prefix}            <intrinsic value=\"true\" />",
        f"{prefix}            <axes value=\"ZYX\" />",
        f"{prefix}        </mvpathfile>",
        f"{prefix}        <mvnull id=\"2\" />",
        f"{prefix}    </objreal>",
        f"{prefix}</motion>",
    ]
    return "\n".join(lines).encode()


def render_case(source: bytes, *, settled_time_s: float, duration_s: float) -> bytes:
    if not source or len(source) > MAX_XML_BYTES or b"\0" in source:
        raise StagingMaterializerError("base case XML size/content is invalid")
    if b"<!DOCTYPE" in source.upper() or b"<!ENTITY" in source.upper():
        raise StagingMaterializerError("base case XML declarations are forbidden")
    matches = list(MOTION_BLOCK_RE.finditer(source))
    if len(matches) != 2:
        raise StagingMaterializerError("base case must contain exactly two motion blocks")
    output: list[bytes] = []
    offset = 0
    for match in matches:
        output.append(source[offset:match.start()])
        output.append(_motion_block(match.group("indent"), settled_time_s, duration_s))
        offset = match.end()
    output.append(source[offset:])
    rendered = b"".join(output)
    validate_rendered_case(rendered, settled_time_s=settled_time_s, duration_s=duration_s)
    return rendered


def validate_rendered_case(source: bytes, *, settled_time_s: float, duration_s: float) -> None:
    try:
        root = ET.fromstring(source)
    except ET.ParseError as exc:
        raise StagingMaterializerError(f"rendered case XML is invalid: {exc}") from exc
    if root.tag != "case" or root.find("./casedef") is None or root.find("./execution") is None:
        raise StagingMaterializerError("rendered case root/casedef/execution differs")
    motions = root.findall("./casedef/motion") + root.findall("./execution/motion")
    if len(motions) != 2:
        raise StagingMaterializerError("rendered case motion cardinality differs")
    for motion in motions:
        if motion.attrib or len(motion) != 1 or motion[0].tag != "objreal" or motion[0].attrib != {"ref": "0"}:
            raise StagingMaterializerError("rendered motion object differs")
        children = list(motion[0])
        if [child.tag for child in children] != ["begin", "mvpathfile", "mvnull"]:
            raise StagingMaterializerError("rendered motion sequence differs")
        begin, path, final = children
        if begin.attrib.get("mov") != "1" or abs(float(begin.attrib.get("start", "nan")) - settled_time_s) > FLOAT_TOLERANCE:
            raise StagingMaterializerError("rendered motion begin differs")
        expected_path = {"id": "1", "duration": _number(duration_s), "next": "2", "anglesunits": "degrees"}
        if path.attrib != expected_path or final.attrib != {"id": "2"}:
            raise StagingMaterializerError("rendered mvpathfile/mvnull differs")
        path_children = list(path)
        if [child.tag for child in path_children] != ["file", "center", "movecenter", "intrinsic", "axes"]:
            raise StagingMaterializerError("rendered mvpathfile child sequence differs")
        expected_file = {
            "name": "solver_path.csv", "fields": "7", "fieldtime": "0", "fieldx": "1",
            "fieldy": "2", "fieldz": "3", "fieldang1": "4", "fieldang2": "5", "fieldang3": "6",
        }
        if (
            path_children[0].attrib != expected_file
            or path_children[1].attrib != {"x": "0", "y": "0", "z": "0"}
            or path_children[2].attrib != {"value": "true"}
            or path_children[3].attrib != {"value": "true"}
            or path_children[4].attrib != {"value": "ZYX"}
        ):
            raise StagingMaterializerError("rendered seven-field/ZYX/intrinsic/movecenter contract differs")


def _open_parent(root_fd: int, relative: str) -> tuple[int, str]:
    parts = PurePosixPath(relative).parts
    descriptor = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor, parts[-1]
    except BaseException:
        os.close(descriptor)
        raise


def _write_at(root_fd: int, relative: str, payload: bytes, mode: int) -> dict[str, Any]:
    parent_fd, name = _open_parent(root_fd, relative)
    try:
        descriptor = os.open(
            name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            mode,
            dir_fd=parent_fd,
        )
    finally:
        os.close(parent_fd)
    try:
        view = memoryview(payload)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise StagingMaterializerError(f"short staged write: {relative}")
            view = view[written:]
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise StagingMaterializerError(f"unsafe staged output: {relative}")
        return _identity(relative, metadata, hashlib.sha256(payload).hexdigest())
    finally:
        os.close(descriptor)


def _stage_inventory(stage_root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    directories: set[str] = set()
    for base, subdirs, names in os.walk(stage_root, topdown=True, followlinks=False):
        base_path = Path(base)
        for name in subdirs:
            path = base_path / name
            metadata = os.lstat(path)
            if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
                raise StagingMaterializerError(f"unsafe staged directory: {path}")
            directories.add(str(path.relative_to(stage_root)))
        for name in names:
            path = base_path / name
            metadata = os.lstat(path)
            if stat.S_ISLNK(metadata.st_mode):
                raise StagingMaterializerError(f"symlink staged output: {path}")
            if not stat.S_ISREG(metadata.st_mode):
                raise StagingMaterializerError(f"special staged output: {path}")
            if metadata.st_nlink != 1:
                raise StagingMaterializerError(f"hard-linked staged output: {path}")
            files.add(str(path.relative_to(stage_root)))
    return files, directories


def _verify_staged(stage_root: Path, staged: Mapping[str, Mapping[str, Any]]) -> None:
    files, directories = _stage_inventory(stage_root)
    if files != {value[0] for value in DESTINATIONS.values()} or directories != {"runtime", "case", "restart"}:
        raise StagingMaterializerError("staged inventory has missing or extra entries")
    for role in ROLES:
        relative, _mode = DESTINATIONS[role]
        observed = observe_source(stage_root / relative)
        observed["path"] = relative
        if observed != dict(staged[role]):
            raise StagingMaterializerError(f"staged output TOCTOU/hash differs: {role}")


def materialize(
    stage_root: Path,
    *,
    expected_stage_root: Path,
    sources: Mapping[str, Mapping[str, Any]],
    restart_part_index: int,
    settled_time_s: float,
    solver_tail_s: float,
) -> dict[str, Any]:
    """Materialize one fresh exact stage and return its deterministic manifest."""

    stage_root = _exact_absolute(Path(stage_root), "stage_root")
    expected_stage_root = _exact_absolute(Path(expected_stage_root), "expected_stage_root")
    if stage_root != expected_stage_root or not stage_root.name.endswith(".partial"):
        raise StagingMaterializerError("stage root differs from exact .partial root")
    if isinstance(restart_part_index, bool) or not isinstance(restart_part_index, int) or restart_part_index < 0:
        raise StagingMaterializerError("restart Part index is invalid")
    settled = _finite(settled_time_s, "settled time")
    tail = _finite(solver_tail_s, "solver tail")
    if settled < 0 or tail <= 0:
        raise StagingMaterializerError("settled time/tail is invalid")
    _assert_no_symlink_components(stage_root.parent)
    if os.path.lexists(stage_root):
        raise StagingMaterializerError("stage root is not fresh")

    payloads, descriptors, source_metadata = _open_exact_sources(sources)
    try:
        expected_part_name = f"Part_{restart_part_index:04d}.bi4"
        if Path(str(sources["restart_part"]["path"])).name != expected_part_name:
            raise StagingMaterializerError("restart Part source filename differs")
        if Path(str(sources["restart_head"]["path"])).name != "Part_Head.ibi4":
            raise StagingMaterializerError("restart Head source filename differs")
        if Path(str(sources["solver_path"]["path"])).name != "solver_path.csv":
            raise StagingMaterializerError("solver path source filename differs")
        rows = _solver_rows(payloads["solver_path"], tail)
        duration = rows[-1].t_s
        rendered_case = render_case(payloads["case_xml"], settled_time_s=settled, duration_s=duration)

        os.mkdir(stage_root, 0o700)
        root_fd = os.open(stage_root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            for directory in ("runtime", "case", "restart"):
                os.mkdir(directory, 0o700, dir_fd=root_fd)
            output_payloads = dict(payloads)
            output_payloads["case_xml"] = rendered_case
            staged: dict[str, dict[str, Any]] = {}
            for role in ROLES:
                relative, mode = DESTINATIONS[role]
                staged[role] = _write_at(root_fd, relative, output_payloads[role], mode)
            os.fsync(root_fd)
        finally:
            os.close(root_fd)

        _postverify_sources(sources, descriptors, source_metadata)
        _verify_staged(stage_root, staged)
        contract = {
            "exact_stage_root": str(expected_stage_root),
            "restart_part_index": restart_part_index,
            "settled_time_s": settled,
            "solver_path_row_count": len(rows),
            "solver_path_last_t_s": duration,
            "solver_tail_s": tail,
            "tmax_s": settled + duration,
            "motion_block_count": 2,
            "motion_element": "mvpathfile",
            "field_indices": [0, 1, 2, 3, 4, 5, 6],
            "anglesunits": "degrees",
            "axes": "ZYX",
            "intrinsic": True,
            "movecenter": True,
            "t0_identity": True,
        }
        semantic = {
            "contract": {key: value for key, value in contract.items() if key != "exact_stage_root"},
            "sources": {role: {"sha256": sources[role]["sha256"], "size_bytes": sources[role]["size_bytes"]} for role in ROLES},
            "staged": {role: {key: staged[role][key] for key in ("path", "mode", "size_bytes", "sha256")} for role in ROLES},
        }
        manifest = {
            "schema_version": "smpcc-r8-liquid-s5b0-staging-case-manifest-v3",
            "document_type": "SMPCC_R8_LIQUID_S5B0_STAGING_CASE_MANIFEST_V3",
            "status": "PASS_S5B0_STAGING_CASE_MATERIALIZED_V3",
            "stage_root": str(stage_root),
            "contract": contract,
            "sources": {role: dict(sources[role]) for role in ROLES},
            "staged": staged,
            "integrity": {
                "external_identities_exact": True,
                "fresh_root": True,
                "o_excl_writes": True,
                "inventory_exact": True,
                "no_symlinks": True,
                "no_hardlinks": True,
                "no_special_files": True,
                "no_toctou": True,
                "two_motion_blocks_exact": True,
                "solver_path_exact_copy": staged["solver_path"]["sha256"] == sources["solver_path"]["sha256"],
                "semantic_manifest_sha256": canonical_sha256(semantic),
            },
            "claims": {
                "source_candidate_executed": False,
                "staged_candidate_executed": False,
                "solver_executed": False,
                "gpu_exposed": False,
                "network_used": False,
                "sudo_used": False,
                "apparmor_loaded": False,
                "real_bag_read": False,
                "real_solver_output_read": False,
            },
        }
        schema = json.loads(SCHEMA_PATH.read_bytes())
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
        Draft202012Validator(schema).validate(manifest)
        return manifest
    finally:
        for descriptor in descriptors.values():
            os.close(descriptor)


def _fixture_case_xml() -> bytes:
    block = b"""    <motion>\n      <objreal ref=\"0\"><begin mov=\"1\" start=\"0\"/><mvnull id=\"1\"/></objreal>\n    </motion>"""
    return b"<?xml version=\"1.0\"?>\n<case>\n  <casedef>\n" + block + b"\n  </casedef>\n  <execution>\n" + block + b"\n  </execution>\n</case>\n"


def _write_fixture_sources(root: Path) -> dict[str, dict[str, Any]]:
    root.mkdir(mode=0o700)
    solver_path = bridge.render_solver_path_csv((
        bridge.SolverPathRow(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        bridge.SolverPathRow(1.0, 0.1, 0.0, 0.0, 5.0, 0.0, 0.0),
        bridge.SolverPathRow(2.0, 0.1, 0.0, 0.0, 5.0, 0.0, 0.0),
    )).encode()
    paths = {
        "candidate": root / "DualSPHysics5.4_linux64",
        "dsph_config": root / "DsphConfig.xml",
        "case_xml": root / "C1M_zero.xml",
        "case_bi4": root / "C1M_zero.bi4",
        "restart_part": root / "Part_0901.bi4",
        "restart_head": root / "Part_Head.ibi4",
        "solver_path": root / "solver_path.csv",
    }
    payloads = {
        "candidate": b"fixture disarmed candidate",
        "dsph_config": b"<dsphconfig/>\n",
        "case_xml": _fixture_case_xml(),
        "case_bi4": b"fixture C1M BI4",
        "restart_part": b"fixture settled Part",
        "restart_head": b"fixture Part Head",
        "solver_path": solver_path,
    }
    for role, path in paths.items():
        path.write_bytes(payloads[role])
        path.chmod(0o400 if role == "candidate" else 0o440)
    return {role: observe_source(path) for role, path in paths.items()}


def self_check() -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="r8-s5b0-staging-v3-fixture-") as temporary:
        base = Path(temporary)
        sources = _write_fixture_sources(base / "sources")
        stage = base / "stage.partial"
        manifest = materialize(
            stage,
            expected_stage_root=stage,
            sources=sources,
            restart_part_index=901,
            settled_time_s=45.05001991890928,
            solver_tail_s=1.0,
        )
        semantic_hash = manifest["integrity"]["semantic_manifest_sha256"]
    return {
        "status": "PASS_S5B0_STAGING_CASE_MATERIALIZER_V3_FIXTURE_SELF_CHECK",
        "manifest_status": "PASS_S5B0_STAGING_CASE_MATERIALIZED_V3",
        "semantic_manifest_sha256": semantic_hash,
        "fixture_root_removed": not stage.exists(),
        "real_external_input_read": False,
        "real_bag_read": False,
        "source_candidate_executed": False,
        "staged_candidate_executed": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
        "sudo_used": False,
        "apparmor_loaded": False,
        "files_written_outside_fixture": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_STAGING_CASE_MATERIALIZER_V3", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
