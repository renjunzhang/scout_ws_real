#!/usr/bin/env python3
"""Fail-closed R8 source-separation, freeze and GO-evidence gate.

R7 is historical shared-target evidence.  It remains readable, but it is not
an execution identity for the source-level simulation fork.  This module is
the deliberately small R8 boundary that proves the controller which would be
run is made from *this* package:

* every C++/header/message/config/launch/runtime-model input is enumerated;
* every copied ACADOS code-generation artifact is enumerated;
* the simulation node and library are hash-bound at their sim-owned build
  paths; and
* the freeze, master and independently generated GO receipt cross-bind the
  same artifact registry.

The registry is re-derived from the live files during validation.  Therefore a
post-freeze source edit, a regenerated solver, a different library, or a path
under the real controller/experiment packages turns the R8 gate into NO-GO.
This is intentionally an execution gate only: it does not manufacture a
formal freeze, timing evidence, liquid truth, or a matrix result.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence


SOURCE_SEPARATED_RELEASE_ID = "SIM-MECHANISM-40-64-88-R8"
SOURCE_SEPARATED_PROTOCOL_ID = "SMPCC-SIM-MECHANISM-40-64-88-v4"
SOURCE_SEPARATED_TARGET_ID = "SMPCC_SIM_LOCAL_PLANNER_TARGET_R8"
SOURCE_SEPARATION_DOCUMENT_TYPE = "SMPCC_SIM_SOURCE_SEPARATION_R8"
SOURCE_SEPARATION_SCHEMA_VERSION = "smpcc-sim-source-separation-r8-v2"
EXECUTION_ARTIFACT_DOCUMENT_TYPE = "SMPCC_SIM_R8_EXECUTION_ARTIFACT_REGISTRY"
EXECUTION_ARTIFACT_SCHEMA_VERSION = "smpcc-sim-r8-execution-artifacts-v1"
R8_GO_DOCUMENT_TYPE = "SMPCC_SIM_R8_SOURCE_SEPARATION_GO_RECEIPT"
R8_GO_SCHEMA_VERSION = "smpcc-sim-r8-source-separation-go-v1"

SIM_PACKAGE = "spmpc_sim_local_planner"
SIM_NODE = "/sim_spmpc_local_planner"
SIM_DIAGNOSTIC_ROOT = "/sim_spmpc"
FORMAL_SCOPE = "SIMULATION_MECHANISM_ONLY"

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_ROOT.parents[3]
SIM_ROOT = Path("/data/a/scout_sim_replacement")
# The simulation executable has its own build/devel prefix under SIM_ROOT.
# Never fall back to scout_ws/devel: that prefix is shared with real-robot
# packages and a simulation build must not alter or select their artifacts.
SIM_BUILD_WORKSPACE = Path(
    os.environ.get("SMPCC_SIM_BUILD_WORKSPACE", str(SIM_ROOT / "r8_controller_ws"))
)
SIM_DEVEL_PREFIX = SIM_BUILD_WORKSPACE / "devel"
SIM_BUILD_SCRIPT = PACKAGE_ROOT / "scripts/build_sim_controller_workspace.sh"
SIM_FORK_PROVENANCE = PACKAGE_ROOT / "SIM_FORK_ORIGIN.md"
HISTORICAL_QUARANTINE_ROOT = PACKAGE_ROOT / "historical_quarantine"
SIM_NODE_BINARY = SIM_DEVEL_PREFIX / "lib/spmpc_sim_local_planner/spmpc_sim_local_planner_node"
SIM_LIBRARY_BINARY = SIM_DEVEL_PREFIX / "lib/libspmpc_sim_local_planner.so"

# Exact path-component matching deliberately permits the package name
# ``spmpc_sim_local_planner`` while rejecting a real-controller component named
# ``spmpc_local_planner``.  Do not use a substring check here.
FORBIDDEN_PATH_COMPONENTS = frozenset(
    ("spmpc_local_planner", "spmpc_experiments", "slosh_models")
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
WRITE_BITS = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH

# These are all *active* source/model/codegen-bearing subtrees that can affect
# an R8 controller.  Runtime scripts include the source-separation/GO code
# itself; test scripts are intentionally included too so that an admitted GO
# receipt cannot silently be detached from the source it exercised.  The
# package-local R7/R1 historical_quarantine is deliberately outside this
# registry: it is read-only historical evidence, never an R8 implementation
# input or executable surface.
PACKAGE_TREE_ROOTS = {
    "build_metadata": (PACKAGE_ROOT, ("CMakeLists.txt", "package.xml")),
    "controller_source": (PACKAGE_ROOT / "src", None),
    "controller_headers": (PACKAGE_ROOT / "include", None),
    "controller_messages": (PACKAGE_ROOT / "msg", None),
    "model_and_codegen_inputs": (PACKAGE_ROOT / "scripts/acados", None),
    "generated_solver_codegen": (PACKAGE_ROOT / "generated/acados", None),
    "runtime_configuration": (PACKAGE_ROOT / "config", None),
    "runtime_launch": (PACKAGE_ROOT / "launch", None),
    "runtime_tooling_and_tests": (PACKAGE_ROOT / "scripts", None),
    "package_tests": (PACKAGE_ROOT / "tests", None),
}

R8_GO_CHECKS = {
    "build": {
        "command": (str(SIM_BUILD_SCRIPT),),
        "source": SIM_BUILD_SCRIPT,
        "test_count": 0,
    },
    "source_isolation": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_source_isolation.py")),
        "source": PACKAGE_ROOT / "tests/test_source_isolation.py",
        "test_count": 7,
    },
    "controller_gate": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_smpcc_sim_controller_gate.py")),
        "source": PACKAGE_ROOT / "tests/test_smpcc_sim_controller_gate.py",
        "test_count": 8,
    },
    # catkin's gtest transcript uses a different grammar from unittest.  Its
    # target verifies the XML result and exits nonzero on any failed case, so
    # retain test_count=0 here while binding the exact C++ test source.
    "node_self_admission": {
        "command": (str(SIM_BUILD_SCRIPT), "--node-admission-test"),
        "source": PACKAGE_ROOT / "test/test_sim_node_admission.cpp",
        "test_count": 0,
    },
    "environment_isolation": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_smpcc_sim_environment_launch.py")),
        "source": PACKAGE_ROOT / "tests/test_smpcc_sim_environment_launch.py",
        "test_count": 5,
    },
    "legacy_execution_quarantine": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_smpcc_sim_legacy_quarantine.py")),
        "source": PACKAGE_ROOT / "tests/test_smpcc_sim_legacy_quarantine.py",
        "test_count": 7,
    },
    "h_proxy_monitor": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_smpcc_sim_h_proxy_monitor.py")),
        "source": PACKAGE_ROOT / "tests/test_smpcc_sim_h_proxy_monitor.py",
        "test_count": 3,
    },
    "r8_source_separation": {
        "command": (sys.executable, str(PACKAGE_ROOT / "tests/test_smpcc_sim_source_separation.py")),
        "source": PACKAGE_ROOT / "tests/test_smpcc_sim_source_separation.py",
        "test_count": 5,
    },
    "r8_release_gate": {
        "command": (sys.executable, str(PACKAGE_ROOT / "scripts/tests/test_smpcc_sim_r8_release.py")),
        "source": PACKAGE_ROOT / "scripts/tests/test_smpcc_sim_r8_release.py",
        "test_count": 4,
    },
}


class SourceSeparationError(RuntimeError):
    """A missing or divergent R8 identity must prevent execution."""


def canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fail(message: str) -> None:
    raise SourceSeparationError("SOURCE_SEPARATED_R8_REQUIRED: " + message)


def _require(value: bool, message: str) -> None:
    if not value:
        _fail(message)


def _is_sha256(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _contains_forbidden_component(path: Path) -> bool:
    return any(component in FORBIDDEN_PATH_COMPONENTS for component in path.parts)


def _has_symlink_component(path: Path) -> bool:
    """Reject a symlink at ``path`` or along any existing parent.

    Hashes follow symlinks by default.  That is unacceptable for a source
    isolation receipt because a later retarget could change the executable
    without changing the apparent manifest location.
    """

    lexical = Path(os.path.abspath(os.fspath(path)))
    for component in (lexical, *lexical.parents):
        if component.is_symlink():
            return True
    return False


def _is_below(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def _require_package_file(path: Path, label: str) -> Path:
    lexical = Path(os.path.abspath(os.fspath(path)))
    _require(lexical.is_absolute(), f"{label} path must be absolute")
    _require(not _has_symlink_component(lexical), f"{label} may not traverse a symlink")
    _require(not _contains_forbidden_component(lexical), f"{label} resolves into a forbidden real-controller/experiment path")
    _require(lexical.is_file(), f"{label} is missing: {lexical}")
    resolved = lexical.resolve()
    _require(_is_below(resolved, PACKAGE_ROOT), f"{label} must be owned by {SIM_PACKAGE}: {resolved}")
    return resolved


def _require_sim_binary(path: Path, expected: Path, label: str) -> Path:
    lexical = Path(os.path.abspath(os.fspath(path)))
    _require(lexical == expected, f"{label} path must be the sim-owned build artifact {expected}")
    _require(not _has_symlink_component(lexical), f"{label} may not traverse a symlink")
    _require(not _contains_forbidden_component(lexical), f"{label} resolves into a forbidden real-controller/experiment path")
    _require(lexical.is_file(), f"{label} is missing; rebuild {SIM_PACKAGE}: {lexical}")
    return lexical.resolve()


def _require_external_sim_build_prefix() -> Path:
    """Reject a shared workspace or symlinked build prefix before hashing it."""

    lexical = Path(os.path.abspath(os.fspath(SIM_BUILD_WORKSPACE)))
    _require(lexical.is_absolute(), "simulation build workspace path must be absolute")
    _require(
        lexical == SIM_ROOT / "r8_controller_ws",
        f"simulation build workspace must be {SIM_ROOT / 'r8_controller_ws'}",
    )
    _require(not _has_symlink_component(lexical), "simulation build workspace may not traverse a symlink")
    _require(lexical.is_dir(), f"simulation build workspace is missing: {lexical}")
    _require(
        not _is_below(lexical, WORKSPACE),
        "simulation build workspace may not be inside the real-robot workspace",
    )
    _require(
        (lexical / "devel" / "setup.bash").is_file(),
        "simulation build workspace is missing its isolated devel setup",
    )
    return lexical


def _descriptor(path: Path, label: str, *, package_owned: bool = True) -> Dict[str, str]:
    resolved = _require_package_file(path, label) if package_owned else path
    return {"path": str(resolved), "sha256": sha256_file(resolved)}


def _include_tree_file(path: Path) -> bool:
    """Keep only durable source/codegen material in a tree digest."""

    if not path.is_file() or "__pycache__" in path.parts:
        return False
    try:
        path.resolve().relative_to(HISTORICAL_QUARANTINE_ROOT.resolve())
    except ValueError:
        pass
    else:
        return False
    if path.suffix == ".pyc" or path.name.endswith((".bag", ".active", "~")):
        return False
    return True


def _tree_descriptor(name: str, root: Path, exact_files: Optional[Sequence[str]]) -> Dict[str, Any]:
    _require(root.is_dir(), f"R8 artifact tree {name} is missing: {root}")
    _require(not _has_symlink_component(root), f"R8 artifact tree {name} may not traverse a symlink")
    _require(_is_below(root, PACKAGE_ROOT), f"R8 artifact tree {name} is not package-owned")
    _require(not _contains_forbidden_component(root), f"R8 artifact tree {name} is under a forbidden path")
    if exact_files is None:
        paths = [path for path in sorted(root.rglob("*")) if _include_tree_file(path)]
    else:
        paths = [root / item for item in exact_files]
    files: Dict[str, str] = {}
    for path in paths:
        resolved = _require_package_file(path, f"R8 artifact {name}")
        relative = resolved.relative_to(PACKAGE_ROOT.resolve()).as_posix()
        _require(relative not in files, f"R8 artifact tree {name} repeats {relative}")
        files[relative] = sha256_file(resolved)
    _require(files, f"R8 artifact tree {name} is empty")
    core: Dict[str, Any] = {
        "root": str(root.resolve()),
        "files": files,
    }
    return dict(core, tree_hash=canonical_hash(core))


def build_execution_artifact_registry() -> Dict[str, Any]:
    """Derive the complete live R8 source/binary/model/codegen registry.

    This is intentionally a pure read operation.  It does not build, copy,
    generate, or modify anything.  If a binary is absent the caller gets a
    deterministic NO-GO rather than a fallback to a real-controller binary.
    """

    build_workspace = _require_external_sim_build_prefix()
    provenance = _descriptor(SIM_FORK_PROVENANCE, "R8 fork provenance")
    trees = {
        name: _tree_descriptor(name, root, exact_files)
        for name, (root, exact_files) in PACKAGE_TREE_ROOTS.items()
    }
    node = _require_sim_binary(SIM_NODE_BINARY, SIM_NODE_BINARY, "R8 controller node binary")
    library = _require_sim_binary(SIM_LIBRARY_BINARY, SIM_LIBRARY_BINARY, "R8 controller library binary")
    core: Dict[str, Any] = {
        "schema_version": EXECUTION_ARTIFACT_SCHEMA_VERSION,
        "document_type": EXECUTION_ARTIFACT_DOCUMENT_TYPE,
        "release_id": SOURCE_SEPARATED_RELEASE_ID,
        "target_id": SOURCE_SEPARATED_TARGET_ID,
        "controller_package": SIM_PACKAGE,
        "controller_node": SIM_NODE,
        "package_root": str(PACKAGE_ROOT.resolve()),
        "sim_build_workspace": str(build_workspace),
        "sim_devel_prefix": str((build_workspace / "devel").resolve()),
        "fork_provenance": provenance,
        "trees": trees,
        "binaries": {
            "controller_node": {"path": str(node), "sha256": sha256_file(node)},
            "controller_library": {"path": str(library), "sha256": sha256_file(library)},
        },
    }
    return dict(core, execution_artifact_registry_hash=canonical_hash(core))


def validate_execution_artifact_registry(value: Any) -> Dict[str, Any]:
    """Require an artifact registry to equal the live sim-owned registry."""

    _require(isinstance(value, Mapping), "source_separation artifact registry is missing")
    expected = build_execution_artifact_registry()
    expected_keys = set(expected)
    _require(
        set(value) == expected_keys,
        "source_separation artifact registry fields are not the exact R8 schema",
    )
    declared = value.get("execution_artifact_registry_hash")
    core = dict(value)
    core.pop("execution_artifact_registry_hash", None)
    _require(
        declared == canonical_hash(core),
        "source_separation artifact registry internal hash mismatch",
    )
    _require(
        dict(value) == expected,
        "source/binary/model/codegen registry differs from the live sim-owned package artifacts",
    )
    return expected


def make_source_separation_binding() -> Dict[str, Any]:
    """Create the exact R8 binding that a freeze must embed before review."""

    registry = build_execution_artifact_registry()
    return {
        "schema_version": SOURCE_SEPARATION_SCHEMA_VERSION,
        "document_type": SOURCE_SEPARATION_DOCUMENT_TYPE,
        "target_id": SOURCE_SEPARATED_TARGET_ID,
        "controller_package": SIM_PACKAGE,
        "controller_node": SIM_NODE,
        "diagnostic_root": SIM_DIAGNOSTIC_ROOT,
        "real_controller_package_dependency": False,
        "source_level_fork": True,
        "fork_provenance": dict(registry["fork_provenance"]),
        "execution_artifact_registry": registry,
        "execution_artifact_registry_hash": registry[
            "execution_artifact_registry_hash"
        ],
    }


def validate_source_separation_binding(value: Any) -> Dict[str, Any]:
    """Validate a source binding before it can enter R8 freeze/master/GO."""

    _require(isinstance(value, Mapping), "formal freeze lacks source_separation binding")
    expected_keys = {
        "schema_version",
        "document_type",
        "target_id",
        "controller_package",
        "controller_node",
        "diagnostic_root",
        "real_controller_package_dependency",
        "source_level_fork",
        "fork_provenance",
        "execution_artifact_registry",
        "execution_artifact_registry_hash",
    }
    _require(
        set(value) == expected_keys,
        "source_separation binding fields are not the exact R8 schema",
    )
    required = {
        "schema_version": SOURCE_SEPARATION_SCHEMA_VERSION,
        "document_type": SOURCE_SEPARATION_DOCUMENT_TYPE,
        "target_id": SOURCE_SEPARATED_TARGET_ID,
        "controller_package": SIM_PACKAGE,
        "controller_node": SIM_NODE,
        "diagnostic_root": SIM_DIAGNOSTIC_ROOT,
        "real_controller_package_dependency": False,
        "source_level_fork": True,
    }
    for key, expected in required.items():
        _require(value.get(key) == expected, f"source_separation.{key} must equal {expected!r}")
    provenance = value.get("fork_provenance")
    expected_provenance = _descriptor(SIM_FORK_PROVENANCE, "R8 fork provenance")
    _require(
        provenance == expected_provenance,
        "source_separation fork provenance is not the live sim-package SIM_FORK_ORIGIN.md",
    )
    registry = validate_execution_artifact_registry(value.get("execution_artifact_registry"))
    _require(
        value.get("execution_artifact_registry_hash")
        == registry["execution_artifact_registry_hash"],
        "source_separation execution artifact registry hash mismatch",
    )
    _require(
        registry.get("fork_provenance") == provenance,
        "source_separation provenance and artifact registry differ",
    )
    return dict(value)


def _bound_readonly_json(value: Any, label: str) -> tuple[Path, Mapping[str, Any]]:
    _require(
        isinstance(value, Mapping) and set(value) == {"path", "sha256"},
        f"{label} must be a path/hash descriptor",
    )
    path = Path(str(value.get("path", "")))
    _require(path.is_absolute(), f"{label} path must be absolute")
    _require(not _has_symlink_component(path), f"{label} may not traverse a symlink")
    _require(path.is_file(), f"{label} file is missing: {path}")
    _require(stat.S_IMODE(path.stat().st_mode) & WRITE_BITS == 0, f"{label} must be read-only")
    expected = value.get("sha256")
    _require(_is_sha256(expected), f"{label} SHA-256 is malformed")
    _require(sha256_file(path) == expected, f"{label} file hash mismatch")
    try:
        with path.open(encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        _fail(f"{label} is not valid JSON: {exc}")
    _require(isinstance(document, Mapping), f"{label} JSON must be an object")
    return path.resolve(), document


def _freeze_semantic_hash(freeze: Mapping[str, Any]) -> str:
    """Return the pre-GO freeze payload hash used by the R8 receipt.

    The receipt descriptor is attached to the freeze after the independent GO
    checks complete, so including that descriptor would create a circular
    hash.  Everything else in the freeze remains bound.  A native R8 freeze
    may expose a matching ``freeze_hash`` for this pre-GO payload; generic
    formal-freeze documents instead use the canonical payload directly.
    """

    core = dict(freeze)
    core.pop("source_separation_go_receipt", None)
    declared = core.pop("freeze_hash", None)
    if _is_sha256(declared) and declared == canonical_hash(core):
        return str(declared)
    # The generic formal freeze schema stores a canonical whole-document hash
    # in its master rather than a `freeze_hash` field.  Canonical content is
    # still an unambiguous binding for the R8 GO receipt.
    return canonical_hash(core)


def _master_semantic_hash(master: Mapping[str, Any]) -> str:
    value = master.get("master_hash")
    _require(_is_sha256(value), "formal master lacks a SHA-256 master_hash")
    return str(value)


def _expected_go_check(name: str) -> Dict[str, Any]:
    item = R8_GO_CHECKS[name]
    source = _descriptor(Path(item["source"]), f"R8 GO check source {name}")
    return {
        "command": list(item["command"]),
        "test_source": source,
        "test_count": item["test_count"],
    }


def _validate_r8_go_checks(value: Any) -> Dict[str, Any]:
    _require(isinstance(value, Mapping), "R8 GO receipt checks are missing")
    _require(set(value) == set(R8_GO_CHECKS), "R8 GO receipt checks are incomplete or unknown")
    normalized: Dict[str, Any] = {}
    for name in sorted(R8_GO_CHECKS):
        item = value.get(name)
        _require(isinstance(item, Mapping), f"R8 GO check {name} is malformed")
        _require(
            set(item) == {"status", "command", "returncode", "test_count", "test_source", "evidence"},
            f"R8 GO check {name} fields are not exact",
        )
        expected = _expected_go_check(name)
        _require(item.get("status") == "PASS" and item.get("returncode") == 0, f"R8 GO check {name} did not PASS")
        _require(item.get("command") == expected["command"], f"R8 GO check {name} command differs from the sim-owned test")
        _require(item.get("test_count") == expected["test_count"], f"R8 GO check {name} test count mismatch")
        _require(item.get("test_source") == expected["test_source"], f"R8 GO check {name} source hash/path mismatch")
        evidence_path, _ = _bound_readonly_json_or_text(item.get("evidence"), f"R8 GO check {name} evidence")
        transcript = evidence_path.read_text(encoding="utf-8")
        _require(f"returncode=0" in transcript, f"R8 GO check {name} evidence does not record returncode=0")
        if expected["test_count"]:
            _require(f"Ran {expected['test_count']} tests" in transcript, f"R8 GO check {name} evidence count transcript mismatch")
            _require("OK" in transcript and "FAILED" not in transcript, f"R8 GO check {name} evidence is not an all-pass unittest transcript")
        else:
            _require("error" not in transcript.lower(), f"R8 GO build evidence contains an error")
        normalized[name] = dict(item)
    return normalized


def _bound_readonly_json_or_text(value: Any, label: str) -> tuple[Path, str]:
    """Verify a read-only evidence file; its caller decides its syntax."""

    _require(
        isinstance(value, Mapping) and set(value) == {"path", "sha256"},
        f"{label} must be a path/hash descriptor",
    )
    path = Path(str(value.get("path", "")))
    _require(path.is_absolute(), f"{label} path must be absolute")
    _require(not _has_symlink_component(path), f"{label} may not traverse a symlink")
    _require(path.is_file(), f"{label} file is missing: {path}")
    _require(stat.S_IMODE(path.stat().st_mode) & WRITE_BITS == 0, f"{label} must be read-only")
    expected = value.get("sha256")
    _require(_is_sha256(expected) and sha256_file(path) == expected, f"{label} file hash mismatch")
    return path.resolve(), str(expected)


def validate_r8_go_receipt(
    value: Any, freeze: Mapping[str, Any], master: Mapping[str, Any]
) -> Dict[str, Any]:
    """Validate immutable R8 GO evidence against freeze/master/live artifacts."""

    _require(isinstance(value, Mapping), "R8 source-separation GO receipt is not an object")
    binding = validate_source_separation_binding(freeze.get("source_separation"))
    binding_hash = canonical_hash(binding)
    registry_hash = str(binding["execution_artifact_registry_hash"])
    expected_keys = {
        "schema_version",
        "document_type",
        "protocol_id",
        "release_id",
        "target_id",
        "status",
        "formal_scope",
        "physical_alignment",
        "physical_primary_eligible",
        "physical_primary",
        "source_separation_hash",
        "execution_artifact_registry_hash",
        "freeze_hash",
        "master_hash",
        "checks",
        "created_utc",
        "go_receipt_hash",
    }
    _require(set(value) == expected_keys, "R8 GO receipt fields are not the exact schema")
    expected_identity = {
        "schema_version": R8_GO_SCHEMA_VERSION,
        "document_type": R8_GO_DOCUMENT_TYPE,
        "protocol_id": SOURCE_SEPARATED_PROTOCOL_ID,
        "release_id": SOURCE_SEPARATED_RELEASE_ID,
        "target_id": SOURCE_SEPARATED_TARGET_ID,
        "status": "PASS",
        "formal_scope": FORMAL_SCOPE,
        "physical_alignment": False,
        "physical_primary_eligible": False,
        "physical_primary": None,
        "source_separation_hash": binding_hash,
        "execution_artifact_registry_hash": registry_hash,
        "freeze_hash": _freeze_semantic_hash(freeze),
        "master_hash": _master_semantic_hash(master),
    }
    for key, expected in expected_identity.items():
        _require(value.get(key) == expected, f"R8 GO receipt {key} mismatch")
    created_utc = value.get("created_utc")
    _require(isinstance(created_utc, str) and created_utc.endswith("Z"), "R8 GO receipt created_utc is invalid")
    _validate_r8_go_checks(value.get("checks"))
    core = dict(value)
    declared = core.pop("go_receipt_hash", None)
    _require(declared == canonical_hash(core), "R8 GO receipt hash mismatch")
    return dict(value)


def _validate_freeze_master_cross_binding(
    freeze: Mapping[str, Any], master: Mapping[str, Any]
) -> Dict[str, Any]:
    _require(
        freeze.get("release_id") == SOURCE_SEPARATED_RELEASE_ID,
        "formal freeze is not the source-separated R8 release",
    )
    _require(
        master.get("release_id") == SOURCE_SEPARATED_RELEASE_ID,
        "formal master is not the source-separated R8 release",
    )
    binding = validate_source_separation_binding(freeze.get("source_separation"))
    binding_hash = canonical_hash(binding)
    _require(
        master.get("source_separation_hash") == binding_hash,
        "formal master does not cross-bind the source-separation object",
    )
    _require(
        master.get("execution_artifact_registry_hash")
        == binding.get("execution_artifact_registry_hash"),
        "formal master does not cross-bind the sim-owned source/binary/model/codegen registry",
    )
    return binding


def require_execution_identity(
    freeze: Mapping[str, Any], master: Mapping[str, Any]
) -> Mapping[str, Any]:
    """Reject any R7/shared source, stale artifact, or incomplete R8 GO proof.

    Preparation/reporting may inspect old artifacts, but execution must bind a
    fresh R8 freeze/master pair to the source-level fork.  A caller cannot
    bypass this by changing a release-id string: the live registry, its
    freeze/master cross-hashes, and a separate immutable GO receipt all have
    to agree.
    """

    binding = _validate_freeze_master_cross_binding(freeze, master)
    receipt_path, receipt = _bound_readonly_json(
        freeze.get("source_separation_go_receipt"),
        "R8 source-separation GO receipt",
    )
    validated = validate_r8_go_receipt(receipt, freeze, master)
    _require(
        master.get("source_separation_go_receipt_hash")
        == validated.get("go_receipt_hash"),
        "formal master does not cross-bind the R8 source-separation GO receipt",
    )
    # Keep the path in the return value so callers can place it in their
    # attempt manifest without reopening a mutable selector.
    output = dict(binding)
    output["source_separation_go_receipt"] = {
        "path": str(receipt_path),
        "sha256": sha256_file(receipt_path),
        "go_receipt_hash": validated["go_receipt_hash"],
    }
    return output


def build_r8_go_receipt(
    freeze: Mapping[str, Any], master: Mapping[str, Any], checks: Mapping[str, Any]
) -> Dict[str, Any]:
    """Build (but do not persist) a receipt from already-frozen check evidence."""

    binding = _validate_freeze_master_cross_binding(freeze, master)
    core: Dict[str, Any] = {
        "schema_version": R8_GO_SCHEMA_VERSION,
        "document_type": R8_GO_DOCUMENT_TYPE,
        "protocol_id": SOURCE_SEPARATED_PROTOCOL_ID,
        "release_id": SOURCE_SEPARATED_RELEASE_ID,
        "target_id": SOURCE_SEPARATED_TARGET_ID,
        "status": "PASS",
        "formal_scope": FORMAL_SCOPE,
        "physical_alignment": False,
        "physical_primary_eligible": False,
        "physical_primary": None,
        "source_separation_hash": canonical_hash(binding),
        "execution_artifact_registry_hash": binding[
            "execution_artifact_registry_hash"
        ],
        "freeze_hash": _freeze_semantic_hash(freeze),
        "master_hash": _master_semantic_hash(master),
        "checks": dict(checks),
        "created_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }
    receipt = dict(core, go_receipt_hash=canonical_hash(core))
    validate_r8_go_receipt(receipt, freeze, master)
    return receipt


def _write_new_readonly(path: Path, payload: str) -> None:
    _require(path.is_absolute(), "R8 GO receipt output must be absolute")
    _require(not _has_symlink_component(path), "R8 GO receipt output may not traverse a symlink")
    _require(not path.exists(), f"refusing to overwrite immutable R8 GO receipt: {path}")
    _require(path.parent.is_dir(), f"R8 GO receipt output parent is missing: {path.parent}")
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(path, 0o444)
    except OSError as exc:
        _fail(f"cannot write R8 GO receipt: {exc}")


def create_r8_go_receipt(
    freeze: Mapping[str, Any], master: Mapping[str, Any], output: Path
) -> Dict[str, Any]:
    """Actually run the sim-owned R8 source admission checks and persist one receipt.

    This helper does not start ROS/Gazebo and does not create a planned row. It
    is intentionally unavailable as an implicit fallback: absent freeze/master
    evidence or an absent rebuilt sim binary raises a fail-closed error.
    """

    _validate_freeze_master_cross_binding(freeze, master)
    evidence_root = output.parent / (output.stem + "_evidence")
    _require(not evidence_root.exists(), f"R8 GO evidence directory already exists: {evidence_root}")
    _require(output.parent.is_dir(), f"R8 GO receipt output parent is missing: {output.parent}")
    environment = dict(os.environ)
    checks: Dict[str, Any] = {}
    try:
        evidence_root.mkdir()
        for name in sorted(R8_GO_CHECKS):
            expected = _expected_go_check(name)
            completed = subprocess.run(
                expected["command"],
                cwd=str(WORKSPACE),
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=300.0,
            )
            transcript = (
                "command=" + json.dumps(expected["command"], ensure_ascii=False) + "\n"
                + f"returncode={completed.returncode}\n"
                + completed.stdout
            )
            evidence_path = evidence_root / f"{name}.log"
            _write_new_readonly(evidence_path, transcript)
            _require(completed.returncode == 0, f"R8 GO check {name} failed")
            if expected["test_count"]:
                _require(
                    f"Ran {expected['test_count']} tests" in completed.stdout
                    and "OK" in completed.stdout
                    and "FAILED" not in completed.stdout,
                    f"R8 GO check {name} did not produce the expected all-pass unittest transcript",
                )
            checks[name] = {
                "status": "PASS",
                "command": expected["command"],
                "returncode": 0,
                "test_count": expected["test_count"],
                "test_source": expected["test_source"],
                "evidence": {"path": str(evidence_path), "sha256": sha256_file(evidence_path)},
            }
        receipt = build_r8_go_receipt(freeze, master, checks)
        _write_new_readonly(
            output,
            json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        )
        return dict(receipt, receipt_path=str(output), receipt_file_hash=sha256_file(output))
    except BaseException:
        # Evidence is deliberately left behind if a test fails: it documents
        # why GO was denied.  No receipt is written after a failing check.
        raise
