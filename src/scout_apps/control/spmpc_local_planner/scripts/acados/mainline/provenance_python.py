"""Lazy Python, CasADi, and NumPy runtime identity for D4 generation."""

from __future__ import annotations

import importlib
import os
import platform
import stat
import sys
from dataclasses import InitVar, dataclass
from pathlib import Path
from typing import Any

from .identity import sha256_json
from .provenance_common import (
    ProvenanceError,
    require_absolute_text,
    require_digest,
    require_no_symlink_directory_components,
    require_text,
)
from .provenance_files import (
    LinkedFileIdentity,
    ToolIdentity,
    capture_linked_file,
    require_tool_identity,
)

PYTHON_IDENTITY_SCHEMA = "spmpc_mainline_python_runtime_identity_v1"

_TOKEN = object()


@dataclass(frozen=True)
class PythonPackageIdentity:
    name: str
    version: str
    files: tuple[LinkedFileIdentity, ...]
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("PythonPackageIdentity requires provenance capture")
        _validate_python_package(self)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "files": [item.to_dict() for item in self.files],
        }


def _validate_python_package(value: PythonPackageIdentity) -> None:
    require_text(value.name, "Python package name")
    require_text(value.version, "Python package version")
    if type(value.files) is not tuple or not value.files:
        raise ProvenanceError("Python package files must be a non-empty tuple")
    logical_names: list[str] = []
    for item in value.files:
        if type(item) is not LinkedFileIdentity:
            raise ProvenanceError("Python package file has the wrong type")
        logical_names.append(item.logical_name)
    if len(logical_names) != len(set(logical_names)):
        raise ProvenanceError("Python package file logical names must be unique")


@dataclass(frozen=True)
class PythonRuntimeIdentity:
    implementation: str
    version: str
    version_info: tuple[int, int, int]
    executable_tool_sha256: str
    sys_prefix: str
    sys_base_prefix: str
    sys_path: tuple[str, ...]
    pythonpath: str | None
    packages: tuple[PythonPackageIdentity, ...]
    semantic_sha256: str
    _construction_token: InitVar[object] = None
    schema_version: str = PYTHON_IDENTITY_SCHEMA

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _TOKEN:
            raise ProvenanceError("PythonRuntimeIdentity requires provenance capture")
        _validate_python_runtime(self)

    def to_dict(self) -> dict[str, Any]:
        payload = _python_payload(self)
        payload["semantic_identity"] = {"sha256": self.semantic_sha256}
        return payload


def _python_payload(value: PythonRuntimeIdentity) -> dict[str, Any]:
    return {
        "schema_version": value.schema_version,
        "implementation": value.implementation,
        "version": value.version,
        "version_info": list(value.version_info),
        "executable_tool_sha256": value.executable_tool_sha256,
        "sys_prefix": value.sys_prefix,
        "sys_base_prefix": value.sys_base_prefix,
        "sys_path": list(value.sys_path),
        "PYTHONPATH": value.pythonpath,
        "packages": [item.to_dict() for item in value.packages],
    }


def _validate_python_runtime(value: PythonRuntimeIdentity) -> None:
    if value.schema_version != PYTHON_IDENTITY_SCHEMA:
        raise ProvenanceError("Python runtime schema drifted")
    require_text(value.implementation, "Python implementation")
    require_text(value.version, "Python version")
    if (
        type(value.version_info) is not tuple
        or len(value.version_info) != 3
        or any(type(item) is not int or item < 0 for item in value.version_info)
    ):
        raise ProvenanceError("Python version_info must contain three integers")
    require_digest(value.executable_tool_sha256, "Python executable tool identity")
    require_absolute_text(value.sys_prefix, "Python sys.prefix")
    require_absolute_text(value.sys_base_prefix, "Python sys.base_prefix")
    if type(value.sys_path) is not tuple or not value.sys_path:
        raise ProvenanceError("Python sys.path must be a non-empty tuple")
    for item in value.sys_path:
        require_absolute_text(item, "Python sys.path entry")
    if value.pythonpath is not None:
        require_text(value.pythonpath, "PYTHONPATH", nonempty=False)
    if type(value.packages) is not tuple or tuple(
        item.name for item in value.packages
    ) != ("casadi", "numpy"):
        raise ProvenanceError("Python package identities must be CasADi then NumPy")
    for item in value.packages:
        _validate_python_package(item)
    require_digest(value.semantic_sha256, "Python runtime semantic identity")


def require_python_runtime(value: Any) -> PythonRuntimeIdentity:
    if type(value) is not PythonRuntimeIdentity:
        raise ProvenanceError(
            "Python runtime must have the exact PythonRuntimeIdentity type"
        )
    _validate_python_runtime(value)
    if sha256_json(_python_payload(value)) != value.semantic_sha256:
        raise ProvenanceError("Python runtime identity is inconsistent")
    return value


def module_file(module: Any, label: str) -> Path:
    value = getattr(module, "__file__", None)
    if type(value) is not str:
        raise ProvenanceError(f"{label} module path is unavailable")
    candidate = Path(value)
    if not candidate.is_absolute() or ".." in candidate.parts:
        raise ProvenanceError(f"{label} module path must be absolute without '..'")
    candidate = Path(os.path.normpath(str(candidate)))
    require_no_symlink_directory_components(candidate, f"{label} module path")
    try:
        metadata = candidate.stat()
    except OSError as exc:
        raise ProvenanceError(f"cannot resolve {label} module path: {exc}") from exc
    if not stat.S_ISREG(metadata.st_mode):
        raise ProvenanceError(f"{label} module path is not a regular file")
    # Return the importer's spelling, including a final symlink.  The caller
    # passes it to capture_linked_file, which is the sole API that records and
    # permits an explicit leaf symlink chain.
    return candidate


def capture_python_runtime(python_tool: ToolIdentity) -> PythonRuntimeIdentity:
    checked_tool = require_tool_identity(python_tool)
    if checked_tool.role != "python":
        raise ProvenanceError("Python runtime requires the captured Python tool")
    try:
        casadi = importlib.import_module("casadi")
        numpy = importlib.import_module("numpy")
    except (ImportError, OSError) as exc:
        raise ProvenanceError("CasADi and NumPy are required for provenance") from exc
    casadi_init = module_file(casadi, "CasADi")
    casadi_root = casadi_init.resolve(strict=True).parent
    casadi_files = (
        capture_linked_file("casadi/__init__.py", casadi_init),
        capture_linked_file("casadi/_casadi.so", casadi_root / "_casadi.so"),
        capture_linked_file("casadi/libcasadi.so", casadi_root / "libcasadi.so"),
    )
    numpy_init = module_file(numpy, "NumPy")
    numpy_root = numpy_init.resolve(strict=True).parent
    numpy_extensions = tuple(sorted(numpy_root.glob("core/_multiarray_umath*.so")))
    if len(numpy_extensions) != 1:
        raise ProvenanceError("NumPy multiarray extension identity is ambiguous")
    numpy_files = (
        capture_linked_file("numpy/__init__.py", numpy_init),
        capture_linked_file("numpy/core/_multiarray_umath.so", numpy_extensions[0]),
    )
    packages = (
        PythonPackageIdentity(
            "casadi",
            require_text(getattr(casadi, "__version__", None), "CasADi version"),
            casadi_files,
            _construction_token=_TOKEN,
        ),
        PythonPackageIdentity(
            "numpy",
            require_text(getattr(numpy, "__version__", None), "NumPy version"),
            numpy_files,
            _construction_token=_TOKEN,
        ),
    )
    normalized_sys_path = tuple(
        str(Path(item or os.getcwd()).resolve(strict=False)) for item in sys.path
    )
    values = {
        "implementation": platform.python_implementation(),
        "version": platform.python_version(),
        "version_info": tuple(sys.version_info[:3]),
        "executable_tool_sha256": checked_tool.semantic_sha256,
        "sys_prefix": str(Path(sys.prefix).resolve(strict=True)),
        "sys_base_prefix": str(Path(sys.base_prefix).resolve(strict=True)),
        "sys_path": normalized_sys_path,
        "pythonpath": os.environ.get("PYTHONPATH"),
        "packages": packages,
    }
    provisional = PythonRuntimeIdentity(
        **values,
        semantic_sha256="0" * 64,
        _construction_token=_TOKEN,
    )
    return PythonRuntimeIdentity(
        **values,
        semantic_sha256=sha256_json(_python_payload(provisional)),
        _construction_token=_TOKEN,
    )


__all__ = [
    "PythonPackageIdentity",
    "PythonRuntimeIdentity",
    "capture_python_runtime",
    "module_file",
    "require_python_runtime",
]
