"""Validate and canonicalize Acados 0.5.4 code-generation outputs."""

from __future__ import annotations

import hashlib
import json
import os
import stat
import struct
import subprocess
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any

from .acados_codegen_result_schema import (
    ELF_MACHINE_BY_PLATFORM,
    ELF_SHARED_OBJECT_TYPE,
    REQUIRED_SOLVER_SYMBOL_SUFFIXES,
)
from .acados_ocp_contract import AcadosOcpAssembly, require_acados_ocp_assembly
from .acados_solver_options_adapter import (
    ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS,
    ACADOS_SCALAR_SOLVER_OPTION_FIELDS,
)
from .acados_solver_options_identity import (
    AcadosSolverOptionsIdentityError,
    acados_solver_options_baseline_sha256,
)
from .artifact_files import (
    SOLVER_LIBRARY_ROLE,
    GeneratedFileRecord,
    generated_file_records_from_dict,
)
from .codegen_options import (
    ACADOS_JSON_FILENAME,
    CodegenOptionsSnapshot,
    require_codegen_options_snapshot,
)
from .identity import canonical_json, read_strict_json, sha256_bytes
from .model_contract import MODEL_ID
from .provenance_common import ProvenanceError
from .provenance_files import capture_linked_file
from .solver_options import SolverOptionsSnapshot, require_solver_options_snapshot

ACADOS_JSON_VALIDATION_SCHEMA = "spmpc_mainline_acados_json_validation_v1"
GENERATED_ROOT_CANONICAL_VALUE = "."


class AcadosCodegenValidationError(ValueError):
    """Generated JSON, build metadata, or solver library is inconsistent."""


def _object(value: Any, label: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise AcadosCodegenValidationError(f"{label} must be a JSON object")
    return value


def _array(value: Any, label: str) -> list[Any]:
    if type(value) is not list:
        raise AcadosCodegenValidationError(f"{label} must be a JSON array")
    return value


def _same(actual: Any, expected: Any, label: str) -> None:
    if type(actual) is not type(expected) or actual != expected:
        raise AcadosCodegenValidationError(f"generated Acados {label} drifted")


def _read_acados_link_libraries(source_root: Path) -> dict[str, Any]:
    """Read the Acados metadata through one recorded leaf-link identity."""

    requested = source_root / "lib" / "link_libs.json"
    label = "Acados link-library metadata"
    try:
        identity = capture_linked_file(label, requested)
        value, payload = read_strict_json(
            Path(identity.resolved_path),
            label=label,
        )
        current = capture_linked_file(label, requested)
    except (ProvenanceError, ValueError) as exc:
        raise AcadosCodegenValidationError(str(exc)) from exc
    if (
        len(payload) != identity.size_bytes
        or sha256_bytes(payload) != identity.raw_sha256
        or current != identity
    ):
        raise AcadosCodegenValidationError(
            "Acados link-library metadata changed while being read"
        )
    return _object(value, label)


def _finite_zero_array(value: Any, count: int, label: str) -> None:
    items = _array(value, label)
    if len(items) != count or any(
        type(item) not in {int, float} or isinstance(item, bool) or item != 0.0
        for item in items
    ):
        raise AcadosCodegenValidationError(
            f"generated Acados {label} must be a {count}-element zero placeholder"
        )


def _acados_internal_md5(document: dict[str, Any]) -> str:
    """Reproduce Acados' non-security code-reuse hash after path normalization."""

    payload = dict(document)
    payload.pop("hash", None)
    payload.pop("external_function_files_model", None)
    payload.pop("external_function_files_ocp", None)
    payload.pop("json_loaded", None)
    dimensions = dict(_object(payload.get("dims"), "generated Acados dimensions"))
    dimensions.pop("n_global_data", None)
    payload["dims"] = dimensions
    encoded = json.dumps(payload, sort_keys=True).encode("utf-8")
    return hashlib.md5(encoded, usedforsecurity=False).hexdigest()


def _relative_generated_source(value: Any, root: Path, label: str) -> None:
    if type(value) is not str or not value or "\\" in value:
        raise AcadosCodegenValidationError(f"{label} is not a POSIX relative path")
    relative = PurePosixPath(value)
    if (
        relative.is_absolute()
        or relative.as_posix() != value
        or any(part in {"", ".", ".."} for part in relative.parts)
        or relative.suffix != ".c"
    ):
        raise AcadosCodegenValidationError(f"{label} escaped the generated tree")
    source = root.joinpath(*relative.parts)
    if source.is_symlink() or not source.is_file():
        raise AcadosCodegenValidationError(f"{label} is missing or not regular")


def _checked_inputs(
    assembly: Any,
    solver_options: Any,
    codegen_options: Any,
) -> tuple[AcadosOcpAssembly, SolverOptionsSnapshot, CodegenOptionsSnapshot]:
    try:
        checked_assembly = require_acados_ocp_assembly(assembly)
        checked_solver = require_solver_options_snapshot(solver_options)
        checked_codegen = require_codegen_options_snapshot(codegen_options)
    except (TypeError, ValueError) as exc:
        raise AcadosCodegenValidationError(
            "generated-output validation inputs are not canonical"
        ) from exc
    if (
        checked_assembly.solver_options_semantic_sha256
        != checked_solver.semantic_sha256
        or checked_assembly.development_layout_sha256
        != checked_codegen.development_layout_sha256
        or checked_assembly.horizon_steps != checked_codegen.horizon_steps
    ):
        raise AcadosCodegenValidationError(
            "OCP, solver, and codegen option identities are inconsistent"
        )
    return checked_assembly, checked_solver, checked_codegen


def validate_generated_acados_json(
    json_path: Path | str,
    output_root: Path | str,
    acados_source_root: Path | str,
    assembly: AcadosOcpAssembly,
    solver_options: SolverOptionsSnapshot,
    codegen_options: CodegenOptionsSnapshot,
    *,
    canonicalized_output_root: bool,
) -> dict[str, Any]:
    """Strictly validate the generated OCP JSON against the typed authorities."""

    checked_assembly, checked_solver, checked_codegen = _checked_inputs(
        assembly,
        solver_options,
        codegen_options,
    )
    root = Path(output_root)
    path = Path(json_path)
    source_root = Path(acados_source_root)
    if path != root / ACADOS_JSON_FILENAME:
        raise AcadosCodegenValidationError("Acados JSON path is not canonical")
    try:
        document, _ = read_strict_json(path, label="generated Acados OCP JSON")
    except ValueError as exc:
        raise AcadosCodegenValidationError(str(exc)) from exc
    top = _object(document, "generated Acados OCP")
    _same(top.get("name"), MODEL_ID, "top-level model name")
    model = _object(top.get("model"), "generated Acados model")
    _same(model.get("name"), MODEL_ID, "model.name")
    if type(model.get("disc_dyn_expr")) is not str or not model["disc_dyn_expr"]:
        raise AcadosCodegenValidationError(
            "generated Acados discrete dynamics expression is empty"
        )
    _same(model.get("f_expl_expr"), [], "explicit continuous dynamics")
    _same(model.get("f_impl_expr"), [], "implicit continuous dynamics")

    dimensions = _object(top.get("dims"), "generated Acados dimensions")
    expected_dimensions = {
        "N": checked_assembly.horizon_steps,
        "nx": checked_assembly.nx,
        "nx_next": checked_assembly.nx,
        "nu": checked_assembly.nu,
        "np": checked_assembly.np,
        "nh_0": len(checked_assembly.stage_constraint_order),
        "nh": len(checked_assembly.stage_constraint_order),
        "nh_e": len(checked_assembly.terminal_constraint_order),
        "nbu": len(checked_assembly.control_order),
        "nbx_0": checked_assembly.nx,
        "nbxe_0": checked_assembly.nx,
        "nbx": 0,
        "nbx_e": 0,
        "ng": 0,
        "ng_e": 0,
        "np_global": 0,
        "nphi_0": 0,
        "nphi": 0,
        "nphi_e": 0,
        "nr_0": 0,
        "nr": 0,
        "nr_e": 0,
        "ns_0": 0,
        "ns": 0,
        "ns_e": 0,
        "nsbu": 0,
        "nsbx": 0,
        "nsbx_e": 0,
        "nsg": 0,
        "nsg_e": 0,
        "nsh_0": 0,
        "nsh": 0,
        "nsh_e": 0,
        "nsphi_0": 0,
        "nsphi": 0,
        "nsphi_e": 0,
        "ny_0": 0,
        "ny": 0,
        "ny_e": 0,
        "nz": 0,
    }
    for name, expected in expected_dimensions.items():
        _same(dimensions.get(name), expected, f"dims.{name}")

    cost = _object(top.get("cost"), "generated Acados cost")
    constraints = _object(top.get("constraints"), "generated Acados constraints")
    for name in ("cost_type_0", "cost_type", "cost_type_e"):
        _same(cost.get(name), "EXTERNAL", f"cost.{name}")
    for name in ("constr_type_0", "constr_type", "constr_type_e"):
        _same(constraints.get(name), "BGH", f"constraints.{name}")
    _same(constraints.get("has_x0"), True, "constraints.has_x0")
    _same(
        constraints.get("idxbu"),
        list(checked_assembly.control_indices),
        "constraints.idxbu",
    )
    _same(
        constraints.get("idxbx_0"),
        list(range(checked_assembly.nx)),
        "constraints.idxbx_0",
    )
    _finite_zero_array(constraints.get("lbx_0"), checked_assembly.nx, "lbx_0")
    _finite_zero_array(constraints.get("ubx_0"), checked_assembly.nx, "ubx_0")
    for name, expected in (
        ("lbu", checked_assembly.control_lower),
        ("ubu", checked_assembly.control_upper),
        ("lh_0", checked_assembly.stage_constraint_lower),
        ("uh_0", checked_assembly.stage_constraint_upper),
        ("lh", checked_assembly.stage_constraint_lower),
        ("uh", checked_assembly.stage_constraint_upper),
        ("lh_e", checked_assembly.terminal_constraint_lower),
        ("uh_e", checked_assembly.terminal_constraint_upper),
    ):
        _same(constraints.get(name), list(expected), f"constraints.{name}")
    _finite_zero_array(
        top.get("parameter_values"), checked_assembly.np, "parameter_values"
    )
    _same(top.get("p_global_values"), [], "p_global_values")
    _same(top.get("problem_class"), "OCP", "problem_class")

    actual_solver = _object(
        top.get("solver_options"), "generated Acados solver options"
    )
    try:
        actual_solver_baseline = acados_solver_options_baseline_sha256(actual_solver)
    except AcadosSolverOptionsIdentityError as exc:
        raise AcadosCodegenValidationError(
            "generated Acados solver-options baseline is malformed"
        ) from exc
    if (
        actual_solver_baseline
        != checked_assembly.backend_solver_options_baseline_sha256
    ):
        raise AcadosCodegenValidationError(
            "generated Acados solver-options baseline drifted"
        )
    _same(
        actual_solver.get("N_horizon"),
        checked_solver.horizon_steps,
        "solver_options.N_horizon",
    )
    _same(
        actual_solver.get("tf"),
        float(checked_solver.time_horizon_sec),
        "solver_options.tf",
    )
    _same(
        actual_solver.get("time_steps"),
        [float(value) for value in checked_solver.time_steps],
        "solver_options.time_steps",
    )
    _same(
        actual_solver.get("cost_scaling"),
        list(checked_solver.cost_scaling),
        "solver_options.cost_scaling",
    )
    for name in ACADOS_SCALAR_SOLVER_OPTION_FIELDS:
        _same(
            actual_solver.get(name),
            getattr(checked_solver, name),
            f"solver_options.{name}",
        )
    for name in ACADOS_INTEGER_BOOLEAN_SOLVER_OPTION_FIELDS:
        _same(
            actual_solver.get(name),
            int(getattr(checked_solver, name)),
            f"solver_options.{name}",
        )
    for name in (
        "ext_fun_compile_flags",
        "ext_fun_expand_constr",
        "ext_fun_expand_cost",
        "ext_fun_expand_dyn",
        "ext_fun_expand_precompute",
        "custom_update_filename",
        "custom_update_header_filename",
        "custom_update_copy",
    ):
        _same(
            actual_solver.get(name),
            getattr(checked_codegen, name),
            f"solver_options.{name}",
        )
    _same(
        actual_solver.get("custom_templates"),
        [list(item) for item in checked_codegen.custom_templates],
        "solver_options.custom_templates",
    )

    actual_codegen = _object(
        top.get("code_gen_opts"), "generated Acados codegen options"
    )
    expected_export = (
        GENERATED_ROOT_CANONICAL_VALUE if canonicalized_output_root else str(root)
    )
    expected_json = ACADOS_JSON_FILENAME if canonicalized_output_root else str(path)
    _same(
        actual_codegen.get("code_export_directory"),
        expected_export,
        "code_gen_opts.code_export_directory",
    )
    _same(
        actual_codegen.get("json_file"),
        expected_json,
        "code_gen_opts.json_file",
    )
    _same(
        actual_codegen.get("acados_include_path"),
        str(source_root / "include"),
        "code_gen_opts.acados_include_path",
    )
    _same(
        actual_codegen.get("acados_lib_path"),
        str(source_root / "lib"),
        "code_gen_opts.acados_lib_path",
    )
    _same(
        actual_codegen.get("acados_version"),
        checked_assembly.acados_git_commit,
        "code_gen_opts.acados_version",
    )
    link_libraries = _read_acados_link_libraries(source_root)
    _same(
        actual_codegen.get("acados_link_libs"),
        link_libraries,
        "code_gen_opts.acados_link_libs",
    )
    _same(actual_codegen.get("os"), "unix", "code_gen_opts.os")
    _same(actual_codegen.get("shared_lib_ext"), ".so", "shared library suffix")

    acados_hash = top.get("hash")
    if (
        type(acados_hash) is not str
        or len(acados_hash) != 32
        or any(character not in "0123456789abcdef" for character in acados_hash)
    ):
        raise AcadosCodegenValidationError("Acados internal MD5 field is malformed")
    if acados_hash != _acados_internal_md5(top):
        raise AcadosCodegenValidationError("Acados internal code-reuse hash drifted")
    for group_name in (
        "external_function_files_model",
        "external_function_files_ocp",
    ):
        sources = _array(top.get(group_name), f"generated Acados {group_name}")
        if not sources:
            raise AcadosCodegenValidationError(
                f"generated Acados {group_name} is empty"
            )
        for index, value in enumerate(sources):
            _relative_generated_source(value, root, f"{group_name}[{index}]")
    return top


def _atomic_replace_regular(path: Path, payload: bytes) -> None:
    try:
        target_metadata = path.lstat()
    except OSError as exc:
        raise AcadosCodegenValidationError(
            f"cannot inspect generated metadata {path}: {exc}"
        ) from exc
    if not stat.S_ISREG(target_metadata.st_mode):
        raise AcadosCodegenValidationError(
            f"generated metadata is not a regular file: {path}"
        )
    target_mode = stat.S_IMODE(target_metadata.st_mode)
    temporary: Path | None = None
    temporary_identity: tuple[int, int] | None = None
    descriptor: int | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.canonical.",
            suffix=".tmp",
            dir=str(path.parent),
        )
        temporary = Path(temporary_name)
        metadata = os.fstat(descriptor)
        temporary_identity = (metadata.st_dev, metadata.st_ino)
        os.fchmod(descriptor, target_mode)
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    except OSError as exc:
        raise AcadosCodegenValidationError(
            f"cannot canonicalize generated metadata {path}: {exc}"
        ) from exc
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        if temporary is not None and temporary_identity is not None:
            try:
                current = os.stat(temporary, follow_symlinks=False)
                current_identity = (current.st_dev, current.st_ino)
                if current_identity == temporary_identity and stat.S_ISREG(
                    current.st_mode
                ):
                    temporary.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                pass


def canonicalize_generated_output_root(
    output_root: Path | str,
    assembly: AcadosOcpAssembly,
    solver_options: SolverOptionsSnapshot,
    codegen_options: CodegenOptionsSnapshot,
    acados_source_root: Path | str,
) -> None:
    """Remove the staging root from JSON/Makefile after a successful build."""

    root = Path(output_root)
    json_path = root / ACADOS_JSON_FILENAME
    document = validate_generated_acados_json(
        json_path,
        root,
        acados_source_root,
        assembly,
        solver_options,
        codegen_options,
        canonicalized_output_root=False,
    )
    codegen = _object(document["code_gen_opts"], "generated Acados codegen options")
    codegen["code_export_directory"] = GENERATED_ROOT_CANONICAL_VALUE
    codegen["json_file"] = ACADOS_JSON_FILENAME
    document["hash"] = _acados_internal_md5(document)
    _atomic_replace_regular(
        json_path,
        (canonical_json(document) + "\n").encode("utf-8"),
    )

    makefile = root / "Makefile"
    try:
        lines = makefile.read_text(encoding="utf-8").splitlines(keepends=True)
    except (OSError, UnicodeDecodeError) as exc:
        raise AcadosCodegenValidationError(
            f"cannot read generated Makefile: {exc}"
        ) from exc
    root_text = str(root)
    replacements = 0
    canonical_lines = []
    for line in lines:
        if root_text not in line:
            canonical_lines.append(line)
            continue
        if line.strip() != f"-I {root_text} \\":
            raise AcadosCodegenValidationError(
                "staging root appeared in an unexpected Makefile field"
            )
        canonical_lines.append(line.replace(root_text, GENERATED_ROOT_CANONICAL_VALUE))
        replacements += 1
    if replacements != 1:
        raise AcadosCodegenValidationError(
            "generated Makefile did not contain one canonical staging-root field"
        )
    _atomic_replace_regular(makefile, "".join(canonical_lines).encode("utf-8"))
    validate_generated_acados_json(
        json_path,
        root,
        acados_source_root,
        assembly,
        solver_options,
        codegen_options,
        canonicalized_output_root=True,
    )


def validate_no_embedded_output_root(
    output_root: Path | str,
    records: tuple[GeneratedFileRecord, ...],
) -> None:
    """Prove the absolute staging directory is absent from all artifact bytes."""

    root = Path(output_root)
    checked = generated_file_records_from_dict([item.to_dict() for item in records])
    marker = str(root).encode("utf-8")
    for record in checked:
        try:
            payload = (root / record.relative_path).read_bytes()
        except OSError as exc:
            raise AcadosCodegenValidationError(
                f"cannot scan generated file for staging path: {record.relative_path}"
            ) from exc
        if marker in payload:
            raise AcadosCodegenValidationError(
                f"absolute staging root leaked into {record.relative_path}"
            )


def validate_solver_shared_library(
    output_root: Path | str,
    library: GeneratedFileRecord,
    nm_executable: Path | str,
    expected_elf_machine: int,
) -> dict[str, Any]:
    """Verify ELF type/machine, raw hash, and the required public solver symbols."""

    if type(library) is not GeneratedFileRecord or library.role != SOLVER_LIBRARY_ROLE:
        raise AcadosCodegenValidationError("solver library record is not canonical")
    if type(expected_elf_machine) is not int or expected_elf_machine <= 0:
        raise AcadosCodegenValidationError("expected ELF machine must be explicit")
    root = Path(output_root)
    path = root / library.relative_path
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise AcadosCodegenValidationError(
            f"cannot read solver library: {exc}"
        ) from exc
    if len(payload) < 64 or payload[:4] != b"\x7fELF":
        raise AcadosCodegenValidationError("solver library is not an ELF file")
    elf_class = payload[4]
    byte_order = payload[5]
    if elf_class not in {1, 2} or byte_order not in {1, 2}:
        raise AcadosCodegenValidationError("solver ELF class or byte order is invalid")
    prefix = "<" if byte_order == 1 else ">"
    elf_type, elf_machine = struct.unpack_from(prefix + "HH", payload, 16)
    if elf_type != ELF_SHARED_OBJECT_TYPE or elf_machine != expected_elf_machine:
        raise AcadosCodegenValidationError("solver ELF type or machine is incompatible")
    if (
        len(payload) != library.size_bytes
        or sha256_bytes(payload) != library.raw_sha256
    ):
        raise AcadosCodegenValidationError(
            "solver library differs from its file record"
        )

    nm = Path(nm_executable)
    if not nm.is_absolute() or nm.is_symlink() or not nm.is_file():
        raise AcadosCodegenValidationError(
            "nm executable is not an absolute regular file"
        )
    try:
        result = subprocess.run(
            [str(nm), "-D", "--defined-only", str(path)],
            check=True,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="strict",
            env={**os.environ, "LC_ALL": "C"},
        )
    except (OSError, subprocess.CalledProcessError, UnicodeDecodeError) as exc:
        raise AcadosCodegenValidationError(
            "cannot inspect solver shared-library symbols"
        ) from exc
    exported = {line.split()[-1] for line in result.stdout.splitlines() if line.split()}
    required = {f"{MODEL_ID}_{suffix}" for suffix in REQUIRED_SOLVER_SYMBOL_SUFFIXES}
    missing = sorted(required - exported)
    if missing:
        raise AcadosCodegenValidationError(
            f"solver shared library is missing symbols: {', '.join(missing)}"
        )
    return {
        "schema_version": ACADOS_JSON_VALIDATION_SCHEMA,
        "format": "ELF",
        "elf_class": 32 if elf_class == 1 else 64,
        "elf_machine": elf_machine,
        "required_exported_symbols": sorted(required),
    }


__all__ = [
    "ACADOS_JSON_VALIDATION_SCHEMA",
    "ELF_MACHINE_BY_PLATFORM",
    "GENERATED_ROOT_CANONICAL_VALUE",
    "REQUIRED_SOLVER_SYMBOL_SUFFIXES",
    "AcadosCodegenValidationError",
    "canonicalize_generated_output_root",
    "validate_generated_acados_json",
    "validate_no_embedded_output_root",
    "validate_solver_shared_library",
]
