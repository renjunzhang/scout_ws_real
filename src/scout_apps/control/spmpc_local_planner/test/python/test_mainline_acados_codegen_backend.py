"""Contract tests for the Stage 3-D4 Acados generate/build boundary."""

from __future__ import annotations

import copy
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline import acados_codegen_backend as codegen_backend
from acados.mainline import acados_codegen_validation as codegen_validation
from acados.mainline.acados_codegen_backend import (
    AcadosCodegenError,
    generate_and_build_acados,
    require_acados_codegen_result,
)
from acados.mainline.acados_codegen_validation import AcadosCodegenValidationError
from acados.mainline.acados_ocp_adapter import assemble_acados_ocp
from acados.mainline.artifact_files import (
    generated_tree_sha256,
    inventory_generated_tree,
    prepare_empty_codegen_directory,
)
from acados.mainline.casadi_adapter import build_casadi_graph
from acados.mainline.codegen_options import (
    ACADOS_JSON_FILENAME,
    build_codegen_options_snapshot,
)
from acados.mainline.constraints_oracle import ConstraintBounds
from acados.mainline.development_capacity import load_development_capacity
from acados.mainline.development_layout import build_development_layout
from acados.mainline.identity import sha256_json
from acados.mainline.solver_options import build_solver_options_snapshot

CAPACITY = (
    PACKAGE_ROOT / "config" / "mainline" / "contracts" / "development_capacity_v1.json"
)
CASADI_AVAILABLE = importlib.util.find_spec("casadi") is not None
ACADOS_AVAILABLE = importlib.util.find_spec("acados_template") is not None
BACKEND_REQUIRED = os.environ.get("SPMPC_REQUIRE_ACADOS_BACKEND") == "1"


class MainlineAcadosCodegenBoundaryTest(unittest.TestCase):
    def test_import_is_lazy_and_does_not_load_numeric_backends(self) -> None:
        code = (
            "import sys; "
            f"sys.path.insert(0, {str(SCRIPTS_ROOT)!r}); "
            "import acados.mainline.acados_codegen_backend; "
            "print('casadi' in sys.modules, 'numpy' in sys.modules, "
            "'acados_template' in sys.modules)"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "False False False")

    def test_non_typed_authorities_fail_before_output_creation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "new-staging"
            with self.assertRaises(AcadosCodegenError):
                generate_and_build_acados(
                    object(),  # type: ignore[arg-type]
                    object(),  # type: ignore[arg-type]
                    object(),  # type: ignore[arg-type]
                    object(),  # type: ignore[arg-type]
                    output,
                    root / "missing-acados",
                    root / "missing-tera",
                )
            self.assertFalse(output.exists())

    def test_backend_required_environment_cannot_silently_skip(self) -> None:
        if BACKEND_REQUIRED:
            self.assertTrue(CASADI_AVAILABLE)
            self.assertTrue(ACADOS_AVAILABLE)

    def _typed_authorities(self):
        capacity = load_development_capacity(CAPACITY)
        layout = build_development_layout(capacity)
        graph = build_casadi_graph(
            capacity,
            layout,
            ConstraintBounds(*([10.0] * 7)),
        )
        solver_options = build_solver_options_snapshot(layout)
        assembly = assemble_acados_ocp(graph, solver_options)
        codegen_options = build_codegen_options_snapshot(
            capacity,
            layout,
            1.0e-12,
            1.0e-12,
            "-O2",
        )
        return graph, assembly, solver_options, codegen_options

    def _backend_paths(self) -> tuple[Path, Path]:
        source_value = os.environ.get("ACADOS_SOURCE_DIR", "")
        source = Path(source_value) if source_value else Path()
        tera_value = os.environ.get("TERA_PATH", "")
        tera = Path(tera_value) if tera_value else source / "bin" / "t_renderer"
        required = (
            source.is_absolute()
            and (source / "include").is_dir()
            and (source / "lib").is_dir()
            and (source / "lib" / "libacados.so").is_file()
            and (source / "lib" / "libhpipm.so").is_file()
            and (source / "lib" / "libblasfeo.so").is_file()
            and tera.is_absolute()
            and tera.is_file()
            and os.access(tera, os.X_OK)
        )
        if not required:
            self.skipTest(
                "explicit Acados install root with libraries/include and Tera is unavailable"
            )
        return source, tera

    @unittest.skipUnless(
        CASADI_AVAILABLE and ACADOS_AVAILABLE,
        "CasADi and acados_template are not installed",
    )
    def test_explicit_backend_generates_loadable_relative_root_hashed_artifact(
        self,
    ) -> None:
        source, tera = self._backend_paths()
        graph, assembly, solver_options, codegen_options = self._typed_authorities()
        with tempfile.TemporaryDirectory() as directory:
            staging_parent = Path(directory)
            output = staging_parent / "generated"
            result = generate_and_build_acados(
                graph,
                assembly,
                solver_options,
                codegen_options,
                output,
                source,
                tera,
            )
            self.assertEqual(result.output_directory, output.resolve())
            self.assertEqual(result.output_directory, output)

            document = json.loads(
                (output / ACADOS_JSON_FILENAME).read_text(encoding="utf-8")
            )
            self.assertEqual(document["code_gen_opts"]["code_export_directory"], ".")
            self.assertEqual(
                document["code_gen_opts"]["json_file"], ACADOS_JSON_FILENAME
            )
            self.assertNotIn(str(output), json.dumps(document, sort_keys=True))

            records = inventory_generated_tree(output)
            self.assertEqual(result.files, records)
            self.assertEqual(
                result.generated_tree_sha256,
                generated_tree_sha256(records),
            )
            self.assertEqual(
                result.to_dict()["generated_tree"]["sha256"],
                generated_tree_sha256(records),
            )
            solver = result.to_dict()["solver_library"]
            self.assertEqual(solver["format"], "ELF")
            self.assertEqual(solver["load_check"], "PASSED_IN_ISOLATED_PROCESS")
            self.assertEqual(
                set(solver["required_exported_symbols"]),
                set(result.required_exported_symbols),
            )
            self.assertGreaterEqual(result.elf_class, 32)
            self.assertGreater(result.elf_machine, 0)
            self.assertTrue((output / result.solver_library_relative_path).is_file())
            self.assertEqual(
                result.solver_library_size_bytes,
                (output / result.solver_library_relative_path).stat().st_size,
            )

            for name, changed_value in (
                ("output_directory", Path("relative/generated")),
                ("files", list(result.files)),
                ("status", "FORGED"),
                ("elf_class", 7),
            ):
                forged = copy.copy(result)
                object.__setattr__(forged, name, changed_value)
                object.__setattr__(
                    forged,
                    "semantic_sha256",
                    sha256_json(codegen_backend._result_payload(forged)),
                )
                with (
                    self.subTest(force_mutated_field=name),
                    self.assertRaises(AcadosCodegenError),
                ):
                    require_acados_codegen_result(forged)

    @unittest.skipUnless(
        CASADI_AVAILABLE and ACADOS_AVAILABLE,
        "CasADi and acados_template are not installed",
    )
    def test_environment_and_unlisted_option_drift_fail_before_staging(self) -> None:
        source, tera = self._backend_paths()
        graph, assembly, solver_options, codegen_options = self._typed_authorities()
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            changed_cflags = os.environ.get("CFLAGS", "") + " -DSPMPC_DRIFT=1"
            environment_output = parent / "environment-drift"
            with (
                patch.dict(os.environ, {"CFLAGS": changed_cflags}),
                self.assertRaises(AcadosCodegenError),
            ):
                generate_and_build_acados(
                    graph,
                    assembly,
                    solver_options,
                    codegen_options,
                    environment_output,
                    source,
                    tera,
                )
            self.assertFalse(environment_output.exists())

            assembly.ocp.solver_options.qp_solver_mu0 = 1.0
            option_output = parent / "solver-option-drift"
            with self.assertRaises(AcadosCodegenError):
                generate_and_build_acados(
                    graph,
                    assembly,
                    solver_options,
                    codegen_options,
                    option_output,
                    source,
                    tera,
                )
            self.assertFalse(option_output.exists())

    def test_fixed_make_binding_rejects_path_drift(self) -> None:
        make_value = codegen_backend.shutil.which("make")
        false_value = codegen_backend.shutil.which("false")
        if make_value is None or false_value is None:
            self.skipTest("make/false are unavailable")
        expected = Path(make_value).resolve(strict=True)
        with (
            patch.object(
                codegen_backend.shutil,
                "which",
                return_value=false_value,
            ),
            self.assertRaises(AcadosCodegenError),
        ):
            codegen_backend._require_tool_binding("make", expected)

    def test_fixed_make_checks_clean_and_build_return_codes(self) -> None:
        make_path = Path("/usr/bin/make")
        output = Path("/absolute/generated")
        with (
            patch.object(codegen_backend, "_require_tool_binding") as binding,
            patch.object(codegen_backend.subprocess, "run") as runner,
        ):
            codegen_backend._build_with_fixed_make(
                make_path,
                output,
                verbose=False,
            )
        binding.assert_called_once_with("make", make_path)
        self.assertEqual(
            [item.args[0] for item in runner.call_args_list],
            [
                [
                    str(make_path),
                    "--no-print-directory",
                    codegen_backend.MAKE_CLEAN_TARGET,
                ],
                [
                    str(make_path),
                    "--no-print-directory",
                    codegen_backend.MAKE_BUILD_TARGET,
                ],
            ],
        )
        for item in runner.call_args_list:
            self.assertEqual(item.kwargs["cwd"], output)
            self.assertIs(item.kwargs["check"], True)
            self.assertIs(item.kwargs["capture_output"], True)

        failure = subprocess.CalledProcessError(
            2,
            [str(make_path), codegen_backend.MAKE_BUILD_TARGET],
        )
        with (
            patch.object(codegen_backend, "_require_tool_binding"),
            patch.object(
                codegen_backend.subprocess,
                "run",
                side_effect=[None, failure],
            ),
            self.assertRaises(AcadosCodegenError),
        ):
            codegen_backend._build_with_fixed_make(
                make_path,
                output,
                verbose=False,
            )

    def test_atomic_replace_uses_unique_same_directory_temporary_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "metadata.json"
            target.write_bytes(b"old")
            target.chmod(0o640)
            created: list[Path] = []
            real_mkstemp = codegen_validation.tempfile.mkstemp

            def record_temp(*args, **kwargs):
                descriptor, name = real_mkstemp(*args, **kwargs)
                created.append(Path(name))
                return descriptor, name

            with patch.object(
                codegen_validation.tempfile,
                "mkstemp",
                side_effect=record_temp,
            ):
                codegen_validation._atomic_replace_regular(target, b"new")

            self.assertEqual(target.read_bytes(), b"new")
            self.assertEqual(target.stat().st_mode & 0o777, 0o640)
            self.assertEqual(len(created), 1)
            self.assertEqual(created[0].parent, root)
            self.assertNotEqual(created[0].name, ".metadata.json.canonical.tmp")
            self.assertEqual(set(root.iterdir()), {target})

    def test_atomic_replace_failure_cleans_only_its_own_temporary_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "metadata.json"
            target.write_bytes(b"old")
            collision = root / ".metadata.json.canonical.tmp"
            collision.write_bytes(b"another process")
            with (
                patch.object(
                    codegen_validation.os,
                    "replace",
                    side_effect=OSError("replace failed"),
                ),
                self.assertRaises(AcadosCodegenValidationError),
            ):
                codegen_validation._atomic_replace_regular(target, b"new")
            self.assertEqual(collision.read_bytes(), b"another process")
            self.assertEqual(target.read_bytes(), b"old")
            self.assertEqual(
                {path.name for path in root.iterdir()},
                {"metadata.json", ".metadata.json.canonical.tmp"},
            )

    def test_atomic_replace_open_collision_does_not_delete_preexisting_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "metadata.json"
            target.write_bytes(b"old")
            collision = root / ".metadata.json.canonical.tmp"
            collision.write_bytes(b"owned by another process")
            with (
                patch.object(
                    codegen_validation.tempfile,
                    "mkstemp",
                    side_effect=FileExistsError("collision"),
                ),
                self.assertRaises(AcadosCodegenValidationError),
            ):
                codegen_validation._atomic_replace_regular(target, b"new")
            self.assertEqual(collision.read_bytes(), b"owned by another process")
            self.assertEqual(target.read_bytes(), b"old")

    @unittest.skipUnless(
        CASADI_AVAILABLE and ACADOS_AVAILABLE,
        "CasADi and acados_template are not installed",
    )
    def test_nonempty_staging_directory_fails_closed(self) -> None:
        source, tera = self._backend_paths()
        graph, assembly, solver_options, codegen_options = self._typed_authorities()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated"
            output.mkdir()
            sentinel = output / "do-not-overwrite"
            sentinel.write_text("sentinel\n", encoding="utf-8")
            with self.assertRaises(AcadosCodegenError):
                generate_and_build_acados(
                    graph,
                    assembly,
                    solver_options,
                    codegen_options,
                    output,
                    source,
                    tera,
                )
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "sentinel\n")

    def test_empty_directory_precondition_is_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "generated"
            checked = prepare_empty_codegen_directory(output)
            self.assertEqual(checked, output)
            (output / "sentinel").write_text("sentinel\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                prepare_empty_codegen_directory(output)


if __name__ == "__main__":
    unittest.main()
