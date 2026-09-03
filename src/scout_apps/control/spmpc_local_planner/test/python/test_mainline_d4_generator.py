"""Boundary tests for the one-shot Stage 3-D4 generator."""

from __future__ import annotations

import ast
import importlib.util
import multiprocessing
import os
import queue
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
MAINLINE_ROOT = SCRIPTS_ROOT / "acados" / "mainline"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline import d4_generator
from acados.mainline.artifact_publication import (
    ArtifactPublicationError,
    load_artifact_contract_directory,
    publish_staging_directory,
)
from acados.mainline.codegen_options import (
    ACADOS_JSON_FILENAME,
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
)
from acados.mainline.constraints_oracle import ConstraintBounds
from acados.mainline.d4_source_paths import (
    D4_SOURCE_PATHS,
    require_d4_source_paths,
)


def _repository_root() -> Path:
    for candidate in (PACKAGE_ROOT, *PACKAGE_ROOT.parents):
        if (candidate / ".git").is_dir():
            return candidate
    raise RuntimeError("test package is not below a Git repository")


REPOSITORY_ROOT = _repository_root()
CLI = SCRIPTS_ROOT / "acados" / "generate_mainline_d4.py"
MAINLINE_RELATIVE_ROOT = MAINLINE_ROOT.relative_to(REPOSITORY_ROOT)
CASADI_AVAILABLE = importlib.util.find_spec("casadi") is not None
ACADOS_AVAILABLE = importlib.util.find_spec("acados_template") is not None
BACKEND_REQUIRED = os.environ.get("SPMPC_REQUIRE_ACADOS_BACKEND") == "1"


def _publish_worker(
    staging: str,
    target: str,
    start_gate: Any,
    results: Any,
) -> None:
    start_gate.wait()
    try:
        publish_staging_directory(Path(staging), Path(target))
    except ArtifactPublicationError as exc:
        results.put(("rejected", Path(staging).name, str(exc)))
    else:
        results.put(("published", Path(staging).name, ""))


def _local_module_paths() -> dict[str, str]:
    prefix = MAINLINE_RELATIVE_ROOT.as_posix()
    result = {
        "acados.mainline": f"{prefix}/__init__.py",
    }
    for path in MAINLINE_ROOT.glob("*.py"):
        if path.name != "__init__.py":
            result[f"acados.mainline.{path.stem}"] = path.relative_to(
                REPOSITORY_ROOT
            ).as_posix()
    return result


def _imported_local_modules(source: Path) -> set[str]:
    package = "acados.mainline" if source.parent == MAINLINE_ROOT else "acados"
    local_modules = _local_module_paths()
    imported: set[str] = set()
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    for node in ast.walk(tree):
        candidates: list[str] = []
        if isinstance(node, ast.Import):
            candidates.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                package_parts = package.split(".")
                keep = len(package_parts) - node.level + 1
                base = package_parts[:keep]
                if node.module:
                    candidates.append(".".join((*base, *node.module.split("."))))
                else:
                    candidates.extend(
                        ".".join((*base, alias.name)) for alias in node.names
                    )
            elif node.module:
                candidates.append(node.module)
        imported.update(name for name in candidates if name in local_modules)
    return imported


class MainlineD4GeneratorTest(unittest.TestCase):
    def test_cli_import_is_lazy_and_does_not_load_numeric_backends(self) -> None:
        code = (
            "import runpy, sys; "
            f"runpy.run_path({str(CLI)!r}, run_name='d4_import_check'); "
            "print('casadi' in sys.modules, 'numpy' in sys.modules, "
            "'acados_template' in sys.modules)"
        )
        result = subprocess.run(
            [sys.executable, "-I", "-c", code],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.stdout.strip(), "False False False")

    def test_source_inventory_is_unique_sorted_and_complete(self) -> None:
        self.assertIs(type(D4_SOURCE_PATHS), tuple)
        self.assertEqual(D4_SOURCE_PATHS, tuple(sorted(set(D4_SOURCE_PATHS))))
        self.assertEqual(
            require_d4_source_paths(REPOSITORY_ROOT),
            D4_SOURCE_PATHS,
        )
        for relative in D4_SOURCE_PATHS:
            with self.subTest(relative=relative):
                self.assertTrue((REPOSITORY_ROOT / relative).is_file())

    def test_required_backend_cannot_silently_skip(self) -> None:
        if BACKEND_REQUIRED:
            self.assertTrue(CASADI_AVAILABLE)
            self.assertTrue(ACADOS_AVAILABLE)

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

    def test_source_inventory_covers_its_local_ast_dependency_closure(self) -> None:
        inventory = set(D4_SOURCE_PATHS)
        local_modules = _local_module_paths()
        required_modules: set[str] = set()
        pending = list(_imported_local_modules(CLI))
        while pending:
            module = pending.pop()
            if module in required_modules:
                continue
            required_modules.add(module)
            source = REPOSITORY_ROOT / local_modules[module]
            pending.extend(_imported_local_modules(source) - required_modules)
        required_paths = {local_modules[module] for module in required_modules} | {
            CLI.relative_to(REPOSITORY_ROOT).as_posix(),
            local_modules["acados.mainline"],
            d4_generator.CAPACITY_CONTRACT_RELATIVE_PATH,
        }
        self.assertEqual(inventory, required_paths)

    def test_source_inventory_excludes_evidence_legacy_and_data_inputs(self) -> None:
        prefix = MAINLINE_RELATIVE_ROOT.as_posix()
        excluded = {
            f"{prefix}/contract_source.py",
            f"{prefix}/cost_oracle.py",
            f"{prefix}/cost_schedule.py",
            f"{prefix}/layout.py",
            f"{prefix}/manifest.py",
            f"{prefix}/parameter_values.py",
            f"{prefix}/reference_oracle.py",
            f"{prefix}/runtime_parameter_assembler.py",
            f"{prefix}/stage1_evidence.py",
        }
        self.assertTrue(excluded.isdisjoint(D4_SOURCE_PATHS))
        for relative in D4_SOURCE_PATHS:
            normalized = f"/{relative.lower()}/"
            self.assertNotIn("/docs/", normalized)
            self.assertNotIn("/bag/", normalized)
            self.assertNotIn("/scripts/analysis/", normalized)
            self.assertNotIn("legacy", normalized)
            self.assertFalse(relative.lower().endswith((".bag", ".db3", ".mcap")))

    def test_existing_target_fails_before_provenance_capture(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "owned-artifact"
            target.mkdir()
            marker = target / "owner"
            marker.write_text("existing\n", encoding="utf-8")
            request = d4_generator.D4GenerationRequest(
                repository_root=REPOSITORY_ROOT,
                output_directory=target,
                acados_install_root=root / "unused-acados",
                tera_executable=root / "unused-tera",
                constraint_bounds=ConstraintBounds(*([1.0] * 7)),
                integer_snap_tolerance_sec=1.0e-12,
                duration_tolerance_sec=1.0e-12,
                ext_fun_compile_flags="-O2",
            )
            capture_patch = patch.object(
                d4_generator,
                "capture_codegen_provenance",
            )
            expected_failure = self.assertRaisesRegex(
                d4_generator.D4GenerationError,
                "preflight",
            )
            with capture_patch as capture, expected_failure:
                d4_generator.generate_d4_development_artifact(request)
            capture.assert_not_called()
            self.assertEqual(marker.read_text(encoding="utf-8"), "existing\n")

    def test_generation_failure_retains_the_unique_staging_directory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "failed-artifact"
            request = d4_generator.D4GenerationRequest(
                repository_root=REPOSITORY_ROOT,
                output_directory=target,
                acados_install_root=root / "fixture-acados",
                tera_executable=root / "fixture-tera",
                constraint_bounds=ConstraintBounds(*([1.0] * 7)),
                integer_snap_tolerance_sec=1.0e-12,
                duration_tolerance_sec=1.0e-12,
                ext_fun_compile_flags="-O2",
            )
            codegen_options = SimpleNamespace(compiler_environment=())
            returns = {
                "require_d4_source_paths": ("fixture.py",),
                "load_development_capacity": object(),
                "build_development_layout": object(),
                "build_solver_parameter_layout": object(),
                "build_solver_options_snapshot": object(),
                "build_codegen_options_snapshot": codegen_options,
                "capture_codegen_provenance": object(),
                "build_casadi_graph": object(),
                "assemble_acados_ocp": object(),
            }
            with ExitStack() as stack:
                for name, value in returns.items():
                    stack.enter_context(
                        patch.object(d4_generator, name, return_value=value)
                    )
                stack.enter_context(
                    patch.object(
                        d4_generator,
                        "generate_and_build_acados",
                        side_effect=RuntimeError("injected backend failure"),
                    )
                )
                stack.enter_context(
                    self.assertRaisesRegex(
                        d4_generator.D4GenerationError,
                        "partial staging retained",
                    )
                )
                d4_generator.generate_d4_development_artifact(request)
            self.assertFalse(target.exists())
            retained = list(root.glob(f".{target.name}.staging-*"))
            self.assertEqual(len(retained), 1)
            self.assertTrue(retained[0].is_dir())

    def test_competing_publishers_never_replace_each_other(self) -> None:
        try:
            context = multiprocessing.get_context("fork")
        except ValueError:
            self.skipTest("fork multiprocessing is unavailable")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "published"
            stagings = (root / "candidate-a", root / "candidate-b")
            for staging in stagings:
                staging.mkdir()
                (staging / "owner").write_text(staging.name, encoding="utf-8")
            start_gate = context.Event()
            results = context.Queue()
            processes = [
                context.Process(
                    target=_publish_worker,
                    args=(str(staging), str(target), start_gate, results),
                )
                for staging in stagings
            ]
            for process in processes:
                process.start()
            start_gate.set()
            for process in processes:
                process.join(timeout=10.0)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=5.0)
                    self.fail("competing publication worker did not finish")
                self.assertEqual(process.exitcode, 0)
            try:
                outcomes = [results.get(timeout=2.0) for _ in processes]
            except queue.Empty:
                self.fail("competing publication worker returned no outcome")
            finally:
                results.close()
                results.join_thread()

            published = [item for item in outcomes if item[0] == "published"]
            rejected = [item for item in outcomes if item[0] == "rejected"]
            self.assertEqual(len(published), 1, outcomes)
            self.assertEqual(len(rejected), 1, outcomes)
            self.assertRegex(rejected[0][2], "already exists|appeared")
            winner_name = published[0][1]
            loser_name = rejected[0][1]
            self.assertEqual(
                (target / "owner").read_text(encoding="utf-8"),
                winner_name,
            )
            self.assertFalse((root / winner_name).exists())
            self.assertEqual(
                (root / loser_name / "owner").read_text(encoding="utf-8"),
                loser_name,
            )

    @unittest.skipUnless(
        CASADI_AVAILABLE and ACADOS_AVAILABLE,
        "CasADi and acados_template are not installed",
    )
    def test_explicit_backend_publishes_complete_reloadable_artifact(self) -> None:
        source, tera = self._backend_paths()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "spmpc-mainline-d4"
            request = d4_generator.D4GenerationRequest(
                repository_root=REPOSITORY_ROOT,
                output_directory=target,
                acados_install_root=source,
                tera_executable=tera,
                constraint_bounds=ConstraintBounds(*([10.0] * 7)),
                integer_snap_tolerance_sec=1.0e-12,
                duration_tolerance_sec=1.0e-12,
                ext_fun_compile_flags="-O2",
            )
            ambient_paths = {
                "ACADOS_SOURCE_DIR": "/ambient/path/must/not/be/used",
                "TERA_PATH": "/ambient/renderer/must/not/be/used",
            }
            with patch.dict(os.environ, ambient_paths):
                result = d4_generator.generate_d4_development_artifact(request)
                self.assertEqual(
                    os.environ["ACADOS_SOURCE_DIR"],
                    ambient_paths["ACADOS_SOURCE_DIR"],
                )
                self.assertEqual(os.environ["TERA_PATH"], ambient_paths["TERA_PATH"])
            self.assertEqual(result.output_directory, target)
            self.assertEqual(
                result.to_dict()["status"],
                "PUBLISHED_DEV_UNVALIDATED",
            )
            reloaded = load_artifact_contract_directory(target)
            self.assertEqual(reloaded.to_dict(), result.contract.to_dict())
            self.assertIs(
                d4_generator.require_d4_generation_result(result),
                result,
            )
            for filename in (
                ACADOS_JSON_FILENAME,
                MODEL_CONTRACT_FILENAME,
                GENERATED_HEADER_FILENAME,
            ):
                self.assertTrue((target / filename).is_file())
            contract = result.contract.to_dict()
            self.assertEqual(contract["status"]["artifact"], "GENERATED_AND_BUILT")
            self.assertEqual(contract["status"]["artifact_class"], "DEV_UNVALIDATED")
            self.assertEqual(contract["status"]["promotion"], "NOT_PROMOTED")
            self.assertTrue(contract["comparison_contract"]["shared_artifact_required"])
            self.assertEqual(
                contract["typed_authorities"]["provenance"],
                result.provenance.to_dict(),
            )
            self.assertEqual(
                list(root.glob(f".{target.name}.staging-*")),
                [],
            )

            contract_bytes = (target / MODEL_CONTRACT_FILENAME).read_bytes()
            with self.assertRaisesRegex(d4_generator.D4GenerationError, "preflight"):
                d4_generator.generate_d4_development_artifact(request)
            self.assertEqual(
                (target / MODEL_CONTRACT_FILENAME).read_bytes(),
                contract_bytes,
            )


if __name__ == "__main__":
    unittest.main()
