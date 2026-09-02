"""Unit tests for the dependency-light Stage 3-D4 provenance interfaces."""

from __future__ import annotations

import os
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline import identity as identity_module
from acados.mainline import provenance as provenance_module
from acados.mainline import provenance_acados as acados_module
from acados.mainline import provenance_git as git_module
from acados.mainline.codegen_options import COMPILER_ENVIRONMENT_NAMES
from acados.mainline.identity import (
    IdentityError,
    read_stable_regular_file,
)
from acados.mainline.provenance import (
    capture_codegen_provenance,
    require_codegen_provenance,
)
from acados.mainline.provenance_acados import capture_acados_install
from acados.mainline.provenance_common import GIT_DIRTY_POLICY, ProvenanceError
from acados.mainline.provenance_files import (
    capture_linked_file,
    capture_selected_source_files,
    capture_tool_identity,
    require_source_tree,
    require_tool_identity,
)
from acados.mainline.provenance_git import capture_repository_identity
from acados.mainline.provenance_python import capture_python_runtime, module_file


def _run(command: list[str], *, cwd: Path | None = None) -> bytes:
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        capture_output=True,
    )
    return completed.stdout


def _write_executable(path: Path, body: str) -> None:
    path.write_text(body, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _tool(path: Path, role: str = "fixture"):
    return capture_tool_identity(role, str(path), (("version", ("--version",)),))


class MainlineProvenanceTest(unittest.TestCase):
    def test_top_level_import_is_lazy_about_optional_backends(self) -> None:
        script = """
import builtins
import sys

blocked = {"casadi", "numpy", "acados_template"}
original_import = builtins.__import__

def guarded_import(name, *args, **kwargs):
    if name.split(".", 1)[0] in blocked:
        raise AssertionError("optional backend imported during provenance import")
    return original_import(name, *args, **kwargs)

builtins.__import__ = guarded_import
import acados.mainline.provenance
assert not any(name.split(".", 1)[0] in blocked for name in sys.modules)
"""
        environment = dict(os.environ)
        environment["PYTHONPATH"] = os.pathsep.join(
            item for item in (str(SCRIPTS_ROOT), environment.get("PYTHONPATH")) if item
        )
        completed = subprocess.run(
            [sys.executable, "-c", script],
            check=False,
            env=environment,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_top_level_capture_rejects_invalid_compiler_environment_fail_closed(self) -> None:
        with self.assertRaisesRegex(ProvenanceError, "compiler environment"):
            provenance_module.capture_codegen_provenance(
                Path("/absolute/repository"),
                ("source.py",),
                (("CC", None), ("CC", None)),
                Path("/absolute/acados"),
                Path("/absolute/tera"),
            )
        with self.assertRaises(ProvenanceError):
            provenance_module.require_codegen_provenance(object())

    def test_read_stable_regular_file_and_final_symlink_policy(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "payload.bin"
            payload = b"stable payload\x00\n"
            target.write_bytes(payload)
            self.assertEqual(read_stable_regular_file(target, label="fixture"), payload)

            final_link = root / "payload-link"
            final_link.symlink_to(target)
            with self.assertRaises(IdentityError):
                read_stable_regular_file(final_link, label="fixture")
            self.assertEqual(
                read_stable_regular_file(final_link.resolve(), label="fixture"), payload
            )

            directory_alias = root / "directory-alias"
            directory_alias.symlink_to(target.parent, target_is_directory=True)
            with self.assertRaisesRegex(IdentityError, "symbolic-link directory"):
                read_stable_regular_file(directory_alias / target.name, label="fixture")

    def test_read_stable_regular_file_rejects_pathname_replacement_during_read(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "payload"
            target.write_bytes(b"payload")
            real_stat = identity_module.os.stat
            path_stat_calls = 0

            def replacing_stat(path, *args, **kwargs):
                nonlocal path_stat_calls
                result = real_stat(path, *args, **kwargs)
                if Path(path) == target and kwargs.get("follow_symlinks") is False:
                    path_stat_calls += 1
                    if path_stat_calls == 2:
                        fields = list(result)
                        fields[1] += 1
                        return identity_module.os.stat_result(fields)
                return result

            with patch.object(
                identity_module.os, "stat", side_effect=replacing_stat
            ), self.assertRaisesRegex(IdentityError, "changed while being read"):
                read_stable_regular_file(target, label="fixture")
            self.assertEqual(path_stat_calls, 2)

    def test_capture_linked_file_records_two_level_leaf_chain(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "real" / "payload"
            target.parent.mkdir()
            target.write_bytes(b"linked")
            second = root / "second-link"
            first = root / "first-link"
            second.symlink_to("real/payload")
            first.symlink_to(second.name)

            identity = capture_linked_file("fixture", first)
            self.assertEqual(identity.requested_path, str(first))
            self.assertEqual(identity.resolved_path, str(target.resolve()))
            self.assertEqual(
                tuple(item.target for item in identity.leaf_symlink_chain),
                ("second-link", "real/payload"),
            )
            self.assertEqual(
                tuple(item.path for item in identity.leaf_symlink_chain),
                (str(first), str(second)),
            )
            self.assertEqual(identity.size_bytes, len(b"linked"))

    def test_capture_linked_file_rejects_intermediate_directory_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            real_directory = root / "real"
            real_directory.mkdir()
            target = real_directory / "payload"
            target.write_bytes(b"linked")
            directory_alias = root / "directory-alias"
            directory_alias.symlink_to(real_directory, target_is_directory=True)
            with self.assertRaisesRegex(ProvenanceError, "symbolic-link directory"):
                capture_linked_file("fixture", directory_alias / "payload")

    def test_selected_sources_are_sorted_and_reject_path_ambiguity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "a.txt").write_bytes(b"a")
            (root / "nested").mkdir()
            (root / "nested" / "b.txt").write_bytes(b"b")
            identity = capture_selected_source_files(
                root,
                ("a.txt", "nested/b.txt"),
                logical_root="fixture",
            )
            self.assertEqual(
                tuple(item.relative_path for item in identity.files),
                ("a.txt", "nested/b.txt"),
            )

            invalid_paths = (
                ("nested/b.txt", "a.txt"),
                ("a.txt", "a.txt"),
                ("/absolute.txt",),
                ("../outside.txt",),
                ("nested/../a.txt",),
            )
            for paths in invalid_paths:
                with self.subTest(paths=paths), self.assertRaises(ProvenanceError):
                    capture_selected_source_files(root, paths, logical_root="fixture")

            outside = root / "outside-real"
            outside.mkdir()
            (outside / "file.txt").write_bytes(b"outside")
            (root / "alias").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ProvenanceError, "symlink"):
                capture_selected_source_files(
                    root, ("alias/file.txt",), logical_root="fixture"
                )

    def _new_git_repository(self) -> tuple[tempfile.TemporaryDirectory[str], Path, object, str]:
        git_executable = shutil.which("git")
        if git_executable is None:
            self.skipTest("git is unavailable")
        temporary = tempfile.TemporaryDirectory()
        root = Path(temporary.name)
        _run([git_executable, "init", "-b", "main"], cwd=root)
        _run([git_executable, "config", "user.email", "provenance@example.invalid"], cwd=root)
        _run([git_executable, "config", "user.name", "Provenance Test"], cwd=root)
        (root / "tracked.txt").write_text("tracked\n", encoding="utf-8")
        _run([git_executable, "add", "tracked.txt"], cwd=root)
        _run([git_executable, "commit", "-m", "fixture"], cwd=root)
        head = _run([git_executable, "rev-parse", "HEAD"], cwd=root).decode().strip()
        return temporary, root, _tool(Path(git_executable), "git"), head

    def test_temporary_git_clean_and_dirty_states_are_recorded(self) -> None:
        temporary, root, git_tool, head = self._new_git_repository()
        try:
            clean = capture_repository_identity(
                root,
                ("tracked.txt",),
                git_tool,
                expected_branch="main",
                base_sha=head,
            )
            self.assertTrue(clean.worktree_clean)
            self.assertEqual(clean.status_entry_count, 0)
            self.assertEqual(clean.dirty_policy, GIT_DIRTY_POLICY)

            (root / "untracked.txt").write_text("dirty\n", encoding="utf-8")
            dirty = capture_repository_identity(
                root,
                ("tracked.txt",),
                git_tool,
                expected_branch="main",
                base_sha=head,
            )
            self.assertFalse(dirty.worktree_clean)
            self.assertEqual(dirty.status_entry_count, 1)
            self.assertEqual(dirty.dirty_policy, GIT_DIRTY_POLICY)
            self.assertNotEqual(clean.status_porcelain_sha256, dirty.status_porcelain_sha256)
        finally:
            temporary.cleanup()

    def test_git_command_failure_is_fail_closed(self) -> None:
        temporary, root, git_tool, head = self._new_git_repository()
        try:
            with patch.object(
                git_module,
                "git_command",
                side_effect=ProvenanceError("fixture Git failure"),
            ), self.assertRaises(ProvenanceError):
                capture_repository_identity(
                    root,
                    ("tracked.txt",),
                    git_tool,
                    expected_branch="main",
                    base_sha=head,
                )
        finally:
            temporary.cleanup()

    def test_temporary_tool_path_and_fixed_version_are_hashed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tool_path = Path(directory) / "fixture-tool"
            _write_executable(
                tool_path,
                "#!/bin/sh\nprintf '%s\\n' 'fixture-tool version 7.2.1'\n",
            )
            identity = _tool(tool_path)
            self.assertEqual(identity.requested_command, str(tool_path))
            self.assertEqual(identity.executable.requested_path, str(tool_path))
            self.assertEqual(identity.probes[0].output_text, "fixture-tool version 7.2.1")
            self.assertEqual(identity, require_tool_identity(identity))

    def test_tool_probe_rejects_executable_replacement(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tool_path = root / "fixture-tool"
            replacement = root / "replacement-tool"
            _write_executable(
                replacement,
                "#!/bin/sh\nprintf '%s\\n' 'replacement version'\n",
            )
            _write_executable(
                tool_path,
                "#!/bin/sh\nprintf '%s\\n' 'original version'\n"
                f"cp {replacement} {tool_path}\n",
            )
            with self.assertRaisesRegex(ProvenanceError, "changed during probe"):
                _tool(tool_path)

    def test_tool_probe_failure_and_empty_output_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            failure = root / "failure-tool"
            _write_executable(failure, "#!/bin/sh\nexit 23\n")
            with self.assertRaises(ProvenanceError):
                _tool(failure)

            empty = root / "empty-tool"
            _write_executable(empty, "#!/bin/sh\nexit 0\n")
            with self.assertRaises(ProvenanceError):
                _tool(empty)

    def test_python_interface_is_lazy_until_runtime_capture_and_checks_tool_role(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tool_path = Path(directory) / "other-tool"
            _write_executable(tool_path, "#!/bin/sh\nprintf '%s\\n' 'other 1'\n")
            wrong_role = _tool(tool_path, "other")
            with self.assertRaisesRegex(ProvenanceError, "captured Python tool"):
                capture_python_runtime(wrong_role)

            module_path = Path(directory) / "module.py"
            module_path.write_text("# fixture\n", encoding="utf-8")
            self.assertEqual(module_file(SimpleNamespace(__file__=str(module_path)), "fixture"), module_path)
            module_link = Path(directory) / "module-link.py"
            module_link.symlink_to(module_path.name)
            self.assertEqual(
                module_file(SimpleNamespace(__file__=str(module_link)), "fixture"),
                module_link,
            )
            module_directory = Path(directory) / "module-directory"
            module_directory.mkdir()
            module_directory_link = Path(directory) / "module-directory-link"
            module_directory_link.symlink_to(module_directory, target_is_directory=True)
            with self.assertRaisesRegex(ProvenanceError, "symbolic-link directory"):
                module_file(
                    SimpleNamespace(__file__=str(module_directory_link / "module.py")),
                    "fixture",
                )
            with self.assertRaises(ProvenanceError):
                module_file(SimpleNamespace(), "missing")

    def test_codegen_provenance_requires_the_canonical_28_environment_entries(self) -> None:
        canonical = tuple((name, None) for name in COMPILER_ENVIRONMENT_NAMES)
        malformed = canonical[:-1]
        with self.assertRaisesRegex(ProvenanceError, "canonical 28"):
            provenance_module.capture_codegen_provenance(
                Path("/absolute/repository"),
                ("source.py",),
                malformed,
                Path("/absolute/acados"),
                Path("/absolute/tera"),
            )
        reordered = list(canonical)
        reordered[0], reordered[1] = reordered[1], reordered[0]
        with self.assertRaisesRegex(ProvenanceError, "exactly match"):
            provenance_module.capture_codegen_provenance(
                Path("/absolute/repository"),
                ("source.py",),
                tuple(reordered),
                Path("/absolute/acados"),
                Path("/absolute/tera"),
            )

    def test_acados_interface_fails_closed_without_touching_optional_imports(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            git_path = shutil.which("git")
            readelf_path = shutil.which("readelf")
            if git_path is None or readelf_path is None:
                self.skipTest("git/readelf unavailable")
            git_tool = _tool(Path(git_path), "git")
            readelf_tool = _tool(Path(readelf_path), "readelf")
            with self.assertRaises(ProvenanceError):
                capture_acados_install(root, git_tool, readelf_tool)

            with self.assertRaisesRegex(ProvenanceError, "git and readelf"):
                capture_acados_install(root, readelf_tool, readelf_tool)
            with self.assertRaises(ProvenanceError):
                acados_module._strict_json_bytes(b'{"x": 1, "x": 2}', "fixture")
            with self.assertRaises(ProvenanceError):
                acados_module._strict_json_bytes(b'{"x": NaN}', "fixture")

    def test_force_mutation_of_structure_and_hash_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "source.txt").write_bytes(b"source")
            source_tree = capture_selected_source_files(
                root, ("source.txt",), logical_root="fixture"
            )
            object.__setattr__(source_tree, "semantic_sha256", "0" * 64)
            with self.assertRaisesRegex(ProvenanceError, "inconsistent"):
                require_source_tree(source_tree)

            source_tree = capture_selected_source_files(
                root, ("source.txt",), logical_root="fixture"
            )
            object.__setattr__(source_tree, "files", list(source_tree.files))
            with self.assertRaisesRegex(ProvenanceError, "immutable non-empty inventory"):
                require_source_tree(source_tree)

            tool_path = root / "tool"
            _write_executable(tool_path, "#!/bin/sh\nprintf '%s\\n' 'tool 1'\n")
            tool = _tool(tool_path)
            object.__setattr__(tool, "semantic_sha256", "0" * 64)
            with self.assertRaisesRegex(ProvenanceError, "inconsistent"):
                require_tool_identity(tool)

    def test_complete_acados_capture_when_environment_is_explicitly_available(self) -> None:
        install_text = os.environ.get("ACADOS_SOURCE_DIR")
        tera_text = os.environ.get("TERA_PATH")
        if not install_text or not tera_text:
            self.skipTest("ACADOS_SOURCE_DIR and TERA_PATH are not both set")
        try:
            import acados_template
            import casadi  # noqa: F401
        except (ImportError, OSError):
            self.skipTest("CasADi/acados_template unavailable")
        install_root = Path(install_text)
        tera_path = Path(tera_text)
        expected_interface = install_root / "interfaces" / "acados_template" / "acados_template"
        marker_path = install_root / "lib" / "git_commit_hash"
        required = (
            install_root / "include",
            marker_path,
            install_root / "lib" / "link_libs.json",
            expected_interface,
            tera_path,
        )
        if not install_root.is_absolute() or not all(item.exists() for item in required):
            self.skipTest("Acados install is incomplete")
        if Path(acados_template.__file__).resolve().parent != expected_interface.resolve():
            self.skipTest("acados_template is not bound to ACADOS_SOURCE_DIR")
        source_repository = marker_path.resolve().parent.parent
        if not (source_repository / ".git").exists():
            self.skipTest("Acados source repository is unavailable")
        if shutil.which("git") is None or shutil.which("readelf") is None:
            self.skipTest("git/readelf unavailable")
        for library in acados_module.ACADOS_LIBRARY_NAMES:
            if not (install_root / "lib" / library).exists():
                self.skipTest("Acados libraries are incomplete")
        git_tool = _tool(Path(shutil.which("git") or "git"), "git")
        readelf_tool = _tool(Path(shutil.which("readelf") or "readelf"), "readelf")
        compiler_environment = tuple(
            (name, os.environ.get(name)) for name in COMPILER_ENVIRONMENT_NAMES
        )
        source_paths = (
            "src/scout_apps/control/spmpc_local_planner/scripts/acados/mainline/provenance.py",
        )
        provenance = capture_codegen_provenance(
            SCRIPTS_ROOT.parents[4],
            source_paths,
            compiler_environment,
            install_root,
            tera_path,
        )
        self.assertIs(require_codegen_provenance(provenance), provenance)
        identity = provenance.acados
        self.assertEqual(identity.source_head_sha, identity.source_head_sha.lower())
        self.assertTrue(identity.source_head_sha.startswith(identity.commit_marker))
        marker_payload = Path(identity.commit_marker_file.resolved_path).read_text(
            encoding="ascii"
        ).strip()
        self.assertEqual(identity.commit_marker, marker_payload)
        self.assertEqual(
            tuple(item.logical_name for item in identity.libraries),
            acados_module.ACADOS_LIBRARY_NAMES,
        )
        self.assertTrue(any(item.file.leaf_symlink_chain for item in identity.libraries))
        self.assertEqual(git_tool.role, "git")
        self.assertEqual(readelf_tool.role, "readelf")
        self.assertEqual(require_tool_identity(git_tool), git_tool)
        self.assertEqual(require_tool_identity(readelf_tool), readelf_tool)


if __name__ == "__main__":
    unittest.main()
