"""Unit tests for the dependency-light Stage 3-D4 provenance interfaces."""

from __future__ import annotations

import copy
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
from acados.mainline.provenance_schema import (
    BUILD_COMMANDS,
    GENERATOR_API,
    GENERATOR_ARGUMENTS,
    PROVENANCE_SCOPE,
    STAGING_LOCATION_POLICY,
    TOOL_ROLES,
    validate_codegen_provenance_document,
)


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


def _semantic(payload: dict, scope: str | None = None) -> dict:
    value = dict(payload)
    identity = {"sha256": identity_module.sha256_json(payload)}
    if scope is not None:
        identity["scope"] = scope
    value["semantic_identity"] = identity
    return value


def _serialized_provenance_fixture() -> dict:
    """Build a complete dependency-free document for schema tamper tests."""

    def linked(
        name: str,
        executable: bool = False,
        *,
        path: str | None = None,
    ) -> dict:
        path = path or f"/fixture/{name}"
        return {
            "schema_version": "spmpc_mainline_linked_file_identity_v1",
            "logical_name": name,
            "requested_path": path,
            "resolved_path": path,
            "leaf_symlink_chain": [],
            "size_bytes": 1,
            "raw_sha256": "1" * 64,
            "executable": executable,
        }

    def source_tree(name: str) -> dict:
        return _semantic(
            {
                "schema_version": "spmpc_mainline_source_tree_identity_v1",
                "logical_root": name,
                "capture_root": "ABSOLUTE_CAPTURE_ROOT_EXCLUDED_FROM_IDENTITY",
                "files": [
                    {
                        "relative_path": "fixture.py",
                        "size_bytes": 1,
                        "raw_sha256": "2" * 64,
                    }
                ],
            }
        )

    repository = _semantic(
        {
            "schema_version": "spmpc_mainline_repository_identity_v1",
            "repository_root": "ABSOLUTE_CAPTURE_ROOT_EXCLUDED_FROM_IDENTITY",
            "branch": "spmpc-mainline",
            "head_sha": "c" * 40,
            "base_sha": git_module.MAINLINE_BASE_SHA,
            "worktree": {
                "clean": True,
                "dirty_policy": "RECORDED_NOT_GATED",
                "status_entry_count": 0,
                "status_porcelain_sha256": "3" * 64,
            },
            "sources": source_tree("MAINLINE_REPOSITORY_SELECTED_SOURCES"),
        }
    )

    probe_arguments = {
        "git": [("version", ["--version"])],
        "python": [("version", ["--version"])],
        "tera": [("version", ["--version"])],
        "make": [("version", ["--version"])],
        "nm": [("version", ["--version"])],
        "readelf": [("version", ["--version"])],
        "cc": [
            ("version", ["--version"]),
            ("target", ["-dumpmachine"]),
            ("full_version", ["-dumpfullversion", "-dumpversion"]),
        ],
        "cxx": [
            ("version", ["--version"]),
            ("target", ["-dumpmachine"]),
            ("full_version", ["-dumpfullversion", "-dumpversion"]),
        ],
        "ar": [("version", ["--version"])],
        "ranlib": [("version", ["--version"])],
    }
    tools = []
    requested_commands = {
        "git": "git",
        "python": "/fixture/tool-python",
        "tera": "/fixture/tool-tera",
        "make": "make",
        "nm": "nm",
        "readelf": "readelf",
        "cc": "cc",
        "cxx": "c++",
        "ar": "ar",
        "ranlib": "ranlib",
    }
    for role in TOOL_ROLES:
        tool_payload = {
            "schema_version": "spmpc_mainline_tool_identity_v1",
            "role": role,
            "requested_command": requested_commands[role],
            "executable": linked(
                f"tool:{role}",
                executable=True,
                path=(
                    requested_commands[role]
                    if requested_commands[role].startswith("/")
                    else f"/fixture/bin/{role}"
                ),
            ),
            "probes": [
                {
                    "name": name,
                    "arguments": arguments,
                    "output_text": "fixture version",
                    "output_raw_sha256": "4" * 64,
                }
                for name, arguments in probe_arguments[role]
            ],
        }
        tools.append(_semantic(tool_payload))

    python_runtime = _semantic(
        {
            "schema_version": "spmpc_mainline_python_runtime_identity_v1",
            "implementation": "CPython",
            "version": "3.fixture",
            "version_info": [3, 11, 0],
            "executable_tool_sha256": tools[1]["semantic_identity"]["sha256"],
            "sys_prefix": "/fixture/python",
            "sys_base_prefix": "/fixture/python",
            "sys_path": ["/fixture/python/lib"],
            "PYTHONPATH": None,
            "packages": [
                {
                    "name": "casadi",
                    "version": "3.fixture",
                    "files": [
                        linked("casadi/__init__.py"),
                        linked("casadi/_casadi.so"),
                        linked("casadi/libcasadi.so"),
                    ],
                },
                {
                    "name": "numpy",
                    "version": "2.fixture",
                    "files": [
                        linked("numpy/__init__.py"),
                        linked("numpy/core/_multiarray_umath.so"),
                    ],
                },
            ],
        }
    )

    acados_libraries = []
    sonames = {
        "libacados.so": "libacados.so",
        "libhpipm.so": "libhpipm.so",
        "libblasfeo.so": "libblasfeo.so.0",
    }
    for name, soname in sonames.items():
        acados_libraries.append(
            {
                "logical_name": name,
                "file": linked(f"acados/lib/{name}"),
                "soname": soname,
                "needed": [],
                "rpath": None,
                "runpath": None,
                "dynamic_section_sha256": "5" * 64,
            }
        )
    acados = _semantic(
        {
            "schema_version": "spmpc_mainline_acados_install_identity_v1",
            "install_root": "/fixture/acados",
            "install_prefix_policy": "ABSOLUTE_ACADOS_PREFIX_EMBEDDED_IN_GENERATED_TREE",
            "source_repository": {
                "root": "/fixture/acados",
                "head_sha": "a" * 40,
                "exact_tag": "v0.5.4",
                "worktree_clean": True,
                "dirty_policy": "RECORDED_NOT_GATED",
                "status_porcelain_sha256": "6" * 64,
            },
            "commit_marker": "aaaaaaa",
            "commit_marker_file": linked("acados/lib/git_commit_hash"),
            "link_libs": {
                "file": linked("acados/lib/link_libs.json"),
                "canonical_json_sha256": "7" * 64,
            },
            "interface_source_binding_status": "MATCHED_SOURCE_ROOT",
            "tera_source_binding_status": "BINARY_AND_SUBMODULE_IDENTITIES_RECORDED_SEPARATELY",
            "interface_tree": source_tree("ACADOS_TEMPLATE_PYTHON_AND_TEMPLATES"),
            "include_tree": source_tree("ACADOS_INSTALLED_INCLUDE_TREE"),
            "submodules": [
                {
                    "path": path,
                    "commit_sha": "b" * 40,
                    "initialized": True,
                    "worktree_matches_index": True,
                }
                for path in sorted(
                    (
                        "external/blasfeo",
                        "external/hpipm",
                        "interfaces/acados_template/tera_renderer",
                    )
                )
            ],
            "libraries": acados_libraries,
        }
    )

    environment = {name: None for name in COMPILER_ENVIRONMENT_NAMES}
    return _semantic(
        {
            "schema_version": "spmpc_mainline_codegen_provenance_v1",
            "scope": PROVENANCE_SCOPE,
            "status": {
                "provenance": "CAPTURED_FOR_DEV_UNVALIDATED",
                "artifact_class": "DEV_UNVALIDATED",
                "promotion": "NOT_PROMOTED",
            },
            "repository": repository,
            "compiler_environment": environment,
            "tools": tools,
            "python_runtime": python_runtime,
            "acados": acados,
            "host": {
                "system": "Linux",
                "release": "fixture",
                "machine": "x86_64",
                "libc": ["glibc", "2.fixture"],
                "byteorder": "little",
            },
            "logical_codegen_commands": {
                "generator_api": GENERATOR_API,
                "generator_arguments": dict(GENERATOR_ARGUMENTS),
                "build_commands": [list(item) for item in BUILD_COMMANDS],
                "staging_location": STAGING_LOCATION_POLICY,
            },
        },
        PROVENANCE_SCOPE,
    )


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

    def test_resigned_serialized_policy_tampering_is_rejected(self) -> None:
        baseline = _serialized_provenance_fixture()
        self.assertIs(validate_codegen_provenance_document(baseline), baseline)

        def resign(document: dict) -> None:
            payload = {
                key: value
                for key, value in document.items()
                if key != "semantic_identity"
            }
            document["semantic_identity"]["sha256"] = identity_module.sha256_json(
                payload
            )

        tamper_cases = {
            "generator API": lambda document: document["logical_codegen_commands"].update(
                generator_api="AcadosOcpSolver.generate.forged"
            ),
            "repository branch": lambda document: document["repository"].update(
                branch="not-mainline"
            ),
            "tool probe arguments": lambda document: document["tools"][0][
                "probes"
            ][0].update(arguments=["--help"]),
            "compiler command syntax": lambda document: (
                document["compiler_environment"].update(CC="cc --forged"),
                document["tools"][6].update(requested_command="cc --forged"),
            ),
            "Acados install policy": lambda document: document["acados"].update(
                install_prefix_policy="FORGED_POLICY"
            ),
        }
        for label, mutate in tamper_cases.items():
            with self.subTest(label=label):
                forged = copy.deepcopy(baseline)
                mutate(forged)
                if label == "repository branch":
                    resign(forged["repository"])
                elif label == "tool probe arguments":
                    resign(forged["tools"][0])
                elif label == "compiler command syntax":
                    resign(forged["tools"][6])
                elif label == "Acados install policy":
                    resign(forged["acados"])
                resign(forged)
                with self.assertRaises(ProvenanceError):
                    validate_codegen_provenance_document(forged)

    def test_resigned_serialized_symlink_chain_tampering_is_rejected(self) -> None:
        baseline = _serialized_provenance_fixture()

        def resign(document: dict) -> None:
            payload = {
                key: value
                for key, value in document.items()
                if key != "semantic_identity"
            }
            document["semantic_identity"]["sha256"] = identity_module.sha256_json(
                payload
            )

        def resign_runtime_and_root(document: dict) -> None:
            resign(document["python_runtime"])
            resign(document)

        target = baseline["python_runtime"]["packages"][0]["files"][0]
        requested_path = target["requested_path"]
        resolved_path = "/fixture/casadi/real-init.py"
        target["resolved_path"] = resolved_path
        target["leaf_symlink_chain"] = [
            {"path": requested_path, "target": "real-init.py"}
        ]
        resign_runtime_and_root(baseline)
        self.assertIs(validate_codegen_provenance_document(baseline), baseline)

        tamper_cases = {
            "empty chain with distinct paths": lambda file: file.update(
                leaf_symlink_chain=[]
            ),
            "first hop differs from requested path": lambda file: file.update(
                leaf_symlink_chain=[
                    {"path": "/fixture/not-requested", "target": resolved_path}
                ]
            ),
            "hop target differs from resolved path": lambda file: file.update(
                leaf_symlink_chain=[
                    {"path": requested_path, "target": "/fixture/elsewhere"}
                ]
            ),
        }
        for label, mutate in tamper_cases.items():
            with self.subTest(label=label):
                forged = copy.deepcopy(baseline)
                file = forged["python_runtime"]["packages"][0]["files"][0]
                mutate(file)
                resign_runtime_and_root(forged)
                with self.assertRaises(ProvenanceError):
                    validate_codegen_provenance_document(forged)

    def test_resigned_serialized_logical_identity_tampering_is_rejected(self) -> None:
        baseline = _serialized_provenance_fixture()

        def resign(document: dict) -> None:
            payload = {
                key: value
                for key, value in document.items()
                if key != "semantic_identity"
            }
            document["semantic_identity"]["sha256"] = identity_module.sha256_json(
                payload
            )

        def resign_all(document: dict) -> None:
            resign(document["repository"]["sources"])
            resign(document["repository"])
            for tool in document["tools"]:
                resign(tool)
            document["python_runtime"]["executable_tool_sha256"] = document["tools"][
                1
            ]["semantic_identity"]["sha256"]
            resign(document["python_runtime"])
            resign(document["acados"]["interface_tree"])
            resign(document["acados"]["include_tree"])
            resign(document["acados"])
            resign(document)

        tamper_cases = {
            "repository source role": lambda document: document["repository"][
                "sources"
            ].update(logical_root="FORGED_REPOSITORY_SOURCES"),
            "tool executable role": lambda document: document["tools"][0][
                "executable"
            ].update(logical_name="tool:forged"),
            "CasADi package file role": lambda document: document["python_runtime"][
                "packages"
            ][0]["files"][0].update(logical_name="casadi/forged.py"),
            "NumPy package file role": lambda document: document["python_runtime"][
                "packages"
            ][1]["files"][1].update(logical_name="numpy/forged.so"),
            "Acados marker role": lambda document: document["acados"][
                "commit_marker_file"
            ].update(logical_name="acados/lib/forged_marker"),
            "Acados marker install binding": lambda document: document["acados"][
                "commit_marker_file"
            ].update(requested_path="/fixture/elsewhere/git_commit_hash"),
            "Acados marker source binding": lambda document: document["acados"][
                "commit_marker_file"
            ].update(resolved_path="/fixture/elsewhere/lib/git_commit_hash"),
            "Acados link-libs role": lambda document: document["acados"][
                "link_libs"
            ]["file"].update(logical_name="acados/lib/forged.json"),
            "Acados link-libs install binding": lambda document: document["acados"][
                "link_libs"
            ]["file"].update(requested_path="/fixture/elsewhere/link_libs.json"),
            "Acados interface-tree role": lambda document: document["acados"][
                "interface_tree"
            ].update(logical_root="FORGED_INTERFACE_TREE"),
            "Acados include-tree role": lambda document: document["acados"][
                "include_tree"
            ].update(logical_root="FORGED_INCLUDE_TREE"),
            "Acados library-file role": lambda document: document["acados"][
                "libraries"
            ][0]["file"].update(logical_name="acados/lib/forged.so"),
            "Acados library install binding": lambda document: document["acados"][
                "libraries"
            ][0]["file"].update(requested_path="/fixture/elsewhere/libacados.so"),
        }
        for label, mutate in tamper_cases.items():
            with self.subTest(label=label):
                forged = copy.deepcopy(baseline)
                mutate(forged)
                resign_all(forged)
                with self.assertRaises(ProvenanceError):
                    validate_codegen_provenance_document(forged)

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
