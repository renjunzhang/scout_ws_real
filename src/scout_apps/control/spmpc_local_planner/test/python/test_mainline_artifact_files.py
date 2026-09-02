"""Tests for safe Stage 3-D4 generated-tree inventory and raw hashes."""

from __future__ import annotations

import ast
import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.artifact_files import (
    ACADOS_JSON_ROLE,
    BUILD_RECIPE_ROLE,
    SOLVER_LIBRARY_BASENAMES,
    SOLVER_LIBRARY_ROLE,
    ArtifactFilesError,
    GeneratedFileRecord,
    generated_file_records_from_dict,
    generated_tree_sha256,
    inventory_generated_tree,
    solver_library_record,
    validate_generated_tree,
)
from acados.mainline.codegen_options import (
    ACADOS_JSON_FILENAME,
    GENERATED_HEADER_FILENAME,
    MODEL_CONTRACT_FILENAME,
)
from acados.mainline.identity import sha256_bytes


class MainlineArtifactFilesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        (self.root / ACADOS_JSON_FILENAME).write_bytes(b'{"model":"mainline"}\n')
        (self.root / "Makefile").write_bytes(b"ocp_shared_lib:\n\t@true\n")
        (self.root / "model").mkdir()
        (self.root / "model" / "disc_dyn.c").write_bytes(b"int dyn(void){return 0;}\n")
        (self.root / SOLVER_LIBRARY_BASENAMES[0]).write_bytes(b"ELF-mainline-solver")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_inventory_is_sorted_root_independent_and_fully_hashed(self) -> None:
        records = inventory_generated_tree(self.root)
        paths = tuple(item.relative_path for item in records)
        self.assertEqual(paths, tuple(sorted(paths)))
        self.assertEqual(len(records), 4)
        self.assertEqual(sum(item.role == ACADOS_JSON_ROLE for item in records), 1)
        self.assertEqual(sum(item.role == BUILD_RECIPE_ROLE for item in records), 1)
        library = solver_library_record(records)
        self.assertEqual(library.role, SOLVER_LIBRARY_ROLE)
        self.assertEqual(library.size_bytes, len(b"ELF-mainline-solver"))
        self.assertEqual(library.raw_sha256, sha256_bytes(b"ELF-mainline-solver"))
        first_digest = generated_tree_sha256(records)

        with tempfile.TemporaryDirectory() as other_directory:
            other = Path(other_directory)
            for record in records:
                destination = other / record.relative_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes((self.root / record.relative_path).read_bytes())
            self.assertEqual(
                generated_tree_sha256(inventory_generated_tree(other)),
                first_digest,
            )

    def test_contract_outputs_are_excluded_to_avoid_hash_cycles(self) -> None:
        baseline = inventory_generated_tree(self.root)
        (self.root / MODEL_CONTRACT_FILENAME).write_bytes(b"manifest")
        (self.root / GENERATED_HEADER_FILENAME).write_bytes(b"header")
        self.assertEqual(inventory_generated_tree(self.root), baseline)

    def test_tamper_missing_extra_and_symlink_fail_closed(self) -> None:
        baseline = inventory_generated_tree(self.root)
        (self.root / "model" / "disc_dyn.c").write_bytes(b"tampered")
        with self.assertRaises(ArtifactFilesError):
            validate_generated_tree(self.root, baseline)
        (self.root / "model" / "disc_dyn.c").write_bytes(b"int dyn(void){return 0;}\n")
        (self.root / "extra.txt").write_bytes(b"extra")
        with self.assertRaises(ArtifactFilesError):
            validate_generated_tree(self.root, baseline)
        (self.root / "extra.txt").unlink()
        (self.root / ACADOS_JSON_FILENAME).unlink()
        with self.assertRaises(ArtifactFilesError):
            inventory_generated_tree(self.root)
        (self.root / ACADOS_JSON_FILENAME).symlink_to("Makefile")
        with self.assertRaises(ArtifactFilesError):
            inventory_generated_tree(self.root)

    def test_relative_intermediate_symlink_and_enumeration_error_fail_closed(
        self,
    ) -> None:
        with self.assertRaises(ArtifactFilesError):
            inventory_generated_tree(Path("relative/generated"))

        with tempfile.TemporaryDirectory() as directory:
            outer = Path(directory)
            real_parent = outer / "real"
            generated = real_parent / "generated"
            generated.mkdir(parents=True)
            (generated / ACADOS_JSON_FILENAME).write_bytes(b"{}\n")
            (generated / "Makefile").write_bytes(b"all:\n\t@true\n")
            (generated / SOLVER_LIBRARY_BASENAMES[0]).write_bytes(b"ELF")
            (outer / "alias").symlink_to(real_parent, target_is_directory=True)
            with self.assertRaises(ArtifactFilesError):
                inventory_generated_tree(outer / "alias" / "generated")

        with (
            patch(
                "acados.mainline.artifact_files.os.scandir",
                side_effect=PermissionError("denied"),
            ),
            self.assertRaises(ArtifactFilesError),
        ):
            inventory_generated_tree(self.root)

    def test_serialized_inventory_rejects_path_role_order_and_type_drift(self) -> None:
        serialized = [item.to_dict() for item in inventory_generated_tree(self.root)]
        self.assertEqual(
            generated_file_records_from_dict(serialized),
            inventory_generated_tree(self.root),
        )
        mutations = []
        unknown = copy.deepcopy(serialized)
        unknown[0]["unknown"] = True
        mutations.append(unknown)
        absolute = copy.deepcopy(serialized)
        absolute[0]["relative_path"] = "/escape"
        mutations.append(absolute)
        traversal = copy.deepcopy(serialized)
        traversal[0]["relative_path"] = "../escape"
        mutations.append(traversal)
        boolean_size = copy.deepcopy(serialized)
        boolean_size[0]["size_bytes"] = True
        mutations.append(boolean_size)
        wrong_role = copy.deepcopy(serialized)
        wrong_role[0]["role"] = "FORGED"
        mutations.append(wrong_role)
        reverse_order = list(reversed(copy.deepcopy(serialized)))
        mutations.append(reverse_order)
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(ArtifactFilesError):
                generated_file_records_from_dict(value)

    def test_record_cannot_be_hand_built(self) -> None:
        with self.assertRaises(ArtifactFilesError):
            GeneratedFileRecord(
                relative_path="file",
                size_bytes=1,
                raw_sha256="a" * 64,
                role="BUILD_OUTPUT",
            )

    def test_module_has_no_backend_evidence_or_legacy_dependency(self) -> None:
        source = SCRIPTS_ROOT / "acados" / "mainline" / "artifact_files.py"
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        imported = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.append(node.module)
        self.assertFalse(
            any(
                name in {"casadi", "numpy", "acados_template"}
                or "legacy" in name
                or name.endswith(("stage1_evidence", "contract_source", "manifest"))
                for name in imported
            )
        )


if __name__ == "__main__":
    unittest.main()
