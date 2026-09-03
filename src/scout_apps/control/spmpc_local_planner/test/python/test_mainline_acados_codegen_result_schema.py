"""Offline contract tests for the Stage 3-D4 generated-result schema."""

from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_ROOT = PACKAGE_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.acados_codegen_result_schema import (
    ACADOS_CODEGEN_ARTIFACT_CLASS,
    ACADOS_CODEGEN_PERFORMANCE_STATUS,
    ACADOS_CODEGEN_PROMOTION_STATUS,
    ACADOS_CODEGEN_RESULT_SCHEMA,
    ACADOS_CODEGEN_RESULT_SCOPE,
    ACADOS_CODEGEN_STATUS,
    CODEGEN_FAILURE_OUTPUT_POLICY,
    OUTPUT_ROOT_IDENTITY_POLICY,
    REQUIRED_SOLVER_SYMBOLS,
    SOLVER_LIBRARY_FORMAT,
    SOLVER_LOAD_CHECK_POLICY,
    AcadosCodegenResultSchemaError,
    validate_acados_codegen_result_document,
)
from acados.mainline.artifact_files import (
    SOLVER_LIBRARY_BASENAMES,
    generated_tree_sha256,
    inventory_generated_tree,
    solver_library_record,
)
from acados.mainline.identity import sha256_json
from acados.mainline.model_contract import MODEL_ID


def _write_generated_fixture(root: Path) -> None:
    (root / "model").mkdir(parents=True)
    (root / f"acados_ocp_{MODEL_ID}.json").write_text(
        '{"model":"mainline"}\n', encoding="utf-8"
    )
    (root / "Makefile").write_text("ocp_shared_lib:\n\t@true\n", encoding="utf-8")
    (root / "model" / "model.c").write_text(
        "int model(void) { return 0; }\n", encoding="utf-8"
    )
    (root / "model" / "model.h").write_text("int model(void);\n", encoding="utf-8")
    (root / SOLVER_LIBRARY_BASENAMES[0]).write_bytes(b"ELF-mainline-solver")


def _result_document(root: Path) -> dict[str, object]:
    _write_generated_fixture(root)
    records = inventory_generated_tree(root)
    library = solver_library_record(records)
    payload = {
        "schema_version": ACADOS_CODEGEN_RESULT_SCHEMA,
        "scope": ACADOS_CODEGEN_RESULT_SCOPE,
        "model_id": MODEL_ID,
        "status": {
            "codegen": ACADOS_CODEGEN_STATUS,
            "artifact_class": ACADOS_CODEGEN_ARTIFACT_CLASS,
            "promotion": ACADOS_CODEGEN_PROMOTION_STATUS,
            "target_performance": ACADOS_CODEGEN_PERFORMANCE_STATUS,
        },
        "output_directory": OUTPUT_ROOT_IDENTITY_POLICY,
        "failure_output_policy": CODEGEN_FAILURE_OUTPUT_POLICY,
        "generated_tree": {
            "sha256": generated_tree_sha256(records),
            "files": [record.to_dict() for record in records],
        },
        "solver_library": {
            "relative_path": library.relative_path,
            "size_bytes": library.size_bytes,
            "raw_sha256": library.raw_sha256,
            "format": SOLVER_LIBRARY_FORMAT,
            "elf_class": 64,
            "elf_machine": 62,
            "required_exported_symbols": list(REQUIRED_SOLVER_SYMBOLS),
            "load_check": SOLVER_LOAD_CHECK_POLICY,
        },
    }
    return {
        **payload,
        "semantic_identity": {
            "sha256": sha256_json(payload),
            "scope": ACADOS_CODEGEN_RESULT_SCOPE,
        },
    }


def _resign(document: dict[str, object]) -> None:
    payload = {
        key: value for key, value in document.items() if key != "semantic_identity"
    }
    identity = document["semantic_identity"]
    assert type(identity) is dict
    identity["sha256"] = sha256_json(payload)


class MainlineAcadosCodegenResultSchemaTest(unittest.TestCase):
    def test_canonical_document_round_trips_without_numeric_backends(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            document = _result_document(Path(directory))
            self.assertIs(validate_acados_codegen_result_document(document), document)

    def test_resigned_policy_and_elf_mutations_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            baseline = _result_document(Path(directory))
        mutations = (
            ("output_directory", "ABSOLUTE_PATH_INCLUDED"),
            ("failure_output_policy", "PARTIAL_OUTPUT_PROMOTED"),
            ("solver_library.format", "MACH_O"),
            ("solver_library.elf_machine", 999),
            ("solver_library.load_check", "SKIPPED"),
        )
        for path, replacement in mutations:
            forged = copy.deepcopy(baseline)
            if path.startswith("solver_library."):
                field = path.split(".", 1)[1]
                solver_library = forged["solver_library"]
                assert type(solver_library) is dict
                solver_library[field] = replacement
            else:
                forged[path] = replacement
            _resign(forged)
            with self.subTest(path=path), self.assertRaises(
                AcadosCodegenResultSchemaError
            ):
                validate_acados_codegen_result_document(forged)

        forged = copy.deepcopy(baseline)
        solver_library = forged["solver_library"]
        assert type(solver_library) is dict
        symbols = solver_library["required_exported_symbols"]
        assert type(symbols) is list
        symbols[-1] += "_tampered"
        _resign(forged)
        with self.assertRaises(AcadosCodegenResultSchemaError):
            validate_acados_codegen_result_document(forged)


if __name__ == "__main__":
    unittest.main()
