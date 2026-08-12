#!/usr/bin/env python3
"""Synthetic-only tests for S5B0 staging/case materializer v3."""

from __future__ import annotations

import ast
import copy
import json
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_motion_bridge_v1 as bridge  # noqa: E402
import r8_liquid_s5b0_staging_case_materializer_v3 as materializer  # noqa: E402


class S5B0StagingCaseMaterializerV3Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-s5b0-stage-v3-test-")
        self.base = Path(self.temporary.name)
        self.source_root = self.base / "sources"
        self.sources = materializer._write_fixture_sources(self.source_root)
        self.stage_number = 0

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def next_stage(self) -> Path:
        self.stage_number += 1
        return self.base / f"stage-{self.stage_number}.partial"

    def refresh(self) -> dict[str, dict]:
        return {
            role: materializer.observe_source(Path(self.sources[role]["path"]))
            for role in materializer.ROLES
        }

    def materialize(self, *, stage: Path | None = None, sources: dict | None = None, **changes: object) -> dict:
        stage = self.next_stage() if stage is None else stage
        arguments: dict[str, object] = {
            "expected_stage_root": stage,
            "sources": self.sources if sources is None else sources,
            "restart_part_index": 901,
            "settled_time_s": 45.05001991890928,
            "solver_tail_s": 1.0,
        }
        arguments.update(changes)
        return materializer.materialize(stage, **arguments)

    def write_solver_rows(self, rows: tuple[bridge.SolverPathRow, ...]) -> None:
        path = Path(self.sources["solver_path"]["path"])
        path.chmod(0o600)
        path.write_text(bridge.render_solver_path_csv(rows), encoding="utf-8")
        path.chmod(0o440)
        self.sources = self.refresh()

    def test_golden_o_excl_copy_and_two_exact_motion_blocks(self) -> None:
        manifest = self.materialize()
        schema = json.loads(materializer.SCHEMA_PATH.read_bytes())
        Draft202012Validator(schema).validate(manifest)
        stage = Path(manifest["stage_root"])
        self.assertEqual(
            {str(path.relative_to(stage)) for path in stage.rglob("*") if path.is_file()},
            {value[0] for value in materializer.DESTINATIONS.values()},
        )
        case = ET.fromstring((stage / "case/C1M_case.xml").read_bytes())
        motions = case.findall("./casedef/motion") + case.findall("./execution/motion")
        self.assertEqual(len(motions), 2)
        for motion in motions:
            path = motion.find("./objreal/mvpathfile")
            self.assertEqual(path.attrib["anglesunits"], "degrees")
            self.assertEqual(path.find("axes").attrib, {"value": "ZYX"})
            self.assertEqual(path.find("intrinsic").attrib, {"value": "true"})
            self.assertEqual(path.find("movecenter").attrib, {"value": "true"})
            self.assertEqual(path.find("file").attrib["fields"], "7")
        self.assertEqual(
            manifest["staged"]["solver_path"]["sha256"],
            manifest["sources"]["solver_path"]["sha256"],
        )
        self.assertNotEqual(
            manifest["staged"]["case_xml"]["sha256"],
            manifest["sources"]["case_xml"]["sha256"],
        )
        self.assertEqual(manifest["contract"]["tmax_s"], 47.05001991890928)

    def test_semantic_manifest_hash_is_root_and_inode_independent(self) -> None:
        first = self.materialize()
        second = self.materialize()
        self.assertNotEqual(first["stage_root"], second["stage_root"])
        self.assertNotEqual(first["staged"]["candidate"]["inode"], second["staged"]["candidate"]["inode"])
        self.assertEqual(
            first["integrity"]["semantic_manifest_sha256"],
            second["integrity"]["semantic_manifest_sha256"],
        )

    def test_external_roles_identity_and_candidate_disarm_fail_closed(self) -> None:
        changed = copy.deepcopy(self.sources)
        changed["candidate"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "identity differs"):
            self.materialize(sources=changed)
        changed = copy.deepcopy(self.sources)
        changed["extra"] = changed["candidate"]
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "role set"):
            self.materialize(sources=changed)
        candidate = Path(self.sources["candidate"]["path"])
        candidate.chmod(0o500)
        changed = self.refresh()
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "source candidate is executable"):
            self.materialize(sources=changed)

    def test_source_symlink_hardlink_and_special_are_rejected(self) -> None:
        alias = self.base / "candidate-link"
        alias.symlink_to(self.sources["candidate"]["path"])
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "symlink path component"):
            materializer.observe_source(alias)
        candidate = Path(self.sources["candidate"]["path"])
        hard = self.base / "candidate-hard"
        os.link(candidate, hard)
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "single-link"):
            materializer.observe_source(candidate)
        hard.unlink()
        fifo = self.base / "fifo"
        os.mkfifo(fifo)
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "single-link regular"):
            materializer.observe_source(fifo)

    def test_fresh_exact_root_and_extra_staged_entry_fail_closed(self) -> None:
        stage = self.next_stage()
        stage.mkdir()
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "not fresh"):
            self.materialize(stage=stage)
        with self.assertRaisesRegex(materializer.StagingMaterializerError, "exact .partial root"):
            self.materialize(stage=self.next_stage(), expected_stage_root=self.base / "different.partial")

        stage = self.next_stage()
        original = materializer._write_at
        count = 0

        def inject_extra(*args: object, **kwargs: object) -> dict:
            nonlocal count
            result = original(*args, **kwargs)
            count += 1
            if count == len(materializer.ROLES):
                (stage / "extra.txt").write_text("extra", encoding="utf-8")
            return result

        with mock.patch.object(materializer, "_write_at", side_effect=inject_extra):
            with self.assertRaisesRegex(materializer.StagingMaterializerError, "missing or extra"):
                self.materialize(stage=stage)

    def test_solver_path_t0_tail_fields_and_finite_fail_closed(self) -> None:
        path = Path(self.sources["solver_path"]["path"])
        invalid_texts = (
            "1,0,0,0,0,0,0\n2,0,0,0,0,0,0\n3,0,0,0,0,0,0\n",
            "0,0,0,0,0,0\n1,0,0,0,0,0\n2,0,0,0,0,0\n",
            "0,0,0,0,0,0,0\n1,0.1,0,0,0,0,0\n2,0.2,0,0,0,0,0\n",
            "0,0,0,0,0,0,0\n1,nan,0,0,0,0,0\n2,nan,0,0,0,0,0\n",
        )
        for text in invalid_texts:
            with self.subTest(text=text):
                path.chmod(0o600)
                path.write_text(text, encoding="utf-8")
                path.chmod(0o440)
                self.sources = self.refresh()
                with self.assertRaisesRegex(materializer.StagingMaterializerError, "solver_path|tail"):
                    self.materialize()

    def test_case_cardinality_declaration_and_source_toctou_fail_closed(self) -> None:
        case = Path(self.sources["case_xml"]["path"])
        original_case = materializer._fixture_case_xml()
        for payload, message in (
            (original_case.replace(b"<motion>", b"<removed>", 1).replace(b"</motion>", b"</removed>", 1), "two motion"),
            (b"<!DOCTYPE case><case><casedef/><execution/></case>", "declarations"),
        ):
            case.chmod(0o600)
            case.write_bytes(payload)
            case.chmod(0o440)
            self.sources = self.refresh()
            with self.assertRaisesRegex(materializer.StagingMaterializerError, message):
                self.materialize()
        case.chmod(0o600)
        case.write_bytes(original_case)
        case.chmod(0o440)
        self.sources = self.refresh()

        stage = self.next_stage()
        original_write = materializer._write_at
        changed = False

        def mutate_source(*args: object, **kwargs: object) -> dict:
            nonlocal changed
            result = original_write(*args, **kwargs)
            if not changed:
                changed = True
                source = Path(self.sources["dsph_config"]["path"])
                source.chmod(0o600)
                source.write_bytes(b"changed after source read")
            return result

        with mock.patch.object(materializer, "_write_at", side_effect=mutate_source):
            with self.assertRaisesRegex(materializer.StagingMaterializerError, "TOCTOU"):
                self.materialize(stage=stage)

    def test_schema_ast_and_self_check_are_closed_fixture_only(self) -> None:
        schema = json.loads(materializer.SCHEMA_PATH.read_bytes())
        Draft202012Validator.check_schema(schema)
        materializer.assert_deep_closed(schema)
        ast.parse(Path(materializer.__file__).read_text(encoding="utf-8"))
        source = Path(materializer.__file__).read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        manifest = self.materialize()
        changed = copy.deepcopy(manifest)
        changed["claims"]["solver_executed"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)
        result = materializer.self_check()
        self.assertEqual(result["status"], "PASS_S5B0_STAGING_CASE_MATERIALIZER_V3_FIXTURE_SELF_CHECK")
        self.assertTrue(result["fixture_root_removed"])
        for key in (
            "real_external_input_read", "real_bag_read", "source_candidate_executed",
            "staged_candidate_executed", "solver_executed", "gpu_exposed",
            "network_used", "sudo_used", "apparmor_loaded", "files_written_outside_fixture",
        ):
            self.assertFalse(result[key], key)


if __name__ == "__main__":
    unittest.main()
