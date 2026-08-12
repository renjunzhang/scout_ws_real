#!/usr/bin/env python3
"""Synthetic-only tests for the S5B0 finalized frame reader v1."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5b0_finalized_frame_reader_v1 as reader  # noqa: E402


class S5B0FinalizedFrameReaderV1Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-s5b0-finalized-reader-test-")
        self.base = Path(self.temporary.name)
        self.output = self.base / "output"
        self.times = [1.0, 1.05]
        self.inventory = reader._write_fixture_tree(self.output, [7, 8], self.times)
        self.ids_sha = reader.ids_sha256([0, 1, 2, 3])

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def freeze_inventory(self) -> dict[str, str]:
        result: dict[str, str] = {}
        for path in sorted(self.output.rglob("*")):
            if path.is_file() and not path.is_symlink():
                result[str(path.relative_to(self.output))] = hashlib.sha256(path.read_bytes()).hexdigest()
        return result

    def build(self, **changes: object) -> dict:
        arguments: dict[str, object] = {
            "expected_root": self.output,
            "expected_inventory": self.inventory,
            "expected_start_index": 7,
            "expected_times_s": self.times,
            "expected_particle_count": 4,
            "expected_ids_sha256": self.ids_sha,
        }
        arguments.update(changes)
        return reader.build_manifest(self.output, **arguments)

    def test_golden_manifest_binds_exact_inventory_and_frames(self) -> None:
        manifest = self.build()
        schema = json.loads(reader.SCHEMA_PATH.read_bytes())
        Draft202012Validator(schema).validate(manifest)
        self.assertEqual([frame["index"] for frame in manifest["frames"]], [7, 8])
        self.assertEqual([frame["time_s"] for frame in manifest["frames"]], self.times)
        self.assertEqual(
            [item["relative_path"] for item in manifest["bindings"]["run_files"]],
            list(reader.RUN_FILES),
        )
        self.assertEqual(manifest["bindings"]["part_head"]["relative_path"], "data/Part_Head.ibi4")
        self.assertTrue(all(item["nlink"] == 1 for item in manifest["inventory"]["files"]))
        self.assertEqual(manifest, self.build())

    def test_exact_root_missing_extra_and_malformed_part_fail_closed(self) -> None:
        with self.assertRaisesRegex(reader.FinalizedFrameError, "exact external root"):
            self.build(expected_root=self.base)
        (self.output / "extra.txt").write_text("extra", encoding="utf-8")
        with self.assertRaisesRegex(reader.FinalizedFrameError, "missing or extra"):
            self.build()
        (self.output / "extra.txt").unlink()
        (self.output / "Run.csv").unlink()
        with self.assertRaisesRegex(reader.FinalizedFrameError, "missing or extra"):
            self.build()
        (self.output / "Run.csv").write_bytes(b"fixture run csv\n")
        malformed = self.output / "data/Part_8.bi4"
        (self.output / "data/Part_0008.bi4").rename(malformed)
        with self.assertRaisesRegex(reader.FinalizedFrameError, "malformed Part"):
            self.build(expected_inventory=self.freeze_inventory())

    def test_symlink_hardlink_special_and_symlink_root_fail_closed(self) -> None:
        alias = self.base / "alias"
        alias.symlink_to(self.output, target_is_directory=True)
        with self.assertRaisesRegex(reader.FinalizedFrameError, "symlink root component"):
            reader.build_manifest(
                alias,
                expected_root=alias,
                expected_inventory=self.inventory,
                expected_start_index=7,
                expected_times_s=self.times,
                expected_particle_count=4,
                expected_ids_sha256=self.ids_sha,
            )
        (self.output / "linked").symlink_to("Run.out")
        with self.assertRaisesRegex(reader.FinalizedFrameError, "symlink output entry"):
            self.build()
        (self.output / "linked").unlink()
        os.link(self.output / "Run.out", self.output / "hard")
        with self.assertRaisesRegex(reader.FinalizedFrameError, "hard-linked"):
            self.build()
        (self.output / "hard").unlink()
        os.mkfifo(self.output / "fifo")
        with self.assertRaisesRegex(reader.FinalizedFrameError, "special output entry"):
            self.build()

    def test_frame_filename_internal_index_and_time_grid_fail_closed(self) -> None:
        old = self.output / "data/Part_0008.bi4"
        new = self.output / "data/Part_0009.bi4"
        old.rename(new)
        with self.assertRaisesRegex(reader.FinalizedFrameError, "index sequence"):
            self.build(expected_inventory=self.freeze_inventory())
        new.rename(old)
        old.write_bytes(reader._fixture_part_bytes(9, 1.05))
        with self.assertRaisesRegex(reader.FinalizedFrameError, "internal index"):
            self.build(expected_inventory=self.freeze_inventory())
        old.write_bytes(reader._fixture_part_bytes(8, 1.051))
        with self.assertRaisesRegex(reader.FinalizedFrameError, "external grid"):
            self.build(expected_inventory=self.freeze_inventory())

    def test_nout_count_ids_class_and_nonfinite_drift_fail_closed(self) -> None:
        path = self.output / "data/Part_0008.bi4"
        mutations = (
            (reader._fixture_part_bytes(8, 1.05, nout=1), "Nout"),
            (
                reader._fixture_part_bytes(
                    8, 1.05, ids=(0, 1, 2), class_counts=(1, 0, 0, 2)
                ),
                "particle count",
            ),
            (reader._fixture_part_bytes(8, 1.05, ids=(0, 2, 1, 3)), "particle IDs"),
            (reader._fixture_part_bytes(8, 1.05, nonfinite=True), "non-finite"),
            (
                reader._fixture_part_bytes(8, 1.05, class_counts=(1, 0, 0, 3)),
                "class count drift",
            ),
        )
        for payload, message in mutations:
            with self.subTest(message=message):
                path.write_bytes(payload)
                with self.assertRaisesRegex(reader.FinalizedFrameError, message):
                    self.build(expected_inventory=self.freeze_inventory())
                path.write_bytes(reader._fixture_part_bytes(8, 1.05))
        with self.assertRaisesRegex(reader.FinalizedFrameError, "particle ID drift"):
            self.build(expected_ids_sha256="0" * 64)

    def test_external_hash_and_toctou_are_rejected(self) -> None:
        forged = dict(self.inventory)
        forged["Run.out"] = "0" * 64
        with self.assertRaisesRegex(reader.FinalizedFrameError, "external inventory SHA"):
            self.build(expected_inventory=forged)

        original = reader._parse_frame
        changed = False

        def replace_after_parse(*args: object, **kwargs: object) -> dict:
            nonlocal changed
            result = original(*args, **kwargs)
            if not changed:
                changed = True
                target = self.output / "Run.out"
                payload = target.read_bytes()
                target.write_bytes(payload + b"changed-after-read")
                target.chmod(0o600)
            return result

        with mock.patch.object(reader, "_parse_frame", side_effect=replace_after_parse):
            with self.assertRaisesRegex(reader.FinalizedFrameError, "TOCTOU"):
                self.build()

    def test_schema_is_deep_closed_and_rejects_claim_promotion(self) -> None:
        schema = json.loads(reader.SCHEMA_PATH.read_bytes())
        Draft202012Validator.check_schema(schema)
        reader.assert_deep_closed(schema)
        manifest = self.build()
        changed = copy.deepcopy(manifest)
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)
        changed = copy.deepcopy(manifest)
        changed["claims"]["solver_executed_by_reader"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_ast_and_fixture_only_self_check_do_not_expose_runtime(self) -> None:
        ast.parse(Path(reader.__file__).read_text(encoding="utf-8"))
        source = Path(reader.__file__).read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        result = reader.self_check()
        self.assertEqual(
            result["status"], "PASS_S5B0_FINALIZED_FRAME_READER_V1_FIXTURE_SELF_CHECK"
        )
        self.assertTrue(result["fixture_root_removed"])
        for key in (
            "real_solver_output_read", "real_bag_read", "candidate_executed",
            "solver_executed", "gpu_exposed", "network_used", "sudo_used",
            "apparmor_loaded", "files_written_outside_fixture",
        ):
            self.assertFalse(result[key], key)


if __name__ == "__main__":
    unittest.main()
