#!/usr/bin/env python3

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TOOL_PATH = (
    PACKAGE_ROOT
    / "tools"
    / "simulation"
    / "generate_phase_rejoin_recovery_dataset.py"
)
SPEC = importlib.util.spec_from_file_location(
    "generate_phase_rejoin_recovery_dataset", TOOL_PATH
)
GENERATOR = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GENERATOR)

DATASET_COLUMNS = (
    GENERATOR.DATASET_ID_COLUMNS
    + GENERATOR.DATASET_STATE_COLUMNS
    + GENERATOR.DATASET_EXECUTION_FIXED_COLUMNS
    + tuple("linear_pending_{}".format(index) for index in range(4))
    + tuple("angular_pending_{}".format(index) for index in range(1))
)


class RecoveryDatasetGeneratorTest(unittest.TestCase):

    def test_bundle_publishes_every_member_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outputs = (
                (root / "dataset.csv", b"dataset\n"),
                (root / "audit.csv", b"audit\n"),
                (root / "manifest.json", b"{}\n"),
            )
            GENERATOR._publish_exclusive_bundle(outputs)
            self.assertEqual(outputs[0][0].read_bytes(), outputs[0][1])
            self.assertEqual(outputs[1][0].read_bytes(), outputs[1][1])
            self.assertEqual(outputs[2][0].read_bytes(), outputs[2][1])
            with self.assertRaisesRegex(
                GENERATOR.DatasetGenerationError, "already exists"
            ):
                GENERATOR._publish_exclusive_bundle(outputs)

    def test_bundle_rolls_back_if_later_publish_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            outputs = (
                (root / "dataset.csv", b"dataset\n"),
                (root / "audit.csv", b"audit\n"),
                (root / "manifest.json", b"{}\n"),
            )
            real_link = GENERATOR.os.link
            calls = 0

            def fail_second_link(source, target):
                nonlocal calls
                calls += 1
                if calls == 2:
                    raise OSError("injected audit publish failure")
                return real_link(source, target)

            with mock.patch.object(
                GENERATOR.os, "link", side_effect=fail_second_link
            ):
                with self.assertRaisesRegex(OSError, "injected"):
                    GENERATOR._publish_exclusive_bundle(outputs)
            self.assertEqual(calls, 2)
            self.assertTrue(all(not path.exists() for path, _ in outputs))
            self.assertEqual(list(root.iterdir()), [])

    def test_session_rejects_seed_leakage_between_splits(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session.yaml"
            session.write_text(
                yaml.safe_dump(
                    {
                        "schema": GENERATOR.SESSION_SCHEMA,
                        "formal_trials_started": False,
                        "scope": {
                            "simulation_only": True,
                            "formal_robot_release": False,
                            "real_robot_enforce_allowed": False,
                            "plant_truth_visible_to_controller": False,
                            "physical_parameter_claim": False,
                        },
                        "seeds": {
                            "locked": True,
                            "recovery_fit": [7101],
                            "recovery_tune": [7101],
                            "recovery_held_out": [7301],
                        },
                    },
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                GENERATOR.DatasetGenerationError, "crosses"
            ):
                GENERATOR.load_session_seeds(session)

    def test_partial_requires_paired_identity_and_truth_isolation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "partial.csv"
            audit = root / "partial-audit.csv"
            rollout_id = "fit-s7101-p0-nominal"
            with dataset.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(
                stream, fieldnames=DATASET_COLUMNS
                )
                writer.writeheader()
                row = {name: "0" for name in DATASET_COLUMNS}
                row.update(
                    {
                        "split": "fit",
                        "rollout_id": rollout_id,
                        "seed": "7101",
                        "phase_index": "0",
                        "recovered": "1",
                    }
                )
                writer.writerow(row)
            audit_header = (
                "rollout_id",
                "external_liquid_truth_visible_to_candidate_policy",
                "external_liquid_truth_used_for_features",
                "external_liquid_truth_used_for_label",
            )
            with audit.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=audit_header)
                writer.writeheader()
                writer.writerow(
                    {
                        "rollout_id": rollout_id,
                        "external_liquid_truth_visible_to_candidate_policy": "0",
                        "external_liquid_truth_used_for_features": "0",
                        "external_liquid_truth_used_for_label": "1",
                    }
                )
            rows, _, _, audits = GENERATOR._validate_partial(
                "fit", 7101, 0, 0, 1, dataset, audit
            )
            self.assertEqual(rows[0]["rollout_id"], audits[0]["rollout_id"])

            values = list(csv.DictReader(audit.read_text("utf-8").splitlines()))
            values[0]["external_liquid_truth_used_for_features"] = "1"
            with audit.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=audit_header)
                writer.writeheader()
                writer.writerows(values)
            with self.assertRaisesRegex(
                GENERATOR.DatasetGenerationError, "truth isolation"
            ):
                GENERATOR._validate_partial(
                    "fit", 7101, 0, 0, 1, dataset, audit
                )


if __name__ == "__main__":
    unittest.main()
