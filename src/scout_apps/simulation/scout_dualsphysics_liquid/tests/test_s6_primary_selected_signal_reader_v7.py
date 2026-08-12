#!/usr/bin/env python3
"""Static/tempdir-only tests for the exact one-bag selected-signal reader v7."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s6_primary_selected_signal_reader_v7.py"
SCHEMA = ROOT / "schema/target_host_s6_primary_selected_signals_v7.json"
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_primary_selected_signal_reader_v7 as reader  # noqa: E402


class SelectedSignalReaderV7Tests(unittest.TestCase):
    @staticmethod
    def identity(path: Path) -> dict[str, object]:
        metadata = path.stat()
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "size_bytes": metadata.st_size, "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
                "device": metadata.st_dev, "inode": metadata.st_ino, "nlink": metadata.st_nlink,
                "mtime_ns": metadata.st_mtime_ns, "ctime_ns": metadata.st_ctime_ns}

    def test_schema_and_public_surface_are_closed_static_only(self) -> None:
        schema = json.loads(SCHEMA.read_bytes())
        Draft202012Validator.check_schema(schema); reader.assert_deep_closed(schema)
        report = reader.self_check()
        self.assertEqual(1, report["planned_denominator"])
        self.assertEqual("UNKNOWN", report["source_outcome"])
        for key in ("real_bag_read", "optional_bag_read", "external_write", "candidate_executed",
                    "solver_or_gpu_executed", "stage6_pass"):
            self.assertFalse(report[key])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        imports.update(node.module.split(".")[0] for node in ast.walk(tree)
                       if isinstance(node, ast.ImportFrom) and node.module)
        self.assertFalse(imports & {"socket", "requests", "subprocess", "rosbag", "rospy"})

    def test_identity_reader_rejects_hash_symlink_hardlink_and_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); source = base / "primary.bag"; source.write_bytes(b"one-primary")
            identity = self.identity(source)
            self.assertEqual(b"one-primary", reader._identity(source, identity, maximum=64))
            changed = dict(identity); changed["sha256"] = "0" * 64
            with self.assertRaisesRegex(reader.SelectedSignalV7Error, "identity"):
                reader._identity(source, changed, maximum=64)
            link = base / "link.bag"; link.symlink_to(source)
            changed = dict(identity); changed["path"] = str(link)
            with self.assertRaises(OSError):
                reader._identity(link, changed, maximum=64)
            hard = base / "hard.bag"; os.link(source, hard)
            with self.assertRaisesRegex(reader.SelectedSignalV7Error, "single-link"):
                reader._identity(source, identity, maximum=64)

    def test_result_negative_claim_and_optional_promotion_rejected(self) -> None:
        schema = json.loads(SCHEMA.read_bytes())
        fixture = {
            "schema_version": "smpcc-r8-liquid-s6-primary-selected-signals-v7",
            "document_type": "SMPCC_R8_LIQUID_S6_PRIMARY_SELECTED_SIGNALS_V7",
            "status": "PASS_S6_PRIMARY_SELECTED_SIGNALS_V7_READ_ONLY",
            "attempt_id": reader.ATTEMPT_ID, "planned_denominator": 1, "source_outcome": "UNKNOWN",
            "parents": {name: {"path": f"/fixture/{name}", "sha256": "a" * 64, "size_bytes": 1,
                                "mode": "0400", "device": 1, "inode": index + 1, "nlink": 1,
                                "mtime_ns": 1, "ctime_ns": 1}
                        for index, name in enumerate(("s5a0_selected_bag_receipt", "s5a1_transfer_manifest", "source_bag"))},
            "reader_contract": {
                "extractor": {"relative_path": "scripts/r8_liquid_s6_real_selected_signal_extractor_v5.py", "sha256": "a" * 64},
                "reader_core": {"relative_path": "scripts/r8_liquid_ros1_bag_v2_reader_v1.py", "sha256": "b" * 64},
                "reader_v4": {"relative_path": "scripts/r8_liquid_ros1_bag_v2_reader_v4.py", "sha256": "c" * 64},
                "extractor_v3": {"relative_path": "scripts/r8_liquid_s5a1_ros1_signal_extractor_v3.py", "sha256": "d" * 64},
                "input_surface": "IMMUTABLE_BOUNDED_EXACT_PRIMARY_BAG_BYTES_ONLY"},
            "time_alignment": {"x_axis": "time_since_odom_header_origin_s", "motion_time_source": "/odom.header.stamp",
                "signal_native_time_source": "ROS1_BAG_RECORD_TIME_NS", "mapping_method": "LOWER_MEDIAN_ODOM_HEADER_MINUS_RECORD_OFFSET_V1",
                "odom_header_origin_ns": 1, "odom_header_end_ns": 4, "offset_sample_count": 3,
                "record_to_odom_header_offset_ns": 0, "residual_mean_ns": 0.0, "residual_rms_ns": 0.0,
                "residual_max_abs_ns": 0, "maximum_allowed_residual_ns": 5000000, "residuals_sha256": "e" * 64,
                "mapping_extrapolation": False, "overlap_start_s": 0.0, "overlap_end_s": 1.0},
            "series": {},
            "integrity": {"source_bag_sha256": "a" * 64, "H_proxy_samples_sha256": "a" * 64,
                          "H_modal_samples_sha256": "a" * 64, "reader_anomalies_absent": True, "parents_unchanged": True},
            "claims": {"read_only": True, "comparison_only": True, "optional_bag_read": False,
                "source_bag_executed": False, "ros_started": False, "motion_exporter_consumed_selected_signals": False,
                "solver_forcing_consumed_selected_signals": False, "stage6_pass": False, "development_only": True,
                "paired_ranking": False, "cross_method_ranking": False, "selected_trajectory_cpu_comparison": False,
                "physical_reference_pending": True, "physical_fidelity_validated": False, "formal": False, "production": False},
        }
        sample = lambda t, value: {"bag_record_t_ns": t + 1, "mapped_odom_header_t_ns": t + 1,
                                   "time_since_odom_origin_s": float(t), "value_native": value,
                                   "value_comparison_mm": value}
        for name, topic, native, scale in (("H_proxy", "/slosh/height", "m", 1000.0),
                                            ("H_modal", "/spmpc/slosh_height", "mm", 1.0)):
            samples = [sample(0, 0.0), sample(1, 1.0)]
            if name == "H_proxy":
                for row in samples: row["value_comparison_mm"] = row["value_native"] * 1000.0
            fixture["series"][name] = {"topic": topic, "message_type": "std_msgs/Float32", "native_unit": native,
                "comparison_unit": "mm", "scale_to_comparison": scale, "offset_to_comparison": 0.0,
                "sample_count": 2, "samples": samples}
            fixture["integrity"][f"{name}_samples_sha256"] = reader.sha256_bytes(reader.canonical_json(samples))
        reader.validate_result(fixture)
        for path, value in ((["claims", "optional_bag_read"], True), (["claims", "cross_method_ranking"], True),
                            (["planned_denominator"], 2), (["source_outcome"], "SPMPC_NON_FIXED")):
            changed = copy.deepcopy(fixture); cursor = changed
            for key in path[:-1]: cursor = cursor[key]
            cursor[path[-1]] = value
            with self.assertRaises((ValidationError, reader.SelectedSignalV7Error)):
                reader.validate_result(changed)


if __name__ == "__main__":
    unittest.main()
