#!/usr/bin/env python3
"""Static/fixture tests for S6 v6; no real parent or media encoder is run."""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s6_real_runtime_delivery_v6.py"
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_real_runtime_delivery_v6 as runtime  # noqa: E402


class S6RealRuntimeDeliveryV6Tests(unittest.TestCase):
    @staticmethod
    def identity(path: Path, *, relative_path: str | None = None) -> dict[str, object]:
        metadata = path.stat()
        result = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "size_bytes": metadata.st_size,
                  "mode": f"{stat.S_IMODE(metadata.st_mode):04o}", "device": metadata.st_dev,
                  "inode": metadata.st_ino, "nlink": metadata.st_nlink,
                  "mtime_ns": metadata.st_mtime_ns, "ctime_ns": metadata.st_ctime_ns}
        result["relative_path" if relative_path is not None else "path"] = relative_path if relative_path is not None else str(path)
        return result

    def gauge(self, count: int = 80) -> dict[str, object]:
        times = [index * .5 for index in range(count)]
        rows = []
        for time_s in times:
            heights = [.058 + .003 * math.exp(-.02 * time_s) * math.sin(2 * math.pi * .1 * time_s + probe * math.pi / 8) for probe in range(16)]
            eta = [value - .058 for value in heights]
            rows.append({"time_s": time_s, "H_crest_m": max(eta), "H_abs_m": max(abs(value) for value in eta),
                         "H_peak_to_peak_m": max(heights) - min(heights), "valid_probe_count": 16})
        per_probe = [{"name": f"s5b0_p{probe:02d}", "valid": count, "missing": 0, "invalid": 0,
                      "missing_ratio": 0.0, "invalid_ratio": 0.0} for probe in range(16)]
        return {"time_grid": times, "time_grid_sha256": runtime.sha256_json(times), "rows": rows, "per_probe": per_probe}

    def selected(self) -> dict[str, object]:
        rows = []
        for index in range(4, 72):
            time_s = index * .5
            rows.append({"time_s": time_s,
                         "H_proxy_m": .0027 * math.exp(-.019 * time_s) * math.sin(2 * math.pi * .1 * time_s + .05),
                         "H_modal_m": .0025 * math.exp(-.018 * time_s) * math.sin(2 * math.pi * .1 * time_s + .1)})
        return {"source_bag_sha256": "a" * 64, "rows": rows,
                "time_alignment": {"overlap_start_s": 2., "overlap_end_s": 35.5}}

    def analysis(self) -> dict[str, object]:
        return runtime.analyze(self.gauge(), self.selected(), {
            "first15": {"start_s": 0., "end_s": 15.},
            "full_motion": {"start_s": 0., "end_s": 20.},
            "recorded_tail": {"start_s": 20., "end_s": 30.},
            "solver_tail": {"start_s": 30., "end_s": 39.5},
        })

    def test_policy_schemas_deep_closed_default_deny(self) -> None:
        policy, schemas = runtime.load_contracts()
        self.assertEqual(3, len(schemas))
        for schema in schemas.values():
            Draft202012Validator.check_schema(schema)
            runtime.assert_deep_closed(schema)
        self.assertEqual("NOT_MATERIALIZED", policy["parent_contract"]["state"])
        receipt = runtime.static_receipt()
        self.assertEqual(runtime.NOT_ADMITTED, receipt["status"])
        self.assertFalse(receipt["claims"]["stage6_pass"])
        self.assertTrue(receipt["claims"]["physical_reference_pending"])

    def test_exact_file_fd_rejects_hash_symlink_hardlink_and_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "primary.bag"; source.write_bytes(b"fixture-primary-bag")
            metadata = source.stat()
            identity = {"path": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                        "size_bytes": metadata.st_size, "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
                        "device": metadata.st_dev, "inode": metadata.st_ino, "nlink": 1,
                        "mtime_ns": metadata.st_mtime_ns, "ctime_ns": metadata.st_ctime_ns}
            self.assertEqual(b"fixture-primary-bag", runtime.read_exact_file(source, identity))
            changed = dict(identity); changed["sha256"] = "0" * 64
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "hash"):
                runtime.read_exact_file(source, changed)
            symlink = root / "link.bag"; symlink.symlink_to(source)
            changed = dict(identity); changed["path"] = str(symlink)
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "symlink"):
                runtime.read_exact_file(symlink, changed)
            hardlink = root / "hard.bag"; os.link(source, hardlink)
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "single-link|metadata"):
                runtime.read_exact_file(source, identity)

    def test_gauge_manifest_and_frame_manifest_fail_closed(self) -> None:
        payloads = {}
        files = []
        for probe in runtime.PROBES:
            relative = f"gauge/{probe}.csv"; raw = b"time_s,zsurf_m\n0,0.058\n1,0.058\n2,0.058\n3,0.058\n"
            payloads[relative] = raw
            files.append({"probe_name": probe, "relative_path": relative, "sha256": runtime.sha256_bytes(raw),
                          "size_bytes": len(raw), "time_grid_sha256": "1" * 64})
        manifest = {"schema_version": "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1",
                    "attempt_id": runtime.ATTEMPT, "gauge_contract_sha256": "2" * 64,
                    "time_grid_sha256": "1" * 64, "files": files}
        runtime.validate_gauge_manifest(manifest, payloads)
        changed = copy.deepcopy(manifest); changed["files"][15]["probe_name"] = "other"
        with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "probe"):
            runtime.validate_gauge_manifest(changed, payloads)
        frames = {"schema_version": "smpcc-r8-liquid-s5b0-finalized-frame-manifest-v1",
                  "attempt_id": runtime.ATTEMPT, "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1",
                  "integrity_pass": True, "root": "/fixture", "frames": [
                      {"index": index, "time_s": float(index), "relative_path": f"data/Part_{index:04d}.bi4",
                       "sha256": "3" * 64, "particle_count": 4, "ids_sha256": "4" * 64,
                       "class_counts": {"fixed_boundary": 1, "moving_boundary": 1, "floating": 0, "fluid": 2}}
                      for index in range(3)]}
        runtime.validate_frame_manifest(frames)
        frames["frames"][1]["time_s"] = 0.0
        with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "times"):
            runtime.validate_frame_manifest(frames)

    def test_real_shape_finalized_s5b0_temp_package_admission(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary); package = base / "s5b0"; package.mkdir(); (package / "gauge").mkdir(); (package / "data").mkdir()
            payloads: dict[str, bytes] = {}
            gauge_files = []
            for probe in runtime.PROBES:
                relative = f"gauge/{probe}.csv"
                raw = b"time_s,zsurf_m\n0,0.058\n1,0.058\n2,0.058\n3,0.058\n"
                payloads[relative] = raw
                gauge_files.append({"probe_name": probe, "relative_path": relative,
                    "sha256": runtime.sha256_bytes(raw), "size_bytes": len(raw), "time_grid_sha256": "1" * 64})
            frame_rows = []
            for index in range(3):
                relative = f"data/Part_{index:04d}.bi4"; raw = f"fixture-bi4-{index}".encode(); payloads[relative] = raw
                frame_rows.append({"index": index, "time_s": float(index), "relative_path": relative,
                    "sha256": runtime.sha256_bytes(raw), "particle_count": 4, "ids_sha256": "2" * 64,
                    "class_counts": {"fixed_boundary": 1, "moving_boundary": 1, "floating": 0, "fluid": 2}})
            documents = {
                "execution_receipt.json": {"status": "S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY", "finalized": True, "attempt_id": runtime.ATTEMPT},
                "result_qc.json": {"status": "PASS_S5B0_REPLAY_RESULT_QC_V2", "pass": True, "attempt_id": runtime.ATTEMPT},
                "frame_manifest.json": {"schema_version": "smpcc-r8-liquid-s5b0-finalized-frame-manifest-v1", "attempt_id": runtime.ATTEMPT,
                    "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1", "integrity_pass": True, "root": str(package), "frames": frame_rows},
                "native_gauge_manifest.json": {"schema_version": "smpcc-r8-liquid-s5b0-native-gauge-manifest-v1", "attempt_id": runtime.ATTEMPT,
                    "gauge_contract_sha256": "3" * 64, "time_grid_sha256": "1" * 64, "files": gauge_files},
            }
            payloads.update({name: runtime.canonical_json(value) for name, value in documents.items()})
            for index in range(2):
                payloads[f"evidence_{index}.txt"] = f"evidence-{index}".encode()
            payloads["checksums.sha256"] = "".join(f"{runtime.sha256_bytes(payloads[name])}  {name}\n" for name in sorted(payloads)).encode()
            for relative, raw in payloads.items():
                target = package / relative; target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(raw)
            records = [self.identity(package / name, relative_path=name) for name in sorted(payloads)]
            policy_identity = self.identity(runtime.POLICY_PATH)
            dummy = base / "dummy.json"; dummy.write_text("{}\n")
            dummy_identity = self.identity(dummy)
            contract = {"schema_version": "smpcc-r8-liquid-s6-real-runtime-contract-v6",
                "document_type": "SMPCC_R8_LIQUID_S6_REAL_RUNTIME_CONTRACT_V6",
                "status": "ADMITTED_EXACT_FINALIZED_S5B0_PRIMARY_RUNTIME_MATERIALIZED_V6",
                "attempt_id": runtime.ATTEMPT, "planned_denominator": 1,
                "parents": {"policy": policy_identity, "s5a0_receipt": dummy_identity,
                    "s5a1_transfer_manifest": dummy_identity, "primary_bag": dummy_identity},
                "s5b0": {"root": str(package), "inventory_sha256": runtime.sha256_json(records), "inventory": records,
                    "required_parents": {"execution_receipt_sha256": runtime.sha256_bytes(payloads["execution_receipt.json"]),
                        "result_qc_sha256": runtime.sha256_bytes(payloads["result_qc.json"]), "frame_manifest_sha256": runtime.sha256_bytes(payloads["frame_manifest.json"]),
                        "native_gauge_manifest_sha256": runtime.sha256_bytes(payloads["native_gauge_manifest.json"]), "checksums_sha256": runtime.sha256_bytes(payloads["checksums.sha256"])}},
                "runtime": {"final_root": str(base / "delivery"), "external_ledger_path": str(base / "ledger.jsonl"),
                    "external_ledger_expected_previous_sha256": "0" * 64, "fps": 5, "windows": {
                        "first15": {"start_s": 0., "end_s": 1.}, "full_motion": {"start_s": 0., "end_s": 1.},
                        "recorded_tail": {"start_s": 1., "end_s": 2.}, "solver_tail": {"start_s": 2., "end_s": 3.}}},
                "claims": {"optional_bag_read": False, "ranking_allowed": False,
                    "selected_trajectory_cpu_comparison": False, "physical_reference_pending": True, "stage6_pass": False}}
            admitted = runtime.admit_finalized_s5b0(contract)
            self.assertEqual("ADMITTED_FINALIZED_S5B0_PRIMARY_PACKAGE_V6", admitted["status"])
            self.assertTrue(admitted["checks"]["optional_unread"])
            changed = copy.deepcopy(contract); changed["s5b0"]["inventory"][0]["sha256"] = "9" * 64
            changed["s5b0"]["inventory_sha256"] = runtime.sha256_json(changed["s5b0"]["inventory"])
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "identity"):
                runtime.admit_finalized_s5b0(changed)

    def test_analysis_four_windows_dual_grid_na_metrics_and_no_ranking(self) -> None:
        result = self.analysis()
        self.assertEqual(80, result["grids"]["solver"]["count"])
        self.assertEqual(68, result["grids"]["comparison"]["count"])
        outside = [row for row in result["solver_rows"] if row["H_proxy_m"] is None]
        self.assertTrue(outside)
        self.assertTrue(all(row["H_proxy_coverage"] == "NA_OUTSIDE_REGISTERED_OVERLAP" for row in outside))
        self.assertEqual(set(runtime.WINDOWS), set(result["window_statistics"]))
        self.assertEqual(6, len(result["comparisons"]))
        self.assertTrue(all(not row["ranking_claimed"] for row in result["comparisons"]))
        self.assertEqual({"amplitude", "frequency_hz", "damping_per_s", "phase_rad"},
                         set(result["series_metrics"]["H_crest"]))
        self.assertTrue(result["figure_contract"]["shared_x"])
        self.assertFalse(result["figure_contract"]["dual_y_axes"])
        self.assertFalse(result["claims"]["stage6_pass"])
        with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "topology"):
            runtime.analyze(self.gauge(), self.selected(), {"first15": {"start_s": 1., "end_s": 15.},
                "full_motion": {"start_s": 0., "end_s": 20.}, "recorded_tail": {"start_s": 20., "end_s": 30.},
                "solver_tail": {"start_s": 30., "end_s": 39.5}})

    def test_artifact_inventory_atomic_noreplace_and_ledger_compare_append(self) -> None:
        analysis = self.analysis()
        selected = {**self.selected(), "schema_version": "fixture"}
        gauge = {**self.gauge(), "schema_version": "fixture"}
        figures = {"artifacts": {name: f"figure:{name}".encode() for name in (
            "figures/primary_shared_x_timeseries.png", "figures/primary_shared_x_timeseries_grayscale.png",
            "figures/primary_shared_x_timeseries.pdf")}}
        media_names = ("animation/primary.mp4", "animation/primary_preview.gif", "keyframes/primary_first.png",
                       "keyframes/primary_middle.png", "keyframes/primary_last.png")
        media = {"artifacts": {name: f"media:{name}".encode() for name in media_names},
                 "manifest": {"schema_version": "fixture-media", "frames": []}}
        artifacts, entry = runtime.build_artifacts(analysis, selected, gauge, figures, media,
                                                    previous_ledger_sha256="0" * 64)
        self.assertEqual(19, len(artifacts))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); final = root / "delivery"; ledger = root / "ledger.jsonl"
            publication = runtime.atomic_publish(final, artifacts)
            self.assertEqual(19, publication["file_count"])
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "create-new"):
                runtime.atomic_publish(final, artifacts)
            appended = runtime.append_external_ledger(ledger, entry, expected_previous_sha256="0" * 64)
            self.assertTrue(appended["append_performed"])
            with self.assertRaisesRegex(runtime.S6RuntimeV6Error, "compare-append"):
                runtime.append_external_ledger(ledger, {**entry, "previous_entry_sha256": entry["entry_sha256"]},
                    expected_previous_sha256=entry["entry_sha256"])

    def test_public_self_check_never_reads_real_or_optional_and_no_exec_network_surface(self) -> None:
        report = runtime.self_check()
        self.assertIn("NOT_ADMITTED", report["status"])
        for key in ("real_parent_read", "real_bag_read", "optional_bag_read", "external_write_performed",
                    "media_executed", "solver_executed", "gpu_exposed", "network_used", "sudo_used", "stage6_pass"):
            self.assertFalse(report[key])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        imports.update(node.module.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module)
        self.assertFalse(imports & {"socket", "requests", "subprocess", "rosbag", "rospy"})
        parser_calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                        and node.func.attr in {"system", "popen", "run", "Popen"}]
        self.assertEqual([], parser_calls)


if __name__ == "__main__":
    unittest.main()
