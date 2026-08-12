#!/usr/bin/env python3
"""Pure-data and create-new tests for S6 final-delivery engine v3."""

from __future__ import annotations

import copy
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
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_final_delivery_engine_v3 as engine  # noqa: E402


SHA = "1" * 64


class S6FinalDeliveryEngineV3Tests(unittest.TestCase):
    def gauge_payloads(self, count: int = 80) -> dict[str, bytes]:
        result = {}
        for probe_index in range(16):
            lines = ["time_s,zsurf_m"]
            for index in range(count):
                time_s = index * .5
                wave = .003 * math.exp(-.02 * time_s) * math.sin(2 * math.pi * .1 * time_s + probe_index * math.pi / 8)
                lines.append(f"{time_s:.6f},{.058 + wave:.12f}")
            result[f"s5b0_p{probe_index:02d}"] = ("\n".join(lines) + "\n").encode()
        return result

    def selected_bytes(self, start: int = 4, end: int = 72) -> bytes:
        lines = ["time_s,H_proxy_m,H_modal_m"]
        for index in range(start, end):
            time_s = index * .5
            proxy = .0027 * math.exp(-.019 * time_s) * math.sin(2 * math.pi * .1 * time_s + .05)
            modal = .0025 * math.exp(-.018 * time_s) * math.sin(2 * math.pi * .1 * time_s + .1)
            lines.append(f"{time_s:.6f},{proxy:.12f},{modal:.12f}")
        return ("\n".join(lines) + "\n").encode()

    def analysis(self) -> dict[str, object]:
        gauge = engine.read_native_gauge_csvs(self.gauge_payloads())
        selected = engine.read_selected_signals_csv(self.selected_bytes())
        return engine.analyze(gauge, selected, {
            "first15": {"start_s": 0., "end_s": 15.},
            "full_motion": {"start_s": 0., "end_s": 20.},
            "recorded_tail": {"start_s": 20., "end_s": 30.},
            "solver_tail": {"start_s": 30., "end_s": 39.5},
        })

    def test_contracts_closed_bound_and_cli_surface_not_admitted(self) -> None:
        policy, schemas = engine.load_contracts()
        self.assertEqual(3, len(schemas))
        for schema in schemas.values():
            Draft202012Validator.check_schema(schema); engine.assert_deep_closed(schema)
        self.assertEqual("NOT_MATERIALIZED", policy["parent"]["state"])
        self.assertEqual(16, len(policy["gauge"]["probe_names"]))
        for command in engine.COMMANDS:
            receipt = engine.static_receipt(command)
            self.assertEqual(engine.NOT_ADMITTED, receipt["status"])
            self.assertFalse(receipt["external_write_performed"])

    def test_selected_signal_reader_is_closed_strict_and_real_shaped(self) -> None:
        value = engine.read_selected_signals_csv(self.selected_bytes())
        self.assertEqual("SPMPC_NON_FIXED", value["source_schema"])
        self.assertEqual({"start_s": 2., "end_s": 35.5}, value["registered_overlap"])
        self.assertTrue(value["optional_unread"])
        with self.assertRaisesRegex(engine.S6EngineV3Error, "header"):
            engine.read_selected_signals_csv(self.selected_bytes().replace(b"H_proxy_m", b"proxy"))
        with self.assertRaisesRegex(engine.S6EngineV3Error, "provenance"):
            engine.read_selected_signals_csv(self.selected_bytes(), source_schema="OTHER")

    def test_sixteen_native_gauge_parsing_surfaces_and_invalid_fail_closed(self) -> None:
        value = engine.read_native_gauge_csvs(self.gauge_payloads())
        self.assertEqual(80, len(value["rows"]))
        self.assertEqual(16, len(value["per_probe"]))
        first = value["rows"][1]
        self.assertGreaterEqual(first["H_abs_m"], abs(first["H_crest_m"]))
        self.assertGreaterEqual(first["H_peak_to_peak_m"], 0)
        changed = self.gauge_payloads(); changed.pop("s5b0_p15")
        with self.assertRaisesRegex(engine.S6EngineV3Error, "sixteen"):
            engine.read_native_gauge_csvs(changed)
        changed = self.gauge_payloads(); lines = changed["s5b0_p00"].decode().splitlines(); lines[1] = "0.000000,0.100000"; changed["s5b0_p00"] = ("\n".join(lines)+"\n").encode()
        with self.assertRaisesRegex(engine.S6EngineV3Error, "invalid ratio"):
            engine.read_native_gauge_csvs(changed)

    def test_analysis_dual_grid_explicit_na_windows_metrics_no_ranking_and_figure_model(self) -> None:
        result = self.analysis()
        self.assertEqual(80, result["grids"]["solver"]["count"])
        self.assertEqual(68, result["grids"]["comparison"]["count"])
        outside = [row for row in result["solver_rows"] if row["H_proxy_m"] is None]
        self.assertTrue(outside)
        self.assertTrue(all(row["H_proxy_coverage"] == "NA_OUTSIDE_REGISTERED_OVERLAP" for row in outside))
        self.assertEqual(set(engine.WINDOWS), set(result["window_statistics"]))
        self.assertEqual({"peak", "p95", "rms", "peak_time_s"}, set(result["window_statistics"]["first15"]["H_crest"]))
        self.assertEqual(5, len(result["series_metrics"]))
        self.assertEqual(6, len(result["comparisons"]))
        self.assertTrue(all(not row["ranking_claimed"] for row in result["comparisons"]))
        self.assertTrue(result["figure_model"]["shared_x"])
        self.assertFalse(result["figure_model"]["dual_y_axes"])
        self.assertFalse(result["claims"]["stage6_pass"])
        with self.assertRaisesRegex(engine.S6EngineV3Error, "topology"):
            gauge = engine.read_native_gauge_csvs(self.gauge_payloads()); selected = engine.read_selected_signals_csv(self.selected_bytes())
            engine.analyze(gauge, selected, {"first15":{"start_s":1.,"end_s":15.},"full_motion":{"start_s":0.,"end_s":20.},"recorded_tail":{"start_s":20.,"end_s":30.},"solver_tail":{"start_s":30.,"end_s":39.5}})

    def test_media_is_bound_to_finalized_manifest_and_rendered_content(self) -> None:
        frames = [{"time_s": float(i), "class_counts_sha256": "2"*64} for i in range(5)]
        parent = {"status":"PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1", "integrity_pass":True, "frames":frames}
        rendered = {f"frame_{i}.png": f"frame-{i}".encode() for i in range(5)}
        media_frames = [{"relative_path":f"frame_{i}.png", "sha256":engine.sha256_bytes(rendered[f"frame_{i}.png"]),
                         "time_s":float(i), "class_counts_sha256":"2"*64, "probe_overlay_sha256":"3"*64,
                         "container_frame":"MOVING_CONTAINER_REFERENCE_REF_0"} for i in range(5)]
        media = {"frame_manifest_sha256":engine.sha256_json(parent), "frames":media_frames,
                 "mp4_sha256":"4"*64, "gif_sha256":"5"*64, "decoded_mp4_frames":5,
                 "decoded_gif_frames":5, "keyframe_indices":[0,2,4]}
        receipt = engine.validate_media_content(parent, rendered, media)
        self.assertTrue(receipt["pass"]); self.assertFalse(receipt["stage6_pass"])
        changed = copy.deepcopy(media); changed["frames"][2]["sha256"] = "9"*64
        with self.assertRaisesRegex(engine.S6EngineV3Error, "content"):
            engine.validate_media_content(parent, rendered, changed)

    def test_planned_assets_are_exact_seventeen_and_atomic_publisher_is_noreplace(self) -> None:
        required = engine.load_contracts()[0]["delivery"]["required_inventory"]
        generated = {"reports/analysis_result.json", "comparison_manifest.json", "evidence_index.json",
                     "liquid_secondary_ledger.jsonl", "secondary_ledger_append_receipt.json",
                     "acceptance_receipt.json", "checksums.sha256"}
        opaque = {name: f"opaque:{name}".encode() for name in required if name not in generated}
        assets = engine.planned_delivery_assets(self.analysis(), opaque)
        self.assertEqual(set(required), set(assets)); self.assertEqual(17, len(assets))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)/"delivery"
            receipt = engine.atomic_publish_inventory(root, assets, required)
            self.assertTrue(root.is_dir()); self.assertFalse(receipt["stage6_pass"])
            with self.assertRaisesRegex(engine.S6EngineV3Error, "create-new"):
                engine.atomic_publish_inventory(root, assets, required)

    def test_hash_chain_ledger_append_detects_stale_parent_duplicate_and_content_chain(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/"ledger.jsonl"
            first = engine.append_hash_chain_ledger(path, {"attempt_id":"a", "status":"ONE"}, expected_previous_sha256="0"*64)
            self.assertFalse(first["stage6_pass"])
            self.assertEqual(0o600, stat.S_IMODE(path.stat().st_mode))
            second = engine.append_hash_chain_ledger(path, {"attempt_id":"b", "status":"TWO"}, expected_previous_sha256=first["entry_sha256"])
            self.assertEqual(first["entry_sha256"], second["previous_entry_sha256"])
            with self.assertRaisesRegex(engine.S6EngineV3Error, "compare-and-append"):
                engine.append_hash_chain_ledger(path, {"attempt_id":"c"}, expected_previous_sha256="0"*64)
            with self.assertRaisesRegex(engine.S6EngineV3Error, "duplicate"):
                engine.append_hash_chain_ledger(path, {"attempt_id":"a"}, expected_previous_sha256=second["entry_sha256"])

    def test_self_check_has_no_external_read_or_write_claim(self) -> None:
        value = engine.self_check()
        self.assertEqual("S6_FINAL_DELIVERY_ENGINE_V3_SELF_CHECK_OK_NOT_ADMITTED", value["status"])
        self.assertTrue(all(status == engine.NOT_ADMITTED for status in value["commands"].values()))
        self.assertFalse(value["external_write_performed"]); self.assertFalse(value["optional_bag_read"])
        self.assertFalse(value["stage6_pass"])


if __name__ == "__main__": unittest.main()
