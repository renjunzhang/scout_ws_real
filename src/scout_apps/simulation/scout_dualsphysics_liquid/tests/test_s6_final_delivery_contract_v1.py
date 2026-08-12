#!/usr/bin/env python3
"""Static golden/fail-closed tests for S6 final-delivery contract v1."""

from __future__ import annotations

import ast
import copy
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "r8_liquid_s6_final_delivery_contract_v1.py"
sys.path.insert(0, str(SCRIPTS))
import r8_liquid_s6_final_delivery_contract_v1 as contract  # noqa: E402


class S6FinalDeliveryContractV1Tests(unittest.TestCase):
    def gauge_bytes(self) -> bytes:
        lines = ["time_s,p00,p01,p02,p03"]
        for index in range(24):
            time_s = float(index)
            wave = 8.0 * math.exp(-0.03 * time_s) * math.sin(2 * math.pi * 0.125 * time_s)
            values = [100.0 + wave, 100.0 + 0.7 * wave, 100.0 - 0.5 * wave, 100.0 - 0.8 * wave]
            if index == 3:
                tokens = [f"{values[0]:.12f}", f"{values[1]:.12f}", "NA", f"{values[3]:.12f}"]
            elif index == 4:
                tokens = [f"{values[0]:.12f}", f"{values[1]:.12f}", "bad", f"{values[3]:.12f}"]
            else:
                tokens = [f"{value:.12f}" for value in values]
            lines.append(f"{time_s:.1f}," + ",".join(tokens))
        return ("\n".join(lines) + "\n").encode("utf-8")

    def selected_source(self) -> dict[str, object]:
        return {
            "attempt_id": contract.ATTEMPT_ID, "planned_denominator": 1,
            "source_schema": "SPMPC_NON_FIXED", "odom_header_origin_ns": 1_000_000_000,
            "series": {
                "H_proxy": {"topic": "/slosh/height", "native_unit": "m",
                            "samples": [{"time_since_odom_origin_s": float(i), "value_native": 0.006 * math.sin(2 * math.pi * 0.125 * i)} for i in range(2, 22)]},
                "H_modal": {"topic": "/spmpc/slosh_height", "native_unit": "mm",
                            "samples": [{"time_since_odom_origin_s": float(i), "value_native": 5.0 * math.sin(2 * math.pi * 0.125 * i + 0.1)} for i in range(1, 23)]},
            },
            "claims": {"optional_bag_read": False, "comparison_only": True,
                       "motion_exporter_consumed_selected_signals": False,
                       "solver_forcing_consumed_selected_signals": False},
        }

    def analyze(self) -> dict[str, object]:
        gauge = contract.read_gauge_zsurf_bytes(self.gauge_bytes(), probe_ids=("p00", "p01", "p02", "p03"), h0_mm=100.0)
        selected = contract.adapt_selected_signals(self.selected_source())
        return contract.analyze_fixture(gauge, selected, windows={
            "first15": {"start_s": 0.0, "end_s": 15.0},
            "full_motion": {"start_s": 0.0, "end_s": 15.0},
            "recorded_tail": {"start_s": 15.0, "end_s": 20.0},
            "solver_tail": {"start_s": 20.0, "end_s": 23.0},
        })

    def test_contracts_are_closed_policy_is_not_admitted(self) -> None:
        policy, schemas = contract.load_contracts()
        self.assertEqual(4, len(schemas))
        for schema in schemas.values():
            Draft202012Validator.check_schema(schema)
            contract.assert_deep_closed(schema)
        self.assertEqual("NOT_MATERIALIZED", policy["s5b0_parent"]["state"])
        admission = contract.admit_real_package()
        self.assertEqual(contract.NOT_ADMITTED, admission["status"])
        self.assertFalse(admission["claims"]["stage6_pass"])
        self.assertTrue(admission["claims"]["physical_reference_pending"])

    def test_gauge_reader_is_raw_per_probe_and_reports_missing_invalid_separately(self) -> None:
        result = contract.read_gauge_zsurf_bytes(self.gauge_bytes(), probe_ids=("p00", "p01", "p02", "p03"), h0_mm=100.0)
        self.assertEqual(24, len(result["rows"]))
        p02 = result["per_probe_qc"][2]
        self.assertEqual(1, p02["missing_count"])
        self.assertEqual(1, p02["invalid_count"])
        self.assertAlmostEqual(2 / 24, p02["missing_or_invalid_ratio"])
        self.assertFalse(result["precomputed_surface_columns_consumed"])
        first = result["rows"][1]
        valid = [value for value in first["zsurf_mm"].values() if value is not None]
        self.assertAlmostEqual(max(value - 100 for value in valid), first["H_crest_mm"])
        self.assertAlmostEqual(max(abs(value - 100) for value in valid), first["H_abs_mm"])

    def test_gauge_header_precomputed_surface_and_ratio_fail_closed(self) -> None:
        changed = self.gauge_bytes().replace(b"time_s,p00,p01,p02,p03", b"time_s,p00,p01,p02,H_crest")
        with self.assertRaisesRegex(contract.S6FinalDeliveryError, "header differs"):
            contract.read_gauge_zsurf_bytes(changed, probe_ids=("p00", "p01", "p02", "p03"), h0_mm=100.0)
        lines = self.gauge_bytes().decode().splitlines()
        for index in range(1, 10):
            columns = lines[index].split(",")
            columns[1] = "NA"
            lines[index] = ",".join(columns)
        with self.assertRaisesRegex(contract.S6FinalDeliveryError, "per-probe"):
            contract.read_gauge_zsurf_bytes(("\n".join(lines) + "\n").encode(), probe_ids=("p00", "p01", "p02", "p03"), h0_mm=100.0)

    def test_semantic_split_never_infers_outcome_from_schema(self) -> None:
        adapted = contract.adapt_selected_signals(self.selected_source())
        self.assertEqual("SPMPC_NON_FIXED", adapted["source_schema"])
        self.assertEqual("UNKNOWN", adapted["source_outcome_evidence"])
        self.assertFalse(adapted["outcome_inferred_from_schema"])
        self.assertEqual({"start_s": 2.0, "end_s": 21.0}, adapted["registered_overlap"])
        changed = copy.deepcopy(self.selected_source())
        changed["source_schema"] = "SPMPC_NON_FIXED_PASS"
        with self.assertRaises(contract.S6FinalDeliveryError):
            contract.adapt_selected_signals(changed)

    def test_dual_grid_explicit_na_no_extrapolation_and_full_3_by_2_matrix(self) -> None:
        result = self.analyze()
        self.assertEqual(24, result["grids"]["solver_output_grid"]["sample_count"])
        self.assertEqual(20, result["grids"]["comparison_grid"]["sample_count"])
        outside = [row for row in result["solver_rows"] if row["H_proxy_mm"] is None]
        self.assertEqual([0.0, 1.0, 22.0, 23.0], [row["time_s"] for row in outside])
        self.assertTrue(all(row["H_proxy_coverage"] == "NA_OUTSIDE_REGISTERED_OVERLAP" for row in outside))
        self.assertFalse(result["grids"]["extrapolation"])
        self.assertTrue(result["grids"]["denominator_preserved"])
        self.assertEqual(6, len(result["comparison_matrix"]))
        self.assertEqual({"H_crest", "H_abs", "H_peak_to_peak"}, {row["surface"] for row in result["comparison_matrix"]})
        self.assertEqual({"H_proxy", "H_modal"}, {row["secondary"] for row in result["comparison_matrix"]})
        self.assertTrue(all(set(row["dimensions"]) == {"amplitude_error_mm", "frequency_error_hz", "damping_error_per_s", "phase_error_rad"} for row in result["comparison_matrix"]))
        self.assertTrue(all(not row["ranking_claimed"] for row in result["comparison_matrix"]))

    def test_figure_media_publishers_and_acceptance_remain_fixture_only(self) -> None:
        analysis = self.analyze()
        from PIL import Image, ImageDraw
        frames = []
        for index in range(5):
            image = Image.new("RGB", (160, 120), "white")
            draw = ImageDraw.Draw(image)
            draw.rectangle((10 + index * 4, 40, 100, 95), fill="#56B4E9", outline="#000000")
            draw.text((5, 5), f"fixture t={index}", fill="#000000")
            stream = io.BytesIO()
            image.save(stream, format="PNG")
            frames.append(stream.getvalue())
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            figure_qa = contract.render_three_panel_figure(analysis, base / "figures", fixture_only=True)
            media_qa = contract.publish_fixture_media(frames, [0, 1, 2, 3, 4], base / "media", fixture_only=True, fps=5)
            self.assertTrue(figure_qa["pass"])
            self.assertTrue(figure_qa["grayscale_distinguishable"])
            self.assertTrue(media_qa["pass"])
            self.assertTrue(media_qa["mp4_complete_decode"])
            self.assertTrue(media_qa["gif_complete_decode"])
            assets = contract.build_delivery_assets(analysis, figure_qa, media_qa)
            self.assertIn("checksums.sha256", assets)
            admission = contract.admit_real_package()
            final = contract.final_acceptance_gate(admission_receipt=admission, analysis_pass=True,
                                                    figure_qa=figure_qa, media_qa=media_qa,
                                                    publication_complete=True)
            self.assertEqual(contract.NOT_ADMITTED, final["status"])
            self.assertFalse(final["claims"]["stage6_pass"])
            self.assertFalse(final["admission"]["pass"])

    def test_real_render_and_media_are_not_admitted(self) -> None:
        with self.assertRaisesRegex(contract.S6FinalDeliveryError, "REAL_FIGURE_NOT_ADMITTED"):
            contract.render_three_panel_figure({}, Path("/not/used"), fixture_only=False)
        with self.assertRaisesRegex(contract.S6FinalDeliveryError, "REAL_MEDIA_NOT_ADMITTED"):
            contract.publish_fixture_media([], [], Path("/not/used"), fixture_only=False)

    def test_self_check_and_ast_have_no_execution_or_network_surface(self) -> None:
        report = contract.self_check()
        self.assertEqual("S6_FINAL_DELIVERY_CONTRACT_V1_SELF_CHECK_OK_NOT_ADMITTED", report["status"])
        self.assertEqual(contract.NOT_ADMITTED, report["final_status"])
        self.assertFalse(report["optional_bag_read"])
        self.assertFalse(report["solver_or_gpu_executed"])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = set()
        calls = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                if node.func.attr in {"system", "popen", "run", "Popen", "twinx"}:
                    calls.append(node.func.attr)
        self.assertFalse(imports & {"rosbag", "rospy", "socket", "requests", "subprocess"})
        self.assertEqual([], calls)


if __name__ == "__main__":
    unittest.main()
