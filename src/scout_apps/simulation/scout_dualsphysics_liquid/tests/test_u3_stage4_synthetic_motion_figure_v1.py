#!/usr/bin/env python3
"""Static and fail-closed tests for the Stage-4 diagnostic figure."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_stage4_synthetic_motion_figure_v1 as figure  # noqa: E402


class SyntheticMotionFigureV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_all_objects_are_closed(self) -> None:
        schema = json.loads(figure.SCHEMA_PATH.read_text(encoding="utf-8"))
        figure._system_schema_check()
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_figure_contract_has_six_panels_and_no_dual_axis(self) -> None:
        spec = figure._figure_spec()
        self.assertEqual(spec["panel_count"], 6)
        self.assertEqual(len(spec["panels"]), 6)
        self.assertFalse(spec["dual_y_axes_used"])
        self.assertFalse(spec["uncertainty_band_used"])
        self.assertEqual(spec["heatmap_colormap"], "RdBu_r")
        self.assertEqual(spec["heatmap_center"], 0.0)

    def test_palette_is_colorblind_safe_and_redundantly_encoded(self) -> None:
        self.assertEqual(figure.PALETTE, {"zero": "#4D4D4D", "translation": "#0072B2", "yaw": "#D55E00"})
        self.assertEqual(len(set(figure.LINESTYLES.values())), 3)
        self.assertEqual(len(set(figure.MARKERS.values())), 3)

    def test_frozen_qc_and_table_load_with_exact_shape(self) -> None:
        inputs, qc, data = figure.verify_inputs(include_local_revision=False)
        self.assertEqual(inputs["qc_receipt"]["sha256"], figure.FROZEN_INPUTS["qc_receipt"][1])
        self.assertEqual(inputs["metrics_csv"]["sha256"], figure.FROZEN_INPUTS["metrics_csv"][1])
        self.assertEqual(data.shape, (143, 37))
        self.assertTrue(qc["verdict"]["stage4_liquid_only_validation_complete"])
        self.assertFalse(qc["verdict"]["stage5_admitted"])

    def test_figure_build_has_six_data_axes_and_clean_layout(self) -> None:
        _, qc, data = figure.verify_inputs(include_local_revision=False)
        fig = figure.build_figure(data, qc)
        data_axes = [axis for axis in fig.axes if axis.get_subplotspec() is not None]
        self.assertEqual(len(data_axes), 6)
        self.assertEqual(figure.audit_layout(fig), [])
        figure.plt.close(fig)

    def test_create_new_guard_detects_existing_and_accepts_absent(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "existing"
            absent = Path(directory) / "absent"
            existing.write_bytes(b"x")
            figure._ensure_absent([absent])
            with self.assertRaises(figure.FigureEvidenceError):
                figure._ensure_absent([existing, absent])

    def test_asset_export_requires_explicit_preview_visual_review(self) -> None:
        with self.assertRaises(figure.FigureEvidenceError):
            figure.export_assets_command(preview_visual_review_pass=False)

    def test_receipt_finalize_requires_explicit_asset_visual_review(self) -> None:
        with self.assertRaises(figure.FigureEvidenceError):
            figure.finalize_receipt_command(visual_review_pass=False)

    def test_cli_separates_asset_export_from_receipt_finalization(self) -> None:
        self.assertEqual(
            figure.parser().parse_args(["export-assets", "--visual-review-pass"]).command,
            "export-assets",
        )
        self.assertEqual(
            figure.parser().parse_args(["finalize-receipt", "--visual-review-pass"]).command,
            "finalize-receipt",
        )

    def test_static_self_check_is_read_only(self) -> None:
        result = figure.static_self_check()
        self.assertEqual(result["rows"], 143)
        self.assertEqual(result["columns"], 37)
        self.assertEqual(result["panels"], 6)
        self.assertEqual(result["layout_issues"], [])
        self.assertFalse(result["dual_y_axes_used"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["network_used"])


if __name__ == "__main__":
    unittest.main()
