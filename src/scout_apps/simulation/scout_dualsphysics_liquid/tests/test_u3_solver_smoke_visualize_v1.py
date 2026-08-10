#!/usr/bin/env python3
"""Offline-only tests for the U3 solver smoke visualization v1."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import r8_liquid_u3_solver_smoke_visualize_v1 as viz  # noqa: E402


def _part(index: int, time_s: float) -> dict:
    return {
        "name": f"Part_{index:04d}.bi4",
        "cpart": index,
        "time_s": time_s,
        "step": index * 100,
        "speed": {
            "rms_m_s": 0.01 + index * 0.001,
            "p95_m_s": 0.02 + index * 0.001,
            "max_m_s": 0.04 + index * 0.001,
            "specific_kinetic_energy_j_kg": 0.0001 + index * 0.00001,
        },
        "surface_proxy": {
            "valid": True,
            "median_height_m": 0.058 + index * 0.0001,
            "spatial_spread_m": 0.0002,
            "sector_heights_m": [0.0579 + index * 0.0001, 0.0581 + index * 0.0001],
        },
        "density": {
            "min_kg_m3": 999.0,
            "p01_kg_m3": 999.5,
            "mean_kg_m3": 1000.0 + index * 0.01,
            "p99_kg_m3": 1000.5,
            "max_kg_m3": 1001.0,
        },
    }


def _smoke_report() -> dict:
    return {
        "run": {
            "parts": [_part(0, 0.0), _part(1, 0.05)],
            "structural_pass": True,
            "short_duration_smoke": True,
            "duration_eligible_for_settle_qc": False,
            "tail_pass": False,
            "numeric_settle_qc_pass": False,
        },
        "verdict": {
            "status": "C1M_ZERO_MOTION_SMOKE_PASS",
            "settled_state_claim_allowed": False,
            "settled_state_freeze_eligible": False,
        },
    }


class SolverSmokeVisualizationTests(unittest.TestCase):
    def test_complete_smoke_is_accepted_without_settle_claim(self) -> None:
        viz.validate_smoke_report(_smoke_report())

    def test_settle_claim_or_tail_pass_changes_are_rejected(self) -> None:
        report = _smoke_report()
        report["verdict"]["settled_state_claim_allowed"] = True
        with self.assertRaisesRegex(ValueError, "settled_claim_forbidden"):
            viz.validate_smoke_report(report)

    def test_metric_rows_preserve_time_and_surface_envelope(self) -> None:
        rows = viz.build_metric_rows(_smoke_report())
        self.assertEqual([row["time_s"] for row in rows], [0.0, 0.05])
        self.assertEqual(rows[1]["surface_proxy_sector_min_m"], 0.058)
        self.assertEqual(rows[1]["surface_proxy_sector_max_m"], 0.0582)

    def test_metric_rows_reject_non_increasing_time(self) -> None:
        report = _smoke_report()
        report["run"]["parts"][1]["time_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            viz.build_metric_rows(report)

    def test_solver_reordered_ids_remain_aligned_with_classes(self) -> None:
        moving_case = {
            "particle_count": 5,
            "counts": {
                "fixed_boundary": 0,
                "moving_boundary": 2,
                "floating": 0,
                "fluid": 3,
            },
        }
        ids = [4, 1, 3, 0, 2]
        positions = [(float(value), 0.0, 0.0) for value in ids]
        classes = viz.classify_final_particles(ids, positions, moving_case)
        self.assertEqual(classes, ["fluid", "moving_boundary", "fluid", "moving_boundary", "fluid"])

    def test_duplicate_or_missing_ids_are_rejected(self) -> None:
        moving_case = {
            "particle_count": 3,
            "counts": {
                "fixed_boundary": 0,
                "moving_boundary": 1,
                "floating": 0,
                "fluid": 2,
            },
        }
        with self.assertRaisesRegex(ValueError, "complete unique"):
            viz.classify_final_particles([0, 1, 1], [(0, 0, 0)] * 3, moving_case)

    def test_metrics_csv_refuses_overwrite(self) -> None:
        rows = viz.build_metric_rows(_smoke_report())
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "metrics.csv"
            viz.write_metrics_csv(target, rows)
            with self.assertRaises(FileExistsError):
                viz.write_metrics_csv(target, rows)

    def test_final_export_requires_completed_external_preview_review(self) -> None:
        with self.assertRaisesRegex(ValueError, "external-preview-reviewed"):
            viz.run_export(SimpleNamespace(external_preview_reviewed=False))

    def test_cid_font_warning_is_resolved_only_by_passing_specific_audit(self) -> None:
        compliance = {
            "figure.pdf": {
                "issues": [("WARN", "PDF 中以下字体可能未嵌入: TestFont")]
            }
        }
        result = viz.resolve_compliance_issues(compliance, {"pass": False})
        self.assertEqual(len(result["unresolved"]), 1)
        result = viz.resolve_compliance_issues(
            compliance,
            {
                "pass": True,
                "status": "PASS_CID_TRUETYPE_FONTS_EMBEDDED_SUBSET_UNICODE",
            },
        )
        self.assertEqual(result["unresolved"], [])
        self.assertEqual(len(result["resolved"]), 1)


if __name__ == "__main__":
    unittest.main()
