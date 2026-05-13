#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke tests for report-only OSCRS candidate diversity diagnostics."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.diagnostics import format_report  # noqa: E402
from oscrs.diversity import candidate_diversity  # noqa: E402


def _row(name, ax, ay, eta_x, eta_y, accepted=True):
    return {
        "name": name,
        "accepted": accepted,
        "reject_reason": "accepted" if accepted else "drift",
        "reject_stage": "ACCEPTED" if accepted else "GEOMETRY_REJECT",
        "score": 0.0,
        "length_ratio": 1.0,
        "max_drift_m": 0.0,
        "endpoint_error_m": 0.0,
        "kappa_p95": 0.1,
        "dkappa_p95": 0.1,
        "kappa_ratio": 1.0,
        "dkappa_ratio": 1.0,
        "predicted_ay_p95": ay,
        "predicted_ay_ratio": 1.0,
        "predicted_ay_peak": ay * 1.2,
        "predicted_ax_p95": ax,
        "predicted_ax_peak": ax * 1.5,
        "predicted_vmax": 1.0,
        "geometry_score": 0.0,
        "slosh_score": 0.0,
        "slosh_h_p95": max(eta_x, eta_y),
        "slosh_h_modal_p95": max(eta_x, eta_y),
        "slosh_h_parabola_p95": 0.0,
        "slosh_h_residual_max": 0.0,
        "slosh_energy_rms": 0.0,
        "slosh_eta_dot_rms": 0.0,
        "slosh_eta_x_p95": eta_x,
        "slosh_eta_y_p95": eta_y,
        "oscrs_feasible": accepted,
        "oscrs_height_pass": accepted,
        "oscrs_residual_pass": accepted,
        "oscrs_violation": 0.0,
        "oscrs_score": 0.0,
        "collision_status": "accepted",
        "collision_idx": -1,
        "collision_cost": -1,
    }


def test_eta_x_alignment():
    rows = [
        (_row("original", ax=0.40, ay=0.10, eta_x=0.100, eta_y=0.010), [(0.0, 0.0), (1.0, 0.0)]),
        (_row("strong", ax=0.80, ay=0.11, eta_x=0.200, eta_y=0.011), [(0.0, 0.0), (1.0, 0.1)]),
    ]
    div = candidate_diversity(rows)
    assert div["dominant_excitation"] == "eta_x"
    assert div["diversity_aligned"] == 1
    assert div["candidate_collapse"] == 0
    assert div["div_ax"] > 0.05
    assert div["div_eta_x"] > 0.05


def test_eta_x_unaligned_when_only_ay_varies():
    rows = [
        (_row("original", ax=0.40, ay=0.10, eta_x=0.200, eta_y=0.010), [(0.0, 0.0), (1.0, 0.0)]),
        (_row("strong", ax=0.41, ay=0.30, eta_x=0.205, eta_y=0.011), [(0.0, 0.0), (1.0, 0.1)]),
    ]
    div = candidate_diversity(rows)
    assert div["dominant_excitation"] == "eta_x"
    assert div["diversity_aligned"] == 0
    assert div["div_ay"] > 0.05
    assert div["div_ax"] < 0.05
    assert div["div_eta_x"] < 0.05


def test_format_report_contains_diversity_keys():
    best = _row("strong", ax=0.80, ay=0.11, eta_x=0.200, eta_y=0.011)
    original = _row("original", ax=0.40, ay=0.10, eta_x=0.100, eta_y=0.010)
    rows = [
        (original, [(0.0, 0.0), (1.0, 0.0)]),
        (best, [(0.0, 0.0), (1.0, 0.1)]),
    ]
    report = format_report(rows, best, original, best, True, True)
    assert "dominant_excitation=eta_x" in report
    assert "diversity_aligned=1" in report
    assert "candidate_collapse=0" in report
    assert "axp=0.800" in report
    assert "ex=0.2" in report


def main():
    test_eta_x_alignment()
    test_eta_x_unaligned_when_only_ay_varies()
    test_format_report_contains_diversity_keys()
    print("OK")


if __name__ == "__main__":
    main()
