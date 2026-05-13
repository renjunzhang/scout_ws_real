#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke tests for OSCRS selector score semantics."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.selector import apply_oscrs_score  # noqa: E402


def _row(name, scale):
    return {
        "name": name,
        "oscrs_feasible": True,
        "slosh_h_p95": 2.0 * scale,
        "slosh_energy_rms": 3.0 * scale,
        "slosh_eta_dot_rms": 5.0 * scale,
        "slosh_terminal_E": 7.0 * scale,
        "geometry_score": 11.0 * scale,
        "oscrs_score": 999.0,
    }


def main():
    rows = [(_row("mild", 1.0), []), (_row("medium", 2.0), [])]
    apply_oscrs_score(rows, 1.0, 0.3, 0.3, 0.2, 0.2, False)
    expected_raw = 2.0 + 0.3 * 3.0 + 0.3 * 5.0 + 0.2 * 7.0 + 0.2 * 11.0
    assert abs(rows[0][0]["oscrs_score"] - expected_raw) < 1e-12

    rows = [(_row("mild", 1.0), []), (_row("medium", 2.0), [])]
    apply_oscrs_score(rows, 1.0, 0.3, 0.3, 0.2, 0.2, True)
    expected_norm = 0.5 + 0.3 * 0.5 + 0.3 * 0.5 + 0.2 * 0.5 + 0.2 * 0.5
    assert abs(rows[0][0]["oscrs_score"] - expected_norm) < 1e-12
    print("OK")


if __name__ == "__main__":
    main()
