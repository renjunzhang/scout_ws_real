#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke test for pipeline row fields used by candidate_report."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.pipeline import run_pipeline  # noqa: E402
from oscrs.types import AntiSloshParams  # noqa: E402
from reference_generation.geometry_candidates import path_metrics  # noqa: E402


def main():
    base = [(0.0, 0.0), (1.0, 0.1), (2.0, 0.0), (3.0, -0.1)]
    candidates = [("original", base), ("mild", base)]
    metrics = path_metrics(base, base)
    params = AntiSloshParams(
        oscrs_shadow_enable=True,
        tail_protect_enable=True,
        tail_gate_enable=False,
    )

    result = run_pipeline(base, candidates, metrics, metrics["length_m"], {}, params)
    rows = {row["name"]: row for row, _ in result.rows}

    assert rows["original"]["tail_protect_applied"] is False
    assert rows["mild"]["tail_protect_applied"] is True
    assert rows["mild"]["tail_gate_enabled"] is False
    assert "oscrs_score" in rows["mild"]
    print("OK")


if __name__ == "__main__":
    main()
