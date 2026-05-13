#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke test for tail diagnostics/gate decoupling."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.feasibility import evaluate_geometry_feasibility  # noqa: E402
from reference_generation.geometry_candidates import path_metrics  # noqa: E402


def evaluate(tail_gate_enable):
    base = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    metrics = path_metrics(base, base)
    return evaluate_geometry_feasibility(
        base, "mild", base, metrics, metrics["length_m"],
        0.01, 10.0, 10.0, 0.0, 0.0, 0.35,
        10.0, 10.0,
        0.1, 1.0, 0.1, 1.0,
        False, "generated",
        "accepted", -1, -1,
        tail_gate_enable, 0.0, 30.0, 0.05, 10.0,
        1.0, 0.5, 0.3, 0.5, 10.0, 2.0,
    )


def main():
    diagnostic_only = evaluate(False)
    assert diagnostic_only["accepted"]
    assert diagnostic_only["tail_gate_enabled"] is False
    assert diagnostic_only["tail_heading_error_deg"] == 30.0

    gated = evaluate(True)
    assert not gated["accepted"]
    assert gated["reject_stage"] == "TERMINAL_REJECT"
    assert gated["reject_reason"].startswith("tail_heading")
    print("OK")


if __name__ == "__main__":
    main()
