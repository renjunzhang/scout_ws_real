#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke test for max_candidate_level as G-layer policy with visible rows."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.feasibility import evaluate_geometry_feasibility  # noqa: E402
from oscrs.generators.georef import generate_georef_candidates_with_meta  # noqa: E402
from oscrs.path_utils import sanitize_points  # noqa: E402
from reference_generation.geometry_candidates import path_metrics  # noqa: E402


def main():
    base = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.0), (3.0, -0.1)]
    specs = [
        ("original", 0, 0.0, 0.0),
        ("mild", 2, 0.1, 0.02),
        ("medium", 4, 0.2, 0.04),
    ]
    levels = {name: i for i, (name, _, _, _) in enumerate(specs)}
    candidates, meta = generate_georef_candidates_with_meta(
        base, specs, 0.01, sanitize_points, levels, "mild",
    )
    names = [name for name, _ in candidates]
    assert names == ["original", "mild", "medium"]
    assert meta["generation_policy"]["medium"]["skipped"]

    metrics = path_metrics(base, base)
    row = evaluate_geometry_feasibility(
        candidates[2][1], "medium", base, metrics, metrics["length_m"],
        0.01, 10.0, 10.0, 0.0, 0.0, 0.35,
        10.0, 10.0,
        0.1, 1.0, 0.1, 1.0,
        True, meta["generation_policy"]["medium"]["reason"],
        "accepted", -1, -1,
        False, 0.0, 0.0, 0.05, 10.0,
        1.0, 0.5, 0.3, 0.5, 10.0, 2.0,
    )
    assert not row["accepted"]
    assert row["reject_stage"] == "GENERATION_SKIPPED"
    assert row["reject_reason"].startswith("level:medium>mild")
    print("OK")


if __name__ == "__main__":
    main()
