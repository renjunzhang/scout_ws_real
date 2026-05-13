"""Candidate diversity diagnostics for OSCRS.

This module is report-only: it does not participate in feasibility gates,
hard gates, or scoring.
"""

import math


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value)


def _spread(values):
    vals = [float(v) for v in values if _finite(v)]
    if len(vals) < 2:
        return 0.0
    vmax = max(vals)
    vmin = min(vals)
    return (vmax - vmin) / max(1e-9, abs(vmax))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_sep(a, b):
    if not a or not b:
        return 0.0
    n = min(len(a), len(b))
    if n <= 0:
        return 0.0
    return max(_dist(a[i], b[i]) for i in range(n))


def _max_path_sep(items):
    best = 0.0
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            best = max(best, _path_sep(items[i][1], items[j][1]))
    return best


def candidate_diversity(rows, path_sep_threshold=0.03, metric_spread_threshold=0.05):
    """Return summary diversity metrics for candidate_report.

    Args:
        rows: list[(row, points)] from pipeline.

    Returns:
        dict with stable report keys.
    """
    usable = [(row, points) for row, points in rows if row.get("accepted", False)]
    if len(usable) < 2:
        usable = list(rows)
    row_items = [row for row, _ in usable]

    div_path = _max_path_sep(usable)
    div_kappa = _spread(row.get("kappa_p95", 0.0) for row in row_items)
    div_dkappa = _spread(row.get("dkappa_p95", 0.0) for row in row_items)
    div_ay = _spread(row.get("predicted_ay_p95", 0.0) for row in row_items)
    div_ax = _spread(row.get("predicted_ax_p95", 0.0) for row in row_items)
    div_ax_pulse = _spread(row.get("predicted_ax_peak", 0.0) for row in row_items)
    div_sH = _spread(row.get("slosh_h_p95", 0.0) for row in row_items)
    div_sE = _spread(row.get("slosh_energy_rms", 0.0) for row in row_items)
    div_eta_x = _spread(row.get("slosh_eta_x_p95", 0.0) for row in row_items)
    div_eta_y = _spread(row.get("slosh_eta_y_p95", 0.0) for row in row_items)

    eta_x_level = max((row.get("slosh_eta_x_p95", 0.0) for row in row_items if _finite(row.get("slosh_eta_x_p95", 0.0))), default=0.0)
    eta_y_level = max((row.get("slosh_eta_y_p95", 0.0) for row in row_items if _finite(row.get("slosh_eta_y_p95", 0.0))), default=0.0)
    if eta_x_level > 1.2 * eta_y_level:
        dominant = "eta_x"
        aligned = div_eta_x >= metric_spread_threshold or div_ax >= metric_spread_threshold
    elif eta_y_level > 1.2 * eta_x_level:
        dominant = "eta_y"
        aligned = div_eta_y >= metric_spread_threshold or div_ay >= metric_spread_threshold
    else:
        dominant = "balanced"
        aligned = max(div_eta_x, div_eta_y, div_ax, div_ay) >= metric_spread_threshold

    max_metric_spread = max(
        div_kappa, div_dkappa, div_ay, div_ax, div_ax_pulse,
        div_sH, div_sE, div_eta_x, div_eta_y,
    )
    collapse = div_path < path_sep_threshold and max_metric_spread < metric_spread_threshold
    return {
        "div_path": div_path,
        "div_kappa": div_kappa,
        "div_dkappa": div_dkappa,
        "div_ay": div_ay,
        "div_ax": div_ax,
        "div_ax_pulse": div_ax_pulse,
        "div_sH": div_sH,
        "div_sE": div_sE,
        "div_eta_x": div_eta_x,
        "div_eta_y": div_eta_y,
        "dominant_excitation": dominant,
        "diversity_aligned": int(bool(aligned)),
        "candidate_collapse": int(bool(collapse)),
    }
