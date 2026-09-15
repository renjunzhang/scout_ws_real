from pathlib import Path
import sys

import casadi as ca
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "acados"))

from planning_terms import (  # noqa: E402
    MAX_REGION_FACES,
    PLANNING_PARAMETER_DEFAULTS,
    geometry_terms,
    piecewise_speed,
    region_reference_constraints,
    speed_references,
)


def symbols():
    x = ca.SX.sym("x", 24)
    p = ca.SX.sym("p", len(PLANNING_PARAMETER_DEFAULTS))
    idx = {name: i for i, name in enumerate(PLANNING_PARAMETER_DEFAULTS)}
    return x, p, idx


def params(idx):
    values = np.array([PLANNING_PARAMETER_DEFAULTS[name]
                       for name in idx], dtype=float)
    return values


def test_legacy_defaults_are_finite_and_region_rows_are_constant():
    x, p, idx = symbols()
    rows = ca.Function("rows", [x, p], [region_reference_constraints(x, p, idx)])
    value = np.zeros(24)
    values = params(idx)
    result = np.asarray(rows(value, values)).ravel()
    assert np.isfinite(result).all()
    assert np.all(result == -1.0)


def test_low_speed_spin_has_finite_nonzero_curvature_cost():
    x, p, idx = symbols()
    values = params(idx)
    values[idx["w_curvature"]] = 2.0
    value = np.zeros(24)
    value[5] = 0.4
    fn = ca.Function("geometry", [x, p], list(geometry_terms(x, 0, 0, p, idx, 1.0)))
    curve = np.asarray(fn(value, values), dtype=float).ravel()
    assert curve[0] > 0
    assert np.isfinite(curve).all()


def test_same_speed_higher_omega_has_larger_curvature_cost():
    x, p, idx = symbols()
    values = params(idx)
    values[idx["w_curvature"]] = 1.0
    fn = ca.Function("geometry", [x, p], list(geometry_terms(x, 0, 0, p, idx, 1.0)))
    slow = np.zeros(24); slow[3] = 0.3; slow[5] = 0.1
    fast_turn = slow.copy(); fast_turn[5] = 0.4
    assert float(fn(fast_turn, values)[0]) > float(fn(slow, values)[0])


def test_actual_alpha_rhs_controls_curvature_change_and_is_finite():
    x, p, idx = symbols()
    values = params(idx)
    values[idx["w_curvature_rate"]] = 1.0
    value = np.zeros(24); value[3] = .2; value[5] = .1
    fn = ca.Function("geometry", [x, p], list(geometry_terms(x, 0, 0, p, idx, 1.0)))
    base = float(fn(value, values)[1])
    accelerated = ca.Function("geometry2", [x, p], list(geometry_terms(x, 0, .5, p, idx, 1.0)))
    changed = float(accelerated(value, values)[1])
    assert changed > base
    assert np.isfinite(changed)


def test_piecewise_progress_reference_and_fixed_mode_sources():
    x, p, idx = symbols(); values = params(idx)
    for i, val in enumerate((.1, .2, .4, .8)):
        values[idx[f"reference_v{i}"]] = val
        values[idx[f"reference_vs{i}"]] = val * .5
        values[idx[f"reference_s{i}"]] = float(i)
    values[idx["reference_mode"]] = 1
    fn = ca.Function("speed", [x, p], list(speed_references(x, p, idx, .9)))
    value = np.zeros(24); value[4] = 1.5
    v, vs = fn(value, values)
    assert float(v) == pytest.approx(.3)
    assert float(vs) == pytest.approx(.15)
    values[idx["reference_mode"]] = 2
    v, vs = fn(value, values)
    assert float(v) == pytest.approx(.1)
    assert float(vs) == pytest.approx(.05)


def test_region_halfspaces_and_progress_rows_reject_outside_point():
    x, p, idx = symbols(); values = params(idx)
    values[idx["region_active"]] = 1
    values[idx["region_nx0"]] = 1; values[idx["region_offset0"]] = 1
    values[idx["region_s_begin"]] = 0; values[idx["region_s_end"]] = 2
    values[idx["reference_mode"]] = 1
    values[idx["reference_s0"]] = 0; values[idx["reference_s3"]] = 2
    fn = ca.Function("region", [x, p], [region_reference_constraints(x, p, idx)])
    inside = np.zeros(24); inside[0] = .5; inside[4] = 1
    outside = inside.copy(); outside[0] = 2
    assert float(fn(inside, values)[0]) < 0
    assert float(fn(outside, values)[0]) > 0
    outside[4] = 3
    # Four task-goal deadline rows follow the two progress rows.
    assert float(fn(outside, values)[-5]) > 0


def test_inactive_task_goal_keeps_deadline_rows_inactive():
    x, p, idx = symbols(); values = params(idx)
    fn = ca.Function("goal_inactive", [x, p], [region_reference_constraints(x, p, idx)])
    result = np.asarray(fn(np.full(24, 1.0), values)).ravel()
    assert np.all(result[-4:] == -1.0)


def test_active_task_goal_deadline_rows_reject_pose_and_velocity():
    x, p, idx = symbols(); values = params(idx)
    values[idx["task_goal_active"]] = 1.0
    values[idx["task_goal_x"]] = 1.0; values[idx["task_goal_y"]] = 2.0
    values[idx["task_goal_position_tolerance"]] = .2
    values[idx["task_goal_speed_tolerance"]] = .1
    values[idx["task_goal_omega_tolerance"]] = .1
    fn = ca.Function("goal_active", [x, p], [region_reference_constraints(x, p, idx)])
    inside = np.zeros(24); inside[0] = 1.05; inside[1] = 2.05
    assert np.all(np.asarray(fn(inside, values)).ravel()[-4:] < 0.0)
    outside = inside.copy(); outside[0] = 2.0; outside[3] = .2; outside[5] = .2
    assert np.any(np.asarray(fn(outside, values)).ravel()[-4:] > 0.0)


def test_cost_components_reconstruct_total():
    from spmpc_acados_model import export_spmpc_b0_symbols, PIDX
    from spmpc_acados_cost import cost_components, stage_cost_expr
    from generate_spmpc_acados import load_config, default_parameter_values
    sym = export_spmpc_b0_symbols(); cfg = load_config()
    values = default_parameter_values(cfg, False)
    fn = ca.Function("cost", [sym["x"], sym["u"], sym["p"]],
                     [cost_components(sym, cfg), stage_cost_expr(sym, cfg)])
    x = np.zeros(sym["nx"]); u = np.array([.01, .02, .1])
    parts, total = fn(x, u, values)
    assert float(np.asarray(parts).sum()) == pytest.approx(float(total), abs=1e-12)
