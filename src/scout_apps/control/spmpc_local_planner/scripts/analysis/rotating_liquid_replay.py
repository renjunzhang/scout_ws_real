#!/usr/bin/env python3
"""Offline model-v1 replay from the production symbolic source (no ROS).

Measured inputs are [ax_C, ay_C, omega_actual, alpha_actual]. OCP replay needs
the entire 24/28-state x0, including final-command delay queues, plus the
recorded per-stage parameters and [a_cmd, alpha_cmd, v_s] controls. It does not
infer actual acceleration from command acceleration or alter the initial state.

CLI JSON: {liquid_model_version: 1, x0: [...], controls: [[...], ...],
           stage_parameters: [[...], ...]}. Rows use the PreSolveSnapshot ABI.
The output contains all N+1 replayed states; this replays dynamics, not solve().

Prediction-liquid CLI JSON: {snapshot: {...}, horizon: {...},
 liquid_initial_state: [eta_x, eta_x_dot, eta_y, eta_y_dot],
 liquid_initial_epoch_ns: integer, liquid_initial_source: "observer",
 liquid_parameters: [two_zeta_omega_n, omega_n_sq, kappa_x, kappa_y],
 height_coeff: number, liquid_initial_valid: true}. The output contains
 independent ``liquid_states`` and ``h_modal`` arrays of length N+1.
"""

import argparse
from functools import lru_cache
import json
import math
from pathlib import Path
import sys

import numpy as np
import casadi as ca

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "acados"))
from model_contract import MODEL_VERSION
from slosh_kernel import integration_substeps, functions
from actual_motion_kernel import functions as actual_motion_functions
from ocp_snapshot_contract import validate_snapshot
from spmpc_acados_model import (
    PIDX, PIDX_SLOSH, export_spmpc_b0_symbols, export_spmpc_slosh_symbols,
)


@lru_cache(maxsize=1)
def _measured_kernel():
    return functions()[1]


def measured_step(state, excitation, liquid_parameters, dt_sec):
    """Same held-input RK4 policy as SloshDynamics::stepWithDt()."""
    x = np.asarray(state, dtype=float).reshape(4)
    excitation = np.asarray(excitation, dtype=float).reshape(4)
    liquid = np.asarray(liquid_parameters, dtype=float).reshape(4)
    dt = float(dt_sec)
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(excitation))
            and np.all(np.isfinite(liquid)) and math.isfinite(dt) and 0 <= dt <= 1):
        raise ValueError("invalid rotating-liquid state, excitation, parameters or dt")
    if liquid[0] < 0 or liquid[1] <= 0:
        raise ValueError("invalid modal damping or stiffness")
    if dt == 0:
        return tuple(x)
    count = integration_substeps(dt)
    kernel = _measured_kernel()
    for _ in range(count):
        x = np.asarray(kernel(x, excitation, liquid, dt/count)).ravel()
    if not np.all(np.isfinite(x)):
        raise ValueError("nonfinite rotating-liquid propagation")
    return tuple(x)


@lru_cache(maxsize=2)
def _ocp_kernel(with_slosh):
    sym = (export_spmpc_slosh_symbols if with_slosh else export_spmpc_b0_symbols)()
    return ca.Function("replay_" + sym["name"], [sym[k] for k in ("x", "u", "p")],
                       [sym["disc_dyn"]]), sym["nx"], sym["np"]


def replay_ocp(x0, controls, stage_parameters, *, liquid_model_version):
    if liquid_model_version != MODEL_VERSION:
        raise ValueError("rotating replay requires liquid_model_version=1")
    x = np.asarray(x0, dtype=float)
    if x.shape not in ((24,), (28,)):
        raise ValueError("x0 must contain all 24/28 explicit actuator states")
    with_slosh = len(x) == 28
    kernel, nx, np_ = _ocp_kernel(with_slosh)
    u = np.asarray(controls, dtype=float)
    p = np.asarray(stage_parameters, dtype=float)
    if u.ndim != 2 or u.shape[1] != 3 or not len(u):
        raise ValueError("controls must have shape (N,3) with N>0")
    if p.shape not in ((len(u), np_), (len(u)+1, np_)):
        raise ValueError("stage_parameters must contain N or N+1 recorded rows")
    if not all(np.all(np.isfinite(z)) for z in (x, u, p)):
        raise ValueError("nonfinite replay input")
    pi = PIDX_SLOSH if with_slosh else PIDX
    # Version 1 is frozen at 30 Hz: the online integration policy uses 4 steps.
    if not np.allclose(p[:, pi["actuator_dt"]], 1/30, atol=1e-8, rtol=0):
        raise ValueError("model v1 replay requires recorded actuator_dt=1/30")
    if np.any(p[:, [pi["actuator_tau_v"], pi["actuator_tau_omega"]]] <= 0):
        raise ValueError("actuator time constants must be positive")
    if with_slosh and (np.any(p[:, pi["omega_n_sq"]] <= 0)
                       or np.any(p[:, pi["two_zeta_omega_n"]] < 0)):
        raise ValueError("invalid modal damping or stiffness")
    states = np.empty((len(u)+1, nx)); states[0] = x
    for k, control in enumerate(u):
        states[k+1] = np.asarray(kernel(states[k], control, p[k])).ravel()
    if not np.all(np.isfinite(states)):
        raise ValueError("nonfinite OCP propagation")
    return states


@lru_cache(maxsize=1)
def _actual_motion_kernel():
    return actual_motion_functions()[0]


_TRUSTED_LIQUID_INITIAL_SOURCES = frozenset(
    ("observer", "observer_continuous", "aligned_observer"))


def evaluate_prediction_liquid(
    snapshot, horizon, liquid_initial_state, liquid_initial_epoch_ns,
    liquid_parameters, height_coeff, liquid_initial_source, liquid_initial_valid,
):
    """Reconstruct liquid height from published OCP actual actuator states.

    The liquid state is supplied by the caller from the same observer stream at
    ``solver_input_epoch_ns``. It is never taken from B0's zero liquid tail,
    initialized implicitly, or re-created by replaying command controls. Each
    interval derives actual ``a``/``alpha`` from that stage's published actual
    and delayed actuator states and parameters, then advances only the
    continuous liquid memory with the shared four-substep RK4 kernel.
    """
    if not isinstance(snapshot, dict) or not isinstance(horizon, dict):
        raise ValueError("snapshot and horizon must be JSON objects")
    if snapshot.get("valid") is not True or horizon.get("valid") is not True:
        raise ValueError("snapshot and horizon must have valid=true")
    checked = validate_snapshot(snapshot)
    if checked["schema_version"] not in (8, 9) or horizon.get("schema_version") != checked["schema_version"] or \
            horizon.get("cost_model_version") != 3 or horizon.get("liquid_model_version") != MODEL_VERSION:
        raise ValueError("prediction evaluation requires schema8/cost3/liquid1 records")
    stages = int(horizon.get("horizon_steps", -1))
    if stages <= 0 or stages != horizon.get("horizon_steps") or stages != int(snapshot["horizon_steps"]):
        raise ValueError("snapshot and horizon horizon_steps differ")
    if "cycle_id" not in snapshot or horizon.get("cycle_id") != snapshot["cycle_id"]:
        raise ValueError("snapshot and horizon cycle_id differ")
    if "solver_input_epoch_ns" not in snapshot or "solver_input_epoch_ns" not in horizon:
        raise ValueError("snapshot and horizon require solver_input_epoch_ns")
    epoch = snapshot["solver_input_epoch_ns"]
    if type(epoch) is not int or type(horizon["solver_input_epoch_ns"]) is not int or \
            epoch <= 0 or horizon["solver_input_epoch_ns"] != epoch:
        raise ValueError("snapshot and horizon solver_input_epoch_ns must match as integers")
    state_width = int(horizon.get("model_state_width", -1))
    if state_width != checked["state_width"]:
        raise ValueError("snapshot and horizon state widths differ")
    try:
        model_states = np.asarray(horizon["model_states"], dtype=float).reshape((stages + 1, state_width))
        dt = float(horizon["dt"])
        snapshot_dt = float(snapshot["dt"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("horizon model_states and dt are required")
    if not np.all(np.isfinite(model_states)) or not math.isfinite(dt) or not math.isfinite(snapshot_dt):
        raise ValueError("horizon model_states and dt must be finite")
    if abs(dt - snapshot_dt) > 1e-8 or abs(dt - 1.0 / 30.0) > 1e-8:
        raise ValueError("model v1 prediction evaluation requires matching dt=1/30")
    if liquid_initial_valid is not True:
        raise ValueError("liquid initial state requires liquid_initial_valid=true")
    if type(liquid_initial_epoch_ns) is not int or liquid_initial_epoch_ns != epoch:
        raise ValueError("liquid initial state epoch must equal solver_input_epoch_ns")
    if liquid_initial_source not in _TRUSTED_LIQUID_INITIAL_SOURCES:
        raise ValueError("liquid initial state source must be an aligned observer source")
    q = np.asarray(liquid_initial_state, dtype=float).reshape(-1)
    liquid = np.asarray(liquid_parameters, dtype=float).reshape(-1)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("liquid_initial_state must contain four finite values")
    if liquid.shape != (4,) or not np.all(np.isfinite(liquid)) or liquid[0] < 0 or liquid[1] <= 0:
        raise ValueError("liquid_parameters require finite coefficients, nonnegative damping and positive stiffness")
    height = float(height_coeff)
    if not math.isfinite(height) or height <= 0.0:
        raise ValueError("height_coeff must be positive and finite")

    width = checked["parameter_width"]
    parameters = np.asarray(snapshot["stage_parameters"], dtype=float).reshape((stages + 1, width))
    if not np.allclose(parameters[:, PIDX["actuator_dt"]], dt, atol=1e-8, rtol=0):
        raise ValueError("snapshot actuator_dt differs from horizon dt")
    states = [q.copy()]
    heights = [height * math.hypot(q[0], q[2])]
    kernel = _actual_motion_kernel()
    for stage in range(stages):
        row = parameters[stage]
        tau_v = float(row[PIDX["actuator_tau_v"]])
        tau_omega = float(row[PIDX["actuator_tau_omega"]])
        gain_v = float(row[PIDX["actuator_gain_v"]])
        gain_omega = float(row[PIDX["actuator_gain_omega"]])
        if not (tau_v > 0.0 and tau_omega > 0.0 and
                math.isfinite(gain_v) and math.isfinite(gain_omega)):
            raise ValueError("invalid actuator parameters at stage {}".format(stage))
        motion = np.asarray([0.0, 0.0, 0.0, model_states[stage, 3], model_states[stage, 5],
                             q[0], q[1], q[2], q[3]], dtype=float)
        delayed = np.asarray([model_states[stage, 8], model_states[stage, 13]], dtype=float)
        actuator = np.asarray([tau_v, tau_omega, gain_v, gain_omega], dtype=float)
        for _ in range(4):
            motion = np.asarray(kernel(motion, delayed, actuator, liquid, dt / 4.0)).ravel()
        if not np.all(np.isfinite(motion)):
            raise ValueError("nonfinite predicted liquid propagation")
        q = motion[5:9].copy()
        states.append(q.copy())
        heights.append(height * math.hypot(q[0], q[2]))
    return {
        "schema_version": 1,
        "liquid_model_version": MODEL_VERSION,
        "cycle_id": snapshot.get("cycle_id"),
        "solver_input_epoch_ns": epoch,
        "liquid_initial_source": liquid_initial_source,
        "liquid_parameters": liquid.tolist(),
        "height_coeff": height,
        "liquid_states": np.asarray(states).tolist(),
        "h_modal": heights,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    request = json.loads(args.input.read_text())
    if "snapshot" in request and "horizon" in request:
        result = evaluate_prediction_liquid(
            request["snapshot"], request["horizon"], request["liquid_initial_state"],
            request["liquid_initial_epoch_ns"], request["liquid_parameters"],
            request["height_coeff"], request["liquid_initial_source"],
            request["liquid_initial_valid"])
    elif "x0" in request and "controls" in request and "stage_parameters" in request:
        # Keep the original full OCP replay CLI available for existing users.
        states = replay_ocp(
            request["x0"], request["controls"], request["stage_parameters"],
            liquid_model_version=request["liquid_model_version"])
        result = {"liquid_model_version": MODEL_VERSION, "states": states.tolist()}
    else:
        raise ValueError("input requires prediction snapshot/horizon or legacy x0/controls")
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
