#!/usr/bin/env python3
"""Offline model-v1 replay from the production symbolic source (no ROS).

Measured inputs are [ax_C, ay_C, omega_actual, alpha_actual]. OCP replay needs
the entire 24/28-state x0, including final-command delay queues, plus the
recorded per-stage parameters and [a_cmd, alpha_cmd, v_s] controls. It does not
infer actual acceleration from command acceleration or alter the initial state.

CLI JSON: {liquid_model_version: 1, x0: [...], controls: [[...], ...],
           stage_parameters: [[...], ...]}. Rows use the PreSolveSnapshot ABI.
The output contains all N+1 replayed states; this replays dynamics, not solve().
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    request = json.loads(args.input.read_text())
    states = replay_ocp(request["x0"], request["controls"], request["stage_parameters"],
                        liquid_model_version=request["liquid_model_version"])
    args.output.write_text(json.dumps({"liquid_model_version": MODEL_VERSION,
                                      "states": states.tolist()}, indent=2) + "\n")


if __name__ == "__main__":
    main()
