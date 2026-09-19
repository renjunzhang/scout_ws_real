"""Longitudinal actuator-output jerk on the control grid (including FIFO shifts).

This is (a[k+1]-a[k])/dt, not continuous jerk across ZOH command jumps.
The row returns delta acceleration to keep the OCP scaling moderate.
"""
import math
import numpy as np
from model_contract import RK4_SUBSTEPS, MAX_RK4_STEP_SEC


def acceleration_delta_row(tau, gain, dt, nx=28):
    if not all(math.isfinite(v) and v > 0 for v in (tau, gain, dt)):
        raise ValueError("invalid actual jerk model")
    steps = max(RK4_SUBSTEPS, math.ceil(dt / MAX_RK4_STEP_SEC - 1e-12))
    z = -dt / (steps*tau)
    decay = (1 + z + z*z/2 + z**3/6 + z**4/24)**steps
    row = np.zeros(nx)
    row[3] = (1-decay)/tau
    row[8] = -gain*(2-decay)/tau
    row[9] = gain/tau
    return row


def actual_acceleration(states, tau, gain):
    """States use the planner layout [state, time]; works with NumPy/CasADi."""
    return (gain*states[8, :] - states[3, :])/tau


def constrain_actual_jerk(opt, states, task):
    limit = task["motion_limits"].get("actual_jerk_max", 0.)
    if limit > 0:
        tau, _, gain, _ = task["actuator_parameters"]
        acceleration = actual_acceleration(states, tau, gain)
        bound = limit * task["dt"]
        opt.subject_to(opt.bounded(-bound, acceleration[1:]-acceleration[:-1], bound))
