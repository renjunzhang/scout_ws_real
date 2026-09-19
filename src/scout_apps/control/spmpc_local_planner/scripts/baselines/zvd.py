"""Straight-line ZVD (Guagliumi et al., DOI 10.1115/1.4054224).

Shape a raised-cosine command-speed pulse on the production clock. Fractional
impulses are split between adjacent ticks, preserving weight and mean delay;
the discretization error is reported, never hidden by compressing the clock.
The exported reference includes actual motion, actuator history and liquid.
"""
from copy import deepcopy
import hashlib
import json
import math
import time

import numpy as np
from scipy.linalg import expm

from planning.task import load_task
from planning.optimizer import dynamics
from planning.validation import validate_plan
from model_contract import MODEL_VERSION, COST_VERSION


def impulses(liquid_parameters, dt):
    damping, omega_sq = liquid_parameters[:2]
    omega = math.sqrt(omega_sq)
    zeta = damping / (2 * omega)
    if not 0 <= zeta < 1 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("ZVD requires a finite positive clock and underdamped liquid")
    half_period = math.pi / (omega * math.sqrt(1-zeta*zeta))
    decay = math.exp(-zeta * math.pi / math.sqrt(1-zeta*zeta))
    weights = np.asarray([1., 2*decay, decay*decay]) / (1+decay)**2
    times = np.arange(3)*half_period
    kernel = np.zeros(math.ceil(times[-1]/dt)+1)
    for t, weight in zip(times, weights):
        tick = t/dt
        left = math.floor(tick)
        fraction = tick-left
        kernel[left] += weight*(1-fraction)
        if fraction > 0:
            kernel[left+1] += weight*fraction
    return times, weights, kernel


def raised_cosine_speed(duration, ramp, dt):
    if (not np.isfinite([duration, ramp, dt]).all() or not 0 < 2*ramp <= duration
            or dt <= 0 or abs(duration/dt-round(duration/dt)) > 1e-7):
        raise ValueError("base duration must be on grid with two positive ramps")
    t = np.arange(round(duration/dt)+1)*dt
    up = .5*(1-np.cos(np.pi*np.clip(t/ramp, 0, 1)))
    down = .5*(1-np.cos(np.pi*np.clip((duration-t)/ramp, 0, 1)))
    return up*down


def ideal_liquid(speed, task):
    """Exact modal response to the piecewise-linear ideal speed reference."""
    damping, omega_sq, kappa, _ = task["liquid_parameters"]
    system = np.array([[0., 1., 0.], [-omega_sq, -damping, -kappa], [0., 0., 0.]])
    step = expm(system*task["dt"])
    state = np.zeros(2)
    response = [state.copy()]
    for acceleration in np.diff(speed)/task["dt"]:
        state = (step @ np.r_[state, acceleration])[:2]
        response.append(state.copy())
    return task["height_coeff"]*np.asarray(response)


def generate_plan(source, *, base_duration, ramp_duration=2.):
    begun = time.monotonic()
    task = load_task(source)
    dt = task["dt"]
    start = np.asarray(task["start_state"], float)
    route = np.asarray(task["route"], float)
    direction = np.array([math.cos(start[2]), math.sin(start[2])])
    length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
    along = (route-start[:2]) @ direction
    normal = (route-start[:2]) @ np.array([-direction[1], direction[0]])
    if (np.max(np.abs(normal)) > 1e-8 or abs(along[0]) > 1e-8
            or np.any(np.diff(along) <= 0) or np.max(np.abs(start[3:24])) > 1e-10
            or abs(np.arctan2(np.sin(task["goal_pose"][2]-start[2]),
                              np.cos(task["goal_pose"][2]-start[2]))) > 1e-8):
        raise ValueError("ZVD supports a stationary, forward straight route aligned with initial yaw")
    if task.get("liquid_policy") or task.get("terminal_speed_policy") or task["liquid_constraint_enable"]:
        raise ValueError("ZVD requires a public feasibility task, without Full-specific qualification policies")
    times, weights, kernel = impulses(task["liquid_parameters"], dt)
    base = raised_cosine_speed(base_duration, ramp_duration, dt)
    shaped = np.convolve(base, kernel)
    moving = round(task["transport_duration"]/dt)
    count = round((task["deadline"]+task["stop_window"])/dt)
    if len(shaped)+11 > moving:
        raise ValueError("transport duration must include shaping and full FIFO/acceleration drain")
    base = np.pad(base, (0, count+1-len(base)))
    shaped = np.pad(shaped, (0, count+1-len(shaped)))
    # The linear actuator preserves DC area up to its calibrated gain. No
    # inverse dynamics, instantaneous plant or reset is used to execute it.
    amplitude = length / (dt*np.sum(base)*task["actuator_parameters"][2])
    base *= amplitude
    shaped *= amplitude
    F = dynamics(task)
    states = np.empty((count+1, 28)); states[0] = start
    controls = np.zeros((count+1, 3))
    for k in range(count):
        controls[k, 0] = (shaped[k+1]-states[k, 6])/dt
        states[k+1] = np.asarray(F(states[k], controls[k])).ravel()
    # Geometric progress follows actual speed, in the existing plan coordinate.
    scale = length / (dt*np.sum(.5*(states[:moving, 3]+states[1:moving+1, 3])))
    controls[:moving, 2] = scale*.5*(states[:moving, 3]+states[1:moving+1, 3])
    states[:, 4] = np.r_[0., np.cumsum(controls[:-1, 2])*dt]
    plan = {key: deepcopy(task[key]) for key in (
        "frame_id", "region", "dt", "transport_duration", "deadline", "stop_window", "height_coeff",
        "actuator_parameters", "liquid_parameters", "motion_limits", "goal_pose", "goal_position_tolerance",
        "goal_yaw_tolerance", "stop_speed_tolerance", "stop_omega_tolerance", "route")}
    identity = hashlib.sha256(json.dumps(task, sort_keys=True).encode()).hexdigest()[:12]
    brake = int(np.flatnonzero(np.diff(shaped) < -1e-12)[0])
    cells = task["region"]["cells"]
    plan.update(schema_version=1, liquid_model_version=MODEL_VERSION, cost_model_version=COST_VERSION,
                plan_id=task["task_id"]+"-zvd-"+identity, region_id=task["region"]["id"], task=task,
                progress_parameterization=dict(method="scaled_actual_speed_trapezoid", scale=scale))
    plan["samples"] = [dict(t=k*dt, state=x.tolist(), control=u.tolist(),
        phase="TAIL" if k >= moving else ("BRAKE" if k >= brake else "MOVE"),
        region_cell_id=next(c["id"] for c in cells if c["s_begin"]-1e-8 <= x[4] <= c["s_end"]+1e-8))
        for k, (x, u) in enumerate(zip(states, controls))]
    # Principle check uses a zero-initial-state ideal model, separately from
    # the production rollout, which preserves the task's liquid initial state.
    base_response = ideal_liquid(base, task)
    shaped_response = ideal_liquid(shaped, task)
    omega_sq = task["liquid_parameters"][1]
    def residual(response, end):
        k = math.ceil(end/dt-1e-9)
        return float(np.hypot(response[k, 0], response[k, 1]/math.sqrt(omega_sq)))
    original = residual(base_response, base_duration)
    filtered = residual(shaped_response, base_duration+(len(kernel)-1)*dt)
    plan["baseline"] = dict(method="ZVD", source_doi="10.1115/1.4054224",
        base_duration_sec=base_duration, ramp_duration_sec=ramp_duration,
        analytic_impulse_times_sec=times.tolist(), analytic_weights=weights.tolist(),
        discrete_kernel=kernel.tolist(), shaping_delay_sec=(len(kernel)-1)*dt,
        fractional_delay_method="adjacent_grid_weight_split", time_compression=False,
        base_distance_m=float(np.sum(base)*dt*task["actuator_parameters"][2]),
        shaped_distance_m=float(np.sum(shaped)*dt*task["actuator_parameters"][2]),
        ideal_base_residual_m=original, ideal_shaped_residual_m=filtered,
        ideal_residual_ratio=filtered/original if original else None,
        nominal_actual_stop_sec=task["transport_duration"], online_liquid_optimization=False,
        generation_seconds=time.monotonic()-begun)
    plan["validation"] = validate_plan(plan)
    return plan
