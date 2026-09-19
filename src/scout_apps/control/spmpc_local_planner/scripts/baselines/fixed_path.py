"""Fixed-duration Scout adaptation of Ferrari 2026, III-A / equation (19).

Geometry is frozen before solving. Only its time law and the commands needed
by the delayed, nonholonomic vehicle are decisions. The objective is progress
jerk, with a small angular-command regularizer; liquid height is a constraint,
not the Full planner's running objective. This is not a time-optimal replica.
"""
import hashlib
import json
import time

import casadi as ca
import numpy as np
from scipy.interpolate import CubicSpline

from planning.task import load_task, halfspaces
from planning.optimizer import dynamics, PlanValidationError
from planning.validation import validate_plan
from planning.warm_start import warm_start_values
from planning.diagnostics import solver_summary
from planning.liquid_policy import height_limits
from model_contract import MODEL_VERSION, COST_VERSION
from actual_motion_kernel import functions as actual_motion_functions


def fixed_geometry(task, knot_count=13):
    """Deterministic clamped cubic through resampled *original* route points.

    The endpoint heading condition adapts the SCARA path to a differential
    drive. Position anchoring uses the declared physical initial pose. These
    edits are frozen and reported before optimization, never optimized.
    """
    route = np.asarray(task["route"], float)
    route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    length = route_s[-1]
    knots = np.linspace(0., length, min(knot_count, len(route)))
    points = np.column_stack([np.interp(knots, route_s, route[:, i]) for i in (0, 1)])
    points[0] = task["start_state"][:2]
    points[-1] = task["goal_pose"][:2]
    yaw0, yaw1 = task["start_state"][2], task["goal_pose"][2]
    tangent0 = np.array([np.cos(yaw0), np.sin(yaw0)])
    tangent1 = np.array([np.cos(yaw1), np.sin(yaw1)])
    spline = CubicSpline(knots, points, bc_type=((1, tangent0), (1, tangent1)))
    s = ca.MX.sym("path_coordinate")
    value = ca.DM(points[-1])
    for j in reversed(range(len(knots)-1)):
        q = s-knots[j]
        polynomial = sum(ca.DM(spline.c[i, j]) * q**(3-i) for i in range(4))
        value = ca.if_else(s < knots[j+1], polynomial, value)
    # Use the final segment at and just beyond L as well, preserving derivatives
    # at the endpoint instead of introducing a constant branch there.
    q = s-knots[-2]
    end = sum(ca.DM(spline.c[i, -1]) * q**(3-i) for i in range(4))
    value = ca.if_else(s >= knots[-2], end, value)
    function = ca.Function("fixed_path_geometry", [s], [value])
    dense_s = np.linspace(0, length, 2001)
    dense = spline(dense_s)
    original = np.column_stack([np.interp(dense_s, route_s, route[:, i]) for i in (0, 1)])
    derivative = spline(dense_s, 1)
    if np.min(np.linalg.norm(derivative, axis=1)) < .1:
        raise ValueError("fixed geometry has a nearly singular tangent")
    normal = np.array([-tangent0[1], tangent0[0]])
    straight = bool(np.max(np.abs((dense-points[0]) @ normal)) < 1e-10 and
                    np.max(np.abs(derivative @ normal)) < 1e-10)
    identity = dict(method="clamped_cubic_original_route", knots=knots.tolist(), points=points.tolist(),
        polynomial_coefficients=spline.c.tolist(),
        maximum_route_edit_m=float(np.linalg.norm(dense-original, axis=1).max()),
        frozen_path_length_m=float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum()),
        straight=straight, original_route_sha256=hashlib.sha256(route.tobytes()).hexdigest())
    return function, spline, identity


def geometry_error(task, states, controls, spline):
    """Bound grid and held-command substep deviation from frozen geometry."""
    moving = round(task["transport_duration"]/task["dt"])
    node = np.linalg.norm(states[:moving+1, :2]-spline(states[:moving+1, 4]), axis=1)
    step = actual_motion_functions()[0]
    dense = float(node.max())
    for k in range(moving):
        x = states[k]
        motion = np.r_[x[:4], x[5], x[24:28]]
        for j in range(1, 5):
            motion = np.asarray(step(motion, x[[8, 13]], task["actuator_parameters"],
                task["liquid_parameters"], task["dt"]/4)).ravel()
            s = x[4]+controls[k, 2]*task["dt"]*j/4
            dense = max(dense, float(np.linalg.norm(motion[:2]-spline(s))))
    return dict(maximum_node_path_error_m=float(node.max()), maximum_substep_path_error_m=dense,
                substeps_per_interval=4, node_tolerance_m=2e-6, substep_tolerance_m=.001)


def solve_fixed_path(source, warm_plan=None, *, progress=None, solver_verbosity=0):
    emit = progress if progress is not None else lambda stage, **details: None
    task = load_task(source)
    if task.get("terminal_speed_policy"):
        raise ValueError("FP-AS uses scheduled braking, not Full's progress slowdown envelope")
    if task.get("liquid_policy") is None:
        raise ValueError("FP-AS requires explicit transport and parking liquid limits")
    start = np.asarray(task["start_state"], float)
    if np.max(np.abs(start[3:24])) > 1e-10:
        raise ValueError("initial FP-AS release supports stationary command/history only")
    options = task.get("fixed_path_options", {})
    if set(options)-{"tail_guard_sec", "progress_jerk_max"}:
        raise ValueError("unknown fixed-path option")
    tail_guard = options.get("tail_guard_sec", 2.)
    jerk_max = options.get("progress_jerk_max", task["motion_limits"]["jerk_max"])
    if not np.isfinite([tail_guard, jerk_max]).all() or not 0 < tail_guard < task["transport_duration"] or jerk_max <= 0:
        raise ValueError("invalid fixed-path tail guard or jerk bound")
    geometry, spline, geometry_info = fixed_geometry(task)
    emit("geometry_frozen", **geometry_info)
    dt = task["dt"]
    moving = round(task["transport_duration"]/dt)
    count = round((task["deadline"]+task["stop_window"])/dt)
    clear = moving-11
    if clear <= 10:
        raise ValueError("transport interval cannot drain the actuator")
    route = np.asarray(task["route"], float)
    length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
    limits = task["motion_limits"]
    F = dynamics(task)
    opt = ca.Opti()
    X, U = opt.variable(28, count+1), opt.variable(3, count)
    opt.subject_to(X[:, 0] == start)
    opt.subject_to(X[:, 1:] == F.map(count)(X[:, :-1], U))
    opt.subject_to(opt.bounded(limits["actual_v_min"], X[3, :], limits["v_max"]))
    opt.subject_to(opt.bounded(-limits["omega_max"], X[5, :], limits["omega_max"]))
    opt.subject_to(opt.bounded(0, X[6, 1:clear], limits["v_max"]))
    opt.subject_to(opt.bounded(0, X[4, 6:moving], length))
    opt.subject_to(U[2, :5] == 0)  # Immutable stationary linear FIFO prefix.
    opt.subject_to(opt.bounded(0, U[2, 5:moving], limits["v_max"]))
    opt.subject_to(U[2, moving:] == 0)
    opt.subject_to(X[4, moving] == length)
    opt.subject_to(opt.bounded(-limits["a_max"], U[0, :clear], limits["a_max"]))
    opt.subject_to(opt.bounded(-limits["jerk_max"]*dt, U[0, :]-X[23, :-1], limits["jerk_max"]*dt))
    opt.subject_to(X[6, clear] == 0)
    opt.subject_to(U[0, clear:] == 0)
    if geometry_info["straight"]:
        # Exact invariant of the full unicycle/actuator model; avoid redundant
        # y equalities at zero lateral motion, which make the NLP rank deficient.
        opt.subject_to(U[1, :] == 0)
        direction = ca.DM([np.cos(start[2]), np.sin(start[2])])
        for k in range(6, moving+1):
            opt.subject_to(ca.dot(X[:2, k]-geometry(X[4, k]), direction) == 0)
    else:
        opt.subject_to(opt.bounded(-limits["omega_max"], X[7, 1:clear], limits["omega_max"]))
        opt.subject_to(opt.bounded(-limits["alpha_max"], U[1, :clear], limits["alpha_max"]))
        opt.subject_to(X[7, clear] == 0)
        opt.subject_to(U[1, clear:] == 0)
        opt.subject_to(X[:2, 6:moving+1] == geometry.map(moving-5)(X[4, 6:moving+1]))
    goal = np.asarray(task["goal_pose"], float)
    opt.subject_to(ca.sum1((X[:2, moving:]-ca.repmat(ca.DM(goal[:2]), 1, count-moving+1))**2) <= task["goal_position_tolerance"]**2)
    yaw_goal = start[2]+np.arctan2(np.sin(goal[2]-start[2]), np.cos(goal[2]-start[2]))
    opt.subject_to(opt.bounded(-task["goal_yaw_tolerance"], X[2, moving:]-yaw_goal, task["goal_yaw_tolerance"]))
    opt.subject_to(opt.bounded(-task["stop_speed_tolerance"], X[3, moving:], task["stop_speed_tolerance"]))
    opt.subject_to(opt.bounded(-task["stop_omega_tolerance"], X[5, moving:], task["stop_omega_tolerance"]))
    caps = height_limits(task, np.arange(count+1)*dt, task["transport_duration"]-tail_guard)
    height_sq = task["height_coeff"]**2*(X[24, :]**2+X[26, :]**2)
    opt.subject_to(height_sq/ca.DM(caps).T**2 <= 1)
    # s is in metres of original-route coordinate, not a free geometry variable.
    speed_sequence = ca.horzcat(0., 0., U[2, :])
    jerk = (speed_sequence[2:]-2*speed_sequence[1:-1]+speed_sequence[:-2])/dt**2
    opt.subject_to(opt.bounded(-jerk_max, jerk, jerk_max))
    objective = dt*ca.sumsqr(jerk/jerk_max)
    if not geometry_info["straight"]:
        objective += .01*dt*ca.sumsqr((U[1, 1:]-U[1, :-1])/(dt*limits["alpha_max"]))
    region_ids = []
    if len(task["region"]["cells"]) != 1:
        raise ValueError("initial fixed-path release requires one explicit convex region")
    cell = task["region"]["cells"][0]
    normal, offset = halfspaces(task["region"], cell, max(limits["v_max"], abs(limits["actual_v_min"]))*dt)
    opt.subject_to(ca.vec(ca.DM(normal) @ X[:2, :] - ca.repmat(ca.DM(offset), 1, count+1)) <= 0)
    region_ids = [cell["id"]]*(count+1)
    opt.minimize(objective)
    if warm_plan is not None:
        warm_x, warm_u, _ = warm_start_values(warm_plan, task, count)
        opt.set_initial(X, warm_x); opt.set_initial(U, warm_u)
    else:
        q = np.clip((np.arange(count+1)-5)/max(1, clear-5), 0, 1)
        s = length*(10*q**3-15*q**4+6*q**5)
        derivative = spline(s, 1)
        yaw = np.unwrap(np.arctan2(derivative[:, 1], derivative[:, 0]))
        v = np.linalg.norm(derivative, axis=1)*np.gradient(s, dt)
        omega = np.gradient(yaw, dt)
        commands = []
        for value, delay, tau, gain, bound in zip((v, omega), (5, 10), task["actuator_parameters"][:2],
                task["actuator_parameters"][2:], (limits["v_max"], limits["omega_max"])):
            index = np.minimum(np.arange(count+1)+delay, count)
            command = (value[index]+tau*np.gradient(value, dt)[index])/gain
            commands.append(np.clip(command, 0 if len(commands) == 0 else -bound, bound))
        commands = np.asarray(commands).T
        commands[0] = 0.; commands[clear:] = 0.
        warm_u = np.column_stack((np.diff(commands, axis=0)/dt, np.diff(s)/dt))
        guess = np.zeros((count+1, 28)); guess[0] = start
        for k in range(count):
            guess[k+1] = np.asarray(F(guess[k], warm_u[k])).ravel()
        guess[:, :2] = spline(s); guess[:, 4] = s; guess[0] = start
        opt.set_initial(X, guess.T); opt.set_initial(U, warm_u.T)
    opt.solver("ipopt", {"print_time": False}, dict(print_level=solver_verbosity,
        max_iter=task["max_iterations"], tol=task["constraint_tolerance"],
        constr_viol_tol=task["constraint_tolerance"], acceptable_tol=task["constraint_tolerance"],
        bound_relax_factor=0., fixed_variable_treatment="make_constraint"))
    begun = time.monotonic()
    emit("solve_started", variables=int(opt.nx), constraints=int(opt.ng), transport_duration_sec=task["transport_duration"])
    try:
        solution = opt.solve()
    except RuntimeError:
        emit("solve_failed", **solver_summary(opt.stats()))
        raise
    emit("solve_finished", **solver_summary(solution.stats()))
    states = np.asarray(solution.value(X)).T
    controls = np.vstack((np.asarray(solution.value(U)).T, np.zeros(3)))
    path_check = geometry_error(task, states, controls, spline)
    if (path_check["maximum_node_path_error_m"] > path_check["node_tolerance_m"] or
            path_check["maximum_substep_path_error_m"] > path_check["substep_tolerance_m"]):
        raise ValueError("fixed geometry node/substep check failed after solve: "+str(path_check))
    plan = {key: task[key] for key in ("frame_id", "region", "dt", "transport_duration", "deadline", "stop_window",
        "height_coeff", "actuator_parameters", "liquid_parameters", "motion_limits", "goal_pose",
        "goal_position_tolerance", "goal_yaw_tolerance", "stop_speed_tolerance", "stop_omega_tolerance", "route", "liquid_policy")}
    identity = hashlib.sha256(json.dumps(task, sort_keys=True).encode()).hexdigest()[:12]
    plan.update(schema_version=1, liquid_model_version=MODEL_VERSION, cost_model_version=COST_VERSION,
        plan_id=task["task_id"]+"-fp-as-"+identity, region_id=task["region"]["id"], task=task,
        optimization=dict(status=solution.stats()["return_status"], solve_seconds=time.monotonic()-begun,
                          iterations=solution.stats()["iter_count"], objective=float(solution.value(objective))),
        baseline=dict(method="FP-AS", source_doi="10.1109/LRA.2025.3643281", fixed_duration=True,
            time_optimal_reproduction=False, fixed_geometry=geometry_info,
            geometry_validation=path_check, tail_guard_sec=tail_guard,
            objective="integral squared discrete progress jerk; curved paths add 0.01 angular command jerk regularization",
            progress_jerk_max=jerk_max, online_liquid_optimization=False))
    brake = int(np.flatnonzero(controls[:moving, 0] > 1e-6)[-1]+1) if np.any(controls[:moving, 0] > 1e-6) else 0
    plan["samples"] = [dict(t=k*dt, state=x.tolist(), control=u.tolist(), region_cell_id=region_ids[k],
        phase="TAIL" if k >= moving else ("BRAKE" if k >= brake else "MOVE")) for k, (x, u) in enumerate(zip(states, controls))]
    try:
        plan["validation"] = validate_plan(plan)
    except ValueError as error:
        raise PlanValidationError(str(error), plan) from error
    emit("validation_finished", **plan["validation"])
    return plan
