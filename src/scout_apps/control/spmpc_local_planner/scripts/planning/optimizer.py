"""Multiple-shooting full-task OCP with the production explicit-actuator model.

Geometry and command timing are decisions. The given route is a weak guide;
convex region cells define the physical space. The chosen region sequence is
assigned from the seed progress schedule (a fixed corridor topology). No claim
of globally optimal topology or recursive feasibility is made.
"""
import hashlib
import json
import time
import casadi as ca
import numpy as np
from .task import load_task, model_parameters, halfspaces
from actual_jerk import constrain_actual_jerk
from .warm_start import warm_start_values
from .diagnostics import solver_summary
from .liquid_policy import height_limits, objective_end_index
from .guide import route_guide
from .terminal_speed import speed_limits
from spmpc_acados_model import export_spmpc_slosh_symbols, PIDX_SLOSH
from planning_terms import geometry_terms
from actual_motion_kernel import actual_motion_rhs
from model_contract import MODEL_VERSION, COST_VERSION


class PlanValidationError(ValueError):
    """Retain a solved but rejected candidate for diagnosis, never execution."""
    def __init__(self, message, candidate):
        super().__init__(message)
        self.candidate = candidate


def dynamics(task):
    sym = export_spmpc_slosh_symbols()
    p = model_parameters(task)
    return ca.Function("task_step", [sym["x"], sym["u"]],
                       [ca.substitute(sym["disc_dyn"], sym["p"], ca.DM(p))])


def solve_task(source, warm_plan=None, *, progress=None, solver_verbosity=0):
    emit = progress if progress is not None else lambda stage, **details: None
    if isinstance(solver_verbosity, bool) or not isinstance(solver_verbosity, int) or not 0 <= solver_verbosity <= 12:
        raise ValueError("solver_verbosity must be an integer from 0 to 12")
    task = load_task(source)
    emit("task_loaded", transport_duration=task["transport_duration"], dt=task["dt"])
    dt = task["dt"]
    moving = round(task["transport_duration"]/dt)
    count = round((task["deadline"]+task["stop_window"])/dt)
    if moving < 15:
        raise ValueError("transport interval is shorter than the delayed stopping tail")
    clear = moving-11
    limits, weights = task["motion_limits"], task["objective"]
    geometry_parameters=model_parameters(task)
    for name,key in (("w_curvature","curvature"),("w_curvature_rate","curvature_change"),
                     ("curvature_speed_floor","speed_floor"),("curvature_ref","curvature_scale"),
                     ("curvature_rate_ref","curvature_change_scale")):
        geometry_parameters[PIDX_SLOSH[name]]=weights[key]
    geometry_parameters=ca.DM(geometry_parameters)
    route = np.asarray(task["route"], float)
    route_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    length = route_s[-1]
    start = np.asarray(task["start_state"], float)
    if not (0 <= start[6] <= limits["v_max"] and abs(start[7]) <= limits["omega_max"]
            and 0 <= start[4] <= length):
        raise ValueError("initial command/progress outside motion bounds")
    # Actual speed cannot react before the known initial linear FIFO drains.
    frozen_progress_steps = 0
    if start[3] == 0:
        for command in start[8:13]:
            if command != 0:
                break
            frozen_progress_steps += 1
    F = dynamics(task)
    guide_fn = route_guide(route_s, route)
    opt = ca.Opti()
    X, U = opt.variable(28, count+1), opt.variable(3, count)
    progress_scale=opt.variable()
    opt.subject_to(opt.bounded(task["progress_scale_min"],progress_scale,task["progress_scale_max"]))
    opt.subject_to(X[:, 0] == start)
    opt.subject_to(X[:, 1:] == F.map(count)(X[:, :-1], U))
    opt.subject_to(opt.bounded(limits["actual_v_min"], X[3, :], limits["v_max"]))
    opt.subject_to(opt.bounded(-limits["omega_max"], X[5, :], limits["omega_max"]))
    # Do not put inequality bounds on coordinates already fixed by equalities.
    # From `clear` onward zero commands + zero accelerations imply zero commands;
    # from `moving` onward zero progress speed fixes progress at route length.
    # Duplicating those active bounds destroys strict interior feasibility.
    opt.subject_to(opt.bounded(0, X[6, 1:clear], limits["v_max"]))
    opt.subject_to(opt.bounded(-limits["omega_max"], X[7, 1:clear], limits["omega_max"]))
    opt.subject_to(opt.bounded(0, X[4, frozen_progress_steps+1:moving], length))
    opt.subject_to(opt.bounded(-limits["a_max"], U[0, :clear], limits["a_max"]))
    opt.subject_to(opt.bounded(-limits["alpha_max"], U[1, :clear], limits["alpha_max"]))
    opt.subject_to(opt.bounded(0, U[2, frozen_progress_steps:moving], limits["v_max"]))
    opt.subject_to(opt.bounded(-limits["jerk_max"]*dt, U[0, :]-X[23, :-1], limits["jerk_max"]*dt))
    constrain_actual_jerk(opt, X, task)
    opt.subject_to(X[4, moving] == length)
    # The exported geometric coordinate must advance with actual motion.
    # Independent virtual progress can otherwise stall while the vehicle turns,
    # making x(s), y(s), v(s) multi-valued for the lower controller.
    opt.subject_to(U[2,:moving] == progress_scale*(X[3,:moving]+X[3,1:moving+1])*.5)
    # Commands, FIFO and acceleration memory are exactly clear when TAIL starts.
    # Actual motion decays continuously through the shared actuator dynamics.
    opt.subject_to(X[6:8, clear] == 0)
    opt.subject_to(U[:2, clear:] == 0)
    opt.subject_to(U[2, moving:] == 0)
    goal = np.asarray(task["goal_pose"], float)
    goal[2]=start[2]+np.arctan2(np.sin(goal[2]-start[2]),np.cos(goal[2]-start[2]))
    dx, dy = X[0, moving:]-goal[0], X[1, moving:]-goal[1]
    opt.subject_to(dx*dx+dy*dy <= task["goal_position_tolerance"]**2)
    opt.subject_to(opt.bounded(-task["goal_yaw_tolerance"], X[2, moving:]-goal[2], task["goal_yaw_tolerance"]))
    opt.subject_to(opt.bounded(-task["stop_speed_tolerance"], X[3, moving:], task["stop_speed_tolerance"]))
    opt.subject_to(opt.bounded(-task["stop_omega_tolerance"], X[5, moving:], task["stop_omega_tolerance"]))
    terminal_policy = task.get("terminal_speed_policy")
    if terminal_policy is not None:
        distance = ca.sqrt((X[0, :]-goal[0])**2+(X[1, :]-goal[1])**2+1e-16)
        speed_cap, command_cap_squared = speed_limits(task, length-X[4, :], distance)
        opt.subject_to(X[3, 1:] <= speed_cap[1:])
        # Command zero is already fixed from `clear` onward.
        opt.subject_to((X[6, 1:clear]**2-command_cap_squared[1:clear])/limits["v_max"]**2 <= 0)
    height_sq = task["height_coeff"]**2*(X[24, :]**2+X[26, :]**2)
    policy = task.get("liquid_policy")
    if policy is not None:
        caps = ca.DM(height_limits(task, np.arange(count+1)*dt)).T
        opt.subject_to(height_sq / caps**2 <= 1)
    elif task["liquid_constraint_enable"]:
        opt.subject_to(height_sq <= task["liquid_height_limit"]**2)
    liquid_end = objective_end_index(task)
    # Initial guess only; it does not constrain autonomous geometry or speed.
    progress_seed = start[4]+np.minimum(np.arange(count+1)/max(1, moving-12), 1.)*(length-start[4])
    region_ids = []
    sweep = max(limits["v_max"], abs(limits["actual_v_min"]))*dt
    objective = 0
    modal_energy = (height_sq+task["height_coeff"]**2*(X[25, :]**2+X[27, :]**2)/task["liquid_parameters"][1])/weights["height_scale"]**2
    for k in range(count+1):
        cell = next(c for c in task["region"]["cells"] if c["s_begin"]-1e-8 <= progress_seed[k] <= c["s_end"]+1e-8)
        normal, offset = halfspaces(task["region"], cell, sweep if k < count else 0)
        opt.subject_to(ca.DM(normal)@X[:2, k] <= offset)
        if frozen_progress_steps < k < moving:
            if cell["s_begin"] > 0:
                opt.subject_to(X[4, k] >= cell["s_begin"])
            if cell["s_end"] < length:
                opt.subject_to(X[4, k] <= cell["s_end"])
        else:
            fixed_s = start[4] if k <= frozen_progress_steps else length
            if not cell["s_begin"]-1e-8 <= fixed_s <= cell["s_end"]+1e-8:
                raise ValueError("fixed endpoint outside assigned region progress")
        region_ids.append(cell["id"])
        if k < moving:
            xy, tangent = guide_fn(X[4, k])
            delta = X[:2, k]-xy
            lag = ca.dot(delta, tangent)
            contour = delta[0]*tangent[1]-delta[1]*tangent[0]
            actual = actual_motion_rhs(ca.vertcat(X[:4, k], X[5, k], X[24:28, k]),
                                      ca.vertcat(X[8, k], X[13, k]), task["actuator_parameters"], task["liquid_parameters"])
            curvature_cost, change_cost=geometry_terms(X[:,k],actual[3],actual[4],
                                                       geometry_parameters,PIDX_SLOSH,1.)
            objective += dt*(weights["contour"]*(contour/weights["contour_scale"])**2 +
                weights["lag"]*(lag/weights["lag_scale"])**2 +
                curvature_cost + change_cost +
                weights["control"]*((U[0, k]/limits["a_max"])**2+(U[1, k]/limits["alpha_max"])**2) +
                weights["jerk"]*((U[0, k]-X[23, k])/(limits["jerk_max"]*dt))**2)
        if k <= liquid_end:
            quadrature = .5 if policy is not None and k in (0, liquid_end) else 1.
            objective += quadrature*dt*weights["liquid"]*modal_energy[k]
    objective += weights["liquid_terminal"]*modal_energy[liquid_end]
    opt.minimize(objective)
    guess = np.zeros((28, count+1))
    guess[4] = progress_seed
    guess[0] = np.interp(progress_seed, route_s, route[:, 0])
    guess[1] = np.interp(progress_seed, route_s, route[:, 1])
    guess[2] = np.unwrap(np.arctan2(np.gradient(guess[1]), np.gradient(guess[0])))
    guess[2, moving:] = goal[2]
    guess[:, 0] = start
    opt.set_initial(X, guess)
    opt.set_initial(U, 0)
    opt.set_initial(progress_scale,1.)
    if warm_plan is not None:
        warm_x, warm_u, warm_scale = warm_start_values(warm_plan, task, count)
        opt.set_initial(X, warm_x)
        opt.set_initial(U, warm_u)
        opt.set_initial(progress_scale, warm_scale)
        emit("warm_start_loaded", progress_scale=warm_scale)
    opt.solver("ipopt", {"print_time": False}, {"print_level": solver_verbosity, "max_iter": task["max_iterations"],
        "tol": task["constraint_tolerance"], "constr_viol_tol": task["constraint_tolerance"],
        "acceptable_tol": task["constraint_tolerance"], "bound_relax_factor": 0., "fixed_variable_treatment": "make_constraint"})
    begun = time.monotonic()
    emit("solve_started", states=count+1, variables=int(opt.nx), constraints=int(opt.ng))
    try:
        solution = opt.solve()
    except RuntimeError:
        try:
            failure_stats = solver_summary(opt.stats())
        except RuntimeError:
            failure_stats = {}
        emit("solve_failed", **failure_stats)
        raise
    emit("solve_finished", **solver_summary(solution.stats()))
    states, controls = np.asarray(solution.value(X)).T, np.asarray(solution.value(U)).T
    controls = np.vstack((controls, np.zeros(3)))
    plan = {k: task[k] for k in ("frame_id", "region", "dt", "transport_duration", "deadline", "stop_window", "height_coeff",
        "actuator_parameters", "liquid_parameters", "motion_limits", "goal_pose", "goal_position_tolerance",
        "goal_yaw_tolerance", "stop_speed_tolerance", "stop_omega_tolerance", "route")}
    if policy is not None:
        plan["liquid_policy"] = policy
    if terminal_policy is not None:
        plan["terminal_speed_policy"] = terminal_policy
    task_hash = hashlib.sha256(json.dumps(task, sort_keys=True, allow_nan=False).encode()).hexdigest()
    plan.update(schema_version=1, liquid_model_version=MODEL_VERSION, cost_model_version=COST_VERSION,
                plan_id=task["task_id"]+"-"+task_hash[:12], region_id=task["region"]["id"], task=task,
                optimization=dict(status=solution.stats()["return_status"], guide_interpolation="smooth_route_v2_padded", objective=float(solution.value(objective)),
                                  solve_seconds=time.monotonic()-begun, iterations=solution.stats()["iter_count"]))
    plan["progress_parameterization"]={"method":"scaled_actual_speed_trapezoid", "scale":float(solution.value(progress_scale))}
    # Find final command braking segment without assuming actual a == commanded a.
    moving_controls = controls[:moving, 0]
    last_positive = np.flatnonzero(moving_controls > 1e-6)
    brake = int(last_positive[-1]+1) if len(last_positive) else 0
    plan["samples"] = [dict(t=k*dt, state=x.tolist(), control=u.tolist(),
        phase="TAIL" if k >= moving else ("BRAKE" if k >= brake else "MOVE"), region_cell_id=region_ids[k])
        for k, (x, u) in enumerate(zip(states, controls))]
    from .validation import validate_plan
    emit("validation_started")
    try:
        plan["validation"] = validate_plan(plan)
    except ValueError as error:
        raise PlanValidationError(str(error), plan) from error
    emit("validation_finished", **plan["validation"])
    return plan
