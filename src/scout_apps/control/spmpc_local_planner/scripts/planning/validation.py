"""Verify a saved plan by replaying the full production dynamics from its x0."""
import numpy as np
from .task import load_task, halfspaces
from .optimizer import dynamics
from actual_motion_kernel import functions as motion_functions
from model_contract import MODEL_VERSION, COST_VERSION


def validate_plan(plan, tolerance=2e-6):
    if (plan["schema_version"], plan["liquid_model_version"], plan["cost_model_version"]) != (1, MODEL_VERSION, COST_VERSION):
        raise ValueError("unsupported plan schema/model")
    task = load_task(plan["task"])
    for key in ("dt", "deadline", "transport_duration", "stop_window", "height_coeff", "motion_limits",
                "actuator_parameters", "liquid_parameters", "goal_pose", "goal_position_tolerance",
                "goal_yaw_tolerance", "stop_speed_tolerance", "stop_omega_tolerance", "route", "region"):
        if plan[key] != task[key]: raise ValueError(f"plan/task contract mismatch: {key}")
    if plan["frame_id"] != task["frame_id"] or plan["region_id"] != task["region"]["id"] or not plan["plan_id"]:
        raise ValueError("plan identity mismatch")
    rows = plan["samples"]
    X, U, times = (np.asarray([r[k] for r in rows], float) for k in ("state", "control", "t"))
    dt = task["dt"]
    n = round((task["deadline"]+task["stop_window"])/dt)
    moving = round(task["transport_duration"]/dt)
    if X.shape != (n+1, 28) or U.shape != (n+1, 3) or not np.allclose(times, np.arange(n+1)*dt, atol=1e-8, rtol=0):
        raise ValueError("plan grid/layout mismatch")
    if not np.isfinite(X).all() or not np.isfinite(U).all() or np.max(np.abs(X[0]-task["start_state"])) > tolerance:
        raise ValueError("plan has nonfinite data or changed initial state")
    route = np.asarray(task["route"], float)
    route_length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
    if np.any(X[:, 4] < -1e-8) or np.any(X[:, 4] > route_length + 1e-8) or np.any(np.diff(X[:, 4]) < -1e-8):
        raise ValueError("progress exceeds original route domain")
    if abs(X[moving, 4] - route_length) > tolerance:
        raise ValueError("transport endpoint does not reach original route length")
    F = dynamics(task)
    replay = np.empty_like(X); replay[0] = task["start_state"]
    for k in range(n): replay[k+1] = np.asarray(F(replay[k], U[k])).ravel()
    defect = float(np.max(np.abs(X-replay)))
    if defect > tolerance: raise ValueError(f"full-state rollout mismatch: {defect}")
    limits = task["motion_limits"]
    errors = []
    def bound(values, lower, upper, name):
        if not np.isfinite(values).all() or np.any(values < lower-tolerance) or np.any(values > upper+tolerance): errors.append(name)
    bound(X[:, 3], limits["actual_v_min"], limits["v_max"], "actual_v")
    bound(X[:, 5], -limits["omega_max"], limits["omega_max"], "actual_omega")
    bound(X[:, 6], 0, limits["v_max"], "command_v")
    bound(X[:, 7], -limits["omega_max"], limits["omega_max"], "command_omega")
    bound(X[:, 8:13], 0, limits["v_max"], "linear_fifo")
    bound(X[:, 13:23], -limits["omega_max"], limits["omega_max"], "angular_fifo")
    bound(X[:, 23], -limits["a_max"], limits["a_max"], "acceleration_memory")
    for i, limit in ((0, "a_max"), (1, "alpha_max")):
        bound(U[:, i], -limits[limit], limits[limit], limit)
    bound(U[:, 2], 0, limits["v_max"], "progress_speed")
    bound(U[:-1, 0]-X[:-1, 23], -limits["jerk_max"]*dt, limits["jerk_max"]*dt, "jerk")
    bound(np.diff(X[:, 4]), 0, limits["v_max"]*dt, "progress")
    if "progress_parameterization" in plan:
        coordinate=plan["progress_parameterization"]
        if coordinate["method"] != "scaled_actual_speed_trapezoid":
            raise ValueError("unknown progress parameterization")
        scale=float(coordinate["scale"])
        bound(scale,task["progress_scale_min"],task["progress_scale_max"],"progress_scale")
        bound(U[:moving,2]-.5*scale*(X[:moving,3]+X[1:moving+1,3]),0,0,"progress_parameterization")
    bound(X[moving:, 6:24], 0, 0, "tail_commands_fifo")
    bound(U[moving:], 0, 0, "tail_controls")
    bound(X[moving:, 3], -task["stop_speed_tolerance"], task["stop_speed_tolerance"], "tail_actual_speed")
    bound(X[moving:, 5], -task["stop_omega_tolerance"], task["stop_omega_tolerance"], "tail_actual_omega")
    goal = np.asarray(task["goal_pose"])
    bound(np.linalg.norm(X[moving:, :2]-goal[:2], axis=1), 0, task["goal_position_tolerance"], "goal_position")
    yaw_error = np.arctan2(np.sin(X[moving:, 2]-goal[2]), np.cos(X[moving:, 2]-goal[2]))
    bound(yaw_error, -task["goal_yaw_tolerance"], task["goal_yaw_tolerance"], "goal_yaw")
    if any(r["phase"] not in ("MOVE", "WAIT", "BRAKE") for r in rows[:moving]) or any(r["phase"] != "TAIL" for r in rows[moving:]):
        errors.append("phase/tail grid")
    clearance = float("inf")
    sweep = max(limits["v_max"], abs(limits["actual_v_min"]))*dt
    cells = {c["id"]: c for c in task["region"]["cells"]}
    for k, x in enumerate(X):
        cell = cells[rows[k]["region_cell_id"]]
        normal, offset = halfspaces(task["region"], cell, sweep if k < n else 0)
        clearance = min(clearance, float(np.min(offset-normal@x[:2])))
        bound(x[4], cell["s_begin"], cell["s_end"], "region_progress")
    if clearance < -tolerance: errors.append("swept_footprint_region")
    # Dense held-command substeps check liquid peaks hidden between OCP nodes.
    motion_step = motion_functions()[0]
    peak = 0.
    for k in range(n):
        z = np.r_[X[k, :4], X[k, 5], X[k, 24:28]]
        for j in range(4):
            peak = max(peak, task["height_coeff"]*float(np.hypot(z[5], z[7])))
            z = np.asarray(motion_step(z, X[k, [8, 13]], task["actuator_parameters"], task["liquid_parameters"], dt/4)).ravel()
        peak = max(peak, task["height_coeff"]*float(np.hypot(z[5], z[7])))
    if task["liquid_constraint_enable"] and peak > task["liquid_height_limit"]+tolerance: errors.append("dense_liquid_cap")
    if errors: raise ValueError("plan violates: " + ", ".join(sorted(set(errors))))
    speed = np.sqrt(X[:, 3]**2+task["objective"]["speed_floor"]**2)
    curvature = X[:, 5]/speed
    return dict(status="SOFTWARE_VERIFIED", dynamics_max_error=defect, minimum_region_clearance=clearance,
                dense_peak_height_m=peak, residual_height_m=float(task["height_coeff"]*np.hypot(X[-1, 24], X[-1, 26])),
                curvature_arc_energy=float(np.sum(curvature[:-1]**2*speed[:-1])*dt), actual_path_length=float(np.sum(np.abs(X[:-1, 3]))*dt),
                hardware_verified=False)
