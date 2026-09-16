"""Explicit task/physical contract for geometry-speed-liquid planning.

Physical coefficients are required, normally copied from a frozen runtime
snapshot. The codegen's placeholder liquid parameters are never used as a cup
calibration. All normalized objectives and bounds are saved in the output.
"""
from copy import deepcopy
import json
from pathlib import Path
import sys
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "acados"))
from generate_spmpc_acados import load_config, default_parameter_values
from spmpc_acados_model import PIDX_SLOSH

DEFAULT_OBJECTIVE = dict(contour=.02, lag=.2, curvature=1., curvature_change=.01,
                         control=.05, jerk=.02, liquid=5., liquid_terminal=10.,
                         height_scale=.005, contour_scale=.3, lag_scale=.1,
                         curvature_scale=1., curvature_change_scale=1., speed_floor=.05)
LIMIT_NAMES = ("actual_v_min", "v_max", "omega_max", "a_max", "alpha_max", "jerk_max")
PROGRESS_TOLERANCE = 1e-6  # Matches the C++ motion-region coordinate tolerance.


def halfspaces(region, cell, sweep=0.):
    vertices = np.asarray(cell["vertices"], float)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or not 3 <= len(vertices) <= 8 or not np.isfinite(vertices).all():
        raise ValueError("region cell needs 3..8 finite xy vertices")
    edges = np.roll(vertices, -1, axis=0)-vertices
    length = np.linalg.norm(edges, axis=1)
    area = np.sum(vertices[:, 0]*np.roll(vertices[:, 1], -1)-vertices[:, 1]*np.roll(vertices[:, 0], -1))
    if np.any(length < 1e-9) or abs(area) < 1e-9:
        raise ValueError("degenerate polygon")
    normal = np.sign(area)*np.column_stack((edges[:, 1], -edges[:, 0]))/length[:, None]
    offset = np.sum(normal*vertices, axis=1)
    if np.any(vertices@normal.T-offset > 1e-8):
        raise ValueError("region must be a convex simple polygon")
    inflation = region["footprint_radius"]+region["margin"]+sweep
    return normal, offset-inflation


def _halfspaces_have_area(normal, offset):
    """Conservative 2-D feasibility check for a bounded half-space set."""
    points = []
    for i in range(len(offset)):
        for j in range(i + 1, len(offset)):
            d = normal[i, 0] * normal[j, 1] - normal[i, 1] * normal[j, 0]
            if abs(d) < 1e-12:
                continue
            point = np.array([(offset[i] * normal[j, 1] - normal[i, 1] * offset[j]) / d,
                              (normal[i, 0] * offset[j] - offset[i] * normal[j, 0]) / d])
            if np.all(normal @ point <= offset + 1e-8):
                points.append(point)
    if len(points) < 3:
        return False
    points = np.asarray(points)
    center = points.mean(axis=0)
    order = np.argsort(np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0]))
    ordered = points[order]
    return abs(np.sum(ordered[:, 0] * np.roll(ordered[:, 1], -1) -
                      ordered[:, 1] * np.roll(ordered[:, 0], -1))) > 1e-9


def load_task(source):
    task = deepcopy(source) if isinstance(source, dict) else yaml.safe_load(Path(source).read_text())
    for key in ("task_id", "frame_id", "route", "start_state", "goal_pose", "deadline", "stop_window",
                "actuator_parameters", "liquid_parameters", "height_coeff", "motion_limits", "region"):
        if key not in task:
            raise ValueError(f"missing task field: {key}")
    task.setdefault("dt", 1/30)
    task.setdefault("transport_duration", task["deadline"])
    task.setdefault("goal_position_tolerance", .05)
    task.setdefault("goal_yaw_tolerance", .1)
    task.setdefault("stop_speed_tolerance", .01)
    task.setdefault("stop_omega_tolerance", .02)
    task.setdefault("liquid_constraint_enable", False)
    task.setdefault("liquid_height_limit", .005)
    supplied_objective = task.get("objective", {})
    unknown_objective = set(supplied_objective) - set(DEFAULT_OBJECTIVE)
    if unknown_objective:
        raise ValueError("unknown objective key: " + sorted(unknown_objective)[0])
    task["objective"] = {**DEFAULT_OBJECTIVE, **supplied_objective}
    task.setdefault("max_iterations", 1000)
    task.setdefault("progress_scale_min", .1)
    task.setdefault("progress_scale_max", 10.)
    task.setdefault("constraint_tolerance", 1e-7)
    # Reject NaN even in optional nested fields; JSON also freezes numeric types.
    json.dumps(task, allow_nan=False)
    if isinstance(task["max_iterations"], bool) or not isinstance(task["max_iterations"], (int, np.integer)) or task["max_iterations"] < 1:
        raise ValueError("invalid max_iterations")
    for key, n in (("start_state", 28), ("goal_pose", 3), ("actuator_parameters", 4), ("liquid_parameters", 4)):
        a = np.asarray(task[key], float)
        if a.shape != (n,) or not np.isfinite(a).all():
            raise ValueError(f"invalid {key}")
    route = np.asarray(task["route"], float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 2 or not np.isfinite(route).all():
        raise ValueError("invalid original route")
    if np.any(np.linalg.norm(np.diff(route, axis=0), axis=1) < 1e-9):
        raise ValueError("duplicate route points")
    for key in ("dt", "transport_duration", "deadline", "stop_window", "height_coeff", "goal_position_tolerance",
                "goal_yaw_tolerance", "stop_speed_tolerance", "stop_omega_tolerance", "constraint_tolerance", "liquid_height_limit"):
        if not np.isfinite(task[key]) or task[key] <= 0:
            raise ValueError(f"invalid {key}")
    if task["goal_yaw_tolerance"] > np.pi:
        raise ValueError("goal_yaw_tolerance exceeds pi")
    if abs(task["dt"]-1/30) > 1e-8 or task["transport_duration"] > task["deadline"]:
        raise ValueError("task needs the production 30 Hz grid and transport <= deadline")
    if (not np.isfinite(task["progress_scale_min"]) or
            not np.isfinite(task["progress_scale_max"]) or
            not 0 < task["progress_scale_min"] <= task["progress_scale_max"]):
        raise ValueError("invalid progress scale bounds")
    for key in ("transport_duration", "deadline", "stop_window"):
        if abs(task[key]/task["dt"]-round(task[key]/task["dt"])) > 1e-5:
            raise ValueError(f"{key} must lie on the model grid")
    if min(task["actuator_parameters"]) <= 0 or task["liquid_parameters"][0] < 0 or min(task["liquid_parameters"][1:]) <= 0:
        raise ValueError("invalid physical coefficients")
    limits = task["motion_limits"]
    if not isinstance(limits, dict) or any(name not in limits for name in LIMIT_NAMES):
        raise ValueError("incomplete motion limits")
    if any(not np.isfinite(limits[k]) or (limits[k] <= 0 if k != "actual_v_min" else limits[k] > 0) for k in LIMIT_NAMES):
        raise ValueError("invalid motion limits")
    if any(not np.isfinite(v) or v < 0 for v in task["objective"].values()):
        raise ValueError("invalid objective weights/scales")
    for k in ("height_scale", "contour_scale", "lag_scale", "curvature_scale", "curvature_change_scale", "speed_floor"):
        if task["objective"][k] <= 0: raise ValueError(f"invalid objective scale: {k}")
    if np.linalg.norm(route[-1]-np.asarray(task["goal_pose"][:2])) > task["goal_position_tolerance"]:
        raise ValueError("actual goal differs from route endpoint")
    region = task["region"]
    if (not region["id"] or region["frame_id"] != task["frame_id"] or not region["cells"] or
            not np.isfinite(region["footprint_radius"]) or not np.isfinite(region["margin"]) or
            region["footprint_radius"] <= 0 or region["margin"] < 0):
        raise ValueError("invalid allowed-region identity/footprint")
    length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
    if region["cells"][0]["s_begin"] > 0 or region["cells"][-1]["s_end"] < length-1e-8:
        raise ValueError("region does not cover route progress")
    ids = set()
    previous = None
    for cell in region["cells"]:
        if (not cell["id"] or cell["id"] in ids or
                not np.isfinite(cell["s_begin"]) or not np.isfinite(cell["s_end"]) or
                cell["s_end"] <= cell["s_begin"]):
            raise ValueError("duplicate/invalid region cell")
        ids.add(cell["id"])
        if previous and (cell["s_begin"] > previous["s_end"] or cell["s_end"] < previous["s_end"] or cell["s_begin"] < previous["s_begin"]):
            raise ValueError("unordered/gapped region progress")
        normal, offset = halfspaces(region, cell, sweep=max(task["motion_limits"]["v_max"], abs(task["motion_limits"]["actual_v_min"])) * task["dt"])
        if not _halfspaces_have_area(normal, offset):
            raise ValueError("region cell has no traversable footprint interior")
        if previous is not None:
            prev_normal, prev_offset = halfspaces(region, previous, sweep=max(task["motion_limits"]["v_max"], abs(task["motion_limits"]["actual_v_min"])) * task["dt"])
            both_normal = np.vstack((prev_normal, normal))
            both_offset = np.r_[prev_offset, offset]
            if not _halfspaces_have_area(both_normal, both_offset):
                raise ValueError("adjacent region cells have no traversable overlap")
        previous = cell
    return task


def model_parameters(task):
    p = default_parameter_values(load_config(), True)
    p[PIDX_SLOSH["actuator_dt"]] = task["dt"]
    for name, value in zip(("actuator_tau_v", "actuator_tau_omega", "actuator_gain_v", "actuator_gain_omega"), task["actuator_parameters"]):
        p[PIDX_SLOSH[name]] = value
    for name, value in zip(("two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y"), task["liquid_parameters"]):
        p[PIDX_SLOSH[name]] = value
    return p
