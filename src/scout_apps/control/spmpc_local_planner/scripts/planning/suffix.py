"""Offline remaining-motion comparison from a truthful full predicted state.

This module does not import ROS, publish commands, select runtime modes, reset a
controller clock, or claim recursive feasibility. Reoptimization can be slow.
"""
from copy import deepcopy
import numpy as np
from .optimizer import dynamics, solve_task
from .task import load_task, halfspaces
from .validation import validate_plan


def remaining_task(plan, state, elapsed):
    task = deepcopy(plan["task"])
    state = np.asarray(state, float)
    if state.shape != (28,) or not np.isfinite(state).all() or not np.isfinite(elapsed) or elapsed < 0:
        raise ValueError("suffix needs a finite, complete 28-state prediction and elapsed time")
    if abs(elapsed/task["dt"]-round(elapsed/task["dt"])) > 1e-5:
        raise ValueError("suffix diagnostic currently requires a model-grid epoch")
    if elapsed >= task["transport_duration"]:
        raise ValueError("no remaining transport interval; use the fixed tail evaluation")
    task["start_state"] = state.tolist()
    task["deadline"] -= elapsed
    task["transport_duration"] -= elapsed
    task["origin_task_elapsed_sec"] = elapsed
    task["task_id"] += "-suffix"
    return load_task(task)


def _nominal_suffix(plan, task, elapsed):
    dt = task["dt"]
    begin = round(elapsed/dt)
    n = round((task["deadline"]+task["stop_window"])/dt)
    moving = round(task["transport_duration"]/dt)
    original = plan["samples"]
    F = dynamics(task)
    state = np.asarray(task["start_state"])
    candidate = deepcopy(plan)
    candidate.pop("optimization", None)  # No optimization is run for this candidate.
    candidate.pop("validation", None)
    candidate.update(task=task, plan_id=plan["plan_id"]+"-nominal-suffix")
    for key in ("deadline", "transport_duration", "dt", "stop_window"):
        candidate[key] = task[key]
    rows = []
    for k in range(n+1):
        index = min(begin+k, len(original)-1)
        control = np.asarray(original[index]["control"] if k < n else [0.,0.,0.])
        cells = [c for c in task["region"]["cells"] if c["s_begin"]-1e-8 <= state[4] <= c["s_end"]+1e-8]
        if not cells: raise ValueError("nominal suffix leaves the region progress domain")
        phase = "TAIL" if k >= moving else original[index]["phase"]
        rows.append(dict(t=k*dt, state=state.tolist(), control=control.tolist(), phase=phase, region_cell_id=cells[0]["id"]))
        if k < n: state = np.asarray(F(state, control)).ravel()
    candidate["samples"] = rows
    candidate["validation"] = validate_plan(candidate)
    return candidate


def compare_suffixes(plan, full_state, task_elapsed, reoptimize=True):
    task = remaining_task(plan, full_state, task_elapsed)
    records = []
    operations = [("nominal_time_suffix", lambda: _nominal_suffix(plan, task, task_elapsed))]
    if reoptimize: operations.append(("full_state_reoptimized_suffix", lambda: solve_task(task)))
    for name, operation in operations:
        try:
            candidate = operation()
            x = np.asarray(candidate["samples"][-1]["state"])
            residual = task["height_coeff"]*float(np.sqrt(x[24]**2+x[26]**2+(x[25]**2+x[27]**2)/task["liquid_parameters"][1]))
            records.append(dict(candidate=name, feasible=True, predicted_residual_energy_height_m=residual,
                                validation=candidate["validation"], optimization=candidate.get("optimization")))
        except (ValueError, RuntimeError, KeyError) as error:
            records.append(dict(candidate=name, feasible=False, failure=str(error)))
    feasible = [r for r in records if r["feasible"]]
    return dict(schema_version=1, mode="OFFLINE_DIAGNOSTIC_ONLY", source_plan_id=plan["plan_id"],
                task_elapsed_sec=task_elapsed, original_deadline_sec=plan["deadline"], full_initial_state=list(full_state),
                candidates=records, best_feasible_candidate=min(feasible, key=lambda r:r["predicted_residual_energy_height_m"])["candidate"] if feasible else None,
                runtime_command_changed=False, recursive_feasibility_claim=False)
