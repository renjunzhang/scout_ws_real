#!/usr/bin/env python3
"""Numerical, ROS-free initial-state A/B audit of the explicit robot model.

This is a conditional replay with known emitted commands, not a new OCP solve
or a bit-for-bit replay of the sampled FIFO/RK4 OCP. No parameters are fitted.
Only [x, y, yaw, v_actual, omega_actual] differ between the two initial states.
"""

import copy
import math

import numpy as np

from analyze_mocap_execution_chain import finite_summary


class ContractError(ValueError):
    pass


ROBOT_FIELDS = ("x", "y", "yaw", "v", "omega")
PARAM_FIELDS = ("actuator_dt", "actuator_tau_v", "actuator_tau_omega",
                "actuator_gain_v", "actuator_gain_omega")


def wrap_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


def strict_rows(rows, width, name):
    """Never silently sort, deduplicate, fill a gap, or use bag arrival time."""
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or data.shape[1] != width or len(data) < 2:
        raise ContractError(name + ": insufficient samples / invalid shape")
    if not np.isfinite(data).all() or np.any(data[:, 0] <= 0):
        raise ContractError(name + ": nonfinite value / invalid physical stamp")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ContractError(name + ": repeated or regressing timestamps")
    return data


def reference_issues(config, solver_frames, mocap_frames, bag_sha256):
    issues = []
    if config.get("status") != "verified_independent":
        issues.append("REFERENCE_UNVERIFIED: 时间、坐标及参考点尚未独立核验")
    if not config.get("provenance", "").strip():
        issues.append("REFERENCE_PROVENANCE_MISSING")
    if bag_sha256 in config.get("calibration_bag_sha256", []):
        issues.append("REFERENCE_FITTED_ON_EVALUATION_BAG")
    if solver_frames != {config.get("solver_frame")} or "" in solver_frames:
        issues.append("SOLVER_FRAME_MISMATCH")
    if mocap_frames != {config.get("mocap_frame")} or "" in mocap_frames:
        issues.append("MOCAP_FRAME_MISMATCH")
    for field in ("time_sec", "position_m", "yaw_rad", "v_mps", "omega_radps"):
        value = config.get("uncertainty", {}).get(field)
        if value is None or not math.isfinite(float(value)) or float(value) < 0:
            issues.append("REFERENCE_UNCERTAINTY_MISSING:" + field)
    return issues


def transform_mocap(rows, config):
    """Rows: header stamp, tracker x/y/yaw. p_base = p_tracker - R_base*r.

    base yaw = tracker yaw + tracker_to_base_yaw_rad. Then apply the
    independently supplied SE(2) transform to the solver frame. Physical time
    = header stamp + mocap_stamp_offset_sec (a late stamp needs a negative offset).
    """
    data = strict_rows(rows, 4, "mocap").copy()
    transform = config["mocap_to_solver"]
    shift = float(config["mocap_stamp_offset_sec"])
    yaw_offset = float(config["tracker_to_base_yaw_rad"])
    lever = np.asarray(config["base_to_tracker_m"], dtype=float)
    angle = float(transform["yaw_rad"])
    translation = np.array([transform["x_m"], transform["y_m"]], dtype=float)
    if lever.shape != (2,) or not np.isfinite(
            [shift, yaw_offset, angle, *lever, *translation]).all():
        raise ContractError("invalid reference transform")
    base_yaw = np.unwrap(data[:, 3]) + yaw_offset
    c, s = np.cos(base_yaw), np.sin(base_yaw)
    base_x = data[:, 1] - c * lever[0] + s * lever[1]
    base_y = data[:, 2] - s * lever[0] - c * lever[1]
    data[:, 0] += shift
    data[:, 1] = math.cos(angle) * base_x - math.sin(angle) * base_y + translation[0]
    data[:, 2] = math.sin(angle) * base_x + math.cos(angle) * base_y + translation[1]
    data[:, 3] = base_yaw + angle
    return strict_rows(data, 4, "transformed mocap")


def causal_support(time, epoch, window_sec, max_gap_sec):
    begin = np.searchsorted(time, epoch - window_sec, side="left")
    end = np.searchsorted(time, epoch, side="right")
    selected = time[begin:end]
    if (len(selected) < 6 or selected[-1] < epoch - max_gap_sec
            or selected[0] > epoch - window_sec + max_gap_sec
            or np.max(np.diff(selected)) > max_gap_sec):
        raise ContractError("MOCAP_SUPPORT_MISSING_OR_GAPPED")
    return np.arange(begin, end)


def fit_state(time, poses, epoch, window_sec):
    """Backward cubic fit at epoch. Caller provides only the causal support.

    Returns x,y,yaw,v,omega plus lateral velocity. The same kernel and sample
    times MUST be applied to model positions for future-error comparisons.
    """
    if len(time) < 6 or np.any(time > epoch + 1e-9):
        raise ContractError("NONCAUSAL_OR_SHORT_REFERENCE_SUPPORT")
    u = (time - epoch) / window_sec
    design = np.column_stack([np.ones(len(u)), u, u * u, u * u * u])
    coefficients, _, rank, _ = np.linalg.lstsq(design, poses, rcond=None)
    if rank < 4:
        raise ContractError("SINGULAR_REFERENCE_FIT")
    x, y, yaw = coefficients[0]
    vx, vy, omega = coefficients[1] / window_sec
    v = vx * math.cos(yaw) + vy * math.sin(yaw)
    lateral = -vx * math.sin(yaw) + vy * math.cos(yaw)
    return np.array([x, y, yaw, v, omega]), float(lateral)


def replace_robot_initial_state(snapshot, corrected):
    result = copy.deepcopy(snapshot)
    for name, value in zip(ROBOT_FIELDS, corrected):
        result["robot_" + name] = float(value)
    return result


def snapshot_contract(snapshot):
    if (snapshot["schema_version"] < 4 or not snapshot["actuator_state_valid"]
            or snapshot["backend"] != "continuous_mpcc_acados_explicit_actuator"
            or snapshot["control_semantics"] != "a_cmd_alpha_cmd"
            or snapshot["state_width"] not in (24, 28)
            or snapshot["control_width"] != 3
            or snapshot["horizon_steps"] < 1):
        raise ContractError("UNSUPPORTED_SNAPSHOT_CONTRACT")
    names = snapshot["parameter_names"]
    width = snapshot["parameter_width"]
    if (len(names) != width or len(set(names)) != width
            or width != (37 if snapshot["state_width"] == 28 else 28)):
        raise ContractError("INVALID_PARAMETER_SCHEMA")
    raw = np.asarray(snapshot["stage_parameters"], dtype=float)
    if raw.size != width * (snapshot["horizon_steps"] + 1) or not np.isfinite(raw).all():
        raise ContractError("INCOMPLETE_STAGE_PARAMETERS")
    stages = raw.reshape((-1, width))
    if not all(field in names for field in PARAM_FIELDS):
        raise ContractError("ACTUATOR_PARAMETERS_MISSING")
    indices = [names.index(field) for field in PARAM_FIELDS]
    if not np.allclose(stages[:, indices], stages[0, indices], rtol=0, atol=1e-12):
        raise ContractError("ACTUATOR_PARAMETERS_CHANGE_WITH_STAGE")
    params = dict(zip(PARAM_FIELDS, stages[0, indices]))
    if any(value <= 0 for value in params.values()):
        raise ContractError("INVALID_ACTUATOR_PARAMETERS")
    if abs(snapshot["dt"] - params["actuator_dt"]) > 1e-9:
        raise ContractError("ACTUATOR_DT_MISMATCH")
    for channel, count in (("linear", 5), ("angular", 10)):
        queue = snapshot["actuator_" + channel + "_delay_queue"]
        if len(queue) != count or not np.isfinite(queue).all():
            raise ContractError("INVALID_DELAY_QUEUE")
    actuator = [snapshot[name] for name in ("actuator_v_cmd", "actuator_omega_cmd",
                "actuator_a_cmd_memory", "actuator_delayed_v_cmd", "actuator_delayed_omega_cmd")]
    if not np.isfinite(actuator).all():
        raise ContractError("INVALID_ACTUATOR_INITIAL_STATE")
    if not np.allclose(actuator[3:], [snapshot["actuator_linear_delay_queue"][0],
                                      snapshot["actuator_angular_delay_queue"][0]], atol=1e-7, rtol=0):
        raise ContractError("DELAY_QUEUE_HEAD_MISMATCH")
    values = [snapshot["robot_" + name] for name in ROBOT_FIELDS]
    if not np.isfinite(values).all():
        raise ContractError("INVALID_ROBOT_INITIAL_STATE")
    params["delay_v"] = len(snapshot["actuator_linear_delay_queue"]) * snapshot["dt"]
    params["delay_omega"] = len(snapshot["actuator_angular_delay_queue"]) * snapshot["dt"]
    return params


def command_at(commands, time):
    index = np.searchsorted(commands[:, 0], time, side="right") - 1
    if index < 0:
        raise ContractError("COMMAND_HISTORY_MISSING")
    return commands[index, 1:]


def check_command_support(commands, begin, end, max_gap_sec):
    first = np.searchsorted(commands[:, 0], begin, side="right") - 1
    last = np.searchsorted(commands[:, 0], end, side="left")
    if first < 0 or last >= len(commands):
        raise ContractError("COMMAND_HISTORY_OR_FUTURE_MISSING")
    if np.max(np.diff(commands[first:last + 1, 0]), initial=0) > max_gap_sec:
        raise ContractError("COMMAND_PUBLICATION_GAP")


def check_snapshot_history(snapshot, commands):
    epoch, dt = snapshot["solver_input_epoch"], snapshot["dt"]
    actual = command_at(commands, epoch)
    if not np.allclose(actual, [snapshot["actuator_v_cmd"],
                                snapshot["actuator_omega_cmd"]], atol=1e-7, rtol=0):
        raise ContractError("CURRENT_COMMAND_MISMATCH")
    for channel, column in (("linear", 0), ("angular", 1)):
        queue = snapshot["actuator_" + channel + "_delay_queue"]
        expected = [command_at(commands, epoch - (len(queue) - i) * dt)[column]
                    for i in range(len(queue))]
        if not np.allclose(queue, expected, atol=1e-7, rtol=0):
            raise ContractError("DELAY_QUEUE_HISTORY_MISMATCH")
    memory = (actual[0] - snapshot["actuator_linear_delay_queue"][-1]) / dt
    if abs(memory - snapshot["actuator_a_cmd_memory"]) > 1e-6:
        raise ContractError("ACCEL_MEMORY_HISTORY_MISMATCH")


def constant_input_step(state, target_v, target_omega, tau_v, tau_omega, dt):
    """Exact v, omega and yaw; Simpson position quadrature on a short substep."""
    x, y, yaw, v, omega = state

    def at(t):
        v_t = target_v + (v - target_v) * math.exp(-t / tau_v)
        w_t = target_omega + (omega - target_omega) * math.exp(-t / tau_omega)
        yaw_t = yaw + target_omega * t + (omega - target_omega) * tau_omega * (-math.expm1(-t / tau_omega))
        return v_t, w_t, yaw_t

    mid_v, _, mid_yaw = at(dt / 2)
    next_v, next_omega, next_yaw = at(dt)
    next_x = x + dt / 6 * (v * math.cos(yaw) + 4 * mid_v * math.cos(mid_yaw)
                          + next_v * math.cos(next_yaw))
    next_y = y + dt / 6 * (v * math.sin(yaw) + 4 * mid_v * math.sin(mid_yaw)
                          + next_v * math.sin(next_yaw))
    return np.array([next_x, next_y, next_yaw, next_v, next_omega])


def replay_robot(initial, epoch, sample_times, commands, params, max_step_sec=0.002):
    sample_times = np.asarray(sample_times, dtype=float)
    if (not len(sample_times) or np.any(np.diff(sample_times) <= 0)
            or sample_times[0] < epoch or max_step_sec <= 0):
        raise ContractError("INVALID_REPLAY_TIMES")
    end = sample_times[-1]
    events = np.concatenate([sample_times, commands[:, 0] + params["delay_v"],
                             commands[:, 0] + params["delay_omega"]])
    events = np.unique(events[(events > epoch) & (events <= end)])
    current, state = float(epoch), np.array(initial, dtype=float)
    states = {epoch: state.copy()}
    for boundary in events:
        # Every delayed ZOH event is a boundary; no step straddles a command change.
        target_v = params["actuator_gain_v"] * command_at(
            commands, (current + boundary) / 2 - params["delay_v"])[0]
        target_omega = params["actuator_gain_omega"] * command_at(
            commands, (current + boundary) / 2 - params["delay_omega"])[1]
        count = max(1, int(math.ceil((boundary - current) / max_step_sec)))
        dt = (boundary - current) / count
        for _ in range(count):
            state = constant_input_step(state, target_v, target_omega,
                                        params["actuator_tau_v"],
                                        params["actuator_tau_omega"], dt)
        current = float(boundary)
        states[current] = state.copy()
    return np.array([states[float(t)] for t in sample_times])


def state_error(state, reference):
    error = np.asarray(state) - np.asarray(reference)
    error[2] = wrap_angle(error[2])
    return error


def error_metrics(values):
    result = finite_summary(values)
    array = np.asarray(values, dtype=float)
    if len(array):
        result["rmse"] = float(np.sqrt(np.mean(array ** 2)))
    return result
