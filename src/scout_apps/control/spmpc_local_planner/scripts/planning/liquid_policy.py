"""Small, offline-only contract for transport liquid height limits."""

import numbers

import numpy as np


_KEYS = {"mode", "move_height_m", "tail_height_m", "height_tolerance_m"}


def normalize_policy(value):
    """Validate and return an independent normalized liquid policy."""
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != _KEYS:
        raise ValueError("liquid_policy must contain exactly the required fields")
    if value["mode"] != "transport_tail_v1":
        raise ValueError("unsupported liquid_policy mode")
    for key in ("move_height_m", "tail_height_m", "height_tolerance_m"):
        number = value[key]
        if isinstance(number, (bool, np.bool_)) or not isinstance(number, numbers.Real):
            raise ValueError("liquid policy heights must be real numbers")
        if not np.isfinite(number):
            raise ValueError("liquid policy heights must be finite")
    move = float(value["move_height_m"])
    tail = float(value["tail_height_m"])
    tolerance = float(value["height_tolerance_m"])
    if not 0.0 < tolerance < tail <= move:
        raise ValueError("liquid policy heights are out of order")
    return {"mode": value["mode"], "move_height_m": move,
            "tail_height_m": tail, "height_tolerance_m": tolerance}


def _policy(task):
    return normalize_policy(task.get("liquid_policy"))


def height_limits(task, times, t_stop=None):
    """Return metre-valued limits, switching to tail at T (or early stop)."""
    policy = _policy(task)
    values = np.asarray(times, dtype=float)
    if not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError("times must be finite and nonnegative")
    if policy is None:
        return np.full(values.shape, task.get("liquid_height_limit", np.inf), dtype=float)
    T = float(task["transport_duration"])
    switch = T if t_stop is None else float(t_stop)
    if not np.isfinite(switch) or switch < 0:
        raise ValueError("t_stop must be finite and nonnegative")
    switch = min(T, switch)
    return np.where(values >= switch, policy["tail_height_m"], policy["move_height_m"])


def objective_end_index(task):
    """End index for the objective, preserving the legacy no-policy timing."""
    dt = float(task["dt"])
    policy = _policy(task)
    end_time = (float(task["transport_duration"]) + float(task["stop_window"])
                if policy is not None else
                float(task["deadline"]) + float(task["stop_window"]))
    return int(round(end_time / dt))
