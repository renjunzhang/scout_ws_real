"""Extract an initial guess without changing its time axis or model states."""
import numpy as np


def warm_start_values(plan, task, count):
    states = np.asarray([row["state"] for row in plan["samples"]], float)
    controls = np.asarray([row["control"] for row in plan["samples"][:-1]], float)
    times = np.asarray([row["t"] for row in plan["samples"]], float)
    if (states.shape != (count + 1, 28) or controls.shape != (count, 3)
            or times.shape != (count + 1,)
            or not np.allclose(times, np.arange(count + 1) * task["dt"], atol=1e-8, rtol=0)):
        raise ValueError("warm plan grid/layout mismatch")
    if not np.isfinite(states).all() or not np.isfinite(controls).all():
        raise ValueError("nonfinite warm plan")
    coordinate = plan.get("progress_parameterization")
    scale = 1.
    if coordinate is not None:
        if coordinate["method"] != "scaled_actual_speed_trapezoid":
            raise ValueError("unknown warm progress parameterization")
        scale = float(coordinate["scale"])
    if not np.isfinite(scale) or not task["progress_scale_min"] <= scale <= task["progress_scale_max"]:
        raise ValueError("warm progress scale out of bounds")
    return states.T, controls.T, scale
