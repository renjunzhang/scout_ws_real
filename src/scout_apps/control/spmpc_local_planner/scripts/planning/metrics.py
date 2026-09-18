"""Predicted liquid peaks and parking windows using the production motion kernel."""
import numpy as np
from actual_motion_kernel import functions as motion_functions
from .stopping import find_stop_time, stopping_windows


def dense_heights(task, states, substeps=4):
    """Propagate held delayed commands between nodes; keep both boundary estimates."""
    dt = task['dt']
    count = len(states) - 1
    step = motion_functions()[0]
    times = np.arange(count * substeps + 1) * (dt / substeps)
    heights = np.zeros(len(times))
    for k in range(count):
        z = np.r_[states[k, :4], states[k, 5], states[k, 24:28]]
        for j in range(substeps + 1):
            index = k * substeps + j
            height = task['height_coeff'] * float(np.hypot(z[5], z[7]))
            heights[index] = max(heights[index], height)
            if j < substeps:
                z = np.asarray(step(z, states[k, [8, 13]], task['actuator_parameters'],
                                    task['liquid_parameters'], dt/substeps)).ravel()
    heights[-1] = max(heights[-1], task['height_coeff'] * float(np.hypot(states[-1, 24], states[-1, 26])))
    return times, heights


def stopping_metrics(task, states, controls, dense_times, heights, clear_tolerance=2e-6):
    """Measure windows without changing legacy plan acceptance or declaring new caps."""
    times = np.arange(len(states)) * task['dt']
    stopped = find_stop_time(times, states, controls, task, clear_tolerance)
    result = dict(t_stop_sec=stopped, clear_tolerance=clear_tolerance,
                  basis='predicted full-state sustained stop; four motion substeps per model interval')
    if stopped is None:
        result['windows'] = None
    else:
        result['windows'] = stopping_windows(dense_times, heights, stopped, task['stop_window'],
                                             max_sample_gap_sec=task['dt']/4 + 1e-10)
    return result
