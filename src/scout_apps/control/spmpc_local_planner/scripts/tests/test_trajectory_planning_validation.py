import copy
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.task import load_task
from planning.validation import validate_plan
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_trajectory_suffix import make_plan


def task(cells=None):
    if cells is None:
        cells = [{"id": "a", "s_begin": 0., "s_end": 2.,
                  "vertices": [[-1., -1.], [3., -1.], [3., 1.], [-1., 1.]]}]
    return {
        "task_id": "t", "frame_id": "map", "route": [[0., 0.], [2., 0.]],
        "start_state": [0.] * 28, "goal_pose": [2., 0., 0.],
        "deadline": .5, "transport_duration": .5, "stop_window": .3,
        "actuator_parameters": [1., 1., 1., 1.],
        "liquid_parameters": [.2, 1., .1, .1], "height_coeff": .1,
        "motion_limits": {"actual_v_min": -.1, "v_max": 1., "omega_max": 1.,
                           "a_max": 1., "alpha_max": 1., "jerk_max": 1.},
        "region": {"id": "r", "frame_id": "map", "footprint_radius": .1,
                   "margin": .01, "cells": cells},
    }


def test_task_rejects_narrow_and_nonintersecting_cells():
    narrow = [{"id": "a", "s_begin": 0., "s_end": 2.,
               "vertices": [[0., 0.], [.1, 0.], [.1, .1], [0., .1]]}]
    with pytest.raises(ValueError, match="interior"):
        load_task(task(narrow))
    cells = [
        {"id": "a", "s_begin": 0., "s_end": 1.,
         "vertices": [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]},
        {"id": "b", "s_begin": 1., "s_end": 2.,
         "vertices": [[2., -1.], [4., -1.], [4., 1.], [2., 1.]]},
    ]
    with pytest.raises(ValueError, match="overlap"):
        load_task(task(cells))


def test_task_rejects_nonfinite_progress_empty_id_and_bad_iterations():
    bad = task(); bad["region"]["cells"][0]["s_end"] = float("nan")
    with pytest.raises(ValueError): load_task(bad)
    bad = task(); bad["region"]["cells"][0]["id"] = ""
    with pytest.raises(ValueError): load_task(bad)
    for value in (0, -1, 1.5, True):
        bad = task(); bad["max_iterations"] = value
        with pytest.raises(ValueError, match="max_iterations"): load_task(bad)
    bad = task(); bad["objective"] = {"curvatur": 1.0}
    with pytest.raises(ValueError, match="unknown objective"): load_task(bad)
    bad = task(); bad["progress_scale_max"] = float("inf")
    with pytest.raises(ValueError): load_task(bad)
    bad = task(); bad["goal_yaw_tolerance"] = np.pi + 0.01
    with pytest.raises(ValueError, match="yaw"): load_task(bad)


def test_validation_rejects_progress_overflow_and_wrong_transport_endpoint():
    plan = make_plan()
    moving = round(plan["transport_duration"] / plan["dt"])
    plan["samples"][moving]["state"][4] = 3.
    with pytest.raises(ValueError, match="route domain|endpoint"):
        validate_plan(plan)


def test_validation_rejects_dynamics_tampering_and_bad_tail():
    plan = make_plan()
    plan["samples"][2]["state"][0] += .01
    with pytest.raises(ValueError, match="rollout"):
        validate_plan(plan)
    plan = make_plan()
    moving = round(plan["transport_duration"] / plan["dt"])
    plan["samples"][moving]["phase"] = "MOVE"
    with pytest.raises(ValueError, match="phase|tail"):
        validate_plan(plan)
