import copy
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.validation import validate_plan
from planning.suffix import remaining_task
from planning_fixture import make_plan

POLICY = dict(mode='transport_tail_v1', move_height_m=.01,
              tail_height_m=.001, height_tolerance_m=1e-6)


def new_plan():
    plan = make_plan()
    plan['task']['liquid_policy'] = copy.deepcopy(POLICY)
    plan['liquid_policy'] = copy.deepcopy(POLICY)
    return plan


def test_new_mode_requires_matching_explicit_contract_and_preserves_suffix():
    plan = new_plan()
    validate_plan(plan)
    suffix = remaining_task(plan, plan['samples'][15]['state'], .5)
    assert suffix['liquid_policy'] == POLICY
    assert suffix['transport_duration'] == plan['transport_duration']-.5
    del plan['liquid_policy']
    with pytest.raises(ValueError, match='contract mismatch: liquid_policy'):
        validate_plan(plan)


def test_early_stop_substep_peak_cannot_hide_before_nominal_tail_or_last_frame():
    plan = new_plan()
    stopping = validate_plan(plan)['stopping_evaluation']['t_stop_sec']
    assert stopping < plan['transport_duration']
    times = np.arange(601)/120.
    heights = np.zeros_like(times)
    # An inter-node peak after physical stopping but before nominal T.
    index = round(stopping*120) + 1
    assert index % 4 != 0 and times[index] < plan['transport_duration']
    heights[index] = .002
    with patch('planning.validation.dense_heights', return_value=(times, heights)):
        with pytest.raises(ValueError, match='dense_transport_tail_liquid_cap'):
            validate_plan(plan)


def test_height_acceptance_uses_explicit_metre_tolerance():
    plan = new_plan()
    times = np.arange(601)/120.
    heights = np.full_like(times, POLICY['tail_height_m'] + .5*POLICY['height_tolerance_m'])
    with patch('planning.validation.dense_heights', return_value=(times, heights)):
        validate_plan(plan)
    heights[-1] += POLICY['height_tolerance_m']
    with patch('planning.validation.dense_heights', return_value=(times, heights)):
        with pytest.raises(ValueError, match='dense_transport_tail_liquid_cap'):
            validate_plan(plan)
