import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest

from planning.liquid_policy import height_limits, normalize_policy, objective_end_index


POLICY = {"mode": "transport_tail_v1", "move_height_m": .12,
          "tail_height_m": .04, "height_tolerance_m": .01}


def task(policy=POLICY):
    return {"liquid_policy": policy, "transport_duration": 2.,
            "deadline": 4., "stop_window": .5, "dt": .1}


@pytest.mark.parametrize("bad", [
    {**POLICY, "mode": "other"},
    {k: v for k, v in POLICY.items() if k != "tail_height_m"},
    {**POLICY, "tail_height_m": np.nan},
    {**POLICY, "move_height_m": True},
    {**POLICY, "height_tolerance_m": .05},
])
def test_policy_validation(bad):
    with pytest.raises(ValueError):
        normalize_policy(bad)


def test_limits_boundary_and_early_stop():
    t = task()
    np.testing.assert_array_equal(height_limits(t, [1.99, 2., 2.1]), [.12, .04, .04])
    np.testing.assert_array_equal(height_limits(t, [1., 1.5, 1.6], t_stop=1.5), [.12, .04, .04])


def test_objective_end_uses_transport_with_policy_and_legacy_without():
    assert objective_end_index(task()) == 25
    assert objective_end_index(task(None)) == 45


def test_normalize_returns_copy():
    source = dict(POLICY)
    result = normalize_policy(source)
    assert result == source
    assert result is not source
