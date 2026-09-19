from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planning.transport_objective import METRICS, evaluate_transport, normalize_limits

LIMITS = dict(zip(METRICS, [.001, .0005, .0002]))


def test_all_three_metrics_must_strictly_improve():
    windows = dict(covered=True, **{key: value/2 for key, value in LIMITS.items()})
    assert evaluate_transport(windows, LIMITS)['passed']
    for metric, limit in LIMITS.items():
        for value in (limit, limit*1.01, None, float('nan')):
            result = evaluate_transport(dict(windows, **{metric: value}), LIMITS)
            assert not result['passed']
            assert metric in result['failures']
    assert not evaluate_transport(dict(windows, covered=False), LIMITS)['passed']
    assert not evaluate_transport(None, LIMITS)['passed']


@pytest.mark.parametrize('limits', [{}, {**LIMITS, 'extra': 1},
                                   {**LIMITS, 'transport_peak_m': True},
                                   {**LIMITS, 'transport_peak_m': -1}])
def test_incomplete_or_invalid_frozen_limits_are_rejected(limits):
    with pytest.raises(ValueError):
        normalize_limits(limits)
