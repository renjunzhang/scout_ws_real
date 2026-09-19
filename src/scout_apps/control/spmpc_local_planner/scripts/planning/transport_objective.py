"""Shared transport-slosh qualification for predicted and observed windows."""
import math
import numbers

METRICS = ('transport_peak_m', 'transport_p95_m', 'transport_rms_m')


def normalize_limits(limits):
    if not isinstance(limits, dict) or set(limits) != set(METRICS):
        raise ValueError('slosh limits require exactly transport peak, P95 and RMS in metres')
    if any(isinstance(v, bool) or not isinstance(v, numbers.Real) or
           not math.isfinite(v) or v <= 0 for v in limits.values()):
        raise ValueError('slosh limits must be finite positive real numbers')
    return {key: float(limits[key]) for key in METRICS}


def evaluate_transport(windows, limits):
    """Require complete windows and all three metrics strictly below frozen limits.

    A prediction gate is not a closed-loop result. Call the same function on
    recorded transport windows before reporting an executed plan as qualified.
    """
    limits = normalize_limits(limits)
    windows = {} if windows is None else windows
    values = {key: windows.get(key) for key in METRICS}
    failures = [] if windows.get('covered') else ['incomplete_windows']
    for key, value in values.items():
        if (isinstance(value, bool) or not isinstance(value, numbers.Real) or
                not math.isfinite(value) or value < 0 or value >= limits[key]):
            failures.append(key)
    return dict(passed=not failures, metrics=values, upper_exclusive_m=limits, failures=failures)
