"""Optional offline envelope compatible with the frozen terminal speed caps.

Continue the runtime slowdown slope outside its activation boundary so the
planner can brake before entry. Using both remaining progress and goal distance
covers the predicted terminal-node cap and the measured distance envelope.
The continued slope stays below the capture cap; command braking also accounts
for actuator gain. This is a conservative planning bound, not a replica of the
runtime stop state machine or its yaw controller.
"""
import numbers
import casadi as ca
import numpy as np


def normalize_policy(value, task):
    if value is None:
        return None
    keys = {'mode', 'slowdown_distance_m', 'slowdown_speed_mps'}
    if not isinstance(value, dict) or set(value) != keys or value['mode'] != 'terminal_speed_envelope_v1':
        raise ValueError('invalid terminal_speed_policy')
    for key in keys-{'mode'}:
        v = value[key]
        if isinstance(v, bool) or not isinstance(v, numbers.Real) or not np.isfinite(v):
            raise ValueError('terminal speed policy needs finite numeric values')
    if (value['slowdown_distance_m'] <= task['goal_position_tolerance'] or
            not 0 < value['slowdown_speed_mps'] <= task['motion_limits']['v_max']):
        raise ValueError('invalid terminal speed policy bounds')
    return dict(value)


def speed_limits(task, remaining, distance):
    """Return actual-speed cap and command-speed cap squared (m/s, m²/s²).

    Finite symbolic derivatives at zero speed: keep the brake constraint squared
    instead of differentiating sqrt(max(distance - half_tolerance, 0)).
    """
    policy = task['terminal_speed_policy']
    tolerance = task['goal_position_tolerance']
    vmax = policy['slowdown_speed_mps']
    ramp = vmax*(ca.fmin(remaining, distance)-tolerance)/(policy['slowdown_distance_m']-tolerance)
    speed = ca.fmin(task['motion_limits']['v_max'], ca.fmax(.2*vmax, ramp))
    brake_squared = 2*task['motion_limits']['a_max']*ca.fmax(0, distance-.5*tolerance)
    gain = max(1., task['actuator_parameters'][2])
    return speed, ca.fmin(speed**2, brake_squared)/gain**2
