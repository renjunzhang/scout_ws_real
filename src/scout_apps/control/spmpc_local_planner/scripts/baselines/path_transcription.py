"""Regular fixed-path equalities for a delayed nonholonomic vehicle.

The planar RK4 displacement is linear in initial linear speed and its held
delayed command (yaw dynamics do not depend on either). Factoring out path
speed removes the vanishing steering derivative of Cartesian path constraints
near rest, without changing the path or the production motion integrator.
"""
import casadi as ca
import numpy as np


def seed_progress_from_motion(states, spline, coordinate_seed, moving):
    """Keep seed progress moving while the delayed actuator is still braking.

    This only supplies a numerical initial guess. The NLP retains its original
    duration, endpoint and full dynamics constraints.
    """
    distance = np.linalg.norm(np.diff(states[:moving+1, :2], axis=0), axis=1)
    metric = np.linalg.norm(spline((coordinate_seed[:moving]+coordinate_seed[1:moving+1])*.5, 1), axis=1)
    increments = distance/metric
    if not np.isfinite(increments).all() or increments.sum() <= 0.:
        raise ValueError("fixed-path initial guess has no finite forward motion")
    coordinate = np.full(len(states), spline.x[-1])
    coordinate[:moving+1] = np.r_[0., np.cumsum(increments)]*(spline.x[-1]/increments.sum())
    return coordinate


def cubic_chord(spline):
    """Exact divided difference (g(s+h)-g(s))/h, with its h=0 limit.

    A C2 cubic spline is a base cubic plus truncated cubic terms at its knots.
    Factoring the difference of cubes avoids subtracting nearby coordinates;
    crossing a knot is handled by its fraction of the interval. h must be >=0.
    """
    s, h = ca.MX.sym("s"), ca.MX.sym("h")
    q = s-float(spline.x[0])
    c = spline.c
    chord = ca.DM(c[0, 0])*(3*q*q+3*q*h+h*h)+ca.DM(c[1, 0])*(2*q+h)+ca.DM(c[2, 0])
    for k in range(1, len(spline.x)-1):
        distance = s-float(spline.x[k])
        before, after = ca.fmax(distance, 0.), ca.fmax(distance+h, 0.)
        fraction = ca.if_else(distance >= 0., 1.,
            ca.if_else(distance+h <= 0., 0., (distance+h)/ca.if_else(h > 0., h, 1.)))
        chord += ca.DM(c[0, k]-c[0, k-1])*fraction*(after*after+after*before+before*before)
    return ca.Function("frozen_cubic_chord", [s, h], [chord])


def normalized_displacement(step):
    """Production planar step with unit path speed and zero position origin."""
    x, u = ca.MX.sym("x", 28), ca.MX.sym("u", 3)
    normalized_v, normalized_command = ca.MX.sym("normalized_v"), ca.MX.sym("normalized_command")
    normalized = ca.vertcat(ca.DM.zeros(2), x[2], normalized_v, x[4:8], normalized_command, x[9:])
    return ca.Function("normalized_planar_displacement", [x, u, normalized_v, normalized_command],
                       [step(normalized, u)[:2]])


def constrain_fixed_path(opt, states, controls, step, spline, dt, first, end, linear_queue_zero_from):
    """Enforce every moving chord exactly, with a regular tangent limit at rest.

    v = path_speed * normalized_v and delayed_v = path_speed * normalized_cmd.
    The remaining vector equality divides out that common path_speed. With the
    stationary prefix anchored on g(0), production dynamics then imply every
    subsequent node lies on g(s). The stopped tail is propagated normally.
    """
    count = end-first
    normalized = opt.variable(2, count)
    speed = controls[2, first:end]
    # Known zero states must not be represented as speed*z=0 near rest. Their
    # physical zero is already implied by the stationary prefix / drained FIFO;
    # set the normalized value directly to remove a vanishing Jacobian pivot.
    opt.subject_to(normalized[0, 0] == 0.)
    opt.subject_to(states[3, first+1:end] == speed[1:]*normalized[0, 1:])
    active = linear_queue_zero_from-first
    opt.subject_to(states[8, first:linear_queue_zero_from] == speed[:active]*normalized[1, :active])
    if active < count:
        opt.subject_to(normalized[1, active:] == 0.)
    displacement = normalized_displacement(step)
    chord = cubic_chord(spline)
    opt.subject_to(displacement.map(count)(states[:, first:end], controls[:, first:end],
        normalized[0, :], normalized[1, :]) == dt*chord.map(count)(states[4, first:end], dt*speed))
    return normalized
