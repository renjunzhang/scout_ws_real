"""Smooth weak route guide for an NLP with free geometry.

A polyline tangent jumps at knots. With different contour/lag weights and a
nonzero path offset this even makes the objective discontinuous. Interpolate
through the same route points with a C2 guide; the physical corridor remains
separately constrained.
"""
import casadi as ca
import numpy as np


def route_guide(route_s, route):
    coordinate = ca.MX.sym('s')
    if len(route_s) >= 4:
        coordinates = [ca.interpolant('guide_' + str(i), 'bspline',
                                      [route_s], route[:, i])(coordinate) for i in range(2)]
    else:
        # The line/quadratic through 2/3 points is smooth without interior knots.
        coordinates = [ca.polyval(np.polyfit(route_s, route[:, i], len(route_s)-1), coordinate)
                       for i in range(2)]
    position = ca.vertcat(*coordinates)
    tangent = ca.jacobian(position, coordinate)
    return ca.Function('route_guide', [coordinate],
                       [position, tangent/ca.sqrt(ca.sumsqr(tangent)+1e-12)])
