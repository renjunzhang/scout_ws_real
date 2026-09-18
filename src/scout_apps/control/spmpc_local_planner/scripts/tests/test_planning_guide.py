import sys
from pathlib import Path
import casadi as ca
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.guide import route_guide


def test_offset_cost_is_continuous_at_original_route_knots():
    route = np.array([[0., 0.], [1., .5], [2., -.5], [3., 0.]])
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    guide = route_guide(distance, route)
    for s, expected in zip(distance, route):
        np.testing.assert_allclose(np.asarray(guide(s)[0]).ravel(), expected, atol=1e-12)
    s = ca.MX.sym('s')
    position, tangent = guide(s)
    delta = ca.DM([1.2, .8])-position
    cost = .02*(delta[0]*tangent[1]-delta[1]*tangent[0])**2 + .2*ca.dot(delta,tangent)**2
    evaluate = ca.Function('guide_cost', [s], [cost, ca.gradient(cost,s)])
    for knot in distance[1:-1]:
        left, right = evaluate(knot-1e-8), evaluate(knot+1e-8)
        np.testing.assert_allclose([float(v) for v in left], [float(v) for v in right], atol=1e-7)


def test_short_route_has_a_smooth_line_or_quadratic_guide():
    for count in (2, 3):
        distance = np.arange(count, dtype=float)
        route = np.column_stack([distance, distance**2])
        guide = route_guide(distance, route)
        np.testing.assert_allclose(np.asarray(guide(.5)[0]).ravel(), [.5, .5 if count==2 else .25], atol=1e-12)
