"""ROS-independent geometry, spatial speed reference and region expressions.

All motion quantities are actual body motion. Geometry regularization remains
finite and penalizes rotation at rest; it does not mask low-speed nodes out.
"""
import casadi as ca

MAX_REGION_FACES = 8
REFERENCE_KNOTS = 4
PLANNING_PARAMETER_DEFAULTS = {
    "w_curvature": 0.0,
    "w_curvature_rate": 0.0,
    "curvature_speed_floor": 0.05,
    "curvature_ref": 1.0,
    "curvature_rate_ref": 1.0,
    "reference_curvature_speed_enable": 1.0,
    "reference_mode": 0.0,
}
for i in range(REFERENCE_KNOTS):
    PLANNING_PARAMETER_DEFAULTS[f"reference_s{i}"] = float(i)
for name in ("v", "vs"):
    for i in range(REFERENCE_KNOTS):
        PLANNING_PARAMETER_DEFAULTS[f"reference_{name}{i}"] = 0.0
PLANNING_PARAMETER_DEFAULTS.update(region_active=0.0, region_s_begin=0.0, region_s_end=1.0)
for i in range(MAX_REGION_FACES):
    PLANNING_PARAMETER_DEFAULTS.update({f"region_nx{i}": 0.0,
                                       f"region_ny{i}": 0.0,
                                       f"region_offset{i}": 1.0})
PLANNING_PARAMETER_DEFAULTS.update(task_goal_active=0., task_goal_x=0., task_goal_y=0., task_goal_yaw=0.,
    task_goal_position_tolerance=.05, task_goal_yaw_tolerance=.1,
    task_goal_speed_tolerance=.01, task_goal_omega_tolerance=.02, task_goal_require_yaw=0.,
    w_task_goal=0., task_goal_position_scale=.3, task_goal_yaw_scale=1.)


def piecewise_speed(s, p, idx, field):
    knots = [p[idx[f"reference_s{i}"]] for i in range(REFERENCE_KNOTS)]
    values = [p[idx[f"reference_{field}{i}"]] for i in range(REFERENCE_KNOTS)]
    # Inputs must have strictly increasing knots even when the mode is inactive.
    pieces = [values[i] + (s-knots[i])*(values[i+1]-values[i])/(knots[i+1]-knots[i])
              for i in range(REFERENCE_KNOTS-1)]
    return ca.if_else(s < knots[1], pieces[0],
                      ca.if_else(s < knots[2], pieces[1], pieces[2]))


def speed_references(x, p, idx, cruise):
    mode = p[idx["reference_mode"]]
    v = ca.if_else(mode == 1, piecewise_speed(x[4], p, idx, "v"), p[idx["reference_v0"]])
    vs = ca.if_else(mode == 1, piecewise_speed(x[4], p, idx, "vs"), p[idx["reference_vs0"]])
    return ca.if_else(mode == 0, cruise, v), ca.if_else(mode == 0, cruise, vs)


def geometry_terms(x, actual_a, actual_alpha, p, idx, v_scale):
    speed = ca.sqrt(x[3]**2 + p[idx["curvature_speed_floor"]]**2)
    curvature = x[5] / speed
    curvature_dot = actual_alpha / speed - x[5]*x[3]*actual_a / speed**3
    # Arc-length quadrature with a nonzero speed floor. At normal motion it
    # approaches integral(kappa^2 dl); spin/stop costs remain explicitly finite.
    amplitude = p[idx["w_curvature"]]*(curvature/p[idx["curvature_ref"]])**2*speed/v_scale
    spatial_change = p[idx["w_curvature_rate"]]*(curvature_dot/speed/p[idx["curvature_rate_ref"]])**2*speed/v_scale
    return amplitude, spatial_change


def region_reference_constraints(x, p, idx):
    active = p[idx["region_active"]]
    rows = [active*(p[idx[f"region_nx{i}"]]*x[0] + p[idx[f"region_ny{i}"]]*x[1]
                    - p[idx[f"region_offset{i}"]]) - (1-active)
            for i in range(MAX_REGION_FACES)]
    rows.extend((active*(p[idx["region_s_begin"]]-x[4])-(1-active),
                 active*(x[4]-p[idx["region_s_end"]])-(1-active)))
    spatial = ca.if_else(p[idx["reference_mode"]] > 0, 1.0, 0.0)
    rows.extend((spatial*(p[idx["reference_s0"]]-x[4])-(1-spatial),
                 spatial*(x[4]-p[idx["reference_s3"]])-(1-spatial)))
    goal=p[idx["task_goal_active"]]
    yaw=goal*p[idx["task_goal_require_yaw"]]
    distance=(x[0]-p[idx["task_goal_x"]])**2+(x[1]-p[idx["task_goal_y"]])**2
    rows.extend((goal*(distance/p[idx["task_goal_position_tolerance"]]**2-1)-(1-goal),
        yaw*(ca.cos(p[idx["task_goal_yaw_tolerance"]])-ca.cos(x[2]-p[idx["task_goal_yaw"]]))-(1-yaw),
        goal*((x[3]/p[idx["task_goal_speed_tolerance"]])**2-1)-(1-goal),
        goal*((x[5]/p[idx["task_goal_omega_tolerance"]])**2-1)-(1-goal)))
    return ca.vertcat(*rows)


def task_goal_cost(x,p,idx):
    position=((x[0]-p[idx["task_goal_x"]])**2+(x[1]-p[idx["task_goal_y"]])**2)/p[idx["task_goal_position_scale"]]**2
    yaw=p[idx["task_goal_require_yaw"]]*(1-ca.cos(x[2]-p[idx["task_goal_yaw"]]))/p[idx["task_goal_yaw_scale"]]**2
    return p[idx["w_task_goal"]]*(position+yaw)
