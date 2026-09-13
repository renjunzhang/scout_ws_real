"""Compose actual actuator/robot motion with the independent liquid RHS.

Only this layer turns held delayed commands into actual container excitation.
Both generated C propagation and the OCP consume this symbolic definition.
"""

import casadi as ca
from slosh_kernel import slosh_rhs, rk4_step


def actual_motion_rhs(state, delayed_command, actuator, liquid):
    """[x,y,yaw,v_actual,omega_actual,eta(4)] with held delayed commands.

    Evaluate actual accelerations at every RK4 evaluation, including the
    lateral v_actual*omega_actual term. Container centre coincides with base.
    actuator = [tau_v, tau_omega, gain_v, gain_omega].
    """
    v, omega = state[3], state[4]
    a = (actuator[2] * delayed_command[0] - v) / actuator[0]
    alpha = (actuator[3] * delayed_command[1] - omega) / actuator[1]
    return ca.vertcat(
        v * ca.cos(state[2]), v * ca.sin(state[2]), omega, a, alpha,
        slosh_rhs(state[5:9], ca.vertcat(a, v * omega, omega, alpha), liquid),
    )


def functions():
    motion = ca.SX.sym("motion", 9)
    delayed = ca.SX.sym("delayed_command", 2)
    actuator = ca.SX.sym("actuator", 4)
    liquid = ca.SX.sym("liquid", 4)
    dt = ca.SX.sym("dt")
    return (ca.Function("spmpc_actual_motion_rk4", [motion, delayed, actuator, liquid, dt],
                        [rk4_step(lambda z: actual_motion_rhs(z, delayed, actuator, liquid),
                                  motion, dt)]),)
