"""Single symbolic definition of the planar, rotating-container liquid model.

State: [eta_x, eta_x_dot, eta_y, eta_y_dot], in container axes.
Excitation: [ax_C, ay_C, omega_actual, alpha_actual]. Acceleration is already
at the container centre; neither lever-arm nor v*omega is added in this RHS.
Parameters: [2*zeta*omega_n, omega_n**2, kappa_x, kappa_y]. The current modal
mass-displacement convention has kappa_x = kappa_y = 1.

Assumptions: horizontal circular container, small first-mode displacement,
isotropic restoring force and damping relative to the container. This model
does not establish a physical slosh bound or identify the actuator response.
"""

import casadi as ca
import math

from model_contract import MODEL_VERSION, RK4_SUBSTEPS, MAX_RK4_STEP_SEC


def integration_substeps(dt):
    return max(RK4_SUBSTEPS, math.ceil(dt / MAX_RK4_STEP_SEC - 1e-12))


def slosh_rhs(state, excitation, params):
    ex, dx, ey, dy = ca.vertsplit(state)
    ax, ay, omega, alpha = ca.vertsplit(excitation)
    damping, wn2, kx, ky = ca.vertsplit(params)
    return ca.vertcat(
        dx,
        -damping * dx - wn2 * ex - kx * ax
        + 2 * omega * dy + alpha * ey + omega**2 * ex,
        dy,
        -damping * dy - wn2 * ey - ky * ay
        - 2 * omega * dx - alpha * ex + omega**2 * ey,
    )


def rk4_step(rhs, state, dt):
    k1 = rhs(state)
    k2 = rhs(state + dt * k1 / 2)
    k3 = rhs(state + dt * k2 / 2)
    k4 = rhs(state + dt * k3)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def functions():
    state = ca.SX.sym("state", 4)
    excitation = ca.SX.sym("excitation", 4)
    liquid = ca.SX.sym("liquid", 4)
    dt = ca.SX.sym("dt")
    return (
        ca.Function("spmpc_slosh_rhs", [state, excitation, liquid],
                    [slosh_rhs(state, excitation, liquid)]),
        ca.Function("spmpc_slosh_rk4", [state, excitation, liquid, dt],
                    [rk4_step(lambda z: slosh_rhs(z, excitation, liquid), state, dt)]),
    )
