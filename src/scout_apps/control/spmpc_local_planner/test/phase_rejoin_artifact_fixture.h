#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace spmpc_local_planner_test {

struct FixtureState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 1.0;
    double omega = 0.0;
    double s = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct FixtureControl {
    double a = 0.0;
    double alpha = 0.0;
    double v_s = 0.0;
};

inline FixtureState fixtureDerivative(const FixtureState& state,
                                      const FixtureControl& control) {
    FixtureState out;
    out.x = state.v * std::cos(state.yaw);
    out.y = state.v * std::sin(state.yaw);
    out.yaw = state.omega;
    out.v = control.a;
    out.omega = control.alpha;
    out.s = control.v_s;
    out.eta_x = state.eta_x_dot;
    out.eta_x_dot = -0.2 * state.eta_x_dot - 4.0 * state.eta_x - control.a;
    out.eta_y = state.eta_y_dot;
    out.eta_y_dot = -0.2 * state.eta_y_dot - 4.0 * state.eta_y -
        state.v * state.omega;
    return out;
}

inline FixtureState fixtureAdd(const FixtureState& state,
                               const FixtureState& change,
                               double scale) {
    FixtureState out;
    out.x = state.x + scale * change.x;
    out.y = state.y + scale * change.y;
    out.yaw = state.yaw + scale * change.yaw;
    out.v = state.v + scale * change.v;
    out.omega = state.omega + scale * change.omega;
    out.s = state.s + scale * change.s;
    out.eta_x = state.eta_x + scale * change.eta_x;
    out.eta_x_dot = state.eta_x_dot + scale * change.eta_x_dot;
    out.eta_y = state.eta_y + scale * change.eta_y;
    out.eta_y_dot = state.eta_y_dot + scale * change.eta_y_dot;
    return out;
}

inline FixtureState fixtureStep(const FixtureState& state,
                                const FixtureControl& control) {
    constexpr double dt = 0.1;
    const FixtureState k1 = fixtureDerivative(state, control);
    const FixtureState k2 = fixtureDerivative(fixtureAdd(state, k1, 0.5 * dt), control);
    const FixtureState k3 = fixtureDerivative(fixtureAdd(state, k2, 0.5 * dt), control);
    const FixtureState k4 = fixtureDerivative(fixtureAdd(state, k3, dt), control);
    FixtureState out = state;
#define SPMPC_FIXTURE_RK4(field) \
    out.field += dt * (k1.field + 2.0 * k2.field + \
                       2.0 * k3.field + k4.field) / 6.0
    SPMPC_FIXTURE_RK4(x);
    SPMPC_FIXTURE_RK4(y);
    SPMPC_FIXTURE_RK4(yaw);
    SPMPC_FIXTURE_RK4(v);
    SPMPC_FIXTURE_RK4(omega);
    SPMPC_FIXTURE_RK4(s);
    SPMPC_FIXTURE_RK4(eta_x);
    SPMPC_FIXTURE_RK4(eta_x_dot);
    SPMPC_FIXTURE_RK4(eta_y);
    SPMPC_FIXTURE_RK4(eta_y_dot);
#undef SPMPC_FIXTURE_RK4
    return out;
}

inline std::string completeArtifactText() {
    std::ostringstream out;
    out.precision(17);
    out << "# schema=phase_rejoin_empirical_v2\n"
        << "# evidence_level=development_only\n"
        << "# source=unit_test_complete_tail\n"
        << "# contract_id=test_contract\n"
        << "# frame_id=map\n"
        << "# dt=0.1\n"
        << "# path_length=3.0\n"
        << "# terminal_contract=stop_settle_zero_hold_v1\n"
        << "# recovery_contract=nominal_command_v1\n"
        << "# terminal_zero_hold_steps=11\n"
        << "# terminal_eta_norm_max=1.0\n"
        << "# terminal_eta_dot_norm_max=1.0\n"
        << "# two_zeta_omega_n=0.2\n"
        << "# omega_n_sq=4.0\n"
        << "# kappa_x=1.0\n"
        << "# kappa_y=1.0\n"
        << "# dynamics_tolerance=1e-8\n"
        << "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
        << "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
        << "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot\n";

    FixtureState state;
    for (int index = 0; index < 46; ++index) {
        FixtureControl control;
        if (index < 25) {
            control.v_s = 1.0;
        } else if (index < 35) {
            control.a = -1.0;
            control.v_s = std::max(0.0, state.v - 0.05);
        }
        const FixtureState next = index + 1 < 46
            ? fixtureStep(state, control)
            : state;
        out << index << ',' << 0.1 * index << ','
            << state.s << ',' << state.x << ',' << state.y << ','
            << state.yaw << ',' << state.v << ',' << state.omega << ','
            << state.eta_x << ',' << state.eta_x_dot << ','
            << state.eta_y << ',' << state.eta_y_dot << ','
            << control.a << ',' << control.alpha << ',' << control.v_s << ','
            << next.v << ',' << next.omega << ','
            << next.v << ',' << next.omega << ','
            << "5,5,6.3,5,5,5,5,5,5\n";
        state = next;
    }
    return out.str();
}

}  // namespace spmpc_local_planner_test
