#include "spmpc_local_planner/phase_rejoin/nominal_dynamics.h"

#include <cmath>

namespace spmpc_local_planner {
namespace {

NominalDynamicsState addScaled(const NominalDynamicsState& state,
                               const NominalDynamicsState& derivative,
                               double scale) {
    NominalDynamicsState out;
    out.x = state.x + scale * derivative.x;
    out.y = state.y + scale * derivative.y;
    out.yaw = state.yaw + scale * derivative.yaw;
    out.v = state.v + scale * derivative.v;
    out.omega = state.omega + scale * derivative.omega;
    out.s = state.s + scale * derivative.s;
    out.eta_x = state.eta_x + scale * derivative.eta_x;
    out.eta_x_dot = state.eta_x_dot + scale * derivative.eta_x_dot;
    out.eta_y = state.eta_y + scale * derivative.eta_y;
    out.eta_y_dot = state.eta_y_dot + scale * derivative.eta_y_dot;
    return out;
}

NominalDynamicsState derivative(const NominalDynamicsState& state,
                                const NominalDynamicsControl& control,
                                const NominalDynamicsModel& model) {
    NominalDynamicsState out;
    out.x = state.v * std::cos(state.yaw);
    out.y = state.v * std::sin(state.yaw);
    out.yaw = state.omega;
    out.v = control.a;
    out.omega = control.alpha;
    out.s = control.v_s;
    out.eta_x = state.eta_x_dot;
    out.eta_x_dot =
        -model.two_zeta_omega_n * state.eta_x_dot -
        model.omega_n_sq * state.eta_x - model.kappa_x * control.a;
    out.eta_y = state.eta_y_dot;
    out.eta_y_dot =
        -model.two_zeta_omega_n * state.eta_y_dot -
        model.omega_n_sq * state.eta_y -
        model.kappa_y * state.v * state.omega;
    return out;
}

}  // namespace

NominalDynamicsState phaseNominalRk4Step(
    const NominalDynamicsState& state,
    const NominalDynamicsControl& control,
    const NominalDynamicsModel& model) {
    const double half_dt = 0.5 * model.dt;
    const NominalDynamicsState k1 = derivative(state, control, model);
    const NominalDynamicsState k2 = derivative(
        addScaled(state, k1, half_dt), control, model);
    const NominalDynamicsState k3 = derivative(
        addScaled(state, k2, half_dt), control, model);
    const NominalDynamicsState k4 = derivative(
        addScaled(state, k3, model.dt), control, model);

    NominalDynamicsState out = state;
    const double scale = model.dt / 6.0;
#define SPMPC_RK4_FIELD(field) \
    out.field += scale * (k1.field + 2.0 * k2.field + \
                          2.0 * k3.field + k4.field)
    SPMPC_RK4_FIELD(x);
    SPMPC_RK4_FIELD(y);
    SPMPC_RK4_FIELD(yaw);
    SPMPC_RK4_FIELD(v);
    SPMPC_RK4_FIELD(omega);
    SPMPC_RK4_FIELD(s);
    SPMPC_RK4_FIELD(eta_x);
    SPMPC_RK4_FIELD(eta_x_dot);
    SPMPC_RK4_FIELD(eta_y);
    SPMPC_RK4_FIELD(eta_y_dot);
#undef SPMPC_RK4_FIELD
    return out;
}

NominalDynamicsState phaseNominalRk4Step(
    const PhaseNominalSample& sample,
    const NominalArtifactMetadata& metadata) {
    NominalDynamicsState state;
    state.x = sample.x;
    state.y = sample.y;
    state.yaw = sample.yaw;
    state.v = sample.v;
    state.omega = sample.omega;
    state.s = sample.s;
    state.eta_x = sample.eta_x;
    state.eta_x_dot = sample.eta_x_dot;
    state.eta_y = sample.eta_y;
    state.eta_y_dot = sample.eta_y_dot;
    NominalDynamicsControl control;
    control.a = sample.a;
    control.alpha = sample.alpha;
    control.v_s = sample.v_s;
    NominalDynamicsModel model;
    model.dt = metadata.dt;
    model.two_zeta_omega_n = metadata.two_zeta_omega_n;
    model.omega_n_sq = metadata.omega_n_sq;
    model.kappa_x = metadata.kappa_x;
    model.kappa_y = metadata.kappa_y;
    return phaseNominalRk4Step(state, control, model);
}

}  // namespace spmpc_local_planner
