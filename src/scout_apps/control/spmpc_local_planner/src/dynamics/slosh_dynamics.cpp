#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include <algorithm>
#include <cmath>
#include <slosh_models/liquid_slosh_model.h>

namespace spmpc_local_planner {

bool SloshDynamics::configure(const SloshModelParams& params) {
    params_ = params;

    slosh_models::LiquidSloshModel model;
    slosh_models::LiquidSloshModel::Params p;
    p.R = params.container_radius;
    p.h = params.liquid_height;
    p.rho = params.liquid_density;
    p.dt = params.dt;
    p.mode_index = params.mode_index;
    p.zeta = params.damping_ratio;
    p.use_linear_model = params.use_linear_model;
    p.use_parabola_term = params.use_parabola_term;

    if (!model.configure(p)) {
        configured_ = false;
        return false;
    }

    model.getDiscreteMatrices(Ad_, Bd_);
    omega_n_ = model.getModalParams().omega_n;
    height_coeff_ = model.getModalParams().height_coeff;
    configured_ = true;
    return true;
}

SloshState SloshDynamics::step(
    const SloshState& state,
    double ax,
    double ay,
    double /*omega_z*/) const {
    if (!configured_) {
        return state;
    }
    Eigen::Vector2d u;
    u << ax, ay;
    return fromEigen(Ad_ * toEigen(state) + Bd_ * u);
}

double SloshDynamics::height(const SloshState& state, double omega_z) const {
    if (!configured_) {
        return 0.0;
    }
    const double modal = height_coeff_ * etaNorm(state);
    double parabola = 0.0;
    if (params_.use_parabola_term) {
        const double r = params_.container_radius;
        parabola = r * r * omega_z * omega_z / (4.0 * 9.81);
    }
    return modal + parabola;
}

double SloshDynamics::etaNorm(const SloshState& state) const {
    return std::hypot(state.eta_x, state.eta_y);
}

double SloshDynamics::etaDotNorm(const SloshState& state) const {
    return std::hypot(state.eta_x_dot, state.eta_y_dot);
}

Eigen::Vector4d SloshDynamics::toEigen(const SloshState& state) const {
    Eigen::Vector4d x;
    x << state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot;
    return x;
}

SloshState SloshDynamics::fromEigen(const Eigen::Vector4d& state) const {
    SloshState out;
    out.eta_x = state(0);
    out.eta_x_dot = state(1);
    out.eta_y = state(2);
    out.eta_y_dot = state(3);
    return out;
}

}  // namespace spmpc_local_planner
