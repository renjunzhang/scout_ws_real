#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include <algorithm>
#include <cmath>
#include <slosh_models/liquid_slosh_model.h>
#include <unsupported/Eigen/MatrixFunctions>

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

bool SloshDynamics::stepWithDt(
    const SloshState& state,
    double ax,
    double ay,
    double omega_z,
    double dt_sec,
    SloshState& next_state) const {
    next_state = state;
    const Eigen::Vector4d x = toEigen(state);
    if (!configured_ || !x.allFinite() || !std::isfinite(ax) || !std::isfinite(ay) ||
        !std::isfinite(dt_sec) || dt_sec <= 1e-9 || !std::isfinite(omega_n_) ||
        omega_n_ <= 0.0 || !std::isfinite(params_.damping_ratio)) {
        return false;
    }

    if (std::abs(dt_sec - params_.dt) <= 1e-12) {
        const SloshState cached_step = step(state, ax, ay, omega_z);
        const Eigen::Vector4d cached = toEigen(cached_step);
        if (!cached.allFinite()) {
            return false;
        }
        next_state = cached_step;
        return true;
    }

    Eigen::Matrix4d continuous_a = Eigen::Matrix4d::Zero();
    const double omega_sq = omega_n_ * omega_n_;
    const double damping = 2.0 * params_.damping_ratio * omega_n_;
    continuous_a(0, 1) = 1.0;
    continuous_a(1, 0) = -omega_sq;
    continuous_a(1, 1) = -damping;
    continuous_a(2, 3) = 1.0;
    continuous_a(3, 2) = -omega_sq;
    continuous_a(3, 3) = -damping;

    Eigen::Matrix<double, 4, 2> continuous_b =
        Eigen::Matrix<double, 4, 2>::Zero();
    continuous_b(1, 0) = -1.0;
    continuous_b(3, 1) = -1.0;

    Eigen::Matrix<double, 6, 6> augmented =
        Eigen::Matrix<double, 6, 6>::Zero();
    augmented.block<4, 4>(0, 0) = continuous_a * dt_sec;
    augmented.block<4, 2>(0, 4) = continuous_b * dt_sec;
    const Eigen::Matrix<double, 6, 6> discretized = augmented.exp();
    const Eigen::Matrix4d ad = discretized.block<4, 4>(0, 0);
    const Eigen::Matrix<double, 4, 2> bd = discretized.block<4, 2>(0, 4);
    if (!ad.allFinite() || !bd.allFinite()) {
        return false;
    }

    Eigen::Vector2d input;
    input << ax, ay;
    const Eigen::Vector4d next = ad * x + bd * input;
    if (!next.allFinite()) {
        return false;
    }
    next_state = fromEigen(next);
    return true;
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
