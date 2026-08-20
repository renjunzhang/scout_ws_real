#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

namespace spmpc_local_planner {

namespace {

// Roots of J'_1(xi_1n) retained byte-for-byte from the historical
// slosh_models::LiquidSloshModel contract.
constexpr double kModalRoots[] = {
    1.8412,
    5.3314,
    8.5363,
    11.7060,
    14.8636,
};
constexpr int kMaxModeIndex = 5;
constexpr double kGravity = 9.81;

}  // namespace

bool SloshDynamics::configure(const SloshModelParams& params) {
    params_ = params;

    if (params_.container_radius <= 0.0 || params_.liquid_height <= 0.0 ||
        params_.liquid_density <= 0.0 || params_.dt <= 1e-4 ||
        params_.mode_index < 1 || params_.mode_index > kMaxModeIndex) {
        configured_ = false;
        return false;
    }

    const double modal_root = kModalRoots[params_.mode_index - 1];
    const double liquid_mass = params_.liquid_density * M_PI *
                               params_.container_radius *
                               params_.container_radius *
                               params_.liquid_height;
    const double frequency_argument = modal_root * params_.liquid_height /
                                      params_.container_radius;
    const double omega_sq = kGravity *
                            (modal_root / params_.container_radius) *
                            std::tanh(frequency_argument);
    omega_n_ = std::sqrt(std::max(omega_sq, 0.0));

    const double modal_mass_numerator =
        2.0 * params_.container_radius * std::tanh(frequency_argument);
    const double modal_mass_denominator =
        modal_root * params_.liquid_height *
        (modal_root * modal_root - 1.0);
    const double modal_mass =
        std::abs(modal_mass_denominator) < 1e-9
            ? 0.0
            : liquid_mass * modal_mass_numerator / modal_mass_denominator;

    if (params_.use_linear_model) {
        height_coeff_ =
            (4.0 * params_.liquid_height * modal_mass) /
            (liquid_mass * params_.container_radius);
    } else {
        height_coeff_ =
            (modal_root * modal_root * params_.liquid_height * modal_mass) /
            (liquid_mass * params_.container_radius);
    }

    Eigen::Matrix4d continuous_a = Eigen::Matrix4d::Zero();
    const double two_zeta_omega =
        2.0 * params_.damping_ratio * omega_n_;
    const double natural_frequency_sq = omega_n_ * omega_n_;
    continuous_a(0, 1) = 1.0;
    continuous_a(1, 0) = -natural_frequency_sq;
    continuous_a(1, 1) = -two_zeta_omega;
    continuous_a(2, 3) = 1.0;
    continuous_a(3, 2) = -natural_frequency_sq;
    continuous_a(3, 3) = -two_zeta_omega;

    Eigen::Matrix<double, 4, 2> continuous_b =
        Eigen::Matrix<double, 4, 2>::Zero();
    continuous_b(1, 0) = -1.0;
    continuous_b(3, 1) = -1.0;

    Eigen::Matrix<double, 6, 6> augmented =
        Eigen::Matrix<double, 6, 6>::Zero();
    augmented.block<4, 4>(0, 0) = continuous_a * params_.dt;
    augmented.block<4, 2>(0, 4) = continuous_b * params_.dt;
    const Eigen::Matrix<double, 6, 6> discretized = augmented.exp();
    Ad_ = discretized.block<4, 4>(0, 0);
    Bd_ = discretized.block<4, 2>(0, 4);
    if (!Ad_.allFinite() || !Bd_.allFinite()) {
        Ad_.setIdentity();
        Bd_.setZero();
    }

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
        !std::isfinite(dt_sec) || dt_sec <= 1e-4 || !std::isfinite(omega_n_) ||
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
