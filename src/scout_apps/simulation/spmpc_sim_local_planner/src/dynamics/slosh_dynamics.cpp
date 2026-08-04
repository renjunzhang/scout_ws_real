#include "spmpc_sim_local_planner/dynamics/slosh_dynamics.h"
#include <algorithm>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

namespace spmpc_sim_local_planner {

namespace {

// This is a local, frozen mathematical implementation for the simulation
// controller fork.  It intentionally does not link or include the physical
// stack's slosh_models package: changes in a real-robot model library cannot
// alter a released simulation controller binary.
constexpr double kGravity = 9.81;
constexpr double kModalRoots[] = {1.8412, 5.3314, 8.5363, 11.7060, 14.8636};
constexpr int kMaxModeIndex = sizeof(kModalRoots) / sizeof(kModalRoots[0]);

bool configureFrozenModalModel(const SloshModelParams& params,
                               Eigen::Matrix4d& ad,
                               Eigen::Matrix<double, 4, 2>& bd,
                               double& omega_n,
                               double& height_coeff) {
    if (!std::isfinite(params.container_radius) ||
        !std::isfinite(params.liquid_height) ||
        !std::isfinite(params.liquid_density) ||
        !std::isfinite(params.damping_ratio) ||
        !std::isfinite(params.dt) ||
        params.container_radius <= 0.0 || params.liquid_height <= 0.0 ||
        params.liquid_density <= 0.0 || params.dt <= 1e-4 ||
        params.mode_index < 1 || params.mode_index > kMaxModeIndex) {
        return false;
    }

    const double radius = params.container_radius;
    const double liquid_height = params.liquid_height;
    const double xi = kModalRoots[params.mode_index - 1];
    const double tanh_argument = xi * liquid_height / radius;
    const double tanh_value = std::tanh(tanh_argument);
    const double omega_squared = kGravity * (xi / radius) * tanh_value;
    if (!std::isfinite(omega_squared) || omega_squared <= 0.0) {
        return false;
    }
    omega_n = std::sqrt(omega_squared);

    const double liquid_mass = params.liquid_density * M_PI * radius * radius * liquid_height;
    const double mass_denominator = xi * liquid_height * (xi * xi - 1.0);
    if (!std::isfinite(liquid_mass) || !std::isfinite(mass_denominator) ||
        std::abs(mass_denominator) < 1e-9) {
        return false;
    }
    const double modal_mass = liquid_mass * (2.0 * radius * tanh_value) / mass_denominator;
    const double numerator = params.use_linear_model ? 4.0 : xi * xi;
    height_coeff = numerator * liquid_height * modal_mass / (liquid_mass * radius);
    if (!std::isfinite(omega_n) || !std::isfinite(height_coeff) ||
        height_coeff <= 0.0) {
        return false;
    }

    Eigen::Matrix4d continuous_a = Eigen::Matrix4d::Zero();
    const double damping = 2.0 * params.damping_ratio * omega_n;
    continuous_a(0, 1) = 1.0;
    continuous_a(1, 0) = -omega_squared;
    continuous_a(1, 1) = -damping;
    continuous_a(2, 3) = 1.0;
    continuous_a(3, 2) = -omega_squared;
    continuous_a(3, 3) = -damping;

    Eigen::Matrix<double, 4, 2> continuous_b =
        Eigen::Matrix<double, 4, 2>::Zero();
    continuous_b(1, 0) = -1.0;
    continuous_b(3, 1) = -1.0;
    Eigen::Matrix<double, 6, 6> augmented = Eigen::Matrix<double, 6, 6>::Zero();
    augmented.block<4, 4>(0, 0) = continuous_a * params.dt;
    augmented.block<4, 2>(0, 4) = continuous_b * params.dt;
    const Eigen::Matrix<double, 6, 6> discretized = augmented.exp();
    ad = discretized.block<4, 4>(0, 0);
    bd = discretized.block<4, 2>(0, 4);
    return ad.allFinite() && bd.allFinite();
}

}  // namespace

bool SloshDynamics::configure(const SloshModelParams& params) {
    params_ = params;
    Eigen::Matrix4d ad = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 2> bd = Eigen::Matrix<double, 4, 2>::Zero();
    double omega_n = 0.0;
    double height_coeff = 0.0;
    if (!configureFrozenModalModel(params, ad, bd, omega_n, height_coeff)) {
        configured_ = false;
        return false;
    }
    Ad_ = ad;
    Bd_ = bd;
    omega_n_ = omega_n;
    height_coeff_ = height_coeff;
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

}  // namespace spmpc_sim_local_planner
