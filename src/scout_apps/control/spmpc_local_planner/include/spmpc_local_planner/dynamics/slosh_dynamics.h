#pragma once

#include "spmpc_local_planner/core/types.h"
#include <Eigen/Dense>

namespace spmpc_local_planner {

struct SloshModelParams {
    double container_radius = 0.0185;
    double liquid_height = 0.058;
    double liquid_density = 1000.0;
    double damping_ratio = 0.05;
    int mode_index = 1;
    double dt = 1.0 / 30.0;
    double slosh_height_ref = 0.005;
    double slosh_height_max = 0.008;
    double slosh_eta_dot_ratio = 0.3;
    bool use_linear_model = true;
    // Default to modal-only height. The yaw-induced parabola correction is kept as
    // an opt-in visualization proxy but is not part of the solver hard constraint.
    bool use_parabola_term = false;
};

class SloshDynamics {
public:
    bool configure(const SloshModelParams& params);
    bool configured() const { return configured_; }

    SloshState step(const SloshState& state, double ax, double ay, double omega_z) const;
    double height(const SloshState& state, double omega_z = 0.0) const;
    double etaNorm(const SloshState& state) const;
    double etaDotNorm(const SloshState& state) const;

    double omegaN() const { return omega_n_; }
    double heightCoeff() const { return height_coeff_; }
    const SloshModelParams& params() const { return params_; }

private:
    Eigen::Vector4d toEigen(const SloshState& state) const;
    SloshState fromEigen(const Eigen::Vector4d& state) const;

    SloshModelParams params_;
    Eigen::Matrix4d Ad_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 2> Bd_ = Eigen::Matrix<double, 4, 2>::Zero();
    double omega_n_ = 0.0;
    double height_coeff_ = 0.0;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
