#pragma once

#include "spmpc_local_planner/dynamics/slosh_types.h"

namespace spmpc_local_planner {

struct SloshModelParams {
    double container_radius = 0.01725;
    double liquid_height = 0.053;
    double liquid_density = 1000.0;
    double damping_ratio = 0.05;
    int mode_index = 1;
    double dt = 1.0 / 30.0;
    double slosh_height_ref = 0.005;
    double slosh_height_max = 0.001;
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

    // Held container excitation, shared RK4 RHS. Caller owns timestamps.
    // Always check the return value. On failure next_state equals state.
    // Changing actuator response belongs to actual_motion_propagator.h.
    bool stepWithDt(const SloshState& state,
                    const ContainerExcitation& excitation,
                    double dt_sec,
                    SloshState& next_state) const;
    static constexpr int modelVersion() { return 1; }
    double height(const SloshState& state, double omega_z = 0.0) const;
    double etaNorm(const SloshState& state) const;
    double etaDotNorm(const SloshState& state) const;

    double omegaN() const { return omega_n_; }
    double heightCoeff() const { return height_coeff_; }
    SloshCoefficients coefficients() const {
        return {2.0 * params_.damping_ratio * omega_n_, omega_n_ * omega_n_, 1.0, 1.0};
    }
    const SloshModelParams& params() const { return params_; }

private:
    SloshModelParams params_;
    double omega_n_ = 0.0;
    double height_coeff_ = 0.0;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
