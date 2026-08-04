#pragma once

#include "spmpc_sim_local_planner/dynamics/slosh_dynamics.h"
#include <string>

namespace spmpc_sim_local_planner {

struct SloshRiskGovernorParams {
    bool enable = false;
    bool require_slosh_variant = true;
    int horizon_steps = 30;
    double height_limit_m = 0.006;
    double risk_threshold = 1.0;
    double release_threshold = 0.75;
    double beta_min = 0.45;
    int beta_grid_count = 12;
    double min_v_ref = 0.05;
    double accel_limit = 0.4;
    double omega_decay_tau = 0.5;
    double beta_rate_up_per_sec = 0.5;
    double beta_rate_down_per_sec = 2.0;
    bool include_parabola_height = true;
};

struct SloshRiskGovernorInput {
    SloshState slosh;
    double robot_v = 0.0;
    double robot_omega = 0.0;
    double nominal_v_ref = 0.0;
    double dt = 1.0 / 30.0;
    bool slosh_variant_enabled = false;
};

struct SloshRiskGovernorOutput {
    bool enabled = false;
    bool active = false;
    bool feasible_found = false;
    bool saturated = false;
    bool predicted_risk_admissible = false;
    double nominal_v_ref = 0.0;
    double governed_v_ref = 0.0;
    double beta_raw = 1.0;
    double beta_filtered = 1.0;
    double risk_now = 0.0;
    double risk_peak = 0.0;
    double risk_margin = 0.0;
    double h_now_m = 0.0;
    double h_peak_m = 0.0;
    double computation_time_ms = 0.0;
    int selected_candidate_index = 0;
    std::string status = "DISABLED";
};

// Finite-horizon predictive reference adaptation:
//   beta_star = max beta in B
//               s.t. max_k H_k(beta) / H_lim <= risk_threshold.
// B is the discrete grid from 1.0 down to beta_min. The raw candidate is the
// largest grid point satisfying the predicted risk condition. The final
// rate-limited beta is rolled out again, so predicted_risk_admissible and
// risk_margin describe the reference actually sent to MPCC.
class SloshRiskGovernor {
public:
    bool configure(const SloshModelParams& slosh_params, const SloshRiskGovernorParams& params);
    void reset();
    SloshRiskGovernorOutput update(const SloshRiskGovernorInput& input);

private:
    struct RolloutResult {
        double risk_peak = 0.0;
        double h_peak_m = 0.0;
    };

    SloshRiskGovernorOutput passThrough(const SloshRiskGovernorInput& input,
                                        const std::string& status) const;
    double height(const SloshState& state, double omega_z) const;
    RolloutResult rollout(const SloshRiskGovernorInput& input, double beta) const;

    SloshRiskGovernorParams params_;
    SloshModelParams slosh_params_;
    SloshDynamics slosh_dyn_;
    bool configured_ = false;
    bool have_beta_filtered_ = false;
    double beta_filtered_ = 1.0;
};

}  // namespace spmpc_sim_local_planner
