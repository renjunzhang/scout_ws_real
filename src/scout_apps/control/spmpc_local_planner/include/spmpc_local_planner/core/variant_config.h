#pragma once

#include <string>

namespace spmpc_local_planner {

struct VariantConfig {
    std::string name = "B0";
    bool slosh_enable = false;
    bool smooth_priority_enable = false;
    bool slosh_constraint_enable = false;
    std::string primitive_mode = "linear";

    double w_contour = 1.0;
    double w_lag = 0.2;
    double w_progress = 0.2;
    double w_v = 1.0;
    double w_vs = 0.3;
    double v_ref = 0.25;
    double w_control = 0.1;
    // Supplemental non-slosh acceleration regularization baseline. Kept separate
    // from w_control so B_accel can penalize a without changing omega weight.
    double w_accel = 0.0;
    // Legacy smoothness knob. Split weights below default to this value when unset.
    double w_smooth = 0.1;
    double w_alpha = -1.0;
    double w_du_a = -1.0;
    double w_du_vs = -1.0;
    double w_slosh = 0.0;

    // Liquid-state cost is only trusted over a short prefix on hardware.
    // -1 preserves the historical full-horizon behavior.  A non-negative
    // value K applies the nominal liquid weights to state stages 0..K and
    // multiplies later stages by slosh_cost_tail_discount.
    int slosh_cost_horizon_steps = -1;
    double slosh_cost_tail_discount = 1.0;
};

VariantConfig makeVariantConfig(const std::string& variant_name);

// Runtime helper shared by the acados parameter update and diagnostics.  The
// stage argument is a state-cost stage (0..N), not a control index.
double sloshCostStageScale(const VariantConfig& config,
                           int stage,
                           int horizon_steps);

// The two development matched variants must remain identical except for the
// liquid weight and their names.  This helper makes the fairness contract
// directly unit-testable without a ROS parameter server.
bool matchedVariantCommonConfigEqual(const VariantConfig& lhs,
                                     const VariantConfig& rhs);

}  // namespace spmpc_local_planner
