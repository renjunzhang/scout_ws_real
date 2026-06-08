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
    double w_control = 0.1;
    // Supplemental non-slosh acceleration regularization baseline. Kept separate
    // from w_control so B_accel can penalize a without changing omega weight.
    double w_accel = 0.0;
    double w_smooth = 0.1;
    double w_slosh = 0.0;
};

VariantConfig makeVariantConfig(const std::string& variant_name);

}  // namespace spmpc_local_planner
