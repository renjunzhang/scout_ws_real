#pragma once
#include <string>

namespace spmpc_local_planner {

struct LiquidLimitParams {
    bool recovery_enable = false;
    double recovery_budget_m = 0.003;
    double slack_linear_weight = 10.0;
    double slack_quadratic_weight = 100.0;
    // Zero means unmeasured: never label the performance cap as a physical boundary.
    double freeboard_m = 0.0;
    double physical_margin_m = 0.0;
};

struct LiquidLimitPolicy {
    bool enabled = false;
    bool recovery_enabled = false;
    bool physical_boundary_known = false;
    double target_m = 0.0;
    double recovery_budget_m = 0.0;
    double cap_m = 0.0;
    double physical_boundary_m = 0.0;
    double linear_weight = 0.0;
    double quadratic_weight = 0.0;
};

bool makeLiquidLimitPolicy(bool enabled, double target_m,
                           const LiquidLimitParams& params,
                           LiquidLimitPolicy& policy, std::string& error);

}  // namespace spmpc_local_planner
