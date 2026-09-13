#include "spmpc_local_planner/core/liquid_limit_policy.h"
#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
bool makeLiquidLimitPolicy(bool enabled, double target,
                           const LiquidLimitParams& params,
                           LiquidLimitPolicy& out, std::string& error) {
    out = {}; error.clear();
    if (!enabled) return true;
    if (!std::isfinite(target) || target <= 0 ||
        !std::isfinite(params.recovery_budget_m) || params.recovery_budget_m < 0 ||
        !std::isfinite(params.slack_linear_weight) || params.slack_linear_weight < 0 ||
        !std::isfinite(params.slack_quadratic_weight) || params.slack_quadratic_weight < 0 ||
        !std::isfinite(params.freeboard_m) || params.freeboard_m < 0 ||
        !std::isfinite(params.physical_margin_m) || params.physical_margin_m < 0 ||
        (params.recovery_enable && params.slack_linear_weight + params.slack_quadratic_weight <= 0)) {
        error = "INVALID_LIQUID_LIMIT_CONFIG"; return false;
    }
    out.enabled = true; out.recovery_enabled = params.recovery_enable;
    out.target_m = target;
    out.recovery_budget_m = params.recovery_enable ? params.recovery_budget_m : 0.0;
    out.cap_m = target + out.recovery_budget_m;
    out.linear_weight = params.recovery_enable ? params.slack_linear_weight : 0.0;
    out.quadratic_weight = params.recovery_enable ? params.slack_quadratic_weight : 0.0;
    if (params.freeboard_m > 0) {
        out.physical_boundary_known = true;
        out.physical_boundary_m = params.freeboard_m - params.physical_margin_m;
        if (out.physical_boundary_m <= 0 || target >= out.physical_boundary_m) {
            error = "LIQUID_TARGET_NOT_BELOW_PHYSICAL_BOUNDARY"; return false;
        }
        out.cap_m = std::min(out.cap_m, out.physical_boundary_m);
        out.recovery_budget_m = out.cap_m - target;
    } else if (params.physical_margin_m > 0) {
        error = "LIQUID_FREEBOARD_NOT_MEASURED"; return false;
    }
    if (!std::isfinite(out.cap_m)) {
        error = "INVALID_LIQUID_LIMIT_CONFIG"; return false;
    }
    return true;
}
}  // namespace spmpc_local_planner
