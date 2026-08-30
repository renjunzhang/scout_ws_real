#include "spmpc_local_planner/core/speed_safety_contract.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool SpeedSafetyContract::configure(const SpeedSafetyParams& params,
                                    double platform_v_max,
                                    std::string* error) {
    configured_ = false;
    latched_ = false;
    params_ = params;
    platform_v_max_ = platform_v_max;
    effective_v_max_ = platform_v_max;

    auto reject = [&](const std::string& reason) {
        if (error) {
            *error = reason;
        }
        return false;
    };

    if (!std::isfinite(params_.v_safe_max) || params_.v_safe_max <= 0.0) {
        return reject("v_safe_max must be finite and > 0");
    }
    if (!std::isfinite(params_.tolerance) || params_.tolerance < 0.0) {
        return reject("tolerance must be finite and >= 0");
    }
    if (params_.enable) {
        if (!std::isfinite(platform_v_max_) || platform_v_max_ <= 0.0) {
            return reject("platform_v_max must be finite and > 0 when enabled");
        }
        if (params_.v_safe_max > platform_v_max_ + params_.tolerance) {
            return reject("v_safe_max must not exceed platform_v_max");
        }
        effective_v_max_ = std::min(platform_v_max_, params_.v_safe_max);
    }

    configured_ = true;
    if (error) {
        error->clear();
    }
    return true;
}

bool SpeedSafetyContract::exceedsLimit(double value) const {
    return !std::isfinite(value) ||
           std::abs(value) > params_.v_safe_max + params_.tolerance;
}

SpeedSafetyDecision SpeedSafetyContract::inspect(
    double solver_cmd_v,
    double post_gate_cmd_v,
    double publish_candidate_v) {
    SpeedSafetyDecision decision;
    decision.enabled = params_.enable;
    decision.v_safe_max = params_.v_safe_max;
    decision.tolerance = params_.tolerance;

    if (!configured_) {
        decision.enabled = true;
        decision.violation = true;
        decision.newly_latched = !latched_;
        latched_ = true;
        decision.latched = true;
        decision.status = "NOT_CONFIGURED";
        return decision;
    }
    if (!params_.enable) {
        decision.status = "DISABLED";
        return decision;
    }

    decision.solver_violation = exceedsLimit(solver_cmd_v);
    decision.post_gate_violation = exceedsLimit(post_gate_cmd_v);
    decision.publish_candidate_violation = exceedsLimit(publish_candidate_v);
    decision.violation = decision.solver_violation ||
                         decision.post_gate_violation ||
                         decision.publish_candidate_violation;
    decision.newly_latched = decision.violation && !latched_;
    latched_ = latched_ || decision.violation;
    decision.latched = latched_;

    if (decision.solver_violation) {
        decision.status = "SOLVER_COMMAND_LIMIT_EXCEEDED";
    } else if (decision.post_gate_violation) {
        decision.status = "POST_GATE_COMMAND_LIMIT_EXCEEDED";
    } else if (decision.publish_candidate_violation) {
        decision.status = "PUBLISH_CANDIDATE_LIMIT_EXCEEDED";
    } else if (decision.latched) {
        decision.status = "LATCHED";
    } else {
        decision.status = "PASS";
    }
    return decision;
}

void SpeedSafetyContract::reset() {
    latched_ = false;
}

}  // namespace spmpc_local_planner
