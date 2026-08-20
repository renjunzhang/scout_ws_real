#include "spmpc_local_planner/config/app_config.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

void ValidationReport::warning(const std::string& key,
                               const std::string& message) {
    issues_.push_back({ValidationSeverity::Warning, key, message});
}

void ValidationReport::fatal(const std::string& key,
                             const std::string& message) {
    issues_.push_back({ValidationSeverity::Fatal, key, message});
}

bool ValidationReport::ok() const {
    return std::none_of(
        issues_.begin(), issues_.end(), [](const ValidationIssue& issue) {
            return issue.severity == ValidationSeverity::Fatal;
        });
}

ValidationReport validateAndNormalize(AppConfig& config) {
    ValidationReport report;
    auto& vref = config.map_vref;

    if (vref.runtime_override_enable &&
        (!std::isfinite(vref.runtime_override_mps) ||
         vref.runtime_override_mps < 0.0)) {
        report.warning(
            "map_vref/runtime_v_ref",
            "invalid enabled override; disabling it to preserve variant fallback");
        vref.runtime_override_enable = false;
    }
    if (!std::isfinite(vref.profile_lookahead_m)) {
        report.warning(
            "map_vref/profile_lookahead_s",
            "non-finite lookahead normalized to zero");
        vref.profile_lookahead_m = 0.0;
    } else if (vref.profile_lookahead_m < 0.0) {
        report.warning(
            "map_vref/profile_lookahead_s",
            "negative lookahead normalized to zero");
        vref.profile_lookahead_m = 0.0;
    }
    if (vref.profile_enable && vref.profile_path.empty()) {
        report.warning(
            "map_vref/profile_path",
            "profile is enabled without a path; cycles will report PROFILE_NOT_CONFIGURED");
    }
    return report;
}

}  // namespace spmpc_local_planner
