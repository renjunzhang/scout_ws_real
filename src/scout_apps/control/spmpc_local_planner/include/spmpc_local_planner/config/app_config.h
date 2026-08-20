#pragma once

#include <string>
#include <vector>

namespace spmpc_local_planner {

enum class ValidationSeverity {
    Warning,
    Fatal,
};

struct ValidationIssue {
    ValidationSeverity severity = ValidationSeverity::Warning;
    std::string key;
    std::string message;
};

class ValidationReport {
public:
    void warning(const std::string& key, const std::string& message);
    void fatal(const std::string& key, const std::string& message);

    bool ok() const;
    const std::vector<ValidationIssue>& issues() const { return issues_; }

private:
    std::vector<ValidationIssue> issues_;
};

struct RuntimeVRefConfig {
    bool runtime_override_enable = false;
    double runtime_override_mps = -1.0;
    bool profile_enable = false;
    std::string profile_path;
    double profile_lookahead_m = 0.0;
};

// Typed root configuration. Sub-configs are added here as parameter groups are
// migrated out of the ROS adapter; runtime code receives only this value type.
struct AppConfig {
    RuntimeVRefConfig map_vref;
};

ValidationReport validateAndNormalize(AppConfig& config);

}  // namespace spmpc_local_planner
