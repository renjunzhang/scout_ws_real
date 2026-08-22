#pragma once

#include <string>
#include <vector>

namespace spmpc_local_planner {

struct DelayAugmentedPhaseResidualDiagnostics {
    bool evaluated = false;
    double stationarity = 0.0;
    double equality = 0.0;
    double inequality = 0.0;
    double complementarity = 0.0;
};

struct DelayAugmentedPhaseIterationDiagnostics {
    int iteration = -1;
    double stationarity = 0.0;
    double equality = 0.0;
    double inequality = 0.0;
    double complementarity = 0.0;
    int qp_status = -1;
    int qp_iterations = -1;
    double step_length = 0.0;
};

struct DelayAugmentedPhaseNamedConstraintDiagnostics {
    int stage = -1;
    int index = -1;
    std::string name;
    double value = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    double normalized_error = 0.0;
    double violation = 0.0;
};

struct DelayAugmentedPhaseConstraintAudit {
    bool evaluated = false;
    bool passed = false;
    std::string status = "NOT_EVALUATED";
    double tolerance = 0.0;
    std::vector<DelayAugmentedPhaseNamedConstraintDiagnostics>
        stage_constraints;
    std::vector<DelayAugmentedPhaseNamedConstraintDiagnostics>
        control_constraints;
    double terminal_empirical_metric = 0.0;
    double terminal_empirical_violation = 0.0;
    std::vector<DelayAugmentedPhaseNamedConstraintDiagnostics>
        terminal_execution_constraints;
    double max_causal_state_error = 0.0;
    int max_causal_state_error_stage = -1;
    int max_causal_state_error_index = -1;
    int max_violation_stage = -1;
    int max_violation_index = -1;
    std::string max_violation_name = "NONE";
    double max_violation_value = 0.0;
    double max_violation = 0.0;
};

}  // namespace spmpc_local_planner
