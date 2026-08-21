#include "spmpc_local_planner/phase_rejoin/execution_compatibility_gate.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finiteValue(double value) {
    return std::isfinite(value);
}

void accumulate(double actual,
                double nominal,
                double bound,
                double& maximum) {
    maximum = std::max(maximum, std::abs(actual - nominal) / bound);
}

}  // namespace

bool ExecutionCompatibilityGate::validBounds(
    const ExecutionCompatibilityBounds& bounds,
    const ExecutionAugmentedState& nominal) {
    if (!bounds.valid || !nominal.valid ||
        !finitePositive(bounds.linear_actuator_output) ||
        !finitePositive(bounds.angular_actuator_output) ||
        bounds.linear_pending_commands.size() !=
            nominal.linear.pending_commands.size() ||
        bounds.angular_pending_commands.size() !=
            nominal.angular.pending_commands.size()) {
        return false;
    }
    return std::all_of(
               bounds.linear_pending_commands.begin(),
               bounds.linear_pending_commands.end(), finitePositive) &&
        std::all_of(
               bounds.angular_pending_commands.begin(),
               bounds.angular_pending_commands.end(), finitePositive);
}

ExecutionCompatibilityGateResult ExecutionCompatibilityGate::evaluate(
    const ExecutionAugmentedState& nominal,
    const ExecutionCompatibilityBounds& bounds,
    const ExecutionAugmentedState& actual) const {
    ExecutionCompatibilityGateResult result;
    if (!validBounds(bounds, nominal)) {
        result.status = "INVALID_EXECUTION_BOUNDS";
        return result;
    }
    if (!actual.valid ||
        actual.linear.pending_commands.size() !=
            nominal.linear.pending_commands.size() ||
        actual.angular.pending_commands.size() !=
            nominal.angular.pending_commands.size() ||
        !finiteValue(actual.linear.actuator_output) ||
        !finiteValue(actual.angular.actuator_output)) {
        result.status = "INVALID_EXECUTION_STATE";
        return result;
    }

    accumulate(actual.linear.actuator_output,
               nominal.linear.actuator_output,
               bounds.linear_actuator_output,
               result.max_normalized_error);
    accumulate(actual.angular.actuator_output,
               nominal.angular.actuator_output,
               bounds.angular_actuator_output,
               result.max_normalized_error);
    for (std::size_t index = 0;
         index < actual.linear.pending_commands.size(); ++index) {
        const double value = actual.linear.pending_commands[index];
        if (!finiteValue(value)) {
            result.status = "NONFINITE_EXECUTION_STATE";
            return result;
        }
        accumulate(value,
                   nominal.linear.pending_commands[index],
                   bounds.linear_pending_commands[index],
                   result.max_normalized_error);
    }
    for (std::size_t index = 0;
         index < actual.angular.pending_commands.size(); ++index) {
        const double value = actual.angular.pending_commands[index];
        if (!finiteValue(value)) {
            result.status = "NONFINITE_EXECUTION_STATE";
            return result;
        }
        accumulate(value,
                   nominal.angular.pending_commands[index],
                   bounds.angular_pending_commands[index],
                   result.max_normalized_error);
    }
    result.valid = std::isfinite(result.max_normalized_error);
    result.accepted = result.valid &&
        result.max_normalized_error <= 1.0 + 1e-12;
    result.status = result.accepted ? "ACCEPTED" : "REJECTED";
    return result;
}

}  // namespace spmpc_local_planner
