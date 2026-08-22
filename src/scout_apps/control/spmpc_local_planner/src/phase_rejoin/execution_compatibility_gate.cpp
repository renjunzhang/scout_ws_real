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
                const char* name,
                int index,
                ExecutionCompatibilityGateResult& result) {
    const double normalized = std::abs(actual - nominal) / bound;
    if (normalized > result.max_normalized_error) {
        result.max_normalized_error = normalized;
        result.max_error_name = name;
        result.max_error_index = index;
        result.actual = actual;
        result.nominal = nominal;
        result.bound = bound;
    }
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
               "linear_actuator_output", -1, result);
    accumulate(actual.angular.actuator_output,
               nominal.angular.actuator_output,
               bounds.angular_actuator_output,
               "angular_actuator_output", -1, result);
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
                   "linear_pending_command", static_cast<int>(index),
                   result);
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
                   "angular_pending_command", static_cast<int>(index),
                   result);
    }
    result.valid = std::isfinite(result.max_normalized_error);
    result.accepted = result.valid &&
        result.max_normalized_error <= 1.0 + 1e-12;
    result.status = result.accepted ? "ACCEPTED" : "REJECTED";
    return result;
}

}  // namespace spmpc_local_planner
