#include "spmpc_local_planner/phase_rejoin/execution_horizon_compatibility_gate.h"

#include "spmpc_local_planner/phase_rejoin/execution_compatibility_gate.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace spmpc_local_planner {
namespace {

struct Interval {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
};

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool intersect(Interval& interval, double lower, double upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
        return false;
    }
    interval.lower = std::max(interval.lower, lower);
    interval.upper = std::min(interval.upper, upper);
    return interval.lower <= interval.upper + 1.0e-12;
}

double clamp(double value, const Interval& interval) {
    return std::max(interval.lower, std::min(interval.upper, value));
}

void updateMaximum(ExecutionHorizonCompatibilityResult& result,
                   std::size_t stage,
                   const ExecutionCompatibilityGateResult& gate) {
    if (gate.max_normalized_error <= result.max_normalized_error) {
        return;
    }
    result.max_error_stage = stage;
    result.max_error_name = gate.max_error_name;
    result.max_error_index = gate.max_error_index;
    result.max_normalized_error = gate.max_normalized_error;
    result.actual = gate.actual;
    result.nominal = gate.nominal;
    result.bound = gate.bound;
}

struct ChannelView {
    const std::deque<double>* initial = nullptr;
    double output_min = 0.0;
    double output_max = 0.0;
    double residual = 0.0;
    double rate_step = 0.0;
    bool angular = false;
};

const std::deque<double>& nominalPending(
    const PhaseNominalSample& sample, bool angular) {
    return angular
        ? sample.augmented_execution.angular.pending_commands
        : sample.augmented_execution.linear.pending_commands;
}

const std::vector<double>& pendingBounds(
    const PhaseNominalSample& sample, bool angular) {
    return angular
        ? sample.execution_bounds.angular_pending_commands
        : sample.execution_bounds.linear_pending_commands;
}

double nominalPublished(const PhaseNominalSample& sample, bool angular) {
    return angular ? sample.u_pub_omega : sample.u_pub_v;
}

bool buildChannelWitness(
    const NominalSequenceArtifact& artifact,
    std::size_t current_index,
    int horizon_steps,
    const ChannelView& channel,
    std::vector<double>& commands,
    std::string& error) {
    commands.clear();
    if (channel.initial == nullptr || channel.initial->empty() ||
        !finitePositive(channel.residual) ||
        !finitePositive(channel.rate_step) ||
        !std::isfinite(channel.output_min) ||
        !std::isfinite(channel.output_max) ||
        channel.output_min >= channel.output_max || horizon_steps <= 0) {
        error = "INVALID_CHANNEL_CONTRACT";
        return false;
    }
    const std::size_t queue_size = channel.initial->size();
    std::vector<Interval> independent(
        static_cast<std::size_t>(horizon_steps));
    for (int stage = 0; stage < horizon_steps; ++stage) {
        const PhaseNominalSample* nominal = artifact.sample(
            current_index + static_cast<std::size_t>(stage));
        if (nominal == nullptr || !nominal->augmented_execution_valid ||
            !ExecutionCompatibilityGate::validBounds(
                nominal->execution_bounds,
                nominal->augmented_execution)) {
            error = "NOMINAL_HORIZON_INCOMPLETE";
            return false;
        }
        const double target = nominalPublished(*nominal, channel.angular);
        if (!intersect(
                independent[static_cast<std::size_t>(stage)],
                std::max(channel.output_min, target - channel.residual),
                std::min(channel.output_max, target + channel.residual))) {
            error = "PUBLISHED_COMMAND_INTERVAL_EMPTY";
            return false;
        }
    }

    // Every future queue coordinate is either an immutable entry from the
    // current pending queue or one of the commands published inside this
    // horizon.  Intersect all appearances of each future command with the
    // corresponding frozen B_exec interval.
    for (int stage = 1; stage <= horizon_steps; ++stage) {
        const PhaseNominalSample* nominal = artifact.sample(
            current_index + static_cast<std::size_t>(stage));
        if (nominal == nullptr || !nominal->augmented_execution_valid ||
            !ExecutionCompatibilityGate::validBounds(
                nominal->execution_bounds,
                nominal->augmented_execution)) {
            error = "NOMINAL_HORIZON_INCOMPLETE";
            return false;
        }
        const std::deque<double>& nominal_pending =
            nominalPending(*nominal, channel.angular);
        const std::vector<double>& bounds =
            pendingBounds(*nominal, channel.angular);
        if (nominal_pending.size() != queue_size ||
            bounds.size() != queue_size) {
            error = "PENDING_QUEUE_SHAPE_MISMATCH";
            return false;
        }
        for (std::size_t position = 0; position < queue_size; ++position) {
            const int command_index = stage -
                static_cast<int>(queue_size) +
                static_cast<int>(position);
            const double lower = nominal_pending[position] - bounds[position];
            const double upper = nominal_pending[position] + bounds[position];
            if (command_index < 0) {
                const std::size_t initial_index = static_cast<std::size_t>(
                    command_index + static_cast<int>(queue_size));
                const double fixed = (*channel.initial)[initial_index];
                if (fixed < lower - 1.0e-12 || fixed > upper + 1.0e-12) {
                    error = "IMMUTABLE_PENDING_QUEUE_OUTSIDE_B_EXEC";
                    return false;
                }
                continue;
            }
            if (command_index >= horizon_steps ||
                !intersect(
                    independent[static_cast<std::size_t>(command_index)],
                    lower, upper)) {
                error = "FUTURE_COMMAND_INTERVAL_EMPTY";
                return false;
            }
        }
    }

    // Difference constraints form a one-dimensional chain.  The reachable
    // set at each stage is therefore an interval; forward propagation plus a
    // backward choice yields a concrete command witness without invoking an
    // optimizer.
    std::vector<Interval> reachable = independent;
    double previous_lower = channel.initial->back();
    double previous_upper = channel.initial->back();
    for (int stage = 0; stage < horizon_steps; ++stage) {
        Interval& interval = reachable[static_cast<std::size_t>(stage)];
        if (!intersect(
                interval,
                previous_lower - channel.rate_step,
                previous_upper + channel.rate_step)) {
            error = "PUBLISHED_RATE_REACHABILITY_EMPTY";
            return false;
        }
        previous_lower = interval.lower;
        previous_upper = interval.upper;
    }

    commands.resize(static_cast<std::size_t>(horizon_steps));
    for (int stage = horizon_steps - 1; stage >= 0; --stage) {
        Interval admissible = reachable[static_cast<std::size_t>(stage)];
        if (stage + 1 < horizon_steps) {
            const double next = commands[static_cast<std::size_t>(stage + 1)];
            if (!intersect(
                    admissible,
                    next - channel.rate_step,
                    next + channel.rate_step)) {
                error = "PUBLISHED_RATE_BACKTRACK_FAILED";
                return false;
            }
        }
        const PhaseNominalSample* nominal = artifact.sample(
            current_index + static_cast<std::size_t>(stage));
        commands[static_cast<std::size_t>(stage)] = clamp(
            nominalPublished(*nominal, channel.angular), admissible);
    }
    if (std::abs(commands.front() - channel.initial->back()) >
            channel.rate_step + 1.0e-12) {
        error = "PUBLISHED_RATE_INITIAL_WITNESS_INVALID";
        return false;
    }
    return true;
}

}  // namespace

ExecutionHorizonCompatibilityResult
ExecutionHorizonCompatibilityGate::evaluate(
    const NominalSequenceArtifact& artifact,
    std::size_t current_index,
    const ExecutionHorizonContext& horizon,
    const ExecutionHorizonCompatibilityParams& params) const {
    ExecutionHorizonCompatibilityResult result;
    if (!artifact.valid() || !horizon.active ||
        !horizon.initial_state.valid || horizon.horizon_steps <= 0 ||
        horizon.contract.dt <= 0.0 ||
        !finitePositive(params.max_residual_v) ||
        !finitePositive(params.max_residual_omega) ||
        !finitePositive(params.max_published_acceleration) ||
        !finitePositive(params.max_published_angular_acceleration)) {
        result.status = "INVALID_HORIZON_FILTER_INPUT";
        return result;
    }
    if (artifact.sample(
            current_index + static_cast<std::size_t>(horizon.horizon_steps)) ==
            nullptr) {
        result.status = "NOMINAL_HORIZON_INCOMPLETE";
        return result;
    }

    ChannelView linear;
    linear.initial = &horizon.initial_state.linear.pending_commands;
    linear.output_min = horizon.contract.linear.output_min;
    linear.output_max = horizon.contract.linear.output_max;
    linear.residual = params.max_residual_v;
    linear.rate_step = params.max_published_acceleration * horizon.contract.dt;
    ChannelView angular;
    angular.initial = &horizon.initial_state.angular.pending_commands;
    angular.output_min = horizon.contract.angular.output_min;
    angular.output_max = horizon.contract.angular.output_max;
    angular.residual = params.max_residual_omega;
    angular.rate_step =
        params.max_published_angular_acceleration * horizon.contract.dt;
    angular.angular = true;

    std::string error;
    if (!buildChannelWitness(
            artifact, current_index, horizon.horizon_steps,
            linear, result.witness_linear_commands, error)) {
        result.valid = true;
        result.status = "LINEAR_" + error;
        return result;
    }
    if (!buildChannelWitness(
            artifact, current_index, horizon.horizon_steps,
            angular, result.witness_angular_commands, error)) {
        result.valid = true;
        result.status = "ANGULAR_" + error;
        return result;
    }

    ExecutionModel execution;
    if (!execution.configure(horizon.contract, params.slosh_model, error)) {
        result.status = "EXECUTION_MODEL_REJECTED_" + error;
        return result;
    }
    ExecutionCompatibilityGate gate;
    ExecutionAugmentedState state = horizon.initial_state;
    for (int stage = 0; stage <= horizon.horizon_steps; ++stage) {
        const PhaseNominalSample* nominal = artifact.sample(
            current_index + static_cast<std::size_t>(stage));
        const ExecutionCompatibilityGateResult evaluated = gate.evaluate(
            nominal->augmented_execution, nominal->execution_bounds, state);
        if (!evaluated.valid) {
            result.status = "EXECUTION_GATE_INVALID_" + evaluated.status;
            return result;
        }
        updateMaximum(result, static_cast<std::size_t>(stage), evaluated);
        if (!evaluated.accepted) {
            result.valid = true;
            result.status = "CAUSAL_WITNESS_OUTSIDE_B_EXEC";
            return result;
        }
        if (stage == horizon.horizon_steps) {
            break;
        }
        VelocityCommand published;
        published.linear = result.witness_linear_commands[
            static_cast<std::size_t>(stage)];
        published.angular = result.witness_angular_commands[
            static_cast<std::size_t>(stage)];
        const ExecutionStepResult step = execution.step(state, published);
        if (!step.valid) {
            result.status = "CAUSAL_WITNESS_ROLLOUT_FAILED_" + step.status;
            return result;
        }
        state = step.state;
    }

    result.valid = true;
    result.accepted = true;
    result.status = "ACCEPTED";
    return result;
}

}  // namespace spmpc_local_planner
