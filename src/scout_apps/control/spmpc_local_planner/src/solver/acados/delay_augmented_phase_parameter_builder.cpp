#include "spmpc_local_planner/solver/acados/delay_augmented_phase_parameter_builder.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

bool lowercaseSha256(const std::string& value) {
    if (value.size() != 64) return false;
    return std::all_of(value.begin(), value.end(), [](char character) {
        return (character >= '0' && character <= '9') ||
            (character >= 'a' && character <= 'f');
    });
}

bool finiteWellConditioned(double value) {
    return std::isfinite(value) &&
        value >= manifest::kMinimumRecoveryDenominator;
}

bool finiteNonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool finiteInRange(double value, double lower, double upper) {
    return std::isfinite(value) && value >= lower && value <= upper;
}

bool serializeState(const PhaseNominalStage& stage,
                    std::array<double, manifest::kStateCount>& state) {
    const ExecutionAugmentedState& execution = stage.augmented_execution;
    if (!stage.valid || !stage.augmented_execution_valid ||
        !execution.valid || !std::isfinite(stage.s) || stage.s < 0.0 ||
        execution.linear.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kLinearBufferCount) ||
        execution.angular.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kAngularBufferCount) ||
        std::abs(execution.robot.v -
                 execution.linear.actuator_output) >
            manifest::kPublishedConsistencyTolerance ||
        std::abs(execution.robot.omega -
                 execution.angular.actuator_output) >
            manifest::kPublishedConsistencyTolerance ||
        !finiteInRange(execution.linear.actuator_output,
                       manifest::kLinearOutputMin,
                       manifest::kLinearOutputMax) ||
        !finiteInRange(execution.angular.actuator_output,
                       manifest::kAngularOutputMin,
                       manifest::kAngularOutputMax) ||
        !std::all_of(execution.linear.pending_commands.begin(),
                     execution.linear.pending_commands.end(),
                     [](double value) {
                         return finiteInRange(value,
                             manifest::kLinearOutputMin,
                             manifest::kLinearOutputMax);
                     }) ||
        !std::all_of(execution.angular.pending_commands.begin(),
                     execution.angular.pending_commands.end(),
                     [](double value) {
                         return finiteInRange(value,
                             manifest::kAngularOutputMin,
                             manifest::kAngularOutputMax);
                     })) {
        return false;
    }
    state = {{
        execution.robot.x,
        execution.robot.y,
        execution.robot.yaw,
        execution.linear.actuator_output,
        stage.s,
        execution.angular.actuator_output,
        execution.slosh.eta_x,
        execution.slosh.eta_x_dot,
        execution.slosh.eta_y,
        execution.slosh.eta_y_dot,
    }};
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kLinearBufferOffset + index)] =
                execution.linear.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kAngularBufferOffset + index)] =
                execution.angular.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    return std::all_of(state.begin(), state.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool validRadii(const EmpiricalRecoveryRadii& radii) {
    const double values[] = {
        radii.x, radii.y, radii.yaw, radii.v, radii.omega,
        radii.eta_x, radii.eta_x_dot, radii.eta_y, radii.eta_y_dot,
    };
    return std::all_of(
        std::begin(values), std::end(values), finiteWellConditioned);
}

bool validBounds(const PhaseNominalStage& stage) {
    const ExecutionCompatibilityBounds& bounds = stage.execution_bounds;
    return bounds.valid &&
        finiteWellConditioned(bounds.linear_actuator_output) &&
        finiteWellConditioned(bounds.angular_actuator_output) &&
        bounds.linear_pending_commands.size() ==
            static_cast<std::size_t>(manifest::kLinearBufferCount) &&
        bounds.angular_pending_commands.size() ==
            static_cast<std::size_t>(manifest::kAngularBufferCount) &&
        std::all_of(bounds.linear_pending_commands.begin(),
                    bounds.linear_pending_commands.end(),
                    finiteWellConditioned) &&
        std::all_of(bounds.angular_pending_commands.begin(),
                    bounds.angular_pending_commands.end(),
                    finiteWellConditioned);
}

bool validNominalControlAndPublished(const PhaseNominalStage& stage) {
    if (!finiteInRange(stage.a, -manifest::kAccelerationMax,
                       manifest::kAccelerationMax) ||
        !finiteInRange(stage.alpha, -manifest::kAngularAccelerationMax,
                       manifest::kAngularAccelerationMax) ||
        !finiteInRange(stage.v_s, 0.0, manifest::kProgressRateMax) ||
        !finiteInRange(stage.u_pub_v, manifest::kLinearOutputMin,
                       manifest::kLinearOutputMax) ||
        !finiteInRange(stage.u_pub_omega, manifest::kAngularOutputMin,
                       manifest::kAngularOutputMax)) {
        return false;
    }
    const auto& linear = stage.augmented_execution.linear.pending_commands;
    const auto& angular = stage.augmented_execution.angular.pending_commands;
    if (linear.empty() || angular.empty()) return false;
    const double expected_v = linear.back() + stage.a * manifest::kDt;
    const double expected_omega = angular.back() +
        stage.alpha * manifest::kDt;
    return std::abs(stage.u_pub_v - expected_v) <=
            manifest::kPublishedConsistencyTolerance &&
        std::abs(stage.u_pub_omega - expected_omega) <=
            manifest::kPublishedConsistencyTolerance;
}

bool validWeights(const DelayAugmentedPhaseCostWeights& weights) {
    const double values[] = {
        weights.position, weights.yaw, weights.progress, weights.v,
        weights.omega, weights.slosh_eta, weights.slosh_eta_dot,
        weights.linear_pending, weights.angular_pending,
        weights.acceleration, weights.angular_acceleration,
        weights.progress_rate,
    };
    return std::all_of(
        std::begin(values), std::end(values), finiteNonnegative);
}

void appendRadii(const EmpiricalRecoveryRadii& radii,
                 std::vector<double>& parameters) {
    const double values[] = {
        radii.x, radii.y, radii.yaw, radii.v, radii.omega,
        radii.eta_x, radii.eta_x_dot, radii.eta_y, radii.eta_y_dot,
    };
    std::copy(std::begin(values), std::end(values),
              parameters.begin() + manifest::kGateRadiusOffset);
}

void appendBounds(const ExecutionCompatibilityBounds& bounds,
                  std::vector<double>& parameters) {
    std::size_t offset = static_cast<std::size_t>(
        manifest::kExecutionBoundOffset);
    parameters[offset++] = bounds.linear_actuator_output;
    parameters[offset++] = bounds.angular_actuator_output;
    for (double value : bounds.linear_pending_commands) {
        parameters[offset++] = value;
    }
    for (double value : bounds.angular_pending_commands) {
        parameters[offset++] = value;
    }
}

void appendWeights(const DelayAugmentedPhaseCostWeights& weights,
                   std::vector<double>& parameters) {
    const double values[] = {
        weights.position, weights.yaw, weights.progress, weights.v,
        weights.omega, weights.slosh_eta, weights.slosh_eta_dot,
        weights.linear_pending, weights.angular_pending,
        weights.acceleration, weights.angular_acceleration,
        weights.progress_rate,
    };
    static_assert(
        sizeof(values) / sizeof(values[0]) == manifest::kWeightCount,
        "delay-augmented weight order drifted");
    std::copy(std::begin(values), std::end(values),
              parameters.begin() + manifest::kWeightOffset);
}

}  // namespace

bool DelayAugmentedPhaseParameterMatrix::hasCanonicalShape() const {
    if (!valid || stage_count != manifest::kHorizonSteps + 1 ||
        parameter_width != manifest::kParameterCount ||
        parameter_names.size() !=
            static_cast<std::size_t>(manifest::kParameterCount) ||
        values.size() != static_cast<std::size_t>(
            stage_count * parameter_width)) {
        return false;
    }
    for (int index = 0; index < manifest::kParameterCount; ++index) {
        if (parameter_names[static_cast<std::size_t>(index)] !=
            manifest::kParameterNames[index]) {
            return false;
        }
    }
    return true;
}

const double* DelayAugmentedPhaseParameterMatrix::stageData(int stage) const {
    if (!hasCanonicalShape() || stage < 0 || stage >= stage_count) {
        return nullptr;
    }
    return values.data() +
        static_cast<std::size_t>(stage * parameter_width);
}

double DelayAugmentedPhaseParameterMatrix::value(
    int stage, int parameter_index) const {
    const double* data = stageData(stage);
    if (data == nullptr || parameter_index < 0 ||
        parameter_index >= parameter_width) {
        return 0.0;
    }
    return data[parameter_index];
}

DelayAugmentedPhaseParameterMatrix
DelayAugmentedPhaseParameterBuilder::build(
    const DelayAugmentedPhaseSolverContext& context) {
    DelayAugmentedPhaseParameterMatrix output;
    if (!context.active ||
        context.parameter_schema_version !=
            manifest::kParameterSchemaVersion ||
        context.parameter_schema_id != manifest::kParameterSchemaId ||
        context.parameter_schema_hash != manifest::kParameterSchemaHash) {
        output.status = "PARAMETER_SCHEMA_MISMATCH";
        return output;
    }
    if (!lowercaseSha256(context.recovery_artifact_hash) ||
        context.execution_compatibility_contract !=
            manifest::kExecutionCompatibilityContract ||
        !context.terminal_empirical_gate_bound ||
        !context.execution_compatibility_bound) {
        output.status = "RECOVERY_ASSET_CONTRACT_MISSING";
        return output;
    }
    if (!finiteNonnegative(context.max_residual_v) ||
        context.max_residual_v >
            manifest::kLinearOutputMax - manifest::kLinearOutputMin ||
        !finiteNonnegative(context.max_residual_omega) ||
        context.max_residual_omega >
            manifest::kAngularOutputMax - manifest::kAngularOutputMin) {
        output.status = "INVALID_RESIDUAL_AUTHORITY";
        return output;
    }
    if (context.state_width != manifest::kStateCount ||
        context.control_width != manifest::kControlCount ||
        context.horizon_steps != manifest::kHorizonSteps ||
        context.stages.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1) ||
        context.current_index > context.terminal_index ||
        context.terminal_index - context.current_index !=
            static_cast<std::size_t>(manifest::kHorizonSteps) ||
        !validWeights(context.weights)) {
        output.status = "PARAMETER_IMAGE_SHAPE_MISMATCH";
        return output;
    }

    output.stage_count = manifest::kHorizonSteps + 1;
    output.parameter_width = manifest::kParameterCount;
    output.parameter_names.reserve(manifest::kParameterCount);
    for (int index = 0; index < manifest::kParameterCount; ++index) {
        output.parameter_names.emplace_back(manifest::kParameterNames[index]);
    }
    output.values.reserve(static_cast<std::size_t>(
        output.stage_count * output.parameter_width));

    for (int stage_index = 0;
         stage_index <= manifest::kHorizonSteps; ++stage_index) {
        const PhaseNominalStage& stage = context.stages[
            static_cast<std::size_t>(stage_index)];
        const bool terminal = stage_index == manifest::kHorizonSteps;
        if (stage.artifact_index != context.current_index +
                static_cast<std::size_t>(stage_index) ||
            stage.gate_active != terminal || !validRadii(stage.radii) ||
            !validBounds(stage)) {
            output.status = "INVALID_NOMINAL_STAGE";
            output.values.clear();
            return output;
        }
        std::array<double, manifest::kStateCount> state{};
        if (!serializeState(stage, state) ||
            !validNominalControlAndPublished(stage)) {
            output.status = "NONFINITE_NOMINAL_STAGE";
            output.values.clear();
            return output;
        }
        std::vector<double> parameters(
            static_cast<std::size_t>(manifest::kParameterCount), 0.0);
        std::copy(state.begin(), state.end(),
                  parameters.begin() + manifest::kNominalStateOffset);
        parameters[manifest::kNominalControlOffset] = stage.a;
        parameters[manifest::kNominalControlOffset + 1] = stage.alpha;
        parameters[manifest::kNominalControlOffset + 2] = stage.v_s;
        parameters[manifest::kNominalPublishOffset] = stage.u_pub_v;
        parameters[manifest::kNominalPublishOffset + 1] =
            stage.u_pub_omega;
        parameters[manifest::kResidualBoundOffset] =
            context.max_residual_v;
        parameters[manifest::kResidualBoundOffset + 1] =
            context.max_residual_omega;
        appendWeights(context.weights, parameters);
        appendRadii(stage.radii, parameters);
        appendBounds(stage.execution_bounds, parameters);
        output.values.insert(
            output.values.end(), parameters.begin(), parameters.end());
    }
    output.valid = true;
    output.status = "OK";
    return output;
}

}  // namespace spmpc_local_planner
