#include "support/delay_augmented_phase_snapshot_runner.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include "support/delay_augmented_phase_kkt_snapshot.h"

#include <array>
#include <cmath>

namespace spmpc_local_planner {
namespace test_support {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

using ClockT = std::int64_t;

ExecutionModelContract snapshotContract() {
    ExecutionModelContract contract;
    contract.schema_version = manifest::kExecutionContractSchemaVersion;
    contract.contract_id = manifest::kContractId;
    contract.contract_hash = manifest::kContractHash;
    contract.dt = manifest::kDt;
    contract.linear.delay_sec = manifest::kLinearDelaySec;
    contract.linear.integer_delay_steps =
        manifest::kLinearIntegerDelaySteps;
    contract.linear.fractional_delay_sec =
        manifest::kLinearFractionalDelaySec;
    contract.linear.time_constant_sec = manifest::kLinearTimeConstantSec;
    contract.linear.positive_gain = manifest::kLinearPositiveGain;
    contract.linear.negative_gain = manifest::kLinearNegativeGain;
    contract.linear.deadzone = manifest::kLinearDeadzone;
    contract.linear.output_min = manifest::kLinearOutputMin;
    contract.linear.output_max = manifest::kLinearOutputMax;
    contract.angular.delay_sec = manifest::kAngularDelaySec;
    contract.angular.integer_delay_steps =
        manifest::kAngularIntegerDelaySteps;
    contract.angular.fractional_delay_sec =
        manifest::kAngularFractionalDelaySec;
    contract.angular.time_constant_sec = manifest::kAngularTimeConstantSec;
    contract.angular.positive_gain = manifest::kAngularPositiveGain;
    contract.angular.negative_gain = manifest::kAngularNegativeGain;
    contract.angular.deadzone = manifest::kAngularDeadzone;
    contract.angular.output_min = manifest::kAngularOutputMin;
    contract.angular.output_max = manifest::kAngularOutputMax;
    return contract;
}

bool assignVector(const SnapshotJson& json, std::vector<double>& out,
                  std::string& error) {
    if (!json.numberArray(out)) {
        error = "expected number array";
        return false;
    }
    return true;
}

// Invert the codegen 22D layout back into the typed initial state.
// Layout (serializeInitialState):
//   [0]=x [1]=y [2]=yaw [3]=v [4]=s [5]=omega
//   [6]=eta_x [7]=eta_x_dot [8]=eta_y [9]=eta_y_dot
//   [10:10+linear_buffer_count]  = linear pending_commands
//   [15:15+angular_buffer_count] = angular pending_commands
bool reconstructInitialState(
    const std::vector<double>& state,
    ExecutionHorizonContext& context) {
    if (state.size() != static_cast<std::size_t>(manifest::kStateCount)) {
        return false;
    }
    context.initial_progress_s = state[4];

    ExecutionAugmentedState& outer = context.initial_state;
    outer.valid = true;
    outer.robot.x = state[0];
    outer.robot.y = state[1];
    outer.robot.yaw = state[2];
    outer.robot.v = state[3];
    outer.robot.omega = state[5];
    outer.slosh.eta_x = state[6];
    outer.slosh.eta_x_dot = state[7];
    outer.slosh.eta_y = state[8];
    outer.slosh.eta_y_dot = state[9];
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        outer.linear.pending_commands.push_back(state[
            static_cast<std::size_t>(manifest::kLinearBufferOffset + index)]);
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        outer.angular.pending_commands.push_back(state[
            static_cast<std::size_t>(manifest::kAngularBufferOffset + index)]);
    }
    outer.linear.actuator_output = outer.robot.v;
    outer.angular.actuator_output = outer.robot.omega;
    return true;
}

}  // namespace

bool loadSnapshot(const SnapshotJson& json,
                 DelayAugmentedPhaseSnapshot& out) {
    out = DelayAugmentedPhaseSnapshot();

    const SnapshotJson diagnostic = json.find("first_solver_failure_diagnostic");
    if (diagnostic.isNull()) {
        out.status = "missing first_solver_failure_diagnostic";
        return false;
    }

    // Reconstruct the parameter image first (it also yields nominal controls).
    std::vector<double> stage_parameters_flat;
    double parameter_width = 0;
    if (!diagnostic.find("stage_parameters").numberArray(
            stage_parameters_flat) ||
        !diagnostic.find("parameter_width").number(parameter_width)) {
        out.status = "stage_parameters/parameter_width missing or malformed";
        return false;
    }
    const int width = static_cast<int>(parameter_width);
    if (width != manifest::kParameterCount ||
        static_cast<int>(stage_parameters_flat.size()) !=
            width * (manifest::kHorizonSteps + 1)) {
        out.status = "parameter image shape mismatch";
        return false;
    }
    out.parameters.valid = true;
    out.parameters.status = "BUILT_FROM_SNAPSHOT";
    out.parameters.stage_count = manifest::kHorizonSteps + 1;
    out.parameters.parameter_width = width;
    out.parameters.parameter_names.assign(
        std::begin(manifest::kParameterNames),
        std::end(manifest::kParameterNames));
    out.parameters.values = std::move(stage_parameters_flat);

    // Nominal controls (a, alpha, v_s) per stage, read from the image's
    // nominal control entries (schema offset kNominalControlOffset).
    std::vector<DelayAugmentedPhaseControl> nominal_controls;
    nominal_controls.reserve(
        static_cast<std::size_t>(manifest::kHorizonSteps));
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        DelayAugmentedPhaseControl control;
        control.acceleration =
            out.parameters.value(stage, manifest::kNominalControlOffset + 0);
        control.angular_acceleration =
            out.parameters.value(stage, manifest::kNominalControlOffset + 1);
        control.progress_rate =
            out.parameters.value(stage, manifest::kNominalControlOffset + 2);
        nominal_controls.push_back(control);
    }

    // Initial state.
    std::vector<double> initial_state;
    if (!diagnostic.find("initial_state_22d").numberArray(initial_state) ||
        !reconstructInitialState(initial_state, out.context)) {
        out.status = "initial_state_22d missing or malformed";
        return false;
    }

    // Context contract + shape + epochs (all deterministic from manifest).
    out.context.active = true;
    out.context.contract = snapshotContract();
    out.context.execution_front_steps = manifest::kExecutionFrontSteps;
    out.context.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    out.context.horizon_steps = manifest::kHorizonSteps;
    const ClockT epoch = secondsToNanoseconds(10.0);
    out.context.initial_epoch_ns = epoch;
    out.context.physical_front_epoch_ns = addSeconds(
        epoch, std::max(manifest::kLinearDelaySec, manifest::kAngularDelaySec));
    out.context.grid_front_epoch_ns = addSeconds(
        epoch, manifest::kExecutionFrontSteps * manifest::kDt);
    out.context.terminal_epoch_ns = addSeconds(
        epoch, manifest::kHorizonSteps * manifest::kDt);
    out.context.initial_progress_s = initial_state[4];

    // Expected residuals + solver identity from the snapshot.
    const SnapshotJson residuals = diagnostic.find("solver_residuals");
    residuals.find("stationarity").number(out.expected_stationarity);
    residuals.find("equality").number(out.expected_equality);
    residuals.find("inequality").number(out.expected_inequality);
    residuals.find("complementarity").number(
        out.expected_complementarity);
    const SnapshotJson backend = diagnostic.find("solver_backend_contract");
    backend.find("solver_id").stringValue(out.solver_id);
    backend.find("solver_config_hash").stringValue(out.solver_config_hash);

    // Retain nominal controls for the warm-start replay.
    out.nominal_controls = std::move(nominal_controls);

    out.valid = true;
    out.status = "OK";
    return true;
}

}  // namespace test_support
}  // namespace spmpc_local_planner
