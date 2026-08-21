#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/nominal_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <array>
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <openssl/sha.h>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace spmpc_local_planner {
namespace {

namespace augmented_manifest = delay_augmented_phase_solver_manifest;

const char* const kSchemaV1 = "phase_rejoin_empirical_v1";
const char* const kSchemaV2 = "phase_rejoin_empirical_v2";
const char* const kSchemaV3 = "phase_rejoin_empirical_augmented_v3";
const char* const kTerminalContractV1 = "stop_settle_zero_hold_v1";
const char* const kTerminalContractV2 = "publish_zero_settle_hold_v2";
const char* const kRecoveryContractV1 = "nominal_command_v1";

const std::map<std::string, std::string>& developmentProxyMetadata() {
    static const std::map<std::string, std::string> expected = {
        {"schema", kSchemaV1},
        {"evidence_level", "development_only"},
        {"source", "development_proxy_replay"},
        {"artifact_role", "interface_smoke_only"},
        {"nominal_sequence_kind",
         "rolling_local_planner_first_stage_proxy"},
        {"offline_slosh_ocp", "false"},
        {"hardware_formal_release", "false"},
        {"paper_main_result_eligible", "false"},
        {"gate_parameter_source",
         "operator_supplied_per_cycle_development_csv"},
        {"recovery_policy_source",
         "operator_supplied_per_cycle_development_csv"},
        {"gate_evidence", "none_development_input_only"},
        {"recovery_policy_evidence", "none_development_input_only"},
        {"row_state_semantics",
         "predicted_horizon_stage0_at_solver_input_epoch"},
        {"row_command_semantics", "same_cycle_final_published_command"},
    };
    return expected;
}

const std::map<std::string, std::string>& developmentNominalMetadata() {
    static const std::map<std::string, std::string> expected = {
        {"schema", kSchemaV2},
        {"evidence_level", "development_only"},
        {"source", "development_dynamics_consistent_nominal"},
        {"artifact_role", "interface_smoke_only"},
        {"nominal_sequence_kind",
         "uniform_dynamics_consistent_complete_tail"},
        {"offline_slosh_ocp", "false"},
        {"hardware_formal_release", "false"},
        {"paper_main_result_eligible", "false"},
        {"gate_evidence", "none_operator_supplied_development_radii"},
        {"recovery_policy_source",
         "nominal_next_command_development_fallback"},
        {"recovery_policy_evidence", "none_development_only"},
    };
    return expected;
}

const std::vector<std::string>& baseHeader() {
    static const std::vector<std::string> header = {
        "index", "t", "s", "x", "y", "yaw", "v", "omega",
        "eta_x", "eta_x_dot", "eta_y", "eta_y_dot",
        "a", "alpha", "v_s", "u_pub_v", "u_pub_omega",
        "kappa_v", "kappa_omega",
        "r_x", "r_y", "r_yaw", "r_v", "r_omega",
        "r_eta_x", "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
    };
    return header;
}

const std::vector<std::string>& augmentedHeader() {
    static const std::vector<std::string> header = [] {
        std::vector<std::string> values = baseHeader();
        const char* extra[] = {
            "exec_linear_output", "exec_angular_output",
            "exec_linear_pending", "exec_angular_pending",
            "exec_beta_linear_output", "exec_beta_angular_output",
            "exec_beta_linear_pending", "exec_beta_angular_pending",
        };
        values.insert(values.end(), std::begin(extra), std::end(extra));
        return values;
    }();
    return header;
}

const std::vector<std::string>& expectedHeader(bool augmented) {
    return augmented ? augmentedHeader() : baseHeader();
}

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return std::string();
    }
    const std::size_t end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) {
        values.push_back(trim(value));
    }
    if (!line.empty() && line.back() == ',') {
        values.emplace_back();
    }
    return values;
}

bool parseDouble(const std::string& text, double& value) {
    const std::string input = trim(text);
    if (input.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    value = std::strtod(input.c_str(), &end);
    return errno == 0 && end != input.c_str() && *end == '\0' &&
           std::isfinite(value);
}

bool parseDoubleList(const std::string& text,
                     std::size_t expected_count,
                     std::vector<double>& values) {
    values.clear();
    std::stringstream stream(trim(text));
    std::string item;
    while (std::getline(stream, item, ';')) {
        double value = 0.0;
        if (!parseDouble(item, value)) {
            values.clear();
            return false;
        }
        values.push_back(value);
    }
    return values.size() == expected_count;
}

std::string doubleList(const std::deque<double>& values) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ';';
        out << values[index];
    }
    return out.str();
}

std::string doubleList(const std::vector<double>& values) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17);
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ';';
        out << values[index];
    }
    return out.str();
}

bool parseIndex(const std::string& text, std::size_t& value) {
    const std::string input = trim(text);
    if (input.empty() || input[0] == '-') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(input.c_str(), &end, 10);
    if (errno != 0 || end == input.c_str() || *end != '\0' ||
        parsed > static_cast<unsigned long long>(
                     std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
}

bool parsePositiveIndex(const std::string& text, std::size_t& value) {
    return parseIndex(text, value) && value > 0;
}

bool isLowercaseSha256(const std::string& value) {
    if (value.size() != 64) {
        return false;
    }
    for (char character : value) {
        if (!((character >= '0' && character <= '9') ||
              (character >= 'a' && character <= 'f'))) {
            return false;
        }
    }
    return true;
}

bool positiveRadii(const EmpiricalRecoveryRadii& radii) {
    return radii.x > 0.0 && radii.y > 0.0 && radii.yaw > 0.0 &&
           radii.v > 0.0 && radii.omega > 0.0 && radii.eta_x > 0.0 &&
           radii.eta_x_dot > 0.0 && radii.eta_y > 0.0 &&
           radii.eta_y_dot > 0.0;
}

bool wellConditionedV3Radii(const EmpiricalRecoveryRadii& radii) {
    const double values[] = {
        radii.x, radii.y, radii.yaw, radii.v, radii.omega,
        radii.eta_x, radii.eta_x_dot, radii.eta_y, radii.eta_y_dot,
    };
    return std::all_of(std::begin(values), std::end(values),
        [](double value) {
            return std::isfinite(value) &&
                value >= augmented_manifest::kMinimumRecoveryDenominator;
        });
}

NominalArtifactLoadResult failure(const std::string& status,
                                  const std::string& detail) {
    NominalArtifactLoadResult result;
    result.status = status;
    result.detail = detail;
    return result;
}

double angleError(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

bool within(double lhs, double rhs, double tolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

NominalArtifactLoadResult validateV2Transitions(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const PhaseNominalSample& current = samples[i];
        const PhaseNominalSample& next = samples[i + 1];
        const NominalDynamicsState predicted =
            phaseNominalRk4Step(current, metadata);
        const bool state_consistent =
            within(predicted.x, next.x, tolerance) &&
            within(predicted.y, next.y, tolerance) &&
            std::abs(angleError(predicted.yaw, next.yaw)) <= tolerance &&
            within(predicted.v, next.v, tolerance) &&
            within(predicted.omega, next.omega, tolerance) &&
            within(predicted.s, next.s, tolerance) &&
            within(predicted.eta_x, next.eta_x, tolerance) &&
            within(predicted.eta_x_dot, next.eta_x_dot, tolerance) &&
            within(predicted.eta_y, next.eta_y, tolerance) &&
            within(predicted.eta_y_dot, next.eta_y_dot, tolerance);
        if (!state_consistent) {
            return failure("DYNAMICS_TRANSITION_MISMATCH",
                           "index " + std::to_string(i));
        }
        if (!within(current.u_pub_v, predicted.v, tolerance) ||
            !within(current.u_pub_omega, predicted.omega, tolerance)) {
            return failure("PUBLISHED_COMMAND_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

ExecutionModelContract compiledV3ExecutionContract() {
    ExecutionModelContract contract;
    contract.schema_version =
        augmented_manifest::kExecutionContractSchemaVersion;
    contract.contract_id = augmented_manifest::kContractId;
    contract.contract_hash = augmented_manifest::kContractHash;
    contract.dt = augmented_manifest::kDt;
    contract.linear.delay_sec = augmented_manifest::kLinearDelaySec;
    contract.linear.time_constant_sec =
        augmented_manifest::kLinearTimeConstantSec;
    contract.linear.positive_gain =
        augmented_manifest::kLinearPositiveGain;
    contract.linear.negative_gain =
        augmented_manifest::kLinearNegativeGain;
    contract.linear.deadzone = augmented_manifest::kLinearDeadzone;
    contract.linear.output_min = augmented_manifest::kLinearOutputMin;
    contract.linear.output_max = augmented_manifest::kLinearOutputMax;
    contract.angular.delay_sec = augmented_manifest::kAngularDelaySec;
    contract.angular.time_constant_sec =
        augmented_manifest::kAngularTimeConstantSec;
    contract.angular.positive_gain =
        augmented_manifest::kAngularPositiveGain;
    contract.angular.negative_gain =
        augmented_manifest::kAngularNegativeGain;
    contract.angular.deadzone = augmented_manifest::kAngularDeadzone;
    contract.angular.output_min = augmented_manifest::kAngularOutputMin;
    contract.angular.output_max = augmented_manifest::kAngularOutputMax;
    return contract;
}

SloshModelParams compiledV3SloshContract() {
    SloshModelParams params;
    params.container_radius = augmented_manifest::kContainerRadius;
    params.liquid_height = augmented_manifest::kLiquidHeight;
    params.liquid_density = augmented_manifest::kLiquidDensity;
    params.damping_ratio = augmented_manifest::kDampingRatio;
    params.mode_index = augmented_manifest::kModeIndex;
    params.dt = augmented_manifest::kDt;
    params.slosh_height_ref = augmented_manifest::kSloshHeightRef;
    params.slosh_eta_dot_ratio =
        augmented_manifest::kSloshEtaDotRatio;
    params.use_linear_model = true;
    params.use_parabola_term = false;
    return params;
}

bool samePendingCommands(const std::deque<double>& predicted,
                         const std::deque<double>& actual,
                         double tolerance) {
    if (predicted.size() != actual.size()) {
        return false;
    }
    for (std::size_t index = 0; index < predicted.size(); ++index) {
        if (!within(predicted[index], actual[index], tolerance)) {
            return false;
        }
    }
    return true;
}

NominalArtifactLoadResult validateV3Transitions(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    static_assert(
        augmented_manifest::kStateCount == 10 +
            augmented_manifest::kLinearBufferCount +
            augmented_manifest::kAngularBufferCount,
        "generated augmented state layout is inconsistent");
    const bool compiled_contract_matches =
        metadata.execution_contract_id == augmented_manifest::kContractId &&
        metadata.execution_contract_hash ==
            augmented_manifest::kContractHash &&
        metadata.execution_state_width == augmented_manifest::kStateCount &&
        metadata.linear_buffer_count ==
            augmented_manifest::kLinearBufferCount &&
        metadata.angular_buffer_count ==
            augmented_manifest::kAngularBufferCount &&
        metadata.dt == augmented_manifest::kDt;
    if (!compiled_contract_matches) {
        return failure("V3_EXECUTION_CONTRACT_MISMATCH",
                       metadata.execution_contract_id);
    }

    // V3 is an executable parameter image, not merely a dynamically
    // self-consistent trajectory.  A sequence can satisfy its own transition
    // equations with an acceleration that the generated OCP can never apply.
    // Reject that asset at load time so it cannot fail only after entering the
    // online solver (and then fall through to a recovery command derived from
    // the same invalid row).
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const PhaseNominalSample& sample = samples[index];
        const auto within_range = [](double value,
                                     double lower,
                                     double upper) {
            return std::isfinite(value) && value >= lower && value <= upper;
        };
        const bool control_within_generated_bounds =
            std::isfinite(sample.a) &&
            sample.a >= -augmented_manifest::kAccelerationMax &&
            sample.a <= augmented_manifest::kAccelerationMax &&
            std::isfinite(sample.alpha) &&
            sample.alpha >=
                -augmented_manifest::kAngularAccelerationMax &&
            sample.alpha <=
                augmented_manifest::kAngularAccelerationMax &&
            std::isfinite(sample.v_s) && sample.v_s >= 0.0 &&
            sample.v_s <= augmented_manifest::kProgressRateMax &&
            std::isfinite(sample.u_pub_v) &&
            sample.u_pub_v >= augmented_manifest::kLinearOutputMin &&
            sample.u_pub_v <= augmented_manifest::kLinearOutputMax &&
            std::isfinite(sample.u_pub_omega) &&
            sample.u_pub_omega >= augmented_manifest::kAngularOutputMin &&
            sample.u_pub_omega <= augmented_manifest::kAngularOutputMax;
        if (!control_within_generated_bounds) {
            return failure("V3_NOMINAL_CONTROL_BOUNDS_MISMATCH",
                           "index " + std::to_string(index));
        }
        if (!std::isfinite(sample.s) || sample.s < 0.0) {
            return failure("V3_NOMINAL_PROGRESS_BOUNDS_MISMATCH",
                           "index " + std::to_string(index));
        }

        const ExecutionAugmentedState& execution =
            sample.augmented_execution;
        const bool execution_within_generated_bounds =
            execution.valid &&
            within_range(execution.linear.actuator_output,
                         augmented_manifest::kLinearOutputMin,
                         augmented_manifest::kLinearOutputMax) &&
            within_range(execution.angular.actuator_output,
                         augmented_manifest::kAngularOutputMin,
                         augmented_manifest::kAngularOutputMax) &&
            execution.linear.pending_commands.size() ==
                static_cast<std::size_t>(
                    augmented_manifest::kLinearBufferCount) &&
            execution.angular.pending_commands.size() ==
                static_cast<std::size_t>(
                    augmented_manifest::kAngularBufferCount) &&
            std::all_of(
                execution.linear.pending_commands.begin(),
                execution.linear.pending_commands.end(),
                [&within_range](double value) {
                    return within_range(
                        value,
                        augmented_manifest::kLinearOutputMin,
                        augmented_manifest::kLinearOutputMax);
                }) &&
            std::all_of(
                execution.angular.pending_commands.begin(),
                execution.angular.pending_commands.end(),
                [&within_range](double value) {
                    return within_range(
                        value,
                        augmented_manifest::kAngularOutputMin,
                        augmented_manifest::kAngularOutputMax);
                });
        if (!execution_within_generated_bounds) {
            return failure("V3_EXECUTION_STATE_BOUNDS_MISMATCH",
                           "index " + std::to_string(index));
        }

        const double expected_published_v =
            execution.linear.pending_commands.back() +
            sample.a * augmented_manifest::kDt;
        const double expected_published_omega =
            execution.angular.pending_commands.back() +
            sample.alpha * augmented_manifest::kDt;
        if (!within(sample.u_pub_v, expected_published_v,
                    augmented_manifest::kPublishedConsistencyTolerance) ||
            !within(sample.u_pub_omega, expected_published_omega,
                    augmented_manifest::kPublishedConsistencyTolerance)) {
            return failure("PUBLISHED_COMMAND_MISMATCH",
                           "index " + std::to_string(index));
        }
    }

    ExecutionModel execution_model;
    std::string configuration_error;
    if (!execution_model.configure(
            compiledV3ExecutionContract(), compiledV3SloshContract(),
            configuration_error)) {
        return failure("V3_EXECUTION_MODEL_UNAVAILABLE",
                       configuration_error);
    }

    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t index = 0; index + 1 < samples.size(); ++index) {
        const PhaseNominalSample& current = samples[index];
        const PhaseNominalSample& next = samples[index + 1];
        const double published_v =
            current.augmented_execution.linear.pending_commands.back() +
            current.a * augmented_manifest::kDt;
        const double published_omega =
            current.augmented_execution.angular.pending_commands.back() +
            current.alpha * augmented_manifest::kDt;

        VelocityCommand published_command;
        published_command.linear = published_v;
        published_command.angular = published_omega;
        const ExecutionStepResult transition = execution_model.step(
            current.augmented_execution, published_command);
        const ExecutionAugmentedState& predicted = transition.state;
        const double predicted_progress =
            current.s + current.v_s * metadata.dt;
        const bool state_consistent = transition.valid &&
            within(predicted.robot.x, next.x, tolerance) &&
            within(predicted.robot.y, next.y, tolerance) &&
            std::abs(angleError(predicted.robot.yaw, next.yaw)) <= tolerance &&
            within(predicted.robot.v, next.v, tolerance) &&
            within(predicted_progress, next.s, tolerance) &&
            within(predicted.robot.omega, next.omega, tolerance) &&
            within(predicted.slosh.eta_x, next.eta_x, tolerance) &&
            within(predicted.slosh.eta_x_dot, next.eta_x_dot, tolerance) &&
            within(predicted.slosh.eta_y, next.eta_y, tolerance) &&
            within(predicted.slosh.eta_y_dot, next.eta_y_dot, tolerance) &&
            within(predicted.linear.actuator_output,
                   next.augmented_execution.linear.actuator_output,
                   tolerance) &&
            within(predicted.angular.actuator_output,
                   next.augmented_execution.angular.actuator_output,
                   tolerance) &&
            samePendingCommands(
                predicted.linear.pending_commands,
                next.augmented_execution.linear.pending_commands,
                tolerance) &&
            samePendingCommands(
                predicted.angular.pending_commands,
                next.augmented_execution.angular.pending_commands,
                tolerance);
        if (!state_consistent) {
            return failure("DYNAMICS_TRANSITION_MISMATCH",
                           "index " + std::to_string(index));
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

NominalArtifactLoadResult validateRecoveryCommandBaseline(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    // Both V2 and V3 bind kappa to the nominal published command.  V3's
    // state-dependent recovery policy applies its bounded residual online;
    // kappa remains the audited, dynamically checked baseline rather than a
    // precomputed fake recovery command.
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const PhaseNominalSample& sample = samples[i];
        if (!within(sample.kappa_v, sample.u_pub_v, tolerance) ||
            !within(sample.kappa_omega, sample.u_pub_omega, tolerance)) {
            return failure("RECOVERY_COMMAND_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

NominalArtifactLoadResult validateV2RecoveryCommands(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    if (metadata.recovery_contract != kRecoveryContractV1) {
        return failure("UNSUPPORTED_RECOVERY_CONTRACT",
                       metadata.recovery_contract);
    }
    return validateRecoveryCommandBaseline(samples, metadata);
}

NominalArtifactLoadResult validateV3RecoveryPolicy(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    const BoundedTrackingRecoveryPolicyParams frozen =
        boundedTrackingRecoveryPolicyV1Params();
    if (metadata.recovery_contract != frozen.contract_id) {
        return failure("UNSUPPORTED_RECOVERY_CONTRACT",
                       metadata.recovery_contract);
    }
    const struct {
        const char* field;
        double actual;
        double expected;
    } fields[] = {
        {"recovery_policy_longitudinal_position_gain",
         metadata.recovery_policy_longitudinal_position_gain,
         frozen.longitudinal_position_gain},
        {"recovery_policy_lateral_position_gain",
         metadata.recovery_policy_lateral_position_gain,
         frozen.lateral_position_gain},
        {"recovery_policy_yaw_gain",
         metadata.recovery_policy_yaw_gain, frozen.yaw_gain},
        {"recovery_policy_linear_velocity_gain",
         metadata.recovery_policy_linear_velocity_gain,
         frozen.linear_velocity_gain},
        {"recovery_policy_angular_velocity_gain",
         metadata.recovery_policy_angular_velocity_gain,
         frozen.angular_velocity_gain},
        {"recovery_policy_max_residual_v",
         metadata.recovery_policy_max_residual_v,
         frozen.max_residual_v},
        {"recovery_policy_max_residual_omega",
         metadata.recovery_policy_max_residual_omega,
         frozen.max_residual_omega},
        {"recovery_policy_published_linear_min",
         metadata.recovery_policy_published_linear_min,
         frozen.published_linear_min},
        {"recovery_policy_published_linear_max",
         metadata.recovery_policy_published_linear_max,
         frozen.published_linear_max},
        {"recovery_policy_published_angular_min",
         metadata.recovery_policy_published_angular_min,
         frozen.published_angular_min},
        {"recovery_policy_published_angular_max",
         metadata.recovery_policy_published_angular_max,
         frozen.published_angular_max},
    };
    for (const auto& field : fields) {
        if (field.actual != field.expected) {
            return failure("V3_RECOVERY_POLICY_MISMATCH", field.field);
        }
    }
    return validateRecoveryCommandBaseline(samples, metadata);
}

NominalArtifactLoadResult validateV2TerminalTail(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    if (metadata.terminal_contract != kTerminalContractV1) {
        return failure("UNSUPPORTED_TERMINAL_CONTRACT",
                       metadata.terminal_contract);
    }
    if (metadata.terminal_zero_hold_steps < 5 ||
        metadata.terminal_zero_hold_steps >= samples.size()) {
        return failure("INVALID_ZERO_HOLD_LENGTH",
                       std::to_string(metadata.terminal_zero_hold_steps));
    }
    const std::size_t begin = samples.size() -
        metadata.terminal_zero_hold_steps;
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = begin; i < samples.size(); ++i) {
        const PhaseNominalSample& sample = samples[i];
        const double values[] = {
            sample.v, sample.omega, sample.a, sample.alpha, sample.v_s,
            sample.u_pub_v, sample.u_pub_omega,
            sample.kappa_v, sample.kappa_omega,
        };
        for (double value : values) {
            if (std::abs(value) > tolerance) {
                return failure("ZERO_HOLD_COMMAND_NONZERO",
                               "index " + std::to_string(i));
            }
        }
        if (!within(sample.s, metadata.path_length, tolerance)) {
            return failure("ZERO_HOLD_PROGRESS_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    const PhaseNominalSample& final = samples.back();
    if (std::hypot(final.eta_x, final.eta_y) >
            metadata.terminal_eta_norm_max + tolerance ||
        std::hypot(final.eta_x_dot, final.eta_y_dot) >
            metadata.terminal_eta_dot_norm_max + tolerance) {
        return failure("TERMINAL_LIQUID_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

NominalArtifactLoadResult validateV3PublishZeroSettleTail(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    if (metadata.terminal_contract != kTerminalContractV2) {
        return failure("UNSUPPORTED_TERMINAL_CONTRACT",
                       metadata.terminal_contract);
    }
    if (metadata.terminal_zero_hold_steps < 5 ||
        metadata.terminal_zero_hold_steps >= samples.size()) {
        return failure("INVALID_ZERO_HOLD_LENGTH",
                       std::to_string(metadata.terminal_zero_hold_steps));
    }

    const std::size_t begin = samples.size() -
        metadata.terminal_zero_hold_steps;
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = begin; i < samples.size(); ++i) {
        const PhaseNominalSample& sample = samples[i];
        // This is a zero *published-command* hold.  The executed velocity and
        // actuator outputs may decay throughout the hold when tau is nonzero.
        const double zero_command_values[] = {
            sample.a, sample.alpha, sample.v_s,
            sample.u_pub_v, sample.u_pub_omega,
            sample.kappa_v, sample.kappa_omega,
        };
        for (double value : zero_command_values) {
            if (std::abs(value) > tolerance) {
                return failure("ZERO_HOLD_COMMAND_NONZERO",
                               "index " + std::to_string(i));
            }
        }
        if (!within(sample.s, metadata.path_length, tolerance)) {
            return failure("ZERO_HOLD_PROGRESS_MISMATCH",
                           "index " + std::to_string(i));
        }
    }

    const PhaseNominalSample& final = samples.back();
    if (std::abs(final.v) > metadata.terminal_v_abs_max + tolerance) {
        return failure("TERMINAL_LINEAR_VELOCITY_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    if (std::abs(final.omega) >
            metadata.terminal_omega_abs_max + tolerance) {
        return failure("TERMINAL_ANGULAR_VELOCITY_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    if (!final.augmented_execution_valid ||
        !final.augmented_execution.valid) {
        return failure("TERMINAL_EXECUTION_STATE_INVALID",
                       "index " + std::to_string(final.index));
    }
    if (std::abs(final.augmented_execution.linear.actuator_output) >
            metadata.terminal_linear_actuator_output_abs_max + tolerance) {
        return failure("TERMINAL_LINEAR_ACTUATOR_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    if (std::abs(final.augmented_execution.angular.actuator_output) >
            metadata.terminal_angular_actuator_output_abs_max + tolerance) {
        return failure("TERMINAL_ANGULAR_ACTUATOR_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    for (double command :
         final.augmented_execution.linear.pending_commands) {
        if (std::abs(command) >
                metadata.terminal_linear_pending_command_abs_max) {
            return failure("TERMINAL_LINEAR_PENDING_COMMAND_NONZERO",
                           "index " + std::to_string(final.index));
        }
    }
    for (double command :
         final.augmented_execution.angular.pending_commands) {
        if (std::abs(command) >
                metadata.terminal_angular_pending_command_abs_max) {
            return failure("TERMINAL_ANGULAR_PENDING_COMMAND_NONZERO",
                           "index " + std::to_string(final.index));
        }
    }
    if (std::hypot(final.eta_x, final.eta_y) >
            metadata.terminal_eta_norm_max + tolerance ||
        std::hypot(final.eta_x_dot, final.eta_y_dot) >
            metadata.terminal_eta_dot_norm_max + tolerance) {
        return failure("TERMINAL_LIQUID_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }

    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

const std::vector<std::string>& canonicalMetadataOrder() {
    static const std::vector<std::string> order = {
        "schema", "evidence_level", "source", "contract_id", "frame_id",
        "dt", "path_length",
        "artifact_role", "nominal_sequence_kind", "offline_slosh_ocp",
        "hardware_formal_release", "paper_main_result_eligible",
        "cycle_id_first", "cycle_id_last", "cycle_count", "planner_variant",
        "gate_parameter_source", "recovery_policy_source", "gate_evidence",
        "recovery_policy_evidence", "bag_sha256",
        "development_parameter_sha256", "row_state_semantics",
        "row_command_semantics",
        "source_bag_sha256", "path_topic", "max_nominal_path_deviation_m",
        "terminal_contract", "recovery_contract",
        "recovery_policy_longitudinal_position_gain",
        "recovery_policy_lateral_position_gain",
        "recovery_policy_yaw_gain",
        "recovery_policy_linear_velocity_gain",
        "recovery_policy_angular_velocity_gain",
        "recovery_policy_max_residual_v",
        "recovery_policy_max_residual_omega",
        "recovery_policy_published_linear_min",
        "recovery_policy_published_linear_max",
        "recovery_policy_published_angular_min",
        "recovery_policy_published_angular_max",
        "terminal_zero_hold_steps",
        "terminal_eta_norm_max", "terminal_eta_dot_norm_max",
        "terminal_v_abs_max", "terminal_omega_abs_max",
        "terminal_linear_actuator_output_abs_max",
        "terminal_angular_actuator_output_abs_max",
        "terminal_linear_pending_command_abs_max",
        "terminal_angular_pending_command_abs_max",
        "two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y",
        "dynamics_tolerance", "execution_contract_id",
        "execution_contract_hash", "execution_state_width",
        "execution_linear_buffer_count", "execution_angular_buffer_count",
        "parameter_schema_version", "parameter_schema_id",
        "parameter_schema_hash", "recovery_artifact_hash",
        "execution_compatibility_contract",
    };
    return order;
}

std::string canonicalCsvText(
    const std::map<std::string, std::string>& metadata,
    const std::vector<PhaseNominalSample>& samples) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    for (const std::string& key : canonicalMetadataOrder()) {
        const auto item = metadata.find(key);
        if (item != metadata.end()) {
            out << "# " << item->first << '=' << item->second << '\n';
        }
    }
    for (const auto& item : metadata) {
        bool already_written = false;
        for (const std::string& key : canonicalMetadataOrder()) {
            if (key == item.first) {
                already_written = true;
                break;
            }
        }
        if (!already_written) {
            out << "# " << item.first << '=' << item.second << '\n';
        }
    }
    const bool augmented = metadata.count("schema") != 0 &&
        metadata.at("schema") == kSchemaV3;
    const std::vector<std::string>& header = expectedHeader(augmented);
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (i != 0) {
            out << ',';
        }
        out << header[i];
    }
    out << '\n' << std::setprecision(17);
    for (const PhaseNominalSample& sample : samples) {
        out << sample.index;
        const double values[] = {
            sample.t, sample.s, sample.x, sample.y, sample.yaw,
            sample.v, sample.omega,
            sample.eta_x, sample.eta_x_dot, sample.eta_y, sample.eta_y_dot,
            sample.a, sample.alpha, sample.v_s,
            sample.u_pub_v, sample.u_pub_omega,
            sample.kappa_v, sample.kappa_omega,
            sample.radii.x, sample.radii.y, sample.radii.yaw,
            sample.radii.v, sample.radii.omega,
            sample.radii.eta_x, sample.radii.eta_x_dot,
            sample.radii.eta_y, sample.radii.eta_y_dot,
        };
        for (double value : values) {
            out << ',' << value;
        }
        if (augmented) {
            out << ',' << sample.augmented_execution.linear.actuator_output
                << ',' << sample.augmented_execution.angular.actuator_output
                << ',' << doubleList(
                    sample.augmented_execution.linear.pending_commands)
                << ',' << doubleList(
                    sample.augmented_execution.angular.pending_commands)
                << ',' << sample.execution_bounds.linear_actuator_output
                << ',' << sample.execution_bounds.angular_actuator_output
                << ',' << doubleList(
                    sample.execution_bounds.linear_pending_commands)
                << ',' << doubleList(
                    sample.execution_bounds.angular_pending_commands);
        }
        out << '\n';
    }
    return out.str();
}

std::string sha256Hex(const std::string& contents) {
    std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
    if (::SHA256(reinterpret_cast<const unsigned char*>(contents.data()),
                 contents.size(), digest.data()) == nullptr) {
        return std::string();
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hex << std::setfill('0');
    for (const unsigned char byte : digest) {
        out << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return out.str();
}

std::string canonicalRecoveryHash(
    const std::map<std::string, std::string>& metadata,
    const std::vector<PhaseNominalSample>& samples) {
    std::map<std::string, std::string> payload_metadata = metadata;
    payload_metadata.erase("recovery_artifact_hash");
    return sha256Hex(canonicalCsvText(payload_metadata, samples));
}

NominalArtifactLoadResult writeAtomically(const std::string& path,
                                          const std::string& contents,
                                          bool overwrite) {
    if (path.empty()) {
        return failure("INVALID_OUTPUT_PATH", path);
    }
    struct stat output_status;
    if (!overwrite && ::stat(path.c_str(), &output_status) == 0) {
        return failure("OUTPUT_EXISTS", path);
    }

    std::vector<char> temporary(path.begin(), path.end());
    const std::string suffix = ".tmp.XXXXXX";
    temporary.insert(temporary.end(), suffix.begin(), suffix.end());
    temporary.push_back('\0');
    const int descriptor = ::mkstemp(temporary.data());
    if (descriptor < 0) {
        return failure("TEMPORARY_OPEN_FAILED", path);
    }
    const std::string temporary_path(temporary.data());
    bool write_ok = true;
    std::size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t count = ::write(descriptor, contents.data() + offset,
                                      contents.size() - offset);
        if (count <= 0) {
            write_ok = false;
            break;
        }
        offset += static_cast<std::size_t>(count);
    }
    if (write_ok && ::fsync(descriptor) != 0) {
        write_ok = false;
    }
    if (::close(descriptor) != 0) {
        write_ok = false;
    }
    if (!write_ok) {
        ::unlink(temporary_path.c_str());
        return failure("WRITE_FAILED", path);
    }

    bool publish_ok = false;
    if (overwrite) {
        publish_ok = ::rename(temporary_path.c_str(), path.c_str()) == 0;
    } else {
        // link+unlink gives no-overwrite publication without a check/rename race.
        publish_ok = ::link(temporary_path.c_str(), path.c_str()) == 0;
        if (publish_ok) {
            ::unlink(temporary_path.c_str());
        }
    }
    if (!publish_ok) {
        const bool output_exists = !overwrite && errno == EEXIST;
        ::unlink(temporary_path.c_str());
        return failure(output_exists ? "OUTPUT_EXISTS" : "PUBLISH_FAILED",
                       path);
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    result.detail = path;
    return result;
}

}  // namespace

std::string NominalSequenceArtifact::canonicalRecoveryArtifactHash(
    const std::map<std::string, std::string>& metadata,
    const std::vector<PhaseNominalSample>& samples) {
    return canonicalRecoveryHash(metadata, samples);
}

void NominalSequenceArtifact::clear() {
    valid_ = false;
    path_.clear();
    metadata_ = NominalArtifactMetadata{};
    metadata_entries_.clear();
    samples_.clear();
}

const PhaseNominalSample* NominalSequenceArtifact::sample(std::size_t index) const {
    return index < samples_.size() ? &samples_[index] : nullptr;
}

NominalArtifactLoadResult NominalSequenceArtifact::validateDevelopmentOnly()
    const {
    if (!valid_) {
        return failure("ARTIFACT_NOT_LOADED", path_);
    }
    const bool proxy_profile =
        metadata_.schema == kSchemaV1 &&
        metadata_.source == "development_proxy_replay";
    const bool nominal_profile =
        metadata_.schema == kSchemaV2 &&
        metadata_.source == "development_dynamics_consistent_nominal";
    if (!proxy_profile && !nominal_profile) {
        return failure("UNSUPPORTED_DEVELOPMENT_PROFILE", metadata_.source);
    }
    const auto& expected_metadata = proxy_profile
        ? developmentProxyMetadata()
        : developmentNominalMetadata();
    for (const auto& expected : expected_metadata) {
        const auto actual = metadata_entries_.find(expected.first);
        if (actual == metadata_entries_.end() ||
            actual->second != expected.second) {
            return failure("DEVELOPMENT_METADATA_MISMATCH", expected.first);
        }
    }
    if (std::abs(samples_.front().t) > 1e-12) {
        return failure("DEVELOPMENT_TIME_ORIGIN_MISMATCH", path_);
    }
    if (proxy_profile) {
        const char* required[] = {
            "cycle_id_first", "cycle_id_last", "cycle_count",
            "planner_variant", "bag_sha256", "development_parameter_sha256",
            "row_state_semantics", "row_command_semantics",
        };
        for (const char* key : required) {
            if (metadata_entries_.count(key) == 0) {
                return failure("MISSING_DEVELOPMENT_METADATA", key);
            }
        }
        if (!isLowercaseSha256(metadata_entries_.at("bag_sha256"))) {
            return failure("INVALID_SHA256", "bag_sha256");
        }
        if (!isLowercaseSha256(
                metadata_entries_.at("development_parameter_sha256"))) {
            return failure("INVALID_SHA256", "development_parameter_sha256");
        }
        std::size_t first_cycle = 0;
        std::size_t last_cycle = 0;
        std::size_t cycle_count = 0;
        if (!parsePositiveIndex(metadata_entries_.at("cycle_id_first"),
                                first_cycle) ||
            !parsePositiveIndex(metadata_entries_.at("cycle_id_last"),
                                last_cycle) ||
            !parsePositiveIndex(metadata_entries_.at("cycle_count"),
                                cycle_count)) {
            return failure("INVALID_CYCLE_METADATA", path_);
        }
        if (last_cycle < first_cycle || cycle_count != samples_.size() ||
            last_cycle - first_cycle != cycle_count - 1) {
            return failure("CYCLE_RANGE_MISMATCH", path_);
        }
    } else {
        const char* required[] = {
            "source_bag_sha256", "path_topic", "max_nominal_path_deviation_m",
        };
        for (const char* key : required) {
            if (metadata_entries_.count(key) == 0) {
                return failure("MISSING_DEVELOPMENT_METADATA", key);
            }
        }
        if (!isLowercaseSha256(metadata_entries_.at("source_bag_sha256"))) {
            return failure("INVALID_SHA256", "source_bag_sha256");
        }
        double max_deviation = 0.0;
        if (!parseDouble(metadata_entries_.at("max_nominal_path_deviation_m"),
                         max_deviation) ||
            max_deviation < 0.0 || max_deviation > 0.20) {
            return failure("INVALID_PATH_DEVIATION_METADATA",
                           "max_nominal_path_deviation_m");
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    result.detail = path_;
    return result;
}

NominalArtifactLoadResult NominalSequenceArtifact::writeCanonicalCsv(
    const std::string& path, bool overwrite) const {
    if (!valid_) {
        return failure("ARTIFACT_NOT_LOADED", path_);
    }
    return writeAtomically(path,
                           canonicalCsvText(metadata_entries_, samples_),
                           overwrite);
}

NominalArtifactLoadResult NominalSequenceArtifact::loadCsv(
    const std::string& path) {
    clear();
    std::ifstream input(path);
    if (!input.is_open()) {
        return failure("OPEN_FAILED", path);
    }
    return loadCsvStream(input, path);
}

NominalArtifactLoadResult NominalSequenceArtifact::assignValidated(
    const std::map<std::string, std::string>& metadata,
    const std::vector<PhaseNominalSample>& samples,
    const std::string& source_name) {
    std::istringstream input(canonicalCsvText(metadata, samples));
    return loadCsvStream(input, source_name);
}

NominalArtifactLoadResult NominalSequenceArtifact::loadCsvStream(
    std::istream& input, const std::string& path) {
    clear();

    std::map<std::string, std::string> metadata;
    bool header_seen = false;
    std::size_t line_number = 0;
    std::string line;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string clean = trim(line);
        if (clean.empty()) {
            continue;
        }
        if (clean[0] == '#') {
            const std::string meta = trim(clean.substr(1));
            const std::size_t separator = meta.find('=');
            if (separator == std::string::npos) {
                return failure("INVALID_METADATA",
                               "line " + std::to_string(line_number));
            }
            const std::string key = trim(meta.substr(0, separator));
            const std::string value = trim(meta.substr(separator + 1));
            if (key.empty() || value.empty() || metadata.count(key) != 0) {
                return failure("INVALID_METADATA",
                               "line " + std::to_string(line_number));
            }
            metadata[key] = value;
            continue;
        }

        const std::vector<std::string> columns = splitCsv(clean);
        const bool augmented = metadata.count("schema") != 0 &&
            metadata.at("schema") == kSchemaV3;
        if (!header_seen) {
            if (columns != expectedHeader(augmented)) {
                return failure("HEADER_MISMATCH",
                               "line " + std::to_string(line_number));
            }
            header_seen = true;
            continue;
        }
        if (columns.size() != expectedHeader(augmented).size()) {
            return failure("COLUMN_COUNT_MISMATCH",
                           "line " + std::to_string(line_number));
        }

        PhaseNominalSample sample;
        std::size_t c = 0;
        bool ok = parseIndex(columns[c++], sample.index);
        double* values[] = {
            &sample.t, &sample.s, &sample.x, &sample.y, &sample.yaw,
            &sample.v, &sample.omega,
            &sample.eta_x, &sample.eta_x_dot, &sample.eta_y,
            &sample.eta_y_dot, &sample.a, &sample.alpha, &sample.v_s,
            &sample.u_pub_v, &sample.u_pub_omega,
            &sample.kappa_v, &sample.kappa_omega,
            &sample.radii.x, &sample.radii.y, &sample.radii.yaw,
            &sample.radii.v, &sample.radii.omega,
            &sample.radii.eta_x, &sample.radii.eta_x_dot,
            &sample.radii.eta_y, &sample.radii.eta_y_dot,
        };
        for (double* value : values) {
            ok = parseDouble(columns[c++], *value) && ok;
        }
        if (augmented) {
            const auto linear_count_item = metadata.find(
                "execution_linear_buffer_count");
            const auto angular_count_item = metadata.find(
                "execution_angular_buffer_count");
            std::size_t linear_count = 0;
            std::size_t angular_count = 0;
            ok = linear_count_item != metadata.end() &&
                angular_count_item != metadata.end() &&
                parsePositiveIndex(linear_count_item->second, linear_count) &&
                parsePositiveIndex(angular_count_item->second, angular_count) &&
                ok;
            ok = parseDouble(
                     columns[c++],
                     sample.augmented_execution.linear.actuator_output) && ok;
            ok = parseDouble(
                     columns[c++],
                     sample.augmented_execution.angular.actuator_output) && ok;
            std::vector<double> linear_pending;
            std::vector<double> angular_pending;
            ok = parseDoubleList(
                     columns[c++], linear_count, linear_pending) && ok;
            ok = parseDoubleList(
                     columns[c++], angular_count, angular_pending) && ok;
            sample.augmented_execution.linear.pending_commands.assign(
                linear_pending.begin(), linear_pending.end());
            sample.augmented_execution.angular.pending_commands.assign(
                angular_pending.begin(), angular_pending.end());
            ok = parseDouble(
                     columns[c++],
                     sample.execution_bounds.linear_actuator_output) && ok;
            ok = parseDouble(
                     columns[c++],
                     sample.execution_bounds.angular_actuator_output) && ok;
            ok = parseDoubleList(
                     columns[c++], linear_count,
                     sample.execution_bounds.linear_pending_commands) && ok;
            ok = parseDoubleList(
                     columns[c++], angular_count,
                     sample.execution_bounds.angular_pending_commands) && ok;
            sample.augmented_execution.robot.x = sample.x;
            sample.augmented_execution.robot.y = sample.y;
            sample.augmented_execution.robot.yaw = sample.yaw;
            sample.augmented_execution.robot.v = sample.v;
            sample.augmented_execution.robot.omega = sample.omega;
            sample.augmented_execution.slosh.eta_x = sample.eta_x;
            sample.augmented_execution.slosh.eta_x_dot = sample.eta_x_dot;
            sample.augmented_execution.slosh.eta_y = sample.eta_y;
            sample.augmented_execution.slosh.eta_y_dot = sample.eta_y_dot;
            sample.augmented_execution.stage_index = sample.index;
            sample.augmented_execution.valid = ok;
            sample.augmented_execution_valid = ok;
            sample.execution_bounds.valid = ok;
        }
        if (!ok || c != columns.size()) {
            return failure("NONFINITE_OR_INVALID_VALUE",
                           "line " + std::to_string(line_number));
        }
        if (sample.index != samples_.size()) {
            return failure("INDEX_NOT_CONTIGUOUS",
                           "line " + std::to_string(line_number));
        }
        if (!positiveRadii(sample.radii)) {
            return failure("NONPOSITIVE_GATE_RADIUS",
                           "line " + std::to_string(line_number));
        }
        if (augmented) {
            const bool positive_execution_bounds =
                sample.execution_bounds.linear_actuator_output >=
                    augmented_manifest::kMinimumRecoveryDenominator &&
                sample.execution_bounds.angular_actuator_output >=
                    augmented_manifest::kMinimumRecoveryDenominator &&
                std::all_of(
                    sample.execution_bounds.linear_pending_commands.begin(),
                    sample.execution_bounds.linear_pending_commands.end(),
                    [](double value) {
                        return value >= augmented_manifest::
                            kMinimumRecoveryDenominator;
                    }) &&
                std::all_of(
                    sample.execution_bounds.angular_pending_commands.begin(),
                    sample.execution_bounds.angular_pending_commands.end(),
                    [](double value) {
                        return value >= augmented_manifest::
                            kMinimumRecoveryDenominator;
                    });
            if (!wellConditionedV3Radii(sample.radii) ||
                !positive_execution_bounds ||
                !within(sample.augmented_execution.linear.actuator_output,
                        sample.v, 1e-12) ||
                !within(sample.augmented_execution.angular.actuator_output,
                        sample.omega, 1e-12)) {
                return failure("INVALID_AUGMENTED_EXECUTION_ROW",
                               "line " + std::to_string(line_number));
            }
        }
        if (!samples_.empty()) {
            const PhaseNominalSample& previous = samples_.back();
            if (sample.t <= previous.t || sample.s + 1e-9 < previous.s) {
                return failure("NONMONOTONIC_SEQUENCE",
                               "line " + std::to_string(line_number));
            }
        } else if (sample.t < 0.0 || sample.s < 0.0) {
            return failure("NEGATIVE_ORIGIN",
                           "line " + std::to_string(line_number));
        }
        samples_.push_back(sample);
    }

    if (!header_seen || samples_.size() < 2) {
        clear();
        return failure("EMPTY_OR_SHORT_ARTIFACT", path);
    }

    const char* required[] = {
        "schema", "evidence_level", "source", "contract_id",
        "frame_id", "dt", "path_length",
    };
    for (const char* key : required) {
        if (metadata.count(key) == 0) {
            clear();
            return failure("MISSING_METADATA", key);
        }
    }
    const bool schema_v1 = metadata["schema"] == kSchemaV1;
    const bool schema_v2 = metadata["schema"] == kSchemaV2;
    const bool schema_v3 = metadata["schema"] == kSchemaV3;
    if (!schema_v1 && !schema_v2 && !schema_v3) {
        clear();
        return failure("UNSUPPORTED_SCHEMA", metadata["schema"]);
    }

    if (schema_v2 || schema_v3) {
        const char* required_v2[] = {
            "terminal_contract", "recovery_contract",
            "terminal_zero_hold_steps",
            "terminal_eta_norm_max", "terminal_eta_dot_norm_max",
            "two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y",
            "dynamics_tolerance",
        };
        for (const char* key : required_v2) {
            if (metadata.count(key) == 0) {
                clear();
                return failure("MISSING_METADATA", key);
            }
        }
    }
    if (schema_v3) {
        const char* required_v3[] = {
            "recovery_policy_longitudinal_position_gain",
            "recovery_policy_lateral_position_gain",
            "recovery_policy_yaw_gain",
            "recovery_policy_linear_velocity_gain",
            "recovery_policy_angular_velocity_gain",
            "recovery_policy_max_residual_v",
            "recovery_policy_max_residual_omega",
            "recovery_policy_published_linear_min",
            "recovery_policy_published_linear_max",
            "recovery_policy_published_angular_min",
            "recovery_policy_published_angular_max",
            "execution_contract_id", "execution_contract_hash",
            "execution_state_width", "execution_linear_buffer_count",
            "execution_angular_buffer_count", "parameter_schema_version",
            "parameter_schema_id", "parameter_schema_hash",
            "recovery_artifact_hash", "execution_compatibility_contract",
        };
        for (const char* key : required_v3) {
            if (metadata.count(key) == 0) {
                clear();
                return failure("MISSING_METADATA", key);
            }
        }
        if (metadata["terminal_contract"] == kTerminalContractV2) {
            const char* required_terminal_v2[] = {
                "terminal_v_abs_max", "terminal_omega_abs_max",
                "terminal_linear_actuator_output_abs_max",
                "terminal_angular_actuator_output_abs_max",
                "terminal_linear_pending_command_abs_max",
                "terminal_angular_pending_command_abs_max",
            };
            for (const char* key : required_terminal_v2) {
                if (metadata.count(key) == 0) {
                    clear();
                    return failure("MISSING_METADATA", key);
                }
            }
        }
    }

    NominalArtifactMetadata parsed_metadata;
    parsed_metadata.schema = metadata["schema"];
    parsed_metadata.source = metadata["source"];
    parsed_metadata.contract_id = metadata["contract_id"];
    parsed_metadata.frame_id = metadata["frame_id"];
    if (!parsePhaseRejoinEvidenceLevel(metadata["evidence_level"],
                                       parsed_metadata.evidence_level)) {
        clear();
        return failure("INVALID_EVIDENCE_LEVEL", metadata["evidence_level"]);
    }
    if (!parseDouble(metadata["dt"], parsed_metadata.dt) ||
        !parseDouble(metadata["path_length"], parsed_metadata.path_length) ||
        parsed_metadata.dt <= 0.0 || parsed_metadata.path_length <= 0.0 ||
        parsed_metadata.contract_id.empty() || parsed_metadata.frame_id.empty()) {
        clear();
        return failure("INVALID_METADATA_VALUE", path);
    }

    if (schema_v2 || schema_v3) {
        parsed_metadata.terminal_contract = metadata["terminal_contract"];
        parsed_metadata.recovery_contract = metadata["recovery_contract"];
        const bool valid_v2_metadata =
            parsePositiveIndex(metadata["terminal_zero_hold_steps"],
                               parsed_metadata.terminal_zero_hold_steps) &&
            parseDouble(metadata["terminal_eta_norm_max"],
                        parsed_metadata.terminal_eta_norm_max) &&
            parseDouble(metadata["terminal_eta_dot_norm_max"],
                        parsed_metadata.terminal_eta_dot_norm_max) &&
            parseDouble(metadata["two_zeta_omega_n"],
                        parsed_metadata.two_zeta_omega_n) &&
            parseDouble(metadata["omega_n_sq"],
                        parsed_metadata.omega_n_sq) &&
            parseDouble(metadata["kappa_x"], parsed_metadata.kappa_x) &&
            parseDouble(metadata["kappa_y"], parsed_metadata.kappa_y) &&
            parseDouble(metadata["dynamics_tolerance"],
                        parsed_metadata.dynamics_tolerance) &&
            parsed_metadata.terminal_eta_norm_max >= 0.0 &&
            parsed_metadata.terminal_eta_dot_norm_max >= 0.0 &&
            parsed_metadata.two_zeta_omega_n >= 0.0 &&
            parsed_metadata.omega_n_sq > 0.0 &&
            parsed_metadata.kappa_x > 0.0 &&
            parsed_metadata.kappa_y > 0.0 &&
            parsed_metadata.dynamics_tolerance >= 1e-12 &&
            parsed_metadata.dynamics_tolerance <= 1e-3 &&
            (!schema_v3 ||
             parsed_metadata.dynamics_tolerance <=
                 augmented_manifest::kPublishedConsistencyTolerance);
        if (!valid_v2_metadata) {
            clear();
            return failure(schema_v3
                               ? "INVALID_V3_METADATA_VALUE"
                               : "INVALID_V2_METADATA_VALUE",
                           path);
        }
    }

    if (schema_v3) {
        const bool valid_recovery_policy_metadata =
            parseDouble(
                metadata["recovery_policy_longitudinal_position_gain"],
                parsed_metadata.
                    recovery_policy_longitudinal_position_gain) &&
            parseDouble(
                metadata["recovery_policy_lateral_position_gain"],
                parsed_metadata.recovery_policy_lateral_position_gain) &&
            parseDouble(
                metadata["recovery_policy_yaw_gain"],
                parsed_metadata.recovery_policy_yaw_gain) &&
            parseDouble(
                metadata["recovery_policy_linear_velocity_gain"],
                parsed_metadata.recovery_policy_linear_velocity_gain) &&
            parseDouble(
                metadata["recovery_policy_angular_velocity_gain"],
                parsed_metadata.recovery_policy_angular_velocity_gain) &&
            parseDouble(
                metadata["recovery_policy_max_residual_v"],
                parsed_metadata.recovery_policy_max_residual_v) &&
            parseDouble(
                metadata["recovery_policy_max_residual_omega"],
                parsed_metadata.recovery_policy_max_residual_omega) &&
            parseDouble(
                metadata["recovery_policy_published_linear_min"],
                parsed_metadata.recovery_policy_published_linear_min) &&
            parseDouble(
                metadata["recovery_policy_published_linear_max"],
                parsed_metadata.recovery_policy_published_linear_max) &&
            parseDouble(
                metadata["recovery_policy_published_angular_min"],
                parsed_metadata.recovery_policy_published_angular_min) &&
            parseDouble(
                metadata["recovery_policy_published_angular_max"],
                parsed_metadata.recovery_policy_published_angular_max);
        if (!valid_recovery_policy_metadata) {
            clear();
            return failure("INVALID_V3_RECOVERY_POLICY_METADATA", path);
        }
        std::size_t state_width = 0;
        std::size_t linear_count = 0;
        std::size_t angular_count = 0;
        std::size_t parameter_schema_version = 0;
        const bool valid_v3_metadata =
            parsePositiveIndex(metadata["execution_state_width"],
                               state_width) &&
            parsePositiveIndex(metadata["execution_linear_buffer_count"],
                               linear_count) &&
            parsePositiveIndex(metadata["execution_angular_buffer_count"],
                               angular_count) &&
            parsePositiveIndex(metadata["parameter_schema_version"],
                               parameter_schema_version) &&
            state_width == 10 + linear_count + angular_count &&
            isLowercaseSha256(metadata["execution_contract_hash"]) &&
            isLowercaseSha256(metadata["parameter_schema_hash"]) &&
            isLowercaseSha256(metadata["recovery_artifact_hash"]) &&
            !metadata["execution_contract_id"].empty() &&
            !metadata["parameter_schema_id"].empty() &&
            metadata["execution_compatibility_contract"] ==
                "phase_indexed_execution_box_v1";
        if (!valid_v3_metadata) {
            clear();
            return failure("INVALID_V3_EXECUTION_METADATA", path);
        }
        const std::string verified_recovery_hash =
            canonicalRecoveryHash(metadata, samples_);
        if (verified_recovery_hash.empty() ||
            metadata["recovery_artifact_hash"] != verified_recovery_hash) {
            clear();
            return failure("RECOVERY_ARTIFACT_HASH_MISMATCH",
                           "recovery_artifact_hash");
        }
        parsed_metadata.delay_augmented_nominal = true;
        parsed_metadata.execution_contract_id =
            metadata["execution_contract_id"];
        parsed_metadata.execution_contract_hash =
            metadata["execution_contract_hash"];
        parsed_metadata.execution_state_width =
            static_cast<int>(state_width);
        parsed_metadata.linear_buffer_count =
            static_cast<int>(linear_count);
        parsed_metadata.angular_buffer_count =
            static_cast<int>(angular_count);
        parsed_metadata.parameter_schema_version =
            static_cast<int>(parameter_schema_version);
        parsed_metadata.parameter_schema_id =
            metadata["parameter_schema_id"];
        parsed_metadata.parameter_schema_hash =
            metadata["parameter_schema_hash"];
        parsed_metadata.recovery_artifact_hash = verified_recovery_hash;
        parsed_metadata.execution_compatibility_contract =
            metadata["execution_compatibility_contract"];

        if (parsed_metadata.terminal_contract == kTerminalContractV2) {
            const bool valid_terminal_v2_metadata =
                parseDouble(metadata["terminal_v_abs_max"],
                            parsed_metadata.terminal_v_abs_max) &&
                parseDouble(metadata["terminal_omega_abs_max"],
                            parsed_metadata.terminal_omega_abs_max) &&
                parseDouble(
                    metadata["terminal_linear_actuator_output_abs_max"],
                    parsed_metadata.
                        terminal_linear_actuator_output_abs_max) &&
                parseDouble(
                    metadata["terminal_angular_actuator_output_abs_max"],
                    parsed_metadata.
                        terminal_angular_actuator_output_abs_max) &&
                parseDouble(
                    metadata["terminal_linear_pending_command_abs_max"],
                    parsed_metadata.
                        terminal_linear_pending_command_abs_max) &&
                parseDouble(
                    metadata["terminal_angular_pending_command_abs_max"],
                    parsed_metadata.
                        terminal_angular_pending_command_abs_max) &&
                parsed_metadata.terminal_v_abs_max >= 0.0 &&
                parsed_metadata.terminal_omega_abs_max >= 0.0 &&
                parsed_metadata.
                    terminal_linear_actuator_output_abs_max >= 0.0 &&
                parsed_metadata.
                    terminal_angular_actuator_output_abs_max >= 0.0 &&
                parsed_metadata.
                    terminal_linear_pending_command_abs_max >= 0.0 &&
                parsed_metadata.
                    terminal_angular_pending_command_abs_max >= 0.0 &&
                parsed_metadata.
                    terminal_linear_pending_command_abs_max <=
                        parsed_metadata.dynamics_tolerance &&
                parsed_metadata.
                    terminal_angular_pending_command_abs_max <=
                        parsed_metadata.dynamics_tolerance;
            if (!valid_terminal_v2_metadata) {
                clear();
                return failure("INVALID_V3_TERMINAL_METADATA", path);
            }
        } else if (parsed_metadata.terminal_contract !=
                   kTerminalContractV1) {
            clear();
            return failure("UNSUPPORTED_TERMINAL_CONTRACT",
                           parsed_metadata.terminal_contract);
        }
    }

    // The development proxy publishes /clock at 50 Hz while the controller
    // runs at 30 Hz, yielding a bounded 40/40/20 ms timer pattern.  Permit the
    // corresponding local quantization, while the cumulative phase bound below
    // still rejects a stream that drifts away from the declared nominal dt.
    const double period_tolerance = (schema_v2 || schema_v3)
        ? std::max(1e-9, 1e-7 * parsed_metadata.dt)
        : std::max(1e-4, 0.40 * parsed_metadata.dt) + 1e-9;
    const double phase_tolerance = (schema_v2 || schema_v3)
        ? std::max(1e-9, 1e-7 * parsed_metadata.dt)
        : std::max(1e-4, parsed_metadata.dt) + 1e-9;
    const double first_time = samples_.front().t;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
        const double period = samples_[i].t - samples_[i - 1].t;
        if (std::abs(period - parsed_metadata.dt) > period_tolerance) {
            clear();
            return failure("SAMPLE_PERIOD_MISMATCH",
                           "index " + std::to_string(i));
        }
        const double nominal_time = first_time +
            static_cast<double>(i) * parsed_metadata.dt;
        if (std::abs(samples_[i].t - nominal_time) > phase_tolerance) {
            clear();
            return failure("SAMPLE_PHASE_DRIFT",
                           "index " + std::to_string(i));
        }
    }
    // The artifact is a complete nominal tail, not an arbitrary path prefix.
    // Keeping the tolerance small allows the final discrete sample to fall one
    // nominal step short of the geometric endpoint without accepting a
    // truncated sequence whose metadata merely claims the full path length.
    constexpr double kEndpointToleranceM = 0.100001;
    if (std::abs(samples_.back().s - parsed_metadata.path_length) >
        kEndpointToleranceM) {
        clear();
        return failure("PATH_LENGTH_MISMATCH", path);
    }

    if (schema_v2) {
        const NominalArtifactLoadResult transition_result =
            validateV2Transitions(samples_, parsed_metadata);
        if (!transition_result.success) {
            clear();
            return transition_result;
        }
        const NominalArtifactLoadResult recovery_result =
            validateV2RecoveryCommands(samples_, parsed_metadata);
        if (!recovery_result.success) {
            clear();
            return recovery_result;
        }
        const NominalArtifactLoadResult terminal_result =
            validateV2TerminalTail(samples_, parsed_metadata);
        if (!terminal_result.success) {
            clear();
            return terminal_result;
        }
        parsed_metadata.complete_terminal_tail = true;
    } else if (schema_v3) {
        const NominalArtifactLoadResult transition_result =
            validateV3Transitions(samples_, parsed_metadata);
        if (!transition_result.success) {
            clear();
            return transition_result;
        }
        const NominalArtifactLoadResult recovery_result =
            validateV3RecoveryPolicy(samples_, parsed_metadata);
        if (!recovery_result.success) {
            clear();
            return recovery_result;
        }
        const NominalArtifactLoadResult terminal_result =
            parsed_metadata.terminal_contract == kTerminalContractV2
            ? validateV3PublishZeroSettleTail(samples_, parsed_metadata)
            : validateV2TerminalTail(samples_, parsed_metadata);
        if (!terminal_result.success) {
            clear();
            return terminal_result;
        }
        parsed_metadata.complete_terminal_tail = true;
    }

    path_ = path;
    metadata_ = parsed_metadata;
    metadata_entries_ = metadata;
    valid_ = true;
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    result.detail = path;
    return result;
}

}  // namespace spmpc_local_planner
