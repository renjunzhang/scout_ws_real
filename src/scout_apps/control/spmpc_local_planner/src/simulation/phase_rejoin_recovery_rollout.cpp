#include "spmpc_local_planner/simulation/phase_rejoin_recovery_rollout.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <set>
#include <sstream>

namespace spmpc_local_planner {
namespace simulation {
namespace {

constexpr const char* kSamplingSchema =
    "spmpc_phase_rejoin_recovery_rollout_sampling_v1";
constexpr double kTimeEpsilonSec = 1.0e-10;
constexpr double kValueEpsilon = 1.0e-12;
constexpr std::size_t kStateErrorCount = 9;
constexpr std::size_t kExecutionErrorCount = 14;
constexpr std::size_t kLinearPendingCount = 5;
constexpr std::size_t kAngularPendingCount = 7;

bool finite(double value) {
    return std::isfinite(value);
}

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

bool validIdentifier(const std::string& value) {
    if (value.empty() || value.size() > 64) return false;
    for (char character : value) {
        const bool valid =
            (character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') ||
            character == '_' || character == '-';
        if (!valid) return false;
    }
    return true;
}

bool validSplit(const std::string& split) {
    return split == "fit" || split == "tune" || split == "held_out";
}

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

template <typename T>
bool requiredScalar(const YAML::Node& node,
                    const char* key,
                    T& value,
                    std::string& error) {
    try {
        if (!node || !node[key] || !node[key].IsScalar()) {
            error = std::string("missing scalar: ") + key;
            return false;
        }
        value = node[key].as<T>();
        return true;
    } catch (const std::exception& exception) {
        error = std::string("invalid scalar ") + key + ": " +
            exception.what();
        return false;
    }
}

struct RuntimeState {
    IndependentScoutLiquidPlant plant;
    SloshState observed_slosh;
    std::deque<double> linear_pending;
    std::deque<double> angular_pending;
};

void updatePending(std::deque<double>& pending, double command) {
    pending.push_back(command);
    pending.pop_front();
}

bool advanceMotionObserver(
    RuntimeState& runtime,
    double target_time_sec,
    const SloshDynamics& observer,
    double maximum_step_sec,
    const std::function<void(const IndependentPlantState&)>& truth_recorder,
    std::string& error) {
    if (!finite(target_time_sec) ||
        target_time_sec + kTimeEpsilonSec < runtime.plant.state().time_sec ||
        !finite(maximum_step_sec) || maximum_step_sec <= 1.0e-4) {
        error = "invalid observer propagation interval";
        return false;
    }
    while (runtime.plant.state().time_sec + kTimeEpsilonSec <
           target_time_sec) {
        const IndependentPlantState previous = runtime.plant.state();
        const double next_time = std::min(
            target_time_sec, previous.time_sec + maximum_step_sec);
        const double dt_sec = next_time - previous.time_sec;
        if (dt_sec <= kTimeEpsilonSec ||
            !runtime.plant.advanceTo(next_time, error)) {
            if (error.empty()) error = "plant time did not advance";
            return false;
        }
        const IndependentPlantState& current = runtime.plant.state();
        const double ax = (current.v - previous.v) / dt_sec;
        const double ay = 0.5 *
            (previous.v * previous.omega + current.v * current.omega);
        SloshState next_slosh;
        if (!observer.stepWithDt(
                runtime.observed_slosh, ax, ay,
                0.5 * (previous.omega + current.omega),
                dt_sec, next_slosh)) {
            error = "motion-only controller liquid observer failed";
            return false;
        }
        runtime.observed_slosh = next_slosh;
        if (truth_recorder) truth_recorder(current);
    }
    return true;
}

bool publishAndAdvance(
    RuntimeState& runtime,
    const IndependentPlantCommand& command,
    double target_time_sec,
    const SloshDynamics& observer,
    double maximum_step_sec,
    const std::function<void(const IndependentPlantState&)>& truth_recorder,
    std::string& error) {
    const double publish_time_sec = runtime.plant.state().time_sec;
    IndependentPlantPublishReceipt receipt;
    if (!runtime.plant.publishCommand(
            publish_time_sec, command, receipt, error)) {
        return false;
    }
    updatePending(runtime.linear_pending, command.linear);
    updatePending(runtime.angular_pending, command.angular);
    return advanceMotionObserver(
        runtime, target_time_sec, observer, maximum_step_sec,
        truth_recorder, error);
}

double interpolatedYaw(double from, double to, double fraction) {
    return wrapAngle(from + wrapAngle(to - from) * fraction);
}

struct LabelAccumulator {
    double max_position_error = 0.0;
    double max_yaw_error = 0.0;
    double max_external_height = 0.0;
};

void recordAgainstSegment(
    LabelAccumulator& accumulator,
    const IndependentPlantState& state,
    const PhaseNominalSample& from,
    const PhaseNominalSample& to,
    double segment_start_sec,
    double segment_end_sec) {
    const double duration = segment_end_sec - segment_start_sec;
    const double fraction = duration > kTimeEpsilonSec
        ? clamp((state.time_sec - segment_start_sec) / duration, 0.0, 1.0)
        : 1.0;
    const double reference_x = from.x + (to.x - from.x) * fraction;
    const double reference_y = from.y + (to.y - from.y) * fraction;
    const double reference_yaw = interpolatedYaw(
        from.yaw, to.yaw, fraction);
    accumulator.max_position_error = std::max(
        accumulator.max_position_error,
        std::hypot(state.x - reference_x, state.y - reference_y));
    accumulator.max_yaw_error = std::max(
        accumulator.max_yaw_error,
        std::abs(wrapAngle(state.yaw - reference_yaw)));
    accumulator.max_external_height = std::max(
        accumulator.max_external_height, state.true_height_m);
}

RobotState observedRobot(const IndependentPlantState& state) {
    RobotState robot;
    robot.x = state.x;
    robot.y = state.y;
    robot.yaw = state.yaw;
    robot.v = state.v;
    robot.omega = state.omega;
    return robot;
}

bool trackingRecoveryCommand(
    const BoundedTrackingRecoveryPolicy& policy,
    const PhaseNominalSample& nominal,
    const IndependentPlantState& observed,
    IndependentPlantCommand& command,
    std::string& error) {
    const BoundedTrackingRecoveryPolicyResult evaluated =
        policy.evaluate(nominal, observedRobot(observed));
    if (!evaluated.valid) {
        error = "bounded tracking recovery policy failed: " +
            evaluated.status;
        return false;
    }
    command.linear = evaluated.command.linear;
    command.angular = evaluated.command.angular;
    return true;
}

bool validNominalSample(const PhaseNominalSample& sample,
                        std::size_t expected_index,
                        double expected_t,
                        double dt,
                        const RecoveryRolloutSamplingConfig& config,
                        std::string& error) {
    const double values[] = {
        sample.t, sample.x, sample.y, sample.yaw, sample.v, sample.omega,
        sample.eta_x, sample.eta_x_dot, sample.eta_y, sample.eta_y_dot,
        sample.u_pub_v, sample.u_pub_omega,
        sample.kappa_v, sample.kappa_omega,
    };
    if (sample.index != expected_index ||
        std::any_of(std::begin(values), std::end(values),
            [](double value) { return !finite(value); }) ||
        std::abs(sample.t - expected_t) >
            std::max(1.0e-9, dt * 1.0e-6) ||
        !sample.augmented_execution_valid ||
        !sample.augmented_execution.valid ||
        sample.augmented_execution.linear.pending_commands.size() !=
            kLinearPendingCount ||
        sample.augmented_execution.angular.pending_commands.size() !=
            kAngularPendingCount ||
        sample.u_pub_v < config.published_linear_min - kValueEpsilon ||
        sample.u_pub_v > config.published_linear_max + kValueEpsilon ||
        sample.u_pub_omega < config.published_angular_min - kValueEpsilon ||
        sample.u_pub_omega > config.published_angular_max + kValueEpsilon ||
        sample.kappa_v < config.published_linear_min - kValueEpsilon ||
        sample.kappa_v > config.published_linear_max + kValueEpsilon ||
        sample.kappa_omega < config.published_angular_min - kValueEpsilon ||
        sample.kappa_omega > config.published_angular_max + kValueEpsilon) {
        error = "invalid nominal projection at phase " +
            std::to_string(expected_index);
        return false;
    }
    return true;
}

RuntimeState snapshotForGlobalStage(
    int stage,
    int warmup_steps,
    const std::vector<RuntimeState>& warmup_snapshots,
    const std::vector<RuntimeState>& nominal_snapshots) {
    if (stage < 0) {
        return warmup_snapshots[static_cast<std::size_t>(
            stage + warmup_steps)];
    }
    return nominal_snapshots[static_cast<std::size_t>(stage)];
}

double stageTimeSec(int stage,
                    int warmup_steps,
                    double nominal_dt_sec,
                    const std::vector<PhaseNominalSample>& samples) {
    if (stage < 0) {
        return static_cast<double>(stage + warmup_steps) * nominal_dt_sec;
    }
    if (stage < static_cast<int>(samples.size())) {
        return static_cast<double>(warmup_steps) * nominal_dt_sec +
            samples[static_cast<std::size_t>(stage)].t;
    }
    const PhaseNominalSample& final = samples.back();
    return static_cast<double>(warmup_steps) * nominal_dt_sec +
        final.t + static_cast<double>(
            stage - static_cast<int>(samples.size()) + 1) *
            nominal_dt_sec;
}

std::string rolloutId(const std::string& split,
                      std::uint32_t seed,
                      std::size_t phase,
                      const std::string& profile) {
    std::ostringstream out;
    out << split << "-s" << seed << "-p" << phase << '-' << profile;
    return out.str();
}

}  // namespace

bool validateRecoveryRolloutSamplingConfig(
    const RecoveryRolloutSamplingConfig& config,
    std::string& error) {
    error.clear();
    if (config.schema != kSamplingSchema) {
        error = "unsupported recovery rollout sampling schema";
    } else if (!config.simulation_only ||
               config.external_liquid_truth_visible_to_candidate_policy ||
               config.external_liquid_truth_used_for_features ||
               !config.external_liquid_truth_used_for_label ||
               !config.controller_liquid_observer_uses_motion_only) {
        error = "recovery sampling violates the truth-isolation contract";
    } else if (config.warmup_steps <= 0 ||
               !finite(config.published_linear_min) ||
               !finite(config.published_linear_max) ||
               config.published_linear_min >= config.published_linear_max ||
               !finite(config.published_angular_min) ||
               !finite(config.published_angular_max) ||
               config.published_angular_min >= config.published_angular_max ||
               !finite(config.maximum_candidate_residual_v) ||
               config.maximum_candidate_residual_v < 0.0 ||
               !finite(config.maximum_candidate_residual_omega) ||
               config.maximum_candidate_residual_omega < 0.0) {
        error = "invalid recovery sampling command envelope";
    }
    if (error.empty() && !validateBoundedTrackingRecoveryPolicyParams(
            config.recovery_policy, error)) {
        error = "recovery policy: " + error;
    }
    const BoundedTrackingRecoveryPolicyParams frozen_policy =
        boundedTrackingRecoveryPolicyV1Params();
    if (error.empty() &&
        (config.recovery_policy.contract_id != frozen_policy.contract_id ||
         config.recovery_policy.longitudinal_position_gain !=
             frozen_policy.longitudinal_position_gain ||
         config.recovery_policy.lateral_position_gain !=
             frozen_policy.lateral_position_gain ||
         config.recovery_policy.yaw_gain != frozen_policy.yaw_gain ||
         config.recovery_policy.linear_velocity_gain !=
             frozen_policy.linear_velocity_gain ||
         config.recovery_policy.angular_velocity_gain !=
             frozen_policy.angular_velocity_gain ||
         config.recovery_policy.max_residual_v !=
             frozen_policy.max_residual_v ||
         config.recovery_policy.max_residual_omega !=
             frozen_policy.max_residual_omega ||
         config.recovery_policy.published_linear_min !=
             frozen_policy.published_linear_min ||
         config.recovery_policy.published_linear_max !=
             frozen_policy.published_linear_max ||
         config.recovery_policy.published_angular_min !=
             frozen_policy.published_angular_min ||
         config.recovery_policy.published_angular_max !=
             frozen_policy.published_angular_max)) {
        error = "recovery policy YAML does not exact-match compiled v1";
    }
    if (error.empty() &&
        (config.recovery_policy.max_residual_v >
             config.maximum_candidate_residual_v + kValueEpsilon ||
         config.recovery_policy.max_residual_omega >
             config.maximum_candidate_residual_omega + kValueEpsilon ||
         config.recovery_policy.published_linear_min !=
             config.published_linear_min ||
         config.recovery_policy.published_linear_max !=
             config.published_linear_max ||
         config.recovery_policy.published_angular_min !=
             config.published_angular_min ||
         config.recovery_policy.published_angular_max !=
             config.published_angular_max)) {
        error = "recovery policy and sampling command envelopes differ";
    }
    const RecoveryRolloutLabelContract& label = config.label;
    const double positive_values[] = {
        label.maximum_path_position_error_m,
        label.maximum_path_yaw_error_rad,
        label.maximum_external_height_m,
        label.terminal_position_error_m,
        label.terminal_yaw_error_rad,
        label.terminal_v_abs_mps,
        label.terminal_omega_abs_radps,
        label.terminal_external_height_m,
        label.fixed_tail_sec,
    };
    if (error.empty() && std::any_of(
            std::begin(positive_values), std::end(positive_values),
            [](double value) { return !finite(value) || value <= 0.0; })) {
        error = "recovery label thresholds must be finite and positive";
    }
    if (error.empty() && config.profiles.empty()) {
        error = "recovery excitation profile table is empty";
    }
    std::set<std::string> profile_ids;
    bool zero_profile = false;
    for (const RecoveryExcitationProfile& profile : config.profiles) {
        if (!error.empty()) break;
        if (!validIdentifier(profile.profile_id) ||
            !profile_ids.insert(profile.profile_id).second ||
            profile.pulse_steps < 0 ||
            profile.pulse_steps > config.warmup_steps ||
            !finite(profile.residual_v) ||
            !finite(profile.residual_omega) ||
            std::abs(profile.residual_v) >
                config.maximum_candidate_residual_v + kValueEpsilon ||
            std::abs(profile.residual_omega) >
                config.maximum_candidate_residual_omega + kValueEpsilon) {
            error = "invalid recovery excitation profile: " +
                profile.profile_id;
            break;
        }
        if (profile.residual_v == 0.0 &&
            profile.residual_omega == 0.0) {
            zero_profile = true;
        }
    }
    if (error.empty() && !zero_profile) {
        error = "recovery sampling requires one zero-residual profile";
    }
    return error.empty();
}

bool loadRecoveryRolloutSamplingConfig(
    const std::string& path,
    RecoveryRolloutSamplingConfig& config,
    std::string& error) {
    config = RecoveryRolloutSamplingConfig{};
    error.clear();
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!requiredScalar(root, "schema", config.schema, error)) return false;
        const YAML::Node scope = root["scope"];
        if (!requiredScalar(scope, "simulation_only",
                            config.simulation_only, error) ||
            !requiredScalar(
                            scope,
                            "external_liquid_truth_visible_to_candidate_policy",
                            config.external_liquid_truth_visible_to_candidate_policy,
                            error) ||
            !requiredScalar(scope, "external_liquid_truth_used_for_features",
                            config.external_liquid_truth_used_for_features,
                            error) ||
            !requiredScalar(scope, "external_liquid_truth_used_for_label",
                            config.external_liquid_truth_used_for_label,
                            error) ||
            !requiredScalar(scope,
                            "controller_liquid_observer_uses_motion_only",
                            config.controller_liquid_observer_uses_motion_only,
                            error)) {
            return false;
        }
        const YAML::Node sampling = root["sampling"];
        if (!requiredScalar(sampling, "warmup_steps",
                            config.warmup_steps, error) ||
            !requiredScalar(sampling, "published_linear_min",
                            config.published_linear_min, error) ||
            !requiredScalar(sampling, "published_linear_max",
                            config.published_linear_max, error) ||
            !requiredScalar(sampling, "published_angular_min",
                            config.published_angular_min, error) ||
            !requiredScalar(sampling, "published_angular_max",
                            config.published_angular_max, error) ||
            !requiredScalar(sampling, "maximum_candidate_residual_v",
                            config.maximum_candidate_residual_v, error) ||
            !requiredScalar(sampling, "maximum_candidate_residual_omega",
                            config.maximum_candidate_residual_omega, error)) {
            return false;
        }
        const YAML::Node policy = sampling["recovery_policy"];
        if (!requiredScalar(policy, "contract_id",
                            config.recovery_policy.contract_id, error) ||
            !requiredScalar(policy, "longitudinal_position_gain",
                            config.recovery_policy.longitudinal_position_gain,
                            error) ||
            !requiredScalar(policy, "lateral_position_gain",
                            config.recovery_policy.lateral_position_gain,
                            error) ||
            !requiredScalar(policy, "yaw_gain",
                            config.recovery_policy.yaw_gain, error) ||
            !requiredScalar(policy, "linear_velocity_gain",
                            config.recovery_policy.linear_velocity_gain,
                            error) ||
            !requiredScalar(policy, "angular_velocity_gain",
                            config.recovery_policy.angular_velocity_gain,
                            error) ||
            !requiredScalar(policy, "max_residual_v",
                            config.recovery_policy.max_residual_v, error) ||
            !requiredScalar(policy, "max_residual_omega",
                            config.recovery_policy.max_residual_omega,
                            error)) {
            return false;
        }
        config.recovery_policy.published_linear_min =
            config.published_linear_min;
        config.recovery_policy.published_linear_max =
            config.published_linear_max;
        config.recovery_policy.published_angular_min =
            config.published_angular_min;
        config.recovery_policy.published_angular_max =
            config.published_angular_max;
        const YAML::Node label = root["label"];
        if (!requiredScalar(label, "maximum_path_position_error_m",
                            config.label.maximum_path_position_error_m,
                            error) ||
            !requiredScalar(label, "maximum_path_yaw_error_rad",
                            config.label.maximum_path_yaw_error_rad, error) ||
            !requiredScalar(label, "maximum_external_height_m",
                            config.label.maximum_external_height_m, error) ||
            !requiredScalar(label, "terminal_position_error_m",
                            config.label.terminal_position_error_m, error) ||
            !requiredScalar(label, "terminal_yaw_error_rad",
                            config.label.terminal_yaw_error_rad, error) ||
            !requiredScalar(label, "terminal_v_abs_mps",
                            config.label.terminal_v_abs_mps, error) ||
            !requiredScalar(label, "terminal_omega_abs_radps",
                            config.label.terminal_omega_abs_radps, error) ||
            !requiredScalar(label, "terminal_external_height_m",
                            config.label.terminal_external_height_m, error) ||
            !requiredScalar(label, "fixed_tail_sec",
                            config.label.fixed_tail_sec, error)) {
            return false;
        }
        const YAML::Node profiles = sampling["profiles"];
        if (!profiles || !profiles.IsSequence()) {
            error = "sampling.profiles must be a sequence";
            return false;
        }
        for (const YAML::Node& node : profiles) {
            RecoveryExcitationProfile profile;
            if (!requiredScalar(node, "profile_id", profile.profile_id,
                                error) ||
                !requiredScalar(node, "pulse_steps", profile.pulse_steps,
                                error) ||
                !requiredScalar(node, "residual_v", profile.residual_v,
                                error) ||
                !requiredScalar(node, "residual_omega",
                                profile.residual_omega, error)) {
                return false;
            }
            config.profiles.push_back(profile);
        }
    } catch (const std::exception& exception) {
        error = std::string("failed to parse recovery sampling config: ") +
            exception.what();
        return false;
    }
    return validateRecoveryRolloutSamplingConfig(config, error);
}

bool PhaseRejoinRecoveryRolloutSampler::configure(
    const IndependentPlantConfig& plant_config,
    const RecoveryRolloutSamplingConfig& sampling_config,
    const SloshModelParams& controller_slosh_params,
    double nominal_dt_sec,
    const std::vector<PhaseNominalSample>& nominal_samples,
    std::string& error) {
    configured_ = false;
    error.clear();
    if (!validateIndependentPlantConfig(plant_config, error) ||
        !validateRecoveryRolloutSamplingConfig(sampling_config, error)) {
        return false;
    }
    SloshDynamics observer;
    if (!observer.configure(controller_slosh_params)) {
        error = "invalid controller liquid observer model";
        return false;
    }
    const double control_dt = 1.0 / plant_config.experiment_control_rate_hz;
    if (!finite(nominal_dt_sec) || nominal_dt_sec <= 1.0e-4 ||
        std::abs(control_dt - nominal_dt_sec) >
            std::max(1.0e-8, nominal_dt_sec * 1.0e-6) ||
        nominal_samples.size() < 2) {
        error = "nominal/Plant control timing mismatch";
        return false;
    }
    const int minimum_warmup_steps = static_cast<int>(std::ceil(
        (std::max(plant_config.linear.delay_sec,
                  plant_config.angular.delay_sec) +
         plant_config.command_transport_jitter_limit_sec) /
        nominal_dt_sec)) + 1;
    if (sampling_config.warmup_steps < minimum_warmup_steps) {
        error = "warmup does not cover the external dual-channel delay";
        return false;
    }
    for (std::size_t index = 0; index < nominal_samples.size(); ++index) {
        if (!validNominalSample(
                nominal_samples[index], index,
                static_cast<double>(index) * nominal_dt_sec,
                nominal_dt_sec, sampling_config, error)) {
            return false;
        }
    }
    plant_config_ = plant_config;
    sampling_config_ = sampling_config;
    controller_slosh_params_ = controller_slosh_params;
    nominal_dt_sec_ = nominal_dt_sec;
    nominal_samples_ = nominal_samples;
    configured_ = true;
    return true;
}

RecoverySeedSampleResult
PhaseRejoinRecoveryRolloutSampler::sampleSeed(
    const std::string& split,
    std::uint32_t seed,
    std::size_t phase_begin,
    std::size_t phase_end_inclusive) const {
    RecoverySeedSampleResult result;
    if (!configured_) {
        result.status = "SAMPLER_NOT_CONFIGURED";
        return result;
    }
    if (!validSplit(split) || phase_begin > phase_end_inclusive ||
        phase_end_inclusive >= nominal_samples_.size()) {
        result.status = "INVALID_SPLIT_OR_PHASE_RANGE";
        return result;
    }

    SloshDynamics observer;
    if (!observer.configure(controller_slosh_params_)) {
        result.status = "OBSERVER_CONFIGURATION_FAILED";
        return result;
    }
    BoundedTrackingRecoveryPolicy recovery_policy;
    std::string error;
    if (!recovery_policy.configure(
            sampling_config_.recovery_policy, error)) {
        result.status = "RECOVERY_POLICY_CONFIGURATION_FAILED: " + error;
        return result;
    }
    RuntimeState initial;
    if (!initial.plant.configure(plant_config_, error)) {
        result.status = "PLANT_CONFIGURATION_FAILED: " + error;
        return result;
    }
    IndependentPlantInitialPose pose;
    pose.x = nominal_samples_.front().x;
    pose.y = nominal_samples_.front().y;
    pose.yaw = nominal_samples_.front().yaw;
    if (!initial.plant.reset(seed, pose, error)) {
        result.status = "PLANT_RESET_FAILED: " + error;
        return result;
    }
    initial.linear_pending.assign(kLinearPendingCount, 0.0);
    initial.angular_pending.assign(kAngularPendingCount, 0.0);

    const int warmup_steps = sampling_config_.warmup_steps;
    std::vector<RuntimeState> warmup_snapshots;
    warmup_snapshots.reserve(static_cast<std::size_t>(warmup_steps));
    RuntimeState baseline = initial;
    for (int stage = -warmup_steps; stage < 0; ++stage) {
        warmup_snapshots.push_back(baseline);
        IndependentPlantCommand zero;
        if (!publishAndAdvance(
                baseline, zero,
                stageTimeSec(stage + 1, warmup_steps, nominal_dt_sec_,
                             nominal_samples_),
                observer, plant_config_.integration_dt_sec,
                std::function<void(const IndependentPlantState&)>(),
                error)) {
            result.status = "WARMUP_FAILED: " + error;
            return result;
        }
    }

    std::vector<RuntimeState> nominal_snapshots;
    nominal_snapshots.reserve(nominal_samples_.size());
    nominal_snapshots.push_back(baseline);
    for (std::size_t phase = 0; phase + 1 < nominal_samples_.size(); ++phase) {
        IndependentPlantCommand tracked;
        if (!trackingRecoveryCommand(
                recovery_policy, nominal_samples_[phase],
                baseline.plant.state(), tracked, error)) {
            result.status = "NOMINAL_TRACKING_POLICY_FAILED: " + error;
            return result;
        }
        if (!publishAndAdvance(
                baseline, tracked,
                stageTimeSec(static_cast<int>(phase + 1), warmup_steps,
                             nominal_dt_sec_, nominal_samples_),
                observer, plant_config_.integration_dt_sec,
                std::function<void(const IndependentPlantState&)>(),
                error)) {
            result.status = "NOMINAL_PREFIX_FAILED: " + error;
            return result;
        }
        nominal_snapshots.push_back(baseline);
    }

    result.rows.reserve(
        (phase_end_inclusive - phase_begin + 1) *
        sampling_config_.profiles.size());
    result.audits.reserve(result.rows.capacity());
    for (std::size_t phase = phase_begin;
         phase <= phase_end_inclusive; ++phase) {
        const PhaseNominalSample& nominal = nominal_samples_[phase];
        for (const RecoveryExcitationProfile& profile :
             sampling_config_.profiles) {
            const int start_stage = static_cast<int>(phase) -
                profile.pulse_steps;
            RuntimeState runtime = snapshotForGlobalStage(
                start_stage, warmup_steps,
                warmup_snapshots, nominal_snapshots);
            bool rollout_failed = false;
            for (int stage = start_stage;
                 stage < static_cast<int>(phase); ++stage) {
                IndependentPlantCommand command;
                if (stage >= 0) {
                    const PhaseNominalSample& stage_nominal =
                        nominal_samples_[static_cast<std::size_t>(stage)];
                    if (!trackingRecoveryCommand(
                            recovery_policy, stage_nominal,
                            runtime.plant.state(), command, error)) {
                        rollout_failed = true;
                        break;
                    }
                    const double total_residual_v = clamp(
                        command.linear - stage_nominal.kappa_v +
                            profile.residual_v,
                        -sampling_config_.maximum_candidate_residual_v,
                        sampling_config_.maximum_candidate_residual_v);
                    const double total_residual_omega = clamp(
                        command.angular - stage_nominal.kappa_omega +
                            profile.residual_omega,
                        -sampling_config_.maximum_candidate_residual_omega,
                        sampling_config_.maximum_candidate_residual_omega);
                    command.linear = stage_nominal.kappa_v + total_residual_v;
                    command.angular =
                        stage_nominal.kappa_omega + total_residual_omega;
                } else {
                    command.linear = profile.residual_v;
                    command.angular = profile.residual_omega;
                }
                command.linear = clamp(
                    command.linear,
                    sampling_config_.published_linear_min,
                    sampling_config_.published_linear_max);
                command.angular = clamp(
                    command.angular,
                    sampling_config_.published_angular_min,
                    sampling_config_.published_angular_max);
                if (!publishAndAdvance(
                        runtime, command,
                        stageTimeSec(stage + 1, warmup_steps,
                                     nominal_dt_sec_, nominal_samples_),
                        observer, plant_config_.integration_dt_sec,
                        std::function<void(const IndependentPlantState&)>(),
                        error)) {
                    rollout_failed = true;
                    break;
                }
            }
            if (rollout_failed) {
                result.status = "CANDIDATE_PULSE_FAILED: " + error;
                result.rows.clear();
                result.audits.clear();
                return result;
            }

            const IndependentPlantState snapshot = runtime.plant.state();
            RecoveryDatasetRow row;
            row.split = split;
            row.rollout_id = rolloutId(
                split, seed, phase, profile.profile_id);
            row.seed = seed;
            row.phase_index = phase;
            row.state_errors = {{
                snapshot.x - nominal.x,
                snapshot.y - nominal.y,
                wrapAngle(snapshot.yaw - nominal.yaw),
                snapshot.v - nominal.v,
                snapshot.omega - nominal.omega,
                runtime.observed_slosh.eta_x - nominal.eta_x,
                runtime.observed_slosh.eta_x_dot - nominal.eta_x_dot,
                runtime.observed_slosh.eta_y - nominal.eta_y,
                runtime.observed_slosh.eta_y_dot - nominal.eta_y_dot,
            }};
            std::size_t execution_index = 0;
            row.execution_errors[execution_index++] =
                snapshot.v -
                nominal.augmented_execution.linear.actuator_output;
            row.execution_errors[execution_index++] =
                snapshot.omega -
                nominal.augmented_execution.angular.actuator_output;
            for (std::size_t index = 0; index < kLinearPendingCount; ++index) {
                row.execution_errors[execution_index++] =
                    runtime.linear_pending[index] -
                    nominal.augmented_execution.linear.pending_commands[index];
            }
            for (std::size_t index = 0; index < kAngularPendingCount; ++index) {
                row.execution_errors[execution_index++] =
                    runtime.angular_pending[index] -
                    nominal.augmented_execution.angular.pending_commands[index];
            }
            if (execution_index != kExecutionErrorCount ||
                row.state_errors.size() != kStateErrorCount) {
                result.status = "INTERNAL_FEATURE_LAYOUT_MISMATCH";
                result.rows.clear();
                result.audits.clear();
                return result;
            }

            LabelAccumulator accumulator;
            recordAgainstSegment(
                accumulator, snapshot, nominal, nominal,
                snapshot.time_sec, snapshot.time_sec);
            for (std::size_t tail_phase = phase;
                 tail_phase + 1 < nominal_samples_.size(); ++tail_phase) {
                const double segment_start = runtime.plant.state().time_sec;
                const double segment_end = stageTimeSec(
                    static_cast<int>(tail_phase + 1), warmup_steps,
                    nominal_dt_sec_, nominal_samples_);
                const PhaseNominalSample& from = nominal_samples_[tail_phase];
                const PhaseNominalSample& to = nominal_samples_[tail_phase + 1];
                const auto recorder = [
                    &accumulator, &from, &to,
                    segment_start, segment_end](
                        const IndependentPlantState& state) {
                    recordAgainstSegment(
                        accumulator, state, from, to,
                        segment_start, segment_end);
                };
                IndependentPlantCommand recovery_command;
                if (!trackingRecoveryCommand(
                        recovery_policy, from, runtime.plant.state(),
                        recovery_command, error)) {
                    rollout_failed = true;
                    break;
                }
                if (!publishAndAdvance(
                        runtime, recovery_command, segment_end,
                        observer, plant_config_.integration_dt_sec,
                        recorder, error)) {
                    rollout_failed = true;
                    break;
                }
            }
            if (rollout_failed) {
                result.status = "RECOVERY_TAIL_FAILED: " + error;
                result.rows.clear();
                result.audits.clear();
                return result;
            }

            const PhaseNominalSample& final_nominal = nominal_samples_.back();
            const double physical_tail_duration =
                std::max(plant_config_.linear.delay_sec,
                         plant_config_.angular.delay_sec) +
                plant_config_.command_transport_jitter_limit_sec +
                sampling_config_.label.fixed_tail_sec;
            const int fixed_tail_steps = std::max(
                1, static_cast<int>(std::ceil(
                    physical_tail_duration / nominal_dt_sec_)));
            for (int step = 0; step < fixed_tail_steps; ++step) {
                const double segment_start = runtime.plant.state().time_sec;
                const double segment_end = segment_start + nominal_dt_sec_;
                const auto recorder = [
                    &accumulator, &final_nominal,
                    segment_start, segment_end](
                        const IndependentPlantState& state) {
                    recordAgainstSegment(
                        accumulator, state, final_nominal, final_nominal,
                        segment_start, segment_end);
                };
                IndependentPlantCommand final_command;
                if (!trackingRecoveryCommand(
                        recovery_policy, final_nominal,
                        runtime.plant.state(), final_command, error)) {
                    rollout_failed = true;
                    break;
                }
                if (!publishAndAdvance(
                        runtime, final_command, segment_end,
                        observer, plant_config_.integration_dt_sec,
                        recorder, error)) {
                    rollout_failed = true;
                    break;
                }
            }
            if (rollout_failed) {
                result.status = "FIXED_RECOVERY_TAIL_FAILED: " + error;
                result.rows.clear();
                result.audits.clear();
                return result;
            }

            const IndependentPlantState& terminal = runtime.plant.state();
            RecoveryRolloutAudit audit;
            audit.split = split;
            audit.rollout_id = row.rollout_id;
            audit.profile_id = profile.profile_id;
            audit.seed = seed;
            audit.phase_index = phase;
            audit.snapshot_time_sec = snapshot.time_sec;
            audit.final_time_sec = terminal.time_sec;
            audit.maximum_path_position_error_m =
                accumulator.max_position_error;
            audit.maximum_path_yaw_error_rad = accumulator.max_yaw_error;
            audit.maximum_external_height_m = accumulator.max_external_height;
            audit.terminal_position_error_m = std::hypot(
                terminal.x - final_nominal.x,
                terminal.y - final_nominal.y);
            audit.terminal_yaw_error_rad = std::abs(
                wrapAngle(terminal.yaw - final_nominal.yaw));
            audit.terminal_v_abs_mps = std::abs(terminal.v);
            audit.terminal_omega_abs_radps = std::abs(terminal.omega);
            audit.terminal_external_height_m = terminal.true_height_m;
            const RecoveryRolloutLabelContract& contract =
                sampling_config_.label;
            audit.path_position_passed =
                audit.maximum_path_position_error_m <=
                    contract.maximum_path_position_error_m + kValueEpsilon;
            audit.path_yaw_passed =
                audit.maximum_path_yaw_error_rad <=
                    contract.maximum_path_yaw_error_rad + kValueEpsilon;
            audit.external_height_passed =
                audit.maximum_external_height_m <=
                    contract.maximum_external_height_m + kValueEpsilon;
            audit.terminal_position_passed =
                audit.terminal_position_error_m <=
                    contract.terminal_position_error_m + kValueEpsilon;
            audit.terminal_yaw_passed =
                audit.terminal_yaw_error_rad <=
                    contract.terminal_yaw_error_rad + kValueEpsilon;
            audit.terminal_velocity_passed =
                audit.terminal_v_abs_mps <=
                    contract.terminal_v_abs_mps + kValueEpsilon &&
                audit.terminal_omega_abs_radps <=
                    contract.terminal_omega_abs_radps + kValueEpsilon;
            audit.terminal_external_height_passed =
                audit.terminal_external_height_m <=
                    contract.terminal_external_height_m + kValueEpsilon;
            audit.recovered =
                audit.path_position_passed && audit.path_yaw_passed &&
                audit.external_height_passed &&
                audit.terminal_position_passed && audit.terminal_yaw_passed &&
                audit.terminal_velocity_passed &&
                audit.terminal_external_height_passed;
            row.recovered = audit.recovered;
            result.rows.push_back(row);
            result.audits.push_back(audit);
        }
    }

    result.valid = true;
    result.status = "OK";
    return result;
}

}  // namespace simulation
}  // namespace spmpc_local_planner
