#include "spmpc_local_planner/simulation/independent_scout_liquid_plant.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace spmpc_local_planner {
namespace simulation {
namespace {

constexpr double kGravity = 9.81;
constexpr double kFirstModalRoot = 1.8412;
constexpr double kEpsilon = 1.0e-12;

bool finite(double value) {
    return std::isfinite(value);
}

bool validChannel(const IndependentChannelParams& params) {
    return finite(params.delay_sec) && params.delay_sec >= 0.0 &&
        finite(params.time_constant_sec) && params.time_constant_sec >= 0.0 &&
        finite(params.positive_gain) && params.positive_gain > 0.0 &&
        finite(params.negative_gain) && params.negative_gain > 0.0 &&
        finite(params.deadzone) && params.deadzone >= 0.0 &&
        finite(params.output_min) && finite(params.output_max) &&
        params.output_min < params.output_max &&
        params.deadzone < std::max(
            std::abs(params.output_min), std::abs(params.output_max));
}

IndependentChannelParams loadChannel(const YAML::Node& node) {
    IndependentChannelParams params;
    params.delay_sec = node["delay_sec"].as<double>();
    params.time_constant_sec = node["time_constant_sec"].as<double>();
    params.positive_gain = node["positive_gain"].as<double>();
    params.negative_gain = node["negative_gain"].as<double>();
    params.deadzone = node["deadzone"].as<double>();
    params.output_min = node["output_min"].as<double>();
    params.output_max = node["output_max"].as<double>();
    return params;
}

}  // namespace

bool validateIndependentPlantConfig(const IndependentPlantConfig& config,
                                    std::string& error) {
    error.clear();
    if (config.schema != "spmpc_independent_simulation_config_v1") {
        error = "unsupported independent simulation schema";
    } else if (config.freeze_id.empty()) {
        error = "simulation freeze_id is empty";
    } else if (config.status != "development_candidate_unbound" &&
               config.status != "formal_simulation_release") {
        error = "independent simulation status is neither development nor formal simulation release";
    } else if (!config.simulation_only || config.formal_robot_release ||
               config.real_robot_enforce_allowed) {
        error = "simulation config attempts to authorize a physical release";
    } else if (!finite(config.integration_dt_sec) ||
               config.integration_dt_sec < 1.0e-4 ||
               config.integration_dt_sec > 0.01) {
        error = "invalid independent plant integration step";
    } else if (!validChannel(config.linear) ||
               !validChannel(config.angular)) {
        error = "invalid independent execution channel";
    } else if (!finite(config.command_transport_jitter_std_sec) ||
               config.command_transport_jitter_std_sec < 0.0 ||
               !finite(config.command_transport_jitter_limit_sec) ||
               config.command_transport_jitter_limit_sec < 0.0 ||
               config.command_transport_jitter_std_sec >
                   config.command_transport_jitter_limit_sec + kEpsilon) {
        error = "invalid command transport jitter";
    } else if (config.command_transport_jitter_limit_sec >
                   std::min(config.linear.delay_sec,
                            config.angular.delay_sec) + kEpsilon) {
        error = "transport jitter can produce a negative channel delay";
    } else if (!finite(config.linear_process_acceleration_std_mps2) ||
               config.linear_process_acceleration_std_mps2 < 0.0 ||
               !finite(config.angular_process_acceleration_std_radps2) ||
               config.angular_process_acceleration_std_radps2 < 0.0) {
        error = "invalid execution process noise";
    } else if ((config.linear.time_constant_sec <= kEpsilon &&
                config.linear_process_acceleration_std_mps2 > 0.0) ||
               (config.angular.time_constant_sec <= kEpsilon &&
                config.angular_process_acceleration_std_radps2 > 0.0)) {
        error = "zero-time-constant channel cannot consume process noise";
    } else if (!finite(config.experiment_control_rate_hz) ||
               config.experiment_control_rate_hz <= 0.0 ||
               config.experiment_control_rate_hz > 1000.0 ||
               !finite(config.experiment_fixed_tail_sec) ||
               config.experiment_fixed_tail_sec <= 0.0) {
        error = "invalid independent simulation experiment timing";
    }
    const IndependentLiquidParams& liquid = config.liquid;
    if (error.empty() &&
        (!finite(liquid.container_radius_m) ||
         liquid.container_radius_m <= 0.0 ||
         !finite(liquid.liquid_height_m) || liquid.liquid_height_m <= 0.0 ||
         !finite(liquid.primary_damping_ratio) ||
         liquid.primary_damping_ratio <= 0.0 ||
         !finite(liquid.primary_frequency_scale) ||
         liquid.primary_frequency_scale <= 0.0 ||
         !finite(liquid.longitudinal_input_gain) ||
         liquid.longitudinal_input_gain <= 0.0 ||
         !finite(liquid.lateral_input_gain) ||
         liquid.lateral_input_gain <= 0.0 ||
         !finite(liquid.primary_height_scale) ||
         liquid.primary_height_scale <= 0.0 ||
         !finite(liquid.second_mode_frequency_ratio) ||
         liquid.second_mode_frequency_ratio <= 1.0 ||
         !finite(liquid.second_mode_damping_ratio) ||
         liquid.second_mode_damping_ratio <= 0.0 ||
         !finite(liquid.second_mode_input_gain) ||
         liquid.second_mode_input_gain < 0.0 ||
         !finite(liquid.second_mode_height_scale) ||
         liquid.second_mode_height_scale < 0.0 ||
         !finite(liquid.height_noise_std_m) ||
         liquid.height_noise_std_m < 0.0)) {
        error = "invalid independent liquid model";
    }
    return error.empty();
}

bool loadIndependentPlantConfig(const std::string& path,
                                IndependentPlantConfig& config,
                                std::string& error) {
    config = IndependentPlantConfig{};
    error.clear();
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node scope = root["scope"];
        const YAML::Node plant = root["external_plant"];
        const YAML::Node liquid = plant["liquid"];
        config.schema = root["schema"].as<std::string>();
        config.freeze_id = root["freeze_id"].as<std::string>();
        config.status = root["status"].as<std::string>();
        config.simulation_only = scope["simulation_only"].as<bool>();
        config.formal_robot_release =
            scope["formal_robot_release"].as<bool>();
        config.real_robot_enforce_allowed =
            scope["real_robot_enforce_allowed"].as<bool>();
        config.integration_dt_sec =
            plant["integration_dt_sec"].as<double>();
        config.default_seed = plant["default_seed"].as<std::uint32_t>();
        config.linear = loadChannel(plant["linear"]);
        config.angular = loadChannel(plant["angular"]);
        config.command_transport_jitter_std_sec =
            plant["command_transport_jitter_std_sec"].as<double>();
        config.command_transport_jitter_limit_sec =
            plant["command_transport_jitter_limit_sec"].as<double>();
        config.linear_process_acceleration_std_mps2 =
            plant["linear_process_acceleration_std_mps2"].as<double>();
        config.angular_process_acceleration_std_radps2 =
            plant["angular_process_acceleration_std_radps2"].as<double>();
        config.liquid.container_radius_m =
            liquid["container_radius_m"].as<double>();
        config.liquid.liquid_height_m =
            liquid["liquid_height_m"].as<double>();
        config.liquid.primary_damping_ratio =
            liquid["primary_damping_ratio"].as<double>();
        config.liquid.primary_frequency_scale =
            liquid["primary_frequency_scale"].as<double>();
        config.liquid.longitudinal_input_gain =
            liquid["longitudinal_input_gain"].as<double>();
        config.liquid.lateral_input_gain =
            liquid["lateral_input_gain"].as<double>();
        config.liquid.primary_height_scale =
            liquid["primary_height_scale"].as<double>();
        config.liquid.second_mode_frequency_ratio =
            liquid["second_mode_frequency_ratio"].as<double>();
        config.liquid.second_mode_damping_ratio =
            liquid["second_mode_damping_ratio"].as<double>();
        config.liquid.second_mode_input_gain =
            liquid["second_mode_input_gain"].as<double>();
        config.liquid.second_mode_height_scale =
            liquid["second_mode_height_scale"].as<double>();
        config.liquid.height_noise_std_m =
            liquid["height_noise_std_m"].as<double>();
        const YAML::Node experiment = root["experiment"];
        config.experiment_control_rate_hz =
            experiment["control_rate_hz"].as<double>();
        config.experiment_fixed_tail_sec =
            experiment["fixed_tail_sec"].as<double>();
    } catch (const std::exception& exception) {
        error = std::string("failed to parse independent simulation config: ") +
            exception.what();
        return false;
    }
    return validateIndependentPlantConfig(config, error);
}

bool IndependentScoutLiquidPlant::configure(
    const IndependentPlantConfig& config,
    std::string& error) {
    configured_ = false;
    if (!validateIndependentPlantConfig(config, error)) {
        return false;
    }
    config_ = config;
    primary_frequency_ = modalFrequency(
        config.liquid.container_radius_m,
        config.liquid.liquid_height_m) *
        config.liquid.primary_frequency_scale;
    primary_height_coefficient_ = modalHeightCoefficient(
        config.liquid.container_radius_m,
        config.liquid.liquid_height_m);
    if (!finite(primary_frequency_) || primary_frequency_ <= 0.0 ||
        !finite(primary_height_coefficient_) ||
        primary_height_coefficient_ <= 0.0) {
        error = "failed to resolve independent liquid modal constants";
        return false;
    }
    configured_ = true;
    return reset(config.default_seed, error);
}

bool IndependentScoutLiquidPlant::reset(
    std::uint32_t seed,
    std::string& error) {
    return reset(seed, IndependentPlantInitialPose{}, error);
}

bool IndependentScoutLiquidPlant::reset(
    std::uint32_t seed,
    const IndependentPlantInitialPose& initial_pose,
    std::string& error) {
    error.clear();
    if (!configured_) {
        error = "independent plant is not configured";
        return false;
    }
    if (!finite(initial_pose.x) || !finite(initial_pose.y) ||
        !finite(initial_pose.yaw)) {
        error = "independent plant initial pose is non-finite";
        return false;
    }
    seedStream(linear_jitter_rng_, seed, 1u);
    seedStream(angular_jitter_rng_, seed, 2u);
    seedStream(linear_process_rng_, seed, 3u);
    seedStream(angular_process_rng_, seed, 4u);
    seedStream(height_noise_rng_, seed, 5u);
    state_ = IndependentPlantState{};
    state_.valid = true;
    state_.x = initial_pose.x;
    state_.y = initial_pose.y;
    state_.yaw = normalizeYaw(initial_pose.yaw);
    linear_queue_.clear();
    angular_queue_.clear();
    active_linear_command_ = 0.0;
    active_angular_command_ = 0.0;
    last_publish_time_sec_ = -1.0;
    primary_x_ = ModalAxisState{};
    primary_y_ = ModalAxisState{};
    second_x_ = ModalAxisState{};
    second_y_ = ModalAxisState{};
    noise_interval_index_ = 0;
    linear_process_acceleration_mps2_ = normalSample(linear_process_rng_) *
        config_.linear_process_acceleration_std_mps2;
    angular_process_acceleration_radps2_ = normalSample(angular_process_rng_) *
        config_.angular_process_acceleration_std_radps2;
    height_noise_m_ = normalSample(height_noise_rng_) *
        config_.liquid.height_noise_std_m;
    updatePublicState();
    return true;
}

bool IndependentScoutLiquidPlant::publishCommand(
    double publish_time_sec,
    const IndependentPlantCommand& command,
    std::string& error) {
    IndependentPlantPublishReceipt receipt;
    return publishCommand(publish_time_sec, command, receipt, error);
}

bool IndependentScoutLiquidPlant::publishCommand(
    double publish_time_sec,
    const IndependentPlantCommand& command,
    IndependentPlantPublishReceipt& receipt,
    std::string& error) {
    receipt = IndependentPlantPublishReceipt{};
    receipt.publish_time_sec = publish_time_sec;
    error.clear();
    if (!configured_ || !finite(publish_time_sec) ||
        publish_time_sec + kEpsilon < state_.time_sec ||
        publish_time_sec <= last_publish_time_sec_ ||
        !finite(command.linear) || !finite(command.angular)) {
        error = "invalid or non-monotonic independent plant command";
        return false;
    }
    // Sampling is transactional.  A rejected publication must not perturb the
    // random realization observed by the next accepted command.
    std::mt19937 candidate_linear_rng = linear_jitter_rng_;
    std::mt19937 candidate_angular_rng = angular_jitter_rng_;
    receipt.linear_transport_jitter_sec =
        boundedJitter(candidate_linear_rng);
    receipt.angular_transport_jitter_sec =
        boundedJitter(candidate_angular_rng);
    ScheduledChannelCommand linear;
    linear.effective_time_sec = publish_time_sec +
        config_.linear.delay_sec + receipt.linear_transport_jitter_sec;
    linear.command = std::max(
        config_.linear.output_min,
        std::min(config_.linear.output_max, command.linear));
    ScheduledChannelCommand angular;
    angular.effective_time_sec = publish_time_sec +
        config_.angular.delay_sec + receipt.angular_transport_jitter_sec;
    angular.command = std::max(
        config_.angular.output_min,
        std::min(config_.angular.output_max, command.angular));
    receipt.linear_effective_time_sec = linear.effective_time_sec;
    receipt.angular_effective_time_sec = angular.effective_time_sec;
    if ((!linear_queue_.empty() &&
         linear.effective_time_sec <= linear_queue_.back().effective_time_sec) ||
        (!angular_queue_.empty() &&
         angular.effective_time_sec <= angular_queue_.back().effective_time_sec)) {
        error = "transport jitter reordered published commands";
        return false;
    }
    linear_jitter_rng_ = candidate_linear_rng;
    angular_jitter_rng_ = candidate_angular_rng;
    linear_queue_.push_back(linear);
    angular_queue_.push_back(angular);
    last_publish_time_sec_ = publish_time_sec;
    receipt.accepted = true;
    return true;
}

bool IndependentScoutLiquidPlant::advanceTo(
    double target_time_sec,
    std::string& error) {
    error.clear();
    if (!configured_ || !finite(target_time_sec) ||
        target_time_sec + kEpsilon < state_.time_sec) {
        error = "invalid independent plant target time";
        return false;
    }
    while (state_.time_sec + kEpsilon < target_time_sec) {
        activateDueCommands(state_.time_sec + kEpsilon);
        double next_time = std::min(
            target_time_sec, state_.time_sec + config_.integration_dt_sec);
        next_time = std::min(next_time, nextNoiseBoundarySec());
        if (!linear_queue_.empty() &&
            linear_queue_.front().effective_time_sec > state_.time_sec) {
            next_time = std::min(
                next_time, linear_queue_.front().effective_time_sec);
        }
        if (!angular_queue_.empty() &&
            angular_queue_.front().effective_time_sec > state_.time_sec) {
            next_time = std::min(
                next_time, angular_queue_.front().effective_time_sec);
        }
        const double dt_sec = next_time - state_.time_sec;
        if (dt_sec <= kEpsilon) {
            activateDueCommands(next_time + kEpsilon);
            continue;
        }
        if (!step(dt_sec, error)) {
            return false;
        }
        while (state_.time_sec + kEpsilon >= nextNoiseBoundarySec()) {
            advanceNoiseInterval();
        }
    }
    activateDueCommands(state_.time_sec + kEpsilon);
    return true;
}

IndependentPlantCommand
IndependentScoutLiquidPlant::activeDelayedCommand() const {
    IndependentPlantCommand command;
    command.linear = active_linear_command_;
    command.angular = active_angular_command_;
    return command;
}

IndependentPlantDisturbanceState
IndependentScoutLiquidPlant::disturbanceState() const {
    IndependentPlantDisturbanceState disturbance;
    disturbance.noise_interval_index = noise_interval_index_;
    disturbance.linear_acceleration_mps2 =
        linear_process_acceleration_mps2_;
    disturbance.angular_acceleration_radps2 =
        angular_process_acceleration_radps2_;
    disturbance.height_noise_m = height_noise_m_;
    return disturbance;
}

double IndependentScoutLiquidPlant::mappedTarget(
    double command,
    const IndependentChannelParams& params) {
    const double magnitude = std::abs(command);
    double mapped = 0.0;
    if (magnitude > params.deadzone) {
        const double gain = command >= 0.0
            ? params.positive_gain
            : params.negative_gain;
        mapped = std::copysign(
            gain * (magnitude - params.deadzone), command);
    }
    return std::max(params.output_min, std::min(params.output_max, mapped));
}

double IndependentScoutLiquidPlant::normalizeYaw(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

double IndependentScoutLiquidPlant::modalFrequency(
    double container_radius_m,
    double liquid_height_m) {
    const double argument =
        kFirstModalRoot * liquid_height_m / container_radius_m;
    return std::sqrt(
        kGravity * kFirstModalRoot / container_radius_m *
        std::tanh(argument));
}

double IndependentScoutLiquidPlant::modalHeightCoefficient(
    double container_radius_m,
    double liquid_height_m) {
    const double argument =
        kFirstModalRoot * liquid_height_m / container_radius_m;
    const double modal_mass_ratio =
        2.0 * container_radius_m * std::tanh(argument) /
        (kFirstModalRoot * liquid_height_m *
         (kFirstModalRoot * kFirstModalRoot - 1.0));
    return 4.0 * liquid_height_m * modal_mass_ratio /
        container_radius_m;
}

void IndependentScoutLiquidPlant::integrateModalAxis(
    ModalAxisState& state,
    double frequency,
    double damping_ratio,
    double input_gain,
    double acceleration,
    double dt_sec) {
    const auto derivative = [=](const ModalAxisState& value) {
        ModalAxisState result;
        result.eta = value.eta_dot;
        result.eta_dot =
            -2.0 * damping_ratio * frequency * value.eta_dot -
            frequency * frequency * value.eta -
            input_gain * acceleration;
        return result;
    };
    const auto add = [](const ModalAxisState& lhs,
                        const ModalAxisState& rhs,
                        double scale) {
        ModalAxisState value;
        value.eta = lhs.eta + scale * rhs.eta;
        value.eta_dot = lhs.eta_dot + scale * rhs.eta_dot;
        return value;
    };
    const ModalAxisState k1 = derivative(state);
    const ModalAxisState k2 = derivative(add(state, k1, 0.5 * dt_sec));
    const ModalAxisState k3 = derivative(add(state, k2, 0.5 * dt_sec));
    const ModalAxisState k4 = derivative(add(state, k3, dt_sec));
    state.eta += dt_sec / 6.0 *
        (k1.eta + 2.0 * k2.eta + 2.0 * k3.eta + k4.eta);
    state.eta_dot += dt_sec / 6.0 *
        (k1.eta_dot + 2.0 * k2.eta_dot +
         2.0 * k3.eta_dot + k4.eta_dot);
}

void IndependentScoutLiquidPlant::seedStream(
    std::mt19937& generator,
    std::uint32_t seed,
    std::uint32_t stream_id) {
    std::seed_seq sequence{seed, stream_id, 0x53504d50u};
    generator.seed(sequence);
}

double IndependentScoutLiquidPlant::boundedJitter(
    std::mt19937& generator) {
    const double raw =
        normalSample(generator) * config_.command_transport_jitter_std_sec;
    return std::max(
        -config_.command_transport_jitter_limit_sec,
        std::min(config_.command_transport_jitter_limit_sec, raw));
}

double IndependentScoutLiquidPlant::normalSample(std::mt19937& generator) {
    std::normal_distribution<double> distribution(0.0, 1.0);
    return distribution(generator);
}

double IndependentScoutLiquidPlant::nextNoiseBoundarySec() const {
    return static_cast<double>(noise_interval_index_ + 1u) *
        config_.integration_dt_sec;
}

void IndependentScoutLiquidPlant::advanceNoiseInterval() {
    ++noise_interval_index_;
    linear_process_acceleration_mps2_ = normalSample(linear_process_rng_) *
        config_.linear_process_acceleration_std_mps2;
    angular_process_acceleration_radps2_ = normalSample(angular_process_rng_) *
        config_.angular_process_acceleration_std_radps2;
    height_noise_m_ = normalSample(height_noise_rng_) *
        config_.liquid.height_noise_std_m;
    // At an exact noise-clock boundary the new interval is active.  Refresh
    // the measured output so its value and disturbanceState() describe the
    // same half-open physical-time interval.
    updatePublicState();
}

bool IndependentScoutLiquidPlant::step(double dt_sec, std::string& error) {
    const double previous_v = state_.v;
    const double previous_omega = state_.omega;
    const double target_v = mappedTarget(
        active_linear_command_, config_.linear);
    const double target_omega = mappedTarget(
        active_angular_command_, config_.angular);
    const auto actuator_step = [dt_sec](
        double current, double target, double time_constant,
        double acceleration_disturbance) {
        if (time_constant <= kEpsilon) return target;
        const double decay = std::exp(-dt_sec / time_constant);
        const double disturbed_target =
            target + time_constant * acceleration_disturbance;
        return disturbed_target + (current - disturbed_target) * decay;
    };
    double next_v = actuator_step(
        previous_v, target_v, config_.linear.time_constant_sec,
        linear_process_acceleration_mps2_);
    double next_omega = actuator_step(
        previous_omega, target_omega, config_.angular.time_constant_sec,
        angular_process_acceleration_radps2_);
    next_v = std::max(
        config_.linear.output_min,
        std::min(config_.linear.output_max, next_v));
    next_omega = std::max(
        config_.angular.output_min,
        std::min(config_.angular.output_max, next_omega));
    const double mean_v = 0.5 * (previous_v + next_v);
    const double mean_omega = 0.5 * (previous_omega + next_omega);
    const double mid_yaw = state_.yaw + 0.5 * mean_omega * dt_sec;
    state_.x += mean_v * std::cos(mid_yaw) * dt_sec;
    state_.y += mean_v * std::sin(mid_yaw) * dt_sec;
    state_.yaw = normalizeYaw(state_.yaw + mean_omega * dt_sec);
    state_.v = next_v;
    state_.omega = next_omega;
    state_.acceleration = (next_v - previous_v) / dt_sec;
    state_.lateral_acceleration = next_v * next_omega;
    updateLiquid(
        state_.acceleration, state_.lateral_acceleration, dt_sec);
    state_.time_sec += dt_sec;
    updatePublicState();
    if (!finite(state_.x) || !finite(state_.y) || !finite(state_.yaw) ||
        !finite(state_.v) || !finite(state_.omega) ||
        !finite(state_.true_height_m)) {
        error = "independent plant produced a non-finite state";
        state_.valid = false;
        return false;
    }
    return true;
}

void IndependentScoutLiquidPlant::activateDueCommands(double next_time_sec) {
    while (!linear_queue_.empty() &&
           linear_queue_.front().effective_time_sec <= next_time_sec) {
        active_linear_command_ = linear_queue_.front().command;
        linear_queue_.pop_front();
    }
    while (!angular_queue_.empty() &&
           angular_queue_.front().effective_time_sec <= next_time_sec) {
        active_angular_command_ = angular_queue_.front().command;
        angular_queue_.pop_front();
    }
}

void IndependentScoutLiquidPlant::updateLiquid(
    double acceleration,
    double lateral_acceleration,
    double dt_sec) {
    const IndependentLiquidParams& params = config_.liquid;
    integrateModalAxis(
        primary_x_, primary_frequency_, params.primary_damping_ratio,
        params.longitudinal_input_gain, acceleration, dt_sec);
    integrateModalAxis(
        primary_y_, primary_frequency_, params.primary_damping_ratio,
        params.lateral_input_gain, lateral_acceleration, dt_sec);
    const double second_frequency =
        primary_frequency_ * params.second_mode_frequency_ratio;
    integrateModalAxis(
        second_x_, second_frequency, params.second_mode_damping_ratio,
        params.second_mode_input_gain * params.longitudinal_input_gain,
        acceleration, dt_sec);
    integrateModalAxis(
        second_y_, second_frequency, params.second_mode_damping_ratio,
        params.second_mode_input_gain * params.lateral_input_gain,
        lateral_acceleration, dt_sec);
}

void IndependentScoutLiquidPlant::updatePublicState() {
    state_.primary_eta_x = primary_x_.eta;
    state_.primary_eta_x_dot = primary_x_.eta_dot;
    state_.primary_eta_y = primary_y_.eta;
    state_.primary_eta_y_dot = primary_y_.eta_dot;
    state_.second_eta_x = second_x_.eta;
    state_.second_eta_x_dot = second_x_.eta_dot;
    state_.second_eta_y = second_y_.eta;
    state_.second_eta_y_dot = second_y_.eta_dot;
    const IndependentLiquidParams& params = config_.liquid;
    const double primary_height = primary_height_coefficient_ *
        std::hypot(primary_x_.eta, primary_y_.eta) *
        params.primary_height_scale;
    const double second_height = primary_height_coefficient_ *
        std::hypot(second_x_.eta, second_y_.eta) *
        params.second_mode_height_scale;
    state_.true_height_m = primary_height + second_height;
    state_.measured_height_m = std::max(
        0.0, state_.true_height_m + height_noise_m_);
}

}  // namespace simulation
}  // namespace spmpc_local_planner
