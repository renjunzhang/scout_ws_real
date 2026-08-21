#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <string>

namespace spmpc_local_planner {
namespace simulation {

struct IndependentChannelParams {
    double delay_sec = 0.0;
    double time_constant_sec = 0.0;
    double positive_gain = 1.0;
    double negative_gain = 1.0;
    double deadzone = 0.0;
    double output_min = -1.0;
    double output_max = 1.0;
};

struct IndependentLiquidParams {
    double container_radius_m = 0.0;
    double liquid_height_m = 0.0;
    double primary_damping_ratio = 0.0;
    double primary_frequency_scale = 1.0;
    double longitudinal_input_gain = 1.0;
    double lateral_input_gain = 1.0;
    double primary_height_scale = 1.0;
    double second_mode_frequency_ratio = 0.0;
    double second_mode_damping_ratio = 0.0;
    double second_mode_input_gain = 0.0;
    double second_mode_height_scale = 0.0;
    double height_noise_std_m = 0.0;
};

struct IndependentPlantConfig {
    std::string schema;
    std::string freeze_id;
    std::string status;
    bool simulation_only = false;
    bool formal_robot_release = true;
    bool real_robot_enforce_allowed = true;
    double integration_dt_sec = 0.0;
    // Default seed for direct library users. Campaign runners may explicitly
    // reset to a trial seed after configure().
    std::uint32_t default_seed = 0;
    IndependentChannelParams linear;
    IndependentChannelParams angular;
    double command_transport_jitter_std_sec = 0.0;
    double command_transport_jitter_limit_sec = 0.0;
    double linear_process_acceleration_std_mps2 = 0.0;
    double angular_process_acceleration_std_radps2 = 0.0;
    IndependentLiquidParams liquid;
    double experiment_control_rate_hz = 0.0;
    double experiment_fixed_tail_sec = 0.0;
};

struct IndependentPlantCommand {
    double linear = 0.0;
    double angular = 0.0;
};

struct IndependentPlantState {
    bool valid = false;
    double time_sec = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double acceleration = 0.0;
    double lateral_acceleration = 0.0;
    double primary_eta_x = 0.0;
    double primary_eta_x_dot = 0.0;
    double primary_eta_y = 0.0;
    double primary_eta_y_dot = 0.0;
    double second_eta_x = 0.0;
    double second_eta_x_dot = 0.0;
    double second_eta_y = 0.0;
    double second_eta_y_dot = 0.0;
    double true_height_m = 0.0;
    double measured_height_m = 0.0;
};

// Simulation-only reset image.  Formal trials use it to start every paired
// condition from the same frozen path pose without granting the controller a
// runtime state-injection port.  Liquid and actuator states intentionally
// start settled; recovery perturbations must be introduced by published
// commands or a separately declared simulation disturbance.
struct IndependentPlantInitialPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct ScheduledChannelCommand {
    double effective_time_sec = 0.0;
    double command = 0.0;
};

// Transaction receipt for one accepted command publication.  The effective
// epochs are part of the simulation evidence contract: they expose the exact
// per-channel delay+jitter event used by the plant rather than requiring an
// analyst to infer it from control-rate samples.
struct IndependentPlantPublishReceipt {
    bool accepted = false;
    double publish_time_sec = 0.0;
    double linear_transport_jitter_sec = 0.0;
    double angular_transport_jitter_sec = 0.0;
    double linear_effective_time_sec = 0.0;
    double angular_effective_time_sec = 0.0;
};

// Read-only diagnostics for the exogenous disturbance clock.  They exist so
// tests and experiment logs can prove that command-event step splitting does
// not change which process/measurement disturbance is active at a physical
// time.  Controllers must not consume these values.
struct IndependentPlantDisturbanceState {
    std::uint64_t noise_interval_index = 0;
    double linear_acceleration_mps2 = 0.0;
    double angular_acceleration_radps2 = 0.0;
    double height_noise_m = 0.0;
};

bool loadIndependentPlantConfig(const std::string& path,
                                IndependentPlantConfig& config,
                                std::string& error);

bool validateIndependentPlantConfig(const IndependentPlantConfig& config,
                                    std::string& error);

class IndependentScoutLiquidPlant {
public:
    bool configure(const IndependentPlantConfig& config, std::string& error);
    bool reset(std::uint32_t seed, std::string& error);
    bool reset(std::uint32_t seed,
               const IndependentPlantInitialPose& initial_pose,
               std::string& error);
    bool publishCommand(double publish_time_sec,
                        const IndependentPlantCommand& command,
                        std::string& error);
    bool publishCommand(double publish_time_sec,
                        const IndependentPlantCommand& command,
                        IndependentPlantPublishReceipt& receipt,
                        std::string& error);
    bool advanceTo(double target_time_sec, std::string& error);

    const IndependentPlantConfig& config() const { return config_; }
    const IndependentPlantState& state() const { return state_; }
    IndependentPlantCommand activeDelayedCommand() const;
    IndependentPlantDisturbanceState disturbanceState() const;

private:
    struct ModalAxisState {
        double eta = 0.0;
        double eta_dot = 0.0;
    };

    static double mappedTarget(double command,
                               const IndependentChannelParams& params);
    static double normalizeYaw(double yaw);
    static double modalFrequency(double container_radius_m,
                                 double liquid_height_m);
    static double modalHeightCoefficient(double container_radius_m,
                                         double liquid_height_m);
    static void integrateModalAxis(ModalAxisState& state,
                                   double frequency,
                                   double damping_ratio,
                                   double input_gain,
                                   double acceleration,
                                   double dt_sec);

    static void seedStream(std::mt19937& generator,
                           std::uint32_t seed,
                           std::uint32_t stream_id);
    static double normalSample(std::mt19937& generator);
    double boundedJitter(std::mt19937& generator);
    double nextNoiseBoundarySec() const;
    void advanceNoiseInterval();
    bool step(double dt_sec, std::string& error);
    void activateDueCommands(double next_time_sec);
    void updateLiquid(double acceleration,
                      double lateral_acceleration,
                      double dt_sec);
    void updatePublicState();

    IndependentPlantConfig config_;
    IndependentPlantState state_;
    std::deque<ScheduledChannelCommand> linear_queue_;
    std::deque<ScheduledChannelCommand> angular_queue_;
    double active_linear_command_ = 0.0;
    double active_angular_command_ = 0.0;
    double last_publish_time_sec_ = -1.0;
    ModalAxisState primary_x_;
    ModalAxisState primary_y_;
    ModalAxisState second_x_;
    ModalAxisState second_y_;
    double primary_frequency_ = 0.0;
    double primary_height_coefficient_ = 0.0;
    std::mt19937 linear_jitter_rng_;
    std::mt19937 angular_jitter_rng_;
    std::mt19937 linear_process_rng_;
    std::mt19937 angular_process_rng_;
    std::mt19937 height_noise_rng_;
    std::uint64_t noise_interval_index_ = 0;
    double linear_process_acceleration_mps2_ = 0.0;
    double angular_process_acceleration_radps2_ = 0.0;
    double height_noise_m_ = 0.0;
    bool configured_ = false;
};

}  // namespace simulation
}  // namespace spmpc_local_planner
