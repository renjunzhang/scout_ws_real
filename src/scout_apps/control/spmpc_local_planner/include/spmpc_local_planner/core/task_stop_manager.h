#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

namespace spmpc_local_planner {

struct TaskStopParams {
    bool enable = false;
    double residual_height_m = 0.001;
    double stable_hold_sec = 0.3;
    double max_settle_sec = 15.0;
    double max_tail_prediction_sec = 8.0;
    double quiet_v = 0.001;
    double quiet_omega = 0.001;
    double command_zero_tolerance = 1e-6;
    double velocity_cost_weight = 1.0;
};

struct StopCommand {
    bool valid = false;
    double v = 0.0;
    double omega = 0.0;
    double a = 0.0;
    std::string status = "STOP_HISTORY_INVALID";
};

// Uses the truthful command acceleration memory and preserves the discrete jerk
// bound, including release of braking acceleration as the command reaches zero.
StopCommand makeJerkLimitedStopCommand(const ActuatorState& history, double dt,
                                       double a_max, double alpha_max, double jerk_max);

struct StopTailPrediction {
    bool valid = false;
    StopCommand first_command;
    double distance_m = 0.0;
    double duration_sec = 0.0;
    double peak_height_m = 0.0;
    double residual_height_m = 0.0;
    std::string status = "NOT_RUN";
};

struct StopReadiness {
    bool valid = false;
    bool queues_clear = false;
    bool vehicle_stopped = false;
    bool excitation_quiet = false;
    bool liquid_stable = false;
    bool timed_out = false;
    double residual_height_m = 0.0;
    double stable_duration_sec = 0.0;
    double vehicle_stop_time_sec = -1.0;
    double liquid_stable_time_sec = -1.0;
};

class TaskStopManager {
public:
    bool configure(const TaskStopParams& params, const ActuatorModelParams& actuator,
                   const SloshModelParams& liquid, double a_max,
                   double alpha_max, double jerk_max);
    void reset();
    StopTailPrediction predict(const SolverInput& input) const;
    StopReadiness observe(const SolverInput& input, bool position_reached,
                          double stopped_v, double stopped_omega);
private:
    bool queuesClear(const ActuatorState& state) const;
    bool excitationQuiet(const RobotState& robot, const ActuatorState& state) const;
    double residualHeight(const SloshState& liquid) const;
    TaskStopParams params_;
    ActuatorModelParams actuator_;
    SloshDynamics liquid_;
    double a_max_ = 0, alpha_max_ = 0, jerk_max_ = 0;
    bool configured_ = false;
    bool timed_out_ = false;
    bool previous_sample_stable_ = false;
    bool previous_observation_settled_ = false;
    double elapsed_ = 0, stable_duration_ = 0;
    double vehicle_stop_time_ = -1, liquid_stable_time_ = -1;
    double settle_wait_start_ = -1;
    std::int64_t start_epoch_ns_ = 0, last_epoch_ns_ = 0;
    std::int64_t last_raw_robot_epoch_ns_ = 0, last_raw_liquid_epoch_ns_ = 0;
};

}  // namespace spmpc_local_planner
