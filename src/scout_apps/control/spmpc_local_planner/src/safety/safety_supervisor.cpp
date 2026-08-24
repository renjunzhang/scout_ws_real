#include "spmpc_local_planner/safety/safety_supervisor.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

const char* safetyInterventionName(SafetyIntervention intervention) {
    switch (intervention) {
    case SafetyIntervention::TerminalSpin:
        return "TERMINAL_SPIN_FAIL";
    case SafetyIntervention::TrackingProjection:
        return "TRACKING_UNSAFE_PROJECTION";
    case SafetyIntervention::TrackingSpin:
        return "TRACKING_SPIN_FAIL";
    case SafetyIntervention::None:
    default:
        return "NONE";
    }
}

bool SafetySupervisor::configure(const SafetySupervisorConfig& config,
                                 std::string& error) {
    error.clear();
    const bool valid =
        std::isfinite(config.nominal_period_sec) &&
        config.nominal_period_sec > 0.0 &&
        std::isfinite(config.terminal_spin.omega_threshold) &&
        config.terminal_spin.omega_threshold >= 0.0 &&
        std::isfinite(config.terminal_spin.max_duration_sec) &&
        config.terminal_spin.max_duration_sec >= 0.0 &&
        std::isfinite(config.tracking.max_projection_distance_m) &&
        config.tracking.max_projection_distance_m >= 0.0 &&
        std::isfinite(config.tracking.max_projection_duration_sec) &&
        config.tracking.max_projection_duration_sec >= 0.0 &&
        std::isfinite(config.tracking.spin_omega_threshold) &&
        config.tracking.spin_omega_threshold >= 0.0 &&
        std::isfinite(config.tracking.spin_max_linear_speed_mps) &&
        config.tracking.spin_max_linear_speed_mps >= 0.0 &&
        std::isfinite(config.tracking.spin_max_duration_sec) &&
        config.tracking.spin_max_duration_sec >= 0.0;
    if (!valid) {
        error = "invalid safety supervisor period, threshold, or duration";
        return false;
    }
    config_ = config;
    reset();
    return true;
}

void SafetySupervisor::reset() {
    terminal_spin_duration_sec_ = 0.0;
    terminal_spin_latched_ = false;
    tracking_projection_duration_sec_ = 0.0;
    tracking_projection_latched_ = false;
    tracking_spin_duration_sec_ = 0.0;
    tracking_spin_latched_ = false;
}

double SafetySupervisor::validPeriod(double period_sec) const {
    if (!std::isfinite(period_sec) || period_sec <= 1e-6) {
        return config_.nominal_period_sec;
    }
    return std::max(0.0, period_sec);
}

bool SafetySupervisor::updateTerminalSpin(
    const SafetySupervisorInput& input,
    double period_sec) {
    if (!config_.terminal_spin.enable || !input.command_accepted ||
        !input.terminal.terminal_phase || input.terminal.reached) {
        terminal_spin_duration_sec_ = 0.0;
        terminal_spin_latched_ = false;
        return false;
    }

    const bool spinning =
        std::abs(input.robot.omega) > config_.terminal_spin.omega_threshold ||
        std::abs(input.command.angular) > config_.terminal_spin.omega_threshold;
    if (!spinning) {
        terminal_spin_duration_sec_ = 0.0;
        terminal_spin_latched_ = false;
        return false;
    }

    terminal_spin_duration_sec_ += period_sec;
    if (terminal_spin_latched_ ||
        terminal_spin_duration_sec_ >=
            config_.terminal_spin.max_duration_sec) {
        terminal_spin_latched_ = true;
        return true;
    }
    return false;
}

SafetyIntervention SafetySupervisor::updateTracking(
    const SafetySupervisorInput& input,
    bool command_accepted,
    const VelocityCommand& command,
    double period_sec) {
    if (!config_.tracking.enable) {
        tracking_projection_duration_sec_ = 0.0;
        tracking_projection_latched_ = false;
        tracking_spin_duration_sec_ = 0.0;
        tracking_spin_latched_ = false;
        return SafetyIntervention::None;
    }
    if (input.terminal.reached || input.status == "GOAL_REACHED") {
        tracking_projection_duration_sec_ = 0.0;
        tracking_projection_latched_ = false;
        tracking_spin_duration_sec_ = 0.0;
        tracking_spin_latched_ = false;
        return SafetyIntervention::None;
    }
    // Preserve historical latch priority: an existing projection latch wins
    // over an existing spin latch, even on a later failed solver cycle.
    if (tracking_projection_latched_) {
        return SafetyIntervention::TrackingProjection;
    }
    if (tracking_spin_latched_) {
        return SafetyIntervention::TrackingSpin;
    }
    if (!command_accepted) {
        return SafetyIntervention::None;
    }

    const bool projection_valid = input.projection.guarded_valid ||
        input.projection.raw_valid;
    const double projection_distance = input.projection.guarded_valid
        ? input.projection.guarded_distance_m
        : input.projection.raw_distance_m;
    const bool projection_unsafe = config_.tracking.projection_enable &&
        projection_valid && config_.tracking.max_projection_distance_m > 0.0 &&
        projection_distance > config_.tracking.max_projection_distance_m;
    if (projection_unsafe) {
        tracking_projection_duration_sec_ += period_sec;
    } else {
        tracking_projection_duration_sec_ = 0.0;
    }

    const bool tracking_phase = !input.terminal.terminal_phase;
    const bool translating_slowly =
        std::abs(input.robot.v) <=
            config_.tracking.spin_max_linear_speed_mps;
    const bool spinning = config_.tracking.spin_enable && tracking_phase &&
        translating_slowly &&
        (std::abs(input.robot.omega) >
             config_.tracking.spin_omega_threshold ||
         std::abs(command.angular) >
             config_.tracking.spin_omega_threshold);
    if (spinning) {
        tracking_spin_duration_sec_ += period_sec;
    } else {
        tracking_spin_duration_sec_ = 0.0;
    }

    if (config_.tracking.projection_enable &&
        config_.tracking.max_projection_duration_sec > 0.0 &&
        tracking_projection_duration_sec_ >=
            config_.tracking.max_projection_duration_sec) {
        tracking_projection_latched_ = true;
        return SafetyIntervention::TrackingProjection;
    }
    if (config_.tracking.spin_enable &&
        config_.tracking.spin_max_duration_sec > 0.0 &&
        tracking_spin_duration_sec_ >=
            config_.tracking.spin_max_duration_sec) {
        tracking_spin_latched_ = true;
        return SafetyIntervention::TrackingSpin;
    }
    return SafetyIntervention::None;
}

SafetySupervisorResult SafetySupervisor::snapshotResult() const {
    SafetySupervisorResult result;
    result.terminal_spin_duration_sec = terminal_spin_duration_sec_;
    result.tracking_projection_duration_sec =
        tracking_projection_duration_sec_;
    result.tracking_spin_duration_sec = tracking_spin_duration_sec_;
    result.terminal_spin_latched = terminal_spin_latched_;
    result.tracking_projection_latched = tracking_projection_latched_;
    result.tracking_spin_latched = tracking_spin_latched_;
    return result;
}

SafetySupervisorResult SafetySupervisor::step(
    const SafetySupervisorInput& input) {
    const double period_sec = validPeriod(input.period_sec);
    const bool terminal_blocked = updateTerminalSpin(input, period_sec);

    const VelocityCommand post_terminal_command = terminal_blocked
        ? VelocityCommand{}
        : input.command;
    SafetySupervisorInput tracking_input = input;
    if (terminal_blocked) {
        // The historical ROS sequence changed the status before invoking the
        // tracking gate.  Preserve that observable ordering here.
        tracking_input.status = "TERMINAL_SPIN_FAIL";
    }
    const SafetyIntervention tracking_intervention = updateTracking(
        tracking_input,
        input.command_accepted && !terminal_blocked,
        post_terminal_command,
        period_sec);

    SafetySupervisorResult result = snapshotResult();
    result.terminal_spin_blocked = terminal_blocked;
    result.tracking_safety_blocked =
        tracking_intervention == SafetyIntervention::TrackingProjection ||
        tracking_intervention == SafetyIntervention::TrackingSpin;
    result.blocked = result.terminal_spin_blocked ||
        result.tracking_safety_blocked;
    result.command = result.blocked ? VelocityCommand{} : input.command;
    result.accepted = input.command_accepted && !result.blocked;
    result.intervention = result.tracking_safety_blocked
        ? tracking_intervention
        : (terminal_blocked
               ? SafetyIntervention::TerminalSpin
               : SafetyIntervention::None);
    result.status = result.blocked
        ? safetyInterventionName(result.intervention)
        : input.status;
    return result;
}

}  // namespace spmpc_local_planner
