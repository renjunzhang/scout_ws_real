#pragma once

#include "spmpc_local_planner/core/terminal_diagnostics.h"
#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/domain/state.h"

#include <string>

namespace spmpc_local_planner {

struct TerminalSpinSafetyConfig {
    bool enable = true;
    double omega_threshold = 0.20;
    double max_duration_sec = 2.0;
};

struct TrackingSafetyConfig {
    bool enable = true;
    bool projection_enable = true;
    double max_projection_distance_m = 0.50;
    double max_projection_duration_sec = 0.20;
    bool spin_enable = true;
    double spin_omega_threshold = 0.50;
    double spin_max_duration_sec = 2.0;
};

struct SafetySupervisorConfig {
    double nominal_period_sec = 1.0 / 30.0;
    TerminalSpinSafetyConfig terminal_spin;
    TrackingSafetyConfig tracking;
};

// Narrow projection view used by the safety layer.  The supervisor must not
// depend on the complete solver output or on backend-specific diagnostics.
struct TrackingProjectionView {
    bool raw_valid = false;
    double raw_distance_m = 0.0;
    bool guarded_valid = false;
    double guarded_distance_m = 0.0;
};

struct SafetySupervisorInput {
    RobotState robot;
    VelocityCommand command;
    bool command_accepted = false;
    std::string status = "NOT_RUN";
    TerminalDiagnostics terminal;
    TrackingProjectionView projection;
    double period_sec = 0.0;
};

enum class SafetyIntervention {
    None,
    TerminalSpin,
    TrackingProjection,
    TrackingSpin,
};

const char* safetyInterventionName(SafetyIntervention intervention);

struct SafetySupervisorResult {
    VelocityCommand command;
    bool accepted = false;
    bool blocked = false;
    bool terminal_spin_blocked = false;
    bool tracking_safety_blocked = false;
    SafetyIntervention intervention = SafetyIntervention::None;
    std::string status;

    double terminal_spin_duration_sec = 0.0;
    double tracking_projection_duration_sec = 0.0;
    double tracking_spin_duration_sec = 0.0;
    bool terminal_spin_latched = false;
    bool tracking_projection_latched = false;
    bool tracking_spin_latched = false;
};

class SafetySupervisor {
public:
    bool configure(const SafetySupervisorConfig& config, std::string& error);
    void reset();

    SafetySupervisorResult step(const SafetySupervisorInput& input);

    const SafetySupervisorConfig& config() const { return config_; }

private:
    bool updateTerminalSpin(const SafetySupervisorInput& input,
                            double period_sec);
    SafetyIntervention updateTracking(const SafetySupervisorInput& input,
                                      bool command_accepted,
                                      const VelocityCommand& command,
                                      double period_sec);
    double validPeriod(double period_sec) const;
    SafetySupervisorResult snapshotResult() const;

    SafetySupervisorConfig config_;
    double terminal_spin_duration_sec_ = 0.0;
    bool terminal_spin_latched_ = false;
    double tracking_projection_duration_sec_ = 0.0;
    bool tracking_projection_latched_ = false;
    double tracking_spin_duration_sec_ = 0.0;
    bool tracking_spin_latched_ = false;
};

}  // namespace spmpc_local_planner
