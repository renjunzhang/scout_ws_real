#pragma once

#include "spmpc_local_planner/controller/command/command_pipeline.h"
#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/domain/time.h"
#include "spmpc_local_planner/estimation/motion_excitation.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

// Pure C++ snapshot owned by ControlCycleEngine.  ROS adapters may enrich it
// with transport timestamps, observer selection and publish/limiter outcomes,
// but must not reconstruct solver/terminal/phase/safety decisions themselves.
struct ControlCycleTelemetrySnapshot {
    std::uint64_t cycle_id = 0;
    StampNs cycle_start_ns = 0;
    std::string status = "NOT_RUN";
    std::string solver_status = "NOT_RUN";
    std::string command_reason = "NO_COMMAND";
    CommandSource command_source = CommandSource::None;

    bool solve_attempted = false;
    bool solve_returned = false;
    bool solve_success = false;
    bool command_accepted = false;
    bool terminal_phase = false;
    bool terminal_priority = false;
    bool terminal_controller_intervened = false;
    bool safety_gate_intervened = false;
    bool terminal_spin_blocked = false;
    bool tracking_safety_blocked = false;
    bool phase_rejoin_evaluated = false;
    bool phase_rejoin_recovery_used = false;
    bool phase_rejoin_controlled_stop_used = false;

    double solver_u0_a = 0.0;
    double solver_u0_alpha = 0.0;
    double planned_ax = 0.0;
    double planned_ay = 0.0;
    VelocityCommand solver_command;
    VelocityCommand terminal_command;
    VelocityCommand post_gate_command;
    VelocityCommand final_command;
};

struct CommandInterventionDebug {
    double solver_cmd_v = 0.0;
    double solver_cmd_omega = 0.0;
    double post_gate_cmd_v = 0.0;
    double post_gate_cmd_omega = 0.0;
    double published_cmd_v = 0.0;
    double published_cmd_omega = 0.0;
    bool output_success = false;
    bool zero_due_to_solver_failure = false;
    bool zero_due_to_waiting_for_odom = false;
    bool zero_due_to_waiting_for_reference = false;
    bool zero_due_to_waiting_for_tf = false;
    bool zero_due_to_waiting_for_slosh_observer = false;
    bool zero_due_to_terminal_spin_fail = false;
    bool zero_due_to_tracking_safety = false;
    bool zero_due_to_command_contract = false;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;
    bool publish_cmd_vel = false;
};

struct ExcitationAuditDebug {
    bool valid = false;
    std::int64_t measurement_stamp_ns = 0;
    std::int64_t accel_effective_stamp_ns = 0;
    std::int64_t receive_stamp_ns = 0;
    double ax = 0.0;
    double ay = 0.0;
    double omega = 0.0;
    double alpha = 0.0;
    double sample_dt_sec = 0.0;
};

inline ExcitationAuditDebug makeExcitationAudit(
    const MotionExcitation& excitation) {
    ExcitationAuditDebug audit;
    audit.valid = excitation.valid;
    audit.measurement_stamp_ns = excitation.measurement_stamp_ns;
    audit.accel_effective_stamp_ns = excitation.accel_effective_stamp_ns;
    audit.receive_stamp_ns = excitation.receive_stamp_ns;
    audit.ax = excitation.ax;
    audit.ay = excitation.ay;
    audit.omega = excitation.omega_z;
    audit.alpha = excitation.alpha_z;
    audit.sample_dt_sec = excitation.sample_dt_sec;
    return audit;
}

struct ControlCycleAuditDebug {
    ControlCycleTimingDebug timing;
    std::string variant;
    std::string status = "NOT_RUN";
    std::string solver_status = "NOT_RUN";
    std::uint8_t observer_source = 0;
    bool solve_attempted = false;
    bool solve_success = false;
    bool command_accepted = false;
    bool publish_cmd_vel = false;
    bool command_was_published = false;
    bool command_contract_violation = false;
    bool terminal_phase = false;
    bool terminal_controller_intervened = false;
    bool safety_gate_intervened = false;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;

    double solver_u0_a = 0.0;
    double solver_u0_alpha = 0.0;
    double planned_ax = 0.0;
    double planned_ay = 0.0;
    double solver_cmd_v = 0.0;
    double solver_cmd_omega = 0.0;
    double terminal_cmd_v = 0.0;
    double terminal_cmd_omega = 0.0;
    double post_gate_cmd_v = 0.0;
    double post_gate_cmd_omega = 0.0;
    double published_cmd_v = 0.0;
    double published_cmd_omega = 0.0;

    bool previous_shifted_plan_available = false;
    std::uint64_t previous_plan_cycle_id = 0;
    double previous_shifted_plan_a = 0.0;
    double previous_shifted_plan_alpha = 0.0;
    double replanned_minus_shifted_a = 0.0;
    double replanned_minus_shifted_alpha = 0.0;

    ExcitationAuditDebug odom_excitation;
    ExcitationAuditDebug imu_excitation;
};

inline CommandInterventionDebug makeCommandInterventionDebug(
    const ControlCycleTelemetrySnapshot& snapshot) {
    CommandInterventionDebug debug;
    debug.solver_cmd_v = snapshot.solver_command.linear;
    debug.solver_cmd_omega = snapshot.solver_command.angular;
    debug.post_gate_cmd_v = snapshot.post_gate_command.linear;
    debug.post_gate_cmd_omega = snapshot.post_gate_command.angular;
    debug.output_success = snapshot.command_accepted;
    debug.zero_due_to_solver_failure = !snapshot.solve_success;
    debug.zero_due_to_terminal_spin_fail = snapshot.terminal_spin_blocked;
    debug.zero_due_to_tracking_safety = snapshot.tracking_safety_blocked;
    return debug;
}

inline void applyControlCycleTelemetry(
    const ControlCycleTelemetrySnapshot& snapshot,
    ControlCycleAuditDebug& audit) {
    audit.timing.cycle_id = snapshot.cycle_id;
    audit.timing.cycle_start_stamp_ns = snapshot.cycle_start_ns;
    audit.status = snapshot.status;
    audit.solver_status = snapshot.solver_status;
    audit.solve_attempted = snapshot.solve_attempted;
    audit.solve_success = snapshot.solve_success;
    audit.command_accepted = snapshot.command_accepted;
    audit.terminal_phase = snapshot.terminal_phase;
    audit.terminal_controller_intervened =
        snapshot.terminal_controller_intervened;
    audit.safety_gate_intervened = snapshot.safety_gate_intervened;
    audit.solver_u0_a = snapshot.solver_u0_a;
    audit.solver_u0_alpha = snapshot.solver_u0_alpha;
    audit.planned_ax = snapshot.planned_ax;
    audit.planned_ay = snapshot.planned_ay;
    audit.solver_cmd_v = snapshot.solver_command.linear;
    audit.solver_cmd_omega = snapshot.solver_command.angular;
    audit.terminal_cmd_v = snapshot.terminal_command.linear;
    audit.terminal_cmd_omega = snapshot.terminal_command.angular;
    audit.post_gate_cmd_v = snapshot.post_gate_command.linear;
    audit.post_gate_cmd_omega = snapshot.post_gate_command.angular;
}

}  // namespace spmpc_local_planner
