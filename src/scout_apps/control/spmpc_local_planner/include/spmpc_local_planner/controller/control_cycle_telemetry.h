#pragma once

#include "spmpc_local_planner/controller/command/command_pipeline.h"
#include "spmpc_local_planner/telemetry/solver_diagnostics.h"
#include "spmpc_local_planner/domain/time.h"
#include "spmpc_local_planner/estimation/motion_excitation.h"
#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

// Pure C++ snapshot owned by ControlCycleEngine.  ROS adapters may encode it
// and add observer-source context, but publication/limiter outcomes and all
// solver/terminal/phase/safety decisions are already final here.
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
    bool phase_rejoin_committed = false;
    bool publication_attempted = false;
    bool command_was_published = false;
    bool publication_receipt_consistent = false;
    bool command_history_committed = false;
    bool command_contract_violation = false;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;
    StampNs command_publish_stamp_ns = 0;
    PublishLatencyObservation publish_timing;

    double solver_u0_a = 0.0;
    double solver_u0_alpha = 0.0;
    double planned_ax = 0.0;
    double planned_ay = 0.0;
    bool previous_shifted_plan_available = false;
    std::uint64_t previous_plan_cycle_id = 0;
    double previous_shifted_plan_a = 0.0;
    double previous_shifted_plan_alpha = 0.0;
    double replanned_minus_shifted_a = 0.0;
    double replanned_minus_shifted_alpha = 0.0;
    VelocityCommand solver_command;
    VelocityCommand terminal_command;
    VelocityCommand post_gate_command;
    VelocityCommand final_command;
    VelocityCommand published_command;
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
    bool publication_receipt_consistent = false;
    bool command_history_committed = false;
    bool phase_rejoin_committed = false;
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
    double finalized_cmd_v = 0.0;
    double finalized_cmd_omega = 0.0;
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
    debug.published_cmd_v = snapshot.command_was_published
        ? snapshot.published_command.linear
        : 0.0;
    debug.published_cmd_omega = snapshot.command_was_published
        ? snapshot.published_command.angular
        : 0.0;
    debug.output_success = snapshot.command_accepted;
    debug.zero_due_to_solver_failure = !snapshot.solve_success;
    debug.zero_due_to_terminal_spin_fail = snapshot.terminal_spin_blocked;
    debug.zero_due_to_tracking_safety = snapshot.tracking_safety_blocked;
    debug.zero_due_to_command_contract =
        snapshot.command_contract_violation;
    debug.linear_limited = snapshot.linear_limited;
    debug.angular_rate_limited = snapshot.angular_rate_limited;
    debug.angular_accel_limited = snapshot.angular_accel_limited;
    return debug;
}

inline void applyPublishEpochEstimate(
    const PublishEpochEstimate& estimate,
    ControlCycleTimingDebug& timing) {
    timing.expected_publish_stamp_ns =
        estimate.expected_publish_stamp_ns;
    timing.publish_deadline_stamp_ns =
        estimate.publish_deadline_stamp_ns;
    timing.estimated_dc_sec = estimate.estimated_dc_sec;
    timing.publish_epoch_estimate_valid = estimate.valid;
    timing.expected_publish_deadline_missed =
        estimate.expected_deadline_missed;
    timing.publish_timing_status = estimate.status;
}

inline void applyPublishLatencyObservation(
    const PublishLatencyObservation& observation,
    ControlCycleTimingDebug& timing) {
    applyPublishEpochEstimate(observation.estimate, timing);
    timing.actual_dc_sec = observation.actual_dc_sec;
    timing.dc_error_sec = observation.dc_error_sec;
    timing.publish_latency_observation_valid = observation.actual_valid;
    timing.publish_deadline_missed =
        observation.publish_deadline_missed;
    timing.publish_timing_status = observation.status;
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
    audit.command_was_published = snapshot.command_was_published;
    audit.publication_receipt_consistent =
        snapshot.publication_receipt_consistent;
    audit.command_history_committed = snapshot.command_history_committed;
    audit.phase_rejoin_committed = snapshot.phase_rejoin_committed;
    audit.command_contract_violation = snapshot.command_contract_violation;
    audit.linear_limited = snapshot.linear_limited;
    audit.angular_rate_limited = snapshot.angular_rate_limited;
    audit.angular_accel_limited = snapshot.angular_accel_limited;
    applyPublishLatencyObservation(snapshot.publish_timing, audit.timing);
    if (snapshot.command_was_published) {
        audit.timing.command_publish_stamp_ns =
            snapshot.command_publish_stamp_ns;
        audit.published_cmd_v = snapshot.published_command.linear;
        audit.published_cmd_omega = snapshot.published_command.angular;
    }
    audit.terminal_phase = snapshot.terminal_phase;
    audit.terminal_controller_intervened =
        snapshot.terminal_controller_intervened;
    audit.safety_gate_intervened = snapshot.safety_gate_intervened;
    audit.solver_u0_a = snapshot.solver_u0_a;
    audit.solver_u0_alpha = snapshot.solver_u0_alpha;
    audit.planned_ax = snapshot.planned_ax;
    audit.planned_ay = snapshot.planned_ay;
    audit.previous_shifted_plan_available =
        snapshot.previous_shifted_plan_available;
    audit.previous_plan_cycle_id = snapshot.previous_plan_cycle_id;
    audit.previous_shifted_plan_a = snapshot.previous_shifted_plan_a;
    audit.previous_shifted_plan_alpha = snapshot.previous_shifted_plan_alpha;
    audit.replanned_minus_shifted_a = snapshot.replanned_minus_shifted_a;
    audit.replanned_minus_shifted_alpha =
        snapshot.replanned_minus_shifted_alpha;
    audit.solver_cmd_v = snapshot.solver_command.linear;
    audit.solver_cmd_omega = snapshot.solver_command.angular;
    audit.terminal_cmd_v = snapshot.terminal_command.linear;
    audit.terminal_cmd_omega = snapshot.terminal_command.angular;
    audit.post_gate_cmd_v = snapshot.post_gate_command.linear;
    audit.post_gate_cmd_omega = snapshot.post_gate_command.angular;
    audit.finalized_cmd_v = snapshot.final_command.linear;
    audit.finalized_cmd_omega = snapshot.final_command.angular;
}

}  // namespace spmpc_local_planner
