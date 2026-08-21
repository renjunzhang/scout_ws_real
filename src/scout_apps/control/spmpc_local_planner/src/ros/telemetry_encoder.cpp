#include "spmpc_local_planner/ros/telemetry_encoder.h"

#include <ros/time.h>

namespace spmpc_local_planner {
namespace {

ros::Time rosTimeFromNanoseconds(std::int64_t stamp_ns) {
    if (stamp_ns <= 0) {
        return ros::Time(0);
    }
    ros::Time stamp;
    stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));
    return stamp;
}

void fillCycleTiming(
    const ControlCycleTimingDebug& timing,
    ControlCycleAudit& msg) {
    msg.cycle_id = timing.cycle_id;
    msg.cycle_start_stamp = rosTimeFromNanoseconds(
        timing.cycle_start_stamp_ns);
    msg.raw_robot_state_stamp = rosTimeFromNanoseconds(
        timing.raw_robot_state_stamp_ns);
    msg.raw_liquid_state_stamp = rosTimeFromNanoseconds(
        timing.raw_liquid_state_stamp_ns);
    msg.robot_state_stamp = rosTimeFromNanoseconds(
        timing.robot_state_stamp_ns);
    msg.liquid_state_stamp = rosTimeFromNanoseconds(
        timing.liquid_state_stamp_ns);
    msg.solver_input_epoch = rosTimeFromNanoseconds(
        timing.solver_input_epoch_ns);
    msg.solve_start_stamp = rosTimeFromNanoseconds(
        timing.solve_start_stamp_ns);
    msg.solve_end_stamp = rosTimeFromNanoseconds(
        timing.solve_end_stamp_ns);
    msg.horizon_available_stamp = rosTimeFromNanoseconds(
        timing.horizon_available_stamp_ns);
    msg.raw_state_skew_sec = timing.raw_state_skew_sec;
    msg.aligned_state_skew_sec = timing.aligned_state_skew_sec;
    msg.state_alignment_required = timing.state_alignment_required;
    msg.state_time_aligned = timing.state_time_aligned;
    msg.robot_state_interpolated = timing.robot_state_interpolated;
    msg.robot_state_extrapolated = timing.robot_state_extrapolated;
    msg.state_alignment_status = timing.state_alignment_status;
}

}  // namespace

std_msgs::Float32MultiArray encodeCommandIntervention(
    const CommandInterventionDebug& intervention) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "solver_cmd_v,solver_cmd_omega,post_gate_cmd_v,post_gate_cmd_omega,published_cmd_v,published_cmd_omega,output_success,zero_due_to_solver_failure,zero_due_to_waiting_for_odom,zero_due_to_waiting_for_reference,zero_due_to_waiting_for_tf,zero_due_to_waiting_for_slosh_observer,zero_due_to_terminal_spin_fail,zero_due_to_tracking_safety,zero_due_to_command_contract,linear_limited,angular_rate_limited,angular_accel_limited,publish_cmd_vel";
    msg.layout.dim[0].size = 19;
    msg.layout.dim[0].stride = 19;
    msg.data.resize(19, 0.0f);
    msg.data[0] = static_cast<float>(intervention.solver_cmd_v);
    msg.data[1] = static_cast<float>(intervention.solver_cmd_omega);
    msg.data[2] = static_cast<float>(intervention.post_gate_cmd_v);
    msg.data[3] = static_cast<float>(intervention.post_gate_cmd_omega);
    msg.data[4] = static_cast<float>(intervention.published_cmd_v);
    msg.data[5] = static_cast<float>(intervention.published_cmd_omega);
    msg.data[6] = intervention.output_success ? 1.0f : 0.0f;
    msg.data[7] = intervention.zero_due_to_solver_failure ? 1.0f : 0.0f;
    msg.data[8] = intervention.zero_due_to_waiting_for_odom ? 1.0f : 0.0f;
    msg.data[9] = intervention.zero_due_to_waiting_for_reference ? 1.0f : 0.0f;
    msg.data[10] = intervention.zero_due_to_waiting_for_tf ? 1.0f : 0.0f;
    msg.data[11] = intervention.zero_due_to_waiting_for_slosh_observer
        ? 1.0f : 0.0f;
    msg.data[12] = intervention.zero_due_to_terminal_spin_fail
        ? 1.0f : 0.0f;
    msg.data[13] = intervention.zero_due_to_tracking_safety ? 1.0f : 0.0f;
    msg.data[14] = intervention.zero_due_to_command_contract ? 1.0f : 0.0f;
    msg.data[15] = intervention.linear_limited ? 1.0f : 0.0f;
    msg.data[16] = intervention.angular_rate_limited ? 1.0f : 0.0f;
    msg.data[17] = intervention.angular_accel_limited ? 1.0f : 0.0f;
    msg.data[18] = intervention.publish_cmd_vel ? 1.0f : 0.0f;
    return msg;
}

ControlCycleAudit encodeControlCycleAudit(
    const ControlCycleAuditDebug& audit,
    const std::string& frame_id) {
    ControlCycleAudit msg;
    msg.header.stamp = rosTimeFromNanoseconds(
        audit.timing.command_publish_stamp_ns > 0
            ? audit.timing.command_publish_stamp_ns
            : audit.timing.horizon_available_stamp_ns);
    msg.header.frame_id = frame_id.empty() ? "map" : frame_id;
    msg.schema_version = 3;
    fillCycleTiming(audit.timing, msg);
    msg.command_publish_stamp = rosTimeFromNanoseconds(
        audit.timing.command_publish_stamp_ns);
    msg.variant = audit.variant;
    msg.status = audit.status;
    msg.solver_status = audit.solver_status;
    msg.observer_source = audit.observer_source;
    msg.solve_attempted = audit.solve_attempted;
    msg.solve_success = audit.solve_success;
    msg.command_accepted = audit.command_accepted;
    msg.publish_cmd_vel = audit.publish_cmd_vel;
    msg.command_was_published = audit.command_was_published;
    msg.publication_receipt_consistent =
        audit.publication_receipt_consistent;
    msg.command_history_committed = audit.command_history_committed;
    msg.phase_rejoin_committed = audit.phase_rejoin_committed;
    msg.command_contract_violation = audit.command_contract_violation;
    msg.terminal_phase = audit.terminal_phase;
    msg.terminal_controller_intervened =
        audit.terminal_controller_intervened;
    msg.safety_gate_intervened = audit.safety_gate_intervened;
    msg.linear_limited = audit.linear_limited;
    msg.angular_rate_limited = audit.angular_rate_limited;
    msg.angular_accel_limited = audit.angular_accel_limited;
    msg.solver_u0_a = audit.solver_u0_a;
    msg.solver_u0_alpha = audit.solver_u0_alpha;
    msg.planned_ax = audit.planned_ax;
    msg.planned_ay = audit.planned_ay;
    msg.solver_cmd_v = audit.solver_cmd_v;
    msg.solver_cmd_omega = audit.solver_cmd_omega;
    msg.terminal_cmd_v = audit.terminal_cmd_v;
    msg.terminal_cmd_omega = audit.terminal_cmd_omega;
    msg.post_gate_cmd_v = audit.post_gate_cmd_v;
    msg.post_gate_cmd_omega = audit.post_gate_cmd_omega;
    msg.finalized_cmd_v = audit.finalized_cmd_v;
    msg.finalized_cmd_omega = audit.finalized_cmd_omega;
    msg.published_cmd_v = audit.published_cmd_v;
    msg.published_cmd_omega = audit.published_cmd_omega;
    msg.previous_shifted_plan_available =
        audit.previous_shifted_plan_available;
    msg.previous_plan_cycle_id = audit.previous_plan_cycle_id;
    msg.previous_shifted_plan_a = audit.previous_shifted_plan_a;
    msg.previous_shifted_plan_alpha = audit.previous_shifted_plan_alpha;
    msg.replanned_minus_shifted_a = audit.replanned_minus_shifted_a;
    msg.replanned_minus_shifted_alpha =
        audit.replanned_minus_shifted_alpha;

    msg.odom_excitation_valid = audit.odom_excitation.valid;
    msg.odom_measurement_stamp = rosTimeFromNanoseconds(
        audit.odom_excitation.measurement_stamp_ns);
    msg.odom_accel_effective_stamp = rosTimeFromNanoseconds(
        audit.odom_excitation.accel_effective_stamp_ns);
    msg.odom_receive_stamp = rosTimeFromNanoseconds(
        audit.odom_excitation.receive_stamp_ns);
    msg.odom_ax = audit.odom_excitation.ax;
    msg.odom_ay = audit.odom_excitation.ay;
    msg.odom_omega = audit.odom_excitation.omega;
    msg.odom_alpha = audit.odom_excitation.alpha;
    msg.odom_sample_dt_sec = audit.odom_excitation.sample_dt_sec;

    msg.imu_excitation_valid = audit.imu_excitation.valid;
    msg.imu_measurement_stamp = rosTimeFromNanoseconds(
        audit.imu_excitation.measurement_stamp_ns);
    msg.imu_accel_effective_stamp = rosTimeFromNanoseconds(
        audit.imu_excitation.accel_effective_stamp_ns);
    msg.imu_receive_stamp = rosTimeFromNanoseconds(
        audit.imu_excitation.receive_stamp_ns);
    msg.imu_ax = audit.imu_excitation.ax;
    msg.imu_ay = audit.imu_excitation.ay;
    msg.imu_omega = audit.imu_excitation.omega;
    msg.imu_alpha = audit.imu_excitation.alpha;
    msg.imu_sample_dt_sec = audit.imu_excitation.sample_dt_sec;
    return msg;
}

}  // namespace spmpc_local_planner
