#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

void DiagnosticsPublisher::initialize(ros::NodeHandle& nh) {
    status_pub_ = nh.advertise<std_msgs::String>("status", 1, true);
    variant_pub_ = nh.advertise<std_msgs::String>("controller_variant", 1, true);
    experiment_mode_pub_ = nh.advertise<std_msgs::String>("experiment_mode", 1, true);
    solver_backend_pub_ = nh.advertise<std_msgs::String>("solver_backend", 1, true);
    effective_config_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/effective_config", 1, true);
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("local_trajectory", 1, true);
    progress_pub_ = nh.advertise<std_msgs::Float32>("debug/progress_s", 1);
    v_ref_current_pub_ = nh.advertise<std_msgs::Float32>("debug/v_ref_current", 1);
    map_vref_status_pub_ = nh.advertise<std_msgs::String>("debug/map_vref_status", 1);
    solver_time_pub_ = nh.advertise<std_msgs::Float32>("solver_time_ms", 1);
    cost_breakdown_pub_ = nh.advertise<std_msgs::Float32MultiArray>("cost_breakdown", 1);
    corridor_pub_ = nh.advertise<std_msgs::Float32MultiArray>("corridor", 1);
    guidance_pub_ = nh.advertise<std_msgs::Float32MultiArray>("guidance", 1);
    primitive_pub_ = nh.advertise<std_msgs::Float32MultiArray>("primitive", 1);
    slosh_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_state", 1);
    slosh_height_pub_ = nh.advertise<std_msgs::Float32>("slosh_height", 1);
    slosh_horizon_summary_pub_ = nh.advertise<std_msgs::Float32MultiArray>("slosh_horizon_summary", 1);
    slosh_hard_constraint_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_hard_constraint", 1);
    slosh_hard_constraint_effective_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_hard_constraint_effective", 1);
    slosh_cost_monitor_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_cost_monitor", 1);
    warm_start_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/warm_start", 1);
    warm_start_status_pub_ = nh.advertise<std_msgs::String>("debug/warm_start_status", 1);
    runtime_bounds_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/runtime_bounds", 1);
    generated_bounds_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/generated_bounds", 1);
    first_shot_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/first_shot_summary", 1);
    projector_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/projector", 1);
    stage0_reference_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/stage0_reference", 1);
    local_traj_head_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/local_traj_head", 1);
    warm_start_head_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/warm_start_head", 1);
    cmd_output_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/cmd_vel_output", 1);
    cmd_output_status_pub_ = nh.advertise<std_msgs::String>("debug/cmd_vel_output_status", 1);
    delay_phase_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/delay_phase", 1);
    odom_timing_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/odom_timing", 1);
    execution_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/execution_state", 1);
    execution_alignment_status_pub_ = nh.advertise<std_msgs::String>("debug/execution_alignment_status", 1);
    delay_compensation_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/delay_compensation", 1);
    cmd_odom_alignment_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/cmd_odom_alignment", 1);
    terminal_pub_ = nh.advertise<std_msgs::Float32MultiArray>("terminal/debug", 1);
    terminal_mode_pub_ = nh.advertise<std_msgs::String>("terminal/mode", 1);
    start_lock_active_pub_ = nh.advertise<std_msgs::Float32>("start_lock/active", 1);
    start_lock_mode_pub_ = nh.advertise<std_msgs::String>("start_lock/mode", 1);
    start_lock_debug_pub_ = nh.advertise<std_msgs::Float32MultiArray>("start_lock/debug", 1);
}

void DiagnosticsPublisher::publishVariant(
    const VariantConfig& variant,
    const std::string& experiment_mode) {
    std_msgs::String msg;
    msg.data = variant.name;
    variant_pub_.publish(msg);

    msg.data = experiment_mode;
    experiment_mode_pub_.publish(msg);
}

void DiagnosticsPublisher::publishSolverBackend(const std::string& solver_backend) {
    std_msgs::String msg;
    msg.data = solver_backend;
    solver_backend_pub_.publish(msg);
}

void DiagnosticsPublisher::publishEffectiveConfig(const EffectiveConfigDebug& config) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "solver_backend_code,control_frequency,dt,horizon_steps,slosh_enable,slosh_constraint_enable,smooth_priority_enable,primitive_mode_code,v_ref,w_slosh,w_control,w_smooth,w_accel,w_alpha,w_du_a,w_du_vs,v_max,omega_max,a_max,alpha_max,shared_linear_accel_limit_enable,shared_linear_accel_max,shared_linear_accel_max_dt,shared_angular_limit_enable,shared_angular_rate_max,shared_angular_accel_max,shared_angular_accel_max_dt,container_radius,liquid_height,damping_ratio,slosh_height_ref,slosh_height_max,slosh_eta_dot_ratio,use_parabola_term,delay_phase_mode_code,delay_linear_sec,delay_angular_sec,delay_cmd_timeout_sec,delay_odom_timeout_sec,delay_history_window_sec,delay_require_complete_history";
    msg.layout.dim[0].size = 41;
    msg.layout.dim[0].stride = 41;
    msg.data.resize(41, 0.0f);
    msg.data[0] = static_cast<float>(config.solver_backend_code);
    msg.data[1] = static_cast<float>(config.control_frequency);
    msg.data[2] = static_cast<float>(config.dt);
    msg.data[3] = static_cast<float>(config.horizon_steps);
    msg.data[4] = static_cast<float>(config.slosh_enable);
    msg.data[5] = static_cast<float>(config.slosh_constraint_enable);
    msg.data[6] = static_cast<float>(config.smooth_priority_enable);
    msg.data[7] = static_cast<float>(config.primitive_mode_code);
    msg.data[8] = static_cast<float>(config.v_ref);
    msg.data[9] = static_cast<float>(config.w_slosh);
    msg.data[10] = static_cast<float>(config.w_control);
    msg.data[11] = static_cast<float>(config.w_smooth);
    msg.data[12] = static_cast<float>(config.w_accel);
    msg.data[13] = static_cast<float>(config.w_alpha);
    msg.data[14] = static_cast<float>(config.w_du_a);
    msg.data[15] = static_cast<float>(config.w_du_vs);
    msg.data[16] = static_cast<float>(config.v_max);
    msg.data[17] = static_cast<float>(config.omega_max);
    msg.data[18] = static_cast<float>(config.a_max);
    msg.data[19] = static_cast<float>(config.alpha_max);
    msg.data[20] = static_cast<float>(config.shared_linear_accel_limit_enable);
    msg.data[21] = static_cast<float>(config.shared_linear_accel_max);
    msg.data[22] = static_cast<float>(config.shared_linear_accel_max_dt);
    msg.data[23] = static_cast<float>(config.shared_angular_limit_enable);
    msg.data[24] = static_cast<float>(config.shared_angular_rate_max);
    msg.data[25] = static_cast<float>(config.shared_angular_accel_max);
    msg.data[26] = static_cast<float>(config.shared_angular_accel_max_dt);
    msg.data[27] = static_cast<float>(config.container_radius);
    msg.data[28] = static_cast<float>(config.liquid_height);
    msg.data[29] = static_cast<float>(config.damping_ratio);
    msg.data[30] = static_cast<float>(config.slosh_height_ref);
    msg.data[31] = static_cast<float>(config.slosh_height_max);
    msg.data[32] = static_cast<float>(config.slosh_eta_dot_ratio);
    msg.data[33] = static_cast<float>(config.use_parabola_term);
    msg.data[34] = static_cast<float>(config.delay_phase_mode_code);
    msg.data[35] = static_cast<float>(config.delay_linear_sec);
    msg.data[36] = static_cast<float>(config.delay_angular_sec);
    msg.data[37] = static_cast<float>(config.delay_cmd_timeout_sec);
    msg.data[38] = static_cast<float>(config.delay_odom_timeout_sec);
    msg.data[39] = static_cast<float>(config.delay_history_window_sec);
    msg.data[40] = static_cast<float>(config.delay_require_complete_history);
    effective_config_pub_.publish(msg);
}

void DiagnosticsPublisher::publishCommandOutput(const geometry_msgs::Twist& desired,
                                                const geometry_msgs::Twist& limited,
                                                const geometry_msgs::Twist& previous,
                                                double dt,
                                                bool linear_limited,
                                                bool angular_rate_limited,
                                                bool angular_accel_limited) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "desired_v,desired_omega,limited_v,limited_omega,prev_v,prev_omega,dt,linear_limited,angular_rate_limited,angular_accel_limited";
    msg.layout.dim[0].size = 10;
    msg.layout.dim[0].stride = 10;
    msg.data.resize(10, 0.0f);
    msg.data[0] = static_cast<float>(desired.linear.x);
    msg.data[1] = static_cast<float>(desired.angular.z);
    msg.data[2] = static_cast<float>(limited.linear.x);
    msg.data[3] = static_cast<float>(limited.angular.z);
    msg.data[4] = static_cast<float>(previous.linear.x);
    msg.data[5] = static_cast<float>(previous.angular.z);
    msg.data[6] = static_cast<float>(dt);
    msg.data[7] = linear_limited ? 1.0f : 0.0f;
    msg.data[8] = angular_rate_limited ? 1.0f : 0.0f;
    msg.data[9] = angular_accel_limited ? 1.0f : 0.0f;
    cmd_output_pub_.publish(msg);

    std_msgs::String status;
    if (linear_limited || angular_rate_limited || angular_accel_limited) {
        status.data = "LIMITED";
    } else {
        status.data = "PASS";
    }
    cmd_output_status_pub_.publish(status);
}

void DiagnosticsPublisher::publishDelayPhase(const DelayPhaseDebugSummary& summary) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "mode_code,cmd_age_ms,cmd_period_ms,odom_age_ms,solver_time_ms,linear_delay_ms,angular_delay_ms,history_span_ms,history_complete,shadow_valid,status_code";
    msg.layout.dim[0].size = 11;
    msg.layout.dim[0].stride = 11;
    msg.data.resize(11, 0.0f);
    msg.data[0] = static_cast<float>(static_cast<int>(summary.mode));
    msg.data[1] = static_cast<float>(summary.cmd_age_ms);
    msg.data[2] = static_cast<float>(summary.cmd_period_ms);
    msg.data[3] = static_cast<float>(summary.odom_age_ms);
    msg.data[4] = static_cast<float>(summary.solver_time_ms);
    msg.data[5] = static_cast<float>(summary.linear_delay_ms);
    msg.data[6] = static_cast<float>(summary.angular_delay_ms);
    msg.data[7] = static_cast<float>(summary.history_span_ms);
    msg.data[8] = summary.history_complete ? 1.0f : 0.0f;
    msg.data[9] = summary.shadow_valid ? 1.0f : 0.0f;
    msg.data[10] = static_cast<float>(static_cast<int>(summary.status_code));
    delay_phase_pub_.publish(msg);
}

void DiagnosticsPublisher::publishOdomTiming(const OdomTimingDebug& timing) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "recv_age_ms,stamp_dt_ms,ax,ay,omega,have_prev_odom,dt_clamped";
    msg.layout.dim[0].size = 7;
    msg.layout.dim[0].stride = 7;
    msg.data.resize(7, 0.0f);
    msg.data[0] = static_cast<float>(timing.recv_age_ms);
    msg.data[1] = static_cast<float>(timing.stamp_dt_ms);
    msg.data[2] = static_cast<float>(timing.ax);
    msg.data[3] = static_cast<float>(timing.ay);
    msg.data[4] = static_cast<float>(timing.omega);
    msg.data[5] = timing.have_prev_odom ? 1.0f : 0.0f;
    msg.data[6] = timing.dt_clamped ? 1.0f : 0.0f;
    odom_timing_pub_.publish(msg);
}

void DiagnosticsPublisher::publishExecutionState(const ExecutionStatePrediction& prediction) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "valid,raw_x,raw_y,raw_yaw,raw_v,raw_omega,pred_x,pred_y,pred_yaw,pred_v,pred_omega,pred_eta_x,pred_eta_x_dot,pred_eta_y,pred_eta_y_dot,integrated_ms,covered_history_ms,missing_history_ms,history_complete";
    msg.layout.dim[0].size = 19;
    msg.layout.dim[0].stride = 19;
    msg.data.resize(19, 0.0f);
    msg.data[0] = prediction.valid ? 1.0f : 0.0f;
    msg.data[1] = static_cast<float>(prediction.raw_robot.x);
    msg.data[2] = static_cast<float>(prediction.raw_robot.y);
    msg.data[3] = static_cast<float>(prediction.raw_robot.yaw);
    msg.data[4] = static_cast<float>(prediction.raw_robot.v);
    msg.data[5] = static_cast<float>(prediction.raw_robot.omega);
    msg.data[6] = static_cast<float>(prediction.predicted_robot.x);
    msg.data[7] = static_cast<float>(prediction.predicted_robot.y);
    msg.data[8] = static_cast<float>(prediction.predicted_robot.yaw);
    msg.data[9] = static_cast<float>(prediction.predicted_robot.v);
    msg.data[10] = static_cast<float>(prediction.predicted_robot.omega);
    msg.data[11] = static_cast<float>(prediction.predicted_slosh.eta_x);
    msg.data[12] = static_cast<float>(prediction.predicted_slosh.eta_x_dot);
    msg.data[13] = static_cast<float>(prediction.predicted_slosh.eta_y);
    msg.data[14] = static_cast<float>(prediction.predicted_slosh.eta_y_dot);
    msg.data[15] = static_cast<float>(1000.0 * prediction.integrated_duration_sec);
    msg.data[16] = static_cast<float>(1000.0 * prediction.covered_history_sec);
    msg.data[17] = static_cast<float>(1000.0 * prediction.missing_history_sec);
    msg.data[18] = prediction.history_complete ? 1.0f : 0.0f;
    execution_state_pub_.publish(msg);
}

void DiagnosticsPublisher::publishExecutionAlignmentStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = status;
    execution_alignment_status_pub_.publish(msg);
}

void DiagnosticsPublisher::publishDelayCompensation(const DelayPhaseDebugSummary& summary) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "mode_code,closed_loop_enabled,linear_delay_ms,angular_delay_ms,shadow_valid,status_code";
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 6;
    msg.data.resize(6, 0.0f);
    msg.data[0] = static_cast<float>(static_cast<int>(summary.mode));
    msg.data[1] = summary.closed_loop_enabled ? 1.0f : 0.0f;
    msg.data[2] = static_cast<float>(summary.linear_delay_ms);
    msg.data[3] = static_cast<float>(summary.angular_delay_ms);
    msg.data[4] = summary.shadow_valid ? 1.0f : 0.0f;
    msg.data[5] = static_cast<float>(static_cast<int>(summary.status_code));
    delay_compensation_pub_.publish(msg);
}

void DiagnosticsPublisher::publishCmdOdomAlignment(const CmdOdomAlignmentDebug& alignment) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "mode_code,cmd_age_ms,cmd_period_ms,odom_age_ms,odom_period_ms,linear_delay_ms,angular_delay_ms,history_span_ms,covered_history_ms,missing_history_ms,history_complete,shadow_valid,fixed_closed_loop_configured,fixed_closed_loop_applied,status_code,dx_pred_raw,dy_pred_raw,dyaw_pred_raw,dv_pred_raw,domega_pred_raw,deta_norm_pred_raw,deta_dot_norm_pred_raw";
    msg.layout.dim[0].size = 22;
    msg.layout.dim[0].stride = 22;
    msg.data.resize(22, 0.0f);
    msg.data[0] = static_cast<float>(static_cast<int>(alignment.mode));
    msg.data[1] = static_cast<float>(alignment.cmd_age_ms);
    msg.data[2] = static_cast<float>(alignment.cmd_period_ms);
    msg.data[3] = static_cast<float>(alignment.odom_age_ms);
    msg.data[4] = static_cast<float>(alignment.odom_period_ms);
    msg.data[5] = static_cast<float>(alignment.linear_delay_ms);
    msg.data[6] = static_cast<float>(alignment.angular_delay_ms);
    msg.data[7] = static_cast<float>(alignment.history_span_ms);
    msg.data[8] = static_cast<float>(alignment.covered_history_ms);
    msg.data[9] = static_cast<float>(alignment.missing_history_ms);
    msg.data[10] = alignment.history_complete ? 1.0f : 0.0f;
    msg.data[11] = alignment.shadow_valid ? 1.0f : 0.0f;
    msg.data[12] = alignment.fixed_closed_loop_configured ? 1.0f : 0.0f;
    msg.data[13] = alignment.fixed_closed_loop_applied ? 1.0f : 0.0f;
    msg.data[14] = static_cast<float>(static_cast<int>(alignment.status_code));
    msg.data[15] = static_cast<float>(alignment.dx_pred_raw);
    msg.data[16] = static_cast<float>(alignment.dy_pred_raw);
    msg.data[17] = static_cast<float>(alignment.dyaw_pred_raw);
    msg.data[18] = static_cast<float>(alignment.dv_pred_raw);
    msg.data[19] = static_cast<float>(alignment.domega_pred_raw);
    msg.data[20] = static_cast<float>(alignment.deta_norm_pred_raw);
    msg.data[21] = static_cast<float>(alignment.deta_dot_norm_pred_raw);
    cmd_odom_alignment_pub_.publish(msg);
}

void DiagnosticsPublisher::publishOutput(const SolverOutput& output, const std::string& frame_id) {
    trajectory_pub_.publish(makePathMsg(output, frame_id));

    std_msgs::Float32 progress;
    progress.data = static_cast<float>(output.progress_s);
    progress_pub_.publish(progress);

    std_msgs::Float32 v_ref_current;
    v_ref_current.data = static_cast<float>(output.v_ref_debug.effective);
    v_ref_current_pub_.publish(v_ref_current);

    std_msgs::String map_vref_status;
    map_vref_status.data = output.v_ref_debug.status.empty()
                               ? (output.v_ref_debug.runtime_override ? "RUNTIME_OVERRIDE" : "VARIANT_FALLBACK")
                               : output.v_ref_debug.status;
    map_vref_status_pub_.publish(map_vref_status);

    std_msgs::Float32 solver_ms;
    solver_ms.data = static_cast<float>(output.solver_time_ms);
    solver_time_pub_.publish(solver_ms);

    const auto& ws = output.warm_start_diagnostics;
    std_msgs::Float32MultiArray warm_start;
    warm_start.layout.dim.resize(1);
    warm_start.layout.dim[0].label =
        "valid,used_flatness,used_previous_solution,used_fallback,used_slosh_rollout,bound_violation_count,max_v,max_omega,max_a,max_lateral_acc,max_slosh_height_pred,reference_fit_error";
    warm_start.layout.dim[0].size = 12;
    warm_start.layout.dim[0].stride = 12;
    warm_start.data.resize(12, 0.0f);
    warm_start.data[0] = ws.warm_start_valid ? 1.0f : 0.0f;
    warm_start.data[1] = ws.used_flatness ? 1.0f : 0.0f;
    warm_start.data[2] = ws.used_previous_solution ? 1.0f : 0.0f;
    warm_start.data[3] = ws.used_fallback ? 1.0f : 0.0f;
    warm_start.data[4] = ws.used_slosh_rollout ? 1.0f : 0.0f;
    warm_start.data[5] = static_cast<float>(ws.bound_violation_count);
    warm_start.data[6] = static_cast<float>(ws.max_v);
    warm_start.data[7] = static_cast<float>(ws.max_omega);
    warm_start.data[8] = static_cast<float>(ws.max_a);
    warm_start.data[9] = static_cast<float>(ws.max_lateral_acc);
    warm_start.data[10] = static_cast<float>(1000.0 * ws.max_slosh_height_pred);
    warm_start.data[11] = static_cast<float>(ws.reference_fit_error);
    warm_start_pub_.publish(warm_start);

    std_msgs::String warm_start_status;
    warm_start_status.data = ws.failure_reason.empty() ? "OK" : ws.failure_reason;
    warm_start_status_pub_.publish(warm_start_status);

    const auto publish_bounds = [](const SolverBoundSummary& bounds, ros::Publisher& pub) {
        std_msgs::Float32MultiArray msg;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "a_min,a_max,alpha_min,alpha_max,vs_min,vs_max,v_min,v_max,omega_min,omega_max";
        msg.layout.dim[0].size = 10;
        msg.layout.dim[0].stride = 10;
        msg.data.resize(10, 0.0f);
        msg.data[0] = static_cast<float>(bounds.a_min);
        msg.data[1] = static_cast<float>(bounds.a_max);
        msg.data[2] = static_cast<float>(bounds.alpha_min);
        msg.data[3] = static_cast<float>(bounds.alpha_max);
        msg.data[4] = static_cast<float>(bounds.v_s_min);
        msg.data[5] = static_cast<float>(bounds.v_s_max);
        msg.data[6] = static_cast<float>(bounds.v_min);
        msg.data[7] = static_cast<float>(bounds.v_max);
        msg.data[8] = static_cast<float>(bounds.omega_min);
        msg.data[9] = static_cast<float>(bounds.omega_max);
        pub.publish(msg);
    };
    publish_bounds(output.runtime_bounds, runtime_bounds_pub_);
    publish_bounds(output.generated_bounds, generated_bounds_pub_);

    const auto& fs = output.first_shot_debug;
    std_msgs::Float32MultiArray first_shot;
    first_shot.layout.dim.resize(1);
    first_shot.layout.dim[0].label =
        "success,status_code,progress_s,progress_abs_s,x0_v,x0_omega,x0_s,u0_a,u0_alpha,u0_vs,cmd_v_pre,cmd_v_post,cmd_omega_pre,cmd_omega_post,x1_v,x1_omega,x1_s,x2_v,x2_omega,x2_s,x3_v,x3_omega,x3_s";
    first_shot.layout.dim[0].size = 23;
    first_shot.layout.dim[0].stride = 23;
    first_shot.data.resize(23, 0.0f);
    first_shot.data[0] = fs.success ? 1.0f : 0.0f;
    first_shot.data[1] = static_cast<float>(fs.status_code);
    first_shot.data[2] = static_cast<float>(fs.progress_s);
    first_shot.data[3] = static_cast<float>(fs.progress_abs_s);
    first_shot.data[4] = static_cast<float>(fs.x0_v);
    first_shot.data[5] = static_cast<float>(fs.x0_omega);
    first_shot.data[6] = static_cast<float>(fs.x0_s);
    first_shot.data[7] = static_cast<float>(fs.u0_a);
    first_shot.data[8] = static_cast<float>(fs.u0_alpha);
    first_shot.data[9] = static_cast<float>(fs.u0_v_s);
    first_shot.data[10] = static_cast<float>(fs.cmd_v_pre_clamp);
    first_shot.data[11] = static_cast<float>(fs.cmd_v_post_clamp);
    first_shot.data[12] = static_cast<float>(fs.cmd_omega_pre_clamp);
    first_shot.data[13] = static_cast<float>(fs.cmd_omega_post_clamp);
    first_shot.data[14] = static_cast<float>(fs.x1_v);
    first_shot.data[15] = static_cast<float>(fs.x1_omega);
    first_shot.data[16] = static_cast<float>(fs.x1_s);
    first_shot.data[17] = static_cast<float>(fs.x2_v);
    first_shot.data[18] = static_cast<float>(fs.x2_omega);
    first_shot.data[19] = static_cast<float>(fs.x2_s);
    first_shot.data[20] = static_cast<float>(fs.x3_v);
    first_shot.data[21] = static_cast<float>(fs.x3_omega);
    first_shot.data[22] = static_cast<float>(fs.x3_s);
    first_shot_pub_.publish(first_shot);

    const auto& pr = output.projector_debug;
    std_msgs::Float32MultiArray projector;
    projector.layout.dim.resize(1);
    projector.layout.dim[0].label =
        "raw_valid,raw_s,raw_distance,raw_signed_distance,raw_x,raw_y,raw_yaw,guarded_valid,guarded_s,guarded_distance,guarded_signed_distance,guarded_x,guarded_y,guarded_yaw,min_progress_s,monotonic_clip_applied";
    projector.layout.dim[0].size = 16;
    projector.layout.dim[0].stride = 16;
    projector.data.resize(16, 0.0f);
    projector.data[0] = pr.raw_valid ? 1.0f : 0.0f;
    projector.data[1] = static_cast<float>(pr.raw_s);
    projector.data[2] = static_cast<float>(pr.raw_distance);
    projector.data[3] = static_cast<float>(pr.raw_signed_distance);
    projector.data[4] = static_cast<float>(pr.raw_x);
    projector.data[5] = static_cast<float>(pr.raw_y);
    projector.data[6] = static_cast<float>(pr.raw_yaw);
    projector.data[7] = pr.guarded_valid ? 1.0f : 0.0f;
    projector.data[8] = static_cast<float>(pr.guarded_s);
    projector.data[9] = static_cast<float>(pr.guarded_distance);
    projector.data[10] = static_cast<float>(pr.guarded_signed_distance);
    projector.data[11] = static_cast<float>(pr.guarded_x);
    projector.data[12] = static_cast<float>(pr.guarded_y);
    projector.data[13] = static_cast<float>(pr.guarded_yaw);
    projector.data[14] = static_cast<float>(pr.min_progress_s);
    projector.data[15] = pr.monotonic_clip_applied ? 1.0f : 0.0f;
    projector_pub_.publish(projector);

    const auto& s0ref = output.stage0_reference_debug;
    std_msgs::Float32MultiArray stage0_reference;
    stage0_reference.layout.dim.resize(1);
    stage0_reference.layout.dim[0].label =
        "s0,ref_x,ref_y,ref_yaw,ref_kappa,robot_x,robot_y,robot_yaw,yaw_error,contour_error,lag_error";
    stage0_reference.layout.dim[0].size = 11;
    stage0_reference.layout.dim[0].stride = 11;
    stage0_reference.data.resize(11, 0.0f);
    stage0_reference.data[0] = static_cast<float>(s0ref.s0);
    stage0_reference.data[1] = static_cast<float>(s0ref.ref_x);
    stage0_reference.data[2] = static_cast<float>(s0ref.ref_y);
    stage0_reference.data[3] = static_cast<float>(s0ref.ref_yaw);
    stage0_reference.data[4] = static_cast<float>(s0ref.ref_kappa);
    stage0_reference.data[5] = static_cast<float>(s0ref.robot_x);
    stage0_reference.data[6] = static_cast<float>(s0ref.robot_y);
    stage0_reference.data[7] = static_cast<float>(s0ref.robot_yaw);
    stage0_reference.data[8] = static_cast<float>(s0ref.yaw_error);
    stage0_reference.data[9] = static_cast<float>(s0ref.contour_error);
    stage0_reference.data[10] = static_cast<float>(s0ref.lag_error);
    stage0_reference_pub_.publish(stage0_reference);

    std_msgs::Float32MultiArray local_traj_head;
    local_traj_head.layout.dim.resize(1);
    local_traj_head.layout.dim[0].label =
        "valid0,x0,y0,yaw0,v0,omega0,s0,proj_s0,proj_distance0,proj_signed_distance0,contour0,lag0,yaw_error0,valid1,x1,y1,yaw1,v1,omega1,s1,proj_s1,proj_distance1,proj_signed_distance1,contour1,lag1,yaw_error1,valid2,x2,y2,yaw2,v2,omega2,s2,proj_s2,proj_distance2,proj_signed_distance2,contour2,lag2,yaw_error2";
    local_traj_head.layout.dim[0].size = 39;
    local_traj_head.layout.dim[0].stride = 39;
    local_traj_head.data.resize(39, 0.0f);
    for (int i = 0; i < 3; ++i) {
        const auto& pt = output.local_traj_head_debug.points[i];
        const int o = 13 * i;
        local_traj_head.data[o + 0] = pt.valid ? 1.0f : 0.0f;
        local_traj_head.data[o + 1] = static_cast<float>(pt.x);
        local_traj_head.data[o + 2] = static_cast<float>(pt.y);
        local_traj_head.data[o + 3] = static_cast<float>(pt.yaw);
        local_traj_head.data[o + 4] = static_cast<float>(pt.v);
        local_traj_head.data[o + 5] = static_cast<float>(pt.omega);
        local_traj_head.data[o + 6] = static_cast<float>(pt.s);
        local_traj_head.data[o + 7] = static_cast<float>(pt.proj_s);
        local_traj_head.data[o + 8] = static_cast<float>(pt.proj_distance);
        local_traj_head.data[o + 9] = static_cast<float>(pt.proj_signed_distance);
        local_traj_head.data[o + 10] = static_cast<float>(pt.contour_error);
        local_traj_head.data[o + 11] = static_cast<float>(pt.lag_error);
        local_traj_head.data[o + 12] = static_cast<float>(pt.yaw_error);
    }
    local_traj_head_pub_.publish(local_traj_head);

    std_msgs::Float32MultiArray warm_start_head;
    warm_start_head.layout.dim.resize(1);
    warm_start_head.layout.dim[0].label =
        "valid0,state_s0,state_omega0,control_alpha0,control_vs0,valid1,state_s1,state_omega1,control_alpha1,control_vs1,valid2,state_s2,state_omega2,control_alpha2,control_vs2";
    warm_start_head.layout.dim[0].size = 15;
    warm_start_head.layout.dim[0].stride = 15;
    warm_start_head.data.resize(15, 0.0f);
    for (int i = 0; i < 3; ++i) {
        const auto& pt = output.warm_start_head_debug.points[i];
        const int o = 5 * i;
        warm_start_head.data[o + 0] = pt.valid ? 1.0f : 0.0f;
        warm_start_head.data[o + 1] = static_cast<float>(pt.state_s);
        warm_start_head.data[o + 2] = static_cast<float>(pt.state_omega);
        warm_start_head.data[o + 3] = static_cast<float>(pt.control_alpha);
        warm_start_head.data[o + 4] = static_cast<float>(pt.control_v_s);
    }
    warm_start_head_pub_.publish(warm_start_head);

    const auto& td = output.terminal_diagnostics;
    std_msgs::Float32MultiArray terminal;
    terminal.layout.dim.resize(1);
    terminal.layout.dim[0].label =
        "enabled,terminal_phase,pre_terminal_phase,envelope_active,stop_pending,position_reached,speed_gate_reached,omega_gate_reached,reached,distance_to_goal,remaining_s,dx_robot,v_envelope,cmd_v_pre_clamp,cmd_v_post_clamp";
    terminal.layout.dim[0].size = 15;
    terminal.layout.dim[0].stride = 15;
    terminal.data.resize(15, 0.0f);
    terminal.data[0] = td.enabled ? 1.0f : 0.0f;
    terminal.data[1] = td.terminal_phase ? 1.0f : 0.0f;
    terminal.data[2] = td.pre_terminal_phase ? 1.0f : 0.0f;
    terminal.data[3] = td.envelope_active ? 1.0f : 0.0f;
    terminal.data[4] = td.stop_pending ? 1.0f : 0.0f;
    terminal.data[5] = td.position_reached ? 1.0f : 0.0f;
    terminal.data[6] = td.speed_gate_reached ? 1.0f : 0.0f;
    terminal.data[7] = td.omega_gate_reached ? 1.0f : 0.0f;
    terminal.data[8] = td.reached ? 1.0f : 0.0f;
    terminal.data[9] = static_cast<float>(td.distance_to_goal);
    terminal.data[10] = static_cast<float>(td.remaining_s);
    terminal.data[11] = static_cast<float>(td.dx_robot);
    terminal.data[12] = static_cast<float>(td.v_envelope);
    terminal.data[13] = static_cast<float>(td.cmd_v_pre_clamp);
    terminal.data[14] = static_cast<float>(td.cmd_v_post_clamp);
    terminal_pub_.publish(terminal);

    std_msgs::String terminal_mode;
    terminal_mode.data = td.mode;
    terminal_mode_pub_.publish(terminal_mode);

    const auto& sl = output.start_lock_recovery;
    std_msgs::Float32 start_lock_active;
    start_lock_active.data = sl.active ? 1.0f : 0.0f;
    start_lock_active_pub_.publish(start_lock_active);

    std_msgs::String start_lock_mode;
    start_lock_mode.data = sl.mode;
    start_lock_mode_pub_.publish(start_lock_mode);

    std_msgs::Float32MultiArray start_lock;
    start_lock.layout.dim.resize(1);
    start_lock.layout.dim[0].label =
        "enabled,detect_only,active,near_start,stall_progress,cmd_suppressed,warmstart_requests_motion,solver_rejects_progress,monotonic_clip_active,projection_distance_unsafe,stall_time_sec,active_count,progress_abs_s,progress_delta_s,projector_raw_s,projector_guarded_s,guard_minus_raw_s,projector_distance,cmd_v,robot_v,warm_start_v_s0,first_shot_u0_vs";
    start_lock.layout.dim[0].size = 22;
    start_lock.layout.dim[0].stride = 22;
    start_lock.data.resize(22, 0.0f);
    start_lock.data[0] = sl.enabled ? 1.0f : 0.0f;
    start_lock.data[1] = sl.detect_only ? 1.0f : 0.0f;
    start_lock.data[2] = sl.active ? 1.0f : 0.0f;
    start_lock.data[3] = sl.near_start ? 1.0f : 0.0f;
    start_lock.data[4] = sl.stall_progress ? 1.0f : 0.0f;
    start_lock.data[5] = sl.cmd_suppressed ? 1.0f : 0.0f;
    start_lock.data[6] = sl.warmstart_requests_motion ? 1.0f : 0.0f;
    start_lock.data[7] = sl.solver_rejects_progress ? 1.0f : 0.0f;
    start_lock.data[8] = sl.monotonic_clip_active ? 1.0f : 0.0f;
    start_lock.data[9] = sl.projection_distance_unsafe ? 1.0f : 0.0f;
    start_lock.data[10] = static_cast<float>(sl.stall_time_sec);
    start_lock.data[11] = static_cast<float>(sl.active_count);
    start_lock.data[12] = static_cast<float>(sl.progress_abs_s);
    start_lock.data[13] = static_cast<float>(sl.progress_delta_s);
    start_lock.data[14] = static_cast<float>(sl.projector_raw_s);
    start_lock.data[15] = static_cast<float>(sl.projector_guarded_s);
    start_lock.data[16] = static_cast<float>(sl.guard_minus_raw_s);
    start_lock.data[17] = static_cast<float>(sl.projector_distance);
    start_lock.data[18] = static_cast<float>(sl.cmd_v);
    start_lock.data[19] = static_cast<float>(sl.robot_v);
    start_lock.data[20] = static_cast<float>(sl.warm_start_v_s0);
    start_lock.data[21] = static_cast<float>(sl.first_shot_u0_v_s);
    start_lock_debug_pub_.publish(start_lock);

    std_msgs::Float32MultiArray cost;
    cost.layout.dim.resize(1);
    cost.layout.dim[0].label =
        "total,J_contour,J_lag,J_progress,J_v,J_control,J_smooth,J_terminal,J_corridor,J_obstacle,J_slosh_eta,J_slosh_eta_dot,pct_contour,pct_lag,pct_progress,pct_v,pct_control,pct_smooth,pct_terminal,pct_corridor,pct_obstacle,pct_slosh_total";
    cost.layout.dim[0].size = 22;
    cost.layout.dim[0].stride = 22;
    cost.data.assign(22, 0.0f);
    const double total = output.cost.total();
    // 占比分母用各项绝对值之和, 而非 |total|: 后者含负的 J_progress 奖励, total 近零时百分比会爆炸。
    const auto& c = output.cost;
    const double abs_sum =
        std::abs(c.J_contour) + std::abs(c.J_lag) + std::abs(c.J_progress) + std::abs(c.J_v) +
        std::abs(c.J_control) + std::abs(c.J_smooth) + std::abs(c.J_terminal) + std::abs(c.J_corridor) +
        std::abs(c.J_obstacle) + std::abs(c.J_slosh_eta) + std::abs(c.J_slosh_eta_dot);
    const double denom = abs_sum > 1e-9 ? abs_sum : 1.0;
    cost.data[0] = static_cast<float>(total);
    cost.data[1] = static_cast<float>(output.cost.J_contour);
    cost.data[2] = static_cast<float>(output.cost.J_lag);
    cost.data[3] = static_cast<float>(output.cost.J_progress);
    cost.data[4] = static_cast<float>(output.cost.J_v);
    cost.data[5] = static_cast<float>(output.cost.J_control);
    cost.data[6] = static_cast<float>(output.cost.J_smooth);
    cost.data[7] = static_cast<float>(output.cost.J_terminal);
    cost.data[8] = static_cast<float>(output.cost.J_corridor);
    cost.data[9] = static_cast<float>(output.cost.J_obstacle);
    cost.data[10] = static_cast<float>(output.cost.J_slosh_eta);
    cost.data[11] = static_cast<float>(output.cost.J_slosh_eta_dot);
    cost.data[12] = static_cast<float>(100.0 * output.cost.J_contour / denom);
    cost.data[13] = static_cast<float>(100.0 * output.cost.J_lag / denom);
    cost.data[14] = static_cast<float>(100.0 * output.cost.J_progress / denom);
    cost.data[15] = static_cast<float>(100.0 * output.cost.J_v / denom);
    cost.data[16] = static_cast<float>(100.0 * output.cost.J_control / denom);
    cost.data[17] = static_cast<float>(100.0 * output.cost.J_smooth / denom);
    cost.data[18] = static_cast<float>(100.0 * output.cost.J_terminal / denom);
    cost.data[19] = static_cast<float>(100.0 * output.cost.J_corridor / denom);
    cost.data[20] = static_cast<float>(100.0 * output.cost.J_obstacle / denom);
    cost.data[21] = static_cast<float>(100.0 * (output.cost.J_slosh_eta + output.cost.J_slosh_eta_dot) / denom);
    cost_breakdown_pub_.publish(cost);

    const auto& sm = output.slosh_cost_monitor;
    std_msgs::Float32MultiArray slosh_cost;
    slosh_cost.layout.dim.resize(1);
    slosh_cost.layout.dim[0].label =
        "J_slosh_eta,J_slosh_eta_dot,J_slosh_total,abs_cost_sum,pct_slosh_total_abs_sum,pct_eta_in_slosh,pct_eta_dot_in_slosh,eta_ref,eta_dot_ref,omega_n,height_coeff,slosh_eta_dot_ratio,eta_norm_peak,eta_dot_norm_peak";
    slosh_cost.layout.dim[0].size = 14;
    slosh_cost.layout.dim[0].stride = 14;
    slosh_cost.data.resize(14, 0.0f);
    slosh_cost.data[0] = static_cast<float>(sm.J_slosh_eta);
    slosh_cost.data[1] = static_cast<float>(sm.J_slosh_eta_dot);
    slosh_cost.data[2] = static_cast<float>(sm.J_slosh_total);
    slosh_cost.data[3] = static_cast<float>(sm.abs_cost_sum);
    slosh_cost.data[4] = static_cast<float>(sm.pct_slosh_total_abs_sum);
    slosh_cost.data[5] = static_cast<float>(sm.pct_eta_in_slosh);
    slosh_cost.data[6] = static_cast<float>(sm.pct_eta_dot_in_slosh);
    slosh_cost.data[7] = static_cast<float>(sm.eta_ref);
    slosh_cost.data[8] = static_cast<float>(sm.eta_dot_ref);
    slosh_cost.data[9] = static_cast<float>(sm.omega_n);
    slosh_cost.data[10] = static_cast<float>(sm.height_coeff);
    slosh_cost.data[11] = static_cast<float>(sm.slosh_eta_dot_ratio);
    slosh_cost.data[12] = static_cast<float>(sm.eta_norm_peak);
    slosh_cost.data[13] = static_cast<float>(sm.eta_dot_norm_peak);
    slosh_cost_monitor_pub_.publish(slosh_cost);

    std_msgs::Float32MultiArray corridor;
    corridor.layout.dim.resize(1);
    corridor.layout.dim[0].label =
        "width,half_width,max_contour_error,max_violation,violation_count,hard_bound_violated";
    corridor.layout.dim[0].size = 6;
    corridor.layout.dim[0].stride = 6;
    corridor.data.resize(6, 0.0f);
    corridor.data[0] = static_cast<float>(output.corridor_summary.width);
    corridor.data[1] = static_cast<float>(output.corridor_summary.half_width);
    corridor.data[2] = static_cast<float>(output.corridor_summary.max_contour_error);
    corridor.data[3] = static_cast<float>(output.corridor_summary.max_violation);
    corridor.data[4] = static_cast<float>(output.corridor_summary.violation_count);
    corridor.data[5] = output.corridor_summary.hard_bound_violated ? 1.0f : 0.0f;
    corridor_pub_.publish(corridor);

    std_msgs::Float32MultiArray guidance;
    guidance.layout.dim.resize(1);
    guidance.layout.dim[0].label = "guidance_id,lateral_bias";
    guidance.layout.dim[0].size = 2;
    guidance.layout.dim[0].stride = 2;
    guidance.data.resize(2, 0.0f);
    guidance.data[0] = static_cast<float>(output.guidance_summary.guidance_id);
    guidance.data[1] = static_cast<float>(output.guidance_summary.lateral_bias);
    guidance_pub_.publish(guidance);

    std_msgs::Float32MultiArray primitive;
    primitive.layout.dim.resize(1);
    primitive.layout.dim[0].label =
        "primitive_id,v_start_scale,v_mid_scale,v_end_scale,omega_start_scale,omega_mid_scale,omega_end_scale";
    primitive.layout.dim[0].size = 7;
    primitive.layout.dim[0].stride = 7;
    primitive.data.resize(7, 0.0f);
    primitive.data[0] = static_cast<float>(output.primitive_summary.primitive_id);
    primitive.data[1] = static_cast<float>(output.primitive_summary.v_start_scale);
    primitive.data[2] = static_cast<float>(output.primitive_summary.v_mid_scale);
    primitive.data[3] = static_cast<float>(output.primitive_summary.v_end_scale);
    primitive.data[4] = static_cast<float>(output.primitive_summary.omega_start_scale);
    primitive.data[5] = static_cast<float>(output.primitive_summary.omega_mid_scale);
    primitive.data[6] = static_cast<float>(output.primitive_summary.omega_end_scale);
    primitive_pub_.publish(primitive);

    std_msgs::Float32MultiArray summary;
    summary.layout.dim.resize(1);
    summary.layout.dim[0].label =
        "h_peak_pred_mm,h_p95_pred_mm,eta_x_peak,eta_y_peak,eta_dot_norm_peak,peak_k";
    summary.layout.dim[0].size = 6;
    summary.layout.dim[0].stride = 6;
    summary.data.resize(6, 0.0f);
    // 高度字段(0,1)统一发布为 mm(内部为米); eta/eta_dot 峰值仍是模态量, 不转换。
    summary.data[0] = static_cast<float>(1000.0 * output.slosh_summary.h_peak_pred);
    summary.data[1] = static_cast<float>(1000.0 * output.slosh_summary.h_p95_pred);
    summary.data[2] = static_cast<float>(output.slosh_summary.eta_x_peak);
    summary.data[3] = static_cast<float>(output.slosh_summary.eta_y_peak);
    summary.data[4] = static_cast<float>(output.slosh_summary.eta_dot_norm_peak);
    summary.data[5] = static_cast<float>(output.slosh_summary.peak_k);
    slosh_horizon_summary_pub_.publish(summary);

    std_msgs::Float32MultiArray hard;
    hard.layout.dim.resize(1);
    hard.layout.dim[0].label = "enabled,h_limit_mm,h_peak_pred_mm,margin_mm,peak_k";
    hard.layout.dim[0].size = 5;
    hard.layout.dim[0].stride = 5;
    hard.data.resize(5, 0.0f);
    hard.data[0] = output.slosh_summary.hard_constraint_enable ? 1.0f : 0.0f;
    hard.data[1] = static_cast<float>(1000.0 * output.slosh_summary.h_limit);
    hard.data[2] = static_cast<float>(1000.0 * output.slosh_summary.h_peak_pred);
    hard.data[3] = static_cast<float>(1000.0 * output.slosh_summary.h_limit_margin);
    hard.data[4] = static_cast<float>(output.slosh_summary.peak_k);
    slosh_hard_constraint_pub_.publish(hard);

    const auto& hc = output.slosh_hard_constraint;
    std_msgs::Float32MultiArray hard_effective;
    hard_effective.layout.dim.resize(1);
    hard_effective.layout.dim[0].label =
        "enabled,h_limit_mm,height_coeff,eta_max,eta_max_sq,h_peak_pred_mm,margin_mm,peak_k,modal_only,observer_uses_parabola";
    hard_effective.layout.dim[0].size = 10;
    hard_effective.layout.dim[0].stride = 10;
    hard_effective.data.resize(10, 0.0f);
    hard_effective.data[0] = hc.enabled ? 1.0f : 0.0f;
    hard_effective.data[1] = static_cast<float>(1000.0 * hc.h_limit);
    hard_effective.data[2] = static_cast<float>(hc.height_coeff);
    hard_effective.data[3] = static_cast<float>(hc.eta_max);
    hard_effective.data[4] = static_cast<float>(hc.eta_max_sq);
    hard_effective.data[5] = static_cast<float>(1000.0 * hc.h_peak_pred);
    hard_effective.data[6] = static_cast<float>(1000.0 * hc.h_limit_margin);
    hard_effective.data[7] = static_cast<float>(hc.peak_k);
    hard_effective.data[8] = hc.modal_only ? 1.0f : 0.0f;
    hard_effective.data[9] = hc.observer_uses_parabola ? 1.0f : 0.0f;
    slosh_hard_constraint_effective_pub_.publish(hard_effective);
}

void DiagnosticsPublisher::publishSloshState(const SloshState& state) {
    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "eta_x,eta_x_dot,eta_y,eta_y_dot";
    msg.layout.dim[0].size = 4;
    msg.layout.dim[0].stride = 4;
    msg.data.resize(4, 0.0f);
    msg.data[0] = static_cast<float>(state.eta_x);
    msg.data[1] = static_cast<float>(state.eta_x_dot);
    msg.data[2] = static_cast<float>(state.eta_y);
    msg.data[3] = static_cast<float>(state.eta_y_dot);
    slosh_state_pub_.publish(msg);
}

void DiagnosticsPublisher::publishSloshHeight(double height_m) {
    // 内部物理为米(SI); 发布边界统一转 mm, 与 RGB /liquid/height 同单位。
    std_msgs::Float32 msg;
    msg.data = static_cast<float>(1000.0 * height_m);
    slosh_height_pub_.publish(msg);
}

void DiagnosticsPublisher::publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
}

nav_msgs::Path DiagnosticsPublisher::makePathMsg(
    const SolverOutput& output,
    const std::string& frame_id) const {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id.empty() ? "map" : frame_id;
    path.poses.reserve(output.trajectory.size());

    for (const auto& p : output.trajectory) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    }
    return path;
}

}  // namespace spmpc_local_planner
