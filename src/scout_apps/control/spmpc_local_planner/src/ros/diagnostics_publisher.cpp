#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

void DiagnosticsPublisher::initialize(ros::NodeHandle& nh) {
    status_pub_ = nh.advertise<std_msgs::String>("status", 1, true);
    variant_pub_ = nh.advertise<std_msgs::String>("controller_variant", 1, true);
    experiment_mode_pub_ = nh.advertise<std_msgs::String>("experiment_mode", 1, true);
    solver_backend_pub_ = nh.advertise<std_msgs::String>("solver_backend", 1, true);
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("local_trajectory", 1, true);
    progress_pub_ = nh.advertise<std_msgs::Float32>("debug/progress_s", 1);
    solver_time_pub_ = nh.advertise<std_msgs::Float32>("solver_time_ms", 1);
    cost_breakdown_pub_ = nh.advertise<std_msgs::Float32MultiArray>("cost_breakdown", 1);
    corridor_pub_ = nh.advertise<std_msgs::Float32MultiArray>("corridor", 1);
    guidance_pub_ = nh.advertise<std_msgs::Float32MultiArray>("guidance", 1);
    primitive_pub_ = nh.advertise<std_msgs::Float32MultiArray>("primitive", 1);
    slosh_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_state", 1);
    slosh_height_pub_ = nh.advertise<std_msgs::Float32>("slosh_height", 1);
    slosh_horizon_summary_pub_ = nh.advertise<std_msgs::Float32MultiArray>("slosh_horizon_summary", 1);
    warm_start_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/warm_start", 1);
    warm_start_status_pub_ = nh.advertise<std_msgs::String>("debug/warm_start_status", 1);
    terminal_pub_ = nh.advertise<std_msgs::Float32MultiArray>("terminal/debug", 1);
    terminal_mode_pub_ = nh.advertise<std_msgs::String>("terminal/mode", 1);
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

void DiagnosticsPublisher::publishOutput(const SolverOutput& output, const std::string& frame_id) {
    trajectory_pub_.publish(makePathMsg(output, frame_id));

    std_msgs::Float32 progress;
    progress.data = static_cast<float>(output.progress_s);
    progress_pub_.publish(progress);

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
