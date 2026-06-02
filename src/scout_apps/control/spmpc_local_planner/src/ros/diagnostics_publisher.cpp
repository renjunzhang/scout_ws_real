#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

void DiagnosticsPublisher::initialize(ros::NodeHandle& nh) {
    status_pub_ = nh.advertise<std_msgs::String>("status", 1, true);
    variant_pub_ = nh.advertise<std_msgs::String>("controller_variant", 1, true);
    experiment_mode_pub_ = nh.advertise<std_msgs::String>("experiment_mode", 1, true);
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("local_trajectory", 1, true);
    progress_pub_ = nh.advertise<std_msgs::Float32>("debug/progress_s", 1);
    solver_time_pub_ = nh.advertise<std_msgs::Float32>("solver_time_ms", 1);
    cost_breakdown_pub_ = nh.advertise<std_msgs::Float32MultiArray>("cost_breakdown", 1);
    corridor_pub_ = nh.advertise<std_msgs::Float32MultiArray>("corridor", 1);
    guidance_pub_ = nh.advertise<std_msgs::Float32MultiArray>("guidance", 1);
    primitive_pub_ = nh.advertise<std_msgs::Float32MultiArray>("primitive", 1);
    slosh_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("debug/slosh_state", 1);
    slosh_horizon_summary_pub_ = nh.advertise<std_msgs::Float32MultiArray>("slosh_horizon_summary", 1);
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

void DiagnosticsPublisher::publishOutput(const SolverOutput& output, const std::string& frame_id) {
    trajectory_pub_.publish(makePathMsg(output, frame_id));

    std_msgs::Float32 progress;
    progress.data = static_cast<float>(output.progress_s);
    progress_pub_.publish(progress);

    std_msgs::Float32 solver_ms;
    solver_ms.data = static_cast<float>(output.solver_time_ms);
    solver_time_pub_.publish(solver_ms);

    std_msgs::Float32MultiArray cost;
    cost.layout.dim.resize(1);
    cost.layout.dim[0].label =
        "total,J_contour,J_lag,J_progress,J_v,J_control,J_smooth,J_terminal,J_corridor,J_obstacle,J_slosh_eta,J_slosh_eta_dot,pct_contour,pct_lag,pct_progress,pct_v,pct_control,pct_smooth,pct_terminal,pct_corridor,pct_obstacle,pct_slosh_total";
    cost.layout.dim[0].size = 22;
    cost.layout.dim[0].stride = 22;
    cost.data.assign(22, 0.0f);
    const double total = output.cost.total();
    const double denom = std::abs(total) > 1e-9 ? std::abs(total) : 1.0;
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
        "h_peak_pred,h_p95_pred,eta_x_peak,eta_y_peak,eta_dot_norm_peak,peak_k";
    summary.layout.dim[0].size = 6;
    summary.layout.dim[0].stride = 6;
    summary.data.resize(6, 0.0f);
    summary.data[0] = static_cast<float>(output.slosh_summary.h_peak_pred);
    summary.data[1] = static_cast<float>(output.slosh_summary.h_p95_pred);
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
