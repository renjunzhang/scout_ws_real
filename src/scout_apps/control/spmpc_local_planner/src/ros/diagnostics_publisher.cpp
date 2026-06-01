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
    cost_breakdown_pub_.publish(cost);
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
