#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/core/variant_config.h"
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

namespace spmpc_local_planner {

class DiagnosticsPublisher {
public:
    void initialize(ros::NodeHandle& nh);
    void publishVariant(const VariantConfig& variant, const std::string& experiment_mode);
    void publishSolverBackend(const std::string& solver_backend);
    void publishOutput(const SolverOutput& output, const std::string& frame_id);
    void publishSloshState(const SloshState& state);
    void publishSloshHeight(double height_m);
    void publishStatus(const std::string& status);

private:
    nav_msgs::Path makePathMsg(const SolverOutput& output, const std::string& frame_id) const;

    ros::Publisher status_pub_;
    ros::Publisher variant_pub_;
    ros::Publisher experiment_mode_pub_;
    ros::Publisher solver_backend_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher progress_pub_;
    ros::Publisher solver_time_pub_;
    ros::Publisher cost_breakdown_pub_;
    ros::Publisher corridor_pub_;
    ros::Publisher guidance_pub_;
    ros::Publisher primitive_pub_;
    ros::Publisher slosh_state_pub_;
    ros::Publisher slosh_height_pub_;
    ros::Publisher slosh_horizon_summary_pub_;
    ros::Publisher warm_start_pub_;
    ros::Publisher warm_start_status_pub_;
    ros::Publisher runtime_bounds_pub_;
    ros::Publisher generated_bounds_pub_;
    ros::Publisher first_shot_pub_;
    ros::Publisher projector_pub_;
    ros::Publisher stage0_reference_pub_;
    ros::Publisher local_traj_head_pub_;
    ros::Publisher warm_start_head_pub_;
    ros::Publisher terminal_pub_;
    ros::Publisher terminal_mode_pub_;
    ros::Publisher start_lock_active_pub_;
    ros::Publisher start_lock_mode_pub_;
    ros::Publisher start_lock_debug_pub_;
};

}  // namespace spmpc_local_planner
