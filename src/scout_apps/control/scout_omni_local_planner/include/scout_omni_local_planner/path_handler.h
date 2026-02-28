/**
 * @file path_handler.h
 * @brief 路径处理器
 * 
 * 负责：
 * 1. 接收全局路径
 * 2. TF 变换（map → base_link）
 * 3. 局部三次样条拟合
 * 4. 计算 Frenet 误差、曲率、切向角
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include "scout_omni_local_planner/cubic_spline.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <memory>
#include <mutex>

namespace scout_omni_local_planner {

class PathHandler {
public:
    PathHandler();
    
    void setParams(const PathHandlerParams& params);
    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    
    bool updateGlobalPath(const nav_msgs::Path& path, double v_des);
    void updateRobotState(const geometry_msgs::PoseStamped& pose,
                          double v, double omega);
    
    bool getReferencePoints(int N, double dt, double v_des,
                            std::vector<ReferencePoint>& ref_points);
    bool getFrenetState(FrenetState& frenet);
    
    bool isGoalReached() const;
    bool isPathValid() const;
    bool consumeResetHint();
    
    double getCurrentArcLength() const { return current_s_; }
    double getGlobalProgress() const;
    double getSplineTotalLength() const;
    bool getSmoothedPath(nav_msgs::Path& path_out, int num_samples) const;

private:
    bool transformPathToBaseLink(const nav_msgs::Path& path_in,
                                 std::vector<Eigen::Vector2d>& points_out);
    int findClosestPointIndex(const std::vector<Eigen::Vector2d>& points,
                              const Eigen::Vector2d& robot_pos) const;
    void computeFrenetProjection(const Eigen::Vector2d& point,
                                 double robot_theta,
                                 double& e_l, double& e_c, double& e_theta);
    bool fitLocalSpline(const std::vector<Eigen::Vector2d>& points,
                        int start_idx, int end_idx);
    bool fitLocalSpline(const std::vector<Eigen::Vector2d>& window_points);
    void updateSpeedProfile(double v_des);
    double projectToPathS(const Eigen::Vector2d& point,
                          int closest_idx,
                          const std::vector<Eigen::Vector2d>& points,
                          const std::vector<double>& path_s) const;
    double getSpeedAtS(double s) const;

private:
    PathHandlerParams params_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string base_frame_ = "base_link";
    
    nav_msgs::Path global_path_;
    ros::Time path_timestamp_;
    bool has_path_ = false;

    std::vector<Eigen::Vector2d> global_points_map_;
    std::vector<double> global_path_s_;
    bool global_cache_valid_ = false;
    bool reset_hint_ = false;
    
    geometry_msgs::PoseStamped robot_pose_;
    double robot_v_ = 0.0;
    double robot_omega_ = 0.0;
    bool has_robot_state_ = false;
    
    CubicSpline2D local_spline_;
    double current_s_ = 0.0;
    int closest_idx_ = 0;

    CubicSpline2D global_spline_;
    double global_spline_length_ = 0.0;

    std::vector<double> speed_profile_s_;
    std::vector<double> speed_profile_v_;
    double speed_profile_v_des_ = 0.0;
    bool speed_profile_valid_ = false;

    double s_global_ = 0.0;
    double last_projection_s_ = 0.0;
    bool s_initialized_ = false;
    
    mutable std::mutex mutex_;
};

}  // namespace scout_omni_local_planner
