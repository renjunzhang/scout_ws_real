/**
 * @file path_handler.cpp
 * @brief 路径处理器实现
 */

#include "scout_local_planner/path_handler.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

PathHandler::PathHandler() = default;

void PathHandler::setParams(const PathHandlerParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    base_frame_ = params.base_frame;
}

void PathHandler::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    tf_buffer_ = tf_buffer;
}

bool PathHandler::updateGlobalPath(const nav_msgs::Path& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查路径有效性
    if (path.poses.size() < static_cast<size_t>(params_.min_path_points)) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Path has too few points: %zu", 
                          path.poses.size());
        return false;
    }
    
    global_path_ = path;
    path_timestamp_ = ros::Time::now();
    has_path_ = true;
    
    // 重置最近点索引
    closest_idx_ = 0;
    current_s_ = 0.0;
    
    ROS_INFO("[PathHandler] Received new path with %zu points", path.poses.size());
    return true;
}

void PathHandler::updateRobotState(const geometry_msgs::PoseStamped& pose,
                                    double v, double omega) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_pose_ = pose;
    robot_v_ = v;
    robot_omega_ = omega;
    has_robot_state_ = true;
}

bool PathHandler::getReferencePoints(int N, double dt, double v_des,
                                      std::vector<ReferencePoint>& ref_points) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || !has_robot_state_) {
        return false;
    }
    
    // 1. 将路径变换到 base_link 坐标系
    std::vector<Eigen::Vector2d> path_points;
    if (!transformPathToBaseLink(global_path_, path_points)) {
        return false;
    }
    
    if (path_points.size() < 2) {
        return false;
    }
    
    // 2. 找最近点
    closest_idx_ = findClosestPointIndex(path_points);
    
    // 3. 截取窗口 [idx-2, idx+N+2]
    int window_start = std::max(0, closest_idx_ - 2);
    int window_end = std::min(static_cast<int>(path_points.size()) - 1, 
                              closest_idx_ + N + 2);
    
    if (window_end - window_start < 2) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Window too small for spline fitting");
        return false;
    }
    
    // 4. 局部样条拟合
    if (!fitLocalSpline(path_points, window_start, window_end)) {
        return false;
    }

    // 4.1 更新当前弧长位置（基于最新样条）
    {
        double tmp_e_l = 0.0;
        double tmp_e_c = 0.0;
        double tmp_e_theta = 0.0;
        const Eigen::Vector2d robot_pos(0.0, 0.0);
        const double robot_theta = 0.0;
        computeFrenetProjection(robot_pos, robot_theta,
                                tmp_e_l, tmp_e_c, tmp_e_theta);
    }

    // 5. 生成参考点序列
    ref_points.clear();
    ref_points.reserve(N);
    
    double total_len = local_spline_.getTotalLength();
    double base_s = std::min(current_s_ + params_.lookahead_distance, total_len);

    for (int k = 0; k < N; ++k) {
        ReferencePoint ref;
        
        // 沿路径推进
        double s = base_s + k * dt * v_des;
        s = std::min(s, total_len);  // 不超过样条末端
        
        // 计算参考点信息
        Eigen::Vector2d pos = local_spline_.evaluate(s);
        ref.x = pos.x();
        ref.y = pos.y();
        ref.theta_path = local_spline_.evaluateTheta(s);
        ref.kappa = local_spline_.evaluateKappa(s);
        ref.v_path = v_des;
        ref.s = s;
        ref.v_ref = v_des;  // 代价函数用
        
        ref_points.push_back(ref);
    }

    // 路径在持续使用时保持有效，避免静态路径超时
    path_timestamp_ = ros::Time::now();
    
    return true;
}

bool PathHandler::getFrenetState(FrenetState& frenet) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || !has_robot_state_ || !local_spline_.isValid()) {
        return false;
    }
    
    // 路径已变换到 base_link 坐标系
    // 在 base_link 坐标系中：
    // - 机器人位置为原点 (0, 0)
    // - 机器人航向为 0（X 轴正方向）
    double robot_theta = 0.0;  // 在 base_link 坐标系下，机器人航向始终为 0
    
    // 计算 Frenet 误差
    Eigen::Vector2d robot_pos(0.0, 0.0);
    
    computeFrenetProjection(robot_pos, robot_theta, 
                            frenet.e_l, frenet.e_c, frenet.e_theta);
    
    return true;
}

bool PathHandler::isGoalReached() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || global_path_.poses.empty() || !tf_buffer_) {
        return false;
    }
    
    // 获取目标点（路径最后一点，在 map 坐标系）
    geometry_msgs::PoseStamped goal_in_map;
    goal_in_map.header = global_path_.header;
    goal_in_map.pose = global_path_.poses.back().pose;
    
    // 将目标点变换到 base_link 坐标系
    geometry_msgs::PoseStamped goal_in_base;
    try {
        goal_in_base = tf_buffer_->transform(goal_in_map, base_frame_, ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error in isGoalReached: %s", ex.what());
        return false;
    }
    
    // 在 base_link 坐标系中，机器人在原点
    // 计算到目标的距离
    double dx = goal_in_base.pose.position.x;
    double dy = goal_in_base.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    // 计算航向误差（目标在 base_link 中的朝向）
    double goal_yaw_in_base = tf2::getYaw(goal_in_base.pose.orientation);
    double yaw_err = std::abs(goal_yaw_in_base);
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    yaw_err = std::abs(yaw_err);
    
    return (dist < params_.goal_tolerance && yaw_err < params_.yaw_tolerance);
}

bool PathHandler::isPathValid() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_) {
        return false;
    }
    
    // 检查超时
    double elapsed = (ros::Time::now() - path_timestamp_).toSec();
    if (elapsed > params_.path_timeout) {
        return false;
    }
    
    return true;
}

double PathHandler::getSplineTotalLength() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_spline_.getTotalLength();
}

bool PathHandler::getSmoothedPath(nav_msgs::Path& path_out, int num_samples) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!local_spline_.isValid()) {
        return false;
    }

    const double total_len = local_spline_.getTotalLength();
    if (total_len <= 1e-6 || num_samples < 2) {
        return false;
    }

    path_out.header.stamp = ros::Time::now();
    path_out.header.frame_id = base_frame_;
    path_out.poses.clear();
    path_out.poses.reserve(static_cast<size_t>(num_samples));

    for (int i = 0; i < num_samples; ++i) {
        const double s = total_len * static_cast<double>(i) / (num_samples - 1);
        const Eigen::Vector2d pos = local_spline_.evaluate(s);
        const double theta = local_spline_.evaluateTheta(s);

        geometry_msgs::PoseStamped pose;
        pose.header = path_out.header;
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        pose.pose.orientation = tf2::toMsg(q);

        path_out.poses.push_back(pose);
    }

    return true;
}

//==============================================================================
// 私有方法
//==============================================================================

bool PathHandler::transformPathToBaseLink(const nav_msgs::Path& path_in,
                                           std::vector<Eigen::Vector2d>& points_out) {
    if (!tf_buffer_) {
        ROS_ERROR_THROTTLE(1.0, "[PathHandler] TF buffer not set");
        return false;
    }
    
    points_out.clear();
    points_out.reserve(path_in.poses.size());
    
    geometry_msgs::TransformStamped tf_map_to_base;
    try {
        tf_map_to_base = tf_buffer_->lookupTransform(
            base_frame_, path_in.header.frame_id,
            ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error: %s", ex.what());
        return false;
    }
    
    for (const auto& pose : path_in.poses) {
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = path_in.header;
        pose_in.pose = pose.pose;
        
        tf2::doTransform(pose_in, pose_out, tf_map_to_base);
        
        points_out.emplace_back(pose_out.pose.position.x,
                                pose_out.pose.position.y);
    }
    
    return true;
}

int PathHandler::findClosestPointIndex(const std::vector<Eigen::Vector2d>& points) const {
    if (points.empty()) return 0;
    
    // 机器人在 base_link 坐标系下位置为 (0, 0)
    Eigen::Vector2d robot_pos(0.0, 0.0);
    
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    // 从上一次的索引附近开始搜索（优化）
    int search_start = std::max(0, closest_idx_ - 5);
    int search_end = std::min(static_cast<int>(points.size()), closest_idx_ + 20);
    
    // 如果索引差距太大，全局搜索
    if (closest_idx_ == 0 || search_end - search_start < 10) {
        search_start = 0;
        search_end = static_cast<int>(points.size());
    }
    
    for (int i = search_start; i < search_end; ++i) {
        double dist = (points[i] - robot_pos).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

void PathHandler::computeFrenetProjection(const Eigen::Vector2d& point,
                                           double robot_theta,
                                           double& e_l, double& e_c, double& e_theta) {
    if (!local_spline_.isValid()) {
        e_l = e_c = e_theta = 0.0;
        return;
    }
    
    // 找到样条上最近的点（简单实现：采样搜索）
    double best_s = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    double total_len = local_spline_.getTotalLength();
    
    // 粗搜索
    const int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double s = total_len * i / num_samples;
        Eigen::Vector2d p = local_spline_.evaluate(s);
        double dist = (p - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_s = s;
        }
    }
    
    // 细搜索（在最佳点附近）
    double delta = total_len / num_samples;
    double s_start = std::max(0.0, best_s - delta);
    double s_end = std::min(total_len, best_s + delta);
    
    for (int i = 0; i <= 20; ++i) {
        double s = s_start + (s_end - s_start) * i / 20;
        Eigen::Vector2d p = local_spline_.evaluate(s);
        double dist = (p - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_s = s;
        }
    }
    
    current_s_ = best_s;
    
    // 计算 Frenet 坐标
    Eigen::Vector2d closest_point = local_spline_.evaluate(best_s);
    double theta_path = local_spline_.evaluateTheta(best_s);
    
    // 误差向量
    Eigen::Vector2d error = point - closest_point;
    
    // 切向量和法向量
    Eigen::Vector2d tangent(std::cos(theta_path), std::sin(theta_path));
    Eigen::Vector2d normal(-std::sin(theta_path), std::cos(theta_path));
    
    // 纵向误差（沿切向）
    e_l = error.dot(tangent);
    
    // 横向误差（沿法向）
    e_c = error.dot(normal);
    
    // 航向误差
    e_theta = robot_theta - theta_path;
    // 归一化到 [-π, π]
    while (e_theta > M_PI) e_theta -= 2 * M_PI;
    while (e_theta < -M_PI) e_theta += 2 * M_PI;
}

bool PathHandler::fitLocalSpline(const std::vector<Eigen::Vector2d>& points,
                                  int start_idx, int end_idx) {
    std::vector<Eigen::Vector2d> window_points;
    for (int i = start_idx; i <= end_idx; ++i) {
        window_points.push_back(points[i]);
    }
    
    if (!local_spline_.fit(window_points)) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Failed to fit local spline");
        return false;
    }
    
    return true;
}

}  // namespace scout_local_planner
