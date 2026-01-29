/**
 * @file path_handler.cpp
 * @brief 路径处理器实现
 */

#include "scout_local_planner/path_handler.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

PathHandler::PathHandler() = default;

void PathHandler::setParams(const PathHandlerParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
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
    
    // 5. 生成参考点序列
    ref_points.clear();
    ref_points.reserve(N);
    
    double total_len = local_spline_.getTotalLength();
    
    for (int k = 0; k < N; ++k) {
        ReferencePoint ref;
        
        // 沿路径推进
        double s = current_s_ + k * dt * v_des;
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
    
    return true;
}

bool PathHandler::getFrenetState(FrenetState& frenet) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || !has_robot_state_ || !local_spline_.isValid()) {
        return false;
    }
    
    // 机器人在 base_link 坐标系下位置为原点 (0, 0)
    // 航向需要从 TF 获取
    double robot_theta = tf2::getYaw(robot_pose_.pose.orientation);
    
    // 计算 Frenet 误差
    // 机器人位置相对于 base_link 是 (0, 0)
    Eigen::Vector2d robot_pos(0.0, 0.0);
    
    computeFrenetProjection(robot_pos, robot_theta, 
                            frenet.e_l, frenet.e_c, frenet.e_theta);
    
    return true;
}

bool PathHandler::isGoalReached() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || global_path_.poses.empty()) {
        return false;
    }
    
    // 获取目标点（路径最后一点）
    const auto& goal = global_path_.poses.back().pose;
    const auto& robot = robot_pose_.pose;
    
    // 计算距离
    double dx = goal.position.x - robot.position.x;
    double dy = goal.position.y - robot.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    // 计算航向误差
    double goal_yaw = tf2::getYaw(goal.orientation);
    double robot_yaw = tf2::getYaw(robot.orientation);
    double yaw_err = std::abs(goal_yaw - robot_yaw);
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
            "base_link", path_in.header.frame_id,
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
