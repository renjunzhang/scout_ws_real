/**
 * @file path_handler.cpp
 * @brief 路径处理器实现
 */

#include "scout_local_planner/path_handler.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace scout_local_planner {

namespace {

std::vector<Eigen::Vector2d> resamplePath(const std::vector<Eigen::Vector2d>& points,
                                          double spacing) {
    if (points.size() < 2 || spacing <= 1e-6) {
        return points;
    }

    std::vector<double> cum_dist;
    cum_dist.reserve(points.size());
    cum_dist.push_back(0.0);

    for (size_t i = 1; i < points.size(); ++i) {
        double d = (points[i] - points[i - 1]).norm();
        cum_dist.push_back(cum_dist.back() + d);
    }

    const double total = cum_dist.back();
    if (total <= spacing) {
        return points;
    }

    std::vector<Eigen::Vector2d> out;
    out.reserve(static_cast<size_t>(total / spacing) + 2);
    out.push_back(points.front());

    double s = spacing;
    size_t idx = 1;
    while (s < total && idx < points.size()) {
        while (idx < points.size() && cum_dist[idx] < s) {
            ++idx;
        }
        if (idx >= points.size()) {
            break;
        }
        const double s0 = cum_dist[idx - 1];
        const double s1 = cum_dist[idx];
        const double t = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
        const Eigen::Vector2d p = points[idx - 1] + t * (points[idx] - points[idx - 1]);
        out.push_back(p);
        s += spacing;
    }

    if ((out.back() - points.back()).norm() > 1e-6) {
        out.push_back(points.back());
    }

    return out;
}

std::vector<Eigen::Vector2d> bsplineSmooth(const std::vector<Eigen::Vector2d>& control_points,
                                            int samples_per_segment) {
    if (control_points.size() < 4 || samples_per_segment < 1) {
        return control_points;
    }

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(control_points.size() + 6);
    for (int i = 0; i < 3; ++i) {
        pts.push_back(control_points.front());
    }
    pts.insert(pts.end(), control_points.begin(), control_points.end());
    for (int i = 0; i < 3; ++i) {
        pts.push_back(control_points.back());
    }

    std::vector<Eigen::Vector2d> out;
    out.reserve((pts.size() - 3) * samples_per_segment + 1);

    for (size_t i = 0; i + 3 < pts.size(); ++i) {
        for (int j = 0; j < samples_per_segment; ++j) {
            double t = static_cast<double>(j) / samples_per_segment;
            double t2 = t * t;
            double t3 = t2 * t;
            double b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6.0;
            double b1 = (3 * t3 - 6 * t2 + 4) / 6.0;
            double b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0;
            double b3 = t3 / 6.0;
            Eigen::Vector2d p = b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3];
            out.push_back(p);
        }
    }

    // 末端补点
    out.push_back(control_points.back());
    return out;
}

}  // namespace

PathHandler::PathHandler() = default;

void PathHandler::setParams(const PathHandlerParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    base_frame_ = params.base_frame;
}

void PathHandler::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    tf_buffer_ = tf_buffer;
}

bool PathHandler::updateGlobalPath(const nav_msgs::Path& path, double v_des) {
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

    // 重置弧长跟踪
    s_initialized_ = false;
    s_global_ = 0.0;
    last_projection_s_ = 0.0;
    
    // 重置最近点索引
    closest_idx_ = 0;
    current_s_ = 0.0;

    // 重置速度曲线
    speed_profile_s_.clear();
    speed_profile_v_.clear();
    speed_profile_valid_ = false;
    speed_profile_v_des_ = v_des;
    global_spline_ = CubicSpline2D();
    global_spline_length_ = 0.0;

    // 构建全局样条（map 坐标系）
    std::vector<Eigen::Vector2d> global_points;
    global_points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
        global_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // 可选：重采样 + B-spline 平滑
    if (params_.resample_spacing > 0.0) {
        global_points = resamplePath(global_points, params_.resample_spacing);
    }
    if (params_.use_bspline_smoothing) {
        global_points = bsplineSmooth(global_points, params_.bspline_samples_per_segment);
    }

    // 过滤重复点（避免样条参数不单调）
    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(global_points.size());
    const double min_dist = 1e-4;
    for (const auto& p : global_points) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
            continue;
        }
        if (filtered.empty() || (p - filtered.back()).norm() > min_dist) {
            filtered.push_back(p);
        }
    }

    if (filtered.size() >= 2 && global_spline_.fit(filtered)) {
        global_spline_length_ = global_spline_.getTotalLength();
        updateSpeedProfile(v_des);
    } else {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Failed to fit global spline for speed profile");
    }
    
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
    
    // 1. 获取 map->base 变换，并在 map 坐标系中找最近点
    if (!tf_buffer_) {
        ROS_ERROR_THROTTLE(1.0, "[PathHandler] TF buffer not set");
        return false;
    }

    geometry_msgs::TransformStamped tf_map_to_base_msg;
    try {
        tf_map_to_base_msg = tf_buffer_->lookupTransform(
            base_frame_, global_path_.header.frame_id,
            ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error: %s", ex.what());
        return false;
    }

    tf2::Transform tf_map_to_base;
    tf2::fromMsg(tf_map_to_base_msg.transform, tf_map_to_base);
    const tf2::Transform tf_base_to_map = tf_map_to_base.inverse();
    const Eigen::Vector2d robot_pos_map(tf_base_to_map.getOrigin().x(),
                                        tf_base_to_map.getOrigin().y());

    // 1.1 使用 map 坐标系下的路径点（仅变换窗口）
    std::vector<Eigen::Vector2d> path_points_map;
    path_points_map.reserve(global_path_.poses.size());
    for (const auto& pose : global_path_.poses) {
        path_points_map.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // 1.2 可选：按固定间隔重采样（map 坐标系）
    if (params_.resample_spacing > 0.0) {
        path_points_map = resamplePath(path_points_map, params_.resample_spacing);
    }

    if (path_points_map.size() < 2) {
        return false;
    }

    // 1.3 计算路径累计弧长（map 坐标系）
    std::vector<double> path_s;
    path_s.reserve(path_points_map.size());
    path_s.push_back(0.0);
    for (size_t i = 1; i < path_points_map.size(); ++i) {
        const double d = (path_points_map[i] - path_points_map[i - 1]).norm();
        path_s.push_back(path_s.back() + d);
    }
    
    // 2. 找最近点（map 坐标系）
    closest_idx_ = findClosestPointIndex(path_points_map, robot_pos_map);

    // 2.1 基于全局路径索引更新全局弧长（用于稳定推进）
    double s_proj = 0.0;
    if (closest_idx_ >= 0 &&
        static_cast<size_t>(closest_idx_) < path_s.size()) {
        s_proj = path_s[static_cast<size_t>(closest_idx_)];
    }
    if (!s_initialized_) {
        s_global_ = s_proj;
        last_projection_s_ = s_proj;
        s_initialized_ = true;
    } else {
        const double ds = s_proj - last_projection_s_;
        if (std::abs(ds) < params_.s_jump_threshold) {
            s_global_ += ds;
        } else {
            // 路径跳变或重规划：重置弧长
            s_global_ = s_proj;
        }
        last_projection_s_ = s_proj;
    }
    
    // 3. 截取窗口 [idx-window_back, idx+N+window_forward]
    int window_start = std::max(0, closest_idx_ - params_.window_back);
    int window_end = std::min(static_cast<int>(path_points_map.size()) - 1, 
                              closest_idx_ + N + params_.window_forward);
    
    if (window_end - window_start < 2) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Window too small for spline fitting");
        return false;
    }
    
    // 4. 局部样条拟合（仅变换窗口，减少 TF 负担）
    std::vector<Eigen::Vector2d> window_points;
    window_points.reserve(static_cast<size_t>(window_end - window_start + 1));
    for (int i = window_start; i <= window_end; ++i) {
        const Eigen::Vector2d& p_map = path_points_map[static_cast<size_t>(i)];
        const tf2::Vector3 p_map_tf(p_map.x(), p_map.y(), 0.0);
        const tf2::Vector3 p_base_tf = tf_map_to_base * p_map_tf;
        window_points.emplace_back(p_base_tf.x(), p_base_tf.y());
    }
    if (params_.use_bspline_smoothing) {
        window_points = bsplineSmooth(window_points, params_.bspline_samples_per_segment);
    }
    if (!fitLocalSpline(window_points)) {
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

    // 4.2 速度曲线由全局路径更新时一次性计算，这里只在必要时补算
    if (params_.time_parameterize &&
        (!speed_profile_valid_ || std::abs(v_des - speed_profile_v_des_) > 1e-3)) {
        updateSpeedProfile(v_des);
    }

    // 5. 生成参考点序列
    ref_points.clear();
    ref_points.reserve(N);
    
    double total_len = local_spline_.getTotalLength();

    // 4.3 使用全局弧长映射到局部样条，减少 s 抖动
    double s_start = 0.0;
    double s_end = 0.0;
    bool has_window_s = false;
    if (!path_s.empty() && window_start >= 0 &&
        static_cast<size_t>(window_end) < path_s.size()) {
        s_start = path_s[static_cast<size_t>(window_start)];
        s_end = path_s[static_cast<size_t>(window_end)];
        const double window_len = std::max(1e-6, s_end - s_start);
        const double scale = total_len / window_len;
        double s_local = (s_global_ - s_start) * scale;
        s_local = std::max(0.0, std::min(total_len, s_local));
        current_s_ = s_local;
        has_window_s = true;
    }

    double base_s_global = s_global_ + params_.lookahead_distance;
    if (has_window_s) {
        base_s_global = std::min(std::max(base_s_global, s_start), s_end);
    }

    double s_global = has_window_s ? base_s_global : std::min(current_s_ + params_.lookahead_distance, total_len);
    for (int k = 0; k < N; ++k) {
        ReferencePoint ref;
        
        // 沿路径推进（时间化速度）
        double s_local = s_global;
        if (params_.time_parameterize) {
            double v_ref = getSpeedAtS(s_global);
            if (v_ref <= 1e-6) {
                v_ref = v_des;
            }
            double s_next = s_global + v_ref * dt;
            if (has_window_s) {
                s_global = std::min(s_next, s_end);
                const double window_len = std::max(1e-6, s_end - s_start);
                const double scale = total_len / window_len;
                s_local = (s_global - s_start) * scale;
            } else {
                s_global = std::min(s_next, total_len);
                s_local = s_global;
            }
            s_local = std::max(0.0, std::min(total_len, s_local));
            // 使用当前 s 采样点
            ref.v_ref = v_ref;
            ref.v_path = v_ref;
        } else {
            s_local = std::min(current_s_ + params_.lookahead_distance + k * dt * v_des, total_len);
            s_local = std::min(s_local, total_len);  // 不超过样条末端
        }

        // 计算参考点信息
        Eigen::Vector2d pos = local_spline_.evaluate(s_local);
        ref.x = pos.x();
        ref.y = pos.y();
        ref.theta_path = local_spline_.evaluateTheta(s_local);
        ref.kappa = local_spline_.evaluateKappa(s_local);
        if (!params_.time_parameterize) {
            double v_ref = v_des;
            if (params_.max_lat_accel > 0.0) {
                double kappa_abs = std::abs(ref.kappa);
                if (kappa_abs > 1e-4) {
                    v_ref = std::min(v_ref, std::sqrt(params_.max_lat_accel / kappa_abs));
                }
            }
            if (params_.min_ref_speed > 0.0) {
                v_ref = std::max(v_ref, params_.min_ref_speed);
            }
            ref.v_path = v_ref;
            ref.v_ref = v_ref;
        }
        ref.s = s_local;
        
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
    
    // 计算航向误差（使用路径末端切线方向，而非终点姿态）
    double goal_yaw_in_base = 0.0;
    const size_t n = global_path_.poses.size();
    if (n >= 2) {
        geometry_msgs::PoseStamped p1_map, p2_map;
        p1_map.header = global_path_.header;
        p2_map.header = global_path_.header;
        p1_map.pose = global_path_.poses[n - 2].pose;
        p2_map.pose = global_path_.poses[n - 1].pose;

        geometry_msgs::PoseStamped p1_base, p2_base;
        try {
            p1_base = tf_buffer_->transform(p1_map, base_frame_, ros::Duration(0.1));
            p2_base = tf_buffer_->transform(p2_map, base_frame_, ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error in goal yaw: %s", ex.what());
            return false;
        }

        const double dx = p2_base.pose.position.x - p1_base.pose.position.x;
        const double dy = p2_base.pose.position.y - p1_base.pose.position.y;
        if (std::hypot(dx, dy) > 1e-6) {
            goal_yaw_in_base = std::atan2(dy, dx);
        }
    }

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

int PathHandler::findClosestPointIndex(const std::vector<Eigen::Vector2d>& points,
                                       const Eigen::Vector2d& robot_pos) const {
    if (points.empty()) return 0;
    
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
    return fitLocalSpline(window_points);
}

bool PathHandler::fitLocalSpline(const std::vector<Eigen::Vector2d>& window_points) {
    if (window_points.size() < 2) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Window too small for spline fitting");
        return false;
    }

    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(window_points.size());
    const double min_dist = 1e-4;
    for (const auto& p : window_points) {
        if (filtered.empty() || (p - filtered.back()).norm() > min_dist) {
            filtered.push_back(p);
        }
    }

    if (filtered.size() < 2) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Filtered window too small for spline fitting");
        return false;
    }

    if (!local_spline_.fit(filtered)) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Failed to fit local spline");
        return false;
    }
    
    return true;
}

void PathHandler::updateSpeedProfile(double v_des) {
    speed_profile_s_.clear();
    speed_profile_v_.clear();
    speed_profile_valid_ = false;
    speed_profile_v_des_ = v_des;

    if (!params_.time_parameterize || !global_spline_.isValid()) {
        return;
    }

    const double total_len = global_spline_length_ > 1e-6
        ? global_spline_length_
        : global_spline_.getTotalLength();
    if (total_len <= 1e-6) {
        return;
    }

    const double ds = std::max(1e-3, params_.speed_profile_ds);
    const int n = static_cast<int>(std::ceil(total_len / ds)) + 1;
    speed_profile_s_.reserve(n);
    speed_profile_v_.reserve(n);

    for (int i = 0; i < n; ++i) {
        const double s = std::min(total_len, i * ds);
        speed_profile_s_.push_back(s);

        double v = v_des;
        if (params_.max_lat_accel > 0.0) {
            double kappa_abs = std::abs(global_spline_.evaluateKappa(s));
            if (kappa_abs > 1e-4) {
                v = std::min(v, std::sqrt(params_.max_lat_accel / kappa_abs));
            }
        }
        speed_profile_v_.push_back(v);
    }

    // 末端速度
    if (!speed_profile_v_.empty()) {
        speed_profile_v_.back() = std::min(speed_profile_v_.back(),
                                           std::max(0.0, params_.goal_speed));
    }

    // 前向遍历（加速限制）
    if (params_.max_tan_accel > 0.0) {
        for (size_t i = 1; i < speed_profile_v_.size(); ++i) {
            double v_prev = speed_profile_v_[i - 1];
            double v_lim = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * params_.max_tan_accel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    // 反向遍历（减速限制）
    double max_decel = params_.max_tan_decel > 0.0 ? params_.max_tan_decel : params_.max_tan_accel;
    if (max_decel > 0.0) {
        for (int i = static_cast<int>(speed_profile_v_.size()) - 2; i >= 0; --i) {
            double v_next = speed_profile_v_[i + 1];
            double v_lim = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * max_decel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    // 参考速度下限（末端除外）
    if (params_.min_ref_speed > 0.0 && speed_profile_v_.size() >= 2) {
        for (size_t i = 0; i + 1 < speed_profile_v_.size(); ++i) {
            speed_profile_v_[i] = std::max(speed_profile_v_[i], params_.min_ref_speed);
        }
    }

    speed_profile_valid_ = true;
}

double PathHandler::getSpeedAtS(double s) const {
    if (!speed_profile_valid_ || speed_profile_s_.empty() || speed_profile_v_.empty()) {
        return 0.0;
    }
    if (s <= speed_profile_s_.front()) {
        return speed_profile_v_.front();
    }
    if (s >= speed_profile_s_.back()) {
        return speed_profile_v_.back();
    }

    auto it = std::upper_bound(speed_profile_s_.begin(), speed_profile_s_.end(), s);
    size_t idx = static_cast<size_t>(std::distance(speed_profile_s_.begin(), it));
    if (idx == 0) {
        return speed_profile_v_.front();
    }
    double s0 = speed_profile_s_[idx - 1];
    double s1 = speed_profile_s_[idx];
    double v0 = speed_profile_v_[idx - 1];
    double v1 = speed_profile_v_[idx];
    double t = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
    return v0 + t * (v1 - v0);
}

}  // namespace scout_local_planner
