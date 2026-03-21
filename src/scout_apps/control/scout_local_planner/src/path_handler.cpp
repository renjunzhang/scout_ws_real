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

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool hasUsableOrientation(const geometry_msgs::Quaternion& q_msg) {
    const double norm_sq =
        q_msg.x * q_msg.x + q_msg.y * q_msg.y +
        q_msg.z * q_msg.z + q_msg.w * q_msg.w;
    return std::isfinite(norm_sq) && norm_sq > 1e-9;
}

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

    // ---------- 路径相似性检测 ----------
    // 若新路径与当前路径"相似"（终点偏差和路径长度比均在阈值内），
    // 则跳过 s_global / warm-start 的完整重置，避免 MPC 参考突变。
    bool path_similar = false;
    if (has_path_ && global_cache_valid_ && global_points_map_.size() >= 2 &&
        path.poses.size() >= 2 && params_.path_change_threshold > 0.0) {
        // 终点距离
        Eigen::Vector2d new_end(path.poses.back().pose.position.x,
                                path.poses.back().pose.position.y);
        double end_dist = (new_end - global_points_map_.back()).norm();

        // 新路径长度（快速累加）
        double new_length = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
            double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
            new_length += std::sqrt(dx*dx + dy*dy);
        }
        double old_length = global_path_s_.empty() ? 0.0 : global_path_s_.back();
        double length_ratio = (old_length > 0.01) ? new_length / old_length : 0.0;

        // 采样中点距离（如果路径足够长）
        double mid_dist = 0.0;
        if (path.poses.size() >= 4 && global_points_map_.size() >= 4) {
            Eigen::Vector2d new_mid(
                path.poses[path.poses.size() / 2].pose.position.x,
                path.poses[path.poses.size() / 2].pose.position.y);
            Eigen::Vector2d old_mid = global_points_map_[global_points_map_.size() / 2];
            mid_dist = (new_mid - old_mid).norm();
        }

        path_similar = (end_dist < params_.path_change_threshold &&
                        mid_dist < params_.path_change_threshold * 2.0 &&
                        length_ratio > 0.7 && length_ratio < 1.3);
    }
    
    global_path_ = path;
    path_timestamp_ = ros::Time::now();
    has_path_ = true;

    if (!path_similar) {
        // 路径显著变化：完整重置弧长跟踪
        s_initialized_ = false;
        s_global_ = 0.0;
        last_projection_s_ = 0.0;
        reset_hint_ = true;
    }
    // 路径相似时：保持 s_global_ / s_initialized_ 连续，不触发 warm-start 重置
    
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

    // 缓存全局路径（map 坐标系）
    global_points_map_.clear();
    global_points_map_.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
        global_points_map_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    if (params_.resample_spacing > 0.0) {
        global_points_map_ = resamplePath(global_points_map_, params_.resample_spacing);
    }
    global_path_s_.clear();
    global_path_s_.push_back(0.0);
    for (size_t i = 1; i < global_points_map_.size(); ++i) {
        const double d = (global_points_map_[i] - global_points_map_[i - 1]).norm();
        global_path_s_.push_back(global_path_s_.back() + d);
    }
    global_cache_valid_ = global_points_map_.size() >= 2;
    // 注：reset_hint_ 已在上方 !path_similar 分支中按需设置，此处不再无条件覆盖

    // 构建全局样条（map 坐标系）
    std::vector<Eigen::Vector2d> global_points = global_points_map_;
    // 可选：B-spline 平滑（仅用于速度曲线）
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

    if (!global_cache_valid_ || global_points_map_.size() < 2 || global_path_s_.size() != global_points_map_.size()) {
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
    double goal_dist_base = 0.0;
    if (!global_path_.poses.empty()) {
        const auto& goal_pose = global_path_.poses.back().pose;
        const tf2::Vector3 goal_map(goal_pose.position.x, goal_pose.position.y, 0.0);
        const tf2::Vector3 goal_base = tf_map_to_base * goal_map;
        goal_dist_base = std::hypot(goal_base.x(), goal_base.y());
    }

    const std::vector<Eigen::Vector2d>& path_points_map = global_points_map_;
    const std::vector<double>& path_s = global_path_s_;
    
    // 2. 找最近点（map 坐标系）
    closest_idx_ = findClosestPointIndex(path_points_map, robot_pos_map);

    // 2.1 基于全局路径索引更新全局弧长（用于稳定推进）
    double s_proj = 0.0;
    if (closest_idx_ >= 0 &&
        static_cast<size_t>(closest_idx_) < path_s.size()) {
        s_proj = projectToPathS(robot_pos_map, closest_idx_, path_points_map, path_s);
    }
    if (!s_initialized_) {
        s_global_ = s_proj;
        last_projection_s_ = s_proj;
        s_initialized_ = true;
    } else {
        double ds = s_proj - last_projection_s_;
        if (std::abs(ds) < params_.s_jump_threshold) {
            if (robot_v_ > -0.05 && ds < 0.0) {
                ds = 0.0;  // 前进时不允许倒退
            }
            s_global_ += ds;
        } else {
            // 路径跳变或重规划：重置弧长
            s_global_ = s_proj;
            reset_hint_ = true;
        }
        last_projection_s_ = s_proj;
    }
    
    // 3. 截取窗口 [idx-window_back, idx+N+window_forward]
    int window_start = std::max(0, closest_idx_ - params_.window_back);
    int window_end = std::min(static_cast<int>(path_points_map.size()) - 1, 
                              closest_idx_ + N + params_.window_forward);
    
    // 允许只有 2 个点（线性样条），避免临近终点时窗口过短导致失败
    if (window_end - window_start < 1) {
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

    const double total_len_global = (!path_s.empty()) ? path_s.back() : total_len;
    double s_progress = std::max(0.0, std::min(total_len_global, s_global_));
    double s_geom_local = std::min(current_s_ + params_.lookahead_distance, total_len);
    const double decel_safety_factor = 0.8;
    double max_decel = params_.max_tan_decel > 0.0 ? params_.max_tan_decel : params_.max_tan_accel;
    max_decel *= decel_safety_factor;
    for (int k = 0; k < N; ++k) {
        ReferencePoint ref;
        
        // 沿路径推进（时间化速度）
        if (params_.time_parameterize) {
            // 速度参考用 s_progress（不加 lookahead），几何参考用 s_geom
            double s_local = s_geom_local;
            if (has_window_s) {
                double s_geom_global = s_progress + params_.lookahead_distance;
                s_geom_global = std::min(std::max(s_geom_global, s_start), s_end);
                const double window_len = std::max(1e-6, s_end - s_start);
                const double scale = total_len / window_len;
                s_local = (s_geom_global - s_start) * scale;
            }
            s_local = std::max(0.0, std::min(total_len, s_local));

            // 计算参考点信息
            Eigen::Vector2d pos = local_spline_.evaluate(s_local);
            ref.x = pos.x();
            ref.y = pos.y();
            ref.theta_path = local_spline_.evaluateTheta(s_local);
            ref.kappa = local_spline_.evaluateKappa(s_local);
            ref.s = s_local;

            const double v_des_cap = std::max(0.0, v_des);
            double v_ref = speed_profile_valid_ ? getSpeedAtS(s_progress) : v_des_cap;
            double v_curve_cap = v_des_cap;
            if (params_.max_lat_accel > 0.0) {
                const double kappa_abs = std::abs(ref.kappa);
                if (kappa_abs > 1e-4) {
                    v_curve_cap = std::min(v_curve_cap, std::sqrt(params_.max_lat_accel / kappa_abs));
                }
            }

            // 末端停车既要看路径弧长剩余，也要看欧氏 goal 距离。
            // 否则 s_progress 已到路径末端但车体仍横向偏离 goal 时，v_ref 会过早掉到 0。
            if (v_des_cap > 1e-6 && max_decel > 1e-6) {
                const double remain_s = std::max(0.0, total_len_global - s_progress);
                const double remain_goal = std::max(0.0, goal_dist_base - params_.goal_tolerance);
                if (remain_goal > remain_s + 1e-3) {
                    double v_goal_capture =
                        std::sqrt(std::max(0.0,
                                           params_.goal_speed * params_.goal_speed +
                                           2.0 * max_decel * remain_goal));
                    v_goal_capture = std::min(v_goal_capture, v_curve_cap);
                    v_ref = std::max(v_ref, v_goal_capture);
                }
            }

            v_ref = std::min(v_ref, v_curve_cap);
            v_ref = std::min(v_ref, v_des_cap);

            // 终点捕获区内保持最低参考速度，避免距离尚未达标时过早停死。
            if (goal_dist_base > params_.goal_tolerance &&
                goal_dist_base < params_.goal_capture_distance &&
                params_.goal_capture_min_speed > 1e-6) {
                v_ref = std::max(v_ref, params_.goal_capture_min_speed);
            }

            // 外部传入的 v_des 必须是硬上界。否则 goal_stop_pending_ 时即便上层将
            // v_des_cmd 压到 0，time-parameterized speed profile 仍可能把 v_ref 抬回正值，
            // 导致终点区继续前冲。
            v_ref = std::min(v_ref, v_des_cap);

            // 使用当前 s 采样点
            ref.v_ref = v_ref;
            ref.v_path = v_ref;

            // 推进到下一步（仅推进 progress，避免 lookahead 提前衰减速度）
            s_progress = std::min(s_progress + v_ref * dt, total_len_global);
            if (!has_window_s) {
                s_geom_local = std::min(s_geom_local + v_ref * dt, total_len);
            }
        } else {
            double s_local = std::min(current_s_ + params_.lookahead_distance + k * dt * v_des, total_len);
            s_local = std::min(s_local, total_len);  // 不超过样条末端

            // 计算参考点信息
            Eigen::Vector2d pos = local_spline_.evaluate(s_local);
            ref.x = pos.x();
            ref.y = pos.y();
            ref.theta_path = local_spline_.evaluateTheta(s_local);
            ref.kappa = local_spline_.evaluateKappa(s_local);

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
            ref.s = s_local;
        }
        
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
    GoalInfo goal_info;
    return computeGoalInfoLocked(goal_info) && goal_info.pose_reached;
}

double PathHandler::getGoalDistance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    GoalInfo goal_info;
    if (!computeGoalInfoLocked(goal_info)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return goal_info.dist;
}

bool PathHandler::getGoalInfo(GoalInfo& goal_info) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return computeGoalInfoLocked(goal_info);
}

double PathHandler::getMaxCurvatureAhead(double lookahead_dist, double preview_dist) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!global_spline_.isValid()) {
        return 0.0;
    }

    const double total_len = global_spline_length_ > 1e-6
        ? global_spline_length_
        : global_spline_.getTotalLength();
    if (total_len <= 1e-6) {
        return 0.0;
    }

    const double start_s = std::max(0.0, std::min(total_len, s_global_ + std::max(0.0, lookahead_dist)));
    const double end_s = std::max(start_s, std::min(total_len, start_s + std::max(0.0, preview_dist)));
    if (end_s - start_s < 1e-6) {
        return std::abs(global_spline_.evaluateKappa(start_s));
    }

    const double ds_hint = params_.resample_spacing > 1e-3 ? params_.resample_spacing : 0.05;
    const int samples = std::max(5, static_cast<int>(std::ceil((end_s - start_s) / ds_hint)) + 1);

    double max_kappa = 0.0;
    for (int i = 0; i < samples; ++i) {
        const double ratio = samples > 1 ? static_cast<double>(i) / static_cast<double>(samples - 1) : 0.0;
        const double s = start_s + (end_s - start_s) * ratio;
        max_kappa = std::max(max_kappa, std::abs(global_spline_.evaluateKappa(s)));
    }
    return max_kappa;
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

bool PathHandler::consumeResetHint() {
    std::lock_guard<std::mutex> lock(mutex_);
    bool flag = reset_hint_;
    reset_hint_ = false;
    return flag;
}

double PathHandler::getGlobalProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return s_global_;
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

    // Newton 迭代精化（最小化 ||C(s) - point||^2）
    // f(s) = (C(s)-P) · C'(s) = 0  →  s ← s - f/f'
    // f'(s) = C'(s)·C'(s) + (C(s)-P)·C''(s)
    for (int iter = 0; iter < 3; ++iter) {
        Eigen::Vector2d c_s = local_spline_.evaluate(best_s);
        double dx = local_spline_.splineX().evaluateDerivative(best_s);
        double dy = local_spline_.splineY().evaluateDerivative(best_s);
        double ddx = local_spline_.splineX().evaluateSecondDerivative(best_s);
        double ddy = local_spline_.splineY().evaluateSecondDerivative(best_s);
        Eigen::Vector2d diff = c_s - point;
        double f  = diff.x() * dx + diff.y() * dy;
        double fp = dx * dx + dy * dy + diff.x() * ddx + diff.y() * ddy;
        if (std::abs(fp) < 1e-12) break;
        double s_new = best_s - f / fp;
        s_new = std::max(0.0, std::min(total_len, s_new));
        if (std::abs(s_new - best_s) < 1e-8) break;
        best_s = s_new;
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

double PathHandler::projectToPathS(const Eigen::Vector2d& point,
                                   int closest_idx,
                                   const std::vector<Eigen::Vector2d>& points,
                                   const std::vector<double>& path_s) const {
    const int n = static_cast<int>(points.size());
    if (n < 2 || closest_idx < 0 || closest_idx >= n ||
        static_cast<int>(path_s.size()) != n) {
        return (closest_idx >= 0 && closest_idx < n && !path_s.empty())
            ? path_s[static_cast<size_t>(closest_idx)]
            : 0.0;
    }

    auto project_on_segment = [&](int i0, int i1, double& s_out, double& dist2_out) {
        const Eigen::Vector2d& a = points[static_cast<size_t>(i0)];
        const Eigen::Vector2d& b = points[static_cast<size_t>(i1)];
        Eigen::Vector2d ab = b - a;
        const double ab2 = ab.squaredNorm();
        if (ab2 < 1e-12) {
            s_out = path_s[static_cast<size_t>(i0)];
            dist2_out = (point - a).squaredNorm();
            return;
        }
        double t = (point - a).dot(ab) / ab2;
        t = std::max(0.0, std::min(1.0, t));
        const Eigen::Vector2d proj = a + t * ab;
        dist2_out = (point - proj).squaredNorm();

        double seg_len = path_s[static_cast<size_t>(i1)] - path_s[static_cast<size_t>(i0)];
        if (seg_len < 1e-9) {
            seg_len = std::sqrt(ab2);
        }
        s_out = path_s[static_cast<size_t>(i0)] + t * seg_len;
    };

    int i0 = 0;
    int i1 = 1;
    double s_best = path_s[static_cast<size_t>(closest_idx)];
    double d2_best = std::numeric_limits<double>::infinity();

    if (closest_idx <= 0) {
        i0 = 0;
        i1 = 1;
        project_on_segment(i0, i1, s_best, d2_best);
        return s_best;
    }
    if (closest_idx >= n - 1) {
        i0 = n - 2;
        i1 = n - 1;
        project_on_segment(i0, i1, s_best, d2_best);
        return s_best;
    }

    // 对相邻两段做投影，取距离更小者
    double s1 = 0.0;
    double d21 = 0.0;
    project_on_segment(closest_idx - 1, closest_idx, s1, d21);

    double s2 = 0.0;
    double d22 = 0.0;
    project_on_segment(closest_idx, closest_idx + 1, s2, d22);

    if (d21 <= d22) {
        return s1;
    }
    return s2;
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

    // 末端速度（使用 goal_tolerance 作为安全余量，提前到达容差区后即视为终点）
    const double goal_margin = std::max(0.0, params_.goal_tolerance);
    if (!speed_profile_v_.empty()) {
        speed_profile_v_.back() = std::min(speed_profile_v_.back(),
                                           std::max(0.0, params_.goal_speed));
        // 容差区内的点也设为终点速度
        for (int i = static_cast<int>(speed_profile_v_.size()) - 1; i >= 0; --i) {
            double dist_to_end = total_len - speed_profile_s_[static_cast<size_t>(i)];
            if (dist_to_end < goal_margin) {
                speed_profile_v_[static_cast<size_t>(i)] = std::max(0.0, params_.goal_speed);
            } else {
                break;
            }
        }
    }

    // 前向遍历（加速限制）
    if (params_.max_tan_accel > 0.0) {
        for (size_t i = 1; i < speed_profile_v_.size(); ++i) {
            double v_prev = speed_profile_v_[i - 1];
            double v_lim = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * params_.max_tan_accel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    // 反向遍历（减速限制，使用保守系数确保实物能停住）
    const double decel_safety_factor = 0.8;  // 保守减速系数
    double max_decel = params_.max_tan_decel > 0.0 ? params_.max_tan_decel : params_.max_tan_accel;
    max_decel *= decel_safety_factor;  // 实物减速能力通常弱于理论值
    if (max_decel > 0.0) {
        for (int i = static_cast<int>(speed_profile_v_.size()) - 2; i >= 0; --i) {
            double v_next = speed_profile_v_[i + 1];
            double v_lim = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * max_decel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    // 高斯平滑：消除 v(s) 在曲率突变处的阶梯抖动
    // 双向滑动窗口均值，半径 = 5 个采样点
    {
        const int half_w = 5;
        const size_t nv = speed_profile_v_.size();
        if (nv > static_cast<size_t>(2 * half_w + 1)) {
            std::vector<double> smoothed(nv);
            for (size_t i = 0; i < nv; ++i) {
                double sum = 0.0;
                int cnt = 0;
                for (int j = -half_w; j <= half_w; ++j) {
                    int idx = static_cast<int>(i) + j;
                    if (idx >= 0 && idx < static_cast<int>(nv)) {
                        sum += speed_profile_v_[static_cast<size_t>(idx)];
                        ++cnt;
                    }
                }
                smoothed[i] = sum / cnt;
                // 平滑结果不能超过原始约束值（取 min 保证安全）
                smoothed[i] = std::min(smoothed[i], speed_profile_v_[i]);
            }
            speed_profile_v_ = smoothed;
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

bool PathHandler::computeGoalInfoLocked(GoalInfo& info) const {
    info = GoalInfo();

    if (!has_path_ || global_path_.poses.empty() || !tf_buffer_) {
        return false;
    }

    geometry_msgs::TransformStamped tf_map_to_base;
    try {
        tf_map_to_base = tf_buffer_->lookupTransform(
            base_frame_, global_path_.header.frame_id,
            ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error in getGoalInfo: %s", ex.what());
        return false;
    }

    const auto& goal_pose = global_path_.poses.back().pose;

    tf2::Transform tf_transform;
    tf2::fromMsg(tf_map_to_base.transform, tf_transform);

    const tf2::Vector3 goal_map(goal_pose.position.x, goal_pose.position.y, 0.0);
    const tf2::Vector3 goal_base = tf_transform * goal_map;

    info.dx = goal_base.x();
    info.dy = goal_base.y();
    info.dist = std::hypot(info.dx, info.dy);
    info.bearing = std::atan2(info.dy, info.dx);

    if (hasUsableOrientation(goal_pose.orientation)) {
        const double goal_yaw_map = tf2::getYaw(goal_pose.orientation);
        const double map_to_base_yaw = tf2::getYaw(tf_transform.getRotation());
        info.goal_yaw_in_base = normalizeAngle(goal_yaw_map + map_to_base_yaw);
        info.goal_yaw_err = info.goal_yaw_in_base;
        info.has_goal_yaw = std::isfinite(info.goal_yaw_in_base);
    }

    if (!info.has_goal_yaw) {
        const size_t n = global_path_.poses.size();
        for (size_t tail = n; tail >= 2; --tail) {
            const auto& p1_pose = global_path_.poses[tail - 2].pose;
            const auto& p2_pose = global_path_.poses[tail - 1].pose;

            const tf2::Vector3 p1_map(p1_pose.position.x, p1_pose.position.y, 0.0);
            const tf2::Vector3 p2_map(p2_pose.position.x, p2_pose.position.y, 0.0);
            const tf2::Vector3 p1_base = tf_transform * p1_map;
            const tf2::Vector3 p2_base = tf_transform * p2_map;

            const double tdx = p2_base.x() - p1_base.x();
            const double tdy = p2_base.y() - p1_base.y();
            if (std::hypot(tdx, tdy) > 1e-6) {
                info.goal_yaw_in_base = std::atan2(tdy, tdx);
                info.goal_yaw_err = info.goal_yaw_in_base;
                info.has_goal_yaw = true;
                break;
            }

            if (tail == 2) {
                break;
            }
        }
    }

    info.position_reached = info.dist < params_.goal_tolerance;
    if (info.has_goal_yaw) {
        info.pose_reached =
            info.position_reached &&
            std::abs(normalizeAngle(info.goal_yaw_err)) < params_.yaw_tolerance;
    } else {
        info.pose_reached = info.position_reached;
    }

    info.valid = true;
    return true;
}

}  // namespace scout_local_planner
