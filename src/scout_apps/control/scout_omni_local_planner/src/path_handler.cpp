/**
 * @file path_handler.cpp
 * @brief 路径处理器实现（全向轮版，与差速版逻辑完全一致）
 */

#include "scout_omni_local_planner/path_handler.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace scout_omni_local_planner {

namespace {

std::vector<Eigen::Vector2d> resamplePath(const std::vector<Eigen::Vector2d>& points,
                                          double spacing) {
    if (points.size() < 2 || spacing <= 1e-6) return points;

    std::vector<double> cum_dist;
    cum_dist.reserve(points.size());
    cum_dist.push_back(0.0);
    for (size_t i = 1; i < points.size(); ++i) {
        cum_dist.push_back(cum_dist.back() + (points[i] - points[i - 1]).norm());
    }

    const double total = cum_dist.back();
    if (total <= spacing) return points;

    std::vector<Eigen::Vector2d> out;
    out.reserve(static_cast<size_t>(total / spacing) + 2);
    out.push_back(points.front());

    double s = spacing;
    size_t idx = 1;
    while (s < total && idx < points.size()) {
        while (idx < points.size() && cum_dist[idx] < s) ++idx;
        if (idx >= points.size()) break;
        const double s0 = cum_dist[idx - 1];
        const double s1 = cum_dist[idx];
        const double t = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
        out.push_back(points[idx - 1] + t * (points[idx] - points[idx - 1]));
        s += spacing;
    }

    if ((out.back() - points.back()).norm() > 1e-6) out.push_back(points.back());
    return out;
}

std::vector<Eigen::Vector2d> bsplineSmooth(const std::vector<Eigen::Vector2d>& control_points,
                                            int samples_per_segment) {
    if (control_points.size() < 4 || samples_per_segment < 1) return control_points;

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(control_points.size() + 6);
    for (int i = 0; i < 3; ++i) pts.push_back(control_points.front());
    pts.insert(pts.end(), control_points.begin(), control_points.end());
    for (int i = 0; i < 3; ++i) pts.push_back(control_points.back());

    std::vector<Eigen::Vector2d> out;
    out.reserve((pts.size() - 3) * samples_per_segment + 1);

    for (size_t i = 0; i + 3 < pts.size(); ++i) {
        for (int j = 0; j < samples_per_segment; ++j) {
            double t = static_cast<double>(j) / samples_per_segment;
            double t2 = t * t, t3 = t2 * t;
            double b0 = (-t3 + 3 * t2 - 3 * t + 1) / 6.0;
            double b1 = (3 * t3 - 6 * t2 + 4) / 6.0;
            double b2 = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6.0;
            double b3 = t3 / 6.0;
            out.push_back(b0 * pts[i] + b1 * pts[i + 1] + b2 * pts[i + 2] + b3 * pts[i + 3]);
        }
    }
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
    
    if (path.poses.size() < static_cast<size_t>(params_.min_path_points)) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Path too few points: %zu", path.poses.size());
        return false;
    }
    
    global_path_ = path;
    path_timestamp_ = ros::Time::now();
    has_path_ = true;

    s_initialized_ = false;
    s_global_ = 0.0;
    last_projection_s_ = 0.0;
    closest_idx_ = 0;
    current_s_ = 0.0;

    speed_profile_s_.clear();
    speed_profile_v_.clear();
    speed_profile_valid_ = false;
    speed_profile_v_des_ = v_des;
    global_spline_ = CubicSpline2D();
    global_spline_length_ = 0.0;

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
        global_path_s_.push_back(global_path_s_.back() + (global_points_map_[i] - global_points_map_[i - 1]).norm());
    }
    global_cache_valid_ = global_points_map_.size() >= 2;
    reset_hint_ = true;

    std::vector<Eigen::Vector2d> global_points = global_points_map_;
    if (params_.use_bspline_smoothing) {
        global_points = bsplineSmooth(global_points, params_.bspline_samples_per_segment);
    }

    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(global_points.size());
    const double min_dist = 1e-4;
    for (const auto& p : global_points) {
        if (!std::isfinite(p.x()) || !std::isfinite(p.y())) continue;
        if (filtered.empty() || (p - filtered.back()).norm() > min_dist) {
            filtered.push_back(p);
        }
    }

    if (filtered.size() >= 2 && global_spline_.fit(filtered)) {
        global_spline_length_ = global_spline_.getTotalLength();
        updateSpeedProfile(v_des);
    }
    
    ROS_INFO("[PathHandler] Received path with %zu points", path.poses.size());
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
    
    if (!has_path_ || !has_robot_state_) return false;
    if (!global_cache_valid_ || global_points_map_.size() < 2 || 
        global_path_s_.size() != global_points_map_.size()) return false;
    
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

    const std::vector<Eigen::Vector2d>& path_points_map = global_points_map_;
    const std::vector<double>& path_s = global_path_s_;
    
    closest_idx_ = findClosestPointIndex(path_points_map, robot_pos_map);

    double s_proj = 0.0;
    if (closest_idx_ >= 0 && static_cast<size_t>(closest_idx_) < path_s.size()) {
        s_proj = projectToPathS(robot_pos_map, closest_idx_, path_points_map, path_s);
    }
    if (!s_initialized_) {
        s_global_ = s_proj;
        last_projection_s_ = s_proj;
        s_initialized_ = true;
    } else {
        double ds = s_proj - last_projection_s_;
        if (std::abs(ds) < params_.s_jump_threshold) {
            if (robot_v_ > -0.05 && ds < 0.0) ds = 0.0;
            s_global_ += ds;
        } else {
            s_global_ = s_proj;
            reset_hint_ = true;
        }
        last_projection_s_ = s_proj;
    }
    
    int window_start = std::max(0, closest_idx_ - params_.window_back);
    int window_end = std::min(static_cast<int>(path_points_map.size()) - 1, 
                              closest_idx_ + N + params_.window_forward);
    
    if (window_end - window_start < 1) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] Window too small");
        return false;
    }
    
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
    if (!fitLocalSpline(window_points)) return false;

    {
        double tmp_e_l, tmp_e_c, tmp_e_theta;
        computeFrenetProjection(Eigen::Vector2d(0.0, 0.0), 0.0, tmp_e_l, tmp_e_c, tmp_e_theta);
    }

    if (params_.time_parameterize &&
        (!speed_profile_valid_ || std::abs(v_des - speed_profile_v_des_) > 1e-3)) {
        updateSpeedProfile(v_des);
    }

    ref_points.clear();
    ref_points.reserve(N);
    
    double total_len = local_spline_.getTotalLength();

    double s_start = 0.0, s_end = 0.0;
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

    for (int k = 0; k < N; ++k) {
        ReferencePoint ref;
        
        if (params_.time_parameterize) {
            double s_local = s_geom_local;
            if (has_window_s) {
                double s_geom_global = s_progress + params_.lookahead_distance;
                s_geom_global = std::min(std::max(s_geom_global, s_start), s_end);
                const double window_len = std::max(1e-6, s_end - s_start);
                const double scale = total_len / window_len;
                s_local = (s_geom_global - s_start) * scale;
            }
            s_local = std::max(0.0, std::min(total_len, s_local));

            double v_ref = speed_profile_valid_ ? getSpeedAtS(s_progress) : v_des;
            ref.v_ref = v_ref;
            ref.v_path = v_ref;
            ref.vy_ref = 0.0;  // 全向轮参考横向速度默认为 0

            s_progress = std::min(s_progress + v_ref * dt, total_len_global);
            if (!has_window_s) {
                s_geom_local = std::min(s_geom_local + v_ref * dt, total_len);
            }

            Eigen::Vector2d pos = local_spline_.evaluate(s_local);
            ref.x = pos.x();
            ref.y = pos.y();
            ref.theta_path = local_spline_.evaluateTheta(s_local);
            ref.kappa = local_spline_.evaluateKappa(s_local);
            ref.s = s_local;
        } else {
            double s_local = std::min(current_s_ + params_.lookahead_distance + k * dt * v_des, total_len);

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
            ref.vy_ref = 0.0;
            ref.s = s_local;
        }
        
        ref_points.push_back(ref);
    }

    path_timestamp_ = ros::Time::now();
    return true;
}

bool PathHandler::getFrenetState(FrenetState& frenet) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || !has_robot_state_ || !local_spline_.isValid()) return false;
    
    double robot_theta = 0.0;
    Eigen::Vector2d robot_pos(0.0, 0.0);
    
    computeFrenetProjection(robot_pos, robot_theta, frenet.e_l, frenet.e_c, frenet.e_theta);
    return true;
}

bool PathHandler::isGoalReached() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_path_ || global_path_.poses.empty() || !tf_buffer_) return false;
    
    geometry_msgs::TransformStamped tf_map_to_base;
    try {
        tf_map_to_base = tf_buffer_->lookupTransform(
            base_frame_, global_path_.header.frame_id,
            ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error in isGoalReached: %s", ex.what());
        return false;
    }
    
    const auto& goal_pose = global_path_.poses.back().pose;
    tf2::Transform tf_transform;
    tf2::fromMsg(tf_map_to_base.transform, tf_transform);
    
    tf2::Vector3 goal_map(goal_pose.position.x, goal_pose.position.y, 0.0);
    tf2::Vector3 goal_base = tf_transform * goal_map;
    
    double dist = std::sqrt(goal_base.x() * goal_base.x() + goal_base.y() * goal_base.y());
    
    double goal_yaw_in_base = 0.0;
    const size_t n = global_path_.poses.size();
    if (n >= 2) {
        const auto& p1_pose = global_path_.poses[n - 2].pose;
        const auto& p2_pose = global_path_.poses[n - 1].pose;
        tf2::Vector3 p1_base = tf_transform * tf2::Vector3(p1_pose.position.x, p1_pose.position.y, 0.0);
        tf2::Vector3 p2_base = tf_transform * tf2::Vector3(p2_pose.position.x, p2_pose.position.y, 0.0);
        double tdx = p2_base.x() - p1_base.x();
        double tdy = p2_base.y() - p1_base.y();
        if (std::hypot(tdx, tdy) > 1e-6) {
            goal_yaw_in_base = std::atan2(tdy, tdx);
        }
    }

    double yaw_err = std::abs(goal_yaw_in_base);
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    yaw_err = std::abs(yaw_err);
    
    return (dist < params_.goal_tolerance && yaw_err < params_.yaw_tolerance);
}

bool PathHandler::isPathValid() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_path_) return false;
    return (ros::Time::now() - path_timestamp_).toSec() <= params_.path_timeout;
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
    if (!local_spline_.isValid()) return false;

    const double total_len = local_spline_.getTotalLength();
    if (total_len <= 1e-6 || num_samples < 2) return false;

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
    if (!tf_buffer_) return false;
    
    points_out.clear();
    points_out.reserve(path_in.poses.size());
    
    geometry_msgs::TransformStamped tf_map_to_base;
    try {
        tf_map_to_base = tf_buffer_->lookupTransform(
            base_frame_, path_in.header.frame_id, ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[PathHandler] TF error: %s", ex.what());
        return false;
    }
    
    for (const auto& pose : path_in.poses) {
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = path_in.header;
        pose_in.pose = pose.pose;
        tf2::doTransform(pose_in, pose_out, tf_map_to_base);
        points_out.emplace_back(pose_out.pose.position.x, pose_out.pose.position.y);
    }
    return true;
}

int PathHandler::findClosestPointIndex(const std::vector<Eigen::Vector2d>& points,
                                       const Eigen::Vector2d& robot_pos) const {
    if (points.empty()) return 0;
    
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    int search_start = std::max(0, closest_idx_ - 5);
    int search_end = std::min(static_cast<int>(points.size()), closest_idx_ + 20);
    
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
    
    double best_s = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    double total_len = local_spline_.getTotalLength();
    
    const int num_samples = 50;
    for (int i = 0; i <= num_samples; ++i) {
        double s = total_len * i / num_samples;
        double dist = (local_spline_.evaluate(s) - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_s = s;
        }
    }
    
    double delta = total_len / num_samples;
    double s_start = std::max(0.0, best_s - delta);
    double s_end = std::min(total_len, best_s + delta);
    
    for (int i = 0; i <= 20; ++i) {
        double s = s_start + (s_end - s_start) * i / 20;
        double dist = (local_spline_.evaluate(s) - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_s = s;
        }
    }
    
    current_s_ = best_s;
    
    Eigen::Vector2d closest_point = local_spline_.evaluate(best_s);
    double theta_path = local_spline_.evaluateTheta(best_s);
    
    Eigen::Vector2d error = point - closest_point;
    Eigen::Vector2d tangent(std::cos(theta_path), std::sin(theta_path));
    Eigen::Vector2d normal(-std::sin(theta_path), std::cos(theta_path));
    
    e_l = error.dot(tangent);
    e_c = error.dot(normal);
    
    e_theta = robot_theta - theta_path;
    while (e_theta > M_PI) e_theta -= 2 * M_PI;
    while (e_theta < -M_PI) e_theta += 2 * M_PI;
}

bool PathHandler::fitLocalSpline(const std::vector<Eigen::Vector2d>& points,
                                  int start_idx, int end_idx) {
    std::vector<Eigen::Vector2d> window_points;
    for (int i = start_idx; i <= end_idx; ++i) window_points.push_back(points[i]);
    return fitLocalSpline(window_points);
}

bool PathHandler::fitLocalSpline(const std::vector<Eigen::Vector2d>& window_points) {
    if (window_points.size() < 2) return false;

    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(window_points.size());
    const double min_dist = 1e-4;
    for (const auto& p : window_points) {
        if (filtered.empty() || (p - filtered.back()).norm() > min_dist) {
            filtered.push_back(p);
        }
    }
    if (filtered.size() < 2) return false;
    return local_spline_.fit(filtered);
}

double PathHandler::projectToPathS(const Eigen::Vector2d& point,
                                   int closest_idx,
                                   const std::vector<Eigen::Vector2d>& points,
                                   const std::vector<double>& path_s) const {
    const int n = static_cast<int>(points.size());
    if (n < 2 || closest_idx < 0 || closest_idx >= n ||
        static_cast<int>(path_s.size()) != n) {
        return (closest_idx >= 0 && closest_idx < n && !path_s.empty())
            ? path_s[static_cast<size_t>(closest_idx)] : 0.0;
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
        double t = std::max(0.0, std::min(1.0, (point - a).dot(ab) / ab2));
        dist2_out = (point - (a + t * ab)).squaredNorm();
        double seg_len = path_s[static_cast<size_t>(i1)] - path_s[static_cast<size_t>(i0)];
        if (seg_len < 1e-9) seg_len = std::sqrt(ab2);
        s_out = path_s[static_cast<size_t>(i0)] + t * seg_len;
    };

    if (closest_idx <= 0) {
        double s_best, d2;
        project_on_segment(0, 1, s_best, d2);
        return s_best;
    }
    if (closest_idx >= n - 1) {
        double s_best, d2;
        project_on_segment(n - 2, n - 1, s_best, d2);
        return s_best;
    }

    double s1, d21, s2, d22;
    project_on_segment(closest_idx - 1, closest_idx, s1, d21);
    project_on_segment(closest_idx, closest_idx + 1, s2, d22);
    return (d21 <= d22) ? s1 : s2;
}

void PathHandler::updateSpeedProfile(double v_des) {
    speed_profile_s_.clear();
    speed_profile_v_.clear();
    speed_profile_valid_ = false;
    speed_profile_v_des_ = v_des;

    if (!params_.time_parameterize || !global_spline_.isValid()) return;

    const double total_len = global_spline_length_ > 1e-6
        ? global_spline_length_ : global_spline_.getTotalLength();
    if (total_len <= 1e-6) return;

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

    const double goal_margin = std::max(0.0, params_.goal_tolerance);
    if (!speed_profile_v_.empty()) {
        speed_profile_v_.back() = std::min(speed_profile_v_.back(),
                                           std::max(0.0, params_.goal_speed));
        for (int i = static_cast<int>(speed_profile_v_.size()) - 1; i >= 0; --i) {
            if (total_len - speed_profile_s_[static_cast<size_t>(i)] < goal_margin) {
                speed_profile_v_[static_cast<size_t>(i)] = std::max(0.0, params_.goal_speed);
            } else {
                break;
            }
        }
    }

    if (params_.max_tan_accel > 0.0) {
        for (size_t i = 1; i < speed_profile_v_.size(); ++i) {
            double v_lim = std::sqrt(std::max(0.0, speed_profile_v_[i - 1] * speed_profile_v_[i - 1] + 2.0 * params_.max_tan_accel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    const double decel_safety_factor = 0.8;
    double max_decel = params_.max_tan_decel > 0.0 ? params_.max_tan_decel : params_.max_tan_accel;
    max_decel *= decel_safety_factor;
    if (max_decel > 0.0) {
        for (int i = static_cast<int>(speed_profile_v_.size()) - 2; i >= 0; --i) {
            double v_lim = std::sqrt(std::max(0.0, speed_profile_v_[i + 1] * speed_profile_v_[i + 1] + 2.0 * max_decel * ds));
            speed_profile_v_[i] = std::min(speed_profile_v_[i], v_lim);
        }
    }

    if (params_.min_ref_speed > 0.0 && speed_profile_v_.size() >= 2) {
        for (size_t i = 0; i + 1 < speed_profile_v_.size(); ++i) {
            speed_profile_v_[i] = std::max(speed_profile_v_[i], params_.min_ref_speed);
        }
    }

    speed_profile_valid_ = true;
}

double PathHandler::getSpeedAtS(double s) const {
    if (!speed_profile_valid_ || speed_profile_s_.empty()) return 0.0;
    if (s <= speed_profile_s_.front()) return speed_profile_v_.front();
    if (s >= speed_profile_s_.back()) return speed_profile_v_.back();

    auto it = std::upper_bound(speed_profile_s_.begin(), speed_profile_s_.end(), s);
    size_t idx = static_cast<size_t>(std::distance(speed_profile_s_.begin(), it));
    if (idx == 0) return speed_profile_v_.front();
    double s0 = speed_profile_s_[idx - 1];
    double s1 = speed_profile_s_[idx];
    double v0 = speed_profile_v_[idx - 1];
    double v1 = speed_profile_v_[idx];
    double t = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
    return v0 + t * (v1 - v0);
}

}  // namespace scout_omni_local_planner
