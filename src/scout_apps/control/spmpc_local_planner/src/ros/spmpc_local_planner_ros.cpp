#include "spmpc_local_planner/ros/spmpc_local_planner_ros.h"
#include "spmpc_local_planner/solvers/solver_factory.h"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

SpmpcLocalPlannerROS::SpmpcLocalPlannerROS()
    : tf_listener_(tf_buffer_) {}

bool SpmpcLocalPlannerROS::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    nh_ = nh;
    pnh_ = pnh;

    std::string variant_name = "B0";
    pnh_.param("planner_variant", variant_name, variant_name);
    pnh_.param("experiment_mode", experiment_mode_, experiment_mode_);
    pnh_.param("topics/odom", odom_topic_, odom_topic_);
    pnh_.param("topics/reference_path", path_topic_, path_topic_);
    pnh_.param("topics/costmap", costmap_topic_, costmap_topic_);
    pnh_.param("topics/cmd_vel", cmd_topic_, cmd_topic_);
    pnh_.param("frames/robot_base", robot_base_frame_, robot_base_frame_);
    pnh_.param("frames/reference_target", reference_target_frame_, reference_target_frame_);
    pnh_.param("frames/use_tf_pose", use_tf_pose_, use_tf_pose_);
    pnh_.param("frames/tf_timeout_sec", tf_timeout_sec_, tf_timeout_sec_);
    pnh_.param("publish_cmd_vel", publish_cmd_vel_, publish_cmd_vel_);
    pnh_.param("control_frequency", control_frequency_, control_frequency_);
    pnh_.param("dt", dt_, dt_);
    pnh_.param("horizon_steps", horizon_steps_, horizon_steps_);
    pnh_.param("reference/preprocess_enable", reference_preprocess_params_.enable, reference_preprocess_params_.enable);
    pnh_.param("reference/resample_spacing", reference_preprocess_params_.resample_spacing, reference_preprocess_params_.resample_spacing);
    pnh_.param("reference/smoothing_window", reference_preprocess_params_.smoothing_window, reference_preprocess_params_.smoothing_window);
    pnh_.param("reference/min_segment_length", reference_preprocess_params_.min_segment_length, reference_preprocess_params_.min_segment_length);

    SolverParams solver_params;
    pnh_.param("robot/v_max", solver_params.v_max, solver_params.v_max);
    pnh_.param("robot/omega_max", solver_params.omega_max, solver_params.omega_max);
    pnh_.param("robot/a_max", solver_params.a_max, solver_params.a_max);
    pnh_.param("robot/alpha_max", solver_params.alpha_max, solver_params.alpha_max);
    shared_cmd_linear_accel_max_ = solver_params.a_max;
    pnh_.param("platform/shared_constraints/linear_accel_limit_enable",
               shared_cmd_linear_accel_limit_enable_,
               shared_cmd_linear_accel_limit_enable_);
    pnh_.param("platform/shared_constraints/linear_accel_max",
               shared_cmd_linear_accel_max_,
               shared_cmd_linear_accel_max_);
    pnh_.param("platform/shared_constraints/linear_accel_max_dt",
               shared_cmd_linear_accel_max_dt_,
               shared_cmd_linear_accel_max_dt_);
    shared_cmd_linear_accel_max_ = std::max(0.0, shared_cmd_linear_accel_max_);
    shared_cmd_linear_accel_max_dt_ = std::max(1e-3, shared_cmd_linear_accel_max_dt_);
    pnh_.param("experiment/corridor_width", solver_params.corridor_width, solver_params.corridor_width);
    pnh_.param("experiment/corridor_enable", solver_params.corridor_enable, solver_params.corridor_enable);
    pnh_.param("experiment/corridor_hard_bound_enable",
               solver_params.corridor_hard_bound_enable,
               solver_params.corridor_hard_bound_enable);
    pnh_.param("experiment/corridor_weight", solver_params.corridor_weight, solver_params.corridor_weight);
    pnh_.param("experiment/obstacle_enable", solver_params.obstacle_enable, solver_params.obstacle_enable);
    pnh_.param("experiment/obstacle_weight", solver_params.obstacle_weight, solver_params.obstacle_weight);
    pnh_.param("experiment/obstacle_influence_radius",
               solver_params.obstacle_influence_radius,
               solver_params.obstacle_influence_radius);
    pnh_.param("experiment/homotopy_enable", solver_params.homotopy_enable, solver_params.homotopy_enable);
    pnh_.param("experiment/homotopy_lateral_offset",
               solver_params.homotopy_lateral_offset,
               solver_params.homotopy_lateral_offset);
    pnh_.param("reference/lookahead_distance", solver_params.lookahead_distance, solver_params.lookahead_distance);
    pnh_.param("terminal/enable", solver_params.terminal.enable, solver_params.terminal.enable);
    pnh_.param("terminal/goal_tolerance", solver_params.terminal.goal_tolerance, solver_params.terminal.goal_tolerance);
    pnh_.param("terminal/goal_reached_max_speed", solver_params.terminal.goal_reached_max_speed, solver_params.terminal.goal_reached_max_speed);
    pnh_.param("terminal/goal_reached_max_omega", solver_params.terminal.goal_reached_max_omega, solver_params.terminal.goal_reached_max_omega);
    pnh_.param("terminal/slowdown/enable", solver_params.terminal.slowdown_enable, solver_params.terminal.slowdown_enable);
    pnh_.param("terminal/slowdown/distance", solver_params.terminal.slowdown_distance, solver_params.terminal.slowdown_distance);
    pnh_.param("terminal/slowdown/v_max", solver_params.terminal.slowdown_v_max, solver_params.terminal.slowdown_v_max);
    pnh_.param("terminal/capture_stop/enable", solver_params.terminal.capture_stop_enable, solver_params.terminal.capture_stop_enable);
    pnh_.param("terminal/capture_stop/distance", solver_params.terminal.capture_stop_distance, solver_params.terminal.capture_stop_distance);
    pnh_.param("terminal/capture_stop/v_cap", solver_params.terminal.capture_v_cap, solver_params.terminal.capture_v_cap);
    pnh_.param("terminal/capture_stop/goal_behind_x", solver_params.terminal.goal_behind_x, solver_params.terminal.goal_behind_x);
    pnh_.param("terminal/command_clamp/enable", solver_params.terminal.command_clamp_enable, solver_params.terminal.command_clamp_enable);
    pnh_.param("terminal/command_clamp/rate_limit_enable", solver_params.terminal.rate_limit_enable, solver_params.terminal.rate_limit_enable);
    pnh_.param("platform/kinematics", solver_params.platform.kinematics, solver_params.platform.kinematics);
    pnh_.param("acados/warm_start_flatness_enable", solver_params.warm_start_flatness_enable, solver_params.warm_start_flatness_enable);
    pnh_.param("acados/warm_start/type", solver_params.warm_start.type, solver_params.warm_start.type);
    pnh_.param("acados/warm_start/use_previous_solution", solver_params.warm_start.use_previous_solution, solver_params.warm_start.use_previous_solution);
    pnh_.param("acados/warm_start/use_slosh_rollout", solver_params.warm_start.use_slosh_rollout, solver_params.warm_start.use_slosh_rollout);
    pnh_.param("acados/warm_start/curvature_speed_limit_enable",
               solver_params.warm_start.curvature_speed_limit_enable,
               solver_params.warm_start.curvature_speed_limit_enable);
    pnh_.param("acados/warm_start/max_reference_fit_error",
               solver_params.warm_start.max_reference_fit_error,
               solver_params.warm_start.max_reference_fit_error);
    pnh_.param("acados/warm_start/fallback_to_previous_solution",
               solver_params.warm_start.fallback_to_previous_solution,
               solver_params.warm_start.fallback_to_previous_solution);
    pnh_.param("acados/warm_start/fallback_to_primitive",
               solver_params.warm_start.fallback_to_primitive,
               solver_params.warm_start.fallback_to_primitive);
    if (pnh_.hasParam("acados/warm_start/enable")) {
        pnh_.param("acados/warm_start/enable", solver_params.warm_start.enable, solver_params.warm_start.enable);
    } else {
        solver_params.warm_start.enable = solver_params.warm_start_flatness_enable;
    }
    pnh_.param("solver_backend", solver_params.solver_backend, solver_params.solver_backend);
    if (!isKnownSolverBackend(solver_params.solver_backend)) {
        ROS_WARN("[spmpc_local_planner] unknown solver_backend '%s'; falling back to primitive",
                 solver_params.solver_backend.c_str());
        solver_params.solver_backend = kSolverBackendPrimitive;
    }
    solver_params.slosh = loadSloshParams();
    solver_params.slosh.dt = dt_;

    variant_ = makeVariantConfig(variant_name);
    if (variant_name != "B0" && variant_.name == "B0") {
        ROS_WARN("[spmpc_local_planner] unknown planner_variant '%s'; falling back to B0", variant_name.c_str());
    }
    loadVariantOverrides(variant_.name);
    if (!pnh_.hasParam("variants/" + variant_.name + "/w_contour")) {
        ROS_WARN("[spmpc_local_planner] 未找到 variants/%s/* 参数：config/planner/variants.yaml "
                 "可能未加载，变体权重回退到内置 B0 默认值。正式实验请用 launch 加载 variants.yaml",
                 variant_.name.c_str());
    }
    problem_.configure(solver_params, variant_);
    if (!slosh_observer_.configure(solver_params.slosh)) {
        ROS_WARN("[spmpc_local_planner] slosh observer configure failed; slosh diagnostics stay zero");
    }
    obstacle_enable_ = solver_params.obstacle_enable;

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &SpmpcLocalPlannerROS::odomCallback, this);
    path_sub_ = nh_.subscribe(path_topic_, 1, &SpmpcLocalPlannerROS::pathCallback, this);
    if (obstacle_enable_) {
        costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &SpmpcLocalPlannerROS::costmapCallback, this);
    }
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

    ros::NodeHandle spmpc_nh(nh_, "spmpc");
    diagnostics_.initialize(spmpc_nh);
    diagnostics_.publishVariant(variant_, experiment_mode_);
    diagnostics_.publishSolverBackend(solver_params.solver_backend);
    diagnostics_.publishStatus("INITIALIZED");

    const double period = 1.0 / std::max(1.0, control_frequency_);
    control_timer_ = nh_.createTimer(ros::Duration(period), &SpmpcLocalPlannerROS::controlTimerCallback, this);

    ROS_INFO("[spmpc_local_planner] initialized variant=%s mode=%s path_topic=%s costmap_topic=%s cmd_topic=%s",
             variant_.name.c_str(), experiment_mode_.c_str(), path_topic_.c_str(), costmap_topic_.c_str(), cmd_topic_.c_str());
    return true;
}

void SpmpcLocalPlannerROS::spin() {
    ros::spin();
}

void SpmpcLocalPlannerROS::publishZeroCommand() {
    geometry_msgs::Twist cmd;
    publishCommand(cmd);
}

geometry_msgs::Twist SpmpcLocalPlannerROS::applySharedCommandLimits(
    const geometry_msgs::Twist& desired,
    const ros::Time& stamp) {
    geometry_msgs::Twist limited = desired;
    if (!shared_cmd_linear_accel_limit_enable_ || shared_cmd_linear_accel_max_ <= 0.0) {
        last_published_cmd_ = desired;
        last_cmd_stamp_ = stamp;
        have_last_published_cmd_ = true;
        return desired;
    }

    const double nominal_dt = 1.0 / std::max(1.0, control_frequency_);
    double dt = nominal_dt;
    if (have_last_published_cmd_ && !last_cmd_stamp_.isZero() && !stamp.isZero()) {
        dt = (stamp - last_cmd_stamp_).toSec();
    }
    if (!std::isfinite(dt) || dt <= 1e-6) {
        dt = nominal_dt;
    }
    dt = std::min(dt, shared_cmd_linear_accel_max_dt_);

    const double max_step = shared_cmd_linear_accel_max_ * dt;
    const double dv = desired.linear.x - last_published_cmd_.linear.x;
    limited.linear.x = last_published_cmd_.linear.x + std::max(-max_step, std::min(max_step, dv));

    last_published_cmd_ = limited;
    last_cmd_stamp_ = stamp;
    have_last_published_cmd_ = true;
    return limited;
}

void SpmpcLocalPlannerROS::publishCommand(const geometry_msgs::Twist& desired) {
    if (!publish_cmd_vel_) {
        return;
    }
    const auto stamp = ros::Time::now();
    const auto cmd = applySharedCommandLimits(desired, stamp);
    cmd_pub_.publish(cmd);
}

void SpmpcLocalPlannerROS::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    updateSloshObserverFromOdom(*msg);
    last_odom_ = *msg;
    have_odom_ = true;
}

void SpmpcLocalPlannerROS::pathCallback(const nav_msgs::PathConstPtr& msg) {
    if (!reference_target_frame_.empty() && msg->header.frame_id != reference_target_frame_) {
        nav_msgs::Path transformed_path;
        transformed_path.header = msg->header;
        transformed_path.header.frame_id = reference_target_frame_;
        transformed_path.header.stamp = ros::Time(0);
        transformed_path.poses.reserve(msg->poses.size());
        try {
            for (const auto& pose : msg->poses) {
                geometry_msgs::PoseStamped stamped = pose;
                if (stamped.header.frame_id.empty()) {
                    stamped.header.frame_id = msg->header.frame_id;
                }
                stamped.header.stamp = ros::Time(0);
                auto transformed = tf_buffer_.transform(stamped, reference_target_frame_, ros::Duration(std::max(0.0, tf_timeout_sec_)));
                transformed.header.stamp = ros::Time(0);
                transformed_path.poses.push_back(transformed);
            }
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0,
                              "[spmpc_local_planner] transform reference path %s -> %s failed: %s",
                              msg->header.frame_id.c_str(),
                              reference_target_frame_.c_str(),
                              ex.what());
            return;
        }
        problem_.setReferencePath(referencePathFromMsg(transformed_path));
        return;
    }

    const auto reference = referencePathFromMsg(*msg);
    problem_.setReferencePath(reference);
}

void SpmpcLocalPlannerROS::costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    problem_.setCostmap(costmapFromMsg(*msg));
}

void SpmpcLocalPlannerROS::controlTimerCallback(const ros::TimerEvent&) {
    diagnostics_.publishVariant(variant_, experiment_mode_);

    if (!have_odom_) {
        diagnostics_.publishStatus("WAITING_FOR_ODOM");
        publishZeroCommand();
        return;
    }
    if (!problem_.hasReferencePath()) {
        diagnostics_.publishStatus("WAITING_FOR_REFERENCE_PATH");
        publishZeroCommand();
        return;
    }

    SolverInput input;
    if (!robotStateFromLatest(input.robot)) {
        diagnostics_.publishStatus("WAITING_FOR_TF_POSE");
        publishZeroCommand();
        return;
    }
    input.slosh = current_slosh_;
    input.dt = dt_;
    input.horizon_steps = horizon_steps_;

    SolverOutput output;
    problem_.solve(input, output);
    diagnostics_.publishStatus(output.status);
    diagnostics_.publishSloshState(input.slosh);
    // 当前标量模型液面高度 = c_h·‖η‖ (+向心项), 由唯一物理核 SloshDynamics 计算; 单位米(模型 proxy)。
    if (slosh_observer_.configured()) {
        const double omega_meas = last_odom_.twist.twist.angular.z;
        diagnostics_.publishSloshHeight(slosh_observer_.height(input.slosh, omega_meas));
    }
    diagnostics_.publishOutput(output, problem_.referenceFrameId());

    geometry_msgs::Twist cmd;
    cmd.linear.x = output.success ? output.cmd_v : 0.0;
    cmd.angular.z = output.success ? output.cmd_omega : 0.0;
    publishCommand(cmd);
}

RobotState SpmpcLocalPlannerROS::robotStateFromOdom(const nav_msgs::Odometry& odom) const {
    RobotState state;
    state.x = odom.pose.pose.position.x;
    state.y = odom.pose.pose.position.y;
    state.yaw = tf2::getYaw(odom.pose.pose.orientation);
    state.v = odom.twist.twist.linear.x;
    state.omega = odom.twist.twist.angular.z;
    return state;
}

bool SpmpcLocalPlannerROS::robotStateFromLatest(RobotState& state) {
    state = robotStateFromOdom(last_odom_);
    if (!use_tf_pose_) {
        return true;
    }

    const std::string reference_frame = problem_.referenceFrameId();
    if (reference_frame.empty()) {
        return true;
    }

    try {
        const auto tf = tf_buffer_.lookupTransform(
            reference_frame,
            robot_base_frame_,
            ros::Time(0),
            ros::Duration(std::max(0.0, tf_timeout_sec_)));
        state.x = tf.transform.translation.x;
        state.y = tf.transform.translation.y;
        state.yaw = tf2::getYaw(tf.transform.rotation);
        return true;
    } catch (const tf2::TransformException& ex) {
        if (reference_frame != last_odom_.header.frame_id) {
            ROS_WARN_THROTTLE(1.0,
                              "[spmpc_local_planner] TF pose unavailable %s <- %s: %s; odom frame is %s, refuse mixed-frame fallback",
                              reference_frame.c_str(),
                              robot_base_frame_.c_str(),
                              ex.what(),
                              last_odom_.header.frame_id.c_str());
            return false;
        }
        ROS_WARN_THROTTLE(1.0,
                          "[spmpc_local_planner] TF pose unavailable %s <- %s: %s; odom frame matches reference, using odom fallback",
                          reference_frame.c_str(),
                          robot_base_frame_.c_str(),
                          ex.what());
        return true;
    }
}

void SpmpcLocalPlannerROS::updateSloshObserverFromOdom(const nav_msgs::Odometry& odom) {
    if (!slosh_observer_.configured()) {
        return;
    }
    if (!have_prev_odom_) {
        prev_odom_ = odom;
        have_prev_odom_ = true;
        return;
    }

    const double dt_msg = (odom.header.stamp - prev_odom_.header.stamp).toSec();
    const double dt_safe = dt_msg > 1e-4 ? dt_msg : dt_;
    const double v = odom.twist.twist.linear.x;
    const double prev_v = prev_odom_.twist.twist.linear.x;
    const double omega = odom.twist.twist.angular.z;
    const double ax = (v - prev_v) / std::max(1e-3, dt_safe);
    const double ay = v * omega;

    if (std::abs(dt_safe - slosh_observer_.params().dt) > 1e-4) {
        auto params = slosh_observer_.params();
        params.dt = dt_safe;
        if (!slosh_observer_.configure(params)) {
            ROS_WARN_THROTTLE(1.0, "[spmpc_local_planner] slosh observer reconfigure failed");
        }
    }
    current_slosh_ = slosh_observer_.step(current_slosh_, ax, ay, omega);
    prev_odom_ = odom;
}

ReferencePath SpmpcLocalPlannerROS::referencePathFromMsg(const nav_msgs::Path& path) const {
    std::vector<TrajectoryPoint> points;
    points.reserve(path.poses.size());
    for (const auto& pose_stamped : path.poses) {
        TrajectoryPoint p;
        p.x = pose_stamped.pose.position.x;
        p.y = pose_stamped.pose.position.y;
        p.yaw = tf2::getYaw(pose_stamped.pose.orientation);
        points.push_back(p);
    }

    const auto processed = reference_preprocessor_.preprocess(points, reference_preprocess_params_);
    ReferencePath reference;
    reference.setPoints(processed, path.header.frame_id);
    return reference;
}

CostmapGrid SpmpcLocalPlannerROS::costmapFromMsg(const nav_msgs::OccupancyGrid& map) const {
    CostmapGrid costmap;
    costmap.setGrid(
        map.info.width,
        map.info.height,
        map.info.resolution,
        map.info.origin.position.x,
        map.info.origin.position.y,
        tf2::getYaw(map.info.origin.orientation),
        map.data);
    return costmap;
}

void SpmpcLocalPlannerROS::loadVariantOverrides(const std::string& variant_name) {
    const std::string prefix = "variants/" + variant_name + "/";
    pnh_.param(prefix + "slosh_enable", variant_.slosh_enable, variant_.slosh_enable);
    pnh_.param(prefix + "smooth_priority_enable", variant_.smooth_priority_enable, variant_.smooth_priority_enable);
    pnh_.param(prefix + "slosh_constraint_enable", variant_.slosh_constraint_enable, variant_.slosh_constraint_enable);
    pnh_.param(prefix + "primitive_mode", variant_.primitive_mode, variant_.primitive_mode);
    pnh_.param(prefix + "w_contour", variant_.w_contour, variant_.w_contour);
    pnh_.param(prefix + "w_lag", variant_.w_lag, variant_.w_lag);
    pnh_.param(prefix + "w_progress", variant_.w_progress, variant_.w_progress);
    pnh_.param(prefix + "w_v", variant_.w_v, variant_.w_v);
    pnh_.param(prefix + "w_vs", variant_.w_vs, variant_.w_vs);
    pnh_.param(prefix + "v_ref", variant_.v_ref, variant_.v_ref);
    pnh_.param(prefix + "w_control", variant_.w_control, variant_.w_control);
    pnh_.param(prefix + "w_accel", variant_.w_accel, variant_.w_accel);
    pnh_.param(prefix + "w_smooth", variant_.w_smooth, variant_.w_smooth);
    pnh_.param(prefix + "w_slosh", variant_.w_slosh, variant_.w_slosh);
}

SloshModelParams SpmpcLocalPlannerROS::loadSloshParams() const {
    SloshModelParams params;
    pnh_.param("slosh/container_radius", params.container_radius, params.container_radius);
    pnh_.param("slosh/liquid_height", params.liquid_height, params.liquid_height);
    pnh_.param("slosh/liquid_density", params.liquid_density, params.liquid_density);
    pnh_.param("slosh/damping_ratio", params.damping_ratio, params.damping_ratio);
    pnh_.param("slosh/mode_index", params.mode_index, params.mode_index);
    pnh_.param("slosh/slosh_height_ref", params.slosh_height_ref, params.slosh_height_ref);
    pnh_.param("slosh/slosh_eta_dot_ratio", params.slosh_eta_dot_ratio, params.slosh_eta_dot_ratio);
    pnh_.param("slosh/use_linear_model", params.use_linear_model, params.use_linear_model);
    pnh_.param("slosh/use_parabola_term", params.use_parabola_term, params.use_parabola_term);
    return params;
}

}  // namespace spmpc_local_planner
