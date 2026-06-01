#include "spmpc_local_planner/ros/spmpc_local_planner_ros.h"
#include <algorithm>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

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
    pnh_.param("topics/cmd_vel", cmd_topic_, cmd_topic_);
    pnh_.param("frames/robot_base", robot_base_frame_, robot_base_frame_);
    pnh_.param("frames/use_tf_pose", use_tf_pose_, use_tf_pose_);
    pnh_.param("frames/tf_timeout_sec", tf_timeout_sec_, tf_timeout_sec_);
    pnh_.param("publish_cmd_vel", publish_cmd_vel_, publish_cmd_vel_);
    pnh_.param("control_frequency", control_frequency_, control_frequency_);
    pnh_.param("dt", dt_, dt_);
    pnh_.param("horizon_steps", horizon_steps_, horizon_steps_);

    SolverParams solver_params;
    pnh_.param("robot/v_max", solver_params.v_max, solver_params.v_max);
    pnh_.param("robot/omega_max", solver_params.omega_max, solver_params.omega_max);
    pnh_.param("robot/a_max", solver_params.a_max, solver_params.a_max);
    pnh_.param("experiment/corridor_width", solver_params.corridor_width, solver_params.corridor_width);
    pnh_.param("reference/lookahead_distance", solver_params.lookahead_distance, solver_params.lookahead_distance);
    pnh_.param("terminal/goal_tolerance", solver_params.goal_tolerance, solver_params.goal_tolerance);
    solver_params.slosh = loadSloshParams();
    solver_params.slosh.dt = dt_;

    variant_ = makeVariantConfig(variant_name);
    loadVariantOverrides(variant_.name);
    problem_.configure(solver_params, variant_);
    if (!slosh_observer_.configure(solver_params.slosh)) {
        ROS_WARN("[spmpc_local_planner] slosh observer configure failed; slosh diagnostics stay zero");
    }

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &SpmpcLocalPlannerROS::odomCallback, this);
    path_sub_ = nh_.subscribe(path_topic_, 1, &SpmpcLocalPlannerROS::pathCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

    ros::NodeHandle spmpc_nh(nh_, "spmpc");
    diagnostics_.initialize(spmpc_nh);
    diagnostics_.publishVariant(variant_, experiment_mode_);
    diagnostics_.publishStatus("INITIALIZED");

    const double period = 1.0 / std::max(1.0, control_frequency_);
    control_timer_ = nh_.createTimer(ros::Duration(period), &SpmpcLocalPlannerROS::controlTimerCallback, this);

    ROS_INFO("[spmpc_local_planner] initialized variant=%s mode=%s path_topic=%s cmd_topic=%s",
             variant_.name.c_str(), experiment_mode_.c_str(), path_topic_.c_str(), cmd_topic_.c_str());
    return true;
}

void SpmpcLocalPlannerROS::spin() {
    ros::spin();
}

void SpmpcLocalPlannerROS::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    updateSloshObserverFromOdom(*msg);
    last_odom_ = *msg;
    have_odom_ = true;
}

void SpmpcLocalPlannerROS::pathCallback(const nav_msgs::PathConstPtr& msg) {
    const auto reference = referencePathFromMsg(*msg);
    problem_.setReferencePath(reference);
}

void SpmpcLocalPlannerROS::controlTimerCallback(const ros::TimerEvent&) {
    diagnostics_.publishVariant(variant_, experiment_mode_);

    if (!have_odom_) {
        diagnostics_.publishStatus("WAITING_FOR_ODOM");
        return;
    }
    if (!problem_.hasReferencePath()) {
        diagnostics_.publishStatus("WAITING_FOR_REFERENCE_PATH");
        return;
    }

    SolverInput input;
    if (!robotStateFromLatest(input.robot)) {
        diagnostics_.publishStatus("WAITING_FOR_TF_POSE");
        return;
    }
    input.slosh = current_slosh_;
    input.dt = dt_;
    input.horizon_steps = horizon_steps_;

    SolverOutput output;
    problem_.solve(input, output);
    diagnostics_.publishStatus(output.status);
    diagnostics_.publishSloshState(input.slosh);
    diagnostics_.publishOutput(output, problem_.referenceFrameId());

    if (publish_cmd_vel_) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = output.success ? output.cmd_v : 0.0;
        cmd.angular.z = output.success ? output.cmd_omega : 0.0;
        cmd_pub_.publish(cmd);
    }
}

RobotState SpmpcLocalPlannerROS::robotStateFromOdom(const nav_msgs::Odometry& odom) const {
    RobotState state;
    state.x = odom.pose.pose.position.x;
    state.y = odom.pose.pose.position.y;
    state.yaw = tf2::getYaw(odom.pose.pose.orientation);
    state.v = odom.twist.twist.linear.x;
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
        ROS_WARN_THROTTLE(1.0,
                          "[spmpc_local_planner] TF pose unavailable %s <- %s: %s; using odom pose fallback",
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

    ReferencePath reference;
    reference.setPoints(points, path.header.frame_id);
    return reference;
}

void SpmpcLocalPlannerROS::loadVariantOverrides(const std::string& variant_name) {
    const std::string prefix = "variants/" + variant_name + "/";
    pnh_.param(prefix + "slosh_enable", variant_.slosh_enable, variant_.slosh_enable);
    pnh_.param(prefix + "smooth_priority_enable", variant_.smooth_priority_enable, variant_.smooth_priority_enable);
    pnh_.param(prefix + "slosh_constraint_enable", variant_.slosh_constraint_enable, variant_.slosh_constraint_enable);
    pnh_.param(prefix + "w_contour", variant_.w_contour, variant_.w_contour);
    pnh_.param(prefix + "w_lag", variant_.w_lag, variant_.w_lag);
    pnh_.param(prefix + "w_progress", variant_.w_progress, variant_.w_progress);
    pnh_.param(prefix + "w_control", variant_.w_control, variant_.w_control);
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
