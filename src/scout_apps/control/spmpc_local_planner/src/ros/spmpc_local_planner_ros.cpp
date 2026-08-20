#include "spmpc_local_planner/ros/spmpc_local_planner_ros.h"
#include "spmpc_local_planner/ros/ros_config_loader.h"
#include "spmpc_local_planner/solver/api/backend_policy.h"
#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "spmpc_local_planner/solvers/solver_factory.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

namespace {

constexpr double kMinimumOdomObserverDtSec = 1e-4;
constexpr double kOdomClockResetThresholdSec = 0.5;

const char* boolText(bool value) {
    return value ? "true" : "false";
}

ros::Time rosTimeFromNanoseconds(std::int64_t stamp_ns) {
    ros::Time stamp;
    if (stamp_ns > 0) {
        stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));
    }
    return stamp;
}

double ageSeconds(std::int64_t receive_stamp_ns, std::int64_t value_stamp_ns) {
    if (receive_stamp_ns <= 0 || value_stamp_ns <= 0) {
        return -1.0;
    }
    return static_cast<double>(receive_stamp_ns - value_stamp_ns) * 1.0e-9;
}

geometry_msgs::Twist velocityCommandToRos(const VelocityCommand& command) {
    geometry_msgs::Twist message;
    message.linear.x = command.linear;
    message.angular.z = command.angular;
    return message;
}


int primitiveModeCode(const std::string& primitive_mode) {
    if (primitive_mode == "linear") {
        return 1;
    }
    if (primitive_mode == "anti_slosh") {
        return 2;
    }
    return 0;
}

std::string appendVRefStatus(const std::string& current, const std::string& suffix) {
    if (current.empty()) {
        return suffix;
    }
    return current + "+" + suffix;
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}


}  // namespace

SpmpcLocalPlannerROS::SpmpcLocalPlannerROS()
    : tf_listener_(tf_buffer_),
      control_cycle_engine_(problem_) {}

SpmpcLocalPlannerROS::~SpmpcLocalPlannerROS() {
    if (imu_spinner_) {
        imu_spinner_->stop();
    }
}

bool SpmpcLocalPlannerROS::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    nh_ = nh;
    pnh_ = pnh;

    ValidationReport app_config_report;
    app_config_ = RosConfigLoader::load(pnh_, app_config_report);
    for (const auto& issue : app_config_report.issues()) {
        if (issue.severity == ValidationSeverity::Fatal) {
            ROS_ERROR("[spmpc_local_planner] invalid config %s: %s",
                      issue.key.c_str(), issue.message.c_str());
        } else {
            ROS_WARN("[spmpc_local_planner] normalized config %s: %s",
                     issue.key.c_str(), issue.message.c_str());
        }
    }
    if (!app_config_report.ok()) {
        return false;
    }
    if (app_config_.map_vref.profile_enable) {
        ensureMapVRefProfileLoaded(app_config_.map_vref.profile_path);
    }

    std::string variant_name = "B0";
    pnh_.param("planner_variant", variant_name, variant_name);
    pnh_.param("experiment_mode", experiment_mode_, experiment_mode_);
    pnh_.param("topics/odom", odom_topic_, odom_topic_);
    pnh_.param("topics/imu", imu_topic_, imu_topic_);
    pnh_.param("topics/reference_path", path_topic_, path_topic_);
    pnh_.param("topics/costmap", costmap_topic_, costmap_topic_);
    pnh_.param("topics/cmd_vel", cmd_topic_, cmd_topic_);
    pnh_.param("frames/robot_base", robot_base_frame_, robot_base_frame_);
    pnh_.param("frames/reference_target", reference_target_frame_, reference_target_frame_);
    pnh_.param("frames/use_tf_pose", use_tf_pose_, use_tf_pose_);
    pnh_.param("frames/tf_timeout_sec", tf_timeout_sec_, tf_timeout_sec_);
    pnh_.param("publish_cmd_vel", publish_cmd_vel_, publish_cmd_vel_);
    pnh_.param("imu_shadow/enable", imu_shadow_enable_, imu_shadow_enable_);
    pnh_.param("imu_shadow/publish_diagnostics",
               imu_shadow_publish_diagnostics_,
               imu_shadow_publish_diagnostics_);
    pnh_.param("imu_shadow/expected_frame", imu_expected_frame_, imu_expected_frame_);
    pnh_.param("imu_shadow/subscriber_queue_size",
               imu_subscriber_queue_size_,
               imu_subscriber_queue_size_);
    if (imu_subscriber_queue_size_ < 1 || imu_subscriber_queue_size_ > 1000) {
        ROS_WARN("[spmpc_local_planner] invalid imu_shadow/subscriber_queue_size=%d; using 10",
                 imu_subscriber_queue_size_);
        imu_subscriber_queue_size_ = 10;
    }
    pnh_.param("imu_shadow/observer_dt_sec", imu_observer_dt_sec_, imu_observer_dt_sec_);
    if (!std::isfinite(imu_observer_dt_sec_) || imu_observer_dt_sec_ <= 0.0) {
        ROS_WARN("[spmpc_local_planner] invalid imu_shadow/observer_dt_sec=%.6f; using 0.02 s",
                 imu_observer_dt_sec_);
        imu_observer_dt_sec_ = 0.02;
    }
    std::string observer_source = "odom";
    std::string observer_fallback_policy = "odom";
    pnh_.param("slosh_observer/source", observer_source, observer_source);
    pnh_.param("slosh_observer/fallback_policy",
               observer_fallback_policy,
               observer_fallback_policy);
    pnh_.param("slosh_observer/latch_fallback",
               slosh_observer_selector_params_.latch_fallback,
               slosh_observer_selector_params_.latch_fallback);
    pnh_.param("slosh_observer/max_imu_state_age_sec",
               slosh_observer_selector_params_.max_imu_state_age_sec,
               slosh_observer_selector_params_.max_imu_state_age_sec);
    pnh_.param("slosh_observer/max_odom_state_age_sec",
               slosh_observer_selector_params_.max_odom_state_age_sec,
               slosh_observer_selector_params_.max_odom_state_age_sec);
    pnh_.param("slosh_observer/max_future_skew_sec",
               slosh_observer_selector_params_.max_future_skew_sec,
               slosh_observer_selector_params_.max_future_skew_sec);
    if (!parseSloshObserverSource(
            observer_source, slosh_observer_selector_params_.nominal_source)) {
        ROS_FATAL("[spmpc_local_planner] invalid slosh_observer/source='%s'; "
                  "expected odom|processed_imu",
                  observer_source.c_str());
        return false;
    }
    if (!parseSloshObserverFallbackPolicy(
            observer_fallback_policy,
            slosh_observer_selector_params_.fallback_policy)) {
        ROS_FATAL("[spmpc_local_planner] invalid slosh_observer/fallback_policy='%s'; "
                  "expected odom|fail_closed",
                  observer_fallback_policy.c_str());
        return false;
    }
    const bool imu_is_nominal =
        slosh_observer_selector_params_.nominal_source ==
        SloshObserverSource::ProcessedImu;
    if (imu_is_nominal && !imu_shadow_enable_) {
        ROS_WARN("[spmpc_local_planner] processed_imu is the nominal liquid observer; "
                 "forcing imu_shadow/enable=true for the processed-IMU pipeline");
        imu_shadow_enable_ = true;
    }
    pnh_.param("control_frequency", control_frequency_, control_frequency_);
    pnh_.param("dt", dt_, dt_);
    pnh_.param("horizon_steps", horizon_steps_, horizon_steps_);
    std::string delay_phase_mode = delayPhaseModeName(delay_phase_params_.mode);
    pnh_.param("delay_phase/mode", delay_phase_mode, delay_phase_mode);
    delay_phase_params_.mode = parseDelayPhaseMode(delay_phase_mode);
    if (!isKnownDelayPhaseMode(delay_phase_mode)) {
        ROS_WARN("[spmpc_local_planner] 未知 delay_phase/mode=\"%s\"，已静默退化为 Off。"
                 "合法值：off / monitor / shadow / fixed_closed_loop / fixed_robot_only（及其别名）。",
                 delay_phase_mode.c_str());
    }
    pnh_.param("delay_phase/publish_diagnostics", delay_phase_params_.publish_diagnostics, delay_phase_params_.publish_diagnostics);
    pnh_.param("delay_phase/history_window_sec", delay_phase_params_.history_window_sec, delay_phase_params_.history_window_sec);
    pnh_.param("delay_phase/cmd_timeout_sec", delay_phase_params_.cmd_timeout_sec, delay_phase_params_.cmd_timeout_sec);
    pnh_.param("delay_phase/odom_timeout_sec", delay_phase_params_.odom_timeout_sec, delay_phase_params_.odom_timeout_sec);
    pnh_.param("delay_phase/linear_delay_sec", delay_phase_params_.linear_delay_sec, delay_phase_params_.linear_delay_sec);
    pnh_.param("delay_phase/angular_delay_sec", delay_phase_params_.angular_delay_sec, delay_phase_params_.angular_delay_sec);
    pnh_.param("delay_phase/linear_time_constant_sec",
               delay_phase_params_.linear_time_constant_sec,
               delay_phase_params_.linear_time_constant_sec);
    pnh_.param("delay_phase/angular_time_constant_sec",
               delay_phase_params_.angular_time_constant_sec,
               delay_phase_params_.angular_time_constant_sec);
    pnh_.param("delay_phase/max_prediction_sec", delay_phase_params_.max_prediction_sec, delay_phase_params_.max_prediction_sec);
    pnh_.param("delay_phase/max_integration_step_sec", delay_phase_params_.max_integration_step_sec, delay_phase_params_.max_integration_step_sec);
    pnh_.param("delay_phase/min_integration_step_sec", delay_phase_params_.min_integration_step_sec, delay_phase_params_.min_integration_step_sec);
    pnh_.param("delay_phase/require_complete_history", delay_phase_params_.require_complete_history, delay_phase_params_.require_complete_history);
    delay_phase_params_.history_window_sec = std::max(0.1, delay_phase_params_.history_window_sec);
    delay_phase_params_.cmd_timeout_sec = std::max(0.0, delay_phase_params_.cmd_timeout_sec);
    delay_phase_params_.odom_timeout_sec = std::max(0.0, delay_phase_params_.odom_timeout_sec);
    delay_phase_params_.linear_delay_sec = std::max(0.0, delay_phase_params_.linear_delay_sec);
    delay_phase_params_.angular_delay_sec = std::max(0.0, delay_phase_params_.angular_delay_sec);
    delay_phase_params_.linear_time_constant_sec = std::max(
        0.0, delay_phase_params_.linear_time_constant_sec);
    delay_phase_params_.angular_time_constant_sec = std::max(
        0.0, delay_phase_params_.angular_time_constant_sec);
    delay_phase_params_.max_prediction_sec = std::max(0.0, delay_phase_params_.max_prediction_sec);
    delay_phase_params_.max_integration_step_sec = std::max(1e-4, delay_phase_params_.max_integration_step_sec);
    delay_phase_params_.min_integration_step_sec = std::max(1e-6, delay_phase_params_.min_integration_step_sec);
    if (delay_phase_params_.min_integration_step_sec > delay_phase_params_.max_integration_step_sec) {
        delay_phase_params_.min_integration_step_sec = delay_phase_params_.max_integration_step_sec;
    }
    command_history_.configure(delay_phase_params_.history_window_sec);

    std::string phase_rejoin_mode = phaseRejoinModeName(
        phase_rejoin_params_.mode);
    pnh_.param("phase_rejoin/mode", phase_rejoin_mode, phase_rejoin_mode);
    if (!parsePhaseRejoinMode(phase_rejoin_mode, phase_rejoin_params_.mode)) {
        ROS_FATAL("[spmpc_local_planner] invalid phase_rejoin/mode='%s'; "
                  "expected off|monitor|enforce",
                  phase_rejoin_mode.c_str());
        return false;
    }
    pnh_.param("phase_rejoin/publish_diagnostics",
               phase_rejoin_publish_diagnostics_,
               phase_rejoin_publish_diagnostics_);
    pnh_.param("phase_rejoin/artifact_path",
               phase_rejoin_artifact_path_,
               phase_rejoin_artifact_path_);
    pnh_.param("phase_rejoin/liquid_horizon_steps",
               phase_rejoin_params_.liquid_horizon_steps,
               phase_rejoin_params_.liquid_horizon_steps);
    pnh_.param("phase_rejoin/max_residual_v",
               phase_rejoin_params_.max_residual_v,
               phase_rejoin_params_.max_residual_v);
    pnh_.param("phase_rejoin/max_residual_omega",
               phase_rejoin_params_.max_residual_omega,
               phase_rejoin_params_.max_residual_omega);
    pnh_.param("phase_rejoin/artifact_dt_tolerance_sec",
               phase_rejoin_params_.artifact_dt_tolerance_sec,
               phase_rejoin_params_.artifact_dt_tolerance_sec);
    pnh_.param("phase_rejoin/artifact_path_length_tolerance_m",
               phase_rejoin_params_.artifact_path_length_tolerance_m,
               phase_rejoin_params_.artifact_path_length_tolerance_m);
    pnh_.param("phase_rejoin/artifact_path_geometry_tolerance_m",
               phase_rejoin_params_.artifact_path_geometry_tolerance_m,
               phase_rejoin_params_.artifact_path_geometry_tolerance_m);
    pnh_.param("phase_rejoin/artifact_model_tolerance",
               phase_rejoin_params_.artifact_model_tolerance,
               phase_rejoin_params_.artifact_model_tolerance);
    pnh_.param("phase_rejoin/artifact_command_tolerance",
               phase_rejoin_params_.artifact_command_tolerance,
               phase_rejoin_params_.artifact_command_tolerance);
    pnh_.param("phase_rejoin/allow_development_artifact_in_enforce",
               phase_rejoin_params_.allow_development_artifact_in_enforce,
               phase_rejoin_params_.allow_development_artifact_in_enforce);
    pnh_.param("phase_rejoin/required_contract_id",
               phase_rejoin_params_.required_contract_id,
               phase_rejoin_params_.required_contract_id);
    pnh_.param("phase_rejoin/required_frame_id",
               phase_rejoin_params_.required_frame_id,
               phase_rejoin_params_.required_frame_id);
    pnh_.param("phase_rejoin/candidate/backward_radius",
               phase_rejoin_params_.candidate.backward_radius,
               phase_rejoin_params_.candidate.backward_radius);
    pnh_.param("phase_rejoin/candidate/forward_radius",
               phase_rejoin_params_.candidate.forward_radius,
               phase_rejoin_params_.candidate.forward_radius);
    pnh_.param("phase_rejoin/candidate/initial_forward_radius",
               phase_rejoin_params_.candidate.initial_forward_radius,
               phase_rejoin_params_.candidate.initial_forward_radius);
    pnh_.param("phase_rejoin/candidate/max_clock_lead_steps",
               phase_rejoin_params_.candidate.max_clock_lead_steps,
               phase_rejoin_params_.candidate.max_clock_lead_steps);
    pnh_.param("phase_rejoin/candidate/weight_position",
               phase_rejoin_params_.candidate.weight_position,
               phase_rejoin_params_.candidate.weight_position);
    pnh_.param("phase_rejoin/candidate/weight_yaw",
               phase_rejoin_params_.candidate.weight_yaw,
               phase_rejoin_params_.candidate.weight_yaw);
    pnh_.param("phase_rejoin/candidate/weight_velocity",
               phase_rejoin_params_.candidate.weight_velocity,
               phase_rejoin_params_.candidate.weight_velocity);
    pnh_.param("phase_rejoin/candidate/weight_liquid",
               phase_rejoin_params_.candidate.weight_liquid,
               phase_rejoin_params_.candidate.weight_liquid);
    std::string phase_rejoin_error;
    if (!control_cycle_engine_.configurePhaseRejoin(
            phase_rejoin_params_, phase_rejoin_error)) {
        ROS_FATAL("[spmpc_local_planner] phase-rejoin configuration failed: %s",
                  phase_rejoin_error.c_str());
        return false;
    }
    if (phase_rejoin_params_.mode != PhaseRejoinMode::Off) {
        if (phase_rejoin_artifact_path_.empty()) {
            if (phase_rejoin_params_.mode == PhaseRejoinMode::Enforce) {
                ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                          "a non-empty artifact_path");
                return false;
            }
            ROS_WARN("[spmpc_local_planner] phase_rejoin monitor has no "
                     "artifact; diagnostics will report not ready");
        } else {
            const NominalArtifactLoadResult load_result =
                control_cycle_engine_.loadPhaseRejoinArtifact(
                    phase_rejoin_artifact_path_);
            if (!load_result.success) {
                if (phase_rejoin_params_.mode == PhaseRejoinMode::Enforce) {
                    ROS_FATAL("[spmpc_local_planner] phase-rejoin artifact "
                              "load failed status=%s detail=%s",
                              load_result.status.c_str(),
                              load_result.detail.c_str());
                    return false;
                }
                ROS_ERROR("[spmpc_local_planner] phase-rejoin monitor "
                          "artifact load failed status=%s detail=%s",
                          load_result.status.c_str(),
                          load_result.detail.c_str());
            }
        }
    }
    pnh_.param("state_timing/require_common_epoch",
               state_timing_params_.require_common_epoch,
               state_timing_params_.require_common_epoch);
    pnh_.param("state_timing/max_raw_skew_sec",
               state_timing_params_.max_raw_skew_sec,
               state_timing_params_.max_raw_skew_sec);
    pnh_.param("state_timing/odom_history_sec",
               state_timing_params_.odom_history_sec,
               state_timing_params_.odom_history_sec);
    pnh_.param("state_timing/max_interpolation_gap_sec",
               state_timing_params_.max_interpolation_gap_sec,
               state_timing_params_.max_interpolation_gap_sec);
    pnh_.param("state_timing/max_robot_extrapolation_sec",
               state_timing_params_.max_robot_extrapolation_sec,
               state_timing_params_.max_robot_extrapolation_sec);
    pnh_.param("execution_contract/fail_closed_on_post_limit_change",
               command_contract_params_.fail_closed_on_post_limit_change,
               command_contract_params_.fail_closed_on_post_limit_change);
    pnh_.param("execution_contract/max_post_limit_delta_v",
               command_contract_params_.max_post_limit_delta_v,
               command_contract_params_.max_post_limit_delta_v);
    pnh_.param("execution_contract/max_post_limit_delta_omega",
               command_contract_params_.max_post_limit_delta_omega,
               command_contract_params_.max_post_limit_delta_omega);
    const bool valid_state_timing =
        std::isfinite(state_timing_params_.max_raw_skew_sec) &&
        state_timing_params_.max_raw_skew_sec >= 0.0 &&
        std::isfinite(state_timing_params_.odom_history_sec) &&
        state_timing_params_.odom_history_sec > 0.0 &&
        std::isfinite(state_timing_params_.max_interpolation_gap_sec) &&
        state_timing_params_.max_interpolation_gap_sec > 0.0 &&
        std::isfinite(state_timing_params_.max_robot_extrapolation_sec) &&
        state_timing_params_.max_robot_extrapolation_sec >= 0.0;
    const bool valid_command_contract =
        std::isfinite(command_contract_params_.max_post_limit_delta_v) &&
        command_contract_params_.max_post_limit_delta_v >= 0.0 &&
        std::isfinite(command_contract_params_.max_post_limit_delta_omega) &&
        command_contract_params_.max_post_limit_delta_omega >= 0.0;
    if (!valid_state_timing || !valid_command_contract) {
        ROS_FATAL("[spmpc_local_planner] invalid state_timing/execution_contract parameters");
        return false;
    }
    if (phase_rejoin_params_.mode == PhaseRejoinMode::Enforce &&
        !state_timing_params_.require_common_epoch) {
        ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                  "state_timing/require_common_epoch=true");
        return false;
    }
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
    shared_cmd_angular_rate_max_ = solver_params.omega_max;
    shared_cmd_angular_accel_max_ = solver_params.alpha_max;
    pnh_.param("platform/shared_constraints/linear_accel_limit_enable",
               shared_cmd_linear_accel_limit_enable_,
               shared_cmd_linear_accel_limit_enable_);
    pnh_.param("platform/shared_constraints/linear_accel_max",
               shared_cmd_linear_accel_max_,
               shared_cmd_linear_accel_max_);
    pnh_.param("platform/shared_constraints/linear_accel_max_dt",
               shared_cmd_linear_accel_max_dt_,
               shared_cmd_linear_accel_max_dt_);
    pnh_.param("platform/shared_constraints/angular_limit_enable",
               shared_cmd_angular_limit_enable_,
               shared_cmd_angular_limit_enable_);
    pnh_.param("platform/shared_constraints/angular_rate_max",
               shared_cmd_angular_rate_max_,
               shared_cmd_angular_rate_max_);
    pnh_.param("platform/shared_constraints/angular_accel_max",
               shared_cmd_angular_accel_max_,
               shared_cmd_angular_accel_max_);
    pnh_.param("platform/shared_constraints/angular_accel_max_dt",
               shared_cmd_angular_accel_max_dt_,
               shared_cmd_angular_accel_max_dt_);
    shared_cmd_linear_accel_max_ = std::max(0.0, shared_cmd_linear_accel_max_);
    shared_cmd_linear_accel_max_dt_ = std::max(1e-3, shared_cmd_linear_accel_max_dt_);
    if (shared_cmd_angular_rate_max_ <= 0.0) {
        shared_cmd_angular_rate_max_ = solver_params.omega_max;
    }
    if (shared_cmd_angular_accel_max_ <= 0.0) {
        shared_cmd_angular_accel_max_ = solver_params.alpha_max;
    }
    shared_cmd_angular_rate_max_ = std::max(0.0, shared_cmd_angular_rate_max_);
    shared_cmd_angular_accel_max_ = std::max(0.0, shared_cmd_angular_accel_max_);
    shared_cmd_angular_accel_max_dt_ = std::max(1e-3, shared_cmd_angular_accel_max_dt_);
    CommandPipelineConfig command_pipeline_config;
    command_pipeline_config.control_frequency = control_frequency_;
    command_pipeline_config.linear_accel_limit_enable =
        shared_cmd_linear_accel_limit_enable_;
    command_pipeline_config.linear_accel_max = shared_cmd_linear_accel_max_;
    command_pipeline_config.linear_accel_max_dt =
        shared_cmd_linear_accel_max_dt_;
    command_pipeline_config.angular_limit_enable =
        shared_cmd_angular_limit_enable_;
    command_pipeline_config.angular_rate_max = shared_cmd_angular_rate_max_;
    command_pipeline_config.angular_accel_max = shared_cmd_angular_accel_max_;
    command_pipeline_config.angular_accel_max_dt =
        shared_cmd_angular_accel_max_dt_;
    command_pipeline_config.fail_closed_on_post_limit_change =
        command_contract_params_.fail_closed_on_post_limit_change;
    command_pipeline_config.max_post_limit_delta_v =
        command_contract_params_.max_post_limit_delta_v;
    command_pipeline_config.max_post_limit_delta_omega =
        command_contract_params_.max_post_limit_delta_omega;
    std::string command_pipeline_error;
    if (!command_pipeline_.configure(
            command_pipeline_config, command_pipeline_error)) {
        ROS_FATAL("[spmpc_local_planner] command pipeline configuration failed: %s",
                  command_pipeline_error.c_str());
        return false;
    }
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
    pnh_.param("terminal/command_clamp/omega_enable", solver_params.terminal.omega_clamp_enable, solver_params.terminal.omega_clamp_enable);
    pnh_.param("terminal/command_clamp/omega_max", solver_params.terminal.omega_clamp_max, solver_params.terminal.omega_clamp_max);
    pnh_.param("terminal/command_clamp/omega_near_goal_max", solver_params.terminal.omega_near_goal_max, solver_params.terminal.omega_near_goal_max);
    pnh_.param("terminal/command_clamp/omega_near_goal_distance", solver_params.terminal.omega_near_goal_distance, solver_params.terminal.omega_near_goal_distance);
    SafetySupervisorConfig safety_config;
    safety_config.nominal_period_sec = dt_;
    pnh_.param("terminal/spin_fail/enable",
               safety_config.terminal_spin.enable,
               safety_config.terminal_spin.enable);
    pnh_.param("terminal/spin_fail/omega_threshold",
               safety_config.terminal_spin.omega_threshold,
               safety_config.terminal_spin.omega_threshold);
    pnh_.param("terminal/spin_fail/max_duration_sec",
               safety_config.terminal_spin.max_duration_sec,
               safety_config.terminal_spin.max_duration_sec);
    safety_config.terminal_spin.omega_threshold = std::max(
        0.0, safety_config.terminal_spin.omega_threshold);
    safety_config.terminal_spin.max_duration_sec = std::max(
        0.0, safety_config.terminal_spin.max_duration_sec);
    pnh_.param("tracking_safety/enable",
               safety_config.tracking.enable,
               safety_config.tracking.enable);
    pnh_.param("tracking_safety/projection/enable",
               safety_config.tracking.projection_enable,
               safety_config.tracking.projection_enable);
    pnh_.param("tracking_safety/projection/max_distance_m",
               safety_config.tracking.max_projection_distance_m,
               safety_config.tracking.max_projection_distance_m);
    pnh_.param("tracking_safety/projection/max_duration_sec",
               safety_config.tracking.max_projection_duration_sec,
               safety_config.tracking.max_projection_duration_sec);
    pnh_.param("tracking_safety/spin_fail/enable",
               safety_config.tracking.spin_enable,
               safety_config.tracking.spin_enable);
    pnh_.param("tracking_safety/spin_fail/omega_threshold",
               safety_config.tracking.spin_omega_threshold,
               safety_config.tracking.spin_omega_threshold);
    pnh_.param("tracking_safety/spin_fail/max_duration_sec",
               safety_config.tracking.spin_max_duration_sec,
               safety_config.tracking.spin_max_duration_sec);
    safety_config.tracking.max_projection_distance_m = std::max(
        0.0, safety_config.tracking.max_projection_distance_m);
    safety_config.tracking.max_projection_duration_sec = std::max(
        0.0, safety_config.tracking.max_projection_duration_sec);
    safety_config.tracking.spin_omega_threshold = std::max(
        0.0, safety_config.tracking.spin_omega_threshold);
    safety_config.tracking.spin_max_duration_sec = std::max(
        0.0, safety_config.tracking.spin_max_duration_sec);
    std::string safety_error;
    if (!control_cycle_engine_.configureSafety(safety_config, safety_error)) {
        ROS_FATAL("[spmpc_local_planner] safety supervisor configuration failed: %s",
                  safety_error.c_str());
        return false;
    }
    pnh_.param("start_lock_recovery/enable", solver_params.start_lock_recovery.enable, solver_params.start_lock_recovery.enable);
    pnh_.param("start_lock_recovery/detect_only", solver_params.start_lock_recovery.detect_only, solver_params.start_lock_recovery.detect_only);
    pnh_.param("start_lock_recovery/start_window_s", solver_params.start_lock_recovery.start_window_s, solver_params.start_lock_recovery.start_window_s);
    pnh_.param("start_lock_recovery/min_stall_duration_sec", solver_params.start_lock_recovery.min_stall_duration_sec, solver_params.start_lock_recovery.min_stall_duration_sec);
    pnh_.param("start_lock_recovery/progress_epsilon_s", solver_params.start_lock_recovery.progress_epsilon_s, solver_params.start_lock_recovery.progress_epsilon_s);
    pnh_.param("start_lock_recovery/cmd_v_small_threshold", solver_params.start_lock_recovery.cmd_v_small_threshold, solver_params.start_lock_recovery.cmd_v_small_threshold);
    pnh_.param("start_lock_recovery/warm_start_v_s_min", solver_params.start_lock_recovery.warm_start_v_s_min, solver_params.start_lock_recovery.warm_start_v_s_min);
    pnh_.param("start_lock_recovery/u0_v_s_max", solver_params.start_lock_recovery.u0_v_s_max, solver_params.start_lock_recovery.u0_v_s_max);
    pnh_.param("start_lock_recovery/require_monotonic_clip", solver_params.start_lock_recovery.require_monotonic_clip, solver_params.start_lock_recovery.require_monotonic_clip);
    pnh_.param("start_lock_recovery/max_projection_distance_m", solver_params.start_lock_recovery.max_projection_distance_m, solver_params.start_lock_recovery.max_projection_distance_m);
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
        ROS_FATAL("[spmpc_local_planner] unknown solver_backend '%s'. Valid backends: %s, %s, %s",
                  solver_params.solver_backend.c_str(),
                  kSolverBackendContinuousMpccAcados,
                  kSolverBackendContinuousMpccDirectOmegaLegacy,
                  kSolverBackendPrimitive);
        return false;
    }
    solver_params.slosh = loadSloshParams();
    solver_params.slosh.dt = dt_;
    phase_rejoin_runtime_contract_ = PhaseRejoinRuntimeContract{};
    phase_rejoin_runtime_contract_.dt = dt_;
    phase_rejoin_runtime_contract_.min_command_v = 0.0;
    phase_rejoin_runtime_contract_.max_command_v = solver_params.v_max;
    phase_rejoin_runtime_contract_.max_abs_command_omega =
        solver_params.omega_max;
    SloshDynamics phase_contract_dynamics;
    phase_rejoin_runtime_contract_.liquid_model_configured =
        phase_contract_dynamics.configure(solver_params.slosh);
    if (phase_rejoin_runtime_contract_.liquid_model_configured) {
        const double omega_n = phase_contract_dynamics.omegaN();
        phase_rejoin_runtime_contract_.two_zeta_omega_n =
            2.0 * solver_params.slosh.damping_ratio * omega_n;
        phase_rejoin_runtime_contract_.omega_n_sq = omega_n * omega_n;
        // SloshDynamics uses unit gains for body-frame ax and ay.
        phase_rejoin_runtime_contract_.kappa_x = 1.0;
        phase_rejoin_runtime_contract_.kappa_y = 1.0;
    } else if (phase_rejoin_params_.mode != PhaseRejoinMode::Off) {
        ROS_FATAL("[spmpc_local_planner] phase-rejoin cannot derive the "
                  "runtime liquid-model contract");
        return false;
    }
    const ProcessedImuParams processed_imu_params = loadProcessedImuParams();
    slosh_risk_governor_params_ = loadSloshRiskGovernorParams();

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
    if (variant_.slosh_cost_horizon_steps < -1 ||
        !std::isfinite(variant_.slosh_cost_tail_discount) ||
        variant_.slosh_cost_tail_discount < 0.0 ||
        variant_.slosh_cost_tail_discount > 1.0) {
        ROS_FATAL("[spmpc_local_planner] invalid liquid cost horizon for variant=%s: steps=%d tail=%.6f",
                  variant_.name.c_str(),
                  variant_.slosh_cost_horizon_steps,
                  variant_.slosh_cost_tail_discount);
        return false;
    }
    const bool matched_development_variant =
        variant_.name == "B_slosh_matched0" ||
        variant_.name == "B_slosh_matched5";
    if (variant_.slosh_enable && state_timing_params_.require_common_epoch &&
        delay_phase_params_.mode == DelayPhaseMode::FixedRobotOnly) {
        ROS_FATAL("[spmpc_local_planner] fixed_robot_only is forbidden when a slosh solver requires a common robot/liquid epoch");
        return false;
    }
    if (phase_rejoin_params_.mode == PhaseRejoinMode::Enforce) {
        if (solver_params.solver_backend !=
                kSolverBackendContinuousMpccAcados ||
            !variant_.slosh_enable) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                      "the main 10D slosh acados backend");
            return false;
        }
        if (delay_phase_params_.mode != DelayPhaseMode::FixedClosedLoop) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                      "delay_phase=fixed_closed_loop");
            return false;
        }
        if (!delay_phase_params_.require_complete_history) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                      "delay_phase/require_complete_history=true");
            return false;
        }
        const bool post_solver_limiter_enabled =
            shared_cmd_linear_accel_limit_enable_ ||
            shared_cmd_angular_limit_enable_;
        if (post_solver_limiter_enabled &&
            !command_contract_params_.fail_closed_on_post_limit_change) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce with a "
                      "post-solver limiter requires "
                      "execution_contract/fail_closed_on_post_limit_change=true");
            return false;
        }
        if (!continuousMpccPhaseRejoinAvailable()) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce requires "
                      "the dedicated generated Phase-Rejoin acados solver; "
                      "it was not compiled into this package");
            return false;
        }
        const int phase_solver_horizon =
            continuousMpccPhaseRejoinHorizonSteps();
        if (phase_rejoin_params_.liquid_horizon_steps !=
            phase_solver_horizon) {
            ROS_FATAL("[spmpc_local_planner] phase_rejoin/enforce horizon "
                      "contract mismatch: configured liquid_horizon_steps=%d "
                      "but the dedicated generated solver has N=%d",
                      phase_rejoin_params_.liquid_horizon_steps,
                      phase_solver_horizon);
            return false;
        }
    }
    if (matched_development_variant && delayPhaseClosedLoopEnabled()) {
        ROS_FATAL("[spmpc_local_planner] matched short-horizon variants require delay_phase=off|monitor|shadow; command-history rollout is audit-only until validated");
        return false;
    }
    if (matched_development_variant) {
        const bool common_weights =
            std::abs(variant_.w_contour - 1.0) <= 1e-12 &&
            std::abs(variant_.w_lag - 0.2) <= 1e-12 &&
            std::abs(variant_.w_progress - 0.2) <= 1e-12 &&
            std::abs(variant_.w_v - 1.0) <= 1e-12 &&
            std::abs(variant_.w_vs - 0.3) <= 1e-12 &&
            std::abs(variant_.v_ref - 0.20) <= 1e-12 &&
            std::abs(variant_.w_control - 0.3) <= 1e-12 &&
            std::abs(variant_.w_smooth - 1.0) <= 1e-12 &&
            std::abs(variant_.w_alpha - 1.0) <= 1e-12 &&
            std::abs(variant_.w_du_a - 1.0) <= 1e-12 &&
            std::abs(variant_.w_du_vs - 1.0) <= 1e-12 &&
            std::abs(variant_.w_accel) <= 1e-12;
        const double expected_slosh_weight =
            variant_.name == "B_slosh_matched5" ? 5.0 : 0.0;
        const bool release_contract =
            solver_params.solver_backend == kSolverBackendContinuousMpccAcados &&
            variant_.slosh_enable && variant_.smooth_priority_enable &&
            !variant_.slosh_constraint_enable && common_weights &&
            std::abs(variant_.w_slosh - expected_slosh_weight) <= 1e-12 &&
            variant_.slosh_cost_horizon_steps == 3 &&
            std::abs(variant_.slosh_cost_tail_discount) <= 1e-12 &&
            slosh_observer_selector_params_.nominal_source ==
                SloshObserverSource::ProcessedImu &&
            slosh_observer_selector_params_.fallback_policy ==
                SloshObserverFallbackPolicy::FailClosed &&
            state_timing_params_.require_common_epoch &&
            delay_phase_params_.mode == DelayPhaseMode::Shadow &&
            !shared_cmd_linear_accel_limit_enable_ &&
            !shared_cmd_angular_limit_enable_ &&
            command_contract_params_.fail_closed_on_post_limit_change;
        if (!release_contract) {
            ROS_FATAL("[spmpc_local_planner] matched development release contract rejected variant=%s; require main 10D solver, processed_imu/fail_closed, common epoch, delay shadow, 3-step liquid cost, common weights, disabled redundant limiters, and fail-closed command audit",
                      variant_.name.c_str());
            return false;
        }
    }

    std::string policy_error;
    if (!validateBackendPolicy(solver_params, variant_, policy_error)) {
        ROS_FATAL("[spmpc_local_planner] backend policy rejected backend=%s role=%s variant=%s: %s",
                  solver_params.solver_backend.c_str(),
                  solverBackendRole(solver_params.solver_backend),
                  variant_.name.c_str(),
                  policy_error.c_str());
        return false;
    }

    ROS_INFO("[spmpc_local_planner] backend=%s role=%s variant=%s mode=%s features: slosh=%s slosh_constraint=%s obstacle=%s homotopy=%s corridor=%s corridor_hard_bound=%s",
             solver_params.solver_backend.c_str(),
             solverBackendRole(solver_params.solver_backend),
             variant_.name.c_str(),
             experiment_mode_.c_str(),
             boolText(variant_.slosh_enable),
             boolText(variant_.slosh_constraint_enable),
             boolText(solver_params.obstacle_enable),
             boolText(solver_params.homotopy_enable),
             boolText(solver_params.corridor_enable),
             boolText(solver_params.corridor_hard_bound_enable));

    effective_config_.solver_backend_code = solverBackendCode(solver_params.solver_backend);
    effective_config_.control_frequency = control_frequency_;
    effective_config_.dt = dt_;
    effective_config_.horizon_steps = static_cast<double>(horizon_steps_);
    effective_config_.slosh_enable = variant_.slosh_enable ? 1.0 : 0.0;
    effective_config_.slosh_constraint_enable = variant_.slosh_constraint_enable ? 1.0 : 0.0;
    effective_config_.smooth_priority_enable = variant_.smooth_priority_enable ? 1.0 : 0.0;
    effective_config_.primitive_mode_code = primitiveModeCode(variant_.primitive_mode);
    effective_config_.v_ref = variant_.v_ref;
    effective_config_.w_slosh = variant_.w_slosh;
    effective_config_.w_control = variant_.w_control;
    effective_config_.w_smooth = variant_.w_smooth;
    effective_config_.w_accel = variant_.w_accel;
    effective_config_.w_alpha = variant_.w_alpha;
    effective_config_.w_du_a = variant_.w_du_a;
    effective_config_.w_du_vs = variant_.w_du_vs;
    effective_config_.v_max = solver_params.v_max;
    effective_config_.omega_max = solver_params.omega_max;
    effective_config_.a_max = solver_params.a_max;
    effective_config_.alpha_max = solver_params.alpha_max;
    effective_config_.shared_linear_accel_limit_enable = shared_cmd_linear_accel_limit_enable_ ? 1.0 : 0.0;
    effective_config_.shared_linear_accel_max = shared_cmd_linear_accel_max_;
    effective_config_.shared_linear_accel_max_dt = shared_cmd_linear_accel_max_dt_;
    effective_config_.shared_angular_limit_enable = shared_cmd_angular_limit_enable_ ? 1.0 : 0.0;
    effective_config_.shared_angular_rate_max = shared_cmd_angular_rate_max_;
    effective_config_.shared_angular_accel_max = shared_cmd_angular_accel_max_;
    effective_config_.shared_angular_accel_max_dt = shared_cmd_angular_accel_max_dt_;
    effective_config_.container_radius = solver_params.slosh.container_radius;
    effective_config_.liquid_height = solver_params.slosh.liquid_height;
    effective_config_.damping_ratio = solver_params.slosh.damping_ratio;
    effective_config_.slosh_height_ref = solver_params.slosh.slosh_height_ref;
    effective_config_.slosh_height_max = solver_params.slosh.slosh_height_max;
    effective_config_.slosh_eta_dot_ratio = solver_params.slosh.slosh_eta_dot_ratio;
    effective_config_.use_parabola_term = solver_params.slosh.use_parabola_term ? 1.0 : 0.0;
    effective_config_.delay_phase_mode_code = static_cast<double>(static_cast<int>(delay_phase_params_.mode));
    effective_config_.delay_linear_sec = delay_phase_params_.linear_delay_sec;
    effective_config_.delay_angular_sec = delay_phase_params_.angular_delay_sec;
    effective_config_.delay_cmd_timeout_sec = delay_phase_params_.cmd_timeout_sec;
    effective_config_.delay_odom_timeout_sec = delay_phase_params_.odom_timeout_sec;
    effective_config_.delay_history_window_sec = delay_phase_params_.history_window_sec;
    effective_config_.delay_require_complete_history = delay_phase_params_.require_complete_history ? 1.0 : 0.0;
    effective_config_.slosh_cost_horizon_steps =
        static_cast<double>(variant_.slosh_cost_horizon_steps);
    effective_config_.slosh_cost_horizon_sec =
        variant_.slosh_cost_horizon_steps < 0
            ? -1.0
            : static_cast<double>(variant_.slosh_cost_horizon_steps) * dt_;
    effective_config_.slosh_cost_tail_discount =
        variant_.slosh_cost_tail_discount;
    effective_config_.state_timing_require_common_epoch =
        state_timing_params_.require_common_epoch ? 1.0 : 0.0;
    effective_config_.state_timing_max_raw_skew_sec =
        state_timing_params_.max_raw_skew_sec;
    effective_config_.w_contour = variant_.w_contour;
    effective_config_.w_lag = variant_.w_lag;
    effective_config_.w_progress = variant_.w_progress;
    effective_config_.w_v = variant_.w_v;
    effective_config_.w_vs = variant_.w_vs;

    const SolverConfigureResult solver_configure =
        problem_.configure(solver_params, variant_);
    if (!solver_configure.success) {
        ROS_FATAL("[spmpc_local_planner] solver configure failed status=%s detail=%s",
                  solver_configure.status.c_str(),
                  solver_configure.detail.c_str());
        return false;
    }
    if (!solver_configure.detail.empty()) {
        ROS_WARN("[spmpc_local_planner] solver configure status=%s detail=%s",
                 solver_configure.status.c_str(),
                 solver_configure.detail.c_str());
    }
    if (!slosh_observers_.configure(solver_params.slosh, imu_observer_dt_sec_)) {
        ROS_WARN("[spmpc_local_planner] slosh observer configure failed; slosh diagnostics stay zero");
    }
    if (!slosh_observer_selector_.configure(slosh_observer_selector_params_)) {
        ROS_FATAL("[spmpc_local_planner] invalid slosh_observer selector parameters");
        return false;
    }
    if (imu_shadow_enable_ && !slosh_observers_.imuConfigured()) {
        if (imu_is_nominal) {
            ROS_FATAL("[spmpc_local_planner] nominal processed-IMU observer configure failed");
            return false;
        }
        ROS_ERROR("[spmpc_local_planner] IMU shadow observer configure failed; disabling shadow");
        imu_shadow_enable_ = false;
    }
    if (imu_shadow_enable_ &&
        !imu_shadow_adapter_.configure(processed_imu_params, imu_expected_frame_)) {
        if (imu_is_nominal) {
            ROS_FATAL("[spmpc_local_planner] nominal processed-IMU pipeline configure failed");
            return false;
        }
        ROS_ERROR("[spmpc_local_planner] processed-IMU pipeline configure failed; disabling shadow");
        imu_shadow_enable_ = false;
    }
    if (!execution_predictor_.configure(solver_params.slosh)) {
        ROS_WARN("[spmpc_local_planner] delay_phase shadow slosh predictor configure failed; shadow slosh stays pass-through");
    }
    if (!slosh_risk_governor_.configure(solver_params.slosh, slosh_risk_governor_params_) &&
        slosh_risk_governor_params_.enable) {
        ROS_WARN("[spmpc_local_planner] slosh risk governor configure failed; governor will pass through v_ref");
    }
    obstacle_enable_ = solver_params.obstacle_enable;

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &SpmpcLocalPlannerROS::odomCallback, this);
    if (imu_shadow_enable_) {
        imu_nh_ = nh_;
        imu_nh_.setCallbackQueue(&imu_callback_queue_);
        imu_sub_ = imu_nh_.subscribe<sensor_msgs::Imu>(
            imu_topic_,
            static_cast<std::uint32_t>(imu_subscriber_queue_size_),
            &SpmpcLocalPlannerROS::imuCallback,
            this,
            ros::TransportHints().tcpNoDelay());
    }
    path_sub_ = nh_.subscribe(path_topic_, 1, &SpmpcLocalPlannerROS::pathCallback, this);
    if (obstacle_enable_) {
        costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &SpmpcLocalPlannerROS::costmapCallback, this);
    }
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);

    ros::NodeHandle spmpc_nh(nh_, "spmpc");
    diagnostics_.initialize(spmpc_nh);
    diagnostics_.publishVariant(variant_, experiment_mode_);
    diagnostics_.publishSolverBackend(solver_params.solver_backend);
    diagnostics_.publishEffectiveConfig(effective_config_);
    diagnostics_.publishStatus("INITIALIZED");
    if (phase_rejoin_publish_diagnostics_) {
        diagnostics_.publishPhaseRejoin(
            control_cycle_engine_.makePhaseRejoinDebug(nullptr, nullptr),
            ControlCycleTimingDebug{},
            phase_rejoin_params_.required_frame_id);
    }

    const double period = 1.0 / std::max(1.0, control_frequency_);
    control_timer_ = nh_.createTimer(ros::Duration(period), &SpmpcLocalPlannerROS::controlTimerCallback, this);
    if (imu_shadow_enable_) {
        imu_spinner_.reset(new ros::AsyncSpinner(1, &imu_callback_queue_));
        imu_spinner_->start();
    }

    ROS_INFO("[spmpc_local_planner] initialized variant=%s mode=%s path_topic=%s costmap_topic=%s cmd_topic=%s imu_pipeline=%s imu_topic=%s imu_queue=%d observer_source=%s observer_fallback=%s latch_fallback=%s phase_rejoin=%s phase_artifact=%s",
             variant_.name.c_str(),
             experiment_mode_.c_str(),
             path_topic_.c_str(),
             costmap_topic_.c_str(),
             cmd_topic_.c_str(),
             boolText(imu_shadow_enable_),
             imu_topic_.c_str(),
             imu_subscriber_queue_size_,
             sloshObserverSourceName(slosh_observer_selector_params_.nominal_source),
             sloshObserverFallbackPolicyName(
                 slosh_observer_selector_params_.fallback_policy),
             boolText(slosh_observer_selector_params_.latch_fallback),
             phaseRejoinModeName(phase_rejoin_params_.mode).c_str(),
             phase_rejoin_artifact_path_.empty()
                 ? "<none>"
                 : phase_rejoin_artifact_path_.c_str());
    return true;
}

void SpmpcLocalPlannerROS::spin() {
    ros::spin();
}

void SpmpcLocalPlannerROS::resetMapVRefProgress() {
    map_vref_last_progress_abs_s_ = 0.0;
    have_map_vref_progress_ = false;
}


bool SpmpcLocalPlannerROS::loadMapVRefProfile(const std::string& path) {
    const SpeedProfileLoadResult result = map_vref_profile_.loadCsv(path);
    if (!result.success) {
        ROS_WARN("[spmpc_local_planner] map_vref profile load failed "
                 "status=%s detail=%s",
                 result.status.c_str(), result.detail.c_str());
        return false;
    }
    ROS_INFO("[spmpc_local_planner] loaded map_vref profile %s with %zu "
             "samples (%zu invalid rows skipped)",
             path.c_str(), result.accepted_rows, result.skipped_rows);
    return true;
}

bool SpmpcLocalPlannerROS::ensureMapVRefProfileLoaded(
    const std::string& path) {
    if (path.empty()) {
        map_vref_profile_.clear();
        return false;
    }
    if (!map_vref_profile_.empty() &&
        map_vref_profile_.sourcePath() == path) {
        return true;
    }
    return loadMapVRefProfile(path);
}

bool SpmpcLocalPlannerROS::lookupMapVRef(double progress_m,
                                         double& speed_mps) const {
    return map_vref_profile_.lookup(progress_m, speed_mps);
}

void SpmpcLocalPlannerROS::applyRuntimeVRef(SolverInput& input) {
    const auto& config = app_config_.map_vref;
    if (config.runtime_override_enable) {
        input.has_v_ref_current = true;
        input.v_ref_current = config.runtime_override_mps;
        input.v_ref_status = "RUNTIME_OVERRIDE";
        return;
    }

    if (!config.profile_enable) {
        input.v_ref_status = "VARIANT_FALLBACK";
        return;
    }
    if (map_vref_profile_.empty()) {
        input.v_ref_status = config.profile_path.empty()
            ? "PROFILE_NOT_CONFIGURED"
            : "PROFILE_LOAD_FAILED";
        return;
    }

    const double current_s = have_map_vref_progress_ ? map_vref_last_progress_abs_s_ : 0.0;
    const double lookup_s = current_s + config.profile_lookahead_m;
    double profile_v_ref = 0.0;
    if (!lookupMapVRef(lookup_s, profile_v_ref)) {
        input.v_ref_status = "PROFILE_LOOKUP_FAILED";
        return;
    }
    input.has_v_ref_current = true;
    input.v_ref_current = profile_v_ref;
    input.v_ref_status = "PROFILE_LOOKUP";
}

void SpmpcLocalPlannerROS::applySloshRiskGovernor(SolverInput& input) {
    const double nominal_v_ref = input.has_v_ref_current ? input.v_ref_current : variant_.v_ref;
    SloshRiskGovernorInput governor_input;
    governor_input.slosh = input.slosh;
    governor_input.robot_v = input.robot.v;
    governor_input.robot_omega = input.robot.omega;
    governor_input.nominal_v_ref = nominal_v_ref;
    governor_input.dt = input.dt;
    governor_input.slosh_variant_enabled = variant_.slosh_enable;

    last_slosh_governor_output_ = slosh_risk_governor_.update(governor_input);
    diagnostics_.publishSloshGovernor(last_slosh_governor_output_);

    if (!last_slosh_governor_output_.enabled ||
        last_slosh_governor_output_.status == "DISABLED" ||
        last_slosh_governor_output_.status == "NOT_SLOSH_VARIANT" ||
        last_slosh_governor_output_.status == "INVALID_CONFIG" ||
        !std::isfinite(last_slosh_governor_output_.governed_v_ref)) {
        return;
    }

    input.has_v_ref_current = true;
    input.v_ref_current = last_slosh_governor_output_.governed_v_ref;
    input.v_ref_status = appendVRefStatus(input.v_ref_status, "SLOSH_GOVERNOR");
}

bool SpmpcLocalPlannerROS::delayPhaseActive() const {
    return delay_phase_params_.publish_diagnostics && delay_phase_params_.mode != DelayPhaseMode::Off;
}

bool SpmpcLocalPlannerROS::delayPhasePredictionEnabled() const {
    return delayPhaseUsesPrediction(delay_phase_params_.mode);
}

bool SpmpcLocalPlannerROS::delayPhaseClosedLoopEnabled() const {
    return delayPhaseUsesClosedLoop(delay_phase_params_.mode);
}

void SpmpcLocalPlannerROS::recordPublishedCommand(
    const geometry_msgs::Twist& cmd,
    const ros::Time& stamp,
    const CommandPublishMeta& meta) {
    TimedCommandSample sample;
    sample.stamp_ns = static_cast<StampNs>(stamp.toNSec());
    sample.command.linear = cmd.linear.x;
    sample.command.angular = cmd.angular.z;
    sample.meta = meta;
    command_history_.push(sample);
}

void SpmpcLocalPlannerROS::publishDelayPhaseDiagnostics(
    const ros::Time& now,
    DelayPhaseStatusCode status_code,
    const ExecutionStatePrediction* prediction,
    double solver_time_ms,
    bool closed_loop_enabled) {
    if (!delayPhaseActive()) {
        return;
    }

    DelayPhaseStatusCode effective_status = status_code;
    const bool has_any_history = !command_history_.empty();
    const double history_span_sec = command_history_.spanSec();
    // has_any_history 只表示是否收到过命令；history_complete 另按补偿窗口判断。
    // 补偿窗口与 ExecutionStatePredictor 一致：max(linear_delay, angular_delay) 且受 max_prediction_sec 限制，
    // 不再误用 history_window_sec（它是 buffer 保留窗口，通常远大于实际补偿窗口）。
    double required_history_sec = std::max(0.0, std::max(delay_phase_params_.linear_delay_sec,
                                                        delay_phase_params_.angular_delay_sec));
    required_history_sec = std::min(required_history_sec, std::max(0.0, delay_phase_params_.max_prediction_sec));
    const bool fallback_history_complete =
        has_any_history && (required_history_sec <= 1e-6 || history_span_sec + 1e-6 >= required_history_sec);
    const double fallback_covered_history_sec = has_any_history ? std::min(history_span_sec, required_history_sec) : 0.0;
    const double fallback_missing_history_sec =
        has_any_history ? std::max(0.0, required_history_sec - history_span_sec) : required_history_sec;
    const double cmd_age_sec = has_any_history
        ? secondsBetween(static_cast<StampNs>(now.toNSec()),
                         command_history_.latestStampNs())
        : -1.0;
    const bool have_odom_receive = !last_odom_receive_stamp_.isZero();
    const double odom_age_sec = have_odom_receive ? (now - last_odom_receive_stamp_).toSec() : -1.0;
    const auto status_requires_freshness = [](DelayPhaseStatusCode status) {
        return status == DelayPhaseStatusCode::MonitorOk ||
               status == DelayPhaseStatusCode::ShadowOk ||
               status == DelayPhaseStatusCode::FixedClosedLoopOk ||
               status == DelayPhaseStatusCode::FixedRobotOnlyOk ||
               status == DelayPhaseStatusCode::PartialHistory;
    };

    if ((effective_status == DelayPhaseStatusCode::MonitorOk ||
         effective_status == DelayPhaseStatusCode::ShadowOk ||
         effective_status == DelayPhaseStatusCode::FixedClosedLoopOk ||
         effective_status == DelayPhaseStatusCode::FixedRobotOnlyOk) &&
        !has_any_history) {
        effective_status = DelayPhaseStatusCode::NoCmdHistory;
        closed_loop_enabled = false;
    }
    if (status_requires_freshness(effective_status) &&
        delay_phase_params_.cmd_timeout_sec > 0.0 && cmd_age_sec > delay_phase_params_.cmd_timeout_sec) {
        effective_status = DelayPhaseStatusCode::CmdStale;
        closed_loop_enabled = false;
    }
    if (status_requires_freshness(effective_status) &&
        delay_phase_params_.odom_timeout_sec > 0.0 && odom_age_sec > delay_phase_params_.odom_timeout_sec) {
        effective_status = DelayPhaseStatusCode::OdomStale;
        closed_loop_enabled = false;
    }

    DelayPhaseDebugSummary summary;
    summary.mode = delay_phase_params_.mode;
    summary.cmd_age_ms = cmd_age_sec >= 0.0 ? 1000.0 * cmd_age_sec : -1.0;
    summary.cmd_period_ms = command_history_.latestPeriodSec() > 0.0 ? 1000.0 * command_history_.latestPeriodSec() : -1.0;
    summary.odom_age_ms = odom_age_sec >= 0.0 ? 1000.0 * odom_age_sec : -1.0;
    summary.solver_time_ms = solver_time_ms;
    summary.linear_delay_ms = 1000.0 * delay_phase_params_.linear_delay_sec;
    summary.angular_delay_ms = 1000.0 * delay_phase_params_.angular_delay_sec;
    summary.history_span_ms = 1000.0 * history_span_sec;
    summary.history_complete = prediction ? prediction->history_complete : fallback_history_complete;
    summary.shadow_valid = prediction ? prediction->valid : false;
    summary.closed_loop_enabled = closed_loop_enabled;
    summary.status_code = effective_status;

    CmdOdomAlignmentDebug alignment;
    alignment.mode = delay_phase_params_.mode;
    alignment.cmd_age_ms = summary.cmd_age_ms;
    alignment.cmd_period_ms = summary.cmd_period_ms;
    alignment.odom_age_ms = summary.odom_age_ms;
    alignment.odom_period_ms = last_odom_timing_.stamp_dt_ms;
    alignment.linear_delay_ms = summary.linear_delay_ms;
    alignment.angular_delay_ms = summary.angular_delay_ms;
    alignment.history_span_ms = summary.history_span_ms;
    alignment.covered_history_ms = prediction ? 1000.0 * prediction->covered_history_sec
                                              : 1000.0 * fallback_covered_history_sec;
    alignment.missing_history_ms = prediction ? 1000.0 * prediction->missing_history_sec
                                              : 1000.0 * fallback_missing_history_sec;
    alignment.history_complete = summary.history_complete;
    alignment.shadow_valid = summary.shadow_valid;
    alignment.fixed_closed_loop_configured = delayPhaseClosedLoopEnabled();
    alignment.fixed_closed_loop_applied = closed_loop_enabled;
    alignment.status_code = effective_status;
    if (prediction) {
        alignment.dx_pred_raw = prediction->predicted_robot.x - prediction->raw_robot.x;
        alignment.dy_pred_raw = prediction->predicted_robot.y - prediction->raw_robot.y;
        alignment.dyaw_pred_raw = wrapAngle(prediction->predicted_robot.yaw - prediction->raw_robot.yaw);
        alignment.dv_pred_raw = prediction->predicted_robot.v - prediction->raw_robot.v;
        alignment.domega_pred_raw = prediction->predicted_robot.omega - prediction->raw_robot.omega;
        alignment.deta_norm_pred_raw = std::hypot(prediction->predicted_slosh.eta_x - prediction->raw_slosh.eta_x,
                                                  prediction->predicted_slosh.eta_y - prediction->raw_slosh.eta_y);
        alignment.deta_dot_norm_pred_raw = std::hypot(prediction->predicted_slosh.eta_x_dot - prediction->raw_slosh.eta_x_dot,
                                                      prediction->predicted_slosh.eta_y_dot - prediction->raw_slosh.eta_y_dot);
    }

    diagnostics_.publishDelayPhase(summary);
    diagnostics_.publishOdomTiming(last_odom_timing_);
    diagnostics_.publishDelayCompensation(summary);
    diagnostics_.publishCmdOdomAlignment(alignment);
    diagnostics_.publishExecutionAlignmentStatus(
        prediction && !prediction->status.empty() && effective_status == prediction->status_code
            ? prediction->status
            : delayPhaseStatusName(effective_status));
    if (prediction && delayPhasePredictionEnabled()) {
        diagnostics_.publishExecutionState(*prediction);
    }
}

void SpmpcLocalPlannerROS::publishDelayPhaseEarlyStatus(DelayPhaseStatusCode status_code) {
    publishDelayPhaseDiagnostics(ros::Time::now(), status_code, nullptr, 0.0);
}

void SpmpcLocalPlannerROS::publishZeroCommand(
    const CommandInterventionDebug& intervention,
    ControlCycleAuditDebug* audit) {
    const ros::Time stamp = ros::Time::now();
    CommandPipelineRequest request;
    request.stamp_ns = static_cast<StampNs>(stamp.toNSec());
    request.force_zero = true;
    request.publish_enabled = publish_cmd_vel_;
    request.source = CommandSource::FailClosed;
    request.reason = audit && !audit->status.empty()
        ? audit->status
        : "FAIL_CLOSED_ZERO";
    const CommandPipelineResult result = command_pipeline_.finalize(request);

    CommandInterventionDebug debug = intervention;
    debug.publish_cmd_vel = publish_cmd_vel_;
    debug.published_cmd_v = result.command_was_published
        ? result.final_command.linear
        : 0.0;
    debug.published_cmd_omega = result.command_was_published
        ? result.final_command.angular
        : 0.0;
    diagnostics_.publishCommandIntervention(debug);

    if (result.command_was_published) {
        const geometry_msgs::Twist command =
            velocityCommandToRos(result.final_command);
        CommandPublishMeta meta;
        meta.is_zero_cmd = true;
        recordPublishedCommand(command, stamp, meta);
        cmd_pub_.publish(command);
    }

    if (audit) {
        audit->publish_cmd_vel = publish_cmd_vel_;
        audit->command_was_published = result.command_was_published;
        audit->published_cmd_v = debug.published_cmd_v;
        audit->published_cmd_omega = debug.published_cmd_omega;
        if (result.command_was_published) {
            audit->timing.command_publish_stamp_ns = request.stamp_ns;
        }
        diagnostics_.publishControlCycleAudit(
            *audit, problem_.referenceFrameId());
    }
}

void SpmpcLocalPlannerROS::publishCommand(
    const CommandDecision& decision,
    const CommandInterventionDebug& intervention,
    ControlCycleAuditDebug* audit) {
    const ros::Time stamp = ros::Time::now();
    CommandPipelineRequest request;
    request.stamp_ns = static_cast<StampNs>(stamp.toNSec());
    request.desired = decision.command;
    request.publish_enabled = publish_cmd_vel_;
    request.source = decision.source;
    request.reason = decision.reason;

    const CommandPipelineResult result = command_pipeline_.finalize(request);
    const geometry_msgs::Twist desired =
        velocityCommandToRos(decision.command);
    const geometry_msgs::Twist command =
        velocityCommandToRos(result.final_command);
    const geometry_msgs::Twist previous =
        velocityCommandToRos(result.previous);

    if (result.decision.source == CommandSource::ExecutionContract) {
        diagnostics_.publishStatus(result.decision.reason);
    }

    CommandInterventionDebug debug = intervention;
    debug.publish_cmd_vel = publish_cmd_vel_;
    debug.published_cmd_v = result.command_was_published
        ? result.final_command.linear
        : 0.0;
    debug.published_cmd_omega = result.command_was_published
        ? result.final_command.angular
        : 0.0;
    debug.linear_limited = result.linear_limited;
    debug.angular_rate_limited = result.angular_rate_limited;
    debug.angular_accel_limited = result.angular_accel_limited;
    debug.zero_due_to_command_contract =
        result.decision.source == CommandSource::ExecutionContract;
    diagnostics_.publishCommandIntervention(debug);

    if (result.command_was_published) {
        CommandPublishMeta meta;
        meta.is_zero_cmd =
            std::abs(result.final_command.linear) <= 1e-9 &&
            std::abs(result.final_command.angular) <= 1e-9;
        meta.linear_limited = result.linear_limited;
        meta.angular_rate_limited = result.angular_rate_limited;
        meta.angular_accel_limited = result.angular_accel_limited;
        recordPublishedCommand(command, stamp, meta);
        diagnostics_.publishCommandOutput(
            desired, command, previous, result.limiter_dt_sec,
            result.linear_limited, result.angular_rate_limited,
            result.angular_accel_limited);
        cmd_pub_.publish(command);
    }

    if (audit) {
        audit->publish_cmd_vel = publish_cmd_vel_;
        audit->command_was_published = result.command_was_published;
        if (result.command_was_published) {
            audit->timing.command_publish_stamp_ns = request.stamp_ns;
            audit->command_contract_violation =
                result.command_contract_violation;
            audit->linear_limited = result.linear_limited;
            audit->angular_rate_limited = result.angular_rate_limited;
            audit->angular_accel_limited = result.angular_accel_limited;
            audit->published_cmd_v = result.final_command.linear;
            audit->published_cmd_omega = result.final_command.angular;
        } else {
            audit->published_cmd_v = 0.0;
            audit->published_cmd_omega = 0.0;
        }
        if (result.decision.source == CommandSource::ExecutionContract) {
            audit->status = result.decision.reason;
        }
        diagnostics_.publishControlCycleAudit(
            *audit, problem_.referenceFrameId());
    }
}


void SpmpcLocalPlannerROS::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    const ros::Time receive_stamp = ros::Time::now();
    if (!processOdomInput(*msg, receive_stamp)) {
        return;
    }
    // Commit odom to the formal control path only after the same monotonicity
    // and finite-value checks used by the liquid-observer input boundary.
    last_odom_receive_stamp_ = receive_stamp;
    last_odom_ = *msg;
    have_odom_ = true;
    appendOdomStateHistory(*msg);
}

void SpmpcLocalPlannerROS::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    const ProcessedImuOutput output = imu_shadow_adapter_.process(*msg, ros::Time::now());
    bool observer_step_ok = false;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        if (output.excitation.valid) {
            observer_step_ok = slosh_observers_.stepImu(output.excitation);
        } else {
            // Invalid samples never advance the IMU observer and never reuse
            // the previous acceleration. Epoch changes clear its modal state.
            slosh_observers_.invalidateImu(output.reset_epoch);
        }
        imu_input_ready_ = observer_step_ok &&
                           output.status == ImuPipelineStatusCode::Ready &&
                           output.bias_ready && output.filter_ready &&
                           slosh_observers_.imu().valid;
        imu_input_reset_epoch_ = output.reset_epoch;
    }
    if (output.excitation.valid && !observer_step_ok) {
        ROS_WARN_THROTTLE(1.0,
                          "[spmpc_local_planner] processed-IMU valid but observer step failed");
    }

    if (output.status == ImuPipelineStatusCode::FrameMismatch) {
        ROS_WARN_THROTTLE(2.0,
                          "[spmpc_local_planner] IMU frame mismatch: expected '%s', got '%s'",
                          imu_expected_frame_.c_str(),
                          msg->header.frame_id.c_str());
    } else if (output.status == ImuPipelineStatusCode::BiasInsufficient ||
               output.status == ImuPipelineStatusCode::BiasMotionDetected) {
        ROS_WARN_THROTTLE(2.0,
                          "[spmpc_local_planner] processed-IMU calibration unavailable: %s",
                          imuPipelineStatusName(output.status));
    }

    if (imu_shadow_publish_diagnostics_) {
        publishImuSloshObserverDebug(*msg, output);
    }
}

bool SpmpcLocalPlannerROS::phaseRejoinNeedsPrediction() const {
    return phase_rejoin_params_.mode != PhaseRejoinMode::Off;
}

void SpmpcLocalPlannerROS::validatePhaseRejoinReference(
    const ReferencePath& reference) {
    if (phase_rejoin_params_.mode == PhaseRejoinMode::Off) {
        return;
    }
    std::string error;
    if (!control_cycle_engine_.validatePhaseRejoinRuntimeContract(
            phase_rejoin_runtime_contract_, reference, error)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] phase-rejoin runtime contract rejected: %s "
            "dt=%.9f path_length=%.6f frame=%s",
            error.c_str(), dt_, reference.length(),
            reference.frameId().c_str());
        return;
    }
    ROS_INFO("[spmpc_local_planner] phase-rejoin runtime contract accepted "
             "dt=%.9f path_length=%.6f frame=%s contract=%s",
             dt_, reference.length(), reference.frameId().c_str(),
             control_cycle_engine_.phaseRejoinCoordinator()
                 .artifact().metadata().contract_id.c_str());
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
        const bool reference_changed = updateReferenceSignature(
            transformed_path);
        if (reference_changed) {
            control_cycle_engine_.resetForReference();
            resetMapVRefProgress();
            slosh_risk_governor_.reset();
        }
        const ReferencePath reference = referencePathFromMsg(transformed_path);
        problem_.setReferencePath(reference);
        if (reference_changed ||
            !control_cycle_engine_.phaseRejoinContractValid()) {
            validatePhaseRejoinReference(reference);
        }
        return;
    }

    const auto reference = referencePathFromMsg(*msg);
    const bool reference_changed = updateReferenceSignature(*msg);
    if (reference_changed) {
        control_cycle_engine_.resetForReference();
        resetMapVRefProgress();
        slosh_risk_governor_.reset();
    }
    problem_.setReferencePath(reference);
    if (reference_changed ||
        !control_cycle_engine_.phaseRejoinContractValid()) {
        validatePhaseRejoinReference(reference);
    }
}

void SpmpcLocalPlannerROS::costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    problem_.setCostmap(costmapFromMsg(*msg));
}

void SpmpcLocalPlannerROS::controlTimerCallback(const ros::TimerEvent& event) {
    // publishVariant 内容（variant code、experiment_mode）在运行期不变，已在初始化时发布一次（latched）。
    // publishEffectiveConfig 等 applyRuntimeVRef() 计算完本周期 v_ref 后再发，避免动态 v_ref 滞后一拍。

    const ros::Time cycle_start = ros::Time::now();
    ControlCycleAuditDebug cycle_audit;
    cycle_audit.timing.cycle_id = ++next_cycle_id_;
    cycle_audit.timing.cycle_start_stamp_ns =
        static_cast<std::int64_t>(cycle_start.toNSec());
    cycle_audit.variant = variant_.name;
    cycle_audit.publish_cmd_vel = publish_cmd_vel_;

    if (!have_odom_) {
        control_cycle_engine_.resetSafety();
        diagnostics_.publishStatus("WAITING_FOR_ODOM");
        publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoOdom);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_odom = true;
        cycle_audit.status = "WAITING_FOR_ODOM";
        publishZeroCommand(intervention, &cycle_audit);
        return;
    }
    if (!problem_.hasReferencePath()) {
        control_cycle_engine_.resetSafety();
        diagnostics_.publishStatus("WAITING_FOR_REFERENCE_PATH");
        publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoReference);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_reference = true;
        cycle_audit.status = "WAITING_FOR_REFERENCE_PATH";
        publishZeroCommand(intervention, &cycle_audit);
        return;
    }

    SolverInput input;
    SloshObserverHealth odom_observer_health;
    SloshObserverHealth imu_observer_health;
    double slosh_height_coeff = 0.0;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        odom_observer_health.snapshot = slosh_observers_.odom();
        // Odom has no separate bias/filter state. Snapshot validity and age are
        // its complete admission contract.
        odom_observer_health.input_ready = odom_observer_health.snapshot.valid;
        imu_observer_health.snapshot = slosh_observers_.imu();
        imu_observer_health.input_ready = imu_input_ready_;
        imu_observer_health.input_reset_epoch = imu_input_reset_epoch_;
        slosh_height_coeff = slosh_observers_.heightCoeff();
    }
    const ros::Time observer_selection_now = ros::Time::now();
    const SloshObserverSelection observer_selection =
        slosh_observer_selector_.select(
            odom_observer_health,
            imu_observer_health,
            static_cast<std::int64_t>(observer_selection_now.toNSec()));
    const bool phase_rejoin_enforce =
        phase_rejoin_params_.mode == PhaseRejoinMode::Enforce;
    const bool solver_consumes_selected_state =
        variant_.slosh_enable || phase_rejoin_enforce;
    cycle_audit.observer_source =
        static_cast<std::uint8_t>(observer_selection.effective_source);
    cycle_audit.odom_excitation = makeExcitationAudit(
        odom_observer_health.snapshot.excitation);
    cycle_audit.imu_excitation = makeExcitationAudit(
        imu_observer_health.snapshot.excitation);
    cycle_audit.timing.raw_robot_state_stamp_ns =
        static_cast<std::int64_t>(last_odom_.header.stamp.toNSec());
    cycle_audit.timing.raw_liquid_state_stamp_ns =
        observer_selection.selected_state_stamp_ns;
    cycle_audit.timing.state_alignment_required =
        solver_consumes_selected_state &&
        state_timing_params_.require_common_epoch;

    if (solver_consumes_selected_state && !observer_selection.valid) {
        control_cycle_engine_.resetSafety();
        const std::string selection_status =
            std::string("WAITING_FOR_SLOSH_OBSERVER_") +
            sloshObserverSelectionStatusName(observer_selection.status) + "_" +
            sloshObserverSelectionReasonName(observer_selection.reason);
        diagnostics_.publishStatus(selection_status);
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] fail-closed liquid observer: nominal=%s status=%s reason=%s odom_age=%.3f imu_age=%.3f",
            sloshObserverSourceName(observer_selection.nominal_source),
            sloshObserverSelectionStatusName(observer_selection.status),
            sloshObserverSelectionReasonName(observer_selection.reason),
            observer_selection.odom_state_age_sec,
            observer_selection.imu_state_age_sec);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_slosh_observer = true;
        cycle_audit.status = selection_status;
        publishSloshObserverSelectionDebug(
            observer_selection_now,
            observer_selection,
            solver_consumes_selected_state,
            cycle_audit.timing);
        publishZeroCommand(intervention, &cycle_audit);
        return;
    }
    if (observer_selection.fallback_active) {
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] liquid observer fallback active: nominal=%s effective=%s reason=%s epoch=%llu",
            sloshObserverSourceName(observer_selection.nominal_source),
            sloshObserverSourceName(observer_selection.effective_source),
            sloshObserverSelectionReasonName(observer_selection.reason),
            static_cast<unsigned long long>(observer_selection.selection_epoch));
    }
    if (observer_selection.valid) {
        input.slosh = observer_selection.state;
    } else if (odom_observer_health.snapshot.configured &&
               odom_observer_health.snapshot.valid) {
        // A non-slosh comparator does not consume this field, but keeping its
        // diagnostic state populated preserves useful paired observer evidence.
        input.slosh = odom_observer_health.snapshot.state;
    }

    if (cycle_audit.timing.state_alignment_required) {
        double raw_skew_sec = 0.0;
        if (!stateSkewWithinContract(
                cycle_audit.timing.raw_robot_state_stamp_ns,
                cycle_audit.timing.raw_liquid_state_stamp_ns,
                state_timing_params_.max_raw_skew_sec,
                raw_skew_sec)) {
            control_cycle_engine_.resetSafety();
            cycle_audit.timing.raw_state_skew_sec = raw_skew_sec;
            cycle_audit.timing.state_alignment_status =
                "RAW_STATE_SKEW_CONTRACT_FAILED";
            cycle_audit.status = "STATE_TIME_ALIGNMENT_FAILED_RAW_SKEW";
            diagnostics_.publishStatus(cycle_audit.status);
            publishSloshObserverSelectionDebug(
                observer_selection_now,
                observer_selection,
                solver_consumes_selected_state,
                cycle_audit.timing);
            CommandInterventionDebug intervention;
            intervention.zero_due_to_waiting_for_slosh_observer = true;
            publishZeroCommand(intervention, &cycle_audit);
            return;
        }
        cycle_audit.timing.raw_state_skew_sec = raw_skew_sec;
        bool interpolated = false;
        bool extrapolated = false;
        std::string alignment_status;
        const ros::Time liquid_epoch = rosTimeFromNanoseconds(
            cycle_audit.timing.raw_liquid_state_stamp_ns);
        if (!robotStateAtEpoch(
                liquid_epoch,
                input.robot,
                interpolated,
                extrapolated,
                alignment_status)) {
            control_cycle_engine_.resetSafety();
            cycle_audit.timing.state_alignment_status = alignment_status;
            cycle_audit.status =
                "STATE_TIME_ALIGNMENT_FAILED_" + alignment_status;
            diagnostics_.publishStatus(cycle_audit.status);
            publishSloshObserverSelectionDebug(
                observer_selection_now,
                observer_selection,
                solver_consumes_selected_state,
                cycle_audit.timing);
            CommandInterventionDebug intervention;
            intervention.zero_due_to_waiting_for_tf = true;
            publishZeroCommand(intervention, &cycle_audit);
            return;
        }
        cycle_audit.timing.robot_state_stamp_ns =
            cycle_audit.timing.raw_liquid_state_stamp_ns;
        cycle_audit.timing.liquid_state_stamp_ns =
            cycle_audit.timing.raw_liquid_state_stamp_ns;
        cycle_audit.timing.solver_input_epoch_ns =
            cycle_audit.timing.raw_liquid_state_stamp_ns;
        cycle_audit.timing.aligned_state_skew_sec = 0.0;
        cycle_audit.timing.state_time_aligned = true;
        cycle_audit.timing.robot_state_interpolated = interpolated;
        cycle_audit.timing.robot_state_extrapolated = extrapolated;
        cycle_audit.timing.state_alignment_status = alignment_status;
    } else {
        if (!robotStateFromLatest(input.robot)) {
            control_cycle_engine_.resetSafety();
            diagnostics_.publishStatus("WAITING_FOR_TF_POSE");
            publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoTfPose);
            cycle_audit.status = "WAITING_FOR_TF_POSE";
            cycle_audit.timing.state_alignment_status = "LATEST_TF_UNAVAILABLE";
            publishSloshObserverSelectionDebug(
                observer_selection_now,
                observer_selection,
                solver_consumes_selected_state,
                cycle_audit.timing);
            CommandInterventionDebug intervention;
            intervention.zero_due_to_waiting_for_tf = true;
            publishZeroCommand(intervention, &cycle_audit);
            return;
        }
        cycle_audit.timing.robot_state_stamp_ns =
            cycle_audit.timing.raw_robot_state_stamp_ns;
        cycle_audit.timing.liquid_state_stamp_ns =
            cycle_audit.timing.raw_liquid_state_stamp_ns;
        cycle_audit.timing.solver_input_epoch_ns =
            cycle_audit.timing.raw_robot_state_stamp_ns;
        cycle_audit.timing.aligned_state_skew_sec =
            cycle_audit.timing.liquid_state_stamp_ns > 0
                ? (cycle_audit.timing.robot_state_stamp_ns -
                   cycle_audit.timing.liquid_state_stamp_ns) * 1e-9
                : 0.0;
        cycle_audit.timing.raw_state_skew_sec =
            cycle_audit.timing.raw_liquid_state_stamp_ns > 0
                ? (cycle_audit.timing.raw_robot_state_stamp_ns -
                   cycle_audit.timing.raw_liquid_state_stamp_ns) * 1e-9
                : 0.0;
        cycle_audit.timing.state_time_aligned =
            !solver_consumes_selected_state ||
            std::abs(cycle_audit.timing.aligned_state_skew_sec) <= 1e-6;
        cycle_audit.timing.state_alignment_status =
            solver_consumes_selected_state
                ? "COMMON_EPOCH_DISABLED"
                : "LIQUID_NOT_CONSUMED";
    }
    publishSloshObserverSelectionDebug(
        observer_selection_now,
        observer_selection,
        solver_consumes_selected_state,
        cycle_audit.timing);
    input.cycle_timing = cycle_audit.timing;
    input.dt = dt_;
    input.horizon_steps = horizon_steps_;
    diagnostics_.publishRawState(input.robot, input.slosh, slosh_height_coeff);

    applyRuntimeVRef(input);
    applySloshRiskGovernor(input);
    // effective_config_.v_ref 是本周期 solver 将看到的速度参考：runtime/profile override 优先，
    // 并按 solver v_max 做同口径 clamp。必须在发布前更新，避免 /spmpc/debug/effective_config 滞后一拍。
    const double requested_config_v_ref = input.has_v_ref_current ? input.v_ref_current : variant_.v_ref;
    if (std::isfinite(requested_config_v_ref)) {
        const double v_max = std::max(0.0, effective_config_.v_max);
        effective_config_.v_ref = std::max(0.0, std::min(v_max, requested_config_v_ref));
    }
    diagnostics_.publishEffectiveConfig(effective_config_);

    const ros::Time delay_phase_now = ros::Time::now();
    DelayPhaseStatusCode delay_phase_status = DelayPhaseStatusCode::MonitorOk;
    ExecutionStatePrediction shadow_prediction;
    ExecutionStatePrediction* shadow_prediction_ptr = nullptr;
    if (delayPhasePredictionEnabled() || phaseRejoinNeedsPrediction()) {
        DelayPhaseParams predictor_params = delay_phase_params_;
        if (predictor_params.mode == DelayPhaseMode::Off) {
            // Phase-rejoin shadow prediction must remain observable even when
            // the legacy delay_phase feature itself is disabled.
            predictor_params.mode = DelayPhaseMode::Shadow;
        }
        shadow_prediction = execution_predictor_.predict(
            input.robot, input.slosh, command_history_,
            input.cycle_timing.solver_input_epoch_ns,
            static_cast<StampNs>(delay_phase_now.toNSec()), predictor_params);
        delay_phase_status = shadow_prediction.status_code;
        shadow_prediction_ptr = &shadow_prediction;
    } else {
        shadow_prediction.raw_robot = input.robot;
        shadow_prediction.raw_slosh = input.slosh;
        shadow_prediction.predicted_robot = input.robot;
        shadow_prediction.predicted_slosh = input.slosh;
        shadow_prediction.status_code = DelayPhaseStatusCode::Off;
    }
    diagnostics_.publishPredictedState(shadow_prediction, slosh_height_coeff);

    SolverInput solve_input = input;
    bool robot_delay_compensation_applied = false;
    bool liquid_delay_compensation_applied = false;
    if (delayPhaseClosedLoopEnabled() && shadow_prediction_ptr) {
        const bool have_odom_receive = !last_odom_receive_stamp_.isZero();
        const double odom_age_sec = have_odom_receive ? (delay_phase_now - last_odom_receive_stamp_).toSec() : -1.0;
        const bool odom_fresh = have_odom_receive &&
                                (!std::isfinite(delay_phase_params_.odom_timeout_sec) ||
                                 delay_phase_params_.odom_timeout_sec <= 0.0 ||
                                 odom_age_sec <= delay_phase_params_.odom_timeout_sec);
        const DelayPhaseApplication application = composeDelayPhaseSolverInput(
            input, shadow_prediction, delay_phase_params_.mode, odom_fresh);
        solve_input = application.solver_input;
        robot_delay_compensation_applied = application.robot_applied;
        liquid_delay_compensation_applied = application.liquid_applied;
        if (application.robot_applied && application.liquid_applied) {
            const std::int64_t predicted_epoch_ns =
                shadow_prediction.prediction_epoch_ns;
            cycle_audit.timing.robot_state_stamp_ns = predicted_epoch_ns;
            cycle_audit.timing.liquid_state_stamp_ns = predicted_epoch_ns;
            cycle_audit.timing.solver_input_epoch_ns = predicted_epoch_ns;
            cycle_audit.timing.aligned_state_skew_sec = 0.0;
            cycle_audit.timing.state_time_aligned = true;
            cycle_audit.timing.state_alignment_status =
                "DELAY_PREDICTED_COMMON_EPOCH";
        } else if (cycle_audit.timing.state_alignment_required &&
                   application.anyApplied()) {
            cycle_audit.timing.state_time_aligned = false;
            cycle_audit.timing.state_alignment_status =
                "PARTIAL_DELAY_STATE_APPLICATION_FORBIDDEN";
            cycle_audit.status = "STATE_TIME_ALIGNMENT_FAILED_DELAY_PHASE";
            diagnostics_.publishStatus(cycle_audit.status);
            CommandInterventionDebug intervention;
            intervention.zero_due_to_waiting_for_slosh_observer = true;
            publishZeroCommand(intervention, &cycle_audit);
            return;
        }
        if (!odom_fresh) {
            delay_phase_status = DelayPhaseStatusCode::OdomStale;
        }
    }

    const bool solver_origin_at_execution_front =
        robot_delay_compensation_applied &&
        liquid_delay_compensation_applied;
    solve_input.cycle_timing = cycle_audit.timing;

    if (have_previous_shifted_plan_ &&
        previous_plan_cycle_id_ + 1 == cycle_audit.timing.cycle_id) {
        cycle_audit.previous_shifted_plan_available = true;
        cycle_audit.previous_plan_cycle_id = previous_plan_cycle_id_;
        cycle_audit.previous_shifted_plan_a = previous_shifted_plan_a_;
        cycle_audit.previous_shifted_plan_alpha =
            previous_shifted_plan_alpha_;
    } else if (have_previous_shifted_plan_) {
        // A gate/failure cycle broke the one-step shift.  Comparing against
        // that stale horizon would overstate replanning overwrite.
        have_previous_shifted_plan_ = false;
    }

    cycle_audit.timing.solve_start_stamp_ns = static_cast<std::int64_t>(
        ros::Time::now().toNSec());
    solve_input.cycle_timing = cycle_audit.timing;
    double spin_gate_dt = dt_;
    if (!event.last_real.isZero() && !event.current_real.isZero()) {
        spin_gate_dt = (event.current_real - event.last_real).toSec();
    }
    const double front_sec = std::max(
        delay_phase_params_.linear_delay_sec,
        delay_phase_params_.angular_delay_sec);
    const int front_steps = static_cast<int>(std::ceil(
        std::max(0.0, front_sec) / std::max(1e-9, dt_)));
    ControlCycleRequest engine_request;
    engine_request.cycle_id = cycle_audit.timing.cycle_id;
    engine_request.cycle_start_ns =
        cycle_audit.timing.cycle_start_stamp_ns;
    engine_request.solver_input = solve_input;
    engine_request.prediction_valid = shadow_prediction.valid;
    engine_request.prediction_status = shadow_prediction.status;
    engine_request.execution_front_robot =
        shadow_prediction.predicted_robot;
    engine_request.execution_front_slosh =
        shadow_prediction.predicted_slosh;
    engine_request.solver_origin_at_execution_front =
        solver_origin_at_execution_front;
    engine_request.execution_front_steps = front_steps;
    engine_request.phase_time_sec = delay_phase_now.toSec();
    engine_request.period_sec = spin_gate_dt;
    cycle_audit.solve_attempted = true;
    ControlCycleResult engine_result =
        control_cycle_engine_.step(engine_request);
    cycle_audit.timing.solve_end_stamp_ns = static_cast<std::int64_t>(
        ros::Time::now().toNSec());
    cycle_audit.timing.horizon_available_stamp_ns =
        cycle_audit.timing.solve_end_stamp_ns;
    solve_input = engine_result.solver_input;
    SolverOutput output = engine_result.output;
    output.cycle_timing = cycle_audit.timing;
    diagnostics_.publishSolverInputState(
        solve_input,
        static_cast<std::uint8_t>(observer_selection.effective_source),
        robot_delay_compensation_applied,
        liquid_delay_compensation_applied,
        slosh_height_coeff);
    applyControlCycleTelemetry(engine_result.telemetry, cycle_audit);
    if (cycle_audit.previous_shifted_plan_available) {
        cycle_audit.replanned_minus_shifted_a =
            cycle_audit.solver_u0_a - cycle_audit.previous_shifted_plan_a;
        cycle_audit.replanned_minus_shifted_alpha =
            cycle_audit.solver_u0_alpha -
            cycle_audit.previous_shifted_plan_alpha;
    }
    if (std::isfinite(output.progress_abs_s)) {
        map_vref_last_progress_abs_s_ = output.progress_abs_s;
        have_map_vref_progress_ = true;
    }
    CommandInterventionDebug intervention =
        makeCommandInterventionDebug(engine_result.telemetry);
    const bool terminal_spin_blocked =
        engine_result.telemetry.terminal_spin_blocked;
    const bool tracking_safety_blocked =
        engine_result.telemetry.tracking_safety_blocked;
    diagnostics_.publishStatus(output.status);
    // 诊断统一使用 solver 的实际液体输入：fixed_closed_loop 使用 rollout，
    // fixed_robot_only 则保留当前 observer 测量。
    diagnostics_.publishSloshState(solve_input.slosh);
    // 当前标量模型液面高度 = c_h·‖η‖ (+向心项), 由唯一物理核 SloshDynamics 计算; 单位米(模型 proxy)。
    bool observer_dynamics_configured = false;
    double selected_height_m = 0.0;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        observer_dynamics_configured = slosh_observers_.odomConfigured();
        selected_height_m = slosh_observers_.solverHeight(
            solve_input.slosh, solve_input.robot.omega);
    }
    if (observer_dynamics_configured) {
        diagnostics_.publishSloshHeight(selected_height_m);
    }
    publishDelayPhaseDiagnostics(
        delay_phase_now,
        delay_phase_status,
        shadow_prediction_ptr,
        output.solver_time_ms,
        robot_delay_compensation_applied || liquid_delay_compensation_applied);
    if (phase_rejoin_publish_diagnostics_) {
        PhaseRejoinDebugData phase_debug =
            control_cycle_engine_.makePhaseRejoinDebug(
                &engine_result.phase_preparation,
                engine_result.have_phase_decision
                    ? &engine_result.phase_decision
                    : nullptr);
        if (terminal_spin_blocked || tracking_safety_blocked) {
            phase_debug.status = "SAFETY_OVERRIDE_" + output.status;
        }
        diagnostics_.publishPhaseRejoin(
            phase_debug,
            cycle_audit.timing,
            problem_.referenceFrameId(),
            shadow_prediction_ptr);
    }
    diagnostics_.publishOutput(output, problem_.referenceFrameId());

    if (output.predicted_horizon.valid &&
        output.predicted_horizon.controls.size() > 1) {
        previous_plan_cycle_id_ = cycle_audit.timing.cycle_id;
        previous_shifted_plan_a_ =
            output.predicted_horizon.controls[1].a;
        previous_shifted_plan_alpha_ =
            output.predicted_horizon.controls[1].alpha_or_omega;
        have_previous_shifted_plan_ = true;
    } else {
        have_previous_shifted_plan_ = false;
    }

    if (!output.success) {
        publishZeroCommand(intervention, &cycle_audit);
        return;
    }

    publishCommand(engine_result.decision, intervention, &cycle_audit);
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

void SpmpcLocalPlannerROS::appendOdomStateHistory(
    const nav_msgs::Odometry& odom) {
    if (odom.header.stamp.isZero()) {
        return;
    }
    StampedRobotState sample;
    sample.stamp_ns = static_cast<std::int64_t>(odom.header.stamp.toNSec());
    sample.state = robotStateFromOdom(odom);
    if (!odom_state_history_.empty() &&
        sample.stamp_ns <= odom_state_history_.back().stamp_ns) {
        // processOdomInput only admits a regression for a detected source clock
        // reset.  A new epoch must not interpolate across that reset.
        odom_state_history_.clear();
    }
    odom_state_history_.push_back(sample);
    const std::int64_t history_ns = static_cast<std::int64_t>(
        std::max(0.1, state_timing_params_.odom_history_sec) * 1e9);
    while (odom_state_history_.size() > 1 &&
           sample.stamp_ns - odom_state_history_.front().stamp_ns > history_ns) {
        odom_state_history_.pop_front();
    }
}

bool SpmpcLocalPlannerROS::robotStateAtEpoch(
    const ros::Time& target_stamp,
    RobotState& state,
    bool& interpolated,
    bool& extrapolated,
    std::string& status) {
    interpolated = false;
    extrapolated = false;
    status = "INVALID_TARGET";
    if (target_stamp.isZero()) {
        return false;
    }
    const auto aligned = alignRobotStateToEpoch(
        odom_state_history_,
        static_cast<std::int64_t>(target_stamp.toNSec()),
        state_timing_params_.max_interpolation_gap_sec,
        state_timing_params_.max_robot_extrapolation_sec);
    status = aligned.status;
    if (!aligned.valid) {
        return false;
    }
    state = aligned.state;
    interpolated = aligned.interpolated;
    extrapolated = aligned.extrapolated;

    const std::string reference_frame = problem_.referenceFrameId();
    if (!use_tf_pose_ || reference_frame.empty() ||
        reference_frame == last_odom_.header.frame_id) {
        return true;
    }
    try {
        const auto tf = tf_buffer_.lookupTransform(
            reference_frame,
            robot_base_frame_,
            target_stamp,
            ros::Duration(std::max(0.0, tf_timeout_sec_)));
        state.x = tf.transform.translation.x;
        state.y = tf.transform.translation.y;
        state.yaw = tf2::getYaw(tf.transform.rotation);
        return true;
    } catch (const tf2::TransformException& ex) {
        status = "TF_AT_COMMON_EPOCH_UNAVAILABLE";
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] common-epoch TF unavailable %s <- %s at %.6f: %s",
            reference_frame.c_str(),
            robot_base_frame_.c_str(),
            target_stamp.toSec(),
            ex.what());
        return false;
    }
}

bool SpmpcLocalPlannerROS::processOdomInput(
    const nav_msgs::Odometry& odom,
    const ros::Time& receive_stamp) {
    OdomTimingDebug timing;
    if (!receive_stamp.isZero() && !odom.header.stamp.isZero()) {
        timing.recv_age_ms = 1000.0 * (receive_stamp - odom.header.stamp).toSec();
    }
    timing.have_prev_odom = have_prev_odom_;

    MotionExcitation excitation;
    excitation.source = MotionExcitationSource::Odom;
    excitation.source_stamp_ns = static_cast<std::int64_t>(odom.header.stamp.toNSec());
    excitation.measurement_stamp_ns = excitation.source_stamp_ns;
    excitation.accel_effective_stamp_ns = excitation.source_stamp_ns;
    excitation.gyro_effective_stamp_ns = excitation.source_stamp_ns;
    excitation.alpha_effective_stamp_ns = excitation.source_stamp_ns;
    excitation.receive_stamp_ns = static_cast<std::int64_t>(receive_stamp.toNSec());

    const geometry_msgs::Point& position = odom.pose.pose.position;
    const geometry_msgs::Quaternion& orientation = odom.pose.pose.orientation;
    const double orientation_norm_sq =
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w;
    const double v = odom.twist.twist.linear.x;
    const double omega = odom.twist.twist.angular.z;
    const bool finite_control_state =
        std::isfinite(position.x) && std::isfinite(position.y) &&
        std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
        std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
        std::isfinite(orientation_norm_sq) &&
        orientation_norm_sq > std::numeric_limits<double>::epsilon() &&
        std::isfinite(v) && std::isfinite(omega);
    if (receive_stamp.isZero() || odom.header.stamp.isZero() || !finite_control_state) {
        timing.dt_clamped = true;
        last_odom_timing_ = timing;
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] rejecting odom control input with invalid stamp, pose, or twist");
        if (imu_shadow_publish_diagnostics_) {
            publishOdomSloshObserverDebug(odom, excitation, "ODOM_INVALID_SAMPLE");
        }
        return false;
    }

    if (!have_prev_odom_) {
        prev_odom_ = odom;
        have_prev_odom_ = true;
        last_odom_timing_ = timing;
        if (imu_shadow_publish_diagnostics_) {
            publishOdomSloshObserverDebug(odom, excitation, "ODOM_WAITING_FOR_PREVIOUS");
        }
        return true;
    }

    const double dt_msg = (odom.header.stamp - prev_odom_.header.stamp).toSec();
    const double prev_v = prev_odom_.twist.twist.linear.x;
    const double prev_omega = prev_odom_.twist.twist.angular.z;
    if (!std::isfinite(dt_msg) || dt_msg <= kMinimumOdomObserverDtSec ||
        !std::isfinite(prev_v) || !std::isfinite(prev_omega)) {
        timing.stamp_dt_ms = std::isfinite(dt_msg) ? 1000.0 * dt_msg : 0.0;
        timing.dt_clamped = true;
        timing.have_prev_odom = true;
        last_odom_timing_ = timing;

        const bool clock_reset = std::isfinite(dt_msg) &&
                                 dt_msg < -kOdomClockResetThresholdSec;
        const char* status = "ODOM_INVALID_TIMESTAMP";
        if (clock_reset) {
            status = "ODOM_CLOCK_RESET";
        } else if (dt_msg == 0.0) {
            status = "ODOM_DUPLICATE_TIMESTAMP";
        } else if (std::isfinite(dt_msg) && dt_msg < 0.0) {
            status = "ODOM_OUT_OF_ORDER_DROP";
        } else if (std::isfinite(dt_msg)) {
            status = "ODOM_DT_TOO_SMALL";
        }
        if (clock_reset) {
            // A large source-clock regression starts a clean liquid epoch.  A
            // small out-of-order packet is only dropped and cannot move the
            // derivative baseline backwards.
            {
                std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
                slosh_observers_.resetOdom();
            }
            prev_odom_ = odom;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] rejecting odom slosh update: %s (dt=%.6f s)",
            status,
            dt_msg);
        if (imu_shadow_publish_diagnostics_) {
            publishOdomSloshObserverDebug(odom, excitation, status);
        }
        return clock_reset;
    }

    const double ax = (v - prev_v) / dt_msg;
    const double ay = v * omega;
    const double alpha = (omega - prev_omega) / dt_msg;
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(alpha)) {
        timing.stamp_dt_ms = 1000.0 * dt_msg;
        timing.dt_clamped = true;
        timing.have_prev_odom = true;
        last_odom_timing_ = timing;
        ROS_WARN_THROTTLE(
            1.0,
            "[spmpc_local_planner] rejecting odom control input with non-finite derived excitation");
        if (imu_shadow_publish_diagnostics_) {
            publishOdomSloshObserverDebug(
                odom, excitation, "ODOM_DERIVED_NONFINITE");
        }
        return false;
    }

    timing.stamp_dt_ms = 1000.0 * dt_msg;
    timing.ax = ax;
    timing.ay = ay;
    timing.omega = omega;
    timing.have_prev_odom = true;
    timing.dt_clamped = false;
    last_odom_timing_ = timing;

    excitation.valid = true;
    excitation.ax = ax;
    excitation.ay = ay;
    excitation.omega_z = omega;
    excitation.alpha_z = alpha;
    excitation.sample_dt_sec = dt_msg;
    const std::int64_t previous_stamp_ns =
        static_cast<std::int64_t>(prev_odom_.header.stamp.toNSec());
    const std::int64_t interval_midpoint_ns = previous_stamp_ns +
        (excitation.source_stamp_ns - previous_stamp_ns) / 2;
    excitation.accel_effective_stamp_ns = interval_midpoint_ns;
    excitation.alpha_effective_stamp_ns = interval_midpoint_ns;
    bool observer_updated = false;
    bool odom_observer_configured = false;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        observer_updated = slosh_observers_.stepOdom(excitation);
        odom_observer_configured = slosh_observers_.odomConfigured();
    }
    if (!observer_updated && odom_observer_configured) {
        ROS_WARN_THROTTLE(1.0, "[spmpc_local_planner] odom slosh observer step rejected");
    } else if (!odom_observer_configured) {
        ROS_WARN_THROTTLE(1.0, "[spmpc_local_planner] slosh observer reconfigure failed");
    }
    if (imu_shadow_publish_diagnostics_) {
        publishOdomSloshObserverDebug(
            odom,
            excitation,
            observer_updated ? "ODOM_READY" : "ODOM_OBSERVER_INVALID");
    }
    prev_odom_ = odom;
    return true;
}

void SpmpcLocalPlannerROS::publishOdomSloshObserverDebug(
    const nav_msgs::Odometry& odom,
    const MotionExcitation& excitation,
    const std::string& status) {
    SloshObserverSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        snapshot = slosh_observers_.odom();
    }
    SloshObserverDebug msg;
    msg.header = odom.header;
    msg.header.frame_id = odom.child_frame_id.empty() ? robot_base_frame_ : odom.child_frame_id;
    msg.schema_version = 2;
    msg.source = SloshObserverDebug::SOURCE_ODOM;
    msg.excitation_axes_frame = msg.header.frame_id;
    msg.excitation_reference_point = msg.header.frame_id;
    msg.configured = snapshot.configured;
    msg.valid = excitation.valid && snapshot.valid;
    msg.input_status_code = 0;
    msg.input_status = status;
    msg.reset_epoch = excitation.reset_epoch;
    msg.accepted_sample_count = snapshot.update_count;
    msg.observer_update_count = snapshot.update_count;
    msg.source_stamp = rosTimeFromNanoseconds(excitation.source_stamp_ns);
    msg.measurement_stamp = rosTimeFromNanoseconds(excitation.measurement_stamp_ns);
    msg.accel_effective_stamp = rosTimeFromNanoseconds(excitation.accel_effective_stamp_ns);
    msg.gyro_effective_stamp = rosTimeFromNanoseconds(excitation.gyro_effective_stamp_ns);
    msg.alpha_effective_stamp = rosTimeFromNanoseconds(excitation.alpha_effective_stamp_ns);
    msg.receive_stamp = rosTimeFromNanoseconds(excitation.receive_stamp_ns);
    msg.state_stamp = rosTimeFromNanoseconds(snapshot.state_stamp_ns);
    msg.transport_age_sec = ageSeconds(
        excitation.receive_stamp_ns, excitation.source_stamp_ns);
    msg.measurement_age_sec = ageSeconds(
        excitation.receive_stamp_ns, excitation.measurement_stamp_ns);
    msg.state_age_sec = ageSeconds(excitation.receive_stamp_ns, snapshot.state_stamp_ns);
    msg.sample_dt_sec = excitation.sample_dt_sec;
    msg.ax_mps2 = excitation.ax;
    msg.ay_mps2 = excitation.ay;
    msg.omega_z_radps = excitation.omega_z;
    msg.alpha_z_radps2 = excitation.alpha_z;
    msg.eta_x = snapshot.state.eta_x;
    msg.eta_x_dot = snapshot.state.eta_x_dot;
    msg.eta_y = snapshot.state.eta_y;
    msg.eta_y_dot = snapshot.state.eta_y_dot;
    msg.modal_height_m = snapshot.modal_height_m;
    msg.total_height_m = snapshot.total_height_m;
    diagnostics_.publishOdomSloshObserver(msg);
}

void SpmpcLocalPlannerROS::publishImuSloshObserverDebug(
    const sensor_msgs::Imu& imu,
    const ProcessedImuOutput& output) {
    SloshObserverSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(slosh_observers_mutex_);
        snapshot = slosh_observers_.imu();
    }
    const MotionExcitation& excitation = output.excitation;
    SloshObserverDebug msg;
    msg.header = imu.header;
    msg.header.frame_id = robot_base_frame_;
    msg.schema_version = 2;
    msg.source = SloshObserverDebug::SOURCE_PROCESSED_IMU;
    msg.excitation_axes_frame = robot_base_frame_;
    msg.excitation_reference_point = "liquid_observer_target_icr_proxy";
    msg.configured = snapshot.configured;
    msg.valid = output.excitation.valid && snapshot.valid;
    msg.input_status_code = static_cast<std::uint8_t>(output.status);
    msg.input_status = imuPipelineStatusName(output.status);
    msg.reset_epoch = output.reset_epoch;
    msg.accepted_sample_count = output.accepted_sample_count;
    msg.observer_update_count = snapshot.update_count;
    msg.source_stamp = rosTimeFromNanoseconds(excitation.source_stamp_ns);
    msg.measurement_stamp = rosTimeFromNanoseconds(excitation.measurement_stamp_ns);
    msg.accel_effective_stamp = rosTimeFromNanoseconds(excitation.accel_effective_stamp_ns);
    msg.gyro_effective_stamp = rosTimeFromNanoseconds(excitation.gyro_effective_stamp_ns);
    msg.alpha_effective_stamp = rosTimeFromNanoseconds(excitation.alpha_effective_stamp_ns);
    msg.receive_stamp = rosTimeFromNanoseconds(excitation.receive_stamp_ns);
    msg.state_stamp = rosTimeFromNanoseconds(snapshot.state_stamp_ns);
    // The filtered acceleration/gyro/alpha components have distinct phase
    // delays.  Until explicit component re-alignment is introduced, the
    // observer's nominal combined time is source minus sensor delay; the three
    // component-effective stamps above preserve the exact alternatives.
    msg.header.stamp = msg.measurement_stamp;
    msg.transport_age_sec = output.transport_age_sec;
    msg.measurement_age_sec = ageSeconds(
        excitation.receive_stamp_ns, excitation.measurement_stamp_ns);
    msg.state_age_sec = ageSeconds(excitation.receive_stamp_ns, snapshot.state_stamp_ns);
    msg.sample_dt_sec = excitation.sample_dt_sec;
    msg.ax_mps2 = excitation.ax;
    msg.ay_mps2 = excitation.ay;
    msg.omega_z_radps = excitation.omega_z;
    msg.alpha_z_radps2 = excitation.alpha_z;
    msg.bias_ready = output.bias_ready;
    msg.filter_ready = output.filter_ready;
    msg.bias_sample_count = static_cast<std::uint32_t>(std::min<std::size_t>(
        output.bias_sample_count,
        static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
    msg.bias_x_mps2 = output.bias_mps2[0];
    msg.bias_y_mps2 = output.bias_mps2[1];
    msg.bias_z_mps2 = output.bias_mps2[2];
    msg.accel_filtered_base_x_mps2 = output.accel_filtered_base_mps2[0];
    msg.accel_filtered_base_y_mps2 = output.accel_filtered_base_mps2[1];
    msg.quaternion_norm = output.quaternion_norm;
    msg.eta_x = snapshot.state.eta_x;
    msg.eta_x_dot = snapshot.state.eta_x_dot;
    msg.eta_y = snapshot.state.eta_y;
    msg.eta_y_dot = snapshot.state.eta_y_dot;
    msg.modal_height_m = snapshot.modal_height_m;
    msg.total_height_m = snapshot.total_height_m;
    diagnostics_.publishImuSloshObserver(msg);
}

void SpmpcLocalPlannerROS::publishSloshObserverSelectionDebug(
    const ros::Time& now,
    const SloshObserverSelection& selection,
    bool solver_consumes_selected_state,
    const ControlCycleTimingDebug& cycle_timing) {
    SloshObserverSelectionDebug msg;
    msg.header.stamp = now;
    msg.header.frame_id = robot_base_frame_;
    msg.schema_version = 2;
    msg.cycle_id = cycle_timing.cycle_id;
    msg.cycle_start_stamp =
        rosTimeFromNanoseconds(cycle_timing.cycle_start_stamp_ns);
    msg.raw_robot_state_stamp =
        rosTimeFromNanoseconds(cycle_timing.raw_robot_state_stamp_ns);
    msg.solver_input_epoch =
        rosTimeFromNanoseconds(cycle_timing.solver_input_epoch_ns);
    msg.solver_consumes_selected_state = solver_consumes_selected_state;
    msg.configured = selection.configured;
    msg.valid = selection.valid;
    msg.fallback_active = selection.fallback_active;
    msg.fallback_latched = selection.fallback_latched;
    msg.nominal_ready_seen = selection.nominal_ready_seen;
    msg.nominal_source = static_cast<std::uint8_t>(selection.nominal_source);
    msg.nominal_source_name = sloshObserverSourceName(selection.nominal_source);
    msg.effective_source = static_cast<std::uint8_t>(selection.effective_source);
    msg.effective_source_name = sloshObserverSourceName(selection.effective_source);
    msg.fallback_policy = static_cast<std::uint8_t>(selection.fallback_policy);
    msg.fallback_policy_name =
        sloshObserverFallbackPolicyName(selection.fallback_policy);
    msg.status_code = static_cast<std::uint8_t>(selection.status);
    msg.status = sloshObserverSelectionStatusName(selection.status);
    msg.reason_code = static_cast<std::uint8_t>(selection.reason);
    msg.reason = sloshObserverSelectionReasonName(selection.reason);
    msg.odom_snapshot_valid = selection.odom_snapshot_valid;
    msg.imu_snapshot_valid = selection.imu_snapshot_valid;
    msg.odom_fresh = selection.odom_fresh;
    msg.imu_fresh = selection.imu_fresh;
    msg.imu_pipeline_ready = selection.imu_pipeline_ready;
    msg.selected_state_stamp =
        rosTimeFromNanoseconds(selection.selected_state_stamp_ns);
    msg.odom_state_stamp = rosTimeFromNanoseconds(selection.odom_state_stamp_ns);
    msg.imu_state_stamp = rosTimeFromNanoseconds(selection.imu_state_stamp_ns);
    msg.odom_state_age_sec = selection.odom_state_age_sec;
    msg.imu_state_age_sec = selection.imu_state_age_sec;
    msg.imu_reset_epoch = selection.imu_reset_epoch;
    msg.selection_epoch = selection.selection_epoch;
    diagnostics_.publishSloshObserverSelection(msg);
}

bool SpmpcLocalPlannerROS::updateReferenceSignature(const nav_msgs::Path& path) {
    if (path.poses.empty()) {
        const bool changed = have_reference_signature_;
        have_reference_signature_ = false;
        reference_signature_frame_.clear();
        reference_signature_size_ = 0;
        return changed;
    }

    const auto& first = path.poses.front().pose.position;
    const auto& last = path.poses.back().pose.position;
    const auto size = path.poses.size();
    const auto frame = path.header.frame_id;
    const bool changed = !have_reference_signature_ || reference_signature_frame_ != frame ||
                         reference_signature_size_ != size ||
                         std::hypot(reference_signature_start_x_ - first.x,
                                    reference_signature_start_y_ - first.y) > 1e-3 ||
                         std::hypot(reference_signature_end_x_ - last.x,
                                    reference_signature_end_y_ - last.y) > 1e-3;
    if (changed) {
        have_reference_signature_ = true;
        reference_signature_frame_ = frame;
        reference_signature_size_ = size;
        reference_signature_start_x_ = first.x;
        reference_signature_start_y_ = first.y;
        reference_signature_end_x_ = last.x;
        reference_signature_end_y_ = last.y;
    }
    return changed;
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
    pnh_.param(prefix + "w_alpha", variant_.w_alpha, variant_.w_alpha);
    pnh_.param(prefix + "w_du_a", variant_.w_du_a, variant_.w_du_a);
    pnh_.param(prefix + "w_du_vs", variant_.w_du_vs, variant_.w_du_vs);
    pnh_.param(prefix + "w_slosh", variant_.w_slosh, variant_.w_slosh);
    pnh_.param(prefix + "slosh_cost_horizon_steps",
               variant_.slosh_cost_horizon_steps,
               variant_.slosh_cost_horizon_steps);
    pnh_.param(prefix + "slosh_cost_tail_discount",
               variant_.slosh_cost_tail_discount,
               variant_.slosh_cost_tail_discount);

    if (variant_.w_alpha < 0.0) {
        variant_.w_alpha = variant_.w_smooth;
    }
    if (variant_.w_du_a < 0.0) {
        variant_.w_du_a = variant_.w_smooth;
    }
    if (variant_.w_du_vs < 0.0) {
        variant_.w_du_vs = variant_.w_smooth;
    }
}

SloshModelParams SpmpcLocalPlannerROS::loadSloshParams() const {
    SloshModelParams params;
    pnh_.param("slosh/container_radius", params.container_radius, params.container_radius);
    pnh_.param("slosh/liquid_height", params.liquid_height, params.liquid_height);
    pnh_.param("slosh/liquid_density", params.liquid_density, params.liquid_density);
    pnh_.param("slosh/damping_ratio", params.damping_ratio, params.damping_ratio);
    pnh_.param("slosh/mode_index", params.mode_index, params.mode_index);
    pnh_.param("slosh/slosh_height_ref", params.slosh_height_ref, params.slosh_height_ref);
    pnh_.param("slosh/slosh_height_max", params.slosh_height_max, params.slosh_height_max);
    // 防呆：历史上部分 launch/yaml 误写在 container/slosh_height_max 命名空间。
    // 若 container/slosh_height_max 存在且合法，用它覆盖并打 WARN 提示迁移。
    {
        double container_height_max = -1.0;
        pnh_.param("container/slosh_height_max", container_height_max, container_height_max);
        if (std::isfinite(container_height_max) && container_height_max > 0.0) {
            ROS_WARN_ONCE(
                "[spmpc_local_planner] 检测到 container/slosh_height_max=%.4f m，"
                "正确命名空间为 slosh/slosh_height_max。"
                "本次自动采用 container 值 (%.4f m)，请将配置迁移到 slosh/ 命名空间。",
                container_height_max, container_height_max);
            params.slosh_height_max = container_height_max;
        }
    }
    if (!std::isfinite(params.slosh_height_max) || params.slosh_height_max <= 0.0) {
        ROS_WARN("[spmpc_local_planner] invalid slosh/slosh_height_max=%.6f, fallback to slosh_height_ref=%.6f",
                 params.slosh_height_max,
                 params.slosh_height_ref);
        params.slosh_height_max = std::max(1e-6, params.slosh_height_ref);
    }
    pnh_.param("slosh/slosh_eta_dot_ratio", params.slosh_eta_dot_ratio, params.slosh_eta_dot_ratio);
    pnh_.param("slosh/use_linear_model", params.use_linear_model, params.use_linear_model);
    pnh_.param("slosh/use_parabola_term", params.use_parabola_term, params.use_parabola_term);
    return params;
}

ProcessedImuParams SpmpcLocalPlannerROS::loadProcessedImuParams() const {
    ProcessedImuParams params;
    const std::string prefix = "imu_shadow/";
    pnh_.param(prefix + "gravity_mps2", params.gravity_mps2, params.gravity_mps2);
    pnh_.param(prefix + "sensor_delay_sec", params.sensor_delay_sec, params.sensor_delay_sec);
    pnh_.param(prefix + "accel_cutoff_hz", params.accel_cutoff_hz, params.accel_cutoff_hz);
    pnh_.param(prefix + "gyro_cutoff_hz", params.gyro_cutoff_hz, params.gyro_cutoff_hz);
    pnh_.param(prefix + "accel_phase_delay_sec",
               params.accel_phase_delay_sec,
               params.accel_phase_delay_sec);
    pnh_.param(prefix + "gyro_phase_delay_sec",
               params.gyro_phase_delay_sec,
               params.gyro_phase_delay_sec);
    pnh_.param(prefix + "alpha_phase_delay_sec",
               params.alpha_phase_delay_sec,
               params.alpha_phase_delay_sec);
    pnh_.param(prefix + "gyro_scale", params.gyro_scale, params.gyro_scale);
    pnh_.param(prefix + "gyro_offset_radps",
               params.gyro_offset_radps,
               params.gyro_offset_radps);
    pnh_.param(prefix + "imu_to_base_yaw_rad",
               params.imu_to_base_yaw_rad,
               params.imu_to_base_yaw_rad);
    pnh_.param(prefix + "lever_arm_imu_to_target_x_m",
               params.lever_arm_imu_to_target_x_m,
               params.lever_arm_imu_to_target_x_m);
    pnh_.param(prefix + "lever_arm_imu_to_target_y_m",
               params.lever_arm_imu_to_target_y_m,
               params.lever_arm_imu_to_target_y_m);
    pnh_.param(prefix + "bias_window_start_sec",
               params.bias_window_start_sec,
               params.bias_window_start_sec);
    pnh_.param(prefix + "bias_window_end_sec",
               params.bias_window_end_sec,
               params.bias_window_end_sec);
    pnh_.param(prefix + "bias_min_samples", params.bias_min_samples, params.bias_min_samples);
    pnh_.param(prefix + "bias_max_accel_mad_mps2",
               params.bias_max_accel_mad_mps2,
               params.bias_max_accel_mad_mps2);
    pnh_.param(prefix + "bias_max_gyro_p95_radps",
               params.bias_max_gyro_p95_radps,
               params.bias_max_gyro_p95_radps);
    pnh_.param(prefix + "filter_warmup_sec",
               params.filter_warmup_sec,
               params.filter_warmup_sec);
    pnh_.param(prefix + "max_sample_gap_sec",
               params.max_sample_gap_sec,
               params.max_sample_gap_sec);
    pnh_.param(prefix + "clock_reset_threshold_sec",
               params.clock_reset_threshold_sec,
               params.clock_reset_threshold_sec);
    pnh_.param(prefix + "max_receive_age_sec",
               params.max_receive_age_sec,
               params.max_receive_age_sec);
    pnh_.param(prefix + "max_future_skew_sec",
               params.max_future_skew_sec,
               params.max_future_skew_sec);
    pnh_.param(prefix + "quaternion_norm_min",
               params.quaternion_norm_min,
               params.quaternion_norm_min);
    pnh_.param(prefix + "quaternion_norm_max",
               params.quaternion_norm_max,
               params.quaternion_norm_max);
    return params;
}

SloshRiskGovernorParams SpmpcLocalPlannerROS::loadSloshRiskGovernorParams() const {
    SloshRiskGovernorParams params;
    pnh_.param("slosh_risk_governor/enable", params.enable, params.enable);
    pnh_.param("slosh_risk_governor/require_slosh_variant", params.require_slosh_variant, params.require_slosh_variant);
    pnh_.param("slosh_risk_governor/horizon_steps", params.horizon_steps, params.horizon_steps);
    pnh_.param("slosh_risk_governor/height_limit_m", params.height_limit_m, params.height_limit_m);
    pnh_.param("slosh_risk_governor/risk_threshold", params.risk_threshold, params.risk_threshold);
    pnh_.param("slosh_risk_governor/release_threshold", params.release_threshold, params.release_threshold);
    pnh_.param("slosh_risk_governor/beta_min", params.beta_min, params.beta_min);
    pnh_.param("slosh_risk_governor/beta_grid_count", params.beta_grid_count, params.beta_grid_count);
    pnh_.param("slosh_risk_governor/min_v_ref", params.min_v_ref, params.min_v_ref);
    pnh_.param("slosh_risk_governor/accel_limit", params.accel_limit, params.accel_limit);
    pnh_.param("slosh_risk_governor/omega_decay_tau", params.omega_decay_tau, params.omega_decay_tau);
    pnh_.param("slosh_risk_governor/beta_rate_up_per_sec", params.beta_rate_up_per_sec, params.beta_rate_up_per_sec);
    pnh_.param("slosh_risk_governor/beta_rate_down_per_sec", params.beta_rate_down_per_sec, params.beta_rate_down_per_sec);
    pnh_.param("slosh_risk_governor/include_parabola_height",
               params.include_parabola_height,
               params.include_parabola_height);
    return params;
}

}  // namespace spmpc_local_planner
