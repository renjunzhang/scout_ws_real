#include "spmpc_local_planner/ros/spmpc_local_planner_ros.h"
#include "spmpc_local_planner/solvers/solver_factory.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace spmpc_local_planner {

namespace {

const char* boolText(bool value) {
    return value ? "true" : "false";
}

const char* solverBackendRole(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return "SPMPC mainline continuous MPCC";
    }
    if (backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
        return "RouteB diagnostic/legacy continuous MPCC, not mainline";
    }
    if (backend == kSolverBackendPrimitive) {
        return "fallback/debug rollout sampling + cost ranking, not MPCC/mainline";
    }
    return "unknown";
}

int solverBackendCode(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return 1;
    }
    if (backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
        return 2;
    }
    if (backend == kSolverBackendPrimitive) {
        return 3;
    }
    return 0;
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

void appendPolicyError(std::string& reason, const std::string& message) {
    if (!reason.empty()) {
        reason += "; ";
    }
    reason += message;
}

std::string trimCopy(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }
    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

std::vector<std::string> splitCsvSimple(const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(trimCopy(cell));
    }
    return cells;
}

bool parseDoubleStrict(const std::string& text, double& value) {
    const std::string trimmed = trimCopy(text);
    if (trimmed.empty()) {
        return false;
    }
    char* end = nullptr;
    value = std::strtod(trimmed.c_str(), &end);
    return end != trimmed.c_str() && *end == '\0' && std::isfinite(value);
}

int findColumn(const std::vector<std::string>& header, const std::vector<std::string>& names) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        for (const auto& name : names) {
            if (header[i] == name) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

bool validateBackendPolicy(const SolverParams& params,
                           const VariantConfig& variant,
                           std::string& reason) {
    reason.clear();

    if (params.solver_backend == kSolverBackendContinuousMpccAcados) {
        if (variant.slosh_constraint_enable && !variant.slosh_enable) {
            appendPolicyError(reason,
                              "slosh_constraint_enable requires slosh_enable on continuous_mpcc_acados");
        }
        if (params.corridor_enable) {
            appendPolicyError(reason,
                              "continuous_mpcc_acados does not support corridor_enable until J_corridor is implemented in the OCP");
        }
        if (params.obstacle_enable) {
            appendPolicyError(reason,
                              "continuous_mpcc_acados does not support obstacle_enable until obstacle OCP terms are implemented");
        }
        if (params.homotopy_enable) {
            appendPolicyError(reason,
                              "continuous_mpcc_acados does not support homotopy_enable until multi-candidate/homotopy SPMPC is implemented");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(reason,
                              "continuous_mpcc_acados does not support corridor_hard_bound_enable yet");
        }
    } else if (params.solver_backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
        if (variant.slosh_enable || variant.slosh_constraint_enable) {
            appendPolicyError(reason,
                              "slosh-enabled variants must use continuous_mpcc_acados mainline, not RouteB legacy backend");
        }
        if (params.corridor_enable) {
            appendPolicyError(reason,
                              "RouteB legacy backend does not support corridor_enable until J_corridor is implemented in the OCP");
        }
        if (params.obstacle_enable) {
            appendPolicyError(reason,
                              "RouteB legacy backend does not support obstacle_enable under the SPMPC mainline policy");
        }
        if (params.homotopy_enable) {
            appendPolicyError(reason,
                              "RouteB legacy backend does not support homotopy_enable under the SPMPC mainline policy");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(reason,
                              "RouteB legacy backend does not support corridor_hard_bound_enable");
        }
    } else if (params.solver_backend == kSolverBackendPrimitive) {
        if (variant.slosh_enable || variant.slosh_constraint_enable) {
            appendPolicyError(reason,
                              "primitive is fallback/debug rollout sampling only and cannot run slosh-enabled SPMPC variants");
        }
        if (params.obstacle_enable) {
            appendPolicyError(reason,
                              "primitive cannot run obstacle_enable=true under the SPMPC mainline policy");
        }
        if (params.homotopy_enable) {
            appendPolicyError(reason,
                              "primitive cannot run homotopy_enable=true under the SPMPC mainline policy");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(reason,
                              "primitive cannot run corridor_hard_bound_enable=true under the SPMPC mainline policy");
        }
    } else {
        appendPolicyError(reason, "unknown solver backend");
    }

    return reason.empty();
}

}  // namespace

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
    std::string delay_phase_mode = delayPhaseModeName(delay_phase_params_.mode);
    pnh_.param("delay_phase/mode", delay_phase_mode, delay_phase_mode);
    delay_phase_params_.mode = parseDelayPhaseMode(delay_phase_mode);
    if (!isKnownDelayPhaseMode(delay_phase_mode)) {
        ROS_WARN("[spmpc_local_planner] 未知 delay_phase/mode=\"%s\"，已静默退化为 Off。"
                 "合法值：off / monitor / shadow / fixed_closed_loop（及其别名）。",
                 delay_phase_mode.c_str());
    }
    pnh_.param("delay_phase/publish_diagnostics", delay_phase_params_.publish_diagnostics, delay_phase_params_.publish_diagnostics);
    pnh_.param("delay_phase/history_window_sec", delay_phase_params_.history_window_sec, delay_phase_params_.history_window_sec);
    pnh_.param("delay_phase/cmd_timeout_sec", delay_phase_params_.cmd_timeout_sec, delay_phase_params_.cmd_timeout_sec);
    pnh_.param("delay_phase/odom_timeout_sec", delay_phase_params_.odom_timeout_sec, delay_phase_params_.odom_timeout_sec);
    pnh_.param("delay_phase/linear_delay_sec", delay_phase_params_.linear_delay_sec, delay_phase_params_.linear_delay_sec);
    pnh_.param("delay_phase/angular_delay_sec", delay_phase_params_.angular_delay_sec, delay_phase_params_.angular_delay_sec);
    pnh_.param("delay_phase/max_prediction_sec", delay_phase_params_.max_prediction_sec, delay_phase_params_.max_prediction_sec);
    pnh_.param("delay_phase/max_integration_step_sec", delay_phase_params_.max_integration_step_sec, delay_phase_params_.max_integration_step_sec);
    pnh_.param("delay_phase/min_integration_step_sec", delay_phase_params_.min_integration_step_sec, delay_phase_params_.min_integration_step_sec);
    pnh_.param("delay_phase/require_complete_history", delay_phase_params_.require_complete_history, delay_phase_params_.require_complete_history);
    delay_phase_params_.history_window_sec = std::max(0.1, delay_phase_params_.history_window_sec);
    delay_phase_params_.cmd_timeout_sec = std::max(0.0, delay_phase_params_.cmd_timeout_sec);
    delay_phase_params_.odom_timeout_sec = std::max(0.0, delay_phase_params_.odom_timeout_sec);
    delay_phase_params_.linear_delay_sec = std::max(0.0, delay_phase_params_.linear_delay_sec);
    delay_phase_params_.angular_delay_sec = std::max(0.0, delay_phase_params_.angular_delay_sec);
    delay_phase_params_.max_prediction_sec = std::max(0.0, delay_phase_params_.max_prediction_sec);
    delay_phase_params_.max_integration_step_sec = std::max(1e-4, delay_phase_params_.max_integration_step_sec);
    delay_phase_params_.min_integration_step_sec = std::max(1e-6, delay_phase_params_.min_integration_step_sec);
    if (delay_phase_params_.min_integration_step_sec > delay_phase_params_.max_integration_step_sec) {
        delay_phase_params_.min_integration_step_sec = delay_phase_params_.max_integration_step_sec;
    }
    command_history_.configure(delay_phase_params_.history_window_sec);
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
    pnh_.param("terminal/spin_fail/enable", terminal_spin_fail_enable_, terminal_spin_fail_enable_);
    pnh_.param("terminal/spin_fail/omega_threshold", terminal_spin_fail_omega_threshold_, terminal_spin_fail_omega_threshold_);
    pnh_.param("terminal/spin_fail/max_duration_sec", terminal_spin_fail_max_duration_sec_, terminal_spin_fail_max_duration_sec_);
    terminal_spin_fail_omega_threshold_ = std::max(0.0, terminal_spin_fail_omega_threshold_);
    terminal_spin_fail_max_duration_sec_ = std::max(0.0, terminal_spin_fail_max_duration_sec_);
    pnh_.param("tracking_safety/enable", tracking_safety_enable_, tracking_safety_enable_);
    pnh_.param("tracking_safety/projection/enable", tracking_safety_projection_enable_, tracking_safety_projection_enable_);
    pnh_.param("tracking_safety/projection/max_distance_m", tracking_safety_max_projection_distance_m_, tracking_safety_max_projection_distance_m_);
    pnh_.param("tracking_safety/projection/max_duration_sec", tracking_safety_max_projection_duration_sec_, tracking_safety_max_projection_duration_sec_);
    pnh_.param("tracking_safety/spin_fail/enable", tracking_safety_spin_enable_, tracking_safety_spin_enable_);
    pnh_.param("tracking_safety/spin_fail/omega_threshold", tracking_safety_spin_omega_threshold_, tracking_safety_spin_omega_threshold_);
    pnh_.param("tracking_safety/spin_fail/max_duration_sec", tracking_safety_spin_max_duration_sec_, tracking_safety_spin_max_duration_sec_);
    tracking_safety_max_projection_distance_m_ = std::max(0.0, tracking_safety_max_projection_distance_m_);
    tracking_safety_max_projection_duration_sec_ = std::max(0.0, tracking_safety_max_projection_duration_sec_);
    tracking_safety_spin_omega_threshold_ = std::max(0.0, tracking_safety_spin_omega_threshold_);
    tracking_safety_spin_max_duration_sec_ = std::max(0.0, tracking_safety_spin_max_duration_sec_);
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

    problem_.configure(solver_params, variant_);
    if (!slosh_observer_.configure(solver_params.slosh)) {
        ROS_WARN("[spmpc_local_planner] slosh observer configure failed; slosh diagnostics stay zero");
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

    const double period = 1.0 / std::max(1.0, control_frequency_);
    control_timer_ = nh_.createTimer(ros::Duration(period), &SpmpcLocalPlannerROS::controlTimerCallback, this);

    ROS_INFO("[spmpc_local_planner] initialized variant=%s mode=%s path_topic=%s costmap_topic=%s cmd_topic=%s",
             variant_.name.c_str(), experiment_mode_.c_str(), path_topic_.c_str(), costmap_topic_.c_str(), cmd_topic_.c_str());
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
    std::ifstream in(path);
    if (!in.is_open()) {
        ROS_WARN("[spmpc_local_planner] map_vref profile open failed: %s", path.c_str());
        return false;
    }

    std::vector<MapVRefProfileSample> samples;
    bool header_parsed = false;
    bool have_header = false;
    int s_col = 0;
    int v_col = 1;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        const auto cells = splitCsvSimple(trimmed);
        if (cells.empty()) {
            continue;
        }

        if (!header_parsed) {
            double first = 0.0;
            double second = 0.0;
            if (cells.size() >= 2 && parseDoubleStrict(cells[0], first) && parseDoubleStrict(cells[1], second)) {
                header_parsed = true;
            } else {
                have_header = true;
                header_parsed = true;
                s_col = findColumn(cells, {"s_m", "s", "progress_s_m"});
                v_col = findColumn(cells, {"v_ref_map_mps", "v_ref_current_mps", "v_ref_mps", "v_safe_mps"});
                if (s_col < 0 || v_col < 0) {
                    ROS_WARN("[spmpc_local_planner] map_vref profile header must include s_m and v_ref_map_mps: %s", path.c_str());
                    return false;
                }
                continue;
            }
        }

        if (have_header && (static_cast<int>(cells.size()) <= std::max(s_col, v_col))) {
            ROS_WARN("[spmpc_local_planner] map_vref profile skip short row %d in %s", line_no, path.c_str());
            continue;
        }
        const int s_index = have_header ? s_col : 0;
        const int v_index = have_header ? v_col : 1;
        double s_m = 0.0;
        double v_ref_mps = 0.0;
        if (static_cast<int>(cells.size()) <= std::max(s_index, v_index) ||
            !parseDoubleStrict(cells[s_index], s_m) ||
            !parseDoubleStrict(cells[v_index], v_ref_mps)) {
            ROS_WARN("[spmpc_local_planner] map_vref profile skip invalid row %d in %s", line_no, path.c_str());
            continue;
        }
        if (s_m < 0.0 || v_ref_mps < 0.0) {
            ROS_WARN("[spmpc_local_planner] map_vref profile skip negative row %d in %s", line_no, path.c_str());
            continue;
        }
        samples.push_back({s_m, v_ref_mps});
    }

    if (samples.empty()) {
        ROS_WARN("[spmpc_local_planner] map_vref profile has no valid samples: %s", path.c_str());
        return false;
    }
    std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) {
        return a.s_m < b.s_m;
    });
    std::vector<MapVRefProfileSample> deduped;
    deduped.reserve(samples.size());
    for (const auto& sample : samples) {
        if (!deduped.empty() && std::abs(sample.s_m - deduped.back().s_m) < 1e-9) {
            deduped.back() = sample;
        } else {
            deduped.push_back(sample);
        }
    }

    map_vref_profile_ = deduped;
    map_vref_profile_path_ = path;
    map_vref_profile_loaded_ = true;
    ROS_INFO("[spmpc_local_planner] loaded map_vref profile %s with %zu samples", path.c_str(), map_vref_profile_.size());
    return true;
}

bool SpmpcLocalPlannerROS::ensureMapVRefProfileLoaded(const std::string& path) {
    if (path.empty()) {
        map_vref_profile_loaded_ = false;
        map_vref_profile_path_.clear();
        map_vref_profile_.clear();
        return false;
    }
    if (map_vref_profile_loaded_ && path == map_vref_profile_path_) {
        return true;
    }
    map_vref_profile_loaded_ = false;
    map_vref_profile_.clear();
    map_vref_profile_path_.clear();
    return loadMapVRefProfile(path);
}

bool SpmpcLocalPlannerROS::lookupMapVRef(double s_m, double& v_ref_mps) const {
    if (map_vref_profile_.empty() || !std::isfinite(s_m)) {
        return false;
    }
    if (s_m <= map_vref_profile_.front().s_m) {
        v_ref_mps = map_vref_profile_.front().v_ref_mps;
        return true;
    }
    if (s_m >= map_vref_profile_.back().s_m) {
        v_ref_mps = map_vref_profile_.back().v_ref_mps;
        return true;
    }
    const auto upper = std::lower_bound(
        map_vref_profile_.begin(), map_vref_profile_.end(), s_m,
        [](const MapVRefProfileSample& sample, double value) { return sample.s_m < value; });
    if (upper == map_vref_profile_.begin() || upper == map_vref_profile_.end()) {
        return false;
    }
    const auto lower = upper - 1;
    const double ds = upper->s_m - lower->s_m;
    if (ds <= 1e-9) {
        v_ref_mps = upper->v_ref_mps;
        return true;
    }
    const double ratio = (s_m - lower->s_m) / ds;
    v_ref_mps = lower->v_ref_mps + ratio * (upper->v_ref_mps - lower->v_ref_mps);
    return std::isfinite(v_ref_mps);
}

void SpmpcLocalPlannerROS::applyRuntimeVRef(SolverInput& input) {
    bool runtime_v_ref_enable = false;
    double runtime_v_ref = -1.0;
    pnh_.param("map_vref/runtime_v_ref_enable", runtime_v_ref_enable, runtime_v_ref_enable);
    pnh_.param("map_vref/runtime_v_ref", runtime_v_ref, runtime_v_ref);
    if (runtime_v_ref_enable && std::isfinite(runtime_v_ref) && runtime_v_ref >= 0.0) {
        input.has_v_ref_current = true;
        input.v_ref_current = runtime_v_ref;
        input.v_ref_status = "RUNTIME_OVERRIDE";
        return;
    }

    bool profile_enable = false;
    std::string profile_path;
    double profile_lookahead_s = 0.0;
    pnh_.param("map_vref/profile_enable", profile_enable, profile_enable);
    pnh_.param("map_vref/profile_path", profile_path, profile_path);
    pnh_.param("map_vref/profile_lookahead_s", profile_lookahead_s, profile_lookahead_s);
    if (!profile_enable) {
        input.v_ref_status = "VARIANT_FALLBACK";
        return;
    }
    if (!ensureMapVRefProfileLoaded(profile_path)) {
        input.v_ref_status = profile_path.empty() ? "PROFILE_NOT_CONFIGURED" : "PROFILE_LOAD_FAILED";
        return;
    }

    const double current_s = have_map_vref_progress_ ? map_vref_last_progress_abs_s_ : 0.0;
    const double lookup_s = current_s + std::max(0.0, profile_lookahead_s);
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
    return delay_phase_params_.mode == DelayPhaseMode::Shadow ||
           delay_phase_params_.mode == DelayPhaseMode::FixedClosedLoop;
}

bool SpmpcLocalPlannerROS::delayPhaseClosedLoopEnabled() const {
    return delay_phase_params_.mode == DelayPhaseMode::FixedClosedLoop;
}

void SpmpcLocalPlannerROS::recordPublishedCommand(
    const geometry_msgs::Twist& cmd,
    const ros::Time& stamp,
    const CommandPublishMeta& meta) {
    last_published_cmd_ = cmd;
    last_cmd_stamp_ = stamp;
    have_last_published_cmd_ = true;
    command_history_.push({stamp, cmd, meta});
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
    const double cmd_age_sec = has_any_history ? (now - command_history_.latestStamp()).toSec() : -1.0;
    const bool have_odom_receive = !last_odom_receive_stamp_.isZero();
    const double odom_age_sec = have_odom_receive ? (now - last_odom_receive_stamp_).toSec() : -1.0;
    const auto status_requires_freshness = [](DelayPhaseStatusCode status) {
        return status == DelayPhaseStatusCode::MonitorOk ||
               status == DelayPhaseStatusCode::ShadowOk ||
               status == DelayPhaseStatusCode::FixedClosedLoopOk ||
               status == DelayPhaseStatusCode::PartialHistory;
    };

    if ((effective_status == DelayPhaseStatusCode::MonitorOk ||
         effective_status == DelayPhaseStatusCode::ShadowOk ||
         effective_status == DelayPhaseStatusCode::FixedClosedLoopOk) &&
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

void SpmpcLocalPlannerROS::publishZeroCommand(const CommandInterventionDebug& intervention) {
    CommandInterventionDebug debug = intervention;
    debug.publish_cmd_vel = publish_cmd_vel_;
    debug.published_cmd_v = 0.0;
    debug.published_cmd_omega = 0.0;
    diagnostics_.publishCommandIntervention(debug);
    if (!publish_cmd_vel_) {
        return;
    }
    geometry_msgs::Twist cmd;
    const auto stamp = ros::Time::now();
    CommandPublishMeta meta;
    meta.is_zero_cmd = true;
    recordPublishedCommand(cmd, stamp, meta);
    cmd_pub_.publish(cmd);
}

geometry_msgs::Twist SpmpcLocalPlannerROS::applySharedCommandLimits(
    const geometry_msgs::Twist& desired,
    const ros::Time& stamp,
    geometry_msgs::Twist& previous,
    double& dt,
    bool& linear_limited,
    bool& angular_rate_limited,
    bool& angular_accel_limited) {
    geometry_msgs::Twist limited = desired;
    previous = last_published_cmd_;
    linear_limited = false;
    angular_rate_limited = false;
    angular_accel_limited = false;

    const double nominal_dt = 1.0 / std::max(1.0, control_frequency_);
    dt = nominal_dt;
    if (have_last_published_cmd_ && !last_cmd_stamp_.isZero() && !stamp.isZero()) {
        dt = (stamp - last_cmd_stamp_).toSec();
    }
    if (!std::isfinite(dt) || dt <= 1e-6) {
        dt = nominal_dt;
    }
    const double linear_dt = std::min(dt, shared_cmd_linear_accel_max_dt_);
    const double angular_dt = std::min(dt, shared_cmd_angular_accel_max_dt_);

    if (shared_cmd_linear_accel_limit_enable_ && shared_cmd_linear_accel_max_ > 0.0) {
        const double max_step = shared_cmd_linear_accel_max_ * linear_dt;
        const double dv = desired.linear.x - previous.linear.x;
        limited.linear.x = previous.linear.x + std::max(-max_step, std::min(max_step, dv));
        linear_limited = std::abs(limited.linear.x - desired.linear.x) > 1e-6;
    }

    if (shared_cmd_angular_limit_enable_) {
        if (shared_cmd_angular_rate_max_ > 0.0) {
            const double before_rate = limited.angular.z;
            limited.angular.z = std::max(-shared_cmd_angular_rate_max_,
                                         std::min(shared_cmd_angular_rate_max_, limited.angular.z));
            angular_rate_limited = std::abs(limited.angular.z - before_rate) > 1e-6;
        }
        if (shared_cmd_angular_accel_max_ > 0.0) {
            const double max_step = shared_cmd_angular_accel_max_ * angular_dt;
            const double dw = limited.angular.z - previous.angular.z;
            const double before_accel = limited.angular.z;
            limited.angular.z = previous.angular.z + std::max(-max_step, std::min(max_step, dw));
            angular_accel_limited = std::abs(limited.angular.z - before_accel) > 1e-6;
        }
    }

    return limited;
}

void SpmpcLocalPlannerROS::publishCommand(const geometry_msgs::Twist& desired,
                                           const CommandInterventionDebug& intervention) {
    if (!publish_cmd_vel_) {
        CommandInterventionDebug debug = intervention;
        debug.publish_cmd_vel = false;
        diagnostics_.publishCommandIntervention(debug);
        return;
    }
    const auto stamp = ros::Time::now();
    geometry_msgs::Twist previous;
    double dt = 0.0;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;
    const auto cmd = applySharedCommandLimits(
        desired, stamp, previous, dt, linear_limited, angular_rate_limited, angular_accel_limited);
    CommandPublishMeta meta;
    meta.is_zero_cmd = std::abs(cmd.linear.x) <= 1e-9 && std::abs(cmd.angular.z) <= 1e-9;
    meta.linear_limited = linear_limited;
    meta.angular_rate_limited = angular_rate_limited;
    meta.angular_accel_limited = angular_accel_limited;
    recordPublishedCommand(cmd, stamp, meta);
    diagnostics_.publishCommandOutput(
        desired, cmd, previous, dt, linear_limited, angular_rate_limited, angular_accel_limited);
    CommandInterventionDebug debug = intervention;
    debug.published_cmd_v = cmd.linear.x;
    debug.published_cmd_omega = cmd.angular.z;
    debug.linear_limited = linear_limited;
    debug.angular_rate_limited = angular_rate_limited;
    debug.angular_accel_limited = angular_accel_limited;
    debug.publish_cmd_vel = true;
    diagnostics_.publishCommandIntervention(debug);
    cmd_pub_.publish(cmd);
}

void SpmpcLocalPlannerROS::resetTerminalSpinFailGate() {
    terminal_spin_fail_duration_sec_ = 0.0;
    terminal_spin_fail_latched_ = false;
}

void SpmpcLocalPlannerROS::resetTrackingSafetyGate() {
    tracking_safety_projection_duration_sec_ = 0.0;
    tracking_safety_projection_latched_ = false;
    tracking_safety_spin_duration_sec_ = 0.0;
    tracking_safety_spin_latched_ = false;
}

bool SpmpcLocalPlannerROS::updateTerminalSpinFailGate(const SolverInput& input, const SolverOutput& output, double period_sec) {
    if (!terminal_spin_fail_enable_) {
        resetTerminalSpinFailGate();
        return false;
    }
    const auto& terminal = output.terminal_diagnostics;
    if (!output.success || !terminal.terminal_phase || terminal.reached) {
        resetTerminalSpinFailGate();
        return false;
    }

    const bool spinning = std::abs(input.robot.omega) > terminal_spin_fail_omega_threshold_ ||
                          std::abs(output.cmd_omega) > terminal_spin_fail_omega_threshold_;
    if (!spinning) {
        resetTerminalSpinFailGate();
        return false;
    }

    if (!std::isfinite(period_sec) || period_sec <= 1e-6) {
        period_sec = dt_;
    }
    terminal_spin_fail_duration_sec_ += std::max(0.0, period_sec);
    if (terminal_spin_fail_latched_ || terminal_spin_fail_duration_sec_ >= terminal_spin_fail_max_duration_sec_) {
        terminal_spin_fail_latched_ = true;
        return true;
    }
    return false;
}

bool SpmpcLocalPlannerROS::updateTrackingSafetyGate(const SolverInput& input,
                                                    const SolverOutput& output,
                                                    double period_sec,
                                                    std::string& failure_status) {
    failure_status.clear();
    if (!tracking_safety_enable_) {
        resetTrackingSafetyGate();
        return false;
    }
    if (!std::isfinite(period_sec) || period_sec <= 1e-6) {
        period_sec = dt_;
    }
    period_sec = std::max(0.0, period_sec);

    const auto& terminal = output.terminal_diagnostics;
    if (terminal.reached || output.status == "GOAL_REACHED") {
        resetTrackingSafetyGate();
        return false;
    }
    if (tracking_safety_projection_latched_) {
        failure_status = "TRACKING_UNSAFE_PROJECTION";
        return true;
    }
    if (tracking_safety_spin_latched_) {
        failure_status = "TRACKING_SPIN_FAIL";
        return true;
    }
    if (!output.success) {
        return false;
    }

    const auto& projector = output.projector_debug;
    const double projection_distance = projector.guarded_valid ? projector.guarded_distance : projector.raw_distance;
    const bool projection_valid = projector.guarded_valid || projector.raw_valid;
    const bool projection_unsafe = tracking_safety_projection_enable_ && projection_valid &&
                                   tracking_safety_max_projection_distance_m_ > 0.0 &&
                                   projection_distance > tracking_safety_max_projection_distance_m_;
    if (projection_unsafe) {
        tracking_safety_projection_duration_sec_ += period_sec;
    } else {
        tracking_safety_projection_duration_sec_ = 0.0;
    }

    const bool tracking_phase = !terminal.terminal_phase;
    const bool spinning = tracking_safety_spin_enable_ && tracking_phase &&
                          (std::abs(input.robot.omega) > tracking_safety_spin_omega_threshold_ ||
                           std::abs(output.cmd_omega) > tracking_safety_spin_omega_threshold_);
    if (spinning) {
        tracking_safety_spin_duration_sec_ += period_sec;
    } else {
        tracking_safety_spin_duration_sec_ = 0.0;
    }

    if (tracking_safety_projection_enable_ && tracking_safety_max_projection_duration_sec_ > 0.0 &&
        tracking_safety_projection_duration_sec_ >= tracking_safety_max_projection_duration_sec_) {
        tracking_safety_projection_latched_ = true;
        failure_status = "TRACKING_UNSAFE_PROJECTION";
        return true;
    }
    if (tracking_safety_spin_enable_ && tracking_safety_spin_max_duration_sec_ > 0.0 &&
        tracking_safety_spin_duration_sec_ >= tracking_safety_spin_max_duration_sec_) {
        tracking_safety_spin_latched_ = true;
        failure_status = "TRACKING_SPIN_FAIL";
        return true;
    }
    return false;
}

void SpmpcLocalPlannerROS::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    last_odom_receive_stamp_ = ros::Time::now();
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
        if (updateReferenceSignature(transformed_path)) {
            resetTerminalSpinFailGate();
            resetTrackingSafetyGate();
            resetMapVRefProgress();
            slosh_risk_governor_.reset();
        }
        problem_.setReferencePath(referencePathFromMsg(transformed_path));
        return;
    }

    const auto reference = referencePathFromMsg(*msg);
    if (updateReferenceSignature(*msg)) {
        resetTerminalSpinFailGate();
        resetTrackingSafetyGate();
        resetMapVRefProgress();
        slosh_risk_governor_.reset();
    }
    problem_.setReferencePath(reference);
}

void SpmpcLocalPlannerROS::costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    problem_.setCostmap(costmapFromMsg(*msg));
}

void SpmpcLocalPlannerROS::controlTimerCallback(const ros::TimerEvent& event) {
    // publishVariant 内容（variant code、experiment_mode）在运行期不变，已在初始化时发布一次（latched）。
    // publishEffectiveConfig 等 applyRuntimeVRef() 计算完本周期 v_ref 后再发，避免动态 v_ref 滞后一拍。

    if (!have_odom_) {
        resetTerminalSpinFailGate();
        resetTrackingSafetyGate();
        diagnostics_.publishStatus("WAITING_FOR_ODOM");
        publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoOdom);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_odom = true;
        publishZeroCommand(intervention);
        return;
    }
    if (!problem_.hasReferencePath()) {
        resetTerminalSpinFailGate();
        resetTrackingSafetyGate();
        diagnostics_.publishStatus("WAITING_FOR_REFERENCE_PATH");
        publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoReference);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_reference = true;
        publishZeroCommand(intervention);
        return;
    }

    SolverInput input;
    if (!robotStateFromLatest(input.robot)) {
        resetTerminalSpinFailGate();
        resetTrackingSafetyGate();
        diagnostics_.publishStatus("WAITING_FOR_TF_POSE");
        publishDelayPhaseEarlyStatus(DelayPhaseStatusCode::NoTfPose);
        CommandInterventionDebug intervention;
        intervention.zero_due_to_waiting_for_tf = true;
        publishZeroCommand(intervention);
        return;
    }
    input.slosh = current_slosh_;
    input.dt = dt_;
    input.horizon_steps = horizon_steps_;
    const double slosh_height_coeff = slosh_observer_.configured() ? slosh_observer_.heightCoeff() : 0.0;
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
    if (delayPhasePredictionEnabled()) {
        shadow_prediction = execution_predictor_.predict(
            input.robot, input.slosh, command_history_, delay_phase_now, delay_phase_params_);
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
    bool delay_compensation_applied = false;
    if (delayPhaseClosedLoopEnabled() && shadow_prediction_ptr) {
        const bool have_odom_receive = !last_odom_receive_stamp_.isZero();
        const double odom_age_sec = have_odom_receive ? (delay_phase_now - last_odom_receive_stamp_).toSec() : -1.0;
        const bool odom_fresh = have_odom_receive &&
                                (!std::isfinite(delay_phase_params_.odom_timeout_sec) ||
                                 delay_phase_params_.odom_timeout_sec <= 0.0 ||
                                 odom_age_sec <= delay_phase_params_.odom_timeout_sec);
        if (shadow_prediction.valid && shadow_prediction.history_complete &&
            shadow_prediction.status_code == DelayPhaseStatusCode::FixedClosedLoopOk && odom_fresh) {
            solve_input.robot = shadow_prediction.predicted_robot;
            solve_input.slosh = shadow_prediction.predicted_slosh;
            delay_compensation_applied = true;
        } else if (!odom_fresh) {
            delay_phase_status = DelayPhaseStatusCode::OdomStale;
        }
    }

    diagnostics_.publishSolverInputState(solve_input, delay_compensation_applied, slosh_height_coeff);

    SolverOutput output;
    problem_.solve(solve_input, output);
    if (std::isfinite(output.progress_abs_s)) {
        map_vref_last_progress_abs_s_ = output.progress_abs_s;
        have_map_vref_progress_ = true;
    }
    double spin_gate_dt = dt_;
    if (!event.last_real.isZero() && !event.current_real.isZero()) {
        spin_gate_dt = (event.current_real - event.last_real).toSec();
    }
    CommandInterventionDebug intervention;
    intervention.solver_cmd_v = output.cmd_v;
    intervention.solver_cmd_omega = output.cmd_omega;
    const bool solver_success = output.success;
    const bool terminal_spin_blocked = updateTerminalSpinFailGate(input, output, spin_gate_dt);
    if (terminal_spin_blocked) {
        output.success = false;
        output.status = "TERMINAL_SPIN_FAIL";
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
    }
    std::string tracking_safety_status;
    const bool tracking_safety_blocked = updateTrackingSafetyGate(input, output, spin_gate_dt, tracking_safety_status);
    if (tracking_safety_blocked) {
        output.success = false;
        output.status = tracking_safety_status;
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
    }
    intervention.post_gate_cmd_v = output.cmd_v;
    intervention.post_gate_cmd_omega = output.cmd_omega;
    intervention.output_success = output.success;
    intervention.zero_due_to_solver_failure = !solver_success;
    intervention.zero_due_to_terminal_spin_fail = terminal_spin_blocked;
    intervention.zero_due_to_tracking_safety = tracking_safety_blocked;
    diagnostics_.publishStatus(output.status);
    // 诊断用 solve_input.slosh：fixed_closed_loop 补偿生效时 solve_input.slosh 是
    // shadow_prediction.predicted_slosh；input.slosh 仍是原始测量值。
    // 统一用 solve_input 确保 bag 记录的状态与 solver 实际输入一致。
    diagnostics_.publishSloshState(solve_input.slosh);
    // 当前标量模型液面高度 = c_h·‖η‖ (+向心项), 由唯一物理核 SloshDynamics 计算; 单位米(模型 proxy)。
    if (slosh_observer_.configured()) {
        const double omega_meas = last_odom_.twist.twist.angular.z;
        diagnostics_.publishSloshHeight(slosh_observer_.height(solve_input.slosh, omega_meas));
    }
    publishDelayPhaseDiagnostics(
        delay_phase_now, delay_phase_status, shadow_prediction_ptr, output.solver_time_ms, delay_compensation_applied);
    diagnostics_.publishOutput(output, problem_.referenceFrameId());

    if (!output.success) {
        publishZeroCommand(intervention);
        return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = output.cmd_v;
    cmd.angular.z = output.cmd_omega;
    publishCommand(cmd, intervention);
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
    OdomTimingDebug timing;
    const ros::Time receive_stamp = last_odom_receive_stamp_.isZero() ? ros::Time::now() : last_odom_receive_stamp_;
    if (!receive_stamp.isZero() && !odom.header.stamp.isZero()) {
        timing.recv_age_ms = 1000.0 * (receive_stamp - odom.header.stamp).toSec();
    }
    timing.have_prev_odom = have_prev_odom_;

    if (!have_prev_odom_) {
        prev_odom_ = odom;
        have_prev_odom_ = true;
        last_odom_timing_ = timing;
        return;
    }

    const double dt_msg = (odom.header.stamp - prev_odom_.header.stamp).toSec();
    const bool dt_clamped = dt_msg <= 1e-4 || !std::isfinite(dt_msg);
    const double dt_safe = !dt_clamped ? dt_msg : dt_;
    const double v = odom.twist.twist.linear.x;
    const double prev_v = prev_odom_.twist.twist.linear.x;
    const double omega = odom.twist.twist.angular.z;
    const double ax = (v - prev_v) / std::max(1e-3, dt_safe);
    const double ay = v * omega;

    timing.stamp_dt_ms = 1000.0 * dt_safe;
    timing.ax = ax;
    timing.ay = ay;
    timing.omega = omega;
    timing.have_prev_odom = true;
    timing.dt_clamped = dt_clamped;
    last_odom_timing_ = timing;

    if (!slosh_observer_.configured()) {
        prev_odom_ = odom;
        return;
    }

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
