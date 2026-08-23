#include "spmpc_local_planner/ros/ros_config_loader.h"

#include "spmpc_local_planner/solver/api/backend.h"

#include <cmath>

namespace spmpc_local_planner {
namespace {

template <typename Value>
void loadParam(const ros::NodeHandle& node,
               const std::string& key,
               Value& value) {
    node.param(key, value, value);
}

void appendReport(const ValidationReport& source,
                  ValidationReport& destination) {
    for (const auto& issue : source.issues()) {
        if (issue.severity == ValidationSeverity::Fatal) {
            destination.fatal(issue.key, issue.message);
        } else {
            destination.warning(issue.key, issue.message);
        }
    }
}

void loadSloshModel(const ros::NodeHandle& node,
                    AppConfig& config,
                    ValidationReport& report) {
    auto& params = config.solver.slosh;
    loadParam(node, "slosh/container_radius", params.container_radius);
    loadParam(node, "slosh/liquid_height", params.liquid_height);
    loadParam(node, "slosh/liquid_density", params.liquid_density);
    loadParam(node, "slosh/damping_ratio", params.damping_ratio);
    loadParam(node, "slosh/mode_index", params.mode_index);
    loadParam(node, "slosh/slosh_height_ref", params.slosh_height_ref);
    loadParam(node, "slosh/slosh_height_max", params.slosh_height_max);

    double legacy_height_max = -1.0;
    loadParam(node, "container/slosh_height_max", legacy_height_max);
    if (std::isfinite(legacy_height_max) && legacy_height_max > 0.0) {
        params.slosh_height_max = legacy_height_max;
        config.compatibility.legacy_container_height_used = true;
        report.warning(
            "container/slosh_height_max",
            "legacy key overrides slosh/slosh_height_max; migrate it to the slosh namespace");
    }
    loadParam(node, "slosh/slosh_eta_dot_ratio", params.slosh_eta_dot_ratio);
    loadParam(node, "slosh/use_linear_model", params.use_linear_model);
    loadParam(node, "slosh/use_parabola_term", params.use_parabola_term);
}

void loadProcessedImu(const ros::NodeHandle& node, AppConfig& config) {
    auto& params = config.imu_shadow.processed;
    const std::string prefix = "imu_shadow/";
    loadParam(node, prefix + "gravity_mps2", params.gravity_mps2);
    loadParam(node, prefix + "sensor_delay_sec", params.sensor_delay_sec);
    loadParam(node, prefix + "accel_cutoff_hz", params.accel_cutoff_hz);
    loadParam(node, prefix + "gyro_cutoff_hz", params.gyro_cutoff_hz);
    loadParam(node, prefix + "accel_phase_delay_sec",
              params.accel_phase_delay_sec);
    loadParam(node, prefix + "gyro_phase_delay_sec",
              params.gyro_phase_delay_sec);
    loadParam(node, prefix + "alpha_phase_delay_sec",
              params.alpha_phase_delay_sec);
    loadParam(node, prefix + "gyro_scale", params.gyro_scale);
    loadParam(node, prefix + "gyro_offset_radps", params.gyro_offset_radps);
    loadParam(node, prefix + "imu_to_base_yaw_rad",
              params.imu_to_base_yaw_rad);
    loadParam(node, prefix + "lever_arm_imu_to_target_x_m",
              params.lever_arm_imu_to_target_x_m);
    loadParam(node, prefix + "lever_arm_imu_to_target_y_m",
              params.lever_arm_imu_to_target_y_m);
    loadParam(node, prefix + "bias_window_start_sec",
              params.bias_window_start_sec);
    loadParam(node, prefix + "bias_window_end_sec",
              params.bias_window_end_sec);
    loadParam(node, prefix + "bias_min_samples", params.bias_min_samples);
    loadParam(node, prefix + "bias_max_accel_mad_mps2",
              params.bias_max_accel_mad_mps2);
    loadParam(node, prefix + "bias_max_gyro_p95_radps",
              params.bias_max_gyro_p95_radps);
    loadParam(node, prefix + "filter_warmup_sec", params.filter_warmup_sec);
    loadParam(node, prefix + "max_sample_gap_sec", params.max_sample_gap_sec);
    loadParam(node, prefix + "clock_reset_threshold_sec",
              params.clock_reset_threshold_sec);
    loadParam(node, prefix + "max_receive_age_sec",
              params.max_receive_age_sec);
    loadParam(node, prefix + "max_future_skew_sec",
              params.max_future_skew_sec);
    loadParam(node, prefix + "quaternion_norm_min",
              params.quaternion_norm_min);
    loadParam(node, prefix + "quaternion_norm_max",
              params.quaternion_norm_max);
}

void loadRiskGovernor(const ros::NodeHandle& node, AppConfig& config) {
    auto& params = config.slosh_risk_governor;
    const std::string prefix = "slosh_risk_governor/";
    loadParam(node, prefix + "enable", params.enable);
    loadParam(node, prefix + "require_slosh_variant",
              params.require_slosh_variant);
    loadParam(node, prefix + "horizon_steps", params.horizon_steps);
    loadParam(node, prefix + "height_limit_m", params.height_limit_m);
    loadParam(node, prefix + "risk_threshold", params.risk_threshold);
    loadParam(node, prefix + "release_threshold", params.release_threshold);
    loadParam(node, prefix + "beta_min", params.beta_min);
    loadParam(node, prefix + "beta_grid_count", params.beta_grid_count);
    loadParam(node, prefix + "min_v_ref", params.min_v_ref);
    loadParam(node, prefix + "accel_limit", params.accel_limit);
    loadParam(node, prefix + "omega_decay_tau", params.omega_decay_tau);
    loadParam(node, prefix + "beta_rate_up_per_sec",
              params.beta_rate_up_per_sec);
    loadParam(node, prefix + "beta_rate_down_per_sec",
              params.beta_rate_down_per_sec);
    loadParam(node, prefix + "include_parabola_height",
              params.include_parabola_height);
}

void loadVariant(const ros::NodeHandle& node,
                 AppConfig& config,
                 ValidationReport& report) {
    loadParam(node, "planner_variant", config.requested_variant);
    config.variant = makeVariantConfig(config.requested_variant);
    if (config.requested_variant != "B0" && config.variant.name == "B0") {
        report.warning(
            "planner_variant",
            "unknown variant '" + config.requested_variant +
                "' normalized to B0");
    }

    auto& variant = config.variant;
    const std::string prefix = "variants/" + variant.name + "/";
    loadParam(node, prefix + "slosh_enable", variant.slosh_enable);
    loadParam(node, prefix + "smooth_priority_enable",
              variant.smooth_priority_enable);
    loadParam(node, prefix + "slosh_constraint_enable",
              variant.slosh_constraint_enable);
    loadParam(node, prefix + "primitive_mode", variant.primitive_mode);
    loadParam(node, prefix + "w_contour", variant.w_contour);
    loadParam(node, prefix + "w_lag", variant.w_lag);
    loadParam(node, prefix + "w_progress", variant.w_progress);
    loadParam(node, prefix + "w_heading", variant.w_heading);
    loadParam(node, prefix + "w_progress_coupling",
              variant.w_progress_coupling);
    loadParam(node, prefix + "w_yaw_rate_tracking",
              variant.w_yaw_rate_tracking);
    loadParam(node, prefix + "heading_feedback_gain",
              variant.heading_feedback_gain);
    loadParam(node, prefix + "w_v", variant.w_v);
    loadParam(node, prefix + "w_vs", variant.w_vs);
    loadParam(node, prefix + "v_ref", variant.v_ref);
    loadParam(node, prefix + "w_control", variant.w_control);
    loadParam(node, prefix + "w_accel", variant.w_accel);
    loadParam(node, prefix + "w_smooth", variant.w_smooth);
    loadParam(node, prefix + "w_alpha", variant.w_alpha);
    loadParam(node, prefix + "w_du_a", variant.w_du_a);
    loadParam(node, prefix + "w_du_vs", variant.w_du_vs);
    loadParam(node, prefix + "w_slosh", variant.w_slosh);
    loadParam(node, prefix + "slosh_cost_horizon_steps",
              variant.slosh_cost_horizon_steps);
    loadParam(node, prefix + "slosh_cost_tail_discount",
              variant.slosh_cost_tail_discount);
    config.compatibility.variant_weight_table_present =
        node.hasParam(prefix + "w_contour");
    if (!config.compatibility.variant_weight_table_present) {
        report.warning(
            prefix + "*",
            "variant table was not loaded; built-in defaults remain active");
    }
}

}  // namespace

AppConfig RosConfigLoader::load(const ros::NodeHandle& private_node,
                                ValidationReport& report) {
    AppConfig config;

    loadVariant(private_node, config, report);
    auto& interface = config.ros_interface;
    loadParam(private_node, "experiment_mode", interface.experiment_mode);
    loadParam(private_node, "topics/odom", interface.odom_topic);
    loadParam(private_node, "topics/imu", interface.imu_topic);
    loadParam(private_node, "topics/reference_path",
              interface.reference_path_topic);
    loadParam(private_node, "topics/costmap", interface.costmap_topic);
    loadParam(private_node, "topics/cmd_vel", interface.cmd_vel_topic);
    loadParam(private_node, "frames/robot_base", interface.robot_base_frame);
    loadParam(private_node, "frames/reference_target",
              interface.reference_target_frame);
    loadParam(private_node, "frames/use_tf_pose", interface.use_tf_pose);
    loadParam(private_node, "frames/tf_timeout_sec", interface.tf_timeout_sec);
    loadParam(private_node, "publish_cmd_vel", interface.publish_cmd_vel);

    auto& imu = config.imu_shadow;
    loadParam(private_node, "imu_shadow/enable", imu.enable);
    loadParam(private_node, "imu_shadow/publish_diagnostics",
              imu.publish_diagnostics);
    loadParam(private_node, "imu_shadow/expected_frame", imu.expected_frame);
    loadParam(private_node, "imu_shadow/subscriber_queue_size",
              imu.subscriber_queue_size);
    loadParam(private_node, "imu_shadow/observer_dt_sec",
              imu.observer_dt_sec);
    loadProcessedImu(private_node, config);

    std::string observer_source = "odom";
    std::string observer_fallback = "odom";
    loadParam(private_node, "slosh_observer/source", observer_source);
    loadParam(private_node, "slosh_observer/fallback_policy",
              observer_fallback);
    loadParam(private_node, "slosh_observer/latch_fallback",
              config.slosh_observer.latch_fallback);
    loadParam(private_node, "slosh_observer/max_imu_state_age_sec",
              config.slosh_observer.max_imu_state_age_sec);
    loadParam(private_node, "slosh_observer/max_odom_state_age_sec",
              config.slosh_observer.max_odom_state_age_sec);
    loadParam(private_node, "slosh_observer/max_future_skew_sec",
              config.slosh_observer.max_future_skew_sec);
    if (!parseSloshObserverSource(
            observer_source, config.slosh_observer.nominal_source)) {
        report.fatal(
            "slosh_observer/source",
            "unknown value '" + observer_source +
                "'; expected odom|processed_imu");
    }
    if (!parseSloshObserverFallbackPolicy(
            observer_fallback, config.slosh_observer.fallback_policy)) {
        report.fatal(
            "slosh_observer/fallback_policy",
            "unknown value '" + observer_fallback +
                "'; expected odom|fail_closed");
    }

    auto& control = config.control;
    loadParam(private_node, "control_frequency", control.frequency_hz);
    loadParam(private_node, "dt", control.dt);
    loadParam(private_node, "horizon_steps", control.horizon_steps);
    loadParam(private_node, "publish_timing/enabled",
              control.publish_latency.enabled);
    loadParam(private_node, "publish_timing/estimated_dc_sec",
              control.publish_latency.estimated_dc_sec);
    std::string delay_mode = delayPhaseModeName(control.delay_phase.mode);
    loadParam(private_node, "delay_phase/mode", delay_mode);
    control.delay_phase.mode = parseDelayPhaseMode(delay_mode);
    if (!isKnownDelayPhaseMode(delay_mode)) {
        report.warning(
            "delay_phase/mode",
            "unknown value '" + delay_mode + "' normalized to off");
    }
    loadParam(private_node, "delay_phase/publish_diagnostics",
              control.delay_phase.publish_diagnostics);
    loadParam(private_node, "delay_phase/history_window_sec",
              control.delay_phase.history_window_sec);
    loadParam(private_node, "delay_phase/cmd_timeout_sec",
              control.delay_phase.cmd_timeout_sec);
    loadParam(private_node, "delay_phase/odom_timeout_sec",
              control.delay_phase.odom_timeout_sec);
    loadParam(private_node, "delay_phase/linear_delay_sec",
              control.delay_phase.linear_delay_sec);
    loadParam(private_node, "delay_phase/angular_delay_sec",
              control.delay_phase.angular_delay_sec);
    loadParam(private_node, "delay_phase/linear_time_constant_sec",
              control.delay_phase.linear_time_constant_sec);
    loadParam(private_node, "delay_phase/angular_time_constant_sec",
              control.delay_phase.angular_time_constant_sec);
    loadParam(private_node, "delay_phase/max_prediction_sec",
              control.delay_phase.max_prediction_sec);
    loadParam(private_node, "delay_phase/max_integration_step_sec",
              control.delay_phase.max_integration_step_sec);
    loadParam(private_node, "delay_phase/min_integration_step_sec",
              control.delay_phase.min_integration_step_sec);
    loadParam(private_node, "delay_phase/require_complete_history",
              control.delay_phase.require_complete_history);

    auto& phase = config.phase_rejoin;
    std::string phase_mode = phaseRejoinModeName(phase.params.mode);
    loadParam(private_node, "phase_rejoin/mode", phase_mode);
    if (!parsePhaseRejoinMode(phase_mode, phase.params.mode)) {
        report.fatal(
            "phase_rejoin/mode",
            "unknown value '" + phase_mode + "'; expected off|monitor|enforce");
    }
    loadParam(private_node, "phase_rejoin/publish_diagnostics",
              phase.publish_diagnostics);
    loadParam(private_node, "phase_rejoin/artifact_path", phase.artifact_path);
    loadParam(private_node, "phase_rejoin/liquid_horizon_steps",
              phase.params.liquid_horizon_steps);
    loadParam(private_node, "phase_rejoin/max_residual_v",
              phase.params.max_residual_v);
    loadParam(private_node, "phase_rejoin/max_residual_omega",
              phase.params.max_residual_omega);
    loadParam(private_node, "phase_rejoin/artifact_dt_tolerance_sec",
              phase.params.artifact_dt_tolerance_sec);
    loadParam(private_node, "phase_rejoin/artifact_path_length_tolerance_m",
              phase.params.artifact_path_length_tolerance_m);
    loadParam(private_node, "phase_rejoin/artifact_path_geometry_tolerance_m",
              phase.params.artifact_path_geometry_tolerance_m);
    loadParam(private_node, "phase_rejoin/artifact_model_tolerance",
              phase.params.artifact_model_tolerance);
    loadParam(private_node, "phase_rejoin/artifact_command_tolerance",
              phase.params.artifact_command_tolerance);
    loadParam(private_node,
              "phase_rejoin/allow_development_artifact_in_enforce",
              phase.params.allow_development_artifact_in_enforce);
    loadParam(private_node, "phase_rejoin/required_contract_id",
              phase.params.required_contract_id);
    loadParam(private_node, "phase_rejoin/required_frame_id",
              phase.params.required_frame_id);
    loadParam(private_node, "phase_rejoin/candidate/backward_radius",
              phase.params.candidate.backward_radius);
    loadParam(private_node, "phase_rejoin/candidate/forward_radius",
              phase.params.candidate.forward_radius);
    loadParam(private_node, "phase_rejoin/candidate/initial_forward_radius",
              phase.params.candidate.initial_forward_radius);
    loadParam(private_node, "phase_rejoin/candidate/max_clock_lead_steps",
              phase.params.candidate.max_clock_lead_steps);
    loadParam(private_node, "phase_rejoin/candidate/weight_position",
              phase.params.candidate.weight_position);
    loadParam(private_node, "phase_rejoin/candidate/weight_yaw",
              phase.params.candidate.weight_yaw);
    loadParam(private_node, "phase_rejoin/candidate/weight_velocity",
              phase.params.candidate.weight_velocity);
    loadParam(private_node, "phase_rejoin/candidate/weight_liquid",
              phase.params.candidate.weight_liquid);

    loadParam(private_node, "state_timing/require_common_epoch",
              control.state_timing.require_common_epoch);
    loadParam(private_node, "state_timing/max_raw_skew_sec",
              control.state_timing.max_raw_skew_sec);
    loadParam(private_node, "state_timing/odom_history_sec",
              control.state_timing.odom_history_sec);
    loadParam(private_node, "state_timing/max_interpolation_gap_sec",
              control.state_timing.max_interpolation_gap_sec);
    loadParam(private_node, "state_timing/max_robot_extrapolation_sec",
              control.state_timing.max_robot_extrapolation_sec);
    loadParam(private_node,
              "execution_contract/fail_closed_on_post_limit_change",
              control.execution_contract.fail_closed_on_post_limit_change);
    loadParam(private_node, "execution_contract/max_post_limit_delta_v",
              control.execution_contract.max_post_limit_delta_v);
    loadParam(private_node, "execution_contract/max_post_limit_delta_omega",
              control.execution_contract.max_post_limit_delta_omega);

    loadParam(private_node, "reference/preprocess_enable",
              config.reference_preprocess.enable);
    loadParam(private_node, "reference/resample_spacing",
              config.reference_preprocess.resample_spacing);
    loadParam(private_node, "reference/smoothing_window",
              config.reference_preprocess.smoothing_window);
    loadParam(private_node, "reference/min_segment_length",
              config.reference_preprocess.min_segment_length);

    auto& solver = config.solver;
    loadParam(private_node, "robot/v_max", solver.v_max);
    loadParam(private_node, "robot/omega_max", solver.omega_max);
    loadParam(private_node, "robot/a_max", solver.a_max);
    loadParam(private_node, "robot/alpha_max", solver.alpha_max);
    auto& limits = config.shared_command_limits;
    limits.linear_accel_max = solver.a_max;
    limits.angular_rate_max = solver.omega_max;
    limits.angular_accel_max = solver.alpha_max;
    loadParam(private_node,
              "platform/shared_constraints/linear_accel_limit_enable",
              limits.linear_accel_limit_enable);
    loadParam(private_node, "platform/shared_constraints/linear_accel_max",
              limits.linear_accel_max);
    loadParam(private_node,
              "platform/shared_constraints/linear_accel_max_dt",
              limits.linear_accel_max_dt);
    loadParam(private_node,
              "platform/shared_constraints/angular_limit_enable",
              limits.angular_limit_enable);
    loadParam(private_node, "platform/shared_constraints/angular_rate_max",
              limits.angular_rate_max);
    loadParam(private_node, "platform/shared_constraints/angular_accel_max",
              limits.angular_accel_max);
    loadParam(private_node,
              "platform/shared_constraints/angular_accel_max_dt",
              limits.angular_accel_max_dt);

    loadParam(private_node, "experiment/corridor_width", solver.corridor_width);
    loadParam(private_node, "experiment/corridor_enable",
              solver.corridor_enable);
    loadParam(private_node, "experiment/corridor_hard_bound_enable",
              solver.corridor_hard_bound_enable);
    loadParam(private_node, "experiment/corridor_weight",
              solver.corridor_weight);
    loadParam(private_node, "experiment/obstacle_enable",
              solver.obstacle_enable);
    loadParam(private_node, "experiment/obstacle_weight",
              solver.obstacle_weight);
    loadParam(private_node, "experiment/obstacle_influence_radius",
              solver.obstacle_influence_radius);
    loadParam(private_node, "experiment/homotopy_enable",
              solver.homotopy_enable);
    loadParam(private_node, "experiment/homotopy_lateral_offset",
              solver.homotopy_lateral_offset);
    loadParam(private_node, "reference/lookahead_distance",
              solver.lookahead_distance);

    loadParam(private_node, "terminal/enable", solver.terminal.enable);
    loadParam(private_node, "terminal/goal_tolerance",
              solver.terminal.goal_tolerance);
    loadParam(private_node, "terminal/goal_reached_max_speed",
              solver.terminal.goal_reached_max_speed);
    loadParam(private_node, "terminal/goal_reached_max_omega",
              solver.terminal.goal_reached_max_omega);
    loadParam(private_node, "terminal/slowdown/enable",
              solver.terminal.slowdown_enable);
    loadParam(private_node, "terminal/slowdown/distance",
              solver.terminal.slowdown_distance);
    loadParam(private_node, "terminal/slowdown/v_max",
              solver.terminal.slowdown_v_max);
    loadParam(private_node, "terminal/capture_stop/enable",
              solver.terminal.capture_stop_enable);
    loadParam(private_node, "terminal/capture_stop/distance",
              solver.terminal.capture_stop_distance);
    loadParam(private_node, "terminal/capture_stop/v_cap",
              solver.terminal.capture_v_cap);
    loadParam(private_node, "terminal/capture_stop/goal_behind_x",
              solver.terminal.goal_behind_x);
    loadParam(private_node, "terminal/command_clamp/enable",
              solver.terminal.command_clamp_enable);
    loadParam(private_node, "terminal/command_clamp/rate_limit_enable",
              solver.terminal.rate_limit_enable);
    loadParam(private_node, "terminal/command_clamp/omega_enable",
              solver.terminal.omega_clamp_enable);
    loadParam(private_node, "terminal/command_clamp/omega_max",
              solver.terminal.omega_clamp_max);
    loadParam(private_node, "terminal/command_clamp/omega_near_goal_max",
              solver.terminal.omega_near_goal_max);
    loadParam(private_node,
              "terminal/command_clamp/omega_near_goal_distance",
              solver.terminal.omega_near_goal_distance);

    auto& safety = config.safety;
    loadParam(private_node, "terminal/spin_fail/enable",
              safety.terminal_spin.enable);
    loadParam(private_node, "terminal/spin_fail/omega_threshold",
              safety.terminal_spin.omega_threshold);
    loadParam(private_node, "terminal/spin_fail/max_duration_sec",
              safety.terminal_spin.max_duration_sec);
    loadParam(private_node, "tracking_safety/enable",
              safety.tracking.enable);
    loadParam(private_node, "tracking_safety/projection/enable",
              safety.tracking.projection_enable);
    loadParam(private_node, "tracking_safety/projection/max_distance_m",
              safety.tracking.max_projection_distance_m);
    loadParam(private_node, "tracking_safety/projection/max_duration_sec",
              safety.tracking.max_projection_duration_sec);
    loadParam(private_node, "tracking_safety/spin_fail/enable",
              safety.tracking.spin_enable);
    loadParam(private_node, "tracking_safety/spin_fail/omega_threshold",
              safety.tracking.spin_omega_threshold);
    loadParam(private_node, "tracking_safety/spin_fail/max_duration_sec",
              safety.tracking.spin_max_duration_sec);

    loadParam(private_node, "start_lock_recovery/enable",
              solver.start_lock_recovery.enable);
    loadParam(private_node, "start_lock_recovery/detect_only",
              solver.start_lock_recovery.detect_only);
    loadParam(private_node, "start_lock_recovery/start_window_s",
              solver.start_lock_recovery.start_window_s);
    loadParam(private_node, "start_lock_recovery/min_stall_duration_sec",
              solver.start_lock_recovery.min_stall_duration_sec);
    loadParam(private_node, "start_lock_recovery/progress_epsilon_s",
              solver.start_lock_recovery.progress_epsilon_s);
    loadParam(private_node, "start_lock_recovery/cmd_v_small_threshold",
              solver.start_lock_recovery.cmd_v_small_threshold);
    loadParam(private_node, "start_lock_recovery/warm_start_v_s_min",
              solver.start_lock_recovery.warm_start_v_s_min);
    loadParam(private_node, "start_lock_recovery/u0_v_s_max",
              solver.start_lock_recovery.u0_v_s_max);
    loadParam(private_node, "start_lock_recovery/require_monotonic_clip",
              solver.start_lock_recovery.require_monotonic_clip);
    loadParam(private_node, "start_lock_recovery/max_projection_distance_m",
              solver.start_lock_recovery.max_projection_distance_m);
    loadParam(private_node, "platform/kinematics", solver.platform.kinematics);
    loadParam(private_node, "acados/warm_start_flatness_enable",
              solver.warm_start_flatness_enable);
    loadParam(private_node, "acados/warm_start/type", solver.warm_start.type);
    loadParam(private_node, "acados/warm_start/use_previous_solution",
              solver.warm_start.use_previous_solution);
    loadParam(private_node, "acados/warm_start/use_slosh_rollout",
              solver.warm_start.use_slosh_rollout);
    loadParam(private_node,
              "acados/warm_start/curvature_speed_limit_enable",
              solver.warm_start.curvature_speed_limit_enable);
    loadParam(private_node, "acados/warm_start/max_reference_fit_error",
              solver.warm_start.max_reference_fit_error);
    loadParam(private_node,
              "acados/warm_start/fallback_to_previous_solution",
              solver.warm_start.fallback_to_previous_solution);
    loadParam(private_node, "acados/warm_start/fallback_to_primitive",
              solver.warm_start.fallback_to_primitive);
    if (private_node.hasParam("acados/warm_start/enable")) {
        loadParam(private_node, "acados/warm_start/enable",
                  solver.warm_start.enable);
    } else {
        solver.warm_start.enable = solver.warm_start_flatness_enable;
    }
    loadParam(private_node, "solver_backend", solver.solver_backend);
    if (!isKnownSolverBackend(solver.solver_backend)) {
        report.fatal(
            "solver_backend",
            "unknown backend '" + solver.solver_backend + "'");
    }
    auto& augmented = solver.delay_augmented_phase;
    loadParam(private_node, "delay_augmented_phase/enabled",
              augmented.enabled);
    loadParam(private_node, "delay_augmented_phase/execution_contract_id",
              augmented.execution_contract_id);
    loadParam(private_node, "delay_augmented_phase/execution_contract_hash",
              augmented.execution_contract_hash);
    loadParam(private_node, "delay_augmented_phase/expected_state_width",
              augmented.expected_state_width);
    loadParam(private_node, "delay_augmented_phase/expected_control_width",
              augmented.expected_control_width);
    loadParam(private_node, "delay_augmented_phase/expected_horizon_steps",
              augmented.expected_horizon_steps);
    loadParam(private_node, "delay_augmented_phase/parameter_schema_version",
              augmented.parameter_schema_version);
    loadParam(private_node, "delay_augmented_phase/parameter_schema_id",
              augmented.parameter_schema_id);
    loadParam(private_node, "delay_augmented_phase/parameter_schema_hash",
              augmented.parameter_schema_hash);
    loadParam(
        private_node,
        "delay_augmented_phase/expected_recovery_artifact_hash",
        augmented.expected_recovery_artifact_hash);
    int required_capabilities = static_cast<int>(
        augmented.required_capabilities);
    loadParam(private_node, "delay_augmented_phase/required_capabilities",
              required_capabilities);
    if (required_capabilities < 0) {
        report.fatal(
            "delay_augmented_phase/required_capabilities",
            "capability mask must be non-negative");
    } else {
        augmented.required_capabilities = static_cast<std::uint32_t>(
            required_capabilities);
    }

    loadSloshModel(private_node, config, report);
    loadRiskGovernor(private_node, config);

    loadParam(private_node, "map_vref/runtime_v_ref_enable",
              config.map_vref.runtime_override_enable);
    loadParam(private_node, "map_vref/runtime_v_ref",
              config.map_vref.runtime_override_mps);
    loadParam(private_node, "map_vref/profile_enable",
              config.map_vref.profile_enable);
    loadParam(private_node, "map_vref/profile_path",
              config.map_vref.profile_path);
    loadParam(private_node, "map_vref/profile_lookahead_s",
              config.map_vref.profile_lookahead_m);

    appendReport(validateAndNormalize(config), report);
    return config;
}

}  // namespace spmpc_local_planner
