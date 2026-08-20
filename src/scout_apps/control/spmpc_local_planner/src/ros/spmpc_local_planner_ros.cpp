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

    ValidationReport app_config_report;
    app_config_ = RosConfigLoader::load(pnh, app_config_report);
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

    const auto& interface = app_config_.ros_interface;
    experiment_mode_ = interface.experiment_mode;
    odom_topic_ = interface.odom_topic;
    imu_topic_ = interface.imu_topic;
    path_topic_ = interface.reference_path_topic;
    costmap_topic_ = interface.costmap_topic;
    cmd_topic_ = interface.cmd_vel_topic;
    robot_base_frame_ = interface.robot_base_frame;
    reference_target_frame_ = interface.reference_target_frame;
    use_tf_pose_ = interface.use_tf_pose;
    tf_timeout_sec_ = interface.tf_timeout_sec;
    publish_cmd_vel_ = interface.publish_cmd_vel;

    imu_shadow_enable_ = app_config_.imu_shadow.enable;
    imu_shadow_publish_diagnostics_ =
        app_config_.imu_shadow.publish_diagnostics;
    imu_expected_frame_ = app_config_.imu_shadow.expected_frame;
    imu_subscriber_queue_size_ =
        app_config_.imu_shadow.subscriber_queue_size;
    imu_observer_dt_sec_ = app_config_.imu_shadow.observer_dt_sec;
    slosh_observer_selector_params_ = app_config_.slosh_observer;

    control_frequency_ = app_config_.control.frequency_hz;
    dt_ = app_config_.control.dt;
    horizon_steps_ = app_config_.control.horizon_steps;
    delay_phase_params_ = app_config_.control.delay_phase;
    state_timing_params_ = app_config_.control.state_timing;
    command_contract_params_ = app_config_.control.execution_contract;
    phase_rejoin_params_ = app_config_.phase_rejoin.params;
    phase_rejoin_publish_diagnostics_ =
        app_config_.phase_rejoin.publish_diagnostics;
    phase_rejoin_artifact_path_ = app_config_.phase_rejoin.artifact_path;
    reference_preprocess_params_ = app_config_.reference_preprocess;
    variant_ = app_config_.variant;

    SolverParams solver_params = app_config_.solver;
    const ProcessedImuParams processed_imu_params =
        app_config_.imu_shadow.processed;
    const auto& limits = app_config_.shared_command_limits;
    shared_cmd_linear_accel_limit_enable_ =
        limits.linear_accel_limit_enable;
    shared_cmd_linear_accel_max_ = limits.linear_accel_max;
    shared_cmd_linear_accel_max_dt_ = limits.linear_accel_max_dt;
    shared_cmd_angular_limit_enable_ = limits.angular_limit_enable;
    shared_cmd_angular_rate_max_ = limits.angular_rate_max;
    shared_cmd_angular_accel_max_ = limits.angular_accel_max;
    shared_cmd_angular_accel_max_dt_ = limits.angular_accel_max_dt;
    command_history_.configure(delay_phase_params_.history_window_sec);

    const bool imu_is_nominal =
        slosh_observer_selector_params_.nominal_source ==
        SloshObserverSource::ProcessedImu;
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
    if (!control_cycle_engine_.configureCommandPipeline(
            command_pipeline_config, command_pipeline_error)) {
        ROS_FATAL("[spmpc_local_planner] command pipeline configuration failed: %s",
                  command_pipeline_error.c_str());
        return false;
    }
    const SafetySupervisorConfig safety_config = app_config_.safety;
    std::string safety_error;
    if (!control_cycle_engine_.configureSafety(safety_config, safety_error)) {
        ROS_FATAL("[spmpc_local_planner] safety supervisor configuration failed: %s",
                  safety_error.c_str());
        return false;
    }
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
    if (!control_input_preparer_.configureObserver(
            slosh_observer_selector_params_)) {
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
    if (!control_input_preparer_.configurePrediction(solver_params.slosh)) {
        ROS_WARN("[spmpc_local_planner] delay_phase shadow slosh predictor configure failed; shadow slosh stays pass-through");
    }
    SpeedReferenceControllerConfig speed_reference_config;
    speed_reference_config.runtime_override_enable =
        app_config_.map_vref.runtime_override_enable;
    speed_reference_config.runtime_override_mps =
        app_config_.map_vref.runtime_override_mps;
    speed_reference_config.profile_enable =
        app_config_.map_vref.profile_enable;
    speed_reference_config.profile_path = app_config_.map_vref.profile_path;
    speed_reference_config.profile_lookahead_m =
        app_config_.map_vref.profile_lookahead_m;
    speed_reference_config.variant_v_ref = variant_.v_ref;
    speed_reference_config.v_max = solver_params.v_max;
    speed_reference_config.slosh_variant_enabled = variant_.slosh_enable;
    speed_reference_config.slosh_model = solver_params.slosh;
    speed_reference_config.slosh_governor =
        app_config_.slosh_risk_governor;
    const SpeedReferenceConfigureResult speed_reference_result =
        control_cycle_engine_.configureSpeedReference(speed_reference_config);
    if (speed_reference_result.profile_requested &&
        speed_reference_result.profile_load.success) {
        ROS_INFO("[spmpc_local_planner] loaded map_vref profile %s with %zu "
                 "samples (%zu invalid rows skipped)",
                 speed_reference_config.profile_path.c_str(),
                 speed_reference_result.profile_load.accepted_rows,
                 speed_reference_result.profile_load.skipped_rows);
    } else if (speed_reference_result.profile_requested &&
               !speed_reference_config.profile_path.empty()) {
        ROS_WARN("[spmpc_local_planner] map_vref profile load failed "
                 "status=%s detail=%s",
                 speed_reference_result.profile_load.status.c_str(),
                 speed_reference_result.profile_load.detail.c_str());
    }
    if (!speed_reference_result.governor_configured &&
        speed_reference_config.slosh_governor.enable) {
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
    const StampNs stamp_ns = static_cast<StampNs>(stamp.toNSec());
    const std::string reason = audit && !audit->status.empty()
        ? audit->status
        : "FAIL_CLOSED_ZERO";
    const CommandPipelineResult result =
        control_cycle_engine_.finalizeFailClosedZero(
            stamp_ns, publish_cmd_vel_, reason);

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
            audit->timing.command_publish_stamp_ns = stamp_ns;
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
    const StampNs stamp_ns = static_cast<StampNs>(stamp.toNSec());
    const CommandPipelineResult result =
        control_cycle_engine_.finalizeCommand(
            decision, stamp_ns, publish_cmd_vel_);
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
            audit->timing.command_publish_stamp_ns = stamp_ns;
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
    // effective config 在纯 C++ speed-reference 阶段完成后发布，避免动态 v_ref 滞后一拍。

    const ros::Time cycle_start = ros::Time::now();
    ControlCycleAuditDebug cycle_audit;
    cycle_audit.timing.cycle_id = ++next_cycle_id_;
    cycle_audit.timing.cycle_start_stamp_ns =
        static_cast<std::int64_t>(cycle_start.toNSec());
    cycle_audit.variant = variant_.name;
    cycle_audit.publish_cmd_vel = publish_cmd_vel_;

    const ControlCycleGateDecision prerequisite_gate =
        evaluateControlCyclePrerequisites(
            have_odom_, problem_.hasReferencePath());
    if (!prerequisite_gate.ready) {
        control_cycle_engine_.resetSafety();
        diagnostics_.publishStatus(prerequisite_gate.status);
        if (prerequisite_gate.publish_early_delay_status) {
            publishDelayPhaseEarlyStatus(
                prerequisite_gate.delay_phase_status);
        }
        cycle_audit.status = prerequisite_gate.status;
        publishZeroCommand(
            prerequisite_gate.intervention, &cycle_audit);
        return;
    }

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
    const bool phase_rejoin_enforce =
        phase_rejoin_params_.mode == PhaseRejoinMode::Enforce;
    const bool solver_consumes_selected_state =
        variant_.slosh_enable || phase_rejoin_enforce;

    ControlCycleInputRequest input_request;
    input_request.cycle_id = cycle_audit.timing.cycle_id;
    input_request.cycle_start_ns =
        cycle_audit.timing.cycle_start_stamp_ns;
    input_request.selection_time_ns = static_cast<StampNs>(
        observer_selection_now.toNSec());
    input_request.raw_robot_state_stamp_ns = static_cast<StampNs>(
        last_odom_.header.stamp.toNSec());
    input_request.last_odom_receive_ns = last_odom_receive_stamp_.isZero()
        ? 0
        : static_cast<StampNs>(last_odom_receive_stamp_.toNSec());
    input_request.odom_observer = odom_observer_health;
    input_request.imu_observer = imu_observer_health;
    input_request.solver_consumes_selected_state =
        solver_consumes_selected_state;
    input_request.state_timing = state_timing_params_;
    input_request.dt = dt_;
    input_request.horizon_steps = horizon_steps_;
    input_request.delay_phase = delay_phase_params_;
    input_request.phase_rejoin_needs_prediction =
        phaseRejoinNeedsPrediction();
    input_request.command_history = &command_history_;
    input_request.robot_state_lookup = [this](StampNs target_epoch_ns) {
        RobotStateLookupResult lookup;
        if (target_epoch_ns <= 0) {
            lookup.valid = robotStateFromLatest(lookup.state);
            lookup.status = lookup.valid
                ? "LATEST"
                : "LATEST_TF_UNAVAILABLE";
            return lookup;
        }
        lookup.valid = robotStateAtEpoch(
            rosTimeFromNanoseconds(target_epoch_ns),
            lookup.state,
            lookup.interpolated,
            lookup.extrapolated,
            lookup.status);
        return lookup;
    };

    ControlCycleInputResult input_preparation =
        control_input_preparer_.prepareState(input_request);
    const SloshObserverSelection observer_selection =
        input_preparation.observer_selection;
    cycle_audit.observer_source =
        static_cast<std::uint8_t>(observer_selection.effective_source);
    cycle_audit.odom_excitation = makeExcitationAudit(
        odom_observer_health.snapshot.excitation);
    cycle_audit.imu_excitation = makeExcitationAudit(
        imu_observer_health.snapshot.excitation);
    cycle_audit.timing = input_preparation.timing;

    ControlCycleGateDecision input_gate =
        evaluateControlInputGate(input_preparation);
    if (!input_gate.ready) {
        control_cycle_engine_.resetSafety();
        diagnostics_.publishStatus(input_gate.status);
        if (input_preparation.failure ==
            ControlInputFailure::ObserverUnavailable) {
            ROS_WARN_THROTTLE(
                1.0,
                "[spmpc_local_planner] fail-closed liquid observer: nominal=%s status=%s reason=%s odom_age=%.3f imu_age=%.3f",
                sloshObserverSourceName(observer_selection.nominal_source),
                sloshObserverSelectionStatusName(observer_selection.status),
                sloshObserverSelectionReasonName(observer_selection.reason),
                observer_selection.odom_state_age_sec,
                observer_selection.imu_state_age_sec);
        }
        if (input_gate.publish_early_delay_status) {
            publishDelayPhaseEarlyStatus(
                input_gate.delay_phase_status);
        }
        cycle_audit.status = input_gate.status;
        publishSloshObserverSelectionDebug(
            observer_selection_now,
            observer_selection,
            solver_consumes_selected_state,
            cycle_audit.timing);
        publishZeroCommand(input_gate.intervention, &cycle_audit);
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
    publishSloshObserverSelectionDebug(
        observer_selection_now,
        observer_selection,
        solver_consumes_selected_state,
        cycle_audit.timing);
    SolverInput input = input_preparation.raw_input;
    diagnostics_.publishRawState(input.robot, input.slosh, slosh_height_coeff);

    const SpeedReferenceEvaluation speed_reference =
        control_cycle_engine_.prepareSpeedReference(input);
    diagnostics_.publishSloshGovernor(speed_reference.governor);
    // effective_config_.v_ref 是本周期 solver 将看到的速度参考：runtime/profile override 优先，
    // 并按 solver v_max 做同口径 clamp。必须在发布前更新，避免 /spmpc/debug/effective_config 滞后一拍。
    if (speed_reference.effective_v_ref_valid) {
        effective_config_.v_ref = speed_reference.effective_v_ref;
    }
    diagnostics_.publishEffectiveConfig(effective_config_);

    const ros::Time delay_phase_now = ros::Time::now();
    input_preparation.raw_input = input;
    input_preparation = control_input_preparer_.completePrediction(
        input_request,
        static_cast<StampNs>(delay_phase_now.toNSec()),
        input_preparation);
    cycle_audit.timing = input_preparation.timing;
    input_gate = evaluateControlInputGate(input_preparation);
    if (!input_gate.ready) {
        control_cycle_engine_.resetSafety();
        cycle_audit.status = input_gate.status;
        diagnostics_.publishStatus(cycle_audit.status);
        publishZeroCommand(input_gate.intervention, &cycle_audit);
        return;
    }

    const DelayPhaseStatusCode delay_phase_status =
        input_preparation.delay_phase_status;
    ExecutionStatePrediction shadow_prediction =
        input_preparation.prediction;
    ExecutionStatePrediction* shadow_prediction_ptr =
        input_preparation.have_prediction ? &shadow_prediction : nullptr;
    diagnostics_.publishPredictedState(shadow_prediction, slosh_height_coeff);

    SolverInput solve_input = input_preparation.solver_input;
    const bool robot_delay_compensation_applied =
        input_preparation.robot_delay_compensation_applied;
    const bool liquid_delay_compensation_applied =
        input_preparation.liquid_delay_compensation_applied;
    const bool solver_origin_at_execution_front =
        input_preparation.solver_origin_at_execution_front;

    cycle_audit.timing.solve_start_stamp_ns = static_cast<std::int64_t>(
        ros::Time::now().toNSec());
    solve_input.cycle_timing = cycle_audit.timing;
    double spin_gate_dt = dt_;
    if (!event.last_real.isZero() && !event.current_real.isZero()) {
        spin_gate_dt = (event.current_real - event.last_real).toSec();
    }
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
    engine_request.execution_front_steps =
        input_preparation.execution_front_steps;
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
    CommandInterventionDebug intervention =
        makeCommandInterventionDebug(engine_result.telemetry);
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
        diagnostics_.publishPhaseRejoin(
            engine_result.phase_debug,
            cycle_audit.timing,
            problem_.referenceFrameId(),
            shadow_prediction_ptr);
    }
    diagnostics_.publishOutput(output, problem_.referenceFrameId());

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

}  // namespace spmpc_local_planner
