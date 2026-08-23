#include "spmpc_local_planner/config/app_config.h"
#include "spmpc_local_planner/solver/api/backend.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

namespace augmented_manifest =
    delay_augmented_phase_solver_manifest;

bool lowercaseSha256(const std::string& value) {
    return value.size() == 64 &&
        std::all_of(value.begin(), value.end(), [](char character) {
            return (character >= '0' && character <= '9') ||
                (character >= 'a' && character <= 'f');
        });
}

}  // namespace

void ValidationReport::warning(const std::string& key,
                               const std::string& message) {
    issues_.push_back({ValidationSeverity::Warning, key, message});
}

void ValidationReport::fatal(const std::string& key,
                             const std::string& message) {
    issues_.push_back({ValidationSeverity::Fatal, key, message});
}

bool ValidationReport::ok() const {
    return std::none_of(
        issues_.begin(), issues_.end(), [](const ValidationIssue& issue) {
            return issue.severity == ValidationSeverity::Fatal;
        });
}

ValidationReport validateAndNormalize(AppConfig& config) {
    ValidationReport report;
    auto& vref = config.map_vref;

    if (vref.runtime_override_enable &&
        (!std::isfinite(vref.runtime_override_mps) ||
         vref.runtime_override_mps < 0.0)) {
        report.warning(
            "map_vref/runtime_v_ref",
            "invalid enabled override; disabling it to preserve variant fallback");
        vref.runtime_override_enable = false;
    }
    if (!std::isfinite(vref.profile_lookahead_m)) {
        report.warning(
            "map_vref/profile_lookahead_s",
            "non-finite lookahead normalized to zero");
        vref.profile_lookahead_m = 0.0;
    } else if (vref.profile_lookahead_m < 0.0) {
        report.warning(
            "map_vref/profile_lookahead_s",
            "negative lookahead normalized to zero");
        vref.profile_lookahead_m = 0.0;
    }
    if (vref.profile_enable && vref.profile_path.empty()) {
        report.warning(
            "map_vref/profile_path",
            "profile is enabled without a path; cycles will report PROFILE_NOT_CONFIGURED");
    }

    auto& imu = config.imu_shadow;
    if (imu.subscriber_queue_size < 1 || imu.subscriber_queue_size > 1000) {
        report.warning(
            "imu_shadow/subscriber_queue_size",
            "value outside [1, 1000] normalized to 10");
        imu.subscriber_queue_size = 10;
    }
    if (!std::isfinite(imu.observer_dt_sec) || imu.observer_dt_sec <= 0.0) {
        report.warning(
            "imu_shadow/observer_dt_sec",
            "non-positive or non-finite value normalized to 0.02 s");
        imu.observer_dt_sec = 0.02;
    }
    if (config.slosh_observer.nominal_source ==
            SloshObserverSource::ProcessedImu &&
        !imu.enable) {
        report.warning(
            "imu_shadow/enable",
            "forced on because processed_imu is the nominal observer");
        imu.enable = true;
    }

    const auto& publish_latency = config.control.publish_latency;
    if (!std::isfinite(publish_latency.estimated_dc_sec) ||
        publish_latency.estimated_dc_sec < 0.0) {
        report.fatal(
            "publish_timing/estimated_dc_sec",
            "publish latency estimate must be finite and non-negative");
    }

    auto& delay = config.control.delay_phase;
    delay.history_window_sec = std::max(0.1, delay.history_window_sec);
    delay.cmd_timeout_sec = std::max(0.0, delay.cmd_timeout_sec);
    delay.odom_timeout_sec = std::max(0.0, delay.odom_timeout_sec);
    delay.linear_delay_sec = std::max(0.0, delay.linear_delay_sec);
    delay.angular_delay_sec = std::max(0.0, delay.angular_delay_sec);
    delay.linear_time_constant_sec = std::max(
        0.0, delay.linear_time_constant_sec);
    delay.angular_time_constant_sec = std::max(
        0.0, delay.angular_time_constant_sec);
    delay.max_prediction_sec = std::max(0.0, delay.max_prediction_sec);
    delay.max_integration_step_sec = std::max(
        1e-4, delay.max_integration_step_sec);
    delay.min_integration_step_sec = std::max(
        1e-6, delay.min_integration_step_sec);
    if (delay.min_integration_step_sec > delay.max_integration_step_sec) {
        delay.min_integration_step_sec = delay.max_integration_step_sec;
    }

    const auto& timing = config.control.state_timing;
    const auto& contract = config.control.execution_contract;
    const bool valid_state_timing =
        std::isfinite(timing.max_raw_skew_sec) &&
        timing.max_raw_skew_sec >= 0.0 &&
        std::isfinite(timing.odom_history_sec) &&
        timing.odom_history_sec > 0.0 &&
        std::isfinite(timing.max_interpolation_gap_sec) &&
        timing.max_interpolation_gap_sec > 0.0 &&
        std::isfinite(timing.max_robot_extrapolation_sec) &&
        timing.max_robot_extrapolation_sec >= 0.0;
    if (!valid_state_timing) {
        report.fatal(
            "state_timing",
            "invalid common-epoch skew/history/interpolation contract");
    }
    const bool valid_execution_contract =
        std::isfinite(contract.max_post_limit_delta_v) &&
        contract.max_post_limit_delta_v >= 0.0 &&
        std::isfinite(contract.max_post_limit_delta_omega) &&
        contract.max_post_limit_delta_omega >= 0.0;
    if (!valid_execution_contract) {
        report.fatal(
            "execution_contract",
            "post-limit command deltas must be finite and non-negative");
    }
    if (config.phase_rejoin.params.mode == PhaseRejoinMode::Enforce &&
        !timing.require_common_epoch) {
        report.fatal(
            "state_timing/require_common_epoch",
            "phase_rejoin/enforce requires common-epoch alignment");
    }

    auto& limits = config.shared_command_limits;
    limits.linear_accel_max = std::max(0.0, limits.linear_accel_max);
    limits.linear_accel_max_dt = std::max(
        1e-3, limits.linear_accel_max_dt);
    if (limits.angular_rate_max <= 0.0) {
        limits.angular_rate_max = config.solver.omega_max;
    }
    if (limits.angular_accel_max <= 0.0) {
        limits.angular_accel_max = config.solver.alpha_max;
    }
    limits.angular_rate_max = std::max(0.0, limits.angular_rate_max);
    limits.angular_accel_max = std::max(0.0, limits.angular_accel_max);
    limits.angular_accel_max_dt = std::max(
        1e-3, limits.angular_accel_max_dt);

    auto& safety = config.safety;
    safety.nominal_period_sec = config.control.dt;
    safety.terminal_spin.omega_threshold = std::max(
        0.0, safety.terminal_spin.omega_threshold);
    safety.terminal_spin.max_duration_sec = std::max(
        0.0, safety.terminal_spin.max_duration_sec);
    safety.tracking.max_projection_distance_m = std::max(
        0.0, safety.tracking.max_projection_distance_m);
    safety.tracking.max_projection_duration_sec = std::max(
        0.0, safety.tracking.max_projection_duration_sec);
    safety.tracking.spin_omega_threshold = std::max(
        0.0, safety.tracking.spin_omega_threshold);
    safety.tracking.spin_max_duration_sec = std::max(
        0.0, safety.tracking.spin_max_duration_sec);

    if (!std::isfinite(config.solver.slosh.slosh_height_max) ||
        config.solver.slosh.slosh_height_max <= 0.0) {
        report.warning(
            "slosh/slosh_height_max",
            "invalid maximum normalized to slosh_height_ref");
        config.solver.slosh.slosh_height_max = std::max(
            1e-6, config.solver.slosh.slosh_height_ref);
    }
    config.solver.slosh.dt = config.control.dt;

    auto& variant = config.variant;
    if (variant.w_alpha < 0.0) {
        variant.w_alpha = variant.w_smooth;
    }
    if (variant.w_du_a < 0.0) {
        variant.w_du_a = variant.w_smooth;
    }
    if (variant.w_du_vs < 0.0) {
        variant.w_du_vs = variant.w_smooth;
    }
    const double heading_progress_values[] = {
        variant.w_heading,
        variant.w_progress_coupling,
        variant.w_yaw_rate_tracking,
        variant.heading_feedback_gain,
    };
    for (double value : heading_progress_values) {
        if (!std::isfinite(value) || value < 0.0) {
            report.fatal(
                "variants/" + variant.name + "/heading_progress",
                "weights and feedback gain must be finite and non-negative");
            break;
        }
    }
    if (variant.slosh_cost_horizon_steps < -1 ||
        !std::isfinite(variant.slosh_cost_tail_discount) ||
        variant.slosh_cost_tail_discount < 0.0 ||
        variant.slosh_cost_tail_discount > 1.0) {
        report.fatal(
            "variants/" + variant.name + "/slosh_cost_horizon",
            "steps must be >= -1 and tail discount must be in [0, 1]");
    }
    const bool augmented_backend = config.solver.solver_backend ==
        kSolverBackendDelayAugmentedPhaseAcados;
    if (config.solver.delay_augmented_phase.enabled != augmented_backend) {
        report.fatal(
            "delay_augmented_phase/enabled",
            "must be true exactly when solver_backend=delay_augmented_phase_acados");
    }
    if (augmented_backend &&
        config.phase_rejoin.params.mode != PhaseRejoinMode::Enforce) {
        report.fatal(
            "phase_rejoin/mode",
            "delay_augmented_phase_acados requires enforce; monitor/off cannot publish its command");
    }
    if (augmented_backend && !config.control.publish_latency.enabled) {
        report.fatal(
            "publish_timing/enabled",
            "delay_augmented_phase_acados requires an explicit publish epoch estimate");
    }
    if (augmented_backend && delay.mode != DelayPhaseMode::Off) {
        report.fatal(
            "delay_phase/mode",
            "delay_augmented_phase_acados forbids a second history-only state shift");
    }
    if (augmented_backend && !delay.require_complete_history) {
        report.fatal(
            "delay_phase/require_complete_history",
            "delay_augmented_phase_acados requires complete final-command history");
    }
    if (augmented_backend &&
        (config.shared_command_limits.linear_accel_limit_enable ||
         config.shared_command_limits.angular_limit_enable)) {
        report.fatal(
            "platform/shared_constraints",
            "delay_augmented_phase_acados forbids post-solver command limiters; its OCP and recovery artifact must own the published-command envelope");
    }
    if (augmented_backend &&
        (config.map_vref.runtime_override_enable ||
         config.map_vref.profile_enable ||
         config.slosh_risk_governor.enable)) {
        report.fatal(
            "speed_reference",
            "delay_augmented_phase_acados follows the frozen phase-indexed nominal sequence and does not consume runtime v_ref/profile/governor overrides");
    }
    if (augmented_backend &&
        config.phase_rejoin.params.required_contract_id.empty()) {
        report.fatal(
            "phase_rejoin/required_contract_id",
            "delay_augmented_phase_acados requires an explicitly frozen nominal contract id");
    }
    if (augmented_backend &&
        (!std::isfinite(config.phase_rejoin.params.max_residual_v) ||
         !std::isfinite(config.phase_rejoin.params.max_residual_omega) ||
         config.phase_rejoin.params.max_residual_v < 0.0 ||
         config.phase_rejoin.params.max_residual_omega < 0.0 ||
         config.phase_rejoin.params.max_residual_v >
             augmented_manifest::kLinearOutputMax -
                 augmented_manifest::kLinearOutputMin ||
         config.phase_rejoin.params.max_residual_omega >
             augmented_manifest::kAngularOutputMax -
                 augmented_manifest::kAngularOutputMin)) {
        report.fatal(
            "phase_rejoin/residual_bounds",
            "delay_augmented_phase_acados residual bounds must fit the compiled published-command envelope");
    }
    if (augmented_backend &&
        !lowercaseSha256(config.solver.delay_augmented_phase
                            .expected_recovery_artifact_hash)) {
        report.fatal(
            "delay_augmented_phase/expected_recovery_artifact_hash",
            "a lowercase SHA-256 from a separately frozen recovery asset is required");
    }
    return report;
}

}  // namespace spmpc_local_planner
