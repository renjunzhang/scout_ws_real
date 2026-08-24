#include "spmpc_local_planner/config/app_config.h"
#include "spmpc_local_planner/solver/api/backend.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace spmpc_local_planner {
namespace {

namespace augmented_manifest =
    delay_augmented_phase_solver_manifest;

bool hasIssue(const ValidationReport& report,
              ValidationSeverity severity,
              const std::string& key) {
    for (const auto& issue : report.issues()) {
        if (issue.severity == severity && issue.key == key) {
            return true;
        }
    }
    return false;
}

TEST(AppConfig, PreservesHistoricalTypedDefaults) {
    AppConfig config;
    const ValidationReport report = validateAndNormalize(config);

    EXPECT_TRUE(report.ok());
    EXPECT_EQ(config.requested_variant, "B0");
    EXPECT_EQ(config.variant.name, "B0");
    EXPECT_EQ(config.ros_interface.odom_topic, "/odom");
    EXPECT_EQ(config.ros_interface.cmd_vel_topic, "/cmd_vel");
    EXPECT_DOUBLE_EQ(config.control.frequency_hz, 30.0);
    EXPECT_DOUBLE_EQ(config.control.dt, 1.0 / 30.0);
    EXPECT_EQ(config.control.horizon_steps, 60);
    EXPECT_FALSE(config.control.publish_latency.enabled);
    EXPECT_DOUBLE_EQ(config.control.publish_latency.estimated_dc_sec, 0.0);
    EXPECT_DOUBLE_EQ(config.control.delay_phase.linear_delay_sec, 0.15);
    EXPECT_DOUBLE_EQ(config.control.delay_phase.angular_delay_sec, 0.22);
    EXPECT_DOUBLE_EQ(config.control.delay_phase.linear_time_constant_sec,
                     0.0);
    EXPECT_DOUBLE_EQ(config.control.delay_phase.angular_time_constant_sec,
                     0.0);
    EXPECT_DOUBLE_EQ(config.solver.v_max, 0.8);
    EXPECT_DOUBLE_EQ(config.solver.omega_max, 1.2);
    EXPECT_DOUBLE_EQ(config.solver.slosh.dt, config.control.dt);
    EXPECT_DOUBLE_EQ(config.shared_command_limits.linear_accel_max,
                     config.solver.a_max);
    EXPECT_DOUBLE_EQ(config.safety.nominal_period_sec, config.control.dt);
    EXPECT_FALSE(config.phase_rejoin.params.progress_governor_enabled);
    EXPECT_FALSE(config.phase_rejoin.params.successor_admission_enabled);
    EXPECT_FALSE(config.phase_rejoin.params.tail_commit_enabled);
    EXPECT_EQ(config.phase_rejoin.params.max_consecutive_phase_holds, 3);
}

TEST(AppConfig, RejectsNegativePhaseHoldLimit) {
    AppConfig config;
    config.phase_rejoin.params.max_consecutive_phase_holds = -1;

    const ValidationReport report = validateAndNormalize(config);

    EXPECT_FALSE(report.ok());
    EXPECT_TRUE(hasIssue(
        report,
        ValidationSeverity::Fatal,
        "phase_rejoin/max_consecutive_phase_holds"));
}

TEST(AppConfig, TailCommitExtensionsAreAtomicAndEnforceOnly) {
    AppConfig partial;
    partial.phase_rejoin.params.progress_governor_enabled = true;
    EXPECT_FALSE(validateAndNormalize(partial).ok());

    AppConfig monitor;
    monitor.phase_rejoin.params.mode = PhaseRejoinMode::Monitor;
    monitor.phase_rejoin.params.progress_governor_enabled = true;
    monitor.phase_rejoin.params.successor_admission_enabled = true;
    monitor.phase_rejoin.params.tail_commit_enabled = true;
    EXPECT_FALSE(validateAndNormalize(monitor).ok());

    AppConfig enforce;
    enforce.phase_rejoin.params.mode = PhaseRejoinMode::Enforce;
    enforce.phase_rejoin.params.progress_governor_enabled = true;
    enforce.phase_rejoin.params.successor_admission_enabled = true;
    enforce.phase_rejoin.params.tail_commit_enabled = true;
    EXPECT_TRUE(validateAndNormalize(enforce).ok());
}

TEST(AppConfig, NormalizesRuntimeGroupsAtOneTypedBoundary) {
    AppConfig config;
    config.imu_shadow.subscriber_queue_size = 0;
    config.imu_shadow.observer_dt_sec =
        std::numeric_limits<double>::quiet_NaN();
    config.slosh_observer.nominal_source =
        SloshObserverSource::ProcessedImu;
    config.imu_shadow.enable = false;

    auto& delay = config.control.delay_phase;
    delay.history_window_sec = -1.0;
    delay.cmd_timeout_sec = -2.0;
    delay.odom_timeout_sec = -3.0;
    delay.linear_delay_sec = -4.0;
    delay.angular_delay_sec = -5.0;
    delay.max_integration_step_sec = 0.001;
    delay.min_integration_step_sec = 0.01;

    config.solver.omega_max = 0.9;
    config.solver.alpha_max = 0.7;
    auto& limits = config.shared_command_limits;
    limits.linear_accel_max = -1.0;
    limits.linear_accel_max_dt = 0.0;
    limits.angular_rate_max = 0.0;
    limits.angular_accel_max = -1.0;
    limits.angular_accel_max_dt = 0.0;

    config.safety.terminal_spin.omega_threshold = -1.0;
    config.safety.tracking.max_projection_distance_m = -1.0;
    config.solver.slosh.slosh_height_ref = 0.006;
    config.solver.slosh.slosh_height_max =
        std::numeric_limits<double>::quiet_NaN();
    config.variant.w_smooth = 0.4;
    config.variant.w_alpha = -1.0;
    config.variant.w_du_a = -1.0;
    config.variant.w_du_vs = -1.0;

    const ValidationReport report = validateAndNormalize(config);

    EXPECT_TRUE(report.ok());
    EXPECT_EQ(config.imu_shadow.subscriber_queue_size, 10);
    EXPECT_DOUBLE_EQ(config.imu_shadow.observer_dt_sec, 0.02);
    EXPECT_TRUE(config.imu_shadow.enable);
    EXPECT_DOUBLE_EQ(delay.history_window_sec, 0.1);
    EXPECT_DOUBLE_EQ(delay.cmd_timeout_sec, 0.0);
    EXPECT_DOUBLE_EQ(delay.odom_timeout_sec, 0.0);
    EXPECT_DOUBLE_EQ(delay.linear_delay_sec, 0.0);
    EXPECT_DOUBLE_EQ(delay.angular_delay_sec, 0.0);
    EXPECT_DOUBLE_EQ(delay.min_integration_step_sec,
                     delay.max_integration_step_sec);
    EXPECT_DOUBLE_EQ(limits.linear_accel_max, 0.0);
    EXPECT_DOUBLE_EQ(limits.linear_accel_max_dt, 1e-3);
    EXPECT_DOUBLE_EQ(limits.angular_rate_max, 0.9);
    EXPECT_DOUBLE_EQ(limits.angular_accel_max, 0.7);
    EXPECT_DOUBLE_EQ(limits.angular_accel_max_dt, 1e-3);
    EXPECT_DOUBLE_EQ(config.safety.terminal_spin.omega_threshold, 0.0);
    EXPECT_DOUBLE_EQ(
        config.safety.tracking.max_projection_distance_m, 0.0);
    EXPECT_DOUBLE_EQ(config.solver.slosh.slosh_height_max, 0.006);
    EXPECT_DOUBLE_EQ(config.variant.w_alpha, 0.4);
    EXPECT_DOUBLE_EQ(config.variant.w_du_a, 0.4);
    EXPECT_DOUBLE_EQ(config.variant.w_du_vs, 0.4);
}

TEST(AppConfig, InvalidTimingAndExecutionContractsAreFatal) {
    AppConfig timing_config;
    timing_config.control.state_timing.max_raw_skew_sec = -0.1;
    const ValidationReport timing_report =
        validateAndNormalize(timing_config);
    EXPECT_FALSE(timing_report.ok());
    EXPECT_TRUE(hasIssue(
        timing_report, ValidationSeverity::Fatal, "state_timing"));

    AppConfig publish_timing_config;
    publish_timing_config.control.publish_latency.enabled = true;
    publish_timing_config.control.publish_latency.estimated_dc_sec = -0.1;
    const ValidationReport publish_timing_report =
        validateAndNormalize(publish_timing_config);
    EXPECT_FALSE(publish_timing_report.ok());
    EXPECT_TRUE(hasIssue(
        publish_timing_report,
        ValidationSeverity::Fatal,
        "publish_timing/estimated_dc_sec"));

    AppConfig execution_config;
    execution_config.control.execution_contract.max_post_limit_delta_v =
        std::numeric_limits<double>::infinity();
    const ValidationReport execution_report =
        validateAndNormalize(execution_config);
    EXPECT_FALSE(execution_report.ok());
    EXPECT_TRUE(hasIssue(
        execution_report,
        ValidationSeverity::Fatal,
        "execution_contract"));
}

TEST(AppConfig, PhaseEnforceRequiresCommonEpoch) {
    AppConfig config;
    config.phase_rejoin.params.mode = PhaseRejoinMode::Enforce;
    config.control.state_timing.require_common_epoch = false;

    const ValidationReport report = validateAndNormalize(config);

    EXPECT_FALSE(report.ok());
    EXPECT_TRUE(hasIssue(
        report,
        ValidationSeverity::Fatal,
        "state_timing/require_common_epoch"));
}

TEST(AppConfig, LiquidCostHorizonContractIsFatal) {
    AppConfig config;
    config.variant.name = "B_slosh";
    config.variant.slosh_cost_horizon_steps = -2;
    config.variant.slosh_cost_tail_discount = 1.5;

    const ValidationReport report = validateAndNormalize(config);

    EXPECT_FALSE(report.ok());
    EXPECT_TRUE(hasIssue(
        report,
        ValidationSeverity::Fatal,
        "variants/B_slosh/slosh_cost_horizon"));
}

TEST(AppConfig, HeadingProgressContractRejectsInvalidValues) {
    AppConfig config;
    config.variant.name = "B_smooth";
    config.variant.w_heading =
        std::numeric_limits<double>::quiet_NaN();

    const ValidationReport non_finite = validateAndNormalize(config);

    EXPECT_FALSE(non_finite.ok());
    EXPECT_TRUE(hasIssue(
        non_finite, ValidationSeverity::Fatal,
        "variants/B_smooth/heading_progress"));

    config.variant.w_heading = 1.0;
    config.variant.w_progress_coupling = -1.0;
    const ValidationReport negative = validateAndNormalize(config);
    EXPECT_FALSE(negative.ok());
    EXPECT_TRUE(hasIssue(
        negative, ValidationSeverity::Fatal,
        "variants/B_smooth/heading_progress"));
}

TEST(AppConfig, AugmentedBackendRequiresEveryExplicitAdmissionBoundary) {
    AppConfig config;
    config.solver.solver_backend =
        kSolverBackendDelayAugmentedPhaseAcados;
    config.solver.delay_augmented_phase.enabled = true;
    config.phase_rejoin.params.mode = PhaseRejoinMode::Enforce;

    const ValidationReport missing = validateAndNormalize(config);
    EXPECT_FALSE(missing.ok());
    EXPECT_TRUE(hasIssue(
        missing, ValidationSeverity::Fatal, "publish_timing/enabled"));
    EXPECT_TRUE(hasIssue(
        missing, ValidationSeverity::Fatal,
        "delay_phase/require_complete_history"));
    EXPECT_TRUE(hasIssue(
        missing, ValidationSeverity::Fatal,
        "phase_rejoin/required_contract_id"));
    EXPECT_TRUE(hasIssue(
        missing, ValidationSeverity::Fatal,
        "delay_augmented_phase/expected_recovery_artifact_hash"));
    EXPECT_TRUE(hasIssue(
        missing, ValidationSeverity::Fatal,
        "platform/shared_constraints"));

    config.control.publish_latency.enabled = true;
    config.control.delay_phase.mode = DelayPhaseMode::Off;
    config.control.delay_phase.require_complete_history = true;
    config.phase_rejoin.params.required_contract_id = "test_v3";
    config.solver.delay_augmented_phase
        .expected_recovery_artifact_hash = std::string(64, 'a');
    config.shared_command_limits.linear_accel_limit_enable = false;
    const ValidationReport complete = validateAndNormalize(config);
    EXPECT_TRUE(complete.ok());
}

TEST(AppConfig, AugmentedBackendRejectsUnusedRuntimeSpeedFeatures) {
    AppConfig config;
    config.solver.solver_backend =
        kSolverBackendDelayAugmentedPhaseAcados;
    config.solver.delay_augmented_phase.enabled = true;
    config.phase_rejoin.params.mode = PhaseRejoinMode::Enforce;
    config.control.publish_latency.enabled = true;
    config.control.delay_phase.require_complete_history = true;
    config.phase_rejoin.params.required_contract_id = "test_v3";
    config.solver.delay_augmented_phase
        .expected_recovery_artifact_hash = std::string(64, 'a');
    config.shared_command_limits.linear_accel_limit_enable = false;

    config.map_vref.profile_enable = true;
    EXPECT_TRUE(hasIssue(
        validateAndNormalize(config), ValidationSeverity::Fatal,
        "speed_reference"));
    config.map_vref.profile_enable = false;
    config.map_vref.runtime_override_enable = true;
    config.map_vref.runtime_override_mps = 0.2;
    EXPECT_TRUE(hasIssue(
        validateAndNormalize(config), ValidationSeverity::Fatal,
        "speed_reference"));
    config.map_vref.runtime_override_enable = false;
    config.slosh_risk_governor.enable = true;
    EXPECT_TRUE(hasIssue(
        validateAndNormalize(config), ValidationSeverity::Fatal,
        "speed_reference"));
}

TEST(AppConfig, AugmentedResidualBoundsFitCompiledCommandEnvelope) {
    AppConfig config;
    config.solver.solver_backend =
        kSolverBackendDelayAugmentedPhaseAcados;
    config.solver.delay_augmented_phase.enabled = true;
    config.phase_rejoin.params.mode = PhaseRejoinMode::Enforce;
    config.control.publish_latency.enabled = true;
    config.control.delay_phase.require_complete_history = true;
    config.phase_rejoin.params.required_contract_id = "test_v3";
    config.solver.delay_augmented_phase
        .expected_recovery_artifact_hash = std::string(64, 'a');
    config.shared_command_limits.linear_accel_limit_enable = false;
    config.phase_rejoin.params.max_residual_v =
        augmented_manifest::kLinearOutputMax -
        augmented_manifest::kLinearOutputMin;
    config.phase_rejoin.params.max_residual_omega =
        augmented_manifest::kAngularOutputMax -
        augmented_manifest::kAngularOutputMin;
    EXPECT_TRUE(validateAndNormalize(config).ok());

    config.phase_rejoin.params.max_residual_v += 1.0e-6;
    const ValidationReport over_bound = validateAndNormalize(config);
    EXPECT_FALSE(over_bound.ok());
    EXPECT_TRUE(hasIssue(
        over_bound, ValidationSeverity::Fatal,
        "phase_rejoin/residual_bounds"));
}

TEST(AppConfig, NormalizationIsDeterministicAndIdempotent) {
    AppConfig config;
    config.map_vref.runtime_override_enable = true;
    config.map_vref.runtime_override_mps = -1.0;
    config.control.delay_phase.history_window_sec = -1.0;
    config.shared_command_limits.angular_rate_max = -1.0;

    const ValidationReport first = validateAndNormalize(config);
    const AppConfig normalized = config;
    const ValidationReport second = validateAndNormalize(config);

    EXPECT_TRUE(first.ok());
    EXPECT_TRUE(second.ok());
    EXPECT_FALSE(config.map_vref.runtime_override_enable);
    EXPECT_EQ(second.issues().size(), 0u);
    EXPECT_DOUBLE_EQ(config.control.delay_phase.history_window_sec,
                     normalized.control.delay_phase.history_window_sec);
    EXPECT_DOUBLE_EQ(config.shared_command_limits.angular_rate_max,
                     normalized.shared_command_limits.angular_rate_max);
    EXPECT_DOUBLE_EQ(config.solver.slosh.dt, normalized.solver.slosh.dt);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
