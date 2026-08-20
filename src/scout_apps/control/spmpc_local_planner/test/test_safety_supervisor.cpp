#include <gtest/gtest.h>

#include "spmpc_local_planner/safety/safety_supervisor.h"

#include <limits>

namespace spmpc_local_planner {
namespace {

SafetySupervisor configuredSupervisor(
    const SafetySupervisorConfig& config = SafetySupervisorConfig{}) {
    SafetySupervisor supervisor;
    std::string error;
    EXPECT_TRUE(supervisor.configure(config, error)) << error;
    return supervisor;
}

SafetySupervisorInput acceptedTrackingInput() {
    SafetySupervisorInput input;
    input.command_accepted = true;
    input.command.linear = 0.4;
    input.status = "OK";
    input.period_sec = 0.1;
    return input;
}

TEST(SafetySupervisorTest, RejectsInvalidConfiguration) {
    SafetySupervisor supervisor;
    SafetySupervisorConfig config;
    config.nominal_period_sec =
        std::numeric_limits<double>::quiet_NaN();
    std::string error;
    EXPECT_FALSE(supervisor.configure(config, error));
    EXPECT_FALSE(error.empty());
}

TEST(SafetySupervisorTest, TerminalSpinLatchesAtBoundaryAndResetsWhenCalm) {
    SafetySupervisorConfig config;
    config.terminal_spin.omega_threshold = 0.2;
    config.terminal_spin.max_duration_sec = 0.3;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.terminal.terminal_phase = true;
    input.command.angular = 0.21;
    EXPECT_FALSE(supervisor.step(input).blocked);
    EXPECT_FALSE(supervisor.step(input).blocked);
    const SafetySupervisorResult latched = supervisor.step(input);
    EXPECT_TRUE(latched.blocked);
    EXPECT_TRUE(latched.terminal_spin_blocked);
    EXPECT_EQ(SafetyIntervention::TerminalSpin, latched.intervention);
    EXPECT_EQ("TERMINAL_SPIN_FAIL", latched.status);

    input.command.angular = 0.2;
    const SafetySupervisorResult reset = supervisor.step(input);
    EXPECT_FALSE(reset.blocked);
    EXPECT_FALSE(reset.terminal_spin_latched);
    EXPECT_DOUBLE_EQ(0.0, reset.terminal_spin_duration_sec);
}

TEST(SafetySupervisorTest, InvalidPeriodUsesConfiguredNominalPeriod) {
    SafetySupervisorConfig config;
    config.nominal_period_sec = 0.2;
    config.terminal_spin.max_duration_sec = 0.4;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.terminal.terminal_phase = true;
    input.command.angular = 1.0;
    input.period_sec = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(supervisor.step(input).blocked);
    EXPECT_TRUE(supervisor.step(input).blocked);
}

TEST(SafetySupervisorTest, ProjectionDurationMustBeConsecutive) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.max_projection_distance_m = 0.5;
    config.tracking.max_projection_duration_sec = 0.2;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.projection.raw_valid = true;
    input.projection.raw_distance_m = 0.6;
    EXPECT_FALSE(supervisor.step(input).blocked);
    input.projection.raw_distance_m = 0.4;
    EXPECT_FALSE(supervisor.step(input).blocked);
    EXPECT_DOUBLE_EQ(
        0.0, supervisor.step(input).tracking_projection_duration_sec);
    input.projection.raw_distance_m = 0.6;
    EXPECT_FALSE(supervisor.step(input).blocked);
    const SafetySupervisorResult latched = supervisor.step(input);
    EXPECT_TRUE(latched.tracking_safety_blocked);
    EXPECT_EQ(SafetyIntervention::TrackingProjection,
              latched.intervention);
}

TEST(SafetySupervisorTest, GuardedProjectionHasHistoricalPriority) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.max_projection_duration_sec = 0.1;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.projection.raw_valid = true;
    input.projection.raw_distance_m = 0.1;
    input.projection.guarded_valid = true;
    input.projection.guarded_distance_m = 0.8;
    const SafetySupervisorResult result = supervisor.step(input);
    EXPECT_TRUE(result.blocked);
    EXPECT_EQ(SafetyIntervention::TrackingProjection,
              result.intervention);
}

TEST(SafetySupervisorTest, TrackingSpinIsDisabledInTerminalPhase) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.projection_enable = false;
    config.tracking.spin_max_duration_sec = 0.1;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.terminal.terminal_phase = true;
    input.command.angular = 1.0;
    EXPECT_FALSE(supervisor.step(input).blocked);
    input.terminal.terminal_phase = false;
    const SafetySupervisorResult result = supervisor.step(input);
    EXPECT_TRUE(result.blocked);
    EXPECT_EQ(SafetyIntervention::TrackingSpin, result.intervention);
}

TEST(SafetySupervisorTest, ExistingTrackingLatchSurvivesFailedCommands) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.spin_enable = false;
    config.tracking.max_projection_duration_sec = 0.1;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.projection.raw_valid = true;
    input.projection.raw_distance_m = 0.8;
    EXPECT_TRUE(supervisor.step(input).blocked);
    input.command_accepted = false;
    input.projection.raw_distance_m = 0.0;
    const SafetySupervisorResult result = supervisor.step(input);
    EXPECT_TRUE(result.blocked);
    EXPECT_EQ("TRACKING_UNSAFE_PROJECTION", result.status);
}

TEST(SafetySupervisorTest, GoalReachedClearsTrackingLatches) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.spin_enable = false;
    config.tracking.max_projection_duration_sec = 0.1;
    SafetySupervisor supervisor = configuredSupervisor(config);

    SafetySupervisorInput input = acceptedTrackingInput();
    input.projection.raw_valid = true;
    input.projection.raw_distance_m = 0.8;
    EXPECT_TRUE(supervisor.step(input).blocked);
    input.terminal.reached = true;
    input.status = "GOAL_REACHED";
    const SafetySupervisorResult result = supervisor.step(input);
    EXPECT_FALSE(result.blocked);
    EXPECT_FALSE(result.tracking_projection_latched);
}

TEST(SafetySupervisorTest, ExplicitResetClearsAllLatches) {
    SafetySupervisorConfig config;
    config.terminal_spin.enable = false;
    config.tracking.spin_enable = false;
    config.tracking.max_projection_duration_sec = 0.1;
    SafetySupervisor supervisor = configuredSupervisor(config);
    SafetySupervisorInput input = acceptedTrackingInput();
    input.projection.raw_valid = true;
    input.projection.raw_distance_m = 0.8;
    EXPECT_TRUE(supervisor.step(input).blocked);

    supervisor.reset();
    input.projection.raw_distance_m = 0.0;
    const SafetySupervisorResult result = supervisor.step(input);
    EXPECT_FALSE(result.blocked);
    EXPECT_FALSE(result.tracking_projection_latched);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
