#include "spmpc_local_planner/core/start_lock_recovery.h"
#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

StartLockRecoveryParams makeEnabledParams() {
    StartLockRecoveryParams params;
    params.enable = true;
    params.detect_only = true;
    params.start_window_s = 0.20;
    params.min_stall_duration_sec = 0.10;
    params.progress_epsilon_s = 0.005;
    params.cmd_v_small_threshold = 0.03;
    params.warm_start_v_s_min = 0.10;
    params.u0_v_s_max = 0.02;
    params.require_monotonic_clip = true;
    params.max_projection_distance_m = 0.50;
    return params;
}

StartLockRecoveryObservation makeLockObservation() {
    StartLockRecoveryObservation obs;
    obs.valid = true;
    obs.status = "B0_ACADOS_OK";
    obs.progress_abs_s = 0.05;
    obs.cmd_v = 0.0;
    obs.robot_v = 0.0;
    obs.raw_projection_valid = true;
    obs.guarded_projection_valid = true;
    obs.projector_raw_s = 0.0;
    obs.projector_guarded_s = 0.05;
    obs.projector_raw_distance = 0.08;
    obs.projector_guarded_distance = 0.08;
    obs.monotonic_clip_applied = true;
    obs.warm_start_v_s_valid = true;
    obs.warm_start_v_s0 = 0.56;
    obs.first_shot_valid = true;
    obs.first_shot_u0_v_s = 0.0;
    return obs;
}

}  // namespace

TEST(StartLockRecovery, DisabledByDefaultDoesNotActivate) {
    StartLockRecovery detector;
    StartLockRecoveryParams params;
    detector.setParams(params);
    for (int i = 0; i < 5; ++i) {
        detector.update(makeLockObservation(), 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_EQ(detector.diagnostics().mode, "DISABLED");
}

TEST(StartLockRecovery, DetectsNearStartLockAfterPersistence) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    detector.update(makeLockObservation(), 0.05);
    EXPECT_FALSE(detector.diagnostics().active);
    detector.update(makeLockObservation(), 0.05);
    EXPECT_TRUE(detector.diagnostics().active);
    EXPECT_EQ(detector.diagnostics().mode, "ACTIVE_START_LOCK");
    EXPECT_TRUE(detector.diagnostics().warmstart_requests_motion);
    EXPECT_TRUE(detector.diagnostics().solver_rejects_progress);
}

TEST(StartLockRecovery, ClearsWhenProgressResumes) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    detector.update(makeLockObservation(), 0.05);
    detector.update(makeLockObservation(), 0.05);
    ASSERT_TRUE(detector.diagnostics().active);

    auto obs = makeLockObservation();
    obs.progress_abs_s = 0.08;
    detector.update(obs, 0.05);
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_EQ(detector.diagnostics().mode, "MONITORING");
    EXPECT_GT(detector.diagnostics().progress_delta_s, detector.params().progress_epsilon_s);
}

TEST(StartLockRecovery, DoesNotTriggerOutsideStartWindow) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    auto obs = makeLockObservation();
    obs.progress_abs_s = 0.30;
    for (int i = 0; i < 3; ++i) {
        detector.update(obs, 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_FALSE(detector.diagnostics().near_start);
}

TEST(StartLockRecovery, DoesNotTriggerWithoutWarmstartVsSolverMismatch) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    auto obs = makeLockObservation();
    obs.warm_start_v_s0 = 0.0;
    for (int i = 0; i < 3; ++i) {
        detector.update(obs, 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_FALSE(detector.diagnostics().warmstart_requests_motion);

    detector.reset();
    obs = makeLockObservation();
    obs.first_shot_u0_v_s = 0.10;
    for (int i = 0; i < 3; ++i) {
        detector.update(obs, 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_FALSE(detector.diagnostics().solver_rejects_progress);
}

TEST(StartLockRecovery, RequiresMonotonicClipWhenConfigured) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    auto obs = makeLockObservation();
    obs.monotonic_clip_applied = false;
    for (int i = 0; i < 3; ++i) {
        detector.update(obs, 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_FALSE(detector.diagnostics().monotonic_clip_active);
}

TEST(StartLockRecovery, UnsafeProjectionDistanceDoesNotActivate) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    auto obs = makeLockObservation();
    obs.projector_guarded_distance = 0.80;
    detector.update(obs, 0.05);
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_TRUE(detector.diagnostics().projection_distance_unsafe);
    EXPECT_EQ(detector.diagnostics().mode, "UNSAFE_PROJECTION_DISTANCE");
}

TEST(StartLockRecovery, GoalReachedDoesNotActivate) {
    StartLockRecovery detector;
    detector.setParams(makeEnabledParams());
    auto obs = makeLockObservation();
    obs.status = "GOAL_REACHED";
    obs.terminal_reached = true;
    for (int i = 0; i < 3; ++i) {
        detector.update(obs, 0.05);
    }
    EXPECT_FALSE(detector.diagnostics().active);
    EXPECT_EQ(detector.diagnostics().mode, "MONITORING");
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
