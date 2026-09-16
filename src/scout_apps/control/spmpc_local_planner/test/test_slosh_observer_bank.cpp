#include "spmpc_local_planner/estimation/slosh_observer_bank.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace spmpc_local_planner {
namespace {

SloshModelParams makeParams(bool use_parabola_term = true) {
    SloshModelParams params;
    params.container_radius = 0.01725;
    params.liquid_height = 0.053;
    params.liquid_density = 997.0;
    params.damping_ratio = 0.047;
    params.mode_index = 1;
    params.dt = 0.031;
    params.use_linear_model = true;
    params.use_parabola_term = use_parabola_term;
    return params;
}

MotionExcitation makeExcitationAt(
    MotionExcitationSource source,
    bool valid,
    double sample_dt_sec,
    double ax,
    double ay,
    double omega_z,
    double alpha_z,
    std::int64_t measurement_stamp_ns,
    std::uint32_t reset_epoch = 0u) {
    MotionExcitation excitation;
    excitation.source = source;
    excitation.valid = valid;
    excitation.ax = ax;
    excitation.ay = ay;
    excitation.omega_z = omega_z;
    excitation.alpha_z = alpha_z;
    excitation.sample_dt_sec = sample_dt_sec;
    excitation.measurement_stamp_ns = measurement_stamp_ns;
    excitation.source_stamp_ns = excitation.measurement_stamp_ns + 15000LL;
    excitation.accel_effective_stamp_ns = excitation.measurement_stamp_ns - 6834LL;
    excitation.gyro_effective_stamp_ns = excitation.measurement_stamp_ns - 5020LL;
    excitation.alpha_effective_stamp_ns = excitation.measurement_stamp_ns - 15001LL;
    excitation.receive_stamp_ns = excitation.source_stamp_ns + 200000LL;
    excitation.reset_epoch = reset_epoch;
    return excitation;
}

MotionExcitation makeExcitation(
    MotionExcitationSource source,
    bool valid,
    double sample_dt_sec,
    double ax,
    double ay,
    double omega_z,
    double alpha_z,
    std::int64_t sequence,
    std::uint32_t reset_epoch = 0u) {
    return makeExcitationAt(source, valid, sample_dt_sec, ax, ay, omega_z,
                            alpha_z, 1000000000LL + sequence * 1000000LL,
                            reset_epoch);
}

void expectStateExactlyEqual(const SloshState& actual, const SloshState& expected) {
    EXPECT_EQ(actual.eta_x, expected.eta_x);
    EXPECT_EQ(actual.eta_x_dot, expected.eta_x_dot);
    EXPECT_EQ(actual.eta_y, expected.eta_y);
    EXPECT_EQ(actual.eta_y_dot, expected.eta_y_dot);
}

void expectStateNear(const SloshState& actual,
                     const SloshState& expected,
                     double tolerance = 1e-13) {
    EXPECT_NEAR(actual.eta_x, expected.eta_x, tolerance);
    EXPECT_NEAR(actual.eta_x_dot, expected.eta_x_dot, tolerance);
    EXPECT_NEAR(actual.eta_y, expected.eta_y, tolerance);
    EXPECT_NEAR(actual.eta_y_dot, expected.eta_y_dot, tolerance);
}

SloshState independentlyDiscretizedStep(
    const SloshModelParams& base_params,
    const SloshState& state,
    const MotionExcitation& excitation) {
    SloshModelParams step_params = base_params;
    step_params.dt = excitation.sample_dt_sec;
    SloshDynamics dynamics;
    EXPECT_TRUE(dynamics.configure(step_params));
    SloshState next;
    EXPECT_TRUE(dynamics.stepWithDt(state, excitation.atContainer(), step_params.dt, next));
    return next;
}

void expectExcitationExactlyEqual(
    const MotionExcitation& actual,
    const MotionExcitation& expected) {
    EXPECT_EQ(actual.source, expected.source);
    EXPECT_EQ(actual.valid, expected.valid);
    EXPECT_EQ(actual.ax, expected.ax);
    EXPECT_EQ(actual.ay, expected.ay);
    EXPECT_EQ(actual.omega_z, expected.omega_z);
    EXPECT_EQ(actual.alpha_z, expected.alpha_z);
    EXPECT_EQ(actual.sample_dt_sec, expected.sample_dt_sec);
    EXPECT_EQ(actual.source_stamp_ns, expected.source_stamp_ns);
    EXPECT_EQ(actual.measurement_stamp_ns, expected.measurement_stamp_ns);
    EXPECT_EQ(actual.accel_effective_stamp_ns, expected.accel_effective_stamp_ns);
    EXPECT_EQ(actual.gyro_effective_stamp_ns, expected.gyro_effective_stamp_ns);
    EXPECT_EQ(actual.alpha_effective_stamp_ns, expected.alpha_effective_stamp_ns);
    EXPECT_EQ(actual.receive_stamp_ns, expected.receive_stamp_ns);
    EXPECT_EQ(actual.reset_epoch, expected.reset_epoch);
}

void expectSnapshotExactlyEqual(
    const SloshObserverSnapshot& actual,
    const SloshObserverSnapshot& expected) {
    EXPECT_EQ(actual.configured, expected.configured);
    EXPECT_EQ(actual.valid, expected.valid);
    expectStateExactlyEqual(actual.state, expected.state);
    expectExcitationExactlyEqual(actual.excitation, expected.excitation);
    EXPECT_EQ(actual.state_stamp_ns, expected.state_stamp_ns);
    EXPECT_EQ(actual.update_count, expected.update_count);
    EXPECT_EQ(actual.modal_height_m, expected.modal_height_m);
    EXPECT_EQ(actual.total_height_m, expected.total_height_m);
}

void expectZeroState(const SloshState& state) {
    EXPECT_EQ(state.eta_x, 0.0);
    EXPECT_EQ(state.eta_x_dot, 0.0);
    EXPECT_EQ(state.eta_y, 0.0);
    EXPECT_EQ(state.eta_y_dot, 0.0);
}

// Compare against direct propagation at each supplied dt, including changes
// below the old discrete-matrix reconfiguration threshold.
SloshState directOdomStep(
    SloshDynamics& dynamics,
    const SloshState& state,
    const MotionExcitation& excitation) {
    SloshState next;
    EXPECT_TRUE(dynamics.stepWithDt(state, excitation.atContainer(), excitation.sample_dt_sec, next));
    return next;
}

TEST(SloshObserverBank, OdomVariableDtUsesEveryActualSampleInterval) {
    const SloshModelParams params = makeParams();
    SloshObserverBank bank;
    ASSERT_TRUE(bank.configure(params, 0.02));

    SloshDynamics legacy_dynamics;
    ASSERT_TRUE(legacy_dynamics.configure(params));
    SloshState legacy_state;

    const std::vector<MotionExcitation> sequence = {
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.031, 0.31, -0.17,
                         0.40, 0.0, 1000000000LL),
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.020, -0.28, 0.23,
                         -0.35, 1.2, 1020000000LL),
        // Inside the old 1e-4 tolerance: v1 must still use this exact interval.
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.02005, 0.11, 0.07,
                         0.20, -0.8, 1040050000LL),
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.047, -0.42, -0.19,
                         0.73, 2.1, 1087050000LL),
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.031, 0.05, 0.38,
                         -0.61, -1.4, 1118050000LL),
    };

    for (std::size_t i = 0; i < sequence.size(); ++i) {
        SCOPED_TRACE(i);
        legacy_state = directOdomStep(legacy_dynamics, legacy_state, sequence[i]);
        ASSERT_TRUE(bank.stepOdom(sequence[i]));

        expectStateExactlyEqual(bank.solverState(), legacy_state);
        expectStateExactlyEqual(bank.odom().state, legacy_state);
        expectExcitationExactlyEqual(bank.odom().excitation, sequence[i]);
        EXPECT_TRUE(bank.odom().configured);
        EXPECT_TRUE(bank.odom().valid);
        EXPECT_EQ(bank.odom().state_stamp_ns, sequence[i].measurement_stamp_ns);
        EXPECT_EQ(bank.odom().update_count, static_cast<std::uint64_t>(i + 1u));
        EXPECT_EQ(bank.odom().modal_height_m,
                  legacy_dynamics.heightCoeff() * legacy_dynamics.etaNorm(legacy_state));
        EXPECT_EQ(bank.odom().total_height_m,
                  legacy_dynamics.height(legacy_state, sequence[i].omega_z));
    }
}

TEST(SloshObserverBank, OdomRejectsUnsafeInputWithoutAdvancingSolverState) {
    const SloshModelParams params = makeParams();
    SloshObserverBank bank;
    SloshObserverBank clean_reference;
    ASSERT_TRUE(bank.configure(params, 0.02));
    ASSERT_TRUE(clean_reference.configure(params, 0.02));

    const MotionExcitation first = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.31, -0.17,
        0.40, 0.0, 100);
    ASSERT_TRUE(bank.stepOdom(first));
    ASSERT_TRUE(clean_reference.stepOdom(first));
    const SloshState state_before = bank.solverState();
    const std::uint64_t count_before = bank.odom().update_count;
    const std::int64_t stamp_before = bank.odom().state_stamp_ns;
    const double modal_height_before = bank.odom().modal_height_m;
    const double total_height_before = bank.odom().total_height_m;

    std::vector<MotionExcitation> rejected_inputs;
    MotionExcitation invalid_flag = makeExcitation(
        MotionExcitationSource::Odom, false, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    rejected_inputs.push_back(invalid_flag);

    MotionExcitation wrong_source = makeExcitation(
        MotionExcitationSource::ProcessedImu, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    rejected_inputs.push_back(wrong_source);

    MotionExcitation nonfinite = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    nonfinite.ax = std::numeric_limits<double>::quiet_NaN();
    rejected_inputs.push_back(nonfinite);

    nonfinite = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    nonfinite.ay = std::numeric_limits<double>::infinity();
    rejected_inputs.push_back(nonfinite);

    nonfinite = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    nonfinite.omega_z = std::numeric_limits<double>::quiet_NaN();
    rejected_inputs.push_back(nonfinite);

    nonfinite = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    nonfinite.alpha_z = std::numeric_limits<double>::infinity();
    rejected_inputs.push_back(nonfinite);

    nonfinite = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    nonfinite.sample_dt_sec = std::numeric_limits<double>::quiet_NaN();
    rejected_inputs.push_back(nonfinite);

    MotionExcitation too_small_dt = makeExcitation(
        MotionExcitationSource::Odom, true, 1e-4, 0.1, 0.2,
        0.3, 0.4, 100);
    too_small_dt.sample_dt_sec = 1e-4;
    rejected_inputs.push_back(too_small_dt);

    MotionExcitation zero_stamp = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, 0.1, 0.2,
        0.3, 0.4, 100);
    zero_stamp.source_stamp_ns = 0;
    zero_stamp.measurement_stamp_ns = 0;
    rejected_inputs.push_back(zero_stamp);

    MotionExcitation duplicate = first;
    duplicate.ax = 9.0;
    rejected_inputs.push_back(duplicate);

    MotionExcitation out_of_order = first;
    out_of_order.source_stamp_ns -= 1000000LL;
    out_of_order.measurement_stamp_ns -= 1000000LL;
    rejected_inputs.push_back(out_of_order);

    for (std::size_t i = 0; i < rejected_inputs.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_FALSE(bank.stepOdom(rejected_inputs[i]));
        expectStateExactlyEqual(bank.solverState(), state_before);
        EXPECT_EQ(bank.odom().update_count, count_before);
        EXPECT_EQ(bank.odom().state_stamp_ns, stamp_before);
        EXPECT_EQ(bank.odom().modal_height_m, modal_height_before);
        EXPECT_EQ(bank.odom().total_height_m, total_height_before);
        EXPECT_TRUE(bank.odomConfigured());
        EXPECT_TRUE(bank.odom().configured);
        EXPECT_FALSE(bank.odom().valid);
    }

    const MotionExcitation recovered = makeExcitation(
        MotionExcitationSource::Odom, true, 0.043, -0.27, 0.34,
        -0.52, -1.5, 143);
    ASSERT_TRUE(bank.stepOdom(recovered));
    ASSERT_TRUE(clean_reference.stepOdom(recovered));
    expectSnapshotExactlyEqual(bank.odom(), clean_reference.odom());
    expectStateExactlyEqual(bank.solverState(), clean_reference.solverState());

    const MotionExcitation imu_before_reset = makeExcitation(
        MotionExcitationSource::ProcessedImu, true, 0.02, 0.3, -0.2,
        0.4, 0.5, 121, 9u);
    ASSERT_TRUE(bank.stepImu(imu_before_reset));
    const SloshObserverSnapshot imu_snapshot_before_reset = bank.imu();

    // The ROS boundary uses resetOdom() only for a large source-clock reset;
    // after that explicit epoch boundary a lower timestamp is valid again.
    bank.resetOdom();
    EXPECT_TRUE(bank.odomConfigured());
    EXPECT_TRUE(bank.odom().configured);
    EXPECT_FALSE(bank.odom().valid);
    EXPECT_EQ(bank.odom().update_count, 0u);
    EXPECT_EQ(bank.odom().state_stamp_ns, 0);
    EXPECT_EQ(bank.odom().modal_height_m, 0.0);
    EXPECT_EQ(bank.odom().total_height_m, 0.0);
    expectZeroState(bank.solverState());
    expectSnapshotExactlyEqual(bank.imu(), imu_snapshot_before_reset);

    MotionExcitation first_new_epoch = makeExcitation(
        MotionExcitationSource::Odom, true, 0.031, -0.2, 0.4,
        -0.5, 0.7, 1);
    ASSERT_TRUE(bank.stepOdom(first_new_epoch));
    EXPECT_EQ(bank.odom().update_count, 1u);
    EXPECT_EQ(bank.odom().state_stamp_ns, first_new_epoch.measurement_stamp_ns);
}

TEST(SloshObserverBank, OdomIntegrationFailureBlocksShortSampleRecoveryUntilExplicitInit) {
    const SloshModelParams params = makeParams();
    SloshObserverBank bank;
    ASSERT_TRUE(bank.configure(params, 0.02));

    for (std::int64_t sequence = 1; sequence <= 30; ++sequence) {
        ASSERT_TRUE(bank.stepOdom(makeExcitationAt(
            MotionExcitationSource::Odom, true, 0.02, 0.0, 0.0, 0.0, 0.0,
            1000000000LL + (sequence - 1) * 20000000LL)));
    }
    const SloshObserverSnapshot before_gap = bank.odom();
    const MotionExcitation gap = makeExcitationAt(
        MotionExcitationSource::Odom, true, 1.2, 0.0, 0.0, 0.0, 0.0,
        2780000000LL);
    EXPECT_FALSE(bank.stepOdom(gap));
    EXPECT_FALSE(bank.odom().valid);
    EXPECT_TRUE(bank.odomNeedsInitialization());
    expectStateExactlyEqual(bank.odom().state, before_gap.state);

    // This models the next ROS packet after its derivative baseline was
    // advanced across the rejected gap.  Its reported dt is short, but the
    // liquid state is still discontinuous and must remain invalid.
    const MotionExcitation short_resume = makeExcitationAt(
        MotionExcitationSource::Odom, true, 0.02, 0.0, 0.0, 0.0, 0.0,
        2780000000LL);
    EXPECT_FALSE(bank.stepOdom(short_resume));
    EXPECT_FALSE(bank.odom().valid);
    EXPECT_TRUE(bank.odomNeedsInitialization());

    SloshState trusted_zero;
    ASSERT_TRUE(bank.initializeOdom(trusted_zero, short_resume));
    EXPECT_TRUE(bank.odom().valid);
    EXPECT_FALSE(bank.odomNeedsInitialization());
    const MotionExcitation next = makeExcitationAt(
        MotionExcitationSource::Odom, true, 0.02, 0.1, -0.2, 0.0, 0.0,
        2800000000LL);
    EXPECT_TRUE(bank.stepOdom(next));
    EXPECT_TRUE(bank.odom().valid);
}

TEST(SloshObserverBank, ImuShadowTrafficCannotMutateOdomSnapshotOrSolverState) {
    const SloshModelParams params = makeParams();
    SloshObserverBank bank;
    SloshObserverBank odom_only_bank;
    ASSERT_TRUE(bank.configure(params, 0.02));
    ASSERT_TRUE(odom_only_bank.configure(params, 0.02));

    const MotionExcitation first_odom = makeExcitation(
        MotionExcitationSource::Odom, true, 0.029, 0.36, -0.22, 0.48, 1.0, 10);
    ASSERT_TRUE(bank.stepOdom(first_odom));
    ASSERT_TRUE(odom_only_bank.stepOdom(first_odom));
    const SloshObserverSnapshot odom_before_shadow = bank.odom();
    const SloshState solver_before_shadow = bank.solverState();

    MotionExcitation valid_imu = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.017, -0.81, 0.43,
        -0.72, 3.0, 1027000000LL, 7u);
    ASSERT_TRUE(bank.stepImu(valid_imu));
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    MotionExcitation invalid_imu = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, false, 0.023, 4.0, -5.0,
        2.0, -9.0, 1027000000LL, 7u);
    EXPECT_FALSE(bank.stepImu(invalid_imu));
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    MotionExcitation wrong_source = valid_imu;
    wrong_source.source = MotionExcitationSource::Odom;
    EXPECT_FALSE(bank.stepImu(wrong_source));
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    MotionExcitation nonfinite_imu = valid_imu;
    nonfinite_imu.ax = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(bank.stepImu(nonfinite_imu));
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    bank.invalidateImu(8u);
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    valid_imu.reset_epoch = 8u;
    valid_imu.source_stamp_ns += 3000000LL;
    ASSERT_TRUE(bank.initializeImu(SloshState{}, valid_imu));
    expectSnapshotExactlyEqual(bank.odom(), odom_before_shadow);
    expectStateExactlyEqual(bank.solverState(), solver_before_shadow);

    const std::vector<MotionExcitation> remaining_odom = {
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.043, -0.27, 0.34,
                         -0.52, -1.5, 1053000000LL),
        makeExcitationAt(MotionExcitationSource::Odom, true, 0.018, 0.19, 0.12,
                         0.67, 2.4, 1071000000LL),
    };
    for (std::size_t i = 0; i < remaining_odom.size(); ++i) {
        SCOPED_TRACE(i);
        ASSERT_TRUE(bank.stepOdom(remaining_odom[i]));
        ASSERT_TRUE(odom_only_bank.stepOdom(remaining_odom[i]));
        expectSnapshotExactlyEqual(bank.odom(), odom_only_bank.odom());
        expectStateExactlyEqual(bank.solverState(), odom_only_bank.solverState());
    }
}

TEST(SloshObserverBank, ImuUsesExactReportedDtAndMatchesIndependentDiscretization) {
    const SloshModelParams params = makeParams();
    const double observer_dt_sec = 0.02;
    SloshObserverBank bank;
    ASSERT_TRUE(bank.configure(params, observer_dt_sec));

    SloshState reference_state;

    // Deliberately vary the reported message dt.  Each shadow step must match a
    // freshly configured, independent exact-ZOH dynamics instance at that dt.
    const std::vector<MotionExcitation> sequence = {
        makeExcitation(MotionExcitationSource::ProcessedImu, true, 0.006, 0.42,
                       -0.31, 0.5, 1.0, 20, 42u),
        makeExcitation(MotionExcitationSource::ProcessedImu, true, 0.100, -0.27,
                       0.18, -0.7, -2.0, 120, 42u),
        makeExcitation(MotionExcitationSource::ProcessedImu, true, 0.019, 0.09,
                       0.36, 1.1, 0.4, 139, 42u),
        makeExcitation(MotionExcitationSource::ProcessedImu, true, 0.500, -0.54,
                       -0.11, -0.9, 3.2, 639, 42u),
    };

    for (std::size_t i = 0; i < sequence.size(); ++i) {
        SCOPED_TRACE(i);
        reference_state = independentlyDiscretizedStep(
            params, reference_state, sequence[i]);
        ASSERT_TRUE(bank.stepImu(sequence[i]));

        expectStateNear(bank.imu().state, reference_state);
        expectExcitationExactlyEqual(bank.imu().excitation, sequence[i]);
        EXPECT_TRUE(bank.imu().configured);
        EXPECT_TRUE(bank.imu().valid);
        EXPECT_EQ(bank.imu().update_count, static_cast<std::uint64_t>(i + 1u));
        EXPECT_EQ(bank.imu().state_stamp_ns, sequence[i].accel_effective_stamp_ns);
        EXPECT_NEAR(bank.imu().modal_height_m,
                    bank.heightCoeff() * std::hypot(reference_state.eta_x,
                                                     reference_state.eta_y),
                    1e-13);
        EXPECT_NEAR(bank.imu().total_height_m,
                    bank.solverHeight(reference_state, sequence[i].omega_z),
                    1e-13);
    }
}

TEST(SloshObserverBank, InvalidImuNeverStepsAndNewEpochClearsAllImuState) {
    const SloshModelParams params = makeParams();
    const double observer_dt_sec = 0.02;
    SloshObserverBank bank;
    ASSERT_TRUE(bank.configure(params, observer_dt_sec));

    SloshModelParams reference_params = params;
    reference_params.dt = observer_dt_sec;
    SloshDynamics reference_dynamics;
    ASSERT_TRUE(reference_dynamics.configure(reference_params));
    SloshState reference_state;

    const MotionExcitation first = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.02, 0.47, -0.29,
        0.55, 1.2, 1030000000LL, 3u);
    const MotionExcitation second = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.02, -0.21, 0.38,
        -0.44, -0.9, 1050000000LL, 3u);
    ASSERT_TRUE(reference_dynamics.stepWithDt(reference_state, first.atContainer(), observer_dt_sec, reference_state));
    ASSERT_TRUE(bank.stepImu(first));
    ASSERT_TRUE(reference_dynamics.stepWithDt(reference_state, second.atContainer(), observer_dt_sec, reference_state));
    ASSERT_TRUE(bank.stepImu(second));
    expectStateExactlyEqual(bank.imu().state, reference_state);
    ASSERT_EQ(bank.imu().update_count, 2u);

    const SloshState state_before_invalid = bank.imu().state;
    const std::int64_t stamp_before_invalid = bank.imu().state_stamp_ns;
    MotionExcitation invalid = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, false, 0.02, 99.0, -88.0,
        7.0, 6.0, 1050000000LL, 3u);
    EXPECT_FALSE(bank.stepImu(invalid));
    EXPECT_FALSE(bank.imu().valid);
    EXPECT_EQ(bank.imu().update_count, 2u);
    EXPECT_EQ(bank.imu().state_stamp_ns, stamp_before_invalid);
    expectStateExactlyEqual(bank.imu().state, state_before_invalid);

    MotionExcitation nonfinite = invalid;
    nonfinite.valid = true;
    nonfinite.ay = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(bank.stepImu(nonfinite));
    EXPECT_EQ(bank.imu().update_count, 2u);
    EXPECT_EQ(bank.imu().state_stamp_ns, stamp_before_invalid);
    expectStateExactlyEqual(bank.imu().state, state_before_invalid);

    // A later valid sample advances exactly once from the pre-invalid state.
    // If either invalid callback performed a hold-last step, this equality fails.
    const MotionExcitation after_invalid = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.02, 0.16, 0.24,
        0.31, -0.5, 1070000000LL, 3u);
    ASSERT_TRUE(reference_dynamics.stepWithDt(reference_state, after_invalid.atContainer(), observer_dt_sec, reference_state));
    ASSERT_TRUE(bank.stepImu(after_invalid));
    EXPECT_EQ(bank.imu().update_count, 3u);
    expectStateExactlyEqual(bank.imu().state, reference_state);

    // ProcessedImuPipeline increments reset_epoch on a sample gap. The ROS
    // adapter passes that epoch to invalidateImu(), which must clear history.
    bank.invalidateImu(4u);
    EXPECT_TRUE(bank.imu().configured);
    EXPECT_FALSE(bank.imu().valid);
    EXPECT_EQ(bank.imu().update_count, 0u);
    EXPECT_EQ(bank.imu().state_stamp_ns, 0);
    EXPECT_EQ(bank.imu().modal_height_m, 0.0);
    EXPECT_EQ(bank.imu().total_height_m, 0.0);
    expectZeroState(bank.imu().state);

    const MotionExcitation first_after_gap = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.3, -0.33, 0.27,
        -0.62, 1.4, 1080000000LL, 4u);
    SloshState zero_state;
    // A new input epoch only rewarms the signal path.  It cannot authorize a
    // zero liquid state or make a short post-gap sample valid by itself.
    EXPECT_TRUE(bank.imuNeedsInitialization());
    EXPECT_FALSE(bank.stepImu(first_after_gap));
    EXPECT_FALSE(bank.imu().valid);
    ASSERT_TRUE(bank.initializeImu(zero_state, first_after_gap));
    EXPECT_EQ(bank.imu().update_count, 1u);
    expectStateExactlyEqual(bank.imu().state, zero_state);
    EXPECT_TRUE(bank.imu().valid);

    const MotionExcitation after_initialization = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, true, 0.02, 0.16, 0.24,
        0.31, -0.5, 1100000000LL, 4u);
    ASSERT_TRUE(bank.stepImu(after_initialization));
    EXPECT_EQ(bank.imu().update_count, 2u);

    // The epoch check happens before input validation, so an invalid first
    // sample in a new epoch must still clear the previous epoch's state.
    MotionExcitation invalid_new_epoch = makeExcitationAt(
        MotionExcitationSource::ProcessedImu, false, 0.02, 5.0, 6.0,
        0.8, 0.9, 1100000000LL, 5u);
    EXPECT_FALSE(bank.stepImu(invalid_new_epoch));
    EXPECT_TRUE(bank.imu().configured);
    EXPECT_FALSE(bank.imu().valid);
    EXPECT_EQ(bank.imu().update_count, 0u);
    EXPECT_EQ(bank.imu().state_stamp_ns, 0);
    EXPECT_EQ(bank.imu().modal_height_m, 0.0);
    EXPECT_EQ(bank.imu().total_height_m, 0.0);
    expectZeroState(bank.imu().state);
}

TEST(SloshObserverBank, SolverHeightRetainsLegacyOdomDynamicsSemantics) {
    const std::vector<bool> parabola_options = {false, true};
    for (std::size_t option = 0; option < parabola_options.size(); ++option) {
        SCOPED_TRACE(option);
        const SloshModelParams params = makeParams(parabola_options[option]);
        SloshObserverBank bank;
        SloshDynamics legacy_dynamics;
        ASSERT_TRUE(bank.configure(params, 0.02));
        ASSERT_TRUE(legacy_dynamics.configure(params));

        const std::vector<SloshState> states = {
            SloshState(),
            SloshState{0.0012, -0.031, -0.0007, 0.024},
            SloshState{-0.0021, 0.018, 0.0018, -0.046},
        };
        const std::vector<double> omega_values = {0.0, 0.83, -1.37};
        for (std::size_t i = 0; i < states.size(); ++i) {
            SCOPED_TRACE(i);
            EXPECT_EQ(bank.heightCoeff(), legacy_dynamics.heightCoeff());
            EXPECT_EQ(bank.solverHeight(states[i], omega_values[i]),
                      legacy_dynamics.height(states[i], omega_values[i]));
        }

        const MotionExcitation odom = makeExcitation(
            MotionExcitationSource::Odom, true, 0.046, 0.39, -0.26,
            1.12, -1.7, static_cast<std::int64_t>(40u + option));
        SloshState legacy_state;
        legacy_state = directOdomStep(legacy_dynamics, legacy_state, odom);
        ASSERT_TRUE(bank.stepOdom(odom));
        expectStateExactlyEqual(bank.solverState(), legacy_state);
        EXPECT_EQ(bank.solverHeight(bank.solverState(), odom.omega_z),
                  legacy_dynamics.height(legacy_state, odom.omega_z));
        EXPECT_EQ(bank.odom().total_height_m,
                  bank.solverHeight(bank.solverState(), odom.omega_z));
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
