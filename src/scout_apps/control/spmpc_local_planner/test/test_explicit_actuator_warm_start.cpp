#include "spmpc_local_planner/warm_start/explicit_actuator_warm_start.h"
#include "spmpc_local_planner/warm_start/diff_drive_flatness_warm_start.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

class ExplicitActuatorWarmStart : public testing::Test {
protected:
    void SetUp() override {
        std::vector<TrajectoryPoint> points(101);
        for (size_t k = 0; k < points.size(); ++k) points[k].x = k * .05;
        reference.setPoints(points, "test");
        spline.build(reference);
        input.reference = &reference;
        input.spline = &spline;
        input.reference_length = reference.length();
        input.horizon_steps = 60;
        input.bounds.omega_rate_max = 1.2;
        input.robot = {0, 0, 0, .1, .2};
        input.slosh = {.001, .002, -.001, .003};
        input.config.use_slosh_rollout = false;
        input.config.max_reference_fit_error = 1e-6;
        ASSERT_TRUE(liquid.configure(input.slosh_params));
        input.slosh_dynamics = &liquid;
        actuator.valid = true;
        actuator.v_cmd = .15;
        actuator.omega_cmd = .25;
        actuator.linear_delay_queue.fill(.15);
        actuator.angular_delay_queue.fill(.25);
    }

    WarmStartOutput candidate() {
        DiffDriveFlatnessWarmStart generator;
        WarmStartOutput output;
        WarmStartDiagnostics diagnostics;
        EXPECT_TRUE(generator.generate(input, output, diagnostics));
        return output;
    }

    ReferencePath reference;
    ReferenceSpline spline;
    WarmStartInput input;
    ActuatorState actuator;
    ActuatorModelParams params;
    SloshDynamics liquid;
};

TEST_F(ExplicitActuatorWarmStart, GeometryControlsDoNotDependOnProvisionalLiquidRollout) {
    const auto geometry = candidate();
    input.config.use_slosh_rollout = true;
    const auto provisional = candidate();
    ASSERT_EQ(geometry.controls.size(), provisional.controls.size());
    for (size_t k = 0; k < geometry.controls.size(); ++k) {
        EXPECT_DOUBLE_EQ(geometry.controls[k].a, provisional.controls[k].a);
        EXPECT_DOUBLE_EQ(geometry.controls[k].alpha, provisional.controls[k].alpha);
        EXPECT_DOUBLE_EQ(geometry.controls[k].v_s, provisional.controls[k].v_s);
        EXPECT_DOUBLE_EQ(geometry.states[k].px, provisional.states[k].px);
        EXPECT_DOUBLE_EQ(geometry.states[k].omega, provisional.states[k].omega);
    }
    EXPECT_FALSE(geometry.diagnostics.used_slosh_rollout);
    EXPECT_TRUE(provisional.diagnostics.used_slosh_rollout);
    EXPECT_NE(geometry.states.back().eta_x, provisional.states.back().eta_x);
}

TEST_F(ExplicitActuatorWarmStart, FinalMetricsReplaceCandidateMetricsWithoutRejectingFit) {
    auto output = candidate();
    const auto controls = output.controls;
    output.diagnostics.max_v = output.diagnostics.max_omega = 999;
    output.diagnostics.max_a = output.diagnostics.max_lateral_acc = 999;
    output.diagnostics.max_slosh_height_pred = output.diagnostics.reference_fit_error = 999;
    output.diagnostics.bound_violation_count = 999;
    ASSERT_TRUE(rolloutExplicitActuatorWarmStart(output, input, actuator, params, liquid, true));
    EXPECT_TRUE(output.diagnostics.used_flatness);
    EXPECT_TRUE(output.diagnostics.used_slosh_rollout);
    double v = 0, omega = 0, a = 0, lateral = 0, h = 0, fit = 0;
    int violations = 0;
    for (size_t k = 0; k < output.states.size(); ++k) {
        const auto& state = output.states[k];
        v = std::max(v, std::abs(state.v));
        omega = std::max(omega, std::abs(state.omega));
        lateral = std::max(lateral, std::abs(state.v * state.omega));
        h = std::max(h, liquid.heightCoeff() * std::hypot(state.eta_x, state.eta_y));
        if (k) fit = std::max(fit, std::hypot(state.px - state.s, state.py));
        violations += state.v < -1e-9 || state.v > input.bounds.v_max + 1e-9 ||
                      std::abs(state.omega) > input.bounds.omega_max + 1e-9;
    }
    for (size_t k = 0; k < controls.size(); ++k) {
        EXPECT_DOUBLE_EQ(output.controls[k].a, controls[k].a);
        EXPECT_DOUBLE_EQ(output.controls[k].alpha, controls[k].alpha);
        EXPECT_DOUBLE_EQ(output.controls[k].v_s, controls[k].v_s);
        a = std::max(a, std::abs(controls[k].a));
    }
    EXPECT_DOUBLE_EQ(output.diagnostics.max_v, v);
    EXPECT_DOUBLE_EQ(output.diagnostics.max_omega, omega);
    EXPECT_DOUBLE_EQ(output.diagnostics.max_a, a);
    EXPECT_DOUBLE_EQ(output.diagnostics.max_lateral_acc, lateral);
    EXPECT_NEAR(output.diagnostics.max_slosh_height_pred, h, 1e-15);
    EXPECT_NEAR(output.diagnostics.reference_fit_error, fit, 1e-14);
    EXPECT_EQ(output.diagnostics.bound_violation_count, violations);
    EXPECT_GT(fit, input.config.max_reference_fit_error);
    EXPECT_TRUE(output.valid);  // Final fit is a diagnostic, not a new acceptance gate.
}

TEST_F(ExplicitActuatorWarmStart, PreservesCommandClampsFifoAndActualActuatorResponse) {
    auto output = candidate();
    actuator.v_cmd = input.bounds.v_max - .001;
    actuator.omega_cmd = input.bounds.omega_max - .001;
    for (auto& control : output.controls) {
        control.a = input.bounds.a_max;
        control.alpha = input.bounds.omega_rate_max;
        control.v_s = .2;
    }
    ASSERT_TRUE(rolloutExplicitActuatorWarmStart(output, input, actuator, params, liquid, true));
    const double dt = input.dt;
    const double expected_v = params.linear_gain * actuator.linear_delay_queue.front() +
        (input.robot.v - params.linear_gain * actuator.linear_delay_queue.front()) *
        std::exp(-dt / params.linear_tau_sec);
    EXPECT_NEAR(output.states[1].v, expected_v, 1e-8);
    for (size_t k = 0; k < output.controls.size(); ++k) {
        const auto& state = output.states[k];
        const auto& next = output.states[k+1];
        EXPECT_DOUBLE_EQ(next.v_cmd, input.bounds.v_max);
        EXPECT_DOUBLE_EQ(next.omega_cmd, input.bounds.omega_max);
        EXPECT_DOUBLE_EQ(next.a_cmd_memory, output.controls[k].a);
        EXPECT_DOUBLE_EQ(next.linear_delay_queue.back(), next.v_cmd);
        EXPECT_DOUBLE_EQ(next.angular_delay_queue.back(), next.omega_cmd);
        for (int j = 0; j + 1 < kExplicitLinearDelaySteps; ++j)
            EXPECT_DOUBLE_EQ(next.linear_delay_queue[j], state.linear_delay_queue[j+1]);
        for (int j = 0; j + 1 < kExplicitAngularDelaySteps; ++j)
            EXPECT_DOUBLE_EQ(next.angular_delay_queue[j], state.angular_delay_queue[j+1]);
        EXPECT_NEAR(next.s - state.s, .2 * dt, 1e-15);
    }
}

TEST_F(ExplicitActuatorWarmStart, FailedLateStageLeavesCandidateStatesIntactAndInvalid) {
    auto output = candidate();
    const auto before = output.states;
    actuator.linear_delay_queue[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(rolloutExplicitActuatorWarmStart(output, input, actuator, params, liquid, true));
    EXPECT_FALSE(output.valid);
    EXPECT_FALSE(output.diagnostics.warm_start_valid);
    EXPECT_EQ(output.fallback_reason, "COUPLED_ACTUATOR_WARM_START_FAILED");
    for (size_t k = 0; k < before.size(); ++k) {
        EXPECT_DOUBLE_EQ(output.states[k].px, before[k].px);
        EXPECT_DOUBLE_EQ(output.states[k].v, before[k].v);
        EXPECT_DOUBLE_EQ(output.states[k].eta_x, before[k].eta_x);
        EXPECT_EQ(output.states[k].linear_delay_queue, before[k].linear_delay_queue);
    }
}

TEST_F(ExplicitActuatorWarmStart, LiquidOffHasNoStaleLiquidMetrics) {
    auto output = candidate();
    output.diagnostics.used_slosh_rollout = true;
    output.diagnostics.max_slosh_height_pred = 100;
    ASSERT_TRUE(rolloutExplicitActuatorWarmStart(output, input, actuator, params, liquid, false));
    EXPECT_FALSE(output.diagnostics.used_slosh_rollout);
    EXPECT_DOUBLE_EQ(output.diagnostics.max_slosh_height_pred, 0);
}

TEST_F(ExplicitActuatorWarmStart, CheckedLiquidRejectsInvalidAngularInputAndOverflow) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), 1e300}) {
        for (bool alpha : {false, true}) {
            ContainerExcitation excitation{.1, .2, .3, .4};
            (alpha ? excitation.alpha : excitation.omega) = invalid;
            SloshState next = input.slosh;
            EXPECT_FALSE(liquid.stepWithDt(next, excitation, input.dt, next));
            EXPECT_DOUBLE_EQ(next.eta_x, input.slosh.eta_x);
            EXPECT_DOUBLE_EQ(next.eta_y_dot, input.slosh.eta_y_dot);
        }
    }
    SloshDynamics unconfigured;
    SloshState next;
    EXPECT_FALSE(unconfigured.stepWithDt(input.slosh, {}, input.dt, next));
    EXPECT_DOUBLE_EQ(next.eta_x_dot, input.slosh.eta_x_dot);
}

TEST_F(ExplicitActuatorWarmStart, JointFailureDoesNotPartiallyAdvanceEitherState) {
    for (double tau : {0., std::numeric_limits<double>::quiet_NaN(), 1e-300}) {
        RobotState robot = input.robot;
        SloshState state = input.slosh;
        params.angular_tau_sec = tau;
        EXPECT_FALSE(propagateActualMotion(robot, state, {.2, .4}, params, liquid, input.dt));
        EXPECT_DOUBLE_EQ(robot.x, input.robot.x);
        EXPECT_DOUBLE_EQ(robot.v, input.robot.v);
        EXPECT_DOUBLE_EQ(robot.omega, input.robot.omega);
        EXPECT_DOUBLE_EQ(state.eta_x, input.slosh.eta_x);
        EXPECT_DOUBLE_EQ(state.eta_y_dot, input.slosh.eta_y_dot);
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
