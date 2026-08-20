#include "spmpc_local_planner/warm_start/warm_start_policy.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace spmpc_local_planner {
namespace {

class FakeWarmStartGenerator : public WarmStartGenerator {
public:
    bool generate(const WarmStartInput& input,
                  WarmStartOutput& output,
                  WarmStartDiagnostics& diagnostics) override {
        ++call_count;
        observed_previous_omega = input.previous_omega;
        diagnostics.used_flatness = flatness;
        diagnostics.failure_reason = valid ? "" : "FAKE_REJECTED";
        output = WarmStartOutput{};
        if (!valid) return false;
        output.states.resize(
            static_cast<std::size_t>(input.horizon_steps + 1));
        output.controls.resize(
            static_cast<std::size_t>(input.horizon_steps));
        for (int stage = 0; stage <= input.horizon_steps; ++stage) {
            output.states[static_cast<std::size_t>(stage)].s =
                input.s0 + static_cast<double>(stage);
        }
        output.valid = true;
        return true;
    }

    bool valid = false;
    bool flatness = false;
    int call_count = 0;
    double observed_previous_omega = 0.0;
};

class WarmStartPolicyTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::vector<TrajectoryPoint> points;
        for (int index = 0; index <= 10; ++index) {
            TrajectoryPoint point;
            point.x = 0.1 * static_cast<double>(index);
            point.y = 0.0;
            point.yaw = 0.0;
            points.push_back(point);
        }
        reference.setPoints(points, "test");
        spline.build(reference);
        solver_input.robot.x = 0.2;
        solver_input.robot.y = -0.1;
        solver_input.robot.yaw = 0.3;
        solver_input.robot.v = 0.25;
        solver_input.robot.omega = -0.4;
        solver_input.dt = 0.1;
        params.v_max = 0.8;
        params.omega_max = 1.2;
        params.a_max = 0.6;
        params.alpha_max = 1.0;
        params.warm_start.enable = true;
        params.warm_start.fallback_to_previous_solution = true;
        params.warm_start.fallback_to_primitive = true;
    }

    WarmStartPolicyInput makePolicyInput() {
        WarmStartPolicyInput input;
        input.solver_input = &solver_input;
        input.reference = &reference;
        input.spline = &spline;
        input.params = &params;
        input.slosh_dynamics = &slosh_dynamics;
        input.generator = &generator;
        input.progress_s = 0.1;
        input.reference_length = reference.length();
        input.horizon_steps = 3;
        input.have_previous_control = true;
        input.previous_control = {{0.2, 9.0, 0.3}};
        return input;
    }

    WarmStartOutput makePreviousSolution(double second_stage_progress) {
        WarmStartOutput previous;
        previous.valid = true;
        previous.states.resize(4);
        previous.controls.resize(3);
        for (int stage = 0; stage <= 3; ++stage) {
            WarmStartState& state =
                previous.states[static_cast<std::size_t>(stage)];
            state.px = 10.0 + stage;
            state.s = second_stage_progress + 0.1 * (stage - 1);
            state.v = 0.4;
        }
        for (int stage = 0; stage < 3; ++stage) {
            WarmStartControl& control =
                previous.controls[static_cast<std::size_t>(stage)];
            control.a = 0.1 * stage;
            control.alpha = 0.2 * stage;
            control.v_s = 0.3 + 0.1 * stage;
        }
        return previous;
    }

    ReferencePath reference;
    ReferenceSpline spline;
    SolverInput solver_input;
    SolverParams params;
    SloshDynamics slosh_dynamics;
    FakeWarmStartGenerator generator;
};

TEST_F(WarmStartPolicyTest, DisabledPolicyKeepsCapsuleReuseUntouched) {
    params.warm_start.enable = false;
    const WarmStartPolicyDecision decision =
        WarmStartPolicy::select(makePolicyInput());

    EXPECT_FALSE(decision.requested);
    EXPECT_FALSE(decision.applied);
    EXPECT_EQ(decision.source, "CAPSULE_REUSE");
    EXPECT_EQ(decision.status, "DISABLED");
    EXPECT_EQ(generator.call_count, 0);
}

TEST_F(WarmStartPolicyTest, ValidGeneratorWinsAndReceivesMeasuredOmega) {
    generator.valid = true;
    generator.flatness = true;
    WarmStartOutput previous = makePreviousSolution(0.1);
    WarmStartPolicyInput input = makePolicyInput();
    input.previous_solution = &previous;

    const WarmStartPolicyDecision decision = WarmStartPolicy::select(input);

    ASSERT_TRUE(decision.applied);
    EXPECT_EQ(decision.source, "FLATNESS_GENERATOR");
    EXPECT_EQ(generator.call_count, 1);
    EXPECT_DOUBLE_EQ(
        generator.observed_previous_omega, solver_input.robot.omega);
    EXPECT_NE(generator.observed_previous_omega, input.previous_control[1]);
}

TEST_F(WarmStartPolicyTest,
       InvalidGeneratorFallsBackToShiftedPreviousSolution) {
    WarmStartOutput previous = makePreviousSolution(0.1);
    WarmStartPolicyInput input = makePolicyInput();
    input.previous_solution = &previous;

    const WarmStartPolicyDecision decision = WarmStartPolicy::select(input);

    ASSERT_TRUE(decision.applied);
    EXPECT_EQ(decision.source, "SHIFTED_PREVIOUS_SOLUTION");
    ASSERT_EQ(decision.warm_start.states.size(), 4U);
    ASSERT_EQ(decision.warm_start.controls.size(), 3U);
    EXPECT_TRUE(decision.warm_start.diagnostics.used_previous_solution);
    EXPECT_DOUBLE_EQ(decision.warm_start.states[0].px,
                     solver_input.robot.x);
    EXPECT_DOUBLE_EQ(decision.warm_start.states[0].s, input.progress_s);
    EXPECT_DOUBLE_EQ(decision.warm_start.states[1].px,
                     previous.states[2].px);
    EXPECT_DOUBLE_EQ(decision.warm_start.controls[0].v_s,
                     previous.controls[1].v_s);
}

TEST_F(WarmStartPolicyTest,
       RejectedPreviousSolutionFallsBackToConservativePathRollout) {
    WarmStartOutput previous = makePreviousSolution(5.0);
    WarmStartPolicyInput input = makePolicyInput();
    input.previous_solution = &previous;

    const WarmStartPolicyDecision decision = WarmStartPolicy::select(input);

    ASSERT_TRUE(decision.applied);
    EXPECT_EQ(decision.source, "CONSERVATIVE_FALLBACK");
    EXPECT_TRUE(decision.warm_start.diagnostics.used_fallback);
    ASSERT_EQ(decision.warm_start.states.size(), 4U);
    EXPECT_DOUBLE_EQ(decision.warm_start.states.front().px,
                     solver_input.robot.x);
    EXPECT_DOUBLE_EQ(decision.warm_start.states.front().s,
                     input.progress_s);
}

TEST_F(WarmStartPolicyTest, ExhaustedFallbacksKeepCapsuleReuseSource) {
    params.warm_start.fallback_to_previous_solution = false;
    params.warm_start.fallback_to_primitive = false;

    const WarmStartPolicyDecision decision =
        WarmStartPolicy::select(makePolicyInput());

    EXPECT_TRUE(decision.requested);
    EXPECT_FALSE(decision.applied);
    EXPECT_EQ(decision.source, "CAPSULE_REUSE");
    EXPECT_EQ(decision.status, "NO_VALID_WARM_START");
    EXPECT_EQ(decision.warm_start.diagnostics.failure_reason,
              "FAKE_REJECTED");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
