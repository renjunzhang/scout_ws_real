#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

ReferencePath makeStraightReference() {
    std::vector<TrajectoryPoint> points;
    for (int i = 0; i <= 100; ++i) {
        TrajectoryPoint point;
        point.x = 0.05 * static_cast<double>(i);
        point.y = 0.0;
        point.yaw = 0.0;
        points.push_back(point);
    }
    ReferencePath path;
    path.setPoints(points, "map");
    return path;
}

SolverParams makeParams() {
    SolverParams params;
    params.v_max = 0.8;
    params.omega_max = 1.2;
    params.a_max = 0.6;
    params.alpha_max = 1.2;
    params.corridor_width = 0.30;
    params.warm_start.enable = true;
    params.warm_start.type = "diff_drive_flatness";
    params.warm_start.use_slosh_rollout = true;
    params.warm_start.fallback_to_previous_solution = false;
    params.warm_start.fallback_to_primitive = true;
    params.platform.kinematics = "differential";
    params.slosh.dt = 1.0 / 30.0;
    return params;
}

VariantConfig makeB0Variant() {
    VariantConfig variant;
    variant.name = "B0";
    variant.slosh_enable = false;
    variant.w_contour = 1.0;
    variant.w_lag = 0.2;
    variant.w_progress = 0.2;
    variant.w_v = 1.0;
    variant.w_vs = 0.3;
    variant.v_ref = 0.20;
    variant.w_control = 0.1;
    variant.w_accel = 0.0;
    variant.w_smooth = 0.1;
    variant.w_alpha = 0.1;
    variant.w_du_a = 0.1;
    variant.w_du_vs = 0.1;
    variant.w_slosh = 0.0;
    return variant;
}

SolverInput makeInput() {
    SolverInput input;
    input.robot.x = 0.0;
    input.robot.y = 0.0;
    input.robot.yaw = 0.0;
    input.robot.v = 0.0;
    input.robot.omega = 0.0;
    input.dt = 1.0 / 30.0;
    input.horizon_steps = 60;
    input.min_progress_s = 0.0;
    input.has_v_ref_current = true;
    input.v_ref_current = 0.20;
    input.v_ref_status = "TEST_OVERRIDE";
    input.actuator.valid = true;
    input.actuator.a_cmd_memory = -0.07;
    return input;
}

}  // namespace

TEST(ReplayDiagnostics, CapturesFullHorizonAndPreSolveContext) {
    ContinuousMpccSolverAcados solver;
    solver.configure(makeParams(), makeB0Variant());
    const ReferencePath reference = makeStraightReference();
    SolverInput input = makeInput();

    SolverOutput first;
    ASSERT_TRUE(solver.solve(input, reference, first)) << first.status;
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(first.predicted_horizon.valid);
    EXPECT_EQ(first.predicted_horizon.states.size(), 61u);
    EXPECT_EQ(first.predicted_horizon.controls.size(), 60u);
    EXPECT_EQ(first.predicted_horizon.control_semantics, "a_cmd_alpha_cmd");
    ASSERT_GT(first.predicted_horizon.states.size(), 1u);
    EXPECT_NEAR(
        first.cmd_v, first.predicted_horizon.states[1].v_cmd, 1.0e-9);
    EXPECT_NEAR(
        first.cmd_omega,
        first.predicted_horizon.states[1].omega_cmd,
        1.0e-9);
    EXPECT_NEAR(
        first.predicted_horizon.states[1].a_cmd_memory,
        first.predicted_horizon.controls[0].a,
        1.0e-9);

    ASSERT_TRUE(first.pre_solve_snapshot.valid);
    EXPECT_TRUE(first.pre_solve_snapshot.primal_guess_only);
    EXPECT_EQ(first.pre_solve_snapshot.horizon_steps, 60);
    EXPECT_EQ(first.pre_solve_snapshot.state_width, 24);
    EXPECT_EQ(first.pre_solve_snapshot.control_width, 3);
    EXPECT_EQ(first.pre_solve_snapshot.parameter_width, 28);
    EXPECT_EQ(first.pre_solve_snapshot.parameter_names.size(), 28u);
    EXPECT_EQ(first.pre_solve_snapshot.stage_parameters.size(), 61u * 28u);
    EXPECT_EQ(first.pre_solve_snapshot.initial_guess_states.size(), 61u);
    EXPECT_EQ(first.pre_solve_snapshot.initial_guess_controls.size(), 60u);
    EXPECT_FALSE(first.pre_solve_snapshot.have_previous_solution);
    EXPECT_EQ(first.pre_solve_snapshot.v_ref_status, "TEST_OVERRIDE");
    ASSERT_EQ(
        first.pre_solve_snapshot.initial_guess_states.front().model_state.size(),
        24u);
    EXPECT_DOUBLE_EQ(
        first.pre_solve_snapshot.initial_guess_states.front()
            .model_state[static_cast<size_t>(kExplicitActuatorAccelMemoryIndex)],
        input.actuator.a_cmd_memory);
    for (int stage : {0, 1, 59, 60}) {
        const size_t base = static_cast<size_t>(stage * 28);
        EXPECT_DOUBLE_EQ(
            first.pre_solve_snapshot.stage_parameters[base + 16],
            makeB0Variant().w_du_a);
        EXPECT_DOUBLE_EQ(
            first.pre_solve_snapshot.stage_parameters[base + 18],
            input.actuator.a_cmd_memory);
    }

    SolverInput second_input = input;
    second_input.robot.x = first.predicted_horizon.states[1].x;
    second_input.robot.y = first.predicted_horizon.states[1].y;
    second_input.robot.yaw = first.predicted_horizon.states[1].yaw;
    second_input.robot.v = first.predicted_horizon.states[1].v;
    second_input.robot.omega = first.predicted_horizon.states[1].omega;
    second_input.actuator.v_cmd = first.predicted_horizon.states[1].v_cmd;
    second_input.actuator.omega_cmd =
        first.predicted_horizon.states[1].omega_cmd;
    const auto& model_state = first.predicted_horizon.states[1].model_state;
    ASSERT_EQ(model_state.size(), 24u);
    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        second_input.actuator.linear_delay_queue[static_cast<size_t>(i)] =
            model_state[static_cast<size_t>(8 + i)];
    }
    for (int i = 0; i < kExplicitAngularDelaySteps; ++i) {
        second_input.actuator.angular_delay_queue[static_cast<size_t>(i)] =
            model_state[static_cast<size_t>(
                8 + kExplicitLinearDelaySteps + i)];
    }
    second_input.actuator.a_cmd_memory =
        model_state[static_cast<size_t>(kExplicitActuatorAccelMemoryIndex)];

    SolverOutput second;
    ASSERT_TRUE(solver.solve(second_input, reference, second)) << second.status;
    ASSERT_TRUE(second.pre_solve_snapshot.valid);
    EXPECT_TRUE(second.pre_solve_snapshot.have_previous_control);
    EXPECT_TRUE(second.pre_solve_snapshot.have_previous_solution);
    EXPECT_EQ(second.pre_solve_snapshot.previous_solution_states.size(), 61u);
    EXPECT_EQ(second.pre_solve_snapshot.previous_solution_controls.size(), 60u);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
