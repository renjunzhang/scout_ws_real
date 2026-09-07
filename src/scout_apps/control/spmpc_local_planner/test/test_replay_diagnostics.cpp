#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "spmpc_local_planner/core/spmpc_problem.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

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

TEST(TerminalHandoff, InfeasiblePublishedHistoryDoesNotReenterOcpAfterStop) {
    auto params = makeParams();
    params.jerk_limit_enable = true;
    params.terminal.mpc_stop_handoff_enable = true;
    SpmpcProblem problem;
    problem.configure(params, makeB0Variant());
    const auto reference = makeStraightReference();
    problem.setReferencePath(reference);
    auto input = makeInput();
    input.robot.x = 4.9;
    input.robot.v = 0.04;
    input.actuator.v_cmd = 0.0;
    input.actuator.a_cmd_memory = -0.7781120448021244;  // bag cycle 1314
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input, output));
    EXPECT_EQ(output.status, "TERMINAL_STOP");
    EXPECT_FALSE(output.ocp_solve_attempted);
    EXPECT_FALSE(output.pre_solve_snapshot.valid);
    EXPECT_FALSE(output.predicted_horizon.valid);
    EXPECT_TRUE(output.terminal_diagnostics.command_owned);
    EXPECT_DOUBLE_EQ(input.actuator.a_cmd_memory, -0.7781120448021244);
    EXPECT_DOUBLE_EQ(output.cmd_v, 0.0);

    // Repeated publication of the same path and odometry noise cannot release it.
    problem.setReferencePath(reference);
    input.robot.x = 4.7;
    input.robot.v = 0.1;
    input.actuator.a_cmd_memory = 0.6879754313324827;  // bag cycle 1316
    ASSERT_TRUE(problem.solve(input, output));
    EXPECT_FALSE(output.ocp_solve_attempted);
    EXPECT_EQ(output.status, "TERMINAL_STOP");
    EXPECT_DOUBLE_EQ(output.cmd_v, 0.0);

    auto points = reference.points();
    for (auto& point : points) point.x += 1.0;
    ReferencePath new_reference;
    new_reference.setPoints(points, "map");
    problem.setReferencePath(new_reference);
    input.robot.x = 1.0;
    input.robot.v = 0.0;
    input.actuator.a_cmd_memory = 0.0;
    problem.solve(input, output);
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_FALSE(output.terminal_diagnostics.command_owned);
}

TEST(TerminalHandoff, SlowdownKeepsPublishedCandidateInsideOcpPlan) {
    auto params = makeParams();
    params.jerk_limit_enable = true;
    params.terminal.mpc_stop_handoff_enable = true;
    SpmpcProblem problem;
    problem.configure(params, makeB0Variant());
    problem.setReferencePath(makeStraightReference());
    auto input = makeInput();
    input.robot.x = 4.0;
    input.actuator.a_cmd_memory = 0.0;
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    ASSERT_TRUE(output.predicted_horizon.valid);
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_FALSE(output.terminal_diagnostics.command_owned);
    EXPECT_EQ(output.pre_solve_snapshot.v_ref_status, "TERMINAL_MPC_SLOWDOWN");
    EXPECT_LT(output.pre_solve_snapshot.requested_v_ref, 0.2);
    EXPECT_DOUBLE_EQ(output.cmd_v, output.predicted_horizon.states[1].v_cmd);
    EXPECT_DOUBLE_EQ(output.cmd_omega, output.predicted_horizon.states[1].omega_cmd);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.2);
}

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

#ifdef SPMPC_TEST_WITH_SLOSH
TEST(ReplayDiagnostics, NoStateZerosOnlyOcpLiquidInitialStateOnEverySolve) {
    auto params = makeParams();
    params.zero_liquid_initial_state = true;
    params.jerk_limit_enable = true;
    auto variant = makeB0Variant();
    variant.name = "B_slosh";
    variant.slosh_enable = true;
    variant.w_slosh = 1.0;
    ContinuousMpccSolverAcados solver;
    solver.configure(params, variant);
    SolverInput input = makeInput();
    input.robot.v = 0.10;
    input.robot.omega = 0.10;
    input.actuator.v_cmd = input.robot.v / params.actuator.linear_gain;
    input.actuator.omega_cmd = input.robot.omega / params.actuator.angular_gain;
    input.actuator.linear_delay_queue.fill(input.actuator.v_cmd);
    input.actuator.angular_delay_queue.fill(input.actuator.omega_cmd);
    input.cycle_timing.solver_input_epoch_ns = 1000000000;
    const auto reference = makeStraightReference();
    for (double eta : {0.0004, -0.0007}) {
        input.slosh = SloshState{eta, 0.002, -eta, -0.003};
        SolverOutput output;
        ASSERT_TRUE(solver.solve(input, reference, output)) << output.status;
        const auto& snapshot = output.pre_solve_snapshot;
        EXPECT_TRUE(snapshot.zero_liquid_initial_state);
        EXPECT_DOUBLE_EQ(snapshot.observed_slosh.eta_x, eta);
        EXPECT_DOUBLE_EQ(snapshot.observed_slosh.eta_y_dot, -0.003);
        EXPECT_DOUBLE_EQ(input.slosh.eta_x, eta);  // caller/monitor untouched
        EXPECT_DOUBLE_EQ(input.slosh.eta_y_dot, -0.003);
        EXPECT_DOUBLE_EQ(snapshot.robot.v, input.robot.v);
        EXPECT_DOUBLE_EQ(snapshot.actuator.a_cmd_memory, input.actuator.a_cmd_memory);
        EXPECT_EQ(snapshot.actuator.linear_delay_queue, input.actuator.linear_delay_queue);
        EXPECT_EQ(output.cycle_timing.solver_input_epoch_ns,
                  input.cycle_timing.solver_input_epoch_ns);
        EXPECT_DOUBLE_EQ(snapshot.slosh.eta_x, 0.0);
        EXPECT_DOUBLE_EQ(snapshot.slosh.eta_x_dot, 0.0);
        EXPECT_DOUBLE_EQ(snapshot.slosh.eta_y, 0.0);
        EXPECT_DOUBLE_EQ(snapshot.slosh.eta_y_dot, 0.0);
        const auto& states = output.predicted_horizon.states;
        ASSERT_EQ(states.size(), 61u);
        EXPECT_NEAR(states.front().eta_x, 0.0, 1e-10);
        EXPECT_NEAR(states.front().eta_y, 0.0, 1e-10);
        // NoState does not zero the future horizon or disable liquid costs.
        EXPECT_GT(std::abs(states[1].eta_y), 1e-9);
        EXPECT_GT(output.cost.J_slosh_eta, 0.0);
    }

    params.zero_liquid_initial_state = false;
    solver.configure(params, variant);
    SolverOutput full;
    ASSERT_TRUE(solver.solve(input, reference, full)) << full.status;
    EXPECT_FALSE(full.pre_solve_snapshot.zero_liquid_initial_state);
    EXPECT_NEAR(full.predicted_horizon.states.front().eta_x, input.slosh.eta_x, 1e-10);
    EXPECT_NEAR(full.predicted_horizon.states.front().eta_y_dot, input.slosh.eta_y_dot, 1e-10);
}
#endif

TEST(ReplayDiagnostics, JerkSwitchBoundsAllStagesAndPublishedHistoryForBothModels) {
    std::vector<bool> models = {false};
#ifdef SPMPC_TEST_WITH_SLOSH
    models.push_back(true);
#endif
    for (bool liquid : models) {
        auto params = makeParams();
        params.jerk_limit_enable = true;
        params.jerk_max = 1.0;
        auto variant = makeB0Variant();
        variant.slosh_enable = liquid;
        variant.w_slosh = liquid ? 1.0 : 0.0;
        ContinuousMpccSolverAcados solver;
        solver.configure(params, variant);
        SolverInput input = makeInput();
        input.actuator.a_cmd_memory = 0.4;
        input.slosh.eta_x = 0.0005;
        SolverOutput limited;
        ASSERT_TRUE(solver.solve(input, makeStraightReference(), limited)) << limited.status;
        const double bound = params.jerk_max * input.dt;
        EXPECT_TRUE(limited.pre_solve_snapshot.jerk_limit_enable);
        EXPECT_DOUBLE_EQ(limited.pre_solve_snapshot.delta_a_max, bound);
        double previous_a = input.actuator.a_cmd_memory;
        ASSERT_EQ(limited.predicted_horizon.controls.size(), 60u);
        for (size_t k = 0; k < limited.predicted_horizon.controls.size(); ++k) {
            const double a = limited.predicted_horizon.controls[k].a;
            EXPECT_LE(std::abs(a - previous_a), bound + 1e-6) << "stage " << k;
            EXPECT_NEAR(limited.predicted_horizon.states[k].a_cmd_memory, previous_a, 1e-7);
            previous_a = a;
        }
        params.jerk_limit_enable = false;
        solver.configure(params, variant);
        SolverOutput unlimited;
        ASSERT_TRUE(solver.solve(input, makeStraightReference(), unlimited)) << unlimited.status;
        EXPECT_FALSE(unlimited.pre_solve_snapshot.jerk_limit_enable);
        EXPECT_DOUBLE_EQ(unlimited.pre_solve_snapshot.delta_a_max, 1e15);
        EXPECT_GT(std::abs(unlimited.predicted_horizon.controls.front().a -
                           input.actuator.a_cmd_memory), bound + 1e-3);
    }
}

TEST(ReplayDiagnostics, InfeasibleJerkHistoryFailsWithoutClampingOrChangingHistory) {
    auto params = makeParams();
    params.jerk_limit_enable = true;
    ContinuousMpccSolverAcados solver;
    solver.configure(params, makeB0Variant());
    SolverInput input = makeInput();
    input.actuator.a_cmd_memory = -3.0;
    SolverOutput output;
    EXPECT_FALSE(solver.solve(input, makeStraightReference(), output));
    EXPECT_FALSE(output.success);
    EXPECT_DOUBLE_EQ(output.cmd_v, 0.0);
    EXPECT_DOUBLE_EQ(input.actuator.a_cmd_memory, -3.0);
    EXPECT_DOUBLE_EQ(output.pre_solve_snapshot.actuator.a_cmd_memory, -3.0);
}

TEST(ReplayDiagnostics, RejectsInvalidAblationConfiguration) {
    auto params = makeParams();
    params.jerk_limit_enable = true;
    params.jerk_max = std::numeric_limits<double>::quiet_NaN();
    ContinuousMpccSolverAcados solver;
    solver.configure(params, makeB0Variant());
    SolverOutput output;
    EXPECT_FALSE(solver.solve(makeInput(), makeStraightReference(), output));
    EXPECT_EQ(output.status, "INVALID_ABLATION_CONFIG");
    params.jerk_max = 1.0;
    params.zero_liquid_initial_state = true;  // cannot call a liquid-off solver NoState
    solver.configure(params, makeB0Variant());
    EXPECT_FALSE(solver.solve(makeInput(), makeStraightReference(), output));
    EXPECT_EQ(output.status, "INVALID_ABLATION_CONFIG");
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
