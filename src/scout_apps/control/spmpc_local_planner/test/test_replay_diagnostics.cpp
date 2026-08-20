#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#include <gtest/gtest.h>

#include <algorithm>

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

VariantConfig makeSloshVariant() {
    VariantConfig variant = makeB0Variant();
    variant.name = "B_slosh_phase_test";
    variant.slosh_enable = true;
    variant.slosh_constraint_enable = false;
    variant.w_slosh = 5.0;
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
    return input;
}

std::size_t parameterIndex(const PreSolveSnapshotDebug& snapshot,
                           const std::string& name) {
    const auto it = std::find(snapshot.parameter_names.begin(),
                              snapshot.parameter_names.end(), name);
    EXPECT_NE(it, snapshot.parameter_names.end()) << name;
    return static_cast<std::size_t>(
        std::distance(snapshot.parameter_names.begin(), it));
}

double stageParameter(const PreSolveSnapshotDebug& snapshot,
                      int stage,
                      std::size_t parameter_index) {
    const std::size_t offset = static_cast<std::size_t>(stage) *
        static_cast<std::size_t>(snapshot.parameter_width) + parameter_index;
    EXPECT_LT(offset, snapshot.stage_parameters.size());
    return offset < snapshot.stage_parameters.size()
        ? snapshot.stage_parameters[offset] : 0.0;
}

EmpiricalRecoveryRadii broadRadii() {
    EmpiricalRecoveryRadii radii;
    radii.x = 100.0;
    radii.y = 100.0;
    radii.yaw = 100.0;
    radii.v = 100.0;
    radii.omega = 100.0;
    radii.eta_x = 100.0;
    radii.eta_x_dot = 100.0;
    radii.eta_y = 100.0;
    radii.eta_y_dot = 100.0;
    return radii;
}

PhaseRejoinSolverContext makeEnforceContext() {
    PhaseRejoinSolverContext context;
    context.active = true;
    context.enforce = true;
    context.empirical_gate = true;
    context.state_complete_for_certificate = false;
    context.current_index = 0;
    context.front_index = 2;
    context.terminal_index = 5;
    context.front_steps = 2;
    context.liquid_steps = 3;
    for (int k = 0; k <= context.liquid_steps; ++k) {
        PhaseNominalStage stage;
        stage.valid = true;
        stage.gate_active = k == context.liquid_steps;
        stage.artifact_index = context.front_index +
            static_cast<std::size_t>(k);
        stage.s = 0.05 * static_cast<double>(k);
        stage.v = 0.20;
        stage.v_s = 0.20;
        stage.radii = broadRadii();
        context.stages.push_back(stage);
    }
    return context;
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
    EXPECT_EQ(first.predicted_horizon.control_semantics, "alpha");

    ASSERT_TRUE(first.pre_solve_snapshot.valid);
    EXPECT_TRUE(first.pre_solve_snapshot.primal_guess_only);
    EXPECT_EQ(first.pre_solve_snapshot.horizon_steps, 60);
    EXPECT_EQ(first.pre_solve_snapshot.state_width, 10);
    EXPECT_EQ(first.pre_solve_snapshot.control_width, 3);
    EXPECT_EQ(first.pre_solve_snapshot.parameter_width, 23);
    EXPECT_EQ(first.pre_solve_snapshot.parameter_names.size(), 23u);
    EXPECT_EQ(first.pre_solve_snapshot.stage_parameters.size(), 61u * 23u);
    EXPECT_EQ(first.pre_solve_snapshot.initial_guess_states.size(), 61u);
    EXPECT_EQ(first.pre_solve_snapshot.initial_guess_controls.size(), 60u);
    EXPECT_FALSE(first.pre_solve_snapshot.have_previous_solution);
    EXPECT_EQ(first.pre_solve_snapshot.v_ref_status, "TEST_OVERRIDE");

    SolverInput second_input = input;
    second_input.robot.x = first.predicted_horizon.states[1].x;
    second_input.robot.y = first.predicted_horizon.states[1].y;
    second_input.robot.yaw = first.predicted_horizon.states[1].yaw;
    second_input.robot.v = first.predicted_horizon.states[1].v;
    second_input.robot.omega = first.predicted_horizon.states[1].omega;

    SolverOutput second;
    ASSERT_TRUE(solver.solve(second_input, reference, second)) << second.status;
    ASSERT_TRUE(second.pre_solve_snapshot.valid);
    EXPECT_TRUE(second.pre_solve_snapshot.have_previous_control);
    EXPECT_TRUE(second.pre_solve_snapshot.have_previous_solution);
    EXPECT_EQ(second.pre_solve_snapshot.previous_solution_states.size(), 61u);
    EXPECT_EQ(second.pre_solve_snapshot.previous_solution_controls.size(), 60u);
}

#ifdef SPMPC_TEST_WITH_ACADOS_SLOSH
TEST(ReplayDiagnostics, MonitorContextPreservesBaselineAndEnforceGateIsStageLocal) {
    ContinuousMpccSolverAcados solver;
    solver.configure(makeParams(), makeSloshVariant());
    const ReferencePath reference = makeStraightReference();

    SolverInput monitor_input = makeInput();
    monitor_input.phase_rejoin.active = true;
    monitor_input.phase_rejoin.enforce = false;
    monitor_input.phase_rejoin.empirical_gate = true;
    monitor_input.phase_rejoin.liquid_steps = 3;
    // Intentionally omit nominal stages: monitor must not alter or validate the
    // baseline OCP parameter stream.
    SolverOutput monitor_output;
    ASSERT_TRUE(solver.solve(monitor_input, reference, monitor_output))
        << monitor_output.status;
    ASSERT_EQ(monitor_output.pre_solve_snapshot.parameter_width, 55);
    const std::size_t phase_active = parameterIndex(
        monitor_output.pre_solve_snapshot, "phase_rejoin_active");
    const std::size_t gate_active = parameterIndex(
        monitor_output.pre_solve_snapshot, "empirical_gate_active");
    const std::size_t w_slosh_eta = parameterIndex(
        monitor_output.pre_solve_snapshot, "w_slosh_eta");
    for (int stage = 0; stage <= 60; ++stage) {
        EXPECT_DOUBLE_EQ(stageParameter(
            monitor_output.pre_solve_snapshot, stage, phase_active), 0.0);
        EXPECT_DOUBLE_EQ(stageParameter(
            monitor_output.pre_solve_snapshot, stage, gate_active), 0.0);
    }
    EXPECT_DOUBLE_EQ(stageParameter(
        monitor_output.pre_solve_snapshot, 4, w_slosh_eta), 5.0);

    SolverInput enforce_input = makeInput();
    enforce_input.phase_rejoin = makeEnforceContext();
    SolverOutput enforce_output;
    ASSERT_TRUE(solver.solve(enforce_input, reference, enforce_output))
        << enforce_output.status;
    ASSERT_EQ(enforce_output.pre_solve_snapshot.parameter_width, 55);
    for (int stage = 0; stage <= 60; ++stage) {
        const bool in_liquid_window = stage <= 3;
        EXPECT_DOUBLE_EQ(stageParameter(
            enforce_output.pre_solve_snapshot, stage, phase_active),
            in_liquid_window ? 1.0 : 0.0);
        EXPECT_DOUBLE_EQ(stageParameter(
            enforce_output.pre_solve_snapshot, stage, gate_active),
            stage == 3 ? 1.0 : 0.0);
    }
    // Long geometry preview remains, while liquid cost is cut after N_l.
    EXPECT_DOUBLE_EQ(stageParameter(
        enforce_output.pre_solve_snapshot, 3, w_slosh_eta), 5.0);
    EXPECT_DOUBLE_EQ(stageParameter(
        enforce_output.pre_solve_snapshot, 4, w_slosh_eta), 0.0);
}

TEST(ReplayDiagnostics, InvalidEnforceContextFailsClosedWithStableStatus) {
    ContinuousMpccSolverAcados solver;
    solver.configure(makeParams(), makeSloshVariant());
    SolverInput input = makeInput();
    input.phase_rejoin = makeEnforceContext();
    input.phase_rejoin.stages.pop_back();

    SolverOutput output;
    EXPECT_FALSE(solver.solve(input, makeStraightReference(), output));
    EXPECT_EQ(output.status, "PHASE_REJOIN_CONTEXT_INVALID_STAGE_COUNT");
}
#endif

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
