#include "spmpc_local_planner/analysis/g4_snapshot_replay.h"
#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "spmpc_parameter_manifest.h"

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
    context.owns_terminal_maneuver = true;
    context.current_index = 0;
    context.front_index = 2;
    context.terminal_index = 5;
    context.front_steps = 2;
    context.liquid_steps = 3;
    context.nominal_publish_v = 0.0;
    context.nominal_publish_omega = 0.0;
    context.max_residual_v = 0.08;
    context.max_residual_omega = 0.20;
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

analysis::G4ReplayFrame makeG4ReplayFrame(
    const PreSolveSnapshotDebug& snapshot) {
    analysis::G4ReplayFrame frame;
    frame.pair_index = 0;
    frame.direction_code = 1;
    frame.horizon_steps = snapshot.horizon_steps;
    frame.state_width = snapshot.state_width;
    frame.control_width = snapshot.control_width;
    frame.parameter_width = snapshot.parameter_width;
    frame.dt = snapshot.dt;
    frame.initial_state = {{
        snapshot.robot.x,
        snapshot.robot.y,
        snapshot.robot.yaw,
        snapshot.robot.v,
        snapshot.s0,
        snapshot.robot.omega,
        snapshot.slosh.eta_x,
        snapshot.slosh.eta_x_dot,
        snapshot.slosh.eta_y,
        snapshot.slosh.eta_y_dot,
    }};
    frame.runtime_bounds = snapshot.runtime_bounds;
    frame.stage_parameters = snapshot.stage_parameters;
    for (const HorizonStateDebug& state : snapshot.initial_guess_states) {
        const double row[] = {
            state.x, state.y, state.yaw, state.v, state.s, state.omega,
            state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot,
        };
        frame.initial_guess_states.insert(
            frame.initial_guess_states.end(), std::begin(row), std::end(row));
    }
    for (const HorizonControlDebug& control :
         snapshot.initial_guess_controls) {
        frame.initial_guess_controls.push_back(control.a);
        frame.initial_guess_controls.push_back(control.alpha_or_omega);
        frame.initial_guess_controls.push_back(control.v_s);
    }
    frame.modal_overrides.push_back({{0.0, 0.0, 0.0, 0.0}});
    return frame;
}

}  // namespace

TEST(ReplayDiagnostics, PhaseRejoinCapabilityReportsDedicatedFixedHorizon) {
    const bool available = continuousMpccPhaseRejoinAvailable();
    const int horizon = continuousMpccPhaseRejoinHorizonSteps();
    EXPECT_EQ(available, horizon > 0);
    if (available) {
        EXPECT_EQ(horizon, 3);
    } else {
        EXPECT_EQ(horizon, 0);
    }
}

TEST(ReplayDiagnostics, CapturesFullHorizonAndPreSolveContext) {
    ContinuousMpccSolverAcados solver;
    const SolverConfigureResult configured =
        solver.configure(makeParams(), makeB0Variant());
    ASSERT_TRUE(configured.success)
        << configured.status << ": " << configured.detail;
    const ReferencePath reference = makeStraightReference();
    SolverInput input = makeInput();

    SolverOutput first;
    ASSERT_TRUE(solver.solve(input, reference, first)) << first.status;
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(first.predicted_horizon.valid);
    EXPECT_EQ(first.predicted_horizon.states.size(), 61u);
    EXPECT_EQ(first.predicted_horizon.controls.size(), 60u);
    EXPECT_EQ(first.predicted_horizon.control_semantics, "alpha");
    EXPECT_DOUBLE_EQ(first.generated_bounds.a_min,
                     -acados_manifest::generated_bounds::kAMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.a_max,
                     acados_manifest::generated_bounds::kAMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.alpha_min,
                     -acados_manifest::generated_bounds::kAlphaMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.alpha_max,
                     acados_manifest::generated_bounds::kAlphaMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.v_s_max,
                     acados_manifest::generated_bounds::kVsMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.v_max,
                     acados_manifest::generated_bounds::kVMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.omega_min,
                     -acados_manifest::generated_bounds::kOmegaMax);
    EXPECT_DOUBLE_EQ(first.generated_bounds.omega_max,
                     acados_manifest::generated_bounds::kOmegaMax);

    ASSERT_TRUE(first.pre_solve_snapshot.valid);
    EXPECT_TRUE(first.pre_solve_snapshot.primal_guess_only);
    EXPECT_EQ(first.pre_solve_snapshot.horizon_steps,
              acados_manifest::generated_bounds::kMainHorizonSteps);
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
TEST(ReplayDiagnostics, G4RunnerReusesCurrentBackendAndGeneratedContract) {
    ContinuousMpccSolverAcados solver;
    const SolverConfigureResult configured =
        solver.configure(makeParams(), makeSloshVariant());
    ASSERT_TRUE(configured.success)
        << configured.status << ": " << configured.detail;
    const ReferencePath reference = makeStraightReference();
    SolverInput first_input = makeInput();
    SolverOutput first_online;
    ASSERT_TRUE(solver.solve(first_input, reference, first_online))
        << first_online.status;
    ASSERT_TRUE(first_online.pre_solve_snapshot.valid);
    ASSERT_TRUE(first_online.predicted_horizon.valid);
    SolverInput second_input = first_input;
    second_input.robot.x = first_online.predicted_horizon.states[1].x;
    second_input.robot.y = first_online.predicted_horizon.states[1].y;
    second_input.robot.yaw = first_online.predicted_horizon.states[1].yaw;
    second_input.robot.v = first_online.predicted_horizon.states[1].v;
    second_input.robot.omega =
        first_online.predicted_horizon.states[1].omega;
    SolverOutput second_online;
    ASSERT_TRUE(solver.solve(second_input, reference, second_online))
        << second_online.status;
    ASSERT_TRUE(analysis::G4SnapshotReplayRunner::available());

    analysis::G4ReplayFrame first_frame =
        makeG4ReplayFrame(first_online.pre_solve_snapshot);
    first_frame.direction_code = 0;
    first_frame.modal_overrides.clear();
    analysis::G4ReplayFrame second_frame =
        makeG4ReplayFrame(second_online.pre_solve_snapshot);
    second_frame.pair_index = 1;
    const analysis::G4SequenceReplayResult replayed =
        analysis::G4SnapshotReplayRunner::run({first_frame, second_frame});
    ASSERT_TRUE(replayed.success) << replayed.detail;
    ASSERT_EQ(replayed.checkpoints.size(), 1u);
    const analysis::G4ReplaySolution& actual =
        replayed.checkpoints.front().actual;
    ASSERT_EQ(actual.status, 0);
    ASSERT_EQ(actual.states.size(), 61u * 10u);
    ASSERT_EQ(actual.controls.size(), 60u * 3u);
    for (std::size_t stage = 0;
         stage < second_online.predicted_horizon.states.size(); ++stage) {
        const HorizonStateDebug& expected =
            second_online.predicted_horizon.states[stage];
        const double row[] = {
            expected.x, expected.y, expected.yaw, expected.v, expected.s,
            expected.omega, expected.eta_x, expected.eta_x_dot,
            expected.eta_y, expected.eta_y_dot,
        };
        for (std::size_t column = 0; column < 10; ++column) {
            EXPECT_NEAR(actual.states[stage * 10 + column], row[column], 1e-8);
        }
    }
    for (std::size_t stage = 0;
         stage < second_online.predicted_horizon.controls.size(); ++stage) {
        const HorizonControlDebug& expected =
            second_online.predicted_horizon.controls[stage];
        EXPECT_NEAR(actual.controls[stage * 3], expected.a, 1e-8);
        EXPECT_NEAR(actual.controls[stage * 3 + 1],
                    expected.alpha_or_omega, 1e-8);
        EXPECT_NEAR(actual.controls[stage * 3 + 2], expected.v_s, 1e-8);
    }

    analysis::G4ReplayFrame invalid = second_frame;
    ++invalid.parameter_width;
    const analysis::G4SequenceReplayResult rejected =
        analysis::G4SnapshotReplayRunner::run({invalid});
    EXPECT_FALSE(rejected.success);
    EXPECT_EQ(rejected.detail, "GENERATED_DIMENSION_MISMATCH");
}

TEST(ReplayDiagnostics, MonitorPreservesBaselineAndEnforceHasNoFreeTail) {
    ContinuousMpccSolverAcados solver;
    const SolverConfigureResult configured =
        solver.configure(makeParams(), makeSloshVariant());
    ASSERT_TRUE(configured.success)
        << configured.status << ": " << configured.detail;
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
    EXPECT_EQ(enforce_output.pre_solve_snapshot.horizon_steps, 3);
    EXPECT_EQ(enforce_output.predicted_horizon.states.size(), 4u);
    EXPECT_EQ(enforce_output.predicted_horizon.controls.size(), 3u);
    EXPECT_EQ(enforce_output.pre_solve_snapshot.stage_parameters.size(),
              4u * 55u);
    for (int stage = 0; stage <= 3; ++stage) {
        EXPECT_DOUBLE_EQ(stageParameter(
            enforce_output.pre_solve_snapshot, stage, phase_active),
            1.0);
        EXPECT_DOUBLE_EQ(stageParameter(
            enforce_output.pre_solve_snapshot, stage, gate_active),
            stage == 3 ? 1.0 : 0.0);
    }
    // N=N_l by construction: there is no stage 4 and no free geometry tail.
    EXPECT_DOUBLE_EQ(stageParameter(
        enforce_output.pre_solve_snapshot, 3, w_slosh_eta), 5.0);
    EXPECT_LE(std::abs(enforce_output.cmd_v -
                       enforce_input.phase_rejoin.nominal_publish_v),
              enforce_input.phase_rejoin.max_residual_v + 1e-7);
    EXPECT_LE(std::abs(enforce_output.cmd_omega -
                       enforce_input.phase_rejoin.nominal_publish_omega),
              enforce_input.phase_rejoin.max_residual_omega + 1e-7);
    // The generated phase OCP replaces the baseline progress/control/smooth
    // priors with nominal-relative terms; diagnostics must report that same
    // objective rather than stacking both formulations.
    EXPECT_DOUBLE_EQ(enforce_output.cost.J_progress, 0.0);
    EXPECT_DOUBLE_EQ(enforce_output.cost.J_smooth, 0.0);
}

TEST(ReplayDiagnostics, InvalidEnforceContextFailsClosedWithStableStatus) {
    ContinuousMpccSolverAcados solver;
    const SolverConfigureResult configured =
        solver.configure(makeParams(), makeSloshVariant());
    ASSERT_TRUE(configured.success)
        << configured.status << ": " << configured.detail;
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
