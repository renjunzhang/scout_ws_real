#include "spmpc_local_planner/solver/acados/solution_decoder.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace spmpc_local_planner {
namespace {

ReferencePath makeLineReference() {
    std::vector<TrajectoryPoint> points;
    for (int index = 0; index <= 20; ++index) {
        TrajectoryPoint point;
        point.x = 0.1 * static_cast<double>(index);
        point.y = 0.0;
        point.yaw = 0.0;
        points.push_back(point);
    }
    ReferencePath reference;
    reference.setPoints(points, "test");
    return reference;
}

void appendState(AcadosRawSolution& raw,
                 double x,
                 double y,
                 double yaw,
                 double velocity,
                 double progress,
                 double omega,
                 double eta_x = 0.0,
                 double eta_x_dot = 0.0,
                 double eta_y = 0.0,
                 double eta_y_dot = 0.0) {
    const double state[] = {
        x, y, yaw, velocity, progress, omega,
        eta_x, eta_x_dot, eta_y, eta_y_dot,
    };
    raw.states.insert(raw.states.end(), state, state + 10);
}

void appendControl(AcadosRawSolution& raw,
                   double acceleration,
                   double alpha,
                   double progress_speed) {
    const double control[] = {acceleration, alpha, progress_speed};
    raw.controls.insert(raw.controls.end(), control, control + 3);
}

AcadosSolutionDecoderInput makeDecoderInput(
    const AcadosRawSolution& raw,
    const SolverInput& solver_input,
    const ReferencePath& reference,
    const SolverParams& params,
    const VariantConfig& variant) {
    AcadosSolutionDecoderInput input;
    input.raw_solution = &raw;
    input.solver_input = &solver_input;
    input.reference = &reference;
    input.params = &params;
    input.variant = &variant;
    input.reference_x_coeffs = {{0.0, 1.0, 0.0, 0.0}};
    input.reference_y_coeffs = {{0.0, 0.0, 0.0, 0.0}};
    input.contour_error_ref = 1.0;
    input.lag_error_ref = 1.0;
    input.effective_v_ref = 0.5;
    return input;
}

TEST(AcadosSolutionDecoder, RejectsMalformedRawMatrixBeforeIndexing) {
    const ReferencePath reference = makeLineReference();
    SolverInput solver_input;
    SolverParams params;
    VariantConfig variant;
    AcadosRawSolution raw;
    raw.horizon_steps = 2;
    raw.states.resize(10);
    AcadosSolutionDecoderInput input = makeDecoderInput(
        raw, solver_input, reference, params, variant);
    SolverOutput output;

    const AcadosSolutionDecodeResult decoded =
        AcadosSolutionDecoder::decode(input, output);

    EXPECT_FALSE(decoded.valid);
    EXPECT_EQ(decoded.status, "RAW_CARDINALITY");
    EXPECT_FALSE(output.success);
}

TEST(AcadosSolutionDecoder,
     B0GoldenMatrixProducesExactCostsCommandAndNextWarmStart) {
    const ReferencePath reference = makeLineReference();
    SolverInput solver_input;
    solver_input.robot.v = 0.4;
    solver_input.robot.omega = 0.2;
    solver_input.dt = 0.1;
    SolverParams params;
    params.v_max = 1.0;
    params.omega_max = 2.0;
    params.a_max = 1.0;
    params.alpha_max = 2.0;
    VariantConfig variant;
    variant.name = "B0_GOLDEN";
    variant.w_contour = 2.0;
    variant.w_lag = 3.0;
    variant.w_progress = 0.6;
    variant.w_control = 4.0;
    variant.w_accel = 1.0;
    variant.w_v = 7.0;
    variant.w_vs = 8.0;
    variant.w_alpha = 5.0;
    variant.w_du_a = 9.0;
    variant.w_du_vs = 10.0;
    AcadosRawSolution raw;
    raw.horizon_steps = 2;
    appendState(raw, 0.0, 1.0, 0.0, 0.4, 0.0, 0.2);
    appendState(raw, 1.2, 0.0, 0.0, 0.6, 1.0, 0.4);
    appendState(raw, 2.0, -0.5, 0.0, 0.7, 2.0, 0.6);
    appendControl(raw, 0.5, 1.0, 0.6);
    appendControl(raw, -0.5, -1.0, 0.4);
    AcadosSolutionDecoderInput input = makeDecoderInput(
        raw, solver_input, reference, params, variant);
    input.have_previous_control = true;
    input.previous_control = {{0.4, 99.0, 0.5}};
    SolverOutput output;

    const AcadosSolutionDecodeResult decoded =
        AcadosSolutionDecoder::decode(input, output);

    ASSERT_TRUE(decoded.valid) << decoded.status;
    EXPECT_EQ(output.status, "B0_GOLDEN_ACADOS_OK");
    EXPECT_TRUE(output.success);
    EXPECT_DOUBLE_EQ(output.cost.J_contour, 1.5);
    EXPECT_NEAR(output.cost.J_lag, 0.06, 1e-12);
    EXPECT_NEAR(output.cost.J_control, 2.60, 1e-12);
    EXPECT_NEAR(output.cost.J_progress, -0.30, 1e-12);
    EXPECT_NEAR(output.cost.J_v, 0.15, 1e-12);
    EXPECT_NEAR(output.cost.J_smooth, 0.095, 1e-12);
    EXPECT_DOUBLE_EQ(output.cmd_v, 0.45);
    EXPECT_NEAR(output.cmd_omega, 0.30, 1e-12);
    EXPECT_EQ(output.trajectory.size(), 3U);
    EXPECT_EQ(output.predicted_horizon.states.size(), 3U);
    EXPECT_EQ(output.predicted_horizon.controls.size(), 2U);
    EXPECT_DOUBLE_EQ(output.first_shot_debug.u0_a, 0.5);
    EXPECT_DOUBLE_EQ(output.first_shot_debug.u0_alpha, 1.0);
    EXPECT_DOUBLE_EQ(output.first_shot_debug.x2_s, 2.0);
    EXPECT_DOUBLE_EQ(decoded.first_control[2], 0.6);
    ASSERT_TRUE(decoded.solved_warm_start.valid);
    EXPECT_EQ(decoded.solved_warm_start.states.size(), 3U);
    EXPECT_EQ(decoded.solved_warm_start.controls.size(), 2U);
    EXPECT_DOUBLE_EQ(decoded.solved_warm_start.states[1].omega, 0.4);
}

TEST(AcadosSolutionDecoder,
     PhaseRejoinUsesNominalRelativeCostButReportsPhysicalSloshPeak) {
    const ReferencePath reference = makeLineReference();
    SolverInput solver_input;
    solver_input.dt = 0.1;
    solver_input.phase_rejoin.active = true;
    solver_input.phase_rejoin.enforce = true;
    solver_input.phase_rejoin.liquid_steps = 2;
    solver_input.phase_rejoin.stages.resize(3);
    SolverParams params;
    params.v_max = 1.0;
    params.omega_max = 2.0;
    params.a_max = 1.0;
    params.alpha_max = 2.0;
    params.slosh.slosh_eta_dot_ratio = 0.5;
    VariantConfig variant;
    variant.name = "PHASE_GOLDEN";
    variant.slosh_enable = true;
    variant.w_slosh = 6.0;
    variant.w_contour = 2.0;
    variant.w_lag = 3.0;
    variant.w_progress = 4.0;
    variant.w_control = 5.0;
    variant.w_accel = 1.0;
    variant.w_v = 7.0;
    variant.w_vs = 8.0;
    variant.w_alpha = 9.0;
    AcadosRawSolution raw;
    raw.horizon_steps = 2;
    appendState(raw, 0.0, 0.0, 0.0, 0.2, 0.0, 0.1,
                3.0, 0.0, 4.0, 0.0);
    appendState(raw, 1.0, 0.0, 0.0, 0.3, 1.0, 0.2,
                0.0, 2.0, 2.0, 0.0);
    appendState(raw, 2.0, 0.0, 0.0, 0.4, 2.0, 0.3,
                1.0, 0.0, 0.0, 0.0);
    appendControl(raw, 0.1, 0.2, 0.3);
    appendControl(raw, 0.2, 0.3, 0.4);
    for (int stage = 0; stage <= 2; ++stage) {
        const double* state = raw.stateData(stage);
        PhaseNominalStage& nominal = solver_input.phase_rejoin.stages[
            static_cast<std::size_t>(stage)];
        nominal.v = state[3];
        nominal.omega = state[5];
        nominal.eta_x = state[6];
        nominal.eta_x_dot = state[7];
        nominal.eta_y = state[8];
        nominal.eta_y_dot = state[9];
        if (stage < 2) {
            const double* control = raw.controlData(stage);
            nominal.a = control[0];
            nominal.alpha = control[1];
            nominal.v_s = control[2];
        }
    }
    AcadosSolutionDecoderInput input = makeDecoderInput(
        raw, solver_input, reference, params, variant);
    input.slosh_enabled = true;
    input.height_coeff = 2.0;
    input.eta_ref = 1.0;
    input.eta_dot_ref = 1.0;
    SolverOutput output;
    output.slosh_summary.hard_constraint_enable = true;
    output.slosh_summary.h_limit = 12.0;

    const AcadosSolutionDecodeResult decoded =
        AcadosSolutionDecoder::decode(input, output);

    ASSERT_TRUE(decoded.valid) << decoded.status;
    EXPECT_DOUBLE_EQ(output.cost.total(), 0.0);
    EXPECT_DOUBLE_EQ(output.slosh_summary.h_peak_pred, 10.0);
    EXPECT_EQ(output.slosh_summary.peak_k, 0);
    EXPECT_DOUBLE_EQ(output.slosh_summary.h_p95_pred, 4.0);
    EXPECT_DOUBLE_EQ(output.slosh_summary.h_limit_margin, 2.0);
    EXPECT_DOUBLE_EQ(output.slosh_hard_constraint.h_peak_pred, 10.0);
    EXPECT_DOUBLE_EQ(output.predicted_horizon.states[0].h_modal, 10.0);
    EXPECT_DOUBLE_EQ(output.slosh_cost_monitor.J_slosh_total, 0.0);
    EXPECT_DOUBLE_EQ(output.cost.J_progress, 0.0);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
