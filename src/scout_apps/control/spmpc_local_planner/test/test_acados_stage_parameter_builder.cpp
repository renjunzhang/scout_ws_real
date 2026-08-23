#include "spmpc_local_planner/solver/acados/stage_parameter_builder.h"

#include "spmpc_parameter_manifest.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace spmpc_local_planner {
namespace {

using namespace acados_manifest::mainline;

void expectStageEquals(const AcadosStageParameterMatrix& matrix,
                       int stage,
                       const std::vector<double>& expected) {
    ASSERT_TRUE(matrix.valid) << matrix.status;
    ASSERT_EQ(matrix.parameter_width, static_cast<int>(expected.size()));
    const double* actual = matrix.stageData(stage);
    ASSERT_NE(actual, nullptr);
    for (int index = 0; index < matrix.parameter_width; ++index) {
        SCOPED_TRACE(
            std::string("stage=") + std::to_string(stage) +
            " parameter=" + matrix.parameter_names[
                static_cast<std::size_t>(index)]);
        EXPECT_DOUBLE_EQ(actual[index], expected[static_cast<std::size_t>(index)]);
    }
}

AcadosStageParameterInput makeInput(bool slosh_enabled, int horizon_steps) {
    AcadosStageParameterInput input;
    input.horizon_steps = horizon_steps;
    input.slosh_enabled = slosh_enabled;
    input.reference_x_coeffs = {{1.0, 2.0, 3.0, 4.0}};
    input.reference_y_coeffs = {{5.0, 6.0, 7.0, 8.0}};
    input.variant.w_contour = 9.0;
    input.variant.w_lag = 10.0;
    input.variant.w_progress = 11.0;
    input.variant.w_heading = 11.1;
    input.variant.w_progress_coupling = 11.2;
    input.variant.w_yaw_rate_tracking = 11.3;
    input.variant.heading_feedback_gain = 11.4;
    input.variant.w_control = 12.0;
    input.variant.w_accel = 0.5;
    input.variant.w_v = 13.0;
    input.variant.w_vs = 14.0;
    input.variant.w_alpha = 15.0;
    input.variant.w_du_a = 16.0;
    input.variant.w_du_vs = 17.0;
    input.contour_error_ref = 18.0;
    input.lag_error_ref = 19.0;
    input.effective_v_ref = 20.0;
    return input;
}

std::vector<double> expectedCommon(
    const AcadosStageParameterInput& input,
    int width) {
    std::vector<double> expected(static_cast<std::size_t>(width), 0.0);
    expected[RX0] = 1.0;
    expected[RX1] = 2.0;
    expected[RX2] = 3.0;
    expected[RX3] = 4.0;
    expected[RY0] = 5.0;
    expected[RY1] = 6.0;
    expected[RY2] = 7.0;
    expected[RY3] = 8.0;
    expected[W_CONTOUR] = input.variant.w_contour;
    expected[W_LAG] = input.variant.w_lag;
    expected[W_PROGRESS] = input.variant.w_progress;
    expected[W_HEADING] = input.variant.w_heading;
    expected[W_PROGRESS_COUPLING] = input.variant.w_progress_coupling;
    expected[W_YAW_RATE_TRACKING] = input.variant.w_yaw_rate_tracking;
    expected[HEADING_FEEDBACK_GAIN] = input.variant.heading_feedback_gain;
    expected[W_A] = input.variant.w_control + input.variant.w_accel;
    expected[W_OMEGA] = input.variant.w_control;
    expected[W_V] = input.variant.w_v;
    expected[W_VS] = input.variant.w_vs;
    expected[W_ALPHA] = input.variant.w_alpha;
    expected[E_C_REF] = input.contour_error_ref;
    expected[E_L_REF] = input.lag_error_ref;
    expected[V_REF] = input.effective_v_ref;
    return expected;
}

void setExpectedSloshBase(const AcadosStageParameterInput& input,
                          std::vector<double>& expected) {
    expected[TWO_ZETA_OMEGA_N] = input.slosh.two_zeta_omega_n;
    expected[OMEGA_N_SQ] = input.slosh.omega_n_sq;
    expected[KAPPA_X] = 1.0;
    expected[KAPPA_Y] = 1.0;
    expected[ETA_REF] = input.slosh.eta_ref;
    expected[ETA_DOT_REF] = input.slosh.eta_dot_ref;
    expected[ETA_MAX_SQ] = input.slosh.eta_max_sq;
    expected[GATE_R_X] = 1.0;
    expected[GATE_R_Y] = 1.0;
    expected[GATE_R_YAW] = 1.0;
    expected[GATE_R_V] = 1.0;
    expected[GATE_R_OMEGA] = 1.0;
    expected[GATE_R_ETA_X] = 1.0;
    expected[GATE_R_ETA_X_DOT] = 1.0;
    expected[GATE_R_ETA_Y] = 1.0;
    expected[GATE_R_ETA_Y_DOT] = 1.0;
}

PhaseNominalStage makeNominalStage(int stage, bool gate_active) {
    const double base = static_cast<double>(stage * 20);
    PhaseNominalStage nominal;
    nominal.valid = true;
    nominal.gate_active = gate_active;
    nominal.x = base + 1.0;
    nominal.y = base + 2.0;
    nominal.yaw = base + 3.0;
    nominal.v = base + 4.0;
    nominal.omega = base + 5.0;
    nominal.eta_x = base + 6.0;
    nominal.eta_x_dot = base + 7.0;
    nominal.eta_y = base + 8.0;
    nominal.eta_y_dot = base + 9.0;
    nominal.a = base + 10.0;
    nominal.alpha = base + 11.0;
    nominal.v_s = base + 12.0;
    nominal.radii.x = base + 13.0;
    nominal.radii.y = base + 14.0;
    nominal.radii.yaw = base + 15.0;
    nominal.radii.v = base + 16.0;
    nominal.radii.omega = base + 17.0;
    nominal.radii.eta_x = base + 18.0;
    nominal.radii.eta_x_dot = base + 19.0;
    nominal.radii.eta_y = base + 20.0;
    nominal.radii.eta_y_dot = base + 21.0;
    return nominal;
}

void setExpectedNominal(const PhaseNominalStage& nominal,
                        std::vector<double>& expected) {
    expected[PHASE_REJOIN_ACTIVE] = 1.0;
    expected[NOM_X] = nominal.x;
    expected[NOM_Y] = nominal.y;
    expected[NOM_YAW] = nominal.yaw;
    expected[NOM_V] = nominal.v;
    expected[NOM_OMEGA] = nominal.omega;
    expected[NOM_ETA_X] = nominal.eta_x;
    expected[NOM_ETA_X_DOT] = nominal.eta_x_dot;
    expected[NOM_ETA_Y] = nominal.eta_y;
    expected[NOM_ETA_Y_DOT] = nominal.eta_y_dot;
    expected[NOM_A] = nominal.a;
    expected[NOM_ALPHA] = nominal.alpha;
    expected[NOM_V_S] = nominal.v_s;
    expected[EMPIRICAL_GATE_ACTIVE] = nominal.gate_active ? 1.0 : 0.0;
    expected[GATE_R_X] = nominal.radii.x;
    expected[GATE_R_Y] = nominal.radii.y;
    expected[GATE_R_YAW] = nominal.radii.yaw;
    expected[GATE_R_V] = nominal.radii.v;
    expected[GATE_R_OMEGA] = nominal.radii.omega;
    expected[GATE_R_ETA_X] = nominal.radii.eta_x;
    expected[GATE_R_ETA_X_DOT] = nominal.radii.eta_x_dot;
    expected[GATE_R_ETA_Y] = nominal.radii.eta_y;
    expected[GATE_R_ETA_Y_DOT] = nominal.radii.eta_y_dot;
}

TEST(AcadosStageParameterBuilder, B0MatchesAllGeneratedParameterSlots) {
    AcadosStageParameterInput input = makeInput(false, 2);
    input.have_previous_control = true;
    input.previous_control = {{21.0, 22.0, 23.0}};

    const AcadosStageParameterMatrix matrix =
        AcadosStageParameterBuilder::build(input);

    ASSERT_TRUE(matrix.valid) << matrix.status;
    EXPECT_EQ(matrix.stage_count, 3);
    EXPECT_EQ(matrix.parameter_width, kB0ParameterCount);
    ASSERT_EQ(matrix.parameter_names.size(),
              static_cast<std::size_t>(kB0ParameterCount));
    EXPECT_EQ(matrix.parameter_names.front(), "rx0");
    EXPECT_EQ(matrix.parameter_names.back(), "v_ref");

    std::vector<double> expected = expectedCommon(input, kB0ParameterCount);
    expected[W_DU_A] = input.variant.w_du_a;
    expected[W_DU_VS] = input.variant.w_du_vs;
    expected[A_PREV] = input.previous_control[0];
    expected[VS_PREV] = input.previous_control[2];
    expectStageEquals(matrix, 0, expected);

    expected[W_DU_A] = 0.0;
    expected[W_DU_VS] = 0.0;
    expected[A_PREV] = 0.0;
    expected[VS_PREV] = 0.0;
    expectStageEquals(matrix, 1, expected);
    expectStageEquals(matrix, 2, expected);
}

TEST(AcadosStageParameterBuilder,
     SloshMonitorMatchesWeightsCapsAndInactiveGateAtEveryStage) {
    AcadosStageParameterInput input = makeInput(true, 3);
    input.variant.w_slosh = 8.0;
    input.variant.slosh_cost_horizon_steps = 1;
    input.variant.slosh_cost_tail_discount = 0.5;
    input.slosh.two_zeta_omega_n = 0.25;
    input.slosh.omega_n_sq = 144.0;
    input.slosh.eta_ref = 0.006;
    input.slosh.eta_dot_ref = 0.072;
    input.slosh.eta_max_sq = 0.04;
    input.slosh.eta_dot_weight_ratio = 0.25;
    // Monitor context must not alter any generated parameter, even if its
    // diagnostic stage payload is incomplete.
    input.phase_rejoin.active = true;
    input.phase_rejoin.enforce = false;
    input.phase_rejoin.liquid_steps = 2;

    const AcadosStageParameterMatrix matrix =
        AcadosStageParameterBuilder::build(input);

    ASSERT_TRUE(matrix.valid) << matrix.status;
    EXPECT_EQ(matrix.parameter_width, kSloshParameterCount);
    std::vector<double> expected = expectedCommon(input, kSloshParameterCount);
    setExpectedSloshBase(input, expected);
    const double stage_scales[] = {1.0, 1.0, 0.5, 0.5};
    for (int stage = 0; stage <= input.horizon_steps; ++stage) {
        expected[W_SLOSH_ETA] =
            input.variant.w_slosh * stage_scales[stage];
        expected[W_SLOSH_ETA_DOT] =
            input.variant.w_slosh * input.slosh.eta_dot_weight_ratio *
            stage_scales[stage];
        expectStageEquals(matrix, stage, expected);
    }
}

TEST(AcadosStageParameterBuilder,
     PhaseRejoinInjectsNominalPrefixAndClearsTheUntrustedTail) {
    AcadosStageParameterInput input = makeInput(true, 3);
    input.variant.w_slosh = 6.0;
    input.slosh.eta_dot_weight_ratio = 0.5;
    input.slosh.eta_max_sq = 0.09;
    input.phase_rejoin.active = true;
    input.phase_rejoin.enforce = true;
    input.phase_rejoin.liquid_steps = 2;
    input.phase_rejoin.stages.push_back(makeNominalStage(0, false));
    input.phase_rejoin.stages.push_back(makeNominalStage(1, false));
    input.phase_rejoin.stages.push_back(makeNominalStage(2, true));

    const AcadosStageParameterMatrix matrix =
        AcadosStageParameterBuilder::build(input);

    ASSERT_TRUE(matrix.valid) << matrix.status;
    std::vector<double> expected = expectedCommon(input, kSloshParameterCount);
    setExpectedSloshBase(input, expected);
    expected[W_SLOSH_ETA] = input.variant.w_slosh;
    expected[W_SLOSH_ETA_DOT] =
        input.variant.w_slosh * input.slosh.eta_dot_weight_ratio;
    for (int stage = 0; stage <= input.phase_rejoin.liquid_steps; ++stage) {
        std::vector<double> expected_stage = expected;
        setExpectedNominal(
            input.phase_rejoin.stages[static_cast<std::size_t>(stage)],
            expected_stage);
        expectStageEquals(matrix, stage, expected_stage);
    }

    std::vector<double> expected_tail = expectedCommon(
        input, kSloshParameterCount);
    setExpectedSloshBase(input, expected_tail);
    expected_tail[W_SLOSH_ETA] = 0.0;
    expected_tail[W_SLOSH_ETA_DOT] = 0.0;
    expected_tail[ETA_MAX_SQ] = kAcadosDisabledEtaMaxSq;
    expectStageEquals(matrix, 3, expected_tail);
}

TEST(AcadosStageParameterBuilder,
     RejectsUnsafePhaseStageCardinalityBeforeCapsuleUpdate) {
    AcadosStageParameterInput input = makeInput(true, 3);
    input.phase_rejoin.active = true;
    input.phase_rejoin.enforce = true;
    input.phase_rejoin.liquid_steps = 2;
    input.phase_rejoin.stages.push_back(makeNominalStage(0, false));

    const AcadosStageParameterMatrix matrix =
        AcadosStageParameterBuilder::build(input);

    EXPECT_FALSE(matrix.valid);
    EXPECT_EQ(matrix.status, "PHASE_REJOIN_STAGE_COUNT");
    EXPECT_TRUE(matrix.values.empty());
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
