#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace spmpc_local_planner {
namespace {

SloshModelParams makeSloshParams() {
    SloshModelParams params;
    params.dt = 1.0 / 30.0;
    params.use_parabola_term = false;
    return params;
}

SloshRiskGovernorParams makeGovernorParams() {
    SloshRiskGovernorParams params;
    params.enable = true;
    params.require_slosh_variant = true;
    params.horizon_steps = 10;
    params.height_limit_m = 0.006;
    params.risk_threshold = 1.0;
    params.release_threshold = 0.75;
    params.beta_min = 0.45;
    params.beta_grid_count = 12;
    params.min_v_ref = 0.05;
    params.accel_limit = 0.4;
    params.omega_decay_tau = 0.5;
    params.beta_rate_up_per_sec = 0.5;
    params.beta_rate_down_per_sec = 2.0;
    params.include_parabola_height = true;
    return params;
}

SloshRiskGovernorInput makeInput() {
    SloshRiskGovernorInput input;
    input.robot_v = 0.25;
    input.robot_omega = 0.0;
    input.nominal_v_ref = 0.25;
    input.dt = 1.0 / 30.0;
    input.slosh_variant_enabled = true;
    return input;
}

SloshState makeSloshAtRisk(double risk, double height_limit_m) {
    SloshDynamics dynamics;
    EXPECT_TRUE(dynamics.configure(makeSloshParams()));
    const double height_coeff = std::max(1e-9, dynamics.heightCoeff());
    SloshState slosh;
    slosh.eta_x = risk * height_limit_m / height_coeff;
    return slosh;
}

double candidateBeta(const SloshRiskGovernorParams& params, int index) {
    const int grid = std::max(1, params.beta_grid_count);
    const double beta_min = std::max(0.0, std::min(1.0, params.beta_min));
    const double ratio = grid == 1 ? 0.0 : static_cast<double>(index) / static_cast<double>(grid - 1);
    return 1.0 - ratio * (1.0 - beta_min);
}

double rolloutRiskPeak(const SloshModelParams& slosh_params,
                       const SloshRiskGovernorParams& params,
                       const SloshRiskGovernorInput& input,
                       double beta) {
    SloshModelParams governed_slosh_params = slosh_params;
    governed_slosh_params.dt = input.dt;
    governed_slosh_params.use_parabola_term = params.include_parabola_height;

    SloshDynamics dynamics;
    if (!dynamics.configure(governed_slosh_params) || params.height_limit_m <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    SloshState state = input.slosh;
    double v_sim = std::max(0.0, std::min(input.robot_v, input.nominal_v_ref));
    const double dt = std::max(1e-4, input.dt);
    const double target_v = std::max(0.0, std::min(beta * input.nominal_v_ref, input.nominal_v_ref));
    double risk_peak = dynamics.height(state, input.robot_omega) / params.height_limit_m;

    for (int k = 0; k < std::max(1, params.horizon_steps); ++k) {
        double omega_sim = input.robot_omega;
        if (std::isfinite(params.omega_decay_tau) && params.omega_decay_tau > 0.0) {
            omega_sim *= std::exp(-static_cast<double>(k) * dt / params.omega_decay_tau);
        }
        const double accel_limit = std::max(0.0, params.accel_limit);
        const double ax = std::max(-accel_limit, std::min(accel_limit, (target_v - v_sim) / dt));
        const double ay = v_sim * omega_sim;
        state = dynamics.step(state, ax, ay, omega_sim);
        risk_peak = std::max(risk_peak, dynamics.height(state, omega_sim) / params.height_limit_m);
        v_sim = std::max(0.0, std::min(v_sim + ax * dt, input.nominal_v_ref));
    }
    return risk_peak;
}

int chooseThresholdWithLowerFeasibleCandidate(const SloshModelParams& slosh_params,
                                              SloshRiskGovernorParams& params,
                                              const SloshRiskGovernorInput& input) {
    const int grid = std::max(1, params.beta_grid_count);
    std::vector<double> risks;
    risks.reserve(static_cast<size_t>(grid));
    for (int i = 0; i < grid; ++i) {
        risks.push_back(rolloutRiskPeak(slosh_params, params, input, candidateBeta(params, i)));
    }

    for (int i = 1; i < grid; ++i) {
        if (risks[static_cast<size_t>(i - 1)] > risks[static_cast<size_t>(i)] + 1e-9) {
            params.risk_threshold = 0.5 * (risks[static_cast<size_t>(i - 1)] +
                                           risks[static_cast<size_t>(i)]);
            return i;
        }
    }
    return -1;
}

}  // namespace

TEST(SloshRiskGovernor, DisabledReturnsNominal) {
    auto params = makeGovernorParams();
    params.enable = false;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.nominal_v_ref = 0.31;
    const auto out = governor.update(input);
    EXPECT_FALSE(out.enabled);
    EXPECT_FALSE(out.active);
    EXPECT_DOUBLE_EQ(out.governed_v_ref, 0.31);
    EXPECT_FALSE(out.feasible_found);
    EXPECT_FALSE(out.saturated);
    EXPECT_GE(out.computation_time_ms, 0.0);
    EXPECT_EQ(out.status, "DISABLED");
}

TEST(SloshRiskGovernor, NonSloshVariantBypassesWhenRequired) {
    auto params = makeGovernorParams();
    params.require_slosh_variant = true;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.slosh_variant_enabled = false;
    input.nominal_v_ref = 0.28;
    const auto out = governor.update(input);
    EXPECT_TRUE(out.enabled);
    EXPECT_FALSE(out.active);
    EXPECT_DOUBLE_EQ(out.governed_v_ref, 0.28);
    EXPECT_FALSE(out.feasible_found);
    EXPECT_FALSE(out.saturated);
    EXPECT_EQ(out.status, "NOT_SLOSH_VARIANT");
}

TEST(SloshRiskGovernor, LowRiskKeepsBetaNearOne) {
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), makeGovernorParams()));

    auto input = makeInput();
    const auto out = governor.update(input);
    EXPECT_NEAR(out.beta_raw, 1.0, 1e-12);
    EXPECT_NEAR(out.beta_filtered, 1.0, 1e-12);
    EXPECT_NEAR(out.governed_v_ref, input.nominal_v_ref, 1e-12);
    EXPECT_FALSE(out.active);
    EXPECT_TRUE(out.feasible_found);
    EXPECT_FALSE(out.saturated);
    EXPECT_TRUE(out.predicted_risk_admissible);
    EXPECT_GE(out.risk_margin, 0.0);
    EXPECT_EQ(out.status, "PASS_THROUGH");
}

TEST(SloshRiskGovernor, HighInitialSloshReducesVRef) {
    auto params = makeGovernorParams();
    params.beta_rate_down_per_sec = 10.0;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.dt = 0.2;
    input.slosh = makeSloshAtRisk(1.5, params.height_limit_m);
    const auto out = governor.update(input);
    EXPECT_LT(out.beta_raw, 1.0);
    EXPECT_LT(out.beta_filtered, 1.0);
    EXPECT_LT(out.governed_v_ref, input.nominal_v_ref);
    EXPECT_TRUE(out.active);
    EXPECT_FALSE(out.feasible_found);
    EXPECT_TRUE(out.saturated);
    EXPECT_FALSE(out.predicted_risk_admissible);
    EXPECT_LT(out.risk_margin, 0.0);
    EXPECT_EQ(out.status, "SATURATED");
}

TEST(SloshRiskGovernor, GovernedVRefNeverExceedsNominalAndNeverBelowMin) {
    auto params = makeGovernorParams();
    params.min_v_ref = 0.08;
    params.beta_min = 0.10;
    params.beta_rate_down_per_sec = 10.0;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.dt = 1.0;
    input.nominal_v_ref = 0.20;
    input.slosh = makeSloshAtRisk(2.0, params.height_limit_m);
    const auto out = governor.update(input);
    EXPECT_LE(out.governed_v_ref, input.nominal_v_ref);
    EXPECT_GE(out.governed_v_ref, params.min_v_ref);
    EXPECT_NEAR(out.governed_v_ref, params.min_v_ref, 1e-12);

    input.nominal_v_ref = 0.03;
    const auto below_min_nominal = governor.update(input);
    EXPECT_LE(below_min_nominal.governed_v_ref, input.nominal_v_ref);
    EXPECT_NEAR(below_min_nominal.governed_v_ref, input.nominal_v_ref, 1e-12);
}

TEST(SloshRiskGovernor, IncludeParabolaHeightFalseExcludesParabolaTerm) {
    auto slosh_params = makeSloshParams();
    slosh_params.use_parabola_term = true;

    auto modal_only_params = makeGovernorParams();
    modal_only_params.include_parabola_height = false;
    SloshRiskGovernor modal_only_governor;
    EXPECT_TRUE(modal_only_governor.configure(slosh_params, modal_only_params));

    auto input = makeInput();
    input.robot_v = 0.0;
    input.robot_omega = 3.0;
    input.slosh = SloshState();
    const auto modal_only = modal_only_governor.update(input);
    EXPECT_NEAR(modal_only.h_now_m, 0.0, 1e-12);

    auto total_height_params = makeGovernorParams();
    total_height_params.include_parabola_height = true;
    SloshRiskGovernor total_height_governor;
    EXPECT_TRUE(total_height_governor.configure(slosh_params, total_height_params));

    const auto total_height = total_height_governor.update(input);
    EXPECT_GT(total_height.h_now_m, modal_only.h_now_m);
}

TEST(SloshRiskGovernor, ResetClearsBetaFilterState) {
    auto params = makeGovernorParams();
    params.beta_rate_down_per_sec = 10.0;
    params.beta_rate_up_per_sec = 0.1;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto risky = makeInput();
    risky.dt = 0.2;
    risky.slosh = makeSloshAtRisk(1.5, params.height_limit_m);
    const auto limited = governor.update(risky);
    ASSERT_LT(limited.beta_filtered, 1.0);

    auto calm = makeInput();
    calm.dt = 0.1;
    const auto slow_release = governor.update(calm);
    ASSERT_LT(slow_release.beta_filtered, 1.0);

    governor.reset();
    const auto after_reset = governor.update(calm);
    EXPECT_NEAR(after_reset.beta_filtered, 1.0, 1e-12);
    EXPECT_NEAR(after_reset.governed_v_ref, calm.nominal_v_ref, 1e-12);
}

TEST(SloshRiskGovernor, SelectedBetaIsLargestFeasibleCandidate) {
    auto slosh_params = makeSloshParams();
    auto params = makeGovernorParams();
    params.horizon_steps = 25;
    params.beta_grid_count = 8;
    params.beta_min = 0.0;
    params.min_v_ref = 0.0;
    params.accel_limit = 0.6;
    params.beta_rate_down_per_sec = 100.0;
    params.include_parabola_height = false;

    auto input = makeInput();
    input.robot_v = 0.0;
    input.nominal_v_ref = 0.45;
    input.slosh = SloshState();
    const int expected_index = chooseThresholdWithLowerFeasibleCandidate(slosh_params, params, input);
    ASSERT_GT(expected_index, 0);

    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(slosh_params, params));
    const auto out = governor.update(input);

    EXPECT_TRUE(out.feasible_found);
    EXPECT_FALSE(out.saturated);
    EXPECT_TRUE(out.predicted_risk_admissible);
    EXPECT_EQ(out.selected_candidate_index, expected_index);
    EXPECT_NEAR(out.beta_raw, candidateBeta(params, expected_index), 1e-12);
    EXPECT_NEAR(out.beta_filtered, out.beta_raw, 1e-12);
    EXPECT_GE(out.risk_margin, -1e-12);
}

TEST(SloshRiskGovernor, InfeasibleCandidatesReportSaturated) {
    auto params = makeGovernorParams();
    params.beta_rate_down_per_sec = 100.0;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.dt = 0.2;
    input.slosh = makeSloshAtRisk(2.0, params.height_limit_m);
    const auto out = governor.update(input);

    EXPECT_FALSE(out.feasible_found);
    EXPECT_TRUE(out.saturated);
    EXPECT_FALSE(out.predicted_risk_admissible);
    EXPECT_LT(out.risk_margin, 0.0);
    EXPECT_EQ(out.status, "SATURATED");
}

TEST(SloshRiskGovernor, FilteredBetaStatusReportsTransientIfNotAdmissible) {
    auto slosh_params = makeSloshParams();
    auto params = makeGovernorParams();
    params.horizon_steps = 25;
    params.beta_grid_count = 8;
    params.beta_min = 0.0;
    params.min_v_ref = 0.0;
    params.accel_limit = 0.6;
    params.beta_rate_down_per_sec = 0.0;
    params.include_parabola_height = false;

    auto input = makeInput();
    input.robot_v = 0.0;
    input.nominal_v_ref = 0.45;
    input.slosh = SloshState();
    const int expected_index = chooseThresholdWithLowerFeasibleCandidate(slosh_params, params, input);
    ASSERT_GT(expected_index, 0);

    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(slosh_params, params));
    const auto out = governor.update(input);

    EXPECT_TRUE(out.feasible_found);
    EXPECT_FALSE(out.saturated);
    EXPECT_GT(out.beta_filtered, out.beta_raw);
    EXPECT_FALSE(out.predicted_risk_admissible);
    EXPECT_LT(out.risk_margin, 0.0);
    EXPECT_EQ(out.status, "TRANSIENT_RATE_LIMITED");
}

TEST(SloshRiskGovernor, GovernedReferenceNeverAmplifiesNominal) {
    auto params = makeGovernorParams();
    params.min_v_ref = 0.20;
    SloshRiskGovernor governor;
    EXPECT_TRUE(governor.configure(makeSloshParams(), params));

    auto input = makeInput();
    input.nominal_v_ref = 0.03;
    const auto out = governor.update(input);

    EXPECT_LE(out.governed_v_ref, input.nominal_v_ref);
    EXPECT_NEAR(out.governed_v_ref, input.nominal_v_ref, 1e-12);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
