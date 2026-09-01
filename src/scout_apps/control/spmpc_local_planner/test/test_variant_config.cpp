#include "spmpc_local_planner/core/variant_config.h"

#include <gtest/gtest.h>

#include <limits>

namespace spmpc_local_planner {
namespace {

TEST(VariantConfig, ShortLiquidCostStopsAfterOneHundredMillisecondsAtThirtyHz) {
    VariantConfig config;
    config.slosh_cost_horizon_steps = 3;
    config.slosh_cost_tail_discount = 0.0;
    for (int stage = 0; stage <= 3; ++stage) {
        EXPECT_DOUBLE_EQ(sloshCostStageScale(config, stage, 60), 1.0);
    }
    for (int stage = 4; stage <= 60; ++stage) {
        EXPECT_DOUBLE_EQ(sloshCostStageScale(config, stage, 60), 0.0);
    }
}

TEST(VariantConfig, HistoricalNegativeHorizonKeepsFullCost) {
    VariantConfig config;
    config.slosh_cost_horizon_steps = -1;
    config.slosh_cost_tail_discount = 0.0;
    EXPECT_DOUBLE_EQ(sloshCostStageScale(config, 60, 60), 1.0);
}

TEST(VariantConfig, MatchedPairMayDifferOnlyInSloshWeight) {
    VariantConfig matched0;
    matched0.name = "B_slosh_matched0";
    matched0.slosh_enable = true;
    matched0.smooth_priority_enable = true;
    matched0.w_control = 0.3;
    matched0.w_smooth = 1.0;
    matched0.w_alpha = 1.0;
    matched0.w_du_a = 1.0;
    matched0.w_du_vs = 1.0;
    matched0.w_slosh = 0.0;
    matched0.slosh_cost_horizon_steps = 3;
    matched0.slosh_cost_tail_discount = 0.0;
    VariantConfig matched5 = matched0;
    matched5.name = "B_slosh_matched5";
    matched5.w_slosh = 5.0;
    EXPECT_TRUE(matchedVariantCommonConfigEqual(matched0, matched5));
    matched5.w_control = 0.1;
    EXPECT_FALSE(matchedVariantCommonConfigEqual(matched0, matched5));
}

TEST(VariantConfig, RecognizesDevelopmentMatchedNames) {
    EXPECT_EQ(makeVariantConfig("B_slosh_matched0").name,
              "B_slosh_matched0");
    EXPECT_EQ(makeVariantConfig("B_slosh_matched5").name,
              "B_slosh_matched5");
}

TEST(VariantConfig, RecognizesLiteralShortOneHundredMillisecondName) {
    EXPECT_EQ(makeVariantConfig("B_slosh_short100").name,
              "B_slosh_short100");
}

TEST(VariantConfig, MatchedReleaseAllowsOnlyConservativePositiveSpeed) {
    EXPECT_TRUE(matchedVariantReleaseSpeedAllowed(0.10));
    EXPECT_TRUE(matchedVariantReleaseSpeedAllowed(0.15));
    EXPECT_TRUE(matchedVariantReleaseSpeedAllowed(0.20));
    EXPECT_FALSE(matchedVariantReleaseSpeedAllowed(0.0));
    EXPECT_FALSE(matchedVariantReleaseSpeedAllowed(-0.10));
    EXPECT_FALSE(matchedVariantReleaseSpeedAllowed(0.200001));
    EXPECT_FALSE(matchedVariantReleaseSpeedAllowed(
        std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
