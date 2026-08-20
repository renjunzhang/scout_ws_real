#include "spmpc_local_planner/tools/short_horizon_matched_preflight.h"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace spmpc_local_planner {
namespace tools {
namespace {

std::string variantDump(const std::string& name, double slosh_weight) {
    const std::string prefix =
        "/spmpc_local_planner/variants/" + name + "/";
    std::ostringstream output;
    output << prefix << "primitive_mode: linear\n"
           << prefix << "slosh_constraint_enable: false\n"
           << prefix << "slosh_cost_horizon_steps: 3\n"
           << prefix << "slosh_cost_tail_discount: 0.0\n"
           << prefix << "slosh_enable: true\n"
           << prefix << "smooth_priority_enable: true\n"
           << prefix << "v_ref: 0.2\n"
           << prefix << "w_accel: 0.0\n"
           << prefix << "w_alpha: 1.0\n"
           << prefix << "w_contour: 1.0\n"
           << prefix << "w_control: 0.3\n"
           << prefix << "w_du_a: 1.0\n"
           << prefix << "w_du_vs: 1.0\n"
           << prefix << "w_lag: 0.2\n"
           << prefix << "w_progress: 0.2\n"
           << prefix << "w_slosh: " << slosh_weight << "\n"
           << prefix << "w_smooth: 1.0\n"
           << prefix << "w_v: 1.0\n"
           << prefix << "w_vs: 0.3\n";
    return output.str();
}

std::string validDump() {
    return "/unrelated/parameter: retained\n" +
        variantDump("B_slosh_matched0", 0.0) +
        variantDump("B_slosh_matched5", 5.0);
}

void replaceOnce(std::string& text,
                 const std::string& from,
                 const std::string& to) {
    const std::size_t position = text.find(from);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, from.size(), to);
}

TEST(ShortHorizonMatchedPreflight, AcceptsFrozenExpandedPair) {
    const auto result = validateShortHorizonMatchedParamDump(validDump());
    EXPECT_TRUE(result.success) << result.detail;
    EXPECT_EQ(result.detail, "OK");
}

TEST(ShortHorizonMatchedPreflight, RejectsAnyCommonFieldDifference) {
    std::string dump = validDump();
    replaceOnce(
        dump,
        "/B_slosh_matched5/w_control: 0.3",
        "/B_slosh_matched5/w_control: 0.4");
    const auto result = validateShortHorizonMatchedParamDump(dump);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.detail, "MATCHED_COMMON_CONFIG_MISMATCH");
}

TEST(ShortHorizonMatchedPreflight, RejectsMissingAndUnknownFields) {
    std::string missing = validDump();
    replaceOnce(
        missing,
        "/B_slosh_matched0/w_alpha: 1.0",
        "/ignored/B_slosh_matched0/w_alpha: 1.0");
    const auto missing_result =
        validateShortHorizonMatchedParamDump(missing);
    EXPECT_FALSE(missing_result.success);
    EXPECT_EQ(missing_result.detail,
              "MISSING_FIELD_B_slosh_matched0_w_alpha");

    const auto unknown_result = validateShortHorizonMatchedParamDump(
        validDump() +
        "/spmpc_local_planner/variants/B_slosh_matched5/new_weight: 1\n");
    EXPECT_FALSE(unknown_result.success);
    EXPECT_EQ(unknown_result.detail, "UNKNOWN_MATCHED_FIELD_new_weight");
}

TEST(ShortHorizonMatchedPreflight, RejectsWrongWeightAndNonfiniteValue) {
    std::string wrong_weight = validDump();
    replaceOnce(
        wrong_weight,
        "/B_slosh_matched5/w_slosh: 5",
        "/B_slosh_matched5/w_slosh: 4");
    const auto weight_result =
        validateShortHorizonMatchedParamDump(wrong_weight);
    EXPECT_FALSE(weight_result.success);
    EXPECT_EQ(weight_result.detail,
              "MATCHED_SLOSH_WEIGHT_CONTRACT_MISMATCH");

    std::string nonfinite = validDump();
    replaceOnce(
        nonfinite,
        "/B_slosh_matched0/w_v: 1.0",
        "/B_slosh_matched0/w_v: nan");
    const auto nonfinite_result =
        validateShortHorizonMatchedParamDump(nonfinite);
    EXPECT_FALSE(nonfinite_result.success);
    EXPECT_EQ(nonfinite_result.detail,
              "INVALID_NUMBER_B_slosh_matched0_w_v");
}

}  // namespace
}  // namespace tools
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
