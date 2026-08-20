#include "spmpc_local_planner/config/app_config.h"
#include "spmpc_local_planner/reference/speed_profile.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace spmpc_local_planner {
namespace {

std::string fixturePath(const std::string& name) {
    std::string source = __FILE__;
    const std::string marker = "/test/test_speed_profile.cpp";
    const auto offset = source.rfind(marker);
    EXPECT_NE(offset, std::string::npos);
    return source.substr(0, offset) + "/test/fixtures/" + name;
}

TEST(SpeedProfile, LoadsSortsSkipsAndInterpolatesHistoricalCsvContract) {
    SpeedProfile profile;
    const auto result = profile.loadCsv(fixturePath("speed_profile.csv"));

    ASSERT_TRUE(result.success) << result.status << ": " << result.detail;
    EXPECT_EQ(result.status, "OK");
    EXPECT_EQ(result.accepted_rows, 4u);
    EXPECT_EQ(result.skipped_rows, 1u);
    ASSERT_EQ(profile.size(), 4u);

    double speed = -1.0;
    ASSERT_TRUE(profile.lookup(-1.0, speed));
    EXPECT_DOUBLE_EQ(speed, 0.10);
    ASSERT_TRUE(profile.lookup(0.5, speed));
    EXPECT_NEAR(speed, 0.20, 1e-12);
    ASSERT_TRUE(profile.lookup(1.5, speed));
    EXPECT_NEAR(speed, 0.25, 1e-12);
    ASSERT_TRUE(profile.lookup(99.0, speed));
    EXPECT_DOUBLE_EQ(speed, 0.40);
    EXPECT_FALSE(profile.lookup(std::nan(""), speed));
}

TEST(SpeedProfile, RejectsUnknownHeaderWithoutLeavingPartialState) {
    SpeedProfile profile;
    const auto result = profile.loadCsv(
        fixturePath("speed_profile_bad_header.csv"));
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "PROFILE_HEADER_INVALID");
    EXPECT_TRUE(profile.empty());
}

TEST(AppConfig, NormalizesInvalidRuntimeVRefAtTheTypedBoundary) {
    AppConfig config;
    config.map_vref.runtime_override_enable = true;
    config.map_vref.runtime_override_mps = -0.1;
    config.map_vref.profile_lookahead_m = -2.0;
    config.map_vref.profile_enable = true;

    const ValidationReport report = validateAndNormalize(config);
    EXPECT_TRUE(report.ok());
    EXPECT_FALSE(config.map_vref.runtime_override_enable);
    EXPECT_DOUBLE_EQ(config.map_vref.profile_lookahead_m, 0.0);
    EXPECT_EQ(report.issues().size(), 3u);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
