#include <gtest/gtest.h>

#include "lt_dwa_official_wrapper/frame_validator.hpp"

namespace lt_dwa_official_wrapper {
namespace {

Pose2d MakePose(double x, double y) {
  Pose2d pose;
  pose.frame_id = "odom";
  pose.stamp = ros::Time(10.0);
  pose.x = x;
  pose.y = y;
  pose.yaw = 0.0;
  return pose;
}

nav_msgs::OccupancyGrid MakeMap(const std::string& frame = "odom") {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = frame;
  map.header.stamp = ros::Time(10.0);
  map.info.width = 2;
  map.info.height = 2;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 0, 0, 0};
  return map;
}

PlannerInput MakeInput() {
  PlannerInput input;
  input.planning_frame = "odom";
  input.stamp = ros::Time(10.0);
  input.robot_pose = MakePose(0.0, 0.0);
  input.target_pose = MakePose(1.0, 0.0);
  input.robot_twist.v = 0.1;
  input.robot_twist.w = 0.0;
  input.reference_path = {MakePose(0.0, 0.0), MakePose(1.0, 0.0)};
  input.occupancy_grid = MakeMap();
  return input;
}

TEST(FrameValidatorTest, AcceptsValidInputInConfiguredFrame) {
  PlannerConfig config;
  FrameValidator validator;

  const auto result = validator.ValidateInput(MakeInput(), config, ros::Time(10.1));

  EXPECT_EQ(result.status, WrapperStatus::kOk);
  EXPECT_TRUE(result.ok());
}

TEST(FrameValidatorTest, RejectsFrameMismatch) {
  PlannerConfig config;
  FrameValidator validator;
  auto input = MakeInput();
  input.occupancy_grid = MakeMap("map");

  const auto result = validator.ValidateInput(input, config, ros::Time(10.1));

  EXPECT_EQ(result.status, WrapperStatus::kInvalidFrame);
}

TEST(FrameValidatorTest, RejectsEmptyPath) {
  PlannerConfig config;
  FrameValidator validator;
  auto input = MakeInput();
  input.reference_path.clear();

  const auto result = validator.ValidateInput(input, config, ros::Time(10.1));

  EXPECT_EQ(result.status, WrapperStatus::kEmptyPath);
}

TEST(FrameValidatorTest, RejectsStaleInputWhenNowIsProvided) {
  PlannerConfig config;
  config.input_stale_timeout_sec = 0.25;
  FrameValidator validator;
  auto input = MakeInput();
  input.stamp = ros::Time(9.0);

  const auto result = validator.ValidateInput(input, config, ros::Time(10.0));

  EXPECT_EQ(result.status, WrapperStatus::kStaleInput);
}

TEST(FrameValidatorTest, RejectsInvalidMapShape) {
  PlannerConfig config;
  FrameValidator validator;
  auto input = MakeInput();
  input.occupancy_grid.data.pop_back();

  const auto result = validator.ValidateInput(input, config, ros::Time(10.1));

  EXPECT_EQ(result.status, WrapperStatus::kInvalidMap);
}

}  // namespace
}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
