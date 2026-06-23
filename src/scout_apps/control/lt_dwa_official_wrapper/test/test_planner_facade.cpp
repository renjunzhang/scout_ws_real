#include <gtest/gtest.h>

#include <string>

#include "lt_dwa_official_wrapper/planner_facade.hpp"

namespace lt_dwa_official_wrapper {
namespace {

Pose2d MakePose(double x, double y, const std::string& frame = "odom") {
  Pose2d pose;
  pose.frame_id = frame;
  pose.stamp = ros::Time(20.0);
  pose.x = x;
  pose.y = y;
  pose.yaw = 0.0;
  return pose;
}

nav_msgs::OccupancyGrid MakeMap(const std::string& frame = "odom") {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = frame;
  map.header.stamp = ros::Time(20.0);
  map.info.width = 4;
  map.info.height = 3;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

PlannerInput MakeInput() {
  PlannerInput input;
  input.planning_frame = "odom";
  input.stamp = ros::Time(20.0);
  input.robot_pose = MakePose(0.0, 0.0);
  input.target_pose = MakePose(1.0, 0.0);
  input.robot_twist.v = 0.2;
  input.robot_twist.w = 0.1;
  input.reference_path = {MakePose(0.0, 0.0), MakePose(0.5, 0.0), MakePose(1.0, 0.0)};
  input.occupancy_grid = MakeMap();

  ObstacleTrack obstacle;
  obstacle.id = 1;
  obstacle.frame_id = "odom";
  obstacle.stamp = ros::Time(20.0);
  obstacle.x = 0.4;
  obstacle.y = 0.2;
  obstacle.vx = 0.0;
  obstacle.vy = 0.0;
  obstacle.radius = 0.15;
  input.dynamic_obstacles.push_back(obstacle);
  return input;
}

TEST(PlannerFacadeTest, ValidInputConvertsButRejectsCommandBecauseCoreCallIsDisabled) {
  PlannerConfig config;
  config.deterministic_seed = 42;
  PlannerFacade facade(config);

  const auto output = facade.PlanOnce(MakeInput(), ros::Time(20.1));

  EXPECT_EQ(output.status, WrapperStatus::kCommandRejected);
  EXPECT_NE(output.diagnostics.reject_reason.find("core call disabled"), std::string::npos);
  EXPECT_TRUE(output.diagnostics.command_rejected);
  EXPECT_DOUBLE_EQ(output.command_raw.v, 0.0);
  EXPECT_DOUBLE_EQ(output.command_raw.w, 0.0);
  EXPECT_EQ(output.diagnostics.path_points_raw, 3u);
  EXPECT_GT(output.diagnostics.path_points_resampled, output.diagnostics.path_points_raw);
  EXPECT_NEAR(output.diagnostics.path_length_m, 1.0, 1.0e-9);
  EXPECT_EQ(output.diagnostics.map_width, 4u);
  EXPECT_EQ(output.diagnostics.map_height, 3u);
  EXPECT_NEAR(output.diagnostics.map_resolution, 0.1, 1.0e-6);
  EXPECT_EQ(output.diagnostics.obstacle_count, 1u);
  EXPECT_EQ(output.diagnostics.deterministic_seed, 42u);
}

TEST(PlannerFacadeTest, InvalidFrameReturnsValidationFailureAndZeroCommand) {
  PlannerConfig config;
  PlannerFacade facade(config);
  auto input = MakeInput();
  input.reference_path[1].frame_id = "map";

  const auto output = facade.PlanOnce(input, ros::Time(20.1));

  EXPECT_EQ(output.status, WrapperStatus::kInvalidFrame);
  EXPECT_TRUE(output.diagnostics.command_rejected);
  EXPECT_DOUBLE_EQ(output.command_raw.v, 0.0);
  EXPECT_DOUBLE_EQ(output.command_raw.w, 0.0);
}

TEST(PlannerFacadeTest, InvalidMapReturnsValidationFailureAndZeroCommand) {
  PlannerConfig config;
  PlannerFacade facade(config);
  auto input = MakeInput();
  input.occupancy_grid.data.pop_back();

  const auto output = facade.PlanOnce(input, ros::Time(20.1));

  EXPECT_EQ(output.status, WrapperStatus::kInvalidMap);
  EXPECT_TRUE(output.diagnostics.command_rejected);
  EXPECT_DOUBLE_EQ(output.command_raw.v, 0.0);
  EXPECT_DOUBLE_EQ(output.command_raw.w, 0.0);
}

TEST(PlannerFacadeTest, StaleInputReturnsValidationFailure) {
  PlannerConfig config;
  config.input_stale_timeout_sec = 0.25;
  PlannerFacade facade(config);
  auto input = MakeInput();
  input.stamp = ros::Time(19.0);

  const auto output = facade.PlanOnce(input, ros::Time(20.0));

  EXPECT_EQ(output.status, WrapperStatus::kStaleInput);
  EXPECT_TRUE(output.diagnostics.command_rejected);
  EXPECT_GT(output.diagnostics.input_stamp_age_sec, config.input_stale_timeout_sec);
}

}  // namespace
}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
