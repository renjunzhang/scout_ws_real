#include <gtest/gtest.h>

#include <string>

#include "lt_dwa_official_wrapper/worker_request.hpp"

namespace lt_dwa_official_wrapper {
namespace {

Pose2d MakePose(double x, double y, const std::string& frame = "odom") {
  Pose2d pose;
  pose.frame_id = frame;
  pose.stamp = ros::Time(30.0);
  pose.x = x;
  pose.y = y;
  pose.yaw = 0.0;
  return pose;
}

nav_msgs::OccupancyGrid MakeMap(const std::string& frame = "odom") {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = frame;
  map.header.stamp = ros::Time(30.0);
  map.info.width = 3;
  map.info.height = 2;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 100, -1, 0, 0, 0};
  return map;
}

PlannerInput MakeInput() {
  PlannerInput input;
  input.planning_frame = "odom";
  input.stamp = ros::Time(30.0);
  input.robot_pose = MakePose(0.0, 0.0);
  input.robot_twist.v = 0.2;
  input.robot_twist.w = 0.1;
  input.target_pose = MakePose(1.0, 0.0);
  input.reference_path = {MakePose(0.0, 0.0), MakePose(0.5, 0.0), MakePose(1.0, 0.0)};
  input.occupancy_grid = MakeMap();

  ObstacleTrack obstacle;
  obstacle.id = 3;
  obstacle.frame_id = "odom";
  obstacle.stamp = ros::Time(30.0);
  obstacle.x = 0.3;
  obstacle.y = 0.2;
  obstacle.vx = 0.0;
  obstacle.vy = 0.1;
  obstacle.radius = 0.2;
  input.dynamic_obstacles.push_back(obstacle);
  return input;
}

}  // namespace

TEST(WorkerRequestTest, RoundTripsPlannerInputAndPlannerConfig) {
  const auto input = MakeInput();
  PlannerConfig config;
  config.planning_frame = "odom";
  config.max_v = 0.8;
  config.max_w = 1.2;
  config.max_acc = 0.6;
  config.max_angular_acc = 1.2;
  config.path_resample_spacing = 0.08;
  config.enable_path_tracking_guard = true;
  config.path_tracking_lookahead_m = 0.72;
  config.path_tracking_min_v = 0.16;
  const std::string serialized = SerializeWorkerRequest(input, config, ros::Time(30.1));

  const auto parsed = ParseWorkerRequestText(serialized);

  ASSERT_TRUE(parsed.ok) << parsed.reason;
  EXPECT_NEAR(parsed.now.toSec(), 30.1, 1.0e-9);
  EXPECT_EQ(parsed.input.planning_frame, "odom");
  EXPECT_NEAR(parsed.input.stamp.toSec(), 30.0, 1.0e-9);
  EXPECT_EQ(parsed.input.reference_path.size(), 3u);
  EXPECT_EQ(parsed.input.occupancy_grid.info.width, 3u);
  EXPECT_EQ(parsed.input.occupancy_grid.info.height, 2u);
  EXPECT_EQ(parsed.input.occupancy_grid.data.size(), 6u);
  EXPECT_EQ(parsed.input.occupancy_grid.data[1], 100);
  EXPECT_EQ(parsed.input.occupancy_grid.data[2], -1);
  EXPECT_TRUE(parsed.has_config);
  EXPECT_DOUBLE_EQ(parsed.config.max_v, 0.8);
  EXPECT_DOUBLE_EQ(parsed.config.max_w, 1.2);
  EXPECT_DOUBLE_EQ(parsed.config.max_acc, 0.6);
  EXPECT_DOUBLE_EQ(parsed.config.max_angular_acc, 1.2);
  EXPECT_DOUBLE_EQ(parsed.config.path_resample_spacing, 0.08);
  EXPECT_TRUE(parsed.config.enable_path_tracking_guard);
  EXPECT_DOUBLE_EQ(parsed.config.path_tracking_lookahead_m, 0.72);
  EXPECT_DOUBLE_EQ(parsed.config.path_tracking_min_v, 0.16);
  ASSERT_EQ(parsed.input.dynamic_obstacles.size(), 1u);
  EXPECT_EQ(parsed.input.dynamic_obstacles[0].id, 3);
  EXPECT_NEAR(parsed.input.dynamic_obstacles[0].radius, 0.2, 1.0e-9);
}

TEST(WorkerRequestTest, RejectsBadMagic) {
  const auto parsed = ParseWorkerRequestText("NOT_A_REQUEST\n");

  EXPECT_FALSE(parsed.ok);
  EXPECT_NE(parsed.reason.find("magic"), std::string::npos);
}

TEST(WorkerRequestTest, RejectsPathCountMismatch) {
  std::string serialized = SerializeWorkerRequest(MakeInput(), ros::Time(30.1));
  const std::string old_token = "path_count 3";
  const auto pos = serialized.find(old_token);
  ASSERT_NE(pos, std::string::npos);
  serialized.replace(pos, old_token.size(), "path_count 4");

  const auto parsed = ParseWorkerRequestText(serialized);

  EXPECT_FALSE(parsed.ok);
  EXPECT_NE(parsed.reason.find("path_count"), std::string::npos);
}

}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
