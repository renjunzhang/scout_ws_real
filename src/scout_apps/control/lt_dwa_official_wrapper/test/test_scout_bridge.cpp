#include <gtest/gtest.h>

#include "lt_dwa_official_wrapper/scout_bridge.hpp"

namespace lt_dwa_official_wrapper {
namespace {

nav_msgs::Odometry MakeOdom() {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = ros::Time(10.0);
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 0.2;
  odom.twist.twist.angular.z = 0.1;
  return odom;
}

nav_msgs::OccupancyGrid MakeMap() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "odom";
  map.header.stamp = ros::Time(10.0);
  map.info.width = 5;
  map.info.height = 5;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(25, 0);
  return map;
}

geometry_msgs::PoseStamped MakePathPose(double x, double y) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::Path MakePath() {
  nav_msgs::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = ros::Time(10.0);
  path.poses = {MakePathPose(0.0, 0.0), MakePathPose(0.5, 0.0), MakePathPose(1.0, 0.0)};
  return path;
}

ScoutBridgeInputCache MakeValidCache() {
  ScoutBridgeInputCache cache;
  cache.has_odom = true;
  cache.has_map = true;
  cache.has_path = true;
  cache.odom = MakeOdom();
  cache.map = MakeMap();
  cache.path = MakePath();
  return cache;
}

ScoutBridgeCommandState MakeCommandState(const ros::Time& stamp) {
  ScoutBridgeCommandState state;
  state.has_command = true;
  state.stamp = stamp;
  state.command_v = 0.3;
  state.command_w = -0.2;
  state.status = WrapperStatus::kOk;
  state.reason = "official_core_ok";
  state.worker_latency_ms = 12.0;
  return state;
}

TEST(ScoutBridgeTest, DefaultsAreShadowOnlyRealtimeBridge) {
  const auto config = DefaultScoutBridgeConfig();

  EXPECT_FALSE(config.enable_actuated_output);
  EXPECT_FALSE(config.publish_cmd_vel);
  EXPECT_FALSE(config.publish_benchmark_raw);
  EXPECT_FALSE(config.enable_worker_core);
  EXPECT_DOUBLE_EQ(config.planner_rate_hz, 5.0);
  EXPECT_DOUBLE_EQ(config.command_publish_rate_hz, 30.0);
  EXPECT_DOUBLE_EQ(config.command_stale_timeout_sec, 0.25);
  EXPECT_EQ(config.worker_tf_topic, "/baseline/official_lt_dwa/worker_tf_sandbox");
  EXPECT_EQ(config.worker_tf_static_topic, "/baseline/official_lt_dwa/worker_tf_static_sandbox");
  EXPECT_EQ(config.shadow_cmd_topic, "/baseline/official_lt_dwa/shadow_cmd_vel");
  EXPECT_EQ(config.path_topic, "/scout/global_path_fixed");
}

TEST(ScoutBridgeTest, BuildsPlannerInputFromScoutMessages) {
  const auto config = DefaultScoutBridgeConfig();
  const auto result = BuildPlannerInputForScoutBridge(config, MakeValidCache(), ros::Time(10.1));

  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.input.planning_frame, "odom");
  EXPECT_EQ(result.input.robot_pose.frame_id, "odom");
  EXPECT_DOUBLE_EQ(result.input.robot_twist.v, 0.2);
  EXPECT_DOUBLE_EQ(result.input.robot_twist.w, 0.1);
  EXPECT_EQ(result.input.reference_path.size(), 3u);
  EXPECT_DOUBLE_EQ(result.input.target_pose.x, 1.0);
  EXPECT_EQ(result.input.occupancy_grid.info.width, 5u);
}

TEST(ScoutBridgeTest, GoalOverridesTargetPositionButKeepsPathEndYawPolicy) {
  auto cache = MakeValidCache();
  cache.has_goal = true;
  cache.goal.header.frame_id = "odom";
  cache.goal.header.stamp = ros::Time(10.0);
  cache.goal.pose.position.x = 2.0;
  cache.goal.pose.position.y = 0.5;
  cache.goal.pose.orientation.z = 1.0;
  cache.goal.pose.orientation.w = 0.0;

  const auto result = BuildPlannerInputForScoutBridge(DefaultScoutBridgeConfig(), cache, ros::Time(10.1));

  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_DOUBLE_EQ(result.input.target_pose.x, 2.0);
  EXPECT_DOUBLE_EQ(result.input.target_pose.y, 0.5);
  EXPECT_DOUBLE_EQ(result.input.target_pose.yaw, 0.0);
}

TEST(ScoutBridgeTest, RejectsFrameMismatch) {
  auto cache = MakeValidCache();
  cache.map.header.frame_id = "map";

  const auto result = BuildPlannerInputForScoutBridge(DefaultScoutBridgeConfig(), cache, ros::Time(10.1));

  EXPECT_EQ(result.status, WrapperStatus::kInvalidFrame);
}

TEST(ScoutBridgeTest, ActuatingRouteParamsDoNotRejectPlannerInput) {
  auto config = DefaultScoutBridgeConfig();
  config.publish_cmd_vel = true;

  const auto result = BuildPlannerInputForScoutBridge(config, MakeValidCache(), ros::Time(10.1));

  EXPECT_TRUE(result.ok()) << result.reason;
  EXPECT_FALSE(ShouldPublishCmdVel(config));
}

TEST(ScoutBridgeTest, ExplicitActuatedGateEnablesRoutes) {
  auto config = DefaultScoutBridgeConfig();
  config.enable_actuated_output = true;
  config.publish_cmd_vel = true;
  config.publish_benchmark_raw = true;

  EXPECT_TRUE(ShouldPublishCmdVel(config));
  EXPECT_TRUE(ShouldPublishBenchmarkRaw(config));
}

TEST(ScoutBridgeTest, FreshCommandDecisionReturnsCommand) {
  const auto config = DefaultScoutBridgeConfig();
  const auto decision = DecideCommandPublication(config, MakeCommandState(ros::Time(10.0)), ros::Time(10.1));

  EXPECT_TRUE(decision.fresh);
  EXPECT_DOUBLE_EQ(decision.command_v, 0.3);
  EXPECT_DOUBLE_EQ(decision.command_w, -0.2);
  EXPECT_NEAR(decision.command_age_sec, 0.1, 1e-9);
  EXPECT_FALSE(decision.publish_cmd_vel);
}

TEST(ScoutBridgeTest, StaleCommandDecisionReturnsZero) {
  const auto config = DefaultScoutBridgeConfig();
  const auto decision = DecideCommandPublication(config, MakeCommandState(ros::Time(10.0)), ros::Time(10.4));

  EXPECT_FALSE(decision.fresh);
  EXPECT_EQ(decision.reason, "command_stale");
  EXPECT_DOUBLE_EQ(decision.command_v, 0.0);
  EXPECT_DOUBLE_EQ(decision.command_w, 0.0);
}

TEST(ScoutBridgeTest, NonOkCommandDecisionReturnsZero) {
  auto state = MakeCommandState(ros::Time(10.0));
  state.status = WrapperStatus::kCoreProcessExited;
  state.reason = "worker timed out";

  const auto decision = DecideCommandPublication(DefaultScoutBridgeConfig(), state, ros::Time(10.1));

  EXPECT_FALSE(decision.fresh);
  EXPECT_EQ(decision.reason, "worker timed out");
  EXPECT_DOUBLE_EQ(decision.command_v, 0.0);
  EXPECT_DOUBLE_EQ(decision.command_w, 0.0);
}

TEST(ScoutBridgeTest, FormatsDiagnosticsWithSafetyFlagsAndRates) {
  const auto text = FormatScoutBridgeDiagnostics(DefaultScoutBridgeConfig(),
                                                WrapperStatus::kCommandRejected,
                                                "worker_disabled",
                                                0.0,
                                                0.0,
                                                -1.0,
                                                false,
                                                -1.0);

  EXPECT_NE(text.find("planner_rate_hz=5"), std::string::npos);
  EXPECT_NE(text.find("command_publish_rate_hz=30"), std::string::npos);
  EXPECT_NE(text.find("enable_actuated_output=false"), std::string::npos);
  EXPECT_NE(text.find("effective_cmd_vel=false"), std::string::npos);
  EXPECT_NE(text.find("effective_benchmark_raw=false"), std::string::npos);
  EXPECT_NE(text.find("worker_tf_topic=/baseline/official_lt_dwa/worker_tf_sandbox"), std::string::npos);
  EXPECT_NE(text.find("expected_map_file="), std::string::npos);
}

}  // namespace
}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
