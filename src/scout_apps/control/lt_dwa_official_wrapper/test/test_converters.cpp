#include <gtest/gtest.h>

#include <cmath>

#include "lt_dwa_official_wrapper/converters.hpp"

namespace lt_dwa_official_wrapper {
namespace {

Pose2d MakePose(double x, double y) {
  Pose2d pose;
  pose.frame_id = "odom";
  pose.x = x;
  pose.y = y;
  pose.yaw = 0.0;
  return pose;
}

nav_msgs::OccupancyGrid MakeMap() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "odom";
  map.info.width = 3;
  map.info.height = 1;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 100, -1};
  return map;
}

TEST(ConvertersTest, ResamplesPathWithCumulativeDistanceAndHeading) {
  std::vector<Pose2d> path = {MakePose(0.0, 0.0), MakePose(0.25, 0.0)};

  const auto official_path = ToOfficialPath(path, 0.10);

  ASSERT_EQ(official_path.size(), 4u);
  EXPECT_NEAR(official_path[0].x_, 0.0, 1.0e-9);
  EXPECT_NEAR(official_path[1].x_, 0.1, 1.0e-9);
  EXPECT_NEAR(official_path[2].x_, 0.2, 1.0e-9);
  EXPECT_NEAR(official_path[3].x_, 0.25, 1.0e-9);
  EXPECT_NEAR(official_path[3].dist_, 0.25, 1.0e-9);
  for (const auto& point : official_path) {
    EXPECT_NEAR(point.y_, 0.0, 1.0e-9);
    EXPECT_NEAR(point.theta_, 0.0, 1.0e-9);
  }
}

TEST(ConvertersTest, ConvertsUnknownOccupancyAsOccupied) {
  const GridMap official_map = ToOfficialGridMap(MakeMap());

  EXPECT_EQ(official_map.getWidth(), 3);
  EXPECT_EQ(official_map.getHeight(), 1);
  EXPECT_FALSE(official_map.isOccupied(0));
  EXPECT_TRUE(official_map.isOccupied(1));
  EXPECT_TRUE(official_map.isOccupied(2));
}

TEST(ConvertersTest, FillsObstacleHistoryWithZeroOrderHold) {
  ObstacleTrack obstacle;
  obstacle.id = 7;
  obstacle.frame_id = "odom";
  obstacle.x = 1.0;
  obstacle.y = 2.0;
  obstacle.vx = 0.3;
  obstacle.vy = -0.1;
  obstacle.radius = 0.25;

  const auto history = ToOfficialObstacleHistory({obstacle});

  ASSERT_EQ(history.size(), 1u);
  const auto it = history.find(7);
  ASSERT_NE(it, history.end());
  EXPECT_EQ(it->second.size(), static_cast<std::size_t>(OBSTACLE_INFO_LEN));
  EXPECT_NEAR(it->second.back().x_, 1.0, 1.0e-9);
  EXPECT_NEAR(it->second.back().y_, 2.0, 1.0e-9);
  EXPECT_NEAR(it->second.back().vx_, 0.3, 1.0e-9);
  EXPECT_NEAR(it->second.back().vy_, -0.1, 1.0e-9);
  EXPECT_NEAR(it->second.back().radius_, 0.25, 1.0e-9);
}

}  // namespace
}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
