#pragma once

#include <map>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "lt_dwa_official_wrapper/types.hpp"

#include "util/common.hpp"
#include "util/grid_map.hpp"
#include "util/tools.hpp"

namespace lt_dwa_official_wrapper {

Pose ToOfficialPose(const Pose2d& pose);
Action ToOfficialAction(const Twist2d& twist);
std::vector<PathPose> ToOfficialPath(const std::vector<Pose2d>& path,
                                     double resample_spacing_m);
GridMap ToOfficialGridMap(const nav_msgs::OccupancyGrid& map);
std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> ToOfficialObstacleHistory(
    const std::vector<ObstacleTrack>& obstacles,
    std::size_t fill_count = OBSTACLE_INFO_LEN);

double ComputePathLength(const std::vector<Pose2d>& path);

}  // namespace lt_dwa_official_wrapper
