#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

#include "lt_dwa_adapter/trajectory_types.h"

namespace lt_dwa_adapter
{
class LtDwaPlanner
{
public:
  void configure(const PlannerConfig& config);

  PlanResult plan(const RobotState& current,
                  const std::vector<Pose2D>& path,
                  const nav_msgs::OccupancyGrid* occupancy) const;

private:
  struct PathMatch
  {
    double distance = 0.0;
    double heading_error = 0.0;
    double index = 0.0;
    Pose2D pose;
  };

  PlannerConfig config_;

  std::vector<double> sampleLinearVelocities(double current_v) const;
  std::vector<double> sampleAngularVelocities(double current_w) const;
  std::vector<Command> sampleCommands(const RobotState& state) const;
  Command pathTrackingSeed(const RobotState& state, const std::vector<Pose2D>& path) const;
  RobotState rolloutStep(const RobotState& state, const Command& command) const;

  PathMatch matchPath(const RobotState& state, const std::vector<Pose2D>& path) const;
  bool isGoalReached(const RobotState& state, const std::vector<Pose2D>& path) const;
  bool collisionAt(const RobotState& state, const nav_msgs::OccupancyGrid* occupancy) const;
  double obstacleCost(const RobotState& state, const nav_msgs::OccupancyGrid* occupancy) const;
  double scorePoint(const RobotState& state,
                    const Command& command,
                    const Command& previous_command,
                    double previous_progress_index,
                    const std::vector<Pose2D>& path,
                    const nav_msgs::OccupancyGrid* occupancy,
                    ScoreBreakdown& score) const;
  void accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc) const;
  double weightedTotal(const ScoreBreakdown& score) const;
};

std::string formatStatus(const PlanResult& result);
}  // namespace lt_dwa_adapter
