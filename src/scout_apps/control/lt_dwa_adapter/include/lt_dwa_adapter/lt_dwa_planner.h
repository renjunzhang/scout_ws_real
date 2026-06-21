#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

#include "lt_dwa_adapter/occupancy_adapter.h"
#include "lt_dwa_adapter/path_reference.h"
#include "lt_dwa_adapter/trajectory_types.h"

namespace lt_dwa_adapter
{
class LtDwaPlanner
{
public:
  void configure(const PlannerConfig& config);

  PlanResult plan(const RobotState& current,
                  const std::vector<Pose2D>& path,
                  const OccupancyAdapter* occupancy,
                  double min_progress_s = 0.0,
                  double max_progress_s = -1.0) const;

private:
  PlannerConfig config_;

  std::vector<double> sampleLinearVelocities(double current_v) const;
  std::vector<double> sampleAngularVelocities(double current_w) const;
  std::vector<Command> sampleCommands(const RobotState& state) const;
  Command pathTrackingSeed(const RobotState& state, const PathReference& path, double previous_progress_s) const;
  RobotState rolloutStep(const RobotState& state, const Command& command) const;

  bool isGoalReached(const RobotState& state, const PathReference& path) const;
  bool collisionAt(const RobotState& state, const OccupancyAdapter* occupancy,
                   CollisionDiagnostics* diagnostics = nullptr) const;
  double obstacleCost(const RobotState& state, const OccupancyAdapter* occupancy) const;
  double scorePoint(const RobotState& state,
                    const Command& command,
                    const Command& previous_command,
                    double previous_progress_s,
                    const PathReference& path,
                    const OccupancyAdapter* occupancy,
                    ScoreBreakdown& score) const;
  void accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc) const;
  double weightedTotal(const ScoreBreakdown& score) const;
};

std::string formatStatus(const PlanResult& result);
}  // namespace lt_dwa_adapter
