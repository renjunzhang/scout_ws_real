#pragma once

#include <string>

#include "lt_dwa_v2_adapter/core/trajectory_types.h"

namespace lt_dwa_v2_adapter
{
struct PlanningTransform2D
{
  bool valid = true;
  std::string source_frame;
  std::string target_frame;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

PlanningTransform2D identityTransform(const std::string& source_frame, const std::string& target_frame);
Pose2D transformPose(const Pose2D& pose, const PlanningTransform2D& transform);
RobotState transformState(const RobotState& state, const PlanningTransform2D& transform);
}  // namespace lt_dwa_v2_adapter
