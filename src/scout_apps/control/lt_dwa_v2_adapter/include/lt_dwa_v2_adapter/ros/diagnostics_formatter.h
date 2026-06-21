#pragma once

#include <cstddef>
#include <string>

#include "lt_dwa_v2_adapter/core/trajectory_types.h"

namespace lt_dwa_v2_adapter
{
struct RosDiagnosticsContext
{
  std::string plan_frame;
  std::string map_frame;
  std::string raw_odom_frame;
  std::string raw_odom_child_frame;
  size_t path_size = 0;
  bool have_last_progress = false;
  double tracker_progress_s = 0.0;
  RobotState state;
  double raw_odom_x = 0.0;
  double raw_odom_y = 0.0;
  double raw_odom_yaw = 0.0;
};

std::string formatDiagnostics(const RosDiagnosticsContext& context, const PlanResult& result);
}  // namespace lt_dwa_v2_adapter
