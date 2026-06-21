#pragma once

#include <ros/ros.h>

#include "lt_dwa_v2_adapter/core/planner_config.h"

namespace lt_dwa_v2_adapter
{
PlannerConfig loadPlannerConfig(const ros::NodeHandle& private_nh);
}  // namespace lt_dwa_v2_adapter
