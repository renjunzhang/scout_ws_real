#pragma once

#include "spmpc_local_planner/planning/planning_config.h"
#include <ros/node_handle.h>

namespace spmpc_local_planner {

// Loads the complete planning namespace. Missing keys retain the C++ defaults;
// malformed values and unknown enum strings throw std::invalid_argument.
PlanningConfig loadPlanningConfig(const ros::NodeHandle& pnh);

}  // namespace spmpc_local_planner
