#pragma once

#include "spmpc_local_planner/ControlCycleAudit.h"
#include "spmpc_local_planner/controller/control_cycle_telemetry.h"

#include <std_msgs/Float32MultiArray.h>

#include <string>

namespace spmpc_local_planner {

// Stateless ROS encoding boundary.  These functions preserve existing topic
// schemas while keeping message construction out of controller/runtime code.
std_msgs::Float32MultiArray encodeCommandIntervention(
    const CommandInterventionDebug& intervention);

ControlCycleAudit encodeControlCycleAudit(
    const ControlCycleAuditDebug& audit,
    const std::string& frame_id);

}  // namespace spmpc_local_planner
