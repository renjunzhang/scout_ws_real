#pragma once

#include <string>

namespace lt_dwa_v2_adapter
{
enum class PlannerStatusCode
{
  Idle,
  WaitingForOdom,
  WaitingForPath,
  WaitingForMap,
  MapTfError,
  RobotInCollision,
  NoValidCommand,
  Tracking,
  TrackingDiverged,
  GoalReached
};

inline const char* statusName(PlannerStatusCode code)
{
  switch (code)
  {
    case PlannerStatusCode::Idle:
      return "IDLE";
    case PlannerStatusCode::WaitingForOdom:
      return "WAITING_FOR_ODOM";
    case PlannerStatusCode::WaitingForPath:
      return "WAITING_FOR_PATH";
    case PlannerStatusCode::WaitingForMap:
      return "WAITING_FOR_MAP";
    case PlannerStatusCode::MapTfError:
      return "MAP_TF_ERROR";
    case PlannerStatusCode::RobotInCollision:
      return "ROBOT_IN_COLLISION";
    case PlannerStatusCode::NoValidCommand:
      return "NO_VALID_CMD";
    case PlannerStatusCode::Tracking:
      return "TRACKING";
    case PlannerStatusCode::TrackingDiverged:
      return "TRACKING_DIVERGED";
    case PlannerStatusCode::GoalReached:
      return "GOAL_REACHED";
  }
  return "UNKNOWN";
}

inline std::string statusString(PlannerStatusCode code)
{
  return statusName(code);
}
}  // namespace lt_dwa_v2_adapter
